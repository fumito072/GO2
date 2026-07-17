"""Cockpit の自律 mission が実機へ出す command の安全回帰試験。

hardware / Claude CLI / background thread を起動せず、探索分類、LiDAR/odom
freshness、frame gate、keeper の最終停止判定を固定する。
"""
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from common import config
from cockpit.mission import (
    AUTONOMY_SENSOR_MAX_AGE_S,
    MissionAgent,
    autonomy_robot_state_error,
    autonomy_sensor_error,
    classify_exploration_request,
)
from cockpit.server import RobotBridge
from navigation.exploration_controller import ControlStatus


NOW = 1_000.0


class FakeBridge:
    """Mission safety helper が読む最小の RobotBridge 契約。"""

    def __init__(self, *, now=NOW, mock=False):
        self.mock = mock
        self.armed = True
        self.pose = (1.0, 2.0, 0.31, 0.0)
        self.pose_src = "mock" if mock else "lidar_odom"
        self.pose_ts = now - 0.10
        self.cloud_frame = "odom/mock-room" if mock else "odom"
        self.cloud_ts = now - 0.10
        self.cloud_scan_valid = True
        # floor evidence として十分な有限点。keeper の guardian 自体は各testでstub化する。
        self.cloud_pts = np.asarray(
            [[0.10 + 0.02 * i, -0.20 + 0.02 * (i % 10), 0.0]
             for i in range(24)],
            dtype=np.float32,
        )
        self.stair = {"kind": "none"}
        self.commands = []
        self.robot_state = {
            "low_age": 0.01,
            "rpy": [0.0, 0.0, 0.0],
        }
        self.bot = SimpleNamespace(state=lambda: dict(self.robot_state))

    def set_cmd(self, vx, vy, wz):
        self.commands.append((float(vx), float(vy), float(wz)))


class ExplorationClassificationTest(unittest.TestCase):
    def test_only_confirmed_exploration_language_selects_explorer(self):
        cases = {
            "部屋を探索してマップを作って": "current_room",
            "地図を作って": "current_room",
            "家中を探索して地図を作って": "all_reachable",
        }
        for text, target in cases.items():
            with self.subTest(text=text):
                self.assertEqual(classify_exploration_request(text), target)

    def test_questions_quotes_other_skills_and_empty_input_do_not_select_explorer(self):
        cases = (
            None,
            "",
            "探索してもいいですか",
            "「部屋を探索して」と言った",
            "今すぐ止まれ",
            "階段を登って",
            "ホームに戻って",
        )
        for text in cases:
            with self.subTest(text=text):
                self.assertIsNone(classify_exploration_request(text))

    def test_negated_exploration_is_fail_closed(self):
        # docstring 契約: 否定文を近似解釈して探索開始してはならない。
        for text in ("探索しないで", "部屋を探索しないで", "地図を作らないで"):
            with self.subTest(text=text):
                self.assertIsNone(classify_exploration_request(text))


class AutonomySensorContractTest(unittest.TestCase):
    def test_fresh_real_sensor_contract_passes(self):
        self.assertIsNone(autonomy_sensor_error(FakeBridge(), now_s=NOW))

    def test_mock_still_requires_fresh_pose_and_cloud_but_not_real_frame_names(self):
        bridge = FakeBridge(mock=True)
        self.assertIsNone(autonomy_sensor_error(bridge, now_s=NOW))
        bridge.cloud_ts = NOW - AUTONOMY_SENSOR_MAX_AGE_S - 0.01
        self.assertIn("stale", autonomy_sensor_error(bridge, now_s=NOW))

    def test_each_missing_or_stale_real_sensor_fact_fails_closed(self):
        cases = (
            ("missing pose", {"pose": None}, "pose"),
            ("nonfinite pose", {"pose": (0.0, 0.0, np.nan, 0.0)}, "pose"),
            ("sport pose", {"pose_src": "sms"}, "pose source"),
            ("missing pose timestamp", {"pose_ts": 0.0}, "stale"),
            ("stale pose", {
                "pose_ts": NOW - AUTONOMY_SENSOR_MAX_AGE_S - 0.01,
            }, "stale"),
            ("future pose", {"pose_ts": NOW + 0.01}, "stale"),
            ("invalid scan", {"cloud_scan_valid": False}, "invalid"),
            ("missing cloud timestamp", {"cloud_ts": 0.0}, "stale"),
            ("stale cloud", {
                "cloud_ts": NOW - AUTONOMY_SENSOR_MAX_AGE_S - 0.01,
            }, "stale"),
            ("future cloud", {"cloud_ts": NOW + 0.01}, "stale"),
            ("missing points", {"cloud_pts": None}, "不足"),
            ("too few points", {
                "cloud_pts": np.zeros((19, 3), dtype=np.float32),
            }, "不足"),
            ("too few finite points", {
                "cloud_pts": np.vstack((
                    np.zeros((19, 3), dtype=np.float32),
                    np.asarray([[np.nan, 0.0, 0.0]], dtype=np.float32),
                )),
            }, "finite"),
        )
        for label, changes, expected in cases:
            with self.subTest(case=label):
                bridge = FakeBridge()
                for name, value in changes.items():
                    setattr(bridge, name, value)
                error = autonomy_sensor_error(bridge, now_s=NOW)
                self.assertIsNotNone(error)
                self.assertIn(expected, error)

        self.assertIn("時刻", autonomy_sensor_error(FakeBridge(), now_s=np.nan))

    def test_spoofed_or_untransformed_cloud_frames_fail_closed(self):
        bad_frames = (
            "", "map", "utlidar_lidar", "not_odom",
            "fake/odom", "map/odom", "odom/lidar", "odom_child",
        )
        for frame in bad_frames:
            with self.subTest(frame=frame or "missing"):
                bridge = FakeBridge()
                bridge.cloud_frame = frame
                self.assertIn("frame", autonomy_sensor_error(bridge, now_s=NOW))


class AutonomyRobotStateContractTest(unittest.TestCase):
    def test_fresh_level_real_state_passes(self):
        self.assertIsNone(autonomy_robot_state_error(FakeBridge()))

    def test_real_lowstate_and_attitude_fail_closed(self):
        cases = (
            ({"low_age": 0.51}, "lowstate"),
            ({"low_age": float("nan")}, "lowstate"),
            ({"rpy": None}, "roll/pitch"),
            ({"rpy": [0.51, 0.0, 0.0]}, "roll"),
            ({"rpy": [0.0, -0.71, 0.0]}, "pitch"),
            ({"rpy": [float("nan"), 0.0, 0.0]}, "不正"),
        )
        for changes, expected in cases:
            with self.subTest(changes=changes):
                bridge = FakeBridge()
                bridge.robot_state.update(changes)
                self.assertIn(expected, autonomy_robot_state_error(bridge))

    def test_robot_state_exception_fails_closed(self):
        bridge = FakeBridge()
        bridge.bot = SimpleNamespace(
            state=Mock(side_effect=RuntimeError("synthetic state failure")))
        self.assertIn("取得失敗", autonomy_robot_state_error(bridge))


class MissionStartContractTest(unittest.TestCase):
    def _accepted_exploration(self):
        from tests.test_mission_executive import CTX, make_exec, make_spec
        spec = make_spec()
        executive = make_exec()
        self.assertTrue(executive.accept_goal(spec, CTX, 2_000).accepted)
        return spec, executive

    def test_direct_language_cannot_bypass_exploration_confirmation(self):
        agent = MissionAgent(FakeBridge())

        error = agent.start("部屋を探索してマップを作って")

        self.assertIn("確認", error)
        self.assertFalse(agent._run_flag)

    def test_only_fsm_accepted_goal_reaches_safe_runner(self):
        agent = MissionAgent(FakeBridge())
        spec, executive = self._accepted_exploration()
        agent._launch_locked = Mock(return_value=None)

        with patch("cockpit.mission.time.monotonic", return_value=NOW):
            error = agent.start_goal(spec, executive)

        self.assertIsNone(error)
        agent._launch_locked.assert_called_once_with(
            spec.transcript.text, "explore", spec.target.ref, spec)

    def test_missing_fsm_or_stale_scan_is_rejected(self):
        spec, executive = self._accepted_exploration()
        agent = MissionAgent(FakeBridge())
        self.assertIn("FSM", agent.start_goal(spec, None))
        agent.bridge.cloud_ts = NOW - AUTONOMY_SENSOR_MAX_AGE_S - 0.01
        with patch("cockpit.mission.time.monotonic", return_value=NOW):
            error = agent.start_goal(spec, executive)
        self.assertIn("stale", error)


class MissionRunGenerationTest(unittest.TestCase):
    def test_abort_during_planner_step_cannot_be_overwritten_by_old_run(self):
        entered = threading.Event()
        release = threading.Event()

        class BlockingController:
            def __init__(self, gmap, *_args, **_kwargs):
                self.gmap = gmap

            def integrate_point_cloud(self, *_args, **_kwargs):
                return 1

            def step(self, *_args, **_kwargs):
                entered.set()
                release.wait(2.0)
                return SimpleNamespace(
                    status=ControlStatus.COMPLETE,
                    vx=0.0, vy=0.0, wz=0.0,
                    reason="synthetic complete", goal=None,
                    map_revision=self.gmap.revision,
                )

            def metrics(self):
                return {}

        now = time.monotonic()
        bridge = FakeBridge(now=now, mock=True)
        agent = MissionAgent(bridge)
        agent._run_flag = True
        agent._run_id = 9
        agent.status = "running"
        agent.mode = "explore"
        agent.exploration_target = "current_room"
        agent.t0 = now

        with patch("cockpit.mission.ExplorationController", BlockingController), \
                patch("cockpit.mission.deploy_log"):
            worker = threading.Thread(target=agent._run_exploration, args=(9,))
            worker.start()
            self.assertTrue(entered.wait(2.0))
            agent.abort("synthetic STOP", expected_run_id=9)
            release.set()
            worker.join(2.0)

        self.assertFalse(worker.is_alive())
        self.assertFalse(agent._run_flag)
        self.assertEqual(agent.status, "aborted")
        self.assertEqual(agent.detail, "中断: synthetic STOP")
        self.assertEqual(agent._cur, (0.0, 0.0, 0.0))
        self.assertTrue(bridge.commands)
        self.assertEqual(bridge.commands[-1], (0.0, 0.0, 0.0))


class ServerFrameGateTest(unittest.TestCase):
    def test_only_odom_frame_contract_is_accepted(self):
        self.assertTrue(RobotBridge._is_world_cloud_frame("odom"))
        for frame in (
            None, "", "map", "utlidar_lidar", "not_odom",
            "fake/odom", "map/odom", "odom/lidar", "odom_child",
        ):
            with self.subTest(frame=frame):
                self.assertFalse(RobotBridge._is_world_cloud_frame(frame))

    def test_odom_callback_records_monotonic_freshness(self):
        bridge = RobotBridge.__new__(RobotBridge)
        bridge._pose_prev = None
        bridge.vel_world = [0.0, 0.0, 0.0]
        msg = SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=1.0, y=2.0, z=0.31),
            orientation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0),
        )))
        with patch("cockpit.server.time.monotonic", return_value=NOW):
            bridge._on_odom(msg)
        self.assertEqual(bridge.pose_src, "lidar_odom")
        self.assertEqual(bridge.pose_ts, NOW)
        self.assertEqual(bridge.pose, (1.0, 2.0, 0.31, 0.0))


class ServerCommandValidationTest(unittest.TestCase):
    @staticmethod
    def make_bridge():
        bridge = RobotBridge.__new__(RobotBridge)
        bridge._lock = threading.Lock()
        bridge.cmd = [0.0, 0.0, 0.0]
        bridge.cmd_ts = 0.0
        bridge.armed = False
        bridge.bot = SimpleNamespace(stop_move=Mock())
        return bridge

    def test_nonfinite_or_unparseable_command_becomes_immediate_zero(self):
        for value in (float("nan"), float("inf"), "not-a-number"):
            with self.subTest(value=value):
                bridge = self.make_bridge()
                with patch("cockpit.server.deploy_log"):
                    accepted = bridge.set_cmd(value, 0.0, 0.0)
                self.assertFalse(accepted)
                self.assertEqual(bridge.cmd, [0.0, 0.0, 0.0])
                bridge.bot.stop_move.assert_called_once_with()

    def test_finite_command_is_clamped_and_invalid_arm_cannot_arm(self):
        bridge = self.make_bridge()
        with patch("cockpit.server.deploy_log"):
            self.assertTrue(bridge.set_cmd(99.0, -99.0, 99.0))
        self.assertEqual(bridge.cmd, [
            config.VEL_LIMIT["vx"][1],
            config.VEL_LIMIT["vy"][0],
            config.VEL_LIMIT["wz"][1],
        ])
        with patch("cockpit.server.deploy_log"):
            self.assertFalse(bridge.set_armed("true"))
        self.assertFalse(bridge.armed)
        self.assertEqual(bridge.cmd, [0.0, 0.0, 0.0])


class _SafeGuard:
    def assess(self, *args, **kwargs):
        return SimpleNamespace(safe=True, reason="corridor clear", clearance_m=None)


class _UnsafeGuard:
    def assess(self, *args, **kwargs):
        return SimpleNamespace(
            safe=False,
            reason="obstacle inside stopping corridor",
            clearance_m=0.20,
        )


class _ExplodingGuard:
    def assess(self, *args, **kwargs):
        raise RuntimeError("synthetic guardian failure")


class MissionKeeperSafetyTest(unittest.TestCase):
    @staticmethod
    def make_agent(bridge, guard, *, command=(0.2, 0.0, 0.0),
                   hold_until=NOW + 1.0):
        # __init__ は CLI probe を行うため、keeper の純粋な契約だけを構成する。
        agent = MissionAgent.__new__(MissionAgent)
        agent.bridge = bridge
        agent.guard = guard
        agent._cur = command
        agent._hold_until = hold_until
        agent._run_flag = True
        agent.safety = {"safe": True, "reason": "fixture"}
        agent._last_safety_log = None
        agent.mode = "vlm"
        agent.detail = ""
        return agent

    @staticmethod
    def run_one_keeper_tick(agent, run_id=None):
        def stop_after_tick(_seconds):
            agent._run_flag = False

        with patch("cockpit.mission.time.monotonic", return_value=NOW), \
                patch("cockpit.mission.time.sleep", side_effect=stop_after_tick), \
                patch("cockpit.mission.deploy_log"):
            agent._keeper(run_id=run_id)

    def test_safe_assessed_command_is_forwarded(self):
        bridge = FakeBridge()
        agent = self.make_agent(bridge, _SafeGuard())
        self.run_one_keeper_tick(agent)
        self.assertEqual(bridge.commands, [(0.2, 0.0, 0.0)])
        self.assertTrue(agent.safety["safe"])

    def test_stale_sensor_forces_zero_without_calling_guard(self):
        bridge = FakeBridge()
        bridge.cloud_ts = NOW - AUTONOMY_SENSOR_MAX_AGE_S - 0.01
        guard = Mock()
        agent = self.make_agent(bridge, guard)
        self.run_one_keeper_tick(agent)
        self.assertEqual(bridge.commands, [(0.0, 0.0, 0.0)])
        self.assertEqual(agent._cur, (0.0, 0.0, 0.0))
        self.assertFalse(agent.safety["safe"])
        guard.assess.assert_not_called()

    def test_stale_lowstate_forces_zero_without_calling_guard(self):
        bridge = FakeBridge()
        bridge.robot_state["low_age"] = 0.51
        guard = Mock()
        agent = self.make_agent(bridge, guard)
        self.run_one_keeper_tick(agent)
        self.assertEqual(bridge.commands, [(0.0, 0.0, 0.0)])
        self.assertFalse(agent.safety["safe"])
        self.assertIn("lowstate", agent.safety["reason"])
        guard.assess.assert_not_called()

    def test_stale_lowstate_invalidates_live_run_without_auto_resume(self):
        bridge = FakeBridge()
        bridge.robot_state["low_age"] = 0.51
        guard = Mock()
        agent = self.make_agent(bridge, guard)
        agent._run_id = 7
        agent._lifecycle_lock = threading.RLock()

        self.run_one_keeper_tick(agent, run_id=7)

        self.assertFalse(agent._run_flag)
        self.assertEqual(agent.status, "aborted")
        self.assertIn("機体状態異常", agent.detail)
        self.assertEqual(bridge.commands, [(0.0, 0.0, 0.0)])
        guard.assess.assert_not_called()

    def test_guard_rejection_forces_zero(self):
        bridge = FakeBridge()
        agent = self.make_agent(bridge, _UnsafeGuard())
        self.run_one_keeper_tick(agent)
        self.assertEqual(bridge.commands, [(0.0, 0.0, 0.0)])
        self.assertEqual(agent._hold_until, 0.0)
        self.assertFalse(agent.safety["safe"])
        self.assertIn("obstacle", agent.detail)

    def test_guard_exception_fails_toward_stop(self):
        bridge = FakeBridge()
        agent = self.make_agent(bridge, _ExplodingGuard())
        self.run_one_keeper_tick(agent)
        self.assertEqual(bridge.commands, [(0.0, 0.0, 0.0)])
        self.assertFalse(agent.safety["safe"])
        self.assertIn("guardian error", agent.safety["reason"])

    def test_expired_hold_explicitly_clears_previous_motion(self):
        # refresh停止だけでは RobotBridge watchdog の0.5秒間、旧commandが残る。
        bridge = FakeBridge()
        agent = self.make_agent(bridge, _SafeGuard(), hold_until=NOW - 0.01)
        self.run_one_keeper_tick(agent)
        self.assertEqual(bridge.commands, [(0.0, 0.0, 0.0)])
        self.assertEqual(agent._cur, (0.0, 0.0, 0.0))

    def test_keeper_never_forwards_a_command_other_than_the_assessed_snapshot(self):
        bridge = FakeBridge()
        agent = self.make_agent(bridge, None)
        assessed = []

        class MutatingGuard:
            def assess(self, _pose, _points, command, **_kwargs):
                assessed.append(tuple(command))
                # controller/VLM threadがguardian評価中に次commandを書いた競合を再現。
                agent._cur = (0.0, 0.2, 0.0)
                return SimpleNamespace(
                    safe=True, reason="old command clear", clearance_m=None)

        agent.guard = MutatingGuard()
        self.run_one_keeper_tick(agent)
        self.assertEqual(len(assessed), 1)
        self.assertEqual(bridge.commands[-1], assessed[0])

    def test_old_keeper_cannot_resume_when_a_new_run_is_active(self):
        bridge = FakeBridge()
        agent = self.make_agent(bridge, _SafeGuard())
        agent._run_id = 2
        with patch("cockpit.mission.time.sleep") as sleep:
            agent._keeper(run_id=1)
        self.assertEqual(bridge.commands, [])
        sleep.assert_not_called()


if __name__ == "__main__":
    unittest.main()
