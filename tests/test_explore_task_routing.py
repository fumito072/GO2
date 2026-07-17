"""cockpit.explore_task のルーティング/確認フローの unit test (E0)。

実行ループ(実機/mock接続)は E2 の mock server E2E で検証する。
ここではスレッドを起こさない範囲(route_text / confirm ガード / snapshot /
map_frame)を stub bridge で検証する。
"""
import struct
import threading
import time
import unittest

import numpy as np

from cockpit.explore_task import BIN_EXPMAP, ExploreTask
from perception.global_map import GlobalOccupancyMap


class StubBot:
    def __init__(self):
        self.stopped = 0

    def state(self):
        return {"low_age": 0.01, "rpy": [0.0, 0.0, 0.0],
                "vel": [0.0, 0.0, 0.0]}

    def stop_move(self):
        self.stopped += 1


class StubBridge:
    def __init__(self):
        self.mock = True
        self.armed = False
        self.pose = (0.0, 0.0, 0.31, 0.0)
        self.pose_ts = None
        self.pose_src = "mock"
        self.cloud_pts = None
        self.cloud_ts = 0.0
        self.cloud_rx_ts = 0.0
        self.cloud_frame = "odom"
        self.cloud_scan_valid = False
        self.bot = StubBot()
        self.cmds = []

    def set_cmd(self, vx, vy, wz):
        self.cmds.append((vx, vy, wz))


def _task():
    return ExploreTask(StubBridge(), artifacts_dir="/tmp/test_explore_maps")


class StubMission:
    def __init__(self, error=None):
        self.error = error
        self.status = "idle"
        self.detail = ""
        self._run_flag = False
        self.goal_spec = None
        self.gmap = None
        self.controller = None
        self.last = {}
        self.map_lock = threading.RLock()
        self.started = []
        self.aborted = []

    def start_goal(self, spec, executive):
        self.started.append((spec, executive))
        if self.error:
            return self.error
        self.goal_spec = spec
        self.status = "running"
        self.detail = "安全探索中"
        self._run_flag = True
        return None

    def abort(self, why):
        self.aborted.append(why)
        self._run_flag = False
        self.status = "aborted"
        self.detail = why


def _ready_delegated_task(error=None):
    bridge = StubBridge()
    bridge.armed = True
    now = time.monotonic()
    bridge.pose_ts = now
    bridge.cloud_ts = now
    bridge.cloud_rx_ts = now
    bridge.cloud_scan_valid = True
    bridge.cloud_pts = np.zeros((24, 3), dtype=np.float32)
    mission = StubMission(error)
    return (ExploreTask(bridge, mission=mission,
                        artifacts_dir="/tmp/test_explore_maps"), mission)


class TestRouting(unittest.TestCase):
    def test_stop_now_is_immediate_and_arm_independent(self):
        t = _task()
        self.assertFalse(t.bridge.armed)   # DISARM でも停止は通る
        r = t.route_text("止まれ")
        self.assertEqual(r["kind"], "stop_now")
        self.assertIn((0, 0, 0), t.bridge.cmds)
        self.assertGreaterEqual(t.bridge.bot.stopped, 1)

    def test_explore_utterance_becomes_proposal(self):
        t = _task()
        r = t.route_text("部屋を探索してマップを作って")
        self.assertEqual(r["kind"], "proposal")
        self.assertEqual(t.status, "proposal")
        self.assertIn("確認", r["say"])

    def test_confirm_word_accepted_but_disarm_refuses(self):
        t = _task()
        t.route_text("部屋を探索してマップを作って")
        r = t.route_text("はい")           # 確認語 → confirm() 経由
        self.assertEqual(r["kind"], "error")
        self.assertIn("ARM", r["say"])     # DISARM 中は開始拒否
        self.assertFalse(t._run)

    def test_cancel_word_cancels_proposal(self):
        t = _task()
        t.route_text("探索して")
        r = t.route_text("キャンセル")
        self.assertEqual(r["kind"], "cancelled")
        self.assertEqual(t.status, "idle")

    def test_non_command_falls_through(self):
        t = _task()
        r = t.route_text("今日はいい天気ですね")
        self.assertEqual(r["kind"], "non_command")
        self.assertFalse(r["handled"])

    def test_question_not_executed(self):
        t = _task()
        r = t.route_text("探索できますか?")
        self.assertFalse(r["kind"] == "proposal")

    def test_stair_intents_rejected(self):
        t = _task()
        r = t.route_text("階段を登って")
        # 階段系は探索統合の対象外(段差パネルへ誘導)
        self.assertIn(r["kind"], ("rejected", "clarification"))
        self.assertFalse(t._run)

    def test_waypoint_without_map_rejected(self):
        t = _task()
        r = t.route_text("ホームに戻って")
        if r["kind"] == "proposal":
            self.fail("home未登録なのに提案が通った")
        self.assertEqual(r["kind"], "rejected")

    def test_confirm_without_proposal(self):
        t = _task()
        r = t.confirm()
        self.assertEqual(r["kind"], "error")

    def test_voice_evidence_accepted(self):
        t = _task()
        r = t.route_text("探索して", modality="voice",
                         evidence={"quality": 0.9, "no_speech": 0.05})
        self.assertEqual(r["kind"], "proposal")

    def test_ui_modality_accepted(self):
        # 🤖 自律モードボタン(explore_auto)の経路
        t = _task()
        r = t.route_text("部屋を探索してマップを作って", modality="ui")
        self.assertEqual(r["kind"], "proposal")


class TestMapFrame(unittest.TestCase):
    def test_no_map_returns_none(self):
        self.assertIsNone(_task().map_frame())

    def test_frame_layout_and_downsample(self):
        t = _task()
        t.gmap = GlobalOccupancyMap(size_m=(2.0, 2.0), resolution_m=0.05,
                                    origin_xy=(-1.0, -1.0), map_id="t")
        t.gmap.mark_hazard([(0.0, 0.0)], now_ns=1)
        buf = t.map_frame()
        head = struct.calcsize("<BfffHH")   # 17 bytes
        kind, ox, oy, res, w, h = struct.unpack("<BfffHH", buf[:head])
        self.assertEqual(kind, BIN_EXPMAP)
        self.assertAlmostEqual(res, 0.1, places=5)
        self.assertEqual((w, h), (20, 20))
        self.assertEqual(len(buf), head + w * h)
        cells = np.frombuffer(buf[head:], dtype=np.uint8).reshape(h, w)
        self.assertEqual(cells.max(), 2)   # max-pool で OCCUPIED が残る


class TestSnapshot(unittest.TestCase):
    def test_snapshot_keys(self):
        s = _task().snapshot()
        for k in ("status", "detail", "say", "pending", "goal", "counts",
                  "waypoints", "elapsed", "trace", "events"):
            self.assertIn(k, s)


class TestStairIntegration(unittest.TestCase):
    """自動登坂統合(2026-07-18)のスレッド非依存部分。"""

    def test_rotate_map_level(self):
        t = _task()
        t.gmap = GlobalOccupancyMap(size_m=(2.0, 2.0), resolution_m=0.05,
                                    origin_xy=(-1.0, -1.0), map_id="t0")
        t._z_floor = 0.0
        t._rotate_map_level()
        self.assertEqual(t.level, 1)
        self.assertIsNone(t.gmap)      # mapper が新フロアで再作成する
        self.assertIsNone(t._z_floor)  # 床高も新フロアで再推定
        self.assertEqual(t.trace, [])

    def test_abort_my_stair_only_mine(self):
        class StubStair:
            def __init__(self):
                self.aborted = []
            def abort(self, why):
                self.aborted.append(why)
        t = _task()
        t._stair = StubStair()
        t._stair_mine = False
        t._abort_my_stair("x")          # 自分のでない → 触らない
        self.assertEqual(t._stair.aborted, [])
        t._stair_mine = True
        t._abort_my_stair("y")
        self.assertEqual(t._stair.aborted, ["y"])
        self.assertFalse(t._stair_mine)

    def test_snapshot_has_level(self):
        s = _task().snapshot()
        self.assertEqual(s["level"], 0)

    def test_mock_proposal_has_no_stair_note(self):
        # mock では自動登坂しない → 提案文にも含めない
        t = _task()
        r = t.route_text("部屋を探索してマップを作って")
        self.assertNotIn("登坂", r["say"])


class TestMapperBootstrap(unittest.TestCase):
    def test_facade_does_not_start_mapper_or_actuator_thread(self):
        t = _task()
        self.assertIsNone(t._z_floor)
        self.assertTrue(hasattr(t, "_map_lock"))
        self.assertIsNone(t._th)


class TestSafeDelegate(unittest.TestCase):
    def test_confirm_delegates_confirmed_goal_once_without_own_thread(self):
        t, mission = _ready_delegated_task()
        self.assertEqual(t.route_text("部屋を探索してマップを作って")["kind"],
                         "proposal")

        result = t.confirm()

        self.assertEqual(result["kind"], "confirmed")
        self.assertEqual(len(mission.started), 1)
        self.assertEqual(mission.started[0][0].goal_id, result["goal_id"])
        self.assertIsNone(t._th)
        self.assertTrue(t._owns_delegate())

    def test_delegate_rejection_is_propagated(self):
        t, mission = _ready_delegated_task("runner busy")
        t.route_text("部屋を探索してマップを作って")

        result = t.confirm()

        self.assertEqual(result["kind"], "error")
        self.assertIn("runner busy", result["say"])
        self.assertEqual(len(mission.started), 1)
        self.assertFalse(t._run)

    def test_stop_now_aborts_delegate_and_sends_zero(self):
        t, mission = _ready_delegated_task()
        t.route_text("部屋を探索してマップを作って")
        t.confirm()

        result = t.route_text("止まれ")

        self.assertEqual(result["kind"], "stop_now")
        self.assertTrue(mission.aborted)
        self.assertEqual(t.bridge.cmds[-1], (0, 0, 0))

    def test_map_frame_reads_the_delegated_controller_map(self):
        t, mission = _ready_delegated_task()
        mission.gmap = GlobalOccupancyMap(
            size_m=(2.0, 2.0), resolution_m=0.1,
            origin_xy=(-1.0, -1.0), map_id="delegated")
        mission.gmap.mark_hazard([(0.0, 0.0)], now_ns=1)

        buf = t.map_frame()

        kind, _ox, _oy, res, w, h = struct.unpack("<BfffHH", buf[:17])
        self.assertEqual(kind, BIN_EXPMAP)
        self.assertAlmostEqual(res, 0.1)
        self.assertEqual((w, h), (20, 20))


if __name__ == "__main__":
    unittest.main()
