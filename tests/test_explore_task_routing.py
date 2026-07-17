"""cockpit.explore_task のルーティング/確認フローの unit test (E0)。

実行ループ(実機/mock接続)は E2 の mock server E2E で検証する。
ここではスレッドを起こさない範囲(route_text / confirm ガード / snapshot /
map_frame)を stub bridge で検証する。
"""
import struct
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
        self.cloud_rx_ts = 0.0
        self.bot = StubBot()
        self.cmds = []

    def set_cmd(self, vx, vy, wz):
        self.cmds.append((vx, vy, wz))


def _task():
    return ExploreTask(StubBridge(), artifacts_dir="/tmp/test_explore_maps")


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
    def test_init_starts_mapper_thread(self):
        # 回帰テスト(実機 2026-07-17 19:44-19:51 の「地図未初期化」):
        # __init__ 末尾のマッパー起動がメソッド挿入で切り離され、
        # スレッドが一度も起動しない状態になっていた
        import threading
        n0 = threading.active_count()
        t = _task()
        self.assertIsNone(t._z_floor)          # __init__ で初期化されている
        self.assertTrue(hasattr(t, "_map_lock"))
        self.assertGreater(threading.active_count(), n0,
                           "mapper スレッドが __init__ で起動していない")


if __name__ == "__main__":
    unittest.main()
