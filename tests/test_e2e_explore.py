"""offline E2E テスト(docs/10 §6 E2 相当、robot 非接続・決定的)。

音声テキスト → parser → 確認 → Mission FSM → frontier 探索 → arbiter →
gateway → 合成世界 sim → マップ構築、の全経路を1本で検証する。
"""
import unittest

from demo.explore_e2e import run_e2e
from mission.executive import MissionState


class TestExploreE2E(unittest.TestCase):
    def test_full_exploration_completes_and_maps_both_rooms(self):
        r = run_e2e()
        self.assertTrue(r.completed, r.narrative)
        self.assertIs(r.final_mission_state, MissionState.ACTIVE_HOLD)
        # 部屋B(x=1.5)まで地図化されている(ドア越しの探索が成立)
        self.assertTrue(r.room_b_mapped, r.map_counts)
        self.assertGreater(r.map_counts["free"], 1000)
        self.assertGreater(r.map_counts["occupied"], 50)

    def test_deterministic(self):
        r1 = run_e2e()
        r2 = run_e2e()
        self.assertEqual(r1.robot_xy, r2.robot_xy)
        self.assertEqual(r1.steps, r2.steps)
        self.assertEqual(r1.map_counts, r2.map_counts)

    def test_stop_now_halts_exploration(self):
        r = run_e2e(stop_after_steps=5, max_steps=40)
        self.assertTrue(r.stopped_by_operator)
        self.assertFalse(r.completed)
        self.assertIs(r.final_mission_state, MissionState.ACTIVE_HOLD)
        # 自動再開しない(その後のステップでも完了に達しない)
        self.assertGreaterEqual(r.steps, 40)


if __name__ == "__main__":
    unittest.main()
