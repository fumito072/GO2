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
        # 地図がドア越しに見えただけでなく、robot自身が部屋Bへ入る。
        self.assertTrue(r.room_b_mapped, r.map_counts)
        self.assertTrue(r.room_b_entered, r.narrative)
        # collision-aware simで壁抜け命令を1回も出さない。
        self.assertEqual(r.collision_count, 0, r.narrative)
        # 外周壁内ROIを部分的に見ただけで完了としない。
        self.assertGreaterEqual(r.coverage_ratio, 0.95, r.map_counts)
        # ルンバ状に面的に移動し、同じcellの往復は有界である。
        self.assertGreaterEqual(r.unique_visited_cells, 60, r.narrative)
        self.assertLessEqual(r.revisit_ratio, 0.15, r.narrative)
        self.assertGreater(r.map_counts["free"], 1000)
        self.assertGreater(r.map_counts["occupied"], 50)

    def test_deterministic(self):
        r1 = run_e2e()
        r2 = run_e2e()
        self.assertEqual(r1.robot_xy, r2.robot_xy)
        self.assertEqual(r1.steps, r2.steps)
        self.assertEqual(r1.map_counts, r2.map_counts)
        self.assertEqual(r1.room_b_entered, r2.room_b_entered)
        self.assertEqual(r1.collision_count, r2.collision_count)
        self.assertEqual(r1.coverage_ratio, r2.coverage_ratio)
        self.assertEqual(r1.unique_visited_cells, r2.unique_visited_cells)
        self.assertEqual(r1.revisit_ratio, r2.revisit_ratio)

    def test_stop_now_halts_exploration(self):
        r = run_e2e(stop_after_steps=5, max_steps=40)
        self.assertTrue(r.stopped_by_operator)
        self.assertFalse(r.completed)
        self.assertIs(r.final_mission_state, MissionState.ACTIVE_HOLD)
        # 自動再開しない(その後のステップでも完了に達しない)
        self.assertGreaterEqual(r.steps, 40)


if __name__ == "__main__":
    unittest.main()
