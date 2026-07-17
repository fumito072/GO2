"""frontier_explorer の synthetic テスト(docs/10 §6 E0、robot 非接続)。

合成の正方形部屋(壁±2m)を中心から観測した地図で、
frontier 抽出・goal 提案・探索完了・誤完了防止を検証する。
"""
import math
import unittest

from contracts import ContractViolation
from perception.global_map import GlobalOccupancyMap, FREE, OCCUPIED
from navigation.frontier_explorer import (
    ExplorationStatus, next_goal, _clip_along_ray,
)


def make_map():
    return GlobalOccupancyMap(size_m=(12.0, 12.0), resolution_m=0.1,
                              origin_xy=(-6.0, -6.0), map_id="test_room")


def wall_points(half=2.0, step=0.02, door_y=None, door_half=0.0):
    """±half の正方形の壁点群。door_y 指定時は東壁(x=+half)の
    |y - door_y| < door_half を開け、開口の先(x=2.5*half)に外壁の点を置く
    (実 LiDAR は開口越しに奥を観測するため。これがないと開口が UNKNOWN の
    まま frontier が育たない)。"""
    pts = []
    n = int(2 * half / step)
    for i in range(n + 1):
        t = -half + i * step
        pts.append((t, half))    # 北
        pts.append((t, -half))   # 南
        pts.append((-half, t))   # 西
        if door_y is not None and abs(t - door_y) < door_half:
            # 開口: 中心からの同一 ray 上の遠方点(開口の先の壁)を観測
            pts.append((2.5 * half, 2.5 * t))
            continue
        pts.append((half, t))    # 東(door を開けられる)
    return pts


def scan_room(m, door=False, now=1_000):
    pts = wall_points(door_y=0.0 if door else None,
                      door_half=0.4 if door else 0.0)
    m.integrate_scan((0.0, 0.0), pts, now_ns=now, max_range_m=8.0)
    return m


class TestExplorationLifecycle(unittest.TestCase):
    def test_no_observations_is_not_complete(self):
        # 観測ゼロを「探索完了」と誤判定しない(誤完了の防止 — docs/10 §5)
        m = make_map()
        d = next_goal(m, (0.0, 0.0))
        self.assertIs(d.status, ExplorationStatus.NO_OBSERVATIONS)
        self.assertIsNone(d.goal)

    def test_enclosed_room_completes(self):
        # 完全に閉じた部屋 → frontier 枯渇 → COMPLETE
        m = scan_room(make_map(), door=False)
        d = next_goal(m, (0.0, 0.0))
        self.assertIs(d.status, ExplorationStatus.COMPLETE, d.reason)

    def test_room_with_door_proposes_goal_toward_opening(self):
        # 東壁(x=+2)に幅 0.8m の開口 → goal は開口方向
        m = scan_room(make_map(), door=True)
        d = next_goal(m, (0.0, 0.0))
        self.assertIs(d.status, ExplorationStatus.GOAL, d.reason)
        self.assertGreater(d.goal.x, 0.5)          # 東(開口)方向
        self.assertLess(abs(d.goal.y), 1.5)        # 開口の cone 内
        self.assertLessEqual(d.goal.distance_m, 3.0 + 1e-6)
        self.assertGreaterEqual(d.goal.frontier_cells, 5)

    def test_max_step_clipping(self):
        m = scan_room(make_map(), door=True)
        d = next_goal(m, (-1.5, 0.0), max_step_m=1.5)
        if d.status is ExplorationStatus.GOAL:
            self.assertLessEqual(d.goal.distance_m, 1.5 + 1e-6)
        else:
            # clip 不能なら「到達可能 frontier なし」であり完了ではない
            self.assertIs(d.status, ExplorationStatus.NO_REACHABLE_FRONTIER)

    def test_deterministic(self):
        m = scan_room(make_map(), door=True)
        d1 = next_goal(m, (0.0, 0.0))
        d2 = next_goal(m, (0.0, 0.0))
        self.assertEqual(d1, d2)

    def test_goal_is_traversable_free_cell(self):
        # goal は必ず通行可能 cell(UNKNOWN や inflate 域に goal を置かない)
        m = scan_room(make_map(), door=True)
        d = next_goal(m, (0.0, 0.0))
        self.assertIs(d.status, ExplorationStatus.GOAL)
        trav = m.traversable_mask(inflate_cells=3)
        c = m.world_to_cell(d.goal.x, d.goal.y)
        self.assertTrue(trav[c[1], c[0]])

    def test_type_checks(self):
        with self.assertRaises(ContractViolation):
            next_goal({"grid": None}, (0.0, 0.0))
        m = scan_room(make_map())
        with self.assertRaises(ContractViolation):
            next_goal(m, (99.0, 99.0))  # 地図範囲外

    def test_yaw_points_toward_frontier(self):
        m = scan_room(make_map(), door=True)
        d = next_goal(m, (0.0, 0.0))
        self.assertIs(d.status, ExplorationStatus.GOAL)
        self.assertTrue(math.isfinite(d.goal.yaw))
        # 開口は東 → yaw はおおむね東向き(±90°)
        self.assertLess(abs(d.goal.yaw), math.pi / 2 + 0.2)


class TestOptimisticWalkable(unittest.TestCase):
    """optimistic モード(操作者要望 2026-07-17): 未踏 UNKNOWN を通って
    frontier へ踏み出せる。frontier の定義(FREE/UNKNOWN 境界)は不変。"""

    def test_clip_crosses_unknown_gap_only_when_optimistic(self):
        # FREE — UNKNOWN の隙間 — FREE(実 LiDAR の床観測が疎なときの形)
        m = make_map()
        bx, by = m.world_to_cell(0.0, 0.0)
        for di in range(6):                     # x ∈ [0, 0.6) を FREE
            for dj in range(-3, 4):
                m.grid[by + dj, bx + di] = FREE
        for di in range(15, 30):                # x ∈ [1.5, 3.0) を FREE
            for dj in range(-3, 4):
                m.grid[by + dj, bx + di] = FREE
        trav = m.traversable_mask(inflate_cells=0)
        walk = m.traversable_mask(inflate_cells=0, optimistic=True)
        p = _clip_along_ray(m, trav, (0.0, 0.0), (2.8, 0.0), 2.5)
        q = _clip_along_ray(m, walk, (0.0, 0.0), (2.8, 0.0), 2.5)
        self.assertIsNotNone(p)
        self.assertLess(p[0], 0.6)              # 既定: 隙間の手前で打ち切り
        self.assertIsNotNone(q)
        self.assertGreater(q[0], 2.0)           # optimistic: 隙間を越えて前進

    def test_optimistic_clip_still_stops_at_occupied(self):
        # optimistic でも観測済み障害物では打ち切る
        m = make_map()
        bx, by = m.world_to_cell(0.0, 0.0)
        for di in range(30):
            for dj in range(-3, 4):
                m.grid[by + dj, bx + di] = FREE
        for dj in range(-3, 4):                 # x≒1.0 に壁
            m.grid[by + dj, bx + 10] = OCCUPIED
        walk = m.traversable_mask(inflate_cells=0, optimistic=True)
        q = _clip_along_ray(m, walk, (0.0, 0.0), (2.8, 0.0), 2.5)
        self.assertIsNotNone(q)
        self.assertLess(q[0], 1.0)

    def test_optimistic_same_goal_on_fully_observed_room(self):
        # 完全観測済み(FREE で連結)の部屋では既定と同じ goal(決定性を保つ)
        m = scan_room(make_map(), door=True)
        d0 = next_goal(m, (0.0, 0.0))
        d1 = next_goal(m, (0.0, 0.0), optimistic=True)
        self.assertEqual(d0, d1)

    def test_optimistic_enclosed_room_still_completes(self):
        # optimistic でも frontier 枯渇 → COMPLETE(誤って探索し続けない)
        m = scan_room(make_map(), door=False)
        d = next_goal(m, (0.0, 0.0), optimistic=True)
        self.assertIs(d.status, ExplorationStatus.COMPLETE, d.reason)

    def test_min_goal_dist_pushes_past_frontier(self):
        # 床観測が疎で robot 自身が frontier 直近に立つケース(実機 2026-07-17):
        # 押し出しなしだと goal が至近(即到達→同じ goal 再選択)で動けない
        m = make_map()
        bx, by = m.world_to_cell(0.0, 0.0)
        for di in range(-3, 4):                 # robot 周囲 ±0.3m のみ FREE
            for dj in range(-3, 4):
                m.grid[by + dj, bx + di] = FREE
        d0 = next_goal(m, (0.0, 0.0), inflate_cells=0, optimistic=True)
        self.assertIs(d0.status, ExplorationStatus.GOAL)
        self.assertLess(d0.goal.distance_m, 0.4)   # 押し出しなし: 至近 goal
        d1 = next_goal(m, (0.0, 0.0), inflate_cells=0, optimistic=True,
                       min_goal_dist_m=0.4)
        self.assertIs(d1.status, ExplorationStatus.GOAL)
        self.assertGreaterEqual(d1.goal.distance_m, 0.4)  # 境界の先へ押し出す

    def test_rng_is_reproducible_and_valid(self):
        # 確率化(操作者要望 2026-07-18): 同じ seed → 同じ goal(再現可能)。
        # goal は常に有効(GOAL status + 通行可能域内)
        import random
        m = scan_room(make_map(), door=True)
        d1 = next_goal(m, (0.0, 0.0), rng=random.Random(42))
        d2 = next_goal(m, (0.0, 0.0), rng=random.Random(42))
        self.assertEqual(d1, d2)
        self.assertIs(d1.status, ExplorationStatus.GOAL)

    def test_rng_diversifies_cluster_choice(self):
        # 2つの独立した frontier(大小)がある地図で、seed を変えると
        # 小さい cluster も時々選ばれる(常に argmax ではない)
        import random
        m = make_map()
        bx, by = m.world_to_cell(0.0, 0.0)
        for di in range(-2, 3):                 # robot 足場
            for dj in range(-2, 3):
                m.grid[by + dj, bx + di] = FREE
        for di in range(10, 20):                # 東に大きな FREE 帯(境界=frontier)
            for dj in range(-8, 9):
                m.grid[by + dj, bx + di] = FREE
        for di in range(-16, -10):              # 西に小さな FREE 帯
            for dj in range(-3, 4):
                m.grid[by + dj, bx + di] = FREE
        got = set()
        for seed in range(30):
            d = next_goal(m, (0.0, 0.0), inflate_cells=0, optimistic=True,
                          rng=random.Random(seed))
            if d.status is ExplorationStatus.GOAL:
                got.add("east" if d.goal.x > 0 else "west")
        self.assertIn("east", got)
        self.assertIn("west", got)   # 低スコア側も選ばれることがある

    def test_no_rng_stays_deterministic(self):
        m = scan_room(make_map(), door=True)
        self.assertEqual(next_goal(m, (0.0, 0.0)), next_goal(m, (0.0, 0.0)))

    def test_avoid_xy_skips_recent_goal(self):
        # 閉塞で断念した goal の近傍は再選択しない(袋小路ループ防止)
        m = scan_room(make_map(), door=True)
        d0 = next_goal(m, (0.0, 0.0))
        self.assertIs(d0.status, ExplorationStatus.GOAL)
        d1 = next_goal(m, (0.0, 0.0),
                       avoid_xy=[(d0.goal.x, d0.goal.y)])
        if d1.status is ExplorationStatus.GOAL:
            self.assertGreaterEqual(
                math.hypot(d1.goal.x - d0.goal.x, d1.goal.y - d0.goal.y), 0.5)


if __name__ == "__main__":
    unittest.main()
