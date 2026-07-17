"""frontier_explorer の synthetic テスト(docs/10 §6 E0、robot 非接続)。

合成の正方形部屋(壁±2m)を中心から観測した地図で、
frontier 抽出・goal 提案・探索完了・誤完了防止を検証する。
"""
import math
import unittest

from contracts import ContractViolation
from perception.global_map import FREE, OCCUPIED, GlobalOccupancyMap
from navigation.frontier_explorer import (
    ExplorationStatus, FrontierExplorer, next_goal,
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


class TestReachabilityAndHistory(unittest.TestCase):
    def test_disconnected_free_island_is_not_returned_as_goal(self):
        """UNKNOWN/壁越しの FREE 島は直線距離が近くても到達不能。"""
        m = GlobalOccupancyMap(size_m=(6.0, 6.0), resolution_m=0.1,
                               origin_xy=(-3.0, -3.0), map_id="islands")
        # robot 周囲は既知 FREE だが OCCUPIED ring で閉じている。
        m.grid[28:33, 28:33] = OCCUPIED
        m.grid[29:32, 29:32] = FREE
        # 1.4 m 先に大きな frontier を持つ分離 FREE 島。
        m.grid[29:34, 44:49] = FREE
        robot_xy = m.cell_to_world(30, 30)

        d = next_goal(m, robot_xy, min_cluster_cells=5,
                      inflate_cells=0, standoff_m=0.0,
                      min_progress_m=0.1)

        self.assertIs(d.status, ExplorationStatus.NO_REACHABLE_FRONTIER,
                      d.reason)
        self.assertIsNone(d.goal)

    def test_astar_path_routes_through_door_instead_of_wall(self):
        """goalへの直線が壁と交差しても、pathは開口を通る。"""
        m = GlobalOccupancyMap(size_m=(10.0, 7.0), resolution_m=0.1,
                               origin_xy=(0.0, 0.0), map_id="door_maze")
        # 既知領域。右端だけ UNKNOWN に開いており frontier になる。
        m.grid[10:61, 10:81] = FREE
        m.grid[9, 9:82] = OCCUPIED
        m.grid[61, 9:82] = OCCUPIED
        m.grid[9:62, 9] = OCCUPIED
        # x=45 の仕切り壁。y=29..41 のみ幅 1.3 m のドア。
        m.grid[10:61, 45] = OCCUPIED
        m.grid[29:42, 45] = FREE
        robot_xy = m.cell_to_world(20, 20)

        d = next_goal(m, robot_xy, min_cluster_cells=5,
                      max_step_m=20.0, inflate_cells=2,
                      standoff_m=0.0, min_progress_m=0.1)

        self.assertIs(d.status, ExplorationStatus.GOAL, d.reason)
        self.assertTrue(d.goal.path)
        path_cells = [m.world_to_cell(*point) for point in d.goal.path]
        self.assertLess(path_cells[0][0], 45)
        self.assertGreater(path_cells[-1][0], 45)
        door_crossings = [cell for cell in path_cells if cell[0] == 45]
        self.assertTrue(door_crossings, path_cells)
        self.assertTrue(all(29 <= y <= 41 for _, y in door_crossings),
                        door_crossings)
        self.assertTrue(all(m.grid[y, x] != OCCUPIED for x, y in path_cells))
        # 開口へ迂回するので、計画 path は直線より長い。
        self.assertGreater(d.goal.path_length_m, d.goal.distance_m + 0.2)

    def test_small_frontier_is_not_misreported_as_complete(self):
        m = GlobalOccupancyMap(size_m=(2.0, 2.0), resolution_m=0.1,
                               origin_xy=(-1.0, -1.0), map_id="small_frontier")
        m.grid[10:12, 10:12] = FREE  # 4 cells < default min_cluster_cells=5
        robot_xy = m.cell_to_world(10, 10)

        d = next_goal(m, robot_xy, min_cluster_cells=5,
                      inflate_cells=0, standoff_m=0.0,
                      min_progress_m=0.1)

        self.assertIsNot(d.status, ExplorationStatus.COMPLETE, d.reason)
        self.assertIs(d.status, ExplorationStatus.GOAL, d.reason)
        self.assertLess(d.goal.frontier_cells, 5)
        self.assertIn("small-frontier", d.reason)

    def test_recent_and_failed_goal_is_not_immediately_reissued(self):
        m = scan_room(make_map(), door=True)
        explorer = FrontierExplorer(
            m, inflation_radius_m=0.30, max_step_m=3.0,
            standoff_m=0.35, failure_cooldown_plans=8)

        first = explorer.plan((0.0, 0.0))
        self.assertIs(first.status, ExplorationStatus.GOAL, first.reason)
        failed_cell = first.goal.goal_cell
        explorer.goal_failed()
        second = explorer.plan((0.0, 0.0))

        self.assertIs(second.status, ExplorationStatus.GOAL, second.reason)
        self.assertNotEqual(second.goal.goal_cell, failed_cell)
        # recent-goal penaltyの範囲(0.5 m)外へ切り替える。
        separation = math.hypot(second.goal.goal_cell[0] - failed_cell[0],
                                second.goal.goal_cell[1] - failed_cell[1]) \
            * m.resolution_m
        self.assertGreater(separation, 0.5)
        self.assertEqual(explorer.goal_attempts[failed_cell], 1)


if __name__ == "__main__":
    unittest.main()
