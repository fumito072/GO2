"""GlobalOccupancyMap の synthetic テスト(docs/10 §6 E0、robot 非接続)。"""
import math
import unittest

import numpy as np

from contracts import ContractViolation
from perception.global_map import (
    GlobalOccupancyMap, UNKNOWN, FREE, OCCUPIED, _bresenham,
)


def make_map():
    return GlobalOccupancyMap(size_m=(10.0, 10.0), resolution_m=0.1,
                              origin_xy=(-5.0, -5.0), map_id="test_map")


class TestBresenham(unittest.TestCase):
    def test_horizontal_and_diagonal(self):
        self.assertEqual(_bresenham(0, 0, 3, 0), [(0, 0), (1, 0), (2, 0), (3, 0)])
        cells = _bresenham(0, 0, 3, 3)
        self.assertEqual(cells[0], (0, 0))
        self.assertEqual(cells[-1], (3, 3))
        # 連続性(隣接 cell 同士)
        for a, b in zip(cells, cells[1:]):
            self.assertLessEqual(max(abs(a[0] - b[0]), abs(a[1] - b[1])), 1)


class TestIntegrateScan(unittest.TestCase):
    def test_ray_marks_free_and_endpoint_occupied(self):
        m = make_map()
        m.integrate_scan((0.0, 0.0), [(2.0, 0.0)], now_ns=1_000)
        wall = m.world_to_cell(2.0, 0.0)
        mid = m.world_to_cell(1.0, 0.0)
        behind = m.world_to_cell(3.0, 0.0)
        self.assertEqual(m.grid[wall[1], wall[0]], OCCUPIED)
        self.assertEqual(m.grid[mid[1], mid[0]], FREE)
        self.assertEqual(m.grid[behind[1], behind[0]], UNKNOWN)  # 壁の向こうは未知のまま
        self.assertEqual(m.age_ns[wall[1], wall[0]], 1_000)

    def test_beyond_max_range_leaves_no_endpoint(self):
        m = make_map()
        m.integrate_scan((0.0, 0.0), [(4.5, 0.0)], now_ns=1_000, max_range_m=3.0)
        near = m.world_to_cell(2.9, 0.0)
        end = m.world_to_cell(3.0, 0.0)
        self.assertEqual(m.grid[near[1], near[0]], FREE)
        # max_range 端点は OCCUPIED にしない(その先は unknown のまま)
        self.assertNotEqual(m.grid[end[1], end[0]], OCCUPIED)

    def test_occupied_not_overwritten_by_free_ray(self):
        # 保守側: 一度 OCCUPIED になった cell は通過 ray で FREE に戻さない
        m = make_map()
        m.integrate_scan((0.0, 0.0), [(2.0, 0.0)], now_ns=1_000)
        m.integrate_scan((0.0, 0.0), [(4.0, 0.0)], now_ns=2_000)
        wall = m.world_to_cell(2.0, 0.0)
        self.assertEqual(m.grid[wall[1], wall[0]], OCCUPIED)

    def test_nan_points_ignored(self):
        m = make_map()
        updated = m.integrate_scan((0.0, 0.0),
                                   [(math.nan, 0.0), (0.0, math.inf)], now_ns=1_000)
        # robot cell 以外は変化しない
        self.assertLessEqual(updated, 1)
        self.assertEqual(m.counts()["occupied"], 0)

    def test_invalid_inputs_fail_closed(self):
        m = make_map()
        with self.assertRaises(ContractViolation):
            m.integrate_scan((0.0, 0.0), [], now_ns=0)
        with self.assertRaises(ContractViolation):
            m.integrate_scan((99.0, 0.0), [], now_ns=1_000)  # 地図範囲外


class TestSafetySemantics(unittest.TestCase):
    def test_unknown_is_not_traversable(self):
        # invariant 9: unknown != free。未観測 cell は通行不可
        m = make_map()
        m.integrate_scan((0.0, 0.0), [(2.0, 0.0)], now_ns=1_000)
        trav = m.traversable_mask(inflate_cells=0)
        behind = m.world_to_cell(3.0, 0.0)
        self.assertFalse(trav[behind[1], behind[0]])

    def test_inflation_blocks_near_wall(self):
        m = make_map()
        m.integrate_scan((0.0, 0.0), [(2.0, 0.0)], now_ns=1_000)
        trav = m.traversable_mask(inflate_cells=3)  # 0.3 m margin
        near_wall = m.world_to_cell(1.85, 0.0)
        far = m.world_to_cell(1.0, 0.0)
        self.assertFalse(trav[near_wall[1], near_wall[0]])
        self.assertTrue(trav[far[1], far[0]])

    def test_hazard_projection(self):
        # elevation の step/drop 分類は costmap 上 OCCUPIED(docs/10 §5)
        m = make_map()
        m.integrate_scan((0.0, 0.0), [(2.0, 0.0)], now_ns=1_000)
        m.mark_hazard([(1.0, 0.0)], now_ns=2_000)
        c = m.world_to_cell(1.0, 0.0)
        self.assertEqual(m.grid[c[1], c[0]], OCCUPIED)


class TestPersistence(unittest.TestCase):
    def test_roundtrip(self):
        m = GlobalOccupancyMap(size_m=(2.0, 2.0), resolution_m=0.1,
                               origin_xy=(-1.0, -1.0), map_id="rt")
        m.integrate_scan((0.0, 0.0), [(0.8, 0.0)], now_ns=1_000)
        m.set_waypoint("home", (0.0, 0.0, 0.0))
        d = m.to_dict()
        m2 = GlobalOccupancyMap.from_dict(d)
        self.assertTrue(np.array_equal(m.grid, m2.grid))
        self.assertTrue(np.array_equal(m.age_ns, m2.age_ns))
        self.assertEqual(m2.waypoints["home"], (0.0, 0.0, 0.0))

    def test_from_dict_rejects_bad_cells(self):
        m = GlobalOccupancyMap(size_m=(1.0, 1.0), resolution_m=0.5,
                               origin_xy=(0.0, 0.0))
        d = m.to_dict()
        d["cells"][0][0] = 9  # 未知の cell 値
        with self.assertRaises(ContractViolation):
            GlobalOccupancyMap.from_dict(d)
        d2 = m.to_dict()
        d2["age_ns"][0][0] = -5
        with self.assertRaises(ContractViolation):
            GlobalOccupancyMap.from_dict(d2)

    def test_waypoint_validation(self):
        m = make_map()
        with self.assertRaises(ContractViolation):
            m.set_waypoint("", (0, 0, 0))
        with self.assertRaises(ContractViolation):
            m.set_waypoint("home", (math.nan, 0, 0))


if __name__ == "__main__":
    unittest.main()
