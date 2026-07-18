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
        # no-return beamはmax_range端点まで観測済みFREE。
        self.assertEqual(m.grid[end[1], end[0]], FREE)

    def test_explicit_miss_marks_endpoint_free(self):
        m = make_map()
        m.integrate_scan((0.0, 0.0), [(2.0, 0.0)], now_ns=1_000,
                         hit_mask=[False])
        end = m.world_to_cell(2.0, 0.0)
        self.assertEqual(m.grid[end[1], end[0]], FREE)
        self.assertEqual(m.counts()["occupied"], 0)

    def test_outside_endpoint_keeps_in_map_free_ray(self):
        m = GlobalOccupancyMap(size_m=(2.0, 2.0), resolution_m=0.1,
                               origin_xy=(-1.0, -1.0))
        m.integrate_scan((0.0, 0.0), [(5.0, 0.0)], now_ns=1_000,
                         max_range_m=8.0)
        near_edge = m.world_to_cell(0.95, 0.0)
        self.assertEqual(m.grid[near_edge[1], near_edge[0]], FREE)
        self.assertEqual(m.counts()["occupied"], 0)

    def test_point_cloud_adapter_separates_floor_and_obstacle(self):
        m = make_map()
        # base z=.31 -> ground z=0。床returnはFREE、10cm箱はOCCUPIED。
        pts = np.asarray([[1.0, 0.0, 0.0], [0.0, 1.0, 0.10]], np.float32)
        m.integrate_point_cloud((0.0, 0.0, 0.31, 0.0), pts, now_ns=1_000)
        floor = m.world_to_cell(1.0, 0.0)
        box = m.world_to_cell(0.0, 1.0)
        self.assertEqual(m.grid[floor[1], floor[0]], FREE)
        self.assertEqual(m.grid[box[1], box[0]], OCCUPIED)

    def test_occupied_not_overwritten_by_free_ray(self):
        # 1回の矛盾観測では実壁を消さない。
        m = make_map()
        m.integrate_scan((0.0, 0.0), [(2.0, 0.0)], now_ns=1_000)
        # 同じscanの多数rayでもevidenceは1回分。
        m.integrate_scan((0.0, 0.0), [(4.0, 0.0)] * 20, now_ns=2_000,
                         hit_mask=[False] * 20)
        wall = m.world_to_cell(2.0, 0.0)
        self.assertEqual(m.grid[wall[1], wall[0]], OCCUPIED)

    def test_dynamic_obstacle_clears_after_three_independent_free_scans(self):
        m = make_map()
        wall = m.world_to_cell(2.0, 0.0)
        m.integrate_scan((0.0, 0.0), [(2.0, 0.0)], now_ns=1_000)
        for i in range(2):
            m.integrate_scan((0.0, 0.0), [(4.0, 0.0)],
                             now_ns=2_000 + i, hit_mask=[False])
            self.assertEqual(m.grid[wall[1], wall[0]], OCCUPIED)
        m.integrate_scan((0.0, 0.0), [(4.0, 0.0)],
                         now_ns=3_000, hit_mask=[False])
        self.assertEqual(m.grid[wall[1], wall[0]], FREE)

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

    def test_stale_free_is_not_traversable(self):
        m = make_map()
        m.integrate_scan((0.0, 0.0), [(2.0, 0.0)], now_ns=1_000)
        fresh = m.traversable_mask(inflate_cells=0, now_ns=1_500,
                                   max_age_ns=1_000)
        stale = m.traversable_mask(inflate_cells=0, now_ns=3_000,
                                   max_age_ns=1_000)
        c = m.world_to_cell(1.0, 0.0)
        self.assertTrue(fresh[c[1], c[0]])
        self.assertFalse(stale[c[1], c[0]])

    def test_metric_inflation_is_resolution_independent(self):
        for res in (0.05, 0.10):
            with self.subTest(resolution=res):
                m = GlobalOccupancyMap(size_m=(4.0, 4.0), resolution_m=res,
                                       origin_xy=(-2.0, -2.0))
                wall = m.world_to_cell(1.0, 0.0)
                m.grid[wall[1], wall[0]] = OCCUPIED
                near = m.world_to_cell(0.76, 0.0)
                far = m.world_to_cell(0.55, 0.0)
                m.grid[near[1], near[0]] = FREE
                m.grid[far[1], far[0]] = FREE
                trav = m.traversable_mask(inflation_radius_m=0.30)
                self.assertFalse(trav[near[1], near[0]])
                self.assertTrue(trav[far[1], far[0]])

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
        for i in range(5):
            m.integrate_scan((0.0, 0.0), [(2.0, 0.0)],
                             now_ns=3_000 + i, hit_mask=[False])
        self.assertEqual(m.grid[c[1], c[0]], OCCUPIED)

    def test_legacy_optimistic_query_is_explicit_only(self):
        # 旧UI照会互換ではUNKNOWNを表示できるが、既定/LIVE pathは常にFREEのみ。
        m = make_map()
        m.integrate_scan((0.0, 0.0), [(2.0, 0.0)], now_ns=1_000)
        opt = m.traversable_mask(inflate_cells=3, optimistic=True)
        default = m.traversable_mask(inflate_cells=3)
        behind = m.world_to_cell(3.0, 0.0)      # 壁の向こう(UNKNOWN)
        wall = m.world_to_cell(2.0, 0.0)        # OCCUPIED
        near_wall = m.world_to_cell(1.85, 0.0)  # inflate 圏内
        self.assertTrue(opt[behind[1], behind[0]])
        self.assertFalse(default[behind[1], behind[0]])
        self.assertFalse(opt[wall[1], wall[0]])
        self.assertFalse(opt[near_wall[1], near_wall[0]])


class TestEvidenceDecay(unittest.TestCase):
    """証拠カウンタ(2026-07-17): 動的障害物は減衰して消え、壁と hazard は残る。"""

    def test_dynamic_obstacle_decays(self):
        # 人の脚など: 一度 OCCUPIED になっても ray が繰り返し通過すれば
        # FREE へ降格する(地図の自己修正 — 幻の壁で探索が詰まらない)
        m = make_map()
        m.integrate_scan((0.0, 0.0), [(1.0, 0.0)], now_ns=1_000)
        c = m.world_to_cell(1.0, 0.0)
        self.assertEqual(m.grid[c[1], c[0]], OCCUPIED)
        for i in range(3):   # 同位置を3回 ray が通過(人は既にいない)
            m.integrate_scan((0.0, 0.0), [(2.0, 0.0)], now_ns=2_000 + i)
        self.assertEqual(m.grid[c[1], c[0]], FREE)

    def test_single_pass_keeps_occupied(self):
        # 保守則: 1回の通過だけでは消えない(ノイズ耐性)
        m = make_map()
        m.integrate_scan((0.0, 0.0), [(1.0, 0.0)], now_ns=1_000)
        m.integrate_scan((0.0, 0.0), [(2.0, 0.0)], now_ns=2_000)
        c = m.world_to_cell(1.0, 0.0)
        self.assertEqual(m.grid[c[1], c[0]], OCCUPIED)

    def test_wall_persists_with_rehits(self):
        # 実在の壁: ヒットが通過より優勢(+2/-1)なので観測が続く限り維持
        m = make_map()
        c = m.world_to_cell(2.0, 0.0)
        for i in range(10):
            m.integrate_scan((0.0, 0.0), [(2.0, 0.0)], now_ns=1_000 + 2 * i)
            m.integrate_free_rays((0.0, 0.0), [(2.0, 0.0)],
                                  now_ns=1_001 + 2 * i)
        self.assertEqual(m.grid[c[1], c[0]], OCCUPIED)

    def test_hazard_never_decays(self):
        # drop hazard は何度 ray が通過しても消えない(安全側で永続)
        m = make_map()
        m.mark_hazard([(1.0, 0.0)], now_ns=1_000)
        for i in range(10):
            m.integrate_free_rays((0.0, 0.0), [(2.0, 0.0)], now_ns=2_000 + i)
        c = m.world_to_cell(1.0, 0.0)
        self.assertEqual(m.grid[c[1], c[0]], OCCUPIED)

    def test_clear_footprint_never_purges_hazard(self):
        # footprintによる自己点除去でもdrop/段差hazardは解除しない。
        m = make_map()
        m.mark_hazard([(0.1, 0.1)], now_ns=1_000)         # 幻(自己点由来)
        m.integrate_scan((0.0, 0.0), [(2.0, 0.0)], now_ns=2_000)  # 実在の壁
        m.clear_footprint((0.0, 0.0), 0.30, now_ns=3_000)
        near = m.world_to_cell(0.1, 0.1)
        wall = m.world_to_cell(2.0, 0.0)
        self.assertEqual(m.grid[near[1], near[0]], OCCUPIED)
        self.assertTrue(m.hazard_mask[near[1], near[0]])
        self.assertEqual(m.grid[wall[1], wall[0]], OCCUPIED)


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
        self.assertTrue(np.array_equal(m.free_evidence, m2.free_evidence))
        self.assertTrue(np.array_equal(m.hazard_mask, m2.hazard_mask))
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
