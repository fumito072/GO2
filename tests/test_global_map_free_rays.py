"""GlobalOccupancyMap.integrate_free_rays の unit test (E0, docs/12 §3)。"""
import unittest

import numpy as np

from contracts.errors import ContractViolation
from perception.global_map import FREE, OCCUPIED, UNKNOWN, GlobalOccupancyMap


def _mk(res=0.1):
    return GlobalOccupancyMap(size_m=(10.0, 10.0), resolution_m=res,
                              origin_xy=(-5.0, -5.0), map_id="t")


class TestIntegrateFreeRays(unittest.TestCase):
    def test_endpoint_becomes_free_not_occupied(self):
        m = _mk()
        m.integrate_free_rays((0.0, 0.0), [(2.0, 0.0)], now_ns=1)
        c = m.world_to_cell(2.0, 0.0)
        self.assertEqual(m.grid[c[1], c[0]], FREE)

    def test_ray_cells_become_free(self):
        m = _mk()
        m.integrate_free_rays((0.0, 0.0), [(2.0, 0.0)], now_ns=1)
        for x in (0.05, 0.55, 1.05, 1.55, 1.95):
            c = m.world_to_cell(x, 0.0)
            self.assertEqual(m.grid[c[1], c[0]], FREE, msg=f"x={x}")

    def test_never_creates_occupied(self):
        m = _mk()
        m.integrate_free_rays((0.0, 0.0),
                              [(2.0, 1.0), (-1.5, -2.0), (3.0, 3.0)], now_ns=1)
        self.assertEqual(m.counts()["occupied"], 0)

    def test_does_not_overwrite_occupied(self):
        m = _mk()
        m.mark_hazard([(1.0, 0.0)], now_ns=1)
        occ_cell = m.world_to_cell(1.0, 0.0)
        m.integrate_free_rays((0.0, 0.0), [(2.0, 0.0)], now_ns=2)
        self.assertEqual(m.grid[occ_cell[1], occ_cell[0]], OCCUPIED)

    def test_max_range_clips(self):
        m = _mk()
        m.integrate_free_rays((0.0, 0.0), [(4.5, 0.0)], now_ns=1, max_range_m=2.0)
        near = m.world_to_cell(1.5, 0.0)
        far = m.world_to_cell(3.0, 0.0)
        self.assertEqual(m.grid[near[1], near[0]], FREE)
        self.assertEqual(m.grid[far[1], far[0]], UNKNOWN)

    def test_updates_age(self):
        m = _mk()
        m.integrate_free_rays((0.0, 0.0), [(1.0, 0.0)], now_ns=77)
        c = m.world_to_cell(0.5, 0.0)
        self.assertEqual(int(m.age_ns[c[1], c[0]]), 77)

    def test_returns_newly_freed_count_and_idempotent(self):
        m = _mk()
        n1 = m.integrate_free_rays((0.0, 0.0), [(1.0, 0.0)], now_ns=1)
        self.assertGreater(n1, 0)
        n2 = m.integrate_free_rays((0.0, 0.0), [(1.0, 0.0)], now_ns=2)
        self.assertEqual(n2, 0)

    def test_nonfinite_points_ignored(self):
        m = _mk()
        n = m.integrate_free_rays((0.0, 0.0),
                                  [(float("nan"), 0.0), (float("inf"), 1.0)],
                                  now_ns=1)
        # 不正rayは無視するが、robotが実在するcellはFREEへ更新される。
        self.assertLessEqual(n, 1)
        self.assertEqual(m.counts()["occupied"], 0)

    def test_out_of_map_ray_preserves_observed_free_cells_to_boundary(self):
        m = _mk()
        # endpointが地図外でも、そこまで実際に通ったrayを捨てない。
        n = m.integrate_free_rays((0.0, 0.0), [(20.0, 0.0)], now_ns=1,
                                  max_range_m=30.0)
        self.assertGreater(n, 0)
        edge = m.world_to_cell(4.95, 0.0)
        self.assertEqual(m.grid[edge[1], edge[0]], FREE)
        self.assertEqual(m.counts()["occupied"], 0)

    def test_bad_now_ns_rejected(self):
        m = _mk()
        for bad in (0, -1, 1.5, True, None):
            with self.assertRaises(ContractViolation, msg=repr(bad)):
                m.integrate_free_rays((0.0, 0.0), [(1.0, 0.0)], now_ns=bad)

    def test_robot_outside_map_rejected(self):
        m = _mk()
        with self.assertRaises(ContractViolation):
            m.integrate_free_rays((99.0, 0.0), [(1.0, 0.0)], now_ns=1)


if __name__ == "__main__":
    unittest.main()
