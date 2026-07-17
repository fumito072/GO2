"""perception.cloud_projector の unit test (E0, docs/12 §3)。

synthetic 3D 点群で z帯分類 → occupancy 統合 → クリアランス照会を検証する。
"""
import math
import unittest

import numpy as np

from perception.cloud_projector import (
    MAX_OBST_PTS, Z_DROP, Z_FLOOR_HI, Z_FLOOR_LO, Z_OBST_HI,
    apply_cloud, classify_cloud, free_clearance,
)
from perception.global_map import FREE, OCCUPIED, UNKNOWN, GlobalOccupancyMap


def _mk(res=0.1):
    return GlobalOccupancyMap(size_m=(12.0, 12.0), resolution_m=res,
                              origin_xy=(-6.0, -6.0), map_id="t")


def _floor_patch(x0, x1, y0, y1, z=0.0, spacing=0.05):
    xs = np.arange(x0, x1, spacing)
    ys = np.arange(y0, y1, spacing)
    gx, gy = np.meshgrid(xs, ys)
    return np.column_stack([gx.ravel(), gy.ravel(),
                            np.full(gx.size, float(z))])


def _wall(x, y0, y1, z0=0.0, z1=1.0, spacing=0.05):
    ys = np.arange(y0, y1, spacing)
    zs = np.arange(z0, z1, spacing)
    gy, gz = np.meshgrid(ys, zs)
    return np.column_stack([np.full(gy.size, float(x)), gy.ravel(), gz.ravel()])


class TestClassifyCloud(unittest.TestCase):
    def test_bands(self):
        pts = np.array([
            [1.0, 0.0, 0.0],                    # 床
            [1.0, 0.5, Z_FLOOR_HI + 0.10],      # 障害物
            [1.0, -0.5, Z_OBST_HI + 0.30],      # 頭上 → 無視
            [1.0, 1.0, Z_DROP - 0.10],          # 落差
        ])
        cc = classify_cloud(pts, (0, 0), z_floor=0.0, resolution_m=0.1)
        self.assertEqual(len(cc.floor_xy), 1)
        self.assertEqual(len(cc.obstacle_xy), 1)
        self.assertEqual(len(cc.drop_xy), 1)

    def test_z_floor_shift(self):
        # odom の z 原点がずれていても z_floor 注入で正しく分類される
        pts = np.array([[1.0, 0.0, 5.0], [1.0, 0.5, 5.3]])
        cc = classify_cloud(pts, (0, 0), z_floor=5.0, resolution_m=0.1)
        self.assertEqual(len(cc.floor_xy), 1)
        self.assertEqual(len(cc.obstacle_xy), 1)

    def test_out_of_range_dropped(self):
        pts = np.array([[10.0, 0.0, 0.3]])
        cc = classify_cloud(pts, (0, 0), z_floor=0.0, resolution_m=0.1,
                            max_range_m=6.0)
        self.assertEqual(len(cc.obstacle_xy), 0)

    def test_nonfinite_dropped(self):
        pts = np.array([[np.nan, 0.0, 0.3], [1.0, np.inf, 0.3],
                        [1.0, 0.0, np.nan]])
        cc = classify_cloud(pts, (0, 0), z_floor=0.0, resolution_m=0.1)
        self.assertEqual(len(cc.floor_xy) + len(cc.obstacle_xy)
                         + len(cc.drop_xy), 0)

    def test_empty_input(self):
        cc = classify_cloud(np.zeros((0, 3)), (0, 0), z_floor=0.0,
                            resolution_m=0.1)
        self.assertEqual(len(cc.floor_xy), 0)

    def test_dedup_and_cap(self):
        # 同一 cell 上の大量点は間引かれ、上限も守られる
        pts = _floor_patch(0.5, 5.5, -3.0, 3.0, z=0.3, spacing=0.01)
        cc = classify_cloud(pts, (0, 0), z_floor=0.0, resolution_m=0.1)
        self.assertLessEqual(len(cc.obstacle_xy), MAX_OBST_PTS)
        self.assertGreater(len(cc.obstacle_xy), 0)


class TestSelfFilter(unittest.TestCase):
    """自己反射フィルタ+フットプリント浄化(2026-07-18 実機: home 0.43m に
    自己点由来 OCCUPIED が現れ全方位閉塞の一因になった)。"""

    def test_self_points_not_obstacle_or_drop(self):
        pts = np.array([[0.3, 0.0, 0.30],    # 機体上の点(障害物帯) → 除外
                        [0.3, 0.1, -0.30],   # 脚元の下向き反射(落差帯) → 除外
                        [1.0, 0.0, 0.30],    # 本物の障害物(0.45m外) → 採用
                        [0.3, -0.1, 0.02]])  # 至近の床 → FREE証拠として採用
        cc = classify_cloud(pts, (0.0, 0.0), z_floor=0.0, resolution_m=0.1)
        self.assertEqual(len(cc.obstacle_xy), 1)
        self.assertAlmostEqual(cc.obstacle_xy[0][0], 1.0)
        self.assertEqual(len(cc.drop_xy), 0)
        self.assertEqual(len(cc.floor_xy), 1)

    def test_apply_cloud_clears_footprint(self):
        # 事前に(誤って)ロボット直下に OCCUPIED があっても、apply_cloud の
        # フットプリント浄化で FREE に戻る
        m = _mk()
        c = m.world_to_cell(0.2, 0.0)
        m.mark_hazard([(0.2, 0.0)], now_ns=1)   # hazard でも浄化される
        self.assertEqual(m.grid[c[1], c[0]], OCCUPIED)
        apply_cloud(m, (0, 0), _floor_patch(0.5, 1.0, -0.3, 0.3, z=0.0),
                    z_floor=0.0, now_ns=2)
        self.assertEqual(m.grid[c[1], c[0]], FREE)


class TestApplyCloud(unittest.TestCase):
    def test_floor_only_carves_free_no_occupied(self):
        m = _mk()
        pts = _floor_patch(-1.0, 2.0, -1.0, 1.0, z=0.0)
        stats = apply_cloud(m, (0, 0), pts, z_floor=0.0, now_ns=1)
        self.assertGreater(stats["updated_free"], 0)
        self.assertEqual(m.counts()["occupied"], 0)
        c = m.world_to_cell(1.5, 0.5)
        self.assertEqual(m.grid[c[1], c[0]], FREE)

    def test_wall_becomes_occupied_with_free_before(self):
        m = _mk()
        pts = np.vstack([_floor_patch(-0.5, 1.9, -1.0, 1.0, z=0.0),
                         _wall(2.0, -1.0, 1.0)])
        apply_cloud(m, (0, 0), pts, z_floor=0.0, now_ns=1)
        wall_c = m.world_to_cell(2.0, 0.0)
        before_c = m.world_to_cell(1.5, 0.0)
        self.assertEqual(m.grid[wall_c[1], wall_c[0]], OCCUPIED)
        self.assertEqual(m.grid[before_c[1], before_c[0]], FREE)

    def test_drop_is_hazard_and_persists(self):
        m = _mk()
        # 前方1.5mに落差(z=-0.5)。床 ray がその上を通っても hazard が残る
        drop = np.array([[1.5, 0.0, -0.5]])
        floor = _floor_patch(0.2, 2.5, -0.3, 0.3, z=0.0)
        apply_cloud(m, (0, 0), np.vstack([floor, drop]), z_floor=0.0, now_ns=1)
        c = m.world_to_cell(1.5, 0.0)
        self.assertEqual(m.grid[c[1], c[0]], OCCUPIED)
        # 次フレームの床 ray でも FREE に戻らない(保守則)
        apply_cloud(m, (0, 0), floor, z_floor=0.0, now_ns=2)
        self.assertEqual(m.grid[c[1], c[0]], OCCUPIED)

    def test_overhead_ignored(self):
        m = _mk()
        pts = np.array([[1.0, 0.0, Z_OBST_HI + 0.5]])
        apply_cloud(m, (0, 0), pts, z_floor=0.0, now_ns=1)
        c = m.world_to_cell(1.0, 0.0)
        self.assertEqual(m.grid[c[1], c[0]], UNKNOWN)

    def test_stats_keys(self):
        m = _mk()
        stats = apply_cloud(m, (0, 0), np.zeros((0, 3)), z_floor=0.0, now_ns=1)
        self.assertEqual(set(stats), {"floor_pts", "obstacle_pts", "drop_pts",
                                      "updated_occ", "updated_free"})


class TestFreeClearance(unittest.TestCase):
    def _room(self, wall_x=2.0):
        m = _mk()
        pts = np.vstack([_floor_patch(-1.0, wall_x - 0.05, -2.0, 2.0, z=0.0),
                         _wall(wall_x, -2.0, 2.0)])
        apply_cloud(m, (0, 0), pts, z_floor=0.0, now_ns=1)
        return m

    def test_clearance_stops_before_wall(self):
        m = self._room(wall_x=2.0)
        d = free_clearance(m, 0.0, 0.0, 0.0, max_m=5.0, inflate_cells=2)
        self.assertGreater(d, 0.8)
        self.assertLess(d, 2.0)   # inflate 分手前で止まる

    def test_unknown_not_counted(self):
        m = _mk()
        # 床を 1m 分だけ観測 → その先は unknown → clearance はそこまで
        apply_cloud(m, (0, 0), _floor_patch(0.1, 1.0, -0.3, 0.3, z=0.0),
                    z_floor=0.0, now_ns=1)
        d = free_clearance(m, 0.0, 0.0, 0.0, max_m=5.0, inflate_cells=0)
        self.assertLess(d, 1.2)

    def test_inflation_reduces_clearance(self):
        m = self._room(wall_x=2.0)
        d0 = free_clearance(m, 0.0, 0.0, 0.0, max_m=5.0, inflate_cells=0)
        d6 = free_clearance(m, 0.0, 0.0, 0.0, max_m=5.0, inflate_cells=6)
        self.assertGreater(d0, d6)

    def test_direction_matters(self):
        m = self._room(wall_x=1.0)
        fwd = free_clearance(m, 0.0, 0.0, 0.0, max_m=5.0, inflate_cells=0)
        back = free_clearance(m, 0.0, 0.0, math.pi, max_m=5.0, inflate_cells=0)
        self.assertLess(fwd, back + 0.01)

    def test_optimistic_counts_unknown(self):
        # 床を 1m 分だけ観測。既定は unknown で止まるが、optimistic は
        # 未踏域を数える(探索計画専用 — 操作者要望 2026-07-17)
        m = _mk()
        apply_cloud(m, (0, 0), _floor_patch(0.1, 1.0, -0.3, 0.3, z=0.0),
                    z_floor=0.0, now_ns=1)
        d_def = free_clearance(m, 0.0, 0.0, 0.0, max_m=5.0, inflate_cells=0)
        d_opt = free_clearance(m, 0.0, 0.0, 0.0, max_m=5.0, inflate_cells=0,
                               optimistic=True)
        self.assertLess(d_def, 1.2)
        self.assertAlmostEqual(d_opt, 5.0, delta=0.1)

    def test_optimistic_still_stops_before_wall(self):
        # optimistic でも観測済み障害物(+inflate)では従来どおり止まる
        m = self._room(wall_x=2.0)
        d = free_clearance(m, 0.0, 0.0, 0.0, max_m=5.0, inflate_cells=2,
                           optimistic=True)
        self.assertLess(d, 2.0)

    def test_optimistic_still_stops_at_hazard(self):
        # drop hazard(OCCUPIED 化)も optimistic の通行対象にならない
        m = _mk()
        apply_cloud(m, (0, 0), _floor_patch(0.1, 0.6, -0.3, 0.3, z=0.0),
                    z_floor=0.0, now_ns=1)
        m.mark_hazard([(1.0, y) for y in np.arange(-0.3, 0.31, 0.1)], now_ns=2)
        d = free_clearance(m, 0.0, 0.0, 0.0, max_m=5.0, inflate_cells=0,
                           optimistic=True)
        self.assertLess(d, 1.0)


if __name__ == "__main__":
    unittest.main()
