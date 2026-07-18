"""cloud_projector — 実LiDAR点群(3D, odom系) → GlobalOccupancyMap 更新(docs/12 §3)。

cockpit が受けている `rt/utlidar/cloud_deskewed`(odom系 xyz)と pose から、
2D occupancy の更新材料を作る純ロジック層。実行許可・速度生成は持たない。

z 帯分類(robot 足元 z_floor 基準, docs/12 §3):
  - 床帯   [Z_FLOOR_LO, Z_FLOOR_HI]  → FREE 証拠(integrate_free_rays)
  - 障害物帯 (Z_FLOOR_HI, Z_OBST_HI] → OCCUPIED 終端(integrate_scan)
  - 頭上帯  > Z_OBST_HI              → 無視(Go2 は下を通れる)
  - 落差    < Z_DROP                 → drop hazard(mark_hazard, OCCUPIED 扱い)

原則:
  - raycast なしで FREE を置かない(integrate系がrayを引く)。
  - 分類できない点は捨てる(fail-closed。unknown ≠ free)。
  - cell 単位で重複除去してから統合する(Python raycast の点数を抑える)。
"""
from __future__ import annotations

import math
from typing import NamedTuple

import numpy as np

from perception.global_map import FREE, OCCUPIED, GlobalOccupancyMap

# z 帯しきい値 [m](z_floor 相対)。安全側以外の変更は根拠と test を伴うこと。
Z_FLOOR_LO = -0.08
Z_FLOOR_HI = 0.10
Z_OBST_HI = 0.60
Z_DROP = -0.15

MAP_MAX_RANGE_M = 6.0    # mapping に使う最大距離(遠方はノイズ・drift が大きい)
MAX_OBST_PTS = 1500      # raycast 負荷上限(cell 重複除去後にさらに間引く)
MAX_FLOOR_PTS = 2000
MAX_DROP_PTS = 400
SELF_RADIUS_M = 0.45     # ロボット自身(脚・機体)の反射を障害物/落差にしない半径
                         # (実機 2026-07-18: home から 0.43m に自己点由来の
                         # OCCUPIED が現れ、至近閉塞の一因になっていた)
FOOTPRINT_R_M = 0.30     # ロボットが物理的に立っている円盤 — 最強の通行可能証拠


class ClassifiedCloud(NamedTuple):
    """z帯分類+cell重複除去済みの 2D 点群(いずれも float64 (N,2))。"""
    floor_xy: np.ndarray
    obstacle_xy: np.ndarray
    drop_xy: np.ndarray


def _dedup_by_cell(pts_xy: np.ndarray, resolution_m: float,
                   cap: int) -> np.ndarray:
    """同一 cell に落ちる点を1つに間引き、cap 超は等間隔サンプリング。"""
    if len(pts_xy) == 0:
        return pts_xy.reshape(0, 2)
    cells = np.floor(pts_xy / resolution_m).astype(np.int64)
    _, idx = np.unique(cells, axis=0, return_index=True)
    out = pts_xy[np.sort(idx)]
    if len(out) > cap:
        out = out[np.linspace(0, len(out) - 1, cap).astype(int)]
    return out


def classify_cloud(points_xyz, robot_xy, z_floor: float,
                   resolution_m: float,
                   max_range_m: float = MAP_MAX_RANGE_M,
                   self_radius_m: float = SELF_RADIUS_M) -> ClassifiedCloud:
    """odom系 3D 点群を z 帯で分類し、cell 重複除去した 2D 点群を返す。

    points_xyz: (N,3) 配列。robot_xy: ロボット world 位置。
    z_floor: ロボット足元の床 z(odom系)。呼び出し側が pose.z - body_height 等で
    推定して注入する(本関数は状態を持たない)。
    self_radius_m 以内の点は自己反射(脚・機体)とみなし、障害物/落差には
    使わない(床=FREE 証拠としては使う)。
    """
    pts = np.asarray(points_xyz, dtype=np.float64).reshape(-1, 3)
    if len(pts) == 0:
        empty = np.zeros((0, 2))
        return ClassifiedCloud(empty, empty, empty)
    finite = np.isfinite(pts).all(axis=1)
    pts = pts[finite]
    dz = pts[:, 2] - float(z_floor)
    dist = np.hypot(pts[:, 0] - float(robot_xy[0]),
                    pts[:, 1] - float(robot_xy[1]))
    in_range = dist <= max_range_m
    not_self = dist > self_radius_m

    floor = pts[in_range & (dz >= Z_FLOOR_LO) & (dz <= Z_FLOOR_HI)][:, :2]
    obst = pts[in_range & not_self & (dz > Z_FLOOR_HI) & (dz <= Z_OBST_HI)][:, :2]
    drop = pts[in_range & not_self & (dz < Z_DROP)][:, :2]
    # 頭上帯(dz > Z_OBST_HI)と分類外は捨てる(fail-closed)

    return ClassifiedCloud(
        floor_xy=_dedup_by_cell(floor, resolution_m, MAX_FLOOR_PTS),
        obstacle_xy=_dedup_by_cell(obst, resolution_m, MAX_OBST_PTS),
        drop_xy=_dedup_by_cell(drop, resolution_m, MAX_DROP_PTS),
    )


def apply_cloud(gmap: GlobalOccupancyMap, robot_xy, points_xyz,
                z_floor: float, now_ns: int,
                max_range_m: float = MAP_MAX_RANGE_M) -> dict:
    """点群1フレームを地図へ統合する。戻り値は統計 dict(ログ用)。

    floor/obstacleを同一scanとして一度に統合し、同じcellではhitを優先する。
    drop hazardはray/footprintでは解除せず、明示的map resetまで保持する。
    """
    cc = classify_cloud(points_xyz, robot_xy, z_floor,
                        gmap.resolution_m, max_range_m)
    before = gmap.grid.copy()
    points = np.vstack((cc.floor_xy, cc.obstacle_xy))
    hit_mask = np.concatenate((
        np.zeros(len(cc.floor_xy), dtype=bool),
        np.ones(len(cc.obstacle_xy), dtype=bool),
    ))
    gmap.integrate_scan(robot_xy, points, now_ns,
                        max_range_m=max_range_m, hit_mask=hit_mask)
    gmap.mark_hazard(cc.drop_xy, now_ns)
    gmap.clear_footprint(robot_xy, FOOTPRINT_R_M, now_ns)
    after = gmap.grid
    n_occ = int(np.count_nonzero((before != OCCUPIED) & (after == OCCUPIED)))
    n_free = int(np.count_nonzero((before != FREE) & (after == FREE)))
    return {"floor_pts": int(len(cc.floor_xy)),
            "obstacle_pts": int(len(cc.obstacle_xy)),
            "drop_pts": int(len(cc.drop_xy)),
            "updated_occ": int(n_occ), "updated_free": int(n_free)}


def clearance_multi(gmap: GlobalOccupancyMap, x: float, y: float,
                    headings, max_m: float = 2.0, inflate_cells: int = 3,
                    optimistic: bool = False):
    """複数方向の前方クリアランスを一括計算(traversable_mask を1回だけ構築)。

    ルンバ風スムーズ操舵(2026-07-18)用: 毎tick 9方向を測るため、
    free_clearance を方向数だけ呼ぶと mask 構築が支配的になる。
    戻り値: headings と同順のクリアランス [m] のリスト。"""
    # optimistic引数は旧API互換で受理するが、移動clearanceは常にFREE-only。
    trav = gmap.traversable_mask(inflate_cells=inflate_cells)
    step = gmap.resolution_m * 0.5
    n = int(max_m / step)
    out = []
    for hyaw in headings:
        cx, cy = math.cos(hyaw), math.sin(hyaw)
        dist = 0.0
        for i in range(1, n + 1):
            d = i * step
            c = gmap.world_to_cell(x + cx * d, y + cy * d)
            if c is None or not trav[c[1], c[0]]:
                break
            dist = d
        out.append(dist)
    return out


def free_clearance(gmap: GlobalOccupancyMap, x: float, y: float, yaw: float,
                   max_m: float = 3.0, inflate_cells: int = 6,
                   optimistic: bool = False) -> float:
    """進行方向 yaw に沿った通行可能(非inflate)距離 [m] を返す。

    waypoint_follower の front_clearance 入力。
    - optimistic=False(既定): FREE のみ数える。unknown はクリアランスに
      数えない(invariant 9)。
    - optimistic=Trueは旧API互換。安全上、結果は既定と同じFREE-only。
    ロボット自身の cell が通行不可でも、直前までの距離(=0)を返すだけで
    例外にはしない。
    """
    trav = gmap.traversable_mask(inflate_cells=inflate_cells)
    step = gmap.resolution_m * 0.5
    n = int(max_m / step)
    cx, cy = math.cos(yaw), math.sin(yaw)
    dist = 0.0
    for i in range(1, n + 1):
        d = i * step
        c = gmap.world_to_cell(x + cx * d, y + cy * d)
        if c is None or not trav[c[1], c[0]]:
            return dist
        dist = d
    return dist
