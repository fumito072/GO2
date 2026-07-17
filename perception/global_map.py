"""perception.global_map — 2D global occupancy map(docs/10 §4)。

自律探索+マップ構築(EXPLORE_AND_MAP)の中核データ。純ロジック・時刻注入で
synthetic 点群による offline test が可能(docs/10 §6 E0)。

原則:
  - UNKNOWN != FREE(invariant 9)。通行判定(traversable_mask)は FREE のみ。
  - 各 cell に最終更新時刻(monotonic ns)を持ち、鮮度を消費側が評価できる。
  - 座標系は odom(loop closure 導入まで)。drift の扱いは docs/10 §4。
  - 本クラスは地図の構築・照会のみ。goal 生成は navigation 層、
    実行許可は Mission/safety 層の責務。
"""
import math

import numpy as np

from contracts.errors import ContractViolation

UNKNOWN = 0
FREE = 1
OCCUPIED = 2

# 証拠カウンタ(2026-07-17: 動的障害物対策)。OCCUPIED cell はヒットで加点・
# ray 通過(=そこは空いていた証拠)で減点し、尽きたら FREE へ降格する。
# 実在の壁は観測のたび再加点されるので維持され、人の脚などの動的障害物や
# ノイズは数フレームで消える(地図の自己修正)。
OCC_HIT = 2        # 障害物ヒット1回の加点
OCC_MAX = 6        # 加点上限(降格には最低3回の通過証拠が必要)
HAZARD_SCORE = 100 # drop hazard 用。50 超は減衰・降格させない(安全側で永続)

SCHEMA_VERSION = "1.0"
OCCUPIED_CLEAR_MISSES = 3


def _bresenham(x0: int, y0: int, x1: int, y1: int):
    """整数格子上の線分 cell 列(端点含む)。"""
    cells = []
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return cells


class GlobalOccupancyMap:
    """固定サイズの 2D occupancy grid(初期 MVP)。

    grid[iy, ix] ∈ {UNKNOWN, FREE, OCCUPIED}、age_ns[iy, ix] = 最終更新時刻
    (0 = 未観測)。
    """

    def __init__(self, size_m=(20.0, 20.0), resolution_m=0.05,
                 origin_xy=(-10.0, -10.0), map_id="map_default",
                 frame_id="odom"):
        if resolution_m <= 0 or size_m[0] <= 0 or size_m[1] <= 0:
            raise ContractViolation("global_map", "size/resolution は正の値")
        self.map_id = map_id
        self.frame_id = frame_id
        self.resolution_m = float(resolution_m)
        self.origin_xy = (float(origin_xy[0]), float(origin_xy[1]))
        self.width = int(round(size_m[0] / resolution_m))
        self.height = int(round(size_m[1] / resolution_m))
        self.grid = np.full((self.height, self.width), UNKNOWN, dtype=np.uint8)
        self.age_ns = np.zeros((self.height, self.width), dtype=np.int64)
        # 動体/noiseのghost obstacleを永久化しない。1 scan内のray本数ではなく
        # 独立scanごとのfree evidenceを数え、3回連続でのみ解除する。
        self.free_evidence = np.zeros((self.height, self.width), dtype=np.uint8)
        # drop/段差などsemantic hazardはLiDAR missで解除しない別layer。
        self.hazard_mask = np.zeros((self.height, self.width), dtype=bool)
        # 旧artifact/診断UIとの互換値。解除判定はocc_scoreではなく、上記の
        # scan単位free_evidenceを唯一の根拠にする。
        self.occ_score = np.zeros((self.height, self.width), dtype=np.int16)
        self.waypoints = {}  # name -> (x, y, yaw)
        self.revision = 0    # occupancy/free state が変化した回数(path invalidation用)

    # ---------- 証拠カウンタ(cell 単位のヒット/ミス) ----------

    def _ray_hit(self, ix: int, iy: int, now_ns: int) -> int:
        """互換用の単cell hit。通常はscan単位のintegrate_scanを使う。"""
        s = self.occ_score[iy, ix]
        if not self.hazard_mask[iy, ix]:
            self.occ_score[iy, ix] = min(OCC_MAX, max(int(s), 0) + OCC_HIT)
        changed = int(self.grid[iy, ix] != OCCUPIED)
        self.grid[iy, ix] = OCCUPIED
        self.age_ns[iy, ix] = now_ns
        self.free_evidence[iy, ix] = 0
        return changed

    def _ray_miss(self, ix: int, iy: int, now_ns: int) -> int:
        """互換用の単cell miss。呼出し1回を独立scan 1回として数える。"""
        if self.hazard_mask[iy, ix]:
            return 0
        if self.grid[iy, ix] == OCCUPIED:
            evidence = min(255, int(self.free_evidence[iy, ix]) + 1)
            self.free_evidence[iy, ix] = evidence
            self.age_ns[iy, ix] = now_ns
            if evidence >= OCCUPIED_CLEAR_MISSES:
                self.grid[iy, ix] = FREE
                self.occ_score[iy, ix] = 0
                self.free_evidence[iy, ix] = 0
                return 1
            return 0
        changed = int(self.grid[iy, ix] != FREE)
        self.grid[iy, ix] = FREE
        self.age_ns[iy, ix] = now_ns
        self.occ_score[iy, ix] = 0
        self.free_evidence[iy, ix] = 0
        return changed

    # ---------- 座標変換 ----------

    def world_to_cell(self, x: float, y: float):
        # int() は 0 方向への切り捨てのため origin 直下の負座標が cell 0 に
        # 誤マップされる — floor を使う(机上検証での指摘)
        ix = int(np.floor((x - self.origin_xy[0]) / self.resolution_m))
        iy = int(np.floor((y - self.origin_xy[1]) / self.resolution_m))
        if 0 <= ix < self.width and 0 <= iy < self.height:
            return ix, iy
        return None

    def cell_to_world(self, ix: int, iy: int):
        return (self.origin_xy[0] + (ix + 0.5) * self.resolution_m,
                self.origin_xy[1] + (iy + 0.5) * self.resolution_m)

    # ---------- 更新 ----------

    def _clip_endpoint_to_bounds(self, rx: float, ry: float,
                                 px: float, py: float):
        """robot→endpoint を map AABB 内へclipする。

        endpoint が地図外でも、境界までの観測済み FREE ray を捨てない。
        戻り値は map 内側へ僅かに寄せた world座標、交差しなければ None。
        robot 自体がmap内であることは caller が保証する。
        """
        xmin, ymin = self.origin_xy
        # world_to_cell の上端はexclusiveなので epsilonだけ内側へ寄せる。
        eps = max(1e-9, self.resolution_m * 1e-6)
        xmax = xmin + self.width * self.resolution_m - eps
        ymax = ymin + self.height * self.resolution_m - eps
        dx, dy = px - rx, py - ry
        t_exit = 1.0
        for origin, delta, lo, hi in ((rx, dx, xmin, xmax),
                                      (ry, dy, ymin, ymax)):
            if abs(delta) < 1e-15:
                if origin < lo or origin > hi:
                    return None
                continue
            boundary = hi if delta > 0 else lo
            t_exit = min(t_exit, (boundary - origin) / delta)
        if t_exit < 0.0:
            return None
        t = max(0.0, min(1.0, t_exit))
        return rx + dx * t, ry + dy * t

    def integrate_scan(self, robot_xy, points_xy, now_ns: int,
                       max_range_m: float = 8.0, hit_mask=None) -> int:
        """robot 位置から見えた障害物点群を統合する。

        - robot→点 の ray 上の cell を FREE、端点 cell を OCCUPIED にする。
        - max_range 超の点は max_range まで FREE のみ(端点は付けない)。
        - hit_mask=False の点は free-space return として端点まで FREE にする。
          PointCloud2のground returnを障害物にしないadapterで使用する。
        - 戻り値: 更新した cell 数。
        """
        if not isinstance(now_ns, int) or isinstance(now_ns, bool) or now_ns <= 0:
            raise ContractViolation("now_ns", "正の monotonic ns が必要")
        rc = self.world_to_cell(float(robot_xy[0]), float(robot_xy[1]))
        if rc is None:
            raise ContractViolation("robot_xy", "地図範囲外: %r" % (robot_xy,))
        points = list(points_xy)
        if hit_mask is None:
            hits = [True] * len(points)
        else:
            hits = list(hit_mask)
            if len(hits) != len(points):
                raise ContractViolation("hit_mask", "points_xy と同じ長さが必要")
            if any(not isinstance(v, (bool, np.bool_)) for v in hits):
                raise ContractViolation("hit_mask", "bool列が必要")
        updated = 0
        free_seen = set()
        hit_seen = set()
        rx, ry = float(robot_xy[0]), float(robot_xy[1])
        for p, reported_hit in zip(points, hits):
            px, py = float(p[0]), float(p[1])
            if not (np.isfinite(px) and np.isfinite(py)):
                continue  # 不正点は無視(FREE にも OCCUPIED にもしない)
            d = float(np.hypot(px - rx, py - ry))
            if d <= 1e-12:
                continue
            hit = bool(reported_hit) and d <= max_range_m
            if not hit:
                # no-return/ground return。max_range超だけ距離を切り詰める。
                if d > max_range_m:
                    scale = max_range_m / d
                    px = rx + (px - rx) * scale
                    py = ry + (py - ry) * scale
            pc = self.world_to_cell(px, py)
            if pc is None:
                clipped = self._clip_endpoint_to_bounds(rx, ry, px, py)
                if clipped is None:
                    continue
                pc = self.world_to_cell(*clipped)
                if pc is None:
                    continue
                hit = False  # map外の実障害物を境界cellの障害物にしない
            if pc == rc:
                continue  # robot 自身の cell 上の点は退化 ray(robot cell を OCCUPIED にしない)
            ray = _bresenham(rc[0], rc[1], pc[0], pc[1])
            free_cells = ray[:-1] if hit else ray
            free_seen.update(free_cells)
            if hit:
                hit_seen.add(ray[-1])

        # 同じscan内でendpoint hitもあるcellはhitを優先する。多数のfloor rayが
        # 1 callback内で同じcellを横切ってもfree evidenceは1回しか増えない。
        free_seen.difference_update(hit_seen)
        for ix, iy in sorted(free_seen, key=lambda c: (c[1], c[0])):
            if self.hazard_mask[iy, ix]:
                continue
            if self.grid[iy, ix] == OCCUPIED:
                evidence = min(255, int(self.free_evidence[iy, ix]) + 1)
                self.free_evidence[iy, ix] = evidence
                if evidence >= OCCUPIED_CLEAR_MISSES:
                    self.grid[iy, ix] = FREE
                    self.age_ns[iy, ix] = now_ns
                    self.occ_score[iy, ix] = 0
                    self.free_evidence[iy, ix] = 0
                    updated += 1
            else:
                if self.grid[iy, ix] != FREE:
                    updated += 1
                self.grid[iy, ix] = FREE
                self.age_ns[iy, ix] = now_ns
                self.occ_score[iy, ix] = 0
                self.free_evidence[iy, ix] = 0
        for ix, iy in sorted(hit_seen, key=lambda c: (c[1], c[0])):
            if self.grid[iy, ix] != OCCUPIED:
                updated += 1
            self.grid[iy, ix] = OCCUPIED
            self.age_ns[iy, ix] = now_ns
            if not self.hazard_mask[iy, ix]:
                self.occ_score[iy, ix] = min(
                    OCC_MAX, max(0, int(self.occ_score[iy, ix])) + OCC_HIT)
            self.free_evidence[iy, ix] = 0
        # robot cellの通常のself-returnは解除する。ただしdrop/段差hazardを
        # footprintだけで消すと、pose driftや誤分類時に落下側へ倒れるため保持する。
        if not self.hazard_mask[rc[1], rc[0]]:
            if self.grid[rc[1], rc[0]] != FREE:
                updated += 1
            self.grid[rc[1], rc[0]] = FREE
            self.age_ns[rc[1], rc[0]] = now_ns
            self.occ_score[rc[1], rc[0]] = 0
            self.free_evidence[rc[1], rc[0]] = 0
        if updated:
            self.revision += 1
        return updated

    def integrate_free_rays(self, robot_xy, points_xy, now_ns: int,
                            max_range_m: float = 8.0) -> int:
        """床return/no-returnを端点までFREEとしてscan単位で統合する。

        同じcallback内で何本のrayが同一cellを通っても解除証拠は1回分だけ。
        map外endpointは境界までのFREE観測を保持する。
        """
        points = list(points_xy)
        return self.integrate_scan(robot_xy, points, now_ns,
                                   max_range_m=max_range_m,
                                   hit_mask=[False] * len(points))

    def integrate_point_cloud(self, robot_pose, points_xyz, now_ns: int,
                              max_range_m: float = 8.0,
                              nominal_base_height_m: float = 0.31,
                              min_obstacle_height_m: float = 0.04,
                              max_obstacle_height_m: float = 1.50,
                              floor_tolerance_m: float = 0.10,
                              max_points: int = 4000) -> int:
        """odom系3D点群を2D occupancyへ安全に投影するadapter。

        ground付近のreturnは端点までFREE、groundより高いreturnはOCCUPIED。
        天井/極端な低点は無視する。入力frameとtimestamp同期の検証はI/O層の責務。
        """
        if not isinstance(now_ns, int) or isinstance(now_ns, bool) or now_ns <= 0:
            raise ContractViolation("now_ns", "正の monotonic ns が必要")
        if len(robot_pose) < 3:
            raise ContractViolation("robot_pose", "(x,y,z[,yaw]) が必要")
        pose = np.asarray(robot_pose[:3], dtype=np.float64)
        if not np.isfinite(pose).all():
            raise ContractViolation("robot_pose", "有限値が必要")
        pts = np.asarray(points_xyz, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[1] < 3:
            raise ContractViolation("points_xyz", "shape (N,3+) が必要")
        if max_points <= 0:
            raise ContractViolation("max_points", "正の値が必要")
        if pts.shape[0] == 0:
            return self.integrate_scan(tuple(pose[:2]), [], now_ns,
                                       max_range_m=max_range_m)
        pts = pts[:, :3]
        pts = pts[np.isfinite(pts).all(axis=1)]
        if pts.shape[0] > max_points:
            # 入力順へ偏らない決定的な等間隔sampling。
            idx = np.linspace(0, pts.shape[0] - 1, max_points, dtype=np.int64)
            pts = pts[idx]
        ground_z = float(pose[2] - nominal_base_height_m)
        height = pts[:, 2] - ground_z
        dist = np.hypot(pts[:, 0] - pose[0], pts[:, 1] - pose[1])
        relevant = (dist > 0.12) & (dist <= max_range_m)
        relevant &= height >= -abs(floor_tolerance_m)
        relevant &= height <= max_obstacle_height_m
        pts = pts[relevant]
        height = height[relevant]
        if not len(pts):
            return self.integrate_scan(tuple(pose[:2]), [], now_ns,
                                       max_range_m=max_range_m)
        hits = height >= min_obstacle_height_m
        return self.integrate_scan(tuple(pose[:2]), pts[:, :2], now_ns,
                                   max_range_m=max_range_m, hit_mask=hits)

    def mark_hazard(self, points_xy, now_ns: int) -> None:
        """elevation 分類(step/drop 等)による進入禁止 cell を OCCUPIED にする
        (docs/10 §5: drop/段差は costmap 上 OCCUPIED 扱い)。
        hazard は証拠カウンタの減衰対象外(ray が通過しても消えない — 安全側)。"""
        if not isinstance(now_ns, int) or isinstance(now_ns, bool) or now_ns <= 0:
            raise ContractViolation("now_ns", "正の monotonic ns が必要")
        changed = False
        for p in points_xy:
            c = self.world_to_cell(float(p[0]), float(p[1]))
            if c is not None:
                changed = changed or self.grid[c[1], c[0]] != OCCUPIED
                self.grid[c[1], c[0]] = OCCUPIED
                self.occ_score[c[1], c[0]] = HAZARD_SCORE
                self.age_ns[c[1], c[0]] = now_ns
                self.free_evidence[c[1], c[0]] = 0
                self.hazard_mask[c[1], c[0]] = True
        if changed:
            self.revision += 1

    def clear_footprint(self, robot_xy, radius_m: float, now_ns: int) -> None:
        """robot が物理的に立っている円盤を FREE にする(2026-07-18)。

        通常OCCUPIEDだけを浄化する。semantic hazardはpose drift時の落下防止の
        ため保持し、再分類または明示的なmap resetでのみ解除する。"""
        if not isinstance(now_ns, int) or isinstance(now_ns, bool) or now_ns <= 0:
            raise ContractViolation("now_ns", "正の monotonic ns が必要")
        rc = self.world_to_cell(float(robot_xy[0]), float(robot_xy[1]))
        if rc is None:
            return
        r = int(radius_m / self.resolution_m)
        changed = False
        for dj in range(-r, r + 1):
            for di in range(-r, r + 1):
                if di * di + dj * dj > r * r:
                    continue
                ix, iy = rc[0] + di, rc[1] + dj
                if 0 <= ix < self.width and 0 <= iy < self.height:
                    if self.hazard_mask[iy, ix]:
                        continue
                    changed = changed or self.grid[iy, ix] != FREE
                    self.grid[iy, ix] = FREE
                    self.occ_score[iy, ix] = 0
                    self.free_evidence[iy, ix] = 0
                    self.age_ns[iy, ix] = now_ns
        if changed:
            self.revision += 1

    def set_waypoint(self, name: str, pose_xyyaw) -> None:
        if not name or not isinstance(name, str):
            raise ContractViolation("waypoint.name", "非空の str が必要")
        x, y, yaw = (float(pose_xyyaw[0]), float(pose_xyyaw[1]),
                     float(pose_xyyaw[2]))
        if not all(np.isfinite(v) for v in (x, y, yaw)):
            raise ContractViolation("waypoint.pose", "有限値が必要")
        self.waypoints[name] = (x, y, yaw)

    # ---------- 照会 ----------

    def counts(self) -> dict:
        return {"unknown": int(np.sum(self.grid == UNKNOWN)),
                "free": int(np.sum(self.grid == FREE)),
                "occupied": int(np.sum(self.grid == OCCUPIED))}

    def traversable_mask(self, inflate_cells: int = 3,
                         inflation_radius_m=None,
                         now_ns=None, max_age_ns=None,
                         optimistic: bool = False) -> np.ndarray:
        """通行可能 = FREE のみ(UNKNOWN は通行不可 — invariant 9)。
        OCCUPIED はrobot footprint相当を膨張させる。

        inflation_radius_mを指定した場合はresolutionに依存しない物理marginを使う。
        now_ns/max_age_nsを指定した場合、古いFREEはUNKNOWN相当として通行不可。
        optimistic=Trueは地図可視化/旧API互換の照会専用でUNKNOWNも含める。
        LIVE経路計画では使用してはならず、freshness指定時は常にFREEのみとする。
        """
        if inflation_radius_m is not None:
            if not np.isfinite(inflation_radius_m) or inflation_radius_m < 0:
                raise ContractViolation("inflation_radius_m", "0以上の有限値が必要")
            inflate_cells = int(math.ceil(inflation_radius_m / self.resolution_m))
        if not isinstance(inflate_cells, int) or isinstance(inflate_cells, bool) \
                or inflate_cells < 0:
            raise ContractViolation("inflate_cells", "0以上のintが必要")
        if (now_ns is None) != (max_age_ns is None):
            raise ContractViolation("freshness", "now_nsとmax_age_nsは同時指定")
        if not isinstance(optimistic, (bool, np.bool_)):
            raise ContractViolation("optimistic", "boolが必要")
        occ = (self.grid == OCCUPIED)
        if inflate_cells > 0:
            # Euclidean disk。従来の4近傍diamondより対角footprintを保護する。
            inflated = np.zeros_like(occ)
            h, w = occ.shape
            for dy in range(-inflate_cells, inflate_cells + 1):
                for dx in range(-inflate_cells, inflate_cells + 1):
                    if dx * dx + dy * dy > inflate_cells * inflate_cells:
                        continue
                    sy0, sy1 = max(0, -dy), min(h, h - dy)
                    sx0, sx1 = max(0, -dx), min(w, w - dx)
                    dy0, dy1 = sy0 + dy, sy1 + dy
                    dx0, dx1 = sx0 + dx, sx1 + dx
                    inflated[dy0:dy1, dx0:dx1] |= occ[sy0:sy1, sx0:sx1]
            occ = inflated
        free = ((self.grid != OCCUPIED) if optimistic and now_ns is None
                else (self.grid == FREE))
        if now_ns is not None:
            if not isinstance(now_ns, int) or isinstance(now_ns, bool) or now_ns <= 0:
                raise ContractViolation("now_ns", "正のmonotonic nsが必要")
            if not isinstance(max_age_ns, int) or isinstance(max_age_ns, bool) \
                    or max_age_ns <= 0:
                raise ContractViolation("max_age_ns", "正のintが必要")
            age = now_ns - self.age_ns
            free &= (self.age_ns > 0) & (age >= 0) & (age <= max_age_ns)
        return free & (~occ)

    # ---------- 保存 / 読込(docs/09 §4 layout: artifacts/maps/<map_id>/) ----------

    def to_dict(self) -> dict:
        return {
            "schema_version": SCHEMA_VERSION,
            "map_id": self.map_id,
            "frame_id": self.frame_id,
            "resolution_m": self.resolution_m,
            "origin": list(self.origin_xy),
            "size": [self.width, self.height],
            "cells": self.grid.tolist(),
            "age_ns": self.age_ns.tolist(),
            "free_evidence": self.free_evidence.tolist(),
            "hazard_mask": self.hazard_mask.tolist(),
            "occ_score": self.occ_score.tolist(),
            "waypoints": {k: list(v) for k, v in self.waypoints.items()},
            "revision": self.revision,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "GlobalOccupancyMap":
        if d.get("schema_version") != SCHEMA_VERSION:
            raise ContractViolation("schema_version",
                                    "未対応 version: %r" % (d.get("schema_version"),))
        w, h = int(d["size"][0]), int(d["size"][1])
        m = cls(size_m=(w * d["resolution_m"], h * d["resolution_m"]),
                resolution_m=d["resolution_m"],
                origin_xy=tuple(d["origin"]),
                map_id=d["map_id"], frame_id=d["frame_id"])
        grid = np.asarray(d["cells"], dtype=np.uint8)
        if grid.shape != (h, w):
            raise ContractViolation("cells", "shape 不一致: %r" % (grid.shape,))
        if not np.isin(grid, (UNKNOWN, FREE, OCCUPIED)).all():
            raise ContractViolation("cells", "未知の cell 値(fail-closed)")
        m.grid = grid
        age = np.asarray(d["age_ns"], dtype=np.int64)
        if age.shape != (h, w):
            raise ContractViolation("age_ns", "shape 不一致")
        if (age < 0).any():
            raise ContractViolation("age_ns", "負の時刻を拒否")
        m.age_ns = age
        free_evidence = np.asarray(
            d.get("free_evidence", np.zeros((h, w), dtype=np.uint8)),
            dtype=np.uint8)
        if free_evidence.shape != (h, w):
            raise ContractViolation("free_evidence", "shape 不一致")
        m.free_evidence = free_evidence
        hazard_mask = np.asarray(
            d.get("hazard_mask", np.zeros((h, w), dtype=bool)), dtype=bool)
        if hazard_mask.shape != (h, w):
            raise ContractViolation("hazard_mask", "shape 不一致")
        if np.any(hazard_mask & (grid != OCCUPIED)):
            raise ContractViolation("hazard_mask", "hazard cellはOCCUPIED必須")
        m.hazard_mask = hazard_mask
        default_score = np.where(
            hazard_mask, HAZARD_SCORE,
            np.where(grid == OCCUPIED, OCC_MAX, 0)).astype(np.int16)
        occ_score = np.asarray(d.get("occ_score", default_score), dtype=np.int16)
        if occ_score.shape != (h, w) or (occ_score < 0).any():
            raise ContractViolation("occ_score", "shape不一致または負値")
        occ_score[hazard_mask] = HAZARD_SCORE
        m.occ_score = occ_score
        revision = d.get("revision", 0)
        if not isinstance(revision, int) or isinstance(revision, bool) or revision < 0:
            raise ContractViolation("revision", "0以上のintが必要")
        m.revision = revision
        for k, v in d.get("waypoints", {}).items():
            m.set_waypoint(k, v)
        return m
