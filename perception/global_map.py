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
        self.occ_score = np.zeros((self.height, self.width), dtype=np.int16)
        self.waypoints = {}  # name -> (x, y, yaw)

    # ---------- 証拠カウンタ(cell 単位のヒット/ミス) ----------

    def _ray_hit(self, ix: int, iy: int, now_ns: int) -> int:
        """障害物ヒット: 加点して OCCUPIED にする。戻り値=状態が変わったか。"""
        s = self.occ_score[iy, ix]
        if s <= 50:  # hazard(>50)のスコアは触らない
            self.occ_score[iy, ix] = min(OCC_MAX, max(int(s), 0) + OCC_HIT)
        changed = int(self.grid[iy, ix] != OCCUPIED)
        self.grid[iy, ix] = OCCUPIED
        self.age_ns[iy, ix] = now_ns
        return changed

    def _ray_miss(self, ix: int, iy: int, now_ns: int) -> int:
        """ray 通過(FREE 証拠): OCCUPIED は減点し、尽きたら FREE へ降格。
        hazard(score>50)は降格しない。戻り値=状態が変わったか。"""
        if self.grid[iy, ix] == OCCUPIED:
            s = int(self.occ_score[iy, ix])
            if s > 50:
                return 0
            s -= 1
            self.occ_score[iy, ix] = s
            self.age_ns[iy, ix] = now_ns
            if s <= 0:
                self.grid[iy, ix] = FREE
                return 1
            return 0
        changed = int(self.grid[iy, ix] != FREE)
        self.grid[iy, ix] = FREE
        self.age_ns[iy, ix] = now_ns
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

    def integrate_scan(self, robot_xy, points_xy, now_ns: int,
                       max_range_m: float = 8.0) -> int:
        """robot 位置から見えた障害物点群を統合する。

        - robot→点 の ray 上の cell を FREE、端点 cell を OCCUPIED にする。
        - max_range 超の点は max_range まで FREE のみ(端点は付けない)。
        - 戻り値: 更新した cell 数。
        """
        if not isinstance(now_ns, int) or isinstance(now_ns, bool) or now_ns <= 0:
            raise ContractViolation("now_ns", "正の monotonic ns が必要")
        rc = self.world_to_cell(float(robot_xy[0]), float(robot_xy[1]))
        if rc is None:
            raise ContractViolation("robot_xy", "地図範囲外: %r" % (robot_xy,))
        updated = 0
        rx, ry = float(robot_xy[0]), float(robot_xy[1])
        for p in points_xy:
            px, py = float(p[0]), float(p[1])
            if not (np.isfinite(px) and np.isfinite(py)):
                continue  # 不正点は無視(FREE にも OCCUPIED にもしない)
            d = float(np.hypot(px - rx, py - ry))
            hit = d <= max_range_m
            if not hit:
                # max_range で切り詰め(その先は unknown のまま)
                scale = max_range_m / d
                px = rx + (px - rx) * scale
                py = ry + (py - ry) * scale
            pc = self.world_to_cell(px, py)
            if pc is None:
                continue  # 地図外への ray は MVP では無視(部分 ray は将来)
            if pc == rc:
                continue  # robot 自身の cell 上の点は退化 ray(robot cell を OCCUPIED にしない)
            ray = _bresenham(rc[0], rc[1], pc[0], pc[1])
            for (ix, iy) in ray[:-1]:
                # ray 通過 = FREE 証拠。既 OCCUPIED は即時には戻さず減衰
                # (証拠カウンタ。ノイズ1回では消えない保守則を保つ)
                updated += self._ray_miss(ix, iy, now_ns)
            ix, iy = ray[-1]
            if hit:
                updated += self._ray_hit(ix, iy, now_ns)
        # robot 自身の cell は FREE 証拠(接地している事実)
        self._ray_miss(rc[0], rc[1], now_ns)
        return updated

    def integrate_free_rays(self, robot_xy, points_xy, now_ns: int,
                            max_range_m: float = 8.0) -> int:
        """床面ヒット点群(= FREE の証拠)を統合する(docs/12 §3)。

        `integrate_scan` は端点 cell を必ず OCCUPIED にするため床点には使えない。
        本メソッドは robot→点 の ray 上の cell を **端点も含めて** FREE にする。
        - どの cell も OCCUPIED にはしない。
        - 既 OCCUPIED は証拠カウンタで減衰させ、尽きたら FREE へ降格
          (動的障害物の除去。1回の通過では消えない — 保守則)。
        - max_range 超の点は max_range まで切り詰める。
        - 戻り値: 状態が変わった cell 数。
        """
        if not isinstance(now_ns, int) or isinstance(now_ns, bool) or now_ns <= 0:
            raise ContractViolation("now_ns", "正の monotonic ns が必要")
        rc = self.world_to_cell(float(robot_xy[0]), float(robot_xy[1]))
        if rc is None:
            raise ContractViolation("robot_xy", "地図範囲外: %r" % (robot_xy,))
        updated = 0
        rx, ry = float(robot_xy[0]), float(robot_xy[1])
        for p in points_xy:
            px, py = float(p[0]), float(p[1])
            if not (np.isfinite(px) and np.isfinite(py)):
                continue  # 不正点は無視(integrate_scan と同じ)
            d = float(np.hypot(px - rx, py - ry))
            if d > max_range_m:
                scale = max_range_m / d
                px = rx + (px - rx) * scale
                py = ry + (py - ry) * scale
            pc = self.world_to_cell(px, py)
            if pc is None:
                continue  # 地図外への ray は MVP では無視(integrate_scan と同じ)
            for (ix, iy) in _bresenham(rc[0], rc[1], pc[0], pc[1]):
                # 端点含め FREE 証拠。既 OCCUPIED は減衰(尽きたら降格)
                updated += self._ray_miss(ix, iy, now_ns)
        return updated

    def mark_hazard(self, points_xy, now_ns: int) -> None:
        """elevation 分類(step/drop 等)による進入禁止 cell を OCCUPIED にする
        (docs/10 §5: drop/段差は costmap 上 OCCUPIED 扱い)。
        hazard は証拠カウンタの減衰対象外(ray が通過しても消えない — 安全側)。"""
        if not isinstance(now_ns, int) or isinstance(now_ns, bool) or now_ns <= 0:
            raise ContractViolation("now_ns", "正の monotonic ns が必要")
        for p in points_xy:
            c = self.world_to_cell(float(p[0]), float(p[1]))
            if c is not None:
                self.grid[c[1], c[0]] = OCCUPIED
                self.occ_score[c[1], c[0]] = HAZARD_SCORE
                self.age_ns[c[1], c[0]] = now_ns

    def clear_footprint(self, robot_xy, radius_m: float, now_ns: int) -> None:
        """robot が物理的に立っている円盤を FREE にする(2026-07-18)。

        接地している事実は最強の通行可能証拠なので、証拠カウンタも hazard も
        リセットする(立位のロボットの真下に drop は存在し得ない)。
        自己反射由来の幻 OCCUPIED による至近閉塞を常時浄化する。"""
        if not isinstance(now_ns, int) or isinstance(now_ns, bool) or now_ns <= 0:
            raise ContractViolation("now_ns", "正の monotonic ns が必要")
        rc = self.world_to_cell(float(robot_xy[0]), float(robot_xy[1]))
        if rc is None:
            return
        r = int(radius_m / self.resolution_m)
        for dj in range(-r, r + 1):
            for di in range(-r, r + 1):
                if di * di + dj * dj > r * r:
                    continue
                ix, iy = rc[0] + di, rc[1] + dj
                if 0 <= ix < self.width and 0 <= iy < self.height:
                    self.grid[iy, ix] = FREE
                    self.occ_score[iy, ix] = 0
                    self.age_ns[iy, ix] = now_ns

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
                         optimistic: bool = False) -> np.ndarray:
        """通行可能 mask。OCCUPIED は inflate_cells 分膨張させて安全 margin を取る。

        - optimistic=False(既定): FREE のみ(UNKNOWN は通行不可 — invariant 9)。
        - optimistic=True: 非OCCUPIED(FREE∪UNKNOWN)を通行可とする。探索計画
          専用(操作者要望 2026-07-17: 未踏域を探索可能領域として扱う)。
          障害物・drop hazard・inflation は引き続きブロックし、走行中の
          LiDAR 実況(障害物帯→OCCUPIED)が安全側の裏付けになる。"""
        occ = (self.grid == OCCUPIED)
        if inflate_cells > 0:
            inflated = occ.copy()
            for _ in range(inflate_cells):
                grown = inflated.copy()
                grown[1:, :] |= inflated[:-1, :]
                grown[:-1, :] |= inflated[1:, :]
                grown[:, 1:] |= inflated[:, :-1]
                grown[:, :-1] |= inflated[:, 1:]
                inflated = grown
            occ = inflated
        base = (self.grid != OCCUPIED) if optimistic else (self.grid == FREE)
        return base & (~occ)

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
            "waypoints": {k: list(v) for k, v in self.waypoints.items()},
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
        # 証拠カウンタは保存しない(schema 据え置き)。読込 OCCUPIED は満点で
        # 再構成する(hazard の永続ロックは失われるが artifacts 用途では許容)
        m.occ_score = np.where(grid == OCCUPIED, OCC_MAX, 0).astype(np.int16)
        age = np.asarray(d["age_ns"], dtype=np.int64)
        if age.shape != (h, w):
            raise ContractViolation("age_ns", "shape 不一致")
        if (age < 0).any():
            raise ContractViolation("age_ns", "負の時刻を拒否")
        m.age_ns = age
        for k, v in d.get("waypoints", {}).items():
            m.set_waypoint(k, v)
        return m
