"""contracts.stair_model — StairModel(知覚→Mission/locomotion の幾何契約)。

正本: docs/05_ASCENT_DESCENT_DESIGN.md §3.2(出力 schema)、§3.3(unknown の扱い)。
      docs/02_TARGET_ARCHITECTURE.md §7 が列挙する visible_region / unknown_mask /
      cell_age / top-level covariance は本 v1 に**未実装**であり、Phase 2/4 の
      grid 二層契約(policy_height_scan / safety_terrain_map)とともに追加する。
      それまで安全 gate は fresh_coverage を部分代替として使うが、
      unknown != flat(invariant 9)の grid レベル保証はまだ契約化されていない。

要点:
  - `confidence` 一つに安全判断を隠さない。terrain class、direction、geometry、
    freshness、coverage を別 field として保持する(docs/05 §3.2)。
  - `DROP` は階段方向の一種ではなく、乗り入れを拒否する地形 class(docs/05 §3.2)。
  - direction は terrain_class=STAIRS のときだけ有効(docs/05 §3.2)。
  - unknown != flat(invariant 9)。未観測 cell の flat 補完は policy 互換層
    (policy_height_scan)に限定し、安全 gate には safety_terrain_map を使う
    (docs/05 §3.3)。その二層の grid 契約は Phase 2/4 の perception 作業で追加する。
  - 本クラスは検証済みデータの型であり、昇降の「許可」を出す method は持たない。
    許可判断は Mission/safety 層の責務(invariant 10)。
"""
from dataclasses import dataclass
from enum import Enum, unique
from typing import Mapping, Optional

from contracts import _validation as V
from contracts._validation import fail

SCHEMA_VERSION = "1.0"

# 物理 sanity 範囲(契約レベルの粗い防壁。適合判定は training_envelope_match と
# stair_registry 照合が担う)
RISER_RANGE_M = (0.01, 0.30)
TREAD_RANGE_M = (0.05, 1.00)
WIDTH_RANGE_M = (0.20, 3.00)
MAX_STEPS = 16
_YAW_RANGE = (-3.1415927, 3.1415927)

_PLANE_KEYS = ("normal", "offset", "covariance")
_COVERAGE_KEYS = ("approach", "next_footholds", "landing")
_MODEL_KEYS = ("schema_version", "stair_id", "timestamp_monotonic_ns", "frame_id",
               "terrain_class", "direction", "pose", "width_m", "riser_height_m",
               "tread_depth_m", "bottom_plane", "top_plane", "visible_steps",
               "fresh_coverage", "training_envelope_match", "confidence")


@unique
class TerrainClass(Enum):
    STAIRS = "STAIRS"
    DROP = "DROP"      # 乗り入れ拒否地形。「下り階段」と混同しない
    WALL = "WALL"
    UNKNOWN = "UNKNOWN"


@unique
class StairDirection(Enum):
    UP = "UP"
    DOWN = "DOWN"
    UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class Plane:
    """landing 平面の推定。covariance は 3x3 row-major(9要素)。
    未推定は要素数 0 で明示し、消費側は「未推定 != 平面あり」として扱う。"""
    normal: tuple
    offset: float
    covariance: tuple

    def validate(self, path: str) -> None:
        n = V.req_float_list(self.normal, path + ".normal", n=3, lo=-1.0, hi=1.0)
        norm2 = n[0] * n[0] + n[1] * n[1] + n[2] * n[2]
        if abs(norm2 - 1.0) > 1e-3:
            fail(path + ".normal", "単位ベクトルが必要(|n|^2=%g)" % norm2)
        V.req_finite(self.offset, path + ".offset", -100.0, 100.0)
        if len(self.covariance) not in (0, 9):
            fail(path + ".covariance", "要素数は 0(未推定)か 9(3x3): %d"
                 % len(self.covariance))
        V.req_float_list(self.covariance, path + ".covariance")
        for i, c in enumerate(self.covariance):
            if i in (0, 4, 8) and c < 0.0:
                fail(path + ".covariance[%d]" % i, "対角成分は非負")

    @classmethod
    def from_dict(cls, d: Mapping, path: str) -> "Plane":
        V.req_mapping(d, path)
        V.no_unknown_keys(d, _PLANE_KEYS, path)
        V.req_keys(d, _PLANE_KEYS, path)
        obj = cls(normal=tuple(d["normal"]), offset=d["offset"],
                  covariance=tuple(d["covariance"]))
        obj.validate(path)
        return obj

    def to_dict(self) -> dict:
        return {"normal": list(self.normal), "offset": self.offset,
                "covariance": list(self.covariance)}


@dataclass(frozen=True)
class FreshCoverage:
    """領域別の fresh 観測 coverage [0,1]。安全 gate は landing 系の未観測を
    NO-GO にする(docs/05 §3.3)— gate 自体は safety 層の責務。"""
    approach: float
    next_footholds: float
    landing: float

    def validate(self, path: str = "fresh_coverage") -> None:
        V.req_score(self.approach, path + ".approach")
        V.req_score(self.next_footholds, path + ".next_footholds")
        V.req_score(self.landing, path + ".landing")

    @classmethod
    def from_dict(cls, d: Mapping, path: str = "fresh_coverage") -> "FreshCoverage":
        V.req_mapping(d, path)
        V.no_unknown_keys(d, _COVERAGE_KEYS, path)
        V.req_keys(d, _COVERAGE_KEYS, path)
        obj = cls(approach=d["approach"], next_footholds=d["next_footholds"],
                  landing=d["landing"])
        obj.validate(path)
        return obj

    def to_dict(self) -> dict:
        return {"approach": self.approach, "next_footholds": self.next_footholds,
                "landing": self.landing}


@dataclass(frozen=True)
class StairModel:
    schema_version: str
    stair_id: str
    timestamp_monotonic_ns: int
    frame_id: str
    terrain_class: TerrainClass
    direction: StairDirection
    pose: tuple                    # (x, y, z, yaw)
    width_m: float
    riser_height_m: tuple
    tread_depth_m: tuple
    bottom_plane: Optional[Plane]
    top_plane: Optional[Plane]
    visible_steps: int
    fresh_coverage: FreshCoverage
    training_envelope_match: bool
    confidence: float

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        if self.schema_version != SCHEMA_VERSION:
            fail("schema_version", "未対応 version: %r" % (self.schema_version,))
        # enum 型を最初に保証する(文字列 "STAIRS"/"DOWN" 等が is 比較を
        # すり抜けて誤分岐する fail-open を防ぐ)
        V.req_enum_member(self.terrain_class, TerrainClass, "terrain_class")
        V.req_enum_member(self.direction, StairDirection, "direction")
        if not isinstance(self.fresh_coverage, FreshCoverage):
            fail("fresh_coverage", "FreshCoverage が必要: %r" % (self.fresh_coverage,))
        for name in ("bottom_plane", "top_plane"):
            v = getattr(self, name)
            if v is not None and not isinstance(v, Plane):
                fail(name, "Plane か null が必要: %r" % (v,))
        V.req_token(self.stair_id, "stair_id")
        V.req_int(self.timestamp_monotonic_ns, "timestamp_monotonic_ns", 1)
        V.req_token(self.frame_id, "frame_id")
        p = V.req_float_list(self.pose, "pose", n=4, lo=-1000.0, hi=1000.0)
        V.req_finite(p[3], "pose[3](yaw)", _YAW_RANGE[0], _YAW_RANGE[1])
        V.req_bool(self.training_envelope_match, "training_envelope_match")
        V.req_score(self.confidence, "confidence")
        self.fresh_coverage.validate()

        if self.terrain_class is TerrainClass.STAIRS:
            # direction は STAIRS のときだけ意味を持つが、UNKNOWN のままでもよい
            # (幾何 fit 未確定)。UNKNOWN の場合の昇降拒否は safety 層が行う。
            V.req_int(self.visible_steps, "visible_steps", 1, MAX_STEPS)
            if len(self.riser_height_m) != self.visible_steps:
                fail("riser_height_m", "要素数 %d != visible_steps %d"
                     % (len(self.riser_height_m), self.visible_steps))
            if len(self.tread_depth_m) != self.visible_steps:
                fail("tread_depth_m", "要素数 %d != visible_steps %d"
                     % (len(self.tread_depth_m), self.visible_steps))
            V.req_float_list(self.riser_height_m, "riser_height_m",
                             lo=RISER_RANGE_M[0], hi=RISER_RANGE_M[1])
            V.req_float_list(self.tread_depth_m, "tread_depth_m",
                             lo=TREAD_RANGE_M[0], hi=TREAD_RANGE_M[1])
            V.req_finite(self.width_m, "width_m", WIDTH_RANGE_M[0], WIDTH_RANGE_M[1])
            if self.bottom_plane is None:
                fail("bottom_plane", "STAIRS では bottom_plane 必須")
            self.bottom_plane.validate("bottom_plane")
            if self.top_plane is not None:
                self.top_plane.validate("top_plane")
        else:
            # 非 STAIRS: direction は無効(UNKNOWN 固定)、段配列は空(docs/05 §3.2)
            if self.direction is not StairDirection.UNKNOWN:
                fail("direction", "direction は terrain_class=STAIRS のときだけ有効: %s/%s"
                     % (self.terrain_class.name, self.direction.name))
            if self.riser_height_m or self.tread_depth_m:
                fail("riser_height_m", "非 STAIRS で段配列は空")
            if self.visible_steps != 0:
                fail("visible_steps", "非 STAIRS では 0")
            V.req_finite(self.width_m, "width_m", 0.0, WIDTH_RANGE_M[1])
            if self.bottom_plane is not None:
                self.bottom_plane.validate("bottom_plane")
            if self.top_plane is not None:
                self.top_plane.validate("top_plane")

    @classmethod
    def from_dict(cls, d: Mapping) -> "StairModel":
        V.req_mapping(d, "stairmodel")
        V.no_unknown_keys(d, _MODEL_KEYS, "stairmodel")
        V.req_keys(d, _MODEL_KEYS, "stairmodel")
        bp, tp = d["bottom_plane"], d["top_plane"]
        return cls(
            schema_version=d["schema_version"],
            stair_id=d["stair_id"],
            timestamp_monotonic_ns=d["timestamp_monotonic_ns"],
            frame_id=d["frame_id"],
            terrain_class=V.req_enum(d["terrain_class"], TerrainClass, "terrain_class"),
            direction=V.req_enum(d["direction"], StairDirection, "direction"),
            pose=tuple(d["pose"]),
            width_m=d["width_m"],
            riser_height_m=tuple(d["riser_height_m"]),
            tread_depth_m=tuple(d["tread_depth_m"]),
            bottom_plane=None if bp is None else Plane.from_dict(bp, "bottom_plane"),
            top_plane=None if tp is None else Plane.from_dict(tp, "top_plane"),
            visible_steps=d["visible_steps"],
            fresh_coverage=FreshCoverage.from_dict(d["fresh_coverage"]),
            training_envelope_match=d["training_envelope_match"],
            confidence=d["confidence"],
        )  # __post_init__ が validate() を実行する

    def to_dict(self) -> dict:
        return {
            "schema_version": self.schema_version,
            "stair_id": self.stair_id,
            "timestamp_monotonic_ns": self.timestamp_monotonic_ns,
            "frame_id": self.frame_id,
            "terrain_class": self.terrain_class.value,
            "direction": self.direction.value,
            "pose": list(self.pose),
            "width_m": self.width_m,
            "riser_height_m": list(self.riser_height_m),
            "tread_depth_m": list(self.tread_depth_m),
            "bottom_plane": None if self.bottom_plane is None else self.bottom_plane.to_dict(),
            "top_plane": None if self.top_plane is None else self.top_plane.to_dict(),
            "visible_steps": self.visible_steps,
            "fresh_coverage": self.fresh_coverage.to_dict(),
            "training_envelope_match": self.training_envelope_match,
            "confidence": self.confidence,
        }
