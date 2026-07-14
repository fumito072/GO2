"""StairModel の契約テスト(Gate 0 素材、robot 非接続)。

fixture は docs/05 §3.2 の出力 schema(10 cm × 4 段)に基づく。
DROP は「下り階段」ではなく乗り入れ拒否地形 class であること、
direction が terrain_class=STAIRS のときだけ有効であることを型レベルで固定する。
"""
import copy
import dataclasses
import math
import unittest

from contracts import ContractViolation
from contracts.stair_model import StairModel, Plane, TerrainClass, StairDirection


def make_stairs_dict():
    """10 cm × 4 段の上り階段(docs/05 §3.2 / デモ仕様)。"""
    plane = {"normal": [0.0, 0.0, 1.0], "offset": 0.0,
             "covariance": [0.0] * 9}
    top = {"normal": [0.0, 0.0, 1.0], "offset": 0.40,
           "covariance": [0.0] * 9}
    return {
        "schema_version": "1.0",
        "stair_id": "test_stair_001",
        "timestamp_monotonic_ns": 123456789,
        "frame_id": "odom",
        "terrain_class": "STAIRS",
        "direction": "UP",
        "pose": [1.2, 0.0, 0.0, 0.05],
        "width_m": 0.6,
        "riser_height_m": [0.10, 0.10, 0.10, 0.10],
        "tread_depth_m": [0.30, 0.30, 0.30, 0.30],
        "bottom_plane": plane,
        "top_plane": top,
        "visible_steps": 4,
        "fresh_coverage": {"approach": 0.95, "next_footholds": 0.90, "landing": 0.85},
        "training_envelope_match": True,
        "confidence": 0.9,
    }


class TestStairModelValid(unittest.TestCase):
    def test_roundtrip(self):
        m = StairModel.from_dict(make_stairs_dict())
        self.assertIs(m.terrain_class, TerrainClass.STAIRS)
        self.assertIs(m.direction, StairDirection.UP)
        out = m.to_dict()
        self.assertEqual(StairModel.from_dict(out).to_dict(), out)

    def test_stairs_direction_unknown_allowed(self):
        # 幾何 fit 未確定は UNKNOWN のまま(昇降拒否は safety 層の責務)
        d = make_stairs_dict()
        d["direction"] = "UNKNOWN"
        StairModel.from_dict(d)

    def test_drop_class_valid(self):
        # DROP は段配列なし・direction UNKNOWN で表現する
        d = make_stairs_dict()
        d.update(terrain_class="DROP", direction="UNKNOWN",
                 riser_height_m=[], tread_depth_m=[], visible_steps=0,
                 width_m=0.0, bottom_plane=None, top_plane=None,
                 training_envelope_match=False, confidence=0.7)
        m = StairModel.from_dict(d)
        self.assertIs(m.terrain_class, TerrainClass.DROP)


class TestDirectionSemantics(unittest.TestCase):
    """direction は terrain_class=STAIRS のときだけ有効(docs/05 §3.2)。"""

    def test_drop_with_down_rejected(self):
        # 「DROP=下り階段」の混同を型レベルで禁止
        d = make_stairs_dict()
        d.update(terrain_class="DROP", direction="DOWN",
                 riser_height_m=[], tread_depth_m=[], visible_steps=0,
                 bottom_plane=None, top_plane=None)
        with self.assertRaises(ContractViolation):
            StairModel.from_dict(d)

    def test_wall_with_up_rejected(self):
        d = make_stairs_dict()
        d.update(terrain_class="WALL", direction="UP",
                 riser_height_m=[], tread_depth_m=[], visible_steps=0,
                 bottom_plane=None, top_plane=None)
        with self.assertRaises(ContractViolation):
            StairModel.from_dict(d)

    def test_unknown_class_with_steps_rejected(self):
        # 非 STAIRS が段配列を持つのは矛盾
        d = make_stairs_dict()
        d["terrain_class"] = "UNKNOWN"
        d["direction"] = "UNKNOWN"
        with self.assertRaises(ContractViolation):
            StairModel.from_dict(d)


class TestGeometryChecks(unittest.TestCase):
    def test_riser_count_mismatch_rejected(self):
        d = make_stairs_dict()
        d["riser_height_m"] = [0.10, 0.10, 0.10]  # visible_steps=4 と不一致
        with self.assertRaises(ContractViolation):
            StairModel.from_dict(d)

    def test_riser_out_of_range_rejected(self):
        for bad in (0.005, 0.35, -0.10):
            d = make_stairs_dict()
            d["riser_height_m"] = [0.10, 0.10, bad, 0.10]
            with self.assertRaises(ContractViolation, msg=str(bad)):
                StairModel.from_dict(d)

    def test_nan_rejected_everywhere(self):
        base = make_stairs_dict()
        for bad in (math.nan, math.inf):
            for path in (("width_m",), ("confidence",),
                         ("pose", 3), ("riser_height_m", 0),
                         ("bottom_plane", "offset"),
                         ("fresh_coverage", "landing")):
                d = copy.deepcopy(base)
                node = d
                for k in path[:-1]:
                    node = node[k]
                node[path[-1]] = bad
                with self.assertRaises(ContractViolation, msg="%s=%r" % (path, bad)):
                    StairModel.from_dict(d)

    def test_coverage_out_of_range_rejected(self):
        for bad in (1.2, -0.1):
            d = make_stairs_dict()
            d["fresh_coverage"]["landing"] = bad
            with self.assertRaises(ContractViolation, msg=str(bad)):
                StairModel.from_dict(d)

    def test_non_unit_normal_rejected(self):
        d = make_stairs_dict()
        d["bottom_plane"]["normal"] = [0.0, 0.0, 0.5]
        with self.assertRaises(ContractViolation):
            StairModel.from_dict(d)

    def test_bad_covariance_rejected(self):
        d = make_stairs_dict()
        d["bottom_plane"]["covariance"] = [0.0] * 5  # 0 でも 9 でもない
        with self.assertRaises(ContractViolation):
            StairModel.from_dict(d)
        d2 = make_stairs_dict()
        cov = [0.0] * 9
        cov[0] = -1.0  # 負の対角成分
        d2["bottom_plane"]["covariance"] = cov
        with self.assertRaises(ContractViolation):
            StairModel.from_dict(d2)

    def test_stairs_requires_bottom_plane(self):
        d = make_stairs_dict()
        d["bottom_plane"] = None
        with self.assertRaises(ContractViolation):
            StairModel.from_dict(d)

    def test_zero_timestamp_rejected(self):
        d = make_stairs_dict()
        d["timestamp_monotonic_ns"] = 0
        with self.assertRaises(ContractViolation):
            StairModel.from_dict(d)

    def test_unknown_key_rejected(self):
        d = make_stairs_dict()
        d["is_climbable"] = True  # 許可 flag の注入は拒否(許可は safety 層の責務)
        with self.assertRaises(ContractViolation):
            StairModel.from_dict(d)

    def test_no_permission_methods(self):
        # StairModel は許可を出さない(invariant 10)— 誤って生えたら fail
        m = StairModel.from_dict(make_stairs_dict())
        for name in ("is_climbable", "allow_ascent", "allow_descent", "approve"):
            self.assertFalse(hasattr(m, name), name)

    def test_direct_construction_rejects_str_enums(self):
        """文字列 "DOWN"/"STAIRS" は enum の is 比較をすり抜けて誤分岐する
        ため、直接構築(replace 含む)でも enum 型を強制する(fail-open 防止)。"""
        m = StairModel.from_dict(make_stairs_dict())
        with self.assertRaises(ContractViolation):
            dataclasses.replace(m, direction="DOWN")
        with self.assertRaises(ContractViolation):
            dataclasses.replace(m, terrain_class="STAIRS")

    def test_replace_revalidates_geometry(self):
        m = StairModel.from_dict(make_stairs_dict())
        with self.assertRaises(ContractViolation):
            dataclasses.replace(m, riser_height_m=(0.10, 0.10, 0.50, 0.10))

    def test_reject_trailing_newline_tokens(self):
        # `$` アンカーの末尾改行許容を塞いだ回帰テスト(fail-closed)
        for key, bad in (("stair_id", "test_stair_001\n"), ("frame_id", "odom\n")):
            d = make_stairs_dict()
            d[key] = bad
            with self.assertRaises(ContractViolation, msg=key):
                StairModel.from_dict(d)

    def test_plane_from_dict_self_validates(self):
        with self.assertRaises(ContractViolation):
            Plane.from_dict({"normal": [0.0, 0.0, 0.5], "offset": 0.0,
                             "covariance": []}, "p")


if __name__ == "__main__":
    unittest.main()
