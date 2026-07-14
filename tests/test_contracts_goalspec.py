"""GoalSpec / GoalProposal の契約テスト(Gate 0 素材、robot 非接続)。

fixture は docs/02 §4.1 と docs/06 §8.1 の canonical 例に基づく。
「一番上まで行ったら止まれ」= completion 条件(TOP_LANDING_STABLE)であり
STOP_NOW ではない、という docs/02 §4.2 の意味分離を型レベルで固定する。
"""
import copy
import dataclasses
import math
import unittest
import uuid

from contracts import ContractViolation
from contracts.goal_spec import (
    GoalSpec, GoalProposal, Intent, CompletionPredicate, ConfirmationStatus,
    Modality, Precondition, Source, Completion, PROJECT_MAX_SPEED_MPS,
)


def _uuid():
    return str(uuid.uuid4())


def make_ascend_dict():
    """docs/06 §8.1 の音声 ASCEND 例(CONFIRMED 済み GoalSpec)。"""
    return {
        "schema_version": "1.0",
        "goal_id": _uuid(),
        "source": {
            "modality": "VOICE",
            "operator_lease_id": _uuid(),
            "session_id": _uuid(),
            "utterance_id": _uuid(),
        },
        "transcript": {
            "text": "その階段を登って一番上まで行ったら止まれ",
            "language": "ja",
            "model_id": "pinned-asr-model-id",
            "evidence": {"asr_quality_score": 0.97, "no_speech_probability": 0.01},
        },
        "intent": "ASCEND_STAIRS",
        "target": {"type": "stairs", "ref": "current", "resolved_id": "test_stair_001"},
        "completion": {"predicate": "TOP_LANDING_STABLE",
                       "terminal_action": "ACTIVE_HOLD", "dwell_s": 1.0},
        "constraints": {"max_speed_mps": 0.25, "require_geometry_confirmation": True},
        "confidence": {"semantic_score": 0.98, "context_score": 1.0,
                       "parser_version": "voice-intent-ja-v1"},
        "confirmation": {"required": True, "status": "CONFIRMED",
                         "proposal_id": _uuid(), "challenge_id": _uuid()},
        "preconditions": ["OPERATOR_LEASE_VALID", "ROBOT_ARMED",
                          "ACTIVE_STAIR_GEOMETRY_VALID", "SAFETY_SUPERVISOR_OK"],
        "created_at_utc": "2026-07-15T00:00:00Z",
        "expires_after_ms": 5000,
    }


def make_stop_now_dict():
    """docs/02 §4.2:「止まれ」「今すぐ止まって」→ STOP_NOW / IMMEDIATE。"""
    return {
        "schema_version": "1.0",
        "goal_id": _uuid(),
        "source": {"modality": "TEXT", "operator_lease_id": _uuid(),
                   "session_id": _uuid(), "utterance_id": None},
        "transcript": {"text": "今すぐ止まって", "language": "ja",
                       "model_id": "text-parser-v1"},
        "intent": "STOP_NOW",
        "target": None,
        "completion": {"predicate": "IMMEDIATE",
                       "terminal_action": "ACTIVE_HOLD", "dwell_s": 1.0},
        "constraints": {"max_speed_mps": 0.25, "require_geometry_confirmation": True},
        "confidence": {"semantic_score": 1.0, "context_score": 1.0,
                       "parser_version": "text-parser-v1"},
        "confirmation": {"required": False, "status": "NOT_REQUIRED",
                         "proposal_id": None, "challenge_id": None},
        "preconditions": [],
        "created_at_utc": "2026-07-15T00:00:00Z",
        "expires_after_ms": 1000,
    }


class TestGoalSpecValid(unittest.TestCase):
    def test_ascend_roundtrip(self):
        d = make_ascend_dict()
        spec = GoalSpec.from_dict(d)
        self.assertIs(spec.intent, Intent.ASCEND_STAIRS)
        self.assertIs(spec.completion.predicate, CompletionPredicate.TOP_LANDING_STABLE)
        out = spec.to_dict()
        self.assertEqual(GoalSpec.from_dict(out).to_dict(), out)

    def test_stop_now_valid(self):
        spec = GoalSpec.from_dict(make_stop_now_dict())
        self.assertIs(spec.intent, Intent.STOP_NOW)
        self.assertIsNone(spec.target)
        self.assertIs(spec.confirmation.status, ConfirmationStatus.NOT_REQUIRED)

    def test_descend_valid(self):
        d = make_ascend_dict()
        d["intent"] = "DESCEND_STAIRS"
        d["completion"]["predicate"] = "BOTTOM_LANDING_STABLE"
        d["transcript"]["text"] = "降りて段を下り切ったらまた止まれ"
        spec = GoalSpec.from_dict(d)
        self.assertIs(spec.completion.predicate, CompletionPredicate.BOTTOM_LANDING_STABLE)

    def test_navigate_without_confirmation(self):
        d = make_ascend_dict()
        d["intent"] = "NAVIGATE_TO_STAIR_APPROACH"
        d["completion"]["predicate"] = "STAIR_APPROACH_POSE_REACHED"
        d["confirmation"] = {"required": False, "status": "NOT_REQUIRED",
                             "proposal_id": None, "challenge_id": None}
        d["preconditions"] = ["OPERATOR_LEASE_VALID", "ROBOT_ARMED"]
        GoalSpec.from_dict(d)  # 例外が出ないこと

    def test_dedup_keys_voice(self):
        """VOICE: 同一 utterance の再送は goal_id が変わっても重複検出できる
        (docs/06 §4.2)。"""
        d = make_ascend_dict()
        spec1 = GoalSpec.from_dict(d)
        d2 = copy.deepcopy(d)
        d2["goal_id"] = _uuid()  # 再 parse で新 goal_id が振られた再送
        spec2 = GoalSpec.from_dict(d2)
        keys1, keys2 = spec1.dedup_keys(), spec2.dedup_keys()
        self.assertEqual(len(keys1), 2)
        self.assertIn(("goal", spec1.goal_id), keys1)
        utterance_key = ("utterance", spec1.source.operator_lease_id,
                         spec1.source.utterance_id)
        self.assertIn(utterance_key, keys1)
        # goal key は異なるが utterance key を共有する → 重複として検出可能
        self.assertIn(utterance_key, keys2)
        self.assertNotIn(("goal", spec1.goal_id), keys2)

    def test_dedup_keys_text(self):
        """TEXT(utterance_id=None): goal key のみ。None を含む utterance key を
        発行しない(operator 内の別命令同士の衝突防止)。"""
        spec = GoalSpec.from_dict(make_stop_now_dict())
        keys = spec.dedup_keys()
        self.assertEqual(keys, (("goal", spec.goal_id),))


class TestGoalSpecSemantics(unittest.TestCase):
    """docs/02 §4.2: 複合文末尾の「止まれ」は completion 条件で STOP_NOW ではない。"""

    def test_stop_now_must_be_immediate(self):
        d = make_stop_now_dict()
        d["completion"]["predicate"] = "TOP_LANDING_STABLE"
        with self.assertRaises(ContractViolation):
            GoalSpec.from_dict(d)

    def test_ascend_must_not_be_immediate(self):
        d = make_ascend_dict()
        d["completion"]["predicate"] = "IMMEDIATE"
        with self.assertRaises(ContractViolation):
            GoalSpec.from_dict(d)

    def test_stop_now_rejects_target(self):
        d = make_stop_now_dict()
        d["target"] = {"type": "stairs", "ref": "current", "resolved_id": None}
        with self.assertRaises(ContractViolation):
            GoalSpec.from_dict(d)

    def test_stop_now_rejects_confirmation(self):
        d = make_stop_now_dict()
        d["confirmation"] = {"required": True, "status": "CONFIRMED",
                             "proposal_id": _uuid(), "challenge_id": _uuid()}
        with self.assertRaises(ContractViolation):
            GoalSpec.from_dict(d)

    def test_stop_now_rejects_preconditions(self):
        d = make_stop_now_dict()
        d["preconditions"] = ["ROBOT_ARMED"]
        with self.assertRaises(ContractViolation):
            GoalSpec.from_dict(d)

    def test_malformed_stop_now_rejected_receiver_must_escalate(self):
        """不正な形の STOP_NOW は契約層で fail-closed に拒否される。

        重要: 「STOP_NOW の拒否 = ロボットは動き続ける」なので、受け側は
        この ContractViolation を通常エラーにせず、raw payload の intent が
        "STOP_NOW" の場合は arbiter 優先度2(OPERATOR_STOP_OR_DISARM)の停止へ
        エスカレートする義務がある(contracts/goal_spec.py module docstring)。
        エスカレート動作自体の enforcement test は Command Arbiter task の
        受入条件(contracts/README.md 参照)。
        """
        d = make_stop_now_dict()
        d["constraints"]["max_speed_mps"] = 0.3  # 契約違反を同乗させた STOP_NOW
        with self.assertRaises(ContractViolation):
            GoalSpec.from_dict(d)


class TestGoalSpecInjection(unittest.TestCase):
    """禁止 field 注入の機械的拒否(docs/02 §4.3、docs/08 §4.2)。"""

    def test_reject_max_top_hold_s(self):
        d = make_ascend_dict()
        d["max_top_hold_s"] = 999999
        with self.assertRaises(ContractViolation):
            GoalSpec.from_dict(d)

    def test_reject_max_top_hold_s_in_constraints(self):
        d = make_ascend_dict()
        d["constraints"]["max_top_hold_s"] = 999999
        with self.assertRaises(ContractViolation):
            GoalSpec.from_dict(d)

    def test_reject_source_priority(self):
        d = make_ascend_dict()
        d["source_priority"] = 1
        with self.assertRaises(ContractViolation):
            GoalSpec.from_dict(d)

    def test_reject_velocity_injection(self):
        d = make_ascend_dict()
        d["constraints"]["vx"] = 2.0
        with self.assertRaises(ContractViolation):
            GoalSpec.from_dict(d)

    def test_reject_unknown_intent(self):
        d = make_ascend_dict()
        d["intent"] = "JUMP"
        with self.assertRaises(ContractViolation):
            GoalSpec.from_dict(d)

    def test_reject_damp_terminal_action(self):
        # 正常完了を Damp に変換しない(invariant 8)
        d = make_ascend_dict()
        d["completion"]["terminal_action"] = "DAMP"
        with self.assertRaises(ContractViolation):
            GoalSpec.from_dict(d)


class TestGoalSpecRanges(unittest.TestCase):
    def test_reject_overspeed(self):
        d = make_ascend_dict()
        d["constraints"]["max_speed_mps"] = PROJECT_MAX_SPEED_MPS + 0.05
        with self.assertRaises(ContractViolation):
            GoalSpec.from_dict(d)

    def test_reject_zero_speed(self):
        d = make_ascend_dict()
        d["constraints"]["max_speed_mps"] = 0.0
        with self.assertRaises(ContractViolation):
            GoalSpec.from_dict(d)

    def test_reject_nan_and_inf_everywhere(self):
        """全 float field への NaN/Inf 注入を網羅拒否(Gate 0: finite property)。"""
        base = make_ascend_dict()
        float_paths = [
            ("completion", "dwell_s"),
            ("constraints", "max_speed_mps"),
            ("confidence", "semantic_score"),
            ("confidence", "context_score"),
            ("transcript", "evidence", "asr_quality_score"),
            ("transcript", "evidence", "no_speech_probability"),
        ]
        for bad in (math.nan, math.inf, -math.inf):
            for path in float_paths:
                d = copy.deepcopy(base)
                node = d
                for k in path[:-1]:
                    node = node[k]
                node[path[-1]] = bad
                with self.assertRaises(ContractViolation, msg="%s=%r" % (path, bad)):
                    GoalSpec.from_dict(d)

    def test_reject_score_out_of_range(self):
        d = make_ascend_dict()
        d["confidence"]["semantic_score"] = 1.5
        with self.assertRaises(ContractViolation):
            GoalSpec.from_dict(d)

    def test_reject_expiry_out_of_range(self):
        for bad in (0, -1, 30001):
            d = make_ascend_dict()
            d["expires_after_ms"] = bad
            with self.assertRaises(ContractViolation, msg=str(bad)):
                GoalSpec.from_dict(d)

    def test_reject_bad_uuid(self):
        d = make_ascend_dict()
        d["goal_id"] = "not-a-uuid"
        with self.assertRaises(ContractViolation):
            GoalSpec.from_dict(d)

    def test_reject_non_canonical_uuid_forms(self):
        """同一 UUID の別表記は dedup key を迂回し得るため canonical のみ受理
        (docs/06 §4.2)。"""
        canonical = "deadbeef-dead-4eef-9bad-deadbeefdead"  # 英字を含む(大文字化で別表記になる)
        for bad in (canonical.upper(),
                    canonical.replace("-", ""),
                    "urn:uuid:" + canonical,
                    "{" + canonical + "}"):
            d = make_ascend_dict()
            d["goal_id"] = bad
            with self.assertRaises(ContractViolation, msg=repr(bad)):
                GoalSpec.from_dict(d)

    def test_reject_bad_created_at(self):
        for bad in ("2026/07/15 00:00", "2026-07-15T00:00:00Z\n"):
            d = make_ascend_dict()
            d["created_at_utc"] = bad
            with self.assertRaises(ContractViolation, msg=repr(bad)):
                GoalSpec.from_dict(d)


class TestGoalSpecConfirmation(unittest.TestCase):
    def test_ascend_requires_confirmation(self):
        d = make_ascend_dict()
        d["confirmation"] = {"required": False, "status": "NOT_REQUIRED",
                             "proposal_id": None, "challenge_id": None}
        with self.assertRaises(ContractViolation):
            GoalSpec.from_dict(d)

    def test_required_without_confirmed_rejected(self):
        # 確認待ちは GoalProposal であり GoalSpec にしない(docs/02 §4.1)
        d = make_ascend_dict()
        d["confirmation"]["status"] = "NOT_REQUIRED"
        with self.assertRaises(ContractViolation):
            GoalSpec.from_dict(d)

    def test_required_without_proposal_ids_rejected(self):
        d = make_ascend_dict()
        d["confirmation"]["proposal_id"] = None
        with self.assertRaises(ContractViolation):
            GoalSpec.from_dict(d)

    def test_ascend_requires_all_preconditions(self):
        d = make_ascend_dict()
        d["preconditions"] = ["ROBOT_ARMED"]
        with self.assertRaises(ContractViolation):
            GoalSpec.from_dict(d)

    def test_duplicate_preconditions_rejected(self):
        d = make_ascend_dict()
        d["preconditions"] = ["ROBOT_ARMED", "ROBOT_ARMED",
                              "OPERATOR_LEASE_VALID", "ACTIVE_STAIR_GEOMETRY_VALID",
                              "SAFETY_SUPERVISOR_OK"]
        with self.assertRaises(ContractViolation):
            GoalSpec.from_dict(d)

    def test_ascend_requires_geometry_confirmation(self):
        d = make_ascend_dict()
        d["constraints"]["require_geometry_confirmation"] = False
        with self.assertRaises(ContractViolation):
            GoalSpec.from_dict(d)


class TestNoValidationBypass(unittest.TestCase):
    """検証迂回経路の閉鎖: 直接コンストラクタ・dataclasses.replace() でも
    __post_init__ が validate() を実行する。"""

    def test_replace_revalidates(self):
        spec = GoalSpec.from_dict(make_ascend_dict())
        with self.assertRaises(ContractViolation):
            dataclasses.replace(spec, expires_after_ms=999999)
        with self.assertRaises(ContractViolation):
            dataclasses.replace(spec, intent="ASCEND_STAIRS")  # str は拒否

    def test_direct_source_with_str_modality_rejected(self):
        # 文字列 "VOICE" は `is Modality.VOICE` をすり抜けて VOICE 必須要件を
        # 迂回できたため、enum 型を強制する(fail-open 防止)
        src = Source(modality="VOICE", operator_lease_id=_uuid(),
                     session_id=_uuid(), utterance_id=None)
        with self.assertRaises(ContractViolation):
            src.validate()

    def test_subobject_from_dict_self_validates(self):
        # 親経由でなくても from_dict は fail-closed
        with self.assertRaises(ContractViolation):
            Completion.from_dict({"predicate": "TOP_LANDING_STABLE",
                                  "terminal_action": "DAMP", "dwell_s": 1.0})
        with self.assertRaises(ContractViolation):
            Source.from_dict({"modality": "TEXT",
                              "operator_lease_id": "not-a-uuid",
                              "session_id": _uuid()})


class TestGoalSpecVoice(unittest.TestCase):
    def test_voice_requires_utterance_id(self):
        d = make_ascend_dict()
        d["source"]["utterance_id"] = None
        with self.assertRaises(ContractViolation):
            GoalSpec.from_dict(d)

    def test_voice_requires_evidence(self):
        d = make_ascend_dict()
        del d["transcript"]["evidence"]
        with self.assertRaises(ContractViolation):
            GoalSpec.from_dict(d)

    def test_text_allows_no_utterance(self):
        spec = GoalSpec.from_dict(make_stop_now_dict())
        self.assertIsNone(spec.source.utterance_id)


class TestGoalProposal(unittest.TestCase):
    def _make(self):
        d = make_ascend_dict()
        return {
            "schema_version": "1.0",
            "proposal_id": _uuid(),
            "challenge_id": _uuid(),
            "source": d["source"],
            "transcript": d["transcript"],
            "intent": d["intent"],
            "target": d["target"],
            "completion": d["completion"],
            "constraints": d["constraints"],
            "confidence": d["confidence"],
            "created_monotonic_ns": 123456789,
            "confirm_ttl_s": 30.0,
        }

    def test_roundtrip(self):
        p = GoalProposal.from_dict(self._make())
        self.assertEqual(GoalProposal.from_dict(p.to_dict()).to_dict(), p.to_dict())

    def test_stop_now_never_proposal(self):
        # STOP_NOW は確認を迂回する — 提案化は契約違反(docs/02 §4.1)
        d = self._make()
        d["intent"] = "STOP_NOW"
        d["completion"]["predicate"] = "IMMEDIATE"
        d["target"] = None
        with self.assertRaises(ContractViolation):
            GoalProposal.from_dict(d)

    def test_ttl_capped_at_30s(self):
        d = self._make()
        d["confirm_ttl_s"] = 31.0
        with self.assertRaises(ContractViolation):
            GoalProposal.from_dict(d)

    def test_reject_unknown_key(self):
        d = self._make()
        d["auto_execute"] = True
        with self.assertRaises(ContractViolation):
            GoalProposal.from_dict(d)


if __name__ == "__main__":
    unittest.main()
