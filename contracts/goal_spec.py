"""contracts.goal_spec — 共通命令 GoalSpec / GoalProposal。

正本: docs/02_TARGET_ARCHITECTURE.md §4.1(canonical schema)、§4.2(正規化)、
      §4.3(前提条件)、docs/06_VOICE_AIRPODS.md §8.1(音声 evidence 拡張)。

要点:
  - 文中末尾の「止まれ」は completion 条件(*_STABLE + ACTIVE_HOLD)であり、
    独立命令 STOP_NOW ではない(docs/02 §4.2)。
  - STOP_NOW は proposal/確認を迂回し NOT_REQUIRED(docs/02 §4.1)。
  - `max_top_hold_s` 等の安全上限は GoalSpec field にしない(docs/02 §4.3)。
    server-side signed config に置く。未知キーは from_dict で拒否される。
  - confidence は監査/gate 用 metadata であり、実行 parameter の補間に使わない
    (docs/06 §8.1)。
  - created_at_utc は監査用。受理後の expiry/watchdog は server の monotonic
    time で判定する(docs/02 §4.1)。
  - 検証迂回経路なし: 直接コンストラクタ・dataclasses.replace() も
    __post_init__ で validate() が走る(GoalSpec / GoalProposal)。
    下位型は from_dict が自己検証する(Transcript のみ modality 文脈が必要な
    ため単体では部分検証 — 完全検証は親経由)。

STOP_NOW の拒否時挙動(受け側の必須要件):
  不正な形の STOP_NOW(禁止 field 付き・constraints 異常等)は本契約が
  fail-closed で拒否するが、「停止要求の拒否 = ロボットは動き続ける」である。
  ContractViolation を受けた層は、raw payload の intent が "STOP_NOW" の場合、
  拒否を通常のエラー応答にせず arbiter 優先度2(OPERATOR_STOP_OR_DISARM)の
  停止へエスカレートし、latched fault と structured log を残すこと
  (docs/CLAUDE.md §10)。エスカレートは認証済み operator session/lease 文脈
  内でのみ行う(未認証 payload を停止 DoS ベクタにしない)。
"""
from dataclasses import dataclass
from enum import Enum, unique
from typing import Mapping, Optional

from contracts import _validation as V
from contracts._validation import fail


@unique
class Modality(Enum):
    TEXT = "TEXT"
    VOICE = "VOICE"
    UI = "UI"
    MANUAL = "MANUAL"


@unique
class Intent(Enum):
    NAVIGATE_TO_STAIR_APPROACH = "NAVIGATE_TO_STAIR_APPROACH"
    ASCEND_STAIRS = "ASCEND_STAIRS"
    DESCEND_STAIRS = "DESCEND_STAIRS"
    STOP_NOW = "STOP_NOW"


@unique
class CompletionPredicate(Enum):
    STAIR_APPROACH_POSE_REACHED = "STAIR_APPROACH_POSE_REACHED"
    TOP_LANDING_STABLE = "TOP_LANDING_STABLE"
    BOTTOM_LANDING_STABLE = "BOTTOM_LANDING_STABLE"
    IMMEDIATE = "IMMEDIATE"


@unique
class ConfirmationStatus(Enum):
    CONFIRMED = "CONFIRMED"
    NOT_REQUIRED = "NOT_REQUIRED"


@unique
class Precondition(Enum):
    """docs/06 §8.1 の具体 instance の4項。docs/02 §4.1 の JSON は union 記法の
    schema sketch であり、その preconditions 2項は例示値。本契約は ASCEND/DESCEND
    に対し modality によらず4項すべてを必須とする(fail-closed。優先順:
    08 safety > 02 — 安全側に倒し、緩和はしない)。"""
    OPERATOR_LEASE_VALID = "OPERATOR_LEASE_VALID"
    ROBOT_ARMED = "ROBOT_ARMED"
    ACTIVE_STAIR_GEOMETRY_VALID = "ACTIVE_STAIR_GEOMETRY_VALID"
    SAFETY_SUPERVISOR_OK = "SAFETY_SUPERVISOR_OK"


# intent → completion.predicate の正準対応(docs/02 §4.2 の表)
INTENT_TO_PREDICATE = {
    Intent.NAVIGATE_TO_STAIR_APPROACH: CompletionPredicate.STAIR_APPROACH_POSE_REACHED,
    Intent.ASCEND_STAIRS: CompletionPredicate.TOP_LANDING_STABLE,
    Intent.DESCEND_STAIRS: CompletionPredicate.BOTTOM_LANDING_STABLE,
    Intent.STOP_NOW: CompletionPredicate.IMMEDIATE,
}

# デモ仕様の速度上限(docs/02 §4.1 constraints)。緩和には根拠・review・test が必要
# (docs/CLAUDE.md §10)。
PROJECT_MAX_SPEED_MPS = 0.25
# GoalSpec の受理期限上限。GoalProposal の確認 TTL 初期値 30 秒(docs/02 §4.1)を超えない。
MAX_EXPIRES_AFTER_MS = 30_000
SCHEMA_VERSION = "1.0"

_SOURCE_KEYS = ("modality", "operator_lease_id", "session_id", "utterance_id")
_TRANSCRIPT_KEYS = ("text", "language", "model_id", "evidence")
_EVIDENCE_KEYS = ("asr_quality_score", "no_speech_probability")
_TARGET_KEYS = ("type", "ref", "resolved_id")
_COMPLETION_KEYS = ("predicate", "terminal_action", "dwell_s")
_CONSTRAINTS_KEYS = ("max_speed_mps", "require_geometry_confirmation")
_CONFIDENCE_KEYS = ("semantic_score", "context_score", "parser_version")
_CONFIRMATION_KEYS = ("required", "status", "proposal_id", "challenge_id")
_GOALSPEC_KEYS = ("schema_version", "goal_id", "source", "transcript", "intent",
                  "target", "completion", "constraints", "confidence",
                  "confirmation", "preconditions", "created_at_utc",
                  "expires_after_ms")


@dataclass(frozen=True)
class Source:
    modality: Modality
    operator_lease_id: str
    session_id: str
    utterance_id: Optional[str]  # VOICE では必須(dedup key の一部, docs/06 §4.2)

    def validate(self, path: str = "source") -> None:
        V.req_enum_member(self.modality, Modality, path + ".modality")
        V.req_uuid(self.operator_lease_id, path + ".operator_lease_id")
        V.req_uuid(self.session_id, path + ".session_id")
        V.opt_uuid(self.utterance_id, path + ".utterance_id")
        if self.modality is Modality.VOICE and self.utterance_id is None:
            fail(path + ".utterance_id", "VOICE では utterance_id 必須(再送重複防止)")

    @classmethod
    def from_dict(cls, d: Mapping, path: str = "source") -> "Source":
        V.req_mapping(d, path)
        V.no_unknown_keys(d, _SOURCE_KEYS, path)
        V.req_keys(d, ("modality", "operator_lease_id", "session_id"), path)
        obj = cls(
            modality=V.req_enum(d["modality"], Modality, path + ".modality"),
            operator_lease_id=d["operator_lease_id"],
            session_id=d["session_id"],
            utterance_id=d.get("utterance_id"),
        )
        obj.validate(path)
        return obj

    def to_dict(self) -> dict:
        return {"modality": self.modality.value,
                "operator_lease_id": self.operator_lease_id,
                "session_id": self.session_id,
                "utterance_id": self.utterance_id}


@dataclass(frozen=True)
class TranscriptEvidence:
    """音声固有の監査 evidence(docs/06 §8.1)。実行判断の補間には使わない。"""
    asr_quality_score: float
    no_speech_probability: float

    def validate(self, path: str = "transcript.evidence") -> None:
        V.req_score(self.asr_quality_score, path + ".asr_quality_score")
        V.req_score(self.no_speech_probability, path + ".no_speech_probability")

    @classmethod
    def from_dict(cls, d: Mapping, path: str = "transcript.evidence") -> "TranscriptEvidence":
        V.req_mapping(d, path)
        V.no_unknown_keys(d, _EVIDENCE_KEYS, path)
        V.req_keys(d, _EVIDENCE_KEYS, path)
        obj = cls(asr_quality_score=d["asr_quality_score"],
                  no_speech_probability=d["no_speech_probability"])
        obj.validate(path)
        return obj

    def to_dict(self) -> dict:
        return {"asr_quality_score": self.asr_quality_score,
                "no_speech_probability": self.no_speech_probability}


@dataclass(frozen=True)
class Transcript:
    """注意: 単体の from_dict は text/language/model_id の構造検査のみで、
    「VOICE では evidence 必須」の完全検証は modality 文脈を持つ親
    (GoalSpec / GoalProposal)経由でのみ行われる。"""
    text: str
    language: str
    model_id: str
    evidence: Optional[TranscriptEvidence] = None  # VOICE では必須

    def validate(self, modality: Modality, path: str = "transcript") -> None:
        V.req_str(self.text, path + ".text", max_len=512)
        V.req_str(self.language, path + ".language", max_len=16)
        V.req_str(self.model_id, path + ".model_id", max_len=128)
        if modality is Modality.VOICE:
            if self.evidence is None:
                fail(path + ".evidence", "VOICE では ASR evidence 必須(docs/06 §8.1)")
            self.evidence.validate(path + ".evidence")
        elif self.evidence is not None:
            self.evidence.validate(path + ".evidence")

    @classmethod
    def from_dict(cls, d: Mapping, path: str = "transcript") -> "Transcript":
        V.req_mapping(d, path)
        V.no_unknown_keys(d, _TRANSCRIPT_KEYS, path)
        V.req_keys(d, ("text", "language", "model_id"), path)
        ev = d.get("evidence")
        return cls(text=d["text"], language=d["language"], model_id=d["model_id"],
                   evidence=None if ev is None else TranscriptEvidence.from_dict(ev, path + ".evidence"))

    def to_dict(self) -> dict:
        out = {"text": self.text, "language": self.language, "model_id": self.model_id}
        if self.evidence is not None:
            out["evidence"] = self.evidence.to_dict()
        return out


@dataclass(frozen=True)
class Target:
    type: str                      # 現仕様は "stairs" のみ
    ref: str                       # "current" | "nearest" | stair_id token
    resolved_id: Optional[str]     # 解決済み stair_id or None

    def validate(self, path: str = "target") -> None:
        if self.type != "stairs":
            fail(path + ".type", "現仕様は 'stairs' のみ: %r" % (self.type,))
        if self.ref not in ("current", "nearest"):
            V.req_token(self.ref, path + ".ref")  # stair_id 直接参照
        if self.resolved_id is not None:
            V.req_token(self.resolved_id, path + ".resolved_id")

    @classmethod
    def from_dict(cls, d: Mapping, path: str = "target") -> "Target":
        V.req_mapping(d, path)
        V.no_unknown_keys(d, _TARGET_KEYS, path)
        V.req_keys(d, ("type", "ref"), path)
        obj = cls(type=d["type"], ref=d["ref"], resolved_id=d.get("resolved_id"))
        obj.validate(path)
        return obj

    def to_dict(self) -> dict:
        return {"type": self.type, "ref": self.ref, "resolved_id": self.resolved_id}


@dataclass(frozen=True)
class Completion:
    predicate: CompletionPredicate
    terminal_action: str = "ACTIVE_HOLD"   # 唯一の許容値(docs/02 §4.1)
    dwell_s: float = 1.0

    def validate(self, path: str = "completion") -> None:
        V.req_enum_member(self.predicate, CompletionPredicate, path + ".predicate")
        if self.terminal_action != "ACTIVE_HOLD":
            fail(path + ".terminal_action",
                 "正常完了の terminal_action は ACTIVE_HOLD のみ(Damp 禁止, invariant 8): %r"
                 % (self.terminal_action,))
        V.req_finite(self.dwell_s, path + ".dwell_s", 0.0, 10.0)
        if self.dwell_s <= 0.0:
            fail(path + ".dwell_s", "dwell_s は正の値")

    @classmethod
    def from_dict(cls, d: Mapping, path: str = "completion") -> "Completion":
        V.req_mapping(d, path)
        V.no_unknown_keys(d, _COMPLETION_KEYS, path)
        V.req_keys(d, ("predicate", "terminal_action", "dwell_s"), path)
        obj = cls(predicate=V.req_enum(d["predicate"], CompletionPredicate, path + ".predicate"),
                  terminal_action=d["terminal_action"], dwell_s=d["dwell_s"])
        obj.validate(path)
        return obj

    def to_dict(self) -> dict:
        return {"predicate": self.predicate.value,
                "terminal_action": self.terminal_action, "dwell_s": self.dwell_s}


@dataclass(frozen=True)
class Constraints:
    max_speed_mps: float = PROJECT_MAX_SPEED_MPS
    require_geometry_confirmation: bool = True

    def validate(self, path: str = "constraints") -> None:
        V.req_finite(self.max_speed_mps, path + ".max_speed_mps",
                     0.0, PROJECT_MAX_SPEED_MPS)
        if self.max_speed_mps <= 0.0:
            fail(path + ".max_speed_mps", "正の値が必要")
        V.req_bool(self.require_geometry_confirmation,
                   path + ".require_geometry_confirmation")

    @classmethod
    def from_dict(cls, d: Mapping, path: str = "constraints") -> "Constraints":
        V.req_mapping(d, path)
        V.no_unknown_keys(d, _CONSTRAINTS_KEYS, path)
        V.req_keys(d, _CONSTRAINTS_KEYS, path)
        obj = cls(max_speed_mps=d["max_speed_mps"],
                  require_geometry_confirmation=d["require_geometry_confirmation"])
        obj.validate(path)
        return obj

    def to_dict(self) -> dict:
        return {"max_speed_mps": self.max_speed_mps,
                "require_geometry_confirmation": self.require_geometry_confirmation}


@dataclass(frozen=True)
class Confidence:
    """監査/gate 用 metadata。実行 parameter の補間に使わない(docs/06 §8.1)。"""
    semantic_score: float
    context_score: float
    parser_version: str

    def validate(self, path: str = "confidence") -> None:
        V.req_score(self.semantic_score, path + ".semantic_score")
        V.req_score(self.context_score, path + ".context_score")
        V.req_str(self.parser_version, path + ".parser_version", max_len=64)

    @classmethod
    def from_dict(cls, d: Mapping, path: str = "confidence") -> "Confidence":
        V.req_mapping(d, path)
        V.no_unknown_keys(d, _CONFIDENCE_KEYS, path)
        V.req_keys(d, _CONFIDENCE_KEYS, path)
        obj = cls(semantic_score=d["semantic_score"], context_score=d["context_score"],
                  parser_version=d["parser_version"])
        obj.validate(path)
        return obj

    def to_dict(self) -> dict:
        return {"semantic_score": self.semantic_score,
                "context_score": self.context_score,
                "parser_version": self.parser_version}


@dataclass(frozen=True)
class Confirmation:
    required: bool
    status: ConfirmationStatus
    proposal_id: Optional[str]
    challenge_id: Optional[str]

    def validate(self, path: str = "confirmation") -> None:
        V.req_bool(self.required, path + ".required")
        V.req_enum_member(self.status, ConfirmationStatus, path + ".status")
        if self.required:
            # 確認待ちデータは GoalSpec でなく GoalProposal(docs/02 §4.1)。
            # よって required=True の GoalSpec は CONFIRMED 済みでなければ不正。
            if self.status is not ConfirmationStatus.CONFIRMED:
                fail(path + ".status",
                     "required=True の GoalSpec は CONFIRMED 必須(確認待ちは GoalProposal)")
            V.req_uuid(self.proposal_id, path + ".proposal_id")
            V.req_uuid(self.challenge_id, path + ".challenge_id")
        else:
            if self.status is not ConfirmationStatus.NOT_REQUIRED:
                fail(path + ".status", "required=False は NOT_REQUIRED のみ")
            if self.proposal_id is not None or self.challenge_id is not None:
                fail(path, "required=False で proposal_id/challenge_id は null")

    @classmethod
    def from_dict(cls, d: Mapping, path: str = "confirmation") -> "Confirmation":
        V.req_mapping(d, path)
        V.no_unknown_keys(d, _CONFIRMATION_KEYS, path)
        V.req_keys(d, ("required", "status"), path)
        obj = cls(required=d["required"],
                  status=V.req_enum(d["status"], ConfirmationStatus, path + ".status"),
                  proposal_id=d.get("proposal_id"), challenge_id=d.get("challenge_id"))
        obj.validate(path)
        return obj

    def to_dict(self) -> dict:
        return {"required": self.required, "status": self.status.value,
                "proposal_id": self.proposal_id, "challenge_id": self.challenge_id}


@dataclass(frozen=True)
class GoalSpec:
    """実行可能な型付き命令(docs/02 §4.1 canonical schema)。

    LLM/VLM/ASR/UI はこれを生成するだけで actuator owner にならない(invariant 1)。
    JSON として正しくても、Mission state と affordance validator が実行可否を
    決める — 本クラスの validate() は schema/整合性検査であり実行許可ではない。
    直接コンストラクタ・dataclasses.replace() も __post_init__ で validate()
    が走る(検証迂回経路なし)。
    """
    schema_version: str
    goal_id: str
    source: Source
    transcript: Transcript
    intent: Intent
    target: Optional[Target]
    completion: Completion
    constraints: Constraints
    confidence: Confidence
    confirmation: Confirmation
    preconditions: tuple
    created_at_utc: str
    expires_after_ms: int

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        if self.schema_version != SCHEMA_VERSION:
            fail("schema_version", "未対応 version: %r(対応: %s)"
                 % (self.schema_version, SCHEMA_VERSION))
        V.req_uuid(self.goal_id, "goal_id")
        V.req_enum_member(self.intent, Intent, "intent")
        for name, want in (("source", Source), ("transcript", Transcript),
                           ("completion", Completion), ("constraints", Constraints),
                           ("confidence", Confidence), ("confirmation", Confirmation)):
            if not isinstance(getattr(self, name), want):
                fail(name, "%s が必要: %r" % (want.__name__, getattr(self, name)))
        if self.target is not None and not isinstance(self.target, Target):
            fail("target", "Target か null が必要: %r" % (self.target,))
        self.source.validate()
        self.transcript.validate(self.source.modality)
        self.completion.validate()
        self.constraints.validate()
        self.confidence.validate()
        self.confirmation.validate()
        V.req_utc(self.created_at_utc, "created_at_utc")
        V.req_int(self.expires_after_ms, "expires_after_ms", 1, MAX_EXPIRES_AFTER_MS)

        # intent → predicate の正準対応(docs/02 §4.2)。
        # 「一番上まで行ったら止まれ」は TOP_LANDING_STABLE であり STOP_NOW ではない。
        want = INTENT_TO_PREDICATE[self.intent]
        if self.completion.predicate is not want:
            fail("completion.predicate",
                 "%s の predicate は %s のみ: %r"
                 % (self.intent.name, want.name, self.completion.predicate.name))

        # preconditions
        if not isinstance(self.preconditions, tuple):
            fail("preconditions", "tuple が必要")
        seen = set()
        for i, p in enumerate(self.preconditions):
            if not isinstance(p, Precondition):
                fail("preconditions[%d]" % i, "Precondition enum が必要: %r" % (p,))
            if p in seen:
                fail("preconditions[%d]" % i, "重複: %s" % p.name)
            seen.add(p)

        if self.intent is Intent.STOP_NOW:
            # STOP_NOW は確認を迂回し(docs/02 §4.1)、target を持たない。
            # 注意: ここでの拒否は「停止しない」を意味する。受け側は module
            # docstring の必須要件どおり、拒否を優先度2の停止へエスカレートすること。
            if self.target is not None:
                fail("target", "STOP_NOW は target=null")
            if self.confirmation.required:
                fail("confirmation.required", "STOP_NOW は確認を要求しない(最優先受理)")
            # STOP_NOW は前提条件で拒否されない(常時受理)
            if self.preconditions:
                fail("preconditions", "STOP_NOW は前提条件なしで常時受理")
        else:
            if self.target is None:
                fail("target", "%s は target 必須" % self.intent.name)
            self.target.validate()
            if self.intent in (Intent.ASCEND_STAIRS, Intent.DESCEND_STAIRS):
                # 昇降は明示確認必須(docs/02 §4.1、CLAUDE.md Phase 3)
                if not self.confirmation.required:
                    fail("confirmation.required", "%s は明示確認必須" % self.intent.name)
                if not self.constraints.require_geometry_confirmation:
                    fail("constraints.require_geometry_confirmation",
                         "%s は幾何再検証必須(invariant 10)" % self.intent.name)
                missing = [p.name for p in Precondition if p not in seen]
                if missing:
                    fail("preconditions", "%s は4前提すべて必須。欠落: %s"
                         % (self.intent.name, missing))

    @classmethod
    def from_dict(cls, d: Mapping) -> "GoalSpec":
        """未知キーを拒否する strict parser。禁止 field(max_top_hold_s 等)の
        注入はここで機械的に弾かれる(docs/02 §4.3)。
        構築時に __post_init__ が validate() を実行する。"""
        V.req_mapping(d, "goalspec")
        V.no_unknown_keys(d, _GOALSPEC_KEYS, "goalspec")
        V.req_keys(d, _GOALSPEC_KEYS, "goalspec")
        target = d["target"]
        return cls(
            schema_version=d["schema_version"],
            goal_id=d["goal_id"],
            source=Source.from_dict(d["source"]),
            transcript=Transcript.from_dict(d["transcript"]),
            intent=V.req_enum(d["intent"], Intent, "intent"),
            target=None if target is None else Target.from_dict(target),
            completion=Completion.from_dict(d["completion"]),
            constraints=Constraints.from_dict(d["constraints"]),
            confidence=Confidence.from_dict(d["confidence"]),
            confirmation=Confirmation.from_dict(d["confirmation"]),
            preconditions=tuple(
                V.req_enum(p, Precondition, "preconditions[%d]" % i)
                for i, p in enumerate(d["preconditions"])),
            created_at_utc=d["created_at_utc"],
            expires_after_ms=d["expires_after_ms"],
        )

    def to_dict(self) -> dict:
        return {
            "schema_version": self.schema_version,
            "goal_id": self.goal_id,
            "source": self.source.to_dict(),
            "transcript": self.transcript.to_dict(),
            "intent": self.intent.value,
            "target": None if self.target is None else self.target.to_dict(),
            "completion": self.completion.to_dict(),
            "constraints": self.constraints.to_dict(),
            "confidence": self.confidence.to_dict(),
            "confirmation": self.confirmation.to_dict(),
            "preconditions": [p.value for p in self.preconditions],
            "created_at_utc": self.created_at_utc,
            "expires_after_ms": self.expires_after_ms,
        }

    def dedup_keys(self) -> tuple:
        """再送重複実行の防止 key 群。いずれか一つでも既実行なら重複とみなす
        (OR 意味論)。
        - ("goal", goal_id): 同一 goal_id の再送は idempotent に ack(docs/09 §2)。
        - ("utterance", operator_lease_id, utterance_id): 同一発話の再送は
          再 parse で goal_id が変わっても二度開始しない(docs/06 §4.2)。
          utterance_id=None(TEXT 等)では発行しない(None 同士の衝突防止)。
        """
        keys = [("goal", self.goal_id)]
        if self.source.utterance_id is not None:
            keys.append(("utterance", self.source.operator_lease_id,
                         self.source.utterance_id))
        return tuple(keys)


# GoalProposal の確認 TTL 初期値(docs/02 §4.1: 初期30秒)
PROPOSAL_CONFIRM_TTL_S = 30.0
_PROPOSAL_KEYS = ("schema_version", "proposal_id", "challenge_id", "source",
                  "transcript", "intent", "target", "completion", "constraints",
                  "confidence", "created_monotonic_ns", "confirm_ttl_s")


@dataclass(frozen=True)
class GoalProposal:
    """復唱確認待ちの提案。実行可能な GoalSpec ではない(docs/02 §4.1)。

    確認後の CONFIRMED GoalSpec 発行(新 goal_id、must_start_by=5s monotonic)は
    server 側の mission 層が operator lease・robot state・stair geometry を
    再検証した上で行う。本クラスはデータ契約のみを定義する。
    challenge_id は一回限り(使い捨て)。
    直接コンストラクタも __post_init__ で validate() が走る。
    """
    schema_version: str
    proposal_id: str
    challenge_id: str
    source: Source
    transcript: Transcript
    intent: Intent
    target: Optional[Target]
    completion: Completion
    constraints: Constraints
    confidence: Confidence
    created_monotonic_ns: int
    confirm_ttl_s: float = PROPOSAL_CONFIRM_TTL_S

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        if self.schema_version != SCHEMA_VERSION:
            fail("schema_version", "未対応 version: %r" % (self.schema_version,))
        V.req_uuid(self.proposal_id, "proposal_id")
        V.req_uuid(self.challenge_id, "challenge_id")
        V.req_enum_member(self.intent, Intent, "intent")
        if self.intent is Intent.STOP_NOW:
            # STOP_NOW は proposal/確認を迂回する(docs/02 §4.1)。提案化は契約違反。
            fail("intent", "STOP_NOW は GoalProposal を経由しない(即時受理)")
        for name, want in (("source", Source), ("transcript", Transcript),
                           ("completion", Completion), ("constraints", Constraints),
                           ("confidence", Confidence)):
            if not isinstance(getattr(self, name), want):
                fail(name, "%s が必要: %r" % (want.__name__, getattr(self, name)))
        if self.target is not None and not isinstance(self.target, Target):
            fail("target", "Target か null が必要: %r" % (self.target,))
        self.source.validate()
        self.transcript.validate(self.source.modality)
        if self.target is None:
            fail("target", "%s は target 必須" % self.intent.name)
        self.target.validate()
        self.completion.validate()
        want = INTENT_TO_PREDICATE[self.intent]
        if self.completion.predicate is not want:
            fail("completion.predicate", "%s の predicate は %s のみ"
                 % (self.intent.name, want.name))
        self.constraints.validate()
        self.confidence.validate()
        V.req_int(self.created_monotonic_ns, "created_monotonic_ns", 1)
        V.req_finite(self.confirm_ttl_s, "confirm_ttl_s", 0.0, PROPOSAL_CONFIRM_TTL_S)
        if self.confirm_ttl_s <= 0.0:
            fail("confirm_ttl_s", "正の値が必要")

    @classmethod
    def from_dict(cls, d: Mapping) -> "GoalProposal":
        V.req_mapping(d, "goalproposal")
        V.no_unknown_keys(d, _PROPOSAL_KEYS, "goalproposal")
        V.req_keys(d, _PROPOSAL_KEYS, "goalproposal")
        target = d["target"]
        return cls(
            schema_version=d["schema_version"],
            proposal_id=d["proposal_id"],
            challenge_id=d["challenge_id"],
            source=Source.from_dict(d["source"]),
            transcript=Transcript.from_dict(d["transcript"]),
            intent=V.req_enum(d["intent"], Intent, "intent"),
            target=None if target is None else Target.from_dict(target),
            completion=Completion.from_dict(d["completion"]),
            constraints=Constraints.from_dict(d["constraints"]),
            confidence=Confidence.from_dict(d["confidence"]),
            created_monotonic_ns=d["created_monotonic_ns"],
            confirm_ttl_s=d["confirm_ttl_s"],
        )

    def to_dict(self) -> dict:
        return {
            "schema_version": self.schema_version,
            "proposal_id": self.proposal_id,
            "challenge_id": self.challenge_id,
            "source": self.source.to_dict(),
            "transcript": self.transcript.to_dict(),
            "intent": self.intent.value,
            "target": None if self.target is None else self.target.to_dict(),
            "completion": self.completion.to_dict(),
            "constraints": self.constraints.to_dict(),
            "confidence": self.confidence.to_dict(),
            "created_monotonic_ns": self.created_monotonic_ns,
            "confirm_ttl_s": self.confirm_ttl_s,
        }
