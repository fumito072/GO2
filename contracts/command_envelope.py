"""contracts.command_envelope — command 調停の wire 契約。

正本: docs/08_SAFETY_TEST_EVALUATION.md §4.2(command request 必須 field と
      arbiter 優先順位)、docs/02_TARGET_ARCHITECTURE.md §7(LocomotionCommand)。

要点:
  - `source_priority` は送信者が指定する field にしない(docs/08 §4.2)。
    priority は arbiter が server-side で ServerAttribution として付与する。
    from_dict の未知キー拒否により、payload 中の自称 priority は機械的に弾かれる。
  - command expiry 後はゼロを推測せず、明示的に Controlled Stop へ遷移する
    (docs/08 §4.2 — 遷移は arbiter の責務。本契約は expiry field を必須化する)。
  - 安全判定は monotonic time のみ(wall clock 禁止)。
  - ASCEND / DESCEND_BACKWARD / DESCEND_FORWARD は別 member であり、
    符号や flag の反転で表現しない(Gate 0: 方向 command の混同禁止)。
  - 同じ LocomotionCommand を S/L 両 backend に二重配送しない(docs/02 §7)。
  - policy_hash は本契約では**形式のみ**検査する。'not_applicable' の受理可否
    (Branch L run での拒否等)は arbiter が signed run manifest の
    selected_backend と突合して判定する(次 task。docs/08 §4.3, docs/09 §5)。
    Branch L publisher は hash 不一致を1フレームも motor へ出さない(docs/08 §4.3)。
  - 検証迂回経路なし: 直接コンストラクタ・dataclasses.replace() も
    __post_init__ で validate() が走る。
"""
from dataclasses import dataclass
from enum import Enum, IntEnum, unique
from typing import Mapping, Optional

from contracts import _validation as V
from contracts._validation import fail
from contracts.stair_model import StairDirection

SCHEMA_VERSION = "1.0"

# wire レベルの物理 sanity 上限(Wave5 学習 command 範囲 |vx|<=1.0 に整合。
# デモ実行時の上限 0.25 m/s は GoalSpec.constraints / server config が締める)
MAX_ABS_VX = 1.0
MAX_ABS_VY = 0.5
MAX_ABS_WZ = 2.0
MAX_ENVELOPE_EXPIRES_MS = 10_000

_ENVELOPE_KEYS = ("schema_version", "source_id", "goal_id", "actuation_request_id",
                  "sender_timestamp", "sequence", "expires_after_ms",
                  "requested_mode", "vx", "vy", "wz", "phase", "policy_hash")
_SUMMARY_KEYS = ("stair_id", "direction", "visible_steps",
                 "riser_height_min_m", "riser_height_max_m", "fresh_coverage_landing")
_LOCOMOTION_KEYS = ("schema_version", "backend", "mode", "local_goal",
                    "velocity_hint", "stair_geometry_summary",
                    "perception_confidence", "command_deadline_monotonic_ns")


@unique
class RequestedMode(Enum):
    """command request の要求 mode。

    COMMON_NAV は平地 Sport navigation(invariant 3 の第1系統)。
    昇降 3 方向は docs/02 §7 の mode と同名。HOLD は能動保持、STOP_NOW は
    即時停止要求(優先度は arbiter が server-side で決める)。
    """
    COMMON_NAV = "COMMON_NAV"
    ASCEND = "ASCEND"
    DESCEND_BACKWARD = "DESCEND_BACKWARD"
    DESCEND_FORWARD = "DESCEND_FORWARD"
    HOLD = "HOLD"
    STOP_NOW = "STOP_NOW"


@unique
class ArbiterPriority(IntEnum):
    """docs/08 §4.2 の8段priority(数値が小さいほど高優先)。
    arbiter が server-side で付与する。wire payload からは決して読まない。"""
    PHYSICAL_ESTOP_OR_HARD_FAULT = 1
    OPERATOR_STOP_OR_DISARM = 2
    SUPERVISOR_SOFT_FAULT_OR_SENSOR_STALE = 3
    HOLD_OR_CONTROLLED_STOP = 4
    WIRED_MANUAL = 5
    STAIR_STATE_MACHINE = 6
    NAV_LOCAL_PLANNER = 7
    VLM_PROPOSED_GOAL = 8


@dataclass(frozen=True)
class ServerAttribution:
    """arbiter が authenticated channel / operator lease / process identity から
    server-side で付与する信頼属性(docs/08 §4.2)。wire からは構築しない
    (from_dict を意図的に提供しない)。"""
    trusted_source_id: str
    priority: ArbiterPriority
    accepted_monotonic_timestamp_ns: int

    def __post_init__(self) -> None:
        self.validate()

    def validate(self, path: str = "attribution") -> None:
        V.req_token(self.trusted_source_id, path + ".trusted_source_id")
        if not isinstance(self.priority, ArbiterPriority):
            fail(path + ".priority", "ArbiterPriority が必要: %r" % (self.priority,))
        V.req_int(self.accepted_monotonic_timestamp_ns,
                  path + ".accepted_monotonic_timestamp_ns", 1)


@dataclass(frozen=True)
class CommandEnvelope:
    """command request の必須 wire field(docs/08 §4.2)。

    source_id は送信者の自称であり、認証・優先度付けには使わない。
    sender_timestamp は送信側 monotonic ns。expiry 判定は arbiter が
    受理時に記録した server monotonic time を基準に行う。
    """
    schema_version: str
    source_id: str
    goal_id: str
    actuation_request_id: str
    sender_timestamp: int          # 送信側 monotonic ns
    sequence: int
    expires_after_ms: int
    requested_mode: RequestedMode
    vx: float
    vy: float
    wz: float
    phase: str                     # gait/mission phase token(fault policy 選択用)
    # policy_hash: 'sha256:<64hex>'(Branch L)/ 'not_applicable'(Sport 系:
    # COMMON_NAV・Branch S)。backend との整合(Branch L run での not_applicable
    # 拒否等)は arbiter が signed run manifest と突合して判定する。
    policy_hash: str

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        if self.schema_version != SCHEMA_VERSION:
            fail("schema_version", "未対応 version: %r" % (self.schema_version,))
        V.req_token(self.source_id, "source_id")
        V.req_uuid(self.goal_id, "goal_id")
        V.req_uuid(self.actuation_request_id, "actuation_request_id")
        V.req_int(self.sender_timestamp, "sender_timestamp", 1)
        V.req_int(self.sequence, "sequence", 0)
        V.req_int(self.expires_after_ms, "expires_after_ms", 1, MAX_ENVELOPE_EXPIRES_MS)
        if not isinstance(self.requested_mode, RequestedMode):
            fail("requested_mode", "RequestedMode が必要: %r" % (self.requested_mode,))
        V.req_finite(self.vx, "vx", -MAX_ABS_VX, MAX_ABS_VX)
        V.req_finite(self.vy, "vy", -MAX_ABS_VY, MAX_ABS_VY)
        V.req_finite(self.wz, "wz", -MAX_ABS_WZ, MAX_ABS_WZ)
        V.req_token(self.phase, "phase")
        V.req_policy_hash(self.policy_hash, "policy_hash")
        if self.requested_mode in (RequestedMode.HOLD, RequestedMode.STOP_NOW):
            # 停止系 request に速度を同乗させない(意味の混在禁止)
            if self.vx != 0.0 or self.vy != 0.0 or self.wz != 0.0:
                fail("vx/vy/wz", "%s では速度は全て 0" % self.requested_mode.name)

    @classmethod
    def from_dict(cls, d: Mapping) -> "CommandEnvelope":
        """strict parser。`source_priority` 等の自称優先度 field は
        未知キー拒否で機械的に弾く(docs/08 §4.2)。"""
        V.req_mapping(d, "envelope")
        V.no_unknown_keys(d, _ENVELOPE_KEYS, "envelope")
        V.req_keys(d, _ENVELOPE_KEYS, "envelope")
        env = cls(
            schema_version=d["schema_version"],
            source_id=d["source_id"],
            goal_id=d["goal_id"],
            actuation_request_id=d["actuation_request_id"],
            sender_timestamp=d["sender_timestamp"],
            sequence=d["sequence"],
            expires_after_ms=d["expires_after_ms"],
            requested_mode=V.req_enum(d["requested_mode"], RequestedMode, "requested_mode"),
            vx=d["vx"], vy=d["vy"], wz=d["wz"],
            phase=d["phase"],
            policy_hash=d["policy_hash"],
        )  # __post_init__ が validate() を実行する
        return env

    def to_dict(self) -> dict:
        return {
            "schema_version": self.schema_version,
            "source_id": self.source_id,
            "goal_id": self.goal_id,
            "actuation_request_id": self.actuation_request_id,
            "sender_timestamp": self.sender_timestamp,
            "sequence": self.sequence,
            "expires_after_ms": self.expires_after_ms,
            "requested_mode": self.requested_mode.value,
            "vx": self.vx, "vy": self.vy, "wz": self.wz,
            "phase": self.phase,
            "policy_hash": self.policy_hash,
        }


@unique
class LocomotionBackend(Enum):
    SPORT_STAIR_API = "SPORT_STAIR_API"    # Branch S(formal API 確認まで BLOCKED)
    LEARNED_LOWCMD = "LEARNED_LOWCMD"      # Branch L


@unique
class LocomotionMode(Enum):
    """docs/02 §7。上りと下降は別 skill・別 Gate(invariant 11)。
    DESCEND_* を「負の速度の ASCEND」で表現してはならない。"""
    ASCEND = "ASCEND"
    DESCEND_BACKWARD = "DESCEND_BACKWARD"
    DESCEND_FORWARD = "DESCEND_FORWARD"
    HOLD = "HOLD"


# mode が要求する StairModel.direction(HOLD は幾何不要)
_MODE_REQUIRED_DIRECTION = {
    LocomotionMode.ASCEND: StairDirection.UP,
    LocomotionMode.DESCEND_BACKWARD: StairDirection.DOWN,
    LocomotionMode.DESCEND_FORWARD: StairDirection.DOWN,
}


@dataclass(frozen=True)
class StairGeometrySummary:
    """LocomotionCommand に載せる StairModel の要約。
    (v1 の具体 field は docs/02 §7 の `stair_geometry_summary` を最小実装した
    提案であり、Phase 4 の知覚実装で review 対象。)"""
    stair_id: str
    direction: StairDirection
    visible_steps: int
    riser_height_min_m: float
    riser_height_max_m: float
    fresh_coverage_landing: float

    def validate(self, path: str = "stair_geometry_summary") -> None:
        V.req_token(self.stair_id, path + ".stair_id")
        if not isinstance(self.direction, StairDirection):
            fail(path + ".direction", "StairDirection が必要")
        V.req_int(self.visible_steps, path + ".visible_steps", 1, 16)
        lo = V.req_finite(self.riser_height_min_m, path + ".riser_height_min_m", 0.01, 0.30)
        hi = V.req_finite(self.riser_height_max_m, path + ".riser_height_max_m", 0.01, 0.30)
        if lo > hi:
            fail(path, "riser min %g > max %g" % (lo, hi))
        V.req_score(self.fresh_coverage_landing, path + ".fresh_coverage_landing")

    @classmethod
    def from_dict(cls, d: Mapping, path: str = "stair_geometry_summary") -> "StairGeometrySummary":
        V.req_mapping(d, path)
        V.no_unknown_keys(d, _SUMMARY_KEYS, path)
        V.req_keys(d, _SUMMARY_KEYS, path)
        obj = cls(stair_id=d["stair_id"],
                  direction=V.req_enum(d["direction"], StairDirection, path + ".direction"),
                  visible_steps=d["visible_steps"],
                  riser_height_min_m=d["riser_height_min_m"],
                  riser_height_max_m=d["riser_height_max_m"],
                  fresh_coverage_landing=d["fresh_coverage_landing"])
        obj.validate(path)
        return obj

    def to_dict(self) -> dict:
        return {"stair_id": self.stair_id, "direction": self.direction.value,
                "visible_steps": self.visible_steps,
                "riser_height_min_m": self.riser_height_min_m,
                "riser_height_max_m": self.riser_height_max_m,
                "fresh_coverage_landing": self.fresh_coverage_landing}


@dataclass(frozen=True)
class LocomotionCommand:
    """Locomotion skill への唯一の入力型(docs/02 §7)。

    backend は arm 前の signed run manifest で固定され、run 中に切り替えない
    (invariant 6)。同一 command の S/L 二重配送は禁止 — 配送の排他は
    Exclusive Actuation Gateway の責務で、本契約は型を規定する。
    """
    schema_version: str
    backend: LocomotionBackend
    mode: LocomotionMode
    local_goal: tuple              # (x, y, yaw, z)
    velocity_hint: tuple           # (vx, vy, wz)
    stair_geometry_summary: Optional[StairGeometrySummary]
    perception_confidence: float
    command_deadline_monotonic_ns: int

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        if self.schema_version != SCHEMA_VERSION:
            fail("schema_version", "未対応 version: %r" % (self.schema_version,))
        if not isinstance(self.backend, LocomotionBackend):
            fail("backend", "LocomotionBackend が必要")
        if not isinstance(self.mode, LocomotionMode):
            fail("mode", "LocomotionMode が必要")
        if (self.stair_geometry_summary is not None
                and not isinstance(self.stair_geometry_summary, StairGeometrySummary)):
            fail("stair_geometry_summary",
                 "StairGeometrySummary か null が必要: %r" % (self.stair_geometry_summary,))
        V.req_float_list(self.local_goal, "local_goal", n=4, lo=-100.0, hi=100.0)
        vh = V.req_float_list(self.velocity_hint, "velocity_hint", n=3)
        V.req_finite(vh[0], "velocity_hint[0]", -MAX_ABS_VX, MAX_ABS_VX)
        V.req_finite(vh[1], "velocity_hint[1]", -MAX_ABS_VY, MAX_ABS_VY)
        V.req_finite(vh[2], "velocity_hint[2]", -MAX_ABS_WZ, MAX_ABS_WZ)
        V.req_score(self.perception_confidence, "perception_confidence")
        V.req_int(self.command_deadline_monotonic_ns,
                  "command_deadline_monotonic_ns", 1)

        need = _MODE_REQUIRED_DIRECTION.get(self.mode)
        if need is not None:
            # 昇降 mode は方向一致の幾何 summary を必須にする(方向混同の型レベル防止)
            if self.stair_geometry_summary is None:
                fail("stair_geometry_summary", "%s では必須" % self.mode.name)
            self.stair_geometry_summary.validate()
            if self.stair_geometry_summary.direction is not need:
                fail("stair_geometry_summary.direction",
                     "%s には direction=%s が必要: %s"
                     % (self.mode.name, need.name,
                        self.stair_geometry_summary.direction.name))
        else:
            if self.stair_geometry_summary is not None:
                self.stair_geometry_summary.validate()
        if self.mode is LocomotionMode.HOLD:
            if any(v != 0.0 for v in self.velocity_hint):
                fail("velocity_hint", "HOLD では速度 hint は全て 0")

    @classmethod
    def from_dict(cls, d: Mapping) -> "LocomotionCommand":
        V.req_mapping(d, "locomotion")
        V.no_unknown_keys(d, _LOCOMOTION_KEYS, "locomotion")
        V.req_keys(d, _LOCOMOTION_KEYS, "locomotion")
        sg = d["stair_geometry_summary"]
        cmd = cls(
            schema_version=d["schema_version"],
            backend=V.req_enum(d["backend"], LocomotionBackend, "backend"),
            mode=V.req_enum(d["mode"], LocomotionMode, "mode"),
            local_goal=tuple(d["local_goal"]),
            velocity_hint=tuple(d["velocity_hint"]),
            stair_geometry_summary=None if sg is None else StairGeometrySummary.from_dict(sg),
            perception_confidence=d["perception_confidence"],
            command_deadline_monotonic_ns=d["command_deadline_monotonic_ns"],
        )  # __post_init__ が validate() を実行する
        return cmd

    def to_dict(self) -> dict:
        return {
            "schema_version": self.schema_version,
            "backend": self.backend.value,
            "mode": self.mode.value,
            "local_goal": list(self.local_goal),
            "velocity_hint": list(self.velocity_hint),
            "stair_geometry_summary": (None if self.stair_geometry_summary is None
                                       else self.stair_geometry_summary.to_dict()),
            "perception_confidence": self.perception_confidence,
            "command_deadline_monotonic_ns": self.command_deadline_monotonic_ns,
        }
