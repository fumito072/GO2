"""mission.executive — 決定的 Mission FSM + affordance validator(純ロジック)。

正本: docs/02 §5(state machine)・§4.3(前提条件)、docs/06 §8.4(validation)。
v1.1 拡張: EXPLORE_AND_MAP のための EXPLORING 状態を追加(docs/10 §3)。
目標再定義(音声/NL操作+自律探索)により、階段系状態は将来 task 用に温存し、
本 MVP では探索/待避/停止経路のみを実際に駆動する。

原則:
  - JSON として正しい GoalSpec でも、state と affordance validator が実行可否を
    決める(docs/02 §4.1)。拒否は理由を返し、勝手に「最も近い解釈」で実行しない。
  - STOP_NOW は状態を問わず受理し、arbiter へ直送済みである前提で FSM を
    ACTIVE_HOLD へ遷移させる(resume_context を保存、自動再開しない)。
  - CRITICAL_STOP からの復帰は manual recovery のみ(→ DISARMED)。
  - 本クラスは I/O を持たない。時刻は now_ns 注入。
"""
from dataclasses import dataclass, field
from enum import Enum, unique
from typing import Optional, Set, Tuple

from contracts import _validation as V
from contracts.errors import ContractViolation
from contracts.goal_spec import (
    GoalSpec, Intent, CompletionPredicate, SUPPORTED_SCHEMA_VERSIONS,
)

# parser の allowlist(docs/06 §8.4: schema/parser/model version が allowlist 内)
DEFAULT_PARSER_ALLOWLIST = ("grammar-ja-v1", "text-parser-v1")


@unique
class MissionState(Enum):
    """docs/02 §5 の canonical 状態 + v1.1 の EXPLORING。"""
    DISARMED = "DISARMED"
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    EXPLORING = "EXPLORING"                      # v1.1(docs/10 §3)
    AT_BASE_HOLD = "AT_BASE_HOLD"
    STAIR_BACKEND_PREFLIGHT = "STAIR_BACKEND_PREFLIGHT"
    ASCENDING = "ASCENDING"
    TOP_HOLD = "TOP_HOLD"
    HOLD_TIMEOUT_RECOVERY = "HOLD_TIMEOUT_RECOVERY"
    DESCENT_PREFLIGHT = "DESCENT_PREFLIGHT"
    DESCENDING = "DESCENDING"
    BOTTOM_HOLD = "BOTTOM_HOLD"
    ACTIVE_HOLD = "ACTIVE_HOLD"
    CRITICAL_STOP = "CRITICAL_STOP"


# goal を受理できる (state, intent) → 遷移先(docs/02 §5 + v1.1)
_GOAL_TRANSITIONS = {
    (MissionState.IDLE, Intent.NAVIGATE_TO_STAIR_APPROACH): MissionState.NAVIGATING,
    (MissionState.IDLE, Intent.NAVIGATE_TO_WAYPOINT): MissionState.NAVIGATING,
    (MissionState.IDLE, Intent.EXPLORE_AND_MAP): MissionState.EXPLORING,
    (MissionState.ACTIVE_HOLD, Intent.NAVIGATE_TO_WAYPOINT): MissionState.NAVIGATING,
    (MissionState.ACTIVE_HOLD, Intent.EXPLORE_AND_MAP): MissionState.EXPLORING,
    (MissionState.AT_BASE_HOLD, Intent.ASCEND_STAIRS): MissionState.STAIR_BACKEND_PREFLIGHT,
    (MissionState.TOP_HOLD, Intent.DESCEND_STAIRS): MissionState.DESCENT_PREFLIGHT,
}

# completion predicate → (許可される現状態, 遷移先)
_COMPLETION_TRANSITIONS = {
    CompletionPredicate.EXPLORATION_COMPLETE:
        (MissionState.EXPLORING, MissionState.ACTIVE_HOLD),
    CompletionPredicate.WAYPOINT_REACHED:
        (MissionState.NAVIGATING, MissionState.ACTIVE_HOLD),
    CompletionPredicate.STAIR_APPROACH_POSE_REACHED:
        (MissionState.NAVIGATING, MissionState.AT_BASE_HOLD),
    CompletionPredicate.TOP_LANDING_STABLE:
        (MissionState.ASCENDING, MissionState.TOP_HOLD),
    CompletionPredicate.BOTTOM_LANDING_STABLE:
        (MissionState.DESCENDING, MissionState.BOTTOM_HOLD),
}

# STOP_NOW / soft fault で ACTIVE_HOLD へ落ちる運動状態
_MOVING_STATES = (MissionState.NAVIGATING, MissionState.EXPLORING,
                  MissionState.ASCENDING, MissionState.DESCENDING)


@dataclass(frozen=True)
class AffordanceContext:
    """呼び出し側(supervisor/認証層)が検証済み事実だけを True にする。
    未検証は False(fail-closed)。"""
    operator_lease_valid: bool = False
    supervisor_ok: bool = False
    stair_geometry_valid: bool = False   # 階段 intent のみ必要

    def __post_init__(self) -> None:
        for name in ("operator_lease_valid", "supervisor_ok",
                     "stair_geometry_valid"):
            V.req_bool(getattr(self, name), "ctx." + name)


@dataclass(frozen=True)
class GoalDecision:
    accepted: bool
    reason: str
    new_state: Optional[MissionState] = None


@dataclass
class _ActiveGoal:
    spec: GoalSpec
    accepted_at_ns: int
    must_start_by_ns: int


class MissionExecutive:
    """決定的 Mission FSM。goal の受理・完了・停止・fault の遷移のみを司る。
    command の発行(explorer→envelope 化)は呼び出し側 loop が行い、
    必ず arbiter/gateway を通す(invariant 2)。"""

    def __init__(self, expected_operator_lease_id: str,
                 parser_allowlist: Tuple[str, ...] = DEFAULT_PARSER_ALLOWLIST):
        V.req_uuid(expected_operator_lease_id, "expected_operator_lease_id")
        self._lease = expected_operator_lease_id
        self._allow_parsers = tuple(parser_allowlist)
        self._state = MissionState.DISARMED
        self._active: Optional[_ActiveGoal] = None
        self._dedup: Set[tuple] = set()
        self._resume_context: Optional[MissionState] = None
        self._last_now_ns = 0

    # ---------- 基本 ----------

    @property
    def state(self) -> MissionState:
        return self._state

    def active_goal(self) -> Optional[GoalSpec]:
        return self._active.spec if self._active else None

    def _clock(self, now_ns: int) -> None:
        V.req_int(now_ns, "now_ns", 1)
        if now_ns < self._last_now_ns:
            raise ContractViolation("now_ns", "monotonic 逆行(%d < %d)"
                                    % (now_ns, self._last_now_ns))
        self._last_now_ns = now_ns

    def _enter(self, state: MissionState) -> None:
        self._state = state

    # ---------- arm / disarm(docs/02 §5) ----------

    def arm(self, self_check_ok: bool, now_ns: int) -> GoalDecision:
        self._clock(now_ns)
        V.req_bool(self_check_ok, "self_check_ok")
        if self._state is not MissionState.DISARMED:
            return GoalDecision(False, "arm は DISARMED からのみ")
        if not self_check_ok:
            return GoalDecision(False, "self-check 未通過(fail-closed)")
        self._enter(MissionState.IDLE)
        return GoalDecision(True, "armed", MissionState.IDLE)

    def disarm(self, now_ns: int) -> None:
        self._clock(now_ns)
        self._active = None
        self._enter(MissionState.DISARMED)

    def manual_recovery(self, now_ns: int) -> GoalDecision:
        """CRITICAL_STOP → DISARMED(manual recovery only — docs/02 §5)。"""
        self._clock(now_ns)
        if self._state is not MissionState.CRITICAL_STOP:
            return GoalDecision(False, "manual recovery は CRITICAL_STOP からのみ")
        self._active = None
        self._enter(MissionState.DISARMED)
        return GoalDecision(True, "recovered to DISARMED", MissionState.DISARMED)

    # ---------- goal 受理(affordance validation — docs/06 §8.4) ----------

    def accept_goal(self, spec: GoalSpec, ctx: AffordanceContext,
                    now_ns: int) -> GoalDecision:
        self._clock(now_ns)
        if not isinstance(spec, GoalSpec):
            raise ContractViolation("spec", "GoalSpec が必要")
        if not isinstance(ctx, AffordanceContext):
            raise ContractViolation("ctx", "AffordanceContext が必要")

        # STOP_NOW は FSM の goal queue を通らない(arbiter 直送 — docs/06 §8.2)。
        # ここに来た場合は配線ミスなので明示拒否する。
        if spec.intent is Intent.STOP_NOW:
            return GoalDecision(False,
                                "STOP_NOW は accept_goal を通さない(arbiter 直送)")

        # version / parser allowlist
        if spec.schema_version not in SUPPORTED_SCHEMA_VERSIONS:
            return GoalDecision(False, "schema version 不許可")
        if spec.confidence.parser_version not in self._allow_parsers:
            return GoalDecision(False, "parser version が allowlist 外: %r"
                                % spec.confidence.parser_version)
        # operator lease(認証は上位層。ここでは一致のみ)
        if spec.source.operator_lease_id != self._lease:
            return GoalDecision(False, "operator lease 不一致")
        if not ctx.operator_lease_valid:
            return GoalDecision(False, "operator lease 未検証(fail-closed)")
        if not ctx.supervisor_ok:
            return GoalDecision(False, "Safety Supervisor 未許可(fail-closed)")

        # 再送重複(docs/06 §4.2 — OR 意味論)
        keys = spec.dedup_keys()
        if any(k in self._dedup for k in keys):
            return GoalDecision(False, "重複 goal(idempotent 拒否)")

        # state × intent(docs/02 §5)
        nxt = _GOAL_TRANSITIONS.get((self._state, spec.intent))
        if nxt is None:
            return GoalDecision(False, "state %s で intent %s は実行不可"
                                % (self._state.name, spec.intent.name))
        # 階段 intent は幾何検証必須(invariant 10)
        if spec.intent in (Intent.ASCEND_STAIRS, Intent.DESCEND_STAIRS) \
                and not ctx.stair_geometry_valid:
            return GoalDecision(False, "階段幾何が未検証(fail-closed)")

        # 受理: must_start_by は受理時 monotonic から数える(docs/02 §4.1)
        for k in keys:
            self._dedup.add(k)
        self._active = _ActiveGoal(
            spec=spec, accepted_at_ns=now_ns,
            must_start_by_ns=now_ns + spec.expires_after_ms * 1_000_000)
        self._enter(nxt)
        return GoalDecision(True, "accepted: %s" % spec.intent.name, nxt)

    def goal_started_in_time(self, now_ns: int) -> bool:
        """開始可能期限(must_start_by)の判定。超過時は呼び出し側が
        abandon_goal() で待避させる。"""
        self._clock(now_ns)
        if self._active is None:
            return False
        return now_ns <= self._active.must_start_by_ns

    def abandon_goal(self, reason: str, now_ns: int) -> None:
        """goal の放棄(期限超過等)→ ACTIVE_HOLD(勝手に別解釈で続行しない)。"""
        self._clock(now_ns)
        V.req_str(reason, "reason", max_len=256)
        self._active = None
        if self._state in _MOVING_STATES:
            self._enter(MissionState.ACTIVE_HOLD)
        elif self._state is not MissionState.DISARMED:
            self._enter(MissionState.ACTIVE_HOLD)

    # ---------- 完了 / 停止 / fault ----------

    def notify_completion(self, predicate: CompletionPredicate,
                          now_ns: int) -> GoalDecision:
        self._clock(now_ns)
        if not isinstance(predicate, CompletionPredicate):
            raise ContractViolation("predicate", "CompletionPredicate が必要")
        entry = _COMPLETION_TRANSITIONS.get(predicate)
        if entry is None or self._state is not entry[0]:
            return GoalDecision(False, "state %s で completion %s は不整合"
                                % (self._state.name, predicate.name))
        self._active = None
        self._enter(entry[1])
        return GoalDecision(True, "completed: %s" % predicate.name, entry[1])

    def notify_stop_now(self, now_ns: int) -> GoalDecision:
        """STOP_NOW の FSM 側反映(arbiter への latch は gateway/parser 経路)。
        resume_context を保存し、自動再開しない(docs/02 §5)。"""
        self._clock(now_ns)
        if self._state in _MOVING_STATES:
            self._resume_context = self._state
            self._active = None
            self._enter(MissionState.ACTIVE_HOLD)
            return GoalDecision(True, "STOP_NOW → ACTIVE_HOLD",
                                MissionState.ACTIVE_HOLD)
        return GoalDecision(True, "既に非運動状態(%s)" % self._state.name,
                            self._state)

    def notify_critical_fault(self, reason: str, now_ns: int) -> GoalDecision:
        self._clock(now_ns)
        V.req_str(reason, "reason", max_len=256)
        if self._state is MissionState.DISARMED:
            return GoalDecision(True, "DISARMED のまま", self._state)
        self._active = None
        self._enter(MissionState.CRITICAL_STOP)
        return GoalDecision(True, "critical fault → CRITICAL_STOP",
                            MissionState.CRITICAL_STOP)

    def resume_context(self) -> Optional[MissionState]:
        return self._resume_context
