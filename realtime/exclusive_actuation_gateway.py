"""realtime.exclusive_actuation_gateway — Exclusive Actuation Gateway(純ロジック)。

正本: docs/02 §7・§10(配置と排他)、docs/08 §4.2(mode transition handshake:
      request → acknowledged inactive → owner generation change → enable)、
      docs/08 §4.3(policy hash 不一致は1フレームも motor へ出さない)、
      docs/09 §5(generation は runtime 採番・append-only transition stream)、
      docs/CLAUDE.md invariant 3(排他)・6(backend は run 中固定)。

contracts/README.md の gateway 受入条件を実装する:
  (a) release_latch は認証済み operator lease 経由のみ。
  (b) arbiter の Directive.stop_state は latch 種別であり stop FSM の target
      ではない — 物理進行は safety.stop_transitions の表に従って gateway が進める。
  (c) ACTIVE_HOLD 滞在中に STOP_NOW latch を受けたら ACTIVE_HOLD 滞在を維持し、
      未定義遷移を評価しない。
  (+) policy_hash × run manifest の突合(Branch L run での stair mode +
      not_applicable を拒否)。
  (+) 不正 payload の STOP_NOW は拒否をエスカレートして latch(fail toward stop)。

本 module は I/O を持たない。backend への実送信・DDS・thread は実機接続 task
(LowCmd servo / Sport adapter)で行い、ここでは「何を送ってよいか」の判定と
authority 管理のみを行う(invariant 5)。
"""
from dataclasses import dataclass
from enum import Enum, unique
from typing import List, Mapping, Optional

from contracts import _validation as V
from contracts.command_envelope import (
    ArbiterPriority, CommandEnvelope, LocomotionBackend, RequestedMode,
    ServerAttribution,
)
from contracts.errors import ContractViolation
from contracts.stop_states import StopState
from mission.command_arbiter import (
    CommandArbiter, Directive, DirectiveKind, OperatorReset,
)
from safety.stop_transitions import (
    TransitionContext, allowed_transitions_with_guards, check_transition,
)

_STAIR_MODES = (RequestedMode.ASCEND, RequestedMode.DESCEND_BACKWARD,
                RequestedMode.DESCEND_FORWARD)


@unique
class Channel(Enum):
    """排他 actuation channel(invariant 3)。stair channel は manifest の
    selected_backend で S/L どちらか一方に固定される(invariant 6)。"""
    COMMON_NAV = "COMMON_NAV"
    STAIR = "STAIR"


@unique
class HandshakeState(Enum):
    """mode transition handshake(docs/08 §4.2)。順序 skip は契約違反。"""
    DISABLED = "DISABLED"
    REQUESTED = "REQUESTED"
    ACK_INACTIVE = "ACK_INACTIVE"
    GENERATION_ASSIGNED = "GENERATION_ASSIGNED"
    ENABLED = "ENABLED"


_HANDSHAKE_ORDER = (HandshakeState.DISABLED, HandshakeState.REQUESTED,
                    HandshakeState.ACK_INACTIVE,
                    HandshakeState.GENERATION_ASSIGNED, HandshakeState.ENABLED)


@dataclass(frozen=True)
class RunManifestControl:
    """signed run manifest の control 節(docs/09 §5 の subset)。
    arm 前に固定され、run 中に変更しない(invariant 6)。"""
    run_id: str
    selected_backend: LocomotionBackend
    policy_hash: str              # Branch L: 'sha256:<64hex>'。Branch S: 'not_applicable'
    operator_lease_id: str        # 認証済み operator lease(gateway 受入条件 (a))

    def __post_init__(self) -> None:
        V.req_token(self.run_id, "manifest.run_id")
        if not isinstance(self.selected_backend, LocomotionBackend):
            raise ContractViolation("manifest.selected_backend",
                                    "LocomotionBackend が必要")
        V.req_policy_hash(self.policy_hash, "manifest.policy_hash")
        V.req_uuid(self.operator_lease_id, "manifest.operator_lease_id")
        if self.selected_backend is LocomotionBackend.LEARNED_LOWCMD \
                and self.policy_hash == "not_applicable":
            raise ContractViolation(
                "manifest.policy_hash",
                "Branch L の manifest は policy hash 必須(docs/08 §4.3)")


@dataclass(frozen=True)
class TransitionRecord:
    """append-only runtime_transition_stream の1件(docs/09 §5)。"""
    channel: Channel
    from_state: HandshakeState
    to_state: HandshakeState
    generation: Optional[int]
    at_ns: int
    reason: str


@unique
class ActionKind(Enum):
    NOOP = "NOOP"                      # 送信なし(enabled channel なし等)
    FORWARD = "FORWARD"                # 受理 command を enabled backend へ
    STOP_PROGRESS = "STOP_PROGRESS"    # 停止列の進行(stop FSM に従う)
    HOLD = "HOLD"                      # 能動保持の維持


@dataclass(frozen=True)
class GatewayAction:
    kind: ActionKind
    channel: Optional[Channel]
    stop_state: Optional[StopState]
    envelope: Optional[CommandEnvelope]
    reason: str


@dataclass(frozen=True)
class RobotStatusFlags:
    """検証済みの物理事実(呼び出し側=supervisor/推定器が True を保証)。
    stop_transitions の guard に渡す(fail-closed: 未検証は False)。"""
    settled_below_thresholds: bool = False
    stable_contact_verified: bool = False
    sport_authority_active: bool = False
    critical_fault: bool = False

    def __post_init__(self) -> None:
        for name in ("settled_below_thresholds", "stable_contact_verified",
                     "sport_authority_active", "critical_fault"):
            V.req_bool(getattr(self, name), "status." + name)


class ExclusiveActuationGateway:
    """唯一の actuator authority 切替点(純ロジック)。

    使い方(実機接続 task 側):
        gw = ExclusiveActuationGateway(manifest, arbiter)
        gw.request_channel(Channel.COMMON_NAV, now); gw.ack_inactive(...);
        gw.assign_generation(...); gw.enable(...)
        act = gw.tick(status_flags, now)   # 毎 tick
    """

    def __init__(self, manifest: RunManifestControl, arbiter: CommandArbiter):
        if not isinstance(manifest, RunManifestControl):
            raise ContractViolation("manifest", "RunManifestControl が必要")
        if not isinstance(arbiter, CommandArbiter):
            raise ContractViolation("arbiter", "CommandArbiter が必要")
        self.manifest = manifest
        self.arbiter = arbiter
        self._hs = {Channel.COMMON_NAV: HandshakeState.DISABLED,
                    Channel.STAIR: HandshakeState.DISABLED}
        self._generation = 0
        self._stream: List[TransitionRecord] = []
        # 停止列の現在状態。開始時は能動保持(DISARM 相当の運動なし状態)。
        self._motion_stop_state: Optional[StopState] = StopState.ACTIVE_HOLD
        # None = 運動中(MOVING、stop FSM の外)

    # ---------- handshake(docs/08 §4.2 の4段) ----------

    def _record(self, ch: Channel, frm: HandshakeState, to: HandshakeState,
                gen: Optional[int], now_ns: int, reason: str) -> None:
        self._stream.append(TransitionRecord(ch, frm, to, gen, now_ns, reason))

    def _advance(self, ch: Channel, expect: HandshakeState,
                 to: HandshakeState, now_ns: int, reason: str) -> None:
        V.req_int(now_ns, "now_ns", 1)
        cur = self._hs[ch]
        if cur is not expect:
            raise ContractViolation(
                "handshake.%s" % ch.name,
                "順序違反: %s から %s へは進めない(期待: %s)"
                % (cur.name, to.name, expect.name))
        # 排他: 他 channel が DISABLED 以外なら REQUESTED にすら進めない
        if to is not HandshakeState.DISABLED:
            other = Channel.STAIR if ch is Channel.COMMON_NAV else Channel.COMMON_NAV
            if self._hs[other] is not HandshakeState.DISABLED:
                raise ContractViolation(
                    "handshake.%s" % ch.name,
                    "排他違反: %s が %s のため進行不可(invariant 3)"
                    % (other.name, self._hs[other].name))
        self._hs[ch] = to
        self._record(ch, cur, to, None, now_ns, reason)

    def request_channel(self, ch: Channel, now_ns: int) -> None:
        if not isinstance(ch, Channel):
            raise ContractViolation("channel", "Channel が必要")
        self._advance(ch, HandshakeState.DISABLED, HandshakeState.REQUESTED,
                      now_ns, "transition requested")

    def ack_inactive(self, ch: Channel, now_ns: int) -> None:
        """前 owner の inactive 確認(実機では実測 ack。ここでは手続きの順序を強制)。"""
        self._advance(ch, HandshakeState.REQUESTED, HandshakeState.ACK_INACTIVE,
                      now_ns, "previous owner acknowledged inactive")

    def assign_generation(self, ch: Channel, now_ns: int) -> int:
        cur = self._hs[ch]
        if cur is not HandshakeState.ACK_INACTIVE:
            raise ContractViolation("handshake.%s" % ch.name,
                                    "generation は ACK_INACTIVE の後のみ")
        self._generation += 1
        self._hs[ch] = HandshakeState.GENERATION_ASSIGNED
        self._record(ch, cur, HandshakeState.GENERATION_ASSIGNED,
                     self._generation, now_ns, "owner generation assigned")
        return self._generation

    def enable(self, ch: Channel, now_ns: int) -> None:
        self._advance(ch, HandshakeState.GENERATION_ASSIGNED,
                      HandshakeState.ENABLED, now_ns, "enabled")

    def disable(self, ch: Channel, now_ns: int) -> None:
        cur = self._hs[ch]
        V.req_int(now_ns, "now_ns", 1)
        if cur is HandshakeState.DISABLED:
            return
        self._hs[ch] = HandshakeState.DISABLED
        self._record(ch, cur, HandshakeState.DISABLED, None, now_ns, "disabled")

    def enabled_channel(self) -> Optional[Channel]:
        enabled = [c for c, s in self._hs.items() if s is HandshakeState.ENABLED]
        if len(enabled) > 1:  # 構造上起きないはずだが fail-closed で検査
            raise ContractViolation("handshake", "複数 channel が ENABLED(排他違反)")
        return enabled[0] if enabled else None

    def transition_stream(self) -> tuple:
        """append-only(読み取りは copy を返す)。"""
        return tuple(self._stream)

    # ---------- command 入口 ----------

    def submit_raw(self, payload: Mapping, attr: ServerAttribution,
                   now_ns: int):
        """wire payload の入口。契約違反の STOP_NOW は拒否をエスカレートして
        latch する(contracts/README.md「STOP_NOW 拒否時の受け側必須要件」)。
        エスカレートは認証済み lease 文脈のみ(attr は server-side 付与済み)。"""
        try:
            env = CommandEnvelope.from_dict(payload)
        except ContractViolation as e:
            wants_stop = isinstance(payload, Mapping) \
                and payload.get("requested_mode") == "STOP_NOW"
            if wants_stop:
                self.arbiter.latch_fault(
                    ArbiterPriority.OPERATOR_STOP_OR_DISARM,
                    StopState.STOP_NOW,
                    "malformed STOP_NOW escalated: %s" % e, now_ns)
                return None  # 停止は latch 済み(fail toward stop)
            raise
        return self.arbiter.submit(env, attr, now_ns)

    def release_latch(self, priority: ArbiterPriority, reset: OperatorReset,
                      operator_lease_id: str, now_ns: int):
        """受入条件 (a): 認証済み operator lease 経由のみ。manifest の lease と
        一致しない解除要求は arbiter へ転送しない(fail-closed)。"""
        V.req_uuid(operator_lease_id, "operator_lease_id")
        if operator_lease_id != self.manifest.operator_lease_id:
            raise ContractViolation(
                "operator_lease_id",
                "manifest の operator lease と不一致 — latch 解除は転送しない")
        return self.arbiter.release_latch(priority, reset, now_ns)

    # ---------- 毎 tick の実行判定 ----------

    def _check_policy_hash(self, env: CommandEnvelope, now_ns: int) -> Optional[str]:
        """stair mode の policy hash を manifest と突合(docs/08 §4.3)。
        不一致/欠落は1フレームも通さない — 拒否して supervisor latch。"""
        if env.requested_mode not in _STAIR_MODES:
            return None
        if env.policy_hash != self.manifest.policy_hash:
            reason = ("policy hash mismatch: envelope=%s manifest=%s"
                      % (env.policy_hash, self.manifest.policy_hash))
            self.arbiter.latch_fault(
                ArbiterPriority.SUPERVISOR_SOFT_FAULT_OR_SENSOR_STALE,
                StopState.CONTROLLED_STOP, reason, now_ns)
            return reason
        return None

    def _mode_channel(self, mode: RequestedMode) -> Channel:
        return Channel.STAIR if mode in _STAIR_MODES else Channel.COMMON_NAV

    def tick(self, status: RobotStatusFlags, now_ns: int) -> GatewayAction:
        """arbiter の Directive を物理進行へ写像する(受入条件 (b)(c))。"""
        if not isinstance(status, RobotStatusFlags):
            raise ContractViolation("status", "RobotStatusFlags が必要")
        d: Directive = self.arbiter.current(now_ns)
        ctx = TransitionContext(
            critical_fault=status.critical_fault,
            settled_below_thresholds=status.settled_below_thresholds,
            stable_contact_verified=status.stable_contact_verified,
            sport_authority_active=status.sport_authority_active,
        )

        if d.kind is DirectiveKind.EXECUTE:
            env = d.envelope
            hash_err = self._check_policy_hash(env, now_ns)
            if hash_err is not None:
                # latch 済み → 次 tick から LATCHED_STOP。今 tick は送信しない。
                return GatewayAction(ActionKind.NOOP, None, None, None, hash_err)
            ch = self._mode_channel(env.requested_mode)
            if self._hs[ch] is not HandshakeState.ENABLED:
                return GatewayAction(ActionKind.NOOP, ch, None, None,
                                     "%s が ENABLED でない(handshake 未完了)" % ch.name)
            self._motion_stop_state = None  # 運動中(stop FSM の外)
            return GatewayAction(ActionKind.FORWARD, ch, None, env, "forward")

        # --- 停止系(LATCHED_STOP / CONTROLLED_STOP / IDLE_HOLD) ---
        want = d.stop_state  # latch 種別 or CONTROLLED_STOP or ACTIVE_HOLD
        cur = self._motion_stop_state

        if cur is None:
            # 運動中に停止指示 → 停止列へ入る(mission FSM の運動状態からの
            # 入口。stop FSM 内の遷移表は適用外 — stop_transitions docstring)
            entry = want if want in (StopState.STOP_NOW,
                                     StopState.DAMP_CRITICAL_STOP,
                                     StopState.PHYSICAL_ESTOP_FUNCTION) \
                else StopState.CONTROLLED_STOP
            self._motion_stop_state = entry
            return GatewayAction(ActionKind.STOP_PROGRESS, self.enabled_channel(),
                                 entry, None, "運動→停止列に入る: %s" % entry.name)

        # 受入条件 (c): ACTIVE_HOLD 滞在中の STOP_NOW latch は滞在で充足
        # (自己遷移 — 未定義遷移を評価しない)
        if cur is StopState.ACTIVE_HOLD and want in (StopState.STOP_NOW,
                                                     StopState.CONTROLLED_STOP,
                                                     StopState.ACTIVE_HOLD):
            return GatewayAction(ActionKind.HOLD, self.enabled_channel(),
                                 StopState.ACTIVE_HOLD, None,
                                 "ACTIVE_HOLD 滞在で停止要求を充足")

        # 停止列の進行(safety.stop_transitions の表に従う — 受入条件 (b))
        nxt = self._next_stop_step(cur, want)
        if nxt is cur:
            kind = (ActionKind.HOLD if cur is StopState.ACTIVE_HOLD
                    else ActionKind.STOP_PROGRESS)
            return GatewayAction(kind, self.enabled_channel(), cur, None,
                                 "%s を維持" % cur.name)
        # guard 未成立は「まだ進めない」= 現状態を維持(fail-closed だが例外に
        # しない — 整定/接地確認を待つのは正常系)。未定義遷移(コードバグ)は
        # check_transition が ContractViolation で表面化させる。
        guards = allowed_transitions_with_guards().get((cur, nxt))
        if guards is not None and not all(getattr(ctx, g) for g in guards):
            unmet = [g for g in guards if not getattr(ctx, g)]
            return GatewayAction(ActionKind.STOP_PROGRESS, self.enabled_channel(),
                                 cur, None,
                                 "guard 未成立(%s)— %s を維持" % (unmet, cur.name))
        check_transition(cur, nxt, ctx)  # 未定義遷移なら fail-closed で raise
        self._motion_stop_state = nxt
        kind = ActionKind.HOLD if nxt is StopState.ACTIVE_HOLD else ActionKind.STOP_PROGRESS
        return GatewayAction(kind, self.enabled_channel(), nxt, None,
                             "%s -> %s" % (cur.name, nxt.name))

    def _next_stop_step(self, cur: StopState, want: Optional[StopState]) -> StopState:
        """停止列の次の一歩を決める(表で許可されるかは check_transition が判定)。"""
        if want is StopState.DAMP_CRITICAL_STOP:
            return StopState.DAMP_CRITICAL_STOP
        if want is StopState.PHYSICAL_ESTOP_FUNCTION:
            return StopState.PHYSICAL_ESTOP_FUNCTION
        if cur is StopState.STOP_NOW:
            return StopState.CONTROLLED_STOP
        if cur is StopState.CONTROLLED_STOP:
            return StopState.ACTIVE_HOLD
        return cur
