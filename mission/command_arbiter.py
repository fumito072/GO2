"""mission.command_arbiter — Command Arbiter(純ロジック、Gate 0 対象)。

正本: docs/08 §4.2(priority 8段・latch 規則・expiry 規則・server-side 付与)、
      docs/02 §10(配置)、docs/CLAUDE.md invariant 2。

規則(docs/08 §4.2 ほか):
  - priority は arbiter が server-side(ServerAttribution)で付与する。
    payload の自称 priority は CommandEnvelope の未知キー拒否で既に弾かれる。
  - 低優先 command は高優先 command の latch を解除できない。latch 中は
    いかなる通常 command も latch を解除・上書きできない(解除は operator
    reset のみ)。
  - STOP_NOW は前提条件なしで受理する(docs/CLAUDE.md Phase 3)。停止要求の
    黙殺こそ fail-open なので、stale sequence・timestamp 異常でも latch する
    (anti-replay は運動 command のみに適用。重複 replay された停止は無害)。
  - command expiry 後はゼロを推測せず、明示的に Controlled Stop へ遷移する。
    到着時点で既に期限切れの command(DOA)は受理しない。
  - 安全判定は monotonic time のみ。時計の逆行・未来の受理時刻は fail-closed
    (clock jump の自動 test — docs/CLAUDE.md Phase 1)。
  - latch は自動復帰しない。健全 packet が 1 個来ても解除せず、
    operator reset + 再 ARM を必須とする(docs/08 §4.1)。
  - 同一 priority への再 latch は重大側へのみ昇格し、降格・破棄しない
    (fault 情報の無言喪失は Gate 0 No-Go「例外を黙って捨てない」に反する)。

設計:
  - 純ロジック・単一スレッド前提・時刻は now_ns 引数で注入(test 可能性)。
  - I/O、DDS、network、thread を一切持たない。実 process への配線は
    Exclusive Actuation Gateway 実装(後続 task)で行う。gateway の受入条件:
    (a) release_latch は認証済み operator lease 経由でのみ呼ぶこと、
    (b) Directive.stop_state は latch 記録の停止種別であり stop FSM の
    target 状態ではない — 物理進行は safety.stop_transitions の表に従うこと。
  - 本 arbiter は「何を実行してよいか」を決めるだけで、actuator command を
    送らない(invariant 5: 独立性を第二の publisher で実現しない)。
"""
from dataclasses import dataclass, replace
from enum import Enum, unique
from typing import Dict, Optional, Tuple

from contracts import _validation as V
from contracts.command_envelope import (
    ArbiterPriority, CommandEnvelope, RequestedMode, ServerAttribution,
)
from contracts.errors import ContractViolation
from contracts.stop_states import StopState


@unique
class DirectiveKind(Enum):
    """arbiter が下流(gateway)へ示す現在の指示種別。"""
    IDLE_HOLD = "IDLE_HOLD"                # 有効 command なし → 能動保持(ゼロ推測ではない)
    EXECUTE = "EXECUTE"                    # 受理済み command を実行
    CONTROLLED_STOP = "CONTROLLED_STOP"    # expiry 等 → 明示の Controlled Stop(08 §4.2)
    LATCHED_STOP = "LATCHED_STOP"          # latch 中の安全停止(解除は operator reset)


@dataclass(frozen=True)
class Directive:
    kind: DirectiveKind
    # 停止種別(latch 記録の種類)。stop FSM の target 状態ではない —
    # 物理進行は gateway/supervisor が safety.stop_transitions に従い実施する。
    stop_state: Optional[StopState]
    envelope: Optional[CommandEnvelope]          # EXECUTE のときのみ
    attribution: Optional[ServerAttribution]     # EXECUTE のときのみ
    reason: str


@dataclass(frozen=True)
class Decision:
    accepted: bool
    reason: str


@dataclass(frozen=True)
class LatchRecord:
    priority: ArbiterPriority
    stop_state: StopState
    reason: str
    latched_at_ns: int
    suppressed_count: int = 0   # 同 priority で記録されなかった後続 fault 報告数


@dataclass(frozen=True)
class OperatorReset:
    """latch 解除に必須の操作者手続き(08 §4.1: 自動復帰禁止)。"""
    operator_lease_id: str
    reset_confirmed: bool     # 明示の reset 操作
    rearm_confirmed: bool     # 別操作の再 ARM(08 §2.5)
    health_confirmed: bool    # 一定時間の健全性確認(supervisor fault の解除に必須)

    def __post_init__(self) -> None:
        V.req_token(self.operator_lease_id, "reset.operator_lease_id")
        V.req_bool(self.reset_confirmed, "reset.reset_confirmed")
        V.req_bool(self.rearm_confirmed, "reset.rearm_confirmed")
        V.req_bool(self.health_confirmed, "reset.health_confirmed")


# latch を作れる stop_state(通常の EXECUTE 命令は latch にならない)
_LATCHABLE_STATES = (
    StopState.STOP_NOW,
    StopState.CONTROLLED_STOP,
    StopState.DAMP_CRITICAL_STOP,
    StopState.PHYSICAL_ESTOP_FUNCTION,
)
# 同一 priority 再 latch の昇格順(大きいほど重大)。STOP_NOW(要求)と
# CONTROLLED_STOP(その実行実体)は同格 — stop_transitions の表に
# CONTROLLED_STOP→STOP_NOW の edge がなく、昇格すると未定義遷移の指示になる。
_STOP_SEVERITY = {
    StopState.STOP_NOW: 0,
    StopState.CONTROLLED_STOP: 0,
    StopState.DAMP_CRITICAL_STOP: 1,
    StopState.PHYSICAL_ESTOP_FUNCTION: 2,
}
# supervisor 系 latch(解除に健全性確認も要求)
_SUPERVISOR_PRIORITIES = (
    ArbiterPriority.PHYSICAL_ESTOP_OR_HARD_FAULT,
    ArbiterPriority.SUPERVISOR_SOFT_FAULT_OR_SENSOR_STALE,
)


class CommandArbiter:
    """単一 owner の command 調停器(純ロジック)。

    使い方(gateway 側):
        arb = CommandArbiter()
        arb.submit(envelope, attribution, now_ns)   # 受理判定
        d = arb.current(now_ns)                     # 現在の指示(毎 tick)
    """

    def __init__(self) -> None:
        self._active: Optional[Tuple[CommandEnvelope, ServerAttribution, int]] = None
        self._latches: Dict[ArbiterPriority, LatchRecord] = {}
        self._last_seq: Dict[str, int] = {}
        self._last_now_ns: int = 0
        self._expired_reason: Optional[str] = None

    # ---------- 内部ヘルパ ----------

    def _check_clock(self, now_ns: int) -> None:
        if not isinstance(now_ns, int) or isinstance(now_ns, bool) or now_ns <= 0:
            raise ContractViolation("now_ns", "正の monotonic ns が必要: %r" % (now_ns,))
        if now_ns < self._last_now_ns:
            # 時計の逆行 = 環境の重大異常。fail-closed で supervisor latch。
            # _last_now_ns は更新しない(逆行が続く限り毎 tick 再 latch =
            # suppressed_count が増える。時刻が回復するまで解除しても再発する)。
            self._latch(LatchRecord(
                priority=ArbiterPriority.SUPERVISOR_SOFT_FAULT_OR_SENSOR_STALE,
                stop_state=StopState.CONTROLLED_STOP,
                reason="monotonic clock jump backwards(%d -> %d)"
                       % (self._last_now_ns, now_ns),
                latched_at_ns=self._last_now_ns))
        else:
            self._last_now_ns = now_ns

    def _latch(self, rec: LatchRecord) -> None:
        cur = self._latches.get(rec.priority)
        if cur is None:
            self._latches[rec.priority] = rec
        elif _STOP_SEVERITY[rec.stop_state] > _STOP_SEVERITY[cur.stop_state]:
            # 同一 priority の再 latch は重大側へのみ昇格。初回時刻を保持し、
            # 両 reason を残す(昇格は最大2回なので有界)。
            self._latches[rec.priority] = replace(
                cur, stop_state=rec.stop_state,
                reason="%s; escalated: %s" % (cur.reason, rec.reason))
        else:
            # 降格・同格は record を変えず件数だけ記録(監査可能・有界)
            self._latches[rec.priority] = replace(
                cur, suppressed_count=cur.suppressed_count + 1)
        self._active = None  # latch 発生時、実行中 command の authority を失効

    def _highest_latch(self) -> Optional[LatchRecord]:
        if not self._latches:
            return None
        pr = min(self._latches.keys())  # 数値が小さいほど高優先
        return self._latches[pr]

    def _expiry_ns(self, env: CommandEnvelope, attr: ServerAttribution) -> int:
        # expiry は server が記録した受理時刻から数える(02 §4.1 / 08 §4.2)。
        # 送信側 timestamp は監査用で、期限計算に使わない。
        return attr.accepted_monotonic_timestamp_ns + env.expires_after_ms * 1_000_000

    def _is_expired(self, env: CommandEnvelope, attr: ServerAttribution,
                    now_ns: int) -> bool:
        return now_ns > self._expiry_ns(env, attr)

    # ---------- 公開 API ----------

    def submit(self, env: CommandEnvelope, attr: ServerAttribution,
               now_ns: int) -> Decision:
        """command request の受理判定。受理しても実行指示は current() が返す。"""
        if not isinstance(env, CommandEnvelope):
            raise ContractViolation("envelope", "CommandEnvelope が必要: %r" % (env,))
        if not isinstance(attr, ServerAttribution):
            raise ContractViolation("attribution",
                                    "ServerAttribution が必要(server-side 付与): %r" % (attr,))
        self._check_clock(now_ns)

        # STOP_NOW はすべての検査より先に受理・latch する(停止要求の黙殺こそ
        # fail-open)。sequence/timestamp 異常は reason に記録するが拒否しない。
        if env.requested_mode is RequestedMode.STOP_NOW:
            last = self._last_seq.get(attr.trusted_source_id)
            reason = "STOP_NOW from %s" % attr.trusted_source_id
            if last is not None and env.sequence <= last:
                reason += "(sequence anomaly: %d <= %d — latched anyway)" % (
                    env.sequence, last)
            # watermark は巻き戻さない(古い運動 command の replay 窓を開けない)
            self._last_seq[attr.trusted_source_id] = (
                env.sequence if last is None else max(last, env.sequence))
            self._latch(LatchRecord(
                priority=attr.priority,
                stop_state=StopState.STOP_NOW,
                reason=reason,
                latched_at_ns=now_ns))
            return Decision(True, "STOP_NOW latched")

        # 時刻整合: server-side 付与の受理時刻が now_ns より未来は契約違反。
        # 放置すると expiry を MAX_ENVELOPE_EXPIRES_MS の意図を超えて延長できる
        # (wall clock ns の混入等)。clock 逆行 latch と対称の防御。
        if attr.accepted_monotonic_timestamp_ns > now_ns:
            raise ContractViolation(
                "attribution.accepted_monotonic_timestamp_ns",
                "now_ns より未来(%d > %d)— 時刻整合違反"
                % (attr.accepted_monotonic_timestamp_ns, now_ns))

        # sequence: source ごとに単調増加を要求(重複/stale の再生を拒否)
        last = self._last_seq.get(attr.trusted_source_id)
        if last is not None and env.sequence <= last:
            return Decision(False, "stale/duplicate sequence(%d <= %d)"
                            % (env.sequence, last))

        # latch 中はいかなる通常 command も受理しない(解除は operator reset のみ)
        latch = self._highest_latch()
        if latch is not None:
            if attr.priority >= latch.priority:
                return Decision(False, "latched by %s(%s)— 低優先は解除も上書きも不可"
                                % (latch.priority.name, latch.reason))
            return Decision(False, "latch 中は通常 command を受理しない"
                                   "(解除は operator reset のみ)")

        # DOA(dead on arrival): 到着時点で既に期限切れの command は受理しない。
        # 受理すると健全な active を置換して spurious CONTROLLED_STOP を作り、
        # 実行不能な command に accepted ACK を返してしまう。
        if self._is_expired(env, attr, now_ns):
            return Decision(False, "expired on arrival(source=%s, seq=%d)"
                            % (attr.trusted_source_id, env.sequence))

        # 既存 active との優先度比較(数値が小さいほど高優先)
        if self._active is not None:
            a_env, a_attr, _ = self._active
            if not self._is_expired(a_env, a_attr, now_ns) \
                    and attr.priority > a_attr.priority:
                return Decision(False, "higher-priority command active(%s)"
                                % a_attr.priority.name)

        self._last_seq[attr.trusted_source_id] = env.sequence
        self._active = (env, attr, now_ns)
        self._expired_reason = None
        return Decision(True, "accepted")

    def current(self, now_ns: int) -> Directive:
        """現在の実行指示。毎 tick 呼ぶ。"""
        self._check_clock(now_ns)
        latch = self._highest_latch()
        if latch is not None:
            return Directive(DirectiveKind.LATCHED_STOP, latch.stop_state,
                             None, None, latch.reason)
        if self._active is None:
            if self._expired_reason is not None:
                # expiry 後はゼロ推測せず Controlled Stop(08 §4.2)。
                # 新しい有効 command が受理されるまで維持する。
                return Directive(DirectiveKind.CONTROLLED_STOP,
                                 StopState.CONTROLLED_STOP, None, None,
                                 self._expired_reason)
            return Directive(DirectiveKind.IDLE_HOLD, StopState.ACTIVE_HOLD,
                             None, None, "no active command")
        env, attr, _ = self._active
        if self._is_expired(env, attr, now_ns):
            self._active = None
            self._expired_reason = ("command expired(source=%s, seq=%d)"
                                    % (attr.trusted_source_id, env.sequence))
            return Directive(DirectiveKind.CONTROLLED_STOP,
                             StopState.CONTROLLED_STOP, None, None,
                             self._expired_reason)
        return Directive(DirectiveKind.EXECUTE, None, env, attr, "executing")

    def latch_fault(self, priority: ArbiterPriority, stop_state: StopState,
                    reason: str, now_ns: int) -> None:
        """supervisor / E-stop 経路からの latched safe-state request。"""
        self._check_clock(now_ns)
        if not isinstance(priority, ArbiterPriority):
            raise ContractViolation("priority", "ArbiterPriority が必要")
        if stop_state not in _LATCHABLE_STATES:
            raise ContractViolation(
                "stop_state", "latch 可能な停止状態のみ: %r" % (stop_state,))
        V.req_str(reason, "reason", max_len=512)
        self._latch(LatchRecord(priority, stop_state, reason, now_ns))

    def release_latch(self, priority: ArbiterPriority, reset: OperatorReset,
                      now_ns: int) -> Decision:
        """latch の解除。operator reset + 再 ARM 必須。自動復帰は存在しない。

        - command 経路からは優先度によらず一切解除できない(submit 参照)。
          解除は操作者手続きであり、operator lease の認証・実在検証は
          gateway の責務(gateway 実装 task の受入条件に contract test を置く)。
        - supervisor 系 latch は健全性確認(health_confirmed)も必須(08 §4.1)。
        - 解除は priority 単位。suppressed_count>0 の record は複数 fault 報告を
          含むため、operator は latches() で確認してから解除すること。
        """
        self._check_clock(now_ns)
        if not isinstance(priority, ArbiterPriority):
            raise ContractViolation("priority", "ArbiterPriority が必要")
        if not isinstance(reset, OperatorReset):
            raise ContractViolation("reset", "OperatorReset が必要")
        rec = self._latches.get(priority)
        if rec is None:
            return Decision(False, "対象 latch なし: %s" % priority.name)
        if not (reset.reset_confirmed and reset.rearm_confirmed):
            return Decision(False, "operator reset + 再 ARM の両方が必要(自動復帰禁止)")
        if priority in _SUPERVISOR_PRIORITIES and not reset.health_confirmed:
            return Decision(False, "supervisor latch は健全性確認後のみ解除可")
        del self._latches[priority]
        return Decision(True, "latch released: %s" % priority.name)

    def latches(self) -> tuple:
        """観測用(読み取りのみ)。"""
        return tuple(sorted(self._latches.values(), key=lambda r: r.priority))
