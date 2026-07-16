"""safety.stop_transitions — 停止系状態の遷移表(Gate 0: 全 edge テスト対象)。

正本: docs/08 §2(用語分離と各状態の意味)、§2.2(通常 STOP の順序: 手順3=速度
      整定・手順4=接地確認の両方を経て HOLD)、§2.3(StopMove は Sport 有効時
      のみ)、§2.4(Damp を選ぶ条件)、§2.5(E-stop は手動 reset 必須)、
      docs/02 §4.3(CONTROLLED_EXIT の前提)。

原則:
  - 表にない遷移は一切許可しない(未定義 transition は Gate 0 の No-Go)。
  - DAMP_CRITICAL_STOP への遷移は critical fault の明示 context がある場合のみ
    (通常停止・通信断を Damp に変換しない — invariant 8)。
  - PHYSICAL_ESTOP_FUNCTION は独立経路であり、どの状態からでも「入る」ことは
    拒否できないが、出る遷移は operator reset + 再 ARM に加えて再起立後の
    接地安定確認を必須とする(08 §2 表: Damp は支持を失い「要再起立」、
    §2.1: HOLD は 4 脚接地が定義要件)。
  - STOP_NOW は「操作者要求イベント」の擬似状態であり、本表への入り edge を
    持たない(entry-only)。停止列への入口は mission FSM の運動状態
    (NAVIGATING/ASCENDING 等 — StopState 非メンバー)からであり、gateway が
    停止要求受理時に STOP_NOW を初期状態として本 FSM を開始する。既に
    ACTIVE_HOLD にいる場合は自己遷移(滞在)で充足する。
  - CONTROLLED_EXIT の正常完了は本 FSM を離れて mission FSM の IDLE へ抜ける
    (docs/02 §5 BOTTOM_HOLD→IDLE、docs/05 §9)。exit 中の通常停止要求は
    CONTROLLED_STOP へ落とす(Damp への変換禁止 — invariant 8)。
  - Damp/E-stop からの復帰も実際は DISARMED 再 ARM 経由(docs/02 §5)であり、
    本表では最も保守的な ACTIVE_HOLD 復帰としてのみ表現する。
  - supervisor latch の解除に必要な健全性確認(08 §4.1)は
    mission.command_arbiter.release_latch が担保する(責務分担)。
  - 本 module は純関数のみ。時刻・I/O・latch 管理は持たない。
"""
from dataclasses import dataclass

from contracts import _validation as V
from contracts.errors import ContractViolation
from contracts.stop_states import StopState

_S = StopState


@dataclass(frozen=True)
class TransitionContext:
    """遷移の guard 条件。呼び出し側が検証済み事実だけを True にする。
    未検証は False のまま(fail-closed)。"""
    critical_fault: bool = False              # 制御発散/NaN/転倒等(08 §2.4)
    operator_reset_and_rearm: bool = False    # 手動 reset + 再 ARM 完了(08 §2.5)
    flat_wide_landing_verified: bool = False  # 平坦・全脚 edge 離れ・静止(02 §4.3)
    settled_below_thresholds: bool = False    # body 速度・角速度が閾値以下(08 §2.2 手順3)
    stable_contact_verified: bool = False     # 4脚接地の安定確認(08 §2.2 手順4)
    sport_authority_active: bool = False      # Sport authority 有効(08 §2.3。Branch L 中は False)

    def __post_init__(self) -> None:
        for name in ("critical_fault", "operator_reset_and_rearm",
                     "flat_wide_landing_verified", "settled_below_thresholds",
                     "stable_contact_verified", "sport_authority_active"):
            V.req_bool(getattr(self, name), "ctx." + name)


_UNDEFINED = object()

# 遷移表: (from, to) → guard 名の tuple(() = 無条件で許可)。
# 表に存在しない (from, to) は未定義遷移として拒否する。
_ALLOWED = {
    # 通常 STOP の実体(08 §2.2): 要求 → CONTROLLED_STOP → (手順3+4) → ACTIVE_HOLD
    (_S.STOP_NOW, _S.CONTROLLED_STOP): (),
    (_S.CONTROLLED_STOP, _S.ACTIVE_HOLD):
        ("settled_below_thresholds", "stable_contact_verified"),
    # Sport 有効時の停止(08 §2.3)。Low-level 中の StopMove に安全効果を前提としない
    (_S.STOP_NOW, _S.STOP_MOVE): ("sport_authority_active",),
    (_S.STOP_MOVE, _S.ACTIVE_HOLD):
        ("settled_below_thresholds", "stable_contact_verified"),
    # HOLD からの安全復帰(02 §4.3: 十分広く平坦な landing のみ)
    (_S.ACTIVE_HOLD, _S.CONTROLLED_EXIT): ("flat_wide_landing_verified",),
    # exit 中の通常停止要求 → 減速整定へ(Damp に変換しない — invariant 8)
    (_S.CONTROLLED_EXIT, _S.CONTROLLED_STOP): (),
    # exit の完了/中断後の能動保持(接地確認必須 — 08 §2.2 手順4)
    (_S.CONTROLLED_EXIT, _S.ACTIVE_HOLD): ("stable_contact_verified",),
    # critical fault 時のみ Damp(どの停止状態からでも。通常経路では不可)
    (_S.ACTIVE_HOLD, _S.DAMP_CRITICAL_STOP): ("critical_fault",),
    (_S.STOP_NOW, _S.DAMP_CRITICAL_STOP): ("critical_fault",),
    (_S.CONTROLLED_STOP, _S.DAMP_CRITICAL_STOP): ("critical_fault",),
    (_S.STOP_MOVE, _S.DAMP_CRITICAL_STOP): ("critical_fault",),
    (_S.CONTROLLED_EXIT, _S.DAMP_CRITICAL_STOP): ("critical_fault",),
    # 物理 E-stop は独立経路: どの状態からも「入る」(拒否不能)
    (_S.ACTIVE_HOLD, _S.PHYSICAL_ESTOP_FUNCTION): (),
    (_S.STOP_NOW, _S.PHYSICAL_ESTOP_FUNCTION): (),
    (_S.CONTROLLED_STOP, _S.PHYSICAL_ESTOP_FUNCTION): (),
    (_S.STOP_MOVE, _S.PHYSICAL_ESTOP_FUNCTION): (),
    (_S.CONTROLLED_EXIT, _S.PHYSICAL_ESTOP_FUNCTION): (),
    (_S.DAMP_CRITICAL_STOP, _S.PHYSICAL_ESTOP_FUNCTION): (),
    # E-stop / Damp からの復帰: operator reset + 再 ARM(08 §2.5)に加え、
    # Damp は支持を失う「要再起立」状態(08 §2 表)なので、ACTIVE_HOLD
    # (= 4脚接地 + 能動保持, 08 §2.1)への入場に接地安定確認も必須。
    (_S.DAMP_CRITICAL_STOP, _S.ACTIVE_HOLD):
        ("operator_reset_and_rearm", "stable_contact_verified"),
    (_S.PHYSICAL_ESTOP_FUNCTION, _S.ACTIVE_HOLD):
        ("operator_reset_and_rearm", "stable_contact_verified"),
}


def check_transition(frm: StopState, to: StopState, ctx: TransitionContext) -> None:
    """許可されない遷移は ContractViolation(fail-closed)。

    - 表にない (frm, to) = 未定義遷移 → 拒否。
    - guard 付き遷移は ctx の該当 flag が**すべて** True の場合のみ許可。
    """
    if not isinstance(frm, StopState):
        raise ContractViolation("from", "StopState が必要: %r" % (frm,))
    if not isinstance(to, StopState):
        raise ContractViolation("to", "StopState が必要: %r" % (to,))
    if not isinstance(ctx, TransitionContext):
        raise ContractViolation("ctx", "TransitionContext が必要: %r" % (ctx,))
    if frm is to:
        return  # 自己遷移(状態維持)は常に許可
    guards = _ALLOWED.get((frm, to), _UNDEFINED)
    if guards is _UNDEFINED:
        raise ContractViolation(
            "%s->%s" % (frm.name, to.name), "未定義遷移(Gate 0 No-Go)")
    for g in guards:
        if not getattr(ctx, g):
            raise ContractViolation(
                "%s->%s" % (frm.name, to.name),
                "guard 未成立: %s=False(fail-closed)" % g)


def allowed_transitions() -> frozenset:
    """テスト用: 許可 edge の集合(guard 含意なし)。"""
    return frozenset(_ALLOWED.keys())


def allowed_transitions_with_guards() -> dict:
    """テスト用: {(from, to): guard 名 tuple} のコピー(guard 含意あり)。
    guard の無言弱体化(mutation)をテストで検出するための観測関数。"""
    return dict(_ALLOWED)
