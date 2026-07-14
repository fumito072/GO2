"""停止状態遷移表の全 edge テスト(Gate 0: docs/08 §9「state transition table の
全 edge をテスト」「未定義 transition が残れば No-Go」)。

guard の同一性(どの flag を見ているか)まで検証する — guard の無言弱体化
(None 化・名前すり替え)を mutation として検出できるようにする。
"""
import dataclasses
import unittest

from contracts import ContractViolation
from contracts.stop_states import StopState
from safety.stop_transitions import (
    TransitionContext, check_transition,
    allowed_transitions, allowed_transitions_with_guards,
)

_S = StopState
_GUARD_FIELDS = ("critical_fault", "operator_reset_and_rearm",
                 "flat_wide_landing_verified", "settled_below_thresholds",
                 "stable_contact_verified", "sport_authority_active")
_FULL = TransitionContext(**{f: True for f in _GUARD_FIELDS})
_NONE = TransitionContext()

# 期待する {edge: guard tuple}(実装の表と独立に手書き — tautology 防止。
# 出典: docs/08 §2.2 手順3/4、§2.3、§2.4、§2.5、docs/02 §4.3)
EXPECTED_ALLOWED = {
    (_S.STOP_NOW, _S.CONTROLLED_STOP): (),
    (_S.CONTROLLED_STOP, _S.ACTIVE_HOLD):
        ("settled_below_thresholds", "stable_contact_verified"),
    (_S.STOP_NOW, _S.STOP_MOVE): ("sport_authority_active",),
    (_S.STOP_MOVE, _S.ACTIVE_HOLD):
        ("settled_below_thresholds", "stable_contact_verified"),
    (_S.ACTIVE_HOLD, _S.CONTROLLED_EXIT): ("flat_wide_landing_verified",),
    (_S.CONTROLLED_EXIT, _S.CONTROLLED_STOP): (),
    (_S.CONTROLLED_EXIT, _S.ACTIVE_HOLD): ("stable_contact_verified",),
    (_S.ACTIVE_HOLD, _S.DAMP_CRITICAL_STOP): ("critical_fault",),
    (_S.STOP_NOW, _S.DAMP_CRITICAL_STOP): ("critical_fault",),
    (_S.CONTROLLED_STOP, _S.DAMP_CRITICAL_STOP): ("critical_fault",),
    (_S.STOP_MOVE, _S.DAMP_CRITICAL_STOP): ("critical_fault",),
    (_S.CONTROLLED_EXIT, _S.DAMP_CRITICAL_STOP): ("critical_fault",),
    (_S.ACTIVE_HOLD, _S.PHYSICAL_ESTOP_FUNCTION): (),
    (_S.STOP_NOW, _S.PHYSICAL_ESTOP_FUNCTION): (),
    (_S.CONTROLLED_STOP, _S.PHYSICAL_ESTOP_FUNCTION): (),
    (_S.STOP_MOVE, _S.PHYSICAL_ESTOP_FUNCTION): (),
    (_S.CONTROLLED_EXIT, _S.PHYSICAL_ESTOP_FUNCTION): (),
    (_S.DAMP_CRITICAL_STOP, _S.PHYSICAL_ESTOP_FUNCTION): (),
    (_S.DAMP_CRITICAL_STOP, _S.ACTIVE_HOLD):
        ("operator_reset_and_rearm", "stable_contact_verified"),
    (_S.PHYSICAL_ESTOP_FUNCTION, _S.ACTIVE_HOLD):
        ("operator_reset_and_rearm", "stable_contact_verified"),
}


class TestAllEdges(unittest.TestCase):
    def test_table_matches_expected_including_guards(self):
        # edge 集合だけでなく guard 名まで完全一致(guard の None 化を検出)
        self.assertEqual(allowed_transitions_with_guards(), EXPECTED_ALLOWED)
        self.assertEqual(allowed_transitions(), frozenset(EXPECTED_ALLOWED))

    def test_every_edge_exhaustive(self):
        """7x7 = 49 edge 全件: 許可 edge は full ctx で通り、
        それ以外は full ctx でも未定義遷移として拒否される。"""
        for frm in StopState:
            for to in StopState:
                if frm is to:
                    check_transition(frm, to, _NONE)  # 自己遷移は常に許可
                elif (frm, to) in EXPECTED_ALLOWED:
                    check_transition(frm, to, _FULL)  # guard 成立なら許可
                else:
                    with self.assertRaises(ContractViolation,
                                           msg="%s->%s" % (frm.name, to.name)):
                        check_transition(frm, to, _FULL)

    def test_guard_identity_positive_and_negative(self):
        """guard 同一性の固定: (a) 必要な flag だけ True で許可(名前すり替え
        検出)、(b) どれか 1 flag 欠けでも拒否(AND 条件・fail-closed)。"""
        for (frm, to), guards in EXPECTED_ALLOWED.items():
            if not guards:
                continue
            # (a) 該当 guard 群のみ True → 許可
            check_transition(frm, to, TransitionContext(**{g: True for g in guards}))
            # (b) 1 flag だけ False(他は全 True)→ 拒否
            for g in guards:
                with self.assertRaises(ContractViolation,
                                       msg="%s->%s !%s" % (frm.name, to.name, g)):
                    check_transition(frm, to, dataclasses.replace(_FULL, **{g: False}))
            # (c) 全 False → 拒否
            with self.assertRaises(ContractViolation,
                                   msg="%s->%s(_NONE)" % (frm.name, to.name)):
                check_transition(frm, to, _NONE)

    def test_unguarded_edges_pass_with_empty_context(self):
        # 無条件 edge は _NONE でも通る(意図しない guard 付与も検出)
        for (frm, to), guards in EXPECTED_ALLOWED.items():
            if not guards:
                check_transition(frm, to, _NONE)


class TestSemantics(unittest.TestCase):
    def test_normal_stop_sequence(self):
        # 通常 STOP(08 §2.2): 要求 → CONTROLLED_STOP → (手順3+4) → ACTIVE_HOLD
        check_transition(_S.STOP_NOW, _S.CONTROLLED_STOP, _NONE)
        check_transition(_S.CONTROLLED_STOP, _S.ACTIVE_HOLD,
                         TransitionContext(settled_below_thresholds=True,
                                           stable_contact_verified=True))

    def test_hold_entry_requires_both_settle_and_contact(self):
        # 手順3(速度整定)と手順4(接地確認)の片方だけでは HOLD に入れない
        for frm in (_S.CONTROLLED_STOP, _S.STOP_MOVE):
            for ctx in (TransitionContext(stable_contact_verified=True),
                        TransitionContext(settled_below_thresholds=True)):
                with self.assertRaises(ContractViolation, msg=frm.name):
                    check_transition(frm, _S.ACTIVE_HOLD, ctx)

    def test_stop_move_requires_sport_authority(self):
        # Low-level 中の StopMove に安全効果を前提としない(08 §2.3)
        with self.assertRaises(ContractViolation):
            check_transition(_S.STOP_NOW, _S.STOP_MOVE, _NONE)
        check_transition(_S.STOP_NOW, _S.STOP_MOVE,
                         TransitionContext(sport_authority_active=True))
        # Branch L でも通常停止経路(CONTROLLED_STOP)は常に使える
        check_transition(_S.STOP_NOW, _S.CONTROLLED_STOP, _NONE)

    def test_no_damp_on_normal_stop(self):
        # 通常停止・通信断を Damp に変換しない(invariant 8)
        with self.assertRaises(ContractViolation):
            check_transition(_S.STOP_NOW, _S.DAMP_CRITICAL_STOP, _NONE)
        with self.assertRaises(ContractViolation):
            check_transition(_S.ACTIVE_HOLD, _S.DAMP_CRITICAL_STOP,
                             TransitionContext(stable_contact_verified=True))

    def test_stop_during_controlled_exit_without_damp(self):
        # exit 中の通常停止は CONTROLLED_STOP へ(Damp 変換禁止 — invariant 8)
        check_transition(_S.CONTROLLED_EXIT, _S.CONTROLLED_STOP, _NONE)
        with self.assertRaises(ContractViolation):
            check_transition(_S.CONTROLLED_EXIT, _S.DAMP_CRITICAL_STOP, _NONE)

    def test_estop_entry_unconditional_exit_guarded(self):
        # E-stop へは無条件で入れる(独立経路)が、復帰は reset+再ARM+接地確認
        for frm in StopState:
            if frm is _S.PHYSICAL_ESTOP_FUNCTION:
                continue
            check_transition(frm, _S.PHYSICAL_ESTOP_FUNCTION, _NONE)
        with self.assertRaises(ContractViolation):
            check_transition(_S.PHYSICAL_ESTOP_FUNCTION, _S.ACTIVE_HOLD, _NONE)

    def test_damp_estop_recovery_requires_reset_and_contact(self):
        # Damp は支持を失う「要再起立」状態(08 §2 表)— reset+再ARM と
        # 接地安定確認の両方が揃うまで ACTIVE_HOLD に入れない
        only_reset = TransitionContext(operator_reset_and_rearm=True)
        only_contact = TransitionContext(stable_contact_verified=True)
        both = TransitionContext(operator_reset_and_rearm=True,
                                 stable_contact_verified=True)
        for frm in (_S.DAMP_CRITICAL_STOP, _S.PHYSICAL_ESTOP_FUNCTION):
            for ctx in (only_reset, only_contact, _NONE):
                with self.assertRaises(ContractViolation, msg=frm.name):
                    check_transition(frm, _S.ACTIVE_HOLD, ctx)
            check_transition(frm, _S.ACTIVE_HOLD, both)

    def test_no_auto_recovery_from_damp(self):
        # Damp から reset なしで他状態へ移れない(E-stop への遷移を除く)
        for to in (_S.CONTROLLED_STOP, _S.STOP_MOVE, _S.CONTROLLED_EXIT, _S.STOP_NOW):
            with self.assertRaises(ContractViolation, msg=to.name):
                check_transition(_S.DAMP_CRITICAL_STOP, to, _FULL)

    def test_type_checks(self):
        with self.assertRaises(ContractViolation):
            check_transition("ACTIVE_HOLD", _S.STOP_NOW, _NONE)
        with self.assertRaises(ContractViolation):
            check_transition(_S.ACTIVE_HOLD, _S.CONTROLLED_EXIT, {"flat": True})
        with self.assertRaises(ContractViolation):
            # 非 bool の truthy flag は構築時点で拒否(fail-open 防止)
            TransitionContext(critical_fault="yes")


if __name__ == "__main__":
    unittest.main()
