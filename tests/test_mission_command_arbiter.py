"""Command Arbiter の priority / expiry / latch 網羅テスト(Gate 0:
docs/08 §9「command arbiter priority/expiry/latch の網羅テスト」)。

すべて robot 非接続・時刻注入の純ロジック test。
priority は 8x8 全組合せを機械的に検証する。
"""
import itertools
import unittest
import uuid

from contracts import ContractViolation
from contracts.command_envelope import (
    ArbiterPriority, CommandEnvelope, RequestedMode, ServerAttribution,
)
from contracts.stop_states import StopState
from mission.command_arbiter import (
    CommandArbiter, DirectiveKind, OperatorReset,
)

MS = 1_000_000  # ns


def _uuid():
    return str(uuid.uuid4())


def make_env(seq=0, mode="COMMON_NAV", expires_ms=200, vx=0.2):
    if mode in ("STOP_NOW", "HOLD"):
        vx = 0.0
    return CommandEnvelope(
        schema_version="1.0",
        source_id="test_source",
        goal_id=_uuid(),
        actuation_request_id=_uuid(),
        sender_timestamp=1,
        sequence=seq,
        expires_after_ms=expires_ms,
        requested_mode=RequestedMode[mode],
        vx=vx, vy=0.0, wz=0.0,
        phase="NAV_FLAT",
        policy_hash="not_applicable",
    )


def make_attr(source="nav_planner", priority=ArbiterPriority.NAV_LOCAL_PLANNER,
              accepted_ns=1_000):
    return ServerAttribution(trusted_source_id=source, priority=priority,
                             accepted_monotonic_timestamp_ns=accepted_ns)


FULL_RESET = OperatorReset(operator_lease_id="lease", reset_confirmed=True,
                           rearm_confirmed=True, health_confirmed=True)


class TestBasics(unittest.TestCase):
    def test_idle_hold_default(self):
        # 有効 command なし → 能動保持(ゼロ速度の推測ではなく明示の HOLD)
        arb = CommandArbiter()
        d = arb.current(1_000)
        self.assertIs(d.kind, DirectiveKind.IDLE_HOLD)
        self.assertIs(d.stop_state, StopState.ACTIVE_HOLD)

    def test_accept_and_execute(self):
        arb = CommandArbiter()
        env, attr = make_env(seq=1), make_attr(accepted_ns=1_000)
        self.assertTrue(arb.submit(env, attr, 1_000).accepted)
        d = arb.current(2_000)
        self.assertIs(d.kind, DirectiveKind.EXECUTE)
        self.assertIs(d.envelope, env)

    def test_type_checks(self):
        arb = CommandArbiter()
        with self.assertRaises(ContractViolation):
            arb.submit({"vx": 1.0}, make_attr(), 1_000)
        with self.assertRaises(ContractViolation):
            arb.submit(make_env(), {"priority": 1}, 1_000)
        with self.assertRaises(ContractViolation):
            arb.current(0)
        with self.assertRaises(ContractViolation):
            arb.current(-5)
        with self.assertRaises(ContractViolation):
            arb.latch_fault(2, StopState.CONTROLLED_STOP, "int priority", 1_000)
        with self.assertRaises(ContractViolation):
            arb.release_latch(ArbiterPriority.OPERATOR_STOP_OR_DISARM,
                              {"reset": True}, 1_000)

    def test_operator_reset_is_validated(self):
        # 空 lease・非 bool flag は構築時点で拒否(truthy すり抜けの防止)
        with self.assertRaises(ContractViolation):
            OperatorReset("", True, True, True)
        with self.assertRaises(ContractViolation):
            OperatorReset("lease", "maybe", True, True)
        with self.assertRaises(ContractViolation):
            OperatorReset("lease", True, 1, True)


class TestExpiry(unittest.TestCase):
    def test_expiry_to_controlled_stop_no_zero_guess(self):
        """expiry 後はゼロを推測せず明示の Controlled Stop(docs/08 §4.2)。"""
        arb = CommandArbiter()
        t0 = 1_000
        env = make_env(seq=1, expires_ms=200)
        self.assertTrue(arb.submit(env, make_attr(accepted_ns=t0), t0).accepted)
        self.assertIs(arb.current(t0 + 100 * MS).kind, DirectiveKind.EXECUTE)
        d = arb.current(t0 + 201 * MS)
        self.assertIs(d.kind, DirectiveKind.CONTROLLED_STOP)
        self.assertIs(d.stop_state, StopState.CONTROLLED_STOP)
        # その後も新 command が来るまで CONTROLLED_STOP を維持
        self.assertIs(arb.current(t0 + 500 * MS).kind, DirectiveKind.CONTROLLED_STOP)

    def test_expiry_boundary_is_exclusive(self):
        # 境界ちょうど(now == accepted + expires)は EXECUTE、+1ns で失効。
        # _is_expired の `>` 包含性を固定する(`>=` への変異を検出)。
        arb = CommandArbiter()
        t0 = 1_000
        arb.submit(make_env(seq=1, expires_ms=200), make_attr(accepted_ns=t0), t0)
        expiry_ns = t0 + 200 * MS
        self.assertIs(arb.current(expiry_ns).kind, DirectiveKind.EXECUTE)
        self.assertIs(arb.current(expiry_ns + 1).kind, DirectiveKind.CONTROLLED_STOP)

    def test_fresh_command_resumes_after_expiry(self):
        arb = CommandArbiter()
        t0 = 1_000
        arb.submit(make_env(seq=1), make_attr(accepted_ns=t0), t0)
        arb.current(t0 + 300 * MS)  # expire
        t1 = t0 + 400 * MS
        self.assertTrue(
            arb.submit(make_env(seq=2), make_attr(accepted_ns=t1), t1).accepted)
        self.assertIs(arb.current(t1 + 1).kind, DirectiveKind.EXECUTE)

    def test_dead_on_arrival_rejected_keeps_healthy_active(self):
        # 到着時点で期限切れの command は受理せず、健全な active を守る
        arb = CommandArbiter()
        t0 = 1_000
        env = make_env(seq=1, expires_ms=10000)
        arb.submit(env, make_attr(source="a", accepted_ns=t0), t0)
        self.assertIs(arb.current(t0 + 1).kind, DirectiveKind.EXECUTE)
        doa = make_env(seq=1, expires_ms=1)
        d = arb.submit(doa, make_attr(source="b", accepted_ns=t0), t0 + 5 * MS)
        self.assertFalse(d.accepted)
        self.assertIn("expired on arrival", d.reason)
        cur = arb.current(t0 + 6 * MS)
        self.assertIs(cur.kind, DirectiveKind.EXECUTE)
        self.assertIs(cur.envelope, env)

    def test_future_accepted_ns_rejected(self):
        """accepted_ns > now_ns は expiry 無期限化経路 → ContractViolation。"""
        arb = CommandArbiter()
        t0 = 1_000
        with self.assertRaises(ContractViolation):
            arb.submit(make_env(seq=1),
                       make_attr(accepted_ns=t0 + 3_600_000 * MS), t0)
        # 等号は正常(同一 tick 受理)
        self.assertTrue(
            arb.submit(make_env(seq=1), make_attr(accepted_ns=t0), t0).accepted)

    def test_expired_active_does_not_block_lower_priority(self):
        # current() を経由せずとも、失効済み active は低優先 submit を妨げない
        arb = CommandArbiter()
        t0 = 1_000
        stair = make_attr(source="stair", priority=ArbiterPriority.STAIR_STATE_MACHINE,
                          accepted_ns=t0)
        arb.submit(make_env(seq=1, expires_ms=200), stair, t0)
        t1 = t0 + 300 * MS  # stair は失効済み
        nav = make_attr(source="nav", priority=ArbiterPriority.NAV_LOCAL_PLANNER,
                        accepted_ns=t1)
        self.assertTrue(arb.submit(make_env(seq=1), nav, t1).accepted)
        self.assertIs(arb.current(t1 + 1).kind, DirectiveKind.EXECUTE)


class TestSequence(unittest.TestCase):
    def test_stale_and_duplicate_rejected(self):
        arb = CommandArbiter()
        t = 1_000
        self.assertTrue(arb.submit(make_env(seq=5), make_attr(accepted_ns=t), t).accepted)
        self.assertFalse(arb.submit(make_env(seq=5), make_attr(accepted_ns=t), t + 1).accepted)
        self.assertFalse(arb.submit(make_env(seq=4), make_attr(accepted_ns=t), t + 2).accepted)
        self.assertTrue(arb.submit(make_env(seq=6), make_attr(accepted_ns=t), t + 3).accepted)

    def test_sequence_tracked_per_source(self):
        arb = CommandArbiter()
        t = 1_000
        a1 = make_attr(source="src_a")
        a2 = make_attr(source="src_b", priority=ArbiterPriority.STAIR_STATE_MACHINE)
        self.assertTrue(arb.submit(make_env(seq=5), a1, t).accepted)
        # 別 source は独立の sequence 空間
        self.assertTrue(arb.submit(make_env(seq=1), a2, t + 1).accepted)


class TestPriorityMatrix(unittest.TestCase):
    def test_active_vs_new_all_64_combinations(self):
        """8x8 全組合せ: 新規 command は active と同等以上の優先度
        (数値 <=)のときだけ受理される。"""
        t = 1_000
        for pa, pn in itertools.product(ArbiterPriority, repeat=2):
            arb = CommandArbiter()
            self.assertTrue(arb.submit(
                make_env(seq=1),
                make_attr(source="a", priority=pa, accepted_ns=t), t).accepted)
            got = arb.submit(
                make_env(seq=1),
                make_attr(source="b", priority=pn, accepted_ns=t), t + 1).accepted
            self.assertEqual(got, pn <= pa, "active=%s new=%s" % (pa.name, pn.name))

    def test_latch_blocks_all_64_combinations(self):
        """latch 中は latch/submit の priority 組合せによらず通常 command を
        受理しない(解除は operator reset のみ)。STOP_NOW だけは常時受理。"""
        t = 1_000
        for pl, pn in itertools.product(ArbiterPriority, repeat=2):
            arb = CommandArbiter()
            arb.latch_fault(pl, StopState.CONTROLLED_STOP, "fault", t)
            self.assertFalse(arb.submit(
                make_env(seq=1),
                make_attr(source="s", priority=pn, accepted_ns=t), t + 1).accepted,
                "latch=%s new=%s" % (pl.name, pn.name))
            self.assertTrue(arb.submit(
                make_env(seq=2, mode="STOP_NOW"),
                make_attr(source="s", priority=pn, accepted_ns=t), t + 2).accepted,
                "STOP_NOW latch=%s new=%s" % (pl.name, pn.name))

    def test_same_priority_stream_replaces(self):
        # 同一 source の連続 stream(20-50Hz nav)は逐次置換
        arb = CommandArbiter()
        t = 1_000
        for seq in (1, 2, 3):
            self.assertTrue(
                arb.submit(make_env(seq=seq),
                           make_attr(accepted_ns=t + seq), t + seq).accepted)

    def test_priority_is_server_side_only(self):
        # priority は ServerAttribution のみが運ぶ。envelope の自称 priority は
        # contracts 層の未知キー拒否で closed(tests/test_contracts_command_envelope.py)
        self.assertFalse(hasattr(CommandEnvelope, "priority"))


class TestStopNow(unittest.TestCase):
    def test_stop_now_latches_and_blocks(self):
        arb = CommandArbiter()
        t = 1_000
        op = make_attr(source="operator", priority=ArbiterPriority.OPERATOR_STOP_OR_DISARM,
                       accepted_ns=t)
        self.assertTrue(arb.submit(make_env(seq=1, mode="STOP_NOW"), op, t).accepted)
        d = arb.current(t + 1)
        self.assertIs(d.kind, DirectiveKind.LATCHED_STOP)
        self.assertIs(d.stop_state, StopState.STOP_NOW)
        # 健全な通常 command が何個来ても latch は解除されない(自動復帰禁止)
        for seq in (2, 3, 4):
            self.assertFalse(
                arb.submit(make_env(seq=seq), make_attr(accepted_ns=t), t + seq).accepted)
        self.assertIs(arb.current(t + 10).kind, DirectiveKind.LATCHED_STOP)

    def test_stop_now_latches_even_with_stale_sequence(self):
        # 操作者 console 再起動等で sequence が巻き戻っても STOP_NOW は latch する
        # (停止経路の fail-open 禁止 — docs/CLAUDE.md Phase 3)
        arb = CommandArbiter()
        t = 1_000
        op = make_attr(source="operator",
                       priority=ArbiterPriority.OPERATOR_STOP_OR_DISARM,
                       accepted_ns=t)
        self.assertTrue(arb.submit(make_env(seq=100, mode="HOLD"), op, t).accepted)
        d = arb.submit(make_env(seq=0, mode="STOP_NOW"), op, t + 1)
        self.assertTrue(d.accepted)
        cur = arb.current(t + 2)
        self.assertIs(cur.kind, DirectiveKind.LATCHED_STOP)
        self.assertIs(cur.stop_state, StopState.STOP_NOW)

    def test_stale_stop_now_does_not_regress_watermark(self):
        # stale STOP_NOW 受理後も _last_seq は巻き戻らない(運動 command の
        # anti-replay は維持)
        arb = CommandArbiter()
        t = 1_000
        op = make_attr(source="operator",
                       priority=ArbiterPriority.OPERATOR_STOP_OR_DISARM,
                       accepted_ns=t)
        self.assertTrue(arb.submit(make_env(seq=100, mode="HOLD"), op, t).accepted)
        self.assertTrue(arb.submit(make_env(seq=0, mode="STOP_NOW"), op, t + 1).accepted)
        arb.release_latch(ArbiterPriority.OPERATOR_STOP_OR_DISARM, FULL_RESET, t + 2)
        # seq=50 (<100) の通常 command は依然拒否される
        self.assertFalse(arb.submit(make_env(seq=50, mode="HOLD"), op, t + 3).accepted)

    def test_stop_now_never_rejected_by_timestamp(self):
        # 未来 accepted_ns の STOP_NOW でも受理・latch(fail toward stop)
        arb = CommandArbiter()
        t = 1_000
        op = make_attr(source="operator",
                       priority=ArbiterPriority.OPERATOR_STOP_OR_DISARM,
                       accepted_ns=t + 999 * MS)
        self.assertTrue(arb.submit(make_env(seq=1, mode="STOP_NOW"), op, t).accepted)
        self.assertIs(arb.current(t + 1).kind, DirectiveKind.LATCHED_STOP)


class TestLatch(unittest.TestCase):
    def test_release_requires_reset_and_rearm(self):
        arb = CommandArbiter()
        t = 1_000
        op = make_attr(source="operator", priority=ArbiterPriority.OPERATOR_STOP_OR_DISARM,
                       accepted_ns=t)
        arb.submit(make_env(seq=1, mode="STOP_NOW"), op, t)
        p = ArbiterPriority.OPERATOR_STOP_OR_DISARM
        self.assertFalse(arb.release_latch(
            p, OperatorReset("l", True, False, True), t + 1).accepted)
        self.assertFalse(arb.release_latch(
            p, OperatorReset("l", False, True, True), t + 2).accepted)
        self.assertTrue(arb.release_latch(p, FULL_RESET, t + 3).accepted)
        self.assertIs(arb.current(t + 4).kind, DirectiveKind.IDLE_HOLD)

    def test_release_nonexistent_latch_fails(self):
        arb = CommandArbiter()
        d = arb.release_latch(ArbiterPriority.WIRED_MANUAL, FULL_RESET, 1_000)
        self.assertFalse(d.accepted)

    def test_supervisor_latch_needs_health_confirmation(self):
        arb = CommandArbiter()
        t = 1_000
        arb.latch_fault(ArbiterPriority.SUPERVISOR_SOFT_FAULT_OR_SENSOR_STALE,
                        StopState.CONTROLLED_STOP, "sensor stale", t)
        p = ArbiterPriority.SUPERVISOR_SOFT_FAULT_OR_SENSOR_STALE
        self.assertFalse(arb.release_latch(
            p, OperatorReset("l", True, True, False), t + 1).accepted)
        self.assertTrue(arb.release_latch(p, FULL_RESET, t + 2).accepted)

    def test_highest_latch_wins(self):
        arb = CommandArbiter()
        t = 1_000
        arb.latch_fault(ArbiterPriority.SUPERVISOR_SOFT_FAULT_OR_SENSOR_STALE,
                        StopState.CONTROLLED_STOP, "stale", t)
        arb.latch_fault(ArbiterPriority.PHYSICAL_ESTOP_OR_HARD_FAULT,
                        StopState.DAMP_CRITICAL_STOP, "hard fault", t + 1)
        d = arb.current(t + 2)
        self.assertIs(d.stop_state, StopState.DAMP_CRITICAL_STOP)
        arb.release_latch(ArbiterPriority.SUPERVISOR_SOFT_FAULT_OR_SENSOR_STALE,
                          FULL_RESET, t + 3)
        self.assertIs(arb.current(t + 4).stop_state, StopState.DAMP_CRITICAL_STOP)

    def test_same_priority_relatch_escalates_never_downgrades(self):
        # 同一 priority の再 latch は重大側へのみ昇格し、両 reason を保持する
        arb = CommandArbiter()
        t = 1_000
        p = ArbiterPriority.SUPERVISOR_SOFT_FAULT_OR_SENSOR_STALE
        arb.latch_fault(p, StopState.CONTROLLED_STOP, "sensor stale", t)
        arb.latch_fault(p, StopState.DAMP_CRITICAL_STOP, "control divergence", t + 1)
        d = arb.current(t + 2)
        self.assertIs(d.stop_state, StopState.DAMP_CRITICAL_STOP)  # 昇格
        self.assertIn("sensor stale", d.reason)
        self.assertIn("control divergence", d.reason)  # 監査情報を失わない
        rec = arb.latches()[0]
        self.assertEqual(rec.latched_at_ns, t)  # 初回時刻を保持
        # 低重大の後続 fault で降格しない(件数のみ記録)
        arb.latch_fault(p, StopState.CONTROLLED_STOP, "later minor", t + 3)
        self.assertIs(arb.current(t + 4).stop_state, StopState.DAMP_CRITICAL_STOP)
        self.assertEqual(arb.latches()[0].suppressed_count, 1)

    def test_clock_jump_inside_latch_fault_does_not_drop_damp(self):
        # 時計逆行検出(CONTROLLED_STOP latch)と同時の DAMP 要求が失われない
        arb = CommandArbiter()
        arb.current(10_000)
        p = ArbiterPriority.SUPERVISOR_SOFT_FAULT_OR_SENSOR_STALE
        arb.latch_fault(p, StopState.DAMP_CRITICAL_STOP, "control divergence", 5_000)
        self.assertIs(arb.current(10_001).stop_state, StopState.DAMP_CRITICAL_STOP)

    def test_stop_now_accepted_even_during_latch(self):
        arb = CommandArbiter()
        t = 1_000
        arb.latch_fault(ArbiterPriority.SUPERVISOR_SOFT_FAULT_OR_SENSOR_STALE,
                        StopState.CONTROLLED_STOP, "stale", t)
        op = make_attr(source="operator", priority=ArbiterPriority.OPERATOR_STOP_OR_DISARM,
                       accepted_ns=t)
        self.assertTrue(arb.submit(make_env(seq=1, mode="STOP_NOW"), op, t + 1).accepted)
        self.assertIs(arb.current(t + 2).stop_state, StopState.STOP_NOW)

    def test_latch_kills_active_command(self):
        arb = CommandArbiter()
        t = 1_000
        arb.submit(make_env(seq=1), make_attr(accepted_ns=t), t)
        arb.latch_fault(ArbiterPriority.PHYSICAL_ESTOP_OR_HARD_FAULT,
                        StopState.DAMP_CRITICAL_STOP, "estop", t + 1)
        arb.release_latch(ArbiterPriority.PHYSICAL_ESTOP_OR_HARD_FAULT,
                          FULL_RESET, t + 2)
        self.assertIs(arb.current(t + 3).kind, DirectiveKind.IDLE_HOLD)

    def test_latch_rejects_non_latchable_state(self):
        arb = CommandArbiter()
        with self.assertRaises(ContractViolation):
            arb.latch_fault(ArbiterPriority.PHYSICAL_ESTOP_OR_HARD_FAULT,
                            StopState.CONTROLLED_EXIT, "bad", 1_000)


class TestClockJump(unittest.TestCase):
    def test_backwards_clock_latches_soft_fault(self):
        """monotonic 逆行 = 環境異常 → fail-closed で supervisor latch。"""
        arb = CommandArbiter()
        arb.current(10_000)
        d = arb.current(5_000)  # 逆行
        self.assertIs(d.kind, DirectiveKind.LATCHED_STOP)
        self.assertIs(d.stop_state, StopState.CONTROLLED_STOP)
        self.assertIn("clock jump", d.reason)

    def test_relatch_until_clock_recovers(self):
        # 逆行が続く限り解除しても再 latch され(fail-closed)、時刻回復後に
        # 解除すれば IDLE_HOLD へ戻れる。record は有界(reason 非成長)。
        arb = CommandArbiter()
        arb.current(10_000)
        arb.current(5_000)  # latch
        p = ArbiterPriority.SUPERVISOR_SOFT_FAULT_OR_SENSOR_STALE
        # 逆行時刻での解除は成功するが、次の逆行 tick で再 latch される
        self.assertTrue(arb.release_latch(p, FULL_RESET, 5_001).accepted)
        self.assertIs(arb.current(5_002).kind, DirectiveKind.LATCHED_STOP)
        # 再 latch が繰り返されても reason は成長しない(suppressed_count のみ)
        arb.current(5_003)
        arb.current(5_004)
        rec = arb.latches()[0]
        self.assertLess(len(rec.reason), 200)
        self.assertGreaterEqual(rec.suppressed_count, 1)
        # 時刻が回復してから解除すれば復帰できる
        self.assertIs(arb.current(10_001).kind, DirectiveKind.LATCHED_STOP)
        self.assertTrue(arb.release_latch(p, FULL_RESET, 10_002).accepted)
        self.assertIs(arb.current(10_003).kind, DirectiveKind.IDLE_HOLD)


if __name__ == "__main__":
    unittest.main()
