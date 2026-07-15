"""Exclusive Actuation Gateway のテスト(Gate 0 素材、robot 非接続)。

contracts/README.md の gateway 受入条件 (a)(b)(c) と、docs/08 §4.2 handshake、
§4.3 policy hash 突合、invariant 3(排他)/6(backend 固定)を固定する。
"""
import unittest
import uuid

from contracts import ContractViolation
from contracts.command_envelope import (
    ArbiterPriority, CommandEnvelope, LocomotionBackend, RequestedMode,
    ServerAttribution,
)
from contracts.stop_states import StopState
from mission.command_arbiter import CommandArbiter, DirectiveKind, OperatorReset
from realtime.exclusive_actuation_gateway import (
    ActionKind, Channel, ExclusiveActuationGateway, HandshakeState,
    RobotStatusFlags, RunManifestControl,
)

MS = 1_000_000
HASH = "sha256:" + "a" * 64


def _uuid():
    return str(uuid.uuid4())


LEASE = _uuid()


def make_manifest(backend=LocomotionBackend.LEARNED_LOWCMD, policy_hash=HASH):
    return RunManifestControl(run_id="run_0001", selected_backend=backend,
                              policy_hash=policy_hash, operator_lease_id=LEASE)


def make_env(seq=0, mode="COMMON_NAV", expires_ms=10_000, vx=0.2,
             policy_hash="not_applicable"):
    if mode in ("STOP_NOW", "HOLD"):
        vx = 0.0
    return CommandEnvelope(
        schema_version="1.0", source_id="src", goal_id=_uuid(),
        actuation_request_id=_uuid(), sender_timestamp=1, sequence=seq,
        expires_after_ms=expires_ms, requested_mode=RequestedMode[mode],
        vx=vx, vy=0.0, wz=0.0, phase="NAV_FLAT", policy_hash=policy_hash)


def make_attr(source="nav", priority=ArbiterPriority.NAV_LOCAL_PLANNER,
              accepted_ns=1_000):
    return ServerAttribution(trusted_source_id=source, priority=priority,
                             accepted_monotonic_timestamp_ns=accepted_ns)


FULL_RESET = OperatorReset("lease", True, True, True)
FLAGS = RobotStatusFlags()
SETTLED = RobotStatusFlags(settled_below_thresholds=True,
                           stable_contact_verified=True)


def make_gateway(backend=LocomotionBackend.LEARNED_LOWCMD):
    arb = CommandArbiter()
    gw = ExclusiveActuationGateway(make_manifest(backend), arb)
    return gw, arb


def enable(gw, ch, t0):
    gw.request_channel(ch, t0)
    gw.ack_inactive(ch, t0 + 1)
    gen = gw.assign_generation(ch, t0 + 2)
    gw.enable(ch, t0 + 3)
    return gen


class TestHandshake(unittest.TestCase):
    def test_full_handshake_order(self):
        gw, _ = make_gateway()
        gen = enable(gw, Channel.COMMON_NAV, 1_000)
        self.assertEqual(gen, 1)
        self.assertIs(gw.enabled_channel(), Channel.COMMON_NAV)

    def test_skipping_steps_rejected(self):
        gw, _ = make_gateway()
        gw.request_channel(Channel.COMMON_NAV, 1_000)
        with self.assertRaises(ContractViolation):
            gw.assign_generation(Channel.COMMON_NAV, 1_001)  # ack を飛ばす
        gw2, _ = make_gateway()
        with self.assertRaises(ContractViolation):
            gw2.enable(Channel.COMMON_NAV, 1_000)  # 全段飛ばす
        gw3, _ = make_gateway()
        with self.assertRaises(ContractViolation):
            gw3.ack_inactive(Channel.COMMON_NAV, 1_000)  # request なし

    def test_exclusivity(self):
        # invariant 3: 一方が DISABLED でない限り他方は進めない
        gw, _ = make_gateway()
        enable(gw, Channel.COMMON_NAV, 1_000)
        with self.assertRaises(ContractViolation):
            gw.request_channel(Channel.STAIR, 2_000)
        gw.disable(Channel.COMMON_NAV, 3_000)
        enable(gw, Channel.STAIR, 4_000)
        self.assertIs(gw.enabled_channel(), Channel.STAIR)

    def test_generation_monotonic_and_stream_appended(self):
        gw, _ = make_gateway()
        g1 = enable(gw, Channel.COMMON_NAV, 1_000)
        gw.disable(Channel.COMMON_NAV, 2_000)
        g2 = enable(gw, Channel.STAIR, 3_000)
        self.assertLess(g1, g2)
        stream = gw.transition_stream()
        self.assertGreaterEqual(len(stream), 9)  # 4+1+4 遷移が全件記録
        gens = [r.generation for r in stream if r.generation is not None]
        self.assertEqual(gens, sorted(gens))


class TestManifest(unittest.TestCase):
    def test_branch_l_requires_policy_hash(self):
        with self.assertRaises(ContractViolation):
            RunManifestControl(run_id="r", policy_hash="not_applicable",
                               selected_backend=LocomotionBackend.LEARNED_LOWCMD,
                               operator_lease_id=LEASE)

    def test_branch_s_allows_not_applicable(self):
        RunManifestControl(run_id="r", policy_hash="not_applicable",
                           selected_backend=LocomotionBackend.SPORT_STAIR_API,
                           operator_lease_id=LEASE)


class TestPolicyHashGate(unittest.TestCase):
    def test_stair_mode_hash_mismatch_rejected_and_latched(self):
        # docs/08 §4.3: hash 不一致は1フレームも通さない
        gw, arb = make_gateway()
        enable(gw, Channel.STAIR, 1_000)
        env = make_env(seq=1, mode="ASCEND", policy_hash="not_applicable")
        self.assertTrue(arb.submit(env, make_attr(accepted_ns=2_000), 2_000).accepted)
        act = gw.tick(FLAGS, 2_001)
        self.assertIs(act.kind, ActionKind.NOOP)
        # latch されて次 tick から停止列
        d = arb.current(2_002)
        self.assertIs(d.kind, DirectiveKind.LATCHED_STOP)

    def test_stair_mode_matching_hash_forwards(self):
        gw, arb = make_gateway()
        enable(gw, Channel.STAIR, 1_000)
        env = make_env(seq=1, mode="ASCEND", policy_hash=HASH)
        arb.submit(env, make_attr(accepted_ns=2_000), 2_000)
        act = gw.tick(FLAGS, 2_001)
        self.assertIs(act.kind, ActionKind.FORWARD)
        self.assertIs(act.channel, Channel.STAIR)
        self.assertIs(act.envelope, env)

    def test_common_nav_hash_not_checked(self):
        gw, arb = make_gateway()
        enable(gw, Channel.COMMON_NAV, 1_000)
        env = make_env(seq=1, mode="COMMON_NAV", policy_hash="not_applicable")
        arb.submit(env, make_attr(accepted_ns=2_000), 2_000)
        act = gw.tick(FLAGS, 2_001)
        self.assertIs(act.kind, ActionKind.FORWARD)


class TestRouting(unittest.TestCase):
    def test_no_forward_without_enabled_channel(self):
        gw, arb = make_gateway()
        env = make_env(seq=1)
        arb.submit(env, make_attr(accepted_ns=1_000), 1_000)
        act = gw.tick(FLAGS, 1_001)
        self.assertIs(act.kind, ActionKind.NOOP)
        self.assertIn("ENABLED でない", act.reason)


class TestStopProgression(unittest.TestCase):
    def test_active_hold_satisfies_stop_now(self):
        # 受入条件 (c): ACTIVE_HOLD 滞在中の STOP_NOW latch は滞在で充足
        gw, arb = make_gateway()
        op = make_attr(source="operator",
                       priority=ArbiterPriority.OPERATOR_STOP_OR_DISARM,
                       accepted_ns=1_000)
        arb.submit(make_env(seq=1, mode="STOP_NOW"), op, 1_000)
        act = gw.tick(FLAGS, 1_001)  # 初期状態は ACTIVE_HOLD
        self.assertIs(act.kind, ActionKind.HOLD)
        self.assertIs(act.stop_state, StopState.ACTIVE_HOLD)

    def test_motion_stop_sequence(self):
        # 運動中の STOP_NOW → STOP_NOW → CONTROLLED_STOP → (整定+接地) → ACTIVE_HOLD
        gw, arb = make_gateway()
        enable(gw, Channel.COMMON_NAV, 1_000)
        arb.submit(make_env(seq=1), make_attr(accepted_ns=2_000), 2_000)
        self.assertIs(gw.tick(FLAGS, 2_001).kind, ActionKind.FORWARD)  # 運動中
        op = make_attr(source="operator",
                       priority=ArbiterPriority.OPERATOR_STOP_OR_DISARM,
                       accepted_ns=2_002)
        arb.submit(make_env(seq=2, mode="STOP_NOW"), op, 2_002)
        a1 = gw.tick(FLAGS, 2_003)
        self.assertIs(a1.stop_state, StopState.STOP_NOW)       # 停止列へ入る
        a2 = gw.tick(FLAGS, 2_004)
        self.assertIs(a2.stop_state, StopState.CONTROLLED_STOP)  # 減速整定へ
        a3 = gw.tick(FLAGS, 2_005)  # guard 未成立 → 維持
        self.assertIs(a3.stop_state, StopState.CONTROLLED_STOP)
        self.assertIn("guard 未成立", a3.reason)
        a4 = gw.tick(SETTLED, 2_006)  # 整定+接地確認 → 能動保持
        self.assertIs(a4.stop_state, StopState.ACTIVE_HOLD)
        self.assertIs(a4.kind, ActionKind.HOLD)

    def test_damp_requires_critical_fault_flag(self):
        gw, arb = make_gateway()
        arb.latch_fault(ArbiterPriority.PHYSICAL_ESTOP_OR_HARD_FAULT,
                        StopState.DAMP_CRITICAL_STOP, "control divergence", 1_000)
        # critical 事実が未検証なら能動保持を維持(invariant 8 側に倒す)
        a1 = gw.tick(FLAGS, 1_001)
        self.assertIs(a1.stop_state, StopState.ACTIVE_HOLD)
        self.assertIn("guard 未成立", a1.reason)
        a2 = gw.tick(RobotStatusFlags(critical_fault=True), 1_002)
        self.assertIs(a2.stop_state, StopState.DAMP_CRITICAL_STOP)


class TestLeaseGatedRelease(unittest.TestCase):
    def test_release_requires_manifest_lease(self):
        # 受入条件 (a): manifest の operator lease と不一致なら転送しない
        gw, arb = make_gateway()
        op = make_attr(source="operator",
                       priority=ArbiterPriority.OPERATOR_STOP_OR_DISARM,
                       accepted_ns=1_000)
        arb.submit(make_env(seq=1, mode="STOP_NOW"), op, 1_000)
        with self.assertRaises(ContractViolation):
            gw.release_latch(ArbiterPriority.OPERATOR_STOP_OR_DISARM,
                             FULL_RESET, _uuid(), 1_001)  # 別 lease
        self.assertEqual(len(arb.latches()), 1)  # latch は残る
        d = gw.release_latch(ArbiterPriority.OPERATOR_STOP_OR_DISARM,
                             FULL_RESET, LEASE, 1_002)
        self.assertTrue(d.accepted)


class TestStopNowEscalation(unittest.TestCase):
    def test_malformed_stop_now_escalates_to_latch(self):
        # 不正な形の STOP_NOW は拒否を通常エラーにせず停止へエスカレート
        gw, arb = make_gateway()
        bad = make_env(seq=1, mode="STOP_NOW").to_dict()
        bad["vx"] = 0.5  # STOP_NOW に速度同乗 → 契約違反
        out = gw.submit_raw(bad, make_attr(
            source="operator", priority=ArbiterPriority.OPERATOR_STOP_OR_DISARM,
            accepted_ns=1_000), 1_000)
        self.assertIsNone(out)
        d = arb.current(1_001)
        self.assertIs(d.kind, DirectiveKind.LATCHED_STOP)
        self.assertIs(d.stop_state, StopState.STOP_NOW)

    def test_malformed_normal_command_reraises(self):
        gw, _ = make_gateway()
        bad = make_env(seq=1).to_dict()
        bad["source_priority"] = 1  # 自称 priority の注入
        with self.assertRaises(ContractViolation):
            gw.submit_raw(bad, make_attr(accepted_ns=1_000), 1_000)

    def test_valid_payload_passes_through(self):
        gw, arb = make_gateway()
        ok = make_env(seq=1).to_dict()
        d = gw.submit_raw(ok, make_attr(accepted_ns=1_000), 1_000)
        self.assertTrue(d.accepted)


class TestTypeChecks(unittest.TestCase):
    def test_constructor_validation(self):
        with self.assertRaises(ContractViolation):
            ExclusiveActuationGateway({"backend": "L"}, CommandArbiter())
        with self.assertRaises(ContractViolation):
            ExclusiveActuationGateway(make_manifest(), {"not": "arbiter"})

    def test_tick_requires_flags(self):
        gw, _ = make_gateway()
        with self.assertRaises(ContractViolation):
            gw.tick({"critical_fault": False}, 1_000)

    def test_status_flags_reject_truthy(self):
        with self.assertRaises(ContractViolation):
            RobotStatusFlags(critical_fault="yes")


if __name__ == "__main__":
    unittest.main()
