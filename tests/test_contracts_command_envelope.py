"""CommandEnvelope / LocomotionCommand の契約テスト(Gate 0 素材、robot 非接続)。

docs/08 §4.2: 自称 priority の拒否、expiry 必須、方向 command の非混同。
docs/02 §7: LocomotionCommand の入力限定、S/L 排他は gateway 責務。
"""
import math
import unittest
import uuid

from contracts import ContractViolation
from contracts.command_envelope import (
    CommandEnvelope, LocomotionCommand, StairGeometrySummary,
    RequestedMode, LocomotionBackend, LocomotionMode, ArbiterPriority,
    ServerAttribution,
)
from contracts.stair_model import StairDirection


def _uuid():
    return str(uuid.uuid4())


def make_envelope_dict():
    return {
        "schema_version": "1.0",
        "source_id": "nav_local_planner",
        "goal_id": _uuid(),
        "actuation_request_id": _uuid(),
        "sender_timestamp": 123456789,
        "sequence": 0,
        "expires_after_ms": 200,
        "requested_mode": "COMMON_NAV",
        "vx": 0.2, "vy": 0.0, "wz": 0.1,
        "phase": "NAV_FLAT",
        "policy_hash": "not_applicable",
    }


def make_summary(direction="UP"):
    return {"stair_id": "test_stair_001", "direction": direction,
            "visible_steps": 4, "riser_height_min_m": 0.09,
            "riser_height_max_m": 0.11, "fresh_coverage_landing": 0.9}


def make_locomotion_dict(mode="ASCEND", direction="UP"):
    return {
        "schema_version": "1.0",
        "backend": "LEARNED_LOWCMD",
        "mode": mode,
        "local_goal": [0.5, 0.0, 0.0, 0.4],
        "velocity_hint": [0.15, 0.0, 0.0],
        "stair_geometry_summary": make_summary(direction),
        "perception_confidence": 0.9,
        "command_deadline_monotonic_ns": 999999,
    }


class TestCommandEnvelope(unittest.TestCase):
    def test_roundtrip(self):
        env = CommandEnvelope.from_dict(make_envelope_dict())
        out = env.to_dict()
        self.assertEqual(CommandEnvelope.from_dict(out).to_dict(), out)

    def test_reject_self_declared_priority(self):
        # docs/08 §4.2: source_priority は送信者 field にしない
        d = make_envelope_dict()
        d["source_priority"] = 1
        with self.assertRaises(ContractViolation):
            CommandEnvelope.from_dict(d)

    def test_reject_trusted_source_injection(self):
        d = make_envelope_dict()
        d["trusted_source_id"] = "supervisor"
        with self.assertRaises(ContractViolation):
            CommandEnvelope.from_dict(d)

    def test_reject_nan_velocity(self):
        for key in ("vx", "vy", "wz"):
            d = make_envelope_dict()
            d[key] = math.nan
            with self.assertRaises(ContractViolation, msg=key):
                CommandEnvelope.from_dict(d)

    def test_reject_over_velocity(self):
        d = make_envelope_dict()
        d["vx"] = 1.5
        with self.assertRaises(ContractViolation):
            CommandEnvelope.from_dict(d)

    def test_stop_now_zero_velocity_only(self):
        d = make_envelope_dict()
        d["requested_mode"] = "STOP_NOW"
        d["vx"], d["vy"], d["wz"] = 0.0, 0.0, 0.0
        CommandEnvelope.from_dict(d)  # OK
        d["vx"] = 0.1
        with self.assertRaises(ContractViolation):
            CommandEnvelope.from_dict(d)

    def test_hold_zero_velocity_only(self):
        d = make_envelope_dict()
        d["requested_mode"] = "HOLD"
        d["vx"], d["vy"], d["wz"] = 0.0, 0.0, 0.0
        CommandEnvelope.from_dict(d)
        d["wz"] = -0.2
        with self.assertRaises(ContractViolation):
            CommandEnvelope.from_dict(d)

    def test_expiry_required_and_bounded(self):
        for bad in (0, -5, 10001):
            d = make_envelope_dict()
            d["expires_after_ms"] = bad
            with self.assertRaises(ContractViolation, msg=str(bad)):
                CommandEnvelope.from_dict(d)

    def test_sequence_nonnegative(self):
        d = make_envelope_dict()
        d["sequence"] = -1
        with self.assertRaises(ContractViolation):
            CommandEnvelope.from_dict(d)

    def test_timestamp_positive(self):
        d = make_envelope_dict()
        d["sender_timestamp"] = 0
        with self.assertRaises(ContractViolation):
            CommandEnvelope.from_dict(d)

    def test_policy_hash_format(self):
        ok = make_envelope_dict()
        ok["policy_hash"] = "sha256:" + "a" * 64
        CommandEnvelope.from_dict(ok)
        for bad in ("", "abc", "sha256:" + "A" * 64, "md5:" + "a" * 32,
                    "not_applicable\n", "sha256:" + "a" * 64 + "\n"):
            d = make_envelope_dict()
            d["policy_hash"] = bad
            with self.assertRaises(ContractViolation, msg=repr(bad)):
                CommandEnvelope.from_dict(d)

    def test_stair_mode_with_not_applicable_passes_contract_layer(self):
        # 契約層は policy_hash の形式のみ検査する(責務境界の固定)。
        # ASCEND + not_applicable の拒否は arbiter が signed run manifest の
        # selected_backend と突合して行う(次 task。docs/08 §4.3, docs/09 §5)。
        d = make_envelope_dict()
        d["requested_mode"] = "ASCEND"
        d["policy_hash"] = "not_applicable"
        CommandEnvelope.from_dict(d)  # 契約層では通る — arbiter 実装時に責務移管

    def test_replace_revalidates(self):
        # 検証迂回経路の閉鎖(__post_init__)
        import dataclasses
        env = CommandEnvelope.from_dict(make_envelope_dict())
        with self.assertRaises(ContractViolation):
            dataclasses.replace(env, vx=5.0)

    def test_direction_modes_distinct(self):
        # Gate 0: ascent / forward descent / backward descent の混同禁止
        names = {RequestedMode.ASCEND, RequestedMode.DESCEND_BACKWARD,
                 RequestedMode.DESCEND_FORWARD}
        self.assertEqual(len(names), 3)
        values = {m.value for m in RequestedMode}
        self.assertEqual(len(values), len(list(RequestedMode)))


class TestArbiterPriority(unittest.TestCase):
    def test_eight_levels_ordered(self):
        # docs/08 §4.2 の8段。数値が小さいほど高優先
        self.assertEqual(len(list(ArbiterPriority)), 8)
        self.assertLess(ArbiterPriority.PHYSICAL_ESTOP_OR_HARD_FAULT,
                        ArbiterPriority.OPERATOR_STOP_OR_DISARM)
        self.assertLess(ArbiterPriority.OPERATOR_STOP_OR_DISARM,
                        ArbiterPriority.SUPERVISOR_SOFT_FAULT_OR_SENSOR_STALE)
        self.assertLess(ArbiterPriority.STAIR_STATE_MACHINE,
                        ArbiterPriority.NAV_LOCAL_PLANNER)
        self.assertLess(ArbiterPriority.NAV_LOCAL_PLANNER,
                        ArbiterPriority.VLM_PROPOSED_GOAL)

    def test_attribution_is_server_side_only(self):
        # wire から構築する from_dict を持たない(自称 priority の禁止)
        self.assertFalse(hasattr(ServerAttribution, "from_dict"))
        attr = ServerAttribution("supervisor", ArbiterPriority.OPERATOR_STOP_OR_DISARM, 1)
        attr.validate()


class TestLocomotionCommand(unittest.TestCase):
    def test_ascend_roundtrip(self):
        cmd = LocomotionCommand.from_dict(make_locomotion_dict())
        self.assertIs(cmd.mode, LocomotionMode.ASCEND)
        out = cmd.to_dict()
        self.assertEqual(LocomotionCommand.from_dict(out).to_dict(), out)

    def test_ascend_requires_summary(self):
        d = make_locomotion_dict()
        d["stair_geometry_summary"] = None
        with self.assertRaises(ContractViolation):
            LocomotionCommand.from_dict(d)

    def test_ascend_rejects_down_geometry(self):
        # 上り command に DOWN 幾何 — 方向混同の型レベル防止
        d = make_locomotion_dict(mode="ASCEND", direction="DOWN")
        with self.assertRaises(ContractViolation):
            LocomotionCommand.from_dict(d)

    def test_descend_backward_requires_down(self):
        ok = make_locomotion_dict(mode="DESCEND_BACKWARD", direction="DOWN")
        LocomotionCommand.from_dict(ok)
        bad = make_locomotion_dict(mode="DESCEND_BACKWARD", direction="UP")
        with self.assertRaises(ContractViolation):
            LocomotionCommand.from_dict(bad)

    def test_descend_forward_requires_down(self):
        bad = make_locomotion_dict(mode="DESCEND_FORWARD", direction="UNKNOWN")
        with self.assertRaises(ContractViolation):
            LocomotionCommand.from_dict(bad)

    def test_hold_requires_zero_hint(self):
        d = make_locomotion_dict(mode="HOLD")
        d["stair_geometry_summary"] = None
        d["velocity_hint"] = [0.0, 0.0, 0.0]
        LocomotionCommand.from_dict(d)  # OK
        d["velocity_hint"] = [0.1, 0.0, 0.0]
        with self.assertRaises(ContractViolation):
            LocomotionCommand.from_dict(d)

    def test_backends_distinct(self):
        self.assertEqual(len(list(LocomotionBackend)), 2)

    def test_reject_unknown_key(self):
        d = make_locomotion_dict()
        d["joint_targets"] = [0.0] * 12  # Mission は joint target を作らない(docs/02 §7)
        with self.assertRaises(ContractViolation):
            LocomotionCommand.from_dict(d)

    def test_reject_bad_local_goal(self):
        d = make_locomotion_dict()
        d["local_goal"] = [0.5, 0.0, 0.0]  # 要素数不足
        with self.assertRaises(ContractViolation):
            LocomotionCommand.from_dict(d)

    def test_reject_zero_deadline(self):
        d = make_locomotion_dict()
        d["command_deadline_monotonic_ns"] = 0
        with self.assertRaises(ContractViolation):
            LocomotionCommand.from_dict(d)

    def test_summary_riser_range(self):
        d = make_locomotion_dict()
        d["stair_geometry_summary"]["riser_height_max_m"] = 0.5
        with self.assertRaises(ContractViolation):
            LocomotionCommand.from_dict(d)
        d2 = make_locomotion_dict()
        d2["stair_geometry_summary"]["riser_height_min_m"] = 0.12
        d2["stair_geometry_summary"]["riser_height_max_m"] = 0.10
        with self.assertRaises(ContractViolation):
            LocomotionCommand.from_dict(d2)


if __name__ == "__main__":
    unittest.main()
