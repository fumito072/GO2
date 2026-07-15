"""MissionExecutive の FSM / affordance validation テスト(robot 非接続)。"""
import unittest
import uuid

from contracts import ContractViolation
from contracts.goal_spec import (
    CompletionPredicate, Confidence, Confirmation, ConfirmationStatus,
    Completion, Constraints, GoalSpec, Intent, Modality, Precondition,
    Source, Target, Transcript, TranscriptEvidence, INTENT_TO_PREDICATE,
    SCHEMA_VERSION,
)
from mission.executive import (
    AffordanceContext, MissionExecutive, MissionState,
)

LEASE = str(uuid.uuid4())
CTX = AffordanceContext(operator_lease_valid=True, supervisor_ok=True)


def make_spec(intent=Intent.EXPLORE_AND_MAP, lease=LEASE, utterance=None,
              parser="grammar-ja-v1", goal_id=None):
    target = {
        Intent.EXPLORE_AND_MAP: Target("area", "current_room", None),
        Intent.NAVIGATE_TO_WAYPOINT: Target("waypoint", "home", None),
        Intent.NAVIGATE_TO_STAIR_APPROACH: Target("stairs", "current", None),
        Intent.ASCEND_STAIRS: Target("stairs", "current", None),
        Intent.DESCEND_STAIRS: Target("stairs", "current", None),
    }[intent]
    required = intent in (Intent.ASCEND_STAIRS, Intent.DESCEND_STAIRS,
                          Intent.EXPLORE_AND_MAP)
    return GoalSpec(
        schema_version=SCHEMA_VERSION,
        goal_id=goal_id or str(uuid.uuid4()),
        source=Source(Modality.VOICE, lease, str(uuid.uuid4()),
                      utterance or str(uuid.uuid4())),
        transcript=Transcript("t", "ja", "m",
                              TranscriptEvidence(0.97, 0.01)),
        intent=intent,
        target=target,
        completion=Completion(INTENT_TO_PREDICATE[intent], "ACTIVE_HOLD", 1.0),
        constraints=Constraints(),
        confidence=Confidence(0.98, 1.0, parser),
        confirmation=(Confirmation(True, ConfirmationStatus.CONFIRMED,
                                   str(uuid.uuid4()), str(uuid.uuid4()))
                      if required else
                      Confirmation(False, ConfirmationStatus.NOT_REQUIRED,
                                   None, None)),
        preconditions=(Precondition.OPERATOR_LEASE_VALID,
                       Precondition.ROBOT_ARMED,
                       Precondition.ACTIVE_STAIR_GEOMETRY_VALID,
                       Precondition.SAFETY_SUPERVISOR_OK),
        created_at_utc="2026-07-15T12:00:00Z",
        expires_after_ms=5000,
    )


def make_exec():
    ex = MissionExecutive(expected_operator_lease_id=LEASE)
    ex.arm(self_check_ok=True, now_ns=1_000)
    return ex


class TestArming(unittest.TestCase):
    def test_starts_disarmed_and_arms(self):
        ex = MissionExecutive(expected_operator_lease_id=LEASE)
        self.assertIs(ex.state, MissionState.DISARMED)
        self.assertFalse(ex.arm(self_check_ok=False, now_ns=1_000).accepted)
        self.assertTrue(ex.arm(self_check_ok=True, now_ns=1_001).accepted)
        self.assertIs(ex.state, MissionState.IDLE)

    def test_goal_rejected_when_disarmed(self):
        ex = MissionExecutive(expected_operator_lease_id=LEASE)
        d = ex.accept_goal(make_spec(), CTX, 1_000)
        self.assertFalse(d.accepted)


class TestAffordance(unittest.TestCase):
    def test_explore_accepted_in_idle(self):
        ex = make_exec()
        d = ex.accept_goal(make_spec(), CTX, 2_000)
        self.assertTrue(d.accepted, d.reason)
        self.assertIs(ex.state, MissionState.EXPLORING)

    def test_ascend_rejected_in_idle(self):
        # 階段昇降は AT_BASE_HOLD からのみ(docs/02 §5)
        ex = make_exec()
        d = ex.accept_goal(make_spec(Intent.ASCEND_STAIRS), CTX, 2_000)
        self.assertFalse(d.accepted)

    def test_supervisor_and_lease_fail_closed(self):
        ex = make_exec()
        no_sup = AffordanceContext(operator_lease_valid=True, supervisor_ok=False)
        self.assertFalse(ex.accept_goal(make_spec(), no_sup, 2_000).accepted)
        no_lease = AffordanceContext(operator_lease_valid=False, supervisor_ok=True)
        self.assertFalse(ex.accept_goal(make_spec(), no_lease, 2_001).accepted)

    def test_wrong_lease_rejected(self):
        ex = make_exec()
        d = ex.accept_goal(make_spec(lease=str(uuid.uuid4())), CTX, 2_000)
        # spec の lease は正しく作れるが executive の期待 lease と不一致
        self.assertFalse(d.accepted)
        self.assertIn("lease", d.reason)

    def test_parser_allowlist(self):
        ex = make_exec()
        d = ex.accept_goal(make_spec(parser="unknown-parser-v9"), CTX, 2_000)
        self.assertFalse(d.accepted)

    def test_duplicate_utterance_rejected(self):
        # 同一 utterance の再送(goal_id 違い)は idempotent 拒否(docs/06 §4.2)
        ex = make_exec()
        utt = str(uuid.uuid4())
        self.assertTrue(ex.accept_goal(make_spec(utterance=utt), CTX, 2_000).accepted)
        ex.notify_stop_now(2_001)  # EXPLORING を抜ける
        d = ex.accept_goal(make_spec(utterance=utt), CTX, 2_002)
        self.assertFalse(d.accepted)
        self.assertIn("重複", d.reason)

    def test_stop_now_not_via_goal_queue(self):
        ex = make_exec()
        stop = GoalSpec(
            schema_version=SCHEMA_VERSION, goal_id=str(uuid.uuid4()),
            source=Source(Modality.TEXT, LEASE, str(uuid.uuid4()), None),
            transcript=Transcript("止まれ", "ja", "m"),
            intent=Intent.STOP_NOW, target=None,
            completion=Completion(CompletionPredicate.IMMEDIATE, "ACTIVE_HOLD", 1.0),
            constraints=Constraints(),
            confidence=Confidence(1.0, 1.0, "grammar-ja-v1"),
            confirmation=Confirmation(False, ConfirmationStatus.NOT_REQUIRED,
                                      None, None),
            preconditions=(), created_at_utc="2026-07-15T12:00:00Z",
            expires_after_ms=1000)
        d = ex.accept_goal(stop, CTX, 2_000)
        self.assertFalse(d.accepted)
        self.assertIn("arbiter", d.reason)


class TestTransitions(unittest.TestCase):
    def test_exploration_lifecycle(self):
        ex = make_exec()
        ex.accept_goal(make_spec(), CTX, 2_000)
        d = ex.notify_completion(CompletionPredicate.EXPLORATION_COMPLETE, 3_000)
        self.assertTrue(d.accepted)
        self.assertIs(ex.state, MissionState.ACTIVE_HOLD)
        self.assertIsNone(ex.active_goal())

    def test_completion_state_mismatch_rejected(self):
        ex = make_exec()
        d = ex.notify_completion(CompletionPredicate.EXPLORATION_COMPLETE, 2_000)
        self.assertFalse(d.accepted)  # EXPLORING でないのに完了通知

    def test_stop_now_saves_resume_context(self):
        ex = make_exec()
        ex.accept_goal(make_spec(), CTX, 2_000)
        d = ex.notify_stop_now(3_000)
        self.assertTrue(d.accepted)
        self.assertIs(ex.state, MissionState.ACTIVE_HOLD)
        self.assertIs(ex.resume_context(), MissionState.EXPLORING)
        # 自動再開はない — 新しい goal が必要(ACTIVE_HOLD から EXPLORE 可)
        d2 = ex.accept_goal(make_spec(), CTX, 4_000)
        self.assertTrue(d2.accepted)
        self.assertIs(ex.state, MissionState.EXPLORING)

    def test_critical_stop_manual_recovery_only(self):
        ex = make_exec()
        ex.accept_goal(make_spec(), CTX, 2_000)
        ex.notify_critical_fault("test fault", 3_000)
        self.assertIs(ex.state, MissionState.CRITICAL_STOP)
        # goal は受理されない
        self.assertFalse(ex.accept_goal(make_spec(), CTX, 4_000).accepted)
        # manual recovery → DISARMED → 再 arm
        self.assertTrue(ex.manual_recovery(5_000).accepted)
        self.assertIs(ex.state, MissionState.DISARMED)
        self.assertTrue(ex.arm(True, 6_000).accepted)

    def test_must_start_by(self):
        ex = make_exec()
        ex.accept_goal(make_spec(), CTX, 2_000)  # expires 5000ms
        self.assertTrue(ex.goal_started_in_time(2_000 + 4_000 * 1_000_000))
        self.assertFalse(ex.goal_started_in_time(2_000 + 6_000 * 1_000_000))
        ex.abandon_goal("must_start_by 超過", 2_000 + 7_000 * 1_000_000)
        self.assertIs(ex.state, MissionState.ACTIVE_HOLD)

    def test_clock_monotonic(self):
        ex = make_exec()
        with self.assertRaises(ContractViolation):
            ex.accept_goal(make_spec(), CTX, 500)  # 逆行


if __name__ == "__main__":
    unittest.main()
