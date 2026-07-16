"""voice_gateway.intent_parser の regression テスト。

docs/06 §12.2 の固定 regression 文を全件含む(Gate 8 / Phase 3 の offline 素材)。
現行 cockpit/voice.py の「止まれ」substring reflex が起こす誤解釈
(README_JP.md:42 の公式例文が実行不能になる問題)の再発防止を固定する。
"""
import unittest
import uuid

from contracts.goal_spec import (
    Intent, CompletionPredicate, Modality, TranscriptEvidence,
)
from voice_gateway.intent_parser import (
    ParseKind, ParserContext, parse_utterance,
)


def make_ctx(modality=Modality.VOICE):
    return ParserContext(
        modality=modality,
        operator_lease_id=str(uuid.uuid4()),
        session_id=str(uuid.uuid4()),
        utterance_id=str(uuid.uuid4()) if modality is Modality.VOICE else None,
        asr_model_id="faster-whisper-test",
        now_monotonic_ns=1_000,
        created_at_utc="2026-07-15T00:00:00Z",
        id_factory=lambda: str(uuid.uuid4()),
        evidence=(TranscriptEvidence(asr_quality_score=0.97,
                                     no_speech_probability=0.01)
                  if modality is Modality.VOICE else None),
    )


def parse(text, modality=Modality.VOICE):
    return parse_utterance(text, make_ctx(modality))


class TestRegressionSet(unittest.TestCase):
    """docs/06 §12.2 の固定 regression 文(全14項目中、音声波形系3件を除く11件)。"""

    def test_tomare(self):
        r = parse("止まれ")
        self.assertIs(r.kind, ParseKind.STOP_NOW)
        self.assertIs(r.stop_spec.intent, Intent.STOP_NOW)
        self.assertIs(r.stop_spec.completion.predicate, CompletionPredicate.IMMEDIATE)
        self.assertFalse(r.stop_spec.confirmation.required)

    def test_ascend_with_conditional_stop(self):
        # 文末の「止まれ」は completion 条件であり STOP_NOW ではない(docs/02 §4.2)
        r = parse("その階段を登って一番上まで行ったら止まれ")
        self.assertIs(r.kind, ParseKind.PROPOSAL)
        self.assertIs(r.proposal.intent, Intent.ASCEND_STAIRS)
        self.assertIs(r.proposal.completion.predicate,
                      CompletionPredicate.TOP_LANDING_STABLE)
        self.assertEqual(r.proposal.completion.terminal_action, "ACTIVE_HOLD")

    def test_descend_with_conditional_stop(self):
        r = parse("下りて、下り切ったらまた止まれ")
        self.assertIs(r.kind, ParseKind.PROPOSAL)
        self.assertIs(r.proposal.intent, Intent.DESCEND_STAIRS)
        self.assertIs(r.proposal.completion.predicate,
                      CompletionPredicate.BOTTOM_LANDING_STABLE)

    def test_climb_without_stopping_rejected(self):
        # 「止まらずに登れ」は STOP ではないが安全制約を弱めるため task ごと拒否
        r = parse("止まらずに登れ")
        self.assertIs(r.kind, ParseKind.REJECTED)

    def test_dont_stop_is_not_command(self):
        r = parse("止まらないで")
        self.assertIs(r.kind, ParseKind.NON_COMMAND)

    def test_quoted_stop_question(self):
        r = parse("「止まれ」って言ったら止まるの?")
        self.assertIs(r.kind, ParseKind.NON_COMMAND)

    def test_prohibition_is_not_command(self):
        r = parse("階段は登らないで")
        self.assertIs(r.kind, ParseKind.NON_COMMAND)

    def test_correction_then_cancel(self):
        r = parse("登って、いや、やっぱりやめて")
        self.assertIs(r.kind, ParseKind.CANCEL)

    def test_compound_ascend_descend_needs_clarification(self):
        # 「上ってから下りて」は一括実行しない(上端 HOLD 後に別命令 — §8.3)
        r = parse("上ってから下りて")
        self.assertIs(r.kind, ParseKind.CLARIFICATION)

    def test_reference_only_needs_clarification(self):
        r = parse("前の階段ではなく右の階段")
        self.assertIs(r.kind, ParseKind.CLARIFICATION)

    def test_stopwatch_is_not_stop(self):
        # 「ストップ」の substring 誤爆の再発防止
        r = parse("ストップウォッチを見せて")
        self.assertIs(r.kind, ParseKind.NON_COMMAND)


class TestStopSemantics(unittest.TestCase):
    def test_immediate_markers(self):
        for text in ("今すぐ止まって", "直ちに停止", "そこで止まれ", "ストップ"):
            r = parse(text)
            self.assertIs(r.kind, ParseKind.STOP_NOW, text)

    def test_immediate_marker_overrides_skill(self):
        # 「今すぐ」があれば skill があっても STOP_NOW(§8.3)
        r = parse("登るのはいいから今すぐ止まって")
        self.assertIs(r.kind, ParseKind.STOP_NOW)

    def test_stop_after_skill_is_completion_not_stop_now(self):
        # README_JP.md:42 の公式例文が反射停止に飲まれない(現行 parser の再発防止)
        r = parse("階段まで行って登って止まって")
        self.assertIs(r.kind, ParseKind.PROPOSAL)
        self.assertIs(r.proposal.intent, Intent.ASCEND_STAIRS)

    def test_stop_spec_validates_as_contract(self):
        r = parse("止まれ")
        # __post_init__ で検証済みだが、round-trip も固定する
        d = r.stop_spec.to_dict()
        from contracts.goal_spec import GoalSpec
        self.assertEqual(GoalSpec.from_dict(d).to_dict(), d)


class TestNavigationAndNewIntents(unittest.TestCase):
    def test_navigate_to_stairs(self):
        r = parse("階段の前まで行け")
        self.assertIs(r.kind, ParseKind.PROPOSAL)
        self.assertIs(r.proposal.intent, Intent.NAVIGATE_TO_STAIR_APPROACH)
        self.assertIs(r.proposal.completion.predicate,
                      CompletionPredicate.STAIR_APPROACH_POSE_REACHED)

    def test_explore_current_room(self):
        r = parse("部屋を探索してマップを作って")
        self.assertIs(r.kind, ParseKind.PROPOSAL)
        self.assertIs(r.proposal.intent, Intent.EXPLORE_AND_MAP)
        self.assertEqual(r.proposal.target.type, "area")
        self.assertEqual(r.proposal.target.ref, "current_room")
        self.assertIs(r.proposal.completion.predicate,
                      CompletionPredicate.EXPLORATION_COMPLETE)

    def test_explore_all(self):
        r = parse("家中を探索して地図を作って")
        self.assertIs(r.kind, ParseKind.PROPOSAL)
        self.assertEqual(r.proposal.target.ref, "all_reachable")

    def test_map_only_phrase(self):
        r = parse("地図を作って")
        self.assertIs(r.kind, ParseKind.PROPOSAL)
        self.assertIs(r.proposal.intent, Intent.EXPLORE_AND_MAP)

    def test_return_home(self):
        r = parse("ホームに戻って")
        self.assertIs(r.kind, ParseKind.PROPOSAL)
        self.assertIs(r.proposal.intent, Intent.NAVIGATE_TO_WAYPOINT)
        self.assertEqual(r.proposal.target.ref, "home")
        self.assertIs(r.proposal.completion.predicate,
                      CompletionPredicate.WAYPOINT_REACHED)

    def test_toorini_is_not_descend(self):
        # 「とおりに」が下降に誤一致しない(regex 穴の回帰テスト)
        r = parse("言うとおりに進んで")
        self.assertNotIn(r.kind, (ParseKind.PROPOSAL,))


class TestFailClosed(unittest.TestCase):
    def test_empty_and_ood(self):
        for text in ("", "   ", "こんにちは", "テレビの音です", "うーん"):
            r = parse(text)
            self.assertIs(r.kind, ParseKind.NON_COMMAND, repr(text))

    def test_question_not_executed(self):
        for text in ("登れますか?", "止まるの?", "階段を登るのかな?"):
            r = parse(text)
            self.assertIs(r.kind, ParseKind.NON_COMMAND, text)

    def test_no_proposal_without_valid_context(self):
        # VOICE で evidence なし → contracts 層(GoalProposal)が fail-closed
        from contracts import ContractViolation
        ctx = ParserContext(
            modality=Modality.VOICE,
            operator_lease_id=str(uuid.uuid4()),
            session_id=str(uuid.uuid4()),
            utterance_id=str(uuid.uuid4()),
            asr_model_id="m",
            now_monotonic_ns=1,
            created_at_utc="2026-07-15T00:00:00Z",
            id_factory=lambda: str(uuid.uuid4()),
            evidence=None,  # ← 欠落
        )
        with self.assertRaises(ContractViolation):
            parse_utterance("階段の前まで行け", ctx)

    def test_text_modality_works_without_utterance(self):
        r = parse("階段の前まで行け", modality=Modality.TEXT)
        self.assertIs(r.kind, ParseKind.PROPOSAL)

    def test_parser_never_returns_executable_for_ambiguity(self):
        # 曖昧・複合・訂正はすべて非実行系(fail-closed)
        for text in ("上ってから下りて", "前の階段ではなく右の階段",
                     "登って、いや、やっぱりやめて"):
            r = parse(text)
            self.assertIsNone(r.proposal, text)
            self.assertIsNone(r.stop_spec, text)


if __name__ == "__main__":
    unittest.main()
