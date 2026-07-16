"""停止系状態の型分離テスト(invariant 7-8、docs/08 §2)。"""
import unittest

from contracts.stop_states import StopState, TERMINAL_ACTION_FOR_NORMAL_COMPLETION


class TestStopStates(unittest.TestCase):
    def test_seven_distinct_states(self):
        # docs/08 §2 の5状態(HOLD/Controlled Stop/StopMove/Damp/E-stop)+
        # invariant 7 の STOP_NOW / CONTROLLED_EXIT = 7状態の型分離
        self.assertEqual(len(list(StopState)), 7)
        values = {s.value for s in StopState}
        self.assertEqual(len(values), 7)

    def test_normal_completion_is_active_hold(self):
        # 通常完了を Damp に変換しない(invariant 8)
        self.assertIs(TERMINAL_ACTION_FOR_NORMAL_COMPLETION, StopState.ACTIVE_HOLD)
        self.assertIsNot(TERMINAL_ACTION_FOR_NORMAL_COMPLETION,
                         StopState.DAMP_CRITICAL_STOP)

    def test_controlled_stop_distinct(self):
        # CONTROLLED_STOP(通常STOPの実行実体, docs/08 §2.2)は
        # STOP_NOW(要求)とも ACTIVE_HOLD(終端)とも CONTROLLED_EXIT
        # (Sport 復帰)とも別状態
        self.assertIsNot(StopState.CONTROLLED_STOP, StopState.STOP_NOW)
        self.assertIsNot(StopState.CONTROLLED_STOP, StopState.ACTIVE_HOLD)
        self.assertIsNot(StopState.CONTROLLED_STOP, StopState.CONTROLLED_EXIT)
        self.assertIsNot(StopState.CONTROLLED_STOP, StopState.DAMP_CRITICAL_STOP)

    def test_estop_is_not_damp(self):
        # physical E-stop は software Damp と同義ではない(docs/CLAUDE.md §5)
        self.assertIsNot(StopState.PHYSICAL_ESTOP_FUNCTION,
                         StopState.DAMP_CRITICAL_STOP)


if __name__ == "__main__":
    unittest.main()
