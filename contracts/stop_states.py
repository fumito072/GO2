"""contracts.stop_states — 停止系状態の型分離。

正本: docs/08 §2 の5状態(HOLD / Controlled Stop / StopMove / Damp / E-stop)と
      docs/CLAUDE.md invariant 7 の6項の和集合 = 7状態。
      docs/02 §4(HOLD/CONTROLLED_EXIT/DAMP 分離)とも整合。

原則:
  - 通常完了・音声の「止まれ」・通信断を、無条件の Damp に変換しない(invariant 8)。
  - 正常停止は検証済みの能動姿勢保持(ACTIVE_HOLD)である。
  - STOP_NOW=操作者の停止「要求」、CONTROLLED_STOP=通常 STOP の「実行実体」
    (減速→4脚支持整定、docs/08 §2.2)、その終端は ACTIVE_HOLD。
    CONTROLLED_EXIT(平坦 landing での Sport 復帰)はさらに別概念。
  - PHYSICAL_ESTOP_FUNCTION は software 停止と独立の物理機能であり、
    未同定の間、階段 LIVE は NO-GO(docs/08 §2.5)。この enum の存在は
    装置の存在証明ではない。
"""
from enum import Enum, unique


@unique
class StopState(Enum):
    """互いに別状態・別 API。alias 禁止(@unique で強制)。"""

    ACTIVE_HOLD = "ACTIVE_HOLD"                    # 速度ゼロの能動姿勢保持(通常停止・上端/下端待機)
    STOP_NOW = "STOP_NOW"                          # 操作者の即時停止要求(確認なしで最優先受理)
    CONTROLLED_STOP = "CONTROLLED_STOP"            # 通常STOPの実行実体: 減速→4脚支持整定→ACTIVE_HOLD(docs/08 §2.2)
    STOP_MOVE = "STOP_MOVE"                        # Sport API StopMove(formal API の停止 request)
    CONTROLLED_EXIT = "CONTROLLED_EXIT"            # 十分広く平坦な landing でのみ許す安全復帰
    DAMP_CRITICAL_STOP = "DAMP_CRITICAL_STOP"      # critical fault 時のみ。階段上は落下防止設備前提
    PHYSICAL_ESTOP_FUNCTION = "PHYSICAL_ESTOP_FUNCTION"  # 独立物理停止(未同定=BLOCKED)


# 正常完了の terminal action は常に能動保持(docs/02 §4.1 completion.terminal_action)。
# Damp を正常終了経路に置く実装(m3_rl/rl_stair_controller.py:283-294 の finally)は
# Phase 1 で本契約に置換される。
TERMINAL_ACTION_FOR_NORMAL_COMPLETION = StopState.ACTIVE_HOLD
