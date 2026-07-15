"""voice_gateway — 音声/自然言語 → GoalSpec 変換層(docs/02 §10 の境界)。

invariant 1: ASR/LLM/UI は GoalProposal / GoalSpec を生成するだけで
actuator owner にならない。ここから先は Mission FSM → Command Arbiter →
Exclusive Actuation Gateway の一方向経路のみ。
"""
