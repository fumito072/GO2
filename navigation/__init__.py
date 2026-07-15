"""navigation — 平地 navigation / 探索 planner(docs/02 §10 の境界)。

planner は goal を提案するだけで actuator owner にならない(invariant 1)。
すべての goal は CommandEnvelope として Command Arbiter を通る。
"""
