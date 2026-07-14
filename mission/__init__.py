"""mission — Mission Executive / affordance validator / Command Arbiter
(docs/02 §10 の境界)。

実行経路の invariant(docs/CLAUDE.md invariant 2):
  GoalSpec → Mission FSM → Command Arbiter → Exclusive Actuation Gateway
  → selected backend(一方向)。

本パッケージの純ロジック部分は robot 非接続で test 可能にする。
Mission FSM 本体と gateway は Phase 1 の後続 task。
"""
