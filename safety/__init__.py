"""safety — 独立 Safety Supervisor と停止遷移契約(docs/02 §10 の境界)。

このパッケージの純ロジック部分は robot 非接続で test 可能にする。
Supervisor 本体(独立 process、heartbeat、latched safe-state request)は
Phase 1 の後続 task。
"""
