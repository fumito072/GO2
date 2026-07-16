"""demo — offline E2E デモ(docs/10 §6 の E0-E2)。robot 非接続・決定的。

実機・DDS・network を一切使わない。実行経路は本番と同じ
voice → GoalSpec → Mission FSM → Command Arbiter → Exclusive Actuation
Gateway → (合成世界の kinematic sim)。
"""
