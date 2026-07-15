"""realtime — Exclusive Actuation Gateway / LowCmd servo(docs/02 §10 の境界)。

Gateway は唯一の actuator authority 切替点。common NAV / Branch S / Branch L を
排他的に選択し(invariant 3)、selected backend は arm 前 manifest で固定する
(invariant 6)。LowCmd servo(C++ 候補・sole DDS writer)は実機接続 task。
"""
