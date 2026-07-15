# contracts/ — safety-critical typed schema(Phase 1 最初の contract task)

作成: 2026-07-15(初回 RTX PC セッション)。docs/CLAUDE.md §7 Phase 1
「最初に GoalSpec, StairModel, CommandEnvelope, backend interface の schema と
validation test を作る」に対応する。3レンズ(docs 忠実性 / 実行可能性 / 安全性)
の adversarial review 済み(確定指摘 12 件を反映)。

## 正本との対応

| 型 | 正本 | 備考 |
|---|---|---|
| `GoalSpec` / `GoalProposal` | docs/02 §4.1-4.3 | docs/06 §8.1 の音声 evidence・4前提条件を統合 |
| `StairModel` / `Plane` / `FreshCoverage` | docs/05 §3.2-3.3 | 02 §7 の visible_region / unknown_mask / cell_age / top-level covariance は**未実装**(Phase 2/4 の grid 二層契約で追加) |
| `CommandEnvelope` | docs/08 §4.2 | 必須 wire field。自称 priority は未知キー拒否で機械的に弾く |
| `ServerAttribution` / `ArbiterPriority` | docs/08 §4.2 | server-side 専用(意図的に from_dict なし) |
| `LocomotionCommand` | docs/02 §7 | S/L 排他・二重配送禁止は gateway 実装(次 task)の責務 |
| `StopState` | docs/08 §2 の5状態 + invariant 7 | **7状態**の型分離(HOLD/STOP_NOW/CONTROLLED_STOP/StopMove/CONTROLLED_EXIT/DAMP/E-stop)。正常完了 = ACTIVE_HOLD |

## 設計判断(review 対象)

1. **stdlib のみ**(dataclasses + enum)。新規依存ゼロ(CLAUDE.md §6.2)。
2. **未知キー拒否を既定**とする strict parser。`max_top_hold_s`(02 §4.3)や
   `source_priority`(08 §4.2)の注入を schema 層で拒否する。
3. `StairGeometrySummary` の具体 field(riser min/max 等)は 02 §7 の
   `stair_geometry_summary` の最小実装案 — Phase 4 知覚実装時に review。
4. `RequestedMode` に COMMON_NAV / STOP_NOW を追加(invariant 3 と 08 の
   優先順位表から導出)— review 対象。
5. sanity 範囲(riser 0.01-0.30 m 等)は契約レベルの粗い防壁であり、
   適合判定は `training_envelope_match` と stair_registry 照合が担う。
6. StairModel は昇降の許可 method を持たない(invariant 10 — 許可は safety 層)。
7. **検証迂回経路なし**: トップレベル契約型は `__post_init__` で validate() が
   走る(直接コンストラクタ・`dataclasses.replace()` を含む)。下位型は
   from_dict が自己検証する。例外: `Transcript` は modality 文脈が必要なため
   単体 from_dict は部分検証(完全検証は親経由)。
8. ASCEND/DESCEND は **modality によらず** Precondition 全4項を必須とする。
   02 §4.1 の schema sketch の 2 項例示より厳格側に倒した(安全緩和はしない)。
9. UUID は canonical 形式(小文字・ハイフン付き)のみ受理。uuid.UUID() の
   寛容パースは同一 UUID の複数表記を通し dedup key を迂回できるため使わない。
10. `GoalSpec.dedup_keys()` は OR 意味論の 2 key(goal / utterance)を返す
    (docs/06 §4.2、docs/09 §2)。

## STOP_NOW 拒否時の受け側必須要件

不正な形の STOP_NOW は本契約が fail-closed で拒否するが、**拒否 = 停止しない**。
ContractViolation を受けた層は、raw payload の intent が "STOP_NOW" の場合、
拒否を通常エラー応答にせず arbiter 優先度2(OPERATOR_STOP_OR_DISARM)の停止へ
エスカレートし、latched fault + structured log を残すこと(CLAUDE.md §10)。
エスカレートは認証済み operator session/lease 文脈内でのみ行う(未認証 payload
を停止 DoS ベクタにしない)。enforcement test は Command Arbiter task の受入条件。

## テスト

```bash
python -m unittest discover -s tests -v   # robot 非接続・依存ゼロ
```

Gate 0(docs/08 §9)のうち本 task が寄与する項目: 停止状態の型分離、
方向 command の非混同、finite/range property、未定義 field の拒否、
検証迂回経路の閉鎖。

## 実装済みの Gate 0 素材(本セッション追加)

- `mission/command_arbiter.py`: priority 8段調停・expiry→Controlled Stop
  (ゼロ推測禁止)・latch(自動復帰なし、同 priority は重大側のみ昇格)・
  STOP_NOW の無条件受理(stale sequence でも latch)・clock jump fail-closed。
  テスト: `tests/test_mission_command_arbiter.py`(8x8 priority 行列を含む)
- `safety/stop_transitions.py`: 停止 7 状態の遷移表(guard tuple、未定義遷移
  拒否)。テスト: `tests/test_safety_stop_transitions.py`(49 edge 全数 +
  guard 同一性の正負検証)

## Exclusive Actuation Gateway(実装済み: `realtime/exclusive_actuation_gateway.py`)

受入条件と実装対応(テスト: `tests/test_realtime_gateway.py`):
- (a) `release_latch` は manifest の operator lease と一致する場合のみ arbiter へ転送
- (b) `Directive.stop_state` は latch 種別 — 物理進行は stop_transitions の表で
  gateway が進める(guard 未成立は例外でなく現状態維持=整定待ちの正常系)
- (c) ACTIVE_HOLD 滞在中の STOP_NOW latch は滞在(自己遷移)で充足
- policy_hash × manifest 突合(stair mode の不一致は 1 frame も通さず latch —
  docs/08 §4.3。Branch L manifest は `not_applicable` を構築時点で拒否)
- 不正 payload の STOP_NOW は `submit_raw` が拒否をエスカレートして latch
- S/L/NAV 排他 + 4段 handshake(request → ack inactive → generation → enable)、
  generation は runtime 採番・append-only transition stream(docs/09 §5)

## 残 task(後続)

- 実機接続 adapter(Sport / LowCmd servo)への配線と heartbeat(実機 task)
- Mission FSM(EXPLORING 含む)と affordance validator
- StairModel grid 二層(policy_height_scan / safety_terrain_map)は Phase 2/4

## 移行メモ

- 既存 PoC(cockpit/server.py の WS command、UDP 43210/43211 の無 schema JSON、
  voice.py parse_intent)は削除せず、adapter を作って段階移行する(CLAUDE.md §10)。
- 最初の適用対象: UDP height_scan/cmd の 3 重実装 → CommandEnvelope/契約化。
- `voice.py` の substring parser → GoalSpec pipeline(Phase 3)。
