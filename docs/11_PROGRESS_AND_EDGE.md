# 11. 進捗サマリとエッジデバイス(現場ノートPC)要件

最終更新: 2026-07-15(branch `claude/phase0-gate0-contracts` @ `8272d89`)

## 1. プロジェクト目標(2026-07-15 再定義)

1. **音声で操作できること**
2. **自然言語で操作できること**
3. **自律で空間を探索してマップを構築すること**

階段 Mission(docs/01〜09 の当初 MVP)は優先度低下。vendor 質問は送付しない
(ユーザー決定 → Branch S は候補除外、詳細は `phase0/api_gate_report.md`)。
SKU は Go2 X(ユーザー申告、serial/firmware 実機確認は未実施)。

## 2. 完了したもの(すべて offline・robot 非接続、テスト 221/221 PASS)

### 2.1 監査と Phase 0

- 静的監査+現状分類(`reports/2026-07-15_static_audit_and_classification.md`)。
  docs/CLAUDE.md §3.1 の危険事項 **9/9 をコード行番号まで実在確認**
- platform inventory: RTX 5090 32GB / CUDA 13.2 / RAM 128GB / WSL2 Ubuntu-24.04。
  policy artifact 4点の SHA-256 が docs/01 記録値と一致(VERIFIED)
- Phase 0 テンプレート(`phase0/`): hardware manifest / stair registry /
  API gate report(質問票保存)/ go2-runtime 依存 freeze

### 2.2 安全基盤(Gate 0 素材)

| 部品 | 内容 |
|---|---|
| `contracts/` v1.1 | GoalSpec(EXPLORE_AND_MAP / NAVIGATE_TO_WAYPOINT 含む)、StairModel、CommandEnvelope、StopState 7状態。strict parser・検証迂回経路なし |
| `mission/command_arbiter.py` | priority 8段調停、expiry→Controlled Stop(ゼロ推測禁止)、latch(自動復帰なし)、STOP_NOW 無条件受理、clock jump fail-closed |
| `safety/stop_transitions.py` | 停止状態遷移表(guard tuple、未定義遷移拒否、49 edge 全数テスト) |
| `realtime/exclusive_actuation_gateway.py` | NAV/STAIR 排他、4段 handshake+generation、policy hash gate、lease 認証つき latch 解除 |
| `mission/executive.py` | docs/02 §5 の Mission FSM + EXPLORING 拡張、affordance validation 内蔵 |

### 2.3 目標①②③ の実装

- `voice_gateway/intent_parser.py`: 限定 grammar parser(docs/06 §8.3)。
  「〜たら止まれ」= completion 条件、「止まれ」単独 = STOP_NOW、否定/引用/質問/
  禁止形は実行しない、「止まらずに登れ」は拒否。**docs/06 §12.2 の固定
  regression 文を全件テスト化**(旧 substring parser の誤動作の再発防止)
- `perception/global_map.py`: 2D occupancy map(unknown≠free を型で強制、
  cell 鮮度、hazard 投影、保存/読込)
- `navigation/frontier_explorer.py`: frontier 探索の決定的 baseline
  (「観測ゼロ」「到達不能」を「完了」と区別 — 誤完了防止)
- `demo/explore_e2e.py`: **offline E2E 成功**。
  「部屋を探索してマップを作って」→ parser → 確認 → FSM → 探索 → arbiter →
  gateway → 合成2部屋世界で **ドアを通って両部屋を地図化(83 steps)** →
  EXPLORATION_COMPLETE → ACTIVE_HOLD。「止まれ」途中注入で即停止・自動再開なし。
  実行: `python -X utf8 -m demo.explore_e2e`

### 2.4 環境

- WSL2 Ubuntu-24.04 に `~/go2-runtime` venv(torch 2.13.0+cpu、
  構築: `scripts/wsl_setup_go2_runtime.sh`、lock: `phase0/go2_runtime_freeze.txt`)

## 3. 未完了(実機・現場・判断が必要)

| 項目 | 状態 |
|---|---|
| Gate 3(平地・停止試験) | **すべての実機自律走行の前提**。監視者+停止経路検証が必要 |
| physical E-stop 同定 | 未同定(階段 LIVE は NO-GO のまま。平地でも停止手段の検証は必須) |
| unitree_sdk2py + cyclonedds 導入 | robot 接続方針の確認後(エッジ機の Ubuntu 上) |
| 実機 LiDAR 録画 → replay 評価(E1) | robot 接続時に採取 |
| faster-whisper ベンチ(モデル選定) | 5090 で実施予定 → エッジ用モデル確定 |
| cockpit UI へのマップ可視化 | 未着手(既存 UI 差分を保全して追加する) |

## 4. 計算機の役割分担

- **現場エッジ = ノートPC(RTX 3060 6GB)** — Go2 直結で持ち出す機体
- **自宅開発機 = RTX 5090 デスクトップ** — ベンチ/コーパス評価/replay 解析/学習

実行時の計算負荷は ASR 以外ほぼ CPU(parser/FSM/arbiter/gateway/occupancy map/
frontier/elevation はすべて numpy+純ロジック)。**5090 を現場に持ち出す必要はない**。
docs/08 §4.1 のとおり、遠隔・クラウド推論を安全ループに入れない(安全系は
エッジ機ローカルで完結させる)。

## 5. エッジノート(RTX 3060 6GB)の注意点

1. **OS は native Ubuntu を強く推奨**。unitree_sdk2py + cyclonedds(DDS)は
   WSL2 だとマルチキャスト/NIC 直結が不安定になりがち。デュアルブートか
   Ubuntu 専用化が安全(WSL2 を使う場合は mirrored networking の実測必須)。
2. **有線 LAN ポート必須**(Go2 直結 192.168.123.x)。なければ USB-Ethernet
   アダプタ。Go2 の DDS subnet を通常 LAN/WAN へ bridge しない(docs/02 §3)。
3. **faster-whisper は medium-int8(≈1.5GB VRAM)から**。large-v3-int8(≈3GB)も
   単独なら載るが、docs/06 のレイテンシ目標(p95 650ms)は 5090 前提の値なので
   3060 では実測して選定する。選定ベンチは 5090 側で実施。
4. **6GB にローカル VLM を同居させない**。状況説明(narration)は Anthropic API
   (現行設計どおり)。ASR と VRAM を取り合うと双方のレイテンシが悪化する。
5. **電源と熱**: 探索デモは連続 GPU/CPU 負荷になるため現場では AC 給電推奨。
   バッテリー駆動時は電源プランで CPU クロックが落ち、制御周期に影響し得る。
6. Wave5 policy(階段を復活させる場合)は 1.1MB MLP で CPU 推論可 —
   3060 でも問題ない。ただし LowCmd の送信周期検証(Gate 2L)は GPU と無関係に
   リアルタイム性(独立 publisher プロセス)の課題であり、エッジ機の CPU/OS
   設定(電源プラン、USB/NIC レイテンシ)の方が支配的。
7. **エッジ機での再現手順**: リポジトリ clone → Ubuntu なら
   `python3 -m venv ~/go2-runtime && pip install numpy torch --index-url
   https://download.pytorch.org/whl/cpu`(scripts/wsl_setup_go2_runtime.sh 参照)
   → `python -m unittest discover -s tests` が 221/221 PASS すること →
   `python -m demo.explore_e2e` で E2E 確認(いずれも robot 不要)。

## 6. 次の作業候補

1. edge プロファイル(3060 向け設定: whisper=medium-int8、VLM=API)の config 化
2. 5090 での faster-whisper モデル別ベンチ(日本語コーパス)
3. cockpit UI へのマップ可視化 layer(mock server 統合、E2 の完成)
4. 実機接続の準備(エッジ機 Ubuntu セットアップ、sdk2py ビルド — 接続は承認後)
