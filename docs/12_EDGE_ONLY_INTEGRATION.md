# 12. エッジ単体統合設計 — 音声・自然言語・自律探索マッピングの実機接続

作成: 2026-07-15。ユーザー決定「RTX 5090 は使わない。エッジノート(RTX 3060 Laptop 6GB)
単体で目標①②③(音声操作 / 自然言語操作 / 自律探索マッピング)をすべて動かす」を受けた
統合設計。docs/02 の invariant、docs/08 の Gate、docs/10 の探索設計に完全従属する。
本書は安全 gate を一切緩めない。

## 1. 前提ハードウェア(実測 2026-07-15)

| 項目 | 値 | 用途 |
|---|---|---|
| GPU | RTX 3060 Laptop 6GB (driver 580.159.03) | **ASR専用**(faster-whisper int8, ctranslate2 CUDA確認済み) |
| CPU | 16 core | parser / FSM / arbiter / gateway / occupancy / frontier / 追従制御(全て純ロジック) |
| RAM | 24GB | — |
| torch | 2.13.0+cpu | ランタイムでは不使用(目標①②③にtorch推論は無い) |
| NIC | enp46s0 有線直結 192.168.123.x | Go2 DDS。通常LAN/WANへbridgeしない(docs/02 §3) |

docs/11 §5 のとおり、6GB にローカル VLM は同居させない。narration が必要な場合のみ
Anthropic API(安全ループ外・fail-closed)。**目標①②③の実行経路にクラウドは入らない。**

## 2. 配置(プロセス/モジュール)

MVP は cockpit サーバ内の autonomy task として実装する(`mission.py` / `stair_task.py`
と同じ流儀)。理由: ARMゲート・0.5sコマンド途絶watchdog・WS切断即停止・速度クランプ・
deploy_log の既存安全機構をそのまま継承できる。独立 Safety Supervisor プロセス化
(invariant 5 の完全形)は Gate 3 以降の課題として残す(§8)。

```text
ブラウザPTT音声 ──┐
テキスト入力 ─────┤
                  ▼
   voice_gateway.intent_parser(限定文法・決定的)     … 目標①②
                  ▼ GoalProposal(要確認) / STOP_NOW(即時)
   操作者確認(復唱 readback + 確認ボタン/「はい」)
                  ▼ GoalSpec(CONFIRMED)
   mission.executive FSM(EXPLORING)                  … 目標③
                  ▼ 確認済みGoalSpecを一度だけ委譲
   MissionAgent + ExplorationController
                  ▼ reachable frontier + FREE-only A* path
   CollisionGuard(同一snapshotを10Hz再検査)
                  ▼ generation付き0.3s command hold
   cockpit Bridge.set_cmd → Sport Move(既存watchdog/クランプ/ARMゲート)
```

- invariant 1: ASR/UI/explorer は GoalSpec/goal pose の提案のみ。actuator owner にならない。
- invariant 2: 探索actuatorはMissionAgentの一系統だけ。手動非ゼロ入力は自律系を
  中断・zero化してからoverrideする。
- invariant 8: 「止まって」「完了」は Damp ではなく **Controlled Stop → ACTIVE_HOLD**(balance stand)。
- invariant 9: unknown ≠ free。点群/pose 途絶は fail-closed(新規移動を発行しない→停止)。

## 3. 知覚: 実LiDAR → GlobalOccupancyMap

入力は cockpit が既に受けている `rt/utlidar/cloud_deskewed`(odom系xyz)と
`rt/utlidar/robot_odom` pose。本番writerは`GlobalOccupancyMap.integrate_point_cloud()`、
`perception/cloud_projector.py`は分類の単体試験/補助アダプタとして使う:

1. 点群を robot 足元基準の z 帯で分類:
   - 床帯 `z ∈ [z_floor−0.08, z_floor+0.10]` → 通過セル(観測された床)
   - 障害物帯 `z ∈ (z_floor+0.10, z_floor+0.60]` → OCCUPIED 終端
   - 頭上帯(>0.60)は無視(Go2は潜れる)
   - 床より深い `z < z_floor−0.15` → **drop hazard = OCCUPIED 扱い**(docs/10 §5)
2. robot セルから各終端セルへ Bresenham raycast し、通過セルを FREE 更新。
   終端が障害物帯なら OCCUPIED 更新。**raycast なしで FREE を置かない。**
3. セル鮮度(cell age)を更新。古い FREE は planner 側で信頼度を落とす。
4. 全て純関数(点群, pose, 時刻注入)で E0 synthetic test 可能にする。

本番runnerは20 m四方・0.10 m/cellの`GlobalOccupancyMap`をodom frameで作り、
`artifacts/maps/<map_id>/`へ保存する。分解能はmap artifact自身に記録し、UIもその値を
送信する。odom drift は単一連続 run 内地図として許容(E1で評価)。

## 4. 音声(目標①) — 既存UIを壊さない差分追加

- 既存 PTT(押して話す)→ faster-whisper の枠組みは維持。モデルは
  **CUDA int8 の small から開始し、`--whisper` フラグで medium-int8 へ変更可**。
  CUDA 初期化失敗時は現行どおり CPU へ自動フォールバック(起動ログに明示)。
- 認識テキストの解釈を二段にする:
  1. **`voice_gateway.intent_parser`(正)** — 限定文法。STOP_NOW は確認なし即時。
     移動を伴う goal(EXPLORE_AND_MAP 等)は **readback + 確認**(docs/06 §8.3/8.4)。
     否定・引用・質問・「止まらず〜」は不実行。
  2. parser が「命令でない」と判定した発話のみ、既存のルールベース
     テレオペ解釈(前進3秒等)へフォールバック(現行機能の互換維持)。
- 「止まって」は経路に関係なく即時 Controlled Stop(既存実装維持+FSM abort 接続)。
- テキスト入力欄(目標②)も**同一の** parser → 確認 → GoalSpec 経路を通す。
  音声とテキストで別の意味論を作らない(docs/02 §4.2)。

## 5. 探索(目標③) — EXPLORING の実機接続

- `FrontierExplorer`はrobotと連結したFREE上だけでinflate済みA* pathを提案する。
  cell visit回数、直近goal、失敗goal cooldownを保持し、同じ場所への再訪を減点する。
- `ExplorationController`はpathの曲がり角をショートカットせず、ヨー整列後に
  **vx ≤ 0.20 m/s, |wz| ≤ 0.45 rad/s**で追従する。map revision、stale FREE、
  3秒の無進捗でそのtickを停止して再計画する。
- `CollisionGuard`は生の3D点群で停止距離内のswept corridor/rotation footprintを
  毎tick検査し、障害物・drop・未観測回廊・点群不足ならゼロcommandにする。
- 完了: frontier 枯渇 → `EXPLORATION_COMPLETE` → ACTIVE_HOLD(自動で次を始めない)。
  地図と robot_trace を `artifacts/maps/` へ保存し、`home` waypoint を登録。
- `home`はartifact上の参照点として保存するだけで、現在のLIVE runnerはwaypoint帰還を
  実行しない。探索中の自動登坂も別GoalSpec/確認/authority handoffが整うまでOFF。
- 到達不能 frontier は除外リストへ(「観測ゼロ」「到達不能」≠「完了」の区別は
  explorer 実装済みの意味論を踏襲)。

## 6. 安全(既存ゲート全維持+追加)

| 条件 | 動作 |
|---|---|
| DISARM / 停止 / DAMP / Space / 「止まって」 / WS切断 | 即 Controlled Stop → FSM abort(自動再開なし) |
| \|roll\|>0.5 / \|pitch\|>0.7 rad | 即中断(stair_task と同値) |
| lowstate 途絶 / pose・有効点群途絶(>0.60s) / frame不一致 / 点密度不足 | fail-closed: 即zero → 継続2sでabort |
| command hold expiry(0.30s) | bridge watchdogを待たず明示zero |
| NaN/Inf/変換不能command、bool以外のARM | commandをzero化し、ARMはOFF。手動入力なら自律runも中断 |
| 探索全体タイムアウト(既定180s) | Controlled Stop → ACTIVE_HOLD |
| 1 goal 移動距離 | A* path累積≤2m。各tickをgeneration付きkeeper/guardianへ通す |
| 開始条件 | ARM 中のみ + 確認済み GoalSpec のみ。起動時/再接続時の自動開始なし |

## 7. 検証計画(docs/10 §6 準拠)

| 段階 | 内容 | 本作業での扱い |
|---|---|---|
| E0 | synthetic 点群で cloud_projector / follower の unit test | **実施(自動テスト)** |
| E2 | `cockpit.server --mock --host 127.0.0.1` で text→confirm→explore→map→STOP のE2E | **実施(HTTP/WebSocket実通信)** |
| E3 | 実機平地 LIVE 探索 | **ユーザー実施**。前提: Gate 3(停止経路検証)。本書§8参照 |

## 8. 実機試験(E3)前にユーザーへ明示する残条件

本実装が完了しても、docs/08/10 上の LIVE 前提は変わらない:

1. **Gate 3 相当の停止確認**を最初に行う(最低限: ARM→停止/Space/「止まって」/DISARM/WS切断
   の各経路で 1.0s 以内静止を平地で確認してから探索を開始する)。
2. physical E-stop は未同定のまま(リモコンDamp・電源断は未検証)。監視者と物理停止手段を手元に。
3. 初回は狭い既知区画・低速(vx上限0.20)・人の立入なしで行う。
4. 探索は Sport モード(純正歩容)のみ。階段・段差は対象外(検出したら OCCUPIED=回避)。

## 9. 本書で追加/変更するモジュール

| ファイル | 種別 | 内容 |
|---|---|---|
| `perception/global_map.py` | 変更 | hit/miss同一scan統合、FREE鮮度、動体解除、永続hazard、保存/読込 |
| `perception/cloud_projector.py` | 新規 | 点群+pose → hit/miss/hazard分類(純ロジック/補助adapter) |
| `navigation/waypoint_follower.py` | 新規 | offline/互換用goal pose追従。LIVE探索runnerでは未使用 |
| `cockpit/explore_task.py` | 新規 | parser→proposal/confirm/STOP/UI facade。actuator threadは持たない |
| `cockpit/mission.py` | 追記 | 確認済みGoalSpec→10Hz単一runner、CollisionGuard、LowState/姿勢/TTL gate |
| `cockpit/voice.py` | 差分 | 認識テキストを intent_parser 優先に二段化 |
| `cockpit/server.py` | 差分 | WS message(explore系)+バイナリ frame type=3(occupancy grid)配信 |
| `cockpit/static/` | 差分 | 探索マップパネル(canvas)+開始/確認/中断UI(既存パネル無変更) |
| `tests/test_cloud_projector.py` ほか | 新規 | E0 テスト群 |
