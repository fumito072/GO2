# GO2 自律階段プロジェクト：将来設計ドキュメント

調査日: 2026-07-13（JST）  
初回監査基準: `real_mac_GO2` / commit `a5535be`（現在の作業 HEAD とは限らない。実装開始時は [CLAUDE.md](CLAUDE.md) に従って再監査する）  
対象ゴール: 自然言語で階段前へ移動し、AirPods または有線マイク付きイヤホンの音声命令で約 10 cm × 4 段のブロック階段を上り、最上段で保持し、次の音声命令で下り切って再び保持する。

## 結論

最適解は、巨大な VLA やワールドモデルに全制御を任せることではない。次の階層型構成を採用する。

1. 文字入力と選択中の音声入力（AirPods／有線マイク）を、同じ型付き `GoalSpec` に変換する。
2. LLM/VLA は指示理解・対象参照・高レベルのスキル選択だけを行う。
3. 階段位置、接近姿勢、上端・下端は LiDAR/RGB(-D) の幾何検証で決める。
4. 平地移動は Sport 歩容＋地図ベース navigation で行う。階段昇降は Go2 X の純正階段歩容を正式 API から使えるか最初に評価し、性能/API が不足する場合だけローカル 50 Hz の知覚条件付き locomotion policy を使う。
5. 決定的な Mission FSM と独立 Safety Supervisor が、モデル出力より常に優先される。
6. 通常の「止まれ」は速度ゼロの**能動姿勢保持**であり、Damp（脱力）とは分離する。

既存コードには、Go2 I/O、LiDAR 標高マップ、上り用状態機械、Wave5 学習方策、UI が既にある。したがって全面刷新ではなく、次の順に穴を埋めるのが最短である。

`実機 API gate → 安全な保持と調停 → 共通 GoalSpec/音声入力 → 下降知覚 → 純正歩容/Wave5 の双方向評価 →（学習方策branchのみsim2sim）→ 段階的実機試験 → 未知環境ナビゲーション`

## 最初に固定すべきデモ仕様

以下を仕様として固定しない限り、成功率は比較できない。

- 階段: 固体ブロック、段高 `0.10 m ± 0.01 m`、4 段。
- 踏面奥行、全幅、表面材、エッジ丸みを実測し、階段 ID とともに記録する。
- 上端・下端に、全 4 足を安定して置ける平面を確保する。
- MVP の下降は、上りと同じ向きを保った**後退下降**を第一候補とする。上端に安全な旋回面が十分ある場合だけ、180°旋回後の前向き下降を比較する。
- 初期位置は既知の試験区画内、階段から 1〜5 m、初期 yaw は規定範囲内とする。
- 実機試験は監視者、吊り下げ/落下防止、マット、物理リモコンを必須とする。
- AirPods／有線マイク音声は操作インターフェースであり、物理 E-stop の代替ではない。MVP の基準入力は、切断・遅延を再現しやすい USB/USB-C 有線マイク付きイヤホンとする。

## 文書一覧

| 文書 | 内容 |
|---|---|
| [CLAUDE.md](CLAUDE.md) | RTX搭載PC上のClaude向け実装契約、安全境界、作業順、報告形式 |
| [01_CURRENT_STATE_AUDIT.md](01_CURRENT_STATE_AUDIT.md) | 現在のコード、実証済み範囲、重大な不足 |
| [02_TARGET_ARCHITECTURE.md](02_TARGET_ARCHITECTURE.md) | GoalSpec、Mission FSM、navigation、locomotion、安全調停 |
| [03_TECHNOLOGY_RESEARCH.md](03_TECHNOLOGY_RESEARCH.md) | VLA、NaVILA、world model、Code as Policies、RL/MPC の比較 |
| [04_ROADMAP.md](04_ROADMAP.md) | フェーズ、成果物、依存関係、Go/No-Go 条件、期間の目安 |
| [05_ASCENT_DESCENT_DESIGN.md](05_ASCENT_DESCENT_DESIGN.md) | 10 cm × 4 段の認識、整列、昇降、上端/下端判定 |
| [06_VOICE_AIRPODS.md](06_VOICE_AIRPODS.md) | AirPods／有線マイク経路、ASR、複合命令、STOP の意味分離 |
| [07_SIM_TRAINING_SIM2REAL.md](07_SIM_TRAINING_SIM2REAL.md) | RTX 5090、Isaac Lab、MuJoCo、domain randomization、実機移行 |
| [08_SAFETY_TEST_EVALUATION.md](08_SAFETY_TEST_EVALUATION.md) | hazard、試験ラダー、KPI、受入条件 |
| [09_DATA_AND_OPERATIONS.md](09_DATA_AND_OPERATIONS.md) | センサ記録、replay、モデル台帳、依存固定、運用 |
<<<<<<< HEAD
| [10_LINGBOT_MAP_INTEGRATION_ASSESSMENT.md](10_LINGBOT_MAP_INTEGRATION_ASSESSMENT.md) | LingBot-Map の統合可否、sidecar 構成、GPU・座標・安全評価 |
=======
| [10_EXPLORATION_MAPPING.md](10_EXPLORATION_MAPPING.md) | 自律探索とマップ構築(EXPLORE_AND_MAP)の設計(2026-07-15 目標再定義で追加) |
| [11_PROGRESS_AND_EDGE.md](11_PROGRESS_AND_EDGE.md) | 進捗サマリと現場エッジ機(RTX 3060 ノート)の要件・注意点 |
>>>>>>> origin/main
| [mermaid/README.md](mermaid/README.md) | 設計図のSVG/PNGプレビューとMermaid編集元 |

推奨の読み順は `01 → 02 → 04 → 05 → 08`。研究選択の根拠は `03`、音声と学習の実装詳細は `06` と `07` に分けた。LingBot-Map を camera-derived visual map 候補として評価する場合は `10` を参照する。

## 現在地の要約

| 能力 | 現状 | 判定 |
|---|---|---|
| Go2 Sport/LowCmd、LowState、カメラ | SDK ラッパあり、実機 M0 疎通ログあり | 再利用 |
| L1 点群、LiDAR odometry、標高マップ | 既存実装あり | 再利用して freshness/未知領域を補強 |
| 階段前への自然言語移動 | VLM の短い move/turn を逐次実行する PoC | 地図/goal pose/navigation が必要 |
| 上り | Sport/RL の状態機械あり | 実機成功の証跡は未確立 |
| 下り | `drop` として拒否 | 新規実装が必要 |
| Wave5 | 上下階段を含む地形で学習した 235→12 MLP | 下降を含む再評価が先、再学習は後 |
| 音声入力（AirPods／有線マイク） | OS の既定マイクなら偶然使える程度 | 明示的な device picker、gateway、切断監視が必要 |
| 複合音声命令 | 文中の「止まれ」で即 stop になる | GoalSpec parser へ置換 |
| 正常停止 | RL 終了後に Damp | 最優先で active hold に変更 |
| Branch L sim2sim / 共通raw sensor replay | リポジトリ内にharnessがない | sim2simはLのみ、raw replayはS/L共通で必須 |

## 成功の定義

研究デモとして最低限、以下がすべて成立して初めて「実現」と呼ぶ。

1. 文字で「階段の前まで行け」と指示し、衝突せず、規定した接近姿勢で停止する。
2. 選択したマイクで「その階段を登って、一番上まで行ったら止まれ」と指示し、全 4 足が上端平面に載った状態で能動保持する。
3. 保持中に「降りて、下り切ったら止まれ」と指示し、全 4 足が下端平面に載った状態で能動保持する。
4. `STOP_NOW`、手動 override、センサ途絶、通信途絶、モデル不正出力の全試験で安全側へ遷移する。
5. 各runがセンサデータ、設定、git commit、firmware、成否理由と、選択backendのevidence（Branch Lはpolicy/LowCmd hash、Branch Sはformal API contract/transaction）まで再現可能に記録される。

この文書群が対象とするのは、監視下の研究デモまでである。人の近くで無監視運用できる安全認証を意味しない。
