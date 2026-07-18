# GO2 自律階段昇降の技術調査と採用判断

> 調査スナップショット: **2026-07-13（JST）**  
> 対象: Unitree GO2 に対し、自然言語で階段前まで移動させ、AirPodsまたは有線マイク経由の音声命令で約 10 cm × 4 段の階段を昇降し、最上段・最下段で自律停止させるシステム  
> 本書の目的: 研究動向を列挙することではなく、どの技術を今回のクリティカルパスに置くか、何を後回しにするか、何を安全上採用しないかを決めること

## 0. この文書の読み方

本書では、論文・公式資料が示す事実と、このプロジェクト固有の設計判断を意識的に分離する。

- **[研究]**: 論文、研究プロジェクト、公式マニュアル、公式リポジトリが報告・規定している内容。報告値は原著者の条件下での値であり、本機での再現を保証しない。
- **[工学判断]**: 対象階段、GO2、開発期間、安全性、RTX 5090、保守性を考慮した本プロジェクトでの判断。
- **[要検証]**: 機体 SKU、ファームウェア、センサー構成、対象階段寸法など、実機を見なければ確定できない事項。

研究の証拠レベルは次のように扱う。

| レベル | 意味 | この案件での解釈 |
|---|---|---|
| E0 | アイデア、コード断片、未検証の実装 | 探索材料。採用根拠にはしない |
| E1 | シミュレーションでの結果 | 学習設計の参考。sim-to-real の証拠ではない |
| E2 | 別機種での実機結果 | 原理の有効性を支持するが、GO2 への移植が必要 |
| E3 | GO2 実機で近いタスクを実証 | 有力。ただし階段形状、速度、センサー、成功条件を照合する |
| E4 | 今回の GO2・階段・安全条件で反復試験に合格 | 製品判断に使える唯一の最終証拠 |

論文の動画で一度成功していることと、「音声命令から 4 段を 100 回安全に往復する」ことは別問題である。本プロジェクトでは E4 を作るために、E1〜E3 の知見を使う。

---

## 1. 結論

### 1.1 推奨する全体戦略

**一つの巨大な VLA / world model に知覚、言語、経路計画、脚制御をすべて学習させない。** 次の階層型システムを採用する。

```text
テキスト入力 ─┐
               ├─> 制約付き CommandIntent ─> アフォーダンス検査 ─> Behavior Tree
選択マイク音声 ─┘                                    │
                                                      ├─ NAVIGATE_TO_STAIR_APPROACH
                                                      │    ├─ SLAM / Nav2
                                                      │    └─ 階段検出・staging pose・整列
                                                      ├─ ASCEND_STAIRS
                                                      │    └─ 昇段専用 locomotion skill
                                                      ├─ DESCEND_STAIRS
                                                      │    └─ 降段専用 locomotion skill
                                                      └─ STOP_NOW

RGB-D / LiDAR / IMU / joint / foot force
        └─> 状態推定・局所標高マップ・StairModel ───────┘

独立 Safety Supervisor
        └─> watchdog / 姿勢・関節・温度・通信監視 / motion inhibit / 物理 E-stop
```

中心となる技術選択は以下である。

1. 階段前までの移動は、既知環境の MVP では **SLAM + Nav2 + 階段専用の最終整列器**を使う。
2. 階段認識は、VLM の一発判定ではなく、**RGB/RGB-D の意味候補 + 深度/LiDAR の幾何検証**で `StairModel` を生成する。
3. 昇降は、まずリモコンで利用できる純正階段 mode を安全治具下でscreeningし、それを外部から開始・中断できる正式APIがあるかUnitree/販売店へ確認する。公開 `SportClient.Move()` を階段専用歩容と同一視しない。性能とAPIの両方が合格する場合だけMVPで使う。
4. 純正動作が 10 cm × 4 段で反復性を満たさない場合、**privileged teacher → real-sensor student の perceptive RL**を主経路にする。昇段と降段は別スキルとして学習・評価する。
5. NaVILA のような VLA は、未知環境で「階段を探す」将来フェーズの上位意味ナビゲーション候補に限定する。脚関節を直接駆動させない。
6. full-body nonlinear MPC と world-model locomotion は有力な比較候補だが、初期 MVP のクリティカルパスには置かない。
7. Code as Policies の実行時コード生成は採用しない。言語は列挙型 intent と事前登録済みスキルに閉じ込める。
8. 低レベル制御は機体上で完結させる。Wi-Fi、選択マイク、音声gateway、LLM、VLM、外部 RTX 5090 の停止が転倒につながってはならない。

### 1.2 採否の要約

| 技術 | 今回の判定 | 使う場所 | 主な理由 |
|---|---|---|---|
| 階層型 skill architecture / Behavior Tree | **採用** | ミッション全体 | 状態、失敗、再試行、安全条件を明示できる |
| 制約付き language-to-skill | **採用** | テキスト・音声共通入口 | 誤認識を有限個の安全な intent に閉じ込められる |
| SLAM + Nav2 | **採用** | 階段近傍までの通常移動 | 成熟した決定論的基盤。階段への最終進入は別制御にする |
| RGB-D/LiDAR 幾何に基づく階段モデル | **採用** | 検出、整列、終端判定 | 10 cm 段差では意味認識だけでなく寸法・向きが必要 |
| 純正階段 gait | **条件付き採用** | 最速 MVP の昇降 | 最小開発工数。ただし正式な外部APIと連続階段・降段の E4 試験が必須 |
| Privileged perceptive RL | **条件付き採用・本命** | 純正 gait 不合格時の昇降 | 実センサーの欠損を扱い、複雑な接触を陽に解かず学習できる |
| NaVILA / VLA | **保留** | 未知環境での階段探索、上位指示 | GO2 実機の意味ナビ実績は魅力的だが、階段接触制御・安全終端の根拠ではない |
| VLM による open-vocabulary 検出 | **保留・補助のみ** | 階段候補の提案、データ注釈 | 汎化性は高いが、幾何・鮮度・停止安全を保証しない |
| Full-body NMPC / MPC | **保留** | 決定論的比較ベースライン、RL 失敗時 | 制約を明示できる一方、GO2 向けモデル・接触計画・WBC 移植コストが高い |
| World-model locomotion | **保留** | 第 2 世代の頑健化研究 | 将来性はあるが、今回の固定 4 段に対して依存関係と移植リスクが大きい |
| Code as Policies の実行時コード生成 | **不採用** | ― | 生成コードの任意実行、時間上限、失敗遷移、安全性を保証しにくい |
| End-to-end VLA → joint command | **不採用** | ― | 対応データ不足、検証不能、遅延・幻覚・OOD が転倒に直結 |
| Cloud / Wi-Fi 経由の低レベル制御 | **不採用** | ― | ジッタ、切断、輻輳を安全に閉じ込められない |
| 実機上のオンライン RL | **不採用** | ― | 転倒・破損リスクとサンプルコストが大きい |

---

## 2. 問題を三つに分解する

「自然言語で階段を登る」は一つの問題に見えるが、必要な時間スケールと失敗モードが異なる三つの問題である。

| 層 | 典型周期 | 入力 | 出力 | 主な失敗 |
|---|---:|---|---|---|
| 意味・ミッション層 | 0.2〜数 Hz | テキスト、音声、地図、ミッション状態 | 有限個の intent / skill 選択 | 誤認識、幻覚、対象の取り違え |
| ナビゲーション・地形モデル層 | 5〜20 Hz | RGB-D、LiDAR、自己位置 | 経路、staging pose、`StairModel` | 誤検出、遮蔽、座標ずれ、古い観測 |
| 接触・locomotion 層 | policy 50 Hz程度、LowCmd target stream既定500 Hz（200〜500 Hz候補）＋機体内servo | proprioception、局所地形、局所目標 | 12関節目標またはトルク | 足滑り、踏み外し、腹打ち、転倒 |

**[工学判断]** 高遅延でも意味を扱える VLA と、数十ミリ秒の遅れが転倒に直結する locomotion を同じ推論経路に置かない。各層の境界を小さな型付きデータにすることで、個別試験、ログ再生、置換、フォールバックが可能になる。

推奨 intent は最低限次の 4 種である。

```text
NAVIGATE_TO_STAIR_APPROACH
ASCEND_STAIRS
DESCEND_STAIRS
STOP_NOW
```

`ASCEND_STAIRS` は即座に関節指令へ変換しない。少なくとも以下をすべて満たした時だけ昇段 skill を arm する。

```text
stair_detected
AND stair_model_is_fresh
AND direction_is_up
AND aligned_to_stair
AND geometry_in_trained_envelope
AND localization_healthy
AND sensor_health_ok
AND battery_and_thermal_ok
AND exclusion_zone_clear
AND no_estop
```

これは SayCan の「言語上もっともらしいスキル」と「その場で実行可能なスキル」を分離する考え方に近いが、今回の実装では学習済み value だけに依存せず、幾何・状態・安全の決定論的 predicate を使う。

---

## 3. 比較評価

### 3.1 相対スコア

以下は論文の絶対性能ではなく、**今回の 10 cm × 4 段・GO2・音声指示**に対する相対評価である。5 が有利。安全性は「安全ケースを作りやすいか」、成熟度は「GO2 へ持ち込む工数が小さいか」を表す。

| 候補 | 接触適応 | 安全ケース | GO2 近接証拠 | データ効率 | 実装成熟度 | 未知環境の意味理解 | MVP 適合 |
|---|---:|---:|---:|---:|---:|---:|---:|
| 純正階段 gait + 外部 supervisor | 2 | 4 | 3〜4（物理能力、外部API未確認） | 5 | 3 | 1 | **5（正式API確認時のみ）** |
| Privileged perceptive RL | **5** | 3 | 3〜4 | 4（sim） | 3 | 1 | **4** |
| Full-body NMPC + elevation map | 4 | **5** | 2〜3 | **5** | 2 | 1 | 3 |
| World-model locomotion | 5 | 2 | 2〜3 | 3 | 2 | 1 | 2 |
| NaVILA 型 VLA + 既存 locomotion | 2 | 2 | **4**（意味ナビ） | 2 | 2〜3 | **5** | 2（将来 4） |
| End-to-end VLA → joints | 2 | 1 | 1 | 1 | 1 | 5 | **1** |
| Code as Policies → runtime API | 1 | 1 | 1 | 4 | 2 | 4 | **1** |

### 3.2 方式ごとの本質的トレードオフ

| 方式 | 何を学習／最適化するか | 強み | 弱み | 今回の位置づけ |
|---|---|---|---|---|
| Perceptive RL | センサー履歴と局所地形から関節目標を直接生成 | 接触、滑り、モデル誤差をまとめて扱う。大量 sim を RTX 5090 で生成可能 | 報酬設計、sim-to-real、失敗説明が難しい | 純正 gait が不合格なら主方式 |
| MPC / NMPC | 予測モデル上で接触・重心・足位置を逐次最適化 | 制約が明示的、挙動を解析しやすい | 高精度モデル、接触計画、WBC、リアルタイム solver が必要 | 比較ベースライン／代替経路 |
| World model | 潜在状態で未来を予測し、方策または計画に利用 | 部分観測や地形ダイナミクスを内在化できる可能性 | 学習・検証・移植がさらに複雑。既存 GO2 階段資産が少ない | 第 2 世代研究 |
| VLA | 画像・言語から意味的行動列や中間命令を生成 | 未知環境で対象を探す、指示を解釈する能力 | 幾何精度、リアルタイム性、安全保証が弱い | 上位探索のみ |
| Code as Policies | LLM がロボット API を呼ぶコードを生成 | 長い命令を組み立てやすい | 任意コード、安全境界、実行時間、再現性の管理が難しい | 実行時は不採用 |
| Language-to-skill | 言語を登録済みスキルと predicate に写像 | 入力自由度と実行安全を分離 | 語彙外命令を拒否する設計が必要 | 採用 |

---

## 4. Perceptive RL と privileged learning

### 4.1 研究が示していること

**[研究] Robust Perceptive Locomotion** は、シミュレーションでのみ得られる完全な地形・接触情報を使う教師方策から、ノイズを含む実センサー観測で動く生徒方策へ知識を移す枠組みを示した。知覚が壊れたときに proprioception へ依存する設計は、脚ロボットの実世界運用で重要である。  
一次情報: [Robust perceptive locomotion project](https://leggedrobotics.github.io/rl-perceptiveloco/)

**[研究] ANYmal Parkour** は、一つの万能方策ではなく、walking、climbing、jumping、crawling 等の複数 skill と、環境に応じてそれを選択する上位層を組み合わせた。複雑な都市障害物で階層化が有効であることを示す。  
一次情報: [ANYmal Parkour project](https://sites.google.com/leggedrobotics.com/agile-navigation)

**[研究] Extreme Parkour** は前方深度知覚を使う学習型 parkour locomotion を実機に展開し、公開コードも提供している。ただし公開基盤は旧 Isaac Gym 系であり、現行 GO2/Blackwell 環境へ無加工で導入できるものではない。  
一次情報: [Extreme Parkour project](https://extreme-parkour.github.io/)、[official code](https://github.com/chengxuxin/extreme-parkour)

**[研究] DreamWaQ++** は、point cloud と proprioception を異なる周期で融合し、知覚の信頼度に応じて proprioceptive fallback を使う方向を示す。プロジェクトページでは階段昇降を含む実機地形試験を報告し、point cloud 10 Hz、proprioception 200 Hz、policy 50 Hz、PD 200 Hz というマルチレート構成を明記している。これは「カメラの 1 フレームを 50 Hz 制御に直結しない」という重要な設計例である。  
一次情報: [DreamWaQ++ project](https://dreamwaqpp.github.io/)

**[研究] StairMaster** は2026年6月の新しい近接研究で、Unitree Go2による急勾配の中空階段上りを報告している。Cross-Attention、空間認識型recurrent memory、実depth artifactの高忠実度modeling、active-perception/edge penaltyは、blind spot、時間統合、sensor realismの設計に有用である。一方、対象は脚が隙間へ落ち得る中空階段の上りで、今回の固体10 cmブロックを往復し上端・下端で止まる課題とは成功条件が異なる。2026-07-13時点では新規arXiv v1なので、再現artifactと下降能力を確認せず主基盤へはしない。  
一次情報: [StairMaster, arXiv:2606.25765](https://arxiv.org/abs/2606.25765)

**[研究] Blind Stair Climbing** は、単純な速度指令だけでなく、局所的な位置目標を方策へ与えることの有効性を論じている。階段では「前進速度を出せ」より「階段座標系のこの姿勢へ到達せよ」の方が、段端との位相を制御しやすい。  
一次情報: [Blind Stair Climbing via RL, arXiv:2402.06143](https://arxiv.org/abs/2402.06143)

**[研究] RMA** は、短いセンサー履歴から環境・ダイナミクスの潜在パラメータをオンライン推定し、異なる摩擦や荷重へ適応する代表的枠組みである。今回そのまま採用するというより、teacher/student と adaptation history の基礎として参照する。  
一次情報: [Rapid Motor Adaptation, arXiv:2107.04034](https://arxiv.org/abs/2107.04034)

### 4.2 今回の採用形

**[工学判断]** 純正 gait が反復試験に不合格だった場合、次を基準構成とする。

#### Teacher

シミュレーションだけで取得できる以下を観測する。

- 真の局所 heightmap / terrain mesh
- 接触状態、接触力、足先位置、滑り
- 真の base pose / velocity
- 質量、重心、payload、摩擦、反発、motor strength、遅延
- 段の riser、tread、段数、エッジ位置
- 外乱

#### Student

実機で取得できる以下だけを観測する。

- IMU、関節角・角速度、直前 action の履歴
- 足裏力または接触推定（搭載 SKU で利用可能な場合）
- RGB-D / LiDAR から生成した robot-centric な局所標高マップまたは point-cloud latent
- `StairModel` 座標系での相対位置・相対 yaw・局所 goal
- sensor age、欠損 mask、perception confidence
- skill id（昇段／降段）または別々の policy

#### Action と周期

- policy 出力: 12 関節の目標角または nominal pose からの offset
- policy: 50 Hz を初期基準
- LowCmd position target stream: 既定500 Hz、200〜500 Hz候補をcompanionから送信し、`q/kp/kd`を受けた機体内servoがmotor制御を行う。代替rateは`T=1/f_tx` timing gate後だけとし、公開例の送信周期を機体内部PD周期の保証と読み替えない。companionで独自torque/PD loopを実装する案は別controllerとして検証する。
- skill 切替: 0.3〜0.5 秒程度の姿勢・action blending を初期値とし、実測で調整
- PC/RTX 5090 との無線切断時: 新しい skill を開始せず、機体側 supervisor が停止姿勢または安全な中断状態へ遷移

数値は研究結果の保証値ではなく、実装開始点である。最終周期は SDK、onboard compute、joint interface の測定で決める。

### 4.3 学習課題

**[工学判断]** 最初から一つの万能 locomotion policy にしない。

1. 平地での停止・姿勢保持
2. 6 cm の 1 段昇段
3. 8 cm、10 cm、12 cm の 1 段昇段
4. 2段、3段、4段、段数ランダム
5. yaw・横ずれを増やす
6. 材質、摩擦、深度欠損、遅延、外乱を増やす
7. 同じ順序を降段専用 policy で行う
8. 終端停止と skill transition を加える

降段は昇段の action を逆再生しても成立しない。前脚から見える段端、重心移動、支持多角形、足の着地衝撃、後脚の視界が異なるため、別 curriculum と別評価が必要である。初期は別 policy とし、十分なデータが得られた後に共有 backbone + skill-conditioned head を比較する。

### 4.4 Domain randomization の初期範囲

| 項目 | 初期学習範囲 | 主対象 |
|---|---:|---|
| riser | 6〜14 cm | 目標 10 cm の前後余裕 |
| tread | 18〜35 cm | 足置きと胴体干渉 |
| 段数 | 1〜8 | 4 段への過適合防止 |
| 初期 yaw | ±30°（curriculum で拡大） | 整列誤差 |
| 初期横ずれ | ±15 cm | 中央進入誤差 |
| 摩擦係数 | 0.3〜1.2 | 木、樹脂、ゴム等 |
| エッジ・riser 誤差 | ±1 cm | 製作公差・摩耗 |
| payload | 0〜3 kg を候補 | 追加センサー・計算機 |
| depth/LiDAR | hole、blur、dropout、外れ値 | 黒色面、反射、遮蔽 |
| perception latency | 0〜150 ms を候補 | USB、ROS、推論遅延 |
| actuator | motor strength、Kp/Kd、action delay | sim-to-real |
| body | mass、COM、慣性 | 機体差・搭載物 |

**[要検証]** この範囲を無条件に使うのではなく、実階段の寸法、床材、GO2 の system identification、センサー記録から分布を更新する。実測分布の周辺を厚くし、極端な範囲で学習が崩れる場合は curriculum を使う。

### 4.5 報酬と失敗判定

報酬には、前進速度だけでなく以下を含める。

- StairModel 座標系の局所 goal への進捗
- 適切な base height / pitch / roll
- 足先 clearance、tread 中央への着地
- slip、edge contact、胴体・膝の衝突ペナルティ
- torque、joint limit、joint speed、温度リスクの proxy
- action rate、jerk、energy
- 最終 landing での 4 足支持、速度ゼロ、安定保持

「上に進んだ距離」だけを報酬にすると、飛び乗り、膝打ち、段端への危険な接触が最適化され得る。成功動画より、失敗を厳密に定義した reward / termination が重要である。

---

## 5. MPC / model-based locomotion

### 5.1 研究が示していること

**[研究]** ETH Zürich 系の perceptive nonlinear MPC は、標高マップ、steppability、局所平面、障害物距離を最適化へ組み込み、ANYmal で階段を含む実機 locomotion を実証している。予測モデルと制約が明示されるため、足位置、関節限界、自己衝突などを設計者が追跡できる。  
一次情報: [Perceptive Locomotion through Nonlinear Model Predictive Control, ETH Research Collection](https://www.research-collection.ethz.ch/entities/publication/ed499b50-44d5-4bd7-95a3-438fbc64a940)

### 5.2 なぜ最初の主方式にしないか

**[工学判断]** MPC の理論的魅力と、GO2 へ安全に移植する工数は別である。full-body NMPC を成立させるには、少なくとも次が必要になる。

- GO2 の十分に正確な rigid-body / actuator model
- robust な状態推定
- terrain segmentation と候補 foothold
- contact sequence / gait schedule
- real-time solver と infeasible 時の挙動
- whole-body controller
- SDK の low-level interface、制御周期、watchdog との統合
- 実機での system identification と gain tuning

固定 4 段だけなら、十分にランダム化した perceptive RL または純正 gait の方が短い経路になりやすい。反対に、厳密な足置き制約、形状が既知の規格階段、説明可能性が最優先なら MPC の優位性が増す。

### 5.3 採用する MPC 的要素

full-body NMPC を採用しなくても、以下の model-based 要素は採用する。

- StairModel に基づく staging pose / approach trajectory
- base pose と足接触の可到達性チェック
- joint / velocity / torque envelope の独立監視
- 終端停止時の決定論的姿勢・速度条件
- RL と比較する deterministic crawl / IK / posture baseline

**採用ゲート:** perceptive RL の sim-to-real が繰り返し失敗し、その原因が知覚ではなく足位置制約・接触計画の不安定さにある場合、MPC/WBC 専任経路を開始する。

---

## 6. World model

### 6.1 用語の整理

world model は単なる 3D map ではない。一般には、観測・action から未来の潜在状態や報酬を予測し、方策学習または planning に利用するモデルを指す。本案件では次の三つを混同しない。

1. **幾何 world representation**: SLAM map、elevation map、StairModel。これは初期版から必要。
2. **locomotion dynamics world model**: 地形と action に対する身体の未来を潜在空間で予測。将来候補。
3. **foundation world-action model**: 多種ロボット・タスクを統合する大規模生成モデル。今回の critical path にはしない。

### 6.2 主な研究

**[研究] WMP (World Model-based Perceptive Locomotion)** は、潜在 world model を使った perceptive locomotion を提案し、プロジェクトでは Unitree A1 実機による 16 cm 級階段を含む結果を示す。公開コードは重要な一次資料だが、確認時点の実装は A1、旧 Isaac Gym Preview 3、CUDA 11.7 系を前提とし、GO2 と RTX 5090/現行 Isaac Lab への移植作業を要する。  
一次情報: [WMP project](https://wmp-loco.github.io/)、[official code](https://github.com/bytedance/WMP)

**[研究] TD-MPC2** は多数の連続制御タスクでスケールする model-based RL を示すが、GO2 の実機階段向け完成スタックではない。アルゴリズムの比較対象であって、直接導入物ではない。  
一次情報: [TD-MPC2 project](https://www.tdmpc2.com/)

**[研究] Unitree UniFoLM-WMA** は Unitree が公開する world-model-action 系資産である。しかし 2026-07-13 時点で公開されているデータ・例は主として Z1/G1 操作系であり、GO2 の階段昇降を解く公開チェックポイントとしては扱えない。  
一次情報: [Unitree UniFoLM-WMA official repository](https://github.com/unitreerobotics/unifolm-world-model-action)

### 6.3 今回の判断

**[工学判断] 保留。** 10 cm × 4 段という限定タスクでは、teacher/student + recurrent sensor history + local terrain latent で先に E4 を作る。world model は以下の条件で第 2 世代候補にする。

- 階段材質や形状の変化に対し、model-free policy の失敗モードが観測履歴だけでは解消しない
- 十分なシミュレーション・実機 rosbag が蓄積した
- world model の予測誤差を OOD 判定や安全停止に結び付けられる
- baseline と同じ条件で成功率、calibration、推論 deadline、故障時挙動を比較できる

world model を使う場合も、直接 safety authority にしない。予測 uncertainty が高い場合は「続行」ではなく「停止／拒否」に倒す。

単眼 RGB sequence から camera pose、depth、world points を推定する LingBot-Map は、上記 1 の幾何 world representation を補助する visual-map 候補である。SLAM/Nav2、metric LiDAR odometry、階段安全知覚の代替としては採用していない。repository 固有の適合性、scale ambiguity、sidecar 構成、導入 gate は [10_LINGBOT_MAP_INTEGRATION_ASSESSMENT.md](10_LINGBOT_MAP_INTEGRATION_ASSESSMENT.md) に分離した。

---

## 7. VLA / VLM と NaVILA

### 7.1 NaVILA が示すもの

**[研究] NaVILA** は Vision-Language-Action を長距離 navigation に使い、高水準の視覚・言語推論と、リアルタイムの低水準 navigation policy を階層化している。プロジェクトは GO2 実機での結果を公開しており、VLA が「廊下を進む」「対象の場所を探す」のような意味 navigation に使える有力な近接証拠である。上位 VLA が空間的な中間指示を生成し、低位 policy がそれを実行する構造は今回の将来フェーズと相性がよい。  
一次情報: [NaVILA project](https://navila-bot.github.io/)、[official code](https://github.com/AnjieCheng/NaVILA)

ただし、NaVILA の GO2 実証がそのまま「10 cm の段端へ正確に足を置き、4 段後に停止する」ことを意味しない。意味ナビゲーションと接触 locomotion は証拠を分けて評価する。

### 7.2 他の VLA

**[研究] QUART / QUART-Online** は四脚ロボットの視覚・言語・action を統合し、QUART-Online は高頻度実行を狙う。研究上は end-to-end 化の重要な比較対象だが、対象 GO2、対象階段、今回の停止安全条件に適合した公開 E4 はない。  
一次情報: [QUART project](https://quart-robot.github.io/)、[QUART-Online project](https://quart-online.github.io/)

**[研究] OpenVLA と π0/openpi** は VLA / generalist robot policy の代表的公開資産だが、公開成果・データの中心は manipulation であり、GO2 階段 locomotion の即用 policy ではない。  
一次情報: [OpenVLA official repository](https://github.com/openvla/openvla)、[Physical Intelligence openpi](https://github.com/Physical-Intelligence/openpi)

**[研究] VLM-Predictive Control (VLM-PC)** はGo1の実環境obstacle courseで、VLMが過去の試行を文脈に入れ、複数の登録済みlocomotion skillを先読み選択・再計画する構成を示している。これはVLMを低レベル制御器ではなくskill selectorへ置く根拠を補強する。ただし機体、skill set、階段停止条件が今回と異なるため、StairModelと安全predicateを省略できるわけではない。  
一次情報: [VLM-PC, arXiv:2407.02666](https://arxiv.org/abs/2407.02666)

### 7.3 今回 VLA に任せること、任せないこと

| 処理 | VLA/VLM の利用 | 条件 |
|---|---|---|
| 「階段へ行け」の意味解釈 | 可。ただし MVP は grammar/intent parser 優先 | 出力を `NAVIGATE_TO_STAIR_APPROACH` に制約 |
| 未知環境で階段らしい場所を探索 | 将来フェーズで有力 | geometry detector と Nav2 が全 waypoint を検査 |
| RGB 画像から階段候補を出す | 補助として可 | 深度/LiDAR 幾何で必ず再検証 |
| staging pose を cm 精度で確定 | 不可 | StairModel と幾何 controller が担当 |
| 昇段／降段の関節指令 | 不可 | pure onboard locomotion skill が担当 |
| 最上段・最下段の安全終端 | 不可 | 幾何、接触、姿勢、速度の決定論的合意 |
| 転倒防止・E-stop | 不可 | 独立 safety supervisor / physical E-stop |

**[工学判断]** MVP は「地図に semantic stair waypoint があり、指定された階段へ行く」構成から始める。これで navigation、stair localization、locomotion の三つを切り分けられる。その後、未知環境の能動探索で NaVILA を shadow mode に入れ、提案 waypoint と人間／幾何 detector の正解を比較する。shadow mode で誤提案と OOD を十分測定するまで、VLA に locomotion 開始権限を与えない。

### 7.4 VLM を階段検出に使う場合

Grounding DINO や SAM 2 は、open-vocabulary な候補領域生成、学習データの初期ラベル作成、遮蔽時のセグメンテーションに使える。  
一次情報: [Grounding DINO official repository](https://github.com/IDEA-Research/GroundingDINO)、[SAM 2 official repository](https://github.com/facebookresearch/sam2)

しかし、VLM の confidence は 10 cm 段差の測量誤差や現在時刻での安全性を表さない。候補領域に対し、平行な段鼻、水平面、周期的 riser、幅、上り／下り方向を RGB-D/LiDAR から fit し、幾何的 covariance と timestamp を持つ `StairModel` へ変換する。

---

## 8. Code as Policies と language-to-skill

### 8.1 研究上の位置づけ

**[研究] Code as Policies** は、LLM が利用可能な perception/control API を組み合わせるコードを生成し、自然言語の長いタスクを実行する考え方を示した。未知の命令を既存 primitive へ組み立てる柔軟性がある。  
一次情報: [Code as Policies project](https://code-as-policies.github.io/)

**[研究] SayCan** は、言語モデルが提案する skill の妥当性と、その状況で skill が成功する可能性・実行可能性を組み合わせる。言語だけで action を決めないという点が今回重要である。  
一次情報: [SayCan project](https://say-can.github.io/)

**[研究] Towards Reliable Code-as-Policies** は、動的・部分観測環境で元のCaPがgrounding不足や不完全codeに陥る問題に対し、symbolic verificationとinteractive validationを追加している。この研究自体が、生成codeを無検証で実機へ流す設計では足りないことを示す。今回採るのは、typed GoalSpec、allowlist skill、state predicate、schema/arbiterによる検証であり、runtime Python生成ではない。  
一次情報: [NeurIPS 2025 proceedings](https://papers.neurips.cc/paper_files/paper/2025/hash/6d13ce54347c65845614d01ced1dbe23-Abstract-Conference.html)

### 8.2 実行時 Code as Policies を不採用にする理由

階段上で生成 Python や任意 API 呼び出しを実行すると、次を形式的に制限しにくい。

- 呼び出してよい motion API と引数範囲
- ループ回数と実行期限
- 途中失敗時の rollback
- 同じ音声 packet の二重実行
- 古い perception を参照した action
- LLM 出力揺らぎによる再現性低下
- ネットワーク切断中の partial execution

**[工学判断]** 実行時に生成コードを走らせない。柔軟な言語理解を使う場合も、LLM 出力を JSON Schema で次のように制約し、検証後に Behavior Tree の既存 action node だけを起動する。

```json
{
  "schema_version": "1.0",
  "goal_id": "uuid",
  "intent": "ASCEND_STAIRS",
  "target": {"type": "stairs", "ref": "current"},
  "confirmation": {"required": true, "status": "CONFIRMED"}
}
```

引数に任意速度、任意関節角、任意 shell/code を許可しない。未知命令は「推測して動く」のではなく `UNSUPPORTED` として拒否し、ユーザーへ再発話を求める。

Code as Policies の考え方は、オフラインで Behavior Tree の候補を作る、テストケースを生成する、ログを説明する用途には使える。ただし生成物はレビュー・静的検査・シミュレーション試験を通してから登録済み skill に昇格させる。

---

## 9. 階段知覚と StairModel

### 9.1 なぜ専用モデルが必要か

通常の 2D navigation costmap では、階段は「通行不能な障害物」に見える。この扱いは階段近傍まで行く段階では正しい。階段へ乗り始める瞬間だけ、一般 navigation から専用 locomotion へ authority を移す必要がある。

推奨する `StairModel` の内容は以下である。

```text
frame_id / timestamp / age
stair_pose_xyz / stair_yaw
terrain_class: STAIRS | DROP | WALL | UNKNOWN
direction: UP | DOWN | UNKNOWN  # terrain_class=STAIRS のときだけ有効
riser_height + covariance
tread_depth + covariance
step_count + confidence
usable_width
edge polylines / local planes
top_landing_plane / bottom_landing_plane
staging_pose
geometry_envelope_valid
semantic_confidence / geometric_confidence
```

**[研究] Staircase Localization** は、RGB で候補を検出し、line segment と RGB-D を使って位置、yaw、上り／下りを推定するモジュール構成を示している。今回の semantic candidate → geometry fit に近い。  
一次情報: [Staircase Localization for Autonomous Exploration in Urban Environments, arXiv:2403.17330](https://arxiv.org/abs/2403.17330)

**[研究] 点群による stair detection/modeling** も、段平面・段鼻から寸法と姿勢を推定する方向を示す。  
一次情報: [Stair detection and modeling, arXiv:2405.01918](https://arxiv.org/abs/2405.01918)。論文が参照する project repository は 2026-07-13 の再確認時点で公開URLへ到達できなかったため、実装依存先には数えない。

**[研究] Elevation Mapping CuPy** は GPU 上のロボット中心標高マップ、semantic layer、traversability を提供する公開基盤である。ROS 世代や依存関係を確認し、必要なら algorithm を ROS 2 node へ移植する。  
一次情報: [Elevation Mapping CuPy official repository](https://github.com/leggedrobotics/elevation_mapping_cupy)

### 9.2 推奨 perception pipeline

1. RGB または RGB-D から階段候補 ROI を出す。
2. ROI の深度、LiDAR 点群を同一時刻・同一座標系へ変換する。
3. gravity で水平面候補を制約する。
4. 平行な段鼻、周期的 riser、tread 平面を RANSAC / line fitting で求める。
5. 上り／下り、幅、段高、踏面奥行、段数、landing を推定する。
6. 数フレーム追跡し、covariance と age を更新する。
7. 学習済み geometry envelope 外、観測が古い、相互矛盾が大きい場合は locomotion を arm しない。

**[工学判断]** staging pose は階段から 0.5〜0.8 m 程度を初期探索範囲とし、Nav2 はそこまでを担当する。その後、専用 align controller が 0.35〜0.5 m 程度まで接近し、横ずれ ±5 cm、yaw ±3〜5°を目標にする。これらは固定値ではなく、GO2 の前脚 reach、カメラ画角、階段 tread、policy の学習分布から確定する。

### 9.3 GO2 LiDAR を過信しない

**[研究]** Unitree L1 4D LiDAR の公式マニュアルは、視野角 360° × 90°、有効点数 21,600 points/s、水平走査 11 Hz、距離確度 ±2 cm 等を記載している。10 cm の riser に対し ±2 cm は無視できない比率である。  
一次情報: [Unitree 4D LiDAR L1 User Manual (official PDF)](https://oss-global-cdn.unitree.com/static/52b72f707b304d229d4321eea223738f.pdf)

L2 を使う場合は仕様と取り付け座標を別に確認する。  
一次情報: [Unitree 4D LiDAR L2 User Manual (official PDF)](https://oss-global-cdn.unitree.com/static/Unitree%204D%20LiDAR%20L2%20User%20Manual.pdf)

**[工学判断]** 単一 scan の段高を真値として使わない。RGB-D と LiDAR の時刻同期、複数 frame の temporal fusion、IMU gravity、外部 calibration、confidence/covariance を組み合わせる。段端を見るため、下向きに pitch した前方 RGB-D の追加を有力候補とする。降段を後ろ向きで行うなら rear/down sensor が別途必要である。

---

## 10. 最上段・最下段での停止

「4 段だから一定時間後に止まる」「画像に床が見えたから止まる」は不十分である。終端判定は、少なくとも以下の複数条件の合意にする。

- StairModel の予測総高と base の鉛直進捗が整合
- 進行方向に次の riser / descending edge がない
- 十分な面積の top / bottom landing plane が観測されている
- 4 足の支持または信頼できる接触推定
- base roll / pitch / height が安定範囲
- locomotion policy が stop transition を完了
- base 速度が 0.05 m/s 未満を 1 秒以上維持（初期 KPI）
- top/bottom stop zone 内にいる

条件は AND の一発判定ではなく、観測欠損を扱う状態機械にする。例として `APPROACHING_TERMINAL → VERIFYING_PLATEAU → SETTLING → STOPPED` と遷移し、検証不能なら低速化または停止する。

**[工学判断]** 段数だけに依存しない。対象が 3 段、5 段、1 段欠けでも、landing geometry と身体状態で止まれる構成にする。

---

## 11. 階段形状と降り方の重大な分岐

**[研究]** GO2 公式製品ページに記載される公称寸法は全長約 70 cm で、段差踏破能力のカタログ値も掲載されている。カタログ上の単一段差能力は、連続 4 段の昇降、停止余裕、滑りやすい材質での成功を保証しない。  
一次情報: [Unitree GO2 official product page](https://www.unitree.com/go2/)

**[工学判断]** 最上段 landing の奥行が設計を決める。

- landing に 180°旋回の余裕がある: 上で旋回し、前向きの降段専用 policy を使える。
- landing が狭い: 後ろ向きで降りる専用 policy と rear/down depth が必要。
- どちらも満たさない: 階段側に約 1 m 級の landing を追加するなど、実験設備を変更する。

この選択は学習開始前に決める。前方センサーしかない GO2 に、狭い landing から盲目的な後ろ向き降段を要求してはならない。

さらに、riser 10 cm だけでなく次を実測する。

- tread depth、usable width、段数
- top/bottom landing の奥行・幅
- 段鼻の丸み、張り出し、隙間
- 表面材質と摩擦、光沢、黒色／透明面
- 階段の固定強度とたわみ
- 周囲の壁、手すり、人の進入経路

---

## 12. GO2 ソフトウェア・ハードウェア基盤

### 12.1 SKU と公式 API を最初に確定する

**[研究]** 現行 GO2 製品表は Air / Pro / X / EDU を区別し、secondary development、compute、depth camera、foot-force sensor などの利用条件が異なる。リポジトリ上の対象は Go2 X だが、X は表上「部分対応」で EDU と同等とは限らないため、公式製品表と実機 serial/firmware を照合する必要がある。  
一次情報: [Unitree GO2 official product page](https://www.unitree.com/go2/)

**[工学判断]** 次を Phase 0 の停止条件にする。

- 正確な SKU、region、firmware version
- SDK で許可される high-level / low-level interface
- onboard compute と利用可能な port
- depth stream、LiDAR、joint state、IMU、foot force の実測 topic と rate
- low-level mode 利用時の保証、規約、安全機能

非公式な firmware unlock や reverse-engineered control path を主計画にしない。必要 API がない SKU なら、初期段階で EDU への変更または外付け compute / sensor 構成を判断する。

公式ソフトウェア一次情報:

- [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2)
- [Unitree ROS 2](https://github.com/unitreerobotics/unitree_ros2)
- [Unitree RL Lab](https://github.com/unitreerobotics/unitree_rl_lab)

Unitree RL Lab は GO2 を含む公式 sim-to-real 基盤で、Isaac Lab で学習し、MuJoCo で sim-to-sim を行ってから実機へ展開する経路を提供する。既存の速度追従タスクをそのまま「階段タスク」とみなさず、terrain、観測、報酬、termination、deploy config を専用化する。

### 12.2 RTX 5090 の使い所

RTX 5090 は以下に使う。

- Isaac Lab の数千並列環境で PPO / distillation
- synthetic depth / point cloud noise の生成
- perception model の fine-tuning
- rosbag のオフライン再生・評価
- VLM / ASR の shadow evaluation

機体の転倒回避に必要な低レベル loop は RTX 5090 に依存させない。学習 PC が落ちても onboard controller と safety supervisor は存続する。

**[研究]** 現行 Isaac Lab は Isaac Sim 上の公式 robot learning framework である。Blackwell GPU 対応は Isaac Sim、PyTorch、CUDA の組合せに依存するため、古い論文リポジトリの環境をそのまま混在させない。  
一次情報: [Isaac Lab documentation](https://isaac-sim.github.io/IsaacLab/develop/index.html)、[Isaac Lab release notes](https://isaac-sim.github.io/IsaacLab/main/source/refs/release_notes.html)

**[工学判断]** 次の環境を分離する。

1. `production-ros2`: Ubuntu 22.04 / ROS 2 Humble / Unitree SDK と実機接続
2. `training-current`: RTX 5090 対応の現行 Isaac Sim / Isaac Lab / PyTorch / CUDA
3. `legacy-research`: WMP や Extreme Parkour を読む／再現する固定 container
4. `sim2sim`: MuJoCo と deploy policy の決定論的回帰試験

論文コードの dependency を production 環境へ直接持ち込まない。再現した algorithm を、現行 Unitree RL Lab の task と deploy interface に移植する。

---

## 13. テキスト・音声入力インターフェース（有線／AirPods）

### 13.1 二系統ではなく一つの command bus

テキストと音声は入力 transport が違うだけで、ロボット側では同じ `CommandIntent` を発行する。

```text
Text UI  ── parser ──┐
                     ├─ intent schema ─ validation ─ affordance gate ─ BT
選択マイク ─ VAD/ASR ─┘
```

これにより、テキストで通った安全・状態試験を音声でも再利用できる。音声系の不具合を locomotion の不具合と分離できる。

### 13.2 音声 pipeline

推奨順序は次である。

1. push-to-talk または明示 wake word
2. VAD
3. ローカル ASR
4. constrained intent parser
5. confidence、現在状態、重複 `goal_id` の検査
6. 復唱または UI acknowledgment
7. affordance gate
8. skill start

一次情報:

- [OpenAI Whisper official repository](https://github.com/openai/whisper)
- [Silero VAD official repository](https://github.com/snakers4/silero-vad)

**[工学判断]** 有線マイクの抜去／接触不良、AirPodsのBluetooth切断、OSのaudio route切替、環境騒音、ASR hallucinationは通常故障として扱う。選択device消失時は内蔵マイクへ暗黙fallbackせず、新規commandを受理しない。古いpacketの再送で同じ階段を二度登らないようUUIDとmission stateを検査する。

`STOP` 音声 command は便利な operational stop だが、物理 E-stop の代替ではない。転倒直前にcable／USB／Bluetooth、ASR、ネットワーク、CPUが正常である保証はない。

---

## 14. 安全アーキテクチャ

安全 supervisor は LLM、VLM、ASR、Nav2、RL policy と独立させる。少なくとも以下を監視する。

- roll / pitch / angular velocity
- base height、足接触、滑り推定
- joint position / velocity / torque limit
- motor temperature、battery
- sensor heartbeat / timestamp / time sync
- locomotion command deadline
- network heartbeat
- StairModel age / confidence
- exclusion zone と人検出
- physical E-stop

ここでいうphysical E-stopは要求機能であり、Go2 Xに安全ratedな装置が既設という意味ではない。公開資料では確認できないため、Unitree/販売店と安全担当者による同定・書面確認をPhase 0 blockerとする。リモコンDamp、電源断、音声STOPを同一視しない。

試験順序はbackendで分ける。Branch L（学習LowCmd）はIsaac Lab → MuJoCo sim-to-sim → 吊り治具 → 5 cm単段 → 1段 → 2段 → 3段 → 4段とする。Branch S（純正Sport）はvendor evidence/API contract → 吊り下げ／平地のauthority・停止試験 → 5 cm単段 → 1段 → 2段 → 3段 → 4段とし、vendor simulatorが正式提供される場合だけSILを追加する。どちらもハーネスが機体を受け止めた試行は「怪我がなかった成功」ではなくlocomotionの失敗として数える。

**不採用事項:**

- 学習PCからWi-Fi経由でcompanion-side PD/torque loopや高周期LowCmd target streamを閉ループ制御
- LLM/VLA の「大丈夫」という出力で safety predicate を上書き
- 実機試行中のオンライン policy update
- 失敗ログなしの手動再試行
- いきなり 4 段・無拘束・後ろ向き降段

---

## 15. 技術採用ゲート

### Gate A: 物理・API 可行性

合格条件:

- SKU、SDK、センサー topic、制御 rate が確定
- 対象階段の全寸法・摩擦・landing が計測済み
- 旋回後の前向き降段か、rear sensor 付き後ろ向き降段か決定
- 物理 E-stop、吊り治具、立入制限、ログが機能

### Gate B: 純正 gait の採否

Phase 0は、リモコン階段modeの存在・方向・選択方法を資料と吊り下げ／平地telemetryで確認し、正式な外部APIを照合するところまでに限定する。独立停止とtimeoutを実証する前は段差へ接地させない。通常の `SportClient.Move()` は階段skillの証拠にしない。安全/記録/知覚の基盤と低段Gateに合格したPhase 6以降に、ハーネス下で5 cm単段、1段、2段、3段、4段を段階評価する。固定条件で昇段・降段それぞれ **30 回連続**、転倒、ハーネス捕捉、胴体・膝のhard contact、関節限界なしを初期合格基準とする。

- 合格: MVP は純正 gait + 外部知覚・整列・停止へ進む。
- 不合格: 失敗を分類し、perceptive RL へ進む。
- API 不足: hardware/SKU 構成を再決定する。

### Gate C: 階段知覚

初期目標:

- 検出 precision 0.98 以上、recall 0.95 以上（0.5〜3 m、対象条件）
- riser MAE 1 cm 以下
- stair pose p95 位置誤差 3 cm 以下、yaw 誤差 3°以下
- approach 100 回中 98 回以上成功
- approach 終端の横ずれ ±5 cm、yaw ±3°
- 階段ではない棚、縞模様、段ボール、影に対する false activation 0

### Gate D: 学習 policy

初期目標:

- domain-randomized Isaac Lab 10,000 episode で 99% 以上
- 学習分布外 geometry を 99% 以上拒否または停止
- MuJoCo sim-to-sim で 98% 以上、Isaac からの低下 2 percentage point 以内
- policy 50 Hz deadline miss 0
- stale command / sensor 時に watchdog が必ず motion inhibit

### Gate E: 実機 locomotion

段階試験後の初期目標:

- 固定 4 段: 昇段 100 回中 95 回以上、降段 100 回中 95 回以上
- 最上段／最下段停止: 100 回中 99 回以上
- 平地／既に安定stanceではSTOP後1秒以内に水平速度0.03 m/s未満。階段途中は、一律1秒停止ではなく署名済みphase別`max_safe_boundary_time_s/distance_m/step_count`以内（最終値は `08_SAFETY_TEST_EVALUATION.md` をcanonicalとする）
- fall、ハーネス捕捉、hard body contact、limit violation は成功に数えない
- 無拘束前に段階試験 300 回で fall / harness capture 0

300 回ゼロ失敗でも「安全が証明された」のではない。いわゆる rule of three では、未知の失敗率の 95% 上限がおおよそ 1% という意味に過ぎない。運用条件を限定し、監視を残す。

### Gate F: 言語・音声

初期目標:

- テキスト intent 正解率 99.5% 以上
- 音声 intent: 静音 98% 以上、想定騒音 95% 以上
- speech end から acknowledgment まで p95 1.5 秒未満
- 1,000 件の負例音声で false actuation 0
- AirPods 切断、ASR timeout、重複 packet、状態不整合で motion start 0

### End-to-end 合格

「階段へ行け」→ staging → 整列 → 音声昇段 → top stop → 音声降段 → bottom stop を、手動介入なしで 100 mission 中 95 mission 以上成功させる。全missionでsensor、intent、state transition、safety eventと、選択backendのartifact/evidence（Branch Lはpolicy/LowCmd、Branch Sはformal API contract/transaction/state）を時刻同期して保存する。

---

## 16. 実装順と研究テーマの解禁条件

以下は技術依存の概念順であり、期間・番号を含むcanonical roadmapは `04_ROADMAP.md` とする。

| フェーズ | 主技術 | 研究テーマを増やさない理由／解禁条件 |
|---|---|---|
| 0. Hardware audit | SDK、センサー、階段、治具 | 物理・API 不明のまま学習しない |
| 1. Flat navigation | ROS 2、SLAM、Nav2、logging | 平地の自己位置と watchdog を先に安定化 |
| 2. StairModel | RGB-D/LiDAR geometry | locomotion と切り離して精度を測る |
| 3. Stock gait benchmark | Unitree Sport | 最小構成で task が解けるかを最初に確認 |
| 4. Perceptive RL | Isaac Lab、teacher/student、MuJoCo | stock 不合格時のみ主経路化 |
| 5. Text → voice | schema、BT、Whisper/VAD | text で安全遷移が通った後に音声を追加 |
| 6. Known-map mission | semantic waypoint | end-to-end の責任分界を確立 |
| 7. NaVILA shadow | VLA、active search | 既知地図版が E4、幾何 validator が稼働後 |
| 8. World model / NMPC comparison | WMP、MPC | baseline の失敗データと比較指標が揃った後 |

**[工学判断]** 「最新研究を使う」ことを目的にしない。より単純な技術で E4 を満たすなら、それが最適解である。研究モデルは、baseline がどこで失敗し、追加複雑性がどの KPI を改善するかを説明できる場合だけ導入する。

---

## 17. 今回の技術選択を覆す条件

現在の判断は固定ではない。次の証拠が得られた場合は更新する。

### Perceptive RL から MPC へ寄せる条件

- target staircase の幾何が完全既知・固定である
- RL の失敗が足置き制約違反に集中し、domain randomization で改善しない
- real-time NMPC + WBC が GO2 low-level interface で deadline を満たす
- 同条件で MPC が成功率・限界遵守・説明可能性を明確に上回る

### Perceptive RL から world model へ寄せる条件

- 長い観測履歴・recurrent student でも partial observability が支配的
- WMP 系を GO2/Isaac Lab へ移植し、baseline と同じ推論予算で改善を再現
- uncertainty calibration が安全な拒否判定に有効

### Nav2 から NaVILA を主経路へ寄せる条件

- 既知地図ではなく未知の建物で、言語対象探索が要件化
- shadow mode で waypoint 提案の precision と OOD refusal が目標を満たす
- 全提案を幾何・collision checker が検証し、VLA 故障が低レベル motion に伝播しない

### 純正 gait のまま進める条件

- 対象 4 段の昇降、停止、材質変化で E4 を満たす
- 外部 StairModel から安全に開始・中断できる API がある
- 失敗時の姿勢・停止挙動が許容可能

---

## 18. 一次情報一覧

### Unitree / 実装基盤

- [Unitree GO2 official product page](https://www.unitree.com/go2/)
- [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2)
- [Unitree ROS 2](https://github.com/unitreerobotics/unitree_ros2)
- [Unitree RL Lab](https://github.com/unitreerobotics/unitree_rl_lab)
- [Unitree 4D LiDAR L1 User Manual](https://oss-global-cdn.unitree.com/static/52b72f707b304d229d4321eea223738f.pdf)
- [Unitree 4D LiDAR L2 User Manual](https://oss-global-cdn.unitree.com/static/Unitree%204D%20LiDAR%20L2%20User%20Manual.pdf)
- [Isaac Lab documentation](https://isaac-sim.github.io/IsaacLab/develop/index.html)
- [Isaac Lab release notes](https://isaac-sim.github.io/IsaacLab/main/source/refs/release_notes.html)
- [Nav2 Behavior Trees documentation](https://docs.nav2.org/behavior_trees/index.html)

### Perceptive / terrain-aware locomotion

- [Robust Perceptive Locomotion](https://leggedrobotics.github.io/rl-perceptiveloco/)
- [ANYmal Parkour](https://sites.google.com/leggedrobotics.com/agile-navigation)
- [Extreme Parkour](https://extreme-parkour.github.io/)
- [Extreme Parkour official code](https://github.com/chengxuxin/extreme-parkour)
- [DreamWaQ++](https://dreamwaqpp.github.io/)
- [StairMaster](https://arxiv.org/abs/2606.25765)
- [Blind Stair Climbing via RL](https://arxiv.org/abs/2402.06143)
- [Rapid Motor Adaptation](https://arxiv.org/abs/2107.04034)

### MPC / world model

- [Perceptive Locomotion through Nonlinear MPC](https://www.research-collection.ethz.ch/entities/publication/ed499b50-44d5-4bd7-95a3-438fbc64a940)
- [WMP: World Model-based Perceptive Locomotion](https://wmp-loco.github.io/)
- [WMP official code](https://github.com/bytedance/WMP)
- [TD-MPC2](https://www.tdmpc2.com/)
- [Unitree UniFoLM-WMA](https://github.com/unitreerobotics/unifolm-world-model-action)

### VLA / VLM / language-to-skill

- [NaVILA](https://navila-bot.github.io/)
- [NaVILA official code](https://github.com/AnjieCheng/NaVILA)
- [VLM-Predictive Control](https://arxiv.org/abs/2407.02666)
- [QUART](https://quart-robot.github.io/)
- [QUART-Online](https://quart-online.github.io/)
- [OpenVLA](https://github.com/openvla/openvla)
- [Physical Intelligence openpi](https://github.com/Physical-Intelligence/openpi)
- [Grounding DINO](https://github.com/IDEA-Research/GroundingDINO)
- [SAM 2](https://github.com/facebookresearch/sam2)
- [Code as Policies](https://code-as-policies.github.io/)
- [Towards Reliable Code-as-Policies](https://papers.neurips.cc/paper_files/paper/2025/hash/6d13ce54347c65845614d01ced1dbe23-Abstract-Conference.html)
- [SayCan](https://say-can.github.io/)

### 階段知覚・音声

- [Staircase Localization for Autonomous Exploration in Urban Environments](https://arxiv.org/abs/2403.17330)
- [Stair detection and modeling](https://arxiv.org/abs/2405.01918)
- [Elevation Mapping CuPy](https://github.com/leggedrobotics/elevation_mapping_cupy)
- [OpenAI Whisper](https://github.com/openai/whisper)
- [Silero VAD](https://github.com/snakers4/silero-vad)

---

## 19. スナップショット上の注意

1. 本調査は **2026-07-13** 時点で公開されている一次情報のスナップショットである。GitHub の branch、release、ライセンス、checkpoint、CUDA/Isaac 互換性は導入時に再確認する。
2. 研究プロジェクトページの動画・報告値は、各研究者の機体、センサー、階段、成功定義による。今回の E4 を代替しない。
3. GO2 の機能は SKU、地域、firmware、保証条件で異なり得る。公式販売元と実機で確認する。
4. 2026 年の GO2 階段論文であっても、simulation-only または ascent-only なら実機往復の証拠として扱わない。例: [Training and Simulation of Quadrupedal Robot in Adaptive Stair Climbing for Indoor Firefighting: An End-to-End Reinforcement Learning Approach, arXiv:2602.03087](https://arxiv.org/abs/2602.03087)。設計参考にはなるが、降段・停止・実機 sim-to-real は別途必要である。
5. 本書の成功率・誤差・試行回数はプロジェクトの初期 engineering gate であり、引用した研究の主張ではない。実測に基づいて厳しくすることはあっても、安全根拠なしに緩めない。

---

## 20. 最終推奨

このプロジェクトの最適解は、現時点では「最新の一モデル」ではなく、以下の責任分界である。

- **言語は意図を決める。** 動作の物理量を決めない。
- **Nav2 / StairModel はどこへ行くか、階段が実行可能かを決める。** 足の接触を直接制御しない。
- **純正 gait または perceptive RL は身体を動かす。** ミッション意味や音声を解釈しない。
- **Safety Supervisor は全層を止められる。** どの学習モデルにも上書きさせない。
- **VLA / world model は baseline を置き換える証拠が得られてから追加する。** 研究上新しいことを、工学上必要なことと混同しない。

最短経路は、純正階段 gait の正式API確認と段階的な実機ベンチマーク、既知地図の StairModel + Nav2、制約付きテキスト intent、device非依存の音声入力/ASR の順である。純正 gait のAPIまたは性能が不足すると判明した時点で、RTX 5090 と Unitree RL Lab / Isaac Lab を使う privileged perceptive RL を投入する。未知環境の自然言語探索は、その土台が E4 に達した後に NaVILA 型上位層を追加する。この順序なら、将来性を捨てずに、失敗の原因を測定できる安全な MVP を最短で作れる。
