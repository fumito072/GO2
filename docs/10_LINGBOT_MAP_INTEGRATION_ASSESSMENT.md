# LingBot-Map の GO2 Cockpit 統合評価

- 調査日: 2026-07-16（JST）
- 対象 upstream: `Robbyant/lingbot-map` / commit [`7ff6f3ed0913d4d326f8f13bbb429c4ffc0195c2`](https://github.com/Robbyant/lingbot-map/tree/7ff6f3ed0913d4d326f8f13bbb429c4ffc0195c2)
- 対象 GO2 branch: `fix/lidar-spatial-point-cloud` / 調査時 HEAD `275083a`
- 採用状態: **CANDIDATE / NOT ADOPTED（候補・未採用）**

## 0. この文書の位置付け

この文書は、LingBot-Map を現在の GO2 Cockpit に追加できるかを判断するための技術評価であり、採用決定や実装完了を示すものではない。

既存設計との優先順位は次のとおり。

1. [08_SAFETY_TEST_EVALUATION.md](08_SAFETY_TEST_EVALUATION.md) の安全条件
2. [02_TARGET_ARCHITECTURE.md](02_TARGET_ARCHITECTURE.md) の authority、座標系、process 境界
3. [04_ROADMAP.md](04_ROADMAP.md) の段階的な導入順
4. この文書の LingBot-Map 固有評価

矛盾がある場合は上位文書を優先する。LingBot-Map を採用するまでは、既存の LiDAR、LiDAR odometry、elevation map、`StairModel` の責任分界を変更しない。

## 1. 結論

**GO2 Cockpit への追加は可能。** ただし、現時点では「既存の LiDAR SLAM や安全知覚を置き換える部品」ではなく、前面 RGB カメラから得た 3D 復元結果を表示する**独立した visual-map sidecar**として追加するのが妥当である。

推奨する最初の用途は次の二つ。

- Cockpit の `AI MAP / VISUAL MAP` レイヤーとして、カメラ由来の色付き点群と推定軌跡を read-only 表示する。
- 記録済みの GO2 カメラ、LiDAR odometry、LiDAR 点群を offline replay し、単眼推定の scale、drift、reset、遅延を測る。

最初から任せてはいけない用途は次のとおり。

- 階段、崖、壁の安全判定
- `StairModel` の単独生成
- collision/drop guardian の置換
- GO2 の actuator command、Sport command、LowCmd の publish
- LingBot-Map の推定 pose を無検証で robot `odom` として使用
- 生成した点群だけを根拠にした自律 navigation

理由は、upstream の main branch が現状では画像フォルダ／動画をまとめて処理するデモ中心であり、ROS/DDS や live camera service を提供していないこと、単眼 3D 復元には metric scale の曖昧性があること、長距離で pose が崩れる可能性を upstream 自身が明記していることにある。

## 2. LingBot-Map で確認できた能力

### 2.1 入出力

LingBot-Map は monocular RGB image sequence を入力し、主に次の値を出力する。

- camera pose encoding
- dense depth
- depth confidence
- world-space points
- world-point confidence

実装上の出力は [`gct_base.py`](https://github.com/Robbyant/lingbot-map/blob/7ff6f3ed0913d4d326f8f13bbb429c4ffc0195c2/lingbot_map/models/gct_base.py#L184-L285)、表示処理は [`point_cloud_viewer.py`](https://github.com/Robbyant/lingbot-map/blob/7ff6f3ed0913d4d326f8f13bbb429c4ffc0195c2/lingbot_map/vis/point_cloud_viewer.py#L157-L223) で確認できる。GO2 側から見れば、RGB 付き 3D point cloud と camera trajectory の候補になる。

論文は 518 × 378 入力で 10,000 frame を超える streaming reconstruction と約 20 FPS を報告している。ただし、この数値を GO2、同時稼働モデル、対象 GPU でそのまま保証するものではない。GO2 実データでの benchmark が必要である。

一次情報:

- [LingBot-Map paper, arXiv:2604.14141](https://arxiv.org/abs/2604.14141)
- [Official repository](https://github.com/Robbyant/lingbot-map)

### 2.2 現在の upstream demo

公式 [`demo.py`](https://github.com/Robbyant/lingbot-map/blob/7ff6f3ed0913d4d326f8f13bbb429c4ffc0195c2/demo.py#L56-L124) は image folder または video file を入力する。video は frame に展開され、sequence の inference 後に Viser viewer を開く構成である。

main branch で確認できないもの:

- Unitree DDS subscriber
- ROS/ROS 2 node
- REST、WebSocket、gRPC の map service
- live GO2 camera adapter
- timestamp、camera calibration、body extrinsics の契約
- LiDAR/IMU fusion
- loop closure、pose graph、occupancy/costmap の完成した navigation stack

したがって、リポジトリを Cockpit process に import するだけでは live map にはならない。入力 adapter、逐次 inference worker、map filter、Cockpit 転送 API を GO2 側で設計する必要がある。

### 2.3 「streaming」の意味

内部実装は最初の既定 8 frame をまとめて処理し、その後は KV cache を使って frame-by-frame に進められる。該当実装は [`gct_stream.py`](https://github.com/Robbyant/lingbot-map/blob/7ff6f3ed0913d4d326f8f13bbb429c4ffc0195c2/lingbot_map/models/gct_stream.py#L350-L520) にある。

一方、公開されている `inference_streaming(images)` は sequence 入力を受け、cache を初期化し、最終的に結果を連結して返す API である。GO2 の live camera を無期限に push する service API ではない。そのため live 統合では、upstream model の逐次 forward を呼ぶ bounded worker wrapper が必要になる。

### 2.4 scale と長距離 drift

単眼 reconstruction の world frame は、LiDAR odometry の meter 単位 `odom` と自動的には一致しない。論文評価も Sim(3) alignment を用いるため、出力を metric odometry と仮定してはいけない。

また upstream README は、非常に長い sequence では pose が collapse する可能性に触れている。運用時は少なくとも次が必要になる。

- session generation と reset の明示
- keyframe/window policy
- confidence 低下の検出
- LiDAR odometry または独立 ground truth による scale/rotation/translation 評価
- map reset 後に古い点と新しい点を混ぜない仕組み

## 3. 現在の GO2 Cockpit 側の基盤

### 3.1 既にあるデータ経路

現在の Cockpit には次の基盤がある。

| 能力 | 現状 |
|---|---|
| 前面カメラ | Unitree `VideoClient` の JPEG を `/video` へ配信 |
| LiDAR 点群 | `rt/utlidar/cloud_deskewed` を `odom` frame として購読 |
| robot pose | `rt/utlidar/robot_odom` を優先し、`(x,y,z,yaw)` を保持 |
| browser 3D | WebSocket binary kind `1` で XYZ 点群を最大 8,000 点送信 |
| height map | WebSocket binary kind `2` で rolling elevation map を送信 |
| local map 表示 | Three.js で最大 120,000 点、2.5 cm voxel、半径 10 m を蓄積 |

主な実装:

- [common/config.py](../common/config.py)
- [cockpit/server.py](../cockpit/server.py)
- [cockpit/static/app.js](../cockpit/static/app.js)

現在の UI が持つ点群は、LiDAR の短い局所 scan を `odom` 上へ蓄積した browser 内の volatile map である。既存 height map は 8 m、5 cm grid の rolling max-z 表現であり、persistent SLAM map ではない。

### 3.2 LingBot-Map 統合前に不足している情報

現在の camera 経路は JPEG の表示を目的にしており、map integration に必要な次の契約がない。

- camera frame ごとの monotonic timestamp
- capture timestamp と受信 timestamp の区別
- camera intrinsics と distortion
- camera optical frame から body frame への extrinsics
- image と `robot_odom` の同期
- full 6DoF pose
- dropped frame、queue depth、input age

LingBot-Map を LiDAR と比較・整列する前に、Phase 2 の記録・較正基盤としてこれらを追加する必要がある。

### 3.3 既存 LiDAR frame 保護

現在の Cockpit server は `cloud_deskewed` が robot pose と同じ `odom` 系かを検査し、`map` や sensor-local frame の点群を変換なしでは受け付けない。この挙動は正しい。

LingBot-Map の arbitrary-scale points を既存 binary kind `1` に混ぜてはいけない。visual map は別 message type、別 layer、別 frame/generation として扱う。

## 4. 適合性評価

| 評価項目 | 適合度 | 判断 |
|---|---:|---|
| GO2 前面 RGB の利用 | 高 | image adapter を用意すれば入力可能 |
| Cockpit への 3D 表示 | 高 | Three.js と既存 WebSocket を拡張可能 |
| 色付き point cloud | 高 | world points と元 RGB を filter/downsample して表示可能 |
| live 無期限 stream | 中 | model 内部は逐次化可能だが custom worker が必要 |
| metric map | 低〜中 | 単眼 scale を外部基準で合わせる必要がある |
| `odom` 代替 | 低 | drift/reset と metric scale の validation が不足 |
| loop closure / persistent SLAM | 低 | 現在の main に完成した機能として確認できない |
| occupancy / Nav2 costmap | 低 | 点群から traversability/occupancy へ変換する別工程が必要 |
| 階段安全判定 | 低 | LiDAR/elevation の独立 guardian を置換できない |
| RTX 5090 standalone PoC | 中〜高 | 実行可能性は高いが GO2 data で VRAM/FPS 測定が必要 |
| Cockpit process への直接同居 | 低 | dependency、CUDA、OOM、latency の fault containment が悪い |

## 5. 統合案

### Option A: upstream viewer だけで offline 確認

記録した camera video を `demo.py` に渡し、Viser で結果を見る。

利点:

- 最短で GO2 映像との相性を確認できる。
- Cockpit code と safety path を変更しない。

制約:

- live Cockpit map ではない。
- Viser の既定 port `8080` は Cockpit の既定 port と競合するため、`8081` などへ変更する。
- timestamp、LiDAR alignment、長時間 reset の評価は別途必要。

目安: 0.5〜2 日。

### Option B: read-only live visual-map sidecar

GO2 camera を latest-only queue で LingBot worker へ渡し、confidence filter と downsample 後の色付き点群を Cockpit の別 layer へ送る。

利点:

- ユーザーが期待する「カメラから作った空間の雰囲気」を Cockpit 内で確認できる。
- LiDAR radar と visual map を切り替え、または重ねて比較できる。
- worker が停止しても control path を維持できる。

制約:

- custom live wrapper が必要。
- 最初は LingBot session frame の可視化であり、metric navigation map ではない。

目安: 1〜2 週間。

### Option C: LiDAR odometry と整列した visual map

camera/body calibration と同期済み odometry を使い、LingBot session を Sim(3) で `odom` と整列する。scale、drift、reset を監視し、色付き visual map を生成する。

利点:

- LiDAR layer と camera-derived layer を同じ画面で比較できる。
- semantic/color 情報を map inspection に使える。

制約:

- calibration、time synchronization、online alignment、reset recovery が必要。
- LingBot の誤差を測る independent reference が必要。
- align 後も safety source にはしない。

目安: 3〜6 週間。

### Option D: navigation / control へ利用

現時点では非推奨。採用には Option C に加え、occupancy/traversability、global/local planner、local collision/drop guardian、localization confidence、recovery、fault injection が必要になる。

LingBot-Map は map proposal であり、それだけで navigation system にはならない。将来利用する場合も velocity command は既存 authority、ARM gate、watchdog、clamp を通し、独立 LiDAR guardian を残す。

## 6. 推奨アーキテクチャ

```text
Go2 VideoClient
    |
    v
Sensor Adapter
  - capture/receive timestamp
  - camera intrinsics
  - camera->body extrinsics
  - rectify/resize
  - dropped-frame metrics
    |
    v
Latest-only bounded queue
  - backlogを処理しない
  - stale frameを破棄
    |
    v
LingBot worker                         Existing L1 path
  - separate process/env/container     cloud_deskewed + robot_odom
  - RTX 5090 inference                 elevation + StairModel
  - cache/reset generation             collision/drop guardian
    |                                  control/safety authority
    v                                           |
confidence/voxel/keyframe filter                |
    |                                           |
    v                                           v
Read-only Visual Map API ---------------> Cockpit WebSocket/UI
  - visual points                         - LIDAR layer
  - camera trajectory                     - VISUAL MAP layer
  - alignment/status                      - stale/reset/confidence
```

### 6.1 process と dependency の境界

LingBot-Map は Cockpit の Python process/venv へ直接入れず、別 conda environment または container、別 process で動かす。

理由:

- 推奨環境が Python 3.10、PyTorch 2.8.0、torchvision 0.23.0、CUDA 12.8 を中心としている。
- FlashInfer が推奨され、利用できない場合は SDPA fallback になる。
- model load、CUDA OOM、worker crash が Cockpit、watchdog、command ownership に波及してはいけない。
- upstream `pyproject.toml` だけでは torch/torchvision/numpy の完全な再現 lock にならない。

一次情報:

- [Installation instructions](https://github.com/Robbyant/lingbot-map/blob/7ff6f3ed0913d4d326f8f13bbb429c4ffc0195c2/README.md#L91-L130)
- [pyproject.toml](https://github.com/Robbyant/lingbot-map/blob/7ff6f3ed0913d4d326f8f13bbb429c4ffc0195c2/pyproject.toml)

### 6.2 queue と backpressure

live map は「全 camera frame を必ず処理する」方式にしない。inference が入力 FPS に追い付かない場合、古い frame の backlog を処理すると UI が数秒遅れになり、現在の map に見えてしまう。

初期 contract:

- queue size は 1〜2。
- worker が busy の間は中間 frame を drop し、最新 frame を残す。
- capture age、inference age、publish age を別々に表示する。
- age が閾値を超えたら visual map を `STALE` 表示にする。
- reset/collapse 検出時は `generation_id` を増やし、過去 generation と混ぜない。

### 6.3 座標系

推奨 frame contract:

| frame | 意味 |
|---|---|
| `camera_optical` | 較正済みの実カメラ frame |
| `base` | GO2 body frame |
| `odom` | LiDAR odometry による連続的な meter 単位 frame。local control の正 |
| `lingbot_session` | LingBot-Map が生成する arbitrary-scale session frame |
| `visual_map` | Sim(3) alignment 後の表示用 frame。未整列時は作らない |
| `map` | 将来の global localization frame。reset/jump を許容し、`T_map_odom` を明示 |

重要事項:

- scale 未確定の Sim(3) は通常の SE(3) TF として publish しない。
- `S_odom_lingbot` の scale、rotation、translation、covariance、timestamp を独立した alignment record として持つ。
- `visual_map` points は alignment record 適用後に生成する。
- control は連続する `odom` を使い、global map correction や LingBot reset で飛ばさない。
- unaligned points を既存 LiDAR `odom` cloud に混ぜない。

### 6.4 Cockpit message の提案

以下は**設計案であり、まだ実装されていない**。

| message | 内容 |
|---|---|
| binary kind `3` | visual-map chunk: generation、timestamp、XYZ、RGB、confidence |
| JSON `visual_map_status` | state、input age、FPS、latency、VRAM、point count、alignment quality、reset reason |
| JSON `visual_camera_pose` | LingBot camera trajectory と alignment 状態 |

UI では既存の LiDAR layer と分離し、次を表示する。

- `LIDAR`
- `VISUAL MAP`
- `OVERLAY`
- `UNALIGNED / ALIGNING / ALIGNED / STALE / RESET`
- data age、confidence threshold、generation

既存 kind `1` と `2` の意味は変えない。

### 6.5 点群量と転送量

518 × 378 は 1 frame 約 196,000 pixel であり、dense XYZ、RGB、confidence を高頻度でそのまま browser へ送ると数十 MB/s 規模になり得る。既存 Cockpit の 8,000 points/frame より大幅に多い。

初期値の提案:

- confidence filter
- pixel stride
- 2〜5 cm voxel filter
- keyframe だけを map へ追加
- 1 update あたり 1,000〜8,000 points
- map chunk と level-of-detail
- browser 側は visual map 用の独立上限を設定
- robot 周辺を高密度、遠方を低密度にする

点の追加数、GPU buffer、WebSocket bandwidth、browser frame time を計測し、固定値は benchmark 後に決める。

### 6.6 GPU と checkpoint

公開 checkpoint は Hugging Face 上で各約 4.63 GB であり、論文 Table 7 は window size 64 の構成で約 13.28 GB の memory と 20.29 FPS を報告している。model weight の file size と runtime VRAM は同じではなく、CUDA context、attention backend、input buffer、viewer、他モデルの使用量を加える必要がある。

RTX 5090 上で LingBot-Map 単独の PoC は現実的と考えられるが、他の VLM/VLA/training job と同時常駐させる判断は benchmark 後に行う。GPU OOM 時は visual map だけを停止し、Cockpit と制御を継続させる。

checkpoint:

- [Hugging Face: robbyant/lingbot-map](https://huggingface.co/robbyant/lingbot-map/tree/main)

## 7. 安全境界

### 7.1 初期導入時の authority

LingBot worker と visual-map service は read-only とし、次を禁止する。

- DDS command publish
- Sport/LowCmd call
- Mission FSM の直接遷移
- ARM/DISARM の変更
- `climbable=true` や `path_clear=true` の単独決定
- LiDAR guardian の無効化

authoritative source は引き続き次のとおり。

- local geometry: L1 LiDAR、elevation、`StairModel`
- continuous local pose: LiDAR `robot_odom`
- drop/wall/collision: 独立 local guardian
- command: Exclusive Actuation Gateway と Safety Supervisor

### 7.2 故障時の期待動作

| 故障 | 必須動作 |
|---|---|
| LingBot process kill | visual map を `OFFLINE` にし、control は継続 |
| CUDA OOM | worker を隔離停止し、Cockpit/control/watchdog は継続 |
| camera stale | visual map を `STALE` にし、古い map を現在値として表示しない |
| frame drop | rate と drop count を表示し、backlog を作らない |
| low confidence | point を抑制し、空白を「free space」と解釈しない |
| scale drift | alignment を解除し、overlay/navigation 利用を禁止 |
| pose collapse/reset | generation を更新し、古い点群と結合しない |
| WebSocket overload | visual layer を間引き、LiDAR/status/control traffic を優先 |
| calibration mismatch | overlay を無効化し、再較正を要求 |

「LingBot-Map が落ちたので robot を Damp にする」という結合も避ける。visualization-only 段階では worker failure は mission safety fault ではなく、optional perception unavailable として扱う。

## 8. 推奨する段階的導入

### Step 1: offline replay

変更前に、GO2 から次を同一 timeline で記録する。

- camera JPEG/raw frame と capture/receive timestamp
- LiDAR `cloud_deskewed`
- LiDAR `robot_odom`
- IMU/body attitude
- camera intrinsics/extrinsics と calibration hash
- run manifest、git commit、firmware

代表 scene:

- 廊下、壁、角、扉
- 対象の 10 cm × 4 段
- 上端・下端
- 低 texture の床・白壁
- motion blur、照度変化
- 人や物体が動く scene
- revisit と長い直進

### Step 2: viewer-only PoC

記録 video を公式 demo へ入力し、GO2 data で次を確認する。

- model load
- frame size と aspect
- point confidence
- 空間形状が視認できるか
- pose collapse の発生位置
- FPS、p50/p95 latency、peak VRAM
- 5分、30分、60分相当 sequence

Viser は Cockpit と port が競合しない設定にする。

### Step 3: live shadow mode

separate worker と latest-only queue を作り、Cockpit には `VISUAL MAP` layer だけを追加する。LiDAR overlay はまだ行わず、LingBot session frame と camera trajectory、health を表示する。

この段階の合格条件:

- process kill/OOM が Cockpit/control に波及しない。
- stale/reset が UI に正しく出る。
- backlog により「数秒前の map」が現在値として表示されない。
- browser の LiDAR FPS と control operation が悪化しない。

### Step 4: calibration と LiDAR alignment

camera intrinsics/extrinsics、timestamp を固定し、recorded replay で `S_odom_lingbot` を推定する。

測る値:

- scale error
- translation drift
- rotation/yaw drift
- trajectory endpoint error
- revisit inconsistency
- alignment covariance
- reset frequency
- confidence と実誤差の calibration

評価対象 LiDAR/LIO を唯一の ground truth にせず、可能なら mocap、AprilTag、測量治具、既知寸法を併用する。

### Step 5: mission UI での advisory 利用

Step 4 が合格した場合だけ、visual map 上の semantic annotation や operator inspection を mission UI に追加する。それでも LingBot 単独で階段 skill を開始しない。

### Step 6: navigation 研究

navigation 利用は別の採用判断とする。最低でも次が必要。

- metric localization と map revision
- occupancy/traversability conversion
- independent collision/drop guardian
- planner と recovery
- uncertainty gate
- dynamic obstacle handling
- map reset 中の停止/再局在
- [08_SAFETY_TEST_EVALUATION.md](08_SAFETY_TEST_EVALUATION.md) に沿う fault injection

## 9. Go / No-Go 条件

### 9.1 viewer / shadow mode の Go

- 公式 checkpoint と固定 commit で再現実行できる。
- GO2 camera dataset の主要 scene で 3D 形状を operator が識別できる。
- 30〜60分相当の処理で crash、unbounded memory growth、無検出 reset がない。
- p95 latency と input age を常時取得できる。
- worker kill/OOM で control、watchdog、LiDAR layer が止まらない。
- model、checkpoint、container、calibration の hash が run に残る。

### 9.2 LiDAR overlay の Go

- intrinsics/extrinsics/time offset が version 管理される。
- scale、rotation、translation drift の閾値が test data から定義される。
- alignment 不良を検出したら overlay を自動解除できる。
- reset generation が混在しない。
- 同じ場所の LiDAR と visual points の誤差を scene 別に報告できる。

### 9.3 navigation 利用の No-Go

次の一つでも該当する間は、navigation/control source として採用しない。

- metric scale が run 中に安定しない。
- long sequence の reset/collapse を確実に検出できない。
- low texture、motion blur、照度変化で confidence が実誤差を反映しない。
- dynamic object の ghost points を除去できない。
- occupancy/traversability と unknown space の区別がない。
- local LiDAR guardian がない、または LingBot failure と同時に停止する。
- GPU OOM が command/control process へ影響する。
- map correction が `odom` control frame を不連続に動かす。

## 10. 主なリスクと対策

| リスク | 影響 | 対策 |
|---|---|---|
| 単眼 scale ambiguity | LiDAR と寸法が合わない | Sim(3) alignment、既知寸法、scale monitor |
| 長距離 pose collapse | map 全体が崩れる | window/keyframe、reset generation、drift gate |
| timestamp 不足 | 点群 overlay がずれる | capture timestamp、monotonic timeline、drop metrics |
| camera calibration 不足 | geometry が歪む | intrinsics/distortion/extrinsics を固定・hash化 |
| dense output | network/browser overload | confidence、stride、voxel、chunk、LOD |
| CUDA OOM | visual map 停止 | separate process、memory limit、restart policy |
| dependency 衝突 | Cockpit が起動不能 | separate env/container、digest lock |
| dynamic scene | ghost map | temporal/dynamic filter、unknown 扱い |
| checkpoint supply chain | 任意コード・改ざん | official source、SHA-256 pin、隔離環境 |
| viewer port 競合 | Cockpit 起動失敗 | Viser を `8081` 等へ変更 |
| map を safety と誤認 | 危険な行動 | UI badge、authority 分離、advisory-only |

## 11. security、license、再現性

source code は Apache License 2.0 で公開されている。

- [LingBot-Map LICENSE.txt](https://github.com/Robbyant/lingbot-map/blob/7ff6f3ed0913d4d326f8f13bbb429c4ffc0195c2/LICENSE.txt)

導入前に次を確認する。

- checkpoint の配布・商用利用条件
- VGGT、DINOv2、FlashInfer、Viser 等の third-party notice
- model file の SHA-256
- container base image と Python package lock
- CUDA/PyTorch compatibility

公式 demo は checkpoint load で `torch.load(..., weights_only=False)` を使用しているため、未知の checkpoint を読み込まない。公式配布元から取得し hash を固定し、control credential を持たない隔離環境で load する。

- [Checkpoint loading implementation](https://github.com/Robbyant/lingbot-map/blob/7ff6f3ed0913d4d326f8f13bbb429c4ffc0195c2/demo.py#L152-L156)

## 12. 工数の目安

以下は commit 時点の技術調査に基づく概算であり、納期の確約ではない。

| 範囲 | 目安 | 完了の意味 |
|---|---:|---|
| offline viewer PoC | 0.5〜2日 | GO2 video で Viser 表示、FPS/VRAM 計測 |
| live visual-map layer | 1〜2週間 | sidecar、bounded queue、filter、WS/UI、health |
| LiDAR/odom alignment | 3〜6週間 | calibration、同期、Sim(3)、reset、replay評価 |
| navigation/control 利用 | 別プロジェクト規模 | occupancy、planner、guardian、安全gateまで含む |

最短の価値は Option B の read-only live visualization にある。Option C 以降は UI 機能ではなく、calibration/localization system の開発として見積もる。

## 13. 未決事項

実装判断前に次を決める。

1. 目的は「空間を見やすくする operator UI」か、「将来 navigation map に使う研究」か。
2. camera raw frame と正確な capture timestamp を Unitree 側から取得できるか。
3. GO2 camera intrinsics/extrinsics をどの手順で較正するか。
4. LingBot worker を同一 RTX 5090 で他モデルと時分割するか、専用にするか。
5. model の通常版と long checkpoint のどちらを評価するか。
6. visual map の retention、chunk、LOD、browser memory 上限をどう置くか。
7. reset/collapse の検出閾値と UI 表示をどう定義するか。
8. checkpoint license と third-party notices を配布形態に照らして確認できたか。

## 14. 採用判断

現時点の判断を次に固定する。

```text
Repository evaluation: PASS
Offline GO2-data PoC: RECOMMENDED
Read-only Cockpit visual layer: RECOMMENDED AFTER PoC
LiDAR/odom-aligned overlay: CONDITIONAL
Replacement for LiDAR/elevation/StairModel: REJECTED
Direct navigation/control authority: REJECTED
Project status: CANDIDATE / NOT ADOPTED
```

次の実装着手点は、Cockpit へ直接コードを足すことではなく、Phase 2 の timestamp/calibration/replay 記録と、隔離した offline viewer PoC である。
