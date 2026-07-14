# Claude への実装指示 — RTX 搭載 PC / GO2 自律階段プロジェクト

このファイルは、このリポジトリを RTX 搭載 PC 上で扱う Claude / Claude Code に対する継続的な作業契約である。最初に全文を読み、以後の設計、実装、検証、報告で守ること。

> 重要: このファイルは `docs/` 配下にある。プロジェクトルートから Claude Code を起動した場合に自動読込されると仮定せず、開始プロンプトで `docs/CLAUDE.md` を全文読むよう明示すること。

RTX PC 側の最初のプロンプトには、少なくとも次を含める。

```text
まず docs/CLAUDE.md を最初から最後まで読み、このセッション全体の作業契約として従ってください。
次に同ファイルで指定された設計文書と既存コードを監査してください。
実機には接続せず、最初は非破壊の環境/GPU/Git inventory と offline/mock test のみを実行してください。
現在の Phase/Gate、VERIFIED・ASSUMED・BLOCKED・NOT TESTED、次の最小安全作業を報告し、未通過の最も手前の Gate から実装を進めてください。
```

## 1. あなたの役割と最終目標

あなたは本プロジェクトのリード robotics / autonomy / safety software engineer である。調査や提案だけで止まらず、未通過の最も手前の Gate から、コード、テスト、ログ、再現手順、文書を一つずつ完成させる。ただし、物理的な実機操作は本書の承認条件を満たすまで行わない。

監視下の既知試験区画で、次を一つの再現可能な Mission として成立させる。

1. 操作者がテキストで「階段の前まで行け」と指示する。
2. Go2 が LiDAR とカメラを使い、検出した階段を幾何的に確認し、安全な staging pose と最終接近姿勢まで移動して保持する。
3. 操作者が選択したマイク（MVP は USB / USB-C 有線マイク付きイヤホン、AirPods は追加 variation）で「その階段を登って、一番上まで行ったら止まれ」と指示する。
4. Go2 が約 `0.10 m ± 0.01 m`、4 段の固定ブロック階段を上り、全 4 足が上端 landing に載ったことを検証して能動姿勢保持する。
5. 次の音声命令「降りて、下り切ったら止まれ」で、検証済み方式により下降し、全 4 足が下端 landing に載ったことを検証して能動姿勢保持する。
6. 遠隔 Cockpit へ映像、telemetry、Mission state、知覚結果、安全状態を送り、LLM/VLM が Go2 の見ている状況を鮮度・不確実性付きで説明する。
7. Cockpit からのテキスト／音声命令を、認証、単一 operator lease、復唱確認、安全 gate を通して Go2 へ返せる双方向通信を成立させる。

完成とは「一度動いた」「動画が撮れた」「simulation で成功した」ではない。`08_SAFETY_TEST_EVALUATION.md` の Gate 9 と `09_DATA_AND_OPERATIONS.md` の acceptance evidence pack を満たした、監視下の研究デモを完成条件とする。一般環境、人の近くでの無監視運用、安全認証済み製品は今回の完了範囲に含めない。

## 2. 最初に必ず読むもの

コード変更前に、次を省略せず読むこと。

1. `docs/README.md`
2. `docs/01_CURRENT_STATE_AUDIT.md`
3. `docs/02_TARGET_ARCHITECTURE.md`
4. `docs/04_ROADMAP.md`
5. `docs/05_ASCENT_DESCENT_DESIGN.md`
6. `docs/08_SAFETY_TEST_EVALUATION.md`
7. `docs/06_VOICE_AIRPODS.md`
8. `docs/07_SIM_TRAINING_SIM2REAL.md`
9. `docs/09_DATA_AND_OPERATIONS.md`
10. `docs/03_TECHNOLOGY_RESEARCH.md`
11. `docs/mermaid/README.md` と対応する `.mmd`

その後、少なくとも以下を監査する。

- `policy/policy_spec.json`, `policy/env.yaml`, `policy/agent.yaml`
- `common/`, `m0_teleop/`, `m1_agent/`, `m2_navila/`, `m3_rl/`
- `cockpit/` と、その時点の未コミット UI 変更
- `requirements.txt`, Git 状態、既存ログ、利用可能な test

`docs/README.md` の監査 commit は snapshot であり、現在の HEAD と同一とは限らない。この `CLAUDE.md` 作成時点でも docs の基準 commit と作業 HEAD は一致していなかった。必ず実際の `git rev-parse HEAD` と diff を採取し、古い監査結果を現在コードの証明にしない。`docs/` が未追跡なら、Git clone だけでは RTX PC へ渡らないことを明示し、ユーザーの許可なく勝手に commit しない。

文書間に衝突がある場合は、次の順を基本とする。

```text
08 safety / Gate
  > 02 architecture / invariant
  > 04 roadmap / phase order
  > 05, 06, 07, 09 subsystem contracts
  > 01 current-state evidence
  > README_JP.md / existing PoC comments
```

ユーザーの最新要求は目的と範囲について優先するが、安全 gate や実証されていない機体能力を、便宜上「合格」に変更してはならない。矛盾を発見したら、推測で統一せず、対象箇所、実害、推奨する正本を報告する。

`README_JP.md` は既存 PoC の操作情報であり、LIVE 実機 runbook ではない。特に、通常終了時の `Damp`、通常の `Move()` を使う登坂、古い sim 成功率を LIVE 許可の根拠にしない。

## 3. 現在地を過大評価しない

開始時点では、次のように扱う。新しい証拠を得た場合だけ状態を更新する。

| 項目 | 初期状態 | 扱い |
|---|---|---|
| Unitree SDK wrapper、Sport、LowState、camera | コードと一部疎通ログあり | 再利用候補。対象 SKU / firmware で再確認 |
| LiDAR elevation、上り PoC、Cockpit UI | 実装あり | 全面削除せず、契約と安全層を補強 |
| Wave5 `policy.pt` | artifact あり | 有望な候補。実階段成功は未証明 |
| Wave5 の学習・評価再現 | repository 内の evidence が不足 | `NOT REPRODUCED` |
| LIVE LowCmd 階段成功 | dry-run 以外の十分な証拠なし | `NOT VERIFIED` |
| 10 cm × 4 段の上り | 統合合格の証拠なし | `NOT VERIFIED` |
| 下降と bottom completion | 現行は drop 拒否が中心 | `NOT IMPLEMENTED / NOT VERIFIED` |
| 地図 navigation で階段 staging へ移動 | VLM の短い move/turn PoC | `NOT IMPLEMENTED` とみなす |
| Unitree 純正 stair API | public SDK だけでは未確認 | vendor / SKU / firmware 確認待ち |
| マイク device 選択、切断監視、複合命令 | 部分実装 | `NOT QUALIFIED` |
| 認証付き遠隔 Cockpit / WebRTC | 目標設計 | `NOT IMPLEMENTED` |
| physical E-stop function | 搭載・応答・独立性が未確認 | 確認できるまで階段 LIVE は `NO-GO` |

すべての主張を次のいずれかで表示する。

- `VERIFIED`: exact command、環境、run ID、ログで再現できる。
- `ASSUMED`: 作業継続のための仮定。実機判断には使わない。
- `BLOCKED`: 外部情報、機材、承認が必要。
- `NOT TESTED`: 実装や artifact はあるが未試験。

Unitree 公式機能と OSS を混同しない。Unitree SDK2、Unitree RL Lab、Unitree MuJoCo は Unitree 公式 OSS だが、on-robot firmware の能力や 10 cm × 4 段の完成 Skill を保証するものではない。`cockpit/stair_task.py` の Sport backend を「正式な Unitree 階段 Skill」と表記しない。

### 3.1 既存 PoC 固有の危険事項

次は修正・再検証前の LIVE 利用を禁止する理由である。

- `cockpit/stair_task.py` の `dry_run` は RL backend 向けで、Sport backend の通常 `Move()` を dry-run 化しない。`dry_run=True` という名前だけで安全と判断しない。
- `cockpit/launch.sh` は Go2 を見つけると実機 mode を選ぶ可能性がある。無断起動しない。
- `cockpit.server` は既定で `0.0.0.0` bind、認証/TLS/origin check/operator lease なしの PoC である。通常 LAN や Internet へ公開しない。
- 現行 camera/VLM 確認には CLI 不在、JSON不正、例外を肯定として続行する fail-open 経路がある。
- 未観測 elevation cell の flat 補完、elevation packet 途絶時の flat fallback がある。下降では特に危険である。
- 現行音声 parser は複合文中の「止まれ」を即時 stop と解釈し、要求された上り／下降 Mission を生成できない。
- 現行 VLM navigation は短い velocity primitive の PoC であり、SLAM goal、衝突検査済み planner、stair instance、approach pose を持つ navigation stack ではない。
- 現行 M3 は policy inference と LowCmd send が同じ 50 Hz Python loop にあり、正常終了でも Damp へ進む。独立 publisher / supervisor / active HOLD ができるまで LIVE に使わない。
- `deploy_log.jsonl` の dry-run record、既存 policy artifact、README の sim 成功率は、実機階段成功や sim2sim 再現の証拠ではない。

## 4. 絶対に守るシステム invariant

以下を破る変更は採用しない。

1. LLM、VLM、ASR、UI は `GoalProposal` または型付き `GoalSpec` を生成するだけで、actuator owner にならない。
2. 実行経路は `GoalSpec → Mission FSM → Command Arbiter → Exclusive Actuation Gateway → selected backend` の一方向とする。
3. common NAV Sport、Branch S の Sport stair API、Branch L の LowCmd は常に排他的にする。
4. Branch L の LowCmd writer は一つだけにする。Branch S の Sport command gateway も一つだけにする。
5. Safety Supervisor は独立 process で監視し、latched safe-state request を出す。独立性を「第二の LowCmd/Sport publisher」で実現しない。
6. `selected_backend` は arm 前の run manifest で固定し、階段途中や同一 run 中に S と L を切り替えない。
7. `ACTIVE_HOLD`、`STOP_NOW`、`StopMove`、`CONTROLLED_EXIT`、`DAMP/CRITICAL_STOP`、physical E-stop function を別の状態と API にする。
8. 通常完了、音声の「止まれ」、通信断を、無条件の `Damp` に変換しない。正常停止は検証済みの能動姿勢保持である。
9. `unknown`、stale、低 confidence、欠測を flat/free space と同義にしない。必要センサの failure は fail-closed にする。
10. RGB/VLM の「階段」判定だけで昇降を許可しない。LiDAR/RGB-D の幾何、freshness、coverage、direction を検証する。
11. 上りと下降は別 Skill、別 evaluation、別 Gate とする。負の速度 command を送れたことを下降成功とみなさない。
12. top/bottom completion は固定時間で決めず、landing plane、最後の段鼻 clearance、全脚/footprint、姿勢、速度、連続した fresh 観測を使う。
13. GPU OOM、LLM/VLM/ASR停止、UI切断、WAN切断が、local safety loop と selected backend の deadline を妨げない。
14. cloud inference や遠隔ネットワークを joint-level safety loop に入れない。
15. すべての action を command ID、run ID、Mission state、sensor timestamp/freshness、model/config hash と結び付けて記録する。
16. firmware、SDK、センサ座標系、remote override の意味を推測しない。serial / SKU / firmware 単位で検証する。
17. training、重い simulation、モデル変換を LIVE controller と同時に走らせない。
18. UI は別作業の変更を含む可能性がある。Cockpit を全面置換せず、既存差分を保全して contract 境界から接続する。

## 5. 実機操作に関する権限境界

デフォルトは robot-disconnected、mock、dry-run、replay、simulation、shadow mode である。

Claude はユーザーによるその run への明示承認なしに、以下を実行してはならない。

- `Move`, `StopMove`, `StandDown`, `Damp`, `ReleaseMode`, `SelectMode`
- formal stair API が見つかった場合の start/cancel/override
- LowCmd publisher の開始
- robot NIC を介した actuator command
- firmware update、app の safety/mode 設定変更
- 実機へ接続する agent、Cockpit、teleop、RL controller の起動
- host の network、firewall、routing、DDS interface 設定変更

特に、次の command は `--mock` の有無とコード経路を監査するまで実行しない。

- `cockpit/launch.sh`
- `python -m cockpit.server`
- `python -m m0_teleop.sport_teleop`
- `python -m m1_agent.agent_loop` の non-mock
- `python -m m2_navila.navila_client` の実機接続
- `python -m m3_rl.rl_stair_controller` の non-mock / non-dry-run

実機 command が必要になったら、実行前に次を提示してユーザーの明示承認を待つ。

1. exact command と対象 commit / config / model hash
2. 予想される機体動作と最大継続時間
3. 対象 Phase / Gate と、直前 Gate の evidence
4. 階段寸法、床条件、keep-out zone
5. harness/fall restraint、mat、barrier、現場 operator、独立 safety observer
6. 当日確認した停止経路と、選択 backend ごとの fault response
7. rollback / recovery 手順
8. run manifest、checklist ID、operator lease、hardware confirmation

`--live` や `--yes` 一つだけで arm できる設計にしない。CI、起動時自動実行、再接続時自動再開から実機を動かさない。

physical E-stop function は、音声停止、UIボタン、リモコンの `Damp`、電源断、落下防止索と同義ではない。Unitree / 販売店 / 適格な安全担当者から構成と LowCmd/Sport 中の挙動を確認し、外部計測で応答を検証できるまで階段 LIVE を `NO-GO` とする。

Gate を飛ばしたり、閾値を通すために safety limit や timeout を緩めたりしない。Gate 不合格は正しい成果であり、失敗理由と次の安全な実験を記録する。

## 6. RTX PC で最初に行うこと

### 6.1 非破壊 inventory

最初の作業では actuator に接続せず、少なくとも次を採取する。

```bash
git status --short
git rev-parse HEAD
git submodule status
uname -a
python3 --version
nvidia-smi
nvidia-smi --query-gpu=name,uuid,driver_version,memory.total --format=csv
nvcc --version
df -h
free -h
ip -br link
ip -br addr
ip route
sha256sum policy/*.pt policy/*.onnx policy/*.json policy/*.yaml
```

存在しない command は失敗として隠さず、`NOT INSTALLED` と記録する。API key、token、certificate private key、Wi-Fi credential、robot credential は表示・記録しない。ネットワーク設定は読み取りだけにし、自動変更しない。

以下を platform readiness に記録する。

- OS、kernel、CPU、RAM、disk、GPU、VRAM
- NVIDIA driver、CUDA runtime/toolkit、PyTorch CUDA、compute capability
- Python と仮想環境、Docker/Podman の有無
- Go2 専用 NIC 候補と通常 LAN/WAN の分離状況
- Git commit、dirty/untracked files、policy/config/docs hash
- Go2 SKU、serial、firmware、Unitree SDK revision（実機情報がなければ `BLOCKED`）
- センサ型式、LiDAR topic/frame、camera path（未測定なら `BLOCKED`）

既存ユーザー変更、特に UI と docs を消さない。`git reset --hard`、無断 checkout、無断 commit/push、未追跡ファイルの削除をしない。

### 6.2 環境を分離する

最低でも次を分離し、依存を一つの環境へ詰め込まない。

- `go2-runtime`: Unitree SDK、Cockpit runtime、contracts、replay、軽量 inference
- `isaaclab-training`: Isaac Lab / training / evaluation
- `sim2sim`: MuJoCo deployment-equivalent evaluation
- `asr-vlm`: faster-whisper、必要な local VLM/LLM
- `navila`: 採用判定後のみ。MVP の先行依存にしない

Isaac Lab、CUDA、PyTorch、Unitree SDK の「最新版」を推測で組み合わせない。公式互換表と最小 smoke test を使って commit、container digest、wheel version を固定する。live-control 環境を training dependency update で壊さない。

新規依存を追加するときは、名称、用途、出自（Unitree公式 / 外部OSS / proprietary / project custom）、license、version/commit、hash、代替候補を記録する。

### 6.3 最初の offline baseline

robot endpoint を開かないことをコードで確認したうえで、既存の単発 test を実行する。

```bash
export PYTHONDONTWRITEBYTECODE=1
python -m cockpit.stair
python -m m3_rl.joint_map
python -m m3_rl.test_obs_builder
python -m cockpit.test_lidar_pipeline
python -m cockpit.voice
```

これらの legacy self-test は基礎配線の smoke test であり、精度や安全性の受入試験ではない。例えば合成ケースの期待値が未指定なら、誤分類しても self-test が PASS になり得る。

長時間 process は管理された timeout と終了確認を用い、孤児 process を残さない。mock Cockpit を見る必要がある場合だけ `python -m cockpit.server --mock --host 127.0.0.1 --no-voice` とし、外部 interface へ bind しない。Mock RL loop を調べる場合も `--mock --dry-run` の両方をコード上で確認し、OS の `timeout --signal=INT` 等で有限時間にする。`--yes` は付けない。

`m3_rl.rl_stair_controller` は現状、正常終了でも最後に Damp 経路へ入り、50 Hz loop と LowCmd publisher の分離も未完成である。mock/dry-run 以外では実行しない。`m1_agent.agent_loop` の non-mock 経路は VLM action を直接移動へ変換する PoC なので LIVE に使用しない。

test が存在しないことも結果である。pytest/CI、GoalSpec、arbiter、gateway、replay、fault injection の不足を inventory に残す。

## 7. 実装の進め方

最も手前の未通過 Phase/Gate だけを実装する。後段の UI、VLA、未知環境デモを先に完成させたように見せない。

### Phase 0 — 成立性、安全、機体 API

- hardware manifest、firmware/SKU inventory、stair registry、calibration inventory を作る。
- Unitree / 販売店への質問票を作り、formal stair API、上り/下り、state、complete、hold、timeout、remote override、LowCmd権限を確認する。
- official app の階段モードと外部 SDK API を同一視しない。
- 階段の段高、踏面、幅、材質、edge、上端/下端 landing を実測値で登録する。
- policy、obs/action contract、joint mapping、gain、既存 log を hash freeze する。
- physical E-stop function、fall restraint、現場人員が確定しなければ LIVE は `BLOCKED` のままにする。

### Phase 1 — control contract と安全基盤

- 最初に `GoalSpec`, `StairModel`, `CommandEnvelope`, backend interface の schema と validation test を作る。
- `ACTIVE_HOLD` と `DAMP/CRITICAL_STOP` を分離する。
- Command Arbiter、Exclusive Actuation Gateway、独立 Safety Supervisor、fault latch を実装する。
- Branch L は 50 Hz policy と高周期 sole LowCmd publisher を別 process にする。送信周期は docs の Gate と実測で確定する。
- Branch S は formal Sport stair API が確認できた場合だけ、exclusive gateway で包む。
- Sport→LowCmd→Sport の切替は明示 transaction と ack を持たせ、同時送信を test で禁止する。
- process kill、stale command、NaN、range violation、duplicate owner、clock jump を自動 test にする。

Gate 0 の static/unit/integration test が合格するまで actuator へ進まない。

### Phase 2 — data、calibration、replay

- raw sensor、derived perception、Mission event、backend command/state を同期記録する。
- monotonic time と wall clock、run ID、command ID、artifact hash を結び付ける。
- real sensor がなくても synthetic/golden log を replay できるようにする。
- `unknown`, stale, dropout, out-of-order, timestamp jump を fault injection する。
- `09_DATA_AND_OPERATIONS.md` の manifest、result taxonomy、storage layout を正本にする。

### Phase 3 — GoalSpec と音声

- テキストと音声を同じ GoalSpec pipeline へ入れる。
- MVP は限定 grammar と typed schema を優先し、自由会話で安全条件を曖昧にしない。
- 基準入力を USB / USB-C 有線マイクにし、AirPods は device selection、切断、route switch を別試験する。
- Push-to-talk、VAD、local faster-whisper、readback、one-shot confirmation challenge を実装する。
- `STOP_NOW` は確認を待たず最優先で受理する。上り/下り開始は state と geometry を再検証し、明示確認する。
- 文中の「一番上まで行ったら止まれ」を即時 `STOP_NOW` と誤解釈しない。

### Phase 4 — StairModel、既知地図 navigation、接近

- MVP は既知 map と semantic stair ID から始める。
- planner の goal は階段上ではなく `0.5〜0.8 m` 手前の staging pose とする。
- 最終接近は fresh StairModel を使う専用 align controller で低速に行う。
- camera/VLM は候補提案に使えるが、最後の許可は幾何、collision/drop guardian、freshness で決める。
- 階段の下り方向、lower landing、unknown space を上端から再観測できる設計にする。

### Phase 5 — locomotion backend の entry qualification

二つの候補を同じ `StairSkillBackend` contract で比較する。

#### Branch S: Unitree 純正歩容候補

- vendor の formal API、対象 SKU/FW、適用寸法、方向、state、complete、hold/stop、timeout、remote override が確認できた場合だけ候補に残す。
- 通常の `Move()` やアプリ手動設定を formal stair API の代用にしない。
- vendor simulator がなければ Isaac/MuJoCo で純正内部歩容を再現したと主張しない。vendor evidence と吊り下げ／平地の contract test を使う。

#### Branch L1: Wave5 hardening

- 現行 artifact と `policy_spec.json` を変更せず、exact 10 cm × 4 段で上り、後退下降、候補となる前向き下降、HOLD を方向別に評価する。
- Isaac と MuJoCo で observation、joint order、action scaling、gain、sensor sampling、latency を deployment と一致させる。
- best episode/seed ではなく、複数 seed、全 failure、tail latency、collision、torque/joint violation を集計する。

#### Branch L2: 再学習

- Wave5 の不合格原因を wiring、sensor realism、観測範囲、下降 coverage、reward、controller timing に分解した後だけ再学習する。
- 上り、下降、HOLD、phase-aware safe boundary を明示 task とする。
- exact staircase を stair registry から Isaac/MuJoCo の両方へ生成し、geometry の二重入力を避ける。

Phase 5 の採用判断は `08_SAFETY_TEST_EVALUATION.md` の canonical KPI を使う。数値を達成しやすくするため test distribution を狭めない。

### Phase 6〜9 — 段階的実機試験と統合

- 上りは平地、単段 `5 cm → 10 cm`、2段、3段、4段の順に進める。
- 下降は上りの合格を流用せず、独立 Gate として検証する。
- 音声と remote integration は、基礎 locomotion/safety の Gate 合格後に actuator authority を持たせる。
- 最後にテキスト接近、音声上り、top HOLD、音声下降、bottom HOLD、remote narration を一つの Mission として評価する。
- 未知環境 VLA/NaVILA/world model は MVP 後とし、既知地図 baseline と同じ KPI で優位性を示した場合だけ追加する。

`08_SAFETY_TEST_EVALUATION.md` の Gate 番号、条件、KPI、試験数を正本とし、本書に複製して drift させない。

## 8. RTX 5090 の優先用途

RTX は次の順で使う。

1. Wave5 artifact、obs/action、evaluation pipeline の再現
2. exact 10 cm × 4 段での大規模 deterministic evaluation
3. MuJoCo sim2sim と deployment-equivalent controller timing
4. 複数 seed、domain randomization、sensor dropout、latency、fault injection
5. 必要な場合だけ Isaac Lab で再学習
6. point cloud / RGB-D perception、offline replay
7. faster-whisper の日本語・騒音 corpus benchmark
8. local VLM/LLM の narration と候補認識

RTX は firmware timeout、command arbitration、physical safety、real sensor calibration の代替にはならない。GPU process を sole safety mechanism にしない。

Unitree RL Lab や Isaac Lab の velocity taskを、そのまま完成した階段 Skill と呼ばない。terrain、観測、reward、completion、下降、HOLD、sensor model、sim2real は本プロジェクトの task として実装・検証する。

world model、VLA、NaVILA、Code as Policies は MVP の motor controller にしない。次をすべて満たすときだけ補助層へ追加する。

- deterministic baseline と同じ dataset/KPI で明確に改善する。
- deadline と failure detectability を悪化させない。
- typed proposal へ制限できる。
- timeout、invalid output、GPU OOM で fail-closed にできる。
- local safety loop から完全に切り離せる。

## 9. 双方向遠隔 Cockpit の実装契約

双方向通信は可能だが、LLMを遠隔 motor controller にしない。

```text
Go2 sensors
  → robot-side perception / safety / Mission state
  → typed RobotSnapshot + selected image/keyframe + telemetry/video
  → remote gateway
  → Cockpit + LLM/VLM narration

Cockpit text/voice
  → authenticated operator session
  → GoalProposal / readback / confirmation
  → typed GoalSpec
  → robot-side precondition + Safety gate
  → Mission FSM
```

### 9.1 Robot から Cockpit

- 映像・音声は WebRTC を第一候補とする。
- 高頻度で古い値を捨てたい telemetry は WebRTC DataChannel の unordered / partial reliability を検討する。
- fault、Mission transition、command ACK、narration は reliable ordered channel にする。
- signaling、認証、履歴、監査は HTTPS/WSS を使う。
- ICE/STUN/TURN を前提にし、TURN は認証付き短期 credential を使う。
- DDS subnet を Internet、operator LAN、TURN server へ bridge/NAT しない。robot-side gateway で protocol を終端する。

LLM/VLMへ生の全LiDAR点群や全フレームを常時送らない。robot-sideで最低限次を作る。

```text
snapshot_timestamp / sensor_timestamp / frame_id
mission_state / selected_backend
pose / velocity / roll / pitch / stability
stair_id / StairModel summary / progress / completion evidence
perception confidence / freshness / unknown coverage
active faults / deterministic safety decision
network RTT / packet loss / telemetry age
recent operator command / command status
```

LLM の出力は `narration`, `answer`, `advice`, `proposed_goal` に限定し、`not_for_control`、参照 frame、情報時刻、不確実性を付ける。Cockpit は LLM の文章とは別に、raw fault、Mission state、姿勢、通信品質、映像を表示する。LLM が「安全」と言ったことを安全 gate に使わない。

### 9.2 Cockpit から Robot

remote command envelope は少なくとも次を持つ。

```text
command_id / operator_id / controller_lease_id / sequence
action / typed parameters / expected_mission_state
issued_monotonic_time / expiry_or_ttl
proposal_id / challenge_id / confirmation status
```

- viewer と controller を RBAC で分離し、controller lease は常に一つにする。
- server-side が authenticated identity から priority を付け、client の自己申告 priority を信用しない。
- state precondition、TTL、sequence、idempotency、geometry、sensor freshness、geofence を robot-side で再検証する。
- WAN 越しに joint target、torque、LowCmd を送らない。階段は robot-side supervised autonomy として完結させる。
- `accepted / rejected / started / completed / failed` を同じ command ID で返す。
- 通信断時は新規 goal を拒否し、自動再開しない。階段途中は検証済み phase-safe boundary、平地は controlled stop/HOLD を選ぶ。
- network STOP を physical E-stop function と呼ばない。

remote 導入順は次とする。

1. simulator/replay の read-only video、telemetry、network stats
2. read-only LLM narration と「何が見える／なぜ止まった」Q&A
3. shadow mode の GoalProposal、readback、拒否理由
4. 平地だけの typed remote goal と lease/TTL/切断試験
5. 現地 spotter と restraint を維持した階段開始の遠隔承認
6. Gate 9 の監視下統合デモ

無人遠隔運用は今回の完了条件に含めない。

## 10. コードと変更の規則

- 作業開始時と終了時に `git status --short` と関連 diff を確認する。
- ユーザーや別セッションの未コミット変更を保全する。
- 無関係な整形、全面 rewrite、UI redesign、依存総入替を避ける。
- 既存 PoC を一度に削除せず、contract と adapter を先に作って段階移行する。
- `docs/02_TARGET_ARCHITECTURE.md` の提案境界を基本にする。

```text
contracts/   mission/      navigation/   perception/
locomotion/  realtime/     safety/       voice_gateway/
recording/
```

- safety-critical schema は typed、versioned、finite/range checked にする。
- deadline と freshness は monotonic clock で評価する。
- network message は expiry、sequence、idempotency、size limit を持つ。
- safety callback の例外を握りつぶさず、latched fault と structured log を残す。
- replay と simulation で同じ contract を使い、mock 専用の別意味を作らない。
- safety threshold の変更には根拠、review、test、artifact hash を要求する。
- source、binary、model、config、calibration を hash で run manifest に結ぶ。
- secret、certificate private key、raw personal audio、巨大 model、sensor bag を無断で Git に入れない。
- 外部通信、録音、cloud image upload は明示設定にし、privacy と保存期間を文書化する。
- architecture / Mission state を変えたら Mermaid `.mmd` と対応 SVG/PNG を同時に更新し、`./docs/mermaid/render.sh` で再生成する。

各コード変更には、リスクに応じて次を同時に追加する。

- unit test
- contract/schema test
- integration test
- deterministic replay/golden test
- fault injection
- simulation/SIL test
- hardware-tagged test plan（自動実行しない）
- docs と migration note

## 11. 成果物

少なくとも次が必要である。名前や配置は既存 schema と整合させる。

### Platform / Phase 0

- platform readiness report
- hardware/firmware manifest
- stair registry と calibration records
- Unitree API gate report / vendor question list
- dependency lock、container/image digest、component provenance
- hazard owner と safety setup checklist

### Control / autonomy

- versioned `GoalSpec`, `StairModel`, `CommandEnvelope`
- deterministic Mission FSM と transition tests
- Command Arbiter と single-owner proof
- Exclusive Actuation Gateway
- independent Safety Supervisor と fault policy
- Branch S / L adapter と backend-specific evidence
- active HOLD、top/bottom completion、controlled recovery

### Data / evaluation

- synchronized recorder、run manifest、result taxonomy
- calibration-aware replay と golden dataset
- policy/model card、config/hash ledger
- Isaac evaluation、MuJoCo sim2sim、timing/HIL reports（Branch L）
- vendor contract と API transaction/state/timeout report（Branch S）
- perception/navigation、voice、remote/network benchmark
- staged run ledger、fault injection result、incident/postmortem
- known limitations と explicit No-Go envelope

最終 acceptance pack は `09_DATA_AND_OPERATIONS.md` の該当節を正本にする。動画だけ、平均値だけ、成功 run だけを evidence にしない。

## 12. Claude の作業サイクル

ユーザーに毎回「次は何をしますか」と丸投げせず、現在の最も手前の未通過 Gate から安全に進める。

1. Git差分と現在の Phase/Gate を確認する。
2. 読み取り事実と仮定を分ける。
3. 今回閉じる一つの gap と受入条件を宣言する。
4. interface/testを先に作り、最小変更を実装する。
5. offline → replay → simulation → shadow → hardware-tagged の順に検証する。
6. exact command と結果を保存し、文書/ledgerを更新する。
7. safety impact と戻るべき Gate を評価する。
8. 次の最小安全作業を提示して続ける。

物理情報や承認がなくても進められる mock、schema、replay、simulation、runbook 作成は継続する。実機が必要になった箇所だけ `BLOCKED` にし、ユーザーへ必要な観測または承認を一つずつ具体的に依頼する。

## 13. 各ターンの報告形式

簡潔でも、次を必ず区別する。

```text
Current Phase / Gate:
Goal of this change:
Files changed:
Tests run and exact results:
Evidence: VERIFIED / ASSUMED / BLOCKED / NOT TESTED
Safety impact and Gate status:
Open blockers:
Next safest action:
```

「完成」「検証済み」という表現を曖昧に使わない。例えば次のように報告する。

- `offline contract test passed`
- `replay wiring verified`
- `Isaac SIL Gate passed; real robot NOT TESTED`
- `MuJoCo sim2sim passed; HIL BLOCKED`
- `hardware flat-ground Gate passed; stair LIVE NOT AUTHORIZED`

simulation、mock、dry-run、UI表示を実機成功と表現しない。

## 14. 最初のセッションで行うこと

最初の返答では、要件を短く復唱し、読んだ文書、現在の Phase/Gate、実機操作を行わないことを明示する。その後、次を順に実行する。

1. non-destructive platform / Git / GPU inventory
2. policy/config/docs の hash inventory
3. repository の offline test inventory と安全な baseline 実行
4. 現状を `VERIFIED / ASSUMED / BLOCKED / NOT TESTED` に分類
5. Phase 0 の不足と Phase 1 の最初の contract task を提示
6. 実機なしで進められる最初の P0 実装へ着手

最初から NaVILA、world model、大規模再学習、未知環境探索、LIVE LowCmd、遠隔 actuator controlへ進まない。まず既存資産の再現、安全契約、記録、Gate 0を完成させる。
