# 09. データ、再現性、セキュリティ、運用

## 1. 目的

このプロジェクトでは、成功動画より先に「なぜ成功/失敗したかを再生できる記録」を作る。各実機runは、sensor、command、state、選択した制御backend、hardware、階段、operator判断を一つの `run_id` で追跡できなければならない。記録契約は、学習方策がLowCmdを送る **Branch L** と、正式な純正Sport階段APIを呼ぶ **Branch S** を区別する。

最低要件:

1. raw dataを失わず、derived dataと分ける。
2. 同じrecordingからperception、parser、Mission、completionと、選択backendの制御判断をoffline replayできる。
3. 両branchの接近で使うSport navigation APIを共通記録し、Branch Lでは実送信を試みたLowCmd mirrorとsingle writerを、Branch SではSport階段APIのrequest/response/stateとexclusive gatewayを記録する。
4. firmware、calibration、git commit、artifact hashをrunに固定する。
5. 音声/映像のprivacyと保持期限を明示する。
6. DDSとcockpitを通常LAN/Internetへ露出しない。

`selected_backend` はrun中に不変とする。Branch SからL、またはLからSへ暗黙にfallbackしない。切替が必要なら安全なlandingでrunを終了し、新しいmanifestと `run_id` でarmし直す。

## 2. 時刻とID

### 2.1 時刻

制御判断にはmonotonic clockを使い、人が読む時刻だけUTC/JST wall clockへ対応付ける。

- 同一host内: `CLOCK_MONOTONIC_RAW` 相当を基準。
- Mac voice gateway、5090、companion compute: PTPを優先し、無理ならchrony/NTPとoffset計測。
- Go2 topic: message timestampの意味とclock domainをtopicごとに検証。
- 受信時刻、source timestamp、sequence numberを別fieldで保存。
- clock jumpを検知し、データを補間して隠さない。

目標offsetは用途別に決める。joint/IMUとBranch LのLowCmdはms級、LiDAR/camera fusionは実測motion errorから許容値を定め、音声は数十ms級でもよい。すべてを同じ曖昧な「同期済み」にしない。

### 2.2 ID

```text
mission_id  一連の「接近→上り→待機→下り」
run_id      一回のarmからdisarmまで
goal_id     一つの実行可能GoalSpec
stair_id    物理階段とgeometry revision
robot_id    serial/SKU/firmwareの組
model_id    immutable model artifact（locomotionはBranch L、ASR/perceptionは共通）
bag_id      raw recording単位
```

同じ `goal_id` の再送はidempotentにackし、再実行しない。

## 3. 記録するデータ

### 3.1 raw robot/sensor stream

| データ | 現行/候補topic/stream | 適用 | 必須field |
|---|---|---|---|
| LowState | `rt/lowstate` | 共通 | joint q/dq/tau、IMU、foot estimate、temperature、battery、remote bytes、seq |
| LowCmd送信mirror | actuator server内部audit stream | Branch Lのみ | q/dq/kp/kd/tau、writer/owner、action/tx seq、deadline、clamp前後、write結果 |
| Sport navigation API transaction | exclusive actuation gateway内部audit stream | 両branchのNAV phase | `Move`/`StopMove` method・parameter、request/response seq、ack/result、deadline、authority/mode generation |
| Sport階段API transaction | exclusive Sport gateway内部audit stream | Branch Sのみ | method、parameter、request/response seq、result、deadline、authority/mode generation |
| Sport skill/state | vendor state API、`rt/sportmodestate`、gateway event | Branch S必須。Lはmode切替前後に取得可能な範囲 | raw state、pose/velocity/gait、availability、vendor seq |
| LiDAR raw/deskew | `rt/utlidar/cloud`, `cloud_deskewed` | 共通 | xyz/intensity、source/receive time、frame |
| LiDAR odometry | `rt/utlidar/robot_odom` | 共通 | pose/twist/covariance、reset counter |
| RGB | VideoClient/bridge | 共通 | frame timestamp、exposure、resolution、drop counter |
| optional RGB-D | camera topics | 構成依存 | RGB/depth/intrinsics/extrinsics、invalid mask |
| network/control timing | internal metrics | 共通、fieldはbackend依存 | RTT、inter-arrival、jitter、deadline miss、queue depth、API latency |

`SportModeState` はLow-level modeで消える可能性があるため、記録できない期間をゼロ/静止として補わず、`unavailable` として残す。LowCmd mirrorも「robotが受理・実行したcommand」ではなく、sole writerがDDS writeを試みたpayloadと結果である。LowStateと外部計測を併用し、送信、受理、物理応答を混同しない。

### 3.2 derived stream

- TF/pose、elevation map update、187点height scan。
- known/unknown、cell age、variance、coverage。
- `StairModel` 全revisionとlockされたrevision。
- detector/VLM output、confidence、timeout/error。
- `GoalSpec`、ASR transcript/evidence、validator結果。
- Mission FSM transition、precondition snapshot、reason code。
- command arbiterのrequest、winner、owner lease、authority generation、mode generation。
- 両branch共通のSport navigation request/response/ack、bounded velocity、`StopMove`、staging到達時のmode handoff。
- Branch Lのpolicy input/output、clamp/rate limit、inference time、LowCmd writer timing。
- Branch SのSport階段skill正規化state、API timeout、`StopMove`、Damp、remote override判定。
- Safety Supervisorのstate、limit、fault、選択action。
- top/bottom completion各subcondition。
- operator confirmation/abort、索による捕捉、manual override。

derived dataはrawから再生成できる場合でも、そのrunで実際に制御へ使った値を別streamとして必ず保存する。

### 3.3 音声

privacy既定値:

- raw audioの常時保存はしない。
- 評価runで明示同意がある場合だけ暗号化保存し、保持期限を設定する。
- 通常は `goal_id`、transcript、ASR evidence、latency、device pseudonymous IDを保存する。
- 発話に人名や会話が入った場合の削除手順を持つ。
- 学習/回帰corpusへ転用する場合は別consentとlicenseを記録する。

STOP誤認識の解析にはraw audioが有用なので、研究試験では「保存するrun」と「保存しないrun」を開始前にUIへ明示する。

### 3.4 backend別の制御証跡

共通して、すべてのcontrol eventに `run_id`、`goal_id`、monotonic timestamp、source timestamp、`authority_generation`、`mode_generation`、owner lease IDを付ける。generation不一致で拒否したrequestも捨てずに記録する。

#### 共通NAV phase — Sport navigation

Branch S/Lのどちらも階段前までの接近はSport navigation executorを使う。exclusive actuation gatewayを通る`Move`/`StopMove`のrequest、response/ack、parameter、bounded velocity、deadline、vendor sequence/state、authority/mode generationと実機pose/twistを記録する。staging到達時は`StopMove`のackと静止判定を残し、その後にBranch Sのformal stair APIへauthorityを移すか、Branch LのSport inactive ack→LowCmd enable transactionへ進む。接近commandをUIやVLMがgateway外から直接送ったrunはinvalidとする。

#### Branch L — 学習LowCmd

Branch Lでは、50 Hz前後のpolicy decisionと、既定500 HzのLowCmd送信を別streamにする。200〜500 Hzの代替rateは`T=1/f_tx` timing gateとhazard review後だけ使う。

- policy stream: `observation_seq`、`action_seq`、model/schema hash、obs/action、inference start/end、target生成時刻、valid-until、finite/range gate、clamp前後、採用/拒否reason。
- LowCmd mirror: `tx_seq`、参照した `action_seq`、writer instance ID、ZOH/filter mode、q/dq/kp/kd/tauのclamp前後、enqueue/write start/write end、DDS write結果、target age、deadline miss、mode/authority generation。
- rate/timing: target/actual policy rate、target/actual TX rate、inter-send interval、jitter、最大gap、queue depth、stale action count、deadline miss countを保存する。summary値だけでなく、各送信eventから再計算できる時系列を残す。
- lifecycle: sole writerのstart/ready/heartbeat/fault/exit、安全frameへの切替、Sport→Low-level→Sport handshakeを記録する。fault時に第2publisherを起動した証跡を「冗長化」とみなさない。

Branch LのrunでLowCmd mirrorが欠落、`tx_seq` に説明不能なgap、またはwriter identityが複数なら、性能結果にかかわらず `INVALID_RUN` またはsafety incidentとする。

#### Branch S — 純正Sport stair API

Branch SではLowCmd/policy streamを要求しない。その代わり、exclusive Sport gatewayを通る階段skillのtransactionと状態を完全に残す。

- request: gateway transaction ID、client `request_seq`、method、direction/parameter、request payload hash、lease、authority/mode generation、send時刻、deadline。階段開始だけでなく、cancel、`StopMove`、Dampも同じ形式で記録する。
- response: 対応するtransaction/request seq、vendor seqがあればその値、ack/result/error、raw response、receive時刻、latency、deadline内/外。responseのないrequestも削除せずtimeout eventへ結び付ける。
- skill state: vendorが公開するraw state/sequence/progress、availability、source/receive時刻を保存し、Mission側の正規化stateとは別streamにする。vendorが公開しない値を推測で埋めない。
- stop/timeout: `StopMove` とDampのrequest/response、client request timeout、state stale timeout、vendor skill timeout、service/process/通信断、および各event後に外部観測した姿勢・速度・接地を一つのtimelineへ結ぶ。
- remote override: LowStateのremote bytesまたは正式remote event、operator入力、gatewayが検出したauthority遷移、進行中requestの扱い、robotの物理応答を記録する。overrideの意味がfirmwareから確認できない場合は `unknown` とし、成功扱いしない。

Branch Sで正式APIのresponse/state stream、`StopMove`・Damp・timeout・remote overrideの少なくとも試験対象eventが欠落したrunは、その契約を評価するevidenceに使わない。LowCmdがinactiveであることはmode handshakeとgenerationから検証し、Branch Sに無関係なLowCmd publisherを「logging用」に起動しない。

## 4. storage layout

```text
artifacts/
  models/<model_id>/
    policy.pt
    policy.onnx
    model_card.yaml
    evaluation_report.json
  calibrations/<calibration_id>/
    lidar_body.yaml
    camera_body.yaml
    time_offsets.yaml
  stairs/<stair_id>/
    geometry.yaml
    photos/
runs/YYYY/MM/DD/<run_id>/
  manifest.yaml
  raw.mcap
  derived.mcap
  audio.enc                 # opt-in only
  operator_notes.md
  result.json
  checksums.sha256
datasets/
  manifests/
  splits/
```

大容量artifactをGitへ直接入れず、immutable object store/NASへ置き、manifestとhashだけversion管理する。policyとcalibrationを「最新版」という可変pathで参照しない。

## 5. run manifest

`schema_version: 2`では`control.selected_backend`（stair backend）をdiscriminatorにし、arm前に署名済みmanifestへ固定する。manifestはexpected gateway/owner/leaseと初期`COMMON_NAV/SPORT`を固定するが、runtime generation値は埋めない。各authority/mode activation時にgenerationを採番し、append-only `runtime_transition_stream`へ全件記録する。preflightはbackendを選び直さず、manifest一致とreadinessを再検証する。

```yaml
schema_version: 2
run_id: 20260713T120000Z_go2x_0001
mission_id: mission_0001
started_at_utc: 2026-07-13T12:00:00Z
operator_ids: [operator, safety_observer]
robot:
  sku: Go2-X
  serial: redacted-id
  firmware: exact-version
  lidar_model: exact-model
  battery_id: battery-01
network:
  dds_domain: 0
  robot_nic: exact-interface
  topology_hash: sha256:...
software:
  repo_commit: exact-commit
  dirty_worktree: false
  container_digest: sha256:...
  unitree_sdk2_commit: exact-commit
  ros_distro: exact-version
models:
  locomotion: {id: ..., sha256: ...}   # Branch Lのみ。Branch Sでは省略
  asr: {id: ..., sha256: ...}
  parser: {id: ..., schema_hash: ...}
calibration:
  id: calibration-20260713
  sha256: ...
stair:
  id: stair-10cm-4-v1
  geometry_sha256: ...
mission:
  direction: ASCEND_THEN_DESCEND_BACKWARD
  allowed_envelope_sha256: ...
  top_hold:
    warning_s: 30
    max_s: 60
    measured_worst_case_energy_wh: ...
    reserve_factor: 1.3
    required_energy_wh: ...
    motor_temperature_limit_c: ...
    timeout_recovery: EXTERNAL_CAPTURE_AT_TOP  # VALIDATED_CONTROLLED_DESCENT | EXTERNAL_CAPTURE_AT_TOP
    recovery_deadline_s: ...
    authorization_id: signed-run-approval-id
    authorization_sha256: sha256:...
control:
  selected_backend: BRANCH_L_LOWCMD_POLICY
  expected_gateway_id: exclusive-actuation-gateway-01
  expected_stair_owner_id: stair-skill-backend
  operator_lease_id: lease-0001
  initial_phase: COMMON_NAV
  initial_mode: SPORT
  runtime_transition_stream: derived/control_authority  # generationはruntime eventで採番
  branch_l:
    policy_model_id: locomotion-model-id
    observation_schema_sha256: sha256:...
    action_schema_sha256: sha256:...
    writer_instance_id: lowcmd-writer-01
    action_transport: ZOH
    policy_rate_hz_target: 50
    lowcmd_tx_rate_hz_target: 500
    lowcmd_tx_period_ms: 2.0
    jitter_p99_budget_ms: 1.0
    tx_interval_p99_limit_ms: 3.0
    tx_interval_max_limit_ms: 10.0
    lowcmd_deadline_ms: ...
safety:
  checklist_id: ...
  supervisor_config_sha256: ...
  phase_safe_boundary_config_sha256: ...  # gait phase別 max time/distance/step count
  signed_run_safety_config_sha256: sha256:...
  signed_run_safety_config_signature: ...
  tether: true
recording:
  bag_ids: []
  raw_audio_consent: false
```

Branch Sでは `control` を次のdiscriminated blockに置き換え、`models.locomotion` と `branch_l` を入れない。

```yaml
control:
  selected_backend: BRANCH_S_SPORT_STAIR_API
  expected_gateway_id: exclusive-actuation-gateway-01
  expected_stair_owner_id: stair-skill-backend
  operator_lease_id: lease-0002
  initial_phase: COMMON_NAV
  initial_mode: SPORT
  runtime_transition_stream: derived/control_authority  # generationはruntime eventで採番
  branch_s:
    api_contract_id: vendor-stair-api-contract-id
    api_contract_sha256: sha256:...
    gateway_instance_id: sport-gateway-01
    stair_skill_name: exact-vendor-method
    request_timeout_ms: ...
    state_stale_timeout_ms: ...
    stopmove_contract_sha256: sha256:...
    damp_contract_sha256: sha256:...
    remote_override_contract_sha256: sha256:...
```

schema validatorは条件付き必須fieldを検証する。

- Branch L: locomotion model、`branch_l`、policy/action stream、LowCmd mirror/rate/timing streamが必須。
- Branch S: `branch_s`、Sport API request/response/sequence/state streamと、停止・timeout・remote overrideのevent dispositionが必須。該当eventが発生・注入されたときは全件を記録し、対象外のrunでは `NOT_EXERCISED` と契約evidenceへの参照を残す。LowCmd mirrorやpolicy modelは必須にしない。
- どちらも`authority_generation`と`mode_generation`をruntime control eventへ付ける。最初のCOMMON_NAV eventがexpected gateway/leaseと一致しなければarmを拒否し、stair activation eventがmanifestの`selected_backend`と一致しなければbackendをenableしない。
- ascentを含むrunでは`mission.top_hold`と署名済みsafety configを必須にする。`VALIDATED_CONTROLLED_DESCENT`は、run前の独立明示承認、下降Gate合格、実行時のfresh geometry、operator/safety leaseをすべて別eventで確認する。既定値は`EXTERNAL_CAPTURE_AT_TOP`であり、timeoutだけでblind descentしない。
- 階段中STOPを試すrunでは、gait phaseごとの`max_safe_boundary_time_s`、`distance_m`、`step_count`を持つ設定hashを必須にし、実測eventがすべてそのbound内か再計算する。

dirty worktreeで研究runを許す場合はdiff patchをartifact化し、`dirty_worktree: true` を隠さない。最終acceptance runはclean/pinned buildだけにする。

## 6. resultと失敗taxonomy

```yaml
result: SUCCESS | REJECTED_SAFE | ABORTED | FAILURE | INVALID_RUN
first_failure_layer: NONE | VOICE | GOAL | NAV | PERCEPTION | ALIGN | LOCOMOTION | COMPLETION | SAFETY | INFRA
reason_code: TOP_LANDING_UNKNOWN
intervention:
  manual_override: false
  tether_capture: false
  physical_stop: false
events:
  fall: false
  body_contact: false
  edge_miss: false
  limit_violation: false
metrics:
  approach_position_error_m: 0.0
  approach_yaw_error_deg: 0.0
  top_hold_s: 0.0
  bottom_hold_s: 0.0
```

安全に拒否したrunをlocomotion失敗と混同せず、危険な誤実行を単なるparser errorへ薄めない。`INVALID_RUN` はlogging欠落、階段変形、監視手順違反など、性能分母へ入れられない理由を必須にする。

## 7. replayとgolden test

### 7.1 replayレベル

1. **Sensor replay**: raw point cloud/image/stateからelevation/StairModelを再生成。
2. **Decision replay**: 記録したderived stateからGoal validator/Mission/FSMを再実行。
3. **Backend replay**: Branch Lは同一obsでactionとLowCmd scheduleを再現し、Branch Sは記録したrequest/response/state sequenceをmock gatewayへ入力してauthority、timeout、停止遷移を再実行する。
4. **Closed-loop sim replay**: Branch Lは実runの初期条件/遅延/geometryをMuJoCoへ再現する。Branch Sはvendorが正式simulator/API stubを提供する場合だけ同等試験を追加し、存在しない純正内部制御を推測実装しない。
5. **Shadow replay**: 新しいperception/Mission、またはBranch Lの新modelを過去runへ適用し、実送信せず比較する。

### 7.2 golden cases

最低限:

- 平地、上り4段、下降4段、単なる崖、棚/壁、未知/遮蔽。
- LiDAR stale、odom reset、camera timeout、cell coverage不足。
- 3要求文、同義表現、否定、conditional「止まれ」、独立STOP。
- topの前足だけ到達、rear feet未通過、bottomの最後脚未通過。
- 共通: owner競合、generation mismatch、lease expiry、backend gateway kill。
- Branch L: policy stale、LowCmd writer kill、rate/jitter/deadline異常、NaN action、複数writer検出。
- Branch S: request重複/順序逆転、ack欠落、state stale、`StopMove`/Damp timeout、remote override、Sport service断。

algorithm変更ごとにgolden差分をreviewし、期待値を安易に更新しない。

## 8. dataset governance

train/validation/testをframe単位でrandom splitしない。同じ階段・同じ連続runが両側へ漏れるためである。

- `train`: 複数session/lighting/approachを含む。
- `validation`: 別run、同じ許可envelope。
- `test-ID`: 日付/配置を分けたtarget階段。
- `test-boundary`: envelope境界。
- `test-OOD`: 高さ、踏面、材質、遮蔽、未知物体。
- `safety-negative`: 崖、棚、影、模様、非命令音声。

splitはstair ID、session、speaker、environment単位で固定し、manifestをGit管理する。test setを見て学習したらrevisionを上げ、旧値と比較できるようにする。

## 9. calibration管理

記録するもの:

- joint zero/offsetと可動域。
- LiDAR/body、camera/bodyのextrinsics。
- camera intrinsics/depth scale。
- sensor time offsets。
- body/foot geometryとURDF revision。
- actuator response、Kp/Kd、latency。
- LIO frame、axis、reset behavior。

再較正trigger:

- sensor mountを触った。
- 転倒/強い接触があった。
- firmware/driverを更新した。
- temperature/振動でずれを検出した。
- stair geometry benchmarkのresidualが閾値超過。

各runのcalibration IDを必須にし、後から現在のyamlで上書き再生しない。

## 10. dependencyとbuild固定

現状のunpinned `requirements.txt` とGit HEAD手動installを、次へ置き換える。

- Python: lockfile、wheel hash、Python minor version。
- ROS 2: distro、apt package version、CycloneDDS config/hash。
- Unitree SDK2/Python/ROS2: exact commitとpatch。
- Branch Lで使うIsaac Sim/Lab、Unitree RL Lab、RSL-RL: exact commit/release。
- Branch Sのvendor stair API contract、SDK、firmware compatibility matrix: document ID/hashと確認日。
- NVIDIA driver/CUDA/PyTorch: host manifestとcontainer digest。
- C++: compiler、CMake preset、dependency version。
- Branch Lのlocomotion model: hash、observation/action schema hash。共通のASR/perception modelも個別にhashを固定する。
- firmware: versionとvendor確認日。

一つの巨大containerでlive servo、ASR、trainingを同居させず、runtime imageを役割別にする。

```text
go2-realtime-runtime
go2-perception-runtime
go2-mission-runtime
go2-voice-runtime
go2-training-runtime
go2-replay-runtime
```

## 11. CIと試験区分

### 11.1 commitごと

- format/lint/type check。
- GoalSpec/schema/parser unit test。
- `cockpit.stair`, `m3_rl.joint_map`, observation builder。
- Branch Lのpolicy artifact/schema/hash compatibility、またはBranch SのSport API contract/state-machine compatibility。
- golden sensor/decision replay。
- arbiter state machineとduplicate/expiry test。
- Safety Supervisor unit/fault test。
- dependency/security scan。

### 11.2 nightly/5090

- Branch LのIsaac/MuJoCo smoke and deterministic regression。Branch Sは正式vendor simulatorがある場合だけ対応するcontract test。
- Branch Lのexact-stair simulation evaluation subset。Branch Sは過去runのtransaction replay、正式API stub、または利用可能な正式vendor simulatorだけを対象にする。
- ASR corpus benchmark。
- VLM/perception replay。
- backend別process kill、latency、packet fault integration test。LはLowCmd writer、SはSport gateway/serviceと各timeout遷移を対象にする。

### 11.3 hardware-tagged

- mock/defaultで実機actuation endpointを絶対に開かない。Branch LのLowCmd publisherとBranch SのSport gatewayの両方を対象にする。
- `--live` 一つだけでarmできない。operator lease、hardware key/confirmation、checklist IDを要求する。
- 吊り下げ、平地、階段testを別tag/pipelineにする。
- 自動CIから実機階段runを開始しない。

## 12. ネットワークとセキュリティ

### 12.1 topology

- GO2専用NIC/subnetを作る。
- CycloneDDSをそのinterfaceへ明示bind。
- operator LANとのbridge、IP forwarding、NATを無効化。
- DDS discovery/trafficをInternetや来客Wi-Fiへ出さない。
- firewall allowlistは必要host/portだけ。
- cockpitはHTTPS/WSS、authentication、CSRF/origin check、rate limit。
- actuator権限には単一operator leaseと短いexpiry。
- training/research userとlive-control service accountを分離。
- secrets/cert private keyをrepo/logへ入れない。

### 12.2 GO2固有の脆弱性を前提にする

2026-07-13時点のNVDには、次が掲載されている。

- [CVE-2026-27509](https://nvd.nist.gov/vuln/detail/CVE-2026-27509): Go2 firmware 1.1.7〜1.1.9および1.1.11 EDUで、特定DDS programming topicに認証/認可がなく、network-adjacentな攻撃者が任意Pythonを登録し得ると記載。
- [CVE-2026-27510](https://nvd.nist.gov/vuln/detail/CVE-2026-27510): firmware 1.1.7〜1.1.11とAndroid appのuser program経路で、完全性検証不足によりroot code executionへ至り得ると記載。
- [CVE-2025-60251](https://nvd.nist.gov/vuln/detail/CVE-2025-60251): 2025-09-20までのGo2等でhandshake secret検証の問題が記載。

これらは「特定versionだけ対処すれば安全」という意味ではない。実機firmwareとvendor mitigation/advisoryをUnitreeの[security response center](https://security.unitree.com/)で確認し、更新前後にSDK/topic/歩容を再検証する。既知脆弱性の有無にかかわらず、robot LANを敵対的networkから物理/論理分離する。

### 12.3 supply chain

- policy/model/downloadをhash allowlistする。
- mobile/community programを実験機へimportしない。
- arbitrary generated codeを実機runtimeで実行しない。
- pip/npm/apt/Git dependencyをpinし、SBOMを保存。
- release artifactに署名し、live hostでsourceから場当たりbuildしない。
- USB/log持出しにもmalware scanとchain of custodyを持つ。

## 13. 実機run前checklist

### 13.1 物理

- [ ] 階段IDと寸法revisionが一致し、各blockが固定されている。
- [ ] 表面が乾燥し、摩擦条件に変化がない。
- [ ] top/bottom landingと下降方式が承認済み。
- [ ] マット、上部安全索、頭上cable支持、立入禁止域がある。
- [ ] operatorと別のsafety observerがいる。
- [ ] remote/独立停止経路をその日の吊り下げ試験で確認した。
- [ ] 人、動物、障害物が試験区画にいない。

### 13.2 robot/sensor

- [ ] SKU/serial/firmware/batteryをmanifestに記録。
- [ ] joint/IMU/temperature/batteryがnormal。
- [ ] LiDAR/camera/LIO rate、freshness、frame、time offsetが合格。
- [ ] Branch Lは `ReleaseMode()` 後に必要topicが生きること、Branch SはSport service/stateが有効なことを確認。
- [ ] calibration期限内、mountに緩みなし。
- [ ] rear/downまたはfront/down landing coverageが方式に適合。

### 13.3 software/control

- [ ] clean/pinned build、backend contract/hash/schema一致。Branch Lではmodel hashも一致。
- [ ] terminal active balance、owner lease、arbiter、Safety Supervisorがready。
- [ ] 共通Sport navigation gatewayの`Move`/`StopMove` request/response/ack recorderと、stagingでのmode handoffが合格。
- [ ] ascent runはtop-hold warning/max、worst-case energy、reserve factor、thermal limit、timeout recovery、独立authorizationが署名済みmanifestと一致。
- [ ] Branch Lはsole LowCmd writerのrate/jitter/deadlineとmirrorが合格。
- [ ] Branch Sはexclusive Sport gateway、正式stair API、request/response/state recorder、`StopMove`・Damp・timeout・remote override contractが合格。
- [ ] logging容量とMCAP writeが正常。
- [ ] HTTPS/WSS、operator auth/lease、選択マイク（有線／AirPods）のdevice名・transport・入力levelが正常。
- [ ] training job、auto-update、不要network serviceが停止。
- [ ] shadow/dry runの意図とlive arm状態を二人で読み合わせた。

## 14. run中の役割

| 役割 | 責任 |
|---|---|
| Operator | command、state/readback監視。安全担当を兼ねない |
| Safety observer | robotと階段だけを見て独立停止/索を担当 |
| Test lead | checklist、run ID、Go/No-Go、条件変更を管理 |
| Data observer | recorder、clock、disk、event markerを監視 |

少人数なら兼務はできるが、robotへ命令する人と停止担当者は初期階段試験で分ける。

run中に条件を変えたら、同じrunで続けず停止して新manifestを作る。critical warningを「あと一回だけ」で無視しない。

## 15. run後

1. 選択backendで検証済みの安全姿勢/terminal balanceへ移し、対応している場合だけdisarmしてcommand ownerを解放。
2. raw/derived recordingを閉じてchecksum検証。
3. operatorとsafety observerが独立にevent/near missを記録。
4. automated KPI/result extractionを実行。
5. videoだけでsuccessを付けず、completion subconditionとlimitを確認。
6. artifact/manifestをimmutable storeへ同期。
7. failure/near missは次の実機run前にreplay testへ追加。
8. calibration/hardware inspectionのtriggerを判定。

## 16. incidentとpostmortem

次は転倒しなくてもincidentとして扱う。

- 索が荷重を受けた。
- safety observerが介入した。
- body/shankがedgeへ接触した。
- owner競合、stale command、deadline連続miss。
- unknown terrainへ足を出そうとした。
- wrong-direction/false voice activation。
- top/bottomの誤完了判定。
- logging欠落により原因が追えない。
- network/firmware/securityの異常。

postmortemにはtimeline、最初の異常、検出層、期待したbarrier、実際のbarrier、再現test、corrective action、owner、再試験gateを含める。個人の注意不足だけをroot causeにしない。

## 17. acceptance evidence pack

既知環境MVPの完了時に残すもの:

- hardware/API gate report。
- stair registryとcalibration report。
- architecture/threat/hazard review。
- model card、sim、sim2sim、HIL report。
- perception/voice/parser benchmark。
- staged real-run ledgerと全failure一覧。
- full Mission acceptance runのmanifest/bag/KPI。
- fault-injection report。
- firmware/security確認記録。
- known limitationsとNo-Go envelope。

これが揃って初めて、次の未知環境/VLA拡張でbaseline比較ができる。文書やlogの欠落を、成功動画で代替しない。
