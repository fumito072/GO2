# 07. Simulation、学習、sim-to-real

## 1. 先に結論

RTX 5090 があることは「新しい大規模モデルを必ず学習する」理由にはならない。最初に現在の Wave5 を、実寸 `10 cm × 4` の上り・後退下降・HOLD で再現評価する。

本書は主に**Branch L（学習LowCmd backend）**の計画である。Branch S（純正Sport stair API）は内部controllerを推測して再現せず、vendorが正式simulatorを提供する場合だけ補助的に使う。Branch Sの必須gateはvendor evidence/API contract、吊り下げ／平地、段階的実階段試験である。

```text
Wave5 exact-stair evaluation
  ├─ 合格 -> sensor realism、latency、safety、sim2simを追加して採用
  └─ 不合格
       ├─ 観測/座標/下降方向/実装bug -> 修正して再評価
       └─ policy能力不足 -> Isaac Labでperception-conditioned policyを再学習
```

world model や巨大な視覚 encoder は、単純な recurrent student との ablation で知覚欠落が支配的と分かってから比較する。

## 2. 推奨 stack

| 役割 | 第一候補 | 理由 |
|---|---|---|
| 大規模並列学習 | Isaac Lab + RSL-RL | GO2 rough locomotion基盤、RTX 5090を活用 |
| Unitree基盤 | `unitreerobotics/unitree_rl_lab` | 公式Go2対応、Isaac Lab→MuJoCo→実機の構造を持つ |
| sim2sim | MuJoCo / `unitree_mujoco` | deployment contractの独立検証 |
| 実機policy | TorchScriptまたはONNXの小型actor | 50 Hz deadlineと再現性 |
| LowCmd target transmitter | C++、既定500 Hz。200〜500 Hz候補 | latest `q/kp/kd` targetのZOH再送とwatchdog。代替rateは`T=1/f_tx` timing gate後。実motor servoは機体内 |
| 実験管理 | git commit＋lockfile＋run ledger＋artifact hash | 学習結果と実機runの追跡 |

公式 `unitree_rl_lab` は Go2 をサポートするとしているが、このリポジトリの staircase task は自作する必要がある。また、Isaac Sim/Lab、NVIDIA driver、PyTorch、Unitree RL Lab の最新版同士が常に互換とは限らない。5090/Blackwellで smoke test に通った組合せを commit/digestまで固定し、安易に途中更新しない。

## 3. 現行 Wave5 の評価を復元する

### 3.1 凍結するもの

- `policy/policy.pt` と ONNX の SHA-256。
- `policy/env.yaml` と `policy/policy_spec.json`。
- observation order/scale/clip。
- Isaac joint order、SDK joint order、gear/limit。
- action scale、default joint pose、Kp/Kd。
- policy rate、physics rate、LowCmd target TX rate、機体内servoとの契約、command filter。
- terrain generator、random seed、episode success definition。

現行 actor は235次元観測から12関節targetを50 Hzで出す。187点のheight scanは body周囲 `1.6 m × 1.0 m`、0.1 m gridである。offline shape testは通っているが、physics能力の証拠ではない。

### 3.2 exact-stair scene

Phase 0で測った階段から、Isaac LabとMuJoCoの両方に同じgeometryを生成する。

- 各段の実測高さと踏面。
- block間の継ぎ目、edge bevel/radius。
- 幅、top/bottom landing。
- collision meshとvisual meshを別管理。
- friction/restitutionは最初にnominal、その後分布化。
- body/foot/shank collisionを記録できるtag。

同じ `stair_registry.yaml` から両 simulator のassetを生成し、手作業の数値転記を避ける。

### 3.3 最初の評価matrix

| 軸 | 値の例 |
|---|---|
| 方向 | 上り、後退下降、前向き下降、HOLD |
| 段高 | 8, 9, 10, 11, 12 cm |
| 踏面 | 実測値、実測±許容差 |
| 初期yaw | 0°, ±3°, ±5°, ±10° |
| 横ずれ | 0, ±5, ±10 cm |
| 摩擦 | nominal、低、高 |
| command | 低/中速度、減速、途中STOP |
| 観測 | ideal、noise、dropout、stale |
| dynamics | nominal、motor/latency/payload randomization |

上りの正速度だけでなく、負の `lin_vel_x` が「平地後退」ではなく rear側の下降geometryと結び付くか確認する。inverted-pyramid terrainを学習したという設定だけでは、連続4段下降能力の証拠にならない。

### 3.4 再学習へ進む条件

次のどれかが残る場合に進む。

- nominal exact stairでも上り/下降 successが目標未達。
- 上りと下降の一方に明確な未学習gaitがある。
- real sensor相当のunknown/noiseを入れると性能が崩れる。
- STOP/HOLDへ安定遷移できない。
- body/shank edge contact、rear foot miss、joint/torque violationが残る。
- 失敗がobservation wiring、joint mapping、sim timingでは説明できない。

## 4. 再学習taskの設計

### 4.1 まず小さい構成

第一案は、privileged teacherとdeployable studentの二段構成である。

Teacher観測:

- simulator真値の局所height field。
- base state、全joint、foot contact/force。
- friction、motor strength、payload/CoM。
- applied disturbance、latency state。
- stair phase、local goal、true foothold/edge distance。

Student観測:

- 実機で得られるjoint position/velocity。
- IMU、推定base velocity。
- 過去actionと短いproprio history。
- causalなLiDAR/RGB-D由来height scanまたは小型latent。
- unknown/freshness/confidence。
- `ASCEND`, `DESCEND_BACKWARD`, `DESCEND_FORWARD`, `HOLD` mode。
- landingに対する相対local goal。

Action:

- 12関節position targetを50 Hz。
- action clamp/rate limit後、高周期LowCmd target transmitterへ渡す。companion側は`q/kp/kd` targetを送信し、実motor PDは機体内で行われる。
- 必要ならresidual actionにし、nominal gait/poseを基準にする。

MLP＋historyで足りなければGRU等のrecurrent encoderを追加する。point cloudを直接巨大Transformerへ入れる案は、height-map studentより定量的に優れる場合だけ採用する。

### 4.2 taskを分ける

一つの万能policyへ急いで統合しない。

1. flat locomotion/active hold。
2. ascent。
3. backward descent。
4. forward descent（必要な場合）。
5. transition/stop policyまたは共通multi-skill policy。

最初は別policyで原因を分離し、その後、共有encoder＋mode commandへ統合した方が安定する場合だけまとめる。

### 4.3 velocity trackingだけにしない

階段では固定 `vx` を追うだけだと、edgeを跨ぐphaseやlanding停止を学びにくい。次を局所goalにする。

- 次の安全なbody targetまたはlanding target。
- stair axis方向の進捗。
- target height/plane。
- top/bottomでゼロ速度・安定姿勢。

Missionがfootstepを厳密指定する必要はないが、policyは「何秒前進するか」ではなく、どこで完了するかを観測できるべきである。

## 5. terrain curriculum

初期案。値は実機同定後に更新する。

| Stage | terrain | 合格後の拡張 |
|---|---|---|
| 0 | 平地、stand、start/stop/hold | delay、push、sensor noise |
| 1 | 5〜8 cm単段 | approach yaw/lateral |
| 2 | 8〜12 cm、1〜2段 | friction/edge variation |
| 3 | 8〜12 cm、4段 | tread/width/step variation |
| 4 | 6〜14 cm、1〜8段 | payload、motor、dropout |
| 5 | OOD境界 | 成功ではなく安全拒否も学習/評価 |

現行envの5〜20 cm分布は広いが、target周辺の密度と下降episodeの割合が不明である。主評価分布8〜12 cmを十分にサンプルし、その外をrobustness/OOD rejection用に分ける。

## 6. domain randomization

| 分類 | randomizeするもの | 注意 |
|---|---|---|
| robot | mass、CoM、payload、link inertia | CAD不確かさと実payloadから決める |
| actuator | motor strength、Kp/Kd、joint offset、friction、backlash | 実機step responseで分布を校正 |
| contact | foot/step friction、restitution、edge radius | 非現実的な極端値で学習を壊さない |
| timing | observation/action delay、jitter、held action | real p95/p99を含める |
| communication | packet loss、duplicate、reorder、stale target | arbiter/publisher testと同条件 |
| state estimation | velocity/attitude bias、drift、reset | `ReleaseMode()`後の実測sourceを再現 |
| terrain | 段高、踏面、幅、blockずれ、yaw/lateral | targetとOODをlabel分離 |
| disturbance | body push、軽いfoot slip | curriculumで段階導入 |

単にrangeを広げるのではなく、実ログからposteriorを更新する。sim-to-real failureを見つけたら実機online RLをせず、原因をdistributionとsensor modelへ戻す。

## 7. sensor realism

これは最重要項目である。engine-level height truthをdeployment studentへ直接渡して成功しても、実機再現性は低い。

再現するもの:

- LiDARの実取り付け位置、垂直/水平FOV、body/legによるocclusion。
- scan rate、点密度、range noise、反射率によるdropout。
- motion distortionとdeskew error。
- LIO pose noise/time offset。
- temporal fusionによるage分布。
- map resolution、variance、unknown holes。
- RGB-Dのedge flying pixel、depth hole、露出/blur。
- camera/LiDAR外部パラメータ誤差。

studentへ与える187点height scanは、simulationの同じperception pipelineを通して生成する。理想raycastで得た値を使うoracle policyは、teacherまたは上限値の比較に限定する。

`unknown`を平地値へimputeする必要がある場合は、未知mask、age、historyを別入力に加えるか、policy外のsafety gateで該当footprintへの進入を止める。

## 8. rewardとtermination

### 8.1 reward候補

- local goal/landingへのprogress。
- target body heightとstair axisの追従。
- heading/lateral alignment。
- foot clearanceとfoothold center。
- top/bottomでのzero velocity、姿勢安定、4足landing。
- slip、edge hit、shank/body contactの罰則。
- roll/pitch、angular rate、vertical impactの抑制。
- joint position/velocity/torque limit。
- action rate、jerk、energyの抑制。
- perception dropout時の速度低下/安全hold。

reward shapingだけでcompletionを定義せず、evaluationでは明示的なgeometry/contact/dynamics条件を使う。

### 8.2 failure termination

- base/bodyが階段へ接触。
- roll/pitch/body heightがfall envelope外。
- footが階段側面/voidへ逸脱。
- joint/torque/temperature proxyがlimit超過。
- NaN/Infまたはaction contract違反。
- timeout/stall。
- 指定方向と逆へedgeを越える。

早すぎるterminationが危険状態からのrecovery学習を妨げないよう、teacher trainingとsafety evaluationのterminationを分けて管理する。

## 9. 学習と評価の分離

- training seedとevaluation seed/terrainを分ける。
- 少なくとも5 seedのpolicyを学習し、best single seedだけで判断しない。
- nominal、in-distribution random、boundary、OOD rejection、fault injectionを別reportにする。
- 上りと各下降方式を別々に各10,000 episode規模で評価する。
- success、fall、body contact、edge miss、joint violation、hold、energy、timeをすべて出す。
- checkpoint選択に実機test結果を直接使い続けない。

sim gate初期値:

- target distribution success ≥ 99%。
- MuJoCo sim2sim success ≥ 98%。
- NaN/Inf 0。
- critical joint/torque/body collision 0を目標とし、発生runを全件解析。
- top/bottom hold success ≥ 99%。
- OOD geometryの安全拒否 ≥ 99%。

閾値はengineering gateであり、論文値や安全認証ではない。

## 10. Isaac LabからMuJoCoへ

sim2simでは、actorだけをMuJoCoへ持ち込むのではなく、実機と同じdeployment codeを通す。

```text
MuJoCo sensors/state
  -> deployment observation builder
  -> frozen policy artifact
  -> action clamp/rate limiter
  -> LowCmd/PD-equivalent servo
  -> MuJoCo actuator
```

検証項目:

- joint orderとsign。
- quaternion/gravity/velocity frame。
- observation normalizationとclip。
- height-scan座標とattach frame。
- default pose/action scale。
- motor model、Kp/Kd、torque saturation。
- physics 500 Hz級、policy 50 Hz、LowCmd target送信周期、機体内servoというrate境界。
- start warm-up、active hold、STOP、owner transition。
- latency/jitter/stale target。

Isaacで高性能でもMuJoCoで崩れる場合、simulator-specific contact exploitや観測差を疑う。MuJoCoを学習分布へただ合わせ込む前に、どちらが実機ログに近いか同定する。

## 11. Hardware-in-the-loop

実階段の前に次を行う。

1. 実機controllerへsynthetic LowState/sensor replayを入力し、実際のpublisher出力を記録する。
2. 5090 process kill、GPU OOM、ASR負荷、network jitterを同時注入する。
3. actuator出力なしのshadow modeで実階段前/上端のpolicy actionを採取する。
4. 吊り下げでaction range、joint mapping、warm-up、remote inputを確認する。
5. elastic support/軽接地でcontact transitionを確認する。

training processとlive robot controlを同じOS sessionで同時実行しない。GPU training、driver reset、OOMが実機controlに影響しない物理/プロセス分離を持たせる。

## 12. 実機への段階移行

```text
吊り下げ非接地
 -> 吊り下げ軽接地
 -> 平地
 -> 5 cm単段
 -> 8 cm単段
 -> 10 cm単段
 -> 10 cm×2段
 -> 10 cm×3段
 -> 10 cm×4段
```

上りと下降はそれぞれ最初からこのladderを通す。各段階で低速度/低action envelopeから始め、logをsimへ戻す。成功runだけでなくnear miss、索による捕捉、operator abortをfailure labelとして残す。

実機online RLは行わない。real logから次を更新し、simulationで再学習する。

- actuator/timing/noise distribution。
- perception occlusion/dropout model。
- stair geometryとfriction。
- reward/terminationのmissing failure mode。
- completion detectorの閾値。

## 13. RTX 5090の使い方

5090の価値が高い順:

1. 数千environmentのIsaac Lab学習。
2. 複数seed/ablation/evaluation。
3. RGB-D/point-cloud encoder、VLM、ASR。
4. simulationのsensor rendering。
5. 小型50 Hz actorの推論。

小型actor推論自体は5090を必要としない。最終制御を外部GPUへ依存させず、onboard/companion computeでdeadlineを満たす形を目標にする。

環境運用:

- Ubuntu、NVIDIA driver、CUDA、Isaac Sim/Lab、PyTorch、RSL-RLをlock manifestに記録。
- container/image digestとhost driverを両方記録。
- 5090で最小Go2 envのcreate/reset/step/render/1,000-step deterministic smoke testをCI化。
- issueで報告される「特定Isaac Lab版＋Unitree RL Lab版」の不整合を避け、動作確認commitをpin。
- training中のtemperature、power、VRAM OOMをmonitorし、途中checkpointを保存。

## 14. artifact/model card

採用policyごとに次を必須にする。

```yaml
model_id: go2_stairs_wave6_seed3
artifact_sha256: ...
code_commit: ...
base_repo_commits:
  isaac_lab: ...
  unitree_rl_lab: ...
training:
  seeds: []
  total_steps: 0
  observation_schema_hash: ...
  action_schema_hash: ...
  terrain_distribution_hash: ...
deployment:
  policy_hz: 50
  lowcmd_tx_hz: 500  # target stream候補。機体内motor servo周期ではない
  kp_kd_hash: ...
validated_envelope:
  riser_height_m: [0.08, 0.12]
  tread_depth_m: []
  modes: [ASCEND, DESCEND_BACKWARD, HOLD]
known_failures: []
evaluation_report: ...
```

「最新policy.pt」のような可変名だけで実機を動かさない。Mission runは必ずartifact hashを記録する。

## 15. world modelを追加する判断

次の順でablationする。

1. proprio＋noisy height mapのMLP/history。
2. GRU等の短期memory。
3. confidence-aware fusion。
4. point-cloud latent。
5. world-model predictive latent。

world model採用条件:

- dropout/occlusion条件で再現性のある改善。
- nominal性能、latency、failure detectabilityを悪化させない。
- 5090だけでなく最終companion computeでdeadlineを満たす。
- hidden-state corruption/reset時のsafe behaviorが評価済み。

WMP-LocoやStairMasterの時間記憶・depth-noise設計は参考になるが、対象robot、階段、公開artifact、実機証拠の差を埋めずにそのまま採用しない。

## 16. 主要一次情報

- [Unitree RL Lab](https://github.com/unitreerobotics/unitree_rl_lab)
- [Unitree RL Gym](https://github.com/unitreerobotics/unitree_rl_gym)
- [Unitree MuJoCo](https://github.com/unitreerobotics/unitree_mujoco)
- [Isaac Lab](https://github.com/isaac-sim/IsaacLab)
- [Isaac Lab GO2 rough environment config](https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/go2/rough_env_cfg.py)
- [Learning robust perceptive locomotion in the wild](https://arxiv.org/abs/2201.08117)
- [DreamWaQ](https://arxiv.org/abs/2301.10602)
- [WMP-Loco](https://wmp-loco.github.io/)
- [StairMaster](https://arxiv.org/abs/2606.25765)

各software repositoryは更新されるため、上記URLではなく採用時のcommitをrun manifestへ固定する。
