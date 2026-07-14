# 安全設計・試験・評価計画

最終更新: 2026-07-13  
対象: Unitree Go2 X、約 10 cm × 4 段のブロック階段、自然言語による接近、有線マイクまたはAirPods音声による昇段・降段

> **重要**
> 本文書は研究デモ用プロトタイプの危険分析と試験計画であり、安全認証、法令適合、機能安全認証、製品安全保証ではない。本文書の KPI を満たしても、人のいる一般環境で無監視運転してよいことを意味しない。実機試験は、固定された試験設備、落下拘束、立入管理、専任の非常停止担当者の下で行う。

## 1. 結論

現行リポジトリは、Mock、dry-run、吊り下げ試験を進める土台としては有用である。一方、次の理由により、**4 段階段の自律昇降を LIVE で実施する判定は現時点で No-Go** とする。

1. 通常停止、姿勢保持、Damp、非常停止が同じ経路へ収束している。低レベル制御終了時に `kp=0, kd=2` を送る現在の動作は、階段上では「停止」ではなく「支持力を失って崩れる」危険を持つ。
2. 学習LowCmd branchでは、方策を 50 Hz で推論し、その同じ Python ループから 50 Hz で LowCmd を送る。Unitree の公式 Go2 低レベル例は 2 ms 周期で送信しており、方策周期とアクチュエータ送信周期を分離する必要がある。
3. 学習LowCmd branchにはLowCmdの単一所有者がなく、純正Sport branchにも正式な階段API、exclusive authority、停止・timeout契約の実証がない。両branchに共通して、独立 safety supervisor、command lease、モード切替の原子的な排他がない。
4. LowState 以外の鮮度を走行中に強制しておらず、未観測地形を平地として方策へ渡す。下降時の「見えない落差」を安全側に扱えていない。
5. 頂上・最下段の完了判定が、主に base 高さ、pitch、前方検出に依存する。最後脚がエッジを越えたこと、4 脚支持、着地点の平面性を確認していない。
6. VLM の判断待ちに最大 8 秒間速度を維持できる。0.3 m/s なら理論上 2.4 m 進み得るため、VLM と独立な高速 local guardian が必要である。
7. 音声認識は便利なタスク入力であり、安全入力ではない。有線マイク／AirPods、USB／cable／Bluetooth、ブラウザ、Whisperのどれかが失敗しても止められる物理経路が必要である。

LIVE へ進める最低条件は、共通P0改修、独立supervisor、階段上の能動balance、fail-closedな下降知覚に加え、選択backendに対応するauthority gateの合格である。Branch Lはsingle LowCmd ownerとGate 1L/2L、Branch Sはexclusive Sport authorityとGate 1S/2Sを通す。その後に共通Gate 3〜6へ進む。下降のLIVE解禁は昇段とは別のGate 7とし、昇段成功を下降の根拠にしてはならない。

---

## 2. 用語を分離する

「止まる」を 1 種類の API と考えない。少なくとも次の 5 種類を別々の状態、UI、ログイベント、試験項目にする。

| 機能 | 適用モード | 目的 | 関節支持 | 正常復帰 | 階段上での扱い |
|---|---|---|---|---|---|
| **HOLD** | 選択backend | 速度ゼロで姿勢を能動保持する | 維持する | 可能 | LはLowCmd、Sは検証済みformal balance state。通常完了と平地／安定stanceの通信断で選ぶ。gait途中はphase-safe boundary |
| **Controlled Stop** | 選択backend | backendを減速し、4脚支持へ整定してHOLDへ入る | 維持する | 可能 | 通常STOPの実体 |
| **StopMove** | Sport | 純正歩容の移動を停止し、純正バランス制御を残す | 純正制御が維持 | 可能 | Sport 有効時のみ使用 |
| **Damp** | Sport/Low-level | トルク・剛性を下げ、能動運動を抑える | 失う、または大幅に低下 | 要再起立 | 落下・崩落を招くため通常停止に使わない |
| **E-stop function** | 独立安全経路 | 現在の危険を最短で低減する | 危険状態に応じる | 手動 reset 必須 | Damp と同義ではない。落下拘束と状態依存戦略が必要 |

### 2.1 HOLD

HOLD は「最後の関節角を永久に再送する」だけではない。階段上では脚ごとの接地高さが違い、単純な固定角は姿勢を悪化させ得る。HOLD は次を満たす専用状態とする。

- 指令速度 `(vx, vy, wz) = (0, 0, 0)`。
- 4 脚接地、または安全に達成可能な静的支持多角形へ移行する。
- base roll/pitch と角速度を抑える地形適応立位を使う。
- 選択backendのactive-balance command/stateを継続する。Branch Lはsingle-owner LowCmd transmitter、Branch Sは検証済みvendor stand/`StopMove` stateを使う。
- HOLD 中も関節角、関節速度、推定トルク、温度、LowState freshness を監視する。
- 操作者が意図的に再開するか、事前に定義した時間・energy・thermal上限へ達するまで維持する。上限のない保持を要求しない。

とくに`TOP_HOLD`は無期限状態にしない。runごとに`top_hold_warning_s`と`max_top_hold_s`を固定し、初期研究値は30秒でwarning、60秒で上限とする。値は最大時間の負荷試験後にのみ延長する。上りpreflightでは、実測worst-caseの「上り＋最大TOP_HOLD＋下降または回収」energyに初期1.3倍のreserveを加え、motor温度余裕も満たすことを確認する。

上限までに音声命令が来ない、operator linkが失われる、battery/thermal warningが出た場合のrecoveryを上る前に一つ選ぶ。現段階のallowlistは、既にGate 7合格済みで幾何がfreshかつ別途明示承認された`VALIDATED_CONTROLLED_DESCENT`、またはsafety operatorによる`EXTERNAL_CAPTURE_AT_TOP`である。どちらも成立しないrunでは上りを開始しない。上限到達時にblind descent、Damp、無期限HOLDのいずれへも自動で逃げない。

### 2.2 通常 STOP

通常 STOP は次の順序で行う。

1. 新しい移動コマンドを arbiter で拒否する。
2. 選択backendへ停止要求を出す。Branch Lは方策へ速度ゼロ、Branch Sは正式な`StopMove`/terminal-balance APIを使う。
3. body速度と角速度が閾値以下になるまで、選択backendのactive balanceを継続する。
4. 安定した接地を確認する。
5. HOLD へ遷移する。

「速度ゼロを 1 パケット送って SIGINT」「SIGINT 後に Damp」は通常 STOP ではない。

### 2.3 StopMove

`SportClient.StopMove()` と `SportClient.Damp()` は Unitree SDK 上でも別 API である。StopMove は Sport mode が有効なときの通常停止に用いる。Motion service を release した Low-level 中に StopMove を呼んでも、安全効果を前提にしてはならない。

### 2.4 Damp

Damp は「緊急だから常に安全」ではない。Go2 は約 15 kg あり、階段上で支持力を失えば、機体の落下、横転、挟み込み、階段破損を起こし得る。Damp を選ぶ典型条件は以下とする。

- 制御出力が発散、NaN、関節誤配線などで、能動保持の継続自体が危険。
- 平地で人や物への継続衝突を止める必要があり、落下危険よりトルク除去が安全。
- 機体が既に転倒し、脚の暴れを止める必要がある。
- 落下拘束が機体重量を確実に支持している。

現在の `kp=0, kd=2` は 2 秒かけた漸減ではなく、最初のフレームから位置剛性をゼロにする。名称、UI 説明、ログを実態に合わせ、吊り下げ状態で挙動を実測する。

### 2.5 E-stop function

E-stop はブラウザの赤いボタン名ではなく、独立入力、優先権、停止戦略、reset、診断を含む一つの安全機能である。少なくとも以下を満たす。

- VLM、Whisper、ブラウザ、WebSocket、Wi-Fi、方策プロセスと独立した物理入力を持つ。
- すべての通常コマンドより優先し、押下中は再始動できない。
- reset しても自動再始動しない。再 ARM とモード確認を別操作で要求する。
- 入力線断、受信機断、heartbeat loss を検出可能にする。
- テストごとに入力から最初の検証済み安全応答までの遅延を記録する。software経路では安全frame、独立hardware/system経路ではactuation/物理応答を外部計測する。
- 階段上では、落下拘束を前提として、状態が健全なら短い Controlled Stop → HOLD、制御出力異常なら Damp、という危険最小化戦略を事前に定義する。

ISO 13850 は emergency stop function の設計原則を扱うが、本プロジェクトは同規格への適合を主張しない。市販リモコンのボタン、ソフトウェア Damp、電源断を、検証なしに「認証済み E-stop」と呼ばない。

現時点で、このGo2 X用に安全ratedと確認済みのE-stop装置は本リポジトリ内に存在しない。Phase 0でUnitree/販売店と適格な機械・電気安全担当者により、利用可能な独立停止入力、受信機断時挙動、駆動エネルギー遮断、LowCmd中のリモコン経路、必要な外部安全回路を同定する。適切な装置と状態別停止戦略を確立できない限り、本文中の「物理E-stop」は要求機能の名称にすぎず、LIVE試験はNo-Goである。落下拘束は被害軽減策であり、E-stopそのものではない。

---

## 3. 現行実装の監査結果

| 箇所 | 現状 | 安全上の問題 | 必須対応 |
|---|---|---|---|
| `m3_rl/rl_stair_controller.py:285-292` | 終了時に `kp=0, kd=2` を 2 秒送る | 正常完了、Ctrl+C、watchdog fault がすべて脱力へ収束する | Controlled Stop、HOLD、Damp を別状態にする |
| `cockpit/rl_bridge.py:164-207` | STOP は SIGINT、5 秒後 SIGKILL、別 publisher で emergency damp | 通常 STOP が Damp。最悪 5 秒待つ。LowCmd owner が切り替わる | owner 内の同期 STOP/DAMP RPC に置換する |
| `cockpit/server.py:393-400, 525-531` | `stop` と `damp` の双方が autonomy abort を通る | Low-level 中の赤い DAMP が即時でなく、最後に Sport Damp を呼ぶ可能性 | E-stop 専用経路を Web server から分離する |
| `cockpit/static/app.js:461-470` | STOP が `rl_stop` と `action:stop` を連送 | 重複停止とモード不一致。結果順序も保証されない | 1 個の安全状態遷移要求にする |
| `common/config.py:42-49` | 方策・LowCmd とも 50 Hz、state watchdog 表記 40 ms | 実際はコードで最低 100 ms。方策遅延と送信遅延を区別していない | policy 50 Hz / TX 500 Hz / supervisor 200 Hz へ分離する |
| `common/go2_iface.py:187-218` | 誰でも callable を取得して LowCmd を送信可能 | 単一所有権、lease、deadline、mode interlock がない | LowCmd actuator server だけが DDS publisher を所有する |
| `m3_rl/rl_stair_controller.py:225-282` | 推論、観測作成、DDS Write、sleep が 1 ループ | 推論 stall がそのままモータ command gap になる | 最新 target の double buffer と独立 TX loop を使う |
| `common/safety.py:37-68` | watchdog callback の例外を黙って破棄 | safety action 失敗を検出できない。callback は LowCmd を直接止めない | latch、監査ログ、owner への同期 fault、二次経路を持つ |
| `m3_rl/obs_builder.py:35-52` | finite、長さ、範囲検証がない | NaN/Inf や異常次元を方策・LowCmd へ伝播できる | 入出力全要素の finite/range/schema gate を追加する |
| `cockpit/server.py:111-124` | 未観測 height cell を足元と同高で埋める | 下り段差や穴が「平地」に変換される | unknown mask/confidence を保持し、進行方向 ROI の unknown は No-Go |
| `cockpit/rl_bridge.py:87-91` | height scan 全体の平均 cover 35% で許可 | 後方だけ見えて前方が空でも通る | 脚別・進行方向・停止距離 ROI の coverage gate にする |
| `cockpit/stair_task.py:158-172` | 実行中 guard は LowState と姿勢中心 | cloud、odom、camera、map の stale を検出しない | センサごとの monotonic age と同期誤差を監視する |
| `cockpit/stair_task.py:405-419` | base z、pitch、前方平坦で頂上判定 | センサ欠損で `none` になっても完了し、後脚がエッジ上でも止まる | 脚接地、エッジ通過、landing clearance を加える |
| `cockpit/mission.py:30, 270-275` | VLM 待ち中も最大 8 秒移動を維持 | 低速でも大きな open-loop 距離になる | 20〜50 Hz local guardian と短い trajectory primitive に制限する |
| `cockpit/voice.py` | 一般的な前後移動だけを即時実行 | 「登る」「降りる」の安全状態機械、read-back、方向確認がない | task grammar と二段階 ARM を追加する |
| `policy/env.yaml` | inverted stairs と負速度は含む | 下降方向別の成功率、センサ dropout、制御遅延、motor gain/COM DR が示されない | ascent/forward-descent/backward-descent を別集計し不足 DR を追加する |

Unitree の公式 Python Go2 stand example は `dt=0.002`、すなわち 500 Hz で LowCmd を再送している。公式 ROS 2 low-level example も 5 ms、200 Hz の timer を用いる。50 Hz の**方策周期**自体は学習契約と一致するが、50 Hz の**モータ送信周期**を正当化するものではない。

---

## 4. 目標安全アーキテクチャ

```text
AirPods / UI / 自然言語
          |
          v
  Task Manager / VLM  ---------  goal, intent only
          |
          v
  Local Planner / Stair FSM --- bounded velocity + phase
          |
          v
  Command Arbiter <------------- operator STOP
          ^
          |
Independent Safety Supervisor <--- status mirror from physical input/path
          |        ^
          |        +--- LowState, contacts, lidar age, odom age,
          |             command age, process heartbeat, battery
          v
  Exclusive Actuation Gateway / mode generation
          |
          +--> NAV phase（S/L共通）: Sport velocity executor + verified StopMove
          |
          +--> STAIR phase Branch S: formal Sport stair API + verified StopMove/timeout
          |
          +--> STAIR phase Branch L: Sport inactive ack後にsingle-owner LowCmd
                                    policy 50 Hz / TX default 500 Hz ZOH
          |
          v
        Go2  （NAV/STAIR phaseとS/L backendを同時にenableしない）

physical E-stop input
          |
          v
verified independent stop path  - - -Phase 0で同定・実証できるまでLIVE No-Go- - -> Go2
```

### 4.1 独立 Safety Supervisor

Supervisor は VLM、音声認識、UI server、方策と同じ event loop や Python thread に置かない。最低でも独立プロセスとし、exclusive actuation gatewayおよび選択backendとの通信にはheartbeatと単調時刻を使う。Branch LではLowCmd actuator server、Branch SではSport gateway/API stateを監視する。実運用を目指すなら、低レベル送信と supervisor は非リアルタイムな Wi-Fi/ブラウザ経路から分離し、Go2 側または有線接続された real-time Linux 側へ置く。RTX 5090 は学習と大規模評価に用い、ネットワーク越しの推論を関節安全ループへ直接入れない。

Supervisor が監視する最低項目:

- LowState age、sequence/lost、CRC、IMU validity。
- policy target age、TX deadline miss、DDS write error。
- q、dq、`tau_est`、q error、motor temperature、battery voltage/current。
- roll/pitch、角速度、base 高さ変化、body contact、脚 contact consistency。
- LiDAR cloud age、odom age/jump、camera age、地図 ROI coverage。
- Sport/Low-level mode、owner generation、command source、lease expiry。
- UI、AirPods、VLM、planner、policy のheartbeat。ただしこれらの消失だけでDampせず、fault sourceとgait phaseに対応するsafe-boundary policyを選ぶ。

Supervisor の fault は latch する。同じ正常パケットが 1 個来ただけで自動復帰せず、健全性を一定時間確認し、操作者 reset と再 ARM を必要とする。

### 4.2 Command arbitration

各command requestに次を必須とする。

```text
source_id, goal_id, actuation_request_id, sender_timestamp,
sequence, expires_after_ms, requested_mode, vx, vy, wz, phase, policy_hash
```

`source_priority` は送信者が指定するfieldにしない。arbiterがauthenticated channel、operator lease、process identityからserver-sideで `trusted_source_id`, priority, accepted_monotonic_timestampを付与する。payload中の自称priority/sourceは無視する。

優先順位は次の通りとする。

1. 物理 E-stop / supervisor hard fault
2. 操作者 STOP / DISARM
3. supervisor soft fault / sensor stale
4. HOLD / Controlled Stop
5. 有線手動操作
6. 階段 state machine
7. navigation local planner
8. VLM が提案した goal

原則:

- 低優先 command は、高優先 command の latch を解除できない。
- command expiry 後はゼロを推測せず、明示的に Controlled Stop へ遷移する。
- wall-clock の `time.time()` ではなく monotonic time を安全判定に使う。
- VLM は直接 `vx` を長時間所有せず、短い primitive または goal を local planner に渡す。
- Sport と Low-level は同時に command を受けない。mode transition は `request -> acknowledged inactive -> owner generation change -> enable` の handshake にする。
- WebSocket 接続が複数あっても controller token は 1 個だけにする。観測 client は command を送れない。

### 4.3 Branch L: LowCmd publisher の設計

LowCmd DDS publisher はシステム全体で1プロセスだけが所有する。`RlController._emergency_damp()` のようにfault時に新しいpublisherを競合起動する設計は廃止し、ownerが生存しているfaultでは既存ownerが安全フレームを送る。

owner process自身が死んだ場合、外部SupervisorはLowCmdを送れない。第2writerを自動起動すると旧writerとの競合raceを作るため、transactional fencingが実証されるまで行わない。対象firmwareがLowCmd途絶時に何をするかを吊り下げ状態で実測し、vendor確認済みの独立停止回路/受信機と合わせてbounded safe responseを定義する。安全なtimeout応答を確立できなければ、落下拘束があっても階段LIVEはNo-Goである。

推奨 state machine:

```text
DISABLED -> PREFLIGHT -> RAMP_IN -> RUN
                              |      |
                              v      v
                            HOLD <- SOFT_STOP
                              |
                              v
                         RAMP_OUT / STAND_DOWN

any state -- invalid output / severe fault --> DAMP_LATCHED
any state -- physical E-stop ---------------> ESTOP_LATCHED
```

実装要件:

- Policy inference は 50 Hz。最新 target と時刻を lock-free/double buffer に書く。
- TXは既定500 Hz。200〜500 Hzを候補範囲とするが、rate `f_tx`、周期`T=1/f_tx`、p99 jitter budgetを署名済みconfigへ固定し、Gate 2Lとhazard review後にのみ500 Hz以外を選ぶ。
- TX は50 Hzの最新targetを原則ZOHで再送する。target間の補間やfilterは学習時のaction contractを変えるため、同じ処理をIsaac/MuJoCo/実機へ入れた比較試験に合格した場合だけ採用する。
- action、q target、gain、CRC 対象全フィールドを送信前に検証する。
- `NaN`、`Inf`、配列長不一致、policy hash 不一致は 1 フレームも motor へ出さない。
- q limit だけでなく q-target slew、q error、dq、推定 torque、temperature を制限する。
- target stale時は最後のgait targetを無期限再送しない。stale thresholdは即時trigger/latchとし、平地／既に安定stanceだけzero target→HOLD、階段途中は署名済みphase bound内の検証済みsafe-boundary反応へ遷移する。
- すべての未使用 motor field は Unitree の公式定数/推奨値で初期化する。
- deadline miss と実送信 interval を flight log に残す。

初期 fault policy は以下とし、吊り下げ試験で調整する。

| 条件 | 初期反応 | 備考 |
|---|---|---|
| policy target age > 40 ms | fault trigger/latch、phase別safe-boundary request | 方策2周期欠落相当。平地／安定stanceだけzero→SOFT_STOP |
| policy target age > 100 ms | fault escalation | 平地／安定stanceはHOLD、階段途中は署名済みphase bound。無期限ZOH禁止 |
| LowState age > 40 ms | hard fault | 現行の実質 100 ms と一致させない |
| output に NaN/Inf | DAMP/ESTOP policy | 能動制御の継続を禁止 |
| LiDAR/odom stale | 新規接近・昇降禁止、phase別safe-boundary | 平地／安定stanceはControlled Stop→HOLD。階段途中は一律停止しない |
| UI/AirPods/operator link disconnect | 新規goal禁止、phase別safe boundary | 平地はControlled Stop→HOLD。階段途中は一律停止しない |
| VLM unavailable/timeout | 新しいproposalを無効化 | required-sensor setに含むrunだけmotion inhibit。advisoryなら幾何gateを優先 |
| roll/pitch envelope 超過 | phase 別 recovery または hard fault | 全 phase 共通 1 閾値にしない |
| torque/temp/battery limit | phase別safe boundary。TOP_HOLDは事前選択recoveryを前倒し | メーカー値、energy budget、実測で確定 |

閾値は提案初期値であり、安全認証値ではない。実機の配信周期、FW、機体個体、荷重を測り、hazard review で確定する。

階段途中のsafe boundaryは、現在の支持状態でHOLD、次の検証済み支持姿勢までのbounded continuation、またはlandingまでの完遂から、gait phaseごとのsimulation/低段fault injectionで選ぶ。音声/HMI喪失と、LowState/policy/actuator喪失を同じfaultへまとめない。最大時間・距離・段数を持たないblind continuationは禁止する。

### 4.4 Branch S: 純正Sport階段歩容のauthority設計

Branch SではLowCmd publisherや500 Hz policy TXを新設しない。代わりに、正式に提供される階段skill APIだけを`StairSkillBackend`から呼び、通常の`SportClient.Move()`を階段skillと取り違えない。次をvendor資料と吊り下げ／平地実測で成立させる。

- stair start、direction、progress/state、normal complete、`StopMove`、Damp、timeout、remote overrideの正式な意味と応答上限。
- Sport serviceのcommand authorityを1つのgateway/leaseに限定し、UI、navigation、voiceが並行に直接Sport RPCを呼べないこと。
- skill開始前にLowCmd ownerがinactiveであるackを取り、終了・fault後のmode generationが一致すること。
- 頂上・最下部で純正balanceを維持するterminal state。`StopMove`が階段上で能動支持を残すことを外部姿勢・接地・速度で検証する。
- Sport service/通信/process断時のfirmware behaviorと独立停止経路のbounded response。これを確認できなければBranch SもLIVE No-Goとする。

純正内部制御をIsaac/MuJoCoに再現できないこと自体は失格理由にしない。ただしブラックボックスであることは安全免除ではなく、外部guardian、StairModel、completion detector、段階的実階段試験をより厳密に要求する。

---

## 5. 階段上の停止・完了条件

### 5.1 頂上停止

base z が上がっただけでは不十分。次を同時に満たすまで「頂上完了」にしない。

- 4 脚の接地が確認できる。
- 各接地点の地面高さが同一 landing plane 内で許容差以下。
- 最後脚が最上段 edge を越え、edge から安全余裕を持つ。
- base linear speed、yaw rate、roll/pitch rate が閾値以下。
- top landing の前方 free space が停止距離と機体 footprint を満たす。
- LiDAR/odom が fresh で、`none` が「未観測」ではなく「観測済み平坦」を意味する。
- 条件が連続 1.0 秒以上成立する。

初期 KPI は、全脚接地、`|v_xy| < 0.03 m/s`、`|yaw_rate| < 0.05 rad/s`、landing 高さ差 2 cm 以下、edge clearance 10 cm 以上とする。実測で保守側に更新する。

### 5.2 最下段停止

下降完了は次で判定する。

- 全脚が最下段下の床へ移った。
- 最後脚が最下 edge を越え、安全余裕を持つ。
- 4 脚下の局所平面が観測済みである。
- 前後に次の drop がない。
- base 高さが平地 nominal に戻り、姿勢・速度が整定した。
- 条件が連続 1.0 秒以上成立する。

### 5.3 中間段で STOP が来た場合

通常STOP requestは即座に受理・latchするが、関節を即freezeしない。Branch Lはpolicyへzero-velocity target、Branch Sはformal stop/terminal-balance requestを出す。物理動作は、現在stanceのHOLD、次の検証済み支持姿勢までのbounded continuation、landing完遂のいずれかを、署名済みphase別`max_safe_boundary_time_s/distance_m/step_count`以内で行う。Branch Lは学習と全phase stop injection、Branch Sはformal契約と低段の全phase fault injectionで実証する。未定義phaseまたはbound超過が1件でもあればNo-Goである。

---

## 6. 下降方向の選択

上りで成功した方策に負の `vx` を与えるだけで、後退下降が安全になるとは限らない。現行 `policy/env.yaml` には inverted stairs と負速度が含まれるが、方向別評価、実センサ欠損、停止注入の証拠がない。したがって両方向を別 controller/skill として評価する。

| 判断軸 | 前向き下降 | 後退下降 |
|---|---|---|
| 現在の前面 camera | 下段を見られる可能性が高い | 進行方向が camera の背面になる |
| L1 LiDAR | 下向き近傍 ROI の実測が必要 | rear/down ROI と胴体・脚 occlusion の実測が必要 |
| 転倒形態 | 頭側から落ちる危険 | 尾側から落ち、前脚で上段を保持できる場合がある |
| 方策 | 前向き下降を明示学習・評価 | 後退下降を明示学習・評価。負速度だけでは不可 |
| 頂上での準備 | 前進上り後は180°回頭が必要 | 上り時の向きを保てるため回頭不要 |
| 人による監視 | 進行先と camera が一致 | 操作者から脚接地が見えにくくなる場合がある |

選択規則:

1. 進行方向の「次の 2 踏面」と最下 landing を、停止距離を含めて観測できる方向だけを候補にする。
2. その方向を含む訓練分布と、方向別の SIL/HIL/実機 KPI があることを必須にする。
3. top landing に安全な回頭空間がなければ、前向き下降のための 180°回頭は禁止する。
4. payload、重心、摩擦、踏面奥行、段鼻形状ごとに成功率を分ける。
5. どちらも観測条件を満たさなければ下降を拒否する。

**MVP の第一研究候補は後退下降**とする。前進上り後に同じ向きを保てるため、狭い top landing で危険な180°回頭を避けられる。ただしこれは rear/down sensing が十分という意味ではない。後方斜め下向き depth sensor を追加するか、L1 の rear ROI が全着地点を十分な密度で観測できることをデータで示した場合だけ解禁する。rear/down coverage が不足し、約1 m級の安全な回頭面を確保できる場合は、回頭後の前向き下降を比較候補にする。

---

## 7. Hazard register

リスク等級は初期スクリーニングであり、正式な ISO リスク評価ではない。

- **C**: Catastrophic。人への重傷、機体落下・激突があり得る。未対策なら LIVE 不可。
- **H**: High。転倒、挟み込み、重大破損、制御喪失があり得る。
- **M**: Medium。ミッション失敗や軽微な接触が中心だが複合故障で悪化し得る。

| ID | 危険事象・原因 | 影響 | 初期 | 防護策 | 検証 / Gate |
|---|---|---|---:|---|---|
| H-01 | 通常 STOP が Damp へ直結 | 階段上で崩落 | C | Controlled Stop/HOLD と Damp を分離 | 全 phase stop injection 0 転倒 |
| H-02 | 赤い DAMP が WebSocket/SIGINT/5 秒 wait に依存 | 緊急停止遅延 | C | 独立物理入力と owner 内安全 state | latency fault injection |
| H-03 | Branch LでpolicyとLowCmdが同じ50 Hz Python loop | command gap、関節跳躍 | C | 50 Hz policy / default 500 Hz TXを分離 | Gate 2Lの60分timing test |
| H-04 | 複数LowCmd publisher、複数Sport command経路、Sport/Low-level競合 | 相反 command | C | backend別single authority、generation lease、mode handshake | Gate 2L/2S competing-source injection |
| H-05 | 未観測 height を平地補完 | drop へ進入 | C | unknown mask、ROI fail-closed | map hole/drop test |
| H-06 | stale cloud/odom/camera を使用 | edge 誤位置、false complete | H | sensor age gate、time sync | delay/replay injection |
| H-07 | VLM 待ち中 8 秒 open-loop | 階段・人へ衝突 | C | local guardian、短 primitive | VLM 90 秒 stall test |
| H-08 | 下降方向の sensing/policy 未証明 | 段外し、頭/尾落下 | C | 方向別 policy と KPI | Gate 7 を独立実施 |
| H-09 | base z/pitch 中心の完了判定 | 後脚が edge 上で停止 | H | contact、edge clearance、landing plane | top/bottom false-complete 0/100 |
| H-10 | ASR 誤認識・環境音・Bluetooth 断 | 意図しない開始、停止不能 | C | push-to-talk、read-back、二段 ARM、物理 STOP | nuisance speech 1000 件 |
| H-11 | NaN、dq、tau、temperature 未監視 | 発散、motor damage | C | finite/range/thermal/power gate | malformed obs/action injection |
| H-12 | mode release/restore の途中失敗 | 支持喪失、二重制御 | H | transactional handoff、ack、timeout latch | 各遷移点 kill test |
| H-13 | 階段ブロックが動く、滑る、狭い | 踏面崩壊・横転 | C | 固定、anti-slip、寸法検査、belay | 毎 run preflight |
| H-14 | 人が catch しようと接近 | 脚・階段との挟み込み | C | 人は catch しない。rated harness と barrier | zone audit |
| H-15 | battery低下、電圧sag、過熱、無期限TOP_HOLD | torque低下、上端で回収不能、突然停止 | H | mission energy reserve、hold上限、warning、事前選択recovery | max-hold/load/sag test |
| H-16 | firmware/OTA、policy/config 差替え | 検証条件が無効 | H | version/hash pin、change control | manifest check |
| H-17 | odom z drift・extrinsic 誤差 | 段高と完了の誤判定 | H | calibration、contact fusion、drift bound | known fixture test |
| H-18 | 1 個の正常 packet で fault 自動解除 | fault chatter、突然再始動 | H | latched fault、manual reset、dwell | intermittent dropout test |
| H-19 | network/command spoof、複数 UI client | 無権限 command | C | isolated LAN、auth、controller token、allowlist | penetration/fuzz test |
| H-20 | top landing が短い | 止まり切れず反対側へ落下 | C | footprint + stop distance gate | geometry preflight |

Hazard register は固定資料ではない。各 incident、near miss、E-stop、足滑り、予期しない body contact の後に更新する。C/H の未解決項目が 1 件でもあれば、対応する LIVE Gate は No-Go とする。

---

## 8. 試験設備と運用ルール

### 8.1 試験設備

- 初期階段は幅 1.2 m 以上、踏面 0.35 m 以上、段高を 5 cm から開始し 10 cm へ上げる。
- 4 段階段の top landing は少なくとも機体全長、停止距離、0.2 m の余裕を合算して確保する。事前に実測して数値を run manifest に記録する。
- ブロックは床へ機械固定し、run 中にずれない。段鼻、摩擦係数、表面材を記録する。
- overhead rail と機体重量・動的荷重に適合した harness を使う。harness は通常歩容を持ち上げず、落下だけを制限する長さに調整する。
- 階段周囲は barrier で立入禁止にし、人は機体の上下流・落下方向に立たない。
- 外部 camera を最低 2 方向に置き、脚接地と全体姿勢を同期記録する。

### 8.2 人員

最低 2 名とする。

- **Test director**: run 開始、チェックリスト、ログ、Go/No-Go 判定。
- **Safety operator**: 物理 E-stop だけを担当し、PC 操作や撮影を兼務しない。

人が機体を手で受け止める運用は禁止する。落下は harness に受けさせる。

### 8.3 run 前チェック

- 機体型番、FW、SDK、git commit、config hashと、選択backend evidence（Branch Lはpolicy SHA-256、Branch Sはformal API contract hash）を記録。
- battery、motor temperature、脚・foot pad、harness、階段固定を確認。
- 実測worst-caseの上り＋最大TOP_HOLD＋下降/回収energy、初期1.3倍reserve、hold warning/上限、timeout recoveryをrun manifestと照合する。
- LowState、LiDAR、odom、camera の rate と age を 30 秒確認。
- command owner が 1 個であること、Sport/Low-level mode を確認。
- STOP、HOLD、Damp、物理 E-stop を吊り下げまたは平地で 1 回ずつ確認。
- test zone と top/bottom landing の無人を確認。
- run ID と想定 fault injection を読み上げる。予定外 injection を行わない。

---

## 9. 段階試験と Go/No-Go Gate

上位Gateの合格は下位Gateを省略する根拠にならない。controller、選択backend artifact/contract、FW、関節map、gain、sensor extrinsic、階段材質を変更した場合は、change impact analysisで戻るGateを決める。

### Gate 0 — 静的監査・単体試験

対象: ロボット非接続。

必須:

- joint order、sign、limit、action scale を machine-readable manifest から検証。
- obs/action の dimension、finite、range property test。
- command arbiter priority/expiry/latch の網羅テスト。
- HOLD/STOP/DAMP/E-stop state transition table の全 edge をテスト。
- ascent/forward descent/backward descent の command が混同されない。
- safety code 内で例外を黙って捨てない。

Go: safety test 全件 pass、C/H hazard に owner と test が割り当て済み。  
No-Go: safety callback の例外握りつぶし、Branch Lの複数LowCmd writer、Branch Sの複数Sport command経路、未定義transitionが残る。

### Gate 1L — Branch L: SIL / 大規模シミュレーション（RTX 5090）

対象: Isaac Lab。

訓練・評価分布:

- 段高 0.08〜0.12 m、踏面 0.25〜0.45 m、3〜6 段。
- 段高・踏面の段ごとのばらつき、斜行、段鼻、摩擦 0.4〜1.2。
- payload、COM、motor strength/gain、latency、action delay、IMU bias。
- height map noise だけでなく dropout、unknown patch、時刻ずれ、odom drift。
- ascent、forward descent、backward descent を別集計。
- 各 gait phase への STOP、sensor loss、policy stall をランダム注入。

Go:

- 各方向 10,000 episode 以上。
- mission success 99% 以上。
- uncontrolled fall 0.1% 未満、base hard contact 0.1% 未満。
- STOP injection 1,000 回/方向で転倒 0、HOLD 30 秒維持 100%。
- unseen 組合せでも KPI を満たす。

No-Go: direction を合算してのみ成績を示す、unknown を平地扱いする、stop injection を評価していない。

### Gate 1S — Branch S: vendor evidence / API contract

対象: 純正Sport階段歩容。vendor simulatorが提供される場合はSILも行うが、内部controllerをIsaac/MuJoCoへ模倣することは必須にしない。

Go:

- 対象Go2 Xのserial/FWで、階段開始・方向・状態・正常完了・停止・timeout・remote overrideの正式APIと適用条件が文書化される。
- 一般の`Move()`ではなく、正式なstair skillであることを確認する。
- vendor test evidenceの階段寸法、方向、failure modeが本試験条件へ適用可能かgap analysisを完了する。
- 不明な停止挙動、非公開topicへの依存、無保証のAPI推測が0件。

### Gate 2L — Branch L: HIL / LowCmd timing（脚を床から離す）

対象: rated harness で完全支持し、脚が設備・人へ届かない状態。

必須:

- 既定500 Hz TXを60分。代替rateを採る場合は同じ60分試験をそのrateでやり直し、signed timing configを更新する。
- policy stall、process kill、NaN action、LowState dropout、DDS error を注入。
- 全 joint の sign/order を低 gain・小振幅で一本ずつ確認。
- RAMP_IN、HOLD、Controlled Stop、Damp、manual disarm後のowner restartを確認。
- Phase 0で同定した独立停止候補のend-to-end latencyをオシロスコープまたは同期logで測る。未同定ならGate 2LはNo-Go。

Go:

- configured rateを`f_tx`、`T=1/f_tx`として、TX interval p99 ≤ `T + 1 ms`（既定500 Hzでは3 ms）、max ≤ 10 ms、20 ms超gap 0回/60分。1 ms jitter budgetを変える場合は再hazard reviewを要する。
- policy inference p99 ≤ 18 ms、deadline miss 時は 40 ms 以内に SOFT_STOP。
- malformed action が motor frame に 1 件も流れない。
- policy process killでは生存中のactuator ownerが安全状態へ遷移する。
- actuator owner killではfirmware timeout/独立hardware pathのbounded responseを実測する。確認できなければNo-Go。

### Gate 2S — Branch S: 吊り下げ／平地のauthority・停止試験

対象: rated harnessで完全支持した試験と、立入禁止・落下拘束下の平地試験。段差にはまだ接地しない。

必須:

- exclusive Sport gateway以外からの同時commandを拒否し、navigation/voice/UIのlease競合を注入する。
- stair skill start/cancel、`StopMove`、normal complete、Damp、remote override、Sport service/process/通信断を各遷移点で試す。
- LowCmdがinactiveであること、mode release/restoreのack、generation mismatch時の開始拒否を確認する。
- `StopMove`後の能動balance、再始動禁止、独立停止のend-to-end responseを外部計測する。

Go: すべての要求がbounded timeで一意の状態へ遷移し、競合command、予期しない再歩行、支持喪失が0件。service/firmware断時のbounded safe responseを確立できなければNo-Go。

### Gate 3 — 平地・低速・落下拘束

必須:

- start/stop/terminal active balanceを100回。Branch LはHOLD、Branch Sは検証済み`StopMove`/純正stand stateを使う。
- 手動速度は 0.1 m/s から開始。
- 各種通信断、UI 断、AirPods 断を注入。
- 30秒terminal active balance、StandDown、backendに対応した復帰を反復。

Go:

- 転倒、body contact、予期しない脚跳ね 0/100。
- STOP から 1.0 秒以内に `|v_xy| < 0.03 m/s`。
- operator UI/lease断は平地でControlled Stop→HOLD。AirPods断は新規voice goalを拒否し、VLM断はproposalを無効化する。いずれもDampへ直結しない。
- 物理 E-stop は仕様どおり latch し、reset だけで再歩行しない。

### Gate 4 — 単段上り 5 cm → 10 cm

順序: 5 cm上り、8 cm上り、10 cm上り。下降は方式選択後にGate 7で5 cm単段から独立に開始する。

Go:

- 各条件 20 回連続成功。
- 転倒、harness catch、body/knee hard contact、E-stop 0 回。
- edge 前、前脚接地直後、後脚接地前後の STOP injection 各 10 回で HOLD 成功。
- stair height MAE ≤ 1 cm、edge distance MAE ≤ 3 cm、yaw error ≤ 3°。

失敗、near miss、harness load があれば停止し、原因を解決して該当カウントをゼロから開始する。

### Gate 5 — 2 段、3 段、4 段の昇段

各段数で 20 回連続成功してから次へ進む。4 段では top completion と rear-foot clearance を重点確認する。

Go:

- 各段数 20/20。
- false top complete 0 回。
- top HOLDを設定上限まで（初期60秒）20/20。温度・電圧余裕を保ち、warningと事前選択recoveryを20/20で確認する。
- 人・障害物・短い top landing を提示した refusal test 20/20。

### Gate 6 — 自然言語接近

昇降 controller はまだ起動せず、階段前の standoff で HOLD する。

Go:

- 50 runでlateral error ≤ 5 cm、longitudinal error ≤ 5 cm、yaw error ≤ 3°を49/50以上。
- edge crossing、階段接触、人への接近 0 回。
- 宣言したrequired sensorのstale、未知障害物、drop、低confidence時 refusal 100%。VLMがadvisoryの構成では、VLM timeoutを `climbable=true` に変換せず、幾何＋local guardianで判定する。
- VLM 90 秒 stall でも local guardian が停止距離内に止める。

### Gate 7 — 4 段下降（昇段と独立）

前向きと後退を別試験にする。最初は自然言語接近・昇段を連結せず、disarmと駆動energy-safeを確認した機体を、rated harness／機械liftと検証済みhandling手順でtop fixtureへ配置してから再preflight・armする。powered robotを人が抱えて階段上へ置かない。

Go:

- 選択方向で単段から 4 段まで Gate 4/5 と同じ ladder を通過。
- 4 段 50 回連続で、転倒、harness catch、hard body contact、E-stop 0 回。
- false bottom complete 0/100。
- bottom HOLD 30 秒 50/50。
- 進行方向 ROI coverage と fresh 判定 100%。unknown/drop 提示で refusal 100%。

後退下降は rear/down sensing の証拠がなければ No-Go のままとする。

### Gate 8 — 音声入力統合（有線マイク／AirPods）

最初は push-to-talk、限定文法、画面/音声 read-back、実行確認を必須にする。

Go:

- STOP 認識 recall 100/100。ただし物理 STOP の代替にはしない。
- 騒音・会話・動画音声 1,000 utterance で hazardous false start 0。
- 「登る」と「降りる」の方向混同 0/500。
- 有線マイク抜去、USB device消失、Bluetooth、ブラウザ、Whisper断で新しいtaskを開始せず、走行中は全phase注入で合格したsafe-boundary動作へ遷移。
- 音声→skill subsystemをascent-only 50回、descent-only 50回で別集計し、それぞれsuccess 95%以上、転倒0。接近・往復を含むpaired missionはGate 9だけで数える。

### Gate 9 — 統合研究デモ

自然言語接近 → HOLD → 音声昇段 → top HOLD → 音声下降 → bottom HOLD を連結する。固定階段、固定照明、立入禁止、落下拘束、専任 safety operator は継続する。

Go:

- 100 end-to-end run で mission success 95% 以上。
- 転倒、harness catch、人/物への接触、false direction、false completion 0。
- 全 run で完全な log と video が保存される。

この Gate は「制約された研究デモが再現できる」という意味に限る。一般環境、無監視、公道、第三者の近傍での運用許可ではない。

---

## 10. Fault injection 計画

fault injection は、適切な Gate と落下拘束の下で一つずつ行う。複合 fault は単一 fault の反応を確認した後に行う。

| Fault | 注入方法 | 期待反応 | 初回 Gate | 合格基準 |
|---|---|---|---:|---|
| policy 40/100/500 ms stall | inference sleep/mock | trigger/latch。平地／安定stanceはSOFT_STOP→HOLD、階段途中はphase-safe boundary | 1L/2L | signed phase bound内、異常frame 0 |
| policy process kill | SIGKILL | actuator ownerがphase-safe boundary/latched fault | 2L | signed phase bound、publisher gap KPI内 |
| LowCmd actuator owner kill | process kill | 自動の第2DDS writerは起動せず、検証済みfirmware timeout/独立hardware path、再始動禁止 | 2L | bounded safe responseを実測。確立できなければBranch L LIVE No-Go |
| NaN/Inf action | tensor corruption | 送信拒否、fault latch | 0/2L | motor へ 0 件 |
| wrong dimension | truncate action/obs | 送信拒否 | 0 | crash せず latch |
| LowState dropout | packet filter/subscriber pause | backend別hard fault policy | 2L/2S | 測定済みdeadlineで反応 |
| Sport gateway/service kill | process/service停止 | verified firmware/independent path、再始動禁止 | 2S | bounded safe response。未確立ならBranch S LIVE No-Go |
| Sport stair API timeout/no-ack | response drop/mock | 新規開始禁止、phase別safe boundary、fault latch | 2S | blind retry/自動再開0 |
| Sport remote override競合 | remoteとgatewayを同時操作 | 定義済み優先権、mode generation更新 | 2S | 二重実行/予期しない再歩行0 |
| stale cloud | replay timestamp | 接近前は拒否、動作中はphase別safe boundary | 0/3 | 未検証地形へ進行0 |
| map hole/drop | ROI を unknown 化 | fail-closed refusal | 0/4 | false flat 0 |
| odom jump | x/y/z/yaw step | map invalidate、平地／安定stanceはHOLD、階段途中はphase-safe boundary | 0/3 | false complete 0 |
| camera freeze | 同一 frame replay | requiredなら開始拒否/safe boundary、advisoryならproposal無効 | 0/6 | VLM出力だけで継続前進0 |
| WebSocket disconnect | browser close | 新規goal拒否、phase別safe boundary | 3 | Damp/無制限継続をしない |
| 選択マイクdisconnect | USB/adapter抜去またはBluetooth off | task開始拒否、実行中はphase別safe boundary | 8 | silent fallback、危険command、自動再開0 |
| competing command | 2 clients/source | priority/lease に従う | 0/3 | low priority 上書き 0 |
| E-stop bounce | intermittent input | latch 維持 | 2L/2S/3 | 自動復帰 0 |
| mode release failure | mock error/timeout | LowCmd enable 禁止 | 0/2L | 二重制御 0 |
| mode restore failure | mock error/timeout | HOLD/DISABLED、再立上げ禁止 | 0/3 | 自動 Move 0 |
| low battery / sag / top hold timeout | bench/load/時間進行 | start refusal、warning、事前選択recovery | 3/5 | reserveを割る前に回収、未定義recovery 0 |
| foot contact mismatch | sensor mock | flat/stableはspeed limit/HOLD、階段途中はphase-safe boundary | 0/4 | false completion 0 |
| motor overtemp | telemetry mock | phase-safe boundary。TOP_HOLDは事前選択recoveryを前倒し | 0/2L/2S | limit超過継続0、signed bound内 |
| VLM prompt error/timeout | malformed reply/90 s delay | proposal拒否。既存motionはVLM非依存local guardianのbounded commandだけ | 0/6 | VLM待ちでopen-loop距離を延長しない |

複合 fault の重点組合せ:

- 階段中間 + policy stall + WebSocket loss。
- 下降開始 + map hole + odom jump。
- top edge 直前 + camera freeze + VLM move。
- Low-level mode + UI DAMP + controller process stall。
- battery sag + 高 torque + LowState delay。

---

## 11. 定量 KPI

この節を最終acceptanceのcanonical KPI表とする。`04_ROADMAP.md` の値は開発entry gate、`06_VOICE_AIRPODS.md` と `07_SIM_TRAINING_SIM2REAL.md` は各subsystemの追加条件であり、より厳しい値がある場合は厳しい方を適用する。数値を変更するときはこの表を先に更新し、理由、測定器、dataset/run IDsを記録する。

### 11.1 制御・通信

次は両branchに共通する外部安全応答KPIである。

| 共通KPI | Gate 値 |
|---|---:|
| LowState stale hard-fault threshold | 40 ms 初期値、実測で確定 |
| UI STOP → terminal safe-state request accepted | p95 ≤ 150 ms |
| STOP on flat / already stable stance → `|v_xy| < 0.03 m/s` | ≤ 1.0 s |
| STOP during stair gait → phase-safe boundary | signed configの`max_safe_boundary_time_s/distance_m/step_count`以内、unbounded continuation 0 |
| physical E-stop → first verified safe response | p95 ≤ 50 ms、max ≤ 100 ms。software frameまたは外部計測したactuation/物理応答 |

次のTX/inference値は**Branch Lだけ**に適用する。

| Branch L KPI | Gate 値 |
|---|---:|
| LowCmd target rate `f_tx` | 500 Hz既定。200〜500 Hz代替はGate 2L＋hazard review後 |
| TX interval p99 | ≤ `T + 1 ms`, `T=1/f_tx`。500 Hzでは≤3 ms |
| TX interval max | ≤ 10 ms |
| TX gap > 20 ms | 0 / 60 min |
| policy inference p99 | ≤ 18 ms |
| stale policy → SOFT_STOP request | ≤ 40 ms |
| malformed motor frame | 0 |
| simultaneous LowCmd writer | 0 |

Branch Sは内部servo周期を外部から推測せず、正式Sport APIのend-to-end contractを測る。

| Branch S KPI | Gate 値 |
|---|---:|
| exclusive Sport command gateway | 1 |
| simultaneous Sport skill source | 0 |
| stair request → accepted/rejected ack | vendor上限以内、timeout 0/100 |
| `StopMove`/terminal balance request → ack | vendor上限以内、timeout 0/100 |
| API/service timeout → latched safe response | Gate 2Sで実測した上限以内 |
| unexpected restart after timeout/remote override | 0 |

E-stop latency は目標であり、既存ハードウェアが満たすと仮定しない。測定できなければ No-Go とする。

### 11.2 知覚

誤差KPIのground truthは評価対象sensorから独立させる。段高/edgeはcaliper・laser distance meter・固定治具等の測量値、robot/staging poseは外部mocap、較正済みAprilTag camera、total station等を使う。評価中のLiDAR/LIOや、それから作ったelevation mapを自身の正解にしてはならない。測定器、較正日、不確かさをrun manifestへ記録する。

| KPI | Gate 値 |
|---|---:|
| step height MAE | ≤ 1 cm |
| edge distance MAE | ≤ 3 cm |
| edge yaw error | ≤ 3° |
| cloud age during stair motion | ≤ 150 ms |
| odom age during stair motion | ≤ 100 ms |
| camera age for semantic decision | ≤ 500 ms |
| 進行方向 foot-placement ROI coverage | ≥ 90% |
| unknown/drop false-flat | 0 / 1,000 scenes |
| false top complete | 0 / 100 |
| false bottom complete | 0 / 100 |

単一の全体平均 coverage は使わない。次の一歩、左右脚着地点、停止距離、landing の ROI ごとに判定する。

### 11.3 ロコモーション

SIL列はBranch Lに必須であり、Branch Sはvendor simulatorが正式提供される場合だけ参考値として同じ評価を行う。実機最終Gateと安全KPIは両branchに共通である。

| KPI | Branch L SIL | 実機最終 Gate（S/L共通） |
|---|---:|---:|
| 4 段 ascent success | ≥ 99% / 10,000 | ≥ 95% / 100 |
| 4 段 descent success | ≥ 99% / 10,000 | ≥ 95% / 100 |
| uncontrolled fall | < 0.1% | 0 / 100 |
| harness catch | — | 0 / 100 |
| hard body/knee contact | < 0.1% | 0 / 100 |
| configured max TOP_HOLD / 30 s BOTTOM_HOLD | 100% | 100 / 100 |
| phase-random STOP → stable HOLD | 100% / 1,000 | 100 / 100 |

「mission success」と「安全失敗」を混ぜない。安全拒否で task が完了しない run は mission failure だが safety success である。転倒して偶然頂上へ着いた run は mission success ではなく catastrophic safety failure である。

### 11.4 音声・自然言語

| KPI | Gate 値 |
|---|---:|
| hazardous false start | 0 / 1,000 nuisance utterances |
| ascend/descend direction confusion | 0 / 500 commands |
| STOP recall | 100 / 100 |
| ASR/connection loss時の新規 motion | 0 |
| natural-language approach edge crossing | 0 / 50 |

音声 STOP の latency は Bluetooth と認識時間に依存するため、E-stop KPI には数えない。

---

## 12. Rule of three と試験数の解釈

独立で同分布という近似の下で、`n` 回試験して故障が 0 回でも、故障確率が 0 と証明されたわけではない。95% 信頼上限の概算は **3 / n** である。

| 0 failure の試験数 | 1 run 当たり故障確率の 95% 上限概算 |
|---:|---:|
| 20 | 15% |
| 50 | 6% |
| 100 | 3% |
| 1,000 | 0.3% |
| 3,000 | 0.1% |
| 30,000 | 0.01% |

したがって「100 回無転倒」は有用な研究 Gate だが、高信頼安全を証明しない。さらに、連続 run は同じ機体、同じ階段、同じ照明で相関するため、実効サンプル数は表より小さくなり得る。

運用規則:

- C/H safety failure が 1 回でも出たら該当 Gate は即 No-Go。
- 原因を特定し、hazard register、設計、テストを更新する。
- safety-relevant な修正後は、該当条件の連続成功カウントをゼロへ戻す。
- 同じ条件だけで回数を増やさず、日、battery、照明、摩擦、機体姿勢、階段個体を分ける。
- 研究デモの 100 run を、一般環境の failure rate へ外挿しない。

---

## 13. ログ、再現性、incident review

各 run で次を同一時刻軸へ保存する。

- raw LowState、共通NAVのSport request/response/ack、arbiterのselected source、authority/mode generation。
- Branch Lは実送信LowCmd mirrorと方策obs/action/TX timing、Branch Sはformal Sport stair API request/response/state/`StopMove`/timeout/remote override。
- supervisor state、fault bit、watchdog age、deadline miss。
- LiDAR cloud、odom、height map と unknown mask、camera frame。
- foot contact/force、q/dq/tau、temperature、battery。
- STOP/HOLD/DAMP/E-stopの要求時刻、受理時刻、最初の検証済み安全応答。software frameがない独立pathは外部actuation/物理応答を記録する。
- VLM prompt/response、ASR transcript、intent、read-back、operator confirmation。
- 外部 video、harness load が取れる場合は load data。
- robot serial/config、FW、SDK、OS、git commit、共通config hash。Branch Lはpolicy hash、Branch Sはformal API contract hash。
- 階段の段高、踏面、幅、表面、固定状態、照明、payload。

incident に含めるもの:

- 転倒、harness catch、足外し、body/knee contact。
- E-stop/Damp 使用。
- 予期しない mode transition、command source 切替。
- top/bottom false complete、dangerous false start。
- KPI を超えた latency、command gap、sensor age。

incident review が終わるまで LIVE を再開しない。動画だけで判断せず、command/state timeline と照合する。

---

## 14. 実装優先順位

### P0 — LIVE 前の blocker

1. HOLD / Controlled Stop / StopMove / Damp / E-stop をコードと UI で分離する。
2. Branch LはLowCmd single-owner actuator serverとdefault 500 Hz TX loopを作る。200〜500 Hzの代替はGate 2L＋hazard review後だけとする。Branch Sはexclusive Sport gatewayと正式stair API/停止/timeout契約を実装する。
3. 独立 safety supervisor、fault latch、manual reset を作る。
4. command arbiter、source priority、lease、mode generation を実装する。
5. Branch Lはobs/action/LowCmdのfinite/range/temperature/torque gate、Branch SはAPI schema/sequence/deadline/state gateを追加する。
6. unknown を平地にしない height map と、進行方向 ROI coverage gate を実装する。
7. cloud/odom/camera/map の freshness と time synchronization を実装する。
8. 脚接地・edge clearance を使う top/bottom completion を実装する。
9. 下降を独立 policy/skill として評価し、方向を固定する。
10. VLM と独立した local collision/drop guardian を実装する。

### P1 — 実験品質

1. Branch Lの低レベルtarget送信loopをGo2側/real-time Linux側へ移す。Branch Sはvendor Sport serviceの配置を変更せず、gateway/supervisorだけを非リアルタイムUI経路から分離する。
2. sensor dropout、latency、motor/COM domain randomization を訓練へ追加する。
3. phase-random STOP と terrain-aware HOLD を学習・評価する。
4. 下向き depth sensing または rear sensing の追加を比較する。
5. run manifest、bag、video、KPI 自動集計を作る。

### P2 — 音声・自然言語統合

1. `NAVIGATE_TO_STAIR_APPROACH`、`ASCEND_STAIRS`、`DESCEND_STAIRS`、`STOP_NOW` の限定 grammar。
2. task/方向/対象階段の read-back と確認。
3. AirPods loss、ASR timeout、誤認識の fault handling。
4. 安定後に確認操作を減らすが、物理 E-stop と立入管理は残す。

---

## 15. 参考一次資料

- [Unitree SDK2 Python README](https://github.com/unitreerobotics/unitree_sdk2_python): Low-level motor control 前に high-level motion service を無効化し、競合 command を避けるよう説明している。
- [Unitree Go2 Low-level stand example](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/go2/low_level/go2_stand_example.py): `dt=0.002` と 2 ms recurrent write を用いる公式例。
- [Unitree ROS 2 low-level example](https://raw.githubusercontent.com/unitreerobotics/unitree_ros2/master/example/src/src/low_level_ctrl.cpp): 5 ms、200 Hz timer の公式例。
- [Unitree SportClient API](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/go2/sport/sport_client.hpp): `Damp()` と `StopMove()` が別機能として定義されている。
- [Unitree ROS 2 documentation](https://github.com/unitreerobotics/unitree_ros2): Sport state の damping mode と `forwardDownStair` gait state、LowState fields を確認できる。ただしstate enumの存在は、公開SportClientから開始できるAPIの証拠ではない。
- [Unitree Go2 official specifications](https://www.unitree.com/go2/): 機体重量、モデル別 climb/drop height、LiDAR、民生ロボットとしての注意。16 cm は製品仕様上の最大値であり、この自作 controller の安全保証値ではない。
- [ISO 12100:2010 overview](https://www.iso.org/standard/51528.html): 機械の lifecycle 全体で hazard identification、risk estimation/evaluation、risk reduction、documentation/verification を行う一般原則。
- [ISO 13850:2015 overview](https://www.iso.org/standard/59970.html): emergency stop function の機能要件と設計原則。本プロジェクトは適合を主張しない。
- [OSHA Machine Guarding guidance](https://www.osha.gov/sites/default/files/enforcement/directives/CPL_02-00-147.pdf): stop/emergency-stop device だけを guarding の代替にしない考え方。落下拘束、barrier、立入管理が別途必要である。
- [Miki et al., Learning robust perceptive locomotion for quadrupedal robots in the wild](https://arxiv.org/abs/2201.08117): 外界知覚の劣化と proprioception/exteroception 融合の必要性を扱う一次研究。
- [Jin et al., Resilient Legged Local Navigation](https://arxiv.org/abs/2310.03581): invisible obstacles/pits を含む知覚故障を訓練・評価する一次研究。

---

## 16. 最終 Go/No-Go チェック

次の質問のどれか一つでも「いいえ」なら LIVE は開始しない。

- 通常 STOP は Damp せず、階段上で能動 HOLD できるか。
- 物理E-stop functionはブラウザ、AirPods、VLM、選択backend processと独立し、外部計測で応答を実証したか。
- Branch LはLowCmd writerが1個だけで実送信deadlineを測り、Branch SはSport gatewayが1個だけでAPI ack/timeoutを測っているか。
- Branch Lはpolicy stall、Branch Sはgateway/service timeout、両branchはLowState lossのfault injectionに合格したか。
- unknown terrain を平地に変換していないか。
- 進行方向の全 foot-placement ROI と landing が fresh に観測できるか。
- 最後脚の edge clearance を確認してから top/bottom complete にしているか。
- 選択した下降方向をbackendに応じて独立に訓練またはvendor-contract検証し、実機評価したか。
- 階段、harness、barrier、専任 safety operator が準備済みか。
- 対象backend artifact/contract、FW、config、extrinsicは合格時と同じhash/versionか。
- TOP_HOLD上限、energy/thermal reserve、timeout recoveryが署名済みrun configにあり、回収まで成立するか。
- 当日の lower Gate smoke test に合格したか。
- C/H hazard の未解決項目がゼロか。

本計画の最優先目的は「何とか 4 段を成功させる」ことではなく、失敗時にも人を近づけず、機体を落とさず、原因を再現可能な形で観測しながら、段階的に能力境界を広げることである。
