# 01. 現在地監査

## 1. 監査範囲

2026-07-13 JST に、ブランチ `feat/sf-ui-implementation`、commit `a5535be` を読み取り中心で監査した。別セッションが UI を編集しているため、本調査では `cockpit/static/` を変更せず、新規 `docs/` だけを対象にする。

確認したもの:

- `common/`, `m0_teleop/`, `m1_agent/`, `m2_navila/`, `m3_rl/`, `cockpit/`, `policy/`
- README と学習設定
- Git 状態、テスト、policy の offline 1-step/100-step 疎通
- `deploy_log.jsonl` のイベント種別と dry-run 状態
- 公式 SDK/LiDAR/研究資料との整合

## 2. 再利用すべき資産

### 2.1 実機 I/O

`common/go2_iface.py` は、次を一つの API にまとめている。

- `SportClient.Move/StopMove/Damp` と姿勢 API
- `rt/lowstate`, `rt/sportmodestate` の購読
- 前面カメラ `VideoClient`
- `MotionSwitcherClient` による Sport/Low-level 切替
- `rt/lowcmd` の publisher
- 同じ上位コードを動かせる `MockGo2`

`common/config.py` には、実機で確認したと記録されている以下の topic がある。

- `rt/utlidar/cloud_deskewed`
- `rt/utlidar/robot_odom`
- `rt/lowstate`
- `rt/lowcmd`
- `rt/sportmodestate`

この層は捨てず、型付き command arbiter と独立 servo/safety process を外側に追加する。

### 2.2 LiDAR 標高マップ

`m2_navila/elevation_node.py` と `cockpit/server.py` は、L1 点群を 2.5D rolling elevation map に統合し、Wave5 が要求する 187 点の `height_scan` を生成する。`cockpit/stair.py` は、距離、段高、幅、edge yaw を推定し、`step / stairs / wall / drop` に分類する。

これは階段前整列と policy 入力の良い共通基盤である。ただし、下降で必要な「未知」と「平面」を区別できるように直す必要がある。

### 2.3 上り状態機械

`cockpit/stair_task.py` には、概ね次の状態遷移がある。

`SCAN → ALIGN → APPROACH → CONFIRM → CLIMB → SETTLE`

上りの接近制御を再利用し、上端保持、下降 preflight、下降、下端保持を追加する方がよい。

### 2.4 Wave5 policy

`policy/policy.pt` は、235 次元観測から 12 関節の位置 action を出す TorchScript MLP である。

- policy rate: 50 Hz
- 観測: base velocity、angular velocity、gravity、velocity command、joint state、last action、187 点 height scan
- action scale: 0.25 rad
- 学習地形: 通常階段 20%、逆階段 20%、box/rough/slope
- 段高範囲: 0.05〜0.20 m
- command `lin_vel_x`: -1.0〜1.0 m/s

したがって、約 10 cm の上りだけでなく、負の速度での下降能力を潜在的に持つ可能性がある。しかし、リポジトリに下降評価はなく、能力があるとはまだ言えない。

現行ファイルの SHA-256:

| ファイル | SHA-256 |
|---|---|
| `policy/policy.pt` | `11ec4446dc368190ef1aba1f810f9bf97c5b7fadb993aeb85e3b3f18719195ae` |
| `policy/policy.onnx` | `0de374d42dd83e61e27b2730e892ca826901ba8560cea3e5c6dfa0afc42673ce` |
| `policy/policy_wave4_maxclimb.pt` | `a8076721c9cf33c72aabf897f5393954015e32fd854e0f42ddad7e4efe3a2c94` |
| `policy/policy_wave4_maxclimb.onnx` | `3ae06ed228c27554fe262d88046180e6251119c2b6656f72fed9b715b86157e0` |

## 3. 実行確認

| 確認 | 結果 | 意味 |
|---|---|---|
| `python3 -m cockpit.stair` | 11/11 PASS | 合成 profile に対する現在の上り検出 |
| `python3 -m m3_rl.joint_map` | PASS | Isaac/SDK joint order の往復 |
| `python3 -m m3_rl.test_obs_builder` | PASS | policy shape、100-step feedback、command sensitivity |
| 要求文を `cockpit.voice.parse_intent` に入力 | 下記の通り | 現行 voice parser の仕様不適合 |

policy offline テストの出力は、`ALL OK - 配線疎通に問題なし（物理検証は sim2sim で）` である。これは network shape の確認であり、物理的な歩行成功ではない。

`cockpit.stair` の 11/11 も精度 benchmark ではない。特に「緩斜面」の case は期待 class が `None`、つまりどの分類でも test 上は合格する設定で、実際の出力は `stairs` だった。斜面と連続階段の識別、棚・縞模様・影・欠測などの negative set を別途作り、危険側の誤検出を 0 件基準で評価する必要がある。

現行 parser の実測:

| 入力 | 現在の出力 |
|---|---|
| 「階段の前まで行って」 | 3 秒の単純前進 |
| 「その前の階段を登って一番上まで行ったら止まれ」 | 即 `stop` |
| 「降りて段を下り切ったらまた止まれ」 | 即 `stop` |
| 「止まれ」 | 即 `stop` |

文中の「止まれ」を一律 emergency stop として先に拾うため、「完了したら保持」と「今すぐ止まれ」を区別できていない。

## 4. 要件に対するギャップ

### 4.1 下降タスクは未実装

下降 edge は `cockpit/stair.py` で `drop` に分類され、`cockpit/stair_task.py` は安全上拒否する。これは現在のコードとして正しいが、目標達成には次が必要である。

- 連続する下降 tread と単なる崖の識別
- 上端 edge 直前の停止・足配置
- 下側 landing の観測
- 下降方向に合った height scan
- 下降専用の completion detector
- Wave5 の下降 sim/real 評価

### 4.2 音声入力deviceは保証されていない

現状はブラウザの既定マイク、または `sounddevice` の既定 input を使うだけである。

- AirPods／有線マイクの `deviceId` 選択・固定なし
- USB/cable/Bluetooth disconnect、入力 level、sample rate の監視なし
- remote HTTP origin から browser mic を使えない
- 音声から `ASCEND/DESCEND` skill への型付き経路なし
- push-to-talk 後に全文を ASR するため、voice stop は物理 E-stop にならない

有線マイクをOSの既定入力にすれば現行PoCでも偶然動く可能性はあるが、選択deviceの表示、抜去検出、内蔵マイクへのsilent fallback禁止は実装されていない。有線化だけでこの監査項目が解消するわけではない。

### 4.3 自然言語 navigation は PoC

`cockpit/mission.py` は camera と heightmap を VLM に渡し、`move/turn/stop/done` を逐次実行する。永続地図、stairs instance、approach pose、global/local planner はなく、VLM 応答待ちの間は制限時間付きで直前速度を維持する。

この方式は研究比較には残せるが、主系統は次へ移すべきである。

`language target → semantic stair candidate → geometry-verified approach pose → collision-checked navigation`

### 4.4 正常終了と Damp が混同されている

現在の RL 統合は、頂上判定後に速度をゼロにし、policy process へ SIGINT を送り、Damp して終了する。Damp は active balance を失わせ得るため、「一番上まで行ったら止まれ」の正常完了として不適切である。

必要な分離:

- `HOLD`: policy を生かし、command `(0,0,0)` で姿勢保持。通常の stop と上端待機。
- `CONTROLLED_EXIT`: 十分広い平面でのみ Sport へ安全に戻す。
- `DAMP/CRITICAL_STOP`: critical fault時の状態別経路。階段上では落下防止設備を前提とする。
- `E-stop function`: software Dampとは別の独立停止機能。現時点で確認済み装置はなく、実機Gateで同定が必要。

上端で policy を保持したままなら、次の下降命令へ mode switch なしで遷移できる。

### 4.5 perception が一部 fail-open

監査で確認した危険箇所:

- camera/VLM 確認が利用不能・parse failure・例外のとき、肯定扱いで続行する経路がある。
- 未観測 height cell を足元と同じ高さで埋める。
- map cell の age を `lookup` で判定しない。
- 全 scan の coverage が閾値以上でも、着地点周辺が未知の可能性がある。
- `kind == none` を前方平坦と扱う箇所があり、知覚喪失と平坦を区別しない。

上りでも修正が必要だが、下降では致命的である。安全判断では `unknown != free` とし、camera/VLM failureを `confirmed=true` へ変換しない。VLMをrequired sensorと宣言したrunは `NO-GO`、既知固定階段でVLMがadvisoryのrunは、LiDAR/RGB-D幾何と独立した人・障害物guardianが全gateを満たせば、VLMなしでも実行可能にする。

### 4.6 LowCmd timing と watchdog

現行 `m3_rl/rl_stair_controller.py` は policy 推論と LowCmd publish を同じ Python loop の 50 Hz で行う。一方、Unitree 公式の Go2 stand example は LowCmd write interval を `0.002 s`、許容例を `0.001〜0.01 s` としている。

50 Hz が対象 firmware で直ちに危険だと断定はしないが、公式例の範囲外なので、実機前の重要な検証項目である。推奨は次の分離である。

- policy inference: 50 Hz
- real-time LowCmd publisher: 200〜500 Hz候補、既定500 Hzで最新50 Hz targetを原則ZOH再送。代替rateは`T=1/f_tx`のtiming gateとhazard review後のみ
- target間補間は学習時のaction contractを変えるため、同じfilterを学習・sim2simへ入れて比較合格した場合だけ採用
- publisher は C++ か deadline を監視できる独立 process
- target staleは即時trigger/latchとし、最後のgait targetを無期限再送しない。平地／既に安定stanceだけlocal active hold、階段途中は署名済みphase別time/distance/step bound内の検証済みsafe-boundary反応

また、現在の `do_damp()` は即座に別の独立 publisher が Damp を開始するのではなく、主 loop に終了を依頼する構造である。主 loop hang を想定した外部 safety process が必要である。

公式例: [Unitree SDK2 Go2 stand example](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/go2/go2_stand_example.cpp)

### 4.7 command arbitration と access control

manual、voice、Mission、StairTask、RL が共有 command state を使い、優先順位が複数箇所に分散している。正式な arbiter を設け、優先順位を一か所で次のようにする。

`physical E-stop > safety supervisor > STOP_NOW/manual override > active stair skill > navigator > language planner`

さらに cockpit server は LAN 向け bind、認証/TLS/operator lease/origin check がない。実機 LAN では少なくとも、単一 operator lease、認証、TLS、DDS network 分離が必要である。

## 5. 実証済み範囲を過大評価しない

`deploy_log.jsonl` には実機 M0 check の記録がある。一方、監査時点で確認できた M3 policy start はすべて `dry: true` または `dry_run: true` であり、live LowCmd による実階段成功の証拠はない。

README が参照する以下の外部成果物も、この workspace には存在しなかった。

- `../out/wave4/DEPLOY_GO2_JP.md`
- `../go2_stair_rl/harness/ledger.jsonl`

Wave5 の README 記載 sim 成功率は有望な手がかりだが、現在の repo だけでは再実行できない。学習 script、evaluation script、seed、episode-level results、sim2sim harness を戻すまで「再現済み」と扱わない。

## 6. 再現性の不足

- `requirements.txt` に version pin/lock がない。
- Unitree SDK は Git HEAD の手動 install。
- NaVILA は別環境で、統合 test がない。
- Isaac Lab の正確な source revision と task override がない。
- Wave5 は設定上 1 seed。
- real sensor bag/MCAP と replay test がない。
- MuJoCo sim2sim code が同梱されていない。
- pytest/CI/lint/type check がない。
- firmware、calibration、stair geometry を run と結び付ける ledger がない。

## 7. 監査から決まる実装順

1. 正常STOPをactive holdにし、独立arbiter／Safety Supervisorとsingle-owner actuation gatewayを作る。Branch Lはsole LowCmd publisher、Branch Sはexclusive Sport API clientとし、安全監視の独立性を第2の送信者で実現しない。
2. 複合命令を型付き GoalSpec に直す。
3. map freshness、unknown maskを導入し、VLM failureを肯定へ変換するfail-openを廃止する。required/advisory構成を明示する。
4. exact 10 cm × 4 段で Wave5 の上り/後退下降を sim2sim 評価する。
5. 下降認識と completion detector を実装する。
6. device非依存の音声gateway（有線baseline／AirPods variation）、navigation、実機試験へ進む。

この順序なら、既存資産を活かしながら最も危険な設計欠陥を先に除ける。
