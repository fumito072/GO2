# AirPods・有線マイク音声命令とローカル ASR 設計

調査・設計日: 2026-07-13（JST）  
対象: AirPods または有線マイク付きイヤホンを使った「上れ」「下れ」「今すぐ止まれ」と、文字入力を共通の型付き `GoalSpec` に変換する経路

## 1. 結論

採用する経路は次である。

```text
Selected operator microphone
  ├─ USB/USB-C wired headset（MVP baseline）
  ├─ 3.5 mm wired headset（対応jackまたはUSB audio adapter経由）
  └─ AirPods（Bluetooth variation）
  → macOS audio input
  → cockpit PWA（明示的な入力デバイス選択、push-to-talk）
  → HTTPS audio upload / WSS session・heartbeat
  → RTX 5090 voice gateway
  → local VAD + faster-whisper
  → confidence calibration + deterministic semantic parser
  → typed GoalSpec
  → command arbiter / Mission FSM / Safety Supervisor
  → navigation または stair skill
```

設計上の最重要点は以下である。

1. AirPods／有線マイクは Go2 へ直接接続しない。Mac の交換可能な音声入力として使用する。
2. ブラウザのマイク取得には secure context が必要である。別の Mac から RTX 5090 の `http://<IP>:8080` を開く構成は採用せず、`HTTPS/WSS` を必須にする。
3. MVP は wake word ではなく push-to-talk（PTT）とする。録音していない時間の会話を命令として扱わない。
4. 音声をそのまま速度指令へ変換しない。文字入力と音声の両方を同じ `GoalSpec` に変換し、Mission FSM と safety gate を通す。
5. 単独の「止まれ」は `STOP_NOW` であり、通常の goal queue を迂回して最優先で処理する。
6. 「一番上まで行ったら止まれ」の「止まれ」は緊急停止ではない。完了条件 `TOP_LANDING_STABLE` と、完了後の `ACTIVE_HOLD` に変換する。
7. `HOLD` は関節制御・姿勢安定化を生かした能動保持である。`DAMP` や低レベル方策プロセス終了とは異なる。
8. 音声経路は物理 E-stop ではない。マイク／USB／cable／Bluetooth、ブラウザ、LAN、ASR、parser の全てに故障可能性があるソフトウェア操作系であり、安全規格上の緊急停止装置にはできない。

## 2. 対象と非対象

この文書が対象とする命令は、最初のデモでは次の四つに限定する。

| 人の意図 | 型付き意図 | 代表的な発話 |
|---|---|---|
| 階段手前へ移動 | `NAVIGATE_TO_STAIR_APPROACH` | 「前の階段のところまで行って」 |
| 階段を上る | `ASCEND_STAIRS` | 「その階段を上って、一番上まで行ったら止まれ」 |
| 階段を下る | `DESCEND_STAIRS` | 「下りて、下り切ったら止まれ」 |
| 今すぐ動作を止める | `STOP_NOW` | 「止まれ」「今すぐストップ」 |

自然な言い換えは受け付けるが、音声から任意の Python、速度列、関節角、未登録 skill を生成してはならない。自由会話や複雑な依頼は、LLM に渡す前に「解釈候補」として表示するだけにし、型付き skill に落ちなければ実行しない。

当面の非対象は以下である。

- 常時待受 wake word
- 離れた場所からの無監視運転
- クラウド ASR を必須にする構成
- AirPods、有線マイク、その他の音声入力による安全認証済み E-stop
- ASR/LLM が直接 Go2 の `Move`、`LowCmd`、関節 target を送る構成
- Go2 内蔵マイクを SDK から取得する構成。公開 SDK には、開発者向けの連続マイク入力 API が確認できない

## 3. 現行リポジトリの監査結果

既存実装は PoC として再利用できるが、目標デモの音声 gateway としては次の不足がある。

### 3.1 ブラウザ側

`cockpit/static/app.js` は、PTT の最初の押下時に次を実行する。

```javascript
navigator.mediaDevices.getUserMedia({ audio: true })
```

そのため現状は以下の状態である。

- OS/browser の既定マイクを使用し、操作者が意図した入力deviceを明示的に選択・表示・固定しない
- `enumerateDevices()` によるデバイス一覧、選択 UI、`deviceId` constraint がない
- USB抜去／Bluetooth切断時の `devicechange`、track の `ended` / `mute`、入力レベルを監視しない
- 選択マイクが消えた際に Mac 内蔵マイクへ暗黙に切り替わる可能性を排除していない
- `MediaRecorder` の MIME type を `audio/webm` に固定し、`MediaRecorder.isTypeSupported()` を確認しない
- 録音時間上限、無音、clipping、音声 format、session ID を送らない
- `/voice` に timeout、`AbortController`、idempotency key がない
- ASR 結果を受け取ると `execIntent()` を即実行し、confidence/readback/GoalSpec validation を通さない
- PTT 後に一括認識するため、音声の「止まれ」は発話終了・ボタン解放・ASR 完了後にしか届かない

`cockpit/static/index.html` には既に押している間だけ録音する PTT ボタンがある。この操作方法は残し、デバイス状態と確認 UI を追加するのがよい。

### 3.2 RTX 5090 / server 側

`cockpit/voice.py` は `faster-whisper` の `small` を `device="cpu", compute_type="int8"` で実行し、文字列だけを返す。`cockpit/server.py` の `/voice` は一時 WebM ファイルを作成し、認識後に削除する。基本的なローカル処理の土台はあるが、次が不足している。

- RTX 5090/CUDA を使わず、README 記載の実測認識時間は約 2〜3 秒
- segment/word probability、`avg_logprob`、`no_speech_prob`、language probability、VAD coverage を破棄する
- calibration 済み intent confidence がない
- `/voice` の application 上限は 32 MiB だが、音声専用の短い duration/size 上限と rate limit がない
- endpoint authentication、operator lease、origin check、TLS がない
- server は既定で `0.0.0.0:8080` の平文 HTTP
- `parse_intent()` は全文のどこかに stop 語があれば、他の節より先に `stop` を返す
- 認識文字列と単純 action しかログせず、audio quality、confidence、parser version、GoalSpec、ack latency を再現できない

また `m1_agent/voice_input.py` にも `sounddevice` と Whisper を使う別実装がある。こちらも OS の既定 input を使う。将来は音声認識を voice gateway に一本化し、cockpit と M1 agent が同じ `GoalSpec` schema を利用するべきである。

### 3.3 「止まれ」の既知の誤解釈

現行 parser では以下になる。

| 入力 | 現在の解釈 | 正しい解釈 |
|---|---|---|
| 「止まれ」 | 即 stop | `STOP_NOW` |
| 「その階段を上って一番上まで行ったら止まれ」 | 即 stop | `ASCEND_STAIRS`、完了条件 `TOP_LANDING_STABLE`、完了後 `ACTIVE_HOLD` |
| 「下り切ったらまた止まれ」 | 即 stop | `DESCEND_STAIRS`、完了条件 `BOTTOM_LANDING_STABLE`、完了後 `ACTIVE_HOLD` |

単語検索の順序変更だけでは不十分である。条件節、否定、引用、話し直しを含む clause-aware parser と、状態整合性検査が必要である。

### 3.4 現行 stop 経路の問題

`cockpit/static/app.js` の `stopAll()` は mission、stair task、RL を全て終了させた後、速度ゼロと `action: stop` を送る。さらに `cockpit/rl_bridge.py` の `RlController.stop()` は SIGINT 後に `Damp` して方策プロセスを終了する。

これは平地の非常退避には利用できても、階段上の正常完了には使用できない。上端・下端で「止まる」は、階段 policy または安全な姿勢 controller を継続した `HOLD` でなければならない。`STOP_NOW` の実装でも、階段上で即 DAMP すべきかは Safety Supervisor が機体状態に基づいて決める必要がある。

WebSocket切断時に自律系をabortし、ゼロ指令と `stop_move()` を送る現行方針は平地Sportでは安全側の出発点である。ただしLowCmd階段方策では、そのabortがDAMP終了につながり、gait phaseも無視する。明示的な `HMI_LOSS` eventをSafety/Missionへ渡し、検証済みphase別safe-boundary経路へ変更する必要がある。

## 4. 目標アーキテクチャ

### 4.1 コンポーネントと信頼境界

```text
┌──────────────── Mac / operator station ────────────────┐
│ Selected microphone ─ macOS audio input ─ cockpit PWA  │
│ USB/USB-C / 3.5 mm / AirPods                           │
│ device picker / PTT / level / transcript / readback    │
└──────────────────────┬──────────────────────────────────┘
                       │ HTTPS: short audio + metadata
                       │ WSS: lease, heartbeat, state, ack
                       ▼
┌──────────────── RTX 5090 workstation ───────────────────┐
│ TLS terminator + authenticated voice gateway            │
│  ├─ decode/resample/VAD/audio quality                    │
│  ├─ faster-whisper CUDA                                 │
│  ├─ confidence calibration                              │
│  ├─ deterministic semantic parser                       │
│  └─ GoalSpec schema validator                           │
│             │                                           │
│             ▼                                           │
│ command arbiter → Mission FSM → skill executor          │
│       ▲                  │                               │
│ Safety Supervisor ───────┘                               │
└──────────────────────┬──────────────────────────────────┘
                       │ isolated robot Ethernet / DDS
                       ▼
                      Go2
```

PWA は operator I/O であり、ロボット命令の最終 authority ではない。ブラウザで action を決定して `Move` を繰り返す現行構成をやめ、RTX 5090 側が GoalSpec を検証し、単一の arbiter が command owner を決める。

### 4.2 HTTPS と WSS の役割

MVP では役割を次のように分ける。

- `HTTPS POST /api/v1/voice/utterances`: PTT で確定した短い音声 clip と metadata を送る
- `WSS /api/v1/operator/session`: operator lease、heartbeat、認識状態、readback、confirmation、GoalSpec ack、Safety Supervisor 状態を送受信する
- `HTTPS GET /api/v1/audio/devices/help`: ブラウザ側の診断表示に必要な静的情報のみ。実際のデバイス一覧は Mac 上の browser API から得る

短い PTT clip なら HTTPS upload は実装と再試験が簡単である。第2段階では、録音中の Opus chunk を WSS で 100〜250 ms ごとに送り、PTT release 時には upload がほぼ完了している構成を比較できる。ただし streaming partial ASR を導入しても、音声 STOP が物理 E-stop になるわけではない。

各 request/event に次を持たせる。

- `session_id`: PWA 起動ごとの UUID
- `utterance_id`: PTT 1回ごとの UUID
- `operator_lease_id`: 現在 command 権限を持つ操作者
- `sequence`: session 内の単調増加番号
- `client_started_mono_ms` / `client_released_mono_ms`
- `content_type`, `codec`, `sample_rate`、録音時間
- `sha256`: retry 時の同一 clip 判定
- `schema_version`

server は `(operator_lease_id, utterance_id)` を idempotency key として扱い、再送で同じ goal を二度開始しない。再接続後に browser の古い queue を自動 replayしてはならない。

## 5. Mac、入力マイク、PWA

### 5.1 デバイス非依存の方針と選定順

音声gatewayは`AirPods`という製品名ではなく、Mac/browserが公開する選択済み`audioinput`を入力契約にする。同じPTT、VAD、ASR、GoalSpec、確認、安全gateを全マイクで共有し、マイク種別ごとの分岐をMission層へ持ち込まない。

初期の推奨順は次である。

| 入力 | 初期位置づけ | 主な利点 | 必ず試験する故障 |
|---|---|---|---|
| USB/USB-C有線マイク付きイヤホン | **MVP baseline** | Bluetooth route切替や電池切れがなく、device検出とlatencyを再現しやすい | 抜去、connector接触不良、sleep復帰、別USB portへの差し替え |
| 3.5 mm TRRSマイク付きイヤホン | 条件付き候補 | 安価で単純 | Mac機種／jack／CTIA対応adapterで入力として列挙されるか、出力専用扱い、接触ノイズ |
| AirPods | 無線variation／最終UX候補 | cableがなく操作者が動きやすい | Bluetooth切断、自動route切替、電池、profile変化、再接続 |

3.5 mm品は「挿さる」ことを合格条件にしない。macOSの入力一覧とbrowserの`enumerateDevices()`の両方に現れ、入力levelと試験録音が合格した場合だけ使用する。入力として認識されない場合は、Macで動作確認したUSB audio adapterまたはUSB/USB-C headsetへ切り替える。4極TRRSの配線規格やadapterのmic対応は製品ごとに確認し、推測でrunを開始しない。

有線ケーブルは操作者のMacにだけ接続し、移動するGo2へつながない。operator station周囲で抜け止めとstrain reliefを行い、通路や落下防止索と交差させない。ケーブル抜去は安全入力ではなく通常の`VOICE_UNAVAILABLE`故障として扱う。

入力設定は概念的に次を持つ。`deviceId`はbrowser環境で変わり得るため、永続IDだけで自動ARMせず、run前にlabel、入力level、試験録音を再確認する。

```yaml
audio_input:
  selected_device_id: browser-session-device-id
  expected_transport: USB_WIRED  # operator-confirmed: USB_WIRED | ANALOG_ADAPTER | BLUETOOTH
  sample_rate_hz_after_decode: 16000
  channels_after_decode: 1
  push_to_talk: true
  silent_fallback_to_default_device: false
  disconnect_action: REJECT_NEW_VOICE_GOALS
```

`expected_transport`はdevice labelから自動推測せず、run前に操作者が現物と照合したmetadataとする。browserが提供しないtransport種別を推測して安全判断へ使わない。

### 5.2 AirPodsの接続と OS 設定

AirPods は通常の Bluetooth headset として Mac に接続し、macOS の「サウンド > 入力」で AirPods が選ばれ、入力レベルが動くことを最初に確認する。Apple の手順は[Macでサウンド入力設定を変更する](https://support.apple.com/en-gb/guide/mac-help/mchlp2567/mac)を参照する。

試験中の意図しない接続先変更を防ぐため、AirPods の「このMacに接続」を「前回このMacに接続していた場合」に設定する。Apple の[自動切り替えの案内](https://support.apple.com/en-ie/guide/airpods/dev228ba3df8/web)も参照する。

注意点:

- AirPods のモデル、macOS、browser、出力デバイスの組み合わせで Bluetooth profile と実効帯域が変わる。モデル名だけから sample rate や遅延を決め打ちしない
- Apple の高品質 Bluetooth 録音 API は対応する native app 向けの能力であり、browser PWA が同じ動作をするとは仮定しない
- 実験前に AirPods の充電量、接続先、入力 level、codec/sample rate を run metadata に残す
- 読み上げ音声を同じ AirPods へ出す場合、録音中には再生しない。PTT release 後に readback する

### 5.3 secure context

`navigator.mediaDevices.getUserMedia()` は secure context でのみ利用できる。[MDN の getUserMedia 仕様説明](https://developer.mozilla.org/en-US/docs/Web/API/MediaDevices/getUserMedia)にある通り、`localhost` は例外だが、別マシンから開く `http://192.168.x.x:8080` は secure context ではない。

本番の研究 LAN では以下を必須にする。

- RTX 5090 の cockpit を `https://go2-console.<lab-domain>/` で提供
- WSS も同一 origin に限定
- Mac が信頼する研究室 CA または適切な証明書を使用
- 自己署名証明書の警告を毎回無視する運用にしない
- HTTP は HTTPS へ redirect し、実機 command endpoint を平文で開けない
- certificate expiry を起動前診断に含める

インターネット公開は不要である。通常 LAN と Go2 の DDS 専用 NIC を分け、TLS cockpit は通常 LAN 側だけで listen させる。

### 5.4 明示的なデバイス選択

PWA は最初に permission を取得し、その後 `enumerateDevices()` で `audioinput` を表示する。permission 前は label が空になる browser があるため、順番を逆にしてはならない。詳細は [MediaDevices.enumerateDevices](https://developer.mozilla.org/en-US/docs/Web/API/MediaDevices/enumerateDevices)を参照する。

推奨 UX:

1. `マイクを許可` をユーザーが明示的に押す
2. 一時 stream を取得して permission を確立
3. audio input 一覧を表示
4. USB/USB-C有線、3.5 mm adapter、AirPodsを同じ一覧に表示し、MVPでは検証済み有線deviceを推奨するが、最終選択は操作者が行う
5. 選択した `deviceId` を `getUserMedia({audio: {deviceId: {exact: ...}}})` に渡す
6. 2秒のレベル確認と試験録音を行う
7. 画面に `選択device名 / transport / readyState / input level / 最終音声時刻` を常時表示

`deviceId` は永続識別子として信用しすぎない。permission、browser profile、再ペアリングで変化し得る。保存した ID がなければ label/group とユーザー確認で再選択する。

以下を監視する。

- `navigator.mediaDevices` の `devicechange`
- `MediaStreamTrack.readyState`
- track の `ended`, `mute`, `unmute`
- 入力 RMS、clipping ratio、長時間の完全無音
- browser の `visibilitychange`, `pagehide`
- Mac sleep/wake、network change

選択した入力deviceが消えたとき、Mac 内蔵マイクや別のマイクへ黙ってfallbackしてはならない。音声状態を`VOICE_UNAVAILABLE`とし、UIを赤にし、手動再選択、level test、試験録音が完了するまで新しいtask goalを受け付けない。この規則はUSB抜去、3.5 mm adapter消失、AirPods切断の全てに共通である。

### 5.5 codec と録音条件

browser 起動時に `MediaRecorder.isTypeSupported()` で候補を調べる。

推奨優先順は例として次であるが、実際の browser 対応結果を使う。

1. `audio/webm;codecs=opus`
2. `audio/mp4`
3. browser が提供する既定 format

server では PyAV/FFmpeg で decode し、ASR 前に mono PCM 16 kHz float へ resample する。browser が宣言した MIME type と実データを検査し、不一致や decode error は実行せず拒否する。

初期制限:

- PTT 1回: 0.25〜8.0秒
- decode 後音声: mono、上限16秒相当を hard limit
- 最大 upload: 2 MiBを目安に codec ごとに調整
- 無音率が高すぎる、clippingが多い、発話が短すぎる場合は再録音を要求
- `echoCancellation`, `noiseSuppression`, `autoGainControl` は機種/browser別に A/B 評価し、固定値を先に決めない

## 6. Push-to-talk の状態機械

常時待受ではなく PTT を選ぶ理由は、命令区間が明確になり、背景会話とロボットのモーター音を誤命令として扱う面積を小さくできるからである。

PWA の状態は最低限次を持つ。

```text
DISCONNECTED
  → PERMISSION_REQUIRED
  → DEVICE_TEST
  → READY
  → RECORDING
  → UPLOADING_OR_FINALIZING
  → TRANSCRIBING
  → REVIEW_OR_CONFIRM
  → ACCEPTED / REJECTED
  → READY

任意の状態 → VOICE_FAULT
```

PTT の仕様:

- pointer down で録音を開始し、pointer capture を取得する
- pointer up で確定する
- `pointercancel`、tab 非表示、page hide は「確定」ではなく `CANCELLED` にする
- pointer がボタン外へ出ただけで偶発確定しない UI を比較する。現在実装は `pointerleave` で stop するため、実機試験で誤解放を測る
- 8秒で自動打ち切りし、長い自由会話を実行しない
- 0.25秒未満は click として破棄
- 録音中は赤表示、経過時間、入力 level、選択デバイス名を表示
- PTT release 後に同じ `utterance_id` の進行状況を WSS で表示
- 認識中にもう一度押した場合は、新規録音を許す前に前の utterance を明示的に cancel するか、順番を表示する。暗黙 queue は作らない

MVP では「録音中だけ発話が命令候補になる」を不変条件とする。wake word は、PTT版の安全性・誤作動率・遅延を確立してから、完全に別の評価項目として扱う。

## 7. RTX 5090 ローカル ASR

### 7.1 runtime

現行 `small/int8/CPU` から、RTX 5090上の `faster-whisper` CUDA backend へ移す。初期候補は日本語性能の高い Whisper `large-v3` 系とし、より小さい model と実測比較する。モデル名だけで決めず、このプロジェクトの階段命令 corpus で semantic accuracy と p95 latency を測る。

ASR worker は control loop と別 process にし、GPU OOM、model reload、長い入力が Go2 の controller deadline を阻害しないようにする。worker は warm state を維持し、起動直後の model load 中は `VOICE_NOT_READY` とする。

推奨 pipeline:

```text
decode → duration/format check → audio quality → VAD
  → ASR（Japanese fixed、condition_on_previous_text=false）
  → normalization/N-best evidence
  → intent parser
  → confidence calibration
  → GoalSpec validation
```

各 PTT utterance は独立して認識し、以前の発話を Whisper prompt として引き継がない。過去文脈による hallucination を避け、ロボット状態は ASR ではなく parser/validator の明示 input として使う。

### 7.2 confidence は単一値ではない

Whisper の内部 score をそのまま「95%正しい」と表示してはならない。少なくとも次を feature とする。

- segment `avg_logprob`
- command token の word probability
- `no_speech_prob`
- language probability
- VAD の speech duration / total duration
- RMS、SNR 近似、clipping ratio
- beam/N-best 間の intent margin
- exact grammar match か fuzzy match か
- 現在の Mission FSM と intent の整合性
- target stair reference が一意に解決できるか

これらから `asr_quality_score`、`semantic_score`、`context_score` を分けて記録し、最終 `intent_confidence` は held-out corpus で logistic/isotonic calibration する。未校正の threshold を実機許可条件にしない。

受理帯の考え方:

| 判定 | 処理 |
|---|---|
| 高 confidence、状態整合、target 一意 | readback/confirmation へ |
| 中 confidence、上位候補が複数 | 候補を表示し「上る、でよいですか」と確認。まだ GoalSpec を実行しない |
| 低 confidence、無音、OOD | 拒否して再発話を求める |
| `STOP_NOW` 候補 | stop 専用の高 recall detector で判定し、受理後は確認なしで即ルーティング |

STOP の受理 threshold は task より低くし、見逃しを抑える。ただし「止まらず」「止まったら」「『止まれ』と言わないで」のような minimal pair を単語一致で STOP にしてはならない。stop 専用 detector も clause/negation を含む semantic classifier と corpus 評価が必要である。

### 7.3 readback

ASR 結果は次の三つを別々に表示する。

1. 認識文字列
2. 型付き解釈
3. 実行予定と完了後動作

例:

```text
聞き取り: 「その階段を登って、一番上まで行ったら止まれ」
解釈: 現在選択中の階段を上る
完了: 上端平面で安定を確認したら HOLD
状態: 実行確認待ち
```

研究デモの初期段階では次を採用する。

- `STOP_NOW`: readback/確認なし。UIへ赤い ack を返すだけ
- `NAVIGATE_TO_STAIR_APPROACH`: readback を表示し、operator lease と geometry gate が正常なら実行可能
- `ASCEND_STAIRS` / `DESCEND_STAIRS`: 必ず確認を要求。画面ボタンまたは次のPTTで「はい、実行」を受けてから開始
- 中 confidence: 「Aですか、Bですか」の clarification。単なる「はい」で曖昧な候補を選ばない
- confirmation proposalは初期30秒で失効し、その間にrobot state/targetが変化したら再確認。確認後に新しい実行GoalSpecを発行する

音声 readback を行う場合は Mac 上の local TTS とし、録音中には再生しない。騒音下では視覚表示と短い tone を主にし、TTSを聞き逃しても実行状態を誤認しない UI にする。

## 8. GoalSpec

### 8.1 共通 envelope

音声と文字命令は同じ schema にする。以下は `02_TARGET_ARCHITECTURE.md` と同じ canonical schema であり、音声固有の品質値だけを `transcript.evidence` へ追加する。

```json
{
  "schema_version": "1.0",
  "goal_id": "uuid",
  "source": {
    "modality": "VOICE",
    "operator_lease_id": "uuid",
    "session_id": "uuid",
    "utterance_id": "uuid"
  },
  "transcript": {
    "text": "その階段を登って一番上まで行ったら止まれ",
    "language": "ja",
    "model_id": "pinned-asr-model-id",
    "evidence": {
      "asr_quality_score": 0.97,
      "no_speech_probability": 0.01
    }
  },
  "intent": "ASCEND_STAIRS",
  "target": {
    "type": "stairs",
    "ref": "current",
    "resolved_id": "stair-id-or-null"
  },
  "completion": {
    "predicate": "TOP_LANDING_STABLE",
    "terminal_action": "ACTIVE_HOLD",
    "dwell_s": 1.0
  },
  "constraints": {
    "max_speed_mps": 0.25,
    "require_geometry_confirmation": true
  },
  "confirmation": {
    "required": true,
    "status": "CONFIRMED",
    "proposal_id": "uuid",
    "challenge_id": "uuid"
  },
  "confidence": {
    "semantic_score": 0.98,
    "context_score": 1.0,
    "parser_version": "voice-intent-ja-v1"
  },
  "preconditions": [
    "OPERATOR_LEASE_VALID",
    "ROBOT_ARMED",
    "ACTIVE_STAIR_GEOMETRY_VALID",
    "SAFETY_SUPERVISOR_OK"
  ],
  "created_at_utc": "2026-07-13T14:00:00.000Z",
  "expires_after_ms": 5000
}
```

確認待ちの時点では、上記の実行可能GoalSpecをまだ発行しない。ASR/parserは30秒程度のTTLを持つ `GoalProposal(proposal_id, challenge_id)` を作り、確認messageが両IDを参照した時だけ状態を再検証する。その後、新しい `goal_id` と5秒の開始可能期限 `must_start_by` を持つ `CONFIRMED` GoalSpecを発行する。これは実行中skillの寿命ではない。開始後はmission timeout、skill/owner lease、低位command deadlineを別に監視する。これにより、復唱中に期限が切れる問題と、古い「はい」のreplayを分離する。

`confidence` は監査・gate 用 metadata であり、実行器が confidence に応じて危険な parameter を補間する用途には使わない。受理後の skill 名、完了条件、上限値は allowlist された離散値である。

### 8.2 四つの基本変換

#### 階段前まで

```json
{
  "intent": "NAVIGATE_TO_STAIR_APPROACH",
  "target": {
    "type": "stairs",
    "ref": "current",
    "resolved_id": null
  },
  "completion": {
    "predicate": "STAIR_APPROACH_POSE_REACHED",
    "terminal_action": "ACTIVE_HOLD"
  }
}
```

`front_visible_stair` は language reference であり、実行前に perception が一意の stair instance ID と approach pose へ解決する。候補が複数なら clarification し、最も近い階段を勝手に選ばない。

#### 上る

```json
{
  "intent": "ASCEND_STAIRS",
  "target": {
    "type": "stairs",
    "ref": "current",
    "resolved_id": "stair-id"
  },
  "completion": {
    "predicate": "TOP_LANDING_STABLE",
    "terminal_action": "ACTIVE_HOLD"
  }
}
```

#### 下る

```json
{
  "intent": "DESCEND_STAIRS",
  "target": {
    "type": "stairs",
    "ref": "current",
    "resolved_id": "stair-id"
  },
  "completion": {
    "predicate": "BOTTOM_LANDING_STABLE",
    "terminal_action": "ACTIVE_HOLD"
  }
}
```

「前向き」「後ろ向き」など descent mode が明示されても、最終 mode は幾何・policy capability・Safety Supervisor が許可した候補から選ぶ。言語だけで危険な gait を強制しない。

#### 今すぐ止まる

```json
{
  "intent": "STOP_NOW",
  "target": null,
  "completion": {
    "predicate": "IMMEDIATE",
    "terminal_action": "ACTIVE_HOLD"
  },
  "confirmation": {
    "required": false,
    "status": "NOT_REQUIRED"
  }
}
```

同じ envelope を使ってログ・認証・idempotency を維持するが、`STOP_NOW` は通常の task queue、LLM、navigator を通さず、command arbiter の高優先度入力へ直送する。

### 8.3 conditional HOLD と STOP_NOW の分離規則

parser は発話を最低限、主命令、対象、完了条件、完了後動作に分ける。

```text
「その階段を / 登って / 一番上まで行ったら / 止まれ」
 target          skill    completion condition   terminal action
```

規則:

- 単独の停止命令、または「今すぐ」「そこで」「直ちに」を伴う主命令の stop は `STOP_NOW`
- `〜したら止まれ`、`〜まで行って止まれ` は、前半の skill の completion 後 `ACTIVE_HOLD`
- `止まらず上れ`、`止まらないで` は STOP ではない。ただし仕様外の「止まらず」は安全制約を弱めるため task 自体を拒否できる
- 引用、質問、否定、メタ発話は command として実行しない
- 発話途中の訂正は、最後の句だけでなく全体を parse し、曖昧なら clarification
- `上れ` と `下れ` の両方を含む複合 mission は、MVPでは分割確認する。「上ってから下りて」は一つの自動連続 goal にせず、上端 HOLD 後に新しい下降 command を要求する
- robot が既に階段途中にいる状態で新しい navigation goal を受けても開始しない。まず Safety Supervisor の hold/recovery 判断を行う

### 8.4 GoalSpec validation

schema が正しくても、次が満たされなければ実行しない。

- schema/parser/model version が allowlist 内
- operator lease が有効
- `expires_after_ms` による `must_start_by` 内であり、server受理時のmonotonic clockで古い再送ではない
- 同じ `goal_id` / `utterance_id` が未実行
- robot mode と `intent` が整合
- target stair ID と geometry が fresh
- command owner を取得できる
- Safety Supervisor が許可
- confirmationが必要な実行GoalSpecは `CONFIRMED`。`PENDING`はGoalProposal側だけに存在

LLM/VLA は自由文を補助的に解釈できるが、最終 GoalSpec は deterministic schema validator を通り、未知の field/action は fail closed とする。

## 9. STOP_NOW、HOLD、DAMP、物理 E-stop

これらは名前も経路も分ける。本書の説明語 `HOLD` と実装enum `ACTIVE_HOLD` は同じ能動保持を指し、schema/log/UI内部では `ACTIVE_HOLD` に統一する。

| 操作 | 意味 | controller | 代表的な発生源 |
|---|---|---|---|
| `HOLD` | その場で速度ゼロ、能動姿勢保持 | 現在の安定化 controller/policy を維持 | goal 正常完了、軽微な通信異常、operator pause |
| `STOP_NOW` | software 上の最優先中断要求 | arbiter が現在状態に適した HOLD/controlled stop を即選択 | 音声「止まれ」、Space、停止ボタン |
| `DAMP` | トルクを落とす低レベル状態 | balance を失い得る | 一部の critical fault、吊り下げ試験 |
| 物理 E-stop | 独立した危険エネルギー停止系 | safety-rated hardware/system | 専用装置・安全担当者 |

`STOP_NOW` を受けたときの望ましい処理は次である。

```text
STOP_NOW received
  → normal goals を cancel、再開禁止
  → arbiter が Safety Supervisor の latched safe-state request を最優先採用
  → LowCmd actuator server は sole DDS writer のまま
  → Sport平地なら StopMove + active stand
  → LowCmd/階段上なら gait phase別のControlled Stop / safe boundary
  → safe-state ack と state freshness を監視
  → safe boundary不成立・転倒進行などcritical faultのみ別recovery/DAMP判断
```

階段上で LowCmd process を SIGINT 終了して即 DAMP する現在の `RlController.stop()` を、正常 completion や通常の `STOP_NOW` の唯一の実装にしてはならない。

音声 STOP は次の理由で物理 E-stop ではない。

- PTT を押し、発話し、離すまでの人間時間がある
- 選択マイクのcable／USB／Bluetooth routeが切れる可能性がある
- browser permission、codec、HTTPS、LAN、server が必要
- VAD/ASR/parser の誤認識があり得る
- Go2へ届くまで複数の非安全認証 process を通る

したがって実機階段試験では、音声利用者とは別の安全担当者、物理リモコン、落下防止設備、独立停止手段を必須とする。Unitree リモコンによる soft damping も、安全規格上のハード E-stop と同一視しない。

2026-07-13時点の公開Go2 X資料からは、安全ratedな物理E-stop/独立有線停止入力を確認できていない。ここでいう「物理E-stop」は既設装置の主張ではなく要求機能である。Unitree/販売店と適格な安全担当者に構成・認証・LowCmd中の挙動を書面確認し、確立できなければ階段LIVEをNo-Goとする。

## 10. 切断・故障時の挙動

原則は「新しい動作を開始しない」「古い命令を replay しない」である。HMI/音声linkの喪失だけで、階段の全gait phaseへ一律 `ACTIVE_HOLD` を命じない。mid-swingや狭い踏面では、その場停止が継続より危険な場合があるためである。

| 事象 | 検出 | 新規 goal | 実行中の goal | 復旧 |
|---|---|---|---|---|
| 選択マイクのtrack `ended` / device消失（USB抜去、adapter消失、Bluetooth断を含む） | track event + `devicechange` | 全て拒否、pending confirmation破棄 | phase-aware `HMI_LOSS` policy。音声だけの喪失なら既存goalを勝手に再解釈しない | 手動再選択、level test、試験録音、lease再確認 |
| 選択マイクの`mute` / 無音継続 / 接触ノイズ | event + RMS/VAD/quality | 音声goal拒否 | 警告。operator linkも失われた場合だけphase-aware policy | test録音成功まで音声不可 |
| PWA tab hidden/page hide | browser event | 録音をcancel | operator lease規則に従いphase-aware safe boundaryへ | visible後も自動ARM/再開しない |
| WSS heartbeat timeout | server monotonic timer | 全て拒否 | operator lease失効、phase-aware safe-boundary request | 再接続・再認証・再ARM |
| HTTPS upload失敗/timeout | client/server timeout | 実行しない | 既存 goalへ影響させない | 同じutteranceの明示retry。重複実行禁止 |
| ASR worker timeout/OOM | process health | 実行しない | active goalは独立 Safety Supervisor が監視 | worker restart後 readiness test |
| parser/schema error | validator | 実行しない | 既存 goalへ影響させない | 再発話 |
| GoalSpec ack timeout | WSS sequence/ack | 実行済みと仮定しない | server truthを照会。重ねて開始しない | 同じIDでstatus照会 |
| voice gateway再起動 | health/connection | stale goal拒否 | voice非依存のMission/Safetyがphase-aware fault policyを実行 | lease/ARM/confirmationを全てやり直す |
| Go2/DDS/LowState stale | Safety Supervisor | 全て拒否 | voiceと無関係に安全fallback | センサ復旧・明示reset |

選択マイクだけの切断を単独で即DAMPに結び付けない。DAMPが階段上で転落を誘発し得るためである。実行中の反応は、音声link故障とrobot/sensor/policy故障を分離し、次からphaseごとに選ぶ。

- 平地navigation: Controlled Stop→`ACTIVE_HOLD`。
- `AT_BASE_HOLD` / `BOTTOM_HOLD`: そのholdを維持。`TOP_HOLD`はserver-side署名済みrun configのwarning/上限内だけ維持し、上限までに事前承認済みrecoveryへ移る。
- 上り/下りの途中: 現在の支持状態でhold、次の検証済み支持姿勢までのbounded continuation、またはlandingまでの完遂のうち、backendに対応するSIL/vendor evidenceと低段の全phase fault injectionで最も安全と確認した動作。
- LowState、selected backend output/API、actuator制御自体の喪失: HMI lossとは別のcritical fault policy。

「通信が切れたら残り階段を常に完遂」も「その場で常に止まる」も固定規則にしない。選択したsafe-boundary動作には最大時間/距離/段数を持たせ、復旧しても自動再開しない。

heartbeat の初期案:

- PWA → gateway: 5 Hz
- 3回連続欠落または 750 ms 無通信で lease を suspect
- 1.0 s で失効し、phase-aware safe-boundary request
- しきい値は実ネットワークの jitter 測定後に固定
- browser の再接続だけで ARM や task を自動再開しない

## 11. Latency 目標

音声 path の latency は、PTT release から測る。操作者がボタンを押し、発話する時間は含まれないため、数字を E-stop 応答時間として扱ってはならない。

5秒以下の日本語発話、検証済み入力マイク + Mac + 有線LAN/良好な研究LAN + warm RTX 5090 worker を前提とした初期 SLOを次に示す。まずUSB/USB-C有線baselineで測り、AirPods、3.5 mm/adapterは別系列でp50/p95/p99とfailure率を比較する。

| 区間 | p50目標 | p95目標 | 備考 |
|---|---:|---:|---|
| PTT press → recording ready | 50 ms | 150 ms | permission・model load は事前完了 |
| PTT release → audio受信完了 | 80 ms | 200 ms | streaming chunk採用時はさらに短縮可能 |
| decode + quality + VAD | 20 ms | 80 ms | 上限時間を強制 |
| RTX 5090 ASR | 200 ms | 650 ms | model別に実測して選ぶ |
| semantic parse + calibration + schema | 10 ms | 50 ms | LLM network callは禁止 |
| GoalSpec route + ack | 30 ms | 100 ms | 同一LAN、warm process |
| release → readback/判定表示 | 400 ms | 1.0 s | task command |
| release → `STOP_NOW` がarbiter到達 | 350 ms | 0.9 s | 音声としての目標。物理E-stopではない |
| WSS失効判定 → safe-boundary request送出 | 20 ms | 100 ms | heartbeat timeout時間と物理整定時間は別 |

受入時には p99、最大値、timeout件数も記録する。平均だけで合格にしない。ASR model 精度を上げて p95 が悪化する場合、階段 command の semantic accuracy を維持できる最小 model を選ぶ。

`STOP_NOW` の latency は二つに分けてログする。

1. `PTT release → STOP intent accepted`
2. `STOP intent accepted → Safety Supervisor ack → HOLD observed`

後者が速くても、前者や発話時間が長ければ人を保護する緊急停止にはならない。

## 12. 評価コーパス

### 12.1 corpus の構成

実機へ接続する前に、最低 2,000 utterance の project-specific corpus を作る。最初の目安は8人以上、speaker-disjoint の train/calibration/test split とする。個人の声へ過適合しないよう、最終 test speaker を threshold 調整に使わない。

カテゴリ例:

| カテゴリ | 比率目安 | 内容 |
|---|---:|---|
| 基本 task | 30% | approach、ascend、descend、言い換え、丁寧語 |
| `STOP_NOW` | 15% | 止まれ、止まって、ストップ、今すぐ停止 |
| conditional HOLD | 15% | 上まで行ったら止まれ、下り切ったら止まれ |
| minimal pair/否定 | 15% | 止まらず、止まらないで、止まったら、登らないで、引用・質問 |
| confirmation/cancel | 10% | はい実行、違う、キャンセル、もう一度 |
| OOD/会話 | 15% | 雑談、テレビ音、他人の声、意味不明、未登録命令 |

環境条件:

- 静かな室内
- Go2立位中のファン/関節音
- 平地歩行音
- 階段昇降音
- RTX 5090 workstation のファン音
- 複数人の会話、離れた位置のテレビ/スピーカー
- USB/USB-C有線、3.5 mm/adapter、AirPodsの入力device差
- 有線の抜き差し／別port、adapter接触ノイズ、AirPodsのモデル・電池・片耳/両耳差
- Mac の蓋開閉後、sleep復帰後、USB再列挙／AirPods再接続後
- browser Safari/Chrome の採用対象 version

各 sample に次を付ける。

- raw audio または同意済み匿名化 corpus ID
- 正規 transcript
- `command / non-command`
- `STOP_NOW / TASK / CANCEL / CONFIRM / OOD`
- skill、target reference、completion predicate、terminal action
- 発話時の Mission FSM state と、実行可否
- input device/transport/browser/codec/sample rate
- ambient condition
- 正解 GoalSpec または rejection reason

同じ発話でも状態で意味が変わる。例えば上端にいないときの「下りて」は、ASRが正しくても execution validator では拒否される。このため transcript accuracy と state-conditioned execution accuracy を分ける。

### 12.2 adversarial / regression 文

最低限、次を固定 regression set にする。

- 「止まれ」
- 「その階段を登って一番上まで行ったら止まれ」
- 「下りて、下り切ったらまた止まれ」
- 「止まらずに登れ」
- 「止まらないで」
- 「『止まれ』って言ったら止まるの？」
- 「階段は登らないで」
- 「登って、いや、やっぱりやめて」
- 「上ってから下りて」
- 「前の階段ではなく右の階段」
- 「ストップウォッチを見せて」
- 背景音声だけで「止まれ」が聞こえる sample
- PTT前後で文が切れた sample
- clipping、無音、極端に小さい声

「止まらずに登れ」は STOP ではないが、安全制約を無効化しようとするため goal 自体を拒否する、というように、ASR正解と実行許可を分けてラベルする。

## 13. KPI と受入基準

初期研究デモの受入候補を以下とする。これは安全認証値ではない。

### 13.1 offline voice/semantic KPI

| KPI | 受入候補 |
|---|---:|
| `STOP_NOW` recall | 99.5%以上、かつ固定 release corpus で見逃し0 |
| conditional HOLD を即STOPへ誤分類 | 0件 |
| `ASCEND` と `DESCEND` の取り違え | 0件 |
| high-risk task の false execution | 0件 |
| OOD/non-command の実行率 | 0.1%未満、実機許可 corpusでは0件 |
| GoalSpec exact semantic match | 98%以上 |
| command keyword accuracy | 99%以上 |
| low-confidence rejection/clarification | 99%以上 |
| speaker-disjoint testでの性能低下 | calibration split比2 percentage points以内 |

`0件` は corpus に失敗がなかったという意味であり、実世界の失敗確率ゼロを証明しない。sample 数と confidence interval を併記する。

### 13.2 system KPI

| KPI | 受入候補 |
|---|---:|
| PTT release → GoalSpec/readback | p95 1.0秒以下、p99 1.5秒以下 |
| PTT release → STOP_NOW arbiter ack | p95 0.9秒以下 |
| 同一utterance retryによる二重実行 | 10,000 retryで0件 |
| 選択マイクの消失検出（有線抜去／Bluetooth断） | p95 1.0秒以下 |
| WSS lease失効後の新規goal受理 | 0件 |
| reconnect時の自動ARM/goal replay | 0件 |
| wrong-device silent fallback | 0件 |
| 8秒超の録音受理 | 0件 |
| ASR worker failure時のrobot command開始 | 0件 |

### 13.3 end-to-end KPI

voice と locomotion を混ぜず、次を別々に数える。

1. 発話が正しい GoalSpec になったか
2. GoalSpec が正しい precondition で受理・拒否されたか
3. skill が成功したか
4. completion detector が上端/下端を正しく認識したか
5. completion 後に DAMP ではなく HOLD になったか
6. STOP_NOW/切断時にSafety Supervisorが検証済みphase-safe boundaryを観測したか。平地または既に安定stanceなら`ACTIVE_HOLD`を要求する

音声認識成功後に階段方策が失敗した run を ASR failure にせず、failure domain を分けて記録する。

## 14. Logging、privacy、security

ローカル ASRを採用し、音声をクラウドへ送らない。通常 run では raw audio を認識後に削除し、評価 corpus 収集モードだけ、操作者の同意と明示フラグの下で保存する。

常時残す metadata:

- UTC/monotonic timestamp
- session/utterance/goal/operator lease ID
- 選択 device label の匿名化 ID、browser、codec、duration
- audio quality metrics。raw waveform は既定で残さない
- ASR model hash、runtime/version、decode options
- transcript、ASR evidence、calibrated confidence
- parser/schema version
- GoalSpec、confirmation、validation/rejection reason
- WSS/HTTPS/ASR/arbiter/HOLD 各 latency
- Mission FSM と Safety Supervisor の state
- robot run ID と最終 outcome

server 側に必要な防御:

- TLS、認証、単一 operator lease
- same-origin WSS、Origin検査
- `/voice` の per-lease rate limit、短い size/duration limit
- decodeをrobot control processから分離
- malformed media、decode bomb、極端に長い音声をfail closed
- transcriptや音声をHTMLへ表示する際のescape
- DDS robot subnetをcockpit LANへbridgeしない
- GoalSpec allowlist、schema validation、expiry、idempotency
- audit logは追記型とし、秘密情報やraw bearer tokenを記録しない

## 15. 実装ロードマップ

### Phase V0: 安全な shadow mode

- HTTPS/WSSを導入
- operator authentication/leaseを導入
- PWAにpermission、device picker、入力transport表示、level meter、USB/adapter/Bluetooth disconnect監視を追加
- RTX 5090 CUDA ASRを別processで起動
- ASR evidenceとlatencyを返す
- 実行はせず、transcriptとGoalSpec候補をログする shadow mode
- corpusを収集し、model/thresholdを選定

**Go/No-Go:** MVPはremote Macのsecure contextで検証済み有線マイクを明示選択でき、そのdeviceの抜去時に新規goalが一切生成されなければ進める。AirPodsを使うrunでは、AirPodsの選択、Bluetooth切断、自動route切替、再接続を含む固有試験にも別途合格する。

### Phase V1: GoalSpec と readback

- canonical GoalSpec JSON Schemaを作る
- 文字入力と音声を同じparser/validatorへ統合
- conditional HOLD、STOP_NOW、否定、OODのunit/regression test
- confirmation/readback UIを追加
- utterance/goal idempotencyとexpiryを実装
- `parse_intent()` から直接速度commandを作る経路を無効化

**Go/No-Go:** fixed regression corpusで conditional HOLD の即STOP誤分類0、ASCEND/DESCEND取り違え0、unsafe false execution 0。

### Phase V2: arbiter と active HOLD

- `STOP_NOW` をnormal goal queueから分離
- `HOLD_REQUEST` をSport/LowCmdの両方に実装
- completion `TOP_LANDING_STABLE` / `BOTTOM_LANDING_STABLE` からHOLDへ遷移
- `DAMP` を正常completionから除外
- WSS/lease/voice lossとSafety Supervisorを結ぶ
- mock/replayでduplicate、timeout、reconnect、ASR crashをfault injection

**Go/No-Go:** 全切断試験で再実行やDAMPへの誤遷移がなく、検証済みphase-safe boundaryが観測される。平地／既に安定stanceではactive HOLD、階段途中では署名済みphase別time/distance/step boundを満たす。

### Phase V3: 監視下の実機試験

- 吊り下げ状態で STOP_NOW、切断、device switchを試験
- 平地でnavigation goalとHOLD
- 5 cm単段、10 cm単段、2段、3段、4段の順に上り
- 上端HOLD後に別の下降音声命令
- 下降は同じ試験ラダーで実施
- 各試験に安全担当者、物理リモコン、落下防止、マットを配置

**Go/No-Go:** 音声・GoalSpec・skill・completion・HOLDの各KPIを分離して満たし、物理停止手段を使わずに済んだという事実だけで安全性を主張しない。

## 16. 実装時に触れる主な場所

将来の実装候補であり、この設計文書作成時点では変更していない。

| 場所 | 主な変更 |
|---|---|
| `cockpit/static/index.html` | device picker、mic state、level、readback/confirmation |
| `cockpit/static/app.js` | secure-context診断、明示deviceId、PTT FSM、disconnect監視、timeout/idempotency |
| `cockpit/voice.py` | ASRだけに責務を絞り、CUDA worker、evidence、quality metrics |
| `cockpit/server.py` | HTTPS/WSS gateway、auth/lease、GoalSpec ack、rate/size/duration limit |
| 新規 `common/goalspec.py` 等 | schema、validator、idempotency、expiry |
| 新規 voice semantic parser | clause/negation、STOP_NOW、completion HOLD |
| command arbiter / Safety Supervisor | priority、HOLD_REQUEST、fault handling |
| `cockpit/rl_bridge.py` | 正常HOLDとSIGINT/DAMP終了の分離 |
| tests / corpus manifest | regression audio、GoalSpec exact match、fault injection |

`m1_agent/voice_input.py` と cockpit の二重ASR実装は、voice gateway clientへ薄く置き換える。ASR modelとparserを二系統で持つと、同じ発話が入口によって異なる GoalSpec になるためである。

## 17. 実機前チェックリスト

- [ ] cockpit originがHTTPSで、WSSも同一origin
- [ ] 証明書がMacから信頼され、期限内
- [ ] operator authenticationと単一leaseが有効
- [ ] PWAに選択中のdevice名とtransport（USB/analog adapter/Bluetooth）が表示される
- [ ] input level testと試験録音に合格
- [ ] 選択device消失時の内蔵マイクへのsilent fallbackを禁止
- [ ] 有線は抜け止め／strain relief、AirPodsは自動切り替え抑制を確認
- [ ] PTT以外の音声を受理しない
- [ ] RTX 5090 ASR workerがwarm/ready
- [ ] model/parser/schema hashが記録される
- [ ] conditional HOLD regressionに全合格
- [ ] STOP_NOWの専用経路とackを確認
- [ ] ASCEND/DESCENDはconfirmation必須
- [ ] WSS切断、有線抜去／AirPods切断、ASR停止で新規goalが開始されない
- [ ] 切断時のphase-safe boundaryをmock/吊り下げで確認。平地／既に安定stanceではactive HOLDになる
- [ ] completion後がHOLDであり、DAMPでない
- [ ] reconnect後に自動ARM・自動再開しない
- [ ] 音声とは独立した停止担当者・物理手段・落下防止がある
- [ ] run logと音声評価IDが紐付く

## 18. 参考資料

- Apple: [Macでサウンド入力設定を変更する](https://support.apple.com/en-gb/guide/mac-help/mchlp2567/mac)
- Apple: [AirPodsの自動切り替え](https://support.apple.com/en-ie/guide/airpods/dev228ba3df8/web)
- Apple Developer: [Bluetooth high-quality recording option](https://developer.apple.com/documentation/avfaudio/avaudiosession/categoryoptions-swift.struct/bluetoothhighqualityrecording)
- MDN: [`MediaDevices.getUserMedia()`](https://developer.mozilla.org/en-US/docs/Web/API/MediaDevices/getUserMedia)
- MDN: [`MediaDevices.enumerateDevices()`](https://developer.mozilla.org/en-US/docs/Web/API/MediaDevices/enumerateDevices)
- MDN: [`MediaDevices.devicechange`](https://developer.mozilla.org/en-US/docs/Web/API/MediaDevices/devicechange_event)
- MDN: [`MediaRecorder.isTypeSupported()`](https://developer.mozilla.org/en-US/docs/Web/API/MediaRecorder/isTypeSupported_static)
- faster-whisper: [SYSTRAN/faster-whisper](https://github.com/SYSTRAN/faster-whisper)
