# GO2 COCKPIT — ブラウザ統合コックピット

カメラ / LiDAR 3D点群 / ハイトマップ / テレメトリ をリアルタイム表示し、
Sport歩容(純正)の速度コマンドをキーボード/ボタンで送る統合UI。M0テレオペのUI版。

## 起動

### Apple Silicon Mac

初回だけ、リポジトリ直下でMac用の固定環境を作る:

```bash
scripts/setup_macos.sh
```

実機を動かさず、状態・カメラ・LiDAR・ハイトマップ・UIを確認する:

```bash
GO2_IFACE=en10 .venv/bin/python -m m0_teleop.check_robot --video --lidar
COCKPIT_NO_VOICE=1 cockpit/launch.sh --real --read-only
```

`--read-only`ではUIのARMがLOCKEDになり、移動・姿勢・DAMP・LowCmdと
LiDAR keepalive publishをサーバ側で遮断する。確認後、周囲を無人にして物理E-stopを
手元に置いた場合だけ通常実機モードを使う:

```bash
cockpit/launch.sh --real       # 起動直後はDISARM
cockpit/launch.sh --stop
```

Macランチャーは`.venv/bin/python`、`route -n get`によるNIC検出、Chrome/Safariの
`open`に対応する。既定bindは`127.0.0.1`で、LANへ無認証公開しない。

検証済み境界（2026-07-18）:

- LowState / SportModeState / 前面カメラ / LiDAR点群 / LiDAR odom: Mac実機で受信成功
- Cockpit WebSocket / 点群表示: Mac実機で確認
- Sport Move / StopMoveの実走: 安全な無人スペース未確保のため未実施
- 音声: 依存導入・importまで。初回実機確認は`COCKPIT_NO_VOICE=1`を推奨
- LowCmd / RL LIVE: Mac実機未検証のためNo-Go（dry-runのみ）
- NaVILA 8B/CUDA: Mac単体対象外。Linux/5090サーバを利用

### Linuxデスクトップ

**デスクトップの「GO2 コックピット」アイコンをダブルクリック**するだけ
(またはアクティビティ検索で "GO2")。サーバが未起動なら自動起動し、
Chromeのアプリウィンドウで開く。アイコン右クリックで「Mockモードで起動」「サーバ停止」。
初回クリック時にGNOMEが「起動を許可しますか?」と聞いたら右クリック→「起動を許可する」。

**ロボットに繋がっていない場合**は「Mockモードで起動しますか?」と聞かれる。
Mockでも カメラ(合成)/LiDAR/ハイトマップ/音声/AI任務まで全機能を試せる。

手動起動する場合:
```bash
cd ~/development/real_mac_GO2
cockpit/launch.sh              # 自動判定(ロボットが居れば実機/居なければMockを提案)
cockpit/launch.sh --mock       # 常にMock
cockpit/launch.sh --real       # 常に実機(繋がらなければエラー)
cockpit/launch.sh --real --read-only # 実機センサのみ・全actuator遮断
cockpit/launch.sh --stop       # サーバ停止
python3 -m cockpit.server --mock                # サーバのみ直接起動
GO2_IFACE=enp46s0 python3 -m cockpit.server     # 実機・NIC明示
```
環境変数: `COCKPIT_PORT`(既定8080) / `COCKPIT_NO_BROWSER=1`(サーバのみ) / `GO2_IP` / `GO2_IFACE` /
`GO2_LIDAR_CLOUD_TOPIC` / `GO2_LIDAR_ODOM_TOPIC`

ランチャー利用時はブラウザで **http://localhost:8080**。安全のため`127.0.0.1`だけに
bindし、同一LANの別PCからもアクセスできない。`cockpit.server`を直接起動した場合の既定は
`0.0.0.0`だが、認証機能がないため信頼できないLAN以外へ公開しないこと。
サーバログ: `/tmp/go2_cockpit_8080.log`（ポート番号ごとに分離）
関連ファイル: `cockpit/launch.sh` / `~/.local/share/applications/go2-cockpit.desktop` /
`~/デスクトップ/go2-cockpit.desktop` / アイコン `cockpit/static/icon.png`

## ネットワーク要件(実機モード)

実機モードは **Unitree SDKのDDS通信**を使うため、**ロボットに到達できるNICが必須**。
「サーバ起動失敗」の大半はここが原因(`/tmp/go2_cockpit_8080.log` に
`does not match an available interface` / `channel factory init error` が出る)。

チェック順:
```bash
ip -br link show            # enp46s0 が NO-CARRIER ならケーブル抜け or ロボット電源OFF
ping 192.168.123.161        # 通らなければDDSも通らない
```
- 標準は **有線直結**(PC側 192.168.123.x/24, Go2 = 192.168.123.161)。
- ランチャーは `ping` が通ったNICを `ip route get` で自動検出し `GO2_IFACE` に設定するので、
  NIC名を手で書く必要はない(WiFi等でGo2と同一L2にいる場合もそのまま動く)。
- ロボットが居ないときは **Mockモード**で全機能を試せる(実機コマンドは一切送られない)。

## 操縦手順(実機)

1. Go2をアプリで通常の立位状態にしておく(Sportモード)。
2. コックピットを開き、右上のバッジが **REAL** / pose が `lidar_odom` になっているのを確認。
3. **ARMスイッチをON**(これをするまで移動コマンドは一切送信されない)。
4. キーボード: `W/S` 前後, `A/D` 平行移動, `Q/E` 旋回, `Space` 停止。
   画面のパッドボタン(押している間だけ動く)でも同じ。
5. 速度スケール(0.2〜1.0)で最大速度を調整。初回は0.2〜0.4推奨。
6. 終わったらARMをOFF(自動で停止コマンドが送られる)。

## 音声操縦 🎤

**「🎤 押して話す」ボタンを押している間だけ録音**され、離すと認識→実行される。
サーバ側の faster-whisper(small, CPU)で文字起こし → ルールベースで解釈(VLM不使用・決定的)。
認識に約2〜3秒。移動系コマンドはARM中のみ実行(姿勢系も同様)。「止まって」は常に有効。

| 話す例 | 動作 |
|---|---|
| 「前に進んで」「まっすぐ」 | 前進3秒(既定) |
| 「5秒前進」「ずっと歩いて」 | 前進5秒 / 8秒(上限) |
| 「ゆっくり下がって」「速く前へ」 | 速度0.5x / 1.5x |
| 「右に曲がって」「左旋回」 | 旋回3秒 |
| 「右に平行移動」「左にスライド」 | 平行移動 |
| 「止まって」「ストップ」 | 即時停止(音声コマンドもキャンセル) |
| 「立って」「伏せて」「おすわり」 | 姿勢変更 |

- 音声実行中は残り時間が表示され、**Space/停止ボタン/「止まって」で即キャンセル**。
- キーボード/パッド操作は音声コマンドより常に優先(手動介入で音声は破棄)。
- マイクはブラウザのセキュア制約により **http://localhost で開いたときのみ使用可**
  (別PCから `http://<IP>:8080` で開くと映像・操縦は使えるがマイクは不可)。
- モデル変更: `--whisper tiny|base|small|medium`(既定small)。無効化: `--no-voice`。

## AI任務 🤖 (VLA: 自然言語 → カメラ+LiDAR → 行動)

操縦パネルの入力欄に自然言語で指示を書いて **▶ AI実行**(またはEnter)。
例: 「前方に見える階段の前まで行って」「部屋の入口を探して、そこまで移動して」

仕組み(`cockpit/mission.py`):
- `claude -p`(Claude Codeヘッドレス, stream-json持続セッション)をプランナとして起動。
  APIキー不要 — Claude Codeのログインをそのまま使う。1ミッション=1セッション(文脈維持)。
  既定モデルは **claude-sonnet-5**(`--vlm-model haiku` で高速化も可)。
- 毎ステップ、①前面カメラ画像(768px縮小) ②ハイトマップ画像(ロボット中心・上=前方)
  ③数値コンテキスト(前方障害物距離・観測率・現在速度)を渡し、
  次の1手 `{move|turn|stop|done, vx, wz, reason}` をJSONで受け取り実行する。
- 判断レイテンシは約8〜15秒/手。**判断待ちの間は直前の速度を
  最大0.8秒だけ維持し、LiDAR guardianが10 Hzで各commandを再検査** —
  障害物・未観測回廊・センサstale時は保持期限を待たず即停止する。
- **プランナは一時ディレクトリをcwdにして起動する。** リポジトリ直下で起動すると本プロジェクトの
  CLAUDE.md/メモリ(開発時の取り決め)を読み込み、操縦判断を拒否することがあるため。
- VLMがJSON以外を返した場合は安全側に倒して `stop` になり、生応答がUIログに出る。

安全(テレオペと同じゲート+追加制約):
- 開始はARM中のみ。**DISARM/停止/DAMP/Space/「止まって」で即中断**。
- 速度はさらに保守的にクランプ(vx≤0.3, |wz|≤0.6)。ミッション全体タイムアウト180秒。
- 全判断を `deploy_log.jsonl` に記録。UIのログに各ステップの判断理由と所要時間を表示。

オプション: `--vlm-model <Claude CLIのモデル名>`(未指定時はコード上の`claude-sonnet-5`)。
音声で長い指示を話すと(単純コマンドに該当しない場合)自動で入力欄に転記される。

## 🗺 自律探索マッピング (音声/自然言語 → 契約パーサ → EXPLORE MAP パネル)

VLM を使わない**決定的**な自然言語/音声経路(docs/12)。エッジ機単体で完結する。

- 入口は3つ、意味論は同一: EXPLORE MAP パネル / AI任務欄の探索文 / 🎤PTT音声。
  いずれも `voice_gateway.intent_parser`(限定文法)で解釈される。
  例: 「部屋を探索してマップを作って」「全部探索して」。
- **移動を伴う指示は必ず復唱確認**(パネルの「✓確認」または音声「はい」)。
  確認期限30秒。**「止まれ」は確認なし・ARM不問で即時停止**(自動再開なし。
  再開には新しい指示+確認が必要)。命令でない発話は従来のテレオペ解釈へ落ちる。
- 実行経路は `GoalSpec → Mission FSM → MissionAgent → ExplorationController →
  CollisionGuard/keeper → RobotBridge → Sport Move` の一方向。探索はfrontier方式で、
  LiDAR点群を z帯分類(床=FREE ray/障害=OCCUPIED/落差=hazard)して
  2D占有格子(0.10m, odom系)を構築する。unknown≠free、LowState/姿勢/
  点群/pose異常はfail-closedで停止。速度はvx≤0.20 / |wz|≤0.45に固定クランプ。
- frontier 枯渇で `EXPLORATION_COMPLETE` → 立位保持(ACTIVE_HOLD)。地図は
  `artifacts/maps/<map_id>/map.json` に保存され、`home`はartifact上の参照点として
  登録される。現在のLIVE runnerではwaypoint帰還は無効。完了は到達可能frontier枯渇を
  3つの別LiDAR scanで確認して確定する。
- 中断経路: DISARM / 停止 / DAMP / Space / WS切断 / roll・pitch超過 /
  lowstate・pose・点群途絶 / タイムアウト180s — いずれも Controlled Stop。
- 探索中の自動登坂はOFF。別GoalSpec・明示確認・制御authority handoffが実装されるまで
  探索runnerから階段タスクを自動起動しない。
- Mockモードで全機能を試せる(背面壁で閉じた合成部屋を探索し完了まで到達する)。

> 実機での初回は docs/12 §8 / docs/13 §6 を必読: Gate 3 相当の停止確認(各停止経路で1s以内
> 静止)を先に行い、狭い既知区画・人の立入なしで開始すること。

## 🪜 段差登坂タスク (LiDAR + カメラ → 自律登坂)

ハイトマップ下のバーに検出結果が常時表示される(5Hz更新)。緑=登坂可能、赤=危険。
検出したエッジはハイトマップ上に線で描かれる。**「🪜 登る」**で自律登坂が始まる。

```
段差: 高さ0.12m 距離0.60m yaw-0.01 幅0.70m   [x]カメラ確認 [x]連続  [🪜 登る] [✕]
```

**2層のセンシング** — どちらか一方でも危険と判断すれば登らない:
1. **LiDAR幾何検出**(`cockpit/stair.py`, 主センサ・決定的)
   標高マップの前方プロファイルから エッジ距離 / 段高 / エッジ傾き(yaw誤差) / 幅 を推定し、
   `none / step / stairs / wall / drop` に分類。単体テスト: `python3 -m cockpit.stair`
   - L1点群の外れ値(実測でz=-14m級)を除去、椅子脚のようなスパイクは踏面の平坦性と
     左右方向の直線性(幅0.35m以上)で棄却する。
2. **カメラ + VLM確認**(`--vlm-model`、既定ON)
   接近完了後にカメラ画像を見せ「登ってよい段差か」を判定。人・動物・家具・暗すぎる等は拒否。
   実例: 幾何は「段高0.12m」と判定 → カメラが「段ではなくオフィスチェア、人の脚も近接」と拒否。

**状態遷移**: `SCAN → ALIGN(正対) → APPROACH(0.33mまで接近) → CONFIRM(カメラ) → CLIMB → SETTLE`
連続チェックONなら次段があれば繰り返す(最大8段)。

**安全**:
| 条件 | 動作 |
|---|---|
| 段高 > 0.16m (Go2公式スペック) | **拒否** — M3の学習方策(`rl_stair_controller`)へ誘導 |
| 手前に落差(drop)を検出 | 開始拒否 / 接近中なら即中断 |
| 幅 < 0.35m(物体の可能性) | 段差とみなさない |
| DISARM / 停止 / DAMP / Space | 即中断 |
| \|roll\|>0.5 or \|pitch\|>0.7 rad | 即中断 |
| lowstate途絶 / タイムアウト(240s) | 即中断 |
| WSクライアント切断 | 即中断 |

速度は接近0.22 / 登坂0.20 m/s に固定(通常テレオペより低速)。全イベントを `deploy_log.jsonl` に記録。

> 純正歩容で登る場合、**スマホアプリで階段モードをONにしておく**と成功率が上がる
> (SDKに階段モード切替APIは無いため)。

## 🧠 学習方策で登る (M3 / 0.20m級) — UI統合版

**ハイトマップ下の「🧠 学習方策(M3)」バーから、UIだけでRL登坂ができる。**
0.16m超の段は純正歩容(🪜 登る)が拒否するので、こちらを使う。

流れ(`cockpit/stair_task.py` の backend=rl / `cockpit/rl_bridge.py`):
1. **純正歩容**で段差へ整列・接近(≤0.33m)し、カメラ確認(任意)
2. `stand_down → ReleaseMode`(**sport解除**)→ 立位ランプ3s → 方策50Hz起動
   （方策の実体は監査済みの `m3_rl/rl_stair_controller.py`。コックピットは起動/監視/停止のみ）
3. コックピットの前進速度指令が **UDP経由で方策の velocity_commands** になる
4. base_z上昇が止まり水平姿勢に戻ったら頂上と判定 → **SIGINT → 2秒Damp** で脱力
5. 「Sport復帰」ボタンで純正歩容へ戻す(解除前のモードを記憶して復元)

**必ず dry-run から。**
- `dry-run`(既定ON): 方策は推論するが**モータへLowCmdを送らない**。obs・立位ランプ・
  頂上判定の配線を機体を動かさず確認できる。
- チェックを外すと確認ダイアログが出て、**● LIVE**(実弾)になる。ボタンも赤くなる。

**base_lin_vel問題への対処** — 方策の観測に base線速度が含まれ、実機性能を左右する。
sport解除後は sportmodestate が止まるため、コックピットが **LiDARオドメトリを微分した
world線速度** を height_scan と同じUDPパケットで配信し、`rl_stair_controller --linvel auto`
が「sms生存中はsms / 途絶したらodom微分 / それも無ければ0」と自動で切り替える。

**安全**(純正登坂の全ガードに加えて):
| 条件 | 動作 |
|---|---|
| 段高 > 0.25m(訓練分布外) | 拒否 |
| height_scan観測率 < 35% | 拒否(地図が不足) |
| 停止 / DAMP / Space / DISARM / WS切断 | 方策へSIGINT → Damp脱力 |
| SIGINTで5秒以内に終わらない | SIGKILL + 自前でLowCmd(kp0,kd2)の非常Damp |
| \|roll\|>0.5 / \|pitch\|>0.7 / lowstate途絶 | 中断→Damp |

> **実機で回す前に必ず**: 人間立会い / 物理E-stop手元 / 初回は吊り下げ / 0.10→0.15→0.20の段階投入。
> sim2sim(unitree_mujoco)で先に確認するのが最も安全。

### 手動でCLIから回す場合(UIを使わない)

コックピットは height_scan(187点)を `elevation_node` と同契約でUDP配信しているので、
`elevation_node` を別途起動せずにRL方策を走らせられる:
```bash
# コックピット起動のまま、別ターミナルで(sportと低レベル制御は排他)
python3 -m m3_rl.rl_stair_controller --hs elev --linvel auto --dry-run   # 予行
python3 -m m3_rl.rl_stair_controller --hs elev --linvel auto             # 本番
```
配信を止めたい場合は `python3 -m cockpit.server --no-publish-hs`。

## キーボードが効かないとき

- **日本語入力(IME)がONでも動くように物理キー判定にしてある**が、
  他アプリにフォーカスがあると届かない → コックピットの画面を一度クリック。
- DISARM中はキーを押してもログに「ARMしてください」と出るだけ(仕様)。

## 安全設計

| 仕組み | 内容 |
|---|---|
| ARMゲート | 起動時DISARMED。ON にするまで move は送信されない |
| コマンド途絶 | 0.5s コマンドが来なければ自動 stop_move(キー離し/フリーズ対策) |
| クライアント切断 | ブラウザが落ちたら即 stop_move |
| 速度クランプ | `common/config.py` の VEL_LIMIT でサーバ側でも制限 |
| 停止/DAMP | ARM状態に関係なく常に受付。**DAMPは全関節脱力=立位から崩れる**(緊急用) |
| 記録 | ARM/コマンド/アクションを `deploy_log.jsonl` に記録 |

※ 本UIは高レベル(Sport)専用。M3のRL低レベル制御は従来通り
`m3_rl.rl_stair_controller` を使う(併用不可: sportと低レベルは排他)。

## 画面

- **前面カメラ**: MJPEGストリーム(実機はロボットのJPEGをそのまま中継)。`/snapshot` で静止画。
- **LiDAR点群**: `rt/utlidar/cloud_deskewed`(odom系)を最大8000点/5Hzで3D表示。
  点は2.5cm voxelでworld上へ蓄積し、再観測した面を明るく、履歴を暗く表示する。
  ドラッグ=回転 / ホイール=ズーム。緑ワイヤ=機体, 橙コーン=前方。
  下端の `ODOM raw→UI` は実機から受けたframeとフィルタ前後の点数。`ERROR` / `EMPTY` の場合は
  マウスを重ねて詳細を確認し、poseが`lidar_odom`、frameがodom系であることを確認する。
- **ハイトマップ**: 点群から作るローリング標高格子(8m四方, 0.1m表示格子)。
  `m2_navila.elevation_node` と同じ `RollingElevationMap` を使用 = **M3方策が見る世界と同じ**。
  白破線 = 方策のheight_scanフットプリント(体周り1.6m×1.0m)。マウスホバーで高さ読取。
- **テレメトリ**: 姿勢儀(roll/pitch)、RPY/速度/位置/体高、関節q/dq/τ(SDK順12関節)。
- **pose ソース**: 実機の自律走行/点群統合は`lidar_odom`(rt/utlidar/robot_odom)のみ。
  freshなposeが無い場合はSMSへ代替せず、scanを破棄してfail-closed停止する。

MockのLiDARは床だけでなく、左右壁・ドア開口・階段の垂直な蹴上げ・高さの違う箱を含む。
ローカルで3D空間が読めるかを確認するfixtureとして使える。

## 実装メモ

- backend: `cockpit/server.py` — aiohttp。WS(10Hzテレメトリ + 5Hzバイナリ) + MJPEG。
  バイナリフレーム: `[u8=1][u32 n][f32 xyz×n]`=点群 / `[u8=2][f32 cx,cy,res][u16 n][f32 h n×n]`=標高格子。
- frontend: `cockpit/static/` — three.js(同梱, r128)。WebGL不可環境では3Dのみ無効化し他は動作。
- コマンド送信は専用スレッド10Hz(`sport_teleop.spin` と同じ思想)。
