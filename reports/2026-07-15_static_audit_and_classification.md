# 静的監査・現状分類レポート — 2026-07-15

- 監査者: Claude Code(docs/CLAUDE.md 作業契約下、RTX PC 初回セッション)
- 対象: `https://github.com/fumito072/GO2.git` ローカルクローン、branch `main`、HEAD `c90ec934fd6e2d93e8413df6e75cd5b7dbc26e76`
- 手法: **読み取り専用の静的監査**(設計文書9本+コード全モジュールの並列読解、主要 finding は行番号までスポットチェック済み)。コード実行・実機接続・ネットワーク操作は一切行っていない。
- docs 監査基準 snapshot(`feat/sf-ui-implementation` @ `a5535be`)と本 HEAD の diff 照合は未実施(NOT TESTED — シェル復旧後に実施)。

```text
Current Phase / Gate:  Phase 0 一部完了(platform readiness/hash 凍結 DONE、実機系は BLOCKED)/
                       Gate 0 素材(contracts + arbiter + 停止遷移表)offline 実装・全テスト PASS。
                       physical E-stop 未同定のため階段 LIVE は NO-GO(08 §2.5)を維持。
Goal of this change:   初回セッションの静的監査・現状分類・Phase 0 成果物・Phase 1 最初の contract 実装。
Files changed:         新規ファイルのみ(reports/, phase0/, contracts/, mission/, safety/, tests/, scripts/)。
                       既存ファイルは非変更。
Tests run:             offline contract test 137/137 PASS(unittest, robot 非接続)。
                       legacy baseline: cockpit.stair 11/11 PASS(要 -X utf8)、
                       test_lidar_pipeline 12/12 PASS、joint_map PASS、
                       voice PASS(assert なし=参考値)、test_obs_builder は torch 未導入で NOT RUN。
Evidence:              §2 の分類表 + §9 の実行結果追記。実機系はすべて NOT TESTED / BLOCKED のまま。
Safety impact:         既存コード変更なし。§3.1 危険事項 9/9 実在確認 → LIVE 禁止判断を維持。
Open blockers:         §6 参照(E-stop / vendor 回答 / 実機環境)。
Next safest action:    WSL2 Ubuntu-24.04 に go2-runtime 分離環境を構築(robot 非接続)、
                       Exclusive Actuation Gateway の offline 実装。
```

---

## 1. Phase/Gate 構造(正本: 04=Phase、08=Gate/KPI)

| Gate | 対象 | 合格条件(要点) |
|---|---|---|
| 0 | 静的監査・単体(robot非接続) | joint manifest 検証、obs/action property test、arbiter priority/expiry/latch 網羅、停止状態遷移表全 edge。No-Go: 例外握りつぶし、複数 LowCmd writer、未定義 transition |
| 1L | Branch L SIL(Isaac Lab) | 各方向 10,000 episode 以上、success ≥99%、uncontrolled fall <0.1%、STOP injection 1,000回/方向で転倒0・HOLD 30秒維持 100% |
| 1S | Branch S vendor evidence | 正式 stair API(開始・方向・状態・停止・timeout・remote override)の文書化。`Move()` 代用禁止 |
| 2L | Branch L HIL(吊り下げ・非接地) | TX 500Hz 60分: interval p99 ≤3ms、max ≤10ms。policy inference p99 ≤18ms。owner kill 時 bounded response 実測 |
| 2S | Branch S 吊り下げ/平地 | exclusive gateway、全遷移点 fault 注入、StopMove 後の能動 balance、独立停止 end-to-end 外部計測 |
| 3 | 平地・低速・落下拘束 | start/stop/terminal active balance 100回転倒0、STOP後1.0秒以内 \|v_xy\|<0.03m/s |
| 4 | 単段上り 5→8→10cm | 各条件20回連続成功、STOP injection 各10回 HOLD 成功、段高 MAE ≤1cm、yaw ≤3° |
| 5 | 2/3/4段昇段 | 各段数 20/20、false top complete 0、TOP_HOLD(初期60秒)20/20 |
| 6 | 自然言語接近 | 50 run で ≤5cm/≤3° を 49/50 以上、edge crossing 0、VLM timeout を climbable=true に変換しない |
| 7 | 4段下降(昇段と独立) | 単段→4段の同 ladder、4段50回連続無事故、false bottom complete 0/100 |
| 8 | 音声入力統合 | STOP recall 100/100、nuisance 1,000発話で hazardous false start 0、方向混同 0/500 |
| 9 | 統合研究デモ(最終受入) | 100 end-to-end run で mission success ≥95%、転倒/誤方向/誤完了 0、全 run 完全 log+video |

運用規則: 上位 Gate 合格は下位 Gate 省略の根拠にならない。C/H hazard 未解決1件でも該当 LIVE Gate は No-Go。03 の Gate A〜F は初期 KPI 案であり正本は 08(04 L183)。

## 2. 現状分類(VERIFIED = 静的読解で行番号確認。実行検証はすべて未実施)

| # | 項目 | 分類 | Evidence |
|---|---|---|---|
| 1 | Sport/LowState/camera/MotionSwitcher/LowCmd/Mock 統合 I/O 層の存在 | VERIFIED(静的) | `common/go2_iface.py:48-214, 296-297` |
| 2 | SDK import は関数内遅延(import だけでは robot endpoint を開かない) | VERIFIED(静的) | `go2_iface.py:11,37,50-52,128,143,188-191` / `elevation_node.py:155-178` / `server.py:150-151,459` |
| 3 | 187点 height_scan + rolling elevation map + 段差分類(step/stairs/wall/drop) | VERIFIED(静的) | `elevation_node.py:35-139` / `server.py:118-146` / `stair.py:141-193` |
| 4 | 上り FSM SCAN→ALIGN→APPROACH→CONFIRM→CLIMB→SETTLE | VERIFIED(静的) | `stair_task.py:206-280`(sport)、`283-432`(rl) |
| 5 | Wave5 policy artifact(policy.pt / policy_wave4_maxclimb.pt / policy_spec.json / env.yaml / agent.yaml)の存在 | VERIFIED(静的) | Glob 確認。SHA-256 は docs/01 L67-72 記載値 — 本セッションでの再計算は未実施(NOT TESTED) |
| 6 | policy 契約: obs 235次元 / 12関節 / 50Hz / action scale 0.25 / 187点 grid | VERIFIED(静的) | `policy/policy_spec.json:92-888` / `obs_builder.py:3-11` |
| 7 | self-test 群(cockpit.stair 11/11 等)の合格 | NOT TESTED | docs/01 L76-83 の過去記録のみ。本セッション未再現 |
| 8 | Wave5 の実階段(10cm×4段)上り/下り能力 | NOT TESTED | deploy_log 全 record dry:true(docs/01 L182)。sim2sim harness 非同梱 |
| 9 | Wave5 学習・評価の再現 | BLOCKED(NOT REPRODUCED) | 参照先 `../out/wave4/...`、`../go2_stair_rl/harness/ledger.jsonl` が workspace 不存在(docs/01 L184-187) |
| 10 | LIVE LowCmd 階段成功 | NOT TESTED(証拠なし) | docs/01 L182 |
| 11 | 下降 skill・bottom completion | 未実装(VERIFIED: drop 拒否のみ) | `stair.py` の drop 分類 → `stair_task.py:110-111,217-219` が拒否 |
| 12 | 正常終了が Damp 経路(ACTIVE_HOLD 不在) | VERIFIED(静的) | `rl_stair_controller.py:283-294`(finally で kp=0,kd=2 を2秒 — 正常終了含む)/ `rl_bridge.py:164-207` |
| 13 | LowCmd 送信が policy 推論と同一 50Hz Python ループ | VERIFIED(静的) | `rl_stair_controller.py:219-282`(公式例 2ms=500Hz の範囲外) |
| 14 | LowCmd 複数 writer 可能(単一所有権なし) | VERIFIED(静的) | `go2_iface.py:187-214` / `rl_bridge.py:198-210`(第2 publisher 実在) |
| 15 | watchdog は同一プロセス・実効 100ms(config 表記 40ms) | VERIFIED(静的) | `safety.py:63-68` / `rl_stair_controller.py:163-172` / `config.py:49` |
| 16 | perception fail-open(VLM 3経路肯定続行、unknown→flat 充填) | VERIFIED(静的) | `stair_task.py:180-200` / `elevation_node.py:236-238` / `server.py:135` / `obs_builder.py:33,51` / `rl_stair_controller.py:235-240` |
| 17 | cockpit 無認証 0.0.0.0 bind(TLS/lease/origin なし)、LAN から rl_start dry_run:false 可能 | VERIFIED(静的) | `server.py:852,880,712-829` / `navila_server.py:113` も同型 |
| 18 | 音声 parser の「止まれ」部分文字列最優先(複合命令不可、2実装) | VERIFIED(静的) | `voice.py:13,60-62` / `agent_loop.py:32,108-113` |
| 19 | GoalSpec / Mission FSM / Command Arbiter / Gateway / 独立 Safety Supervisor | 未実装(VERIFIED: 不存在) | `contracts/` 不存在。command 経路は WS/voice/mission/stair_task が直接 `set_cmd`(`server.py:477-502`) |
| 20 | 再現性基盤(lockfile/CI/pytest/run manifest/ledger) | 未実装(VERIFIED: 不存在) | `requirements.txt:5-12` 全行未 pin。tests//CI/lock なし |
| 21 | Go2 X の LowCmd 可否・ReleaseMode・純正 stair API・foot_force 意味 | BLOCKED(vendor 確認要) | 04 L47-51 / 03 / 05 L38 |
| 22 | physical E-stop function | BLOCKED(No-Go 要因) | 08 §2.5。repo 内に安全 rated 装置の証拠なし |
| 23 | LiDAR/state topic 5種の実機確認 | ASSUMED(過去記録ベース) | `config.py:16-24` / docs/01 L28-34。現 SKU/firmware での再確認未実施 |
| 24 | ReleaseMode 後の LIO/VIO・SportModeState 可用性 | ASSUMED(未検証) | 05 L37 / `rl_stair_controller.py:76-77,243-255` |
| 25 | mock は watchdog/Damp 経路を検証しない(low_age=0.001 固定) | VERIFIED(静的) | `go2_iface.py:243-245,258` |
| 26 | checkout は main @ c90ec93(docs 監査 snapshot は a5535be) | VERIFIED(静的) | `.git/HEAD`, `.git/refs/heads/main`, `.git/packed-refs`。diff 照合は NOT TESTED |

## 3. docs/CLAUDE.md §3.1 危険事項の実在確認(9/9 確認済み)

| §3.1 項目 | 判定 | Evidence(スポットチェック済み ✔) |
|---|---|---|
| 1. `dry_run` は Sport backend を守らない | 確認済み ✔ | `stair_task.py:91-92`(既定 backend="sport", dry_run=True)、`:206-209`(dry_run は RL 分岐のみに渡る)。sport 経路は dry_run 非参照で実速度コマンド送出 |
| 2. `launch.sh` の実機自動選択 | 確認済み | `launch.sh:3,19-22,45-48,73`(192.168.123.161 へ ping → 到達可なら無確認で実機モード nohup 起動) |
| 3. `cockpit.server` 0.0.0.0 bind 無認証 | 確認済み ✔ | `server.py:852`(既定 '0.0.0.0')、`:880`。認証コードなし。--mock でも同一 bind |
| 4. camera/VLM 確認の fail-open | 確認済み | `stair_task.py:180-181`(CLI 不在→肯定)、`:194-196`(JSON 不正→肯定)、`:199-200`(例外→肯定) |
| 5. 未観測 cell の flat 補完・途絶時 flat fallback | 確認済み | `elevation_node.py:236-238` / `server.py:135` / `obs_builder.py:33,51` / `rl_stair_controller.py:235-240`(途絶→WARN 1回で平地仮定続行) |
| 6. 音声 parser の「止まれ」即時 stop 解釈 | 確認済み ✔ | `voice.py:13,60-62` / `agent_loop.py:32,108-113`。**新規発見**: README_JP.md:42 の公式例文「階段まで行って登って止まって」自体が反射停止に飲まれ実行不能 |
| 7. VLM navigation は velocity primitive の PoC | 確認済み | `mission.py:270-330` / `agent_loop.py:145-166` / `navila_client.py:93-113`。SLAM/planner/approach pose 不在 |
| 8. M3 は推論と LowCmd 送信が同一 50Hz ループ+正常終了でも Damp | 確認済み ✔ | `rl_stair_controller.py:219-282`(同一 while)、`:283-294`(全終了経路が finally で Damp)。README_JP.md:72 が仕様として明記 |
| 9. deploy_log / artifact / sim 成功率は実機成功の証拠でない | 確認済み | `safety.py:71-78`(無条件追記、run ID/model hash なし)、`.gitignore:18`(deploy_log を Git 除外)、README_JP.md:4,100,103-107 |

## 4. 新規発見(§3.1 に未記載の問題)

1. **README 公式例文が音声 reflex で実行不能**(上表 #6)— parser 改修まで README 例文に注記が必要。
2. **watchdog 実効値の乖離**: `config.py:49` は 40ms 表記だが実装は下限 100ms(`rl_stair_controller.py:172` / `safety.py:62`)。08 KPI(LowState age >40ms hard fault)未達。
3. **制御定数の死に値**: `config.py:44-46` の ACTION_SCALE/KP/KD は未参照。真値は `policy_spec.json` 由来(`joint_map.py:31-34`)。config 書換で挙動が変わらない罠。
4. **flat 値の三重定義**: `obs_builder.py:33`(-0.225)/ config / spec flat_sample(-0.22505)。
5. **転倒閾値の不整合**: config MAX_ROLL_PITCH=0.8rad vs `stair_task.py:48-49` の 0.5/0.7rad — より危険な低レベル側が緩い。
6. **`--mock` の意味の不統一**: `navila_client.py:6` は「サーバもモック」と記載だが実際は常に HTTP POST。`server.py:492-493` は mock 中 `bot.move()` を方策の代役にする(mock 専用の別意味 — CLAUDE.md §10 違反)。
7. **`cockpit.voice` self-test は assert ゼロ**(`voice.py:108-116`)— いかなる誤分類でも PASS。受入試験として無効(§6.3 の警告が実在)。
8. **sport 分岐の完了判定は上昇量からの段数推定のみ**(`stair_task.py:256-258,271-274`)— landing 面/全脚検証なし(invariant 12 違反状態)。
9. **UDP 43210/43211 の無 schema JSON が三重実装**(server / elevation_node / rl_stair_controller)— sequence/expiry/署名なし。Phase 1 contract の最初の適用対象。
10. **deploy_log の mock フラグ欠落行**: `navila_client.py:76` の m2_start は mock 有無を記録しない。

## 5. offline test の実行可否(この Windows RTX PC / 静的読解による判定)

### 5.1 Windows で今すぐ実行可能(実機・SDK・Linux 不要)

| コマンド | 依存 | 注意 |
|---|---|---|
| `python -m cockpit.stair` | numpy | 合成地形 self-test(exit code あり)。「緩斜面」case は期待 None=常時合格 |
| `python -m cockpit.test_lidar_pipeline` | numpy | unittest。環境変数 COCKPIT_MOCK_STEP が結果を汚染し得る点に注意 |
| `python -m m3_rl.joint_map` | numpy + policy_spec.json | 往復マッピング assert |
| `python -m m3_rl.test_obs_builder` | torch(CPU可)+ policy.pt | 「配線疎通であり物理の検証ではない」(自己申告) |
| `python -m cockpit.voice` | stdlib | assert ゼロ=常時 exit 0(結果は参考値) |

実行しないもの(mock でも): `cockpit.server`(0.0.0.0 既定 bind。実行する場合は必ず `--mock --host 127.0.0.1 --no-voice`)、`m2_navila.navila_server --mock`(0.0.0.0 bind)、rl_bridge 経由の RL 起動/停止(POSIX 専用: `start_new_session`/`os.killpg`/`SIGKILL`)。`--yes` は付けない(CLAUDE.md §5)。

### 5.2 Ubuntu / WSL2 が必要なもの

| 対象 | 理由 |
|---|---|
| 実機接続全般(non-mock) | unitree_sdk2py + cyclonedds(requirements.txt:2-4 は git install 指示。Windows ビルド未整備) |
| `cockpit/launch.sh` | bash/zenity/ip route/ping -c/pkill/nohup/dev/tcp 依存。※実機自動選択の穴があるため自動化フローから除外 |
| rl_bridge の RL ライフサイクル | POSIX signal 前提 |
| `docs/mermaid/render.sh` | bash + node/npx(Git Bash でも可の見込み) |
| NaVILA / Isaac Lab / MuJoCo sim2sim | Linux 前提 stack(INSTALL_NAVILA_JP.md:7)。sim2sim code はリポジトリ非同梱 |

## 6. Open blockers

1. **physical E-stop function の同定・書面確認**(Phase 0 blocker、階段 LIVE NO-GO 要因)
2. **vendor 確認**(Go2 X の LowCmd/ReleaseMode 可否、純正 stair API、保証条件 → `phase0/api_gate_report.md` の質問票)
3. **Phase 0 実測系成果物**(hardware_manifest / stair_registry / 安全設備 — 実機・現場が必要)
4. **実機接続環境**(Ubuntu または WSL2 + cyclonedds/unitree_sdk2py — 本 PC 上の構築は可能、robot 接続はユーザー承認まで行わない)
5. **docs snapshot(a5535be)と HEAD(c90ec93)の diff 照合**(シェル復旧後)
6. **依存 lock の不在**(requirements 未 pin — go2-runtime 用 lockfile 新規作成が必要)

## 7. docs 間・docs-コード間の主要矛盾(推奨正本つき)

| # | 矛盾 | 推奨正本 |
|---|---|---|
| D1 | Gate 体系の二重定義(03 の A〜F vs 08 の 0〜9) | 08(04 L183 が明示) |
| D2 | 連続成功回数(03「各30回」vs 08 Gate 4/5「各20回」) | 08 |
| D3 | 実機4段 KPI の集計単位(03 割合 vs 08 連続+最終95%) | 08 |
| C1 | README 例文「…止まって」が音声 reflex で即 stop | 06/02 の GoalSpec 設計 |
| C4 | watchdog 40ms(config)vs 実効 100ms(実装) | 08 §11 の 40ms(コードが修正対象) |
| C5 | config の死に制御定数 vs policy_spec.json | policy_spec.json を唯一の真とする |
| C8 | 転倒閾値(config 0.8rad vs stair_task 0.5/0.7rad) | 実測前は厳しい側に統一(08 fault policy で確定) |

## 8. 実行結果 追記(2026-07-15 シェル復旧後)

### 9.1 Platform readiness(VERIFIED — `reports/inventory_2026-07-15_040853.md`)

| 項目 | 値 |
|---|---|
| OS | Windows 11 Pro 10.0.26200(64bit)、RAM 127.5 GB、C: 空き ~1.1 TB |
| CPU | Intel Core Ultra 9 285(24C/24T) |
| GPU | **NVIDIA GeForce RTX 5090、VRAM 32,607 MiB、driver 610.74** |
| CUDA toolkit | 13.2(nvcc V13.2.51) |
| Python | 3.10.11(既定)+ 3.12。**torch 未導入**(→ go2-runtime 環境で導入) |
| WSL2 | **Ubuntu-24.04 あり(Stopped)** — go2-runtime / SDK ビルドはここに分離構築する |
| Docker | NOT INSTALLED |
| Network | Ethernet 192.168.1.2/24 のみ。Go2 専用 NIC 未接続(実機不在 — 想定どおり) |

### 9.2 ハッシュ照合(VERIFIED)

policy artifact 4点の SHA-256 は docs/01 L67-72 の記録値と**完全一致**:
`policy.pt=11ec4446…`, `policy.onnx=0de374d4…`, `policy_wave4_maxclimb.pt=a8076721…`, `.onnx=3ae06ed2…`。
→ 分類表 #5 を「VERIFIED(ハッシュ一致)」へ更新。docs/CLAUDE.md の
「artifact は有望な候補、実階段成功は未証明」の位置づけは不変。
全ハッシュ(policy_spec.json / env.yaml / agent.yaml / docs 11本 / requirements.txt)は
inventory ファイルに凍結記録済み。

### 9.3 offline test 実行結果

| テスト | 結果 |
|---|---|
| `python -m unittest discover -s tests`(新規 contracts/arbiter/停止遷移) | **137/137 PASS**(0.01s) |
| `python -X utf8 -m cockpit.stair` | 11/11 PASS(docs/01 の記録と一致。`-X utf8` なしでは cp932 で UnicodeEncodeError — 新規発見 #11) |
| `python -m cockpit.test_lidar_pipeline` | 12/12 PASS |
| `python -m m3_rl.joint_map` | PASS(round-trip) |
| `python -m cockpit.voice` | exit 0(assert なし=参考値。§4-7 のとおり受入試験としては無効) |
| `python -m m3_rl.test_obs_builder` | **NOT RUN — torch 未導入**(Windows 側)。WSL2 go2-runtime 環境構築後に実行 |

補足確認: cockpit.stair の「緩斜面」case は期待値 None のため h=0.058 の
「stairs」分類でも PASS になる(§監査指摘の実在をログで確認)。

### 9.4 新規発見(追記)

11. **cp932 エンコード問題**: `cockpit/stair.py:278` の出力(em-dash)が Windows
    既定 console で UnicodeEncodeError → self-test が本来の合否と無関係に exit 1。
    回避は `python -X utf8`。恒久対応は将来の整備 task(既存コード非変更方針のため)。

## 9. 次の最小安全作業

1. ~~作業ブランチ・inventory・ハッシュ照合・offline baseline~~ → **完了**(§8)。
2. ~~Phase 1 最初の contract task~~ → **完了**: `contracts/`+`mission/command_arbiter.py`+
   `safety/stop_transitions.py`、offline test 137/137 PASS。multi-agent adversarial
   review 2巡(確定指摘 19 件反映、棄却 3 件)。
3. **次**: WSL2 Ubuntu-24.04 に go2-runtime 分離環境を構築(python venv + torch CPU +
   unitree_sdk2py。robot 接続はしない)→ `m3_rl.test_obs_builder` 実行で baseline 完了。
4. **次**: Exclusive Actuation Gateway の offline 実装(受入条件は contracts/README.md)。
5. BLOCKED 継続: vendor 質問票の送付(phase0/api_gate_report.md)、physical E-stop 同定、
   階段実測(phase0/stair_registry.yaml)— いずれも実機・現場・ユーザー判断が必要。
