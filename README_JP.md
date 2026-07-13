# real_mac_GO2 — 実機 Go2 X 階段デモ 完全ハンドオフ

**このフォルダ1つで M0(テレオペ) → M1(音声+VLM) → M2(NaVILA+標高マップ) → M3(学習方策で0.20m階段) まで動かす。**
学習済みモデル: `policy/policy.pt`（**Wave5**: simで3段登坂成功 0.20m=93.0% / 0.23m=56.3%）。
計画と評価の経緯は `../out/wave4/DEPLOY_GO2_JP.md`、実験台帳は `../go2_stair_rl/harness/ledger.jsonl`。

> **安全（絶対）**: 実機は必ず人間立会い。物理的な緊急停止を手元に。M3の初回は吊り下げ+`--dry-run`から。
> Sport mode と低レベル制御は同時に使えない（M3スクリプトが切替を仕切る）。

---

## 0. セットアップ（ロボット側PC = Ethernet直結のノート/5090機）

1. **Go2 X**: スマホアプリ → Device → **Secondary Development → Enable** → 再起動。
2. **PCのネットワーク**: 有線LANを Go2 背面ポートへ。PC側を静的IP `192.168.123.51/24` に。
   `ping 192.168.123.161` が通ること。
3. **Python**: `pip install -r requirements.txt` ＋
   `pip install git+https://github.com/unitreerobotics/unitree_sdk2_python.git`
4. 環境変数: `GO2_IFACE`（PCのNIC名）、VLM用に `ANTHROPIC_API_KEY`（M1）。
5. **ロボット無しの疎通確認**（このPCだけで全部試せる／検証済み）:
   ```bash
   python -m m3_rl.joint_map                  # 関節マッピング
   python -m m3_rl.test_obs_builder           # obs235×policy.pt疎通
   python -m m1_agent.agent_loop --mock --instruction "go to stairs"   # エージェント
   python -m m2_navila.elevation_node --mock  # 合成階段の標高マップ配信
   python -m m3_rl.rl_stair_controller --mock --dry-run --hs elev --yes # RLループ
   ```

## M0 — テレオペと純正実力測定（数日）
```bash
python -m m0_teleop.check_robot --video      # 接続確認(LowState/カメラ)
python -m m0_teleop.sport_teleop             # WASD+QE テレオペ / x=Damp
python -m m0_teleop.video_stream --save m0.mp4
```
- **純正登坂の実力測定**: アプリで階段モードON → 10/15cm段でテレオペ登坂。
  公式スペックは**16cm**まで。**20cmは純正では登れない前提**（それがM3の存在理由）。
- ここで確認して `common/config.py` に反映するもの: NIC名 / LiDARトピック名 /
  sport解除後も `sportmodestate` が出るか（→M3の `--linvel` 選択）。

## M1 — 音声+VLMエージェント（純正歩容, ≤16cm段）
```bash
python -m m1_agent.agent_loop        # マイクにEnter→「階段まで行って登って止まって」
```
- 「止まって/ストップ」はVLMを経由しない**即時反射**で停止。
- watchdog: コマンド0.6s途絶→自動停止。登坂完了はVLM判定+IMUピッチ補助。
- バックエンド: 既定 Claude（`VLM_MODEL`で変更可）。ローカルVLMは `OPENAI_BASE_URL`。

## M2 — NaVILA + 標高マップ（幾何終了判定）
```bash
# 5090側(別環境, INSTALL_NAVILA_JP.md):
python m2_navila/navila_server.py --ckpt a8cheng/navila-llama3-8b-8f
# ロボットPC側:
python -m m2_navila.elevation_node --pose sms          # L1→標高マップ→height_scan配信
python -m m2_navila.navila_client --instruction "walk to the stairs and climb up" --use-elev
```
- elevation_node は **M3のRL方策の「目」も兼ねる**（同じUDPをrl_stair_controllerが読む）。
- 終了判定: NaVILAのstop ＋ 幾何判定（0.3m以上登った後、平坦化+高度上昇停止）。

## M3 — 学習方策で20cm階段（本丸）
**順序: sim2sim → 平地 → 低い段 → 20cm。** 各段階で人間立会い。
```bash
# (a) このPCでの疎通(ロボット不要, 検証済み):
python -m m3_rl.rl_stair_controller --mock --dry-run --hs elev --yes
# (b) 実機・平地・知覚なし(平地仮定)。初回は吊り下げて:
python -m m3_rl.rl_stair_controller --hs flat
#     → 立位ランプ3s→方策開始。速度は m1_agent --rl-backend か手動UDP:
python -m m1_agent.agent_loop --rl-backend       # 音声→RL方策 (言語→モータ制御の完成形)
# (c) 階段: elevation_node を起動してから
python -m m2_navila.elevation_node --pose sms &
python -m m3_rl.rl_stair_controller --hs elev
```
- 切替振付: 階段手前で停止(sport) → StandDown → ReleaseMode → 立位ランプ → RL 50Hz → Damp。
- 安全装置: lowstate途絶→Damp / |roll,pitch|>0.8rad→Damp / 関節リミット+変化率クランプ /
  コマンド途絶→速度0。すべて `deploy_log.jsonl` に記録される。
- **sim2sim**(推奨): unitree_mujoco (https://github.com/unitreerobotics/unitree_mujoco) の
  Go2シーンに対し、本フォルダの obs_builder/joint_map を使って policy.pt を回す。
  関節順・height_scanの意味は `policy/policy_spec.json` が唯一の正。

## フォルダ構成
```
policy/     policy.pt(.onnx) = 学習済み方策 / policy_spec.json = obs/action契約(simダンプ)
            env.yaml, agent.yaml = 学習時の完全な設定（監査用）
common/     config.py(トピック/ポート/安全定数) go2_iface.py(SDKラッパ+Mock) safety.py
m0_teleop/  check_robot.py sport_teleop.py video_stream.py
m1_agent/   agent_loop.py voice_input.py(Whisper) vlm_client.py(Claude/OpenAI互換)
m2_navila/  navila_server.py navila_client.py elevation_node.py INSTALL_NAVILA_JP.md
m3_rl/      rl_stair_controller.py obs_builder.py joint_map.py test_obs_builder.py
cockpit/    server.py static/ = ブラウザ統合コックピット(カメラ/LiDAR/ハイトマップ/操縦)
            voice.py(音声操縦) mission.py(自然言語VLA) stair.py+stair_task.py(段差登坂)
            → 使い方: cockpit/README_COCKPIT_JP.md。M0テレオペのUI版(Sport高レベル専用)
            height_scan(187点)をelevation_nodeと同契約でUDP配信 → M3のRL方策の目にもなる
```

## 既知の未確定点（実機到着後にM0で潰す）
1. LiDAR点群のトピック名/座標系（`config.TOPIC_LIDAR_CLOUD` / `--cloud-frame`）。
2. sport解除後の `sportmodestate`（速度推定源）。出ない場合 `--linvel zero` で開始し、
   LiDARオドメトリ(`--pose odom`)に切替。base_lin_velはこの方策の観測に含まれるため、
   ここの質が実機性能を左右する（最重要の残課題）。
3. 純正の階段モードON/OFFのSDK API有無（M1はアプリ操作で代替可）。
4. 実機20cmの成功率は sim(94.5%)より下がる。0.10→0.15→0.20の段階投入を厳守。

## モデルの出自（正直な注記）
- **`policy.pt` = Wave5**（Wave4→push外乱DR fine-tune, 2026-07-08採用, 1seed）。
  sim評価: 3段成功 @0.15 **95.3%** / @0.20 **93.0%** / @0.23 56.3%。
  push外乱(±0.5m/s, 10-15s毎)下で訓練し転倒2.6% — **実機向けの既定はこちら**。
- `policy_wave4_maxclimb.*` = Wave4（push未経験・sim最高記録: @0.20 94.5% / @0.23 89.8%）。
  0.23m級に挑む場合のみこちらを `--policy` で指定（外乱への裕度は低い）。
- 未対応のsim2realギャップ: アクチュエータ遅延・詳細なモータ同定・実カメラ/実LiDARノイズ。
- 実機20cmの成功率はsimより下がる前提で段階投入（0.10→0.15→0.20）を厳守。
