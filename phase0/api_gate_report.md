# Phase 0 成果物: API gate report / vendor 質問票(04_ROADMAP.md L63)

状態: **SKIPPED(ユーザー決定 2026-07-15)+ 一部 USER_DECLARED**

- ユーザー決定: vendor への質問送付は行わない。SKU は **Go2 X**(申告)。
- 帰結(記録):
  - **Branch S(純正階段歩容の formal API)は候補から除外**(Gate 1S の成立条件を
    満たす手段がないため)。階段昇降を行う場合は Branch L(LowCmd)一本になる。
  - LowCmd 可否・保証条件・firmware timeout・remote override の意味は vendor 確認
    なしのため **実測でしか確認できない**。該当項目は「吊り下げ/非接地の実測 Gate
    (2L)」へ先送りし、それまで当該能力は ASSUMED のまま実機判断に使わない。
  - physical E-stop の同定(08 §2.5)は**スキップ対象外**。これが未同定の間、
    階段 LIVE は NO-GO のまま(平地 LIVE も Gate 3 の停止試験が前提)。
- プロジェクト目標の再定義(ユーザー指示): ①音声操作 ②自然言語操作
  ③自律探索+マップ構築。階段 Mission は優先度低下(後回し)。

以下の質問票は将来 vendor 確認を行う場合のために保存する。

## 1. Unitree / 販売店への質問票(未送付)

### A. LowCmd / モード切替(Branch L の成立条件)

1. Go2 **X** SKU(serial: BLOCKED, firmware: BLOCKED)で `LowCmd` 送信と `MotionSwitcherClient.ReleaseMode()` は許可されているか。
2. 上記を行った場合の保証条件への影響。
3. `ReleaseMode()` 後(Low-level mode 中)の以下の可用性と意味:
   - `rt/utlidar/cloud_deskewed` / `rt/utlidar/robot_odom`(LiDAR odometry は維持されるか)
   - `rt/sportmodestate`(速度推定は凍結するか)
   - リモコン入力(`LowState` remote bytes)の意味と優先権
   - `foot_force` の較正状態と単位
4. LowCmd の command owner が死亡/停止した場合の firmware 側挙動(timeout の有無、bounded response、姿勢維持 or 脱力)。
5. LowCmd 送信周期の公式要件(公式例は 2ms。50Hz 送信は許容範囲か)。

### B. 純正階段歩容(Branch S の成立条件 — 08 Gate 1S)

6. 純正階段モードを**外部 SDK から開始する正式 API** は存在するか(アプリ手動設定・通常 `Move()` の代用は不可)。
7. 存在する場合: 開始・方向指定(上り/下り)・進行 state 取得・正常完了通知・停止/cancel・timeout・remote override の各 API と適用条件(段高範囲、踏面、材質)。
8. vendor 側 test evidence(適用階段寸法、方向、既知 failure mode)。
9. `ObstaclesAvoidClient` の対応状況と階段モードとの干渉。
10. vendor simulator の提供有無。

### C. 停止系(階段 LIVE の前提 — 08 §2.5)

11. physical E-stop に相当する**独立停止機能**の構成(受信機断時挙動、駆動エネルギー遮断の有無)。
12. リモコン `Damp` の経路(software 経由か、firmware 直か)と LowCmd/Sport 実行中の挙動。
13. `StopMove` の ack 仕様と no-ack 時の推奨対応。

## 2. 実測記録(ReleaseMode 前後の topic matrix — 04 L57)

| topic | ReleaseMode 前 | ReleaseMode 後 | rate | latency | jitter |
|---|---|---|---|---|---|
| rt/utlidar/cloud_deskewed | BLOCKED | BLOCKED | | | |
| rt/utlidar/robot_odom | BLOCKED | BLOCKED | | | |
| rt/lowstate | BLOCKED | BLOCKED | | | |
| rt/sportmodestate | BLOCKED | BLOCKED | | | |
| rt/lowcmd(権限) | BLOCKED | BLOCKED | | | |

実測はユーザー承認済みの run でのみ行う(CLAUDE.md §5)。

## 3. 判定(すべて回答待ち)

- Branch S 候補維持可否: **BLOCKED**(質問 6-10)
- Branch L 成立可否: **BLOCKED**(質問 1-5)
- 階段 LIVE 前提(E-stop): **BLOCKED / NO-GO 維持**(質問 11-13)
