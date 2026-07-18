# 10. 自律探索とマップ構築(EXPLORE_AND_MAP)設計

作成: 2026-07-15。目標再定義(①音声操作 ②自然言語操作 ③自律探索+マップ構築)
を受けた新規サブシステム設計。docs/02 のアーキテクチャ invariant と
docs/08 の安全原則に完全に従属する(本書は安全 gate を一切緩めない)。

## 1. 位置づけと非対象

- 対象: 平地・屋内の既知/未知空間を Sport モード(純正歩容)で探索し、
  2D occupancy map(+ 2.5D elevation)を構築・保存・再利用する。
- **非対象**: 階段・段差の昇降(別 Gate)、屋外、無監視運用、人の近くの自律走行。
- LIVE の前提: Gate 3(平地・低速・停止試験)合格まで実機探索は行わない。
  それまでは synthetic / replay / mock server で開発・検証する。

## 2. 既存資産の再利用(docs/01 監査より)

| 資産 | 再利用 |
|---|---|
| `rt/utlidar/cloud_deskewed`(L1 deskewed 点群) | mapping の入力 |
| `rt/utlidar/robot_odom`(LiDAR odometry) | pose 源(drift は loop 対応まで許容) |
| `m2_navila/elevation_node.py` の rolling elevation map | local 層として流用(unknown≠flat の二層化は Phase 2/4 契約に従う) |
| `cockpit/server.py` の点群 WS 配信・UI | 可視化(既存 UI を壊さず追加) |

## 3. アーキテクチャ(invariant 2 の一方向経路に従う)

```text
voice/text → GoalSpec(EXPLORE_AND_MAP, target=area)
  → Mission FSM(EXPLORING state を追加)
  → Frontier Explorer(planner)
      入力: GlobalOccupancyMap(fresh/unknown/occupied/free + cell age)
      出力: 次の観測 goal pose(CommandEnvelope 経由の COMMON_NAV 要求)
  → Command Arbiter(priority=NAV_LOCAL_PLANNER)
  → Exclusive Actuation Gateway → Sport backend(Move/StopMove)
```

- **explorer は actuator owner にならない**。goal pose 提案のみ(invariant 1)。
- STOP_NOW / supervisor latch は探索中も常に最優先(arbiter 実装済み)。
- 通信断・operator lease 失効時: 新規 goal 拒否 → CONTROLLED_STOP → ACTIVE_HOLD
  (自動再開しない — docs/CLAUDE.md §9.2)。

## 4. GlobalOccupancyMap(新規契約、Phase 2 の記録契約と整合)

```yaml
schema_version: "1.0"
map_id: token
frame_id: odom            # loop closure 導入までは odom 座標系と明記
resolution_m: 0.05
origin: [x, y]
size: [W, H]
cells: OCCUPIED | FREE | UNKNOWN   # unknown != free(invariant 9)
cell_age_s: per-cell 鮮度
robot_trace: [poses]
waypoints: {home: pose, ...}       # NAVIGATE_TO_WAYPOINT の解決先
provenance: {run_id, odom_source, git_commit}
```

- 保存形式は `artifacts/maps/<map_id>/`(09 §4 の layout に節を追加)。
- 未観測 cell を FREE と同義にしない。planner の通行判定は FREE のみ。
- odom drift の扱い: MVP は「単一連続 run 内の地図」を成果物とし、
  run をまたぐ再利用は drift 評価(Gate E1)後に限定する。

## 5. Frontier 探索(決定的 baseline)

1. map から frontier(FREE と UNKNOWN の境界 cell 群)を抽出・クラスタ化。
2. 到達コスト(距離+回転)と情報利得(cluster サイズ)で決定的にスコア。
3. 最良 frontier への goal pose を発行(接近 0.5 m 手前、進入禁止 margin 付き)。
4. frontier 枯渇 or coverage 目標到達 → `EXPLORATION_COMPLETE` → ACTIVE_HOLD。
5. すべて純関数+注入時刻で offline test 可能にする(arbiter と同じ流儀)。

安全規則:
- 通行判定は FREE cell のみ・inflate 済み costmap・drop/段差 cell は OCCUPIED 扱い
  (elevation の step/drop 分類を costmap へ投影)。
- 1 goal あたりの移動は有限(例: ≤3 m)。goal ごとに arbiter を通る。
- explorer の全 goal 発行は CommandEnvelope(expiry 付き)— ゼロ推測なし。

## 6. 段階検証(実機なしで進む範囲)

| 段階 | 内容 | 実機 |
|---|---|---|
| E0 | synthetic 点群(既存 self-test の合成流儀)で map 構築の unit test | 不要 |
| E1 | 録画 replay(実機で取った rosbag/MCAP)での map 品質・drift 評価 | データのみ |
| E2 | mock server(`--mock --host 127.0.0.1`)での end-to-end(voice→explore→map 可視化) | 不要 |
| E3 | 平地 LIVE 探索(Gate 3 合格後、監視者+停止経路検証済み) | 必要 |

## 7. 実装順(offline)

1. `perception/global_map.py`: 点群+odom → occupancy 更新(純ロジック)+ synthetic test
2. `navigation/frontier_explorer.py`: frontier 抽出/スコア/goal 発行 + test
3. Mission FSM に EXPLORING/EXPLORATION_COMPLETE を追加(Gateway task と同時)
4. map の保存/読込(`artifacts/maps/`)+ waypoint(home)登録
5. cockpit UI へ map 可視化 layer(既存 UI 差分を保全して追加)

## 8. 2026-07-18 実装更新

初期baselineに存在した直線goal、探索履歴なし、VLM open-loop実機経路は置換した。
現在はreachable frontier、inflate済みA* path、visit/failure履歴、progress timeout、
cell freshness、独立local collision guardianを使用する。Cockpitの探索入力も
`GlobalOccupancyMap → ExplorationController → guardian → RobotBridge`へ接続済み。

原因、実装契約、全自動testとcollision付きE2Eの最新結果は
[13_EXPLORATION_SAFETY_IMPLEMENTATION.md](13_EXPLORATION_SAFETY_IMPLEMENTATION.md)
を正本とする。なお、設計上のCommand Arbiter/Exclusive Actuation Gatewayを
Cockpit I/O adapterへ統合する作業は残るため、guardian/TTL/watchdogを外した状態を
LIVE許可しない。
