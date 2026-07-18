# 13. 自律探索の衝突防止・全域探索修正（2026-07-18）

対象: Cockpit の `EXPLORE_AND_MAP` 実機経路と、offline探索baseline。

## 1. 再現した問題と原因

| 症状 | 直接原因 |
|---|---|
| 正面に障害物があっても衝突する | 旧 `MissionAgent` はLiDARをVLM説明文に使うだけで、速度commandを遮断する決定的なguardianがなかった。1判断を最大8秒再送したため、0.3 m/sで最大2.4 m進めた。 |
| 局所だけを探索する | frontierまでの直線距離だけでgoalを選び、壁・UNKNOWNを回り込む到達可能pathを計算していなかった。goal到着時以外は再計画しなかった。 |
| 同じ場所へ戻る | robot trace、cell visit回数、直近goal、失敗goalのcooldown、no-progress timeoutがなかった。さらに遠いfrontierでは、失敗履歴を実際の切り詰め後goalでなく遠方viewpointへ照合していた。 |
| 未探索なのに完了する | 5 cell未満のfrontierを削除してから完了判定し、stale FREEや同一scanのcontrol tickも完了確認に数えていた。 |
| デモだけ成功する | kinematic simulatorに機体半径と壁のcollision判定がなく、仕切り壁を通過できた。 |
| UI追加後に安全修正を迂回する | 上流の`ExploreTask`が別の5 Hz actuator runnerを持ち、UNKNOWNを通行可にして`bridge.set_cmd`へ直接送っていた。 |
| 古い点群をfresh扱いする | callback失敗時にも更新される`cloud_rx_ts`だけを見て、前回の`cloud_pts`を再利用できた。 |

追加で、`map/odom`、`fake/odom`、`odom/lidar`をodom点群として受理できる
frame判定、freshな`robot_odom`なしで点群を統合する経路、動体点を永久に
OCCUPIEDとして残すghost obstacleも確認した。

## 2. 実装した制御経路

```text
odom + odom-frame PointCloud2
  → 3D点群をfloor return / obstacle hitへ分類
  → GlobalOccupancyMap（UNKNOWN≠FREE、cell freshness、動体解除）
  → reachable frontier + inflate済みA*/Dijkstra path
  → body-frame低速controller（旋回してから前進）
  → local collision guardian（毎tick）
  → Mission keeper（同一sensor snapshotを再検査、0.3 s TTL）
  → RobotBridge watchdog → Sport Move/StopMove
```

Cockpitで探索文が入力された場合、`ExploreTask`は提案・明示確認・STOP・UI表示だけを
担当する。確認済み`GoalSpec`を`MissionExecutive(EXPLORING)`で受理した後、
`MissionAgent.start_goal()`が上記の決定的controllerを一度だけ起動する。
旧`ExploreTask` actuator loopは明示的に無効化し、地図writerと走行runnerを一系統にした。
通常のVLM missionにも最終guardianを適用し、保持時間は0.8秒へ短縮した。
探索の実速度上限は0.20 m/sである。

## 3. 安全側の契約

- 実機点群frameは正規化後の完全一致`odom`のみ。LiDAR odometry poseは0.60秒以内。
- LiDAR scan、pose、点密度、進行回廊のいずれかが不十分なら運動commandを許可しない。
- 実機LowStateは0.50秒以内を必須とし、|roll|>0.50 rad、|pitch|>0.70 rad、
  欠損/非有限姿勢は開始前・runner・keeperの全段で即時停止する。
- 停止距離は `robot radius + margin + reaction*v + v²/(2*deceleration)` で計算する。
- 直進・斜行はswept corridor、純旋回は脚を含むswept circleを点群で確認する。
- dropは常に停止。wall/障害物が停止距離内なら即ゼロcommandを送る。
- map revisionでpathが塞がれた場合と、FREEの期限切れでpathがstaleになった場合は、
  そのtickでは動かず停止してから再計画する。
- hold期限切れはbridgeの0.5秒watchdogを待たず明示的にゼロを送る。
- command/ARM入力は型と有限値を検査する。NaN/Inf/変換不能値はclampせず即ゼロにし、
  手動入力由来なら自律runも失効させる。
- guardianが評価したcommand snapshot以外は送らない。command世代とlockで
  controller/VLM/abort間のTOCTOUを防ぐ。旧mission threadは新runへ復帰できない。
- planner計算中のSTOP/手動overrideでも、run IDの失効・zero化・UI状態更新を
  lifecycle lockで直列化し、旧threadが完了状態やheld commandを上書きしない。
- `探索しないで`、質問、引用は探索開始へ昇格しない。
- AI MISSION欄の探索文も同じproposal/confirm経路へ転送し、直接開始を拒否する。
- 非ゼロ手動入力は全自律runnerを停止・command世代を失効してから適用する。
- 探索中の自動登坂は、別GoalSpec・別確認・authority handoffが整うまで既定OFF。

## 4. 全域探索・再訪抑制

- robotと同じinflate済みFREE連結成分にあるfrontierだけを候補にする。
- 8近傍の最短pathを計算し、対角corner cuttingを禁止する。
- `max_step_m`は直線距離ではなくpath累積距離へ適用する。
- frontierそのものではなく既知FREE側のstandoff viewpointをgoalにする。
- information gain/path costに加え、cell visit回数、直近goal、失敗goalを減点する。
- 2 mへ切り詰めた実際の中間goalに履歴・visit・avoidを適用し、同じ中間地点の
  再発行を防ぐ。
- 進捗が3秒間増えなければ停止し、そのgoalをcooldownして別候補を探索する。
- 小frontierは優先度だけを下げ、存在する限り`COMPLETE`にしない。
- 完了はfrontier枯渇を3つの別LiDAR観測で確認する。同じscanのtick反復では進まない。
- sensor由来のOCCUPIEDは、別々の3 scanでFREE証拠を得た場合だけ解除する。
  drop/段差hazard layerはLiDAR missでは解除しない。

## 5. 検証結果

実行コマンド:

```bash
python3 -m unittest discover -v
python3 -m unittest cockpit.test_lidar_pipeline -v
python3 -m demo.explore_e2e
```

2026-07-18に最新`origin/main`を統合後の結果:

- 全unit/integration test: 385件成功
- Cockpit LiDAR pipeline: 13件成功
- collision付き2部屋E2E: 完了、壁接触0、room Bへ実進入
- E2E対象ROI観測率: 100.0%
- unique visited cells: 95、revisit ratio: 5.7%
- STOP_NOW途中注入後の自動再開なし
- Mock WebSocket実通信: HTTP 200、ARM、proposal、confirm、安全runner起動、
  occupancy binary frame(type=3)、STOP、DISARMを確認

E2Eは120-ray、最大2.2 mの有限rangeを使い、body-frame速度とyawを積分する。
機体半径0.22 mを含むcollision判定が接触を検出した時点でtestを失敗させる。

## 6. 実行と実機Gate

ローカル確認:

```bash
python3 -m cockpit.server --mock --no-voice --host 127.0.0.1 --port 8080
# http://127.0.0.1:8080
```

実機:

```bash
GO2_IFACE=<有線NIC> python3 -m cockpit.server --no-voice --host 0.0.0.0 --port 8080
```

ソフトウェアのsynthetic試験成功は、実機の停止性能を保証しない。LIVE探索は
`08_SAFETY_TEST_EVALUATION.md` Gate 3に従い、監視者、物理停止手段、安全索、
低速平地区画を用意する。障害物投入、LiDAR停止、odom停止、drop提示について、
外部計測で停止距離内に止まることを確認するまで無監視運用は禁止する。
