# mirs_coverage 設計書

## 1. 背景と目的

`ble_server` がBLE経由で受信した清掃範囲JSONは、ROS 2トピック
`/cleaning_zone` (`geometry_msgs/PolygonStamped`, `frame_id=map`) に配信される
(4点の座標、自己交差なし・面積検証済み)。
本パッケージは、この4点で囲まれた区間をすべて走行するための
Nav2ビヘイビアツリー(BT)で使える形への変換を担う。

要件:

- 送られてきた4点の座標内の区間をすべて走行する
- 座標の点とコストマップを合わせ、走行できないところは避ける

## 2. 方針: カスタムBTプラグインは作らない

`mirs` の `nav2_params.yaml` が使うBTは素のNav2プラグインのみで構成され、
C++製BTノードの追加はビルド・運用コストが高い。
「4点内を全部走行 + 走行不可はスキップ」は、**ウェイポイント列の生成時に
解決できる**ため、BT実行時の分岐は不要と判断した。

既存の `NavigateThroughPoses` アクションと標準BT
(`navigate_through_poses_w_replanning_and_recovery.xml`) の
リプラン・クリアコストマップ・スピン等のリカバリに丸投げする。

## 3. アーキテクチャ

```
[Flutter] --BLE(JSON)--> [ble_receiver_node] --/cleaning_zone-->
  --> [coverage_executor] --/coverage/path(Path)-->
  --> [coverage_executor] --NavigateThroughPoses--> [Nav2]
       参照: /global_costmap/costmap, /map, tf(map->base_footprint)
       進捗: /coverage/state (IDLE/WAITING_COSTMAP/EXECUTING/DONE/FAILED_*)
```

- `ble_server`: 無改修。前段の検証済み出力をそのまま使う
- `mirs_coverage/coverage_planner.py`: ROS非依存の純粋関数。ホストpytestで検証可能
- `mirs_coverage/coverage_executor.py`: ROSノード。購読・Path公開・アクション投入に徹する
- BT・Nav2パラメータ・アプリ側の変更は不要

## 4. カバレッジ生成 (planner)

1. ポリゴン正規化 (`normalize_order`): 頂点を重心周りの偏角でソートし巻き順を統一。
   アプリ側の送信順序に依存しない。凹多角形にも破綻しない
2. 走査線カバレッジ (`generate_boustrophedon`): Y方向バウンディングボックスに
   `spacing` 間隔(既定 `0.35m` = footprint 0.5m - オーバーラップ)で水平走査線を敷き、
   多角形との交差区間の端点ペアを往復(boustrophedon)で連結。yawは進行方向(±X)
3. 補間 (`interpolate_waypoints`): 端点間を `resolution` (既定 `0.1m`) で直線補間し、
   コスト判定用の密な候補点列にする

最小面積外接矩形の厳密解は使わない。4点の清掃範囲では水平スキャン近似で十分であり、
凹四角形にも対応できるためである。

## 5. コストマップ連携 (走行不可は避ける)

`/global_costmap/costmap` (`nav_msgs/OccupancyGrid`, resolution 0.05)を購読し、
候補点を1点ずつ判定する (`filter_by_cost`):

- 除外条件: `cost >= lethal_cost` (既定80)、`cost == -1 (unknown)` は除外
  (`allow_unknown=false` 相当。plannerの `allow_unknown:true` より安全側に倒す)
- 範囲外セルもunknown扱いで除外
- 残った連続区間のみウェイポイント化。`min_segment_length` (既定0.5m) 未満の
  短区間・孤立点は捨てる。上限 `max_waypoints` (既定100) で打ち切る
- 各ウェイポイントはNavfn `tolerance: 0.5` の範囲で到達可能なため、
  そのまま `NavigateThroughPoses` に投げられる

動的障害物はNav2側 (local_costmap + DWB + リカバリBT) が吸収する。
全点除外の場合はロボットを動かさず `FAILED_UNREACHABLE` を通知する。

## 6. BTでの使い方

当面はBT改修なし。executorが `NavigateThroughPoses` アクションクライアントに
なるだけである。

将来BTに組み込みたい場合のみ、既存XMLをコピーした `coverage_bt.xml` を作り、
`bt_navigator` の `default_nav_through_poses_bt_xml` に差す
(カスタムノード不要。既存 `ComputePathThroughPoses` + `FollowPath` で表現可能)。

起動は `nav.launch.py` への1ノード追加か、別launch:

```bash
ros2 launch mirs_coverage coverage.launch.py
```

## 7. I/F・可観測性

| 方向 | 名前 | 型・備考 |
|-----|------|---------|
| 購読 | `/cleaning_zone` | `PolygonStamped`, QoS 10 |
| 購読 | `/global_costmap/costmap` | `OccupancyGrid` |
| 公開 | `/coverage/path` | `nav_msgs/Path`, RViz確認用 |
| 公開 | `/coverage/state` | `std_msgs/String` (`EXECUTING n=..` / `DONE` / `FAILED_*`) |
| 送出 | `navigate_through_poses` | `nav2_msgs/NavigateThroughPoses` |

新規 `/cleaning_zone` 受信で実行中ゴールはキャンセル→再計画(プリエンプト)する想定。
RVizでは `/cleaning_zone` をPolygon表示し、`/coverage/path` を重ねるとデバッグしやすい。

## 8. パラメータ

| パラメータ | 既定値 | 意味 |
|-----------|-------|------|
| `zone_topic` | `/cleaning_zone` | 入力ポリゴン |
| `costmap_topic` | `/global_costmap/costmap` | 参照コストマップ |
| `spacing` | `0.35` | 走査線間隔 [m] |
| `resolution` | `0.1` | 補間分解能 [m] |
| `lethal_cost` | `80` | 走行不可コスト閾値 |
| `frame_id` | `map` | JSONにframe_idが無い場合の代替 |

## 9. エッジケース

- 全点除外 → `FAILED_UNREACHABLE` をログ+通知し、ロボットは動かさない
- ポリゴンがマップ外/unknownだらけ → 同上(壁に突っ込まない)
- 凹四角形 → スキャンライン法は凹対応のため問題なし
- Nav2サーバ不在 → `FAILED_NO_SERVER`、ゴール拒否 → `FAILED_REJECTED`

## 10. 実装順 (本パッケージの現状)

1. [x] `coverage_planner.py` 純粋関数 + 単体テスト
2. [x] `coverage_executor.py` (購読・Path公開・アクション投入) + `coverage.launch.py`
3. [ ] RVizでの目視確認 (`/cleaning_zone` + `/coverage/path`)
4. [ ] 実機走行 (`ros2 topic echo /cleaning_zone` → `/coverage/path` → 実走)
