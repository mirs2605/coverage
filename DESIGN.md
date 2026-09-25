# coverage 設計書 (opennav_coverage版 v0.2.0)

## 1. 背景と目的

`ble_server` がBLE経由で受信した清掃範囲JSONは、ROS 2トピック
`/cleaning_zone` (`geometry_msgs/PolygonStamped`, `frame_id=map`) に配信される
(自己交差なし・面積検証済みの多角形)。
本パッケージは、この多角形をNav2の完全カバレージ機能
([opennav_coverage](https://github.com/open-navigation/opennav_coverage),
Fields2Coverベース) に引き渡す。

旧来の自作boustrophedon生成 (`coverage_planner.py`/`coverage_executor.py`) は
v0.2.0で削除した。理由:

- swath・旋回・headland・迂回路の生成は既存ライブラリが遥かに堅牢
- コストマップ連携・リカバリを自前で持つ必要がなくなる
- Nav2のlifecycle/BTの枠組みに乗れる

## 2. アーキテクチャ

```
[Flutter] --BLE(JSON)--> [ble_receiver_node] --/cleaning_zone-->
  --> [coverage_bridge] --NavigateCompleteCoverage--> [bt_navigator/CoverageNavigator]
        --ComputeCoveragePath--> [coverage_server] --nav_path--> FollowPath --> DWB
```

- `ble_server`: 無改修。前段の検証済み出力をそのまま使う
- `coverage/zone_field.py`: ROS非依存の純粋関数(検証・正規化)。pytest対象
- `coverage/coverage_bridge.py`: ROSノード。購読・ゴール投入・状態通知に徹する
- `opennav_coverage/coverage_server`: Nav2 Task Server。swath/route/path生成
- `opennav_coverage_navigator/CoverageNavigator`: bt_navigatorプラグイン。
  `navigate_complete_coverage` アクションを公開する
- BT XMLは純正デフォルト
  (`opennav_coverage_bt/behavior_trees/navigate_w_basic_complete_coverage.xml`)
  を使う。本パッケージ独自のBTは持たない

## 3. I/F・可観測性

| 方向 | 名前 | 型・備考 |
|-----|------|---------|
| 購読 | `/cleaning_zone` | `PolygonStamped`, QoS 10 |
| 公開 | `/coverage/state` | `std_msgs/String` (`EXECUTING` / `DONE` / `CANCELED` / `FAILED_*` / `WAITING_*`) |
| 送出 | `navigate_complete_coverage` | `opennav_coverage_msgs/NavigateCompleteCoverage` |
| 可視化 | `/coverage_server/coverage_plan` | `nav_msgs/Path` (RViz緑) |
| 可視化 | `/coverage_server/field_boundary` | `PolygonStamped` (RViz緑) |
| 可視化 | `/coverage_server/swaths` | `Marker/LINE_LIST` (RViz青) |

新規 `/cleaning_zone` 受信で実行中ゴールはキャンセル→再投入(プリエンプト)する。
Nav2に拒否され続けた場合は `retry_limit` (既定3回) で諦め、
`FAILED_REJECTED` を通知する。

## 4. パラメータ

bridge (`coverage_bridge`):

| パラメータ | 既定値 | 意味 |
|-----------|-------|------|
| `zone_topic` | `/cleaning_zone` | 入力ポリゴン |
| `state_topic` | `/coverage/state` | 状態通知 |
| `action_name` | `navigate_complete_coverage` | CoverageNavigator |
| `frame_id` | `map` | ゾーンにframe_idが無い場合の代替 |
| `behavior_tree` | `""` | BT上書き(空=純正デフォルト) |
| `nav_timeout_sec` | `5.0` | アクションサーバ待ち |
| `retry_limit` | `3` | 拒否の上限 |
| `backoff_sec` | `5.0` | 再送間隔 |

coverage_server (`mirs/.../nav2_params.yaml` の `coverage_server` 節が単一真実):

| パラメータ | 既定値 | 意味 |
|-----------|-------|------|
| `coordinates_in_cartesian_frame` | `true` | 屋内なので直交座標 |
| `robot_width` | `0.5` | 車体幅(footprint 0.5m) |
| `operation_width` | `0.35` | 清掃幅(旧spacing相当) |
| `min_turning_radius` | `0.2` | 最小旋回半径 |
| `default_headland_width` | `0.0` | 壁際まで清掃(除外幅なし) |
| `default_route_type` | `BOUSTROPHEDON` | 往復 |
| `default_swath_angle_type` | `BRUTE_FORCE` | 任意四角形に対応 |

## 5. エッジケース

- 頂点数不足・非有限値 → ロボットを動かさず `FAILED_INVALID_ZONE`
- Nav2サーバ不在 → `WAITING_NAV_SERVER` で待機(1秒周期再試行)
- ゴール拒否 → `WAITING_NAV_RETRY n/3`、上限で `FAILED_REJECTED`
- 非凸多角形: 上流Fields2Coverの制約。凸四角形(BLE仕様)を想定

## 6. 依存の導入 (コンテナ内。Fields2Cover v2構成)

```bash
cd ~/ws  # srcの親ディレクトリ
# opennav_coverage は jazzy-v2 (F2C v2対応) を使う
git clone -b jazzy-v2 https://github.com/open-navigation/opennav_coverage.git src/opennav_coverage
# Fields2Cover v2 はソースビルドする。apt版(v1系)が入っていると混同するので除去
sudo apt remove -y ros-jazzy-fields2cover
git clone -b v2.0.0 https://github.com/Fields2Cover/Fields2Cover.git src/Fields2Cover
rosdep install --from-path src --ignore-src -r -y
colcon build --symlink-install
```
