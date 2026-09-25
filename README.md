# coverage (ROS 2 Package, v0.2.0)

BLEで受信した清掃範囲ポリゴン (`/cleaning_zone`, `geometry_msgs/PolygonStamped`)
を、Nav2の完全カバレージ機能 ([opennav_coverage](https://github.com/open-navigation/opennav_coverage))
に引き渡すブリッジパッケージです。

詳細な設計は [DESIGN.md](./DESIGN.md) を参照してください。

## 構成

```
coverage/
├── coverage/
│   ├── zone_field.py       # ROS非依存の検証・正規化(pytest対象)
│   └── coverage_bridge.py  # ROSノード(購読・NavigateCompleteCoverage投入・状態通知)
├── launch/production.launch.py # 本番一発起動(hardware+Nav2+coverage_server+BLE+bridge+RViz)
├── launch/coverage.launch.py   # 最小起動(coverage_server+bridge+RViz)
├── rviz/coverage.rviz          # /cleaning_zone + coverage_server可視化の確認用設定
├── test/test_zone_field.py
├── DESIGN.md
└── README.md
```

## 前提 (コンテナ内)

```bash
cd ~/ws  # srcの親ディレクトリ
vcs import src < src/coverage/dependencies.repos
rosdep install --from-path src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 実行方法

```bash
# 本番一発起動 (RViz付き)
ros2 launch coverage production.launch.py
# RVizなし / BLEなし(手動テスト)
ros2 launch coverage production.launch.py use_rviz:=false enable_ble:=false
# 手動でゾーン投入
ros2 topic pub --once /cleaning_zone geometry_msgs/msg/PolygonStamped "{...}"
ros2 topic echo /coverage/state
```

RVizでは地図・Global Costmap・LaserScan・清掃範囲 (`/cleaning_zone`, オレンジ)・
計画境界 (`/coverage_server/field_boundary`, 緑)・カバレッジ経路
(`/coverage_server/coverage_plan`, 緑)・swath (`/coverage_server/swaths`, 青)・
Nav2計画 (`/plan`, 赤)を確認できます。

## トピック

| 方向 | トピック | 型 |
|-----|---------|----|
| 購読 | `/cleaning_zone` | `geometry_msgs/PolygonStamped` |
| 公開 | `/coverage/state` | `std_msgs/String` |
| 送出 | `navigate_complete_coverage` | `opennav_coverage_msgs/NavigateCompleteCoverage` アクション |

## テスト

```bash
python3 -m pytest src/coverage/test/ -v
```
