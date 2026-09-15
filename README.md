# mirs_coverage (ROS 2 Package)

BLEで受信した清掃範囲ポリゴン (`/cleaning_zone`, `geometry_msgs/PolygonStamped`)
を、Nav2で実行できるカバレッジ走行ウェイポイント列に変換するパッケージです。

詳細な設計は [DESIGN.md](./DESIGN.md) を参照してください。

## 構成

```
mirs_coverage/
├── mirs_coverage/
│   ├── coverage_planner.py   # ROS非依存の純粋関数(生成・補間・コストフィルタ)
│   └── coverage_executor.py  # ROSノード(購読・Path公開・Nav2投入)
├── launch/coverage.launch.py
├── test/test_coverage_planner.py
├── DESIGN.md
└── README.md
```

## ビルド & 実行方法 (Jazzy コンテナ内)

```bash
colcon build --symlink-install --packages-select mirs_coverage
source install/setup.bash

# ble_server + Nav2 起動後に実行
ros2 launch mirs_coverage coverage.launch.py
```

## トピック

| 方向 | トピック | 型 |
|-----|---------|----|
| 購読 | `/cleaning_zone` | `geometry_msgs/PolygonStamped` |
| 購読 | `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` |
| 公開 | `/coverage/path` | `nav_msgs/Path` (RViz確認用) |
| 公開 | `/coverage/state` | `std_msgs/String` |
| 送出 | `navigate_through_poses` | `nav2_msgs/NavigateThroughPoses` アクション |

## テスト

```bash
python3 -m pytest src/mirs_coverage/test/ -v
```
