# backlog（coverage）

実機統合で検出した未完了項目。横断依存は `ws/TODO.md` §6。

最終更新: 2026-10-01

## 未完了

### COV-1. Nav2 bringupが `smoother_server` 以降で停止する [高]
- 現象: 02:41・03:33の両launchで再現。`controller_server`・`smoother_server`は
  active化するが、`planner_server` 以降（behavior・bt_navigator・waypoint・
  smoother・collision_monitor）が inactive のまま残る。
  `lifecycle_manager_navigation` 自体は生存。`global_costmap` 未配信、
  `navigate_complete_coverage` は受理後の追従に入れない。
- 初期位置未確定時の `map -> odom` 欠落との関連が濃厚（起動直後はAMCLがTFを張れない）。
  managerが一度失敗すると再試行しない。
- 暫定手順: 初期位置送信後に未activeノードを `ros2 lifecycle set <node> activate`。
- `nav2_params.yaml` の `node_names` に `collision_monitor` を追加済み
  （`feature/lifecycle-collision-monitor`、mirs側）。

### COV-2. featureブランチのpush・PR [中]
- 未push: `coverage` の `close-coverage-polygon`・`refactor-coverage-bridge`・
  `coverage-obstacle-aware`、`mirs` の `lifecycle-collision-monitor`、
  `ble_app` の `closed-polygon-wire`。
- デプロイは `install/` 反映（`colcon build`）＋ launch再起動が必要。
  稼働中ノードは旧コードのままな点に注意。

### COV-3. `max_occupied_ratio` 既定値の実機チューニング [低]
- 既定 `0.05`。廊下地図での誤拒否・見逃しのバランスを実走で確認すること。
