# AGENTS.md — coverage

BLE清掃ポリゴン → Nav2完全カバレージのブリッジ。CI: `pytest` ワークフロー。

## ブランチ運用

- `main` / `develop` 直push禁止。`feature/*` → `develop` → `main` のPRのみ
- 1コミット1話題

## テスト・ビルド

```bash
PYTHONPATH=. python -m pytest test/ -q
colcon build --symlink-install --packages-select coverage
```

## 注意

- 経路生成は自前実装しない。opennav_coverage（jazzy-v2＋Fields2Cover v2）に任せる
- coverage_serverのパラメータは `mirs` の `nav2_params.yaml` が単一真実
- `ros2 launch` は `install/` を見る。src編集後は必ず再ビルドする
- コードコメントは [`docs/COMMENT_GUIDE.md`](docs/COMMENT_GUIDE.md) に従う
- root権限を使う操作はしない
