#!/bin/bash
# 依存ソースの取得＋ビルド (コンテナ内で実行)。
# 未存在のものだけ取得する。バージョン固定は下記の変数でレビューする。
# 使い分け:
#   apt解決分 (Fields2Cover含むバイナリ依存) は rosdep が package.xml から解決する。
#   未リリースのソース (opennav_coverage, Fields2Cover v2) のみここで git clone する。
set -euo pipefail

WS="${WS:-$HOME/ws}"
OPENNAV_BRANCH="${OPENNAV_BRANCH:-jazzy-v2}"
F2C_TAG="${F2C_TAG:-v2.0.0}"

# apt版(v1系)とソース版(v2)の混同防止。入っていれば除去する
if dpkg -l ros-jazzy-fields2cover >/dev/null 2>&1; then
  echo "remove ros-jazzy-fields2cover (v1) to avoid mixing with source v2"
  sudo apt remove -y ros-jazzy-fields2cover
fi

if [ ! -d "$WS/src/opennav_coverage" ]; then
  git clone -b "$OPENNAV_BRANCH" \
    https://github.com/open-navigation/opennav_coverage.git \
    "$WS/src/opennav_coverage"
else
  echo "skip: $WS/src/opennav_coverage exists"
fi

if [ ! -d "$WS/src/Fields2Cover" ]; then
  git clone -b "$F2C_TAG" \
    https://github.com/Fields2Cover/Fields2Cover.git \
    "$WS/src/Fields2Cover"
else
  echo "skip: $WS/src/Fields2Cover exists"
fi

cd "$WS"
rosdep install --from-path src --ignore-src -r -y
colcon build --symlink-install
