#!/usr/bin/env bash
# 把 deploy/images/ 里的镜像导入本机 docker（离线部署用）。
#
#   bash deploy/load_images.sh
#
# 导入后的名字与 compose / Dockerfile 里写死的完全一致，所以不需要重新 tag：
#   ghcr.io/arduino/app-bricks/python-apps-base:0.12.0
#   ghcr.io/arduino/app-bricks/ei-models-runner:0.12.1
#   ros:jazzy-ros-base-noble
#   ros2_bridge:local

set -uo pipefail

APP_DIR=$(cd "$(dirname "$0")/.." && pwd)
SRC="$APP_DIR/deploy/images"

DOCKER="docker"
if ! $DOCKER info >/dev/null 2>&1; then
  if sudo -n docker info >/dev/null 2>&1; then DOCKER="sudo docker"; else
    echo "✗ 没有 docker 权限" >&2; exit 3
  fi
fi

shopt -s nullglob
files=("$SRC"/*.tar "$SRC"/*.tar.gz "$SRC"/*.tgz)
if [ ${#files[@]} -eq 0 ]; then
  echo "✗ $SRC 里没有镜像文件（先在有网的机器上跑 deploy/save_images.sh）" >&2
  exit 2
fi

fail=0
for f in "${files[@]}"; do
  echo "→ 导入 $(basename "$f")（$(du -h "$f" | cut -f1)）"
  $DOCKER load -i "$f" || { echo "  ✗ 失败" >&2; fail=1; }
done

echo
echo "导入结果："
$DOCKER images --format '  {{.Repository}}:{{.Tag}}  {{.Size}}' | \
  grep -E 'python-apps-base|ei-models-runner|ros2_bridge|ros .*jazzy' || true

echo
if [ "$fail" = "0" ]; then
  echo "完成。接着跑：bash deploy/prepare.sh --offline --skip-build"
else
  echo "有文件导入失败" >&2
fi
exit "$fail"
