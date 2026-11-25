#!/usr/bin/env bash
# 把本 App 需要的镜像导出成文件，供完全离线的机器使用。
#
#   bash deploy/save_images.sh            # 导出到 deploy/images/（约 5.5 GB）
#   bash deploy/save_images.sh --gzip     # 边导边压（约 2.5 GB，A53 上要十几分钟）
#
# 目标机器上按顺序执行：deploy/load_images.sh → deploy/prepare.sh --offline
#
# 为什么不在包里直接塞镜像：三个镜像解压后 5.5 GB、压缩后也有 2.5 GB，
# 而只要目标板能上网（或走镜像站），deploy/prepare.sh 两分钟就能拉齐。
# 这个脚本是给"车间没网/客户现场没网"的场景准备的。

set -uo pipefail

APP_DIR=$(cd "$(dirname "$0")/.." && pwd)
OUT="$APP_DIR/deploy/images"
GZIP=0
[ "${1:-}" = "--gzip" ] && GZIP=1

DOCKER="docker"
if ! $DOCKER info >/dev/null 2>&1; then
  if sudo -n docker info >/dev/null 2>&1; then DOCKER="sudo docker"; else
    echo "✗ 没有 docker 权限" >&2; exit 3
  fi
fi

mkdir -p "$OUT"

save_one() {
  local ref=$1 name=$2
  if ! $DOCKER image inspect "$ref" >/dev/null 2>&1; then
    echo "✗ 没有 $ref（先跑 deploy/prepare.sh）" >&2
    return 1
  fi
  if [ "$GZIP" = "1" ]; then
    local dst="$OUT/$name.tar.gz"
    echo "→ $ref → $dst"
    $DOCKER save "$ref" | gzip -1 > "$dst" || { rm -f "$dst"; return 1; }
  else
    local dst="$OUT/$name.tar"
    echo "→ $ref → $dst"
    $DOCKER save -o "$dst" "$ref" || { rm -f "$dst"; return 1; }
  fi
  return 0
}

echo "导出目录：$OUT"
echo "剩余空间（需要 5~6 GB）："
df -h "$OUT" | tail -1
echo

fail=0
save_one ghcr.io/arduino/app-bricks/python-apps-base:0.12.0 python-apps-base-0.12.0 || fail=1
save_one ghcr.io/arduino/app-bricks/ei-models-runner:0.12.1    ei-models-runner-0.12.1 || fail=1
save_one ros:jazzy-ros-base-noble                              ros-jazzy-ros-base-noble || fail=1
# 目标机不一定能重建 ros2_bridge（约 10 分钟 + 需要基础镜像），可选一起带走。
if [ "${SAVE_ROS2_BRIDGE:-1}" = "1" ]; then
  save_one ros2_bridge:local ros2_bridge-local || fail=1
fi

echo
ls -lh "$OUT"
[ "$fail" = "0" ] && echo "完成。目标机上跑 deploy/load_images.sh" || echo "有镜像导出失败" >&2
exit "$fail"
