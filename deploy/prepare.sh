#!/usr/bin/env bash
# 准备这台板子上运行本 App 所需的全部镜像。
#
#   bash deploy/prepare.sh              # 拉镜像 + 构建 ros2_bridge + 自检
#   bash deploy/prepare.sh --skip-build # 只准备镜像，不构建（约 2 GB 下载）
#   bash deploy/prepare.sh --offline    # 不联网，只用 deploy/images/ 里导入过的镜像
#
# 为什么要单独一个脚本：这个 App 用到三个外部镜像，其中两个在国内容易拉不下来
# （ghcr.io 直连超时、Docker Hub 慢/被墙）。脚本按"目标名 → 镜像站"的顺序
# 逐个尝试，成功后统一 tag 成 compose 里写死的 ghcr.io / 官方名字，
# 这样 app.yaml / compose / Dockerfile 都不用改，换台机器也能直接起。
#
# 不做的事：不删除任何已有镜像，不动 App 目录，不启动应用。

set -uo pipefail

APP_DIR=$(cd "$(dirname "$0")/.." && pwd)
SKIP_BUILD=0
OFFLINE=0
for arg in "$@"; do
  case "$arg" in
    --skip-build) SKIP_BUILD=1 ;;
    --offline) OFFLINE=1 ;;
    -h|--help) sed -n '2,14p' "$0"; exit 0 ;;
    *) echo "未知参数：$arg" >&2; exit 2 ;;
  esac
done

DOCKER="docker"
if ! $DOCKER info >/dev/null 2>&1; then
  if sudo -n docker info >/dev/null 2>&1; then
    DOCKER="sudo docker"
  else
    echo "✗ 当前用户没有 docker 权限（试过 sudo -n 也不行）" >&2
    exit 3
  fi
fi

# 国内可用的镜像站。顺序就是尝试顺序；要用别的源就改这两个环境变量。
GHCR_MIRROR=${GHCR_MIRROR:-ghcr.nju.edu.cn}
HUB_MIRRORS=${HUB_MIRRORS:-"docker.m.daocloud.io/library docker.1ms.run/library"}

ok=0
fail=0

have() { $DOCKER image inspect "$1" >/dev/null 2>&1; }

note() { printf '%s\n' "$*"; }

# 磁盘先看：三个镜像约 5.6 GB，构建缓存还要 1~2 GB。UNO Q 的根分区只有
# 9.8 GB，默认 Docker Root Dir 就在根上 —— 空间不够时 build 会在最后一步
# 报 "no space left on device"，看起来像网络/编译问题，其实是盘满了。
DOCKER_ROOT=$($DOCKER info --format '{{.DockerRootDir}}' 2>/dev/null)
if [ -n "$DOCKER_ROOT" ]; then
  avail_kb=$(df -Pk "$DOCKER_ROOT" | awk 'NR==2 {print $4}')
  avail_gb=$(( ${avail_kb:-0} / 1024 / 1024 ))
  note "镜像存储目录：$DOCKER_ROOT（剩余 ${avail_gb} GB）"
  if [ "$avail_gb" -lt 6 ]; then
    note "⚠ 剩余 ${avail_gb} GB。全新板子要把 3 个镜像（约 5.6 GB）全下来，"
    note "  再加构建缓存 1~2 GB，空间不够时 build 会在最后一步报"
    note "  \"no space left on device\"，看起来像编译/网络问题。两个办法："
    note "   1) 把 docker 数据目录挪到大分区（本机实测 /home 更大）："
    note "      sudo systemctl stop docker docker.socket"
    note "      sudo rsync -aHAX /var/lib/docker/ /home/arduino/docker-data/"
    note "      echo '{\"data-root\":\"/home/arduino/docker-data\"}' | sudo tee /etc/docker/daemon.json"
    note "      sudo systemctl start docker"
    note "   2) 构建完立刻清缓存：sudo docker builder prune -f"
    note "  已经拉过的镜像不会重复下载，空间紧张时可以先清 dangling："
    note "      sudo docker image prune -f"
  fi
  note
fi

# 逐个镜像站尝试，成功后 tag 成 compose 期望的名字。
# ensure_image <目标名> <候选1> <候选2> ...
ensure_image() {
  local target=$1; shift
  if have "$target"; then
    note "✓ 已有 $target"
    ok=$((ok + 1))
    return 0
  fi
  if [ "$OFFLINE" = "1" ]; then
    note "✗ 离线模式且没有 $target（先跑 deploy/load_images.sh）"
    fail=$((fail + 1))
    return 1
  fi
  local src
  for src in "$@"; do
    note "→ 拉取 $src"
    if timeout 1200 $DOCKER pull "$src"; then
      [ "$src" = "$target" ] || $DOCKER tag "$src" "$target"
      note "✓ $target（来自 $src）"
      ok=$((ok + 1))
      return 0
    fi
    note "  ✗ $src 失败，换下一个源"
  done
  note "✗ 所有源都失败：$target"
  fail=$((fail + 1))
  return 1
}

note "== 1/3 官方基础镜像（App 运行时） =="
ensure_image ghcr.io/arduino/app-bricks/python-apps-base:0.12.0 \
  "$GHCR_MIRROR/arduino/app-bricks/python-apps-base:0.12.0" \
  ghcr.io/arduino/app-bricks/python-apps-base:0.12.0

note
note "== 2/3 物品检测推理镜像（模型已内置，约 970 MB） =="
ensure_image ghcr.io/arduino/app-bricks/ei-models-runner:0.12.1 \
  "$GHCR_MIRROR/arduino/app-bricks/ei-models-runner:0.12.1" \
  ghcr.io/arduino/app-bricks/ei-models-runner:0.12.1

note
note "== 3/3 ROS 2 构建基础镜像（只用于本地 build ros2_bridge） =="
hub_candidates=()
for m in $HUB_MIRRORS; do hub_candidates+=("$m/ros:jazzy-ros-base-noble"); done
hub_candidates+=("ros:jazzy-ros-base-noble")
ensure_image ros:jazzy-ros-base-noble "${hub_candidates[@]}"

if [ "$SKIP_BUILD" = "0" ] && [ "$fail" = "0" ]; then
  note
  note "== 构建 ros2_bridge:local（首次约 10 分钟，改过代码后重建） =="
  if ( cd "$APP_DIR/bricks/ros2_bridge" && $DOCKER build -t ros2_bridge:local ./docker ); then
    note "✓ ros2_bridge:local"
    ok=$((ok + 1))
    note "  提示：构建中间层可用 sudo docker builder prune -f 清掉（约 1~2 GB，"
    note "        代价是下次改代码重建慢几分钟）。"
  else
    note "✗ ros2_bridge 构建失败"
    fail=$((fail + 1))
  fi
fi

note
note "== 自检 =="
for ref in \
  ghcr.io/arduino/app-bricks/python-apps-base:0.12.0 \
  ghcr.io/arduino/app-bricks/ei-models-runner:0.12.1 \
  ros:jazzy-ros-base-noble \
  ros2_bridge:local ; do
  if have "$ref"; then
    size=$($DOCKER image inspect "$ref" --format '{{.Size}}' 2>/dev/null)
    printf '  ✓ %-58s %s MB\n' "$ref" "$(( ${size:-0} / 1024 / 1024 ))"
  else
    printf '  ✗ %-58s 缺失\n' "$ref"
  fi
done

# 固件平台：App Lab 会按 sketch.yaml 把 arduino:zephyr 1.1.0 装进
# ~/.arduino15/internal/，这里只看一眼，不动用户的库（升级库会改变现有固件行为）。
if command -v arduino-cli >/dev/null 2>&1; then
  note
  note "== 固件平台（由 App Lab 自动安装，这里仅检查） =="
  arduino-cli core list 2>/dev/null | sed -n '1,4p'
  note "  提示：sketch.yaml 要求 arduino:zephyr 1.1.0 + RouterBridge 0.4.4 + RPClite 0.3.1；"
  note "        1.0.0 能编译但不会注册 RPC，遥测会全空。缺版本时让 app start 自己装。"
fi

note
if [ "$fail" -gt 0 ]; then
  note "结果：$ok 项就绪，$fail 项失败 —— 先把失败项解决再 app start"
  exit 1
fi
note "结果：全部就绪。下一步："
note "  arduino-app-cli app start $APP_DIR"
