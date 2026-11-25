#!/usr/bin/env bash
# 容器入口：先叠好 ROS 2 与本地工作区的环境，再执行 CMD。
#
# 为什么需要它：`ros2 launch arduino_ros2_bringup ...` 要能找到本包，
# 必须在进程环境里 source install/setup.bash。写进 ENTRYPOINT 比在 CMD 里
# 拼一长串 `bash -lc "source ... && ..."` 更好排错：CMD 保持干净的可执行
# 形式，docker exec 进来也不会污染环境。
set -e

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash

if [ -f "${ROS_WS:-/opt/ros_ws}/install/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "${ROS_WS:-/opt/ros_ws}/install/setup.bash"
else
    # 镜像构建异常时不至于连 rosbridge 都起不来，至少把原因说清楚。
    echo "[entrypoint] 警告: 未找到 ${ROS_WS:-/opt/ros_ws}/install/setup.bash" >&2
fi

exec "$@"
