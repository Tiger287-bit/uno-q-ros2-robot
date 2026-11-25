
# ROS 2 Jazzy 运行时：rosbridge + Foxglove + 手柄遥控 + 速度仲裁。
#
# 构建上下文是本文件所在目录（见 brick_compose.yaml 的 build.context）。
# 构建由 App Lab 编排器在 App 启动时执行，也可手工验证：
#   cd bricks/ros2_bridge && docker compose build
#   docker build -t ros2_bridge:local ./docker
#
# 代码组织按标准 ROS 2 工作区：ros_ws/src/<包>，由 colcon 构建安装，
# 运行时 ros2 launch arduino_ros2_bringup bringup.launch.py。
# 详见 docs/ARCHITECTURE.md。

FROM ros:jazzy-ros-base-noble


# ============================================================
# 1. APT 软件源优化
# ============================================================

# 使用清华镜像加速 Ubuntu ARM64 和 ROS 2 Jazzy 软件包下载。
#
# 修复：
# 清华 ROS 2 镜像不提供 noble/main/source/Sources 索引，
# 而基础镜像可能启用了 deb-src。
#
# 这里禁用 ROS 2 源码仓库，只保留二进制软件包仓库。
# 不影响 ros-jazzy-* 软件包的正常安装。

RUN set -eux; \
    for f in /etc/apt/sources.list \
             /etc/apt/sources.list.d/*.list \
             /etc/apt/sources.list.d/*.sources; do \
        if [ -f "$f" ]; then \
            sed -i \
              -e 's|http://ports.ubuntu.com/ubuntu-ports|https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports|g' \
              -e 's|https://ports.ubuntu.com/ubuntu-ports|https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports|g' \
              -e 's|http://packages.ros.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|g' \
              -e 's|https://packages.ros.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|g' \
              "$f"; \
            if grep -q 'ros2/ubuntu' "$f"; then \
                case "$f" in \
                    *.sources) \
                        sed -i \
                          '/^Types:/c\Types: deb' "$f"; \
                        ;; \
                    *.list) \
                        sed -i -E \
                          '/^[[:space:]]*deb-src[[:space:]].*ros2\/ubuntu/s/^/# /' "$f"; \
                        ;; \
                esac; \
            fi; \
        fi; \
    done


# ============================================================
# 2. APT 网络优化
# ============================================================

# 下载失败时自动重试 3 次。
# 不关闭软件包签名和 HTTPS 证书验证。

RUN printf 'Acquire::Retries "3";\n' \
    > /etc/apt/apt.conf.d/99-network-retries


# ============================================================
# 3. ROS 2 运行环境
# ============================================================

# 容器日志写入 /tmp，避免持续占用 UNO Q 的 eMMC。
# Python 输出不缓冲，确保 docker logs 可实时查看节点日志。

ENV ROS_LOG_DIR=/tmp/unoq-ros-log \
    PYTHONUNBUFFERED=1 \
    ROS_WS=/opt/ros_ws


# ============================================================
# 4. 安装 ROS 2 依赖
# ============================================================

# rosbridge_server：
# 提供 WebSocket ROS 2 Topic 通信接口。
#
# foxglove_bridge：
# 为 Windows / 浏览器端 Foxglove 提供 ROS 2 数据连接。
#
# 手柄使用项目自定义 js_joy_node，
# 不需要 ros-jazzy-joy 或 ros-jazzy-teleop-twist-joy。
#
# nav2_map_server：
# slam_toolbox 的 /slam_toolbox/save_map 服务本身不写文件，实现是
#     system("ros2 run nav2_map_server map_saver_cli -f <name> ...")
# （slam_toolbox 2.8.5 src/map_saver.cpp）。缺这个包时 system() 返回非 0，
# 服务回 RESULT_UNDEFINED_FAILURE，日志里只有一行 "process has died" 之类的
# 间接痕迹，很容易误判成路径或权限问题。
#
# Nav2 导航栈（按需最小集，不装 navigation2 总包）：
#   nav2_amcl                    map->odom 定位，替代建图时的 slam_toolbox
#   nav2_planner + navfn         planner_server 与全局规划插件
#   nav2_controller + rpp        controller_server 与 Regulated Pure Pursuit
#   nav2_behaviors               spin / backup / wait 等恢复动作
#   nav2_bt_navigator            NavigateToPose 行为树
#   nav2_lifecycle_manager       一次性 configure/activate 上面这些节点
# 不装 nav2_bringup：它会连带拖进 rviz 插件与一整套默认参数，板子只有 9.8 GB eMMC。
#
# /dev/input 设备映射仍由 brick_compose.yaml 负责。

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-jazzy-rosbridge-server \
        ros-jazzy-foxglove-bridge \
        ros-jazzy-slam-toolbox \
        ros-jazzy-nav2-map-server \
        ros-jazzy-nav2-amcl \
        ros-jazzy-nav2-planner \
        ros-jazzy-nav2-navfn-planner \
        ros-jazzy-nav2-controller \
        ros-jazzy-nav2-regulated-pure-pursuit-controller \
        ros-jazzy-nav2-behaviors \
        ros-jazzy-nav2-bt-navigator \
        ros-jazzy-nav2-lifecycle-manager \
    && rm -rf /var/lib/apt/lists/*


# ============================================================
# 5. 构建 ROS 2 工作空间
# ============================================================

WORKDIR /opt/ros_ws

# 仅复制 ROS 2 源代码。
COPY ros_ws/src ./src

# 构建自定义 ROS 2 Python 包。
# 保留 install 目录，删除不需要的 build 和 log。

RUN bash -lc 'source /opt/ros/jazzy/setup.bash \
    && colcon build --packages-select arduino_ros2_bringup unoq_description \
       --event-handlers console_direct+ \
    && rm -rf build log'


# ============================================================
# 6. 容器启动入口
# ============================================================

COPY docker-entrypoint.sh /usr/local/bin/unoq-ros-entrypoint

RUN chmod +x /usr/local/bin/unoq-ros-entrypoint


# ============================================================
# 7. 优雅退出
# ============================================================

# 使用 SIGINT，让 ros2 launch 正确关闭所有节点。
# 避免默认 SIGTERM 导致节点异常退出。

STOPSIGNAL SIGINT


# ============================================================
# 8. 默认启动命令
# ============================================================

ENTRYPOINT ["/usr/local/bin/unoq-ros-entrypoint"]

CMD ["ros2", "launch", "arduino_ros2_bringup", "bringup.launch.py"]
