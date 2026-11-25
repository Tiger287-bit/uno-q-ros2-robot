"""rosbridge WebSocket 服务：容器内提供 ws://<主机>:<port> 的 ROS 2 图入口。

单独启动（改端口调试时）：
    ros2 launch arduino_ros2_bringup rosbridge.launch.py port:=9091

默认监听 0.0.0.0，但 brick_compose.yaml 刻意不映射宿主端口，所以只在
Compose 网络的容器之间可达；要用电脑上的 rviz 观察话题请走 SSH 隧道。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    @description         : 组装 rosbridge_server 官方 launch 的包含描述
    @return              : launch.LaunchDescription
    """
    port = LaunchConfiguration("port")
    address = LaunchConfiguration("address")

    return LaunchDescription([
        DeclareLaunchArgument(
            "port", default_value="9090",
            description="rosbridge WebSocket 端口，App 侧 ROSBRIDGE_URL 必须与之一致"),
        DeclareLaunchArgument(
            "address", default_value="0.0.0.0",
            description="监听地址；容器间通过 Compose 网络访问，故不限定 127.0.0.1"),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(
                get_package_share_directory("rosbridge_server"),
                "launch",
                "rosbridge_websocket_launch.xml",
            )),
            launch_arguments={"port": port, "address": address}.items(),
        ),
    ])
