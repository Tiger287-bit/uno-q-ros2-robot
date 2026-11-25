"""速度仲裁：多路 Twist -> /cmd_vel。

单独启动（配合手动 ros2 topic pub 验证仲裁规则）：
    ros2 launch arduino_ros2_bringup mux.launch.py

输入路数由 config/mux.yaml 的 input_topics 决定，加注释掉的行即可增减。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PACKAGE = "arduino_ros2_bringup"


def generate_launch_description():
    """
    @description         : 组装仲裁节点启动描述
    @return              : launch.LaunchDescription
    """
    default_params = PathJoinSubstitution(
        [FindPackageShare(PACKAGE), "config", "mux.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file", default_value=default_params,
            description="参数文件，input_topics 在这里配置"),
        Node(
            package=PACKAGE,
            executable="cmd_vel_mux",
            name="cmd_vel_mux",
            parameters=[LaunchConfiguration("params_file")],
            output="screen",
            emulate_tty=True,
        ),
    ])
