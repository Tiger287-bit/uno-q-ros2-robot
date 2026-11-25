"""手柄遥控：js_joy_node -> /cmd_vel_joy（并发布 /joy）。

单独启动（不跑 rosbridge / 仲裁，直接命令行看话题）：
    ros2 launch arduino_ros2_bringup teleop.launch.py
    ros2 launch arduino_ros2_bringup teleop.launch.py params_file:=/tmp/mine.yaml

全部参数只有 config/teleop.yaml 一个来源，本文件不再单独传 device。
早先这里额外把 {"device": LaunchConfiguration("joy_device")} 追加到 parameters，
而 launch_ros 按顺序后者覆盖前者 —— 那个硬默认 /dev/input/js0 会盖掉 YAML，
把手柄换到 js1 时按文档改 YAML 完全没反应。要临时换设备，复制一份 YAML 传给
params_file；别用 `ros2 param set`：device 只在节点构造时读一次，改参数不会
重新打开设备。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PACKAGE = "arduino_ros2_bringup"


def generate_launch_description():
    """
    @description         : 组装手柄节点启动描述
    @return              : launch.LaunchDescription
    """
    default_params = PathJoinSubstitution(
        [FindPackageShare(PACKAGE), "config", "teleop.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file", default_value=default_params,
            description="参数文件；换成自己的 YAML 即可在线调试映射"),
        Node(
            package=PACKAGE,
            executable="js_joy_node",
            name="js_joy_node",
            parameters=[LaunchConfiguration("params_file")],
            output="screen",
            emulate_tty=True,
        ),
    ])
