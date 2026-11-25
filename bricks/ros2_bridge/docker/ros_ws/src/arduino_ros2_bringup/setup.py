"""ament_python 构建脚本：注册可执行入口，安装 launch 与参数文件。"""

import os
from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = "arduino_ros2_bringup"

# 新增节点时只需在 entry_points 里加一行，
# 例如 "odom_tf = arduino_ros2_bringup.nodes.odom_tf:main"，
# 之后即可 ros2 run arduino_ros2_bringup odom_tf。
ENTRY_POINTS = [
    "js_joy_node = arduino_ros2_bringup.nodes.js_joy_node:main",
    "cmd_vel_mux = arduino_ros2_bringup.nodes.cmd_vel_mux:main",
    "wheel_odom_node = arduino_ros2_bringup.nodes.wheel_odom_node:main",
    "scan_relay = arduino_ros2_bringup.nodes.scan_relay:main",
    "nav_bridge = arduino_ros2_bringup.nodes.nav_bridge:main",
    "mode_manager = arduino_ros2_bringup.nodes.mode_manager:main",
    "start_pose_tf = arduino_ros2_bringup.nodes.start_pose_tf:main",
]

setup(
    name=PACKAGE_NAME,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # ament 索引标记，colcon 与 ros2 pkg 靠它发现本包。
        (os.path.join("share", "ament_index", "resource_index", "packages"),
         [os.path.join("resource", PACKAGE_NAME)]),
        (os.path.join("share", PACKAGE_NAME), ["package.xml"]),
        # launch 与参数文件都装进 share/，launch 里用 FindPackageShare 取。
        (os.path.join("share", PACKAGE_NAME, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", PACKAGE_NAME, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="UNO Q ROS 2 Demo",
    maintainer_email="noreply@arduino.cc",
    description="Arduino UNO Q 的 ROS 2 启动包（rosbridge / 遥控 / 仲裁 / 里程计 / 建图 / 导航）",
    license="Apache-2.0",
    entry_points={"console_scripts": ENTRY_POINTS},
)
