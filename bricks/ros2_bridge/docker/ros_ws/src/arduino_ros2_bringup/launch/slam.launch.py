"""SLAM Toolbox：扫描中继 + 异步建图，发布 /map 与 map→odom。

为什么单独一个 launch
    与其它功能一样按"一个功能一个文件"组织，便于单独排查：
        ros2 launch arduino_ros2_bringup slam.launch.py use_sim_time:=false
    bringup 只是按开关把它包含进来。

链路完整形状
    /scan --(scan_relay)--> /scan_slam --(本节点)--> map
    map --(本节点)--> odom --(wheel_odom)--> base_footprint
         --(robot_state_publisher, URDF)--> base_link --> laser_link
    上游两段必须都在跑，否则 slam_toolbox 会一直等 TF 而不出图。

中继为什么放在这里而不是 bringup
    slam 的 scan_topic 是 /scan_slam，而 /scan 是 App 经 rosbridge 发的，
    QoS 是 RELIABLE + TRANSIENT_LOCAL，slam 订阅端要的是 sensor data QoS。
    早先中继只写在 bringup 里，于是"单独跑 slam.launch.py 排查"这条路
    必然等不到扫描。中继跟着 slam 走，两个入口才是同一条链路；
    bringup 那边不再重复起，否则两个中继会把同一帧发两遍。

use_sim_time 默认 false
    实车没有 /clock，一旦置 true，节点所有时间戳都停在 0，
    表现为"扫描永远对不上 TF"。回放 bag 才需要 true，由调用方显式传。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PACKAGE = "arduino_ros2_bringup"


def generate_launch_description():
    """
    @description         : 组装扫描中继与 SLAM Toolbox 的启动描述
    @return              : launch.LaunchDescription
    """
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("slam_params_file")
    enable_scan_relay = LaunchConfiguration("enable_scan_relay")
    autostart = LaunchConfiguration("autostart")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="实车必须 false；只有回放 bag 时才置 true"),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare(PACKAGE), "config", "slam.yaml"]),
            description="SLAM Toolbox 参数文件"),
        DeclareLaunchArgument(
            "enable_scan_relay", default_value="true",
            description="是否启动 /scan -> /scan_slam 中继。"
                        "已经有别的进程在发 /scan_slam 时置 false"),
        # bringup 传 false：建图节点常驻但不激活，等 mode_manager 切过来。
        # 注意 slam_toolbox 的 autostart 走的是 launch 事件（configure+activate），
        # 不是 lifecycle_manager；关掉之后节点停在 unconfigured，服务仍在。
        DeclareLaunchArgument(
            "autostart", default_value="true",
            description="是否立刻 configure + activate slam_toolbox"),

        LogInfo(msg=["[slam] use_sim_time=", use_sim_time,
                     " params=", params_file,
                     " relay=", enable_scan_relay]),

        # 扫描中继：把 App 经 rosbridge 发的 /scan 用 sensor data QoS 转一份。
        Node(
            package=PACKAGE,
            executable="scan_relay",
            name="scan_relay",
            output="log",
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }],
            condition=IfCondition(enable_scan_relay),
        ),

        # slam_toolbox 的 launch 在它自己的包里，不能用本包的 _sub_launch。
        # 它自带 lifecycle 的 configure/activate（autostart），
        # 自己起 async_slam_toolbox_node 会停在 unconfigured 不出图。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("slam_toolbox"),
                                      "launch", "online_async_launch.py"])),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "slam_params_file": params_file,
                "autostart": autostart,
            }.items(),
        ),
    ])
