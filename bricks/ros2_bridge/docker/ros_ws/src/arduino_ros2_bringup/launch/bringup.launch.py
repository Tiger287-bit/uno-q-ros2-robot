"""容器内全量启动：rosbridge + 手柄遥控 + 速度仲裁。

节点关系：
    /dev/input/js* -> js_joy_node -> /cmd_vel_joy -+
                                                   +-> cmd_vel_mux -> /cmd_vel -> rosbridge(9090)
    浏览器(App) -> /cmd_vel_web ------------------+

    另：js_joy_node 发布 /joy（原始轴与按钮，仅供调试）。

组成方式：
    bringup 只做编排，本身不含逻辑。每个功能一个 launch 文件，这里用
    IncludeLaunchDescription 按开关拼起来，所以任何一部分都能单独
    `ros2 launch arduino_ros2_bringup <功能>.launch.py` 起，便于逐个排查。

    >>> 新增 ROS 功能时，在这里加一个 IncludeLaunchDescription 与一个
    >>> enable_xxx 开关即可，节点代码不进入本文件。

    ros2 launch arduino_ros2_bringup bringup.launch.py --show-args   # 看全部开关
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import (AnyLaunchDescriptionSource,
                                               PythonLaunchDescriptionSource)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PACKAGE = "arduino_ros2_bringup"


def _sub_launch(filename):
    """
    @description         : 引用同包 launch/ 目录下的子启动文件
    @param filename      : 文件名，含 .launch.py 后缀
    @return              : PythonLaunchDescriptionSource
    """
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(PACKAGE), "launch", filename]))


def _robot_description():
    """
    @description         : 读取 unoq_description 里的 URDF 文本
    @return              : URDF 字符串；包或文件缺失时返回空串，让下面的条件判断跳过节点
    """
    try:
        path = os.path.join(get_package_share_directory("unoq_description"),
                            "urdf", "unoq.urdf")
        with open(path, "r", encoding="utf-8") as handle:
            return handle.read()
    except Exception:
        return ""


def generate_launch_description():
    """
    @description         : 按开关组装各功能启动描述
    @return              : launch.LaunchDescription
    """
    enable_rosbridge = LaunchConfiguration("enable_rosbridge")
    enable_teleop = LaunchConfiguration("enable_teleop")
    enable_mux = LaunchConfiguration("enable_mux")
    enable_description = LaunchConfiguration("enable_description")
    enable_scan_relay = LaunchConfiguration("enable_scan_relay")
    enable_foxglove = LaunchConfiguration("enable_foxglove")
    enable_odom = LaunchConfiguration("enable_odom")
    enable_slam = LaunchConfiguration("enable_slam")
    enable_nav = LaunchConfiguration("enable_nav")

    return LaunchDescription([
        # ---- 开关：默认全开，与 brick_config.yaml 描述的默认行为一致 ----
        DeclareLaunchArgument(
            "enable_rosbridge", default_value="true",
            description="是否启动 rosbridge（关掉后 App 侧无法连接）"),
        DeclareLaunchArgument(
            "enable_teleop", default_value="true",
            description="是否启动手柄节点（不插手柄可关闭以省资源）"),
        DeclareLaunchArgument(
            "enable_mux", default_value="true",
            description="是否启动速度仲裁（关掉后须自己往 /cmd_vel 发指令）"),
        DeclareLaunchArgument(
            "rosbridge_port", default_value="9090",
            description="rosbridge 端口，须与 App 侧 ROSBRIDGE_URL 一致"),
        # URDF 里的固定关节（base_footprint→base_link、laser_link、caster）
        # 由 robot_state_publisher 一次性发到 /tf_static，替代外部手写的静态变换。
        DeclareLaunchArgument(
            "enable_description", default_value="true",
            description="是否启动 robot_state_publisher（发布 URDF 的固定关节 TF）"),
        DeclareLaunchArgument(
            "enable_foxglove", default_value="true",
            description="是否启动 foxglove_bridge（网页/桌面端可视化）"),
        DeclareLaunchArgument(
            "foxglove_port", default_value="8765",
            description="foxglove websocket 端口，须与 brick_compose 的映射一致"),
        # 轮式里程计。参数与底盘一致：轮径 65 mm、轮距 153.5 mm。
        DeclareLaunchArgument(
            "enable_odom", default_value="true",
            description="是否启动 wheel_odom_node（发布 /odom 与 odom→base_footprint）"),
        DeclareLaunchArgument(
            "wheel_radius", default_value="0.0325",
            description="车轮半径，米"),
        DeclareLaunchArgument(
            "wheel_separation", default_value="0.1535",
            description="两轮中心距，米"),
        # 航向改用 IMU：编码器差速实测量出 3.5°，而 IMU 和雷达都指向 13~14°，
        # 差速在转向时明显欠计。位移仍取轮速，只有 dψ 换源。
        DeclareLaunchArgument(
            "use_imu_heading", default_value="true",
            description="里程计航向取 IMU 而非两轮差速"),
        # SLAM：订阅 /scan 建图，发布 /map 与 map→odom。
        # 需要 wheel_odom（odom→base_footprint）与 robot_state_publisher
        # （base_footprint→…→laser_link）同时在线，否则它只等 TF 不出图。
        #
        # 默认 true：网页上的"建图/导航"开关靠它常驻 —— 切模式只是激活/停用，
        # 不重启进程。两套都启动时 autostart 一律 false，谁 active 由
        # mode_manager 决定（它们都发 map->odom，绝不能同时 active）。
        DeclareLaunchArgument(
            "enable_slam", default_value="true",
            description="是否启动 slam_toolbox（常驻待激活，供模式切换）"),
        # 导航：map_server + AMCL + 规划/控制/行为树，发布 map→odom 与 /cmd_vel_nav。
        # 导航：map_server + AMCL + 规划/控制/行为树，发布 map→odom 与 /cmd_vel_nav。
        # 默认 true。两套栈常驻但只有一套 active，互斥由 mode_manager 保证。
        DeclareLaunchArgument(
            "enable_nav", default_value="true",
            description="是否启动 Nav2 导航（与 enable_slam 互斥）"),
        # 开机进入哪个模式。导航是日常状态；改成 slam 可以直接开机进建图，
        # 但地图从空白开始，日常不建议。
        DeclareLaunchArgument(
            "initial_mode", default_value="nav",
            description="开机模式：nav（导航，默认）或 slam（建图）"),
        # 中继只在这里起一份：nav.launch / slam.launch 各自也带一份（单独
        # 排查时要用），由这里显式传 false 关掉，否则同一帧 /scan 会被转发
        # 两遍，slam 把每帧当两帧处理。
        DeclareLaunchArgument(
            "enable_scan_relay", default_value="true",
            description="是否启动 /scan -> /scan_slam 中继（bringup 统一起一份）"),
        DeclareLaunchArgument(
            "map_yaml", default_value="/opt/ros_ws/maps/room_20261122.yaml",
            description="导航使用的地图 yaml（pgm 需同目录）"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="实车必须 false；回放 bag 时才置 true"),
        # 手柄设备路径刻意不在这里开参数：它只有一个来源 config/teleop.yaml。
        # 早先这里再透传一次 joy_device（默认 js0），与 teleop.launch.py 一起
        # 形成两重硬默认，把 YAML 彻底顶掉。

        # msg 是替换列表，各项在启动时才求值；不能用 % 拼 LaunchConfiguration。
        LogInfo(msg=["[bringup] rosbridge=", enable_rosbridge,
                     " teleop=", enable_teleop,
                     " mux=", enable_mux,
                     " slam=", enable_slam,
                     " nav=", enable_nav,
                     " mode=", LaunchConfiguration("initial_mode")]),

        # URDF 为空时（包没装进镜像）直接不建这个节点，避免 launch 整体失败。
        *([Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="log",
            parameters=[{"robot_description": _robot_description()}],
            condition=IfCondition(enable_description),
        )] if _robot_description() else []),

        # ---- 功能：按需包含 ----
        IncludeLaunchDescription(
            _sub_launch("rosbridge.launch.py"),
            launch_arguments={"port": LaunchConfiguration("rosbridge_port")}.items(),
            condition=IfCondition(enable_rosbridge),
        ),
        IncludeLaunchDescription(
            _sub_launch("teleop.launch.py"),
            condition=IfCondition(enable_teleop),
        ),
        IncludeLaunchDescription(
            _sub_launch("mux.launch.py"),
            condition=IfCondition(enable_mux),
        ),
        # 轮式里程计：读 /joint_states 积分出位姿。
        Node(
            package="arduino_ros2_bringup",
            executable="wheel_odom_node",
            name="wheel_odom",
            output="log",
            parameters=[{
                "wheel_radius": ParameterValue(
                    LaunchConfiguration("wheel_radius"), value_type=float),
                "wheel_separation": ParameterValue(
                    LaunchConfiguration("wheel_separation"), value_type=float),
                "use_imu_heading": ParameterValue(
                    LaunchConfiguration("use_imu_heading"), value_type=bool),
            }],
            condition=IfCondition(enable_odom),
        ),

        # 扫描中继：统一在这里起一份，两个子 launch 都用 enable_scan_relay=false
        # 关掉自己那份。这样"只开导航""只开建图""两个都开"三种组合都只有
        # 一个中继，不会出现同一帧 /scan 被转发两遍。
        Node(
            package=PACKAGE,
            executable="scan_relay",
            name="scan_relay",
            output="log",
            parameters=[{
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool),
            }],
            condition=IfCondition(enable_scan_relay),
        ),

        # SLAM / 导航都常驻，但 autostart 一律关掉：两者都发 map->odom，
        # 同时 active 就是两个发布者抢同一个 TF。谁 active 由 mode_manager
        # 通过生命周期服务决定（见 nodes/mode_manager.py）。
        IncludeLaunchDescription(
            _sub_launch("slam.launch.py"),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "enable_scan_relay": "false",
                "autostart": "false",
            }.items(),
            condition=IfCondition(enable_slam),
        ),
        IncludeLaunchDescription(
            _sub_launch("nav.launch.py"),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "map_yaml": LaunchConfiguration("map_yaml"),
                "enable_scan_relay": "false",
                "autostart": "false",
            }.items(),
            condition=IfCondition(enable_nav),
        ),
        # 模式管理：开机按 initial_mode 激活一套栈，之后接网页的切换请求。
        # 放在两个 include 之后，节点名固定，服务地址不随 launch 变化。
        Node(
            package=PACKAGE,
            executable="mode_manager",
            name="mode_manager",
            output="log",
            parameters=[{
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool),
                "initial_mode": LaunchConfiguration("initial_mode"),
            }],
        ),

        # foxglove 的 launch 文件在它自己的包里，不能用本包的 _sub_launch。
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("foxglove_bridge"),
                                      "launch", "foxglove_bridge_launch.xml"])),
            launch_arguments={
                "port": LaunchConfiguration("foxglove_port"),
                # 只读：去掉 clientPublish / parameters / services，
                # 否则局域网内任何人都能往 /cmd_vel 发消息直接驱动底盘。
                # 保留 connectionGraph 与 assets —— 前者是话题列表，后者供网格读取。
                "capabilities": "['connectionGraph', 'assets']",
            }.items(),
            condition=IfCondition(enable_foxglove),
        ),
    ])
