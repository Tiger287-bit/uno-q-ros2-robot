"""Nav2 导航：加载已建好的地图、AMCL 定位、规划与控制。

为什么和 slam.launch.py 分开
    两者都发布 map->odom：slam_toolbox 建图时自己算，AMCL 定位时自己算。
    同时 active 会出现两个发布者抢同一个 TF，表现为定位跳变、路径画歪。
    bringup 里两套都启动，但都把 autostart 关掉，由 mode_manager 决定谁 active。

为什么自带扫描中继
    与 slam.launch.py 同理：AMCL 与局部代价地图都按 sensor data QoS 订阅，
    而 /scan 是 rosbridge 发的 RELIABLE+TRANSIENT_LOCAL，直接订阅有
    "只收到第一帧"的旧账。中继后的 /scan_slam 两边都认。

单独跑
    ros2 launch arduino_ros2_bringup nav.launch.py
    ros2 launch arduino_ros2_bringup nav.launch.py map_yaml:=/opt/ros_ws/maps/room1.yaml
    ros2 launch arduino_ros2_bringup nav.launch.py autostart:=false   # 交给外部管理
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PACKAGE = "arduino_ros2_bringup"
# 建图时存下的地图。compose 把宿主机的 docker/ros_ws/maps 挂到这里。
DEFAULT_MAP = "/opt/ros_ws/maps/room_20261122.yaml"
# 地图坐标系名，与 nav2.yaml 的 global_frame 一致。
MAP_FRAME = "map"


# 注意：不要设 bt_navigator 的 plugin_lib_names。
#   Jazzy 的行为树插件是"内建自动注册"的（官方 nav2_params.yaml 里那行是
#   # plugin_lib_names: []），这个参数只用来追加自定义插件。把所有
#   lib*_bt_node.so 扫出来塞进去，会让每个插件注册两次，configure 阶段直接
#   FATAL: ID [AreErrorCodesPresent] already registered，bt_navigator 段错误退出。


def generate_launch_description():
    """
    @description         : 组装地图服务、AMCL、规划/控制/恢复与行为树
    @return              : launch.LaunchDescription
    """
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("nav_params_file")
    map_yaml = LaunchConfiguration("map_yaml")
    enable_scan_relay = LaunchConfiguration("enable_scan_relay")
    autostart = LaunchConfiguration("autostart")
    use_amcl = LaunchConfiguration("use_amcl")

    def node(package, executable, name, extra=None, remappings=None):
        """统一写法：参数文件 + 覆盖项 + 生命周期由 manager 负责。"""
        parameters = [params_file,
                      {"use_sim_time": ParameterValue(use_sim_time,
                                                      value_type=bool)}]
        if extra:
            parameters.append(extra)
        return Node(package=package, executable=executable, name=name,
                    output="log", parameters=parameters,
                    remappings=remappings or [])

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="实车必须 false；只有回放 bag 时才置 true"),
        DeclareLaunchArgument(
            "nav_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare(PACKAGE), "config", "nav2.yaml"]),
            description="Nav2 参数文件"),
        DeclareLaunchArgument(
            "map_yaml", default_value=DEFAULT_MAP,
            description="已保存地图的 yaml（pgm 与它同目录）"),
        DeclareLaunchArgument(
            "enable_scan_relay", default_value="true",
            description="是否启动 /scan -> /scan_slam 中继"),
        # bringup 传 false：六个节点的 configure/activate 由 mode_manager
        # 按"导航/建图"的切换需要来调，不能让 manager 一上来就全激活。
        DeclareLaunchArgument(
            "autostart", default_value="true",
            description="lifecycle manager 是否立刻激活全部导航节点"),
        DeclareLaunchArgument(
            "use_amcl", default_value="false",
            description="true 用 AMCL 定位；false（默认）用固定起始位姿："
                        "map->odom 恒为单位变换，位姿等于里程计位姿"),

        LogInfo(msg=["[nav] map=", map_yaml, " params=", params_file,
                     " relay=", enable_scan_relay]),

        # 扫描中继：与 slam.launch.py 里的实现相同，二者互斥不会重复起。
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

        # 地图：发布 /map（transient_local），并把栅格喂给全局代价地图的静态层。
        node("nav2_map_server", "map_server", "map_server",
             {"yaml_filename": map_yaml}),

        # 定位。默认走"固定起始位姿"：map->odom 是一条恒为单位变换的静态 TF，
        # 机器人位姿直接等于里程计位姿 —— 每次开机都认为车在地图原点、朝向 +x。
        # 去掉 AMCL 的扫描匹配之后，定位跳变、TF 外推、粒子收敛这些全都不存在了；
        # 代价是开机必须把车摆在原点，中途被抱走位置就对不上（只能重启应用）。
        # 想切回 AMCL 用 use_amcl:=true，nav2.yaml 里的 amcl 段原样保留。
        # 可调的 map->odom：默认 (0,0,0)，页面重设初始位姿时通过 /initialpose
        # 挪过去。比 static_transform_publisher 实用——后者发出去就改不了，
        # 车摆偏一点整张图就跟着偏，规划会直接撞墙。
        Node(package=PACKAGE, executable="start_pose_tf", name="start_pose_tf",
             output="log",
             parameters=[{"x": 0.0, "y": 0.0, "yaw": 0.0, "rate_hz": 20.0}],
             condition=UnlessCondition(use_amcl)),
        Node(package="nav2_amcl", executable="amcl", name="amcl", output="log",
             parameters=[params_file,
                         {"use_sim_time": ParameterValue(use_sim_time,
                                                         value_type=bool)}],
             condition=IfCondition(use_amcl)),

        node("nav2_planner", "planner_server", "planner_server"),
        # 关键重映射：controller_server 与 behavior_server 默认把速度发到
        # /cmd_vel，而 /cmd_vel 是 mux 的输出口。不重映射会有两个发布者抢同一
        # 个话题，App 收到的速度帧一半来自 mux、一半直接来自 Nav2 —— 人工
        # 与导航的仲裁完全失效。走 /cmd_vel_nav 才是 mux 的一路输入。
        node("nav2_controller", "controller_server", "controller_server",
             remappings=[("cmd_vel", "/cmd_vel_nav")]),
        node("nav2_behaviors", "behavior_server", "behavior_server",
             remappings=[("cmd_vel", "/cmd_vel_nav")]),
        node("nav2_bt_navigator", "bt_navigator", "bt_navigator"),

        # 一次性把上面六个节点 configure + activate。少一个就会停在
        # unconfigured，表现为"节点在但话题永远不出"。
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="log",
            parameters=[params_file,
                        {"use_sim_time": ParameterValue(use_sim_time,
                                                        value_type=bool),
                         "autostart": ParameterValue(autostart,
                                                     value_type=bool)}],
        ),

        # 与 App 的 JSON 桥：动作客户端、地图编码、人工接管取消都在里面。
        node(PACKAGE, "nav_bridge", "nav_bridge"),
    ])
