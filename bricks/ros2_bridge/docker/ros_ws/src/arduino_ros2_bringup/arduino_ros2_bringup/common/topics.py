"""ROS 2 话题契约：全 brick 的话题名与消息类型只在这里定义一次。

为什么集中定义：
    话题名是跨容器的接口。App 主容器、网页、外部 ros2 命令行都按名字对接，
    一旦某处写死字符串，改名就要全仓库搜索。节点与 launch 统一从这里取常量，
    新增功能时也是先在这里补一行再写节点。

注意 App 侧是另一个容器：
    bricks/ros2_bridge/__init__.py 与 python/main.py 无法 import 本模块，
    它们各自保留了用到的常量。改动本文件时，请同步
    - bricks/ros2_bridge/__init__.py 里的 CMD_VEL_TOPIC / WEB_CMD_TOPIC 等常量
    - docs/API.md 与 README.md 的话题表
    这三处是同一份契约的多语言副本，文档里已标注。
"""


class Topics:
    """话题名（绝对路径，避免受节点命名空间影响）。"""

    # 速度链路
    CMD_VEL = "/cmd_vel"            # 仲裁输出，App 控制环订阅
    CMD_VEL_JOY = "/cmd_vel_joy"    # 手柄 -> 仲裁
    CMD_VEL_WEB = "/cmd_vel_web"    # 网页/App -> 仲裁
    CMD_VEL_NAV = "/cmd_vel_nav"    # Nav2 -> 仲裁；人工输入优先，见 mux 的仲裁规则
    CMD_VEL_SOURCE = "/cmd_vel_source"  # 仲裁当前持有者，导航据此判断"人手接管"

    # 调试与复用
    JOY = "/joy"                    # 手柄原始轴与按钮

    # 传感器与里程计
    ODOM = "/odom"                  # 轮式里程计输出
    JOINT_STATES = "/joint_states"  # 轮子关节角，App 侧从编码器发来
    IMU = "/imu"                    # 航向与角速度，App 侧从 MCU 转发
    SCAN = "/scan"                  # App 经 rosbridge 发的原始扫描
    SCAN_SLAM = "/scan_slam"        # 中继后的扫描（sensor data QoS），SLAM/AMCL 用

    # 导航
    MAP = "/map"                    # map_server 的 OccupancyGrid，transient_local
    PLAN = "/plan"                  # 全局路径
    AMCL_POSE = "/amcl_pose"        # AMCL 的位姿估计（map 坐标系）
    INITIAL_POSE = "/initialpose"   # 重设初始位姿（AMCL 订阅）
    NAV_UI = "/app/nav_ui"          # nav_bridge -> App，JSON 字符串
    NAV_CMD = "/app/nav_cmd"        # App -> nav_bridge/mode_manager，JSON 字符串
    MODE_STATE = "/app/mode_state"  # mode_manager -> nav_bridge，JSON 字符串
    NAV_HOLD = "/app/nav_hold"      # nav_bridge -> mux，true=禁止自动速度


class MsgTypes:
    """rosbridge 与 launch 里用到的消息类型字符串。"""

    TWIST = "geometry_msgs/msg/Twist"
    JOY = "sensor_msgs/msg/Joy"
    STRING = "std_msgs/msg/String"
