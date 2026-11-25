"""arduino_ros2_bringup —— 板端 ROS 2 容器里的全部节点与启动文件。

分层（细节见 docs/ARCHITECTURE.md）：

    common/   跨节点共享的契约：话题名、消息类型。改这里等于改全 brick 的接口。
    nodes/    每个文件一个节点类 + main()，不互相 import，只依赖 common/。
    config/   ROS 2 参数 YAML，按节点分文件；换手柄、调频率只改这里。
    launch/   bringup 组合入口 + 每个功能一个可单独启动的 launch 文件。

约束：
    nodes/ 下不要写模块级副作用（打开设备、建 socket、起线程）。所有资源
    获取都放在节点的 __init__ 里，这样 `ros2 param dump`、`--show-args`
    之类的工具不会意外拉起硬件。
"""
