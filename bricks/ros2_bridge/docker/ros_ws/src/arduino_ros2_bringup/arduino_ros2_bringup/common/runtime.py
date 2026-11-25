"""节点运行骨架：统一 rclpy 生命周期，供各节点 main() 复用。

为什么单独抽出来：
    每个节点各写一份 `rclpy.init/spin/shutdown` 很容易漏掉边界情况。这里踩过
    的坑都收在这一处：
    1. 停机时 docker 把 STOPSIGNAL 发给容器 PID 1（本镜像是 tini），tini 转给
       ros2 launch，launch 再向各节点转发同一个信号；rclpy 收到 SIGINT 后关
       context，spin 抛 ExternalShutdownException，这里接住就不打 traceback。
       前提是镜像里设了 STOPSIGNAL SIGINT：默认的 SIGTERM 会按默认动作直接杀
       掉 launch，节点被一起带走，这条清理路径根本跑不到（实测 443 ms、
       退出码 143、零清理日志）。
    2. context 已被关过时再调 rclpy.shutdown() 会抛 RCLError
       （rcl_shutdown already called），所以先判 rclpy.ok()。
"""

import rclpy
from rclpy.executors import ExternalShutdownException


def run(factory):
    """
    @description         : 初始化 rclpy、构造节点并阻塞运行，退出时释放资源
    @param factory       : 无参可调用对象，返回已构造的 rclpy.node.Node
    @return              : None
    """
    rclpy.init()
    node = None
    try:
        node = factory()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
