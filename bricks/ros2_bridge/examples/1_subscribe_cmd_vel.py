"""示例 1：在 App 的 main.py 里订阅 /cmd_vel，供控制环读取。

运行位置是 App 主容器，不是 ros2 容器 —— Ros2Subscriber 走 rosbridge 的
WebSocket，不需要容器内有 ROS 2 环境。
"""

from ros2_bridge import Ros2Subscriber

# 模块级实例化：构造阶段不建链，因此写在 import 之后是安全的。
LINK = Ros2Subscriber(topic="/cmd_vel", message_type="geometry_msgs/msg/Twist")

# 链路新鲜度上限，s。超过视为上游没在发，按停车处理。
STALE_S = 0.5


def start():
    """启动后台订阅线程。"""
    LINK.start()


def current_speed():
    """读取最新速度指令。

    @return: (线速度 m/s, 角速度 rad/s)；链路不可用或数据过期时返回 (0.0, 0.0)
    """
    message, age_s, connected = LINK.snapshot()
    if not connected or age_s > STALE_S:
        return 0.0, 0.0

    linear = message.get("linear", {}).get("x", 0.0)
    angular = message.get("angular", {}).get("z", 0.0)
    return linear, angular


if __name__ == "__main__":
    start()
    print("ROSBRIDGE_URL =", LINK.url)
    print("当前指令:", current_speed())
