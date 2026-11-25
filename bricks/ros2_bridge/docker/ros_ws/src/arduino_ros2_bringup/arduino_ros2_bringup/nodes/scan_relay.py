"""把 App 的 /scan 原样转发一份，供 slam_toolbox 订阅。

为什么需要它
    /scan 是由 App 经 rosbridge 广播的。rosbridge 给话题挂的 QoS 是
    RELIABLE + TRANSIENT_LOCAL + lifespan 1 s，而 slam_toolbox 用的是
    sensor data QoS（BEST_EFFORT + VOLATILE）。两者理论上兼容，但实测
    slam 只在启动时收到一帧、之后彻底静默（/map 和 /slam_toolbox/update
    都不再发布，日志里连丢帧都不打）。

    这个中继用 ROS 原生 publisher 重新发一份，QoS 用 sensor data，
    与 slam 的期望完全一致；时间戳原样保留，不改动几何关系。

    它只做转发，不做任何坐标或时间上的修正 —— 一旦 slam 这边确认能
    正常建图，就可以回头再查 rosbridge 那条链路的 QoS 细节。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

INPUT_TOPIC = "/scan"
OUTPUT_TOPIC = "/scan_slam"


class ScanRelay(Node):
    """订阅 /scan，按 sensor data QoS 转发到 /scan_slam。"""

    def __init__(self):
        super().__init__("scan_relay")
        # 订阅端用默认 QoS：rosbridge 的发布端是 RELIABLE，默认 QoS 能配上。
        self.create_subscription(LaserScan, INPUT_TOPIC, self.on_scan, 10)
        # 发布端用 sensor data QoS，与 slam_toolbox 的订阅端一致。
        self.publisher = self.create_publisher(LaserScan, OUTPUT_TOPIC,
                                               qos_profile_sensor_data)
        self.count = 0
        self.get_logger().info("扫描中继就绪：%s -> %s" % (INPUT_TOPIC, OUTPUT_TOPIC))

    def on_scan(self, msg):
        self.publisher.publish(msg)     # 原样转发，时间戳不动
        self.count += 1
        if self.count % 50 == 1:
            self.get_logger().info("已转发 %d 帧" % self.count)


def main():
    rclpy.init()
    node = ScanRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
