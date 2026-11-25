#!/usr/bin/env python3
"""速度指令仲裁节点：多路 Twist 输入 -> 单一 /cmd_vel 输出。

为什么需要仲裁：
    手柄节点与网页键盘是两条独立输入链路。若都直接写 /cmd_vel，后到的消息会
    覆盖先到的 —— 手柄空闲时持续输出的 0 会把网页指令瞬间冲掉。

仲裁规则（人工优先 + 非零接管）：
    1. manual_topics 里的输入（手柄、网页）只要在 manual_hold_s 内有非零指令，
       就由它输出。导航（或任何非人工来源）永远抢不过人手；
    2. 人工松手（发零）后立刻输出 0，并在 manual_hold_s 内继续压住自动来源，
       避免"松手一瞬间导航把车又拽走"；
    3. 没有人工输入时，非人工来源按最近一帧非零指令接管，静默超过 hold_s
       视为结束，输出 0；
    4. 无来源时恒定发布 0，下游不必依赖"消息超时"来判断停车。

    为什么要"人工优先"而不是"最后非零者赢"：
    Nav2 控制器以 10 Hz 连续发速度，人手推摇杆的频率与它相当。按最后
    非零者裁决时，导航帧会周期性地压过手柄帧（实测：手柄 -> 导航 -> 定时
    发布，最终输出仍来自导航），表现为人推着摇杆、车却被导航拽走，而且
    控制权没变回人工，nav_bridge 也就不会取消目标。改成优先级后，只要
    人手还在动，导航永远拿不到控制权。

    "非零才争权"这条保留着：手柄按住使能键、摇杆回中时会以 50 Hz 持续发 0，
    若让 0 也参与竞争，它会一直霸占控制权，网页键盘怎么点都被下一帧 0 覆盖。

输入路数由参数 input_topics 决定，优先级由 manual_topics 决定：
    追加 /cmd_vel_rc 之类的新人工源时，两个列表都加一项即可。

发布：
    /cmd_vel         geometry_msgs/Twist，publish_hz 固定频率
    /cmd_vel_source  std_msgs/String，当前来源；变化时立刻发，另按
                     source_hz 心跳重发（App 靠它决定是否关闭轮速吸附）
"""

import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

from arduino_ros2_bringup.common.runtime import run
from arduino_ros2_bringup.common.topics import Topics

# 参数默认值：与 config/mux.yaml 保持一致，YAML 缺失时也能正常跑。
DEFAULT_INPUT_TOPICS = [Topics.CMD_VEL_JOY, Topics.CMD_VEL_WEB,
                        Topics.CMD_VEL_NAV]
# 人工来源：优先级高于自动来源（导航）。顺序不代表优先级，同时有多路非零时
# 取时间戳最新的一路 —— 手不会同时操作手柄和键盘。
DEFAULT_MANUAL_TOPICS = [Topics.CMD_VEL_JOY, Topics.CMD_VEL_WEB]
DEFAULT_PUBLISH_HZ = 50.0    # 输出频率，与 js_joy_node 一致
DEFAULT_HOLD_S = 0.25        # 自动来源静默多久视为结束，s
DEFAULT_MANUAL_HOLD_S = 0.30 # 人工输入非零后压住自动来源的时长，s
DEFAULT_SOURCE_HZ = 2.0      # 来源话题心跳频率，s
DEFAULT_TAKEOVER_LATCH_S = 1.5  # 人工动过之后压住自动来源的时长，s
DEFAULT_EPSILON = 1e-4       # 判定"非零"的下限，滤掉摇杆回中后的浮点残值


class CmdVelMux(Node):
    """把多路速度指令按控制权模型合并为单一 /cmd_vel 输出。"""

    def __init__(self):
        """
        @description         : 读取参数、建立多路订阅与一路发布，并启动定时合帧
        @return              : None
        """
        super().__init__("cmd_vel_mux")
        self._read_parameters()

        self._lock = threading.Lock()
        # 每路保存 (线速度, 角速度, 单调时间戳)，键为话题名。
        self._values = {topic: (0.0, 0.0, 0.0) for topic in self.input_topics}
        self._owner = None
        self._block_auto_until = 0.0
        # nav_bridge 说"旧目标还没停"时为真：锁存时长兜不住的情况由它补。
        self._nav_hold = False     # 人工动过之后，自动来源要等到这个时刻
        self._reported = None
        self._reported_at = 0.0

        self.publisher = self.create_publisher(Twist, self.output_topic, 10)
        # 控制权归属，只在变化时发。导航侧靠它判断"人是不是在开车"：
        # 一旦变回手柄或网页，就取消当前导航目标，免得人一松手车又自己跑。
        self.source_publisher = self.create_publisher(
            String, Topics.CMD_VEL_SOURCE, 10)
        for topic in self.input_topics:
            self.create_subscription(
                Twist, topic, lambda msg, topic=topic: self._on_cmd(topic, msg), 10)
        self.create_subscription(Bool, Topics.NAV_HOLD, self._on_hold, 10)
        self.create_timer(1.0 / self.publish_hz, self._publish)

        self.get_logger().info(
            "速度仲裁就绪: %s -> %s, %.0f Hz, 松手 %.2fs 归零, 人工优先 %s"
            % (" + ".join(self.input_topics), self.output_topic,
               self.publish_hz, self.hold_s,
               ",".join(self.manual_topics) or "无"))

    def _read_parameters(self):
        """
        @description         : 集中声明并读取参数，保证与 config/mux.yaml 一一对应
        @return              : None
        """
        self.input_topics = [str(topic) for topic in self.declare_parameter(
            "input_topics", DEFAULT_INPUT_TOPICS).value]
        self.output_topic = str(
            self.declare_parameter("output_topic", Topics.CMD_VEL).value)
        self.publish_hz = float(
            self.declare_parameter("publish_hz", DEFAULT_PUBLISH_HZ).value)
        self.hold_s = float(self.declare_parameter("hold_s", DEFAULT_HOLD_S).value)
        self.manual_hold_s = float(self.declare_parameter(
            "manual_hold_s", DEFAULT_MANUAL_HOLD_S).value)
        self.takeover_latch_s = float(self.declare_parameter(
            "takeover_latch_s", DEFAULT_TAKEOVER_LATCH_S).value)
        self.manual_topics = [str(topic) for topic in self.declare_parameter(
            "manual_topics", DEFAULT_MANUAL_TOPICS).value]
        self.source_hz = float(
            self.declare_parameter("source_hz", DEFAULT_SOURCE_HZ).value)
        self.epsilon = float(
            self.declare_parameter("epsilon", DEFAULT_EPSILON).value)

        if not self.input_topics:
            raise ValueError("input_topics 不能为空，否则仲裁节点没有任何输入源")
        if self.publish_hz <= 0.0:
            raise ValueError(f"publish_hz 必须为正数，当前 {self.publish_hz}")
        # 配错名字时不静默：manual_topics 里写了 input_topics 没有的话题，
        # 那一路永远不会被订阅，人工优先也就失效了。
        unknown = [t for t in self.manual_topics if t not in self.input_topics]
        if unknown:
            raise ValueError("manual_topics 里的 %s 不在 input_topics 中" % unknown)

    def _on_hold(self, msg):
        """导航侧声明"目标还没结束"，期间不放行任何自动速度。"""
        self._nav_hold = bool(msg.data)

    def _on_cmd(self, topic, msg):
        """
        @description         : 按控制权模型处理一路输入
        @param topic         : 来源话题名
        @param msg           : geometry_msgs/Twist 消息
        @return              : None
        """
        moving = abs(msg.linear.x) > self.epsilon or abs(msg.angular.z) > self.epsilon
        now = time.monotonic()
        urgent = None
        with self._lock:
            self._values[topic] = (msg.linear.x, msg.angular.z, now)
            if not moving or topic not in self.manual_topics:
                return
            # 人工一动：立刻压住自动来源，并且**马上**把接管事件发出去。
            # 不能等 _publish 里的 _pick：人手一拍非零、下一拍就回中时，
            # _pick 可能一次都没选中它，事件永远发不出去，导航桥也就不会
            # 取消目标 —— 0.3 s 后车又按旧目标走了。
            self._block_auto_until = now + self.takeover_latch_s
            urgent = topic
        if urgent is not None and urgent != self._reported:
            # 锁外发，避免和 _publish 抢锁
            self._announce(urgent)

    def _pick(self, now):
        """选当前来源：人工优先，其次自动。调用方必须已持锁。"""
        manual = None
        manual_stamp = -1.0
        for topic in self.manual_topics:
            linear, angular, stamp = self._values[topic]
            if (abs(linear) > self.epsilon or abs(angular) > self.epsilon) \
                    and now - stamp <= self.manual_hold_s and stamp > manual_stamp:
                manual, manual_stamp = topic, stamp
        if manual is not None:
            # 锁存由 _on_cmd 按 takeover_latch_s 设置；这里不能再按
            # manual_hold_s 重算，否则一被选中就把 1.5 s 缩短成 0.3 s，
            # 取消还没落地旧导航就恢复了。
            return manual
        if now < self._block_auto_until:
            return None                       # 刚松手：先停稳，别让导航立刻接手
        if self._nav_hold:
            return None                       # 旧目标还没确认结束，先别放行
        auto = None
        auto_stamp = -1.0
        for topic in self.input_topics:
            if topic in self.manual_topics:
                continue
            linear, angular, stamp = self._values[topic]
            if (abs(linear) > self.epsilon or abs(angular) > self.epsilon) \
                    and now - stamp <= self.hold_s and stamp > auto_stamp:
                auto, auto_stamp = topic, stamp
        return auto

    def _publish(self):
        """
        @description         : 按优先级发布一帧，来源切换时打日志
        @return              : None
        """
        now = time.monotonic()
        linear, angular = 0.0, 0.0
        with self._lock:
            owner = self._pick(now)
            self._owner = owner
            if owner is not None:
                linear, angular, _stamp = self._values[owner]

        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher.publish(twist)
        self._note(owner)

    def _note(self, owner):
        """
        @description         : 记录控制权变化，1 秒内不重复报同一状态
        @param owner         : 当前持有者话题名，None 表示无指令
        @return              : None
        """
        now = time.monotonic()
        changed = owner != self._reported
        # 心跳重发：App 订阅 /cmd_vel_source 判断"这一帧是导航还是人"，
        # 只在变化时发的话，App 重连后要等到下一次切换才知道来源。
        if not changed and now - self._reported_at < 1.0 / max(self.source_hz, 0.1):
            return
        if not changed and owner is None and now - self._reported_at < 1.0:
            return                              # 无来源时心跳放慢到 1 Hz，别刷屏
        self._reported = owner
        self._reported_at = now
        self._announce(owner)
        if changed:
            if owner is None:
                self.get_logger().info("速度来源: 无指令，输出停车")
            else:
                kind = "人工" if owner in self.manual_topics else "自动"
                self.get_logger().info("速度来源: %s（%s）" % (owner, kind))

    def _announce(self, owner):
        """发一帧来源。人工接管要抢在下一个发布周期之前发出去。"""
        with self._lock:
            self._reported = owner
            self._reported_at = time.monotonic()
        message = String()
        message.data = owner or ""
        self.source_publisher.publish(message)

    def destroy_node(self):
        """
        @description         : 退出前补一帧停车，避免下游停在最后一帧速度上
        @return              : None
        """
        if rclpy.ok():
            try:
                self.publisher.publish(Twist())
            except Exception:                # 发布者已失效时不必影响退出流程
                pass
        super().destroy_node()


def main():
    """
    @description         : 节点入口（rclpy 生命周期见 common/runtime.py）
    @return              : None
    """
    run(CmdVelMux)


if __name__ == "__main__":
    main()
