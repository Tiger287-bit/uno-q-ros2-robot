#!/usr/bin/env python3
"""游戏杆遥控节点：/dev/input/js* -> /cmd_vel_joy（同时发布 /joy）。

为什么不用官方 joy_node + teleop_twist_joy：
    1. Jazzy 的 joy 3.3.0 基于 SDL2，需要 /dev/input/event* 与 udev，而 Brick
       容器只映射了 /dev/input 下的 js*；
    2. teleop_twist_joy 在本手柄上实测线速度恒为 0（负向轴 + 负 scale 组合行为
       不确定）。
    自建节点直接读内核 joystick API，映射规则显式可见，便于量产维护。

发布：
    /cmd_vel_joy  geometry_msgs/Twist，仅在按住使能键时发布；松杆或掉线后补发
                  一帧 0 立即停车，随后保持静默 —— 空闲时不用 0 覆盖网页指令，
                  两路统一交给 cmd_vel_mux 仲裁。
    /joy          sensor_msgs/Joy，原始轴与按钮，便于调试和上层复用。

映射默认值（本车配套亚博手柄，用户逐项确认）：
    左摇杆 上下 = 轴 1，右摇杆 左右 = 轴 2；推上/推左为负值，故 invert_* 默认 true；
    使能键 = 键 7（右上肩键），加速键 = 键 8（R1）。
    以上全部是 ROS 2 参数，改 config/teleop.yaml 即可适配其他手柄，不必改代码。
"""

import fcntl
import os
import select
import struct
import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy

from arduino_ros2_bringup.common.runtime import run
from arduino_ros2_bringup.common.topics import Topics

# js_event 结构：uint32 时间戳、int16 值、uint8 类型、uint8 编号。
JS_EVENT = struct.Struct("IhBB")
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

# ioctl 查询设备的轴数与按钮数。
JSIOCGAXES = 0x80016A11
JSIOCGBUTTONS = 0x80016A12

# 设备报不出长度时的兜底尺寸。
DEFAULT_AXES = 8
DEFAULT_BUTTONS = 15


class JsJoyNode(Node):
    """读取游戏杆并发布 /joy 与 /cmd_vel_joy。"""

    def __init__(self):
        """
        @description         : 读取参数、创建发布者并启动设备读取线程
        @return              : None
        """
        super().__init__("js_joy_node")
        self._read_parameters()

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._fd = -1
        self._axes = [0.0] * self._axis_count
        self._buttons = [0] * self._button_count
        self._commanded = False      # 上一帧是否发过非零指令

        self.cmd_publisher = self.create_publisher(Twist, self.cmd_topic, 10)
        self.joy_publisher = self.create_publisher(Joy, self.joy_topic, 10)
        self.create_timer(1.0 / self.rate, self._publish)
        threading.Thread(target=self._read_loop, name="js-reader", daemon=True).start()

        self.get_logger().info(
            f"游戏杆节点启动: {self.device} @ {self.rate:.0f} Hz -> {self.cmd_topic}, "
            f"使能键 {self.button_enable}, 加速键 {self.button_turbo}, "
            f"轴 linear={self.axis_linear} angular={self.axis_angular}")

    def _read_parameters(self):
        """
        @description         : 集中声明并读取参数，保证与 config/teleop.yaml 一一对应
        @return              : None
        """
        self.device = str(self.declare_parameter("device", "/dev/input/js0").value)
        self.rate = max(1.0, float(self.declare_parameter("rate", 50.0).value))

        # 话题名默认取 common/topics.py 的契约值，允许参数覆盖（多车同图联调时用）。
        self.cmd_topic = str(
            self.declare_parameter("cmd_topic", Topics.CMD_VEL_JOY).value)
        self.joy_topic = str(
            self.declare_parameter("joy_topic", Topics.JOY).value)

        # 控制映射。
        self.axis_linear = int(self.declare_parameter("axis_linear", 1).value)
        self.axis_angular = int(self.declare_parameter("axis_angular", 2).value)
        self.button_enable = int(self.declare_parameter("button_enable", 7).value)
        self.button_turbo = int(self.declare_parameter("button_turbo", 8).value)
        self.scale_linear = float(self.declare_parameter("scale_linear", 0.075).value)
        self.scale_angular = float(self.declare_parameter("scale_angular", 1.20).value)
        self.turbo_factor = float(self.declare_parameter("turbo_factor", 1.30).value)
        self.deadband = float(self.declare_parameter("deadband", 0.05).value)
        self.invert_linear = bool(self.declare_parameter("invert_linear", True).value)
        self.invert_angular = bool(self.declare_parameter("invert_angular", True).value)

        # 数组长度至少覆盖配置用到的轴/键，避免 _publish 里的越界判断永假。
        self._axis_count = max(DEFAULT_AXES, self.axis_linear + 1, self.axis_angular + 1)
        self._button_count = max(DEFAULT_BUTTONS,
                                 self.button_enable + 1, self.button_turbo + 1)

    def _open(self):
        """
        @description         : 打开设备并读取轴/按钮数量
        @return              : 打开成功时返回 True
        """
        try:
            fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as exc:
            # 每秒重试一次，但日志限流：没插手柄是允许的常态，不该刷屏。
            self.get_logger().warning(
                f"打开 {self.device} 失败: {exc}（未插手柄时可忽略，30 秒内不重复）",
                throttle_duration_sec=30)
            return False

        axes, buttons = DEFAULT_AXES, DEFAULT_BUTTONS
        try:
            axes = fcntl.ioctl(fd, JSIOCGAXES, b"\x00")[0] or DEFAULT_AXES
            buttons = fcntl.ioctl(fd, JSIOCGBUTTONS, b"\x00")[0] or DEFAULT_BUTTONS
        except OSError:
            pass  # 少数设备不支持 ioctl，退回默认长度。

        with self._lock:
            self._fd = fd
            self._axes = [0.0] * max(axes, self._axis_count)
            self._buttons = [0] * max(buttons, self._button_count)
        self.get_logger().info(f"已连接游戏杆，轴 {axes} 个，按钮 {buttons} 个")
        return True

    def _close(self):
        """
        @description         : 关闭设备并复位状态，避免掉线后继续输出最后一帧
        @return              : None
        """
        with self._lock:
            if self._fd >= 0:
                try:
                    os.close(self._fd)
                except OSError:
                    pass
                self._fd = -1
            self._axes = [0.0] * len(self._axes)
            self._buttons = [0] * len(self._buttons)
            self._commanded = True       # 掉线时补发一帧 0，避免停在旧速度

    def _read_loop(self):
        """
        @description         : 读取线程主体，把内核事件写入共享状态
        @return              : None
        """
        while not self._stop.is_set():
            if self._fd < 0:
                if not self._open():
                    self._stop.wait(1.0)
                    continue

            try:
                ready, _, _ = select.select([self._fd], [], [], 0.5)
            except (OSError, ValueError):
                self._close()
                continue
            if not ready:
                continue

            try:
                data = os.read(self._fd, JS_EVENT.size)
            except (BlockingIOError, InterruptedError):
                continue
            except OSError as exc:
                self.get_logger().warning(f"读取游戏杆失败: {exc}")
                self._close()
                continue
            if len(data) < JS_EVENT.size:
                continue

            _, value, kind, number = JS_EVENT.unpack(data)
            kind &= ~JS_EVENT_INIT
            with self._lock:
                if kind == JS_EVENT_AXIS and number < len(self._axes):
                    self._axes[number] = max(-1.0, min(1.0, value / 32767.0))
                elif kind == JS_EVENT_BUTTON and number < len(self._buttons):
                    self._buttons[number] = 1 if value else 0

    def _publish(self):
        """
        @description         : 按固定频率发布原始 /joy 与解析后的 /cmd_vel_joy
        @return              : None
        """
        with self._lock:
            axes = list(self._axes)
            buttons = list(self._buttons)

        joy = Joy()
        joy.header.stamp = self.get_clock().now().to_msg()
        joy.header.frame_id = "gamepad"
        joy.axes = axes
        joy.buttons = buttons
        self.joy_publisher.publish(joy)

        twist = Twist()
        enabled = (len(buttons) > self.button_enable
                   and buttons[self.button_enable] == 1)
        if enabled:
            linear = self._shape(axes[self.axis_linear] if len(axes) > self.axis_linear else 0.0)
            angular = self._shape(axes[self.axis_angular] if len(axes) > self.axis_angular else 0.0)
            turbo = len(buttons) > self.button_turbo and buttons[self.button_turbo] == 1
            factor = self.turbo_factor if turbo else 1.0
            twist.linear.x = (-linear if self.invert_linear else linear) * self.scale_linear * factor
            twist.angular.z = (-angular if self.invert_angular else angular) * self.scale_angular * factor
            self._commanded = True
        elif self._commanded:
            # 松杆/掉线后补发一帧 0 立即停车，之后静默，停车状态由 cmd_vel_mux 维持。
            self._commanded = False
        else:
            return                       # 未操作且已停车，不占用话题

        self.cmd_publisher.publish(twist)

    def _shape(self, value):
        """
        @description         : 应用死区并把剩余行程重新映射到 0~1
        @param value         : 归一化后的轴值，-1~1
        @return              : 处理后的轴值
        """
        if abs(value) <= self.deadband:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadband) / (1.0 - self.deadband)

    def destroy_node(self):
        """
        @description         : 停止读取线程、释放设备并补一帧停车
        @return              : None
        """
        self._stop.set()
        self._close()
        if rclpy.ok():
            # 单独运行本节点（不经 mux）时，退出前把速度归零。
            try:
                self.cmd_publisher.publish(Twist())
            except Exception:            # 发布者已失效时不必影响退出流程
                pass
        super().destroy_node()


def main():
    """
    @description         : 节点入口（rclpy 生命周期见 common/runtime.py）
    @return              : None
    """
    run(JsJoyNode)


if __name__ == "__main__":
    main()
