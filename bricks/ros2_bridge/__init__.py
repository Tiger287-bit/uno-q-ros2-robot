"""ROS 2 Bridge Brick 的 Python 侧接口。

设计说明：
    brick_compose.yaml 启动的 ROS 2 容器与本文件所在的 App 容器是两个独立容器，
    二者不能直接函数调用，只能通过 rosbridge 的 WebSocket 交换消息。

    这里提供两个方向的接口：
      Ros2Subscriber —— "只保留最新一条"的订阅器，把 ROS 2 话题降频为 App
                        控制循环可直接读取的共享状态（/cmd_vel、/joy）。
      Ros2Publisher  —— 单向发布器，把 App 侧产生的指令送进 ROS 2 图
                        （网页键盘/虚拟摇杆 -> /cmd_vel_web）。

    两个类各自维持一条 WebSocket，互不干扰；rosbridge 对连接数没有硬限制。
"""

import json
import os
import queue
import threading
import time

from websockets.exceptions import ConnectionClosed
from websockets.sync.client import connect

CMD_VEL_TOPIC = "/cmd_vel"                       # 遥控速度指令话题。
WEB_CMD_TOPIC = "/cmd_vel_web"                   # 网页键盘/摇杆指令话题。
DEFAULT_URL = "ws://ros2:9090"                   # ros2 为 Compose 服务名。
RECONNECT_DELAY_S = 1.0                          # 断线后的重连间隔，s。
RECV_TIMEOUT_S = 0.5                             # 单次接收超时，便于响应停止请求。
OPEN_TIMEOUT_S = 3.0                             # 建链超时，s。


class Ros2Subscriber:
    """订阅任意 ROS 2 话题并缓存最新一条消息，断线后自动重连。

    本类同时用于 /cmd_vel（速度指令）与 /joy（原始手柄状态），
    因此只保存原始消息字典，由调用方按字段解释。
    """

    def __init__(self, url=None, topic=CMD_VEL_TOPIC, message_type="geometry_msgs/msg/Twist"):
        """
        @description         : 创建订阅器，不立即连接
        @param url           : rosbridge 地址，缺省读取 ROSBRIDGE_URL 环境变量
        @param topic         : 要订阅的 ROS 2 话题名
        @param message_type  : 话题的消息类型，rosbridge 订阅时必填
        @return              : None
        """
        self.url = url or os.getenv("ROSBRIDGE_URL", DEFAULT_URL)
        self.topic = topic
        self.message_type = message_type
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._connected = threading.Event()
        self._thread = None
        self._message = {}
        self._stamp = 0.0
        self._messages = 0
        self._error = ""

    def start(self):
        """
        @description         : 启动后台订阅线程，重复调用无副作用
        @return              : None
        """
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="cmd_vel", daemon=True)
        self._thread.start()

    def stop(self):
        """
        @description         : 请求订阅线程退出并等待其结束
        @return              : None
        """
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def snapshot(self):
        """
        @description         : 读取最新消息及其新鲜度
        @return              : (消息字典, 距上次消息的秒数, 是否已连接)
        """
        with self._lock:
            age = time.monotonic() - self._stamp if self._stamp else float("inf")
            return self._message, age, self._connected.is_set()

    def stats(self):
        """
        @description         : 返回链路诊断信息，供网页展示
        @return              : 包含地址、连接状态、消息计数和最近错误的字典
        """
        with self._lock:
            age = time.monotonic() - self._stamp if self._stamp else None
            return {
                "url": self.url,
                "topic": self.topic,
                "connected": self._connected.is_set(),
                "messages": self._messages,
                "age_s": None if age is None else round(age, 3),
                "error": self._error,
            }

    def _run(self):
        """
        @description         : 订阅线程主体，负责建链、收消息和断线重连
        @return              : None
        """
        while not self._stop.is_set():
            try:
                with connect(self.url, proxy=None, open_timeout=OPEN_TIMEOUT_S,
                             close_timeout=1.0) as ws:
                    ws.send(json.dumps({
                        "op": "subscribe",
                        "topic": self.topic,
                        "type": self.message_type,
                        "queue_length": 1,
                    }))
                    self._connected.set()
                    with self._lock:
                        self._error = ""
                    print(f"[ROS 2] 已订阅 {self.topic}", flush=True)

                    while not self._stop.is_set():
                        try:
                            raw = ws.recv(timeout=RECV_TIMEOUT_S)
                        except TimeoutError:
                            continue
                        self._handle(raw)

            except (OSError, TimeoutError, ConnectionClosed, ValueError) as exc:
                with self._lock:
                    self._error = str(exc)
                if not self._stop.is_set():
                    print(f"[ROS 2] 连接中断: {exc}", flush=True)
            finally:
                self._connected.clear()

            if not self._stop.is_set():
                time.sleep(RECONNECT_DELAY_S)

    def _handle(self, raw):
        """
        @description         : 解析一条 rosbridge 报文并更新缓存
        @param raw           : WebSocket 收到的原始文本
        @return              : None
        """
        try:
            packet = json.loads(raw)
        except ValueError:
            # 断线重连瞬间可能收到截断的报文。丢这一帧就够了 —— 若让它冒到 _run 的
            # except，整条连接会被关掉并空等 RECONNECT_DELAY_S，代价远大于一帧。
            return
        # 合法 JSON 也可能是数组或标量，.get 会抛 AttributeError 把订阅线程打死
        if not isinstance(packet, dict):
            return
        if packet.get("op") != "publish" or packet.get("topic") != self.topic:
            return
        message = packet.get("msg")
        if not isinstance(message, dict):
            return
        with self._lock:
            self._message = message
            self._stamp = time.monotonic()
            self._messages += 1


SEND_POLL_S = 0.2                                # 发布线程的唤醒周期，s。


class Ros2Publisher:
    """向 ROS 2 话题发布消息，断线后自动重连。

    与 Ros2Subscriber 相反，待发槽深度恒为 1：速度指令是连续流，旧值对
    下游没有意义，积压只会让车延迟响应。新帧直接覆盖尚未发出的旧帧，
    被覆盖的帧计入 dropped 供诊断（每次成功发送即清零，是"当前积压"而非
    "历史累计"，否则一旦出错就永久告警）。
    """

    def __init__(self, url=None, topic=WEB_CMD_TOPIC,
                 message_type="geometry_msgs/msg/Twist"):
        """
        @description         : 创建发布器，不立即连接
        @param url           : rosbridge 地址，缺省读取 ROSBRIDGE_URL 环境变量
        @param topic         : 目标话题名
        @param message_type  : 话题的消息类型，advertise 时必填
        @return              : None
        """
        self.url = url or os.getenv("ROSBRIDGE_URL", DEFAULT_URL)
        self.topic = topic
        self.message_type = message_type
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._connected = threading.Event()
        self._thread = None
        self._pending = None
        self._pending_expires = 0.0      # 0 表示不过期
        self._pending_tag = None         # 调用方给这一帧的标识
        self._sent_tag = None            # 最近一次真正发出去的帧的标识
        self._sent = 0
        self._dropped = 0
        self._error = ""

    def start(self):
        """
        @description         : 启动后台发布线程，重复调用无副作用
        @return              : None
        """
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="ros2_pub", daemon=True)
        self._thread.start()

    def stop(self):
        """
        @description         : 请求发布线程退出并等待其结束
        @return              : None

        不必在退出时补发停车帧：下游 cmd_vel_mux 有 HOLD_S 静默超时，
        本链路停止后 0.2 s 内即自行归零。
        """
        self._stop.set()
        self._wake.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def publish(self, message, ttl=None, tag=None):
        """
        @description         : 投递一帧，非阻塞，供网页 socket 回调直接调用
        @param message       : 消息字典，结构须与该话题的消息类型一致
        @param ttl           : 可选有效期（秒）。过期就不发，断线重连后也不会补发
        @param tag           : 可选标识。发送成功后原样出现在 stats()["sent_tag"]，
                               调用方据此确认"**这一条**"真的发出去了
        @return              : None

        为什么需要 ttl：待发槽在断线时会一直留着最后一帧，重连后立刻发出去。
        速度指令这样没问题（旧值被下一帧覆盖），但导航命令不行 —— 几分钟前的
        目标在重连后补发，就是车自己跑出去。上层队列有有效期还不够，那一帧可能
        已经躺在这个槽里了。
        """
        with self._lock:
            if self._pending is not None:
                self._dropped += 1
            self._pending = dict(message)
            self._pending_expires = (time.monotonic() + ttl) if ttl else 0.0
            self._pending_tag = tag
        self._wake.set()

    def stats(self):
        """
        @description         : 返回链路诊断信息，供网页展示
        @return              : 包含地址、连接状态、累计发出帧数与当前被覆盖帧数的字典

        dropped 为"尚未发出的帧数"：每次成功发送归零，只有持续积压（发帧线程
        跟不上输入）才会一直大于 0，页面据此判断是否告警。
        """
        with self._lock:
            return {
                "url": self.url,
                "topic": self.topic,
                "connected": self._connected.is_set(),
                "sent": self._sent,
                "sent_tag": self._sent_tag,
                "dropped": self._dropped,
                "error": self._error,
            }

    def _run(self):
        """
        @description         : 发布线程主体，负责建链、advertise、发帧与重连
        @return              : None
        """
        while not self._stop.is_set():
            try:
                with connect(self.url, proxy=None, open_timeout=OPEN_TIMEOUT_S,
                             close_timeout=1.0) as ws:
                    ws.send(json.dumps({
                        "op": "advertise",
                        "topic": self.topic,
                        "type": self.message_type,
                    }))
                    self._connected.set()
                    with self._lock:
                        self._error = ""
                    print(f"[ROS 2] 已发布 {self.topic}", flush=True)

                    while not self._stop.is_set():
                        self._wake.wait(SEND_POLL_S)
                        self._wake.clear()
                        with self._lock:
                            message = self._pending
                            expires = self._pending_expires
                            tag = self._pending_tag
                            self._pending = None
                            self._pending_expires = 0.0
                            self._pending_tag = None
                        if message is None:
                            continue
                        if expires and expires <= time.monotonic():
                            # 断线期间攒下来的过期帧：直接丢，重连不补发
                            with self._lock:
                                self._dropped += 1
                            continue
                        try:
                            ws.send(json.dumps({
                                "op": "publish",
                                "topic": self.topic,
                                "msg": message,
                            }))
                        except (OSError, TimeoutError, ConnectionClosed) as exc:
                            # 发不出去就放回待发槽，重连后再发，避免丢掉最后一帧
                            with self._lock:
                                if self._pending is None and not (
                                        expires and expires <= time.monotonic()):
                                    self._pending = message
                                    self._pending_expires = expires
                                    self._pending_tag = tag
                            raise exc
                        with self._lock:
                            self._sent += 1
                            self._sent_tag = tag
                            # 发出去就把积压计数清零：dropped 表示"当前还没发出去的
                            # 帧数"，恢复正常即为 0。若改成累计计数，网页徽章会因为
                            # 偶发一次覆盖而永久停在告警色，失去意义。
                            self._dropped = 0

            except (OSError, TimeoutError, ConnectionClosed, ValueError) as exc:
                with self._lock:
                    self._error = str(exc)
                if not self._stop.is_set():
                    print(f"[ROS 2] 发布链路中断: {exc}", flush=True)
            finally:
                self._connected.clear()

            if not self._stop.is_set():
                time.sleep(RECONNECT_DELAY_S)
