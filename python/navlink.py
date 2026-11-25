"""导航链路的 App 侧：一条订阅 + 一条发布。

为什么不是几条话题直连
    导航需要地图（transient_local 的大消息）、AMCL 位姿（四元数）、动作状态，
    这三样在容器里已被 nav_bridge 折成一条 JSON 流；网页又不能直连 rosbridge
    （9090 没有鉴权、也没映射）。所以这里只做一件事：把 JSON 原样转给页面，
    把页面的命令原样转回容器。

为什么命令要排队
    ros2_bridge 的 Ros2Publisher 只有一个待发槽（速度指令的语义：旧值没意义）。
    导航命令不能套用这个假设：页面先点"取消"、紧接着点地图（触发一次
    request_map），后到的 map 会把还没发出去的 cancel 覆盖掉 —— 页面显示已
    取消，车却照常开出去。这里自己维护一个有序队列，逐条发、按 sent 计数
    确认真的送出去了；只有"同一条命令重发"会被合并，不同命令绝不互相覆盖。

线程模型
    Ros2Subscriber / Ros2Publisher 各自带后台线程，本类不再起线程。
    snapshot()/pump() 只做短临界区，可在控制环里安全调用。
"""

import json
import threading
import time
from collections import deque

from ros2_bridge import Ros2Publisher, Ros2Subscriber

# 与容器内 common/topics.py 的 NAV_UI / NAV_CMD 对应（跨语言契约，改一处要同步）。
NAV_UI_TOPIC = "/app/nav_ui"
NAV_CMD_TOPIC = "/app/nav_cmd"

# 一条命令多久没被确认发出就重投。rosbridge 断线时 publish 不会报错，
# 只能靠 sent 计数判断。
RETRY_S = 1.0
# 命令有效期：超时就丢，不再发。
#
# 为什么必须有：断线时 goal 可能已经投进待发槽，排队中的 cancel 要等它先
# 发出去。重连后先把"几分钟前的目标"补发出去、再发取消，车会先动一下。
# 目标过期 5 s 之后就没必要再执行了 —— 用户当下要的是取消。
TTL_GOAL = 5.0
TTL_POSE = 6.0
TTL_CANCEL = 15.0     # 取消本身幂等，留久一点，断线重连还有意义
TTL_MODE = 20.0


class NavLink:
    """导航 JSON 流的收发封装。"""

    def __init__(self):
        self._sub = Ros2Subscriber(topic=NAV_UI_TOPIC,
                                   message_type="std_msgs/msg/String")
        self._pub = Ros2Publisher(topic=NAV_CMD_TOPIC,
                                  message_type="std_msgs/msg/String")
        self._seq = 0
        self._data = {}
        self._raw = ""
        self._at = 0.0
        self._last_parse_error = ""

        self._lock = threading.Lock()
        self._queue = deque()          # [(payload, expires_at)]，按顺序
        self._want_map = False         # 单独一个标志：不与命令抢位置
        self._inflight = None          # 已投递、等 sent_tag 确认的那一条
        self._inflight_at = 0.0
        self._inflight_expires = 0.0
        self._inflight_tag = 0
        self._tag_seq = 0
        self._expired = 0              # 因过期被丢弃的条数，诊断用

    def start(self):
        """启动收发线程，重复调用无副作用。"""
        self._sub.start()
        self._pub.start()

    def stop(self):
        """停止收发线程。"""
        self._sub.stop()
        self._pub.stop()

    # ---------------------------------------------------------------- 接收

    def snapshot(self):
        """返回 (已解析的帧, 帧龄秒, 帧序号)。

        帧序号变化才需要往页面推，页面据此去重；seq 从 0 开始，
        第一帧到达之前返回的空字典 age 为无穷大。
        """
        message, age, _ = self._sub.snapshot()
        raw = message.get("data") if isinstance(message, dict) else None
        if not isinstance(raw, str) or not raw:
            return self._data, float("inf"), self._seq
        # 解析只在内容变化时做一次：10 Hz、偶尔带 26 KB 的地图，
        # 每拍都重新 json.loads 是白烧 CPU。
        if raw != self._raw:
            self._raw = raw
            try:
                parsed = json.loads(raw)
            except ValueError:
                parsed = None
            if isinstance(parsed, dict):
                self._data = parsed
                self._seq += 1
                self._at = time.monotonic()
            elif raw != self._last_parse_error:
                self._last_parse_error = raw[:80]
                print("[nav] 导航帧不是合法 JSON：%r" % self._last_parse_error, flush=True)
        age = time.monotonic() - self._at if self._at else float("inf")
        return self._data, age, self._seq

    # ---------------------------------------------------------------- 发送

    def pump(self):
        """把队列里的命令按顺序发出；返回当前积压条数（诊断用）。

        由控制环每拍调用：一条命令确认发出后才发下一条，顺序与用户操作
        一致。发出的判定是 Ros2Publisher 的 sent 计数变化 —— 它只在
        WebSocket 真的把帧送出去之后才自增。超期的命令直接丢。
        """
        stats = self._pub.stats()
        now = time.monotonic()
        send = None
        with self._lock:
            if self._inflight is not None:
                # 必须对"这一条"确认：sent 是全局计数，上一条（比如旧目标）
                # 刚发完也会让它 +1，把它当成取消已发送会让待发槽里那条取消
                # 被随后的 map 请求覆盖掉。
                if stats.get("sent_tag") == self._inflight_tag:
                    self._inflight = None          # 这一条真的发出去了
                elif self._inflight_expires <= now:
                    self._inflight = None          # 太旧，不再重投
                    self._expired += 1
                elif now - self._inflight_at >= RETRY_S:
                    self._inflight_at = now
                    send = self._inflight
            if send is None and self._inflight is None:
                payload, expires = self._next_locked(now)
                if payload is not None:
                    self._tag_seq += 1
                    self._inflight = payload
                    self._inflight_expires = expires
                    self._inflight_tag = self._tag_seq
                    self._inflight_at = now
                    send = payload
            depth = self._depth_locked()
            expires = self._inflight_expires if send is not None else None
            tag = self._inflight_tag if send is not None else None
        if send is not None:
            self._publish(send, expires, tag)
        return depth

    def _next_locked(self, now):
        """取下一条待发命令，跳过已过期的。调用方必须持锁。"""
        while self._queue:
            payload, expires = self._queue.popleft()
            if expires <= now:
                self._expired += 1
                continue
            return payload, expires
        if self._want_map:
            self._want_map = False
            return {"cmd": "map"}, now + TTL_CANCEL
        return None, 0.0

    def _publish(self, payload, expires=None, tag=None):
        """投递一条命令；带剩余有效期与标识，底层据此过期丢弃、按条确认。"""
        if expires is not None:
            ttl = expires - time.monotonic()
            if ttl <= 0:
                # 从队列取出到投递之间刚好过期：不能再补 50 ms 余量，
                # 那等于给过期命令续命。
                return
        else:
            ttl = None
        self._pub.publish({"data": json.dumps(payload, separators=(",", ":"))},
                          ttl=ttl, tag=tag)

    def _depth(self):
        with self._lock:
            return self._depth_locked()

    def _depth_locked(self):
        return len(self._queue) + (1 if self._inflight is not None else 0)

    def _push(self, payload, ttl):
        with self._lock:
            self._queue.append((payload, time.monotonic() + ttl))

    # ---------------------------------------------------------------- 命令

    def goal(self, x, y, yaw):
        """下发导航目标（地图坐标，米/弧度）。连续两次只留最后一个。"""
        with self._lock:
            self._queue = deque(item for item in self._queue
                                if item[0].get("cmd") != "goal")
            self._queue.append(({"cmd": "goal", "x": float(x), "y": float(y),
                                 "yaw": float(yaw)},
                                time.monotonic() + TTL_GOAL))

    def cancel(self, reason="user"):
        """取消当前导航目标。reason 会带到容器里，用于页面提示。

        同时把还没发出去的 goal 丢掉：点了取消就不该再把它发出去。
        已经在队列里的 cancel 也一并合并，避免连点产生一串重复取消。
        """
        now = time.monotonic()
        with self._lock:
            self._queue = deque(item for item in self._queue
                                if item[0].get("cmd") not in ("goal", "cancel"))
            # 已经投出去但没确认的 goal 也别再重投：用户此刻要的是取消。
            # 万一它其实已经到了容器，nav_bridge 会收到紧随其后的 cancel。
            if self._inflight is not None and self._inflight.get("cmd") == "goal":
                self._inflight = None
            self._queue.append(({"cmd": "cancel", "reason": str(reason)},
                                now + TTL_CANCEL))

    def set_pose(self, x, y, yaw, req=0):
        """重设初始位姿，保留页面请求标识；旧调用默认标识为 0。"""
        self._push({"cmd": "pose", "x": float(x), "y": float(y),
                    "yaw": float(yaw), "req": req}, TTL_POSE)

    def set_mode(self, mode):
        """切换导航/建图模式（由容器内的 mode_manager 执行）。"""
        self._push({"cmd": "mode", "mode": str(mode)}, TTL_MODE)

    def request_map(self):
        """要求容器内立刻重发一次地图（页面刚连上时用）。

        走独立标志而不是队列：地图请求永远可以等，绝不能挤掉取消命令。
        """
        with self._lock:
            self._want_map = True


NAV = NavLink()
