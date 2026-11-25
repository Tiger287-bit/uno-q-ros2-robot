"""内存遥测环形缓冲，不写磁盘。"""

import threading
import time
from collections import deque
from itertools import islice

# 末两列：session_id 跨进程唯一，seq 进程内单调。
HEADER = ["time", "cmd_linear", "cmd_angular",
          "m1_target", "m1_speed", "m1_pwm",
          "m2_target", "m2_speed", "m2_pwm",
          "yaw", "rate", "vbat", "mode", "reason",
          "session_id", "seq"]

SEQ_COL = HEADER.index("seq")
RATE_WINDOW = 60          # 参与计算的帧数上限，5 Hz 下约 12 秒
# 只统计最近这么长时间内到达的帧。按"帧数窗口"算会在采集停掉后永远保持旧值：
# 没有新帧进来，最后 60 帧的跨度就冻结不变，速率仍是 5 Hz —— 徽章看着健康，
# 实际已经停更几分钟。所以既限帧数也限时间跨度。
RATE_SPAN_S = 3.0


def _opt(value, digits):
    """缺失值保持 None，不伪造成 0。"""
    return None if value is None else round(value, digits)


class Telemetry:
    """按固定频率缓存最近若干帧。"""

    def __init__(self, capacity=12000):
        """capacity 为保留帧数，5 Hz 下 12000 约 40 分钟。"""
        if capacity <= 0:
            raise ValueError(f"capacity 必须为正，当前 {capacity}")
        self.samples = deque(maxlen=capacity)
        self._lock = threading.Lock()
        # 微秒级会话标识，跨进程唯一且不超出 JS 安全整数
        self._session_id = time.time_ns() // 1000
        self._seq = 0
        self._wall0 = time.time()
        self._mono0 = time.monotonic()
        self._last_stamp = 0.0
        self._last_mono = 0.0

    def _stamp(self):
        """以启动墙钟为锚点的单调时间戳，秒。"""
        return self._wall0 + (time.monotonic() - self._mono0)

    def record(self, status, linear, angular):
        """记录一帧，字段缺失时保持 None。"""
        if status is None:
            return
        wheels = status.get("wheels") or []
        if len(wheels) < 2:
            return
        imu = status.get("imu") or {}
        battery = status.get("battery") or {}
        w1, w2 = wheels[0], wheels[1]
        now = self._stamp()
        with self._lock:
            self._seq += 1
            row = [round(now, 2), round(linear, 4), round(angular, 4),
                   w1["target"], round(w1["speed"], 1), w1["pwm"],
                   w2["target"], round(w2["speed"], 1), w2["pwm"],
                   _opt(imu.get("yaw_deg"), 2),
                   _opt(imu.get("rate_dps"), 2),
                   battery.get("voltage"),
                   status.get("mode"), status.get("reason"),
                   self._session_id, self._seq]
            self.samples.append(row)
            self._last_stamp = now
            self._last_mono = time.monotonic()

    def recent(self, limit=600):
        """返回最近 limit 帧，首行表头；行对象与缓冲共享，调用方只读。"""
        if limit <= 0:
            return [list(HEADER)]
        with self._lock:
            tail = list(islice(reversed(self.samples), 0, limit))
        tail.reverse()
        return [list(HEADER)] + tail

    def rate_hz(self):
        """最近 RATE_SPAN_S 秒内的实测采样率；样本不足返回 None，停更返回 0.0。

        三个边界：
          - 采集停掉（最新一帧都比窗口还老）→ 0.0。不能沿用"最后 60 帧的平均"，
            那样断流后读数会一直停在 5 Hz，等于把失联报成健康。
          - 帧还在来但变稀（例如 5 Hz 掉到 1 Hz）→ 立刻反映成 1 Hz，不必等
            60 帧（1 Hz 下要等一分钟）。
          - 起动头两帧 → 按这两帧的间隔算，不返回 None，避免徽章先显示标称值。
        """
        with self._lock:
            tail = list(islice(reversed(self.samples), 0, RATE_WINDOW))
            idle = time.monotonic() - self._last_mono if self._seq else 0.0
        if len(tail) < 2:
            return None
        if idle > RATE_SPAN_S:
            return 0.0
        newest = tail[0][0]
        recent = [row for row in tail if row[0] >= newest - RATE_SPAN_S]
        if len(recent) < 2:                      # 两帧间隔超过窗口，就按这两帧算
            recent = tail[:2]
        span = recent[0][0] - recent[-1][0]
        return round((len(recent) - 1) / span, 2) if span > 0 else None

    def since(self, seq, limit=600):
        """返回 (首行表头 + 新帧, truncated)，供增量拉取。

        seq 超出当前序号说明来自上一会话（进程重启后序号归零），
        此时按跨会话处理，直接回退到最近 limit 帧。

        truncated 表示"游标早于缓冲区最旧一帧"，即中间那段已经被挤掉。
        不报这个标志的话，客户端拿到的是一段看起来连续、实际缺口的序列：
        last_seq 会直接跳到最新，导出工具无从发现漏采。
        """
        with self._lock:
            oldest = self._seq - len(self.samples) + 1   # 缓冲区里最旧一帧的序号
            truncated = bool(self.samples) and seq < oldest - 1
            if seq > self._seq:
                rows = list(islice(reversed(self.samples), 0, limit))
            else:
                rows = []
                for row in reversed(self.samples):
                    if row[SEQ_COL] <= seq or len(rows) >= limit:
                        break
                    rows.append(row)
            # 截断到 limit 时同样丢了帧（中间那段留不住），一并报出来
            if len(rows) >= limit and rows and rows[-1][SEQ_COL] > seq + 1:
                truncated = True
        rows.reverse()
        return [list(HEADER)] + rows, truncated

    def status(self):
        """返回缓冲状态与采集新鲜度。"""
        with self._lock:
            fresh = time.monotonic() - self._last_mono if self._seq else None
            return {"frames": len(self.samples),
                    "capacity": self.samples.maxlen,
                    "started_at": round(self._wall0, 3),
                    "last_time": round(self._last_stamp, 3) if self._seq else None,
                    "session_id": self._session_id,
                    "last_seq": self._seq,
                    "age_s": round(fresh, 2) if fresh is not None else None}

    def close(self):
        """无资源需释放，保留接口。"""
