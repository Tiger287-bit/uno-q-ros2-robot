"""链路延迟与调度探针：只统计，不参与控制。

首轮基线要回答的是"延迟到底花在哪一段"，所以这里分开记四类量：

- ``loop``     控制环两拍之间的间隔（名义 1/30 s）
- ``work``     一拍里实际干活的时间（不含等待）
- ``drive`` / ``status`` / ``stop``  Bridge 单次调用往返耗时
- ``cmd_age``  指令帧年龄，``yaw_age`` 航向状态帧年龄，``imu_age`` MCU 侧 IMU 样本年龄

所有量单位都是毫秒。统计量按滑动窗口算 p50/p95/max：窗口上限 LATENCY_CAPACITY 个样本，
报告周期由 config.LATENCY_REPORT_S 决定。

设计约束：``add``/``call`` 必须足够便宜（控制环 30 Hz，每拍最多 5 次调用）。
这里只做 deque.append 与一次减法，排序只发生在报告时。
"""

import math
import time
from collections import deque


class Series:
    """单指标滑动窗口。"""

    def __init__(self, capacity):
        self.values = deque(maxlen=capacity)

    def add(self, value):
        if value is None:
            return
        try:
            value = float(value)
        except (TypeError, ValueError):
            return
        if math.isfinite(value):
            self.values.append(value)

    def stats(self):
        """返回 (n, p50, p95, max)；无样本时返回 None。"""
        if not self.values:
            return None
        ordered = sorted(self.values)
        n = len(ordered)
        p50 = ordered[min(n - 1, int(0.50 * (n - 1)))]
        p95 = ordered[min(n - 1, int(math.ceil(0.95 * (n - 1))))]
        return n, p50, p95, ordered[-1]


class Probe:
    """一组指标 + 定时汇总成一行日志。"""

    def __init__(self, capacity=600, report_s=5.0):
        self.capacity = max(1, int(capacity))
        self.report_s = float(report_s)
        self.series = {}
        self.last_report = time.monotonic()

    def add(self, name, value):
        series = self.series.get(name)
        if series is None:
            series = self.series[name] = Series(self.capacity)
        series.add(value)

    def call(self, name, func, *args, **kwargs):
        """计时执行一次 Bridge 调用，异常照常向上抛。"""
        started = time.monotonic()
        try:
            return func(*args, **kwargs)
        finally:
            self.add(name, (time.monotonic() - started) * 1000.0)

    def due(self):
        return time.monotonic() - self.last_report >= self.report_s

    def report(self, extra=""):
        """返回一行可读日志，并把窗口清零重新计时。"""
        self.last_report = time.monotonic()
        known = ("loop", "work", "drive", "status", "stop",
                 "cmd_age", "yaw_age", "imu_age")
        order = list(known) + [name for name in self.series if name not in known]
        parts = []
        for name in order:
            series = self.series.get(name)
            stats = series.stats() if series else None
            if not stats:
                continue
            n, p50, p95, peak = stats
            parts.append(f"{name}={p50:.1f}/{p95:.1f}/{peak:.1f}(n{n})")
        for series in self.series.values():
            series.values.clear()
        suffix = f" {extra}" if extra else ""
        return "[lat] " + " ".join(parts) + suffix


class Timer:
    """上下文管理器形式，避免在 try/finally 里重复写时间戳。"""

    def __init__(self, probe, name):
        self.probe = probe
        self.name = name
        self.started = 0.0

    def __enter__(self):
        self.started = time.monotonic()
        return self

    def __exit__(self, *exc):
        self.probe.add(self.name, (time.monotonic() - self.started) * 1000.0)
        return False
