"""100 Hz MCU trace 采集：分页拉取、CSV 落盘、丢帧统计。

为什么不在控制环里一次性拉完：一页 16 条，2048 条要 128 次 Bridge 往返，按 5 ms/次
算约 0.6 s 的连续阻塞，会把 30 Hz 控制环卡住。这里用协作式状态机：控制环在停车后
每拍拉一页，4 s 左右拉完，单拍只多约 5 ms。

丢帧有两种来源，分开统计：
- 环形覆盖：MCU 侧 ``dropped`` 字段，样本在导出前就被挤掉了；
- 序号跳变：相邻样本 ``seq`` 不连续，说明 MCU 重启或整页丢失。
"""

import csv
import os
import tempfile
import time

TRACE_FIELDS = 16
TRACE_META = 8
ENV_FIELDS = 4
ENV_META = 4
PAGE_ROWS = 16
# 单次导出只取最近这么多行（10 ms 一拍时 600 行≈6 s）。0 表示整圈。
MAX_DUMP_ROWS = 600

# 列序必须与固件 rosTracePoll 的回包顺序逐项一致：它按"每拍先把两轮同名
# 字段并排"展开（target[0], target[1], ramp[0], ramp[1] …），不是按轮分块。
# 原来表头按轮分组写（m1_target, m1_ramp …）而数据按字段交错，值本身没错，
# 但列名整体错位：m1_pwm 那一列其实是 m1_ramp，m1_speed 那一列是 m2_target。
# 拿这种 CSV 做"pwm 明明 188 为什么还在加速"的分析会得出完全相反的结论。
TRACE_HEADER = ["tick_ms", "seq", "dt_ms", "flags",
                "m1_target", "m2_target", "m1_ramp", "m2_ramp",
                "m1_speed", "m2_speed", "m1_pwm", "m2_pwm",
                "m1_ff", "m2_ff", "rate_dps", "vbat_raw"]
ENV_HEADER = ["tick_ms", "temp_c", "bias_dps", "state", "seq"]


class TraceStore:
    """一次采集的样本与元数据。"""

    def __init__(self):
        self.rows = []
        self.env = []
        self.epoch = None       # None 表示尚未读到任何一页，不能拿 0 当"已初始化"
        self.total = 0
        self.dropped = 0
        self.capacity = 0
        self.bytes_per_sample = 0
        self.period_ms = 0
        self.seq_gaps = 0
        self.error = ""
        self.fetched_at = 0.0

    def _count_gaps(self):
        previous = None
        for row in self.rows:
            seq = int(row[1]) & 0xFFFF
            if previous is not None:
                if ((seq - previous) & 0xFFFF) != 1:
                    self.seq_gaps += 1
            previous = seq

    def duration_s(self):
        if len(self.rows) < 2:
            return 0.0
        return (int(self.rows[-1][0]) - int(self.rows[0][0])) / 1000.0

    def rate_hz(self):
        span = self.duration_s()
        return round((len(self.rows) - 1) / span, 2) if span > 0 else 0.0

    def summary(self):
        return {"epoch": self.epoch, "rows": len(self.rows), "env_rows": len(self.env),
                "duration_s": round(self.duration_s(), 2), "rate_hz": self.rate_hz(),
                "dropped": self.dropped, "seq_gaps": self.seq_gaps,
                "capacity": self.capacity, "bytes_per_sample": self.bytes_per_sample,
                "error": self.error}

    def save_trace_csv(self, path):
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(TRACE_HEADER)
            for row in self.rows:
                writer.writerow([row[0], row[1], row[2], row[3],
                                 row[4], row[5], row[6], row[7], row[8],
                                 row[9], row[10], row[11], row[12], row[13],
                                 round(int(row[14]) / 100.0, 2), row[15]])
        return path

    def save_env_csv(self, path):
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(ENV_HEADER)
            for row in self.env:
                writer.writerow([row[0], round(row[1] / 100.0, 2),
                                 round(row[2] / 1000.0, 3), row[3], row[4]])
        return path

    # ---------- 分页原语 ----------

    def begin_page(self, raw):
        """解析一页 trace 的元数据；返回 (count, total, 是否会话号冲突)。"""
        if not raw or len(raw) < TRACE_META:
            raise ValueError("ros_trace_poll 返回空或元数据不足")
        epoch, total, dropped = int(raw[0]), int(raw[1]), int(raw[2])
        conflict = self.epoch is not None and epoch != self.epoch
        if self.epoch is None:
            self.epoch = epoch
            self.total, self.dropped = total, dropped
            self.capacity, self.bytes_per_sample = int(raw[3]), int(raw[4])
            self.period_ms = int(raw[5])
        return int(raw[6]), total, conflict

    def append_page(self, raw, count):
        body = raw[TRACE_META:]
        if count < 0 or len(body) < count * TRACE_FIELDS:
            raise ValueError("样本字段长度不足")
        for i in range(count):
            base = i * TRACE_FIELDS
            self.rows.append(body[base:base + TRACE_FIELDS])

    def load_env(self, raw_list):
        """把若干页 ros_env_poll 的回包拼成行。"""
        for raw in raw_list:
            if not raw or len(raw) < ENV_META:
                continue
            body = raw[ENV_META:]
            for i in range(len(body) // ENV_FIELDS):
                base = i * ENV_FIELDS
                packed = int(body[base + 3])
                self.env.append([body[base], body[base + 1], body[base + 2],
                                 (packed >> 16) & 0xFF, packed & 0xFFFF])


class TraceDumper:
    """停车后把 trace 分页拉出来的协作式状态机（不另起线程）。

    这个 App 里只有控制环一个线程调用 Bridge，main.py 的退出路径也写明
    "两线程并发访问 Bridge" 的风险，所以这里不引入第二个调用者。
    """

    def __init__(self, directory=None, timeout=0.2):
        self.directory = directory or tempfile.gettempdir()
        try:
            os.makedirs(self.directory, exist_ok=True)
        except OSError:
            # 目录建不出来就退回系统临时目录，导出本身不该因此失败
            self.directory = tempfile.gettempdir()
        self.timeout = timeout
        self.store = TraceStore()
        self.offset = 0
        self.pending = False
        self.finished = 0
        self.last_error = ""
        self.held = False          # MCU 环形缓冲是否已被我们冻结
        self.windowed = False      # 是否已经定位到尾部窗口
        self.requested_rows = 0    # 本次请求的窗口行数

    def request(self):
        """置位一次导出请求；重复调用不叠加。"""
        if self.pending:
            return
        self.pending = True
        self.offset = 0
        self.windowed = False
        self.store = TraceStore()

    def busy(self):
        return self.pending

    def summary(self):
        """供 REST 回读：是否在导出、已导出几次、最近一次的信息与错误。"""
        info = dict(self.store.summary())
        info.update({"pending": self.pending, "finished": self.finished,
                     "directory": self.directory, "last_error": self.last_error,
                     "offset": self.offset})
        return info

    def pump(self, call, timeout=None):
        """拉一页；返回 True 表示还需要下一拍继续。"""
        if not self.pending:
            return False
        timeout = self.timeout if timeout is None else timeout
        try:
            if not self.held:
                # 先冻结再读：不冻结的话，一次 2048 行的导出要几秒，
                # 环形缓冲在这期间继续被覆盖，拼出来的曲线会跳段。
                call("ros_trace_hold", 1, timeout=timeout)
                self.held = True
            raw = call("ros_trace_poll", 1, self.offset, timeout=timeout)
            count, total, conflict = self.store.begin_page(raw)
            if conflict:
                self._fail("采集期间 trace 会话号变化，本次数据作废")
                return False
            if not self.windowed:
                # 只有第一页知道 total，据此把起点挪到尾部窗口；挪了就要重取这一页，
                # 否则会把"最旧的一段"和"最近的一段"拼在一起。
                self.windowed = True
                self._seek_tail(raw, total)
                if self.offset != 0:
                    raw = call("ros_trace_poll", 1, self.offset, timeout=timeout)
                    count, total, _ = self.store.begin_page(raw)
            self.store.append_page(raw, count)
            self.offset += count
            if count < PAGE_ROWS or self.offset >= total:
                self._finish(call, timeout)
                return False
            return True
        except Exception as exc:
            self._release(call, timeout)
            self._fail(str(exc))
            return False

    def _finish(self, call, timeout):
        self._release(call, timeout)
        self.store._count_gaps()
        self.store.fetched_at = time.time()
        name = "unoq-trace-%d.csv" % int(time.time())
        path = os.path.join(self.directory, name)
        try:
            self.store.save_trace_csv(path)
            self._fetch_env(call, timeout)
            if self.store.env:
                self.store.save_env_csv(path.replace(".csv", ".env.csv"))
        except (OSError, ValueError) as exc:
            self._fail("落盘失败：%s" % exc)
            return
        self.pending = False
        self.finished += 1
        info = self.store.summary()
        print("[trace] saved %s rows=%d %.2fs %.1fHz dropped=%d gaps=%d env=%d"
              % (path, info["rows"], info["duration_s"], info["rate_hz"],
                 info["dropped"], info["seq_gaps"], info["env_rows"]), flush=True)

    def _fetch_env(self, call, timeout):
        """环境样本条数少（≤600），按页拉完即可。"""
        pages, offset = [], 0
        for _ in range(40):
            raw = call("ros_env_poll", 1, offset, timeout=timeout)
            if not raw or len(raw) < ENV_META:
                break
            total = int(raw[1])
            count = len(raw[ENV_META:]) // ENV_FIELDS
            pages.append(raw)
            offset += count
            if count < PAGE_ROWS or offset >= total:
                break
        self.store.load_env(pages)

    def _seek_tail(self, raw, total):
        """把读取起点挪到环形缓冲的尾部窗口，默认只取最近 MAX_DUMP_ROWS 行。

        整圈 2048 行要走 128 次 RPC，每次约 50 ms，合计 6 s 以上；而关心的
        往往只是停车前那十几秒。窗口可配，0 表示整圈。
        """
        if not MAX_DUMP_ROWS:
            return
        capacity = int(raw[3])
        stored = min(total, capacity)
        self.requested_rows = min(MAX_DUMP_ROWS, stored)
        self.offset = max(0, stored - self.requested_rows)

    def _release(self, call, timeout):
        if not self.held:
            return
        self.held = False
        try:
            call("ros_trace_hold", 0, timeout=timeout)
        except Exception:
            # 解冻失败不影响已导出的数据，下一次 request 会重新冻结
            pass

    def _fail(self, message):
        self.store.error = message
        self.pending = False
        self.last_error = message
        print("[trace] 导出失败：%s" % message, flush=True)
