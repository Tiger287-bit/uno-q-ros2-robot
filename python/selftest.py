"""底盘回归自检：在控制环内按时间线下发指令并采样，给出逐项判定。

为什么放在控制环里：这个 App 只有控制环一个线程调用 Bridge，另起线程会引入
并发访问；而且自检要求"每拍保持心跳"，否则 MCU 的 600 ms 看门狗会在测试中途
把车停掉，测出来的现象和被测缺陷混在一起。

用例（全部要求架空车轮，车不许落地）：

- ``zero_left``  (647, 0)：左轮目标为零，右轮必须继续受控，左轮 pwm 必须归零
- ``zero_right`` (0, 647)：镜像
- ``reverse``    +800 → -800：换向必须先撤旧输出再给反向 PWM
- ``estop``      +800 → ros_stop：立即停机，pwm 归零且 reason=Manual
- ``watchdog``   +800 后不再下发：600 ms 后必须 Watchdog 停机

nFAULT 无法用软件注入（需要短接驱动故障脚或制造过流），单独列在现场清单里。
"""

import csv
import os
import time

import control

# 逐帧状态落盘目录：与 MCU trace 放在一起，便于同一次试验的两种采样对照
CSV_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "trace")

CASES = {
    # timeline: (相对时刻 s, 动作)；heartbeat=True 表示每拍重发当前指令
    "zero_left": {
        # 先两轮同速，再把左轮目标压到 0 —— 只有"上一拍还在转"的轮才会暴露
        # applyPidWheel 提前返回导致的旧 PWM 残留。
        "timeline": [(0.0, "drive", (647, 647)), (0.8, "drive", (647, 0)),
                     (2.4, "stop", ())],
        "heartbeat": True,
        "duration": 2.8,
        "hold_from": 1.0,
        "hold_to": 2.3,
        "expect_zero": 1,          # M2 左轮
        "expect_drive": 0,         # M1 右轮
    },
    "zero_right": {
        "timeline": [(0.0, "drive", (647, 647)), (0.8, "drive", (0, 647)),
                     (2.4, "stop", ())],
        "heartbeat": True,
        "duration": 2.8,
        "hold_from": 1.0,
        "hold_to": 2.3,
        "expect_zero": 0,
        "expect_drive": 1,
    },
    "reverse": {
        "timeline": [(0.0, "drive", (800, 800)), (1.5, "drive", (-800, -800)),
                     (3.0, "stop", ())],
        "heartbeat": True,
        "duration": 3.4,
    },
    "estop": {
        "timeline": [(0.0, "drive", (800, 800)), (1.5, "stop", ())],
        "heartbeat": True,
        "duration": 2.2,
    },
    "watchdog": {
        # 只下发一次，之后完全沉默，等 MCU 自己超时停机。
        "timeline": [(0.0, "drive", (800, 800))],
        "heartbeat": False,
        "duration": 1.6,
    },
}

# 诊断用例：只为了"看见"某一轮是否受控，不参与回归判定。
# 单次通电最多 0.7 s（再长会触发堵转保护），中间用 ros_stop 清故障后再来一次。
def _pulses(right, left, times):
    timeline = []
    for k in range(times):
        start = round(k * 1.6, 2)
        timeline.append((start, "drive", (right, left)))
        timeline.append((round(start + 0.7, 2), "stop", ()))
    return {"timeline": timeline, "heartbeat": True,
            "duration": round(times * 1.6 + 0.6, 2), "diagnostic": True}


DIAG = {
    "spin_left": _pulses(0, 600, 4),
    "spin_right": _pulses(600, 0, 4),
}


def _step(cps):
    """双轮同目标阶跃：从静止一步给到 cps，保持 3 s 后停车。

    判定用的是 status 里的滤波轮速（tau=80 ms），所以上升时间会被低估约
    一个 tau、峰值会被削掉一点。它的作用只是"每次上板都跑一遍、不许退化"；
    真正的上升时间/超调以同一时段的 100 Hz trace（未滤波）为准。
    """
    return {"timeline": [(0.0, "drive", (cps, cps)), (3.0, "stop", ())],
            "heartbeat": True, "duration": 3.5, "step_cps": cps}


STEPS = {
    "step_p600": _step(600),
    "step_p1200": _step(1200),
    "step_p2400": _step(2400),
    "step_n600": _step(-600),
    "step_n1200": _step(-1200),
}

# 10 分钟保持：验证长时间恒速下没有热漂、积分漂移或周期性塌陷。
# 采样按 6 拍抽 1 拍存（5 Hz），否则 600 s × 30 Hz ≈ 1.8 万个样本，内存和落盘都太重。
SOAK = {
    "soak_p1200": {
        "timeline": [(0.0, "drive", (1200, 1200)), (600.0, "stop", ())],
        "heartbeat": True, "duration": 604.0, "step_cps": 1200,
        "decimate": 6, "soak": True,
    },
}

STOP_REASON = {"manual": 1, "watchdog": 3, "expired": 2, "complete": 13}

# 上升时间判定的物理构成，必须与固件同步：
# RAMP_CPS_PER_S 对应 sketch/motion_bridge.cpp 的 DRIVE_RAMP_CPS_PER_S；
# FILTER_FLOOR_MS 是"读到的速度必然滞后"的下限——遥测轮速经过 80 ms 一阶滤波
# （10%→90% 约 176 ms），PID 用的速度又来自 50 ms 滑动窗（约 25 ms）；
# BREAKAWAY_BUDGET_MS 是静摩擦起转段：起转要 58 码、运行只要 33 码，
# 从静止到动起来实测 100~200 ms，这段靠控制压不掉（见 batch2/REPORT.md §6）。
# 80 ms 这种指标只能拿 100 Hz trace 的未滤波 speed 去量，用本文件的 30 Hz
# 滤波遥测永远量不到，判据写成固定 80 ms 只会永远 FAIL。
RAMP_CPS_PER_S = 6000
FILTER_FLOOR_MS = 220.0
BREAKAWAY_BUDGET_MS = 200.0


def known_cases():
    """全部可用用例名。凡是做白名单校验的地方都必须走这里。

    上一版把 CASES/DIAG 直接写在 main.py 里，新增 STEPS 后接口层仍按旧名单
    拒绝请求，现象是"文件已推上板、导入也成功，但 REST 一直回未知用例"。
    """
    return sorted(CASES) + sorted(STEPS) + sorted(SOAK) + sorted(DIAG)


class SelfTest:
    """时间线驱动的自检状态机，由控制环每拍推进。"""

    def __init__(self):
        self.case = ""
        self.started = 0.0
        self.command = (0, 0)
        self.sent = set()
        self.samples = []
        self._ticks = 0
        self.verdict = ""
        self.passed = None
        self.error = ""
        self.history = []          # 已结束用例的判定，供接口回读（脚本采集用）
        self._cancel = False       # 外部取消请求，独立于网页急停锁存
        self._pending_dumps = []   # 因急停推迟的 CSV：先停车，停稳再补写
        # 最近一次读到的 MCU 状态。控制环在自检期间要靠它继续发 /joint_states
        # 与 /imu（否则里程计的 odom→base_footprint 会停更），而自检每拍本来
        # 就读一次 ros_status，直接复用这一份，不再多占 Bridge。
        self.last_status = None
        self.last_status_at = 0.0

    @property
    def active(self):
        return bool(self.case)

    @property
    def cancelled(self):
        """是否有待处理的取消请求。控制环每拍读一次。"""
        return self._cancel

    def clear_cancel(self):
        """网页解除锁定：放开自检阻断。"""
        self._cancel = False

    def cancel(self):
        """请求中止当前用例。

        只置标志、不做任何 IO：这个可能在 socket 线程里调用，写 CSV 会阻塞回调。
        必须和网页急停锁存分开——stop 之后紧跟 resume 时，控制环可能一次都没
        看到 PAUSED，自检就漏掉了这次急停。
        """
        self._cancel = True

    def _case(self):
        """当前用例配置：回归、阶跃或诊断三者之一。"""
        return (CASES.get(self.case) or STEPS.get(self.case)
                or SOAK.get(self.case) or DIAG[self.case])

    def request(self, name):
        """开始一个用例；已有用例在跑时拒绝。"""
        if self.active:
            self.error = "上一项自检尚未结束：%s" % self.case
            return False
        if self._cancel:
            # 急停置起的阻断只有"解除锁定"能清。若在这里清掉，急停前排队
            # 的那个请求会自己把标志抹掉，照样跑起来并开始发心跳。
            self.error = "急停未解除，自检不启动"
            return False
        if name not in CASES and name not in DIAG and name not in STEPS \
                and name not in SOAK:
            self.error = "未知用例：%s" % name
            return False
        self.case = name
        self.started = time.monotonic()
        self.command = (0, 0)
        self.sent = set()
        self.samples = []
        self.last_status = None
        self.last_status_at = 0.0
        self.verdict = ""
        self.passed = None
        self.error = ""
        return True

    def recent_status(self, max_age):
        """最近一次状态帧；超过 max_age 秒没有更新就返回 None。

        自检被急停打断时 step() 不再读状态，这个函数就是"别再拿旧帧去发
        /joint_states"的闸门 —— 陈旧帧会让里程计误以为轮子停住。
        """
        if self.last_status is None:
            return None
        if time.monotonic() - self.last_status_at > max_age:
            return None
        return self.last_status

    def abort(self, reason="", dump=True):
        """外部打断（网页急停）：立刻收尾，不再下发任何指令。

        刻意不清 _cancel：阻断要一直留到网页解除锁定为止，否则急停前排队
        的请求会在下一拍重新启动。
        """
        if not self.active:
            return False
        self._finish(False, reason or "被外部中止", dump=dump)
        return True

    def step(self, call, timeout=0.2, abort=None):
        """推进一拍：按时间线下发、保持心跳、采样；到时给出判定。

        abort 是外部阻断判据（急停锁存 / 待停车）。下发任何非零指令之前都要
        重新问一次：检查和下发之间隔着一个 Bridge 往返，急停恰好落在这个窗口里
        时，这一拍仍会把已经算好的轮速发出去。
        """
        def blocked():
            return self._cancel or (abort is not None and abort())

        if blocked():
            self._finish(False, "急停中止", dump=False)
            return True            # 告诉调用方：本拍就要停车

        case = self._case()
        elapsed = time.monotonic() - self.started

        for index, (at, action, args) in enumerate(case["timeline"]):
            if elapsed >= at and index not in self.sent:
                self.sent.add(index)
                try:
                    if action == "drive":
                        if blocked():
                            self._finish(False, "急停中止", dump=False)
                            return True
                        self.command = args
                        call("ros_drive", args[0], args[1], timeout=timeout)
                    elif action == "stop":
                        self.command = None
                        call("ros_stop", timeout=timeout)
                except Exception as exc:
                    # RPC 超时可能只是应答丢了，MCU 已经按这条指令动起来。
                    # 所以这里必须让调用方进入停车流程，不能当成"没下发"。
                    self._finish(False, "指令下发失败：%s" % exc, dump=False)
                    return True

        # 心跳：正常驾驶每拍都会重发 ros_drive，自检必须复现这一点，
        # 否则看门狗会在用例中途触发，把"零目标"和"通信超时"混为一谈。
        if case.get("heartbeat") and self.command not in (None, (0, 0)):
            if blocked():
                self._finish(False, "急停中止", dump=False)
                return True
            try:
                call("ros_drive", self.command[0], self.command[1], timeout=timeout)
            except Exception as exc:
                self._finish(False, "心跳下发失败：%s" % exc, dump=False)
                return True

        status = control.decode_status(call("ros_status", timeout=timeout))
        if status is not None:
            # 控制环要用这一帧继续发 /joint_states 与 /imu（见 recent_status）。
            self.last_status = status
            self.last_status_at = time.monotonic()
            # 长时间用例按 decimate 抽帧落盘：600 s 全存会让内存和 CSV 都不可用。
            decimate = int(case.get("decimate", 1))
            self._ticks += 1
            if self._ticks % decimate == 0:
                self.samples.append((elapsed, status))

        if elapsed >= case["duration"]:
            self._judge(elapsed)
        return False

    # ---------- 判定 ----------

    def _window(self, start, end):
        return [(t, s) for t, s in self.samples if start <= t <= end]

    def _tail_state(self):
        """末帧的 mode/reason，判定失败时用来区分"没下发"和"被固件拒了"。"""
        if not self.samples:
            return "无状态帧"
        last = self.samples[-1][1]
        return "末帧 mode=%d reason=%d" % (last["mode"], last["reason"])

    def _judge(self, elapsed):
        case = self._case()
        if case.get("diagnostic"):
            peak = [max(abs(s["wheels"][i]["speed"]) for _, s in self.samples)
                    for i in (0, 1)]
            self._finish(True, "诊断：M1 峰值=%.0f cps，M2 峰值=%.0f cps"
                         % (peak[0], peak[1]))
            return
        if not self.samples:
            self._finish(False, "没有采到任何状态帧")
            return

        if "step_cps" in case:
            if case.get("soak"):
                self._judge_soak(case)
            else:
                self._judge_step(case)
            return

        if "expect_zero" in case:
            window = self._window(case["hold_from"], case["hold_to"])
            if not window:
                self._finish(False, "稳态窗口内没有样本")
                return
            zero_index = case["expect_zero"]
            drive_index = case["expect_drive"]
            zero_pwm = max(abs(s["wheels"][zero_index]["pwm"]) for _, s in window)
            drive_pwm = min(abs(s["wheels"][drive_index]["pwm"]) for _, s in window)
            zero_speed = max(abs(s["wheels"][zero_index]["speed"]) for _, s in window)
            last_zero = abs(window[-1][1]["wheels"][zero_index]["speed"])
            detail = ("零目标轮最大|pwm|=%d（要求 0），末速=%.0f cps；"
                      "另一轮最小|pwm|=%d，末速=%.0f cps；%s"
                      % (zero_pwm, last_zero, drive_pwm,
                         window[-1][1]["wheels"][drive_index]["speed"],
                         self._tail_state()))
            ok = zero_pwm == 0 and drive_pwm > 0 and last_zero < 100
            self._finish(ok, detail)
            return

        if self.case == "reverse":
            signs = []
            for _, s in self.samples:
                pwm = s["wheels"][0]["pwm"]
                if pwm:
                    signs.append(1 if pwm > 0 else -1)
            rose = any(s > 0 for s in signs)
            fell = any(s < 0 for s in signs)
            # 换向必须先把旧输出撤掉：出现"正负两轮同时非零"即为失败。
            mixed = any((s["wheels"][0]["pwm"] > 0) != (s["wheels"][1]["pwm"] > 0)
                        and s["wheels"][0]["pwm"] and s["wheels"][1]["pwm"]
                        for _, s in self.samples)
            detail = "正向出现=%s 反向出现=%s 双轮异号=%s；%s" % (
                rose, fell, mixed, self._tail_state())
            self._finish(rose and fell and not mixed, detail)
            return

        if self.case == "estop":
            stop_at = case["timeline"][1][0]
            after = [(t, s) for t, s in self.samples if t >= stop_at]
            if not after:
                self._finish(False, "停机后没有样本")
                return
            delay = None
            for t, s in after:
                if s["wheels"][0]["pwm"] == 0 and s["wheels"][1]["pwm"] == 0:
                    delay = (t - stop_at) * 1000.0
                    break
            reason = after[-1][1]["reason"]
            detail = ("pwm 归零耗时=%s ms，末帧 mode=%d reason=%d"
                      % ("未归零" if delay is None else "%.0f" % delay,
                         after[-1][1]["mode"], reason))
            ok = delay is not None and delay <= 120 and after[-1][1]["mode"] == 0
            ok = ok and reason in (STOP_REASON["manual"], STOP_REASON["complete"])
            self._finish(ok, detail)
            return

        if self.case == "watchdog":
            # 最后一次下发在 t=0，之后沉默；600 ms 后应由固件看门狗停机。
            late = [(t, s) for t, s in self.samples if t >= 0.9]
            if not late:
                self._finish(False, "看门狗窗口内没有样本")
                return
            reason = late[-1][1]["reason"]
            zero = late[-1][1]["wheels"][0]["pwm"] == 0
            detail = "t>=0.9s 末帧 reason=%d pwm=%d（期望 reason=3）" % (
                reason, late[-1][1]["wheels"][0]["pwm"])
            self._finish(reason == STOP_REASON["watchdog"] and zero, detail)
            return

        self._finish(False, "用例没有对应判定：%s" % self.case)

    def _judge_soak(self, case):
        """10 分钟保持：看漂移和是否有周期性塌陷，不看单点峰谷。

        跳过前 10 s（斜坡和起转补一脚都在这段），比较"第一个 60 s"和"最后一个
        60 s"的均值差：恒速工况下这条差值就是温漂 + 积分漂移的总和。
        """
        command = case["step_cps"]
        magnitude = float(abs(command))
        sign = 1.0 if command > 0 else -1.0
        window = [(t, s) for t, s in self.samples if 10.0 <= t <= 595.0]
        if len(window) < 60:
            self._finish(False, "长稳样本不足：%d 帧" % len(window))
            return

        parts = []
        ok = True
        for index in (0, 1):
            series = [(t, s["wheels"][index]["speed"] * sign) for t, s in window]
            head = [v for t, v in series if t <= 70.0]
            tail = [v for t, v in series if t >= 535.0]
            drift = (sum(tail) / len(tail) - sum(head) / len(head)) / magnitude
            allv = [v for _, v in series]
            worst = max(abs(v - command) for v in allv) / magnitude
            # 任何一帧 mode 不是运行、或 pwm 归零，都算中途掉线
            dropout = sum(1 for _, s in window
                          if s["mode"] != 1 or s["wheels"][index]["pwm"] == 0)
            parts.append("M%d 漂移=%+.1f%% 最大偏差=%.0f%% 掉帧=%d"
                         % (index + 1, drift * 100.0, worst * 100.0, dropout))
            ok = ok and abs(drift) <= 0.01 and worst <= 0.25 and dropout == 0
        self._finish(ok, "10 min 保持 %d cps：" % command + "；".join(parts))

    def _judge_step(self, case):
        """阶跃用例：逐轮算超调、90% 上升时间、稳态均值与纹波。

        只在停车前取窗（t<=2.9 s），否则减速段会把"稳态"和"超调"一起污染。
        """
        command = case["step_cps"]
        magnitude = float(abs(command))
        window = [(t, s) for t, s in self.samples if 0.0 <= t <= 2.9]

        parts = []
        ok = True
        for index in (0, 1):
            series = [(t, s["wheels"][index]["speed"]) for t, s in window]
            # 沿命令方向取幅值：倒车用例的期望符号为负。
            series = [(t, v if command > 0 else -v) for t, v in series]
            if len(series) < 10:
                self._finish(False, "样本太少，无法判定阶跃响应")
                return

            peak = max(v for _, v in series)
            overshoot = (peak - magnitude) / magnitude

            # 90% 上升时间：第一次达到 0.9 倍目标，之后再也没跌破过 0.9 倍。
            rise = None
            for k, (t, v) in enumerate(series):
                if v >= 0.9 * magnitude and all(x[1] >= 0.9 * magnitude for x in series[k:]):
                    rise = t * 1000.0
                    break

            tail = [v for t, v in series if t >= 2.0]
            mean = sum(tail) / len(tail)
            ripple = max(tail) - min(tail)
            step_error = (mean - magnitude) / magnitude

            parts.append("M%d 超调=%+.0f%% 上升=%s ms 稳态=%+.0f%% 纹波=%.0f cps"
                         % (index + 1, overshoot * 100.0,
                            "未达" if rise is None else "%.0f" % rise,
                            step_error * 100.0, ripple))
            # 上升时间的上界 = 斜坡时间 + 滤波/窗口滞后 + 静摩擦起转段。
            rise_limit = (FILTER_FLOOR_MS + BREAKAWAY_BUDGET_MS
                          + 0.9 * magnitude / RAMP_CPS_PER_S * 1000.0)
            ok = ok and rise is not None and rise <= rise_limit
            ok = ok and overshoot <= 0.10
            ok = ok and abs(step_error) <= 0.02
            ok = ok and ripple <= 0.05 * magnitude
        self._finish(ok, "%d cps 阶跃：" % command + "；".join(parts))

    def _finish(self, passed, detail, dump=True):
        """收尾。dump=False 时只记录判定，CSV 留到停稳后由 flush_dumps 补写——
        写盘是同步的，几百个样本足够把这一拍的停车推迟掉。"""
        self.passed = bool(passed)
        self.verdict = detail
        name = self.case
        self.case = ""
        self.history.append({"case": name, "passed": bool(passed),
                             "detail": detail, "at": time.time()})
        samples = list(self.samples)       # 快照：下一个用例会覆盖 self.samples
        if dump:
            self._dump_csv(name, samples)
        else:
            self._pending_dumps.append((name, samples))
        print("[selftest] %s %s  %s" % (name, "PASS" if passed else "FAIL", detail),
              flush=True)

    def flush_dumps(self):
        """补写因急停推迟的 CSV。调用方须保证此刻车已经停稳。"""
        pending, self._pending_dumps = self._pending_dumps, []
        for name, samples in pending:
            self._dump_csv(name, samples)

    def _dump_csv(self, name, samples=None):
        """把逐帧状态落盘：判定失败时要靠它看出"哪一拍开始不对"。

        /api/telemetry 只在正常驾驶分支更新，自检期间不刷新，所以必须自己存。
        """
        rows = self.samples if samples is None else samples
        if not rows:
            return
        try:
            os.makedirs(CSV_DIR, exist_ok=True)
            path = os.path.join(CSV_DIR, "selftest-%s-%d.csv" % (name, int(time.time())))
            with open(path, "w", newline="", encoding="utf-8") as handle:
                writer = csv.writer(handle)
                writer.writerow(["t", "mode", "reason", "phase",
                                 "m1_target", "m1_speed", "m1_pwm", "m1_invalid",
                                 "m2_target", "m2_speed", "m2_pwm", "m2_invalid"])
                for t, status in rows:
                    wheels = status.get("wheels") or [{}, {}]
                    writer.writerow([round(t, 3), status.get("mode"), status.get("reason"),
                                     status.get("phase"),
                                     wheels[0].get("target", 0), wheels[0].get("speed", 0),
                                     wheels[0].get("pwm", 0), wheels[0].get("invalid", 0),
                                     wheels[1].get("target", 0), wheels[1].get("speed", 0),
                                     wheels[1].get("pwm", 0), wheels[1].get("invalid", 0)])
            print("[selftest] csv %s" % path, flush=True)
        except OSError as exc:
            print("[selftest] csv 写入失败：%s" % exc, flush=True)

    def summary(self):
        return {"case": self.case, "verdict": self.verdict,
                "passed": self.passed, "error": self.error,
                "samples": len(self.samples), "history": self.history}
