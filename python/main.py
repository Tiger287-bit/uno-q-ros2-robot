"""App 装配层：控制环、启动与 WebUI 接线。

模块分工（按依赖顺序）
    app_state.py       跨线程共享的状态与单例
    ros_publishers.py  /scan /joint_states /imu 的发布格式
    scan_pipeline.py   雷达分片拼帧
    payload.py         遥测载荷与调参
    socket_handlers.py 网页按钮回调
    api_routes.py      HTTP 接口
    main.py            本文件：控制环与启动

控制环只做"算与下发"，不关心消息怎么序列化、网页怎么读。
"""

import json
import queue
import threading
import time

from arduino.app_utils import App, Bridge

import api_routes
import app_state
import config
import control
import latency
import ros_publishers as publishers
import socket_handlers

from app_state import (CAMERA, DETECTOR, JOY, LINK, MCU_STALE_S,
                       NAV, PAUSED, QUIT, RECORDER, REQUESTS, SELFTEST, SOURCE,
                       STARTUP_GRACE_S, STATE, STATE_LOCK, STOP_PENDING, TRACE,
                       UI, WEB)
from payload import TUNE_REFRESH_TICKS, apply_tune, build_payload, refresh_tune
import scan_pipeline
from scan_pipeline import LIDAR_EVERY_TICKS, SCAN_STATS, fetch_scan_slice

# MCU 会话跟踪：重启后 uptime/sequence 会归零，而轮子累计计数也跟着清零。
# 那种变化量可能只有几厘米，和"快速倒车"分不开，光看角度必然漏判 ——
# 所以给每一代计数编号，并把这个编号写进 joint_states（见 ros_publishers）。
MCU_SESSION = {"uptime": None, "sequence": None, "epoch": 0, "fault_key": None}
# 每拍最多处理一个排队请求。队列里可能堆着网页连点的整帧读取，
# 一次清空会让几个 Bridge 调用串在同一拍，把驾驶指令挤到 33 ms 之后。
MAX_REQUESTS_PER_TICK = 1
# 自检期间复用 step() 读到的那帧状态的时效上限。30 Hz 下正常只差一拍；
# 放宽到 0.2 s 是为了容忍偶发慢拍，再旧就别发出去误导里程计了。
SELFTEST_STATUS_MAX_AGE_S = 0.2
# 链路延迟探针：只统计，不改控制行为。日志里带 MCU 的 uptime/sequence，
# 方便把 Linux 侧的一行汇总对上 MCU 侧的一拍 trace。
PROBE = latency.Probe(config.LATENCY_CAPACITY, config.LATENCY_REPORT_S)
# 发布类失败的最近一次内容与时刻，用来做去重打印（同名错误 5 s 内只报一次）。
ROS_ERRORS = {}


def report_ros_error(key, text, now=None):
    """发布通路的失败只打日志、不进底盘错误通道，并且要去重。

    这些失败只影响 /scan /joint_states /imu，不该让网页上的底盘状态变红；
    但完全静默更糟：下发链路正常、地图却不动，现场无从下手。30 Hz 下
    同一个异常会刷满日志，所以按"内容变化或 5 s"节流。
    """
    now = time.monotonic() if now is None else now
    last = ROS_ERRORS.get(key)
    if last is None or text != last[0] or now - last[1] > 5.0:
        print("[ros] " + text, flush=True)
        ROS_ERRORS[key] = (text, now)


def track_mcu_session(status):
    """MCU 重启检测：uptime 或 sequence 往回走就说明是新的会话。

    轮子累计计数已归零，必须先把 epoch 递增，再让这一帧带着新 epoch 发出去，
    里程计才会把它当基准而不是拿"清零后的值"去减"清零前的基准"。
    """
    uptime = status.get("uptime_ms")
    seq = status.get("sequence")
    if (MCU_SESSION["uptime"] is not None
            and ((uptime is not None and uptime < MCU_SESSION["uptime"])
                 or (seq is not None and MCU_SESSION["sequence"] is not None
                     and seq < MCU_SESSION["sequence"]))):
        # 只递增计数器，不再发独立通知：它随下一帧 joint_states 一起走。
        # 单独发一条会与轮帧竞态，晚到时还会二次重置。
        MCU_SESSION["epoch"] += 1
        # 雷达拼帧必须跟着换会话：旧会话的分片、帧号基准、退避状态全部作废，
        # 否则新会话的 seq 与旧的一致时会把两圈数据拼在一起（NAV-006-D）。
        scan_pipeline.on_mcu_session_change(time.monotonic(),
                                            "epoch=%d" % MCU_SESSION["epoch"])
        print("[main] 检测到 MCU 重启，计数世代 +1（epoch=%d）"
              % MCU_SESSION["epoch"], flush=True)
    MCU_SESSION["uptime"] = uptime
    MCU_SESSION["sequence"] = seq


def publish_telemetry(status, wall=None):
    """发一帧 /imu 与 /joint_states，两者互不牵连。

    原来这两条写在同一段 try 里：IMU 序列化抛一次（字段缺失、脏数据）就会把
    轮状态一起跳过，里程计丢一整帧，表现为"车在动、odom 不动"。分开之后
    一条失败只丢自己，而且都有日志。

    IMU 先发：两个 publisher 各自排队，先入队的先到；轮状态后发能让同一批的
    IMU 先落进里程计的历史，减少"轮状态到了、IMU 还没到"从而退回轮差的机会。
    """
    wall = time.time() if wall is None else wall
    track_mcu_session(status)
    fault = status.get("fault")
    if fault is not None:
        fault_key = (MCU_SESSION["epoch"], fault["uptime_ms"], fault["reason"])
        if fault_key != MCU_SESSION["fault_key"]:
            MCU_SESSION["fault_key"] = fault_key
            print("[mcu-fault] " + json.dumps(fault, ensure_ascii=False), flush=True)
    if status.get("mode") == 2:
        if not MCU_SESSION.get("fault_canceled"):
            MCU_SESSION["fault_canceled"] = True
            NAV.cancel("fault")
    else:
        MCU_SESSION["fault_canceled"] = False
    try:
        publishers.publish_imu(status, wall)
    except Exception as exc:
        report_ros_error("imu", "IMU 发布失败：%s" % exc)
    try:
        publishers.publish_joints(status, wall, MCU_SESSION["epoch"])
    except Exception as exc:
        report_ros_error("joints", "轮状态发布失败：%s" % exc)


def pump_scan(now):
    """读一片雷达并（必要时）发布 /scan；异常只丢一帧，不许带走控制环。

    Bridge 调用超时、固件没应答都会走到这里。以前是 `except: pass`，
    结果是 /scan 悄悄停发、SLAM 那边只是"地图不动"，Linux 侧没有任何痕迹。
    """
    try:
        fetch_scan_slice(now)
    except Exception as exc:
        SCAN_STATS["errors"] += 1
        report_ros_error("scan", "雷达分片读取失败：%s" % exc, now)


def control_loop():
    """唯一访问 MCU 的线程：处理命令、下发速度、读状态、记录遥测。"""
    period = 1.0 / config.CONTROL_HZ
    tick = 0
    driving = False
    reject_at = 0.0                    # 首次被拒起转的时刻，0 表示当前没有连续被拒
    mcu = None
    logged_error = ""
    logged_at = 0.0
    boot_at = time.monotonic()         # 上电时刻，用于区分"启动中"与"失联"
    mcu_at = 0.0                       # 最近一次读到有效状态的时刻
    push_error = ""
    push_at = 0.0
    last_started = 0.0                 # 上一拍开始时刻，用于算控制环间隔
    stuck_reads = 0                    # 连续"满占空比但轮速为 0"的状态帧数
    # 导航指令短暂掉零时的保持：控制器偶尔一拍没发出速度，mux 立刻输出 0，
    # App 就下发 ros_stop，把驱动和增量式 PID 的累加器一起清零。原地转
    # 需要的占空比要靠累加器攒，一清零就永远攒不起来（实测导航原地转
    # mode 在 1/0 之间反复切，轮速始终 0，而遥控连续输入就正常）。
    nav_hold_at = 0.0
    nav_hold = (0, 0, False)
    was_driving = False                # 上一拍是否在驱动，用于识别"刚停车"
    was_selftest = False               # 上一拍是否在自检，用于结束后补一次 trace
    nav_seq = 0                        # 已推给页面的导航帧序号，用于去重
    nav_in_motion = False

    while not QUIT.is_set():
        started = time.monotonic()
        if last_started:
            # 名义 1/30 s；这里记的是拍与拍之间的实际间隔，包含等待。
            gap_ms = (started - last_started) * 1000.0
            PROBE.add("loop", gap_ms)
            # 停顿超过 250 ms 单独打一行。这个量级已经逼近固件 WATCHDOG_MS：
            # 连续两拍各 300 ms，MCU 就会因为收不到心跳自己把车停下，现场表现
            # 是"跑着跑着突然不动了"，而 5 s 一次的 [lat] 汇总只报 max，
            # 事后完全看不出是哪一拍、当时在驱动还是在等雷达。
            if gap_ms > 250.0:
                print("[stall] 控制环停顿 %.0f ms（driving=%s paused=%s tick=%d）"
                      % (gap_ms, driving, PAUSED.is_set(), tick), flush=True)
        last_started = started
        errors = []
        try:
            NAV.pump()
        except Exception as exc:
            report_ros_error("nav", "导航命令发送失败：%s" % exc)

        # 自检接管底盘期间，正常指令通道整拍让路：两个来源同时下发会让
        # "零目标轮停转"这类判据失效（另一路每拍都在把目标写回去）。
        if SELFTEST.active:
            DETECTOR.set_motion_active(True)
            # 网页急停必须先于自检生效。原来这个分支直接 continue，把后面的
            # PAUSED 检查整段跳过去了，急停对正在跑的自检无效。
            # 三个条件都要看：
            # - cancelled：急停时置起，只有 resume 能清
            # - PAUSED：常规锁存
            # - STOP_PENDING：急停立即置位、只有 ros_stop=True 才清。前两者
            #   都可能被同一拍内的 resume 清掉，这条不会。
            if SELFTEST.cancelled or PAUSED.is_set() or STOP_PENDING.is_set():
                # 顺序要紧：先停车再落盘。abort 只把判定和样本放进内存队列
                # （dump=False），不写盘：
                # 同步写 CSV 会把这一拍的停车推迟掉。真正的落盘在停稳后做。
                SELFTEST.abort("网页急停", dump=False)
                try:
                    accepted = Bridge.call("ros_stop", timeout=config.MCU_TIMEOUT_S)
                    if accepted:
                        driving = False
                        STOP_PENDING.clear()
                        SELFTEST.flush_dumps()     # 停稳了，补写刚才推迟的记录
                    else:
                        # 驱动还没休眠（电机在减速）。置待停车状态，下一拍
                        # 强制零目标、只重试停车，不会被摇杆输入插队。
                        driving = True
                        STOP_PENDING.set()
                except Exception as exc:
                    errors.append(f"急停失败：{exc}")
                    driving = True
                    STOP_PENDING.set()
            else:
                try:
                    needs_stop = SELFTEST.step(
                        Bridge.call, config.MCU_TIMEOUT_S,
                        abort=lambda: PAUSED.is_set() or STOP_PENDING.is_set())
                except Exception as exc:
                    # ros_status 超时之类的异常不能把控制线程带走：那会让网页急停
                    # 和后续的停车重试全部无人处理。
                    errors.append(f"自检异常：{exc}")
                    SELFTEST.abort("自检异常", dump=False)
                    needs_stop = True
                if needs_stop:
                    # 急停落在 step 内部、或驱动/心跳 RPC 超时：MCU 可能已经收下
                    # 那条轮速指令，只是应答丢了。本拍立刻停车，不拖到下一拍。
                    STOP_PENDING.set()
                    driving = True
                    try:
                        if Bridge.call("ros_stop", timeout=config.MCU_TIMEOUT_S):
                            STOP_PENDING.clear()
                            driving = False
                            SELFTEST.flush_dumps()
                    except Exception as exc:
                        errors.append(f"自检停车失败：{exc}")
                else:
                    driving = False        # 自检结束后的第一拍按静止处理
            # 自检只接管"指令通道"，遥测必须继续。这个分支原来直接 continue，
            # 把下面的读状态与雷达分片整段跳过了 —— /scan 一断，中继就转不出
            # /scan_slam，SLAM 收不到任何新扫描；/joint_states 一断，轮式里程计
            # 不再更新 odom→base_footprint，于是地图、动态 TF、扫描同时"冻结"，
            # 看起来像 SLAM 自己挂了。
            # 状态帧复用 step() 每拍读到的那一份，不额外占 Bridge；只有雷达
            # 分片要额外发 RPC，所以和正常分支一样避开读状态那一拍。
            if tick % config.STATUS_EVERY_TICKS == 0:
                status = SELFTEST.recent_status(SELFTEST_STATUS_MAX_AGE_S)
                if status is not None:
                    publish_telemetry(status)
            else:
                pump_scan(started)

            if errors:
                # 自检分支里的错误（急停失败、自检异常、停车失败）原来只进
                # 列表、没人打印，现场看不到任何提示。沿用同一套去重打印。
                text = "；".join(errors)
                if text != logged_error or time.monotonic() - logged_at > 5.0:
                    print(f"[selftest] {text}", flush=True)
                    logged_error, logged_at = text, time.monotonic()
            was_selftest = True
            PROBE.add("work", (time.monotonic() - started) * 1000.0)
            if config.LATENCY_ENABLED and PROBE.due():
                print(PROBE.report("selftest=%s" % SELFTEST.case), flush=True)
            tick += 1
            QUIT.wait(max(0.0, period - (time.monotonic() - started)))
            continue
        if was_selftest:
            # 自检有自己 30 Hz 的逐帧 CSV；100 Hz trace 留给正常驾驶，
            # 免得每次自检都拖几十秒的分页读取，把控制拍的节奏带偏。
            was_selftest = False

        try:
            message, age, _ = LINK.snapshot()
            # 指令年龄：/cmd_vel 消息从发布到被这一拍取用的间隔。它不计入
            # MCU 的 600 ms 看门狗（那是多久没收到有效驾驶请求），但决定
            # 本拍车体速度指令的新鲜度，所以单独记一条。
            PROBE.add("cmd_age", age * 1000.0)
        except Exception as exc:
            errors.append(f"读取指令失败：{exc}")
            message, age = {}, float("inf")           # 拿不到指令就按超时停车

        linear, angular = control.cmd_from(message)
        if age > config.CMD_TIMEOUT_S:
            linear = angular = 0.0
        movement_requested = bool(linear or angular)
        drive_fault = bool(mcu and mcu["mode"] == 2)

        # 这一帧是导航发的还是人发的：mux 以 2 Hz 心跳把来源发在
        # /cmd_vel_source 上。速度与来源是两条独立消息，来源可能晚到或
        # 超时 —— 只有"明确是人手"才允许吸附。来源不确定时按导航处理：
        # 错判成导航只是人手的转向手感略黏；错判成人手会让 Nav2 的
        # 碰撞预测失效（实测同一条 0.53 rad/s 指令会被吸附成 0.763）。
        try:
            source_message, source_age, _ = SOURCE.snapshot()
            command_source = (source_message.get("data")
                              if isinstance(source_message, dict) else "") or ""
        except Exception:
            command_source, source_age = "", float("inf")
        manual_command = (command_source in config.MANUAL_SOURCE_TOPICS
                          and source_age <= config.SOURCE_TIMEOUT_S)
        # 急停锁存：置位期间不论来源（手柄还是网页）一律按 0 处理。放在超时判断
        # 之后、wheel_targets 之前，下游的停车分支与故障清除逻辑都按静止走。
        if PAUSED.is_set() or STOP_PENDING.is_set() or drive_fault:
            linear = angular = 0.0

        # 只有人手指令才做内轮吸附；导航（以及来源不明）一律精确差速。
        # allow_pivot 保持 True：实测遥控的纯旋转完全正常，底盘能原地转，
        # 之前"转不动"的判断是错的（那是驱动/命令链断流造成的假象）。
        # 这个开关留着备用，但不再对导航关闭。
        right, left = control.wheel_targets(linear, angular,
                                            snap=manual_command,
                                            allow_pivot=True)
        # 导航指令保持：只在"先前是导航、这一拍突然变 0"时生效。
        # 人手给 0 是明确意图（松杆），绝不能保持，否则松杆后车还往前走。
        #
        # 安全前提：急停 / 停车未确认 / 驱动故障这三条必须先于保持生效。
        # 否则上一拍的非零指令会在 0.45 s 内被重新下发，把急停"顶"回去 ——
        # 这是保持机制最容易引入的致命回归，必须让安全状态清空保持缓存。
        now_hold = time.monotonic()
        if PAUSED.is_set() or STOP_PENDING.is_set() or drive_fault:
            nav_hold = (0, 0, False)
            nav_hold_at = 0.0
        elif right or left:
            if not manual_command:
                nav_hold = (right, left, False)
                nav_hold_at = now_hold
        elif (not manual_command) and nav_hold_at and \
                now_hold - nav_hold_at < config.NAV_CMD_HOLD_S:
            right, left = nav_hold[0], nav_hold[1]

        # MCU 按本地 IMU 执行保向。请求只由驾驶意图决定，网页快照年龄仅用于显示。
        hold_heading = (abs(linear) > config.ASSIST_LINEAR_MIN
                        and abs(angular) < config.ASSIST_ANGULAR_MAX
                        and right == left and right != 0)

        DETECTOR.set_motion_active(bool(right or left) or nav_in_motion)

        accepted = None
        try:
            if right == 0 and left == 0:
                reject_at = 0.0
                if driving or STOP_PENDING.is_set():
                    # 固件的 ros_stop 返回的是"驱动是否已休眠"，不是"命令是否收到"：
                    # 电机还在减速时它返回 false。只捕获异常不够，必须看返回值，
                    # 否则 false 被当成成功，重试状态就丢了。
                    try:
                        accepted = PROBE.call("stop", Bridge.call, "ros_stop",
                                              timeout=config.MCU_TIMEOUT_S)
                        if accepted:
                            driving = False
                            STOP_PENDING.clear()
                        else:
                            STOP_PENDING.set()
                    except Exception as exc:
                        STOP_PENDING.set()
                        errors.append(f"停车失败，下一拍重试：{exc}")
            elif PAUSED.is_set() or STOP_PENDING.is_set():
                # 从算完 wheel_targets 到这里之间来了急停：本拍不再下发非零指令。
                # driving 置真，下一拍会正常走停车分支（并重试到 ros_stop=True）。
                driving = True
                accepted = None
            else:
                ok = bool(PROBE.call("drive", Bridge.call, "ros_drive_heading",
                                     right, left, int(hold_heading),
                                     timeout=config.MCU_TIMEOUT_S))
                if ok:
                    reject_at = 0.0
                    accepted = True
                else:
                    # 固件在停车后 300 ms 内拒绝起转。连续快照里这属于"还没起转"，
                    # 不是指令失败：宽限期内回报 None（页面显示"等待起转"），
                    # 超过 DRIVE_REJECT_GRACE_S 仍被拒才回报 False。
                    now = time.monotonic()
                    if not reject_at:
                        reject_at = now
                    accepted = (None if now - reject_at < config.DRIVE_REJECT_GRACE_S
                                else False)
                driving = ok
        except Exception as exc:
            errors.append(f"下发失败：{exc}")
            # 抛异常说明"不确定固件有没有收到"：RPC 超时往往是固件已经执行、
            # 只是回包迟到或丢了。这里必须按"可能正在跑"处理——置 False 的话，
            # 下一拍指令归零时就不会再进停车分支，只能等固件 600 ms 看门狗；
            # 而界面此时显示 targets=[0,0]，看起来已经停了。与上面停车重试同理，
            # 保守方向是 True（多发一次 ros_stop 没有副作用）。
            driving = True

        # 排队请求放到驾驶指令之后处理。这里全是整帧读取、调参写入、
        # 雷达启停这类慢操作（单个 RPC 超时 200 ms，一拍只有 33 ms），
        # 排在前头会把停车命令推后。代价是它们晚一拍生效，可以接受。
        handled = 0
        for _ in range(MAX_REQUESTS_PER_TICK):
            try:
                request = REQUESTS.get_nowait()
            except queue.Empty:
                break
            handled += 1
            try:
                if request == "calibrate":
                    if Bridge.call("ros_calibrate_heading",
                                   timeout=config.MCU_TIMEOUT_S) is False:
                        errors.append("校准航向被拒绝：IMU 未就绪或车未静止")
                elif isinstance(request, tuple) and request[0] == "selftest":
                    if PAUSED.is_set() or STOP_PENDING.is_set() or drive_fault:
                        # 急停期间或停车未确认时进来的自检直接拒收，否则它会
                        # 先启动一拍、发一次心跳，再被自检分支中止。
                        errors.append("停车未完成，不启动自检")
                    elif len(request) > 2 and request[2] != app_state.ESTOP_SEQ:
                        # 这条请求入队之后发生过急停：作废，别让它跑起来。
                        errors.append("自检请求在急停前排队，已作废")
                    elif not SELFTEST.request(request[1]):
                        errors.append(f"自检未启动：{SELFTEST.error}")
                elif isinstance(request, tuple) and request[0] == "lidar_ctl":
                    error = ""
                    try:
                        method = "lidar_start" if request[1] == "start" else "lidar_stop"
                        Bridge.call(method, timeout=config.MCU_TIMEOUT_S)
                    except Exception as exc:
                        error = str(exc)
                    with STATE_LOCK:
                        STATE["lidar_frame"] = []
                        STATE["lidar_frame_error"] = error
                        STATE["lidar_frame_seq"] += 1
                elif isinstance(request, tuple) and request[0] == "lidar_frame":
                    frame, error = [], ""
                    try:
                        raw = Bridge.call("lidar_frame", int(request[1]),
                                          timeout=config.MCU_TIMEOUT_S)
                        frame = list(raw) if isinstance(raw, (list, tuple)) else []
                    except Exception as exc:
                        error = str(exc)
                    with STATE_LOCK:
                        STATE["lidar_frame"] = frame
                        STATE["lidar_frame_error"] = error
                        STATE["lidar_frame_seq"] += 1
                elif isinstance(request, tuple) and request[0] == "tune":
                    tune_errors = apply_tune(request[1] or {})
                    with STATE_LOCK:
                        STATE["tune_errors"] = list(tune_errors)
                        STATE["tune_seq"] += 1     # 回执：HTTP 线程靠它判断本批已执行
                elif request == "reset":
                    if movement_requested:
                        errors.append("请松开摇杆并取消导航后再复位")
                    elif not Bridge.call("ros_reset_drive", timeout=config.MCU_TIMEOUT_S):
                        errors.append("复位未完成：请等轮子停稳，检查驱动故障指示后重试")
                elif request == "stop":
                    # 先假定没停稳，拿到 True 再解除。异常时标志保持置位，
                    # 下一拍会继续重试，不会静默丢掉停车。
                    STOP_PENDING.set()
                    accepted = Bridge.call("ros_stop", timeout=config.MCU_TIMEOUT_S)
                    if accepted:
                        driving = False
                        STOP_PENDING.clear()
                    else:
                        driving = True
            except Exception as exc:
                errors.append(f"命令 {request} 失败：{exc}")

        push = tick % config.STATUS_EVERY_TICKS == 0

        # 雷达分片：排在指令之后，并且避开读状态的那一拍。
        # 一次 ros_status 约 13 ms、一片雷达约 12 ms，叠在同一拍会顶穿 33 ms 周期。
        # 失败只影响 /scan，不该污染底盘错误，但必须留痕（见 pump_scan）。
        #
        # 再加一道时间闸：本拍已经花掉 20 ms 就直接跳过这一片。雷达掉一片只是
        # /scan 晚 33 ms，而拖慢这一拍会连带推迟下一拍的心跳，顶到固件看门狗
        # 就是整车停下 —— 宁可丢一片点云。
        if not push and not handled and (time.monotonic() - started) < 0.020:
            pump_scan(started)
        fresh = False                             # 本拍是否真的读到了新状态
        if push:
            try:
                raw_status = PROBE.call("status", Bridge.call, "ros_status",
                                        timeout=config.MCU_TIMEOUT_S)
                if isinstance(raw_status, (list, tuple)) and len(raw_status) > 5:
                    with STATE_LOCK:
                        STATE["clock_ref"] = (raw_status[5], time.time())
                status = control.decode_status(raw_status)
                if status is not None:
                    mcu = status
                    mcu_at = time.monotonic()
                    fresh = True
                    # 顶死检测：占空比已经拉满、轮速却一直是 0，说明轮子被
                    # 顶住/卡住（或驱动没给电）。现场表现是"推杆没反应"，而
                    # 页面上 PWM 只是个小数字，看不出所以然。这里只提示、
                    # 不锁存、不停车 —— 松杆反向就能自己脱困。
                    wheels = status.get("wheels") or []
                    # 阈值 190 对应固件 pwmLimit=215 的 88%：固件已经把可用上限
                    # 压到 215（满占空比那一档驱动不输出），这里再用 240 判就永远
                    # 不会触发。
                    # 自检期间不要插手：自检本身就是"给一个目标、看它跟不跟"，
                    # 它只需要 2~3 s，而下面的断电重启会把自检直接掐断，
                    # 结果变成"自检失败"，其实是自己把自己打断的。
                    stuck = (not SELFTEST.active) and bool(wheels) and all(
                        abs(w.get("target", 0)) > 300 and abs(w.get("speed", 0)) < 120
                        and abs(w.get("pwm", 0)) >= 190 for w in wheels)
                    stuck_reads = stuck_reads + 1 if stuck else 0
                    if stuck_reads == 5:      # 状态 5 Hz，连续 5 帧 ≈ 1.0 s
                        report_ros_error(
                            "motor", "轮子被卡住：占空比已到上限但轮速一直是 0"
                                     "（顶到障碍/线束，或驱动进入保护）")
                    # 连续堵转就断电重启一次驱动：AT8833 的过流/过温保护是锁存的，
                    # 只有把 nSLEEP 拉低再拉高才会解除。固件在 ros_stop 里正是
                    # 这么做的，所以"停一拍再走"能把它救回来。
                    # 这个值必须比"一次正常动作"长得多，否则重启本身会把动作打断：
                    # 曾经压到 10 帧（2 s），结果自检（2.5 s）和导航起步都被它掐死，
                    # 表现成"车根本不动"。50 帧 = 10 s，正常加速/转向远用不到这么久。
                    if stuck_reads == 50:
                        try:
                            Bridge.call("ros_stop", timeout=config.MCU_TIMEOUT_S)
                            report_ros_error("motor", "连续堵转，已断电重启一次驱动")
                        except Exception as exc:
                            errors.append("驱动重启失败：%s" % exc)
                        stuck_reads = 0
                    # 轮子转角跟着编码器走，URDF 模型才能和实车对上态。
                    # 会话检测（epoch）在 publish_telemetry 内部先做。
                    publish_telemetry(status)
                    RECORDER.record(mcu, linear, angular)
                    PROBE.add("imu_age", status["imu"].get("age_ms"))
                else:
                    errors.append("状态解码失败：ros_status 数据不足 38 项")
            except Exception as exc:
                errors.append(f"读状态失败：{exc}")

        # 用 MCU 已执行的保向状态更新网页，缓存快照不参与轮速计算。
        imu = (mcu or {}).get("imu") or {}
        heading = (mcu or {}).get("heading") or {}
        mcu_stale = mcu_at <= 0.0 or time.monotonic() - mcu_at > MCU_STALE_S
        # 雷达统计低频刷新。雷达没启动或出错都不该影响底盘，异常一律吞掉。
        if tick % LIDAR_EVERY_TICKS == 0 and not (right or left):
            try:
                raw = Bridge.call("lidar_status", timeout=config.MCU_TIMEOUT_S)
                if isinstance(raw, (list, tuple)) and len(raw) >= 8:
                    with STATE_LOCK:
                        STATE["lidar"] = {
                            "state": raw[0], "frames": raw[1], "points": raw[2],
                            "errors": raw[3], "hz": raw[4] / 100.0,
                            "last_mm": raw[5], "min_mm": raw[6], "age_ms": raw[7],
                        }
            except Exception:
                pass

        # 参数快照定期刷新。MCU 失联时保留最后一次的值，由 /api/tune 的 stale 标出来，
        # 免得界面上那排数字看起来像刚从车上读回来的。
        if tick % TUNE_REFRESH_TICKS == 3 and not mcu_stale and not (right or left):
            try:
                refresh_tune()
            except Exception as exc:
                errors.append(f"读取调参失败：{exc}")
        imu_ready = (not mcu_stale and imu.get("state") == config.IMU_STATE_READY
                     and 0 <= imu.get("age_ms", 51) <= 50)
        heading_active = (hold_heading and imu_ready and (mcu or {}).get("mode") == 1
                          and heading.get("mode") == 1)
        # 网页原有正值表示减右轮、加左轮，MCU 修正量的符号与之相反。
        assist = -heading.get("correction_cps", 0) if heading_active else 0.0
        if PAUSED.is_set():
            assist_reason = "paused"
        elif mcu is None and time.monotonic() - boot_at <= STARTUP_GRACE_S:
            assist_reason = "starting"
        elif mcu_stale:
            assist_reason = "mcu_stale"
        elif not imu_ready:
            assist_reason = "imu_not_ready"
        elif abs(linear) <= config.ASSIST_LINEAR_MIN:
            assist_reason = "no_linear"
        elif not hold_heading:
            assist_reason = "turning"
        elif not heading_active:
            assist_reason = "starting"
        elif assist == 0.0:
            assist_reason = "deadband"
        else:
            assist_reason = "active"

        error = "；".join(errors)                 # 同一拍多个失败全部保留
        # 记录网页航向快照的缓存年龄，便于区分本地控制与显示更新的时间尺度。
        if mcu_at:
            PROBE.add("yaw_age", (time.monotonic() - mcu_at) * 1000.0)
        # 刚停车就把 MCU trace 拉出来：车已停，多一次 RPC 不会影响行驶。
        if was_driving and not driving:
            TRACE.request()
        was_driving = driving
        if TRACE.busy() and not (right or left) and (mcu is None or mcu["mode"] != 1):
            TRACE.pump(Bridge.call)
        # 内容变化或每 5 秒重发一次；这里不能清去重键，否则每个推流拍都会重报一遍
        if error and (error != logged_error or time.monotonic() - logged_at > 5.0):
            print(f"[control] {error}", flush=True)
            logged_error, logged_at = error, time.monotonic()

        with STATE_LOCK:
            STATE["targets"] = [right, left]
            STATE["accepted"] = accepted
            STATE["assist"] = round(assist, 0)
            STATE["assist_reason"] = assist_reason
            STATE["driving"] = driving
            STATE["cmd_source"] = command_source
            if error:
                STATE["last_error"] = error
                STATE["last_error_mono"] = time.monotonic()
            if fresh:                          # 只有新状态才刷新时间戳，否则 age_s 恒为 0
                STATE["mcu"] = mcu
                STATE["mcu_mono"] = time.monotonic()

        if push:
            text = ""
            try:
                payload = build_payload()               # 必须在 STATE 更新之后
            except Exception as exc:
                text = f"状态组装失败：{exc}"
            else:
                try:
                    UI.send_message("robot_status", payload)
                except Exception as exc:
                    text = f"推送失败：{exc}"
            # 两种失败都只该丢一帧界面，不能把控制线程带走
            if text and (text != push_error or time.monotonic() - push_at > 5.0):
                print(f"[main] {text}", flush=True)
                push_error, push_at = text, time.monotonic()

        # 导航帧：位姿/地图/路径由容器内的 nav_bridge 折成一条 JSON，
        # 这里只做转发。序号没变就不推，避免每秒十次无意义的 socket 编码。
        if tick % config.NAV_PUSH_TICKS == 0:
            frame, _age, seq = NAV.snapshot()
            nav_in_motion = frame.get("nav") in ("navigating", "canceling")
            if seq and seq != nav_seq:
                nav_seq = seq
                try:
                    UI.send_message("robot_nav", frame)
                except Exception as exc:
                    report_ros_error("nav", "导航状态推送失败：%s" % exc)

        PROBE.add("work", (time.monotonic() - started) * 1000.0)
        if config.LATENCY_ENABLED and PROBE.due():
            # 带上 MCU 的时间与序号：Linux 侧这一行能和 MCU trace 的 tick_ms/seq 对上。
            current = mcu or {}
            print(PROBE.report(f"uptime={current.get('uptime_ms', '-')} "
                               f"seq={current.get('sequence', '-')}"), flush=True)
        tick += 1
        QUIT.wait(max(0.0, period - (time.monotonic() - started)))


def main():
    """注册接口、启动控制线程并交给 App 框架。"""
    QUIT.clear()                       # 允许同进程内二次运行
    PAUSED.clear()                     # 新会话从"可遥控"开始，锁存不跨进程
    while True:                        # 丢掉上一轮残留命令
        try:
            REQUESTS.get_nowait()
        except queue.Empty:
            break

    UI.on_message("stop", socket_handlers.on_stop)
    UI.on_message("resume", socket_handlers.on_resume)
    UI.on_message("reset", socket_handlers.on_reset)
    UI.on_message("calibrate", socket_handlers.on_calibrate)
    UI.on_message("selftest", socket_handlers.on_selftest)
    UI.on_message("cmd_vel", socket_handlers.on_cmd_vel)
    UI.on_message("nav_cmd", socket_handlers.on_nav_cmd)
    UI.expose_api("GET", "/api/telemetry", api_routes.get_telemetry)
    UI.expose_api("GET", "/api/telemetry/full", api_routes.get_telemetry_full)
    UI.expose_api("GET", "/api/selftest", api_routes.get_selftest)
    UI.expose_api("GET", "/api/selftest/status", api_routes.get_selftest_status)
    UI.expose_api("GET", "/api/trace/dump", api_routes.get_trace_dump)
    UI.expose_api("GET", "/api/tune", api_routes.get_tune)
    UI.expose_api("GET", "/api/motor_diag", api_routes.get_motor_diag)
    UI.expose_api("GET", "/api/camera/status", api_routes.get_camera_status)
    UI.expose_api("GET", "/api/camera/stream", api_routes.get_camera_stream)
    UI.expose_api("GET", "/api/camera/snapshot", api_routes.get_camera_snapshot)
    UI.expose_api("GET", "/api/detect", api_routes.get_detect)
    UI.expose_api("POST", "/api/tune", api_routes.set_tune)
    UI.expose_api("GET", "/api/lidar", api_routes.get_lidar)
    UI.expose_api("GET", "/api/lidar/frame", api_routes.get_lidar_frame)
    UI.expose_api("GET", "/api/lidar/control", api_routes.get_lidar_control)

    LINK.start()
    SOURCE.start()
    JOY.start()
    WEB.start()
    NAV.start()
    publishers.start()
    # 摄像头是独立设备，起不来不该影响底盘控制：线程内部按 REOPEN_S 重试，
    # 失败原因通过 /api/camera/status 给页面。
    CAMERA.start()
    # 检测依赖推理容器，起不来时线程内部退避重试，不影响底盘与画面。
    DETECTOR.start()
    worker = threading.Thread(target=control_loop, name="control", daemon=True)
    worker.start()

    try:
        App.run()
    finally:
        QUIT.set()
        worker.join(timeout=2.0)
        LINK.stop()
        SOURCE.stop()
        JOY.stop()
        WEB.stop()
        NAV.stop()
        CAMERA.stop()
        DETECTOR.stop()
        publishers.stop()
        RECORDER.close()
        if worker.is_alive():
            # 线程可能仍在 Bridge 中，再发指令会两线程并发访问
            print("[main] 控制线程未按时退出，跳过最终停止指令", flush=True)
        else:
            try:
                Bridge.call("ros_stop", timeout=config.MCU_TIMEOUT_S)
            except Exception:
                pass


if __name__ == "__main__":
    main()
