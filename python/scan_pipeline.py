"""雷达分片拼帧：每拍最多读一片，拼齐且校验通过才发布。

为什么要分片
    ros_status 的整帧读取在 Bridge 上是一次 RPC，180 个分区一次读完会让控制拍
    从 33 ms 撑到几百毫秒。拆成三片、每拍一片，最坏也只占一拍的三分之一。

为什么调用点必须排在指令之后
    雷达读得再慢，也不能让停车晚一拍。main 的控制环把 fetch_scan_slice
    放在驾驶指令与队列处理之后。

本模块只有控制线程会调用，所以内部状态不加锁。

时序约定
    * ``next_try`` 是"下一次允许尝试读分片"的时刻，统一做节流与退避。
      ``last_pub`` 只记录**真正发布成功**的时刻，不写未来值。
    * 固定元数据（seq / start_ms / period_ms / valid / sectors）在拼帧期间不可变，
      动态量（latest / mcu_now）每片都更新 —— 过期判断必须用最后一片读到的最新
      帧号，用第一片的会让 latest-seq 永远为 0，过期检查形同虚设。
"""

import math
import time

from arduino.app_utils import Bridge

import config
import ros_publishers as publishers

LIDAR_FRAME_WAIT_S = 1.0
LIDAR_SLICE = 60
LIDAR_SECTORS = 180

SCAN_PUBLISH_S = 0.38        # ≈2.6 Hz，首版只保证 2~3 Hz
SCAN_STALE_SLICES = 4        # 最新帧号比本帧大这么多就判过期
SCAN_IDLE_BACKOFF_S = 2.0    # **确认**停扫后的长退避
SCAN_DUP_RETRY_S = 0.12      # 偶发重复帧的短重试
SCAN_DUP_LIMIT = 3           # 连续重复这么多次，且帧确实变旧，才判停扫
SCAN_MAX_AGE_MS = 400        # 帧龄上限
# 固件协议：距离只能是 0（无回波）或 50~12000 mm。超出即视为分片损坏。
SCAN_MIN_DIST_MM = 50
SCAN_MAX_DIST_MM = 12000
# lidar::start() 会 ++epoch 并把帧号从头开始；uptime 却不回退。帧号比上次发布的
# 小这么多就认为雷达重新启动过（10 Hz 下正常只会递增，32 位回绕要 13 年）。
SCAN_SEQ_REWIND = 5
# MCU uptime 走 int32 传输，接近上限后回绕到 0。留 60 s 窗口判定为回绕而非重启。
INT32_WRAP = 2 ** 31
SCAN_WRAP_WINDOW_MS = 60000

LIDAR_EVERY_TICKS = max(1, int(config.CONTROL_HZ * 2.0))


def _new_state():
    return {
        "seq": 0,
        "ranges": None,
        "meta": None,        # (seq, start_ms, period_ms, valid, sectors) 固定
        "dynamic": None,     # {"latest": int, "mcu_now": int} 每片更新
        "next": 0,
        "next_try": 0.0,
        "last_pub": 0.0,     # 最近一次**成功发布**的时刻
        "last_pub_at": 0.0,
        "clock": None,
        "last_seq": 0,       # 最近一次发布的帧号
        "dup_streak": 0,
        "last_mcu_now": None,
        "backoff_reason": "",
        "backoff_until": 0.0,
        "last_error": "",    # 最近一次 RPC/校验失败原因，给 /api/lidar
        "last_frame_age_ms": -1,
    }


SCAN_ASSEMBLY = _new_state()

SCAN_STATS = {
    "slices": 0,
    "published": 0,
    "stale": 0,
    "missing": 0,
    "old": 0,
    "errors": 0,
    "duplicate_frames": 0,
    "invalid_slices": 0,
    "expired_frames": 0,
    "rpc_errors": 0,
    "session_resets": 0,
    "lidar_restarts": 0,
    "dup_backoff": 0,
    "no_echo_frames": 0,      # valid==0，整圈没回波，不作为导航观测发布
    "valid_mismatch": 0,      # 非零分区数与 valid 不一致
    "max_publish_gap_ms": 0,
}


def _note(key, amount=1):
    if key in SCAN_STATS:
        SCAN_STATS[key] += amount


def reset_assembly(now=0.0, reason=""):
    """清空拼帧状态。MCU 重启、雷达重启、会话变更时调用。"""
    global SCAN_ASSEMBLY
    SCAN_ASSEMBLY = _new_state()
    SCAN_ASSEMBLY["next_try"] = now
    if reason:
        SCAN_ASSEMBLY["backoff_reason"] = reason
        SCAN_ASSEMBLY["last_error"] = reason
        _note("session_resets")


def on_mcu_session_change(now, epoch_note=""):
    """给 main.py 的 track_mcu_session() 在检测到新会话时调用。"""
    reset_assembly(now, "MCU 会话变更 %s" % (epoch_note or ""))


def _int(value, default=None):
    """元数据字段必须是整数；bool 也是 int 的子类，这里一并当非法。"""
    if isinstance(value, bool):
        return default
    if isinstance(value, int):
        return value
    if isinstance(value, float) and math.isfinite(value) and value.is_integer():
        return int(value)
    return default


def _valid_distance(value):
    """返回毫米值；协议不允许的值返回 None。0 表示无回波，是合法值。"""
    number = _int(value)
    if number is None or number < 0:
        return None
    if number == 0:
        return 0
    if number < SCAN_MIN_DIST_MM or number > SCAN_MAX_DIST_MM:
        return None
    return number


def _fill_slice(state, offset, count, values):
    """把一片毫米值写进米制 ranges。任何校验失败都返回 False 且不修改 state。"""
    if count <= 0 or count > LIDAR_SLICE:
        return False
    if offset < 0 or offset + count > LIDAR_SECTORS:
        return False
    if offset != state["next"]:
        return False
    # 严格相等：声明 60 却只给 30，不能把剩下的当成 0 米发出去。
    if len(values) != count:
        return False
    converted = []
    for value in values:
        mm = _valid_distance(value)
        if mm is None:
            return False
        converted.append(round(mm / 1000.0, 3))
    for index, metre in enumerate(converted):
        state["ranges"][offset + index] = metre
    state["next"] = offset + count
    return True


def _read_slice(seq, offset, now):
    _note("slices")
    try:
        raw = Bridge.call("lidar_slice", seq, offset, LIDAR_SLICE,
                          timeout=config.MCU_TIMEOUT_S)
    except Exception as exc:     # noqa: BLE001
        _note("errors")
        _note("rpc_errors")
        SCAN_ASSEMBLY["last_error"] = "lidar_slice 异常: %s" % exc
        return None
    if not isinstance(raw, (list, tuple)) or len(raw) < 10:
        _note("errors")
        _note("rpc_errors")
        SCAN_ASSEMBLY["last_error"] = "lidar_slice 返回长度不足"
        return None
    return raw


def _is_int32_wrap(previous, current):
    return (previous > INT32_WRAP - SCAN_WRAP_WINDOW_MS
            and current < SCAN_WRAP_WINDOW_MS)


def _check_session(state, mcu_now, rseq):
    """识别 MCU 重启（uptime 回退）与雷达重启（帧号回退且 uptime 未退）。"""
    previous = state["last_mcu_now"]
    if (previous is not None and mcu_now < previous
            and not _is_int32_wrap(previous, mcu_now)):
        reset_assembly(0.0, "MCU 重启（uptime 回退 %d->%d）" % (previous, mcu_now))
        return True
    last_seq = state["last_seq"]
    if last_seq > 0 and rseq + SCAN_SEQ_REWIND < last_seq:
        reset_assembly(0.0, "雷达重启（帧号 %d->%d）" % (last_seq, rseq))
        _note("lidar_restarts")
        return True
    state["last_mcu_now"] = mcu_now
    return False


def fetch_scan_slice(now):
    """每拍最多读一个雷达分片；拼齐且全部校验通过才发布。"""
    state = SCAN_ASSEMBLY
    if now < state["next_try"]:
        return

    seq = state["seq"]
    if seq != 0:
        state["next_try"] = now      # 拼帧中：每拍一片

    offset_wanted = 0 if seq == 0 else state["next"]
    raw = _read_slice(seq, offset_wanted, now)
    if raw is None:
        SCAN_ASSEMBLY["seq"] = 0
        SCAN_ASSEMBLY["next_try"] = now + SCAN_PUBLISH_S
        if seq != 0:
            _note("missing")
        return

    rseq = _int(raw[0])
    start_ms = _int(raw[1])
    period_ms = _int(raw[2])
    valid = _int(raw[3])
    sectors = _int(raw[4])
    offset = _int(raw[5])
    count = _int(raw[6])
    latest = _int(raw[7])
    mcu_now = _int(raw[8])
    if None in (rseq, start_ms, period_ms, valid, sectors, offset, count,
                latest, mcu_now):
        _note("invalid_slices")
        SCAN_ASSEMBLY["last_error"] = "元数据非整数"
        SCAN_ASSEMBLY["seq"] = 0
        SCAN_ASSEMBLY["next_try"] = now + SCAN_PUBLISH_S
        return

    if _check_session(state, mcu_now, rseq):
        return

    if sectors != LIDAR_SECTORS or count <= 0 or count > LIDAR_SLICE:
        _note("invalid_slices")
        SCAN_ASSEMBLY["last_error"] = "sectors/count 非法 %s/%s" % (sectors, count)
        SCAN_ASSEMBLY["seq"] = 0
        SCAN_ASSEMBLY["next_try"] = now + SCAN_PUBLISH_S
        return
    if not (0 <= valid <= LIDAR_SECTORS):
        _note("invalid_slices")
        SCAN_ASSEMBLY["last_error"] = "valid 越界 %d" % valid
        SCAN_ASSEMBLY["seq"] = 0
        SCAN_ASSEMBLY["next_try"] = now + SCAN_PUBLISH_S
        return

    if seq == 0:
        # ---------------- 第一片：建立固定元数据 ----------------
        if offset != 0:
            _note("invalid_slices")
            SCAN_ASSEMBLY["seq"] = 0
            SCAN_ASSEMBLY["next_try"] = now + SCAN_PUBLISH_S
            return
        if rseq == state["last_seq"]:
            _note("duplicate_frames")
            frame_age = mcu_now - start_ms
            state["last_frame_age_ms"] = frame_age
            state["dup_streak"] += 1
            if (state["dup_streak"] >= SCAN_DUP_LIMIT
                    and frame_age > SCAN_PUBLISH_S * 1000.0):
                # 连续重复 **且** 这一帧本身已经不年轻，才判停扫。
                state["next_try"] = now + SCAN_IDLE_BACKOFF_S
                state["backoff_until"] = now + SCAN_IDLE_BACKOFF_S
                state["backoff_reason"] = (
                    "雷达停扫：连续 %d 帧重复，帧龄 %d ms"
                    % (state["dup_streak"], frame_age))
                _note("dup_backoff")
            else:
                state["next_try"] = now + SCAN_DUP_RETRY_S
            return
        state["dup_streak"] = 0
        state["backoff_reason"] = ""
        if mcu_now - start_ms > SCAN_MAX_AGE_MS:
            _note("old")
            state["last_frame_age_ms"] = mcu_now - start_ms
            state["last_error"] = "首片帧龄 %d ms 超限" % (mcu_now - start_ms)
            state["next_try"] = now + SCAN_PUBLISH_S
            return
        state["ranges"] = [0.0] * LIDAR_SECTORS
        state["next"] = 0
        state["meta"] = (rseq, start_ms, period_ms, valid, sectors)
        state["dynamic"] = {"latest": latest, "mcu_now": mcu_now}
        state["clock"] = (mcu_now, time.time())
        if not _fill_slice(state, offset, count, raw[9:]):
            _note("invalid_slices")
            state["last_error"] = "首片数据校验失败"
            SCAN_ASSEMBLY["seq"] = 0
            SCAN_ASSEMBLY["next_try"] = now + SCAN_PUBLISH_S
            return
        state["seq"] = rseq
        state["next_try"] = now
    else:
        # ---------------- 后续分片：固定元数据必须完全一致 ----------------
        meta = state["meta"]
        fixed_ok = (rseq == meta[0] and start_ms == meta[1]
                    and period_ms == meta[2] and valid == meta[3]
                    and sectors == meta[4])
        if not fixed_ok or offset != state["next"]:
            _note("invalid_slices")
            state["last_error"] = (
                "分片元数据不一致 seq=%s start=%s period=%s valid=%s/%s"
                % (rseq, start_ms, period_ms, valid, meta[3]))
            SCAN_ASSEMBLY["seq"] = 0
            SCAN_ASSEMBLY["next_try"] = now + SCAN_PUBLISH_S
            return
        # 动态量：每片都更新，发布时用**最后一片**的最新帧号与 MCU 时间
        state["dynamic"]["latest"] = latest
        state["dynamic"]["mcu_now"] = mcu_now
        state["clock"] = (mcu_now, time.time())
        if not _fill_slice(state, offset, count, raw[9:]):
            _note("invalid_slices")
            state["last_error"] = "分片数据校验失败 offset=%d" % offset
            SCAN_ASSEMBLY["seq"] = 0
            SCAN_ASSEMBLY["next_try"] = now + SCAN_PUBLISH_S
            return

    if state["next"] < LIDAR_SECTORS:
        return

    # ---------------- 发布前最终校验 ----------------
    meta_seq, start_ms, period_ms, valid, _sectors = state["meta"]
    latest = state["dynamic"]["latest"]
    mcu_now = state["dynamic"]["mcu_now"]
    state["last_frame_age_ms"] = mcu_now - start_ms

    if meta_seq != seq:
        _note("invalid_slices")
        SCAN_ASSEMBLY["seq"] = 0
        return
    if latest - seq > SCAN_STALE_SLICES:
        _note("stale")
        state["last_error"] = "过期：latest-seq=%d" % (latest - seq)
        SCAN_ASSEMBLY["seq"] = 0
        SCAN_ASSEMBLY["next_try"] = now + SCAN_PUBLISH_S
        return
    if mcu_now - start_ms > SCAN_MAX_AGE_MS:
        _note("expired_frames")
        state["last_error"] = "帧龄 %d ms 超限" % (mcu_now - start_ms)
        SCAN_ASSEMBLY["seq"] = 0
        SCAN_ASSEMBLY["next_try"] = now
        return
    # valid 是本圈有回波的分区数，必须与实际非零分区数一致，否则数据是坏的。
    nonzero = sum(1 for value in state["ranges"] if value > 0.0)
    if nonzero != valid:
        _note("valid_mismatch")
        state["last_error"] = "valid 与实际有效分区不符 %d!=%d" % (valid, nonzero)
        SCAN_ASSEMBLY["seq"] = 0
        SCAN_ASSEMBLY["next_try"] = now + SCAN_PUBLISH_S
        return
    if valid == 0:
        # 整圈没有回波：不算硬件故障，但不能作为导航障碍物观测发布。
        _note("no_echo_frames")
        state["last_error"] = "整圈无回波（valid=0），不发布"
        SCAN_ASSEMBLY["seq"] = 0
        SCAN_ASSEMBLY["next_try"] = now + SCAN_PUBLISH_S
        return

    try:
        publishers.publish_scan(start_ms, period_ms, state["ranges"],
                                LIDAR_SECTORS, state.get("clock"))
    except Exception as exc:     # noqa: BLE001
        _note("errors")
        SCAN_ASSEMBLY["last_error"] = "publish_scan 异常: %s" % exc
        SCAN_ASSEMBLY["seq"] = 0
        return

    if state["last_pub"] > 0.0:
        gap_ms = int((now - state["last_pub"]) * 1000.0)
        if gap_ms > SCAN_STATS["max_publish_gap_ms"]:
            SCAN_STATS["max_publish_gap_ms"] = gap_ms
    state["last_pub"] = now
    state["last_pub_at"] = time.time()
    state["last_seq"] = seq
    state["seq"] = 0
    state["next"] = 0
    state["ranges"] = None
    state["meta"] = None
    state["dynamic"] = None
    state["next_try"] = now + SCAN_PUBLISH_S
    state["backoff_reason"] = ""
    state["backoff_until"] = 0.0
    _note("published")


def status():
    """诊断快照。由 /api/lidar 直接返回，保证与拼帧同一份数据。"""
    state = SCAN_ASSEMBLY
    snapshot = dict(SCAN_STATS)
    snapshot.update({
        "consecutive_duplicates": state["dup_streak"],
        "backoff_reason": state["backoff_reason"],
        "backoff_remaining_s": max(0.0, round(state["backoff_until"]
                                              - time.monotonic(), 2)),
        "assembling": state["seq"] != 0,
        "assembled_slices": state["next"] // LIDAR_SLICE,
        "published_frames": SCAN_STATS["published"],
        "last_publish_at": state["last_pub_at"],
        "last_frame_age_ms": state["last_frame_age_ms"],
        "last_error": state["last_error"],
        # 语义澄清：publish_scan() 只是投给本地 Publisher 的队列，
        # 不代表 WebSocket 已发出，也不代表 Nav2 已收到。
        "published_means": "local_publisher_enqueue",
    })
    return snapshot
