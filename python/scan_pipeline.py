"""雷达分片拼帧：每拍最多读一片，拼齐且不过期才发布。

为什么要分片
    ros_status 的整帧读取在 Bridge 上是一次 RPC，180 个分区一次读完会让控制拍
    从 33 ms 撑到几百毫秒。拆成三片、每拍一片，最坏也只占一拍的三分之一。

为什么调用点必须排在指令之后
    雷达读得再慢，也不能让停车晚一拍。main 的控制环把 fetch_scan_slice
    放在驾驶指令与队列处理之后。

本模块只有控制线程会调用，所以内部状态不加锁。

时序约定
    * ``next_try`` 是"下一次允许尝试读分片"的时刻，用它统一做节流与退避。
      以前把 ``last_pub`` 直接写成 ``now + 2.0``（未来时刻），既是语义污染，
      也导致一次重复帧就静默停读 2.38 s —— 超过代价地图 expected_update_rate
      允许的间隔，Nav2 会判定观测过期而停止出速度。
    * ``last_pub`` 只记录**真正发布成功**的时刻，供统计用。
"""

import math
import time

from arduino.app_utils import Bridge

import config
import ros_publishers as publishers

LIDAR_FRAME_WAIT_S = 1.0
# 扫描分片：180 个 2° 分区分三片，每拍最多读一片，避免一次把控制拍撑爆。
LIDAR_SLICE = 60
LIDAR_SECTORS = 180
SCAN_PUBLISH_S = 0.38        # ≈2.6 Hz，首版只保证 2~3 Hz
SCAN_STALE_SLICES = 4        # 拼齐时最新帧比目标大这么多才判过期，给慢拍留余量
SCAN_IDLE_BACKOFF_S = 2.0    # **持续**停扫才用的长退避，偶发重复帧不走这里
SCAN_DUP_RETRY_S = 0.12      # 偶发重复帧的短重试间隔：差不多下一拍再读
SCAN_DUP_LIMIT = 3           # 连续这么多次重复才认为雷达真的停扫了
# 帧龄上限。拼帧要三拍，正常只滞后 100~150 ms；首次读到的帧若是停扫前留下的
# 旧帧，帧号仍是"最新"，只靠 latest-seq 判断不出来，必须按真实年龄拒。
SCAN_MAX_AGE_MS = 400
# 雷达状态 2 s 读一次。一次 ros_status 约 13 ms，控制拍只有 33 ms，
# 两次 RPC 会把那一拍挤爆，所以只按低频读统计，帧数据按需另取。
LIDAR_EVERY_TICKS = max(1, int(config.CONTROL_HZ * 2.0))


def _new_state():
    """拼帧状态。所有字段都在这里初始化为"空"，重置时整体换新更不易漏字段。"""
    return {
        "seq": 0,           # 正在拼的帧号，0 表示空闲
        "ranges": None,
        "meta": None,       # (seq, start_ms, period_ms, latest)
        "next": 0,          # 下一片应从哪个分区开始
        "next_try": 0.0,    # 下一次允许读分片的时刻
        "last_pub": 0.0,    # 最近一次**成功发布**的时刻
        "last_pub_at": 0.0,
        "clock": None,
        "last_seq": 0,      # 最近一次发布的帧号
        "dup_streak": 0,    # 连续重复帧计数
        "last_mcu_now": None,   # 上一次读到的 MCU uptime，用于识别重启
        "backoff_reason": "",
    }


SCAN_ASSEMBLY = _new_state()

# 拼帧统计。以前只有计数、看不出原因；现在细分到"哪一环出的问题"，
# 因为导航不动时最需要回答的就是"雷达到底是没数据，还是我们没去读"。
SCAN_STATS = {
    "slices": 0,              # 分片 RPC 次数
    "published": 0,           # 成功发布的帧数
    "stale": 0,               # 拼帧期间被新圈覆盖
    "missing": 0,             # 分片缺失（帧被覆盖 / 帧号偏移不符）
    "old": 0,                 # 帧龄超限被拒
    "errors": 0,              # Bridge 抛异常
    "duplicate_frames": 0,    # 读到与上次已发布帧号相同的帧
    "invalid_slices": 0,      # 参数越界、长度不符、数值非法
    "expired_frames": 0,      # 拼齐时按真实帧龄判为过期
    "rpc_errors": 0,          # 同 errors，语义化别名
    "session_resets": 0,      # 检测到 MCU 重启而清空拼帧状态
    "dup_backoff": 0,         # 因持续重复帧进入长退避
    "max_publish_gap_ms": 0,  # 两次成功发布之间的最大间隔
}


def _note(key, amount=1):
    if key in SCAN_STATS:
        SCAN_STATS[key] += amount


def reset_assembly(now=0.0, reason=""):
    """清空拼帧状态。MCU 重启、会话变更、通道异常时调用。"""
    global SCAN_ASSEMBLY
    old = SCAN_ASSEMBLY
    SCAN_ASSEMBLY = _new_state()
    # 保留节流位，避免刚重置就立刻读；其余全部丢弃。
    SCAN_ASSEMBLY["next_try"] = now
    if reason:
        SCAN_ASSEMBLY["backoff_reason"] = reason
        _note("session_resets")
    del old


def _valid_value(value):
    """分区值必须是有限的非负数（毫米）。NaN/inf/负值一律视为分片非法。"""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number) or number < 0.0:
        return None
    return number


def _fill_slice(state, offset, count, values):
    """把一片毫米值写进米制的 ranges。

    @return: 参数合法并已写入返回 True；否则返回 False 且**不修改** state
             （调用方据此丢弃整帧，避免把半帧无效数据发成扫描）。
    """
    if count <= 0 or offset < 0 or offset + count > LIDAR_SECTORS:
        return False
    if offset != state["next"]:
        return False
    if len(values) < count:
        return False
    converted = []
    for value in values[:count]:
        number = _valid_value(value)
        if number is None:
            return False
        converted.append(round(number / 1000.0, 3))
    for index, metre in enumerate(converted):
        state["ranges"][offset + index] = metre
    state["next"] = offset + count
    return True


def _read_slice(seq, offset, now):
    """读一片，返回解包后的元组；异常或长度不足返回 None。"""
    _note("slices")
    try:
        raw = Bridge.call("lidar_slice", seq, offset, LIDAR_SLICE,
                          timeout=config.MCU_TIMEOUT_S)
    except Exception:            # noqa: BLE001  抛异常也不能让控制拍崩
        _note("errors")
        _note("rpc_errors")
        return None
    if not isinstance(raw, (list, tuple)) or len(raw) < 9:
        _note("errors")
        _note("rpc_errors")
        return None
    return raw


def _check_mcu_restart(state, mcu_now):
    """MCU 重启会让 uptime 回到 0。用较大的回退量判断，避免计数回绕误判。"""
    previous = state["last_mcu_now"]
    if previous is not None and mcu_now + 1000 < previous:
        reset_assembly(0.0, "MCU 重启（uptime 回退）")
        return True
    state["last_mcu_now"] = mcu_now
    return False


def fetch_scan_slice(now):
    """每拍最多读一个雷达分片；拼齐且不过期才发布，否则整帧丢弃。

    调用点必须排在指令下发之后——雷达读得再慢，也不能让停车晚一拍。
    """
    state = SCAN_ASSEMBLY
    if now < state["next_try"]:
        return

    seq = state["seq"]
    if seq != 0:
        # 拼帧中：每拍允许再读一片。空闲态的 next_try 由下面各条出口各自决定，
        # 不能在读之前就推后 —— 否则第一片读成功后，第 2、3 片会被同一个
        # 发布周期节流挡住，三片永远凑不齐（首版测试就是这么暴露出来的）。
        state["next_try"] = now

    raw = _read_slice(seq, 0 if seq == 0 else state["next"], now)
    if raw is None:
        SCAN_ASSEMBLY["seq"] = 0
        SCAN_ASSEMBLY["next_try"] = now + (SCAN_PUBLISH_S if seq == 0
                                           else SCAN_PUBLISH_S)
        if seq != 0:
            _note("missing")
        return

    (rseq, start_ms, period_ms, _valid, sectors, offset, count,
     latest, mcu_now) = raw[:9]
    if _check_mcu_restart(state, mcu_now):
        return                    # 状态已重置，这一拍作废

    if sectors != LIDAR_SECTORS or count <= 0:
        _note("invalid_slices")
        SCAN_ASSEMBLY["seq"] = 0
        return

    if seq == 0:
        # ---- 空闲态读到的第一片 ----
        if offset != 0:
            _note("invalid_slices")
            state["next_try"] = now + SCAN_PUBLISH_S
            return
        if rseq == state["last_seq"]:
            # 帧号没前进。偶发重复只做短重试；连续重复才认为真的停扫。
            _note("duplicate_frames")
            if state["dup_streak"] + 1 >= SCAN_DUP_LIMIT:
                state["dup_streak"] = 0
                state["next_try"] = now + SCAN_IDLE_BACKOFF_S
                state["backoff_reason"] = ("雷达停扫：连续 %d 帧重复"
                                           % SCAN_DUP_LIMIT)
                _note("dup_backoff")
            else:
                state["dup_streak"] += 1
                state["next_try"] = now + SCAN_DUP_RETRY_S
            return
        state["dup_streak"] = 0
        if mcu_now - start_ms > SCAN_MAX_AGE_MS:
            # 停扫前留下的旧帧：帧号最新但已经放了很久，按真实年龄拒。
            _note("old")
            state["next_try"] = now + SCAN_PUBLISH_S
            return
        state["ranges"] = [0.0] * LIDAR_SECTORS
        state["next"] = 0
        state["meta"] = (rseq, start_ms, period_ms, latest)
        state["clock"] = (mcu_now, time.time())
        if not _fill_slice(state, offset, count, raw[9:]):
            _note("invalid_slices")
            SCAN_ASSEMBLY["seq"] = 0
            SCAN_ASSEMBLY["next_try"] = now + SCAN_PUBLISH_S
            return
        state["seq"] = rseq
        state["next_try"] = now        # 拼帧中：下一拍立刻可以读第二片
    else:
        # ---- 拼帧中的后续分片 ----
        if rseq != seq or offset != state["next"]:
            _note("missing")
            SCAN_ASSEMBLY["seq"] = 0
            SCAN_ASSEMBLY["next_try"] = now + SCAN_PUBLISH_S
            return
        meta_seq, start_ms0, _p0, _l0 = state["meta"]
        if rseq != meta_seq:          # 分片不属于同一帧
            _note("invalid_slices")
            SCAN_ASSEMBLY["seq"] = 0
            return
        state["clock"] = (mcu_now, time.time())   # 用最后一片时刻做发布基准最准
        if not _fill_slice(state, offset, count, raw[9:]):
            _note("invalid_slices")
            SCAN_ASSEMBLY["seq"] = 0
            return

    if state["next"] < LIDAR_SECTORS:
        return                        # 还没拼齐，下一拍继续

    # ---- 发布前的最终校验：全部按"此刻"的真实条件判，不看历史 ----
    meta_seq, start_ms, period_ms, latest = state["meta"]
    if meta_seq != seq:
        _note("invalid_slices")
        SCAN_ASSEMBLY["seq"] = 0
        return
    if latest - seq > SCAN_STALE_SLICES:
        # 拼的过程中已经过去好几圈：帧号能看出来，直接丢。
        SCAN_ASSEMBLY["seq"] = 0
        SCAN_ASSEMBLY["next_try"] = now + SCAN_PUBLISH_S
        _note("stale")
        return
    if mcu_now - start_ms > SCAN_MAX_AGE_MS:
        # 帧号差看不出来的情况：分片之间 RPC 抖动，拼齐时已经过期。
        SCAN_ASSEMBLY["seq"] = 0
        SCAN_ASSEMBLY["next_try"] = now
        _note("expired_frames")
        return

    try:
        publishers.publish_scan(start_ms, period_ms, state["ranges"],
                                LIDAR_SECTORS, state.get("clock"))
    except Exception:            # noqa: BLE001
        _note("errors")
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
    state["next_try"] = now + SCAN_PUBLISH_S
    state["backoff_reason"] = ""
    _note("published")


def status():
    """给 /api/lidar 的诊断快照。"""
    state = SCAN_ASSEMBLY
    snapshot = dict(SCAN_STATS)
    snapshot.update({
        "consecutive_duplicates": state["dup_streak"],
        "backoff_reason": state["backoff_reason"],
        "assembling": state["seq"] != 0,
        "assembled_slices": state["next"] // LIDAR_SLICE,
        "published_frames": SCAN_STATS["published"],
        "last_publish_at": state["last_pub_at"],
    })
    return snapshot
