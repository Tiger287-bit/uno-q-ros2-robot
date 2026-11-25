"""雷达分片拼帧：每拍最多读一片，拼齐且不过期才发布。

为什么要分片
    ros_status 的整帧读取在 Bridge 上是一次 RPC，180 个分区一次读完会让控制拍
    从 33 ms 撑到几百毫秒。拆成三片、每拍一片，最坏也只占一拍的三分之一。

为什么调用点必须排在指令之后
    雷达读得再慢，也不能让停车晚一拍。main 的控制环把 fetch_scan_slice
    放在驾驶指令与队列处理之后。

本模块只有控制线程会调用，所以内部状态不加锁。
"""

import time

from arduino.app_utils import Bridge

import config
import ros_publishers as publishers

LIDAR_FRAME_WAIT_S = 1.0
# 扫描分片：180 个 2° 分区分三片，每拍最多读一片，避免一次把控制拍撑爆。
LIDAR_SLICE = 60
SCAN_PUBLISH_S = 0.38        # ≈2.6 Hz，首版只保证 2~3 Hz
SCAN_STALE_SLICES = 4        # 拼齐时最新帧比目标大这么多才判过期，给慢拍留余量
SCAN_IDLE_BACKOFF_S = 2.0    # 停扫后帧号不再前进，把重试间隔拉长，别空读分片占 Bridge
# 帧龄上限。拼帧要三拍，正常只滞后 100~150 ms；首次读到的帧若是停扫前留下的
# 旧帧，帧号仍是"最新"，只靠 latest-seq 判断不出来，必须按真实年龄拒。
SCAN_MAX_AGE_MS = 400
# 雷达状态 2 s 读一次。一次 ros_status 约 13 ms，控制拍只有 33 ms，
# 两次 RPC 会把那一拍挤爆，所以只按低频读统计，帧数据按需另取。
LIDAR_EVERY_TICKS = max(1, int(config.CONTROL_HZ * 2.0))


LIDAR_SECTORS = 180
SCAN_ASSEMBLY = {"seq": 0, "ranges": None, "meta": None, "next": 0,
                 "last_pub": 0.0, "clock": None, "last_seq": 0}
# 拼帧统计，用来核对"每拍最多一片"是否真的按预期频率发生。
# errors 记的是 Bridge 调用抛异常的次数（固件没应答、RPC 超时等）——这些
# 以前被控制环静默吞掉，/scan 停发时看不出任何线索。现在既计数也打日志，
# 并随 /api/lidar 一起返回。
SCAN_STATS = {"slices": 0, "published": 0, "stale": 0, "missing": 0,
              "old": 0, "errors": 0}


def _fill_slice(state, offset, count, values):
    """把一片毫米值写进米制的 ranges，并把游标推到下一个分区。"""
    for index, value in enumerate(values[:count]):
        target = offset + index
        if 0 <= target < LIDAR_SECTORS:
            state["ranges"][target] = round(float(value) / 1000.0, 3)
    state["next"] = offset + count


def fetch_scan_slice(now):
    """每拍最多读一个雷达分片；拼齐且不过期才发布，否则整帧丢弃。

    调用点必须排在指令下发之后——雷达读得再慢，也不能让停车晚一拍。
    缺片（帧被新圈覆盖）或拼的过程中已经过去好几圈，都直接放弃这一帧。
    """
    state = SCAN_ASSEMBLY
    seq = state["seq"]

    if seq == 0:
        if now - state["last_pub"] < SCAN_PUBLISH_S:
            return                       # 还没到发布间隔，这拍不占 Bridge
        SCAN_STATS["slices"] += 1
        raw = Bridge.call("lidar_slice", 0, 0, LIDAR_SLICE,
                          timeout=config.MCU_TIMEOUT_S)
        if not isinstance(raw, (list, tuple)) or len(raw) < 10:
            return
        seq, start_ms, period_ms, _valid, sectors, offset, count, latest, mcu_now = raw[:9]
        if count <= 0 or sectors != LIDAR_SECTORS:
            return
        if seq == state["last_seq"]:
            # 停扫：帧号没前进。第一片就能判断，别把后两片也白读了。
            state["last_pub"] = now + SCAN_IDLE_BACKOFF_S
            return
        if mcu_now - start_ms > SCAN_MAX_AGE_MS:
            # 这帧在 MCU 里已经放了太久（停扫前留下的），帧号却是最新的，
            # 只看帧号看不出来。按真实年龄丢掉，别把陈旧数据发成新扫描。
            SCAN_STATS["old"] += 1
            state["last_pub"] = now
            return
        state["seq"] = seq
        state["meta"] = (start_ms, period_ms, latest)
        state["clock"] = (mcu_now, time.time())
        state["ranges"] = [0.0] * LIDAR_SECTORS
        state["next"] = 0
        _fill_slice(state, offset, count, raw[9:])
    else:
        offset = state["next"]
        SCAN_STATS["slices"] += 1
        raw = Bridge.call("lidar_slice", seq, offset, LIDAR_SLICE,
                          timeout=config.MCU_TIMEOUT_S)
        if not isinstance(raw, (list, tuple)) or len(raw) < 10:
            state["seq"] = 0             # 帧被新圈覆盖，丢掉重来
            state["last_pub"] = now      # 退避：下一轮发布周期再试，别每拍重读
            SCAN_STATS["missing"] += 1
            return
        rseq, start_ms, period_ms, _valid, _sectors, roffset, count, latest, mcu_now = raw[:9]
        if rseq != seq or roffset != offset or count <= 0:
            state["seq"] = 0
            state["last_pub"] = now
            return
        state["meta"] = (start_ms, period_ms, latest)
        state["clock"] = (mcu_now, time.time())   # 用最后一片的时刻做发布基准最准
        _fill_slice(state, roffset, count, raw[9:])

    if state["next"] < LIDAR_SECTORS:
        return

    start_ms, period_ms, latest = state["meta"]
    if seq == state["last_seq"]:
        state["seq"] = 0
        state["last_pub"] = now + SCAN_IDLE_BACKOFF_S
        return
    if latest - seq > SCAN_STALE_SLICES:
        state["seq"] = 0                 # 拼的过程中已经过去好几圈
        state["last_pub"] = now          # 同样退避，不然会每拍重来
        SCAN_STATS["stale"] += 1
        return
    publishers.publish_scan(start_ms, period_ms, state["ranges"],
                            LIDAR_SECTORS, state.get("clock"))
    state["last_seq"] = seq
    SCAN_STATS["published"] += 1
    state["last_pub"] = now
    state["seq"] = 0
