"""HTTP 接口：网页与脚本读取状态、下发调参、取雷达帧。

全部是只读或"投递请求"，不直接驱动底盘。调参写请求会排队交给控制环执行，
HTTP 线程靠 tune_seq 判断这一批是否已经落地。
"""

import time

from arduino.app_utils import Bridge
from fastapi import HTTPException
from fastapi.responses import Response, StreamingResponse

import selftest
import telemetry as telemetry_module
import tune

import app_state
from app_state import (CAMERA, DETECTOR, PAUSED, RECORDER,
                       REQUESTS, SELFTEST, STATE, STATE_LOCK, TRACE)
from payload import TUNE_APPLY_WAIT_S, TUNE_STALE_S, _rate_hz
from scan_pipeline import LIDAR_FRAME_WAIT_S, SCAN_STATS

def get_telemetry():
    """REST：最近 2 帧，供页面兜底轮询。

    页面只读最后一行（首行是表头），整段回 300 帧是白烧流量：实测一次回包
    25 KB，兜底轮询 2 Hz 就是 180 MB/小时，而它恰恰只在 socket 断流的弱网
    场景下才启用。要历史数据请走 /api/telemetry/full。

    顺带回一个 paused：socket 断流、页面切到兜底轮询时，界面仍要能核对急停
    锁存状态，否则本地按钮显示"已锁定"、实际可能早就被别的页面解除了。
    """
    return {"samples": RECORDER.recent(2), "rate_hz": _rate_hz(),
            "paused": PAUSED.is_set()}


def get_telemetry_full(since_seq: str = ""):
    """REST：整个缓冲；带 since_seq 时只回该游标之后的新帧。

    游标比缓冲区还旧时，中间那段已经被环形缓冲挤掉，回包会带 truncated=true ——
    否则调用方拿到的是"看着连续、实际缺一段"的数据，无从发现漏采。
    参数按字符串接收再自行转换，非法值退化为全量，避免 FastAPI 抛 422。
    """
    status = RECORDER.status()
    limit = status["capacity"]
    try:
        cursor = int(since_seq)
    except (TypeError, ValueError):
        cursor = -1
    truncated = False
    if cursor < 0:
        samples = RECORDER.recent(limit)
    else:
        samples, truncated = RECORDER.since(cursor, limit)
    tail = samples[1:]
    # 游标取实际发出的末帧；空回包时既不能超过本会话已有序号，
    # 也不能把上一会话的游标回吐给客户端。
    last_seq = (tail[-1][telemetry_module.SEQ_COL] if tail
                else min(max(cursor, 0), status["last_seq"]))
    return {"samples": samples,
            "rate_hz": _rate_hz(),
            "session_id": status["session_id"],
            "last_seq": last_seq,
            # true 表示 (旧游标, 首帧) 之间那段已被挤掉，调用方应记为丢失
            "truncated": truncated,
            "paused": PAUSED.is_set()}


def get_selftest(case: str = ""):
    """REST：触发一次底盘回归自检，返回是否已排队。

    为什么需要这个口子：自检原本只能从网页点，现场用 SSH 采集时无法复现。
    这里只排队，真正下发仍在控制环里做；与网页消息走同一个队列，行为一致。
    """
    name = (case or "").strip()
    known = selftest.known_cases()
    if not name:
        return {"cases": known, "queued": None}
    if SELFTEST.active:
        return {"queued": None, "error": f"上一项自检尚未结束：{SELFTEST.case}"}
    if name not in known:
        return {"queued": None, "error": f"未知用例：{name}"}
    REQUESTS.put(("selftest", name, app_state.ESTOP_SEQ))
    return {"queued": name, "cases": known}


def get_selftest_status():
    """REST：自检当前状态与历史判定（含 detail 文本）。"""
    return SELFTEST.summary()


def get_trace_dump(capture: str = ""):
    """REST：查询最近一次 trace 导出结果；capture=1 时触发一次新导出。"""
    if str(capture).strip() in ("1", "true", "yes"):
        TRACE.request()
    return TRACE.summary()


def get_tune():
    """REST：调参面板需要的分组、范围、默认值与当前值。

    stale 表示这份快照不是刚从车上读回来的（MCU 失联或尚未读完），
    界面据此标灰，避免把陈旧数字当成实时参数。
    """
    with STATE_LOCK:
        snapshot = STATE["tune"]
        at = STATE["tune_at"]
    payload = tune.describe(snapshot)
    payload["stale"] = snapshot is None or time.monotonic() - at > TUNE_STALE_S
    return payload


def set_tune(payload: dict = None):
    """REST：写入一批参数（{"values": {...}}）或恢复默认（{"reset": true}）。

    写入必须由控制线程执行，这里只排队并等回执：HTTP 线程直接调 Bridge 会
    和 30 Hz 的 ros_drive 抢同一条 RPC 通道。等待上限 TUNE_APPLY_WAIT_S，
    超时不算失败，返回 applied=false 让前端稍后刷新即可。
    """
    if not isinstance(payload, dict):
        return {"ok": False, "applied": False, "errors": [],
                "error": "请求体必须是 JSON 对象，例如 {\"values\": {\"speed_kp\": 2.5}}"}
    with STATE_LOCK:
        before = STATE["tune_seq"]
    REQUESTS.put(("tune", payload))

    deadline = time.monotonic() + TUNE_APPLY_WAIT_S
    while time.monotonic() < deadline:
        with STATE_LOCK:
            if STATE["tune_seq"] != before:
                break
        time.sleep(0.02)

    with STATE_LOCK:
        applied = STATE["tune_seq"] != before
        snapshot = STATE["tune"]
        errors = list(STATE["tune_errors"])
    result = tune.describe(snapshot)
    result["applied"] = applied
    result["ok"] = applied and not errors
    result["errors"] = errors
    if not applied:
        result["error"] = "控制环未在 %.1f s 内执行，请稍后刷新" % TUNE_APPLY_WAIT_S
    return result


def get_lidar():
    """REST：雷达统计，由控制环每 2 s 刷新一次；附带 /scan 拼帧计数。"""
    with STATE_LOCK:
        status = STATE["lidar"]
    return {"status": status, "scan": dict(SCAN_STATS)}


def get_lidar_frame(seq: str = "0"):
    """REST：取一帧 180 个 2° 分区；seq=0 取最新，否则按序号。

    帧数据有 185 项，不在周期状态里传输，按需由控制环读一次再回。
    """
    try:
        index = max(0, int(seq))
    except (TypeError, ValueError):
        index = 0
    with STATE_LOCK:
        before = STATE["lidar_frame_seq"]
    REQUESTS.put(("lidar_frame", index))

    deadline = time.monotonic() + LIDAR_FRAME_WAIT_S
    while time.monotonic() < deadline:
        with STATE_LOCK:
            if STATE["lidar_frame_seq"] != before:
                break
        time.sleep(0.02)

    with STATE_LOCK:
        done = STATE["lidar_frame_seq"] != before
        frame = STATE["lidar_frame"]
        error = STATE["lidar_frame_error"]
    if not done:
        return {"ok": False, "error": "控制环未在 %.1f s 内响应" % LIDAR_FRAME_WAIT_S}
    if error:
        return {"ok": False, "error": error}
    if not frame or len(frame) < 5:
        return {"ok": False, "error": "帧不存在：序号太旧或雷达未启动"}
    sectors = int(frame[4])
    return {
        "ok": True,
        "seq": frame[0],
        "start_ms": frame[1],
        "period_ms": frame[2],
        "valid": frame[3],
        "sectors": sectors,
        "dist_mm": frame[5:5 + sectors],
    }


def get_lidar_control(op: str = ""):
    """REST：启动或停止雷达。op=start|stop。

    下发仍在控制线程里做，这里只排队等回执，避免 HTTP 线程与 30 Hz 控制拍抢 Bridge。
    """
    name = (op or "").strip()
    if name not in ("start", "stop"):
        return {"ok": False, "error": "op 只能是 start 或 stop"}
    with STATE_LOCK:
        before = STATE["lidar_frame_seq"]
    REQUESTS.put(("lidar_ctl", name))

    deadline = time.monotonic() + LIDAR_FRAME_WAIT_S
    while time.monotonic() < deadline:
        with STATE_LOCK:
            if STATE["lidar_frame_seq"] != before:
                break
        time.sleep(0.02)
    with STATE_LOCK:
        done = STATE["lidar_frame_seq"] != before
        error = STATE["lidar_frame_error"]
    if not done:
        return {"ok": False, "error": "控制环未在 %.1f s 内响应" % LIDAR_FRAME_WAIT_S}
    return {"ok": not error, "op": name, "error": error}


def get_motor_diag():
    """REST：四路 PWM 的上电自检结果，用于判断"PWM 给了但轮子不转"。

    自检在 initPwm() 里做一次，之后不再改动电机：驱动休眠时给四路输入输出
    已知占空比，再回读波形（周期、脉宽）和配对脚电平。passed=true 说明
    MCU 到驱动输入这一段是好的，问题在驱动输出级/电机/供电。
    """
    try:
        raw = Bridge.call("ros_motor_diag", timeout=1.0)
    except Exception as exc:
        return {"ok": False, "error": str(exc)}
    if not isinstance(raw, (list, tuple)) or len(raw) < 4:
        return {"ok": False, "error": "unexpected payload"}
    points = []
    # 固定 6 个自检点，末尾 3 项是实时电平（见 motion_bridge.rosMotorDiag）
    for index in range(min(6, max(0, (len(raw) - 7) // 5))):
        base = 4 + index * 5
        points.append({
            "pin": raw[base],
            "paired_ok": bool(raw[base + 1]),
            "cycles": raw[base + 2],
            "period_us": raw[base + 3],
            "high_us": raw[base + 4],
        })
    result = {
        "ok": True,
        "passed": bool(raw[0]),
        "driver_asleep": bool(raw[1]),
        "outputs_low": bool(raw[2]),
        "frequency_hz": raw[3],
        "points": points,
    }
    if len(raw) >= 7:
        levels = raw[-3]
        result["live"] = {
            # D12/D13 是 M1 的两路输入，D9/D10 是 M2 的两路输入。
            "d12": bool(levels & 1), "d13": bool(levels & 2),
            "d9": bool(levels & 4), "d10": bool(levels & 8),
            "awake": bool(raw[-2]),
            "fault_pin": bool(raw[-1]),
        }
    return result


# ---------------------------------------------------------------- 摄像头
def get_camera_status():
    """REST：摄像头状态（分辨率、实测帧率、客户端数、最近错误）。

    页面每 2 s 拉一次。MJPEG 的 <img> 在断流时不一定触发 onerror，
    所以"画面还在不在更新"要看这里的 age_s，而不是只依赖 onerror。
    """
    return CAMERA.status()


def get_camera_stream():
    """REST：MJPEG 流，直接给 <img src> 用。

    用 multipart/x-mixed-replace 是浏览器原生支持的格式，不需要 WebRTC
    或额外的播放器；代价是每帧都要完整 JPEG，没有帧间压缩。640x480@15
    在局域网里约 3~6 Mbit/s。
    """
    return StreamingResponse(
        CAMERA.frames(),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-store"},
    )


def get_camera_snapshot():
    """REST：单帧 JPEG，用于截图或 MJPEG 不可用时兜底。"""
    jpeg, _seq, _age = CAMERA.snapshot()
    if jpeg is None:
        raise HTTPException(status_code=503, detail="camera unavailable")
    return Response(content=jpeg, media_type="image/jpeg",
                    headers={"Cache-Control": "no-store"})


# ---------------------------------------------------------------- 物品检测
def get_detect():
    """REST：物品检测结果（官方 object_detection brick 推理）。

    检测频率由后端决定（实测推理一次 0.6~0.8 s，只取最新帧），页面按 2 Hz
    轮询即可；返回里的 width/height 是检测所用图像的尺寸，前端据此把
    像素坐标换算成画布坐标。
    """
    return DETECTOR.status()
