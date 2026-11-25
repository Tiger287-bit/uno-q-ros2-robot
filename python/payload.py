"""遥测载荷与调参：把控制环的状态翻成网页要的 JSON，以及参数读写封装。

_axis / _button 的容错不是洁癖：NaN 会让浏览器 JSON.parse 直接失败，
整页停止更新；摇杆原始值偶尔真的会出非有限数。
"""

import math
import time

from arduino.app_utils import Bridge

import config
import control
import tune

from app_state import JOY, LINK, PAUSED, RECORDER, STATE, STATE_LOCK, WEB

RATE_HZ = config.CONTROL_HZ / config.STATUS_EVERY_TICKS   # 遥测实际采样率，Hz
# 调参快照的刷新周期与新鲜度上限。参数只在网页上展示，不参与控制，
# 2 s 一次足够，免得为了显示再占一份 Bridge 带宽。
TUNE_REFRESH_S = 2.0
TUNE_STALE_S = 8.0
TUNE_APPLY_WAIT_S = 0.8  # POST /api/tune 等待控制环回执的上限，s
TUNE_REFRESH_TICKS = max(1, int(config.CONTROL_HZ * TUNE_REFRESH_S))


def _body(wheels):
    """双轮 cps 换算车体 (线速度 m/s, 角速度 rad/s)，数据不足返回 (None, None)。"""
    if not wheels or len(wheels) < 2:
        return None, None
    right, left = wheels[0]["speed"], wheels[1]["speed"]
    return (round((right + left) / 2 / config.COUNTS_PER_METER, 4),
            round((right - left) / config.COUNTS_PER_METER / config.TRACK_M, 3))


def _axis(value):
    """单个摇杆轴值；非法或非有限返回 None，NaN 会让浏览器 JSON.parse 直接失败。"""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return round(number, 3) if math.isfinite(number) else None


def _button(value):
    """按键状态统一成 0/1，异常值按未按下处理。"""
    try:
        return 1 if int(value) else 0
    except (TypeError, ValueError):
        return 0


def _rate_hz():
    """实测采样率，样本不足时回落到标称值。"""
    measured = RECORDER.rate_hz()
    return measured if measured is not None else round(RATE_HZ, 2)


def build_payload():
    """汇总链路、指令与 MCU 状态，供网页渲染。"""
    with STATE_LOCK:
        mcu = STATE["mcu"]
        mcu_mono = STATE["mcu_mono"]
        targets = list(STATE["targets"])
        accepted = STATE["accepted"]
        assist = STATE["assist"]
        assist_reason = STATE["assist_reason"]
        driving = STATE["driving"]
        cmd_source = STATE["cmd_source"]
        last_error = STATE["last_error"]
        last_error_mono = STATE["last_error_mono"]

    message, age, _ = LINK.snapshot()
    linear, angular = control.cmd_from(message)
    joy_message, joy_age, joy_connected = JOY.snapshot()

    # 错误按时间衰减：2 秒内算"正在错"，10 秒内仍可回溯，避免只在推送拍才可见。
    error_age = round(time.monotonic() - last_error_mono, 1) if last_error_mono else None
    error_active = error_age is not None and error_age < config.ERROR_ACTIVE_S

    cmd_right, cmd_left = targets
    actual_linear, actual_angular = _body(mcu["wheels"] if mcu else None)

    return {
        "link": dict(LINK.stats(), stale=age > config.CMD_TIMEOUT_S),
        # 网页速度输入的物理上限，前端据此缩放滑块与摇杆，不必写死数值
        "limits": {"linear": round(control.VMAX_M_S, 4),
                   "angular": config.MAX_ANGULAR},
        "web": WEB.stats(),
        "cmd": {"linear": round(linear, 4), "angular": round(angular, 4)},
        # 当前指令来源：手柄 / 网页 / 导航（mux 的仲裁结果），空串表示停车
        "cmd_source": cmd_source,
        "targets": targets,
        # commanded 为限幅后目标，actual 为 MCU 实测，差值即限幅损失
        "commanded": {
            "linear": round((cmd_right + cmd_left) / 2 / config.COUNTS_PER_METER, 4),
            "angular": round((cmd_right - cmd_left) / config.COUNTS_PER_METER
                             / config.TRACK_M, 3),
        },
        "actual": {
            "linear": actual_linear,
            "angular": actual_angular,
            "age_s": round(time.monotonic() - mcu_mono, 2) if mcu_mono else None,
        },
        "accepted": accepted,
        "assist": assist,
        # starting / paused / mcu_stale / imu_not_ready / no_linear / turning /
        # deadband / no_headroom / active
        "assist_reason": assist_reason,
        "driving": driving,
        # 界面靠它对齐"实际下发的锁存状态"，而不是各页面自己的按钮状态
        "paused": PAUSED.is_set(),
        "error": last_error if error_active else "",
        "last_error": (last_error if error_age is not None
                       and error_age < config.ERROR_HISTORY_S else ""),
        "error_age_s": error_age,
        "error_active": error_active,
        "rate_hz": _rate_hz(),
        "rate_nominal_hz": round(RATE_HZ, 2),
        # axes 项可能为 None（原始值非有限），buttons 恒为 0/1
        "raw": {
            "axes": [_axis(v) for v in (joy_message.get("axes") or [])],
            "buttons": [_button(v) for v in (joy_message.get("buttons") or [])],
            "age_s": None if math.isinf(joy_age) else round(joy_age, 3),
            "connected": joy_connected,
        },
        "telemetry": RECORDER.status(),
        "mcu": mcu,
    }


def refresh_tune():
    """从 MCU 读回全部运行期参数并更新快照。只在控制线程里调用。"""
    snapshot = tune.decode(Bridge.call("ros_tune_get", timeout=config.MCU_TIMEOUT_S))
    if snapshot is None:
        return None
    with STATE_LOCK:
        STATE["tune"] = snapshot
        STATE["tune_at"] = time.monotonic()
    return snapshot


def apply_tune(payload):
    """在控制线程里执行一批调参请求，返回错误列表。

    必须在控制线程里跑：Bridge 的调用方一共只有这一处，HTTP 线程插进来
    会和 30 Hz 的 ros_drive 抢同一条 RPC 通道。
    """
    errors = []
    if payload.get("reset"):
        try:
            Bridge.call("ros_tune_reset", timeout=config.MCU_TIMEOUT_S)
        except Exception as exc:                     # 超时或 RPC 失败
            return [{"key": "", "error": "恢复默认失败：%s" % exc}]
        refresh_tune()
        return errors

    values = payload.get("values")
    if not isinstance(values, dict) or not values:
        return [{"key": "", "error": "没有要写入的参数"}]
    for key, value in values.items():
        code, error = tune.encode(key, value)
        if error:
            errors.append({"key": key, "error": error})
            continue
        index = tune.BY_KEY[key].index
        try:
            accepted = Bridge.call("ros_tune_set", index, code,
                                   timeout=config.MCU_TIMEOUT_S)
        except Exception as exc:
            errors.append({"key": key, "error": "%s 写入失败：%s" % (key, exc)})
            continue
        # 固件对越界值返回 False（不是抛异常），必须显式判，否则界面会显示成功。
        if accepted is False:
            errors.append({"key": key, "error": "%s 被固件拒绝（超出允许范围）" % key})
    refresh_tune()
    return errors


# 雷达拼帧状态。只有控制线程碰它，所以不加锁。
