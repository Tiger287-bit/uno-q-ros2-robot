"""WebUI socket 回调：网页按钮与键盘输入进来的地方。

这些回调跑在 socket 线程，只做"置标志 / 投递请求"，绝不直接调 Bridge——
Bridge 的调用方只有控制环一处，HTTP 或 socket 线程插进来会和 30 Hz 的
ros_drive 抢同一条 RPC 通道。
"""

import app_state
import config
import control

from app_state import (NAV, PAUSED, REQUESTS, SELFTEST,
                       STOP_PENDING, WEB)


def on_stop(sid=None, payload=None):
    """网页「急停」：锁存并投递停车请求。框架按 (sid, data) 回调。

    Event.set() 在 socket 线程里完成，控制环下一拍就能看到；紧接着的停车请求
    只是把 ros_stop 提前一拍，不依赖它也安全（锁存期间控制环自己会走停车分支）。
    """
    app_state.bump_estop()  # 与控制环共用代次，让急停前的自检请求作废
    PAUSED.set()
    # 立即置位，不等控制环：resume 会把 PAUSED 和自检取消标志一起清掉，
    # 只有这个标志能在"急停与解除挤在同一拍"时继续把车按在停车路径上。
    STOP_PENDING.set()
    # 取消自检要和锁存分开：stop 之后紧跟 resume 时，控制环可能一拍都没看到
    # PAUSED，而自检分支又会在队列处理之前 continue，队列里的 stop 也被跳过。
    SELFTEST.cancel()
    # 急停同时取消导航目标：否则解除锁定后 Nav2 会把车重新开向原目标。
    NAV.cancel()
    REQUESTS.put("stop")


def on_resume(sid=None, payload=None):
    """网页「解除锁定」：清除急停锁存，交回控制权。

    只清锁存，不主动起转 —— 车是否继续走由当时手柄/网页的实际输入决定。
    顺带放开自检阻断：急停前排队的自检请求要等到这一步才允许启动。
    """
    PAUSED.clear()
    SELFTEST.clear_cancel()


def on_reset(sid=None, payload=None):
    """网页「复位故障」：停车并清锁存故障。"""
    REQUESTS.put("reset")


def on_calibrate(sid=None, payload=None):
    """网页「校准航向」：投递校准请求。"""
    REQUESTS.put("calibrate")


def on_selftest(sid=None, payload=None):
    """网页触发底盘回归自检：payload 为用例名，必须架空车轮。

    只投递请求，不在这里碰 Bridge —— 下发与采样都在控制环里做，
    否则会和正常指令通道抢同一个底盘。
    """
    name = payload.get("case") if isinstance(payload, dict) else payload
    REQUESTS.put(("selftest", str(name or ""), app_state.ESTOP_SEQ))


def on_cmd_vel(sid=None, payload=None):
    """网页键盘/虚拟摇杆：限幅后发到 /cmd_vel_web，由容器内的 mux 仲裁。

    走 App 自己的 WebUI 通道转发，而不是让浏览器直连 rosbridge：9090 因此
    不必暴露到局域网，省掉一整片无鉴权的攻击面。

    这里不阻塞、不抛异常 —— 它是 socket 回调，出错会连带打断框架的消息派发。
    """
    data = payload if isinstance(payload, dict) else {}
    linear, angular = control.clamp_cmd(data.get("linear"), data.get("angular"))
    WEB.publish({
        "linear": {"x": round(linear, 4), "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": round(angular, 4)},
    }, ttl=config.CMD_TIMEOUT_S)


def on_nav_cmd(sid=None, payload=None):
    """网页的地图操作：下发目标 / 取消 / 重设初始位姿 / 切换模式。

    坐标是地图坐标系（米、弧度），由页面从画布像素换算好再发上来。
    这里只做数值合法性检查：NaN/Inf 或离谱的坐标直接丢，避免把
    Nav2 的目标端塞进一个无法规划的点后一直报错。
    """
    data = payload if isinstance(payload, dict) else {}
    command = str(data.get("cmd") or "")
    try:
        if command in ("goal", "pose"):
            x = float(data.get("x"))
            y = float(data.get("y"))
            yaw = float(data.get("yaw") or 0.0)
            if not (-100.0 < x < 100.0 and -100.0 < y < 100.0
                    and -10.0 < yaw < 10.0):
                return
            if command == "goal":
                NAV.goal(x, y, yaw)
            else:
                NAV.set_pose(x, y, yaw, req=data.get("req", 0))
        elif command == "cancel":
            NAV.cancel()
        elif command == "map":
            NAV.request_map()
        elif command == "mode":
            mode = str(data.get("mode") or "")
            if mode in ("nav", "slam"):
                # 切模式前先撤目标：建图模式下没有 Nav2，留着目标只会在
                # 切回导航时突然复活（navlink 的队列保证 cancel 先到）。
                NAV.cancel()
                NAV.set_mode(mode)
    except (TypeError, ValueError):
        return
