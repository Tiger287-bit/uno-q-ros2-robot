"""跨线程共享的状态与单例。

为什么单独成文件
    main.py 里最纠缠的部分不是控制逻辑，而是"谁在改什么"：
    控制线程写 STATE、socket 线程改 PAUSED、HTTP 线程读 STATE、
    控制线程从 REQUESTS 取命令。这些对象被七八个模块共用，
    放在 main.py 里就必然出现 `from main import ...` 的循环导入。

    所有可变状态集中在这里，其他模块只 import 本文件，不再互相依赖。
    读写规则不变：STATE 必须持 STATE_LOCK，Event 本身线程安全。

生命周期
    本模块只做定义，不做初始化。启动顺序仍由 main() 控制。
"""

import os
import queue
import threading

from arduino.app_bricks.web_ui import WebUI
from ros2_bridge import Ros2Publisher, Ros2Subscriber

import camera
import config
import detector
import navlink
import selftest
import telemetry as telemetry_module
import trace as trace_module

# 退出信号：main 结束时置位，控制环每拍检查。
QUIT = threading.Event()

# 控制环与网页共享的一帧状态。写方只有控制循环，读方是 HTTP 线程。
STATE_LOCK = threading.Lock()
STATE = {
    "mcu": None, "mcu_mono": 0.0,
    "targets": [0, 0], "accepted": None,
    "assist": 0.0, "assist_reason": "idle",
    "driving": False,
    "cmd_source": "",
    "last_error": "", "last_error_mono": 0.0,
    # 调参：tune 是最近一次读到的物理值，tune_seq 每执行完一批写请求 +1，
    # HTTP 线程靠它判断"我这批改完没有"。
    "tune": None, "tune_at": 0.0, "tune_seq": 0, "tune_errors": [],
    # 雷达：status 由控制环周期刷新；frame 按请求取一次，用 frame_seq 做回执。
    "lidar": None, "lidar_frame": None, "lidar_frame_seq": 0,
    "lidar_frame_error": "",
    # MCU 单调时钟与墙上时钟的对应点 (uptime_ms, wall_time)，用来把雷达帧的
    # startMs 换算成真实采集时刻。每读一次 ros_status 刷新。
    "clock_ref": None,
}

# 网页 → 控制线程的命令队列。每拍只取一个，避免一次清空把几个 Bridge 调用
# 串在同一拍，把驾驶指令挤到 33 ms 之后。
REQUESTS = queue.Queue()

# 急停锁存。必须放在控制环这一层，而不是只锁浏览器：/cmd_vel_joy（手柄）与
# /cmd_vel_web（网页）都汇到容器内 mux 的 /cmd_vel，而消费 /cmd_vel 并调用
# ros_drive_heading 的地方只有控制环一处。只锁浏览器时，按住使能键推着摇杆再按
# 急停，车会停 SETTLE_MS(300 ms) 后自己接着跑，页面却仍显示"已锁定"。
PAUSED = threading.Event()

# 停车未确认。置位期间强制零目标，只重试 ros_stop，不发任何驾驶指令——
# 否则解除锁定后摇杆一推，drive 会抢在停车重试前面把车重新开起来。
STOP_PENDING = threading.Event()

# 急停世代号。每次急停自增；自检请求入队时带上当时的值，处理时不匹配就作废。
# 单靠 SELFTEST.cancel() 挡不住"急停与解除锁定挤在同一拍"——那种情况下
# 取消标志已经被 resume 清掉，队列里那条旧请求照样会启动。
ESTOP_SEQ = 0

# MCU 状态帧龄上限，约 2~3 个推流周期，超时即视为失联。
MCU_STALE_S = 0.5
# 上电后仍无状态就报"启动中"，超过该时长才判定失联。
STARTUP_GRACE_S = 3.0

# ---------- 单例 ----------

LINK = Ros2Subscriber(topic="/cmd_vel", message_type="geometry_msgs/msg/Twist")
# 仲裁当前来源（mux 以 2 Hz 心跳重发）。导航指令不做轮速吸附，详见 control.wheel_targets。
SOURCE = Ros2Subscriber(topic="/cmd_vel_source",
                        message_type="std_msgs/msg/String")
JOY = Ros2Subscriber(topic="/joy", message_type="sensor_msgs/msg/Joy")
# 网页键盘/虚拟摇杆的出口。与手柄分开成独立话题，由容器内的 cmd_vel_mux
# 仲裁后再进 /cmd_vel，避免两路输入互相覆盖。
WEB = Ros2Publisher(topic="/cmd_vel_web", message_type="geometry_msgs/msg/Twist")

SELFTEST = selftest.SelfTest()
# 导航链路的 App 侧（一条 JSON 订阅 + 一条命令发布），由 main 负责 start/stop。
NAV = navlink.NAV
# 摄像头采集线程同样由 main 启停；HTTP 线程只读它的最新一帧。
CAMERA = camera.CAMERA
# 物品检测：后台线程按"最新帧"推理，页面轮询 /api/detect 拿结果。
DETECTOR = detector.DETECTOR
RECORDER = telemetry_module.Telemetry()     # 纯内存缓冲，无目录参数
# 停车后把 100 Hz trace 分页拉出来；由控制环驱动，HTTP 线程只置请求位。
# 落在 App 目录下的 trace/：容器里是 /app，宿主机能直接取回（/tmp 是容器私有的）。
TRACE = trace_module.TraceDumper(
    directory=os.path.join(os.path.dirname(os.path.abspath(__file__)), "trace"),
    timeout=config.MCU_TIMEOUT_S)
UI = WebUI()


def bump_estop():
    """递增共享急停代次，返回新值。

    封装成函数是为了避免各模块对 `ESTOP_SEQ += 1` 各写一份 global。
    """
    global ESTOP_SEQ
    with STATE_LOCK:
        ESTOP_SEQ += 1
        return ESTOP_SEQ
