"""ROS 2 发布层：把控制环算出来的量翻译成标准消息。

为什么单独成文件
    这三个 publisher 只依赖 config 与数值，不碰 STATE / REQUESTS / 锁，
    是 main.py 里最容易独立的一块。拆出来后控制环只负责"算"，
    发布格式（frame_id、四元数、协方差、无效值约定）集中在这里，
    改话题名或消息字段时不必在 1200 行的控制文件里翻。

约定
    /scan        无效应填 0 而不是 inf：rosbridge 走 JSON，Infinity 不是合法值。
    /joint_states 两个轮子都是 continuous joint，没有它 robot_state_publisher
                 不给轮子发 TF，模型会停在初始姿态。
    /imu         只在 MCU 报告就绪且样本新鲜时发布，细节见 publish_imu。
"""

import math
import time

import config
from ros2_bridge import Ros2Publisher

# 雷达扫描。无效点填 0 而不是 inf：rosbridge 走 JSON，Infinity 不是合法 JSON 值。
SCAN = Ros2Publisher(topic="/scan", message_type="sensor_msgs/msg/LaserScan")
# 轮子转角。URDF 里两个轮子都是 continuous joint，没有 joint_states 时
# robot_state_publisher 不会给它们发 TF，模型会停在初始姿态。
JOINTS = Ros2Publisher(topic="/joint_states", message_type="sensor_msgs/msg/JointState")
# 航向与角速度。MCU 的 yawDeg 是相对校准起点累计、左转为正，不回绕，
# 正好适合给里程计当航向源。
IMU = Ros2Publisher(topic="/imu", message_type="sensor_msgs/msg/Imu")

SCAN_FRAME_ID = "laser_link"            # 必须和静态 TF 的 child frame 一致
SCAN_ANGLE_INCREMENT = math.pi / 90.0   # 2°，180 格满是 2π
SCAN_RANGE_MIN = 0.05
SCAN_RANGE_MAX = 12.0

# 只在 state=就绪时发 /imu：校准中(1)、未就绪(0)、数据超时(3) 一律停发，
# 下游因此会按"断流"退回轮速。恢复后再压 0.6 s 才发，让下游先退出 IMU 分支、
# 重新对齐零点，否则它会沿用旧偏移把航向一下拉回去。
_IMU_PUBLISH = {"last": None, "hold_until": 0.0, "samples": None}
# 样本年龄上限。上报周期实测 202 ms，超过一个周期就说明这条不是"刚采到的"。
# 放到 250 ms 会让 206 ms 的旧样本蒙混过关，正好卡在周期之后就失去意义。
IMU_MAX_AGE_MS = 200
# IMU 从失效状态恢复后，先压这么久不发，让下游超时退回轮速再重新对齐。
IMU_RESUME_HOLD_S = 0.6


def start():
    """启动全部发布器。"""
    SCAN.start()
    JOINTS.start()
    IMU.start()


def stop():
    """停止全部发布器。"""
    SCAN.stop()
    JOINTS.stop()
    IMU.stop()



def publish_scan(start_ms, period_ms, ranges, sectors, clock_ref=None):
    """把一帧 2° 分区发成 sensor_msgs/LaserScan。

    C1 的原始角度按顺时针增大，ROS 的 angle 逆时针为正，两者方向相反，
    必须按镜像映射：ROS 第 j 点对应雷达分区 (90 - j) mod sectors。
    只平移起点（用 90 + j）会得到左右镜像的点云。

    时间戳取帧的真实采集时刻：用读最后一片时的 (MCU uptime, 墙上时钟) 做基准，
    再减去帧开始到那一刻的间隔。直接用 now 会把拼帧和限速等待的 100~400 ms
    全算进去，订阅方做外参变换时会有可见滞后。
    """
    wall = time.time()
    if clock_ref:
        wall = clock_ref[1] - (clock_ref[0] - start_ms) / 1000.0
    sec = int(wall)
    ordered = [ranges[(90 - j) % sectors] for j in range(sectors)]
    SCAN.publish({
        "header": {"stamp": {"sec": sec, "nanosec": int((wall - sec) * 1e9)},
                   "frame_id": SCAN_FRAME_ID},
        "angle_min": -math.pi,
        # 180 束、2° 步长，最后一束是 +178°；写成 π 会让订阅方以为覆盖到 180°。
        "angle_max": -math.pi + (sectors - 1) * SCAN_ANGLE_INCREMENT,
        "angle_increment": SCAN_ANGLE_INCREMENT,
        "time_increment": 0.0,
        "scan_time": period_ms / 1000.0,
        "range_min": SCAN_RANGE_MIN,
        "range_max": SCAN_RANGE_MAX,
        "ranges": ordered,
        "intensities": [],
    })


# 计数基准的会话号，挂在 joint_states 里一起发。
# 为什么不用单独的 /wheel_epoch 话题：两条链路各自排队，到达顺序无法保证。
# 实测三种坏顺序都会发生——清零后的轮状态先到、复位通知先到但旧帧还在队列里。
# 把 epoch 塞进同一条消息，"这一帧属于哪一代计数"就是消息自带的属性，
# 不再依赖任何跨话题顺序。robot_state_publisher 会忽略 URDF 里没有的名字。
EPOCH_JOINT = "__wheel_epoch"


def publish_joints(status, wall, epoch=0):
    """把编码器累计计数换成轮子转角，发出去让 URDF 模型跟着转。

    URDF 两轮 axis 都是 +Y，绕 +Y 正转时轮顶朝 +X —— 正是前进方向，所以
    前进对应角度增加。ros_status 给的是原始计数：M1 换向符号 -1、M2 是 +1
    （固件 ENCODER_SIGN_*），按各自符号折算回来即可。
    left_wheel_joint 在 +Y 侧（左），对应 M2；right_wheel_joint 对应 M1。

    末尾附带 __wheel_epoch：同一帧里告诉里程计"这批计数的第几代"。
    它的值一变，接收端就知道计数被清零过、当前帧只能当基准。
    """
    wheels = status.get("wheels") if isinstance(status, dict) else None
    if not wheels or len(wheels) < 2:
        return
    scale = 2.0 * math.pi / config.COUNTS_PER_WHEEL_REV
    left = float(wheels[1].get("count", 0)) * scale
    right = -float(wheels[0].get("count", 0)) * scale
    sec = int(wall)
    JOINTS.publish({
        "header": {"stamp": {"sec": sec, "nanosec": int((wall - sec) * 1e9)},
                   "frame_id": ""},
        "name": ["left_wheel_joint", "right_wheel_joint", EPOCH_JOINT],
        "position": [round(left, 4), round(right, 4), int(epoch)],
        "velocity": [],
        "effort": [],
    })


def publish_imu(status, wall):
    """把 MCU 的航向与角速度发成 sensor_msgs/Imu。

    MCU 只有陀螺（yaw + yaw rate），没有加速度计，所以 linear_acceleration
    填 0 并把协方差首项置 -1（ROS 惯例：表示该字段不可用），
    免得下游把它当成真实加速度去二次积分。

    发布条件三条，缺一不可：
    - state 是"就绪"：校准中 / 未就绪 / 数据超时都不发
    - 样本年龄不超过一个上报周期
    - 样本计数在增长（不增 = 旧数据；回退 = IMU 被复位）
    """
    imu = status.get("imu") if isinstance(status, dict) else None
    if not imu:
        return

    state = imu.get("state")
    if state != config.IMU_STATE_READY:
        _IMU_PUBLISH["last"] = state
        return

    if _IMU_PUBLISH["last"] is not None and _IMU_PUBLISH["last"] != config.IMU_STATE_READY:
        # 刚从失效状态恢复。校零会让 yawDeg 整体平移，下游还拿着旧零点，
        # 第一帧就会把它的航向拽回去。压一段时间不发，让它先超时退回轮速。
        _IMU_PUBLISH["hold_until"] = wall + IMU_RESUME_HOLD_S
    _IMU_PUBLISH["last"] = state

    # 样本年龄：即使计数在涨，样本本身也可能是旧的（复位后重新累积的那几帧）。
    age = imu.get("age_ms")
    if age is not None and age > IMU_MAX_AGE_MS:
        return

    # 样本计数：不增加 = 没有新数据；回退 = IMU 被复位，两种情况都不能发。
    samples = imu.get("samples")
    if samples is not None:
        last = _IMU_PUBLISH["samples"]
        if last is not None and samples <= last:
            if samples < last:
                _IMU_PUBLISH["samples"] = samples          # 复位：重新记基准
                _IMU_PUBLISH["hold_until"] = wall + IMU_RESUME_HOLD_S
            return
        _IMU_PUBLISH["samples"] = samples

    if wall < _IMU_PUBLISH["hold_until"]:
        return

    yaw_deg = imu.get("yaw_deg")
    if yaw_deg is None:
        return

    half = math.radians(yaw_deg) * 0.5
    sec = int(wall)
    IMU.publish({
        "header": {"stamp": {"sec": sec, "nanosec": int((wall - sec) * 1e9)},
                   "frame_id": "imu_link"},
        "orientation": {"x": 0.0, "y": 0.0,
                        "z": math.sin(half), "w": math.cos(half)},
        "orientation_covariance": [0.0] * 9,
        "angular_velocity": {"x": 0.0, "y": 0.0,
                             "z": math.radians(imu.get("rate_dps") or 0.0)},
        "angular_velocity_covariance": [0.0] * 9,
        "linear_acceleration": {"x": 0.0, "y": 0.0, "z": 0.0},
        "linear_acceleration_covariance": [-1.0] + [0.0] * 8,
    })

