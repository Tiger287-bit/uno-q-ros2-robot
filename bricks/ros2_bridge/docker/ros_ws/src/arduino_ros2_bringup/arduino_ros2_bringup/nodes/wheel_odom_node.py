"""轮式里程计：把 /joint_states 的轮子累计角度差分成 /odom 和 odom→base_footprint。

为什么走 joint_states 而不是直接读编码器
    编码器在 MCU 上，ROS 容器看不到；整条链路是 MCU → Bridge → main.py →
    Ros2Publisher(/joint_states) → 本节点。好处是里程计和 URDF 模型读的是同一份
    角度，不会出现"模型转了但里程计没动"这种偏差，也不需要第二条数据通道。

差速模型（中点法）
    ds_l = Δθ_l · r，ds_r = Δθ_r · r
    ds   = (ds_l + ds_r) / 2              车体位移
    dψ   = (ds_r - ds_l) / L              航向变化
    x   += ds · cos(ψ + dψ/2)             用半程后的航向，比用旧航向少一半误差
    y   += ds · sin(ψ + dψ/2)
    ψ   += dψ

航向来源可以用 IMU
    MCU 的 yawDeg 是相对零点累计、左转为正、不回绕，比"两轮差速"稳得多
    —— 但经过四元数这一道会丢掉圈数（Quat 只能表示 ±180° 以内），
    所以节点收到后要逐帧展开成连续角，否则跨 ±180° 的插值会算错。
    —— 实车实测同一次左转，编码器差速只积出 3.5°，IMU 与雷达都指向 13~14°。
    把 use_imu_heading 置真后，位移仍取轮速、航向改取 IMU。

    但直接用 IMU 的绝对 yaw 有三个坑，这里都做了处理：
    1. 重新校零会让 yawDeg 整体平移。那种跳变不是车真的转了，必须识别出来
       并重新对齐零点，否则 /odom 的航向会跟着跳一大步。
    2. IMU 断流后不能继续沿用最后一个值 —— 车轮还在转，航向却冻住，位姿会
       越走越偏。超时后要退回轮速差速。
    3. IMU 和 joint_states 是两条独立链路，到达时刻不同。要按采样时间戳配对，
       不能用"最新收到的那帧"去配"当前的轮子状态"。
       做法是排队 + 按需等待：该帧戳上有样本就立刻发，没有才等一小段（IMU 在流
       时最多等一个周期，断流时到宽限期就走），到齐后**插值**取航向。
       不用外推 —— 急停时旧角速度看不出要停，会凭空多报一个周期。

符号约定（与 URDF 的 +Y 轴一致）
    URDF 两轮 axis 都取 +Y，前进时 joint position 增加 → ds 为正 → x 增加；
    左转时右轮比左轮走得多 → ds_r > ds_l → dψ 为正 → yaw 增加。
    left_wheel_joint 对应 M2、right_wheel_joint 对应 M1，与 URDF 的命名一致。

发布
    /odom   nav_msgs/Odometry，header.stamp 直接取 joint_states 的戳，
            保证位姿和产生它的那批角度是同一时刻。
    TF      odom → base_footprint，与 /odom 同戳。
"""

import math
import time
from collections import deque

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster

from arduino_ros2_bringup.common.runtime import run
from arduino_ros2_bringup.common.topics import Topics

# 与底盘配置一致：轮径 65 mm、轮距 153.5 mm。
DEFAULT_WHEEL_RADIUS = 0.0325
DEFAULT_WHEEL_SEPARATION = 0.1535
DEFAULT_LEFT_JOINT = "left_wheel_joint"
DEFAULT_RIGHT_JOINT = "right_wheel_joint"
DEFAULT_ODOM_FRAME = "odom"
DEFAULT_BASE_FRAME = "base_footprint"

# IMU 超过这么久没有新样本，就退回轮速差速。IMU 是 5 Hz，0.5 s 约等于两拍。
IMU_TIMEOUT_S = 0.5
# 单拍航向变化上限。5 Hz 下一拍最多转 30° 上下，留一倍余量；
# 超过这个量级只可能是 IMU 零点被重置，不是真的转向。
IMU_JUMP_RAD = math.radians(60.0)
# ---- 同批配对的等待策略 ----
# 不采用"固定等下一帧轮状态"（会稳定多 200 ms，且轮状态一停最后一帧就发不出去）。
# 改成队列 + 按需等待：
#   * 该帧时间戳上已有 IMU 样本 → 立刻处理，零额外延迟（常见情形）
#   * 还没有 → 先等一小段。IMU 在流说明样本会来，最多等一个周期；
#     IMU 已断流就没必要等，到宽限期直接按轮速出。
PAIR_GRACE_S = 0.06          # IMU 不在流时的宽限，约 1/3 个周期
PAIR_MAX_WAIT_S = 0.26       # IMU 在流时最多等一个周期多一点
PAIR_EXACT_S = 0.02          # 戳差在这个范围内直接取样本，不必插值
PAIR_QUEUE_MAX = 6           # 队列上限，防止 IMU 长期缺席时无限堆积
PAIR_FLUSH_PERIOD_S = 0.02   # 定时冲刷：轮状态断流时靠它把最后一帧发出去
# 单帧轮角变化上限，用来识别"累计计数被清零"。
# 物理上限来自固件的 MAX_TARGET_CPS=3200（≈3.0 rad/s），这里取 2 倍余量。
# 再宽就分不出"小幅清零"和"快速倒车"了 —— 800 ms 内走 6 cm 那种清零会漏判。
MAX_WHEEL_RATE_RAD = 6.0
# 插值允许跨的最大间隔。超过这个宽度说明中间断了，两侧样本可能属于不同零点，
# 硬插会把校零前后的航向混在一起（实测 40° 被算成 37.43°）。
MAX_INTERP_GAP_S = 0.3
# joint_states 里附带计数会话号的那个"关节"。它不是真关节，只用来把
# "这一帧属于第几代计数"随帧带过来（见 ros_publishers.EPOCH_JOINT）。
EPOCH_JOINT = "__wheel_epoch"
# 单帧航向跳变超过这个值就认为 IMU 换了零点（5 Hz 下真实转角最多 30°）。
IMU_RAW_JUMP_RAD = math.radians(45.0)


class WheelOdomNode(Node):
    """订阅 /joint_states，积分出二维位姿并发布 /odom 与 odom→base_footprint。"""

    def __init__(self):
        super().__init__("wheel_odom")

        self.wheel_radius = float(self.declare_parameter(
            "wheel_radius", DEFAULT_WHEEL_RADIUS).value)
        self.wheel_separation = float(self.declare_parameter(
            "wheel_separation", DEFAULT_WHEEL_SEPARATION).value)
        self.left_joint = str(self.declare_parameter(
            "left_joint", DEFAULT_LEFT_JOINT).value)
        self.right_joint = str(self.declare_parameter(
            "right_joint", DEFAULT_RIGHT_JOINT).value)
        self.odom_frame = str(self.declare_parameter(
            "odom_frame", DEFAULT_ODOM_FRAME).value)
        self.base_frame = str(self.declare_parameter(
            "base_frame", DEFAULT_BASE_FRAME).value)
        # 航向改用 IMU 时，位移仍来自轮速，只有 dψ 换源。
        self.use_imu_heading = bool(self.declare_parameter(
            "use_imu_heading", False).value)
        # odom→base 的 TF 重发频率。轮状态只有 5 Hz，而 AMCL 收到 /initialpose
        # 时会用 **当前时刻** 去查 odom→base（handleInitialPose 的双时间戳
        # lookupTransform），最新 TF 落后 200 ms 就必然报
        # "Failed to transform initial pose in time"。拿到位姿就立刻生效，
        # 但日志一直刷告警，现场容易误判。20 Hz 重发把这段间隔压到 50 ms。
        # 位姿本身不改，只是把"最后一次已知位姿"按当前时刻再播一遍；
        # 0.08 m/s 下 50 ms 最多差 4 mm，可忽略。
        self.tf_rate = float(self.declare_parameter("tf_rate", 20.0).value)
        # 重播的截止时间。轮状态一断流（App 停了、串口掉线），最后一次位姿
        # 不能继续带着"现在"的时间戳发出去 —— 那等于告诉 Nav2/网页"我刚量过"，
        # 定位明明已经断了却看着很新鲜。超过这个时长就停止重播，让 TF 自然老化。
        # 5 Hz 轮状态对应 0.2 s 一帧，0.6 s 容忍丢两帧再判死。
        self.tf_timeout = float(self.declare_parameter(
            "tf_timeout", 0.6).value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._prev = None            # (left_angle, right_angle, stamp_seconds)
        # 待处理的轮状态。两路各自异步发布，调用顺序不保证 —— 板上实测 44 批里
        # 有 10 批是 joint_states 先于 imu 到达。这里只排队、不固定延迟：
        # 该帧的 IMU 样本到了就立刻处理，没到才等一小段（见 _flush_pending）。
        self._pending = deque()
        # 当前计数世代。轮帧里带的 __wheel_epoch 一变，说明 MCU 侧的累计计数
        # 被清零过，这一帧只能当基准。
        self._epoch = 0
        # 已处理过的最新帧戳。用来挡乱序帧：它必须在动任何状态之前判断，
        # 否则一帧旧代数据会把基准和 epoch 一起带回去。
        self._last_stamp = None

        # IMU 历史：按采样时间存 (stamp_seconds, yaw)，用于和 joint_states 配对。
        # yaw 是**展开后的连续角**，不是 atan2 出来的 ±π 回绕值（见 on_imu）。
        self._imu_history = deque(maxlen=40)
        self._imu_last_wrapped = None   # 上一帧回绕角，用于逐帧展开
        self._imu_unwrapped = None      # 当前连续角
        self._imu_seen = None        # time.monotonic()，判断断流
        self._imu_ref = None         # IMU yaw 与里程计 yaw 的固定偏移
        # 上一拍是否在断流。断流期间轮速会把航向推走，恢复时若沿用旧偏移，
        # 第一帧就会把航向拉回断流前的位置 —— 必须先重新对齐。
        self._imu_was_stale = True

        self.publisher = self.create_publisher(Odometry, Topics.ODOM, 20)
        self.broadcaster = TransformBroadcaster(self)
        # 最近一次发布的位姿，供 TF 定时重发使用（None 表示还没算过）
        self._tf_pose = None
        self._odom_at = 0.0          # 最近一次真正算完轮状态并发布的时间
        self._tf_stale = False       # 是否已经因断流停发（只为少刷日志）
        self.create_subscription(JointState, Topics.JOINT_STATES, self.on_joints, 20)
        if self.tf_rate > 0.0:
            self.create_timer(1.0 / self.tf_rate, self._republish_tf)
        if self.use_imu_heading:
            self.create_subscription(Imu, Topics.IMU, self.on_imu, 20)
            # 轮状态停了以后没有"下一帧"来触发处理，靠这个定时器把队列里
            # 最后一帧冲刷出去，否则 /odom 会一直停在上一帧。
            self.create_timer(PAIR_FLUSH_PERIOD_S, self._flush_pending)

        self.get_logger().info(
            "轮式里程计就绪：r=%.4f m，L=%.4f m，%s→%s，航向源=%s，TF %.0f Hz"
            % (self.wheel_radius, self.wheel_separation,
               self.odom_frame, self.base_frame,
               "IMU" if self.use_imu_heading else "轮速差速", self.tf_rate))

    def on_imu(self, msg):
        """缓存一帧 IMU，并尝试放行队列里已经配对成功的轮状态。

        必须把航向展开成连续值再入历史。四元数只能表示 ±180° 以内的角度，
        用 atan2 反解出来是回绕的：+179° 之后下一帧就是 −179°。历史里存这种
        值，插值会把跨 ±180° 的中间值算成 0°，还会被误判成"零点重置"。
        上游 yawDeg 本身是累计不回绕的，只是过了四元数这一道又丢了圈数。
        """
        q = msg.orientation
        wrapped = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        now = time.monotonic()

        # 断流后恢复：零点可能已经被重设（校准、MCU 复位），旧样本不能和新样本
        # 放在同一条历史里 —— 否则跨断流的插值会把两个零点混起来。
        if self._imu_seen is not None and now - self._imu_seen > IMU_TIMEOUT_S:
            self._imu_history.clear()
            self._imu_unwrapped = None
            self._imu_was_stale = True

        if self._imu_unwrapped is not None:
            step = math.remainder(wrapped - self._imu_last_wrapped, 2.0 * math.pi)
            if abs(step) > IMU_RAW_JUMP_RAD:
                # 连续出数据但航向突变：校零了。旧样本全部作废，重新起算。
                self._imu_history.clear()
                self._imu_unwrapped = wrapped
                self._imu_was_stale = True
                self.get_logger().warn(
                    "IMU 航向突变 %.0f°，判定为重新校零，已清空历史"
                    % math.degrees(step),
                    throttle_duration_sec=2.0)
            else:
                self._imu_unwrapped += step
        else:
            self._imu_unwrapped = wrapped
        self._imu_last_wrapped = wrapped

        stamp = msg.header.stamp
        self._imu_history.append((stamp.sec + stamp.nanosec * 1e-9,
                                  self._imu_unwrapped))
        self._imu_seen = now
        if self.use_imu_heading:
            self._flush_pending()    # 晚到的那一帧一到，等待中的轮状态立刻放行

    def _imu_live(self, now):
        """IMU 是否还在出数据。决定队列要不要为它继续等。"""
        return (self._imu_seen is not None
                and now - self._imu_seen <= IMU_TIMEOUT_S)

    def _imu_yaw_at(self, stamp_seconds):
        """取给定时刻的 IMU 航向：同一时刻直接取，夹在两帧之间就插值。

        严格配对，绝不"沿用上一帧"。这一点很容易写错：如果这里在样本缺失时
        返回上一个值，调用方会以为配对成功、立刻处理，等待逻辑就形同虚设
        （实测就是因此重新出现了过冲）。取不到必须返回 None，让调用方决定
        是继续等还是退回轮速。

        也不外推：急停时旧角速度看不出要停，往前推会凭空多报一个周期。
        """
        history = self._imu_history
        if not history:
            return None

        target = stamp_seconds
        # 同一时刻（含毫秒级抖动）：直接取最近的样本。
        best = min(history, key=lambda item: abs(item[0] - target))
        if abs(best[0] - target) <= PAIR_EXACT_S:
            return best[1]

        # 被相邻两帧夹住：线性插值。历史里存的是展开后的连续角，
        # 跨 ±180° 不会出现 +179 → −179 那种回绕，可以直接插。
        # 但中间断了太久就不能插：两侧可能属于不同零点（校零/断流恢复）。
        for i in range(len(history) - 1, 0, -1):
            t1, y1 = history[i - 1]
            t2, y2 = history[i]
            if t1 <= target <= t2:
                if t2 - t1 > MAX_INTERP_GAP_S:
                    return None
                if t2 - t1 <= 1e-6:
                    return y2
                ratio = (target - t1) / (t2 - t1)
                return y1 + (y2 - y1) * ratio
        return None

    def on_joints(self, msg):
        """收到一帧关节角：入队，能配对就立刻处理。

        不用"固定等下一帧"：那会稳定多出 200 ms 延迟，而且轮状态一停，
        最后一帧永远等不到下一帧、根本发不出去。
        """
        if not self.use_imu_heading:
            self._process_joints(msg)
            return

        self._pending.append((msg, time.monotonic()))
        # 队列超限说明 IMU 长期没跟上，强制把旧的放出去，保住位移累积。
        while len(self._pending) > PAIR_QUEUE_MAX:
            stale, _ = self._pending.popleft()
            self._process_joints(stale)
        self._flush_pending()

    def _flush_pending(self):
        """按顺序处理队列：配上了就走，等超时了也走。

        等待上限分两种：
          * IMU 还在流 —— 样本大概率会来，最多等一个周期，等到了就是精确配对。
          * IMU 已断流 —— 再等也不会有，到宽限期立刻按轮速出。
        """
        if not self._pending:
            return
        now = time.monotonic()
        live = self._imu_live(now)
        while self._pending:
            msg, recv = self._pending[0]
            stamp = msg.header.stamp
            stamp_s = stamp.sec + stamp.nanosec * 1e-9
            if self._imu_yaw_at(stamp_s) is not None:
                self._pending.popleft()
                self._process_joints(msg)
                continue
            limit = PAIR_MAX_WAIT_S if live else PAIR_GRACE_S
            if now - recv < limit:
                break                # 还没到时候，等下一个事件或定时器
            self._pending.popleft()
            if live:
                self.get_logger().warn(
                    "轮状态 %.0f ms 未等到同批 IMU，按轮速处理"
                    % ((now - recv) * 1000.0),
                    throttle_duration_sec=5.0)
            self._process_joints(msg)

    def _epoch_of(self, msg):
        """取这一帧携带的计数世代；没有该字段（旧版发布端）就沿用当前值。"""
        try:
            return int(msg.position[msg.name.index(EPOCH_JOINT)])
        except (ValueError, IndexError):
            return self._epoch

    def _process_joints(self, msg):
        """对一帧完整数据做差分积分并发布。"""
        try:
            left = float(msg.position[msg.name.index(self.left_joint)])
            right = float(msg.position[msg.name.index(self.right_joint)])
        except (ValueError, IndexError):
            return                   # 还没收到这两个关节，等下一帧

        stamp = msg.header.stamp
        now_s = stamp.sec + stamp.nanosec * 1e-9

        # 乱序帧必须先挡掉，而且**不能动任何状态**。旧代数据迟到时若先过了
        # 下面的世代判断，会把 _epoch 和基准一起带回旧代，接着再切回新代，
        # 结果是先发一帧时间戳回退的 /odom，再丢掉新代的一段位移。
        if self._last_stamp is not None and now_s <= self._last_stamp:
            self.get_logger().warn(
                "轮状态乱序（%.0f ms 前），已丢弃"
                % ((self._last_stamp - now_s) * 1000.0),
                throttle_duration_sec=5.0)
            return
        self._last_stamp = now_s

        epoch = self._epoch_of(msg)
        if epoch != self._epoch:
            # 这一帧的计数是新一代的。只丢掉比它更旧的待处理帧 ——
            # 队列里可能已经排着同代的新帧，整队清空会把它们一起吞掉
            # （最后一帧就是这么丢的）。
            self._epoch = epoch
            before = len(self._pending)
            self._pending = deque(
                item for item in self._pending
                if (item[0].header.stamp.sec + item[0].header.stamp.nanosec * 1e-9)
                > now_s)
            self._prev = None        # 当前帧只当新基准，不发位移
            self._imu_was_stale = True   # MCU 重启通常把 IMU 零点一起复位了
            self.get_logger().warn(
                "新计数世代（epoch=%d），丢弃 %d 帧旧帧、重设基准"
                % (epoch, before - len(self._pending)),
                throttle_duration_sec=2.0)

        if self._prev is None:
            # 第一帧只用来对齐基准。仍发一次零位姿，让 TF 树立刻有 odom 段，
            # 否则下游在收到第二帧之前查 odom→base_footprint 会直接失败。
            self._prev = (left, right, now_s)
            self._publish(stamp, 0.0, 0.0)
            return

        prev_left, prev_right, prev_s = self._prev

        ds_left = (left - prev_left) * self.wheel_radius
        ds_right = (right - prev_right) * self.wheel_radius
        dt = now_s - prev_s
        if dt <= 0.0:
            # 乱序或重复的帧：直接丢弃，**不能更新基准**。原来先写 _prev 再判
            # dt，一帧旧数据就会把基准覆盖成旧值，下一帧按旧基准差分，位移算多。
            self.get_logger().warn(
                "轮状态乱序（dt=%.0f ms），已丢弃" % (dt * 1000.0),
                throttle_duration_sec=5.0)
            return
        if dt > IMU_TIMEOUT_S:
            # 轮状态断流过：这段"位移"其实是断流期间累积的，一次积进去会让
            # 位置跳一大步。这一帧只用来重设基准，不积分。
            # 阈值取 IMU_TIMEOUT_S(0.5 s) 而不是两倍：806 ms 那种短断流
            # 也必须触发重新对齐，否则 IMU 先恢复并被复位时航向会整个跳回去。
            # 同时必须把 IMU 也标记为失效：两路一起断时，IMU 可能先恢复并被
            # 复位，若沿用断流前的基准，下一帧会把航向整个拉回去（30°→0°）。
            self._prev = (left, right, now_s)
            self._imu_was_stale = True
            self.get_logger().warn(
                "轮状态中断 %.0f ms，已重设积分基准" % (dt * 1000.0),
                throttle_duration_sec=2.0)
            return

        # 累计计数被清零（MCU 复位、手动清计数）：一帧的角度变化会大到物理不可能，
        # 直接积分会把已走的位移一把抹掉、位置跳回原点。判据按轮角速度折算，
        # 时间下限取 50 ms，免得时间戳抖动把正常帧误判。
        max_dtheta = MAX_WHEEL_RATE_RAD * max(dt, 0.05)
        if abs(left - prev_left) > max_dtheta or abs(right - prev_right) > max_dtheta:
            self._prev = (left, right, now_s)
            self.get_logger().warn(
                "轮子计数突变（疑似清零，Δ=%.0f/%.0f rad），已重设积分基准"
                % (left - prev_left, right - prev_right),
                throttle_duration_sec=2.0)
            return                   # 基准已更新，下一帧从新值继续

        # 全部校验通过，才提交新基准。
        self._prev = (left, right, now_s)

        ds = 0.5 * (ds_left + ds_right)
        d_yaw = self._heading_delta(ds_left, ds_right, now_s)

        # 中点法：先用半个航向变化推进，再把航向更新完整。
        mid = self.yaw + 0.5 * d_yaw
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.yaw = math.remainder(self.yaw + d_yaw, 2.0 * math.pi)

        self._publish(stamp, ds / dt, d_yaw / dt)

    def _heading_delta(self, ds_left, ds_right, now_s):
        """算这一拍的航向变化：优先用 IMU，取不到就退回轮速差速。"""
        wheel_delta = (ds_right - ds_left) / self.wheel_separation
        if not self.use_imu_heading:
            return wheel_delta

        # 断流判断用墙钟：IMU 停了但轮子还在转时，必须退回轮速，
        # 否则航向会冻结在最后一个值上，位姿越走越偏。
        if self._imu_seen is None or time.monotonic() - self._imu_seen > IMU_TIMEOUT_S:
            self._imu_was_stale = True
            return wheel_delta

        imu_yaw = self._imu_yaw_at(now_s)
        if imu_yaw is None:
            return wheel_delta       # 这一时刻附近没有 IMU 样本，先按轮速走

        if self._imu_ref is None or self._imu_was_stale:
            # 首次可用，或刚从断流/校零恢复。基准要对齐到**本拍积分之后**的
            # 航向：这一拍轮速已经把车推到 self.yaw + wheel_delta，若按积分前
            # 对齐，下一拍 IMU 追上来时会把这拍转过的角度又吐回去（20°→25°→20°）。
            self._imu_ref = imu_yaw - (self.yaw + wheel_delta)
            self._imu_was_stale = False

        # 直接跟随 IMU 的绝对位置，不再用增量累加。
        # 增量在"IMU 比轮速晚一拍"时会把上一拍的角度吐回去；绝对位置每拍
        # 都重新同步一次，不会累积也不会丢拍。
        target = math.remainder(imu_yaw - self._imu_ref, 2.0 * math.pi)
        delta = math.remainder(target - self.yaw, 2.0 * math.pi)
        if self._imu_ref is not None and abs(delta) > IMU_JUMP_RAD:
            # 这不是车真的转了，是 IMU 被重新校零、零点整体平移了。
            self._imu_ref = imu_yaw - (self.yaw + wheel_delta)
            self.get_logger().warn(
                "IMU 航向跳变 %.0f°，判定为零点重置，已重新对齐" % math.degrees(delta),
                throttle_duration_sec=2.0)
            return wheel_delta

        return delta

    def _publish(self, stamp, linear, angular):
        """发 /odom 与 odom→base_footprint，两者时间戳一致。"""
        half = 0.5 * self.yaw
        qz = math.sin(half)
        qw = math.cos(half)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        # 二维轮式里程计不观测 z / roll / pitch，协方差给大值让下游别当有效值用。
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[14] = 1e6
        odom.pose.covariance[21] = 1e6
        odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = 0.05
        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular
        self.publisher.publish(odom)

        self._tf_pose = (self.x, self.y, qz, qw)
        self._odom_at = time.monotonic()
        self._tf_stale = False
        self.broadcaster.sendTransform(self._make_tf(stamp))

    def _make_tf(self, stamp):
        """按给定时间戳组装 odom→base 的 TF。"""
        x, y, qz, qw = self._tf_pose
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        return transform

    def _republish_tf(self):
        """按 tf_rate 把最后一次已知位姿重播一遍，时间戳取当前时刻。

        轮状态 5 Hz，而 AMCL、代价地图、tf2_echo 都按"现在"查 TF。
        不重播的话，任何按 now 查 odom→base 的调用都有 0~200 ms 的概率
        落在最新样本之后（extrapolation into the future）。/odom 话题
        保持 5 Hz 不变 —— 它带速度、下游按需订阅；只有 TF 需要更密。

        但重播必须能停：轮状态断流后继续用当前时间戳发旧位姿，等于把
        "10 秒前的位姿"标成刚测到的。超过 tf_timeout 就停，TF 断流由
        Nav2 自己去报（代价地图会停止更新），不能靠我们伪造新鲜度。
        """
        if self._tf_pose is None:
            return
        age = time.monotonic() - self._odom_at
        if age > self.tf_timeout:
            if not self._tf_stale:
                self._tf_stale = True
                self.get_logger().warn(
                    "轮状态已断流 %.1f s，停止重发 odom→base（不再伪装新鲜位姿）"
                    % age)
            return
        self._tf_stale = False
        self.broadcaster.sendTransform(
            self._make_tf(self.get_clock().now().to_msg()))


def main():
    """节点入口。"""
    run(WheelOdomNode)
