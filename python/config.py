"""整车常量与状态词表，调参只改这里。"""

import math

# ---------- 机械参数 ----------
TRACK_M = 0.1535                     # 左右轮中心距，m
WHEEL_DIAMETER_M = 0.065             # 车轮直径，m
COUNTS_PER_WHEEL_REV = 6608.0        # 每圈编码器计数（四倍频）
COUNTS_PER_METER = COUNTS_PER_WHEEL_REV / (math.pi * WHEEL_DIAMETER_M)

# ---------- 速度限制 ----------
MAX_WHEEL_CPS = 3200                 # 单轮上限，与固件 MAX_TARGET_CPS 一致
MAX_ANGULAR = 1.20                   # 原地旋转最大角速度，rad/s
MIN_INNER_CPS = 600                  # 内轮脱离粘滑区的目标值，不是硬约束

# ---------- 直行保向 ----------
ASSIST_KP = 80.0                     # 每度航向偏差对应的修正，cps/°
ASSIST_LIMIT_CPS = 400               # 修正上限，cps
ASSIST_DEADBAND_DEG = 0.15           # 航向死区，°
ASSIST_ANGULAR_MAX = 0.02            # 低于此角速度视为直行，rad/s
ASSIST_LINEAR_MIN = 0.019            # 高于此线速度才启用，m/s；刚好越过 VMIN=0.0185

# ---------- 通信与调度 ----------
CONTROL_HZ = 30                      # 控制环频率
STATUS_EVERY_TICKS = 6               # 每 6 拍读一次 MCU 状态（≈5 Hz）
# mux 的输入里哪些算"人手"（与容器内 mux.yaml 的 manual_topics 一致）。
# 只有明确来自它们的指令才做内轮吸附：吸附会改写曲率，而 Nav2 用自己下发的
# (v, ω) 做碰撞预测，改写后实际轨迹与预测对不上。
# 所以判定必须是"确定是人手"才吸附 —— 来源未知/超时一律不吸附。
# 反过来错一次只是人手的转向手感略黏，不会让车走出一条没人预测过的弧线。
MANUAL_SOURCE_TOPICS = ("/cmd_vel_joy", "/cmd_vel_web")
# 来源心跳 2 Hz，1 s 容忍丢一拍。来源与速度是两条消息、分开到达，
# 这个窗口只用来判断"标签还算不算新"，不参与仲裁。
SOURCE_TIMEOUT_S = 1.0
# 导航帧（位姿/地图/路径）向网页推送的节拍：每 3 拍一次 ≈10 Hz。
# 比 5 Hz 的遥测快一档，机器人在页面上移动才不会一格一格跳。
NAV_PUSH_TICKS = 3
CMD_TIMEOUT_S = 0.40                 # 指令超时即停车，s
# 单次 Bridge 调用超时，s。
#
# 0.2 s 太宽：一次没应答的 ros_status 会把整拍卡住 200 ms，两拍就顶到固件
# 看门狗（WATCHDOG_MS）上，现场表现是"跑着跑着突然停"。正常往返 9~15 ms，
# 100 ms 足够；超时只是丢掉这次读取，下一拍重来，代价远小于停车。
MCU_TIMEOUT_S = 0.20
# 导航速度指令短暂掉零时继续沿用的时长，s。控制器 10 Hz 偶发漏拍、
# mux 的 hold_s 只有 0.35 s，掉零就会让车停车并清零累加器。
NAV_CMD_HOLD_S = 0.45
# 0.10 太紧：串口一有噪声（实测 20 分钟 185 次 invalid packet）回包就迟到，
# 于是每一拍都判超时、指令被丢弃，车表现为"完全不动"。正常往返 9~15 ms，
# 0.2 s 仍然远大于它，只在链路真的卡住时才等到。
# 停车后 300 ms 内固件会拒绝起转（SETTLE_MS）。控制环 30 Hz，这段时间会连续
# 返回 False，页面若如实显示就是"被拒绝"闪 6~9 拍。超过该时长才算真拒绝。
DRIVE_REJECT_GRACE_S = 0.5           # 起转被拒的观察宽限期，s
FAULT_CLEAR_DELAY_S = 1.0            # 松杆后多久自动复位故障，s
FAULT_CLEAR_RETRY_S = 1.0            # 快速重试间隔，s
FAULT_CLEAR_BACKOFF_S = 5.0          # 多次失败后的重试间隔，s
FAULT_CLEAR_MAX_FAST = 3             # 前几次走快重试，之后降频并报一次
ERROR_ACTIVE_S = 2.0                 # 该窗口内出过错，payload.error 才算正在错
ERROR_HISTORY_S = 10.0               # last_error 的保留时长，s

# ---------- 链路延迟探针 ----------
LATENCY_ENABLED = True               # 关闭后不打印 [lat] 汇总，探针本身仍计时
LATENCY_REPORT_S = 5.0               # 汇总打印周期，s
LATENCY_CAPACITY = 600               # 单指标滑动窗口样本数，30 Hz 下约 20 s

# ---------- A4 电池采样 ----------
ADC_FULL_SCALE = 4095
ADC_REF_V = 3.3
DIVIDER_RATIO = (100.0 + 27.0) / 27.0   # R8 100K / R9 27K

# ---------- 状态词表（与固件枚举一一对应）----------
MODE_NAMES = {0: "空闲", 1: "运行", 2: "故障"}
IMU_NAMES = {0: "未就绪", 1: "校准中", 2: "就绪", 3: "数据超时"}
IMU_STATE_READY = 2                  # 仅该状态下航向可信
PHASE_NAMES = {0: "空闲", 1: "加速", 2: "保速", 3: "减速", 4: "停稳"}
REASON_NAMES = {
    0: "上电", 1: "手动停止", 2: "超时结束", 3: "通信看门狗", 4: "驱动故障",
    5: "编码器接反", 6: "编码器无进展保护", 7: "控制周期异常", 8: "PWM 异常",
    11: "运行中", 12: "点动中", 13: "试跑完成",
    14: "IMU 失去响应", 15: "航向超时", 16: "航向误差过大", 17: "标定中",
}

if CONTROL_HZ <= 0 or STATUS_EVERY_TICKS <= 0:
    raise RuntimeError("CONTROL_HZ 与 STATUS_EVERY_TICKS 必须为正")
