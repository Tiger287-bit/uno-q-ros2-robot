"""底盘控制律：指令解析、差速分解、直行保向、状态解码。"""

import math

import config

# 派生量：只在本模块使用，随 config 自动跟随。
VMAX_M_S = config.MAX_WHEEL_CPS / config.COUNTS_PER_METER    # 单轮最大线速度，m/s
MAX_TURN_M_S = config.MAX_ANGULAR * config.TRACK_M / 2.0     # 转向分量上限 ω·L/2，m/s

# 脱离粘滑区的最低线速度；配到接近上限时夹取，避免下面的分母趋零。
VMIN_M_S = config.MIN_INNER_CPS / config.COUNTS_PER_METER
if VMIN_M_S >= 0.5 * VMAX_M_S:
    print(f"[control] MIN_INNER_CPS={config.MIN_INNER_CPS} 超过单轮上限的 50%，已夹取", flush=True)
    VMIN_M_S = 0.5 * VMAX_M_S


def cmd_from(message):
    """从 Twist 消息取出 (线速度 m/s, 角速度 rad/s)。"""
    linear = (message or {}).get("linear") or {}
    angular = (message or {}).get("angular") or {}
    try:
        x = float(linear.get("x", 0.0))
        z = float(angular.get("z", 0.0))
    except (TypeError, ValueError):
        return 0.0, 0.0
    if not (abs(x) < 10.0 and abs(z) < 10.0):   # 丢弃明显异常的数值
        return 0.0, 0.0
    return x, z


def clamp_cmd(linear, angular):
    """把任意来源的速度请求限到整车能力内，非法值一律归零。

    与 wheel_targets 的峰值等比缩放不同，这里做硬限幅：网页发来
    0.5 m/s 会直接截到 VMAX_M_S，而不是按比例缩小整车速度。网页属于
    不可信输入，缺字段、字符串、NaN 都要在这里挡住。
    """
    try:
        linear = float(linear)
        angular = float(angular)
    except (TypeError, ValueError):
        return 0.0, 0.0
    if not (math.isfinite(linear) and math.isfinite(angular)):
        return 0.0, 0.0
    return (max(-VMAX_M_S, min(VMAX_M_S, linear)),
            max(-config.MAX_ANGULAR, min(config.MAX_ANGULAR, angular)))


def _turn_channels(linear_m_s):
    """转向分量的两条可用通道 (正向上限, 反向下限)，inf 表示不设限。

    轮速禁区是幅值 (0, MIN_INNER_CPS)。下限随平移速度推迟起算：
    a <= VMIN 不设限，a >= 2*VMIN 恢复 VMIN，中间线性过渡。
    若从 min(VMIN, a) 起步，a 小时 forward 恒为 0，整段低速会吸附到 0 或 2a。
    """
    a = abs(linear_m_s)
    need = 0.0 if a <= VMIN_M_S else min(VMIN_M_S, a - VMIN_M_S)
    if need <= 0.0:
        return MAX_TURN_M_S, float("inf")

    span = VMAX_M_S + need
    forward = max(0.0, min(a - need, a * (VMAX_M_S - need) / span))
    reverse = max(a + need, a * span / (VMAX_M_S - need))
    return forward, reverse


# 低于这个线速度时允许原地转；超过它，内轮就不允许反转（避免刮擦与突然换向）。
VMIN_PIVOT_M_S = 0.008


def _continuous_turn(request, linear_m_s):
    """把转向分量压进连续可达范围，替代原来的"二选一吸附"。

    旧实现把不可达的转向量吸附到 forward / reverse 两个端点，结果是摇杆在
    临界点附近挪一点，实际角速度就从 0.280 跳到 0.763（实测，2.7 倍），内轮
    同时由 +600 翻到 -600 —— 转弯突然变急、车头甩动、电流突增都来自这里。

    新函数只做限幅、不做吸附：
      * 线速度 >= VMIN_PIVOT：转向分量不超过 |v|，内轮最低到 0，绝不反转；
      * 线速度接近 0：可用转向额度连续放大到 MAX_TURN_M_S，原地转仍可用；
      * 两段之间线性过渡，映射整体连续、单调，不存在跳变点。
    """
    span = abs(linear_m_s)
    if span >= VMIN_PIVOT_M_S:
        return min(request, span)
    blend = span / VMIN_PIVOT_M_S if VMIN_PIVOT_M_S > 0 else 1.0
    limit = span + (MAX_TURN_M_S - span) * (1.0 - blend)
    return min(request, limit)


def _snap_turn(magnitude, forward, reverse):
    """把落在两通道之间的转向分量吸附到最近的可达边界。"""
    if magnitude <= forward:
        return magnitude
    if reverse <= MAX_TURN_M_S and magnitude >= reverse:
        return magnitude
    if reverse <= MAX_TURN_M_S and magnitude >= 0.5 * (forward + reverse):
        return reverse
    return forward


def wheel_targets(linear_m_s, angular_rad_s, snap=True, allow_pivot=True,
                  mapping="continuous"):
    """差速分解为双轮目标，返回 (M1 右轮, M2 左轮)，计数/s。

    snap 是"内轮吸附"，只该给人手操作用：摇杆的小幅转向会让内轮落在
    (0, MIN_INNER_CPS) 的死区里，轮子几乎不转、车在原地磨，吸附到边界
    手感好得多。代价是曲率被改写 —— 导航指令绝不能这么做：Nav2 的碰撞
    预测用的就是它下发的 (v, ω)，吸附后实际轨迹与预测不一致。

    实测（v=0.04 m/s，ω 从 0.52 到 0.53 rad/s）：吸附让内轮从正转翻成
    反转，实际角速度从 0.280 跳到 0.763 rad/s。
    """
    if not (math.isfinite(linear_m_s) and math.isfinite(angular_rad_s)):
        return 0, 0
    # 零输入必须严格输出零：摇杆回中、松杆、急停走的都是这条路径。
    # allow_pivot=False 的分支会把零线速度补成最小前进速度，若不在这里拦住，
    # (0,0) 会输出 (600,600) —— 实测确认过，属于必须堵死的安全缺陷。
    if not linear_m_s and not angular_rad_s:
        return 0, 0

    linear_m_s = max(-VMAX_M_S, min(VMAX_M_S, linear_m_s))
    request = min(abs(angular_rad_s * config.TRACK_M / 2.0), MAX_TURN_M_S)

    if not allow_pivot:
        # 这台车原地转不动：两轮反向时轮胎横向刮擦，实测占空比给到 220/255
        # 仍是零转速，导航一走纯旋转就卡死。遥控之所以"正常"，是因为人开车
        # 总是边走边转 —— 两轮同向只滚动，几乎没有刮擦。
        # 所以自动驾驶这里把转向分量压在平移分量以内：两轮始终同向，
        # 转弯退化成小弧线。代价是掉头半径变大、极窄处可能过不去。
        span = abs(linear_m_s)
        if span < VMIN_M_S:
            # 完全没有平移分量（纯旋转指令）时先给一个最小前进速度，
            # 否则 0 乘以任何数都还是 0，弧线无从谈起。
            linear_m_s = math.copysign(VMIN_M_S, linear_m_s if linear_m_s else 1.0)
            span = abs(linear_m_s)
        request = min(request, span * 0.9)
        turn = math.copysign(request, angular_rad_s)
    elif snap:
        # mapping="snap" 保留旧的二元吸附，仅供手感对比；默认走连续限幅。
        if mapping == "snap":
            forward, reverse = _turn_channels(linear_m_s)
            turn = math.copysign(_snap_turn(request, forward, reverse),
                                 angular_rad_s)
        else:
            turn = math.copysign(_continuous_turn(request, linear_m_s),
                                 angular_rad_s)
    else:
        # 精确差速：按指令的曲率分配，内轮该反转就反转。
        turn = math.copysign(request, angular_rad_s)

    right = linear_m_s + turn
    left = linear_m_s - turn

    # 峰值超限时等比缩放，保持内外轮比例。
    peak = max(abs(right), abs(left))
    factor = config.COUNTS_PER_METER if peak <= VMAX_M_S else config.MAX_WHEEL_CPS / peak
    return int(round(right * factor)), int(round(left * factor))


def apply_straight_assist(right, left, linear, angular, yaw, yaw_ref):
    """按 IMU 航向做差速修正，返回 (右轮, 左轮, 修正量, 新的 yaw_ref)。

    修正量按较慢一轮的余量收紧，不会把轮子推进 (0, MIN_INNER_CPS)；
    航向角速度与行驶方向无关，倒车沿用同一符号。
    """
    assist_on = (yaw is not None and math.isfinite(yaw)
                 and abs(linear) > config.ASSIST_LINEAR_MIN
                 and abs(angular) < config.ASSIST_ANGULAR_MAX)
    if not assist_on:
        return right, left, 0.0, None
    if yaw_ref is None:
        yaw_ref = yaw

    error = yaw - yaw_ref
    if abs(error) <= config.ASSIST_DEADBAND_DEG:
        return right, left, 0.0, yaw_ref

    headroom = min(abs(right), abs(left)) - config.MIN_INNER_CPS
    if headroom <= 0:                       # 已在死区内，再修正会让轮子停转
        return right, left, 0.0, yaw_ref

    limit = min(config.ASSIST_LIMIT_CPS, headroom)
    correction = max(-limit, min(limit, config.ASSIST_KP * error))
    right = int(round(max(-config.MAX_WHEEL_CPS,
                          min(config.MAX_WHEEL_CPS, right - correction))))
    left = int(round(max(-config.MAX_WHEEL_CPS,
                         min(config.MAX_WHEEL_CPS, left + correction))))
    return right, left, correction, yaw_ref


def decode_fault(raw):
    """解析可选的 V1 故障快照。旧固件和未知版本只提供基本状态。"""
    if len(raw) < 60 or raw[38] != 1:
        return None
    wheels = [
        {"target": raw[base], "speed": raw[base + 1] / 10.0, "pwm": raw[base + 2],
         "count": raw[base + 3], "invalid": raw[base + 4]}
        for base in (49, 54)
    ]
    return {
        "reason": raw[39], "wheel": raw[40], "uptime_ms": raw[41] & 0xFFFFFFFF,
        "requested_pwm": raw[42], "progress": raw[43], "awake": bool(raw[44]),
        "fault_pin": bool(raw[45]), "vm_raw": raw[46], "vm_min_raw": raw[47],
        "vm_age_ms": raw[48], "wheels": wheels,
        "input_levels": {pin: bool(raw[59] & (1 << bit))
                         for bit, pin in enumerate(("D12", "D13", "D9", "D10"))},
    }


def decode_status(raw):
    """解析 ros_status 的 38 项固定前缀和可选故障快照。"""
    if not raw or len(raw) < 38:
        return None

    wheels = []
    for index in (0, 1):
        base = 10 + index * 6
        wheels.append({
            "target": raw[base],
            "ramp": raw[base + 1] / 10.0,
            "speed": raw[base + 2] / 10.0,
            "pwm": raw[base + 3],
            "count": raw[base + 4],
            "invalid": raw[base + 5],
        })

    # 33–37：A4 分压采样的序号、最新值、100 ms 极值与样本年龄。
    # 序号只说明"启动以来采到过"，采样一旦超时（固件 vmMonitorRead 会把 raw
    # 置 -1），再按它算就会得到 -0.0，界面显示成 "0 V"，看着像整块电池掉了。
    # 判据必须是"本次回包里的读数有效"，而不是"以前采过"。
    sample_sequence = raw[33]
    voltage = None
    if sample_sequence > 0 and raw[34] >= 0:
        voltage = round(raw[34] / config.ADC_FULL_SCALE * config.ADC_REF_V
                        * config.DIVIDER_RATIO, 2)

    return {
        "mode": raw[0],
        "mode_text": config.MODE_NAMES.get(raw[0], "未知"),
        "reason": raw[1],
        "reason_text": config.REASON_NAMES.get(raw[1], f"代码 {raw[1]}"),
        "awake": bool(raw[2]),
        "fault_pin": bool(raw[3]),
        "ready": bool(raw[4]),
        "uptime_ms": raw[5],
        "sequence": raw[6],
        "period_ms": raw[7],
        "run_id": raw[8],
        "phase": raw[9],
        "phase_text": config.PHASE_NAMES.get(raw[9], "未知"),
        "wheels": wheels,
        "fault": decode_fault(raw),
        "imu": {
            "state": raw[22],
            "state_text": config.IMU_NAMES.get(raw[22], "未知"),
            "age_ms": raw[23],
            "rate_dps": raw[24] / 1000.0,
            "yaw_deg": raw[25] / 1000.0,
            "bias_dps": raw[26] / 1000.0,
            "samples": raw[27],
        },
        "heading": {
            "mode": raw[28],
            "target_deg": raw[29] / 1000.0,
            "yaw_deg": raw[30] / 1000.0,
            "error_deg": raw[31] / 1000.0,
            "correction_cps": raw[32],
        },
        "battery": {
            "samples": sample_sequence,
            "raw": raw[34],
            "min_raw": raw[35],
            "max_raw": raw[36],
            "age_ms": raw[37],
            "voltage": voltage,
        },
    }


if __name__ == "__main__":
    cases = [
        (0.000, 0.05, "静止小幅转向，比例响应"),
        (0.000, 0.12, "静止 10% 摇杆"),
        (0.000, 1.20, "原地满舵"),
        (0.010, 1.20, "1 cm/s 满舵，峰值缩放"),
        (0.020, 1.20, "2 cm/s 满舵，内轮反转"),
        (0.063, 1.20, "反向通道临界，内轮刚好 600"),
        (0.090, 0.00, "9 cm/s 直行"),
        (0.090, 1.20, "9 cm/s 满舵，内轮正向 600"),
        (-0.075, -0.60, "倒车右转"),
    ]
    print(f"vmax={VMAX_M_S:.4f}  vmin={VMIN_M_S:.4f}  max_turn={MAX_TURN_M_S:.4f} m/s")
    print(f"{'lin':>7} {'ang':>6} {'M1':>6} {'M2':>6} {'内侧':>6} {'ω':>6}  说明")
    for linear, angular, note in cases:
        m1, m2 = wheel_targets(linear, angular)
        omega = (m1 - m2) / config.COUNTS_PER_METER / config.TRACK_M
        print(f"{linear:+7.3f} {angular:+6.2f} {m1:6d} {m2:6d} "
              f"{min(abs(m1), abs(m2)):6d} {omega:+6.3f}  {note}")
