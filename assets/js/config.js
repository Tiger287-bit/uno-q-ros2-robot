/**
 * 常量与词表。
 *
 * 这里的数值与词表是**跨语言契约**，必须与下面几处保持一致：
 *   - MAX_CPS / LIMITS      → 固件 MAX_TARGET_CPS、后端 python/config.py
 *   - MODE/REASON/PHASE     → sketch/motion_controller.cpp 的枚举
 *   - IMU_NAMES             → sketch 里 imu.state 的取值
 *   - SEND_HZ               → 容器内 cmd_vel_mux 的 hold_s（见下方注释）
 * 改动前先确认对端，否则界面会显示"未知名"或速度标定错位。
 */

/** 编码器计数/秒的显示上限，仅用于进度条比例。 */
export const MAX_CPS = 3200;

export const MODE_NAMES = { 0: "空闲", 1: "运行", 2: "故障" };
export const IMU_NAMES = { 0: "未就绪", 1: "校准中", 2: "就绪", 3: "数据超时" };
export const PHASE_NAMES = { 0: "空闲", 1: "加速", 2: "保速", 3: "减速", 4: "停稳" };
export const REASON_NAMES = {
  0: "上电", 1: "手动停止", 2: "超时结束", 3: "通信看门狗", 4: "驱动故障",
  5: "编码器接反", 6: "编码器无进展保护", 7: "控制周期异常", 8: "PWM 异常",
  11: "运行中", 12: "点动中", 13: "试跑完成",
  14: "IMU 失去响应", 15: "航向超时", 16: "航向误差过大", 17: "标定中",
};
export const ASSIST_NAMES = {
  starting: "启动中", mcu_stale: "状态失联", imu_not_ready: "IMU 未就绪",
  no_linear: "无直行分量", turning: "转向中", deadband: "航向在死区",
  no_headroom: "内轮无余量", active: "生效中", paused: "急停锁定",
};

/* 后端 payload.limits 会覆盖这两个值，这里只是首帧到达前的兜底。
   整车与固件 MAX_TARGET_CPS=3200 对应，约 0.0989 m/s。 */
export const LIMITS_DEFAULT = { linear: 0.0989, angular: 1.20 };

/* 网页指令下发频率。必须 >= 5 Hz：容器内 cmd_vel_mux 的静默超时是 0.2 s，
   发得比这慢（早年试过 1 Hz）就会被判为链路静默、指令归零，实车呈脉冲式
   一顿一顿。20 Hz 对 0.2 s 有 4 倍余量，兼顾流量与丢帧容忍。 */
export const SEND_HZ = 20;
export const SEND_INTERVAL_MS = 1000 / SEND_HZ;

/** 摇杆死区（归一化后）。小于该值的位移视为回中。 */
export const DEADZONE = 0.08;

/** 判定"非零指令"的下限，避免浮点残值反复触发下发。 */
export const ZERO_EPS = 1e-6;

/** REST 兜底轮询周期，ms。 */
export const REST_POLL_MS = 500;

/** socket 静默多久后才允许 REST 兜底接管，ms。两路交替覆盖会让读数抖动。 */
export const SOCKET_GRACE_MS = 1500;

/** MCU 状态帧龄超过这个值时徽章转黄，ms。
    后端控制律用 0.5 s 判"状态失联"（MCU_STALE_S），界面放宽一倍：
    真实帧龄在 0~200 ms 之间抖动，用 0.5 s 会让徽章跟着闪。 */
export const MCU_STALE_MS = 1000;

/** 采样率低于这个值时认为采集已停（不是"变慢"而是"没有"）。 */
export const RATE_MIN_HZ = 1.0;

/** 点过急停/解除后，忽略服务端 paused 回声的时间窗，ms。 */
export const LOCK_ECHO_MS = 1500;

/** 一次性本地提示的默认寿命，ms。 */
export const LOCAL_MSG_MS = 3000;
