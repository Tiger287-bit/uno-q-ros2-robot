#pragma once

#include <cstdint>
#include "heading_imu.h"

namespace motion {

constexpr uint32_t PERIOD_MS = 20;                 // 空闲和开环测试的采样周期，ms。
constexpr int MAX_PID_PWM = 255;                  // 全占空比对应的 PWM 代码值。
// AT8833 这一档在这块板子上有死区：实测 duty=255 时两路 H 桥输入都被拉低
// （writeInputDuty 走的是 holdInputLevel + pwm_set_dt(…,0) 的纯 GPIO 分支），
// 电机进入滑行，编码器一动不动；duty≈217 时正常转（1942 cps）。表现为
// "目标越小 PID 越往上顶，顶到满占空比反而彻底停住"，导航里每隔几秒就卡死。
// 在驱动层彻底改掉之前，先把可用上限压到实测有效的区间顶端。
constexpr int MAX_APPLIED_PWM = 220;              // 实际允许写入的最大占空比。
// 实测：149~194 正常输出，250 与 255 都是零输出（同一个失效区）。
// 220 落在已验证的可用区间内，又比 215 多留一点速度余量。
constexpr int MAX_DEADZONE_PWM = 102;             // 点动和前馈允许的最大 PWM 代码值。
constexpr int MAX_TARGET_CPS = 3200;               // 单轮速度目标幅值上限，计数/s。
constexpr int MIN_RAMP_CPS_PER_S = 500;            // 最小轮速斜率，计数/s²。
constexpr int MAX_RAMP_CPS_PER_S = 6000;           // 最大轮速斜率，计数/s²。
constexpr float YAW_TOLERANCE_DEG = 2.0f;          // 定角转向的停稳角度容差，°。

// 运行期可调参数索引。顺序就是 Bridge 协议顺序，python/tune.py 与前端
// assets/js/tune.js 必须与这里逐项一致，改顺序会让界面把值写到别的参数上。
enum TuneParam {
  TuneKp = 0,         // 轮速比例增益，×10000。
  TuneKi,             // 轮速积分增益，×10000。
  TuneKd,             // 轮速微分增益，×10000。
  TuneRamp,           // 轮速斜坡，计数/s²。
  TuneFfOffset,       // 前馈截距，×10，PWM 代码值。
  TuneFfSlope,        // 前馈斜率，×100，计数/秒每 PWM 代码值。
  TuneFfSafety,       // 前馈系数，×1000。
  TuneBreakawayDuty,  // 静摩擦起转占空比，PWM 代码值。
  TuneBreakawayBoost, // 起转相对前馈的最大抬升，PWM 代码值。
  TuneHeadingKp,      // 直行保向比例增益，×1000，1/s。
  TuneHeadingKd,      // 直行保向角速度反馈增益，×1000。
  TuneHeadingLimit,   // 直行保向单轮修正上限，计数/s。
  TuneParamCount      // 参数个数，恒在末尾。
};

// 枚举值直接写入 Bridge 状态数组，须与 Python 解码一致。
enum class Mode { Idle = 0, Active = 1, Fault = 2 };
enum StopReason {
  Boot, Manual, Expired, Watchdog, DriverFault, EncoderWrong, EncoderStall,
  Timing, PwmError, Running = 11, DeadzoneRunning, Complete,
  ImuLost, HeadingTimeout, HeadingError, PwmCalibrationRunning = 17
};
// DriveRun 用于 ROS 2 连续遥控：没有固定时限，只受通信看门狗和故障保护约束。
enum RunKind { IdleRun, PidRun, DeadzoneRun, PwmCalibrationRun = 3, DriveRun = 4 };
enum class MotionPhase { Idle, RampUp, Hold, RampDown, Settle };
enum class HeadingMode { Wheel, Straight, Turn };

struct HeadingSettings {
  HeadingMode mode = HeadingMode::Wheel; // 轮速、直行保向或定角转向。
  float target = 0;                      // 相对起始车头的目标航向，°，左转为正。
  float kp = 2.5f;                       // 航向比例增益，1/s。
  float kd = 0.25f;                      // 航向角速度反馈增益，无量纲。
  int correctionLimit = 300;            // 单轮直行修正上限，计数/s。
};

struct WheelSettings {
  int32_t target = 0;       // 带方向的轮速目标，计数/s。
  int32_t gainCodes[3] = {}; // Kp、Ki、Kd，按 10000 倍编码。
  int sign = 1;             // 原始编码器计数转换到车辆前进方向的符号。
  int deadzonePwm = 0;     // 非零 PID 输出附加的单轮死区补偿，UNO Q PWM 代码值。
  int feedforwardPwm = 0;  // 按目标速度估算的前馈占空比，与死区补偿叠加但不受其限幅。
};

struct MotionSettings {
  int id = 0;                // 本次试跑编号，用于匹配心跳。
  uint32_t holdMs = 0;       // 保速时间或转向时限，ms。
  int pwmLimit = 0;          // 单轮 PWM 代码值上限。
  int rampCpsPerS = 0;       // 轮速参考斜率，计数/s²。
  WheelSettings wheels[2];  // M1 右轮、M2 左轮。
  HeadingSettings heading;  // 航向外环配置。
  bool driveHoldHeading = false; // 连续驾驶是否请求本地直行保向。
};

constexpr int PWM_CALIBRATION_STAGES = 3;            // 一次标定的固定 PWM 档数。
constexpr uint32_t PWM_CALIBRATION_MIN_STAGE_MS = 2000;  // 单档最短时间，ms。
constexpr uint32_t PWM_CALIBRATION_MAX_STAGE_MS = 10000; // 单档最长时间，ms。
constexpr int PWM_CALIBRATION_PAGE_ROWS = 8;        // 单次 Bridge 读取的最大行数。

struct PwmCalibrationSettings {
  int id = 0;                                    // 标定编号，用于匹配心跳和读取记录。
  uint32_t stageMs = PWM_CALIBRATION_MIN_STAGE_MS; // 每档持续时间，ms，兼容旧的 2 秒指令。
  uint8_t duty[PWM_CALIBRATION_STAGES][2] = {};    // 每档 M1 右轮、M2 左轮的前进 PWM 代码值。
};

struct PwmCalibrationSample {
  uint16_t elapsedMs = 0; // 本采样区间结束时距启动的时间，ms。
  uint8_t stage = 0;      // 本采样区间施加的档位，从 0 开始。
  uint8_t dtMs = 0;       // 实际采样间隔，ms。
  int32_t delta[2] = {}; // 两轮前进方向的原始编码器增量。
  uint8_t duty[2] = {};  // 本采样区间两轮实际施加的 PWM 代码值。
};

struct PwmCalibrationPage {
  int id = 0;                                  // 匹配的标定编号，0 表示找不到记录。
  int total = 0;                               // 当前标定已记录的样本数。
  int stopReason = -1;                         // 标定停止原因，-1 表示仍在运行。
  int count = 0;                               // 本页实际样本数。
  PwmCalibrationSample rows[PWM_CALIBRATION_PAGE_ROWS]; // 本页连续样本。
};

struct WheelStatus {
  int32_t target = 0;        // 带方向的轮速目标，计数/s。
  float ramp = 0;            // 带方向的斜坡速度参考，计数/s。
  float speed = 0;           // 带方向的滤波轮速，计数/s。
  int pwm = 0;               // 实际带方向的 PWM 代码值。
  uint32_t count = 0;        // 原始编码器累计计数，按 32 位回绕。
  uint32_t invalid = 0;      // 编码器无效跳变累计次数。
  int32_t gainCodes[3] = {}; // Kp、Ki、Kd，按 10000 倍编码。
  int sign = 1;              // 编码器方向换算符号。
};

struct DeadzoneStatus {
  int id = 0;            // 点动试跑编号。
  int wheel = 0;         // 1 为 M1，2 为 M2，0 表示尚无试跑。
  int pwm = 0;           // 带方向的点动 PWM 代码值。
  uint32_t elapsed = 0;  // 点动已持续时间，ms。
  int32_t delta = 0;     // 点动期间原始编码器计数增量。
  uint32_t invalid = 0;  // 点动期间编码器无效跳变次数。
  int stopReason = -1;   // 停止原因代码，-1 表示尚未记录。
};

struct FaultWheelSnapshot {
  int32_t target = 0;    // 故障瞬间的带方向轮速目标，计数/s。
  float speed = 0;       // 故障瞬间的滤波轮速，计数/s。
  int pwm = 0;           // 故障前最后一次成功写入的带方向 PWM 代码值。
  uint32_t count = 0;    // 故障瞬间的编码器原始累计计数。
  uint32_t invalid = 0;  // 故障瞬间的编码器累计无效跳变次数。
};

struct FaultSnapshot {
  int reason = -1;       // 触发故障的原因代码，-1 表示无故障快照。
  int wheel = 0;         // 1 为 M1，2 为 M2，0 为整车故障。
  int requestedPwm = 0;  // 触发轮当拍请求的 PWM；该值可能还没有写入。
  int32_t progress = 0;  // 触发周期沿目标方向的编码器增量。
  uint32_t uptime = 0;   // 故障时的 MCU 启动后时间，ms。
  bool awake = false;    // 关断前 nSLEEP 回读。
  bool faultPinHigh = false; // 关断前 nFAULT 回读。
  uint8_t inputLevels = 0; // 关断前 H 桥输入的瞬时回读；非 PWM 脉宽测量。
  int32_t vmRaw = -1;    // 故障时可用的最新 VM 采样。
  int32_t vmMinRaw = -1; // 故障前 100 ms 内最低 VM 采样。
  int32_t vmAgeMs = -1;
  FaultWheelSnapshot wheels[2];
};

struct HeadingStatus {
  HeadingMode mode = HeadingMode::Wheel; // 当前航向控制模式。
  float target = 0;                      // 相对目标航向，°。
  float yaw = 0;                         // 相对实测航向，°，左转为正。
  float error = 0;                       // 目标航向减实测航向，°。
  float correction = 0;                  // M1 的航向轮速修正量，计数/s。
  int correctionLimit = 300;            // 单轮直行修正上限，计数/s。
};

// ---------- 100 Hz 控制拍 trace ----------
// 只记录定长整数，控制线程写入路径上不做分配、不做浮点格式化。
// 字段选择对应"首轮基线日志"要求：时间戳、目标/斜坡/原始实测、实际 PWM、
// 本拍前馈、陀螺角速度与样本年龄、母线电压、故障与限幅标志。
constexpr int TRACE_SAMPLE_MAX = 2048;   // 环形容量，10 ms 一拍约 20.5 s。
constexpr int TRACE_PAGE_ROWS = 16;      // 单次 Bridge 读取的最大样本数。
constexpr int ENV_SAMPLE_MAX = 600;      // 环境样本容量，1 Hz 约 10 分钟。
constexpr int ENV_PAGE_ROWS = 16;

// trace 标志位（TraceSample::flags）。
enum TraceFlag : uint8_t {
  TRACE_SAT_M1 = 1u << 0,      // M1 本拍输出撞到 pwmLimit。
  TRACE_SAT_M2 = 1u << 1,      // M2 本拍输出撞到 pwmLimit。
  TRACE_ZERO_M1 = 1u << 2,     // M1 本拍目标为零、未参与闭环。
  TRACE_ZERO_M2 = 1u << 3,     // M2 本拍目标为零、未参与闭环。
  TRACE_IMU_NOT_READY = 1u << 4, // IMU 非 Ready。
  TRACE_IMU_STALE = 1u << 5,   // IMU 样本年龄超过 50 ms。
  TRACE_NOT_ACTIVE = 1u << 6,  // 本拍 mode != Active。
  TRACE_FAULT = 1u << 7,       // 本拍处于 Mode::Fault。
};

struct TraceSample {
  uint32_t tickMs = 0;     // MCU 单调时间，ms（与 status 的 uptimeMs 同源）。
  uint16_t seq = 0;        // 控制拍序号低 16 位，用于发现丢拍。
  uint8_t dtMs = 0;        // 距上一拍的实际间隔，ms，饱和到 255。
  uint8_t flags = 0;       // TraceFlag 位域。
  uint8_t imuAgeMs = 0;    // 记录时刻 IMU 样本年龄，ms，饱和到 255。
  uint8_t reserved = 0;    // 对齐用，恒为 0。
  int16_t target[2] = {};  // 带方向的轮速目标，计数/s。
  int16_t ramp[2] = {};    // 带方向的斜坡参考，计数/s。
  int16_t speed[2] = {};   // 本拍原始实测轮速（未滤波），计数/s。
  int16_t pwm[2] = {};     // 本拍实际写入的带方向 PWM 代码值。
  uint8_t ffDuty[2] = {};  // 本拍实际叠加的前馈占空比（0–255）。
  int16_t rateDpsX100 = 0; // 扣偏后的偏航角速度 ×100，°/s。
  uint16_t vbatRaw = 0;    // A4 母线电压原始码值，无效时为 0。
};
static_assert(sizeof(TraceSample) == 32, "TraceSample 必须保持 32 字节定长");

struct TracePage {
  uint32_t epoch = 0;      // trace 会话号，每次启动新的运行 +1。
  int total = 0;           // 已写入样本总数（含已被覆盖的）。
  int dropped = 0;         // 因环形覆盖而未能导出的样本数。
  int capacity = 0;        // 环形容量。
  int bytesPerSample = 0;  // 单样本字节数，供 Linux 侧核对结构布局。
  int periodMs = 0;        // 记录周期，ms。
  int count = 0;           // 本页实际样本数。
  TraceSample rows[TRACE_PAGE_ROWS] = {}; // 本页连续样本，按时间从旧到新。
};

struct EnvSample {
  uint32_t tickMs = 0;      // 采样时刻，ms。
  int16_t tempCx100 = 0;    // IMU 芯片温度 ×100，℃。
  int16_t biasDpsX1000 = 0; // 当前陀螺零偏 ×1000，°/s。
  uint16_t seq = 0;         // 环境样本序号低 16 位。
  uint8_t state = 0;        // ImuHeadingState。
  uint8_t reserved = 0;     // 对齐用。
};
static_assert(sizeof(EnvSample) == 12, "EnvSample 必须保持 12 字节定长");

struct EnvPage {
  uint32_t epoch = 0;   // 与 TracePage 同步的会话号。
  int total = 0;        // 已写入环境样本总数。
  int dropped = 0;      // 被覆盖的样本数。
  int count = 0;        // 本页实际样本数。
  EnvSample rows[ENV_PAGE_ROWS] = {};
};

struct MotionStatus {
  Mode mode = Mode::Idle;             // 控制器状态。
  StopReason reason = Boot;           // 当前状态对应的原因。
  bool awake = false;                 // 驱动 nSLEEP 的逻辑电平。
  bool faultPinHigh = false;          // 驱动 nFAULT 的逻辑电平，高为正常。
  uint32_t remainingMs = 0;           // 当前阶段剩余时间，ms。
  uint32_t sequence = 0;              // 状态更新序号，按 32 位回绕。
  uint32_t uptimeMs = 0;              // MCU 启动后时间，ms。
  uint32_t controlPeriodMs = PERIOD_MS; // 当前模式设定的控制周期，ms。
  bool ready = false;                 // PWM 和控制线程是否均已就绪。
  int runId = 0;                      // 正在运行的试跑编号，空闲时为 0。
  int pwmLimit = MAX_PID_PWM;         // 单轮 PWM 代码值上限。
  WheelStatus wheels[2];             // 同一快照中的 M1 右轮和 M2 左轮状态。
  RunKind runKind = IdleRun;          // 本次试跑类型。
  DeadzoneStatus deadzone;            // 最近一次点动结果。
  MotionPhase phase = MotionPhase::Idle; // 当前运动阶段。
  int rampCpsPerS = 0;                // 轮速参考斜率，计数/s²。
  uint32_t holdMs = 0;                // 保速时间或转向时限，ms。
  FaultSnapshot fault;                // 最近故障触发时的快照。
  ImuHeadingSample imu;               // 最新 IMU 状态、航向和角速度。
  HeadingStatus heading;             // 本次试跑的航向外环状态。
};

/**
 * @description         : 初始化控制线程和 Bridge API 共用的状态锁
 * @return              : void
 */
void init();

/**
 * @description         : 初始化 PWM 和编码器，并启动轮速控制线程
 * @return              : void
 */
void startWorker();

/**
 * @description         : 按已经校验的参数启动双轮闭环试跑
 * @param settings      : 双轮、航向和运行时限配置
 * @param kind          : 试跑类型，遥控连续驾驶时传 DriveRun
 * @return              : 试跑启动时返回 true
 */
bool start(const MotionSettings &settings, RunKind kind = PidRun);

/**
 * @description         : 按已经校验的参数启动遥控连续驾驶，不设固定时限
 * @param settings      : 双轮、航向配置，holdMs 字段被忽略
 * @return              : 驾驶启动时返回 true
 */
bool drive(const MotionSettings &settings);

/**
 * @description         : 在线更新连续驾驶的双轮目标速度并刷新通信心跳
 * @param rightCps      : M1 右轮目标，计数/s，正值前进
 * @param leftCps       : M2 左轮目标，计数/s，正值前进
 * @param holdHeading   : 直行时请求 MCU 本地保向
 * @return              : 当前正处于连续驾驶状态时返回 true
 */
bool driveSet(int32_t rightCps, int32_t leftCps, bool holdHeading);

/**
 * @description         : 按已经校验的参数点动一只轮子
 * @param wheel         : 1 为 M1 右轮，2 为 M2 左轮
 * @param pwm           : 带方向的 PWM 代码值
 * @param durationMs    : 点动时长，ms
 * @param id            : 试跑编号
 * @return              : 点动启动时返回 true
 */
bool jog(int wheel, int pwm, int durationMs, int id);

/**
 * @description         : 按已经校验的三档设置启动双轮前进标定
 * @param settings      : 本次标定的编号和双轮 PWM 档位
 * @return              : 试跑启动时返回 true
 */
bool startPwmCalibration(const PwmCalibrationSettings &settings);

/**
 * @description         : 从 MCU 标定缓冲区读取连续样本
 * @param id            : 标定编号
 * @param offset        : 本页第一个样本的索引
 * @return              : 包含总行数、停止原因和本页样本的快照
 */
PwmCalibrationPage readPwmCalibration(int id, int offset);

/**
 * @description         : 从 100 Hz 控制拍 trace 环形缓冲读取连续样本
 * @param offset        : 本页第一个样本的索引，0 表示缓冲区里最旧的一拍
 * @return              : 含总条数、丢弃数和本页样本的快照
 */
TracePage readTrace(int offset);

/**
 * @description         : 从 1 Hz 环境样本缓冲读取连续记录
 * @param offset        : 本页第一个样本的索引
 * @return              : 含总条数和本页样本的快照
 */
EnvPage readEnv(int offset);

/**
 * @description         : 查询控制线程栈的剩余余量
 * @return              : 剩余字节数；内核未开启线程栈统计时返回 -1
 */
int32_t stackHeadroom();

/**
 * @description         : 清空 trace 与环境缓冲，并推进会话号
 * @return              : void
 */
void resetTrace();

/**
 * @description         : 冻结/解冻 trace 环形缓冲，供分页导出期间保持内容不变
 * @param hold          : true 时控制拍不再写入环形缓冲
 * @return              : void
 */
void setTraceHold(bool hold);

/**
 * @description         : 刷新正在运行且编号匹配的试跑心跳
 * @param id            : 试跑编号
 * @return              : 心跳接受时返回 true
 */
bool heartbeat(int id);

/**
 * @description         : 立即休眠驱动并确认停止状态
 * @return              : 驱动已休眠时返回 true
 */
bool stop();
bool resetDrive();

/**
 * @description         : 获取一个一致的控制器、编码器和 IMU 状态快照
 * @return              : 结构化遥测快照
 */
MotionStatus status();

/**
 * @description         : 驱动休眠且车轮停稳时请求重新校准 IMU
 * @return              : 校准请求接受时返回 true
 */
bool calibrateHeading();

/**
 * @description         : 读取全部运行期可调参数，索引见 TuneParam
 * @param out           : 长度至少 TuneParamCount 的数组
 * @return              : void
 */
void tuneRead(int32_t out[TuneParamCount]);

/**
 * @description         : 读取单个运行期参数，供 driveSettings 逐项取增益
 * @param index         : TuneParam 中的索引
 * @return              : 当前编码值，越界返回 0
 */
int32_t tuneValue(int index);

/**
 * @description         : 写入单个运行期参数，越界值被拒绝
 * @param index         : TuneParam 中的索引
 * @param value         : 按该参数编码规则量化后的整数
 * @return              : 数值在允许范围内并已写入时返回 true
 */
bool tuneWrite(int index, int32_t value);

/**
 * @description         : 把全部运行期参数恢复成出厂标定值
 * @return              : void
 */
void tuneReset();

} // namespace motion
