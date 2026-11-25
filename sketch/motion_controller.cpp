#include "motion_controller.h"

#include <Arduino.h>
#include <zephyr/kernel.h>
#include <algorithm>
#include <cmath>

#include "drive_start.h"
#include "motor_driver.h"
#include "speed_pid.h"
#include "vm_monitor.h"
#include "wheel_encoder.h"

namespace motion {
namespace {

// 没有收到有效心跳的允许时长，ms。
//
// 600 ms 对这块板子太紧：主控 CPU 被摄像头编码/推理占满时，控制环单拍能到
// 200~380 ms（实测 [lat] loop p95 200 ms、max 381 ms），连续两拍就顶穿
// 600 ms，固件自己把车停下 —— 现场表现是"跑着跑着突然不走了"，推杆还能再走，
// 但演示里看着就是失控。1.2 s 覆盖得住这种抖动；真的失联时 0.075 m/s 也只
// 多滑 9 cm。
// 底盘自主保持：上位机只负责改目标，不再靠每拍心跳续命。
// 实测上位机的航向流断过 3.6 s（串口错帧 + CPU 打满），1.2 s 的窗口会在
// 每次抖动时把车停下，紧接着的重新起转又要过 SETTLE_MS，最后表现为
// 命令发下去了、目标始终是 0、车不动。放到 3 s：0.05 m/s 也才多滑 15 cm。
constexpr uint32_t WATCHDOG_MS = 3000;
constexpr uint32_t DRIVE_HOLD_MS = 24u * 60u * 60u * 1000u; // 连续驾驶的名义时长上限，ms。
constexpr uint32_t SETTLE_MS = 300;            // 两轮停稳后才允许再启动的等待时间，ms。
// 换向瞬间车轮靠惯性继续前滚，新方向下这段位移是"倒转"，10 ms 内就能到几十个
// 计数。真正的编码器接反不会自行恢复，因此要求反向位移持续这么久才判故障：
// 惯性衰减通常 < 150 ms，仍比堵转保护的 600 ms 更早发现问题。
// 反向位移需持续的时长。换向那一拍实测能到 -34 计数，300 ms 太容易把
// "起步时轻微后退/编码器抖动"判成接反，放宽到 800 ms。
constexpr uint32_t REVERSE_HOLD_MS = 800;

// 编码器接反保护（已停用）。
//
// 现场不再需要它兜底：接反在装机时就能看出来（轮子转反、里程计倒退），
// 而运行中它只会在换向、打滑、单轮悬空时误报。误报的代价和堵转一样是
// 进 Mode::Fault 锁死整车，而 canStart() 只允许从 Idle 起步。
constexpr bool ENCODER_DIRECTION_GUARD = false;

// 编码器反馈保护（堵转/无进展）与故障锁存：默认关闭。
//
// 现场要的是"走不动就停一下，松手能立刻再走"，不是"一旦判故障就锁死整车"。
// 这两项在轮子打滑、单轮悬空、地毯阻力大、低速顶到障碍时都会误报；而误报
// 的代价是进 Mode::Fault —— canStart() 只允许从 Idle 起步，于是必须点一次
// "复位"才能再动，现场看起来就是"车突然死了"。
//
// 关掉之后的行为：
//   - 反馈保护：只保留计时器复位，不再因"占空比够高但没有进展"停车；
//   - 故障锁存：任何原因都只关输出、mode 回 Idle，下一次 start()/driveSet()
//     立刻可以再走。故障现场仍然照常记录（isReportable），遥测和 trace 不变。
// 驱动过流（nFAULT）、PWM 写入失败这类硬件问题依然会立即停车，只是不再锁存。
constexpr bool STALL_GUARD_ENABLED = false;
constexpr bool FAULT_LATCH_ENABLED = false;

// 只在实际带载起转附近的占空比开始计时；缓慢爬升不消耗窗口。
constexpr int STALL_ARM_DUTY = 175;
constexpr uint32_t DRIVER_FAULT_LOW_MS = 50;
constexpr uint32_t MAX_CONTROL_GAP_MS = 65;    // 运行中两次控制更新的最大间隔，ms。
constexpr uint32_t HEADING_QUIET_MS = 150;      // 停车时持续满足偏航角速度阈值的时间，ms。
constexpr uint32_t HEADING_SETTLE_MS = 600;     // 等待航向稳定的最长时间，ms。
constexpr uint32_t CONTROL_POLL_MS = 2;         // 控制线程检查周期，ms。
constexpr uint32_t PID_PERIOD_MS = 10;          // 增量式 PID 的定时更新周期，ms。
// 测速窗口长度（控制拍数）。10 ms 单拍测速在 600 计数/秒时只有 6 个计数，
// 一个计数的量化误差就是 100 计数/秒（16%），PID 的微分项等于在放大量化噪声，
// 积分项也分不清"真的慢了"还是"这一拍少了一个计数"。
// 取 5 拍（50 ms）：600 计数/秒 → 30 个计数，量化误差降到 20 计数/秒（3%）；
// 代价是测速滞后约 25 ms（滑动窗均值），对本环 1~2 Hz 的穿越频率可接受。
// 窗口内按"累加计数 / 累加时间"求速度，dt 在 10/11 ms 之间抖动也不会偏。
constexpr int SPEED_WINDOW_SAMPLES = 5;
// 起转补偿的最低目标速度。原来 300 会让"目标 200~300 cps 的低速起步"
// （导航靠近目标、微调位姿时的常见值）完全不补偿，车只能靠积分慢慢爬。
constexpr int BREAKAWAY_MIN_TARGET_CPS = 100;
constexpr int DRIVE_HEADING_MIN_WHEEL_CPS = 600;
constexpr int DRIVE_HEADING_MAX_IMU_AGE_MS = 50;
// 回正角速度上限，°/s。误差稍大时 kp 项会给出远超需要的修正，冲过头再反向修正，
// 正是左右摆动的来源；把它压在 4°/s 以内，阻尼项不参与限幅。
constexpr float MAX_RECOVER_RATE_DPS = 4.0f;

struct WheelFeedforward {
  float runningOffset;    // 运行曲线截距，PWM 代码值。
  float cpsPerDuty;        // 运行曲线斜率，计数/秒每 PWM 代码值。
  int breakawayDuty;       // 静摩擦起转占空比，PWM 代码值。
};

// 前馈标定值。原为 constexpr，改成运行期可写是为了让网页能在线调参。
//
// breakawayDuty 从 60 提到 160：实测这套车满占空比约 2400~2600 cps，即
// 约 10 cps/duty，而 13 + |v|/27.3 的斜率（27.3）比真实曲线陡 2.6 倍，
// 低速档前馈只有 38 左右。60 的起转占空比又远低于实测起转中位 168，
// 于是导航 0.08 m/s 起步时占空比要从 38 靠积分爬到 168，约 1.6 s ——
// 期间车纹丝不动，看起来就是"导航发了速度但车不走"。
constexpr WheelFeedforward DEFAULT_FEEDFORWARD = {13.0f, 27.3f, 160};
// M1 右轮、M2 左轮。每轮依次为前进和倒车，四份统一由 tuneWrite 一起改写。
WheelFeedforward driveFeedforward[2][2] = {
    {DEFAULT_FEEDFORWARD, DEFAULT_FEEDFORWARD},
    {DEFAULT_FEEDFORWARD, DEFAULT_FEEDFORWARD},
};
// 前馈安全系数与起转抬升上限，同样开放到调参接口。
float driveFfSafety = 0.95f;
// 允许的抬升量必须覆盖 160 - 前馈(38) ≈ 122；留到 150 是为了网页调参时
// 还能往上加一点。原来 30 让补偿最多只到 68，等于没有起转补偿。
int driveBreakawayMaxBoost = 150;
// 速度环增益与斜坡。原本放在 motion_bridge.cpp，参数化后集中到这里，
// 与 tuneRead/tuneWrite 同源；motion_bridge 通过 tuneValue() 取值。
int32_t driveKpX10000 = 20000;  // 轮速比例增益 2.0。
int32_t driveKiX10000 = 5000;   // 轮速积分增益 0.5，时间常数约 0.42 s。
int32_t driveKdX10000 = 0;      // 轮速微分增益 0（二阶差分放大噪声，实测无收益）。
int32_t driveRampCpsPerS = 6000; // 轮速斜坡，计数/s²。
// 直行保向（MCU 本地执行）。原先是 motion_bridge 里写死的 1.85 与结构体默认值，
// 一并开放到调参接口，默认值保持不变。
// 直行抑摆那一版实测过的值：1.0 / 0.50 / 220。不要改回 1.85 / 0.25 / 300，
// 那组是未抑摆前的原始默认值，会重新引入左右摆动。
int32_t driveHeadingKpX1000 = 1000;   // 比例增益 1.0，约 43 count/s/°。
int32_t driveHeadingKdX1000 = 500;    // 角速度反馈 0.50，加重阻尼压摆动。
int32_t driveHeadingLimitCps = 220;   // 单轮修正上限，计数/s。

constexpr int64_t PID_PERIOD_TICKS = k_ms_to_ticks_ceil64(PID_PERIOD_MS); // PID 更新间隔，内核刻度。
constexpr int64_t CONTROL_POLL_TICKS = k_ms_to_ticks_ceil64(CONTROL_POLL_MS); // 保护检查间隔，内核刻度。
constexpr size_t CONTROL_STACK_BYTES = 4096;   // 控制线程栈大小，字节。
constexpr int CONTROL_PRIORITY = 4;             // Zephyr 控制线程优先级。
constexpr int DEFAULT_RAMP_CPS_PER_S = 2000;     // 默认轮速斜率，计数/s²。
// PI 使用五拍计数窗口，额外的低通仅用于遥测和故障快照。
constexpr float SPEED_FILTER_TAU_S = 0.08f;     // 遥测轮速一阶滤波时间常数，s。
constexpr float QUIET_RATE_DPS = 2.0f;          // 判定航向稳定的偏航角速度阈值，°/s。
constexpr float TRACK_WIDTH_M = 0.1535f;        // 左右轮中心距，m。
constexpr float WHEEL_DIAMETER_M = 0.065f;      // 车轮标称直径，m。
constexpr float COUNTS_PER_WHEEL_REV = 6608.0f; // 按减速比和四倍频计算的每圈编码器计数。
constexpr float WHEEL_MM_PER_COUNT =
    3.14159265f * 1000.0f * WHEEL_DIAMETER_M / COUNTS_PER_WHEEL_REV; // 轮上每个编码器计数对应的行程，mm。
constexpr float UNO_PWM_PER_YAHBOOM_UNIT = float(MAX_PID_PWM) / 3600.0f; // 两套 PWM 满量程的比例。
constexpr float YAW_CPS_PER_DPS =
    (TRACK_WIDTH_M * COUNTS_PER_WHEEL_REV) / (360.0f * WHEEL_DIAMETER_M); // 偏航角速度到单轮修正的换算。
constexpr int defaultEncoderSigns[] = {-1, 1};     // M1 右轮、M2 左轮的默认计数换向符号。
constexpr int startupDuty[][2] = {{46, 48}, {51, 51}}; // 两轮前进、后退的堵转检测 PWM 起点。

constexpr uint32_t PWM_CALIBRATION_MAX_MS =
    PWM_CALIBRATION_STAGES * PWM_CALIBRATION_MAX_STAGE_MS; // 允许的最长标定时限，ms。
constexpr int PWM_CALIBRATION_ROWS =
    PWM_CALIBRATION_MAX_MS / PERIOD_MS; // 按最长时限和最短采样周期预留的样本数。
static_assert(PWM_CALIBRATION_MAX_MS <= UINT16_MAX, "PWM calibration timestamp fits elapsedMs");

struct HeadingControl {
  HeadingMode mode = HeadingMode::Wheel; // 当前航向控制模式。
  StopReason finishReason = Complete;   // 减速停稳后报告的结束原因。
  float origin = 0;                      // 起步时 IMU 的累计航向，°。
  float target = 0;                      // 相对起步方向的目标航向，°。
  float yaw = 0;                         // 相对起步方向的实测航向，°。
  float error = 0;                       // 目标航向减实测航向，°。
  float kp = 2.5f;                       // 航向比例增益，1/s。
  float kd = 0.25f;                      // 航向角速度反馈增益，无量纲。
  float correction = 0;                  // M1 轮速修正量，计数/s。
  int correctionLimit = 300;            // 单轮直行修正上限，计数/s。
  uint32_t rampUpMs = 0;                 // 加速阶段时限，ms。
  uint32_t settleAt = 0;                 // 进入停稳阶段时的 MCU 时间，ms。
  uint32_t quietAt = 0;                  // 偏航角速度进入稳定范围的时间，ms，0 表示未稳定。
};

struct Wheel {
  SpeedPid pid;                // 单轮 PID 的增益和动态状态。
  int32_t gainCodes[3] = {};    // 按 10000 倍编码的 Kp、Ki、Kd。
  int32_t target = 0;          // 带方向的轮速目标，计数/s。
  int sign = 1;                // 编码器计数转换到前进方向的符号。
  int pwm = 0;                 // 当前带方向的 PWM 代码值。
  int deadzonePwm = 0;         // 非零 PID 输出附加的单轮死区补偿，UNO Q PWM 代码值。
  int ffDuty = 0;              // 按目标速度估算的前馈占空比，随斜坡按比例淡入。
  int ffApplied = 0;           // 本拍实际叠加进 duty 的前馈，PWM 代码值，供 trace 使用。
  float ramp = 0;              // 当前带方向的斜坡速度参考，计数/s。
  float speed = 0;             // 带方向的滤波轮速，计数/s。
  // 测速滑动窗口：deltaHist 存每拍的原始计数增量，dtUsHist 存对应采样间隔（µs）。
  // 两者一起累加，才能用"总计数/总时间"求速度；只累加计数会把 10/11 ms 的
  // 周期抖动直接算成速度抖动。
  int16_t deltaHist[SPEED_WINDOW_SAMPLES] = {};  // 各拍原始增量，计数。
  uint16_t dtUsHist[SPEED_WINDOW_SAMPLES] = {};  // 各拍采样间隔，µs。
  int32_t deltaSum = 0;        // 窗口内增量之和，计数。
  uint32_t dtUsSum = 0;        // 窗口内时间之和，µs。
  int histIndex = 0;           // 环形写指针。
  int histCount = 0;           // 窗口内已填充的拍数，起转初期小于窗口长度。
  bool kickApplied = false;    // 起转补偿刚施加过，等待检测到正向进展。
  bool kickPending = false;    // 本次起步尚未施加静摩擦起转占空比。
  DriveStartAssist driveStart;  // ROS 驾驶每轮的一次性起转补偿。
  uint32_t previousCount = 0;  // 上次控制更新时的原始编码器累计计数。
  EncoderProgressGuard progressGuard;
  uint32_t wrongSince = 0;     // 开始出现持续反向位移的 MCU 时间，ms，0 表示当前没有。
  int64_t position = 0;        // 本轮试跑沿目标方向的相对编码器计数。
  int64_t furthestPosition = 0; // 本轮试跑沿目标方向到达的最大计数。
};

struct DeadzoneTrial {
  DeadzoneStatus status;      // 最近一次点动的状态与结果。
  uint32_t startCount = 0;    // 点动起始时的原始编码器计数。
  uint32_t startInvalid = 0;  // 点动起始时的无效跳变累计次数。
};

struct PwmCalibrationTrial {
  PwmCalibrationSettings settings;                // 当前或最近一次标定的编号和 PWM 设置。
  PwmCalibrationSample rows[PWM_CALIBRATION_ROWS]; // 固定容量的采样缓冲区。
  int count = 0;                                 // 已记录的样本数。
  int stage = 0;                                 // 正在施加的档位，从 0 开始。
  int stopReason = -1;                           // 停止原因，-1 表示仍在运行。
};

PwmCalibrationTrial pwmCalibration;   // 标定样本保存到下一次标定开始。
Wheel wheels[2];                      // M1 右轮、M2 左轮。
DeadzoneTrial deadzone;                // 最近一次单轮点动记录。
FaultSnapshot fault;                   // 最近故障触发时的状态。
bool driverFaultLow = false;
uint32_t driverFaultLowAt = 0;
HeadingControl heading;                // 当前航向配置及动态状态。
bool driveHoldHeading = false;          // 普通驾驶的直行保向请求。

// 增量式速度环增益：误差单位是编码器计数/s，输出是 PWM 代码值 0~255。
// 参照亚博 M1 的 PID_Incre_Calc（Kp 0.8 / Ki 0.06 / Kd 0.5，误差 mm/s、
// PWM 量程 0~3600）折算：1 mm/s 约 32.4 counts/s，PWM 量程 255/3600。
// Kd 先取 0：编码器 10 ms 差分噪声大，微分项只会放大量化抖动。
// 增益不再写死：由在线调参的 Kp/Ki/Kd 折算成增量式系数。
// 折算系数按"默认值复现原来手调的 0.10 / 0.004 / 0"标定：
//   speed_kp=2.0  -> tuneValue 20000 -> 20000*5e-6 = 0.10
//   speed_ki=0.5  -> tuneValue  5000 ->  5000*8e-7 = 0.004
//   speed_kd=0    ->                    0
// 这样调参面板改的就是实际参与输出的增益，不会再出现"改了不生效"。
constexpr float GAIN_KP_SCALE = 5.0e-6f;
constexpr float GAIN_KI_SCALE = 8.0e-7f;
constexpr float GAIN_KD_SCALE = 5.0e-6f;

float incErr1[2] = {0.0f, 0.0f};   // e(k-1)
float incErr2[2] = {0.0f, 0.0f};   // e(k-2)
// 浮点累加器：原来用 int 存上一拍输出、每拍把 delta 取整后累加，
// 小于 1 码的积分增量会被永久丢掉，低增益时表现为"永远起不来"。
// 现在只在最终写 PWM 时取整，并把饱和后的实际值回写，避免积分饱和。
float incPwmF[2] = {0.0f, 0.0f};

/**
 * @description         : 清空某一轮（或两轮）的速度环动态状态
 * @param index         : 0/1 指定轮；-1 表示两轮
 *
 * 所有停车、故障、复位、重新起转的路径都必须调用它。漏掉一处，残留的
 * 历史误差和累加器就会让下一次起转带着旧状态，表现为"停一次之后再也走不准"。
 */
void resetSpeedLoop(int index) {
  if (index < 0) {
    resetSpeedLoop(0);
    resetSpeedLoop(1);
    return;
  }
  incErr1[index] = 0.0f;
  incErr2[index] = 0.0f;
  incPwmF[index] = 0.0f;
  wheels[index].pid.reset();
}
bool pwmReady = false;                 // 四路 PWM 已初始化且未发生写入错误。
bool workerReady = false;              // 已分配轮速线程栈。
struct k_mutex stateMutex;             // 串行化 Bridge API 和轮速线程对状态及输出的访问。
struct k_thread controlThread;         // Zephyr 轮速控制线程。
k_thread_stack_t *controlStack = nullptr; // 轮速控制线程栈。
Mode mode = Mode::Idle;                 // 控制器空闲、运行或故障状态。
int runId = 0;                          // 当前试跑编号。
int pwmLimit = MAX_PID_PWM;             // 本次试跑的单轮 PWM 代码值上限。
StopReason reason = Boot;               // 当前状态对应的原因。
RunKind runKind = IdleRun;              // 当前试跑的类型。
MotionPhase motionPhase = MotionPhase::Idle; // 当前闭环运行阶段。
int rampCpsPerS = DEFAULT_RAMP_CPS_PER_S; // 本次试跑的轮速斜率，计数/s²。
uint32_t holdDuration = 0;             // 保速时长或定角转向时限，ms。
uint32_t holdStarted = 0;              // 开始保速时的 MCU 时间，ms。
uint32_t runStarted = 0;               // 试跑启动时的 MCU 时间，ms。
uint32_t runDuration = 0;              // 整次试跑的总时限，ms。
uint32_t heartbeatAt = 0;              // 最近有效心跳的 MCU 时间，ms。
uint32_t tickAt = 0;                   // 上次轮速控制更新的 MCU 时间，ms。
int64_t pidSampleTick = 0;             // 最近一次闭环编码器采样的内核时钟刻度。
int64_t nextPidTick = 0;               // 下一次闭环控制的绝对时钟刻度。
uint32_t stoppedAt = 0;                // 最近停止输出的 MCU 时间，ms。
uint32_t lastEncoderMotionAt = 0;      // 最近检测到车轮运动的 MCU 时间，ms。
uint32_t sequence = 0;                 // 遥测状态更新序号。

TraceSample traceRing[TRACE_SAMPLE_MAX]; // 100 Hz 控制拍环形缓冲，控制线程单写。
TraceSample pendingTrace;              // 本拍正在填充的样本。
bool pendingTraceValid = false;        // 本拍已开始、尚未提交。
uint32_t traceHead = 0;                // 下一个写入位置。
uint32_t traceTotal = 0;               // 已写入样本总数，含被覆盖的。
bool traceHold = false;                // 导出期间冻结环形缓冲：分页读到的是同一段数据。
uint32_t traceEpoch = 1;               // trace 会话号，resetTrace 时递增。
EnvSample envRing[ENV_SAMPLE_MAX];     // 1 Hz 环境样本（温度、零偏）。
uint32_t envHead = 0;                  // 下一个写入位置。
uint32_t envTotal = 0;                 // 已写入环境样本总数。
uint32_t envLastAt = 0;                // 上次环境采样的 MCU 时间，ms。

/**
 * @description         : 在控制拍开始时填充 trace 的静态字段
 * @param now           : 本拍 MCU 时间，ms
 * @param elapsed       : 距上一拍的实际间隔，ms
 * @param measured      : 本拍原始实测轮速，计数/s
 * @return              : void
 */
void traceStart(uint32_t now, uint32_t elapsed, const float (&measured)[2]) {
  pendingTrace = {};
  pendingTrace.tickMs = now;
  pendingTrace.seq = uint16_t(sequence & 0xFFFFu);
  pendingTrace.dtMs = uint8_t(std::min<uint32_t>(elapsed, 255));
  if (mode != Mode::Active) pendingTrace.flags |= TRACE_NOT_ACTIVE;
  if (mode == Mode::Fault) pendingTrace.flags |= TRACE_FAULT;
  for (int i = 0; i < 2; ++i) {
    pendingTrace.target[i] = int16_t(std::clamp<int32_t>(wheels[i].target, -32767, 32767));
    pendingTrace.ramp[i] = int16_t(std::clamp<int32_t>(
        int32_t(std::lround(wheels[i].ramp)), -32767, 32767));
    pendingTrace.speed[i] = int16_t(std::clamp<int32_t>(
        int32_t(std::lround(measured[i])), -32767, 32767));
  }
  pendingTraceValid = true;
}

/**
 * @description         : 提交本拍 trace：补齐输出与传感器字段后定长写入环形缓冲
 * @note                : 只做整型搬运，不做分配、不做格式化；控制线程每拍一次。
 * @return              : void
 */
void traceCommit() {
  if (!pendingTraceValid || traceHold) return;
  pendingTraceValid = false;

  const ImuHeadingSample imu = headingImuRead();
  pendingTrace.rateDpsX100 = int16_t(std::clamp<long>(std::lround(imu.rateDegPerS * 100.0f),
                                                      -32767L, 32767L));
  pendingTrace.imuAgeMs = uint8_t(std::clamp<int32_t>(imu.ageMs, 0, 255));
  if (imu.state != ImuHeadingState::Ready) pendingTrace.flags |= TRACE_IMU_NOT_READY;
  if (imu.ageMs > 50) pendingTrace.flags |= TRACE_IMU_STALE;

  const VmMonitorSample vm = vmMonitorRead();
  pendingTrace.vbatRaw = vm.raw > 0 ? uint16_t(vm.raw) : uint16_t(0);

  for (int i = 0; i < 2; ++i) {
    pendingTrace.pwm[i] = int16_t(std::clamp<int32_t>(wheels[i].pwm, -32767, 32767));
    pendingTrace.ffDuty[i] = uint8_t(std::clamp<int32_t>(wheels[i].ffApplied, 0, 255));
    if (wheels[i].target == 0) {
      pendingTrace.flags |= (i == 0 ? TRACE_ZERO_M1 : TRACE_ZERO_M2);
    } else if (std::abs(wheels[i].pwm) >= pwmLimit) {
      pendingTrace.flags |= (i == 0 ? TRACE_SAT_M1 : TRACE_SAT_M2);
    }
  }

  traceRing[traceHead] = pendingTrace;
  traceHead = (traceHead + 1) % uint32_t(TRACE_SAMPLE_MAX);
  if (traceTotal < UINT32_MAX) ++traceTotal;
}

/**
 * @description         : 低频（1 Hz）记录 IMU 温度与零偏，供温漂分析
 * @param now           : 当前 MCU 时间，ms
 * @return              : void
 */
void envSample(uint32_t now) {
  if (envLastAt != 0 && now - envLastAt < 1000) return;
  envLastAt = now;
  const ImuHeadingSample imu = headingImuRead();
  EnvSample sample;
  sample.tickMs = now;
  sample.seq = uint16_t(envTotal & 0xFFFFu);
  sample.tempCx100 = int16_t(std::clamp<long>(std::lround(imu.temperatureC * 100.0f),
                                              -32767L, 32767L));
  sample.biasDpsX1000 = int16_t(std::clamp<long>(std::lround(imu.biasDegPerS * 1000.0f),
                                                 -32767L, 32767L));
  sample.state = uint8_t(imu.state);
  envRing[envHead] = sample;
  envHead = (envHead + 1) % uint32_t(ENV_SAMPLE_MAX);
  if (envTotal < UINT32_MAX) ++envTotal;
}

/**
 * @description         : 根据编码器快照更新开环点动记录
 * @param trial         : 当前点动记录
 * @param now           : MCU 时间，单位为毫秒
 * @param counts        : 两轮原始计数
 * @param invalid       : 两轮无效跳变次数
 * @return              : void
 */
void sampleDeadzone(DeadzoneStatus &trial, uint32_t now, const uint32_t (&counts)[2],
                    const uint32_t (&invalid)[2]) {
  const int index = trial.wheel - 1;
  trial.elapsed = now - runStarted;
  trial.delta = int32_t(counts[index] - deadzone.startCount);
  trial.invalid = invalid[index] - deadzone.startInvalid;
}

/**
 * @description         : 在输出清零前记录故障触发时的驱动和编码器状态
 * @param cause         : 故障原因
 * @param index         : 电机索引，负值表示整车故障
 * @param pwm           : 触发时的带符号 PWM 代码值
 * @param progress      : 触发周期沿目标方向的编码器增量
 * @return              : void
 */
void captureFault(StopReason cause, int index, int pwm, int32_t progress) {
  fault = {};
  fault.reason = cause;
  fault.uptime = millis();
  fault.wheel = index + 1;
  fault.requestedPwm = pwm;
  fault.progress = progress;
  fault.awake = motorIsAwake();
  fault.faultPinHigh = !motorHasFault();
  fault.inputLevels = motorInputLevels();
  const VmMonitorSample vm = vmMonitorRead();
  fault.vmRaw = vm.raw;
  fault.vmMinRaw = vm.minRaw;
  fault.vmAgeMs = vm.ageMs;

  uint32_t counts[2], invalid[2];
  readCounters(counts, invalid);
  for (int i = 0; i < 2; ++i) {
    const Wheel &wheel = wheels[i];
    fault.wheels[i] = {wheel.target, wheel.speed, wheel.pwm, counts[i], invalid[i]};
  }
}

// 需要记录故障现场的原因。只用于遥测/日志，不再决定是否锁存。
bool isReportable(StopReason cause) {
  return cause == DriverFault || cause == EncoderWrong || cause == EncoderStall ||
    cause == Timing || cause == PwmError || cause == ImuLost;
}

bool isLatchedFault(StopReason cause) {
  return cause == DriverFault || cause == EncoderWrong || cause == EncoderStall ||
    cause == PwmError || (cause == Timing && !workerReady);
}

/**
 * @description         : 关闭驱动并保留停止原因和故障数据
 * @param cause         : 停止原因
 * @return              : void
 */
void stopOutputs(StopReason cause, int index = -1, int pwm = 0, int32_t progress = 0) {
  // 同一个原因反复收到 stop 时只记录第一次现场，避免快照被覆盖成"最后一拍"。
  // 是否记录与是否锁存无关：所有报告类原因都留快照，方便事后看 trace。
  const bool captured = isReportable(cause)
      && (mode != Mode::Fault || cause != reason);
  if (captured) captureFault(cause, index, pwm, progress);
  motorSetAwake(false);
  driverFaultLow = false;
  // 普通停车、急停、故障停车、失联停车全部汇合到这里：一并清空速度环的
  // 误差历史与浮点累加器。少清一处，下一次起转就会带着上一次的残留状态
  // （MCU-002/003：换向、重新起转后速度对不上，就是这里漏清导致的）。
  resetSpeedLoop(-1);
  const bool endingDeadzone = mode == Mode::Active && runKind == DeadzoneRun;
  if (endingDeadzone) {
    uint32_t counts[2], invalid[2];
    readCounters(counts, invalid);
    sampleDeadzone(deadzone.status, millis(), counts, invalid);
  }
  if (!motorDisableOutputs()) pwmReady = false;
  if (!pwmReady) cause = PwmError;
  if (!workerReady && cause != PwmError) cause = Timing;
  const bool faulted = isLatchedFault(cause);
  if (isReportable(cause) && !captured && cause != reason) {
    captureFault(cause, index, pwm, progress);
  }
  for (Wheel &wheel : wheels) {
    wheel.pwm = 0;
    wheel.ramp = 0;
    // 目标也必须清零：它是遥测里的 target 字段（见 status()）。停车后留着
    // 上一次的目标，会出现 target=1494 而 speed=0/pwm=0/mode=Idle 的自相矛盾
    // 快照，任何人看遥测或导出的 CSV 都会误判成"车还在转"。
    // 下一次驱动由 start()/driveSet() 重新写入，这里清掉不影响控制。
    wheel.target = 0;
    wheel.pid.reset();
    wheel.kickPending = false;   // 下次起步重新允许起转一脚
    wheel.kickApplied = false;
    wheel.driveStart.reset();
    wheel.progressGuard.reset();
  }
  if (endingDeadzone) deadzone.status.stopReason = cause;
  if (mode == Mode::Active && runKind == PwmCalibrationRun) pwmCalibration.stopReason = cause;
  motionPhase = MotionPhase::Idle;
  driveHoldHeading = false;
  if (runKind == DriveRun) heading.mode = HeadingMode::Wheel;
  heading.correction = 0;
  // 故障只停车、不锁存：mode 始终回 Idle，canStart() 立刻允许再起步。
  // 需要"判故障就锁死整车"时把 FAULT_LATCH_ENABLED 打开（见文件头）。
  mode = (FAULT_LATCH_ENABLED && faulted) ? Mode::Fault : Mode::Idle;
  reason = cause;
  runId = 0;
  stoppedAt = millis();
  ++sequence;
}

/**
 * @description         : 检查驱动故障、运行时限和通信看门狗
 * @param now           : MCU 时间，单位为毫秒
 * @return              : void
 */
void checkStop(uint32_t now) {
  if (mode != Mode::Active) return;
  if (motionPhase != MotionPhase::Settle) {
    if (!motorIsAwake()) {
      stopOutputs(DriverFault);
      return;
    }
    if (motorHasFault()) {
      if (!driverFaultLow) {
        driverFaultLow = true;
        driverFaultLowAt = now;
      } else if (now - driverFaultLowAt >= DRIVER_FAULT_LOW_MS) {
        stopOutputs(DriverFault);
        return;
      }
    } else {
      driverFaultLow = false;
    }
  }
  // 遥控连续驾驶没有固定时限，只由故障、看门狗和显式停止结束。
  if (runKind != DriveRun && now - runStarted >= runDuration) {
    stopOutputs(runKind == PwmCalibrationRun ? Complete : Expired);
  }
  else if (now - heartbeatAt >= WATCHDOG_MS) stopOutputs(Watchdog);
}

/**
 * @description         : 用共同插值比例更新两轮速度参考
 * @param goals         : 两轮带符号的速度目标，单位为计数每秒
 * @param dt            : 本次实际控制周期，单位为秒
 * @return              : 两轮均达到本阶段目标时返回 true
 */
bool advanceRamps(const float (&goals)[2], float dt) {
  const float maxGap = std::max(std::fabs(goals[0] - wheels[0].ramp),
                                std::fabs(goals[1] - wheels[1].ramp));
  const float step = float(rampCpsPerS) * dt;
  const bool reached = maxGap <= step;
  const float fraction = reached ? 1.0f : step / maxGap;
  for (int i = 0; i < 2; ++i) {
    wheels[i].ramp = reached ? goals[i] :
        wheels[i].ramp + (goals[i] - wheels[i].ramp) * fraction;
  }
  return reached;
}

/**
 * @description         : 检查单轮编码器方向和堵转保护
 * @param index         : 电机索引，0 为 M1 右轮，1 为 M2 左轮
 * @param direction     : 目标方向，前进为 1，后退为 -1
 * @param duty          : PWM 代码值
 * @param progress      : 本周期沿目标方向的编码器增量
 * @param now           : MCU 时间，ms
 * @param monitorDirection : 是否累计并检查反向位移
 * @return              : 未触发保护时返回 true
 */
bool checkWheelProgress(int index, int direction, int duty, int32_t progress,
                        uint32_t now, bool monitorDirection) {
  Wheel &wheel = wheels[index];
  bool advancing = progress > 0;
  if (monitorDirection) {
    wheel.position += progress;
    advancing = wheel.position > wheel.furthestPosition;
    if (advancing) wheel.furthestPosition = wheel.position;
    // 单向棘轮：反向位移超过阈值只开始计时，持续 REVERSE_HOLD_MS 才判接反。
    // 只看瞬时值会把"目标变号后车轮还带着惯性前滚"当成接反（实测换向一拍就是
    // -34 计数）。真正接反时轮子不会有任何一拍沿命令方向走，因此只要出现一拍
    // progress > 0 就撤销计时 —— 惯性前滚无论多长都不会累积成故障。
    //
    // 该保护默认关闭（ENCODER_DIRECTION_GUARD）：运行中误报的代价是整车锁死，
    // 而它要抓的问题（编码器接反）在装机和第一次手动试车时就能看出来。
    if (ENCODER_DIRECTION_GUARD) {
      if (progress > 0) {
        wheel.wrongSince = 0;
      } else if (wheel.furthestPosition - wheel.position >= 6) {
        if (!wheel.wrongSince) wheel.wrongSince = now;
        if (now - wheel.wrongSince >= REVERSE_HOLD_MS) {
          stopOutputs(EncoderWrong, index, direction * duty, progress);
          return false;
        }
      } else {
        wheel.wrongSince = 0;   // 位移回到临界区以内，撤销计时
      }
    }
  }

  const bool aboveThreshold = duty >= startupDuty[index][direction < 0 ? 1 : 0]
      && duty >= STALL_ARM_DUTY;
  // 反馈保护关闭时这里只是空转一次计时器复位（见文件头 STALL_GUARD_ENABLED），
  // 不再因为"占空比够高但没进展"把车停下。
  if (STALL_GUARD_ENABLED && wheel.progressGuard.expired(aboveThreshold, advancing, now)) {
    stopOutputs(EncoderStall, index, direction * duty, progress);
    return false;
  }
  return true;
}

/**
 * @description         : 写入单轮 PWM 并在失败时立即关闭驱动
 * @param index         : 电机索引，0 为 M1 右轮，1 为 M2 左轮
 * @param direction     : 目标方向，前进为 1，后退为 -1
 * @param duty          : PWM 代码值
 * @param progress      : 本周期沿目标方向的编码器增量
 * @return              : PWM 已成功写入时返回 true
 */
bool writeWheelDuty(int index, int direction, int duty, int32_t progress) {
  if (!applyDuty(driveInput(index, direction), duty)) {
    pwmReady = false;
    stopOutputs(PwmError, index, direction * duty, progress);
    return false;
  }
  wheels[index].pwm = direction * duty;
  return true;
}

/**
 * @description         : 清空单轮测速滑动窗口
 * @param wheel         : 待清空的车轮状态
 * @return              : void
 *
 * 起转、换向、零目标停车后必须清空：窗口里留着上一段的计数，会让"刚起步"
 * 的第一拍就报出一个几十毫秒之前的速度，PID 按它算出来的第一拍增量是错的。
 */
void clearSpeedWindow(Wheel &wheel) {
  for (int i = 0; i < SPEED_WINDOW_SAMPLES; ++i) {
    wheel.deltaHist[i] = 0;
    wheel.dtUsHist[i] = 0;
  }
  wheel.deltaSum = 0;
  wheel.dtUsSum = 0;
  wheel.histIndex = 0;
  wheel.histCount = 0;
}

/**
 * @description         : 用实测轮速更新增量式 PID，再叠加该轮死区补偿
 * @param index         : 电机索引，0 为 M1 右轮，1 为 M2 左轮
 * @param measuredCps   : 本拍带符号的编码器轮速，计数/s
 * @param goalCps       : 航向修正后的轮速目标，计数/s
 * @param differentialPwm : 沿驱动方向施加的差动 PWM 修正
 * @param now             : MCU 时间，ms
 * @return              : 沿目标方向的 UNO Q PWM 代码值
 */
int pidWheelDuty(int index, float measuredCps, float goalCps, int differentialPwm, uint32_t now) {
  Wheel &wheel = wheels[index];
  const float targetCps = std::fabs(wheel.ramp);
  wheel.ffApplied = 0;
  (void)goalCps;   // 增量式只用本拍斜坡目标，保向差动另走 differentialPwm
  if (targetCps == 0 || wheel.target == 0) {
    resetSpeedLoop(index);
    return 0;
  }

  const int direction = wheel.target > 0 ? 1 : -1;
  const float alongSpeed = measuredCps * direction;

  // 增量式速度环，结构照搬亚博 M1 的 PID_Incre_Calc：
  //     pwm += Kp*(e - e1) + Ki*e + Kd*(e - 2*e1 + e2)
  // 为什么换掉原来的 前馈 + 位置式 PI：
  //     前馈模型（duty = 13 + v/slope）是按空载标定的，slope 27.3 对应地面
  //     实测只需要 7，差 4 倍。位置式 PI 于是要积 137 码去补这个缺口 ——
  //     补到一半轮子还没转，补满才冲开，冲开后积分又过大导致掉速，表现就是
  //     "冲一下停一下"。增量式不存积分、只累加增量，负载怎么变就怎么跟，
  //     既不需要前馈曲线，也不存在积分饱和后回不来的问题。
  // 增益从在线调参取，单位换算见 GAIN_*_SCALE 的注释。
  const float kp = float(tuneValue(TuneKp)) * GAIN_KP_SCALE;
  const float ki = float(tuneValue(TuneKi)) * GAIN_KI_SCALE;
  const float kd = float(tuneValue(TuneKd)) * GAIN_KD_SCALE;

  const float err = targetCps - alongSpeed;             // 误差，编码器计数/s
  const float delta = kp * (err - incErr1[index])
                    + ki * err
                    + kd * (err - 2.0f * incErr1[index] + incErr2[index]);
  incErr2[index] = incErr1[index];
  incErr1[index] = err;

  if (!std::isfinite(delta)) {   // 编码器异常可能给出 inf/nan，宁可停车
    resetSpeedLoop(index);
    return 0;
  }

  // 浮点累加：小于 1 码的增量也保留，低增益时才能慢慢爬上去。
  float accum = incPwmF[index] + delta;
  accum = std::clamp(accum, 0.0f, float(pwmLimit));
  incPwmF[index] = accum;        // 饱和时回写，等价于条件积分限幅
  const int duty = std::clamp(int(std::lround(accum)), 0, pwmLimit);
  wheel.ffApplied = duty;
  // 保向差动是叠加在本拍输出上的独立修正。
  return std::clamp(duty + differentialPwm, 0, pwmLimit);
}

/**
 * @description         : 检查单轮编码器反馈并写入最终的闭环 PWM
 * @param index         : 电机索引，0 为 M1 右轮，1 为 M2 左轮
 * @param delta         : 本周期原始编码器增量
 * @param duty          : 当前方向的最终 PWM 代码值
 * @param now           : MCU 时间，单位为毫秒
 * @return              : 未触发保护且 PWM 已更新时返回 true
 */
bool applyPidWheel(int index, int32_t delta, int duty, uint32_t now) {
  Wheel &wheel = wheels[index];
  if (wheel.target == 0) {
    // 目标为零的轮必须显式撤销驱动。原实现直接 return true，于是"一轮有速度、
    // 另一轮目标为 0"时（低速急转可解出 (647, 0)，见 control.wheel_targets），
    // 零目标轮的 PWM 停在上一拍的值：车会持续单边拉转，遥测里的 pwm 也是旧的，
    // 看日志会以为"这一轮已经停了"。这里改成主动写 0 并复位该轮的控制状态。
    wheel.ramp = 0;
    wheel.ffApplied = 0;
    wheel.pid.reset();      // 再起转时从零开始累加，不带旧积分
    clearSpeedWindow(wheel); // 再起转时窗口里不能留着上一段的计数
    wheel.kickApplied = false;
    wheel.kickPending = false;
    wheel.driveStart.reset();
    wheel.progressGuard.reset();
    wheel.wrongSince = 0;
    if (wheel.pwm == 0) return true;   // 已经为零就不重复写寄存器
    // 此处方向没有意义：applyDuty(…, 0) 只清当前方向对应的那一路输出。
    if (!applyDuty(driveInput(index, 1), 0)) {
      pwmReady = false;
      stopOutputs(PwmError, index);
      return false;
    }
    wheel.pwm = 0;
    return true;
  }

  const int direction = wheel.target > 0 ? 1 : -1;
  const int32_t progress = int32_t(int64_t(delta) * wheel.sign * direction);
  const bool monitorDirection = motionPhase != MotionPhase::RampDown && wheel.ramp != 0;
  return checkWheelProgress(index, direction, duty, progress, now, monitorDirection) &&
         writeWheelDuty(index, direction, duty, progress);
}

/**
 * @description         : 断开 PWM，等待车体停稳后读出最终航向
 * @param now           : MCU 时间，单位为毫秒
 * @return              : void
 */
void beginHeadingSettle(uint32_t now) {
  motorSetAwake(false);
  for (int i = 0; i < 4; ++i) {
    if (!applyDuty(i, 0)) {
      pwmReady = false;
      stopOutputs(PwmError);
      return;
    }
  }
  for (Wheel &wheel : wheels) wheel.pwm = 0;
  heading.correction = 0;
  heading.settleAt = now;
  heading.quietAt = 0;
  motionPhase = MotionPhase::Settle;
}

/**
 * @description         : 根据航向误差和角速度限制单轮差速修正
 * @param magnitude     : 两轮共同速度幅值，单位为计数每秒
 * @param yawRate       : 偏航角速度，单位为度每秒
 * @return              : M1 的速度修正，M2 使用相反值，单位为计数每秒
 */
float straightHeadingCorrection(float magnitude, float yawRate) {
  const float limit = std::min(float(heading.correctionLimit), 0.30f * magnitude);
  const float recover = std::clamp(heading.kp * heading.error,
                                   -MAX_RECOVER_RATE_DPS, MAX_RECOVER_RATE_DPS);
  const float desired = (recover - heading.kd * yawRate) * YAW_CPS_PER_DPS;
  return std::clamp(desired, -limit, limit);
}

// 普通驾驶使用本地 IMU。失去有效样本时回到基础轮速，恢复后重新锁航向。
void driveHeadingGoals(float (&goals)[2]) {
  const float magnitude = std::fabs(goals[0]);
  if (!driveHoldHeading || goals[0] != goals[1] ||
      magnitude <= DRIVE_HEADING_MIN_WHEEL_CPS || motionPhase == MotionPhase::RampDown) {
    heading.mode = HeadingMode::Wheel;
    heading.correction = 0;
    return;
  }

  const ImuHeadingSample imu = headingImuRead();
  if (imu.state != ImuHeadingState::Ready || imu.ageMs < 0 ||
      imu.ageMs > DRIVE_HEADING_MAX_IMU_AGE_MS ||
      !std::isfinite(imu.yawDeg) || !std::isfinite(imu.rateDegPerS)) {
    heading.mode = HeadingMode::Wheel;
    heading.correction = 0;
    return;
  }

  if (heading.mode != HeadingMode::Straight) {
    heading.mode = HeadingMode::Straight;
    heading.origin = imu.yawDeg;
    heading.target = 0;
  }
  heading.yaw = std::remainder(imu.yawDeg - heading.origin, 360.0f);
  heading.error = -heading.yaw;
  const float limit = std::max(0.0f, std::min(magnitude - DRIVE_HEADING_MIN_WHEEL_CPS,
                                            float(MAX_TARGET_CPS) - magnitude));
  const float correction = straightHeadingCorrection(magnitude, imu.rateDegPerS);
  heading.correction = std::clamp(correction, -limit, limit);
  goals[0] += heading.correction;
  goals[1] -= heading.correction;
}

/**
 * @description         : 在两轮各自减速的范围内分配直行航向修正
 * @param dt            : 实际控制周期，单位为秒
 * @param yawRate       : 偏航角速度，单位为度每秒
 * @param goals         : 输出两轮带方向的速度目标，单位为计数每秒
 * @return              : void
 */
void straightBrakingGoals(float dt, float yawRate, float (&goals)[2]) {
  const float direction = std::copysign(1.0f, float(wheels[0].target));
  const float right = std::fabs(wheels[0].ramp);
  const float left = std::fabs(wheels[1].ramp);
  const float step = float(rampCpsPerS) * dt;
  if (std::max(right, left) <= step) {
    goals[0] = goals[1] = heading.correction = 0;
    return;
  }

  const float rightDrop = std::min(right, step);
  const float leftDrop = std::min(left, step);
  // 双轮平均以设定斜率的 75% 减速，为差速修正留出余量。
  const float totalDrop = std::min(1.5f * step, rightDrop + leftDrop);
  const float common = 0.5f * (right + left - totalDrop);
  const float desiredRight = common + direction * straightHeadingCorrection(common, yawRate);
  // 右轮减速量的可行范围同时受左右轮各自的斜率限制。
  const float maxRightDrop = std::min(rightDrop, totalDrop);
  const float minRightDrop = std::clamp(totalDrop - leftDrop, 0.0f, maxRightDrop);
  const float appliedRightDrop = std::clamp(right - desiredRight, minRightDrop, maxRightDrop);
  goals[0] = direction * (right - appliedRightDrop);
  goals[1] = direction * std::max(0.0f, left - (totalDrop - appliedRightDrop));
  heading.correction = 0.5f * (goals[0] - goals[1]);
}

/**
 * @description         : 用航向误差和偏航角速度计算两轮目标
 * @param now           : MCU 时间，单位为毫秒
 * @param dt            : 实际控制周期，单位为秒
 * @param goals         : 输出 M1 右轮和 M2 左轮的目标，单位为计数每秒
 * @return              : 本周期还需更新轮速内环时返回 true
 */
bool headingGoals(uint32_t now, float dt, float (&goals)[2]) {
  const ImuHeadingSample imu = headingImuRead();
  if (imu.state != ImuHeadingState::Ready) {
    stopOutputs(ImuLost);
    return false;
  }
  heading.yaw = imu.yawDeg - heading.origin;
  heading.error = heading.target - heading.yaw;

  if (motionPhase == MotionPhase::Settle) {
    if (std::fabs(imu.rateDegPerS) <= QUIET_RATE_DPS) {
      if (!heading.quietAt) heading.quietAt = now;
    } else {
      heading.quietAt = 0;
    }
    const bool settled = heading.quietAt && now - heading.quietAt >= HEADING_QUIET_MS;
    if (settled || now - heading.settleAt >= HEADING_SETTLE_MS) {
      StopReason result = heading.finishReason;
      if (heading.mode == HeadingMode::Turn && result == Complete &&
          (!settled || std::fabs(heading.error) > YAW_TOLERANCE_DEG)) {
        result = HeadingError;
      }
      stopOutputs(result);
    }
    return false;
  }

  if (heading.mode == HeadingMode::Straight) {
    if (motionPhase == MotionPhase::RampDown) {
      straightBrakingGoals(dt, imu.rateDegPerS, goals);
      return true;
    }
    const float base = float(wheels[0].target);
    const float magnitude = std::fabs(base);
    heading.correction = straightHeadingCorrection(magnitude, imu.rateDegPerS);
    // 保留外环要求的轮速差，接近上限时降低两轮共同速度。
    const float commonMagnitude = std::min(magnitude,
        float(MAX_TARGET_CPS) - std::fabs(heading.correction));
    const float common = std::copysign(commonMagnitude, base);
    goals[0] = common + heading.correction;
    goals[1] = common - heading.correction;
    return true;
  }

  if (motionPhase == MotionPhase::RampDown) return true;
  const float direction = heading.target > 0 ? 1.0f : -1.0f;
  const float remaining = heading.error * direction;
  const float alongRate = std::max(0.0f, imu.rateDegPerS * direction);
  const float deceleration = float(rampCpsPerS) / YAW_CPS_PER_DPS;
  // 角制动距离加上 40 ms 响应时间内的预计转角。
  const float brakingDistance = alongRate * alongRate / (2.0f * deceleration) +
      alongRate * 0.04f;
  if (remaining <= std::max(YAW_TOLERANCE_DEG, brakingDistance)) {
    motionPhase = MotionPhase::RampDown;
  } else if (now - runStarted >= holdDuration) {
    heading.finishReason = HeadingTimeout;
    motionPhase = MotionPhase::RampDown;
  }
  if (motionPhase == MotionPhase::RampDown) return true;

  const float maxCps = std::fabs(float(wheels[0].target));
  const float controlledRate = heading.kp * remaining - heading.kd * alongRate;
  const float brakingRate = std::sqrt(2.0f * deceleration * remaining);
  const float cps = std::clamp(std::min(controlledRate, brakingRate) * YAW_CPS_PER_DPS,
                               0.0f, maxCps);
  heading.correction = direction * cps;
  goals[0] = heading.correction;
  goals[1] = -heading.correction;
  return true;
}

/**
 * @description         : 切换双轮前进标定的 PWM 档位
 * @param stage         : 目标档位，从 0 开始
 * @param progress      : 切换前一周期的两轮前进方向编码器增量
 * @return              : 两轮 PWM 均已成功写入时返回 true
 */
bool applyCalibrationStage(int stage, const int32_t (&progress)[2]) {
  for (int i = 0; i < 2; ++i) {
    const int duty = pwmCalibration.settings.duty[stage][i]; // 当前档位的前进 PWM 代码值。
    if (!writeWheelDuty(i, 1, duty, progress[i])) return false;
  }
  pwmCalibration.stage = stage;
  return true;
}

/**
 * @description         : 记录双轮固定 PWM 的一个采样区间，并在档位时间到达后切换
 * @param now           : MCU 时间，ms
 * @param dtMs          : 本采样区间长度，ms
 * @param deltas        : 本采样区间两轮编码器原始增量
 * @return              : void
 */
void updatePwmCalibration(uint32_t now, uint32_t dtMs, const int32_t (&deltas)[2]) {
  const uint32_t elapsed = now - runStarted; // 本采样区间末端距标定开始的时间，ms。
  const int32_t progress[2] = {int32_t(int64_t(deltas[0]) * wheels[0].sign),
                              int32_t(int64_t(deltas[1]) * wheels[1].sign)};
  if (pwmCalibration.count < PWM_CALIBRATION_ROWS) {
    PwmCalibrationSample &sample = pwmCalibration.rows[pwmCalibration.count++];
    sample.elapsedMs = uint16_t(elapsed);
    sample.dtMs = uint8_t(dtMs);
    sample.stage = uint8_t(pwmCalibration.stage);
    for (int i = 0; i < 2; ++i) {
      sample.delta[i] = progress[i];
      sample.duty[i] = uint8_t(wheels[i].pwm);
    }
  }
  for (int i = 0; i < 2; ++i) {
    if (!checkWheelProgress(i, 1, wheels[i].pwm, progress[i], now, true)) return;
  }
  const int nextStage = int(elapsed / pwmCalibration.settings.stageMs);
  if (nextStage > pwmCalibration.stage && nextStage < PWM_CALIBRATION_STAGES) {
    applyCalibrationStage(nextStage, progress);
  }
}

/**
 * @description         : 根据运行模式选择编码器采样和轮速控制周期
 * @return              : 当前模式设定的控制周期，ms
 */
uint32_t controlPeriodMs() {
  // 连续驾驶与定长试跑共用 10 ms 的增量式 PID 周期。
  return mode == Mode::Active && (runKind == PidRun || runKind == DriveRun)
      ? PID_PERIOD_MS : PERIOD_MS;
}

/**
 * @description         : 采样编码器并执行加速、保持、减速三个闭环阶段
 * @param now           : MCU 时间，单位为毫秒
 * @param nowTick       : 与 now 同一次读取的内核时钟刻度
 * @return              : void
 */
void controlTick(uint32_t now, int64_t nowTick) {
  const bool pidActive = controlPeriodMs() == PID_PERIOD_MS;
  const uint32_t elapsed = now - tickAt;
  if (pidActive ? nowTick < nextPidTick : elapsed < PERIOD_MS) return;
  tickAt = now;
  if (mode == Mode::Active && elapsed > MAX_CONTROL_GAP_MS) {
    stopOutputs(Timing);
    return;
  }

  float dt = elapsed * 0.001f; // 编码器采样间隔，s。
  if (pidActive) {
    dt = float(nowTick - pidSampleTick) / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    pidSampleTick = nowTick;
    // 迟到后以实际采样时刻为基准排下一拍：稳态 dt 恒为 P 或 P+1，负载抖动时
    // 只会更长，绝不会短于 P。
    //
    // 原来按绝对网格推进（lateTicks/P + 1）再看余量补半拍：迟到恰好 P/2 时
    // "余量 < P/2" 不成立，下一拍只隔 5 ms，编码器增量少一半、量化噪声翻倍；
    // 改成 "<= P/2 就再加一拍" 只是把短周期换成 15 ms 的长周期。
    // 改成"就近对齐网格"也不行：准点那一拍 lateTicks = 0，加 0 之后
    // nextPidTick 落在当前时刻，循环会退回每 1 ms 跑一次 PID。
    // 相对排期没有这两个边界，代价是周期在 10/11 ms 之间抖一个 tick。
    nextPidTick = pidSampleTick + PID_PERIOD_TICKS;
  }
  uint32_t counts[2], invalid[2];
  int32_t deltas[2];
  float measuredCps[2]; // 滑动窗口算出的轮速，计数/s，直接供 PID 使用。
  const uint32_t dtUs = uint32_t(std::min(65535.0f, dt * 1000000.0f)); // 本拍间隔，µs。
  readCounters(counts, invalid);
  for (int i = 0; i < 2; ++i) {
    Wheel &wheel = wheels[i];
    deltas[i] = int32_t(counts[i] - wheel.previousCount);
    wheel.previousCount = counts[i];
    if (deltas[i] != 0) lastEncoderMotionAt = now;

    // 起转补偿施加后，第一拍有沿目标方向的进展即释放旧积分。
    const int direction = wheel.target > 0 ? 1 : -1;
    const int32_t progress = int32_t(int64_t(deltas[i]) * wheel.sign * direction);
    if (runKind == DriveRun && wheel.target != 0) wheel.driveStart.observe(progress, now);
    if (wheel.kickApplied && progress > 0) {
      wheel.pid.reset();
      wheel.kickApplied = false;
    }

    // 把这拍压进滑动窗口，同时减掉最旧的一拍；速度用"窗口内总计数 / 总时间"。
    // 起转后头几拍窗口未填满，dtUsSum 只含已填的拍，求出来的仍是真实平均速度。
    const int slot = wheel.histIndex;
    wheel.deltaSum -= wheel.deltaHist[slot];
    wheel.dtUsSum -= wheel.dtUsHist[slot];
    const int32_t sample = std::clamp<int32_t>(deltas[i], -32768, 32767);
    wheel.deltaHist[slot] = int16_t(sample);
    wheel.dtUsHist[slot] = uint16_t(dtUs);
    wheel.deltaSum += sample;
    wheel.dtUsSum += dtUs;
    wheel.histIndex = (slot + 1) % SPEED_WINDOW_SAMPLES;
    if (wheel.histCount < SPEED_WINDOW_SAMPLES) ++wheel.histCount;

    measuredCps[i] = wheel.dtUsSum > 0
        ? float(wheel.deltaSum) * wheel.sign * 1000000.0f / float(wheel.dtUsSum) : 0.0f;
    wheel.speed += (dt / (dt + SPEED_FILTER_TAU_S)) * (measuredCps[i] - wheel.speed);
  }
  ++sequence;
  traceStart(now, elapsed, measuredCps);   // 本拍 trace 的静态字段，提交在 runControl 里做
  envSample(now);                          // 1 Hz 温度与零偏，与 10 ms 环路解耦
  if (mode != Mode::Active) return;
  if (runKind == PwmCalibrationRun) {
    updatePwmCalibration(now, elapsed, deltas);
    return;
  }
  if (runKind != PidRun && runKind != DriveRun) return;

  if (heading.mode != HeadingMode::Turn &&
      motionPhase == MotionPhase::Hold && now - holdStarted >= holdDuration) {
    motionPhase = MotionPhase::RampDown;
  }
  const bool hasHeading = runKind != DriveRun && heading.mode != HeadingMode::Wheel;
  float goals[2] = {float(wheels[0].target), float(wheels[1].target)};
  if (runKind == DriveRun) driveHeadingGoals(goals);
  else if (hasHeading && !headingGoals(now, dt, goals)) return;

  if (motionPhase == MotionPhase::RampDown && heading.mode != HeadingMode::Straight) {
    goals[0] = goals[1] = 0;
    heading.correction = 0;
  }
  const bool reached = advanceRamps(goals, dt);
  if (motionPhase == MotionPhase::RampUp &&
      (reached || (hasHeading && now - runStarted >= heading.rampUpMs))) {
    motionPhase = MotionPhase::Hold;
    holdStarted = now;
  } else if (motionPhase == MotionPhase::RampDown && reached && goals[0] == 0 && goals[1] == 0) {
    if (hasHeading) beginHeadingSettle(now);
    else stopOutputs(Complete);
    return;
  }

  // 不再在 PWM 层叠加差动修正。航向环已经在目标速度上给出差速，再叠一层 PWM
  // 等于两个环串起来抢同一个自由度，是左右摆动的主要来源。
  for (int i = 0; i < 2; ++i) {
    const int duty = pidWheelDuty(i, measuredCps[i], goals[i], 0, now);
    if (!applyPidWheel(i, deltas[i], duty, now)) return;
  }
}

/**
 * @description         : 运行电机保护检查及闭环控制线程
 * @return              : void
 */
void runControl(void *, void *, void *) {
  while (true) {
    k_mutex_lock(&stateMutex, K_FOREVER);
    const int64_t nowTick = k_uptime_ticks(); // 同一时刻用于保护和轮速控制。
    const uint32_t now = uint32_t(k_ticks_to_ms_floor64(nowTick));
    checkStop(now);
    controlTick(now, nowTick);
    // 控制拍内的所有早退路径都在这里统一提交，保证每拍恰好一条 trace。
    traceCommit();
    const bool pidActive = controlPeriodMs() == PID_PERIOD_MS;
    const int64_t nextControl = nextPidTick;
    k_mutex_unlock(&stateMutex);

    if (pidActive) {
      // 保护检查仍约每 2 ms 执行，遇到闭环控制时刻则按绝对时刻提前唤醒。
      const int64_t nextPoll = k_uptime_ticks() + CONTROL_POLL_TICKS;
      k_sleep(K_TIMEOUT_ABS_TICKS(std::min(nextPoll, nextControl)));
    } else {
      k_msleep(CONTROL_POLL_MS);
    }
  }
}

// 试跑启动和点动在持有状态锁时执行唤醒及状态切换。
/**
 * @description         : 判断驱动就绪且轮子已停稳
 * @return              : 可开始试跑时返回 true
 */
bool canStart() {
  const uint32_t now = millis();
  return mode == Mode::Idle && pwmReady && workerReady &&
    now - stoppedAt >= SETTLE_MS && now - lastEncoderMotionAt >= SETTLE_MS;
}

/**
 * @description         : 唤醒驱动并检查故障脚
 * @return              : 驱动正常时返回 true
 */
bool wakeDriver() {
  // motorPrepareBrake() 的第一句是 "驱动已经醒着就返回 false"。如果上一次停车
  // 没把 nSLEEP 拉低（异常路径、复位时序），这里就会一直失败，而原来直接
  // 把 pwmReady 置假 —— 之后 canStart() 永久拒绝起转，车表现为"命令发下去了
  // 但目标始终是 0"，只能靠手动复位才恢复。先强制睡一次再重试，避免自锁。
  if (!motorPrepareBrake()) {
    motorSetAwake(false);
    k_msleep(2);
  }
  if (!motorPrepareBrake()) {
    pwmReady = false;
    stopOutputs(PwmError);
    return false;
  }
  motorSetAwake(true);
  k_msleep(5);
  if (motorIsAwake() && !motorHasFault()) return true;
  stopOutputs(DriverFault);
  return false;
}

/**
 * @description         : 开启一次具有时限和心跳保护的试跑
 * @param id            : 试跑编号
 * @param durationMs    : 运行时限，单位为毫秒
 * @param kind          : 点动、闭环或固定 PWM 标定
 * @param limit         : PWM 代码值上限
 * @return              : void
 */
void beginRun(int id, int durationMs, RunKind kind, int limit) {
  // 统一在这里夹一刀：闭环、点动、标定都走这个入口，任何一条路径都不许把
  // 占空比写到 MAX_APPLIED_PWM 之上（原因见 MAX_APPLIED_PWM 的注释）。
  pwmLimit = std::min(limit, MAX_APPLIED_PWM);
  runId = id;
  runDuration = durationMs;
  pidSampleTick = k_uptime_ticks();
  runStarted = heartbeatAt = tickAt = uint32_t(k_ticks_to_ms_floor64(pidSampleTick));
  nextPidTick = pidSampleTick + PID_PERIOD_TICKS;
  runKind = kind;
  // 闭环试跑与连续驾驶都从加速段开始，点动和标定保持空闲阶段。
  motionPhase = (kind == PidRun || kind == DriveRun) ? MotionPhase::RampUp : MotionPhase::Idle;
  mode = Mode::Active;
  if (kind == DeadzoneRun) reason = DeadzoneRunning;
  else if (kind == PwmCalibrationRun) reason = PwmCalibrationRunning;
  else reason = Running;
  ++sequence;
}

/**
 * @description         : 清零试跑的双轮控制状态
 * @param counts        : 两轮的初始原始计数
 * @return              : void
 */
void resetRunWheels(const uint32_t (&counts)[2]) {
  for (int i = 0; i < 2; ++i) {
    wheels[i] = {};
    wheels[i].sign = defaultEncoderSigns[i];
    wheels[i].previousCount = counts[i];
  }
}

} // namespace

/**
 * @description         : 初始化控制线程和 Bridge API 共用的状态锁
 * @return              : void
 */
void init() {
  k_mutex_init(&stateMutex);
}

/**
 * @description         : 初始化 PWM 和编码器并启动轮速控制线程
 * @return              : void
 */
void startWorker() {
  pwmReady = initPwm();
  encoderBegin();
  for (int i = 0; i < 2; ++i) wheels[i].sign = defaultEncoderSigns[i];
  controlStack = k_thread_stack_alloc(CONTROL_STACK_BYTES, 0);
  workerReady = controlStack != nullptr;
  stopOutputs(Boot);
  tickAt = millis();
  if (workerReady) k_thread_create(&controlThread, controlStack, CONTROL_STACK_BYTES,
      runControl, nullptr, nullptr, nullptr, CONTROL_PRIORITY, 0, K_NO_WAIT);
}

/**
 * @description         : 应用闭环参数并启动轮速或航向控制
 * @param settings      : 已经校验的试跑参数
 * @return              : 试跑启动时返回 true
 */
bool start(const MotionSettings &settings, RunKind kind) {
  HeadingControl requestedHeading; // 待启动的航向配置和起点。
  requestedHeading.mode = settings.heading.mode;
  requestedHeading.target = requestedHeading.error = settings.heading.target;
  requestedHeading.kp = settings.heading.kp;
  requestedHeading.kd = settings.heading.kd;
  requestedHeading.correctionLimit = settings.heading.correctionLimit;

  k_mutex_lock(&stateMutex, K_FOREVER);
  if (!canStart()) {
    k_mutex_unlock(&stateMutex);
    return false;
  }
  if (requestedHeading.mode != HeadingMode::Wheel) {
    const ImuHeadingSample imu = headingImuRead();
    if (imu.state != ImuHeadingState::Ready || std::fabs(imu.rateDegPerS) > 3.0f) {
      k_mutex_unlock(&stateMutex);
      return false;
    }
    requestedHeading.origin = imu.yawDeg;
  }

  uint32_t counts[2], invalid[2];
  readCounters(counts, invalid);
  resetRunWheels(counts);
  for (int i = 0; i < 2; ++i) {
    const WheelSettings &config = settings.wheels[i];
    Wheel &wheel = wheels[i];
    wheel.target = config.target;
    for (int gain = 0; gain < 3; ++gain) wheel.gainCodes[gain] = config.gainCodes[gain];
    wheel.pid.kp = wheel.gainCodes[0] / 10000.0f;
    wheel.pid.ki = wheel.gainCodes[1] / 10000.0f;
    wheel.pid.kd = wheel.gainCodes[2] / 10000.0f;
    wheel.sign = config.sign;
    wheel.deadzonePwm = config.deadzonePwm;
    wheel.ffDuty = config.feedforwardPwm;
    wheel.kickPending = kind != DriveRun && config.target != 0;
    wheel.driveStart.reset(kind == DriveRun && config.target != 0);
  }
  const bool ready = wakeDriver();
  driverFaultLow = false;
  if (ready) {
    rampCpsPerS = settings.rampCpsPerS;
    holdDuration = settings.holdMs;
    const int maxTarget = std::max(std::abs(wheels[0].target), std::abs(wheels[1].target));
    const int rampMs = (maxTarget * 1000 + rampCpsPerS - 1) / rampCpsPerS;
    requestedHeading.rampUpMs = rampMs;
    heading = requestedHeading;
    driveHoldHeading = kind == DriveRun && settings.driveHoldHeading;
    // 直行减速保向可延长减速过程，预留额外的运行时限。
    const int brakingBudgetMs = heading.mode == HeadingMode::Straight ? 2 * rampMs : rampMs;
    beginRun(settings.id, holdDuration + rampMs + brakingBudgetMs + 1000, kind, settings.pwmLimit);
  }
  k_mutex_unlock(&stateMutex);
  return ready;
}

/**
 * @description         : 按指定 PWM 和时长点动一只轮子
 * @param wheel         : 1 为 M1 右轮，2 为 M2 左轮
 * @param pwm           : 带方向的 PWM 代码值
 * @param durationMs    : 点动时长，单位为毫秒
 * @param id            : 点动编号
 * @return              : 点动开始时返回 true
 */
bool jog(int wheel, int pwm, int durationMs, int id) {
  k_mutex_lock(&stateMutex, K_FOREVER);
  if (!canStart() || !wakeDriver()) {
    k_mutex_unlock(&stateMutex);
    return false;
  }

  uint32_t counts[2], invalid[2];
  readCounters(counts, invalid);
  resetRunWheels(counts);
  const int index = wheel - 1;
  deadzone = {};
  deadzone.status.id = id;
  deadzone.status.wheel = wheel;
  deadzone.status.pwm = pwm;
  deadzone.startCount = counts[index];
  deadzone.startInvalid = invalid[index];
  const int duty = pwm > 0 ? pwm : -pwm;
  heading = {};
  beginRun(id, durationMs, DeadzoneRun, duty);
  const bool applied = applyDuty(driveInput(index, pwm), duty);
  if (applied) {
    wheels[index].pwm = pwm;
  } else {
    pwmReady = false;
    stopOutputs(PwmError);
  }
  k_mutex_unlock(&stateMutex);
  return applied;
}

bool startPwmCalibration(const PwmCalibrationSettings &settings) {
  k_mutex_lock(&stateMutex, K_FOREVER);
  if (!canStart() || !wakeDriver()) {
    k_mutex_unlock(&stateMutex);
    return false;
  }

  uint32_t counts[2], invalid[2];
  readCounters(counts, invalid);
  resetRunWheels(counts);
  heading = {};
  holdDuration = 0;
  pwmCalibration.settings = settings;
  pwmCalibration.count = 0;
  pwmCalibration.stopReason = -1;
  int maxDuty = 0; // 本次标定实际使用的最大 PWM 代码值。
  for (const auto &stage : settings.duty) {
    for (uint8_t duty : stage) maxDuty = std::max(maxDuty, int(duty));
  }
  beginRun(settings.id, PWM_CALIBRATION_STAGES * settings.stageMs, PwmCalibrationRun, maxDuty);
  const int32_t initialProgress[2] = {}; // 启动前尚无本次试跑的编码器增量。
  const bool applied = applyCalibrationStage(0, initialProgress);
  k_mutex_unlock(&stateMutex);
  return applied;
}

PwmCalibrationPage readPwmCalibration(int id, int offset) {
  PwmCalibrationPage page;
  k_mutex_lock(&stateMutex, K_FOREVER);
  if (id > 0 && id == pwmCalibration.settings.id && offset >= 0 && offset <= pwmCalibration.count) {
    page.id = id;
    page.total = pwmCalibration.count;
    page.stopReason = pwmCalibration.stopReason;
    page.count = std::min(PWM_CALIBRATION_PAGE_ROWS, page.total - offset);
    for (int i = 0; i < page.count; ++i) page.rows[i] = pwmCalibration.rows[offset + i];
  }
  k_mutex_unlock(&stateMutex);
  return page;
}

/**
 * @description         : 启动遥控连续驾驶，不设固定时限，由心跳和故障保护结束
 * @param settings      : 双轮、航向和限幅配置，holdMs 字段被忽略
 * @return              : 驾驶启动时返回 true
 */
bool drive(const MotionSettings &settings) {
  MotionSettings endless = settings;   // 改写为无固定时限的配置副本。
  endless.holdMs = DRIVE_HOLD_MS;
  return start(endless, DriveRun);
}

/**
 * @description         : 在线更新连续驾驶的双轮目标速度并刷新通信心跳
 * @param rightCps      : M1 右轮目标，计数/s，正值前进
 * @param leftCps       : M2 左轮目标，计数/s，正值前进
 * @param holdHeading   : 直行时请求 MCU 本地保向
 * @return              : 当前正处于连续驾驶状态时返回 true
 */
bool driveSet(int32_t rightCps, int32_t leftCps, bool holdHeading) {
  k_mutex_lock(&stateMutex, K_FOREVER);
  const uint32_t now = millis();
  checkStop(now);
  const bool active = mode == Mode::Active && runKind == DriveRun;
  if (active) {
    const int32_t requested[2] = {rightCps, leftCps}; // 按 M1 右轮、M2 左轮顺序排列的目标。
    driveHoldHeading = holdHeading;
    // 在线调参：把当前运行期参数同步到这一拍，网页改完不用停车重起步就能生效。
    // 斜坡、增益、保向系数都只是本拍限幅或增益，运行中改不会造成跳变。
    const int32_t kp = tuneValue(TuneKp);
    const int32_t ki = tuneValue(TuneKi);
    const int32_t kd = tuneValue(TuneKd);
    rampCpsPerS = int(tuneValue(TuneRamp));
    heading.kp = float(tuneValue(TuneHeadingKp)) / 1000.0f;
    heading.kd = float(tuneValue(TuneHeadingKd)) / 1000.0f;
    heading.correctionLimit = int(tuneValue(TuneHeadingLimit));
    for (int i = 0; i < 2; ++i) {
      wheels[i].pid.kp = kp / 10000.0f;
      wheels[i].pid.ki = ki / 10000.0f;
      wheels[i].pid.kd = kd / 10000.0f;
    }
    if (!holdHeading) {
      heading.mode = HeadingMode::Wheel;
      heading.correction = 0;
    }
    for (int i = 0; i < 2; ++i) {
      const int32_t target = std::clamp(requested[i], -MAX_TARGET_CPS, MAX_TARGET_CPS);
      const int32_t before = wheels[i].target;
      const bool reversed = (before > 0 && target < 0) || (before < 0 && target > 0);
      // 目标换向：清积分、把斜坡打回 0、重置棘轮基准。
      // 斜坡不清的话，换向后第一拍仍用旧方向的幅值 fabs(ramp) 当新方向目标，
      // 会给出一次反向满占空比冲击；棘轮不清则把换向时的惯性前滚记成"反向位移"。
      if (reversed) {
        heading.mode = HeadingMode::Wheel;
        wheels[i].pid.reset();
        wheels[i].ramp = 0;
        clearSpeedWindow(wheels[i]);   // 换向后窗口里的反向计数不能算进新方向
        wheels[i].position = wheels[i].furthestPosition = 0;
        wheels[i].wrongSince = 0;
      }
      wheels[i].target = target;
      if (target == 0 || before == 0 || reversed) {
        wheels[i].driveStart.reset(target != 0);
        wheels[i].progressGuard.reset();
      }
    }
    heartbeatAt = now;
  }
  k_mutex_unlock(&stateMutex);
  return active;
}

/**
 * @description         : 刷新正在运行且编号匹配的试跑心跳
 * @param id            : 当前试跑编号
 * @return              : 心跳接受时返回 true
 */
bool heartbeat(int id) {
  k_mutex_lock(&stateMutex, K_FOREVER);
  const uint32_t now = millis();
  checkStop(now);
  const bool accepted = mode == Mode::Active && id == runId;
  if (accepted) heartbeatAt = now;
  k_mutex_unlock(&stateMutex);
  return accepted;
}

/**
 * @description         : 立即停止驱动或确认已停机的故障状态
 * @return              : 驱动已休眠时返回 true
 */
bool stop() {
  k_mutex_lock(&stateMutex, K_FOREVER);
  if (mode == Mode::Active || motorIsAwake()) {
    stopOutputs(mode == Mode::Fault ? reason : Manual);
  }
  const bool stopped = !motorIsAwake();
  k_mutex_unlock(&stateMutex);
  return stopped;
}

bool resetDrive() {
  k_mutex_lock(&stateMutex, K_FOREVER);
  const uint32_t now = millis();
  bool reset = false;
  if (mode != Mode::Active && !motorIsAwake() && workerReady &&
      now - stoppedAt >= SETTLE_MS && now - lastEncoderMotionAt >= SETTLE_MS &&
      !motorHasFault()) {
    if (!pwmReady) {
      motorDisableOutputs();
      pwmReady = initPwm();
    }
    stopOutputs(pwmReady ? Manual : PwmError);
    reset = pwmReady && mode == Mode::Idle && !motorIsAwake();
  }
  k_mutex_unlock(&stateMutex);
  return reset;
}

/**
 * @description         : 在状态锁内获取运行状态、双轮计数和航向遥测
 * @return              : 一次控制状态的快照
 */
MotionStatus status() {
  k_mutex_lock(&stateMutex, K_FOREVER);
  MotionStatus snapshot;
  const uint32_t now = millis();
  const uint32_t elapsed = now - runStarted;
  const bool active = mode == Mode::Active;
  uint32_t remaining = active && elapsed < runDuration ? runDuration - elapsed : 0;
  if (active && motionPhase == MotionPhase::Hold) {
    const uint32_t holdElapsed = now - (heading.mode == HeadingMode::Turn ? runStarted : holdStarted);
    remaining = holdElapsed < holdDuration ? holdDuration - holdElapsed : 0;
  }

  uint32_t counts[2], invalid[2];
  readCounters(counts, invalid);
  snapshot.mode = mode;
  snapshot.reason = reason;
  snapshot.awake = motorIsAwake();
  snapshot.faultPinHigh = !motorHasFault();
  snapshot.remainingMs = remaining;
  snapshot.sequence = sequence;
  snapshot.uptimeMs = now;
  snapshot.controlPeriodMs = controlPeriodMs();
  snapshot.ready = pwmReady && workerReady;
  snapshot.runId = active ? runId : 0;
  snapshot.pwmLimit = pwmLimit;
  for (int i = 0; i < 2; ++i) {
    const Wheel &wheel = wheels[i];
    WheelStatus &value = snapshot.wheels[i];
    value.target = wheel.target;
    value.ramp = wheel.ramp;
    value.speed = wheel.speed;
    value.pwm = wheel.pwm;
    value.count = counts[i];
    value.invalid = invalid[i];
    for (int gain = 0; gain < 3; ++gain) value.gainCodes[gain] = wheel.gainCodes[gain];
    value.sign = wheel.sign;
  }
  snapshot.runKind = runKind;
  snapshot.deadzone = deadzone.status;
  if (active && runKind == DeadzoneRun) sampleDeadzone(snapshot.deadzone, now, counts, invalid);
  snapshot.phase = motionPhase;
  snapshot.rampCpsPerS = rampCpsPerS;
  snapshot.holdMs = holdDuration;
  snapshot.fault = fault;
  snapshot.imu = headingImuRead();
  snapshot.heading = {heading.mode, heading.target, heading.yaw, heading.error,
                      heading.correction, heading.correctionLimit};
  k_mutex_unlock(&stateMutex);
  return snapshot;
}

/**
 * @description         : 在驱动休眠且车轮停稳时重新校准 IMU
 * @return              : 校准请求已提交时返回 true
 */
bool calibrateHeading() {
  k_mutex_lock(&stateMutex, K_FOREVER);
  const bool accepted = canStart() && headingImuCalibrate();
  k_mutex_unlock(&stateMutex);
  return accepted;
}

TracePage readTrace(int offset) {
  TracePage page;
  k_mutex_lock(&stateMutex, K_FOREVER);
  const int total = int(traceTotal);
  const int stored = std::min(total, TRACE_SAMPLE_MAX);
  page.epoch = traceEpoch;
  page.total = total;
  page.dropped = total - stored;      // 被环形覆盖、已经取不回来的样本数
  page.capacity = TRACE_SAMPLE_MAX;
  page.bytesPerSample = int(sizeof(TraceSample));
  // 实测周期：闭环运行时是 10 ms，空闲时控制拍按 PERIOD_MS(20 ms) 调度，
  // 因此空转段 trace 是 50 Hz。这里如实上报，不写死 PID_PERIOD_MS。
  page.periodMs = int(controlPeriodMs());
  if (offset >= 0 && offset < stored) {
    page.count = std::min(TRACE_PAGE_ROWS, stored - offset);
    // 最旧样本的位置：写指针回退 stored 格。
    const int oldest = int((uint32_t(traceHead) + uint32_t(TRACE_SAMPLE_MAX) -
                            uint32_t(stored)) % uint32_t(TRACE_SAMPLE_MAX));
    for (int i = 0; i < page.count; ++i) {
      page.rows[i] = traceRing[(oldest + offset + i) % TRACE_SAMPLE_MAX];
    }
  }
  k_mutex_unlock(&stateMutex);
  return page;
}

EnvPage readEnv(int offset) {
  EnvPage page;
  k_mutex_lock(&stateMutex, K_FOREVER);
  const int total = int(envTotal);
  const int stored = std::min(total, ENV_SAMPLE_MAX);
  page.epoch = traceEpoch;
  page.total = total;
  page.dropped = total - stored;
  if (offset >= 0 && offset < stored) {
    page.count = std::min(ENV_PAGE_ROWS, stored - offset);
    const int oldest = int((uint32_t(envHead) + uint32_t(ENV_SAMPLE_MAX) -
                            uint32_t(stored)) % uint32_t(ENV_SAMPLE_MAX));
    for (int i = 0; i < page.count; ++i) {
      page.rows[i] = envRing[(oldest + offset + i) % ENV_SAMPLE_MAX];
    }
  }
  k_mutex_unlock(&stateMutex);
  return page;
}

void resetTrace() {
  k_mutex_lock(&stateMutex, K_FOREVER);
  traceHead = 0;
  traceTotal = 0;
  pendingTraceValid = false;
  traceHold = false;
  envHead = 0;
  envTotal = 0;
  envLastAt = 0;
  ++traceEpoch;
  k_mutex_unlock(&stateMutex);
}

void setTraceHold(bool hold) {
  k_mutex_lock(&stateMutex, K_FOREVER);
  traceHold = hold;
  if (hold) pendingTraceValid = false;   // 冻结那一拍的半成品不要写进去
  k_mutex_unlock(&stateMutex);
}

int32_t stackHeadroom() {
  // 本板 arduino:zephyr 1.0.0 打开了 CONFIG_THREAD_STACK_INFO 头文件宏，
  // 但没有链接 z_impl_k_thread_stack_space_get，调用会直接链接失败。
  // 因此这里不做猜测：返回 -1，由链接期 RAM 报告 + 现场压测来判定余量。
  return -1;
}

/**
 * @description         : 无锁读取全部运行期参数
 * @param out           : 长度至少 TuneParamCount 的输出数组
 * @return              : void
 */
static void readTuneUnlocked(int32_t out[TuneParamCount]) {
  // 前馈表四象限目前是同一组值，取 [0][0] 回读即可；写入时四份一起改。
  const WheelFeedforward &ff = driveFeedforward[0][0];
  out[TuneKp] = driveKpX10000;
  out[TuneKi] = driveKiX10000;
  out[TuneKd] = driveKdX10000;
  out[TuneRamp] = driveRampCpsPerS;
  out[TuneFfOffset] = int32_t(std::lround(ff.runningOffset * 10.0f));
  out[TuneFfSlope] = int32_t(std::lround(ff.cpsPerDuty * 100.0f));
  out[TuneFfSafety] = int32_t(std::lround(driveFfSafety * 1000.0f));
  out[TuneBreakawayDuty] = int32_t(ff.breakawayDuty);
  out[TuneBreakawayBoost] = driveBreakawayMaxBoost;
  out[TuneHeadingKp] = driveHeadingKpX1000;
  out[TuneHeadingKd] = driveHeadingKdX1000;
  out[TuneHeadingLimit] = driveHeadingLimitCps;
}

void tuneRead(int32_t out[TuneParamCount]) {
  if (out == nullptr) return;
  k_mutex_lock(&stateMutex, K_FOREVER);
  readTuneUnlocked(out);
  k_mutex_unlock(&stateMutex);
}

int32_t tuneValue(int index) {
  // driveSettings() 逐项取增益用。这里不取锁：调用方紧接着的 drive() 会持锁，
  // 而 tuneWrite 同样持锁，两者不会交错，因此读到的不会是新旧混合的一组。
  int32_t values[TuneParamCount];
  readTuneUnlocked(values);
  return index >= 0 && index < TuneParamCount ? values[index] : 0;
}

bool tuneWrite(int index, int32_t value) {
  k_mutex_lock(&stateMutex, K_FOREVER);
  bool accepted = true;
  switch (index) {
    case TuneKp:
      if (value < 0 || value > 100000) { accepted = false; break; }
      driveKpX10000 = value;
      break;
    case TuneKi:
      if (value < 0 || value > 100000) { accepted = false; break; }
      driveKiX10000 = value;
      break;
    case TuneKd:
      if (value < 0 || value > 100000) { accepted = false; break; }
      driveKdX10000 = value;
      break;
    case TuneRamp:
      // 上限就是控制器允许的最大斜坡，超过它 driveSet 内部也会夹取。
      if (value < MIN_RAMP_CPS_PER_S || value > MAX_RAMP_CPS_PER_S) { accepted = false; break; }
      driveRampCpsPerS = value;
      break;
    case TuneFfOffset:
      if (value < 0 || value > 2000) { accepted = false; break; }
      for (auto &pair : driveFeedforward)
        for (auto &ff : pair) ff.runningOffset = float(value) / 10.0f;
      break;
    case TuneFfSlope:
      // 斜率为 0 会让前馈除零，下限留 1.0 计数/秒每码。
      if (value < 100 || value > 10000) { accepted = false; break; }
      for (auto &pair : driveFeedforward)
        for (auto &ff : pair) ff.cpsPerDuty = float(value) / 100.0f;
      break;
    case TuneFfSafety:
      if (value < 0 || value > 1500) { accepted = false; break; }
      driveFfSafety = float(value) / 1000.0f;
      break;
    case TuneBreakawayDuty:
      if (value < 0 || value > MAX_PID_PWM) { accepted = false; break; }
      for (auto &pair : driveFeedforward)
        for (auto &ff : pair) ff.breakawayDuty = int(value);
      break;
    case TuneBreakawayBoost:
      if (value < 0 || value > MAX_PID_PWM) { accepted = false; break; }
      driveBreakawayMaxBoost = int(value);
      break;
    case TuneHeadingKp:
      // 上限 20 对应约 870 cps/°，远超修正限幅，留足实验空间。
      if (value < 0 || value > 20000) { accepted = false; break; }
      driveHeadingKpX1000 = value;
      break;
    case TuneHeadingKd:
      if (value < 0 || value > 10000) { accepted = false; break; }
      driveHeadingKdX1000 = value;
      break;
    case TuneHeadingLimit:
      if (value < 0 || value > MAX_TARGET_CPS) { accepted = false; break; }
      driveHeadingLimitCps = value;
      break;
    default:
      accepted = false;
      break;
  }
  k_mutex_unlock(&stateMutex);
  return accepted;
}

void tuneReset() {
  k_mutex_lock(&stateMutex, K_FOREVER);
  driveKpX10000 = 20000;
  driveKiX10000 = 5000;
  driveKdX10000 = 0;
  driveRampCpsPerS = 6000;
  driveFfSafety = 0.95f;
  driveBreakawayMaxBoost = 150;   // 与全局默认值保持一致（见文件头说明）
  driveHeadingKpX1000 = 1000;
  driveHeadingKdX1000 = 500;
  driveHeadingLimitCps = 220;
  for (auto &pair : driveFeedforward)
    for (auto &ff : pair) ff = DEFAULT_FEEDFORWARD;
  k_mutex_unlock(&stateMutex);
}

} // namespace motion
