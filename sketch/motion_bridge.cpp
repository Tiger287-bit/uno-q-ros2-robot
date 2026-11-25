#include "motion_bridge.h"

#include <Arduino_RouterBridge.h>
#include <cmath>
#include <cstddef>
#include <vector>

#include "lidar_bridge.h"
#include "motion_controller.h"
#include "motor_driver.h"
#include "vm_monitor.h"

namespace {

/**
 * @description         : 本 App 的底盘固定参数说明
 * @note                : 轮速环增益按 2026-11-20 空载实测重定。
 *                        换算关系：增量式 PID 的 u = kp·e + ki·Σe，落到代码值是
 *                        乘以 UNO_PWM_PER_YAHBOOM_UNIT=0.0708，所以 kp=2 相当于
 *                        每 1 mm/s 误差给 0.14 码、ki=0.5 相当于每拍 0.035 码。
 *                        旧值 0.8/0.2 的积分时间常数约 1.35 s，带载时靠它补前馈
 *                        误差要 1.5 s 以上，实测 3 s 内都收不干净。
 *                        增益、斜坡、前馈与起转参数现在都是运行期可写的，
 *                        集中在 motion_controller.cpp 的 driveKpX10000 等变量，
 *                        默认值就是这里原来那组标定值。网页调参经
 *                        motion::tuneWrite 改写，本文件用 tuneValue 读取当前值。
 */
constexpr int DRIVE_PWM_LIMIT = motion::MAX_PID_PWM; // 单轮 PWM 上限，全占空比。
constexpr int32_t ENCODER_SIGN_M1 = -1;    // M1 右轮编码器换向符号。
constexpr int32_t ENCODER_SIGN_M2 = 1;     // M2 左轮编码器换向符号。

// ros_status 固定前缀的字段位置，与 python/control.py 一致。
enum StatusField : std::size_t {
  STATUS_MODE = 0,          // 控制器状态：0 空闲，1 运行，2 故障。
  STATUS_REASON,            // 停止原因代码。
  STATUS_AWAKE,             // 驱动 nSLEEP 逻辑电平。
  STATUS_FAULT_PIN,         // 驱动 nFAULT 逻辑电平，高为正常。
  STATUS_READY,             // PWM 与控制线程是否就绪。
  STATUS_UPTIME,            // MCU 启动后时间，ms。
  STATUS_SEQUENCE,          // 状态更新序号。
  STATUS_PERIOD,            // 当前控制周期，ms。
  STATUS_RUN_ID,            // 运行编号，空闲时为 0。
  STATUS_PHASE,             // 运动阶段：0 空闲，1 加速，2 保速，3 减速，4 停稳。
  STATUS_WHEEL_BASE,        // 双轮遥测起始位置，每轮 6 个字段。
  WHEEL_FIELDS = 6,         // 每轮字段数。
  STATUS_IMU_BASE = STATUS_WHEEL_BASE + 2 * WHEEL_FIELDS, // IMU 字段起始位置。
  STATUS_FIELDS = STATUS_IMU_BASE + 16                    // 固定前缀长度。
};
constexpr int32_t FAULT_STATUS_VERSION = 1;
constexpr std::size_t FAULT_STATUS_FIELDS = 22;

// 单轮遥测在数组中的相对位置。
enum WheelField : std::size_t {
  WHEEL_TARGET = 0, // 带方向的目标速度，计数/s。
  WHEEL_RAMP,       // 带方向的斜坡参考，放大 10 倍。
  WHEEL_SPEED,      // 带方向的实测速度，放大 10 倍。
  WHEEL_PWM,        // 当前 PWM 代码值。
  WHEEL_COUNT,      // 原始编码器累计计数。
  WHEEL_INVALID     // 无效跳变累计次数。
};

/**
 * @description         : 按底盘固定参数构造连续驾驶配置
 * @param rightCps      : M1 右轮目标，计数/s
 * @param leftCps       : M2 左轮目标，计数/s
 * @return              : 可直接交给运动控制器的配置
 */
motion::MotionSettings driveSettings(int32_t rightCps, int32_t leftCps, bool holdHeading) {
  motion::MotionSettings settings; // 待填充的控制器输入。
  settings.id = 1;                 // 连续驾驶固定编号，便于状态判断。
  settings.pwmLimit = DRIVE_PWM_LIMIT;
  settings.rampCpsPerS = int(motion::tuneValue(motion::TuneRamp));
  settings.driveHoldHeading = holdHeading;
  // 直行保向增益走运行期参数，默认 1.85 对应原 80 count/s/° 的比例修正量级。
  settings.heading.kp = float(motion::tuneValue(motion::TuneHeadingKp)) / 1000.0f;
  settings.heading.kd = float(motion::tuneValue(motion::TuneHeadingKd)) / 1000.0f;
  settings.heading.correctionLimit = int(motion::tuneValue(motion::TuneHeadingLimit));

  const int32_t targets[2] = {rightCps, leftCps}; // 按 M1 右轮、M2 左轮顺序排列。
  const int32_t signs[2] = {ENCODER_SIGN_M1, ENCODER_SIGN_M2};
  for (int i = 0; i < 2; ++i) {
    auto &wheel = settings.wheels[i];
    wheel.target = targets[i];
    wheel.gainCodes[0] = motion::tuneValue(motion::TuneKp);
    wheel.gainCodes[1] = motion::tuneValue(motion::TuneKi);
    wheel.gainCodes[2] = motion::tuneValue(motion::TuneKd);
    wheel.sign = int(signs[i]);
    wheel.deadzonePwm = 0; // 遥控模式由速度前馈承担死区补偿，避免两者叠加造成过冲。
  }
  return settings;
}

/**
 * @description         : 启动或更新连续驾驶，速度为 0 时立即停车
 * @param rightCps      : M1 右轮目标，计数/s，正值前进
 * @param leftCps       : M2 左轮目标，计数/s，正值前进
 * @param holdHeading   : 直行时请求 MCU 本地保向
 * @return              : 指令被执行时返回 true
 */
bool rosDriveHeading(int rightCps, int leftCps, int holdHeading) {
  if (holdHeading < 0 || holdHeading > 1) return false;
  if (rightCps == 0 && leftCps == 0) return motion::stop();
  if (rightCps < -motion::MAX_TARGET_CPS || rightCps > motion::MAX_TARGET_CPS ||
      leftCps < -motion::MAX_TARGET_CPS || leftCps > motion::MAX_TARGET_CPS) {
    return false;
  }

  if (motion::driveSet(rightCps, leftCps, holdHeading != 0)) return true;
  return motion::drive(driveSettings(rightCps, leftCps, holdHeading != 0));
}

bool rosDrive(int rightCps, int leftCps) {
  return rosDriveHeading(rightCps, leftCps, 0);
}

/**
 * @description         : 显式停车并休眠驱动
 * @return              : 驱动已休眠时返回 true
 */
bool rosStop() {
  return motion::stop();
}

bool rosResetDrive() {
  return motion::resetDrive();
}

/**
 * @description         : 重新校准陀螺仪零偏，仅在停车且静止时生效
 * @return              : 校准请求被接受时返回 true
 */
bool rosCalibrateHeading() {
  return motion::calibrateHeading();
}

/**
 * @description         : 将浮点遥测按协议比例量化为整数
 * @param value         : 原始物理量
 * @param scale         : 每单位物理量对应的整数刻度数
 * @return              : 四舍五入后的整数代码值
 */
int32_t encodeScaled(float value, int scale) {
  return int32_t(std::lround(value * scale));
}

/**
 * @description         : 组装给 Linux 侧的紧凑遥测数组
 * @return              : 38 项基本状态，出现故障后追加版本化的故障快照
 */
std::vector<int32_t> rosStatus() {
  const motion::MotionStatus state = motion::status();
  std::vector<int32_t> result;
  const auto &fault = state.fault;
  result.reserve(STATUS_FIELDS + (fault.reason >= 0 ? FAULT_STATUS_FIELDS : 0));

  // 0–9 为运行和驱动状态。
  result.insert(result.end(), {int32_t(state.mode), int32_t(state.reason), int32_t(state.awake),
      int32_t(state.faultPinHigh), int32_t(state.ready), int32_t(state.uptimeMs),
      int32_t(state.sequence), int32_t(state.controlPeriodMs), int32_t(state.runId),
      int32_t(state.phase)});

  // 10–21 为双轮遥测，每轮 6 项。
  for (const auto &wheel : state.wheels) {
    result.insert(result.end(), {wheel.target, encodeScaled(wheel.ramp, 10),
        encodeScaled(wheel.speed, 10), int32_t(wheel.pwm), int32_t(wheel.count),
        int32_t(wheel.invalid)});
  }

  // 22–27 为 IMU 状态，角度和角速度放大 1000 倍。
  const auto &imu = state.imu;
  result.insert(result.end(), {int32_t(imu.state), imu.ageMs, encodeScaled(imu.rateDegPerS, 1000),
      encodeScaled(imu.yawDeg, 1000), encodeScaled(imu.biasDegPerS, 1000), int32_t(imu.samples)});

  // 28–32 为航向外环状态，角度放大 1000 倍。
  const auto &heading = state.heading;
  result.insert(result.end(), {int32_t(heading.mode), encodeScaled(heading.target, 1000),
      encodeScaled(heading.yaw, 1000), encodeScaled(heading.error, 1000),
      encodeScaled(heading.correction, 1)});

  // 33–37 为 A4 电池分压采样：序号、最新值、100 ms 最小/最大值与样本年龄。
  const VmMonitorSample vm = vmMonitorRead();
  result.insert(result.end(), {int32_t(vm.sequence), vm.raw, vm.minRaw, vm.maxRaw, vm.ageMs});
  // 旧 App 仍读前 38 项。出现故障后追加带版本号的快照，不增加 Bridge 往返。
  if (fault.reason >= 0) {
    result.insert(result.end(), {FAULT_STATUS_VERSION, int32_t(fault.reason),
        int32_t(fault.wheel), int32_t(fault.uptime), int32_t(fault.requestedPwm),
        fault.progress, int32_t(fault.awake), int32_t(fault.faultPinHigh),
        fault.vmRaw, fault.vmMinRaw, fault.vmAgeMs});
    for (const auto &wheel : fault.wheels) {
      result.insert(result.end(), {wheel.target, encodeScaled(wheel.speed, 10),
          int32_t(wheel.pwm), int32_t(wheel.count), int32_t(wheel.invalid)});
    }
    result.push_back(int32_t(fault.inputLevels));
  }
  return result;
}

} // namespace

/**
 * @description         : 分页读取 100 Hz 控制拍 trace
 * @param id            : 请求标识，原样回显给调用方做配对
 * @param offset        : 本页首个样本索引，0 为缓冲区里最旧的一拍
 * @return              : 前 8 项为元数据，其后每 16 项一条样本
 */
static std::vector<int32_t> rosTracePoll(int32_t id, int32_t offset) {
  const motion::TracePage page = motion::readTrace(offset);
  std::vector<int32_t> result;
  result.reserve(8 + std::size_t(page.count) * 16);
  // 元数据：会话号、总条数、已丢弃、容量、单样本字节数、周期、本页条数、回显 id。
  result.insert(result.end(), {int32_t(page.epoch), page.total, page.dropped, page.capacity,
      page.bytesPerSample, page.periodMs, page.count, id});
  for (int i = 0; i < page.count; ++i) {
    const motion::TraceSample &s = page.rows[i];
    result.insert(result.end(), {int32_t(s.tickMs), int32_t(s.seq), int32_t(s.dtMs),
        int32_t(s.flags), int32_t(s.target[0]), int32_t(s.target[1]),
        int32_t(s.ramp[0]), int32_t(s.ramp[1]), int32_t(s.speed[0]), int32_t(s.speed[1]),
        int32_t(s.pwm[0]), int32_t(s.pwm[1]), int32_t(s.ffDuty[0]), int32_t(s.ffDuty[1]),
        int32_t(s.rateDpsX100), int32_t(s.vbatRaw)});
  }
  return result;
}

/**
 * @description         : 分页读取 1 Hz 环境样本（IMU 温度与零偏）
 * @param id            : 请求标识，原样回显
 * @param offset        : 本页首个样本索引
 * @return              : 前 4 项为元数据，其后每 4 项一条样本
 */
static std::vector<int32_t> rosEnvPoll(int32_t id, int32_t offset) {
  const motion::EnvPage page = motion::readEnv(offset);
  std::vector<int32_t> result;
  result.reserve(4 + std::size_t(page.count) * 4);
  result.insert(result.end(), {int32_t(page.epoch), page.total, page.dropped, id});
  for (int i = 0; i < page.count; ++i) {
    const motion::EnvSample &s = page.rows[i];
    result.insert(result.end(), {int32_t(s.tickMs), int32_t(s.tempCx100),
        int32_t(s.biasDpsX1000), (int32_t(s.state) << 16) | int32_t(s.seq)});
  }
  return result;
}

/**
 * @description         : 清空 trace 缓冲并推进会话号，开始新一次采集
 * @return              : 恒为 true
 */
static bool rosTraceReset() {
  motion::resetTrace();
  return true;
}

/**
 * @description         : 冻结/解冻 trace 环形缓冲，让分页导出读到同一段数据
 * @param hold          : 非零表示冻结
 * @return              : 恒为 true
 */
static bool rosTraceHold(int32_t hold) {
  motion::setTraceHold(hold != 0);
  return true;
}

/**
 * @description         : 自检：trace 结构尺寸、容量与控制线程栈余量
 * @return              : 4 项整数，便于 Linux 侧核对结构布局与 RAM 预算
 */
static std::vector<int32_t> rosDiag() {
  return {int32_t(sizeof(motion::TraceSample)), int32_t(sizeof(motion::EnvSample)),
          motion::TRACE_SAMPLE_MAX, motion::stackHeadroom()};
}

/**
 * @description         : 返回上电时在驱动休眠状态采集的四路 PWM 波形自检
 * @return              : {passed, driverAsleep, outputsLow, frequencyHz,
 *                         每点 pin, pairedLevelOk, cycles, periodUs, highUs}
 *
 * 现场"PWM 写了 255 但轮子不转"时先看这里：
 *   passed=1 → MCU 四路输入波形（周期、脉宽、配对脚电平）都对，问题在
 *              驱动输出级、电机线或 VM 供电；
 *   passed=0 → pwmReady 本来就是假，所有驱动都会报 PwmError（reason=8）。
 * 自检只在 initPwm() 里跑一次，之后不再改动电机。
 */
static std::vector<int32_t> rosMotorDiag() {
  const ::MotorPwmDiagnostic &d = ::motorPwmDiagnostic();
  std::vector<int32_t> out;
  out.reserve(4 + 6 * 5);
  out.push_back(d.passed ? 1 : 0);
  out.push_back(d.driverAsleep ? 1 : 0);
  out.push_back(d.outputsLow ? 1 : 0);
  out.push_back(int32_t(d.frequencyHz));
  for (const ::MotorPwmTestPoint &p : d.points) {
    out.push_back(p.pin);
    out.push_back(p.pairedLevelOk ? 1 : 0);
    out.push_back(int32_t(p.timing.cycles));
    out.push_back(int32_t(p.timing.periodNs / 1000));
    out.push_back(int32_t(p.timing.highNs / 1000));
  }
  // 末尾三项是**实时**读数，与上面的上电自检不同：
  //   inputLevels —— D12/D13/D9/D10 当前电平（D12 为最低位）
  //   awake       —— nSLEEP 回读
  //   faultPin    —— nFAULT 是否有效
  // 在驱动过程中调用它，才能判断"方向脚真的拉到高电平了吗"。
  // 自检走的是休眠下的 PWM 路径，而实际驱动走的是 GPIO 常高路径，
  // 两条路径不同，自检通过不代表驱动时电平正确。
  out.push_back(int32_t(::motorInputLevels()));
  out.push_back(::motorIsAwake() ? 1 : 0);
  out.push_back(::motorHasFault() ? 1 : 0);
  return out;
}

/**
 * @description         : 读取全部运行期可调参数
 * @return              : TuneParamCount 项整数，顺序见 motion::TuneParam
 */
static std::vector<int32_t> rosTuneGet() {
  int32_t values[motion::TuneParamCount] = {};
  motion::tuneRead(values);
  return std::vector<int32_t>(values, values + motion::TuneParamCount);
}

/**
 * @description         : 写入单个运行期参数
 * @param index         : TuneParam 中的索引
 * @param value         : 按该参数编码规则量化后的整数
 * @return              : 索引有效且数值在范围内时返回 true
 */
static bool rosTuneSet(int32_t index, int32_t value) {
  return motion::tuneWrite(int(index), value);
}

/**
 * @description         : 把全部运行期参数恢复成出厂标定值
 * @return              : 恒为 true
 */
static bool rosTuneReset() {
  motion::tuneReset();
  return true;
}

void motionBridgeBegin() {
  Bridge.begin();
}

void motionBridgeRegister() {
  Bridge.provide_safe("ros_drive", rosDrive);
  Bridge.provide_safe("ros_drive_heading", rosDriveHeading);
  Bridge.provide_safe("ros_stop", rosStop);
  Bridge.provide_safe("ros_reset_drive", rosResetDrive);
  Bridge.provide_safe("ros_status", rosStatus);
  Bridge.provide_safe("ros_calibrate_heading", rosCalibrateHeading);
  // 高频诊断使用独立调用，ros_status 前 38 项的布局保持原样。
  Bridge.provide_safe("ros_trace_poll", rosTracePoll);
  Bridge.provide_safe("ros_env_poll", rosEnvPoll);
  Bridge.provide_safe("ros_trace_reset", rosTraceReset);
  Bridge.provide_safe("ros_trace_hold", rosTraceHold);
  Bridge.provide_safe("ros_diag", rosDiag);
  Bridge.provide_safe("ros_motor_diag", rosMotorDiag);
  // 在线调参：只读写运行期变量。
  Bridge.provide_safe("ros_tune_get", rosTuneGet);
  Bridge.provide_safe("ros_tune_set", rosTuneSet);
  Bridge.provide_safe("ros_tune_reset", rosTuneReset);
  // 雷达走独立的 Serial1 与解析线程，这里只把它的 RPC 挂上去。
  lidarBridgeRegister();
}
