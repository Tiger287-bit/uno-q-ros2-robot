#include "motor_driver.h"

#include <Arduino.h>
#include <zephyr/drivers/pwm.h>
#include <zephyrPinctrl.h>

#include <algorithm>

namespace {

constexpr int SLEEP_PIN = 7;  // AT8833 的 nSLEEP，高电平唤醒。
constexpr int FAULT_PIN = 8;  // AT8833 的 nFAULT，低电平有效。
constexpr int inputPins[] = {12, 13, 9, 10};  // M1 和 M2 各占两路 H 桥输入。
constexpr int driveSigns[] = {-1, 1};  // M1、M2 的逻辑前进方向与 H 桥极性的换算。
constexpr size_t pinctrlIndices[] = {3, 2, 0, 1};  // UNO Q core 1.1.0 的 PWM 引脚复用索引。
constexpr int FULL_DUTY = 255;               // 100% 有效驱动占空比的代码值。
constexpr uint32_t MOTOR_PWM_HZ = 20000;      // 电机 PWM 载波频率，Hz。
constexpr uint32_t MOTOR_PWM_PERIOD_NS = 1000000000UL / MOTOR_PWM_HZ; // 电机 PWM 周期，ns。
constexpr int DIAGNOSTIC_DUTY[] = {92, 71};   // 原故障试跑的第一档代码值，M1、M2。
constexpr uint32_t WAVE_TOLERANCE_NS = 4000;   // 引脚轮询误差及定时器量化余量，ns。

#define PWM_SPEC(node, prop, index) PWM_DT_SPEC_GET_BY_IDX(node, index),
#define PWM_PIN(node, prop, index) DIGITAL_PIN_GPIOS_FIND_PIN( \
    DT_REG_ADDR(DT_PHANDLE_BY_IDX(node, prop, index)), DT_PHA_BY_IDX(node, prop, index, pin)),

const struct pwm_dt_spec pwmSpecs[] = {DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), pwms, PWM_SPEC)};
const int pwmPins[] = {DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), pwm_pin_gpios, PWM_PIN)};
#undef PWM_SPEC
#undef PWM_PIN
static_assert(ARRAY_SIZE(pwmSpecs) == ARRAY_SIZE(pwmPins), "PWM pin mapping");

int pwmIndex[] = {-1, -1, -1, -1};  // 各输入对应的 PWM 设备索引，-1 表示尚未映射。
int pinDuty[4] = {};               // 每个物理输入引脚的高电平占空比代码值。
int activeInput[] = {-1, -1};      // 每轮当前保持高电平的方向输入，-1 表示两脚都低。
MotorPwmDiagnostic diagnostic;     // 驱动休眠时的上电自检结果。

/**
 * @description         : 把驱动输入切换为指定电平的 GPIO
 * @param index         : 驱动输入索引
 * @param high          : true 为高电平，false 为低电平
 * @return              : GPIO 电平与输出模式配置成功时返回 true
 */
bool holdInputLevel(int index, bool high) {
  const int pin = inputPins[index];
  // Arduino pinMode(OUTPUT) 会先输出低。制动时配对脚被短暂拉低会产生全驱动。
  return gpio_pin_configure(digitalPinToPortDevice(pin), digitalPinToPinIndex(pin),
                            high ? GPIO_OUTPUT_HIGH : GPIO_OUTPUT_LOW) == 0;
}

/**
 * @description         : 写入单个物理引脚的高电平占空比
 * @param index         : 驱动输入索引
 * @param duty          : 0 到 255 的引脚高电平占空比代码值
 * @return              : 引脚输出配置成功时返回 true
 */
bool writeInputDuty(int index, int duty) {
  // 兜底夹取：越界的代码值会让脉冲宽度超过周期，pwm_set_dt 返回错误，
  // 进而把 pwmReady 永久置假。调用方本应已限幅，这里再挡一道。
  duty = std::clamp(duty, 0, FULL_DUTY);
  const int previous = pinDuty[index]; // 上一次实际写入的高电平占空比代码值。
  if (duty == previous) return true;
  const auto &spec = pwmSpecs[pwmIndex[index]];
  if (duty == 0 || duty == FULL_DUTY) {
    if (!holdInputLevel(index, duty == FULL_DUTY)) return false;
    if (pwm_set_dt(&spec, MOTOR_PWM_PERIOD_NS, 0) != 0) return false;
  } else {
    const uint32_t pulseNs = MOTOR_PWM_PERIOD_NS * uint32_t(duty) / FULL_DUTY;
    if (pwm_set_dt(&spec, MOTOR_PWM_PERIOD_NS, pulseNs) != 0) return false;
    if ((previous == 0 || previous == FULL_DUTY) &&
        zephyr::arduino::init_dev_apply_channel_pinctrl(spec.dev, pinctrlIndices[index]) != 0) return false;
  }
  pinDuty[index] = duty;
  return true;
}

/**
 * @description         : 把一轮输出切到制动，中途不经过任何驱动状态
 * @param input         : 当前保持高电平的方向输入索引
 * @return              : 配对输入已切到高电平时返回 true
 */
bool clearWheelOutput(int input) {
  // 驱动表（见 applyDuty）：方向脚高、配对脚低才是驱动，幅度 = 1 - 配对占空比。
  // 所以任何"先清某一脚"的两步走，都会在两次写入之间留下满速窗口：
  //   先清配对（原顺序）→ 方向高、配对 0   = 正向 100%；
  //   先清方向          → 方向 0、配对还高 = 反向 100%。
  // 把配对脚拉到高电平只有一次写入，方向高 + 配对高 = 制动、零驱动，
  // 中间不存在驱动状态；随后由 applyDuty 直接从这个制动状态切到新方向。
  return writeInputDuty(input ^ 1, FULL_DUTY);
}

/**
 * @description         : 检查指定高电平比例对应的引脚波形及配对输入
 * @param point         : 当前测试点记录
 * @param duty          : 期望的引脚高电平占空比代码值
 * @return              : 周期、脉宽和配对输入均符合预期时返回 true
 */
bool matchesWaveform(const MotorPwmTestPoint &point, int duty) {
  const uint32_t pulseNs = MOTOR_PWM_PERIOD_NS * uint32_t(duty) / FULL_DUTY;
  const MotorPwmTiming &timing = point.timing;
  return point.pairedLevelOk && timing.cycles >= 3 &&
      timing.periodNs + WAVE_TOLERANCE_NS >= MOTOR_PWM_PERIOD_NS &&
      timing.periodNs <= MOTOR_PWM_PERIOD_NS + WAVE_TOLERANCE_NS &&
      timing.highNs + WAVE_TOLERANCE_NS >= pulseNs &&
      timing.highNs <= pulseNs + WAVE_TOLERANCE_NS;
}

/**
 * @description         : 驱动休眠时回读旧快衰减和新慢衰减的实际引脚波形
 * @return              : 慢衰减的正反方向均通过且驱动仍处于休眠时返回 true
 */
bool probeMotorInputs() {
  diagnostic.frequencyHz = MOTOR_PWM_HZ;
  diagnostic.driverAsleep = !motorIsAwake();
  if (!diagnostic.driverAsleep) return false;

  bool slowPassed = true; // 两轮正反方向的慢衰减测试汇总结果。
  for (int pointIndex = 0; pointIndex < 6; ++pointIndex) {
    const bool slow = pointIndex >= 2;
    const int wheel = pointIndex % 2;
    const int active = driveInput(wheel, pointIndex >= 4 ? -1 : 1);
    const int pwm = slow ? active ^ 1 : active;
    const int duty = DIAGNOSTIC_DUTY[wheel];
    MotorPwmTestPoint &point = diagnostic.points[pointIndex];
    point.pin = inputPins[pwm];
    point.pairedPin = inputPins[pwm ^ 1];
    if (motorIsAwake()) {
      diagnostic.driverAsleep = false;
      break;
    }
    const bool applied = slow ? applyDuty(active, duty) : writeInputDuty(active, duty);
    if (applied) {
      point.timing = measureMotorPwm(point.pin);
      point.pairedLevelOk = motorPinHoldsLevel(point.pairedPin, slow);
    }
    const bool cleared = motorDisableOutputs();
    diagnostic.driverAsleep = diagnostic.driverAsleep && !motorIsAwake();
    if (!cleared || !diagnostic.driverAsleep) {
      slowPassed = false;
      break;
    }
    if (slow) slowPassed = slowPassed && applied && matchesWaveform(point, FULL_DUTY - duty);
  }
  diagnostic.outputsLow = true;
  for (int pin : inputPins) diagnostic.outputsLow = motorPinHoldsLevel(pin, false) && diagnostic.outputsLow;
  diagnostic.passed = diagnostic.driverAsleep && diagnostic.outputsLow && slowPassed;
  return diagnostic.passed;
}

} // namespace

void motorInitPins() {
  digitalWrite(SLEEP_PIN, LOW);
  pinMode(SLEEP_PIN, OUTPUT);
  digitalWrite(SLEEP_PIN, LOW);
  for (int i = 0; i < 4; ++i) holdInputLevel(i, false);
  pinMode(FAULT_PIN, INPUT);
}

void motorSetAwake(bool awake) {
  digitalWrite(SLEEP_PIN, awake ? HIGH : LOW);
}

bool motorIsAwake() {
  return digitalRead(SLEEP_PIN) == HIGH;
}

bool motorHasFault() {
  return digitalRead(FAULT_PIN) != HIGH;
}

uint8_t motorInputLevels() {
  uint8_t levels = 0;
  for (int i = 0; i < 4; ++i) {
    if (digitalRead(inputPins[i]) == HIGH) levels |= uint8_t(1u << i);
  }
  return levels;
}

bool motorDisableOutputs() {
  bool cleared = true; // 即使某个通道清零失败，也继续拉低全部引脚。
  for (int i = 0; i < 4; ++i) {
    if (!holdInputLevel(i, false)) cleared = false;
    if (pwmIndex[i] >= 0 && pwm_is_ready_dt(&pwmSpecs[pwmIndex[i]]) &&
        pwm_set_dt(&pwmSpecs[pwmIndex[i]], MOTOR_PWM_PERIOD_NS, 0) != 0) cleared = false;
    pinDuty[i] = 0;
  }
  activeInput[0] = activeInput[1] = -1;
  return cleared;
}

bool motorPrepareBrake() {
  if (motorIsAwake()) return false;
  // 休眠时把两轮输入各自置成双高，再唤醒。首拍只需改配对脚的 PWM。
  for (int i = 0; i < 4; ++i) {
    if (!writeInputDuty(i, FULL_DUTY)) return false;
  }
  return true;
}

bool initPwm() {
  diagnostic = {};
  for (int i = 0; i < 4; ++i) {
    for (size_t j = 0; j < ARRAY_SIZE(pwmPins); ++j) {
      if (pwmPins[j] == inputPins[i]) {
        pwmIndex[i] = int(j);
        break;
      }
    }
    if (pwmIndex[i] < 0) return false;
    const auto &spec = pwmSpecs[pwmIndex[i]];
    const bool ready = zephyr::arduino::init_dev_apply_channel_pinctrl(spec.dev, pinctrlIndices[i]) == 0
      && pwm_is_ready_dt(&spec) && pwm_set_dt(&spec, MOTOR_PWM_PERIOD_NS, 0) == 0;
    if (!holdInputLevel(i, false) || !ready) return false;
  }
  return probeMotorInputs();
}

int driveInput(int wheel, int logicalDirection) {
  return 2 * wheel + (logicalDirection * driveSigns[wheel] < 0 ? 1 : 0);
}

bool applyDuty(int index, int duty) {
  int &active = activeInput[index / 2]; // 当前电机的方向输入。
  if (active >= 0 && (duty == 0 || active != index)) {
    if (!clearWheelOutput(active)) return false;
    active = -1;
  }
  if (duty == 0) return true;

  // 方向输入保持高电平，配对输入为高时制动，为低时按选定方向驱动。
  if (active < 0 && !writeInputDuty(index, FULL_DUTY)) return false;
  if (!writeInputDuty(index ^ 1, FULL_DUTY - duty)) return false;
  active = index;
  return true;
}

const MotorPwmDiagnostic &motorPwmDiagnostic() {
  return diagnostic;
}
