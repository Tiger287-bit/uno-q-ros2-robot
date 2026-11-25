// 主机测试：重放 motor_driver 的引脚写入序列，检查换向与停车过程中
// 有没有"某个引脚还高、另一个已低"的满速驱动窗口。
//
// 驱动表（motor_driver.cpp applyDuty）：同一轮两个 H 桥输入里，一个保持
// 高电平当方向脚，另一个按 (255 - duty) 调制，高电平部分是制动。于是
//   方向高 + 调制脚高   = 制动，零驱动
//   方向高 + 调制脚低   = 该方向 100%
//   方向高 + 调制脚 p%  = 该方向 (1 - p/255)
// 任何"先清某个脚"的两步走都会在两次写入之间留下一个 100% 窗口。
//
// 桩只替换引脚与 PWM 驱动，其余直接编固件源码（motor_pwm_probe.h 用固件那份，
// 只把 measureMotorPwm / motorPinHoldsLevel 两个函数在这里实现）。因此
// probeMotorInputs() 里的波形断言是"驱动自证"：波形从已记录的引脚状态合成，
// 证明不了真实波形。这一段的价值在于把那 6 次换向序列纳入回放。

#include <Arduino.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "motor_driver.cpp"  // 直接编固件源码，只替换引脚与 PWM 驱动

namespace {

constexpr int PINS[4] = {12, 13, 9, 10};  // 与 motor_driver.cpp 的 inputPins 一致
// 名字带 _TEST：与固件同处一个匿名命名空间，重名会直接编不过。
constexpr int SLEEP_PIN_TEST = 7;         // AT8833 的 nSLEEP，高电平唤醒
constexpr int FAULT_PIN_TEST = 8;         // AT8833 的 nFAULT，低电平有效
constexpr double PERIOD_NS = 50000.0;     // 20 kHz 载波周期
constexpr double EPS = 1e-6;

// 前进方向对应的方向脚，按电机接线写死。故意不从固件的 driveSigns 取：
// 那张表本身也在被测范围内（M1 前进是索引 1，M2 是索引 2，二者相反）。
constexpr int FORWARD_PIN[2] = {1, 2};

// mode：0 = 输入/高阻，1 = GPIO 输出，2 = 复用为 PWM。
struct PinState {
  int mode = INPUT;
  int level = LOW;
  long pulseNs = -1;
};

PinState pins[4];
int sleepLevel = LOW;  // 上电状态：驱动休眠，与 motorInitPins() 之后一致。
int failures = 0;
double maxDrive[2] = {0.0, 0.0};  // 自上次 resetPeak() 以来每轮 |驱动| 的最大值。

int indexOfPin(int pin) {
  for (int i = 0; i < 4; ++i) {
    if (PINS[i] == pin) return i;
  }
  return -1;
}

double highFraction(int index) {
  const PinState &p = pins[index];
  if (p.mode == OUTPUT) return p.level ? 1.0 : 0.0;
  if (p.mode == 2 && p.pulseNs >= 0) return double(p.pulseNs) / PERIOD_NS;
  return 0.0;
}

/** 该轮当前的有效驱动：正值前进、负值后退、0 为制动或滑行。 */
double driveOf(int wheel) {
  const int forward = FORWARD_PIN[wheel];
  const double dir = highFraction(forward);
  const double pair = highFraction(forward ^ 1);
  if (dir >= 1.0 - EPS && pair >= 1.0 - EPS) return 0.0;  // 两脚同高：制动
  if (dir >= 1.0 - EPS) return 1.0 - pair;                 // 方向脚高：按配对脚调制
  if (pair >= 1.0 - EPS) return -(1.0 - dir);              // 换了方向：反向驱动
  return 0.0;                                              // 两脚都低：滑行
}

/** 记录两轮当前 |驱动| 的峰值。必须按轮分开：只动一轮时，另一轮的稳态
 *  驱动也算进来，就会把"低占空比换向"误判成满速窗口。 */
void samplePeak() {
  if (sleepLevel == LOW) return;
  for (int wheel = 0; wheel < 2; ++wheel) {
    const double d = std::fabs(driveOf(wheel));
    if (d > maxDrive[wheel]) maxDrive[wheel] = d;
  }
}

void expect(bool ok, const char *label, double got, double want) {
  if (ok) {
    std::printf("PASS  %-46s got=%.3f\n", label, got);
  } else {
    std::printf("FAIL  %-46s got=%.3f want<=%.3f\n", label, got, want);
    ++failures;
  }
}

void checkNear(const char *label, double got, double want) {
  expect(std::fabs(got - want) <= 1e-3, label, got, want);
}

void checkInt(const char *label, int got, int want) {
  if (got == want) {
    std::printf("PASS  %-46s got=%d\n", label, got);
  } else {
    std::printf("FAIL  %-46s got=%d want=%d\n", label, got, want);
    ++failures;
  }
}

void checkFlag(const char *label, bool ok) {
  std::printf("%s  %-46s %s\n", ok ? "PASS" : "FAIL", label, ok ? "yes" : "no");
  if (!ok) ++failures;
}

void resetPeak() { maxDrive[0] = maxDrive[1] = 0.0; }

}  // namespace

int stubReadPin(int pin) {
  if (pin == SLEEP_PIN_TEST) return sleepLevel;
  if (pin == FAULT_PIN_TEST) return HIGH;  // 低电平有效，因此高电平表示无故障
  const int i = indexOfPin(pin);
  return i >= 0 ? pins[i].level : LOW;
}

// motor_pwm_probe.h 的两个实现：从引脚状态合成，不测真实波形。
MotorPwmTiming measureMotorPwm(int pin) {
  MotorPwmTiming timing;
  const int i = indexOfPin(pin);
  if (i < 0) return timing;
  timing.periodNs = uint32_t(PERIOD_NS);
  timing.highNs = uint32_t(highFraction(i) * PERIOD_NS);
  timing.cycles = 4;
  return timing;
}

bool motorPinHoldsLevel(int pin, bool high) {
  const int i = indexOfPin(pin);
  if (i < 0) return false;
  const double fraction = highFraction(i);
  return high ? fraction >= 1.0 - EPS : fraction <= EPS;
}

void recordPin(int kind, int pin, int value) {
  if (pin == SLEEP_PIN_TEST) {
    if (kind == 1) {
      sleepLevel = value;
      samplePeak();
    }
    return;
  }
  const int i = indexOfPin(pin);
  if (i < 0) return;  // 含 pinctrl 的复用索引：那不是引脚号
  if (kind == 0) {
    pins[i].mode = value;
    if (value == OUTPUT) pins[i].level = LOW;  // 对齐 ArduinoCore-zephyr。
  } else if (kind == 1) {
    pins[i].level = value;
  } else if (kind == 2) {
    // 非零脉宽说明这个通道被复用成 PWM 了（duty 0/255 走 GPIO，那里脉宽恒为 0，
    // 且之前已经 pinMode(OUTPUT) 过）。
    pins[i].pulseNs = value;
    if (value > 0) pins[i].mode = 2;
  } else if (kind == 3) {
    pins[i].mode = OUTPUT;
    pins[i].level = value;
  }
  samplePeak();
}

int main() {
  motorInitPins();  // 与固件同一顺序：先配置四路输入，nSLEEP 拉低（休眠）
  const bool pwmOk = initPwm();
  const MotorPwmDiagnostic &diag = motorPwmDiagnostic();

  // 自检必须真的跑过：桩若把 nSLEEP 回读成高电平，motorIsAwake() 恒为真，
  // probeMotorInputs() 会在第一行 return false，下面 6 个点一次都不执行。
  checkFlag("上电自检通过（休眠 + 波形 + 收尾）", pwmOk);
  checkFlag("自检后四脚全低", diag.outputsLow);
  int probed = 0;
  for (const MotorPwmTestPoint &point : diag.points) {
    if (point.timing.cycles >= 3 && point.pairedLevelOk) ++probed;
  }
  checkInt("自检 6 个测试点都测到波形", probed, 6);

  // 驱动在双高制动时唤醒。两轮的第一拍也不能超过目标占空比。
  checkFlag("休眠时预置双轮制动", motorPrepareBrake());
  resetPeak();
  motorSetAwake(true);
  applyDuty(driveInput(0, 1), 204);
  applyDuty(driveInput(1, 1), 204);
  expect(maxDrive[0] <= 0.8 + 1e-3, "M1 首次起步不超过目标占空比", maxDrive[0], 0.8);
  expect(maxDrive[1] <= 0.8 + 1e-3, "M2 首次起步不超过目标占空比", maxDrive[1], 0.8);

  // 1. 换向：+80% → -80%，中途不允许出现超过 80% 的驱动。
  resetPeak();
  applyDuty(driveInput(0, -1), 204);
  expect(maxDrive[0] <= 0.8 + 1e-3, "M1 换向不超过原占空比", maxDrive[0], 0.8);
  checkNear("M1 换向后为反向 80%", driveOf(0), -0.8);

  // 2. 低占空比换向：20% 也不该出现 100% 窗口。
  applyDuty(driveInput(0, 1), 51);
  resetPeak();
  applyDuty(driveInput(0, -1), 51);
  expect(maxDrive[0] <= 0.2 + 1e-3, "低占空比换向不出现满速窗口", maxDrive[0], 0.2);

  // 3. 归零：applyDuty(..., 0) 后两脚同高（制动），不是"方向高 + 配对低"。
  applyDuty(driveInput(0, 1), 204);
  resetPeak();
  applyDuty(driveInput(0, 1), 0);
  expect(maxDrive[0] <= 0.8 + 1e-3, "归零过程中不超过原占空比", maxDrive[0], 0.8);
  checkNear("归零后停在制动（两脚同高）", highFraction(0) + highFraction(1), 2.0);

  // 4. 制动状态可以直接起步到新方向，且不经过 100%。
  resetPeak();
  applyDuty(driveInput(0, 1), 102);
  expect(maxDrive[0] <= 0.4 + 1e-3, "从制动起步不超过目标占空比", maxDrive[0], 0.4);
  checkNear("起步后为正向 40%", driveOf(0), 0.4);

  // 5. M2 换向：引脚 9/10，pwmIndex 与输入索引不同序，换向逻辑写错会先在这里露。
  applyDuty(driveInput(1, 1), 204);
  checkNear("M2 起点为正向 80%", driveOf(1), 0.8);
  resetPeak();
  applyDuty(driveInput(1, -1), 204);
  expect(maxDrive[1] <= 0.8 + 1e-3, "M2 换向不超过原占空比", maxDrive[1], 0.8);
  checkNear("M2 换向后为反向 80%", driveOf(1), -0.8);

  // 6. 两轮到达 GPIO 满驱动端点，分别回到 PWM 调制。
  applyDuty(driveInput(0, 1), 255);
  applyDuty(driveInput(1, 1), 255);
  checkNear("M1 满驱动", driveOf(0), 1.0);
  checkNear("M2 满驱动", driveOf(1), 1.0);
  checkInt("瞬时回读顺序为 D12 D13 D9 D10", motorInputLevels(), 0b0110);
  applyDuty(driveInput(0, 1), 102);
  checkNear("M1 从满驱动恢复调制", driveOf(0), 0.4);
  checkNear("M1 更新期间 M2 仍满驱动", driveOf(1), 1.0);
  applyDuty(driveInput(1, 1), 51);
  checkNear("M2 从满驱动恢复调制", driveOf(1), 0.2);

  // 7. 休眠/故障路径把四个脚全部拉低。
  motorSetAwake(false);
  motorDisableOutputs();
  checkNear("停用输出后四脚全低",
            highFraction(0) + highFraction(1) + highFraction(2) + highFraction(3), 0.0);

  std::printf(failures ? "\n%d 项失败\n" : "\n全部通过\n", failures);
  return failures ? 1 : 0;
}
