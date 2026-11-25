// 主机测试用的 Arduino 桩：pinMode / digitalWrite 经统一钩子记录，供测试回放。
//
// digitalRead 必须交给测试实现，否则 motorIsAwake() 恒为真：
// initPwm() 末尾的 probeMotorInputs() 会在第一行 `if (!driverAsleep) return false`
// 直接返回，6 个自检点一次都不跑，而 initPwm() 的返回值以前没人接 —— 零覆盖
// 也会显示全绿。
#pragma once

#include <cstdint>

constexpr int HIGH = 1;
constexpr int LOW = 0;
constexpr int OUTPUT = 1;
constexpr int INPUT = 0;
constexpr int GPIO_OUTPUT_LOW = 0;
constexpr int GPIO_OUTPUT_HIGH = 1;

// kind 0 = pinMode，1 = digitalWrite，2 = pwm_set_dt，3 = 带初始电平的 GPIO 配置。
void recordPin(int kind, int pin, int value);

// 由测试文件实现：返回该引脚当前电平，未记录的引脚返回 LOW。
int stubReadPin(int pin);

inline void pinMode(int pin, int mode) { recordPin(0, pin, mode); }
inline void digitalWrite(int pin, int value) { recordPin(1, pin, value); }
inline int digitalRead(int pin) { return stubReadPin(pin); }
struct device;
inline const device *digitalPinToPortDevice(int) { return nullptr; }
inline int digitalPinToPinIndex(int pin) { return pin; }
inline int gpio_pin_configure(const device *, int pin, int flags) {
  recordPin(3, pin, flags == GPIO_OUTPUT_HIGH ? HIGH : LOW);
  return 0;
}
inline uint32_t millis() { return 0; }
