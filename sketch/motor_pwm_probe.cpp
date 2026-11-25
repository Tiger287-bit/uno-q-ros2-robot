#include "motor_pwm_probe.h"

#include <Arduino.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/time_units.h>

namespace {

constexpr uint32_t SAMPLE_GAP_US = 5;       // 两次连续读取之间允许的最长间隔，us。
constexpr uint32_t PWM_WINDOW_US = 3000;   // 采集完整 PWM 周期的最长时间，us。
constexpr uint32_t MIN_PERIOD_US = 40;     // 用于筛选 20 kHz 波形的最短有效周期，us。
constexpr uint32_t MAX_PERIOD_US = 60;     // 用于筛选 20 kHz 波形的最长有效周期，us。
constexpr uint16_t MIN_COMPLETE_CYCLES = 3; // 计算平均脉宽所需的最少完整周期数。
constexpr uint32_t LEVEL_WINDOW_US = 300;  // 持续检查引脚电平的最短时间，us。
constexpr uint32_t LEVEL_TIMEOUT_US = 1000; // 含线程抢占在内的最长电平检查时间，us。

/**
 * @description         : 将采样时长转换为 Zephyr 硬件计数。
 * @param microseconds  : 采样时长，us。
 * @param clockHz       : Zephyr 硬件计数频率，Hz。
 * @return              : 与采样时长对应的计数值。
 */
uint32_t toCycles(uint32_t microseconds, uint32_t clockHz) {
  return uint32_t(uint64_t(microseconds) * clockHz / 1000000ULL);
}

/**
 * @description         : 将完整周期的累计计数转换为平均时间。
 * @param totalCycles   : 有效周期或脉宽的硬件累计计数。
 * @param count         : 有效周期数量，调用者保证大于 0。
 * @param clockHz       : Zephyr 硬件计数频率，Hz。
 * @return              : 平均时间，ns。
 */
uint32_t meanNs(uint32_t totalCycles, uint16_t count, uint32_t clockHz) {
  return uint32_t(uint64_t(totalCycles) * 1000000000ULL / (uint64_t(count) * clockHz));
}

} // namespace

MotorPwmTiming measureMotorPwm(int pin) {
  const uint32_t clockHz = sys_clock_hw_cycles_per_sec(); // 硬件计数频率，Hz。
  const uint32_t limit = toCycles(PWM_WINDOW_US, clockHz);
  const uint32_t maxGap = toCycles(SAMPLE_GAP_US, clockHz);
  const uint32_t minPeriod = toCycles(MIN_PERIOD_US, clockHz);
  const uint32_t maxPeriod = toCycles(MAX_PERIOD_US, clockHz);
  bool previousHigh = digitalRead(pin) == HIGH; // 上次读取的物理电平。
  const uint32_t start = k_cycle_get_32();     // 采样窗口起始计数。
  uint32_t previousAt = start;                // 上次读取的硬件计数。
  uint32_t riseAt = 0;                        // 最近一个有效上升沿的硬件计数。
  uint32_t highCycles = 0;                    // 本周期的高电平宽度，硬件计数。
  bool haveRise = false;                      // 已观察到本周期的上升沿。
  bool haveFall = false;                      // 已观察到本周期的下降沿。
  uint32_t periodTotal = 0;                   // 有效周期长度之和，硬件计数。
  uint32_t highTotal = 0;                     // 有效高电平宽度之和，硬件计数。
  MotorPwmTiming timing;

  while (true) {
    const bool high = digitalRead(pin) == HIGH;
    const uint32_t now = k_cycle_get_32();
    if (now - start >= limit) break;
    if (now - previousAt > maxGap) {
      // 放弃采样间隔过大的半周期，避免把抢占期间的脉冲计为一个周期。
      haveRise = false;
      haveFall = false;
    } else if (high != previousHigh) {
      if (high) {
        const uint32_t period = now - riseAt;
        if (haveRise && haveFall && period >= minPeriod && period <= maxPeriod &&
            highCycles > 0 && highCycles < period) {
          periodTotal += period;
          highTotal += highCycles;
          ++timing.cycles;
        }
        riseAt = now;
        haveRise = true;
        haveFall = false;
      } else if (haveRise) {
        highCycles = now - riseAt;
        haveFall = true;
      }
    }
    previousAt = now;
    previousHigh = high;
  }

  if (timing.cycles >= MIN_COMPLETE_CYCLES) {
    timing.periodNs = meanNs(periodTotal, timing.cycles, clockHz);
    timing.highNs = meanNs(highTotal, timing.cycles, clockHz);
  }
  return timing;
}

bool motorPinHoldsLevel(int pin, bool high) {
  const uint32_t clockHz = sys_clock_hw_cycles_per_sec(); // 硬件计数频率，Hz。
  const uint32_t window = toCycles(LEVEL_WINDOW_US, clockHz);
  const uint32_t limit = toCycles(LEVEL_TIMEOUT_US, clockHz);
  const uint32_t maxGap = toCycles(SAMPLE_GAP_US, clockHz);
  const uint32_t start = k_cycle_get_32(); // 整个电平检查的起始计数。
  uint32_t continuousAt = start;        // 连续有效检查段的起始计数。
  uint32_t previousAt = start;          // 上次读取的硬件计数。

  while (true) {
    if ((digitalRead(pin) == HIGH) != high) return false;
    const uint32_t now = k_cycle_get_32();
    if (now - start >= limit) return false;
    if (now - previousAt > maxGap) continuousAt = now;
    if (now - continuousAt >= window) return true;
    previousAt = now;
  }
}
