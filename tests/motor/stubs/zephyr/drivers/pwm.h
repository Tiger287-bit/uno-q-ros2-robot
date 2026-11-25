// 主机测试用的 PWM 桩：记录每次 pwm_set_dt，供测试重放引脚状态。
#pragma once

#include <cstdint>

#include <Arduino.h>

// 设备树宏的替身：四个引脚各有一条 PWM 通道，通道顺序由 pinMap 决定。
// 引脚顺序故意与固件的 inputPins（12、13、9、10）不同：这样 initPwm() 查出来的
// pwmIndex[] 与输入索引不再相等，任何"通道号恰好等于输入索引"的假设都会立刻暴露。
inline constexpr int pinMap[4] = {9, 12, 10, 13};

struct device {};

struct pwm_dt_spec {
  const struct device *dev;
  uint32_t channel;
  uint32_t flags;
};

inline const struct device deviceStorage{};
inline const struct device *const deviceInstance = &deviceStorage;

inline int pwm_set_dt(const struct pwm_dt_spec *spec, uint32_t period, uint32_t pulse) {
  (void)period;
  // channel 是 PWM 设备里的通道号，不是输入索引。固件靠 pwmIndex[]（按引脚号在
  // pwmPins[] 里查出来的）把输入映射到通道，所以这里必须按 pinMap 折回物理引脚，
  // 再交给和 GPIO 同一套记录器 —— 否则桩等于偷偷规定"通道顺序 == inputPins 顺序"，
  // 而这个前提在真实设备树里并不成立。
  recordPin(2, pinMap[int(spec->channel)], int(pulse));
  return 0;
}

inline bool pwm_is_ready_dt(const struct pwm_dt_spec *spec) { return spec->dev != nullptr; }

#define DT_PATH(x) 0
#define DT_FOREACH_PROP_ELEM(node, prop, fn) fn(node, prop, 0) fn(node, prop, 1) \
  fn(node, prop, 2) fn(node, prop, 3)
#define PWM_DT_SPEC_GET_BY_IDX(node, index) {deviceInstance, index, 0}
#define DT_PHANDLE_BY_IDX(node, prop, index) (index)
#define DT_REG_ADDR(phandle) (phandle)
#define DT_PHA_BY_IDX(node, prop, index, cell) (0)
#define DIGITAL_PIN_GPIOS_FIND_PIN(reg, pha) (pinMap[(reg)])
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
