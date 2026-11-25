#pragma once

#include <cstdint>

struct MotorPwmTiming {
  uint32_t periodNs = 0; // 完整有效周期的平均长度，ns。
  uint32_t highNs = 0;   // 完整有效周期内高电平的平均宽度，ns。
  uint16_t cycles = 0;   // 纳入统计的有效周期数。
};

/**
 * @description         : 在电机驱动休眠时读取 MCU 引脚的 20 kHz PWM 波形。
 * @param pin           : Arduino 引脚编号，保持当前引脚复用配置。
 * @return              : PWM 周期和高电平脉宽，少于 3 个完整周期时两项为 0。
 */
MotorPwmTiming measureMotorPwm(int pin);

/**
 * @description         : 在电机驱动休眠时检查 MCU 引脚能否连续保持指定电平。
 * @param pin           : Arduino 引脚编号，保持当前引脚复用配置。
 * @param high          : 要求的物理电平，true 为高电平。
 * @return              : 连续约 300 us 的样本均符合要求时返回 true。
 */
bool motorPinHoldsLevel(int pin, bool high);
