#pragma once

#include "motor_pwm_probe.h"

/**
 * @description         : 将驱动置于休眠并初始化四路输入和故障脚
 * @return              : void
 */
void motorInitPins();

/**
 * @description         : 设置驱动的 nSLEEP 电平
 * @param awake         : true 唤醒驱动，false 休眠
 * @return              : void
 */
void motorSetAwake(bool awake);

/**
 * @description         : 读取驱动 nSLEEP 电平
 * @return              : 唤醒脚为高电平时返回 true
 */
bool motorIsAwake();

/**
 * @description         : 读取驱动 nFAULT 电平
 * @return              : 故障脚有效时返回 true
 */
bool motorHasFault();

/** 读取 D12、D13、D9、D10 的瞬时电平，各占一位，D12 为最低位。 */
uint8_t motorInputLevels();

/**
 * @description         : 拉低四路输入并清零已就绪的 PWM 通道
 * @return              : 已就绪的 PWM 通道均清零成功时返回 true
 */
bool motorDisableOutputs();

/** 在驱动休眠时将两轮输入置成双高制动，为下一次唤醒准备。 */
bool motorPrepareBrake();

/**
 * @description         : 初始化四路硬件 PWM 并在驱动休眠时检查慢衰减输入波形
 * @return              : 初始化和慢衰减波形检查通过时返回 true
 */
bool initPwm();

/**
 * @description         : 查找逻辑行驶方向对应的 H 桥输入
 * @param wheel         : 电机索引，0 为 M1 右轮，1 为 M2 左轮
 * @param logicalDirection : 正值为前进，负值为后退
 * @return              : 驱动输入索引，顺序为 D12、D13、D9、D10
 */
int driveInput(int wheel, int logicalDirection);

/**
 * @description         : 更新单轮慢衰减驱动，活动轮归零时进入双高制动
 * @param index         : 选定方向的输入索引，顺序为 D12、D13、D9、D10
 * @param duty          : 0 到 255 的有效驱动占空比代码值
 * @return              : 写入成功时返回 true
 */
bool applyDuty(int index, int duty);

struct MotorPwmTestPoint {
  int pin = 0;                 // 实际测量的 PWM 引脚编号。
  int pairedPin = 0;           // 同一电机的另一个驱动输入。
  MotorPwmTiming timing;       // 从 PWM 引脚读回的周期、高电平脉宽和周期数。
  bool pairedLevelOk = false;  // 另一个输入保持规定电平。
};

struct MotorPwmDiagnostic {
  bool passed = false;             // 四组慢衰减波形和收尾检查全部通过。
  bool driverAsleep = false;       // 自检期间 nSLEEP 始终回读为低。
  bool outputsLow = false;         // 自检后四个驱动输入均回读为低。
  uint32_t frequencyHz = 0;        // 设定的电机 PWM 载波频率，Hz。
  MotorPwmTestPoint points[6];     // 快衰减正转、慢衰减正转、慢衰减反转，各 M1/M2。
};

/**
 * @description         : 读取初始化时在休眠状态下采集的 PWM 引脚诊断
 * @return              : 本次上电自检的结果
 */
const MotorPwmDiagnostic &motorPwmDiagnostic();
