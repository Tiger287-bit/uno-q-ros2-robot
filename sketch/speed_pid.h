#pragma once

#include <algorithm>

struct SpeedPid {
  float kp = 0;               // 亚博原生单位的比例增益。
  float ki = 0;               // 亚博原生单位的离散积分增益，每次更新累加。
  float kd = 0;               // 亚博原生单位的离散微分增益。
  float output = 0;           // 受执行器余量约束的原生 PWM 修正量。
  float previousError = 0;    // 上一拍轮速误差，mm/s。
  float olderError = 0;       // 前两拍的轮速误差，mm/s。

  /**
   * @description         : 停止时清除增量输出和误差历史
   * @return              : void
   */
  void reset() {
    output = previousError = olderError = 0;
  }

  /**
   * @description         : 按亚博逐轮增量式 PID 公式更新 PWM
   * @param targetMmPerS  : 沿命令方向的轮速目标，mm/s
   * @param speedMmPerS   : 沿命令方向的实测轮速，mm/s
   * @param lower         : 扣除前馈后的修正下限，亚博原生量程。
   * @param upper         : 扣除前馈后的修正上限，亚博原生量程。
   * @return              : 沿命令方向的增量式 PID 输出，亚博原生量程
   */
  float update(float targetMmPerS, float speedMmPerS, float lower, float upper) {
    const float error = targetMmPerS - speedMmPerS; // 本拍轮速误差，mm/s。
    const float delta = kp * (error - previousError) + ki * error
        + kd * (error - 2.0f * previousError + olderError); // 本拍 PWM 增量。
    olderError = previousError;
    previousError = error;
    output = std::clamp(output + delta, lower, upper);
    return output;
  }
};
