#pragma once

#include <stdint.h>

enum class ImuHeadingState {
  Unavailable,  // IMU 尚未成功初始化。
  Calibrating,  // 正在积累静止样本。
  Ready,        // 航向数据可用于闭环。
  Stale         // 最近样本超过允许的时效。
};

struct ImuHeadingSample {
  ImuHeadingState state = ImuHeadingState::Unavailable;  // 样本可用状态。
  int32_t ageMs = -1;     // 样本距当前的时间，单位为毫秒，-1 表示尚未发布有效时刻。
  uint32_t samples = 0;   // 本轮校准开始后接收的有效样本数。
  float yawDeg = 0;       // 相对校准起点累计的航向角，单位为度，向左转为正。
  float rateDegPerS = 0;  // 沿车体竖直轴投影并扣除零偏的角速度，单位为度每秒。
  float biasDegPerS = 0;  // 沿车体竖直轴投影的陀螺零偏，单位为度每秒。
  float temperatureC = 0; // 芯片温度，℃。LSM6DS3TR-C：16 LSB/℃，0 码对应 25 ℃。
};

/**
 * @description         : 启动独立的 IMU 采样线程
 * @return              : void
 */
void headingImuBegin();

/**
 * @description         : 请求重新采集静止零偏和车体竖直方向
 * @return              : 已提交请求时返回 true
 */
bool headingImuCalibrate();

/**
 * @description         : 读取相对航向、偏航角速度和样本状态快照
 * @return              : IMU 航向样本
 */
ImuHeadingSample headingImuRead();
