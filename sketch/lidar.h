#pragma once

#include <cstdint>

/**
 * RPLIDAR C1 驱动（主工程版）。
 *
 * 只负责 Serial1：D0=RX / D1=TX，460800 8N1。电机由串口 SET_MOTOR_PWM 启动，
 * 不动任何 GPIO —— 底盘电机引脚归 motor_driver 管，两边互不干扰。
 *
 * 每圈按 180 个 2° 分区归并，每格只留最近距离。一帧（一圈）带帧序号、
 * 起始时刻和圈周期，供上层做时间对齐。
 */
namespace lidar {

constexpr int SECTORS = 180;        // 每圈 180 个 2° 分区
constexpr int FRAME_BUFFER = 8;     // 环形保留最近 8 圈
constexpr int PAGE_ROWS = 0;        // 保留字段，点云按整帧读取

constexpr uint16_t MIN_DIST_MM = 50;     // C1 量程 0.05~12 m
constexpr uint16_t MAX_DIST_MM = 12000;

struct Status {
  uint8_t state;        // 0 空闲 1 扫描中 2 数据流中断
  uint32_t frames;      // 已完成帧数
  uint32_t points;      // 累计有效点
  uint32_t errors;      // 同步/校验错误数
  uint16_t hzX100;      // 扫描频率 ×100
  uint16_t lastDistMm;  // 最新一点距离
  uint16_t minDistMm;   // 最近一圈的最近距离，0 表示本圈无有效点
  uint32_t ageMs;       // 距最新一点的毫秒数，无数据时为 0xFFFFFFFF
};

struct Frame {
  uint32_t seq;                    // 帧序号，从 1 开始单调递增；16 位会在约 109 分钟后回绕
  uint32_t startMs;                // 本圈起始时刻，MCU 上电毫秒
  uint16_t periodMs;               // 与上一圈起始的间隔；首帧为 0
  uint16_t valid;                  // 本圈有回波的分区数，0~180
  uint16_t distMm[SECTORS];        // 每个 2° 分区的最近距离，0 = 无回波
};

void begin();
bool start();
bool stop();
void snapshot(Status &out);

/**
 * @description         : 读取一帧。seq=0 取最新一帧，否则按序号取
 * @param seq           : 帧序号，0 表示最新
 * @param out           : 输出帧
 * @return              : 命中返回 true，帧还没生成或被覆盖返回 false
 */
bool readFrame(uint32_t seq, Frame &out);

/** 已完成帧总数，也就是最新帧序号；0 表示还没有完整的一圈。 */
uint32_t latestSeq();

/** 读取最近收到的原始字节，用于排查串口层问题。 */
int readRaw(uint8_t *out, int maxCount);

} // namespace lidar
