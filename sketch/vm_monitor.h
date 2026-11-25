#pragma once

#include <cstdint>

struct VmMonitorSample {
  uint32_t sequence = 0;  // 成功采样序号，0 表示尚未采集。
  int32_t raw = -1;       // A4 最近 100 ms 的 12 位 ADC 均值，-1 表示不可用。
  int32_t minRaw = -1;    // 最近 100 ms 的有效 ADC 最小值。
  int32_t maxRaw = -1;    // 最近 100 ms 的有效 ADC 最大值。
  int32_t ageMs = -1;     // 最新读数距快照的时间，ms。
};

/**
 * @description         : 启动独立线程，从 A4 被动采集 VM 分压。
 * @return              : void
 */
void vmMonitorBegin();

/**
 * @description         : 读取同一次发布的 VM 原始值和最近窗口。
 * @return              : VM ADC 快照，失败或过期时 ADC 字段和样本年龄为 -1。
 */
VmMonitorSample vmMonitorRead();
