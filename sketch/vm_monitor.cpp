#include "vm_monitor.h"

#include <Arduino.h>
#include <zephyr/kernel.h>
#include <algorithm>

namespace {

constexpr int VM_ADC_PIN = A4;               // PC1，接电机 VM 供电轨的分压点。
constexpr int ADC_BITS = 12;                 // 向 Arduino ADC 接口请求的分辨率，bit。
constexpr int ADC_MAX = (1 << ADC_BITS) - 1; // 12 位 ADC 最大代码。
constexpr uint32_t SAMPLE_MS = 10;           // 两次采样间的休眠时间，ms。
constexpr uint32_t WINDOW_MS = 100;          // 最小值和最大值的滑动统计窗口，ms。
constexpr size_t HISTORY_SIZE = WINDOW_MS / SAMPLE_MS + 2; // 有效读数环形缓存容量。
constexpr size_t STACK_SIZE = 4096;          // Zephyr 采样线程栈大小，字节。
constexpr int PRIORITY = 6;                  // 低于控制线程和 IMU 线程的抢占优先级。

struct Reading {
  int32_t raw = -1; // 原始 ADC 读数，-1 表示此槽无有效样本。
  uint32_t at = 0;  // 采样结束时的 MCU 时间，ms。
};

struct k_thread worker;         // VM 采样线程控制块。
struct k_mutex sampleMutex;     // 只保护发布快照和时间戳。
VmMonitorSample published;      // 最近一次原子发布的采样结果。
uint32_t publishedAt = 0;       // 对应读数的 MCU 时间，ms。

/**
 * @description         : 在独立线程采集 A4 并发布窗口统计。
 * @return              : void
 */
void sampleWorker(void *, void *, void *) {
  analogReadResolution(ADC_BITS);
  Reading history[HISTORY_SIZE]; // 仅由采样线程访问。
  size_t next = 0;                // 环形缓存的下一写入位置。
  uint32_t sequence = 0;         // 成功采集的累计次数。
  while (true) {
    const int raw = analogRead(VM_ADC_PIN); // 同步 ADC 转换，发生在锁外。
    const uint32_t now = millis();        // 本次转换结束时间，ms。
    VmMonitorSample sample;
    // analogRead 在本核上返回 0~4095，下界永远成立；留着只是防转换驱动返回
    // 异常值（history 里的 -1 槽同样靠这条判据产生）。
    if (raw >= 0 && raw <= ADC_MAX) {
      ++sequence;
      history[next] = {raw, now};
      next = (next + 1) % HISTORY_SIZE;
      // raw 取 100 ms 窗口均值，而不是最新一次转换：单次读数在本板实测有
      // 2~3 LSB（3.8 mV/LSB）的低频抖动，屏上就在 10.88/10.89/10.91 V 之间
      // 来回跳。抖动是慢变（背靠背连采 4 次取均值没有改善，试过），只有跨
      // 100 ms 平均才压得住；电池读数滞后 50 ms 没有代价。
      int32_t sum = 0;
      int32_t count = 0;
      sample.minRaw = sample.maxRaw = raw;
      for (const Reading &reading : history) {
        if (reading.raw >= 0 && now - reading.at <= WINDOW_MS) {
          sum += reading.raw;
          ++count;
          sample.minRaw = std::min(sample.minRaw, reading.raw);
          sample.maxRaw = std::max(sample.maxRaw, reading.raw);
        }
      }
      // 极值仍取原始样本：电机启动时的压降是要看见的，不能被均值抹平。
      sample.raw = (sum + count / 2) / count;
    }
    sample.sequence = sequence;
    k_mutex_lock(&sampleMutex, K_FOREVER);
    published = sample;
    publishedAt = now;
    k_mutex_unlock(&sampleMutex);
    k_msleep(SAMPLE_MS);
  }
}

} // namespace

void vmMonitorBegin() {
  k_mutex_init(&sampleMutex);
  k_thread_stack_t *stack = k_thread_stack_alloc(STACK_SIZE, 0);
  if (stack) {
    k_thread_create(&worker, stack, STACK_SIZE, sampleWorker, nullptr, nullptr, nullptr,
                    PRIORITY, 0, K_NO_WAIT);
  }
}

VmMonitorSample vmMonitorRead() {
  k_mutex_lock(&sampleMutex, K_FOREVER);
  VmMonitorSample sample = published; // 锁内同时复制统计值和采样时刻。
  const uint32_t at = publishedAt;
  k_mutex_unlock(&sampleMutex);
  const uint32_t age = millis() - at;
  if (sample.raw >= 0 && age <= WINDOW_MS) {
    sample.ageMs = int32_t(age);
  } else {
    // 三个 ADC 字段和样本年龄必须一起失效。ageMs 现在恒为 -1（worker 每拍新建
    // 的 sample 就是默认值），但不能靠这条暗规：一旦有人在 worker 里填 ageMs，
    // 过期分支就会报出一个很小的年龄，读起来像"数据是新的，只是值无效"。
    sample.raw = sample.minRaw = sample.maxRaw = -1;
    sample.ageMs = -1;
  }
  return sample;
}
