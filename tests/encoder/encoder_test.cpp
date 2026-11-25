/* 正交解码的主机测试：直接 include 固件那份 wheel_encoder.cpp，不打桩逻辑。
   只替换 Arduino 的 digitalRead/pinMode/attachInterrupt 与 Zephyr 的 irq_lock。

   编译（板端没有 g++，借 ros2 容器）：见 tests/run.sh
*/
#include <cstdint>
#include <cstdio>
#include <deque>
#include <utility>

int g_pinState[64] = {0};
int (*g_readHook)(int) = nullptr;

static std::deque<int> g_queue;  // 逐个读取返回的预设电平，用于注入撕裂样本。

static int queueHook(int pin) {
  if (!g_queue.empty()) {
    const int v = g_queue.front();
    g_queue.pop_front();
    return v;
  }
  return g_pinState[pin];
}

#include "wheel_encoder.cpp"  // 与固件同一份源码，同一 TU 才能碰匿名命名空间

static int failures = 0;

static void expect(bool ok, const char *label) {
  printf("%s  %s\n", ok ? "PASS" : "FAIL", label);
  if (!ok) ++failures;
}

// M1 的两个引脚：A 相 encoderPins[0][0]、B 相 encoderPins[0][1]。
static void setPhase(int phase) {
  g_pinState[encoderPins[0][0]] = (phase >> 1) & 1;
  g_pinState[encoderPins[0][1]] = phase & 1;
}

static uint32_t baseCount = 0, baseInvalid = 0;

static void snapshot(uint32_t &count, uint32_t &invalid) {
  uint32_t counts[2], invalids[2];
  readCounters(counts, invalids);
  count = counts[0];
  invalid = invalids[0];
}

/** 计数器是累计量，逐个用例比较增量，避免前一个用例的 invalid 影响后一个。 */
static void expectDelta(int32_t wantCount, int32_t wantInvalid, const char *label) {
  uint32_t count, invalid;
  snapshot(count, invalid);
  const int32_t dc = int32_t(count - baseCount), di = int32_t(invalid - baseInvalid);
  expect(dc == wantCount && di == wantInvalid, label);
  baseCount = count;
  baseInvalid = invalid;
}

int main() {
  // 1. 解码表本身：16 项，对角（两相同跳）必须为 0，相邻相位必须 ±1。
  bool diagonalZero = true, neighboursValid = true;
  for (int previous = 0; previous < 4; ++previous) {
    for (int value = 0; value < 4; ++value) {
      const int8_t delta = quadrature[(previous << 2) | value];
      if (((previous ^ value) == 3) && delta != 0) diagonalZero = false;
      if (((previous ^ value) != 3) && delta != 0) {
        const bool adjacent = ((previous ^ value) == 1) || ((previous ^ value) == 2);
        if (!adjacent || (delta != 1 && delta != -1)) neighboursValid = false;
      }
    }
  }
  expect(diagonalZero, "表：对角项全为 0（漏相位不瞎计数）");
  expect(neighboursValid, "表：相邻相位只出 ±1");

  // 2. 正转：A 相先变 10 -> 11 -> 01 -> 00，四步应 +4。
  setPhase(0);
  encoderBegin();
  const int forward[] = {2, 3, 1, 0};
  for (int phase : forward) {
    setPhase(phase);
    encoder1Edge();
  }
  expectDelta(4, 0, "正转 4 个边沿 = +4，invalid 仍为 0");

  // 3. 反转：01 -> 11 -> 10 -> 00，四步应回到 0。
  const int reverse[] = {1, 3, 2, 0};
  for (int phase : reverse) {
    setPhase(phase);
    encoder1Edge();
  }
  expectDelta(-4, 0, "反转 4 个边沿 = -4");

  // 4. 漏相位：00 直接跳到 11，计一次 invalid，计数不变。
  setPhase(0);
  encoderBegin();
  setPhase(3);
  encoder1Edge();
  expectDelta(0, 1, "两相同跳：invalid+1，计数不变");

  // 5. 撕裂样本：中断里两次读之间压着一个边沿，读到 (A=1,B=1) 再读回到 (0,0)。
  //    旧实现直接采纳 11，既记 invalid 又把 previous 写成非法相位，下一个边沿
  //    再按非法相位解码就可能数反方向；现在重读到一致，这一次边沿等于没发生。
  setPhase(0);
  encoderBegin();
  g_queue = {1, 1, 0, 0};  // 第 1、2 次读=11，第 3、4 次读=00，之后回落到真实电平
  g_readHook = queueHook;
  encoder1Edge();
  g_readHook = nullptr;
  expectDelta(0, 0, "撕裂样本被丢弃：不计数、不记 invalid");

  // 6. 撕裂样本之后 previous 未被污染：真正的边沿仍然正常 +1。
  setPhase(2);
  encoder1Edge();
  expectDelta(1, 0, "撕裂样本后仍能正常解码下一个边沿");

  printf(failures ? "\n%d 项失败\n" : "\n全部通过\n", failures);
  return failures ? 1 : 0;
}
