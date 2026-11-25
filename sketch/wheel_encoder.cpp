#include "wheel_encoder.h"

#include <Arduino.h>
#include <zephyr/irq.h>

namespace {

constexpr int encoderPins[][2] = {{A0, 2}, {A1, A2}};  // M1、M2 的 A、B 相输入。
constexpr int8_t quadrature[] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};  // 旧相位和新相位对应的计数增量。

struct Encoder {
  volatile uint32_t count = 0;  // 正交边沿原始累计计数，按 32 位无符号值回绕。
  volatile uint32_t invalid = 0;  // 去抖确认后仍两相同跳（漏掉一个中间相位）的次数。
  volatile uint8_t previous = 0;  // 上一次的两位相位编码，A 相在高位。
};

Encoder encoders[2];  // M1 右轮、M2 左轮的中断累计状态。

/**
 * @description         : 读取两相编码器当前电平
 * @param index         : 电机索引，0 为 M1 右轮，1 为 M2 左轮
 * @return              : A 相在高位的两位相位编码
 */
uint8_t readEncoder(int index) {
  return (digitalRead(encoderPins[index][0]) << 1) | digitalRead(encoderPins[index][1]);
}

/**
 * @description         : 读到两相一致为止，避开"两次 digitalRead 之间压着一个边沿"
 * @param index         : 电机索引，0 为 M1 右轮，1 为 M2 左轮
 * @return              : A 相在高位的两位相位编码
 */
uint8_t readStablePhase(int index) {
  uint8_t value = readEncoder(index);
  // 两相各挂一个 CHANGE 中断，中断里连读两次：若在这两次读之间恰好有一个边沿，
  // 会拿到 A、B 同时跳变的非法组合，本次计数白丢、previous 还被写成非法相位，
  // 下一个边沿再按非法相位解码就可能数反方向。重读到一致再走解码，最多 3 次
  // （边沿间隔在最高转速 3200 cps 下约 300 µs，两次读相隔不到 1 µs）。
  for (int attempt = 0; attempt < 3; ++attempt) {
    const uint8_t again = readEncoder(index);
    if (again == value) break;
    value = again;
  }
  return value;
}

/**
 * @description         : 解码正交编码器的边沿并累加计数
 * @param index         : 电机索引，0 为 M1 右轮，1 为 M2 左轮
 * @return              : void
 */
void encoderEdge(int index) {
  Encoder &encoder = encoders[index];
  const uint8_t value = readStablePhase(index);
  const uint8_t previous = encoder.previous;
  // 确认后仍为"两相同跳"：真的漏了一个中间相位（中断被推迟），这一拍不计增量，
  // 但把 previous 推进到当前相位，下一拍才能继续正常解码。
  if ((value ^ previous) == 3) ++encoder.invalid;
  encoder.count += quadrature[(previous << 2) | value];
  encoder.previous = value;
}

/**
 * @description         : 处理 M1 右轮编码器边沿
 * @return              : void
 */
void encoder1Edge() { encoderEdge(0); }

/**
 * @description         : 处理 M2 左轮编码器边沿
 * @return              : void
 */
void encoder2Edge() { encoderEdge(1); }

}  // namespace

void encoderBegin() {
  for (int i = 0; i < 2; ++i) {
    for (int pin : encoderPins[i]) pinMode(pin, INPUT_PULLUP);
    encoders[i].previous = readEncoder(i);
  }
  attachInterrupt(digitalPinToInterrupt(encoderPins[0][0]), encoder1Edge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPins[0][1]), encoder1Edge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPins[1][0]), encoder2Edge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPins[1][1]), encoder2Edge, CHANGE);
}

void readCounters(uint32_t (&count)[2], uint32_t (&invalid)[2]) {
  const unsigned int key = irq_lock();  // 保存中断状态，确保两轮快照取自同一个临界区。
  for (int i = 0; i < 2; ++i) {
    count[i] = encoders[i].count;
    invalid[i] = encoders[i].invalid;
  }
  irq_unlock(key);
}
