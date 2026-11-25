#include "lidar.h"

#include <Arduino.h>
#include <zephyr/kernel.h>
#include <cstring>

namespace {

constexpr uint32_t BAUD = 460800;
constexpr size_t STACK_BYTES = 3072;
// 必须低于底盘控制线程（CONTROL_PRIORITY=4），否则 10 ms 控制拍会被挤掉。
constexpr int THREAD_PRIORITY = 5;
// 25 KB/s 下 1 ms 只积 25 字节，UART FIFO 够放；每个周期都会把缓冲读干净。
constexpr uint32_t POLL_MS = 1;
constexpr uint32_t RESEND_MS = 500;
constexpr uint32_t STALE_MS = 1000;
constexpr uint32_t MOTOR_PWM = 1023;   // SET_MOTOR_PWM 占空比，1023 = 全速
constexpr uint32_t ARM_MS = 600;       // 等电机起转、丢掉电机命令的响应

enum class ParseState : uint8_t { Idle, Descriptor, Streaming };

K_THREAD_STACK_DEFINE(parserStack, STACK_BYTES);
k_thread parserThread;
k_mutex frameMutex;

ParseState parseState = ParseState::Idle;
volatile uint8_t armPhase = 0;         // 0 空闲 1 等电机起转 2 扫描中
uint32_t armAt = 0;

uint8_t descBuf[7];                    // A5 5A + size_q30(4) + type（7 字节，不含 checksum）
int descLen = 0;
uint8_t nodeBuf[5];
int nodeLen = 0;

uint8_t rawRing[256];
uint32_t rawHead = 0;

uint8_t state = 0;
uint32_t pointCount = 0;
uint32_t errorCount = 0;
uint32_t lastPointMs = 0;
uint16_t lastDist = 0;
uint16_t minDist = lidar::MAX_DIST_MM;
uint16_t hzX100 = 0;

// 当前正在累积的一圈
uint16_t sectorDist[lidar::SECTORS];
uint16_t sectorValid = 0;
uint32_t frameStartMs = 0;
uint32_t prevStartMs = 0;

// 已完成的一圈
lidar::Frame frames[lidar::FRAME_BUFFER];
uint32_t frameHead = 0;                // 下一个写入位置
uint32_t frameTotal = 0;               // 已完成帧数

uint32_t epoch = 0;

/**
 * @description         : 把当前累积的一圈封成帧并推入环形缓冲
 * @param now           : 当前毫秒
 * @note                : 只由解析线程调用；写 frames 时持 frameMutex。
 */
void commitFrame(uint32_t now) {
  if (frameStartMs == 0) return;       // 第一圈还没开始，没有可封的内容

  lidar::Frame snapshot = {};
  snapshot.seq = frameTotal + 1;
  snapshot.startMs = frameStartMs;
  snapshot.periodMs = prevStartMs ? uint16_t(frameStartMs - prevStartMs) : 0;
  snapshot.valid = sectorValid;
  for (int i = 0; i < lidar::SECTORS; ++i) snapshot.distMm[i] = sectorDist[i];

  k_mutex_lock(&frameMutex, K_FOREVER);
  frames[frameHead] = snapshot;
  frameHead = (frameHead + 1) % lidar::FRAME_BUFFER;
  ++frameTotal;
  if (snapshot.periodMs >= 20 && snapshot.periodMs <= 500) {
    hzX100 = uint16_t(100000u / snapshot.periodMs);   // (1000/period) × 100
  }
  k_mutex_unlock(&frameMutex);

  prevStartMs = frameStartMs;
  std::memset(sectorDist, 0, sizeof(sectorDist));
  sectorValid = 0;
  minDist = lidar::MAX_DIST_MM;
  (void)now;
}

/**
 * @description         : 解析一个 5 字节测量节点
 * @param buf           : 连续 5 字节
 * @param now           : 当前毫秒
 * @return              : void
 */
void parseNode(const uint8_t *buf, uint32_t now) {
  const uint8_t s = buf[0] & 0x01;              // 新一圈起始标志
  const uint8_t sInv = (buf[0] >> 1) & 0x01;
  if (s == sInv) {                              // 两者必须相反，否则失步
    ++errorCount;
    return;
  }
  // 角度字段的 bit0 是 check bit，必须为 1（SDK: RPLIDAR_RESP_MEASUREMENT_CHECKBIT）
  if ((buf[1] & 0x01) == 0) {
    ++errorCount;
    return;
  }

  const uint16_t raw16 = uint16_t((uint16_t(buf[2]) << 8) | buf[1]);
  const uint16_t angleQ6 = uint16_t(raw16 >> 1);      // 15 位，360° = 23040
  const uint16_t distQ2 = uint16_t((uint16_t(buf[4]) << 8) | buf[3]);
  const uint16_t distMm = uint16_t(distQ2 / 4u);

  if (s) {                                      // 本节点属于新的一圈
    commitFrame(now);
    frameStartMs = now;
  }

  // C1 对空处返回 0；超出量程的也丢掉，免得污染分区。
  if (distMm < lidar::MIN_DIST_MM || distMm > lidar::MAX_DIST_MM) return;

  // 2° 一格：angleQ6 是 ×64，2° 对应 128。
  int sector = int(angleQ6 / 128);
  if (sector >= lidar::SECTORS) sector = lidar::SECTORS - 1;
  if (sectorDist[sector] == 0) {
    sectorDist[sector] = distMm;
    ++sectorValid;
  } else if (distMm < sectorDist[sector]) {
    sectorDist[sector] = distMm;                // 同一格取最近
  }

  ++pointCount;
  lastDist = distMm;
  lastPointMs = now;
  if (distMm < minDist) minDist = distMm;
  state = 1;                                    // 有数据即视为数据流正常
}

/** 吃一个字节，推进解析状态机。 */
void feed(uint8_t byte, uint32_t now) {
  switch (parseState) {
    case ParseState::Idle:
      if (byte == 0xA5) { descBuf[0] = byte; descLen = 1; parseState = ParseState::Descriptor; }
      break;

    case ParseState::Descriptor:
      // 响应头 7 字节：A5 5A + size_q30(4) + type。C1 不带 checksum，
      // 多读一个字节会让后面整条 5 字节节点流错位一格。
      descBuf[descLen++] = byte;
      if (descLen == 2 && descBuf[1] != 0x5A) {
        descLen = 0;
        parseState = ParseState::Idle;
      } else if (descLen >= 7) {
        descLen = 0;
        parseState = ParseState::Streaming;
        nodeLen = 0;
      }
      break;

    case ParseState::Streaming:
      // 节点首字节 bit0/bit1 必须相反；相同说明不是节点边界，丢弃并重新对齐。
      if (nodeLen == 0) {
        const uint8_t pair = uint8_t(byte & 0x03);
        if (pair == 0x00 || pair == 0x03) return;
      }
      nodeBuf[nodeLen++] = byte;
      if (nodeLen == 5) {
        parseNode(nodeBuf, now);
        nodeLen = 0;
      }
      break;
  }
}

void sendScan() {
  const uint8_t cmd[2] = {0xA5, 0x20};          // SCAN；STOP 才是 0x25
  Serial1.write(cmd, sizeof(cmd));
}

/** SET_MOTOR_PWM：0xA5 0xF0 + 16 位占空比，电机由雷达内置驱动，不占外部引脚。 */
void sendMotorPwm(uint16_t pwm) {
  const uint8_t cmd[4] = {0xA5, 0xF0, uint8_t(pwm & 0xFF), uint8_t(pwm >> 8)};
  Serial1.write(cmd, sizeof(cmd));
}

void parserLoop(void *, void *, void *) {
  uint32_t lastCmdMs = 0;
  while (true) {
    const uint32_t now = millis();

    if (armPhase == 1 && now - armAt > ARM_MS) {
      parseState = ParseState::Idle;
      descLen = 0;
      nodeLen = 0;
      armPhase = 2;
      lastCmdMs = now;
      sendScan();
    }

    if (armPhase == 2 && parseState != ParseState::Streaming && now - lastCmdMs > RESEND_MS) {
      sendScan();
      lastCmdMs = now;
    }

    while (Serial1.available() > 0) {
      const uint8_t byte = uint8_t(Serial1.read());
      rawRing[rawHead] = byte;
      rawHead = (rawHead + 1) % sizeof(rawRing);
      if (armPhase == 2) feed(byte, now);
    }

    if (state == 1 && lastPointMs != 0 && now - lastPointMs > STALE_MS) {
      state = 2;
    }
    k_msleep(POLL_MS);
  }
}

} // namespace

namespace lidar {

void begin() {
  Serial1.begin(BAUD);
  k_mutex_init(&frameMutex);
  k_thread_create(&parserThread, parserStack, STACK_BYTES, parserLoop,
                  nullptr, nullptr, nullptr, THREAD_PRIORITY, 0, K_NO_WAIT);
}

bool start() {
  // 先关闸再重置：解析线程每 1 ms 跑一轮，直接改 frameStartMs/sectorDist
  // 会与它的读写撞在一起。armPhase 转成 0 之后它不再 touch 这些状态。
  armPhase = 0;
  k_msleep(2);
  ++epoch;
  state = 1;
  armPhase = 1;
  armAt = millis();
  parseState = ParseState::Idle;
  descLen = 0;
  nodeLen = 0;
  std::memset(sectorDist, 0, sizeof(sectorDist));
  sectorValid = 0;
  frameStartMs = 0;
  prevStartMs = 0;
  pointCount = 0;
  errorCount = 0;
  lastPointMs = 0;
  minDist = MAX_DIST_MM;
  hzX100 = 0;
  k_mutex_lock(&frameMutex, K_FOREVER);
  frameHead = 0;
  frameTotal = 0;
  std::memset(frames, 0, sizeof(frames));
  k_mutex_unlock(&frameMutex);

  sendMotorPwm(MOTOR_PWM);     // 电机先转，SCAN 由线程在 ARM_MS 之后发出
  return true;
}

bool stop() {
  armPhase = 0;          // 同样先关闸，再拆解析状态
  state = 0;
  k_msleep(2);
  parseState = ParseState::Idle;
  const uint8_t cmd[2] = {0xA5, 0x25};   // STOP
  Serial1.write(cmd, sizeof(cmd));
  sendMotorPwm(0);
  return true;
}

void snapshot(Status &out) {
  out.state = state;
  out.points = pointCount;
  out.errors = errorCount;
  out.hzX100 = hzX100;
  out.lastDistMm = lastDist;
  out.minDistMm = (minDist == MAX_DIST_MM) ? 0 : minDist;
  out.ageMs = lastPointMs == 0 ? 0xFFFFFFFFu : (millis() - lastPointMs);
  k_mutex_lock(&frameMutex, K_FOREVER);
  out.frames = frameTotal;
  k_mutex_unlock(&frameMutex);
}

bool readFrame(uint32_t seq, Frame &out) {
  k_mutex_lock(&frameMutex, K_FOREVER);
  bool found = false;
  if (frameTotal > 0) {
    uint32_t target;
    if (seq == 0) {
      target = frameTotal;                       // 最新一帧
    } else {
      target = seq;
    }
    if (target <= frameTotal && frameTotal - target < uint32_t(FRAME_BUFFER)) {
      const uint32_t index = (target - 1) % FRAME_BUFFER;
      out = frames[index];
      found = true;
    }
  }
  k_mutex_unlock(&frameMutex);
  return found;
}

uint32_t latestSeq() {
  k_mutex_lock(&frameMutex, K_FOREVER);
  const uint32_t seq = frameTotal;
  k_mutex_unlock(&frameMutex);
  return seq;
}

int readRaw(uint8_t *out, int maxCount) {
  if (out == nullptr || maxCount <= 0) return 0;
  if (maxCount > int(sizeof(rawRing))) maxCount = int(sizeof(rawRing));
  for (int i = 0; i < maxCount; ++i) {
    const uint32_t index = (rawHead + sizeof(rawRing) - maxCount + i) % sizeof(rawRing);
    out[i] = rawRing[index];
  }
  return maxCount;
}

} // namespace lidar
