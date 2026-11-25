#include "heading_imu.h"

#include <Arduino.h>
#include <Wire.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <algorithm>
#include <cmath>

namespace {
constexpr uint8_t ADDRESS = 0x6A;               // LSM6DS3TR-C 的七位 I²C 地址。
constexpr float GYRO_DPS_PER_LSB = 0.00875f;    // 陀螺每 LSB 对应的度每秒。
constexpr float ACC_G_PER_LSB = 0.000061f;      // 加速度计每 LSB 对应的 g 值。
constexpr float TEMPERATURE_OFFSET_C = 25.0f;   // 温度输出 0 码对应的摄氏度。
constexpr uint32_t MAX_INTEGRATE_MS = 40;      // 单拍积分最多计入的时间跨度，约 4 个采样周期。
constexpr uint32_t WINDOW_BREAK_MS = 150;      // 校准静止窗口的连续采样中断阈值，约 15 个周期。
constexpr uint32_t RESYNC_MS = 500;            // 断更超过此值视为传感器掉线：丢弃航向重新校准。
                                               // 读者也用它判 Stale，所以 ≤500 ms 的断更仍报
                                               // Ready，控制器继续按旧航向做保向修正。
constexpr uint32_t CALIBRATION_SAMPLES = 80;   // 一次零偏校准所需的连续静止样本数。
constexpr uint32_t SENSOR_WARMUP_MS = 200;      // 配置成功后的预热时间，单位为毫秒。

struct k_mutex sampleMutex;              // 保护航向样本和发布时刻的成对读写。
struct k_thread imuThread;               // Zephyr 的 IMU 采样线程控制块。
k_thread_stack_t *imuStack = nullptr;     // 采样线程栈，空指针表示尚未分配成功。
atomic_t calibrationRequested;           // 跨线程校准请求标志，1 表示待处理。
ImuHeadingSample published;               // 供轮速线程读取的最新样本快照。
uint32_t publishedAt = 0;                 // 样本发布时的 MCU 毫秒计数，0 表示暂无时刻。

struct Calibration {
  uint32_t count = 0;     // 窗口内连续通过静止检测的样本数。
  float gyroSum[3] = {};  // 按 X、Y、Z 累计角速度，用于计算各轴零偏。
  float accelSum[3] = {}; // 按 X、Y、Z 累计加速度，用于确定竖直轴。
  float gyroMin[3] = {};  // 窗口内各轴最小角速度，单位为度每秒。
  float gyroMax[3] = {};  // 窗口内各轴最大角速度，单位为度每秒。
};

/**
 * @description         : 按指定起始地址连续读取 IMU 寄存器
 * @param reg           : 起始寄存器地址
 * @param bytes         : 输出缓冲区
 * @param size          : 读取字节数
 * @return              : 传输完整时返回 true
 */
bool readRegisters(uint8_t reg, uint8_t *bytes, uint8_t size) {
  Wire.beginTransmission(ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(ADDRESS, size) != size) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (uint8_t i = 0; i < size; ++i) bytes[i] = uint8_t(Wire.read());
  return true;
}

/**
 * @description         : 写入一个 IMU 配置寄存器
 * @param reg           : 寄存器地址
 * @param value         : 写入的字节
 * @return              : IMU 应答时返回 true
 */
bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

/**
 * @description         : 配置并回读 104 Hz 陀螺仪和加速度计
 * @return              : 芯片标识与配置正确时返回 true
 */
bool configureImu() {
  uint8_t who = 0;      // WHO_AM_I 芯片识别值。
  uint8_t controls[3];  // CTRL1_XL、CTRL2_G、CTRL3_C 的回读值。
  // CTRL3_C 启用块数据更新和地址自动递增，另两项配置各传感器的采样率。
  return readRegisters(0x0F, &who, 1) && who == 0x6A &&
         writeRegister(0x12, 0x44) && writeRegister(0x10, 0x40) &&
         writeRegister(0x11, 0x40) && readRegisters(0x10, controls, 3) &&
         controls[0] == 0x40 && controls[1] == 0x40 && (controls[2] & 0x47) == 0x44;
}

/**
 * @description         : 发布最新样本，锁内只复制状态
 * @param sample        : 待发布样本
 * @param now           : 样本时刻，单位为毫秒
 * @return              : void
 */
void publish(const ImuHeadingSample &sample, uint32_t now) {
  k_mutex_lock(&sampleMutex, K_FOREVER);
  published = sample;
  publishedAt = now;
  k_mutex_unlock(&sampleMutex);
}

/**
 * @description         : 将 IMU 小端字节对转换成有符号读数
 * @param bytes         : 指向低字节的指针
 * @return              : 有符号的 16 位传感器读数
 */
int16_t readSigned(const uint8_t *bytes) {
  return int16_t(uint16_t(bytes[0]) | (uint16_t(bytes[1]) << 8));
}

/**
 * @description         : 累计符合静止条件的样本，估计零偏和竖直方向
 * @param window        : 当前静止样本窗口
 * @param gyro          : 三轴角速度，单位为度每秒
 * @param accel         : 三轴加速度，单位为 g
 * @param bias          : 输出三轴陀螺零偏，单位为度每秒
 * @param up            : 输出车体竖直方向在 IMU 坐标系的单位向量
 * @return              : 完成静止校准时返回 true
 */
bool calibrate(Calibration &window, const float (&gyro)[3], const float (&accel)[3],
               float (&bias)[3], float (&up)[3]) {
  // 静止样本的加速度向量模长应接近 1 g。
  const float magnitude = std::sqrt(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
  if (magnitude < 0.85f || magnitude > 1.15f ||
      std::max({std::fabs(gyro[0]), std::fabs(gyro[1]), std::fabs(gyro[2])}) > 6.0f) {
    window = {};
    return false;
  }
  for (int i = 0; i < 3; ++i) {
    window.gyroMin[i] = window.count ? std::min(window.gyroMin[i], gyro[i]) : gyro[i];
    window.gyroMax[i] = window.count ? std::max(window.gyroMax[i], gyro[i]) : gyro[i];
    if (window.gyroMax[i] - window.gyroMin[i] > 2.0f) {
      window = {};
      return false;
    }
    window.gyroSum[i] += gyro[i];
    window.accelSum[i] += accel[i];
  }
  if (++window.count < CALIBRATION_SAMPLES) return false;

  // 累计加速度的模长用于归一化，得到竖直方向单位向量。
  const float verticalNorm = std::sqrt(window.accelSum[0]*window.accelSum[0] +
      window.accelSum[1]*window.accelSum[1] + window.accelSum[2]*window.accelSum[2]);
  if (verticalNorm < float(window.count) * 0.85f) {
    window = {};
    return false;
  }
  for (int i = 0; i < 3; ++i) {
    bias[i] = window.gyroSum[i] / float(window.count);
    up[i] = window.accelSum[i] / verticalNorm;
  }
  return true;
}

/**
 * @description         : 在低优先级线程采样、估计零偏并积分相对航向
 * @return              : void
 */
void imuWorker(void *, void *, void *) {
  Wire.begin();
  Wire.setClock(100000);
  ImuHeadingSample sample;  // 当前采样与积分状态，由本线程更新。
  Calibration window;      // 静止校准窗口，检测到运动时清空。
  float bias[3] = {};       // 三轴陀螺零偏，单位为度每秒。
  float up[3] = {};         // 车体竖直方向在 IMU 坐标系中的单位向量。
  uint32_t configuredAt = 0;  // IMU 配置完成时的 MCU 毫秒计数。
  uint32_t lastAt = 0;        // 上次成功读到样本时的 MCU 毫秒计数。
  bool configured = false;   // 芯片配置已写入并通过回读。

  while (true) {
    if (!configured) {
      configured = configureImu();
      if (!configured) {
        publish({}, 0);
        k_msleep(500);
        continue;
      }
      configuredAt = millis();
      lastAt = configuredAt;
      atomic_set(&calibrationRequested, 0);
      window = {};
      sample = {};
      sample.state = ImuHeadingState::Calibrating;
      publish(sample, 0);
    }
    if (atomic_cas(&calibrationRequested, 1, 0)) {
      window = {};
      sample = {};
      sample.state = ImuHeadingState::Calibrating;
      publish(sample, 0);
    }
    uint8_t status = 0;  // STATUS_REG，第 0、1 位分别表示加速度计、陀螺仪数据就绪。
    uint8_t bytes[14];   // 连续寄存器原始字节，依次为温度、三轴角速度、三轴加速度。
    if (readRegisters(0x1E, &status, 1) && (status & 3) == 3 &&
        readRegisters(0x20, bytes, sizeof(bytes))) {
      const uint32_t now = millis();
      const uint32_t elapsed = now - lastAt;  // 与上一样本的间隔，单位为毫秒。
      lastAt = now;
      if (now - configuredAt >= SENSOR_WARMUP_MS) {
        // 缺口两级处理：静止窗口作废、真掉线才重新校准。原来 80 ms 一个常量
        // 同时充当"丢帧容忍"和"数据可信判据"：一次 I²C 抖动就把 yawDeg 清零
        // 退回校准，而行进中凑不满 80 个静止样本，只能停车静置才恢复 ——
        // 一次抖动换一次强制停车。
        if (elapsed > WINDOW_BREAK_MS) window = {};  // 缺口期间的静止状态没有被观测过
        if (elapsed > RESYNC_MS) {
          sample = {};
          sample.state = ImuHeadingState::Calibrating;
        }
        float gyro[3];   // 本次三轴角速度，单位为度每秒。
        float accel[3];  // 本次三轴加速度，单位为 g。
        for (int i = 0; i < 3; ++i) {
          gyro[i] = float(readSigned(bytes + 2 + 2*i)) * GYRO_DPS_PER_LSB;
          accel[i] = float(readSigned(bytes + 8 + 2*i)) * ACC_G_PER_LSB;
        }
        // 温度本来就和三轴数据在同一次突发读取里，之前读完就丢掉了。
        // LSM6DS3TR-C 的温度为 16 LSB/℃、0 码对应 25 ℃，直接换算即可，
        // 不额外占用 I²C 带宽，温漂分析（第 4 批）靠它才有观测量。
        sample.temperatureC = TEMPERATURE_OFFSET_C + float(readSigned(bytes)) / 16.0f;
        ++sample.samples;
        if (sample.state == ImuHeadingState::Calibrating) {
          if (calibrate(window, gyro, accel, bias, up)) {
            sample.state = ImuHeadingState::Ready;
            sample.biasDegPerS = bias[0]*up[0] + bias[1]*up[1] + bias[2]*up[2];
          }
        } else {
          sample.rateDegPerS = (gyro[0]-bias[0])*up[0] + (gyro[1]-bias[1])*up[1] +
                               (gyro[2]-bias[2])*up[2];
          // 缺口期间转过的角度测不到，所以单拍最多按 MAX_INTEGRATE_MS 计入，
          // 其余丢弃。用截断而不是"超过阈值就整拍不积"，是让误差随缺口线性增长：
          // 否则 149 ms 的缺口全信、151 ms 的全丢，同一个不确定量在阈值两侧差出
          // 一个台阶。lastAt 已更新为新样本时刻，下一拍恢复正常步长。
          const uint32_t step = std::min(elapsed, MAX_INTEGRATE_MS);
          sample.yawDeg += sample.rateDegPerS * (float(step) * 0.001f);
        }
        publish(sample, now);
      }
    } else if (millis() - lastAt > 300) {
      configured = false;
    }
    k_msleep(4);
  }
}
}  // namespace

void headingImuBegin() {
  k_mutex_init(&sampleMutex);
  imuStack = k_thread_stack_alloc(4096, 0);
  if (imuStack) {
    k_thread_create(&imuThread, imuStack, 4096, imuWorker, nullptr, nullptr, nullptr,
                    5, 0, K_NO_WAIT);
  }
}

bool headingImuCalibrate() {
  if (!imuStack) return false;
  atomic_set(&calibrationRequested, 1);
  return true;
}

ImuHeadingSample headingImuRead() {
  k_mutex_lock(&sampleMutex, K_FOREVER);
  ImuHeadingSample copy = published;  // 锁内复制同一时刻的状态和时间戳。
  const uint32_t at = publishedAt;    // 对应样本的 MCU 毫秒计数。
  k_mutex_unlock(&sampleMutex);
  if (at != 0) copy.ageMs = int32_t(millis() - at);
  if (copy.state == ImuHeadingState::Ready) {
    if (atomic_get(&calibrationRequested)) copy.state = ImuHeadingState::Calibrating;
    // 与采样线程的重置阈值同源：两个数不一样就会出现"线程已重来、读者还报
    // Ready"，或者反过来"读者报 Stale、线程仍在积分"。
    else if (copy.ageMs > int32_t(RESYNC_MS)) copy.state = ImuHeadingState::Stale;
  }
  return copy;
}
