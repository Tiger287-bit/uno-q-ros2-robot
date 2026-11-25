#include "lidar_bridge.h"

#include <Arduino_RouterBridge.h>
#include <vector>

#include "lidar.h"

namespace {

/**
 * @description         : 启动扫描：串口发电机命令，随后由解析线程发 SCAN
 * @return              : 恒为 true
 */
bool lidarStart() {
  return lidar::start();
}

/**
 * @description         : 停止扫描并停电机
 * @return              : 恒为 true
 */
bool lidarStop() {
  return lidar::stop();
}

/**
 * @description         : 雷达统计
 * @return              : 8 项整数
 * @note                : [state, frames, points, errors, hzX100, lastDist, minDist, ageMs]
 */
std::vector<int32_t> lidarStatus() {
  lidar::Status s = {};
  lidar::snapshot(s);
  return {int32_t(s.state), int32_t(s.frames), int32_t(s.points), int32_t(s.errors),
          int32_t(s.hzX100), int32_t(s.lastDistMm), int32_t(s.minDistMm),
          s.ageMs == 0xFFFFFFFFu ? -1 : int32_t(s.ageMs)};
}

/**
 * @description         : 读取一帧 180 个 2° 分区
 * @param seq           : 帧序号，0 表示最新一帧
 * @return              : 5 项元数据 + 180 个距离；帧不存在时返回空数组
 * @note                : [seq, startMs, periodMs, valid, sectors]
 */
std::vector<int32_t> lidarFrame(int32_t seq) {
  lidar::Frame frame = {};
  if (!lidar::readFrame(seq < 0 ? 0u : uint32_t(seq), frame)) return {};
  std::vector<int32_t> out;
  out.reserve(5 + lidar::SECTORS);
  out.insert(out.end(), {int32_t(frame.seq), int32_t(frame.startMs), int32_t(frame.periodMs),
                         int32_t(frame.valid), int32_t(lidar::SECTORS)});
  for (int i = 0; i < lidar::SECTORS; ++i) out.push_back(int32_t(frame.distMm[i]));
  return out;
}

/**
 * @description         : 分片读取一帧的若干个 2° 分区
 * @param seq           : 帧序号，0 表示最新
 * @param offset        : 首个分区下标 0~179
 * @param count         : 本次要几个分区
 * @return              : 8 项元数据 + count 个距离；帧不存在时返回空数组
 * @note                : [seq, startMs, periodMs, valid, sectors, offset, count, latestSeq, nowMs]
 *                        latestSeq 用来判断这一帧是否已经过期；nowMs 是本次读取时刻的
 *                        MCU uptime，上层用它把 startMs 换算成真实采集时刻。
 */
std::vector<int32_t> lidarSlice(int32_t seq, int32_t offset, int32_t count) {
  lidar::Frame frame = {};
  if (!lidar::readFrame(seq < 0 ? 0u : uint32_t(seq), frame)) return {};
  int start = offset < 0 ? 0 : int(offset);
  int length = count <= 0 ? 0 : int(count);
  if (start >= lidar::SECTORS) length = 0;
  // 先按剩余空间收敛再判断：count 来自 RPC，传 INT32_MAX 时 start+length 会溢出
  // 成负数，跳过截断直接越界写。
  if (length > lidar::SECTORS - start) length = lidar::SECTORS - start;

  std::vector<int32_t> out;
  out.reserve(9 + std::size_t(length));
  out.insert(out.end(), {int32_t(frame.seq), int32_t(frame.startMs), int32_t(frame.periodMs),
                         int32_t(frame.valid), int32_t(lidar::SECTORS), start, length,
                         int32_t(lidar::latestSeq()), int32_t(millis())});
  for (int i = 0; i < length; ++i) out.push_back(int32_t(frame.distMm[start + i]));
  return out;
}

/**
 * @description         : 最近收到的原始字节，排查串口层用
 * @param id            : 请求标识，原样回显
 * @return              : 首项回显 id，其后为字节值
 */
std::vector<int32_t> lidarRaw(int32_t id) {
  uint8_t buf[96];
  const int count = lidar::readRaw(buf, int(sizeof(buf)));
  std::vector<int32_t> out;
  out.reserve(1 + std::size_t(count));
  out.push_back(id);
  for (int i = 0; i < count; ++i) out.push_back(int32_t(buf[i]));
  return out;
}

/**
 * @description         : 自检：帧结构尺寸，便于核对 RAM 预算
 * @return              : [sizeof(Frame), SECTORS, FRAME_BUFFER]
 */
std::vector<int32_t> lidarDiag() {
  return {int32_t(sizeof(lidar::Frame)), lidar::SECTORS, lidar::FRAME_BUFFER};
}

} // namespace

void lidarBridgeRegister() {
  Bridge.provide_safe("lidar_start", lidarStart);
  Bridge.provide_safe("lidar_stop", lidarStop);
  Bridge.provide_safe("lidar_status", lidarStatus);
  Bridge.provide_safe("lidar_frame", lidarFrame);
  Bridge.provide_safe("lidar_slice", lidarSlice);
  Bridge.provide_safe("lidar_raw", lidarRaw);
  Bridge.provide_safe("lidar_diag", lidarDiag);
}
