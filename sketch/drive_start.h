#pragma once

#include <algorithm>
#include <cstdint>

// 每次从零起步或换向时最多补偿一次。移动后失去反馈由保护处理。
class DriveStartAssist {
 public:
  // 保持时长从 120 ms 提到 250 ms：这套电机从给电到真正起转要 150 ms 以上，
  // 120 ms 的补偿在轮子开始动之前就撤掉了。轮子一旦真的转了（累计 3 个计数），
  // observe() 会立刻转入淡出，所以不会因为延长时间而"窜车"。
  static constexpr uint32_t HOLD_MS = 250;
  static constexpr uint32_t FADE_MS = 120;
  static constexpr int32_t CONFIRM_COUNTS = 3;

  void reset(bool arm = false) {
    phase = arm ? Phase::Pending : Phase::Off;
    progress = 0;
    phaseAt = 0;
  }

  void observe(int32_t delta, uint32_t now) {
    if (phase != Phase::Pending && phase != Phase::Hold) return;
    progress = int32_t(std::clamp<int64_t>(int64_t(progress) + delta, 0, CONFIRM_COUNTS));
    if (progress < CONFIRM_COUNTS) return;
    if (phase == Phase::Pending) {
      reset();
    } else if (now - phaseAt < HOLD_MS) {
      phase = Phase::Fade;
      phaseAt = now;
    }
  }

  int apply(int duty, int threshold, int breakawayDuty, int maxBoost, bool eligible, uint32_t now) {
    if (phase == Phase::Pending) {
      // 等正常输出到达检测门槛，补偿本身不提前消耗无进展保护窗口。
      if (!eligible || duty < threshold || maxBoost == 0) return duty;
      phase = Phase::Hold;
      phaseAt = now;
    }
    if (phase == Phase::Hold && now - phaseAt >= HOLD_MS) {
      phase = Phase::Fade;
      phaseAt += HOLD_MS;
    }
    if (phase == Phase::Fade && now - phaseAt >= FADE_MS) reset();
    if (!eligible || phase == Phase::Off) return duty;

    const int boost = std::clamp(breakawayDuty - duty, 0, maxBoost);
    if (phase == Phase::Hold) return duty + boost;
    const int remaining = int(FADE_MS - (now - phaseAt));
    return duty + boost * remaining / int(FADE_MS);
  }

 private:
  enum class Phase { Off, Pending, Hold, Fade };
  Phase phase = Phase::Off;
  int32_t progress = 0;
  uint32_t phaseAt = 0;
};

// 只有驱动输出达到实际起转区间时才给无进展计时；低占空比的缓慢起步不计时。
class EncoderProgressGuard {
 public:
  static constexpr uint32_t TIMEOUT_MS = 2000;

  void reset() { armed = false; }

  bool expired(bool dutyAboveThreshold, bool advancing, uint32_t now) {
    if (!dutyAboveThreshold) {
      reset();
      return false;
    }
    if (!armed || advancing) {
      armed = true;
      lastProgress = now;
    }
    return now - lastProgress >= TIMEOUT_MS;
  }

 private:
  bool armed = false;
  uint32_t lastProgress = 0;
};
