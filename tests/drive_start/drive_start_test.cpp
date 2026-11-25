#include <cassert>
#include <cstdint>
#include <cstdio>
#include <limits>

#include "drive_start.h"

int main() {
  DriveStartAssist right;
  DriveStartAssist left;
  right.reset(true);
  left.reset(true);

  // 达到原检测门槛后才开始补偿。两侧分别使用自己的标定值和进展。
  assert(right.apply(45, 46, 63, 30, true, 0) == 45);
  assert(right.apply(46, 46, 63, 30, true, 10) == 63);
  assert(left.apply(49, 51, 65, 30, true, 10) == 49);
  assert(left.apply(51, 51, 65, 30, true, 20) == 65);
  assert(right.apply(29, 46, 63, 30, true, 20) == 59);
  right.observe(1, 30);
  right.observe(-1, 40);
  assert(right.apply(50, 46, 63, 30, true, 50) == 63);
  right.observe(3, 60);
  assert(right.apply(50, 46, 63, 30, true, 100) == 58);
  assert(right.apply(50, 46, 63, 30, true, 140) == 54);
  assert(right.apply(50, 46, 63, 30, true, 180) == 50);
  // 运行中再丢失反馈，原方向指令也不会重新起转补偿。
  assert(right.apply(50, 46, 63, 30, true, 830) == 50);
  assert(left.apply(51, 51, 65, 30, true, 139) == 65);
  assert(left.apply(51, 51, 65, 30, true, 330) == 58);
  assert(left.apply(51, 51, 65, 30, true, 390) == 51);

  // 从零起步可重新使能。输出保持零或换向惯性尚在时，遵守原始输出。
  right.reset(true);
  assert(right.apply(50, 48, 63, 30, false, 900) == 50);
  assert(right.apply(50, 48, 63, 0, true, 910) == 50);
  assert(right.apply(50, 48, 63, 30, true, 920) == 63);
  assert(right.apply(70, 48, 63, 30, true, 930) == 70);
  right.reset();
  assert(right.apply(0, 48, 63, 30, false, 940) == 0);
  right.reset(true);
  right.observe(4, 950);
  assert(right.apply(50, 48, 63, 30, true, 960) == 50);

  EncoderProgressGuard guard;
  constexpr auto timeout = EncoderProgressGuard::TIMEOUT_MS;
  // 低于实测起转门槛的持续慢速命令不计时。
  assert(!guard.expired(false, false, 0));
  assert(!guard.expired(false, false, 4000));
  // 已给足力却无编码器进展，保持高占空比到窗口才停车。
  assert(!guard.expired(true, false, 4010));
  assert(!guard.expired(true, false, 4010 + timeout - 1));
  assert(guard.expired(true, false, 4010 + timeout));
  // 降低占空比或恢复有效进展会结束当前计时。
  assert(!guard.expired(false, false, 7000));
  assert(!guard.expired(true, false, 7100));
  assert(!guard.expired(true, true, 8100));
  assert(!guard.expired(true, false, 8100 + timeout - 1));
  assert(guard.expired(true, false, 8100 + timeout));
  guard.reset();
  const uint32_t beforeWrap = std::numeric_limits<uint32_t>::max() - 400;
  assert(!guard.expired(true, false, beforeWrap));
  assert(!guard.expired(true, false, beforeWrap + timeout - 1));
  assert(guard.expired(true, false, beforeWrap + timeout));

  std::puts("驱动起转与持续高输出无进展保护回归通过");
}
