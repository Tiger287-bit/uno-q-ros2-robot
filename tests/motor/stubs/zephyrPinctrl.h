// 主机测试用的 pinctrl 桩：引脚复用切换总是成功。
#pragma once

namespace zephyr {
namespace arduino {
inline int init_dev_apply_channel_pinctrl(const struct device *, int index) {
  (void)index;
  return 0;
}
}  // namespace arduino
}  // namespace zephyr
