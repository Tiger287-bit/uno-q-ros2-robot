"""示例 2：用 Ros2Publisher 发 /cmd_vel_web，验证网页键盘通路。

这正是 python/main.py 里 on_cmd_vel 的做法：浏览器把速度发给 App 的
socket.io 通道，App 侧再经 rosbridge 转进 /cmd_vel_web，由 cmd_vel_mux
仲裁后进入 /cmd_vel。浏览器全程不直连 rosbridge。

不接浏览器也能跑本示例，用来单独检查仲裁节点：预期 ros2 容器日志出现
「速度来源: web」，且 /cmd_vel 上能看到对应速度。

关于 dropped：Ros2Publisher 是单槽位信箱 + 独立发送线程，新帧会覆盖尚未发出的
旧帧。本示例用 time.sleep 打 20 Hz，在慢板上睡眠抖动常常超过实际发送周期，
于是 stats()["dropped"] 会稳定大于 0 —— 这是设计内的行为，不是链路故障：
速度指令是连续流，被覆盖的永远是"过期"的帧，下游只关心最新的那个。
真正该担心的是 sent 不增长（链路不通）。另外示例只看行为，实际停车由
cmd_vel_mux 的静默超时（0.2 s）兜底，不依赖这里发出最后一帧 0。

运行位置是 App 主容器（依赖 requirements.txt 里的 websockets）。
"""

import time

from ros2_bridge import Ros2Publisher

SEND_HZ = 20.0


def main():
    """依次发布直行、左转、停车三组指令，每组 2 秒。"""
    web = Ros2Publisher(topic="/cmd_vel_web")
    web.start()
    try:
        # 建链需要一点时间，先等一下再发，否则头几帧会被丢弃
        time.sleep(1.0)
        for linear, angular, note in (
            (0.060, 0.0, "直行 6 cm/s"),
            (0.000, 0.8, "原地左转"),
            (0.000, 0.0, "停车"),
        ):
            print("发送:", note)
            end = time.monotonic() + 2.0
            while time.monotonic() < end:
                web.publish({
                    "linear": {"x": linear, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": angular},
                })
                time.sleep(1.0 / SEND_HZ)
    finally:
        web.stop()

    print("结束；停车状态由 cmd_vel_mux 持续维持")
    # dropped 大于 0 属正常（单槽位信箱覆盖过期帧），只关注 connected 与 sent
    print("链路统计:", web.stats())


if __name__ == "__main__":
    main()
