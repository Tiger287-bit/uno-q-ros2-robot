"""NAV-006 雷达拼帧回归测试。

用 mock 替换 Bridge / publish_scan，直接调用 **生产代码**
`python/scan_pipeline.py` 里的 `fetch_scan_slice()`，不重新实现一套拼帧算法。

运行：
    cd <repo 根目录>
    python -m unittest discover -s tests/scan -v
或：
    python tests/scan/test_scan_pipeline.py
"""

import math
import os
import sys
import types
import unittest

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
PYDIR = os.path.join(REPO, "python")
if PYDIR not in sys.path:
    sys.path.insert(0, PYDIR)


# ---------------------------------------------------------------- 桩
class BridgeStub:
    """按预设序列返回分片负载；记录每次调用参数。"""

    def __init__(self):
        self.responses = []
        self.calls = []
        self.raise_on = set()

    def queue(self, *payloads):
        self.responses.extend(payloads)

    def call(self, name, seq=None, offset=None, count=None, timeout=None):
        self.calls.append((name, seq, offset, count))
        if name in self.raise_on:
            raise OSError("stub rpc failure")
        if not self.responses:
            raise AssertionError("Bridge 被调用的次数超出预设")
        return self.responses.pop(0)


def _install_stubs():
    """在导入 scan_pipeline 之前把依赖换成桩。"""
    bridge = BridgeStub()
    published = []

    arduino = types.ModuleType("arduino")
    utils = types.ModuleType("arduino.app_utils")
    utils.Bridge = bridge
    arduino.app_utils = utils
    sys.modules["arduino"] = arduino
    sys.modules["arduino.app_utils"] = utils

    config = types.ModuleType("config")
    config.MCU_TIMEOUT_S = 0.2
    config.CONTROL_HZ = 30
    sys.modules["config"] = config

    publishers = types.ModuleType("ros_publishers")

    def publish_scan(start_ms, period_ms, ranges, sectors, clock):
        published.append({"start_ms": start_ms, "period_ms": period_ms,
                          "ranges": list(ranges), "sectors": sectors,
                          "clock": clock})

    publishers.publish_scan = publish_scan
    sys.modules["ros_publishers"] = publishers
    return bridge, published


BRIDGE, PUBLISHED = _install_stubs()
import scan_pipeline  # noqa: E402  必须在装好桩之后导入


# ---------------------------------------------------------------- 辅助
def slice_payload(seq, offset, count, *, sectors=180, start_ms=1000,
                  period_ms=100, latest=1, mcu_now=1200, values=None):
    """构造一片 lidar_slice 的返回：9 个元数据 + count 个毫米值。"""
    if values is None:
        values = [3000] * count
    return [seq, start_ms, period_ms, 1, sectors, offset, count,
            latest, mcu_now] + list(values)


def full_frame(seq, *, start_ms=1000, period_ms=100, mcu_now=1200,
               latest=None):
    """三片 60，凑满 180 个分区。"""
    latest = seq if latest is None else latest
    return [
        slice_payload(seq, 0, 60, start_ms=start_ms, period_ms=period_ms,
                      latest=latest, mcu_now=mcu_now),
        slice_payload(seq, 60, 60, start_ms=start_ms, period_ms=period_ms,
                      latest=latest, mcu_now=mcu_now),
        slice_payload(seq, 120, 60, start_ms=start_ms, period_ms=period_ms,
                      latest=latest, mcu_now=mcu_now),
    ]


class ScanPipelineTest(unittest.TestCase):
    def setUp(self):
        BRIDGE.responses.clear()
        BRIDGE.calls.clear()
        BRIDGE.raise_on.clear()
        PUBLISHED.clear()
        scan_pipeline.reset_assembly(0.0, "")
        scan_pipeline.SCAN_STATS.update({k: 0 for k in scan_pipeline.SCAN_STATS})
        self.t = 100.0

    def tick(self, step=0.05):
        """推进时间并调用一次生产函数。"""
        self.t += step
        scan_pipeline.fetch_scan_slice(self.t)
        return self.t

    # ------------------------------------------------------------ T01
    def test_t01_normal_three_slices(self):
        BRIDGE.queue(*full_frame(seq=7))
        for _ in range(3):
            self.tick()
        self.assertEqual(len(PUBLISHED), 1, "三片拼齐应发布一帧")
        self.assertEqual(PUBLISHED[0]["sectors"], 180)
        self.assertEqual(PUBLISHED[0]["start_ms"], 1000)
        self.assertAlmostEqual(PUBLISHED[0]["ranges"][0], 3.0, places=3)
        self.assertEqual(scan_pipeline.SCAN_STATS["published"], 1)

    # ------------------------------------------------------------ T02
    def test_t02_duplicate_frame_uses_short_retry(self):
        BRIDGE.queue(*full_frame(seq=7))
        for _ in range(3):
            self.tick()
        self.assertEqual(len(PUBLISHED), 1)

        # 同一帧号再读一次：只允许短重试，绝不能退避 2 s
        self.t = scan_pipeline.SCAN_ASSEMBLY["next_try"] + 0.01
        BRIDGE.queue(slice_payload(7, 0, 60))
        attempt_at = self.tick(step=0.01)
        state = scan_pipeline.SCAN_ASSEMBLY
        self.assertGreaterEqual(state["next_try"], attempt_at + 0.05,
                                "重复帧应安排重试而不是立刻重读")
        self.assertLess(state["next_try"], attempt_at + 0.5,
                        "偶发重复帧不能触发 2 s 退避")
        self.assertLess(state["next_try"], attempt_at
                        + scan_pipeline.SCAN_PUBLISH_S,
                        "短重试间隔应小于正常发布周期")
        self.assertEqual(scan_pipeline.SCAN_STATS["duplicate_frames"], 1)
        self.assertEqual(scan_pipeline.SCAN_STATS["dup_backoff"], 0)

        # 新帧到达后应正常发布
        self.t = state["next_try"] + 0.01
        now_ms = int(self.t * 1000) + 200
        BRIDGE.queue(*full_frame(seq=8, start_ms=now_ms - 100, mcu_now=now_ms))
        for _ in range(3):
            self.tick()
        self.assertEqual(len(PUBLISHED), 2, "重复帧之后新帧必须能发布")

    # ------------------------------------------------------------ T03
    def test_t03_sustained_duplicates_backoff(self):
        BRIDGE.queue(*full_frame(seq=7))
        for _ in range(3):
            self.tick()
        PUBLISHED.clear()

        attempt_at = None
        for _ in range(scan_pipeline.SCAN_DUP_LIMIT):
            BRIDGE.queue(slice_payload(7, 0, 60))
            self.t = scan_pipeline.SCAN_ASSEMBLY["next_try"] + 0.01
            attempt_at = self.tick(step=0.01)

        state = scan_pipeline.SCAN_ASSEMBLY
        self.assertGreaterEqual(state["next_try"], attempt_at + 1.9,
                                "连续重复帧应进入约 2 s 的退避")
        self.assertEqual(scan_pipeline.SCAN_STATS["dup_backoff"], 1)
        self.assertIn("停扫", state["backoff_reason"])
        self.assertEqual(PUBLISHED, [], "停扫期间不应发布")

    # ------------------------------------------------------------ T04
    def test_t04_expired_at_publish_time(self):
        # 第一片新鲜，第三片时已经过了 500 ms —— 帧号差仍是 0，
        # 只靠 latest-seq 判断不出来，必须靠真实帧龄拦下。
        BRIDGE.queue(
            slice_payload(9, 0, 60, start_ms=1000, mcu_now=1050),
            slice_payload(9, 60, 60, start_ms=1000, mcu_now=1200),
            slice_payload(9, 120, 60, start_ms=1000, mcu_now=1500),
        )
        for _ in range(3):
            self.tick()
        self.assertEqual(PUBLISHED, [], "拼齐时已过期的帧不得发布")
        self.assertEqual(scan_pipeline.SCAN_STATS["expired_frames"], 1)

    # ------------------------------------------------------------ T05
    def test_t05_short_slice_discarded(self):
        # 声明 count=60，实际只给 30 个值
        short = slice_payload(11, 0, 60, values=[3000] * 30)
        BRIDGE.queue(short)
        self.tick()
        self.assertEqual(PUBLISHED, [])
        self.assertEqual(scan_pipeline.SCAN_STATS["invalid_slices"], 1)
        self.assertEqual(scan_pipeline.SCAN_ASSEMBLY["seq"], 0,
                         "非法分片应放弃当前拼帧")

    # ------------------------------------------------------------ T06
    def test_t06_wrong_offset_discarded(self):
        BRIDGE.queue(slice_payload(12, 0, 60, mcu_now=1200))
        self.tick()
        self.assertNotEqual(scan_pipeline.SCAN_ASSEMBLY["seq"], 0)
        # 第二片的 offset 不是 60，直接丢弃
        BRIDGE.queue(slice_payload(12, 120, 60, mcu_now=1200))
        self.tick()
        self.assertEqual(scan_pipeline.SCAN_ASSEMBLY["seq"], 0)
        self.assertEqual(PUBLISHED, [])

    # ------------------------------------------------------------ T07
    def test_t07_mcu_restart_clears_assembly(self):
        BRIDGE.queue(*full_frame(seq=5, start_ms=8900, mcu_now=9000))
        for _ in range(3):
            self.tick()
        self.assertEqual(len(PUBLISHED), 1)

        # uptime 从 9000 掉到 120：MCU 重启
        self.t = scan_pipeline.SCAN_ASSEMBLY["next_try"] + 0.01
        BRIDGE.queue(slice_payload(1, 0, 60, start_ms=50, mcu_now=120))
        self.tick(step=0.01)
        self.assertEqual(scan_pipeline.SCAN_STATS["session_resets"], 1,
                         "uptime 回退应识别为重启并清空历史")
        state = scan_pipeline.SCAN_ASSEMBLY
        self.assertEqual(state["seq"], 0)
        self.assertEqual(state["last_seq"], 0, "重启后旧的已发布帧号必须清掉")
        self.assertEqual(state["dup_streak"], 0)

    # ------------------------------------------------------------ T08
    def test_t08_recover_after_backoff(self):
        BRIDGE.queue(*full_frame(seq=3))
        for _ in range(3):
            self.tick()
        PUBLISHED.clear()

        # 走到退避
        for _ in range(scan_pipeline.SCAN_DUP_LIMIT):
            BRIDGE.queue(slice_payload(3, 0, 60))
            self.t = scan_pipeline.SCAN_ASSEMBLY["next_try"] + 0.01
            self.tick(step=0.01)
        self.assertGreater(scan_pipeline.SCAN_ASSEMBLY["next_try"], self.t)

        # 等退避结束，雷达恢复
        self.t = scan_pipeline.SCAN_ASSEMBLY["next_try"] + 0.01
        now_ms = int(self.t * 1000) + 200
        BRIDGE.queue(*full_frame(seq=4, start_ms=now_ms - 100, mcu_now=now_ms))
        for _ in range(3):
            self.tick()
        self.assertEqual(len(PUBLISHED), 1, "退避结束后必须能恢复正常发布")
        self.assertEqual(scan_pipeline.SCAN_ASSEMBLY["backoff_reason"], "")

    # ------------------------------------------------------- 诊断字段
    def test_status_exposes_diagnostics(self):
        BRIDGE.queue(*full_frame(seq=2))
        for _ in range(3):
            self.tick()
        snapshot = scan_pipeline.status()
        for key in ("duplicate_frames", "invalid_slices", "expired_frames",
                    "published_frames", "max_publish_gap_ms",
                    "consecutive_duplicates", "backoff_reason"):
            self.assertIn(key, snapshot)


if __name__ == "__main__":
    unittest.main(verbosity=2)
