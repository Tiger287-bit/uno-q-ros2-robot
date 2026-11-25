"""NAV-006 雷达拼帧回归测试。

mock 掉 Bridge / publish_scan，直接调用生产代码 `python/scan_pipeline.py`。

    python -m unittest discover -s tests/scan -v
"""

import os
import sys
import types
import unittest

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
PYDIR = os.path.join(REPO, "python")
if PYDIR not in sys.path:
    sys.path.insert(0, PYDIR)


class BridgeStub:
    def __init__(self):
        self.responses = []
        self.calls = []
        self.fail_next = 0

    def queue(self, *payloads):
        self.responses.extend(payloads)

    def call(self, name, seq=None, offset=None, count=None, timeout=None):
        self.calls.append((name, seq, offset, count))
        if self.fail_next > 0:
            self.fail_next -= 1
            raise OSError("stub rpc failure")
        if not self.responses:
            raise AssertionError("Bridge 调用次数超出预设")
        return self.responses.pop(0)


BRIDGE = BridgeStub()
PUBLISHED = []


def _install_stubs():
    arduino = types.ModuleType("arduino")
    utils = types.ModuleType("arduino.app_utils")
    utils.Bridge = BRIDGE
    arduino.app_utils = utils
    sys.modules["arduino"] = arduino
    sys.modules["arduino.app_utils"] = utils

    config = types.ModuleType("config")
    config.MCU_TIMEOUT_S = 0.2
    config.CONTROL_HZ = 30
    sys.modules["config"] = config

    publishers = types.ModuleType("ros_publishers")

    def publish_scan(start_ms, period_ms, ranges, sectors, clock):
        PUBLISHED.append({"start_ms": start_ms, "period_ms": period_ms,
                          "ranges": list(ranges), "sectors": sectors})

    publishers.publish_scan = publish_scan
    sys.modules["ros_publishers"] = publishers


_install_stubs()
import scan_pipeline  # noqa: E402


def slice_payload(seq, offset, count, *, start_ms, mcu_now, period_ms=100,
                  valid=None, sectors=180, latest=None, values=None):
    """构造一片 lidar_slice 返回。valid 默认按实际非零值计算，符合固件语义。"""
    if values is None:
        values = [3000] * count
    if valid is None:
        valid = sum(1 for v in values if v)
    if latest is None:
        latest = seq
    return [seq, start_ms, period_ms, valid, sectors, offset, count,
            latest, mcu_now] + list(values)


def frame(seq, *, mcu_now, start_ms=None, latest=None, period_ms=100,
          values=None, valid=None):
    """三片 60，凑满 180 分区。

    注意 valid 是**整圈**有效分区数（0~180），每片都报同一个值 —— 这是固件
    协议（lidar.h 的 Frame::valid）。桩里若按"每片 60"来算，拼出来的 180 个
    有效分区就会与 valid 不符，被 valid 一致性检查正确拦下。
    """
    start_ms = mcu_now - 100 if start_ms is None else start_ms
    latest = seq if latest is None else latest
    if values is None:
        values = [3000] * 180
    if valid is None:
        valid = sum(1 for v in values if v)
    out = []
    for offset in (0, 60, 120):
        out.append(slice_payload(seq, offset, 60, start_ms=start_ms,
                                 mcu_now=mcu_now, period_ms=period_ms,
                                 latest=latest, valid=valid,
                                 values=values[offset:offset + 60]))
    return out


class Base(unittest.TestCase):
    def setUp(self):
        BRIDGE.responses.clear()
        BRIDGE.calls.clear()
        BRIDGE.fail_next = 0
        PUBLISHED.clear()
        scan_pipeline.reset_assembly(0.0, "")
        for key in scan_pipeline.SCAN_STATS:
            scan_pipeline.SCAN_STATS[key] = 0
        self.state = scan_pipeline.SCAN_ASSEMBLY
        self.t = 100.0

    def tick(self, step=0.05):
        self.t += step
        scan_pipeline.fetch_scan_slice(self.t)
        return self.t

    def publish_once(self, seq, mcu_now=1200, **kw):
        BRIDGE.queue(*frame(seq, mcu_now=mcu_now, **kw))
        for _ in range(3):
            self.tick()
        self.state = scan_pipeline.SCAN_ASSEMBLY


class CoreTest(Base):
    def test_t01_normal_three_slices(self):
        self.publish_once(7)
        self.assertEqual(len(PUBLISHED), 1)
        self.assertEqual(PUBLISHED[0]["sectors"], 180)
        self.assertAlmostEqual(PUBLISHED[0]["ranges"][0], 3.0, places=3)

    def test_t02_duplicate_short_retry_then_recover(self):
        self.publish_once(7)
        self.t = self.state["next_try"] + 0.01
        BRIDGE.queue(slice_payload(7, 0, 60, start_ms=1100, mcu_now=1200))
        at = self.tick(step=0.01)
        st = scan_pipeline.SCAN_ASSEMBLY
        self.assertLess(st["next_try"], at + 0.5, "偶发重复帧不能退避 2s")
        self.assertEqual(scan_pipeline.SCAN_STATS["duplicate_frames"], 1)
        self.t = st["next_try"] + 0.01
        BRIDGE.queue(*frame(8, mcu_now=int(self.t * 1000) + 200))
        for _ in range(3):
            self.tick()
        self.assertEqual(len(PUBLISHED), 2)

    def test_t03_sustained_duplicates_and_old_frame(self):
        self.publish_once(7, mcu_now=1200)
        PUBLISHED.clear()
        at = self.t
        for _ in range(scan_pipeline.SCAN_DUP_LIMIT):
            self.t = scan_pipeline.SCAN_ASSEMBLY["next_try"] + 0.01
            BRIDGE.queue(slice_payload(7, 0, 60, start_ms=600, mcu_now=1200))
            at = self.tick(step=0.01)
        st = scan_pipeline.SCAN_ASSEMBLY
        self.assertGreaterEqual(st["next_try"], at + 1.9)
        self.assertEqual(scan_pipeline.SCAN_STATS["dup_backoff"], 1)
        self.assertIn("停扫", st["backoff_reason"])
        self.assertEqual(st["dup_streak"], scan_pipeline.SCAN_DUP_LIMIT,
                         "退避期间必须保留真实连续重复次数")
        self.assertEqual(PUBLISHED, [])

    def test_t04_expired_at_publish_time(self):
        BRIDGE.queue(
            slice_payload(9, 0, 60, start_ms=1000, mcu_now=1050),
            slice_payload(9, 60, 60, start_ms=1000, mcu_now=1200),
            slice_payload(9, 120, 60, start_ms=1000, mcu_now=1500),
        )
        for _ in range(3):
            self.tick()
        self.assertEqual(PUBLISHED, [])
        self.assertEqual(scan_pipeline.SCAN_STATS["expired_frames"], 1)

    def test_t05_short_values(self):
        BRIDGE.queue(slice_payload(11, 0, 60, start_ms=1100, mcu_now=1200,
                                   values=[3000] * 30, valid=30))
        self.tick()
        self.assertEqual(PUBLISHED, [])
        self.assertEqual(scan_pipeline.SCAN_STATS["invalid_slices"], 1)

    def test_t06_wrong_offset(self):
        BRIDGE.queue(slice_payload(12, 0, 60, start_ms=1100, mcu_now=1200))
        self.tick()
        self.assertNotEqual(scan_pipeline.SCAN_ASSEMBLY["seq"], 0)
        BRIDGE.queue(slice_payload(12, 120, 60, start_ms=1100, mcu_now=1200))
        self.tick()
        self.assertEqual(scan_pipeline.SCAN_ASSEMBLY["seq"], 0)
        self.assertEqual(PUBLISHED, [])

    def test_t07_mcu_restart(self):
        self.publish_once(5, mcu_now=9000)
        self.assertEqual(len(PUBLISHED), 1)
        self.t = self.state["next_try"] + 0.01
        BRIDGE.queue(slice_payload(1, 0, 60, start_ms=50, mcu_now=120))
        self.tick(step=0.01)
        self.assertEqual(scan_pipeline.SCAN_STATS["session_resets"], 1)
        st = scan_pipeline.SCAN_ASSEMBLY
        self.assertEqual((st["seq"], st["last_seq"], st["dup_streak"]), (0, 0, 0))

    def test_t08_recover_after_backoff(self):
        self.publish_once(3, mcu_now=1200)
        PUBLISHED.clear()
        for _ in range(scan_pipeline.SCAN_DUP_LIMIT):
            self.t = scan_pipeline.SCAN_ASSEMBLY["next_try"] + 0.01
            BRIDGE.queue(slice_payload(3, 0, 60, start_ms=600, mcu_now=1200))
            self.tick(step=0.01)
        self.t = scan_pipeline.SCAN_ASSEMBLY["next_try"] + 0.01
        BRIDGE.queue(*frame(4, mcu_now=int(self.t * 1000) + 200))
        for _ in range(3):
            self.tick()
        self.assertEqual(len(PUBLISHED), 1)
        self.assertEqual(scan_pipeline.SCAN_ASSEMBLY["backoff_reason"], "")


class MetadataTest(Base):
    def test_t10_latest_from_last_slice(self):
        """第一片 latest=100、第三片 latest=106，帧龄仍新鲜 -> 必须判过期。"""
        BRIDGE.queue(
            slice_payload(100, 0, 60, start_ms=1100, mcu_now=1200, latest=100),
            slice_payload(100, 60, 60, start_ms=1100, mcu_now=1200, latest=103),
            slice_payload(100, 120, 60, start_ms=1100, mcu_now=1200, latest=106),
        )
        for _ in range(3):
            self.tick()
        self.assertEqual(PUBLISHED, [], "应按最后一片的 latest 判过期")
        self.assertEqual(scan_pipeline.SCAN_STATS["stale"], 1)

    def test_t11_start_ms_mismatch(self):
        BRIDGE.queue(
            slice_payload(20, 0, 60, start_ms=1100, mcu_now=1200),
            slice_payload(20, 60, 60, start_ms=1150, mcu_now=1200),
        )
        self.tick()
        self.tick()
        self.assertEqual(PUBLISHED, [])
        self.assertEqual(scan_pipeline.SCAN_STATS["invalid_slices"], 1)

    def test_t12_period_and_valid_mismatch(self):
        for patch in ({"period_ms": 200}, {"valid": 20}):
            with self.subTest(patch=patch):
                self.setUp()
                first = slice_payload(21, 0, 60, start_ms=1100, mcu_now=1200)
                second = slice_payload(21, 60, 60, start_ms=1100, mcu_now=1200,
                                       **patch)
                BRIDGE.queue(first, second)
                self.tick()
                self.tick()
                self.assertEqual(PUBLISHED, [], "元数据不一致必须丢弃整帧")
                self.assertEqual(scan_pipeline.SCAN_STATS["invalid_slices"], 1)

    def test_t13_more_values_than_count(self):
        payload = slice_payload(22, 0, 60, start_ms=1100, mcu_now=1200,
                                values=[3000] * 70, valid=70)
        BRIDGE.queue(payload)
        self.tick()
        self.assertEqual(PUBLISHED, [])
        self.assertEqual(scan_pipeline.SCAN_STATS["invalid_slices"], 1)

    def test_t14_count_exceeds_request(self):
        BRIDGE.queue(slice_payload(23, 0, 90, start_ms=1100, mcu_now=1200,
                                   values=[3000] * 90, valid=90))
        self.tick()
        self.assertEqual(PUBLISHED, [])
        self.assertEqual(scan_pipeline.SCAN_STATS["invalid_slices"], 1)


class ValidFieldTest(Base):
    def test_t15a_valid_zero_not_published(self):
        BRIDGE.queue(*frame(30, mcu_now=1200, values=[0] * 180))
        for _ in range(3):
            self.tick()
        self.assertEqual(PUBLISHED, [], "valid=0 不得作为导航观测发布")
        self.assertEqual(scan_pipeline.SCAN_STATS["no_echo_frames"], 1)

    def test_t15b_valid_out_of_range(self):
        BRIDGE.queue(slice_payload(31, 0, 60, start_ms=1100, mcu_now=1200,
                                   valid=181))
        self.tick()
        self.assertEqual(scan_pipeline.SCAN_STATS["invalid_slices"], 1)

    def test_t16_valid_mismatch_with_ranges(self):
        payloads = frame(32, mcu_now=1200, values=[3000] * 180)
        for payload in payloads:
            payload[3] = 10          # 声明 valid=10，实际 180 个分区都有值
        BRIDGE.queue(*payloads)
        for _ in range(3):
            self.tick()
        self.assertEqual(PUBLISHED, [])
        self.assertEqual(scan_pipeline.SCAN_STATS["valid_mismatch"], 1)


class SessionTest(Base):
    def test_t17_uptime_int32_wrap_is_not_restart(self):
        self.publish_once(40, mcu_now=2 ** 31 - 1000)
        self.assertEqual(len(PUBLISHED), 1)
        self.t = self.state["next_try"] + 0.01
        wrap_now = 500
        BRIDGE.queue(*frame(41, mcu_now=wrap_now, start_ms=wrap_now - 100))
        for _ in range(3):
            self.tick()
        self.assertEqual(scan_pipeline.SCAN_STATS["session_resets"], 0,
                         "int32 回绕不能被当成 MCU 重启")
        self.assertEqual(len(PUBLISHED), 2)

    def test_t18_mcu_session_change_resets(self):
        self.publish_once(50, mcu_now=1200)
        scan_pipeline.on_mcu_session_change(self.t, "epoch=2")
        st = scan_pipeline.SCAN_ASSEMBLY
        self.assertEqual(st["last_seq"], 0)
        self.assertEqual(scan_pipeline.SCAN_STATS["session_resets"], 1)
        self.t = st["next_try"] + 0.01
        BRIDGE.queue(*frame(1, mcu_now=int(self.t * 1000) + 200))
        for _ in range(3):
            self.tick()
        self.assertEqual(len(PUBLISHED), 2, "新会话必须能正常发布")

    def test_t19_lidar_restart_without_mcu_restart(self):
        self.publish_once(7, mcu_now=100000)
        self.t = self.state["next_try"] + 0.01
        # 重启后的第一片会触发会话重置并被丢弃（只排一片，避免余下分片
        # 被后续读取当成"偏移不符"白白消耗掉）
        BRIDGE.queue(slice_payload(1, 0, 60, start_ms=100900, mcu_now=101000))
        self.tick()
        self.assertEqual(scan_pipeline.SCAN_STATS["lidar_restarts"], 1)
        # 重置后能正常接受新会话的帧
        self.t = scan_pipeline.SCAN_ASSEMBLY["next_try"] + 0.01
        BRIDGE.queue(*frame(2, mcu_now=int(self.t * 1000) + 200))
        for _ in range(3):
            self.tick()
        self.assertEqual(len(PUBLISHED), 2, "雷达重启后应能发布新会话数据")

    def test_t20_rpc_error_then_recover(self):
        BRIDGE.fail_next = 1
        self.tick()
        self.assertEqual(scan_pipeline.SCAN_STATS["rpc_errors"], 1)
        self.assertIn("异常", scan_pipeline.SCAN_ASSEMBLY["last_error"])
        self.t = scan_pipeline.SCAN_ASSEMBLY["next_try"] + 0.01
        BRIDGE.queue(*frame(60, mcu_now=int(self.t * 1000) + 200))
        for _ in range(3):
            self.tick()
        self.assertEqual(len(PUBLISHED), 1, "RPC 异常后必须能恢复拼帧")


class ApiTest(Base):
    def test_t21_status_exposes_diagnostics(self):
        self.publish_once(70)
        snap = scan_pipeline.status()
        for key in ("consecutive_duplicates", "backoff_reason",
                    "backoff_remaining_s", "assembling", "assembled_slices",
                    "published_frames", "last_publish_at", "last_frame_age_ms",
                    "last_error", "published_means",
                    "duplicate_frames", "invalid_slices", "expired_frames",
                    "rpc_errors", "session_resets", "max_publish_gap_ms"):
            self.assertIn(key, snap)

    def test_t21b_api_uses_status(self):
        with open(os.path.join(PYDIR, "api_routes.py"), encoding="utf-8") as fh:
            source = fh.read()
        self.assertIn("scan_pipeline.status()", source,
                      "/api/lidar 必须返回 status()，而不是裸 SCAN_STATS")


if __name__ == "__main__":
    unittest.main(verbosity=2)
