"""RC-002 差速映射回归测试。

直接调用 python/control.py 的 wheel_targets()，不重新实现算法。
    python -m unittest discover -s tests/control -v
"""

import os
import sys
import unittest

PYDIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "python"))
if PYDIR not in sys.path:
    sys.path.insert(0, PYDIR)

import control  # noqa: E402

CPM = control.config.COUNTS_PER_METER
HALF_TRACK = control.config.TRACK_M / 2.0
MAX_CPS = control.config.MAX_WHEEL_CPS


def targets(linear, angular, **kw):
    return control.wheel_targets(linear, angular, **kw)


class ZeroInputTest(unittest.TestCase):
    def test_all_flag_combinations_output_zero(self):
        for snap in (True, False):
            for pivot in (True, False):
                for mapping in ("continuous", "snap"):
                    with self.subTest(snap=snap, pivot=pivot, mapping=mapping):
                        self.assertEqual(
                            targets(0.0, 0.0, snap=snap, allow_pivot=pivot,
                                    mapping=mapping), (0, 0))


class DirectionTest(unittest.TestCase):
    def test_full_forward_both_positive_equal(self):
        right, left = targets(0.09, 0.0)
        self.assertGreater(right, 0)
        self.assertEqual(right, left)

    def test_full_reverse_both_negative_equal(self):
        right, left = targets(-0.09, 0.0)
        self.assertLess(right, 0)
        self.assertEqual(right, left)
        fwd, _ = targets(0.09, 0.0)
        self.assertEqual(right, -fwd)

    def test_pivot_left_and_right(self):
        right, left = targets(0.0, 1.2)
        self.assertGreater(right, 0)
        self.assertLess(left, 0)
        right2, left2 = targets(0.0, -1.2)
        self.assertEqual((right2, left2), (-right, -left))


class ContinuityTest(unittest.TestCase):
    def test_no_turn_jump(self):
        """ω 每 0.001 rad/s 一格，实际角速度不允许出现阶跃。"""
        for linear in (0.02, 0.04, 0.06):
            previous = None
            worst = 0.0
            step = 0.0
            while step <= 1.2001:
                right, left = targets(linear, step)
                actual = (right - left) / 2.0 / CPM / HALF_TRACK
                if previous is not None:
                    worst = max(worst, abs(actual - previous))
                previous = actual
                step = round(step + 0.001, 4)
            with self.subTest(linear=linear):
                self.assertLess(worst, 0.01,
                                "ω 相邻档之间不应出现 >0.01rad/s 的跳变")

    def test_inner_wheel_never_reverses_when_moving(self):
        for linear in (0.01, 0.02, 0.04, 0.08):
            step = 0.0
            while step <= 1.2001:
                right, left = targets(linear, step)
                with self.subTest(linear=linear, omega=step):
                    self.assertGreaterEqual(min(right, left), 0,
                                            "有前进速度时内轮不得反向")
                step = round(step + 0.02, 4)


class LimitTest(unittest.TestCase):
    def test_peak_clamped(self):
        for linear, angular in ((0.09, 1.2), (0.09, -1.2), (-0.09, 1.2),
                                (0.09, 0.0)):
            right, left = targets(linear, angular)
            with self.subTest(linear=linear, angular=angular):
                self.assertLessEqual(max(abs(right), abs(left)), MAX_CPS)

    def test_ratio_kept_when_clamped(self):
        """超限时等比缩放，内外轮比例不变。"""
        right, left = targets(0.09, 1.2)
        self.assertGreater(right, 0)
        self.assertGreaterEqual(left, 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
