"""离线验证真实位姿命令链，不需要 Arduino、ROS 或开发板。"""

import ast
import json
import math
from pathlib import Path
import time
import types
import unittest


ROOT = Path(__file__).resolve().parents[2]
NAV_BRIDGE = ROOT / "bricks/ros2_bridge/docker/ros_ws/src/arduino_ros2_bringup/arduino_ros2_bringup/nodes/nav_bridge.py"


def production_function(path, name, namespace):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    function = next(node for node in ast.walk(tree)
                    if isinstance(node, ast.FunctionDef) and node.name == name)
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), "exec"), namespace)
    return namespace[name]


def initial_message():
    return types.SimpleNamespace(
        header=types.SimpleNamespace(),
        pose=types.SimpleNamespace(
            pose=types.SimpleNamespace(position=types.SimpleNamespace(),
                                       orientation=types.SimpleNamespace()),
            covariance=[0.0] * 36))


class PoseChainTest(unittest.TestCase):
    def setUp(self):
        self.ns = {"json": json, "math": math, "time": time, "TTL_POSE": 6.0,
                   "MAP_FRAME": "map", "PoseWithCovarianceStamped": initial_message,
                   "yaw_to_quat": lambda yaw: (0., 0., math.sin(yaw / 2), math.cos(yaw / 2))}

    def test_socket_navlink_bridge_preserve_request(self):
        set_pose = production_function(ROOT / "python/navlink.py", "set_pose", self.ns)
        frames = []
        nav = types.SimpleNamespace(_push=lambda payload, ttl: frames.append(payload))
        nav.set_pose = types.MethodType(set_pose, nav)
        self.ns["NAV"] = nav
        on_socket = production_function(ROOT / "python/socket_handlers.py", "on_nav_cmd", self.ns)
        on_bridge = production_function(NAV_BRIDGE, "_on_cmd", self.ns)
        for request in ("page-a:1", "page-b:1", 7, None):
            payload = {"cmd": "pose", "x": 1, "y": 2, "yaw": 0.7}
            if request is not None:
                payload["req"] = request
            on_socket(None, payload)
            expected = 0 if request is None else request
            self.assertEqual(frames[-1]["req"], expected)
            node = types.SimpleNamespace(_set_initial_pose=lambda *args: None)
            on_bridge(node, types.SimpleNamespace(data=json.dumps(frames[-1])))
            self.assertEqual(node._pose_req, expected)

    def test_request_baseline_and_stale_tf(self):
        set_pose = production_function(NAV_BRIDGE, "_set_initial_pose", self.ns)
        check = production_function(NAV_BRIDGE, "_check_pose_ack", self.ns)
        current = [(0., 0., 0.)]
        at_publish = []
        node = types.SimpleNamespace(
            _cancel=lambda reason: None,
            _odom_stamp=lambda: types.SimpleNamespace(to_msg=lambda: None),
            _mapodom_pose=lambda: current[0], _mapodom_at=None, _nav="idle",
            get_clock=lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=1000000000)),
            get_logger=lambda: types.SimpleNamespace(info=lambda text: None))
        node._pub_initial = types.SimpleNamespace(publish=lambda msg: at_publish.append(node._mapodom_at))
        set_pose(node, 0.2, 0., 0.)
        self.assertEqual(at_publish, [(0., 0., 0.)])
        check(node, [0., 0., 0.])
        self.assertEqual(node._pose_ack, "waiting")
        current[0] = (0.2, 0., 0.)
        check(node, [0.2, 0., 0.], age=2.0)
        self.assertEqual(node._pose_ack, "waiting")
        check(node, [0.2, 0., 0.], age=0.1)
        self.assertEqual(node._pose_ack, "ok")

    def test_web_command_ttl_and_shared_estop_generation(self):
        sent = []
        queued = []
        shared = types.SimpleNamespace(ESTOP_SEQ=3)
        def bump():
            shared.ESTOP_SEQ += 1
        shared.bump_estop = bump
        event = types.SimpleNamespace(set=lambda: None)
        self.ns.update(app_state=shared, config=types.SimpleNamespace(CMD_TIMEOUT_S=0.4),
                       control=types.SimpleNamespace(clamp_cmd=lambda linear, angular: (linear, angular)),
                       WEB=types.SimpleNamespace(publish=lambda payload, **kw: sent.append(kw)),
                       PAUSED=event, STOP_PENDING=event,
                       SELFTEST=types.SimpleNamespace(cancel=lambda: None),
                       NAV=types.SimpleNamespace(cancel=lambda: None),
                       REQUESTS=types.SimpleNamespace(put=queued.append))
        command = production_function(ROOT / "python/socket_handlers.py", "on_cmd_vel", self.ns)
        stop = production_function(ROOT / "python/socket_handlers.py", "on_stop", self.ns)
        selftest = production_function(ROOT / "python/socket_handlers.py", "on_selftest", self.ns)
        command(None, {"linear": 0.05, "angular": 0.0})
        self.assertEqual(sent[-1], {"ttl": 0.4})
        stop()
        selftest(None, {"case": "zero_left"})
        self.assertEqual(shared.ESTOP_SEQ, 4)
        self.assertEqual(queued[-1], ("selftest", "zero_left", 4))


if __name__ == "__main__":
    unittest.main()
