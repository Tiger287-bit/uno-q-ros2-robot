"""运行模式管理：导航 <-> 建图，任何时候只有一套栈处于 active。

为什么需要
    slam_toolbox 与 AMCL 都发布 map->odom。两套一起 active，同一个 TF 就有
    两个发布者，表现为定位跳变、路径画歪，而且没人报错 —— 看起来像"雷达不准"。
    地图已经建好时跑导航；要重扫时切到建图，由本节点保证互斥。

为什么两套栈都常驻，而不是切换时重启 launch
    - 重启一次 nav.launch 要 10 s 以上（六个节点重配置 + AMCL 重新撒粒子），
      而且 nav_bridge 与 App 的连接会断一次；
    - 生命周期切换是原地 configure/activate，2~5 s 完成，TF 与话题都不重建。
    代价是两份进程内存同时占着（约 300 MB），板子扛得住。

为什么导航组走 nav2_lifecycle_manager，建图组直接调 slam_toolbox
    Jazzy 的 slam_toolbox 用 launch 事件自己 configure/activate，不接
    lifecycle_manager（online_async_launch.py 的 use_lifecycle_manager
    默认 false）。导航组有六个节点，顺序与依赖由 nav2 的 manager 管更稳。
    两边的入口不同，但都是标准的 lifecycle 服务，本节点只调用、不改实现。

接口
    /app/nav_cmd    {"cmd":"mode","mode":"nav"|"slam"}
    /app/mode_state {"mode":"nav","state":"ready|switching|error","note":"..."}

    命令走 /app/nav_cmd 是为了复用 App 已经建好的那一条发布链路 —— 页面上
    "导航/建图"开关和"设目标"是同一条 socket 通道，不额外开一个 WebSocket。

只用 PAUSE/RESUME，不用 SHUTDOWN
    nav2_lifecycle_manager 的 shutdown() 会把每个节点一路带到 finalized：
    之后 STARTUP 与 RESUME 都无效，而且它的服务客户端在 shutdown 末尾被销毁，
    再调 RESUME 会让 manager 自己段错误退出。实测过一次，"导航组启动失败" +
    manager 死亡，只能重启容器。
    所以导航组只做 deactivate（PAUSE）/ activate（RESUME）：代价地图与插件
    留在内存里，切换 1~2 s 完成，来回切多少次都不会坏。

为什么不把 slam 的位姿交给 AMCL
    切回导航时，AMCL 加载的是**已保存的旧地图**，slam 建的是**这一次的新地图**，
    两者原点无关（slam 每次启动都从新的原点开始）。把 slam 坐标直接当 AMCL
    初始位姿，等于把车定位到旧地图里的随机位置，Nav2 会按错位规划。所以切回
    导航只做"重新激活 AMCL"，由页面提示用户重设初始位姿（pose_ack=needed）。

互斥宁可失败也不含糊
    slam_toolbox 的 get_state 查不到时（服务在但节点卡住），不能当成"它没运行"
    就直接激活导航 —— 那样可能两个 map->odom 发布者同时存在。查不到就报错，
    让人来处理。
"""

import json
import queue
import threading
import time

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.srv import GetState as _GetState_Srv
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from arduino_ros2_bringup.common.runtime import run
from arduino_ros2_bringup.common.topics import Topics

# lifecycle_msgs/msg/State 的主状态 id。写常量而不是 import 消息类，
# 是为了让这里读起来是"数字 -> 名字"的对照表。
STATE_UNCONFIGURED = 1
STATE_INACTIVE = 2
STATE_ACTIVE = 3
STATE_FINALIZED = 4
_GetStateRequest = _GetState_Srv.Request

NAV_MANAGE_SERVICE = "/lifecycle_manager_navigation/manage_nodes"
# 用 controller_server 探活，不能用 amcl：固定起始位姿模式下 amcl 根本不启动，
# 探测它会一直失败，mode_manager 会误判"导航组没起来"。
NAV_PROBE_SERVICE = "/controller_server/get_state"
SLAM_STATE_SERVICE = "/slam_toolbox/get_state"
SLAM_CHANGE_SERVICE = "/slam_toolbox/change_state"

MODES = ("nav", "slam")
MODE_LABEL = {"nav": "导航", "slam": "建图"}


class ModeManager(Node):
    """按请求在导航与建图之间切换，并把当前模式广播出去。"""

    def __init__(self):
        super().__init__("mode_manager")
        self._initial = str(
            self.declare_parameter("initial_mode", "nav").value)
        self._timeout = float(
            self.declare_parameter("switch_timeout", 40.0).value)
        state_hz = float(self.declare_parameter("state_hz", 1.0).value)
        # 激活导航前等多久的扫描。App 那边的雷达要 8~12 s 才起来，
        # 而 AMCL 只有收到扫描才会发 map->odom。不等就会撞上冷启动竞态：
        # planner 的代价地图等不到 map->odom，5 s 后放弃，整组只激活一半。
        self._scan_wait = float(
            self.declare_parameter("scan_wait", 30.0).value)

        self._lock = threading.Lock()
        self._mode = {"mode": "", "state": "booting", "note": "启动中"}
        self._requests = queue.Queue()
        self._stop = threading.Event()

        # 服务客户端在构造时一次建好：worker 线程里 create_client 会与
        # executor 抢 node 的内部状态，宁可提前建。
        self._nav_manage = self.create_client(ManageLifecycleNodes,
                                              NAV_MANAGE_SERVICE)
        self._nav_get = self.create_client(GetState, NAV_PROBE_SERVICE)
        self._slam_get = self.create_client(GetState, SLAM_STATE_SERVICE)
        self._slam_change = self.create_client(ChangeState,
                                               SLAM_CHANGE_SERVICE)
        self._bt_get = self.create_client(GetState, "/bt_navigator/get_state")

        self._pub = self.create_publisher(String, Topics.MODE_STATE, 10)
        self._scan_at = 0.0          # 最近一次收到 /scan_slam 的时刻
        # 超时后仍在执行的请求：时间戳非 0 表示"上一次没等到结果"，
        # 反向切换前必须先等它落地，否则它会迟到把对端也激活起来。
        self._nav_late = None
        self._slam_late = None
        self.create_subscription(LaserScan, Topics.SCAN_SLAM,
                                 self._on_scan, qos_profile_sensor_data)

        self.create_subscription(String, Topics.NAV_CMD, self._on_cmd, 10)
        # 心跳：nav_bridge 是普通（volatile）订阅，App 重连后要能立刻知道
        # 当前模式，而不是等到下一次切换。
        self.create_timer(1.0 / max(state_hz, 0.1), self._publish)

        self._worker = threading.Thread(target=self._work, name="mode_worker",
                                        daemon=True)
        self._worker.start()
        self.get_logger().info("模式管理就绪，初始模式 %s" % self._initial)

    # ---------------------------------------------------------------- 输入

    def _on_scan(self, _msg):
        self._scan_at = time.monotonic()

    def _on_cmd(self, msg):
        """页面/命令行请求切模式。只认 nav/slam，其余忽略。"""
        try:
            command = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(command, dict) or command.get("cmd") != "mode":
            return
        mode = str(command.get("mode") or "")
        if mode not in MODES:
            self.get_logger().warn("未知模式：%r" % (mode,))
            return
        self._requests.put(mode)

    # ---------------------------------------------------------------- 状态

    def _set(self, mode, state, note=""):
        with self._lock:
            self._mode = {"mode": mode, "state": state, "note": note}
        self._publish()
        text = "模式 -> %s（%s）" % (mode, state)
        if note:
            text += " %s" % note
        self.get_logger().info(text)

    def _publish(self):
        with self._lock:
            payload = dict(self._mode)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False,
                              separators=(",", ":"))
        try:
            self._pub.publish(msg)
        except Exception:
            pass

    def _current(self):
        with self._lock:
            return dict(self._mode)

    # ---------------------------------------------------------------- 主循环

    def _work(self):
        """独立线程里做切换：服务调用要阻塞等结果，放在回调里会卡住 executor。"""
        pending = self._initial
        while not self._stop.is_set():
            if pending is not None:
                self._apply(pending)
                pending = None
            try:
                pending = self._requests.get(timeout=0.5)
            except queue.Empty:
                continue
            current = self._current()
            if pending == current["mode"] and current["state"] == "ready":
                pending = None          # 已经是这个模式，不重复切

    def _apply(self, mode):
        self._set(mode, "switching", "正在切换到%s…" % MODE_LABEL[mode])
        try:
            # 生命周期服务请求超时后可能仍在执行，必须等原请求完成。
            if self._nav_late is not None:
                if not self._wait(self._nav_late, 15.0):
                    raise RuntimeError("上一次导航生命周期操作仍未结束，暂不切换")
                self._nav_late = None
            if self._slam_late is not None:
                if not self._wait(self._slam_late, 15.0):
                    raise RuntimeError("上一次建图生命周期操作仍未结束，暂不切换")
                self._slam_late = None
            if mode == "slam":
                self._stop_nav()
                self._start_slam()
            else:
                # 先确认 slam 真的停了，再激活导航：两张地图之间没有坐标变换，
                # 任何"顺手把位姿搬过去"的做法都是错的（见文件头）。
                self._stop_slam()
                self._start_nav()
            self._set(mode, "ready",
                      # 固定起始位姿：没有 AMCL 要"确认"，位姿就是里程计位姿，
                      # 开机把车摆在原点即可。文案不再提示用户去点定位。
                      "导航就绪（固定起始位姿，车须在原点）" if mode == "nav"
                      else "建图中：沿用本次运行已有的地图；"
                           "要留给下次导航请先 save_map，"
                           "切回导航仍读 map_yaml")
        except Exception as exc:                    # 任何一步失败都只报状态
            self._set(mode, "error", str(exc))

    # ---------------------------------------------------------------- 导航组

    def _command_nav(self, command):
        if not self._nav_manage.wait_for_service(timeout_sec=5.0):
            return False
        request = ManageLifecycleNodes.Request()
        request.command = command
        future = self._nav_manage.call_async(request)
        if not self._wait(future, self._timeout):
            self._nav_late = future
            return False
        try:
            return bool(future.result().success)
        except Exception:
            return False

    def _nav_lifecycle(self, timeout):
        """返回 'absent'（节点没起）/ 'unknown'（服务在但不响应）/ 状态 id。"""
        if not self._nav_get.wait_for_service(timeout_sec=min(timeout, 3.0)):
            return "absent"
        state = self._wait_state(self._nav_get, timeout)
        return "unknown" if state is None else state

    def _stop_nav(self):
        state = self._nav_lifecycle(self._timeout)
        if state == "absent":
            return                          # 没起导航，没什么可停的
        if state == "unknown":
            raise RuntimeError("查不到导航组状态，拒绝切到建图")
        if state == STATE_FINALIZED:
            raise RuntimeError("导航节点已终结，需要重启容器")
        if state != STATE_ACTIVE:
            return                          # 已经不在跑，不用动
        # 部分节点可能本来就没 active，PAUSE 会返回 false；以复核状态为准。
        self._command_nav(ManageLifecycleNodes.Request.PAUSE)
        if self._nav_late is not None:
            raise RuntimeError("暂停导航请求仍未结束，暂不启动建图")
        after = self._nav_lifecycle(5.0)
        if after == STATE_ACTIVE:
            raise RuntimeError("导航组仍然 active，建图不能启动")
        if after == "unknown":
            raise RuntimeError("暂停后查不到导航组状态，拒绝启动建图")

    def _start_nav(self):
        state = self._nav_lifecycle(self._timeout)
        if state == "absent":
            raise RuntimeError(
                "导航组未启动（bringup 里 enable_nav 是否关掉了？）")
        if state == "unknown":
            raise RuntimeError("查不到导航组状态，拒绝激活")
        if state == STATE_FINALIZED:
            raise RuntimeError("导航节点已终结，需要重启容器")
        # 最后一道互斥：slam 还在 active 就绝不激活 AMCL。
        slam = self._slam_lifecycle(3.0)
        if slam == STATE_ACTIVE:
            raise RuntimeError("slam_toolbox 仍在运行，不能激活导航")
        if slam == "unknown":
            raise RuntimeError("查不到 slam_toolbox 状态，不能激活导航")
        if state == STATE_ACTIVE:
            # 半激活（amcl 在、bt_navigator 不在）时不能当成功。RESUME 对
            # 已 active 的节点会失败，没法自愈，只能如实报错。
            if self._bt_state() == STATE_ACTIVE:
                return
            raise RuntimeError("导航组半激活（controller_server 在、bt_navigator 不在），请重启应用")
        if not self._wait_for_scan():
            raise RuntimeError("等不到 /scan_slam，先不启动导航（雷达/中继未就绪）")
        command = (ManageLifecycleNodes.Request.STARTUP
                   if state == STATE_UNCONFIGURED
                   else ManageLifecycleNodes.Request.RESUME)
        if not self._command_nav(command):
            raise RuntimeError("启动导航组失败（lifecycle_manager 是否已退出？）")
        if self._nav_lifecycle(5.0) != STATE_ACTIVE:
            raise RuntimeError("导航组没有进入 active")
        # 迟到的生命周期操作兜底：上一次 STARTUP 可能超时后才真正完成。
        # 等一拍再确认没有别人把 slam 激活起来，有就立刻撤掉并报错。
        time.sleep(2.0)
        if self._slam_lifecycle(3.0) == STATE_ACTIVE:
            self._change_slam(Transition.TRANSITION_DEACTIVATE, "deactivate")
            if self._slam_lifecycle(3.0) == STATE_ACTIVE:
                raise RuntimeError("slam 仍 active 且撤不下来，请重启容器")
            raise RuntimeError("激活导航后 slam 变成 active，已撤下，请重试")

    # ---------------------------------------------------------------- 建图组

    def _change_slam(self, transition, label):
        if not self._slam_change.wait_for_service(timeout_sec=5.0):
            return False
        request = ChangeState.Request()
        request.transition = Transition()
        request.transition.id = transition
        request.transition.label = label
        future = self._slam_change.call_async(request)
        if not self._wait(future, self._timeout):
            self._slam_late = future
            return False
        try:
            return bool(future.result().success)
        except Exception:
            return False

    def _slam_lifecycle(self, timeout):
        """返回 'absent' / 'unknown' / 状态 id。服务在但查不到 = unknown。"""
        if not self._slam_get.wait_for_service(timeout_sec=min(timeout, 3.0)):
            return "absent"
        state = self._wait_state(self._slam_get, timeout)
        return "unknown" if state is None else state

    def _stop_slam(self):
        state = self._slam_lifecycle(5.0)
        if state == "absent":
            return                              # 建图节点没起，没什么可停的
        if state == "unknown":
            # 以前这里当成"没起来"直接放行，于是 slam 还在发 map->odom 时
            # 导航也被激活，两个发布者同时写同一个 TF。
            raise RuntimeError("查不到 slam_toolbox 状态，拒绝激活导航")
        if state == STATE_FINALIZED:
            raise RuntimeError("建图节点已终结，需要重启容器")
        if state != STATE_ACTIVE:
            return                              # 已 inactive，TF 不再发布
        # 只 deactivate、不 cleanup：保持在 configure 过的状态，切回来
        # 只差一次 activate，1 s 内完成。
        if not self._change_slam(Transition.TRANSITION_DEACTIVATE,
                                 "deactivate"):
            raise RuntimeError("暂停建图节点失败")
        if self._slam_lifecycle(5.0) == STATE_ACTIVE:
            raise RuntimeError("slam_toolbox 仍 active，导航不能启动")

    def _start_slam(self):
        state = self._slam_lifecycle(self._timeout)
        if state == "absent":
            raise RuntimeError(
                "slam_toolbox 未启动（bringup 里 enable_slam 是否关掉了？）")
        if state == "unknown":
            raise RuntimeError("查不到 slam_toolbox 状态，拒绝激活")
        if state == STATE_FINALIZED:
            raise RuntimeError("建图节点已终结，需要重启容器")
        if self._nav_lifecycle(3.0) == STATE_ACTIVE:
            raise RuntimeError("导航组仍在 active，不能激活建图")
        if state == STATE_UNCONFIGURED:
            if not self._change_slam(Transition.TRANSITION_CONFIGURE,
                                     "configure"):
                raise RuntimeError("配置建图节点失败")
            state = self._wait_state(self._slam_get, 10.0)
        if state == STATE_INACTIVE:
            if not self._change_slam(Transition.TRANSITION_ACTIVATE,
                                     "activate"):
                raise RuntimeError("激活建图节点失败")
        # 复验：不能"发了 activate 就当成功"
        if self._slam_lifecycle(5.0) != STATE_ACTIVE:
            raise RuntimeError("建图节点没有进入 active")
        # 同上：之前超时的导航 STARTUP 可能现在才落地。看到就暂停它。
        time.sleep(2.0)
        if self._nav_lifecycle(3.0) == STATE_ACTIVE:
            self._command_nav(ManageLifecycleNodes.Request.PAUSE)
            if self._nav_lifecycle(3.0) == STATE_ACTIVE:
                raise RuntimeError("导航仍 active 且停不下来，请重启容器")
            raise RuntimeError("激活建图后导航仍变成 active，已暂停，请重试")

    # ---------------------------------------------------------------- 工具

    def _wait(self, future, timeout):
        """等一个 async 服务的 future，超时返回 False（future 不取消）。"""
        deadline = time.monotonic() + timeout
        while not future.done():
            if self._stop.is_set() or time.monotonic() > deadline:
                return False
            time.sleep(0.02)
        return True

    def _bt_state(self):
        """bt_navigator 的当前状态（LM 最后激活它，用它判断整组是否齐全）。"""
        try:
            future = self._bt_get.call_async(_GetStateRequest())
            if not self._wait(future, 3.0):
                return None
            return int(future.result().current_state.id)
        except Exception:
            return None

    def _wait_for_scan(self):
        """等到一条"本次等待之后"收到的 /scan_slam。"""
        started = time.monotonic()
        deadline = started + self._scan_wait
        while time.monotonic() < deadline and not self._stop.is_set():
            if self._scan_at >= started:
                return True
            time.sleep(0.1)
        return self._scan_at >= started

    def _wait_state(self, client, timeout):
        """轮询 GetState 直到服务可用并返回状态 id；拿不到返回 None。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not self._stop.is_set():
            if not client.wait_for_service(timeout_sec=0.5):
                continue
            future = client.call_async(GetState.Request())
            if not self._wait(future, 2.0):
                continue
            try:
                return int(future.result().current_state.id)
            except Exception:
                continue
        return None

    def destroy_node(self):
        self._stop.set()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        super().destroy_node()


def main():
    """
    @description         : 节点入口（rclpy 生命周期见 common/runtime.py）
    @return              : None
    """
    run(ModeManager)


if __name__ == "__main__":
    main()
