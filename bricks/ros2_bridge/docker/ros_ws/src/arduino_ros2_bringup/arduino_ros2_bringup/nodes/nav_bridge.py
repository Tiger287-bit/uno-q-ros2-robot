"""导航桥：把 Nav2 折成一条 JSON 流给 App，并代发导航目标。

为什么需要这一层
    网页不直连 rosbridge（9090 没有鉴权，也没映射到宿主机），App 只会
    "订阅几个话题、发几帧 JSON"。而导航需要三样东西：动作客户端
    （NavigateToPose）、transient_local 的 /map（否则晚订阅拿不到锁存帧）、
    以及 AMCL 的 /amcl_pose。把这些塞进 App 那个只管速度的进程里，
    等于给它加一套动作状态机。所以放在容器内：本节点只说 ROS，
    App 只说 JSON。

对外接口（都在 common/topics.py 里登记过）
    /app/nav_cmd   std_msgs/String
        {"cmd":"goal","x":1.2,"y":-0.3,"yaw":1.57}   下发目标
        {"cmd":"cancel"}                             取消当前目标
        {"cmd":"pose","x":0,"y":0,"yaw":0}           重设初始位姿（发 /initialpose）
        {"cmd":"map"}                                立刻重发一次地图
    /app/nav_ui    std_msgs/String
        {"pose":[x,y,yaw],"age":0.1,"nav":"navigating","dist":1.23,
         "path":[[x,y],...],"map":{"w":193,"h":102,"res":0.05,
         "ox":-0.976,"oy":-0.645,"seq":1,"data":"<base64>"}}

地图为什么用 base64 而不是数组
    193x102 = 19686 格。JSON 数组是 5 万多个字符，base64 只要 2.6 万；
    而且 App 与浏览器都只要"原样搬"，不需要中间解释。每格一字节：
    0=空闲、1=占据、2=未知。

人手接管就取消导航
    mux 把当前控制权归属发在 /cmd_vel_source 上。导航中一旦变成手柄或网页，
    说明人在开车，这里直接 cancel —— 否则人一松手，Nav2 会把车拽回原目标。

取消的三个时间点都要覆盖
    1. 目标已受理：调 cancel_goal_async；
    2. 目标已发出、动作服务器还没回：此刻没有 goal handle，只能记下
       "受理后立刻撤"，否则取消动作丢失，车随后照样开出去；
    3. 连目标都还没发出去（App 侧的发送队列里）：由 navlink 的队列保证
       cancel 一定在 goal 之后发出，见 python/navlink.py。

模式
    导航与建图互斥（都发 map->odom），实际切换由 mode_manager 负责。
    本节点订阅 /app/mode_state，只做两件事：把模式带进 JSON 给页面；
    切离导航时先撤掉目标。
"""

import base64
import json
import math
import time

import rclpy.time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from arduino_ros2_bringup.common.runtime import run
from arduino_ros2_bringup.common.topics import Topics

# 与 nav2.yaml 的 map_server 保持一致；地图坐标系固定叫 map。
MAP_FRAME = "map"
# 输出节奏。10 Hz 对网页上的机器人图标足够顺滑，又不会把 rosbridge 压满。
UI_HZ = 10.0
# 每多少帧带一次地图。App 与网页都要"晚连上也能拿到图"，所以周期性重发比
# 只在变化时发更稳；2 s 一次、约 26 KB，局域网可忽略。
MAP_EVERY_FRAMES = 20
# 路径抽稀上限。全局路径在这个尺度上几十个点就够画，原样转发会让每帧 JSON
# 从几百字节涨到几 KB。
PATH_MAX_POINTS = 120
# 重设位姿后等多久算失败。AMCL 2 Hz 更新 + TF 20 Hz，2 s 足够。
POSE_ACK_TIMEOUT_S = 4.0
# 固定起始位姿模式：去掉 AMCL 之后 map->odom 是一条静态变换（nav.launch.py
# 里的 static_transform_publisher，恒为单位变换），机器人的位姿就等于它的
# 里程计位姿。这样每次开机都认为"车在地图原点 (0,0,0)"，不再做扫描匹配，
# 也就不会有定位跳变、TF 外推这些毛病。代价是：开机时必须把车摆在原点、
# 朝向 +x，而且中途被抱走之后位置就再也对不上了（只能重启应用）。
FIXED_START_POSE = True
# 自动速度屏蔽的最长持续时间。正常情况下取消 1 s 内就结束，够宽；
# 万一某条路径忘了收尾，宁可恢复自动驾驶也不能让车永远不动。
# 人工来源：这两个话题一旦成为 mux 的持有者，视为人在开车。
MANUAL_SOURCES = (Topics.CMD_VEL_JOY, Topics.CMD_VEL_WEB)


def yaw_to_quat(yaw):
    """偏航角转四元数，返回 (x, y, z, w)。"""
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def quat_to_yaw(q):
    """四元数转偏航角（弧度）。只关心绕 z 轴的旋转。"""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class NavBridge(Node):
    """Nav2 与 App 之间的双向桥。"""

    def __init__(self):
        super().__init__("nav_bridge")
        self._pose = None            # [x, y, yaw]
        self._pose_age = 99.0        # 位姿数据的新鲜度，秒
        self._pose_at = 0.0          # 最近一次查到位姿的本地时刻（monotonic）
        # 固定起始位姿下没有"等 AMCL 响应"这回事，一上来就是已确认。
        self._pose_ack = "ok" if FIXED_START_POSE else ""
        self._pose_target_stamp = 0.0
        self._mapodom_at = None      # 请求那一刻的 map→odom，用来判断 AMCL 动没动
        self._pose_target = None     # 等待确认的 (x, y, yaw)
        self._pose_target_at = 0.0
        self._path = []
        self._map_json = None        # 已编码好的地图元数据，None 表示还没有
        self._map_bytes = ""         # base64 后的栅格，跟着 _map_json 一起有效
        self._grid = None            # 原始栅格（0 空闲 / 1 占据 / 2 未知），供目标预检
        self._map_seq = 0
        self._map_dirty = True
        self._frame = 0

        self._nav = "idle"           # idle / navigating / succeeded / failed / canceled
        self._nav_note = ""
        self._notice = ""            # 一次性提示（例如目标被拒），不影响 nav 状态
        self._distance = None
        self._goal_handle = None
        self._goal_gen = 0           # 目标代次：旧目标的回调据此作废
        self._goal_pending = False   # 已发出、还没收到受理结果
        self._cancel_wanted = False  # 受理前就要求取消
        self._cancel_at = 0.0        # 最近一次发 cancel_goal_async 的时刻
        self._cancel_tries = 0       # 重试次数，超过就报失败
        self._hold = False           # 是否禁止 mux 放行自动速度
        self._hold_at = 0.0
        self._pose_req = 0           # 网页给这次"重设位姿"的编号，回执原样带回
        self._mode = {"mode": "", "state": "", "note": ""}

        # 位姿走 TF（map -> base_footprint）而不是等 /amcl_pose：
        # AMCL 只在"动够了 2 cm / 0.05 rad"时才更新并发布一次位姿。车停着不动
        # 时 /amcl_pose 会长时间静默，页面就会一直显示"定位超时"，而实际上
        # 位姿是有效的。TF 链一直在，查一次就知道当前位姿与它的时间戳。
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pub_ui = self.create_publisher(String, Topics.NAV_UI, 10)
        self._pub_initial = self.create_publisher(
            PoseWithCovarianceStamped, Topics.INITIAL_POSE, 10)
        # 取消 / 重设位姿时直接给 Nav2 的输出口补一帧 0：控制器撤目标后
        # 并不保证再发一帧，mux 会按 hold_s 把最后一帧速度再保持 0.25 s。
        self._pub_nav = self.create_publisher(Twist, Topics.CMD_VEL_NAV, 10)
        # 取消期间/旧目标没收尾时，让 mux 别放行自动速度。mux 自己只看得到
        # "谁在发"，看不到"目标有没有真的停"，必须由这里明确告诉它。
        self._pub_hold = self.create_publisher(Bool, Topics.NAV_HOLD, 10)
        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # /map 是 transient_local + reliable：普通 volatile 订阅端在 map_server
        # 已经发完之后才连上来时，永远收不到那一帧。
        map_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, Topics.MAP, self._on_map, map_qos)
        self.create_subscription(Path, Topics.PLAN, self._on_plan, 10)
        self.create_subscription(String, Topics.CMD_VEL_SOURCE,
                                 self._on_source, 10)
        self.create_subscription(String, Topics.NAV_CMD, self._on_cmd, 10)
        self.create_subscription(String, Topics.MODE_STATE, self._on_mode, 10)

        self.create_timer(1.0 / UI_HZ, self._publish_ui)
        self.get_logger().info(
            "导航桥就绪：%s -> %s，%.0f Hz；目标走 navigate_to_pose 动作"
            % (Topics.NAV_CMD, Topics.NAV_UI, UI_HZ))

    # ---------------------------------------------------------------- 输入

    def _on_map(self, msg):
        """/map 变化时重新编码。地图是静态的，正常只会进来一两次。"""
        width, height = msg.info.width, msg.info.height
        if width <= 0 or height <= 0 or len(msg.data) < width * height:
            return
        cells = bytearray(width * height)
        for index in range(width * height):
            value = msg.data[index]
            cells[index] = 2 if value < 0 else (1 if value >= 50 else 0)
        self._map_json = {
            "w": width,
            "h": height,
            "res": round(msg.info.resolution, 4),
            "ox": round(msg.info.origin.position.x, 4),
            "oy": round(msg.info.origin.position.y, 4),
        }
        self._map_bytes = base64.b64encode(bytes(cells)).decode("ascii")
        self._grid = bytes(cells)
        self._map_seq += 1
        self._map_dirty = True

    def _on_plan(self, msg):
        points = list(msg.poses)
        if not points:
            self._path = []
            return
        step = max(1, len(points) // PATH_MAX_POINTS)
        self._path = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)]
                      for p in points[::step]]

    def _on_source(self, msg):
        """mux 的控制权归属变了：人工接管就取消导航。"""
        active = self._goal_handle is not None or self._goal_pending
        if (msg.data in MANUAL_SOURCES and active
                and not self._hold_needed()):
            self.get_logger().info("检测到人工接管（%s），取消导航目标" % msg.data)
            self._cancel("manual")

    def _on_mode(self, msg):
        """模式变了：切离导航时先撤目标，别让车在导航栈关掉前继续跑。"""
        try:
            info = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(info, dict):
            return
        previous = self._mode.get("mode", "")
        self._mode = {"mode": str(info.get("mode", "")),
                      "state": str(info.get("state", "")),
                      "note": str(info.get("note", ""))}
        if self._mode["mode"] != "nav" and self._nav == "navigating":
            self._cancel("mode")
        # 从建图切回导航：AMCL 加载的是旧地图，slam 这次的坐标跟它无关，
        # 不能沿用。标记成"需要重设"，页面会提示用户点一下车的实际位置。
        if self._mode["mode"] == "nav" and previous and previous != "nav":
            if FIXED_START_POSE:
                # 固定位姿：坐标就是里程计坐标，切模式不改变它。
                self._pose_ack = "ok"
                self._pose_target = None
                self.get_logger().info("切回导航：固定起始位姿，无需重设")
            else:
                self._pose_ack = "needed"
                self._pose_target = None
                self.get_logger().info("切回导航：请重设初始位姿（两张地图之间没有变换）")

    def _on_cmd(self, msg):
        """App 发来的 JSON 命令。任何解析错误只丢这一条，不能把节点带走。"""
        try:
            command = json.loads(msg.data)
        except ValueError:
            self.get_logger().warn("导航命令不是合法 JSON：%r" % msg.data[:80])
            return
        if not isinstance(command, dict):
            return
        action = command.get("cmd")
        if action == "goal":
            # 没给朝向就传 None，由 _send_goal 按"车 -> 目标"的方向补上。
            # 以前默认 0：点到东边却要求朝 +x，车到了还要把姿态扭过去，而原地
            # 转向已关（只能画弧），于是看起来"到点了还在走"。
            raw_yaw = command.get("yaw")
            self._send_goal(float(command.get("x", 0.0)),
                            float(command.get("y", 0.0)),
                            None if raw_yaw is None else float(raw_yaw))
        elif action == "cancel":
            self._cancel(str(command.get("reason") or "user"))
        elif action == "pose":
            request = command.get("req", 0)
            # 字符串包含页面会话，整数兼容旧客户端；回执原样返回。
            self._pose_req = (request if type(request) in (str, int)
                              and len(str(request)) <= 128 else 0)
            self._set_initial_pose(float(command.get("x", 0.0)),
                                   float(command.get("y", 0.0)),
                                   float(command.get("yaw", 0.0)))
        elif action == "map":
            self._map_dirty = True
        elif action == "mode":
            # 同一条 /app/nav_cmd 上还有 mode_manager 在听；这里只是不报"未知命令"。
            pass
        else:
            self.get_logger().warn("未知导航命令：%r" % (action,))

    # ---------------------------------------------------------------- 动作

    def _goal_allowed(self):
        """能不能受理导航目标：导航模式就绪 + 当前定位有效且已确认。"""
        if self._mode.get("mode") not in ("", "nav"):
            return False
        if self._mode.get("state") != "ready":
            return False
        if self._pose is None or self._pose_age > 1.0:
            return False
        return self._pose_ack not in ("waiting", "timeout", "needed")

    def _cell_state(self, x, y):
        """返回坐标所在栅格：0 空闲 / 1 占据 / 2 未知；出界或没地图时 None。"""
        meta, grid = self._map_json, self._grid
        if not meta or grid is None:
            return None
        res = meta["res"] or 0.05
        cx = int((x - meta["ox"]) / res)
        cy = int((y - meta["oy"]) / res)
        if cx < 0 or cy < 0 or cx >= meta["w"] or cy >= meta["h"]:
            return None
        return grid[cy * meta["w"] + cx]

    def _reject_goal(self, note):
        """目标被预检拦下：只挂提示，不改写正在跑的目标。"""
        self.get_logger().warn("拒绝目标：%s" % note)
        if self._goal_handle is not None or self._goal_pending:
            self._notice = note
        else:
            self._set_state("failed", note)

    def _send_goal(self, x, y, yaw):
        if yaw is None:
            # 按行进方向定朝向：车头顺着走的方向到点，不必再做姿态调整。
            if self._pose is not None:
                yaw = math.atan2(y - self._pose[1], x - self._pose[0])
            else:
                yaw = 0.0
        if self._goal_handle is not None or self._goal_pending or self._hold_needed():
            self._notice = "请先取消当前目标，并等待停车确认"
            return
        if not self._goal_allowed():
            note = "定位未确认，先重设初始位姿"
            self.get_logger().warn("拒绝目标：%s" % note)
            # 正在跑的目标不能被这条拒绝改写成 failed —— 一旦改了，
            # _on_source 里"人工接管才取消"的判断就会跳过，旧目标反而撤不掉。
            if self._goal_handle is not None or self._goal_pending:
                self._notice = note
            else:
                self._set_state("failed", note)
            return

        # 落点预检。全局代价地图是 track_unknown_space=true + allow_unknown=false，
        # 黑格（占据）和灰格（未扫描）都当墙。不预检的话 Nav2 会先受理、规划失败，
        # 再跑完一整套恢复动作（约 25 s）才回 status=6，页面上只有一句"导航失败"，
        # 看不出是点歪了还是车坏了。起点也要查：起点压在黑/灰格里规划必然失败，
        # 而那种情况真正的原因是定位偏了，必须明确提示重设位姿。
        target_state = self._cell_state(x, y)
        if target_state in (1, 2):
            self._reject_goal("目标点在障碍物上，换一个位置" if target_state == 1
                              else "目标点在未扫描区域（灰区），车不能进")
            return
        if self._pose is not None:
            start_state = self._cell_state(self._pose[0], self._pose[1])
            if start_state in (1, 2):
                self._reject_goal("车当前定位落在%s，请重设初始位姿"
                                  % ("障碍物里" if start_state == 1 else "未扫描区域"))
                return

        self._notice = ""
        if not self._client.server_is_ready() and not self._client.wait_for_server(timeout_sec=2.0):
            self._set_state("failed", "导航服务未就绪")
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = MAP_FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quat(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        self.get_logger().info("导航目标：x=%.2f y=%.2f yaw=%.2f" % (x, y, yaw))
        self._goal_gen += 1
        generation = self._goal_gen
        self._cancel_wanted = False
        self._goal_pending = True
        self._set_state("navigating", "")
        future = self._client.send_goal_async(goal,
                                              feedback_callback=self._on_feedback)
        # 代次随回调带下去：取消/新目标之后到达的旧响应必须作废，
        # 否则一条已经被用户撤销的目标会在这里被"救活"并开始执行。
        future.add_done_callback(
            lambda done, generation=generation: self._on_goal_response(
                done, generation))

    def _on_goal_response(self, future, generation):
        try:
            handle = future.result()
        except Exception as exc:                      # 动作调用本身失败
            if generation == self._goal_gen:
                self._goal_pending = False
                self._set_state("failed", "目标下发失败：%s" % exc)
            return
        if generation != self._goal_gen:
            # 期间又下发了新目标或取消了：这条迟到的响应直接撤掉。
            if handle.accepted:
                handle.cancel_goal_async()
            return
        self._goal_pending = False
        if not handle.accepted:
            canceled = self._cancel_wanted
            self._cancel_wanted = False
            self._set_state("canceled" if canceled else "failed",
                            "已取消" if canceled else "目标被拒绝")
            return
        if self._cancel_wanted:
            # 取消发生在受理之前：这一刻才拿到 handle，立刻撤。
            # 必须把 handle 存下来并挂结果回调：只有结果回来才算"旧目标真的
            # 结束了"。以前这里只发 cancel 就 return，hold 永远等不到解除，
            # mux 从此不放行自动速度 —— 表现为导航再也不动，且没有任何报错。
            self._cancel_wanted = False
            self._goal_handle = handle
            self._cancel_tries = 0
            self._distance = None
            self._set_state("canceling", "已取消")
            self._request_cancel()
            handle.get_result_async().add_done_callback(
                lambda done, generation=generation: self._on_result(
                    done, generation))
            return
        self._goal_handle = handle
        # 代次随回调带下去：旧目标的结局不能动新目标的状态，
        # 尤其不能把新 handle 清空 —— 清空后"取消"就没有对象了。
        handle.get_result_async().add_done_callback(
            lambda done, generation=generation: self._on_result(
                done, generation))

    def _on_feedback(self, feedback_msg):
        self._distance = round(float(feedback_msg.feedback.distance_remaining), 2)

    def _on_result(self, future, generation):
        if generation != self._goal_gen:
            # 这是被取代的旧目标回来了：既不能清 handle，也不能改状态
            self.get_logger().info("旧目标的结果已忽略（代次 %d）" % generation)
            return
        try:
            status = future.result().status
        except Exception as exc:
            self._cancel_tries = max(1, self._cancel_tries)
            self._set_state("failed", "无法确认目标已结束：%s" % exc)
            return
        if status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
                          GoalStatus.STATUS_ABORTED):
            self._cancel_tries = max(1, self._cancel_tries)
            self._set_state("failed", "尚未确认目标结束（status=%d）" % status)
            return
        self._goal_handle = None
        self._cancel_tries = 0
        self._distance = None
        # 目标结束后必须把路径清掉：以前只在收到空 /plan 时才清，而规划失败或
        # 取消时根本不会再来 /plan，页面就一直画着上一个目标的路线。用户点了
        # 一个近点，看到的却是几米外的旧路径，自然会以为"规划器绕了一大圈"。
        self._path = []
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._set_state("succeeded", "已到达")
        elif status == GoalStatus.STATUS_CANCELED:
            self._set_state("canceled", "已取消")
        else:
            self._set_state("failed", "导航失败（status=%d）" % status)

    def _cancel(self, reason):
        note = {"user": "已取消", "manual": "人工接管",
                "pose": "重设位姿", "mode": "切换模式",
                "fault": "MCU 故障，已取消导航"}.get(reason, "已取消")
        # 取消后这条路径立刻作废：页面继续画着它会被当成"当前规划结果"。
        self._path = []
        self._stop_base()
        if self._goal_pending:
            # 目标已发出但动作服务器还没受理：此刻没有 handle 可撤，
            # 只能标记，等 _on_goal_response 拿到 handle 的瞬间再撤。
            self._cancel_wanted = True
            self._distance = None
            self._set_state("canceling", note)
            return
        if self._goal_handle is None:
            if self._nav == "navigating":
                self._set_state("canceled", note)
            return
        # 句柄保留到结果回来。以前这里立刻清掉句柄：一旦 Nav2 拒绝取消，
        # 就再也没对象可撤，页面却已经显示"已取消"，旧目标继续输出速度。
        if self._nav == "canceling":
            return
        self._distance = None
        self._cancel_tries = 0
        self._set_state("canceling", note)
        self._request_cancel()

    def _request_cancel(self):
        handle = self._goal_handle
        if handle is None:
            return
        self._cancel_at = time.monotonic()
        self._cancel_tries += 1
        tries = self._cancel_tries
        future = handle.cancel_goal_async()
        future.add_done_callback(
            lambda done, tries=tries: self._on_cancel_response(done, tries))

    def _on_cancel_response(self, future, tries):
        try:
            response = future.result()
        except Exception as exc:
            response = None
            self.get_logger().warn("取消请求失败：%s" % exc)
        if response is not None and response.goals_canceling:
            self.get_logger().info("取消已被接受，等目标结束（第 %d 次）" % tries)
            return
        self.get_logger().warn("取消未被接受（第 %d 次），1 s 后重试" % tries)

    def _hold_needed(self):
        """是否需要禁止 mux 放行自动速度。

        判据只有两条，都直接对应"目标还没收尾"：
          - `_cancel_wanted`：目标已发出、还没拿到 handle，但用户已要求撤；
          - `_goal_handle` 存在且 `_cancel_tries > 0`：正在走取消流程。
        取消成功时 `_on_result` 会清 handle、清计数，这里自动变 false。
        """
        if self._cancel_wanted:
            return True
        return self._goal_handle is not None and self._cancel_tries > 0

    def _set_hold(self, value):
        """告诉 mux：现在禁止自动速度。true 期间人工以外一律输出 0。"""
        now = time.monotonic()
        if value == self._hold and now - self._hold_at < 0.4:
            return
        self._hold = bool(value)
        self._hold_at = now
        try:
            message = Bool()
            message.data = self._hold
            self._pub_hold.publish(message)
        except Exception:
            pass

    def _stop_base(self):
        """给 Nav2 的输出口补两帧零速度，别等 mux 的 hold_s 超时。"""
        try:
            self._pub_nav.publish(Twist())
            self._pub_nav.publish(Twist())
        except Exception:
            pass

    def _odom_stamp(self):
        """返回最新 odom->base_footprint 的 TF 时间戳。

        AMCL 收到 /initialpose 会拿 header.stamp 去查 odom->base_footprint。
        轮式里程计只有 5 Hz，用"此刻"必然落在最新 TF 之后，报
        "Failed to transform initial pose in time"（位姿仍会生效，但日志
        看着像失败）。

        取最新 TF 的时间戳还不够：TF 到达本节点与到达 AMCL 的时序不一致，
        实测 AMCL 的 buffer 会落后几十毫秒，仍报同样的告警。所以在最新
        TF 的时刻上再回退 0.1 s —— 5 Hz 下这个时刻一定已有样本，且落在
        buffer 的缓存窗口内。
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                "odom", "base_footprint", rclpy.time.Time())
            stamp = rclpy.time.Time.from_msg(tf.header.stamp)
            if stamp.nanoseconds > 0:
                return stamp - Duration(seconds=0.1)
        except Exception:
            pass
        return self.get_clock().now() - Duration(seconds=0.3)

    def _set_initial_pose(self, x, y, yaw):
        # 位姿一改，旧目标的前提就没了：先撤目标并停车，否则 Nav2 会拿着
        # 新位姿继续追旧目标点（P2：导航中重设位姿，车会自己接着跑）。
        self._cancel("pose")
        # 固定起始位姿下 /initialpose 由 start_pose_tf 接：它把 map->odom
        # 挪到这个位姿，等于"告诉系统车其实在这儿"。没有 AMCL 要做扫描匹配，
        # 所以不用等 map->odom 变化，直接算已确认。
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = MAP_FRAME
        msg.header.stamp = self._odom_stamp().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quat(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # 对角线给一个不大不小的方差：AMCL 会把粒子撒在附近再收敛。
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        # 必须在发布前保存基线；否则 None 会让每次检查都误判为新变换。
        self._mapodom_at = self._mapodom_pose()
        self._pub_initial.publish(msg)
        self._pose_target = (x, y, yaw)

        self._pose_target_at = time.monotonic()
        self._pose_target_stamp = self.get_clock().now().nanoseconds * 1e-9
        self._pose_ack = "ok" if FIXED_START_POSE else "waiting"
        if FIXED_START_POSE:
            self._notice = "起点已对齐"
        self.get_logger().info("重设初始位姿：x=%.2f y=%.2f yaw=%.2f" % (x, y, yaw))

    # ---------------------------------------------------------------- 输出

    def _set_state(self, state, note=""):
        if (state, note) == (self._nav, self._nav_note):
            return
        self._nav = state
        self._nav_note = note
        if note:
            self.get_logger().info("导航状态：%s（%s）" % (state, note))

    def _mapodom_pose(self):
        """map→odom 的当前值 (x, y, yaw)；查不到返回 None。"""
        try:
            tf = self._tf_buffer.lookup_transform(MAP_FRAME, "odom",
                                                  rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        return (t.x, t.y, quat_to_yaw(tf.transform.rotation))

    def _lookup_pose(self):
        """查一次 map->base_footprint，返回 (位姿, 数据年龄秒) 或 None。"""
        try:
            tf = self._tf_buffer.lookup_transform(MAP_FRAME, "base_footprint",
                                                  rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        pose = [round(t.x, 3), round(t.y, 3),
                round(quat_to_yaw(tf.transform.rotation), 3)]
        age = max(0.0, now - stamp)
        self._check_pose_ack(pose, age)
        return pose, age

    def _check_pose_ack(self, pose, age=0.0):
        """判断 AMCL 是否已经把位姿搬到我们要求的那个位置。"""
        if self._pose_target is None or age > 1.0:
            return
        # 旧位姿落在容差里也会"命中"。真正的证据是 map→odom 这个变换本身
        # 变了 —— 时间戳不行（AMCL 每个周期都用新时间戳重播旧值），
        # /amcl_pose 的采集时间也不行（静止时用旧扫描的戳，永远早于请求）。
        current = self._mapodom_pose()
        if current is None:
            return
        if self._mapodom_at is not None:
            dx = current[0] - self._mapodom_at[0]
            dy = current[1] - self._mapodom_at[1]
            dyaw = abs(math.atan2(math.sin(current[2] - self._mapodom_at[2]),
                                  math.cos(current[2] - self._mapodom_at[2])))
            moved = math.hypot(dx, dy) > 0.01 or dyaw > 0.01
        else:
            moved = True            # 请求时还没有 map→odom，现在有了也算新
        target = self._pose_target
        distance = math.hypot(pose[0] - target[0], pose[1] - target[1])
        yaw_error = abs(math.atan2(math.sin(pose[2] - target[2]),
                                   math.cos(pose[2] - target[2])))
        # moved：AMCL 真的重算过；already：请求时就已经在这个位置上。
        # 只有这两种情况才给 ok，避免"旧位姿落在容差里"被当成确认。
        already = distance <= 0.05 and yaw_error <= 0.10
        if (moved or already) and distance <= 0.25 and yaw_error <= 0.5:
            self._pose_ack = "ok"
            self._pose_target = None
            # 之前因为"定位未确认"被拒的目标，失败提示到这里就过期了。
            if self._nav == "failed" and "定位未确认" in self._nav_note:
                self._set_state("idle", "")
            self.get_logger().info(
                "定位已确认：x=%.2f y=%.2f yaw=%.2f" % tuple(pose))

    def _publish_ui(self):
        """10 Hz 推一帧给 App。地图按周期或 dirty 标记附带。"""
        self._frame += 1
        # 取消重试：Nav2 拒绝或丢包时补发，最多 3 次；仍不行就报失败，
        # 但 hold 不回退 —— 宁可不动，也不能让旧目标把车带走。
        if (self._nav == "canceling" and self._goal_handle is not None
                and time.monotonic() - self._cancel_at > 1.0):
            if self._cancel_tries >= 3:
                self._set_state("failed", "取消未被接受，请再点一次或急停")
            else:
                self._request_cancel()
        # 屏蔽由状态推导，不靠各条路径手动开关。以前每个分支各写一次
        # _set_hold，漏掉一条（受理前取消、切换模式时动作服务器不回结果）
        # 就会永久停在 true，mux 从此不放行自动速度 —— 导航能规划、能下发，
        # 车就是不动，而且日志里没有任何异常。
        self._set_hold(self._hold_needed())

        # 超时判断必须每帧都跑：查不到 TF 时 _lookup_pose 直接返回 None，
        # 放在里面就会一直停在 waiting（离线复现：20 s 还在等）。
        if (self._pose_target is not None
                and time.monotonic() - self._pose_target_at > POSE_ACK_TIMEOUT_S):
            self._pose_ack = "timeout"
            self._pose_target = None
            self.get_logger().warn("重设初始位姿后 %.0f s 未确认（TF 里没有新位姿）"
                                   % POSE_ACK_TIMEOUT_S)

        measured = self._lookup_pose()
        if measured is not None:
            self._pose, self._pose_age = measured
            self._pose_at = time.monotonic()
        elif self._pose is not None:
            # 查不到 TF（AMCL 被关掉、TF 断流）时帧龄必须继续涨，
            # 否则页面永远显示"0.14 s 前"，图标一直是实心的。
            self._pose_age = time.monotonic() - self._pose_at

        payload = {
            "nav": self._nav,
            "note": self._nav_note,
            "notice": self._notice,
            "path": self._path,
            "mode": self._mode,
            "pose_ack": self._pose_ack,
            "pose_req": self._pose_req,
        }
        if self._pose is not None:
            payload["pose"] = self._pose
            payload["age"] = round(self._pose_age, 2)
        if self._distance is not None:
            payload["dist"] = self._distance

        include_map = self._map_json is not None and (
            self._map_dirty or self._frame % MAP_EVERY_FRAMES == 0)
        if include_map:
            grid = dict(self._map_json)
            grid["seq"] = self._map_seq
            grid["data"] = self._map_bytes
            payload["map"] = grid
            self._map_dirty = False

        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self._pub_ui.publish(msg)


def main():
    """
    @description         : 节点入口（rclpy 生命周期见 common/runtime.py）
    @return              : None
    """
    run(NavBridge)


if __name__ == "__main__":
    main()
