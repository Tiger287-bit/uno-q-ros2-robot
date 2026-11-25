"""固定起始位姿：用一条可调的 map->odom 代替 AMCL。

为什么不用 static_transform_publisher
    静态变换发布出去就改不了。而"每次开机从地图原点开始"要求车必须精确摆在
    原点 —— 摆偏十几厘米，整张地图就跟着偏，规划器会把墙当成走廊，车一开
    就顶住。这里改成订阅 /initialpose：在页面上点一下车的真实位置，映射立刻
    对齐。没有扫描匹配，也就没有定位跳变、TF 外推、粒子收敛这些问题。

变换方向
    map->odom = (x, y, yaw) 表示"里程计原点在地图里的位姿"。开机时里程计是
    (0,0,0)，所以这条变换同时就是"车在地图里的位姿"。
"""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

MAP_FRAME = "map"
ODOM_FRAME = "odom"


def yaw_from_quat(q):
    """四元数取偏航角。只用 z/w 分量，车体是平面运动。"""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class StartPoseTf(Node):
    """把 map->odom 广播成一条可被 /initialpose 改写的变换。"""

    def __init__(self):
        super().__init__("start_pose_tf")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("rate_hz", 20.0)
        self._x = float(self.get_parameter("x").value)
        self._y = float(self.get_parameter("y").value)
        self._yaw = float(self.get_parameter("yaw").value)
        rate = float(self.get_parameter("rate_hz").value) or 20.0

        self._broadcaster = TransformBroadcaster(self)
        # 需要 odom -> base_footprint 才能把"车应该在哪"换算成 map -> odom：
        # map->base = map->odom ⊕ odom->base，而里程计开机后不一定在原点
        # （实测会跑到 (-0.06, -0.25, -1.79)）。直接把 map->odom 设成点击值，
        # 车就会出现在"点击值 + 里程计值"那里，位置和朝向一起偏。
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose",
                                 self._on_pose, 10)
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            "固定起始位姿：map->odom = (%.3f, %.3f, %.3f rad)；"
            "订阅 /initialpose，可在页面重设" % (self._x, self._y, self._yaw))

    def _on_pose(self, msg):
        pose = msg.pose.pose
        want_x = float(pose.position.x)
        want_y = float(pose.position.y)
        want_yaw = yaw_from_quat(pose.orientation)

        # 里程计当前位姿（没有就按 0 处理，等价于旧行为）。
        odom_x = odom_y = odom_yaw = 0.0
        try:
            tf = self._buffer.lookup_transform(ODOM_FRAME, "base_footprint",
                                               Time())
            odom_x = float(tf.transform.translation.x)
            odom_y = float(tf.transform.translation.y)
            odom_yaw = yaw_from_quat(tf.transform.rotation)
        except Exception as exc:            # noqa: BLE001
            self.get_logger().warn("查不到 odom->base_footprint（%s），"
                                   "按里程计为原点处理" % exc)

        # 解 T_map_odom：要 map 下的 base 等于点击位姿
        #   want = R(off_yaw) * odom + off_xy
        off_yaw = want_yaw - odom_yaw
        cos_yaw = math.cos(off_yaw)
        sin_yaw = math.sin(off_yaw)
        self._x = want_x - (odom_x * cos_yaw - odom_y * sin_yaw)
        self._y = want_y - (odom_x * sin_yaw + odom_y * cos_yaw)
        self._yaw = off_yaw
        self.get_logger().info(
            "起点对齐：车应在地图 (%.3f, %.3f, %.3f)，里程计 (%.3f, %.3f, %.3f) "
            "-> map->odom = (%.3f, %.3f, %.3f)"
            % (want_x, want_y, want_yaw, odom_x, odom_y, odom_yaw,
               self._x, self._y, self._yaw))

    def _publish(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = MAP_FRAME
        transform.child_frame_id = ODOM_FRAME
        transform.transform.translation.x = self._x
        transform.transform.translation.y = self._y
        transform.transform.rotation.z = math.sin(self._yaw / 2.0)
        transform.transform.rotation.w = math.cos(self._yaw / 2.0)
        self._broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = StartPoseTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
