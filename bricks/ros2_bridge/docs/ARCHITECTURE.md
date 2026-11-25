# ROS 2 侧代码结构

本 brick 的 ROS 代码按**标准 ROS 2 工作区**组织：容器里是一个 `colcon` 工作区，
功能以 `ament_python` 包的形式存在，节点是可被 `ros2 run` 调用的独立可执行，
启动逻辑集中在 `launch/`。新增 ROS 功能时只增文件、不改容器骨架。

## 目录

```
bricks/ros2_bridge/
├── brick_config.yaml          # Brick 清单（App Lab 读）
├── brick_compose.yaml         # ros2 容器定义
├── __init__.py                # App 侧 WebSocket 客户端（另一个容器，见 docs/API.md）
├── requirements.txt
└── docker/                    # ← 构建上下文，容器里的东西全在这里
    ├── Dockerfile
    ├── docker-entrypoint.sh   # 叠好 ROS 环境后执行 CMD
    └── ros_ws/src/arduino_ros2_bringup/     # ← ROS 2 包
        ├── package.xml
        ├── setup.py           # 注册可执行入口 + 安装 launch/config
        ├── setup.cfg
        ├── resource/arduino_ros2_bringup
        ├── arduino_ros2_bringup/                # Python 模块
        │   ├── __init__.py
        │   ├── common/            # 共享层（无状态）
        │   │   ├── topics.py      # 话题名 / 消息类型，全 brick 唯一事实来源
        │   │   └── runtime.py     # 统一 rclpy 生命周期（init/spin/干净退出）
        │   └── nodes/             # 能力层：一个文件一个节点
        │       ├── js_joy_node.py
        │       └── cmd_vel_mux.py
        ├── config/                # 参数层
        │   ├── teleop.yaml
        │   └── mux.yaml
        └── launch/                # 编排层
            ├── bringup.launch.py  # 组合入口（容器 CMD 调它）
            ├── rosbridge.launch.py
            ├── teleop.launch.py
            └── mux.launch.py
```

## 四层职责

| 层 | 放什么 | 不放什么 |
|---|---|---|
| `common/` | 话题名、消息类型、运行骨架等无状态工具 | 打开设备、建连接、起线程 |
| `nodes/` | 每个文件一个节点类 + `main()`；资源获取都在 `__init__` | 模块级副作用、跨节点 import |
| `config/` | ROS 2 参数 YAML，按节点名分文件 | 逻辑、默认值之外的魔法数字 |
| `launch/` | 组合节点、传参、开关 | 业务逻辑 |

**为什么参数外置**：手柄轴号、速度标定、仲裁频率这些是会随硬件变的东西。
放在 YAML 里，换手柄只改 `config/teleop.yaml`；节点代码里保留同名默认值，
YAML 缺项时仍能跑。

**为什么 `common/topics.py` 单独存在**：话题名是跨容器契约。之前每个文件
各写一遍字符串，改名要全仓库搜索；现在节点与 launch 都从常量取。
App 侧是另一个容器、import 不到这里，所以 `__init__.py` 与 `python/main.py`
各自保留用到的常量 —— 改话题名时**两处都要改**，`docs/API.md` 与
`README.md` 的话题表是共同契约。

## 数据流

```
 /dev/input/js* ──> js_joy_node ──> /cmd_vel_joy ─┐
                                                  ├─> cmd_vel_mux ──> /cmd_vel ──┐
 App 主容器 Ros2Publisher ──ws──> rosbridge ──────┘                              │
        ▲                       /cmd_vel_web                                    │
        └──── 浏览器 socket.io ──── App WebUI ────────────────────────────────────┘
                                        App 控制环订阅 /cmd_vel → Bridge → MCU
```

`cmd_vel_mux` 的输入路数由参数 `input_topics` 决定（默认两路），接导航或第二只
遥控器时往 YAML 列表追加话题即可，节点代码不用动。

## 新增一个 ROS 功能

以"发布里程计 `/odom`"为例，五步，全部是加文件：

**1. 声明话题**（`arduino_ros2_bringup/common/topics.py`）

```python
class Topics:
    ...
    ODOM = "/odom"
```

**2. 写节点**（`arduino_ros2_bringup/nodes/odom_publisher.py`）

```python
from arduino_ros2_bringup.common.runtime import run
from arduino_ros2_bringup.common.topics import Topics


class OdomPublisher(Node):
    def __init__(self):
        super().__init__("odom_publisher")
        self.period = float(self.declare_parameter("period_s", 0.1).value)
        self.pub = self.create_publisher(Odometry, Topics.ODOM, 10)
        self.create_timer(self.period, self._publish)

    def _publish(self):
        ...   # 从 /cmd_vel 或 MCU 遥测积分出位姿


def main():
    run(OdomPublisher)          # 生命周期交给 common/runtime.py
```

**3. 注册入口**（`setup.py` 的 `ENTRY_POINTS`）

```python
"odom_publisher = arduino_ros2_bringup.nodes.odom_publisher:main",
```

**4. 加参数**（`config/odom.yaml`，键名必须等于节点名）

```yaml
odom_publisher:
  ros__parameters:
    period_s: 0.1
```

**5. 加 launch 并挂进 bringup**

```python
# launch/odom.launch.py
Node(package="arduino_ros2_bringup", executable="odom_publisher",
     name="odom_publisher",
     parameters=[PathJoinSubstitution([FindPackageShare(PACKAGE), "config", "odom.yaml"])],
     output="screen")
```

```python
# launch/bringup.launch.py 里加一个开关和一行包含
DeclareLaunchArgument("enable_odom", default_value="true", description="..."),
IncludeLaunchDescription(_sub_launch("odom.launch.py"),
                         condition=IfCondition(LaunchConfiguration("enable_odom"))),
```

最后重建镜像并重启 App（见下节）。App 侧要读这个话题时：

```python
from ros2_bridge import Ros2Subscriber
odom = Ros2Subscriber(topic="/odom", message_type="nav_msgs/msg/Odometry")
```

## 构建与调试

镜像由 App Lab 在启动 App 时构建；也可以手工构建后直接用容器验证：

```bash
cd ~/ArduinoApps/uno-q-ros2-demo/bricks/ros2_bridge
docker build -t ros2_bridge:local ./docker        # 首次 5~10 分钟，改代码后约 15 秒
```

容器内调试（`docker exec` 进去后先 `source /opt/ros_ws/install/setup.bash`）：

| 目的 | 命令 |
|---|---|
| 看包与可执行 | `ros2 pkg executables arduino_ros2_bringup` |
| 单跑一个节点 | `ros2 run arduino_ros2_bringup cmd_vel_mux` |
| 单跑一个功能 | `ros2 launch arduino_ros2_bringup teleop.launch.py` |
| 换一套手柄参数 | `ros2 launch ... teleop.launch.py params_file:=/tmp/mine.yaml` |
| 看全部开关 | `ros2 launch arduino_ros2_bringup bringup.launch.py --show-args` |
| 临时改参数 | `ros2 run ... js_joy_node --ros-args -p scale_linear:=0.05` |
| 导出实际参数 | `ros2 param dump /cmd_vel_mux` |
| 验证频率 | `ros2 topic hz /cmd_vel` （应约 50 Hz） |

改参数不必改代码：先命令行试，确认后落到 `config/*.yaml`，再重建镜像。

## 约定

1. **不要用 `ExecuteProcess(cmd=["python3", ...])` 起节点** —— 那样节点没有
   ROS 元信息（`ros2 node list` 看不到、参数文件不生效、`ros2 run` 用不了）。
   一律 `Node(package=..., executable=...)`。
2. **节点名 = `config/*.yaml` 的顶层键 = launch 里的 `name=`**，三者必须一致，
   否则参数文件静默不生效。
3. 节点里不写模块级副作用，`--show-args`、`param dump` 这类工具才不会拉起硬件。
4. 退出统一走 `common/runtime.py` 的 `run()`，它处理了 SIGINT 与重复
   `rclpy.shutdown()` 这两个容器场景下的坑。镜像里必须配 `STOPSIGNAL SIGINT`：
   ros2 launch 只认 SIGINT，默认 SIGTERM 会直接杀掉它，节点跟着被带走，
   `run()` 里那条清理路径等于死代码。
5. 新增话题先改 `common/topics.py`，不要在节点里写字面量。
6. **YAML 里有的键，launch 不要再传一遍**。`parameters` 列表是后者覆盖前者，
   多传一次就等于给该参数加了第二个来源和第二个默认值——`device` 曾因此被
   launch 的硬默认 `js0` 永久顶掉，改 YAML 完全没反应。

