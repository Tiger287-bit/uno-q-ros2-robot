# ROS 2 Bridge

把 ROS 2 Jazzy 接进 App Lab：一个容器里跑 rosbridge、手柄驱动和速度仲裁节点，
App 侧用几行 Python 就能订阅 `/cmd_vel`，也能把网页指令发回 ROS 2 图。

## 它在整条链路里的位置

```
 手柄 /dev/input/js* ──> js_joy_node ──> /cmd_vel_joy ─┐
                                                       ├─> cmd_vel_mux ──> /cmd_vel
 App 主容器 Ros2Publisher ──ws──> rosbridge ───────────┘        │
            ▲                          /cmd_vel_web           │
            │                                                  ▼
            └──── 浏览器 ──socket.io──> App WebUI      App 控制环订阅 /cmd_vel
                   (App 的 7000 端口)                   经 Bridge 下发 MCU
```

两处容器边界都只能走网络：App 侧 Python 通过 rosbridge 的 WebSocket 收发
话题，不依赖容器内的 ROS 2 环境。

**网页不直连 rosbridge。** 浏览器用 App 自己的 socket.io 通道发速度指令，
由 `main.py` 交给 `Ros2Publisher` 转进 `/cmd_vel_web`。所以 9090 完全不必
映射到宿主机 —— 也就没有"局域网内任何人发一帧就能驱动整车"的问题。

## 代码结构

容器里是标准 ROS 2 工作区：功能做成 `ament_python` 包，节点是可 `ros2 run`
的独立可执行，启动逻辑集中在 `launch/`。

```
bricks/ros2_bridge/
├── __init__.py                  # App 侧 WebSocket 客户端（另一个容器）
├── brick_config.yaml            # Brick 清单
├── brick_compose.yaml           # ros2 容器定义
└── docker/                      # 构建上下文
    ├── Dockerfile               # colcon build + entrypoint
    ├── docker-entrypoint.sh
    └── ros_ws/src/arduino_ros2_bringup/
        ├── package.xml  setup.py  setup.cfg  resource/
        ├── arduino_ros2_bringup/
        │   ├── common/topics.py     # 话题契约（唯一事实来源）
        │   ├── common/runtime.py    # 统一 rclpy 生命周期
        │   └── nodes/               # 一个文件一个节点
        ├── config/                  # 参数 YAML（换手柄只改这里）
        └── launch/                  # bringup 组合 + 每功能一个
```

协议与"如何新增一个 ROS 功能"见 **`docs/ARCHITECTURE.md`**。

## 为什么需要 cmd_vel_mux

手柄节点和网页键盘是两条独立输入链路。若都直接写 `/cmd_vel`，后到的消息会
覆盖先到的：手柄按住使能键、摇杆回中时会以 50 Hz 持续发 0，网页指令会被下一
帧瞬间冲掉。

仲裁采用**控制权模型**，而不是"最近活跃优先"：

1. 收到**非零**指令即取得控制权，多路竞争时后动者接管；
2. **零值不参与竞争**，只有当前持有者发零才生效 —— 生效的同时释放控制权，
   让另一路可以无缝接手；
3. 持有者静默超过 200 ms 视为松手，输出 0 并释放控制权；
4. 无持有者时恒定发布 0，下游不必依赖消息超时判断停车。

第 2 条是关键：把"回中"和"抢权"分开，避免手柄按住使能键时独占控制权。

输入路数不写死：`cmd_vel_mux` 按参数 `input_topics` 建订阅，默认两路。接导航
（`/cmd_vel_nav`）或第二只遥控器时，往 `config/mux.yaml` 的列表追加一行即可，
节点代码与 App 侧都不用改。

## 话题

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/cmd_vel_joy` | `geometry_msgs/Twist` | 手柄 → 仲裁 | 仅按住使能键时发布，松杆补一帧 0 后静默 |
| `/cmd_vel_web` | `geometry_msgs/Twist` | App → 仲裁 | 网页摇杆/键盘经 `main.py` 发布 |
| `/cmd_vel` | `geometry_msgs/Twist` | 仲裁 → App | 50 Hz 恒定发布，无输入时为 0 |
| `/joy` | `sensor_msgs/Joy` | 手柄 → 调试 | 原始轴与按钮 |

## 配置变量

| 变量 | 默认值 | 说明 |
|---|---|---|
| `ROSBRIDGE_URL` | `ws://ros2:9090` | App 侧连接地址，`ros2` 是 compose 服务名 |
| `ROS_DOMAIN_ID` | `0` | ROS 2 域号，同网多机联调时用来隔离话题 |

刻意**不声明** `BIND_ADDRESS`：那是平台与官方 compose 共用的保留名，在这里
声明同名变量会覆盖官方默认值。本 brick 不做端口映射，不需要它。

节点自身的参数（手柄轴号、速度标定、仲裁频率与输入路数）不在这里，而是
`docker/ros_ws/src/arduino_ros2_bringup/config/*.yaml`。改完重建镜像即生效：

```bash
cd ~/ArduinoApps/uno-q-ros2-demo/bricks/ros2_bridge
docker build -t ros2_bridge:local ./docker   # 首次 5~10 分钟，改代码后约 15 秒
```

## 调试

进容器后先 `source /opt/ros_ws/install/setup.bash`，然后：

```bash
ros2 pkg executables arduino_ros2_bringup              # 有哪些可执行
ros2 run arduino_ros2_bringup cmd_vel_mux              # 单跑一个节点
ros2 launch arduino_ros2_bringup teleop.launch.py      # 单跑一个功能
ros2 launch arduino_ros2_bringup bringup.launch.py --show-args   # 看全部开关
ros2 run arduino_ros2_bringup js_joy_node --ros-args -p scale_linear:=0.05
ros2 param dump /cmd_vel_mux                           # 导出实际生效参数
ros2 topic hz /cmd_vel                                 # 应约 50 Hz
```

`bringup.launch.py` 的开关远不止三个：`enable_rosbridge`、`enable_teleop`、
`enable_mux`、`enable_description`、`enable_odom`、`enable_slam`、
`enable_foxglove`，排查时可只留一部分（`--show-args` 看全量）。

## 建图（SLAM）

```bash
ros2 launch arduino_ros2_bringup slam.launch.py     # 单独建图，自带 /scan 中继
```

- 中继 `/scan -> /scan_slam` 在 `slam.launch.py` 里，bringup 不再重复起，
  两个入口是同一条链路。
- `/map` **不是**随扫描发布的：它按 `map_update_interval`（这里设 1 s）
  定时重算。判断"有没有在匹配"看 `/pose`（每成功处理一帧发一次）。
- 原地转向要能进匹配，必须 `check_min_dist_and_heading_precisely: true`
  （默认 false 时纯旋转一帧都不处理）。

地图存放在 Brick 的 `docker/ros_ws/maps/`。容器中的保存路径为 `/opt/ros_ws/maps/`。

```bash
ros2 run nav2_map_server map_saver_cli -t /map --fmt pgm \
    -f /opt/ros_ws/maps/room1 --ros-args -p save_map_timeout:=15.0
```

保存后，`room1.pgm` 和 `room1.yaml` 一起留在该目录。


## 导航（Nav2）与建图切换

```bash
ros2 launch arduino_ros2_bringup bringup.launch.py                     # 默认导航模式
ros2 launch arduino_ros2_bringup bringup.launch.py initial_mode:=slam  # 开机进建图
ros2 launch arduino_ros2_bringup nav.launch.py                         # 只起导航栈（调试）
ros2 launch arduino_ros2_bringup nav.launch.py map_yaml:=/opt/ros_ws/maps/room1.yaml
```

- 地图与建图共用 `docker/ros_ws/maps/`（容器内 `/opt/ros_ws/maps/`）。
- **两套栈常驻、只有一套 active**：`slam_toolbox` 与 `amcl` 都发 `map->odom`。
  bringup 里两个 launch 都传 `autostart:=false`，由 `mode_manager` 用生命周期
  服务（导航组 PAUSE/RESUME、建图组 deactivate/activate）决定谁在跑。
  页面上「导航 / 建图」开关发的就是 `nav_cmd {"cmd":"mode","mode":"slam"}`，
  切换通常 1 s 内完成，期间两个按钮都禁用。
- **不要对导航组用 SHUTDOWN**：nav2 的 `lifecycle_manager` 在 shutdown 末尾会
  销毁自己的服务客户端，之后再 RESUME 会直接段错误退出（实测过，只能重启容器）。
  节点一旦走到 finalized 也只能重启。所以切换只用 deactivate/activate。
- 定位用 AMCL，初始位姿默认 (0,0,0)——建图时 map 原点就是车的出发点。
  页面「重设初始位姿」会先撤掉当前目标再下发；状态条显示「定位已确认」才算
  真的生效（`nav_bridge` 比对 TF 位姿与请求值）。从建图切回导航时，会把切走
  前 slam 的位姿自动作为 AMCL 初始位姿，车不用重新对位。
- 速度出口是 `/cmd_vel_nav`，经 `cmd_vel_mux` 仲裁后进 `/cmd_vel`。
  **仲裁是人工优先**：`manual_topics`（手柄、网页）只要在 `manual_hold_s` 内有
  非零指令就压过导航，并把来源发到 `/cmd_vel_source`。接管事件在收到人工非零
  **当拍**就发出去（不等仲裁周期），否则"人手点一下就回中"会让事件永远发不出来；
  之后 `takeover_latch_s`(1.5 s) 内自动来源被压住，给取消留时间。
  `nav_bridge` 收到人工来源立刻取消目标。App 也订阅这条来源：导航指令**不做内轮吸附**（吸附会改写曲率，
  Nav2 的碰撞预测会失效），人手指令保持吸附。
- 代价地图参数是**顶层**的 `local_costmap.local_costmap.ros__parameters` /
  `global_costmap.global_costmap...`，不是 `controller_server` 的子键。写在父节点
  下面不会报错，但节点读不到，症状是局部代价地图退回 5x5 m、订阅 `/scan`，
  避障配置全部失效。`width/height` 是格子数，必须写整数 `3`。
- **建图模式不会清空已有地图**：Jazzy 的 slam_toolbox 在 deactivate/activate 之间
  保留位姿图，同一次 App 运行里切回建图是"接着画"。建完要 `save_map` 存新文件；
  切回导航读的仍是启动参数 `map_yaml`（默认 `room_20261122.yaml`），不是刚画的那张。
- 受理目标要求「导航模式就绪 + 位姿新鲜 + 定位已确认」；`waiting` / `timeout` /
  `needed` 三种状态都会被后端拒绝（页面同样禁用按钮）。
- 页面侧：`nav_bridge` 把地图、位姿、路径、状态、运行模式折成一条 JSON 发到
  `/app/nav_ui`，App 转成 socket 事件 `robot_nav`；点击地图发 `nav_cmd` 回来。
  地图每 2 s 附带一次，每格一字节 base64（0 空闲 / 1 占据 / 2 未知）。
  建图模式下显示的是 slam 的实时地图（同样是 `/map`，QoS 都是 transient_local）。


## 硬件与安全

- 手柄通过 `devices: /dev/input` 整体挂进容器，**不插手柄也能正常启动**。
- Docker 只在容器创建时把 `/dev/input` 下已存在的节点加入 cgroup，因此
  **容器运行期间才插上的手柄不会被识别** —— 节点内部的 open() 重试救不了
  "设备从未被映射进来"。换手柄或热插拔后请重启 App。
- 9090 **不做端口映射**，rosbridge 只在 Compose 网络内可达。需要用电脑上的
  ros2 / rviz 观察话题时走 SSH 隧道：
  `ssh -L 9090:127.0.0.1:9090 arduino@<板 IP>`
- 首次启动会在板上构建镜像，需要约 1.5 GB 空闲磁盘、5~10 分钟。

## 快速开始

订阅（读车当前收到的速度指令）：

```python
from ros2_bridge import Ros2Subscriber

link = Ros2Subscriber(topic="/cmd_vel", message_type="geometry_msgs/msg/Twist")
link.start()

message, age_s, connected = link.snapshot()
```

发布（把网页指令送进 ROS 2 图）：

```python
from ros2_bridge import Ros2Publisher

web = Ros2Publisher(topic="/cmd_vel_web", message_type="geometry_msgs/msg/Twist")
web.start()
web.publish({"linear": {"x": 0.05, "y": 0.0, "z": 0.0},
             "angular": {"x": 0.0, "y": 0.0, "z": 0.0}})
```

`publish()` 非阻塞，可直接在 socket 回调里调用；待发槽深度为 1，只保留最新
一帧速度指令。

完整接口见 **API documentation** 页签，可直接运行的片段见 **Usage examples**。
