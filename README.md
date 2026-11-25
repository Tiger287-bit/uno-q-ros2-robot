# uno-q-ros2-demo

UNO Q 差速小车底盘控制 App：MCU 侧跑运动控制固件，Linux 侧跑控制环与 WebUI，
ROS 2 通过 `ros2_bridge` brick 接入。

## 结构

```
├── app.yaml              App Lab 清单：端口 7000，bricks = web_ui + ros2_bridge
│                         + arduino:object_detection
├── assets/               WebUI：index.html + css/ + js/ + vendor/（结构见 docs/FRONTEND.md）
├── docs/FRONTEND.md      前端分层、依赖方向、摄像头与检测数据通路
├── python/
│   ├── main.py           装配层：WebUI 接线、控制线程、命令队列
│   ├── control.py        差速运动学、直行保向、状态解码
│   ├── config.py         整车常量与状态词表
│   ├── camera.py         摄像头采集：单线程 + 最新帧缓存（MJPEG 与快照共用）
│   ├── detector.py       物品检测：取最新帧喂 object_detection brick，缓存结果
│   └── telemetry.py      内存环形缓冲（17 列，12000 帧）
├── sketch/               MCU 固件（motion_controller 状态机 + PID + 编码器 + IMU）
├── tests/                固件主机测试：换向时序 / 上电自检 / 正交解码（见"测试"）
└── bricks/ros2_bridge/   ROS 2 Jazzy 容器：rosbridge + 手柄驱动 + 速度仲裁
                         （代码结构见 bricks/ros2_bridge/docs/ARCHITECTURE.md）
```

前端是一个不打包的单页应用：`assets/js/` 下按 `config / dom / state / ui / input /
net / status / actions / main` 分层，浏览器原生 ES 模块加载，改完刷新即生效。
`assets/vendor/socket.io.min.js` 是随 App 发布的 socket.io 客户端 —— 本版 App Lab
不提供 `/socket.io/socket.io.js`（返回 400），缺了它页面会静默失灵。

## 遥控链路

```
手柄 ──> /cmd_vel_joy ─┐
                      ├─> cmd_vel_mux ──> /cmd_vel ──> App 控制环 ──> MCU
网页 ──> /cmd_vel_web ┘
        （由 main.py 经 rosbridge 发布，浏览器不直连 9090）
```

两路输入由容器内的 `cmd_vel_mux` 仲裁：非零指令取得控制权，零值只有持有者
发了才算数（相当于"让位"），静默 200 ms 自动归零。

## 启动

```bash
arduino-app-cli app start ~/ArduinoApps/uno-q-ros2-demo
```

首次启动会在板上构建 ROS 镜像（约 5~10 分钟、1.5 GB 磁盘），之后走缓存。
启动会编译并烧录 sketch 到 MCU，**电机可能动作，首次请把车轮架空**。

## 用网页遥控

打开 `http://<板 IP>:7000`：

- 拖动摇杆，或按住 `W A S D` / 方向键；
- 速度滑块按百分比**同时缩放线速度与角速度**，读数会给出百分比、cm/s 与 °/s；上限由后端下发，改轮径或换电机后自动跟着变；
- 松开即停车并交还控制权给手柄；
- 急停会在**后端**锁存：网页与手柄两路输入同时被忽略（手柄走 `/cmd_vel_joy`→mux→`/cmd_vel`，同样被拦），须再次点击「解除锁定」恢复；锁存状态由状态推送回传，刷新页面或多开一个浏览器不会出现"界面显示已锁定、实际能开"的分歧。
  - **解除前请先松开摇杆/按键**：锁存只忽略输入、不记忆输入，若手柄仍推着，解除瞬间 `/cmd_vel` 就是非零，车轮在固件 `SETTLE_MS`（300 ms）后直接起步。

两路遥控的"满舵"不是同一条刻度，换着用容易误以为车变肉/变窜：

| 输入 | 线速度 | 角速度 |
|---|---|---|
| 手柄满杆 | 0.075 m/s | 1.20 rad/s（= 网页 100%） |
| 手柄 + 加速键 | 0.0975 m/s | 1.20 rad/s（turbo 对角速度无效：轮速分解前按 `MAX_ANGULAR` 截断） |
| 网页滑块默认 60% | 0.0593 m/s | 0.72 rad/s |
| 网页滑块 100% | 0.0989 m/s | 1.20 rad/s |

手柄的线速度上限刻意压到 0.075 m/s（再高进入轮子间歇打滑区），而网页是一个滑块同时
缩放两轴，所以不存在某个滑块位置能同时对上"手柄满杆"的两轴——这是有意为之。要改标定
看 `bricks/ros2_bridge/docker/ros_ws/src/arduino_ros2_bringup/config/teleop.yaml` 的
`scale_linear` / `scale_angular`。

顶部状态栏的徽章就是链路诊断（左侧三个：`ROS 2` App↔rosbridge、`MCU` 状态、
`N Hz` 采样率；右侧两个：`网页指令 N 帧` 页面上行是否送达、`实时已连接` socket.io）。
两种"看着正常其实已经旧了"的情形各有专门文案：`MCU 状态过期`（状态帧超过 1 s
没更新，面板上的轮速/PWM 都是断链前那一帧）、`采样停更`（遥测超过 3 s 没进新帧，
采样率归零）。看到这两个就别信面板数值。若显示 `socket.io 未加载`，说明
`assets/vendor/socket.io.min.js` 缺失或被删——本版 App Lab 不提供该客户端脚本。

### 停车是三层的独立兜底

任何一层断掉，下一层都会在各自时限内把轮速归零，不要求链路整体健康：

| 层 | 判据 | 时限 | 位置 |
|---|---|---|---|
| ROS 仲裁 | 输入话题静默 | 0.2 s | `cmd_vel_mux` 的 `hold_s` |
| App | 迟迟没有新指令 | 0.4 s | `python/config.CMD_TIMEOUT_S` |
| 固件 | 没收到 `ros_drive` 心跳 | 0.6 s | `sketch/motion_controller.cpp` 的 `WATCHDOG_MS` |

所以 App 进程被杀、控制线程卡死、RPC 断链，最终都会在 600 ms 内停车。松杆时主动下发 0
并重试只是为了停得更快、错误可见，不承担"防失控"职责。

> **安全提示**：7000 是唯一的控制入口，`web_ui` 绑 `0.0.0.0` 且**没有任何鉴权**——
> 同一网段的任何设备（手机、笔记本）打开这个地址就能直接开车，`cmd_vel` 也没有口令校验。
> **「解除锁定」同样无鉴权**：别人不仅能开车，也能把别人按下的急停解掉，急停不能当成
> 有权限边界的互锁来用。只在可信局域网内 `app start`，不要在公共网络/咖啡店 WiFi/
> 公司访客网下运行；运行期间建议让小车处在架空或封闭场地状态。ROS 侧的 9090 未对外
> 暴露，但这不影响入口开放的事实。

## 摄像头

车头 UVC 摄像头（`/dev/video0`，设备名 `openaicam`），页面左列最下方实时显示。

- 采集 1280×720 MJPG，编码前**顺时针旋转 90°**（摄像头被线束逼着装成侧向），
  再按长边缩到 640 → 输出 **360×640**（竖屏）。这枚摄像头只枚举 1080p/720p 两档，
  720p 的 JPEG 每帧 100~200 KB，缩到 360×640 后约 20~35 KB（15 fps ≈ 3 Mbit/s）。
  方向不对时改 `python/camera.py` 的 `ROTATE_DEG`（0/90/180/270）。
- 页面用 `<img src="/api/camera/stream">` 吃 MJPEG 流；状态走 `/api/camera/status`。
- **暂停**会真正断开视频连接；**双击画面**或点「放大」全屏查看。
- 摄像头起不来不影响底盘：采集线程按 2 s 重试，原因显示在画面上方的状态标签里
  （`找不到 /dev/video0`、`设备打不开` 等）。

前端与后端的模块划分见 [docs/FRONTEND.md](docs/FRONTEND.md)。

## 物品检测

页面摄像头下方显示检测结果：画面里画框 + 类别/置信度列表。模型在官方
`arduino:object_detection` brick 起的 `ei-obj-detection-runner` 容器里
（Edge Impulse `yolo-x-nano.eim`，80 类 COCO，HTTP 1337 仅 compose 内网可达），
App 容器只负责喂帧和缓存结果。

- 单帧推理**实测 0.55~0.8 s**（含 JPEG 解码与 HTTP 往返），所以检测约 1.2 帧/秒，
  框会**落后画面半秒左右**；视频仍是 15 fps，框画在独立 canvas 上，不烧进画面。
- 只取最新帧、推理期间的新帧直接丢弃 —— 宁可结果旧一点，也不要排队积压。
- runner 缺失或模型加载中不会影响底盘与摄像头：`detector.py` 记 `error` 后按 5 s
  重试，`/api/detect` 的 `ready` 为 false，页面显示「检测不可用」。
- 首次启动要等模型加载十几秒，页面标签会先显示「检测启动中」。
- 这个 brick 会多占约 900 MB 镜像、160 MB 内存；整机只剩约 0.5 GB 可用
  （`ros2` 容器 2.4 GB + 主容器 + runner），导航栈与推理同时在跑时要留意 OOM。

前端与后端的模块划分见 [docs/FRONTEND.md](docs/FRONTEND.md)。

## 接口

| 通道 | 名称 | 说明 |
|---|---|---|
| REST | `GET /api/telemetry` | 最近 2 帧 + `paused`，供页面兜底轮询（只要最后一行，回 300 帧纯属烧流量） |
| REST | `GET /api/telemetry/full?since_seq=` | 全量或增量，供同类导出工具拉取；游标已被缓冲挤掉时回包带 `truncated: true` |
| REST | `GET /api/detect` | 物品检测结果：`ready`、`objects[]`（label/score/框）、`width/height`、`infer_ms`、`age_s`、`error` |
| socket.io | `robot_status`（下行） | 约 5 Hz 状态推送，含 `paused`（急停锁存是否生效） |
| socket.io | `cmd_vel`（上行） | `{linear, angular}`，限幅后转 `/cmd_vel_web` |
| socket.io | `stop` / `resume` / `reset` / `calibrate`（上行） | 急停锁存 / 解除锁存 / 清故障 / 校准航向，**必须带 payload**（见下） |

> 上行事件一律要带参数，哪怕是"无参数"命令也要发 `{}`。官方 `web_ui` 的通用分发器是
> `handle_generic_event(event, sid, data)`，`data` 无默认值：`socket.emit("stop")` 会在框架里
> 抛 `TypeError` 且回调不执行——命令被静默丢弃，前端却以为已下发。

## 测试

```bash
# 固件：换向时序、上电自检回放、正交解码。板端没有 g++，脚本借 ros2 容器编
# sketch/ 下的真源码（不是副本）—— 改过 sketch/ 就跑一遍。
bash tests/run.sh

# 前端：逻辑回归（DOM 桩 + 分层检查）。_web_test/ 不随 App 下发，在主机工作区
# 根目录对着本 App 的 assets/js 跑；浏览器探针与窄屏见 docs/FRONTEND.md。
node _web_test/run.mjs assets/js
```

`tests/` 随 App 下发但不参与运行：不进固件编译，也不进 ROS 镜像。两套工装都是
"改坏了会红"的 —— 把 `clearWheelOutput` 退回两步走会红 6 项；把 PWM 桩里的通道号
直接当输入索引用会红 5 项；让 nSLEEP 回读恒为高（自检早退）会红 3 项。
