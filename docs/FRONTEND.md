# 前端结构

单页应用，不打包，浏览器原生 ES 模块加载；改完刷新即生效。
所有副作用（绑事件、起定时器、建连接）集中在 `js/main.js` 的 `boot()`，
其余模块**导入时不产生任何副作用**，因此每个模块都能单独 import 进测试环境。

## 文件职责

| 文件 | 职责 |
|---|---|
| `index.html` | 静态骨架：顶部状态栏；左列 = 遥控 / 控制指令 / 摄像头；右列 = 地图 / 遥测 / 调参 |
| `css/tokens.css` | 设计令牌与基础元素（颜色、间距、按钮、输入框） |
| `css/app.css` | 页面布局与通用组件（卡片、徽章、标签、摇杆、地图、调参栅格） |
| `css/camera.css` | 摄像头面板：固定高度画面（竖屏也能居中）、深色底、全屏放大 |
| `js/config.js` | 前端常量与阈值 |
| `js/dom.js` | DOM 小工具（取元素、写文本、设标签） |
| `js/state.js` | 前端共享状态（连接时间、socket 状态等） |
| `js/ui.js` | 顶层 UI 开关（急停锁存时禁用输入） |
| `js/input.js` | 摇杆 / 键盘 → 速度指令 |
| `js/net.js` | socket 连接；`robot_status` / `robot_nav` 分发；命令上行（含导航命令队列） |
| `js/status.js` | 遥测渲染 + socket 断流时的兜底轮询 |
| `js/actions.js` | 急停 / 复位故障 / 校准航向 |
| `js/map.js` | 地图画布：OccupancyGrid、机器人位姿、规划路径、点击设目标/设位姿 |
| `js/camera.js` | 摄像头：MJPEG 播放、暂停/继续、全屏放大、状态轮询与断流重连 |
| `js/detect.js` | 物品检测叠加层：轮询 `/api/detect`，画检测框 + 类别列表 |
| `js/tune.js` | 调参面板（纯 REST，与实时链路无关） |
| `js/main.js` | 装配层：按顺序初始化以上模块 |

## 依赖方向

```
main.js ──> 其余所有模块
                │
                └─> 只允许共用 config / dom / state
```

模块之间不互相 import 业务逻辑：`map.js` 与 `camera.js` 各自持有局部状态，
不往 `state.js` 里塞"当前地图""当前画面"这类只属于一个面板的数据。

## 摄像头

摄像头刻意**不经过 socket**，因为它和遥测的节奏完全不同（15 fps 视频 vs 5 Hz 状态）：

- 视频：`<img src="/api/camera/stream">`，服务端返回 `multipart/x-mixed-replace`，
  浏览器原生支持，不需要 WebRTC 或额外播放器。
- 状态：`GET /api/camera/status`，每 2 s 一次，返回分辨率、实测 fps、观看端数、错误。
- **断流判断看 `age_s`，不看 `onerror`**：MJPEG 是长连接，服务端卡住时浏览器
  只是安静地等下一帧，画面停在最后一帧且不报错。`camera.js` 在帧龄超过 3 s
  时重设 `src` 重连，两次重连之间至少隔 4 s（设备没插时不要疯狂建连）。
- **暂停会移除 `src`**：只把元素隐藏不会断开 HTTP 请求，服务端仍占一个客户端
  和一份采集槽。

服务端（`python/camera.py`）只有一个采集线程，所有客户端读同一份最新 JPEG；
`WebUI.expose_camera` 是每个客户端各跑一个采集生成器，多标签页会互相抢帧。

画面方向由 `camera.py` 的 `ROTATE_DEG` 决定（当前 90° 顺时针，输出 360×640）。
前端不假设宽高比：容器固定高度、`img` 用 `object-fit: contain`，改角度不用动 CSS。

## 物品检测

推理不在 App 容器里，而在官方 `arduino:object_detection` brick 起的
`ei-obj-detection-runner` 容器里（Edge Impulse `yolo-x-nano.eim`，HTTP 1337，
compose 内网）。App 侧只负责三件事：

- **喂帧**：`python/detector.py` 起一条线程，取 `CAMERA.snapshot()` 的最新 JPEG
  发给 brick；推理期间产生的新帧直接丢弃，不排队，否则画面在动、框停在几秒前。
- **限速**：`MIN_INTERVAL_S = 0.2` 只是下限，实测单帧 0.55~0.8 s，整机满栈
  运行时约 1.2~1.8 帧/秒。检测框因此**落后画面半秒左右**，这是 SoC 算力决定的，
  不是前端丢帧。
- **出错不冒泡**：runner 没起来、模型还在加载都会抛异常，`detector.py` 只记
  `error` 并按 5 s 退避重试，绝不让底盘控制或摄像头跟着起不来。

前端每 500 ms `GET /api/detect`，返回 `{ready, objects[], width, height,
infer_ms, age_s, error}`。框的坐标是**检测时那张图**的像素（360×640），
`detect.js` 按 `object-fit: contain` 的内容区自己算缩放和偏移 —— 竖屏画面在
`.camera-frame` 里左右有黑边，直接按容器尺寸画会整体偏。

`age_s > 3 s`、`ready=false` 或 `error` 非空时**不画框**：视频还在动而框停在
几秒前，比没有框更容易让人误判。检测结果走单独 canvas 叠加，不烧进 JPEG ——
检测约 1 fps、视频 15 fps，烧进画面框会跟着冻住。
