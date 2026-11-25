# ROS 2 Bridge — API

本 brick 的 Python 侧运行在 App 主容器里，通过 rosbridge 的 WebSocket 访问
ROS 2 话题。导入方式是直接用目录名：

```python
from ros2_bridge import Ros2Publisher, Ros2Subscriber
```

两个类各自维持一条 WebSocket 连接，互不干扰；rosbridge 对连接数没有硬限制。

话题名是跨容器契约：容器内的事实来源是
`docker/ros_ws/src/arduino_ros2_bringup/arduino_ros2_bringup/common/topics.py`，
本文件「常量」一节是它在 App 侧的副本（两个容器无法互相 import）。改话题名时
两处同步，并更新 README 的话题表。

---

## `class Ros2Subscriber`

订阅一个 ROS 2 话题，只保留最新一条消息，断线后自动重连。
适用于把高频话题降频成控制环可以直接读取的共享状态。

### 构造

```python
Ros2Subscriber(url=None, topic="/cmd_vel", message_type="geometry_msgs/msg/Twist")
```

| 参数 | 类型 | 说明 |
|---|---|---|
| `url` | `str \| None` | rosbridge 地址。`None` 时读环境变量 `ROSBRIDGE_URL`，再退到 `ws://ros2:9090` |
| `topic` | `str` | 要订阅的话题名 |
| `message_type` | `str` | 话题的消息类型，rosbridge 订阅时必填 |

构造只保存参数，不建立连接。因此可以在模块级直接实例化。

### `start()`

启动后台订阅线程。重复调用无副作用（线程已存活时直接返回）。

### `stop()`

请求订阅线程退出并最多等待 2 秒。App 退出前应调用，避免线程悬挂。

### `snapshot()`

返回 `(message, age_s, connected)`：

| 字段 | 类型 | 说明 |
|---|---|---|
| `message` | `dict` | 最新一条消息的原始字段，尚未收到时为 `{}` |
| `age_s` | `float` | 距该消息的秒数；从未收到过为 `inf` |
| `connected` | `bool` | WebSocket 当前是否连通 |

消费侧建议同时判断 `connected` 与 `age_s`：链路通但话题没数据时 `age_s`
会持续增大，可据此判定上游节点已停发。

### `stats()`

返回链路诊断字典，适合直接塞进网页状态推送：

```python
{"url": "ws://ros2:9090", "topic": "/cmd_vel", "connected": True,
 "messages": 18243, "age_s": 0.021, "error": ""}
```

`error` 保存最近一次连接失败的原因，连接成功后清空。

---

## `class Ros2Publisher`

向一个 ROS 2 话题发布消息，断线后自动重连。

与订阅器相反，待发槽**深度恒为 1**：速度指令是连续流，旧值对下游没有意义，
积压只会让车延迟响应。新帧直接覆盖尚未发出的旧帧，被覆盖的帧计入 `dropped`，
并在每次成功发送后清零（"当前积压"而非"历史累计"）。

### 构造

```python
Ros2Publisher(url=None, topic="/cmd_vel_web", message_type="geometry_msgs/msg/Twist")
```

参数含义同 `Ros2Subscriber`。

### `start()`

启动后台发布线程。连接建立后会先发 `advertise`，再按需发 `publish`。

### `stop()`

请求发布线程退出并最多等待 2 秒。

退出时**不必**补发停车帧：下游 `cmd_vel_mux` 有 200 ms 静默超时，本链路停止
后会自动归零。

### `publish(message)`

非阻塞投递一帧，可以直接在 socket 回调里调用（不会阻塞框架的消息派发）。

```python
web.publish({
    "linear":  {"x": 0.05, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0,  "y": 0.0, "z": 0.0},
})
```

消息结构必须与该话题的消息类型一致 —— 本类不做字段校验和限幅，边界检查应在
调用方完成（例如 `main.py` 里的 `control.clamp_cmd`）。

### `stats()`

```python
{"url": "ws://ros2:9090", "topic": "/cmd_vel_web", "connected": True,
 "sent": 512, "dropped": 0, "error": ""}
```

`sent` 是累计发出帧数，只增不减；`dropped` 是**当前尚未发出的帧数**——每成功发送
一帧就归零，只有发帧线程跟不上输入（持续积压）时才会一直大于 0。因此它可以
直接当告警判据用：稳态恒为 0，若 `sent` 长期不增长而 `dropped` 持续变大，说明
链路不通或 ws 写入被阻塞。

---

## 消息字段

消息内容由话题类型决定，两个类都原样收发，不做解释：

```python
message, age_s, connected = link.snapshot()
linear  = message.get("linear", {}).get("x", 0.0)     # Twist 线速度 m/s
angular = message.get("angular", {}).get("z", 0.0)    # Twist 角速度 rad/s
```

`/joy` 的 `sensor_msgs/Joy` 结构为 `{"axes": [...], "buttons": [...]}`。

---

## 环境变量

| 变量 | 默认值 | 说明 |
|---|---|---|
| `ROSBRIDGE_URL` | `ws://ros2:9090` | 由 `brick_config.yaml` 注入 |
| `ROS_DOMAIN_ID` | `0` | 仅影响容器内节点，Python 侧不读取 |

---

## 常量

下表是 `__init__.py` 里的模块级常量，与容器内 `common/topics.py` 保持一致。

| 名称 | 值 | 说明 |
|---|---|---|
| `CMD_VEL_TOPIC` | `/cmd_vel` | `Ros2Subscriber` 的默认订阅话题 |
| `WEB_CMD_TOPIC` | `/cmd_vel_web` | `Ros2Publisher` 的默认发布话题 |
| `DEFAULT_URL` | `ws://ros2:9090` | 兜底地址 |
| `RECONNECT_DELAY_S` | `1.0` | 断线重连间隔 |
| `RECV_TIMEOUT_S` | `0.5` | 单次接收超时，便于及时响应 `stop()` |
| `SEND_POLL_S` | `0.2` | 发布线程的唤醒周期 |
| `OPEN_TIMEOUT_S` | `3.0` | 建链超时 |
