# UNO Q ROS 2 Robot —— 系统性诊断报告

- 日期：2026-11-25
- 范围：启动与部署、MCU 通信、底盘控制、ROS 2、WebUI
- 方法：源码静态审查 + 实机日志/遥测取证 + 对照实验（架空 / 落地 / 阶跃自检）
- 原则：**每条结论后面都附证据；没有实测支撑的只标"推测"**

---

## 一、执行摘要

本次「App 无法正常运行」的**直接原因不是业务代码，而是 MCU 串口链路断开**：

`arduino-router` 与主控之间的 RPC 通道失效后，App 进程仍然能"启动成功"、WebUI 也能开，
但所有底盘 RPC 都卡满超时，表现为「启动了但车不动、页面数据全停」。

链路恢复需要两步（缺一不可）：

```bash
sudo systemctl restart arduino-router          # 重建串口与 socket
arduino-app-cli app stop  <APP>
arduino-app-cli app start <APP>                # 容器重连新 socket
```

只做第一步不够——容器仍持有旧 socket inode，继续报 `Connection refused`（已实测）。

除此之外，本次排查还确认了 5 类**会让"改了代码却不生效"或"误判为硬件故障"**的系统性问题，
它们比单个 bug 更值得优先处理，详见第四节。

---

## 二、启动链路（当前实现）

```
上电
 └─ arduino-router.service        串口 /dev/ttyHS1 + Unix socket
     └─ arduino-app-cli.service   容器编排
         └─ uno-q-ros2-demo-*     三个容器
             ├─ ros2-1            ROS 2 Jazzy：rosbridge / mux / 里程计 / Nav2 / SLAM
             ├─ ei-obj-detection  视觉推理（依赖模型与摄像头）
             └─ main-1            Python 控制环 30 Hz + WebUI :7000 + 遥测
                  └─ Bridge RPC → arduino-router → MCU(Zephyr)
```

**关键耦合**：`main-1` 的 30 Hz 控制环在**同一个线程**里同步调用 RPC（`ros_status` /
`ros_drive_heading` / `lidar_slice`）。任何一次 RPC 阻塞都会直接拉长控制周期。
实测链路故障时 `loop=202 ms`（正常 33 ms），掉到 5 Hz。

**可选功能耦合**：`ei-obj-detection` 与摄像头属于可选能力，但参与同一套 compose 启动；
其故障目前会以"容器不健康"的形式影响整体观感，虽然不影响底盘链路。

---

## 三、本次启动失败的根因（实测证据）

### 3.1 现象

```
docker ps           →  三个容器 Up
curl :7000          →  200
但：
[control] 读状态失败：Request 'ros_status' timed out after 0.2s
[ros]     雷达分片读取失败：Request 'lidar_slice' timed out after 0.2s
[lat]     loop=202.6/270.6/402.7   uptime=-  seq=-
```

`uptime=-  seq=-` 是关键：**完全没有 MCU 数据**，不是"慢"，是"断"。

### 3.2 判定过程

| 步骤 | 观察 | 结论 |
|---|---|---|
| 查 router | `arduino-router` 进程 active | 进程没死 |
| 重启 router | 日志出现 `Opened serial connection /dev/ttyHS1` | 串口重新打开 |
| 再看 App | 仍 `Connection refused` | 容器持有旧 socket |
| 重启 App | `样本数 2, vbat 12.06` | 链路恢复 |

### 3.3 根因

`arduino-router` 重启（或异常退出后由 systemd 拉起）会重建 Unix socket，
而容器内的 `Bridge` 客户端只在**启动时**连接一次，连接断开后不重建，
因此在 socket 被替换后**永久处于 `Connection refused`**，直到容器重启。

`Bridge.read_loop` 有重连尝试，但重连目标是同一个已失效路径，且失败后不触发上层告警——
上层只看到"RPC 超时"，被误读成"MCU 挂了"。

### 3.4 为什么之前"看起来是好的"

早前部署过 `mcu-link-watchdog.service`：检测到遥测断流就 `app stop` + `app start`。
它掩盖了这个缺陷，代价是**用户手动关闭 App 也会被它拉起来**（已按用户要求移除）。
移除后缺陷暴露，本次失败即由此而来。

---

## 四、系统性缺陷（比单个 bug 更值得先修）

### S1. Docker 构建缓存导致"改了不生效"

`COPY ros_ws/src ./src` 在**文件内容变化后仍命中缓存**（实测 `#7 CACHED`），
导致 `nav2.yaml`、`nav_bridge.py` 的修改没有进入镜像，却显示构建成功。

**规避**：构建前写入 `docker/ros_ws/src/.build-stamp`（内容变化即可打断缓存），
或用 `docker build --no-cache`。

### S2. 容器不随镜像更新而重建

`systemctl restart` / `arduino-app-cli app start` **不会重建已存在的容器**。
实测容器仍运行旧镜像 `7921ffb6`，而 `ros2_bridge:local` 已是 `2b2b7c84`。

**判定**：`docker inspect -f '{{.Image}}' <container>` 与镜像 ID 对比。
**规避**：`app stop` → `docker rm -f <container>` → `app start`。

### S3. YAML 重复键静默覆盖

`nav2.yaml` 的 `general_goal_checker` 曾同时存在两组 `xy/yaw_goal_tolerance`，
后写者生效，导致此前多次调参**全部无效**且无任何报错。

**规避**：改配置后必须用 `yaml.safe_load` 读回生效值验证。

### S4. 安全机制与"舒适性机制"互相覆盖

`nav_hold`（导航指令 0.45 s 保持）未检查 `PAUSED / STOP_PENDING / drive_fault`，
急停后会把上一条非零指令重新下发，**最多 0.45 s 内顶回急停**。
本次已修复（安全状态优先清空保持缓存）。

### S5. 控制环与 RPC 超时的量级错配

控制环目标周期 **33.3 ms**，单次 RPC 超时 **200 ms**（曾为 100 ms）。
一次超时＝丢失 6 个控制周期；链路故障时控制环退化为 5 Hz，
且 MCU 侧 3000 ms 看门狗在极端情况下会触发停车。

---

## 五、Bug 清单

### P0（阻断可用性）

| 编号 | 问题 | 证据 | 修复策略 |
|---|---|---|---|
| P0-1 | Router socket 重建后容器不重连，链路永久失效 | `Connection refused` 持续，重启 App 后恢复 | Bridge 增加 socket 健康探测与**按路径重建**重连；或容器侧探活失败即自退由编排拉起 |
| P0-2 | 构建缓存导致镜像不更新 | `COPY ros_ws/src CACHED` | 构建脚本强制 `.build-stamp` 或 `--no-cache` |
| P0-3 | 容器不随镜像重建 | 容器镜像 ID ≠ `local` 镜像 ID | 部署脚本固化 `rm -f` 重建流程 |
| P0-4 | 急停被 `nav_hold` 覆盖 | 代码路径确认 | **已修复** |
| P0-5 | 看门狗判据错误 + 无启动保护（已移除，能力缺失） | 仅比较样本数量，不判新鲜度 | 重做为「只重启 router、不碰 App」+ 启动宽限期 |
| P0-6 | 默认导航模式依赖 `room_20261122.yaml`，仓库内缺失 | `DEFAULT_MAP` 指向不存在文件 | 提供示例地图或在缺失时降级为 SLAM/Base 模式 |

### P1（功能不稳定）

| 编号 | 问题 | 证据 | 修复策略 |
|---|---|---|---|
| P1-1 | `nav2.yaml` 重复键 | 两组 tolerance | **已修复**，并加校验脚本 |
| P1-2 | 导航终点判定与朝向 | `xy 0.08 / yaw 3.15` 曾使车"到了不摆正" | **已恢复** `0.05 / 0.30` + `use_rotate_to_heading: true` |
| P1-3 | `api_routes.get_motor_diag()` 直连 Bridge，绕过统一调度 | 源码 | 改为投递到控制环队列（同 `tune` / `selftest` 模式） |
| P1-4 | 摄像头 / AI / SLAM / Nav2 与基础启动耦合 | compose 单组 | 拆为 profile：`base` / `slam` / `nav` / `vision` |
| P1-5 | 前端可选模块初始化失败会阻断核心 socket | `assets/js/*` 串行初始化 | 每个模块 try/catch 隔离 + 核心链路先就绪 |
| P1-6 | 摄像头设备路径硬编码、`VideoCapture` 可能阻塞 | `camera.py` | 路径可配 + 打开超时 + 失败降级 |
| P1-7 | 部署脚本硬编码旧路径 `uno-q-ros2-demo` | `deploy/*` | 统一从 `app.yaml` 读取 app id |
| P1-8 | 缺少独立 `requirements.txt` | 仓库根目录 | 补全并锁定版本 |

### P2（健壮性 / 可维护性）

| 编号 | 问题 | 修复策略 |
|---|---|---|
| P2-1 | 无启动集成测试、无异常场景回归 | 增加 `tests/integration`：链路断开、镜像不一致、配置重复键 |
| P2-2 | 遥测停更无告警（只能靠页面肉眼） | 暴露 `/api/health`，包含各链路新鲜度 |
| P2-3 | 日志缺少 correlation id，难以串联一次请求 | 关键路径加 `tick`/`session` 标识（现已有部分） |
| P2-4 | README 缺少接线、参数、启动步骤 | 补文档 |

---

## 六、本次已修复

| 项 | 改动 | 影响范围 |
|---|---|---|
| 速度环 | 位置式 PI + 前馈 → **增量式 PID**（参照 Yahboom M1 `PID_Incre_Calc`） | `sketch/motion_controller.cpp` |
| MCU 看门狗 | 1200 ms → **3000 ms**（底盘自主保持） | 同上 |
| 占空比上限 | 255 → **220**（255 档实测零输出） | `motion_controller.h` |
| 导航指令保持 | 新增 `NAV_CMD_HOLD_S = 0.45`，修复"原地转不动" | `python/main.py` |
| 急停优先 | 保持逻辑增加安全前置判断 | 同上（本次） |
| 终点判定 | 删除重复键，恢复 `0.05 / 0.30`，`use_rotate_to_heading: true` | `config/nav2.yaml` |
| 定位 | 去掉 AMCL，`map→odom` 改为可调节点，按位姿合成 | `nodes/start_pose_tf.py` |
| 前端 | 修正画布绘制/点击坐标系不一致 | `assets/js/map.js` |
| 系统服务 | 移除 `unoq-robot-app.service`、`mcu-link-watchdog.service` | 部署侧 |

**实测效果**（阶跃自检，地面）：

```
改前（位置式+前馈）：稳态 -100%（不动）
改后（增量式）      ：1200 cps 稳态 +0%/-3%，纹波 87/79 cps，超调 +4%/+3%
导航 0.4 m          ：succeeded，PWM 99~135（上限 220）
```

---

## 七、分阶段整改计划

**阶段 0（已完成）**：恢复可用——链路恢复流程、增量式速度环、终点判定、安全前置。

**阶段 1（P0，优先）**
1. Bridge 断线自愈：socket 健康探测 + 按路径重建重连 + 上层告警（P0-1）
2. 部署脚本固化：构建打断缓存 + 强制重建容器 + 镜像 ID 校验（P0-2/P0-3）
3. 轻量看门狗重做：只重启 `arduino-router`、启动宽限期 120 s、不触碰 App（P0-5）
4. 地图缺失降级：无地图时自动切 SLAM/Base，不阻塞启动（P0-6）

**阶段 2（P1）**
5. 统一 RPC 调度：所有 RPC 经同一队列，禁止业务层直连（P1-3）
6. Compose profile 解耦：base / slam / nav / vision（P1-4）
7. 前端模块隔离 + 核心链路优先（P1-5）
8. 配置校验脚本（重复键、必需文件、镜像一致性）纳入 CI（P1-1/P1-8）

**阶段 3（P2）**
9. 集成测试与异常回归（P2-1）
10. `/api/health` 与遥测新鲜度告警（P2-2）
11. 文档与部署说明（P2-4）

---

## 八、验证方法（每个阶段必须执行）

1. **静态**：`python -m py_compile`、`node --check`、`yaml.safe_load` 读回生效值
2. **构建**：确认 `colcon build` 实际执行（非 CACHED）、镜像内文件内容比对
3. **部署**：`docker inspect -f '{{.Image}}'` 与镜像 ID 一致
4. **链路**：`/api/telemetry` 有数据、`uptime`/`seq` 非 `-`、`loop` 中位数 ≈ 33 ms
5. **运动**：**必须架空**；先用阶跃自检（`step_p1200`）验证跟随，再落地；落地测试限速 ≤ 0.037 m/s
6. **回归**：每次改动后重跑上述 1~5，并记录到提交信息

---

## 九、明确不做的事

- 不重写已验证的电机驱动、编码器解码、PID 数值与雷达协议（只做参数与结构替换）
- 不在未通过阶跃自检的情况下做落地运动测试
- 不用"看门狗重启 App"的方式掩盖链路缺陷
- 不在缺少实机日志的情况下对硬件（电机/驱动板）下结论

---

## 十、遗留风险

1. **定位**：已去掉 AMCL，位姿＝里程计位姿。每次重启必须重设初始位姿，走久有累积误差。
2. **链路**：串口错帧仍存在（曾 20 分钟 185 次），阶段 1 的自愈只缓解不根治。
3. **速度**：底盘可稳定维持约 0.037 m/s；再快需解决机械阻力。
4. **供电**：实测电池 12.2 V → 11.0 V 持续下降，低电压会放大链路与驱动异常。
