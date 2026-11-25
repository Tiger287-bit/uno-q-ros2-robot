# 第一轮 Bug 修复状态

基准：`3ac53a9` · 报告时间：2026-11-25 22:20

| Bug | 优先级 | 状态 | Commit |
|---|---|---|---|
| BUG-001 部署路径硬编码 | P0 | ✅ 已修 | `2f4008b` |
| BUG-002 看门狗判活错误 | P0 | ✅ 已修 | `2f4008b` |
| BUG-003 控制循环阻塞 | P0 | ⬜ 未开始 | — |
| BUG-004 Bridge 并发访问 | P0 | ⬜ 未开始 | — |
| BUG-005 急停被速度缓存覆盖 | P0 | ✅ 已修 | `a3924ed` |
| BUG-006 默认地图缺失 | P1 | ⬜ 未开始 | — |
| BUG-007 依赖管理不完整 | P0 | ✅ 已修 | `7aa32a6` |
| BUG-008 健康检查不完整 | P1 | ⬜ 未开始 | — |
| BUG-009 前端单点故障 | P1 | ⬜ 未开始 | — |
| BUG-010 摄像头阻塞 | P1 | ⬜ 未开始 | — |
| BUG-011 环境版本一致性 | P0 | ⬜ 待实机确认 | — |

---

## BUG-001 部署路径硬编码 ✅

**改动**
- 新增 `deploy/system/unoq-robot.env`：`APP_DIR` / `PORT` / 看门狗调参单点定义
- `unoq-robot-app.service`：`Environment=APP_DIR=…` + `EnvironmentFile=-/etc/default/unoq-robot`
  + `ExecStart=/usr/bin/arduino-app-cli app start ${APP_DIR}`（systemd 展开），Description 去掉项目名
- `mcu-link-watchdog.sh`：从 `/etc/default/unoq-robot` 读 `APP_DIR`/`PORT`

**验证**：`bash -n` 通过；路径已改为配置驱动。**未上板验证**换目录场景。

**遗留**：`deploy/部署说明.md` 里的示例文件名仍含旧项目名（属文档示例，非代码路径）。

---

## BUG-002 看门狗判活错误 ✅

**改动**
- **判据**：`samples 条数 >= 1` → **`last_seq` 连续 STALE_LIMIT 次不前进**。
  旧判据的致命缺陷：缓冲区里留着历史样本时条数恒 ≥1，数据停更也判在线，真失联反而不动手。
- **启动宽限期** `GRACE_S=120`，覆盖编译烧录窗口
- **恢复冷却** `COOLDOWN_S=180`，防重启风暴
- **flock 互斥**，防两个实例同时恢复
- **尊重用户意图**：恢复前查 `app list`，状态 `stopped` 则只重启 router、不启动 App
- **日志**：去掉所有 `>/dev/null 2>&1`，`systemctl` / `app-cli` 输出进 journal

**验证**：`bash -n` 通过。**未上板验证**（触发条件需构造 MCU 失联）。

**未解决**：`app list --format json` 的输出字段未经实机确认（脚本对解析失败做了
`unknown` 兜底，此时不干预、只记日志），需在板子上跑一次核对字段名。

---

## BUG-005 急停被导航速度缓存覆盖 ✅

**问题（本会话引入的回归）**：为修「导航原地转不动」新增的 `NAV_CMD_HOLD_S=0.45`
保持逻辑未检查安全状态，急停后 0.45 s 内会把上一条非零指令重新下发。

**改动**（`python/main.py`）
```python
if PAUSED.is_set() or STOP_PENDING.is_set() or drive_fault:
    nav_hold = (0, 0, False); nav_hold_at = 0.0     # 安全状态优先，清空缓存
elif right or left: ...
```
安全状态优先于保持；解除急停后需重新收到有效运动指令才会再次建立缓存。

**验证**：代码路径确认 + `py_compile`。**未实机验证**急停时序。

---

## BUG-007 依赖管理不完整 ✅

**改动**
- `python/camera.py`：`import cv2` → try/except 软导入，暴露 `CV2_AVAILABLE`/`CV2_ERROR`。
  已确认模块级无任何 `cv2.X` 调用（16 处全在函数内），因此 cv2 缺失时模块仍可导入，
  仅在实际采集时失败，可由 `status()` 上报。
- 新增 `requirements.txt`：只列直接依赖（`opencv-python-headless`），
  明确标注 `arduino` / `ros2_bridge` / `rclpy` 由运行时与容器提供。

**验证**：`py_compile` 通过。**未实机验证**缺 cv2 场景。

---

## 未开始项的计划

### BUG-003 控制循环阻塞（P0，改动最大）
控制线程串行执行 速度下发 / 遥测 / 雷达 / 调参 / trace。方案：
1. 抽出 `RpcDispatcher`：单线程闸门 + 优先级队列，**急停与速度下发走同步快速通道**
2. 低优先级任务（雷达分片、diag、trace）设每拍时间预算（如 8 ms），超预算顺延到下一拍
3. `config.MCU_TIMEOUT_S` 与控制周期解耦：快速通道用 40 ms，慢通道用 200 ms
4. 暴露 `rpc_latency` / `loop_overrun` 指标到 `/api/telemetry`
验收：人为让雷达超时，底盘仍能在 ≤100 ms 内响应停车。

### BUG-004 Bridge 并发访问（P0，依赖 BUG-003）
`api_routes.get_motor_diag()` 直连 Bridge。做法同 `tune`/`selftest`：HTTP 侧投递请求到
队列并等待 seq 回执；或由控制环每 2 s 低频刷新缓存，接口只读缓存 + 时间戳。
**必须先有 BUG-003 的调度器，否则只是把阻塞换个地方。**

### BUG-006 默认地图缺失（P1）
`nav.launch.py` 启动前 `os.path.exists(map_yaml)` 与同名 `.pgm` 检查；缺失时
`LogInfo` 说明原因并跳过 map_server → Nav2 结构未就绪，`mode_manager` 报
「导航不可用（缺地图）」，Base 模式照常可用。

### BUG-008 健康检查（P1）
`brick_compose.yaml` 的 healthcheck 分层：容器存活（process）、rosbridge 连通（9090
+ `/rosapi/topics`）、底盘就绪（`/api/telemetry` 的 `uptime`/`seq` 前进）、导航就绪
（`/app/nav_ui` 的 `mode.state=ready`）。容器 healthcheck 只放前两层，后两层由
`/api/health` 暴露。

### BUG-009 前端单点故障（P1）
`assets/js/main.js`：核心 socket + 控制/急停先初始化并 `await`；地图/摄像头/检测
各自 `try { boot() } catch`，失败只在对应面板显示，不影响 socket。

### BUG-010 摄像头阻塞（P1）
设备路径配置化（`UNOQ_CAMERA_DEV`，默认 `/dev/video0`）；采集线程用
「帧时间戳 + 看门狗」判活，`cap.read()` 放独立线程并设硬超时；连续 N 帧
不更新则释放重开并上报。

### BUG-011 环境版本（P0，待实机确认）
需在板子上取：`arduino-cli core list`、`arduino-app-cli --version`、
`docker images`（基础镜像 tag）、`ros2 --version`、以及一次完整启动日志。
**不做任何盲目升级**，以日志为准。

---

## 明确未验证事项

本轮所有改动均**未经实机运行验证**，只做了静态检查（`bash -n` / `py_compile` /
配置读回）。原因：板上当前处于「App 已启动但 MCU 链路需重启恢复」的状态，
不适合在修复未审查前反复重启。

下一轮上板验证建议顺序：
1. 部署 `mcu-link-watchdog.sh` → 构造失联（`systemctl stop` 后观察 GPU70 复位）
2. 换 App 目录名跑一次启动/停止（BUG-001 验收）
3. 急停时序：导航中按下急停，确认 `mode` 立即 0 且 0.45 s 内不下发非零
