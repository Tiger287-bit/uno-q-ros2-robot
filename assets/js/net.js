/**
 * 网络层：socket.io 上行链路 + 20 Hz 指令下发 + REST 兜底触发。
 *
 * 上行通道为什么走 App 自己的 socket.io 而不是直连 rosbridge：
 *   网页拿不到容器内的 ROS 2 图，9090 也没有映射到宿主机（没有鉴权，开了
 *   等于局域网内任何人发一帧就能驱动整车）。所以网页只发"归一化速度"，
 *   由 App 进程转成 /cmd_vel_web，再经容器内 mux 仲裁。
 *
 * 约定：本模块不碰业务渲染，收到的状态帧原样交给 main.js 注入的回调。
 */

import { SEND_INTERVAL_MS, ZERO_EPS } from "./config.js";
import { currentCmd, releaseAll, setOnline } from "./input.js";
import { state } from "./state.js";
import { setBadge, showLocal } from "./ui.js";

let socket = null;
let onStatus = null;
let onNav = null;

/** 建立/重建连接。两个处理函数由 main.js 注入：遥测帧与导航帧。 */
export function connectSocket(statusHandler, navHandler) {
  onStatus = statusHandler;
  onNav = navHandler;

  // 客户端脚本没加载成功时整页会"看起来正常但完全不动"：读数不刷新、按钮
  // 点了没反应。这里显式报错，比让人对着"页面未连接"猜原因省事得多。
  // （历史上踩过：App Lab 不提供 /socket.io/socket.io.js，那个路径返回 400。）
  if (typeof io !== "function") {
    setBadge("connBadge", "socket.io 未加载", "bad");
    showLocal("socket.io 客户端未加载：检查 assets/vendor/socket.io.min.js 是否存在");
    return;
  }

  if (socket) socket.disconnect();
  socket = io();

  socket.on("connect", () => {
    setBadge("connBadge", "实时已连接", "ok");
    state.lastSocketAt = performance.now();
    // 重连也要清一次：断线期间产生的输入不能带过来，必须重新操作。
    setOnline(true);
  });

  socket.on("disconnect", () => {
    setBadge("connBadge", "页面未连接", "bad");
    setOnline(false);      // 内部会 releaseAll() 并锁住指令输出
  });

  socket.on("robot_status", (payload) => {
    state.lastSocketAt = performance.now();
    if (onStatus) onStatus(payload);
  });

  // 导航帧：{pose, age, nav, dist, path, map?}。地图只在 seq 变化时附带。
  socket.on("robot_nav", (payload) => {
    state.lastSocketAt = performance.now();
    if (onNav) onNav(payload);
  });
}

export function socketConnected() {
  return Boolean(socket && socket.connected);
}

/**
 * 发一条无参数命令（stop / resume / reset / calibrate）。
 *
 * 必须带 payload。官方 web_ui 的通用事件处理器是
 *     async def handle_generic_event(event, sid, data)
 * data 没有默认值，socket.emit("stop") 会在框架里抛 TypeError
 * （容器日志：missing 1 required positional argument: 'data'），后端回调根本
 * 不会执行 —— 也就是说车其实没收到这条命令，界面却以为发了。
 */
export function emit(type) {
  if (!socketConnected()) { showLocal("未连接，无法下发指令"); return false; }
  socket.emit(type, {});
  return true;
}

/**
 * 下发导航命令（goal / cancel / pose / map）。
 * 坐标是地图坐标系（米、弧度），由 map.js 从画布像素换算好再传进来。
 */
export function emitNav(payload) {
  if (!socketConnected()) { showLocal("未连接，无法下发导航命令"); return false; }
  socket.emit("nav_cmd", payload);
  return true;
}

/** 立即下发一帧零速度，用于锁存/解锁时确保车真的收到 0。 */
export function sendZero() {
  if (!socketConnected()) return;
  socket.emit("cmd_vel", { linear: 0, angular: 0 });
}

function pushCmd() {
  if (!socketConnected()) return;
  const cmd = state.locked ? { linear: 0, angular: 0 } : currentCmd();
  const isZero = Math.abs(cmd.linear) < ZERO_EPS && Math.abs(cmd.angular) < ZERO_EPS;
  const changed = Math.abs(cmd.linear - state.lastSent.linear) > ZERO_EPS
    || Math.abs(cmd.angular - state.lastSent.angular) > ZERO_EPS;

  // 已经停稳且没有新指令：不再占用链路。容器内的 mux 会靠静默超时维持停车。
  if (isZero && !changed) return;

  // volatile：socket 断开/传输不可写时直接丢弃，不进 sendBuffer。
  // 普通 emit 断线期间会排队，重连后整批复放——推着满舵断线再连上，
  // 会先收到一批过期指令，车按 0.2 s 兜底全速窜一下（约 2 cm）。
  // 速度指令是"最新值有效"的流，丢一帧有 50 ms 后的下一帧和 mux 的 0.2 s
  // 静默兜底，比补发旧值安全。停止命令（sendZero / emit）仍走可靠通道。
  socket.volatile.emit("cmd_vel", {
    linear: Number(cmd.linear.toFixed(4)),
    angular: Number(cmd.angular.toFixed(4)),
  });
  state.lastSent = cmd;
}

/** 启动 20 Hz 下发循环。由 main.js 显式调用（模块导入不应产生定时器）。 */
export function startCommandLoop() {
  return setInterval(pushCmd, SEND_INTERVAL_MS);
}
