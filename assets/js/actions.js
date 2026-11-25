/**
 * 动作层：急停锁存与按钮。把"用户意图"翻译成"改状态 + 通知后端"。
 *
 * 锁存必须同时打到后端：/cmd_vel_joy（手柄）与 /cmd_vel_web（网页）都汇到容器
 * 内 mux 的 /cmd_vel，而消费 /cmd_vel 并下发 MCU 的是 App 控制环。只在前端把
 * 摇杆禁用，手柄照样能开 —— 所以 stop/resume 各要发一条命令。
 */

import { LOCK_ECHO_MS } from "./config.js";
import { $ } from "./dom.js";
import { releaseAll } from "./input.js";
import { emit, sendZero } from "./net.js";
import { state } from "./state.js";
import { renderLock, showLocal } from "./ui.js";

/** 设置锁存并同步全部副作用（视觉、输入清空、强制下一帧下发 0）。 */
export function setLocked(value) {
  state.locked = value;
  renderLock(value);
  releaseAll();
  state.lastSent = { linear: 0, angular: 0 };   // 强制下一次下发，确保车真的收到 0
  sendZero();
}

function onStopClick() {
  // 点完的 1~2 帧服务端状态可能还带着旧的 paused（30 Hz 控制拍推送），
  // 这里先立个时间窗，renderStatus 期间忽略回声。
  state.lockEchoUntil = performance.now() + LOCK_ECHO_MS;

  if (state.locked) {
    setLocked(false);
    // 必须通知后端清锁存：急停锁的是整条下发链路（含手柄），只改本地变量
    // 等于界面解锁、车依然不走。
    emit("resume");
    showLocal("");
    return;
  }
  setLocked(true);
  emit("stop");
}

/** 绑定按钮。由 main.js 显式调用，模块导入本身无副作用。 */
export function initActions() {
  $("stop").addEventListener("click", onStopClick);
  $("reset").addEventListener("click", () => emit("reset"));
  $("calibrate").addEventListener("click", () => {
    if (emit("calibrate")) showLocal("校准航向：请保持小车静止，IMU 就绪后生效");
  });
}
