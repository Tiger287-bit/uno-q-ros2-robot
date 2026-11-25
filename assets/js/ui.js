/**
 * 展示层：把"值"写进固定的 DOM 结构，不含任何业务判断。
 *
 * 拆分原则：业务语义（什么时候该显示"被拒绝"）留在 status.js / actions.js，
 * 这里只负责"怎么显示"。因此本文件没有 import 任何状态模块。
 */

import { MAX_CPS, LOCAL_MSG_MS } from "./config.js";
import { $ } from "./dom.js";

export function setText(id, text) {
  $(id).textContent = text;
}

export function setTag(id, text, kind) {
  const el = $(id);
  el.textContent = text;
  el.className = "tag" + (kind ? " " + kind : "");
}

export function setBadge(id, text, kind) {
  const el = $(id);
  el.textContent = text;
  el.className = "badge" + (kind ? " " + kind : "");
}

/** 进度条：宽度按 MAX_CPS 归一，负值切换配色（.neg）。 */
export function setBar(id, value) {
  const el = $(id);
  const pct = Math.min(100, Math.abs(value) / MAX_CPS * 100);
  el.style.width = pct.toFixed(1) + "%";
  el.parentElement.classList.toggle("neg", value < 0);
}

/** 错误横幅。传空串即隐藏。 */
export function showError(msg) {
  const el = $("errorBanner");
  if (msg) {
    el.textContent = msg;
    el.classList.add("show");
  } else {
    el.classList.remove("show");
  }
}

/* ---- 一次性本地提示 ---- */
/* 状态帧 5 Hz 会把 errorBanner 无条件重写成服务端错误，点「校准航向」之类的
   一次性提示活不过 200 ms，用户以为没点上。这里给本地提示一个寿命，期间由
   status.js 在"没有服务端错误"时回填。 */
let localMsg = "";
let localMsgUntil = 0;

export function showLocal(msg, ms = LOCAL_MSG_MS) {
  localMsg = msg;
  localMsgUntil = msg ? performance.now() + ms : 0;
  showError(msg);
}

/** 仍在有效期内的本地提示，没有则返回空串。 */
export function localMessage() {
  return performance.now() < localMsgUntil ? localMsg : "";
}

/** 急停锁存的三处视觉反馈：按钮文案/配色、锁定横幅、摇杆禁用。 */
export function renderLock(locked) {
  const stop = $("stop");
  stop.textContent = locked ? "解除锁定" : "急停";
  stop.className = locked ? "warn" : "danger";
  $("lockBanner").classList.toggle("show", locked);
  $("stickCard").classList.toggle("locked", locked);
  $("stick").classList.toggle("disabled", locked);
}
