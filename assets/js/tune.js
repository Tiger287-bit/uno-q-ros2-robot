/**
 * 调试参数面板：按 /api/tune 的描述渲染输入框，改动批量写回 MCU。
 *
 * 参数为什么不走 30 Hz 状态帧：一共 12 项、几秒才变一次，塞进 robot_status
 * 只会让每帧都多背一份不变的常量。这里单独 5 s 轮询，写入走 POST。
 *
 * 面板结构完全由后端描述驱动（分组、标签、单位、范围、步长），前端不硬编码
 * 任何参数名，增删参数只需要改 python/tune.py 和固件的 TuneParam 枚举。
 */

import { $ } from "./dom.js";
import { showLocal } from "./ui.js";

const REFRESH_MS = 5000;

let meta = null;        // 最近一次后端描述
let dirty = new Set();  // 有未提交改动的 key
let busy = false;       // 有请求在途：期间不做自动刷新，免得把结果盖掉

function inputs() {
  return document.querySelectorAll("#tuneGroups input");
}

function findInput(key) {
  return document.querySelector('#tuneGroups input[data-key="' + key + '"]');
}

function buildPanel(data) {
  const host = $("tuneGroups");
  host.textContent = "";
  for (const group of data.groups) {
    const items = data.params.filter((p) => p.group === group.key);
    if (!items.length) continue;

    const section = document.createElement("div");
    section.className = "tune-group";

    const head = document.createElement("div");
    head.className = "tune-group-head";
    const label = document.createElement("span");
    label.className = "tune-group-label";
    label.textContent = group.label;
    const note = document.createElement("span");
    note.className = "tune-group-note";
    note.textContent = group.note || "";
    head.append(label, note);

    const grid = document.createElement("div");
    grid.className = "tune-grid";
    for (const param of items) grid.append(buildItem(param));

    section.append(head, grid);
    host.append(section);
  }
}

function buildItem(param) {
  const row = document.createElement("label");
  row.className = "tune-item";

  const name = document.createElement("span");
  name.className = "tune-k";
  name.textContent = param.label;

  const input = document.createElement("input");
  input.type = "number";
  input.dataset.key = param.key;
  input.min = String(param.min);
  input.max = String(param.max);
  input.step = String(param.step);
  input.value = String(param.value);
  input.addEventListener("input", onEdit);
  input.addEventListener("blur", onEdit);
  input.addEventListener("keydown", (event) => {
    if (event.key === "Enter") {
      event.preventDefault();
      apply();
    }
  });

  const unit = document.createElement("span");
  unit.className = "tune-u";
  unit.textContent = param.unit || "";

  row.append(name, input, unit);
  return row;
}

/** 输入框变化：只和"后端当前值"比，避免来回改又改回时仍算脏。 */
function onEdit(event) {
  const input = event.target;
  const key = input.dataset.key;
  const param = meta && meta.params.find((p) => p.key === key);
  if (!param) return;
  const value = Number(input.value);
  const changed = input.value !== "" && Number.isFinite(value)
    && Math.abs(value - param.value) > 1e-9;
  if (changed) dirty.add(key); else dirty.delete(key);
  input.classList.toggle("dirty", changed);
  renderStatus();
}

function renderStatus() {
  const el = $("tuneStatus");
  if (busy) {
    el.textContent = "正在写入…";
    el.className = "tune-status";
    return;
  }
  if (dirty.size) {
    el.textContent = dirty.size + " 项待应用";
    el.className = "tune-status dirty";
    return;
  }
  if (!meta || meta.stale) {
    el.textContent = "参数未同步（MCU 未连接）";
    el.className = "tune-status warn";
    return;
  }
  el.textContent = "已与 MCU 同步";
  el.className = "tune-status ok";
}

/** 用后端值刷新输入框，跳过正在编辑或有未提交改动的项。 */
function syncValues(params) {
  for (const param of params || []) {
    const input = findInput(param.key);
    if (!input) continue;
    if (dirty.has(param.key) || input === document.activeElement) continue;
    const next = String(param.value);
    if (input.value !== next) input.value = next;
  }
}

export async function loadTune() {
  if (busy) return;
  try {
    const resp = await fetch("/api/tune", { cache: "no-store" });
    if (!resp.ok) throw new Error("http " + resp.status);
    const data = await resp.json();
    const first = meta === null;
    meta = data;
    if (first) buildPanel(data);
    syncValues(data.params);
    renderStatus();
  } catch (err) {
    $("tuneStatus").textContent = "读取失败：" + err.message;
    $("tuneStatus").className = "tune-status warn";
  }
}

async function post(body) {
  const resp = await fetch("/api/tune", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
    cache: "no-store",
  });
  if (!resp.ok) throw new Error("http " + resp.status);
  return resp.json();
}

/** 把后端结果落到界面：成功的项清脏，被拒的留着让用户就地改。 */
function handleResult(data) {
  if (!data) return;
  meta = data;
  const errors = data.errors || [];
  if (!data.applied) {
    // 控制环还没轮到这批请求：保持 dirty，提示稍后刷新。
    showLocal(data.error || "参数尚未执行，请稍后刷新", 6000);
    return;
  }
  const failed = new Set(errors.map((item) => item.key).filter(Boolean));
  for (const key of Array.from(dirty)) {
    if (!failed.has(key)) dirty.delete(key);
  }
  for (const input of inputs()) {
    input.classList.toggle("dirty", dirty.has(input.dataset.key));
  }
  syncValues(data.params);
  if (errors.length) {
    showLocal("未全部生效：" + errors.map((item) => item.error).join("；"), 8000);
  } else {
    showLocal("参数已下发");
  }
  renderStatus();
}

async function apply() {
  if (busy) return;
  const values = {};
  for (const key of dirty) {
    const input = findInput(key);
    if (!input) continue;
    const value = Number(input.value);
    if (Number.isFinite(value)) values[key] = value;
  }
  if (!Object.keys(values).length) {
    renderStatus();
    return;
  }
  busy = true;
  renderStatus();
  try {
    handleResult(await post({ values }));
  } catch (err) {
    showLocal("调参失败：" + err.message, 6000);
  } finally {
    busy = false;
    renderStatus();
  }
}

async function reset() {
  if (busy) return;
  busy = true;
  renderStatus();
  try {
    const data = await post({ reset: true });
    dirty.clear();
    for (const input of inputs()) input.classList.remove("dirty");
    handleResult(data);
  } catch (err) {
    showLocal("恢复默认失败：" + err.message, 6000);
  } finally {
    busy = false;
    renderStatus();
  }
}

/** 绑定按钮并启动轮询。由 main.js 显式调用。 */
export function initTune() {
  $("tuneApply").addEventListener("click", apply);
  $("tuneReset").addEventListener("click", reset);
  loadTune();
  setInterval(loadTune, REFRESH_MS);
}
