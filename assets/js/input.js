/**
 * 输入层：摇杆与键盘统一成归一化指令 (px, py)，并维护摇杆球与速度读数。
 *
 * 为什么几何计算留在这里而不是 ui.js：摇杆的可达半径同时被"渲染球心位置"和
 * "把指针坐标换算成轴值"使用，两处必须用同一个数，否则满舵时球会冲出圆环。
 *
 * 输出：写 state.px / state.py，并刷新本模块负责的三处显示
 * （摇杆球、按键高亮、指令读数）。网络下发由 net.js 定时读取。
 */

import { DEADZONE } from "./config.js";
import { $, fixed } from "./dom.js";
import { state } from "./state.js";
import { setText, showLocal } from "./ui.js";

/* 摇杆球半径 px，对应 CSS 的 width:52px 与 margin:-26px。 */
const KNOB_R = 26;
/* CSS 里摇杆的固定直径，仅在布局尚未就绪（rect.width 为 0）时兜底。 */
const STICK_PX = 168;

const stick = $("stick");
const knob = $("knob");
let stickActive = false;
let activePointer = null;                        // 正在拖摇杆的 pointerId
// 存的是**物理按键**（w / arrowup / a ...），不是方向名。
// 以前存方向名：W 与方向键上都映射到 "up"，两个一起按住再松一个，
// up 就被删掉了 —— 另一个键明明还按着，车却提前停。
const held = new Set();

/** 该事件是否来自当前拖动中的指针。没有 pointerId 的事件（合成/旧环境）一律放行。 */
function ownsPointer(event) {
  return event.pointerId === undefined || event.pointerId === activePointer;
}

/** 摇杆球心的可达半径 px。渲染与拖拽共用，避免两处各写一个数导致对不上。 */
export function stickRadius(rect) {
  const w = rect.width > 0 ? rect.width : STICK_PX;
  return Math.max(1, w / 2 - KNOB_R);
}

/** 可达半径占摇杆直径的百分比（168px 下为 34.5%）。 */
export function stickRangePercent(rect) {
  const w = rect.width > 0 ? rect.width : STICK_PX;
  return (stickRadius(rect) / w) * 100;
}

export function renderStick() {
  // 渲染与拖拽必须共用同一个可达半径：早先这里写死 44%，而 pointToAxes 的限幅
  // 是 rect.width/2-26 = 58px（34.5%），满舵时球心画到 74px、球边缘到 100px，
  // 而圆环半径只有 84px —— 球会明显冲出圆环 16px。按同一常量换算就没有偏差。
  const rect = stick.getBoundingClientRect();
  const pct = stickRangePercent(rect);
  knob.style.left = (50 + state.px * pct) + "%";
  knob.style.top = (50 - state.py * pct) + "%";
}

export function renderKeys() {
  document.querySelectorAll(".kbd").forEach((el) => {
    el.classList.toggle("on", activeDirections().has(el.dataset.key));
  });
}

/** 当前速度比例（滑块百分比）。 */
function speedScale() {
  return Number($("speedScale").value) / 100;
}

/** 归一化指令换算成物理速度。px 向右 → angular.z 取负（ROS 左手系）。 */
export function currentCmd() {
  const scale = speedScale();
  return {
    linear: state.py * state.limits.linear * scale,
    angular: -state.px * state.limits.angular * scale,
  };
}

export function renderCmdReadout() {
  const cmd = currentCmd();
  setText("cmdLinear", fixed(cmd.linear, 3));
  setText("cmdAngular", fixed(cmd.angular, 3));
}

/* 刷新速度比例读数。上限来自后端，首帧到达前用常量兜底。
   除了百分比还要给出绝对速度：100% 也才 0.099 m/s，只看"60%"会高估车能跑多快。
   角速度一并显示：手柄满杆是不打折的 1.20 rad/s，而滑块默认 60% 只有 0.72，
   两路换着用时这个差异会被误当成"车变肉了"，写在读数里比放文档里管用。 */
export function refreshScale() {
  const pct = Number($("speedScale").value);
  const speed = (pct / 100) * state.limits.linear;
  const turn = (pct / 100) * state.limits.angular;
  setText("speedScaleOut", pct + "% · " + (speed * 100).toFixed(1) + " cm/s · "
    + Math.round(turn * 180 / Math.PI) + "°/s");
  renderCmdReadout();
}

/** 一次输入变化后同步三处显示。 */
export function applyInput() {
  renderStick();
  renderKeys();
  renderCmdReadout();
}

/* ---- 摇杆拖拽 ---- */

/** 指针坐标 → 归一化轴值；超出圆环时按模长限幅到 1，落在死区内归零。 */
export function pointToAxes(event) {
  const rect = stick.getBoundingClientRect();
  const cx = rect.left + rect.width / 2;
  const cy = rect.top + rect.height / 2;
  const radius = stickRadius(rect);            // 减去摇杆球半径，留边
  let dx = (event.clientX - cx) / radius;
  let dy = (cy - event.clientY) / radius;      // 向上为正
  const len = Math.hypot(dx, dy);
  if (len > 1) { dx /= len; dy /= len; }
  if (Math.hypot(dx, dy) < DEADZONE) { dx = 0; dy = 0; }
  return [dx, dy];
}

function onPointerDown(event) {
  if (state.locked) return;
  // 只认鼠标左键：右键菜单、中键滚动不该顺带把操控权抢走（pointerdown 对这些
  // 按键同样触发，且 setPointerCapture 会让后续 pointermove 全被当成拖拽）。
  if (event.pointerType === "mouse" && event.button !== 0) return;
  // 已有指针在拖就不再接管：第二根手指按下来若覆盖状态，抬手时先松开的那根
  // 会触发 endDrag，另一根还按着却已经不算拖拽，表现为"手指还压着、车停了"。
  if (stickActive) return;
  event.preventDefault();
  stickActive = true;
  activePointer = event.pointerId === undefined ? null : event.pointerId;
  stick.classList.add("dragging");
  stick.setPointerCapture(event.pointerId);
  [state.px, state.py] = pointToAxes(event);
  applyInput();
}

function onPointerMove(event) {
  if (!stickActive || !ownsPointer(event)) return;
  [state.px, state.py] = pointToAxes(event);
  applyInput();
}

function endDrag(event) {
  if (!stickActive) return;
  // 绑在 window 上的 pointerup 会收到"别的指针"的抬手事件（例如另一根手指、
  // 外接鼠标的第二键），那不是这次拖拽的结束。
  if (event && event.pointerId !== undefined && activePointer !== null
    && event.pointerId !== activePointer) return;
  stickActive = false;
  activePointer = null;
  stick.classList.remove("dragging");
  // 松手时键盘可能还按着：交给 syncFromKeys 判断是归零还是继续走。
  // 这里若直接清零，会出现"按着 W 拖一下摇杆，松手后永久停车"——keydown
  // 被 held 挡着不再触发，除非先松开 W 再按一次。
  syncFromKeys();
}

/* ---- 键盘 ---- */
const KEY_MAP = {
  w: "up", arrowup: "up",
  s: "down", arrowdown: "down",
  a: "left", arrowleft: "left",
  d: "right", arrowright: "right",
};
const ARROW_KEYS = new Set(["arrowup", "arrowdown", "arrowleft", "arrowright"]);

/** 当前生效的方向集合：由按住的物理按键逐个映射后合并。
 *  这样 W 与 ↑ 同时按住时，松开任意一个，另一个仍在集合里。 */
function activeDirections() {
  const dirs = new Set();
  for (const raw of held) {
    const dir = KEY_MAP[raw];
    if (dir) dirs.add(dir);
  }
  return dirs;
}

function syncFromKeys() {
  if (stickActive) return;                    // 拖拽优先，键盘不抢
  const dirs = activeDirections();
  let x = 0, y = 0;
  if (dirs.has("up")) y += 1;
  if (dirs.has("down")) y -= 1;
  if (dirs.has("left")) x -= 1;
  if (dirs.has("right")) x += 1;
  // 斜向（W+A 等）原始模长是 1.41，必须先归一化：否则摇杆球同样会冲出圆环，
  // 且同一个斜向用鼠标推和用键盘按会得到不同的速度（pointToAxes 是归一化的）。
  const len = Math.hypot(x, y);
  if (len > 1) { x /= len; y /= len; }
  state.px = x;
  state.py = y;
  applyInput();
}

/** 该按键是否该让给表单控件。文本类控件全让；滑块只让方向键（方向键调数值），
 *  W A S D 仍然开车 —— 否则点过一次滑块，键盘遥控就无声失效了。 */
function inFormControl(event) {
  const target = event.target;
  if (!(target instanceof HTMLInputElement)) return false;
  if (target.type !== "range") return true;
  return ARROW_KEYS.has(String(event.key).toLowerCase());
}

function onKeyDown(event) {
  const raw = event.key.toLowerCase();
  if (!KEY_MAP[raw]) return;
  if (inFormControl(event)) return;
  event.preventDefault();
  if (state.locked) { showLocal("遥控已锁定，请先解除"); return; }
  if (held.has(raw)) return;          // 按住不放的重复 keydown 直接忽略
  held.add(raw);
  syncFromKeys();
}

function onKeyUp(event) {
  const raw = event.key.toLowerCase();
  if (!KEY_MAP[raw]) return;
  // 收键永远先清状态，不看焦点在哪个控件上：按下 W 后把焦点移到滑块再松手，
  // keyup 的 target 就是 <input>，早先在这里 return 会把 up 永远留在 held 里 ——
  // 车保持全速，之后按 W/S 也不生效（keydown 被拦），只能靠急停。
  if (!held.has(raw)) return;
  event.preventDefault();
  held.delete(raw);
  syncFromKeys();
}

/** 失焦 / 切标签页 / 断线时必须清空按键与摇杆，否则收不到 keyup，车会一直跑。 */
export function releaseAll() {
  held.clear();
  // 指针状态也要一并清掉：切标签页或 alt-tab 时 pointerup 未必送达，
  // 残留的 stickActive 会让之后"没按住、只是划过摇杆"持续操舵。
  stickActive = false;
  activePointer = null;
  stick.classList.remove("dragging");
  state.px = 0;
  state.py = 0;
  applyInput();
}

/** 绑定全部输入监听。由 main.js 显式调用，模块导入本身无副作用。 */
export function initInput() {
  stick.addEventListener("pointerdown", onPointerDown);
  stick.addEventListener("pointermove", onPointerMove);
  stick.addEventListener("pointerup", endDrag);
  stick.addEventListener("pointercancel", endDrag);
  // 指针捕获在异常情况下可能直接丢失（触控被系统接管、设备热插拔、浏览器
  // 切换 Window），此时既没有 pointerup 也没有 pointercancel，摇杆会卡在
  // 最后一次的偏转上一直操舵。lostpointercapture 是这条路径上唯一的兜底。
  stick.addEventListener("lostpointercapture", endDrag);
  window.addEventListener("pointerup", endDrag);

  document.addEventListener("keydown", onKeyDown);
  document.addEventListener("keyup", onKeyUp);
  window.addEventListener("blur", releaseAll);
  document.addEventListener("visibilitychange", () => { if (document.hidden) releaseAll(); });

  $("speedScale").addEventListener("input", refreshScale);
}
