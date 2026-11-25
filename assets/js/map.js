/**
 * 地图与导航面板：画 OccupancyGrid、机器人位姿、规划路径，点击下发目标。
 *
 * 数据从哪来
 *   页面不直连 rosbridge（9090 没有鉴权）。容器内的 nav_bridge 把地图、
 *   位姿、路径、导航状态折成一条 JSON，App 原样转成 socket 事件
 *   "robot_nav"，本模块只负责画。命令走 "nav_cmd" 回同一条链路。
 *
 * 坐标系
 *   OccupancyGrid：origin 是左下角格子；世界坐标 y 向上，画布 y 向下。
 *   所以横向 cellPx 与纵向都要翻一次，见 cellToCanvas / canvasToWorld。
 *
 * 为什么用一张离屏 canvas 存栅格
 *   193x102 个格子逐次 fillRect 在 10 Hz 下会明显吃 CPU；整张画一次
 *   （只在 map.seq 变化时重画），之后每帧只是 drawImage + 叠加。
 */

import { emitNav } from "./net.js";

const FREE_COLOR = [255, 255, 255];
const OCCUPIED_COLOR = [34, 34, 38];
const UNKNOWN_COLOR = [206, 211, 218];
// 页面会话加序号，避免两个页面或刷新后的第一条请求互相命中回执。
const POSE_CLIENT = `${Date.now().toString(36)}-${Math.random().toString(36).slice(2)}`;
const POSE_REPLY_TIMEOUT_MS = 12000; // 队列有效期 6 s + 容器确认 4 s，留出传输余量。
let poseSequence = 0;

const state = {
  map: null,        // { w, h, res, ox, oy, seq }
  grid: null,       // Uint8Array，0=空闲 1=占据 2=未知
  bitmap: null,     // 离屏 canvas
  pose: null,       // [x, y, yaw]
  age: 999,         // 位姿帧龄（秒），超过 1 s 图标画成空心
  path: [],
  nav: "idle",
  note: "",
  dist: null,
  click: "idle",    // 点击模式：idle | goal | pose
  run: { mode: "", state: "", note: "" },  // 运行模式，来自容器
  poseAck: "",      // "", waiting / ok / timeout / needed
  poseReq: null,    // 本页面正在等待的请求标识；null 时接收全局定位状态
  poseSentAt: 0,
  poseError: "",
  ackAt: 0,         // 最近一次"定位已确认"的本地时刻
  goalMark: null,   // [x, y]
  drag: null,       // 拖动中：{x, y, yaw, moved}，松开才下发
};

let canvas = null;
let ctx = null;
let statusEl = null;
let lastMapSeq = -1;
let layout = { cellPx: 1, pad: 6, w: 1, h: 1 };

/** base64 -> Uint8Array。地图每格一字节，2.6 万字符，一次解码即可。 */
function decodeBase64(text) {
  const binary = atob(text);
  const bytes = new Uint8Array(binary.length);
  for (let i = 0; i < binary.length; i += 1) bytes[i] = binary.charCodeAt(i);
  return bytes;
}

/** 把栅格画进离屏 canvas；行序翻转，让 +y 朝上。 */
function buildBitmap() {
  const { w, h } = state.map;
  const off = document.createElement("canvas");
  off.width = w;
  off.height = h;
  const octx = off.getContext("2d");
  const image = octx.createImageData(w, h);
  for (let j = 0; j < h; j += 1) {
    const row = (h - 1 - j) * w;            // 画布第 0 行是地图最上面一行
    for (let i = 0; i < w; i += 1) {
      const value = state.grid[j * w + i];
      const color = value === 1 ? OCCUPIED_COLOR
        : value === 2 ? UNKNOWN_COLOR : FREE_COLOR;
      const p = (row + i) * 4;
      image.data[p] = color[0];
      image.data[p + 1] = color[1];
      image.data[p + 2] = color[2];
      image.data[p + 3] = 255;
    }
  }
  octx.putImageData(image, 0, 0);
  state.bitmap = off;
}

/** 世界坐标 -> 画布像素。 */
function worldToCanvas(x, y) {
  const { cellPx, pad } = layout;
  const { res, ox, oy, h } = state.map;
  return [
    pad + (x - ox) / res * cellPx,
    pad + (h - 1) * cellPx - (y - oy) / res * cellPx,
  ];
}

/** 画布像素 -> 世界坐标。 */
function canvasToWorld(px, py) {
  const { cellPx, pad } = layout;
  const { res, ox, oy, h } = state.map;
  return [
    ox + (px - pad) / cellPx * res,
    oy + ((h - 1) * cellPx - (py - pad)) / cellPx * res,
  ];
}

/** 重新按容器宽度排版：格子保持正方形，整图铺满宽度。 */
function resize() {
  if (!canvas || !state.map) return;
  const width = canvas.parentElement.clientWidth;
  if (width <= 0) return;
  const { w, h } = state.map;
  const dpr = window.devicePixelRatio || 1;
  const cssHeight = Math.round(width * h / w);
  // width 也要显式写：只设 style.height 的话，画布的显示宽度由 width 属性
  // （已乘 dpr）决定，高分屏上会被放大 dpr 倍，点击坐标就整体偏掉。
  canvas.style.width = `${width}px`;
  canvas.style.height = `${cssHeight}px`;
  canvas.width = Math.round(width * dpr);
  canvas.height = Math.round(cssHeight * dpr);
  ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  // pad 归零、格子铺满整幅画布：绘制区域和点击区域必须是同一个矩形，
  // 之前 pad=6 让图内缩了 6 px，而点击按 rect 原始像素算，右下角越点越偏。
  layout = { cellPx: width / w, pad: 0, w: width, h: cssHeight };
  draw();
}

/** 画一帧：地图、路径、目标点、机器人。 */
function draw() {
  if (!ctx) return;
  const { w: cw, h: ch } = layout;
  const dpr = window.devicePixelRatio || 1;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, cw, ch);

  if (!state.map || !state.bitmap) {
    ctx.fillStyle = "#FFFFFF";
    ctx.fillRect(0, 0, cw, ch);
    ctx.fillStyle = "#9CA3AF";
    ctx.font = "13px sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("等待地图…", cw / 2, ch / 2);
    return;
  }

  const { cellPx, pad } = layout;
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(state.bitmap, pad, pad, state.map.w * cellPx, state.map.h * cellPx);

  // 规划路径
  if (state.path.length > 1) {
    ctx.strokeStyle = "#4F46E5";
    ctx.lineWidth = 2;
    ctx.beginPath();
    state.path.forEach(([x, y], index) => {
      const [cx, cy] = worldToCanvas(x, y);
      if (index === 0) ctx.moveTo(cx, cy); else ctx.lineTo(cx, cy);
    });
    ctx.stroke();
  }

  // 目标点
  if (state.goalMark) {
    const [gx, gy] = worldToCanvas(state.goalMark[0], state.goalMark[1]);
    ctx.strokeStyle = "#DC2626";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(gx, gy, 6, 0, Math.PI * 2);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(gx - 9, gy); ctx.lineTo(gx + 9, gy);
    ctx.moveTo(gx, gy - 9); ctx.lineTo(gx, gy + 9);
    ctx.stroke();
  }

  drawRobot();
  drawDragPreview();
}

/** 机器人：朝向三角形 + 定位新鲜度。位姿过期时画成灰色空心。 */
function drawRobot() {
  if (!state.pose) return;
  const [x, y, yaw] = state.pose;
  const [cx, cy] = worldToCanvas(x, y);
  const fresh = state.age === undefined || state.age < 1.0;
  const size = Math.max(7, Math.min(12, layout.cellPx * 5));
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(-yaw);                     // 画布 y 向下，世界 yaw 逆时针为正
  ctx.beginPath();
  ctx.moveTo(size, 0);
  ctx.lineTo(-size * 0.7, size * 0.62);
  ctx.lineTo(-size * 0.7, -size * 0.62);
  ctx.closePath();
  if (fresh) {
    ctx.fillStyle = "#16A34A";
    ctx.fill();
  } else {
    ctx.strokeStyle = "#9CA3AF";
    ctx.lineWidth = 2;
    ctx.stroke();
  }
  ctx.restore();
}

/** 能不能操作地图：建图模式、或正在切换时都不能。 */
function navUsable() {
  return state.run.mode === "nav" && state.run.state !== "switching";
}

/**
 * 能不能发导航目标。四个条件缺一不可：
 *   导航模式且不是切换中、有位姿、位姿新鲜（1 s 内）、定位已确认。
 * "等待确认/确认超时/需要重设"三种状态都不放行 —— 后端也会再挡一次。
 */
function navReady() {
  if (!navUsable() || state.run.state !== "ready" || !state.pose) return false;
  if (typeof state.age === "number" && state.age > 1.0) return false;
  return !["waiting", "timeout", "needed"].includes(state.poseAck);
}

/** 模式按钮的选中/禁用状态。切换中禁用，避免连点产生两条相位的请求。 */
function renderModeButtons() {
  const navButton = document.getElementById("modeNav");
  const slamButton = document.getElementById("modeSlam");
  const label = document.getElementById("modeLabel");
  const busy = state.run.state === "switching";
  const mode = state.run.mode;
  if (navButton) {
    navButton.classList.toggle("active", mode === "nav");
    navButton.disabled = busy;
  }
  if (slamButton) {
    slamButton.classList.toggle("active", mode === "slam");
    slamButton.disabled = busy;
  }
  // 建图模式下没有 Nav2/AMCL，这些操作只会失败，直接禁用。
  // 位姿待确认时禁发目标，但"重设初始位姿"必须留着 —— 那是唯一的解法。
  const targetButton = document.getElementById("navMode");
  const cancelButton = document.getElementById("navCancel");
  const poseButton = document.getElementById("poseMode");
  if (targetButton) targetButton.disabled = !navReady();
  // 取消必须独立于定位状态：定位变旧/切换失败时，恰恰更需要能停车。
  if (cancelButton) cancelButton.disabled = state.run.mode === "slam";
  if (poseButton) poseButton.disabled = !navUsable();
  if (label) {
    let text = "模式未知";
    let tone = "warn";
    if (busy) {
      text = "切换中…";
      tone = "warn";
    } else if (state.run.state === "error") {
      text = `模式切换失败${state.run.note ? "：" + state.run.note : ""}`;
      tone = "bad";
    } else if (mode === "nav") {
      text = "导航模式";
      tone = "ok";
    } else if (mode === "slam") {
      text = "建图模式";
      tone = "info";
    }
    label.textContent = text;
    label.className = `tag ${tone}`;
  }
}

/** 状态文案：运行模式 > 导航状态 > 定位确认 > 定位新鲜度。 */
function renderStatusLine() {
  if (!statusEl) return;
  let text = "未定位";
  let tone = "warn";
  if (state.run.state === "switching") {
    text = "正在切换模式…";
    tone = "warn";
  } else if (state.run.state === "error") {
    text = `切换失败${state.run.note ? " · " + state.run.note : ""}`;
    tone = "bad";
  } else if (state.run.mode === "slam") {
    text = "建图中 · 地图从空白开始";
    tone = "info";
  } else if (state.poseAck === "needed") {
    text = "请重设初始位姿（刚切回导航，旧地图对不上）";
    tone = "warn";
  } else if (state.poseAck === "waiting") {
    text = "已发送，等待定位确认…";
    tone = "warn";
  } else if (state.poseAck === "timeout") {
    text = state.poseError || "定位未确认（AMCL 没有响应）";
    tone = "bad";
  } else if (state.nav === "navigating") {
    text = state.dist === null || state.dist === undefined
      ? "导航中" : `导航中 · 剩余 ${state.dist.toFixed(2)} m`;
    tone = "info";
  } else if (state.poseAck === "ok" && performance.now() - state.ackAt < 5000) {
    text = "定位已确认";
    tone = "ok";
  } else if (state.nav === "succeeded") {
    text = "已到达"; tone = "ok";
  } else if (state.nav === "failed") {
    text = `导航失败${state.note ? " · " + state.note : ""}`; tone = "bad";
  } else if (state.nav === "canceling") {
    text = "正在取消…";
    tone = "warn";
  } else if (state.nav === "canceled") {
    text = "已取消"; tone = "warn";
  } else if (state.notice) {
    text = state.notice;
    tone = "warn";
  } else if (state.pose) {
    text = state.age !== undefined && state.age > 1.0 ? "定位超时" : "就绪";
    tone = state.age !== undefined && state.age > 1.0 ? "warn" : "ok";
  }
  statusEl.textContent = text;
  statusEl.className = `tag ${tone}`;
}

/** socket 帧入口：由 net.js 的 robot_nav 事件调用。 */
export function renderNav(frame) {
  if (!frame || typeof frame !== "object") return;
  if (frame.map) {
    const grid = frame.map;
    if (grid.seq !== lastMapSeq) {
      lastMapSeq = grid.seq;
      state.map = { w: grid.w, h: grid.h, res: grid.res, ox: grid.ox, oy: grid.oy };
      state.grid = decodeBase64(grid.data);
      state.bitmap = null;
      buildBitmap();
      resize();
    }
  }
  if (Array.isArray(frame.pose)) state.pose = frame.pose;
  state.age = frame.age;
  if (frame.mode && typeof frame.mode === "object") {
    const previous = state.run.mode;
    state.run = {
      mode: frame.mode.mode || "",
      state: frame.mode.state || "",
      note: frame.mode.note || "",
    };
    if (previous && previous !== state.run.mode) {
      // 换了一套栈：旧路径与目标都属于上一套地图，清掉免得画错位置
      state.path = [];
      state.goalMark = null;
      state.nav = "idle";
      state.dist = null;
      state.poseAck = "";
      state.poseReq = null;
      state.poseSentAt = 0;
      state.poseError = "";
    }
  }
  if (typeof frame.notice === "string") state.notice = frame.notice;
  // 只认这一次请求的回执：上一轮留下的 ok 不能把本轮的 waiting 覆盖掉。
  if (typeof frame.pose_ack === "string"
      && (state.poseReq === null || frame.pose_req === state.poseReq)) {
    if (frame.pose_ack === "ok" && state.poseAck !== "ok") {
      state.ackAt = performance.now();
    }
    state.poseAck = frame.pose_ack;
    state.poseError = "";
    if (["ok", "timeout", "needed"].includes(frame.pose_ack)) {
      state.poseReq = null;
      state.poseSentAt = 0;
    }
  }
  if (Array.isArray(frame.path)) state.path = frame.path;
  if (typeof frame.nav === "string") state.nav = frame.nav;
  state.note = frame.note || "";
  state.dist = frame.dist;
  renderModeButtons();
  renderStatusLine();
  draw();
}

/** 切换点击模式：设为目标 / 设为初始位姿 / 取消。 */
function setClickMode(mode) {
  state.click = state.click === mode ? "idle" : mode;
  document.getElementById("navMode")?.classList.toggle("active", state.click === "goal");
  document.getElementById("poseMode")?.classList.toggle("active", state.click === "pose");
  if (canvas) {
    canvas.style.cursor = state.click === "idle" ? "default" : "crosshair";
    // 触摸屏上按住拖动要改朝向，必须关掉浏览器自己的滚动/缩放手势
    canvas.style.touchAction = state.click === "idle" ? "auto" : "none";
  }
}

/** 请求切换运行模式（导航 / 建图）。由容器的 mode_manager 执行。 */
function requestRunMode(mode) {
  if (state.run.mode === mode && state.run.state === "ready") return;
  if (state.run.state === "switching") return;
  state.run = { mode: state.run.mode, state: "switching", note: "" };
  renderModeButtons();
  renderStatusLine();
  emitNav({ cmd: "mode", mode });
}

/** 画布坐标 -> 世界坐标；落在画布外返回 null。 */
function pointerWorld(event) {
  const rect = canvas.getBoundingClientRect();
  // 画布可能被 CSS 缩放（width:100% 或外层 transform），rect 是缩放后的尺寸，
  // layout 是 CSS 像素下的尺寸。先归一化，否则点击位置会按缩放比例偏出去。
  const sx = layout.w > 0 ? rect.width / layout.w : 1;
  const sy = layout.h > 0 ? rect.height / layout.h : 1;
  const px = (event.clientX - rect.left) / sx;
  const py = (event.clientY - rect.top) / sy;
  if (px < 0 || py < 0 || px > layout.w || py > layout.h) return null;
  return canvasToWorld(px, py);
}

/** 拖动预览：锚点处画半透明机器人，虚线指向当前朝向。 */
function drawDragPreview() {
  if (!state.drag) return;
  const { x, y, yaw } = state.drag;
  const [ax, ay] = worldToCanvas(x, y);
  const [hx, hy] = worldToCanvas(x + Math.cos(yaw) * 0.35,
                                 y + Math.sin(yaw) * 0.35);
  ctx.save();
  ctx.strokeStyle = "rgba(79,70,229,0.75)";
  ctx.setLineDash([4, 3]);
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(ax, ay);
  ctx.lineTo(hx, hy);
  ctx.stroke();
  ctx.setLineDash([]);
  const size = Math.max(7, Math.min(12, layout.cellPx * 5));
  ctx.translate(ax, ay);
  ctx.rotate(-yaw);
  ctx.beginPath();
  ctx.moveTo(size, 0);
  ctx.lineTo(-size * 0.7, size * 0.62);
  ctx.lineTo(-size * 0.7, -size * 0.62);
  ctx.closePath();
  ctx.fillStyle = "rgba(79,70,229,0.55)";
  ctx.fill();
  ctx.restore();
}

/**
 * 按下即锚点。设位姿时"按下 -> 拖 -> 松开"定朝向：
 * 松手那一刻指针相对锚点的方向就是车头方向，和 rviz 的 2D Pose Estimate 一致。
 * 不拖直接松手时保留原朝向（设位姿）或按"指向目标"算（设目标），
 * 避免手抖把车头甩到随机方向。
 */
function onPointerDown(event) {
  if (event.pointerType === "mouse" && event.button !== 0) return;
  if (!state.map || state.click === "idle") return;
  if (!navUsable()) {
    if (statusEl) {
      statusEl.textContent = "建图模式下不能导航，先切回导航模式";
      statusEl.className = "tag warn";
    }
    return;
  }
  if (state.click === "goal" && !navReady()) {
    if (statusEl) {
      statusEl.textContent = "先重设初始位姿，再设目标";
      statusEl.className = "tag warn";
    }
    return;
  }
  const point = pointerWorld(event);
  if (!point) return;
  event.preventDefault();
  if (canvas.setPointerCapture) canvas.setPointerCapture(event.pointerId);
  const yaw = state.click === "goal" && state.pose
    ? Math.atan2(point[1] - state.pose[1], point[0] - state.pose[0])
    : (state.pose ? state.pose[2] : 0);
  state.drag = { x: point[0], y: point[1], yaw, moved: false };
  draw();
}

function onPointerMove(event) {
  if (!state.drag) return;
  const point = pointerWorld(event);
  if (!point) return;
  const dx = point[0] - state.drag.x;
  const dy = point[1] - state.drag.y;
  // 2 cm 以上才改朝向：地图 5 cm 一格，低于这个量级基本都是手抖
  if (Math.hypot(dx, dy) >= 0.02) {
    state.drag.yaw = Math.atan2(dy, dx);
    state.drag.moved = true;
    if (statusEl) {
      statusEl.textContent = `朝向 ${Math.round(state.drag.yaw * 180 / Math.PI)}°`;
      statusEl.className = "tag info";
    }
  }
  draw();
}

function onPointerUp() {
  const drag = state.drag;
  if (!drag) return;
  state.drag = null;
  const mode = state.click;
  setClickMode("idle");
  if (mode === "pose") {
    // 不做本地乐观更新：命令可能没发出去（断线/队列里被挤掉），
    // 先把图标挪过去等于骗人。等服务端把 AMCL 的真实位姿算出来，
    // 下一帧 pose 自己会到位；pose_ack 负责告诉用户成没成。
    state.poseAck = "waiting";
    state.poseReq = `${POSE_CLIENT}:${++poseSequence}`;
    state.poseSentAt = performance.now();
    state.poseError = "";
    state.ackAt = 0;
    if (!emitNav({ cmd: "pose", x: drag.x, y: drag.y, yaw: drag.yaw,
                   req: state.poseReq })) {
      state.poseAck = "timeout";
      state.poseSentAt = 0;
      state.poseError = "未连接，初始位姿未发送，请重试";
    }
  } else if (mode === "goal") {
    state.goalMark = [drag.x, drag.y];
    emitNav({ cmd: "goal", x: drag.x, y: drag.y, yaw: drag.yaw });
  }
  renderModeButtons();
  renderStatusLine();
  draw();
}

/** 命令没到容器或回执断流时，页面也必须结束等待并允许重新设位姿。 */
function checkPoseTimeout() {
  if (state.poseReq === null || state.poseAck !== "waiting") return;
  if (performance.now() - state.poseSentAt <= POSE_REPLY_TIMEOUT_MS) return;
  state.poseAck = "timeout";
  state.poseError = "初始位姿回执超时，请重试";
  renderModeButtons();
  renderStatusLine();
}

function onPointerCancel() {
  state.drag = null;
  renderStatusLine();
  draw();
}

/** 初始化：取 DOM、绑事件、按窗口尺寸重排。 */
export function initMap() {
  canvas = document.getElementById("mapCanvas");
  statusEl = document.getElementById("navStatus");
  if (!canvas) return;
  canvas.addEventListener("pointerdown", onPointerDown);
  canvas.addEventListener("pointermove", onPointerMove);
  canvas.addEventListener("pointerup", onPointerUp);
  canvas.addEventListener("pointercancel", onPointerCancel);
  document.getElementById("navMode")?.addEventListener("click", () => setClickMode("goal"));
  document.getElementById("poseMode")?.addEventListener("click", () => setClickMode("pose"));
  document.getElementById("navCancel")?.addEventListener("click", () => {
    state.goalMark = null;
    emitNav({ cmd: "cancel" });
  });
  document.getElementById("modeNav")?.addEventListener("click", () => requestRunMode("nav"));
  document.getElementById("modeSlam")?.addEventListener("click", () => requestRunMode("slam"));
  window.addEventListener("resize", resize);
  window.setInterval(checkPoseTimeout, 500);
  renderModeButtons();
  renderStatusLine();
  draw();
}
