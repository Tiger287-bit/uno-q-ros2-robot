/**
 * 物品检测结果叠加层。
 *
 * 输入是 /api/detect 的 JSON（后端用官方 object_detection brick 推理），
 * 坐标相对检测时的图像尺寸（360x640），这里换算成画布像素画框。
 *
 * 为什么要单独一个 canvas 而不是后端把框画进 JPEG
 *   检测约 1 fps、视频 15 fps，把框烧进画面会让框在两次检测之间"粘"在
 *   旧位置；叠加层可以每帧独立重画，视频保持流畅，框也能立刻消失。
 *
 * 为什么自己算 contain 矩形
 *   .camera-frame 固定高度、img 用 object-fit: contain，画面两侧（竖屏时）
 *   有黑边。框必须画在"内容区"而不是整个容器上，否则整体偏移。
 */

const POLL_MS = 500;      // 拉取结果的间隔（检测本身约 1 fps，2 Hz 足够跟上）
const STALE_S = 3.0;      // 结果超过这个时间就认为检测停了
const MAX_ROWS = 4;       // 列表最多显示几条
const PALETTE = ["#22c55e", "#3b82f6", "#f59e0b", "#ef4444",
                 "#a855f7", "#06b6d4", "#eab308", "#ec4899"];

const view = {
  data: null,
  timer: 0,
  canvas: null,
  image: null,
  frame: null,
};

const $ = (id) => document.getElementById(id);

/** 同一个类别固定同一种颜色：同一物在相邻帧之间不会变色。 */
function colorOf(label) {
  let hash = 0;
  for (let i = 0; i < label.length; i += 1) {
    hash = (hash * 31 + label.charCodeAt(i)) % 997;
  }
  return PALETTE[hash % PALETTE.length];
}

/** 按容器尺寸与设备像素比重设画布，避免高分屏上框发虚。 */
function resizeCanvas() {
  if (!view.canvas || !view.frame) return;
  const rect = view.frame.getBoundingClientRect();
  if (!rect.width || !rect.height) return;
  const dpr = window.devicePixelRatio || 1;
  const width = Math.round(rect.width * dpr);
  const height = Math.round(rect.height * dpr);
  if (view.canvas.width !== width || view.canvas.height !== height) {
    view.canvas.width = width;
    view.canvas.height = height;
  }
}

/** 画所有检测框。 */
function draw() {
  const canvas = view.canvas;
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  const data = view.data;
  const image = view.image;
  if (!data || !image || !data.objects || !data.objects.length) return;
  if (!data.width || !data.height) return;
  // 检测停了（报错 / 未就绪 / 结果过期）就别再画旧框：视频还在动而框停在
  // 几秒前，比没有框更容易让人误判。上面已经 clearRect，直接返回就是空白。
  if (data.error || !data.ready) return;
  if (data.age_s === null || data.age_s > STALE_S) return;

  const rect = image.getBoundingClientRect();
  if (!rect.width || !rect.height) return;
  // object-fit: contain 的内容区：等比缩放到容器内并居中。
  const scale = Math.min(rect.width / data.width, rect.height / data.height);
  const offsetX = (rect.width - data.width * scale) / 2;
  const offsetY = (rect.height - data.height * scale) / 2;

  // 后续都按 CSS 像素画，缩放交给画布自身的 dpr 变换。
  const dpr = canvas.width / rect.width;
  ctx.scale(dpr, dpr);
  ctx.lineWidth = 2;
  ctx.font = "600 12px system-ui, -apple-system, sans-serif";
  ctx.textBaseline = "top";

  for (const object of data.objects) {
    const [x1, y1, x2, y2] = object.box;
    const x = offsetX + x1 * scale;
    const y = offsetY + y1 * scale;
    const width = (x2 - x1) * scale;
    const height = (y2 - y1) * scale;
    if (width <= 0 || height <= 0) continue;

    const color = colorOf(object.label);
    ctx.strokeStyle = color;
    ctx.strokeRect(x, y, width, height);

    const text = `${object.label} ${Math.round(object.score * 100)}%`;
    const textWidth = ctx.measureText(text).width;
    // 框太靠上时把标签放进框内，避免被画布裁掉。
    const labelY = y >= 18 ? y - 18 : y;
    ctx.fillStyle = color;
    ctx.fillRect(x, labelY, textWidth + 10, 18);
    ctx.fillStyle = "#fff";
    ctx.fillText(text, x + 5, labelY + 3);
  }
}

/** 更新状态标签、说明行与右侧列表。 */
function renderPanel() {
  const tag = $("detectStatus");
  const detail = $("detectDetail");
  const list = $("detectList");
  const data = view.data;

  if (!data) {
    if (tag) { tag.textContent = "检测未就绪"; tag.className = "tag warn"; }
    if (detail) detail.textContent = "—";
    if (list) list.innerHTML = "";
    return;
  }

  const stale = data.age_s === null || data.age_s > STALE_S;
  if (data.error) {
    if (tag) { tag.textContent = "检测不可用"; tag.className = "tag bad"; }
    if (detail) detail.textContent = data.error;
  } else if (!data.ready) {
    if (tag) { tag.textContent = "检测启动中"; tag.className = "tag warn"; }
    if (detail) detail.textContent = "等待推理服务加载模型";
  } else if (stale) {
    if (tag) { tag.textContent = "检测无结果"; tag.className = "tag warn"; }
    if (detail) detail.textContent = `${data.infer_ms} ms/帧 · 结果已过期`;
  } else {
    const count = (data.objects || []).length;
    if (tag) {
      tag.textContent = count ? `检测到 ${count} 个物体` : "画面内无目标";
      tag.className = count ? "tag ok" : "tag info";
    }
    if (detail) {
      detail.textContent = `${data.infer_ms} ms/帧 · ${data.frames} 帧已推理`;
    }
  }

  if (!list) return;
  const objects = (data.objects || []).slice().sort((a, b) => b.score - a.score);
  if (!objects.length) {
    list.innerHTML = "";
    return;
  }
  // 用 textContent 构造而不是拼 HTML：标签来自模型，理论上不该含尖括号，
  // 但模板字符串拼进 innerHTML 是最容易忘的一处注入面。
  list.replaceChildren(...objects.slice(0, MAX_ROWS).map((object) => {
    const row = document.createElement("div");
    row.className = "detect-item";
    const dot = document.createElement("span");
    dot.className = "detect-dot";
    dot.style.background = colorOf(object.label);
    const name = document.createElement("span");
    name.className = "detect-name";
    name.textContent = object.label;
    const score = document.createElement("span");
    score.className = "detect-score";
    score.textContent = `${Math.round(object.score * 100)}%`;
    row.append(dot, name, score);
    return row;
  }));
}

/** 拉一次结果。 */
async function poll() {
  try {
    const response = await fetch("/api/detect", { cache: "no-store" });
    view.data = await response.json();
  } catch (_error) {
    view.data = { ok: false, ready: false, objects: [], error: "状态读取失败" };
  }
  renderPanel();
  draw();
}

/** 初始化：绑定画布、监听尺寸变化、开始轮询。 */
export function initDetect() {
  const canvas = $("detectCanvas");
  const image = $("cameraFeed");
  const frame = document.querySelector(".camera-frame");
  if (!canvas || !image || !frame) return;

  view.canvas = canvas;
  view.image = image;
  view.frame = frame;

  // 窗口缩放、全屏切换、手机转屏都会改变容器尺寸，靠 ResizeObserver
  // 统一处理，比监听 window.resize 更可靠（全屏是容器自身变化）。
  if (typeof ResizeObserver === "function") {
    new ResizeObserver(() => { resizeCanvas(); draw(); }).observe(frame);
  } else {
    window.addEventListener("resize", () => { resizeCanvas(); draw(); });
  }

  resizeCanvas();
  renderPanel();
  view.timer = window.setInterval(poll, POLL_MS);
  poll();
}
