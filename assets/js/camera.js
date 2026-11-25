/**
 * 摄像头面板：MJPEG 预览 + 暂停/继续 + 放大 + 状态轮询。
 *
 * 为什么状态要单独走 REST
 *   MJPEG 是长连接，断流时浏览器不一定会触发 error —— 它只是安静地等下一帧，
 *   画面就停在最后一帧上。所以"还在不在更新"必须看 /api/camera/status 的
 *   age_s：超过 STALE_S 就重设 src，让浏览器重新建连。
 *
 * 为什么暂停要移除 src
 *   只把图片藏起来不会断开 MJPEG 连接，服务端仍占着一个客户端和一个采集槽。
 *   移除 src 才会真正关闭请求。
 */

const STALE_S = 3.0;        // 帧龄超过这个值就认为断流（秒）
const POLL_MS = 2000;       // 状态轮询间隔
const RETRY_MIN_MS = 4000;  // 两次重连的最小间隔，避免设备没插时疯狂建连

const view = {
  playing: false,
  status: null,
  timer: 0,
  lastRestart: 0,
  failures: 0,
};

const $ = (id) => document.getElementById(id);

/** 建立（或重建）MJPEG 连接。加时间戳是为了绕过浏览器对同一 URL 的缓存。 */
function connect() {
  const img = $("cameraFeed");
  if (!img) return;
  view.lastRestart = Date.now();
  img.src = `/api/camera/stream?t=${view.lastRestart}`;
}

/** 播放/暂停。暂停时移除 src，真正断开服务端的 MJPEG 连接。 */
function setPlaying(playing) {
  view.playing = playing;
  const img = $("cameraFeed");
  const button = $("cameraToggle");
  if (img) {
    if (playing) {
      connect();
    } else {
      img.removeAttribute("src");
    }
  }
  if (button) button.textContent = playing ? "暂停" : "继续";
  if (!playing) renderStatus("warn", "已暂停");
}

/** 状态标签：tone 取 ok / warn / bad / info，与全站 .tag 的配色一致。 */
function renderStatus(tone, text) {
  const tag = $("cameraStatus");
  if (!tag) return;
  tag.textContent = text;
  tag.className = `tag ${tone}`;
}

/** 把 status 接口的数据画成一行说明。 */
function renderDetail(data) {
  const line = $("cameraDetail");
  if (!line) return;
  if (!data) {
    line.textContent = "—";
    return;
  }
  if (data.error) {
    line.textContent = data.error;
    return;
  }
  const size = data.width && data.height ? `${data.width}×${data.height}` : "—";
  const fps = data.fps ? `${data.fps.toFixed(1)} fps` : "— fps";
  const clients = data.clients > 0 ? `${data.clients} 个观看端` : "无观看端";
  line.textContent = `${size} · ${fps} · ${clients}`;
}

/** 轮询状态：更新说明文字，并在断流时重连。 */
async function poll() {
  let data = null;
  try {
    const response = await fetch("/api/camera/status", { cache: "no-store" });
    data = await response.json();
  } catch (_error) {
    renderStatus("bad", "状态读取失败");
    return;
  }
  view.status = data;
  renderDetail(data);

  if (!view.playing) return;

  if (data.error) {
    renderStatus("bad", "摄像头不可用");
    return;
  }
  // age_s 为 null 表示启动后还没出过一帧（设备刚插上或正被别的进程占用）。
  const stale = data.age_s === null || data.age_s > STALE_S;
  if (!stale) {
    view.failures = 0;
    renderStatus("ok", "画面已连接");
    return;
  }
  renderStatus("warn", "等待画面…");
  if (Date.now() - view.lastRestart > RETRY_MIN_MS) connect();
}

/** 初始化：取 DOM、绑按钮、开始播放并起轮询。 */
export function initCamera() {
  const card = $("cameraCard");
  const img = $("cameraFeed");
  if (!card || !img) return;

  $("cameraToggle")?.addEventListener("click", () => setPlaying(!view.playing));

  $("cameraExpand")?.addEventListener("click", () => {
    const expanded = card.classList.toggle("expanded");
    const button = $("cameraExpand");
    if (button) button.textContent = expanded ? "还原" : "放大";
  });

  // 双击画面等同点"放大"，和常见监控界面一致。
  img.addEventListener("dblclick", () => $("cameraExpand")?.click());

  // 连接直接失败（设备没插、路由 503）时浏览器会触发 error；
  // 这里只做提示，重连交给 poll 的节流逻辑，避免两处同时建连。
  img.addEventListener("error", () => {
    view.failures += 1;
    renderStatus("bad", "画面未连接");
  });

  setPlaying(true);
  view.timer = window.setInterval(poll, POLL_MS);
  poll();
}
