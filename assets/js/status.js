/**
 * 状态渲染：把后端状态帧（socket 推送或 REST 兜底）映射成面板读数。
 *
 * 两个数据源的取舍：
 *   - socket 每帧都带完整字段（含 phase / awake / nFAULT / IMU 状态），是实时主源；
 *   - REST /api/telemetry 只是兜底，CSV 列不全，缺的字段必须显式置空，
 *     不能让它们停在断流前的旧值上冒充实时数据。
 */

import {
  ASSIST_NAMES, IMU_NAMES, MCU_STALE_MS, MODE_NAMES, PHASE_NAMES, RATE_MIN_HZ,
  REASON_NAMES, REST_POLL_MS, SOCKET_GRACE_MS,
} from "./config.js";
import { fixed } from "./dom.js";
import { refreshScale } from "./input.js";
import { state } from "./state.js";
import { setLocked } from "./actions.js";
import {
  localMessage, setBadge, setBar, setTag, setText, showError,
} from "./ui.js";

/**
 * /cmd_vel_source 的原始值是话题名，页面只显示"谁在开车"。
 * 值与容器内 cmd_vel_mux 的 input_topics 对应。
 */
const SOURCE_NAMES = {
  "/cmd_vel_joy": "手柄",
  "/cmd_vel_web": "网页",
  "/cmd_vel_nav": "导航",
};

/** 处理一条实时状态帧。 */
export function renderStatus(payload) {
  if (!payload) return;

  // 急停锁存以服务端为准：多开一个页面、或刷新后重连，界面必须和实际下发一致。
  // 点按钮后 1.5 s 内忽略回声帧 —— 服务端状态是随 30 Hz 控制拍推送的，
  // 刚点完的那一两帧可能还带着旧的 paused，会把界面又切回去。
  if (payload.paused !== undefined && performance.now() >= state.lockEchoUntil
    && payload.paused !== state.locked) {
    setLocked(payload.paused);
  }

  // 上限只在两轴都是有限数时才采纳。缺一轴就整天跳过，而不是把另一轴写成
  // undefined —— 那会让 currentCmd() 产出 NaN，JSON 里变成 null，后端
  // clamp_cmd 再归零，表现成"推杆完全不动"且一句报错都没有。
  const limits = payload.limits;
  if (limits && Number.isFinite(limits.linear) && Number.isFinite(limits.angular)) {
    // 上限是常量，只有变化时才刷新读数 —— 别每帧都触发一次 DOM 事件
    const changed = limits.linear !== state.limits.linear
      || limits.angular !== state.limits.angular;
    state.limits.linear = limits.linear;
    state.limits.angular = limits.angular;
    if (changed) refreshScale();
  }

  // connected 只说明 websocket 活着；stale 才是"容器内 mux 静默"——后端按
  // /cmd_vel 断流超过 CMD_TIMEOUT_S（0.4 s）算的。mux 或 rosbridge 的发布端
  // 挂掉时 websocket 照旧，只看 connected 会一直报绿色"已连接"而车不动。
  // 用黄色而非红色：链路是通的，缺的是数据，和"连不上"要能区分开。
  const link = payload.link || {};
  setBadge("linkBadge",
    !link.connected ? "ROS 2 未连接" : (link.stale ? "ROS 2 无数据" : "ROS 2 已连接"),
    !link.connected ? "bad" : (link.stale ? "warn" : "ok"));

  const web = payload.web || {};
  if (web.connected) {
    // dropped 是"当前积压帧数"（后端每次成功发送即清零），持续 > 0 才说明
    // 发帧线程跟不上输入；偶发覆盖一帧不会再让徽章永久变黄。
    setBadge("webBadge", "网页指令 " + (web.sent || 0) + " 帧",
      web.dropped > 0 ? "warn" : "ok");
  } else {
    setBadge("webBadge", "网页指令未连接", "bad");
  }

  const mcu = payload.mcu;
  // 状态帧龄：payload.actual.age_s 就是"最近一次真正读到 MCU 状态距今多久"。
  // 链路活着但 RPC/固件停了时，payload.mcu 会一直是断链前那份快照——
  // 车轮速度、PWM 全部冻结，徽章若还写"MCU 空闲"就是在拿旧数据报健康。
  const mcuAgeMs = payload.actual && typeof payload.actual.age_s === "number"
    ? payload.actual.age_s * 1000 : null;
  const mcuStale = mcuAgeMs !== null && mcuAgeMs > MCU_STALE_MS;
  if (mcu && mcuStale) {
    setBadge("mcuBadge", "MCU 状态过期", "warn");
  } else if (mcu) {
    const bad = !mcu.ready || mcu.mode === 2 || !mcu.fault_pin;
    setBadge("mcuBadge", "MCU " + (MODE_NAMES[mcu.mode] || "未知"), bad ? "bad" : "ok");
  } else if (payload.assist_reason === "starting") {
    // 上电宽限期内还没有第一帧状态，属于正常启动过程，不能当故障报红
    setBadge("mcuBadge", "MCU 启动中", "info");
  } else {
    setBadge("mcuBadge", "MCU 无响应", "bad");
  }

  // 采样率归零就是"采集停了"：后端在最新一帧超过 3 s 时回 0.0。此时读数全是
  // 停更前那一帧，继续显示"5.0 Hz"等于把失联当健康。
  const rate = payload.rate_hz;
  if (typeof rate === "number" && rate < RATE_MIN_HZ) {
    setBadge("rateBadge", "采样停更", "warn");
  } else {
    setBadge("rateBadge", (rate || 0).toFixed(1) + " Hz", "info");
  }

  // 车实际收到的指令（mux 仲裁后的 /cmd_vel）
  const car = payload.cmd || {};
  setText("carCmd", fixed(car.linear, 3) + " / " + fixed(car.angular, 3));
  // 谁在开车：手柄/网页是人工，导航是 Nav2。人工优先，见 cmd_vel_mux。
  setText("cmdSource", SOURCE_NAMES[payload.cmd_source] || "—");
  setText("assist", ASSIST_NAMES[payload.assist_reason]
    ? ASSIST_NAMES[payload.assist_reason] + "（" + Math.round(payload.assist || 0) + "）"
    : "—");

  if (mcu) {
    renderWheels(mcu);
    renderPose(mcu);
    renderController(mcu);
  }

  // accepted === null 有两种含义：没有指令（目标为 0），或刚推杆但还在固件
  // SETTLE_MS 宽限期内、尚未起转。用 targets 区分，别把后者显示成"被拒绝"。
  const wantMove = (payload.targets || []).some((v) => v !== 0);
  const acked = payload.accepted;
  setText("ack", (acked === null || acked === undefined)
    ? (wantMove ? "等待起转" : "—")
    : (acked ? "已接受" : "被拒绝"));

  // 服务端错误优先；没有错误时保留未过期的本地提示（"请保持静止"这类要看得见）
  showError(payload.error || localMessage());
}

function renderWheels(mcu) {
  if (!mcu.wheels || mcu.wheels.length < 2) return;
  const [m1, m2] = mcu.wheels;
  setText("m1Target", m1.target + " cps");
  setText("m1Speed", fixed(m1.speed, 0));
  setText("m1Pwm", m1.pwm + " / 255");
  setBar("m1Bar", m1.speed);

  setText("m2Target", m2.target + " cps");
  setText("m2Speed", fixed(m2.speed, 0));
  setText("m2Pwm", m2.pwm + " / 255");
  setBar("m2Bar", m2.speed);
}

function renderPose(mcu) {
  const imu = mcu.imu || {};
  setText("yaw", fixed(imu.yaw_deg, 2) + " °");
  setText("rate", fixed(imu.rate_dps, 2) + " °/s");
  setTag("imuState", IMU_NAMES[imu.state] || "未知", imu.state === 2 ? "ok" : "warn");

  const battery = mcu.battery || {};
  setText("voltage", battery.voltage === null || battery.voltage === undefined
    ? "—" : battery.voltage + " V");
}

function renderController(mcu) {
  // 空闲是正常状态，不该配告警色；黄色留给"能跑但没在跑"的中间态
  setTag("mode", MODE_NAMES[mcu.mode] || "未知",
    mcu.mode === 0 ? "" : (mcu.mode === 1 ? "ok" : "bad"));
  const reason = REASON_NAMES[mcu.reason] ?? ("代码 " + mcu.reason);
  const faultWheel = mcu.mode === 2 && mcu.fault?.reason === mcu.reason && mcu.fault.wheel;
  setText("reason", faultWheel ? "M" + faultWheel + " " + reason : reason);
  setText("phase", PHASE_NAMES[mcu.phase] || "未知");
  setText("awake", mcu.awake ? "已使能" : "休眠");
  setText("nfault", mcu.fault_pin ? "正常" : "故障");
}

/* ---- REST 兜底 ---- */

/** 是否有兜底请求在途。setInterval 不等 await，没有它就会并发重叠。 */
let pollBusy = false;

/**
 * 从 /api/telemetry 的 CSV 样本里取最后一行刷新面板。
 * 仅在 socket 静默 SOCKET_GRACE_MS 后接管：两路数据交替覆盖会让数字来回抖。
 */
export async function pollTelemetry() {
  // 单飞：后端慢于轮询周期时 setInterval 不会等上一次 await，两三个请求会叠在
  // 一起，晚到的旧响应覆盖新读数（面板数字往回跳）。丢一拍没有代价，下一拍
  // 500 ms 后照旧。
  if (pollBusy) return;
  if (performance.now() - state.lastSocketAt < SOCKET_GRACE_MS) return;

  pollBusy = true;
  try {
    // 必须带超时：后端把请求挂住时（REST 线程池占满、容器卡在 Bridge 上）
    // 这个 await 永不返回，finally 不执行，pollBusy 会永久为真 —— 兜底轮询
    // 从此停摆，而它正是 socket 断线时唯一的刷新通道。
    // AbortSignal.timeout 在 iOS < 16 / Chrome < 103 里不存在，直接调用会抛
    // TypeError（同样被 catch 吞掉），所以先做特性检测，拿不到就退化成无超时。
    const signal = typeof AbortSignal.timeout === "function"
      ? AbortSignal.timeout(REST_POLL_MS * 2) : undefined;
    const resp = await fetch("/api/telemetry", { cache: "no-store", signal });
    // 500 的错误体不是样本：不判 ok 就会把它当数据解析，面板静默停在旧值上。
    if (!resp.ok) throw new Error("http " + resp.status);
    const data = await resp.json();
    const samples = data.samples || [];
    if (samples.length < 2) return;

    const header = samples[0];
    const row = samples[samples.length - 1];
    // 列名不存在时 indexOf 返回 -1，取值会误拿末列（seq）。必须显式判空。
    const cell = (name) => {
      const i = header.indexOf(name);
      return i < 0 ? null : row[i];
    };
    const num = (name) => {
      const v = cell(name);
      return v === null || v === undefined ? null : Number(v);
    };

    // CSV 有的列全部补上：只更新 speed/yaw 会让面板出现"速度是新值、
    // 目标与 PWM 是 socket 断前旧值"的混合读数，比整齐地空着更容易误判。
    for (const tag of ["m1", "m2"]) {
      const target = cell(tag + "_target");
      const pwm = cell(tag + "_pwm");
      setText(tag + "Target", target === null ? "—" : target + " cps");
      setText(tag + "Speed", fixed(num(tag + "_speed"), 0));
      setText(tag + "Pwm", pwm === null ? "—" : pwm + " / 255");
      setBar(tag + "Bar", num(tag + "_speed") || 0);
    }

    setText("carCmd", fixed(num("cmd_linear"), 3) + " / " + fixed(num("cmd_angular"), 3));
    setText("yaw", fixed(num("yaw"), 2) + " °");
    setText("rate", fixed(num("rate"), 2) + " °/s");
    const vbat = cell("vbat");
    setText("voltage", vbat === null ? "—" : vbat + " V");

    const mode = num("mode");
    const reason = num("reason");
    if (mode !== null) {
      setTag("mode", MODE_NAMES[mode] || "未知", mode === 0 ? "" : (mode === 1 ? "ok" : "bad"));
    }
    if (reason !== null) {
      setText("reason", REASON_NAMES[reason] !== undefined
        ? REASON_NAMES[reason] : ("代码 " + reason));
    }

    // 这几项 CSV 里没有（phase / awake / nFault / IMU 状态 / 保向修正）：
    // 一律置空，宁可不显示，也不要让它们停在 socket 断流前的值上冒充实时数据。
    setText("phase", "—");
    setText("awake", "—");
    setText("nfault", "—");
    setText("assist", "—");
    setTag("imuState", "—", "");

    // 兜底轮询也要能核对锁存：socket 断流时本地按钮写着"已锁定"，实际可能
    // 早被别的页面解除了，两边对不上比不显示更危险。
    if (typeof data.paused === "boolean" && performance.now() >= state.lockEchoUntil
      && data.paused !== state.locked) {
      setLocked(data.paused);
    }

    // 明确标出"当前读数来自兜底轮询"，避免和实时推送混淆；采样也停了就直说
    const restRate = typeof data.rate_hz === "number" ? data.rate_hz : null;
    setBadge("rateBadge",
      restRate !== null && restRate < RATE_MIN_HZ
        ? "采样停更 · 兜底" : fixed(restRate, 1) + " Hz · 兜底",
      "warn");
  } catch (err) {
    // 必须留痕：静默吞掉的话，rateBadge 会停在上次成功时的"x Hz · 兜底"，
    // 面板写着"我在兜底"，实际兜底早断了。只碰 rateBadge —— linkBadge 和
    // connBadge 描述的是 socket 链路，与 REST 请求成败无关。
    setBadge("rateBadge", "兜底中断", "warn");
  } finally {
    pollBusy = false;
  }
}

/** 启动兜底轮询。由 main.js 显式调用。 */
export function startPolling() {
  return setInterval(pollTelemetry, REST_POLL_MS);
}
