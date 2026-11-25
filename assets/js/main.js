/**
 * 装配层：把各模块按顺序接起来，全部初始化都发生在这里。
 *
 * 为什么单独一层：其余模块导入时都不产生副作用（不绑事件、不起定时器、
 * 不建连接），这样每个模块都能单独 import 进测试环境。副作用集中在 boot()，
 * 排查"页面一加载就发指令"这类问题时只需要看这一个文件。
 *
 * 启动顺序有讲究：
 *   1. initInput / initActions 先绑事件，避免首帧状态帧到达时按钮还没接上；
 *   2. renderStick + refreshScale 先画一次，避免首帧前摇杆球位置为空；
 *   3. 最后才 connectSocket —— 状态帧一到就会调 renderStatus / renderNav，
 *      此时地图画布与按钮都已就绪。
 */

import { initActions } from "./actions.js";
import { initCamera } from "./camera.js";
import { initDetect } from "./detect.js";
import { initInput, refreshScale, renderStick } from "./input.js";
import { initMap, renderNav } from "./map.js";
import { connectSocket, startCommandLoop } from "./net.js";
import { renderStatus, startPolling } from "./status.js";
import { initTune } from "./tune.js";

function boot() {
  initInput();
  initActions();
  renderStick();
  refreshScale();
  initMap();
  // 摄像头与实时链路互不影响：设备没插时它只显示"摄像头不可用"，
  // 不会挡住 socket 连接或首帧状态渲染。
  initCamera();
  // 检测依赖摄像头画面与 /api/detect，放在 initCamera 之后：
  // 它要拿 #cameraFeed 的渲染尺寸来对齐叠加层。
  initDetect();
  connectSocket(renderStatus, renderNav);
  startCommandLoop();
  startPolling();
  // 调参面板自己走 REST，与实时链路无关，放最后不影响首帧渲染。
  initTune();
}

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", boot);
} else {
  boot();
}
