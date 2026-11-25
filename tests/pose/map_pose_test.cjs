/* 直接运行生产地图模块的位姿交互；无浏览器、网络或开发板。 */
const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const vm = require("node:vm");
const source = fs.readFileSync(path.resolve(__dirname, "../../assets/js/map.js"), "utf8")
  .replace(/import \{ emitNav \} from "\.\/net\.js";/, "const emitNav = send;")
  .replace(/export function /g, "function ");

function page(connected = true) {
  let now = 100;
  const elements = {};
  const element = id => elements[id] ??= {
    textContent: "", className: "", disabled: false, classList: { toggle() {} },
  };
  const sent = [];
  const context = vm.createContext({
    document: { getElementById: element }, window: { devicePixelRatio: 1 },
    performance: { now: () => now }, Math, Uint8Array, atob,
    send(payload) { if (connected) sent.push(payload); return connected; },
  });
  vm.runInContext(source, context);
  const run = code => vm.runInContext(code, context);
  run('statusEl=document.getElementById("navStatus"); state.run={mode:"nav",state:"ready"}; state.pose=[0,0,0]; state.age=0.1;');
  const request = () => run('state.click="pose";state.drag={x:0.2,y:0,yaw:0};onPointerUp();state.poseReq');
  return { run, request, sent, elements, advance(ms) { now += ms; } };
}

const first = page();
const req = first.request();
assert.equal(typeof req, "string");
assert.equal(first.sent[0].req, req);
first.run('renderNav({pose_ack:"ok",pose_req:0})');
assert.equal(first.run("state.poseAck"), "waiting");
first.run('renderNav({pose_ack:"ok"})');
assert.equal(first.run("state.poseAck"), "waiting");
first.run(`renderNav({pose_ack:"ok",pose_req:${JSON.stringify(req)}})`);
assert.equal(first.run("state.poseAck"), "ok");
assert.equal(first.run("state.poseReq"), null);
assert.equal(first.run("navReady()"), true);
assert.equal(first.elements.navStatus.textContent, "定位已确认");

const second = page();
assert.notEqual(second.request(), req);
second.advance(12001);
second.run("checkPoseTimeout()");
assert.equal(second.run("state.poseAck"), "timeout");
assert.equal(second.run("navReady()"), false);
assert.match(second.elements.navStatus.textContent, /回执超时/);
assert.notEqual(second.request(), req);

const offline = page(false);
offline.request();
assert.equal(offline.run("state.poseAck"), "timeout");
assert.match(offline.elements.navStatus.textContent, /未发送/);
assert.equal(offline.elements.poseMode.disabled, false);

const legacy = page();
legacy.run('renderNav({pose_ack:"ok",pose_req:0})');
assert.equal(legacy.run("state.poseAck"), "ok");
legacy.run('state.nav="canceled";state.poseAck="timeout";renderStatusLine()');
assert.match(legacy.elements.navStatus.textContent, /定位未确认/);
legacy.run('state.age=1.1;renderModeButtons()');
assert.equal(legacy.elements.navCancel.disabled, false);
console.log("map pose flow: passed");
