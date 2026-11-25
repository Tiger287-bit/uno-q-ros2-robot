#!/bin/bash
# MCU 链路看门狗。
#
# 背景
#   MCU 与 Linux 之间是 arduino-router 持着 /dev/ttyHS1 的一条串口链路。MCU 在
#   router 还开着串口时复位/被烧录，router 会读到一串 "invalid packet" 后把连接
#   丢掉，且**不会自己重连**；App 侧所有 RPC 全部超时。反过来，router 重启会重建
#   Unix socket，而容器里的 Bridge 只在启动时连一次，之后永久 Connection refused。
#   所以恢复必须"先 router、后 App"两步都做。
#
# 判活依据（旧版的问题就在这一行）
#   旧版数 samples 的**条数**：只要缓冲区里还留着历史样本，条数就 >= 1，
#   即使数据早已停更也判为在线 —— MCU 真失联时它反而不动手。
#   现在比较连续两次轮询的 **last_seq**：序列号不前进 = 数据停更，与缓冲深度无关。
#
# 安全约束
#   * 用户主动 stop 过的 App 绝不拉起（先查 app list 的运行状态）。
#   * 启动宽限期 + 恢复冷却 + flock 互斥，避免编译烧录期间被反复重启。
#   * 所有动作写 journal 与标准输出，不吞异常。
set -u

CONF=${CONF:-/etc/default/unoq-robot}
[ -r "$CONF" ] && . "$CONF"

# 应用目录只在这里定义一次；换目录/改名只需改配置文件。
APP_DIR=${APP_DIR:-/home/arduino/ArduinoApps/uno-q-ros2-demo}
PORT=${PORT:-7000}
URL="http://127.0.0.1:${PORT}/api/telemetry/full?since_seq=0"

POLL_S=${POLL_S:-5}            # 轮询周期
STALE_LIMIT=${STALE_LIMIT:-3}  # 连续多少次 seq 不前进才判定失联（3×5s=15s）
GRACE_S=${GRACE_S:-120}        # 启动宽限期：编译烧录期间不动手
COOLDOWN_S=${COOLDOWN_S:-180}  # 两次恢复之间的最小间隔
LOCK=/run/unoq-mcu-watchdog.lock

log() { logger -t unoq-mcu-watchdog "$*"; echo "$(date -Is) $*"; }

app_state() {
  # 返回 running / stopped / unknown。unknown 时按"不干预"处理。
  WANT_APP=$(basename "$APP_DIR") \
  sudo -E -u arduino -H arduino-app-cli app list --format json 2>/dev/null \
    | python3 -c '
import sys, json, os
want = os.environ.get("WANT_APP", "")
try:
    data = json.load(sys.stdin)
except Exception:
    print("unknown"); raise SystemExit
apps = data if isinstance(data, list) else data.get("apps", [])
for a in apps:
    path = a.get("path") or a.get("appPath") or a.get("dir") or ""
    if want and os.path.basename(str(path).rstrip("/")) == want:
        print(a.get("status") or a.get("state") or "unknown"); raise SystemExit
print("unknown")
' 2>/dev/null || echo unknown
}

last_seq() {
  curl -s --max-time 3 "$URL" | python3 -c '
import sys, json
try:
    d = json.load(sys.stdin)
except Exception:
    print(-1); raise SystemExit
seq = d.get("last_seq")
print(seq if isinstance(seq, int) else -1)
' 2>/dev/null || echo -1
}

restart_router() {
  log "重启 arduino-router（串口 + socket 重建）"
  systemctl restart arduino-router 2>&1 | logger -t unoq-mcu-watchdog -s
}

restart_app() {
  log "重启 App：$APP_DIR"
  # 必须用 arduino 身份，否则 .cache/ 会被写成 root 所有，下次手动启动就失败。
  sudo -u arduino -H arduino-app-cli app stop  "$APP_DIR" 2>&1 | logger -t unoq-mcu-watchdog -s
  sleep 3
  sudo -u arduino -H arduino-app-cli app start "$APP_DIR" 2>&1 | logger -t unoq-mcu-watchdog -s
}

# flock：两个实例同时恢复会互相踩，也会被 systemd 的 Restart=always 放大成风暴。
exec 9>"$LOCK"
if ! flock -n 9; then
  log "另一个实例正在运行，退出"
  exit 0
fi

log "启动：app=$APP_DIR poll=${POLL_S}s stale=${STALE_LIMIT} grace=${GRACE_S}s"
sleep "$GRACE_S"
log "宽限期结束，开始监视"

prev=-1
stale=0
last_recover=0

while true; do
  sleep "$POLL_S"
  state=$(app_state)
  if [ "$state" = "stopped" ]; then
    # 用户主动停的（或尚未启动），按意图处理：不启动、不恢复。
    prev=-1; stale=0
    continue
  fi

  seq=$(last_seq)
  now=$(date +%s)

  if [ "$seq" -ge 0 ] && [ "$seq" != "$prev" ]; then
    prev=$seq; stale=0
    continue
  fi

  stale=$((stale + 1))
  log "遥测序列号未前进（seq=$seq prev=$prev stale=${stale}/${STALE_LIMIT} state=$state）"
  [ "$stale" -lt "$STALE_LIMIT" ] && continue

  if [ $((now - last_recover)) -lt "$COOLDOWN_S" ]; then
    log "冷却中（$((now - last_recover))s < ${COOLDOWN_S}s），跳过本次恢复"
    stale=0
    continue
  fi

  restart_router
  if [ "$state" = "running" ]; then
    restart_app
  else
    log "App 状态为 $state，不启动它（用户可能已手动停止）"
  fi
  last_recover=$(date +%s)
  prev=-1; stale=0
done
