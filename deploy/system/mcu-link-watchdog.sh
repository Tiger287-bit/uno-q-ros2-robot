#!/bin/bash
# MCU 链路看门狗：连续拿不到 MCU 遥测就重启 router + App。
#
# 为什么需要：MCU 与 Linux 之间是 arduino-router 持着 /dev/ttyHS1 的一条串口链路。
# 实测只要 MCU 在 router 还开着串口的时候复位/被烧录，router 会读到一堆
# "invalid packet, expected array" 然后直接把这条连接丢掉；之后它**不会自己重连**，
# App 侧所有 RPC 全部超时（ros_status / ros_motor_diag / lidar_slice），
# 现场表现就是"车完全没反应，重启 App 也没用"。
#
# 恢复顺序有讲究：先重启 router（它会 pulse GPIO70 复位 MCU），再重启 App
# （App 的 Bridge 是在启动时连 router 的，router 重启后旧连接已断，必须一起重启）。
#
# 只读 App 的 HTTP 接口判断活死，不碰 ROS、不占 MCU。
set -u

APP=/home/arduino/ArduinoApps/uno-q-ros2-demo
URL=http://127.0.0.1:7000/api/telemetry
# 连续多少次（每 5 s 一次）没有样本才动手。3 次 = 15 s，够滤掉重启中的空窗。
LIMIT=${LIMIT:-3}

fail=0
while true; do
  sleep 5
  n=$(curl -s --max-time 3 "$URL" \
      | python3 -c 'import sys,json; print(len(json.load(sys.stdin).get("samples",[]))-1)' 2>/dev/null)
  case "$n" in
    ''|*[!0-9]*) n=0 ;;
  esac
  if [ "$n" -ge 1 ]; then
    fail=0
    continue
  fi
  fail=$((fail + 1))
  if [ "$fail" -lt "$LIMIT" ]; then
    continue
  fi
  echo "$(date -Is) MCU 遥测连续 ${fail} 次为空，重启 router 与应用" 
  systemctl restart arduino-router
  sleep 3
  # 用 arduino 身份执行，否则 .cache/ 会被写成 root 所有，下次手动启动就失败。
  sudo -u arduino -H arduino-app-cli app stop "$APP"  >/dev/null 2>&1
  sudo -u arduino -H arduino-app-cli app start "$APP" >/dev/null 2>&1
  fail=0
  sleep 30
done
