#!/usr/bin/env bash
# 固件主机测试：把 tests/ 与 sketch/ 拷进 ros2 容器，用 g++ 直接编固件源码。
#
#   bash tests/run.sh                        # 在板上跑，默认找 uno-q-ros2-demo-ros2-1
#   CONTAINER=其他名字 bash tests/run.sh
#
# 为什么借容器：板端本身没有 g++（which g++ 为空），镜像里有；容器又没挂 App
# 目录，所以先 docker cp 一份到 /tmp/apptests。测试读的是 sketch/ 下的真源码，
# 不是副本 —— 改了固件不跑这一步，改动就只被编译器看过一次。
set -euo pipefail

APP=$(cd "$(dirname "$0")/.." && pwd)
CONTAINER=${CONTAINER:-uno-q-ros2-demo-ros2-1}

if ! docker inspect "$CONTAINER" >/dev/null 2>&1; then
  echo "找不到容器 $CONTAINER：先 arduino-app-cli app start user:uno-q-ros2-demo" >&2
  exit 2
fi

docker exec "$CONTAINER" rm -rf /tmp/apptests
docker exec "$CONTAINER" mkdir -p /tmp/apptests
docker cp "$APP/tests" "$CONTAINER:/tmp/apptests/tests" >/dev/null
docker cp "$APP/sketch" "$CONTAINER:/tmp/apptests/sketch" >/dev/null

# 失败即中止：/tmp/apptests 留着方便进去复现。
docker exec "$CONTAINER" bash -lc '
set -e
cd /tmp/apptests/tests/motor
g++ -std=c++17 -I stubs -I ../../sketch -o /tmp/motor_test motor_test.cpp
/tmp/motor_test
cd ../encoder
g++ -std=c++17 -I stubs -I ../../sketch -o /tmp/encoder_test encoder_test.cpp
/tmp/encoder_test
cd ../drive_start
g++ -std=c++17 -I ../../sketch -o /tmp/drive_start_test drive_start_test.cpp
/tmp/drive_start_test
'

docker exec "$CONTAINER" rm -rf /tmp/apptests /tmp/motor_test /tmp/encoder_test /tmp/drive_start_test
