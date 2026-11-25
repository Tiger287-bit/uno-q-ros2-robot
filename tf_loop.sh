#!/bin/bash
# 保证 base_link -> laser_link 的静态变换一直在。
# 容器重建后容器内的 static_transform_publisher 会消失，这里每 5 s 补一次。
# 参数与 /scan 的 frame_id 对应：激光装在车体中心正上方，yaw 0（实测 0° 与车头对齐）。
CONTAINER=uno-q-ros2-demo-ros2-1

while true; do
  if docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    if ! docker exec "$CONTAINER" pgrep -f static_transform_publisher >/dev/null 2>&1; then
      docker exec -d "$CONTAINER" bash -c \
        'source /opt/ros/jazzy/setup.bash && exec ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.10 --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id laser_link'
    fi
  fi
  sleep 5
done
