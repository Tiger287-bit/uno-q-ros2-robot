#include <Arduino.h>

#include "heading_imu.h"
#include "lidar.h"
#include "lidar_bridge.h"
#include "motion_bridge.h"
#include "motion_controller.h"
#include "motor_driver.h"
#include "vm_monitor.h"

/**
 * @description         : 初始化底盘与雷达
 * @return              : void
 * @note                : 顺序有依赖——电机引脚先置安全态，Bridge 先就绪再挂 RPC，
 *                        雷达解析线程最后启动，避免它在 RPC 还没注册时就开始抢串口。
 */
void setup() {
  motorInitPins();
  motion::init();
  motionBridgeBegin();
  motion::startWorker();
  headingImuBegin();
  vmMonitorBegin();
  lidar::begin();            // Serial1 + 解析线程，不碰任何电机引脚
  motionBridgeRegister();    // 含 ros_* 与 lidar_*
  lidar::start();            // 发电机命令，SCAN 由解析线程发出
}

/**
 * @description         : 让出执行时间，轮速、航向与雷达解析都在各自线程里跑
 * @return              : void
 */
void loop() {
  delay(2);
}
