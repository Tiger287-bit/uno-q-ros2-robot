#pragma once

/**
 * @description         : 启动 Arduino Bridge 传输层
 * @return              : void
 */
void motionBridgeBegin();

/**
 * @description         : 注册 Linux 侧用于试跑、点动和读取遥测的 RPC 接口
 * @return              : void
 */
void motionBridgeRegister();
