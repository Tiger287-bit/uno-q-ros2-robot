#pragma once

/**
 * 注册雷达相关的 Bridge RPC。
 * 不调用 Bridge.begin()：Bridge 由 motionBridgeBegin() 初始化，
 * 这里只挂路由，避免重复初始化。
 */
void lidarBridgeRegister();
