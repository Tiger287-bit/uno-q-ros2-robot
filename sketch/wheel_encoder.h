#pragma once

#include <stdint.h>

/**
 * @description         : 初始化两个正交编码器的输入和双边沿中断
 * @return              : void
 */
void encoderBegin();

/**
 * @description         : 获取同一时刻的双轮编码器计数快照
 * @param count         : 输出 M1、M2 原始累计计数，按 32 位无符号值回绕
 * @param invalid       : 输出 M1、M2 的无效跳变次数
 * @return              : void
 */
void readCounters(uint32_t (&count)[2], uint32_t (&invalid)[2]);
