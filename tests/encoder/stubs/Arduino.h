#pragma once
#include <cstdint>

/* Arduino API 的最小替身：只实现 wheel_encoder.cpp 真正用到的那几个符号。
   引脚编号与真实板子无关，A0..A5 用 14..19 只是为了让测试可读。 */

enum { A0 = 14, A1 = 15, A2 = 16, A3 = 17, A4 = 18, A5 = 19 };
enum { INPUT_PULLUP = 2, CHANGE = 3 };

extern int g_pinState[64];          // 假引脚电平，测试直接写。
extern int (*g_readHook)(int pin);  // 非空时优先于 g_pinState，用于注入抖动。

inline int digitalRead(int pin) { return g_readHook ? g_readHook(pin) : g_pinState[pin]; }
inline void pinMode(int, int) {}
inline void attachInterrupt(int, void (*)(), int) {}
inline int digitalPinToInterrupt(int pin) { return pin; }
