#pragma once
/* 主机测试里没有中断，临界区退化成空操作。 */
inline unsigned int irq_lock() { return 0; }
inline void irq_unlock(unsigned int) {}
