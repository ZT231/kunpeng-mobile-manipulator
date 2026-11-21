#ifndef CMSIS_OS_H
#define CMSIS_OS_H
/*
 * ============================================================================
 * cmsis_os.h —— FreeRTOS CMSIS-OS v2 封装层的「假头文件」（host 测试专用）
 * ============================================================================
 *
 * 【为什么要有这个文件】uart_protocol.c 用 osMutexId_t / osMutexAcquire /
 *   osMutexRelease 保护共享缓冲。真机里这些来自 FreeRTOS，PC 上没有。
 *   测试里互斥锁根本不需要真的锁（单线程），所以 osMutex 全部退化成
 *   「恒成功」的空操作——锁的语义保留在接口层，行为由测试替身决定。
 *
 * 【规范定义】互斥量（mutex）是「同一时刻只允许一个执行单元进入临界区」
 *   的同步原语；单线程 host 测试天然满足互斥，故 stub 恒返回成功。
 * ============================================================================
 */
#include <stdint.h>

typedef struct { int unused; } osMutexId_t;

#define osWaitForever 0xFFFFFFFFU

int osMutexAcquire(osMutexId_t m, uint32_t timeout);
int osMutexRelease(osMutexId_t m);
void osDelay(uint32_t ms);

#endif /* CMSIS_OS_H */
