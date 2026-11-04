#ifndef __BT_H__
#define __BT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ================= HC-05 蓝牙控制源 =================
 *
 * 架构：本工程采用"多控制源"设计——
 *   USART1（PC / OrangePi）和 USART2（HC-05 蓝牙）都接入 uart_protocol.c
 *   的同一套指令引擎，命令语法与响应格式完全一致（见 uart_protocol.h）。
 *   蓝牙对 MCU 而言只是一路普通 UART 透传，因此本模块只负责：
 *   把蓝牙串口"接入协议栈"，并集中管理蓝牙专属逻辑。
 *
 * 接线（HC-05 -> STM32F407VET6）：
 *   HC-05.VCC -> 5V
 *   HC-05.GND -> GND
 *   HC-05.TXD -> PD6 (USART2_RX)
 *   HC-05.RXD -> PD5 (USART2_TX)  建议串 1kΩ 电阻保护
 *
 * 移植步骤（USART2 驱动已手写在 usart.c 的 USER CODE 区）：
 *   1. usart.c/usart.h 的 USER CODE 区已含 huart2 + MX_USART2_UART_Init()
 *      （工程为 CubeMX 虚拟模式，USART2 不会自动生成，故手写且生成后保留）；
 *   2. USART2_IRQHandler 已在 stm32f4xx_it.c 的 USER CODE 区实现；
 *   3. main() 调用 BT_Init(&huart2) 完成注册。
 */

/* 初始化：把蓝牙串口注册进通用指令协议栈（USART2 底层驱动在 usart.c USER CODE 区） */
void BT_Init(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __BT_H__ */
