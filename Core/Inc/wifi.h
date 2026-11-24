#ifndef __WIFI_H__
#define __WIFI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ================= USART3 第三控制源（预留） =================
 *
 * 架构：与蓝牙(bt.c)完全对等——USART3 被实现为"接入通用指令协议栈的一路
 *   串口控制源"：
 *   - USART3 底层驱动手写在 usart.c 的 USER CODE 区
 *       （huart3：PB10=TX、PB11=RX，115200 8N1、无流控）；
 *   - USART3_IRQHandler 在 stm32f4xx_it.c 的 USER CODE 区实现（与 USART1/2 对称）；
 *   - 本模块调用 UartProto_Init() 把该串口注册进协议栈；
 *   - 收到的 `$<CMD>[,<ARG>...]*<CK>\r\n` 裸帧与 USART1/USART2 走同一解析引擎，
 *     响应也会被协议栈发回"最后发出命令 '$' 的那个串口"。
 *
 * 当前状态：USART3 引脚已初始化，**未接外部设备**。
 *   ESP32-S3 通过 Wi-Fi/TCP 连鲲鹏，不占用 STM32 串口（见 esp32/README.md）；
 *   串口直连方案未启用。将来接模块时按同一裸帧协议接线即可，无需改本模块。
 *
 * 接线（模块 -> STM32F407VET6）：
 *   模块.TX -> PB11 (USART3_RX)
 *   模块.RX -> PB10 (USART3_TX)
 *   GND 共地；电平 3.3V TTL
 */

/* 初始化：注册 USART3 控制源（USART3 底层驱动在 usart.c USER CODE 区） */
void WIFI_Init(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __WIFI_H__ */
