#ifndef __WIFI_H__
#define __WIFI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ================= ESP-01S(WiFi) 控制源 =================
 *
 * 架构：与蓝牙(bt.c)完全对称——ESP-01S 被实现为"接入通用指令协议栈的
 *   一路串口控制源"：
 *   - USART3 底层驱动手写在 usart.c 的 USER CODE 区（huart3：PB10=TX、PB11=RX，115200）；
 *   - USART3_IRQHandler 在 stm32f4xx_it.c 的 USER CODE 区实现（与 USART1/2 对称）；
 *   - 本模块调用 UartProto_Init() 把 WiFi 串口注册进协议栈；
 *   - WiFi 收到的 $CMD... 帧与 USART1/USART2 走同一解析引擎，
 *     响应也会被协议栈发回"最后发出命令'$'的那个串口"（即 WiFi 自己）。
 *
 * 非透传模式：ESP 配为 CIPMUX=1+CIPSERVER=1，TCP→UART 自动转发（带 +IPD 前缀，
 *   协议栈剥除）；UART→TCP 需协议栈发 AT+CIPSEND 包装（见 uart_protocol.c）。
 *
 * 接线（ESP-01S -> STM32F407VET6）：
 *   ESP-01S.VCC  -> 3.3V          (注意：ESP8266 是 3.3V 模块，禁止 5V！)
 *   ESP-01S.GND  -> GND
 *   ESP-01S.RX   -> PB10 (USART3_TX)   建议接 1k 分压/串阻做电平匹配
 *   ESP-01S.TX   -> PB11 (USART3_RX)
 *   ESP-01S.EN/CH_PD -> 3.3V（使能）
 */

/* 初始化：注册 WiFi 控制源并自动配置 ESP-01S（USART3 底层驱动在 usart.c USER CODE 区） */
void WIFI_Init(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __WIFI_H__ */

