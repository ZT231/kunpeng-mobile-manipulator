/* ================= USART3 第三控制源（预留） =================
 *
 * USART3 底层驱动（huart3：PB10=TX、PB11=RX，115200 8N1、无流控）手写在
 * usart.c 的 USER CODE 区；USART3_IRQHandler 在 stm32f4xx_it.c 的 USER CODE 区实现。
 *
 * 本模块只做一件事：把 USART3 注册进通用指令协议栈，与 USART1(PC)、
 * USART2(HC-05 蓝牙) 完全对等——收到 `$<CMD>[,<ARG>...]*<CK>\r\n` 裸帧走同一
 * 解析引擎，响应回发到"最后发出命令 '$' 的那个串口"。
 *
 * 当前接线状态：USART3 引脚已初始化、未接外部设备。
 *   ESP32-S3 走 Wi-Fi/TCP 连鲲鹏，不占用 STM32 串口（见 esp32/README.md）。
 * 将来接模块时按同一裸帧协议接线即可，本文件无需改动。
 */

#include "wifi.h"
#include "uart_protocol.h"

void WIFI_Init(UART_HandleTypeDef *huart)
{
    UartProto_Init(huart);
}
