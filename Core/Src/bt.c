/* ================= HC-05 蓝牙控制源 =================
 *
 * 蓝牙被实现为"接入通用指令协议栈的一路串口控制源"：
 *   - USART2 底层驱动（huart2：PD5=TX、PD6=RX，115200）手写在 usart.c 的
 *     USER CODE 区（工程为 CubeMX 虚拟模式，不会自动生成 USART2）；
 *   - USART2_IRQHandler 在 stm32f4xx_it.c 的 USER CODE 区实现（与 USART1 对称）；
 *   - 本模块调用 UartProto_Init() 把蓝牙串口注册进协议栈；
 *   - 之后蓝牙收到的 $CMD... 帧与 USART1 走同一解析引擎，
 *     响应也会被协议栈发回"最后发出命令'$'的那个串口"（即蓝牙自己）。
 */

#include "bt.h"
#include "uart_protocol.h"

void BT_Init(UART_HandleTypeDef *huart)
{
    /* 注册蓝牙到协议栈：蓝牙/PC 共用同一指令引擎与命令集 */
    UartProto_Init(huart);
}
