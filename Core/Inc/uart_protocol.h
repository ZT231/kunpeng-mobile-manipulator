#ifndef __UART_PROTOCOL_H__
#define __UART_PROTOCOL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* 初始化协议栈：注册串口并启动单字节中断接收（可多次调用，支持 USART1/USART2/USART3 多控制源） */
void UartProto_Init(UART_HandleTypeDef *huart);

/* 由 UART 接收回调喂入一个字节，内部做帧状态机解析 */
void UartProto_HandleByte(uint8_t byte);

/* 主循环周期调用：处理接收重启等后台任务 */
void UartProto_Process(void);

/* 周期遥测开关：1=开启 main 循环的周期打印，0=关闭（默认关闭） */
void UartProto_SetStreamLog(uint8_t enable);
uint8_t UartProto_GetStreamLog(void);

#ifdef __cplusplus
}
#endif

#endif /* __UART_PROTOCOL_H__ */
