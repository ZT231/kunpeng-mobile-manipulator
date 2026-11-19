#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "stm32f4xx.h"

#define BT_USART  USART1
#define BT_TX_PIN GPIO_Pin_9
#define BT_RX_PIN GPIO_Pin_10
#define BT_PORT   GPIOA
#define BT_AF     GPIO_AF_USART1
#define BT_BUF_LEN 32

void BT_Init(uint32_t baud);
void BT_SendChar(char ch);
void BT_SendStr(const char *str); 
char BT_Read(void);
void BT_ClearBuf(void);

#endif
