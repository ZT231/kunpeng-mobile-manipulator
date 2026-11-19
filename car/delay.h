#ifndef __DELAY_H
#define __DELAY_H
#include "stm32f4xx.h"

void DWT_Init(void);
uint32_t millis(void);
uint32_t micros(void); 
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

#define SYS_CLK SystemCoreClock

#endif
