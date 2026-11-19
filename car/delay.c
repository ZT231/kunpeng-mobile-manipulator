#include "delay.h"

void delay_us(uint32_t us)
{
    uint32_t tick = SYS_CLK / 1000000UL;
    uint32_t max  = 0xFFFFFFUL / tick;
    if(!us) return;
    if(us > max) us = max;
    SysTick->CTRL = 0;
    SysTick->LOAD = tick * us - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;
    while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
    SysTick->CTRL = 0;
}

void delay_ms(uint32_t ms)
{
    uint64_t total = (uint64_t)ms * 1000;
    uint32_t max = 0xFFFFFFUL / (SYS_CLK / 1000000UL);
    while(total > max)
    {
        delay_us(max);
        total -= max;
    }
    delay_us((uint32_t)total);
}

void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t micros(void)
{
    return DWT->CYCCNT / (SYS_CLK / 1000000UL);
}

uint32_t millis(void)
{
    return DWT->CYCCNT / (SYS_CLK / 1000UL);
}
