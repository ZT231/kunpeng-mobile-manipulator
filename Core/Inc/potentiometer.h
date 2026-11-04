#ifndef __POTENTIOMETER_H__
#define __POTENTIOMETER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "adc.h"
#include <stdint.h>

/* PA0~PA5 对应 J1~J6，ADC1_IN0~IN5。
 * 扫描顺序（Rank 1~6）在 adc.c 中配置，DMA 缓冲下标 0~5 依次对应 J1~J6。
 */
#define POT_CHANNEL_COUNT   6U      /* 采集通道数 */
#define POT_ADC_SAMPLES     16U     /* 滑动平均深度（越大越平滑） */
#define POT_ADC_MAX         4095U
#define POT_HYSTERESIS_LSB  8U      /* 回滞死区(L)，平均结果变化小于此值沿用上次，滤掉静止抖动 */

typedef struct
{
    uint16_t adc_value[POT_CHANNEL_COUNT];  /* J1~J6 滑动平均后的 ADC 值 */
} Pot_Data_t;

/* 启动 DMA 循环采样并预热滑动窗口 */
void Pot_Init(ADC_HandleTypeDef *hadc);
/* 从 DMA 缓冲读取最新值并做滑动平均 */
void Pot_ReadAll(Pot_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __POTENTIOMETER_H__ */
