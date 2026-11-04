#include "potentiometer.h"

/* DMA 循环采样缓冲，DMA 持续写入，CPU 每 20ms 定时读取 */
static volatile uint16_t pot_dma_buffer[POT_CHANNEL_COUNT];

/* 滑动平均历史值与累加和 */
static uint16_t pot_history[POT_CHANNEL_COUNT][POT_ADC_SAMPLES];
static uint32_t pot_sum[POT_CHANNEL_COUNT];
static uint8_t  pot_index = 0U;

/* 上次经回滞死区后采纳的输出值，用于滤掉静止时才出现的 ADC 抖动 */
static uint16_t pot_out_last[POT_CHANNEL_COUNT];

static uint16_t pot_apply_hysteresis(uint8_t ch, uint16_t new_val)
{
    uint16_t last = pot_out_last[ch];
    uint32_t diff;

    if (new_val >= last)
    {
        diff = (uint32_t)(new_val - last);
    }
    else
    {
        diff = (uint32_t)(last - new_val);
    }

    /* 变化小于死区 → 沿用上次值，杜绝静止时被噪声推动 */
    if (diff < POT_HYSTERESIS_LSB)
    {
        return last;
    }

    pot_out_last[ch] = new_val;
    return new_val;
}

/* 启动 DMA 循环采样并预热滑动窗口 */
void Pot_Init(ADC_HandleTypeDef *hadc)
{
    uint16_t v;

    for (uint8_t ch = 0U; ch < POT_CHANNEL_COUNT; ch++)
    {
        pot_dma_buffer[ch] = 0U;
        pot_sum[ch] = 0U;
        pot_out_last[ch] = 0U;
        for (uint8_t s = 0U; s < POT_ADC_SAMPLES; s++)
        {
            pot_history[ch][s] = 0U;
        }
    }

    if (HAL_ADC_Start_DMA(hadc, (uint32_t *)pot_dma_buffer, POT_CHANNEL_COUNT) != HAL_OK)
    {
        Error_Handler();
    }

    /* 纯轮询读 DMA 缓冲，关闭传输完成/半传输/错误中断，
     * 避免 Circular 持续转换产生 ~36kHz 无谓中断抢占 CPU 与 USART。 */
    __HAL_DMA_DISABLE_IT(hadc->DMA_Handle, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);

    /* 等待 DMA 完成首轮扫描填充缓冲 */
    HAL_Delay(1);

    /* 用首轮读数预热滑动窗口，避免上电误读为 0 */
    for (uint8_t ch = 0U; ch < POT_CHANNEL_COUNT; ch++)
    {
        v = pot_dma_buffer[ch];
        pot_sum[ch] = (uint32_t)v * POT_ADC_SAMPLES;
        pot_out_last[ch] = v;   /* 同步死区基准，避免首拍与 0 作无效比较 */
        for (uint8_t s = 0U; s < POT_ADC_SAMPLES; s++)
        {
            pot_history[ch][s] = v;
        }
    }
    pot_index = 0U;
}

/* 从 DMA 缓冲读取最新值并做 16 点滑动平均 */
void Pot_ReadAll(Pot_Data_t *data)
{
    uint16_t v;

    if (data == NULL)
    {
        return;
    }

    for (uint8_t ch = 0U; ch < POT_CHANNEL_COUNT; ch++)
    {
        v = pot_dma_buffer[ch];

        pot_sum[ch] -= pot_history[ch][pot_index];
        pot_history[ch][pot_index] = v;
        pot_sum[ch] += v;

        /* 16点平均 + 回滞死区：滤掉静止时的 ADC 抖动 */
        data->adc_value[ch] = pot_apply_hysteresis(
            ch,
            (uint16_t)(pot_sum[ch] / POT_ADC_SAMPLES)
        );
    }

    pot_index = (uint8_t)((pot_index + 1U) % POT_ADC_SAMPLES);
}
