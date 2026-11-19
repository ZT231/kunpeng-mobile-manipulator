/**
  ******************************************************************************
  * @file    acs712.c
  * @brief   ACS712-5A 电流采样：ADC1 注入通道 + 滑动平均 + 零点标定
  *
  * 大白话：ACS712 把「流过 J6 舵机的电流」变成「一个电压」，STM32 用 ADC 读这个电压。
  *         2.5V 表示没电流，每多 0.185V 就是多了 1A。
  *         舵机使劲夹住东西时电流会涨，涨到某个台阶就说明「夹到了」。
  *
  * 规范定义：霍尔效应电流传感器。初级电流在芯片内部导线产生的磁场被霍尔元件检出，
  *         经放大后输出与电流成正比的模拟电压 Vout = Vcc/2 + Sensitivity × I。
  *         ACS712-05B 的 Sensitivity = 185 mV/A，量程 ±5A，带宽 80kHz。
  *
  * 🔴 为什么这一层必须存在（11-16 实机教训）：
  *   舵机是位置控制，它不知道「有没有夹到东西」。J6 发 1550 时可能夹住了、
  *   也可能差一点没碰到，只能靠人眼看。加上电流这一路，才能判断「夹到位没有」。
  ******************************************************************************
  */

#include "acs712.h"
#include "adc.h"        /* hadc1；它本身已 include main.h，所以这里不必再写一次 */

/* ---- 传感器参数（ACS712-05B，5V 供电）---- */
#define ACS712_ZERO_MV_DEFAULT   2500U   /* 无电流时理论输出 Vcc/2 */
#define ACS712_MV_PER_A          185L    /* 灵敏度 185 mV/A */
#define ACS712_VDDA_MV           3300U   /* STM32 ADC 参考电压 */

/* ---- 采样参数 ---- */
/* 🔴 电流读数的采样点数＝跨一个完整 PWM 周期（见 ACS712_PWM_STEP_MS 的说明）。
   PCA9685 输出 50Hz，一个周期 20ms，所以 20 个点、每点隔 1ms 正好铺满。 */
#define ACS712_AVG_SAMPLES       20U
#define ACS712_PWM_STEP_MS       1U      /* 采样间隔 ms，20 × 1ms = 20ms = 一个 PWM 周期 */
#define ACS712_ZERO_SAMPLES      32U     /* 零点标定：连续快速采即可，不需要跨周期 */
/* 饱和阈值 ACS712_SATURATE_MV 定义在 acs712.h —— 那边一份，供 uart_protocol.c 共用 */

static uint32_t s_zero_mv = ACS712_ZERO_MV_DEFAULT;
static uint32_t s_last_mv = ACS712_ZERO_MV_DEFAULT;
static uint8_t  s_inited  = 0U;

/**
  * @brief  读一次注入通道，换算成 mV。失败返回 0。
  *
  * 🔴 为什么直接写寄存器，而不是调 HAL_ADCEx_InjectedStart / ...PollForConversion：
  *    规则通道由 potentiometer.c 的 HAL_ADC_Start_DMA() 常驻跑着，HAL 内部状态是
  *    HAL_ADC_STATE_BUSY_REG；注入启动会把它改写成 BUSY_INJ_REG，与 DMA 侧反复
  *    争用同一份状态机，属于没必要的耦合。注入转换本身只要三步：
  *        置 JSWSTART → 等 JEOC → 读 JDR1（读 JDR1 会顺带清 JEOC）
  *    直接操作更干净，也不影响 DMA 搬运规则通道的结果。
  */
static uint32_t acs712_read_once_mv(void)
{
    uint32_t raw;
    uint32_t guard = 0U;

    if (s_inited == 0U)
    {
        return 0U;
    }

    ADC1->CR2 |= ADC_CR2_JSWSTART;

    while ((ADC1->SR & ADC_SR_JEOC) == 0U)
    {
        /* 忙等上限 ~1ms（20000 次空转 @168MHz）。
           单次注入转换只要约 4.6µs，即使被规则序列插队也就几十 µs，
           所以正常绝不会等到这里；上限设小一点是为了万一 ADC 没启动时，
           别把串口接收线程卡上十几毫秒（115200 下那是上百字节）。 */
        if (++guard > 20000U)
        {
            return 0U;
        }
    }

    raw = ADC1->JDR1;
    if (raw > 4095U)
    {
        raw = 4095U;
    }

    /* 12 位满量程 4095 对应 VDDA：mv = raw * 3300 / 4095 */
    return (raw * ACS712_VDDA_MV) / 4095U;
}

HAL_StatusTypeDef ACS712_Init(void)
{
    ADC_InjectionConfTypeDef sInj;
    GPIO_InitTypeDef gpio;

    /* 🔴 第一步必须是这句：把 PA6 配成**模拟模式**。
       为什么不能省：HAL_ADC_MspInit（adc.c）里只把规则通道的 PA0~PA5 配成
       GPIO_MODE_ANALOG，PA6 在 .ioc 里从来没出现过，于是复位后它是默认的
       **浮空输入**。浮空输入下 GPIO 的模拟开关是断开的，ADC 采到的不是引脚电压
       而是内部漏电/串扰 —— 2026-11-16 实测：万用表量 PA6 明明是 2.5V（ACS712 零点），
       ADC 却读出 795mV，而且标零后立刻再读就漂了 111mV。
       这一句加上之后读数才是真的。 */
    __HAL_RCC_GPIOA_CLK_ENABLE();      /* MX_GPIO_Init 已开过，这里只是不依赖调用顺序 */
    gpio.Pin   = GPIO_PIN_6;
    gpio.Mode  = GPIO_MODE_ANALOG;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;  /* 模拟模式下实际被忽略，HAL 要求填 */
    HAL_GPIO_Init(GPIOA, &gpio);

    /* 🔴 注意：F4 的 ADC_InjectionConfTypeDef **没有** InjectedSingleDiff /
       InjectedOffsetNumber 这两个字段（那是 L4/H7 才有的），
       照抄别的型号的例程会直接编译不过。本结构体字段以
       Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc_ex.h 为准。 */
    sInj.InjectedChannel                = ADC_CHANNEL_6;      /* PA6 = ADC1_IN6 */
    sInj.InjectedRank                   = ADC_INJECTED_RANK_1;
    sInj.InjectedSamplingTime           = ADC_SAMPLETIME_84CYCLES;
    sInj.InjectedOffset                 = 0U;
    sInj.InjectedNbrOfConversion        = 1U;
    sInj.InjectedDiscontinuousConvMode  = DISABLE;
    sInj.AutoInjectedConv               = DISABLE;
    sInj.ExternalTrigInjecConvEdge      = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
    sInj.ExternalTrigInjecConv          = ADC_INJECTED_SOFTWARE_START;

    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sInj) != HAL_OK)
    {
        /* 配置失败：s_inited 置 0，同时把 last_mv 清零当「无效标记」。
           不清的话上层会看到默认的 2500mV，误以为传感器正常、只是「没电流」，
           而真实情况是这条采样通道压根没配起来。 */
        s_inited  = 0U;
        s_last_mv = 0U;
        return HAL_ERROR;
    }

    s_inited  = 1U;
    s_zero_mv = ACS712_ZERO_MV_DEFAULT;
    s_last_mv = ACS712_ZERO_MV_DEFAULT;
    return HAL_OK;
}

void ACS712_CalibrateZero(void)
{
    uint32_t i;
    uint32_t sum = 0U;
    uint32_t n = 0U;
    uint32_t mv;

    if (s_inited == 0U)
    {
        return;
    }

    for (i = 0U; i < ACS712_ZERO_SAMPLES; i++)
    {
        mv = acs712_read_once_mv();
        if ((mv > 0U) && (mv < ACS712_SATURATE_MV))
        {
            sum += mv;
            n++;
        }
    }

    if (n > 0U)
    {
        s_zero_mv = sum / n;
    }
}

int32_t ACS712_Read_mA(void)
{
    uint32_t i;
    uint32_t sum = 0U;
    uint32_t n = 0U;
    uint32_t n_sat = 0U;
    uint32_t mv;
    int32_t  dmv;

    if (s_inited == 0U)
    {
        s_last_mv = 0U;          /* 未初始化 → 无效标记 */
        return 0;
    }

    for (i = 0U; i < ACS712_AVG_SAMPLES; i++)
    {
        mv = acs712_read_once_mv();
        if (mv == 0U)
        {
            /* 读失败（guard 超时），不参与统计 */
        }
        else if (mv >= ACS712_SATURATE_MV)
        {
            n_sat++;             /* 封顶失真，也不参与统计，但要单独记数 */
        }
        else
        {
            sum += mv;
            n++;
        }

        /* 🔴 关键：必须铺满一个完整 PWM 周期。
           PCA9685 是 50Hz（20ms 周期），舵机电流只在脉宽 950~2000µs 的高电平
           窗口里流动，占空比仅 4.75%~10%。以前连续快采（每次间隔几 µs，全在
           同一个相位上）时，若采样点落在低电平就读到漏电流，且相位随机 ⇒
           读数忽大忽小。2026-11-16 实测：不动时连读 5 次极差 118mA，
           而 J6 从 950 合到 2000 的"信号"只有 107mA —— 完全被相位淹没。
           改成每点隔 1ms 采 20 点后，平均覆盖整个周期，才是真实平均电流。
           注意这里用 if/else 而不是 continue：continue 会跳过下面的延时，
           采样点就不再均匀了。 */
        if ((i + 1U) < ACS712_AVG_SAMPLES)
        {
            HAL_Delay(ACS712_PWM_STEP_MS);
        }
    }

    if (n == 0U)
    {
        /* 一个有效样本都没有。必须区分两种成因，否则上层提示会张冠李戴：
             · 全是饱和 → last_mv 标 3300，上层回 CUR,SAT
             · 全读失败 → last_mv 标 0，上层回 CUR,NA */
        s_last_mv = (n_sat > 0U) ? ACS712_VDDA_MV : 0U;
        return 0;
    }

    s_last_mv = sum / n;
    dmv = (int32_t)s_last_mv - (int32_t)s_zero_mv;

    /* mA = ΔmV / (185 mV/A) * 1000 */
    return (dmv * 1000L) / ACS712_MV_PER_A;
}

uint32_t ACS712_GetLast_mV(void)
{
    return s_last_mv;
}

uint32_t ACS712_GetZero_mV(void)
{
    return s_zero_mv;
}

/************************ (C) COPYRIGHT *****END OF FILE***********************/
