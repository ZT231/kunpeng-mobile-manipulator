/**
 * chs_tb6612.c — 底盘底层：2×TB6612FNG + 4×JGB37-520(1:90) 霍尔编码器
 * ----------------------------------------------------------------------------
 * 只碰四样东西：
 *   ① TIM4 CH1~CH4：20kHz PWM，CCR 0~4199；
 *   ② GPIO：8 根 DIR（每轮 IN1/IN2）+ 1 根 STBY（两块并联）
 *      + 1 根板载 W25Q16 CS（PB0，常高隔离）；
 *   ③ TIM1/TIM2/TIM3/TIM8：各自一个正交编码器接口，4 倍频计数。
 *
 * 所有硬件写操作只允许在 ChassisTask（20ms 任务）里调用；
 * 协议任务/其它任务只改 chassis.c 里的目标值，不直接碰寄存器，
 * 避免「CCR 已写 500、DIR 却被另一个任务改成反向」这种瞬态直通风险。
 * ============================================================================
 */
#include "chs_tb6612.h"

/* ==================== 内部状态 ==================== */
static TIM_HandleTypeDef s_pwm;                 /* TIM4：四路 PWM */
static TIM_HandleTypeDef s_enc[CHS_TB_NUM];     /* 四个定时器：四路编码器 */

static uint8_t  s_ready = 0U;                   /* 1=初始化完成 */
static int16_t  s_duty[CHS_TB_NUM];             /* 最近一次下发的带符号占空比 */
static uint32_t s_enc_last[CHS_TB_NUM];         /* 上次 CNT 快照（回绕用无符号相减） */
static int32_t  s_enc_total[CHS_TB_NUM];        /* 上电以来累计原始计数 */

/* DIR 引脚表：顺序与 CHS_TB_Wheel 一致 */
static GPIO_TypeDef * const s_dir_port[CHS_TB_NUM] = {
    CHS_TB_DIR_A_PORT, CHS_TB_DIR_B_PORT, CHS_TB_DIR_C_PORT, CHS_TB_DIR_D_PORT
};
static const uint16_t s_dir_pin1[CHS_TB_NUM] = {
    CHS_TB_DIR_A_PIN1, CHS_TB_DIR_B_PIN1, CHS_TB_DIR_C_PIN1, CHS_TB_DIR_D_PIN1
};
static const uint16_t s_dir_pin2[CHS_TB_NUM] = {
    CHS_TB_DIR_A_PIN2, CHS_TB_DIR_B_PIN2, CHS_TB_DIR_C_PIN2, CHS_TB_DIR_D_PIN2
};

/* PWM 通道表：A→CH1  B→CH2  C→CH3  D→CH4 */
static const uint32_t s_pwm_ch[CHS_TB_NUM] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
};

/* 编码器定时器 / 周期 / 引脚 / AF 表 */
static TIM_TypeDef * const s_enc_inst[CHS_TB_NUM] = {
    CHS_TB_ENC_A_TIM, CHS_TB_ENC_B_TIM, CHS_TB_ENC_C_TIM, CHS_TB_ENC_D_TIM
};
static const uint32_t s_enc_period[CHS_TB_NUM] = {
    CHS_TB_ENC_A_PERIOD, CHS_TB_ENC_B_PERIOD, CHS_TB_ENC_C_PERIOD, CHS_TB_ENC_D_PERIOD
};

typedef struct {
    GPIO_TypeDef *port1;
    uint16_t      pin1;
    GPIO_TypeDef *port2;
    uint16_t      pin2;
    uint8_t       af;
} TB_EncPin;

static const TB_EncPin s_enc_pin[CHS_TB_NUM] = {
    { CHS_TB_ENC_A_PORT1, CHS_TB_ENC_A_PIN1, CHS_TB_ENC_A_PORT2, CHS_TB_ENC_A_PIN2, CHS_TB_ENC_A_AF },
    { CHS_TB_ENC_B_PORT1, CHS_TB_ENC_B_PIN1, CHS_TB_ENC_B_PORT2, CHS_TB_ENC_B_PIN2, CHS_TB_ENC_B_AF },
    { CHS_TB_ENC_C_PORT1, CHS_TB_ENC_C_PIN1, CHS_TB_ENC_C_PORT2, CHS_TB_ENC_C_PIN2, CHS_TB_ENC_C_AF },
    { CHS_TB_ENC_D_PORT1, CHS_TB_ENC_D_PIN1, CHS_TB_ENC_D_PORT2, CHS_TB_ENC_D_PIN2, CHS_TB_ENC_D_AF }
};

/* ==================== 内部工具 ==================== */

/**
 * @brief  有符号 32 位限幅。
 */
static int32_t tb_clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/**
 * @brief  配置一路定时器为正交编码器接口（4 倍频）。
 *         EncoderMode=TIM_ENCODERMODE_TI12：TI1、TI2 的每个边沿都计数 ⇒ x4。
 *         IC1/IC2Filter=6：数字滤波 6 个采样，滤掉霍尔线上的毛刺；
 *         采样时钟为 fDTS/2（F4 的 CKD=DIV1 时 fDTS=定时器时钟），
 *         对 11 线/90 减速比这种机械脉冲频率（最高约 7kHz/相）完全够用，
 *         不会把真实边沿滤掉。
 */
static int tb_enc_init(uint8_t idx)
{
    TIM_HandleTypeDef    *h  = &s_enc[idx];
    TIM_Encoder_InitTypeDef ec = {0};

    h->Instance               = s_enc_inst[idx];
    h->Init.Prescaler         = 0U;                 /* 不分频：编码器计数不用降速 */
    h->Init.CounterMode       = TIM_COUNTERMODE_UP;
    h->Init.Period            = s_enc_period[idx];  /* 16 位 0xFFFF；TIM2 32 位 0xFFFFFFFF */
    h->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    h->Init.RepetitionCounter = 0U;                 /* TIM1/TIM8 高级定时器必需 */
    h->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    ec.EncoderMode  = TIM_ENCODERMODE_TI12;         /* 4 倍频：A/B 双边沿都算 */
    ec.IC1Polarity  = TIM_ICPOLARITY_RISING;
    ec.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    ec.IC1Prescaler = TIM_ICPSC_DIV1;
    ec.IC1Filter    = 6U;
    ec.IC2Polarity  = TIM_ICPOLARITY_RISING;
    ec.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    ec.IC2Prescaler = TIM_ICPSC_DIV1;
    ec.IC2Filter    = 6U;

    if (HAL_TIM_Encoder_Init(h, &ec) != HAL_OK) return -1;

    /* 必须用 TIM_CHANNEL_ALL：HAL 里 TIM_CHANNEL_1 的值是 0x0，
       写 CH1|CH2 实际等于 CHANNEL_2，只会开 CC2，TI1 不使能 ⇒ 编码器不计数。
       ALL 走 default 分支，CC1/CC2 一起开。 */
    if (HAL_TIM_Encoder_Start(h, TIM_CHANNEL_ALL) != HAL_OK) return -1;

    __HAL_TIM_SET_COUNTER(h, 0U);
    return 0;
}

/* ==================== 初始化 ==================== */

int CHS_TB_Init(void)
{
    GPIO_InitTypeDef gi = {0};
    TIM_OC_InitTypeDef oc = {0};
    uint8_t i;

    s_ready = 0U;

    /* ---- ① 时钟：GPIO A~E + TIM1/2/3/4/8 ---- */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();

    /* ---- ② 先把 STBY 拉低、DIR 全清零，保证上电不会乱转 ---- */
    HAL_GPIO_WritePin(CHS_TB_STBY_PORT, CHS_TB_STBY_PIN, GPIO_PIN_RESET);

    for (i = 0U; i < CHS_TB_NUM; i++)
    {
        HAL_GPIO_WritePin(s_dir_port[i], (uint16_t)(s_dir_pin1[i] | s_dir_pin2[i]), GPIO_PIN_RESET);
        s_duty[i]      = 0;
        s_enc_last[i]  = 0U;
        s_enc_total[i] = 0;
    }

    /* ---- ③ STBY 输出（低电平待机） ---- */
    gi.Pin       = CHS_TB_STBY_PIN;
    gi.Mode      = GPIO_MODE_OUTPUT_PP;
    gi.Pull      = GPIO_NOPULL;
    gi.Speed     = GPIO_SPEED_FREQ_LOW;
    gi.Alternate = 0U;
    HAL_GPIO_Init(CHS_TB_STBY_PORT, &gi);

    /* ---- ④ 板载 W25Q16 CS：先写高再配输出，FLASH 全程不选中 ----
       本板 FLASH 的 CLK/MISO/MOSI = PB3/PB4/PB5，与编码器 A/B 共用；
       CS(PB0) 一旦为低，FLASH 会驱动 MISO(PB4)，编码器 A 计数必乱。 */
    HAL_GPIO_WritePin(CHS_TB_FLASH_CS_PORT, CHS_TB_FLASH_CS_PIN, GPIO_PIN_SET);
    gi.Pin       = CHS_TB_FLASH_CS_PIN;
    gi.Mode      = GPIO_MODE_OUTPUT_PP;
    gi.Pull      = GPIO_NOPULL;
    gi.Speed     = GPIO_SPEED_FREQ_LOW;
    gi.Alternate = 0U;
    HAL_GPIO_Init(CHS_TB_FLASH_CS_PORT, &gi);

    /* ---- ⑤ 8 根 DIR 输出（推挽、低速即可；TB6612 输入是高阻 CMOS） ---- */
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    for (i = 0U; i < CHS_TB_NUM; i++)
    {
        gi.Pin = (uint16_t)(s_dir_pin1[i] | s_dir_pin2[i]);
        HAL_GPIO_Init(s_dir_port[i], &gi);
    }

    /* ---- ⑥ 四路 PWM 引脚：PD12~PD15，AF2，推挽，最高速 ---- */
    gi.Pin       = (uint16_t)(CHS_TB_PWM_PIN_A | CHS_TB_PWM_PIN_B |
                              CHS_TB_PWM_PIN_C | CHS_TB_PWM_PIN_D);
    gi.Mode      = GPIO_MODE_AF_PP;
    gi.Pull      = GPIO_NOPULL;
    gi.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gi.Alternate = CHS_TB_PWM_AF;
    HAL_GPIO_Init(CHS_TB_PWM_PORT, &gi);

    /* ---- ⑦ 四路编码器引脚：AF 输入，内部上拉（编码器多为集电极开路/推挽） ---- */
    for (i = 0U; i < CHS_TB_NUM; i++)
    {
        gi.Pin       = s_enc_pin[i].pin1;
        gi.Mode      = GPIO_MODE_AF_PP;
        gi.Pull      = GPIO_PULLUP;
        gi.Speed     = GPIO_SPEED_FREQ_HIGH;
        gi.Alternate = s_enc_pin[i].af;
        HAL_GPIO_Init(s_enc_pin[i].port1, &gi);

        gi.Pin = s_enc_pin[i].pin2;
        HAL_GPIO_Init(s_enc_pin[i].port2, &gi);
    }

    /* ---- ⑧ TIM4：20kHz PWM，四路 CCR 先清 0 ---- */
    s_pwm.Instance               = CHS_TB_PWM_TIM;
    s_pwm.Init.Prescaler         = 0U;
    s_pwm.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_pwm.Init.Period            = CHS_TB_PWM_ARR;
    s_pwm.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_pwm.Init.RepetitionCounter = 0U;
    s_pwm.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&s_pwm) != HAL_OK) return CHS_TB_ERR_TIM;

    oc.OCMode       = TIM_OCMODE_PWM1;
    oc.Pulse        = 0U;
    oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode   = TIM_OCFAST_DISABLE;
    oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    for (i = 0U; i < CHS_TB_NUM; i++)
    {
        if (HAL_TIM_PWM_ConfigChannel(&s_pwm, &oc, s_pwm_ch[i]) != HAL_OK) return CHS_TB_ERR_TIM;
        if (HAL_TIM_PWM_Start(&s_pwm, s_pwm_ch[i]) != HAL_OK) return CHS_TB_ERR_TIM;
    }

    /* ---- ⑨ 四路编码器 ---- */
    for (i = 0U; i < CHS_TB_NUM; i++)
    {
        if (tb_enc_init(i) != 0) return CHS_TB_ERR_TIM;
    }

    s_ready = 1U;
    return CHS_TB_OK;
}

/* ==================== 输出 ==================== */

/**
 * @brief  只改方向脚，不碰 PWM。
 * @param  dir +1=正转(IN1=1,IN2=0)；-1=反转(IN1=0,IN2=1)；0=滑行(IN1=IN2=0)
 */
static void tb_dir_set(uint8_t wheel, int8_t dir)
{
    if (dir > 0)
    {
        HAL_GPIO_WritePin(s_dir_port[wheel], s_dir_pin1[wheel], GPIO_PIN_SET);
        HAL_GPIO_WritePin(s_dir_port[wheel], s_dir_pin2[wheel], GPIO_PIN_RESET);
    }
    else if (dir < 0)
    {
        HAL_GPIO_WritePin(s_dir_port[wheel], s_dir_pin1[wheel], GPIO_PIN_RESET);
        HAL_GPIO_WritePin(s_dir_port[wheel], s_dir_pin2[wheel], GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(s_dir_port[wheel], (uint16_t)(s_dir_pin1[wheel] | s_dir_pin2[wheel]), GPIO_PIN_RESET);
    }
}

void CHS_TB_SetDuty(uint8_t wheel, int16_t permille)
{
    int32_t d;
    uint32_t ccr;
    int16_t old;
    int8_t  new_dir;

    if ((wheel >= CHS_TB_NUM) || (s_ready == 0U)) return;

    d = tb_clamp_i32((int32_t)permille, -1000, 1000);
    ccr = ((uint32_t)((d >= 0) ? d : -d) * (CHS_TB_PWM_ARR + 1U)) / 1000U;

    old     = s_duty[wheel];
    new_dir = (int8_t)((d > 0) ? 1 : ((d < 0) ? -1 : 0));

    /* 换向那一拍必须先把 CCR 清 0，再改 DIR，最后给新占空比。
       否则会出现「新占空比 + 旧方向」的几微秒直通脉冲。 */
    if ((old > 0 && d < 0) || (old < 0 && d > 0))
    {
        __HAL_TIM_SET_COMPARE(&s_pwm, s_pwm_ch[wheel], 0U);
        tb_dir_set(wheel, new_dir);
        __HAL_TIM_SET_COMPARE(&s_pwm, s_pwm_ch[wheel], ccr);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&s_pwm, s_pwm_ch[wheel], ccr);
        tb_dir_set(wheel, new_dir);
    }

    s_duty[wheel] = (int16_t)d;
}

void CHS_TB_Coast(uint8_t wheel)
{
    if ((wheel >= CHS_TB_NUM) || (s_ready == 0U)) return;

    __HAL_TIM_SET_COMPARE(&s_pwm, s_pwm_ch[wheel], 0U);
    HAL_GPIO_WritePin(s_dir_port[wheel], (uint16_t)(s_dir_pin1[wheel] | s_dir_pin2[wheel]), GPIO_PIN_RESET);
    s_duty[wheel] = 0;
}

void CHS_TB_SetStandby(uint8_t enable)
{
    uint8_t i;

    if (enable == 0U)
    {
        /* 待机前先把四路 PWM/DIR 清干净，避免恢复时从旧占空比猛冲 */
        for (i = 0U; i < CHS_TB_NUM; i++)
        {
            if (s_ready != 0U)
            {
                __HAL_TIM_SET_COMPARE(&s_pwm, s_pwm_ch[i], 0U);
            }
            HAL_GPIO_WritePin(s_dir_port[i], (uint16_t)(s_dir_pin1[i] | s_dir_pin2[i]), GPIO_PIN_RESET);
            s_duty[i] = 0;
        }
    }

    HAL_GPIO_WritePin(CHS_TB_STBY_PORT, CHS_TB_STBY_PIN, (enable != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ==================== 编码器 ==================== */

int32_t CHS_TB_ReadDelta(uint8_t wheel)
{
    uint32_t now;
    int32_t  delta;

    if ((wheel >= CHS_TB_NUM) || (s_ready == 0U)) return 0;

    now = __HAL_TIM_GET_COUNTER(&s_enc[wheel]);

    if (wheel == CHS_TB_B)
    {
        /* TIM2 是 32 位，直接按 32 位回绕相减 */
        delta = (int32_t)(now - s_enc_last[wheel]);
    }
    else
    {
        /* 16 位定时器：先按 uint16 相减（自动处理 65535→0 回绕），再解释成 int16 */
        delta = (int32_t)(int16_t)((uint16_t)now - (uint16_t)s_enc_last[wheel]);
    }

    s_enc_last[wheel]  = now;
    s_enc_total[wheel] += delta;

    return delta;
}

int32_t CHS_TB_GetTotal(uint8_t wheel)
{
    if (wheel >= CHS_TB_NUM) return 0;
    return s_enc_total[wheel];
}

void CHS_TB_ResetEncoder(void)
{
    uint8_t i;

    for (i = 0U; i < CHS_TB_NUM; i++)
    {
        if (s_ready != 0U)
        {
            __HAL_TIM_SET_COUNTER(&s_enc[i], 0U);
        }
        s_enc_last[i]  = 0U;
        s_enc_total[i] = 0;
    }
}
