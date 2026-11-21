/**
 * chassis.c — 四路编码器 TT 马达（1:90）+ 80mm 麦轮 底盘驱动实现
 * ----------------------------------------------------------------------------
 * 驱动板：松甲/Yeahbot 四路编码器电机驱动模块（I2C 从机 0x51，AT8236×4）
 * 协议依据：官方手册 v1.2 §2.1③ 寄存器表 + 例程 motor_4ch.c
 *
 * 关键时序（手册例程就是这么写的，不能省）：
 *   写：START → 0xA2 → 寄存器 → 数据… → STOP
 *   读：START → 0xA2 → 寄存器 → STOP → 等 2ms → START → 0xA3 → 读 N 字节（末字节 NACK）→ STOP
 *
 * 设计要点：
 *   1) 速度闭环交给驱动板（0x06，单位 0.01m/s），STM32 只做麦轮逆解 + 里程计；
 *   2) 每拍读 0x01（4×int32 累计脉冲），自己算轮速和里程，不依赖板载 20ms 窗口；
 *   3) 梯形限加速度，避免 4 个 TT 同时起跳造成电流冲击和麦轮打滑；
 *   4) 500ms 指令超时自动停车，防上位机掉线后车一直跑；
 *   5) I2C1 与 PCA9685 共用，所有传输都套 i2c_lock 互斥（见 freertos.c）。
 * ============================================================================
 */
#include "chassis.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"

#include <stdio.h>

/* ==================== 驱动板寄存器地址（手册 v1.2 §2.1③） ==================== */
#define REG_ADC_BATTERY     0x00U   /* 读 2B：电池 ADC（大端） */
#define REG_ENC_TOTAL       0x01U   /* 读 16B：4×int32 各自累计脉冲（大端） */
#define REG_ENC_20MS        0x02U   /* 读 8B ：4×int16 各自 20ms 脉冲（本工程不用） */
#define REG_MOTOR_TYPE      0x03U   /* 写 1B ：0=520 / 1=TT / 2=310 */
#define REG_ENC_POLARITY    0x04U   /* 写 1B ：0=默认 / 1=编码器 AB 取反（全局） */
#define REG_MOTOR_PWM       0x05U   /* 写 4B ：int8 -100~100 开环 PWM */
#define REG_MOTOR_SPEED     0x06U   /* 写 4B ：int8 -100~100 闭环速度，单位 0.01m/s */
#define REG_ENC_CLEAR       0x07U   /* 写 0B ：清 4 路累计脉冲 */
#define REG_MOTOR_PARAM     0x08U   /* 写 3B ：轮径(cm) / 编码器线数 / 减速比 */

/* 7 位地址 → 8 位读写地址 */
#define CHS_ADDR_W          ((uint16_t)(CHS_I2C_ADDR7 << 1))
#define CHS_ADDR_R          ((uint16_t)((CHS_I2C_ADDR7 << 1) | 0x01U))

/* 自带三角函数常量（MicroLIB 不带 sinf/cosf，见下方说明） */
#define CHS_PI              3.14159265f
#define CHS_TWO_PI          6.28318531f
#define CHS_HALF_PI         1.57079633f

/* I2C1 总线互斥：由 freertos.c 创建，Arm_Update(PCA9685) 与本模块共用。
   声明成 extern 而不是塞进头文件，避免 chassis.h 反向依赖 cmsis_os。 */
extern osMutexId_t i2c_lock;

/* 全局唯一实例 */
CHS_Handle g_chassis;

/* ==================== 内部小工具 ==================== */

/**
 * @brief  浮点限幅。
 */
static float chs_clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/**
 * @brief  m/s → 驱动板 int8（单位 0.01m/s，四舍五入，限 ±100）。
 *         不用 roundf：MicroLIB 不提供 math.h 的浮点函数。
 */
static int8_t chs_speed_to_i8(float v_ms)
{
    float v = v_ms * 100.0f;

    if (v >= 0.0f) v += 0.5f;
    else           v -= 0.5f;

    if (v >  100.0f) v =  100.0f;
    if (v < -100.0f) v = -100.0f;

    return (int8_t)v;
}

/**
 * @brief  工程开了 MicroLIB（.uvprojx 里 useUlib=1），它不提供 sinf/cosf。
 *         这里用 7 阶泰勒展开自己算，先折叠到 [-π/2, π/2]。
 *         最大误差 ≈1.6e-4 rad（半径 1m 时约 0.16mm，里程计足够）。
 */
static float chs_sin(float x)
{
    float x2, r;

    /* 归一化到 [-π, π]；里程计航向已经被卷绕，这里最多循环一次 */
    while (x >  CHS_PI) x -= CHS_TWO_PI;
    while (x < -CHS_PI) x += CHS_TWO_PI;

    /* 折叠到 [-π/2, π/2]：sin(π - x) = sin(x) */
    if (x >  CHS_HALF_PI) x =  CHS_PI - x;
    if (x < -CHS_HALF_PI) x = -CHS_PI - x;

    /* sin(x) ≈ x - x³/6 + x⁵/120 - x⁷/5040 */
    x2 = x * x;
    r  = 1.0f - x2 * ((1.0f / 6.0f) - x2 * ((1.0f / 120.0f) - x2 * (1.0f / 5040.0f)));

    return x * r;
}

/**
 * @brief  cos(x) = sin(x + π/2)。
 */
static float chs_cos(float x)
{
    return chs_sin(x + CHS_HALF_PI);
}

/* ==================== I2C 传输（带总线互斥） ==================== */

/**
 * @brief  总线加锁。i2c_lock 还没创建时（调度器启动前）直接跳过。
 */
static void chs_bus_lock(void)
{
    if (i2c_lock != NULL)
    {
        (void)osMutexAcquire(i2c_lock, osWaitForever);
    }
}

/**
 * @brief  总线解锁（与 chs_bus_lock 成对）。
 */
static void chs_bus_unlock(void)
{
    if (i2c_lock != NULL)
    {
        (void)osMutexRelease(i2c_lock);
    }
}

/**
 * @brief  写寄存器：START → 0xA2 → reg → data… → STOP。
 * @param  len 数据字节数（0 表示只发寄存器地址，用于 0x07 清零）
 */
static HAL_StatusTypeDef chs_i2c_write(uint8_t reg, const uint8_t *data, uint8_t len)
{
    uint8_t buf[9];             /* 1 字节寄存器 + 最多 8 字节数据 */
    uint8_t n = 0U;
    uint8_t i;
    uint8_t retry;
    HAL_StatusTypeDef st = HAL_ERROR;

    if (len > 8U) return HAL_ERROR;

    buf[n++] = reg;
    for (i = 0U; i < len; i++)
    {
        buf[n++] = data[i];
    }

    chs_bus_lock();

    for (retry = 0U; retry < CHS_I2C_RETRY; retry++)
    {
        st = HAL_I2C_Master_Transmit(&CHS_I2C_HANDLE, CHS_ADDR_W, buf, n, CHS_I2C_TIMEOUT_MS);
        if (st == HAL_OK) break;
        if (st != HAL_BUSY) break;      /* NACK/总线错重试没用，直接返回 */
        osDelay(1);
    }

    chs_bus_unlock();
    return st;
}

/**
 * @brief  读寄存器：写寄存器地址 → 等 2ms → 读 len 字节。
 *         手册例程用两次独立 START（中间有 STOP），这里严格照做，
 *         不能用 HAL_I2C_Mem_Read（那是重复起始且没有 2ms 间隔）。
 */
static HAL_StatusTypeDef chs_i2c_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t retry;
    HAL_StatusTypeDef st = HAL_ERROR;

    chs_bus_lock();

    /* ① 发寄存器地址 */
    for (retry = 0U; retry < CHS_I2C_RETRY; retry++)
    {
        st = HAL_I2C_Master_Transmit(&CHS_I2C_HANDLE, CHS_ADDR_W, &reg, 1U, CHS_I2C_TIMEOUT_MS);
        if (st != HAL_BUSY) break;
        osDelay(1);
    }
    if (st != HAL_OK)
    {
        chs_bus_unlock();
        return st;
    }

    /* ② 给板载 MCU 准备数据的时间（手册：≥2ms），顺便让出 CPU */
    osDelay(CHS_READ_GAP_MS);

    /* ③ 读数据 */
    for (retry = 0U; retry < CHS_I2C_RETRY; retry++)
    {
        st = HAL_I2C_Master_Receive(&CHS_I2C_HANDLE, CHS_ADDR_R, buf, len, CHS_I2C_TIMEOUT_MS);
        if (st != HAL_BUSY) break;
        osDelay(1);
    }

    chs_bus_unlock();
    return st;
}

/* ==================== 编码器与里程计 ==================== */

/**
 * @brief  读 4 路累计脉冲（16 字节大端 int32），算增量、轮速、累计值。
 * @retval CHS_OK / CHS_ERR_OFFLINE
 */
static int chs_read_encoders(CHS_Handle *h)
{
    uint8_t buf[16];
    uint8_t i;
    int32_t cur;
    int32_t delta;

    if (chs_i2c_read(REG_ENC_TOTAL, buf, 16U) != HAL_OK)
    {
        h->online = 0U;
        for (i = 0U; i < CHS_W_NUM; i++)
        {
            h->w[i].speed_ms = 0.0f;    /* 读不到就不敢积分，避免里程计乱飘 */
        }
        return CHS_ERR_OFFLINE;
    }

    h->online = 1U;

    for (i = 0U; i < CHS_W_NUM; i++)
    {
        cur = (int32_t)(((uint32_t)buf[4U * i]           << 24) |
                        ((uint32_t)buf[4U * i + 1U]      << 16) |
                        ((uint32_t)buf[4U * i + 2U]      <<  8) |
                         (uint32_t)buf[4U * i + 3U]);

        /* 用无符号相减再转回有符号：编码器回绕/溢出也不会算出天文数字 */
        delta = (int32_t)((uint32_t)cur - (uint32_t)h->w[i].enc_last);

        h->w[i].enc_last   = cur;
        h->w[i].enc_total += delta;

        /* mm/计数 × 计数 ÷ 20ms = m/s（1mm/ms 恰好等于 1m/s） */
        h->w[i].speed_ms = (float)delta * CHS_MM_PER_COUNT / (float)CHS_PERIOD_MS;
    }

    return CHS_OK;
}

/**
 * @brief  麦轮正运动学：四轮实测轮速 → 车体 vx/vy/ω → 积分成世界系位姿。
 */
static void chs_update_odometry(CHS_Handle *h)
{
    const float k  = (CHS_HALF_WHEELBASE_MM + CHS_HALF_TRACK_MM) * 0.001f;  /* m */
    const float dt = (float)CHS_PERIOD_MS * 0.001f;                          /* s */

    float a = h->w[CHS_W_A].speed_ms;
    float b = h->w[CHS_W_B].speed_ms;
    float c = h->w[CHS_W_C].speed_ms;
    float d = h->w[CHS_W_D].speed_ms;

    float vx = ( a + b + c + d) * 0.25f;
    float vy = (-a + b + c - d) * 0.25f;
    float wz = (-a + b - c + d) / (4.0f * k);

    float yaw = h->odom_yaw;
    float dx_m = ( vx * chs_cos(yaw) - vy * chs_sin(yaw)) * dt;
    float dy_m = ( vx * chs_sin(yaw) + vy * chs_cos(yaw)) * dt;

    h->odom_x_mm += dx_m * 1000.0f;
    h->odom_y_mm += dy_m * 1000.0f;
    h->odom_yaw  += wz * dt;

    /* 航向卷绕到 [-π, π]，防止长时间跑圈后精度变差 */
    while (h->odom_yaw >  CHS_PI) h->odom_yaw -= CHS_TWO_PI;
    while (h->odom_yaw < -CHS_PI) h->odom_yaw += CHS_TWO_PI;
}

/* ==================== 对外 API ==================== */

/**
 * @brief  探活：读 0x00 电池 ADC（2 字节大端）。
 */
int CHS_Probe(CHS_Handle *h)
{
    uint8_t buf[2];

    if (h == NULL) return CHS_ERR_PARAM;

    if (chs_i2c_read(REG_ADC_BATTERY, buf, 2U) != HAL_OK)
    {
        h->online = 0U;
        return CHS_ERR_OFFLINE;
    }

    h->batt_adc = (uint16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
    h->online   = 1U;

    return CHS_OK;
}

/**
 * @brief  上电初始化。
 */
int CHS_Init(CHS_Handle *h)
{
    uint8_t v;
    uint8_t p[3];
    uint8_t i;

    if (h == NULL) return CHS_ERR_PARAM;

    /* ---- ① 状态清零 ---- */
    for (i = 0U; i < CHS_W_NUM; i++)
    {
        h->w[i].enc_last   = 0;
        h->w[i].enc_total  = 0;
        h->w[i].speed_ms   = 0.0f;
        h->w[i].target_ms  = 0.0f;
        h->w[i].cmd_ms     = 0.0f;
    }
    h->mode          = CHS_MODE_IDLE;
    h->online        = 0U;
    h->param_ok      = 0U;
    h->batt_div      = 0U;
    h->batt_adc      = 0U;
    h->last_cmd_tick = 0U;
    h->stop_at_tick  = 0U;
    CHS_ResetOdometry(h);

    /* ---- ② 电机类型 = TT（手册 0x03；例程默认写 0 是给 520 电机用的，别照抄） ---- */
    v = CHS_MOTOR_TYPE;
    if (chs_i2c_write(REG_MOTOR_TYPE, &v, 1U) != HAL_OK)
    {
        h->online = 0U;
        return CHS_ERR_I2C;
    }
    osDelay(20U);       /* 例程 Motor_Init 的 20ms 等待：给板载 MCU 落配置的时间 */

    /* ---- ③ 编码器极性 = 0（正）。某轮方向反了先查电机线，不要盲改这一位 ---- */
    v = 0U;
    if (chs_i2c_write(REG_ENC_POLARITY, &v, 1U) != HAL_OK)
    {
        h->online = 0U;
        return CHS_ERR_I2C;
    }
    osDelay(20U);

    /* ---- ④ 运动学参数：轮径 8cm / 13 线 / 1:90。
             写失败不致命：只影响板载闭环的测速换算，轮子照样能转 ---- */
    p[0] = (uint8_t)CHS_WHEEL_D_CM;
    p[1] = (uint8_t)CHS_ENC_LINES;
    p[2] = (uint8_t)CHS_GEAR_RATIO;

    if (chs_i2c_write(REG_MOTOR_PARAM, p, 3U) == HAL_OK)
    {
        h->param_ok = 1U;
    }
    osDelay(20U);

    /* ---- ⑤ 清四路累计脉冲 ---- */
    if (chs_i2c_write(REG_ENC_CLEAR, NULL, 0U) != HAL_OK)
    {
        h->online = 0U;
        return CHS_ERR_I2C;
    }

    /* ---- ⑥ 探活（顺带读一次电池） ---- */
    if (CHS_Probe(h) != CHS_OK)
    {
        return CHS_ERR_OFFLINE;
    }

    /* ---- ⑦ 上电先停车，防止板子上电默认状态乱转 ---- */
    (void)CHS_Stop(h);

    return CHS_OK;
}

/**
 * @brief  麦轮逆运动学 → 四轮目标速度（只存目标，真正下发在 CHS_Tick 里）。
 *         A=左前 B=右前 C=左后 D=右后；vx 前、vy 右、wz 逆时针为正。
 */
int CHS_SetVelocity(CHS_Handle *h, float vx_ms, float vy_ms, float wz_rads)
{
    const float k = (CHS_HALF_WHEELBASE_MM + CHS_HALF_TRACK_MM) * 0.001f;   /* m */
    float t[CHS_W_NUM];
    uint8_t i;

    if (h == NULL) return CHS_ERR_PARAM;

    /* 标准麦轮逆解（R=40mm 已折算进线速度，不再除轮径） */
    t[CHS_W_A] = vx_ms - vy_ms - k * wz_rads;
    t[CHS_W_B] = vx_ms + vy_ms + k * wz_rads;
    t[CHS_W_C] = vx_ms + vy_ms - k * wz_rads;
    t[CHS_W_D] = vx_ms - vy_ms + k * wz_rads;

    for (i = 0U; i < CHS_W_NUM; i++)
    {
        h->w[i].target_ms = chs_clampf(t[i], -CHS_MAX_WHEEL_MS, CHS_MAX_WHEEL_MS);
    }

    h->mode          = CHS_MODE_SPEED;
    h->last_cmd_tick = HAL_GetTick();
    h->stop_at_tick  = 0U;      /* 新指令作废上一次的定时停止 */

    return CHS_OK;
}

/**
 * @brief  定时版逆解：dur_ms>0 时运行 dur_ms 后自动刹停。
 *         期间 500ms 看门狗暂停，由 stop_at_tick 兜底。
 */
int CHS_SetVelocityMs(CHS_Handle *h, float vx_ms, float vy_ms, float wz_rads, uint32_t dur_ms)
{
    int r = CHS_SetVelocity(h, vx_ms, vy_ms, wz_rads);

    if (r == CHS_OK && dur_ms > 0U)
    {
        h->stop_at_tick = HAL_GetTick() + dur_ms;
    }

    return r;
}

/**
 * @brief  跳过逆解，直接给四轮目标线速度。
 */
int CHS_SetWheelSpeed(CHS_Handle *h, float a_ms, float b_ms, float c_ms, float d_ms)
{
    if (h == NULL) return CHS_ERR_PARAM;

    h->w[CHS_W_A].target_ms = chs_clampf(a_ms, -CHS_MAX_WHEEL_MS, CHS_MAX_WHEEL_MS);
    h->w[CHS_W_B].target_ms = chs_clampf(b_ms, -CHS_MAX_WHEEL_MS, CHS_MAX_WHEEL_MS);
    h->w[CHS_W_C].target_ms = chs_clampf(c_ms, -CHS_MAX_WHEEL_MS, CHS_MAX_WHEEL_MS);
    h->w[CHS_W_D].target_ms = chs_clampf(d_ms, -CHS_MAX_WHEEL_MS, CHS_MAX_WHEEL_MS);

    h->mode          = CHS_MODE_SPEED;
    h->last_cmd_tick = HAL_GetTick();
    h->stop_at_tick  = 0U;

    return CHS_OK;
}

/**
 * @brief  开环 PWM 直驱（0x05，-100~100），台架验证接线/转向用。
 */
int CHS_SetPwmRaw(CHS_Handle *h, int8_t a, int8_t b, int8_t c, int8_t d)
{
    uint8_t buf[4];

    if (h == NULL) return CHS_ERR_PARAM;

    /* 驱动板按 int8 解释，负值直接发补码 */
    buf[0] = (uint8_t)a;
    buf[1] = (uint8_t)b;
    buf[2] = (uint8_t)c;
    buf[3] = (uint8_t)d;

    if (chs_i2c_write(REG_MOTOR_PWM, buf, 4U) != HAL_OK)
    {
        return CHS_ERR_I2C;
    }

    /* 切到开环后把目标清掉，免得下次切回闭环时从旧目标猛冲 */
    h->w[CHS_W_A].target_ms = 0.0f; h->w[CHS_W_A].cmd_ms = 0.0f;
    h->w[CHS_W_B].target_ms = 0.0f; h->w[CHS_W_B].cmd_ms = 0.0f;
    h->w[CHS_W_C].target_ms = 0.0f; h->w[CHS_W_C].cmd_ms = 0.0f;
    h->w[CHS_W_D].target_ms = 0.0f; h->w[CHS_W_D].cmd_ms = 0.0f;

    h->mode          = CHS_MODE_PWM;
    h->stop_at_tick  = 0U;

    return CHS_OK;
}

/**
 * @brief  立即停车：0x06 与 0x05 都写 0。
 */
int CHS_Stop(CHS_Handle *h)
{
    uint8_t zero4[4] = { 0U, 0U, 0U, 0U };
    uint8_t i;
    int ret = CHS_OK;

    if (h == NULL) return CHS_ERR_PARAM;

    for (i = 0U; i < CHS_W_NUM; i++)
    {
        h->w[i].target_ms = 0.0f;
        h->w[i].cmd_ms    = 0.0f;
    }
    h->mode          = CHS_MODE_IDLE;
    h->stop_at_tick  = 0U;

    if (chs_i2c_write(REG_MOTOR_SPEED, zero4, 4U) != HAL_OK)
    {
        ret = CHS_ERR_I2C;
    }
    if (chs_i2c_write(REG_MOTOR_PWM, zero4, 4U) != HAL_OK)
    {
        ret = CHS_ERR_I2C;
    }

    return ret;
}

/**
 * @brief  20ms 周期心跳。
 */
void CHS_Tick(CHS_Handle *h)
{
    uint8_t i;
    uint8_t out[4];
    float step;

    if (h == NULL) return;

    /* ---- ① 读编码器并更新轮速 / 里程计 ---- */
    (void)chs_read_encoders(h);
    chs_update_odometry(h);

    /* ---- ② 电池 ADC 分频读取（0.5s 一次） ---- */
    h->batt_div++;
    if (h->batt_div >= CHS_BATT_DIV)
    {
        h->batt_div = 0U;
        (void)CHS_Probe(h);
    }

    /* ---- ③ 定时停止：VEL,..,ms 的 ms 到期立即刹停（SPEED/PWM 都适用） ---- */
    if (h->stop_at_tick != 0U)
    {
        if ((int32_t)(HAL_GetTick() - h->stop_at_tick) >= 0)
        {
            (void)CHS_Stop(h);      /* 内部清 stop_at_tick 并回 IDLE */
            return;
        }
    }

    /* ---- ④ 只有闭环速度模式才碰速度寄存器 ---- */
    if (h->mode != CHS_MODE_SPEED) return;

    /* ---- ⑤ 指令超时保护：无定时移动且 500ms 没新指令 → 目标清零（防掉线） ---- */
    if (h->stop_at_tick == 0U &&
        (uint32_t)(HAL_GetTick() - h->last_cmd_tick) > CHS_CMD_TIMEOUT_MS)
    {
        for (i = 0U; i < CHS_W_NUM; i++)
        {
            h->w[i].target_ms = 0.0f;
        }
    }

    /* ---- ⑥ 梯形限加速度：每拍最多变化 step m/s ---- */
    step = CHS_ACCEL_MS2 * (float)CHS_PERIOD_MS * 0.001f;

    for (i = 0U; i < CHS_W_NUM; i++)
    {
        float t = h->w[i].target_ms;
        float c = h->w[i].cmd_ms;

        if (step <= 0.0f)
        {
            c = t;                          /* 不限加速，直接跟随 */
        }
        else if (t > c)
        {
            c += step;
            if (c > t) c = t;
        }
        else if (t < c)
        {
            c -= step;
            if (c < t) c = t;
        }
        else
        {
            /* 已到位，保持不变 */
        }

        h->w[i].cmd_ms = c;
        out[i] = (uint8_t)chs_speed_to_i8(c);
    }

    /* ---- ⑦ 下发四轮目标速度（0x06，int8，0.01m/s） ---- */
    (void)chs_i2c_write(REG_MOTOR_SPEED, out, 4U);
}

/**
 * @brief  读里程计（任一指针可为 NULL）。
 */
void CHS_GetOdometry(const CHS_Handle *h, float *x_mm, float *y_mm, float *yaw_deg)
{
    if (h == NULL) return;

    if (x_mm    != NULL) *x_mm    = h->odom_x_mm;
    if (y_mm    != NULL) *y_mm    = h->odom_y_mm;
    if (yaw_deg != NULL) *yaw_deg = h->odom_yaw * (180.0f / CHS_PI);
}

/**
 * @brief  里程计清零（不动驱动板累计脉冲）。
 */
void CHS_ResetOdometry(CHS_Handle *h)
{
    if (h == NULL) return;

    h->odom_x_mm = 0.0f;
    h->odom_y_mm = 0.0f;
    h->odom_yaw  = 0.0f;
}

/* ==================== 台架自检（可选） ==================== */
#if CHS_SELFTEST_EN

#define CHS_ST_PWM      20          /* 自检占空比 20% */
#define CHS_ST_MS       800U        /* 每个方向转 800ms */

/* 串口打印锁：与 freertos.c 的 PrintTask(3s 遥测) 共用，防止两行混在一起 */
extern osMutexId_t print_lock;

/**
 * @brief  自检专用的带锁打印：先格式化进 buf，再一次性上锁输出。
 *         避免与 PrintTask 的 STATE 遥测在 USART1 上逐字节交错成乱行。
 */
static void chs_st_log(const char *msg)
{
    osMutexAcquire(print_lock, osWaitForever);
    printf("%s", msg);
    osMutexRelease(print_lock);
}

/**
 * @brief  逐轮正反转自检，核对编码器计数方向。
 *         ⚠️ 轮子必须悬空！结果从 USART1(115200) 打印。
 */
int CHS_SelfTest(CHS_Handle *h)
{
    static const char *nm[CHS_W_NUM] = { "A(LF)", "B(RF)", "C(LR)", "D(RR)" };
    char    buf[96];
    uint8_t i;
    int     fail = 0;

    if (h == NULL) return CHS_ERR_PARAM;

    chs_st_log("\r\n[CHS] self-test start: LIFT THE WHEELS OFF THE GROUND!\r\n");

    if (CHS_Probe(h) != CHS_OK)
    {
        chs_st_log("[CHS] probe FAIL: check SCL/SDA/GND + pull-up, board VM power\r\n");
        return CHS_ERR_OFFLINE;
    }

    (void)snprintf(buf, sizeof(buf), "[CHS] battery ADC = %u\r\n", (unsigned)h->batt_adc);
    chs_st_log(buf);

    for (i = 0U; i < CHS_W_NUM; i++)
    {
        int8_t  p[4] = { 0, 0, 0, 0 };
        int32_t base, fwd, rev;

        (void)CHS_Stop(h);

        /* ---- 正转（+20% PWM）：期望该轮累计计数往正走 ---- */
        p[i] = (int8_t)CHS_ST_PWM;
        (void)CHS_SetPwmRaw(h, p[0], p[1], p[2], p[3]);
        (void)chs_read_encoders(h);
        base = h->w[i].enc_last;
        osDelay(CHS_ST_MS);
        (void)chs_read_encoders(h);
        fwd = h->w[i].enc_last - base;

        /* ---- 反转（-20% PWM）：期望该轮累计计数往负走 ---- */
        p[i] = (int8_t)(-CHS_ST_PWM);
        (void)CHS_SetPwmRaw(h, p[0], p[1], p[2], p[3]);
        base = h->w[i].enc_last;
        osDelay(CHS_ST_MS);
        (void)chs_read_encoders(h);
        rev = h->w[i].enc_last - base;

        (void)CHS_Stop(h);

        (void)snprintf(buf, sizeof(buf), "[CHS] %s fwd=%ld rev=%ld -> %s\r\n",
                       nm[i], (long)fwd, (long)rev,
                       (fwd > 20L && rev < -20L) ? "OK" : "CHECK WIRING/POLARITY");
        chs_st_log(buf);

        if (!(fwd > 20L && rev < -20L))
        {
            fail++;
        }
    }

    (void)snprintf(buf, sizeof(buf), "[CHS] self-test done, %d/%d passed\r\n",
                   (int)(CHS_W_NUM - fail), (int)CHS_W_NUM);
    chs_st_log(buf);

    return (fail == 0) ? CHS_OK : CHS_ERR_PARAM;
}

#endif /* CHS_SELFTEST_EN */
