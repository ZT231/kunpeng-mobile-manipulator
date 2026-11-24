/**
 * chassis.c — 四轮麦克纳姆底盘控制层
 * ----------------------------------------------------------------------------
 * 20ms 标称节拍（实际 dt 用 HAL tick 测量）固定做六件事，顺序不能乱：
 *   ① 读四路编码器（硬件定时器已 4 倍频）→ 算轮速、里程计；
 *   ② 安全闸门：急停/故障/定时停止/掉线超时；
 *   ③ 清故障后的 STBY 重新使能（只在 ChassisTask 写硬件）；
 *   ④ 梯形限加速：目标 m/s 每拍最多变 CHS_ACCEL_MS2×dt；
 *   ⑤ 四轮独立 PID：前馈 + PI，输出带符号占空比；
 *   ⑥ 堵转判定：给油 300ms 还不走 → 锁故障、拉低 STBY。
 *
 * 控制上的三个关键取舍：
 *   1) 逆解超限时【等比缩放】而不是逐轮裁剪。麦轮四轮是一个约束整体，
 *      砍某一个轮子会改变合速度方向，车会跑歪；等比缩放只减速、不改方向。
 *   2) PID 用「测量值微分」而不是「误差微分」。目标阶跃时误差微分会产生
 *      一个巨大的尖峰（微分冲击），麦轮底盘会猛点头。
 *   3) 停车分两种：$CAR,STOP 走 coast（请求式，下一拍 ≤20ms 断输出，快）；掉线超时走
 *      限加速减速（可控，不会让 4 个轮子瞬间抱死打滑）。
 * ============================================================================
 */
#include "chassis.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"

#include <stdio.h>

/* 全局唯一实例 */
CHS_Handle g_chassis;

/* USART1 打印互斥锁，定义在 freertos.c；本文件只读引用 */
extern osMutexId_t print_lock;

/* 三角函数常量。本工程开 MicroLIB，不提供 sinf/cosf，里程计自己算多项式。 */
#define CHS_PI              3.14159265f
#define CHS_TWO_PI          6.28318531f
#define CHS_HALF_PI         1.57079633f
#define CHS_RAD_TO_DEG      57.2957795f

/* ==================== 内部小工具 ==================== */

/**
 * @brief  浮点绝对值（不用 fabsf：MicroLIB 的浮点库不保证带这个符号）。
 */
static float chs_absf(float v)
{
    return (v < 0.0f) ? -v : v;
}

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
 * @brief  sin(x) 的 7 阶泰勒展开，先折叠到 [-π/2, π/2]。
 *         最大误差约 1.6e-4 rad（半径 1m 时约 0.16mm），里程计足够。
 */
static float chs_sin(float x)
{
    float x2, r;

    while (x >  CHS_PI) x -= CHS_TWO_PI;
    while (x < -CHS_PI) x += CHS_TWO_PI;

    if (x >  CHS_HALF_PI) x =  CHS_PI - x;
    if (x < -CHS_HALF_PI) x = -CHS_PI - x;

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

/**
 * @brief  某轮的目标是否与上一拍反号（用于反向时清积分，防「反向要先泄掉
 *         正向积分」导致迟滞）。
 */
static uint8_t chs_sign_flip(float old_v, float new_v)
{
    return (uint8_t)(((old_v > 0.0f) && (new_v < 0.0f)) ||
                     ((old_v < 0.0f) && (new_v > 0.0f)));
}

/**
 * @brief  清单轮 PID/堵转状态。
 */
static void chs_wheel_reset(CHS_WheelState *w)
{
    w->speed_ms   = 0.0f;
    w->target_ms  = 0.0f;
    w->cmd_ms     = 0.0f;
    w->integ      = 0.0f;
    w->prev_speed = 0.0f;
    w->duty       = 0;
    w->stall_ms   = 0U;
}

/* ==================== 安全闸门 ==================== */

/**
 * @brief  当前是否允许产生运动（初始化完成、无锁存故障）。
 *         底盘堵转锁存故障是最高优先级，故障时禁止一切运动。
 */
static uint8_t chs_can_move(const CHS_Handle *h)
{
    if ((h == NULL) || (h->initialized == 0U) || (h->fault != CHS_FAULT_NONE))
    {
        return 0U;
    }
    return 1U;
}

/**
 * @brief  锁存堵转故障：拉低 STBY（H 桥全关），清所有目标，打印一次。
 *         只能从 ChassisTask 调用（会直接写 STBY）。
 */
static void chs_latch_stall(CHS_Handle *h, uint8_t wheel)
{
    char    buf[96];
    uint8_t i;
    float   cmd_before = h->w[wheel].cmd_ms;   /* 清零前先留证，打印才有意义 */
    float   spd_before = h->w[wheel].speed_ms;

    h->fault    |= CHS_FAULT_STALL;
    h->online    = 0U;
    h->mode      = CHS_MODE_IDLE;
    h->stop_req  = 0U;

    for (i = 0U; i < CHS_W_NUM; i++)
    {
        h->w[i].target_ms = 0.0f;
        h->w[i].cmd_ms    = 0.0f;
        h->w[i].integ     = 0.0f;
        h->w[i].duty      = 0;
        h->w[i].stall_ms = 0U;
    }

    /* 先断输出再打印：打印走 USART1，不能挡在保护动作前面 */
    CHS_TB_SetStandby(0U);

    /* 不用 %.2f：本工程开 MicroLIB，浮点 printf 不可用，统一按 mm/s 打整数 */
    (void)snprintf(buf, sizeof(buf),
                   "[CHS] STALL wheel %c: cmd=%d mm/s, spd=%d mm/s -> FAULT\r\n",
                   (char)('A' + wheel),
                   (int)(cmd_before * 1000.0f),
                   (int)(spd_before * 1000.0f));

    if (print_lock != NULL)
    {
        osMutexAcquire(print_lock, osWaitForever);
        printf("%s", buf);
        osMutexRelease(print_lock);
    }
    else
    {
        printf("%s", buf);
    }
}

/* ==================== 初始化 ==================== */

int CHS_Init(CHS_Handle *h)
{
    uint8_t i;

    if (h == NULL) return CHS_ERR_PARAM;

    h->mode         = CHS_MODE_IDLE;
    h->online       = 0U;
    h->initialized  = 0U;
    h->fault        = CHS_FAULT_NONE;
    h->stop_req     = 0U;
    h->enable_req   = 0U;
    h->last_cmd_tick = HAL_GetTick();
    h->last_tick_ms  = h->last_cmd_tick;
    h->stop_at_tick = 0U;

    for (i = 0U; i < CHS_W_NUM; i++)
    {
        h->w[i].enc_total = 0;
        h->raw_permille[i] = 0;
        chs_wheel_reset(&h->w[i]);
    }

    h->kp = CHS_PID_KP;
    h->ki = CHS_PID_KI;
    h->kd = CHS_PID_KD;

    CHS_ResetOdometry(h);

    /* 底层初始化：GPIO/TIM4 PWM/4 路编码器。失败说明定时器或引脚配置有问题，
       不要继续跑控制环，否则会拿着没起来的 PWM 乱算。 */
    if (CHS_TB_Init() != CHS_TB_OK)
    {
        return CHS_ERR_HW;
    }

    CHS_TB_ResetEncoder();
    CHS_TB_SetStandby(1U);      /* 先使能 H 桥，但 PWM 全 0、DIR 全低 */

    for (i = 0U; i < CHS_W_NUM; i++)
    {
        CHS_TB_Coast(i);
    }

    h->last_tick_ms = HAL_GetTick();   /* 初始化耗时不计入第一拍测速 dt */
    h->initialized = 1U;
    h->online      = 1U;

    return CHS_OK;
}

/* ==================== 目标设定 ==================== */

int CHS_SetVelocity(CHS_Handle *h, float vx_ms, float vy_ms, float wz_rads)
{
    float k = (CHS_HALF_WHEELBASE_MM + CHS_HALF_TRACK_MM) * 0.001f;  /* m */
    float t[CHS_W_NUM];
    float max_abs = 0.0f;
    float scale = 1.0f;
    uint8_t i;
    uint32_t now;

    if (h == NULL) return CHS_ERR_PARAM;
    if (chs_can_move(h) == 0U) return CHS_ERR_STATE;
    /* NaN 防护：NaN 参与比较永远为 false，会绕过下面的限幅，
       最后 (int16_t)NaN 是未定义行为。宁可回参数错，也不下坏指令。 */
    if (!(vx_ms == vx_ms) || !(vy_ms == vy_ms) || !(wz_rads == wz_rads))
    {
        return CHS_ERR_PARAM;
    }

    /* 几何参数填 0 会让自转项变成 0/0；夹一个下限，宁可转得比理论快也不出 NaN */
    if (k < 0.01f) k = 0.01f;

    /* 从 IDLE/OPEN 重新进入闭环时，用当前实测值重置上一拍速度，
       避免 Kd 的测量值微分在首拍拿到一个陈旧的 prev_speed 产生尖峰。 */
    if (h->mode != CHS_MODE_SPEED)
    {
        for (i = 0U; i < CHS_W_NUM; i++)
        {
            h->w[i].prev_speed = h->w[i].speed_ms;
        }
    }

    /* 标准麦轮逆解（R=40mm 已折进线速度，不再除以轮径）
       A 左前 = vx - vy - k*wz
       B 右前 = vx + vy + k*wz
       C 左后 = vx + vy - k*wz
       D 右后 = vx - vy + k*wz */
    t[CHS_W_A] = vx_ms - vy_ms - k * wz_rads;
    t[CHS_W_B] = vx_ms + vy_ms + k * wz_rads;
    t[CHS_W_C] = vx_ms + vy_ms - k * wz_rads;
    t[CHS_W_D] = vx_ms - vy_ms + k * wz_rads;

    for (i = 0U; i < CHS_W_NUM; i++)
    {
        float a = chs_absf(t[i]);
        if (a > max_abs) max_abs = a;
    }

    /* 超轮速上限就等比缩放：只减速，不改运动方向。
       逐轮裁剪会让麦轮合力方向偏掉，车会画弧。 */
    if (max_abs > CHS_MAX_WHEEL_MS)
    {
        scale = CHS_MAX_WHEEL_MS / max_abs;
        for (i = 0U; i < CHS_W_NUM; i++)
        {
            t[i] *= scale;
        }
    }

    now = HAL_GetTick();

    /* 四轮目标 + 模式切换作为一个整体写入：ChassisTask 与本函数同优先级，
       若恰好在两轮之间被时间片切走，会出现一拍「半新半旧」的目标。 */
    taskENTER_CRITICAL();
    for (i = 0U; i < CHS_W_NUM; i++)
    {
        /* 只在「换向」或「从停车/开环重新进入闭环」时清堵转计时。
           连续 $CAR,VEL 心跳（50Hz）不能每帧都清，否则轮子真堵住时
           stall_ms 永远累计不到 300ms，堵转保护形同虚设。 */
        if (chs_sign_flip(h->w[i].target_ms, t[i]) || (h->mode != CHS_MODE_SPEED))
        {
            h->w[i].integ   = 0.0f;
            h->w[i].stall_ms = 0U;
        }
        h->w[i].target_ms = t[i];
    }

    h->stop_req      = 0U;              /* 先撤销上一拍的停车请求，再切模式，避免中间态被 Tick 当成“边开边走” */
    h->stop_at_tick  = 0U;              /* 新指令作废上一次定时停止 */
    h->mode          = CHS_MODE_SPEED;
    h->last_cmd_tick = now;
    taskEXIT_CRITICAL();

    return CHS_OK;
}

int CHS_SetVelocityMs(CHS_Handle *h, float vx_ms, float vy_ms, float wz_rads, uint32_t dur_ms)
{
    int r = CHS_SetVelocity(h, vx_ms, vy_ms, wz_rads);

    if ((r == CHS_OK) && (dur_ms > 0U))
    {
        h->stop_at_tick = HAL_GetTick() + dur_ms;
    }

    return r;
}

int CHS_SetPwmRaw(CHS_Handle *h, int8_t a, int8_t b, int8_t c, int8_t d)
{
    uint32_t now;

    if (h == NULL) return CHS_ERR_PARAM;
    if (chs_can_move(h) == 0U) return CHS_ERR_STATE;

    now = HAL_GetTick();

    /* 四个原始占空比 + 模式切换必须原子写入；否则 Tick 可能拿到“两轮新、两轮旧” */
    taskENTER_CRITICAL();
    /* int8 的 -100~100 乘 10 变成 -1000~1000‰，与 CHS_TB_SetDuty 单位对齐 */
    h->raw_permille[CHS_W_A] = (int16_t)((int16_t)a * 10);
    h->raw_permille[CHS_W_B] = (int16_t)((int16_t)b * 10);
    h->raw_permille[CHS_W_C] = (int16_t)((int16_t)c * 10);
    h->raw_permille[CHS_W_D] = (int16_t)((int16_t)d * 10);

    for (uint8_t i = 0U; i < CHS_W_NUM; i++)
    {
        h->w[i].target_ms = 0.0f;
        h->w[i].cmd_ms    = 0.0f;
        h->w[i].integ     = 0.0f;
        h->w[i].stall_ms = 0U;
    }

    h->stop_req      = 0U;              /* 同 VEL：先撤销停车请求，再切 OPEN */
    h->stop_at_tick  = 0U;
    h->mode          = CHS_MODE_OPEN;
    h->last_cmd_tick = now;
    taskEXIT_CRITICAL();

    return CHS_OK;
}

int CHS_SetGains(CHS_Handle *h, float kp, float ki, float kd)
{
    if (h == NULL) return CHS_ERR_PARAM;
    if (!(kp == kp) || !(ki == ki) || !(kd == kd)) return CHS_ERR_PARAM;

    /* 只接受合理区间：负增益会把速度环变成正反馈；过大等于没有控制。
       这里不做持久化，重启回默认宏，避免调参调坏后砖机。 */
    if ((kp < 0.0f) || (kp > 5000.0f) ||
        (ki < 0.0f) || (ki > 5000.0f) ||
        (kd < 0.0f) || (kd > 2000.0f))
    {
        return CHS_ERR_PARAM;
    }

    h->kp = kp;
    h->ki = ki;
    h->kd = kd;

    for (uint8_t i = 0U; i < CHS_W_NUM; i++)
    {
        h->w[i].integ = 0.0f;   /* 改增益先清积分，避免旧积分突然放大 */
    }

    return CHS_OK;
}

/* ==================== 停车 / 清故障 ==================== */

int CHS_Stop(CHS_Handle *h)
{
    uint8_t i;

    if (h == NULL) return CHS_ERR_PARAM;

    for (i = 0U; i < CHS_W_NUM; i++)
    {
        h->w[i].target_ms = 0.0f;
        h->w[i].cmd_ms    = 0.0f;
        h->w[i].integ     = 0.0f;
        h->w[i].prev_speed = 0.0f;
        h->w[i].stall_ms = 0U;
    }
    for (i = 0U; i < CHS_W_NUM; i++)
    {
        h->raw_permille[i] = 0;
    }

    h->mode         = CHS_MODE_IDLE;
    h->stop_at_tick = 0U;
    h->stop_req     = 1U;   /* 真正写 GPIO/PWM 在下一拍 Tick，避免跨任务抢寄存器 */

    return CHS_OK;
}

int CHS_ClearFault(CHS_Handle *h)
{
    uint8_t i;

    if (h == NULL) return CHS_ERR_PARAM;
    if (h->initialized == 0U) return CHS_ERR_STATE;   /* 没初始化成功，清故障也没意义 */
    if (h->fault == CHS_FAULT_NONE) return CHS_ERR_STATE;  /* 没故障就别顺手把车停了 */

    /* 先把状态改成 IDLE + 停车请求，再清 fault：
       否则 ChassisTask 可能在「fault 已清、mode 还没改」的空隙里跑一拍 PID。 */
    h->mode         = CHS_MODE_IDLE;
    h->stop_at_tick = 0U;
    h->stop_req     = 1U;
    h->enable_req   = 1U;              /* 重新使能 STBY 由下一拍 ChassisTask 执行 */
    h->last_tick_ms = HAL_GetTick();   /* 故障期间可能停了好几秒，别把这段算进下一拍 dt */

    h->fault  = CHS_FAULT_NONE;
    h->online = 1U;

    for (i = 0U; i < CHS_W_NUM; i++)
    {
        chs_wheel_reset(&h->w[i]);
        h->raw_permille[i] = 0;
    }

    return CHS_OK;
}

/* ==================== 速度环 ==================== */

/**
 * @brief  单轮 PID：前馈 + PI + 可选取样值微分。
 * @param  t    限加速后的目标 m/s
 * @param  m    实测 m/s
 * @param  dt_s 本拍实际间隔（s）；积分/微分必须用真实 dt
 * @retval 带符号占空比 ‰（车体约定，未乘 WDIR）
 */
static int16_t chs_pid_wheel(CHS_Handle *h, uint8_t i, float dt_s)
{
    CHS_WheelState *w = &h->w[i];
    float t = w->cmd_ms;
    float m = w->speed_ms;
    float err = t - m;
    float ff;
    float u;
    float d_term;

    if (dt_s < 0.001f) dt_s = CHS_PERIOD_S;   /* 防除 0；正常不会走到 */

    /* 目标与实测反向：先 coast 让轮子自己停，等真正过零再反接。
       TB6612 没有电流环，高速正转时直接反接 = 反接制动，母线电流会冲一下；
       麦轮底盘又重，反复「正→反」最容易打坏 H 桥。这里用一个小死区换安全。 */
    if (((t > 0.01f) && (m < -0.02f)) || ((t < -0.01f) && (m > 0.02f)))
    {
        w->integ     = 0.0f;
        w->prev_speed = m;
        return 0;
    }

    /* 目标、实测都接近 0：直接断输出，别让积分在零点附近来回蹭 */
    if ((chs_absf(t) < CHS_ZERO_TARGET_MS) && (chs_absf(m) < CHS_ZERO_SPEED_MS))
    {
        w->integ = 0.0f;
        w->prev_speed = m;
        return 0;
    }

    /* 积分分离：误差大时先只用 P+前馈把速度拉起来，避免积分很快饱和；
       等误差进了 ±CHS_PID_ERR_GATE 再积分消静差。 */
    if (chs_absf(err) < CHS_PID_ERR_GATE)
    {
        w->integ += err * dt_s;
        w->integ = chs_clampf(w->integ, -CHS_PID_I_LIMIT, CHS_PID_I_LIMIT);
    }

    /* 测量值微分：D = -Kd * d(measured)/dt，不对目标阶跃产生微分冲击 */
    d_term = (m - w->prev_speed) / dt_s;
    w->prev_speed = m;

    /* 前馈：目标速度按静态增益折算成占空比，再加一点静摩擦死区补偿。
       死区只在「有明确目标」时加，目标为 0 时不加，否则会往一个方向蹭。 */
    ff = t * CHS_FF_GAIN;
    if (chs_absf(t) > 0.01f)
    {
        ff += (t > 0.0f) ? CHS_DUTY_DEADBAND : -CHS_DUTY_DEADBAND;
    }

    u = ff + h->kp * err + h->ki * w->integ - h->kd * d_term;
    u = chs_clampf(u, -1000.0f, 1000.0f);

    return (int16_t)u;
}

/**
 * @brief  读四路编码器，更新每轮实测速度与累计计数。
 * @param  dt_ms 本拍实际间隔（HAL tick 测量，不是固定 20ms）
 *         速度换算：count × (mm/count) ÷ dt_ms = mm/ms = m/s，不用再除 1000。
 */
static void chs_read_speeds(CHS_Handle *h, uint32_t dt_ms)
{
    uint8_t i;
    float   dt = (dt_ms == 0U) ? 1.0f : (float)dt_ms;

    static const float edir[CHS_W_NUM] = {
        (float)CHS_EDIR_A, (float)CHS_EDIR_B, (float)CHS_EDIR_C, (float)CHS_EDIR_D
    };

    for (i = 0U; i < CHS_W_NUM; i++)
    {
        int32_t delta = CHS_TB_ReadDelta(i);

        h->w[i].speed_ms  = (float)delta * CHS_MM_PER_COUNT / dt;
        h->w[i].speed_ms *= edir[i];
        h->w[i].enc_total = CHS_TB_GetTotal(i);   /* 底层已经累计，这里只镜像 */
    }
}

/**
 * @brief  麦轮正运动学 + 航向积分（世界系）。
 * @param  dt_ms 本拍实际间隔；里程计必须用真实 dt，否则任务被拖后会多积分。
 */
static void chs_update_odometry(CHS_Handle *h, uint32_t dt_ms)
{
    float k = (CHS_HALF_WHEELBASE_MM + CHS_HALF_TRACK_MM) * 0.001f;  /* m */
    float dt = (dt_ms == 0U) ? CHS_PERIOD_S : ((float)dt_ms * 0.001f);

    if (k < 0.01f) k = 0.01f;

    float a = h->w[CHS_W_A].speed_ms;
    float b = h->w[CHS_W_B].speed_ms;
    float c = h->w[CHS_W_C].speed_ms;
    float d = h->w[CHS_W_D].speed_ms;

    float vx = ( a + b + c + d) * 0.25f;
    float vy = (-a + b + c - d) * 0.25f;
    float wz = (-a + b - c + d) / (4.0f * k);

    float yaw = h->odom_yaw;
    /* 坐标约定：+x 车头向前、+y 车体向右、+yaw 逆时针（俯视）。
       在这个「x 向前、y 向右」的平面里，+yaw 会把车头转向左侧（-y），
       所以世界系变换必须是：
         world_x =  vx·cosθ + vy·sinθ
         world_y = -vx·sinθ + vy·cosθ
       不能套标准的 [cos -sin; sin cos]（那对应 +y 向左/顺时针）。 */
    float dx_m = ( vx * chs_cos(yaw) + vy * chs_sin(yaw)) * dt;
    float dy_m = (-vx * chs_sin(yaw) + vy * chs_cos(yaw)) * dt;

    h->odom_x_mm += dx_m * 1000.0f;
    h->odom_y_mm += dy_m * 1000.0f;
    h->odom_yaw  += wz * dt;

    while (h->odom_yaw >  CHS_PI) h->odom_yaw -= CHS_TWO_PI;
    while (h->odom_yaw < -CHS_PI) h->odom_yaw += CHS_TWO_PI;
}

/* ==================== 20ms 心跳 ==================== */

void CHS_Tick(CHS_Handle *h)
{
    uint8_t  i;
    int16_t  duty[CHS_W_NUM];
    uint32_t now;
    uint32_t dt_ms;
    float    dt_s;
    float    ctrl_dt;
    float    step;

    if ((h == NULL) || (h->initialized == 0U)) return;

    /* ---- ⓪ 实际 dt：任务可能被高优先级打印/协议任务拖后，不能硬按 20ms 算 ---- */
    now   = HAL_GetTick();
    dt_ms = now - h->last_tick_ms;
    if (dt_ms == 0U) dt_ms = 1U;
    h->last_tick_ms = now;

    dt_s = (float)dt_ms * 0.001f;

    /* 控制环用的 dt 夹到 100ms：真被拖了 1s 就别让积分/限加速一步冲到底；
       但测速和里程计仍用真实 dt_ms，宁可慢，不能把速度算翻倍。 */
    ctrl_dt = (dt_s > 0.1f) ? 0.1f : dt_s;

    /* ---- ① 编码器：即使停车/故障也要读，里程计不能断 ---- */
    chs_read_speeds(h, dt_ms);
    chs_update_odometry(h, dt_ms);

    /* ---- ② 已锁故障：输出在锁存时已经全断，保持不动 ---- */
    if (h->fault != CHS_FAULT_NONE) return;

    /* ---- ②b 清故障后重新使能 STBY：硬件仍只在 ChassisTask 里写 ----
       CHS_ClearFault 可能由 ProtocolTask 调用，它只置 enable_req；
       这里先把四轮 coast、再拉高 STBY，保证使能瞬间 PWM=0、IN1=IN2=0。 */
    if (h->enable_req != 0U)
    {
        h->enable_req = 0U;
        for (i = 0U; i < CHS_W_NUM; i++)
        {
            CHS_TB_Coast(i);
        }
        CHS_TB_SetStandby(1U);
    }

    /* ---- ③ 定时停止（$CAR,VEL,...,ms）到期 ---- */
    if (h->stop_at_tick != 0U)
    {
        if ((int32_t)(HAL_GetTick() - h->stop_at_tick) >= 0)
        {
            h->stop_at_tick = 0U;
            for (i = 0U; i < CHS_W_NUM; i++)
            {
                h->w[i].target_ms = 0.0f;
                h->w[i].cmd_ms    = 0.0f;
                h->w[i].integ     = 0.0f;
            }
            h->mode     = CHS_MODE_IDLE;
            h->stop_req = 1U;
        }
    }

    /* ---- ⑤ 执行停车请求：真正写 coast 只在本任务 ---- */
    if (h->stop_req != 0U)
    {
        for (i = 0U; i < CHS_W_NUM; i++)
        {
            CHS_TB_Coast(i);
            h->w[i].duty       = 0;
            h->w[i].target_ms  = 0.0f;
            h->w[i].cmd_ms     = 0.0f;
            h->w[i].integ      = 0.0f;
            h->w[i].stall_ms   = 0U;
            h->w[i].prev_speed = h->w[i].speed_ms;
        }
        h->stop_req = 0U;
        h->mode     = CHS_MODE_IDLE;
    }

    if (h->mode == CHS_MODE_IDLE) return;

    /* ---- ⑥ 开环模式：直接给占空比，不跑 PID ---- */
    if (h->mode == CHS_MODE_OPEN)
    {
        /* 掉线看门狗：500ms 没有新的 RAW 也收油，防 PC 断开后一直转 */
        if ((uint32_t)(HAL_GetTick() - h->last_cmd_tick) > CHS_CMD_TIMEOUT_MS)
        {
            for (i = 0U; i < CHS_W_NUM; i++) h->raw_permille[i] = 0;
        }

        for (i = 0U; i < CHS_W_NUM; i++)
        {
            /* OPEN 是「原始占空比」：不加 WDIR，方便逐轮核对电机转向和接线 */
            int16_t raw = h->raw_permille[i];
            CHS_TB_SetDuty(i, raw);
            h->w[i].duty = raw;
        }
        return;
    }

    /* ---- ⑦ 闭环速度模式：500ms 没新 VEL 自动收油（可控减速，不抱死） ---- */
    if ((h->stop_at_tick == 0U) &&
        ((uint32_t)(HAL_GetTick() - h->last_cmd_tick) > CHS_CMD_TIMEOUT_MS))
    {
        for (i = 0U; i < CHS_W_NUM; i++)
        {
            h->w[i].target_ms = 0.0f;
        }
    }

    /* ---- ⑧ 梯形限加速：每拍最多变 step m/s（用实际控制 dt，不用固定 20ms） ---- */
    step = CHS_ACCEL_MS2 * ctrl_dt;

    for (i = 0U; i < CHS_W_NUM; i++)
    {
        float t = h->w[i].target_ms;
        float c = h->w[i].cmd_ms;

        if (step <= 0.0f)
        {
            c = t;
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
            /* 已到位 */
        }

        h->w[i].cmd_ms = c;
    }

    /* ---- ⑨ 四轮 PID 先全部算出来，堵转判定通过后再统一写硬件 ---- */
    for (i = 0U; i < CHS_W_NUM; i++)
    {
        duty[i] = chs_pid_wheel(h, i, ctrl_dt);
    }

    /* ---- ⑩ 堵转判定：有目标、但按实际 dt 累计 300ms 仍不走 ---- */
    for (i = 0U; i < CHS_W_NUM; i++)
    {
        if ((chs_absf(h->w[i].target_ms) >= CHS_STALL_CMD_MS) &&
            (chs_absf(h->w[i].cmd_ms)    >= CHS_STALL_CMD_MS) &&
            (chs_absf(h->w[i].speed_ms)  <  CHS_STALL_SPD_MS))
        {
            h->w[i].stall_ms += dt_ms;
            if (h->w[i].stall_ms >= CHS_STALL_MS)
            {
                chs_latch_stall(h, i);
                return;
            }
        }
        else
        {
            h->w[i].stall_ms = 0U;
        }
    }

    /* ---- ⑪ 下发：占空比乘方向宏，+ = 车前进 ---- */
    static const int16_t wdir[CHS_W_NUM] = {
        (int16_t)CHS_WDIR_A, (int16_t)CHS_WDIR_B, (int16_t)CHS_WDIR_C, (int16_t)CHS_WDIR_D
    };

    for (i = 0U; i < CHS_W_NUM; i++)
    {
        int16_t raw = (int16_t)(duty[i] * wdir[i]);
        CHS_TB_SetDuty(i, raw);
        h->w[i].duty = raw;
    }
}

/* ==================== 查询 ==================== */

void CHS_GetOdometry(const CHS_Handle *h, float *x_mm, float *y_mm, float *yaw_deg)
{
    if (h == NULL) return;

    if (x_mm    != NULL) *x_mm    = h->odom_x_mm;
    if (y_mm    != NULL) *y_mm    = h->odom_y_mm;
    if (yaw_deg != NULL) *yaw_deg = h->odom_yaw * CHS_RAD_TO_DEG;
}

void CHS_ResetOdometry(CHS_Handle *h)
{
    if (h == NULL) return;

    h->odom_x_mm = 0.0f;
    h->odom_y_mm = 0.0f;
    h->odom_yaw  = 0.0f;
}

void CHS_GetWheelState(const CHS_Handle *h, uint8_t idx,
                       float *target_ms, float *speed_ms, int16_t *duty)
{
    if ((h == NULL) || (idx >= CHS_W_NUM)) return;

    if (target_ms != NULL) *target_ms = h->w[idx].cmd_ms;
    if (speed_ms  != NULL) *speed_ms  = h->w[idx].speed_ms;
    if (duty      != NULL) *duty      = h->w[idx].duty;
}

/* ==================== 台架自检 ==================== */

#if CHS_SELFTEST_EN

/**
 * @brief  带锁打印，避免与 PrintTask 的 STATE 行在 USART1 上逐字节交错。
 */
static void chs_st_log(const char *msg)
{
    if (print_lock != NULL)
    {
        osMutexAcquire(print_lock, osWaitForever);
        printf("%s", msg);
        osMutexRelease(print_lock);
    }
    else
    {
        printf("%s", msg);
    }
}

int CHS_SelfTest(CHS_Handle *h)
{
    static const char *nm[CHS_W_NUM] = { "A(LF)", "B(RF)", "C(LR)", "D(RR)" };
    char    buf[96];
    uint8_t i;
    int     fail = 0;

    if ((h == NULL) || (h->initialized == 0U)) return CHS_ERR_STATE;

    chs_st_log("\r\n[CHS] self-test start: LIFT ALL FOUR WHEELS OFF THE GROUND!\r\n");

    /* 先把四轮停住，再逐轮单独给 ±20%：同一时刻只有一个电机吃电流，
       12V 母线压力最小，也最容易用眼睛看清是哪一轮在转。 */
    for (i = 0U; i < CHS_W_NUM; i++)
    {
        int32_t fwd;
        int32_t rev;
        int     ok;

        CHS_TB_Coast(i);
        (void)CHS_TB_ReadDelta(i);          /* 清掉上一轮残留增量 */

        CHS_TB_SetDuty(i, (int16_t)CHS_ST_DUTY);
        osDelay(CHS_ST_MS);
        fwd = CHS_TB_ReadDelta(i);
        CHS_TB_Coast(i);
        osDelay(100U);                      /* 等轮子惯性停住再换向 */

        (void)CHS_TB_ReadDelta(i);
        CHS_TB_SetDuty(i, (int16_t)(-CHS_ST_DUTY));
        osDelay(CHS_ST_MS);
        rev = CHS_TB_ReadDelta(i);
        CHS_TB_Coast(i);
        osDelay(100U);

        /* 判据只看「正转/反转计数是否符号相反且足够大」，不看绝对符号：
           编码器 A/B 接反时 fwd 会是负的，但正交解码本身是好的；
           这种情况应该判 OK，再让你把该路 EDIR 改成 -1，而不是判失败。 */
        ok = (((fwd > 20L) && (rev < -20L)) || ((fwd < -20L) && (rev > 20L))) ? 1 : 0;

        (void)snprintf(buf, sizeof(buf), "[CHS] %s fwd=%ld rev=%ld -> %s\r\n",
                       nm[i], (long)fwd, (long)rev,
                       (ok != 0) ? "OK" : "CHECK WIRING/ENCODER");
        chs_st_log(buf);

        if (ok == 0)
        {
            fail++;
        }
    }

    (void)snprintf(buf, sizeof(buf), "[CHS] self-test done, %d/%d passed\r\n",
                   (int)(CHS_W_NUM - fail), (int)CHS_W_NUM);
    chs_st_log(buf);

    /* 自检期间没有跑 CHS_Tick，四路编码器累计了好几秒的计数。
       进主循环前必须清零，否则第一拍会把「几秒计数 ÷ 20ms」当成超高速度，
       PID 会瞬间反向刹车，甚至触发堵转误判。 */
    CHS_TB_ResetEncoder();
    for (i = 0U; i < CHS_W_NUM; i++)
    {
        h->w[i].enc_total  = 0;
        h->w[i].speed_ms   = 0.0f;
        h->w[i].prev_speed = 0.0f;
        h->w[i].duty       = 0;
    }

    /* 自检阻塞了好几秒，主循环第一拍不能把这段算进 dt */
    h->last_tick_ms = HAL_GetTick();

    /* 自检只证明「电机转、编码器跟着走」。车体方向按这两条改：
       ① raw + 时该轮应把车往前推；反了就交换该电机红/白线（WDIR 保持 +1）；
       ② 若 fwd 计数为负，说明该轮编码器 A/B 反了，只翻该轮 EDIR 为 -1。 */
    chs_st_log("[CHS] note1: raw + must push the car FORWARD; if not, swap that motor's red/white wires\r\n");
    chs_st_log("[CHS] note2: if fwd counts are negative, flip that wheel's EDIR (A/B swapped)\r\n");

    return (fail == 0) ? CHS_OK : CHS_ERR_PARAM;
}

#endif /* CHS_SELFTEST_EN */
