/**
 * chassis.h — 四轮麦克纳姆底盘控制层（TB6612 + JGB37-520 1:90 + 80mm 麦轮）
 * ----------------------------------------------------------------------------
 * 分层：
 *   底层 chs_tb6612.c：TIM4 PWM、8 根 DIR、STBY、4 路定时器编码器；
 *   本层 chassis.c  ：麦轮逆/正运动学、20ms 速度环 PID、梯形限加速、
 *                     编码器测速、里程计、堵转保护、指令超时、台架自检。
 *
 * 单位约定（全层统一，别混）：
 *   vx/vy  m/s      +x 车头向前，+y 车体向右
 *   wz     rad/s    +逆时针（俯视）
 *   轮速   m/s      轮面线速度，+ = 该轮驱动车体向前
 *   占空比 ‰         -1000~1000，对应 TB6612 PWM 0~100%
 *   计数   count    编码器 4 倍频后的原始计数
 *
 * 机械常量（按实车改）：
 *   JGB37-520 12V 1:90，空载 110rpm，额定 85rpm，堵转 3.2A；
 *   霍尔编码器 11 线，A/B 正交，4 倍频 ⇒ 11×90×4 = 3960 计数/输出轴圈；
 *   80mm 麦轮周长 π×80 = 251.33mm ⇒ 0.06347mm/计数；
 *   110rpm × 0.25133m = 0.4607m/s（空载极限），所以限 0.45m/s。
 *
 * 🔴 轮距/轴距必须拿尺子量：
 *   CHS_HALF_WHEELBASE_MM = 前后轮中心距 / 2
 *   CHS_HALF_TRACK_MM     = 左右轮中心距 / 2
 *   它们只影响自转 ω 的换算和里程计航向，不影响直行；先按 170×150 的
 *   常见亚克力底板填 85/75，装车后实测回填。
 * ============================================================================
 */
#ifndef __CHASSIS_H
#define __CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "chs_tb6612.h"
#include <stdint.h>

/* ==================== 机械参数（实车标定后固化） ==================== */
#define CHS_WHEEL_D_MM              80.0f       /* 80mm 麦轮 */
#define CHS_ENC_LINES               11U         /* 霍尔编码器 11 线/电机轴圈 */
#define CHS_GEAR_RATIO              90U         /* 减速比 1:90 */

#define CHS_COUNTS_PER_REV          ((float)CHS_ENC_LINES * (float)CHS_GEAR_RATIO * 4.0f)
#define CHS_WHEEL_CIRC_MM           (3.14159265f * CHS_WHEEL_D_MM)
#define CHS_MM_PER_COUNT            (CHS_WHEEL_CIRC_MM / CHS_COUNTS_PER_REV)

/* 底盘几何（mm，填「一半」） */
#define CHS_HALF_WHEELBASE_MM       85.0f       /* 前后轮中心距 / 2，实测回填 */
#define CHS_HALF_TRACK_MM           75.0f       /* 左右轮中心距 / 2，实测回填 */

/* ==================== 控制节奏与限幅 ==================== */
#define CHS_PERIOD_MS               20U         /* 速度环周期，与 ChassisTask 一致 */
#define CHS_PERIOD_S                0.020f
#define CHS_MAX_WHEEL_MS            0.45f       /* 单轮线速度上限（空载极限 0.4607） */
#define CHS_ACCEL_MS2               1.20f       /* 梯形限加速度，m/s²；0=不限 */
#define CHS_CMD_TIMEOUT_MS          500U        /* 500ms 无新 VEL 自动收油（防掉线） */

/* ==================== 速度环 PID（20ms 一拍） ==================== */
/* 前馈把「目标 m/s」直接折算成占空比，PID 只修残差。
   静态增益估算：1000‰ / 0.46m/s ≈ 2174‰/(m/s)，取 1800 留一点给 PI 补。
   调参顺序：先把 KP/KI 都置 0，只留前馈，看车能不能大致跟上；
   再从小往大加 KP 到「不抖但跟得快」；最后加 KI 消静差。 */
#define CHS_PID_KP                  700.0f      /* ‰ 每 (m/s) */
#define CHS_PID_KI                  900.0f      /* ‰ 每 (m/s·s) */
#define CHS_PID_KD                  0.0f        /* 测量值微分；编码器噪声大时保持 0 */
#define CHS_PID_I_LIMIT             0.25f       /* 积分限幅（m/s·s），防饱和 */
#define CHS_PID_ERR_GATE            0.15f       /* 误差大于此值暂停积分（积分分离） */
#define CHS_FF_GAIN                 1800.0f     /* 前馈 ‰/(m/s) */
#define CHS_DUTY_DEADBAND           70.0f       /* 静摩擦死区补偿 ‰（只在有目标时加） */

/* ==================== 堵转保护 ==================== */
/* TB6612 没有电流反馈脚，只能用「给了占空比但编码器不走」判堵转。
   JGB37-520 堵转 3.2A，正好等于 TB6612 的峰值上限（连续只有 1.2A/路）；
   所以判据不能拖：连续 300ms 给油不走就锁故障、拉低 STBY。
   300ms 已经留够了「从静止克服静摩擦」的时间，再长就可能烧驱动。 */
#define CHS_STALL_CMD_MS            0.08f       /* 目标与限加速后的 cmd 都大于此值才启用判据 */
#define CHS_STALL_SPD_MS            0.02f       /* 实测低于此值算「不走」 */
#define CHS_STALL_MS                300U        /* 连续「给油不走」累计 300ms 就锁故障 */

/* 目标与实测都接近 0 时直接输出 0，避免 PID 在零点附近抖 */
#define CHS_ZERO_TARGET_MS          0.005f
#define CHS_ZERO_SPEED_MS           0.030f

/* ==================== 台架自检 ==================== */
/* 0=正常启动（默认，安全）；1=启动后逐轮正反转核对编码器。
   ⚠️ 打开后四轮必须悬空！测完改回 0。 */
#define CHS_SELFTEST_EN             0
#define CHS_ST_DUTY                 200         /* 自检占空比 ‰ = 20% */
#define CHS_ST_MS                   500U        /* 每个方向 500ms */

/* ==================== 方向宏（装车后按实车改） ==================== */
/* WDIR：目标「车前进」→ 该电机正转的方向。
   接线规则（推荐）：四个轮子统一按 红→AO1、白→AO2 接；用 $CAR,RAW 给正数，
   若该轮把车往后推，就交换该电机的红/白两根线。这样四个轮子的 WDIR 都是 +1，
   麦轮逆解「四轮同为正 = 前进」才成立。
   如果你坚持不换线，也可以把该轮 WDIR 改成 -1；但必须同时按自检结果修正
   该轮 EDIR（见下），否则速度环会把「前进」测成负速度，越跑越反。 */
#define CHS_WDIR_A                  (+1)        /* 左前 */
#define CHS_WDIR_B                  (+1)        /* 右前 */
#define CHS_WDIR_C                  (+1)        /* 左后 */
#define CHS_WDIR_D                  (+1)        /* 右后 */

/* EDIR：编码器原始计数 → 「车前进为正」的换算。
   自检里 raw + 时 fwd 计数为正 → EDIR=+1；fwd 为负 → EDIR=-1（A/B 对调）。
   正常接线（红→AO1、白→AO2、黄→A、绿→B）四个都是 +1。
   只翻 EDIR 不动 WDIR 是「只交换编码器 A/B」的软件等价；如果是换电机线，
   WDIR 和 EDIR 会一起翻，别只改一个。 */
#define CHS_EDIR_A                  (+1)
#define CHS_EDIR_B                  (+1)
#define CHS_EDIR_C                  (+1)
#define CHS_EDIR_D                  (+1)

/* ==================== 故障码 ==================== */
#define CHS_FAULT_NONE              0x00U
#define CHS_FAULT_STALL             0x01U       /* 堵转：占空比有、编码器不走 */

/* ==================== 返回值 ==================== */
#define CHS_OK                      0
#define CHS_ERR_PARAM              -1
#define CHS_ERR_STATE              -2           /* 未初始化 / 已锁故障 */
#define CHS_ERR_HW                 -3           /* 定时器/GPIO 初始化失败 */

/* ==================== 运行模式 ==================== */
typedef enum {
    CHS_MODE_IDLE = 0,      /* 停车（目标/输出都清零） */
    CHS_MODE_SPEED,         /* 闭环速度：$CAR,VEL 走的模式 */
    CHS_MODE_OPEN           /* 开环占空比：$CAR,RAW 台架接线用 */
} CHS_Mode;

/* 轮序与底层一致：A 左前 / B 右前 / C 左后 / D 右后 */
typedef enum {
    CHS_W_A = CHS_TB_A,
    CHS_W_B = CHS_TB_B,
    CHS_W_C = CHS_TB_C,
    CHS_W_D = CHS_TB_D,
    CHS_W_NUM = CHS_TB_NUM
} CHS_Wheel;

/* 单轮状态 */
typedef struct {
    int32_t enc_total;      /* 本地上电以来累计原始计数（未乘 EDIR） */
    float   speed_ms;       /* 实测轮面线速度，+ = 车前进方向 */
    float   target_ms;      /* 逆解目标（未限加速度） */
    float   cmd_ms;         /* 限加速度后真正送进 PID 的目标 */
    float   integ;          /* PID 积分项（m/s·s） */
    float   prev_speed;     /* 上一拍实测值，用于 Kd 的测量值微分 */
    int16_t duty;           /* 最近一次下发的带符号占空比 ‰（已乘 WDIR） */
    uint32_t stall_ms;      /* 连续「给油不走」累计时间（ms），按实际 dt 累加 */
} CHS_WheelState;

/* 底盘状态句柄（全局唯一实例，定义在 chassis.c） */
typedef struct {
    CHS_WheelState w[CHS_W_NUM];
    CHS_Mode mode;          /* 当前模式 */
    uint8_t  online;        /* 1 = 已初始化且无锁存故障 */
    uint8_t  initialized;   /* 1 = CHS_Init 成功 */
    uint8_t  fault;         /* CHS_FAULT_* 位掩码 */
    uint8_t  stop_req;      /* 1 = 请求下一拍立即 coast（跨任务安全停车） */
    uint8_t  enable_req;    /* 1 = 请求下一拍重新拉高 STBY（清故障后由 ChassisTask 执行） */
    int16_t  raw_permille[CHS_W_NUM]; /* OPEN 模式：-1000~1000 目标占空比 */
    uint32_t last_cmd_tick; /* 最近一次收到 VEL/RAW 的 HAL tick */
    uint32_t last_tick_ms;  /* 上一拍 CHS_Tick 的 HAL tick（算实际 dt，防任务被拖后测速翻倍） */
    uint32_t stop_at_tick;  /* 定时停止到期 tick（0=不启用） */
    float    odom_x_mm;     /* 里程计世界系 X（mm） */
    float    odom_y_mm;     /* 里程计世界系 Y（mm） */
    float    odom_yaw;      /* 航向（rad，-π~π，逆时针为正） */
    float    kp, ki, kd;    /* 速度环增益（可由 $CAR,GAIN 在线改，不持久化） */
} CHS_Handle;

extern CHS_Handle g_chassis;    /* 全局实例：协议层/任务直接引用 */

/* ==================== API ==================== */

/**
 * @brief  初始化底层 + 清状态 + 使能 STBY + 上电先停车。
 *         必须在 ChassisTask（调度器启动后）调用，且是唯一调用点。
 * @retval CHS_OK / CHS_ERR_HW
 */
int CHS_Init(CHS_Handle *h);

/**
 * @brief  麦轮逆运动学：车体系速度 → 四轮目标线速度（只存目标，Tick 里下发）。
 *         超限时按比例等比缩放，保持运动方向不变（不逐轮硬裁剪）。
 * @param  vx_ms   前进 m/s（+向前）
 * @param  vy_ms   横移 m/s（+向右）
 * @param  wz_rads 自转 rad/s（+逆时针，俯视）
 */
int CHS_SetVelocity(CHS_Handle *h, float vx_ms, float vy_ms, float wz_rads);

/**
 * @brief  定时版逆解：dur_ms>0 时跑 dur_ms 后自动停；dur_ms=0 等价于持续模式。
 */
int CHS_SetVelocityMs(CHS_Handle *h, float vx_ms, float vy_ms, float wz_rads, uint32_t dur_ms);

/**
 * @brief  开环占空比直驱（-100~100，即 -1000~1000‰），台架验证转向/接线用。
 *         调用后进入 OPEN 模式，Target 清零；实际写入在下一拍 Tick（≤20ms）。
 *         范围故意限 ±100%，协议层再限 ±30%，防台架误操作飞车。
 */
int CHS_SetPwmRaw(CHS_Handle *h, int8_t a, int8_t b, int8_t c, int8_t d);

/**
 * @brief  在线改速度环增益（不写 Flash，重启恢复默认宏）。
 */
int CHS_SetGains(CHS_Handle *h, float kp, float ki, float kd);

/**
 * @brief  请求停车：目标/积分清零，mode=IDLE，下一拍 coast（跨任务安全）。
 */
int CHS_Stop(CHS_Handle *h);

/**
 * @brief  清除锁存故障：fault=0、online=1；下一拍由 ChassisTask 重新拉高 STBY 并停车。
 *         本函数只改软件标志，不跨任务写 GPIO。
 */
int CHS_ClearFault(CHS_Handle *h);

/**
 * @brief  20ms 周期心跳：读编码器 → 测速/里程计 → 超时/限加速 → PID → 下发。
 *         只允许 ChassisTask 调用；其它任务只改目标值。
 */
void CHS_Tick(CHS_Handle *h);

/**
 * @brief  读里程计（任一指针可为 NULL）。
 * @param  yaw_deg 航向角，度（-180~180，逆时针为正）
 */
void CHS_GetOdometry(const CHS_Handle *h, float *x_mm, float *y_mm, float *yaw_deg);

/**
 * @brief  里程计清零（不动编码器累计值）。
 */
void CHS_ResetOdometry(CHS_Handle *h);

/**
 * @brief  读单轮状态（供 $CAR,WHEEL 遥测/调参）。
 * @param  idx       CHS_W_A~D
 * @param  target_ms 限加速后的目标 m/s（可 NULL）
 * @param  speed_ms  实测 m/s（可 NULL）
 * @param  duty     最近下发占空比 ‰（可 NULL）
 */
void CHS_GetWheelState(const CHS_Handle *h, uint8_t idx,
                       float *target_ms, float *speed_ms, int16_t *duty);

#if CHS_SELFTEST_EN
/**
 * @brief  台架自检（阻塞约 4×2×500ms）：逐轮 +20% / -20%，核对编码器增量方向。
 *         ⚠️ 四轮必须悬空！结果从 USART1 打印。
 *         只验证「电机转、编码器跟着走」，不验证车体前进方向；
 *         raw + 倒转就交换该电机红/白线（WDIR 保持 +1）；编码器 A/B 反了只翻 EDIR。
 */
int CHS_SelfTest(CHS_Handle *h);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_H */
