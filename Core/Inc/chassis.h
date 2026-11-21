/**
 * chassis.h — 四路编码器 TT 马达（1:90）+ 80mm 麦轮 底盘驱动
 * ----------------------------------------------------------------------------
 * 硬件：松甲/Yeahbot「四路编码器电机驱动模块」（AT8236×4 + 板载 MCU，I2C 从机）
 * 协议：官方手册 v1.2 §2.1③ 寄存器表 + 例程 motor_4ch.c 逐字节核对
 *
 * 接线（5264-4P I2C 口只有 3 根线要接）：
 *   SCL → PB6 (I2C1_SCL)
 *   SDA → PB7 (I2C1_SDA)
 *   GND → GND
 *   NC  → 悬空（不要接 VCC！板子由自己的 VM 端子独立供电）
 *   电机口 PH2.0-6P：M- / 5V / A / B / GND / M+（5V 是霍尔供电，板子自己出）
 *   VM 端子：TT 马达 6~8V（本车 6.0V），带拨动开关
 *
 * ⚠️ 三条上电铁律：
 *   1) I2C 是开漏，控制器侧必须有上拉（本工程已在 HAL_I2C_MspInit 里开内部上拉；
 *      若通信偶发 NACK，外接 4.7kΩ 到 3.3V，SCL/SDA 各一只）；
 *   2) 先给驱动板 VM 上电，再跑固件（板载 MCU 没电时会把 SDA 拉死）；
 *   3) 第一次自检把 CHS_SELFTEST_EN 改成 1，轮子必须悬空，确认四轮转向都对再落地。
 *
 * 轮序（手册 §1.2②，装车必须照这个接）：
 *   A = 左前   B = 右前   C = 左后   D = 右后
 *   四路都给正速度时：AC 逆时针、BD 顺时针 → 小车向前。
 * ============================================================================
 */
#ifndef __CHASSIS_H
#define __CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ==================== 配置区（按实机改这里） ==================== */

/* I2C：与机械臂 PCA9685(0x40) 共用 I2C1（PB6=SCL / PB7=SDA，100kHz）。
   两器件地址不同，电气上可共总线；软件上靠 i2c_lock 互斥（见 freertos.c）。 */
extern I2C_HandleTypeDef hi2c1;
#define CHS_I2C_HANDLE          hi2c1
#define CHS_I2C_ADDR7           0x51U       /* 手册 §2.1③：驱动板 7 位地址固定 0x51 */

/* 电机类型（手册 0x03：0=520 / 1=TT / 2=310） */
#define CHS_MOTOR_TYPE          1U          /* 编码器 TT 马达 1:90 */

/* 机械参数（写进驱动板 0x08，让板载闭环自己把 m/s 换算成脉冲） */
#define CHS_WHEEL_D_CM          8U          /* 80mm 麦轮 = 8cm */
#define CHS_ENC_LINES           13U         /* 霍尔编码器 13 线 */
#define CHS_GEAR_RATIO          90U         /* 减速比 1:90 */

/* 由上面三个推出来的常量（不要手改） */
#define CHS_COUNTS_PER_REV      ((float)CHS_ENC_LINES * (float)CHS_GEAR_RATIO * 4.0f)  /* 4680 计数/轮转（4 倍频） */
#define CHS_WHEEL_CIRC_MM       (3.14159265f * (float)CHS_WHEEL_D_CM * 10.0f)          /* 251.327mm */
#define CHS_MM_PER_COUNT        (CHS_WHEEL_CIRC_MM / CHS_COUNTS_PER_REV)               /* 0.05370mm/计数 */

/* 底盘几何（mm）——拿尺子量轮子中心距，填「一半」的值。
   只影响自转（ω）的换算比例，不影响直行；先按 190×150 的常见麦轮底板填。 */
#define CHS_HALF_WHEELBASE_MM   85.0f       /* 前后轮中心距 / 2 */
#define CHS_HALF_TRACK_MM       75.0f       /* 左右轮中心距 / 2 */

/* 控制节奏与限幅 */
#define CHS_PERIOD_MS           20U         /* 与驱动板 20ms 速度环同节拍 */
#define CHS_MAX_WHEEL_MS        0.55f       /* 单轮线速度上限 m/s（6V 空载约 0.63） */
#define CHS_ACCEL_MS2           1.00f       /* 梯形限加速度 m/s²；填 0 = 不限（直接给目标） */
#define CHS_CMD_TIMEOUT_MS      500U        /* 500ms 没新指令自动停车（防上位机掉线疯跑） */

/* I2C 时序（手册例程要求：写寄存器 → 等 ≥2ms → 再读） */
#define CHS_I2C_TIMEOUT_MS      10U         /* 单次传输超时 */
#define CHS_READ_GAP_MS         2U          /* 写寄存器与读数据之间的间隔 */
#define CHS_I2C_RETRY           3U          /* 忙/超时重试次数 */

/* 电池 ADC 分频：每 25 拍（0.5s）读一次 0x00，只做趋势观察 */
#define CHS_BATT_DIV            25U

/* 首次台架自检开关：0=正常启动；1=启动后逐轮正反转并核对编码器计数。
   ⚠️ 自检时四轮必须悬空，结果从 USART1 打印；测完改回 0。 */
#define CHS_SELFTEST_EN         1

/* 返回值 */
#define CHS_OK                  0
#define CHS_ERR_PARAM          -1
#define CHS_ERR_I2C            -2
#define CHS_ERR_OFFLINE        -3

/* 运行模式 */
typedef enum {
    CHS_MODE_IDLE = 0,      /* 停车（速度环与 PWM 都已清零） */
    CHS_MODE_SPEED,         /* 闭环速度模式：每拍写 0x06 */
    CHS_MODE_PWM            /* 开环 PWM 模式：只在调用时写 0x05（台架调试用） */
} CHS_Mode;

/* 轮序：与驱动板接口一一对应（手册 §1.2②） */
typedef enum {
    CHS_W_A = 0,            /* 左前 */
    CHS_W_B,                /* 右前 */
    CHS_W_C,                /* 左后 */
    CHS_W_D,                /* 右后 */
    CHS_W_NUM
} CHS_Wheel;

/* 单轮状态 */
typedef struct {
    int32_t enc_last;       /* 上次累计计数（算增量用） */
    int32_t enc_total;      /* 本地上电以来累计计数（初始化时清零后从 0 起） */
    float   speed_ms;       /* 实测轮面线速度 m/s（正 = 手册定义的前进方向） */
    float   target_ms;      /* 逆解目标 m/s（未限加速度） */
    float   cmd_ms;         /* 限加速度后真正下发的 m/s */
} CHS_WheelState;

/* 底盘状态句柄（全局唯一实例，定义在 chassis.c） */
typedef struct {
    CHS_WheelState w[CHS_W_NUM];
    CHS_Mode mode;          /* 当前运行模式 */
    uint8_t  online;        /* 1 = 最近一次 I2C 读写成功 */
    uint8_t  param_ok;      /* 1 = 0x08 三参数已写入（失败只影响板载测速精度，不影响转动） */
    uint8_t  batt_div;      /* 电池读取分频计数 */
    uint16_t batt_adc;      /* 0x00 原始 ADC 值（非电压，只看趋势） */
    uint32_t last_cmd_tick; /* 最近一次收到速度指令的 tick（超时保护用） */
    uint32_t stop_at_tick;  /* 定时停止到期 tick（0=不启用；由 CHS_SetVelocityMs 设置） */
    float    odom_x_mm;     /* 里程计：世界系 X（mm） */
    float    odom_y_mm;     /* 里程计：世界系 Y（mm） */
    float    odom_yaw;      /* 里程计：航向（rad，-π~π，逆时针为正） */
} CHS_Handle;

extern CHS_Handle g_chassis;    /* 全局实例：自检/协议层/任务都直接用它 */

/* ==================== API ==================== */

/**
 * @brief  上电初始化：电机类型→编码器极性→运动学参数→清脉冲→探活停车。
 *         必须在 FreeRTOS 调度器启动后调用（内部用 osDelay / i2c_lock）。
 * @retval CHS_OK / CHS_ERR_I2C / CHS_ERR_OFFLINE / CHS_ERR_PARAM
 */
int CHS_Init(CHS_Handle *h);

/**
 * @brief  探活：读一次 0x00 电池 ADC（2 字节大端）。成功即 online=1。
 */
int CHS_Probe(CHS_Handle *h);

/**
 * @brief  麦轮逆运动学：车体系速度 → 四轮目标线速度（只存目标，Tick 里限加速下发）。
 * @param  vx_ms   前进速度 m/s（+ 向前）
 * @param  vy_ms   横移速度 m/s（+ 向右）
 * @param  wz_rads 自转速度 rad/s（+ 逆时针，俯视）
 */
int CHS_SetVelocity(CHS_Handle *h, float vx_ms, float vy_ms, float wz_rads);

/**
 * @brief  定时版逆解：与 CHS_SetVelocity 相同，但 dur_ms>0 时运行 dur_ms 后自动停。
 *         期间 500ms 看门狗暂停（由 stop_at_tick 兜底）。dur_ms=0 等价于 CHS_SetVelocity。
 */
int CHS_SetVelocityMs(CHS_Handle *h, float vx_ms, float vy_ms, float wz_rads, uint32_t dur_ms);

/**
 * @brief  绕过逆解，直接给四轮目标线速度 m/s（A/B/C/D）。
 */
int CHS_SetWheelSpeed(CHS_Handle *h, float a_ms, float b_ms, float c_ms, float d_ms);

/**
 * @brief  开环 PWM 直驱（0x05，-100~100），台架验证转向/接线用。
 *         调用后进入 PWM 模式，Tick 不再写速度寄存器。
 */
int CHS_SetPwmRaw(CHS_Handle *h, int8_t a, int8_t b, int8_t c, int8_t d);

/**
 * @brief  立即停车：目标/下发清零，0x06 与 0x05 都写 0，模式转 IDLE。
 */
int CHS_Stop(CHS_Handle *h);

/**
 * @brief  20ms 周期心跳：读四轮累计脉冲 → 算轮速/里程计 → 限加速 → 下发 0x06。
 *         由 ChassisTask 每 20ms 调用一次（osDelayUntil 严格对齐）。
 */
void CHS_Tick(CHS_Handle *h);

/**
 * @brief  读里程计（任一指针可为 NULL）。
 * @param  yaw_deg 航向角，度（-180~180，逆时针为正）
 */
void CHS_GetOdometry(const CHS_Handle *h, float *x_mm, float *y_mm, float *yaw_deg);

/**
 * @brief  里程计清零（不清驱动板累计脉冲；需要同时清就重新 CHS_Init）。
 */
void CHS_ResetOdometry(CHS_Handle *h);

#if CHS_SELFTEST_EN
/**
 * @brief  台架自检（阻塞约 8 秒）：逐轮 +20% / -20% 各转 800ms，核对编码器计数方向。
 *         ⚠️ 轮子必须悬空！结果从 USART1 打印。
 * @retval CHS_OK=四轮全过；否则返回失败轮数相关错误
 */
int CHS_SelfTest(CHS_Handle *h);
#endif

#ifdef __cplusplus
}
#endif
#endif /* __CHASSIS_H */
