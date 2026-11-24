/**
 * chs_tb6612.h — 底盘底层：2×TB6612FNG + 4×JGB37-520(1:90) 霍尔编码器
 * ----------------------------------------------------------------------------
 * 这一层直接由 STM32 产生 PWM/DIR、读编码器，不含运动学/PID/里程计：
 *   20ms 速度环和麦轮逆解都在 chassis.c，本层只负责「把占空比写进 H 桥、
 *   把正交计数读回来」。好处是节拍和闭环都在本板，上层只下发 m/s。
 *
 * 🔴 编码器接口的硬事实：
 *   STM32F4 每个通用/高级定时器只有【一个】正交编码器接口，固定占用
 *   CH1/CH2（TI1/TI2）。CH3/CH4 不参与正交解码。四路必须用四个定时器：
 *     A 左前 → TIM3_CH1/CH2  (PB4/PB5,  AF2)
 *     B 右前 → TIM2_CH1/CH2  (PA15/PB3, AF1)
 *     C 左后 → TIM8_CH1/CH2  (PC6/PC7,  AF3)
 *     D 右后 → TIM1_CH1/CH2  (PE9/PE11, AF1)
 *   TIM2 的 PA15/PB3 是 JTAG 的 TDI/TDO；本工程调试用 SWD（PA13/PA14），
 *   把这两个脚改 AF1 不影响 SWD 下载/调试，但请勿再把调试口切回 JTAG。
 *
 * 🔴 引脚为什么避开 PB2：
 *   PB2 = BOOT1 strap，部分核心板焊了下拉/跳线帽，不建议当普通输出。
 *   本文件的映射全部落在当前 .ioc 未使用、也不与任何外设冲突的脚上。
 *
 * 轮序（全项目统一，装车必须照这个接）：
 *   A = 左前 LF   B = 右前 RF   C = 左后 LR   D = 右后 RR
 * ============================================================================
 */
#ifndef __CHS_TB6612_H
#define __CHS_TB6612_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ==================== 本模块占用引脚（21 条） ====================
 * 全部由 CHS_TB_Init() 运行期配置，不经过 CubeMX（.ioc 不含这些脚）。
 *
 *   PWM（TIM4，APB1 定时器时钟 84MHz，ARR=4199 → 20kHz）
 *     PD12 → CH1 → A 左前        PD13 → CH2 → B 右前
 *     PD14 → CH3 → C 左后        PD15 → CH4 → D 右后
 *
 *   DIR（8 根 GPIO 推挽输出，低速；模块 1 = A/B，模块 2 = C/D）
 *     PC0  AIN1 ┐ A 左前         PC2  BIN1 ┐ B 右前
 *     PC1  AIN2 ┘                PC3  BIN2 ┘
 *     PC10 AIN1 ┐ C 左后         PC12 BIN1 ┐ D 右后
 *     PC11 AIN2 ┘（模块 2）       PC13 BIN2 ┘（模块 2）
 *
 *   STBY（1 根）
 *     PE8 → 两块 TB6612 的 STBY 并联；高 = 使能，低 = 待机（H 桥全关）
 *
 *   编码器（4 个定时器各一路正交；AF 输入 + 内部上拉）
 *     A  PB4 / PB5   ← TIM3_CH1/CH2  AF2
 *     B  PA15/ PB3   ← TIM2_CH1/CH2  AF1（JTAG 的 TDI/TDO，SWD 模式可释放）
 *     C  PC6 / PC7   ← TIM8_CH1/CH2  AF3
 *     D  PE9 / PE11  ← TIM1_CH1/CH2  AF1
 *
 * 与固件其它外设零重叠：USART1(PA9/PA10)、USART2(PD5/PD6)、
 * USART3(PB10/PB11)、SWD(PA13/PA14)、HSE(PH0/PH1)、KEY0(PE4)。
 * 全项目引脚总表见 Core/Inc/main.h 的「引脚分配总表」。
 * ================================================================= */

/* ==================== 轮位编号 ==================== */
typedef enum {
    CHS_TB_A = 0,       /* 左前 LF */
    CHS_TB_B,           /* 右前 RF */
    CHS_TB_C,           /* 左后 LR */
    CHS_TB_D,           /* 右后 RR */
    CHS_TB_NUM          /* 4 */
} CHS_TB_Wheel;

/* ==================== PWM（TIM4 CH1~CH4 → PD12~PD15） ==================== */
/* 20kHz：高于人耳，且 TB6612 手册允许最高 100kHz。
   TIM4 挂在 APB1，定时器时钟 = 84MHz（SYSCLK 168MHz、APB1 四分频后倍频回来）。
   ARR = 84MHz / 20kHz - 1 = 4199，占空比分辨率 1/4200 ≈ 0.024%。 */
#define CHS_TB_PWM_TIM              TIM4
#define CHS_TB_PWM_AF               GPIO_AF2_TIM4
#define CHS_TB_PWM_PORT             GPIOD
#define CHS_TB_PWM_PIN_A            GPIO_PIN_12
#define CHS_TB_PWM_PIN_B            GPIO_PIN_13
#define CHS_TB_PWM_PIN_C            GPIO_PIN_14
#define CHS_TB_PWM_PIN_D            GPIO_PIN_15
#define CHS_TB_PWM_ARR              4199U

/* ==================== DIR（8 根，模块 1=A/B，模块 2=C/D） ==================== */
/* 全部放在 GPIOC 的空闲脚，同一端口也方便把 8 根 DIR 排成一条线。 */
#define CHS_TB_DIR_A_PORT           GPIOC
#define CHS_TB_DIR_A_PIN1           GPIO_PIN_0      /* AIN1 */
#define CHS_TB_DIR_A_PIN2           GPIO_PIN_1      /* AIN2 */

#define CHS_TB_DIR_B_PORT           GPIOC
#define CHS_TB_DIR_B_PIN1           GPIO_PIN_2      /* BIN1 */
#define CHS_TB_DIR_B_PIN2           GPIO_PIN_3      /* BIN2 */

#define CHS_TB_DIR_C_PORT           GPIOC
#define CHS_TB_DIR_C_PIN1           GPIO_PIN_10     /* AIN1（模块 2 的 A 通道） */
#define CHS_TB_DIR_C_PIN2           GPIO_PIN_11     /* AIN2 */

#define CHS_TB_DIR_D_PORT           GPIOC
#define CHS_TB_DIR_D_PIN1           GPIO_PIN_12     /* BIN1（模块 2 的 B 通道） */
#define CHS_TB_DIR_D_PIN2           GPIO_PIN_13     /* BIN2 */

/* ==================== STBY（两块并联 → PE8） ==================== */
/* TB6612 STBY 内部 200kΩ 下拉，悬空=待机；必须主动拉高才出力。
   低电平待机时 H 桥全关，是最可靠的「断电级」软件停车。 */
#define CHS_TB_STBY_PORT            GPIOE
#define CHS_TB_STBY_PIN             GPIO_PIN_8

/* ==================== 四路编码器 ==================== */
/* A：TIM3_CH1/CH2 → PB4/PB5（AF2，PB4 复位后是 NJTRST，SWD 模式下可复用） */
#define CHS_TB_ENC_A_TIM            TIM3
#define CHS_TB_ENC_A_PERIOD         0xFFFFU
#define CHS_TB_ENC_A_PORT1          GPIOB
#define CHS_TB_ENC_A_PIN1           GPIO_PIN_4
#define CHS_TB_ENC_A_PORT2          GPIOB
#define CHS_TB_ENC_A_PIN2           GPIO_PIN_5
#define CHS_TB_ENC_A_AF             GPIO_AF2_TIM3

/* B：TIM2_CH1/CH2 → PA15/PB3（AF1，JTAG 的 TDI/TDO；SWD 调试可释放） */
#define CHS_TB_ENC_B_TIM            TIM2
#define CHS_TB_ENC_B_PERIOD         0xFFFFFFFFU
#define CHS_TB_ENC_B_PORT1          GPIOA
#define CHS_TB_ENC_B_PIN1           GPIO_PIN_15
#define CHS_TB_ENC_B_PORT2          GPIOB
#define CHS_TB_ENC_B_PIN2           GPIO_PIN_3
#define CHS_TB_ENC_B_AF             GPIO_AF1_TIM2

/* C：TIM8_CH1/CH2 → PC6/PC7（AF3） */
#define CHS_TB_ENC_C_TIM            TIM8
#define CHS_TB_ENC_C_PERIOD         0xFFFFU
#define CHS_TB_ENC_C_PORT1          GPIOC
#define CHS_TB_ENC_C_PIN1           GPIO_PIN_6
#define CHS_TB_ENC_C_PORT2          GPIOC
#define CHS_TB_ENC_C_PIN2           GPIO_PIN_7
#define CHS_TB_ENC_C_AF             GPIO_AF3_TIM8

/* D：TIM1_CH1/CH2 → PE9/PE11（AF1） */
#define CHS_TB_ENC_D_TIM            TIM1
#define CHS_TB_ENC_D_PERIOD         0xFFFFU
#define CHS_TB_ENC_D_PORT1          GPIOE
#define CHS_TB_ENC_D_PIN1           GPIO_PIN_9
#define CHS_TB_ENC_D_PORT2          GPIOE
#define CHS_TB_ENC_D_PIN2           GPIO_PIN_11
#define CHS_TB_ENC_D_AF             GPIO_AF1_TIM1

/* ==================== 返回值 ==================== */
#define CHS_TB_OK                   0
#define CHS_TB_ERR_TIM             -2

/* ==================== API ==================== */

/**
 * @brief  初始化 GPIO + TIM4 PWM + 4 路编码器。上电默认 STBY=低、PWM=0。
 *         必须在 FreeRTOS 调度器启动后调用（内部只做寄存器级配置，不阻塞）。
 * @retval CHS_TB_OK / CHS_TB_ERR_TIM
 */
int CHS_TB_Init(void);

/**
 * @brief  给单个轮子设带符号占空比。
 * @param  wheel    CHS_TB_A~D
 * @param  permille -1000~1000（正=该轮「正转」，负=反转，0=滑行）
 * @note   正负只代表 H 桥方向，是否等于「车前进」由 chassis.c 的方向宏决定。
 *         PWM 先写 CCR 再改 DIR；duty=0 时 IN1=IN2=0（coast，不刹车）。
 */
void CHS_TB_SetDuty(uint8_t wheel, int16_t permille);

/**
 * @brief  滑行：PWM=0，IN1=IN2=0，H 桥上下管全关，轮子自由转。
 */
void CHS_TB_Coast(uint8_t wheel);

/**
 * @brief  两块 TB6612 的 STBY 总开关（PE8，两块并联）。
 * @param  enable 1=拉高使能；0=待机（H 桥全关，最可靠的软件断电）
 */
void CHS_TB_SetStandby(uint8_t enable);

/**
 * @brief  读某轮「自上次调用以来」的带符号计数增量（4 倍频后）。
 * @note   16 位定时器用无符号相减再转 int16，天然处理回绕；
 *         TIM2 是 32 位，用 int32 相减。一次 20ms 最多约 145 计数，
 *         远不会溢出，但保留回绕写法是为了换电机/换轮径后仍然正确。
 */
int32_t CHS_TB_ReadDelta(uint8_t wheel);

/**
 * @brief  本地上电以来该轮的累计原始计数（未乘方向宏）。
 */
int32_t CHS_TB_GetTotal(uint8_t wheel);

/**
 * @brief  把四路计数器、增量基准、累计值全部清零。
 */
void CHS_TB_ResetEncoder(void);

#ifdef __cplusplus
}
#endif

#endif /* __CHS_TB6612_H */
