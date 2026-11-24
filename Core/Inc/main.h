/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY0_Pin GPIO_PIN_4
#define KEY0_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* ============================ 引脚分配总表 ============================
 * 本表是固件引脚的唯一权威来源，与 RoboticArm_Kunpeng.ioc + chs_tb6612.h 同步。
 * 封装 F407VET6 / LQFP100：可用 GPIO 共 82 个
 *   PA0~PA15 / PB0~PB15 / PC0~PC15 / PD0~PD15 / PE0~PE15 / PH0~PH1
 * 已占用 32 个 · 结构禁用 1 个 · 净空闲 49 个。
 *
 * ── 一、CubeMX 管理（.ioc 内 13 条，改这里必须同步改 .ioc）────────────
 *   PA9      USART1_TX   AF7      PC 上位机 / 统一调试口    115200 8N1
 *   PA10     USART1_RX   AF7
 *   PD5      USART2_TX   AF7      HC-05 蓝牙              115200 8N1
 *   PD6      USART2_RX   AF7
 *   PB10     USART3_TX   AF7      预留第三控制源（当前未接）  115200 8N1
 *   PB11     USART3_RX   AF7
 *   PA13     SWDIO               SWD 调试（禁止改作他用）
 *   PA14     SWCLK               SWD 调试（禁止改作他用）
 *   PH0      OSC_IN              HSE 8MHz 晶振
 *   PH1      OSC_OUT             HSE 8MHz 晶振
 *   PE4      KEY0      输入上拉   板载按键，当前固件未读取（预留）
 *
 * ── 二、底盘驱动（chs_tb6612.c 运行期配置，.ioc 不管理，21 条）────────
 *   PD12~PD15   TIM4_CH1~CH4   AF2   四路 20kHz PWM（A/B/C/D 轮）
 *   PC0 / PC1   AIN1 / AIN2           A 左前方向
 *   PC2 / PC3   BIN1 / BIN2           B 右前方向
 *   PC10/PC11   AIN1 / AIN2           C 左后方向（模块 2）
 *   PC12/PC13   BIN1 / BIN2           D 右后方向（模块 2）
 *   PE8         STBY                  两块 TB6612 并联使能
 *   PB4 / PB5   TIM3_CH1/CH2   AF2    编码器 A（左前）
 *   PA15/ PB3   TIM2_CH1/CH2   AF1    编码器 B（右前；JTAG 脚，SWD 模式下可用）
 *   PC6 / PC7   TIM8_CH1/CH2   AF3    编码器 C（左后）
 *   PE9 / PE11  TIM1_CH1/CH2   AF1    编码器 D（右后）
 *
 * ── 三、禁止占用（3 处，占用会导致无法烧录或无法启动）────────────────
 *   PA13 / PA14   SWD，占用即失去下载/调试
 *   PH0  / PH1    HSE 晶振
 *   PB2           BOOT1 strap，部分核心板带下拉/跳线帽
 *
 * ── 四、净空闲（49 个，按端口列出）──────────────────────────────────
 *   PA0~PA8, PA11, PA12                              11 个
 *   PB0, PB1, PB6~PB9, PB12~PB15                     10 个
 *   PC4, PC5, PC8, PC9, PC14, PC15                    6 个
 *   PD0~PD4, PD7~PD11                                10 个
 *   PE0~PE3, PE5~PE7, PE10, PE12~PE15                12 个
 *
 *   ⚠️ 上表是「芯片级」可用集：开发板板载 LED / 按键 / SD 卡 / USB 会额外
 *      占脚，接线前先用万用表或原理图确认该脚是否已被板载外设使用。
 *   ⚠️ PC14 / PC15 驱动能力弱（约 3mA），只适合低速信号，不驱动 LED。
 *   ⚠️ PB6 / PB7 现完全空闲（I2C1 外设已从 .ioc 移除）；将来做 I2C 扩展
 *      可直接 AF 到 I2C1，无需改线序。
 * ==================================================================== */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
