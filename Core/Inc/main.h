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
 * 已占用 33 个 · 结构禁用 1 个 · 净空闲 48 个。
 *
 * 本映射按「当前成品 F407VET6 核心板（DevEBox STM32F4VE：板载 W25Q16 + SD 卡座
 * + NRF24L01 排座 + TFT/触摸排座）的机器人专用工况」制定：关闭/不插板载 FLASH、
 * NRF、TFT，只把 F407 当底盘控制器。SD 卡座（SDIO 4 位）与本固件引脚不冲突。
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
 *   PB12/PB13   AIN1 / AIN2           C 左后方向（模块 2）
 *   PB14/PB15   BIN1 / BIN2           D 右后方向（模块 2）
 *   PE8         STBY                  两块 TB6612 并联使能
 *   PB4 / PB5   TIM3_CH1/CH2   AF2    编码器 A（左前；与 W25Q16 MISO/MOSI 共用）
 *   PA5 / PB3   TIM2_CH1/CH2   AF1    编码器 B（右前；PA5 空闲，PB3=W25Q16 CLK 兼 NRF SCK）
 *   PC6 / PC7   TIM8_CH1/CH2   AF3    编码器 C（左后；SDIO_D6/D7，4 位 SD 不用）
 *   PE9 / PE11  TIM1_CH1/CH2   AF1    编码器 D（右后）
 *
 * ── 三、板载外设隔离（1 条，成品板兼容）────────────────────────────
 *   PB0         W25Q16_CS   GPIO 输出高   拉高片选，禁止板载 SPI FLASH 选中（板上 R20=10K 上拉）
 *
 * ── 四、禁止占用（3 处，占用会导致无法烧录或无法启动）────────────────
 *   PA13 / PA14   SWD，占用即失去下载/调试
 *   PH0  / PH1    HSE 晶振
 *   PB2           BOOT1 strap，部分核心板带下拉/跳线帽
 *
 * ── 五、净空闲（48 个，按端口列出；板载排座未插模块时才可自由使用）────
 *   PA0~PA4, PA6~PA8, PA11, PA12, PA15              11 个
 *   PB1, PB6~PB9                                     5 个
 *   PC4, PC5, PC8~PC15                              10 个
 *   PD0~PD4, PD7~PD11                               10 个
 *   PE0~PE3, PE5~PE7, PE10, PE12~PE15               12 个
 *
 *   ⚠️ PB3/PB4/PB5 与板载 W25Q16 的 CLK/MISO/MOSI 共用，也接到 NRF24L01 排座；
 *      本固件已把 FLASH CS(PB0) 配成输出高，FLASH 全程不选中。编码器由 TB6612 板
 *      MOTOR 座的 3.3V 供电、A/B 输出为 3.3V 逻辑，与 FLASH 电平兼容，无需拆片/钳位。
 *      前提：不插 NRF24L01、不开 SPI1 外设、FLASH CS(PB0) 保持高。
 *   ⚠️ PB12~PB15 是本板 TFT 触摸屏 SPI2（T_CS/T_SCK/T_MISO/T_MOSI）；
 *      不插 TFT 时才是普通 GPIO，插屏则这 4 根 DIR 冲突。
 *   ⚠️ PD12~PD15 / PE8 / PE9 / PE11 同时是板载 TFT 的 FSMC 排座脚；
 *      不插 3.5" TFT 时可用，插屏则 PWM/STBY/编码器 D 全部冲突。
 *   ⚠️ PB0 已用作 FLASH CS；PB1 是 TFT 背光(LCD_BL) 且另有 servo_bus 的 DE 预留宏
 *      （当前未加入工程），当普通输出前确认不插 TFT、且不再启用 servo_bus。
 *   ⚠️ PC10~PC13 虽在空闲表内，但 PC10~PC12 是 SDIO_D2/D3/CK、PC13 属备份域
 *      弱驱动且部分板带 LED；不要拿来驱动 TB6612 方向，也不要驱动 LED。
 *      编码器 C 的 PC6/PC7 只是 SDIO_D6/D7 的复用名，本板 SD 卡为 4 位 SDIO
 *      （DAT0~3=PC8/PC9/PC10/PC11），PC6/PC7 未接到卡座，插 SD 卡也不冲突。
 *   ⚠️ PC5 在本板是 TFT 触摸 PEN 中断脚，PA0 是 WK_UP 按键，PA6/PA7 是 LED D2/D3；
 *      当普通 IO 前先确认不插 TFT、不依赖按键/LED。
 *   ⚠️ PC14 / PC15 驱动能力弱（约 3mA），只适合低速信号，不驱动 LED。
 *   ⚠️ PB6 / PB7 / PB8 是 NRF24L01 排座的 CE/CS/IRQ（本板无 24C02 EEPROM）；
 *      不插 NRF 时是普通 GPIO。
 * ==================================================================== */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
