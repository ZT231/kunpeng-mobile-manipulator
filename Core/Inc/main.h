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
#define RC522_RST_Pin GPIO_PIN_0
#define RC522_RST_GPIO_Port GPIOB
#define RC522_CS_SDA_Pin GPIO_PIN_12
#define RC522_CS_SDA_GPIO_Port GPIOB
#define SCK_Pin GPIO_PIN_13
#define SCK_GPIO_Port GPIOB
#define MISO_Pin GPIO_PIN_14
#define MISO_GPIO_Port GPIOB
#define MOSI_Pin GPIO_PIN_15
#define MOSI_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* 电位器控制参数（ADC 边界 POT_ADC_MAX 在 potentiometer.h 中定义） */
#define SERVO_STEP_US  25U     /* 每 20ms 最大步进，调大就快 */

/* 每路舵机的脉宽实测标定上下限（us，按机械臂实际行程标定） */
#define SERVO_J1_MIN  800U    /* 基座：左右各约90度 */
#define SERVO_J1_MAX  2300U
#define SERVO_J2_MIN  750U    /* 大臂：机械臂顺时针90°，逆时针90° */
#define SERVO_J2_MAX  2150U
#define SERVO_J3_MIN  520U    /* 小臂：实测 520(逆110°)~2500(顺130°) */
#define SERVO_J3_MAX  2500U
#define SERVO_J4_MIN  500U    /* 腕部俯仰：软限位 500~1850（另行收窄留安全余量） */
#define SERVO_J4_MAX  1850U   /* 物理实测端点 520~2500 仍记在 arm_params.json，勿覆盖 */
#define SERVO_J5_MIN  500U    /* 腕部旋转：MG996R，顺时针70°，逆时针82° */
#define SERVO_J5_MAX  2500U
#define SERVO_J6_MIN  900U   /* 夹爪：900=张开极限；软限位上限 2000（另行收窄留夹持余量） */
#define SERVO_J6_MAX  2000U  /* 物理实测闭合端点 2100 仍记在 arm_params.json / 标定记录，勿覆盖 */

/* 每路舵机的「初始位」脉宽（us）。
 * 大白话：上电瞬间、以及收到 HOME 命令时，六路舵机各自要摆到的那个安全姿态
 *         （折叠收拢、重心靠近基座，不是一条直线）。
 * 规范定义：Arm_Init() 直接写入 PCA9685 的开环位置指令，同时是
 *          Arm_CommandHome() / Arm_CommandResume() 的目标值。
 * ⚠️ 注意区分「初始位」与「标定零位」：标定零位是该关节 0° 对应的脉宽，
 *    记录在 kunpeng/robot/arm_params.json 的 pulse_mid / map 中点，供逆解算角度用。
 *    两者不是一回事 —— 不要拿这里的值去改 arm_params.json，会破坏运动学映射。
 */
#define SERVO_J1_INIT  1580U
#define SERVO_J2_INIT  1900U
#define SERVO_J3_INIT  2400U
#define SERVO_J4_INIT  700U
#define SERVO_J5_INIT  1450U
#define SERVO_J6_INIT  1500U

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
