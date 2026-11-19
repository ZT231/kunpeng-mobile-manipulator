#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx.h"

// ================= L9110S 电机输入引脚 =================
// MOTOR A：右侧两个同步马达
#define MOTOR_R_IN1       GPIO_Pin_6
#define MOTOR_R_IN2       GPIO_Pin_7

// MOTOR B：左侧两个同步马达
#define MOTOR_L_IN1       GPIO_Pin_8
#define MOTOR_L_IN2       GPIO_Pin_9

#define MOTOR_PORT        GPIOC
#define MOTOR_AF          GPIO_AF_TIM8
#define MOTOR_TIM         TIM8

// TIM8_CH1 -> PC6 -> 右电机 A-1A
// TIM8_CH2 -> PC7 -> 右电机 A-1B
// TIM8_CH3 -> PC8 -> 左电机 B-1A
// TIM8_CH4 -> PC9 -> 左电机 B-1B

#define MOTOR_PWM_PERIOD  999

// TIM8 时钟按常见 STM32F4 168MHz 系统时钟配置
// 168MHz / 168 = 1MHz，PWM 频率为 1MHz / 1000 = 1kHz
#define MOTOR_PWM_PRESCALER  167

// 三档速度，可直接修改，范围 0~999
#define MOTOR_SPEED_FAST   999
#define MOTOR_SPEED_MID    850
#define MOTOR_SPEED_SLOW   700

void Car_GPIO_Init(void);
void Car_PWM_Init(void);

void Car_Stop(void);
void Car_SetSpeed(uint16_t pwm_val);

void Car_Forward(void);
void Car_Back(void);
void Car_Spin_Left(void);
void Car_Spin_Right(void);

void Car_Turn_Left(void);
void Car_Turn_Right(void);

void Car_ResetAll(void);

#endif
