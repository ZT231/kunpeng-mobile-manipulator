#ifndef __SERVO_H
#define __SERVO_H
#include "stm32f4xx.h"

#define SERVO_TIM TIM3
#define SERVO_AF GPIO_AF_TIM3
#define SERVO_PIN GPIO_Pin_6
#define SERVO_PORT GPIOA
#define SERVO_MIN_CCR 500
#define SERVO_MAX_CCR 2500
#define SERVO_MID_CCR 1500

void Servo_Init(void);
void Servo_SetAngle(uint16_t angle);
void Servo_Reset(void); // 舵机回中，模式切换复位
#endif
