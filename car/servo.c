#include "servo.h"

void Servo_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_DeInit(SERVO_TIM); 

    GPIO_InitStructure.GPIO_Pin = SERVO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(SERVO_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(SERVO_PORT, GPIO_PinSource6, SERVO_AF);

    TIM_TimeBaseStructure.TIM_Prescaler = 83;	//TIM3£¨84MHz£©
    TIM_TimeBaseStructure.TIM_Period = 19999;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(SERVO_TIM, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = SERVO_MID_CCR;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(SERVO_TIM, &TIM_OCInitStructure);
    TIM_Cmd(SERVO_TIM, ENABLE);
}

void Servo_SetAngle(uint16_t angle)
{
    uint16_t ccr_int;
    float ccr;  
    if(angle > 180) angle = 180;
    ccr = SERVO_MIN_CCR + (float)(SERVO_MAX_CCR - SERVO_MIN_CCR) / 180.0f * angle;
    ccr_int = (uint16_t)ccr;
    TIM_SetCompare1(SERVO_TIM, ccr_int);
}

void Servo_Reset(void)
{
    Servo_SetAngle(90);
}
