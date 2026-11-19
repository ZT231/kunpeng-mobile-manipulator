#include "motor.h"

typedef enum
{
    MOTOR_STATE_STOP = 0,
    MOTOR_STATE_FORWARD,
    MOTOR_STATE_BACK
} MotorState_t;

static uint16_t g_motor_speed = MOTOR_SPEED_FAST;
static MotorState_t g_left_state = MOTOR_STATE_STOP;
static MotorState_t g_right_state = MOTOR_STATE_STOP;

static void Motor_SetCompare(uint8_t channel, uint16_t value)
{
    if (value > MOTOR_PWM_PERIOD)
        value = MOTOR_PWM_PERIOD;

    switch (channel)
    {
        case 1:
            TIM_SetCompare1(MOTOR_TIM, value);
            break;

        case 2:
            TIM_SetCompare2(MOTOR_TIM, value);
            break;

        case 3:
            TIM_SetCompare3(MOTOR_TIM, value);
            break;

        case 4:
            TIM_SetCompare4(MOTOR_TIM, value);
            break;

        default:
            break;
    }
}

static void Motor_Output_Update(void)
{
    uint16_t left_in1 = 0;
    uint16_t left_in2 = 0;
    uint16_t right_in1 = 0;
    uint16_t right_in2 = 0;

    // 右电机：TIM8_CH1、TIM8_CH2
    if (g_right_state == MOTOR_STATE_FORWARD)
    {
        right_in1 = g_motor_speed;
        right_in2 = 0;
    }
    else if (g_right_state == MOTOR_STATE_BACK)
    {
        right_in1 = 0;
        right_in2 = g_motor_speed;
    }

    // 左电机：TIM8_CH3、TIM8_CH4
    if (g_left_state == MOTOR_STATE_FORWARD)
    {
        left_in1 = g_motor_speed;
        left_in2 = 0;
    }
    else if (g_left_state == MOTOR_STATE_BACK)
    {
        left_in1 = 0;
        left_in2 = g_motor_speed;
    }

    Motor_SetCompare(1, right_in1);
    Motor_SetCompare(2, right_in2);
    Motor_SetCompare(3, left_in1);
    Motor_SetCompare(4, left_in2);
}

static void Car_DIR_IO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = MOTOR_R_IN1 | MOTOR_R_IN2 |
                                    MOTOR_L_IN1 | MOTOR_L_IN2;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_Init(MOTOR_PORT, &GPIO_InitStructure);

    GPIO_PinAFConfig(MOTOR_PORT, GPIO_PinSource6, MOTOR_AF);
    GPIO_PinAFConfig(MOTOR_PORT, GPIO_PinSource7, MOTOR_AF);
    GPIO_PinAFConfig(MOTOR_PORT, GPIO_PinSource8, MOTOR_AF);
    GPIO_PinAFConfig(MOTOR_PORT, GPIO_PinSource9, MOTOR_AF);
}

void Car_PWM_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0};
    TIM_OCInitTypeDef TIM_OCInitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

    TIM_DeInit(MOTOR_TIM);

    TIM_TimeBaseStructure.TIM_Prescaler     = MOTOR_PWM_PRESCALER;
    TIM_TimeBaseStructure.TIM_Period        = MOTOR_PWM_PERIOD;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;

    TIM_TimeBaseInit(MOTOR_TIM, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 0;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;

    TIM_OC1Init(MOTOR_TIM, &TIM_OCInitStructure);
    TIM_OC2Init(MOTOR_TIM, &TIM_OCInitStructure);
    TIM_OC3Init(MOTOR_TIM, &TIM_OCInitStructure);
    TIM_OC4Init(MOTOR_TIM, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(MOTOR_TIM, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(MOTOR_TIM, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(MOTOR_TIM, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(MOTOR_TIM, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(MOTOR_TIM, ENABLE);

    // TIM8 是高级定时器，必须开启主输出
    TIM_CtrlPWMOutputs(MOTOR_TIM, ENABLE);
    TIM_Cmd(MOTOR_TIM, ENABLE);
}

void Car_GPIO_Init(void)
{
    Car_DIR_IO_Init();
    Car_PWM_Init();
    Car_Stop();
}

void Car_SetSpeed(uint16_t pwm_val)
{
    if (pwm_val > MOTOR_PWM_PERIOD)
        pwm_val = MOTOR_PWM_PERIOD;

    g_motor_speed = pwm_val;
    Motor_Output_Update();
}

void Car_ResetAll(void)
{
    Car_Stop();
    Car_SetSpeed(MOTOR_SPEED_FAST);
}

void Car_Stop(void)
{
    g_left_state = MOTOR_STATE_STOP;
    g_right_state = MOTOR_STATE_STOP;
    Motor_Output_Update();
}

void Car_Forward(void)
{
    g_left_state = MOTOR_STATE_FORWARD;
    g_right_state = MOTOR_STATE_FORWARD;
    Motor_Output_Update();
}

void Car_Back(void)
{
    g_left_state = MOTOR_STATE_BACK;
    g_right_state = MOTOR_STATE_BACK;
    Motor_Output_Update();
}

void Car_Spin_Left(void)
{
    g_left_state = MOTOR_STATE_BACK;
    g_right_state = MOTOR_STATE_FORWARD;
    Motor_Output_Update();
}

void Car_Spin_Right(void)
{
    g_left_state = MOTOR_STATE_FORWARD;
    g_right_state = MOTOR_STATE_BACK;
    Motor_Output_Update();
}

void Car_Turn_Left(void)
{
    g_left_state = MOTOR_STATE_STOP;
    g_right_state = MOTOR_STATE_FORWARD;
    Motor_Output_Update();
}

void Car_Turn_Right(void)
{
    g_left_state = MOTOR_STATE_FORWARD;
    g_right_state = MOTOR_STATE_STOP;
    Motor_Output_Update();
}
