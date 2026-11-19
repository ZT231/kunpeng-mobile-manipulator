#include "avoidance.h"

static CarState_t state = STATE_FORWARD;
static float dist_l = 999.0f;
static float dist_r = 999.0f;
static float dist_front = 999.0f;
static uint32_t state_tick = 0;
static uint32_t scan_tick = 0;
static uint32_t turn_check_tick = 0;
static uint8_t turn_left = 1;
static uint8_t turn_retry = 0;
static uint16_t back_time = AVOID_BACK_MS;

static void IR_Init(void)
{
    GPIO_InitTypeDef gp = {0};

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    gp.GPIO_Pin  = IR_L | IR_R;
    gp.GPIO_Mode = GPIO_Mode_IN;
    gp.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_Init(IR_PORT, &gp);
}

static uint8_t IR_L_Read(void)
{
    return GPIO_ReadInputDataBit(IR_PORT, IR_L);
}

static uint8_t IR_R_Read(void)
{
    return GPIO_ReadInputDataBit(IR_PORT, IR_R);
}

static void Ultra_Init(void)
{
    GPIO_InitTypeDef gp = {0};

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB |
                           RCC_AHB1Periph_GPIOE, ENABLE);

    gp.GPIO_Pin   = TRIG_PIN;
    gp.GPIO_Mode  = GPIO_Mode_OUT;
    gp.GPIO_Speed = GPIO_Speed_100MHz;
    gp.GPIO_OType = GPIO_OType_PP;
    gp.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(TRIG_PORT, &gp);

    gp.GPIO_Pin   = ECHO_PIN;
    gp.GPIO_Mode  = GPIO_Mode_IN;
    gp.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(ECHO_PORT, &gp);

    GPIO_ResetBits(TRIG_PORT, TRIG_PIN);
}

static float Ultra_GetDist(void)
{
    uint32_t t_start;
    uint32_t t_echo_start;
    uint32_t t_echo_end;
    uint32_t us;

    // 触发 HC-SR04
    GPIO_SetBits(TRIG_PORT, TRIG_PIN);
    delay_us(12);
    GPIO_ResetBits(TRIG_PORT, TRIG_PIN);

    t_start = micros();
    while (!GPIO_ReadInputDataBit(ECHO_PORT, ECHO_PIN))
    {
        if (micros() - t_start > ULTRA_TIMEOUT_US)
            return 999.0f; // 超时按无障碍处理
    }

    t_echo_start = micros();
    while (GPIO_ReadInputDataBit(ECHO_PORT, ECHO_PIN))
    {
        if (micros() - t_echo_start > ULTRA_TIMEOUT_US)
            return 999.0f;
    }

    t_echo_end = micros();
    us = t_echo_end - t_echo_start;

    // 声速换算：距离(cm)=回波时间(us)/58
    return (float)us / 58.0f;
}

static uint8_t Obstacle_Detected(float dist)
{
    if (IR_L_Read() == 0 || IR_R_Read() == 0)
        return 1;

    if (dist <= AVOID_NEAR_CM)
        return 1;

    return 0;
}

static void Avoidance_StartBack(uint32_t now, uint16_t time_ms)
{
    Car_SetSpeed(MOTOR_SPEED_SLOW);
    Car_Back();
    back_time = time_ms;
    state_tick = now;
    state = STATE_BACK;
}

static void Avoidance_StartTurn(uint32_t now)
{
    Servo_SetAngle(SERVO_CENTER_ANGLE);
    turn_check_tick = now;
    state_tick = now;
    state = STATE_TURN;
}

void Avoidance_Init(void)
{
    IR_Init();
    Ultra_Init();

    Servo_Init();
    Servo_SetAngle(SERVO_CENTER_ANGLE);

    state = STATE_FORWARD;
    dist_l = 999.0f;
    dist_r = 999.0f;
    dist_front = 999.0f;
    state_tick = millis();
    scan_tick = state_tick;
    turn_check_tick = state_tick;
    turn_left = 1;
    turn_retry = 0;
    back_time = AVOID_BACK_MS;
}

void Avoidance_Reset(void)
{
    Car_Stop();
    Car_SetSpeed(MOTOR_SPEED_FAST);
    Servo_SetAngle(SERVO_CENTER_ANGLE);

    state = STATE_FORWARD;
    dist_l = 999.0f;
    dist_r = 999.0f;
    dist_front = 999.0f;
    state_tick = millis();
    scan_tick = state_tick;
    turn_check_tick = state_tick;
    turn_left = 1;
    turn_retry = 0;
    back_time = AVOID_BACK_MS;
}

void Avoidance_Run(void)
{
    uint32_t now;

    now = millis();

    switch (state)
    {
        case STATE_FORWARD:
        {
            // 舵机保持 90 度，持续探测正前方
            Servo_SetAngle(SERVO_CENTER_ANGLE);

            if (now - scan_tick < AVOID_SCAN_INTERVAL_MS)
                break;

            scan_tick = now;
            dist_front = Ultra_GetDist();

            if (Obstacle_Detected(dist_front))
            {
                Car_Stop();
                // 每次遇障先后退，避免车头贴住墙后原地转不动
                Avoidance_StartBack(now, AVOID_BACK_MS);
            }
            else if (dist_front <= AVOID_FAR_CM)
            {
                Car_SetSpeed(MOTOR_SPEED_MID);
                Car_Forward();
            }
            else
            {
                Car_SetSpeed(MOTOR_SPEED_FAST);
                Car_Forward();
            }
            break;
        }

        case STATE_BACK:
        {
            Car_SetSpeed(MOTOR_SPEED_SLOW);
            Car_Back();

            if (now - state_tick >= back_time)
            {
                Car_Stop();
                // 先扫左侧 45 度
                Servo_SetAngle(SERVO_LEFT_ANGLE);
                state_tick = now;
                state = STATE_SCAN_LEFT;
            }
            break;
        }

        case STATE_SCAN_LEFT:
        {
            Car_Stop();

            if (now - state_tick >= SERVO_WAIT_MS)
            {
                dist_l = Ultra_GetDist();
                // 再扫右侧 135 度；只扫单侧容易在墙角作出错误判断
                Servo_SetAngle(SERVO_RIGHT_ANGLE);
                state_tick = now;
                state = STATE_SCAN_RIGHT;
            }
            break;
        }

        case STATE_SCAN_RIGHT:
        {
            Car_Stop();

            if (now - state_tick >= SERVO_WAIT_MS)
            {
                dist_r = Ultra_GetDist();
                Servo_SetAngle(SERVO_CENTER_ANGLE);

                // 选择距离更大的方向；相等时交替，避免固定方向绕圈
                if (dist_l > dist_r)
                    turn_left = 1;
                else if (dist_r > dist_l)
                    turn_left = 0;
                else
                    turn_left = !turn_left;

                Avoidance_StartTurn(now);
            }
            break;
        }

        case STATE_TURN:
        {
            Car_SetSpeed(MOTOR_SPEED_SLOW);

            // 原地转向比单轮转向角度更大，适合从角落脱困
            if (turn_left)
                Car_Spin_Left();
            else
                Car_Spin_Right();

            // 转过最小角度后，重新测量正前方；不再只依赖固定转向时间
            if (now - turn_check_tick >= AVOID_TURN_CHECK_MS)
            {
                turn_check_tick = now;
                dist_front = Ultra_GetDist();

                if (now - state_tick >= AVOID_TURN_MIN_MS &&
                    !Obstacle_Detected(dist_front))
                {
                    Car_Stop();
                    state_tick = now;
                    state = STATE_VERIFY;
                }
            }

            // 转向超过上限仍无路，后退并换方向，防止死循环
            if (now - state_tick >= AVOID_TURN_MAX_MS)
            {
                Car_Stop();
                turn_retry++;
                turn_left = !turn_left;

                if (turn_retry <= AVOID_MAX_RETRY)
                    Avoidance_StartBack(now, AVOID_BACK_LONG_MS);
                else
                {
                    // 多次失败后保留交替方向策略，重新执行一次完整探测
                    turn_retry = 0;
                    Avoidance_StartBack(now, AVOID_BACK_LONG_MS);
                }
            }
            break;
        }

        case STATE_VERIFY:
        {
            // 转向后短距离前进确认，避免刚转开又立即回到原障碍状态
            Car_SetSpeed(MOTOR_SPEED_SLOW);
            Car_Forward();

            if (now - state_tick >= AVOID_VERIFY_MS)
            {
                dist_front = Ultra_GetDist();
                Car_Stop();
                state_tick = now;

                if (Obstacle_Detected(dist_front))
                {
                    // 确认仍有障碍，继续换方向脱困
                    turn_left = !turn_left;
                    turn_retry++;
                    Avoidance_StartBack(now, AVOID_BACK_LONG_MS);
                }
                else
                {
                    turn_retry = 0;
                    state = STATE_IDLE;
                }
            }
            break;
        }

        case STATE_IDLE:
        {
            Car_Stop();

            if (now - state_tick >= AVOID_IDLE_MS)
            {
                Servo_SetAngle(SERVO_CENTER_ANGLE);
                scan_tick = now;
                Car_SetSpeed(MOTOR_SPEED_FAST);
                state = STATE_FORWARD;
            }
            break;
        }

        default:
        {
            Car_Stop();
            Servo_SetAngle(SERVO_CENTER_ANGLE);
            state = STATE_FORWARD;
            scan_tick = now;
            break;
        }
    }
}
