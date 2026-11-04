#include "arm_control.h"
#include "pca9685.h"
#include "i2c.h"

/* 六路舵机软限位表，直接由 main.h 中的标定宏初始化。
 * 后续实测标定后，只需改 main.h 的 SERVO_JX_MIN/MAX，此表自动跟随。
 */
static const uint16_t servo_min[ARM_CHANNEL_COUNT] =
{
    SERVO_J1_MIN, SERVO_J2_MIN, SERVO_J3_MIN,
    SERVO_J4_MIN, SERVO_J5_MIN, SERVO_J6_MIN
};

static const uint16_t servo_max[ARM_CHANNEL_COUNT] =
{
    SERVO_J1_MAX, SERVO_J2_MAX, SERVO_J3_MAX,
    SERVO_J4_MAX, SERVO_J5_MAX, SERVO_J6_MAX
};

static const uint16_t servo_mid[ARM_CHANNEL_COUNT] =
{
    SERVO_J1_MID, SERVO_J2_MID, SERVO_J3_MID,
    SERVO_J4_MID, SERVO_J5_MID, SERVO_J6_MID
};

/* 目标脉宽、当前脉宽、HOME 位 */
static uint16_t target_pulse[ARM_CHANNEL_COUNT];
static uint16_t current_pulse[ARM_CHANNEL_COUNT];
static uint16_t home_pulse[ARM_CHANNEL_COUNT];

/* 当前状态机状态，上电/复位后停在 HOME 保持（指令态），电位器不接管 */
static Arm_State_t arm_state = ARM_STATE_CMD;

/* 把脉宽裁剪到 [min,max] 软限位内 */
static uint16_t clamp_pulse(uint8_t ch, uint16_t pulse)
{
    if (pulse < servo_min[ch]) return servo_min[ch];
    if (pulse > servo_max[ch]) return servo_max[ch];
    return pulse;
}

/* 电位器 ADC(0~4095) 线性映射到该通道脉宽范围 */
static uint16_t map_adc_to_pulse(uint8_t ch, uint16_t adc)
{
    uint32_t range = (uint32_t)servo_max[ch] - (uint32_t)servo_min[ch];
    uint32_t p = ((uint32_t)adc * range) / POT_ADC_MAX + (uint32_t)servo_min[ch];
    return clamp_pulse(ch, (uint16_t)p);
}

void Arm_Init(void)
{
    uint8_t i;

    /* 上电停在 HOME 保持（指令态），不自动进入手动态。
     * 否则复位/上电瞬间电位器会接管舵机导致乱动；须收到 MOVE/JOINT/HOME 才动。 */
    arm_state = ARM_STATE_CMD;

    for (i = 0U; i < ARM_CHANNEL_COUNT; i++)
    {
        home_pulse[i] = servo_mid[i];
        target_pulse[i] = home_pulse[i];
        current_pulse[i] = home_pulse[i];
    }

    /* 上电先把六路输出到安全的 HOME 位 */
    for (i = 0U; i < ARM_CHANNEL_COUNT; i++)
    {
        PCA9685_SetServoPulse(&hi2c1, i, current_pulse[i]);
    }
}

void Arm_SetManualTargets(const Pot_Data_t *pot)
{
    uint8_t i;

    if (pot == NULL) return;
    if (arm_state != ARM_STATE_MANUAL && arm_state != ARM_STATE_RECORD) return;

    for (i = 0U; i < ARM_CHANNEL_COUNT; i++)
    {
        target_pulse[i] = map_adc_to_pulse(i, pot->adc_value[i]);
    }
}

void Arm_Update(void)
{
    uint8_t i;

    /* 急停：冻结，不更新输出 */
    if (arm_state == ARM_STATE_ESTOP) return;

    for (i = 0U; i < ARM_CHANNEL_COUNT; i++)
    {
        int32_t diff = (int32_t)target_pulse[i] - (int32_t)current_pulse[i];

        if (diff > (int32_t)SERVO_STEP_US)
        {
            current_pulse[i] = (uint16_t)((int32_t)current_pulse[i] + (int32_t)SERVO_STEP_US);
        }
        else if (diff < -(int32_t)SERVO_STEP_US)
        {
            current_pulse[i] = (uint16_t)((int32_t)current_pulse[i] - (int32_t)SERVO_STEP_US);
        }
        else
        {
            current_pulse[i] = target_pulse[i];
        }

        PCA9685_SetServoPulse(&hi2c1, i, current_pulse[i]);
    }
}

uint8_t Arm_SetJointTarget(uint8_t ch, uint16_t pulse_us)
{
    if (ch >= ARM_CHANNEL_COUNT) return 0U;
    if (arm_state == ARM_STATE_ESTOP) return 0U;

    target_pulse[ch] = clamp_pulse(ch, pulse_us);
    arm_state = ARM_STATE_CMD;
    return 1U;
}

uint8_t Arm_SetAllTargets(const uint16_t pulse6[ARM_CHANNEL_COUNT])
{
    uint8_t i;

    if (pulse6 == NULL) return 0U;
    if (arm_state == ARM_STATE_ESTOP) return 0U;

    for (i = 0U; i < ARM_CHANNEL_COUNT; i++)
    {
        target_pulse[i] = clamp_pulse(i, pulse6[i]);
    }
    arm_state = ARM_STATE_CMD;
    return 1U;
}

void Arm_CommandHome(void)
{
    uint8_t i;

    if (arm_state == ARM_STATE_ESTOP) return;

    for (i = 0U; i < ARM_CHANNEL_COUNT; i++)
    {
        target_pulse[i] = home_pulse[i];
    }
    arm_state = ARM_STATE_CMD;
}

void Arm_CommandStop(void)
{
    arm_state = ARM_STATE_ESTOP;
}

void Arm_CommandResume(void)
{
    uint8_t i;

    /* 仅从急停恢复；恢复后先回 HOME 保持（指令态），避免电位器接管乱动。
     * 如需手动拧电位器，请显式发送 MANUAL 命令。 */
    if (arm_state == ARM_STATE_ESTOP)
    {
        for (i = 0U; i < ARM_CHANNEL_COUNT; i++)
        {
            target_pulse[i] = home_pulse[i];
        }
        arm_state = ARM_STATE_CMD;
    }
}

void Arm_CommandManual(void)
{
    /* 急停态禁止切手动：仅 RESUME 可解除急停，防止急停瞬间被电位器接管乱动 */
    if (arm_state == ARM_STATE_ESTOP) return;
    arm_state = ARM_STATE_MANUAL;
}

void Arm_CommandRecord(void)
{
    if (arm_state == ARM_STATE_ESTOP) return;
    arm_state = ARM_STATE_RECORD;
}

void Arm_CommandPlay(void)
{
    if (arm_state == ARM_STATE_ESTOP) return;
    arm_state = ARM_STATE_PLAYBACK;
}

void Arm_SetPlaybackTargets(const uint16_t pulse6[ARM_CHANNEL_COUNT])
{
    uint8_t i;

    if (pulse6 == NULL) return;

    for (i = 0U; i < ARM_CHANNEL_COUNT; i++)
    {
        target_pulse[i] = clamp_pulse(i, pulse6[i]);
    }
    /* 注意：不改变 arm_state，回放态由 Arm_CommandPlay 维持 */
}

Arm_State_t Arm_GetState(void)
{
    return arm_state;
}

uint16_t Arm_GetCurrent(uint8_t ch)
{
    if (ch >= ARM_CHANNEL_COUNT) return 0U;
    return current_pulse[ch];
}

