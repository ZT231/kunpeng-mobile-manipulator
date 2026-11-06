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

/* 六路「初始位」表：上电与 HOME 命令的目标脉宽，来自 main.h 的 SERVO_Jx_INIT。
 * 注意：这不是运动学 0° 的标定零位（那个在 arm_params.json 的 map 里）。 */
static const uint16_t servo_init[ARM_CHANNEL_COUNT] =
{
    SERVO_J1_INIT, SERVO_J2_INIT, SERVO_J3_INIT,
    SERVO_J4_INIT, SERVO_J5_INIT, SERVO_J6_INIT
};

/* 目标脉宽、当前脉宽、初始位（HOME 位） */
static uint16_t target_pulse[ARM_CHANNEL_COUNT];
static uint16_t current_pulse[ARM_CHANNEL_COUNT];
static uint16_t home_pulse[ARM_CHANNEL_COUNT];

/* 当前状态机状态，上电/复位后停在 HOME 保持（指令态），电位器不接管 */
static Arm_State_t arm_state = ARM_STATE_CMD;

/* ================== 平滑过渡参数（只改"怎么走"，不改"走到哪"）==================
 *
 * 原来的写法是每 20ms 固定走 SERVO_STEP_US（25μs），速度曲线是一条方波：
 * 启动瞬间从 0 跳到 112°/s，停下瞬间又从 112°/s 掉回 0，加速度理论上是
 * 无穷大 —— 所以启停会"咯噔"一下。
 *
 * 现在改成"离目标越近走得越慢"，本质是一个比例（P）控制：
 *     本拍步长 = 剩余距离 / ARM_STEP_DIV      （再夹在 [MIN, MAX] 之间）
 * 剩余距离大时被 SERVO_STEP_US 封顶，速度跟原来一样快；快到位时步长自动
 * 缩小，柔滑落位，而且天然不过冲（速度正比于距离，是指数收敛）。
 *
 * ★ 量化底线：PCA9685 在 50Hz 下把 20000μs 分成 4096 级，
 *   一级 = 20000/4096 = 4.88μs（见 pca9685.c 的 pulse_us*4096/20000 整数除法）。
 *   比这更小的指令增量算出的计数值不变，那几拍等于白写。
 *
 * ★ 为什么最小步长取 12μs（≈2.5 个量化级），而不是 5μs（1 个量化级）：
 *   第一版按"1 个量化级"取 5μs，实机上网页 ±20μs 的微调被拆成 4 拍（80ms），
 *   舵机落进约 22°/s 的极低速区。模拟舵机在这个速度会"粘滑爬行"
 *   （静摩擦卡住 → 力矩攒够猛跳一格 → 再卡住），手感就是抖。
 *   而且 5μs 与 4.88μs 比值 1.024 不整除 —— 有的拍跨 2 格、有的拍跨 0 格，
 *   输出本身就不均匀。抬到 12μs 就跳出了这个灰色地带。
 *
 * ★ 小步不拆分：剩余差值 ≤ SERVO_STEP_US(25μs) 时一拍走完。
 *   否则 20μs 的微调要被拆成 4 拍，既慢又不均匀。
 *
 * ★ 为什么 ARM_STEP_DIV 用 4 而不是 8（1500μs 满行程实测）：
 *   指数收敛的尾巴很长。DIV=8 时末端减速段约 21 拍（0.42s），动作收尾会
 *   明显"黏"；DIV=4 只要约 10 拍（0.20s），总时长 66 拍（1.32s）vs 原来
 *   固定步长的 60 拍（1.20s）—— 只慢 0.12s，基本不牺牲速度。
 */
#define ARM_USE_PROPORTIONAL_RAMP  1   /* 1=比例减速（新） 0=固定步长（旧，便于对比） */
#define ARM_STEP_MIN_US            12U /* 最小步长 ≈ 2.5 个 PWM 量化级，避开低速粘滑区 */
#define ARM_STEP_DIV               4   /* 减速陡度：越大越慢越柔和 */

/* 上一次"真正写成功"的脉宽。
 * 0xFFFF 不可能出现在正常脉宽里（软限位最大 2500），用它表示"还没下发过"。
 * 作用：值没变就不再重复发 I2C。运动时每拍都在变，省不了；但机械臂大部分
 *       时间是静止的，旧代码静止时也会每 20ms 把 6 路重写一遍（每路约 0.7ms，
 *       共约 4.2ms，占 20ms 周期的 21%），现在这部分是 0。 */
#define PULSE_SENT_NONE  0xFFFFU
static uint16_t last_sent[ARM_CHANNEL_COUNT];

/* 算出本拍该走多少 μs（剩余差值为 0 时返回 0）。 */
static uint16_t step_of(int32_t diff)
{
#if ARM_USE_PROPORTIONAL_RAMP
    int32_t d;      /* 剩余距离（取绝对值） */
    int32_t step;

    /* 回放态要严格跟着录制的轨迹走，不能做减速收尾：
     * 目标每一拍都在变，减速会让每一拍都落后一点，越放越慢、越放越小。
     * 所以回放保持原来的固定步长。 */
    if (arm_state == ARM_STATE_PLAYBACK) return (uint16_t)SERVO_STEP_US;

    d = (diff < 0) ? -diff : diff;

    /* 小步一拍到位：差值不超过一个满步长就不再拆分。
     * 例：网页 ±20μs 的微调 → 1 拍（20ms）走完，而不是拆成 4 拍（80ms）
     *     每拍 5μs 掉进低速粘滑区。这一句同时掐掉了指数收敛的长尾巴。 */
    if (d <= (int32_t)SERVO_STEP_US) return (uint16_t)d;

    step = d / ARM_STEP_DIV;                                             /* 越近越慢 */
    if (step > (int32_t)SERVO_STEP_US) step = (int32_t)SERVO_STEP_US;     /* 远处封顶 */
    if (step < (int32_t)ARM_STEP_MIN_US) step = (int32_t)ARM_STEP_MIN_US; /* 不低于量化级 */

    return (uint16_t)step;
#else
    (void)diff;
    return (uint16_t)SERVO_STEP_US;
#endif
}

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
        home_pulse[i] = servo_init[i];
        target_pulse[i] = home_pulse[i];
        current_pulse[i] = home_pulse[i];
        last_sent[i] = PULSE_SENT_NONE;   /* 标记"还没下发"，让第一拍 Arm_Update 可以补发 */
    }

    /* 上电先把六路输出到安全的初始位（HOME 位）。
     * 注意：这里无法"缓启"——开环控制没有位置反馈，不知道舵机当前停在哪，
     * 写任何值对舵机来说都是突变。所以上电前必须人工把臂摆到接近「初始位」
     * （折叠姿态）；尤其 J2/J3/J4 的初始位与标定零位相差很大，摆错会猛甩。 */
    for (i = 0U; i < ARM_CHANNEL_COUNT; i++)
    {
        /* 只有写成功才记账；失败的通道 last_sent 保持 0xFFFF，
         * 下一拍 Arm_Update 会自动重试，不会因为一次 NACK 就永久停住。 */
        if (PCA9685_SetServoPulse(&hi2c1, i, current_pulse[i]) == HAL_OK)
        {
            last_sent[i] = current_pulse[i];
        }
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
        int32_t  diff = (int32_t)target_pulse[i] - (int32_t)current_pulse[i];
        uint16_t step = step_of(diff);

        /* 按算出来的步长往目标靠；最后一拍直接对齐，避免差 1μs 磨不完 */
        if (diff > (int32_t)step)
        {
            current_pulse[i] = (uint16_t)((int32_t)current_pulse[i] + (int32_t)step);
        }
        else if (diff < -(int32_t)step)
        {
            current_pulse[i] = (uint16_t)((int32_t)current_pulse[i] - (int32_t)step);
        }
        else
        {
            current_pulse[i] = target_pulse[i];
        }

        /* 值没变就不再发 I2C：PCA9685 会自己保持上一次的占空比，
         * 重复写同一个数只是白占总线时间（6 路约 4.2ms/20ms）。
         * 注意必须是"写成功"才记账 —— 否则一次 NACK 之后这个通道
         * 就再也不会重发，舵机会永远卡在旧位置。 */
        if (current_pulse[i] != last_sent[i])
        {
            if (PCA9685_SetServoPulse(&hi2c1, i, current_pulse[i]) == HAL_OK)
            {
                last_sent[i] = current_pulse[i];
            }
        }
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

