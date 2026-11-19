#include "track.h"
#include "delay.h"

// 记录最后一次执行“全黑丢线左右交替找线”的时间戳
static uint32_t Track_Process_LastT = 0;

static uint8_t search_left = 1;

/**
 * @brief  单路红外探头读取（带软件消抖）
 * @param  pin 红外探头的引脚
 * @return 读取到的稳定电平
 */
static uint8_t Track_Read_Single(uint16_t pin)
{
    uint8_t cnt = 0;
    // 1. 读取当前引脚的第一遍电平作为参考
    uint8_t ref = GPIO_ReadInputDataBit(TRACK_PORT, pin);
    uint8_t i;

    // 2. 连续快速读取 5 次
    for (i = 0; i < 5; i++)
    {
        delay_us(2); // 极短延时，滤除硬件电子噪声
        if (GPIO_ReadInputDataBit(TRACK_PORT, pin) == ref)
            cnt++;   // 如果读取结果等于参考值，计数器加 1
    }

    // 3. 多数表决法：如果 5 次中有 3 次以上与参考值相同，就认为读取有效
    // 如果读数波动严重，容错机制返回 TRACK_WHITE_LEVEL（白底）
    return cnt >= 3 ? ref : TRACK_WHITE_LEVEL;
}

/**
 * @brief 循迹红外硬件初始化
 */
void Track_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin  = TRACK_L2 | TRACK_L1 | TRACK_R1 | TRACK_R2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;    // 输入模式
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;    // 开启内部上拉（没有黑线时，引脚保持高电平）
    GPIO_Init(TRACK_PORT, &GPIO_InitStructure);
}

/**
 * @brief 循迹状态复位（模式切换时调用）
 */
void Track_Reset(void)
{
    Car_Stop();                       // 强制停车
    Car_SetSpeed(MOTOR_SPEED_SLOW);   // 初始速度降为慢速，防止冲出黑线
    Track_Process_LastT = millis();   // 重置寻找黑线的交替计时器
}

/**
 * @brief 核心循迹控制函数（状态机）
 */
void Track_Process(void)
{
    // 1. 读取 4 路红外传感器状态（0为白底，1为黑线）
    uint8_t L1 = Track_Read_Single(TRACK_L1); // 左中
    uint8_t L2 = Track_Read_Single(TRACK_L2); // 最左
    uint8_t R1 = Track_Read_Single(TRACK_R1); // 右中
    uint8_t R2 = Track_Read_Single(TRACK_R2); // 最右

    /* 
     * 2. 按位拼接成 4 位状态码
     * 对应关系：[L2][L1][R1][R2]
     * 例如：状态码 0x06（二进制 0110）表示：
     * L2=0(白), L1=1(黑), R1=1(黑), R2=0(白)
     * 黑线正好卡在中间两个探头上，路线完美居中！
     */
    uint8_t status = (L2 << 3) | (L1 << 2) | (R1 << 1) | R2;
    uint32_t t;

    // 3. 状态机执行转向
    switch (status)
    {
        // ==================== 直行状态 ====================
        // 黑线居中 0110
        case 0x06: 
        // 只要中间两个探头有任一半压线，也视为正常直行
        case 0x04: case 0x02:
            Car_SetSpeed(MOTOR_SPEED_MID);
            Car_Forward();
            break;

        // ==================== 微调纠正状态 ====================
        // 【左直角/锐角弯】1110：左、左中、右中压线，右白底 -> 必须左转弯过弯
        case 0x0E: case 0x0C:
            Car_SetSpeed(MOTOR_SPEED_SLOW);
            Car_Turn_Left();  // 左轮刹车，右轮前进，车身向左转弯
            break;

        // 【右直角/锐角弯】0111：左白底，左中、右中、右压线 -> 必须右转弯过弯
        case 0x07: case 0x03:
            Car_SetSpeed(MOTOR_SPEED_SLOW);
            Car_Turn_Right(); // 右轮刹车，左轮前进，车身向右转弯
            break;

        // ==================== 严重偏航/极限急弯自旋 ====================
        // 【黑线严重偏左】1000：左探头压线，其余都白 -> 向左原地急转弯找线
        case 0x08:
            Car_SetSpeed(MOTOR_SPEED_SLOW);
            Car_Spin_Left();       // 原地左自旋
            break;

        // 【黑线严重偏右】0001：右探头压线，其余都白 -> 向右原地急转弯找线
        case 0x01:
            Car_SetSpeed(MOTOR_SPEED_SLOW);
            Car_Spin_Right();      // 原地右自旋
            break;

        // ==================== 高级脱困 ====================
        // 您的全黑(0x00)和全白(0x0F)找线逻辑继续保持
        case 0x00:
        case 0x0F:
            Car_Stop(); 
            t = millis();
            if (t - Track_Process_LastT >= 500)
			{
				search_left = !search_left;
				Track_Process_LastT = t;
			}

			Car_SetSpeed(MOTOR_SPEED_SLOW);

			if (search_left)
				Car_Spin_Left();
			else
				Car_Spin_Right();
            break;

        default:
            Car_Stop();
            break;
    }
}
