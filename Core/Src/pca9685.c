#include "pca9685.h"

/*
 * PCA9685内部振荡器频率，默认约25MHz
 */
#define PCA9685_OSC_CLOCK 25000000.0f

/*
 * 向PCA9685写一个寄存器
 */
static HAL_StatusTypeDef PCA9685_WriteReg(I2C_HandleTypeDef *hi2c,
                                          uint8_t reg,
                                          uint8_t data)
{
    return HAL_I2C_Mem_Write(
        hi2c,
        PCA9685_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &data,
        1,
        100
    );
}

/*
 * 从PCA9685读取一个寄存器
 */
static HAL_StatusTypeDef PCA9685_ReadReg(I2C_HandleTypeDef *hi2c,
                                         uint8_t reg,
                                         uint8_t *data)
{
    return HAL_I2C_Mem_Read(
        hi2c,
        PCA9685_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        1,
        100
    );
}

static void PCA9685_AllOff(I2C_HandleTypeDef *hi2c)
{
    uint8_t data[4];
    /*
     * ALL_LED 寄存器：一次设置全部 16 通道。
     * data[3] = 0x10 → 所有通道 OFF_H 置 FULL_OFF，输出高阻，清除任何异常 PWM。
     * 复位瞬间 I2C 毛刺可能把通道寄存器写脏，初始化时先整体释放，避免启动期舵机乱摆。
     */
    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x10;
    (void)HAL_I2C_Mem_Write(hi2c, PCA9685_ADDR, PCA9685_ALL_LED_ON_L,
                            I2C_MEMADD_SIZE_8BIT, data, 4, 100);
}

/*
 * 初始化PCA9685
 */
HAL_StatusTypeDef PCA9685_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;
    uint8_t mode1;
    /*
     * 读取MODE1，确认设备能够通信
     */
    status = PCA9685_ReadReg(hi2c, PCA9685_MODE1, &mode1);
    if (status != HAL_OK)
    {
        return status;
    }
    /*
     * MODE1 = 0x00：
     * 正常模式，使用内部时钟
     */
    status = PCA9685_WriteReg(hi2c, PCA9685_MODE1, 0x00);
    if (status != HAL_OK)
    {
        return status;
    }
    /*
     * MODE2 = 0x04：
     * 输出推挽模式，适合大多数模块
     */
    status = PCA9685_WriteReg(hi2c, PCA9685_MODE2, 0x04);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(10);
    /*
     * 设置舵机常用的50Hz
     */
    status = PCA9685_SetPWMFreq(hi2c, PCA9685_SERVO_FREQ);
    if (status != HAL_OK)
    {
        return status;
    }
    /*
     * 清空全部通道输出，抹掉复位瞬间 I2C 毛刺可能写入的脏 PWM，
     * 让后续 Arm_Init() 从干净状态把六路设到 HOME。
     */
    PCA9685_AllOff(hi2c);
    return HAL_OK;
}

/*
 * 设置PCA9685 PWM频率
 */
HAL_StatusTypeDef PCA9685_SetPWMFreq(I2C_HandleTypeDef *hi2c,
                                     float freq)
{
    uint8_t oldmode;
    uint8_t newmode;
    uint8_t prescale;
    /*
     * PCA9685预分频计算公式：
     *
     * prescale = round(25MHz / (4096 * freq)) - 1
     *
     * 对于50Hz，结果约为121
     */
    prescale = (uint8_t)(
        (PCA9685_OSC_CLOCK / (4096.0f * freq)) + 0.5f
    ) - 1;
    /*
     * 读取当前MODE1
     */
    if (PCA9685_ReadReg(hi2c, PCA9685_MODE1, &oldmode) != HAL_OK)
    {
        return HAL_ERROR;
    }
    /*
     * 进入睡眠模式，才能修改PRE_SCALE
     */
    newmode = (oldmode & 0x7F) | 0x10;
    if (PCA9685_WriteReg(hi2c, PCA9685_MODE1, newmode) != HAL_OK)
    {
        return HAL_ERROR;
    }
    /*
     * 设置预分频
     */
    if (PCA9685_WriteReg(hi2c,
                         PCA9685_PRESCALE,
                         prescale) != HAL_OK)
    {
        return HAL_ERROR;
    }
    /*
     * 恢复正常模式
     */
    if (PCA9685_WriteReg(hi2c, PCA9685_MODE1, oldmode) != HAL_OK)
    {
        return HAL_ERROR;
    }
    HAL_Delay(5);
    /*
     * 设置RESTART位
     */
    newmode = oldmode | 0xA1;
    if (PCA9685_WriteReg(hi2c, PCA9685_MODE1, newmode) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

/*
 * 设置某个通道的PWM
 *
 * channel：0~15
 * on：PWM开始计数值
 * off：PWM结束计数值
 */
HAL_StatusTypeDef PCA9685_SetPWM(I2C_HandleTypeDef *hi2c,
                                 uint8_t channel,
                                 uint16_t on,
                                 uint16_t off)
{
    uint8_t data[4];
    uint8_t reg;
    if (channel > 15)
    {
        return HAL_ERROR;
    }
    if (on > 4095)
    {
        on = 4095;
    }
    if (off > 4095)
    {
        off = 4095;
    }
    reg = PCA9685_LED0_ON_L + 4 * channel;
    data[0] = on & 0xFF;
    data[1] = (on >> 8) & 0x0F;
    data[2] = off & 0xFF;
    data[3] = (off >> 8) & 0x0F;
    return HAL_I2C_Mem_Write(
        hi2c,
        PCA9685_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        4,
        100
    );
}

/*
 * 根据脉宽设置舵机位置
 *
 * 50Hz下，一个周期是20000us
 * PCA9685一个周期有4096个计数
 *
 * count = pulse_us * 4096 / 20000
 */
HAL_StatusTypeDef PCA9685_SetServoPulse(I2C_HandleTypeDef *hi2c,
                                        uint8_t channel,
                                        uint16_t pulse_us)
{
    uint16_t pulse_count;
    if (pulse_us < 500)
    {
        pulse_us = 500;
    }
    if (pulse_us > 2500)
    {
        pulse_us = 2500;
    }
    pulse_count = (uint16_t)(
        ((uint32_t)pulse_us * 4096UL) / 20000UL
    );
    return PCA9685_SetPWM(
        hi2c,
        channel,
        0,
        pulse_count
    );
}
