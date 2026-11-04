#ifndef PCA9685_DRIVER_H
#define PCA9685_DRIVER_H
#include "main.h"
#include <stdint.h>

/* PCA9685 默认7位地址：0x40
 * HAL库发送地址时需要左移1位，所以这里写成 0x40 << 1
 */
#define PCA9685_ADDR        (0x40 << 1)

/* PCA9685寄存器 */
#define PCA9685_MODE1       0x00
#define PCA9685_MODE2       0x01
#define PCA9685_SUBADR1     0x02
#define PCA9685_SUBADR2     0x03
#define PCA9685_SUBADR3     0x04
#define PCA9685_PRESCALE    0xFE
#define PCA9685_LED0_ON_L   0x06
#define PCA9685_ALL_LED_ON_L 0xFA

/* PCA9685工作频率 */
#define PCA9685_SERVO_FREQ  50.0f

HAL_StatusTypeDef PCA9685_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef PCA9685_SetPWMFreq(I2C_HandleTypeDef *hi2c, float freq);
HAL_StatusTypeDef PCA9685_SetPWM(I2C_HandleTypeDef *hi2c, uint8_t channel, uint16_t on, uint16_t off);
HAL_StatusTypeDef PCA9685_SetServoPulse(I2C_HandleTypeDef *hi2c, uint8_t channel, uint16_t pulse_us);

#endif
