/**
 * i2c_probe.c —— I2C 扫描 / 寄存器读写 / 寄存器枚举（STM32 HAL 实现）
 */
#include "i2c_probe.h"
#include <stdio.h>

/* 单次 I2C 传输超时（ms）：100kHz 下读 1 字节约 0.1ms，50ms 足够宽松 */
#define IP_TIMEOUT_MS       50U
/* 地址探测重试次数 */
#define IP_RETRY            3U
/* 地址探测单次超时（ms）：扫描用短超时，避免无设备时整体太慢 */
#define IP_SCAN_TIMEOUT_MS  10U

/**
 * @brief  扫描 7 位地址 0x08~0x77，打印所有应答地址。
 *         只做地址探测（HAL_I2C_IsDeviceReady），不读写任何数据，不会改设备状态。
 */
void I2C_Scan(I2C_HandleTypeDef *hi2c)
{
    uint8_t found = 0U;

    if (hi2c == NULL) { printf("[I2C] hi2c is NULL\r\n"); return; }

    printf("\r\n[I2C] scan 0x08~0x77 ...\r\n");

    /* 7 位地址有效范围 0x08~0x77（避开保留地址） */
    for (uint8_t addr = 0x08U; addr <= 0x77U; addr++)
    {
        /* 停在这里只发地址帧，收到 ACK 即认为设备在线 */
        if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr << 1), IP_RETRY,
                                  IP_SCAN_TIMEOUT_MS) == HAL_OK)
        {
            printf("[I2C] found 0x%02X\r\n", addr);
            found++;
        }
    }

    if (found == 0U)
        printf("[I2C] no device: check SDA/SCL/VCC/GND and pull-ups\r\n");
    else
        printf("[I2C] total %u device(s)\r\n", found);
}

/**
 * @brief  标准 8 位寄存器读（写寄存器地址 -> STOP -> 等 2ms -> 重新起始读）。
 *         注意：这里故意不用 HAL_I2C_Mem_Read —— 它发的是「重复起始」且中间没有
 *         2ms 间隔，而松甲四路电机驱动板(0x51)要求两次传输之间留 ≥2ms 给板载
 *         MCU 准备数据，否则读回全 0xFF。PCA9685(0x40)用 Mem_Read 则没问题。
 * @retval 0=成功，-1=失败/参数错
 */
int I2C_RegRead(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg,
                uint8_t *buf, uint16_t len)
{
    if (hi2c == NULL || buf == NULL || len == 0U) return -1;

    /* ① 只发寄存器地址 */
    if (HAL_I2C_Master_Transmit(hi2c, (uint16_t)(addr << 1), &reg, 1U,
                                IP_TIMEOUT_MS) != HAL_OK)
        return -1;

    /* ② 给从机准备数据的时间（松甲手册规定 ≥2ms） */
    HAL_Delay(2);

    /* ③ 读数据（最后一字节由 HAL 自动 NACK） */
    if (HAL_I2C_Master_Receive(hi2c, (uint16_t)((addr << 1) | 1U), buf, len,
                               IP_TIMEOUT_MS) != HAL_OK)
        return -1;

    return 0;
}

/**
 * @brief  标准 8 位寄存器写。
 * @retval 0=成功，-1=失败/参数错
 */
int I2C_RegWrite(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg,
                 const uint8_t *buf, uint16_t len)
{
    if (hi2c == NULL || buf == NULL || len == 0U) return -1;

    if (HAL_I2C_Mem_Write(hi2c, (uint16_t)(addr << 1), (uint16_t)reg,
                          I2C_MEMADD_SIZE_8BIT, (uint8_t *)buf, len,
                          IP_TIMEOUT_MS) != HAL_OK)
        return -1;

    return 0;
}

/**
 * @brief  枚举某地址 reg_start~reg_end 的寄存器，打印读到的值；读失败打印 <NACK>。
 *         用于反推协议表：看哪些寄存器可读、值随电机动作是否变化。
 */
void I2C_RegDump(I2C_HandleTypeDef *hi2c, uint8_t addr,
                 uint8_t reg_start, uint8_t reg_end)
{
    uint8_t v;

    printf("[I2C] dump 0x%02X reg 0x%02X~0x%02X\r\n", addr, reg_start, reg_end);

    /* 循环变量用 uint16_t：防止 reg_end=0xFF 时 uint8_t 自增回绕成死循环 */
    for (uint16_t r = reg_start; r <= (uint16_t)reg_end; r++)
    {
        if (I2C_RegRead(hi2c, addr, (uint8_t)r, &v, 1U) == 0)
            printf("  reg 0x%02X = 0x%02X (%u)\r\n", (uint8_t)r, v, v);
        else
            printf("  reg 0x%02X = <NACK>\r\n", (uint8_t)r);
    }
}
