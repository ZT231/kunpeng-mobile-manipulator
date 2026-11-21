/**
 * i2c_probe.h —— I2C 总线扫描 + 寄存器读写/枚举工具
 *
 * 场景：四路编码器电机驱动板（板载 MCU、I2C 从机）协议未知时，
 *       先用本工具把地址扫出来、把寄存器空间枚举一遍，再对照商家协议表。
 *
 * 用法（在 main.c 的 USER CODE 2 段、所有 MX_xxx_Init 之后、osKernelStart 之前）：
 *     #include "i2c_probe.h"
 *     I2C_Scan(&hi2c1);                        // 先扫出在线地址
 *     I2C_RegDump(&hi2c1, 0x24, 0x00, 0xFF);   // 枚举某地址寄存器（0x24 为示例，换扫描结果）
 *
 * 注意：
 *   - 本工具按"8 位寄存器地址 + 数据"的标准 I2C 寄存器模型读写；若商家协议是
 *     16 位地址或"命令字"格式，枚举值会异常，以商家资料为准。
 *   - 与 PCA9685 共用 I2C1(PB6/PB7) 时，地址不冲突即可（PCA9685 默认 0x40）。
 *   - 总线需上拉（板载或外部 4.7k 到 VCC）；驱动板 VCC 用 3.3V 还是 5V 以商家为准。
 *   - 打印走 printf(huart1)，若在任务里调用需自行持 print_lock。
 */
#ifndef __I2C_PROBE_H
#define __I2C_PROBE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* 扫描 7 位地址 0x08~0x77，打印所有应答的地址 */
void I2C_Scan(I2C_HandleTypeDef *hi2c);

/* 标准 8 位寄存器读：addr=7 位从机地址，reg=寄存器地址，len=读取字节数；返回 0=成功 */
int  I2C_RegRead(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg,
                 uint8_t *buf, uint16_t len);

/* 标准 8 位寄存器写：addr=7 位从机地址，reg=寄存器地址，len=写入字节数；返回 0=成功 */
int  I2C_RegWrite(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg,
                  const uint8_t *buf, uint16_t len);

/* 枚举某地址 reg_start~reg_end 的寄存器并打印（读失败的打印 <NACK>） */
void I2C_RegDump(I2C_HandleTypeDef *hi2c, uint8_t addr,
                 uint8_t reg_start, uint8_t reg_end);

#ifdef __cplusplus
}
#endif
#endif /* __I2C_PROBE_H */
