/**
  ******************************************************************************
  * @file    acs712.h
  * @brief   ACS712-5A 霍尔电流传感器 —— 测 J6 夹爪舵机单路电流
  *
  * 接线（2026-11-16 实机确认）：
  *   ACS712 绿端子 串在 J6 舵机 V+ 红线上（PCA9685 V+ → IP+ → IP- → 舵机红线）
  *   VCC → 5V      GND → 共地      OUT → PA6（ADC1_IN6，**直连，未分压**）
  *
  * 🔴 为什么走 ADC1 注入通道而不是规则通道：
  *    规则通道的 6 路（CH0~CH5）已经被 potentiometer.c 的 DMA 循环采样占满，
  *    再往规则序列里塞一路就得改 .ioc；而注入通道 InjNumberOfConversion=0 是空的，
  *    可以在运行时 HAL_ADCEx_InjectedConfigChannel() 配出来，**一行 .ioc 都不用改**。
  *
  * 🔴 直连的电气边界（别记错，也别照抄别人的分压系数）：
  *    输出 = 2.5V + 0.185V/A × I，ADC 参考 3.3V
  *      · I ≤ 4.3A  → 输出 ≤ 3.3V，读数有效
  *      · 4.3~6A   → 输出 3.3~3.6V，ADC 读数封顶失真，引脚通常不坏（3.6V 是绝对上限）
  *      · I > 6A   → 才可能损坏
  *    J6 舵机堵转约 1.5~2.5A，正常在量程内。
  ******************************************************************************
  */
#ifndef __ACS712_H
#define __ACS712_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"        /* HAL_StatusTypeDef / HAL_OK */

/**
  * @brief 注入通道饱和阈值（mV）。
  *
  * 到 VDDA(3300mV) 就是读数封顶，再往上完全不可信。
  *
  * 🔴 这个数被两处共用：`acs712.c` 的样本过滤、`uart_protocol.c` 的 `$CUR` 提示。
  *    只在这里定义一份 —— 之前两处分别写 3300 和 3290，导致「样本被丢弃了、
  *    但提示没触发」或者反过来的不一致。改这个值只需要改这里。
  */
#define ACS712_SATURATE_MV   3290U

/**
  * @brief  配置 ADC1 注入通道（PA6 / IN6）。
  * @return HAL_OK = 配置成功；HAL_ERROR = 配置失败（此时 $CUR 会回 CUR,NA）。
  * @note   调用时机：MX_ADC1_Init() 之后、Pot_Init() **之前**。
  *         那时 ADC 还没启动，改注入序列绝对安全；放在 Pot_Init() 之后
  *         虽然一般也能配，但那时 ADC 已连续转换，没必要冒这个不确定。
  *         （注入读取在 $CUR/$CURZ 里发生，那时 ADC 已使能，能正常触发。）
  *         本函数不标零点 —— Arm_Init() 刚拉过臂，那刻电流不为零，
  *         零点要用 ACS712_CalibrateZero() 单独做。
  */
HAL_StatusTypeDef ACS712_Init(void);

/**
  * @brief  把当前电流当作零点（采 32 次取平均）。
  * @note   调用前务必确认舵机**完全静止**，否则零点会被当场电流污染。
  */
void ACS712_CalibrateZero(void);

/**
  * @brief  读一次电流，单位 mA（可为负，表示电流方向与 IP+ → IP- 相反）。
  * @note   内部采 16 次平均；饱和样本（≥3300mV）会被丢弃。
  */
int32_t ACS712_Read_mA(void);

/** @brief 最近一次采样到的 OUT 电压（mV），排查接线时看这个。 */
uint32_t ACS712_GetLast_mV(void);

/** @brief 当前使用的零点（mV），理论应为 2500。 */
uint32_t ACS712_GetZero_mV(void);

#ifdef __cplusplus
}
#endif

#endif /* __ACS712_H */
