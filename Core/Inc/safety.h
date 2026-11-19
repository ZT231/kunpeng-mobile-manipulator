#ifndef __SAFETY_H
#define __SAFETY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * 安全监控模块（阶段 1）
 *
 * 职责（大白话）：像机器人的「安全员」，定时检查有没有危险，一发现就把
 * 机械臂拉进故障态（FAULT）冻住，不让它继续动。
 *
 * 职责（规范定义）：独立的低优先级监控任务，周期性采集硬件异常信号
 * （当前只有 ACS712 过流），超过阈值并满足去抖条件时调用
 * Arm_ReportFault() 将系统置为 FAULT 锁存态。
 *
 * 🔴 为什么过流检测要放独立任务、而不是塞进 20ms 的 ServoTask：
 *   ACS712_Read_mA() 内部每点 HAL_Delay(1ms) 采 20 点 = 阻塞 20ms（为了跨
 *   一个完整 PWM 周期）。若放进 ServoTask，舵机刷新节奏会被直接拖慢一倍，
 *   运动不平滑。放到低优先级 SafetyTask 里，20ms 阻塞对舵机零影响。
 *
 * 🔴 为什么阈值定 800mA、且要「连续 N 次超阈值」才触发（双重去抖）：
 *   正常夹持的电流抬升只有 ~+140mA（11-16 实机：基线 -16 → 夹住 +140），
 *   而 J6 堵转约 1.5~2.5A。800mA 夹在两者之间——既不打断正常夹持判据，
 *   又能拦住堵转。连续 N 次是为了滤掉启动瞬间的电流尖峰，避免误触发。
 */

/* 过流阈值（mA）。夹持抬升 ~+140mA < 800 < 堵转 1.5A，夹在中间不误伤。 */
#define SAFETY_OVERCURRENT_MA   800

/* 故障码：过流（与改造方案表 7 的 L3 过流 = 0x03 一致） */
#define SAFETY_FAULT_OVERCURRENT  0x03U

/*
 * 安全监控周期调用（由 SafetyTask 每 SAFETY_TICK_MS 调一次）。
 * 内部读一次电流并判断；连续超阈值达标则触发 Arm_ReportFault。
 * 调用方须保证在任务上下文执行（本函数会阻塞约 20ms 采样，不用于中断）。
 */
void Safety_Tick(void);

#ifdef __cplusplus
}
#endif

#endif /* __SAFETY_H */
