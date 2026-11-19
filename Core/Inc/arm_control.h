#ifndef __ARM_CONTROL_H__
#define __ARM_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "potentiometer.h"
#include <stdint.h>

/* 舵机通道数（J1~J6） */
#define ARM_CHANNEL_COUNT 6U

/* 系统运行状态机 */
typedef enum
{
    ARM_STATE_MANUAL = 0,   /* 手动：电位器直接控制 */
    ARM_STATE_CMD,          /* 指令：上位机 UART 指令控制 */
    ARM_STATE_RECORD,       /* 示教记录：电位器驱动 + 定时采样 */
    ARM_STATE_PLAYBACK,     /* 示教回放：按记录轨迹复现 */
    ARM_STATE_ESTOP,        /* 急停：冻结所有输出（可恢复） */
    ARM_STATE_FAULT         /* 故障锁存：冻结输出，需显式清除（不可由 RESUME 解除） */
} Arm_State_t;

/* 初始化：计算 HOME 位、复位状态、上电先输出 HOME 位 */
void Arm_Init(void);

/* 每个 20ms 周期调用一次：平滑逼近目标并输出 PWM。急停时直接返回。 */
void Arm_Update(void);

/* 手动模式输入：把电位器 ADC 映射为目标脉宽（仅在 MANUAL 态生效） */
void Arm_SetManualTargets(const Pot_Data_t *pot);

/* 单路目标脉宽（带软限位裁剪），成功返回 1，失败（急停/通道非法）返回 0 */
uint8_t Arm_SetJointTarget(uint8_t ch, uint16_t pulse_us);

/* 六路同时设目标（带软限位裁剪） */
uint8_t Arm_SetAllTargets(const uint16_t pulse6[ARM_CHANNEL_COUNT]);

/* 命令：回 HOME / 急停 / 解除急停 / 切手动 */
void Arm_CommandHome(void);
void Arm_CommandStop(void);
void Arm_CommandResume(void);
void Arm_CommandManual(void);

/*
 * 故障锁存（新增）：
 *   Arm_ReportFault(code) —— 记录故障码并进入 FAULT 态（冻结输出）。
 *                            用于过流等「必须人工诊断后才能复位」的异常。
 *   Arm_ClearFault()      —— 从 FAULT 态恢复：清故障码、回 HOME 保持。
 *                            仅 FAULT 态有效，其余状态调用无副作用。
 *   Arm_GetFault()        —— 查询当前故障码（0 = 无故障）。
 *
 * 为什么 FAULT 和 ESTOP 要分开（大白话 + 规范）：
 *   急停像「按暂停键」，人松开就能继续；故障像「机器亮红灯」，得先查清楚
 *   为什么坏了、排除后才能按复位。两者若混成一个状态，过流这类需要
 *   人工介入的异常会被 RESUME 一句话带过去，等于没有保护。
 */
void Arm_ReportFault(uint8_t code);
void Arm_ClearFault(void);
uint8_t Arm_GetFault(void);

/* 命令：进入示教记录 / 示教回放状态（急停态下拒绝） */
void Arm_CommandRecord(void);
void Arm_CommandPlay(void);

/* 回放专用：仅更新目标脉宽（带软限位裁剪），不改变状态机状态 */
void Arm_SetPlaybackTargets(const uint16_t pulse6[ARM_CHANNEL_COUNT]);

/* 查询接口 */
Arm_State_t Arm_GetState(void);
uint16_t Arm_GetCurrent(uint8_t ch);

/*
 * 查询是否处于「冻结输出」态（急停 或 故障），1=冻结 0=正常。
 * 供协议层在回 OK/ERR,4 前判断用：冻结态下运动类命令一律拒绝，
 * 避免「回 OK 但固件内部实际没执行」的静默失效。
 */
uint8_t Arm_IsFrozen(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARM_CONTROL_H__ */
