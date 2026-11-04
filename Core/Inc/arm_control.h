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
    ARM_STATE_ESTOP         /* 急停：冻结所有输出 */
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

/* 命令：进入示教记录 / 示教回放状态（急停态下拒绝） */
void Arm_CommandRecord(void);
void Arm_CommandPlay(void);

/* 回放专用：仅更新目标脉宽（带软限位裁剪），不改变状态机状态 */
void Arm_SetPlaybackTargets(const uint16_t pulse6[ARM_CHANNEL_COUNT]);

/* 查询接口 */
Arm_State_t Arm_GetState(void);
uint16_t Arm_GetCurrent(uint8_t ch);

#ifdef __cplusplus
}
#endif

#endif /* __ARM_CONTROL_H__ */
