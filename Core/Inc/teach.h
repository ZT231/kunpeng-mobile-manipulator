#ifndef __TEACH_H__
#define __TEACH_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "arm_control.h"
#include <stdint.h>

/*
 * 示教回放模块（S2）
 *
 * 记录：手动（电位器）移动机械臂时，以固定间隔采样六路当前脉宽存入 RAM；
 * 回放：按相同间隔逐点取出，交给 Arm 平滑执行，复现手动轨迹。
 *
 * 与状态机的对应：
 *   记录 -> ARM_STATE_RECORD（电位器仍被当作手动输入驱动）
 *   回放 -> ARM_STATE_PLAYBACK（目标由本模块提供）
 */

#define TEACH_MAX_POINTS       600U   /* 最多保存的示教点数量 */
#define TEACH_TICK_MS           20U   /* Teach_Tick 调用周期，与主循环舵机周期一致 */
#define TEACH_SAMPLE_MS        100U   /* 示教采样/回放间隔 */
#define TEACH_TICKS_PER_SAMPLE  (TEACH_SAMPLE_MS / TEACH_TICK_MS)  /* = 5 */

/* 初始化：清空缓冲与所有标志 */
void Teach_Init(void);

/* 开始记录：清空旧轨迹并进入 RECORD 态 */
void Teach_StartRecord(void);
/* 结束记录：回到 MANUAL，保留本次已记录的点 */
void Teach_StopRecord(void);

/* 开始回放：从头播放已记录轨迹，进入 PLAYBACK 态（无点时直接返回） */
void Teach_StartPlayback(void);
/* 清空已记录轨迹（需在非记录/回放态调用） */
void Teach_Clear(void);

/* 每个 20ms 周期调用一次：推进记录采样或回放取点 */
void Teach_Tick(void);

/* 查询已记录点数 */
uint16_t Teach_GetPointCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __TEACH_H__ */
