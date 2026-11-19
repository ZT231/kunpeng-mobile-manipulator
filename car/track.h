#ifndef __TRACK_H
#define __TRACK_H

#include "stm32f4xx.h"
#include "motor.h"  // 需要用到电机差速转弯函数和速度控制
		 
// 循迹红外线引脚定义（使用 GPIOB 端口）
#define TRACK_PORT GPIOB
#define TRACK_L2   GPIO_Pin_8  // 左二（最左）红外探头
#define TRACK_L1   GPIO_Pin_9  // 左一（左中）红外探头
#define TRACK_R1   GPIO_Pin_10 // 右一（右中）红外探头
#define TRACK_R2   GPIO_Pin_11 // 右二（最右）红外探头

//硬件真实电平
//0 → 指示灯亮 → 白底（不在黑线）
//1 → 指示灯灭 → 压黑线
#define TRACK_BLACK_LEVEL 1
#define TRACK_WHITE_LEVEL 0

// 循迹状态机 功能接口
void Track_Init(void);       // 初始化循迹硬件
void Track_Process(void);    // 核心循迹逻辑（需在主循环调用）
void Track_Reset(void);      // 复位循迹状态（模式切换时调用）

#endif
