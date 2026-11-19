#ifndef __AVOIDANCE_H
#define __AVOIDANCE_H

#include "stm32f4xx.h"
#include "delay.h"
#include "motor.h"
#include "servo.h"

// 红外避障：低电平表示检测到障碍
#define IR_PORT GPIOE
#define IR_L    GPIO_Pin_0
#define IR_R    GPIO_Pin_1

// 超声波：HC-SR04，TRIG 需要 10us 以上高电平
#define TRIG_PORT GPIOB
#define TRIG_PIN  GPIO_Pin_0
#define ECHO_PORT GPIOE
#define ECHO_PIN  GPIO_Pin_6

// 距离参数，单位：cm
#define AVOID_FAR_CM       35.0f  // 小于该距离时降速
#define AVOID_NEAR_CM      25.0f  // 小于等于该距离时认为遇障
#define ULTRA_TIMEOUT_US   25000  // 超声波等待超时

// 舵机角度：90度为正前方；左右方向需按实际安装校准
#define SERVO_CENTER_ANGLE 90
#define SERVO_LEFT_ANGLE   45    // 左侧扫描角度
#define SERVO_RIGHT_ANGLE  135   // 右侧扫描角度

// 避障动作参数，单位：ms
#define AVOID_SCAN_INTERVAL_MS 80   // 正常前进时，正前方测距周期
#define SERVO_WAIT_MS          220  // 舵机转到目标角度后的稳定等待时间
#define AVOID_BACK_MS          500  // 第一次遇障后的后退时间
#define AVOID_BACK_LONG_MS     1000  // 多次受困时的加长后退时间
#define AVOID_TURN_MIN_MS      600  // 原地转向至少持续时间，防止转角不够
#define AVOID_TURN_MAX_MS      1500 // 单次转向最大时间，防止一直转圈
#define AVOID_TURN_CHECK_MS    100  // 转向过程中再次测量周期
#define AVOID_VERIFY_MS        160  // 转向完成后直行确认时间
#define AVOID_IDLE_MS          100  // 动作切换短暂停车时间
#define AVOID_MAX_RETRY        2    // 同一障碍最多改变方向的次数

typedef enum
{
    STATE_FORWARD,
    STATE_BACK,
    STATE_SCAN_LEFT,
    STATE_SCAN_RIGHT,
    STATE_TURN,
    STATE_VERIFY,
    STATE_IDLE
} CarState_t;

void Avoidance_Init(void);
void Avoidance_Run(void);
void Avoidance_Reset(void);

#endif
