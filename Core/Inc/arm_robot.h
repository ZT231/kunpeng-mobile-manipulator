/**
 * arm_robot.h — 机械臂控制层（六轴总线舵机，挂在 USART3 / PB10+PB11）
 *
 * 职责边界：
 *   · servo_bus.c  只做「帧 + 舵机个体」：组帧、校验、位置/温度/电压回读、软限位、背隙补偿。
 *   · 本模块        做「整臂」：六轴零位、作业邮箱、10ms 节拍轮询、$ARM 命令解析。
 *   · uart_protocol.c 只做「收帧/校验/分发/回帧」，不碰舵机细节。
 *
 * 为什么要作业邮箱：SB_MovePos 对 J5 会做「越过 24 单位 → 停顿 150ms → 反向逼近」的
 * 阻塞两段式（J5 背隙 3.6°），单次可能占住调用者 1 秒以上。若直接在协议任务里执行，
 * 协议的 5ms 循环会被卡住，底盘 500ms 心跳（$CAR,VEL 看门狗）会误判超时自动刹停。
 * 因此协议任务只「投递」作业、立刻回 OK；真正的舵机 I/O 全部由 ArmTask 执行。
 * 同一时刻只受理一条运动：`ArmTask` 正在下发 6 帧那一刻再发动作回 $ERR,4（状态禁止）。
 * 🔴 但置忙窗口就是「下发这几毫秒」，发完即释放；此后舵机仍在按 ms 爬坡，再发 POS/J
 * 会被接受并覆盖在途斜坡（舵机从当前位置重新规划，不跳变）。要中止在途运动用 $ARM,STOP。
 */
#ifndef __ARM_ROBOT_H__
#define __ARM_ROBOT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "servo_bus.h"

/* ARM_Exec 返回值：0=已受理/成功；非 0 直接作为 $ERR,<code> 的 code 回给上位机
   （复用 uart_protocol.c 的统一错误码：3=参数错误 4=状态禁止 5=设备故障） */
#define ARM_OK            0
#define ARM_ERR_UNKNOWN   2     /* 未知子命令 */
#define ARM_ERR_PARAM     3
#define ARM_ERR_BUSY      4
#define ARM_ERR_FAULT     5

/* 初始化（main 里、调度器启动前调用一次）：绑定 huart3、装载零位、清邮箱 */
void ARM_Init(void);

/* ArmTask 每 10ms 调用一次：先执行邮箱里的作业，再做一轮温度/位置/电压轮询 */
void ARM_TaskTick(void);

/* 解析并执行 $ARM 之后的参数串；成功时把回帧 payload 写进 out（如 "OK,ARM,POS,800"） */
int ARM_Exec(const char *arg, char *out, uint16_t out_max);

#ifdef __cplusplus
}
#endif
#endif /* __ARM_ROBOT_H__ */
