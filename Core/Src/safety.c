/*
 * safety.c —— 安全监控：ACS712 过流检测 → FAULT 锁存
 *
 * 大白话：定时去读 J6 夹爪的电流，电流太大了（说明舵机在死命夹、要堵转烧掉），
 *         连续几次都超就立刻把机械臂冻住，报一个「过流故障」，等人来复位。
 *
 * 规范定义：周期性采样 ACS712 电流，与阈值 SAFETY_OVERCURRENT_MA 比较，
 *         用「连续超阈值计数」做软件去抖（debounce），达标后调用
 *         Arm_ReportFault(SAFETY_FAULT_OVERCURRENT) 进入 FAULT 锁存态。
 */
#include "safety.h"
#include "acs712.h"
#include "arm_control.h"
#include "cmsis_os.h"

/* arm_lock 定义于 freertos.c：机械臂状态互斥。SafetyTick 触发故障前要持锁，
 * 与 ServoTask / ProtocolTask 对 arm_state 的访问互斥。 */
extern osMutexId_t arm_lock;

/*
 * 连续超阈值计数门槛。
 * SafetyTask 每 200ms 调一次 Safety_Tick，3 次 = 600ms 内持续过流才触发。
 * 为什么不是 1 次就触发：舵机启动/换向瞬间有电流尖峰，单次采样撞上尖峰
 * 就误报「堵转」会把正常运行打断。连 3 次（约 0.6s）足以滤掉几十 ms 的尖峰，
 * 又不会漏掉真正的堵转（堵转是持续的，600ms 内必被多次采到）。
 */
#define SAFETY_OVERCURRENT_COUNT   3U

/* 过流计数：静态变量，函数每次被调累加/清零，不依赖外部状态 */
static uint8_t s_overcurrent_cnt = 0U;

void Safety_Tick(void)
{
    int32_t ma;

    /* 读一次电流（内部跨 20ms PWM 周期平均，会阻塞 ~20ms）。
     * 返回 0 且 last_mv 为 0 表示 ACS712 未初始化/读失败——此时不能当作
     * 「没电流」，否则传感器坏了系统就永远不触发过流保护，静默失效。
     * 所以读失败直接返回，不参与过流判断，也不清零计数（保持保守）。 */
    ma = ACS712_Read_mA();

    if (ACS712_GetLast_mV() == 0U)
    {
        return;   /* 传感器无效，不做判断（宁可保护失效也不误动作） */
    }

    if (ma >= (int32_t)SAFETY_OVERCURRENT_MA)
    {
        s_overcurrent_cnt++;
        if (s_overcurrent_cnt >= SAFETY_OVERCURRENT_COUNT)
        {
            /* 确认持续过流：锁存故障并冻结输出。
             * 持 arm_lock 调 Arm_ReportFault，与 ServoTask 的运动更新互斥。
             * 计数清零，避免触发后反复上报。 */
            osMutexAcquire(arm_lock, osWaitForever);
            Arm_ReportFault(SAFETY_FAULT_OVERCURRENT);
            osMutexRelease(arm_lock);
            s_overcurrent_cnt = 0U;
        }
    }
    else
    {
        /* 电流回落到阈值以下：清零计数，重新开始连续判定。
         * 这样偶发的单次尖峰不会累积成假故障。 */
        s_overcurrent_cnt = 0U;
    }
}

/************************ (C) COPYRIGHT *****END OF FILE***********************/
