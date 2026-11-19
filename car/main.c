/*
 * ============================================================================
 * main.c —— 底盘 STM32F407ZE 主循环（统一帧版）
 * ============================================================================
 *
 * 【这一版改了什么】
 *   原项目 1.Smart car 的 main.c 是「读一个字符、switch 一个字符」
 *   （手机蓝牙发 F/B/L/R/S）。本版在**不删这套老逻辑**的前提下，
 *   加了一条新通道：以 '$' 开头的统一帧 `$CAR,FWD,800*CK\r\n`。
 *
 *   为什么要保留单字符？因为手机蓝牙 APP 已经调通了，删掉等于把能演示的
 *   东西弄丢。这叫「向后兼容」：新功能加进来，老功能一个不动。
 *
 * 【怎么区分两种协议】
 *   - 见到 '$' 就进入「帧模式」，之后的字节全部交给 car_frame.c，
 *     直到收到 '\n' 才退出帧模式；
 *   - 帧模式外的单字节，才按老的单字符命令处理。
 *   这样帧里的 'F'（$CAR,FWD...）不会被误当成「前进」，是关键的一处。
 *   另外加了 200ms 帧间超时：万一上位机只发了半帧，不会永远卡在帧模式。
 *
 * 【为什么用 millis() 而不是 delay_ms() 做定时停】
 *   delay_ms(800) 会把 CPU 死等 800ms，这期间收不到任何新命令 ——
 *   上位机想中途喊 STOP 都喊不住。改成「记一个到期时间戳，主循环每圈比一下」
 *   （规范说法：非阻塞定时 / 状态机轮询），停车、急停、自动模式都能照常跑。
 *   这也是 RTOS 里 osDelay 之外更常用的轻量做法。
 *
 * Keil MDK 默认 C89：变量一律声明在块开头，注释用 / * * /。
 */

#include "stm32f4xx.h"
#include "delay.h"
#include "motor.h"
#include "servo.h"
#include "track.h"
#include "avoidance.h"
#include "bluetooth.h"
#include "car_frame.h"

#define MODE_REMOTE 0
#define MODE_AVOID  1
#define MODE_TRACK  2

/* 帧模式最长等下一个字节的时间（毫秒） */
#define FRAME_GAP_MS 200U

static uint8_t  curr_mode = MODE_REMOTE;
static uint16_t remote_speed = MOTOR_SPEED_FAST;

static uint8_t  s_in_frame = 0;        /* 1 = 正在收一帧 */
static uint32_t s_last_byte_ms = 0;    /* 最后一个字节进来的时刻 */
static uint32_t s_stop_at_ms = 0;      /* 0 = 不自动停；否则到点就停车 */


/* ---------------- 模式切换（沿用原项目，未改逻辑） ---------------- */
static void Mode_Switch(uint8_t new_mode)
{
    if (curr_mode == new_mode) return;

    Car_ResetAll();
    Servo_Reset();
    Track_Reset();
    Avoidance_Reset();
    BT_ClearBuf();

    if (new_mode == MODE_REMOTE)
    {
        remote_speed = MOTOR_SPEED_FAST;
        Car_SetSpeed(remote_speed);
    }
    curr_mode = new_mode;
    s_stop_at_ms = 0;   /* 切模式时取消未完成的定时停，防止半路突然停车 */
}


/* ---------------- 老协议：单字符命令 ---------------- */
/* 从原 main.c 原样搬进来，只是包成了一个函数，读起来更清楚。 */
static void Legacy_OneChar(char cmd)
{
    if (cmd == '1') Mode_Switch(MODE_REMOTE);
    if (cmd == '2') Mode_Switch(MODE_AVOID);
    if (cmd == '3') Mode_Switch(MODE_TRACK);

    if (curr_mode == MODE_REMOTE)
    {
        switch (cmd)
        {
            case '4': remote_speed = MOTOR_SPEED_FAST; Car_SetSpeed(remote_speed); break;
            case '5': remote_speed = MOTOR_SPEED_MID;  Car_SetSpeed(remote_speed); break;
            case '6': remote_speed = MOTOR_SPEED_SLOW; Car_SetSpeed(remote_speed); break;

            case 'F': Car_SetSpeed(remote_speed); Car_Forward();    s_stop_at_ms = 0; break;
            case 'B': Car_SetSpeed(remote_speed); Car_Back();       s_stop_at_ms = 0; break;
            case 'L': Car_SetSpeed(remote_speed); Car_Spin_Left();  s_stop_at_ms = 0; break;
            case 'R': Car_SetSpeed(remote_speed); Car_Spin_Right(); s_stop_at_ms = 0; break;
            case 'S': Car_Stop(); s_stop_at_ms = 0; break;
            default: break;
        }
    }
}


/* ---------------- 帧层回调①：把应答发回串口 ---------------- */
/* car_frame.c 已经把 '$'、'*CK'、'\r\n' 都拼好了，这里只管原样吐出去。 */
static void Frame_Send(const char *frame_with_ck)
{
    BT_SendStr(frame_with_ck);
}


/* ---------------- 帧层回调②：真正去动电机 ---------------- */
static void Frame_OnCommand(const car_command_t *cmd)
{
    uint16_t speed;

    /* PING 的应答协议层已经回了，这里不用再动硬件 */
    if (cmd->action == CAR_ACT_PING) return;

    /* 速度：帧里给了第二参数就用它，否则沿用当前档位 */
    speed = remote_speed;
    if (cmd->has_arg2) speed = (uint16_t)cmd->arg2;

    /* 上位机要控车，就先把自动模式切回遥控（避障/循迹时不抢电机） */
    if (curr_mode != MODE_REMOTE) Mode_Switch(MODE_REMOTE);

    switch (cmd->action)
    {
    case CAR_ACT_FWD:
        Car_SetSpeed(speed); Car_Forward();
        s_stop_at_ms = (cmd->arg1 == 0U) ? 0U : millis() + cmd->arg1;
        break;

    case CAR_ACT_BWD:
        Car_SetSpeed(speed); Car_Back();
        s_stop_at_ms = (cmd->arg1 == 0U) ? 0U : millis() + cmd->arg1;
        break;

    case CAR_ACT_SPINL:
        Car_SetSpeed(speed); Car_Spin_Left();
        s_stop_at_ms = (cmd->arg1 == 0U) ? 0U : millis() + cmd->arg1;
        break;

    case CAR_ACT_SPINR:
        Car_SetSpeed(speed); Car_Spin_Right();
        s_stop_at_ms = (cmd->arg1 == 0U) ? 0U : millis() + cmd->arg1;
        break;

    case CAR_ACT_TURNL:
        Car_SetSpeed(speed); Car_Turn_Left();
        s_stop_at_ms = (cmd->arg1 == 0U) ? 0U : millis() + cmd->arg1;
        break;

    case CAR_ACT_TURNR:
        Car_SetSpeed(speed); Car_Turn_Right();
        s_stop_at_ms = (cmd->arg1 == 0U) ? 0U : millis() + cmd->arg1;
        break;

    case CAR_ACT_STOP:
        Car_Stop();
        s_stop_at_ms = 0U;
        break;

    case CAR_ACT_SPD:
        remote_speed = (uint16_t)cmd->arg1;
        Car_SetSpeed(remote_speed);
        break;

    case CAR_ACT_MODE:
        Mode_Switch((uint8_t)cmd->arg1);
        break;

    default:
        /* 理论上到不了这里：协议层已把未知命令挡在 ERR,2 */
        break;
    }
}


int main(void)
{
    char c;

    /* 初始化顺序沿用原项目，别随便调：GPIO 先于外设、串口最后开中断 */
    DWT_Init();
    Car_GPIO_Init();
    Avoidance_Init();
    Track_Init();
    BT_Init(115200);
    Car_Stop();

    /* 把两个函数「注册」给协议层。这一步就是回调/依赖注入，
       协议层从此不认识 STM32，PC 上也能编译测试。 */
    car_frame_init(Frame_OnCommand, Frame_Send);

    while (1)
    {
        c = BT_Read();

        if (c != 0)
        {
            s_last_byte_ms = millis();

            if (c == '$')
            {
                /* 帧开始：之后的字节都归协议层，老的单字符逻辑不许插手 */
                s_in_frame = 1U;
                car_frame_feed_byte((uint8_t)c);
            }
            else if (s_in_frame)
            {
                car_frame_feed_byte((uint8_t)c);
                if (c == '\n') s_in_frame = 0U;   /* 一帧收完 */
            }
            else
            {
                Legacy_OneChar(c);                /* 老蓝牙 APP 的单字符 */
            }
        }

        /* 半帧保护：200ms 没新字节就当这帧废了，别把后面的命令吞掉 */
        if (s_in_frame && (millis() - s_last_byte_ms) > FRAME_GAP_MS)
        {
            s_in_frame = 0U;
        }

        /* 定时停：每圈比一下时间戳，不阻塞、随时能被新命令打断
           （millis() 在 F407 上约 49 天才回绕，演示/验收可忽略回绕） */
        if (s_stop_at_ms != 0U && millis() >= s_stop_at_ms)
        {
            Car_Stop();
            s_stop_at_ms = 0U;
        }

        /* 自动模式照常跑 */
        switch (curr_mode)
        {
            case MODE_AVOID: Avoidance_Run(); break;
            case MODE_TRACK: Track_Process(); break;
            case MODE_REMOTE: break;
            default: Car_Stop(); break;
        }
    }
}
