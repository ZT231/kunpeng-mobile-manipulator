#ifndef __CAR_FRAME_H
#define __CAR_FRAME_H

#include <stdint.h>

/*
 * ============================================================================
 * car_frame.h —— 底盘「统一帧」协议层（只认字节，不碰硬件）
 * ============================================================================
 *
 * 【大白话】这一层干的活：串口一个字节一个字节地送进来，它把字节攒成一句
 *   完整的话（一帧），先对一下「暗号」（XOR 校验），再把这句话翻译成
 *   「前进 800 毫秒」这样的动作，最后喊一声让应用层去执行。
 *   它完全不知道 STM32、不知道 GPIO、不知道 TIM —— 所以在电脑上用 gcc
 *   也能编译跑测试，不用接板子。
 *
 * 【规范定义】这叫「协议层 / 硬件层解耦」，靠「回调函数（函数指针）」做
 *   依赖注入（Dependency Inversion）：底层不依赖上层，上层把「怎么发应答」
 *   和「怎么执行动作」两个函数注册给底层。C 里没有类和虚函数，函数指针
 *   就是最常用的多态手段（POSIX / C99 标准，<stdint.h> 提供定宽整数）。
 *
 * ----------------------------------------------------------------------------
 * 帧格式（与机械臂固件 Core/Src/uart_protocol.c 完全一致，这才叫「统一帧」）：
 *
 *      $<CMD>[,<ARG>...]*<CK><CR><LF>
 *
 *   CK = '$' 与 '*' 之间所有字节逐字节 XOR，写成 2 位大写十六进制。
 *   例：$CAR,FWD,800*XX\r\n  → 车前进 800ms 后自动停
 *       $CAR,STOP*XX\r\n     → 立即停车
 *       $PING*XX\r\n         → 回 $OK,PONG*XX\r\n（两个节点都认）
 *
 *   应答：$OK[,<数据>]*XX\r\n   /   $ERR,<code>*XX\r\n
 *   错误码沿用机械臂那套：1=帧/校验错  2=未知命令  3=参数错  4=状态禁止
 * ----------------------------------------------------------------------------
 */

/* 一帧最多收多少字节。定死上限是安全习惯：串口线上如果有人一直灌垃圾，
   不会把内存写爆（嵌入式里没有 OS 保护，越界就是跑飞）。 */
#define CAR_FRAME_MAX_LEN   64

/* 动作量程上限：本地仲裁，防止上位机发个 FWD,999999 让车一直跑。
   对应架构文档里「LLM 只能提议、本地仲裁器才授权」的那条原则。 */
#define CAR_MAX_RUN_MS      5000
#define CAR_MAX_PWM         999

/* 动作枚举：用 enum 而不是裸数字 0/1/2，读代码的人不用猜「3 是啥」。
   （enum 是 C89 就有的基础类型，<stdint.h> 的 uint32_t 是 C99。） */
typedef enum {
    CAR_ACT_FWD = 0,    /* 前进 */
    CAR_ACT_BWD,        /* 后退 */
    CAR_ACT_SPINL,      /* 原地左转 */
    CAR_ACT_SPINR,      /* 原地右转 */
    CAR_ACT_TURNL,      /* 前进中左转 */
    CAR_ACT_TURNR,      /* 前进中右转 */
    CAR_ACT_STOP,       /* 停车 */
    CAR_ACT_SPD,        /* 设置速度档（PWM 0~999） */
    CAR_ACT_MODE,       /* 切模式：0遥控 / 1避障 / 2循迹 */
    CAR_ACT_PING,       /* 探活 */
    CAR_ACT_UNKNOWN
} car_action_t;

/* 解析完一帧得到的结构体。用结构体把「一组相关参数」打包传参，
   比一长串函数参数好读，也不容易把顺序写反。 */
typedef struct {
    car_action_t action;
    uint32_t     arg1;      /* 主参数：时长 ms / PWM 值 / 模式号 */
    uint32_t     arg2;      /* 可选参数：速度 PWM（只对运动命令有效） */
    uint8_t      has_arg2;  /* 0=没给 arg2，用当前速度档 */
} car_command_t;

/* 回调①：解析出合法命令后调用，由应用层去真正动电机 */
typedef void (*car_on_command_fn)(const car_command_t *cmd);

/* 回调②：把一整帧应答（已含 *CK\r\n）原样发出去 */
typedef void (*car_send_fn)(const char *frame_with_ck);

/* 注册两个回调，必须在喂字节之前调一次 */
void car_frame_init(car_on_command_fn on_cmd, car_send_fn send);

/* 每收到一个字节就喂一次。可以来自串口中断，也可以来自主循环读环形缓冲。
   收满一帧会当场校验 + 分发（回调在调用者上下文执行，注意别在中断里做重活）。 */
void car_frame_feed_byte(uint8_t byte);

/* 只有测试/排障才用：手动让协议层产生一帧应答，检查串口通不通 */
void car_frame_test_echo(const char *payload);

#endif
