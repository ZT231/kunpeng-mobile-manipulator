#include "uart_protocol.h"
#include "chassis.h"
#include "arm_robot.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>

/* print_lock 定义于 freertos.c：任务层共享，保护 huart1 上的 printf/响应发送互斥，
 * 避免 PrintTask 周期打印与 ProtocolTask 发送响应帧按字符交错。 */
extern osMutexId_t print_lock;

/*
 * UART 指令协议（v1）
 *
 * 帧格式：  $<CMD>[,<ARG>...]*<CK><CR><LF>
 *   CK 为 CMD 字段（$ 与 * 之间）所有字节的 XOR，2 位大写十六进制。
 *
 * 请求示例：
 *   $PING*XX\r\n
 *   $CAR,VEL,0.2,0,0*XX\r\n          (底盘前进 0.2m/s，500ms 心跳式)
 *   $CAR,VEL,0.2,0,0,800*XX\r\n      (前进 0.2m/s，800ms 后自动停)
 *   $CAR,STOP*XX\r\n
 *   $CAR,ODOM*XX\r\n                 (回 $ODOM,x_mm,y_mm,yaw_deg,online*XX)
 *   $CAR,RESET*XX\r\n
 *   $CAR,RAW,10,0,0,0*XX\r\n     (开环 10% 只转 A 轮，台架逐轮核对转向/接线；500ms 自动收油)
 *   $CAR,WHEEL*XX\r\n                (回 $WHEEL,tA,sA,dA,...,tD,sD,dD*XX，单位 mm/s、‰)
 *   $CAR,ENC*XX\r\n                  (回 $ENC,a,b,c,d*XX，原始累计计数，标定 3960 用)
 *   $CAR,GAIN,700,900,0*XX\r\n       (在线改 KP/KI/KD，不写 Flash)
 *   $CAR,CLRFAULT*XX\r\n             (清堵转故障并重新使能 STBY)
 *   $ARM,POS,460,514,516,502,494,674*XX\r\n   (六轴同步到零位，位置单位 0~1000)
 *   $ARM,J,5,500,800*XX\r\n          (只动 J5 到 500，800ms；J5 自动走背隙补偿)
 *   $ARM,HOME*XX\r\n                 (六轴回零位)
 *   $ARM,READ*XX\r\n                 (回 $OK,POS,p1..p6)
 *   $ARM,STOP*XX\r\n                 (六轴原地停，撤销未执行作业)
 * 响应格式：
 *   $OK[,<数据>]*XX\r\n
 *   $ERR,<code>*XX\r\n
 *     错误码（本项目统一）：
 *       1=帧/校验错误  2=未知命令  3=参数错误  4=状态禁止  5=设备故障(FAULT)
 */

#define PROTO_BUF_SIZE   64U
#define PROTO_TX_SIZE    96U
#define PROTO_UART_MAX   2U

static const char HEX_DIGITS[] = "0123456789ABCDEF";

/* 多控制源：最多 2 个串口（USART1=PC/OrangePi，USART2=蓝牙）。
   USART3 自 2026-11-26 起改为机械臂总线舵机专用（servo_bus.c 阻塞收发），
   不注册进本协议栈，也不走中断。 */
static UART_HandleTypeDef *proto_huarts[PROTO_UART_MAX];
static uint8_t proto_uart_count = 0U;
/* 响应发回目标：由 frame_hold_src 在解析前设置（谁命令谁收响应） */
static UART_HandleTypeDef *proto_tx_huart = NULL;

/* 周期遥测打印开关（默认关闭） */
static uint8_t stream_log_enabled = 0U;

/* 单字节中断接收缓冲（每个串口独立，避免互踩） */
static uint8_t rx_byte[PROTO_UART_MAX];
/* 若 TX 占用 HAL Lock 导致接收重启失败，置位后由主循环重试 */
static volatile uint8_t rx_restart_pending[PROTO_UART_MAX];

/* 待发响应帧缓冲与标志。
 * proto_send_frame 仅由 ProtocolTask 命令解析调用，只负责"入队"（写缓冲+置位），
 * 真正发送放在 UartProto_Process() 内，避免阻塞 IO。
 * 双缓冲：tx_buf 为活动槽（发送中），tx_buf2 为备用槽（tx_busy 时写入）。
 * 发完活动槽后自动搬运备用槽，确保连发命令的每条应答都不丢失。 */
static uint8_t tx_buf[PROTO_TX_SIZE];
static uint16_t tx_len = 0U;
static volatile uint8_t tx_pending = 0U;
static volatile uint8_t tx_busy = 0U;
static uint8_t tx_buf2[PROTO_TX_SIZE];
static uint16_t tx_len2 = 0U;
static volatile uint8_t tx_pending2 = 0U;

/* 帧接收缓冲区与状态。
 * ISR 只负责收字节成帧；收满后拷入 frame_hold 并置 frame_ready，
 * 由 ProtocolTask(UartProto_Process) 取走并解析分发——命令处理不再在中断上下文执行。 */
static uint8_t frame_buf[PROTO_BUF_SIZE];
static uint8_t frame_len = 0U;
static uint8_t frame_active = 0U;   /* 0=等待'$'，1=收集中 */
static uint8_t frame_hold[PROTO_BUF_SIZE];     /* 完整帧保留缓冲（任务解析用） */
static uint16_t frame_hold_len = 0U;
static volatile uint8_t frame_ready = 0U;      /* 1=一帧已就绪待任务解析 */

/* 帧来源串口：收帧时随帧记录，任务解析时再切 proto_tx_huart。
   不能只用「最后收到 '$' 的串口」——PC 与蓝牙前后脚发命令时，
   上一条应答会被发到后一个口。 */
static UART_HandleTypeDef *frame_src = NULL;
static UART_HandleTypeDef *volatile frame_hold_src = NULL;

/* ---------- 发送辅助 ---------- */

/* 发送一帧（入队）：payload 为 '$' 与 '*ck' 之间的内容（如 "OK,PONG"）。
 * 由 ProtocolTask 调用（命令解析已移入任务，不再从 ISR 调用）：
 * 不阻塞、不发送，只写入缓冲交给 UartProto_Process 发送。
 * 双缓冲策略：tx_busy 时写入备用槽（不丢弃），主循环发完活动槽后自动搬运。 */
static void proto_send_frame(const char *payload)
{
    uint8_t ck = 0U;
    uint16_t len;
    uint16_t i;
    uint8_t *buf;
    uint16_t *plen;

    if (payload == NULL) return;

    len = (uint16_t)strlen(payload);
    if ((uint16_t)(len + 6U) >= PROTO_TX_SIZE) return;

    if (tx_busy != 0U)
    {
        if (tx_pending2 != 0U) return;   /* 两槽都满，丢弃（演示不会触发） */
        buf = tx_buf2;
        plen = &tx_len2;
    }
    else
    {
        buf = tx_buf;
        plen = &tx_len;
    }

    for (i = 0U; i < len; i++) ck ^= (uint8_t)payload[i];

    buf[0] = '$';
    for (i = 0U; i < len; i++) buf[1U + i] = (uint8_t)payload[i];
    buf[1U + len] = '*';
    buf[2U + len] = HEX_DIGITS[(ck >> 4) & 0x0FU];
    buf[3U + len] = HEX_DIGITS[ck & 0x0FU];
    buf[4U + len] = '\r';
    buf[5U + len] = '\n';

    *plen = (uint16_t)(len + 6U);
    if (tx_busy != 0U) tx_pending2 = 1U;
    else tx_pending = 1U;
}

/* ---------- 帧完成保留（ISR → 任务交接） ---------- */

/* ISR 收完一帧后调用：拷贝完整帧到 frame_hold 并置 frame_ready。
 * 若任务尚未取走上一帧（frame_ready 仍为 1），丢弃新帧（连发场景演示不触发）。 */
static void frame_complete(void)
{
    uint16_t i;

    if (frame_ready != 0U) return;

    for (i = 0U; i <= (uint16_t)frame_len; i++) frame_hold[i] = frame_buf[i];
    frame_hold_len = frame_len;
    frame_hold_src = frame_src;     /* 随帧保留来源，任务解析时再切应答目标 */
    frame_ready = 1U;
}

/* ---------- 解析辅助 ---------- */

static uint8_t hex_to_nib(uint8_t c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    return 0U;
}

static uint8_t is_hex(uint8_t c)
{
    return ((c >= '0' && c <= '9') ||
            (c >= 'A' && c <= 'F') ||
            (c >= 'a' && c <= 'f')) ? 1U : 0U;
}

/* 从 pp 指向的位置解析一个十进制无符号整数，跳过前导空格/逗号。
 * 成功返回 1，并把 pp 推进到该数字之后；失败返回 0。 */
static int parse_uint(const char **pp, uint32_t *val)
{
    const char *p = *pp;
    uint32_t v = 0U;
    int any = 0;

    while (*p == ' ' || *p == ',') p++;

    if (*p < '0' || *p > '9') return 0;

    while (*p >= '0' && *p <= '9')
    {
        uint32_t digit = (uint32_t)(*p - '0');
        /* 溢出检查：先判再乘，避免 999999999999 回绕成一个小数后通过量程。
           UINT32_MAX 的十进制是 4294967295，这里用等价的安全条件。 */
        if (v > ((0xFFFFFFFFU - digit) / 10U)) return 0;
        v = v * 10U + digit;
        p++;
        any = 1;
    }

    *val = v;
    *pp = p;
    return any;
}

/* 从 pp 指向的位置解析一个十进制浮点数（可选符号 + 可选小数点，
 * 不支持科学计数法/十六进制）。成功返回 1 并把 pp 推进到数字之后；失败返回 0。
 * 用于底盘 $CAR,VEL 的 vx/vy/wz 参数（m/s、rad/s）。 */
static int parse_float(const char **pp, float *val)
{
    const char *p = *pp;
    float sign = 1.0f;
    float v = 0.0f;      /* 整数部分 */
    float frac = 0.0f;   /* 小数部分累加 */
    float scale = 0.1f;  /* 小数位权：0.1 / 0.01 / 0.001 ... */
    int any = 0;

    while (*p == ' ' || *p == ',') p++;

    if (*p == '-') { sign = -1.0f; p++; }
    else if (*p == '+') { p++; }

    while (*p >= '0' && *p <= '9')
    {
        v = v * 10.0f + (float)(*p - '0');
        p++;
        any = 1;
    }

    if (*p == '.')
    {
        p++;
        while (*p >= '0' && *p <= '9')
        {
            frac += (float)(*p - '0') * scale;
            scale *= 0.1f;
            p++;
            any = 1;
        }
    }

    if (!any) return 0;

    *val = sign * (v + frac);
    *pp = p;
    return 1;
}

/* ---------- 命令处理 ---------- */

static void cmd_ping(void)
{
    proto_send_frame("OK,PONG");
}

static void cmd_log(const char *arg)
{
    uint32_t v = 0U;

    if (!parse_uint(&arg, &v) || v > 1U)
    {
        proto_send_frame("ERR,3");
        return;
    }

    UartProto_SetStreamLog((uint8_t)v);
    proto_send_frame("OK");
}

/* ==================== 底盘命令 $CAR,... ==================== */

/**
  * @brief  $CAR,VEL,<vx>,<vy>,<wz>[,<ms>] —— 底盘速度（麦轮逆解）。
  *         vx/vy 单位 m/s（+前 / +右）、wz 单位 rad/s（+逆时针俯视）。
  *         给 ms 且 >0：跑 ms 后自动停；不给 ms 或 ms=0：持续跑，
  *         固件 500ms 收不到新 VEL 会自动刹停（心跳式安全看门狗）。
  *         本地仲裁量程：|vx|,|vy| ≤ 1.0m/s、|wz| ≤ 5.0rad/s、ms ≤ 60000。
  */
static void cmd_car_vel(const char *arg)
{
    float vx, vy, wz;
    uint32_t ms = 0U;
    int has_ms;

    if (!parse_float(&arg, &vx)) { proto_send_frame("ERR,3"); return; }
    if (!parse_float(&arg, &vy)) { proto_send_frame("ERR,3"); return; }
    if (!parse_float(&arg, &wz)) { proto_send_frame("ERR,3"); return; }

    if (vx < -1.0f || vx > 1.0f ||
        vy < -1.0f || vy > 1.0f ||
        wz < -5.0f || wz > 5.0f)
    {
        proto_send_frame("ERR,3");
        return;
    }

    has_ms = parse_uint(&arg, &ms);
    if (has_ms && ms > 60000U) { proto_send_frame("ERR,3"); return; }
    if (!(vx == vx) || !(vy == vy) || !(wz == wz)) { proto_send_frame("ERR,3"); return; }

    /* 底盘未初始化/已锁堵转故障：如实回 ERR,5，绝不用 OK 装成功。 */
    if (!g_chassis.online) { proto_send_frame("ERR,5"); return; }

    if (has_ms && ms > 0U)
    {
        if (CHS_SetVelocityMs(&g_chassis, vx, vy, wz, ms) != CHS_OK)
        {
            proto_send_frame("ERR,5");
            return;
        }
    }
    else
    {
        if (CHS_SetVelocity(&g_chassis, vx, vy, wz) != CHS_OK)
        {
            proto_send_frame("ERR,5");
            return;
        }
    }

    proto_send_frame("OK");
}

/**
  * @brief  $CAR,STOP —— 立即刹停底盘。
  */
static void cmd_car_stop(void)
{
    (void)CHS_Stop(&g_chassis);
    proto_send_frame("OK");
}

/**
  * @brief  $CAR,ODOM —— 回读里程计与在线状态。
  * @note   回 $ODOM,<x_mm>,<y_mm>,<yaw_deg>,<online>*CK（编码器积分值，取整）。
  */
static void cmd_car_odom(void)
{
    char buf[64];
    float x, y, yaw;

    CHS_GetOdometry(&g_chassis, &x, &y, &yaw);
    (void)snprintf(buf, sizeof(buf), "ODOM,%ld,%ld,%ld,%u",
                   (long)x, (long)y, (long)yaw, (unsigned)g_chassis.online);
    proto_send_frame(buf);
}

/**
  * @brief  $CAR,RESET —— 里程计清零（x/y/yaw 归 0，不清编码器累计计数）。
  */
static void cmd_car_reset(void)
{
    CHS_ResetOdometry(&g_chassis);
    proto_send_frame("OK");
}

/**
  * @brief  $CAR,RAW,<a>,<b>,<c>,<d> —— 开环占空比直驱（台架接线/转向核对）。
  *         范围 ±30（单位 %），负值反转；对应 A 左前/B 右前/C 左后/D 右后。
  *         注意：RAW 是「原始」占空比，不乘 WDIR，方便逐轮看电机真实转向。
  *         500ms 内没有新的 RAW 会自动收油（与 VEL 同一个看门狗）。
  */
static void cmd_car_raw(const char *arg)
{
    float a, b, c, d;

    if (!parse_float(&arg, &a) || !parse_float(&arg, &b) ||
        !parse_float(&arg, &c) || !parse_float(&arg, &d))
    {
        proto_send_frame("ERR,3");
        return;
    }

    if (!(a == a) || !(b == b) || !(c == c) || !(d == d))
    {
        proto_send_frame("ERR,3");
        return;
    }

    if (a < -30.0f || a > 30.0f || b < -30.0f || b > 30.0f ||
        c < -30.0f || c > 30.0f || d < -30.0f || d > 30.0f)
    {
        proto_send_frame("ERR,3");
        return;
    }

    if (!g_chassis.online) { proto_send_frame("ERR,5"); return; }

    if (CHS_SetPwmRaw(&g_chassis,
                      (int8_t)a, (int8_t)b, (int8_t)c, (int8_t)d) != CHS_OK)
    {
        proto_send_frame("ERR,5");
        return;
    }

    proto_send_frame("OK");
}

/**
  * @brief  $CAR,WHEEL —— 四轮遥测：目标/实测（mm/s）+ 最近下发占空比（‰）。
  *         回 $WHEEL,tA,sA,dA,tB,sB,dB,tC,sC,dC,tD,sD,dD*CK（共 12 个整数）。
  *         调 PID 时用它看「目标有没有跟上、哪一轮拖后腿」。
  */
static void cmd_car_wheel(void)
{
    char    buf[96];
    int     n = 0;
    uint8_t i;

    n = snprintf(buf, sizeof(buf), "WHEEL");
    if (n < 0) { proto_send_frame("ERR,5"); return; }

    for (i = 0U; i < CHS_W_NUM; i++)
    {
        float   tgt = 0.0f;
        float   spd = 0.0f;
        int16_t duty = 0;
        int     w;

        CHS_GetWheelState(&g_chassis, i, &tgt, &spd, &duty);

        /* 不用 %f（MicroLIB 浮点 printf 不可用），统一放大成 mm/s 整数 */
        w = snprintf(buf + n, sizeof(buf) - (size_t)n,
                     ",%d,%d,%d",
                     (int)(tgt * 1000.0f), (int)(spd * 1000.0f), (int)duty);
        if (w < 0) { proto_send_frame("ERR,5"); return; }
        n += w;
    }

    proto_send_frame(buf);
}

/**
  * @brief  $CAR,ENC —— 四路编码器原始累计计数（上电以来，未乘方向宏）。
  *         回 $ENC,a,b,c,d*CK。台架标定用：手转某轮正好一圈，看该路是否
  *         增加 3960（11 线 × 90 减速比 × 4 倍频）；对不上就改 CHS_ENC_LINES。
  */
static void cmd_car_enc(void)
{
    char buf[64];

    (void)snprintf(buf, sizeof(buf), "ENC,%ld,%ld,%ld,%ld",
                   (long)g_chassis.w[CHS_W_A].enc_total,
                   (long)g_chassis.w[CHS_W_B].enc_total,
                   (long)g_chassis.w[CHS_W_C].enc_total,
                   (long)g_chassis.w[CHS_W_D].enc_total);
    proto_send_frame(buf);
}

/**
  * @brief  $CAR,GAIN,<kp>,<ki>,<kd> —— 在线改速度环增益（不写 Flash，重启恢复宏）。
  *         调参顺序：KP/KI 先 0 只留前馈 → 加 KP 到不抖 → 加 KI 消静差。
  */
static void cmd_car_gain(const char *arg)
{
    float kp, ki, kd;

    if (!parse_float(&arg, &kp) || !parse_float(&arg, &ki) || !parse_float(&arg, &kd))
    {
        proto_send_frame("ERR,3");
        return;
    }
    if (!(kp == kp) || !(ki == ki) || !(kd == kd)) { proto_send_frame("ERR,3"); return; }

    if (kp < 0.0f || kp > 5000.0f ||
        ki < 0.0f || ki > 5000.0f ||
        kd < 0.0f || kd > 2000.0f)
    {
        proto_send_frame("ERR,3");
        return;
    }

    if (!g_chassis.online) { proto_send_frame("ERR,5"); return; }

    if (CHS_SetGains(&g_chassis, kp, ki, kd) != CHS_OK)
    {
        proto_send_frame("ERR,3");
        return;
    }

    proto_send_frame("OK");
}

/**
  * @brief  $CAR,CLRFAULT —— 清锁存堵转故障并重新使能 STBY。
  *         清完后车不会自己动，必须重新下发 VEL；故障没清就别回 OK。
  */
static void cmd_car_clrfault(void)
{
    if (CHS_ClearFault(&g_chassis) != CHS_OK)
    {
        proto_send_frame("ERR,5");
        return;
    }

    proto_send_frame("OK");
}

/**
  * @brief  $CAR,<子命令> —— 底盘命令分发：
  *         VEL / STOP / ODOM / RESET / RAW / WHEEL / ENC / GAIN / CLRFAULT。
  */
static void cmd_car(const char *arg)
{
    char sub[12];
    uint8_t i = 0U;
    const char *p = arg;

    while (*p != '\0' && *p != ',' && i < (sizeof(sub) - 1U))
    {
        sub[i++] = *p++;
    }
    sub[i] = '\0';
    if (*p == ',') p++;      /* p 指向子命令参数 */

    if (strcmp(sub, "VEL") == 0)
        cmd_car_vel(p);
    else if (strcmp(sub, "STOP") == 0)
        cmd_car_stop();
    else if (strcmp(sub, "ODOM") == 0)
        cmd_car_odom();
    else if (strcmp(sub, "RESET") == 0)
        cmd_car_reset();
    else if (strcmp(sub, "RAW") == 0)
        cmd_car_raw(p);
    else if (strcmp(sub, "WHEEL") == 0)
        cmd_car_wheel();
    else if (strcmp(sub, "ENC") == 0)
        cmd_car_enc();
    else if (strcmp(sub, "GAIN") == 0)
        cmd_car_gain(p);
    else if (strcmp(sub, "CLRFAULT") == 0)
        cmd_car_clrfault();
    else
        proto_send_frame("ERR,2");
}

/* ==================== 机械臂命令 $ARM,... ==================== */

/**
  * @brief  $ARM,<子命令>[,...] —— 转发给 arm_robot.c 的 ARM_Exec。
  *         子命令：POS（六轴同步）/ J（单轴）/ HOME / STOP / RESUME /
  *                 READ / TEMP / VIN / CLRFAULT。
  *         POS/J/HOME 只「投递作业」立刻回 OK，真正运动由 ArmTask 执行（见 arm_robot.h）；
  *         READ/TEMP/VIN 是阻塞短查询（每轴一帧，最长约 50ms），在本任务里直接取回。
  *         ARM_Exec 返回非 0 时按统一错误码回 $ERR,<code>。
  */
static void cmd_arm(const char *arg)
{
    char payload[96];
    int rc;

    rc = ARM_Exec(arg != NULL ? arg : "", payload, sizeof(payload));
    if (rc == ARM_OK)
    {
        proto_send_frame(payload);
    }
    else
    {
        char err[16];
        (void)snprintf(err, sizeof(err), "ERR,%d", rc);
        proto_send_frame(err);
    }
}

/* 分发：frame_buf 中为去除 '$' 的完整命令串（"*" 已被截断为 '\0'） */
static void proto_dispatch(void)
{
    char *cmd = (char *)frame_buf;
    char *arg = strchr(cmd, ',');

    if (arg != NULL)
    {
        *arg = '\0';
        arg++;
    }

    if (strcmp(cmd, "PING") == 0)
    {
        cmd_ping();
    }
    else if (strcmp(cmd, "STOP") == 0)
    {
        (void)CHS_Stop(&g_chassis);
        proto_send_frame("OK");
    }
    else if (strcmp(cmd, "LOG") == 0)
    {
        cmd_log(arg != NULL ? arg : "");
    }
    else if (strcmp(cmd, "CAR") == 0)
    {
        cmd_car(arg != NULL ? arg : "");
    }
    else if (strcmp(cmd, "ARM") == 0)
    {
        cmd_arm(arg != NULL ? arg : "");
    }
    else
    {
        proto_send_frame("ERR,2"); /* 未知命令（阶段2定版：2=未知命令） */
    }
}

/* 解析一帧：校验并分发 */
static void proto_parse_frame(void)
{
    uint8_t star = 0U;
    uint8_t has_star = 0U;
    uint8_t i;
    uint8_t rx_ck;
    uint8_t calc_ck;

    for (i = 0U; i < frame_len; i++)
    {
        if (frame_buf[i] == '*')
        {
            star = i;
            has_star = 1U;
            break;
        }
    }

    /* 需要至少 "*" + 2 位校验；两位必须都是十六进制，
       否则 hex_to_nib 会把非法字符当 0，可能让错误校验码蒙混过关。 */
    if (!has_star || (uint8_t)(star + 2U) >= frame_len ||
        !is_hex(frame_buf[star + 1U]) || !is_hex(frame_buf[star + 2U]))
    {
        proto_send_frame("ERR,3");
        return;
    }

    calc_ck = 0U;
    for (i = 0U; i < star; i++) calc_ck ^= frame_buf[i];

    rx_ck = (uint8_t)((hex_to_nib(frame_buf[star + 1U]) << 4) |
                       hex_to_nib(frame_buf[star + 2U]));

    if (rx_ck != calc_ck)
    {
        proto_send_frame("ERR,1"); /* 阶段2定版：1=帧/校验错误 */
        return;
    }

    frame_buf[star] = '\0';
    proto_dispatch();
}

/* ---------- 对外接口 ---------- */

void UartProto_Init(UART_HandleTypeDef *huart)
{
    uint8_t idx;

    if (huart == NULL || proto_uart_count >= PROTO_UART_MAX) return;

    for (idx = 0U; idx < proto_uart_count; idx++)
    {
        if (proto_huarts[idx] == huart) return; /* 已注册 */
    }

    idx = proto_uart_count++;
    proto_huarts[idx] = huart;
    rx_restart_pending[idx] = 0U;
    if (proto_tx_huart == NULL) proto_tx_huart = huart;

    if (huart->Instance == USART1)
    {
        HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
    else if (huart->Instance == USART2)
    {
        HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }
    /* USART3 不在此列：那是总线舵机专用口，阻塞轮询、无中断 */

    if (HAL_UART_Receive_IT(huart, &rx_byte[idx], 1U) != HAL_OK)
    {
        rx_restart_pending[idx] = 1U;
    }
}

void UartProto_HandleByte(uint8_t byte)
{
    if (byte == '$')
    {
        frame_active = 1U;
        frame_len = 0U;
        return;
    }

    if (frame_active == 0U) return; /* 尚未收到起始符 */

    if (byte == '\r' || byte == '\n')
    {
        if (frame_len > 0U)
        {
            frame_active = 0U;
            frame_buf[frame_len] = '\0';
            frame_complete();   /* 只入队，解析由任务完成 */
        }
        return;
    }

    if (frame_len < (PROTO_BUF_SIZE - 1U))
    {
        /* 收满 "...*CK" 两位校验即判定一帧完成并立即应答，不强制依赖 \r\n。
         * 兼容两类手机 TCP 工具：发不带 \\r\\n 的 $PING*10（len=8）或带回车的（len=10）。 */
        if (is_hex(byte) && frame_len >= 2U &&
            frame_buf[frame_len - 2U] == '*' &&
            is_hex(frame_buf[frame_len - 1U]))
        {
            frame_buf[frame_len++] = byte;
            frame_buf[frame_len] = '\0';
            frame_active = 0U;
            frame_complete();   /* 只入队，解析由任务完成 */
            frame_len = 0U;
            return;
        }
        frame_buf[frame_len++] = byte;
    }
    else
    {
        frame_active = 0U; /* 溢出，丢弃整帧 */
        frame_len = 0U;
    }
}

void UartProto_Process(void)
{
    uint8_t idx;

    /* 1. 取走 ISR 收好的帧并解析分发。
     * 命令处理整体从 ISR 迁到本任务（ISR 只收字节入队）。 */
    __disable_irq();
    if (frame_ready != 0U)
    {
        uint16_t i;
        for (i = 0U; i <= frame_hold_len; i++) frame_buf[i] = frame_hold[i];
        /* 解析长度以保留帧为准，防解析期间 ISR 改写全局 frame_len */
        frame_len = (uint8_t)frame_hold_len;
        if (frame_hold_src != NULL) proto_tx_huart = frame_hold_src;
        frame_ready = 0U;

        /* 关键：不能在解析前开中断。frame_buf 是 ISR 的收帧缓冲，
           一旦提前 __enable_irq()，下一帧的 '$' 会立刻改写 frame_buf，
           而 proto_parse_frame()/proto_dispatch() 还在读同一块内存，
           表现为偶发校验错、命令被截断。命令处理本身只做内存操作和
           短临界区，不会阻塞，因此把解析放在临界区里是安全的。 */
        proto_parse_frame();
        __enable_irq();
    }
    else
    {
        __enable_irq();
    }

    for (idx = 0U; idx < proto_uart_count; idx++)
    {
        if (rx_restart_pending[idx] != 0U)
        {
            rx_restart_pending[idx] = 0U;
            if (HAL_UART_Receive_IT(proto_huarts[idx], &rx_byte[idx], 1U) != HAL_OK)
            {
                rx_restart_pending[idx] = 1U;
            }
        }
    }

    /* 真正发送待发响应帧（命令处理已移入本任务，ISR 不再写发送缓冲）。
     * 三个串口统一直接发送。
     * 双缓冲：发完活动槽后检查备用槽，有则搬运继续。
     * 竞态规避：
     *   a)搬运(tx_buf2→tx_buf) b)发送前(tx_pending置0/tx_busy置1)
     *   c)发送后(tx_busy清0) 三处标志/缓冲切换都在关中断临界区内，
     *     确保 ISR 与任务对发送缓冲的操作互不穿插。 */
    __disable_irq();
    if (tx_pending2 != 0U && tx_pending == 0U)
    {
        uint16_t i2;
        for (i2 = 0U; i2 < tx_len2; i2++) tx_buf[i2] = tx_buf2[i2];
        tx_len = tx_len2;
        tx_pending2 = 0U;
        tx_pending = 1U;
    }
    __enable_irq();

    if (tx_pending != 0U && proto_tx_huart != NULL)
    {
        __disable_irq();
        tx_pending = 0U;
        tx_busy = 1U;
        __enable_irq();

        /* 三个串口统一直接发送。目标为 huart1 时与 PrintTask 的 printf
         * 共用 print_lock 互斥，避免响应帧插入周期打印中间。 */
        if (proto_tx_huart->Instance == USART1)
        {
            osMutexAcquire(print_lock, osWaitForever);
            HAL_UART_Transmit(proto_tx_huart, tx_buf, tx_len, 50);
            osMutexRelease(print_lock);
        }
        else
        {
            HAL_UART_Transmit(proto_tx_huart, tx_buf, tx_len, 50);
        }

        __disable_irq();
        tx_busy = 0U;
        __enable_irq();
    }
}

void UartProto_SetStreamLog(uint8_t enable)
{
    stream_log_enabled = (enable != 0U) ? 1U : 0U;
}

uint8_t UartProto_GetStreamLog(void)
{
    return stream_log_enabled;
}

/* ---------- HAL 回调：UART1/UART2 收满 1 字节 ---------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t idx;

    if (huart->Instance != USART1 && huart->Instance != USART2) return;

    for (idx = 0U; idx < proto_uart_count; idx++)
    {
        if (proto_huarts[idx] == huart) break;
    }
    if (idx >= proto_uart_count) return;

    if (rx_byte[idx] == '$')
    {
        frame_src = huart; /* 记录本帧来源，解析后把响应发回这里 */
    }
    UartProto_HandleByte(rx_byte[idx]);

    if (HAL_UART_Receive_IT(huart, &rx_byte[idx], 1U) != HAL_OK)
    {
        rx_restart_pending[idx] = 1U;
    }
}

/* ---------- HAL 回调：UART 错误（ORE 溢出 / NE 噪声 / FE 帧错 / PE 校验错） ----------
 *
 * 【为什么必须有这个函数】
 * HAL 在 UART_IRQHandler 里一旦检测到 ORE（接收溢出），会走 "blocking error" 分支
 * （stm32f4xx_hal_uart.c:2416-2421），先调 UART_EndRxTransfer()，而它会把
 * CR1 的 RXNEIE|PEIE（:3373）和 CR3 的 EIE（:3374）三个中断位全部清掉，
 * 再把 RxState 置为 READY（:3383）。随后 HAL 调用本回调 —— 而 HAL 自带的实现是
 * __weak 空函数（:2679），什么都不做。
 *
 * 【不这么写会怎样】
 * RXNE 中断被关掉后，HAL_UART_RxCpltCallback 再也不会被触发；
 * 而本文件的重挂机制（UartProto_Process 里的 :652-661）依赖 rx_restart_pending，
 * 这个标志只有在 RxCplt（:765）里才会置 1 —— 三条路同时断掉。
 * 结果：USART1 永久收不到任何字节，但 CPU 照常跑、PWM 照常输出、TX 照常能发，
 * 表现为「臂不动、没复位、PING 毫无回应」，只能断电重启才能恢复。
 * 这是 STM32 HAL 的经典坑，现场表现为「莫名其妙就再也收不到命令了」。
 *
 * 【为什么必须先清 ORE 再重挂】
 * HAL 只把 ORE 记进 ErrorCode（:2401），并不主动清硬件标志（唯一的例外是
 * 「ORE 同时伴随 RXNE」时，:2410 的 UART_Receive_IT() 读 DR 会顺带把它清掉）。
 * 若纯 ORE 场景下不清就重挂，下一次中断进来时 SR 里的 ORE 仍为 1，
 * 会立刻再次判定 blocking、再关一次中断，变成「刚挂上就被关掉」的死循环。
 *
 * 【为什么只在 RxState==READY 时才重挂】
 * 非 ORE 的轻度错误（NE/FE/PE）走 HAL 的 non-blocking 分支（:2464-2477），
 * 中断位没被关、接收仍在进行（RxState 还是 BUSY_RX）。这时再调 Receive_IT
 * 只会拿到 HAL_BUSY，虽然无害但没必要，判一下状态更干净。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint8_t idx;

    if (huart->Instance != USART1 && huart->Instance != USART2) return;

    /* 找到这个串口在协议里的下标（拿对应的单字节接收缓冲 rx_byte[idx]） */
    for (idx = 0U; idx < proto_uart_count; idx++)
    {
        if (proto_huarts[idx] == huart) break;
    }
    if (idx >= proto_uart_count) return;

    /* 只有 ORE 需要清硬件标志。__HAL_UART_CLEAR_OREFLAG 的实现是「读 SR + 读 DR」，
     * 读 DR 会把当前字节取走，所以不能无条件调用：
     *   ① NE/FE/PE 时 DR 里可能是有效数据，乱读就丢字节；
     *   ② 若本次 ORE 同时伴随 RXNE，HAL 已在 UART_Receive_IT()（:2410）里读过 DR，
     *      硬件 ORE 位其实已被「读 SR + 读 DR」的时序清掉。这时再读一次 DR，
     *      可能把刚到达的下一个字节吃掉。
     * 所以先用 SR 判硬件位（只读 SR，不动 DR），真的还挂着才清。 */
    if ((huart->ErrorCode & HAL_UART_ERROR_ORE) != 0U)
    {
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)
        {
            __HAL_UART_CLEAR_OREFLAG(huart);
        }
    }
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    /* blocking 错误已被 HAL 置回 READY ⇒ 说明中断被关了，重新挂上单字节接收。
     * 说明：HAL_UART_Receive_IT() 内部只判 RxState==READY（hal_uart.c:1350），
     * 不取 HAL 锁、也不和 TX 仲裁，不会因为 printf 正在发送就失败；
     * 这里刚判过 READY，正常情况必然成功。保留 pending 兜底是防御未来改动。 */
    if (huart->RxState == HAL_UART_STATE_READY)
    {
        if (HAL_UART_Receive_IT(huart, &rx_byte[idx], 1U) != HAL_OK)
        {
            rx_restart_pending[idx] = 1U;
        }
    }
}
