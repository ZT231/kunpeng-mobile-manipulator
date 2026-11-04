#include "uart_protocol.h"
#include "arm_control.h"
#include "teach.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>

/* print_lock 定义于 freertos.c：任务层共享，保护 huart1 上的 printf/响应发送互斥，
 * 避免 PrintTask 周期打印与 ProtocolTask 发送响应帧按字符交错。 */
extern osMutexId_t print_lock;

/* arm_lock 定义于 freertos.c：机械臂状态互斥。
 * ProtocolTask 处理命令时持有（覆盖所有 Arm 与 Teach 接口调用点），
 * 与 ServoTask 的运动/Teach_Tick 更新互斥。 */
extern osMutexId_t arm_lock;

/*
 * UART 指令协议（v1）
 *
 * 帧格式：  $<CMD>[,<ARG>...]*<CK><CR><LF>
 *   CK 为 CMD 字段（$ 与 * 之间）所有字节的 XOR，2 位大写十六进制。
 *
 * 请求示例：
 *   $PING*XX\r\n
 *   $MOVE,0,1500*XX\r\n
 *   $JOINT,1500,1500,1500,1500,1500,1500*XX\r\n
 * 响应格式：
 *   $OK[,<数据>]*XX\r\n
 *   $ERR,<code>*XX\r\n
 *     错误码：2=校验失败 3=格式/参数错误 4=当前状态禁止
 */

#define PROTO_BUF_SIZE   64U
#define PROTO_TX_SIZE    96U
#define PROTO_UART_MAX   3U

static const char HEX_DIGITS[] = "0123456789ABCDEF";

/* 多控制源：最多 3 个串口（USART1=PC/OrangePi，USART2=蓝牙，USART3=WiFi） */
static UART_HandleTypeDef *proto_huarts[PROTO_UART_MAX];
static uint8_t proto_uart_count = 0U;
/* 响应发回目标：最后收到 '$' 起始符的那个串口（谁命令谁收响应） */
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

/* ---------------- USART3 (WiFi) CIPSERVER 适配 ----------------
 * ESP 处于 CIPMUX=1 + CIPSERVER=1（非透传）：手机 TCP 数据以
 *   +IPD,<link>,<len>:<data...>
 * 前缀形式上报；STM32 应答需先发 AT+CIPSEND=<link>,<len> 并等 '>' 提示符，
 * 再发真数据。这两处与 USART1/USART2 不同，故单独处理。
 * 注意：第一条命令可带无 link 的旧式 +IPD,<len>:，这里按 CIPMUX 多连接
 * 的标准 +IPD,<link>,<len>: 解析（演示单手机连接，link 固定为 0）。 */
static UART_HandleTypeDef *wifi_uart = NULL;     /* 记录 USART3 句柄 */
static uint8_t  wifi_ipd_state = 0U;             /* 0=空闲 1=收头部 2=收数据 */
static uint8_t  wifi_ipd_link = 0U;              /* 最近 +IPD 的 link id */
static uint16_t wifi_ipd_len = 0U;               /* 期望的数据长度 */
static uint16_t wifi_ipd_cnt = 0U;               /* 已收数据字节数 */
static char     wifi_ipd_line[20];               /* "+IPD,l,l" 头部缓冲 */
static uint8_t  wifi_ipd_line_len = 0U;
static volatile uint8_t wifi_cipsend_wait_prompt = 0U; /* 1=等'>' */

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

/* ---------- USART3 (WiFi) 专用：+IPD/CIPSEND ---------- */

/* 前向声明：wifi_ipd_parse 在本文件定义处(解析辅助一节)之后用到 */
static int parse_uint(const char **pp, uint32_t *val);

/* 解析 ipd 头部缓冲（内容形如 "IPD,0,10:"）。注意：wifi_rx_byte 的 state0
 * 收到 '+' 时只切状态、不存入 line，故 line 里不含 '+'，以 "IPD..." 开头。
 * 成功返回 1 并把 link/len 写入；失败返回 0。 */
static int wifi_ipd_parse(void)
{
    const char *p = (const char *)wifi_ipd_line + 3U; /* 跳过 'I''P''D' */
    uint32_t v;

    if (strncmp((const char *)wifi_ipd_line, "IPD", 3U) != 0) return 0;
    if (!parse_uint(&p, &v)) return 0;
    wifi_ipd_link = (uint8_t)v;
    if (!parse_uint(&p, &v)) return 0;
    wifi_ipd_len = (uint16_t)v;
    return 1;
}

/* 把 USART3 收到的 1 字节喂给 WiFi IPD 状态机：
 *   剥壳出的"真数据"（手机发来的 $CMD 帧）交由通用解析，其余文字被忽略。
 * 可从 ISR 调用。 */
static void wifi_rx_byte(uint8_t b)
{
    if (wifi_ipd_state == 0U)
    {
        if (b == '+') { wifi_ipd_state = 1U; wifi_ipd_line_len = 0U; }
        return; /* 其余（AT 回显等）忽略 */
    }
    if (wifi_ipd_state == 1U)
    {
        if (b == '\r' || b == '\n') { wifi_ipd_state = 0U; return; } /* 坏行 */
        if (wifi_ipd_line_len < (uint8_t)sizeof(wifi_ipd_line) - 1U)
        {
            wifi_ipd_line[wifi_ipd_line_len++] = (char)b;
        }
        if (b == ':')
        {
            if (!wifi_ipd_parse() || wifi_ipd_len == 0U) { wifi_ipd_state = 0U; return; }
            wifi_ipd_cnt = 0U;
            wifi_ipd_state = 2U;
        }
        return;
    }
    /* state 2：接收 len 字节真数据 */
    if (wifi_ipd_cnt < wifi_ipd_len)
    {
        if (b == '$' && wifi_uart != NULL) proto_tx_huart = wifi_uart;
        UartProto_HandleByte(b);
        wifi_ipd_cnt++;
    }
    if (wifi_ipd_cnt >= wifi_ipd_len) wifi_ipd_state = 0U;
}

/* USART3 发送：用 AT+CIPSEND 封装响应帧，等 ESP 返回 '>' 提示符后发真数据。 */
static void wifi_send_tcp_frame(const uint8_t *data, uint16_t len)
{
    char cmd[32];
    int n;
    uint32_t t0;

    if (wifi_uart == NULL) return;

    n = snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u,%u\r\n",
                 (unsigned)wifi_ipd_link, (unsigned)len);
    if (n <= 0 || n >= (int)sizeof(cmd)) return;

    wifi_cipsend_wait_prompt = 1U;
    (void)HAL_UART_Transmit(wifi_uart, (uint8_t *)cmd, (uint16_t)n, 100);

    /* 等 '>'（由 ISR 置位 wifi_cipsend_wait_prompt=0），最多 ~500ms。
     * osDelay 让出 CPU，避免忙等饿死同/低优先级任务（PrintTask）。 */
    t0 = HAL_GetTick();
    while (wifi_cipsend_wait_prompt != 0U && (HAL_GetTick() - t0) < 500U)
    {
        osDelay(1);
    }

    if (data != NULL && len > 0U)
        (void)HAL_UART_Transmit(wifi_uart, (uint8_t *)data, len, 300);
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
        v = v * 10U + (uint32_t)(*p - '0');
        p++;
        any = 1;
    }

    *val = v;
    *pp = p;
    return any;
}

/* ---------- 命令处理 ---------- */

static void cmd_ping(void)
{
    proto_send_frame("OK,PONG");
}

static void cmd_state(void)
{
    char payload[PROTO_TX_SIZE];
    char state_ch;

    switch (Arm_GetState())
    {
        case ARM_STATE_MANUAL:   state_ch = 'M'; break;
        case ARM_STATE_CMD:      state_ch = 'C'; break;
        case ARM_STATE_RECORD:   state_ch = 'R'; break;
        case ARM_STATE_PLAYBACK: state_ch = 'P'; break;
        default:                 state_ch = 'E'; break; /* ESTOP */
    }

    (void)snprintf(payload, sizeof(payload),
                   "OK,S,%c,%u,%u,%u,%u,%u,%u",
                   state_ch,
                   (unsigned)Arm_GetCurrent(0), (unsigned)Arm_GetCurrent(1),
                   (unsigned)Arm_GetCurrent(2), (unsigned)Arm_GetCurrent(3),
                   (unsigned)Arm_GetCurrent(4), (unsigned)Arm_GetCurrent(5));
    proto_send_frame(payload);
}

static void cmd_move(const char *arg)
{
    uint32_t ch = 0U;
    uint32_t pulse = 0U;

    if (!parse_uint(&arg, &ch)) { proto_send_frame("ERR,3"); return; }
    if (!parse_uint(&arg, &pulse)) { proto_send_frame("ERR,3"); return; }

    if (ch >= ARM_CHANNEL_COUNT || pulse > 2500U)
    {
        proto_send_frame("ERR,3");
        return;
    }

    if (Arm_SetJointTarget((uint8_t)ch, (uint16_t)pulse))
    {
        proto_send_frame("OK");
    }
    else
    {
        proto_send_frame("ERR,4"); /* 急停等状态禁止 */
    }
}

static void cmd_joint(const char *arg)
{
    uint16_t p[ARM_CHANNEL_COUNT];
    uint32_t v = 0U;
    uint8_t i;

    for (i = 0U; i < ARM_CHANNEL_COUNT; i++)
    {
        if (!parse_uint(&arg, &v) || v > 2500U)
        {
            proto_send_frame("ERR,3");
            return;
        }
        p[i] = (uint16_t)v;
    }

    if (Arm_SetAllTargets(p))
    {
        proto_send_frame("OK");
    }
    else
    {
        proto_send_frame("ERR,4");
    }
}

static void cmd_rec(void)
{
    if (Arm_GetState() == ARM_STATE_ESTOP) { proto_send_frame("ERR,4"); return; }
    Teach_StartRecord();
    proto_send_frame("OK");
}

static void cmd_recend(void)
{
    char payload[PROTO_TX_SIZE];

    if (Arm_GetState() == ARM_STATE_ESTOP) { proto_send_frame("ERR,4"); return; }
    Teach_StopRecord();
    (void)snprintf(payload, sizeof(payload), "OK,%u", (unsigned)Teach_GetPointCount());
    proto_send_frame(payload);
}

static void cmd_play(void)
{
    if (Arm_GetState() == ARM_STATE_ESTOP) { proto_send_frame("ERR,4"); return; }
    if (Teach_GetPointCount() == 0U) { proto_send_frame("ERR,3"); return; } /* 无轨迹 */
    Teach_StartPlayback();
    proto_send_frame("OK");
}

static void cmd_tnum(void)
{
    char payload[PROTO_TX_SIZE];

    (void)snprintf(payload, sizeof(payload), "OK,%u", (unsigned)Teach_GetPointCount());
    proto_send_frame(payload);
}

static void cmd_clear(void)
{
    if (Arm_GetState() == ARM_STATE_RECORD || Arm_GetState() == ARM_STATE_PLAYBACK)
    {
        proto_send_frame("ERR,4");
        return;
    }
    Teach_Clear();
    proto_send_frame("OK");
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
    else if (strcmp(cmd, "STATE") == 0)
    {
        cmd_state();
    }
    else if (strcmp(cmd, "HOME") == 0)
    {
        if (Arm_GetState() == ARM_STATE_ESTOP) { proto_send_frame("ERR,4"); return; }
        Arm_CommandHome();
        proto_send_frame("OK");
    }
    else if (strcmp(cmd, "STOP") == 0)
    {
        Arm_CommandStop();
        proto_send_frame("OK");
    }
    else if (strcmp(cmd, "RESUME") == 0)
    {
        Arm_CommandResume();
        proto_send_frame("OK");
    }
    else if (strcmp(cmd, "MANUAL") == 0)
    {
        if (Arm_GetState() == ARM_STATE_ESTOP) { proto_send_frame("ERR,4"); return; }
        Arm_CommandManual();
        proto_send_frame("OK");
    }
    else if (strcmp(cmd, "MOVE") == 0)
    {
        cmd_move(arg != NULL ? arg : "");
    }
    else if (strcmp(cmd, "JOINT") == 0)
    {
        cmd_joint(arg != NULL ? arg : "");
    }
    else if (strcmp(cmd, "REC") == 0)
    {
        cmd_rec();
    }
    else if (strcmp(cmd, "RECEND") == 0)
    {
        cmd_recend();
    }
    else if (strcmp(cmd, "PLAY") == 0)
    {
        cmd_play();
    }
    else if (strcmp(cmd, "TNUM") == 0)
    {
        cmd_tnum();
    }
    else if (strcmp(cmd, "CLEAR") == 0)
    {
        cmd_clear();
    }
    else if (strcmp(cmd, "LOG") == 0)
    {
        cmd_log(arg != NULL ? arg : "");
    }
    else
    {
        proto_send_frame("ERR,1"); /* 未知命令 */
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

    /* 需要至少 "*" + 2 位校验 */
    if (!has_star || (uint8_t)(star + 2U) >= frame_len)
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
        proto_send_frame("ERR,2");
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
    else if (huart->Instance == USART3)
    {
        HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
        wifi_uart = huart; /* 记录 WiFi 句柄，供 CIPSEND 发送使用 */
    }

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
     * 命令处理整体从 ISR 迁到本任务（ISR 只收字节入队），并且处理全程持有
     * arm_lock，与 ServoTask 的运动/Teach 状态操作互斥。 */
    __disable_irq();
    if (frame_ready != 0U)
    {
        uint16_t i;
        for (i = 0U; i <= frame_hold_len; i++) frame_buf[i] = frame_hold[i];
        /* 解析长度以保留帧为准，防解析期间 ISR 改写全局 frame_len */
        frame_len = (uint8_t)frame_hold_len;
        frame_ready = 0U;
        __enable_irq();
        /* 命令处理持有 arm_lock：与 ServoTask 的 Arm_Update/Teach_Tick/按键/RFID
         * 状态操作互斥，保证 target_pulse/arm_state/teach_* 无任务间竞态。 */
        osMutexAcquire(arm_lock, osWaitForever);
        proto_parse_frame();
        osMutexRelease(arm_lock);
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
     * USART1/USART2 直接发送；USART3(WiFi) 处于 CIPSERVER 非透传，
     * 用 AT+CIPSEND 封装后发送（见 wifi_send_tcp_frame）。
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

        if (proto_tx_huart->Instance == USART3)
        {
            /* WiFi：CIPSERVER 非透传，用 CIPSEND 封装后发送（会阻塞等 '>'） */
            wifi_send_tcp_frame(tx_buf, tx_len);
        }
        else
        {
            /* USART1/USART2：直接发送。目标为 huart1 时与 PrintTask/ServoTask
             * 的 printf 共用 print_lock 互斥，避免响应帧插入周期打印中间。 */
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

    if (huart->Instance != USART1 && huart->Instance != USART2 &&
        huart->Instance != USART3) return;

    for (idx = 0U; idx < proto_uart_count; idx++)
    {
        if (proto_huarts[idx] == huart) break;
    }
    if (idx >= proto_uart_count) return;

    if (huart->Instance == USART3)
    {
        /* USART3(WiFi)：'$' 会经 +IPD 剥壳，在 wifi_rx_byte 内部更新 proto_tx_huart；
         * 这里只处理 CIPSEND 的 '>' 提示符与 IPD 解析。 */
        if (wifi_cipsend_wait_prompt != 0U && rx_byte[idx] == '>')
        {
            wifi_cipsend_wait_prompt = 0U;
        }
        else
        {
            wifi_rx_byte(rx_byte[idx]);
        }
    }
    else
    {
        if (rx_byte[idx] == '$')
        {
            proto_tx_huart = huart; /* 响应发回命令来源串口 */
        }
        UartProto_HandleByte(rx_byte[idx]);
    }

    if (HAL_UART_Receive_IT(huart, &rx_byte[idx], 1U) != HAL_OK)
    {
        rx_restart_pending[idx] = 1U;
    }
}
