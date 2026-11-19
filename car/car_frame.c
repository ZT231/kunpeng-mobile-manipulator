/*
 * ============================================================================
 * car_frame.c —— 统一帧的「收字节 → 成帧 → 校验 → 翻译 → 分发」
 * ============================================================================
 *
 * 【为什么单独一个 .c】原项目 1.Smart car 的 main.c 里是「读一个字符、
 *   switch 一个字符」（F/B/L/R/S）。那样只能手机蓝牙按键，没法带参数，
 *   也没法被鲲鹏 / ESP32 用同一套语法调度。这里把「怎么解析一句话」抽出来，
 *   应用层只管「收到命令后怎么动电机」，两边互不干扰。
 *
 * 【C 新手提示】文件里所有 static 的变量/函数都是「本文件私有」的，
 *   别的文件看不见 —— C 没有 private 关键字，static 就是最朴素的封装。
 *   变量一律声明在块的开头：Keil MDK 默认按 C89 编译，C89 不允许
 *   「用到哪、定义到哪」（那是 C99 才有的便利）。
 */

#include "car_frame.h"
#include <string.h>

/* ---------------- 私有状态 ---------------- */
static char     s_buf[CAR_FRAME_MAX_LEN + 1];  /* 攒 payload 的缓冲，+1 放 '\0' */
static uint16_t s_len;                        /* 已经攒了几个字节 */
static uint8_t  s_state;                      /* 状态机当前状态 */
static char     s_ck_hi;                      /* 收到的校验和高位字符 */
static char     s_ck_lo;                      /* 收到的校验和低位字符 */
static car_on_command_fn s_on_cmd;            /* 应用层注册：执行命令 */
static car_send_fn       s_send;              /* 应用层注册：发应答 */

/* 状态机的四个状态。用宏给数字起名字，比直接写 0/1/2/3 好懂得多。 */
#define ST_IDLE   0   /* 等 '$' 开头 */
#define ST_BODY   1   /* 正在收 '$' 与 '*' 之间的内容 */
#define ST_CK_HI  2   /* 正在收校验和的高 1 位 */
#define ST_CK_LO  3   /* 正在收校验和的低 1 位 */

static const char HEX[] = "0123456789ABCDEF";   /* 校验和要输出大写十六进制 */

/* ---------------- 小工具函数 ---------------- */

/* XOR 校验：把一段字节从头到尾逐个异或。
   为什么用 XOR 而不是累加和？XOR 对「字节顺序颠倒」也会变，
   且实现只要一行，单片机上零成本。（机械臂固件用的就是同一套） */
static uint8_t xor_of(const char *p, uint16_t n)
{
    uint8_t ck = 0U;
    uint16_t i;
    for (i = 0U; i < n; i++) {
        ck ^= (uint8_t)p[i];
    }
    return ck;
}

/* 把一个 '0'~'9' / 'A'~'F' 的字符转成 0~15；不是合法字符返回 -1。
   用 int 而不是 uint8_t 当返回值，就是为了能返回 -1 表示「出错」。 */
static int hex_val(char c)
{
    if (c >= '0' && c <= '9') return (int)(c - '0');
    if (c >= 'A' && c <= 'F') return (int)(c - 'A') + 10;
    if (c >= 'a' && c <= 'f') return (int)(c - 'a') + 10;  /* 顺手兼容小写 */
    return -1;
}

/* 复位状态机。注意：只清「状态和长度」，不清 s_buf 里的字符 ——
   因为结算校验和时还要用 s_buf 的内容。 */
static void frame_reset(void)
{
    s_len = 0U;
    s_state = ST_IDLE;
}

/* 组装并发一帧应答。
   入参 payload 是 '$' 与 '*' 之间的内容（如 "OK,PONG" / "ERR,3"），
   本函数负责补上 '$'、'*'、两位校验和、'\r'、'\n' 和字符串结尾 '\0'。
   这样应用层永远不用自己算校验，少一个能写错的地方。 */
static void send_reply(const char *payload)
{
    char out[CAR_FRAME_MAX_LEN + 8];
    uint16_t n;
    uint16_t i;
    uint8_t ck;

    if (s_send == 0) return;              /* 没注册发送回调就静默返回，防野指针 */
    n = (uint16_t)strlen(payload);
    if (n > CAR_FRAME_MAX_LEN) n = CAR_FRAME_MAX_LEN;

    out[0] = '$';
    for (i = 0U; i < n; i++) {
        out[1U + i] = payload[i];
    }
    ck = xor_of(payload, n);
    out[1U + n] = '*';
    out[2U + n] = HEX[(ck >> 4) & 0x0FU];
    out[3U + n] = HEX[ck & 0x0FU];
    out[4U + n] = '\r';
    out[5U + n] = '\n';
    out[6U + n] = '\0';

    s_send(out);
}

/* 匹配一个字段，并把指针往后挪。
   例如 p 指向 "FWD,800"，match_field(&p,"FWD") 成功后 p 指向 "800"。
   用「指针的指针」是因为要在函数里改动调用者手里的那个指针 ——
   这是 C 里最基础也最容易写错的一处，写成 const char ** 表示「改的是指针，不改字符」。 */
static int match_field(const char **pp, const char *word)
{
    uint16_t n = (uint16_t)strlen(word);
    if (strncmp(*pp, word, n) != 0) return 0;
    /* 必须刚好到词尾或遇逗号，否则 "FWDX" 会被误认成 "FWD" */
    if ((*pp)[n] != '\0' && (*pp)[n] != ',') return 0;
    *pp += n;
    if (**pp == ',') (*pp)++;
    return 1;
}

/* 解析一个十进制无符号数，成功返回 1 并把值写进 *out。
   不调用 sscanf / atoi：这两个在单片机上会拖进一大块库代码，
   而且碰到非法字符行为不好预测。手写一个循环反而更小、更可控。 */
static int parse_u32(const char **pp, uint32_t *out)
{
    uint32_t v = 0U;
    int got = 0;
    while (**pp >= '0' && **pp <= '9') {
        /* 顺手防溢出：超过上限就钳住，不让它绕回 0 */
        if (v < 100000000UL) {
            v = v * 10U + (uint32_t)(**pp - '0');
        }
        (*pp)++;
        got = 1;
    }
    if (!got) return 0;
    *out = v;
    return 1;
}

/* ---------------- 校验与分发 ---------------- */

/* 把 payload 翻译成动作，做量程仲裁，然后回调应用层 */
static void parse_and_dispatch(void)
{
    const char *p = s_buf;
    car_command_t cmd;

    cmd.action   = CAR_ACT_UNKNOWN;
    cmd.arg1     = 0U;
    cmd.arg2     = 0U;
    cmd.has_arg2 = 0U;

    /* PING 是两个节点（小车 / 机械臂）都认的通用命令，先单独处理 */
    if (strcmp(p, "PING") == 0) {
        cmd.action = CAR_ACT_PING;
        send_reply("OK,PONG");
        if (s_on_cmd != 0) s_on_cmd(&cmd);
        return;
    }

    /* 小车命令一律以 "CAR," 开头；别的节点前缀（如 JOINT）不该发到这里 */
    if (strncmp(p, "CAR,", 4) != 0) {
        send_reply("ERR,2");            /* 未知命令 */
        return;
    }
    p += 4;

    /* ---- 逐个认动作 ---- */
    if      (match_field(&p, "STOP"))  cmd.action = CAR_ACT_STOP;
    else if (match_field(&p, "FWD"))   cmd.action = CAR_ACT_FWD;
    else if (match_field(&p, "BWD"))   cmd.action = CAR_ACT_BWD;
    else if (match_field(&p, "SPINL")) cmd.action = CAR_ACT_SPINL;
    else if (match_field(&p, "SPINR")) cmd.action = CAR_ACT_SPINR;
    else if (match_field(&p, "TURNL")) cmd.action = CAR_ACT_TURNL;
    else if (match_field(&p, "TURNR")) cmd.action = CAR_ACT_TURNR;
    else if (match_field(&p, "SPD"))   cmd.action = CAR_ACT_SPD;
    else if (match_field(&p, "MODE"))  cmd.action = CAR_ACT_MODE;
    else {
        send_reply("ERR,2");
        return;
    }

    /* ---- 按动作收参数 ---- */
    if (cmd.action == CAR_ACT_STOP) {
        /* 停车不需要参数；多给了参数也宽容忽略（上位机好写） */
        send_reply("OK");
        if (s_on_cmd != 0) s_on_cmd(&cmd);
        return;
    }

    if (!parse_u32(&p, &cmd.arg1)) {
        send_reply("ERR,3");            /* 缺参数 / 参数不是数字 */
        return;
    }

    /* 可选第二参数：速度 PWM。例 $CAR,FWD,800,900 表示走 800ms、PWM=900 */
    if (*p == ',') {
        p++;
        if (!parse_u32(&p, &cmd.arg2)) {
            send_reply("ERR,3");
            return;
        }
        cmd.has_arg2 = 1U;
    }

    /* ---- 本地仲裁：量程闸门（这就是「LLM 只能提议、本地才授权」的落点） ---- */
    if (cmd.action == CAR_ACT_SPD || cmd.action == CAR_ACT_MODE) {
        if (cmd.action == CAR_ACT_SPD && cmd.arg1 > CAR_MAX_PWM) {
            send_reply("ERR,3");
            return;
        }
        if (cmd.action == CAR_ACT_MODE && cmd.arg1 > 2U) {
            send_reply("ERR,3");
            return;
        }
    } else {
        /* 运动命令：arg1 是时长 ms，arg2（若有）是 PWM */
        if (cmd.arg1 > CAR_MAX_RUN_MS) {
            send_reply("ERR,3");
            return;
        }
        if (cmd.has_arg2 && cmd.arg2 > CAR_MAX_PWM) {
            send_reply("ERR,3");
            return;
        }
    }

    send_reply("OK");
    if (s_on_cmd != 0) s_on_cmd(&cmd);
}

/* 收满两位校验和后结算 */
static void frame_finish(void)
{
    int hi;
    int lo;
    uint8_t ck_recv;
    uint8_t ck_calc;
    uint16_t body_len;

    /* 先把状态复位，回调里如果又喂字节也不会串帧 */
    hi = hex_val(s_ck_hi);
    lo = hex_val(s_ck_lo);
    body_len = (uint16_t)strlen(s_buf);
    frame_reset();

    if (hi < 0 || lo < 0) {
        send_reply("ERR,1");            /* 校验和不是十六进制 */
        return;
    }
    ck_recv = (uint8_t)(((uint8_t)hi << 4) | (uint8_t)lo);
    ck_calc = xor_of(s_buf, body_len);
    if (ck_recv != ck_calc) {
        send_reply("ERR,1");            /* 校验和不符 —— 线上干扰 / 波特率不对 */
        return;
    }
    parse_and_dispatch();
}

/* ---------------- 对外接口 ---------------- */

void car_frame_init(car_on_command_fn on_cmd, car_send_fn send)
{
    s_on_cmd = on_cmd;
    s_send = send;
    frame_reset();
    s_buf[0] = '\0';
}

void car_frame_feed_byte(uint8_t byte)
{
    char c = (char)byte;

    switch (s_state) {
    case ST_IDLE:
        /* 帧外的字节（包括上一帧尾巴的 \r\n、蓝牙连上时的垃圾）全部丢掉 */
        if (c == '$') {
            s_len = 0U;
            s_state = ST_BODY;
        }
        break;

    case ST_BODY:
        if (c == '*') {
            s_buf[s_len] = '\0';        /* 把 '*' 换成字符串结束符 */
            s_state = ST_CK_HI;
        } else if (c == '\r' || c == '\n') {
            frame_reset();              /* 没带校验和，整帧作废 */
        } else if (s_len < CAR_FRAME_MAX_LEN) {
            s_buf[s_len] = c;
            s_len++;
        } else {
            send_reply("ERR,1");        /* 超长：直接丢弃并报错 */
            frame_reset();
        }
        break;

    case ST_CK_HI:
        s_ck_hi = c;
        s_state = ST_CK_LO;
        break;

    case ST_CK_LO:
        s_ck_lo = c;
        /* 收满两位就结算，不强求后面一定有 \r\n；
           随后到来的 \r / \n 会在 ST_IDLE 被自然忽略。 */
        frame_finish();
        break;

    default:
        frame_reset();
        break;
    }
}

void car_frame_test_echo(const char *payload)
{
    send_reply(payload);
}
