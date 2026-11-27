/**
 * arm_robot.c — 机械臂控制层（六轴总线舵机，USART3 / PB10=TX + PB11=RX）
 *
 * 【命令一览】（完整帧形如 $ARM,<子命令>[,参数...]*CK\r\n，CK = $ 与 * 之间字节的 XOR）
 *   $ARM,POS,<p1>,<p2>,<p3>,<p4>,<p5>,<p6>[,<ms>]  六轴同步移动，位置单位 0~1000，ms 默认 800
 *   $ARM,J,<轴号1~6>,<位置>[,<ms>]                单轴移动（J5 自动走背隙补偿，ms 默认 500）
 *   $ARM,HOME[,<ms>]                              六轴回零位（460/514/516/502/494/674）
 *   $ARM,STOP                                     六轴停在当前位置，并撤销未执行的作业
 *   $ARM,RESUME                                   解除 STOP 的暂停
 *   $ARM,READ                                     回 $OK,POS,<p1>,...,<p6>
 *   $ARM,TEMP                                     回 $OK,TEMP,<t1>,...,<t6>（℃）
 *   $ARM,VIN                                      回 $OK,VIN,<mv1>,...,<mv6>（mV）
 *   $ARM,CLRFAULT                                 清过温锁停（须温度已回落）
 *
 * 【位置语义】全部是舵机原始位置单位（0~1000 ↔ 0~240°，1 单位 = 0.24°），
 * 不是角度。零位/方向换算在鲲鹏侧 arm_params.json（joints 的 pos_zero 与 dir），
 * 固件只负责把「已知零位的原始位置」忠实执行，不在 MCU 里重复一遍运动学。
 */
#include "arm_robot.h"
#include <stdio.h>
#include <string.h>

/* ================= 内部状态 ================= */

static SB_Handle s_sb;                 /* 六轴档案（位置/温度/电压/在线/故障） */

/* 作业邮箱：生产者 = ProtocolTask（ARM_Exec），消费者 = ArmTask（ARM_TaskTick）。
   单生产者单消费者，且 s_job_busy 用「先填写、再置忙」的顺序，
   消费者执行完才清零，所以不存在读到半套参数的窗口。 */
typedef enum {
    ARM_JOB_NONE = 0,
    ARM_JOB_MOVE_ALL,
    ARM_JOB_MOVE_ONE,
    ARM_JOB_HOME
} ARM_Job;

static volatile uint8_t  s_job_kind;                 /* ARM_Job */
static volatile uint8_t  s_job_busy;                 /* 1=已受理（待执行/执行中） */
static volatile uint8_t  s_job_joint;                /* 单轴作业的轴号 0~5 */
static volatile uint16_t s_job_pos[SB_JOINT_NUM];    /* 六轴作业的目标位置 */
static volatile uint16_t s_job_ms;                   /* 运动时间 */

static const uint16_t k_home[SB_JOINT_NUM] = SB_HOME_POS_LIST;

/* ================= 小工具 ================= */

/* 解析十进制无符号整数，跳过前导空格/逗号；成功 1、失败 0。
   与 uart_protocol.c 的同名逻辑保持一致的溢出判定。 */
static int arm_parse_u32(const char **pp, uint32_t *val)
{
    const char *p = *pp;
    uint32_t v = 0U;
    int any = 0;

    while (*p == ' ' || *p == ',') p++;
    if (*p < '0' || *p > '9') return 0;

    while (*p >= '0' && *p <= '9')
    {
        uint32_t digit = (uint32_t)(*p - '0');
        if (v > ((0xFFFFFFFFU - digit) / 10U)) return 0;
        v = v * 10U + digit;
        p++;
        any = 1;
    }

    *val = v;
    *pp = p;
    return any;
}

/* 取子命令（到下一个逗号为止），p 推进到参数起点 */
static void arm_sub(const char **pp, char *sub, uint8_t sub_max)
{
    const char *p = *pp;
    uint8_t i = 0U;

    while (*p != '\0' && *p != ',' && i < (uint8_t)(sub_max - 1U))
    {
        sub[i++] = *p++;
    }
    sub[i] = '\0';
    if (*p == ',') p++;
    *pp = p;
}

/* ================= 作业执行（只在 ArmTask 里跑） ================= */

static void arm_run_job(void)
{
    uint8_t kind, i;
    uint16_t pos[SB_JOINT_NUM];

    if (s_job_busy == 0U) return;

    kind = s_job_kind;
    for (i = 0U; i < SB_JOINT_NUM; i++) pos[i] = s_job_pos[i];

    switch (kind)
    {
    case ARM_JOB_MOVE_ALL:
        (void)SB_MoveAll(&s_sb, pos, s_job_ms);
        break;

    case ARM_JOB_MOVE_ONE:
        /* 单轴走 SB_MovePos：J5 会做背隙补偿两段式，这里阻塞是设计如此 */
        (void)SB_MovePos(&s_sb, s_job_joint, s_job_pos[s_job_joint], s_job_ms);
        break;

    case ARM_JOB_HOME:
        (void)SB_MoveAll(&s_sb, k_home, s_job_ms);
        break;

    default:
        break;
    }

    s_job_kind = ARM_JOB_NONE;
    s_job_busy = 0U;                    /* 🔴 只覆盖「下发 6 帧」这几毫秒：框内新命令回 $ERR,4；
                                           发完即返回，此时舵机仍在爬坡 —— 再发运动命令会被接受并
                                           覆盖在途斜坡（不跳变），要中止请用 $ARM,STOP */
}

/* ================= 对外 ================= */

void ARM_Init(void)
{
    s_job_kind  = ARM_JOB_NONE;
    s_job_busy  = 0U;
    s_job_joint = 0U;
    s_job_ms    = 0U;
    SB_Init(&s_sb);
}

void ARM_TaskTick(void)
{
    arm_run_job();       /* 有作业先执行（可能阻塞到运动结束） */
    SB_Tick10ms(&s_sb);  /* 再走一拍轮询：位置 1s / 电压 5s / 温度状态机 */
}

int ARM_Exec(const char *arg, char *out, uint16_t out_max)
{
    char sub[10];
    const char *p;
    uint32_t v, ms;
    uint16_t pos[SB_JOINT_NUM];
    uint8_t i;
    int rc;

    if (arg == NULL || out == NULL || out_max < 8U) return ARM_ERR_PARAM;

    p = arg;
    arm_sub(&p, sub, sizeof(sub));

    /* ---------- 六轴同步移动 ---------- */
    if (strcmp(sub, "POS") == 0)
    {
        for (i = 0U; i < SB_JOINT_NUM; i++)
        {
            if (!arm_parse_u32(&p, &v) || v > SB_POS_MAX) return ARM_ERR_PARAM;
            pos[i] = (uint16_t)v;
        }
        ms = 800U;
        if (arm_parse_u32(&p, &v))
        {
            if (v > 60000U) return ARM_ERR_PARAM;
            ms = v;
        }
        if (s_sb.fault)    return ARM_ERR_FAULT;
        if (s_job_busy)    return ARM_ERR_BUSY;

        for (i = 0U; i < SB_JOINT_NUM; i++) s_job_pos[i] = pos[i];
        s_job_ms = (uint16_t)ms;
        s_job_kind = ARM_JOB_MOVE_ALL;
        s_job_busy = 1U;

        (void)snprintf(out, out_max, "OK,ARM,POS,%u", (unsigned)ms);
        return ARM_OK;
    }

    /* ---------- 单轴移动 ---------- */
    if (strcmp(sub, "J") == 0)
    {
        if (!arm_parse_u32(&p, &v) || v < 1U || v > SB_JOINT_NUM) return ARM_ERR_PARAM;
        i = (uint8_t)(v - 1U);                       /* 上位机用 1~6，内部 0~5 */
        if (!arm_parse_u32(&p, &v) || v > SB_POS_MAX) return ARM_ERR_PARAM;
        pos[i] = (uint16_t)v;
        ms = 500U;
        if (arm_parse_u32(&p, &v))
        {
            if (v > 60000U) return ARM_ERR_PARAM;
            ms = v;
        }
        if (s_sb.fault)    return ARM_ERR_FAULT;
        if (s_job_busy)    return ARM_ERR_BUSY;

        s_job_joint = i;
        s_job_pos[i] = pos[i];
        s_job_ms = (uint16_t)ms;
        s_job_kind = ARM_JOB_MOVE_ONE;
        s_job_busy = 1U;

        (void)snprintf(out, out_max, "OK,ARM,J,%u,%u", (unsigned)(i + 1U), (unsigned)ms);
        return ARM_OK;
    }

    /* ---------- 回零 ---------- */
    if (strcmp(sub, "HOME") == 0)
    {
        ms = 1000U;
        if (arm_parse_u32(&p, &v))
        {
            if (v > 60000U) return ARM_ERR_PARAM;
            ms = v;
        }
        if (s_sb.fault) return ARM_ERR_FAULT;
        if (s_job_busy) return ARM_ERR_BUSY;

        for (i = 0U; i < SB_JOINT_NUM; i++) s_job_pos[i] = k_home[i];
        s_job_ms = (uint16_t)ms;
        s_job_kind = ARM_JOB_HOME;
        s_job_busy = 1U;

        (void)snprintf(out, out_max, "OK,ARM,HOME,%u", (unsigned)ms);
        return ARM_OK;
    }

    /* ---------- 急停：撤销作业 + 六轴原地停 ---------- */
    if (strcmp(sub, "STOP") == 0)
    {
        s_job_busy = 0U;                 /* 撤销还没开始执行的作业 */
        s_job_kind = ARM_JOB_NONE;
        for (i = 0U; i < SB_JOINT_NUM; i++) (void)SB_MoveStop(&s_sb, i);
        (void)snprintf(out, out_max, "OK,ARM,STOP");
        return ARM_OK;
    }

    /* ---------- 解除暂停 ---------- */
    if (strcmp(sub, "RESUME") == 0)
    {
        rc = ARM_OK;
        for (i = 0U; i < SB_JOINT_NUM; i++)
        {
            if (SB_MoveResume(&s_sb, i) != SB_OK) rc = ARM_ERR_FAULT;
        }
        if (rc != ARM_OK) return rc;
        (void)snprintf(out, out_max, "OK,ARM,RESUME");
        return ARM_OK;
    }

    /* ---------- 位置回读 ---------- */
    if (strcmp(sub, "READ") == 0)
    {
        rc = ARM_OK;
        for (i = 0U; i < SB_JOINT_NUM; i++)
        {
            uint16_t pp = 0U;
            int r = SB_ReadPos(&s_sb, i, &pp);
            if (r != SB_OK) { rc = ARM_ERR_FAULT; pp = 0U; }
            pos[i] = pp;
        }
        if (rc != ARM_OK) return rc;     /* 有轴答不上来就如实报 5，不装成功 */
        (void)snprintf(out, out_max, "OK,POS,%u,%u,%u,%u,%u,%u",
                       (unsigned)pos[0], (unsigned)pos[1], (unsigned)pos[2],
                       (unsigned)pos[3], (unsigned)pos[4], (unsigned)pos[5]);
        return ARM_OK;
    }

    /* ---------- 温度回读 ---------- */
    if (strcmp(sub, "TEMP") == 0)
    {
        uint8_t t[SB_JOINT_NUM];
        for (i = 0U; i < SB_JOINT_NUM; i++)
        {
            t[i] = 0U;
            if (SB_ReadTemp(&s_sb, i, &t[i]) != SB_OK) return ARM_ERR_FAULT;
        }
        (void)snprintf(out, out_max, "OK,TEMP,%u,%u,%u,%u,%u,%u",
                       (unsigned)t[0], (unsigned)t[1], (unsigned)t[2],
                       (unsigned)t[3], (unsigned)t[4], (unsigned)t[5]);
        return ARM_OK;
    }

    /* ---------- 母线电压回读（夹爪判握力靠的就是它：堵转时电压会被拉低） ---------- */
    if (strcmp(sub, "VIN") == 0)
    {
        uint16_t mv[SB_JOINT_NUM];
        for (i = 0U; i < SB_JOINT_NUM; i++)
        {
            mv[i] = 0U;
            if (SB_ReadVin(&s_sb, i, &mv[i]) != SB_OK) return ARM_ERR_FAULT;
        }
        (void)snprintf(out, out_max, "OK,VIN,%u,%u,%u,%u,%u,%u",
                       (unsigned)mv[0], (unsigned)mv[1], (unsigned)mv[2],
                       (unsigned)mv[3], (unsigned)mv[4], (unsigned)mv[5]);
        return ARM_OK;
    }

    /* ---------- 清过温锁停 ---------- */
    if (strcmp(sub, "CLRFAULT") == 0)
    {
        if (SB_ClearFault(&s_sb) != SB_OK) return ARM_ERR_FAULT;
        (void)snprintf(out, out_max, "OK,ARM,CLR");
        return ARM_OK;
    }

    return ARM_ERR_UNKNOWN;
}
