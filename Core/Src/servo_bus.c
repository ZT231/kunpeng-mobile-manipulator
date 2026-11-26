/**
 * servo_bus.c  —  幻尔总线舵机驱动 + 温度轮询/阈值保护
 * 配套 servo_bus.h；STM32F407VET6 + HAL + FreeRTOS
 *
 * 协议（LX-224 手册核对）：
 *   0x55 0x55 | ID | LEN | CMD | PARAM... | CHK
 *   LEN = 3 + 参数个数；CHK = ~(ID+LEN+CMD+PARAM...) 逐字节累加取反
 *   115200 8N1，半双工单线
 *
 * 【上板顺序】
 *   1) SB_Init(&h);
 *   2) SB_DumpRaw(&h, 0);        // 抓原始字节，确认帧长/校验对得上
 *   3) SB_ScanIDs(ids, 6);       // 二手件先查 ID 冲突
 *   4) SB_SelfTest(&h, 30000);   // 30s 空载建温度基线
 *   5) 10ms 任务里 SB_Tick10ms(&h)
 *
 * 【温度策略】
 *   <60℃ 100% / 60~70℃ 70% / 70~80℃ 40% / ≥80℃ 锁停
 *   回落 <55℃ 且保持 30s 才能 SB_ClearFault() 复位到 70% 档
 *
 * 【位置安全（J1 HTS-35H 实测）】
 *   无机械限位：0~1000=240° 控制行程；1000~1200 越程余量（真实读数，实测最高 1198）；
 *   越过 ~1200 是轨道缺口，回读变负数（0xFFFF=-1）。驱动层：
 *     命令目标钳到 10~990（SB_POS_SAFE_MIN/MAX）；
 *     回读按有符号 16 位解析，0~1200 可信，负数（缺口）作废、保持上一有效值，
 *     在线状态不受影响（有应答就是在线，缺口不算掉线）。
 *
 * 【单侧逼近背隙补偿（J5 HTS-20H 实测背隙 3.6°=15 单位）】
 *   该单轴舵机换向到位会差约 15 单位（从下逼近 492、从上逼近 506）。
 *   SB_MovePos 对开启补偿的关节做两段式：先越过目标 24 单位，再反向慢速逼近，
 *   保证每次都从同一侧啮合，回差被吃掉。越过量必须 > 背隙，否则齿轮翻不到另一侧。
 *   补偿是阻塞两段式，只在单关节 SB_MovePos 生效；SB_MoveAll 走原路径不补偿，
 *   避免破坏多轴同步（J5 要精确到位时单独调 SB_MovePos）。
 */
#include "servo_bus.h"
#include <stdio.h>
#include <string.h>

/* ================= 内部状态 ================= */
#define SB_TX_TIMEOUT_MS   5U
#define SB_RX_TIMEOUT_MS   8U
#define SB_FRAME_MAX       16U
#define SB_POLL_MAX_MS     40U     /* 一轮轮询最多占用时间，超时算掉线 */

static UART_HandleTypeDef *s_uart = NULL;
static GPIO_TypeDef       *s_de_port = NULL;
static uint16_t            s_de_pin = 0U;
static void              (*s_hook)(const char *line) = NULL;
static uint8_t             s_scale = SB_SCALE_NORMAL;
static osMutexId_t         s_bus_lock = NULL;   /* 总线互斥锁，见 SB_SetBusLock */

/* ================= 底层 ================= */

static void sb_de_tx(void)
{
#if !SB_USE_HW_HALFDUPLEX
    if (s_de_port) HAL_GPIO_WritePin(s_de_port, s_de_pin, GPIO_PIN_SET);
#endif
}

static void sb_de_rx(void)
{
#if !SB_USE_HW_HALFDUPLEX
    if (s_de_port) HAL_GPIO_WritePin(s_de_port, s_de_pin, GPIO_PIN_RESET);
#endif
}

#if SB_CHK_MODE != SB_CHK_NONE
/* 幻尔默认：从 ID 开始逐字节累加，然后取反（按字节） */
static uint8_t sb_chk(const uint8_t *buf, uint8_t n)
{
    uint8_t i, s = 0U;
    for (i = 0U; i < n; i++) { s = (uint8_t)(s + buf[i]); }
    return (uint8_t)(~s);
}
#endif

/* 组帧发送：LEN = 3 + plen。整段（组帧→发送→换向余量）在总线锁内执行，
   避免与 SB_MoveStop/轮询等在另一个任务里并发抢 UART 导致 HAL_BUSY 丢帧。 */
static int sb_send(uint8_t id, uint8_t cmd, const uint8_t *par, uint8_t plen)
{
    uint8_t f[SB_FRAME_MAX];
    uint8_t n = 0U, i, lenv;
    HAL_StatusTypeDef st;

    if (!s_uart) return SB_ERR_PARAM;
    if ((uint16_t)plen + 6U > SB_FRAME_MAX) return SB_ERR_PARAM;

    lenv = (uint8_t)(plen + 3U);

    f[n++] = SB_HEAD0;
    f[n++] = SB_HEAD1;
    f[n++] = id;
    f[n++] = lenv;
    f[n++] = cmd;
    for (i = 0U; i < plen; i++) { f[n++] = par[i]; }
#if SB_CHK_MODE != SB_CHK_NONE
    f[n] = sb_chk(&f[2], lenv);
    n++;
#endif

    if (s_bus_lock) (void)osMutexAcquire(s_bus_lock, osWaitForever);

    sb_de_tx();
    st = HAL_UART_Transmit(s_uart, f, n, SB_TX_TIMEOUT_MS);
    sb_de_rx();
    HAL_Delay(1);                  /* 换向余量 */

    if (s_bus_lock) (void)osMutexRelease(s_bus_lock);

    return (st == HAL_OK) ? SB_OK : SB_ERR_TX;
}

/* 收响应：参数个数 = LEN - 3。整段接收状态机在总线锁内执行。 */
static int sb_recv(uint8_t want_id, uint8_t want_cmd,
                   uint8_t *data, uint8_t data_max, uint8_t *data_len)
{
    uint8_t st = 0U;               /* 0=找帧头1 1=找帧头2 2=ID 3=LEN 4=CMD 5=DATA 6=CHK */
    uint8_t id = 0U, lenv = 0U, cmd = 0U, got = 0U, par = 0U;
    uint8_t sum = 0U, chk = 0U;
    uint32_t t0;
    int result = SB_ERR_RX;

    if (!s_uart) return SB_ERR_PARAM;

    if (s_bus_lock) (void)osMutexAcquire(s_bus_lock, osWaitForever);

    t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < SB_RX_TIMEOUT_MS) {
        uint8_t c;
        if (HAL_UART_Receive(s_uart, &c, 1U, 1U) != HAL_OK) continue;

        switch (st) {
        case 0U:
            if (c == SB_HEAD0) st = 1U;
            break;
        case 1U:
            if (c == SB_HEAD1)      st = 2U;
            else if (c == SB_HEAD0) st = 1U;
            else                    st = 0U;
            break;
        case 2U:
            id = c; sum = c; st = 3U;
            break;
        case 3U:
            lenv = c;
            if (lenv < 3U || lenv > 12U) {
                st = 0U; id = 0U; lenv = 0U; cmd = 0U; got = 0U; par = 0U; sum = 0U;
                break;
            }
            sum = (uint8_t)(sum + c);
            par = (uint8_t)(lenv - 3U);
            st = 4U;
            break;
        case 4U:
            cmd = c;
            sum = (uint8_t)(sum + c);
            got = 0U;
            st = (par == 0U) ? 6U : 5U;
            break;
        case 5U:
            if (data && got < data_max) data[got] = c;
            got++;
            sum = (uint8_t)(sum + c);
            if (got >= par) st = 6U;
            break;
        case 6U:
            chk = c;
#if SB_CHK_MODE != SB_CHK_NONE
        {
            uint8_t calc = (uint8_t)(~(uint8_t)sum);
            if (chk != calc) {
                st = 0U; id = 0U; lenv = 0U; cmd = 0U; got = 0U; par = 0U; sum = 0U;
                break;
            }
        }
#endif
            if (data_len) *data_len = got;
            if (id == want_id && cmd == want_cmd) { result = SB_OK; }
            st = 0U; id = 0U; lenv = 0U; cmd = 0U; got = 0U; par = 0U; sum = 0U;
            break;
        default:
            st = 0U;
            break;
        }

        if (result == SB_OK) break;
    }

    if (s_bus_lock) (void)osMutexRelease(s_bus_lock);

    return result;
}

static int sb_query(uint8_t id, uint8_t cmd,
                    const uint8_t *par, uint8_t plen,
                    uint8_t *data, uint8_t data_max, uint8_t *data_len)
{
    int r = sb_send(id, cmd, par, plen);
    if (r != SB_OK) return r;
    return sb_recv(id, cmd, data, data_max, data_len);
}

/* ---- 位置解析与有效性（无机械限位机种的死区防护） ---- */

/* 位置按有符号 16 位解释：0xFFFF 就是 -1（舵机转进轨道缺口后的回读） */
static int16_t sb_parse_pos16(const uint8_t *d)
{
    return (int16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8));
}

/* 回读有效性：0~1200 是真实读数（0~1000 控制行程、1000~1200 越程余量）；
   负值 = 轨道缺口（死区），作废、保持上一有效值 */
static int sb_pos_valid(int16_t p)
{
    return (p >= 0 && p <= (int16_t)SB_POS_READ_MAX);
}

/* ================= 降速折算 ================= */
static uint16_t sb_scale_time(uint16_t time_ms)
{
    uint32_t v;
    if (s_scale == SB_SCALE_HOLD) return 0xFFFFU;
    if (time_ms == 0U)            return 0U;
    v = ((uint32_t)time_ms * 100U) / (uint32_t)s_scale;
    return (v > 60000U) ? (uint16_t)60000U : (uint16_t)v;
}

/* ================= 上报 ================= */
static void sb_report(const SB_Handle *h)
{
    char buf[96];
    if (!s_hook) return;
    (void)snprintf(buf, sizeof(buf),
                   "$BUS,ST=%u,T=%u,%u,%u,%u,%u,%u,V=%u",
                   (unsigned)h->st,
                   (unsigned)h->j[0].temp, (unsigned)h->j[1].temp,
                   (unsigned)h->j[2].temp, (unsigned)h->j[3].temp,
                   (unsigned)h->j[4].temp, (unsigned)h->j[5].temp,
                   (unsigned)h->j[0].vin_mv);
    s_hook(buf);
}

/* ================= 轮询 ================= */
static void sb_poll_once(SB_Handle *h, uint8_t read_vin)
{
    uint8_t i;
    for (i = 0U; i < SB_JOINT_NUM; i++) {
        SB_Joint *j = &h->j[i];
        uint8_t d[4], len = 0U;

        /* 位置：CMD 28，返回 2 字节；按有符号解析，负值（轨道缺口）作废 */
        len = 0U;
        if (sb_query(j->id, SB_CMD_POS_READ, NULL, 0U, d, sizeof(d), &len) == SB_OK && len >= 2U) {
            int16_t p = sb_parse_pos16(d);
            j->online = 1U;             /* 有应答就是在线，缺口不算掉线 */
            if (sb_pos_valid(p)) {
                j->pos = (uint16_t)p;   /* 无效读数不覆盖，保持上一有效值 */
                j->err_cnt = 0U;
            }
        } else if (j->err_cnt < 250U) {
            j->err_cnt++;
            if (j->err_cnt >= SB_ERR_MAX) { j->online = 0U; }
        }

        /* 温度：CMD 26，返回 1 字节 ℃ */
        len = 0U;
        if (sb_query(j->id, SB_CMD_TEMP_READ, NULL, 0U, d, sizeof(d), &len) == SB_OK && len >= 1U) {
            j->temp = d[0];
            j->online = 1U;
            if (j->temp > j->peak_temp) j->peak_temp = j->temp;
        }

        /* 电压：CMD 27，返回 2 字节 mV */
        if (read_vin) {
            len = 0U;
            if (sb_query(j->id, SB_CMD_VIN_READ, NULL, 0U, d, sizeof(d), &len) == SB_OK && len >= 2U) {
                j->vin_mv = (uint16_t)(d[0] | ((uint16_t)d[1] << 8));
            }
        }
    }
}

/* ================= 温度状态机 ================= */
static void sb_temp_fsm(SB_Handle *h)
{
    uint8_t i, mx = 0U, any_online = 0U;

    for (i = 0U; i < SB_JOINT_NUM; i++) {
        if (h->j[i].online) {
            any_online = 1U;
            if (h->j[i].temp > mx) mx = h->j[i].temp;
        }
    }
    if (!any_online) return;       /* 全掉线：保持档位，让上层报错 */

    switch (h->st) {
    case SB_TEMP_NORMAL:
        if (mx >= SB_T_FAULT)           h->st = SB_TEMP_FAULT;
        else if (mx >= SB_T_HOT)        h->st = SB_TEMP_HOT;
        else if (mx >= SB_T_NORMAL_MAX) h->st = SB_TEMP_WARN;
        break;
    case SB_TEMP_WARN:
        if (mx >= SB_T_FAULT)     h->st = SB_TEMP_FAULT;
        else if (mx >= SB_T_HOT)  h->st = SB_TEMP_HOT;
        else if (mx < SB_T_RECOVER) h->st = SB_TEMP_NORMAL;
        break;
    case SB_TEMP_HOT:
        if (mx >= SB_T_FAULT)                 h->st = SB_TEMP_FAULT;
        else if (mx < (SB_T_NORMAL_MAX - 5U)) h->st = SB_TEMP_WARN;
        break;
    case SB_TEMP_FAULT:
        if (mx < SB_T_RECOVER) {
            if (h->cool_ms < 600000U) h->cool_ms += SB_POLL_PERIOD_MS;
        } else {
            h->cool_ms = 0U;
        }
        break;
    default:
        h->st = SB_TEMP_NORMAL;
        break;
    }

    switch (h->st) {
    case SB_TEMP_NORMAL: s_scale = SB_SCALE_NORMAL; h->fault = 0U; break;
    case SB_TEMP_WARN:   s_scale = SB_SCALE_WARN;   h->fault = 0U; break;
    case SB_TEMP_HOT:    s_scale = SB_SCALE_HOT;    h->fault = 0U; break;
    case SB_TEMP_FAULT:  s_scale = SB_SCALE_HOLD;   h->fault = 1U; break;
    default:             s_scale = SB_SCALE_NORMAL; h->fault = 0U; break;
    }
    h->speed_scale = s_scale;
}

/* ================= 对外 ================= */

void SB_SetReportHook(void (*fn)(const char *line))
{
    s_hook = fn;
}

void SB_SetBusLock(osMutexId_t lock)
{
    s_bus_lock = lock;
}

void SB_Init(SB_Handle *h)
{
    uint8_t i;
    static const uint16_t home[SB_JOINT_NUM] = SB_HOME_POS_LIST;

    if (!h) return;
    memset(h, 0, sizeof(SB_Handle));

    s_uart = &SB_UART_HANDLE;
    s_de_port = SB_DE_PORT;
    s_de_pin = SB_DE_PIN;
    s_scale = SB_SCALE_NORMAL;
    sb_de_rx();

    for (i = 0U; i < SB_JOINT_NUM; i++) {
        h->j[i].id = (uint8_t)(i + 1U);
        h->j[i].pos_min = SB_POS_SAFE_MIN;   /* 无机械限位机种：软限位 10~990 */
        h->j[i].pos_max = SB_POS_SAFE_MAX;   /* TODO: 新结构到货后按实测再收紧 */
        h->j[i].target = home[i];            /* 上电目标 = 六轴零位 */
        h->j[i].pos = home[i];
        h->j[i].bl_en = 0U;
        h->j[i].bl_dir = 1;
    }
    /* J6 夹爪：300=全张 / 674=全闭（实测软限位），盖掉上面的通用 10~990 */
    h->j[5].pos_min = SB_J6_POS_MIN;
    h->j[5].pos_max = SB_J6_POS_MAX;

    /* J5(index 4) 用 HTS-20H，实测背隙 3.6°(15 单位)，默认开启单侧逼近补偿 */
    h->j[4].bl_en = 1U;
    h->j[4].bl_dir = 1;
    h->st = SB_TEMP_NORMAL;
    h->speed_scale = SB_SCALE_NORMAL;
}

int SB_ReadPos(SB_Handle *h, uint8_t joint, uint16_t *pos)
{
    uint8_t d[4], len = 0U;
    int16_t p;
    if (!h || joint >= SB_JOINT_NUM || !pos) return SB_ERR_PARAM;
    if (sb_query(h->j[joint].id, SB_CMD_POS_READ, NULL, 0U, d, sizeof(d), &len) != SB_OK || len < 2U)
        return SB_ERR_RX;
    p = sb_parse_pos16(d);
    if (!sb_pos_valid(p))
        return SB_ERR_POS_RANGE;    /* 死区读数：不覆盖 *pos，也不更新档案 */
    *pos = (uint16_t)p;
    h->j[joint].pos = *pos;
    return SB_OK;
}

int SB_ReadTemp(SB_Handle *h, uint8_t joint, uint8_t *temp)
{
    uint8_t d[4], len = 0U;
    if (!h || joint >= SB_JOINT_NUM || !temp) return SB_ERR_PARAM;
    if (sb_query(h->j[joint].id, SB_CMD_TEMP_READ, NULL, 0U, d, sizeof(d), &len) != SB_OK || len < 1U)
        return SB_ERR_RX;
    *temp = d[0];
    h->j[joint].temp = *temp;
    if (*temp > h->j[joint].peak_temp) h->j[joint].peak_temp = *temp;
    return SB_OK;
}

int SB_ReadVin(SB_Handle *h, uint8_t joint, uint16_t *vin_mv)
{
    uint8_t d[4], len = 0U;
    if (!h || joint >= SB_JOINT_NUM || !vin_mv) return SB_ERR_PARAM;
    if (sb_query(h->j[joint].id, SB_CMD_VIN_READ, NULL, 0U, d, sizeof(d), &len) != SB_OK || len < 2U)
        return SB_ERR_RX;
    *vin_mv = (uint16_t)(d[0] | ((uint16_t)d[1] << 8));
    h->j[joint].vin_mv = *vin_mv;
    return SB_OK;
}

int SB_ReadID(uint8_t id_on_bus, uint8_t *real_id)
{
    uint8_t d[4], len = 0U;
    if (!real_id) return SB_ERR_PARAM;
    if (sb_query(id_on_bus, SB_CMD_ID_READ, NULL, 0U, d, sizeof(d), &len) != SB_OK || len < 1U)
        return SB_ERR_RX;
    *real_id = d[0];
    return SB_OK;
}

/* 原始单点移动（不含背隙补偿）：软限位钳位 + 降速折算 + 发送 */
static int sb_move_raw(SB_Joint *j, uint16_t pos, uint16_t time_ms)
{
    uint8_t par[4];
    uint16_t t;
    int r;

    /* 安全软限位优先：绝不主动开进电位器死区（0 以下即缺口） */
    if (pos < SB_POS_SAFE_MIN) pos = SB_POS_SAFE_MIN;
    if (pos > SB_POS_SAFE_MAX) pos = SB_POS_SAFE_MAX;
    if (pos < j->pos_min) pos = j->pos_min;
    if (pos > j->pos_max) pos = j->pos_max;

    t = sb_scale_time(time_ms);
    if (t == 0xFFFFU) return SB_ERR_FAULT_LOCK;

    par[0] = (uint8_t)(pos & 0xFFU);
    par[1] = (uint8_t)(pos >> 8);
    par[2] = (uint8_t)(t & 0xFFU);
    par[3] = (uint8_t)(t >> 8);

    r = sb_send(j->id, SB_CMD_MOVE, par, 4U);   /* 移动命令无回包 */
    if (r == SB_OK) {
        j->target = pos;
        if (j->cyc_cnt < 65000U) j->cyc_cnt++;
    }
    return r;
}

int SB_MovePos(SB_Handle *h, uint8_t joint, uint16_t pos, uint16_t time_ms)
{
    SB_Joint *j;
    uint16_t over;
    int r;

    if (!h || joint >= SB_JOINT_NUM) return SB_ERR_PARAM;
    if (h->fault)                    return SB_ERR_FAULT_LOCK;
    j = &h->j[joint];

    /* 先钳位目标，保证 overshoot 计算基于合法值（防 uint16 回绕） */
    if (pos < SB_POS_SAFE_MIN) pos = SB_POS_SAFE_MIN;
    if (pos > SB_POS_SAFE_MAX) pos = SB_POS_SAFE_MAX;
    if (pos < j->pos_min) pos = j->pos_min;
    if (pos > j->pos_max) pos = j->pos_max;

    /* 单侧逼近背隙补偿：先越过目标、再反向逼近。
     * 越过量必须 > 背隙（J5 实测 15 单位），否则反向时齿轮翻不到另一侧齿面。 */
    if (j->bl_en) {
        if (j->bl_dir >= 0) {
            /* 从上逼近（最终负向）：先走到目标上方 */
            over = pos + SB_BL_OVERSHOOT;
            if (over > SB_POS_SAFE_MAX) over = SB_POS_SAFE_MAX;
        } else {
            /* 从下逼近（最终正向）：先走到目标下方 */
            over = (pos > SB_BL_OVERSHOOT) ? (uint16_t)(pos - SB_BL_OVERSHOOT) : SB_POS_SAFE_MIN;
        }
        r = sb_move_raw(j, over, time_ms);
        if (r != SB_OK) return r;
        HAL_Delay(time_ms + SB_BL_SETTLE_MS);        /* 等越过点到位 */
        return sb_move_raw(j, pos, SB_BL_FINAL_MS);  /* 反向慢速逼近目标 */
    }

    return sb_move_raw(j, pos, time_ms);
}

int SB_MoveAll(SB_Handle *h, const uint16_t pos[SB_JOINT_NUM], uint16_t time_ms)
{
    uint8_t i;
    int r;
    if (!h || !pos) return SB_ERR_PARAM;
    if (h->fault)   return SB_ERR_FAULT_LOCK;

    /* 半双工总线，逐颗下发；同一 time_ms 让它们尽量同步起停。
     * 不做单侧逼近补偿：补偿是阻塞两段式，插在循环里会破坏多轴同步；
     * J5 要精确到位时单独调 SB_MovePos。 */
    for (i = 0U; i < SB_JOINT_NUM; i++) {
        r = sb_move_raw(&h->j[i], pos[i], time_ms);
        if (r != SB_OK) return r;
    }
    return SB_OK;
}

int SB_MoveStop(SB_Handle *h, uint8_t joint)
{
    if (!h || joint >= SB_JOINT_NUM) return SB_ERR_PARAM;
    return sb_send(h->j[joint].id, SB_CMD_MOVE_STOP, NULL, 0U);
}

int SB_MoveResume(SB_Handle *h, uint8_t joint)
{
    if (!h || joint >= SB_JOINT_NUM) return SB_ERR_PARAM;
    if (h->fault) return SB_ERR_FAULT_LOCK;
    return sb_send(h->j[joint].id, SB_CMD_MOVE_START, NULL, 0U);
}

void SB_Tick10ms(SB_Handle *h)
{
    uint8_t read_vin;

    if (!h) return;
    h->tick_ms += 10U;

    if ((h->tick_ms - h->poll_ms) >= SB_POLL_PERIOD_MS) {
        h->poll_ms = h->tick_ms;
        read_vin = ((h->tick_ms - h->vin_ms) >= SB_VIN_PERIOD_MS) ? 1U : 0U;
        if (read_vin) h->vin_ms = h->tick_ms;

        sb_poll_once(h, read_vin);
        sb_temp_fsm(h);
        sb_report(h);
    }
    /* 一轮约 12~18 帧、阻塞 25~35ms，建议放独立 FreeRTOS 任务，别和电机 PID 挤一起 */
}

SB_TempState SB_GetTempState(const SB_Handle *h)
{
    return h ? h->st : SB_TEMP_FAULT;
}

uint8_t SB_GetSpeedScale(const SB_Handle *h)
{
    return h ? h->speed_scale : SB_SCALE_HOLD;
}

uint8_t SB_GetPeakTemp(const SB_Handle *h, uint8_t joint)
{
    if (!h || joint >= SB_JOINT_NUM) return 0U;
    return h->j[joint].peak_temp;
}

int SB_ClearFault(SB_Handle *h)
{
    uint8_t i;
    if (!h) return SB_ERR_PARAM;
    if (!h->fault) return SB_OK;

    for (i = 0U; i < SB_JOINT_NUM; i++) {
        if (h->j[i].online && h->j[i].temp >= SB_T_RECOVER) return SB_ERR_FAULT_LOCK;
    }
    if (h->cool_ms < SB_T_RECOVER_HOLD_MS) return SB_ERR_FAULT_LOCK;

    h->fault = 0U;
    h->st = SB_TEMP_WARN;          /* 先回 70% 档 */
    h->cool_ms = 0U;
    s_scale = SB_SCALE_WARN;
    h->speed_scale = s_scale;
    return SB_OK;
}

void SB_SelfTest(SB_Handle *h, uint32_t duration_ms)
{
    uint32_t t0;
    uint16_t sum[SB_JOINT_NUM];
    uint8_t  mx[SB_JOINT_NUM];
    uint16_t n = 0U;
    uint8_t  i;

    if (!h) return;
    memset(sum, 0, sizeof(sum));
    memset(mx, 0, sizeof(mx));

    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < duration_ms) {
        for (i = 0U; i < SB_JOINT_NUM; i++) {
            uint8_t t = 0U;
            if (SB_ReadTemp(h, i, &t) == SB_OK) {
                sum[i] = (uint16_t)(sum[i] + t);
                if (t > mx[i]) mx[i] = t;
            }
        }
        n++;
        HAL_Delay(SB_POLL_PERIOD_MS);
    }

    if (n == 0U) n = 1U;
    for (i = 0U; i < SB_JOINT_NUM; i++) {
        h->j[i].base_temp = (uint8_t)(sum[i] / n);
        h->j[i].peak_temp = mx[i];
    }

    if (s_hook) {
        char buf[96];
        (void)snprintf(buf, sizeof(buf),
                       "$BUS,BOOT,base=%u,%u,%u,%u,%u,%u,peak=%u,%u,%u,%u,%u,%u",
                       (unsigned)h->j[0].base_temp, (unsigned)h->j[1].base_temp,
                       (unsigned)h->j[2].base_temp, (unsigned)h->j[3].base_temp,
                       (unsigned)h->j[4].base_temp, (unsigned)h->j[5].base_temp,
                       (unsigned)mx[0], (unsigned)mx[1], (unsigned)mx[2],
                       (unsigned)mx[3], (unsigned)mx[4], (unsigned)mx[5]);
        s_hook(buf);
    }
}

int SB_ScanIDs(int *ids, uint8_t max_num)
{
    uint8_t id;
    int found = 0;

    if (!ids || max_num == 0U) return 0;
    for (id = 1U; id <= 20U && found < (int)max_num; id++) {
        uint8_t d[4], len = 0U;
        /* 用 ID_READ 而不是 TEMP_READ：发错 ID 时不会有别的舵机代替应答 */
        if (sb_query(id, SB_CMD_ID_READ, NULL, 0U, d, sizeof(d), &len) == SB_OK && len >= 1U) {
            ids[found++] = (int)id;
        }
    }
    return found;
}

int SB_SetID(uint8_t old_id, uint8_t new_id)
{
    uint8_t par = new_id;
    return sb_send(old_id, SB_CMD_ID_WRITE, &par, 1U);
}

int SB_SetBacklash(SB_Handle *h, uint8_t joint, uint8_t enable, int8_t dir)
{
    if (!h || joint >= SB_JOINT_NUM) return SB_ERR_PARAM;
    h->j[joint].bl_en = enable ? 1U : 0U;
    h->j[joint].bl_dir = (dir >= 0) ? 1 : -1;
    return SB_OK;
}

int SB_WriteAngleOffset(uint8_t id, int8_t offset)
{
    uint8_t par;
    if (offset > 125) offset = 125;
    if (offset < -125) offset = -125;
    par = (uint8_t)offset;
    return sb_send(id, SB_CMD_ANGLE_OFFSET_WRITE, &par, 1U);
}

int SB_ReadAngleOffset(uint8_t id, int8_t *offset)
{
    uint8_t d[4], len = 0U;
    if (!offset) return SB_ERR_PARAM;
    if (sb_query(id, SB_CMD_ANGLE_OFFSET_READ, NULL, 0U, d, sizeof(d), &len) != SB_OK || len < 1U)
        return SB_ERR_RX;
    *offset = (int8_t)d[0];
    return SB_OK;
}

int SB_ResetAngleOffset(uint8_t id)
{
    /* 无独立复位命令：写 0 即复位（2026-11-26 实测确认 17=写、19=读） */
    return SB_WriteAngleOffset(id, 0);
}

int SB_WriteTempLimit(uint8_t id, uint8_t temp_c)
{
    uint8_t par = temp_c;
    return sb_send(id, SB_CMD_TEMP_LIMIT_WRITE, &par, 1U);
}

int SB_WriteAngleLimit(uint8_t id, uint16_t min_pos, uint16_t max_pos)
{
    uint8_t par[4];
    /* 写入是掉电保存的，钳到安全限位，防止把死区写进舵机内部 */
    if (min_pos < SB_POS_SAFE_MIN) min_pos = SB_POS_SAFE_MIN;
    if (max_pos > SB_POS_SAFE_MAX) max_pos = SB_POS_SAFE_MAX;
    if (min_pos > max_pos) return SB_ERR_PARAM;
    par[0] = (uint8_t)(min_pos & 0xFFU);
    par[1] = (uint8_t)(min_pos >> 8);
    par[2] = (uint8_t)(max_pos & 0xFFU);
    par[3] = (uint8_t)(max_pos >> 8);
    return sb_send(id, SB_CMD_ANGLE_LIMIT_WRITE, par, 4U);
}

int SB_WriteVinLimit(uint8_t id, uint16_t min_mv, uint16_t max_mv)
{
    uint8_t par[4];
    if (min_mv > max_mv) return SB_ERR_PARAM;
    par[0] = (uint8_t)(min_mv & 0xFFU);
    par[1] = (uint8_t)(min_mv >> 8);
    par[2] = (uint8_t)(max_mv & 0xFFU);
    par[3] = (uint8_t)(max_mv >> 8);
    return sb_send(id, SB_CMD_VIN_LIMIT_WRITE, par, 4U);
}

int SB_DumpRaw(SB_Handle *h, uint8_t joint)
{
    uint8_t id, d[4], len = 0U;
    uint32_t t0;
    char line[64];
    int n;

    if (!h || joint >= SB_JOINT_NUM) return SB_ERR_PARAM;
    id = h->j[joint].id;

    n = sb_send(id, SB_CMD_POS_READ, NULL, 0U);
    if (s_hook) {
        (void)snprintf(line, sizeof(line), "$DBG,TX id=%u cmd=%u,%d", (unsigned)id,
                       (unsigned)SB_CMD_POS_READ, n);
        s_hook(line);
    }

    /* 20ms 内把收到的每个字节按十六进制打出来，对照手册逐字节比 */
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 20U) {
        uint8_t c;
        if (HAL_UART_Receive(s_uart, &c, 1U, 1U) == HAL_OK && s_hook) {
            (void)snprintf(line, sizeof(line), "$DBG,RX 0x%02X", (unsigned)c);
            s_hook(line);
        }
    }

    len = 0U;
    if (sb_query(id, SB_CMD_POS_READ, NULL, 0U, d, sizeof(d), &len) == SB_OK) {
        int16_t p = (len >= 2U) ? sb_parse_pos16(d) : 0;
        if (s_hook) {
            (void)snprintf(line, sizeof(line), "$DBG,OK id=%u n=%u pos=%d%s",
                           (unsigned)id, (unsigned)len, (int)p,
                           sb_pos_valid(p) ? "" : " (DEADZONE)");
            s_hook(line);
        }
        return sb_pos_valid(p) ? SB_OK : SB_ERR_POS_RANGE;
    }
    return SB_ERR_RX;
}
