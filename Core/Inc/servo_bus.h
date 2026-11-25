/**
 * servo_bus.h  —  幻尔总线舵机接口（HTS-35H / HTD-45H / LX-224(HV) 通用）
 * 适用：J1=HTS-35H(35kg) / J2=HTD-45H(45kg) / J3~J6=LX-224(HV)(20kg)
 *       全部二手件，状态未知，需建档 + 温度基线
 *
 * 协议依据：LX-224 官方手册（CMD 1 = MOVE_TIME_WRITE、CMD 13 = ID_WRITE）
 *   帧 = 0x55 0x55 | ID | LEN | CMD | PARAM... | CHK
 *   LEN = 3 + 参数个数（例：MOVE 4 参数 -> LEN=7；ID_WRITE 1 参数 -> LEN=4）
 *   CHK = ~(ID + LEN + CMD + 所有 PARAM) 按字节累加后取反
 *   波特率 115200，8N1，半双工单线（SIG 既收又发）
 *
 * 位置单位：0~1000 ↔ 0~240°，1 单位 = 0.24°
 *
 * ⚠️ 无机械限位（J1 HTS-35H 实测）：
 *    控制行程 0~1000（240°）；电位器可读范围实际到 ~1200（越程余量 ≈48°，实测最高 1198）；
 *    越过 ~1200 是轨道缺口，回读变成**负数**（0xFFFF = -1，按无符号看就是"6 万多"）。
 *    因此：命令目标统一钳到安全软限位 SB_POS_SAFE_MIN/MAX（10~990）；
 *    回读按有符号解析：0~1200 是真实读数，负数（缺口）作废、保持上一有效值。
 *
 * ⚠️ 电压域警告：LX-224 非 HV 版工作电压 6~8.4V，严禁挂 11.1V 母线；
 *    LX-224HV / HTS-35H / HTD-45H 才是 9~12.6V。上电前逐颗看丝印/标签。
 */
#ifndef __SERVO_BUS_H
#define __SERVO_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ==================== 配置区（按实机改这里） ==================== */

#define SB_JOINT_NUM            6U

/* 总线舵机串口与方向脚（经 Waveshare Bus Servo Adapter (A) 转接）
 * 本模块未加入 Keil 工程（等机械臂阶段再启用），以下为启用时的接线约定：
 *   · USART3(PB10/PB11) 已被 uart_protocol.c 注册为第三控制源，
 *     SB 启用时必须另选串口（如 USART6），不要直接抢占 huart3。
 *   · 转接板拨到 A 档（UART~SERVO），板内已完成 UART↔单线总线方向切换，
 *     主机侧用【全双工 UART】按丝印 TX/RX 同名相接即可，不需要 DE 方向脚；
 *     PB1 的 DE 是早期面向裸 RS485 收发器的预留，接 Waveshare 转接板时不使用。 */
/* 占位符故意不绑定现成句柄：huart3 已被 uart_protocol.c 注册为第三控制源，
   直接绑过去会造成两个驱动抢同一个串口。启用本模块时必须在 CubeMX 新建
   独立 UART（如 USART6），再把下面这行改成该句柄；未改之前编译/链接会报
   未定义符号，这是有意为之的「防误用」闸门。 */
extern UART_HandleTypeDef sb_uart_handle;
#define SB_UART_HANDLE          sb_uart_handle
#define SB_DE_PORT              GPIOB
#define SB_DE_PIN               GPIO_PIN_1      /* 预留；接 Waveshare 转接板时不用 */
#define SB_USE_HW_HALFDUPLEX    0               /* 1=HAL 单线半双工，0=GPIO 控收发器（两者均为旧收发器方案） */

/* 协议帧 */
#define SB_HEAD0                0x55U
#define SB_HEAD1                0x55U
#define SB_CHK_NONE             0
#define SB_CHK_NEG_SUM          1               /* ~(累加和)，幻尔默认 */
#define SB_CHK_MODE             SB_CHK_NEG_SUM

/* 命令字（幻尔/Lobot 总线协议；CMD1、CMD13 已按 LX-224 手册核对） */
#define SB_CMD_MOVE                 1U    /* 位置(2B)+时间(2B)，LEN=7，无回包 */
#define SB_CMD_MOVE_READ            2U    /* 读上次下发的目标 */
#define SB_CMD_MOVE_WAIT            7U    /* 带等待的移动（阻塞型，一般不用） */
#define SB_CMD_MOVE_START          11U    /* 解除 STOP 后的暂停 */
#define SB_CMD_MOVE_STOP           12U    /* 暂停在当前位置 */
#define SB_CMD_ID_WRITE            13U    /* 新 ID(1B)，LEN=4 */
#define SB_CMD_ID_READ             14U    /* 返回 ID(1B) */
#define SB_CMD_ANGLE_LIMIT_WRITE   20U    /* min(2B)+max(2B)，LEN=7，掉电保存 */
#define SB_CMD_ANGLE_LIMIT_READ    21U
#define SB_CMD_VIN_LIMIT_WRITE     22U    /* min(mV)+max(mV)，LEN=7，掉电保存 */
#define SB_CMD_VIN_LIMIT_READ      23U
#define SB_CMD_TEMP_LIMIT_WRITE    24U    /* 过温保护阈值(1B ℃)，LEN=4，掉电保存 */
#define SB_CMD_TEMP_LIMIT_READ     25U
#define SB_CMD_TEMP_READ           26U    /* 返回 温度(1B ℃) */
#define SB_CMD_VIN_READ            27U    /* 返回 电压(2B mV) */
#define SB_CMD_POS_READ            28U    /* 返回 位置(2B 0~1000) */

/* 位置单位 */
#define SB_POS_MIN              0U
#define SB_POS_MID              500U
#define SB_POS_MAX              1000U

/* 读回可信上限（J1 实测最高约 1198，取整 1200）：
   0~1000 控制行程、1000~1200 越程余量，两者都是真实读数；只有负值（轨道缺口）无效 */
#define SB_POS_READ_MAX         1200U

/* 安全软限位（J1 实测）：0 以下即进轨道缺口，1000 以上仅约 200 单位余量。
   命令目标钳到 10~990，给闭环和机械误差留边，绝不主动开进死区。 */
#define SB_POS_SAFE_MIN         10U
#define SB_POS_SAFE_MAX         990U

/* ---------------- 温度保护阈值（℃） ---------------- */
#define SB_T_NORMAL_MAX         60U     /* 正常上限 */
#define SB_T_HOT                70U     /* 高温：降到 40% 速度 */
#define SB_T_FAULT              80U     /* 故障：锁停，拒绝新动作 */
#define SB_T_RECOVER            55U     /* 解除阈值（需保持 30s） */
#define SB_T_RECOVER_HOLD_MS    30000U
#define SB_T_BOOT_WARN          55U     /* 自检基线超过此值 = 疑似个体异常 */
#define SB_T_WRITE_LIMIT        70U     /* 写进舵机内部的过温阈值（永久，慎改） */

/* 降速档位（百分比，参与 MovePos 时间折算） */
#define SB_SCALE_NORMAL         100U
#define SB_SCALE_WARN           70U
#define SB_SCALE_HOT            40U
#define SB_SCALE_HOLD           0U

/* 母线电压门限（mV） */
#define SB_VIN_UNDER_MV         9000U   /* <9.0V 欠压：不再加动作 */
#define SB_VIN_OVER_MV         12600U   /* >12.6V 超压：高压舵机上限 */

/* 轮询节奏 */
#define SB_POLL_PERIOD_MS       1000U
#define SB_VIN_PERIOD_MS        5000U
#define SB_ERR_MAX              3U

/* 返回值 */
#define SB_OK                   0
#define SB_ERR_PARAM           -1
#define SB_ERR_TX              -2
#define SB_ERR_RX              -3
#define SB_ERR_OFFLINE         -4
#define SB_ERR_FAULT_LOCK      -5
#define SB_ERR_LIMIT           -6
#define SB_ERR_POS_RANGE       -7    /* 帧正常，但位置为负（轨道缺口/死区），已保持上一有效值 */

/* ---------------- 温度状态机 ---------------- */
typedef enum {
    SB_TEMP_NORMAL = 0,
    SB_TEMP_WARN,
    SB_TEMP_HOT,
    SB_TEMP_FAULT
} SB_TempState;

/* ---------------- 单关节档案 ---------------- */
typedef struct {
    uint8_t  id;
    uint8_t  online;
    uint16_t pos;
    uint16_t target;
    uint8_t  temp;
    uint16_t vin_mv;        /* 回读母线电压，单位 mV（11.20V -> 11200） */
    uint8_t  err_cnt;
    uint8_t  base_temp;
    uint8_t  peak_temp;
    uint16_t pos_min;
    uint16_t pos_max;
    uint16_t cyc_cnt;
} SB_Joint;

typedef struct {
    SB_Joint  j[SB_JOINT_NUM];
    SB_TempState st;
    uint8_t   speed_scale;
    uint8_t   fault;
    uint32_t  cool_ms;
    uint32_t  tick_ms;
    uint32_t  poll_ms;
    uint32_t  vin_ms;
} SB_Handle;

/* ==================== API ==================== */

void SB_Init(SB_Handle *h);
void SB_SelfTest(SB_Handle *h, uint32_t duration_ms);
void SB_Tick10ms(SB_Handle *h);

int  SB_MovePos(SB_Handle *h, uint8_t joint, uint16_t pos, uint16_t time_ms);
int  SB_MoveAll(SB_Handle *h, const uint16_t pos[SB_JOINT_NUM], uint16_t time_ms);
int  SB_MoveStop(SB_Handle *h, uint8_t joint);
int  SB_MoveResume(SB_Handle *h, uint8_t joint);

int  SB_ReadPos(SB_Handle *h, uint8_t joint, uint16_t *pos);
int  SB_ReadTemp(SB_Handle *h, uint8_t joint, uint8_t *temp);
int  SB_ReadVin(SB_Handle *h, uint8_t joint, uint16_t *vin_mv);
int  SB_ReadID(uint8_t id_on_bus, uint8_t *real_id);

int  SB_DumpRaw(SB_Handle *h, uint8_t joint);
int  SB_ScanIDs(int *ids, uint8_t max_num);
int  SB_SetID(uint8_t old_id, uint8_t new_id);

/* 把保护阈值写进舵机内部（掉电保存，写错会永久生效，先看手册） */
int  SB_WriteTempLimit(uint8_t id, uint8_t temp_c);
int  SB_WriteAngleLimit(uint8_t id, uint16_t min_pos, uint16_t max_pos);  /* 入参自动钳到 10~990 */
int  SB_WriteVinLimit(uint8_t id, uint16_t min_mv, uint16_t max_mv);

SB_TempState SB_GetTempState(const SB_Handle *h);
uint8_t      SB_GetSpeedScale(const SB_Handle *h);
uint8_t      SB_GetPeakTemp(const SB_Handle *h, uint8_t joint);
int          SB_ClearFault(SB_Handle *h);

void SB_SetReportHook(void (*fn)(const char *line));

#ifdef __cplusplus
}
#endif
#endif /* __SERVO_BUS_H */
