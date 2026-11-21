/*
 * ============================================================================
 * test_arm_firmware.c —— 机械臂固件核心逻辑的「电脑端」host 单元测试
 * ============================================================================
 *
 * 【为什么要有这个文件】arm_control.c（状态机）与 uart_protocol.c（协议）
 *   是纯逻辑：不碰寄存器、不碰真舵机。所以能在 PC 上用 gcc 编译跑，
 *   验证「FAULT 状态机」和「错误码」这两块刚改过的逻辑对不对——
 *   比烧进板子拿串口助手一条条敲快得多，也能在编译进 Keil 之前先抓错。
 *
 * 【规范定义】host-side unit test + 测试替身（test double）：
 *   用假头文件（stubs/）顶掉 STM32 HAL 和 FreeRTOS，用假函数顶掉
 *   PCA9685 / 示教 / ACS712 / 串口收发，只测被测模块自己的判断。
 *
 * 【关键手法】#include "arm_control.c" / "uart_protocol.c" 把源码「展开」进
 *   本文件，这样它们内部的 static 函数（state_is_frozen / proto_dispatch 等）
 *   对测试代码可见，才能直接断言。这是 C 单测的常见做法（Unity 同理）。
 *
 * 编译（在项目根 RoboticArm_Kunpeng/ 下，Git Bash）：
 *   gcc -std=c99 -Wall -Wextra \
 *       -I Core/tests/stubs -I Core/Inc \
 *       Core/tests/test_arm_firmware.c -o Core/tests/build/test_arm_firmware.exe
 *   ./Core/tests/build/test_arm_firmware.exe
 * ============================================================================
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* 先引入假头文件：让 I2C_HandleTypeDef / osMutexId_t / HAL_StatusTypeDef 等
 * 类型在下面的测试替身定义之前就可用。stubs/ 目录在 -I 最前，会优先命中。 */
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "safety.h"   /* SAFETY_FAULT_OVERCURRENT 故障码常量 */

/* ============================================================
 * 一、测试替身：顶掉所有硬件 / FreeRTOS / 外设依赖
 * ============================================================ */

/* 假 I2C 句柄（i2c.h 里 extern I2C_HandleTypeDef hi2c1） */
I2C_HandleTypeDef hi2c1;

/* 假互斥量（freertos.c 定义、uart_protocol.c extern 引用） */
osMutexId_t print_lock;
osMutexId_t arm_lock;

/* ---- 假 PCA9685：记录最后一次写的通道与脉宽，恒返回成功 ---- */
static uint8_t  g_pca_ch;
static uint16_t g_pca_pulse;
HAL_StatusTypeDef PCA9685_SetServoPulse(I2C_HandleTypeDef *hi2c,
                                        uint8_t channel, uint16_t pulse_us)
{
    (void)hi2c;
    g_pca_ch = channel;
    g_pca_pulse = pulse_us;
    return HAL_OK;
}

/* ---- 假示教模块：点数固定返回 3，其余空操作 ---- */
void Teach_Init(void) {}
void Teach_StartRecord(void) {}
void Teach_StopRecord(void) {}
void Teach_StartPlayback(void) {}
void Teach_Clear(void) {}
void Teach_Tick(void) {}
uint16_t Teach_GetPointCount(void) { return 3U; }

/* ---- 假 ACS712：电流固定 100mA，电压 2500mV ---- */
HAL_StatusTypeDef ACS712_Init(void) { return HAL_OK; }
void ACS712_CalibrateZero(void) {}
int32_t ACS712_Read_mA(void) { return 100; }
uint32_t ACS712_GetLast_mV(void) { return 2500U; }
uint32_t ACS712_GetZero_mV(void) { return 2500U; }

/* ---- 假 FreeRTOS 互斥：单线程，恒成功 ---- */
int osMutexAcquire(osMutexId_t m, uint32_t timeout) { (void)m; (void)timeout; return 0; }
int osMutexRelease(osMutexId_t m) { (void)m; return 0; }
void osDelay(uint32_t ms) { (void)ms; }

/* ---- 假串口发送：把发出的字节截进全局缓冲，供测试断言应答帧 ---- */
static char g_tx[256];
static uint16_t g_tx_len;
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart,
                                    const uint8_t *pData,
                                    uint16_t Size, uint32_t Timeout)
{
    (void)huart; (void)Timeout;
    if (Size >= sizeof(g_tx)) Size = (uint16_t)(sizeof(g_tx) - 1U);
    memcpy(g_tx, pData, Size);
    g_tx[Size] = '\0';
    g_tx_len = Size;
    return HAL_OK;
}

/* ---- 假串口接收 / 中断 / 时钟 ---- */
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart,
                                      uint8_t *pData, uint16_t Size)
{ (void)huart; (void)pData; (void)Size; return HAL_OK; }
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t a, uint32_t b)
{ (void)IRQn; (void)a; (void)b; }
void HAL_NVIC_EnableIRQ(IRQn_Type IRQn) { (void)IRQn; }
uint32_t HAL_GetTick(void) { return 1000U; }
void __disable_irq(void) {}
void __enable_irq(void) {}

/* ============================================================
 * 二、展开被测源码（static 函数对测试可见）
 * ============================================================ */
#include "../Src/arm_control.c"
#include "../Src/uart_protocol.c"

/* ============================================================
 * 三、测试工具
 * ============================================================ */
static int g_fail = 0;

#define CHECK(cond, name) do { \
    if (cond) { \
        printf("[PASS] %s\n", name); \
    } else { \
        printf("[FAIL] %s  <-- 看上面\n", name); \
        g_fail++; \
    } \
} while (0)

/* 喂一整串字节进协议层（模拟串口逐字节到达） */
static void feed(const char *s)
{
    while (*s) UartProto_HandleByte((uint8_t)*s++);
}

/* 清空假串口发送缓冲 */
static void tx_clear(void)
{
    g_tx[0] = '\0';
    g_tx_len = 0;
}

/* ============================================================
 * 四、用例组 1：FAULT 状态机（阶段 1 改动）
 * ============================================================ */
static void test_fault_state_machine(void)
{
    printf("\n--- FAULT 状态机 ---\n");

    /* 复位到已知状态 */
    Arm_Init();
    CHECK(Arm_GetState() == ARM_STATE_CMD && Arm_GetFault() == 0U,
          "上电后是 CMD 态、无故障");

    /* 报告故障 → 进 FAULT、故障码锁存、冻结 */
    Arm_ReportFault(SAFETY_FAULT_OVERCURRENT);
    CHECK(Arm_GetState() == ARM_STATE_FAULT, "ReportFault 进 FAULT 态");
    CHECK(Arm_GetFault() == SAFETY_FAULT_OVERCURRENT, "故障码锁存为 0x03");
    CHECK(Arm_IsFrozen() == 1U, "FAULT 态视为冻结");

    /* 冻结态下运动命令被拒 */
    {
        uint16_t six[6] = {1500,1500,1500,1500,1500,1500};
        CHECK(Arm_SetJointTarget(0, 1500U) == 0U, "FAULT 态拒绝单路目标");
        CHECK(Arm_SetAllTargets(six) == 0U, "FAULT 态拒绝六路目标");
    }

    /* 急停命令不能覆盖 FAULT（阶段 1 修的 bug：否则 fault_code 残留） */
    Arm_CommandStop();
    CHECK(Arm_GetState() == ARM_STATE_FAULT, "FAULT 态下 STOP 不覆盖故障态");
    CHECK(Arm_GetFault() == SAFETY_FAULT_OVERCURRENT, "STOP 后故障码仍在");

    /* RESUME 不能解除 FAULT */
    Arm_CommandResume();
    CHECK(Arm_GetState() == ARM_STATE_FAULT, "RESUME 不能解除 FAULT");

    /* CLEARFAULT 恢复并回 HOME（CMD 态） */
    Arm_ClearFault();
    CHECK(Arm_GetState() == ARM_STATE_CMD, "CLEARFAULT 回 CMD 态");
    CHECK(Arm_GetFault() == 0U, "CLEARFAULT 清故障码");
    CHECK(Arm_IsFrozen() == 0U, "恢复后不再冻结");

    /* 普通急停走 ESTOP 路径，仍可被 RESUME 解除 */
    Arm_CommandStop();
    CHECK(Arm_GetState() == ARM_STATE_ESTOP, "STOP 进 ESTOP");
    CHECK(Arm_IsFrozen() == 1U, "ESTOP 视为冻结");
    Arm_CommandResume();
    CHECK(Arm_GetState() == ARM_STATE_CMD, "RESUME 解除 ESTOP");
}

/* ============================================================
 * 五、用例组 2：协议错误码（阶段 2 改动）
 * ============================================================ */
static void test_protocol_errcode(void)
{
    printf("\n--- 协议错误码 ---\n");

    UART_HandleTypeDef huart;
    huart.Instance = USART1;
    huart.ErrorCode = HAL_UART_ERROR_NONE;
    huart.RxState = HAL_UART_STATE_READY;
    UartProto_Init(&huart);
    Arm_Init();

    /* 1. 未知命令 → ERR,2（阶段 2 从 ERR,1 改过来） */
    tx_clear();
    feed("$FLY*53\r\n");
    UartProto_Process();
    CHECK(strcmp(g_tx, "$ERR,2*5B\r\n") == 0, "未知命令回 ERR,2");

    /* 2. 校验失败 → ERR,1（阶段 2 从 ERR,2 改过来） */
    tx_clear();
    feed("$PING*00\r\n");  /* PING 正确 CK 是 10，这里故意给错 */
    UartProto_Process();
    CHECK(strcmp(g_tx, "$ERR,1*58\r\n") == 0, "校验失败回 ERR,1");

    /* 3. 参数错误仍 ERR,3（未改） */
    tx_clear();
    feed("$MOVE*11\r\n");  /* MOVE 缺参数，CK=11 */
    UartProto_Process();
    CHECK(strcmp(g_tx, "$ERR,3*5A\r\n") == 0, "缺参数回 ERR,3");

    /* 4. FAULT 态下运动命令 → ERR,5（frozen_err_code 新逻辑） */
    Arm_ReportFault(0x03);
    tx_clear();
    feed("$MOVE,0,1500*25\r\n");
    UartProto_Process();
    CHECK(strcmp(g_tx, "$ERR,5*5C\r\n") == 0, "FAULT 态运动命令回 ERR,5");
    Arm_ClearFault();

    /* 5. ESTOP 态下运动命令 → ERR,4 */
    Arm_CommandStop();
    tx_clear();
    feed("$MOVE,0,1500*25\r\n");
    UartProto_Process();
    CHECK(strcmp(g_tx, "$ERR,4*5D\r\n") == 0, "ESTOP 态运动命令回 ERR,4");
    Arm_CommandResume();

    /* 6. RESUME 在 FAULT 态 → ERR,5（显式拒绝，不静默失败） */
    Arm_ReportFault(0x03);
    tx_clear();
    feed("$RESUME*19\r\n");
    UartProto_Process();
    CHECK(strcmp(g_tx, "$ERR,5*5C\r\n") == 0, "FAULT 态 RESUME 回 ERR,5");
    Arm_ClearFault();

    /* 7. STATE 末尾追加 fault 字段（阶段 1/2：OK,S,<state>,<6脉宽>,<fault>） */
    Arm_ReportFault(0x03);
    tx_clear();
    feed("$STATE*57\r\n");
    UartProto_Process();
    /* 期望：状态字符 F（FAULT）、末尾 fault=3（形如 ,3*3F\r\n） */
    CHECK(strncmp(g_tx, "$OK,S,F,", 8) == 0, "FAULT 态 STATE 状态字符是 F");
    CHECK(strstr(g_tx, ",3*") != NULL, "STATE 末尾带故障码 3");
    Arm_ClearFault();

    /* 8. $FAULT 查询命令 */
    Arm_ReportFault(0x03);
    tx_clear();
    feed("$FAULT*4A\r\n");
    UartProto_Process();
    CHECK(strcmp(g_tx, "$FAULT,3*55\r\n") == 0, "$FAULT 回故障码 3");
    Arm_ClearFault();

    /* 9. $CLEARFAULT 命令 */
    Arm_ReportFault(0x03);
    tx_clear();
    feed("$CLEARFAULT*13\r\n");
    UartProto_Process();
    CHECK(strcmp(g_tx, "$OK*04\r\n") == 0, "$CLEARFAULT 回 OK");
    CHECK(Arm_GetState() == ARM_STATE_CMD, "$CLEARFAULT 后回 CMD 态");
}

/* ============================================================
 * 六、入口
 * ============================================================ */
int main(void)
{
    printf("=== arm_control.c / uart_protocol.c host 单元测试 ===\n");
    test_fault_state_machine();
    test_protocol_errcode();
    printf("\n=== 结果：%s（失败 %d 项）===\n",
           g_fail ? "不通过" : "全部通过", g_fail);
    return g_fail ? 1 : 0;
}
