#ifndef STM32F4XX_HAL_H
#define STM32F4XX_HAL_H
/*
 * ============================================================================
 * stm32f4xx_hal.h —— 硬件层的「假头文件」（host 测试专用，不参与 Keil 编译）
 * ============================================================================
 *
 * 【为什么要有这个文件】真机的 stm32f4xx_hal.h 在 Drivers/ 里，它会把整个
 *   CMSIS 内核寄存器定义、上百个外设结构体都拖进来，PC 的 gcc 编译不过，
 *   也没必要——协议层/状态机层根本不碰寄存器。
 *
 * 【做法】只定义被测模块（arm_control.c / uart_protocol.c）真正用到的
 *   类型和宏：状态枚举、UART/I2C/ADC/GPIO 句柄的「壳」、几个错误码宏。
 *   它们只有字段、没有真实硬件行为，真行为由测试文件里的 stub 函数提供。
 *   这叫「依赖倒置 / 测试替身（test double）」——把不稳定的硬件依赖
 *   换成稳定可控的假实现，只测被测模块自己的逻辑。
 *
 * 【红线】本文件只存在于 Core/tests/stubs/，绝不进 Keil 工程、绝不 #include
 *   真机代码。它和真机的 stm32f4xx_hal.h 是两套东西，互不干扰。
 * ============================================================================
 */
#include <stdint.h>

/* HAL 通用返回码 */
typedef enum {
    HAL_OK      = 0x00U,
    HAL_ERROR   = 0x01U,
    HAL_BUSY    = 0x02U,
    HAL_TIMEOUT = 0x03U,
} HAL_StatusTypeDef;

/* I2C / ADC / GPIO 句柄：只留占位，被测代码只拿它当指针传，不碰内容 */
typedef struct { int unused; } I2C_HandleTypeDef;
typedef struct { int unused; } ADC_HandleTypeDef;
typedef struct { int unused; } GPIO_TypeDef;

/*
 * UART 句柄：uart_protocol.c 会读 Instance（判断是哪个串口）、
 * ErrorCode、RxState 三个字段，所以要真给它留出来。
 */
typedef struct {
    void    *Instance;   /* 真机是 USART_TypeDef*，这里用 void* 只做身份比较 */
    uint32_t ErrorCode;
    uint32_t RxState;
} UART_HandleTypeDef;

/* UART 实例身份：三个不同的地址，只用于 == 比较 */
#define USART1  ((void*)0x40011000UL)
#define USART2  ((void*)0x40004400UL)
#define USART3  ((void*)0x40004800UL)

/* 中断号（HAL_NVIC_SetPriority/EnableIRQ 的参数，测试里是空操作） */
typedef int IRQn_Type;
#define USART1_IRQn 37
#define USART2_IRQn 38
#define USART3_IRQn 39

/* UART 错误码 / 状态（uart_protocol.c 的 HAL_UART_ErrorCallback 用到） */
#define HAL_UART_ERROR_NONE   0x00000000U
#define HAL_UART_ERROR_ORE    0x00000008U
#define UART_FLAG_ORE         0x00000008U
#define HAL_UART_STATE_READY  0x20U
#define HAL_UART_STATE_BUSY_RX 0x22U

/* 读/清 ORE 标志位：真机读 SR 寄存器，这里读 ErrorCode 字段（够测逻辑用） */
#define __HAL_UART_GET_FLAG(__HANDLE__, __FLAG__) \
    (((__HANDLE__)->ErrorCode & (__FLAG__)) != 0U)
#define __HAL_UART_CLEAR_OREFLAG(__HANDLE__) \
    do { (__HANDLE__)->ErrorCode &= ~HAL_UART_ERROR_ORE; } while (0)

#define RESET 0U
#define SET   1U

/* GPIO 引脚 / 端口（main.h 里 KEY0/RC522 的宏要用） */
#define GPIO_PIN_0  ((uint16_t)0x0001U)
#define GPIO_PIN_4  ((uint16_t)0x0010U)
#define GPIO_PIN_12 ((uint16_t)0x1000U)
#define GPIO_PIN_13 ((uint16_t)0x2000U)
#define GPIO_PIN_14 ((uint16_t)0x4000U)
#define GPIO_PIN_15 ((uint16_t)0x8000U)
#define GPIOB ((GPIO_TypeDef*)0x42420400UL)
#define GPIOE ((GPIO_TypeDef*)0x42421000UL)

/*
 * 下面这些 HAL 函数在真机里是正式实现，这里只声明、由测试文件提供 stub 定义。
 * 被测模块调它们时，走的是测试里的假实现（捕获发送内容 / 假装接收成功）。
 */
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart,
                                    const uint8_t *pData,
                                    uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart,
                                      uint8_t *pData, uint16_t Size);
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority,
                          uint32_t SubPriority);
void HAL_NVIC_EnableIRQ(IRQn_Type IRQn);
uint32_t HAL_GetTick(void);

/* CMSIS 内建：真机是关/开全局中断，测试里是空操作 */
void __disable_irq(void);
void __enable_irq(void);

#endif /* STM32F4XX_HAL_H */
