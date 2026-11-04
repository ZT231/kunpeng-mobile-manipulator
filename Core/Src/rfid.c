/* ============================================================
 *  MFRC-522 RC522 射频卡读取驱动（GPIO 软件 SPI 实现）
 * ============================================================
 *  软件 SPI 引脚：SCK=PB13，MISO=PB14，MOSI=PB15，CS/SDA=PB12，RST=PB0。
 *  使用 GPIO 软件 SPI，避免 SPI2 复用/模式与不同 RC522 模块不兼容。
 * ============================================================ */

#include "rfid.h"
#include <stdio.h> /* printf */

/* ---------------- 软件 SPI 引脚 ---------------- */
#define RFID_SCK_Pin       SCK_Pin
#define RFID_SCK_Port      SCK_GPIO_Port
#define RFID_MISO_Pin      MISO_Pin
#define RFID_MISO_Port     MISO_GPIO_Port
#define RFID_MOSI_Pin      MOSI_Pin
#define RFID_MOSI_Port     MOSI_GPIO_Port

#define RFID_SCK_LOW()     HAL_GPIO_WritePin(RFID_SCK_Port, RFID_SCK_Pin, GPIO_PIN_RESET)
#define RFID_SCK_HIGH()    HAL_GPIO_WritePin(RFID_SCK_Port, RFID_SCK_Pin, GPIO_PIN_SET)
#define RFID_MOSI_LOW()    HAL_GPIO_WritePin(RFID_MOSI_Port, RFID_MOSI_Pin, GPIO_PIN_RESET)
#define RFID_MOSI_HIGH()   HAL_GPIO_WritePin(RFID_MOSI_Port, RFID_MOSI_Pin, GPIO_PIN_SET)
#define RFID_MISO_READ()   HAL_GPIO_ReadPin(RFID_MISO_Port, RFID_MISO_Pin)

/* ---------------- 微秒延时：DWT 周期计数器 ---------------- */
static void rfid_delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (SystemCoreClock / 1000000U) * us;
    while ((DWT->CYCCNT - start) < cycles) { }
}

static void rfid_dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* ---------------- 软件 SPI 字节收发（Mode 0，MSB先出） ---------------- */
static uint8_t spi_transfer(uint8_t out)
{
    uint8_t in = 0U;
    int8_t i;

    for (i = 7; i >= 0; i--)
    {
        if ((out & (uint8_t)(1U << i)) != 0U)
        {
            RFID_MOSI_HIGH();
        }
        else
        {
            RFID_MOSI_LOW();
        }

        RFID_SCK_LOW();
        rfid_delay_us(1U);
        RFID_SCK_HIGH();
        rfid_delay_us(1U);

        if (RFID_MISO_READ() == GPIO_PIN_SET)
        {
            in |= (uint8_t)(1U << i);
        }
    }
    return in;
}
#define RC522_COMMAND          0x01   /* 命令寄存器 */
#define RC522_COMIEN           0x02   /* 中断使能 */
#define RC522_COMIRQ           0x04   /* 通用中断标志 */
#define RC522_ERROR            0x06   /* 错误寄存器 */
#define RC522_STATUS2          0x08
#define RC522_FIFODATA         0x09
#define RC522_FIFOLEVEL        0x0A
#define RC522_CONTROL          0x0C
#define RC522_BITFRAMING       0x0D   /* 位格式化 */
#define RC522_COLL             0x0E   /* 冲突寄存器 */
#define RC522_MODE             0x11
#define RC522_TXCONTROL        0x14   /* 发送控制（天线开关） */
#define RC522_TXASK            0x15   /* 发送调制（100% ASK） */
#define RC522_TMODE            0x2A   /* 定时器模式 */
#define RC522_TPRESCALER       0x2B   /* 定时器分频 */
#define RC522_TRELOAD_H        0x2C   /* 定时器重载高位 */
#define RC522_TRELOAD_L        0x2D   /* 定时器重载低位 */
#define RC522_VERSION          0x37

/* ---------------- MFRC522 命令字 ---------------- */
#define RC522_CMD_IDLE         0x00
#define RC522_CMD_TRANSCEIVE   0x0C
#define RC522_CMD_SOFTRESET    0x0F

/* ---------------- 与卡通信返回码（参考工程） ---------------- */
#define MI_OK        0
#define MI_NOTAGERR  1
#define MI_ERR       2

/* 防碰撞级1：SEL=0x93 / Cascaded init 0x20（读取 UID + 1 字节 BCC） */
#define PICC_ANTICOLL_CL1  0x93

/* ---------------- 手动 GPIO 操作（CS/RST） ---------------- */
#define RFID_CS_LOW()   HAL_GPIO_WritePin(RC522_CS_SDA_GPIO_Port, RC522_CS_SDA_Pin, GPIO_PIN_RESET)
#define RFID_CS_HIGH()  HAL_GPIO_WritePin(RC522_CS_SDA_GPIO_Port, RC522_CS_SDA_Pin, GPIO_PIN_SET)

/* ---------------- 寄存器读写（CS 低有效） ---------------- */
/* 读地址：bit7=1；写地址：bit7=0。与参考工程 ReadRawRC/WriteRawRC 一致 */
static uint8_t rc522_read(uint8_t reg)
{
    uint8_t val;
    RFID_CS_LOW();
    spi_transfer((uint8_t)(((reg << 1U) & 0x7EU) | 0x80U));
    val = spi_transfer(0x00U);
    RFID_CS_HIGH();
    return val;
}

static void rc522_write(uint8_t reg, uint8_t val)
{
    RFID_CS_LOW();
    spi_transfer((uint8_t)((reg << 1U) & 0x7EU));
    spi_transfer(val);
    RFID_CS_HIGH();
}

static void rc522_set_bits(uint8_t reg, uint8_t mask)
{
    rc522_write(reg, (uint8_t)(rc522_read(reg) | mask));
}

static void rc522_clear_bits(uint8_t reg, uint8_t mask)
{
    rc522_write(reg, (uint8_t)(rc522_read(reg) & (~mask)));
}

/* ---------------- 发送/接收一帧（移植自参考工程 PcdComMF522） ---------------- */
static uint8_t rc522_com_mf522(uint8_t cmd, const uint8_t *in, uint8_t inLen,
                               uint8_t *out, uint8_t *outLenBits)
{
    uint8_t n;
    uint8_t irqEn = 0x00U;
    uint8_t waitFor = 0x00U;
    uint32_t cnt;
    char status = MI_ERR;

    if (cmd == RC522_CMD_TRANSCEIVE)
    {
        irqEn   = 0x77U;   /* 允许 TxI/RxI/IdleI/LoAlertI/ErrI/TimerI 中断 */
        waitFor = 0x30U;   /* 等待接收/空闲事件 */
    }

    rc522_write(RC522_COMIEN, (uint8_t)(irqEn | 0x80U));
    rc522_clear_bits(RC522_COMIRQ, 0x80U);
    rc522_write(RC522_COMMAND, RC522_CMD_IDLE);
    rc522_write(RC522_FIFOLEVEL, 0x80U);            /* 清 FIFO */
    for (cnt = 0U; cnt < inLen; cnt++)
    {
        rc522_write(RC522_FIFODATA, in[cnt]);
    }

    rc522_write(RC522_COMMAND, cmd);
    if (cmd == RC522_CMD_TRANSCEIVE)
    {
        rc522_set_bits(RC522_BITFRAMING, 0x80U);    /* StartSend */
    }

    /* 等待循环次数：软件 SPI 单次寄存器读（16 个时钟）约十几µs，
     * 3000 次循环耗时远超 20ms，远大于应答窗口3.2ms → 定时器中断必定先到；
     * 之前1000次循环短于窗口12.5ms 是 REQA 一直 Irq=00 的根因 */
    cnt = 3000U;
    n = 0U;
    do
    {
        n = rc522_read(RC522_COMIRQ);
        cnt--;
    } while ((cnt != 0U) && (!(n & 0x01U)) && (!(n & waitFor)));

    rc522_clear_bits(RC522_BITFRAMING, 0x80U);

    if (cnt != 0U)
    {
        if (!(rc522_read(RC522_ERROR) & 0x1BU))     /* 无 BufferOvfl/Coll/Parity/Protocol 错误 */
        {
            status = MI_OK;
            if (n & (irqEn & 0x01U))                   /* 定时器到时 -> 无卡应答 */
            {
                status = MI_NOTAGERR;
            }

            if (cmd == RC522_CMD_TRANSCEIVE)
            {
                n = rc522_read(RC522_FIFOLEVEL);       /* FIFO 中的字节数 */
                {
                    uint8_t lastBits = (uint8_t)(rc522_read(RC522_CONTROL) & 0x07U);
                    if (lastBits != 0U)
                    {
                        *outLenBits = (uint8_t)((n - 1U) * 8U + lastBits);
                    }
                    else
                    {
                        *outLenBits = (uint8_t)(n * 8U);
                    }
                }
                if (n == 0U)
                {
                    n = 1U;
                }
                if (n > 18U)
                {
                    n = 18U;
                }
                for (cnt = 0U; cnt < n; cnt++)
                {
                    out[cnt] = rc522_read(RC522_FIFODATA);
                }
            }
        }
        else
        {
            status = MI_ERR;
        }
    }

    rc522_set_bits(RC522_CONTROL, 0x80U);            /* 停止定时器 */
    rc522_write(RC522_COMMAND, RC522_CMD_IDLE);
    return (uint8_t)status;
}

/* ---------------- 寻卡（移植自参考工程 PcdRequest） ---------------- */
static char rc522_request(uint8_t reqCode, uint8_t *tagType)
{
    uint8_t buf[18];
    uint8_t outLenBits;
    char status;

    rc522_clear_bits(RC522_STATUS2, 0x08U);
    rc522_write(RC522_BITFRAMING, 0x07U);            /* REQA 最后 7 位有效 */
    rc522_set_bits(RC522_TXCONTROL, 0x03U);          /* 开天线驱动 */

    buf[0] = reqCode;
    status = (char)rc522_com_mf522(RC522_CMD_TRANSCEIVE, buf, 1U, buf, &outLenBits);
    if ((status == MI_OK) && (outLenBits == 0x10U))  /* 收到 2 字节 ATQA */
    {
        tagType[0] = buf[0];
        tagType[1] = buf[1];
    }
    else
    {
        status = MI_ERR;
    }
    return status;
}

/* ---------------- 防碰撞读 UID（移植自参考工程 PcdAnticoll） ---------------- */
static char rc522_anticoll(uint8_t *snr)
{
    uint8_t buf[18];
    uint8_t outLenBits;
    uint8_t i;
    uint8_t bcc = 0U;
    char status;

    rc522_clear_bits(RC522_STATUS2, 0x08U);
    rc522_write(RC522_BITFRAMING, 0x00U);
    rc522_clear_bits(RC522_COLL, 0x80U);

    buf[0] = PICC_ANTICOLL_CL1;
    buf[1] = 0x20U;
    status = (char)rc522_com_mf522(RC522_CMD_TRANSCEIVE, buf, 2U, buf, &outLenBits);

    if (status == MI_OK)
    {
        for (i = 0U; i < 4U; i++)
        {
            snr[i] = buf[i];
            bcc ^= buf[i];
        }
        if (bcc != buf[4])
        {
            status = MI_ERR;
        }
    }

    rc522_set_bits(RC522_COLL, 0x80U);
    return status;
}

/* ---------------- 定时器/调制参数配置（应答窗口核心，见 RFID_Init 注释） ---------------- */
static void rc522_apply_config(void)
{
    rc522_write(RC522_MODE, 0x3DU);          /* 定义发送/接收常用模式 */
    rc522_write(RC522_TMODE, 0x80U);         /* 定时器模式：TAuto 开 */
    rc522_write(RC522_TPRESCALER, 0xA9U);    /* 定时器分频 */
    rc522_write(RC522_TRELOAD_H, 0x00U);     /* 定时器重载值高位 */
    rc522_write(RC522_TRELOAD_L, 0xFAU);     /* 定时器重载值低位（250 → ~3.2ms 窗口） */
    rc522_write(RC522_TXASK, 0x40U);         /* 100% ASK 调制度 */
}

/* ============================================================
 *  初始化（GPIO + 软复位 + 定时器/调制参数，参数与参考工程一致）
 * ============================================================ */
HAL_StatusTypeDef RFID_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* 软件 SPI 使用 Mode 0，不依赖 SPI2 外设配置；CS/RST 由 CubeMX GPIO 宏控制。 */

    /* 使能 GPIOB 时钟（CS PB12 / RST PB0 / 软件SPI PB13~PB15） */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 软件SPI输出：SCK、MOSI；输入：MISO；CS/RST均为输出 */
    gpio.Pin   = RC522_RST_Pin | RC522_CS_SDA_Pin | RFID_SCK_Pin | RFID_MOSI_Pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin   = RFID_MISO_Pin;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    rfid_dwt_init();

    /* 软件SPI Mode 0 空闲低电平，CS空闲高 */
    RFID_SCK_LOW();
    RFID_MOSI_LOW();
    RFID_CS_HIGH();
    HAL_GPIO_WritePin(RC522_RST_GPIO_Port, RC522_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(RC522_RST_GPIO_Port, RC522_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(50);

    /* 软复位后延时等待（与原版可 init OK 的写法一致；不忙等读回，避免状态被破坏） */
    rc522_write(RC522_COMMAND, RC522_CMD_SOFTRESET);
    HAL_Delay(50);

    /* 应答窗口定时器（根因修复）：
     * 窗口 = (TPrescaler+1)*(TReload+1)/13.56MHz。
     * 之前 TReload=0x03E8 → 窗口≈12.5ms，比 rc522_com_mf522 的等待循环
     * （1000次 × 软件SPI每次读十余µs ≈ 十几ms）还长，循环先耗尽 → 永远 Irq=00。
     * 现改为 TPrescaler=0xA9 + TReload=0x00FA → 窗口≈3.2ms，
     * 远小于 3000 次循环耗时，保证定时器中断先到：
     *   有卡 → RxI 置位(≈1ms)；无卡 → TimerI 置位(≈3.2ms) */
    rc522_apply_config();

    /* 开启天线驱动两路 */
    if (!(rc522_read(RC522_TXCONTROL) & 0x03U))
    {
        rc522_set_bits(RC522_TXCONTROL, 0x03U);
    }

    /* 版本确认（仅诊断，不阻断）：MFRC522 通常 0x92，国产兼容芯片 0x82/0x91/0x90 */
    {
        uint8_t ver = 0U;
        uint8_t retry;
        for (retry = 0U; retry < 6U; retry++)
        {
            ver = rc522_read(RC522_VERSION);
            if (ver != 0x00U && ver != 0xFFU)
            {
                break;
            }
            HAL_Delay(10);
        }
        printf("RC522 version: 0x%02X\r\n", (unsigned)ver);
    }
    return HAL_OK;
}

/* ============================================================
 *  检测感应区是否有卡并读取 4 字节 UID
 *  （寻卡 + 防碰撞，逻辑与参考工程 Car_Start 一致）
 * ============================================================ */
HAL_StatusTypeDef RFID_CheckCard(RFID_Card_t *card)
{
    uint8_t tagType[2];
    uint8_t snr[4];
    uint8_t i;

    if (card == NULL)
    {
        return HAL_ERROR;
    }

    /* 检测期间才开天线，结束立即关闭，降低模块平均电流 */
    rc522_set_bits(RC522_TXCONTROL, 0x03U);

    if (rc522_request(0x52U, tagType) != MI_OK)
    {
        rc522_clear_bits(RC522_TXCONTROL, 0x03U);
        /* 芯片失联兜底：Ver=00/FF 说明芯片无响应（供电瞬态/接触不良常致复位），
         * 节流执行硬件 RST 脉冲 + 软复位 + 重配参数，让读卡能力自动恢复 */
        {
            static uint8_t recover = 0U;
            if (recover == 0U)
            {
                uint8_t v = rc522_read(RC522_VERSION);
                if (v == 0x00U || v == 0xFFU)
                {
                    recover = 10U;   /* 恢复后 10 轮内不再重复 */
                    HAL_GPIO_WritePin(RC522_RST_GPIO_Port, RC522_RST_Pin, GPIO_PIN_RESET);
                    HAL_Delay(20);
                    HAL_GPIO_WritePin(RC522_RST_GPIO_Port, RC522_RST_Pin, GPIO_PIN_SET);
                    HAL_Delay(20);
                    rc522_write(RC522_COMMAND, RC522_CMD_SOFTRESET);
                    HAL_Delay(10);
                    rc522_apply_config();
                }
            }
            else
            {
                recover--;
            }
        }
        return HAL_ERROR;
    }
    if (rc522_anticoll(snr) != MI_OK)
    {
        rc522_clear_bits(RC522_TXCONTROL, 0x03U);
        return HAL_ERROR;
    }

    rc522_clear_bits(RC522_TXCONTROL, 0x03U);

    for (i = 0U; i < RFID_UID_LEN; i++)
    {
        card->uid[i] = snr[i];
    }
    return HAL_OK;
}
