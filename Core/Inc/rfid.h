#ifndef __RFID_H__
#define __RFID_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ============================================================
 *  MFRC-522 RC522 射频卡读取模块（GPIO 软件 SPI）
 *
 *      RC522 RST   -> PB0
 *      RC522 SDA   -> PB12（SPI 片选 CS，低有效）
 *      RC522 SCK   -> PB13（GPIO 输出）
 *      RC522 MISO  -> PB14（GPIO 输入）
 *      RC522 MOSI  -> PB15（GPIO 输出）
 *      RC522 IRQ   -> 不接（轮询）
 *      RC522 3.3V  -> 3.3V（严禁 5V）
 *
 *  底层使用 GPIO 软件 SPI，不依赖 SPI2 外设。
 * ============================================================ */

/* MIFARE Classic 卡的 UID 长度 */
#define RFID_UID_LEN      4U

/* 卡片结构 */
typedef struct {
    uint8_t uid[RFID_UID_LEN];   /* 4 字节 UID */
} RFID_Card_t;

/* ============================================================
 *  刷卡切换模式使用的两张卡（UID 已实测填入）
 *  两张卡 UID 请保证不同，否则按先匹配者生效。
 * ============================================================ */
#define RFID_UID_CARD_MANUAL { 0x9A, 0x7C, 0x73, 0x8E }   /* 钥匙扣实测 UID：刷卡 -> 进入 MANUAL 手动模式 */
#define RFID_UID_CARD_HOME   { 0x5B, 0x0B, 0xDE, 0x53 }   /* 卡实测 UID：刷卡 -> 回到 HOME / CMD 命令模式 */

/* 初始化 RC522：配置 GPIO、软复位、开启天线。初始化失败返回 HAL_ERROR */
HAL_StatusTypeDef RFID_Init(void);

/* 检测感应区是否有卡。
 *   有卡：返回 HAL_OK，并把 4 字节 UID 写入 card->uid[]
 *   无卡：返回 HAL_ERROR
 * 应在主循环轮询调用（内部非阻塞，单次调用约 1~2ms）。 */
HAL_StatusTypeDef RFID_CheckCard(RFID_Card_t *card);

#ifdef __cplusplus
}
#endif

#endif /* __RFID_H__ */
