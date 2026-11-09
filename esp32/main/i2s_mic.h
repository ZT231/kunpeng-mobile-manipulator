#ifndef I2S_MIC_H
#define I2S_MIC_H

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"

/* ============================================================
 * i2s_mic —— INMP441 数字麦采音（I2S0 RX）
 *
 * 【概念（大白话）】I2S 就是专门传声音的几根线：一根打拍子(BCLK)、
 *   一根说「现在左耳还是右耳」(WS)、一根传数据(SD)。
 *   麦克风把声音变成 0/1 数字顺着 SD 送进来，比模拟麦不容易被干扰。
 * 【规范定义】Inter-IC Sound（I²S / Philips 标准）串行音频总线；
 *   INMP441 输出 24-bit 有效数据、左对齐放在 32-bit 时隙里，
 *   由主机提供 BCLK/WS，器件在 WS 指定的时隙驱动 SD 线。
 *
 * 🔴 诚实边界：这是驱动骨架，只验证「能不能读到 PCM」，
 *    不含唤醒词、不含 ASR、不把数据发给鲲鹏。
 * ============================================================ */

/* ---------- 接线宏：按成品板实际接线只改这里，别散落到 .c 里 ---------- */
/* 【为什么用宏】引脚一变只改一处；写死在 .c 里会有 5 处要同步，漏一处就静音。 */
#define MIC_I2S_PORT      I2S_NUM_0     /* I2S0 给麦；I2S1 留给 MAX98357A 功放 */
#define MIC_GPIO_BCLK     GPIO_NUM_4    /* → INMP441 SCK  */
#define MIC_GPIO_WS       GPIO_NUM_5    /* → INMP441 WS   */
#define MIC_GPIO_DIN      GPIO_NUM_6    /* → INMP441 SD   */
#define MIC_SAMPLE_RATE   16000         /* 16kHz 足够做唤醒/命令词，省算力 */
#define MIC_READ_BYTES    1024          /* 单次最多读多少字节，防止一次读爆 */

/* 启动 I2S0 RX 通道；成功返回 ESP_OK */
esp_err_t i2s_mic_start(void);

/* 读一批 PCM。返回值 = 实际读到的采样个数（已右移成 24-bit 有符号值）；失败返回 0 */
int i2s_mic_read(int32_t *samples, size_t max_samples);

/* 关掉并释放通道（当前 app_main 不调用，留着给以后低功耗/重启用） */
void i2s_mic_stop(void);

#endif /* I2S_MIC_H */
