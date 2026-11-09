#include "i2s_mic.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* 【概念】TAG 只是日志前缀，ESP_LOGI(TAG, "...") 打印出来是 I (xxx) i2s_mic: ...
 *   串口 monitor 里一眼能看出是哪块代码在说话，排障时很有用。 */
static const char *TAG = "i2s_mic";

/* 【概念】句柄 handle：驱动内部替我们管着的「通道对象」的一个编号。
 *   静态全局变量 = 本文件私有、整个程序运行期间只有一份。
 *   不这么写而下一次读数据时才临时创建，就会每次都重新初始化硬件，麦根本来不及出数。 */
static i2s_chan_handle_t s_rx_chan = NULL;

esp_err_t i2s_mic_start(void)
{
    /* ---------- 第 1 步：开一个「通道」 ----------
     * 【规范】ESP-IDF v5.x 的新 I2S 驱动分两层：
     *   i2s_new_channel() 先申请 DMA 资源并决定「谁当主机」，
     *   i2s_channel_init_std_mode() 再填时钟/时隙/GPIO。
     * 【为什么分开】同一个通道既能配 Philips 标准也能配 PCM/TDM，
     *   拆开才不用为每种时序重写一遍 DMA 初始化。 */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(MIC_I2S_PORT, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 4;     /* DMA 描述符个数：多一点，CPU 稍微忙一点也不会丢采样 */
    chan_cfg.dma_frame_num = 240;  /* 每个描述符装多少帧 */

    /* 只要 RX（录音）：第 2 个参数传 NULL = 不建 TX 通道。
     * 功放 MAX98357A 走 I2S1，是另一路，别在这里一起开。 */
    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &s_rx_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel 失败: %s", esp_err_to_name(err));
        return err;
    }

    /* ---------- 第 2 步：配 Philips(I2S) 标准时序 ---------- */
    i2s_std_config_t std_cfg = {
        /* 时钟：主机按 16kHz 产生 BCLK/WS */
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(MIC_SAMPLE_RATE),

        /* 时隙：32-bit 宽、单声道。
         * 【为什么选 32-bit】INMP441 的 24-bit 数据是装在 32-bit 时隙里的，
         *   驱动按 32-bit 读最省事，后面我们自己右移 8 位还原。 */
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                        I2S_SLOT_MODE_MONO),

        /* GPIO：只接 3 根线。mclk/dout 用 I2S_GPIO_UNUSED 表示不接 */
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,   /* INMP441 不需要主时钟 MCLK */
            .bclk = MIC_GPIO_BCLK,
            .ws   = MIC_GPIO_WS,
            .dout = I2S_GPIO_UNUSED,   /* 只录不播 */
            .din  = MIC_GPIO_DIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,     /* 三根线都不反相；若波形反了再改这里 */
            },
        },
    };

    /* 【坑点】INMP441 的 L/R 引脚决定它占「左时隙」还是「右时隙」：
     *   L/R 接 GND = 左声道，接 VDD = 右声道。
     *   这里按接 GND 写死左时隙；若读出来全是 0，第一个要查的就是它。 */
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

    err = i2s_channel_init_std_mode(s_rx_chan, &std_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_init_std_mode 失败: %s", esp_err_to_name(err));
        i2s_del_channel(s_rx_chan);
        s_rx_chan = NULL;
        return err;
    }

    /* ---------- 第 3 步：使能通道（这一步之后 DMA 才真的开始搬数据） ---------- */
    err = i2s_channel_enable(s_rx_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_enable 失败: %s", esp_err_to_name(err));
        i2s_del_channel(s_rx_chan);
        s_rx_chan = NULL;
        return err;
    }

    ESP_LOGI(TAG, "I2S0 RX 已启动: %d Hz, BCLK=%d WS=%d DIN=%d",
             MIC_SAMPLE_RATE, MIC_GPIO_BCLK, MIC_GPIO_WS, MIC_GPIO_DIN);
    return ESP_OK;
}

int i2s_mic_read(int32_t *samples, size_t max_samples)
{
    /* 【新手习惯】入口先挡住明显不能干的情况，别让 NULL 指针把板子搞死机 */
    if (s_rx_chan == NULL || samples == NULL || max_samples == 0) {
        return 0;
    }

    size_t want_bytes = max_samples * sizeof(int32_t);
    if (want_bytes > MIC_READ_BYTES) {
        want_bytes = MIC_READ_BYTES;
    }

    size_t bytes_read = 0;
    /* i2s_channel_read 会阻塞到有数据或超时；超时给 200ms，别让任务永久卡住 */
    esp_err_t err = i2s_channel_read(s_rx_chan, (char *)samples, want_bytes,
                                     &bytes_read, pdMS_TO_TICKS(200));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2s_channel_read 返回: %s", esp_err_to_name(err));
        return 0;
    }

    int n = (int)(bytes_read / sizeof(int32_t));

    /* 【规范】INMP441 输出 24-bit 有效数据、左对齐放在 32-bit 时隙的高 24 位，
     *   低 8 位是补零。右移 8 位就得到「有符号 24-bit」的 PCM 值。
     * 【为什么在 C 里简单】C 的 int32_t 算术右移保留符号位，正好还原负半周；
     *   这也和项目 Python 侧「XOR 必须先 encode 成字节」是同一类思维：
     *   音频/校验都是按字节和位在算，不是按字符。 */
    for (int i = 0; i < n; i++) {
        samples[i] = samples[i] >> 8;
    }
    return n;
}

void i2s_mic_stop(void)
{
    if (s_rx_chan == NULL) {
        return;
    }
    /* 【为什么先 disable 再 del】enable 状态下直接删通道，DMA 还在写内存，
     *   轻则报资源忙，重则踩内存。顺序反了是新手常见坑。 */
    i2s_channel_disable(s_rx_chan);
    i2s_del_channel(s_rx_chan);
    s_rx_chan = NULL;
    ESP_LOGI(TAG, "I2S0 RX 已停止");
}
