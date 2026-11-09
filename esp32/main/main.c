#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "i2s_mic.h"
#include "wifi_sta.h"
#include "protocol.h"

/* ============================================================
 * app_main —— ESP-IDF 的程序入口
 *
 * 【概念（大白话）】写单片机裸机是 main()，ESP-IDF 里叫 app_main()：
 *   FreeRTOS 先把系统、调度器都起好，再调这个函数，所以我们
 *   一进来就能用任务、延时、日志，不用自己初始化时钟。
 * 【规范定义】app_main 运行在「main task」上下文，栈大小由
 *   CONFIG_ESP_MAIN_TASK_STACK_SIZE 决定；函数里可以阻塞、可以起新任务。
 *
 * 🔴 诚实边界（每次上板前默念一遍）：
 *   本固件只做「连 Wi-Fi + 采 PCM + 打印封帧自检」。
 *   没有 WakeNet 唤醒、没有 ASR、没有把文本发到鲲鹏、没有下发到 STM32。
 *   protocol_build_frame() 产出的帧**只打印不下发**，
 *   与鲲鹏侧 kunpeng/llm/text_gateway.py 的「只打印不下发」保持一致。
 * ============================================================ */

static const char *TAG = "voice_main";

void app_main(void)
{
    ESP_LOGI(TAG, "==== ESP32-S3 语音入口骨架启动 ====");
    ESP_LOGW(TAG, "只采音 / 只打印，未接 WakeNet、未接 ASR、未接真串口、未下发到鲲鹏或 STM32");

    /* ---------- 第 1 步：连 Wi-Fi ----------
     * 【为什么先连】B 的目标之一就是证明网络通；连不上也别直接 return，
     *   让采音自检照样跑，这样一根杜邦线没接好也能单独定位，不互相掩盖。
     * 【顺序】不这么写（连不上就死等）会把「Wi-Fi 错」和「I2S 错」混成一次故障。 */
    esp_err_t err = wifi_sta_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi 未连上，继续跑 I2S 采音自检（两者互不依赖）");
    } else {
        ESP_LOGI(TAG, "Wi-Fi OK, is_connected=%d", wifi_sta_is_connected() ? 1 : 0);
    }

    /* ---------- 第 2 步：启 I2S0 采音 ---------- */
    err = i2s_mic_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2S 启动失败，停止自检。先查 BCLK/WS/DIN 三根线与 L/R 引脚");
        return;
    }

    /* ---------- 第 3 步：封帧自检（只打印不下发） ----------
     * 【为什么用 ASCII "ASR,mic_ok"】避免中文 UTF-8 在串口 monitor 里
     *   因终端编码显示成乱码，干扰校验结果的判读；中文逐字节对齐已由
     *   Python 侧 test_text_gateway.py 与 docs/排障记录-LLM.md 覆盖。
     * 这行打印出来的帧，正好是鲲鹏 text_gateway.parse_frame() 能收的形状。 */
    char frame[96];
    int frame_len = protocol_build_frame("ASR,mic_ok", frame, sizeof(frame));
    if (frame_len > 0) {
        ESP_LOGI(TAG, "封帧自检（只打印不下发，长度=%d）: %s", frame_len, frame);
    } else {
        ESP_LOGE(TAG, "封帧失败：缓冲区太小，检查 frame[] 大小");
    }

    /* ---------- 第 4 步：循环读 PCM，每秒打印一次前 4 个采样 ----------
     * 【概念】采样 = 某一瞬间声音的振幅，一个 int32_t 就是一个采样。
     *   打印出来不是 0 / 不是恒定的常数，就说明麦克风真的在出数据。
     * 【为什么 memset】数组是栈上分配的，不清零会带着上次的旧值；
     *   若这次只读到 1 个采样，后面 3 个打印出来是垃圾，会误导排障。 */
    int32_t samples[256];
    int loop = 0;

    while (1) {
        memset(samples, 0, sizeof(samples));

        int n = i2s_mic_read(samples, 256);
        if (n > 0) {
            ESP_LOGI(TAG, "[%03d] 读到 %d 个采样: %d %d %d %d",
                     loop, n,
                     (int)samples[0], (int)samples[1],
                     (int)samples[2], (int)samples[3]);
        } else {
            ESP_LOGW(TAG, "[%03d] 没读到采样：查 BCLK/WS/DIN 接线、L/R 引脚、3.3V 供电", loop);
        }

        loop++;
        /* vTaskDelay 让出 CPU，别用空 for 循环硬等（那会把一个核跑满） */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* 实际不会走到这里（while(1)）；留着是提醒：真要退出得先 i2s_mic_stop() */
    /* i2s_mic_stop(); */
}
