#include <string.h>       /* memset */
#include <stdbool.h>      /* bool / true / false —— 用到就自己 include，别靠 wifi_ap.h 间接带进来 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "i2s_mic.h"
#include "wifi_ap.h"
#include "protocol.h"
#include "tcp_client.h"
#include "audio_tx.h"

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
 *   本固件做「开 SoftAP 热点 + 文本链路自检 + 连续采 PCM 经 TCP 上行」。
 *   - 仍然没有 WakeNet 唤醒、没有 ASR、没有语音转文字；
 *   - 仍然不直接驱动舵机、不碰 STM32、不改 .ioc；
 *   - 9000 端口发「链路自检帧」$ESTOP / $RESUME，网关 dry-run 回 $OK，
 *     目的是证明「ESP32(SoftAP) → 局域网 → 鲲鹏网关(STA) → 应答」这段通了；
 *   - 9001 端口连续上行 PCM（16kHz/16bit/单声道），鲲鹏 audio_sink.py 落 WAV；
 *   - 中文帧 $ASR,回初始位 默认关闭（SEND_CN_HOME_FRAME=0），
 *     因为网关 --apply 时它会真的让臂回初始位，确认安全前不许误触发。
 *   - ✅ 已实机验证：SoftAP 起来、鲲鹏(STA)接入、9000 文本帧往返、
 *     I2S0 采到非零 PCM；9001 音频上行待鲲鹏侧起 audio_sink.py 后验收。
 * ============================================================ */

static const char *TAG = "voice_main";

/* ------------------------------------------------------------
 * 帧表：链路自检用哪几帧
 *
 * 【概念】把「要发什么」集中写成一张表，而不是散在函数体里。
 *   加一条帧只改这张表、不动发送逻辑 —— 这就是「数据」和「代码」分开。
 * 【为什么先纯 ASCII】$ESTOP / $RESUME 走 voice_gateway 的
 *   special_text() 快速通道，不依赖中文编码，能先把「网络通不通」
 *   和「中文编码对不对」两个问题分开定位
 *   （详 → docs/ESP32-S3接入与阶段收尾路线.md 3.3 实证）。
 * ------------------------------------------------------------ */
typedef struct {
    const char *payload;   /* '$' 与 '*' 之间的内容，交给 protocol_build_frame() */
    const char *note;      /* 打印给人看的一句话 */
} link_frame_t;

static const link_frame_t s_link_frames[] = {
    { "ESTOP",  "纯 ASCII 快速通道：只验链路，网关回 $OK,ESTOP" },
    { "RESUME", "纯 ASCII 快速通道：解除急停，网关回 $OK,RESUMED" },
};

/* "ASR,回初始位" 的 UTF-8 字节串。
 * 【为什么写成八进制 \ooo 而不直接写中文】源码文件编码 / 编译器执行字符集
 *   在不同机器上可能不一致；写成字节值就与鲲鹏侧 printf 八进制兜底、
 *   以及 Python 的 .encode("utf-8") 完全一致，CK 一定算得对。
 *   回 345 233 236 / 初 345 210 235 / 始 345 247 213 / 位 344 275 215
 *   ⇒ payload 共 4 + 12 = 16 字节，逐字节 XOR 的 CK = 0x61。
 * 【规范定义】C99 的 \ooo 是「八进制转义，最多 3 位」，长度固定，
 *   不像 \xNN 会一直吞后面的十六进制字符，拼接多字节时更安全。
 * 【对照】docs/上板验收-语音网关阶段2.md 里板端 printf 用的是同一串字节。 */
#define CN_HOME_PAYLOAD  "ASR,\345\233\236\345\210\235\345\247\213\344\275\215"

/* 🔴 默认 0：中文帧在网关 --apply 时会真的下发 HOME、让臂回初始位。
 *   明天回教室确认「网关已 --apply + 臂已摆 C 姿态 + 周围无人」再改成 1 重烧。 */
#define SEND_CN_HOME_FRAME  0

/* ------------------------------------------------------------
 * 本地封帧自检：只打印，不联网
 * 【为什么单独留这一步】Wi-Fi 没连上 / 网关没起时，也要能证明
 *   protocol.c 造出来的帧是对的，别把「网络错」和「帧错」混成一次故障。
 * ------------------------------------------------------------ */
static void local_frame_selfcheck(void)
{
    char frame[96];

    for (size_t i = 0; i < sizeof(s_link_frames) / sizeof(s_link_frames[0]); i++) {
        int len = protocol_build_frame(s_link_frames[i].payload, frame, sizeof(frame));
        if (len > 0) {
            ESP_LOGI(TAG, "[本地封帧 %d] %s（长度=%d，只打印不下发）",
                     (int)i, frame, len);
        } else {
            ESP_LOGE(TAG, "[本地封帧 %d] 失败：frame[] 太小", (int)i);
        }
    }

#if SEND_CN_HOME_FRAME
    int cn_len = protocol_build_frame(CN_HOME_PAYLOAD, frame, sizeof(frame));
    if (cn_len > 0) {
        ESP_LOGI(TAG, "[本地封帧 中文] %s（长度=%d）", frame, cn_len);
    }
#endif
}

/* ------------------------------------------------------------
 * TCP 链路自检：造帧 → 发给鲲鹏网关 → 打印应答
 * 【为什么用 protocol_build_frame() 而不是硬写 "$ESTOP*5D\r\n"】
 *   CK 由封帧函数按字节算，避免手算错、也避免以后改词表忘了改 CK；
 *   这正是「统一帧语法只有一份真源」的意思。
 * ------------------------------------------------------------ */
static void tcp_link_check(void)
{
    char frame[96];
    char reply[TCP_CLIENT_REPLY_SIZE];

    for (size_t i = 0; i < sizeof(s_link_frames) / sizeof(s_link_frames[0]); i++) {
        int len = protocol_build_frame(s_link_frames[i].payload, frame, sizeof(frame));
        if (len <= 0) {
            ESP_LOGE(TAG, "封帧失败，跳过发送：%s", s_link_frames[i].payload);
            continue;
        }

        ESP_LOGI(TAG, "准备发送：%s —— %s",
                 s_link_frames[i].payload, s_link_frames[i].note);
        int rc = tcp_client_send(GATEWAY_HOST, GATEWAY_PORT, frame,
                                 reply, sizeof(reply));
        if (rc >= 0) {
            ESP_LOGI(TAG, "网关应答（%d 字节）: %s", rc, reply);
        } else {
            /* 失败不 return：让后面几帧和 I2S 自检照样跑，
             * 一次上板能把「哪几帧不通」一次看全，排障更快。 */
            ESP_LOGE(TAG, "发送失败 rc=%d: %s（继续跑下一帧）",
                     rc, tcp_client_strerror(rc));
        }
        /* 【为什么要让一让】网关是单连接短事务，两条之间给一点间隔，
         * 避免上一轮 close 还没被对端处理完就又 connect。 */
        vTaskDelay(pdMS_TO_TICKS(200));
    }

#if SEND_CN_HOME_FRAME
    {
        int len = protocol_build_frame(CN_HOME_PAYLOAD, frame, sizeof(frame));
        if (len > 0) {
            ESP_LOGW(TAG, "中文帧已启用：确认网关 --apply 且臂已摆 C 姿态才会动");
            int rc = tcp_client_send(GATEWAY_HOST, GATEWAY_PORT, frame,
                                     reply, sizeof(reply));
            if (rc >= 0) {
                ESP_LOGI(TAG, "中文帧网关应答（%d 字节）: %s", rc, reply);
            } else {
                ESP_LOGE(TAG, "中文帧发送失败 rc=%d: %s",
                         rc, tcp_client_strerror(rc));
            }
        }
    }
#else
    ESP_LOGI(TAG, "中文帧默认关闭（SEND_CN_HOME_FRAME=0），明天联调再开");
#endif
}

void app_main(void)
{
    ESP_LOGI(TAG, "==== ESP32-S3 语音入口骨架启动（含 TCP 上行自检） ====");
    ESP_LOGW(TAG, "只采音 / 只发链路自检帧；未接 WakeNet、未接 ASR、未直接驱动舵机");

    /* ---------- 第 1 步：开 SoftAP 热点 ----------
     * 【为什么先开】TCP 上行要等鲲鹏作为 STA 连进来；开失败也别直接 return，
     *   让本地封帧自检 + I2S 采音自检照样跑，这样热点没开或麦克风没接
     *   都能单独定位，不互相掩盖。 */
    esp_err_t err = wifi_ap_start();
    bool net_ok = (err == ESP_OK) && wifi_ap_is_started();
    if (!net_ok) {
        ESP_LOGE(TAG, "SoftAP 启动失败，跳过 TCP 上行；本地封帧自检 + I2S 自检照跑");
    } else {
        ESP_LOGI(TAG, "SoftAP 已启动 192.168.77.1；网关目标 %s:%d",
                 GATEWAY_HOST, GATEWAY_PORT);
    }

    /* ---------- 第 2 步：本地封帧自检（不联网） ---------- */
    local_frame_selfcheck();

    /* ---------- 第 3 步：TCP 上行自检（仅在 Wi-Fi 通时） ----------
     * 【顺序】先验本地帧、再验网络，出问题能一眼分是「帧错」还是「网错」。 */
    if (net_ok) {
        tcp_link_check();
    }

    /* ---------- 第 4 步：启 I2S0 采音 ---------- */
    err = i2s_mic_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2S 启动失败，停止自检。先查 BCLK/WS/DIN 三根线与 L/R 引脚");
        return;
    }

    /* ---------- 第 5 步：采音自检，确认麦在出数 ----------
     * 【为什么先自检再交任务】麦没接 / L/R 悬空时，得让「硬件问题」
     *   和「网络问题」分开暴露；这一步只读一批打印前 4 个采样。 */
    int32_t probe[256];
    int pn = i2s_mic_read(probe, 256);
    if (pn > 0) {
        ESP_LOGI(TAG, "采音自检：%d 个采样 %d %d %d %d",
                 pn, (int)probe[0], (int)probe[1], (int)probe[2], (int)probe[3]);
    } else {
        ESP_LOGW(TAG, "采音自检：没读到采样，查 BCLK/WS/DIN 接线、L/R 引脚、3.3V 供电");
    }

    /* ---------- 第 6 步：启动 PCM 上行任务（TCP 9001） ---------- */
    if (net_ok) {
        err = audio_tx_start();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "音频上行任务启动失败");
        }
    } else {
        ESP_LOGW(TAG, "SoftAP 未就绪，跳过音频上行；采音自检已单独完成");
    }
}
