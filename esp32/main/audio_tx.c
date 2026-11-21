#include "audio_tx.h"
#include "i2s_mic.h"

#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "audio_tx";

/* connect 阶段限时：对端不在线时，lwIP 的阻塞 connect 会因 TCP SYN
 * 重传卡到约 60 秒才报 EHOSTUNREACH。用非阻塞 connect + select 限 3 秒，
 * 这样鲲鹏晚几秒接入热点也能很快重试成功，不至于干等一分钟。 */
#define AUDIO_TX_CONNECT_TIMEOUT_SEC   3
#define AUDIO_TX_RECONNECT_DELAY_MS    1000
/* 每 64 次读取打一行心跳；16kHz 下 64×16ms ≈ 1 秒一行，不刷屏。 */
#define AUDIO_TX_HEARTBEAT_READS       64

/* ------------------------------------------------------------
 * 建一条到鲲鹏音频端口的 TCP 连接（非阻塞 connect + select 限时）
 * 成功返回 socket fd，失败返回 -1。
 * ------------------------------------------------------------ */
static int audio_tx_connect(void)
{
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() 失败: errno=%d", errno);
        return -1;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(AUDIO_TX_PORT);
    if (inet_pton(AF_INET, AUDIO_TX_HOST, &addr.sin_addr) != 1) {
        ESP_LOGE(TAG, "音频目标地址解析失败: %s", AUDIO_TX_HOST);
        close(sock);
        return -1;
    }

    int flags = fcntl(sock, F_GETFL, 0);
    if (flags < 0) {
        flags = 0;
    }
    if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) < 0) {
        ESP_LOGW(TAG, "设置非阻塞失败，改用阻塞 connect");
        int rc = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
        if (rc != 0) {
            close(sock);
            return -1;
        }
        return sock;
    }

    int rc = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
    if (rc != 0 && errno == EINPROGRESS) {
        fd_set wfds;
        FD_ZERO(&wfds);
        FD_SET(sock, &wfds);
        struct timeval tv;
        tv.tv_sec = AUDIO_TX_CONNECT_TIMEOUT_SEC;
        tv.tv_usec = 0;
        rc = select(sock + 1, NULL, &wfds, NULL, &tv);
        if (rc > 0) {
            int so_err = 0;
            socklen_t len = sizeof(so_err);
            if (getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_err, &len) == 0 && so_err != 0) {
                close(sock);
                return -1;
            }
        } else {
            /* 超时或 select 出错 */
            close(sock);
            return -1;
        }
    } else if (rc != 0) {
        close(sock);
        return -1;
    }

    /* 连上后恢复阻塞，后续 send 语义保持一致 */
    fcntl(sock, F_SETFL, flags);
    return sock;
}

/* ------------------------------------------------------------
 * 音频上行主任务：连上就持续「读麦 → 转 int16 → send」，
 * 断线则关连接重试。
 * ------------------------------------------------------------ */
static void audio_tx_task(void *arg)
{
    (void)arg;
    int16_t pcm16[AUDIO_TX_SAMPLES_PER_SEND];

    while (1) {
        int sock = audio_tx_connect();
        if (sock < 0) {
            ESP_LOGW(TAG, "连接 %s:%d 失败，%d ms 后重试",
                     AUDIO_TX_HOST, AUDIO_TX_PORT, AUDIO_TX_RECONNECT_DELAY_MS);
            vTaskDelay(pdMS_TO_TICKS(AUDIO_TX_RECONNECT_DELAY_MS));
            continue;
        }

        ESP_LOGI(TAG, "音频流已连上 %s:%d（16kHz/16bit/单声道）",
                 AUDIO_TX_HOST, AUDIO_TX_PORT);

        int32_t raw[AUDIO_TX_SAMPLES_PER_SEND];
        uint32_t total = 0;
        uint32_t reads = 0;
        bool link_ok = true;

        while (link_ok) {
            int n = i2s_mic_read(raw, AUDIO_TX_SAMPLES_PER_SEND);
            if (n <= 0) {
                /* 读超时（200ms 内没凑满）：立即再来，数据由 DMA 攒着 */
                continue;
            }

            for (int i = 0; i < n; i++) {
                /* i2s_mic_read 已把 32-bit 右移 8 位成 24-bit；
                 * 再右移 8 位取高 16 位，正好是无损截到 16-bit PCM。 */
                pcm16[i] = (int16_t)(raw[i] >> 8);
            }

            size_t to_send = (size_t)n * sizeof(int16_t);
            size_t sent = 0;
            while (sent < to_send) {
                int w = send(sock, (const char *)pcm16 + sent, to_send - sent, 0);
                if (w <= 0) {
                    ESP_LOGW(TAG, "send() 返回 %d, errno=%d，准备重连", w, errno);
                    link_ok = false;
                    break;
                }
                sent += (size_t)w;
            }

            total += (uint32_t)to_send;
            reads++;
            if ((reads % AUDIO_TX_HEARTBEAT_READS) == 0) {
                ESP_LOGI(TAG, "音频上行中：累计 %u 字节（最近采样 %d %d %d %d）",
                         (unsigned)total,
                         (int)pcm16[0], (int)pcm16[1], (int)pcm16[2], (int)pcm16[3]);
            }
        }

        close(sock);
    }
}

esp_err_t audio_tx_start(void)
{
    /* 栈 5120：两个 256 采样的 int16/int32 缓冲加调用帧足够；挂核 1，避开 WiFi 核 0 */
    BaseType_t r = xTaskCreatePinnedToCore(audio_tx_task, "audio_tx", 5120,
                                           NULL, 5, NULL, 1);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "创建音频上行任务失败");
        return ESP_FAIL;
    }
    return ESP_OK;
}
