#include "wifi_sta.h"

#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

static const char *TAG = "wifi_sta";

/* 【概念】事件组 EventGroup = 一组「旗子/比特位」，任务可以等某几根旗子被立起来。
 *   这里用两根：BIT0=连上了，BIT1=重试太多次彻底放弃。
 * 【为什么不用一个全局变量 while 轮询】轮询会空转吃满 CPU；
 *   事件组让任务在等的时候睡眠，等到了由回调唤醒，这是 FreeRTOS 的标准做法。 */
#define WIFI_CONNECTED_BIT  (1 << 0)
#define WIFI_FAIL_BIT       (1 << 1)

static EventGroupHandle_t s_wifi_event_group = NULL;
static int  s_retry_num = 0;
static bool s_connected = false;

/* ---------- 事件回调：连上 / 断开 / 拿到 IP 都进这一个函数 ---------- */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    /* 【概念】回调 callback：不是我们主动调，而是「事情发生后系统反过来调我们」。
     *   event_base 区分是哪一类事件（WIFI_EVENT / IP_EVENT），event_id 区分具体哪一种。 */
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        /* STA 刚启动，主动发起第一次连接 */
        esp_wifi_connect();

    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        /* 断了就重试，重试到上限才立「失败」旗 */
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "断开，重连第 %d 次", s_retry_num);
        } else {
            s_connected = false;
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        /* 拿到 IP 才算真的连上（只是 WIFI_EVENT_STA_CONNECTED 还不算能通信） */
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "拿到 IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        s_connected = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

esp_err_t wifi_sta_start(void)
{
    /* ---------- 第 1 步：NVS ----------
     * 【概念】NVS = Non-Volatile Storage，掉电不丢的小仓库。
     *   Wi-Fi 驱动要把射频校准数据存进去，不先初始化会直接报错。
     * 【坑点】NVS 分区满 / 版本旧时 nvs_flash_init() 会返回特定错误码，
     *   这时擦掉重来是官方推荐写法，不是「暴力清数据」。 */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init 失败: %s", esp_err_to_name(err));
        return err;
    }

    /* ---------- 第 2 步：网络接口 + 默认事件循环 ---------- */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    /* 【顺序坑】事件组必须「先创建、后注册回调」。
     *   回调里会 xEventGroupSetBits()，若那时句柄还是 NULL 就是空指针写，直接崩。 */
    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* ---------- 第 3 步：初始化 Wi-Fi 驱动并注册回调 ---------- */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* 【规范】v5.x 推荐用 esp_event_handler_instance_register（带句柄可反注册），
     *   老写法 esp_event_handler_register 也能用但不好管理生命周期。 */
    esp_event_handler_instance_t instance_any_id = NULL;
    esp_event_handler_instance_t instance_got_ip = NULL;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    /* ---------- 第 4 步：填 SSID/密码，切 STA 模式，启动 ---------- */
    wifi_config_t wifi_config = { 0 };   /* { 0 } 先把整块清零，别让残留字节混进密码 */
    strncpy((char *)wifi_config.sta.ssid,     WIFI_SSID,     sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, WIFI_PASSWORD, sizeof(wifi_config.sta.password) - 1);
    /* 【为什么设 authmode】限定最低加密方式，避免连到同名但不安全的假热点 */
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* ---------- 第 5 步：等结果 ----------
     * pdFALSE/pdFALSE = 等到后不自动清旗、不需要「所有位同时成立」，任一即可。
     * portMAX_DELAY = 一直等；生产代码应改有限超时，这里骨架求简单。 */
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Wi-Fi 已连上: %s", WIFI_SSID);
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Wi-Fi 连不上 %s（已重试 %d 次）", WIFI_SSID, WIFI_MAX_RETRY);
    return ESP_FAIL;
}

bool wifi_sta_is_connected(void)
{
    return s_connected;
}
