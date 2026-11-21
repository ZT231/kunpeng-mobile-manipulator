#include "wifi_ap.h"

#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"

static const char *TAG = "wifi_ap";
static bool s_started = false;

esp_err_t wifi_ap_start(void)
{
    /* ---------- 第 1 步：NVS ----------
     * 【为什么】Wi-Fi 驱动要把射频校准数据存进 NVS，不初始化会报错。
     *   和 STA 版同一套逻辑：满了 / 版本旧就擦掉重来。 */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init 失败: %s", esp_err_to_name(err));
        return err;
    }

    /* ---------- 第 2 步：网络接口 + 事件循环 + 默认 AP 接口 ----------
     * 【为什么用 esp_netif_create_default_wifi_ap()】它会把 AP 网段
     *   配成官方默认的 192.168.4.1/24 并挂上 DHCP 服务器；
     *   随后我们立刻改成唯一网段（见第 2.5 步），避免与教室/家用路由器撞网。 */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    if (ap_netif == NULL) {
        ESP_LOGE(TAG, "创建默认 AP 接口失败");
        return ESP_FAIL;
    }

    /* ---------- 第 2.5 步：把默认 192.168.4.1/24 改成唯一网段 ----------
     * 【为什么】官方默认 192.168.4.1/24 极易与教室/家用路由器同网段冲突；
     *   一旦撞网，PC 去 192.168.77.100 的包会被另一张网卡抢走，
     *   表现为「时通时不通」，还查不出原因。
     * 【顺序】必须先停 DHCP 服务器 → 再改 IP → 最后重新起 DHCP，
     *   否则 DHCP 池仍按旧网段发地址，连进来的 STA 拿不到新网段 IP。 */
    esp_netif_ip_info_t ap_ip;
    memset(&ap_ip, 0, sizeof(ap_ip));
    ap_ip.ip.addr      = ESP_IP4TOADDR(192, 168, 77, 1);   /* AP 自身地址 */
    ap_ip.gw.addr      = ESP_IP4TOADDR(192, 168, 77, 1);   /* 网关同 AP */
    ap_ip.netmask.addr = ESP_IP4TOADDR(255, 255, 255, 0);
    esp_netif_dhcps_stop(ap_netif);                        /* 已停会返回错误，忽略 */
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ap_ip));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));      /* 池按 192.168.77.x 重发 */

    /* ---------- 第 3 步：初始化 Wi-Fi 驱动 ---------- */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* ---------- 第 4 步：填热点参数，切 AP 模式，启动 ---------- */
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = strlen(AP_SSID),
            .channel = 1,
            .password = AP_PASSWORD,
            .max_connection = AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,   /* WPA2：密码至少 8 位 */
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    s_started = true;
    ESP_LOGI(TAG, "SoftAP 已启动: SSID=%s, IP=192.168.77.1, 最大连接=%d",
             AP_SSID, AP_MAX_CONN);
    return ESP_OK;
}

bool wifi_ap_is_started(void)
{
    return s_started;
}
