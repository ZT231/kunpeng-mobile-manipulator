#ifndef WIFI_STA_H
#define WIFI_STA_H

#include <stdbool.h>
#include "esp_err.h"

/* ============================================================
 * wifi_sta —— Wi-Fi STA（站点）连接骨架
 *
 * 【概念（大白话）】STA = 让 ESP32 像手机一样「去连一个热点」，
 *   另一种 AP 模式是「ESP32 自己变成热点让别人连」。这里用 STA。
 * 【规范定义】IEEE 802.11 infrastructure mode 中的 station；
 *   ESP-IDF 用 esp_netif + esp_event + esp_wifi 三个组件协作完成，
 *   连上后由 DHCP 拿到 IPv4 地址（IP_EVENT_STA_GOT_IP 事件）。
 *
 * 🔴 诚实边界：骨架只证明「能连上、能拿 IP」，
 *    没有 TCP 客户端、没有把文本发到鲲鹏、没接主链路。
 * ============================================================ */

/* ⚠️ 运行前改成你自己的热点（PC 与鲲鹏板也必须连同一个热点/路由）。
 * 【项目惯例】根 README 已说明：仓库里的 SSID/密码一律是脱敏占位，
 *   真实凭据只在本地改、不要提交进 Git（这就是这两行写 YOUR_* 的原因）。 */
#define WIFI_SSID       "YOUR_HOTSPOT"
#define WIFI_PASSWORD   "YOUR_PASSWORD"
#define WIFI_MAX_RETRY  5

/* 启动 STA 并阻塞等待「拿到 IP」或「重试耗尽」；成功返回 ESP_OK */
esp_err_t wifi_sta_start(void);

/* 查询当前是否已连上（供主循环决定要不要上报状态） */
bool wifi_sta_is_connected(void);

#endif /* WIFI_STA_H */
