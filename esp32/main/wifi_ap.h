#ifndef WIFI_AP_H
#define WIFI_AP_H

#include <stdbool.h>
#include "esp_err.h"

/* ============================================================
 * wifi_ap —— ESP32-S3 当 Wi-Fi SoftAP（热点）
 *
 * 【概念（大白话）】AP = Access Point，让 ESP32 自己变成一个热点，
 *   手机 / 鲲鹏作为 STA 连进来，都落在同一个局域网里。
 * 【规范定义】IEEE 802.11 infrastructure mode 中的接入点；
 *   ESP-IDF 用 esp_netif + esp_event + esp_wifi 三件套完成，
 *   本工程把默认的 192.168.4.1/24 主动改成唯一网段 192.168.77.1/24
 *   （避开教室/家用路由器常见的 192.168.0.x / 1.x / 4.x），并自带 DHCP 给 STA 发地址。
 *
 * 🔴 诚实边界：本模块只负责「把热点开起来、让设备能连进来」，
 *    不负责音频、封帧、协议转发；那些仍由各自模块处理。
 * ============================================================ */

/* ⚠️ 运行前改成你自己的热点名与密码（密码至少 8 位，WPA2 要求）。
 * 下面 `Kunpeng-Robot` / `12345678` 是演示默认值，可提交进 Git；
 * 正式部署 / 外出演示前请改成本地值，或后续迁移到 Kconfig 注入。 */
#define AP_SSID       "Kunpeng-Robot"
#define AP_PASSWORD   "12345678"
#define AP_MAX_CONN   4          /* 手机 + 鲲鹏，留余量 */

/* 启动 SoftAP 并返回结果；成功返回 ESP_OK */
esp_err_t wifi_ap_start(void);

/* 查询热点是否已启动（供主循环决定是否尝试上报） */
bool wifi_ap_is_started(void);

#endif /* WIFI_AP_H */
