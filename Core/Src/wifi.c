/* ================= ESP-01S WiFi 控制源 =================
 *
 * WiFi 与蓝牙(bt.c)在架构上完全相同：ESP-01S 被实现为"接入通用指令协议栈的
 * 一路串口控制源"：
 *   - USART3 底层驱动（huart3：PB10=TX、PB11=RX，115200）手写在 usart.c 的
 *     USER CODE 区（工程为 CubeMX 虚拟模式，不会自动生成 USART3）；
 *   - USART3_IRQHandler 在 stm32f4xx_it.c 的 USER CODE 区实现（与 USART1/2 对称）；
 *   - 本模块调用 UartProto_Init() 把 WiFi 串口注册进协议栈；
 *   - 之后 WiFi 收到的 $CMD... 帧与 USART1/USART2 走同一解析引擎，
 *     响应也会被协议栈发回"最后发出命令'$'的那个串口"（即 WiFi 自己）。
 *
 * 开机自动配置：ESP-01S 断电重启后 CIPSERVER 等连接配置会丢失。
 * WIFI_Init 在注册协议栈前，先用阻塞方式发 AT 指令序列，把 ESP 配成 AP 热点，
 * 并开启 TCP server（CIPSERVER）监听 9000。
 *
 * 架构：ESP 做 TCP server（CIPSERVER=1），手机/电脑 TCP 工具直连
 *   ESP 自建热点 后，主动连 192.168.4.1:9000。
 *   注意：CIPSERVER 必须配合 CIPMUX=1（多连接），与透传(CIPMODE=1)互斥。
 *   ESP 非透传模式下，手机发来的 TCP 数据以 "+IPD,<link>,<len>:" 前缀上报，
 *   STM32 应答需先用 AT+CIPSEND=<link>,<len> 封装——这两处适配在
 *   uart_protocol.c 的 USART3 专用收发逻辑中处理。
 *
 * 前提：手机/电脑连接 ESP 自建的热点（SSID/密码见下方 WIFI_AP_SSID/WIFI_AP_PASS），
 *   TCP 连接 192.168.4.1:9000 即可控制机械臂，无需电脑 TCP server。
 */

#include "wifi.h"
#include "uart_protocol.h"
#include <stdio.h>
#include <string.h>

#define WIFI_AP_SSID     "YOUR_SSID"      /* 改成你自己的热点名 */
#define WIFI_AP_PASS     "YOUR_PASSWORD"  /* 改成你自己的热点密码 */
#define WIFI_TCP_PORT    9000U            /* ESP CIPSERVER 监听端口 */

static void wifi_send_at(UART_HandleTypeDef *huart, const char *cmd)
{
    char buf[80];
    int len = snprintf(buf, sizeof(buf), "%s\r\n", cmd);
    if (len > 0 && len < (int)sizeof(buf))
        (void)HAL_UART_Transmit(huart, (uint8_t *)buf, (uint16_t)len, 200);
}

/* 轮询读 ESP 回显，直到出现 match 指定字符串（覆盖 busy p... / +CIFSR 等）。
 * USART3 此时尚未注册进协议栈（无中断接收），故用 polling 读。
 * 返回 1=已匹配；0=超时。
 * 注意：AT+RST 后 ESP 先回 "OK"（确认收到），然后才重启输出 "ready"，
 *   所以 RST 必须传 match="ready"，否则会在 "OK" 处提前返回、ESP 还没重启完。 */
static int wifi_wait_str(UART_HandleTypeDef *huart, const char *match, uint32_t timeout_ms)
{
    uint8_t c;
    char ring[32];
    uint8_t n = 0U;
    uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < timeout_ms)
    {
        if (HAL_UART_Receive(huart, &c, 1U, 25) != HAL_OK)
        {
            continue;
        }
        if (n < (uint8_t)sizeof(ring) - 1U)
        {
            ring[n++] = (char)c;
        }
        else
        {
            memmove(ring, ring + 1, sizeof(ring) - 2U);
            ring[sizeof(ring) - 2U] = (char)c;
        }
        ring[n] = '\0';
        if (strstr(ring, match) != NULL)
        {
            (void)HAL_UART_Receive(huart, &c, 1U, 50);
            return 1;
        }
    }
    return 0;
}

void WIFI_Init(UART_HandleTypeDef *huart)
{
    uint8_t drain;

    /* 等 ESP-01S 上电就绪 */
    HAL_Delay(2000);

    /* AT 配置序列：发一条、等 ESP 回 OK/ready 再发下一条，
     * 彻底规避固定延时撞上 busy p... 导致 CIPSERVER 偶发不生效的问题。 */
    wifi_send_at(huart, "AT");
    wifi_wait_str(huart, "OK", 3000U);

    wifi_send_at(huart, "AT+CWMODE=2");
    wifi_wait_str(huart, "OK", 3000U);

    /* CWMODE 改变后重启 ESP，确保模式切换生效。
     * RST 后 ESP 先回 "OK"（确认），然后才重启输出 "ready"，
     * 必须等 "ready" 而非 "OK"，否则后续命令会发到正在重启的 ESP 上。 */
    wifi_send_at(huart, "AT+RST");
    wifi_wait_str(huart, "ready", 6000U);

    /* 重启后重新唤醒 AT 引擎 */
    wifi_send_at(huart, "AT");
    wifi_wait_str(huart, "OK", 3000U);

    wifi_send_at(huart, "AT+CWSAP=\"" WIFI_AP_SSID "\",\"" WIFI_AP_PASS "\",5,3");
    wifi_wait_str(huart, "OK", 4000U);

    /* ESP 做 TCP server：多连接 + 开启 server 监听 9000。
     * CIPSERVER 依赖 CIPMUX=1（多连接），且与透传(CIPMODE=1)互斥，
     * 因此这里不进入透传，走 "+IPD...+CIPSEND" 封装交互（见 uart_protocol.c）。 */
    wifi_send_at(huart, "AT+CIPMUX=1");
    wifi_wait_str(huart, "OK", 3000U);

    {
        char cmd[40];
        int n = snprintf(cmd, sizeof(cmd), "AT+CIPSERVER=1,%u", (unsigned)WIFI_TCP_PORT);
        if (n > 0 && n < (int)sizeof(cmd))
            wifi_send_at(huart, cmd);
    }
    wifi_wait_str(huart, "OK", 3000U);

    /* 排空 UART 接收缓冲中残留的 ESP 回显字节，
     * 避免注册协议栈后这些字节被误当作命令数据喂给解析器。 */
    while (HAL_UART_Receive(huart, &drain, 1U, 10) == HAL_OK) { }

    /* 配置完成，注册协议栈，开始接收命令帧 */
    UartProto_Init(huart);
}
