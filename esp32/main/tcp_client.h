#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

#include <stddef.h>
#include <stdint.h>

/* ============================================================
 * tcp_client —— ESP32 → 鲲鹏语音网关的 TCP 上行层
 *
 * 【概念（大白话）】把「一句话」通过 Wi-Fi 发给鲲鹏板上那个已经实机
 *   验收过的收帧服务，再等它回一句「收到 / 拒绝」。像打电话：
 *   先拨号(connect)、说话(send)、听回话(recv)、挂断(close)。
 * 【规范定义】BSD socket（POSIX 接口，本项目由 lwIP 实现，RFC 9293）：
 *   getaddrinfo() 解析主机 → socket() 建套接字 → connect() 三次握手 →
 *   send()/recv() 收发可靠字节流 → close() 释放资源。面向连接、不丢不重排。
 *
 * 🔴 诚实边界：
 *   - 这一层只负责「把已经封好的帧送出去、把应答收回来」；
 *     帧的内容由 protocol.c 生成，意图理解 / 仲裁 / 下发全在鲲鹏侧；
 *   - 本层**不解析应答语义**（$OK / $ERR 只是原样交回调用方打印），
 *     不驱动任何舵机、不碰 STM32、不改 .ioc；
 *   - **未上板联调**：connect 成功与否取决于明天教室里的实际热点与网关 IP。
 * ============================================================ */

/* 鲲鹏收帧网关地址 —— 🔴 明天回教室按实际网络改这两个宏再烧录。
 * 【为什么用宏而不是写死在函数里】不同网络下鲲鹏的 IP 会变：
 *   USB 共享网络时才是记忆里的 10.160.120.1；教室热点下是路由器
 *   分配的局域网 IP。集中放在头文件一处，改一次重新 build 即可。
 * 【为什么不在这里写 Wi-Fi 密码】凭据放 wifi_sta.h，且只在本地副本改、
 *   不进 Git（见该文件第 20~22 行的项目惯例）。
 * 【规范】端口 9000 = kunpeng/robot/voice_gateway.py 的 DEFAULT_PORT；
 *   该服务默认 bind 127.0.0.1，要 ESP32 进来必须显式 --host 0.0.0.0。 */
#define GATEWAY_HOST   "10.160.120.1"
#define GATEWAY_PORT   9000

/* 收 / 发超时（秒）：网关没起来或网络断了，不能让任务永久卡在 recv()。 */
#define TCP_CLIENT_TIMEOUT_SEC   3

/* 应答缓冲区：网关应答最长约 "$OK,LOOK_NOOP*6E\r\n" ≈ 20 字节，留足余量。 */
#define TCP_CLIENT_REPLY_SIZE    128

/* 负返回值错误码。用枚举而不是裸 -1/-2：打印日志时一眼看懂是哪一步挂了。
 * 【注意】成功时函数返回「收到的应答字节数」（>=0），不是这里的枚举值。 */
typedef enum {
    TCP_ERR_ARG     = -1,   /* 参数为空 / 端口为 0 / 缓冲区太小 */
    TCP_ERR_DNS     = -2,   /* getaddrinfo 解析主机失败 */
    TCP_ERR_SOCKET  = -3,   /* socket() 创建失败 */
    TCP_ERR_CONNECT = -4,   /* connect() 失败：网关没起 / IP 不对 / 不同网段 */
    TCP_ERR_SEND    = -5,   /* send() 没把整帧发完 */
    TCP_ERR_RECV    = -6,   /* recv() 超时 / 出错 / 对端先关 */
} tcp_client_err_t;

/**
 * 发一帧、收一帧（一次一连接）。
 *
 * @param host        网关主机名或 IP 字符串
 * @param port        网关 TCP 端口
 * @param frame       已经封好的完整帧（含 \r\n），由 protocol_build_frame() 产出
 * @param reply       输出：应答字节（函数保证以 '\0' 结尾，可直接 %s 打印）
 * @param reply_size  reply 缓冲区大小
 * @return >=0 成功，值为收到的应答字节数；<0 失败，见 tcp_client_err_t
 *
 * 【为什么一次一连接】骨架阶段帧是「发一句、等一句」的短事务，
 *   长连接要额外处理断线重连 / 心跳，现在用不上；一次一连接最简单，
 *   也最不容易漏关 fd。将来要连续上报遥测再改成长连接不迟。
 */
int tcp_client_send(const char *host, uint16_t port, const char *frame,
                    char *reply, size_t reply_size);

/* 把负错误码翻成一句人话，方便 ESP_LOGE 直接打印 */
const char *tcp_client_strerror(int rc);

#endif /* TCP_CLIENT_H */
