#include "tcp_client.h"

#include <stdio.h>        /* snprintf */
#include <string.h>       /* memset / strlen */
#include <errno.h>
#include <sys/time.h>     /* struct timeval */
#include <sys/socket.h>   /* socket / connect / send / recv / setsockopt */
#include <netdb.h>        /* getaddrinfo / freeaddrinfo / struct addrinfo */
#include <unistd.h>       /* close */

#include "esp_log.h"

/* lwIP 说明（【规范】）：ESP-IDF v5.x 里上面这几个 BSD socket 头由
 * lwip 组件提供，所以 main/CMakeLists.txt 的 REQUIRES 必须加 lwip。
 * 只 include 不写依赖，链接期会报 undefined reference to getaddrinfo。 */

static const char *TAG = "tcp_client";

const char *tcp_client_strerror(int rc)
{
    switch (rc) {
    case TCP_ERR_ARG:     return "参数为空 / 端口为 0 / 缓冲区太小";
    case TCP_ERR_DNS:     return "getaddrinfo 解析主机失败（IP 写错？）";
    case TCP_ERR_SOCKET:  return "socket() 创建失败（fd 用尽？）";
    case TCP_ERR_CONNECT: return "connect() 失败：查网关 IP / 网关须 --host 0.0.0.0 / 两边同一热点";
    case TCP_ERR_SEND:    return "send() 未把整帧发完";
    case TCP_ERR_RECV:    return "recv() 超时或失败：网关没回应 / 网络断了";
    default:              return "未知错误";
    }
}

int tcp_client_send(const char *host, uint16_t port, const char *frame,
                    char *reply, size_t reply_size)
{
    /* ---------- 第 0 步：入口先挡住明显不能干的情况 ----------
     * 【新手习惯】C 不会替你检查空指针，入口挡一次比后面崩了再查便宜得多。 */
    if (host == NULL || frame == NULL || reply == NULL || reply_size == 0 || port == 0) {
        return TCP_ERR_ARG;
    }
    size_t frame_len = strlen(frame);
    if (frame_len == 0) {
        return TCP_ERR_ARG;
    }
    /* 先清零：recv 收到多少就用多少，别把上次的旧字节当成本次应答打印 */
    memset(reply, 0, reply_size);

    /* ---------- 第 1 步：把 "10.160.120.1" 变成能喂给 socket 的地址 ----------
     * 【概念】getaddrinfo() 是「地址翻译官」：给它主机字符串和端口，
     *   它回一串可直接用的地址结构。IP 字面量也走它，代码只需一套写法。
     * 【为什么不用 inet_addr()】后者只认 IPv4 点分十进制、不支持域名，
     *   也不支持 IPv6；getaddrinfo 是 POSIX 推荐的与协议无关写法。
     * 【为什么 ai_family = AF_INET】鲲鹏网关当前只有 IPv4 地址，
     *   限定 IPv4 能避免解析出一堆用不上的 IPv6 结果。 */
    char port_str[8];
    snprintf(port_str, sizeof(port_str), "%u", (unsigned)port);

    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_STREAM;    /* TCP 字节流，不是 UDP 数据报 */

    struct addrinfo *res = NULL;
    int gai_rc = getaddrinfo(host, port_str, &hints, &res);
    if (gai_rc != 0 || res == NULL) {
        ESP_LOGE(TAG, "getaddrinfo(%s:%s) 失败: rc=%d", host, port_str, gai_rc);
        return TCP_ERR_DNS;
    }

    /* ---------- 第 2 步：建 socket ---------- */
    int sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() 失败: errno=%d", errno);
        freeaddrinfo(res);
        return TCP_ERR_SOCKET;
    }

    /* 【为什么必须设超时】不设的话，网关没起 / 网线断了，recv() 会一直挂着，
     *   连后面的 I2S 采音循环都跑不到，串口看起来像「整机死机」。
     *   SO_RCVTIMEO / SO_SNDTIMEO 是 BSD socket 标准选项，lwIP 支持。 */
    struct timeval tv;
    tv.tv_sec  = TCP_CLIENT_TIMEOUT_SEC;
    tv.tv_usec = 0;
    /* 🔴 返回值必须检查：设置失败时 recv() 会退化成「无限阻塞」，
     * 串口看起来像整机死机，却查不出是超时没生效 —— 与上面注释的承诺自相矛盾。 */
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) != 0 ||
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) != 0) {
        ESP_LOGW(TAG, "setsockopt 超时设置失败: errno=%d（recv 可能阻塞）", errno);
    }

    /* ---------- 第 3 步：connect ---------- */
    if (connect(sock, res->ai_addr, res->ai_addrlen) != 0) {
        ESP_LOGE(TAG, "connect(%s:%s) 失败: errno=%d", host, port_str, errno);
        close(sock);
        freeaddrinfo(res);
        return TCP_ERR_CONNECT;
    }
    freeaddrinfo(res);   /* 地址链表用完立刻释放，别泄漏 */
    ESP_LOGI(TAG, "TCP connected -> %s:%s", host, port_str);

    /* ---------- 第 4 步：send 整帧 ----------
     * 【坑点】TCP 是字节流，send() 不保证一次把 len 个字节全发走
     *   （返回值可能小于 len）。骨架帧只有十几字节、几乎不会分片，
     *   但仍写循环才是对的：累加已发字节，返回 <=0 才算真失败。 */
    size_t sent = 0;
    while (sent < frame_len) {
        int n = send(sock, frame + sent, frame_len - sent, 0);
        if (n <= 0) {
            ESP_LOGE(TAG, "send() 返回 %d, errno=%d", n, errno);
            close(sock);
            return TCP_ERR_SEND;
        }
        sent += (size_t)n;
    }
    /* 【关于中文帧的显示】纯 ASCII 帧（$ESTOP / $RESUME）在串口 monitor
     *   能正常显示；中文 UTF-8 帧在部分终端会显示成乱码，但那只是
     *   「终端怎么显示」的问题 —— 真正发出去的字节由 protocol_build_frame()
     *   按字节拼、CK 也按字节算，乱码不影响网关解析。别因此改用宽字符。 */
    ESP_LOGI(TAG, "send %d bytes: %s", (int)frame_len, frame);

    /* ---------- 第 5 步：recv 一帧应答 ----------
     * 【为什么减 1】故意只收 reply_size-1 个字节，留最后一格给 '\0'，
     *   这样调用方可以直接把 reply 当 C 字符串用 %s 打印。 */
    int n = recv(sock, reply, reply_size - 1, 0);
    if (n <= 0) {
        ESP_LOGE(TAG, "recv() 返回 %d, errno=%d（超时？网关没起？）", n, errno);
        close(sock);
        return TCP_ERR_RECV;
    }
    reply[n] = '\0';
    ESP_LOGI(TAG, "recv %d bytes: %s", n, reply);

    /* ---------- 第 6 步：close ----------
     * 【为什么必须关】lwIP 的 socket fd 是有限资源，一次一连接若不关，
     *   多发几十次就把可用 fd 耗光，之后 connect 直接失败，
     *   而且现象很容易被误判成「Wi-Fi 掉了」。 */
    close(sock);
    return n;
}
