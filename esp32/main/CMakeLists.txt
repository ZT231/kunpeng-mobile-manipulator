# main 组件的构建清单
#
# 【概念】ESP-IDF 把代码按「组件 component」组织，每个组件自己写一份 CMakeLists.txt
#        告诉构建系统：我有哪些 .c、我的头文件在哪、我依赖哪些官方组件。
# 【规范】idf_component_register() 是官方注册宏；SRCS 列源文件，
#        INCLUDE_DIRS "." 表示本目录的 .h 可以被 #include "xxx.h" 直接找到，
#        REQUIRES 列依赖（不写会在编译时报「找不到 driver/i2s_std.h」这类错）。
#
# 🔴 11-14 新增 tcp_client.c 与 lwip：
#    tcp_client.c 用了 BSD socket（socket/connect/send/recv/getaddrinfo），
#    这些函数在 ESP-IDF 里由 lwip 组件提供 <sys/socket.h> / <netdb.h>。
#    只 include 头、不写 REQUIRES lwip，链接期会报
#    undefined reference to `getaddrinfo' —— 这是加网络代码最容易漏的一行。

idf_component_register(
    SRCS
        "main.c"
        "i2s_mic.c"
        "wifi_ap.c"
        "protocol.c"
        "tcp_client.c"
        "audio_tx.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        driver
        esp_wifi
        esp_netif
        esp_event
        nvs_flash
        lwip
)
