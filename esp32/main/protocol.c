#include "protocol.h"

/* 【概念】查表法：把 0~15 映射成字符 '0'~'9'/'A'~'F'。
 *   比写 if/else 分支快也好读；注意必须大写，固件收包按大写 HEX 解析。
 * 【锚点】S2 实机跑通过的两帧，就是本实现要对上的答案：
 *   "PING"    XOR = 0x10 → $PING*10
 *   "OK,PONG" XOR = 0x3E → $OK,PONG*3E */
static const char HEX_DIGITS[] = "0123456789ABCDEF";

uint8_t protocol_xor_checksum(const uint8_t *payload, size_t len)
{
    uint8_t ck = 0;
    for (size_t i = 0; i < len; i++) {
        /* 【规范】按「字节」异或。uint8_t 明确告诉读代码的人：
         *   这里算的是 0~255 的字节，不是 wchar_t、不是 Unicode 码点。
         *   Python 侧中文必须先 encode("utf-8") 才能和这里对齐。 */
        ck ^= payload[i];
    }
    return ck;
}

int protocol_build_frame(const char *payload, char *out, size_t out_size)
{
    if (payload == NULL || out == NULL) {
        return -1;
    }

    /* C 字符串没有自带长度，先数出来。strlen 也能用，但这里手写一遍
     * 是为了让「payload 就是一串字节」这件事对新手更直观。 */
    size_t len = 0;
    while (payload[len] != '\0') {
        len++;
    }

    /* 需要的空间：'$'(1) + payload(len) + '*'(1) + CK(2) + "\r\n"(2) + '\0'(1) */
    size_t need = len + 7;
    if (out_size < need) {
        return -1;   /* 缓冲区不够，宁可不写也不能溢出 */
    }

    uint8_t ck = protocol_xor_checksum((const uint8_t *)payload, len);

    size_t pos = 0;
    out[pos++] = '$';
    for (size_t i = 0; i < len; i++) {
        out[pos++] = payload[i];
    }
    out[pos++] = '*';
    out[pos++] = HEX_DIGITS[(ck >> 4) & 0x0F];
    out[pos++] = HEX_DIGITS[ck & 0x0F];
    out[pos++] = '\r';
    out[pos++] = '\n';
    out[pos] = '\0';

    /* 返回不含 '\0' 的帧长度 = len + 6，方便以后换 UART 发送时直接当长度用 */
    return (int)pos;
}
