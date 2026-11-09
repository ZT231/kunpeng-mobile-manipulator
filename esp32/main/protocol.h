#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

/* ============================================================
 * protocol —— 项目统一串口帧的「封包」实现（ESP32 侧）
 *
 * 【概念（大白话）】给一句话末尾加个「指纹」：把内容每个字节异或一遍，
 *   收方重算一遍，对不上就说明传错了，整帧丢掉。
 * 【规范定义】帧格式 $<CMD>[,<ARG>...]*<CK>\r\n
 *   CK = payload（'$' 与 '*' 之间）所有字节逐字节 XOR，结果写成 2 位大写 HEX。
 *   真源：Core/Src/uart_protocol.c 的 proto_send_frame() / proto_parse_frame()。
 *   Python 侧对照：kunpeng/llm/text_gateway.py 的 xor_checksum()。
 *
 * 🔴 跨语言坑（docs/排障记录-LLM.md 第 2 条）：XOR 定义在**字节序列**上。
 *   C 的 char* 本来就是字节，天然对齐；Python 的 str 是 Unicode 码点，
 *   中文必须先 .encode("utf-8") 再逐字节算，否则将来接板子全被判 ERR,2。
 *
 * 🔴 诚实边界：这里只提供「封帧 + 打印」，
 *    没有 UART 写、没有 TCP send、没往鲲鹏/STM32 发过任何一帧。
 * ============================================================ */

/* 算 payload 的 CK：逐字节 XOR，返回 0~255 */
uint8_t protocol_xor_checksum(const uint8_t *payload, size_t len);

/* 把 payload 封成完整帧写进 out；返回帧长度（不含结尾 '\0'），失败返回 -1。
 * 【为什么传 out_size】C 数组不自带长度，不检查就会缓冲区溢出——
 *   这是 C 语言最经典的安全坑，比「写起来麻烦」重要得多。 */
int protocol_build_frame(const char *payload, char *out, size_t out_size);

#endif /* PROTOCOL_H */
