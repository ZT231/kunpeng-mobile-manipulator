#pragma once

#include <cstdint>
#include <string>

namespace serial {

// 协议 v1 的校验方式：把 payload（'$' 与 '*CK' 之间的所有字节）逐字节「异或」。
// 为什么用异或、不用 CRC？
//   固件 uart_protocol.c 里用的就是异或（CK = 异或，2 位大写十六进制），
//   我们的上位机必须和固件完全一致，否则校验对不上。这是「协议唯一真理来源」，
//   改协议前必须先看固件那个文件，别自己拍脑袋换 CRC16。
using Checksum = std::uint8_t;

// 把字符串里每个字节异或起来，得到 1 字节校验值。
// 作用：串口传数据可能出错（干扰、断线），接收方用同样的方法算一遍，
//       和帧尾带来的 CK 比对，不一样就说明这帧脏了。
inline Checksum xorChecksum(const std::string& payload) {
    Checksum ck = 0;
    for (std::size_t i = 0; i < payload.size(); ++i) {
        // static_cast 把 char 安全地变成无符号字节再异或（char 可能是有符号的，先转无符号防坑）
        ck ^= static_cast<Checksum>(static_cast<unsigned char>(payload[i]));
    }
    return ck;
}

// 把 1 字节校验值变成「2 位大写十六进制字符串」：0x10 -> "10"，0x0A -> "0A"。
// 为什么是 2 位？因为 1 字节最多 0xFF，正好两位。补零是为了和固件格式对齐。
inline std::string toHex2(Checksum ck) {
    static const char kDigits[] = "0123456789ABCDEF";  // 查表法：下标 0..15 对应字符 '0'..'F'
    std::string s(2, '0');                             // 先放两个 '0'
    s[0] = kDigits[(ck >> 4) & 0x0FU];                 // 高 4 位 -> 十位
    s[1] = kDigits[ck & 0x0FU];                        // 低 4 位 -> 个位
    return s;
}

// 把单个十六进制字符（'0'~'9'、'A'~'F'、'a'~'f'）转成 0~15。
// 返回 true 表示成功，结果写在 v 里；不是十六进制字符就返回 false。
// 为什么单独抽成一个函数？因为 fromHex2 要转两位，每位的逻辑完全一样，复用它更清晰。
inline bool hexNibble(char c, Checksum& v) {
    if (c >= '0' && c <= '9') { v = static_cast<Checksum>(c - '0'); return true; }
    if (c >= 'A' && c <= 'F') { v = static_cast<Checksum>(c - 'A' + 10); return true; }
    if (c >= 'a' && c <= 'f') { v = static_cast<Checksum>(c - 'a' + 10); return true; }
    return false;
}

// 把两位十六进制字符（如 '3' 和 'E'）拼成一个字节：fromHex2('3','E',out) -> out = 0x3E。
// 和固件 hex_to_nib() 一致：大小写都收。
inline bool fromHex2(char hi, char lo, Checksum& out) {
    Checksum h = 0;
    Checksum l = 0;
    if (!hexNibble(hi, h) || !hexNibble(lo, l)) {
        return false;   // 任一位不是十六进制 -> 失败
    }
    out = static_cast<Checksum>((h << 4) | l);  // 高 4 位左移 4 位，或上低 4 位
    return true;
}

// 判断一个字符是不是十六进制数字（给解析器用来「看 CK 是不是合法」）。
inline bool isHexDigit(char c) {
    return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
}

}  // namespace serial
