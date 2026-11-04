#pragma once

#include "serial/checksum.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace serial {

// 把一串字节（uint8_t 数组）按字符拼成 std::string。
//
// 为什么需要它？串口底层按「字节」收发（read 返回 uint8_t*），而上层协议
// （Frame / FrameParser）按「字符 / 字符串」处理，这步转换在 Session、AsyncReceiver
// 里都要做。抽成公共函数，避免每个读串口的地方都重写一遍 for 循环（那正是冗余的来源）。
// char 和 uint8_t 都是 1 字节，static_cast<char> 只是「换看待类型」，不丢数据，安全。
inline std::string bytesToString(const std::uint8_t* data, std::size_t n) {
    std::string s;
    s.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        s.push_back(static_cast<char>(data[i]));
    }
    return s;
}

// 一帧协议数据。这里我们只负责「切字段」，不负责判断业务逻辑
// （比如 6 个关节脉宽够不够、脉宽有没有超 2500）。那是命令层（commands.hpp）的事。
//
// payload 指 '$' 与 '*CK' 之间的内容（不含 '$'、'*'、CK、CRLF），例如：
//   "PING"                 请求
//   "MOVE,0,1500"          请求
//   "OK,S,M,1500,..."      应答
//   "ERR,4"                应答
class Frame {
public:
    // 默认构造：空帧
    Frame() {}

    // explicit：防止字符串被「偷偷」当成 Frame 用。要造帧就明确写 Frame("MOVE,0,1500")。
    explicit Frame(std::string payload) : payload_(payload) {}

    // 取完整 payload 文本（如 "MOVE,0,1500"）
    const std::string& payload() const { return payload_; }

    // 现场算校验值（用 payload 异或）。每改一次 payload，这个值都会重新算，不会过期。
    Checksum checksum() const { return xorChecksum(payload_); }

    // 逗号之前的字段，如 "MOVE"、"OK"。没有逗号时返回整个 payload。
    std::string command() const {
        std::size_t comma = payload_.find(',');
        if (comma == std::string::npos) {
            return payload_;
        }
        return payload_.substr(0, comma);
    }

    // 逗号之后的各字段，按 ',' 切开。"MOVE,0,1500" -> {"0", "1500"}，没有逗号返回空 vector。
    std::vector<std::string> args() const {
        std::vector<std::string> out;
        std::size_t comma = payload_.find(',');
        if (comma == std::string::npos) {
            return out;
        }
        std::size_t start = comma + 1;
        while (true) {
            std::size_t next = payload_.find(',', start);
            if (next == std::string::npos) {
                out.push_back(payload_.substr(start));  // 最后一段
                break;
            }
            out.push_back(payload_.substr(start, next - start));
            start = next + 1;
        }
        return out;
    }

    // 把第 index 个参数当「十进制无符号整数」取出来。
    // 返回 true 表示成功，结果写在 out 里；否则返回 false（不是纯数字 / 越界 / 溢出）。
    //
    // 为什么不用「返回数字、出错返回 -1」这种写法？
    //   因为 0 本身就是合法值，-1 会和「真出错」混在一起分不清。用 bool+出参最清楚：
    //   调用方先判断返回值，再使用 out。
    bool argUint(std::size_t index, std::uint32_t& out) const {
        std::vector<std::string> all = args();
        if (index >= all.size()) {
            return false;   // 要的参数不存在
        }
        const std::string& s = all[index];
        if (s.empty()) {
            return false;
        }
        std::uint32_t v = 0;
        for (std::size_t i = 0; i < s.size(); ++i) {
            char c = s[i];
            // 前导空格、负号、字母都一律拒绝：无符号整数只能是 '0'~'9'
            if (c < '0' || c > '9') {
                return false;
            }
            std::uint32_t d = static_cast<std::uint32_t>(c - '0');
            // 防溢出：先判断 v*10+d 会不会超过 uint32 最大值（4294967295），
            // 超了就拒绝，而不是返回一个被「绕回」的错误数字。
            if (v > (4294967295U - d) / 10U) {
                return false;
            }
            v = v * 10U + d;
        }
        out = v;
        return true;
    }

    // 生成可以真正发到串口上的完整帧："$MOVE,0,1500*25\r\n"
    std::string toString() const {
        return "$" + payload_ + "*" + toHex2(checksum()) + "\r\n";
    }

    // 从一段完整帧文本解析出 Frame。
    // 返回 true 表示解析成功（是个合法帧），结果写在 out 里；否则返回 false。
    // 解析规则（从严，和固件对齐）：
    //   - 宽容吃尾部 CR/LF（串口常带换行）
    //   - 必须 '$' 开头、必须有 '*' + 2 位十六进制 CK、CK 后不能再有其它字符
    //   - 校验必须通过
    static bool parse(const std::string& text, Frame& out) {
        std::string t = text;
        while (!t.empty() && (t.back() == '\r' || t.back() == '\n')) {
            t.pop_back();   // 去掉末尾换行
        }
        if (t.size() < 4 || t.front() != '$') {
            return false;   // 太短或没以 '$' 开头
        }
        std::size_t star = t.find('*');
        // 必须正好 "*XX" 结尾（'*' 在第 size-3 位，后面跟两位）
        if (star == std::string::npos || t.size() != star + 3) {
            return false;
        }
        std::string payload = t.substr(1, star - 1);   // '$' 和 '*' 之间的内容
        if (payload.empty()) {
            return false;   // 空 payload 不允许
        }
        Checksum rx = 0;
        if (!fromHex2(t[star + 1], t[star + 2], rx)) {
            return false;   // CK 不是合法十六进制
        }
        if (rx != xorChecksum(payload)) {
            return false;   // 校验对不上 -> 脏帧
        }
        out = Frame(payload);
        return true;
    }

    // 两个帧相等：看 payload 字符串是否一样。== 让测试里能直接写 a == b。
    friend bool operator==(const Frame& a, const Frame& b) { return a.payload_ == b.payload_; }
    friend bool operator!=(const Frame& a, const Frame& b) { return !(a == b); }

private:
    std::string payload_;   // 帧的核心数据，所有方法都围着它转
};

}  // namespace serial
