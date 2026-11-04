#pragma once

#include "serial/frame.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace serial {

// 字节流 -> 帧 的状态机解析器。
//
// 真实串口上必然会遇到这四种情况，本类就是为它们而生的：
//   1) 半包：一帧被拆到好几次 IO 里到（这里按「逐字节喂」处理，天然支持）
//   2) 粘包：一次读回来 3 条完整帧（feed(string) 一次就能吐出多帧）
//   3) 噪声：AT 回显、乱码、上电毛刺（不在帧内的字节，只计数不报错）
//   4) 超长帧：对端发疯或波特率不匹配导致收不满（整帧丢弃并计数，之后能自动重新同步）
//
// 逻辑和固件 UartProto_HandleByte() 一一对应，但用 C++ 写成了「对象 + 返回值」，
// 不再是全局变量 + 裸缓冲。
class FrameParser {
public:
    // 固件 PROTO_BUF_SIZE = 64，扣掉结尾 '\0' 后最多 63 字节内容（含 "*CK" 的 3 字节）。
    // 所以 payload 本身最多 63 - 3 = 60 字节。
    static const std::size_t kMaxContent = 63;

    // 丢帧统计：上位机跑起来第一件事就是盯这几个数，而不是靠肉眼猜哪里出问题。
    struct Stats {
        std::size_t frames_ok = 0;        // 成功解析的帧
        std::size_t checksum_errors = 0;  // 校验不对（对应固件 ERR,2）
        std::size_t format_errors = 0;    // 结构不对：没收到 "*CK"、CK 不是十六进制
        std::size_t overflow_drops = 0;   // 超长帧整体丢弃
        std::size_t noise_bytes = 0;      // 帧外的噪声字节
    };

    // 喂一个字节。如果这字节让一帧「凑完整且校验通过」，就把帧写进 out 并返回 true；
    // 否则返回 false（还在收 / 收到坏帧 / 只是噪声——这些都算「没拿到合法帧」）。
    //
    // 为什么用「bool + 出参」而不是返回「可能有帧可能没有」的高级类型？
    //   因为新手一眼能看懂：返回 true 就说明 out 里有一帧；false 就啥也没有。
    bool feed(std::uint8_t byte, Frame& out) {
        char ch = static_cast<char>(byte);

        if (ch == '$') {              // 起始符：丢掉之前没收完的一切，从新开始收一帧
            active_ = true;
            buf_.clear();
            return false;
        }

        if (!active_) {
            // 还没进入一帧：CR/LF 是上一帧的结束符，不算噪声；其它字节才算噪声
            if (ch != '\r' && ch != '\n') {
                ++stats_.noise_bytes;
            }
            return false;
        }

        if (ch == '\r' || ch == '\n') {
            // 靠 CRLF 结束的帧：也必须自带 "*CK"，否则算格式错
            active_ = false;
            if (buf_.empty()) {
                return false;
            }
            return completeFrame(out);
        }

        buf_.push_back(ch);

        // 收满 "*CK"（'*' + 两个十六进制）就立即判帧结束——和固件一致：
        // 不强制等 CRLF，兼容不发 \r\n 的上位机工具。
        std::size_t n = buf_.size();
        if (n >= 3 && buf_[n - 3] == '*' && isHexDigit(buf_[n - 2]) && isHexDigit(buf_[n - 1])) {
            active_ = false;
            return completeFrame(out);
        }

        // 超长：整帧丢弃，回到等 '$'（防止一个发疯的对端拖死解析器）
        if (buf_.size() > kMaxContent) {
            active_ = false;
            buf_.clear();
            ++stats_.overflow_drops;
        }
        return false;
    }

    // 一次喂一串字节（实际读串口通常一次读回几十字节）：把其中所有「完整且合法」的帧收进返回值。
    // 返回 vector 是因为粘包可能一次出多帧，用 vector 装着最自然，调用方直接遍历。
    std::vector<Frame> feed(const std::string& bytes) {
        std::vector<Frame> out;
        for (std::size_t i = 0; i < bytes.size(); ++i) {
            Frame f;
            if (feed(static_cast<std::uint8_t>(bytes[i]), f)) {
                out.push_back(f);   // 只有真正凑成合法帧才收进来
            }
        }
        return out;
    }

    void reset() {
        active_ = false;
        buf_.clear();
    }

    bool receiving() const { return active_; }
    const Stats& stats() const { return stats_; }
    void resetStats() { stats_ = Stats(); }
    std::size_t partialLength() const { return buf_.size(); }

private:
    // 对 buf_ 做结构校验 + 校验和比对，成功则把帧写进 out 并返回 true。
    bool completeFrame(Frame& out) {
        std::size_t star = buf_.find('*');

        // 从严：必须有 '*'，后面正好跟 2 位十六进制，且 CK 之后不能再有其它字符
        if (star == std::string::npos || buf_.size() != star + 3 ||
            !isHexDigit(buf_[star + 1]) || !isHexDigit(buf_[star + 2])) {
            buf_.clear();
            ++stats_.format_errors;
            return false;
        }

        std::string payload = buf_.substr(0, star);
        char hi = buf_[star + 1];
        char lo = buf_[star + 2];
        buf_.clear();

        if (payload.empty()) {
            ++stats_.format_errors;
            return false;
        }

        Checksum rx = 0;
        if (!fromHex2(hi, lo, rx)) {   // 上面已判过 isHexDigit，这里把接口走完
            ++stats_.format_errors;
            return false;
        }

        if (rx != xorChecksum(payload)) {
            ++stats_.checksum_errors;   // 校验对不上 -> 脏帧，丢弃
            return false;
        }

        stats_.frames_ok++;
        out = Frame(payload);
        return true;
    }

    bool active_ = false;   // 当前是否处于「正在收一帧」的状态
    std::string buf_;       // 累积当前帧的内容（不含 '$' 和 '*CK'）
    Stats stats_;           // 各种丢帧计数
};

}  // namespace serial
