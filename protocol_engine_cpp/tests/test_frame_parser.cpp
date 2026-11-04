// FrameParser 单元测试：重点覆盖真实串口必然遇到的四种情况
//   半包 / 粘包 / 噪声 / 超长帧，以及坏帧之后的重新同步能力
#include "serial/frame.hpp"
#include "serial/frame_parser.hpp"

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

static int g_passed = 0;
static int g_failed = 0;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (cond) {                                                          \
            ++g_passed;                                                      \
        } else {                                                             \
            ++g_failed;                                                      \
            std::cout << "[FAIL] line " << __LINE__ << ": " << #cond << "\n"; \
        }                                                                    \
    } while (0)

using serial::Frame;
using serial::FrameParser;

// 按「逐字节喂」的方式跑一整段字节流，把解析出来的所有帧收进 vector
static std::vector<Frame> feedByteByByte(FrameParser& parser, const std::string& bytes) {
    std::vector<Frame> out;
    for (std::size_t i = 0; i < bytes.size(); ++i) {
        Frame f;
        // feed 返回 true 才说明这字节让一帧「凑完整且校验通过」，f 里才有帧
        if (parser.feed(static_cast<std::uint8_t>(bytes[i]), f)) {
            out.push_back(f);
        }
    }
    return out;
}

static void test_half_packet_byte_by_byte() {
    FrameParser parser;
    std::string text = Frame("MOVE,0,1500").toString();   // $MOVE,0,1500*25\r\n

    std::vector<Frame> frames = feedByteByByte(parser, text);
    CHECK(frames.size() == 1);
    CHECK(frames[0].command() == "MOVE");
    std::uint32_t v = 0;
    CHECK(frames[0].argUint(1, v) && v == 1500);
    CHECK(parser.stats().frames_ok == 1);
    CHECK(parser.stats().checksum_errors == 0);
    CHECK(parser.stats().format_errors == 0);
    // 帧结束后跟的 \r\n 不该被算成噪声（这是有意设计，不是巧合）
    CHECK(parser.stats().noise_bytes == 0);
    CHECK(!parser.receiving());
}

static void test_glued_packets() {
    FrameParser parser;
    // 一次性喂 3 条完整帧：真实场景里的「粘包」
    std::string burst = Frame("PING").toString()          // $PING*10\r\n
                        + Frame("STATE").toString()
                        + Frame("TNUM").toString();

    // 字符串版 feed：一次返回所有解析出来的帧（vector）
    std::vector<Frame> frames = parser.feed(burst);
    CHECK(frames.size() == 3);
    CHECK(frames[0].command() == "PING");
    CHECK(frames[1].command() == "STATE");
    CHECK(frames[2].command() == "TNUM");
    CHECK(parser.stats().frames_ok == 3);
    CHECK(parser.stats().noise_bytes == 0);
}

static void test_packet_split_inside_later_packet() {
    // 真正的「半包 + 粘包」混合：第一段止于第 2 帧中间，第二段补齐并带第 3 帧
    FrameParser parser;
    std::string s1 = Frame("PING").toString();
    std::string s2 = Frame("STATE").toString();
    std::string s3 = Frame("TNUM").toString();

    std::string combined = s1 + s2 + s3;
    std::size_t cut = s1.size() + s2.size() / 2;

    std::vector<Frame> first = parser.feed(combined.substr(0, cut));
    CHECK(first.size() == 1);                       // 只有第 1 帧完整
    CHECK(first[0].command() == "PING");
    CHECK(parser.receiving());                      // 第 2 帧收了一半

    std::vector<Frame> second = parser.feed(combined.substr(cut));
    CHECK(second.size() == 2);
    CHECK(second[0].command() == "STATE");
    CHECK(second[1].command() == "TNUM");
    CHECK(parser.stats().frames_ok == 3);
}

static void test_noise_prefix() {
    FrameParser parser;
    // 前面是 AT 回显/乱码（真实 WiFi 模块场景），后面才是有效帧
    std::string mixed = "AT+CIPSEND=0,8\r\nOK\r\njunk<<$" + Frame("PING").toString();
    std::vector<Frame> frames = parser.feed(mixed);
    CHECK(frames.size() == 1);
    CHECK(frames[0].command() == "PING");
    CHECK(parser.stats().noise_bytes > 0);          // 噪声被计数，不是被忽略
    CHECK(parser.stats().frames_ok == 1);
}

static void test_bad_checksum_then_resync() {
    FrameParser parser;
    std::vector<Frame> bad = parser.feed(std::string("$MOVE,0,1500*00\r\n"));  // CK 故意写错
    CHECK(bad.empty());
    CHECK(parser.stats().checksum_errors == 1);
    CHECK(parser.stats().frames_ok == 0);

    // 坏帧之后必须能自动重新同步，这是「一个坏帧不能拖死整条链路」
    std::vector<Frame> frames = parser.feed(Frame("PING").toString());
    CHECK(frames.size() == 1);
    CHECK(frames[0].command() == "PING");
    CHECK(parser.stats().frames_ok == 1);
}

static void test_missing_checksum_is_format_error() {
    FrameParser parser;
    CHECK(parser.feed(std::string("$MOVE,0,1500\r\n")).empty());      // 只有 CRLF，没有 *CK
    CHECK(parser.stats().format_errors == 1);

    FrameParser parser2;
    CHECK(parser2.feed(std::string("$PING*1Z\r\n")).empty());          // CK 不是十六进制
    CHECK(parser2.stats().format_errors == 1);

    FrameParser parser3;
    CHECK(parser3.feed(std::string("$*10\r\n")).empty());              // 空 payload
    CHECK(parser3.stats().format_errors == 1);
}

static void test_overflow_drop_then_resync() {
    FrameParser parser;
    std::string junk = "$" + std::string(100, 'A') + "\r\n";     // 永远等不到 *CK
    CHECK(parser.feed(junk).empty());
    CHECK(parser.stats().overflow_drops == 1);
    CHECK(!parser.receiving());

    std::vector<Frame> frames = parser.feed(Frame("TNUM").toString());
    CHECK(frames.size() == 1);
    CHECK(frames[0].command() == "TNUM");
}

static void test_new_start_byte_restarts() {
    FrameParser parser;
    CHECK(parser.feed(std::string("$PIN")).empty());                  // 半帧
    CHECK(parser.receiving());
    CHECK(parser.partialLength() == 3);

    // 新的 '$' 到来 -> 丢掉旧的半帧，按新帧重新开始
    std::vector<Frame> frames = parser.feed(Frame("PING").toString());
    CHECK(frames.size() == 1);
    CHECK(frames[0].command() == "PING");
    CHECK(parser.stats().format_errors == 0);                          // 半帧不算格式错
}

static void test_reset_and_reset_stats() {
    FrameParser parser;
    parser.feed(std::string("$PIN"));
    CHECK(parser.receiving());
    parser.reset();
    CHECK(!parser.receiving());
    CHECK(parser.partialLength() == 0);

    parser.feed(Frame("PING").toString());
    CHECK(parser.stats().frames_ok == 1);
    parser.resetStats();
    CHECK(parser.stats().frames_ok == 0);
}

int main() {
    test_half_packet_byte_by_byte();
    test_glued_packets();
    test_packet_split_inside_later_packet();
    test_noise_prefix();
    test_bad_checksum_then_resync();
    test_missing_checksum_is_format_error();
    test_overflow_drop_then_resync();
    test_new_start_byte_restarts();
    test_reset_and_reset_stats();

    std::cout << "passed = " << g_passed << ", failed = " << g_failed << "\n";
    return g_failed == 0 ? 0 : 1;
}
