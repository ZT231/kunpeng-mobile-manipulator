// Frame / Checksum / ErrorCode 单元测试
#include "serial/checksum.hpp"
#include "serial/error_code.hpp"
#include "serial/frame.hpp"

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

using serial::Checksum;
using serial::Frame;

static void test_checksum_against_real_firmware() {
    // 基准值直接取自固件源码 uart_protocol.c 的注释与实现：
    //   $PING*10 是手机端发 PING 的真实可用帧
    CHECK(serial::xorChecksum("PING") == 0x10);
    CHECK(serial::toHex2(serial::xorChecksum("PING")) == "10");

    // 固件 cmd_ping() 回的就是 "OK,PONG"
    CHECK(serial::xorChecksum("OK,PONG") == 0x3E);

    // 2 位大写、补零
    CHECK(serial::toHex2(0x00) == "00");
    CHECK(serial::toHex2(0x0A) == "0A");
    CHECK(serial::toHex2(0xFF) == "FF");

    // 十六进制解析：大小写都收
    Checksum ck = 0;
    CHECK(serial::fromHex2('3', 'E', ck) && ck == 0x3E);
    CHECK(serial::fromHex2('3', 'e', ck) && ck == 0x3E);
    CHECK(!serial::fromHex2('G', '0', ck));   // 非法字符必须拒绝
    CHECK(!serial::fromHex2('0', ' ', ck));
}

static void test_frame_parse_fields() {
    Frame move("MOVE,0,1500");
    CHECK(move.command() == "MOVE");

    std::vector<std::string> args = move.args();
    CHECK(args.size() == 2);
    CHECK(args[0] == "0" && args[1] == "1500");

    // argUint 用「bool + 出参」：返回 true 表示解析成功，结果在 v 里
    std::uint32_t v0 = 0, v1 = 0;
    CHECK(move.argUint(0, v0) && v0 == 0);
    CHECK(move.argUint(1, v1) && v1 == 1500);
    std::uint32_t vx = 0;
    CHECK(!move.argUint(2, vx));          // 越界 -> false

    // 无参数命令
    Frame ping("PING");
    CHECK(ping.command() == "PING");
    CHECK(ping.args().empty());
    CHECK(!ping.argUint(0, vx));

    // 六关节应答：OK,S,M,1500,1500,1500,1500,1500,1500
    Frame state("OK,S,M,1500,1500,1500,1500,1500,1500");
    CHECK(state.command() == "OK");
    CHECK(state.args().size() == 8);
    CHECK(state.args()[0] == "S" && state.args()[1] == "M");
    std::uint32_t v2 = 0;
    CHECK(state.argUint(2, v2) && v2 == 1500);
    CHECK(!state.argUint(0, vx));          // "S" 不是数字

    // 错误帧
    Frame err("ERR,4");
    CHECK(err.command() == "ERR");
    CHECK(err.args().size() == 1);
    std::uint32_t ve = 0;
    CHECK(err.argUint(0, ve) && ve == 4);

    // 数字解析的坑：前导空格、正负号、溢出都不该被当成合法数字
    Frame weird("MOVE, 5,-1,99999999999");
    std::uint32_t w0 = 0, w1 = 0, w2 = 0;
    CHECK(!weird.argUint(0, w0));          // " 5" 带空格 -> 拒绝
    CHECK(!weird.argUint(1, w1));          // "-1" 无符号解析必须拒绝
    CHECK(!weird.argUint(2, w2));          // 超出 uint32 -> 拒绝，而不是截断
}

static void test_frame_to_string() {
    Frame f("MOVE,0,1500");
    std::string text = f.toString();

    // 结构：$ + payload + * + 2 位大写 CK + \r\n
    CHECK(text == "$MOVE,0,1500*" + serial::toHex2(f.checksum()) + "\r\n");
    CHECK(text.front() == '$');
    CHECK(text.substr(text.size() - 2) == "\r\n");
    CHECK(serial::isHexDigit(text[text.size() - 3]) && serial::isHexDigit(text[text.size() - 4]));
    CHECK(text[text.size() - 5] == '*');
    CHECK(text.size() == 1 + 11 + 1 + 2 + 2);

    // PING 的帧文本必须与固件注释 / 手机端工具能用的写法一致
    CHECK(Frame("PING").toString() == "$PING*10\r\n");
}

static void test_frame_parse_roundtrip() {
    Frame f("JOINT,1500,1500,1500,1500,1500,1500");
    Frame back;
    CHECK(Frame::parse(f.toString(), back) && back == f);

    // parse 对尾部 CR/LF 宽容
    Frame t1, t2, t3, t4;
    CHECK(Frame::parse("$PING*10", t1));
    CHECK(Frame::parse("$PING*10\r", t2));
    CHECK(Frame::parse("$PING*10\r\n", t3));
    CHECK(Frame::parse("$PING*10\n", t4));

    // 以下都必须拒绝
    Frame dummy;
    CHECK(!Frame::parse("PING*10\r\n", dummy));                  // 缺 '$'
    CHECK(!Frame::parse("$PING\r\n", dummy));                    // 缺 "*CK"
    CHECK(!Frame::parse("$PING*10X\r\n", dummy));                // CK 后有多余字符（从严）
    CHECK(!Frame::parse("$PING*1Z\r\n", dummy));                 // CK 不是十六进制
    CHECK(!Frame::parse("$PING*11\r\n", dummy));                 // 校验不对
    CHECK(!Frame::parse("$*10\r\n", dummy));                     // 空 payload
    CHECK(!Frame::parse("", dummy));
}

static void test_frame_copy() {
    Frame a("TNUM");
    Frame b = a;                       // Frame 内部只有 std::string，拷贝构造自动深拷贝
    CHECK(b == a);

    a = b;                            // 拷贝赋值
    CHECK(a == b);
}

static void test_error_code() {
    CHECK(static_cast<int>(serial::ErrorCode::UnknownCommand) == 1);
    CHECK(static_cast<int>(serial::ErrorCode::BadChecksum) == 2);
    CHECK(static_cast<int>(serial::ErrorCode::BadArgument) == 3);
    CHECK(static_cast<int>(serial::ErrorCode::StateForbidden) == 4);
    CHECK(std::string(serial::toString(serial::ErrorCode::StateForbidden)) == "当前状态禁止该操作");
}

int main() {
    test_checksum_against_real_firmware();
    test_frame_parse_fields();
    test_frame_to_string();
    test_frame_parse_roundtrip();
    test_frame_copy();
    test_error_code();

    std::cout << "passed = " << g_passed << ", failed = " << g_failed << "\n";
    return g_failed == 0 ? 0 : 1;
}
