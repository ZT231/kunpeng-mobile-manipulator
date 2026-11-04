// 全链路集成测试：MockSerialPort + FrameParser + CommandDispatcher 串起来，
// 证明「不插机械臂，也能跑通 收字节 → 解析 → 分派 → 应答 → 写回」的完整闭环。
#include "serial/command_dispatcher.hpp"
#include "serial/commands.hpp"
#include "serial/frame.hpp"
#include "serial/frame_parser.hpp"
#include "serial/mock_serial_port.hpp"

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

using serial::CommandDispatcher;
using serial::Frame;
using serial::FrameParser;
using serial::MockSerialPort;

// 把「一轮收发包」封装成一个动作：注入请求帧 → 读字节 → 解析 → 分派 → 把应答写回串口。
// 返回写回的应答 payload（字符串），方便断言。
static std::string runOneRound(MockSerialPort& port, FrameParser& parser, CommandDispatcher& d,
                               const std::string& requestFrameText) {
    // 1) 模拟设备发来一帧
    port.inject(requestFrameText);

    // 2) 上位机从串口读字节（这里一次性读回，真实代码会循环读到超时）
    std::uint8_t raw[128] = {0};
    int n = port.read(raw, sizeof(raw), 0);
    if (n <= 0) {
        return "<no data>";
    }

    // 把读到的字节重新拼成字符串，喂给状态机，得到解析出来的帧
    std::string s = serial::bytesToString(raw, static_cast<std::size_t>(n));
    std::vector<Frame> frames = parser.feed(s);

    // 4) 每帧分派，把应答写回串口
    for (std::size_t i = 0; i < frames.size(); ++i) {
        Frame out;
        if (d.dispatch(frames[i], out)) {
            // 分发成功：out 就是应答帧
            std::string text = out.toString();
            std::vector<std::uint8_t> bytes(text.begin(), text.end());
            port.write(bytes.data(), bytes.size());
            return out.payload();
        } else {
            // 分发失败（未知命令）：转成 ERR,1（对应固件「未知命令」）
            Frame err = serial::makeError(serial::ErrorCode::UnknownCommand);
            std::string text = err.toString();
            std::vector<std::uint8_t> bytes(text.begin(), text.end());
            port.write(bytes.data(), bytes.size());
            return err.payload();
        }
    }
    return "<no frame>";
}

static void test_ping_roundtrip() {
    MockSerialPort port;
    FrameParser parser;
    CommandDispatcher d;
    serial::registerFirmwareCommands(d);
    port.open();

    // 请求 "$PING*10\r\n"（校验和 0x10 是固件里 PING 的真实值）
    std::string resp = runOneRound(port, parser, d, "$PING*10\r\n");
    CHECK(resp == "OK,PONG");

    // 写回串口的字节必须是合法的完整帧 "$OK,PONG*3E\r\n"
    CHECK(port.sentString() == "$OK,PONG*3E\r\n");
    CHECK(parser.stats().frames_ok == 1);
}

static void test_move_valid_and_invalid() {
    MockSerialPort port;
    FrameParser parser;
    CommandDispatcher d;
    serial::registerFirmwareCommands(d);
    port.open();

    // 合法：ch=0，pulse=1500
    std::string ok = runOneRound(port, parser, d, Frame("MOVE,0,1500").toString());
    CHECK(ok == "OK");

    // 非法：ch 越界 -> ERR,3
    std::string bad = runOneRound(port, parser, d, Frame("MOVE,6,1500").toString());
    CHECK(bad == "ERR,3");
}

static void test_unknown_command_returns_err1() {
    MockSerialPort port;
    FrameParser parser;
    CommandDispatcher d;
    serial::registerFirmwareCommands(d);
    port.open();

    // 未知命令：分派器返回 false，集成层转成 ERR,1（对应固件「未知命令」）
    std::string resp = runOneRound(port, parser, d, Frame("FLY").toString());
    CHECK(resp == "ERR,1");
}

static void test_glued_packets_all_answered() {
    MockSerialPort port;
    FrameParser parser;
    CommandDispatcher d;
    serial::registerFirmwareCommands(d);
    port.open();

    // 粘包：两条请求一次到，读回、解析出 2 帧，分别应答
    std::string burst = Frame("PING").toString() + Frame("LOG,1").toString();
    port.inject(burst);

    std::uint8_t raw[128] = {0};
    int n = port.read(raw, sizeof(raw), 0);
    std::string s = serial::bytesToString(raw, static_cast<std::size_t>(n));
    std::vector<Frame> frames = parser.feed(s);
    CHECK(frames.size() == 2);
    CHECK(parser.stats().frames_ok == 2);
    CHECK(frames[0].command() == "PING" && frames[1].command() == "LOG");
}

int main() {
    test_ping_roundtrip();
    test_move_valid_and_invalid();
    test_unknown_command_returns_err1();
    test_glued_packets_all_answered();

    std::cout << "passed = " << g_passed << ", failed = " << g_failed << "\n";
    return g_failed == 0 ? 0 : 1;
}
