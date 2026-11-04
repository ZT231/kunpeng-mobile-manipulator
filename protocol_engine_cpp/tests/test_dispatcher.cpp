// CommandDispatcher / 固件命令表 单元测试
#include "serial/command_dispatcher.hpp"
#include "serial/commands.hpp"
#include "serial/frame.hpp"

#include <iostream>
#include <string>

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

// 本地辅助处理函数（普通函数，签名和 CommandDispatcher::Handler 一致）。
// 这里故意用普通函数：演示「自由函数也能直接注册」（会隐式转成 std::function）。
// lambda 注册的示例见 commands.hpp 里的 PING。
static Frame handleHello(const Frame&) {
    return Frame("OK,HI");
}
static Frame handleAgain(const Frame&) {
    return Frame("OK,AGAIN");
}

// 便捷封装：把「分发 + 取 payload」合成一个动作，测试写起来清爽
static std::string dispatchStr(CommandDispatcher& d, const std::string& payload) {
    Frame out;
    if (d.dispatch(Frame(payload), out)) {
        return out.payload();
    }
    return "<null>";   // 分发失败（未知命令）时用这个占位串
}

static void test_register_and_dispatch() {
    CommandDispatcher d;
    CHECK(d.empty() && d.size() == 0);

    // 注册一个处理函数（函数名就是函数指针，自动传进去）
    const bool reg = d.registerCommand("HELLO", handleHello);
    CHECK(reg);
    CHECK(d.size() == 1 && d.has("HELLO") && !d.has("NOPE"));

    // 不允许覆盖：重复注册同名命令必须失败，且旧 handler 不被替换
    CHECK(!d.registerCommand("HELLO", handleAgain));

    Frame r;
    bool ok = d.dispatch(Frame("HELLO"), r);
    CHECK(ok && r.payload() == "OK,HI");   // 还是旧的 "OK,HI"，没被覆盖

    // 未知命令 -> dispatch 返回 false（调用方负责回 ERR,1）
    Frame dummy;
    CHECK(!d.dispatch(Frame("UNKNOWN"), dummy));

    // 注销
    CHECK(d.unregisterCommand("HELLO"));
    CHECK(!d.dispatch(Frame("HELLO"), dummy));
    CHECK(d.empty());
}

static void test_firmware_commands() {
    CommandDispatcher d;
    serial::registerFirmwareCommands(d);

    CHECK(d.size() == 4);
    CHECK(d.has("PING") && d.has("MOVE") && d.has("JOINT") && d.has("LOG"));

    // PING
    CHECK(dispatchStr(d, "PING") == "OK,PONG");

    // MOVE：参数校验（对应固件 ERR,3）
    CHECK(dispatchStr(d, "MOVE,0,1500") == "OK");
    CHECK(dispatchStr(d, "MOVE,5,2500") == "OK");      // 边界值合法
    CHECK(dispatchStr(d, "MOVE,6,1500") == "ERR,3");   // ch 越界（只有 0~5 路）
    CHECK(dispatchStr(d, "MOVE,0,2501") == "ERR,3");   // pulse 超限
    CHECK(dispatchStr(d, "MOVE,0") == "ERR,3");        // 缺参数
    CHECK(dispatchStr(d, "MOVE,0,abc") == "ERR,3");    // 非法数字
    CHECK(dispatchStr(d, "MOVE,0,1500,999") == "OK");  // 多一个参数：与固件一样宽容（只取前两个）

    // JOINT：必须 6 路、每路 <= 2500
    CHECK(dispatchStr(d, "JOINT,1500,1500,1500,1500,1500,1500") == "OK");
    CHECK(dispatchStr(d, "JOINT,1500,1500,1500,1500,1500") == "ERR,3");        // 缺 1 路
    CHECK(dispatchStr(d, "JOINT,1500,1500,1500,1500,1500,2600") == "ERR,3");   // 1 路超限

    // LOG：只有 0 / 1
    CHECK(dispatchStr(d, "LOG,0") == "OK");
    CHECK(dispatchStr(d, "LOG,1") == "OK");
    CHECK(dispatchStr(d, "LOG,2") == "ERR,3");
    CHECK(dispatchStr(d, "LOG") == "ERR,3");
    CHECK(dispatchStr(d, "LOG,abc") == "ERR,3");

    // 未知命令整体
    CHECK(dispatchStr(d, "FLY") == "<null>");
}

static void test_error_helper() {
    // makeError 生成的帧，其 payload 必须是精确的 "ERR,<code>"
    CHECK(serial::makeError(serial::ErrorCode::BadArgument).payload() == "ERR,3");
    CHECK(serial::makeError(serial::ErrorCode::StateForbidden).payload() == "ERR,4");
    // 且能直接 to-string 成合法帧
    Frame e = serial::makeError(serial::ErrorCode::BadChecksum);
    CHECK(e.toString().size() == 1 + 5 + 1 + 2 + 2);   // "$ERR,2*XX\r\n"
}

int main() {
    test_register_and_dispatch();
    test_firmware_commands();
    test_error_helper();

    std::cout << "passed = " << g_passed << ", failed = " << g_failed << "\n";
    return g_failed == 0 ? 0 : 1;
}
