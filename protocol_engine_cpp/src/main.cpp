// armctl —— 机械臂串口上位机（协议 v1，与固件 uart_protocol.c 对齐）
//
// 用法：
//   armctl selftest                    用内存假串口自检闭环（无需硬件）
//   armctl [--port 名] [--baud 波特率] [--timeout 毫秒] <命令> [参数]
//
// 命令：
//   ping                测链路，回 OK,PONG
//   move <ch> <pulse>   单舵机：ch=0..5，pulse<=2500
//   joint <p1..p6>      六舵机同步（6 个脉宽）
//   log <0|1>           开关周期遥测
//   raw <payload>       直接发原始 payload（调试）
//
// 示例：
//   armctl --port COM3 ping
//   armctl --port /dev/ttyUSB0 move 0 1500
//   armctl --mock ping                      （无硬件演示）

#include "serial/commands.hpp"
#include "serial/frame.hpp"
#include "serial/logger.hpp"
#include "serial/mock_serial_port.hpp"
#include "serial/serial_port_factory.hpp"
#include "serial/session.hpp"

#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <memory>

using serial::Frame;
using serial::Logger;
using serial::LogLevel;

// ---- 一个极简的单元测试框架（不引第三方库，g++ 直接编）----
// 用全局计数 + CHECK 宏，和各个 tests/*.cpp 保持一致风格。
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

namespace {

void printUsage() {
    std::cout <<
        "armctl - 机械臂串口上位机（协议 v1）\n"
        "\n"
        "用法：\n"
        "  armctl selftest                        用内存假串口自检闭环（无需硬件）\n"
        "  armctl [--port 名] [--baud 波特率] [--timeout 毫秒] <命令> [参数]\n"
        "\n"
        "命令：\n"
        "  ping                 测链路，回 OK,PONG\n"
        "  move <ch> <pulse>    单舵机：ch=0..5，pulse<=2500\n"
        "  joint <p1..p6>       六舵机同步（6 个脉宽）\n"
        "  log <0|1>            开关周期遥测\n"
        "  raw <payload>        直接发原始 payload（调试）\n"
        "\n"
        "示例：\n"
        "  armctl --port COM3 ping\n"
        "  armctl --port /dev/ttyUSB0 move 0 1500\n"
        "  armctl --mock ping\n";
}

struct Options {
    std::string port = "COM3";
    int baud = 115200;
    int timeout = 1000;
    bool mock = false;
    std::string command;
    std::vector<std::string> args;
};

// 解析命令行参数，填进 opt。返回 false 表示参数非法。
bool parseArgs(int argc, char** argv, Options& opt) {
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--port") {
            if (++i >= argc) return false;
            opt.port = argv[i];
        } else if (a == "--baud") {
            if (++i >= argc) return false;
            opt.baud = std::stoi(argv[i]);
        } else if (a == "--timeout") {
            if (++i >= argc) return false;
            opt.timeout = std::stoi(argv[i]);
        } else if (a == "--mock") {
            opt.mock = true;
        } else if (a == "--help" || a == "-h") {
            printUsage();
            std::exit(0);
        } else if (opt.command.empty()) {
            opt.command = a;   // 第一个非选项参数当作命令名
        } else {
            opt.args.push_back(a);
        }
    }
    return true;
}

// 自检：证明「收字节 → 解析 → 分派 → 应答」整条链路，用内存假串口，无需硬件。
int runSelfTest() {
    // 1) 模拟器端：dispatcher + 固件命令表，处理 PING 应回 OK,PONG
    serial::CommandDispatcher sim;
    serial::registerFirmwareCommands(sim);
    Frame reply;
    bool ok = sim.dispatch(Frame("PING"), reply);
    CHECK(ok && reply.payload() == "OK,PONG");

    // 2) 客户端端：Session 通过 Mock 发请求、收应答（应答已预先注入，模拟设备回包）
    //    Mock 用 unique_ptr 管理：离开作用域自动析构（不需要手动 delete）。
    auto mock = std::make_unique<serial::MockSerialPort>("mock");
    mock->open();
    mock->inject(Frame("OK,PONG").toString());
    serial::SerialSession client(std::move(mock));   // 把端口 move 进会话（会话独占拥有，析构自动释放）
    Frame resp;
    bool got = client.request(Frame("PING"), resp, 200);
    CHECK(got && resp.command() == "OK" && resp.args()[0] == "PONG");

    // 3) 往返后的帧文本必须合法（$OK,PONG*3E\r\n）
    CHECK(got && resp.toString() == "$OK,PONG*3E\r\n");

    std::cout << (g_failed == 0 ? "自检通过\n" : "自检失败\n");
    return g_failed == 0 ? 0 : 1;
}

// 构造请求 payload（只拼字段，不校验——校验是设备端的事）
bool buildPayload(const Options& opt, std::string& payload) {
    if (opt.command == "ping") {
        payload = "PING";
    } else if (opt.command == "move") {
        if (opt.args.size() < 2) return false;
        payload = "MOVE," + opt.args[0] + "," + opt.args[1];
    } else if (opt.command == "joint") {
        if (opt.args.size() < 6) return false;
        payload = "JOINT";
        for (std::size_t i = 0; i < opt.args.size(); ++i) {
            payload += "," + opt.args[i];
        }
    } else if (opt.command == "log") {
        if (opt.args.size() < 1) return false;
        payload = "LOG," + opt.args[0];
    } else if (opt.command == "raw") {
        if (opt.args.empty()) return false;
        payload = opt.args[0];
    } else {
        return false;
    }
    return true;
}

int runCommand(const Options& opt) {
    Logger& log = Logger::instance();

    // 1) 创建串口：mock 模式用 MockSerialPort，否则用平台真实串口。
    //    统一用 unique_ptr 管理（RAII）：离开作用域或 move 进 Session 后自动释放，
    //    不用再手动 delete，也杜绝了「中途 return 漏释放」导致的内存泄漏。
    std::unique_ptr<serial::ISerialPort> port;
    if (opt.mock) {
        // mock 模式：先用「具体类型」造 Mock 串口、注入模拟应答，再 move 给基类指针 port。
        // 为什么不能直接写 port->inject(...)？inject() 是 MockSerialPort 独有的测试辅助方法，
        // 并没有写进 ISerialPort 接口（接口只放真实串口也有的 open/read/write/close）。
        // 拿基类指针调它，编译期直接报「has no member named 'inject'」——
        // 这恰恰证明「接口里不该塞某个实现特有的方法」，也是这个坑的活教材。
        auto mock = std::make_unique<serial::MockSerialPort>(opt.port);
        mock->inject(opt.command == "ping" ? Frame("OK,PONG").toString()
                                           : Frame("OK").toString());
        port = std::move(mock);   // move 之后 mock 变空，port 独占所有权（RAII 不变）
    } else {
        port = serial::createSerialPort(opt.port);   // 工厂现在也返回 unique_ptr
        if (port == nullptr) {
            log.error("当前平台没有可用的串口后端（Windows 用 MinGW/MSVC 编译，Linux 用原生 GCC）。"
                      "可用 --mock 演示。");
            return 1;
        }
    }

    if (!port->open()) {
        log.error("打开串口失败：" + opt.port);
        return 1;   // port 是 unique_ptr，离开作用域自动释放，无需手动 delete
    }

    // 2) 把端口 move 进会话：会话从此独占拥有它，port 自身变空、析构时自动释放。
    serial::SerialSession session(std::move(port));

    std::string payload;
    if (!buildPayload(opt, payload)) {
        log.error("参数不足或命令未知：" + opt.command);
        printUsage();
        return 1;
    }

    Frame req(payload);
    std::string txt = req.toString();
    while (!txt.empty() && (txt.back() == '\r' || txt.back() == '\n')) txt.pop_back();
    log.info("发送: " + txt);

    // 3) 发请求、等应答
    Frame resp;
    if (!session.request(req, resp, opt.timeout)) {
        log.error("超时未收到应答（" + std::to_string(opt.timeout) + " ms）");
        return 1;
    }

    log.info("应答: " + resp.payload());
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "selftest") {
        return runSelfTest();
    }

    // 注意：parseArgs 里的 std::stoi（--baud / --timeout）遇到非数字会抛异常，
    // 所以参数解析也必须放进 try——否则 armctl --baud abc ping 会以「未捕获异常」直接崩。
    try {
        Options opt;
        if (!parseArgs(argc, argv, opt) || opt.command.empty()) {
            printUsage();
            return 1;
        }
        return runCommand(opt);
    } catch (const std::exception& e) {
        Logger::instance().error(std::string("异常: ") + e.what());
        return 1;
    }
}
