// SerialSession 单元测试：客户端「发请求→收应答」闭环（用 Mock，无需硬件）
#include "serial/frame.hpp"
#include "serial/mock_serial_port.hpp"
#include "serial/session.hpp"

#include <iostream>
#include <string>
#include <memory>

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
using serial::MockSerialPort;
using serial::SerialSession;

static void test_request_gets_reply() {
    // Mock 用 unique_ptr 管理，离开作用域自动析构（不需要手动 delete）
    auto mock = std::make_unique<MockSerialPort>("mock");
    mock->open();
    // 设备应答已「就位」：注入 OK,PONG
    mock->inject(Frame("OK,PONG").toString());

    SerialSession session(std::move(mock));   // 把端口 move 进会话（会话独占拥有，析构自动释放）
    Frame resp;
    bool got = session.request(Frame("PING"), resp, 200);

    CHECK(got);
    CHECK(resp.payload() == "OK,PONG");
    CHECK(session.stats().frames_ok == 1);
}

static void test_request_skips_noise() {
    auto mock = std::make_unique<MockSerialPort>("mock");
    mock->open();
    // 前面是噪声（AT 回显之类），后面才是有效应答
    mock->inject("AT junk\r\n" + Frame("OK,PONG").toString());

    SerialSession session(std::move(mock));
    Frame resp;
    bool got = session.request(Frame("PING"), resp, 200);

    CHECK(got);
    CHECK(resp.payload() == "OK,PONG");
    CHECK(session.stats().noise_bytes > 0);
}

static void test_request_timeout() {
    auto mock = std::make_unique<MockSerialPort>("mock");
    mock->open();
    // 不注入任何应答：应超时返回 false
    SerialSession session(std::move(mock));
    Frame resp;
    bool got = session.request(Frame("PING"), resp, 50);
    CHECK(!got);
}

static void test_request_on_closed_port() {
    auto mock = std::make_unique<MockSerialPort>("mock");
    // 不 open：write 失败，request 应立即返回 false（而不是卡到超时）
    SerialSession session(std::move(mock));
    Frame resp;
    bool got = session.request(Frame("PING"), resp, 200);
    CHECK(!got);
}

int main() {
    test_request_gets_reply();
    test_request_skips_noise();
    test_request_timeout();
    test_request_on_closed_port();

    std::cout << "passed = " << g_passed << ", failed = " << g_failed << "\n";
    return g_failed == 0 ? 0 : 1;
}
