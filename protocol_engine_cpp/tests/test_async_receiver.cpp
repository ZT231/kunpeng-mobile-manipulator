// AsyncReceiver 单元测试：后台接收线程把帧异步送进队列，主线程 tryPop 取走
#include "serial/async_receiver.hpp"
#include "serial/frame.hpp"
#include "serial/mock_serial_port.hpp"

#include <chrono>
#include <iostream>
#include <optional>
#include <string>
#include <thread>

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

using serial::AsyncReceiver;
using serial::Frame;
using serial::MockSerialPort;

static void test_receive_frames_async() {
    MockSerialPort port("mock");
    port.open();

    AsyncReceiver rx(port);
    CHECK(rx.start());
    CHECK(rx.running());

    // 注入两条帧：接收线程会异步读到、解析、送进队列
    port.inject(Frame("OK,PONG").toString());
    port.inject(Frame("OK,S,M,1500,1500,1500,1500,1500,1500").toString());

    // tryPop 返回 std::optional<Frame>：有帧返回包装好的帧，超时/关闭返回「空」
    std::optional<Frame> f = rx.tryPop(1000);
    CHECK(f.has_value());
    CHECK(f.value().payload() == "OK,PONG");

    f = rx.tryPop(1000);
    CHECK(f.has_value());
    CHECK(f.value().command() == "OK" && f.value().args().size() == 8);

    rx.stop();
    CHECK(!rx.running());

    // stop 后队列已关闭：再 tryPop 立即返回「空」，不会卡满超时
    CHECK(!rx.tryPop(50).has_value());
}

static void test_timeout_when_empty() {
    MockSerialPort port("mock");
    port.open();

    AsyncReceiver rx(port);
    rx.start();

    CHECK(!rx.tryPop(50).has_value());   // 没注入任何数据：50ms 超时返回「空」

    rx.stop();
}

static void test_clean_stop() {
    MockSerialPort port("mock");
    port.open();

    AsyncReceiver rx(port);
    rx.start();
    rx.stop();               // 应立即 join，不挂起、不卡死

    CHECK(!rx.running());
    // 析构再调一次 stop 是幂等的，不会崩
}

static void test_read_error_does_not_hang() {
    MockSerialPort port("mock");
    port.open();

    AsyncReceiver rx(port);
    rx.start();

    port.close();            // 关闭端口：接收线程 read 会返回 -1（读错误），但不能崩、不能卡死

    // 给接收线程一点时间走到"读错误"分支
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    rx.stop();               // 必须能正常停
    CHECK(!rx.running());
}

int main() {
    test_receive_frames_async();
    test_timeout_when_empty();
    test_clean_stop();
    test_read_error_does_not_hang();

    std::cout << "passed = " << g_passed << ", failed = " << g_failed << "\n";
    return g_failed == 0 ? 0 : 1;
}
