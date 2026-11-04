// 平台工厂测试：确认 createSerialPort 按编译平台返回正确后端。
// 构造不 open，全程不碰真实硬件。
#include "serial/mock_serial_port.hpp"
#include "serial/serial_port_factory.hpp"

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

int main() {
    // createSerialPort 现在返回 unique_ptr（独占所有权的智能指针），离开作用域自动释放。
    std::unique_ptr<serial::ISerialPort> p = serial::createSerialPort("COM999");

#if defined(_WIN32) || defined(__linux__)
    // 真实平台：工厂应返回对应后端对象（Windows→Win32SerialPort，Linux→LinuxSerialPort）
    CHECK(p != nullptr);
    if (p != nullptr) {
        CHECK(p->name() == "COM999");
        CHECK(!p->isOpen());   // 构造不等于打开
    }
#else
    // 无平台后端（例如本机 Cygwin 的 POSIX g++ 不定义 _WIN32）：工厂返回 nullptr，但 Mock 仍可用
    CHECK(p == nullptr);
#endif

    // Mock 与平台无关，任何机器都能用（这里用栈对象，离开作用域自动析构，不用手动 delete）
    serial::MockSerialPort m("mock");
    CHECK(m.name() == "mock");

    std::cout << "passed = " << g_passed << ", failed = " << g_failed << "\n";
    return g_failed == 0 ? 0 : 1;
}
