// MockSerialPort 单元测试：验证接口语义，为"不插硬件测全链路"打底
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

using serial::MockSerialPort;

// 把 uint8_t 缓冲区转成 std::string：逐字符拷贝（static_cast<char>），
// 和库代码 session.hpp 里「字节流 -> 字符串」的写法一致，不用 reinterpret_cast。
static std::string bufToString(const std::uint8_t* buf, std::size_t n) {
    std::string s;
    for (std::size_t i = 0; i < n; ++i) {
        s.push_back(static_cast<char>(buf[i]));
    }
    return s;
}

static void test_lifecycle() {
    MockSerialPort port("mock");
    CHECK(!port.isOpen());
    CHECK(port.name() == "mock");

    CHECK(port.open());
    CHECK(port.isOpen());

    port.close();
    CHECK(!port.isOpen());
}

static void test_read_write_roundtrip() {
    MockSerialPort port;
    port.open();

    // 上位机写出去的数据被记录，可核对
    const std::uint8_t out[] = {'H', 'i', '!', '\r', '\n'};
    CHECK(port.write(out, sizeof(out)) == static_cast<int>(sizeof(out)));
    CHECK(port.sentString() == "Hi!\r\n");

    // 注入设备回包，read 按 FIFO 顺序取
    port.inject(std::vector<std::uint8_t>{'$', 'P', 'I', 'N', 'G', '*', '1', '0', '\r', '\n'});

    std::uint8_t buf[16] = {};
    const int n = port.read(buf, sizeof(buf), 0);
    CHECK(n == 10);
    CHECK(bufToString(buf, static_cast<std::size_t>(n)) == "$PING*10\r\n");

    // 读空了返回 0（不是负数：0 = 没数据，负 = 出错）
    CHECK(port.read(buf, sizeof(buf), 0) == 0);
    CHECK(port.rxAvailable() == 0);

    // 分多次注入，read 能跨越注入边界连续取
    port.inject(std::string("AB"));
    port.inject(std::string("CD"));
    const int n2 = port.read(buf, sizeof(buf), 0);
    CHECK(n2 == 4);
    CHECK(bufToString(buf, 4) == "ABCD");
}

static void test_closed_behavior() {
    MockSerialPort port;
    std::uint8_t buf[8] = {};
    // 未打开就读写：返回 -1（错误），而不是崩溃或返回 0
    CHECK(port.read(buf, sizeof(buf), 0) == -1);
    const std::uint8_t x = 'x';
    CHECK(port.write(&x, 1) == -1);

    port.open();
    CHECK(port.write(&x, 1) == 1);
    port.clearSent();
    CHECK(port.sentBytes().empty());
}

static void test_fragmented_inject() {
    MockSerialPort port;
    port.open();
    // 模拟真实串口"一帧被拆成好几段到"：分三次 inject，read 一次拿回完整帧
    port.inject(std::string("$MOVE,"));
    port.inject(std::string("0,1500"));
    port.inject(std::string("*25\r\n"));

    std::uint8_t buf[32] = {};
    const int n = port.read(buf, sizeof(buf), 0);
    CHECK(n == 17);   // "$MOVE,0,1500*25\r\n" 长度 17
    CHECK(bufToString(buf, static_cast<std::size_t>(n)) == "$MOVE,0,1500*25\r\n");
}

int main() {
    test_lifecycle();
    test_read_write_roundtrip();
    test_closed_behavior();
    test_fragmented_inject();

    std::cout << "passed = " << g_passed << ", failed = " << g_failed << "\n";
    return g_failed == 0 ? 0 : 1;
}
