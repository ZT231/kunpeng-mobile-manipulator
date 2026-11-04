// RingBuffer<T> 单元测试：不依赖任何第三方框架，g++ 直接编译即可
#include "serial/ring_buffer.hpp"

#include <iostream>
#include <sstream>
#include <stdexcept>
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

using serial::RingBuffer;

static void test_empty() {
    RingBuffer<int> buf(4);
    CHECK(buf.empty());
    CHECK(buf.size() == 0);
    CHECK(buf.capacity() == 4);

    int v = -1;
    CHECK(buf.pop(v) == false);      // 空的时候取数据：返回 false，不是崩溃
    CHECK(v == -1);                  // 且不能碰 out 参数

    bool threw = false;
    try {
        buf.front();                 // 空的时候看数据：这是逻辑错误 -> 抛异常
    } catch (const std::out_of_range&) {
        threw = true;
    }
    CHECK(threw);
}

static void test_fifo_order() {
    RingBuffer<int> buf(4);
    CHECK(buf.push(1));
    CHECK(buf.push(2));
    CHECK(buf.push(3));
    CHECK(buf.size() == 3);

    int v = 0;
    CHECK(buf.pop(v) && v == 1);     // 先进先出
    CHECK(buf.pop(v) && v == 2);
    CHECK(buf.size() == 1);
}

static void test_full_and_wrap() {
    RingBuffer<int> buf(3);
    CHECK(buf.push(10));
    CHECK(buf.push(20));
    CHECK(buf.push(30));
    CHECK(buf.full());
    CHECK(buf.push(40) == false);    // 满时拒绝写入，由调用方决定策略

    int v = 0;
    CHECK(buf.pop(v) && v == 10);
    CHECK(buf.push(40));             // 出现回绕：tail_ 绕回 0
    CHECK(buf.size() == 3);
    CHECK(buf[0] == 20 && buf[1] == 30 && buf[2] == 40);  // 逻辑下标仍然是顺序的
    CHECK(buf.front() == 20 && buf.back() == 40);
}

static void test_capacity_one() {
    RingBuffer<int> buf(1);          // 容量 1 是最容易写错的边界（full/empty 同时成立）
    CHECK(buf.push(7));
    CHECK(buf.full());
    CHECK(buf.push(8) == false);
    int v = 0;
    CHECK(buf.pop(v) && v == 7);
    CHECK(buf.empty());
}

static void test_copy_is_deep() {
    RingBuffer<int> a(4);
    a.push(1);
    a.push(2);

    RingBuffer<int> b = a;           // 拷贝构造（深拷贝）
    CHECK(b.size() == 2);
    b.push(3);                       // 改 b 不能影响 a
    CHECK(a.size() == 2);
    CHECK(b.size() == 3);

    b[0] = 99;
    CHECK(a[0] == 1);

    RingBuffer<int> c(2);
    c.push(-1);
    c = a;                           // 拷贝赋值（会先释放 c 原有的内存）
    CHECK(c.size() == 2 && c[0] == 1 && c[1] == 2 && c.capacity() == a.capacity());
}

static void test_self_assignment() {
    RingBuffer<int> a(4);
    a.push(5);
    a = a;                           // 自赋值不能崩、不能丢数据
    CHECK(a.size() == 1 && a[0] == 5);
}

static void test_stream_output() {
    RingBuffer<int> buf(4);
    buf.push(1);
    buf.push(2);
    std::ostringstream oss;
    oss << buf;
    CHECK(oss.str() == "RingBuffer(2/4, [1, 2])");
}

static void test_generic_type() {
    RingBuffer<std::string> buf(2);     // 模板的意义：T 换成 string 也照跑
    buf.push("hello");
    buf.push("world");
    CHECK(buf.full());
    CHECK(buf[0] == "hello" && buf[1] == "world");
}

int main() {
    test_empty();
    test_fifo_order();
    test_full_and_wrap();
    test_capacity_one();
    test_copy_is_deep();
    test_self_assignment();
    test_stream_output();
    test_generic_type();

    std::cout << "passed = " << g_passed << ", failed = " << g_failed << "\n";
    return g_failed == 0 ? 0 : 1;
}
