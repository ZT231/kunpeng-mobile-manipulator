// BlockingQueue 单元测试：FIFO / 超时 / close 唤醒 / 生产者-消费者
#include "serial/blocking_queue.hpp"

#include <chrono>
#include <iostream>
#include <optional>
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

using serial::BlockingQueue;

static void test_fifo_and_timeout() {
    BlockingQueue<int> q;
    q.push(1);
    q.push(2);
    q.push(3);

    // tryPop 返回 std::optional<int>：有值返回包装好的数，空返回 std::nullopt
    std::optional<int> a = q.tryPop(10);
    CHECK(a.has_value() && a.value() == 1);   // FIFO：先判「有值」，再取 value()
    a = q.tryPop(10);
    CHECK(a.has_value() && a.value() == 2);
    a = q.tryPop(10);
    CHECK(a.has_value() && a.value() == 3);
    CHECK(!q.tryPop(20).has_value());         // 空了：等 20ms 后超时返回「空」
}

static void test_close_wakes_consumer() {
    BlockingQueue<int> q;
    bool got = false;

    // 消费者线程：阻塞等 5000ms（正常情况下会等满）
    std::thread consumer([&q, &got]() {
        got = q.tryPop(5000).has_value();   // 空队列时 close 应立刻返回「空」，而不是等满 5 秒
    });

    // 稍等让消费者先睡下，然后 close：应立即唤醒，而不是等满 5 秒
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    q.close();

    consumer.join();   // 如果 close 没唤醒它，这里会卡 5 秒
    CHECK(got == false);
}

static void test_producer_consumer() {
    BlockingQueue<int> q;
    const int kN = 1000;

    // 一个生产者线程往队列里塞 0..N-1
    std::thread producer([&q]() {
        for (int i = 0; i < kN; ++i) {
            q.push(i);
        }
    });

    // 主线程当消费者，全部取完
    int got = 0;
    long long sum = 0;
    while (got < kN) {
        std::optional<int> v = q.tryPop(2000);
        if (v) {                 // optional 能直接当 bool 用：有值 true、空 false
            sum += v.value();
            ++got;
        }
    }
    producer.join();

    CHECK(got == kN);
    CHECK(sum == static_cast<long long>(kN) * (kN - 1) / 2);   // 一个不漏、不重、不丢
}

int main() {
    test_fifo_and_timeout();
    test_close_wakes_consumer();
    test_producer_consumer();

    std::cout << "passed = " << g_passed << ", failed = " << g_failed << "\n";
    return g_failed == 0 ? 0 : 1;
}
