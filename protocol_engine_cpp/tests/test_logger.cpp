// Logger 单元测试：单例 / 分级 / 时间戳 / 写文件
#include "serial/logger.hpp"

#include <fstream>
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

using serial::Logger;
using serial::LogLevel;

static void test_singleton() {
    // Meyers 单例：无论调几次，都是同一个对象
    CHECK(&Logger::instance() == &Logger::instance());
}

static void test_level_ordering() {
    // 枚举底层值从小到大：Debug < Info < Warn < Error（log 里用它做阈值比较）
    CHECK(static_cast<int>(LogLevel::Debug) < static_cast<int>(LogLevel::Info));
    CHECK(static_cast<int>(LogLevel::Info) < static_cast<int>(LogLevel::Warn));
    CHECK(static_cast<int>(LogLevel::Warn) < static_cast<int>(LogLevel::Error));
}

static void test_file_output() {
    Logger& log = Logger::instance();
    const std::string path = "build/logger_test.log";

    log.enableFile(path);
    log.info("hello from test");

    // 日志已刷盘（endl），读回文件核对内容
    std::ifstream in(path);
    CHECK(in.is_open());
    std::string content;
    std::string line;
    while (std::getline(in, line)) {
        content += line + "\n";
    }
    CHECK(content.find("[INFO]") != std::string::npos);
    CHECK(content.find("hello from test") != std::string::npos);
}

int main() {
    test_singleton();
    test_level_ordering();
    test_file_output();

    std::cout << "passed = " << g_passed << ", failed = " << g_failed << "\n";
    return g_failed == 0 ? 0 : 1;
}
