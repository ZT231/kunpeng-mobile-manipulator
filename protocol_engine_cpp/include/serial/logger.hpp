#pragma once

#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>

namespace serial {

// 日志级别：从最啰嗦的 Debug 到最严重的 Error
enum class LogLevel { Debug = 0, Info = 1, Warn = 2, Error = 3 };

// 日志器：分级过滤 + 可选写文件 + 线程安全。
//
// 设计点（新手重点看这三处）：
//   1) 单例：全局只有一个日志器，避免到处传对象。用「函数内 static 变量」实现（Meyers 单例），
//      C++11 保证它初始化是线程安全的，且第一次调用时才构造（懒加载）。
//   2) 分级：低于设定级别的日志直接丢弃，比如线上只看 Warn/Error，就不打 Debug。
//   3) 线程安全：接收线程和主线程可能同时写日志，用 mutex 加锁，否则两条日志会交织错乱。
class Logger {
public:
    // 取全局唯一实例：每次调用都拿到同一个对象
    static Logger& instance() {
        static Logger inst;
        return inst;
    }

    void setLevel(LogLevel level) {
        std::lock_guard<std::mutex> lock(mutex_);
        level_ = level;
    }

    // 写文件（追加，不覆盖历史日志）；再次调用会先关掉旧文件
    void enableFile(const std::string& path) {
        std::lock_guard<std::mutex> lock(mutex_);
        file_.close();
        file_.open(path, std::ios::app);
    }

    void log(LogLevel level, const std::string& message) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (static_cast<int>(level) < static_cast<int>(level_)) {
            return;   // 低于阈值就丢弃
        }
        const std::string line = timestamp() + " [" + levelName(level) + "] " + message;
        std::cout << line << "\n";
        if (file_.is_open()) {
            file_ << line << std::endl;   // endl 会刷盘：保证日志立刻落文件，而不是攒在缓冲里
        }
    }

    // 几个简便函数，少打几个字
    void debug(const std::string& m) { log(LogLevel::Debug, m); }
    void info(const std::string& m)  { log(LogLevel::Info,  m); }
    void warn(const std::string& m)  { log(LogLevel::Warn,  m); }
    void error(const std::string& m) { log(LogLevel::Error, m); }

private:
    // 构造函数设为 private：禁止在类外用 Logger x; 直接造对象，只能走 instance()。
    Logger() = default;
    // 禁止拷贝：单例不能有第二个实例。= delete 就是「编译器层面禁止使用拷贝构造/赋值」。
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    // 生成 "年-月-日 时:分:秒" 时间戳字符串
    static std::string timestamp() {
        std::time_t t = std::time(nullptr);
        std::tm* ptm = std::localtime(&t);   // 本函数总在 log() 的 mutex 保护下调，已序列化，安全
        char buf[32] = {0};
        if (ptm != nullptr) {
            std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", ptm);
        }
        return buf;
    }

    static const char* levelName(LogLevel level) {
        switch (level) {
            case LogLevel::Debug: return "DEBUG";
            case LogLevel::Info:  return "INFO";
            case LogLevel::Warn:  return "WARN";
            case LogLevel::Error: return "ERROR";
        }
        return "?";
    }

    std::mutex mutex_;                // 保护下面所有成员的并发访问
    LogLevel level_ = LogLevel::Info;
    std::ofstream file_;              // 日志文件流（可选）
};

}  // namespace serial
