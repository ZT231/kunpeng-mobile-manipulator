#pragma once

#include "serial/blocking_queue.hpp"
#include "serial/frame.hpp"
#include "serial/frame_parser.hpp"
#include "serial/iserial_port.hpp"
#include "serial/logger.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace serial {

// 异步接收器：一个后台线程持续「读串口 → 喂状态机 → 把完整帧 push 进线程安全队列」，
// 主线程用 tryPop 取走处理。把「阻塞读」从主线程剥离，主线程可以同时干别的
//（比如响应按键、刷新 UI、周期遥测）。
//
// 生命周期约定：
//   - 单次使用：start() 只能调一次（内部有 started_ 标志）。
//   - stop() 置停止标志并 join，返回时线程已退出。
//   - port 由外部持有，其生命周期必须覆盖 AsyncReceiver 的使用期（这里只存指针）。
//   - stats() 在 stop() 之后读是准确的；运行中读是近似值（与工作线程存在良性竞态，仅诊断用）。
class AsyncReceiver {
public:
    // 构造时把串口对象的地址交进来（用引用，保证一定是有效对象）
    explicit AsyncReceiver(ISerialPort& port) : port_(&port) {}

    // 析构时自动 stop：即使调用方忘了调 stop，线程也会被正确回收，不会泄漏。
    ~AsyncReceiver() { stop(); }

    // 禁止拷贝：一个接收器对应一个线程，拷贝会出乱子。直接删掉拷贝构造/赋值。
    AsyncReceiver(const AsyncReceiver&) = delete;
    AsyncReceiver& operator=(const AsyncReceiver&) = delete;

    bool start() {
        if (started_) {
            return false;   // 单次使用，重复 start 直接失败
        }
        started_ = true;
        stop_.store(false);   // atomic 的 store：线程安全地写「停止标志」
        worker_ = std::thread(&AsyncReceiver::runLoop, this);   // 起后台线程
        running_.store(true);
        return true;
    }

    void stop() {
        if (!running_.load()) {
            return;   // 没在跑就什么都不做
        }
        stop_.store(true);            // 告诉工作线程「可以退出了」
        if (worker_.joinable()) {
            worker_.join();           // 等线程真正退出（join 会阻塞到这里）
        }
        running_.store(false);
    }

    bool running() const { return running_.load(); }

    // 消费者：等一帧，返回「包装好的帧」；超时 / 队列关闭后返回「空」。
    // 直接转发给底层 BlockingQueue::tryPop（它返回 std::optional<Frame>）。
    std::optional<Frame> tryPop(int timeoutMs) {
        return queue_.tryPop(timeoutMs);
    }

    // 诊断统计：stop() 之后读准确；运行中读是近似值
    const FrameParser::Stats& stats() const { return parser_.stats(); }

private:
    // 后台线程主循环：反复读串口、解析、入队，直到 stop_ 被置位。
    void runLoop() {
        Logger::instance().info("接收线程启动: " + port_->name());

        std::uint8_t buf[256];
        while (!stop_.load()) {
            // 50ms 超时：让循环有机会检查 stop_，别把线程永久卡死在 read 上
            int n = port_->read(buf, sizeof(buf), 50);
            if (n > 0) {
                std::string s = bytesToString(buf, static_cast<std::size_t>(n));
                std::vector<Frame> frames = parser_.feed(s);
                for (std::size_t i = 0; i < frames.size(); ++i) {
                    queue_.push(std::move(frames[i]));   // 移动进队列：frames[i] 用完即弃，直接搬不拷贝
                }
            } else if (n < 0) {
                // 读错误：可能串口被拔/没开。记日志、短睡后继续，不退出（等它恢复）
                Logger::instance().warn("读串口出错: " + port_->name());
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            } else {
                // 无数据：让出一点 CPU，避免忙等
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

        queue_.close();   // 通知消费者：不会再有新帧了
        Logger::instance().info("接收线程退出: " + port_->name());
    }

    ISerialPort* port_;                 // 串口对象地址（不拥有）
    FrameParser parser_;                // 复用一个解析器
    BlockingQueue<Frame> queue_;        // 解析好的帧排队等主线程取
    std::thread worker_;                // 后台线程
    std::atomic<bool> running_{false};  // 是否正在运行（原子量：多线程读写安全）
    std::atomic<bool> stop_{false};     // 停止标志（原子量：工作线程和主线程都能安全读写）
    bool started_ = false;              // 是否已 start 过（保证只 start 一次）
};

}  // namespace serial
