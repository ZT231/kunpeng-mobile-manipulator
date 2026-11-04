#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <utility>

namespace serial {

// 线程安全队列（生产者-消费者模式）：一个/多个线程 push，一个/多个线程带超时 tryPop。
// 典型用法：接收线程把解析好的帧 push 进来，主线程 tryPop 出来处理。
//
// 为什么不用「主线程自己死循环抢着读」？
//   那样会空转 CPU（忙等）。这里用 std::condition_variable：没数据时消费者睡着，
//   push 时被唤醒，不空转。这是「线程协作」最基础的积木。
template <typename T>
class BlockingQueue {
public:
    // 生产者：入队并唤醒一个正在等的消费者。已 close 后入队被忽略。
    void push(T value) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (closed_) {
                return;
            }
            // 移动语义：value 是「按值传参」进来的副本，没人再用了，直接「搬」进队列不再拷贝。
            // 对 Frame 这种里面带 std::string 的对象，搬 = 偷走字符串指针，省一次字符串内存复制。
            queue_.push_back(std::move(value));
        }
        cv_.notify_one();   // 放锁之后再通知，避免被唤醒的线程立刻又卡在锁上
    }

    // 消费者：有数据就取走返回「包装好的值」，超时 / 已关闭且空 返回「空」。
    // 返回类型用 std::optional<T>：optional 专门表达「可能没有值」——
    //   有值返回 optional(值)，没值返回 std::nullopt（空）。
    // 为什么不用「bool 成功 + 出参」？也能用，但 optional 更安全：
    //   它把「有没有值」写进类型，调用方必须先 if (结果) 判断，才能解引用 *结果 拿到值；
    //   想不判断直接当值用，编译期就报错。C++17 里「可能取不到」的标准写法就是 optional。
    std::optional<T> tryPop(int timeoutMs) {
        std::unique_lock<std::mutex> lock(mutex_);
        // 用「循环 + 不带谓词的 wait_for」代替「带 lambda 谓词的 wait_for」：
        // 每醒来一次就自己再查一遍条件，既防伪唤醒，又不用 lambda（新手更好懂）。
        while (queue_.empty() && !closed_) {
            // 无谓词版 wait_for 返回 std::cv_status：timeout 表示「等了 timeoutMs 都没被唤醒」
            if (cv_.wait_for(lock, std::chrono::milliseconds(timeoutMs)) ==
                std::cv_status::timeout) {
                return std::nullopt;   // 超时了还空着 -> 返回「空」
            }
            // 被唤醒（可能是 push 或 close），回到 while 顶部再查一次条件
        }
        if (queue_.empty()) {
            return std::nullopt;   // 已关闭且队列空
        }
        // 把队头元素「搬」出来（移动语义，不再拷贝），用这个值构造 optional 返回
        std::optional<T> result(std::move(queue_.front()));
        queue_.pop_front();
        return result;
    }

    // 关闭：唤醒所有等待者。之后 push 无效；tryPop 在队列排空后立即返回「空」
    //（而不是傻等满整个超时）。接收线程退出时用它通知主线程「不会再有数据了」。
    void close() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            closed_ = true;
        }
        cv_.notify_all();
    }

    bool closed() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return closed_;
    }

    // 非阻塞快照（诊断用，别拿来做同步判断）
    std::size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

private:
    mutable std::mutex mutex_;           // 保护下面所有成员
    std::condition_variable cv_;        // 条件变量：用来「睡 / 唤醒」
    std::deque<T> queue_;               // 底层用双端队列存数据
    bool closed_ = false;               // 是否已关闭
};

}  // namespace serial
