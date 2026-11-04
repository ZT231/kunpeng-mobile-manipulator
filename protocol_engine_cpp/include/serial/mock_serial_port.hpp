#pragma once

#include "serial/iserial_port.hpp"
#include "serial/ring_buffer.hpp"

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace serial {

// 内存里的「假串口」：不碰任何硬件，用于单元测试和「不插机械臂也能跑通全链路」。
//
// 它实现了 ISerialPort 的全部接口，但读写都在内存里完成：
//   - 调用 inject(...) 模拟「设备发来数据」；read() 把数据按 FIFO 取出来；
//   - write() 把上位机写出的字节存进 tx_，用 sentString() 核对「上位机到底发了什么」。
//
// 为什么需要它？没有它，每次测解析/分派都得插真硬件、连真机械臂，又慢又难复现。
// 用 Mock，所有逻辑都能在纯软件里测，且结果可重复。
class MockSerialPort : public ISerialPort {
public:
    // 构造函数：取个名字（默认 "mock"），并给接收缓冲区 rx_ 一个容量。
    explicit MockSerialPort(std::string name = "mock", std::size_t rxCapacity = 4096)
        : rx_(rxCapacity), name_(name) {}

    bool open() override {
        std::lock_guard<std::mutex> lock(mutex_);   // 加锁：多线程下保护下面的 opened_
        opened_ = true;
        return true;
    }

    void close() override {
        std::lock_guard<std::mutex> lock(mutex_);
        opened_ = false;
    }

    bool isOpen() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return opened_;
    }

    int read(std::uint8_t* buffer, std::size_t size, int /*timeoutMs*/) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!opened_ || buffer == nullptr) {
            return -1;   // 没打开或参数错 -> 返回负数表示出错（约定：0=没数据，负=出错）
        }
        std::size_t n = 0;
        while (n < size) {
            std::uint8_t b = 0;
            if (!rx_.pop(b)) {
                break;   // 缓冲区空了就停
            }
            buffer[n++] = b;
        }
        return static_cast<int>(n);
    }

    int write(const std::uint8_t* data, std::size_t size) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!opened_ || (data == nullptr && size != 0)) {
            return -1;
        }
        // 把要发的字节追加到 tx_ 末尾（像写到发送缓存）
        for (std::size_t i = 0; i < size; ++i) {
            tx_.push_back(data[i]);
        }
        return static_cast<int>(size);
    }

    std::string name() const override { return name_; }

    // ---- 测试辅助（Mock 独有，真实串口没有这些）----

    // 模拟「设备发来一串字节」，直接塞进接收缓冲区 rx_。
    void inject(const std::vector<std::uint8_t>& bytes) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (std::size_t i = 0; i < bytes.size(); ++i) {
            rx_.push(bytes[i]);
        }
    }

    // 重载：方便直接传字符串，比如 inject("$PING*10\r\n")
    void inject(const std::string& s) {
        std::vector<std::uint8_t> bytes(s.begin(), s.end());
        inject(bytes);
    }

    // 返回「上位机已写出字节」的副本（返回副本，避免把锁内的引用暴露给并发方）
    std::vector<std::uint8_t> sentBytes() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return tx_;
    }

    std::string sentString() const {
        std::vector<std::uint8_t> copy = sentBytes();
        return std::string(copy.begin(), copy.end());
    }

    void clearSent() {
        std::lock_guard<std::mutex> lock(mutex_);
        tx_.clear();
    }

    std::size_t rxAvailable() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return rx_.size();
    }

private:
    mutable std::mutex mutex_;        // mutable：const 成员函数里也要能加锁
    RingBuffer<std::uint8_t> rx_;     // 接收缓冲区（设备 -> 上位机）
    std::vector<std::uint8_t> tx_;    // 发送记录（上位机 -> 设备），仅用于核对
    std::string name_;
    bool opened_ = false;
};

}  // namespace serial
