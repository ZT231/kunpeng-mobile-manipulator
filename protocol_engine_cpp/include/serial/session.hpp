#pragma once

#include "serial/frame.hpp"
#include "serial/frame_parser.hpp"
#include "serial/iserial_port.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>
#include <memory>

namespace serial {

// 一次「发请求 → 等应答」的会话，把串口 + 解析器包成一个高层入口。
// 这是上位机的「客户端」角色：发送命令帧，等设备回一帧，返回解析好的应答。
// （「设备端/模拟器」角色由 CommandDispatcher + commands.hpp 承担，两者职责分开。）
class SerialSession {
public:
    // 构造时把串口「独占所有权」move 进来（unique_ptr）。
    // 会话从此拥有这个串口：析构时 port_ 自动释放，调用方不用再 delete——
    // 这就是 RAII：资源（串口句柄）的生命周期绑定到拥有它的对象上。
    explicit SerialSession(std::unique_ptr<ISerialPort> port) : port_(std::move(port)) {}

    ISerialPort& port() { return *port_; }
    const ISerialPort& port() const { return *port_; }
    const FrameParser::Stats& stats() const { return parser_.stats(); }

    bool open() { return port_->open(); }
    void close() { port_->close(); }

    // 发一帧，循环读直到收到一帧应答或超时。
    // 成功把应答写进 out 并返回 true；超时/出错返回 false。
    bool request(const Frame& request, Frame& out, int timeoutMs = 1000) {
        std::string text = request.toString();

        // 串口底层按「字节」收发，Frame 的字符串要变成字节数组再发。
        // char 和 uint8_t 都是 1 字节，这里逐字符复制，安全且易懂（不用 reinterpret_cast）。
        std::vector<std::uint8_t> bytes(text.begin(), text.end());
        if (!bytes.empty()) {
            int written = port_->write(bytes.data(), bytes.size());
            if (written < 0) {
                return false;   // 写失败
            }
        }

        const std::uint64_t deadline = nowMs() + static_cast<std::uint64_t>(timeoutMs);

        std::uint8_t buf[256];   // 一次最多读 256 字节
        while (nowMs() < deadline) {
            int remain = static_cast<int>(deadline - nowMs());
            if (remain < 1) remain = 1;   // 至少等 1ms，避免最后一次死循环

            int n = port_->read(buf, sizeof(buf), remain);
            if (n < 0) {
                return false;   // 读错误
            }
            if (n > 0) {
                // 把读到的字节重新拼成字符串，喂给解析器（bytesToString 是公共转换，避免每处重写循环）
                std::string s = bytesToString(buf, static_cast<std::size_t>(n));
                std::vector<Frame> frames = parser_.feed(s);
                if (!frames.empty()) {
                    out = frames[0];   // 第一帧就是应答（连发/粘包时取最早一帧）
                    return true;
                }
            }
            // 暂时没数据：让出一点 CPU，避免忙等把单核跑满
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return false;   // 超时
    }

private:
    // 返回「当前时间，单位毫秒」。用来算超时截止时刻，比反复调系统时间方便。
    static std::uint64_t nowMs() {
        return static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
    }

    std::unique_ptr<ISerialPort> port_;    // 独占拥有串口：析构时自动释放（RAII），不用手动 delete
    FrameParser parser_;   // 复用一个解析器处理回包
};

}  // namespace serial
