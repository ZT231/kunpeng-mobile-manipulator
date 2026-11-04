#pragma once

#include "serial/iserial_port.hpp"

// 注意：Linux 实现依赖 <termios.h> / <fcntl.h> / <unistd.h>，只能在 Linux 下编译。
// 本文件用 __linux__ 守卫，在 Windows 上完全惰性（不参与编译）。
// 要验证它：在 Linux 机器（或 WSL）上 `g++ -std=c++17 -Iinclude -c` 编译一次即可。
#ifdef __linux__

#include <fcntl.h>    // open() / O_RDWR / O_NOCTTY 都来自这里，必须显式 include
#include <termios.h>
#include <unistd.h>

#include <sys/select.h>
#include <sys/types.h>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <utility>

namespace serial {

// Linux 串口实现：基于 open("/dev/ttyUSB0") + tcgetattr/tcsetattr + read/write。
class LinuxSerialPort : public ISerialPort {
public:
    // 端口名用 const 引用接收，直接拷贝进成员变量 name_。
    // 新手版不玩「按值传参 + std::move」那套优化，老老实实拷贝最直观、也不容易出错。
    explicit LinuxSerialPort(const std::string& portName) : name_(portName) {}

    ~LinuxSerialPort() override { close(); }

    bool open() override {
        if (fd_ >= 0) {
            return true;
        }
        fd_ = ::open(name_.c_str(), O_RDWR | O_NOCTTY);
        if (fd_ < 0) {
            return false;
        }

        termios t{};
        if (tcgetattr(fd_, &t) != 0) {
            close();
            return false;
        }
        // 波特率
        speed_t speed = mapBaud(config_.baudRate);
        cfsetispeed(&t, speed);
        cfsetospeed(&t, speed);
        // 8N1
        t.c_cflag &= ~PARENB;   // 无校验
        t.c_cflag &= ~CSTOPB;   // 1 停止位
        t.c_cflag &= ~CSIZE;
        t.c_cflag |= CS8;
        t.c_cflag &= ~CRTSCTS;  // 关硬件流控
        t.c_cflag |= (CLOCAL | CREAD);
        // 原始模式：不做行处理
        t.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        t.c_iflag &= ~(IXON | IXOFF | IXANY);
        t.c_oflag &= ~OPOST;
        t.c_cc[VMIN] = 0;       // 非阻塞式读
        t.c_cc[VTIME] = 0;

        if (tcsetattr(fd_, TCSANOW, &t) != 0) {
            close();
            return false;
        }
        return true;
    }

    void close() override {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    bool isOpen() const override { return fd_ >= 0; }

    int read(std::uint8_t* buffer, std::size_t size, int timeoutMs) override {
        if (fd_ < 0 || buffer == nullptr || size == 0) {
            return -1;
        }
        if (timeoutMs > 0) {
            // 用 select 做简单超时（比忙等省 CPU，也不阻塞死）
            fd_set set;
            FD_ZERO(&set);
            FD_SET(fd_, &set);
            timeval tv{};
            tv.tv_sec = timeoutMs / 1000;
            tv.tv_usec = (timeoutMs % 1000) * 1000;
            const int ready = ::select(fd_ + 1, &set, nullptr, nullptr, &tv);
            if (ready == 0) {
                return 0;   // 超时：无数据
            }
            if (ready < 0) {
                return -1;
            }
        }
        const ssize_t n = ::read(fd_, buffer, size);
        if (n < 0) {
            return (errno == EAGAIN || errno == EWOULDBLOCK) ? 0 : -1;
        }
        return static_cast<int>(n);
    }

    int write(const std::uint8_t* data, std::size_t size) override {
        if (fd_ < 0 || (data == nullptr && size != 0)) {
            return -1;
        }
        const ssize_t n = ::write(fd_, data, size);
        return n < 0 ? -1 : static_cast<int>(n);
    }

    std::string name() const override { return name_; }

    SerialConfig& config() { return config_; }

private:
    static speed_t mapBaud(int baud) {
        switch (baud) {
            case 9600:   return B9600;
            case 19200:  return B19200;
            case 38400:  return B38400;
            case 57600:  return B57600;
            case 115200: return B115200;
            case 230400: return B230400;
            case 460800: return B460800;
            case 921600: return B921600;
            default:     return B115200;
        }
    }

    std::string name_;
    int fd_ = -1;
    SerialConfig config_;
};

}  // namespace serial

#endif  // __linux__
