#pragma once

#include "serial/iserial_port.hpp"

#ifdef _WIN32

#include <windows.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>

namespace serial {

// Windows 串口实现：基于 CreateFile / SetCommState / ReadFile / WriteFile。
// 端口名形如 "COM3"，内部会补成 "\\\\.\\COM3"（COM10 以上必须这么写，否则打不开）。
class Win32SerialPort : public ISerialPort {
public:
    // 端口名用 const 引用接收，直接拷贝进成员变量 name_。
    // 新手版不玩「按值传参 + std::move」那套优化，老老实实拷贝最直观、也不容易出错。
    explicit Win32SerialPort(const std::string& portName) : name_(portName) {}

    // RAII：析构里 close，绝不把句柄泄漏给调用方去管。
    ~Win32SerialPort() override { close(); }

    bool open() override {
        if (h_ != INVALID_HANDLE_VALUE) {
            return true;
        }
        const std::string path = "\\\\.\\" + name_;
        h_ = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                         OPEN_EXISTING, 0, nullptr);
        if (h_ == INVALID_HANDLE_VALUE) {
            return false;
        }

        DCB dcb{};
        dcb.DCBlength = sizeof(dcb);
        if (!GetCommState(h_, &dcb)) {
            close();
            return false;
        }
        dcb.BaudRate = static_cast<DWORD>(config_.baudRate);
        dcb.ByteSize = static_cast<BYTE>(config_.dataBits);
        dcb.Parity = parityFlag();
        dcb.StopBits = stopBitsFlag();
        if (!SetCommState(h_, &dcb)) {
            close();
            return false;
        }

        // 默认非阻塞式读：ReadFile 立即返回当前可用字节，超时交给 read() 的 timeoutMs 控制
        COMMTIMEOUTS to{};
        to.ReadIntervalTimeout = MAXDWORD;
        to.ReadTotalTimeoutConstant = 0;
        to.ReadTotalTimeoutMultiplier = 0;
        SetCommTimeouts(h_, &to);

        return true;
    }

    void close() override {
        if (h_ != INVALID_HANDLE_VALUE) {
            CloseHandle(h_);
            h_ = INVALID_HANDLE_VALUE;
        }
    }

    bool isOpen() const override { return h_ != INVALID_HANDLE_VALUE; }

    int read(std::uint8_t* buffer, std::size_t size, int timeoutMs) override {
        if (h_ == INVALID_HANDLE_VALUE || buffer == nullptr || size == 0) {
            return -1;
        }
        // 简单实现：把整体超时设进 COMMTIMEOUTS，再做一次 ReadFile。
        // （更精细的做法是 OVERLAPPED + WaitForSingleObject，这里为了可读性用同步模型。）
        COMMTIMEOUTS to{};
        if (timeoutMs <= 0) {
            to.ReadIntervalTimeout = MAXDWORD;
            to.ReadTotalTimeoutConstant = 0;
            to.ReadTotalTimeoutMultiplier = 0;
        } else {
            to.ReadIntervalTimeout = MAXDWORD;
            to.ReadTotalTimeoutConstant = static_cast<DWORD>(timeoutMs);
            to.ReadTotalTimeoutMultiplier = 0;
        }
        SetCommTimeouts(h_, &to);

        DWORD got = 0;
        if (!ReadFile(h_, buffer, static_cast<DWORD>(size), &got, nullptr)) {
            return -1;
        }
        return static_cast<int>(got);
    }

    int write(const std::uint8_t* data, std::size_t size) override {
        if (h_ == INVALID_HANDLE_VALUE || (data == nullptr && size != 0)) {
            return -1;
        }
        DWORD sent = 0;
        if (!WriteFile(h_, data, static_cast<DWORD>(size), &sent, nullptr)) {
            return -1;
        }
        return static_cast<int>(sent);
    }

    std::string name() const override { return name_; }

    SerialConfig& config() { return config_; }

private:
    BYTE parityFlag() const {
        switch (config_.parity) {
            case 'E': return EVENPARITY;
            case 'O': return ODDPARITY;
            default:  return NOPARITY;
        }
    }

    BYTE stopBitsFlag() const {
        return config_.stopBits == 2 ? TWOSTOPBITS : ONESTOPBIT;
    }

    std::string name_;
    HANDLE h_ = INVALID_HANDLE_VALUE;
    SerialConfig config_;
};

}  // namespace serial

#endif  // _WIN32
