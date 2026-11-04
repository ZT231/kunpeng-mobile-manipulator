#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace serial {

// 串口配置。先给常见默认值（115200 8N1），各平台实现会把它翻译成对应系统结构。
struct SerialConfig {
    int baudRate = 115200;   // 波特率：每秒传多少比特
    int dataBits = 8;        // 数据位：每字节 8 位
    char parity = 'N';       // 校验位：'N' 无 / 'E' 偶 / 'O' 奇
    int stopBits = 1;        // 停止位：1 位
};

// 串口抽象接口：定义「串口能做什么」的契约。
// Windows / Linux / Mock 三个实现各自落地这套契约。
//
// 这是整个项目「跨平台」三个字的落点：上层代码只依赖 ISerialPort 这个接口，
// 不关心底层是 CreateFile（Windows）还是 open（Linux），甚至可以是内存里的 Mock（不插硬件就能跑）。
// 这就是「面向接口编程」：依赖抽象，不依赖具体实现。
//
// 为什么用「纯虚函数 + 继承」？因为不同平台的打开/读写实现完全不同，
// 但上层只想要一个统一的 open()/read()/write()。把差异藏在子类里，
// 上层拿着基类指针就能调用，换平台不换上层代码。
class ISerialPort {
public:
    // 虚析构必须显式给出来：用基类指针 delete 派生对象时，没有虚析构，
    // 派生类的析构就不会被执行（资源泄漏 + 未定义行为）。这是新手最容易踩的坑之一。
    virtual ~ISerialPort() {}

    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool isOpen() const = 0;

    // 读取：返回实际读到的字节数（>0）；无数据返回 0；出错返回负数。
    // timeoutMs = 0 表示非阻塞（有多少拿多少）。
    virtual int read(std::uint8_t* buffer, std::size_t size, int timeoutMs) = 0;

    // 写入：返回实际写出的字节数；出错返回负数。
    virtual int write(const std::uint8_t* data, std::size_t size) = 0;

    // 便于日志/报错：如 "COM3"、"/dev/ttyUSB0"、"mock"
    virtual std::string name() const = 0;
};

}  // namespace serial
