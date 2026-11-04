#pragma once

#include "serial/iserial_port.hpp"
#include <memory>

// 平台分发：按编译目标选择真实串口后端。
// 这样上层只需要一句 createSerialPort("COM3")，不用自己写 #ifdef。

// 只有在对应平台才把平台头文件拉进来，其它平台完全看不见这些代码（避免编译失败）。
#ifdef _WIN32
#include "serial/win32_serial_port.hpp"
#elif defined(__linux__)
#include "serial/linux_serial_port.hpp"
#endif

namespace serial {

// 创建一个匹配当前平台的真实串口，返回 unique_ptr（独占所有权的智能指针）。
//
// 为什么用 unique_ptr 而不是裸指针？
//   裸指针只给地址、不管释放——谁 new 的谁得记得 delete，忘了就内存泄漏，
//   中途 return 更容易漏。unique_ptr 把「释放」交给编译器：它独占这个对象，
//   离开作用域（或 move 给别的对象）时自动 delete，这就是 RAII（资源获取即初始化）。
//   这是现代 C++ 管理「一个资源归一个所有者」的标准做法，比裸指针更安全、也不难懂。
inline std::unique_ptr<ISerialPort> createSerialPort(const std::string& portName) {
#ifdef _WIN32
    return std::make_unique<Win32SerialPort>(portName);     // Windows：用 CreateFile 那套
#elif defined(__linux__)
    return std::make_unique<LinuxSerialPort>(portName);     // Linux：用 open/termios 那套
#else
    // 未知平台（比如本机 Cygwin 的 POSIX g++ 不定义 _WIN32）：没有真实后端，
    // 只能返回 nullptr。但 MockSerialPort 任何平台都能用。
    (void)portName;
    return nullptr;
#endif
}

}  // namespace serial
