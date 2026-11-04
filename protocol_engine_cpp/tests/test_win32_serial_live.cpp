// 真·Win32 串口实时测试
// ------------------------------------------------------------------
// 用项目自带的 Win32SerialPort 直接调用 Windows API：
//   打开 COM 口 -> 配置 115200 8N1 -> 写一帧 -> 读回来（回环则原样返回）。
//
// 编译（必须用 MinGW-w64 交叉编译器，它才会定义 _WIN32）：
//   x86_64-w64-mingw32-g++ -std=c++17 -Wall -Wextra -pthread -static -static-libgcc -static-libstdc++ -Iinclude tests/test_win32_serial_live.cpp -o build/test_win32_serial_live.exe
//
// 运行：
//   build\test_win32_serial_live.exe COM3
//
// 说明：
//   - 本机若插了 USB 转串口（CH340/CP210x 等）并短接 TX-RX，会看到真实字节回环；
//   - 若没插硬件，open() 会真正走到 Windows CreateFile 并返回真实错误码
//     （ERROR_FILE_NOT_FOUND 等）—— 这本身就证明 Win32 真后端是活的，
//     跟本机 Cygwin 的 POSIX g++（不定义 _WIN32、createSerialPort 永远返回 nullptr）完全不同。
#include "serial/serial_port_factory.hpp"

#include <cstdio>
#include <cstdint>
#include <string>
#include <memory>
#include <windows.h>

// 把 Windows 错误码翻译成可读文字，便于排查。
static std::string lastErrorMsg(DWORD code) {
    LPSTR buf = nullptr;
    DWORD n = FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM,
                             nullptr, code, 0, reinterpret_cast<LPSTR>(&buf), 0, nullptr);
    std::string msg = (n && buf) ? std::string(buf, n) : "未知错误";
    if (buf) LocalFree(buf);
    return msg;
}

int main(int argc, char** argv) {
    const std::string port = (argc > 1) ? argv[1] : "COM3";

    // createSerialPort 在 _WIN32 下返回真正的 Win32SerialPort（MinGW-w64 交叉编译器即定义 _WIN32）
    auto portObj = serial::createSerialPort(port);
    if (!portObj) {
        std::printf("createSerialPort 返回 nullptr：当前编译器未定义 _WIN32。\n"
                    "请用 x86_64-w64-mingw32-g++ 编译本测试。\n");
        return 1;
    }
    std::printf("[OK] 已创建 Win32 串口对象 name=%s\n", portObj->name().c_str());

    if (!portObj->open()) {
        DWORD err = GetLastError();
        std::printf("[FAIL] open(%s) 失败 —— 但已真正走到 Windows CreateFile 路径。\n"
                    "       真实错误码=%lu: %s\n",
                    port.c_str(), err, lastErrorMsg(err).c_str());
        return 2;  // 走到这里就证明 Win32 真后端是活的（只是本机没插串口硬件）
    }
    std::printf("[OK] open 成功，端口=%s\n", port.c_str());

    // 写一帧原始字节（回环时会被原样读回）
    const unsigned char payload[] = {'$', 'P', 'I', 'N', 'G', ',', '*', '\r', '\n'};
    int w = portObj->write(payload, sizeof(payload));
    std::printf("[INFO] write 返回 %d 字节\n", w);

    unsigned char rx[64] = {0};
    int r = portObj->read(rx, sizeof(rx), 800);
    if (r > 0) {
        std::printf("[OK] read 返回 %d 字节：", r);
        for (int i = 0; i < r; ++i) std::printf("%c", rx[i]);
        std::printf("\n");
    } else {
        std::printf("[INFO] read 返回 %d（无回环数据；若 TX-RX 短接或接了虚拟串口对，会读到写入内容）\n", r);
    }

    portObj->close();
    std::printf("[DONE] 已 close。\n");
    return 0;
}
