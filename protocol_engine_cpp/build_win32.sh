#!/bin/sh
# 用 MinGW-w64 交叉编译器编译 Win32 真后端（它会定义 _WIN32，产出原生 Windows exe）
# 用法（Git Bash 或 Cygwin64 Terminal，先 cd 到本项目目录）：
#   sh build_win32.sh
# 说明：本机默认的 g++ 是 Cygwin/POSIX 版（不定义 _WIN32），会让 Win32 后端变死代码；
#       这个脚本改用 x86_64-w64-mingw32-g++（已装），并用 -static 静态链接避免缺 MinGW DLL。
set -e
CC=x86_64-w64-mingw32-g++
CFLAGS="-std=c++17 -Wall -Wextra -pthread -static -static-libgcc -static-libstdc++"
mkdir -p build
$CC $CFLAGS -Iinclude tests/test_win32_serial_live.cpp -o build/test_win32_serial_live.exe
$CC $CFLAGS -Iinclude src/main.cpp -o build/armctl.exe
echo "build OK -> build/test_win32_serial_live.exe  build/armctl.exe"
echo "运行真实串口测试（把 COM3 换成你机器上真实的串口号，USB 转串口 TX-RX 短接即可回环）："
echo "  ./build/test_win32_serial_live.exe COM3"
