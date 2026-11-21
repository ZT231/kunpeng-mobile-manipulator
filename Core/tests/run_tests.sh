#!/bin/sh
# ============================================================================
# run_tests.sh —— 一键编译 + 运行机械臂固件 host 单元测试
#
# 【用法】在项目根 RoboticArm_Kunpeng/ 下执行（Git Bash）：
#   sh Core/tests/run_tests.sh
#
# 【为什么用 sh 脚本而不是 .bat】机械臂固件是 C99，本机 Git Bash 的 gcc
#   直接能编；底盘 car/ 的 test_frame.c 也是同一套路（gcc + 假回调）。
#
# 【红线】stubs/ 里的假头文件只服务本测试，绝不进 Keil 工程。
# ============================================================================
set -e
cd "$(dirname "$0")/../.."    # 回到项目根，保证相对路径一致

mkdir -p Core/tests/build

echo "=== 编译 test_arm_firmware.c ..."
gcc -std=c99 -Wall -Wextra \
    -I Core/tests/stubs -I Core/Inc \
    Core/tests/test_arm_firmware.c \
    -o Core/tests/build/test_arm_firmware.exe

echo "=== 运行 ..."
./Core/tests/build/test_arm_firmware.exe
