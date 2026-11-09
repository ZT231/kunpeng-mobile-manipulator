# ESP-IDF 顶层工程文件
#
# 【概念】CMake 是「构建脚本生成器」：它不直接编译，而是先读懂这份清单，
#        再生成真正给编译器用的 Makefile/Ninja 脚本。
# 【规范】ESP-IDF 要求顶层 CMakeLists.txt 至少做三件事：声明最低版本、
#        include 官方 project.cmake、用 project() 起工程名。
# 【为什么这么写】不 include project.cmake 的话，idf.py 不认识这个目录，
#        直接报 "not an ESP-IDF project"。

cmake_minimum_required(VERSION 3.16)

# $ENV{IDF_PATH} 是环境变量，指向你装好的 esp-idf 目录。
# 用环境变量而不是把路径写死，换电脑 / 换版本都不用改这一行。
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# 工程名会变成固件名：kunpeng_voice.elf / kunpeng_voice.bin
project(kunpeng_voice)
