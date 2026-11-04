cmake_minimum_required(VERSION 3.16)
project(serial_engine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 全项目 header-only：serial_engine 是 INTERFACE 库，只提供 include 目录与编译选项；
# 唯一的 .cpp 是 src/main.cpp（armctl CLI），作为独立可执行文件构建。
add_library(serial_engine INTERFACE)
target_include_directories(serial_engine INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# 阶段 4 起用到 std::thread/std::condition_variable，Linux 下需要链接 pthread
find_package(Threads REQUIRED)
target_link_libraries(serial_engine INTERFACE Threads::Threads)

if(MSVC)
    target_compile_options(serial_engine INTERFACE /W4 /utf-8)
else()
    target_compile_options(serial_engine INTERFACE -Wall -Wextra)
endif()

enable_testing()

add_executable(test_ring_buffer tests/test_ring_buffer.cpp)
target_link_libraries(test_ring_buffer PRIVATE serial_engine)
add_test(NAME ring_buffer COMMAND test_ring_buffer)

add_executable(test_frame tests/test_frame.cpp)
target_link_libraries(test_frame PRIVATE serial_engine)
add_test(NAME frame COMMAND test_frame)

add_executable(test_frame_parser tests/test_frame_parser.cpp)
target_link_libraries(test_frame_parser PRIVATE serial_engine)
add_test(NAME frame_parser COMMAND test_frame_parser)

add_executable(test_dispatcher tests/test_dispatcher.cpp)
target_link_libraries(test_dispatcher PRIVATE serial_engine)
add_test(NAME dispatcher COMMAND test_dispatcher)

add_executable(test_mock_serial_port tests/test_mock_serial_port.cpp)
target_link_libraries(test_mock_serial_port PRIVATE serial_engine)
add_test(NAME mock_serial_port COMMAND test_mock_serial_port)

add_executable(test_serial_integration tests/test_serial_integration.cpp)
target_link_libraries(test_serial_integration PRIVATE serial_engine)
add_test(NAME serial_integration COMMAND test_serial_integration)

add_executable(test_serial_port_factory tests/test_serial_port_factory.cpp)
target_link_libraries(test_serial_port_factory PRIVATE serial_engine)
add_test(NAME serial_port_factory COMMAND test_serial_port_factory)

add_executable(test_logger tests/test_logger.cpp)
target_link_libraries(test_logger PRIVATE serial_engine)
add_test(NAME logger COMMAND test_logger)

add_executable(test_session tests/test_session.cpp)
target_link_libraries(test_session PRIVATE serial_engine)
add_test(NAME session COMMAND test_session)

add_executable(test_blocking_queue tests/test_blocking_queue.cpp)
target_link_libraries(test_blocking_queue PRIVATE serial_engine)
add_test(NAME blocking_queue COMMAND test_blocking_queue)

add_executable(test_async_receiver tests/test_async_receiver.cpp)
target_link_libraries(test_async_receiver PRIVATE serial_engine)
add_test(NAME async_receiver COMMAND test_async_receiver)

# 上位机 CLI 主程序
add_executable(armctl src/main.cpp)
target_link_libraries(armctl PRIVATE serial_engine)
