# C++ 跨平台串口协议引擎（serial_engine）

> 一句话定位：把机械臂固件里那套用 C 手搓的 `uart_protocol`（双缓冲 / 状态机收帧 / 持锁解析）搬到 PC 端，
> 用现代 C++17 重新实现一遍，做成面向六自由度机械臂的 PC 端上位机（真机联调待硬件就位，见「阶段进度」）。

## 项目定位

- 三个项目的**完整故事链**：STM32 固件（下位机）→ 通信协议（本项目的骨架）→ PC 上位机（本项目的形态）。
- 可深入讨论的三个技术点：**粘包/半包怎么处理**、**应答帧被覆盖怎么办**、**Mock 串口为什么有价值** —— 每层都有真实排障素材（详见《排障记录.md》）。
- 技术要点一句话版：
  《基于 C++17 实现跨平台串口协议引擎：自研 RingBuffer 与状态机帧解析器解决粘包、半包与噪声干扰；
   抽象 ISerialPort 接口支持 Windows/Linux 串口与 Mock 测试；命令分派采用回调注册机制；
   协议与固件 `uart_protocol.c` 逐条对齐（异或校验、命令表、接收/发送缓冲上限）；真机联调待硬件就位。》

## 项目结构

```
3.C++ 跨平台串口协议引擎/
├── README.md              ← 本文件：项目定位、结构、构建、协议速查、进度与已知限制
├── 设计说明.md            ← 各模块的实现要点、设计理由与替代方案（含设计取舍 FAQ）
├── 排障记录.md            ← 踩坑记录（现象→排查→根因→解决）
├── 项目使用说明.md        ← 编译方式、armctl 命令详解、作为库使用的代码示例
├── CMakeLists.txt         ← 用 Qt Creator / CLion 直接打开这个文件
├── build.bat              ← 命令行一键编译 + 跑全部测试（双击即可）
├── build_win32.sh         ← 用 MinGW-w64 交叉编译真机串口测试 + armctl（Git Bash / Cygwin 下运行）
├── .gitignore              ← 忽略 build/、*.exe、*.o、CMake/Qt/编辑器产物
├── .gitattributes          ← 行尾规则：仓库内存 LF；.bat 检出 CRLF（cmd 解析批处理必须 CRLF），.sh 保持 LF
├── include/serial/
│   ├── ring_buffer.hpp           ← 模块 1：环形缓冲区              ✅
│   ├── checksum.hpp              ← 模块 2：异或校验 + 错误码         ✅
│   ├── error_code.hpp
│   ├── frame.hpp                 ← 模块 3：一帧的解析/生成           ✅
│   ├── frame_parser.hpp          ← 模块 4：字节流 → 帧 状态机         ✅
│   ├── command_dispatcher.hpp    ← 模块 5：命令分派 + 命令表          ✅
│   ├── commands.hpp
│   ├── iserial_port.hpp          ← 模块 6：串口抽象接口（虚析构）     ✅
│   ├── mock_serial_port.hpp      ← 模块 7：Mock 串口（不插硬件测）    ✅
│   ├── win32_serial_port.hpp     ← 模块 8：Windows 串口（CreateFile） ✅
│   ├── linux_serial_port.hpp     ←         Linux 串口（termios，未在真实 Linux 验证）⚠️
│   ├── serial_port_factory.hpp   ←         平台工厂（按宏选后端）     ✅
│   ├── logger.hpp                ← 模块 9：日志（Meyers 单例 + 线程安全） ✅
│   ├── session.hpp               ← 模块 10：发请求→收应答会话          ✅
│   ├── blocking_queue.hpp        ← 模块 11：线程安全队列（生产者-消费者）✅
│   └── async_receiver.hpp        ← 模块 12：异步接收线程（std::thread）  ✅
├── src/
│   └── main.cpp                  ← 上位机 CLI：armctl（selftest / --mock / 真机）
├── tests/
│   ├── test_ring_buffer.cpp
│   ├── test_frame.cpp
│   ├── test_frame_parser.cpp
│   ├── test_dispatcher.cpp
│   ├── test_mock_serial_port.cpp
│   ├── test_serial_integration.cpp
│   ├── test_serial_port_factory.cpp
│   ├── test_logger.cpp
│   ├── test_session.cpp
│   ├── test_blocking_queue.cpp
│   ├── test_async_receiver.cpp
│   └── test_win32_serial_live.cpp   ← 真机串口回环测试（需真实 COM 口，用 build_win32.sh 单独编译）
└── build/                 ← 编译产物（已在 .gitignore 中忽略）
```

当前测试：**243 处 `CHECK` 断言全部通过，`-Wall -Wextra` 零警告**；`armctl selftest` 自检通过、`--mock` 演示链路通。

> **断言计数口径**：`tests/` 内匹配 `CHECK(` 的共 254 行，其中 11 行是各测试文件自己的 `#define CHECK` 宏定义，254 − 11 = 243 处调用。
> `test_serial_port_factory` 有 3 处位于 `#if defined(_WIN32) || defined(__linux__)` 分支内，与 `#else` 分支互斥，
> 所以单次构建实际执行的断言数：Cygwin/POSIX 工具链 **240** 项，Windows/Linux 工具链 **242** 项。
> `tests/test_win32_serial_live.cpp` 需要真实 COM 口，不参与 CMake/CTest 与 `build.bat` 的自动跑测。
> 历史备注：曾写「247 项」属统计口径错误；`armctl` 也一度因在 `unique_ptr<ISerialPort>` 上调 Mock 独有的 `inject()` 而编译不过，已修复，详见排障记录第 6 条。

## 构建与运行

**方式一：命令行（推荐日常用）**
双击 `build.bat`（它会自动编译并运行 `tests\*.cpp` 里的每一个单元测试；真机测试 `test_win32_serial_live.cpp` 需要真实 COM 口 + MinGW-w64，脚本里已显式跳过），等价于：

```bat
g++ -std=c++17 -Wall -Wextra -Iinclude tests\test_frame.cpp -o build\test_frame.exe
build\test_frame.exe
```

**方式二：Qt Creator**
用 Qt Creator 打开 `CMakeLists.txt` → 选择 MinGW 或 MSVC 套件 → 直接构建运行（测试用例会注册到 CTest）。

> ⚠️ 中文路径注意事项：Git Bash 下凡是把中文路径**当参数写进命令**都会失败
> （报 `No such file or directory`，连 `ls "中文路径目录"` 都不行）。
> 解决办法有两个：① 用 ASCII 通配符进目录，例如 `cd 3.C++*`；
> ② 直接走 `build.bat`（cmd 对中文路径没有问题），或者用支持 Unicode 路径的编辑器打开。

## 阶段进度

| 阶段 | 内容 | 关键技术点 | 状态 |
|---|---|---|---|
| 1 | 协议核心：`RingBuffer<T>`、异或校验、`Frame`、状态机 `FrameParser`、`CommandDispatcher` + 命令表 | 类模板 / 拷贝控制（深拷贝·三法则）/ explicit / 运算符重载 / std::function 回调 | ✅ 已完成 |
| 2 | 跨平台串口层：`ISerialPort` 抽象 + Windows/Linux 实现 + `MockSerialPort` + 平台工厂 | 纯虚接口 / 虚析构 / unique_ptr 独占所有权 / RAII / 工厂 / 平台宏 | ✅ 已完成（Linux 版本机已用 `-D__linux__` 编译检查零警告，未在真实 Linux 环境验证）|
| 3 | 上位机 CLI：`armctl`（selftest / --mock / 真机命令）、`Logger`、`SerialSession`、连真机联调 | 单例 / 文件操作 / string 解析 / 命令行参数 / 超时循环 | ✅ 代码完成（真机联调待硬件就位）|
| 4 | 加分项：接收线程 + 线程安全队列、Logger/Mock 线程安全化 | std::thread / condition_variable / atomic / mutex | ✅ 已完成 |

---

# 协议速查表（v1，与固件逐条对齐）

> 来源：机械臂固件 `Core/Src/uart_protocol.c`（见 `robotic-arm-vision-stm32` 仓库）。**上位机必须与对端一致，
> 所以这里放弃了原计划的 CRC16，改用固件真正在用的异或校验。**

```
帧格式：  $<CMD>[,<ARG>...]*<CK>\r\n
  CK = CMD 字段（'$' 与 '*' 之间）所有字节逐字节 XOR，2 位大写十六进制
  例：$PING*10\r\n        （'P'^'I'^'N'^'G' = 0x10）

响应：   $OK[,<数据>]*XX\r\n
        $ERR,<code>*XX\r\n
错误码： 1=未知命令  2=校验失败  3=格式/参数错误  4=当前状态禁止

命令表：
  PING                      → OK,PONG
  STATE                     → OK,S,<M|C|R|P|E>,<6 路脉宽>   状态：手动/命令/录制/回放/急停
  MOVE,<ch>,<pulse>         → OK | ERR,3(ch/pulse 非法) | ERR,4(急停)
  JOINT,<6 路脉宽>          → OK | ERR,3 | ERR,4
  HOME / STOP / RESUME / MANUAL → OK | ERR,4
  REC / RECEND / PLAY       → OK（RECEND 返回 OK,<点数>）| ERR,3 | ERR,4
  TNUM                      → OK,<点数>
  CLEAR                     → OK | ERR,4
  LOG,<0|1>                 → OK | ERR,3
固件缓冲上限：接收 64 字节 / 发送 96 字节 → 本引擎 `kMaxContent = 63`
```

**两个已确认的设计决策（对应上面的选择）**
1. **解析从严**：CK 之后还有多余字符、CK 不是十六进制、缺 `*CK`，一律记 `format_errors` 并丢帧。
   固件对这个场景更宽容（只认第一个 `*`，后面的字符不管）；上位机是调试工具，宁可直接暴露问题。
2. **解析器只切字段，不判业务数值**：`FrameParser` 只负责把 `JOINT,1500,...` 切成命令+参数，
   关节个数够不够、脉宽是否 ≤2500 交给命令层判断（对应固件 `ERR,3`）。分层清楚了，加命令不用动解析器。

---

# 延伸阅读

- 《设计说明.md》—— 12 个模块的实现要点、设计理由与替代方案，以及设计取舍 FAQ。
- 《排障记录.md》—— 真实踩坑记录（现象 → 排查 → 根因 → 解决）。
- 《项目使用说明.md》—— 编译方式、armctl 命令详解、作为库使用的代码示例。

---

# 已知限制 / 待办

- [ ] `RingBuffer` 暂不支持非默认构造的 `T`。
- [ ] `RingBuffer` 满时的策略目前是"丢新数据"，后续给 `FrameParser` 增加"丢最旧"的覆盖模式。
- [ ] `FrameParser` 没有行长度上限之外的 DoS 防护（恶意对端持续发 `$` 会不断清缓冲，当前无害但值得写进文档）。
- [ ] `FrameParser` 只在单线程使用：`AsyncReceiver` 由单一工作线程独占解析器，跨线程交接走 `BlockingQueue`（不是 RingBuffer）。
- [ ] `AsyncReceiver` 单次使用（`start()` 只能调一次）；运行中读 `stats()` 是近似值（与工作线程有良性竞态，`stop()` 之后读才准确）。
- [ ] 涉及运动/状态的命令（STATE/HOME/STOP/RESUME/MANUAL/REC/RECEND/PLAY/TNUM/CLEAR）尚未注册，
  需要串口后端（阶段 2）与机械臂状态模型（尚未单独成阶段；`STATE` 要回当前状态和 6 路脉宽，`REC`/`PLAY` 要状态机）。
- [ ] `Frame::parse` 的严格度已确认（从严），若将来要与固件宽容策略完全一致，需要一个 `parseLenient` 变体。
- [ ] Linux 串口后端（termios）已写完源码，本机已用 `-D__linux__` 做过编译检查（零警告），
  但未在真实 Linux 环境（真机 / 虚拟机 / WSL）编译运行过，需在 Linux 上 `g++ -Iinclude -c` 复核。
- [ ] 平台宏依赖 `_WIN32`/`__linux__`；Cygwin/POSIX 的 g++ 不定义 `_WIN32`，在该工具链下只有 Mock 可用（真实 MinGW-w64 / MSVC / GCC on Linux 都正常定义，不受影响）。
- [ ] `armctl` 的 `--baud` 参数目前只被解析、还没喂给 `SerialConfig`（真实串口默认 115200 8N1，够用，后续把 baud 传进 config）。
- [ ] 尚未与真机联调（阶段 3 的验收标准，需要机械臂 + 串口线就位）。
