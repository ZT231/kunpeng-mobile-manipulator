# 排障记录：固件与 ESP32

> 本文由 2026-11-21 的多份固件 / ESP32 草稿合并整理而成（内容未改写，仅重组与订正；源文件已删除，以本文为现行版本）。
>
> 硬件口径以《硬件清单与接线》与《00-项目总览与硬数据速查》为准。

> 🔴 **历史记录提醒（2026-11-24）**：本文第 1 条等段落描述的是**旧机械臂/六舵机/PCA9685 固件**的排障过程；这些代码已从当前工程删除，当前固件只保留四轮底盘 + 三路 UART + 总线舵机骨架。不要按本文的 `Arm_Init()`、`$JOINT`、`PCA9685` 步骤操作现行板子。

## 目录

1. 固件与串口
2. ESP32-S3 语音入口骨架
3. S3b 鲲鹏端视觉服务与网页控制台

---

## 一、固件与串口

> 来源：`排障记录-固件与ESP32.md`

> 只记关键问题（面试官会追问的那类）。每条固定五段：**现象 → 排查过程 → 根因 → 解决 → 一句话面试说辞**。
> 概念一律「大白话 + 规范定义」双行。

---

### 第 1 条　机械臂没复位、也没让人动，PING 却永久无应答 —— 一次 ORE 把 USART1 接收中断彻底关死

#### 一、现象

- 上位机 `POST /api/cmd {"payload":"PING"}` 返回 `{"ok":true,"resp":""}`（或超时），**一个字都收不到**。
- 同一条串口 **TX 完全正常**：`printf` 的日志照样往外打，`STATE` 遥测照样出。
- 机械臂 **没有甩回折叠初始位**（这是判「STM32 有没有复位」的铁证：`Arm_Init()` 会无条件把六路写成 `servo_init[]`，只要正常复位必然甩臂）。
- 表现是**永久性**的：不拔电重启，怎么重开串口、怎么重发命令都没用。
- 触发时机不固定，但常出现在 CH341 被 USB 复位（`dmesg` 里 `usb usb5-port1: disabled by hub (EMI?), re-enabling...`）之后。

#### 二、排查过程

1. **先分清「谁死了」**：`POST /api/close` 让 Flask 放串口 → 裸开串口发 `$LOG,1*CK` → 5 秒内每 3 秒一行 `STATE n | PULSE ...`。
   - 有 `STATE` ⇒ **STM32 活着**，问题只在接收方向（TX 好、RX 死）。
   - 反复打 `Robot arm ready.` ⇒ STM32 在重启循环（供电问题）。
   - `PCA9685 init failed` ⇒ 卡在 `Error_Handler()`（I2C 老坑）。
   - 一个字节都没有 ⇒ 没上电或 TX/RX 断线。
2. **读芯片手册路径**：接收是 `USART1_IRQHandler` → `HAL_UART_IRQHandler(&huart1)` 单字节中断模式。
3. **读 HAL 源码**（`Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c`）：
   - `:2398` 检测到 `USART_SR_ORE` 且 `RXNEIE`/`EIE` 使能 ⇒ `ErrorCode |= HAL_UART_ERROR_ORE`（`:2401`）；
   - `:2416` 判定为 blocking ⇒ 调 `UART_EndRxTransfer()`（`:2421`）；
   - `UART_EndRxTransfer()` 里 `:3373` 清 `CR1` 的 `RXNEIE|PEIE`、`:3374` 清 `CR3` 的 `EIE`、`:3383` 把 `RxState` 置 `READY`；
   - 随后调 `HAL_UART_ErrorCallback()`（`:2460`），而它是 `__weak` 空函数（`:2679`）。
4. **查工程里有没有人接住这个回调**：`grep -rn "HAL_UART_ErrorCallback" Core/` ⇒ **0 结果**。
5. **查工程自带的重挂兜底为什么救不了**：`UartProto_Process()`（`uart_protocol.c:652-661`）确实会重挂，但前提是 `rx_restart_pending[idx]` 置位；该标志只在 `HAL_UART_RxCpltCallback`（`:765`）里置 —— **接收中断已关，RxCplt 永远不会再来**，三条路同时断。
6. **用旧构建的 `.map` 反证**（最硬的一步）：`HAL_UART_ErrorCallback` 落在 `stm32f4xx_hal_uart.o`，**Thumb Code 只有 2 字节**（空函数就一条 `BX LR`）；对照 `HAL_UART_RxCpltCallback` 落在 `uart_protocol.o` ⇒ 同一份构建里，一个 weak 被覆盖了，另一个没有。

#### 三、根因

**HAL 的「溢出错误」处理路径会主动关掉接收中断，然后把控制权交给一个默认什么也不做的空回调。**

- 大白话：串口收得太慢，硬件缓冲区（1 字节的 DR）里上一字节还没被读走，下一字节就来了，这叫「溢出」。HAL 遇到溢出干脆把「有新数据」的中断开关**关掉**，指望用户写回调把它打开；用户没写，这个开关就永远关着 —— CPU 还在跑、PWM 还在输出、串口还能发，就是**再也收不到一个字节**。
- 规范定义：`ORE`（OverRun Error，溢出错误）是 USART 的 `SR` 寄存器标志位，表示 `DR` 中数据未被读走时移位寄存器又收到完整一帧；STM32F4 HAL 将其归类为 blocking error，通过 `UART_EndRxTransfer()` 终止接收（清 `RXNEIE`/`PEIE`/`EIE`、`RxState=READY`），并依赖用户实现 `HAL_UART_ErrorCallback()` 重新调用 `HAL_UART_Receive_IT()` 恢复。

#### 四、解决

在 `Core/Src/uart_protocol.c` 末尾实现强符号 `HAL_UART_ErrorCallback()`（覆盖 `__weak`），要点三条：

| 要点 | 为什么 |
|---|---|
| 先用 `__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)` 判硬件位，**真的挂着才** `__HAL_UART_CLEAR_OREFLAG` | 该宏是「读 SR + 读 DR」，读 DR 会取走字节。若 ORE 同时伴随 `RXNE`，HAL 在 `:2410` 已读过 DR、硬件位已被清，再读一次会**吃掉刚到达的下一个字节** |
| 只在 `huart->RxState == HAL_UART_STATE_READY` 时重挂 | NE/FE/PE 走 non-blocking 分支，中断位没关、`RxState` 仍是 `BUSY_RX`，无需（也不该）重挂 |
| 重挂失败置 `rx_restart_pending[idx]` | 交给 `UartProto_Process()`（**每 5 ms 一轮**）兜底重试 |

验证前提（都已核对）：`USE_HAL_UART_REGISTER_CALLBACKS = 0`（`stm32f4xx_hal_conf.h:193`）⇒ HAL 直接调 weak 符号，强符号覆盖有效；`HAL_UART_Receive_IT()` **不取 `__HAL_LOCK`、只判 `RxState`**（`:1350`）⇒ 与 `printf` 的阻塞发送无冲突。

#### 五、一句话面试说辞

> 「STM32 HAL 里 ORE 溢出会走 blocking 分支，`UART_EndRxTransfer()` 把 `RXNEIE` 关掉，然后调一个默认空的 `HAL_UART_ErrorCallback`。工程里没实现这个回调，也没有任何代码会重新打开接收中断，所以一次溢出就让那路串口永久失聪——表现是能发不能收、CPU 正常、PING 没回应。我在回调里清 ORE 标志再重新挂 `HAL_UART_Receive_IT`，并且特意先判硬件标志位，避免清标志时把刚到的字节读掉。」

---

### 本文件诚实边界

- ✅ 实机复现过的：CH341 被 USB 复位、`ttyUSB` 翻号、串口读线程静默死亡。
- ✅ 源码级确证的：上述 ORE 路径、`.map` 里 2 字节空回调、工程内无任何恢复路径。
- 🔶 **未实机验证**：补丁的「溢出后自愈」行为尚未在板上触发过（需要人为制造一次 ORE 才能看到。补丁生效后若真发生，串口不会再失聪）。**不要对外说「已验证自愈」**。

---

### 阶段 1 安全改造：FAULT 状态机 + 过流急停（2026-11-19）

#### 一、现象（要解决的隐患）

审阅固件源码确认：状态机只有 5 态、无「故障锁存」态；急停（ESTOP）与故障未分离——过流这类需要人工诊断后才能复位的异常，理论上可以被 `$RESUME` 一句话解除。同时 ACS712 已采样 J6 电流，但只用于 `$CUR` 查询，未接入任何自动保护：上位机/手机失联或舵机堵转时，固件不会主动停机。

#### 二、排查过程 / 关键决策

1. **ACS712 采样会阻塞 20ms**（`acs712.c` 里 `ACS712_Read_mA()` 为跨 PWM 周期每点 `HAL_Delay(1ms)` ×20 点）。若把过流检测塞进 20ms 的 `StartServoTask`，舵机刷新被拖慢一倍 → 决定放独立低优先级 `StartSafetyTask`（200ms 周期），20ms 阻塞对舵机零影响。
2. **过流阈值与夹持判据冲突**：动态夹持是 Python 侧读 `$CUR`、电流抬升约 +140mA 判「夹住」。若固件过流阈值设太低，会把正常夹持误判成过流打断抓取。定 800mA（远高于夹持抬升、低于堵转 1.5~2.5A），并加「连续 3 次超阈值」去抖，滤掉启动尖峰。
3. **IWDG 看门狗延后**：硬件看门狗一旦初始化软件无法关闭，喂狗点/超时配错会反复复位成砖，且本地无 Keil 无法编译验证 → 按用户决策延后，等真机跑稳再上。
4. **断链保护延后**：机械臂是「目标位驱动、到位即停」，失联只会停在最后目标位不乱动；真正「失联会撞」的是小车底盘，等底盘接入再做。

#### 三、根因 / 四、解决（合并）

状态机把「可恢复的安全停止」与「需诊断复位的故障」混为一体。解决：新增 `ARM_STATE_FAULT` + `fault_code` 锁存，`Arm_ReportFault(code)` 进故障态、`Arm_ClearFault()` 显式清除（`$CLEARFAULT` 命令），`$RESUME` 在 FAULT 态显式拒绝（`ERR,5`）。关键是把「急停或故障都冻结」收敛成 `state_is_frozen()` 单函数，协议层用 `Arm_IsFrozen()` 判冻结——避免各处写 `==ESTOP || ==FAULT` 漏一处。STATE 响应末尾追加 fault 字段，向后兼容老解析。

#### 五、一句话面试说辞

> 「原来的状态机急停和故障不分，过流这种需要人工诊断的异常会被一条 RESUME 带过去，等于没有保护。我新增了 FAULT 锁存态和故障码，把『可恢复的急停』和『需诊断的故障』分开，急停按一下能继续、故障必须显式清除才能复位。过流检测单独放了一个低优先级任务，因为电流采样要跨一个 PWM 周期、会阻塞 20ms，放舵机任务里会拖慢刷新。阈值定 800mA、连续三次超阈值才触发，既拦住堵转又不误伤正常夹持的电流抬升。」

> ⚠️ 诚实边界：本阶段代码**未上板编译验证**（本地无 Keil），IWDG 与断链保护按决策延后。

---

## 二、ESP32-S3 语音入口骨架

> 来源：`排障记录-固件与ESP32.md`

> 固定五段式：**现象 → 排查过程 → 根因 → 解决 → 一句话面试说辞**。
> 只记重要 / 关键 / 面试官关心的问题，鸡毛蒜皮不记。
> 本文件是个人复习材料，**不进 Git**（`docs/排障记录-*.md` 一律不 add）。

---

### 第 1 条　本机无 ESP-IDF 工具链，整机骨架编译不了，却差点把「落盘」说成「跑通」

**现象**
B 阶段要「上板建 ESP-IDF 骨架 + Wi-Fi + I2S 采音」，但本机检查回执是：
`gcc (GCC) 9.3.0` 有，`command -v idf.py` 无输出，`~/.espressif` 不存在，`esp32/` 里只有 `README.md`。
也就是说本机只能编纯标准 C，编不了依赖 `driver/i2s_std.h`、`esp_wifi.h` 的 ESP32 工程。

**排查过程**
1. 先分清「哪些文件依赖 ESP-IDF 组件」：
   `main.c` / `i2s_mic.c` / `wifi_ap.c` / `main/CMakeLists.txt` 都依赖官方组件头与构建系统；
   只有 `protocol.c` / `protocol.h` 只 include `<stddef.h>` `<stdint.h>`，是纯标准 C。
2. 于是把验证拆成两层：能在本机做的（纯 C 封帧算法）与必须去装有 ESP-IDF 的机器做的（整机 build / flash）。
3. 用本机 gcc 把 `protocol.c` 单独编译成一个自测 exe，跑 S2 实机锚点帧反查。

**根因**
ESP-IDF 是一整套交叉编译工具链（Xtensa 编译器 + 组件库 + `idf.py` 构建脚本），本机只装了宿主机 gcc，
两者目标架构不同：宿主机 gcc 出 x86 代码，ESP32-S3 要 Xtensa 代码。
把「文件已落盘」和「工程已编译烧录」混为一谈，就是简历上最容易被面试官一问就露馅的越线。

**解决**
- 骨架照常落盘（当时 10 个文件，11-14 晚补 `tcp_client.c/.h` 后为 12 个，11-21 再补 `audio_tx.c/.h` 现为 14 个）；`esp32/README.md` 第十节**当时**明确写「整机 `idf.py build` 未跑、未烧录」，命令只在 ESP-IDF 环境里执行（该节 11-14 17:38 已按「PC 已装 v5.5.5」订正、11-14 晚又按「整机 build 已通过」订正，见本条文末与第 5 条）。
- 能验证的那部分真的验证：`x86_64-w64-mingw32-gcc -static` 编译 `protocol.c` + 自测 main，跑出
  `$PING*10`（CK=`10`）、`$OK,PONG*3E`（CK=`3E`）、`$ASR,mic_ok*50`（CK=`50`）、中文 UTF-8 三字节逐字节 XOR=`FF`，`EXIT=0`。
  这与 STM32 `Core/Src/uart_protocol.c`、Python `kunpeng/llm/text_gateway.py` 三处一致。
- 软件路线表把步骤 1 标成「骨架已落盘 / 整机 build 未跑」，诚实边界表里补一行「能说 / 不能说」。

**一句话面试说辞**
「我先分清了宿主机编译器和交叉编译工具链的边界：ESP-IDF 工程在本机只能落骨架、不能整机编译，所以我把不依赖芯片 SDK 的协议层单独抽成纯标准 C，用宿主机 gcc 拿项目实机跑通的 `$PING*10` 反查校验算法，剩下的 build/flash 明确留给 ESP-IDF 专用环境，绝不把『代码已写』写成『板子已跑』。」

> ⚠️ **11-14 17:38 订正（撤回本条里的过时事实描述）**：本条写的「本机无 `idf.py` / 本机只装了宿主机 gcc / 要去另一台装好 ESP-IDF 的机器」是**当时在 Git Bash 里看到的历史现象**，不是「没装」。
> 现已查明 **PC 上早已装好完整 ESP-IDF v5.5.5**（`D:\ESP32\Espressif`，EIM / Espressif-IDE 风格安装，纯 ASCII 路径合规），启动脚本 `Initialize-Idf.ps1` / `idf_cmd_init.bat` / `export.ps1` / `export.bat` 齐全。
> 当时看不到 `idf.py` 只是因为 **Git Bash 的 PATH 里没有 ESP-IDF 工具链**；且本轮实测 **Git Bash 里跑 `idf.py` 会被官方直接拒 `MSys/Mingw is no longer supported`** ⇒ 必须从 **ESP-IDF PowerShell / cmd 快捷方式**启动。
> ✅ **结论订正（11-14 晚）**：骨架整机 `idf.py build` **已在 PC 上跑通并回读确认**（`BUILD_RC=0` / `Project build complete` / 产物 4 件齐全，见第 5 条与 `docs/固件与ESP32开发.md` 第一节）；🔴 **11-21 再订正：已烧录实机（TCP 9000/采音通过）**，语音入口整体仍 🔶（识别未接）。「宿主机 gcc ≠ 交叉工具链」这条边界判断仍成立、`protocol.c` 三处一致的实证仍有效。
> 🔴 今后一律写「PC 已装 v5.5.5，须从 ESP-IDF PowerShell 快捷方式启动」，**不再写「本机没装 / 要先装 ESP-IDF」**；ESP-IDF 装在 PC（USB 烧录要摸到板载 CH343），**不装在鲲鹏板**（鲲鹏只跑 `web_control.py` / `voice_gateway.py` 等运行时）。详见 `docs/固件与ESP32开发.md` 第零节。

---

### 第 2 条　Cygwin gcc 编出的 exe 在 `/tmp` 下报 `No such file or directory`

**现象**
第一次用本机 `gcc` 把 `protocol.c` 编到 `/tmp/tp.exe`，执行 `/tmp/tp.exe` 报
`bash: /tmp/tp.exe: No such file or directory`，`Exit 127`；gcc 本身没报错，说明编译是成功的。

**排查过程**
1. 换 `x86_64-w64-mingw32-gcc -static` 编到同一个 `/tmp/tp.exe`，仍然 `No such file or directory` ⇒ 不是单一编译器的问题，是**可执行文件所在位置 / 运行方式**的问题。
2. 回忆本机既有结论：Cygwin 版 gcc 的产物依赖 `cygwin1.dll`，放到 Git Bash 的 `/tmp` 视图里 exec 会解析失败；
   编到 Windows 文件系统上的项目目录（`build/` 一类）就能正常跑。
3. 把输出路径改成工程目录内的相对路径 `./_tp.exe`，用 `./_tp.exe` 运行 ⇒ 一次通过。

**根因**
【概念（大白话）】Cygwin 是「在 Windows 上模拟一层 Linux」的运行库，用它编出来的程序要带一个 `cygwin1.dll` 才能跑；
【规范定义】Cygwin 提供 POSIX API 兼容层，动态链接 `cygwin1.dll`；Git Bash 的 `/tmp` 是 Cygwin 内部路径视图，
与 Windows 进程的 DLL 搜索路径不一致，导致 `execve` 阶段解析失败，表现为误导性的 `ENOENT`。
MinGW-w64 则走另一条路：直接链接 `msvcrt`，产出**原生 Windows PE**，不依赖任何 Cygwin DLL。

**解决**
- 临时 exe 输出到工程目录内（`./_tp.exe`）、用相对路径运行，跑完 `rm -f ./_tp.exe` 清理，不在仓库留垃圾。
- 交叉编译器改用 `x86_64-w64-mingw32-gcc -static`，产出真·Windows 原生 exe，彻底绕开 DLL 依赖。
- 最终自测输出：`ck(PING)=10 len=10 $PING*10`、`ck(OK,PONG)=3E len=13 $OK,PONG*3E`、
  `ck(ASR,mic_ok)=50 len=16 $ASR,mic_ok*50`、`ck(UTF-8 3 bytes)=FF`、`EXIT=0`。

**一句话面试说辞**
「报错是 `No such file or directory` 但文件其实存在，我没急着改代码，而是先判断这是链接/装载问题不是逻辑问题：Cygwin 产物依赖 cygwin1.dll，在 Git Bash 的 /tmp 视图里解析不到；换成 MinGW-w64 静态链接出原生 PE、并把 exe 落到 Windows 文件系统路径上跑，问题就消失了。」

---

### 第 3 条　PC 上明明装了 ESP-IDF v5.5.5，却「跑不起来」整机 build（11-14 晚）

**现象**
今晚要按项目规矩「写完代码真编译跑一遍」，但在本环境的 Git Bash 里：
`idf.py --version` 直接回 `MSys/Mingw is no longer supported`；绕开它裸调 ESP-IDF 自带 Python，又报缺 `IDF_PYTHON_ENV_PATH`；
用 PowerShell 工具起命令则**不回传 stdout、写盘结果对 Bash 也不可见**，没法作为可见的 build 通道。
一度看起来像「工具链没装 / 编译不了」。

**排查过程**
1. 先查 `D:\ESP32\Espressif`：`esp_idf.json`（`version: 5.5`）、`Initialize-Idf.ps1`、`idf_cmd_init.bat`、
   `frameworks/esp-idf-v5.5.5`、自带 Python venv 全在 ⇒ **工具链是好的，错的是启动通道**。
2. 读 `idf.py` / `idf_tools.py`，拒绝点在 `idf.py:929` / `idf_tools.py:3613`：它检查环境变量 `MSYSTEM`。
   Git Bash（MSYS2）会注入 `MSYSTEM` / `MINGW_*` / `MSYS` / `CONFIG_SITE`，官方明确不支持 MSys/Mingw。
3. 用 Python `subprocess(env=...)` 构造干净 env、删掉上述变量后重试，**仍失败**，报
   `"bash.exe" shell is not among the supported options` ⇒ 说明 `activate_venv.py` 是**按父进程名**判 shell，
   只改 `SHELL` 变量没用。
4. `Grep` 找到 `activate.py` 的 `--shell` 参数 ⇒ 改成显式
   `python.exe -B activate.py --export --shell bash`，导出成功（`EXPORT_RC=0`）。

**根因**
【概念（大白话）】ESP-IDF 是「一整套交叉编译工具箱」，启动脚本要先给自己配好 PATH 和编译器位置；
Git Bash 是「在 Windows 上模拟一层 Linux」，两者环境变量打架，工具箱直接拒绝开工。
【规范定义】ESP-IDF 官方仅支持 `cmd.exe` / PowerShell / 原生 POSIX shell；MSYS2 注入的 `MSYSTEM` 被
`idf_tools.py` 判为不支持平台；`activate.py --export` 生成的临时脚本以 `export KEY="VALUE"` 形式给出
`IDF_PATH` / `IDF_PYTHON_ENV_PATH` / `PATH` / `OPENOCD_SCRIPTS` 等变量，需由调用方解析并注入 `env`。

**解决**
写 `D:\esp\_idf_build.py`：用 ESP-IDF 自带 Python（`python_env/idf5.5_py3.11_env/Scripts/python.exe`）
以干净 env 依次跑 ① `activate.py --export --shell bash` → 解析导出脚本里的 `export` 行合并进 env
（`PATH` 里的 `%PATH%` 用当前 PATH 替换）→ ② `idf.py --version`（`ESP-IDF v5.5.5`，rc=0）
→ ③ `idf.py set-target esp32s3`（**rc=0**，cmake 配置完成 145.7s、`Build files have been written to`）
→ ④ `idf.py build`。工程先拷到纯 ASCII 副本 `D:\esp\s3_voice`（官方不支持中文路径）。
**BUILD 已回读确认通过**（`BUILD_RC=0` / `FINAL_RC=0` / `Project build complete` / 产物 4 件齐全；首次全量编译超时后续跑的经过见第 5 条）。🔴 编译通过 ≠ 实机，档位仍 🔶。
⚠️ 本条只解决「本机交叉编译通道」；**烧录仍必须开 ESP-IDF PowerShell 快捷方式**（USB 要摸到板载 CH343）。

**一句话面试说辞**
「报错看似是工具链没装，我先核实安装目录和版本文件，确认是启动通道问题：Git Bash 注入的 `MSYSTEM` 被 ESP-IDF 官方判为不支持平台。
于是用安装自带的 Python 构造干净环境、显式 `--shell bash` 导出环境变量，再依次跑 version / set-target / build，把『代码写了但没编译』这个诚实边界真正补上——但编译通过不等于烧录成功，这一步我仍然明确留到上板。」

---

### 第 4 条　帧「格式」逐字节都对，接上真网关却一律 `$ERR,2`（11-14 晚）

**现象**
本机把 ESP32 `protocol.c` 用 gcc 编出来，三条帧与鲲鹏侧 Python 逐字节一致（含 UTF-8 中文，CK 全对）。
但把骨架原来打印的自检帧 `$ASR,mic_ok*50` / `$ASR,home*63` 喂给**真** `voice_gateway` TCP server，
两条都回 **`$ERR,2*5B`**（未识别），看起来像「校验/编码又错了」。

**排查过程**
1. 先分清「语法层」和「语义层」：帧格式、XOR、UTF-8 字节都没问题（本机 harness 24 项 PASS 已证），
   所以不是编码问题，是**载荷内容**问题。
2. 查 `kunpeng/llm/intent_parser.py` 的 `ACTION_KEYWORDS`：HOME 词表只有
   `回初始位 / 回到初始 / 归位 / 复位 / 收起`，**没有英文 `home`**；`mic_ok` 根本不在任何词表里。
3. 换 `$ESTOP*5D` / `$RESUME*19`（`special_text()` 纯 ASCII 快速通道）⇒ 立刻 `$OK,ESTOP*75` / `$OK,RESUMED*75`；
   换中文 `$ASR,回初始位*61` ⇒ `$OK,HOME*27`。证明链路与校验都好，只是骨架自检帧选错了词。

**根因**
【概念（大白话）】「暗号格式对」和「暗号内容对方认得」是两件事；前者是字节，后者是词表。
【规范定义】`voice_gateway.handle_frame()` 先 `parse_frame()` 校验帧语法，再 `parse_intent()` 做受限词表匹配，
最后 `arbitrate()` 白名单授权；`ERR,2` 专指「意图未识别」，与 `ERR,1`（帧格式错）严格分开。

**解决**
- `main.c` 自检帧改成 `ESTOP` / `RESUME` 两条纯 ASCII 快速通道 ⇒ **先用不依赖中文编码的帧验「链路通不通」，再验「语义对不对」**，两个问题分开定位。
- 中文帧 `$ASR,回初始位*61` 用 C99 八进制 `\ooo` 字节串写死（与板端 `printf` 兜底、Python `.encode("utf-8")` 三方对齐），
  由 `#define SEND_CN_HOME_FRAME 0` **默认关闭**：它在网关 `--apply` 下会真下发 `HOME` 让臂回初始位，
  明天确认「网关已 `--apply` + 臂已摆 C 姿态 + 周围无人」再改 `1` 重烧。
- 完整实证与分流表 → `docs/固件与ESP32开发.md` 3.3、`docs/固件与ESP32开发.md` 第七节。

**一句话面试说辞**
「跨端联调我先把『帧格式逐字节一致』和『载荷语义被识别』拆成两层验证：XOR/UTF-8 用本机 harness 钉死，
再把原来写死的英文自检帧换成纯 ASCII 快速通道先验链路，中文帧单独留安全闸门。`ERR,2` 是词表不匹配不是编码错，
这样定位一次就准，也避免在没确认现场安全时误触发机械臂动作。」

---

### 第 5 条　首次全量编译 20 分钟超时，被误读成「编译失败」（11-14 晚）

**现象**
按第 3 条的干净 env 通道跑 `idf.py build`，脚本给单步设了 `timeout=1200`（20 分钟）。
首次全量编译 ESP-IDF v5.5.5 组件超过 20 分钟，脚本输出 `BUILD_TIMEOUT` / `FINAL_RC=6`，
`idf_build.log` 停在 `===== BUILD =====` 没有回写结论，看起来像「编译挂了 / 代码有错」。

**排查过程**
1. 先读 `build/` 目录：Ninja 已推进到 `[857/1082]`，`.obj` 仍在持续生成 ⇒ **不是编译报错，是被外层 timeout 掐断**。
2. 核对 `_idf_build.py`：超时是脚本自己设的 `subprocess.run(timeout=1200)`，与 ESP-IDF / Ninja 无关。
3. 不删 `build/`、不重跑 `set-target`，另写不设 timeout 的增量脚本只重跑 `idf.py build`，让 Ninja 跳过已完成目标。

**根因**
【概念（大白话）】第一次编译要把整套 ESP-IDF 组件从源码编成 `.a`，耗时远超单个应用文件；
外层脚本若给整步套固定超时，就会在「还在正常干活」时把进程判死。
【规范定义】Ninja 是增量构建系统，以目标文件时间戳判定是否需要重编；
中断后 `build/` 内已完成目标仍有效，重跑 `ninja all` 只补剩余目标与链接，无需清理重建。

**解决**
- 写 `D:\esp\_idf_build2.py`：不设 timeout，重新导出环境后只跑 `idf.py build`；
  增量 `[1/96] → [96/96]`，`[75/96]` 编到我新加的 `tcp_client.c.obj`，链接 `kunpeng_voice.elf` 成功。
- 日志 `idf_build2.log` 末尾确认 `BUILD (rc=0)` / `BUILD_RC=0` / `FINAL_RC=0` / `Project build complete`；
  产物已核实落盘：`kunpeng_voice.bin` 803648 B、`kunpeng_voice.elf` 9035736 B、
  `bootloader.bin` 20832 B、`partition-table.bin` 3072 B、`flash_args` 155 B。
- `REQUIRES lwip` 生效：链接期**没有** `undefined reference to getaddrinfo`（BSD socket 由 lwIP 提供）。
- 🔴 编译通过只把状态推进到「🔶 有代码、本机交叉编译通过」，**仍未烧录 / 未上板 / 未联调**，档位不变。（✅ 11-21 订正：已烧录实机并 TCP 9000/采音通过，见第 6~8 条）

**一句话面试说辞**
「超时报错我先去看构建目录有没有在推进，发现 Ninja 已到 857/1082，判断是外层脚本的固定超时而非编译错误；
删掉超时、复用增量构建续跑即拿到 `Project build complete`。全程没改一行业务代码，也没有把『编译通过』写成『板子跑通』。」

---

> 11-21 已上板实测并逐条记录如下（第 6、7 条），档位已按实测抬到「SoftAP / TCP 9000 / 采音 ✅」，语音识别仍未接。

---

### 6 I2S 每秒只读 256 采样（采音节奏缺陷）

**现象**：烧录后 `main.c` 每秒只打印 `256 个采样: 0 0 0 0`（接麦前）；接麦后变为非 0（±2000~±31000），证实 INMP441 出数。但 16kHz 下每秒产生 16000 采样，旧循环每秒只读 256（1.6%），DMA 环形缓冲被不断覆盖，只能验活、不能录音。

**根因**：`i2s_read` 调用频率与采样率不匹配、读取粒度太小。

**解决**：新增 `audio_tx.c/.h`，独立 FreeRTOS 任务（core 1、prio 5、stack 5120）连续读 I2S0，`raw[i]>>8` 还原 int16，每 256 采样（512B）经 TCP 9001 持久连接上行；非阻塞 `connect`+`select` 限 3s，重连间隔 1s，每 64 读（≈1s）打一行心跳。鲲鹏侧 `audio_sink.py` 落 WAV。

---

### 7 TCP 9000 首次 connect 报 errno=113（EHOSTUNREACH）卡 ~60s

**现象**：ESP32 串口 `connect(192.168.4.100:9000) 失败: errno=113`，两帧各卡约 60s。

**根因**：鲲鹏尚未接入热点（`nmcli con up esp-ap` 前）目标不可达；lwIP `connect()` 不受 3s `SO_SNDTIMEO` 管，SYN 重传约 60s 才报 113。

**解决**：鲲鹏 `nmcli con up esp-ap` 切到 `Kunpeng-Robot` 后复位 ESP32，日志出现 `wifi:station: 28:f5:2b:a7:bd:54 join` → `TCP connected -> 192.168.4.100:9000` → `$ESTOP*5D` 回 `$OK,ESTOP*75`、`$RESUME*19` 回 `$OK,RESUMED*75`，全程约 1s。四层硬判据（SoftAP→STA→TCP9000→网关应答）全通过。另修 firewalld：放行 9000/tcp（原只 5000/8080）。

> ⚠️ 本节第 6、7 条的 IP 是**迁移前旧网段** `192.168.4.x` 的实测记录，保留作历史证据；现网段已改为唯一网段 **`192.168.77.0/24`**（AP `192.168.77.1`、鲲鹏 `192.168.77.100`），原因见第 8 条。

---

### 8 教室路由器与 ESP32 热点同网段 192.168.4.x（撞网，时通时不通）

**现象**：PC 同时连教室网线（以太网 `192.168.4.15`，网关 MAC `7c-b5-9b-0b-d5-60`）和 ESP32 热点（WLAN `192.168.4.2`，网关 MAC `98-c3-77-b2-ce-c1`）。去 `192.168.4.100` 的包被以太网抢走 → 超时；同一目标连发几次表现不一（第一包“无法访问目标主机”、第二包通）。

**根因**：两张网卡同在 `192.168.4.0/24`，Windows 按接口优先级选路；ESP32 官方默认热点网段恰好也是 `192.168.4.1/24`，撞车。

**解决**：把 ESP32 SoftAP 主动改成唯一网段 `192.168.77.1/24`（`wifi_ap.c` 第 2.5 步：停 DHCP → `esp_netif_set_ip_info` → 重启 DHCP），鲲鹏 `esp-ap` 静态改 `192.168.77.100`。此后与教室/家用路由不再冲突，普通 `ssh`/`scp` 直连即可，无需加路由或绑源地址。

---

### 9 网段迁移「改了源码没烧录」，77.100 实际不存在（11-21）

**现象**：`scp`/`ssh` 到 `192.168.77.100` 直接超时。查运行态：PC 的 WLAN 网关仍是 `192.168.4.1`、本机 `192.168.4.2`；`arp -a` 无任何 `192.168.77` 条目；而 `wifi_ap.c` 源码里明明已是 `ESP_IP4TOADDR(192,168,77,1)`。

**根因**：迁移只落在了**源码和文档**，没落板——① ESP32 板上仍跑旧固件（热点 4.x）；② 鲲鹏 `esp-ap` 的静态 IP 也仍是 `192.168.4.100`（文档写 77.100 但没改板子）。和「编译通过 ≠ 板子跑通」是同一类坑：**配置改在仓库里 ≠ 配置已生效在设备上**。

**解决**：重拷 `esp32` → `D:\esp\s3_voice` 后 `idf.py build && idf.py -p COM14 flash`（ESP32 实为 COM14）；PC 断连重连 `Kunpeng-Robot` 拿到 `192.168.77.2`；鲲鹏 `nmcli connection modify esp-ap ipv4.method manual ipv4.addresses 192.168.77.100/24 ipv4.gateway 192.168.77.1 && nmcli connection up esp-ap`。三方 `ping 77.1` / `ping 77.100` 均通。

**一句话面试说辞**：「先核对运行态（网关/ARP/实际 IP），再动手；别在错误网段上瞎试。配置文件改没改、和板子实际跑没跑，是两件事。」

---

### 10 9001 音频上行连不上：防火墙未放行 + 孤儿进程占端口（11-21）

**现象**：ESP32 monitor 里 `audio_tx` 每 3 秒超时重试连 `192.168.77.100:9001`；重跑 `audio_sink.py` 报 `OSError: [Errno 98] Address already in use`。

**根因**：① `firewalld` public zone 只放行了 `5000/8080/9000`，**缺 9001**，SYN 被静默丢 → 非阻塞 connect 3 秒超时；② 上次关 SSH 终端时远端 `audio_sink.py` 成了孤儿进程仍占 9001。

**解决**：`fuser -k 9001/tcp` 杀孤儿 + `firewall-cmd --permanent --add-port=9001/tcp --reload`；重起 sink 后 ESP32 下一个重试周期连上，**10 秒实收 320000 字节**（=16kHz×2B×10s，无丢包），`cap.wav` 取回可播、含咳嗽/人声。9001 音频上行 ✅。

**一句话面试说辞**：「TCP 9001 超时先查三层：对端进程在不在、端口被谁占、防火墙放没放行；别一上来就怀疑 WiFi 没连上。」

---

## 三、S3b 鲲鹏端视觉服务与网页控制台

> 来源：`排障记录-固件与ESP32.md`

> **为什么有这份文件**：工程里真正值钱的往往不是最终代码，而是「遇到过什么坑、怎么定位、怎么解决的」。
> 根 README 的「开发中遇到的问题与解决（16 条）」是固件 / 旧 PC 上位机阶段的历史记录；
> 本文件只记 **S3b 鲲鹏端**（`detector_service.py` :8080 / `web_control.py` :5000）阶段
> **重要、关键、面试官会追问**的问题，鸡毛蒜皮的小问题不记。
>
> **每条固定格式**：现象（带真实细节/数字）→ 排查过程 → 根因 → 解决 → 一句话面试说辞。
> 范本见 `protocol_engine_cpp/排障记录.md`。
>
> 记录区间：2026-11-07 ~ 11-08，S3b 线 1 / 线 2 实机联调 → 帧率优化 → A 方案点选抓取 → 收尾固化 → 眼在手取点约束复盘。

---

### 已发生

#### 1. 画面卡成 PPT：6.8 → 10 → 24.8 FPS，根因不在 YOLO 而在摄像头像素格式

- **现象**：浏览器画面约 6.8 FPS；重传 B 方案（抽帧推理）后 `/health` 两次帧号 `8928 → 9028`，Δ100/10s = **10.0 FPS**；
  把 `--detect-every` 从 4 调到 16、并加 `OMP_NUM_THREADS=2 MKL_NUM_THREADS=2` 限线程，CPU 从 72% 降到 18%，
  帧率**仍是精确 10.0**（`213 → 313`）。商品页标的却是「GF100 模组 60 帧」。
- **排查过程**：
  1. 先做判别实验：推理次数砍到 1/4、CPU 砍掉 3/4，帧率纹丝不动 ⇒ **推理不是瓶颈**（`ps -ef` 确认新进程真带 `--detect-every 16`，判别前提成立）。
  2. 停 8080（摄像头是独占设备，不停会测出假数据）后，用 `cv2.VideoCapture(0, cv2.CAP_V4L2)` 单独读帧自测：
     `size 640 480 / FPS_prop 10.0 / FOURCC YUYV / read FPS 9.1`。
  3. 试 `cap.set(CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))` 再测：
     `FOURCC MJPG / FPS_prop 25.0 / read FPS 22.1`。同分辨率、同摄像头，约 2.4 倍。
  4. 交叉核对：查 Hiwonder Aurora930 Pro 官方规格（RGB 640×400@12fps、NV12），确认买深度相机**不能**解决糊 + 卡。
- **根因**：GF100 UVC 摄像头默认协商到 **YUYV 未压缩 YUV 4:2:2**，640×480 每帧约 600KB，
  USB 2.0 带宽下设备原生只出 10FPS；`detector_service.py` 的 10.0FPS 精确等于采集上限、零额外损耗。
  旧版「每帧 `infer()`」只是叠加项（6.8 → 10），B 方案已把代码侧全部拿掉，**代码层优化到此为止**。
  商品页「60 帧」是 MJPEG 压缩 + 特定分辨率下的条件标称，不是 YUYV 下的实测值，也不是 720P@60fps。
- **解决**：`Detector.start()` 显式 `cv2.CAP_V4L2` 后端；**先**设 `CAP_PROP_FOURCC=MJPG` **再**设宽高（顺序反了会被打回 YUYV）；
  `IMWRITE_JPEG_QUALITY` 75 → 90；做成 `--fourcc MJPG|YUYV|AUTO` / `--jpeg-quality` 可回退；
  启动时回读并打印 `[采集] 实际协商：WxH / FOURCC / CAP_PROP_FPS`，请求值与回读不符就告警。
  **720P 不设默认**（未实测，贸然翻倍分辨率可能把 22FPS 打回 10），只留 `--width/--height` 现场可试。
  板上验收：`md5 0d4d69c1…` + banner `640x480 / MJPG / CAP_PROP_FPS=25.0` + `/health` 稳态 **24.8 FPS**。零成本、零硬件、零装包。
- **一句话面试说辞**：「我没有一上来调参，而是用『把推理次数砍到 1/4 看帧率变不变』的判别实验把瓶颈定位到采集端，
  再用 V4L2 回读确认是 YUYV 带宽问题，最后靠切 MJPG 零成本把画面从 10FPS 提到 25FPS——先分层归因，再动手改。」

#### 2. `read FPS 514638.5` 的假数据：测量本身无效，却算出了漂亮数字

- **现象**：摄像头自测打印 `can't open camera by index`、`VIDIOC_G_INPUT: Inappropriate ioctl for device`、
  `obsensor_uvc_stream_channel ... Camera index out of range`，读数是 `size -1 -1`、`FPS_prop -1.0`、
  `FOURCC ÿÿÿÿ`，却算出 `read FPS 514638.5`（第二次 `346159.9`）。
- **排查过程**：`-1` / `0xFFFFFFFF`（显示为 `ÿÿÿÿ`）都是「设备未打开」的哨兵值；
  计时代码是 `60 / (time.time() - t)`，而 `c.read()` 全部立刻返回 False，60 次循环几乎零耗时，
  除以极小的 Δt 就得到几十万。`ps -ef` 显示 8080 进程（PID 9522）仍在跑 ⇒ 摄像头被独占，
  第二个 `VideoCapture(0)` 被 V4L2 拒绝，`obsensor` 只是 OpenCV 回退乱试的深度相机后端噪音。
- **根因**：**测量前提不成立**（设备根本没打开）但脚本没有校验，把「读失败」当成「读得飞快」。
- **解决**：先 `Ctrl+C` 停 8080 再测；显式 `cv2.CAP_V4L2` 去掉 `obsensor` 噪音；
  给读数设有效性门槛（size 为正、FOURCC 是 YUYV/MJPG 之一、FPS 落在 5~60），不满足就判定测量无效、数字作废。
  停服务后重测才拿到有效的 `YUYV / 10.0 / 9.1`。
- **一句话面试说辞**：「任何性能数字先问『这次测量有效吗』——哨兵值加上除以极小时间差，会把无效采样包装成几十万 FPS；
  我是靠检查设备是否真打开把它识破的。」

#### 3. `FPS_prop 60.0` 但 `read FPS 26.5`：协商值不是真实吞吐

- **现象**：`cap.set(CAP_PROP_FPS, 60)` 后 `c.get(cv2.CAP_PROP_FPS)` 回读 `60.0`，
  但实测读 60 帧仍是 `26.5`；自测 `release()` 后重启 8080，banner 依旧是 `CAP_PROP_FPS=25.0`。
- **排查过程**：与 `/health` 稳态 24.8、短测 `read 22.1` 交叉对比，三者同属 ~25FPS 档；
  且 FPS 设置不跨进程残留，而 MJPG 由新版显式 `cap.set(FOURCC, MJPG)` 保证，与残留无关。
- **根因**：`CAP_PROP_FPS` 是驱动**当前协商值**，V4L2 接受「请求 60」就回读 60，
  但物理出帧受 USB2.0 带宽 / 固件 / 自动曝光限制，改标称改不了吞吐。与第 2 条同属「标称 vs 实测」坑。
- **解决**：**明确不把 `cap.set(CAP_PROP_FPS, 60)` 写进 `detector_service.py`** —— 不改真实帧率，
  只会让协商值虚高、误导日后排障。以 `/health` 帧号差分（Δframe_id / Δt）作为真实帧率口径。
- **一句话面试说辞**：「标称值不等于实测值，把 60 写进代码只会让以后排障被虚高数字带偏；
  我坚持用端到端帧号差分做验收口径，而不是读驱动的一个属性。」

#### 4. 改了网页内嵌 `PAGE`，浏览器狂刷还是旧图

- **现象**：PC 上已删掉 2D 侧视卡片、验证 `GET /` 渲染无残留，但浏览器 `Ctrl+F5` 后页面仍显示旧图 / 旧滑杆值。
- **排查过程**：区分两层缓存 —— 浏览器缓存（`?v=N` 改资源 URL 能绕）与
  **Flask 常驻进程内存**（服务启动时把 `PAGE` 字符串读进内存，每次 `render_template_string` 用的是那份）。
- **根因**：`web_control.py` 是长期运行的进程，覆盖磁盘上的 `.py` **不会**改变已加载的模块内容；
  `?v=N` 只改前端资源 URL，对后端进程内存无效。
- **解决**：改 `PAGE` / 模块级常量后一律 `Ctrl+C` 重启 5000 进程；`?v=N` 只用来绕浏览器缓存，两者配合。
  板上 5000 重启时启动命令沿用原条、末尾保留 `--ik-dir /root/kunpeng_vision`。
- **一句话面试说辞**：「前端缓存和后端进程内存是两层，我只刷浏览器当然不生效；
  先定位是哪一层，再决定是加版本参数还是重启服务。」

#### 5. Werkzeug 访问日志刷屏，`setLevel(ERROR)` 却把 `Running on` 一起屏蔽了

- **现象**：浏览器 1s 轮询导致 5000 终端被 `" - - ["` 访问日志刷满；
  改成 `logging.setLevel(ERROR)` 后启动 banner、`WARNING: development server`、`Press CTRL+C` 全消失，一度误判「服务没起来」。
- **排查过程**：Werkzeug 的访问日志与启动日志走同一个 logger，级别过滤是「一刀切」，
  把 INFO 的启动信息和 WARNING 的开发服务器提示一起干掉了。
- **根因**：想按**内容**过滤（只挡访问日志），却用了按**级别**过滤的工具，粒度不匹配。
- **解决**：模块级自定义 `_NoAccessLog(logging.Filter)`，`filter()` 里按 `" - - ["` 特征串返回 False，
  挂到 werkzeug logger；启动 / WARNING / 真错误原样保留。板上 Werkzeug 3.1.9 实测与 PC 版一致
  （banner 齐全、点 PING/STATE 后终端零新增日志）。
- **一句话面试说辞**：「过滤粒度要匹配目标——按级别一刀切会误伤启动日志，
  我改用 Filter 按内容精确匹配，只挡访问行，保留排障真正需要的信息。」

#### 6. PowerShell 里中文 commit 回显乱码，还跟了一句 `fatal: unknown write failure`

- **现象**：`git commit -m "S3b: 视觉服务抽帧推理 + ..."` 回显 `[main a9954c3] S3b: 瑙嗚鏈嶅姟鎶藉抚鎺ㄧ悊 ...` 全乱码；
  第 2 个 commit 后又冒出 `fatal: unknown write failure on standard output`。
  但随后 `git push origin main` 成功（`cc196bf..9f8d95b`），`git log -4 --oneline` 显示的是**正常中文**。
- **排查过程**：`git log -2 --format=%B` 直接读 commit object 的原始 message，读回两条完整正常中文
  （`S3b: 视觉服务抽帧推理 + MJPG 采集 + JPEG 质量 90 + 启动参数回读` /
  `S3b: 网页新增点选抓取 IK 桥接 + 彻底移除 2D 侧视示意图`）；
  再核 `git rev-parse HEAD` = `git rev-parse origin/main` = `9f8d95b`、`git diff --stat` 为空 ⇒ 两个 commit 都成功写入。
- **根因**：Windows 控制台码页 936（GBK）把 Git 输出的 UTF-8 字节按 GBK 解码，产生 mojibake；
  `fatal: unknown write failure` 是往 stdout 写摘要那一下失败，对象与 ref 已落盘。
  两者都是**显示 / 输出层**问题，不是数据损坏。
- **解决**：不 `amend`、不 `force`、不改 `i18n.commitEncoding`；用 `git log --format=%B` 验证存储内容、
  用 `git status` 验证提交状态，别被终端回显带偏去重跑 commit（那会变成空提交或重复提交）。
- **一句话面试说辞**：「终端显示乱码先分清是数据坏了还是只是解码错了——
  我直接读 Git 对象确认是合法 UTF-8，避免了一次多余的 amend。」

#### 7. 「换姿态点同一像素，(x,y) 应该会偏」——反向验证判据写反了，反而暴露出眼在手的算法前提

- **现象**：整理「固定观察位取点」验证清单时写了一条反向验证：「用滑杆换个姿态 → 点**同一画面位置** → 只试算 → `(x,y)` 明显偏移」，
  打算拿它当「单姿态边界」的面试证据。按 `web_control.py` 实际跑，两次试算的 `(x,y)` 与六路脉宽**完全一样**，一步都没偏。
- **排查过程**：
  1. 读 `api_ik_solve()`（L1581-1584）：只把 `(u,v,w,h)` 交给 `pixel_to_table()`，再 `solve_to_pulses(x,y)`；
     函数签名和函数体里**没有** `STATE` / 关节角 / 相机位姿 / `_HOST_STATE` 任何一个入参。
  2. 读 `pixel_to_table()`（L128 起）：`x = center_x + (u/w − 0.5)·span_x`、`y = center_y + (0.5 − v/h)·span_y`，
     `z` 取常量 `_IK_MAP["z"]`；只吃像素 + 5 个标定标量。
  3. 交叉核对前端 `ikSolve()` 的 body 只有 `{u, v, w, h, apply}`（L1203），没有回传当前姿态 ⇒ 前后端一致，不是某一层漏传。
- **根因**：**把「物理世界会变」错当成「代码输出会变」**。相机装在臂末端（eye-in-hand），换姿态后相机位姿 `T_base_cam` 确实变了，
  **同一个物理物体在画面里的像素位置**会跑掉；但代码不做相机位姿建模，等于把相机当成永远固定在标定姿态 q₀，
  所以「同一像素」永远映射到同一个 `(x,y)`。这是「平面约束单目定位」隐含 `T_base_cam ≡ 常量` 的必然结果，**不是 bug**。
- **解决**：
  1. 撤回错误判据，改成两条 —— 点**同一画面位置** `(x,y)` **不变**（反证「拍照姿态 = 标定姿态」这一前提）；
     换姿态后点**同一物体的新像素** `(x,y)` 才明显变化，这才是「眼在手必须做手眼标定」的直观证据。面试只讲后者。
  2. 把「⚠️ 再取点前先回观察位」从操作建议升级为**算法前提**写进验证清单：不回观察位的点全部作废。
  3. 真要随姿态取点，需 `T_base_cam(q)=T_base_end(q)·X`（`X` 由 `handeye_calib.py` 解 AX=XB）+ 内参 K 反投影射线与 `Z=z` 平面求交；
     当前**未实标、未接入**，不写成已完成。
- **一句话面试说辞**：「我一开始把验证判据写反了，读码才发现逆解只吃像素、不吃关节角——这恰恰说明当前方案是开环平面映射，
  相机装在臂末端就必须固定观察位；要随姿态取点得先做手眼标定，我没做，所以不吹闭环。」

#### 8. `{"ok":true,"resp":""}` —— 接口说成功，命令却进了黑洞（USB 串口旧 fd 假活）

- **现象**：`curl -X POST /api/cmd -d '{"payload":"JOINT,1580,1470,2177,1106,1450,1567"}'` 返回
  `{"ok":true,"resp":""}`，机械臂**一动不动**；紧接着 `STATE` 同样只回 `{"ok":true,"resp":""}`。
  而 `POST /api/open {"port":"/dev/ttyUSB0"}` 之后 `PING` 立刻回 `OK,PONG`，同一批命令再发就正常动了。
- **排查过程**：
  1. 先怀疑命令格式：固件 `proto_dispatch()` 用 `strchr(cmd, ',')` 切命令名与参数，写成空格会回 `ERR,1`。
     改成逗号后**仍然**是空串、也不是 `ERR,1` ⇒ 说明命令**根本没到固件**，不是语法问题。
  2. `ps -ef | grep python3`：只有**一个** `web_control.py`（无第二个进程抢串口）；
     `ls -l /dev/ttyUSB*`：节点在、权限 `crw-rw---- root dialout`。
  3. `dmesg | grep -i -E "ch341|ttyUSB"` 只有开机时那条 attach，**没有 disconnect/reset** ⇒ 不是标准 USB 掉线日志；
     但 `ls -l` 里节点的日期是 `11月11 11:03`，而 `web_control.py` 是 **09:41** 启动的
     ⇒ 它手里的 fd 指的是 **11:03 之前**那个节点，故障恰好发生在其后一分钟。
  4. 重开串口后 `PING` 立即通 ⇒ 写路径与固件都是好的，**坏的就是那个旧 fd**。
  5. 读 `SerialHost._rx_loop`：`read(1)` 在链路失效后**不抛异常**，只一直返回 `b''`，走 `if not ch: continue`；
     而 `_dead_reason` 只在 **except** 分支里置位 ⇒ 永远不报错。`send()` 等满 0.6 s 后 `return ""`，
     `/api/cmd` 又把它包成 `ok:true` ⇒ 假成功。
- **根因**（两层，缺一不可）：
  1. **传输层**：USB 串口节点被销毁/重建后，pyserial 的 `is_open` 仍为 `True`、`write()` 不报错，
     读线程静默收 `b''`。原 `_dead_reason` 只捕获**异常**，抓不到「静默 EOF」这个洞。
  2. **接口层**：`send()` 用**空串**表示「超时失败」，而 `/api/cmd` 把「写成功」当成「执行成功」，
     失败被包装成 `ok:true` —— 上层完全无法区分。
- **解决**：
  1. `SerialHost` 打开时记下设备节点身份 `(st_dev, st_ino, st_ctime)`（`_read_dev_id()`），
     `port_replaced()` 负责比对；`_rx_loop` **每秒**体检一次、`_write` **写前**再查一次，
     不一致立即置 `_dead_reason` 并抛出「请重新 POST /api/open」。
     Windows `COMx` 拿不到节点身份 ⇒ 返回 `None` ⇒ 不启用该检测（不误杀）。
  2. 三个 Host 类（`SerialHost` / `TcpHost` / `TcpServerHost`）的 `send()` 超时一律
     **抛 `RuntimeError`**，绝不返回空串；`/api/cmd`、`/api/state` 里既有的 `except` 直接把它变成
     `ok:false` + 明确文案（含"查 12V/5V 供电与杜邦线"的下一步）。
  3. `/api/open` 打开成功后**补一次 `PING`**：端口开了但 STM32 不说话时，回 `ok:true + warning`
     —— 端口确实开了，但不谎报链路可用。
  4. `/api/state` 补 `len(parts) < 9` 校验：Python 切片越界**不抛异常**，原来会静默返回少于 6 路脉宽。
  5. 页面 1 s 轮询失败时打日志（**原因变化才打**，避免 1 秒一条刷屏），链路坏了肉眼可见。
  6. 本地验证：`test_eye_in_hand_mock.py` 45 项全过（含 `/api/open` 与 apply 下发路径）；
     另跑临时自检覆盖「无应答必抛异常 / 节点身份变则拒发 / 拿不到身份不误杀」。
  7. **真机反向验证（关键一步，顺序不能错）**：拔 CH341 → 等 3 s → 插回 → **先不发 `/api/open`**，
     直接发 `/api/cmd {"payload":"PING"}`。实测回
     `{"ok":false,"error":"串口读取线程异常退出（device reports readiness to read but returned no data
     (device disconnected or multiple access on port?)）；请重新 POST /api/open 连串口"}`
     —— 旧代码在同一场景给的是 `{"ok":true,"resp":""}`，对比成立。
     ⚠️ **不能先发 `/api/open` 再测**：`api_open()` 会 `close()` 旧 Host 再 `new SerialHost()`，
     等于亲手把「手里攥着旧 fd」这个要验的状态清掉，测不出任何东西。
  8. **顺带挖出节点名不稳**：同一物理口拔插，`ttyUSB0`（inode 600 / 11:35:31）变成了
     `ttyUSB1`（inode 622 / 11:39:53）。原因：`ttyUSBn` 的 minor 号取「当前最小编号空闲位」，
     驱动不记「这个口上次是几号」，旧 tty 注销与新设备注册交叠时就会顺延。
     ⇒ **不能硬编码节点名**，改用 udev 稳定别名（CH341 无序列号时 `by-id` 按 vid/pid 生成，
     实测存在）：`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`（也可用 `by-path`，但按物理口，换口会变）。
     注意 `os.stat()` 会**跟随软链**取到目标设备节点，所以 inode 比对在软链下同样有效。
- **一句话面试说辞**：「我的网页接口出现过『返回 ok:true 但机械臂不动』——根因是 USB 串口掉线后
  `read()` 只返回 EOF、不抛异常，而我的超时用空串表示失败、接口层又只判断『写成功』，于是失败被包装成成功。
  修法是两层：传输层打开时记设备节点 inode，写前和读线程每秒比对，变了立刻判死；接口层超时直接抛异常，
  再也不会假成功。」

---

### 本阶段诚实边界（写简历 / 面试时不要越线）

- 画面约 **25 FPS** 是**采集 + 推流**帧率；YOLO 推理仍是 `--detect-every 4` 约 6Hz、检测框滞后，**不能写「实时视觉」**。
- A 方案是**开环**：点画面取点 → 平面约束逆解 → 六路脉宽 → `JOINT` 下发 → `STATE` 回读；
  不含下降 / 闭合 / 抬起的完整抓取序列，也没有关节编码器反馈，**不能写「自动抓取」「闭环控制」**。
- 单目 + 平面假设做 2D 定位，**不是真 3D 定位**；没有深度相机。（📌 订正 11-16：原文「没有手眼标定实标」为 11-06 旧状态；**11-11 已完成 12 组姿态实标并接入 `--eye-in-hand`**，resid 1.60mm。本文第 7 条 130/132 行的「未实标 / 我没做」同属当时历史叙述，保留原貌不改写。当前权威口径见 `00-项目总览与硬数据速查.md` §10。）
- MJPG 下 `c.read()` 仍解码成 BGR 再由 `imencode` 压一次，双重有损只是**减轻**、没有消除，画质提升有限。
- 2D 侧视姿态图已彻底移除（按脉宽线性映射、非运动学正解），不要再当成果展示。
- **眼在手 / 固定观察位**：相机装在臂末端，`pixel_to_table()` 无相机位姿建模，5 个标定常量只在「拍照姿态 = 标定姿态」成立；
  臂一动 `T_base_cam` 变、映射失效 ⇒ **每次取点前必须回同一观察位**，这是算法前提不是操作习惯。
  换姿态后点同一画面位置 `(x,y)` 不变（代码如此），点同一物体的新像素才变。
  `handeye_calib.py` 已实现但**未相机实标、未接入点选链路**，不能写「眼在手闭环 / 视觉伺服 / 实时跟踪 / 边看边抓 / 自动抓取」。
- **三档口径**：以 ✅ 已实证 / 🔶 已搭建未验证 / 📋 纯规划 标注（原 `面试与简历材料.md` 已删除，2026-11-24）。
  下面这行是 **S3b 阶段（11-07）的历史快照**，1.2b 之后已部分失效，别直接拿到面试用：
  - 当时：`iot/`（Docker 脚手架 + 模拟设备）、`esp32/`（仅 README 占位）均属 🔶/📋。
  - 现在（11-09）：`iot/` 已是 ✅ **板内可观测性闭环**（真实遥测 → EMQX → eKuiper → InfluxDB → Grafana，收口 `e6cd194`；仍不能写「云端 IoT / 自动抓取 / 闭环控制」）；`esp32/` 是 🔶 **ESP-IDF 骨架已落盘、未编译 / 未烧录 / 未上板** + 📋 语音整体规划。（✅ 11-21 订正：已编译 + 烧录实机，TCP 9000/采音通过）
  - 仍属 📋 纯规划：双 STM32、双相机 6D 位姿、LLM 规划、OTA、云端 IoT、数字孪生，**不得写成已完成**。