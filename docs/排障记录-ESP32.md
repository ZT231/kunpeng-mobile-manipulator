# 排障记录 · ESP32-S3 语音入口骨架

> 固定五段式：**现象 → 排查过程 → 根因 → 解决 → 一句话面试说辞**。
> 只记重要 / 关键 / 面试官关心的问题，鸡毛蒜皮不记。
> 本文件是个人复习材料，**不进 Git**（`docs/排障记录-*.md` 一律不 add）。

---

## 第 1 条　本机无 ESP-IDF 工具链，整机骨架编译不了，却差点把「落盘」说成「跑通」

**现象**
B 阶段要「上板建 ESP-IDF 骨架 + Wi-Fi + I2S 采音」，但本机检查回执是：
`gcc (GCC) 9.3.0` 有，`command -v idf.py` 无输出，`~/.espressif` 不存在，`esp32/` 里只有 `README.md`。
也就是说本机只能编纯标准 C，编不了依赖 `driver/i2s_std.h`、`esp_wifi.h` 的 ESP32 工程。

**排查过程**
1. 先分清「哪些文件依赖 ESP-IDF 组件」：
   `main.c` / `i2s_mic.c` / `wifi_sta.c` / `main/CMakeLists.txt` 都依赖官方组件头与构建系统；
   只有 `protocol.c` / `protocol.h` 只 include `<stddef.h>` `<stdint.h>`，是纯标准 C。
2. 于是把验证拆成两层：能在本机做的（纯 C 封帧算法）与必须去装有 ESP-IDF 的机器做的（整机 build / flash）。
3. 用本机 gcc 把 `protocol.c` 单独编译成一个自测 exe，跑 S2 实机锚点帧反查。

**根因**
ESP-IDF 是一整套交叉编译工具链（Xtensa 编译器 + 组件库 + `idf.py` 构建脚本），本机只装了宿主机 gcc，
两者目标架构不同：宿主机 gcc 出 x86 代码，ESP32-S3 要 Xtensa 代码。
把「文件已落盘」和「工程已编译烧录」混为一谈，就是简历上最容易被面试官一问就露馅的越线。

**解决**
- 骨架照常落盘（当时 10 个文件，11-14 晚补 `tcp_client.c/.h` 后为 12 个）；`esp32/README.md` 第十节**当时**明确写「整机 `idf.py build` 未跑、未烧录」，命令只在 ESP-IDF 环境里执行（该节 11-14 17:38 已按「PC 已装 v5.5.5」订正、11-14 晚又按「整机 build 已通过」订正，见本条文末与第 5 条）。
- 能验证的那部分真的验证：`x86_64-w64-mingw32-gcc -static` 编译 `protocol.c` + 自测 main，跑出
  `$PING*10`（CK=`10`）、`$OK,PONG*3E`（CK=`3E`）、`$ASR,mic_ok*50`（CK=`50`）、中文 UTF-8 三字节逐字节 XOR=`FF`，`EXIT=0`。
  这与 STM32 `Core/Src/uart_protocol.c`、Python `kunpeng/llm/text_gateway.py` 三处一致。
- 软件路线表把步骤 1 标成「骨架已落盘 / 整机 build 未跑」，诚实边界表里补一行「能说 / 不能说」。

**一句话面试说辞**
「我先分清了宿主机编译器和交叉编译工具链的边界：ESP-IDF 工程在本机只能落骨架、不能整机编译，所以我把不依赖芯片 SDK 的协议层单独抽成纯标准 C，用宿主机 gcc 拿项目实机跑通的 `$PING*10` 反查校验算法，剩下的 build/flash 明确留给 ESP-IDF 专用环境，绝不把『代码已写』写成『板子已跑』。」

> ⚠️ **11-14 17:38 订正（撤回本条里的过时事实描述）**：本条写的「本机无 `idf.py` / 本机只装了宿主机 gcc / 要去另一台装好 ESP-IDF 的机器」是**当时在 Git Bash 里看到的历史现象**，不是「没装」。
> 现已查明 **PC 上早已装好完整 ESP-IDF v5.5.5**（`D:\ESP32\Espressif`，EIM / Espressif-IDE 风格安装，纯 ASCII 路径合规），启动脚本 `Initialize-Idf.ps1` / `idf_cmd_init.bat` / `export.ps1` / `export.bat` 齐全。
> 当时看不到 `idf.py` 只是因为 **Git Bash 的 PATH 里没有 ESP-IDF 工具链**；且本轮实测 **Git Bash 里跑 `idf.py` 会被官方直接拒 `MSys/Mingw is no longer supported`** ⇒ 必须从 **ESP-IDF PowerShell / cmd 快捷方式**启动。
> ✅ **结论订正（11-14 晚）**：骨架整机 `idf.py build` **已在 PC 上跑通并回读确认**（`BUILD_RC=0` / `Project build complete` / 产物 4 件齐全，见第 5 条与 `docs/明日回教室-ESP32照跑清单.md` 第一节）；**仍未烧录，档位仍 🔶**。「宿主机 gcc ≠ 交叉工具链」这条边界判断仍成立、`protocol.c` 三处一致的实证仍有效。
> 🔴 今后一律写「PC 已装 v5.5.5，须从 ESP-IDF PowerShell 快捷方式启动」，**不再写「本机没装 / 要先装 ESP-IDF」**；ESP-IDF 装在 PC（USB 烧录要摸到板载 CH343），**不装在鲲鹏板**（鲲鹏只跑 `web_control.py` / `voice_gateway.py` 等运行时）。详见 `docs/ESP-IDF安装与首次build指引.md` 第零节。

---

## 第 2 条　Cygwin gcc 编出的 exe 在 `/tmp` 下报 `No such file or directory`

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

## 第 3 条　PC 上明明装了 ESP-IDF v5.5.5，却「跑不起来」整机 build（11-14 晚）

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

## 第 4 条　帧「格式」逐字节都对，接上真网关却一律 `$ERR,2`（11-14 晚）

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
- 完整实证与分流表 → `docs/ESP32-S3接入与阶段收尾路线.md` 3.3、`docs/明日回教室-ESP32照跑清单.md` 第七节。

**一句话面试说辞**
「跨端联调我先把『帧格式逐字节一致』和『载荷语义被识别』拆成两层验证：XOR/UTF-8 用本机 harness 钉死，
再把原来写死的英文自检帧换成纯 ASCII 快速通道先验链路，中文帧单独留安全闸门。`ERR,2` 是词表不匹配不是编码错，
这样定位一次就准，也避免在没确认现场安全时误触发机械臂动作。」

---

## 第 5 条　首次全量编译 20 分钟超时，被误读成「编译失败」（11-14 晚）

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
- 🔴 编译通过只把状态推进到「🔶 有代码、本机交叉编译通过」，**仍未烧录 / 未上板 / 未联调**，档位不变。

**一句话面试说辞**
「超时报错我先去看构建目录有没有在推进，发现 Ninja 已到 857/1082，判断是外层脚本的固定超时而非编译错误；
删掉超时、复用增量构建续跑即拿到 `Project build complete`。全程没改一行业务代码，也没有把『编译通过』写成『板子跑通』。」

---

> 后续上板实测（I2S 读出全 0、Wi-Fi 连不上、CH343 下载口识别等）再逐条补。
> 当前骨架 **11-14 晚在 PC 上做过本机实证 + 整机 `idf.py build`（已通过），未烧录、未上板**，不预先编造排障现象。
