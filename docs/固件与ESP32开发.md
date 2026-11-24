# 固件与 ESP32 开发

> ⚠️ **2026-11-23 旧底盘作废**：旧 `car/`（L9110S + `$CAR,FWD` 开环定时）已整目录删除，
> 相应双板流程不要再照做。本文第五节**已替换为现行底盘验收**；新底盘是
> **2×TB6612FNG + 4×JGB37-520(1:90) + 80mm 麦轮**，Keil 工程仍是
> `MDK-ARM/RoboticArm_Kunpeng.uvprojx`；接线、烧录、`$CAR,VEL` 验收命令见
> `docs/底盘-TB6612与JGB37-520-接线、固件与调试.md`。

> 🔴 **2026-11-24 固件清理**：`pca9685.c`、`arm_control.c`、`safety.c`、`acs712.c`、`rfid.c`、`teach.c`、
> 电位器 ADC+DMA 与 I2C1 已全部删除。本文凡出现 `$JOINT` / `$MOVE` / `STATE` / `PCA9685` / `Arm_Init`
> 的段落，都是**历史记录或未来目标形态**，当前固件不处理这些命令；现行命令只有
> `$PING/$LOG/$CAR,.../$STOP`。

> 本文由 2026-11-21 的多份固件 / 协议 / ESP32 草稿合并整理而成（内容未改写，仅重组与订正；源文件已删除，以本文为现行版本）。
>
> 硬件口径以《硬件清单与接线》与《00-项目总览与硬数据速查》为准。

## 目录

1. ESP-IDF 环境与首次 build
2. ESP32-S3 接入与阶段收尾路线
3. ESP32 与 IoT 核心化路线
4. 照跑清单（11-14 版，命令仍可用）

---

## 一、ESP-IDF 环境与首次 build

> 来源：`固件与ESP32开发.md`

> 🔴 **重大订正**：本文 11-14 早先版本写「下一步是装 ESP-IDF」——**错，已撤回**。
> 依据是 `esp32/README.md` 里那句过时的「本机无 `idf.py`」；用户 17:38 指出 **PC 上早已装好**，
> 本轮实测确认：**`D:\ESP32\Espressif` = 完整 ESP-IDF v5.5.5（EIM 安装），无需再装**。
> 文件名沿用早先命名，**内容已改为「现有环境核对 + 直接 build」**；原「安装」章节降级为**附录 A**（换电脑 / 重装时才用）。
>
> 📋 本文是**操作指引，不是结果**。17:38 版说「本轮未 build」——**11-14 晚已补做并回读确认通过**：PC 上用现成 v5.5.5 在 ASCII 副本 `D:\esp\s3_voice` 跑了 `set-target esp32s3`（**rc=0**）+ 整机 `idf.py build`（**`BUILD_RC=0` / `Project build complete` / 产物 4 件齐全 / `tcp_client.c` 链接成功**；证据见 `docs/固件与ESP32开发.md` 第一节）。🔴 **编译通过 ≠ 实机**：**已烧录实机验证**；`esp32/` 档位仍 🔶。
> 关联：`docs/固件与ESP32开发.md`（明天直接照跑）、`esp32/README.md` 第十节、`docs/固件与ESP32开发.md` 第四节、`docs/固件与ESP32开发.md` 第七节。

---

### 零、先答用户的两个问题

#### 0.1 「不是装在鲲鹏开发板上吗？」→ **不是，装在 PC 上是对的**

| 角色 | 跑什么 | 装不装 ESP-IDF |
|---|---|---|
| **Windows PC** | 编译 ESP32 固件、USB 烧录、看串口 | ✅ **装在这里**（就是你 `D:\ESP32\Espressif`） |
| 鲲鹏 KunPeng Pro（openEuler，STA 静态 `192.168.77.100`） | 视觉 / Flask `web_control.py` / **`voice_gateway.py` TCP :9000** / IoT 四容器 | ❌ **不装** |
| ESP32-S3 成品板 | 采音 → 唤醒 → 封帧，经 **Wi-Fi TCP** 连鲲鹏 `:9000` | ❌ 它是被编译/烧录的**目标芯片**，不是开发主机 |

**大白话**：ESP-IDF 是「给 ESP32 写程序 + 把程序灌进板子」的工具箱。程序要**通过 USB 线**灌进 ESP32 板，而那块板插在 **PC** 上，所以工具箱必须装在 PC。鲲鹏板是「收 ESP32 发来的话、再去指挥机械臂」的**运行侧**，它不碰编译，也不碰那根 USB 烧录线。

**规范定义**：ESP-IDF（Espressif IoT Development Framework）是含 Xtensa/RISC-V 交叉编译器、CMake/Ninja、`esptool.py` 的**主机端交叉编译工具链**；烧录需主机通过 UART/USB-Serial（本项目板载 **CH343**）与目标芯片物理连接。鲲鹏侧是 ARM64 Linux **运行时**：`web_control.py`（Flask，已实机 ✅）、`voice_gateway.py`（TCP 收帧网关，阶段 2 已实机 ✅）、EMQX→eKuiper→InfluxDB→Grafana 四容器。**编译工具链与运行时是两个不同节点**。

🔴 **三条为什么不该装到鲲鹏板上**：
1. ESP32 板的 USB 插在 PC ⇒ 烧录 `esptool.py` 必须能摸到那个串口，鲲鹏板上没有这根线。
2. 鲲鹏的 USB 口已被**机械臂 CH341 串口** + **GF100 相机**占用（项目铁律：CH341 走 USB2.0、相机走 USB3.0、地星型汇合），再插板会抢资源。
3. 鲲鹏是 ARM64 openEuler；乐鑫虽在部分版本提供 linux-arm64 工具链，但**编译完也没法在那儿烧录**，纯属多绕一圈。

> 🔍 **混淆来源**：上一轮说的「上板」指的是鲲鹏侧 **`voice_gateway.py`（Python 网关）** 在板上跑通（16:40 已 ✅），**不是 ESP-IDF**。两者不是一回事。

#### 0.2 「我 PC 上有 ESP-IDF」→ **对，实测确认，不用装**

本轮在你 PC 上**只读核对**（未安装、未改配置）：

| 核对项 | 实测值 | 结论 |
|---|---|---|
| 安装根目录 | `D:\ESP32\Espressif` | 纯 ASCII、无空格、19 字符 ⇒ ✅ **官方路径规范本来就满足** |
| IDF 本体 | `frameworks\esp-idf-v5.5.5\` | ✅ **v5.5.5**，正是骨架该用的 5.5.x 最后一档 |
| Python venv | `python_env\idf5.5_py3.11_env\Scripts\python.exe` | ✅ Python 3.11 |
| Git | `tools\idf-git\2.44.0\cmd\git.exe` | ✅ |
| 配置 | `esp_idf.json`：`version 5.5`，`idfSelectedId = esp-idf-9e0fb603f7ef23663b5245b0846b9a0c` | ✅ EIM / Espressif-IDE 风格安装 |
| 启动脚本 | `Initialize-Idf.ps1`、`idf_cmd_init.bat`、`frameworks\...\export.ps1` / `export.bat` | ✅ 齐全 |
| `tools\idf.py` | 存在（40975 B） | ✅ |
| 裸调 Python 读版本 | 输出 **`ESP-IDF v5.5.5`**（附 `WARNING: IDF_PYTHON_ENV_PATH is missing`） | ✅ 本体可用；⚠️ 警告说明**必须先跑初始化脚本** |
| Git Bash 里调 `idf.py` | `MSys/Mingw is no longer supported` | 🔴 **不能用 Git Bash 跑**，必须 ESP-IDF PowerShell / cmd |

🔴 **撤回旧口径**：`esp32/README.md` / `docs/排障记录-固件与ESP32.md` 里「本机没有 `idf.py`、没有 `~/.espressif`」是**历史记录**（当时 Git Bash 的 PATH 里确实没有），**不等于没装**。今后一律写「PC 已装 v5.5.5，须从 ESP-IDF 快捷方式启动」。

---

### 一、概念先讲清（大白话 + 规范定义）

| 概念 | 大白话 | 规范定义 |
|---|---|---|
| ESP-IDF | 乐鑫官方给 ESP32 写程序的「整套工具箱」，装了才能编译、烧录 | Espressif IoT Development Framework，基于 FreeRTOS 的官方 SDK；提供 `idf.py` 构建前端（CMake + Ninja）、Xtensa/RISC-V 交叉编译工具链与组件库 |
| `idf.py` | 一个命令干完「配置 / 编译 / 烧录 / 看串口」四件事 | IDF 的 Python 构建前端，封装 `cmake`、`ninja`、`esptool.py`、`idf_monitor` |
| `Initialize-Idf.ps1` | 开一个「已经配好环境变量」的专用 PowerShell 窗口 | 由安装器生成的 Windows 快捷方式调用；用 `idf-env config get` 查出 Python / Git / IDF 路径，再 dot-source `export.ps1`，把工具链加进 `PATH`，并定义 `idf.py` 这个 PowerShell **函数** |
| `IDF_PYTHON_ENV_PATH` | 工具箱记「我用哪个 Python」的标签 | v5.x `activate.py --export` 设置的环境变量；裸调 `python.exe tools\idf.py` 会缺它并告警，`build` 更会缺编译器 ⇒ 必须走初始化 |
| `set-target` | 告诉工具箱「我这块芯片是 ESP32-S3」，第一次必须做 | `idf.py set-target <chip>` 会清掉已有 build 配置并按芯片重新初始化 `sdkconfig` |
| `sdkconfig.defaults` | 第一次编译时自动填进去的「默认开关表」 | 首次 build 由它生成工程根的 `sdkconfig`；已生成后再改 defaults 不生效，须 `menuconfig` 或删 `sdkconfig` 重来 |

---

### 二、🔴 仍然有效的坑：中文**工程**路径（安装路径本身没问题）

**官方原文**（ESP-IDF v5.5.4《Standard Setup of Toolchain for Windows》）：

> - The installation path of ESP-IDF and ESP-IDF Tools **must not be longer than 90 characters**.
> - The installation path of Python or ESP-IDF **must not contain white spaces or parentheses**.
> - The installation path of Python or ESP-IDF **should not contain special characters (non-ASCII)** unless the OS is configured with "Unicode UTF-8" support.
> - **The ESP-IDF build system does not support spaces in the paths to either ESP-IDF or to projects.**

✅ **安装侧**：`D:\ESP32\Espressif` 纯 ASCII、无空格、19 字符 ⇒ **完全合规，一个字都不用改**（上一版文档让你重装到 `D:\esp\esp-idf` 是多余的，撤回）。

🔴 **工程侧**：`esp32/` 在
```
D:\简历\STM32项目\4.ESP32-S3项目\RoboticArm_Kunpeng\esp32\
```
含「简历 / STM32项目 / ESP32-S3项目」三组中文 ⇒ **属于官方点名的非法工程路径**。`esp32/README.md` 第十节旧写法 `Set-Location "D:\简历\...\esp32"; idf.py build` 会失败或出诡异错（CMake 找不到源文件 / Python 路径编码错 / Ninja 路径解析失败），11-14 已订正。

✅ **解法（沿用项目「拷贝 + 原目录不动」的老规矩）**：把 `esp32/` 整个**拷到纯 ASCII 短路径**再 build。

| 项 | 值 |
|---|---|
| 工程副本目标 | `D:\esp\s3_voice`（15 字符，无中文 / 空格 / 括号） |
| ESP-IDF 安装目录 | **`D:\ESP32\Espressif`（现有，不动）** |
| 原工程 | `...\RoboticArm_Kunpeng\esp32\` **保持不动**，只当源码真源 |

🔴 **别用「开启系统 Unicode UTF-8」去硬扛中文路径**：官方是当例外写的，要重启、还可能让 SecureCRT / Keil 乱码。拷目录零成本，绕开最干净。

---

### 三、完整指令（PowerShell，单行逐条粘）

> 🔴 **在哪跑**：**Windows PC**（USB 插 ESP32 板），不是鲲鹏板。
> 🔴 **用哪个窗口**：从开始菜单打开 **「ESP-IDF 5.5 PowerShell」**（或 `ESP-IDF Command Prompt`）快捷方式 —— 安装器必建，`Initialize-Idf.ps1` 第 1 行注释明说「called from a Windows shortcut」。
> 🔴 **绝不在 Git Bash / MSYS 里跑**：本轮实测 `idf.py` 直接回 `MSys/Mingw is no longer supported`。
> 🔴 **PS 5.1 不支持 `&&`**；下面每条一行、单独粘，不合并。

#### A. 启动环境（不用装，一次性每个新窗口做一次）

**A1（首选）**：开始菜单搜 `ESP-IDF` → 打开 **「ESP-IDF 5.5 PowerShell」**。窗口里已自动跑过 `Initialize-Idf.ps1`，`idf.py` 是现成的函数。

**A2（找不到快捷方式时的手动等价写法）**，在**普通 PowerShell** 里逐条粘：

```powershell
$env:IDF_TOOLS_PATH = "D:\ESP32\Espressif"
```

```powershell
. "D:\ESP32\Espressif\Initialize-Idf.ps1" -IdfId "esp-idf-9e0fb603f7ef23663b5245b0846b9a0c"
```

> 原理：`Initialize-Idf.ps1` 用 `-IdfId` 向 `idf-env` 查 path / python（本轮已实证这两条查询都返回正确路径），再 dot-source `export.ps1` 把工具链加进 `PATH`。
> ⚠️ 本助手在**非交互** PowerShell 会话里跑这条初始化脚本没能完整跑通（`export.ps1` 需要交互式 export），所以**首选 A1 快捷方式**；A2 是逻辑等价的退路，请在你本机窗口里验。

> ⚠️ 首次插上 ESP32-S3 成品板，若设备管理器里看不到 COM 口 ⇒ 先装 **CH343 USB-TTL 驱动**（板载 CH343），记下实际串口号，后面 `flash` 要用。

#### B. 验环境（零硬件、不碰板）

```powershell
idf.py --version
```

**判据**：输出 `ESP-IDF v5.5.5`。
（本轮已用安装自带 Python 裸调 `tools\idf.py` 拿到 `ESP-IDF v5.5.5`，但带 `IDF_PYTHON_ENV_PATH` 警告 ⇒ **仍要在 A1 窗口里复验一次干净的**，确认工具链 PATH 也配好了。）

#### C. 把工程拷到 ASCII 路径（🔴 必做，绕开中文路径）

```powershell
New-Item -ItemType Directory -Force -Path D:\esp | Out-Null
```

```powershell
Copy-Item -Path "D:\简历\STM32项目\4.ESP32-S3项目\RoboticArm_Kunpeng\esp32" -Destination "D:\esp\s3_voice" -Recurse -Force
```

```powershell
Set-Location D:\esp\s3_voice
```

> 拷完应能看到 `CMakeLists.txt`、`sdkconfig.defaults`、`main\`。
> 🔴 **源码以仓库内 `esp32/` 为准**，`D:\esp\s3_voice` 只是纯 ASCII 构建副本：每次改完 `esp32/main/` 源码都要**重拷到副本再 build**，否则烧的是旧代码。热点参数 `Kunpeng-Robot` / `12345678` 是演示默认值（已提交），正式部署前改成本地值。

#### D. 首次 build（本机就能验，不用接板）

```powershell
idf.py set-target esp32s3
```

```powershell
idf.py build
```

**判据**：结尾出现 `Project build complete. To flash, run:` 以及生成的 `.bin` / `.elf` 路径。
🔴 `set-target` 会**清掉已有 build 配置**并从 `sdkconfig.defaults` 重新生成 `sdkconfig`（16MB Flash / Octal PSRAM / 1000Hz tick 在这一步生效），只在第一次做。

> `sdkconfig.defaults` 里的 `CONFIG_SPIRAM_SPEED_80M` 等若在 v5.5.5 里被改名，build 会报 Kconfig 警告/未知项 ⇒ **把原始报错整段贴回来**，按版本订正，别硬凑。

#### E. 烧录 + 看串口（要接板，最后才做）

```powershell
idf.py -p COM7 flash monitor
```

`COM7` 换成设备管理器里 CH343 的**实际**串口号（每次重插重查）。退出 monitor 按 `Ctrl+]`。

**判据**（🔴 **11-14 晚固件已改，判据跟着改**，完整四层见 `docs/固件与ESP32开发.md` 第五、七节）：
- `I (xxx) voice_main: ==== ESP32-S3 语音入口骨架启动（含 TCP 上行自检） ====`
- `[本地封帧 0] $ESTOP*5D`、`[本地封帧 1] $RESUME*19`（**不再是** `$ASR,mic_ok*50`）
- `TCP connected -> <鲲鹏IP>:9000`、`网关应答: $OK,ESTOP*75` / `$OK,RESUMED*75`
- `中文帧默认关闭（SEND_CN_HOME_FRAME=0），明天联调再开`
- 每秒打印 I2S 采样值

> 🔴 看到应答帧只证明**「ESP32 → Wi-Fi → 鲲鹏网关 → 应答」链路通了**，**不证明语音控制**：发的还是写死的自检帧、没接 WakeNet / ASR、没让臂动。
> 旧骨架那帧 `$ASR,mic_ok*50` 发到网关会被判 `$ERR,2*5B`（词表只有中文、语义层未识别），11-14 实证见 `docs/固件与ESP32开发.md` 3.3。

---

### 四、判据表（分层，不许跳级打勾）

| 层 | 判据 | 档位变化 |
|---|---|---|
| 1 环境 | A1 窗口里 `idf.py --version` → `ESP-IDF v5.5.5` 且无 `IDF_PYTHON_ENV_PATH` 警告 | 环境就绪，**项目档位不变**（本轮已核对到版本号，待你在自己窗口复验） |
| 2 编译 | `idf.py build` → `Project build complete` | `esp32/` 仍是 🔶（有代码未实机），只是「能编译」 |
| 3 板上 | `flash monitor` 见启动横幅 + 本地封帧 `$ESTOP*5D`/`$RESUME*19` + `TCP connected` + PCM 打印 | 骨架可算「上板跑通」，但**语音入口整体仍 🔶** |
| 4 联调 | ESP32 真发 `$ESTOP*5D` / `$RESUME*19` 到鲲鹏网关并收到 `$OK,ESTOP*75`/`$OK,RESUMED*75`（四层硬判据） | 🔶 **代码已落盘**（`tcp_client.c/.h`，11-14 晚本机实证 `PASS=24 FAIL=0`），**待明天上板联调** → `docs/固件与ESP32开发.md` |
| 5 语义 | `$ASR,回初始位*61` → `$OK,HOME*27` → 臂回初始位 | 📋 待做 |

🔴 **能 build ≠ 完成**：第 2 层「能编译」不等于「能演示」，第 5 层才谈得上「语音→动作」。按路线图第七节，**ESP-SR 放最后**。

---

### 五、失败分流

| 现象 | 先查什么 |
|---|---|
| `MSys/Mingw is no longer supported` | 🔴 在 **Git Bash** 里跑了 ⇒ 换 **ESP-IDF PowerShell / cmd** 窗口 |
| `WARNING: IDF_PYTHON_ENV_PATH is missing` | 裸调了 `python.exe tools\idf.py` ⇒ 走 A1/A2 初始化脚本，别直接点 python |
| `idf.py` 不是 cmdlet / 找不到命令 | 没在 ESP-IDF 快捷方式窗口里；`Initialize-Idf.ps1` 没 dot-source |
| CMake / Ninja 报路径含奇怪字符、找不到源文件 | 🔴 **中文工程路径没绕开** ⇒ 确认当前目录是 `D:\esp\s3_voice` |
| `set-target` 报 Kconfig 未知项 | `sdkconfig.defaults` 的 `CONFIG_SPIRAM*` 命名差异 ⇒ 贴原始报错，按 v5.5.5 实际 Kconfig 订正 |
| 热点没起来 | `main/wifi_ap.h` 里 `AP_SSID` / `AP_PASSWORD` 配错（当前 `Kunpeng-Robot` / `12345678`）⇒ 改后重新 build；鲲鹏以 STA 接入该热点 |
| `flash` 找不到端口 | 装 CH343 驱动；`-p COMx` 换设备管理器实际值；换一根**数据线**（很多线只能充电） |
| monitor 全乱码 | 晶振 26MHz vs 40MHz（官方 hello_world 文档有此坑）；或波特率不匹配 |
| I2S 读出来全 0 | 先查 INMP441 的 L/R 引脚（接 GND = 左时隙），别先怀疑代码 |

---

### 六、诚实边界（面试口径不变）

- 🔴 **未安装**（本来就不用）、**已烧录实机验证**、**未改任何 `.ioc`**、未改 PC 上 ESP-IDF 配置。11-14 晚**改了 `esp32/` 源码**（新增 `tcp_client.c/.h`、覆写 `main.c` / `CMakeLists.txt`、订正两个头的注释）并在 PC 上跑通整机 `idf.py build`（`BUILD_RC=0` / `Project build complete` / 产物齐全，回读见清单第一节）。
- 🔴 `esp32/` 仍是 🔶 **已烧录实机验证**；支柱② 语音入口整体仍 🔶，阶段 2 鲲鹏侧网关链路的 ✅ 保持原状。
- 🔴 禁写「已接 ESP32 语音链路 / 语音控制已实现 / 语音→动作闭环」；只叫「可观测性闭环 / 视觉引导的移动操作平台」。
- ✅ 能写：「ESP-IDF v5.5.5 环境已在 PC 就位；`protocol.c` 本机 gcc 编译并与鲲鹏侧逐字节一致；`esp32/` 骨架已补 Wi-Fi TCP 上行层并在 PC 上完成首次整机编译（已烧录实机验证）」。

---

### 七、README 三行法

- **① 用了什么语法 / 技术**：`Initialize-Idf.ps1 -IdfId` + `export.ps1`（dot-source 注入工具链 PATH 并把 `idf.py` 定义成 PowerShell 函数）；`idf.py set-target` / `build` / `-p PORT flash monitor` 四段式；PowerShell `New-Item` + `Copy-Item -Recurse` 做 ASCII 路径副本；`sdkconfig.defaults` 声明 16MB Flash / Octal PSRAM / 1000Hz tick。
- **② 为什么这么设计**：编译工具链在 PC（USB 烧录要摸到 CH343 串口），鲲鹏板只跑运行时 ⇒ 两边职责分开，不在 ARM64 板上重复装工具链；官方构建系统不支持路径含空格与非 ASCII，而工程路径带中文 ⇒ 用「拷到 `D:\esp\s3_voice` 的副本」把**环境约束**和**源码真源**分开，原工程一行不动，与 A 项合并纪律一致；v5.5.5 正好是骨架按 v5.x API 写、ESP-SR 依赖下限 5.0 的公约数。
- **③ 不这么写会怎样**：在 Git Bash 里跑 ⇒ 官方直接拒 `MSys/Mingw is no longer supported`；裸调 `python.exe tools\idf.py` ⇒ `IDF_PYTHON_ENV_PATH` 警告、build 缺编译器；直接在中文路径 build ⇒ CMake / Ninja / Python 路径解析失败；把 ESP-IDF 装到鲲鹏板 ⇒ 编译完也没那根 USB 烧录线；把 `flash` 的 `COM7` 写死不查 ⇒ 找不到端口还以为板子坏了。

---

### 附录 A · 换电脑 / 重装时才用（本轮**不需要**）

| 选项 | 做法 | 体积 | 联网 |
|---|---|---|---|
| A | 官方 Windows 离线安装包 **v5.5.x** | ≈1.51 GB | 装完不用网 |
| B | 在线安装包 / **EIM**（ESP-IDF Installation Manager，v6.0+ 官方新方式；你现有环境就是这个风格） | 在线包 ≈4 MB | 必须联网 |
| C | VS Code + ESP-IDF 扩展，或手动 `git clone` + `install.bat` | 看情况 | 必须联网，新手不推荐 |

- 官方下载页 <https://dl.espressif.com/dl/esp-idf/>，只从乐鑫服务器下（官方每个包发 SHA-256）。
- 🔴 别装 v6.x：骨架用 `CONFIG_SPIRAM*` / `i2s_std` 等 v5.x API，v6.0 起安装方式与 API 都有变动。
- 🔴 新路径仍要满足：≤90 字符、无空格 / 括号 / 非 ASCII（你现在的 `D:\ESP32\Espressif` 就是范本）。
- ESP-SR 组件清单声明 IDF 依赖下限 **5.0**；社区同款硬件（S3 N16R8 + INMP441 + MAX98357A）实跑组合是 **IDF v5.5.4 + WakeNet9** ⇒ 5.5.x 也覆盖将来进 ESP-SR。

---

## 二、ESP32-S3 接入与阶段收尾路线

> 来源：`固件与ESP32开发.md`

> 🔴 **历史指令（其中“4 条判据 / 第二路 USB-TTL”已作废）**：「补齐全部收尾项，跑 4 条实机判据，鲲鹏第二路 USB-TTL 实发帧验统一调度，**小车烧录先放到最后，先接入 ESP-S3**」。
> 本文是**路线 + 规程 + 本轮实证**，**不是实机结果**。三档标注 ✅实机 / 🔶有代码未实机 / 📋规划 一律不抬。
> 关联：`docs/00-项目总览与硬数据速查.md` 第六节、`docs/固件与ESP32开发.md` 第七节、`docs/KunPeng部署与语音网关上板验收.md`。

---

### 零、先说清一件事：你给的清单里有一个依赖矛盾

| 你列的事 | 真正的前置条件 | 现在能不能跑 |
|---|---|---|
| 跑「底盘最小判据」（现行见第五节） | 先把 `MDK-ARM/RoboticArm_Kunpeng.uvprojx` 烧进 **F407VE**（全项目唯一板） | ✅ Rebuild 0/0 通过，未烧录实机 |
| 鲲鹏第二路 USB-TTL 统一调度 | **旧双板方案已随 `car/` 删除**；现在同一块 F407VE 共用 USART1，不再需要第二路 | 🚫 作废 |
| **先接入 ESP32-S3** | 鲲鹏侧网关 ✅ 已上板实机验证（16:40）；成品板在手 | ✅ **现在就能推进** |
| ACS712 + J6 限力夹持 | 见独立方案文档 | 📋 方案阶段 |

🔴 **结论**：旧「A 项实机验收包 = 烧 `car/` + 4 条判据 + 第二路 USB-TTL」是**两块 F407 板**的旧方案，已随 `car/` 删除；现行底盘验收见第五节，目标板只有 **F407VE**，USART1 就是统一调试口。旧 `car/` 烧录仍按你的要求放最后，但它现在已不存在，不再有“第二块底盘板”这一步。

---

### 一、上一轮收尾项核对：已全部补齐，无遗漏

| 项 | 状态 |
|---|---|
| 限定抬档：只把「阶段 2 网关链路」🔶→✅，语音入口整体仍 🔶 | ✅ 已完成 |
| 残留旧档位口径订正 9 处（根 README 3 / robot README 4 / 路线图第七节 / 简历第五节+第六节 / 验收规程第八节 / MEMORY.md §8 消歧） | ✅ 已完成 |
| `docs/排障记录-服务端与IoT.md` 补第 8 条正文（板端两个非失败误判点） | ✅ 已完成，现 8 条 |
| 简历第六节证据清单补第 **13** 条实机证据 | ✅ 已完成 |
| 当日日志 `2026-11-14.md` 追加「限定抬档收官」 | ✅ 已完成 |
| 临时文件清理（7 个：`_verify_*` / `_ck_*` / `_wg_audit_*`） | ✅ 已 `CLEAN` |
| 全局复核无残留旧口径、其它模块 🔶 未动 | ✅ Grep 通过 |
| 任务 `#33`~`#37` | ✅ 全部 completed |

---

### 二、新顺序（按你的优先级重排）

| 顺序 | 做什么 | 依赖烧录吗 | 档位 |
|---|---|---|---|
| **1（现在）** | **接入 ESP32-S3**：骨架整机 build 已通过 → 烧录 → TCP 上行鲲鹏 → 帧草案实机对齐 | ❌ | ✅ 已烧录实机（TCP 9000 + 采音） |
| 2 | 阶段 0~1：ESP32 真发帧钉死「帧从板子发出来了」 | ❌ | 📋 |
| 3 | 最后才进 ESP-SR 命令词（唯一有技术不确定性的一段） | ❌ | 📋 |
| 4 | IoT 补 ESP32 节点遥测 + eKuiper 告警规则 | ❌ | 🔶 |
| 5 | ACS712 / J6 限力夹持（📋 方案已出，独立于主线） | ⚠️ 要接线 + 标定 | 📋 |
| **6（最后）** | **底盘实机验收**：Rebuild `RoboticArm_Kunpeng.uvprojx` → ST-LINK 烧 **F407VE** → 跑第五节最小判据 | ✅ **必须烧** | 🔶 Rebuild 0/0 通过（Code=24400），未烧录实机 |

---

### 三、ESP32 接入：本轮本机实证（真跑，不是推断）

#### 3.1 概念先讲清（大白话 + 规范定义）

| 概念 | 大白话 | 规范定义 |
|---|---|---|
| TCP 上行 | ESP32 像「打电话」一样连到鲲鹏的一个端口，把一句话发过去，对方回一句 | 传输控制协议（RFC 9293），面向连接、可靠字节流；本项目用 BSD socket `socket()/connect()/send()/recv()` |
| STA 模式 | ESP32 像手机一样去连路由器，不是自己开热点 | IEEE 802.11 infrastructure mode 的 station；连上后由 DHCP 拿 IPv4（`IP_EVENT_STA_GOT_IP`） |
| 帧草案对齐 | 两边说好的「一句话的格式」必须一模一样，否则对不上暗号 | 项目统一帧 `$<CMD>[,<ARG>...]*<CK>\r\n`，CK = payload 逐 **UTF-8 字节** XOR、2 位大写 HEX（真源 `Core/Src/uart_protocol.c`） |
| `--host 0.0.0.0` | 服务默认只许本机自己连；要让别的机器连，得把门开到所有网卡 | `bind()` 到 `INADDR_ANY`；默认 `127.0.0.1` 是 loopback，外部主机连接会被拒 |

#### 3.2 实证 ①：ESP32 侧 C 封帧 vs 鲲鹏侧 Python 帧，**逐字节 5/5 一致**

本机用 `gcc -std=c99 -Wall -Wextra` 编译 `esp32/main/protocol.c`（它只依赖 `<stdint.h>/<stddef.h>`），把 C 侧 `protocol_build_frame()` 的输出与 Python 侧 `text_gateway.xor_checksum()` 逐字节比对：

| payload | CK | C 侧帧 | Python 侧帧 | 结果 |
|---|---|---|---|---|
| `ASR,mic_ok` | `50` | `$ASR,mic_ok*50\r\n` | 同 | ✅ PASS |
| `ASR,home` | `63` | `$ASR,home*63\r\n` | 同 | ✅ PASS |
| `ASR,回初始位` | `61` | `$ASR,\xe5\x9b\x9e...*61\r\n` | 同（含 UTF-8 多字节） | ✅ PASS |
| `ESTOP` | `5D` | `$ESTOP*5D\r\n` | 同 | ✅ PASS |
| `RESUME` | `19` | `$RESUME*19\r\n` | 同 | ✅ PASS |

✅ **结论 1**：骨架里 `protocol.c` 的封帧实现**语法层完全正确**，中文 UTF-8 逐字节 XOR 也与 Python 对齐（这正是 `docs/排障记录-服务端与IoT.md` 第 2 条一直在防的坑）。**帧草案的「格式」不用改。**

#### 3.3 实证 ②：语义层有坑 —— 这是接入最先会撞上的

起真 `voice_gateway` TCP server（`apply=False` dry-run），用 socket 发上面 5 条帧：

| 发出的帧 | 收到的应答 | 说明 |
|---|---|---|
| `$ASR,mic_ok*50\r\n` | **`$ERR,2*5B\r\n`** | 🔴 **骨架 `main.c` 现在打印的自检帧**，接上去会被判「未识别」 |
| `$ASR,home*63\r\n` | **`$ERR,2*5B\r\n`** | 🔴 HOME 词表**只有中文**（`回初始位/回到初始/归位/复位/收起`），英文 home 不行 |
| `$ASR,回初始位*61\r\n` | `$OK,HOME*27\r\n` | ✅ 中文帧能驱动（16:40 已实机验过） |
| `$ESTOP*5D\r\n` | `$OK,ESTOP*75\r\n` | ✅ 纯 ASCII 快速通道 |
| `$RESUME*19\r\n` | `$OK,RESUMED*75\r\n` | ✅ 纯 ASCII 快速通道 |

🔴 **结论 2**：ESP32 侧要发「能驱动臂的帧」，**必须支持 UTF-8 中文字节串**；或者**先用 `$ESTOP/$RESUME` 两条纯 ASCII 帧把链路打通**（不依赖中文编码，避开 SecureCRT / 串口 monitor 的乱码坑）。
⇒ 这就是「先链路、后语义」的分层接入顺序，别一上来就怼中文。

#### 3.4 实证 ③：一个静态配置事实

`voice_gateway.py:137` → `DEFAULT_HOST = "127.0.0.1"`（注释原文：「默认只本机可连；要 ESP32 进来须显式 `--host 0.0.0.0`」）。
⇒ **ESP32 从 Wi-Fi 远程连过来，网关必须带 `--host 0.0.0.0` 启动**，否则连不上。这是接入时最容易漏的一条。

---

### 四、ESP32 接入三步（现在能推进的）

| 步 | 做什么 | 本机能否验 | 档位 |
|---|---|---|---|
| 1 | **用 PC 上现有的 ESP-IDF v5.5.5**（`D:\ESP32\Espressif`）→ `idf.py -p PORT flash monitor`，看串口打印出 `$ESTOP*5D` / `$RESUME*19` 与 PCM 采样 | ✅ 环境已就位（11-14 17:38 核对）；✅ 骨架**整机 `idf.py build` 11-14 晚已通过**（`Project build complete` / 产物齐全），工程路径含中文已拷 ASCII 副本 `D:\esp\s3_voice` | ✅ 已烧录实机（TCP 9000/采音） |
| 2 | 补 TCP 上行；鲲鹏网关 `--host 0.0.0.0`；先发 `$ESTOP*5D` / `$RESUME*19` 验「ESP32 → Wi-Fi → 网关 → 应答」 | ✅ 帧层面本轮已实证 | 🔶 有代码才验 |
| 3 | 再补 UTF-8 中文帧 `$ASR,回初始位*61` → `$OK,HOME*27`，`--apply` 下臂回初始位 | ✅ 16:40 已实证同形状帧 | ✅ ESP32 已接入；`$ASR` 语义层待 ESP-SR 落地 |

#### 4.1 代码清单（✅ **11-14 晚已全部落地，不是清单了**）

> 用户 11-14 晚明确：「在宿舍没硬件，身边只有鲲鹏板，能不能先把相关代码完成，等明天回教室直接烧录 / 上传实测」。
> ⇒ 本轮把「不依赖教室硬件、能本机验证」的部分全部写完并跑实证；**热点 SSID/密码在 `wifi_ap.h` 集中配置，要换只改这一处**（凭据不进 Git、不远程代填）。

| 文件 | 实际补了什么 | 落地状态 |
|---|---|---|
| `esp32/main/tcp_client.c/.h`（新） | BSD socket：`getaddrinfo` → `socket` → `setsockopt(SO_RCVTIMEO/SO_SNDTIMEO)` → `connect(鲲鹏IP, 9000)` → `send()` 循环发整帧 → `recv()` 收应答 → `close()`；**一次一连接**，失败返回分级错误码 `TCP_ERR_*`（不自动重连，见头文件「为什么一次一连接」） | ✅ 已落盘，含「大白话 + 规范定义」双行注释 |
| `esp32/main/main.c`（覆写） | 开 SoftAP 热点后调 `tcp_client_send()`；自检帧换成 **`ESTOP` / `RESUME`** 两条纯 ASCII 快速通道；中文帧 `$ASR,回初始位*61` 用 C 八进制字节串写好、由 `SEND_CN_HOME_FRAME=0` **默认关闭** | ✅ 已落盘；Wi-Fi 没连上也让本地封帧自检 + I2S 自检照跑，便于分层定位 |
| `esp32/main/CMakeLists.txt`（覆写） | `SRCS` 加 `tcp_client.c`；`REQUIRES` 补 `lwip` | ✅ 已落盘 |
| `esp32/main/wifi_ap.h` | `AP_SSID` / `AP_PASSWORD` 集中配置（当前 `Kunpeng-Robot` / `12345678`） | ✅ 已落盘，要换热点只改这里、重新 build |
| `esp32/sdkconfig.defaults` | 一行没动（16MB Flash / Octal PSRAM / 1000Hz 已配好） | ✅ 不动 |

🔴 **不改 `.ioc`、不动 STM32 固件**：ESP32 是独立节点，只经 Wi-Fi TCP 到鲲鹏网关；网关再经 `/api/cmd` → 已验收的 Flask → 串口 → 固件。全程不碰 16:40 已实机验收的链路。
🔴 **中文帧安全闸门**：`SEND_CN_HOME_FRAME=0`。它在网关 `--apply` 下会真的下发 `HOME` 让臂回初始位 ⇒ 明天确认「网关已 `--apply` + 臂已摆 C 姿态 + 周围无人」再改 `1` 重烧。

**本轮真跑过的本机实证（不是推断）**：

1. 用 `gcc -std=c99 -Wall -Wextra` 编 `esp32/main/protocol.c` + harness，C 侧三条帧输出：
   `ESTOP|244553544F502A35440D0A|11`、`RESUME|24524553554D452A31390D0A|12`、
   `CN_HOME|244153522CE59B9EE5889DE5A78BE4BD8D2A36310D0A|22`；
   与 Python 侧 `text_gateway.build_frame()` **逐字节一致**（含 UTF-8 中文，CK `61`）。
2. 用**真** `voice_gateway.GatewayServer`（dry-run）收上面三条帧：`$OK,ESTOP*75` / `$OK,RESUMED*75` / `$OK,HOME*27`；
   负例 `$ASR,mic_ok*50`、`$ASR,home*63` 均回 `$ERR,2*5B`。**`PASS=24 FAIL=0`**。
3. 用 PC 上现成 **ESP-IDF v5.5.5**，在纯 ASCII 副本 `D:\esp\s3_voice` 跑整机
   `activate.py --export --shell bash` → `idf.py --version`（`ESP-IDF v5.5.5`）→ `idf.py set-target esp32s3`（**rc=0**，cmake 配置完成）→ `idf.py build`（**rc=0 / `Project build complete` / 产物齐全**）。
   🔴 绕开 Git Bash 注入 `MSYSTEM` 导致官方拒 `MSys/Mingw is no longer supported` 的做法见 `docs/排障记录-固件与ESP32.md` 第 3 条；
   **整机 build 已回读确认通过**（证据与产物清单见 `docs/固件与ESP32开发.md` 第一节）；✅ 已烧录实机（TCP 9000/采音）；识别/下发仍 🔶。

> 🔴 **订正 11-14 早先的过时口径**：本节原文写「本机没有 ESP-IDF，这些代码写了也编译不了，本轮不落地代码」——
> 已被 17:38 的环境核对与今晚的实际 build **证伪并撤回**。PC 上 `D:\ESP32\Espressif` 就是完整 v5.5.5，
> 只是必须走 ESP-IDF PowerShell / cmd 或干净 env 的 Python 通道，Git Bash 直接跑会被官方拒。
> ✅ **明天回教室直接照跑的单行清单已单独落盘**：`docs/固件与ESP32开发.md`（确认 build 已通过 → 改热点 / 网关 IP → 起鲲鹏网关 `--host 0.0.0.0` → `flash monitor` → 四层硬判据 → 可选中文帧加试 → 记录表 / 失败分流）。
> 🔴 **第一步的指令已单独落盘**：`docs/固件与ESP32开发.md`（**11-14 17:38 已订正为「环境已就位、直接 build」**：PC 上的 `D:\ESP32\Espressif` 就是完整 v5.5.5，不用再装；含启动方式、验版本、build / flash 命令、判据表、失败分流）。
> 🔴 **不是装在鲲鹏板上**：ESP-IDF 是 PC 上的交叉编译/烧录工具链（USB 要摸到板载 CH343）；鲲鹏板只跑 `web_control.py` / `voice_gateway.py` 等运行时。
> 🔴 **中文路径坑（必看）**：安装路径 `D:\ESP32\Espressif` 本身合规，但工程在 `D:\简历\STM32项目\...\esp32\` 含中文 ⇒ **先把 `esp32/` 拷到 `D:\esp\s3_voice` 再 build**（仓库源码不动，只改副本构建）；`esp32/README.md` 第十节已按此订正。
> 🔴 **别在 Git Bash 里跑**：实测 `idf.py` 回 `MSys/Mingw is no longer supported` ⇒ 必须开 **ESP-IDF PowerShell / cmd 快捷方式**。

#### 4.2 鲲鹏侧可粘贴命令（沿用 `docs/KunPeng部署与语音网关上板验收.md` 已实证写法，只多一个 `--host`）

前置：Flask 必须在跑（网关靠 `/api/cmd` 下发）。判活只认 `curl /api/ik/eye` 的 `enabled:true`；启动必用绝对路径 `/root/kunpeng_vision/web_control.py` + `python3 -u`，**不带 `--radial-fix`**。

```bash
# ① 结构自检（零硬件、不碰臂）
python3 -u /root/kunpeng_vision/robot/voice_gateway.py --host 0.0.0.0 --port 9000 --self-test

# ② 先 dry-run 起网关，只验「帧能不能进来」，不下发
python3 -u /root/kunpeng_vision/robot/voice_gateway.py --host 0.0.0.0 --port 9000 > /root/gw.log 2>&1 &

# ③ 从另一台机器（PC / ESP32 所在网段）探两条纯 ASCII 快速通道
printf '$ESTOP*5D\r\n' | nc 192.168.77.100 9000
printf '$RESUME*19\r\n' | nc 192.168.77.100 9000

# ④ 链路通了再停掉 dry-run，改 --apply 真下发（此时要先摆 C 姿态，见验收规程）
pkill -f voice_gateway.py
python3 -u /root/kunpeng_vision/robot/voice_gateway.py --host 0.0.0.0 --port 9000 --apply > /root/gw.log 2>&1 &
```

🔴 板端命令一律**单行逐条粘**；`printf` 能解释 `\ooo` / `\r` / `\n`，`echo` 默认不解释转义，`nc` 不通时用 Python `socket.create_connection` 一行兜底（写法见验收规程 3.3）。

---

### 五、底盘实机验收包（旧 `car/` 版已作废）

> 🔴 旧 `car/demo.uvprojx`、`car_frame.c`、`$CAR,FWD/BWD/SPIN`、蓝牙 `F/S`、两块 F407 板
> 全部随 `car/` 删除，**本节不再保留旧命令**。现行底盘验收只看
> `docs/底盘-TB6612与JGB37-520-接线、固件与调试.md` 与下面这组最小判据。

#### 5.1 烧录

打开 `MDK-ARM/RoboticArm_Kunpeng.uvprojx`，**Rebuild All**，ST-LINK 下载到 **STM32F407VET6**。
不要找 `car/demo.uvprojx`（已删除）；现行只有底盘（机械臂代码已删），USART1 PA9/PA10 就是统一调试口。

#### 5.2 现行最小判据

| # | 发什么 | 期望 |
|---|---|---|
| 1 | `$PING*10` | 回 `$OK,PONG*3E` |
| 2 | `$CAR,ENC*34` | 手转单轮一圈，计数差约 **3960**（13 线则 4680，改 `CHS_ENC_LINES=13`） |
| 3 | `$CAR,RAW,10,0,0,0*09` | A 轮 10% 正转；其余三轮不动 |
| 4 | `$CAR,WHEEL*2F` | 回 12 个整数（四轮 target/cmd/speed/duty） |
| 5 | `$CAR,VEL,0.1,0,0,1000*0D` | 前进 0.1m/s，约 1s 后自动停 |
| 6 | 行进中 `$CAR,STOP*64` | 下一拍（≤20ms）coast |
| 7 | `$CAR,CLRFAULT*6B` | 仅故障锁存后有效；无故障回 `ERR,5` |

#### 5.3 接线与串口

| 项 | 值 |
|---|---|
| 调试串口 | USART1 **PA9/PA10，115200 8N1**，USB-TTL 的 TX→PA10、RX→PA9、GND 共地 |
| 底盘电源 | TB6612 `VM` 接 **11.2~12.0V**；JGB37-520 是 12V 电机，**不要接旧 6V 电机档** |
| 编码器 | 蓝线 3.3V、黑线 GND；黄/绿 A/B 按 `chs_tb6612.h` 宏接 |
| 串口隔离 | 单块 F407VET6 独占 USART1；不再有“第二路 USB-TTL 给 F407ZE 底盘”的旧架构 |

#### 5.4 验收记录模板（跑完填，别提前打勾）

| # | 判据 | 期望 | 实测 | 通过 |
|---|---|---|---|---|
| 1 | `$PING*10` | `$OK,PONG*3E` | ☐ | ☐ |
| 2 | `$CAR,ENC*34` | 单轮一圈约 3960 | ☐ | ☐ |
| 3 | `$CAR,RAW,10,0,0,0*09` | A 轮正转，其余不动 | ☐ | ☐ |
| 4 | `$CAR,WHEEL*2F` | 12 个整数可解析 | ☐ | ☐ |
| 5 | `$CAR,VEL,0.1,0,0,1000*0D` | 前进约 1s 后停 | ☐ | ☐ |
| 6 | 行进中 `$CAR,STOP*64` | ≤20ms coast | ☐ | ☐ |

🔴 **档位**：底盘代码已落盘、host GCC 语法检查通过；**Keil 编译/烧录/实机未做**。只有上表全绿并回报，才按 `docs/00-项目总览与硬数据速查.md` 统一抬档。

---

### 六、诚实边界 / 命名红线

- ✅ 本轮 11-21 **已烧录实机验证**（SoftAP/TCP 9000/采音）；11-14 晚已在 PC 上跑整机 `set-target esp32s3`（rc=0）+ `idf.py build` **并回读确认通过**（`BUILD_RC=0` / `Project build complete` / 产物 4 件齐全 / `tcp_client.c` 链接成功；首次全量编译曾触发 20 分钟 timeout，增量续跑通过，**不是编译失败**），新增代码全是 🔶 有代码未实机；**不许因「先接入 ESP32 / 编译通过」把语音入口整体抬成 ✅**。
- 🔴 鲲鹏侧阶段 2 网关链路是 ✅，但 `source` 恒 `mock`、**ESP32 已接入但只发链路自检帧**；`$ASR` 帧草案**语义层仍待**（本轮只做到 C/Python 语法层逐字节一致，语义层仍待实机）。
- 🔴 禁写「语音控制已实现 / 语音→动作闭环 / 已接 ESP32 语音链路 / 自然语言控制机械臂」；只叫「可观测性闭环 / 视觉引导的移动操作平台」。
- ✅ 能写：「ESP32-S3 侧的封帧实现与鲲鹏侧网关在逐字节层面一致，并用纯 ASCII 快速通道完成了链路联通准备」。

---

### 七、README 三行法（本轮新增的「帧草案对齐」验证）

- **① 用了什么语法**：`gcc -std=c99 -Wall -Wextra` 把只依赖标准 C 头的 `protocol.c` 编进本机 harness；Python 侧 `xor_checksum()` 逐 UTF-8 字节 XOR；`socket.create_connection()` 起真 TCP 连到 `ThreadingTCPServer`，`recv()` 收应答帧。
- **② 为什么这么设计**：跨语言协议最怕「各自算各自都对、接起来全错」——把 **C 侧真源**编译出真实字节流，再和 Python 侧逐字节比对，比两边各自跑单测硬得多；而且先用纯 ASCII 帧验链路、再验中文语义，能把「编码问题」和「连通问题」分开定位。
- **③ 不这么写会怎样**：只看骨架打印的 `$ASR,mic_ok*50` 就以为「帧做好了」，接上去一律 `$ERR,2`；用英文 `home` 也会被拒；忘了 `--host 0.0.0.0`，ESP32 从 Wi-Fi 连过来直接 `Connection refused`，会被误判成「Wi-Fi 没连上」。

---

## 三、ESP32 与 IoT 核心化路线

> 来源：`固件与ESP32开发.md`

> 定稿：2026-11-13。决策：**ESP32-S3 语音入口与可观测性闭环提升为本项目三大核心支柱之二、之三**。
> 判据：**核心 = 能演示 + 能讲清边界**。三档（✅ 实机 / 🔶 有代码未实机 / 📋 规划）照旧，不许因为"核心"就写成已完成。

### 一、三大支柱与当前真实进度

| # | 支柱 | 载体 | 状态 | 缺口 |
|---|---|---|---|---|
| ① | 视觉引导执行 | STM32F407 + KunPeng Pro | ✅ 实机 | 定位径向 2.9cm 系统偏近（根因已定，修法待落地） |
| ② | **语音入口** | ESP32-S3（N16R8 成品板）+ 鲲鹏侧收帧服务 | 🔶 **整体未完成**（SoftAP/TCP 9000/采音已实机，识别未接）；✅ **鲲鹏侧阶段 2 网关链路 11-14 16:40 已上板实机验证**（38 项单测 + 四层硬判据全绿） | 唤醒、命令词、TCP 上行、接主链路、ESP32 帧草案对齐、ESP32 遥测进 EMQX |
| ③ | **可观测性闭环** | KunPeng Pro 四容器 | ✅ 板内实证 | 未覆盖 ESP32 节点、无告警规则 |

### 二、支柱② 的第一个关键判断：不做通用 ASR，只做命令词识别

**这决定了整个工程量，先说清。**

下游 `kunpeng/llm/intent_parser.py` 的契约是**固定 7 键动作集**（`action / target / qualifier / params / raw` 等），
动作枚举是有限集合。既然下游只需要十几个动作，就**不需要通用语音转文字**。

| 方案 | 做法 | 优点 | 代价 / 风险 | 结论 |
|---|---|---|---|---|
| **A. 端侧命令词识别**（ESP-SR：WakeNet + MultiNet） | ESP32 本地判断"是不是唤醒词 / 是哪条命令" | **全离线、无 Key、延迟 <300ms、演示不依赖网络**；N16R8 的 8MB PSRAM 够 | 词表需预先定义；中文模型要放 flash | ✅ **推荐** |
| B. 音频上行，鲲鹏跑离线 ASR | ESP32 只采音上传，鲲鹏用 Vosk 等 | ESP32 轻、词表可随时改 | 要传 16kHz PCM（约 32KB/s），占带宽；鲲鹏多一个常驻进程 | 备选（A 的中文模型踩坑时启用） |
| C. 云端 ASR API | 音频/文本上云 | 识别率最高 | **需要 Key、依赖网络、演示有失败风险**、与"端侧轻量"叙事矛盾 | ❌ 不用 |

**面试可讲的一句话**：*"因为下游只接受有限动作集，我把语音问题从『通用识别』裁剪成『命令词分类』，用端侧 MultiNet 做，全离线、不依赖云。"*

**命令词表建议**（与 `intent_parser` 的动作键对齐，先 6 条，够演示）：

| 命令词 | 意图 | 后续动作 |
|---|---|---|
| 抓取 / 拿起来 | `pick` | `--hover` 定位后下落 |
| 回原点 | `home` | `JOINT,1580,1900,2400,700,1450,1500` |
| 左移 / 右移 | `move` + `qualifier` | J1 ±Δ |
| 抬高 / 降低 | `z_up` / `z_down` | `--hover` ±Δ |
| 停 | `stop` | 冻结输出（复用 ESTOP 语义） |

### 三、支柱② 分阶段落地（每阶段都有硬验收判据）

> 🔴 **顺序是刻意的：阶段 2 先做，且不依赖 ESP32 硬件。** 先把鲲鹏侧"收帧 → 意图 → 下发"整链打通，
> ESP32 的工作量就从"整套语音识别"降到"发一个 TCP 帧"。
> ✅ **2026-11-14：阶段 2 已完整收口** —— 代码落盘（`voice_gateway.py` + `test_voice_gateway.py`、
> **38 项 unittest 全绿**）＋ **16:40 板端 `--apply` 实机验收四层硬判据全绿**（假帧驱动臂回初始位、
> `PICK` 被拒 `ERR,3`、急停可冻结/恢复、解除急停加固项实机通过）。⇒ **阶段 2 这一条抬 ✅**；
> 🔴 语音入口**整体仍 🔶**（ESP32 整机 build 已通过、已烧录实机（TCP/采音））。原始记录 → `docs/KunPeng部署与语音网关上板验收.md` 第五节。
> 🔴 **判据订正**：原写「下发 `$JOINT`」会一步踩进 `to_pulses()` 符号 / 软限位静默夹紧 / `SAG_DZ` 净空折算
> 四个已定案雷区，改为只下发固件原生 `HOME / STOP / RESUME` 三个**无参数命令字**
> （详见 `docs/排障记录-服务端与IoT.md` 第 5 条）。

| 阶段 | 做什么 | 验收判据（做到才算过） | 依赖 |
|---|---|---|---|
| **0** | `idf.py set-target esp32s3` → `idf.py build` → `idf.py flash monitor` | 串口看到 `读到 256 个采样: x x x x`，**说话时数值变化、安静时接近 0** | ✅ PC 已有 ESP-IDF v5.5.5（`D:\ESP32\Espressif`，见 `docs/固件与ESP32开发.md`） |
| **1** | Wi-Fi SoftAP 热点起来 + TCP client 连鲲鹏 `:9000`（静态 192.168.77.100） | 串口打印 `TCP connected` + `网关应答: $OK,ESTOP*75` / `$OK,RESUMED*75`；鲲鹏 `/root/gw.log` 有对应 `payload` 记录；PC 侧 `nc <鲲鹏IP> 9000` 外部探活同样回 `$OK,...`（四层硬判据见 `docs/固件与ESP32开发.md` 第五节）。🔴 原判据写 `$ASR,mic_ok` 已被 11-14 实证证伪（网关判 `ERR,2`），自检帧已改纯 ASCII 快速通道 | 阶段 0 |
| **2** | 🔴 **鲲鹏侧收帧服务**（`kunpeng/robot/voice_gateway.py`）：TCP server 收帧 → `parse_frame` 校验 → 急停快速通道 → `parse_intent` → **本地仲裁器白名单** → `/api/cmd` 下发固件原生 `HOME/STOP/RESUME` | ✅ **2026-11-14 16:40 上板 `--apply` 实机验收通过**：38 项本机单测全绿 + **四层硬判据全绿** —— 喂一条**假帧** ⇒ 机械臂真的动（`/api/state` 脉宽 C 姿态→初始位 + 肉眼确认）、`PICK` 被拒 `ERR,3`、急停冻结→恢复→再动全对、`解除急停` 加固项实机通过；**链路打通（与语音无关）**。记录见 `docs/KunPeng部署与语音网关上板验收.md` | 无（已完成） |
| **3** | ESP-SR 接入：WakeNet 唤醒 | 说唤醒词，串口打印 `WAKENET: detected`；不说就静默 | 阶段 1、模型文件 |
| **4** | MultiNet 命令词 → 映射到 6 条意图 → 封 `$ASR,<intent>*CK` 发出 | 说"抓取"⇒ 鲲鹏日志收到 `action=pick` ⇒ 机械臂响应；**误识别率当场数出来**（说 20 次、对几次） | 阶段 2、3 |
| **5** | ESP32 遥测进 EMQX（RSSI / 剩余堆 / 唤醒计数 / 识别成功数） | Grafana 新增 ESP32 面板，三条曲线随时间变化 | 阶段 1、MQTT client |
| **6** | eKuiper 告警规则（RSSI < −70dBm、心跳超时 >10s） | 拔网线 / 走远，Grafana 告警面板变红；恢复后自动转绿 | 阶段 5 |

> 🔴 **阶段 0 的路径前置（11-14 17:38 订正）**：ESP-IDF 环境**已就位、不用装**（PC `D:\ESP32\Espressif` = v5.5.5）；但工程目录 `D:\简历\STM32项目\...\esp32\` 含中文，官方构建系统明确不支持 ⇒ **先拷到 `D:\esp\s3_voice` 再 build**（仓库源码不动，只改副本构建）。必须从 **ESP-IDF PowerShell 快捷方式**启动，别用 Git Bash。完整指令 → `docs/固件与ESP32开发.md`。

**工作量粗估**：阶段 0~2 约 1 天（其中阶段 2 半天、可先行）；阶段 3~4 约 2~3 天（模型与 API 版本是主要不确定项）；阶段 5~6 约 1 天。

### 四、支柱③ 从"能存"升级到"能看能报"

已实证的部分不动（四容器 + 六路曲线）。核心化要补三笔欠账：

| 欠账 | 做法 | 验收判据 |
|---|---|---|
| 未覆盖 ESP32 节点 | ESP32 用 paho 风格 MQTT client 发 `device/esp32_01/telemetry`，字段 `rssi / heap_free / wake_cnt / cmd_ok` | Grafana 出现第二条设备线，字段与 InfluxDB measurement 对齐 |
| 无告警 | eKuiper 规则：`SELECT * FROM mqtt_stream WHERE rssi < -70` → 写入 `alerts` measurement | 人为走远，`alerts` 出现记录、Grafana 告警面板变红 |
| 无"健康度"视图 | Grafana 面板：三节点在线状态 + 六路脉宽 + 识别成功率 | 一屏能看到"端①在动、端②在听、边在算" |

> 🔴 口径红线不变：这是**板内可观测性闭环**，不是云端 IoT、不做设备影子、不加抓取能力。

### 五、三支柱联动后的完整演示脚本（目标形态）

```
① 对 ESP32-S3 说唤醒词 → "抓取"
② ESP32-S3 封帧 $ASR,pick*CK → Wi-Fi TCP → 鲲鹏 :9000
③ 鲲鹏 parse_frame 校验 → parse_intent → 判「需要先定位」
④ 鲲鹏 eye_check 定位棋盘 → ArmIK 解 6 路 → 串口 $JOINT → STM32 → PCA9685
⑤ STATE 回读六路脉宽 → 同时经 iot/ 进 EMQX → Grafana 曲线跳动
⑥ 若识别失败/网络断：Grafana 告警面板变红（这条比"成功"更能说明系统完整）
```

**这条演示线的价值**：同一套帧格式贯穿四端（STM32 / 鲲鹏 / C++ 引擎 / ESP32-S3），
同一份时序库同时看到"执行"与"感知"两个维度 —— 这是把三根支柱连成一个系统、而不是三个 demo 拼盘的关键。

### 六、风险与红线（写简历前必看）

| 风险 | 说明 | 应对 |
|---|---|---|
| 🔴 ESP-SR 版本/模型不确定 | 中文命令词模型在不同 ESP-SR 版本里 API 差异大（v1.x 与 v2.x 不同），首次接入可能卡住 | 以所装版本头文件为准；卡超过 1 天就切方案 B（音频上行 + 鲲鹏 ASR） |
| 🔴 演示环境噪声 | 命令词识别在嘈杂环境误触发 | 现场先演示"唤醒 + 单条命令"，别追求连续多轮 |
| 🔴 核心化＝简历必须能演 | 若面试时 ESP32 还停在骨架，会被直接问穿 | **在阶段 4 通过之前，简历一律写「已搭建骨架，识别与上行在办」**，不许写"语音控制已实现" |
| ⚠️ 时间冲突 | 阶段 0~2 只需 1 天，阶段 3~4 才是深水区 | 先拿阶段 0~2 的成果（链路通）保底，再谈识别率 |

### 七、下一步（按顺序，不要跳跃）

> 🔴 **2026-11-24 底盘口径**：旧 `car/`（L9110S + F407ZE）已删除；现行只有**单块 F407VE**
> （底盘独占，机械臂代码已删），不再有“第二块底盘板 + 第二路 USB-TTL”。底盘验收见第五节。

1. ✅ **阶段 2 已完成并封板（11-14 16:40）**：`voice_gateway.py` + 38 项单测全绿 + **板端 `--apply` 四层硬判据全绿**（喂假帧 ⇒ 臂真的动、`PICK` 被拒 `ERR,3`、急停冻结/恢复、`解除急停` 加固项实机通过），阶段 2 网关链路档位 🔶→✅（只抬这一条）。👉 实跑记录表见 `docs/KunPeng部署与语音网关上板验收.md`。🔴 **语音入口整体仍 🔶**。
2. **（现在优先）接入 ESP32-S3**：骨架 `idf.py build` → TCP 上行 → 帧草案实机对齐。**本轮本机实证**：C 侧 `protocol.c` 封帧与 Python 侧逐字节 **5/5 一致**（含 UTF-8 中文）；但骨架自检帧 `$ASR,mic_ok*50` 与 `$ASR,home*63` 会被判 `$ERR,2` ⇒ 先用 `$ESTOP*5D`/`$RESUME*19` 纯 ASCII 快速通道验链路，且网关须 `--host 0.0.0.0`。详见 `docs/固件与ESP32开发.md` 第三、四节。🔴 **不用装 ESP-IDF**：11-14 17:38 核对确认 PC 上已有完整 **v5.5.5**（`D:\ESP32\Espressif`，EIM 安装，纯 ASCII 路径合规），直接开 **ESP-IDF PowerShell 快捷方式**即可；**别在 Git Bash 里跑**（实测被官方拒 `MSys/Mingw is no longer supported`）。唯一前置是**中文工程路径** ⇒ 先拷到 `D:\esp\s3_voice` 再 build（11-14 晚已照此跑通整机 build，`BUILD_RC=0`）。完整核对 / 指令 → `docs/固件与ESP32开发.md`。
   ✅ **11-14 晚已落地**：`esp32/main/tcp_client.c/.h`（新）+ 覆写 `main.c` / `CMakeLists.txt`（自检帧改 `$ESTOP/$RESUME`、中文帧 `SEND_CN_HOME_FRAME=0` 安全闸门、`REQUIRES lwip`）；本机帧/网关语义实证 **`PASS=24 FAIL=0`**；PC 上 `set-target esp32s3`（**rc=0**）+ 整机 `idf.py build` **并回读确认通过**（**`BUILD_RC=0` / `Project build complete` / 产物 4 件齐全 / `tcp_client.c` 链接成功**，证据见 `docs/固件与ESP32开发.md` 第一节）。✅ 11-21 已烧录实机验证（SoftAP/TCP 9000/采音）；语音识别仍未接，语音入口整体仍 🔶。；明天照跑清单独成一份 → `docs/固件与ESP32开发.md`。
3. 阶段 0~1 钉死「帧真的从 ESP32 发出来了」→ **最后才进 ESP-SR**（阶段 3~4，唯一有技术不确定性的一段）。
4. 阶段 5~6：ESP32 遥测进 EMQX + eKuiper 告警规则，让支柱 ③ 覆盖三节点。
5. ⚠️ **ACS712 限力夹持（历史，已废弃）**：机械臂代码已于 2026-11-24 删除，`docs/末端执行器-舵机标定与限力夹持.md` 亦已删除。
6. 🔴 **（最后）底盘实机验收**：Rebuild `MDK-ARM/RoboticArm_Kunpeng.uvprojx` → ST-LINK 烧 **F407VE** → 跑第五节的 `$PING/$CAR,ENC/$CAR,RAW/$CAR,WHEEL/$CAR,VEL/$CAR,STOP`。不再有旧 `car/` 4 条判据，也没有第二路 USB-TTL。

---

## 四、照跑清单（11-14 版，命令仍可用）

> 来源：`固件与ESP32开发.md`

> 🔴 **本文是「明天照着跑」的规程，不是实机结果**。三档 ✅实机 / 🔶有代码未实机 / 📋规划 一律不抬。
> ✅ **11-14 晚补记**：整机 `idf.py build` 已在 PC 上跑完并**回读确认通过**（`BUILD_RC=0` / `Project build complete` / 产物齐全，见第一节）；这仍是**编译通过**，不是实机 ⇒ 档位仍 🔶。
> 用户处境：11-14 晚在宿舍，**硬件都在教室**，身边只有鲲鹏开发板；要求「先把相关代码完成，明天回教室直接烧录 / 上传实测」。
> 关联：`docs/固件与ESP32开发.md`（帧实证 + 路线）、`docs/固件与ESP32开发.md`（环境核对）、
> `docs/KunPeng部署与语音网关上板验收.md`（鲲鹏侧网关已实机验收的完整命令/CK 表）、`esp32/README.md`（组件口径与三行法）。

---

### 零、一句话说清「今晚做了什么 / 没做什么」

| | 内容 | 档位 |
|---|---|---|
| ✅ 11-14 晚做完 | `esp32/main/tcp_client.c/.h`（新）、`main.c`（覆写）、`CMakeLists.txt`（覆写）落盘；本机帧/网关语义实证 **`PASS=24 FAIL=0`**；PC 上用现成 ESP-IDF v5.5.5 在 ASCII 副本跑 `set-target esp32s3`（**rc=0**）+ 整机 `idf.py build` **通过**（`Project build complete`、`tcp_client.c` 编译链接成功、产物 4 件齐全） | ✅ 11-21 已烧录实机（TCP 9000 + 采音） |
| 🔴 11-14 晚没做 | 改 SoftAP SSID/密码（如需）、ESP-SR 命令词、IoT 节点遥测、旧 `car/` 烧录（**2026-11-23 作废，现行底盘验收见第五节**）；烧录与上板联调 **11-21 已补通**（TCP 9000 + 采音） | 📋 / 🔶 顺延 |
| 🔴 不许写 | 「ESP32 语音链路已接」「语音控制已实现」「语音→动作闭环」 | 命名红线 |

**核心边界**：今晚写的是「**ESP32 → Wi-Fi → 鲲鹏网关 → 应答**」这段链路自检代码，
发的只是 `$ESTOP*5D` / `$RESUME*19` 两条**纯 ASCII 快速通道**帧 —— 网关 dry-run 也回 `$OK`，
**默认固件不会让机械臂动**。要动臂必须额外开中文帧闸门（第六节），那是可选加试。

---

### 一、整机 build 结论（11-14 晚已回读确认通过，明早不用再等）

今晚的 build 在后台跑完，日志在 PC 的 **`D:\esp\`**（纯 ASCII 副本，不是中文工程目录）。
**已回读 `D:\esp\idf_build2.log` / `D:\esp\_build2_console.log` 末尾，确认：**

| 证据 | 实测值 |
|---|---|
| `EXPORT` | `rc=0`、`EXPORTED_VARS=10`（`activate.py --export --shell bash` 干净 env 生效） |
| `BUILD` | `rc=0` / `BUILD_RC=0` / `FINAL_RC=0` / `BUILD2_SCRIPT_RC=0` |
| 结论行 | `Project build complete. To flash, run: idf.py flash` |
| 新代码 | `[75/96] ... tcp_client.c.obj` → `Linking CXX executable kunpeng_voice.elf` → **无** `undefined reference to getaddrinfo` ⇒ `REQUIRES lwip` 生效 |
| 产物（已核实落盘） | `kunpeng_voice.bin` **803648 B**（0xc4340，app 分区剩 23%）、`kunpeng_voice.elf` 9035736 B、`bootloader/bootloader.bin` 20832 B、`partition_table/partition-table.bin` 3072 B、`flash_args` 155 B |

> 🔴 这只是**交叉编译通过**，不是实机：已烧录实机（TCP 9000/采音）⇒ **识别未接仍 🔶**，不许写「已接 ESP32 语音链路 / 语音控制已实现」。
> 首次全量编译曾触发 20 分钟 `timeout`（`BUILD_TIMEOUT` / `FINAL_RC=6`，Ninja 已到 `[857/1082]`）——**不是编译失败**；用不设 timeout 的增量 build 续跑即通过，排障见 `docs/排障记录-固件与ESP32.md` 第 5 条。

**明早照跑时若要复查 / 环境变了要重跑**（PS 5.1 无 `&&`，单条复制也能跑）：

```powershell
Get-Content D:\esp\idf_build2.log -Tail 25
Get-Content D:\esp\_build2_console.log -Tail 8
Get-ChildItem D:\esp\s3_voice\build\*.bin, D:\esp\s3_voice\build\*.elf -ErrorAction SilentlyContinue | Select-Object Name,Length
```

| 看到什么 | 判定 | 下一步 |
|---|---|---|
| `BUILD_RC=0` + `FINAL_RC=0` + `Project build complete`，且有 `kunpeng_voice.bin` / `.elf` | ✅ 整机编译通过 | 直接跳到第二节 |
| `BUILD_RC≠0` 或日志停在 `===== BUILD =====` 没有回写 | 🔴 编译没过 / 没跑完 | **先别烧**，把日志末尾报错贴回来，按报错改 `esp32/` 源码 → 重新拷副本 → 重跑 `idf.py build` |
| 报 `undefined reference to getaddrinfo` 之类 | 组件依赖没生效 | 核对 `main/CMakeLists.txt` 的 `REQUIRES` 里有 `lwip` |

---

### 二、A 准备（PC 侧，约 5 分钟）

#### 2.1 重新同步 ASCII 副本（🔴 必做，别拿昨晚的旧副本直接烧）

今晚副本是 `D:\esp\s3_voice`。如果第一节里改过 `esp32/` 源码，**必须重拷一次**，否则烧的是旧代码：

```powershell
New-Item -ItemType Directory -Force -Path D:\esp | Out-Null
Copy-Item -Path "D:\简历\STM32项目\4.ESP32-S3项目\RoboticArm_Kunpeng\esp32" -Destination "D:\esp\s3_voice" -Recurse -Force
Set-Location D:\esp\s3_voice
```

> 🔴 官方构建系统**不支持路径含中文 / 空格**，所以工程只能在这份 ASCII 副本里 build；
> 原工程 `RoboticArm_Kunpeng\esp32\` 一行不动。改动只在副本里做，改完记得**回填原工程**（否则下次同步会丢）。

#### 2.2 确认热点参数（🔴 ESP32 自己开热点，不再改 STA 密码）

ESP32 固件已改为 **SoftAP 热点**，热点参数集中在 `D:\esp\s3_voice\main\wifi_ap.h`：

```c
#define AP_SSID       "Kunpeng-Robot"
#define AP_PASSWORD   "12345678"
```

**手机和鲲鹏板作为 STA 连入这个 ESP32 热点**（`Kunpeng-Robot`，密码 `12345678`），ESP32 固定 `192.168.77.1`，不再依赖任何动态 IP；要换名/密码只改这两行、重新 build。

#### 2.3 鲲鹏侧固定地址（🔴 已改静态 STA，不用再查动态 IP）

鲲鹏作为 STA 以**静态 `192.168.77.100`** 接入 ESP32 热点（`.100` 避开 AP 的 DHCP 池 `.2` 起发）。ESP32 侧 `D:\esp\s3_voice\main\tcp_client.h` 已写死：

```c
#define GATEWAY_HOST   "192.168.77.100"
#define GATEWAY_PORT   9000
```

等 ESP32 热点起来后，在鲲鹏执行 `nmcli con up esp-ap`（预置连接：SSID `Kunpeng-Robot`、静态 `.100`），从 `192.168.77.100` 重连即可；`10.160.120.1` 是 USB 共享网络时期的临时地址，不是产品入口。

> ⚠️ **网段迁移（11-21）**：ESP32 热点已从官方默认 `192.168.4.1/24` 改为唯一网段 **`192.168.77.1/24`**，避免与教室/家用路由器撞网。板上 `esp-ap` 若仍停在旧地址 `192.168.4.100`，需在板本机（键盘/显示器或串口 console）执行一次：
> ```bash
> nmcli con modify esp-ap ipv4.addresses 192.168.77.100/24 ipv4.gateway 192.168.77.1 ipv4.method manual
> nmcli con modify esp-ap connection.autoconnect yes
> nmcli con up esp-ap
> hostname -I   # 应出现 192.168.77.100
> ```
> 🔴 **顺序不能反**：先重烧 ESP32（让热点变成 `.77.1`），再到板上改静态 IP；反了板子会连不上新热点。

#### 2.4 查烧录串口号

PC 设备管理器 → 端口（COM 和 LPT），找 **CH343** 对应的 `COMx`，记下来（下面用 `COM7` 举例，实际按你机器写死、每次重插重查）。

---

### 三、B 鲲鹏侧起服务（板端命令一律**单行逐条粘**）

> 前置：机械臂固件已上电、串口线接好。**这些命令不动臂**（网关默认 dry-run）。

**B1 · 确认 Flask 在跑**（网关靠 `/api/cmd` 下发，判活只认 `enabled:true`；🔴 启动必用绝对路径 + `python3 -u`，**不带 `--radial-fix`**）

```bash
curl -s -X POST http://127.0.0.1:5000/api/ik/eye -H "Content-Type: application/json" -d '{}'
```

没起就先起（**5000 只能一个 Flask**，先 `pkill -f web_control.py`）：

```bash
pkill -f web_control.py
python3 -u /root/kunpeng_vision/web_control.py --host 0.0.0.0 --port 5000 --eye-in-hand > /root/web.log 2>&1 &
```

**B2 · 网关结构自检（零硬件、不碰臂）**

```bash
python3 -u /root/kunpeng_vision/robot/voice_gateway.py --host 0.0.0.0 --port 9000 --self-test
```

**B3 · 起网关（先 dry-run，只验「帧能不能进来」）**

```bash
pkill -f voice_gateway.py
python3 -u /root/kunpeng_vision/robot/voice_gateway.py --host 0.0.0.0 --port 9000 > /root/gw.log 2>&1 &
```

🔴 **`--host 0.0.0.0` 不能省**：`voice_gateway.py` 默认 bind `127.0.0.1`，ESP32 从 Wi-Fi 连过来会 `Connection refused`。

**B4 · 从 PC 先探一枪，确认门确实开在网卡上**（IP 固定为 192.168.77.100）

```bash
printf '$ESTOP*5D\r\n' | nc 192.168.77.100 9000
printf '$RESUME*19\r\n' | nc 192.168.77.100 9000
```

期望分别回 `$OK,ESTOP*75`、`$OK,RESUMED*75`。`nc` 不通时用 Python socket 一行兜底：

```bash
python3 -c 'import socket;s=socket.create_connection(("192.168.77.100",9000));s.sendall(b"$ESTOP*5D\r\n");print(repr(s.recv(256)));s.close()'
```

> `printf` 能解释 `\ooo` / `\r` / `\n`；**别换成 `echo`**（默认不解释转义，CK 会错）。

**B5 · 9001 音频上行验收（11-21 新增，独立于 9000）**

> `audio_tx` 只要 SoftAP 起来（`net_ok`）就会连 `192.168.77.100:9001`，**不依赖 9000 网关是否通**；两个进程监听不同端口，可并存。

先在 **PC** 工程根上传接收端（相对路径）：

```powershell
Set-Location "D:\简历\STM32项目\4.ESP32-S3项目\RoboticArm_Kunpeng\kunpeng"
scp "robot\audio_sink.py" root@192.168.77.100:/root/kunpeng_vision/robot/
```

板端放行 9001 + 结构自检（零硬件、不占端口）：

```bash
firewall-cmd --permanent --add-port=9001/tcp && firewall-cmd --reload
python3 -u /root/kunpeng_vision/robot/audio_sink.py --self-test
```

起接收端（120 秒自动收尾，**退出时才把 WAV 头写完整**）：

```bash
pkill -f audio_sink.py
python3 -u /root/kunpeng_vision/robot/audio_sink.py --out /root/cap.wav --max-secs 120 > /root/audio_sink.log 2>&1 &
```

📌 **顺序**：B5 起好，再去「四、C 烧录」。烧完 ESP32 复位，串口应依次出现：

```
I (xxx) audio_tx: 音频流已连上 192.168.77.100:9001（16kHz/16bit/单声道）
I (xxx) audio_tx: 音频上行中：累计 N 字节（最近采样 ...）
```

120 秒后验收：

```bash
ls -l /root/cap.wav        # 16kHz/16bit/单声道 ≈ 32 KB/秒，120 秒约 3.8 MB
tail -6 /root/audio_sink.log
```

> 若 120 秒内烧录没跑完、没等到数据，**重跑一次 B5 即可**（不改代码）。

> ✅ **11-21 实机验收结果（B5 已过）**：接收端本次实际部署在 `/root/audio_sink.py`（与本节命令里的 `/root/kunpeng_vision/robot/` 路径等价，以后统一用 `/root/`）；`firewall-cmd --add-port=9001/tcp` 已放行并 `--reload`；ESP32 连上后 10 秒实收 **320000 字节**（=16kHz×2B×10s，无丢包），`cap.wav` 取回可播放、含咳嗽/说话声学信号。**9001 音频上行 ✅**；语音识别仍未接 🔶。

---

### 四、C 烧录（PC · 开「ESP-IDF 5.5 PowerShell」快捷方式，🔴 别用 Git Bash）

> Git Bash 跑 `idf.py` 会被官方拒 `MSys/Mingw is no longer supported`；
> ESP-IDF 装在 **PC**（USB 要摸到板载 CH343），**不装在鲲鹏板**。

```powershell
Set-Location D:\esp\s3_voice; idf.py build
```

```powershell
Set-Location D:\esp\s3_voice; idf.py -p COM7 flash monitor
```

> `COM7` 换成 2.4 查到的实际串口号。若自动找口成功，可去掉 `-p COM7` 直接 `idf.py flash monitor`。

**烧录后串口 monitor 期望看到（顺序）**：

```text
I (xxx) voice_main: ==== ESP32-S3 语音入口骨架启动（含 TCP 上行自检） ====
I (xxx) wifi_ap: SoftAP 已启动: SSID=Kunpeng-Robot, IP=192.168.77.1
I (xxx) voice_main: SoftAP 已启动 192.168.77.1；网关目标 192.168.77.100:9000
I (xxx) voice_main: [本地封帧 0] $ESTOP*5D（长度=11，只打印不下发）
I (xxx) voice_main: [本地封帧 1] $RESUME*19（长度=12，只打印不下发）
I (xxx) tcp_client: TCP connected -> 192.168.77.100:9000
I (xxx) tcp_client: send 11 bytes: $ESTOP*5D
I (xxx) tcp_client: recv 14 bytes: $OK,ESTOP*75
I (xxx) voice_main: 网关应答（14 字节）: $OK,ESTOP*75
I (xxx) voice_main: 中文帧默认关闭（SEND_CN_HOME_FRAME=0），明天联调再开
I (xxx) voice_main: [000] 读到 256 个采样: ... ... ... ...
```

🔴 **看到 `$ASR,mic_ok*50` 是旧固件的打印**；新固件自检帧是 `$ESTOP*5D` / `$RESUME*19`。
若还看到旧的，说明烧的是旧副本 / 没重新 build。

---

### 五、D 四层硬判据（跑到这一层才算「ESP32 链路通」）

| # | 证据 | 在哪看 | 通过标准 | ☐ |
|---|---|---|---|---|
| 1 | ESP32 侧 `TCP connected` | 串口 monitor | 打印 `TCP connected -> 192.168.77.100:9000`，且两条帧都 `send N bytes` | ☐ |
| 2 | 网关应答帧 | 串口 monitor | `$OK,ESTOP*75` 与 `$OK,RESUMED*75`，**不是** `$ERR,2*5B` | ☐ |
| 3 | 鲲鹏侧网关日志 | `tail -20 /root/gw.log` | 有 `payload`=`STOP` / `RESUME` 的记录、`apply:false`（dry-run） | ☐ |
| 4 | 外部主机也能连 | PC 上 B4 的 `nc` | 回同样的 `$OK,...`，证明 `--host 0.0.0.0` 生效、不是本机自连 | ☐ |

```bash
tail -20 /root/gw.log
```

**🆕 11-15 补：别靠人眼翻日志，用 `kunpeng/robot/check_esp32_link.py` 自动判定**（宿舍无硬件时写，本机 `--self-test` 5 组全绿）。把串口 monitor 存成 `uart.txt`、`/root/gw.log` 存成 `gw.log`、B4 的 `nc` 输出存成 `probe.txt`，在 PC 上跑：

```bash
python3 check_esp32_link.py --uart-log uart.txt --gw-log gw.log --probe-out probe.txt
```

它会自动否决三类最容易看走眼的情况：烧成旧固件（串口出现 `$ASR,mic_ok*50`）、收到 `$ERR,2*5B`、
网关被误开 `--apply`（日志 `apply=true`，会真动臂）；输出 `RESULT: PASS=4 FAIL=0` 才算四层齐。
🔴 **脚本只判据、不抬档**，仍要人工填第八节记录表并回报后才统一订文档。

> 🔴 这四层**只证明链路通**，不证明「语音控制已实现」：ESP32 还没接 WakeNet / ASR，
> 发的是写死的自检帧，`source` 恒 `mock`。**档位仍是 🔶**，回报记录表后才按第八节统一订文档。

---

### 六、E 可选加试：让臂真的动（🔴 不是今晚交付的必做项，做之前先读三条安全前提）

只有**链路四层全绿**后，才考虑这一步。三条前提缺一不可：

1. 臂已摆 **C 姿态**，脉宽 `1636,1470,2177,1106,1450,1567`（下发 curl 见 `kunpeng/robot/README.md`），**周围无人**；
2. 鲲鹏网关切到 `--apply`（此时中文帧才会真下发 `HOME`）：
   ```bash
   pkill -f voice_gateway.py
   python3 -u /root/kunpeng_vision/robot/voice_gateway.py --host 0.0.0.0 --port 9000 --apply > /root/gw.log 2>&1 &
   ```
3. PC 副本里把 `D:\esp\s3_voice\main\main.c` 的 `#define SEND_CN_HOME_FRAME 0` 改成 `1`，
   **重新 `idf.py build` + 重烧**。中文帧用 C 八进制字节串写死：
   `"ASR,\345\233\236\345\210\235\345\247\213\344\275\215"` ⇒ `$ASR,回初始位*61\r\n` ⇒ `$OK,HOME*27`。

加试判据（沿用阶段 2 已实机验收的四层）：① 应答 `$OK,HOME*27` ② 日志 `payload=HOME` / `sent:true` / `apply:true`
③ `/api/state` 脉宽从 C 姿态变初始位 `1580,1900,2400,700,1450,1500`
④ **肉眼看到臂从 C 姿态收回初始位**。

🔴 **收尾（必做）**：断电前把臂回初始位、停掉 `--apply` 网关；中文帧宏改回 `0` 再重烧，避免下次上电误触发。

```bash
pkill -f voice_gateway.py
```

---

### 七、F 失败分流（别一上来就改代码）

| 现象 | 最可能根因 | 先做这一条 |
|---|---|---|
| `SoftAP 启动失败，跳过 TCP 上行` | 热点没开起来（SSID/密码/信道配置错） | 核对 2.2；看串口 `wifi_ap` 的日志 |
| `connect() 失败: errno=...` | 鲲鹏没接入 ESP32 热点，或 `GATEWAY_HOST` 不是 `192.168.77.100` | 核对 2.3：鲲鹏 `nmcli con up esp-ap` 后 `ip a` 看 wlan0 是否为 `.100` |
| `recv() 返回 -1（超时）` | 网关没起 / 没带 `--host 0.0.0.0` / 防火墙 | 板端 `tail -20 /root/gw.log`；PC 上先跑 B4 |
| 收到 `$ERR,2*5B` | 发成了 `$ASR,mic_ok` 或 `$ASR,home`（词表只有中文） | 确认烧的是新固件；纯 ASCII 快速通道不该出现这个 |
| 串口全是乱码 | 波特率 / 中文帧显示问题 | 只看 ASCII 帧；中文 UTF-8 帧显示乱码但**字节是对的**，别改宽字符 |
| `I2S 启动失败` | BCLK/WS/DIN 三根线或 L/R 引脚 | 单独定位，与网络链路无关；本地封帧自检仍会先跑 |

> 🔴 板端命令一律**单行逐条粘**；SecureCRT 多行粘贴会回显串扰，别把「看着像乱了」当成命令失败。

---

### 八、记录表（跑完填，别提前打勾）

| # | 判据 | 期望 | 实测 | 通过 |
|---|---|---|---|---|
| 0 | 第一节 build 结论 | `BUILD_RC=0` + 有 `.bin` | ☐ | ☐ |
| 1 | 串口 `TCP connected` | 连到鲲鹏 `:9000` | ☐ | ☐ |
| 2 | 两条应答帧 | `$OK,ESTOP*75` / `$OK,RESUMED*75` | ☐ | ☐ |
| 3 | 网关日志 | `payload`=`STOP`/`RESUME`，`apply:false` | ☐ | ☐ |
| 4 | PC 侧 `nc` 外部探活 | 同样回 `$OK,...` | ☐ | ☐ |
| 5 | 每秒 PCM 采样 | 非恒定、说话时变化 | ☐ | ☐ |
| 6 | （可选）中文帧加试 | 四层证据 + 肉眼见臂动 | ☐ | ☐ |

这张表全绿并回报后，才统一订 `esp32/README.md`、`docs/固件与ESP32开发.md`、根 `README.md`、
`MEMORY.md` 的档位；**只抬「ESP32 链路自检已实机打通」这一条**，
语音入口整体仍 🔶（WakeNet / 命令词 / ESP32 遥测进 EMQX 都还没有代码）。

---

### 九、诚实边界 / 命名红线

- ✅ 11-21 已烧录实机（TCP 9000 + 9001 音频上行 10s/320000B）；语音识别仍 🔶。
- 🔴 鲲鹏侧阶段 2 网关链路 ✅（11-14 16:40 实机验收），但 `source` 恒 `mock`、**ESP32 已接入但只发链路自检帧**；
  `$ASR` 帧草案此前只做到「C / Python 语法层逐字节一致」，**语义层实机对齐要等明天**。
- 🔴 禁写「语音控制已实现 / 语音→动作闭环 / 已接 ESP32 语音链路 / 自然语言控制机械臂」；
  只叫「可观测性闭环 / 视觉引导的移动操作平台」。
- ✅ 明天四层全绿后能写：「ESP32-S3 侧经 Wi-Fi TCP 把统一帧发到鲲鹏网关并收到校验通过的应答，
  完成端侧节点到边缘侧收帧服务的链路联通」——**仅此一句**，不外延到语音识别或抓取。
- 🔴 现行底盘（2×TB6612FNG + JGB37-520）实机验收按用户要求**仍放最后**；旧 `car/` 烧录与第二路 USB-TTL 已作废，见第五节。