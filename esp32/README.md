# esp32/ —— ESP32-S3 端（支柱② 语音入口，P0 核心）

> 🔴 **状态口径（2026-11-13 定位调整，务必先读）**
> **定位**：ESP32-S3 与本目录从「P2 可选项」提升为**项目三大核心支柱之二**（支柱② 语音入口）。
> 判据不变：**核心 = 必须能演示**。所以下面的缺口必须补齐，不许因为「它是核心」就写成已完成。
> **硬件**：✅ 成品 ESP32-S3 开发板可直接使用（CH343 / N16R8 / Type-C / 已焊），另有 INMP441 I2S 麦模块、MAX98357A I2S 功放 + 3W 喇叭（均已焊排针，无需焊接）。
> 自画核心板 / 扩展板**仍待焊**，用于最终装车集成，与成品板是两条并行的线，不冲突。
> **软件**：🔶 **ESP-IDF 工程骨架已落盘（12 个文件，11-14 晚新增 TCP 上行层）；PC 整机 `idf.py build` 已通过，未烧录 / 未上板 / 未联调**。
> 已有：Wi-Fi STA 连接、I2S 采音、`$ASR,...*CK` 封帧、**Wi-Fi TCP 上行鲲鹏网关**（`tcp_client.c/.h`，一次一连接收发一帧）。
> **11-14 晚**：PC 上用现成 ESP-IDF v5.5.5 在纯 ASCII 副本 `D:\esp\s3_voice` 跑了 `idf.py set-target esp32s3`（**rc=0**）+ 整机 `idf.py build` **通过**（`Project build complete`、`BUILD_RC=0`、`tcp_client.c` 编译链接成功、产物 4 件齐全，回读记录见 `docs/明日回教室-ESP32照跑清单.md` 第一节）；
> 🔴 **编译通过 ≠ 实机**：仍未烧录 / 未上板 / 未联调，档位仍 🔶，不许写「已接 ESP32 / 语音控制已实现」。
> **未写**：WakeNet 唤醒、命令词识别（MultiNet）、ST7789 显示、语音→动作序列、ESP32 遥测进 EMQX。
> 🔴 **自检帧是写死的 `$ESTOP*5D` / `$RESUME*19` 纯 ASCII 快速通道**（网关 dry-run 也回 `$OK`，**默认固件不会让臂动**）；
> 中文 `$ASR,回初始位*61` 已写好但由 `SEND_CN_HOME_FRAME=0` 默认关闭（`--apply` 下会真让臂回初始位，明天确认安全再开）。
> **前置条件已部分达成**（原三条）：① 主链路稳定 → ✅ 已通；② 手眼至少一次实标 → ✅ **已实标 12 组、resid 1.60mm**；③ 语音前端自身调通 → 🔶 **待做（当前唯一阻塞）**。
> 鲲鹏侧 `kunpeng/llm/text_gateway.py` 仍是**离线文本入口网关草案**（🔶，mock、只打印不下发），不是「语音控制已实现」。
> 依据：`docs/简历-具身智能项目条目.md`、`kunpeng/llm/README.md` 第六节、`README.md` 三大核心支柱表。

---

## 一、概念先讲清（小白视角：大白话 + 规范定义）

| 概念 | 大白话 | 规范定义 |
|---|---|---|
| ESP-IDF | 乐鑫官方给 ESP32 写程序的「整套工具箱」，装了才能编译、烧录 | Espressif IoT Development Framework，基于 FreeRTOS 的官方 SDK，提供 `idf.py` 构建系统与组件库（本项目按 ESP-IDF v5.x 写） |
| I2S | 专门传数字音频的「几根线」，麦克风和功放都用它，比模拟麦抗干扰 | Inter-IC Sound（I²S）串行音频总线，信号为 BCLK / WS / SD；INMP441 走 RX、MAX98357A 走 TX |
| N16R8 | 模组型号里「内存多大」：16MB 存程序 + 8MB 运行内存 | ESP32-S3-WROOM-1-**N16R8** = 16 MB Flash + 8 MB Octal PSRAM；放 WakeNet/ESP-SR 模型需在 `sdkconfig` 开启 Octal PSRAM |
| WakeNet | 「嘿 Siri」那种：一直听着、听到固定唤醒词才醒 | 乐鑫 ESP-SR 中的离线唤醒词模型（keyword spotting，本地神经网络推理），无需联网 |
| XOR 校验帧 | 给每句话末尾加个「指纹」，收方重算一遍对不上就丢掉 | 项目统一串口帧 `$<CMD>[,<ARG>...]*<CK>\r\n`，CK = payload 各**字节**逐字节异或、2 位大写 HEX（真源 `Core/Src/uart_protocol.c`） |

**⚠️ 跨语言坑（完整五段式见 `docs/排障记录-LLM.md` 第 2 条）**：XOR 定义在**字节序列**上，中文是 UTF-8 多字节。Python 侧必须先 `.encode("utf-8")` 再逐字节 XOR；若按 `ord(ch)` 逐字符算，与 C 固件 `(uint8_t)` 对不齐，将来真接板子会被全判 `ERR,1`。

## 二、这块板在系统里的位置

```
[你说话] → ESP32-S3（唤醒 + 语音转文字）→ 鲲鹏（理解意图 + 算目标点）
         → 串口发 $JOINT → STM32（驱动 6 路舵机抓取）
```

ESP32-S3 只做**最前端一件事：把人说的话变成一句文本**，再经 Wi-Fi/串口交给鲲鹏；不参与运动学解算、不直接驱动舵机。分工理由：ESP32 算力有限（双核 240MHz），但 Wi-Fi/BLE 与 I2S 麦克风接口最方便。

**当前实现到哪一步**：鲲鹏侧已能把「一句文本」翻成结构化指令（`kunpeng/llm/intent_parser.py`），收帧服务 `kunpeng/robot/voice_gateway.py` **11-14 16:40 已上板 `--apply` 实机验收四层硬判据全绿**（但 `source` 恒 `mock`、没接 ESP32）。
ESP32 侧 11-14 晚补上了 **Wi-Fi TCP 上行层**（`tcp_client.c/.h` + `main.c` 自检帧改 `$ESTOP/$RESUME` 纯 ASCII 快速通道）：
本机实证 C 侧封帧与 Python 侧逐字节一致、真网关应答正确（`PASS=24 FAIL=0`），整机 build 在 PC 上已启动（结论见清单）。
🔴 **仍未上板联调**：ESP32 是否真能把帧发到教室热点下的鲲鹏，取决于明天的实际网络与网关 IP；
**唤醒 / ASR / 语音→动作序列仍无代码**，发的只是写死的链路自检帧，不是「说话控制机械臂」。

## 三、硬件清单（分两组，别混）

**A 组 · 成品板 —— 已到手，插线就能调（无需焊接）**

| 部件 | 型号 / 特征 | 用途 |
|---|---|---|
| 主控开发板 | ESP32-S3-WROOM-1 **N16R8**，板载 **CH343** USB-TTL + 双 Type-C + 稳压 + Wi-Fi 天线，已焊 | 语音 + 联网 + 一键下载 |
| 麦克风模块 | **INMP441** 兼容 I2S 数字麦，已焊排针 | 采 PCM（I2S0 RX） |
| 功放 + 喇叭 | **MAX98357A** I2S D 类功放 + 3W 扬声器 | 播提示音（I2S1 TX） |

> 🔌 供电提醒：MAX98357A 播放瞬时电流大，建议**独立 5V 供电、只与主控共地**；INMP441 用 3.3V。I2S0 给麦、I2S1 给功放，两路分开别抢同一控制器。

**B 组 · 自画板 —— 仍待焊，最终装车集成用**

| 部件 | 型号 | 接到哪 |
|---|---|---|
| 核心板 | 自画 35.56×63mm（参照官方 DevKitC-1），H1/H2 按 J1/J3 顺序 1×22 | — |
| USB 转串口 | CH343G + 双 Type-C（一键下载） | — |
| 显示屏 | ST7789（SPI） | SPI，显示识别结果 |
| 存储 | microSD（SDIO/SPI） | 存唤醒词模型 / 音频 |
| 灯效 | WS2812 | RMT，状态指示 |
| 总线 | CAN（需 TJA1050 等收发器） | TWAI，预留接底盘 |

> A 组是「现在就能插线调」的，B 组是「以后装车」的，两者不互相阻塞。

## 四、软件路线（含当前状态）

| 步骤 | 做什么 | 状态 | 关键点 |
|---|---|---|---|
| 1 | ESP-IDF 工程骨架：`idf.py` 建工程、串口打印 + Wi-Fi 连接 | 🔶 **骨架已落盘（12 个文件，含 `tcp_client.c/.h`）**；`protocol.c` 本机 gcc 编译 + S2 实机锚点帧反查通过；**11-14 晚 PC 上整机 `idf.py build` 已通过**（`Project build complete` / `BUILD_RC=0` / 产物齐全，回读见 `docs/明日回教室-ESP32照跑清单.md` 第一节）**；**未烧录 / 未上板** | ✅ PC 已装 ESP-IDF v5.5.5（`D:\ESP32\Espressif`），须从 **ESP-IDF PowerShell 快捷方式**启动（Git Bash 会被拒）；工程须拷到 ASCII 路径；先确认 CH343 下载电路正常 |
| 2 | INMP441 采音，I2S0 读原始 PCM，串口打印波形 | 🔶 驱动骨架已落盘（`i2s_std` 新 API），**未上板验证** | 先验证麦克风有数据，再谈识别；读出全 0 先查 L/R 引脚（接 GND = 左时隙） |
| 3 | ESP-SR / WakeNet 离线唤醒词 | 📋 无代码 | 模型放 Flash/PSRAM，`sdkconfig` 开 Octal PSRAM |
| 4 | 唤醒后录音 → 在线 ASR 或本地命令词 | 📋 无代码 | 在线需 Wi-Fi + API Key |
| 5 | 封好的帧 → Wi-Fi TCP 发鲲鹏收帧网关 | 🔶 **有代码未实机**：`tcp_client.c/.h` 已落盘、`main.c` 发 `$ESTOP/$RESUME` 自检帧；本机真网关实证 `PASS=24 FAIL=0`；**未上板联调** | **逐字节** XOR；中文帧用 C 八进制字节串写死（CK `61`）且默认关闭；鲲鹏网关须 `--host 0.0.0.0`；帧格式**语法层已对齐、语义层待明天实机对齐** |
| 6 | MAX98357A 回 TTS 提示音 | 📋 无代码 | I2S1 TX |

## 五、和现有代码的对接点

- 统一串口帧：`$<CMD>[,<ARG>...]*<CK>\r\n`，CK = payload 逐**字节** XOR、2 位大写 HEX。真源 `Core/Src/uart_protocol.c`（`proto_send_frame()` / `proto_parse_frame()`）；Python 侧参照 `kunpeng/llm/text_gateway.py` 的 `xor_checksum()`。
- S2 实机锚点帧：`$PING*10` → `$OK,PONG*3E`（`"PING"` XOR = `0x10`、`"OK,PONG"` XOR = `0x3E`）。任何新实现先拿这两帧反查，别自己手算。
- 文本入口草案帧：`$ASR,<文本>*<CK>\r\n`（`text_gateway.py` 建议）。🔴 这是**建议草案**，不是 STM32 ↔ 鲲鹏 `$JOINT` 链路，不进 `uart_protocol.c`；**语法层 11-14 晚已与 ESP32 `protocol.c` 本机逐字节对齐，语义层实机对齐待明天**、未接真串口。
- 通信方式**已选 Wi-Fi TCP**：`tcp_client.c` 作为 BSD socket 客户端连鲲鹏 `:9000`，一次一连接发一帧收一应答（见下文三行法）。串口方案（ESP32 UART ↔ 鲲鹏 USB-TTL）留作备选，未实现。🔴 网关默认 bind `127.0.0.1` ⇒ **必须 `--host 0.0.0.0`**，否则外部连不上。

## 六、当前目录结构（12 个工程文件 + 本 README = 13 个，11-14 晚补 TCP 上行层）

> 口径说明：其它文档写「12 文件骨架」指**不含本 README 的工程文件**；11-15 完整检查时同步脚本实测 `esp32/` 共 **13 个文件**（含 README.md，不含 `build/` 产物），两者不矛盾。

```
esp32/
├── README.md                  ← 本文件（口径 + 三行法 + 诚实边界）
├── CMakeLists.txt             ← ESP-IDF 顶层工程文件
├── sdkconfig.defaults         ← 默认配置（16MB Flash / Octal PSRAM / 1000Hz tick），一行未动
└── main/
    ├── CMakeLists.txt         ← 组件注册：5 个 .c + REQUIRES 六个官方组件（含 lwip）
    ├── main.c                 ← app_main：连 Wi-Fi → 本地封帧自检 → TCP 上行自检 → 启 I2S → 循环读 PCM
    ├── tcp_client.c / .h      ← 🆕 Wi-Fi TCP 上行鲲鹏网关（BSD socket，一次一连接）
    ├── i2s_mic.c / .h         ← INMP441 I2S0 RX 采 PCM（新 I2S 驱动）
    ├── wifi_sta.c / .h        ← Wi-Fi STA 连接骨架（事件组等 IP；SSID/密码占位符明天本机改）
    └── protocol.c / .h        ← 统一帧 XOR 封包（与 STM32 对齐；只封帧，发送交给 tcp_client.c）
```

> 排障记录统一放工程根 `docs/排障记录-ESP32.md`（与 S3b / IoT / LLM 三个模块一致，个人复习材料不进 Git）。
> 🔴 **代码已落盘 ≠ 可用**：这些文件在 PC 上跑过本机实证与整机 build（结论见清单），但**从未烧进 ESP32、从未上板联调**。
> 明天照 `docs/明日回教室-ESP32照跑清单.md` 跑完四层硬判据，才能把「链路」写成已验证；之后才逐步填 WakeNet / ASR。

## 七、诚实边界（🔴 面试不能越线）

| ✅ 能说 | ❌ 不能说 |
|---|---|
| 「ESP32-S3 语音是三大支柱②（P0 核心，整体仍 🔶）；成品开发板 + 麦/功放/喇叭已可直接用，自画核心板仍待焊」 | 「语音控制已实现 / 已接 ESP32-S3」 |
| 「鲲鹏侧收帧网关 11-14 16:40 已上板实机验收（38 项单测 + 四层硬判据）；`llm/` 44 项单元测试全绿」 | 「语音唤醒 / 离线 ASR / 语音→动作闭环 / 自动抓取」 |
| 「统一帧用逐字节 XOR，与 STM32 固件对齐；ESP32 侧 `protocol.c` 与鲲鹏侧 Python 帧语法层本机逐字节一致」 | 「`$ASR` 帧已是实机协议 / 语义层已联调」 |
| 「ESP-IDF 工程骨架已落盘 12 文件（含 Wi-Fi TCP 上行层），`protocol.c` 本机 gcc 编译并用 `$PING*10` / `$OK,PONG*3E` 反查通过；PC 上用现成 v5.5.5 整机 `idf.py build` 已通过（`Project build complete` / 产物齐全，仍未烧录）」 | 「已在 ESP32 上编译烧录成功 / 语音唤醒已实现 / 已接 ESP32 主链路 / 明天已能说话控制机械臂」 |

---

## 八、为什么这个目录之前是空的（保留原因说明）

**先做能做的**：STM32 固件已跑通、鲲鹏识别已跑通、网页遥控已能控舵机，这三块不依赖 ESP32 就能演示完整闭环（用手点网页代替说话）。等这条链路稳定，再把「手点」换成「嘴说」，风险最低——语音识别本身（尤其离线唤醒 + 中文 ASR）是独立的坑，混在一起调会分不清是识别错了还是机械臂错了。

> 现在成品板到手，硬件门槛已解除，但上面这条**接入顺序判断不变**：可以并行开 ESP-IDF 支线，却不把语音塞进主链路。

---

## 九、README 三行法（写完代码主动补，面试前当复习提纲）

> 三行 = ① 用了什么语法 / 技术 ② 为什么这么设计 ③ 不这么写会出什么问题。重点看「为什么」。

**整体工程（`CMakeLists.txt` + `sdkconfig.defaults`）**
① CMake 的 `cmake_minimum_required` / `include(project.cmake)` / `project()`，加 `idf_component_register()` 注册组件；`sdkconfig.defaults` 声明 16MB Flash + Octal PSRAM + 1000Hz tick。
② 把「有哪些源文件、依赖哪些官方组件、芯片有哪些资源」集中声明，而不是散落在命令行里；换电脑只靠 `IDF_PATH` 环境变量定位工具链，不必改工程文件。
③ 不写 `REQUIRES driver` 会找不到 `driver/i2s_std.h`；不声明 16MB Flash / PSRAM，WakeNet 模型会按默认小容量编译、根本放不下。

**`i2s_mic.c`（I2S0 RX 采音）**
① ESP-IDF v5.x 新 I2S 驱动三步 `i2s_new_channel()` → `i2s_channel_init_std_mode()` → `i2s_channel_enable()`；`i2s_chan_config_t` / `i2s_std_config_t` 结构体指定初始化，宏 `I2S_STD_CLK_DEFAULT_CONFIG` / `I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG` 填默认时序；`static` 全局句柄 + `#define` 集中接线。
② 「申请通道」和「配时序」分开是官方分层，同一通道可换 Philips/PCM/TDM 而不重做 DMA；引脚宏集中一处，接线一改只动一个地方；24-bit 左对齐数据统一右移 8 位还原成有符号值。
③ enable 之前不配时序会读不到数；停的时候不先 `i2s_channel_disable()` 就 `i2s_del_channel()`，DMA 还在写内存会踩内存；24-bit 不右移会把振幅当成放大 256 倍的假数据。

**`wifi_sta.c`（Wi-Fi STA）**
① `nvs_flash_init()` → `esp_netif_init()` + `esp_event_loop_create_default()` → `esp_event_handler_instance_register()` 注册回调 → `esp_wifi_start()` → `xEventGroupWaitBits()` 阻塞等事件；用了事件组 EventGroup、回调 callback、`strncpy`、`{ 0 }` 结构体清零。
② 用事件回调 + 事件组而不是 while 轮询全局变量，等待时任务睡眠让出 CPU；只认 `IP_EVENT_STA_GOT_IP` 才算连上（拿到 IP 才能通信）；`wifi_config_t = { 0 }` 先清零避免残留字节混进密码。
③ 事件组若在注册回调之后才创建，回调里 `xEventGroupSetBits()` 会写空指针直接崩；把 `WIFI_EVENT_STA_CONNECTED` 当成「能通信」会过早发数据导致丢包。

**`protocol.c`（统一帧封包）**
① 纯标准 C：`uint8_t` 逐字节 `^=`、`static const char HEX_DIGITS[]` 查表、`char *out` + `size_t out_size` 显式长度、手工拼帧并返回长度。
② 只依赖 `<stddef.h>` / `<stdint.h>`，不碰 ESP-IDF 头 ⇒ 能脱离板子在本机 gcc 编译跑自测；传 `out_size` 让调用方无法溢出；`uint8_t` 明确「按字节算」，和 C 固件 `(uint8_t)` 天然对齐。
③ 不检查 `out_size` 就是经典缓冲区溢出；按 Unicode 码点算中文（Python 侧常见错）会与固件判 `ERR,1`；`protocol_build_frame()` 若直接接 UART 写就变成「下发」，本骨架**故意只打印不下发**。

**`tcp_client.c` + `main.c`（Wi-Fi TCP 上行层，11-14 晚新增）**
① **用了什么语法**：BSD socket 四件套 `getaddrinfo()` → `socket()` → `connect()` → `send()/recv()/close()`（头 `<sys/socket.h>` / `<netdb.h>` / `<sys/time.h>` / `<unistd.h>`，在 ESP-IDF 里由 **lwip** 组件提供）；`setsockopt(SO_RCVTIMEO/SO_SNDTIMEO)` 配 `struct timeval` 超时；`snprintf()` 把端口转字符串、`memset()` 清零、`size_t` 累加已发字节；`enum` 定义 `TCP_ERR_*` 分级错误码 + `tcp_client_strerror()` 翻人话；`#define` 把网关 IP / 端口 / 超时 / 缓冲区集中在头文件；`typedef struct` 做「帧表」把数据和发送逻辑分开；C99 八进制转义 `\ooo` 写中文 UTF-8 字节串；`#if SEND_CN_HOME_FRAME` 做编译期安全闸门；`vTaskDelay(pdMS_TO_TICKS())` 让出 CPU。
② **为什么这么设计**：骨架是「发一句、等一句」的短事务 ⇒ **一次一连接**，不维护长连接状态机，最不容易漏关 fd；入口先挡空指针 / 端口 0 / 缓冲区 0；`recv()` 故意只收 `reply_size-1`，留最后一格给 `'\0'` 让调用方能直接 `%s` 打印；**超时必须设** —— 网关没起 / 网断了 `recv()` 会永久挂着，连后面的 I2S 采音循环都跑不到，串口看起来像「整机死机」；网关 IP 会随网络变，集中成宏改一处重新 build 即可；中文帧写成字节值而不是直接写汉字，是因为源码文件编码 / 编译器执行字符集可能因机器而异 ⇒ 与鲲鹏侧 `printf` 八进制兜底、Python `.encode("utf-8")` 三者对齐，CK 一定算得对。
③ **不这么写会出什么问题**：`main/CMakeLists.txt` 不补 `REQUIRES lwip` ⇒ 链接期 `undefined reference to getaddrinfo`；不设超时 ⇒ 断网时整机像死机；`send()` 不写循环 ⇒ TCP 是字节流、理论上可能只发一半；不 `close()` / `freeaddrinfo()` ⇒ 几十次后 fd 耗尽、`connect()` 失败会被误判成「Wi-Fi 掉了」；`recv()` 收满不留 `'\0'` ⇒ `%s` 打印越界；自检帧若还用 `$ASR,mic_ok*50` / `$ASR,home*63` ⇒ 网关一律 `$ERR,2*5B`（词表只有中文）；中文帧若忘了 `SEND_CN_HOME_FRAME=0`，在 `--apply` 网关下会**真的让臂回初始位**，属于误触发安全风险；网关忘了 `--host 0.0.0.0` ⇒ ESP32 从 Wi-Fi 连过来 `Connection refused`，极易被误判成热点问题。

**`main.c` / `tcp_client.c` 11-15 完整检查后的 3 处最小加固**（一次性检查报告已按「不落盘一次性步骤」规则删除，结论直接记在这里）
① **用了什么**：删掉 `main.c` 里全文没用到的 `<stdio.h>`；用到 `bool` 就自己 `#include <stdbool.h>`，不再靠 `wifi_sta.h` 间接带进来；两个 `setsockopt()` 的返回值用 `if (... != 0 || ... != 0)` 检查，失败时 `ESP_LOGW` 打一条警告。
② **为什么这么设计**：C 的头文件是**传递包含**，靠别的头顺带带进 `<stdbool.h>` 属于隐式依赖，以后换个 include 顺序就会莫名编不过；上面三行法已经写了「超时必须设」，那就必须验证超时**真的设上了**，否则 `setsockopt` 静默失败时现象仍是「整机死机」、却查不出原因 —— 代码要和注释自洽。
③ **不这么写会怎样**：未用 include 是无效代码，读者会以为用了 `printf` 白找一圈；隐式依赖是典型的「在我机器上能编」；超时保护形同虚设而串口只看到「卡住」，极易被误判成 Wi-Fi 掉了或网关没起。**这 3 处都不改功能、不改协议、不改帧字节**。
**复验证据**：`gcc -std=c99 -Wall -Wextra -Werror` 零警告；当前源码同步到 ASCII 副本（13 文件 md5 全一致）后整机增量 `idf.py build` **`BUILD_RC=0`**，`[4/10] main.c.obj` + `[5/10] tcp_client.c.obj` 确实重编、链接出 `kunpeng_voice.bin` 803760 B；21 项静态/本机动态检查全绿。

---

## 十、编译与烧录命令（标注执行位置）

> 🔴 **前置 0：环境已就位，不用装**（11-14 17:38 订正，撤回本节早先「先装 ESP-IDF」的写法）。
> PC 上已有完整 **ESP-IDF v5.5.5** 在 **`D:\ESP32\Espressif`**（EIM / Espressif-IDE 风格安装，纯 ASCII 路径、符合官方限制）。
> 旧记录「本机没有 `idf.py`」只指 **Git Bash 的 PATH 里没有**；且实测 Git Bash 里跑 `idf.py` 会被官方拒 `MSys/Mingw is no longer supported`
> ⇒ **必须从开始菜单「ESP-IDF 5.5 PowerShell」快捷方式启动**（或按指引里的 `Initialize-Idf.ps1 -IdfId` 手动初始化）。
> 完整环境核对 / 启动 / build 指令 → **`docs/ESP-IDF安装与首次build指引.md`**。
> 🔴 ESP-IDF 装在 **PC**（USB 烧录要摸到板载 CH343 串口），**不装在鲲鹏板**；鲲鹏只跑 `web_control.py` / `voice_gateway.py` 等运行时。
> 下面命令要在那个快捷方式窗口里跑；助手没有代跑、也没有声称编译 / 烧录成功。
>
> 🔴 **前置 1（11-14 订正，旧写法会炸）**：官方明确「ESP-IDF 构建系统不支持路径含空格 / 非 ASCII 字符」，
> 而本目录在 `D:\简历\STM32项目\4.ESP32-S3项目\...\esp32\` ⇒ **必须先把 `esp32/` 拷到纯 ASCII 短路径**，原工程一行不动：

```powershell
New-Item -ItemType Directory -Force -Path D:\esp | Out-Null
Copy-Item -Path "D:\简历\STM32项目\4.ESP32-S3项目\RoboticArm_Kunpeng\esp32" -Destination "D:\esp\s3_voice" -Recurse -Force
```

> 之后所有命令与改动都在 **`D:\esp\s3_voice`** 副本里做（含 `wifi_sta.h` 的 `WIFI_SSID` / `WIFI_PASSWORD`：改成真实热点、**别提交 Git**；PC 与鲲鹏板连同一热点）。
> 三条命令各自带 `Set-Location`，**单条复制也能独立跑**，不必依赖上一条的会话状态；PS 5.1 无 `&&`，用 `;`。

【PC · ESP-IDF PowerShell】
```
Set-Location D:\esp\s3_voice; idf.py set-target esp32s3
```

【PC · ESP-IDF PowerShell】
```
Set-Location D:\esp\s3_voice; idf.py build
```

【PC · ESP-IDF PowerShell】
```
Set-Location D:\esp\s3_voice; idf.py flash monitor
```

> 若 `idf.py flash monitor` 自动找口失败，改用 `idf.py -p COM7 flash monitor`；
> `COM7` 要换成设备管理器里 CH343 的实际串口号，值按当前机器写死、每次重插重查。
> 🔴 **11-14 晚固件已改，判据跟着改**：新固件自检帧是 `$ESTOP*5D` / `$RESUME*19`（纯 ASCII 快速通道），
> 不再是 `$ASR,mic_ok*50`；烧录后期望看到「本地封帧 0/1」→ `TCP connected` → `网关应答: $OK,ESTOP*75` / `$OK,RESUMED*75`
> → `中文帧默认关闭（SEND_CN_HOME_FRAME=0）` → 每秒打印的 PCM 采样值。完整四层硬判据与失败分流见
> **`docs/明日回教室-ESP32照跑清单.md` 第四、五、七节**。
> 🔴 看到应答帧只证明「ESP32 ↔ 鲲鹏网关链路通」，**不证明语音控制**：发的还是写死的自检帧、没接 WakeNet / ASR、没让臂动。
