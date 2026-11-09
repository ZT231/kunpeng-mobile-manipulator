# esp32/ —— ESP32-S3 端（语音入口，P2 阶段）

> 🔴 **状态口径（2026-11-09 同步，务必先读）**
> **硬件**：✅ 已有买好的**成品 ESP32-S3 开发板可直接使用**（CH343 / N16R8 / Type-C / 已焊），另有 INMP441 I2S 麦模块、MAX98357A I2S 功放 + 3W 喇叭（均已焊排针，无需焊接）。
> 自画核心板 / 扩展板**仍待焊**，用于最终装车集成，与成品板是两条并行的线，不冲突。
> **软件**：🔶 **ESP-IDF 工程骨架已落盘（10 个文件），未编译 / 未烧录 / 未上板**；语音识别整体仍是 **P2 / 📋 规划**——WakeNet 唤醒、ASR、ST7789 显示、语音→动作序列**均无代码**，采音与封帧只有骨架（只打印不下发）、未接主链路。
> 鲲鹏侧 `kunpeng/llm/text_gateway.py` 只是**离线文本入口网关草案**（🔶，mock、只打印不下发），不是「语音控制已实现」。
> **接进主链路的时间点不因成品板到手而提前**：仍等 ① 主链路稳定 ② 手眼至少一次实标 ③ 语音前端自身调通。
> 依据：`docs/简历-具身智能项目条目.md` 第三节、`kunpeng/llm/README.md` 第六节。

---

## 一、概念先讲清（小白视角：大白话 + 规范定义）

| 概念 | 大白话 | 规范定义 |
|---|---|---|
| ESP-IDF | 乐鑫官方给 ESP32 写程序的「整套工具箱」，装了才能编译、烧录 | Espressif IoT Development Framework，基于 FreeRTOS 的官方 SDK，提供 `idf.py` 构建系统与组件库（本项目按 ESP-IDF v5.x 写） |
| I2S | 专门传数字音频的「几根线」，麦克风和功放都用它，比模拟麦抗干扰 | Inter-IC Sound（I²S）串行音频总线，信号为 BCLK / WS / SD；INMP441 走 RX、MAX98357A 走 TX |
| N16R8 | 模组型号里「内存多大」：16MB 存程序 + 8MB 运行内存 | ESP32-S3-WROOM-1-**N16R8** = 16 MB Flash + 8 MB Octal PSRAM；放 WakeNet/ESP-SR 模型需在 `sdkconfig` 开启 Octal PSRAM |
| WakeNet | 「嘿 Siri」那种：一直听着、听到固定唤醒词才醒 | 乐鑫 ESP-SR 中的离线唤醒词模型（keyword spotting，本地神经网络推理），无需联网 |
| XOR 校验帧 | 给每句话末尾加个「指纹」，收方重算一遍对不上就丢掉 | 项目统一串口帧 `$<CMD>[,<ARG>...]*<CK>\r\n`，CK = payload 各**字节**逐字节异或、2 位大写 HEX（真源 `Core/Src/uart_protocol.c`） |

**⚠️ 跨语言坑（完整五段式见 `docs/排障记录-LLM.md` 第 2 条）**：XOR 定义在**字节序列**上，中文是 UTF-8 多字节。Python 侧必须先 `.encode("utf-8")` 再逐字节 XOR；若按 `ord(ch)` 逐字符算，与 C 固件 `(uint8_t)` 对不齐，将来真接板子会被全判 `ERR,2`。

## 二、这块板在系统里的位置

```
[你说话] → ESP32-S3（唤醒 + 语音转文字）→ 鲲鹏（理解意图 + 算目标点）
         → 串口发 $JOINT → STM32（驱动 6 路舵机抓取）
```

ESP32-S3 只做**最前端一件事：把人说的话变成一句文本**，再经 Wi-Fi/串口交给鲲鹏；不参与运动学解算、不直接驱动舵机。分工理由：ESP32 算力有限（双核 240MHz），但 Wi-Fi/BLE 与 I2S 麦克风接口最方便。

**当前实现到哪一步**：鲲鹏侧已能把「一句文本」翻成结构化指令（`kunpeng/llm/intent_parser.py`），也有一层离线网关草案（`text_gateway.py`）能收 `$ASR,<文本>*<CK>\r\n` 帧并校验；ESP32 侧本轮 B 只落了**采音 + 封帧的骨架**（`i2s_mic.c` / `protocol.c`，未上板验证、只打印不下发）；**唤醒 / ASR / 真正把这句话发出去仍无代码、未接主链路**。

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
| 1 | ESP-IDF 工程骨架：`idf.py` 建工程、串口打印 + Wi-Fi 连接 | 🔶 **骨架已落盘（10 个文件）**；`protocol.c` 已用本机 gcc 编译 + S2 实机锚点帧反查通过；**整机 `idf.py build` 未跑、未烧录** | 本机无 `idf.py`；须在装好 ESP-IDF v5.x 的环境 build，先确认 CH343 下载电路正常 |
| 2 | INMP441 采音，I2S0 读原始 PCM，串口打印波形 | 🔶 驱动骨架已落盘（`i2s_std` 新 API），**未上板验证** | 先验证麦克风有数据，再谈识别；读出全 0 先查 L/R 引脚（接 GND = 左时隙） |
| 3 | ESP-SR / WakeNet 离线唤醒词 | 📋 无代码 | 模型放 Flash/PSRAM，`sdkconfig` 开 Octal PSRAM |
| 4 | 唤醒后录音 → 在线 ASR 或本地命令词 | 📋 无代码 | 在线需 Wi-Fi + API Key |
| 5 | 文本 → 封成 `$ASR,<文本>*<CK>\r\n` → 发鲲鹏 | 📋 无代码（鲲鹏侧接收草案已就绪） | **逐字节** XOR；帧格式是**建议草案**，未与 ESP32 实机对齐 |
| 6 | MAX98357A 回 TTS 提示音 | 📋 无代码 | I2S1 TX |

## 五、和现有代码的对接点

- 统一串口帧：`$<CMD>[,<ARG>...]*<CK>\r\n`，CK = payload 逐**字节** XOR、2 位大写 HEX。真源 `Core/Src/uart_protocol.c`（`proto_send_frame()` / `proto_parse_frame()`）；Python 侧参照 `kunpeng/llm/text_gateway.py` 的 `xor_checksum()`。
- S2 实机锚点帧：`$PING*10` → `$OK,PONG*3E`（`"PING"` XOR = `0x10`、`"OK,PONG"` XOR = `0x3E`）。任何新实现先拿这两帧反查，别自己手算。
- 文本入口草案帧：`$ASR,<文本>*<CK>\r\n`（`text_gateway.py` 建议）。🔴 这是**建议草案**，不是 STM32 ↔ 鲲鹏 `$JOINT` 链路，不进 `uart_protocol.c`，未与 ESP32 实机对齐、未接真串口。
- 通信方式二选一：**串口**（ESP32 UART ↔ 鲲鹏 USB-TTL，最简，与 STM32↔鲲鹏 同思路）；**Wi-Fi TCP**（ESP32 作客户端连鲲鹏，省一根线但多一层）。当前未选定、未实现。

## 六、当前目录结构（B 已落盘）

```
esp32/
├── README.md                  ← 本文件（口径 + 三行法 + 诚实边界）
├── CMakeLists.txt             ← ESP-IDF 顶层工程文件
├── sdkconfig.defaults         ← 默认配置（16MB Flash / Octal PSRAM / 1000Hz tick）
└── main/
    ├── CMakeLists.txt         ← 组件注册：4 个 .c + REQUIRES 五个官方组件
    ├── main.c                 ← app_main：连 Wi-Fi → 启 I2S → 封帧自检 → 循环读 PCM
    ├── i2s_mic.c / .h         ← INMP441 I2S0 RX 采 PCM（新 I2S 驱动）
    ├── wifi_sta.c / .h        ← Wi-Fi STA 连接骨架（事件组等 IP）
    └── protocol.c / .h        ← 统一帧 XOR 封包（与 STM32 对齐）
```

> 排障记录统一放工程根 `docs/排障记录-ESP32.md`（与 S3b / IoT / LLM 三个模块一致，个人复习材料不进 Git）。
> 什么时候往这个目录放**可用**代码：在装有 ESP-IDF 的机器上 `idf.py build` 通过、`idf.py flash monitor`（或第十节里写死串口号的写法）看到串口打印之后，再逐步往 `main/` 填 WakeNet / ASR / 封帧发送。

## 七、诚实边界（🔴 面试不能越线）

| ✅ 能说 | ❌ 不能说 |
|---|---|
| 「ESP32-S3 语音列为 P2；成品开发板 + 麦/功放/喇叭已可直接用，自画核心板仍待焊」 | 「语音控制已实现 / 已接 ESP32-S3」 |
| 「鲲鹏侧搭了离线文本入口网关草案，44 项单元测试全绿」 | 「语音唤醒 / 离线 ASR / 语音→动作闭环 / 自动抓取」 |
| 「串口帧协议用逐字节 XOR，与 STM32 固件对齐」 | 「`$ASR` 帧已是实机协议」 |
| 「ESP-IDF 工程骨架已落盘，`protocol.c` 本机 gcc 编译并用 `$PING*10` / `$OK,PONG*3E` 反查通过」 | 「已在 ESP32 上编译烧录成功 / 语音唤醒已实现 / 已接 ESP32 主链路」 |

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
③ 不检查 `out_size` 就是经典缓冲区溢出；按 Unicode 码点算中文（Python 侧常见错）会与固件判 `ERR,2`；`protocol_build_frame()` 若直接接 UART 写就变成「下发」，本骨架**故意只打印不下发**。

---

## 十、编译与烧录命令（标注执行位置）

> 🔴 前提：本机当前**没有** `idf.py`、没有 `~/.espressif`（已实测 `command -v idf.py` 无输出）。
> 下面命令要在**已安装 ESP-IDF v5.x 的 PC** 上跑；助手没有代跑、也没有声称编译 / 烧录成功。
> 跑 build 前先把 `esp32/main/wifi_sta.h` 里的 `WIFI_SSID` / `WIFI_PASSWORD` 改成自己的热点
> （PC 与鲲鹏板也要连同一个热点）；真实密码不要提交进 Git。
> 三条命令各自带 `Set-Location`，**单条复制也能独立跑**，不必依赖上一条的会话状态。

【PC · PowerShell】
```
Set-Location "D:\简历\STM32项目\4.ESP32-S3项目\RoboticArm_Kunpeng\esp32"; idf.py set-target esp32s3
```

【PC · PowerShell】
```
Set-Location "D:\简历\STM32项目\4.ESP32-S3项目\RoboticArm_Kunpeng\esp32"; idf.py build
```

【PC · PowerShell】
```
Set-Location "D:\简历\STM32项目\4.ESP32-S3项目\RoboticArm_Kunpeng\esp32"; idf.py flash monitor
```

> 若 `idf.py flash monitor` 自动找口失败，改用 `idf.py -p COM7 flash monitor`；
> `COM7` 要换成设备管理器里 CH343 的实际串口号，值按当前机器写死、每次重插重查。
> 看到 `I (xxx) voice_main: ==== ESP32-S3 语音入口骨架启动 ====`、
> `封帧自检（只打印不下发）: $ASR,mic_ok*50`，以及每秒打印的采样值，才算骨架在板上跑通。
