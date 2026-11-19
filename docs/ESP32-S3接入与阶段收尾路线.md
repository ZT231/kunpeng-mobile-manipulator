# 阶段收尾路线重排 · ESP32-S3 优先接入（2026-11-14）

> 🔴 **用户指令**：「补齐全部收尾项，跑 4 条实机判据，鲲鹏第二路 USB-TTL 实发帧验统一调度，**小车烧录先放到最后，先接入 ESP-S3**」。
> 本文是**路线 + 规程 + 本轮实证**，**不是实机结果**。三档标注 ✅实机 / 🔶有代码未实机 / 📋规划 一律不抬。
> 关联：`docs/第四项目整合说明.md` 第六节、`docs/核心化路线图-ESP32与IoT.md` 第七节、`docs/上板验收-语音网关阶段2.md`、`docs/ACS712-5A与J6限力夹持方案.md`。

---

## 零、先说清一件事：你给的清单里有一个依赖矛盾

| 你列的事 | 真正的前置条件 | 现在能不能跑 |
|---|---|---|
| 跑「4 条实机判据」（`car/README.md` 5.4） | **必须先把 `car/` 烧进 F407ZE**，车才收得到帧 | ❌ 顺延 |
| 鲲鹏第二路 USB-TTL 实发帧验「统一调度」 | 同上：车得能收帧、能动，才谈得上「统一调度」 | ❌ 顺延 |
| **先接入 ESP32-S3** | 鲲鹏侧网关 ✅ 已上板实机验证（16:40）；成品板在手 | ✅ **现在就能推进** |
| ACS712 + J6 限力夹持 | 见独立方案文档 | 📋 方案阶段 |

🔴 **结论**：「4 条判据」和「第二路 USB-TTL 统一调度」不是两件独立的事，它们是**同一个「A 项实机验收包」里、烧录之后跑的两组验证**。
你要求「小车烧录先放到最后」⇒ 这整个包顺延到最后一步。本轮**不假装跑过**，改为先把它的单行命令清单与验收记录模板备好（第五节），到那一步直接照跑。
现在真正推进的是 **ESP32-S3 接入**，以及 ACS712 的方案答复。

---

## 一、上一轮收尾项核对：已全部补齐，无遗漏

| 项 | 状态 |
|---|---|
| 限定抬档：只把「阶段 2 网关链路」🔶→✅，语音入口整体仍 🔶 | ✅ 已完成 |
| 残留旧档位口径订正 9 处（根 README 3 / robot README 4 / 路线图第七节 / 简历第五节+第六节 / 验收规程第八节 / MEMORY.md §8 消歧） | ✅ 已完成 |
| `docs/排障记录-语音网关.md` 补第 8 条正文（板端两个非失败误判点） | ✅ 已完成，现 8 条 |
| 简历第六节证据清单补第 **13** 条实机证据 | ✅ 已完成 |
| 当日日志 `2026-11-14.md` 追加「限定抬档收官」 | ✅ 已完成 |
| 临时文件清理（7 个：`_verify_*` / `_ck_*` / `_wg_audit_*`） | ✅ 已 `CLEAN` |
| 全局复核无残留旧口径、其它模块 🔶 未动 | ✅ Grep 通过 |
| 任务 `#33`~`#37` | ✅ 全部 completed |

---

## 二、新顺序（按你的优先级重排）

| 顺序 | 做什么 | 依赖烧录吗 | 档位 |
|---|---|---|---|
| **1（现在）** | **接入 ESP32-S3**：骨架整机 build 已通过 → 烧录 → TCP 上行鲲鹏 → 帧草案实机对齐 | ❌ | 🔶 整机 build 已通过、未烧录 / 未联调 |
| 2 | 阶段 0~1：ESP32 真发帧钉死「帧从板子发出来了」 | ❌ | 📋 |
| 3 | 最后才进 ESP-SR 命令词（唯一有技术不确定性的一段） | ❌ | 📋 |
| 4 | IoT 补 ESP32 节点遥测 + eKuiper 告警规则 | ❌ | 🔶 |
| 5 | ACS712 / J6 限力夹持（📋 方案已出，独立于主线） | ⚠️ 要接线 + 标定 | 📋 |
| **6（最后）** | **A 项实机验收包**：烧 `car/` → 跑 4 条判据 → 第二路 USB-TTL 统一调度 | ✅ **必须烧** | 🔶 未烧录 |

---

## 三、ESP32 接入：本轮本机实证（真跑，不是推断）

### 3.1 概念先讲清（大白话 + 规范定义）

| 概念 | 大白话 | 规范定义 |
|---|---|---|
| TCP 上行 | ESP32 像「打电话」一样连到鲲鹏的一个端口，把一句话发过去，对方回一句 | 传输控制协议（RFC 9293），面向连接、可靠字节流；本项目用 BSD socket `socket()/connect()/send()/recv()` |
| STA 模式 | ESP32 像手机一样去连路由器，不是自己开热点 | IEEE 802.11 infrastructure mode 的 station；连上后由 DHCP 拿 IPv4（`IP_EVENT_STA_GOT_IP`） |
| 帧草案对齐 | 两边说好的「一句话的格式」必须一模一样，否则对不上暗号 | 项目统一帧 `$<CMD>[,<ARG>...]*<CK>\r\n`，CK = payload 逐 **UTF-8 字节** XOR、2 位大写 HEX（真源 `Core/Src/uart_protocol.c`） |
| `--host 0.0.0.0` | 服务默认只许本机自己连；要让别的机器连，得把门开到所有网卡 | `bind()` 到 `INADDR_ANY`；默认 `127.0.0.1` 是 loopback，外部主机连接会被拒 |

### 3.2 实证 ①：ESP32 侧 C 封帧 vs 鲲鹏侧 Python 帧，**逐字节 5/5 一致**

本机用 `gcc -std=c99 -Wall -Wextra` 编译 `esp32/main/protocol.c`（它只依赖 `<stdint.h>/<stddef.h>`），把 C 侧 `protocol_build_frame()` 的输出与 Python 侧 `text_gateway.xor_checksum()` 逐字节比对：

| payload | CK | C 侧帧 | Python 侧帧 | 结果 |
|---|---|---|---|---|
| `ASR,mic_ok` | `50` | `$ASR,mic_ok*50\r\n` | 同 | ✅ PASS |
| `ASR,home` | `63` | `$ASR,home*63\r\n` | 同 | ✅ PASS |
| `ASR,回初始位` | `61` | `$ASR,\xe5\x9b\x9e...*61\r\n` | 同（含 UTF-8 多字节） | ✅ PASS |
| `ESTOP` | `5D` | `$ESTOP*5D\r\n` | 同 | ✅ PASS |
| `RESUME` | `19` | `$RESUME*19\r\n` | 同 | ✅ PASS |

✅ **结论 1**：骨架里 `protocol.c` 的封帧实现**语法层完全正确**，中文 UTF-8 逐字节 XOR 也与 Python 对齐（这正是 `docs/排障记录-LLM.md` 第 2 条一直在防的坑）。**帧草案的「格式」不用改。**

### 3.3 实证 ②：语义层有坑 —— 这是接入最先会撞上的

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

### 3.4 实证 ③：一个静态配置事实

`voice_gateway.py:137` → `DEFAULT_HOST = "127.0.0.1"`（注释原文：「默认只本机可连；要 ESP32 进来须显式 `--host 0.0.0.0`」）。
⇒ **ESP32 从 Wi-Fi 远程连过来，网关必须带 `--host 0.0.0.0` 启动**，否则连不上。这是接入时最容易漏的一条。

---

## 四、ESP32 接入三步（现在能推进的）

| 步 | 做什么 | 本机能否验 | 档位 |
|---|---|---|---|
| 1 | **用 PC 上现有的 ESP-IDF v5.5.5**（`D:\ESP32\Espressif`）→ `idf.py -p PORT flash monitor`，看串口打印出 `$ESTOP*5D` / `$RESUME*19` 与 PCM 采样 | ✅ 环境已就位（11-14 17:38 核对）；✅ 骨架**整机 `idf.py build` 11-14 晚已通过**（`Project build complete` / 产物齐全），工程路径含中文已拷 ASCII 副本 `D:\esp\s3_voice` | 🔶 已编译通过 / 未烧录 |
| 2 | 补 TCP 上行；鲲鹏网关 `--host 0.0.0.0`；先发 `$ESTOP*5D` / `$RESUME*19` 验「ESP32 → Wi-Fi → 网关 → 应答」 | ✅ 帧层面本轮已实证 | 🔶 有代码才验 |
| 3 | 再补 UTF-8 中文帧 `$ASR,回初始位*61` → `$OK,HOME*27`，`--apply` 下臂回初始位 | ✅ 16:40 已实证同形状帧 | 🔶 未与 ESP32 联调 |

### 4.1 代码清单（✅ **11-14 晚已全部落地，不是清单了**）

> 用户 11-14 晚明确：「在宿舍没硬件，身边只有鲲鹏板，能不能先把相关代码完成，等明天回教室直接烧录 / 上传实测」。
> ⇒ 本轮把「不依赖教室硬件、能本机验证」的部分全部写完并跑实证；**只有 `wifi_sta.h` 的真实热点密码要明天在教室本机改**（凭据不进 Git、不远程代填）。

| 文件 | 实际补了什么 | 落地状态 |
|---|---|---|
| `esp32/main/tcp_client.c/.h`（新） | BSD socket：`getaddrinfo` → `socket` → `setsockopt(SO_RCVTIMEO/SO_SNDTIMEO)` → `connect(鲲鹏IP, 9000)` → `send()` 循环发整帧 → `recv()` 收应答 → `close()`；**一次一连接**，失败返回分级错误码 `TCP_ERR_*`（不自动重连，见头文件「为什么一次一连接」） | ✅ 已落盘，含「大白话 + 规范定义」双行注释 |
| `esp32/main/main.c`（覆写） | Wi-Fi 拿到 IP 后调 `tcp_client_send()`；自检帧换成 **`ESTOP` / `RESUME`** 两条纯 ASCII 快速通道；中文帧 `$ASR,回初始位*61` 用 C 八进制字节串写好、由 `SEND_CN_HOME_FRAME=0` **默认关闭** | ✅ 已落盘；Wi-Fi 没连上也让本地封帧自检 + I2S 自检照跑，便于分层定位 |
| `esp32/main/CMakeLists.txt`（覆写） | `SRCS` 加 `tcp_client.c`；`REQUIRES` 补 `lwip` | ✅ 已落盘 |
| `esp32/main/wifi_sta.h` | `WIFI_SSID` / `WIFI_PASSWORD` 占位符**原样保留** | 🔴 **明天在教室本机改**（不提交 Git） |
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
   🔴 绕开 Git Bash 注入 `MSYSTEM` 导致官方拒 `MSys/Mingw is no longer supported` 的做法见 `docs/排障记录-ESP32.md` 第 3 条；
   **整机 build 已回读确认通过**（证据与产物清单见 `docs/明日回教室-ESP32照跑清单.md` 第一节）；🔴 编译通过 ≠ 实机，仍未烧录。

> 🔴 **订正 11-14 早先的过时口径**：本节原文写「本机没有 ESP-IDF，这些代码写了也编译不了，本轮不落地代码」——
> 已被 17:38 的环境核对与今晚的实际 build **证伪并撤回**。PC 上 `D:\ESP32\Espressif` 就是完整 v5.5.5，
> 只是必须走 ESP-IDF PowerShell / cmd 或干净 env 的 Python 通道，Git Bash 直接跑会被官方拒。
> ✅ **明天回教室直接照跑的单行清单已单独落盘**：`docs/明日回教室-ESP32照跑清单.md`（确认 build 已通过 → 改热点 / 网关 IP → 起鲲鹏网关 `--host 0.0.0.0` → `flash monitor` → 四层硬判据 → 可选中文帧加试 → 记录表 / 失败分流）。
> 🔴 **第一步的指令已单独落盘**：`docs/ESP-IDF安装与首次build指引.md`（**11-14 17:38 已订正为「环境已就位、直接 build」**：PC 上的 `D:\ESP32\Espressif` 就是完整 v5.5.5，不用再装；含启动方式、验版本、build / flash 命令、判据表、失败分流）。
> 🔴 **不是装在鲲鹏板上**：ESP-IDF 是 PC 上的交叉编译/烧录工具链（USB 要摸到板载 CH343）；鲲鹏板只跑 `web_control.py` / `voice_gateway.py` 等运行时。
> 🔴 **中文路径坑（必看）**：安装路径 `D:\ESP32\Espressif` 本身合规，但工程在 `D:\简历\STM32项目\...\esp32\` 含中文 ⇒ **先把 `esp32/` 拷到 `D:\esp\s3_voice` 再 build**，原工程不动；`esp32/README.md` 第十节已按此订正。
> 🔴 **别在 Git Bash 里跑**：实测 `idf.py` 回 `MSys/Mingw is no longer supported` ⇒ 必须开 **ESP-IDF PowerShell / cmd 快捷方式**。

### 4.2 鲲鹏侧可粘贴命令（沿用 `docs/上板验收-语音网关阶段2.md` 已实证写法，只多一个 `--host`）

前置：Flask 必须在跑（网关靠 `/api/cmd` 下发）。判活只认 `curl /api/ik/eye` 的 `enabled:true`；启动必用绝对路径 `/root/kunpeng_vision/web_control.py` + `python3 -u`，**不带 `--radial-fix`**。

```bash
# ① 结构自检（零硬件、不碰臂）
python3 -u /root/kunpeng_vision/robot/voice_gateway.py --host 0.0.0.0 --port 9000 --self-test

# ② 先 dry-run 起网关，只验「帧能不能进来」，不下发
python3 -u /root/kunpeng_vision/robot/voice_gateway.py --host 0.0.0.0 --port 9000 > /root/gw.log 2>&1 &

# ③ 从另一台机器（PC / ESP32 所在网段）探两条纯 ASCII 快速通道
printf '$ESTOP*5D\r\n' | nc 10.160.120.1 9000
printf '$RESUME*19\r\n' | nc 10.160.120.1 9000

# ④ 链路通了再停掉 dry-run，改 --apply 真下发（此时要先摆 C 姿态，见验收规程）
pkill -f voice_gateway.py
python3 -u /root/kunpeng_vision/robot/voice_gateway.py --host 0.0.0.0 --port 9000 --apply > /root/gw.log 2>&1 &
```

🔴 板端命令一律**单行逐条粘**；`printf` 能解释 `\ooo` / `\r` / `\n`，`echo` 默认不解释转义，`nc` 不通时用 Python `socket.create_connection` 一行兜底（写法见验收规程 3.3）。

---

## 五、A 项实机验收包（烧录放最后，规程先备好）

### 5.1 烧录（最后一步才做）

打开 `car/demo.uvprojx`（`MAIN` 组已含 `car_frame.c`），Keil Build 后下载到 **F407ZE**。
🔴 两套库不能混编：小车是 Keil+SPL（**无 `.ioc`**），机械臂是 CubeMX+HAL+FreeRTOS（`.ioc` 是 F407VE）。**本次不动任何 `.ioc`** —— 改了要 Generate Code + 重烧，会甩臂、破坏已验收链路。

### 5.2 4 条实机判据（原文照抄 `car/README.md` 5.4，🔶 待做）

| # | 发什么 | 期望 |
|---|---|---|
| 1 | `$PING*10\r\n` | 回 `$OK,PONG*3E\r\n` |
| 2 | `$CAR,FWD,800*3D\r\n` | 车前进约 **0.8 秒后自动停** |
| 3 | 走的过程中发 `$CAR,STOP*64\r\n` | **立即停**（证明非阻塞定时有效） |
| 4 | 手机蓝牙 APP 按 `F` / `S` | 仍能控车（证明老单字符协议没被弄坏） |

### 5.3 第二路 USB-TTL 统一调度（与 5.2 同一包，🔶 待做）

| 项 | 值 |
|---|---|
| 接线 | USB-TTL **TX→PA10 / RX→PA9 / GND 共地**（`car/README.md` 第四节方案 B） |
| 🔴 串口隔离 | `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` 已给机械臂，**底盘必须另开一路**，别抢 |
| 命令 | `python3 kunpeng/car/car_gateway.py --port /dev/ttyUSB1 --cmd FWD --ms 800` |
| 判据 | 车按帧动作 + 鲲鹏侧日志显示帧已发出 + 机械臂那路串口同时不受干扰 |

> `800` 是**毫秒不是厘米**，纯开环定时（无编码器 / 里程计），长距误差 20~40%。

### 5.4 验收记录模板（跑完填，别提前打勾）

| # | 判据 | 期望 | 实测 | 通过 |
|---|---|---|---|---|
| 1 | `$PING*10` | `$OK,PONG*3E` | ☐ | ☐ |
| 2 | `$CAR,FWD,800*3D` | 前进 0.8s 后自动停 | ☐ | ☐ |
| 3 | 行进中 `$CAR,STOP*64` | 立即停 | ☐ | ☐ |
| 4 | 蓝牙 `F`/`S` | 老协议仍可用 | ☐ | ☐ |
| 5 | 第二路 USB-TTL 发帧 | 车动 + 机械臂串口不受扰 | ☐ | ☐ |

🔴 **档位**：`car/` 目前 🔶 未烧录 / 未实机联调 / 未装车。只有这张表全绿并回报，才按 `docs/第四项目整合说明.md` 第八节统一抬档；**「第四个完整项目」是项目定位，不是已完成**。

---

## 六、诚实边界 / 命名红线

- 🔴 本轮**未烧录、未上板、未联调**；11-14 晚已在 PC 上跑整机 `set-target esp32s3`（rc=0）+ `idf.py build` **并回读确认通过**（`BUILD_RC=0` / `Project build complete` / 产物 4 件齐全 / `tcp_client.c` 链接成功；首次全量编译曾触发 20 分钟 timeout，增量续跑通过，**不是编译失败**），新增代码全是 🔶 有代码未实机；**不许因「先接入 ESP32 / 编译通过」把语音入口整体抬成 ✅**。
- 🔴 鲲鹏侧阶段 2 网关链路是 ✅，但 `source` 恒 `mock`、**没接 ESP32**；`$ASR` 帧草案**未与 ESP32 实机对齐**（本轮只做到 C/Python 语法层逐字节一致，语义层仍待实机）。
- 🔴 禁写「语音控制已实现 / 语音→动作闭环 / 已接 ESP32 / 自然语言控制机械臂」；只叫「可观测性闭环 / 视觉引导的移动操作平台」。
- ✅ 能写：「ESP32-S3 侧的封帧实现与鲲鹏侧网关在逐字节层面一致，并用纯 ASCII 快速通道完成了链路联通准备」。

---

## 七、README 三行法（本轮新增的「帧草案对齐」验证）

- **① 用了什么语法**：`gcc -std=c99 -Wall -Wextra` 把只依赖标准 C 头的 `protocol.c` 编进本机 harness；Python 侧 `xor_checksum()` 逐 UTF-8 字节 XOR；`socket.create_connection()` 起真 TCP 连到 `ThreadingTCPServer`，`recv()` 收应答帧。
- **② 为什么这么设计**：跨语言协议最怕「各自算各自都对、接起来全错」——把 **C 侧真源**编译出真实字节流，再和 Python 侧逐字节比对，比两边各自跑单测硬得多；而且先用纯 ASCII 帧验链路、再验中文语义，能把「编码问题」和「连通问题」分开定位。
- **③ 不这么写会怎样**：只看骨架打印的 `$ASR,mic_ok*50` 就以为「帧做好了」，接上去一律 `$ERR,2`；用英文 `home` 也会被拒；忘了 `--host 0.0.0.0`，ESP32 从 Wi-Fi 连过来直接 `Connection refused`，会被误判成「Wi-Fi 没连上」。
