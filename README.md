# 六自由度机械臂视觉抓取系统（STM32F407 + FreeRTOS）

基于 STM32F407VET6 的六自由度机械臂平台，配合 KunPeng Pro（openEuler）边缘端与网页控制台，
打通 **视觉识别 → 平面坐标解算 → 逆运动学 → 串口下发 → 状态回读** 链路。

> 🔴 **项目定位（2026-11-14 定稿）**：本目录即简历上的**第四个完整项目** ——
> 在**前三个项目**（`1.Smart car` 智能小车 / `2-1.Robotic arm vision` 机械臂视觉 /
> `3.C++ 跨平台串口协议引擎`）的基础上，加入**鲲鹏开发板 + ESP32-S3 + IoT 可观测性闭环**，
> 整合为「云-边-端移动操作平台」。项目 2、3 早已合并进本工程，2026-11-14 把项目 1 的
> 底盘以「拷贝 + 加统一帧层」的方式补进 `car/`，最后一块拼图到位。
> 完整映射表、CubeMX 结论与简历说辞见 **`docs/第四项目整合说明.md`**。
> 🔴 **本次没有改任何 `.ioc`**：小车是 Keil+SPL 工程、机械臂是 CubeMX+HAL 工程，
> 按两个 MCU 节点各自保留、靠 `$<CMD>*CK` 统一帧通信（详见该文档第四节）。

> 固件：Keil MDK（CubeMX 生成框架 + 手写模块）｜ 边缘端 / 上位机：Python + OpenCV + Flask
>
> 🔴 **状态口径（2026-11-08 收口）**：本 README 混有「旧 STM32 单板项目能力」与「当前 Kunpeng 云-边-端工程已实证能力」两代内容。
> 三档划分与**可直接用于简历的正文**，以 `docs/简历-具身智能项目条目.md` 为准：
> ✅ 已实机实证 / 🔶 代码已有但未实机验证或未接入主链路 / 📋 纯规划占位。
> 下文凡标 🔶 / 📋 的功能**不得写成已完成**。

---

## 主要功能

### ✅ 当前 Kunpeng 工程已实机实证

- **串口闭环**：USART1（PA9/PA10，115200 8N1）自定义帧 `$<CMD>[,<ARG>...]*<CK>\r\n`（XOR 校验、2 位大写 HEX），
  `PING` / `STATE` / `HOME` / `JOINT` 实机通过（`$PING*10` → `$OK,PONG*3E`；`STATE` 回读六路实际脉宽与源码初始位一致）。
- **六路舵机安全执行**：PCA9685（I2C1 PB6/PB7，50Hz）驱动六路，逐轴软限位 clamp + 25µs/20ms 斜坡限速；
  `main.h` / `web_control.py` / `docs/舵机标定记录.md` 三方数值同步。
- **边缘视觉服务**：`detector_service.py` :8080 跑 YOLOv8n + MJPEG 推流 / `/detections` / `/health`；
  B 方案抽帧推理 + C 方案 V4L2/MJPG 采集优化后，实测约 **26.6 FPS**（原 6.8 FPS）。
- **网页控制台 + 点选抓取**：`web_control.py` :5000 内嵌单页，滑杆控六路、1s 状态回填、
  点画面取点 → 平面约束单目定位 → `ArmIK` 解六路脉宽 → 软限位检查 → `JOINT` 下发 → `STATE` 回读；
  标定参数常驻面板 + 浏览器本地预设（保存 / 切换 / 删除）。
- **板内 IoT 可观测性闭环（1.2b）**：`iot/` 真实串口遥测 → MQTT → EMQX → eKuiper → InfluxDB → Grafana，
  四容器板上跑通、Grafana 六路 `j1~j6` 曲线已出（16:07 `records_in=records_out=59` / `exceptions=0`）；
  `deploy.sh` 动态重绑 firewalld trusted zone、`telemetry_bridge.py` 发布端拦 null 点；收口 `e6cd194`。
  🔴 只叫「可观测性闭环」，不是云端 IoT、不增抓取能力。

### 🔶 旧 STM32 单板项目已有，当前 Kunpeng 工程未重新实机复验

> 旧版演示脚本与测试方法保留在 `docs/完整功能演示流程.md`、`docs/视觉识别测试清单.md`（历史归档，命令不可直接照跑）。

- **视觉引导抓取（旧版）**：USB 摄像头 HSV 颜色阈值 + 轮廓检测识别目标，四点 Homography 完成像素→毫米换算，平面几何逆运动学解算 6 路舵机脉宽；按目标形状自适应夹爪方向（沿边抓取），远目标自动前倾扩臂（上限 45°）。
- **多通道控制**：串口（PC 上位机）/ 蓝牙（HC-05）/ WiFi（ESP-01S）/ RFID（RC522）四通道复用同一套 `$CMD*CK\r\n` 协议（XOR 校验）。代码在 `Core/Src/bt.c`、`wifi.c`；RC522 当前未接芯片（`version 0xFF`）。
- **模式状态机**：MANUAL（电位器手控）/ CMD（待命）/ RECORD（示教录制）/ PLAYBACK（回放）/ ESTOP（急停）五态；急停冻结输出，仅 RESUME 可解除。
- **示教回放**：上位机 REC 采集轨迹，PLAY 一键回放。

## 硬件平台

| 部件 | 型号 / 说明 |
|---|---|
| 主控 | STM32F407VET6 |
| 舵机驱动 | PCA9685（I2C，16 路 50Hz PWM） |
| 电位器 | 6 路 ADC 采样（DMA 循环 + 滑动平均 + 回滞死区） |
| 摄像头 | USB 摄像头（上位机 OpenCV 处理） |
| 无线 | ESP-01S（WiFi）、HC-05（蓝牙） |
| 刷卡 | RC522（GPIO 软件 SPI） |

## 软件架构

固件基于 **FreeRTOS（CMSIS-RTOS v2）** 划分三个任务，按实时性分级：

| 任务 | 周期 | 优先级 | 栈 | 职责 |
|---|---|---|---|---|
| `StartServoTask` | 20ms | High | 4096 | 电位器/按键/RFID 采集、示教、`Arm_Update()` 刷新 6 路舵机 |
| `StartProtocolTask` | 5ms | Normal | 2048 | 解析 UART 帧、分发命令、执行运动 |
| `StartPrintTask` | 3s | Low | 2048 | 周期遥测打印 |
| `StartSafetyTask` | 200ms | Low | 2048 | 过流检测（ACS712 采样跨 20ms，独立任务不拖慢舵机）→ `Arm_ReportFault` |

- **中断只收字节**：ISR 内仅做单字节接收与帧状态机，命令解析与执行全部放在任务上下文（中断优先级 5，匹配 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`）。
- **两把互斥锁**：`arm_lock`（机械臂状态）、`print_lock`（串口输出），从不同时持有，避免死锁。
- **协议帧格式**：`$<CMD>[,<ARG>...]*<CK>\r\n`，`CK` = CMD 字段（`$` 与 `*` 之间）所有字节的 XOR，2 位大写十六进制；响应 `$OK[,<数据>]` / `$ERR,<code>`。
- **双缓冲应答**：活动槽 + 备用槽，临界区保护，保证连发命令的应答不丢失。

### 阶段 1 安全监控（safety 模块 + FAULT 状态机，2026-11-19 新增）

> 🔶 代码已落盘，**未上板编译/实机验证**（本地无 Keil，需回教室编译）。IWDG 与断链保护按决策延后，本阶段只做了「FAULT 状态机 + 过流急停」两项无砖级风险的纯软件改造。

- **① 用了什么语法/设计**：`arm_control.h` 状态机枚举新增 `ARM_STATE_FAULT`；`arm_control.c` 加 `static uint8_t fault_code`（故障码锁存）+ `state_is_frozen()`（把「急停或故障」的冻结判断收敛成一个函数）+ `Arm_ReportFault/ClearFault/GetFault/IsFrozen` 四个接口；新增 `safety.h/safety.c`（`Safety_Tick()` 读 ACS712、连续 3 次超 800mA 才触发）；`uart_protocol.c` 加 `$FAULT`/`$CLEARFAULT` 命令、STATE 响应末尾追加 fault 字段；`freertos.c` 加 `StartSafetyTask`。
- **② 为什么这么设计**：急停像「按暂停」、故障像「机器亮红灯」，必须分开——故障需人工诊断后才能复位，不能被 RESUME 一句话带过去。过流检测放**独立低优先级任务**是因为 `ACS712_Read_mA()` 跨 PWM 周期采样会阻塞约 20ms，塞进 20ms 的舵机任务会把刷新拖慢一倍。STATE 的 fault 字段**追加到末尾**（而非插中间）是为了向后兼容 `arm_host.py`/`web_control.py` 的 `parts[3:9]` 精确切片解析。
- **③ 不这么写会出什么问题**：若把 ESTOP/FAULT 混成一个态，过流异常会被 RESUME 带过 = 保护形同虚设；若在 `Arm_Update`/各 Command 里各自写 `==ESTOP || ==FAULT`，漏一处就是「急停不住」的安全漏洞；若 STATE 把 fault 插进字段中间，老上位机解析直接崩。

## 三大核心支柱（项目定位定稿，2026-11-13）

> 🔴 **「核心必备」的唯一判据 = 能演示 + 能讲清边界。** 三根支柱缺一不可：
> 任何一根只能按实际情况标 ✅ / 🔶 / 📋，**不允许因为「它是核心」就写成已完成**。
> 顺序即优先级：支柱 ① 已站起来，② 是当前最大缺口，③ 已实证但覆盖面待扩。

| # | 支柱 | 载体 | 职责 | 代码位置 | 当前状态 |
|---|---|---|---|---|---|
| **①** | **视觉引导执行**（端 + 边） | STM32F407VET6 + KunPeng Pro | 网页点选 / 眼在手定位 → `ArmIK` 解 6 路脉宽 → 串口 `$JOINT` → PCA9685 → `STATE` 回读 | `Core/`、`kunpeng/robot`、`kunpeng/vision`、`kunpeng/web` | ✅ 已实机；🔴 定位径向有 2.9cm 系统偏近（根因已定案，修法待落地） |
| **②** | **语音入口** | **ESP32-S3**（成品板 N16R8）+ 鲲鹏侧收帧服务 | WakeNet 唤醒 + 命令词识别 → 封帧 `$ASR,...*CK` → Wi-Fi 上行鲲鹏 → **本地仲裁 → 固件原生命令字** | `esp32/`、`kunpeng/robot/voice_gateway.py` | 🔶 **P0 核心，整体进行中**：IDF 骨架 **12 文件**（11-14 晚补 `tcp_client.c/.h` Wi-Fi TCP 上行 + 自检帧改纯 ASCII 快速通道），本机帧/网关实证 `PASS=24 FAIL=0`、PC 上整机 `set-target`（rc=0）+ `idf.py build` **已通过**（`BUILD_RC=0` / `Project build complete` / 产物齐全，**证据见 `docs/明日回教室-ESP32照跑清单.md`**）；🔴 **未烧录 / 未上板 / 未联调**；✅ **B 项阶段 2 鲲鹏侧网关链路 11-14 16:40 已上板实机验证**（38 项单测 + 四层硬判据全绿，见 `docs/上板验收-语音网关阶段2.md`）；唤醒/命令词/ESP32 遥测仍无代码 |
| **③** | **可观测性闭环** | KunPeng Pro（板内四容器） | 节点遥测 → MQTT → EMQX → eKuiper → InfluxDB → Grafana | `iot/` | ✅ 板内已实证：16:07 `records_in=records_out=59` / `exceptions=0`、Grafana 六路 `j1~j6` 曲线；🔶 **尚未覆盖 ESP32 节点、无告警规则** |

**目标主流程（三支柱联动，🔶 差最后一段）**：
ESP32-S3 听语音 → 鲲鹏意图解析 + `ArmIK` 算 6 路脉宽 → 串口 `$JOINT` → STM32 驱动 PCA9685 → `STATE` 回读；
全过程六路脉宽与三节点健康度进 EMQX → Grafana。

- **✅ 已打通**：浏览器点选 → 眼在手定位 → `ArmIK` → `$JOINT` → PCA9685 → `STATE` 回读。
- **🔶 待打通**：ESP32-S3 语音帧 → 鲲鹏收帧服务。✅ **鲲鹏侧 `voice_gateway.py` 已落盘 + 38 项本机单测全绿 + 11-14 16:40 上板 `--apply` 实机验收四层硬判据全绿**（TCP 收帧 → 帧校验 → 急停快速通道 → 本地仲裁 → `/api/cmd` 下发，默认 dry-run、只授权三个无参数命令字；假帧驱动臂回初始位、`PICK` 被拒、急停可冻结）；🔴 仍缺「ESP32 真的把帧发出来」这一段（11-14 晚已补 `tcp_client.c` 上行代码并本机验证，**但没上板**），`$ASR` 帧**语法层**已本机逐字节对齐、**语义层实机对齐待明天**，语音入口整体档位仍是 🔶。`text_gateway.py` 仍只做离线契约验证、不下发。
- **✅ 已打通**：STM32 经鲲鹏的串口遥测 → 四容器 → Grafana 六路曲线。
- **🔶 待打通**：ESP32 自身遥测（RSSI / 堆 / 唤醒计数）进 EMQX，让支柱 ③ 真正覆盖三节点。

协议仍是一套：STM32 固件 / Python `arm_host.py` / C++ 引擎 / ESP32-S3 四方一致，
帧格式 `$<CMD>[,ARGS]*<CK>\r\n`，CK = payload 逐字节 XOR、2 位大写 HEX。

当前进度：

| 环节 | 状态 |
|---|---|
| STM32 固件（协议 + 六路舵机 + 软限位 + 斜坡） | ✅ 实机通过：`$PING*10` → `$OK,PONG*3E`，`STATE` 回读六路脉宽 |
| 鲲鹏 YOLOv8 识别 + MJPEG 推流 | ✅ 板上实测约 26.6 FPS（B 抽帧推理 + C V4L2/MJPG 采集优化后） |
| 鲲鹏网页控制台 + 点选抓取 | ✅ 5000 单页：滑杆控 6 路、点选→IK→`JOINT`→`STATE`、标定参数常驻面板 + 本地预设 |
| **支柱③** `iot/` 可观测性闭环 | ✅ 真实串口遥测 → MQTT → EMQX → eKuiper → InfluxDB → Grafana，四容器板上跑通、Grafana 六路曲线已出；收口 `e6cd194`。🔶 缺口：未覆盖 ESP32-S3 节点、eKuiper 无告警规则（🔴 不是云端 IoT / 不增抓取能力） |
| 手眼标定（眼在手，AX=XB） | ✅ **已实标**（12 组姿态）：`resid` 16.7mm→**1.60mm**、`axis_cond` 0.519、`|t|`=16.7cm≈L3、棋盘不动 P 极差 0.30/0.51/0.36cm。`s`=0.7549 已实测拆分（相机侧 9.5% + J1 map 伪折点 21%，🔶 已定位未修）。**A+ 眼在手点选链路代码已有（`--eye-in-hand`），🔶 未用实标 X 实机验证** |
| 蓝牙 / WiFi / RFID 多通道、示教回放 | 🔶 旧项目源码已有，当前工程未重新实机复验（RC522 当前未接） |
| `kunpeng/llm/` 意图解析 + 离线文本入口网关 + **VLM 决策层** | 🔶 `intent_parser.py` + `text_gateway.py`，44 项 unittest 全绿；🆕 `vlm_decider.py`（阶段 6 / P2）8 组自测过 —— **模型只选编号、不输出坐标**，越界编号拒绝执行。**均只输出决策、不下发**、未接真模型 / 未接主链路；`$ASR` 帧是建议草案 |
| **支柱②** ESP32-S3 语音入口 | 🔶 **P0 核心在办**：IDF 骨架 **12 文件**（11-14 晚补 `tcp_client.c/.h` Wi-Fi TCP 上行层、`main.c` 自检帧改 `$ESTOP/$RESUME`、中文帧 `SEND_CN_HOME_FRAME=0` 闸门），本机帧/网关语义实证 **`PASS=24 FAIL=0`**、PC 上整机 `set-target`（rc=0）+ `idf.py build` **已通过**（`BUILD_RC=0` / `Project build complete` / 产物齐全，证据见 `docs/明日回教室-ESP32照跑清单.md`）；🔴 **未烧录 / 未上板 / 未联调**。待补：WakeNet 唤醒、命令词识别、ESP32 遥测进 EMQX。✅ 硬件就绪（成品板 CH343/N16R8 + INMP441 + MAX98357A + 3W 喇叭）。👉 **11-14 用户重排优先级后本项排第一**，C 侧 `protocol.c` 封帧已与鲲鹏侧逐字节 5/5 一致，路线见 `docs/ESP32-S3接入与阶段收尾路线.md` |
| **支柱② 鲲鹏侧收帧服务（B 项 / 阶段 2）** | ✅ **11-14 16:40 上板 `--apply` 实机验收通过**：`kunpeng/robot/voice_gateway.py` 已落盘 + **38 项 unittest 全绿** + 四层硬判据全绿（假帧驱动臂回初始位、`PICK` 被拒、急停冻结/恢复）；独立进程不改 `web_control.py`、不直连串口，只经 `/api/cmd` 下发固件原生 `HOME/STOP/RESUME`；🔴 **仍未与 ESP32 联调**（`$ASR` 帧**语法层**已与 ESP32 `protocol.c` 本机逐字节对齐，**语义层实机对齐待明天**），`source` 恒 `mock`。记录见 `docs/上板验收-语音网关阶段2.md`，排障见 `docs/排障记录-语音网关.md` |
| 双 STM32 分工、双相机 6D 位姿、LLM 规划、OTA、云端 IoT、数字孪生 | 📋 纯规划，**无代码** |

> 状态口径与可直接投递的简历正文见 `docs/简历-具身智能项目条目.md`；写简历 / 面试时的红线见 `docs/排障记录-S3b.md`「本阶段诚实边界」。

## 目录结构

本工程已从「单一 STM32 桌面机械臂」升级为「**云-边-端**移动操作机器人」的单一工程根目录，
STM32 / 鲲鹏 / ESP32-S3 三块板的代码与全部文档都收在这一个文件夹里。

```
RoboticArm_Kunpeng/
├── Robotic arm vision.ioc       CubeMX 工程文件（打开它即可重新生成代码）
├── Core/                        ★STM32 固件源码（CubeMX 标准结构，勿改目录名）
│   ├── Inc/  Src/               main / usart / bt / wifi / uart_protocol /
│   │                            pca9685 / arm_control / teach / freertos 等
│   ├── Drivers/                 ST HAL 库（ST 官方）
│   ├── Middlewares/             FreeRTOS + CMSIS（官方）
│   └── MDK-ARM/                 Keil 工程（Robotic arm vision.uvprojx）
├── kunpeng/                     ★鲲鹏 KunPeng Pro 边缘端（openEuler + Python）
│   ├── vision/                          YOLOv8 识别链路
│   │   ├── stage1_yolov8.py     旧版识别主程序（双后端 + --imgsz 320 约 7FPS）
│   │   ├── detector_service.py  常驻 HTTP/JSON 识别服务（:8080/detections、/observe 原子快照）
│   │   ├── a_vision_demo.py     颜色追踪 / KCF 追踪演示
│   │   └── deploy_stage1.sh     依赖与权重部署脚本
│   ├── robot/                   机械臂解算与控制
│   │   ├── arm_ik.py            逆/正运动学（ArmIK）
│   │   ├── arm_params.json      舵机标定表
│   │   ├── arm_host.py          串口发帧上位机（$JOINT/$MOVE/$STATE）
│   │   ├── handeye_calib.py     ✅ 手眼标定（AX=XB）：已实标 resid 1.60mm；A+ 点选链路待实测
│   │   ├── eye_check.py         🆕 眼在手点选自检：固定棋盘 + 换姿态，看算出的坐标重不重合
│   │   ├── action_primitives.py 🆕 动作原语：pick/place/home，每步回读关节角确认到位（🔶 离线全绿，未真机跑）
│   │   ├── test_action_primitives.py 🆕 动作原语回归测试：假 Flask 模拟斜坡/拒收/卡死/未确认
│   │   ├── voice_gateway.py 🆕 鲲鹏侧语音入口网关（阶段 2 / B 项）：TCP :9000 收帧 → 本地仲裁 → /api/cmd；默认 dry-run（✅ 38 项单测 + 11-14 16:40 上板 --apply 实机验收，阶段 2 链路已通；语音入口整体仍 🔶 未与 ESP32 联调）
│   │   └── test_voice_gateway.py 🆕 网关回归考卷：38 项 unittest，不动臂不联网
│   ├── web/                     ✅ web_control.py 网页遥控（Flask + 滑杆 + 点选抓取）
│   ├── llm/                     🔶 意图解析 + VLM 决策层（vlm_decider.py 只选编号不输出坐标，8 组自测过）
│   ├── car/                     🆕 car_gateway.py 底盘统一帧网关（串口 / TCP / dry-run，🔶 未接主链路）
│   └── docs/                    stage1_README.md 识别部署说明、A档执行计划
├── car/                         🆕 项目 1 底盘合并（STM32F407ZE / Keil+SPL）：统一帧 `$CAR,...*CK` + 14 项 host 单测；🔶 未上板，见 car/README.md
├── esp32/                       ★支柱② ESP32-S3 语音入口（P0 核心）：12 文件骨架（含 11-14 晚新增 tcp_client.c/.h Wi-Fi TCP 上行）；成品板已可用、软件未烧录未上板；口径见 esp32/README.md
├── iot/                         ✅ 板内可观测性闭环（EMQX→eKuiper→InfluxDB→Grafana + 真实串口遥测）：已实证，收口 e6cd194
├── protocol_engine_cpp/         项目 3：C++ 跨平台串口协议引擎（243 断言，三方协议金标准）
└── docs/                        全部文档
    ├── 核心化路线图-ESP32与IoT.md       三大支柱 / 阶段划分；第七节按 11-14 用户优先级重排（先接 ESP32，烧录放最后）
    ├── 上板验收-语音网关阶段2.md 🆕 板端单行命令 + CK 速查表 + 四层硬判据 + 16:40 实跑记录表（已归档）
    ├── ESP32-S3接入与阶段收尾路线.md 🆕 11-14 优先级重排 / C-Python 帧逐字节实证 / A 项验收包规程
    ├── ESP-IDF安装与首次build指引.md 🆕 现有环境 v5.5.5 核对（PC，不装鲲鹏板）/ 中文路径坑 / build+flash 单行指令 / 判据表
    ├── 明日回教室-ESP32照跑清单.md 🆕 11-14 晚「今晚写代码、明天烧录实测」的可照跑规程：build 结论回读 / 改热点与网关 IP / 起网关 / flash monitor / 四层硬判据 / 可选中文帧加试 / 记录表
    ├── ACS712-5A与J6限力夹持方案.md 🆕 📋 限力夹持保护方案（只串 J6 单路 / ADC1 注入通道 / 不改 .ioc）
    ├── 一车全载-手眼标定口径订正与落地路线.md / 过时口径清理记录-2026-11-16.md 🆕 双工作方式并存顶层框架（标题已改、文件名保留）+ 已作废口径清单（动手前先扫）
    ├── 第四项目整合说明.md 🆕 前三个项目 → 第四个项目的映射 / CubeMX 结论 / 简历说辞
    ├── 排障记录-语音网关.md 🆕 B 项 / 阶段 2 的 8 条排障（字面量 \r\n、急停快速通道、下发容错、多线程锁、$JOINT 取舍、ASCII stdout、解除急停歧义、板端验收两个非失败误判）
    ├── 项目实施方案.md / 硬件清单.md / 低预算能力清单（功能边界）.md 等方案类
    ├── 串口联调测试清单.md / 舵机标定记录.md / 多通道控制接入指南.md 等实测类
    └── pdf_util/                演示 PDF 辅助工具
```

> **命名约定**：`Core / Drivers / Middlewares / MDK-ARM` 是 CubeMX + Keil 的**强制约定目录名**
> （`.ioc` 必须与 `Core`、`Drivers` 同级，Keil 工程内 100% 使用 `../Core/...` 相对路径），
> 改名会让 CubeMX 和 Keil 都找不到文件，因此保持原名——这不是"没改"，是**不能改**。

## 快速开始

### 1. 固件
Keil 打开 `MDK-ARM/Robotic arm vision.uvprojx` 编译下载。

### 2. 鲲鹏端（上位机 / 视觉 / 网页）
把 `kunpeng/` 整个目录拷到开发板（openEuler 22.03），在板上执行：
```bash
pip3 install flask pyserial          # 板端已有 ultralytics / opencv / numpy

# 网页遥控：手机/电脑浏览器打开 http://<板IP>:5000，滑杆控 6 路舵机
python3 kunpeng/web/web_control.py /dev/ttyUSB0 --baud 115200 --host 0.0.0.0 --port 5000

# 眼在手点选（A+ 方案）：用「内参 K + 实标手眼 X + 当前 STATE 脉宽」把像素反投影成 3D 点再逆解，
# 换任何臂姿都能定位（A 方案 pixel_to_table 只对固定拍照姿态有效）
python3 kunpeng/web/web_control.py /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
        --host 0.0.0.0 --port 5000 --eye-in-hand

# 常驻识别服务：HTTP :8080/detections
python3 kunpeng/vision/detector_service.py --source 0 --imgsz 320 --port 8080
```

### 3. 眼在手标定（✅ 已实标：resid 1.60mm；A+ 点选链路待用实标 X 实机验证）
```bash
python3 kunpeng/robot/handeye_calib.py selftest               # 自检正运动学，无需相机
python3 kunpeng/robot/handeye_calib.py intrinsics --camera 0  # 标定相机内参（640×480）
python3 kunpeng/robot/handeye_calib.py collect --camera 0 --arm-serial /dev/ttyUSB0 \
        --intrinsics camera_intrinsics.json --square 0.0082 --out samples.json
python3 kunpeng/robot/handeye_calib.py solve --samples samples.json --out handeye_X.json
```

> 🔴 **改过正运动学就必须先用 `refk_samples.py` 重算样本**：`collect` 落盘的是**算好的 4×4 矩阵**，
> `solve` 只读矩阵、不读 `pulses` ⇒ 直接重跑 `solve` 输出会**一字不变**（踩过：FK 姿态符号改完，`resid` 仍报 16.7mm）。
> 正确顺序：`refk_samples.py samples.json --out samples_fkfix.json` → `solve --samples samples_fkfix.json`
> （`refk` 幂等，会自检并打印「位置改动 0.0000cm」「X轴·L3 = +1.0000」）。

> 旧「眼在外 + PC 上位机」版本（Homography 单应标定）已移除；其演示与测试方法保留在
> `docs/完整功能演示流程.md`、`docs/视觉识别测试清单.md`（历史归档，命令不可直接照跑）。

> WiFi 热点 SSID/密码在 `Core/Src/wifi.c` 的 `WIFI_AP_SSID` / `WIFI_AP_PASS` 中配置。
> 仓库中已置为占位符（`YOUR_SSID` / `YOUR_PASSWORD`），运行前请改成自己的。

## 本人负责的模块

本项目固件由 CubeMX 生成外设框架，以下模块为本人手写实现：

- **自定义 UART 指令协议**：帧状态机、XOR 校验、双缓冲应答、多串口路由，以及 WiFi `+IPD` 上报 / `AT+CIPSEND` 封包的专用适配。
- **机械臂运动控制**：斜坡限速、逐轴软限位、`Arm_Update()` 20ms 刷新、五态状态机与急停逻辑。
- **FreeRTOS 三任务架构与两把互斥锁**设计（任务划分、优先级、栈大小、锁序）。
- **驱动与功能模块**：PCA9685 驱动、电位器 ADC+DMA 采样滤波、示教录制/回放。
- **视觉抓取链路**（鲲鹏端 Python）：YOLOv8n 目标检测、眼在手手眼标定（AX=XB）、六自由度逆解算脉宽、串口发帧与网页遥控。

> **第三方代码声明**：`Drivers/`（ST HAL）、`Middlewares/`（FreeRTOS、CMSIS）为 ST / FreeRTOS 官方库，非本人编写。

## S3b 鲲鹏端模块 · README 三行法（面试前复习提纲）

> 规矩：每写完一个模块补三行 —— ① 用了什么语法 ② 为什么这么设计 ③ 不这么写会出什么问题。
> 用大白话写，重点在「为什么」，面试前直接拿这一节当提纲。

**模块 A：`kunpeng/vision/detector_service.py`（8080 视觉服务，B 抽帧推理 + C 采集格式）**

- **① 用了什么语法**：`argparse` 收启动参数（`--fourcc` 用 `type=str.upper` + `choices` 把取值锁在 MJPG/YUYV/AUTO）；用一个类 `Detector` 把「开摄像头 → 循环读帧 → JPEG 编码 → 推 HTTP」装在一起；`threading.Thread` 开后台线程专门跑 YOLO 推理；`cv2.VideoCapture(src, cv2.CAP_V4L2)` 显式指定 V4L2 后端、`CAP_PROP_FOURCC` 协商像素格式、`cv2.imencode` 编码；自己写 `fourcc_to_str()` 用位移 `>>` 和 `& 0xFF` 把驱动返回的整数还原成 `MJPG` 四个字符；帧号 `_jpeg_frame_id` 做去重。
- **② 为什么这么设计**：采集是每帧都要干的事，推理在纯 CPU 上很慢，两者必须拆开 —— 推理放独立线程、每 N 帧才跑一次（抽帧），画面才不会被一次 140ms 的推理卡住。FourCC 设完还要**回读一次**打印实际协商值，是因为 `cap.set()` 只是「请求」，摄像头可以不接受；不回读就会把「我以为设了 MJPG」当成事实，排障时被假象误导。
- **③ 不这么写会出什么问题**：不抽帧 → 每帧都等 YOLO，画面 ~7FPS 卡成 PPT；不显式 `CAP_V4L2` → OpenCV 会去乱试奥比中光深度相机后端，刷一堆无关警告；不默认 MJPG → 摄像头协商到未压缩 YUYV，640×480 在 USB2.0 下物理只给 10FPS，Python 层再怎么优化都没用；不回读 → 改了参数却不知道到底生效没有。

**模块 B：`kunpeng/web/web_control.py`（5000 网页控制台，A 方案点选抓取）**

- **① 用了什么语法**：Flask 单文件，`render_template_string` 把整页 HTML/CSS/JS 内嵌在 `PAGE` 字符串里；`pyserial` 串口收发 + 独立 `_rx_loop` 收帧线程；模块级全局软引用 `_IK = None` 配 `init_ik()` 里的 `try/except` 做可选功能降级；`/api/ik/solve` 这类 JSON 接口用 `request.get_json()` / `jsonify()`；协议帧自己算 XOR 拼成 `$JOINT,...*CK\r\n`；前端原生 JS `fetch` + 事件监听点画面取点，两步「① 试算 → ② 确认下发」。
- **② 为什么这么设计**：`arm_ik.py` / `arm_params.json` 不在板上最小上传集里，硬 `import` 会让缺文件时整个 5000 起不来 —— 所以让 `_IK` 允许是 None，缺了只关掉「点选抓取」，PING/MOVE/STATE/视频主链路照跑。下发前先试算再确认，是因为六路脉宽会真驱动舵机，不给二次确认，误点一下画面臂就猛甩。
- **③ 不这么写会出什么问题**：硬 import → 板上少一个文件整站打不开，还容易误判成 Flask/端口故障；不做两步确认 → 误触直接下发；不裁软限位 → IK 解出的越界脉宽把舵机顶到堵转；改了内嵌 `PAGE` 不重启进程 → 浏览器刷到天亮还是旧页面（`?v=N` 只清浏览器缓存，不清 Python 进程内存）。

**模块 C：`kunpeng/robot/eye_check.py` + `/api/ik/eye/solve` 的径向修正（A+ 定位对齐，2026-11-14）**

- **① 用了什么语法**：`argparse` 的 `--radial-fix` 收三种写法（`auto` / `0` / 一个数字），解析成一个「要么是 `None` 表示交给服务端默认、要么是具体浮点」的变量，再决定要不要往 JSON 里塞这个键；服务端把修正抽成纯函数 `_radial_fix_xy(x, y, k)`，系数 `_EYE_RADIAL_K` 放模块级、用 `--radial-fix` 启动参数 + 函数内 `global` 覆盖；返回体额外回 `x_raw / y_raw / radial_k / radial_shift` 四个字段，让「改了多少」可查。
- **② 为什么这么设计**：修正只乘**半径**、不碰方位角 —— 因为物理原因就是「光轴俯仰角错了 δ」，射线沿自身方向前后滑，横向不受影响（`交点位移 ≈ depth·tanδ / sinα`）；用系数型而不是固定偏移，是让 r 小时修正量自动趋近 0，不会把近处的点硬推远。位置选在 **API 出口**而不是 `handeye_geometry.localize`，是因为 localize 是标定链的一环（`refk_samples` / `AX=XB` 都走它），往里塞经验系数会把 X 污染得说不清；放出口则网页点选 / `eye_check` / `pick_pipeline` 一起受益，还能用 `radial_fix=0` 当场对比。
- **③ 不这么写会出什么问题**：① 把系数塞进 localize → 以后每次重算标定都带一个来路不明的缩放，X 一旦被重解就和历史数据对不上；② 用固定偏移 → r=10cm 处也硬加 3cm，近处直接过冲；③ 只改 x/y 不给 `x_raw` → 同一块棋盘前后两次坐标差 3cm 根本没法追溯，而这正是当初卡了两天的现象；④ **`do_hover` 的净空闸门继续用常数 `SAG_DZ` → 远端「报 1.12cm 净空、实际只剩 0.2cm」照样放行**（30cm 那次就这么擦到纸的）。现在闸门按力矩臂折算并取 `max(常数, 折算)`，宁可误拒也不放行。

**模块 D：`car/car_frame.c` + `kunpeng/car/car_gateway.py`（底盘统一帧，A 项，2026-11-14）**

- **① 用了什么语法**：协议层用 `static` 私有状态 + 四态 `switch` 状态机逐字节成帧、`enum` 给动作命名、
  `struct` 打包命令、**函数指针**（`car_on_command_fn` / `car_send_fn`）把「发应答」「动电机」两个回调注册进来；
  自己写 `parse_u32()` 而不用 `atoi`；Python 侧用 `argparse` 的 `add_mutually_exclusive_group` 让
  `--port/--tcp/--dry-run` 三选一，`with` 自动关串口、`%02X` 格式化 XOR 校验和。
- **② 为什么这么设计**：协议层**不 include `stm32f4xx.h`**，硬件动作靠回调注入 ⇒ 在 PC 上用 gcc
  就能编译跑 14 项单元测试，不必每改一次协议就烧板子；帧语法直接沿用机械臂 `uart_protocol.c`
  的 `$<CMD>[,<ARG>...]*<CK>\r\n`，两个 MCU 节点共用一套句子才叫「统一帧」。网关做成**独立进程**，
  不碰已实机验收的 `web_control.py`。固件侧把量程仲裁（时长 ≤5000ms / PWM ≤999）放在本地，
  落实「LLM 只能提议、本地仲裁器才授权」。
- **③ 不这么写会出什么问题**：把 `BT_SendStr` / `Car_Forward` 写进协议层 ⇒ 无法主机测试、两套协议难复用；
  用 `delay_ms(800)` 做动作时长 ⇒ 800ms 内收不到 `$CAR,STOP`，急停喊不住、避障循迹被卡住（改用 `millis()` 时间戳轮询）；
  不区分「帧模式 / 单字符模式」⇒ 帧 `$CAR,FWD,...` 里的 `F` 会被老蓝牙协议误当成「前进」；
  不给定长缓冲上限 ⇒ 串口灌垃圾字节写爆数组，裸机没有 OS 保护直接跑飞。
- 🔴 **本次没改 `.ioc`**：小车是 Keil+SPL、机械臂是 CubeMX+HAL，按两个节点各自保留工程、
  靠统一帧通信；改成单板才需要动 CubeMX，但会重生成代码 + 重烧甩臂、破坏已验收链路。详见 `docs/第四项目整合说明.md` 第四节。

**模块 E：`kunpeng/robot/voice_gateway.py`（鲲鹏侧语音入口网关，B 项 / 阶段 2，2026-11-14）**

- **① 用了什么语法**：`sys.path.insert` 跨目录复用 `../llm/` 的解析器与 XOR；`enum.Enum` + `@dataclasses.dataclass` 把仲裁判决与结果关进固定集合（`set_estop` 三态让仲裁器保持**纯函数**）；`socketserver.ThreadingTCPServer` + `StreamRequestHandler` + `threading.Lock` 保护跨连接急停标志；`urllib.request` 调 `/api/cmd`；`argparse --apply` 显式开关；纯函数 `normalize_frame()` 还原 shell 字面量 `\r\n`；**11-14 加固**：`_emit()`（`sys.stdout.reconfigure` + `backslashreplace` 兜底）做 locale 无关输出、`_FrameHandler` **先发应答后打日志**，`_match_resume()` / `_match_estop()` 两个纯函数消除短词表子串歧义。
- **② 为什么这么设计**：做**独立进程**，不改已实机验收的 `web_control.py`、不直连串口（串口由它独占，第二个进程会抢帧），只通过 HTTP 调它已有的 `/api/cmd` ⇒ 删掉即完整回滚。**本地仲裁器白名单只授权 `HOME/STOP/RESUME`** 三个固件原生无参数命令字，从根上不需要软限位裁剪 / `to_pulses()` 反解 / `SAG_DZ` 净空折算；`PICK/PLACE/MOVE_*` 拒、`LOOK` 为 NOOP，落实「LLM 只提议、本地仲裁器才授权」。**默认 dry-run**，`--apply` 才真下发。急停文本匹配放在意图解析之前，且 `$ESTOP*CK` 走快速通道。规范定义：协议适配层（Gateway / Anti-Corruption Layer）+ 本地仲裁器。
- **③ 不这么写会怎样**：按路线图原写法下发 `$JOINT` ⇒ 一步踩进脉宽越界 / `to_pulses()` 符号 / 软限位静默夹紧 / 下垂净空折算四个雷区；把归一化塞进底层 `text_gateway.py` ⇒ 牵动它 18 项单测、且纯函数不该替调用方猜格式；多线程急停标志不上锁 ⇒ 并发漏判；只看 `/api/cmd` 的 `ok:true` 不看 `resp` ⇒ 把「写进串口」当成「固件执行成功」；Flask 没起不兜异常 ⇒ 客户端收不到任何应答；**11-14 实测**：板端 `LANG=C` 而输出层假设 UTF-8 ⇒ 中文启动横幅崩服务、`--self-test` rc=1、日志写在应答前把回帧吞掉；短词表裸子串匹配 ⇒ 喊「解除急停」反而再急停一次（均已加固，见 `kunpeng/robot/README.md` G 节与排障第 6、7 条）。
- ✅ **档位（11-14 16:40 抬，只抬「阶段 2 网关链路」这一条）**：代码 + **38 项本机 unittest 全绿**（含 4 项 ASCII stdout 端到端回归）+ **板端 `--apply` 四层硬判据全绿**（假帧 `$ASR,回初始位*61` → `$OK,HOME*27` → `/api/state` 脉宽回初始位 → 日志 `sent:true` → 肉眼见臂动；记录见 `docs/上板验收-语音网关阶段2.md`）。🔴 **语音入口整体仍 🔶**：`source` 恒 `mock`，没接云端大模型 / ASR / ESP32；`$ASR` 帧**语法层**已与 ESP32 本机逐字节对齐、**语义层实机对齐待明天**；`esp32/` 骨架 11-14 晚已在 PC 上跑通整机 `set-target` + `idf.py build`（`BUILD_RC=0` / `Project build complete` / 产物齐全，证据见清单），**仍未烧录 / 未上板 / 未联调**（编译通过 ≠ 实机）。三行法详解与可粘贴命令见 `kunpeng/robot/README.md` 第十一节 / E、F、G 节，排障见 `docs/排障记录-语音网关.md`（8 条）。

**模块 F：`esp32/main/tcp_client.c/.h` + `main.c`（Wi-Fi TCP 上行层，2026-11-14 晚）**

- **① 用了什么语法**：BSD socket 四件套 `getaddrinfo()` → `socket()` → `connect()` → `send()/recv()/close()`（头由 **lwip** 组件提供，`main/CMakeLists.txt` 的 `REQUIRES` 必须补 `lwip`）；`setsockopt(SO_RCVTIMEO/SO_SNDTIMEO)` + `struct timeval` 配超时；`enum` 分级错误码 + `tcp_client_strerror()`；`typedef struct` 做「帧表」把数据与发送逻辑分开；C99 八进制转义 `\ooo` 写死中文 UTF-8 字节串；`#if SEND_CN_HOME_FRAME` 编译期安全闸门；`vTaskDelay(pdMS_TO_TICKS())` 让出 CPU。
- **② 为什么这么设计**：骨架是「发一句、等一句」短事务 ⇒ **一次一连接**，不维护长连接状态机、最不容易漏关 fd；`recv()` 故意只收 `reply_size-1`，留一格给 `'\0'`；**超时必须设**，否则网关没起时 `recv()` 永久挂着，连 I2S 采音循环都跑不到、看起来像整机死机；网关 IP/端口集中成宏，换网络改一处重 build；中文帧写成字节值而非直接写汉字，避开源码编码差异，与板端 `printf` 八进制兜底、Python `.encode("utf-8")` 三方对齐。
- **③ 不这么写会怎样**：不补 `REQUIRES lwip` ⇒ 链接期 `undefined reference to getaddrinfo`；不设超时 ⇒ 断网像死机；不 `close()` / `freeaddrinfo()` ⇒ fd 耗尽被误判成「Wi-Fi 掉了」；自检帧若还用 `$ASR,mic_ok*50` / `$ASR,home*63` ⇒ 网关一律 `$ERR,2`（词表只有中文，11-14 实证）；中文帧忘了 `SEND_CN_HOME_FRAME=0`，在 `--apply` 下会**真的让臂回初始位**；网关忘了 `--host 0.0.0.0` ⇒ ESP32 连不上被误判成热点问题。
- 🔴 **档位**：本机帧/网关语义实证 `PASS=24 FAIL=0`、PC 整机 build 已跑（结论见 `docs/明日回教室-ESP32照跑清单.md` 第一节）；**未烧录 / 未上板 / 未联调，仍 🔶**。三行法全文见 `esp32/README.md` 第九节，排障见 `docs/排障记录-ESP32.md` 第 3、4 条。

> **精度口径（别写错）**：径向修正补掉的是**定位侧**那 2.98cm 系统偏近；`TIP_BIAS_XY` 补的是**执行侧**爪尖的 0.10/−0.95cm。
> 两者物理上同源（都是 J2/J3 被重力压下），但作用点不同、必须分开扣。
> 网页点选抓取那条路**只做了径向修正、没扣 TIP_BIAS** ⇒ 残留约 1cm；`--hover` 验收路径两边都扣，才是完整闭环。

> **为什么删掉 2D 侧视姿态图**（2026-11-07）：那张图只是把脉宽线性映射成角度画个示意，标题自己就写着「非运动学正解」，
> 没有连杆长度也没有零位偏置，面试官一问「怎么算的」就会露馅；而且 A 方案已经有真逆解，留着反而多余。
> 删掉是**诚实边界**，不是功能倒退。

## 开发中遇到的问题与解决（16 条）

按模块记录开发中实际遇到、并已解决的问题，附根因分析。

> S3b 阶段（鲲鹏视觉服务 + 网页控制台）的排障记录按「现象 → 排查过程 → 根因 → 解决 → 一句话面试说辞」
> 五段式单独维护在 `docs/排障记录-S3b.md`；本节 16 条是固件 / 旧 PC 上位机阶段的历史记录。

**RFID（RC522）**

- **初始化时好时坏，版本寄存器读回 0x00/0xFF**：原用硬件 SPI Mode 3 与模块时序不匹配，RST 引脚空闲电平实测仅 2.2V 低于复位阈值 2.31V；改为 GPIO 软件 SPI（Mode 0、MSB 先出）后时序完全由代码控制，版本校验改非阻断，并增加"芯片失联自动硬件 RST + 软复位 + 重配寄存器"兜底（后期确认旧模块射频前端损坏，换新模块后全程 Ver=0x82 稳定）。
- **刷卡无反应（REQA fail，Irq=00）**：定时器应答窗口 12.5ms 远大于接收等待循环时长，时序颠倒——循环先耗尽，定时器中断永远来不及置位；窗口缩至 3.2ms（TReload=0x00FA）并加长循环至 3000 次。
- **刷卡偶发失败（Irq=64 / Err=40）**：供电瞬态或接触不良致芯片偶发复位，Err=40 为射频错误；检测周期仅在 1~2ms 内开天线随后立即关闭以降低平均电流，保留硬件 RST + 软复位恢复（静默执行，不打印）。

**蓝牙（HC-05）**

- **手机连接后频繁掉线，指令无响应**：模块直接由 STM32 3.3V 引脚供电，建链瞬间电流不足导致射频复位；另有一批克隆模块设备名非标准、固件不兼容。改为独立稳压 5V 供电并共地，AT 模式配置波特率 115200、8N1、名称 ARM-BT、密码 1234，EN/KEY 拉低断电重启进入透传模式后才可收发。

**WiFi（ESP-01S）**

- **TCP 服务器模式下客户端收不到 UART→TCP 数据**：CIPSERVER（CIPMUX=1）与透传模式（CIPMODE=1）在克隆固件上不能共存，且非透传下 UART→TCP 不会自动发生；固定 CIPMODE=0 + AT+CIPSEND 包封转发，命令间必须等待 'OK'/'ready' 应答（AT+RST 要等 'ready' 而非首个 'OK'），不能用固定延时，否则命令丢失。
- **CIPMUX/CIPSERVER 掉电丢失**：克隆模块 flash 不保存配置；上电后由 STM32 主循环重新配置，不依赖模块自身保存。

**机械臂标定与逆运动学**

- **J3/J4 大角度前倾区线性度差，角度拟合误差达 106°**：舵机大角度区输出非线性，且 J3 角度测量方向取反；改为 5 点分段标定并校正方向符号后重测达标。
- **夹爪方向整体偏 90°，沿边抓取失败**：J5 标定时误以夹爪轴线为基准，实际应以夹爪两指连线方向为基准；按两指连线重标 J5，并用 `--grip-conv 90` 约定 J5=0 时张开方向相对臂前向的关系。
- **夹取点相对 J5 旋转轴横向偏心约 1cm，出现位置相关偏差（±3°）**：逆解按夹爪中心计算，未考虑偏心导致的方位角偏差；改为基于 J5 轴点方位建立偏心补偿模型，直立抓不到时前倾角每次 +5°（上限 45°）逐步扩臂。
- **J1/J2 端点脉宽可能堵转，长时间大角度过热**：端点只做快速测量、不长时间停留；各关节软限位写入 `main.h`（J3 520~2500µs、J4 500~1850µs、J5 500~2500µs；J4 为另行收窄的安全限位，物理实测端点仍在 `arm_params.json`），上电初始位用独立的 `SERVO_Jx_INIT` 宏（折叠收拢姿态，与各关节 0° 标定零位不是一回事）。

**视觉识别与抓取**

- **相机画面目标乱跳、抓取目标重复**：反光重影、按面积排序抖动、杂物误识别；工作区 ROI 裁剪 + min_area 面积过滤，大工作区下 min_area 从 500 调至 200~300，画面干净稳定。
- **工作区边缘目标方向角偏、抓不准**：相机挂高不足覆盖 250×200mm 工作区，目标越靠边缘、相机越斜，像素形状畸变越大；挂高覆盖全视野，光轴与桌面垂直偏差控制在 10~15° 以内，ROI 像素框用 `set_roi.py` 按工作区实测自动计算，四点围成矩形内精度最佳。
- **标定后抓取位置偏移**：标定后挪动相机或机械臂底座，Homography 与 base_x/base_y 只对同一固定安装有效；改为固定安装（眼在外）后不挪动，标定后把①原点的机械臂坐标实测填入 `config.json` 的 `base_x/base_y`，换过位置或相机就重新执行四点标定。

**串口协议与固件**

- **多串口同时收发时偶发数据错乱**：发送侧双缓冲（活动槽 + 备用槽）在 tx_busy 期间 ISR 写备用槽，主循环搬运与发送前置位存在竞态；搬运与发送前置位两处均用 `__disable_irq`/`__enable_irq` 临界区保护，避免 ISR 误写活动槽或破坏拷贝。
- **蓝牙指令返回 $ERR,1**：校验码错误；上位机自动计算 XOR 校验码（如 `$MOVE,0,1800*28`），无需手算。
- **编译报 huart2 未定义 / 链接缺少 bt、wifi 模块**：`stm32f4xx_it.c` 的 USER CODE Includes 段补 `#include "usart.h"`；bt.c、wifi.c 手动加入 Keil 工程的 Application/User/Core 分组。

## 演示要点（建议 15 分钟流程）

### ✅ 当前 Kunpeng 工程（照这个演示，全部有实机证据）

1. 架构讲解：端（STM32 + PCA9685 六路）/ 边（鲲鹏 8080 检测 + 5000 控制台）/ 统一 `$CMD*CK\r\n` 帧协议。
2. 串口链路：浏览器点 PING / STATE / HOME，看终端 `$OK,PONG*3E` 与六路脉宽回填；滑杆单轴微调看软限位与斜坡限速。
3. 视觉链路：8080 画面 LIVE / 帧率约 26.6 FPS / YOLO 检测框叠加，讲 B 抽帧推理 + C V4L2/MJPG 采集优化（6.8 → 26.6 FPS 的分层归因过程）。
4. 点选抓取链路（重点）：回初始位等稳 → 勾选「允许点击画面取点」→ 点画面中心附近 →「① 试算（不动臂）」看 `(x,y)` / 六路脉宽 /「✓ 六路均在软限位内」→「② 确认下发」看 `OK` 与 `STATE` 瞬时值逼近。
5. 诚实边界主动交代：这是**开环点选运动**，平面映射未做物理标定、无编码器反馈、无下降/闭合/抬起序列；相机装在臂末端，取点前必须回同一观察位。
6. 标定参数面板：常驻 5 框 + 预设保存 / 点行切换 / 删除（浏览器本地）。

### 🔶 旧 STM32 单板项目演示清单（当前工程未复验，勿照此演示）

1. 系统架构讲解（三层架构 + 统一指令协议 + 多串口路由）
2. PC 上位机控制：PING/STATE/HOME、单轴微调、示教 REC→PLAY 回放
3. RFID 刷卡切换：钥匙扣刷入 MANUAL 电位器手控，卡片刷回 HOME/CMD
4. 颜色识别 + 分步抓取 → 码放，展示沿边夹取与远目标前倾（旧版 Homography 单应标定流程）
5. 无线通道：蓝牙 APP / WiFi TCP 客户端发指令控制

## 仓库说明

- **已脱敏**：WiFi 热点 SSID/密码为占位符，蓝牙名称/密码等演示参数不含真实凭据。
- **已在 `.gitignore` 排除**：Keil 构建产物与备份副本（`MDK-ARM/Robotic arm vision/`、`*.uvoptx`、`*.0000` 等）、CMSIS DSP 官方示例（`Drivers/CMSIS/DSP/Examples/`）、Python 缓存、`docs/pdf_util/main.pdf` 与第三方手册解包文本。
