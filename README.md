# 六自由度机械臂视觉抓取系统（STM32F407 + FreeRTOS）

基于 STM32F407VET6 的六自由度机械臂平台，配合 KunPeng Pro（openEuler）边缘端与网页控制台，
打通 **视觉识别 → 平面坐标解算 → 逆运动学 → 串口下发 → 状态回读** 链路。

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

- **中断只收字节**：ISR 内仅做单字节接收与帧状态机，命令解析与执行全部放在任务上下文（中断优先级 5，匹配 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`）。
- **两把互斥锁**：`arm_lock`（机械臂状态）、`print_lock`（串口输出），从不同时持有，避免死锁。
- **协议帧格式**：`$<CMD>[,<ARG>...]*<CK>\r\n`，`CK` = CMD 字段（`$` 与 `*` 之间）所有字节的 XOR，2 位大写十六进制；响应 `$OK[,<数据>]` / `$ERR,<code>`。
- **双缓冲应答**：活动槽 + 备用槽，临界区保护，保证连发命令的应答不丢失。

## 云-边-端三层架构（规划终态 = 三块板一套代码；当前只落地了「端 + 边」）

| 层 | 板子 | 跑什么 | 代码位置 | 当前状态 |
|---|---|---|---|---|
| 端（执行） | **STM32F407VET6** | 6 路舵机 / 串口帧解析 / 运动学执行 / 急停与软限位 | `Core/` | ✅ 已实机 |
| 边（大脑） | **KunPeng Pro** | YOLOv8 识别、逆解算脉宽、网页控制台 | `kunpeng/` | ✅ 已实机 |
| 边（IoT，规划） | KunPeng Pro | EMQX / eKuiper / InfluxDB / Grafana 网关 | `iot/` | 🔶 Docker 脚手架 + 模拟设备，未起容器、未接真实设备 |
| 端（入口，规划） | **ESP32-S3** | 语音唤醒（WakeNet）+ ASR + Wi-Fi/BLE | `esp32/` | 📋 仅 README 占位，核心板+扩展板待焊 |

**规划终态主流程**（📋 尚未打通）：ESP32-S3 听语音 → 鲲鹏识别+决策算 6 路脉宽 → 串口发 `$JOINT` → STM32 驱动 PCA9685 → 舵机执行。
**当前已打通的主流程**（✅）：浏览器网页点选取点 → 鲲鹏 5000 平面映射 + `ArmIK` 逆解 → 串口 `$JOINT` → STM32 驱动 PCA9685 → `STATE` 回读。
三方协议一致（STM32 固件 / Python `arm_host.py` / C++ 引擎），帧格式 `$<CMD>[,ARGS]*<CK>\r\n`，XOR 校验。

当前进度：

| 环节 | 状态 |
|---|---|
| STM32 固件（协议 + 六路舵机 + 软限位 + 斜坡） | ✅ 实机通过：`$PING*10` → `$OK,PONG*3E`，`STATE` 回读六路脉宽 |
| 鲲鹏 YOLOv8 识别 + MJPEG 推流 | ✅ 板上实测约 26.6 FPS（B 抽帧推理 + C V4L2/MJPG 采集优化后） |
| 鲲鹏网页控制台 + 点选抓取 | ✅ 5000 单页：滑杆控 6 路、点选→IK→`JOINT`→`STATE`、标定参数常驻面板 + 本地预设 |
| `iot/` IoT 网关五件套 | 🔶 Docker Compose + 模拟设备脚本已备，**未在板上起容器、未接真实设备** |
| 手眼标定（眼在手） | 🔶 `handeye_calib.py` 自检通过（FK 误差 0.000cm），**未相机实标、未接入点选链路** |
| 蓝牙 / WiFi / RFID 多通道、示教回放 | 🔶 旧项目源码已有，当前工程未重新实机复验（RC522 当前未接） |
| ESP32-S3 语音 / ASR / WakeNet | 📋 P2，`esp32/` 仅 README，核心板+扩展板待焊 |
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
│   │   ├── detector_service.py  常驻 HTTP/JSON 识别服务（:8080/detections）
│   │   ├── a_vision_demo.py     颜色追踪 / KCF 追踪演示
│   │   └── deploy_stage1.sh     依赖与权重部署脚本
│   ├── robot/                   机械臂解算与控制
│   │   ├── arm_ik.py            逆/正运动学（ArmIK）
│   │   ├── arm_params.json      舵机标定表
│   │   ├── arm_host.py          串口发帧上位机（$JOINT/$MOVE/$STATE）
│   │   └── handeye_calib.py     🔶 手眼标定（AX=XB）：自检通过，未实标、未接入点选链路
│   ├── web/                     ✅ web_control.py 网页遥控（Flask + 滑杆 + 点选抓取）
│   └── docs/                    stage1_README.md 识别部署说明、A档执行计划
├── esp32/                       📋 ESP32-S3 端（语音唤醒 + ASR，P2）：仅 README 占位，板待焊
├── iot/                         🔶 IoT 五件套（EMQX→eKuiper→InfluxDB→Grafana + 模拟设备）：未起容器
├── protocol_engine_cpp/         C++ 跨平台串口协议引擎（243 断言，三方协议金标准）
└── docs/                        全部文档
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

# 常驻识别服务：HTTP :8080/detections
python3 kunpeng/vision/detector_service.py --source 0 --imgsz 320 --port 8080
```

### 3. 眼在手手眼标定（🔶 脚本自检通过，未相机实标、未接入点选链路；相机装到臂末端后再做）
```bash
python3 kunpeng/robot/handeye_calib.py selftest               # 自检正运动学，无需相机
python3 kunpeng/robot/handeye_calib.py intrinsics --camera 0  # 标定相机内参
python3 kunpeng/robot/handeye_calib.py collect --camera 0 --arm-serial /dev/ttyUSB0 \
        --intrinsics camera_intrinsics.json --out samples.json
python3 kunpeng/robot/handeye_calib.py solve --samples samples.json --out handeye_X.json
```

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
- **蓝牙指令返回 $ERR,2**：校验码错误；上位机自动计算 XOR 校验码（如 `$MOVE,0,1800*28`），无需手算。
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
