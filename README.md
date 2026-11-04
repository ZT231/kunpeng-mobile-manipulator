# 六自由度机械臂视觉抓取系统（STM32F407 + FreeRTOS）

基于 STM32F407VET6 的六自由度机械臂平台，打通 **视觉识别 → 坐标解算 → 逆运动学 → 机械臂抓取** 全链路，并支持串口 / 蓝牙 / WiFi / RFID 四种控制通道共用同一套指令协议。

> 固件：Keil MDK（CubeMX 生成框架 + 手写模块）｜ 上位机：Python + OpenCV

---

## 主要功能

- **视觉引导抓取**：USB 摄像头 HSV 颜色阈值 + 轮廓检测识别目标，四点 Homography 完成像素→毫米换算，平面几何逆运动学解算 6 路舵机脉宽；按目标形状自适应夹爪方向（沿边抓取），远目标自动前倾扩臂（上限 45°）。
- **多通道控制**：串口（PC 上位机）/ 蓝牙（HC-05）/ WiFi（ESP-01S）/ RFID（RC522）四通道复用同一套 `$CMD*CK\r\n` 协议（XOR 校验），可分别独立演示。
- **模式状态机**：MANUAL（电位器手控）/ CMD（待命）/ RECORD（示教录制）/ PLAYBACK（回放）/ ESTOP（急停）五态；急停冻结输出，仅 RESUME 可解除。
- **示教回放**：上位机 REC 采集轨迹，PLAY 一键回放。
- **安全机制**：每关节独立软限位（写舵机前 clamp）、斜坡限速（25µs/20ms ≈ 112°/s）、急停冻结输出。

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

## 云-边-端三层架构（本工程 = 三块板一套代码）

| 层 | 板子 | 跑什么 | 代码位置 |
|---|---|---|---|
| 端（执行） | **STM32F407VET6** | 6 路舵机 / 底盘电机 / 串口帧解析 / 运动学执行 / 急停与软限位 | `Core/` |
| 边（大脑） | **KunPeng Pro** | YOLOv8 识别、逆解算脉宽、网页/APP 服务、IoT 网关 | `kunpeng/`（+ `iot/`） |
| 端（入口） | **ESP32-S3** | 语音唤醒（WakeNet）+ ASR + Wi-Fi/BLE | `esp32/`（P2 阶段） |

主流程：**ESP32-S3 听语音 → 鲲鹏识别+决策算 6 路脉宽 → 串口发 `$JOINT` → STM32 驱动 PCA9685 → 舵机执行**。
三方协议完全一致（STM32 固件 / Python `arm_host.py` / C++ 引擎），帧格式 `$<CMD>[,ARGS]*<CK>\r\n`，XOR 校验。

当前进度：

| 环节 | 状态 |
|---|---|
| STM32 固件（协议 + 舵机 + 示教 + 多通道） | ✅ 旧项目已跑通，可直接烧录 |
| 鲲鹏 YOLOv8 识别 | ✅ 板上实测 7.0 FPS（`--imgsz 320`），常驻服务已封装 |
| 鲲鹏网页遥控 | ✅ `kunpeng/web/web_control.py`，浏览器滑杆控 6 路 |
| 手眼标定（眼在手） | 🔶 脚本已验证（FK 误差 0.000cm、合成解算闭环通过），待相机到货实标 |
| ESP32-S3 语音 | 📋 P2，核心板+扩展板待焊 |

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
│   │   ├── stage1_yolov8.py     识别主程序（双后端 + --imgsz 320 跑到 7FPS）
│   │   ├── detector_service.py  常驻 HTTP/JSON 识别服务（:8080/detections）
│   │   ├── a_vision_demo.py     颜色追踪 / KCF 追踪演示
│   │   └── deploy_stage1.sh     依赖与权重部署脚本
│   ├── robot/                   机械臂解算与控制
│   │   ├── arm_ik.py            逆/正运动学（ArmIK）
│   │   ├── arm_params.json      舵机标定表
│   │   ├── arm_host.py          串口发帧上位机（$JOINT/$MOVE/$STATE）
│   │   └── handeye_calib.py     ★手眼标定（AX=XB，眼在手必备，本工程新增）
│   ├── web/                     web_control.py 网页遥控（Flask + 滑杆 + 状态回填）
│   └── docs/                    stage1_README.md 识别部署说明、A档执行计划
├── esp32/                       ★ESP32-S3 端（语音唤醒 + ASR，P2 阶段）
├── iot/                         IoT 网关五件套（EMQX→eKuiper→InfluxDB→Grafana + 模拟设备）
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

### 3. 眼在手手眼标定（相机装到臂末端后）
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

## 开发中遇到的问题与解决（16 条）

按模块记录开发中实际遇到、并已解决的问题，附根因分析。

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
- **J1/J2 端点脉宽可能堵转，长时间大角度过热**：端点只做快速测量、不长时间停留；各关节软限位/中位全部按实测标定值写入 `main.h`（J3/J4 限位 520~2500µs、J5 500~2500µs，各关节有独立 `SERVO_Jx_MID` 中位宏）。

**视觉识别与抓取**

- **相机画面目标乱跳、抓取目标重复**：反光重影、按面积排序抖动、杂物误识别；工作区 ROI 裁剪 + min_area 面积过滤，大工作区下 min_area 从 500 调至 200~300，画面干净稳定。
- **工作区边缘目标方向角偏、抓不准**：相机挂高不足覆盖 250×200mm 工作区，目标越靠边缘、相机越斜，像素形状畸变越大；挂高覆盖全视野，光轴与桌面垂直偏差控制在 10~15° 以内，ROI 像素框用 `set_roi.py` 按工作区实测自动计算，四点围成矩形内精度最佳。
- **标定后抓取位置偏移**：标定后挪动相机或机械臂底座，Homography 与 base_x/base_y 只对同一固定安装有效；改为固定安装（眼在外）后不挪动，标定后把①原点的机械臂坐标实测填入 `config.json` 的 `base_x/base_y`，换过位置或相机就重新执行四点标定。

**串口协议与固件**

- **多串口同时收发时偶发数据错乱**：发送侧双缓冲（活动槽 + 备用槽）在 tx_busy 期间 ISR 写备用槽，主循环搬运与发送前置位存在竞态；搬运与发送前置位两处均用 `__disable_irq`/`__enable_irq` 临界区保护，避免 ISR 误写活动槽或破坏拷贝。
- **蓝牙指令返回 $ERR,2**：校验码错误；上位机自动计算 XOR 校验码（如 `$MOVE,0,1800*25`），无需手算。
- **编译报 huart2 未定义 / 链接缺少 bt、wifi 模块**：`stm32f4xx_it.c` 的 USER CODE Includes 段补 `#include "usart.h"`；bt.c、wifi.c 手动加入 Keil 工程的 Application/User/Core 分组。

## 演示要点（建议 15 分钟流程）

1. 系统架构讲解（三层架构 + 统一指令协议 + 多串口路由）
2. PC 上位机控制：PING/STATE/HOME、单轴微调、示教 REC→PLAY 回放
3. RFID 刷卡切换：钥匙扣刷入 MANUAL 电位器手控，卡片刷回 HOME/CMD
4. 视觉自动抓取：识别 → 分步抓取 → 码放，展示沿边夹取与远目标前倾
5. 无线通道：蓝牙 APP / WiFi TCP 客户端发指令控制

## 仓库说明

- **已脱敏**：WiFi 热点 SSID/密码为占位符，蓝牙名称/密码等演示参数不含真实凭据。
- **已在 `.gitignore` 排除**：Keil 构建产物与备份副本（`MDK-ARM/Robotic arm vision/`、`*.uvoptx`、`*.0000` 等）、CMSIS DSP 官方示例（`Drivers/CMSIS/DSP/Examples/`）、Python 缓存、`docs/pdf_util/main.pdf` 与第三方手册解包文本。
