# 移动抓取机器人 · STM32F407 底盘固件（FreeRTOS）

> ⚠️ **代码现状（2026-11-26 更新）**：STM32 固件已完成大清理，彻底移除「固定工位机械臂」的全部旧代码——
> PCA9685(PWM 舵机) / RC522(RFID) / 电位器(ADC+DMA) / 示教回放 / ACS712(电流) / 安全监控 / I2C1 / ADC1 / DMA。
> 当前固件只保留：**底盘**（2×TB6612 + 4×JGB37-520 麦轮 + 编码器，`chs_tb6612.c`/`chassis.c`）、
> **两路 UART 指令协议**（USART1 PC / USART2 蓝牙，共同使用 `uart_protocol.c`），
> 以及**已接入工程的机械臂层**：`servo_bus.c/h`（总线舵机帧与个体）+ `arm_robot.c/h`（六轴零位 / 作业邮箱 / `$ARM` 命令），
> 绑定 **USART3（PB10/PB11，115200 8N1）**——原「预留第三控制源」取消，`wifi.c/h` 已删除，USART3 只做阻塞收发、不进协议栈。
> `.ioc` 已同步移除 I2C1/ADC1/DMA。
> 下文提及 RFID/示教/电流/手眼标定的内容均为**历史记录**（代码已删），仅作面试参考。

基于 STM32F407VET6 的移动底盘 + 三路串口协议固件，配合 KunPeng Pro（openEuler）边缘端与 ESP32-S3 语音入口，
构建「云-边-端移动操作平台」的底层执行单元。

> 🔴 **项目定位（2026-11-14 定稿）**：本目录即简历上的**第四个完整项目** ——
> 在**前三个项目**（`1.Smart car` 智能小车 / `2-1.Robotic arm vision` 机械臂视觉 /
> `3.C++ 跨平台串口协议引擎`）的基础上，加入**鲲鹏开发板 + ESP32-S3 + IoT 可观测性闭环**，
> 整合为「云-边-端移动操作平台」。2026-11-23 底盘升级为 **2×TB6612FNG + 4×JGB37-520
> (1:90) + 80mm 麦轮**，代码并入 `Core/` 的 `chs_tb6612.c` / `chassis.c`；旧 `car/`
> 拷贝工程已移除（原件在 `1.Smart car` 目录，可随时回退对照）。
> 完整映射表、CubeMX 结论与简历说辞见 **`docs/00-项目总览与硬数据速查.md`**。
> 🔴 **2026-11-24 已改 `.ioc`**：删除 I2C1 / ADC1 / DMA 三个外设，现只保留 7 个 IP
> （FREERTOS / NVIC / RCC / SYS / USART1 / USART2 / USART3）；引脚 13 条
> （含 TIM6 时基、SWD、KEY0，`Mcu.IPNb=7` / `Mcu.PinsNb=13`）。
> 底盘 `chs_tb6612.c` 自己开 TIM1/2/3/4/8 与 GPIO，不动 CubeMX 已配的 USART/SWD。
> CubeMX 重新生成后，若 `chs_tb6612.c` / `chassis.c` / `uart_protocol.c` / `bt.c` / `servo_bus.c` / `arm_robot.c`
> 被移出 Keil 工程，需手动加回。

> 固件：Keil MDK（CubeMX 生成框架 + 手写模块）｜ 边缘端 / 上位机：Python + OpenCV + Flask
>
> 🔴 **状态口径（2026-11-24 收口）**：本 README 混有「旧 STM32 单板项目能力」与「当前 Kunpeng 云-边-端工程已实证能力」两代内容。
> 三档划分：
> ✅ 已实机实证 / 🔶 代码已有但未实机验证或未接入主链路 / 📋 纯规划占位。
> 下文凡标 🔶 / 📋 的功能**不得写成已完成**。原 `docs/面试与简历材料.md` 已删除（2026-11-24）。

---

## 主要功能

### ✅ 当前 STM32 固件（底盘）：Rebuild 0 Error / 0 Warning（Code=24440）；未实机上电

- **底盘驱动**：2×TB6612FNG + 4×JGB37-520(1:90) + 80mm 麦轮；TIM4 20kHz PWM（PD12~PD15）+
  8 根 DIR（PC0~3 + PB12~15）+ PE8 并联 STBY；四路定时器编码器（TIM1/TIM2/TIM3/TIM8）4 倍频测速；
  PB0 常高关闭板载 W25Q16 CS（编码器 A/B 复用 PB3/PB4/PB5 的前提；编码器由 TB6612 板 3.3V 供电，电平与 FLASH 兼容，见硬件文档）；
  麦轮逆解 + PID + 里程计，20ms 固定节拍（`CHS_Tick`）。
- **串口协议**：USART1（PA9/PA10，115200 8N1）自定义帧 `$<CMD>[,<ARG>...]*<CK>\r\n`（XOR 校验、2 位大写 HEX），
  命令集 `PING / LOG / CAR:VEL / STOP / ODOM / RESET / RAW / WHEEL / ENC / GAIN / CLRFAULT`，
  以及机械臂 `ARM:POS / J / HOME / STOP / RESUME / READ / TEMP / VIN / CLRFAULT`。
- **机械臂（六轴总线舵机）**：`servo_bus.c`（幻尔/Lobot 帧、位置/温度/电压回读、软限位、J5 背隙补偿）+
  `arm_robot.c`（六轴零位 460/514/516/502/494/674、作业邮箱、ArmTask 10ms 轮询）；串口 **USART3 PB10/PB11**
  经 Waveshare Bus Servo Adapter (A) 接单线总线，115200 8N1。
- **多通道**：USART1（PC 上位机）/ USART2（HC-05 蓝牙）两路对等，复用同一套帧协议；USART3 已改为机械臂专用、不再作控制源。ESP32-S3 语音节点走 Wi-Fi TCP 连鲲鹏，不占用 STM32 串口。

### 🔶 上位机 / 边缘端（鲲鹏）历史能力（不在 STM32 固件内）

- **边缘视觉服务**：`detector_service.py` :8080 跑 YOLOv8n + MJPEG 推流 / `/detections` / `/health`；
  B 方案抽帧推理 + C 方案 V4L2/MJPG 采集优化后，实测约 **26.6 FPS**（原 6.8 FPS）。
- **网页控制台 + 点选抓取**：`web_control.py` :5000 内嵌单页，滑杆控六路、1s 状态回填、
  点画面取点 → 眼在手定位 → `ArmIK` 解六路位置 → `$ARM,POS` 下发 → `$ARM,READ` 回读；
  标定参数常驻面板 + 浏览器本地预设（保存 / 切换 / 删除）。
  🔶 协议层已于 2026-11-26 整体迁到总线舵机 `$ARM`（纯软件层已验证：py_compile + 离线
  自测全绿）；**实机（装车后）复验未做**，手眼 X 需用 Orbbec Gemini E 重标。
- **板内 IoT 可观测性闭环（1.2b）**：`iot/` 真实串口遥测 → MQTT → EMQX → eKuiper → InfluxDB → Grafana，
  四容器板上跑通、Grafana 六路 `j1~j6` 曲线已出（16:07 `records_in=records_out=59` / `exceptions=0`）；
  `deploy.sh` 动态重绑 firewalld trusted zone、`telemetry_bridge.py` 发布端拦 null 点；收口 `e6cd194`。
  🔴 只叫「可观测性闭环」，不是云端 IoT、不增抓取能力。
- 上述上位机脚本已于 2026-11-26 迁移到 `$ARM` 协议（`$JOINT/$STATE/$MOVE/$HOME` 全部替换、
  `$CUR/ACS712` 换成 `$ARM,VIN` 电压回读）；旧 PWM 时代的示教配方 / 下垂补偿 / C 姿态固定值
  已一并删除。**新协议只在离线自测里验证过**，装车后仍需实机复验。

### 🔶 旧 STM32 单板项目已有，当前 Kunpeng 工程未重新实机复验

> 旧版演示脚本与视觉测试程序已移除（`vision_old/`）；对应验收文档已于 2026-11-24 删除。

- **视觉引导抓取（旧版）**：USB 摄像头 HSV 颜色阈值 + 轮廓检测识别目标，四点 Homography 完成像素→毫米换算，平面几何逆运动学解算 6 路舵机脉宽；按目标形状自适应夹爪方向（沿边抓取），远目标自动前倾扩臂（上限 45°）。
- **多通道控制（历史）**：串口（PC 上位机）/ 蓝牙（HC-05）/ WiFi（ESP-01S）/ RFID（RC522）四通道复用同一套 `$CMD*CK\r\n` 协议（XOR 校验）。当前保留 `bt.c`（USART2 蓝牙）；`wifi.c`（USART3 第三控制源）已于 2026-11-26 删除，该口改作总线舵机专用；RC522 驱动已随机械臂代码删除。
- **模式状态机**：MANUAL（电位器手控）/ CMD（待命）/ RECORD（示教录制）/ PLAYBACK（回放）/ ESTOP（急停）五态；急停冻结输出，仅 RESUME 可解除。
- **示教回放**：上位机 REC 采集轨迹，PLAY 一键回放。

## 硬件平台

| 部件 | 型号 / 说明 |
|---|---|
| 主控 | STM32F407VET6 |
| 底盘电机驱动 | 2× TB6612FNG（TIM4 20kHz PWM，8 根 DIR，PE8 并联 STBY） |
| 底盘电机 | JGB37-520 直流减速编码器电机 1:90 ×4 + 80mm 黑色麦轮 + 6mm 金属联轴器（11 线 AB 霍尔，4 倍频 3960 计数/转，15.75 计数/mm） |
| 编码器接口 | TIM1/TIM2/TIM3/TIM8 四路正交，4 倍频；TIM2 为 32 位 |
| 无线 | HC-05（蓝牙，USART2）；ESP32-S3（Wi-Fi TCP 独立节点，不接 STM32 串口） |

> 完整引脚分配（占用 33 / 结构禁用 1 / 净空闲 48）见 `docs/硬件清单与接线.md` 的「3.1 引脚分配总表」；
> 代码侧同一份表在 `Core/Inc/main.h`，底盘部分展开在 `Core/Inc/chs_tb6612.h`。
> 当前按成品 F407VET6 核心板（DevEBox STM32F4VE：板载 W25Q16 + SD + NRF/TFT 排座）的机器人专用工况：PB0 常高关闭板载 FLASH 片选，不插 NRF/TFT；SD 卡座（4 位 SDIO）与固件引脚不冲突。
> ✅ 编码器电平：TB6612 板 MOTOR 座给编码器供 3.3V，A/B 输出为 3.3V 逻辑，与 W25Q16 的 3.3V 电平兼容，无需拆片/钳位；前提是 FLASH CS(PB0) 保持高、不插 NRF、不开 SPI1。

## 软件架构

固件基于 **FreeRTOS（CMSIS-RTOS v2）** 划分三个任务，按实时性分级：

| 任务 | 周期 | 优先级 | 栈 | 职责 |
|---|---|---|---|---|
| `StartProtocolTask` | 5ms 轮询 | Normal | 2048 | 解析 UART 帧、分发命令（PING/LOG/CAR 系列） |
| `StartPrintTask` | 3s | Low | 2048 | 周期打印底盘里程计/在线状态 |
| `StartChassisTask` | 20ms | Normal | 2048 | 四路编码器读取 + 麦轮逆解 + PID + 里程计 + 速度下发 |

- **中断只收字节**：ISR 内仅做单字节接收与帧状态机，命令解析与执行全部放在任务上下文。
- **一把互斥锁**：`print_lock`（串口输出互斥，PrintTask 与 ProtocolTask 共用）。底盘 PID/里程计只在 ChassisTask 单任务内更新，无跨任务竞态。
- **协议帧格式**：`$<CMD>[,<ARG>...]*<CK>\r\n`，`CK` = CMD 字段（`$` 与 `*` 之间）所有字节的 XOR，2 位大写十六进制；响应 `$OK[,<数据>]` / `$ERR,<code>`。
- **双缓冲应答**：活动槽 + 备用槽，临界区保护，保证连发命令的应答不丢失。

### 阶段 1 安全监控（历史，2026-11-24 已随机械臂代码删除）

> 🔴 `arm_control.c` / `safety.c` / `acs712.c` 已整批删除，当前固件**没有** ACS712 过流急停、`ARM_STATE_FAULT`、`$FAULT/$CLEARFAULT` 或 `StartSafetyTask`。
> 现在只剩底盘自己的堵转保护：`CHS_FAULT_STALL`（给油 300ms 编码器不走 → 拉低 STBY 并锁存），见 `chassis.c` 的 `chs_latch_stall()`。
> 下面这段描述只作为面试时的历史设计记录，**不要照着它去找当前代码**：
>
> - 旧方案用 `arm_control.h` 的五态机 + `safety.c` 独立任务读 ACS712，连续 3 次超 800mA 锁 `FAULT`；`STATE` 末尾追加 fault 字段以兼容老上位机 `parts[3:9]` 切片。
> - 结论仍然成立：急停（可恢复）与故障（需人工诊断）必须分开；保护逻辑不能散落在各命令里，否则漏一处就是「急停不住」。

## 三大核心支柱（2026-11-13 历史定位；机械臂链已删除）

> 🔴 **下面是旧「云-边-端移动操作平台」的完整构想，不是当前固件能力。** STM32 端的 PCA9685 / `$JOINT` / `$MOVE` / `STATE` 代码已于 2026-11-24 删除；
> `kunpeng/robot`、`kunpeng/web` 里的 Python 脚本已于 2026-11-26 把协议层改写成 `$ARM`
> （纯软件层验证；实机复验待装车）。看当前能力请以本节顶部的「代码现状」和 `docs/00-项目总览…` 为准。

> 🔴 **「核心必备」的唯一判据 = 能演示 + 能讲清边界。** 三根支柱缺一不可：
> 任何一根只能按实际情况标 ✅ / 🔶 / 📋，**不允许因为「它是核心」就写成已完成**。
> 顺序即优先级：支柱 ① 已站起来，② 是当前最大缺口，③ 已实证但覆盖面待扩。

| # | 支柱 | 载体 | 职责 | 代码位置 | 当前状态 |
|---|---|---|---|---|---|
| **①** | **视觉引导执行**（端 + 边） | STM32F407VET6 + KunPeng Pro | 网页点选 / 眼在手定位 → `ArmIK` 解六路位置 → `$ARM,POS` → `servo_bus` 总线舵机 → `$ARM,READ` 回读 | `Core/`、`kunpeng/robot`、`kunpeng/vision`、`kunpeng/web` | 🔶 旧 PWM 链 ✅（历史实证）；2026-11-26 换总线舵机 + `$ARM` 协议后**仅离线自测全绿**，装车实机复验待做；旧 1.091 径向系数已作废 |
| **②** | **语音入口** | **ESP32-S3**（成品板 N16R8）+ 鲲鹏侧收帧服务 | WakeNet 唤醒 + 命令词识别 → 封帧 `$ASR,...*CK` → Wi-Fi 上行鲲鹏 → **本地仲裁 → 固件原生命令字** | `esp32/`、`kunpeng/robot/voice_gateway.py` | 🔶 **P0 核心，整体进行中**：IDF 骨架 **14 文件**（11-14 晚补 `tcp_client.c/.h` Wi-Fi TCP 上行 + 自检帧改纯 ASCII 快速通道，11-21 再补 `audio_tx.c/.h` PCM 上行），本机帧/网关实证 `PASS=24 FAIL=0`、PC 上整机 `set-target`（rc=0）+ `idf.py build` **已通过**（`BUILD_RC=0` / `Project build complete` / 产物齐全，**证据见 `docs/固件与ESP32开发.md`**）；✅ **已烧录实机验证**（SoftAP/TCP 9000 往返 $OK/采音非零；9001 音频上行待验收）；✅ **B 项阶段 2 鲲鹏侧网关链路 11-14 16:40 已上板实机验证**（38 项单测 + 四层硬判据全绿，见 `docs/KunPeng部署与语音网关上板验收.md`）；唤醒/命令词/ESP32 遥测仍无代码 |
| **③** | **可观测性闭环** | KunPeng Pro（板内四容器） | 节点遥测 → MQTT → EMQX → eKuiper → InfluxDB → Grafana | `iot/` | ✅ 板内已实证：16:07 `records_in=records_out=59` / `exceptions=0`、Grafana 六路 `j1~j6` 曲线；🔶 **尚未覆盖 ESP32 节点、无告警规则** |

**目标主流程（三支柱联动，🔶 差最后一段）**：
ESP32-S3 听语音 → 鲲鹏意图解析 + `ArmIK` 算 6 路脉宽 → 串口 `$JOINT` → STM32 驱动 PCA9685 → `STATE` 回读；
全过程六路脉宽与三节点健康度进 EMQX → Grafana。

- **✅ 已打通（旧 PWM 链，历史）**：浏览器点选 → 眼在手定位 → `ArmIK` → `$JOINT` → PCA9685 → `STATE` 回读。
  2026-11-26 起这条链的协议层已换成 `$ARM`，旧链仅作历史对照。
- **🔶 待打通**：ESP32-S3 语音帧 → 鲲鹏收帧服务。✅ **鲲鹏侧 `voice_gateway.py` 已落盘 + 38 项本机单测全绿 + 11-14 16:40 上板 `--apply` 实机验收四层硬判据全绿**（TCP 收帧 → 帧校验 → 急停快速通道 → 本地仲裁 → `/api/cmd` 下发，默认 dry-run、只授权三个无参数命令字；假帧驱动臂回初始位、`PICK` 被拒、急停可冻结）；🔴 仍缺「ESP32 真的把帧发出来」这一段（11-14 晚已补 `tcp_client.c` 上行代码并本机验证，**但没上板**），`$ASR` 帧**语法层**已本机逐字节对齐、**语义层实机对齐待明天**，语音入口整体档位仍是 🔶。`text_gateway.py` 仍只做离线契约验证、不下发。
- **✅ 已打通**：STM32 经鲲鹏的串口遥测 → 四容器 → Grafana 六路曲线。
- **🔶 待打通**：ESP32 自身遥测（RSSI / 堆 / 唤醒计数）进 EMQX，让支柱 ③ 真正覆盖三节点。

协议仍是一套：STM32 固件 / Python `arm_host.py` / C++ 引擎 / ESP32-S3 四方一致，
帧格式 `$<CMD>[,ARGS]*<CK>\r\n`，CK = payload 逐字节 XOR、2 位大写 HEX。

当前进度：

| 环节 | 状态 |
|---|---|
| STM32 固件（底盘 + 协议） | ✅ Rebuild 0 Error / 0 Warning（Code=24440 / RO=1016 / RW=212 / ZI=20860）；`PING`、`$CAR/$ODOM` 等**尚未实机验证** |
| 鲲鹏 YOLOv8 识别 + MJPEG 推流 | ✅ 板上实测约 26.6 FPS（B 抽帧推理 + C V4L2/MJPG 采集优化后） |
| 鲲鹏网页控制台 + 点选抓取 | 🔶 协议层已迁 `$ARM`（离线自测全绿）；**装车后实机复验未做**，手眼 X 待用深度相机重标 |
| **支柱③** `iot/` 可观测性闭环（历史） | 🔶 旧六舵机遥测链路曾跑通（四容器 + Grafana `j1~j6` 曲线）；**STM32 固件已删舵机/机械臂代码，当前没有六路关节遥测**，需改成底盘 `$CAR,ODOM/WHEEL` 遥测后才能复用；ESP32 节点、eKuiper 告警仍未接 |
| 手眼标定（眼在手，AX=XB） | 🔶 **历史能力**（12 组姿态实标，`resid` 16.7mm→**1.60mm**、`axis_cond` 0.519、`|t|`=16.7cm≈L3）；机械臂代码已于 2026-11-24 删除，`kunpeng/robot/handeye_calib.py` 仍保留 |
| 蓝牙 / WiFi / RFID 多通道、示教回放 | 🔶 旧项目源码已有，当前工程未重新实机复验（RC522 当前未接） |
| `kunpeng/llm/` 意图解析 + 离线文本入口网关 + **VLM 决策层** | 🔶 `intent_parser.py` + `text_gateway.py`，44 项 unittest 全绿；🆕 `vlm_decider.py`（阶段 6 / P2）8 组自测过 —— **模型只选编号、不输出坐标**，越界编号拒绝执行。**均只输出决策、不下发**、未接真模型 / 未接主链路；`$ASR` 帧是建议草案 |
| **支柱②** ESP32-S3 语音入口 | 🔶 **P0 核心在办**：IDF 骨架 **14 文件**（11-14 晚补 `tcp_client.c/.h` Wi-Fi TCP 上行层、`main.c` 自检帧改 `$ESTOP/$RESUME`、中文帧 `SEND_CN_HOME_FRAME=0` 闸门；11-21 再补 `audio_tx.c/.h` PCM 上行），本机帧/网关语义实证 **`PASS=24 FAIL=0`**、PC 上整机 `set-target`（rc=0）+ `idf.py build` **已通过**（`BUILD_RC=0` / `Project build complete` / 产物齐全，证据见 `docs/固件与ESP32开发.md`）；✅ **已烧录实机验证**（SoftAP/TCP 9000 往返 $OK/采音非零；9001 音频上行待验收）。待补：WakeNet 唤醒、命令词识别、ESP32 遥测进 EMQX。✅ 硬件就绪（成品板 CH343/N16R8 + INMP441 + MAX98357A + 3W 喇叭）。👉 **11-14 用户重排优先级后本项排第一**，C 侧 `protocol.c` 封帧已与鲲鹏侧逐字节 5/5 一致，路线见 `docs/固件与ESP32开发.md` |
| **支柱② 鲲鹏侧收帧服务（B 项 / 阶段 2）** | ✅ **11-14 16:40 上板 `--apply` 实机验收通过**：`kunpeng/robot/voice_gateway.py` 已落盘 + **38 项 unittest 全绿** + 四层硬判据全绿（假帧驱动臂回初始位、`PICK` 被拒、急停冻结/恢复）；独立进程不改 `web_control.py`、不直连串口，只经 `/api/cmd` 下发固件原生 `HOME/STOP/RESUME`；✅ **ESP32 已接入并实机发 `$ESTOP/$RESUME` 自检帧**（TCP 9000 往返 $OK）；🔴 `$ASR` 语义层仍待 ESP-SR 命令词落地后对齐，`source` 恒 `mock`。记录见 `docs/KunPeng部署与语音网关上板验收.md`，排障见 `docs/排障记录-服务端与IoT.md` |
| 双 STM32 分工、双相机 6D 位姿、LLM 规划、OTA、云端 IoT、数字孪生 | 📋 纯规划，**无代码** |

> 🔴 **读表提醒**：上表中凡涉及 PCA9685 / `$JOINT` / `$MOVE` / `$STATE` / `$HOME` / `$RESUME` / `$ESTOP` 的“✅/🔶”都是 **2026-11-24 之前的历史实证**。
> 当前 STM32 固件认：`$PING/$LOG/$CAR,*`(VEL/STOP/ODOM/RESET/RAW/WHEEL/ENC/GAIN/CLRFAULT)/`$STOP`，
> 以及机械臂 `$ARM,*`（POS/J/HOME/STOP/RESUME/READ/TEMP/VIN/CLRFAULT，见 `servo_bus.c` + `arm_robot.c`）；
> **没有**顶层 `$ESTOP/$RESUME/$HOME/$JOINT/$MOVE/$STATE`。
> `kunpeng/web`、`kunpeng/robot`、`voice_gateway.py` 已改用 `$ARM`（机械臂）/`$CAR,STOP`（底盘急停）；
> ESP32 自检帧仍是顶层 `$ESTOP/$RESUME`，进 STM32 前必须改写成 `$ARM,STOP/$ARM,RESUME`。

> 写简历 / 面试时的红线见 `docs/排障记录-固件与ESP32.md`「本阶段诚实边界」（原 `docs/面试与简历材料.md` 已删除，2026-11-24）。

## 目录结构

本工程已从「单一 STM32 桌面机械臂」升级为「**云-边-端**移动操作机器人」的单一工程根目录，
STM32 / 鲲鹏 / ESP32-S3 三块板的代码与全部文档都收在这一个文件夹里。

```
RoboticArm_Kunpeng/
├── RoboticArm_Kunpeng.ioc     CubeMX 工程文件（打开它即可重新生成代码）
├── Core/                        ★STM32 固件源码（CubeMX 标准结构，勿改目录名）
│   ├── Inc/                     头文件：main / usart / bt / uart_protocol / freertos /
│   │                            chs_tb6612 / chassis / servo_bus / arm_robot
│   └── Src/                     源文件：与 Inc 同名成对
├── Drivers/                     ST HAL 库（ST 官方）
├── Middlewares/                 FreeRTOS + CMSIS（官方）
├── MDK-ARM/                     Keil 工程（RoboticArm_Kunpeng.uvprojx）
├── kunpeng/                     ★鲲鹏 KunPeng Pro 边缘端（openEuler + Python）
│   ├── vision/                          YOLOv8 识别链路
│   │   ├── stage1_yolov8.py     旧版识别主程序（双后端 + --imgsz 320 约 7FPS）
│   │   ├── detector_service.py  常驻 HTTP/JSON 识别服务（:8080/detections、/observe 原子快照）
│   │   ├── a_vision_demo.py     颜色追踪 / KCF 追踪演示
│   │   └── deploy_stage1.sh     依赖与权重部署脚本
│   ├── robot/                   机械臂解算与控制
│   │   ├── arm_ik.py            逆/正运动学（ArmIK）
│   │   ├── arm_params.json      舵机标定表
│   │   ├── arm_host.py          旧 PWM 时代串口发帧上位机（$JOINT/$MOVE/$STATE）——已废弃，仅留作对照
│   │   ├── handeye_calib.py     ✅ 手眼标定（AX=XB）：已实标 resid 1.60mm；A+ 点选链路待实测
│   │   ├── eye_check.py         🆕 眼在手点选自检：固定棋盘 + 换姿态，看算出的坐标重不重合
│   │   ├── action_primitives.py 🆕 动作原语：pick/place/home，每步回读关节角确认到位（🔶 离线全绿，未真机跑）
│   │   ├── test_action_primitives.py 🆕 动作原语回归测试：假 Flask 模拟斜坡/拒收/卡死/未确认
│   │   ├── voice_gateway.py 🆕 鲲鹏侧语音入口网关（阶段 2 / B 项）：TCP :9000 收帧 → 本地仲裁 → /api/cmd；默认 dry-run（✅ 38 项单测 + 11-14 16:40 上板 --apply 实机验收，阶段 2 链路已通；语音入口整体仍 🔶（ESP32 已接入发自检帧，ESP-SR/ASR 未落地））
│   │   └── test_voice_gateway.py 🆕 网关回归考卷：38 项 unittest，不动臂不联网
│   ├── web/                     ✅ web_control.py 网页遥控（Flask + 滑杆 + 点选抓取）
│   ├── llm/                     🔶 意图解析 + VLM 决策层（vlm_decider.py 只选编号不输出坐标，8 组自测过）
│   ├── docs/                    stage1_README.md 识别部署说明、A档执行计划
│   └── gamepad/                 🆕 手柄遥控（B 位 USB 直连，不动固件）：gp_teleop.py 驾驶/机械臂双模式 + 键位表 README（✅ 本机自检/模拟通过，🔶 未实机）
├── esp32/                       ★支柱② ESP32-S3 语音入口（P0 核心）：14 文件骨架（含 11-14 晚新增 tcp_client.c/.h Wi-Fi TCP 上行）；成品板已可用、软件已烧录、链路已实机；口径见 esp32/README.md
├── iot/                         ✅ 板内可观测性闭环（EMQX→eKuiper→InfluxDB→Grafana + 真实串口遥测）：已实证，收口 e6cd194
├── protocol_engine_cpp/         项目 3：C++ 跨平台串口协议引擎（243 断言，三方协议金标准）
└── docs/                        全部文档（8 份，2026-11-24 清理旧机械臂专题）
    ├── 00-项目总览与硬数据速查.md  项目定位 / 四项目整合 / 全部硬数据（口径唯一来源）
    ├── 硬件清单与接线.md          买什么 / 多少钱 / 怎么接，含 BOM 核对与供电
    ├── 底盘-TB6612与JGB37-520-接线、固件与调试.md  现行底盘唯一口径：接线 / 烧录 / $CAR 调试
    ├── 焊接与上电检查.md          焊接顺序 / 上电流程 / 黄金测试点 / 异常速查
    ├── 固件与ESP32开发.md        IDF 环境 / 接入路线 / 核心化 / 照跑清单
    ├── 排障记录-固件与ESP32.md    固件、串口、ESP32、S3b 的坑
    ├── 排障记录-服务端与IoT.md    语音网关、IoT 遥测、LLM 层的坑
    └── KunPeng部署与语音网关上板验收.md  板卡局限 / 部署 / 上板验收 / eKuiper 草案
```

> **命名约定**：`Core / Drivers / Middlewares / MDK-ARM` 是 CubeMX + Keil 的**强制约定目录名**
> （`.ioc` 必须与 `Core`、`Drivers` 同级，Keil 工程内 100% 使用 `../Core/...` 相对路径），
> 改名会让 CubeMX 和 Keil 都找不到文件，因此保持原名——这不是"没改"，是**不能改**。

## 开发约定

### 1. 源码编码（防 Keil 中文注释乱码）

**规矩：`Core/` 下参与 Keil 编译的 `.c/.h` 一律存成 UTF-8 with BOM；`.py` / `.md` / `.cpp` 等其余文件存 UTF-8 无 BOM。**

- **为什么**：Keil uVision 打开「UTF-8 无 BOM」的文件时，会按 ANSI(GB2312) 解码，中文注释直接变乱码；带 BOM 才会被自动识别为 UTF-8。反过来，GitHub 网页、VS Code、ESP-IDF、Python 全部按 UTF-8 渲染——若把源码转成 GB2312，**仓库网页上的中文会全部乱码**。所以结论是「**全仓库统一 UTF-8，只给 Keil 源码加 BOM**」，不采用 GB2312。
- **怎么固化**：仓库根 `.editorconfig` 已写好规则——VS Code 新建/编辑 `*.c`、`*.h` 自动存成 UTF-8 with BOM + CRLF；`*.py`、`*.md` 存 UTF-8 + LF；`*.cpp`、`*.hpp` 存 UTF-8 + LF。与 `.gitattributes` 里的 `eol=crlf` / `eol=lf` 一一对应。
- **怎么检查**（提交前跑一次，0 违规再 commit）：
  ```bash
  python tools/check_encoding.py          # 只检查；有违规会列出文件并以退出码 1 结束
  python tools/check_encoding.py --fix    # 顺带把 Core/ 下缺 BOM 的 .c/.h 自动补上
  ```
  覆盖 `Core/`、`esp32/`、`protocol_engine_cpp/`、`docs/`、`kunpeng/`、`iot/`、`tools/` 与根目录散文件，跳过 `Drivers/`、`Middlewares/`、`MDK-ARM/`。
- **提交前自动拦截**：钩子脚本 `tools/hooks/pre-commit` 已入库。**每个新克隆的仓库执行一次** `git config core.hooksPath tools/hooks` 即可生效，之后每次 `git commit` 都会先跑上面这项检查，不合规**直接拦下**（报出具体文件名；`python tools/check_encoding.py --fix` 修好再提交，确需跳过用 `git commit --no-verify`）。钩子文件必须保持 LF，`.gitattributes` 已用 `tools/hooks/*  text eol=lf` 固定，否则 Git Bash 会报 `bad interpreter`。
- **新建文件**：`.c/.h` **不要用 Keil 的 `File → New`**（它按当前 GB2312 设置存盘，正好是乱码那一类），一律在 VS Code 里新建；用 Keil 改完已有文件保存后，也建议再跑一次检查。
- 历史：2026-11-25 由 commit `6fca0b3` 一次性给 15 个源文件补的 BOM（`uart_protocol.c` 原本就带），编译结果 `Code=24440` 与补 BOM 前**完全一致**，说明 BOM 不影响 ARMCC 生成代码。

## 快速开始

### 1. 固件
Keil 打开 `MDK-ARM/RoboticArm_Kunpeng.uvprojx` 编译下载。

### 2. 鲲鹏端（上位机 / 视觉 / 网页）
把 `kunpeng/` 整个目录拷到开发板（openEuler 22.03），在板上执行：
```bash
pip3 install flask pyserial          # 板端已有 ultralytics / opencv / numpy

# 网页遥控：手机/电脑浏览器打开 http://<板IP>:5000，滑杆控 6 路舵机
python3 kunpeng/web/web_control.py /dev/ttyUSB0 --baud 115200 --host 0.0.0.0 --port 5000

# 眼在手点选（A+ 方案）：用「内参 K + 手眼 X + 当前 $ARM,READ 位置」把像素反投影成 3D 点再逆解，
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

> 旧「眼在外 + PC 上位机」版本（Homography 单应标定）已移除；原验收文档（`联调验收清单.md`）已于 2026-11-24 删除，
> 原程序已删除、命令不可照跑。

## 本人负责的模块

本项目固件由 CubeMX 生成外设框架，以下模块为本人手写实现：

- **自定义 UART 指令协议**：帧状态机、XOR 校验、双缓冲应答、USART1/2/3 三路对等路由（同一套代码，无模块专用分支）。
- **四轮麦轮底盘控制（当前主线）**：2×TB6612FNG 双路驱动、4×JGB37-520(1:90) 正交编码器、TIM4 20kHz PWM、麦轮逆/正解、20ms 速度环、梯形限加速、里程计、500ms 指令超时与 300ms 堵转保护。
- **FreeRTOS 多任务架构**设计（当前三任务 + 一把 `print_lock`；早期五任务/三把锁版本含舵机与安全任务，2026-11-24 已随机械臂代码清理）。
- **驱动与功能模块（历史，代码已删）**：PCA9685 驱动、电位器 ADC+DMA 采样滤波、示教录制/回放、RC522、ACS712 过流保护。
- **视觉抓取链路（鲲鹏端 Python，历史）**：YOLOv8n 目标检测、眼在手手眼标定（AX=XB）、六自由度逆解算脉宽、串口发帧与网页遥控；STM32 端协议已删，接入前需重写。

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

**模块 E：`kunpeng/robot/voice_gateway.py`（鲲鹏侧语音入口网关，B 项 / 阶段 2，2026-11-14）**

- **① 用了什么语法**：`sys.path.insert` 跨目录复用 `../llm/` 的解析器与 XOR；`enum.Enum` + `@dataclasses.dataclass` 把仲裁判决与结果关进固定集合（`set_estop` 三态让仲裁器保持**纯函数**）；`socketserver.ThreadingTCPServer` + `StreamRequestHandler` + `threading.Lock` 保护跨连接急停标志；`urllib.request` 调 `/api/cmd`；`argparse --apply` 显式开关；纯函数 `normalize_frame()` 还原 shell 字面量 `\r\n`；**11-14 加固**：`_emit()`（`sys.stdout.reconfigure` + `backslashreplace` 兜底）做 locale 无关输出、`_FrameHandler` **先发应答后打日志**，`_match_resume()` / `_match_estop()` 两个纯函数消除短词表子串歧义。
- **② 为什么这么设计**：做**独立进程**，不改已实机验收的 `web_control.py`、不直连串口（串口由它独占，第二个进程会抢帧），只通过 HTTP 调它已有的 `/api/cmd` ⇒ 删掉即完整回滚。**本地仲裁器白名单只授权 `HOME/STOP/RESUME`** 三个固件原生无参数命令字，从根上不需要软限位裁剪 / `to_pulses()` 反解 / `SAG_DZ` 净空折算；`PICK/PLACE/MOVE_*` 拒、`LOOK` 为 NOOP，落实「LLM 只提议、本地仲裁器才授权」。**默认 dry-run**，`--apply` 才真下发。急停文本匹配放在意图解析之前，且 `$ESTOP*CK` 走快速通道。规范定义：协议适配层（Gateway / Anti-Corruption Layer）+ 本地仲裁器。
- **③ 不这么写会怎样**：按路线图原写法下发 `$JOINT` ⇒ 一步踩进脉宽越界 / `to_pulses()` 符号 / 软限位静默夹紧 / 下垂净空折算四个雷区；把归一化塞进底层 `text_gateway.py` ⇒ 牵动它 18 项单测、且纯函数不该替调用方猜格式；多线程急停标志不上锁 ⇒ 并发漏判；只看 `/api/cmd` 的 `ok:true` 不看 `resp` ⇒ 把「写进串口」当成「固件执行成功」；Flask 没起不兜异常 ⇒ 客户端收不到任何应答；**11-14 实测**：板端 `LANG=C` 而输出层假设 UTF-8 ⇒ 中文启动横幅崩服务、`--self-test` rc=1、日志写在应答前把回帧吞掉；短词表裸子串匹配 ⇒ 喊「解除急停」反而再急停一次（均已加固，见 `kunpeng/robot/README.md` G 节与排障第 6、7 条）。
- ✅ **档位（11-14 16:40 抬，只抬「阶段 2 网关链路」这一条）**：代码 + **38 项本机 unittest 全绿**（含 4 项 ASCII stdout 端到端回归）+ **板端 `--apply` 四层硬判据全绿**（假帧 `$ASR,回初始位*61` → `$OK,HOME*27` → `/api/state` 脉宽回初始位 → 日志 `sent:true` → 肉眼见臂动；记录见 `docs/KunPeng部署与语音网关上板验收.md`）。🔴 **语音入口整体仍 🔶**：`source` 恒 `mock`，没接云端大模型 / ASR（ESP32 已接入但只发链路自检帧）；`$ASR` 帧**语法层**已与 ESP32 本机逐字节对齐、**语义层待 ESP-SR 落地后对齐**；`esp32/` 骨架 11-14 晚已在 PC 上跑通整机 `set-target` + `idf.py build`（`BUILD_RC=0` / `Project build complete` / 产物齐全，证据见清单），✅ **11-21 已烧录实机验证**（SoftAP 起、TCP 9000 往返 `$OK`、采音非零；9001 音频上行待验收）；编译通过 ≠ 实机。三行法详解与可粘贴命令见 `kunpeng/robot/README.md` 第十一节 / E、F、G 节，排障见 `docs/排障记录-服务端与IoT.md`（8 条）。

**模块 F：`esp32/main/tcp_client.c/.h` + `main.c`（Wi-Fi TCP 上行层，2026-11-14 晚）**

- **① 用了什么语法**：BSD socket 四件套 `getaddrinfo()` → `socket()` → `connect()` → `send()/recv()/close()`（头由 **lwip** 组件提供，`main/CMakeLists.txt` 的 `REQUIRES` 必须补 `lwip`）；`setsockopt(SO_RCVTIMEO/SO_SNDTIMEO)` + `struct timeval` 配超时；`enum` 分级错误码 + `tcp_client_strerror()`；`typedef struct` 做「帧表」把数据与发送逻辑分开；C99 八进制转义 `\ooo` 写死中文 UTF-8 字节串；`#if SEND_CN_HOME_FRAME` 编译期安全闸门；`vTaskDelay(pdMS_TO_TICKS())` 让出 CPU。
- **② 为什么这么设计**：骨架是「发一句、等一句」短事务 ⇒ **一次一连接**，不维护长连接状态机、最不容易漏关 fd；`recv()` 故意只收 `reply_size-1`，留一格给 `'\0'`；**超时必须设**，否则网关没起时 `recv()` 永久挂着，连 I2S 采音循环都跑不到、看起来像整机死机；网关 IP/端口集中成宏，换网络改一处重 build；中文帧写成字节值而非直接写汉字，避开源码编码差异，与板端 `printf` 八进制兜底、Python `.encode("utf-8")` 三方对齐。
- **③ 不这么写会怎样**：不补 `REQUIRES lwip` ⇒ 链接期 `undefined reference to getaddrinfo`；不设超时 ⇒ 断网像死机；不 `close()` / `freeaddrinfo()` ⇒ fd 耗尽被误判成「Wi-Fi 掉了」；自检帧若还用 `$ASR,mic_ok*50` / `$ASR,home*63` ⇒ 网关一律 `$ERR,2`（词表只有中文，11-14 实证）；中文帧忘了 `SEND_CN_HOME_FRAME=0`，在 `--apply` 下会**真的让臂回初始位**；网关忘了 `--host 0.0.0.0` ⇒ ESP32 连不上被误判成热点问题。
- ✅ **链路档位（11-21 实机）**：本机帧/网关语义实证 `PASS=24 FAIL=0`、PC 整机 `idf.py build` 通过；**已烧录实机**——SoftAP 起、TCP 9000 往返 `$OK,ESTOP*75` / `$OK,RESUMED*75`、I2S0 采音非零。🔴 语音识别（ESP-SR 命令词）无代码、9001 音频上行待验收，**语音入口整体仍 🔶**。三行法全文见 `esp32/README.md` 第九节，排障见 `docs/排障记录-固件与ESP32.md` 第 3、4 条。

> **精度口径（别写错）**：径向修正补掉的是**定位侧**那 2.98cm 系统偏近；`TIP_BIAS_XY` 补的是**执行侧**爪尖的 0.10/−0.95cm。
> 两者物理上同源（都是 J2/J3 被重力压下），但作用点不同、必须分开扣。
> 网页点选抓取那条路**只做了径向修正、没扣 TIP_BIAS** ⇒ 残留约 1cm；`--hover` 验收路径两边都扣，才是完整闭环。

> **为什么删掉 2D 侧视姿态图**（2026-11-07）：那张图只是把脉宽线性映射成角度画个示意，标题自己就写着「非运动学正解」，
> 没有连杆长度也没有零位偏置，面试官一问「怎么算的」就会露馅；而且 A 方案已经有真逆解，留着反而多余。
> 删掉是**诚实边界**，不是功能倒退。

## 开发中遇到的问题与解决（16 条）

按模块记录开发中实际遇到、并已解决的问题，附根因分析。

> S3b 阶段（鲲鹏视觉服务 + 网页控制台）的排障记录按「现象 → 排查过程 → 根因 → 解决 → 一句话面试说辞」
> 五段式单独维护在 `docs/排障记录-固件与ESP32.md`；本节 14 条是固件 / 旧 PC 上位机阶段的历史记录。

**RFID（RC522）**

- **初始化时好时坏，版本寄存器读回 0x00/0xFF**：原用硬件 SPI Mode 3 与模块时序不匹配，RST 引脚空闲电平实测仅 2.2V 低于复位阈值 2.31V；改为 GPIO 软件 SPI（Mode 0、MSB 先出）后时序完全由代码控制，版本校验改非阻断，并增加"芯片失联自动硬件 RST + 软复位 + 重配寄存器"兜底（后期确认旧模块射频前端损坏，换新模块后全程 Ver=0x82 稳定）。
- **刷卡无反应（REQA fail，Irq=00）**：定时器应答窗口 12.5ms 远大于接收等待循环时长，时序颠倒——循环先耗尽，定时器中断永远来不及置位；窗口缩至 3.2ms（TReload=0x00FA）并加长循环至 3000 次。
- **刷卡偶发失败（Irq=64 / Err=40）**：供电瞬态或接触不良致芯片偶发复位，Err=40 为射频错误；检测周期仅在 1~2ms 内开天线随后立即关闭以降低平均电流，保留硬件 RST + 软复位恢复（静默执行，不打印）。

**蓝牙（HC-05）**

- **手机连接后频繁掉线，指令无响应**：模块直接由 STM32 3.3V 引脚供电，建链瞬间电流不足导致射频复位；另有一批克隆模块设备名非标准、固件不兼容。改为独立稳压 5V 供电并共地，AT 模式配置波特率 115200、8N1、名称 ARM-BT、密码 1234，EN/KEY 拉低断电重启进入透传模式后才可收发。

**机械臂标定与逆运动学**

- **J3/J4 大角度前倾区线性度差，角度拟合误差达 106°**：舵机大角度区输出非线性，且 J3 角度测量方向取反；改为 5 点分段标定并校正方向符号后重测达标。
- **夹爪方向整体偏 90°，沿边抓取失败**：J5 标定时误以夹爪轴线为基准，实际应以夹爪两指连线方向为基准；按两指连线重标 J5，并用 `--grip-conv 90` 约定 J5=0 时张开方向相对臂前向的关系。
- **夹取点相对 J5 旋转轴横向偏心约 1cm，出现位置相关偏差（±3°）**：逆解按夹爪中心计算，未考虑偏心导致的方位角偏差；改为基于 J5 轴点方位建立偏心补偿模型，直立抓不到时前倾角每次 +5°（上限 45°）逐步扩臂。
- **J1/J2 端点脉宽可能堵转，长时间大角度过热**：端点只做快速测量、不长时间停留；各关节软限位写入**旧机械臂固件的 `main.h`**（该文件里的 `SERVO_J*` 宏已于 2026-11-24 随旧代码删除；历史值 J3 520~2500µs、J4 500~1850µs、J5 500~2500µs），上电初始位用独立的 `SERVO_Jx_INIT` 宏（折叠收拢姿态，与各关节 0° 标定零位不是一回事）。

**视觉识别与抓取**

- **相机画面目标乱跳、抓取目标重复**：反光重影、按面积排序抖动、杂物误识别；工作区 ROI 裁剪 + min_area 面积过滤，大工作区下 min_area 从 500 调至 200~300，画面干净稳定。
- **工作区边缘目标方向角偏、抓不准**：相机挂高不足覆盖 250×200mm 工作区，目标越靠边缘、相机越斜，像素形状畸变越大；挂高覆盖全视野，光轴与桌面垂直偏差控制在 10~15° 以内，ROI 像素框用 `set_roi.py` 按工作区实测自动计算，四点围成矩形内精度最佳。
- **标定后抓取位置偏移**：标定后挪动相机或机械臂底座，Homography 与 base_x/base_y 只对同一固定安装有效；改为固定安装（眼在外）后不挪动，标定后把①原点的机械臂坐标实测填入 `config.json` 的 `base_x/base_y`，换过位置或相机就重新执行四点标定。

**串口协议与固件**

- **多串口同时收发时偶发数据错乱**：发送侧双缓冲（活动槽 + 备用槽）在 tx_busy 期间 ISR 写备用槽，主循环搬运与发送前置位存在竞态；搬运与发送前置位两处均用 `__disable_irq`/`__enable_irq` 临界区保护，避免 ISR 误写活动槽或破坏拷贝。
- **蓝牙指令返回 $ERR,1**：校验码错误；上位机自动计算 XOR 校验码（如 `$MOVE,0,1800*28`），无需手算。
- **编译报 huart2 未定义 / 链接缺少 bt 模块**：`stm32f4xx_it.c` 的 USER CODE Includes 段补 `#include "usart.h"`；bt.c 手动加入 Keil 工程的 Application/User/Core 分组。（`wifi.c` 已于 2026-11-26 删除，USART3 改作总线舵机，报「缺少 wifi 模块」属于旧工程残留，删掉即可。）

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
- **已在 `.gitignore` 排除**：Keil 构建产物（`MDK-ARM/RoboticArm_Kunpeng/`、`*.uvoptx`、`*.0000` 等）、CMSIS DSP 官方示例（`Drivers/CMSIS/DSP/Examples/`）、Python 缓存与第三方手册解包文本。