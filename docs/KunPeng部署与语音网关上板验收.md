# KunPeng 部署与语音网关上板验收

> 本文由以下文档合并而成（2026-11-21 整理，内容未改写，仅重组与订正）：
> - KunPeng部署与语音网关上板验收.md
> - KunPeng部署与语音网关上板验收.md
> - KunPeng部署与语音网关上板验收.md
>
> 硬件口径以《硬件清单与接线》与《00-项目总览与硬数据速查》为准。

## 目录

1. 板卡局限与服务端部署
2. 阶段 2 上板验收
3. IoT 遥测与 eKuiper 告警草案

---

## 一、板卡局限与服务端部署

> 来源：`KunPeng部署与语音网关上板验收.md`

> 适用：OrangePi KunPeng Pro v1.1（openEuler 22.03 LTS，aarch64，16GB）——本项目「边/云」层主控。
> 本文分两部分：**第 1~5 节 = 板卡已知局限、供电红线与面试话术**（每条均来自实际上板自检日志 + 官方规格/社区查证）；**第 6~7 节 = 在该板上自托管 xiaozhi 兼容服务端的部署步骤**（语音交互的备选路线，非当前主链路）。
> 三档口径照旧：✅ 实机 / 🔶 有代码未实机 / 📋 规划。

---

### 0 一句话定位

板子**硬件主体健康、可放心用于本项目**；但有两个出厂已知的「设计局限」（NPU 不可用、eMMC 为外接插座），不是使用中弄坏的，必须在方案里写清、在面试里能答。把这两点讲清楚，反而证明真上手摸过板子。

---

### 1 板卡规格与已实测健康项

| 项 | 实测/官方值 | 状态 |
|---|---|---|
| CPU | 4 核 ARMv8-A（华为自研 TaiShan v110 微架构），aarch64 | ✅ 可用 |
| 内存 | 16GB LPDDR4X（系统可用约 15GiB） | ✅ 可用 |
| 系统 | openEuler 22.03 LTS（SP4） | ✅ 已装 |
| 网络 | WiFi5（欧智通 6221BUUC）+ 蓝牙 4.2 + 千兆以太网（RTL8211FW） | ✅ 可用 |
| 接口 | USB3.0 ×2 + Type-C + MicroUSB；40-pin 扩展口（兼容树莓派 HAT 布局，官方 wiringOP） | ✅ 可用 |
| 其它 | I2C eeprom / 显示 `/dev/dri` / CAN 总线 | ✅ 可用 |
| 温度 | 满载前 60~62℃ | ✅ 正常 |

> 硬件自检脚本实测 **PASS=9 / FAIL=1 / WARN=1**；唯一 FAIL 的 eMMC 经核实是脚本误报（外接插座未插），已修正为 INFO。结论：板子主体完全健康。

---

### 2 已知局限清单（均已实跑/查官方核实）

#### 2.1 局限 1：NPU（昇腾 8TOPS INT8）不可用 —— ⚠️ 板级硬件设计缺陷

- **现象**：加载 NPU 驱动/算子时报 `0xE3A203`（官方定义：node type=LPM, sensor type=Chip Hardware, event=obtaining current abnormal，即「取电流异常」）。
- **根因（已查社区证实）**：载板缺失电流检测电阻，是**硬件设计缺陷**，非驱动/配置/操作问题。同型号 RMA 大概率复现，属全系列通病。
- **对本项目影响**：**不构成阻塞**。架构里视觉推理走 KunPeng **CPU(OpenCV/YOLOv8n)** + 云端 LLM/VLM，NPU 支线已放弃。
- **规避**：算力三层——KunPeng CPU 跑视觉/边缘检测、云端跑重推理、ESP32 跑语音。若真要 NPU 算力，用云端 Ascend 实例或 Atlas 200I DK A2，不跟这块板的 NPU 死磕。

#### 2.2 局限 2：eMMC 为可选外接插座（非板载）

- **现象**：`/dev/mmcblk0` 不存在；自检脚本一度误报 FAIL（已修正为 INFO）。
- **根因**：官方设计 eMMC 是「可选外接模块需另购插入」，不是板载焊接。未插模块属完全正常。
- **对本项目影响**：启动盘用 **TF 卡**（与拨码开关「TF 卡启动」吻合），不影响功能。
- **规避**：系统装 TF 卡；数据/大镜像放 NVMe(M.2) 或 USB 盘。

#### 2.3 局限 3：无板载 RTC

- **根因**：板子无电池供电 RTC 电路。
- **规避**：用 `chrony` / `ntpd` 联网校时，系统启动后从 NTP 同步。

#### 2.4 局限 4：SD 卡启动偶发随机失败

- **规避**：内核命令行加 `rootdelay=10`（等 TF 卡设备就绪再挂载根文件系统）。

#### 2.5 局限 5：i2c8 周期性 NAK（幽灵地址）

- **现象**：I2C 扫描 i2c8 偶发 NAK。
- **根因**：已知幽灵地址，与 NPU 缺陷无关，**非硬件故障**，可忽略。

---

### 3 供电与散热红线（非缺陷，是操作要求）

- **20V PD 供电**：随盒附赠 65W PD 电源 + 风扇，**风扇必接**（裸板高负载会过热降频/关机）。
- **移动供电**：板子有 2-pin 电池接口(2.54mm) 可接 3 串锂电（≈11.1V，支持快充）；小车不能拖 220V 线时走电池口。
- **严禁小车 5V 直灌 KunPeng**：KunPeng 用 20V PD 或电池口，与 STM32/ESP32 的 5V/3.3V 系统电源**必须隔离**。

---

### 4 面试应对话术（场景化，直接背）

**Q1：你用的什么板子？几核？**
> “OrangePi KunPeng Pro v1.1，香橙派出的鲲鹏 920 开发板，4 核 ARMv8-A（华为自研 TaiShan v110 微架构）aarch64，16GB LPDDR4X，跑 openEuler 22.03。我选它是因为要原生跑 arm64 的 Docker 容器栈（EMQX/InfluxDB 那些），x86 板子反而要 qemu 模拟。”

**Q2：板子有什么坑/限制吗？**
> “两个出厂已知局限，我都实测 + 查社区确认过：① NPU（昇腾 8TOPS）是载板电流采样电阻缺失的硬件设计缺陷，全系列通病、RMA 也复现，所以视觉推理我没指望 NPU，改走 CPU+OpenCV 加云端大模型；② eMMC 是外接插座得另购模块，我没插，系统跑在 TF 卡上。这些都提前规避了，没卡住项目。”

**Q3：那视觉识别 / AI 推理怎么跑？性能够吗？**
> “分层调度：轻量目标检测在 KunPeng 的 4 核 CPU 上跑 YOLOv8n（320 输入），传统 CV 用 OpenCV 兜底；重推理（LLM 任务规划、复杂识别）走云端 API；语音走 ESP32 端侧。NPU 废了的板子，我用软件栈 + 云端补足，反而体现异构算力调度。”

**Q4：（追问）你确定 NPU 是板子问题不是你配置错？**
> “确定。错误码 0xE3A203 官方定义就是取电流异常，是硬件传感器层面的报错，不是驱动/算子问题。昇腾和鲲鹏社区都有同型号 RMA 复现的帖子，属于公认坑。”

**Q5：启动 / 存储怎么处理的？**
> “TF 卡启动（拨码开关设 TF 启动），SD 卡偶发启动慢我加了 rootdelay 等盘就绪；数据和大镜像放 M.2 NVMe；eMMC 没插不影响功能。”

---

### 5 文档 / 面试千万别踩的雷

- ❌ 写「8 核 A55」——实际 **4 核**。
- ❌ 写/说「NPU 可用、用 NPU 跑推理」——会立刻被戳穿。
- ❌ 把它和 STM32 的 VCAP / ESP32 的 AMS1117 混为一谈（三块板电源架构不同）。
- ❌ 说「完全没问题」——NPU 缺陷客观存在，诚实说「有两个已知局限已规避」更专业。
- ❌ 把「已购订单里的 ESP8266/HC-05/MFRC522」塞进本项目——ESP32-S3 仅 BLE5.0 无经典蓝牙，那些器件本项目不用。

---

### 6 服务端部署（xiaozhi-esp32-server，备选路线）

> 目标：在 KunPeng Pro 上自托管 xiaozhi 兼容服务端，作为 ESP32-S3 语音交互节点的云端（小智兼容服务端）。
> **定位**：这是「像小智那样聊天」路线的服务端；本项目当前主链路是自研 `voice_gateway.py`（见第 7 节），二者不冲突，但**不要同时接线**。
> 参考：https://github.com/xinnan-tech/xiaozhi-esp32-server

#### 6.1 前置确认

- KunPeng Pro 能联网（下镜像/依赖/模型需要）。先 `ping www.baidu.com`。
- 拿到一个 LLM API key（推荐 DeepSeek：https://platform.deepseek.com/ ，新用户有免费额度；或阿里百炼 Qwen）。
- TTS 用 EdgeTTS（免费、纯 Python，无需 key）。

#### 6.2 方式一：Docker 部署（★推荐，ARM64 最稳）

容器内已配好 Python 依赖与 ASR 模型，避免裸装踩坑。

```bash
# 1. 装 docker（openEuler）
sudo dnf install -y docker
sudo systemctl enable --now docker
sudo docker info          # 确认 daemon 起来

# 2. 建工作目录
mkdir -p ~/xiaozhi/data && cd ~/xiaozhi

# 3. 拉部署文件（网络不通可手动从仓库拷）
wget https://raw.githubusercontent.com/xinnan-tech/xiaozhi-esp32-server/main/main/xiaozhi-server/docker-compose.yml
# 把官方 config.yaml 样例下到 data/，改名 .config.yaml 再改
wget https://raw.githubusercontent.com/xinnan-tech/xiaozhi-esp32-server/main/main/xiaozhi-server/config.yaml -O data/.config.yaml

# 4. 编辑 data/.config.yaml（见 6.4）
vi data/.config.yaml

# 5. 启动
docker compose up -d
docker logs -f xiaozhi-esp32-server
```

启动成功日志里会有：`Websocket地址是 ws://xxx:8000/xiaozhi/v1/`、`OTA接口是 .../xiaozhi/ota/`。

> 若 `docker compose pull` 报 ARM64 镜像缺失，说明该镜像未提供 arm64 版本，退回方式二。

#### 6.3 方式二：纯 Python 部署（备选，没装 docker 时用）

⚠️ **ASR 必须改云端（AliyunASR 等），不要用默认 FunASR**——FunASR 依赖 PyTorch，ARM64 裸装极易踩坑。

```bash
# 1. 系统依赖
sudo dnf install -y python3-pip git gcc python3-devel

# 2. 拉代码（注意：此分支为社区 fork，与 6.2 的官方仓库不同，选一个即可）
git clone https://github.com/pluto-xm/xiaozhi-esp32-server.git
cd xiaozhi-esp32-server

# 3. 虚拟环境
python3 -m venv venv
source venv/bin/activate

# 4. 装依赖（换阿里源快）
pip config set global.index-url https://mirrors.aliyun.com/pypi/simple/
pip install -r requirements.txt

# 5. 改 config.yaml（见 6.4，ASR 用 AliyunASR + 云端）
vi config.yaml

# 6. 启动
python app.py
```

#### 6.4 关键配置（config.yaml 片段）

```yaml
selected_module:
  VAD: SileroVAD
  ASR: AliyunASR        # 云端 ASR，避开 ARM 装 PyTorch 的坑（没有阿里 key 就先用 FunASR 本地，但风险自担）
  LLM: DeepSeekLLM
  TTS: EdgeTTS

LLM:
  DeepSeekLLM:
    type: openai
    model_name: deepseek-chat
    url: https://api.deepseek.com
    api_key: sk-你的key

TTS:
  EdgeTTS:
    voice: zh-CN-XiaoxiaoNeural

server:
  ip: 0.0.0.0
  port: 8000

auth:
  enabled: true
  tokens:
    - token: "你的自定义token"     # 烧固件时必须填一致的 token
      name: "我的小智"
```

> `selected_module.LLM` 决定用哪个；也可选 `ChatGLMLLM`(glm-4-flash 免费) / `AliyunLLM`(qwen)。
> `server.port` 默认 8000，WebSocket 地址形如 `ws://192.168.77.100:8000/xiaozhi/v1/`。

#### 6.5 固件端对接（现状：板已有，不必再买）

1. 硬件：**ESP32-S3-WROOM-1 N16R8 成品板已烧录实机**（8MB PSRAM + 16MB Flash）；自研核心板已回板待焊，无需再买 DevKitC-1。
2. 烧录 xiaozhi-esp32 固件时，把服务端地址填成 `ws://192.168.77.100:8000/xiaozhi/v1/`，token 与 6.4 的 `auth.tokens` 一致。
   - 改地址位置：`idf.py menuconfig` → xiaozhi-esp32 配置项，或 `main/boards/<板>/config.h` 的 `WEBSOCKET` 宏。
3. ⚠️ 本项目现有固件是**自研 `kunpeng_voice`**（TCP 9000/9001，见第 7 节），不是 xiaozhi 固件。两条路线要么选一，要么分两个分区/两块板验证，**不要指望同一份固件同时连两个服务端**。

#### 6.6 验证顺序

1. ✅ 服务端起好 → 浏览器打开官方仓库的 `test/test_page.html`（需先从仓库下载到本地，项目内不含此文件），填 WebSocket 地址和 token，点「拨号」能对话 = **云端链路通**（不依赖 ESP32）。
2. ✅ 板端烧 xiaozhi 固件并填地址 → 实体语音对话 = **端云打通**。
3. ⚠️ 想本地看对话只能用该 `test_page.html`，别用浏览器直接开 ws 地址（那会 404）。

#### 6.7 排错

- `LLM 的 API key 未设置`：config 里 key 还是占位符，填真实 key。
- Docker 起不来 / 镜像拉不到 arm64：退回方式二，且 ASR 改 AliyunASR。
- 板子连不上服务端：确认 KunPeng 防火墙放行 8000（`sudo firewall-cmd --add-port=8000/tcp --permanent && sudo firewall-cmd --reload`），且 ESP32 与服务端在同一网段（本项目为 `192.168.77.x`）。

---

### 7 与当前语音主链路的关系（避免混淆）

| 路线 | 服务端 | 端口/协议 | 状态 |
|---|---|---|---|
| **主链路（自研）** | `kunpeng/robot/voice_gateway.py` | TCP **9000** 文本帧 | ✅ 已实机（PC/板侧网关验收） |
| 主链路音频上行 | `kunpeng/robot/audio_sink.py` | TCP **9001** PCM（16kHz/16bit/单声道） | ✅ 已部署实机（10s/320000B） |
| 备选（小智生态） | xiaozhi-esp32-server | WebSocket 8000 + OTA | 📋 未部署 |

- 网络固定口径：ESP32 SoftAP `192.168.77.1`，鲲鹏 STA 静态 `192.168.77.100`。
- 主链路固件：`esp32/main/`（`wifi_ap.c` / `tcp_client.c` / `audio_tx.c`），已烧录实机（TCP 9000/采音），ESP-SR 命令词与 ASR 未接。

---

### 8 关联文档

- `00-项目总览与硬数据速查.md` — 板卡/接口/精度硬数据
- `00-项目总览与硬数据速查.md` — 三主控四板架构与项目定位
- `硬件清单与接线.md` — 采购与型号核对
- `KunPeng部署与语音网关上板验收.md` — 主链路 9000 端口的实机验收记录
- `排障记录-服务端与IoT.md` — 网关侧踩坑记录

---

## 二、阶段 2 上板验收

> 来源：`KunPeng部署与语音网关上板验收.md`

> 定稿 2026-11-14。对应 `docs/固件与ESP32开发.md` 第七节第 1 步（既定顺序里 B 项之后的**第一优先**）。
> ✅ **验收结果（2026-11-14 16:40）：已在鲲鹏板上跑完全部硬判据，四层证据齐，阶段 2 网关链路抬成 ✅**（记录表见第五节，抬档清单见第八节）。
> 🔴 **抬的只是「阶段 2 网关链路」这一条，不是整个语音入口**：`esp32/` 骨架整机 `idf.py build` 11-14 晚已在 PC 上通过，但 11-21 已烧录实机验证（TCP 9000/采音），语音入口整体仍 🔶（识别未接），
> ESP-SR 命令词、Wi-Fi 上行、IoT 节点遥测与告警规则仍 📋/🔶；`source` 恒 `mock`，没接云端大模型 / ASR / ESP32。
> 本文上半部保留**原始验收规程**（可复跑），第五节是**实跑填写后的记录表**。

---

### 零、这份文档是什么 / 不是什么

| | 说明 |
|---|---|
| ✅ 是 | 一份**单行、逐条粘**的板端命令清单 + 精确校验和速查表 + 验收记录模板 |
| ✅ 目的 | 过硬判据：**喂一条假帧 ⇒ 机械臂真的动**（`docs/固件与ESP32开发.md` 阶段 2 原文） |
| ❌ 不是 | 语音识别验收。**全程没有语音、没有 ASR、没有 ESP32、没有大模型**，`source` 恒 `mock` |
| ❌ 不碰 | 棋盘 / 定位链 / 眼在手 / `plane_z` / 手眼 X —— 本验收只走 `网关 → /api/cmd → 固件 → 臂` |
| ❌ 不改 | `web_control.py`、`uart_protocol.c`、`intent_parser.py`、`text_gateway.py`、任何 `.ioc` |

**一句话面试说辞**：「我把『语音识别』和『链路是否通』解耦了 —— 阶段 2 用一条**手写的假帧**去证明收帧、仲裁、下发这条链能真的让臂动，等 ESP32 接上时就只剩『发一个 TCP 帧』。」

#### 三条纪律（先读，不然会踩坑）

1. 🔴 **板端命令一律单行、一条一条粘**。SecureCRT 多行 `python3 -c "..."` 会进入续行并吃掉首字符（11-14 实测踩过）；本文每条命令都是完整一行。
2. 🔴 **PC 侧是 PowerShell 5.1，不支持 `&&`**，所以上传命令分成两行分别粘。板端是 bash，`;` 串联没问题。
3. 🔴 **本验收要让臂动**：动之前必须交代「当前脉宽 / 目标脉宽」，动完必须回初始位（断电前回初始位铁律）。所有 curl 都在本文里给全，别手改数字。

---

### 一、验收前提（逐条确认，不满足就别往下跑）

#### 1.1 🔴 最容易踩的坑：板端是**平铺目录**，网关却要 `import ../llm/`

`voice_gateway.py:71-79` 写死了：

```python
_HERE = os.path.dirname(os.path.abspath(__file__))
_LLM_DIR = os.path.normpath(os.path.join(_HERE, "..", "llm"))
sys.path.insert(0, _LLM_DIR)
from intent_parser import parse_intent, to_dict
from text_gateway import build_frame, parse_frame, xor_checksum, RejectReason
```

而板端 `~/kunpeng_vision/` 是**平铺**的（`web_control.py` / `eye_check.py` / `arm_ik.py` 都在根，之前用 `scp web/web_control.py`、`scp robot/eye_check.py` 直接丢进去）。
**把 `voice_gateway.py` 直接丢进根目录 ⇒ `../llm` 指向 `/root/llm` ⇒ `ModuleNotFoundError`。**

> 本轮本机实测复现了同一现象：在 `robot/` 里先 `from text_gateway import ...` 再 `from voice_gateway import ...`，
> 报 `ModuleNotFoundError: No module named 'text_gateway'`；**反过来先 import voice_gateway**（它会把 `../llm` 插进 `sys.path`）就正常。
> ⇒ 结论：**目录结构对了，就不需要手动设 `PYTHONPATH`**；结构错了，设什么都没用。

**正确做法：在板端镜像本地 `kunpeng/` 的两层结构**（`mkdir -p` 幂等，不碰任何既有文件）：

```bash
mkdir -p /root/kunpeng_vision/llm /root/kunpeng_vision/robot
```

| 板端路径 | 放什么 |
|---|---|
| `/root/kunpeng_vision/llm/intent_parser.py` | 意图翻译员（7 键契约、`source` 恒 mock） |
| `/root/kunpeng_vision/llm/text_gateway.py` | 拆帧 / XOR / 造帧（三个纯函数） |
| `/root/kunpeng_vision/robot/voice_gateway.py` | 网关本体 |

#### 1.2 上传清单（PC 侧 PowerShell，分两行粘）

🔴 **顺序硬约束：必须先在板端做完 1.1 的 `mkdir -p`，再回来跑 1.2 的 `scp`。**
目标目录不存在时 `scp` 会直接失败（不会自动建目录），这是最容易漏的一步。

```powershell
Set-Location "D:\简历\STM32项目\4.ESP32-S3项目\RoboticArm_Kunpeng\kunpeng"
```
```powershell
scp llm/intent_parser.py llm/text_gateway.py root@10.160.120.1:/root/kunpeng_vision/llm/
```
```powershell
scp robot/voice_gateway.py root@10.160.120.1:/root/kunpeng_vision/robot/
```

⚠️ 必须先 `Set-Location` 到 `kunpeng\`。之前踩过「人在 `kunpeng\robot` 却写 `scp robot/eye_check.py`」（等于 `robot\robot\...`）和 PS5.1 不支持 `&&` 两个坑。

#### 1.3 板端自检 —— 这一条同时验「文件齐了 + import 路径对了」

```bash
python3 -B /root/kunpeng_vision/robot/voice_gateway.py --self-test
```

期望 `SELF_TEST PASS` + 一段 7 键 JSON（`action: home` / `source: mock`）。
🔴 **若报 `ModuleNotFoundError` ⇒ 回到 1.1 查目录，别往下跑**。这条自检是整份清单唯一「零硬件就能判结构对不对」的闸门。

#### 1.4 前提清单（打勾再继续）

| # | 前提 | 怎么确认 |
|---|---|---|
| 1 | 板子网口通、能 scp | 上面 1.2 没报错 |
| 2 | `llm/` + `robot/` 结构对 | 1.3 出 `SELF_TEST PASS` |
| 3 | 5000 网页服务在跑 | 2.1 判活 |
| 4 | **串口真连着**（`/api/cmd` 要能写进去） | 2.1 `POST /api/state` 回 `ok:true` + 6 路真实脉宽 |
| 5 | 机械臂供电、周围没人手 | 肉眼确认 |
| 6 | 本验收不碰棋盘 / 定位链 | 不用管棋盘摆哪 |

---

### 二、起服务

#### 2.1 Flask（已实机验收的主链路）

🔴 **如果 5000 已经在跑且串口已 open，就不要重启** —— 重启会清掉 `_IK_MAP` 标定值、打断现场。网关只需要 `/api/cmd`，不需要 `--eye-in-hand`。

**判活（两条都单行）**：

```bash
curl -s -X POST http://127.0.0.1:5000/api/state -H "Content-Type: application/json" -d '{}'
```

期望：`ok:true` + `pulses` 6 路真实脉宽。`/api/state` **没有兜底值**，`ok:true` 就等于「STATE 真回读、串口真连着」。

```bash
curl -s http://127.0.0.1:5000/api/ik/eye
```
（可选，看 `enabled:true` 确认眼在手链路没被破坏；本验收不依赖它。）

**若 5000 没起才用这条**（绝对路径 + `-u`，两个「不带」/ 一个「必带」见 `kunpeng/robot/README.md` C 节）：

```bash
pkill -f web_control.py; sleep 1; nohup python3 -u /root/kunpeng_vision/web_control.py --host 0.0.0.0 --port 5000 --ik-dir /root/kunpeng_vision --eye-in-hand > /root/web.log 2>&1 &
```

- 🔴 **不带 `--radial-fix`**：代码默认 `_EYE_RADIAL_K=1.091`，命令行会覆盖默认值。
- 🔴 **`-u` 必带**：`nohup … > 文件` 时 stdout 是块缓冲，中文启动横幅会卡在内存里，只看日志会误判启动失败。判活以 `curl` 为准，不以横幅为准。

#### 2.2 起网关（`--apply` 是唯一会让臂动的开关）

```bash
pkill -f voice_gateway.py; sleep 1; nohup python3 -u /root/kunpeng_vision/robot/voice_gateway.py --apply > /root/voice_gw.log 2>&1 &
```

```bash
tail -20 /root/voice_gw.log
```

期望看到：

```
语音入口网关 · TCP 127.0.0.1:9000
模式：🔴 --apply 真下发（臂会动！）
下发目标：http://127.0.0.1:5000/api/cmd
```

- 🔴 **11-14 加固**：即使板上 `LANG=C`（stdout 纯 ASCII），中文启动横幅也不会让进程崩掉；若日志里中文显示成 `\uXXXX` 转义属正常（输出层 ASCII 兜底）。**判活以第三节的应答帧 + 日志 `sent:true` 为准**，别因横幅乱码以为服务没起来。详见 `docs/排障记录-服务端与IoT.md` 第 6 条。
- 默认只监听 `127.0.0.1`，板端本机 `nc` 够用，**更安全**（不把会动臂的端口暴露给外部）。将来 ESP32 要连进来才显式加 `--host 0.0.0.0`。
- `pkill -f voice_gateway.py` 不会误杀 Flask（进程名不同）。
- 🔴 **网关与固件的急停是两套状态**：网关重启后自己的 `_estop` 清零，但**固件可能还在 ESTOP** ⇒ 发 `HOME` 会收到固件 `ERR,4`，网关回 `$ERR,4*5D`。遇到就先补一帧 `$RESUME*19`（第七节）。

---

### 三、帧速查表（精确 CK，别手算）

CK = payload（`$` 与 `*` 之间）**逐 UTF-8 字节 XOR**、2 位大写 HEX。中文一字 3 字节，不能按 Unicode 码点算（否则与 C 固件对不上）。
下列 CK 全部用 `text_gateway.build_frame()` / `xor_checksum()` 实算并本机验证过。

#### 3.1 请求帧（发给 `:9000`）

| 用途 | 帧（`\r\n` 是真实 CRLF） | CK | 期望 |
|---|---|---|---|
| **回初始位 —— 唯一让臂动的帧** | `$ASR,回初始位*61\r\n` | `61` | `$OK,HOME*27` |
| 抓取（应被仲裁器拒） | `$ASR,抓取*6F\r\n` | `6F` | `$ERR,3*5A` |
| 急停（ASR 文本帧） | `$ASR,急停*57\r\n` | `57` | `$OK,ESTOP*75` |
| 恢复（ASR 文本帧） | `$ASR,恢复*65\r\n` | `65` | `$OK,RESUMED*75` |
| 解除急停（11-14 加固，等价恢复） | `$ASR,解除急停*6F\r\n` | `6F` | `$OK,RESUMED*75` |
| 看看桌面（NOOP，不动臂） | `$ASR,看看桌面*71\r\n` | `71` | `$OK,LOOK_NOOP*6E` |
| 急停快速通道（**纯 ASCII**） | `$ESTOP*5D\r\n` | `5D` | `$OK,ESTOP*75` |
| 恢复快速通道（**纯 ASCII**） | `$RESUME*19\r\n` | `19` | `$OK,RESUMED*75` |

> 🔴 **11-14 代码审查加固**（详见 `docs/排障记录-服务端与IoT.md` 第 6、7 条）：
> ① 网关输出已做成 **locale 无关**，`LANG=C` / `PYTHONIOENCODING=ascii` 下也能起服务、`--self-test` rc=0、中文帧照样回 `$OK,HOME*27`；
> ② **应答帧先于日志发出**，日志再出编码问题也不会吞掉回帧；
> ③ 「解除急停」「继续」等恢复词不再被「急停」子串反向误判，「stop」「resume」在两层都识别。
> ⚠️ 若板上 locale 是纯 ASCII 且 `reconfigure` 不可用，日志/横幅里的中文可能显示成 `\uXXXX` 转义 —— **帧与应答本身是纯 ASCII，不受影响**，判活仍以应答帧 + `sent:true` 为准。

#### 3.2 应答帧（网关回给客户端）

| 应答 | CK | 含义 |
|---|---|---|
| `$OK,HOME*27\r\n` | `27` | 已下发 `HOME`，固件回 `OK` |
| `$OK,ESTOP*75\r\n` | `75` | 已下发 `STOP`，网关进入冻结 |
| `$OK,RESUMED*75\r\n` | `75` | 已下发 `RESUME`，解除冻结 |
| `$OK,LOOK_NOOP*6E\r\n` | `6E` | 允许但不动臂 |
| `$ERR,1*58\r\n` | `58` | 帧格式 / XOR 校验错 |
| `$ERR,2*5B\r\n` | `5B` | 未识别动作 |
| `$ERR,3*5A\r\n` | `5A` | 本地仲裁器拒绝（含急停冻结中） |
| `$ERR,4*5D\r\n` | `5D` | 下发失败（Flask 没起 / 固件回 ERR） |

错误码沿用项目统一语义，与 `Core/Src/uart_protocol.c`、`car_frame.c` 同一套。

#### 3.3 🔴 中文乱码的判定与兜底（这一步很关键）

帧里含中文，SecureCRT 会话编码若不是 UTF-8，粘进去字节会被改 ⇒ CK 不匹配 ⇒ 一律 `$ERR,1*58`。

**核对 CK**（板端，中文只出现在参数里、不是发帧）：

```bash
python3 -B /root/kunpeng_vision/robot/voice_gateway.py --make-frame "回初始位"
```
期望打印 `'$ASR,回初始位*61\r\n'`。

**兜底 A —— 纯 ASCII 八进制字节**（POSIX `printf` 通用，不依赖终端编码）：

```bash
printf '$ASR,\345\233\236\345\210\235\345\247\213\344\275\215*61\r\n' | nc 127.0.0.1 9000
```
`\345\233\236` = E5 9B 9E = 「回」，四组共 12 字节 = 「回初始位」的 UTF-8；XOR 仍是 `61`。本机已验证 `parse_frame` 通过、CK=61。

**全部 6 条中文帧的八进制版**（11-14 端到端实算：逐条验证 UTF-8 字节等价 + CK + 实际应答，可直接替换 3.1 表里任意一条）：

```bash
printf '$ASR,\345\233\236\345\210\235\345\247\213\344\275\215*61\r\n' | nc 127.0.0.1 9000   # 回初始位  -> $OK,HOME*27
printf '$ASR,\346\212\223\345\217\226*6F\r\n' | nc 127.0.0.1 9000                       # 抓取      -> $ERR,3*5A
printf '$ASR,\346\200\245\345\201\234*57\r\n' | nc 127.0.0.1 9000                       # 急停      -> $OK,ESTOP*75
printf '$ASR,\346\201\242\345\244\215*65\r\n' | nc 127.0.0.1 9000                       # 恢复      -> $OK,RESUMED*75
printf '$ASR,\350\247\243\351\231\244\346\200\245\345\201\234*6F\r\n' | nc 127.0.0.1 9000 # 解除急停  -> $OK,RESUMED*75
printf '$ASR,\347\234\213\347\234\213\346\241\214\351\235\242*71\r\n' | nc 127.0.0.1 9000 # 看看桌面  -> $OK,LOOK_NOOP*6E
```

🔴 只能用 `printf`（会解释 `\ooo` / `\r` / `\n`），**别换成 `echo`**（默认不解释转义 ⇒ 发出去的是坏帧）。
中文会话确认是 UTF-8 时，也可直接用 3.1 表的中文 `printf` 版，二者字节完全相同（本机已实证）。

**兜底 B —— 不依赖 `nc`**（有的 openEuler 镜像没装 `nc`）：

```bash
python3 -c 'import socket;s=socket.create_connection(("127.0.0.1",9000));s.sendall(b"$ASR,\345\233\236\345\210\235\345\247\213\344\275\215*61\r\n");print(repr(s.recv(256)));s.close()'
```

🔴 **`home` 词表只有中文**（`intent_parser.py:92`：`回初始位 / 回到初始 / 归位 / 复位 / 收起`）。
`$ASR,home*63\r\n` 会落 `unknown` ⇒ **`$ERR,2*5B\r\n`，不能拿英文代替**（本机已验证）。
⇒ 只有**急停 / 恢复**有纯 ASCII 快速通道（`$ESTOP` / `$RESUME`）；让臂动的那条**必须带中文或用八进制字节**。
⚠️ 这也顺带暴露一个阶段 4 待办：将来 ESP32 端侧命令词映射，得发中文文本或扩充词表。

---

### 四、硬判据：喂假帧 ⇒ 机械臂真的动

> **客观证据要四层齐全**，缺一层都不算过：
> ① 网关应答帧 ② 网关日志 `sent:true` ③ `/api/state` 脉宽真的变了 ④ 肉眼看到臂动。
> 只看到 `$OK,HOME` 不算 —— `ok:true` 只代表串口写进去了，真结果在 `resp` 里（`send_cmd()` 已替你查）。

#### 4.1 先把臂摆到 **C 姿态**（让它不在初始位，否则「动」看不出来）

C 姿态 6 路脉宽 `1636,1470,2177,1106,1450,1567`：

```bash
curl -s -X POST http://127.0.0.1:5000/api/cmd -H "Content-Type: application/json" -d '{"payload":"JOINT,1636,1470,2177,1106,1450,1567"}'
```
期望 `{"ok":true,"resp":"OK"}`。⚠️ `/api/cmd` 的 `payload` 必须**逗号分隔**，写空格会 `ERR,1`。

```bash
curl -s -X POST http://127.0.0.1:5000/api/state -H "Content-Type: application/json" -d '{}'
```
确认 `pulses` = `1636,1470,2177,1106,1450,1567`（**这是「动之前」的基线**）。

#### 4.2 喂假帧 → 臂真的回初始位

```bash
printf '$ASR,回初始位*61\r\n' | nc 127.0.0.1 9000
```
期望 `$OK,HOME*27\r\n`。中文乱码就换 3.3 的兜底 A / B。

```bash
curl -s -X POST http://127.0.0.1:5000/api/state -H "Content-Type: application/json" -d '{}'
```
🔴 **期望 `pulses` = `1580,1900,2400,700,1450,1500`（初始位）** —— 这是「臂真的动了」的硬证据。

```bash
tail -5 /root/voice_gw.log
```
期望日志里该条 `"payload": "HOME"`、`"sent": true`、`"apply": true`。

#### 4.3 `PICK` 必须被拒，且**臂不动**

```bash
printf '$ASR,抓取*6F\r\n' | nc 127.0.0.1 9000
```
期望 `$ERR,3*5A\r\n`；再 `POST /api/state` 确认脉宽**没变**（仍是初始位）。

#### 4.4 急停冻结 → 恢复 → 再动（四帧，逐条粘）

```bash
printf '$ASR,急停*57\r\n' | nc 127.0.0.1 9000
```
期望 `$OK,ESTOP*75\r\n`（固件进入 ESTOP，冻结输出）。

```bash
printf '$ASR,回初始位*61\r\n' | nc 127.0.0.1 9000
```
期望 `$ERR,3*5A\r\n` —— **急停中，连 HOME 都不授权**（网关层拦住，根本没下发）。

```bash
printf '$ASR,恢复*65\r\n' | nc 127.0.0.1 9000
```
期望 `$OK,RESUMED*75\r\n`。

```bash
printf '$ASR,回初始位*61\r\n' | nc 127.0.0.1 9000
```
期望 `$OK,HOME*27\r\n` —— 恢复后能再动。

#### 4.5 纯 ASCII 快速通道（可选，验将来 ESP32 的最短帧）

```bash
printf '$ESTOP*5D\r\n' | nc 127.0.0.1 9000
```
```bash
printf '$RESUME*19\r\n' | nc 127.0.0.1 9000
```
期望分别 `$OK,ESTOP*75` / `$OK,RESUMED*75`。这两帧走 `special_text()` 快速通道（否则会被 `parse_frame` 判 `NOT_ASR` 丢掉），**CK 仍由 `parse_frame` 校验过，没有绕过校验**。

#### 4.6 `ERR,4` 容错（可选，验不崩）

停掉 Flask 再喂一帧（或临时改 `--base-url` 到不存在的端口重启网关），期望 `$ERR,4*5D` 且网关进程不崩、日志有 `downlink_error`。**验完记得把 Flask 起回来**。

---

### 五、验收记录表（2026-11-14 16:40 板端实跑，已填写）

> ✅ **11 行全绿 + 四层硬判据齐 ⇒ 阶段 2 网关链路抬 ✅**。板：`openEuler / 10.160.120.1`。

| # | 命令 | 期望应答 | 实测应答 | `/api/state` 脉宽 | 臂是否动 | 通过 | 时间 / 备注 |
|---|---|---|---|---|---|---|---|
| 0 | `--self-test` | `SELF_TEST PASS` | `SELF_TEST PASS` + `action:home` / `source:mock` | — | — | ✅ | 两层目录结构对、`../llm` import 正常 |
| 1 | `POST /api/state` | `ok:true` + 6 路脉宽 | `ok:true` | `1636,1470,2177,1106,1450,1567`（C 姿态基线） | — | ✅ | 另 `POST /api/open` 回 `ping:"OK,PONG"` + by-id 串口号 |
| 2 | 摆 C 姿态 curl | `resp:"OK"` | `{"ok":true,"resp":"OK"}` | `1636,1470,2177,1106,1450,1567` | 是 | ✅ | `payload` 逗号分隔正确 |
| 3 | `$ASR,回初始位*61`（八进制字节） | `$OK,HOME*27` | `$OK,HOME*27` | **`1580,1900,2400,700,1450,1500`** | **是（用户回报「看到臂从 C 姿态收回初始位」）** | ✅ | ← **硬判据** |
| 4 | `tail -5 voice_gw.log` | `sent:true` / `apply:true` | `{"payload":"HOME","apply":true,"sent":true}` | — | — | ✅ | |
| 5 | `$ASR,抓取*6F` | `$ERR,3*5A` | `$ERR,3*5A` | 网关层拒绝、根本未下发 | 否 | ✅ | |
| 6 | `$ASR,急停*57` | `$OK,ESTOP*75` | `$OK,ESTOP*75` | — | 否 | ✅ | |
| 7 | `$ASR,回初始位*61`（急停中） | `$ERR,3*5A` | `$ERR,3*5A` | 没动 | 否 | ✅ | |
| 8 | `$ASR,恢复*65` | `$OK,RESUMED*75` | `$OK,RESUMED*75` | — | — | ✅ | |
| 9 | `$ASR,回初始位*61`（恢复后） | `$OK,HOME*27` | `$OK,HOME*27` | 初始位 | 是 | ✅ | 臂已在初始位，只证「恢复后授权放行」 |
| 10 | `$ESTOP*5D` / `$RESUME*19` | `$OK,ESTOP*75` / `$OK,RESUMED*75` | 均对 | — | — | ✅ | 可选；另 `解除急停`→`$OK,RESUMED*75`（加固项实机通过）、`看看桌面`→`$OK,LOOK_NOOP*6E`、python socket 兜底→`b'$OK,HOME*27\r\n'` |
| 11 | 收尾 `POST /api/state` | 初始位 6 路脉宽 | `ok:true` | `1580,1900,2400,700,1450,1500` | — | ✅ | 补 `$RESUME*19` 后 `pkill -f voice_gateway.py`，未留 `--apply` 进程 |

**实机回报补充**：① 无中文乱码 / 无 `$ERR,1`（全程用 POSIX `printf` 八进制字节兜底）② 板上 `nc` 存在（另用 python socket 兜底也通过）③ 网关日志无异常栈 ④ 棋盘 / 定位链 / `plane_z` 全程未触碰。

⚠️ **两个「非失败」的现场观察**（详见 `docs/排障记录-服务端与IoT.md` 第 8 条）：
① A4 已 `ok:true` 仍跑了条件触发的 A6 **重启 Flask** —— 命令带 `--ik-dir /root/kunpeng_vision --eye-in-hand` 且**不带 `--radial-fix`**，重启后 `/api/cmd` / `/api/state` 全成功 ⇒ 串口确实连着；但 `/api/ik/eye` 的 `enabled:true` 是**重启前**查的，重启后未复查 ⇒ 「重启后眼在手仍 enabled」没有证据，只能说本验收不依赖它。属不必要重启、无损害。
② B2 刚起后台进程就 `tail -20` 得到空输出，后面 `tail -5` 证明日志正常写入；SecureCRT 多行粘贴有回显串扰（命令残影混入），命令本身均执行成功。

---

### 六、收尾与安全

```bash
curl -s -X POST http://127.0.0.1:5000/api/state -H "Content-Type: application/json" -d '{}'
```
🔴 **确认臂停在初始位** `1580,1900,2400,700,1450,1500`（断电前回初始位铁律）。若上一步停在 ESTOP，先补：

```bash
printf '$RESUME*19\r\n' | nc 127.0.0.1 9000
```

```bash
pkill -f voice_gateway.py
```
验收完把 `--apply` 网关停掉（或重启成不带 `--apply` 的 dry-run）。**别把会动臂的 `--apply` 进程留着**。

---

### 七、边界与坑（面试官会追问的部分）

| # | 现象 | 根因 | 处理 |
|---|---|---|---|
| 1 | `ModuleNotFoundError: text_gateway` | 板端平铺目录，`../llm` 指错 | 1.1 建 `llm/` + `robot/`，用 1.3 自检验 |
| 2 | `$ASR,home*63` 回 `ERR,2` | HOME 词表只有中文 | 用中文帧或八进制字节；阶段 4 再扩词表 |
| 3 | 好帧回 `$ERR,1` | SecureCRT 非 UTF-8，中文被改坏 ⇒ CK 不符 | 3.3 核对 `--make-frame` + 兜底 A/B |
| 4 | 网关重启后 `HOME` 回 `$ERR,4` | 网关 `_estop` 清零但**固件仍在 ESTOP** | 先补 `$RESUME*19`；这是两套状态，不是 bug |
| 5 | `--apply` 但 Flask 没起 | `send_cmd()` 抛 `URLError` | 已收口成 `$ERR,4` 且不崩（38 项单测覆盖） |
| 6 | `nc` 命令不存在 | 镜像没装 netcat | 用 3.3 兜底 B 的 Python socket 一行 |
| 7 | `printf` 管道给 `nc` 挂住不返回 | 帧没带 `\n`，`readline()` 一直等 | handler 读一行回一帧后关连接；帧必须带 `\n`（`echo` 也够，`rstrip("\r\n")` 容忍） |
| 8 | 只看到 `$OK,HOME` 就以为成功 | `/api/cmd` 的 `ok:true` 只＝串口写进去了 | 必须回读 `/api/state` 看脉宽（第四节四层证据） |
| 9 | 网关日志/横幅中文变成 `\uXXXX`、或服务直接起不来 | 板上 `LANG=C`，stdout 只认 ASCII，中文 `print` 抛 `UnicodeEncodeError`（11-14 实测） | **已加固**：输出走 UTF-8 优先 + ASCII 转义兜底，服务能起、应答先于日志发出；帧与应答是纯 ASCII，不受影响（排障记录第 6 条） |
| 10 | 喊「解除急停」却回 `$OK,ESTOP*75` | 短词表子串歧义（「急停」⊂「解除急停」），旧裸匹配反向命中（11-14 实测） | **已修复**：现正确回 `$OK,RESUMED*75`；混合语句按恢复属 mock 已知边界（排障记录第 7 条） |

🔴 **硬判据过了 ≠ 语音控制实现**。这事只证明「网关 → Flask → 固件 → 臂」链路通，与语音无关：
`source` 恒 `mock`，没接云端大模型 / ASR / ESP32；`$ASR,<文本>*<CK>` 仍是 `text_gateway.py` 的**建议草案、未与 ESP32 实机对齐**。
急停只是**网关层冻结后续下发 + 转发 `STOP`**，仍是开环、无位置反馈，**不是**功能安全认证意义上的急停。

---

### 八、跑完之后：档位怎么改（由 assistant 统一改，别自己先抬）

> ✅ **本节已于 2026-11-14 16:40 执行完毕**：第五节记录表全绿并回报四层证据后，assistant 按下表统一做了**限定抬档**（阶段 2 网关链路 🔶→✅，其余 ESP32 / ESP-SR / IoT 口径原样不动）。下表保留作为「改了哪些文件」的归档清单；后续复跑本规程不需要再抬一次档。

前置：**只有第五节记录表全绿并回报**，才动下面这些文件；任何一条没过都保持 🔶 并在表里注明。

| 文件 | 改什么 |
|---|---|
| `.workbuddy/memory/MEMORY.md` §8 | B 项档位 🔶 → ✅（限定为「阶段 2 网关链路实机打通」），补验收日期与证据 |
| `docs/固件与ESP32开发.md` | 阶段 2 表状态、第一节支柱 ② 缺口、第七节下一步第 1 条 |
| `docs/面试与简历材料.md` | 头部 B 项增补、第二节 🔶 行、第三节落地顺序、第五节能说 / 不能说、第六节证据清单 |
| 根 `README.md` | 支柱 ② 表 / 待打通清单 / 当前进度表 |
| `kunpeng/robot/README.md` | E 节诚实边界、F 节验收状态 |
| `kunpeng/llm/README.md` | 接口表阶段 2 下游状态 |
| `docs/排障记录-服务端与IoT.md` | 若实机踩到新坑，按「现象 → 排查 → 根因 → 解决 → 一句话面试说辞」追加，**只记关键的** |
| `.workbuddy/memory/YYYY-MM-DD.md` | 追加当天实机验收日志 |

🔴 **抬的是「阶段 2 链路」这一条，不是整个语音入口**：
`esp32/` 骨架整机 `idf.py build` 已在 PC 上通过，但 11-21 已烧录实机验证（TCP 9000/采音）；ESP-SR 命令词、Wi-Fi 上行、IoT 节点遥测与告警规则仍 📋/🔶。
正文里 ✅ 只写「鲲鹏侧收帧网关与本地仲裁器已上板验证：假帧驱动机械臂回初始位、PICK 被拒、急停可冻结」，
**不许写**「语音控制已实现 / 语音→动作闭环 / 已接 ESP32 语音链路 / 自然语言控制机械臂」。

> 既定顺序不变：本步过完 → 阶段 0~1（ESP32 烧录 + 发帧；整机 build 已通过）→ 阶段 3~4（ESP-SR，唯一有技术不确定性的一段）→ 阶段 5~6（ESP32 遥测 + eKuiper 告警）。见 `docs/固件与ESP32开发.md` 第七节。

---

## 三、IoT 遥测与 eKuiper 告警草案

> 来源：`KunPeng部署与语音网关上板验收.md`

> 📋 **提前草案 / 未实机**：本文件 2026-11-15 傍晚在宿舍写就（身边只有鲲鹏板、教室硬件不在）。
> 它**不是**「ESP32 遥测已接入」，只是把「不依赖教室硬件、能在本机验证」的
> **字段契约 + 板侧适配层 + 规则文本**先落地，等回教室烧录 / 联调时照着接。
> **三档**：支柱③ 仍是 ✅ 板内四容器 / 🔶 未覆盖 ESP32 节点、无告警规则 —— 本草案**不改变档位**。

### 0. 为什么提前写、以及它在路线图的位置

- 用户 11-14 重排的下一步：①先接 ESP32-S3 → ②ESP32 真发帧 → ③ESP-SR →
  **④IoT 补 ESP32 节点遥测 + eKuiper 告警** → ⑤ACS712 → ⑥A 项实机验收包。
- 本草案对应第 ④ 步，即 `docs/固件与ESP32开发.md` 的**阶段 5 / 阶段 6**（第 58、59 行）。
  按排序它必须排在「ESP32 真发帧」之后，所以现在只写**可静态校验、可本机单测**的部分，
  不碰任何已实机的板内组件。
- 大白话：宿舍没硬件，能干的只有「把规矩定清楚、把代码写好、在本机跑通」；
  真正连 EMQX / eKuiper / InfluxDB / 真 ESP32 节点，都得回教室。

### 1. 边界：本轮动了什么、绝对没动什么

| 类别 | 文件 | 状态 |
|---|---|---|
| 🆕 新增 | `iot/esp32_telemetry.py` | 字段契约 + 板侧适配层 |
| 🆕 新增 | `iot/esp32_rules.json` | 2 流 3 规则草案文本（`meta.status = "draft"`） |
| 🆕 新增 | `iot/test_esp32_telemetry.py` | 27 项 unittest，零第三方、零联网 |
| 🆕 新增 | `docs/KunPeng部署与语音网关上板验收.md` | 本文件 |
| ✏️ 追加 | `iot/README.md` 末尾三行法 | 按项目「每模块补三行」规矩 |
| ✏️ 追加 | `.workbuddy/memory/2026-11-15.md`、`MEMORY.md` | 日志 / 指针同步 |

🔴 **绝对没动**（越线即破坏已实机现场）：

- `iot/docker-compose.yml`：板内四容器已实机，且读取被敏感保护拦截 ⇒ 不读、不改、不推断。
- `iot/deploy.sh`：里面 `to_influx` 规则已实机，本轮不改一行（但落地前**必须**给它加过滤，见 §4.2）。
- `esp32/` 固件源码：明天要照 `docs/固件与ESP32开发.md` 烧录，当前编译副本
  `D:\esp\s3_voice` 已整机 build 通过，一行不动。
- `.ioc`：不碰 CubeMX 配置；不装任何包。
- `docs/固件与ESP32开发.md` 的**阶段表与档位结论**：本文件只**引用**路线图第 58/59/71/72 行，不改它。
- 根 `README.md`、`docs/面试与简历材料.md`、`docs/00-项目总览与硬数据速查.md`：未实机内容不扩散到对外口径。
- **不抬档**。

### 2. 字段契约（全部来自路线图，不自己发明）

主题与 measurement（对齐 `docs/固件与ESP32开发.md` 第 71、72 行）：

| 项 | 值 | 出处 |
|---|---|---|
| 遥测主题 | `device/esp32_01/telemetry` | 路线图第 71 行 |
| 告警主题 | `alerts/esp32_01` | 草案自定义（`alerts/+` 通配覆盖） |
| 遥测 measurement | `esp32_telemetry` | **独立**，绝不混进 `arm_telemetry` |
| 告警 measurement | `alerts` | 路线图第 72 行 |
| 告警阈值 | `rssi < -70 dBm`、心跳超时 `> 10 s` | 路线图第 59、72 行 |

payload 字段：

| 字段 | 含义 | 类型 / 合法范围 | InfluxDB 里 |
|---|---|---|---|
| `deviceId` | 设备 ID，恒 `esp32_01` | string | tag `device` |
| `ts` | Unix 秒 | int | 不选（避开事件时间字段歧义） |
| `rssi` | Wi-Fi 信号强度 | float ∈ [-120, 0]，dBm 不为正 | field (float) |
| `heap_free` | 剩余堆字节 | 非负整数 | field (integer) |
| `wake_cnt` | 唤醒累计次数 | 非负整数 | field (integer) |
| `cmd_ok` | 命令词识别成功累计次数 | 非负整数 | field (integer) |

> 🔴 `esp32_telemetry.py` 里的 `DEMO_VALUES = (-65.0, 120000.0, 7, 5)` 只是给 `--dry-run` 有东西可打印，
> **不是真实 ESP32 遥测**；真节点的取数逻辑（读 `esp_wifi_sta_get_ap_info()` 的 rssi、
> `esp_get_free_heap_size()`、唤醒 / 识别计数）**尚未写**。

#### 概念（大白话 + 规范定义）

- **MQTT 主题**：大白话 = 设备把消息贴到一块「公告栏」，订阅这块板的人就收到；
  规范定义 = MQTT 3.1.1 / 5.0（OASIS 标准）基于 TCP 的发布-订阅协议，
  主题用 `/` 分层、`+` 为单层通配符、`#` 为多层通配符。
- **InfluxDB field 类型锁定**：大白话 = 一格第一次装了「小数」，以后就只能装小数，
  塞字符串会连累整批写不进去；规范定义 = InfluxDB 2.x 的 field value 只允许
  `float / integer / unsignedInteger / string / boolean` 五种标量，且每个
  `measurement + field` 的类型**首次写入即锁定**（`docs/排障记录-服务端与IoT.md` 第 11 条的 Root Cause）。
- **eKuiper 流 / 连续查询**：大白话 = 在板子本地用一句 SQL 把流过的数据筛一遍再转发，不用上云；
  规范定义 = 面向无界流的连续查询引擎，用类 SQL 对数据流做投影 / 过滤 / 聚合，
  schemaless 建流必须写空括号 `()`。
- **paho 延迟导入**：大白话 = 只在真要连 MQTT 时才 `import`，没装包的机器也能跑自测；
  规范定义 = Python 模块导入发生在运行期，把 `import` 放进函数体可把
  `ModuleNotFoundError` 推迟到该分支真正执行。
- **`${INFLUX_TOKEN}` 占位符**：大白话 = 别把密码写进文件，部署时由环境变量临时填；
  规范定义 = 12-factor app 的「配置与代码分离」，凭据走环境变量 / secret store，不进版本库。

### 3. 三个模块各自做了什么

#### 3.1 `esp32_telemetry.py`（契约 + 板侧适配层）

纯函数为主，方便零联网自测：

| 函数 / 开关 | 职责 |
|---|---|
| `build_payload()` | 拼装字段契约；脏值经 `_clean_number()` 洗成 `None` |
| `is_publishable()` | 四个契约字段必须全是真数字（显式排除 `bool`）才允许发 |
| `is_heartbeat_stale()` | `now - last_ts > 10` 判超时；时钟回拨返回 False 防误报 |
| `build_alert()` | 只生成 `rssi_low` / `heartbeat_timeout` 两种告警，未知类型直接抛错 |
| `parse_values()` | 解析 `--values=-82,120000,7,5` |
| `make_client()` | paho 延迟导入 + 兼容 2.x / 1.x 回调 API |
| `--dry-run` / `--publish` | 默认只打印；**dry-run 优先**，同时给两个参数也绝不连 MQTT |

#### 3.2 `esp32_rules.json`（静态可校验文本）

`meta.status = "draft"`，2 个流、3 条规则，**未部署、未连任何服务**：

| id | 类型 | SQL / 动作 |
|---|---|---|
| `esp32_stream` | 流 | `DATASOURCE="device/esp32_01/telemetry"`（精确主题） |
| `alert_stream` | 流 | `DATASOURCE="alerts/+"` |
| `esp32_to_influx` | 规则 | `SELECT rssi, heap_free, wake_cnt, cmd_ok, deviceId` → `esp32_telemetry` measurement |
| `esp32_rssi_low` | 规则 | `WHERE rssi < -70` → MQTT 回 `alerts/esp32_01` |
| `esp32_alerts_to_influx` | 规则 | `SELECT deviceId, kind, level, detail` → `alerts` measurement |

#### 3.3 `test_esp32_telemetry.py`（27 项 unittest）

覆盖纯函数、规则 JSON 静态结构、子进程 CLI / ASCII stdout 回归。
🔴 只证明「本机逻辑对」，**不证明**已部署 / 已上板 / 已接入 ESP32。

### 4. 五个关键技术决策（讲根因，不是调参打补丁）

#### 4.1 脏数据在「发布端」挡掉，不靠下游

- **现象**：串口断连 / 字段缺失时，eKuiper influx2 sink 不跳过 JSON `null`，会把它序列化成 **string**，
  撞上历史点锁定的 **float** ⇒ InfluxDB `partial write: field type conflict`、整批写不出去。
- **根因**：`docs/排障记录-服务端与IoT.md` 第 11 条 —— InfluxDB 2.x 类型首次写入即锁定；
  「进程不崩」不等于「数据无害」。
- **解决**：`is_publishable()` 在 `client.publish()` 之前判定，四字段必须全是真数字，
  `bool` 显式排除（Python 里 `bool` 是 `int` 子类，`isinstance(True, int)` 为 True，
  不排除会把 `True` 写成 1）。
- **一句话面试说辞**：「我不在下游清库或换 measurement 绕开，而是在发布端保证每一格都是合法标量，
  因为时序库的 field 类型是写一次就锁死的。」

#### 4.2 独立 measurement + 精确 DATASOURCE，且落地前必须改 `deploy.sh` 的过滤

- **现象**：现有 `deploy.sh` 的 `to_influx` 订阅 `device/+/telemetry` **通配**，
  会把 `device/esp32_01/telemetry` 也当成机械臂数据，`SELECT` 出 `j1~j6` 等 `null` 字段写进 `arm_telemetry`。
- **根因**：通配主题 + schemaless 流 ⇒ 契约外消息也会进同一条规则。
- **解决（草案）**：ESP32 用**精确** `DATASOURCE` 的独立流 + 独立 `esp32_telemetry` measurement 隔离。
- 🔴 **落地前必做**：给 `to_influx` 的 SQL 加 `WHERE deviceId='kunpeng_01'`，否则隔离不完整。
  **本草案不擅自改 `deploy.sh`**（已实机文件），只在 `esp32_rules.json` 的
  `meta.isolation_note` 里钉死这条待办。
- **一句话面试说辞**：「两套设备共用通配主题时，先按 deviceId 分流再写库，否则类型冲突是迟早的。」

#### 4.3 心跳超时放板侧看门狗，不用 eKuiper 窗口

- **根因**：窗口 / 触发器由「收到消息」驱动；设备彻底离线时**一条消息都没有**，SQL 窗口根本不会触发。
- **解决**：`is_heartbeat_stale(last_ts, now)` 单调比较，超时由板侧生成 `heartbeat_timeout` 告警 →
  `alerts/esp32_01` → `esp32_alerts_to_influx` 落库。时钟回拨（`now < last_ts`）返回 False，
  避免 NTP 校时误报全网离线。
- **一句话面试说辞**：「离线是『没有事件』，流引擎只能处理『有事件』，所以离线检测必须在设备侧或外部看门狗。」

#### 4.4 不用 `dataTemplate`，tags 用静态 map

- **根因**：`docs/排障记录-服务端与IoT.md` 第 9 条 —— eKuiper 2.1.0 求模板时上下文是
  `[]map[string]interface{}` **切片**，Go `text/template` 对切片取 `{{.deviceId}}` 会报
  `can't evaluate field deviceId`。`deviceId` 本就是常量，没必要模板化。
- **解决**：`"tags": {"device": "esp32_01"}` 静态 map；
  单测 `test_no_password_key_and_no_data_template` 显式禁止出现 `dataTemplate` 键。

#### 4.5 凭据一律占位符

- `esp32_rules.json` 里所有 `token` 都是 `${INFLUX_TOKEN}`，不写密码、不写真实值；
  单测 `test_tokens_are_placeholders_only` 逐个校验 `${...}` 格式。
- 🔴 本轮甚至没有独立核实 `iot/docker-compose.yml` 的凭据（读取被敏感保护拦截，
  按规则不重试 / 不推断 / 不绕过），草案只沿用「既有 provisioning 已注入」这一事实，不复述任何值。

### 5. 本机验证（硬证据；不是实机）

`D:\esp\_iot_verify.py` 驱动（纯 ASCII 路径绕开 Git Bash 中文参数坑），最终：

```text
[1] unittest test_esp32_telemetry -v      rc=0 OK   Ran 27 tests / OK
[2] esp32_telemetry --self-test            rc=0 OK   SELF_TEST PASS
[3] --self-test (ASCII stdout)             rc=0 OK
[4] --dry-run --once (ASCII)               rc=0 OK
[5] 回归 telemetry_bridge --self-test      rc=0 OK   （没碰坏既有桥）
[6] compileall -q iot                      rc=0 OK
VERIFY PASS=6 FAIL=0
```

#### 排障：首跑 `PASS=4 FAIL=2` 的 3 个不一致

| 现象 | 根因 | 解决 | 一句话面试说辞 |
|---|---|---|---|
| `--dry-run` 参数不存在，子进程 `rc=2` | docstring 写了用法，但 argparse 没实现 | 补 `add_argument("--dry-run")`，并让 `if args.publish and not args.dry_run` 保证 dry-run 优先 | 「文档和 CLI 要一起改，否则示例命令根本跑不通」 |
| `--values "-82,120000,7,5"` 报 `expected one argument` | 值以 `-` 开头，argparse 把它当成新选项 | 改成等号形式 `--values=-82,120000,7,5`，并在 docstring 写明这个坑 | 「负号开头的参数值必须用 `--key=value`，这是 argparse 的词法规则」 |
| 规则 JSON 单测 `DATASOURCE` 断言失败 | JSON 里 `\"` 解析后就是 `"`，测试多写了一层反斜杠 | 断言改成 `'DATASOURCE="device/esp32_01/telemetry"'` | 「JSON 字符串里的转义在解析后就不存在了，断言要按解析后的值写」 |

> 这三条都属「检查工具与被检查对象不一致」，不是产品逻辑缺陷；
> 按项目规矩只记**关键 / 面试官关心**的，鸡毛蒜皮不记。

### 6. 诚实边界（🔴 别越线）

- 🚫 禁写：「ESP32 遥测已接入」「已部署告警规则」「IoT 三节点闭环已打通」「已上板 / 已联调」。
- 🔴 **ESP32 固件侧真正的 MQTT client 尚未写**：真节点还没烧录、没连 Wi-Fi，
  本模块只覆盖「契约层 + 板侧适配层」，不是端到端已接入。
- 🔴 `esp32_rules.json` 只是静态文本：`meta.status = "draft"`，未连 EMQX / eKuiper / InfluxDB，未部署。
- 🔴 `DEMO_VALUES` 是假数据；27 项单测只证明本机逻辑，**编译 / 单测通过 ≠ 实机**。
- 🔴 档位：支柱③ 仍 ✅ 板内四容器 / 🔶 未覆盖 ESP32 节点 + 无告警规则；本草案**不改变档位**，
  也没有把「核心」抬成已完成。

### 7. 回教室落地前必做（照跑清单）

1. **先按主线走**：`docs/固件与ESP32开发.md` —— 烧 ESP32-S3、跑四层硬判据、
   ESP32 真发帧钉死语义、再进 ESP-SR。本草案排在它们之后。
2. 写 **ESP32 固件侧 MQTT client**：把 `rssi / heap_free / wake_cnt / cmd_ok` 换成真读数，
   `DEMO_VALUES` 不许留。
3. **`deploy.sh` 的 `to_influx` 加 `WHERE deviceId='kunpeng_01'`**（§4.2），再部署本草案的 2 流 3 规则。
4. `${INFLUX_TOKEN}` 由环境变量注入，不硬编码；Grafana 加 ESP32 面板与告警面板。
5. 验收判据回到路线图阶段 5 / 6：第二条设备线随时间变化；人为走远 / 拔网线，
   `alerts` 出现记录、面板变红。

---

### 附：本轮同步范围（为什么没改根 README / 简历条目 / 路线图档位）

按项目纪律，未实机的提前草案不扩散到对外口径文件：根 `README.md`、
`docs/面试与简历材料.md`、`docs/00-项目总览与硬数据速查.md`、
`docs/固件与ESP32开发.md` 的**阶段表与档位结论**均保持原样（本文件只**引用**路线图
第 58/59/71/72 行）。只做三件同步：① `iot/README.md` 补三行法；
② `.workbuddy/memory/` 留日志与指针；③ 本草案文档。
都属于「指针 + 复习提纲」，不改任何状态结论。