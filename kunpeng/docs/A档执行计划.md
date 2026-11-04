# A 档执行计划（仅 A 档：0 元纯软件）

> 决策日期：2026-11-02。小Z 确认「只做 A 档」，不买激光雷达、不买深度相机、不做 B/C/D 档降级。
> A 档 = 幻尔套件 17 格里的 8 项纯软件功能：**KCF 追踪 / 颜色追踪 / 视觉追踪 / 文本聊天 / 自然对话 / 场景理解 / 语音控制 / 语音交互**。

## 一、A 档内部也有先后（按「能否立刻跑」排）

| 优先级 | 功能 | 前置条件 | 现在能做？ |
|---|---|---|---|
| P0 | 摄像头取流 + 颜色追踪 + 质心定位 | GF100 已购、KunPeng 已装 OpenCV | ✅ 立刻可做 |
| P0 | KCF 追踪（增强） | 同上 + opencv-contrib | ✅ 装包装好即可 |
| P1 | 场景理解 / 文本聊天 / 自然对话 | 云端大模型 API Key（你自己提供） | ⏳ 需 Key |
| P2 | 语音控制 / 语音交互 | ESP32-S3 核心板焊完 + INMP441 + MAX98357A | ⏳ 等硬件 |

**结论：今天先啃 P0（视觉取流+追踪），它不花一分钱、不卡任何硬件，且是后续一切视觉的基础。**

## 二、P0：视觉取流 + 追踪（已交付 `a_vision_demo.py`）

### 在 KunPeng 上跑
```bash
# 1) 装依赖（二选一）
#    方案a：系统包（推荐，省事）
sudo dnf install -y opencv python3-numpy python3-opencv
#    方案b：pip（若要 KCF 必须走这个，把 contrib 一起装上）
pip3 install opencv-contrib-python-headless numpy

# 2) 把脚本传上去
scp a_vision_demo.py root@<板子IP>:~/

# 3) 接了 HDMI 就直接跑；ssh 连就加 -X 弹窗口，或加 --headless 远程看质心
ssh -X root@<板子IP>
python3 a_vision_demo.py            # 有窗口：放个红色物体，看框+质心+轨迹
python3 a_vision_demo.py --headless # 无窗口：终端每 10 帧打印一次质心坐标
```

### 验证「跑通」的判据（照着勾）
1. `ls /dev/video*` 能看到设备（GF100 是 UVC，一般 video0）
2. 脚本打印 `[OK] 摄像头已打开`
3. 放红色物体进画面 → 出现红框 + 黄色质心点 + 黄色轨迹
4. 物体移出中心 → 终端/画面提示「目标在左/右/上/下」（这是给机器人用的决策信号）

### 进阶（P0 内可继续加，仍 0 元）
- 换追踪颜色：改 `LOWER_RED*/UPPER_RED*` 的 HSV 范围即可（蓝/绿同理）
- 多目标：把 `max(contours...)` 改成遍历所有轮廓
- 接 ROS2：用 `cv_bridge` 把帧发到 `sensor_msgs/Image` 话题（openEuler 有官方 ros-humble-cv-bridge）
- 接 IoT：把质心坐标经 MQTT 发到 EMQX（复用 `iot/` 那条链路）→ Grafana 看轨迹

## 三、P1：场景理解 / 对话（需云端大模型 Key）

A 档里的「文本聊天 / 自然对话 / 场景理解」本质是：**视觉给出画面描述 → 发给云端大模型 → 大模型返回指令/回答**。

落地方式（等你给 Key 我再写具体脚本）：
- 用 `requests` 调云端大模型 HTTP API（OpenAI 兼容 / 通义千问 / 文心 等任选）
- Key 走环境变量 `export LLM_API_KEY=xxx`，**绝不写进代码**
- 输入：摄像头帧的描述文本（或编码后的图片）→ 输出：结构化指令（如 `{"action":"move_left","dist":10}`）
- 架构话术：「端侧只做唤醒+目标检测，重模型推理放云端，边界清晰、不卡板子算力」

## 四、P2：语音（等 ESP32-S3 焊完）

- ESP32-S3 核心板焊好 → 跑 ESP-IDF + WakeNet（唤醒）+ I2S（INMP441 采集）/ MAX98357A（播放）
- 语音走 WiFi → KunPeng（WebSocket/MQTT）→ 云端大模型 → 回传指令
- 这是你「端-边-云」架构里「端」的那一环，和 P0/P1 在 KunPeng 侧汇合

## 五、面试怎么讲 A 档（差异化，别踩雷）

- ✅ 说清：「视觉追踪用 OpenCV 做颜色+质心定位，KCF 做相关滤波增强；对话/场景理解走云端大模型，端侧只做轻量前处理」
- ✅ 强调：「这 8 项全是 0 元纯软件，基于开源框架，我能讲清每一层边界」
- ❌ 别写「自研 SLAM / 自研大模型」——这些是 A 档没有的，一追问就露馅
- ❌ 别把 A 档说成「完整移动机器人」——它只是视觉+交互层，导航/建图在 B 档（未做）

## 六、诚实边界
- 我在 Windows 上**没法连你的 KunPeng 实跑** `a_vision_demo.py`，代码基于 OpenCV 标准 API 写、逻辑已走查，但首次上板请按上面 4 条判据验证。
- KCF 的 `cv2.legacy` 写法对 opencv-contrib-python 版本敏感，若 import 失败脚本会自动降级为纯颜色追踪（已处理）。
- 大模型 Key、ESP32-S3 硬件这两块的前置，需要你提供/到货后我再补对应脚本。
