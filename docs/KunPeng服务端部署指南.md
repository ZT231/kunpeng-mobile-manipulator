# KunPeng Pro 服务端部署指南（xiaozhi-esp32-server）

> 目标：在 KunPeng Pro v1.1（openEuler 22.03，ARM64）上自托管 xiaozhi 兼容服务端，作为 ESP32-S3 语音交互节点的云端（小智兼容服务端）。
> 本步**不依赖任何 ESP32 硬件**，今天就能做，用来先验证整条链路。
> 参考：官方 https://github.com/xinnan-tech/xiaozhi-esp32-server ；xiaozhi.me 文档。

---

## 0. 前置确认

- KunPeng Pro 能联网（下镜像/依赖/模型需要）。之前是用手机热点或寝室 Wi-Fi，先确认能 `ping www.baidu.com`。
- 拿到一个 LLM API key（推荐 DeepSeek：https://platform.deepseek.com/ ，新用户有免费额度；或阿里百炼 Qwen）。
- TTS 用 EdgeTTS（免费、纯 Python，无需 key）。

---

## 方式一：Docker 部署（★推荐，ARM64 最稳）

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

# 4. 编辑 data/.config.yaml（见下方"关键配置"）
vi data/.config.yaml

# 5. 启动
docker compose up -d
docker logs -f xiaozhi-esp32-server
```

启动成功日志里会有：`Websocket地址是 ws://xxx:8000/xiaozhi/v1/`、`OTA接口是 .../xiaozhi/ota/`。

> 若 `docker compose pull` 报 ARM64 镜像缺失，说明该镜像未提供 arm64 版本，退回到**方式二**。

---

## 方式二：纯 Python 部署（备选，没装 docker 时用）

⚠️ **ASR 必须改云端（AliyunASR 等），不要用默认 FunASR**——FunASR 依赖 PyTorch，ARM64 裸装极易踩坑（你刚被 CANN/numpy 坑过，懂的）。

```bash
# 1. 系统依赖
sudo dnf install -y python3-pip git gcc python3-devel

# 2. 拉代码
git clone https://github.com/pluto-xm/xiaozhi-esp32-server.git
cd xiaozhi-esp32-server

# 3. 虚拟环境
python3 -m venv venv
source venv/bin/activate

# 4. 装依赖（换阿里源快）
pip config set global.index-url https://mirrors.aliyun.com/pypi/simple/
pip install -r requirements.txt

# 5. 改 config.yaml（见下方"关键配置"，ASR 用 AliyunASR + 云端）
vi config.yaml

# 6. 启动
python app.py
```

---

## 关键配置（config.yaml 片段）

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
> `server.port` 默认 8000，WebSocket 地址形如 `ws://<KunPeng-Pro-IP>:8000/xiaozhi/v1/`。

---

## 固件端对接（等 ESP32-S3 开发板到货后做）

1. 买 **ESP32-S3-DevKitC-1 N16R8**（带 PSRAM，约 100 元），先不阻塞在自研核心板打样上。
2. 烧录 xiaozhi-esp32 固件（参考 xiaozhi.me 快速制作指引），配网/编译时把服务端地址填成：
   `ws://<KunPeng-Pro-IP>:8000/xiaozhi/v1/`，token 与上面 `auth.tokens` 一致。
   - 改地址的位置：固件 `idf.py menuconfig` → xiaozhi-esp32 配置项，或 `main/boards/<你的板>/config.h` 里的 `WEBSOCKET` 宏。
3. 自研核心板打样回来后，把同样固件迁过去（改引脚定义），这一步本身就是面试素材。

---

## 验证顺序（今天就能走完第 1 步）

1. ✅ 服务端起好 → 浏览器打开 xiaozhi-esp32-server 仓库里的 `test/test_page.html`（需先从 https://github.com/xinnan-tech/xiaozhi-esp32-server 下载到本地，项目内不含此文件），填 WebSocket 地址和 token，点"拨号"能对话 = **云端链路通**（不依赖 ESP32）。
2. ✅ 开发板到货 → 烧固件填地址 → 实体语音对话 = **端云打通**。

---

## 排错

- `LLM 的 API key 未设置`：config 里 key 还是占位符，填真实 key。
- Docker 起不来 / 镜像拉不到 arm64：退回方式二，且 ASR 改 AliyunASR。
- 板子连不上服务端：确认 KunPeng Pro 防火墙放行 8000 端口（`sudo firewall-cmd --add-port=8000/tcp --permanent && sudo firewall-cmd --reload`），且 ESP32 与服务端在同一网段。
- 想本地看对话：用从 xiaozhi-esp32-server 仓库下载的 `test_page.html`（项目内不含此文件），别用浏览器直接开 ws 地址（那会 404）。
