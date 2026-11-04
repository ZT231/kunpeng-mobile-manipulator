# KunPeng Pro · IoT 栈最小闭环部署包

把"设备 → Broker → 边缘规则引擎 → 时序库 → 看板"这条 IoT 链路在 KunPeng Pro（openEuler 22.03 / aarch64）上跑通。
这是简历项目里 **IoT 端-边-云重度融合** 的最小骨架，先立起来、不卡任何硬件。

## 链路
```
模拟设备(python) --MQTT 1883--> EMQX --eKuiper 规则引擎--> InfluxDB 2.x --Grafana 看板
        device/+/telemetry         (边缘计算/筛选)         (时序存储)      (可视化/可观测)
```
- **EMQX**：MQTT Broker（边网关，设备/客户端接入）。
- **eKuiper**：边缘规则引擎（在 KunPeng 本地做字段筛选/计算，断网也能自治；后续可加阈值告警）。
- **InfluxDB 2.x**：时序数据库（存遥测）。
- **Grafana**：看板（自动注入 InfluxDB 数据源）。

> 所有镜像均为官方 multi-arch，在鲲鹏 920 (arm64) 上自动拉取 aarch64 版本，无需手动指定架构。

---

## 0. 前置：在 openEuler 22.03 上装 Docker

KunPeng 出厂系统可能没有 Docker，或只有旧版 docker-engine（会和 docker-ce 冲突）。

```bash
# 1. 卸载自带的旧 docker-engine
sudo dnf remove -y docker docker-client docker-client-latest docker-common \
  docker-latest docker-latest-logrotate docker-logrotate docker-engine

# 2. 添加 Docker CE 软件源（华为云镜像；CentOS 8 路径对 aarch64 同样适用）
sudo tee /etc/yum.repos.d/docker-ce.repo <<'EOF'
[docker-ce-stable]
name=Docker CE Stable - $basearch
baseurl=https://repo.huaweicloud.com/docker-ce/linux/centos/8/$basearch/stable
enabled=1
gpgcheck=1
gpgkey=https://repo.huaweicloud.com/docker-ce/linux/centos/gpg
EOF

# 若第 3 步报 GPG 校验失败，把上面 gpgcheck=1 改成 gpgcheck=0 重试即可

# 3. 安装（含 docker compose V2 插件）
sudo dnf clean all && sudo dnf makecache
sudo dnf install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin

# 4. 启动 + 开机自启
sudo systemctl start docker
sudo systemctl enable docker

# 5. 配置国内镜像加速（拉取快很多）
sudo mkdir -p /etc/docker
sudo tee /etc/docker/daemon.json <<'EOF'
{
  "registry-mirrors": ["https://docker.mirrors.ustc.edu.cn", "https://hub-mirror.c.163.com"]
}
EOF
sudo systemctl daemon-reload && sudo systemctl restart docker

# 6. 验证
docker -v
sudo docker run --rm hello-world     # 看到 Hello from Docker! 即成功
```

---

## 1. 部署

把本目录整个 `scp` 到 KunPeng（例如 `~/iot`），然后：

```bash
cd ~/iot
chmod +x deploy.sh
./deploy.sh
```

> **部署前预检（强烈建议，30 秒）**：先单独验证四个镜像都能拉到，尤其是 eKuiper：
> ```bash
> docker pull emqx:5.8.8
> docker pull lfedge/ekuiper:2.1.0-full
> docker pull influxdb:2.7
> docker pull grafana/grafana:11.3.0
> ```
> 若 `lfedge/ekuiper:2.1.0-full` 报 `manifest unknown`，说明该版本未发布 `-full` 变体——
> 去 Docker Hub 把 `docker-compose.yml` 里这一行的 tag 改成最新的 `-full`（如 `2.2.0-full` 等）即可，
> 其余配置（流、规则、token）**完全不用动**。其余三个镜像已核实均为有效 tag。

脚本会：① `docker compose up -d` 起 4 个容器；② 等 eKuiper 就绪；③ 通过 REST API 建流 `device_stream`（订阅 `device/+/telemetry`）和规则 `to_influx`（选字段写入 InfluxDB）。

## 2. 模拟设备（产生数据）

```bash
pip3 install paho-mqtt        # 只需一次
python3 simulate_device.py    # 每 2 秒上报一条温/湿/电压遥测
```

上报主题 `device/esp32_01/telemetry`，正好匹配 eKuiper 流的 `device/+/telemetry`。
将来真实 ESP32-S3 / STM32 上云时，只要把数据发到 `device/<id>/telemetry` 这个主题格式即可，无需改后端。

## 3. 看效果

- **Grafana 看板**：浏览器开 `http://<KunPeng的IP>:3000`，登录 `admin / robotics123`。
  数据源 **InfluxDB** 已自动注入。新建 Dashboard → Add panel，用下面三条 Flux 分别建三个面板：
  ```
  from(bucket: "telemetry")
    |> range(start: -5m)
    |> filter(fn: (r) => r._measurement == "sensor" and r._field == "temperature")
  ```
  `humidity`、`voltage` 把最后一行 `_field == "temperature"` 换成对应字段即可。
  有线出来了 = 闭环通了。
- **EMQX 控制台**：开 `http://<IP>:18083`，登录 `admin / robotics123`，看 Connections / Topics 里的消息数。

> 怎么拿到 KunPeng 的 IP：在 KunPeng 上 `hostname -I`。电脑需与 KunPeng 同一网段（连同一个手机热点或局域网）；
> 若浏览器打不开，先在 KunPeng 本机 `curl -s localhost:3000` 确认服务在跑，多半是电脑与板子不在同一网段或防火墙。

---

## 4. 排障

1. **eKuiper 规则报错 / 提示 influx2 插件未找到**
   说明当前 `lfedge/ekuiper:2.1.0-full` 镜像未含 influx2 预编译插件（个别版本会这样）。
   两种解法：
   - 换镜像 tag：去 Docker Hub 查 `lfedge/ekuiper` 带 `-full` 的最新版本号替换 `docker-compose.yml` 里的 tag，重跑 `./deploy.sh`。
   - 或走**降级桥（附录 A）**：让 eKuiper 把结果用 MQTT sink 发回 `processed/<id>`，由 Telegraf 落库，完全不依赖 eKuiper 的 influx 插件。
2. **Grafana 面板没数据**
   - 先确认 `python3 simulate_device.py` 在跑（终端有 `[sim] -> {...}` 输出）。
   - 查 eKuiper 规则是否在跑：`curl -s localhost:9081/rules/to_influx`，看返回里 `status` 是否为 running、有无 `messages` 计数。
   - 查 InfluxDB 是否收到：Grafana 左侧 Explore → 选 InfluxDB → 跑上面的 Flux，看有无点。
   - 确认订阅主题匹配：模拟设备发 `device/esp32_01/telemetry`，流是 `device/+/telemetry`，层级一致才匹配。
3. **镜像拉不下来**
   已配国内加速；若仍慢，手动 `docker pull <镜像>` 逐个拉。ekuiper 若 `2.1.0-full` 不存在，换成 Docker Hub 上该仓库最新的 `-full` tag。
4. **端口从电脑访问不了**
   KunPeng 与电脑须同网段；可临时关防火墙 `sudo systemctl stop firewalld` 排查，确认后再放行 1883/18083/3000/9081/8086。

---

## 5. 下一步（往正式项目演进）

- 真实设备：ESP32-S3 / 双 STM32 用串口/CAN 经 USB 适配器接 KunPeng，由协议桥把数据封装成 `device/<id>/telemetry` 上报（替换模拟脚本）。
- 边缘智能：在 eKuiper 规则里加 `WHERE temperature > 阈值` 触发告警 / 本地急停（断网自治）。
- 设备影子：EMQX 开 `retained` 消息 + 自研轻量影子服务，或用阿里云 IoT 现成影子。
- OTA：ESP32 `esp_https_ota` 先落地，STM32 串口 IAP 双 bank 作为深度项。

---

## 附录 A：Telegraf 降级桥（当 eKuiper influx2 插件不可用时）

A1. 把 `deploy.sh` 里建规则那段替换成（用 mqtt sink 发回 EMQX）：
```json
{
  "id": "to_mqtt",
  "sql": "SELECT temperature, humidity, voltage, deviceId FROM device_stream",
  "actions": [
    { "log": {} },
    { "mqtt": { "server": "tcp://emqx:1883", "topic": "processed/{{.deviceId}}", "qos": 1 } }
  ]
}
```

A2. 在 `docker-compose.yml` 末尾加 Telegraf 服务，并新建 `telegraf/telegraf.conf`：
```yaml
  telegraf:
    image: telegraf:1.31
    container_name: telegraf
    restart: unless-stopped
    depends_on: [emqx, influxdb]
    networks: [iot]
    volumes:
      - ./telegraf/telegraf.conf:/etc/telegraf/telegraf.conf:ro
```
```toml
# telegraf/telegraf.conf
[[inputs.mqtt_consumer]]
  servers = ["tcp://emqx:1883"]
  topics = ["processed/+"]
  data_format = "json"

[[outputs.influxdb_v2]]
  urls = ["http://influxdb:8086"]
  token = "robotics_admin_token_2026"
  organization = "robotics"
  bucket = "telemetry"
```
重跑 `./deploy.sh` + `docker compose up -d` 即可，Grafana 看板无需改动。
