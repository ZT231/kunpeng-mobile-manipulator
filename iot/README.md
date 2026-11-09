# KunPeng Pro · IoT 栈最小闭环部署包

把"设备 → Broker → 边缘规则引擎 → 时序库 → 看板"这条 IoT 链路在 KunPeng Pro（openEuler 22.03 / aarch64）上跑通。
这是简历项目里 IoT 端-边-云链路的**最小骨架**；本包只落地端-边侧的板内闭环，先立起来、不卡任何硬件。

## 链路
```
真实遥测桥 --MQTT 1883--> EMQX --eKuiper 规则引擎--> InfluxDB 2.x --Grafana 看板
  device/kunpeng_01/telemetry   (边缘计算/筛选)      (时序存储)     (可视化/可观测)
```
- **EMQX**：MQTT Broker（边网关，设备/客户端接入）。
- **eKuiper**：边缘规则引擎（在 KunPeng 本地做字段筛选/计算，断网也能自治；后续可加阈值告警）。
- **InfluxDB 2.x**：时序数据库（存遥测）。
- **Grafana**：看板（自动注入 InfluxDB 数据源）。

> 🔴 **架构现状（2026-11-09 更新）**：阶段 1.1 实证「真实数据 → MQTT 发布 → 订阅收到」；
> 阶段 1.2b 已把 eKuiper / InfluxDB / Grafana 补上并换成真实字段，
> **15:13 端到端实证：真实串口遥测 → MQTT → EMQX → eKuiper → InfluxDB → Grafana 六路曲线**；
> 15:32 复核 15s 内 `records_in` +3 而 `exceptions_total` 不涨，是**稳态写入**不是瞬时侥幸。
> ⚠️ 这仍是**板内可观测性闭环**（曲线是臂静止在初始位的六条平线），
> **不是云端 IoT、不是自动抓取、不是闭环控制**；1.2 只增可观测性、不增抓取能力。
> 八连修排障链与四次错误假设证伪见 `docs/排障记录-IoT.md` 第 5~11 条。

### 概念（大白话 + 规范定义）
- **MQTT**：设备把消息贴到「主题」上，订阅这个主题的人就收到；规范定义是 MQTT 3.1.1 / 5.0，
  基于 TCP 的发布-订阅轻量消息协议，主题用 `/` 分层、`+` 为单层通配。
- **边缘规则引擎（eKuiper）**：在板子本地用一句 SQL 把消息筛一遍再转发，不用上云；
  规范定义是面向无界流的连续查询引擎，用类 SQL 对数据流做投影 / 过滤 / 聚合。
- **时序数据库（InfluxDB 2.x）**：专门存「带时间戳的一串数」；
  规范定义是按 measurement + field + tag + timestamp 组织的时间序列存储，
  **field value 只允许 float / integer / unsignedInteger / string / boolean 五种标量，没有数组类型**；
  且每个 `measurement + field` 的类型**首次写入即锁定**（第 11 条排障的根因）。
- **Grafana**：把这些数画成曲线的网页；规范定义是可观测性可视化前端，靠数据源插件发查询。

> 🔴 **两个板上实测约束（别删）**
> ① `docker-compose.yml` 必须写 `version: "2.4"`：本机是 **V1 的 `docker-compose 1.22.0`**，
> 没有 version 时会按 v1 文件格式解析，连 `services:` 都不认；而 2.4 是同时支持
> `healthcheck` 和 `depends_on: condition: service_healthy` 的最后一版（v3 移除过 condition）。
> ② 四个服务都要 `security_opt: [seccomp=unconfined]`：本机 **Docker 19.03.14** 的默认
> seccomp profile 不含 `clone3`，新镜像里 glibc >= 2.34 的 `pthread_create()` 改用
> `clone3(2)` 会被 `EPERM` 拒，容器起来就 `Exited (1)`（阶段 1.1 的 EMQX 首行日志
> `Failed to create thread: Operation not permitted (1)` 已实证）。
> **这是本地实验的安全取舍**（等于撤掉该容器的系统调用过滤层）；正解是升 Docker >= 20.10.10
> 或自定义 profile 只放行 `clone3`，但官方源只有 19.03.14、引第三方源有依赖冲突。

---

## 0. 前置：在 openEuler 上装 Docker

🔴 **本板实测（11-08）用的是官方源，别照抄通用文档里「卸旧包 + 加 CentOS 8 docker-ce 源」那套**：
`dnf list available 'docker*'` 直接有 `docker 19.03.14` + `docker-compose 1.22.0`，
`rpm -qa | grep -i docker` 为空（干净系统、无旧包冲突）。`docker compose`（V2 插件）**不存在**，
只有带横线的 `docker-compose`（V1）。

**【板 · openEuler 终端】**
```bash
sudo dnf install -y docker docker-compose
sudo systemctl start docker && sudo systemctl enable docker
docker -v && docker-compose version
```

**Docker Hub 直连超时**（实测 `Get https://registry-1.docker.io/v2/: ... Client.Timeout exceeded`）
要配镜像加速。旧文档里的 USTC / 网易源已停同步，**别再用**；当前可用候选：
`https://docker.m.daocloud.io`、`https://docker.1ms.run`、`https://docker.xuanyuan.me`。
先用 `curl -s -o /dev/null -w "%{http_code}" https://docker.m.daocloud.io/v2/` 探活
（**401 / 200 都算通，000 或超时算不通**），再写配置并重启：

**【板 · openEuler 终端】**
```bash
sudo mkdir -p /etc/docker && printf '%s\n' '{"registry-mirrors":["https://docker.m.daocloud.io","https://docker.1ms.run","https://docker.xuanyuan.me"]}' | sudo tee /etc/docker/daemon.json && sudo systemctl daemon-reload && sudo systemctl restart docker
```

> ⚠️ `docker run -d` 回显容器 ID **不等于**容器活着，必须 `docker ps -a` 复核 STATUS 为 `Up`。

---

## 1. 部署

把本目录整个 `scp` 到板子 `~/iot`（板上已有，只需覆盖改动过的文件）。

**【板 · openEuler 终端】** 先验 yml 能否被 V1 工具解析（零副作用）：
```bash
cd ~/iot && docker-compose config
```

🔴 **必做一次的前置**：阶段 1.1 是用单条 `docker run` 手动起的 `emqx`，
`container_name: emqx` 与 compose 的定义冲突，先删掉它，再由 compose 统一托管四个容器：

**【板 · openEuler 终端】**
```bash
docker rm -f emqx
```

> 删掉后已连上的 `telemetry_bridge.py` 发布端会断一次连接，重跑发布端即可（1.1 实证已完成，无损失）。
> 以后重跑本脚本时 emqx 已归 compose 管，**别再删**。

🔴 **scp 之后、部署之前必做一次**：Grafana 官方镜像以非 root 用户运行，bind mount 会保留
宿主机 `root:root 600` 权限，容器内读不到 provisioning 文件 ⇒ 数据源一个都注册不上（第 10 条排障）。

**【板 · openEuler 终端】**
```bash
chmod -R a+rX ~/iot/grafana/provisioning/
```

**【板 · openEuler 终端】**
```bash
cd ~/iot && chmod +x deploy.sh && ./deploy.sh
```

脚本会：① `docker-compose up -d` 起 4 个容器；② **动态把当前 docker 网桥纳入 firewalld
trusted zone**（openEuler nftables 兜底 REJECT 会让容器间「no route to host」，第 8 条排障；
runtime 绑定重启会丢，所以每次部署都重绑一次，幂等可重跑）；③ 等 eKuiper 就绪；
④ 先删规则再删流，通过 REST API 建流 `device_stream`（订阅 `device/+/telemetry`）和规则
`to_influx`（**真实字段**写入 InfluxDB）；⑤ 查规则运行态 `/rules/to_influx/status`。

> **部署前预检（30 秒，强烈建议）** —— **【板 · openEuler 终端】**：先单独验证四个 arm64 镜像能拉到：
> ```bash
> docker pull emqx:5.8.8
> docker pull lfedge/ekuiper:2.1.0-full
> docker pull influxdb:2.7
> docker pull grafana/grafana:11.3.0
> ```
> ✅ `lfedge/ekuiper:2.1.0-full` 的 **aarch64 manifest 已于 11-09 板上实证存在**（5 层 Pull complete），
> 早期「可能 manifest unknown」的预警已撤回。若仍报 `manifest unknown`，
> 去 Docker Hub 查该仓库带 `-full` 的最新 tag 替换 `docker-compose.yml` 里的镜像行，
> 其余配置（流、规则、token）**完全不用动**。

## 2. 真实遥测桥（产生数据）

🔴 已用 `telemetry_bridge.py` **顶替** `simulate_device.py` 的假温湿电压；
`simulate_device.py` 仅留作旧脚手架对照，不要再跑。

**【板 · openEuler 终端】**
```bash
cd ~/iot && python3 telemetry_bridge.py --self-test
cd ~/iot && python3 telemetry_bridge.py --dry-run --once
cd ~/iot && python3 telemetry_bridge.py --broker 127.0.0.1 --port 1883 --topic device/kunpeng_01/telemetry
```

上报主题 `device/kunpeng_01/telemetry`，匹配 eKuiper 流的 `device/+/telemetry`。

**真实 payload 字段**（`build_payload()` 产出）：

| 字段 | 含义 | 类型 | InfluxDB 里 |
|---|---|---|---|
| `deviceId` | 设备 ID `kunpeng_01` | string | tag `device` |
| `ts` | Unix 秒 | int | 不选（避开 eKuiper 事件时间字段） |
| `pulses` | 六路脉宽原数组 | array | **不写**（field 不接受数组，留给人看 / 订阅端） |
| `j1`~`j6` | 六路脉宽拆成的标量 | int | field，画六路曲线 |
| `frame_id` | 8080 推流帧号 | int | field |
| `det_count` | YOLO 检测数 | int | field |
| `mode` / `baud` | `serial` / `115200` | string / int | field |
| `camera_on` | 是否启本地摄像头 | bool | field |

> 🔴 **为什么多出 `j1~j6`**：阶段 1.2 为了落 InfluxDB **纯新增**的六个标量，
> 原来的 `pulses` 数组**保留不动** —— 阶段 1.1 的发布/订阅截图与订阅端解析都还成立。
> 不这么写，直接把数组塞进 `fields`，要么写入报错、要么被转成字符串，看板画不出六路曲线。
>
> 🔴 **六路不完整时整条跳过，不发 MQTT**（1.2b 收口）：串口断连时 `/api/state` 回「未连接」，
> `build_payload()` 会把 `j1~j6` 填成 `null`；这种点发出去会被 influx2 sink 写成 **string**，
> 撞上历史 float 触发 InfluxDB `field type conflict`、整批写不出去（第 11 条排障）。
> 所以 `is_publishable()` 判定六路必须全是真数字（显式排除 `bool`），否则打印
> `[warn] 六路脉宽不完整，跳过本次发布` 并跳过本次 publish。`--dry-run` 仍会打印出来供诊断。

## 3. 看效果

- **Grafana 看板**：**【PC · 浏览器】** 打开板子 `wlan0` 的 IP 加 `:3000`（板上 `hostname -I` 查），
  登录 `admin / robotics123`。数据源 **InfluxDB** 已自动注入
  （`grafana/provisioning/datasources/influx.yml`：Flux / org `robotics` / bucket `telemetry` /
  token 与 InfluxDB 初始化 env 一致）。新建 Dashboard → Add panel，用下面 Flux 建面板：
  ```
  from(bucket: "telemetry")
    |> range(start: v.timeRangeStart, stop: v.timeRangeStop)
    |> filter(fn: (r) => r._measurement == "arm_telemetry" and (r._field == "j1" or r._field == "j2" or r._field == "j3" or r._field == "j4" or r._field == "j5" or r._field == "j6"))
  ```
  🔴 **必须写 `range(start: v.timeRangeStart, stop: v.timeRangeStop)`，别硬编码 `-5m`**：
  Flux 的 `range()` 决定从 InfluxDB 取回哪些点，而 Grafana 右上角的时间选择器只决定图的显示窗口；
  写死 `-5m` 时就算 UI 调到 `Last 24 hours` 也取不到更早的点（14:21 实证：11:45 的历史点
  用 `-5m` 查是空 body，换 `-24h` 立刻返回真实 CSV）。用 `v.timeRangeStart/Stop` 让取数窗口
  自动跟随 UI，才是根本写法。
  六路脉宽曲线出来了 = 闭环通了。把 `_field` 换成 `frame_id` / `det_count` 可看帧率与检测数。
  `measurement` 由旧假数据时代的 `sensor` 改成了 `arm_telemetry`，**别再用旧查询**。
- **EMQX 控制台**：**【PC · 浏览器】** 开板子 IP 的 `:18083`，登录 `admin / robotics123`，看 Connections / Topics 消息数。
- **验 eKuiper 规则**：**【板 · openEuler 终端】** `curl -s localhost:9081/rules/to_influx/status`，
  看 `status` 是否 `running`，以及 `sink_influx2_1_0_records_in_total` 涨、`exceptions_total` 不涨
  （两个都是**累计值**，判断当前是否恢复要看**增量**）。
  🔴 `GET /rules/to_influx` 只是 **describe**（回显规则定义、**不含 status**），别拿它判运行态。

> 电脑需与板子同一网段（连同一个热点/局域网）；板子 wlan0 是 DHCP，**IP 每次重连要重查**。
> 若浏览器打不开，先在板子本机 `curl -s localhost:3000` 确认服务在跑。
> 🔴 **PC 打不开 `:3000`**：firewalld `public` zone 默认只放行 5000 / 8080，需先在板上放行：
> **【板 · openEuler 终端】** `firewall-cmd --zone=public --add-port=3000/tcp`。

---

## 4. 排障

> 完整五段式（现象 → 排查 → 根因 → 解决 → 面试说辞）见 `docs/排障记录-IoT.md` 第 5~11 条，这里只放速查。

1. **容器 `Exited (1)`，日志首行 `Failed to create thread: Operation not permitted (1)`**
   = Docker 19.03 seccomp 不含 `clone3` 与 glibc >= 2.34 冲突（EMQX 阶段 1.1 实证）。
   每个服务都要 `security_opt: [seccomp=unconfined]`；已写进 `docker-compose.yml`。
   `docker run -d` 返回 ID 不等于成功，必须 `docker ps -a` 复核 `Up`。
2. **`docker-compose` 报 `services` / `Unsupported config option`**
   = yml 缺 `version:`，V1 按 v1 格式解析。补 `version: "2.4"`（已补）。
3. **eKuiper 规则报错 / 提示 influx2 插件未找到**
   说明 `lfedge/ekuiper:2.1.0-full` 镜像未含 influx2 预编译插件。
   换 tag（见第 1 节预检），或走**降级桥（附录 A）**。
   🔴 建流 SQL 必须写 `CREATE STREAM device_stream () WITH (...)`（schemaless **空括号**），
   缺 `()` 报 `found "WITH", expected lparen after stream name`；建流 / 建规则的 REST 失败是
   **HTTP 200 + 带 `"error"` 的 JSON**（curl 退出码仍是 0、`set -e` 抓不到），
   所以 `deploy.sh` 里必须查响应体 `grep -q '"error"'` 并 `exit 1`，否则会一路打印「完成!」却什么都没建成。
4. **Grafana 面板没数据 / `No data`**（按层排查，别只调 UI 时间）
   - 先确认 `telemetry_bridge.py` 在跑（终端有 `[bridge] -> {...}`；六路不完整时会打印
     `[warn] 六路脉宽不完整，跳过本次发布`，此时不发 MQTT，属**预期**而非故障）。
   - 查规则运行态：`curl -s localhost:9081/rules/to_influx/status`，看 `status` running、
     `sink_influx2_1_0_records_in_total` 涨而 `exceptions_total` 不涨；`stopped by error` 先看 `last_exception`。
   - Flux 用 `v.timeRangeStart/Stop`，UI 时间范围调到覆盖数据时刻，点 `Run query`。
   - 查 InfluxDB：Grafana Explore → 选 InfluxDB → 跑上面的 Flux，看有无点；
     数据源下拉没有 `InfluxDB` ⇒ provisioning 没加载（第 10 条）。
   - 确认主题匹配：`device/kunpeng_01/telemetry` vs 流 `device/+/telemetry`；
     `measurement` 是 `arm_telemetry` 不是旧的 `sensor`。
   - 查 `nft list ruleset | grep -iE 'reject|icmpx'` 排除容器间网络被 REJECT（第 8 条）。
5. **镜像拉不下来**
   见第 0 节镜像加速；`curl` 探 `/v2/` 401/200 算通、000 算不通。
6. **端口从电脑访问不了**
   板子与电脑须同网段；firewalld 放行 1883 / 18083 / 9081 / 8087 / 3000 / 5000 / 8080
   （InfluxDB 宿主机端口是 **8087** 不是 8086，见第 7 条）。
7. **`Creating influxdb ... error`：`listen tcp 0.0.0.0:8086: bind: address already in use`**
   = 板上预装的 DevKit 工具链自带 nginx 已绑 `127.0.0.1:8086`。Docker userland proxy 默认绑
   `0.0.0.0:8086`，Linux 下通配地址与具体地址同端口互斥（`bind(2)` 返回 `EADDRINUSE`）。
   查法：`ss -lntp | grep 8086`、`docker ps -a`、`ps -ef | grep -iE "influx|docker-proxy"`。
   **别停那个 nginx**（`/opt/DevKit/tools/nginx`，用户 devkit，无 nginx.service、非 systemd 托管），
   也别 `systemctl restart docker`（没有 8086 的 proxy 残留可清）。
   已把 `docker-compose.yml` 的宿主机映射改成 `8087:8086`；容器内 `http://influxdb:8086` 三处
   与 `deploy.sh` 全不动（走 iot bridge 网络）。PC 访问 InfluxDB UI 用板 IP 的 `:8087`。
8. **规则 `created successfully` 却 `stopped by error`：`no route to host` / `Host is unreachable`**
   = firewalld 走 **nftables 后端**，`inet` 表独立于 `iptables -S FORWARD` 的 compat 视图；
   docker 网桥 `br-*` 未纳管任何 zone，容器发出的三层包落到 `target: default` 兜底
   `reject with icmpx admin-prohibited`，被内核转成 `EHOSTUNREACH`。
   🔴 四个错假设都已证伪：hosts 静态 IP 陈旧 / eKuiper veth 专属 / iptables FORWARD 拦截 /
   改 sink 网关绕行 —— `iptables -S FORWARD` 全放行却仍被拒时，用 `nft list ruleset` 才看得到真凶。
   修法：把当前 docker 网桥加入 `trusted` zone（**runtime、不加 `--permanent`、不 `--reload`**，
   避免冲掉 Docker 自建 iptables 链）。`deploy.sh` 已在每次 `up -d` 后**动态取当前网桥名重绑**，
   不用手敲接口名；15:03 实证 runtime 绑定重启会丢，15:32 的 `ZONE_ALREADY_SET` 只是幂等空操作。
9. **sink `op_influx2_1_0_transform_0_exceptions_total` 全涨、`sink records_in: 0`**
   = `tags` 用了 `{{.deviceId}}` dataTemplate。eKuiper 2.1.0 求模板时喂的是
   `[]map[string]interface{}` **切片**，Go `text/template` 对切片取 `.deviceId` 失败
   （`can't evaluate field deviceId`）。`deviceId` 本就是常量 ⇒ `tags` 改**静态**
   `{"device": "kunpeng_01"}`（已写进 `deploy.sh`）。顺带：`tags` 必须是 **map 对象**，
   写成 JSON 字符串会报 `'tags' expected a map, got 'string'`。
10. **Grafana 数据源下拉没有 `InfluxDB` / `/api/datasources` 返回 `[]`**
    = provisioning 没加载。官方镜像非 root（`uid=472(grafana) gid=0(root)`），
    bind mount 保留宿主机 `root:root 600`，容器内 `Permission denied`；
    Linux DAC 匹配「owner→group→other、命中即停」，group 位 `---` 时 other 的 `rwx` 也不生效。
    修法：`chmod -R a+rX ~/iot/grafana/provisioning/` + `docker restart grafana`
    （provisioning **只在容器启动时扫描**，改完不重启不热加载）。
11. **`records_in` 涨、`exceptions_total` 同步涨、`records_out: 0`、
    `last_exception` 报 `partial write: field type conflict ... already exists as type float`**
    = InfluxDB 2.x 每个 `measurement+field` 类型**首次写入即锁定**。串口断连时
    `/api/state` 回「未连接」，bridge 把 `j1~j6` 填成 `null`，eKuiper influx2 sink 不跳过 nil，
    把它序列化成 **string**，撞上 11:45 历史点 `j1=1580` 锁定的 **float** ⇒ 整批 drop。
    🔴 **进程不崩不等于数据无害**：治本在**发布端** —— `telemetry_bridge.py` 的
    `is_publishable()` 判定六路必须全是真数字（显式排除 `bool`），不完整就**跳过本次发布**。
    临时恢复串口：`curl -s -X POST http://127.0.0.1:5000/api/open -H 'Content-Type: application/json' -d '{"port":"/dev/ttyUSB0"}'`。
    **不推荐清库 / 换 measurement 绕开**（会丢已实证数据）。

---

## 5. 下一步（往正式项目演进）

- 真实设备：ESP32-S3 / 双 STM32 用串口/CAN 经 USB 适配器接 KunPeng，由协议桥把数据封装成 `device/<id>/telemetry` 上报。
- 边缘智能：在 eKuiper 规则里加 `WHERE` 阈值触发告警 / 本地急停（断网自治）。
- 设备影子 / OTA / 断网补传：阶段 7 深度项，当前未实现。

---

## 附录 A：Telegraf 降级桥（当 eKuiper influx2 插件不可用时）

A1. 把 `deploy.sh` 里建规则那段替换成（用 mqtt sink 发回 EMQX）：
```json
{
  "id": "to_mqtt",
  "sql": "SELECT j1, j2, j3, j4, j5, j6, frame_id, det_count, mode, baud, camera_on, deviceId FROM device_stream",
  "actions": [
    { "log": {} },
    { "mqtt": { "server": "tcp://emqx:1883", "topic": "processed/{{.deviceId}}", "qos": 1 } }
  ]
}
```

A2. 在 `docker-compose.yml` 末尾加 Telegraf 服务（**同样要带 `security_opt`**），并新建 `telegraf/telegraf.conf`：
```yaml
  telegraf:
    image: telegraf:1.31
    container_name: telegraf
    restart: unless-stopped
    security_opt:
      - seccomp=unconfined
    depends_on:
      - emqx
      - influxdb
    networks: [iot]
    volumes:
      - ./telegraf/telegraf.conf:/etc/telegraf/telegraf.conf:ro
```
```toml
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
重跑 `./deploy.sh` 即可，Grafana 看板无需改动。

---

## 附：`telemetry_bridge.py` · README 三行法（面试复习用）

> 阶段 1 新增的「真实遥测桥」。大白话：它把 5000 网页服务里**真实**的六路脉宽、
> 识别帧号、检测数搬到 MQTT 上，顶替 `simulate_device.py` 造的假温湿电压。

- **① 用了什么语法 / 技术栈**：`argparse` 管命令行参数、`urllib.request` 发 HTTP（GET / POST）、
  `json` 解析与序列化、纯函数 `build_payload()` 与 `collect_once()` 分层、`try/except` 分层容错、
  `paho.mqtt` **延迟导入**并兼容 2.x / 1.x 两种回调 API、`if __name__ == "__main__"` 入口。
- **② 为什么这么设计**：拆成独立脚本，只「读」5000 已有的 `/api/config`、`/api/detections`、
  `/api/state` 三个接口，**一行 `web_control.py` 都不改** —— 改主链路就要 scp + 重启 Flask，
  重启会把 `_IK_MAP` 标定值清回默认、打断实机现场；独立脚本 Ctrl+C 就回滚干净。
  paho 延迟导入是为了让 `--self-test` / `--dry-run` 在没装包的机器上也能跑（零装包先验）。
  `mode` 从 `/api/config` **透传**而非硬编码（这块板实测是 `"serial"`，早期草稿写过 `"uart"`）。
- **③ 不这么写会怎样**：① 若把三个接口的错误直接抛出去，5000 一抖动桥进程就死、遥测断流；
  ② 若在顶层 `import paho`，本地自测和 dry-run 会因 `ModuleNotFoundError` 直接崩，就没法零装包先验；
  ③ 若把 `/api/state` 按 1 秒轮询，会往串口多发一倍 STATE 帧、和页面轮询抢串口 —— 所以默认 5 秒，
  并在注释里写明「这是全脚本唯一有副作用的地方」；④ 若 `pulses` 不是 6 路也照发，会把错数据
  写进 InfluxDB，看板看起来有曲线其实是脏数据（1.2b 收口后已由 `is_publishable()` 挡掉）。

## 附：阶段 1.2 字段扩展 · README 三行法

> 大白话：给每条遥测再多带 6 个单独的数字，好让时序数据库能存、看板能画六条线。

- **① 用了什么语法 / 技术栈**：`dict` 字面量集中声明字段、`enumerate()` 遍历六路脉宽、
  字符串格式化 `"j%d" % (i + 1)` 生成键名、纯函数 `build_payload()` 内同步扩展、
  `assert all(...)` 在 `--self-test` 里对降级 / 畸形两类边界一起验。
- **② 为什么这么设计**：InfluxDB 2.x 的 field 只收标量，`pulses` 是数组写不进去；
  不赌 eKuiper 的数组下标语法，直接在**发布端**拆好，规则端只做 `SELECT j1..j6` 的确定性投影。
  而且**纯新增、不删 `pulses`** —— 阶段 1.1 已实证的发布/订阅 JSON、订阅端解析、两端截图全部继续成立，
  改动是可回滚的增量而不是契约破坏。
- **③ 不这么写会怎样**：① 直接把 `pulses` 列进 eKuiper `fields`，要么写入报类型错、要么被转成
  字符串，看板画不出六路曲线；② 若只在正常分支拆、降级分支忘了把 `j1~j6` 置 `None`，
  串口没连时会把脏值带进规则；③ 若手写 6 行赋值，以后改路数要改 6 处、还容易把 `j3`/`j4` 写串 ——
  `enumerate` 一处生成最不容易错。

## 附：阶段 1.2b 收口 · README 三行法

> 大白话：闭环跑通后，把板上临时试出来的修法**固化回工程文件**，让下次 `./deploy.sh` 一键就能复现，
> 而不是靠手工敲命令；同时把「会写脏数据的发布」在源头挡掉。

- **① 用了什么语法 / 技术栈**：Python 侧新增纯函数 `is_publishable()` + `isinstance` 类型判定 +
  `dict` 浅拷贝造边界用例 + `assert` 在 `--self-test` 里验「正常 / 断连 / 畸形 / bool」四类；
  Shell 侧 `awk` 从 `ip -4 addr show` 动态提网桥名 + `command -v` 判命令存在 + `if/else` 容错 +
  `curl` 响应体 `grep -q '"error"'` 查 REST 静默失败。
- **② 为什么这么设计**：① 脏数据在**发布端**挡掉，比在 eKuiper 加 `WHERE` 或事后清库都干净，
  也不动已实证的历史数据；② firewalld runtime 绑定**每次部署重绑**而不是 `--permanent --reload`，
  因为后者会冲掉 Docker 自建 iptables 链；③ 动态取网桥名而不是写死接口名，网络重建后名字会变；
  ④ 清理顺序「先删规则再删流」，规则引用流，反过来重跑会撞 `already exists`；
  ⑤ 查运行态用 `/rules/to_influx/status` 而不是 `/rules/to_influx`（后者是 describe）。
- **③ 不这么写会怎样**：① 六路带 `null` 照发 → InfluxDB `field type conflict` 整批写不出去（第 11 条）；
  ② trusted zone 只手工绑一次 → 重启 / 网络重建后复发 `no route to host`（第 8 条，15:03 实证）；
  ③ `tags` 留 `{{.deviceId}}` → 每条记录卡在 transform 阶段（第 9 条）；
  ④ REST 只靠 `set -e` + curl 退出码 → HTTP 200 带 `"error"` 的 JSON 会静默失败、脚本还打印「完成!」。
