# IoT · ESP32 节点遥测与 eKuiper 告警（提前草案）

> 📋 **提前草案 / 未实机**：本文件 2026-11-15 傍晚在宿舍写就（身边只有鲲鹏板、教室硬件不在）。
> 它**不是**「ESP32 遥测已接入」，只是把「不依赖教室硬件、能在本机验证」的
> **字段契约 + 板侧适配层 + 规则文本**先落地，等回教室烧录 / 联调时照着接。
> **三档**：支柱③ 仍是 ✅ 板内四容器 / 🔶 未覆盖 ESP32 节点、无告警规则 —— 本草案**不改变档位**。

## 0. 为什么提前写、以及它在路线图的位置

- 用户 11-14 重排的下一步：①先接 ESP32-S3 → ②ESP32 真发帧 → ③ESP-SR →
  **④IoT 补 ESP32 节点遥测 + eKuiper 告警** → ⑤ACS712 → ⑥A 项实机验收包。
- 本草案对应第 ④ 步，即 `docs/核心化路线图-ESP32与IoT.md` 的**阶段 5 / 阶段 6**（第 58、59 行）。
  按排序它必须排在「ESP32 真发帧」之后，所以现在只写**可静态校验、可本机单测**的部分，
  不碰任何已实机的板内组件。
- 大白话：宿舍没硬件，能干的只有「把规矩定清楚、把代码写好、在本机跑通」；
  真正连 EMQX / eKuiper / InfluxDB / 真 ESP32 节点，都得回教室。

## 1. 边界：本轮动了什么、绝对没动什么

| 类别 | 文件 | 状态 |
|---|---|---|
| 🆕 新增 | `iot/esp32_telemetry.py` | 字段契约 + 板侧适配层 |
| 🆕 新增 | `iot/esp32_rules.json` | 2 流 3 规则草案文本（`meta.status = "draft"`） |
| 🆕 新增 | `iot/test_esp32_telemetry.py` | 27 项 unittest，零第三方、零联网 |
| 🆕 新增 | `docs/IoT-ESP32遥测与eKuiper告警草案.md` | 本文件 |
| ✏️ 追加 | `iot/README.md` 末尾三行法 | 按项目「每模块补三行」规矩 |
| ✏️ 追加 | `.workbuddy/memory/2026-11-15.md`、`MEMORY.md` | 日志 / 指针同步 |

🔴 **绝对没动**（越线即破坏已实机现场）：

- `iot/docker-compose.yml`：板内四容器已实机，且读取被敏感保护拦截 ⇒ 不读、不改、不推断。
- `iot/deploy.sh`：里面 `to_influx` 规则已实机，本轮不改一行（但落地前**必须**给它加过滤，见 §4.2）。
- `esp32/` 固件源码：明天要照 `docs/明日回教室-ESP32照跑清单.md` 烧录，当前编译副本
  `D:\esp\s3_voice` 已整机 build 通过，一行不动。
- `.ioc`：不碰 CubeMX 配置；不装任何包。
- `docs/核心化路线图-ESP32与IoT.md` 的**阶段表与档位结论**：本文件只**引用**路线图第 58/59/71/72 行，不改它。
- 根 `README.md`、`docs/简历-具身智能项目条目.md`、`docs/第四项目整合说明.md`：未实机内容不扩散到对外口径。
- **不抬档**。

## 2. 字段契约（全部来自路线图，不自己发明）

主题与 measurement（对齐 `docs/核心化路线图-ESP32与IoT.md` 第 71、72 行）：

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

### 概念（大白话 + 规范定义）

- **MQTT 主题**：大白话 = 设备把消息贴到一块「公告栏」，订阅这块板的人就收到；
  规范定义 = MQTT 3.1.1 / 5.0（OASIS 标准）基于 TCP 的发布-订阅协议，
  主题用 `/` 分层、`+` 为单层通配符、`#` 为多层通配符。
- **InfluxDB field 类型锁定**：大白话 = 一格第一次装了「小数」，以后就只能装小数，
  塞字符串会连累整批写不进去；规范定义 = InfluxDB 2.x 的 field value 只允许
  `float / integer / unsignedInteger / string / boolean` 五种标量，且每个
  `measurement + field` 的类型**首次写入即锁定**（`docs/排障记录-IoT.md` 第 11 条的 Root Cause）。
- **eKuiper 流 / 连续查询**：大白话 = 在板子本地用一句 SQL 把流过的数据筛一遍再转发，不用上云；
  规范定义 = 面向无界流的连续查询引擎，用类 SQL 对数据流做投影 / 过滤 / 聚合，
  schemaless 建流必须写空括号 `()`。
- **paho 延迟导入**：大白话 = 只在真要连 MQTT 时才 `import`，没装包的机器也能跑自测；
  规范定义 = Python 模块导入发生在运行期，把 `import` 放进函数体可把
  `ModuleNotFoundError` 推迟到该分支真正执行。
- **`${INFLUX_TOKEN}` 占位符**：大白话 = 别把密码写进文件，部署时由环境变量临时填；
  规范定义 = 12-factor app 的「配置与代码分离」，凭据走环境变量 / secret store，不进版本库。

## 3. 三个模块各自做了什么

### 3.1 `esp32_telemetry.py`（契约 + 板侧适配层）

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

### 3.2 `esp32_rules.json`（静态可校验文本）

`meta.status = "draft"`，2 个流、3 条规则，**未部署、未连任何服务**：

| id | 类型 | SQL / 动作 |
|---|---|---|
| `esp32_stream` | 流 | `DATASOURCE="device/esp32_01/telemetry"`（精确主题） |
| `alert_stream` | 流 | `DATASOURCE="alerts/+"` |
| `esp32_to_influx` | 规则 | `SELECT rssi, heap_free, wake_cnt, cmd_ok, deviceId` → `esp32_telemetry` measurement |
| `esp32_rssi_low` | 规则 | `WHERE rssi < -70` → MQTT 回 `alerts/esp32_01` |
| `esp32_alerts_to_influx` | 规则 | `SELECT deviceId, kind, level, detail` → `alerts` measurement |

### 3.3 `test_esp32_telemetry.py`（27 项 unittest）

覆盖纯函数、规则 JSON 静态结构、子进程 CLI / ASCII stdout 回归。
🔴 只证明「本机逻辑对」，**不证明**已部署 / 已上板 / 已接入 ESP32。

## 4. 五个关键技术决策（讲根因，不是调参打补丁）

### 4.1 脏数据在「发布端」挡掉，不靠下游

- **现象**：串口断连 / 字段缺失时，eKuiper influx2 sink 不跳过 JSON `null`，会把它序列化成 **string**，
  撞上历史点锁定的 **float** ⇒ InfluxDB `partial write: field type conflict`、整批写不出去。
- **根因**：`docs/排障记录-IoT.md` 第 11 条 —— InfluxDB 2.x 类型首次写入即锁定；
  「进程不崩」不等于「数据无害」。
- **解决**：`is_publishable()` 在 `client.publish()` 之前判定，四字段必须全是真数字，
  `bool` 显式排除（Python 里 `bool` 是 `int` 子类，`isinstance(True, int)` 为 True，
  不排除会把 `True` 写成 1）。
- **一句话面试说辞**：「我不在下游清库或换 measurement 绕开，而是在发布端保证每一格都是合法标量，
  因为时序库的 field 类型是写一次就锁死的。」

### 4.2 独立 measurement + 精确 DATASOURCE，且落地前必须改 `deploy.sh` 的过滤

- **现象**：现有 `deploy.sh` 的 `to_influx` 订阅 `device/+/telemetry` **通配**，
  会把 `device/esp32_01/telemetry` 也当成机械臂数据，`SELECT` 出 `j1~j6` 等 `null` 字段写进 `arm_telemetry`。
- **根因**：通配主题 + schemaless 流 ⇒ 契约外消息也会进同一条规则。
- **解决（草案）**：ESP32 用**精确** `DATASOURCE` 的独立流 + 独立 `esp32_telemetry` measurement 隔离。
- 🔴 **落地前必做**：给 `to_influx` 的 SQL 加 `WHERE deviceId='kunpeng_01'`，否则隔离不完整。
  **本草案不擅自改 `deploy.sh`**（已实机文件），只在 `esp32_rules.json` 的
  `meta.isolation_note` 里钉死这条待办。
- **一句话面试说辞**：「两套设备共用通配主题时，先按 deviceId 分流再写库，否则类型冲突是迟早的。」

### 4.3 心跳超时放板侧看门狗，不用 eKuiper 窗口

- **根因**：窗口 / 触发器由「收到消息」驱动；设备彻底离线时**一条消息都没有**，SQL 窗口根本不会触发。
- **解决**：`is_heartbeat_stale(last_ts, now)` 单调比较，超时由板侧生成 `heartbeat_timeout` 告警 →
  `alerts/esp32_01` → `esp32_alerts_to_influx` 落库。时钟回拨（`now < last_ts`）返回 False，
  避免 NTP 校时误报全网离线。
- **一句话面试说辞**：「离线是『没有事件』，流引擎只能处理『有事件』，所以离线检测必须在设备侧或外部看门狗。」

### 4.4 不用 `dataTemplate`，tags 用静态 map

- **根因**：`docs/排障记录-IoT.md` 第 9 条 —— eKuiper 2.1.0 求模板时上下文是
  `[]map[string]interface{}` **切片**，Go `text/template` 对切片取 `{{.deviceId}}` 会报
  `can't evaluate field deviceId`。`deviceId` 本就是常量，没必要模板化。
- **解决**：`"tags": {"device": "esp32_01"}` 静态 map；
  单测 `test_no_password_key_and_no_data_template` 显式禁止出现 `dataTemplate` 键。

### 4.5 凭据一律占位符

- `esp32_rules.json` 里所有 `token` 都是 `${INFLUX_TOKEN}`，不写密码、不写真实值；
  单测 `test_tokens_are_placeholders_only` 逐个校验 `${...}` 格式。
- 🔴 本轮甚至没有独立核实 `iot/docker-compose.yml` 的凭据（读取被敏感保护拦截，
  按规则不重试 / 不推断 / 不绕过），草案只沿用「既有 provisioning 已注入」这一事实，不复述任何值。

## 5. 本机验证（硬证据；不是实机）

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

### 排障：首跑 `PASS=4 FAIL=2` 的 3 个不一致

| 现象 | 根因 | 解决 | 一句话面试说辞 |
|---|---|---|---|
| `--dry-run` 参数不存在，子进程 `rc=2` | docstring 写了用法，但 argparse 没实现 | 补 `add_argument("--dry-run")`，并让 `if args.publish and not args.dry_run` 保证 dry-run 优先 | 「文档和 CLI 要一起改，否则示例命令根本跑不通」 |
| `--values "-82,120000,7,5"` 报 `expected one argument` | 值以 `-` 开头，argparse 把它当成新选项 | 改成等号形式 `--values=-82,120000,7,5`，并在 docstring 写明这个坑 | 「负号开头的参数值必须用 `--key=value`，这是 argparse 的词法规则」 |
| 规则 JSON 单测 `DATASOURCE` 断言失败 | JSON 里 `\"` 解析后就是 `"`，测试多写了一层反斜杠 | 断言改成 `'DATASOURCE="device/esp32_01/telemetry"'` | 「JSON 字符串里的转义在解析后就不存在了，断言要按解析后的值写」 |

> 这三条都属「检查工具与被检查对象不一致」，不是产品逻辑缺陷；
> 按项目规矩只记**关键 / 面试官关心**的，鸡毛蒜皮不记。

## 6. 诚实边界（🔴 别越线）

- 🚫 禁写：「ESP32 遥测已接入」「已部署告警规则」「IoT 三节点闭环已打通」「已上板 / 已联调」。
- 🔴 **ESP32 固件侧真正的 MQTT client 尚未写**：真节点还没烧录、没连 Wi-Fi，
  本模块只覆盖「契约层 + 板侧适配层」，不是端到端已接入。
- 🔴 `esp32_rules.json` 只是静态文本：`meta.status = "draft"`，未连 EMQX / eKuiper / InfluxDB，未部署。
- 🔴 `DEMO_VALUES` 是假数据；27 项单测只证明本机逻辑，**编译 / 单测通过 ≠ 实机**。
- 🔴 档位：支柱③ 仍 ✅ 板内四容器 / 🔶 未覆盖 ESP32 节点 + 无告警规则；本草案**不改变档位**，
  也没有把「核心」抬成已完成。

## 7. 回教室落地前必做（照跑清单）

1. **先按主线走**：`docs/明日回教室-ESP32照跑清单.md` —— 烧 ESP32-S3、跑四层硬判据、
   ESP32 真发帧钉死语义、再进 ESP-SR。本草案排在它们之后。
2. 写 **ESP32 固件侧 MQTT client**：把 `rssi / heap_free / wake_cnt / cmd_ok` 换成真读数，
   `DEMO_VALUES` 不许留。
3. **`deploy.sh` 的 `to_influx` 加 `WHERE deviceId='kunpeng_01'`**（§4.2），再部署本草案的 2 流 3 规则。
4. `${INFLUX_TOKEN}` 由环境变量注入，不硬编码；Grafana 加 ESP32 面板与告警面板。
5. 验收判据回到路线图阶段 5 / 6：第二条设备线随时间变化；人为走远 / 拔网线，
   `alerts` 出现记录、面板变红。

---

## 附：本轮同步范围（为什么没改根 README / 简历条目 / 路线图档位）

按项目纪律，未实机的提前草案不扩散到对外口径文件：根 `README.md`、
`docs/简历-具身智能项目条目.md`、`docs/第四项目整合说明.md`、
`docs/核心化路线图-ESP32与IoT.md` 的**阶段表与档位结论**均保持原样（本文件只**引用**路线图
第 58/59/71/72 行）。只做三件同步：① `iot/README.md` 补三行法；
② `.workbuddy/memory/` 留日志与指针；③ 本草案文档。
都属于「指针 + 复习提纲」，不改任何状态结论。
