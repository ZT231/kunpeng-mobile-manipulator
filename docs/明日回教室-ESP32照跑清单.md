# 明日回教室 · ESP32-S3 照跑清单（2026-11-14 晚备）

> 🔴 **本文是「明天照着跑」的规程，不是实机结果**。三档 ✅实机 / 🔶有代码未实机 / 📋规划 一律不抬。
> ✅ **11-14 晚补记**：整机 `idf.py build` 已在 PC 上跑完并**回读确认通过**（`BUILD_RC=0` / `Project build complete` / 产物齐全，见第一节）；这仍是**编译通过**，不是实机 ⇒ 档位仍 🔶。
> 用户处境：11-14 晚在宿舍，**硬件都在教室**，身边只有鲲鹏开发板；要求「先把相关代码完成，明天回教室直接烧录 / 上传实测」。
> 关联：`docs/ESP32-S3接入与阶段收尾路线.md`（帧实证 + 路线）、`docs/ESP-IDF安装与首次build指引.md`（环境核对）、
> `docs/上板验收-语音网关阶段2.md`（鲲鹏侧网关已实机验收的完整命令/CK 表）、`esp32/README.md`（组件口径与三行法）。

---

## 零、一句话说清「今晚做了什么 / 没做什么」

| | 内容 | 档位 |
|---|---|---|
| ✅ 今晚做完 | `esp32/main/tcp_client.c/.h`（新）、`main.c`（覆写）、`CMakeLists.txt`（覆写）落盘；本机帧/网关语义实证 **`PASS=24 FAIL=0`**；PC 上用现成 ESP-IDF v5.5.5 在 ASCII 副本跑 `set-target esp32s3`（**rc=0**）+ 整机 `idf.py build` **通过**（`Project build complete`、`tcp_client.c` 编译链接成功、产物 4 件齐全） | 🔶 有代码、本机交叉编译通过，**未烧录 / 未上板** |
| 🔴 今晚没做 | 改真实 Wi-Fi 密码、烧录、ESP32 上板联调、ESP-SR 命令词、IoT 节点遥测、`car/` 烧录 | 📋 / 🔶 顺延 |
| 🔴 不许写 | 「ESP32 已接入」「语音控制已实现」「语音→动作闭环」 | 命名红线 |

**核心边界**：今晚写的是「**ESP32 → Wi-Fi → 鲲鹏网关 → 应答**」这段链路自检代码，
发的只是 `$ESTOP*5D` / `$RESUME*19` 两条**纯 ASCII 快速通道**帧 —— 网关 dry-run 也回 `$OK`，
**默认固件不会让机械臂动**。要动臂必须额外开中文帧闸门（第六节），那是可选加试。

---

## 一、整机 build 结论（11-14 晚已回读确认通过，明早不用再等）

今晚的 build 在后台跑完，日志在 PC 的 **`D:\esp\`**（纯 ASCII 副本，不是中文工程目录）。
**已回读 `D:\esp\idf_build2.log` / `D:\esp\_build2_console.log` 末尾，确认：**

| 证据 | 实测值 |
|---|---|
| `EXPORT` | `rc=0`、`EXPORTED_VARS=10`（`activate.py --export --shell bash` 干净 env 生效） |
| `BUILD` | `rc=0` / `BUILD_RC=0` / `FINAL_RC=0` / `BUILD2_SCRIPT_RC=0` |
| 结论行 | `Project build complete. To flash, run: idf.py flash` |
| 新代码 | `[75/96] ... tcp_client.c.obj` → `Linking CXX executable kunpeng_voice.elf` → **无** `undefined reference to getaddrinfo` ⇒ `REQUIRES lwip` 生效 |
| 产物（已核实落盘） | `kunpeng_voice.bin` **803648 B**（0xc4340，app 分区剩 23%）、`kunpeng_voice.elf` 9035736 B、`bootloader/bootloader.bin` 20832 B、`partition_table/partition-table.bin` 3072 B、`flash_args` 155 B |

> 🔴 这只是**交叉编译通过**，不是实机：未烧录 / 未上板 / 未联调 ⇒ **档位仍 🔶**，不许写「已接 ESP32 / 语音控制已实现」。
> 首次全量编译曾触发 20 分钟 `timeout`（`BUILD_TIMEOUT` / `FINAL_RC=6`，Ninja 已到 `[857/1082]`）——**不是编译失败**；用不设 timeout 的增量 build 续跑即通过，排障见 `docs/排障记录-ESP32.md` 第 5 条。

**明早照跑时若要复查 / 环境变了要重跑**（PS 5.1 无 `&&`，单条复制也能跑）：

```powershell
Get-Content D:\esp\idf_build2.log -Tail 25
Get-Content D:\esp\_build2_console.log -Tail 8
Get-ChildItem D:\esp\s3_voice\build\*.bin, D:\esp\s3_voice\build\*.elf -ErrorAction SilentlyContinue | Select-Object Name,Length
```

| 看到什么 | 判定 | 下一步 |
|---|---|---|
| `BUILD_RC=0` + `FINAL_RC=0` + `Project build complete`，且有 `kunpeng_voice.bin` / `.elf` | ✅ 整机编译通过 | 直接跳到第二节 |
| `BUILD_RC≠0` 或日志停在 `===== BUILD =====` 没有回写 | 🔴 编译没过 / 没跑完 | **先别烧**，把日志末尾报错贴回来，按报错改 `esp32/` 源码 → 重新拷副本 → 重跑 `idf.py build` |
| 报 `undefined reference to getaddrinfo` 之类 | 组件依赖没生效 | 核对 `main/CMakeLists.txt` 的 `REQUIRES` 里有 `lwip` |

---

## 二、A 准备（PC 侧，约 5 分钟）

### 2.1 重新同步 ASCII 副本（🔴 必做，别拿昨晚的旧副本直接烧）

今晚副本是 `D:\esp\s3_voice`。如果第一节里改过 `esp32/` 源码，**必须重拷一次**，否则烧的是旧代码：

```powershell
New-Item -ItemType Directory -Force -Path D:\esp | Out-Null
Copy-Item -Path "D:\简历\STM32项目\4.ESP32-S3项目\RoboticArm_Kunpeng\esp32" -Destination "D:\esp\s3_voice" -Recurse -Force
Set-Location D:\esp\s3_voice
```

> 🔴 官方构建系统**不支持路径含中文 / 空格**，所以工程只能在这份 ASCII 副本里 build；
> 原工程 `RoboticArm_Kunpeng\esp32\` 一行不动。改动只在副本里做，改完记得**回填原工程**（否则下次同步会丢）。

### 2.2 填真实热点（🔴 只在副本里改，不提交 Git）

用记事本 / VS Code 打开 `D:\esp\s3_voice\main\wifi_sta.h`，改这两行：

```c
#define WIFI_SSID       "教室热点真实SSID"
#define WIFI_PASSWORD   "教室热点真实密码"
```

**PC、ESP32、鲲鹏板三者必须连同一个热点 / 路由**，否则 `connect()` 一定失败。

### 2.3 查鲲鹏板在教室网络里的真实 IP（🔴 极容易漏）

记忆里的 `10.160.120.1` 是**USB 共享网络**时的地址；教室热点下鲲鹏是路由器分配的局域网 IP。在鲲鹏板上：

```bash
ip -4 addr show | grep inet
```

把结果（例如 `192.168.1.23`）填进 `D:\esp\s3_voice\main\tcp_client.h`：

```c
#define GATEWAY_HOST   "192.168.1.23"
#define GATEWAY_PORT   9000
```

改完**重新 build**（`idf.py build`，增量编译很快）。不改这一条，ESP32 会连到不存在的地址，现象是 `connect() 失败: errno=...`，很容易被误判成「Wi-Fi 没连上」。

### 2.4 查烧录串口号

PC 设备管理器 → 端口（COM 和 LPT），找 **CH343** 对应的 `COMx`，记下来（下面用 `COM7` 举例，实际按你机器写死、每次重插重查）。

---

## 三、B 鲲鹏侧起服务（板端命令一律**单行逐条粘**）

> 前置：机械臂固件已上电、串口线接好。**这些命令不动臂**（网关默认 dry-run）。

**B1 · 确认 Flask 在跑**（网关靠 `/api/cmd` 下发，判活只认 `enabled:true`；🔴 启动必用绝对路径 + `python3 -u`，**不带 `--radial-fix`**）

```bash
curl -s -X POST http://127.0.0.1:5000/api/ik/eye -H "Content-Type: application/json" -d '{}'
```

没起就先起（**5000 只能一个 Flask**，先 `pkill -f web_control.py`）：

```bash
pkill -f web_control.py
python3 -u /root/kunpeng_vision/web_control.py --host 0.0.0.0 --port 5000 --eye-in-hand > /root/web.log 2>&1 &
```

**B2 · 网关结构自检（零硬件、不碰臂）**

```bash
python3 -u /root/kunpeng_vision/robot/voice_gateway.py --host 0.0.0.0 --port 9000 --self-test
```

**B3 · 起网关（先 dry-run，只验「帧能不能进来」）**

```bash
pkill -f voice_gateway.py
python3 -u /root/kunpeng_vision/robot/voice_gateway.py --host 0.0.0.0 --port 9000 > /root/gw.log 2>&1 &
```

🔴 **`--host 0.0.0.0` 不能省**：`voice_gateway.py` 默认 bind `127.0.0.1`，ESP32 从 Wi-Fi 连过来会 `Connection refused`。

**B4 · 从 PC 先探一枪，确认门确实开在网卡上**（把 IP 换成 2.3 查到的）

```bash
printf '$ESTOP*5D\r\n' | nc 192.168.1.23 9000
printf '$RESUME*19\r\n' | nc 192.168.1.23 9000
```

期望分别回 `$OK,ESTOP*75`、`$OK,RESUMED*75`。`nc` 不通时用 Python socket 一行兜底：

```bash
python3 -c 'import socket;s=socket.create_connection(("192.168.1.23",9000));s.sendall(b"$ESTOP*5D\r\n");print(repr(s.recv(256)));s.close()'
```

> `printf` 能解释 `\ooo` / `\r` / `\n`；**别换成 `echo`**（默认不解释转义，CK 会错）。

---

## 四、C 烧录（PC · 开「ESP-IDF 5.5 PowerShell」快捷方式，🔴 别用 Git Bash）

> Git Bash 跑 `idf.py` 会被官方拒 `MSys/Mingw is no longer supported`；
> ESP-IDF 装在 **PC**（USB 要摸到板载 CH343），**不装在鲲鹏板**。

```powershell
Set-Location D:\esp\s3_voice; idf.py build
```

```powershell
Set-Location D:\esp\s3_voice; idf.py -p COM7 flash monitor
```

> `COM7` 换成 2.4 查到的实际串口号。若自动找口成功，可去掉 `-p COM7` 直接 `idf.py flash monitor`。

**烧录后串口 monitor 期望看到（顺序）**：

```text
I (xxx) voice_main: ==== ESP32-S3 语音入口骨架启动（含 TCP 上行自检） ====
I (xxx) wifi_sta: ... Got IP ...
I (xxx) voice_main: Wi-Fi OK, is_connected=1；网关目标 192.168.1.23:9000
I (xxx) voice_main: [本地封帧 0] $ESTOP*5D（长度=11，只打印不下发）
I (xxx) voice_main: [本地封帧 1] $RESUME*19（长度=12，只打印不下发）
I (xxx) tcp_client: TCP connected -> 192.168.1.23:9000
I (xxx) tcp_client: send 11 bytes: $ESTOP*5D
I (xxx) tcp_client: recv 14 bytes: $OK,ESTOP*75
I (xxx) voice_main: 网关应答（14 字节）: $OK,ESTOP*75
I (xxx) voice_main: 中文帧默认关闭（SEND_CN_HOME_FRAME=0），明天联调再开
I (xxx) voice_main: [000] 读到 256 个采样: ... ... ... ...
```

🔴 **看到 `$ASR,mic_ok*50` 是旧固件的打印**；新固件自检帧是 `$ESTOP*5D` / `$RESUME*19`。
若还看到旧的，说明烧的是旧副本 / 没重新 build。

---

## 五、D 四层硬判据（跑到这一层才算「ESP32 链路通」）

| # | 证据 | 在哪看 | 通过标准 | ☐ |
|---|---|---|---|---|
| 1 | ESP32 侧 `TCP connected` | 串口 monitor | 打印 `TCP connected -> <鲲鹏IP>:9000`，且两条帧都 `send N bytes` | ☐ |
| 2 | 网关应答帧 | 串口 monitor | `$OK,ESTOP*75` 与 `$OK,RESUMED*75`，**不是** `$ERR,2*5B` | ☐ |
| 3 | 鲲鹏侧网关日志 | `tail -20 /root/gw.log` | 有 `payload`=`STOP` / `RESUME` 的记录、`apply:false`（dry-run） | ☐ |
| 4 | 外部主机也能连 | PC 上 B4 的 `nc` | 回同样的 `$OK,...`，证明 `--host 0.0.0.0` 生效、不是本机自连 | ☐ |

```bash
tail -20 /root/gw.log
```

**🆕 11-15 补：别靠人眼翻日志，用 `kunpeng/robot/check_esp32_link.py` 自动判定**（宿舍无硬件时写，本机 `--self-test` 5 组全绿）。把串口 monitor 存成 `uart.txt`、`/root/gw.log` 存成 `gw.log`、B4 的 `nc` 输出存成 `probe.txt`，在 PC 上跑：

```bash
python3 check_esp32_link.py --uart-log uart.txt --gw-log gw.log --probe-out probe.txt
```

它会自动否决三类最容易看走眼的情况：烧成旧固件（串口出现 `$ASR,mic_ok*50`）、收到 `$ERR,2*5B`、
网关被误开 `--apply`（日志 `apply=true`，会真动臂）；输出 `RESULT: PASS=4 FAIL=0` 才算四层齐。
🔴 **脚本只判据、不抬档**，仍要人工填第八节记录表并回报后才统一订文档。

> 🔴 这四层**只证明链路通**，不证明「语音控制已实现」：ESP32 还没接 WakeNet / ASR，
> 发的是写死的自检帧，`source` 恒 `mock`。**档位仍是 🔶**，回报记录表后才按第八节统一订文档。

---

## 六、E 可选加试：让臂真的动（🔴 不是今晚交付的必做项，做之前先读三条安全前提）

只有**链路四层全绿**后，才考虑这一步。三条前提缺一不可：

1. 臂已摆 **C 姿态**，脉宽 `1636,1470,2177,1106,1450,1567`（下发 curl 见 `kunpeng/robot/README.md`），**周围无人**；
2. 鲲鹏网关切到 `--apply`（此时中文帧才会真下发 `HOME`）：
   ```bash
   pkill -f voice_gateway.py
   python3 -u /root/kunpeng_vision/robot/voice_gateway.py --host 0.0.0.0 --port 9000 --apply > /root/gw.log 2>&1 &
   ```
3. PC 副本里把 `D:\esp\s3_voice\main\main.c` 的 `#define SEND_CN_HOME_FRAME 0` 改成 `1`，
   **重新 `idf.py build` + 重烧**。中文帧用 C 八进制字节串写死：
   `"ASR,\345\233\236\345\210\235\345\247\213\344\275\215"` ⇒ `$ASR,回初始位*61\r\n` ⇒ `$OK,HOME*27`。

加试判据（沿用阶段 2 已实机验收的四层）：① 应答 `$OK,HOME*27` ② 日志 `payload=HOME` / `sent:true` / `apply:true`
③ `/api/state` 脉宽从 C 姿态变初始位 `1580,1900,2400,700,1450,1500`
④ **肉眼看到臂从 C 姿态收回初始位**。

🔴 **收尾（必做）**：断电前把臂回初始位、停掉 `--apply` 网关；中文帧宏改回 `0` 再重烧，避免下次上电误触发。

```bash
pkill -f voice_gateway.py
```

---

## 七、F 失败分流（别一上来就改代码）

| 现象 | 最可能根因 | 先做这一条 |
|---|---|---|
| `Wi-Fi 未连上，跳过 TCP 上行` | SSID/密码错、PC/ESP32/鲲鹏不在同一热点 | 核对 2.2；看串口 `wifi_sta` 的 retry 日志 |
| `connect() 失败: errno=...` | `GATEWAY_HOST` 还是 `10.160.120.1`（USB 网段），教室不是这个 | 按 2.3 查真实 IP 改宏、**重新 build** |
| `recv() 返回 -1（超时）` | 网关没起 / 没带 `--host 0.0.0.0` / 防火墙 | 板端 `tail -20 /root/gw.log`；PC 上先跑 B4 |
| 收到 `$ERR,2*5B` | 发成了 `$ASR,mic_ok` 或 `$ASR,home`（词表只有中文） | 确认烧的是新固件；纯 ASCII 快速通道不该出现这个 |
| 串口全是乱码 | 波特率 / 中文帧显示问题 | 只看 ASCII 帧；中文 UTF-8 帧显示乱码但**字节是对的**，别改宽字符 |
| `I2S 启动失败` | BCLK/WS/DIN 三根线或 L/R 引脚 | 单独定位，与网络链路无关；本地封帧自检仍会先跑 |

> 🔴 板端命令一律**单行逐条粘**；SecureCRT 多行粘贴会回显串扰，别把「看着像乱了」当成命令失败。

---

## 八、记录表（跑完填，别提前打勾）

| # | 判据 | 期望 | 实测 | 通过 |
|---|---|---|---|---|
| 0 | 第一节 build 结论 | `BUILD_RC=0` + 有 `.bin` | ☐ | ☐ |
| 1 | 串口 `TCP connected` | 连到鲲鹏 `:9000` | ☐ | ☐ |
| 2 | 两条应答帧 | `$OK,ESTOP*75` / `$OK,RESUMED*75` | ☐ | ☐ |
| 3 | 网关日志 | `payload`=`STOP`/`RESUME`，`apply:false` | ☐ | ☐ |
| 4 | PC 侧 `nc` 外部探活 | 同样回 `$OK,...` | ☐ | ☐ |
| 5 | 每秒 PCM 采样 | 非恒定、说话时变化 | ☐ | ☐ |
| 6 | （可选）中文帧加试 | 四层证据 + 肉眼见臂动 | ☐ | ☐ |

这张表全绿并回报后，才统一订 `esp32/README.md`、`docs/核心化路线图-ESP32与IoT.md`、根 `README.md`、
`docs/简历-具身智能项目条目.md`、`MEMORY.md` 的档位；**只抬「ESP32 链路自检已实机打通」这一条**，
语音入口整体仍 🔶（WakeNet / 命令词 / ESP32 遥测进 EMQX 都还没有代码）。

---

## 九、诚实边界 / 命名红线

- 🔴 今晚**未烧录、未上板、未联调**；全部新增内容 🔶 有代码未实机。
- 🔴 鲲鹏侧阶段 2 网关链路 ✅（11-14 16:40 实机验收），但 `source` 恒 `mock`、**没接 ESP32**；
  `$ASR` 帧草案此前只做到「C / Python 语法层逐字节一致」，**语义层实机对齐要等明天**。
- 🔴 禁写「语音控制已实现 / 语音→动作闭环 / 已接 ESP32 / 自然语言控制机械臂」；
  只叫「可观测性闭环 / 视觉引导的移动操作平台」。
- ✅ 明天四层全绿后能写：「ESP32-S3 侧经 Wi-Fi TCP 把统一帧发到鲲鹏网关并收到校验通过的应答，
  完成端侧节点到边缘侧收帧服务的链路联通」——**仅此一句**，不外延到语音识别或抓取。
- 🔴 小车 `car/` 烧录、第二路 USB-TTL 统一调度按用户要求**仍放最后**，本文不涉及。
