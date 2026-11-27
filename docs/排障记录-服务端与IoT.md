# 排障记录：服务端与 IoT

> 本文由 2026-11-21 的多份服务端 / IoT 草稿合并整理而成（内容未改写，仅重组与订正；源文件已删除，以本文为现行版本）。
>
> 硬件口径以《硬件清单与接线》与《00-项目总览与硬数据速查》为准。

> 🔴 **历史记录提醒（2026-11-24）**：本文中的 `$JOINT`、`HOME/STOP/RESUME`、`Arm_Init()` 都是**旧机械臂固件口径**；STM32 相关代码已删除，当前底盘急停用 `$CAR,STOP` 或 `$STOP`。本文的价值在于记录当时的决策与边界，不是当前固件操作手册。

## 目录

1. 鲲鹏侧语音入口网关
2. IoT 遥测阶段
3. LLM 意图解析层

---

## 一、鲲鹏侧语音入口网关

> 来源：`排障记录-服务端与IoT.md`

> 定稿 2026-11-14。格式固定「现象 → 排查过程 → 根因 → 解决 → 一句话面试说辞」。
> 只记**关键 / 面试官会追问**的问题；1~4 是本次开发真实踩到或当场验证的坑，5 是**设计阶段预判并规避**（不是实机故障），
> **6~7 是 11-14「代码仔细检查一遍」时用 `PYTHONIOENCODING=ascii` 与词表逐条实跑实证的两处真冲突**（照已删除的 `car/排障记录.md` 的口径如实标注）。
> 1~7 是**本机 / PC 侧**验证，不是实机故障；**8 是 11-14 板端实机验收里两个「看着像故障」的误判点**（全程零故障）。
> ✅ 网关**阶段 2 链路 2026-11-14 16:40 已上板实机验证**（四层硬判据全绿，记录见 `docs/KunPeng部署与语音网关上板验收.md`）；
> 🔴 **语音入口整体仍 🔶**（ESP32 已烧录实机（TCP 9000/采音），$ASR 帧草案未与 ESP32 对齐）。
> 关联：~~`car/排障记录.md`~~（4 条，随 `car/` 于 2026-11-24 删除）、`docs/排障记录-服务端与IoT.md`（原 `手眼标定与一车全载抓取.md` 已于 2026-11-24 删除）。

---

### 1. 好帧被判 `ERR,1 bad_ck_len`：shell 单引号里的 `\r\n` 是 4 个普通字符（真实踩到）

**现象**：用 `python3 voice_gateway.py --frame '$ASR,回初始位*61\r\n'` 喂一条**校验和正确**的帧，网关一律回 `$ERR,1`，`parse_frame()` 报 `bad_ck_len`；换成 `--text "回初始位"`（内部用 `build_frame()` 造帧）却完全正常。

**排查过程**：先确认 CK 没错 —— `xor_checksum("ASR,回初始位")` 与手工算的一致，`--make-frame "回初始位"` 打出来的帧也是 `*61`。差异只在**帧从哪来**：一个 shell 里手写、一个程序造。把收到的字符串 `repr()` 打出来，看到 `'...*61\\r\\n'`（4 个可打印字符 `\` `r` `\` `n`），而不是真实的 CR/LF。

**根因**：**转义序列与字节值的差异**。shell 单引号不做转义展开，`\r\n` 在命令行里是字面量 4 个 ASCII 字符；`parse_frame()` 用 `rstrip("\r\n")` 只去真实 CR/LF，于是 `*` 后面变成 `61\r\n`（字面量），校验位长度不是 2 ⇒ 判成坏帧。规范定义：CR/LF 是控制字符 U+000D / U+000A，字面量 `\r\n` 是四个可打印字符，二者字节序列不同。

**解决**：在网关**适配层**加纯函数 `normalize_frame()`，把字面量 `\r\n` 还原成真实换行（**顺序不能反**：先换长的 `\r\n`、再换短的 `\n`，否则会残留一个字面量 `\r`），在 `handle_frame()` 最前面调用。刻意**不改** `llm/text_gateway.py` —— 它是底层纯函数，不该替调用方猜格式，改了还会牵动它 18 项既有单测。归一化后 `handle_frame()` 与 `special_text()` 都受益；单测 `test_normalize_literal_crlf` / `test_normalize_order_matters` 锁住。

**一句话面试说辞**：「外部输入和内部契约的边界要由**适配层**处理，不是往底层纯函数里塞特判 —— 我用一层归一化把 shell 字面量换行还原，底层解析器和它的 18 项单测一行没动。」

---

### 2. `$ESTOP*CK` 急停帧被 `NOT_ASR` 丢掉：急停不该等意图解析（真实踩到）

**现象**：想给 ESP32 留一条最短的急停通道 `$ESTOP*CK\r\n`，实测被 `parse_frame()` 判 `RejectReason.NOT_ASR` 后直接丢弃；急停必须先用 `$ASR,急停*CK` 这种文本帧才能进，慢了一层。

**排查过程**：读 `text_gateway.parse_frame()`，它按帧头命令字校验、只认 `ASR,...`，非 ASR 一律拒收 —— 这是**入口契约**的正确行为，不是 bug。问题在于「急停」这条语义比「ASR 文本」更底层、更紧急，不该被文本入口的格式要求挡住；而且翻译员 `parse_intent()` 可能返回 `unknown`，等它查表会多一跳延迟。

**根因**：把「急停」和「普通语音文本」放在了同一条校验路径上。急停优先级应当高于命令字分类，也高于意图解析。

**解决**：加 `extract_payload()`（只读 `$` 与 `*` 之间的 payload，**CK 仍由 `parse_frame` 校验过，不是绕过校验**）+ `special_text()` 快速通道：`result.reason == NOT_ASR` 且 payload 命中 `SPECIAL_PAYLOADS`（`ESTOP`/`STOP` → 急停、`RESUME` → 恢复）时，直接把对应中文文本交给仲裁器。同时把**急停词文本匹配放在 `parse_intent()` 之前**（`arbitrate()` 第①步），保证最快。单测 `test_non_asr_estop_fast_channel` 覆盖。

**一句话面试说辞**：「急停是安全路径，我让它绕过意图解析和命令字分类，只在网关层做文本匹配 + 转发固件 `STOP`；但**校验没绕过** —— CK 仍由原有解析器验过，快速通道只是『再读一次 payload』。」

---

### 3. `--apply` 但 Flask 没起：网关抛异常、客户端收不到应答（真实踩到）

**现象**：加了 `--apply` 真下发、但 5000 网页服务没启动时，`urllib.request.urlopen()` 抛 `URLError`（`Connection refused`；有系统代理时是 `502 Bad Gateway`），异常一路冒到 TCP handler，连接直接断，客户端既收不到 `$OK` 也收不到 `$ERR`。

**排查过程**：把「下发」从主流程里单独拎出来看 —— `send_cmd()` 是唯一会碰外部世界的函数，网络错、Flask 回 `ok:false`、固件回 `ERR,x` 三种失败此前都没被统一收口。

**根因**：**副作用没有与错误码映射隔离**。同理，`/api/cmd` 的 `ok:true` 只代表「串口写进去了」，固件真正执行结果在 `resp` 里；只看 `ok` 会把「帧写错」当成「执行成功」（`action_primitives.send_joint()` 是同一套口径）。

**解决**：`send_cmd()` 统一抛 `RuntimeError`；`handle_frame()` 的 `Decision.ALLOW` 分支用 `try/except RuntimeError` 兜住，回 `build_reply("ERR,4")` 并在日志字典里记 `downlink_error`，进程不崩。错误码沿用项目统一语义：`ERR,1` 帧错 / `ERR,2` 未识别 / `ERR,3` 仲裁拒绝 / `ERR,4` 下发失败。单测 `test_downlink_failure_returns_err4_without_crash` 用会抛错的假 sender 覆盖。

**一句话面试说辞**：「下发层是唯一有副作用的地方，我把它做成可替换函数并统一异常→错误码映射：Flask 没起就回 `ERR,4` 而不是让连接断掉，单测能用一个会抛错的假 sender 验证。」

---

### 4. 多线程 TCP 下急停状态会漏判 + 日志看不见（预判，已用并发单测验证）

**现象（预判并当场验证）**：`ThreadingTCPServer` 每个连接一个线程，若急停标志是裸布尔，A 连接刚判完「没急停」、B 连接紧接着发急停，A 的后续下发仍会被放行；另外板上 `nohup … > 日志` 时，`print()` 走 stdout 块缓冲，日志里看不到裁决记录。

**排查过程**：这两条都是**已有事故的同源问题** —— 11-14 启动 Flask 时中文横幅卡在 stdout 缓冲区，误判成启动失败（后用 `python3 -u` 根治）；串口帧是字节流，两个进程同时 `read()` 会抢帧（标定链踩过），所以本网关刻意不直连串口，只通过 HTTP 调 5000。

**根因**：共享可变状态无互斥保护 + 重定向后 stdout 缓冲策略变化。规范定义：互斥量（`threading.Lock`）保护共享状态避免竞态条件；`print(flush=True)` 或 `python3 -u` 强制无缓冲。

**解决**：`GatewayState` 把 `_estop` 收进 `threading.Lock` 的 `get_estop()` / `set_estop()`；`_FrameHandler` 处理完统一 `print(json.dumps(record, ensure_ascii=False), flush=True)`。仲裁器 `arbitrate()` 保持**纯函数**（不读全局状态，`estop_active` 由外层传入），改状态交给 TCP 外壳，单测才能断言 `new_estop` 正确传播。`test_concurrent_toggle_does_not_crash` 用 8 线程 × 200 次 toggle 验证不崩。

**一句话面试说辞**：「仲裁器写成纯函数、状态交给带锁的外壳，这样既能并发安全，又能在不开端口不动臂的情况下单测每一条判决。」

---

### 5.【设计阶段预判并规避，不是实机故障】路线图原写「下发 `$JOINT`」：一步踩进四个雷区

**现象**：`docs/固件与ESP32开发.md` 阶段 2 原判据是「TCP 收帧 → `parse_frame` 校验 → `parse_intent` → 下发 `$JOINT`」，并写「喂一条假帧 ⇒ 机械臂真的动」。按这个写，第一版实现就会去拼六路脉宽。

**排查过程**：对照本项目已定案的四类静默风险逐条过：① `to_pulses()` 里 J3/J5 map 反向、J3/J4 存相对角，手算符号必错；② `to_pulses()` 对越界脉宽**静默夹紧**，`solve()` 成功 ≠ 舵机能到；③ 六路软限位与量程；④ 重力下垂 `SAG_DZ` 净空折算闸门（远端不折算会擦到桌面，11-14 实测踩过）。

**根因**：把「语音链路打通」和「运动控制正确」两件事混在一起了。阶段 2 的目的是验**链路**，不是验**逆解**。

**解决**：白名单只授权固件原生的 `HOME / STOP / RESUME`（`uart_protocol.c:428-451`，均回 `OK`）三个**无参数命令字**，从结构上消掉以上四类风险；`PICK / PLACE / MOVE_LEFT / MOVE_RIGHT` 一律 `ERR,3`，`LOOK` 为 NOOP。默认 dry-run，只有显式 `--apply` 才真下发。路线图阶段 2 的「下发 `$JOINT`」已据此订正为「下发固件原生命令字」，硬判据仍保留「喂假帧 ⇒ 臂真的动」（**上板 `--apply` 才算过**）。

**一句话面试说辞**：「我把『语音能不能驱动动作』和『动作算得对不对』拆成两个阶段：阶段 2 只发三个不带参数的固件原生命令，用白名单把脉宽越界、符号、夹紧、下垂净空这四类风险变成结构上不可能，先证明链路通，再谈复杂动作。」

---

### 6. 板端 `LANG=C`：服务起不来、`--self-test` rc=1、日志吞掉应答（11-14 代码审查实测）

**现象**：用 `PYTHONIOENCODING=ascii` 模拟鲲鹏板 `LANG=C` 本机复现，三个症状连成一条链：
① 起网关（含 `--apply`）时 `main()` 启动横幅 `print("语音入口网关 · TCP …")` 抛 `UnicodeEncodeError`，**进程直接退出，服务根本没起来**；
② `python3 -B voice_gateway.py --self-test` 先打出 `SELF_TEST PASS`，紧接着中文 JSON 抛编码异常，**退出码 1** —— 而它正是上板验收规程 1.3 唯一「零硬件判结构对不对」的自检，会误判成失败；
③ 即便服务侥幸起来（UTF-8 环境），`_FrameHandler.handle()` 里 `print(json.dumps(record, ensure_ascii=False), flush=True)` 写在 `self._write(reply)` **之前**，日志含中文/emoji 抛异常时**应答帧根本发不出去**（客户端 `recv` 得空字节）。

**排查过程**：把每个 `print` 的位置列出来，发现它们全在本机 UTF-8 环境测试，所以原有 32 项单测、TCP 四步链路、dry-run 演练全绿也测不出这个坑；用 `PYTHONIOENCODING=ascii` + socket 发一条中文帧才实证 `GOT REPLY: b''` 与 stderr 的 `UnicodeEncodeError`。根因不在协议、不在仲裁，而在**输出层假设了终端编码**。

**根因**：`print()` 走 `sys.stdout`（TextIOWrapper），编码由 locale 决定；`ensure_ascii=False` 让中文/emoji 原样进入编码步骤。规范定义：`TextIOWrapper.reconfigure(encoding=..., errors=...)` 可就地改编码与错误处理策略；`str.encode("ascii", "backslashreplace")` 把不可编码字符转成 `\uXXXX` 转义，输出恒为可编码字节。**更关键的是副作用顺序**：日志是给人看的、应答是给客户端的，把日志写在应答之前，等于让「可观测性」拥有了阻断「功能」的能力。

**解决**：① 模块加载时 `_configure_stdout()` 尽量把 stdout 设成 UTF-8 + `backslashreplace`；② 所有 `print` 收口到 `_emit()`（正常路径 UTF-8 中文可读，退路 `backslashreplace` 转纯 ASCII，绝不抛）；③ `_FrameHandler` 改成**先 `_write(reply)`、后 `_emit(日志)`**，把「日志不许吞应答」变成结构上不可能；④ 新增 4 项端到端回归（`TestAsciiStdout`）：ASCII 下 `--self-test` rc=0、`--make-frame` 出 `*61`、`_emit` 退路输出纯 ASCII、真 TCP server 能起且中文帧回 `$OK,HOME*27`。**修完重跑 38 项全绿**，`text_gateway.py` / `intent_parser.py` 自检未受影响。

**一句话面试说辞**：「我不信任目标板的 locale —— 把所有输出收口到一层编码容错，并让应答帧先于日志发出；再用强制 ASCII 的子进程回归，把『本机测不出、上板才炸』的问题锁死在本机。」

---

### 7. 喊「解除急停」反而又急停一次：短词表子串歧义（11-14 代码审查实测）

**现象**：`arbitrate("解除急停")` 返回 `payload='STOP'`、`set_estop=True`，整帧回 `$OK,ESTOP*75` —— 用户想解除急停，系统却再次冻结。`RESUME_WORDS` 里明明列了「解除急停」，却永远走不到。另外 `arbitrate("stop")` / `arbitrate("resume")` 落到意图层变「未识别」，而 `special_text()` 里同样的词能识别，**纯函数层与适配层不自洽**。

**排查过程**：逐条实跑词表，发现 `EMERGENCY_WORDS` 含裸词「急停」，而「解除急停」是它的超集字符串 ⇒ `any(w in text for w in EMERGENCY_WORDS)` 先命中，恢复分支根本没机会执行。这是**子串包含歧义**（「急停」⊂「解除急停」），不是解析错；同类问题在 `special_text()` 里也存在（它把整帧文本归一化成「急停」再交给仲裁器，两层都错）。

**根因**：用裸子串匹配做**语义分类**，且两个词表存在包含关系；两层各自实现匹配逻辑、行为不一致。规范定义：关键词分类应做「消歧 / 优先级显式化」，短词表尤其要处理一个词是另一个词子串的情形。

**解决**：抽 `_match_resume()` / `_match_estop()` 两个纯函数 helper，`_match_estop()` **先问是不是恢复、是就不算急停**，再判急停词；`special_text()` 与 `arbitrate()` 统一复用这一套，顺带把小写 `stop` / `resume` 在纯函数层也认出来。急停优先级语义不变 —— 它由 `arbitrate()` 第③步的 `estop_active` 冻结闸门保证（急停中除「恢复」外一律拒绝），词表顺序只决定「这一帧归类为急停还是恢复」。回归单测 `test_resume_word_containing_estop_is_not_misread`、`test_ascii_words_recognized_by_arbiter_itself` 锁住；验收规程 3.1 新增 `$ASR,解除急停*6F` → `$OK,RESUMED*75`。⚠️ 诚实边界：混合语句（同时含急停词与恢复词，如「急停后恢复」）按「恢复」处理，属 mock 词表已知边界，阶段 4 接真模型再细化。

**一句话面试说辞**：「短词表里一个词是另一个词的子串时，裸子串匹配会给出反向语义；我把匹配抽成两个纯函数、显式消歧，并让两层共用同一套，再用单测把『喊解除反而急停』这个危险 case 钉死。」

---

### 8. 板端实机验收里两个「看着像故障」的误判点（11-14 16:40，全程零故障）

**现象**：阶段 2 上板 `--apply` 实跑时，有两处输出容易被当成失败：① 起后台网关后立刻 `tail -20` 日志**是空的**；② SecureCRT 多行粘贴时命令回显串扰、看着像命令没执行完。但四层硬判据（应答帧 / 日志 `sent:true` / `/api/state` 脉宽变化 / 肉眼见臂动）全部照常拿到，链路零故障。

**排查过程**：① `nohup … > 日志 2>&1 &` 后进程刚 fork、日志还没落盘就去 `tail`，属**时序问题**；隔几秒再 `tail` 或 `curl` 探活即可，不是服务没起。② SecureCRT 多行粘贴会把后续行的回显挤在一起，但每条命令本身都成功执行 —— 用「板端命令一律单行、逐条粘」的既有纪律复核即可确认。另外两处非失败观察：`/api/open` 已 `ok:true` 仍按条件触发重启了 Flask（命令带 `--ik-dir /root/kunpeng_vision --eye-in-hand`、不带 `--radial-fix`，后续成功，但 `/api/ik/eye` 的 `enabled:true` 是重启前查的）；scp 首次密码 `Permission denied` 后重试成功。

**根因**：**「日志/回显的时序与呈现」不等于「功能结果」**。规范定义：进程重定向到文件的写入有缓冲与调度延迟；终端回显是本地呈现层，不改变远端命令的实际执行。判活只认功能性证据（应答帧、`/api/state` 回读、物理动作），不认日志出现的早晚。

**解决**：验收规程把「四层硬判据」写成唯一通过判据，日志只作辅助；明确 `tail` 空 / 回显串扰属非失败观察，记录进 `docs/KunPeng部署与语音网关上板验收.md` 第五节「非失败现场观察」，不追加为故障、不改动任何代码。本次抬档只依据四层证据，不因这些观察反复重试或改参。

**一句话面试说辞**：「实机验收我只认功能性证据 —— 应答帧、状态回读、物理动作；日志晚到几秒、终端回显串扰是呈现层的事，我不会把它们当成故障去调参，这也是先写死判据再上板的好处。」

---

> 本机验证留档（11-14 代码审查加固后复跑）：`python3 -B test_voice_gateway.py -v` → `Ran 38 tests in 1.4s / OK`；
> `python3 -B voice_gateway.py --self-test` → `SELF_TEST PASS`；
> `PYTHONIOENCODING=ascii` 下 `--self-test` rc=0、TCP server 能起且回 `$OK,HOME*27`（第 6 条回归）；
> `text_gateway.py` / `intent_parser.py` 自检均 rc=0（未牵动）。**均非实机证据。**

---

## 二、IoT 遥测阶段

> 来源：`排障记录-服务端与IoT.md`

> 范本：`protocol_engine_cpp/排障记录.md`。固定五段式，只记重要 / 关键 / 面试官关心的问题。
> 记录区间：2026-11-08。本地自用，不进 Git。

#### 1. 板上 `docker -v` 报「未找到命令」——先验发现是干净系统，不必照 README 走 CentOS 8 源

- **现象**：阶段 1 先验时 `docker -v` 与 `docker compose version` 都回
  `-bash: docker：未找到命令`，`docker compose up -d emqx` 随之失败。
  但 `rpm -qa | grep -i docker` 输出为空，而 `dnf list available 'docker*'` 列出
  `docker.aarch64 19.03.14-1 (oepkgs)`、`docker-engine.aarch64 2:18.09.0-355 (update)`、
  `docker-compose.noarch 1.22.0-4 (everything)`。
- **排查过程**：
  1. `iot/README.md` 第 0 节写的方案是「先 `dnf remove` 旧 docker-engine → 加华为云
     CentOS 8 docker-ce 源 → 装 docker-ce + compose V2 插件」，还用了 heredoc 多行。
  2. `rpm -qa` 为空 ⇒ **根本没有旧包冲突**，第一步 `dnf remove` 是多余的。
  3. `dnf list` 显示 openEuler 22.03 官方源（oepkgs / update / everything）自带 docker 与
     docker-compose ⇒ 舍近求远加第三方源，反而可能引入依赖冲突（不同发行版的
     `container-selinux` / libc / systemd 依赖版本不一致）。
  4. heredoc 多行也不符合本项目「终端命令零占位符、单行可整段复制」的规矩
     （小Z 习惯整段粘贴，heredoc 容易在粘贴时被终端拆行）。
- **根因**：`iot/README.md` 是**通用部署文档**，描述的是「最坏情况」（出厂自带旧版
  docker-engine 且官方源不可用）；这块板是**干净系统**，最优路径不同。把通用文档的
  「最坏情况流程」当成当前机器的实况照抄，会白做无用功还引入依赖风险。
- **解决**：先跑只读探测（`rpm -qa | grep -i docker` + `dnf list available 'docker*'`）
  判断机器实况，再决定安装路线；干净系统直接走官方源
  `dnf install -y docker docker-compose` + `systemctl start docker`，
  用 `printf | tee` 写成单行命令、不用 heredoc。
  另外发现官方源只有 `docker-compose 1.22.0`（V1，命令是带横线的 `docker-compose`，
  不是 `docker compose` V2 插件），而当时的 `docker-compose.yml` **缺 `version:` 字段**：
  V1 工具无 version 时会按 **v1 文件格式**解析，连 `services:` 都不认 ⇒ 阶段 1.1 先用
  单条 `docker run` 只起 EMQX 绕开，等 1.2 再统一处理。
  🔴 **17:36 更正（WebSearch 核实 Docker Compose v2 参考 / Compose Spec）**：
  `depends_on: condition: service_healthy` 与 `healthcheck` 本身**不是**问题 —— 它们是
  compose file **v2.1** 引入的合法语法，只要在 yml 顶部补 `version: "2.4"`
  （legacy CLI 与 v2.4 搭配的官方推荐写法）就全部生效；真正移除过 condition 的是 **v3**，
  Compose Spec 之后才回归。⇒ 1.2 的修法是「补 version」而**不是**「删 condition /
  降级 depends_on」，已在 `docker-compose.yml` 落盘。
- **一句话面试说辞**：「部署前先探测目标机实况（`rpm -qa` + `dnf list`），而不是照抄
  通用文档的最坏情况流程；这块板官方源自带 docker，就没必要引第三方源增加依赖冲突面。
  先测量再动手，比直接照教程跑更稳。」

#### 2. `--dry-run --once` 三个核心字段全 `null`——不是桥的 bug，是上游服务没跑全

- **现象**：`telemetry_bridge.py --dry-run --once` 打印
  `{"mode":"serial","baud":115200,"camera_on":false, "pulses":null,"frame_id":null,"det_count":null}`
  —— `/api/config` 那三个有真实值，六路脉宽 / 帧号 / 检测数全是 `null`。
  **输出里一条 `[warn]` 都没有**。
- **排查过程**：
  1. 四条只读诊断：`curl -X POST /api/state` → `{"ok":false,"error":"未连接"}`；
     `curl /api/detections` → `{"ok":false,"error":"<urlopen error [Errno 111] Connection refused>"}`；
     `curl :8080/health` → 空；`ps -ef | grep -E "web_control|detector_service"` → 只有 `web_control.py`。
  2. 读 `telemetry_bridge.py::fetch_json()`：只有 `URLError / OSError / ValueError`（超时、
     拒绝连接、JSON 解析失败）才打 `[warn]` ⇒ **无 warn = 三个接口都 HTTP 200 拿到了 JSON**，
     只是 body 里 `ok:false`，`build_payload()` 按设计跳过该字段。这一步把「网络超时」排除掉。
  3. 读 `web_control.py::main()` L1720-1725：serial 分支只设
     `_MODE="serial"`、`_HOST_STATE["port"]=args.serial`、`baud`，**并不创建 `_HOST`**
     （`_HOST` 全局初始 `None`）⇒ 启动后无论带不带串口参数，`/api/state` 都会走
     `_HOST is None` 返回「未连接」。
  4. `ps -ef` 实证启动命令是 `python3 web_control.py /dev/ttyUSB0 --baud 115200 ...`
     —— **带了串口位置参数**。据此**撤回**此前「5000 没带位置参数、`port` 落到默认 `COM7`」
     的推断，也撤回「不传 port 调 `/api/open` 会失败」的说法（此时 `_HOST_STATE["port"]`
     本就是 `/dev/ttyUSB0`）。带上 `{"port":"/dev/ttyUSB0"}` 仍是更稳的写法，但理由不同。
- **根因**：两个上游缺位，且都不属于桥本身：① 5000 **启动时不会自动连串口**，必须显式
  `POST /api/open`（或页面点「连接」）才把 `_HOST` 建成 `SerialHost`；② 8080 的
  `detector_service.py` 没起，`/api/detections` 代理拿到 `Connection refused`。两者都返回
  `ok:false`，桥的容错逻辑「只填 `ok:true` 的字段」正确生效 ⇒ `null` 是**预期行为**，
  不是解析错、不是崩溃。
- **解决**：**零重启补上游**——`POST /api/open` 连串口（不重启 5000，保住 `_IK_MAP` 标定值），
  再新开窗口起 8080。复验：`/api/state` → `{"ok":true,"pulses":[1580,1900,2400,700,1450,1500]}`
  （正好等于固件初始位 INIT，证明是真实串口回读而非 mock）；`/api/detections` → `ok:true`
  `frame_id:1044`；`--dry-run --once` → 全字段有值，`frame_id:1048` 在涨。
  `det_count:0` 只是当前画面没有 YOLO 能识别的物体（`detections:[]` 印证），不是故障。
- **一句话面试说辞**：「遥测桥做了分层容错：上游挂了只跳过字段、不拖垮进程。排障时先靠
  `[warn]` 的有无把『HTTP 200 但 `ok:false`』和『网络超时』分开，再逐层定位到串口连接和
  8080 服务，最后用 `/api/open` 零重启恢复，没有动主链路一行代码。」

#### 3. `docker pull emqx:5.8.8` 报 `Client.Timeout exceeded`——Docker Hub 直连不通，改镜像加速

- **现象**：`docker pull emqx:5.8.8` 与 `docker run ... emqx:5.8.8` 都报
  `Get https://registry-1.docker.io/v2/: net/http: request canceled ... Client.Timeout exceeded`；
  `docker ps` 为空，无残留容器。
- **排查过程**：
  1. 联网核实 2026 年现状：`iot/README.md` 里写的 USTC `docker.mirrors.ustc.edu.cn`
     （2024-06 停 Docker Hub 同步）与网易 `hub-mirror.c.163.com`（已停同步）**均已失效**，
     照抄只会再超时一次。
  2. 对四个候选源逐个 `curl -s --max-time 5 -o /dev/null -w "%{http_code}" https://xxx/v2/`：
     `docker.m.daocloud.io` / `docker.1ms.run` / `docker.xuanyuan.me` 全回 **401**，
     `dockerproxy.com` 回 **000**。
     🔴 `401` 不是错误——Registry v2 协议对未带认证_token 的 `/v2/` 探测本来就回
     `401 Unauthorized`，说明链路通；`000` 才是超时/不通。
  3. `cat /etc/docker/daemon.json` → `NO_DAEMON_JSON`，确认没有现成配置，写入不会覆盖别人的设置。
- **根因**：板子 Wi-Fi 出口到 Docker Hub 官方 registry 直连超时，而 daemon 没配
  `registry-mirrors`，只会硬磕官方源；旧加速站又已停服。
- **解决**：**先探活再写配置**——`printf` 单行写
  `{"registry-mirrors":["https://docker.m.daocloud.io"]}` 到 `/etc/docker/daemon.json`
  → `systemctl restart docker` → `docker pull emqx:5.8.8` 四层 `Pull complete`、
  `Status: Downloaded newer image` → 单条 `docker run -d --name emqx -p 1883:1883 -p 18083:18083
  emqx:5.8.8` 起容器（当时 yml 缺 `version:`，V1 工具按 v1 格式解析不了，故先绕开；
  1.2 已补 `version: "2.4"`，改由 compose 统一托管四个容器）。
- **一句话面试说辞**：「镜像拉不下来先分清是网络还是配置：用 `curl -w %{http_code}` 探
  `/v2/`，401 算通、000 算不通，只把探活的源写进 `registry-mirrors`；不照抄教程里那份
  已经停服的加速清单，也不盲目堆多个源拖慢每次 pull。」

#### 4. EMQX 容器 `Exited (1)`：`Failed to create thread: Operation not permitted`——旧 Docker seccomp 拦了 `clone3`

- **现象**：`docker run -d` 回显容器 ID 但 `docker ps` 只有表头；`docker ps -a` 显示
  `Exited (1) 10 minutes ago`；`ss -lntp | grep 1883` → `NO_1883`；paho 订阅与 bridge
  正式跑都在 `client.connect()` 抛 `ConnectionRefusedError [Errno 111]`。
  `docker logs` 首行 `ERROR: Failed to create thread: Operation not permitted (1)`。
- **排查过程**：
  1. `docker inspect -f '{{.State.Status}} exit={{.State.ExitCode}} err={{.State.Error}}'`
     → `exited exit=1 err=` ⇒ `err` 为空，**是容器内主进程自己崩了**，不是 OCI/runc 创建失败。
  2. 日志里 `Please ensure it is running on the correct platform: arch aarch64 / os debian13 /
     erlang 26.2.5.14` 与 `Required dependencies: openssl-1.1.1 (libcrypto), libncurses and
     libatomic1` 是 EMQX 启动脚本**崩溃后打印的平台诊断清单**，不是「架构不匹配」或「缺库」
     实证：架构 `aarch64` 与板子一致、镜像能 pull 能创建容器；真缺库会报
     `error while loading shared libraries: libcrypto.so.1.1`。⇒ 别去装 openssl-1.1.1
     （Debian 13 里也没有），那是死路。唯一有效错误是首行。
  3. 联网核实（moby / docker-library issue、StackOverflow RabbitMQ 同款报错）：
     **Docker < 20.10.10 默认 seccomp profile 不含 `clone3`**；而 glibc ≥ 2.34
     （Debian 13 / Ubuntu 22.04+）的 `pthread_create()` 改用 `clone3(2)`；旧 profile 对
     未知 syscall 返回 `SCMP_ACT_ERRNO(EPERM)` 而非 `ENOSYS`，glibc 不会 fallback 到老
     `clone()`，建线程直接失败。Erlang/OTP 的 `erts` 启动时创建调度线程失败即打印该行并退出
     —— RabbitMQ（同为 Erlang）在 Docker 19.03 上报过一模一样的错，升 20.10+/24 或加
     `seccomp=unconfined` 即好。本机 `docker 19.03.14` 正好落在受影响区间。
- **根因**：**宿主 Docker 引擎太老（19.03.14）**，默认 seccomp-BPF 白名单早于 `clone3`；
  容器里 EMQX 5.8.8 基于 Debian 13 / glibc ≥ 2.34 / Erlang 26，创建线程触发被拦的
  `clone3` → `EPERM` → `erts` 退出 → 容器 `Exited (1)` → 1883 无人监听。
  与镜像架构、缺库、MQTT 配置均无关。
- **解决**：本地实验环境用最小改动重建——`docker rm -f emqx`（该容器是刚创建、无数据的
  Exited 测试容器，删掉安全）→
  `docker run -d --name emqx --security-opt seccomp=unconfined -p 1883:1883 -p 18083:18083 emqx:5.8.8`
  → **`sleep 5 && docker ps -a` 复核 STATUS 必须是 `Up`**、再看 `ss -lntp | grep 1883`。
  规范正解是升级 Docker ≥ 20.10.10，或自定义 profile 只 `SCMP_ACT_ALLOW` 放行
  `clone3`/`rseq`/`faccessat2`（保持最小权限）；但 openEuler 官方源只有 19.03.14，
  引第三方源会带来依赖冲突，本地实验用 `seccomp=unconfined` 是合理取舍，
  须写明它撤掉了该容器的系统调用过滤层。若仍 Exited 就转官方源 `mosquitto`
  绕开 Docker，不再打补丁。🔴 1.2 起四容器时**每个服务都要带 `security_opt`**，
  否则 eKuiper / Grafana 等新镜像同样会崩。
- **一句话面试说辞**：「容器 `Exited (1)` 先看 `docker inspect` 的 `err` 是否为空，判定是
  进程自崩而非 runtime 失败；再从 `Failed to create thread: EPERM` 追到旧 Docker seccomp
  白名单缺 `clone3` 与新 glibc 的兼容问题。日志里那串 arch / dependencies 是崩溃后的
  诊断清单，不是根因。修的时候讲清『绕开过滤』与『升级引擎』的区别和安全取舍。」

### 第 5 条：`./deploy.sh` 起 influxdb 报 `0.0.0.0:8086: bind: address already in use`

- **现象**：1.2b 第 4 组 `docker-compose up -d` 时 `Creating emqx ... done` / `Creating ekuiper ... done`，
  到 `Creating influxdb ... error`，报
  `driver failed programming external connectivity ... Error starting userland proxy: listen tcp 0.0.0.0:8086: bind: address already in use`；
  grafana 未创建，eKuiper 流/规则也还没建。
- **排查过程**：先排除镜像 / seccomp / compose 格式（第 0~3 组已实证 `docker-compose config` 成功、
  四个 arm64 镜像全拉成功）。再按「谁占端口」做三条只读诊断：
  `ss -lntp | grep 8086` → `LISTEN 127.0.0.1:8086 users:(("nginx",pid=1143/1144/1145))`；
  `docker ps -a` → 只有 1.1 手动起的旧 emqx 与本次 `Created` 的 influxdb，无其他容器；
  `ps -ef | grep -iE "influx|docker-proxy"` → 只有 18083 / 1883 / 9081 三个 proxy，**无 8086 残留**。
  再 `ps -fp 1143,1144,1145` → `devkit` 用户跑 `/opt/DevKit/tools/nginx/sbin/nginx`（PPID=1）；
  `systemctl status nginx` 无输出 ⇒ 板上 DevKit 工具链自带服务、无 nginx.service、非 systemd 托管。
- **根因**：Linux `bind(2)` 中通配地址 `INADDR_ANY (0.0.0.0)` 与具体地址 `127.0.0.1` 绑定同一端口互斥，
  返回 `EADDRINUSE`。DevKit nginx 已占 `127.0.0.1:8086`，Docker userland proxy 默认绑
  `0.0.0.0:8086`，必然冲突。既不是残留容器、也不是 proxy 残留 ⇒ `docker rm -f influxdb` 与
  `systemctl restart docker` 都无效；停 nginx 则要动系统工具链且无 systemd 可干净回滚（B 方案否决）。
- **解决**：只改**宿主机端口映射**：`docker-compose.yml` 的 `"8086:8086"` → `"8087:8086"`，
  README 端口列表与第 4 节第 7 条排障同步。容器内 `http://influxdb:8086` 三处
  （eKuiper sink / Grafana datasource / Telegraf 降级桥）与 `deploy.sh` 全不动 ——
  容器间走 `iot` bridge 网络的服务名，与宿主机门牌号无关。
  ✅ 板上重跑实证：`Recreating influxdb ... done` / `Creating grafana ... done`，四容器全 `Up`，
  influxdb `0.0.0.0:8087->8086/tcp (healthy)`、grafana `0.0.0.0:3000->3000/tcp`，DevKit nginx 未动。
- **一句话面试说辞**：「容器起不来先分清是端口被占还是进程崩了。我用 `ss -lntp` 定位到占用者，
  发现是系统自带 nginx 绑了回环地址，和 Docker 默认绑 `0.0.0.0` 冲突（通配地址与具体地址互斥）。
  我没有去停系统服务，而是只改了宿主机端口映射，容器内部地址一行没动 —— 改动面最小、可回滚。」

---

### 第 6 条：eKuiper 建流报 `found "WITH", expected lparen after stream name`（error 3000），且脚本静默失败

- **现象**：1.2b 首次真正跑 `./deploy.sh` 建流/建规则时，
  `POST /streams` 返回 `{"error":3000,"message":"Stream command error: found \"WITH\", expected lparen after stream name."}`；
  连带 `POST /rules` → `error 2101 fail to get stream device_stream`、
  `GET /rules/to_influx` → `error 1002 Rule to_influx is not found`。
  但脚本一路跑到底还打印「==> 完成!」，看上去像成功。
- **排查过程**：先确认不是 MQTT/EMQX 问题（broker `Up (healthy)`、bridge 能连 1883），报错文本指向 SQL 解析器本身；
  对照 eKuiper 官方文档（v2.4 内存源 `CREATE TABLE memoryTableDemo () WITH (...)`、
  v1.13 规则管道 `{"sql":"create stream demo () WITH (DATASOURCE=..., FORMAT=...)"}`），发现流名后必须有括号。
- **根因**：两层。
  ① `deploy.sh` 写 `CREATE STREAM device_stream WITH (...)`，**缺流名后的 `()`**。eKuiper DDL 是
  `CREATE STREAM name (列定义) WITH (...)`，不声明字段（schemaless）时必须写**空括号 `()`**。
  该写法从旧 IoT 脚手架继承，1.1 只单条 `docker run` 起 EMQX、从未执行建流，所以潜伏到 1.2b 才暴露。
  ② REST 失败返回的是 **HTTP 200 + 带 `"error"` 的 JSON**，curl 退出码仍为 0，`set -e` 完全拦不住
  ⇒ 原脚本静默失败。
- **解决**：建流 SQL 改 `CREATE STREAM device_stream () WITH (...)`；并按小Z 选的 B 方案给建流/建规则
  两处 curl 加 `grep -q '"error"'` 响应体检查 + `|| exit 1` + 成功 `[OK]` 回显。
  ✅ 板上实证：重跑后 `Stream device_stream is created.` + `[OK] 流 device_stream 已创建`；
  规则阶段再失败时脚本立刻打印 `建规则失败, 已停止` 并 `exit 1`，错误检查确实生效。
- **一句话面试说辞**：「脚本『跑完了』不等于『成功了』。eKuiper 的 REST 失败是 HTTP 200 带 error 字段，
  curl 退出码仍是 0，`set -e` 拦不住。我改成显式解析响应体判断 error，并让成功路径也打印 [OK]，
  失败当场停、成功有回执 —— 顺手把『静默失败』这个更隐蔽的问题一起修了。」

---

### 第 7 条：eKuiper influx2 sink 报 `'tags' expected a map, got 'string'`（error 2201）

- **现象**：第 6 条把建流修好后，`POST /rules` 返回
  `{"error":2201,"message":"error configuring influx2 sink: 1 error(s) decoding:\n\n* 'tags' expected a map, got 'string'"}`；
  新加的响应体错误检查立刻打印 `建规则失败, 已停止` 并 `exit 1`（B 方案实证生效，没再静默失败）。
- **排查过程**：报错指向 **sink 配置解码阶段的类型不匹配**，与 SQL / MQTT / InfluxDB 连通性无关。
  对照官方：v2.4 influx2 sink 属性表写 `tags` 格式 `{"tag1":"value1"}`、value 支持 dataTemplate
  `{{.temperature}}`；rekuiper 内置 influx2 文档示例 `"tags": {"location": "{{.location}}"}`；
  火山引擎 eKuiper 实践同样是对象 + `{{.__meta.deviceName}}`，且强调 key-value 必须都是 String。
- **根因**：`/tmp/rule.json` 里写的是 `"tags": "{\"device\":\"{{.deviceId}}\"}"`
  —— 一个 **JSON 字符串**（把对象又序列化了一层），而 eKuiper 2.1.0 的 influx2 sink 用
  mapstructure 解码，要求 `tags` 是 **map[string]string 对象**。
  ⚠️ 该写法源自官方 v2.4 文档 **Sample usage**（那里也把 tags 写成 `"{\"tag1\":\"value1\"}"`），
  **文档示例与 2.1.0 实现不一致/版本差异** —— 照抄官方示例不等于能在当前版本跑通。
- **解决**：改成 `"tags": {"device": "{{.deviceId}}"}`（去掉外层引号；模板 `{{.deviceId}}` 保留，
  官方明确支持 dataTemplate）。先在板上临时改 `/tmp/rule.json` 验证：
  `POST /rules` → `Rule to_influx was created successfully.`，
  `GET /rules/to_influx` 回显 `"tags": {"device": "{{.deviceId}}"}` ⇒ 修法实证通过；
  再按小Z `写` 授权把同一行同步回 `deploy.sh` 的 heredoc，保证脚本可复现。
  PC 侧从 `deploy.sh` 抽出 heredoc 用 `json.loads` 校验：`TAGS_TYPE dict` / `FIELDS_COUNT 11`。
- **一句话面试说辞**：「这是配置类型不匹配：sink 要 map，我给的是被二次序列化的 JSON 字符串。
  有意思的是这写法是照抄官方文档示例的，但当前版本实现只收对象 —— 所以我先看实现报错、
  再对照多份文档确认类型，最后做最小改动并回读规则定义验证，而不是靠猜着改。」

---

### 第 8 条：规则 `created successfully` 却 `stopped by error` —— firewalld nftables 兜底 REJECT 把容器流量全拒了

- **现象**：1.2b 第 6 组之后，`POST /rules` 回 `Rule to_influx was created successfully.`，
  但 `GET /rules/to_influx/status` 是 `"status":"stopped by error"`，message
  `Get "http://influxdb:8086/ping": dial tcp 172.18.0.5:8086: connect: no route to host`；
  `nextStartTimestamp: 0`（不自愈）、`records_in_total` 全 0、
  `sink_influx2_1_0_exceptions_total: 1`。`docker logs --tail 40 ekuiper` 显示规则先
  `transit to state running`、同一秒 sink 探活报错就 `closing Rule for error`。
  四容器 `docker ps -a` 全 `Up`（influxdb `healthy`、`0.0.0.0:8087->8086/tcp`），
  5000 / 8080 都回 200，InfluxDB Flux 查 `arm_telemetry` 返回空。
  ⇒ **规则静态创建成功 ≠ 规则在跑**；bridge 发的消息一条都没被消费，数据一条没进库。
- **排查过程**（四层假设依次证伪，全程只读、没改一行配置）：
  1. 先排除已修过的三处：8086 冲突（已改 8087、容器 healthy）、`tags` 类型（规则定义已是
     map、静态创建通过）、token（报错在 TCP dial 阶段，还没到鉴权；且 Flux 查询没报 401）。
  2. **假设 1「influxdb recreate 后 eKuiper `/etc/hosts` 旧 IP」→ 11:02 证伪**：
     `docker exec ekuiper cat /etc/hosts` 里**根本没有 influxdb 条目**（解析走 Docker embedded
     DNS 127.0.0.11）；`docker inspect influxdb` 真 IP `172.18.0.5` 与报错里的 IP 完全一致 ⇒
     DNS 正确。同时 `docker exec ekuiper wget http://influxdb:8086/ping` 复现 `WGET_EXIT=4`，
     `docker exec influxdb influx ping` 却 `OK` ⇒ 跨容器 TCP 确实不通。
  3. **假设 2「eKuiper 容器 veth 专属异常」→ 11:07 证伪**：宿主机直连
     `http://172.18.0.5:8086/ping` 得 `204`（走 OUTPUT，不经 FORWARD），但
     `docker exec grafana wget` 同样 `Host is unreachable` ⇒ 全局问题、非 eKuiper 专属
     （`force-recreate ekuiper` 作废）。`ekuiper:/proc/net/arp` 里 `172.18.0.5` 有完整 MAC
     `02:42:ac:12:00:05` Flags `0x2`，`/proc/net/route` 默认路由 `172.18.0.1` +
     `172.18.0.0/16` 直连全对 ⇒ 二层 / ARP / 路由 / DNS 全部正常。
  4. **假设 3「iptables FORWARD 拦截」→ 11:10 证伪**：`iptables -S FORWARD` 实证
     `-P FORWARD ACCEPT`，Docker 链齐全（DOCKER-USER / ISOLATION / DOCKER），且
     `-i br-e0c9e4d3012e -o br-e0c9e4d3012e -j ACCEPT` 同网桥明确放行、全链无 REJECT/DROP；
     `iptables -L FORWARD -n -v` 默认策略 0 包。`ip_forward=1`、
     `bridge-nf-call-iptables=1`、`br-e0c9e4d3012e` UP `172.18.0.1/16` 全正常。
  5. **假设 4「改 sink addr 走宿主机网关绕行」→ 11:13 证伪**：`GW_EXIT=4`
     （`172.18.0.1:8087`）、`GW_GRAFANA_EXIT=1`、`WLAN_EXIT=4`（`10.160.120.1:8087`）
     ⇒ 不是「容器→容器」，**容器发出的所有三层流量都被拒**。
  6. 🔴 **抓到铁证**：`nft list ruleset | grep` 出现 **`reject with icmpx admin-prohibited` ×4**
     （另有 `ct state invalid drop`）；`iptables -S | grep DOCKER-*` 证实 Docker 自定义链内部
     干净（`DOCKER-USER -j RETURN`；ISOLATION-STAGE-2 的 DROP 只管跨网桥）。⇒ 拦截不在
     iptables-legacy 视图里，而在 **firewalld 的 nftables `inet` 表**。
- **根因**：openEuler 的 `firewalld` 走 **nftables 后端**，其 `inet` 表独立 hook 于
  iptables-legacy（所以 `iptables -S FORWARD` 全放行却仍被拒）。`--get-active-zones` 只有
  `virbr0`(libvirt) 与 `wlan0`(public)，**docker 网桥 `br-e0c9e4d3012e` 未被任何 zone 纳管**
  ⇒ 容器发出的包落到 zone 链 `target: default` 的兜底 `reject with icmpx admin-prohibited`，
  内核转成 `EHOSTUNREACH`，表现就是 `no route to host` / `Host is unreachable`。
  宿主机→容器走 OUTPUT 不经该链故 `204`；ARP 是二层不受影响故有完整 MAC —— 这两条
  「看着像通」的证据正好解释了为什么前四层排查都指向别处。
  ⚠️ 四容器 `Up (healthy)` ≠ 容器间互通：influxdb 的 healthcheck 是容器内 `influx ping`
  走 127.0.0.1，根本不跨网桥。
- **解决**：把网桥纳入 `trusted` zone（`target: ACCEPT`，不再落兜底 REJECT）：
  **`firewall-cmd --zone=trusted --add-interface=br-e0c9e4d3012e`**。
  用 **runtime、不加 `--permanent`、不 `--reload`** ⇒ 立即生效、不动 Docker iptables 链、
  不重启容器、可用 `--remove-interface` 回滚；代价是**重启或网络重建后失效**（接口名会变），
  收口时再决定固化方式。备选 `--zone=trusted --add-source=172.18.0.0/16`。
  ✅ 11:15 板上实证：`success` / `trusted (active)` `target: ACCEPT` /
  `interfaces: br-e0c9e4d3012e`；`docker exec ekuiper wget http://influxdb:8086/ping` →
  **`EKUIPER_EXIT=0`**、`docker exec grafana wget` → **`GRAFANA_EXIT=0`**，容器间网络恢复。
  🔴 **15:03 复发实证**：14:40 `docker restart grafana` 之后，`trusted --list-all` 的
  `interfaces:` 与 `sources:` **全空**（runtime 绑定丢失），而 `ip -4 addr` 显示网桥名仍是
  `br-e0c9e4d3012e` **未变** ⇒ 是「绑定失效」不是「接口改名」。同期 Grafana 报
  `no route to host`、规则 `stopped by error`。重新 `--add-interface` 后双容器 `wget` 又
  `EXIT=0` ⇒ **「重启/网络重建后失效」从预警变成实证**，收口必须 `--permanent` 或写进
  `deploy.sh` 固化，否则每次重启都会复发。
  另：`public` zone 只放行 5000/8080，PC 开 Grafana `:3000` 前需
  `firewall-cmd --zone=public --add-port=3000/tcp`。
- **一句话面试说辞**：「规则创建成功但 `stopped by error`，我没去猜着改配置，而是按
  DNS → 二层/ARP → 路由 → iptables FORWARD → nftables 逐层做对照实验，四次假设都被
  实证证伪，最后用 `nft list ruleset` 抓到 firewalld 兜底 `icmpx admin-prohibited`。
  关键是分清 `iptables -S FORWARD` 只是 compat 视图，openEuler 上 firewalld 走的是
  nftables；以及『容器 healthy』和『容器之间能通信』根本是两回事。修法只做一条
  runtime 的 trusted zone 绑定，不动 Docker 链、不重启容器、可回滚。」

---

### 第 9 条：规则 running、数据也进来了，却一条没写进 InfluxDB —— `tags` 的 dataTemplate 在运行时求值失败

- **现象**：第 8 条把 firewalld 修好后，`POST /rules/to_influx/restart` →
  `Rule to_influx was restarted`，`GET /rules/to_influx/status` →
  `"status":"running"` / `message:""`，`source_...connection_status: 1` 且
  **`sink_influx2_1_0_connection_status: 1`**（sink 探活成功）。bridge 重跑后 payload 全是
  真实值（`j1~j6` = `[1580,1900,2400,700,1450,1500]`、`frame_id` 116248→120896 在涨、
  `det_count` 0/1/2 变化）。但 Flux 查 `arm_telemetry` 仍为空。`/status` 里
  `source/decoder/project` 各 **18** 条、`sink_log_0_0_records_in_total: 18`，而
  **`op_influx2_1_0_transform_0_records_out_total: 0`、`exceptions_total: 18`、
  `sink_influx2_1_0_records_in_total: 0`** ⇒ 数据流到 influx2 sink 之前全部正常，
  断在写库前最后一步。
- **排查过程**：`op_influx2_1_0_transform_0_last_exception` 给出铁证：
  `fail to calculate props {{.deviceId}} through data [map[baud:115200 ...
  deviceId:kunpeng_01 frame_id:118272 j1:1580 ...]] with dataTemplate for error template:
  sink:1:2: executing "sink" at <.deviceId>: can't evaluate field deviceId in type
  []map[string]interface {}`。
  ① 异常里的 data map 带着**真实** `deviceId` / `frame_id` / `j1~j6` ⇒ 不是上游没数据、
     不是 SQL 投影丢字段；
  ② 报错指向 `dataTemplate` 求值而不是网络或鉴权 ⇒ 与第 7 条（`tags` 类型）是**两层**问题；
  ③ 对照第 7 条：那条把 `"tags": "{\"device\":...}"` 改成 `{"device":"{{.deviceId}}"}`，
     解决的是 **sink 配置解码阶段的静态类型**（map 而非 string），所以规则能
     `created successfully`；但 `{{.deviceId}}` 是**运行时**才求值的模板，静态校验根本不执行它。
- **根因**：eKuiper 2.1.0 的 influx2 sink 用 Go `text/template` 求 `tags` 里的 dataTemplate 时，
  传给模板的数据是 **`[]map[string]interface{}`（一批记录的切片）**，而不是单条 `map`；
  模板表达式 `.deviceId` 直接在切片上取字段，Go 模板引擎报
  `can't evaluate field deviceId in type []map[string]interface {}` ⇒ 每条事件在
  `transform` 阶段就被拒（18 条全异常、`records_out_total: 0`），sink 一条都没收到。
  ⚠️ 这是**第二次「静态创建成功 ≠ 能写数据」**（第一次见第 8 条：规则静态创建通过，
  但运行时 sink 探活 `no route to host` 而 `stopped by error`）。两次都说明：`POST /rules`
  成功只代表配置语法/类型过关，运行态必须看 `/status` 的 `exceptions_total` 与
  `records_out_total`。
- **解决**：把 `tags` 的值改成**静态字符串**，去掉模板：
  `"tags": {"device": "kunpeng_01"}`。
  它仍是 **map 对象**（第 7 条的静态类型约束继续满足），但不含 `{{ }}` ⇒ 不再触发模板引擎；
  而 `deviceId` 本来就是发布端常量 `DEVICE_ID = "kunpeng_01"`，动态模板没有任何收益，
  去掉是最小且可回滚的改动。板上临时改 `/tmp/rule.json`（不碰项目文件）→
  `DELETE /rules/to_influx` → `POST /rules -d @/tmp/rule.json` 重建。
  ✅ 实证：`Rule to_influx was created successfully.`；15 秒后 `/status` →
  **`op_influx2_1_0_transform_0_exceptions_total: 0`、`records_out_total: 11`、
  `sink_influx2_1_0_records_in_total: 11`、`sink_influx2_1_0_exceptions_total: 0`**
  ⇒ 模板异常归零、11 条真实记录进入 influx2 sink 且写入零异常（同时排除了「token 无效」
  的担心：sink 用规则里的真值写成功了）。
  另：Flux 查询那条 curl 因 `sed` 的 `\(` `\)` 在复制时被吃掉而报
  `invalid reference \1`，属命令粘贴转义坑（零副作用），改用无反斜杠的提取方式即可。
- **一句话面试说辞**：「规则能创建、状态 running、上游计数都在涨，数据却没入库 —— 我没去调
  sink 参数，而是看 `/status` 里每个算子的 `exceptions_total` 和 `records_out_total`，
  一眼定位到 `transform` 阶段 18 条全异常。根因是配置里的 `{{.deviceId}}` 属于运行时模板求值，
  静态校验根本不执行，而 eKuiper 传给模板的是切片不是单条记录。修法只是把无收益的动态模板
  换成常量字符串，改动最小、可回滚。这也让我记住：『静态校验通过』和『运行时能写数据』
  是两层，验收要看运行态计数而不是 API 返回值。」

---

### 第 10 条：Grafana 数据源列表为空 `[]` —— provisioning 目录权限 600，容器内非 root 用户读不了

- **现象**：1.2b 最后一步，`GET /api/datasources` 返回 `[]`，Explore 下拉只有
  `-- Grafana --`（内置 TestData）与 `-- Mixed --`，**没有 InfluxDB** ⇒ 看板画不出六路曲线。
  而 InfluxDB 侧 `-24h` Flux 已查到真实 `arm_telemetry` 点（11 个 field 全齐）⇒ 数据没问题，
  问题只在 Grafana 没把 provision 文件读进去。
- **排查过程**：
  1. 先排除「InfluxDB 没数据 / token 无效 / firewalld 拦截」：`-24h` Flux 返回真实 CSV，
     11:15 trusted zone 修法后 `GRAFANA_EXIT=0`（容器间互通），数据源定义里
     URL / organization / bucket 全对。
  2. 只读诊断四连：宿主机 `ls -la ~/iot/grafana/provisioning/datasources/` → 文件在、
     时间 `11月 8 16:01`（**比容器启动早一天**），但目录 `drwx---rwx root:root`(707)、
     文件 `-rw------- 1 root root 311`(**600**)；
     `docker exec grafana ls/cat /etc/grafana/provisioning/datasources/` → **`Permission denied`**；
     `docker logs grafana | grep -i provisioning` → 两次启动（UTC 02:06、06:16）都有
     `logger=provisioning.datasources level=error msg="can't read datasource provisioning files
     from directory" error="open /etc/grafana/provisioning/datasources: permission denied"`，
     plugins / alerting / dashboards 同样被拒。
  3. **撤回「文件在容器启动之后才传（时序问题）」假设**：文件 11-08 16:01 就在、容器 14:16 才起，
     时序无误。`docker exec grafana id` → **`uid=472(grafana) gid=0(root) groups=0(root)`**，
     实证容器内用户 GID 命中 root 组。
- **根因**：官方 `grafana/grafana:11.3.0` 镜像以**非 root 用户**（uid 472）运行，而 Docker
  bind mount（compose L114 `./grafana/provisioning:/etc/grafana/provisioning`）**保留宿主机
  UID/GID/mode**；文件从 PC 经 scp 传上来被设成 `root:root 600`。Linux DAC 权限检查按
  **owner → group → other 顺序、命中即停不叠加**：目录 `drwx---rwx` 的 group 位是 `---`，
  而 grafana 用户 `gid=0(root)` 正好命中 group ⇒ 连 other 的 `rwx` 都用不上；文件 `600`
  更直接决定非 owner 读不了。Grafana 启动时扫描 provisioning 失败**不中断**，于是静默地
  一个数据源都没注册（日志有 error，但服务照常 `Up`、页面照常打开）。
  ⇒ 第 4 次「配置看着对 / 容器 healthy ≠ 运行时生效」。
- **解决**：最小、可逆、只动权限不改一个字节内容 ——
  `chmod -R a+rX ~/iot/grafana/provisioning/`（文件 `600→644`、目录 `707→757`，补 group/other 读位）
  → `docker exec grafana cat /etc/grafana/provisioning/datasources/influx.yml > /dev/null && echo READ_OK`
  短路验证可读 → `docker restart grafana`（**provisioning 只在容器启动时扫描**，不重启不生效；
  单容器重启不重建 `iot` 网桥 ⇒ firewalld trusted zone 绑定继续有效）
  → `GET /api/datasources` 返回
  `[{"id":1,"name":"InfluxDB","type":"influxdb","url":"http://influxdb:8086","isDefault":true,
  "jsonData":{"version":"Flux","organization":"robotics","defaultBucket":"telemetry"}}]`
  ⇒ **修法实证通过**，Explore 下拉出现 `InfluxDB`、Flux 编辑器生效。
  备选 `chown -R 472:472`（需先 `docker exec grafana id` 确认 UID）。根子上应在 scp 上传流程里
  就统一 `chmod`，别等页面空白才回头查。
- **一句话面试说辞**：「数据源列表是空的，我没去页面上手动补一个了事，而是先看 Grafana 启动日志
  —— 它明确写了 `can't read provisioning directory: permission denied`。再查到容器里跑的是
  uid 472、gid 0，而 scp 传上来的文件是 `root:root 600`，Linux 权限按 owner→group→other
  匹配即停、命中 group 的 `---` 就不再看 other，所以读不到。修法只是 `chmod` 补读权限加一次
  重启让 provisioning 重新扫描，不换镜像、不改配置、不重启网桥。值得一提的是：容器 `Up`、
  页面能打开、日志里那条 error 不致命 —— 这三层『看着正常』叠起来最容易让人误判成别的问题。」

---

### 第 11 条：配置/网络/鉴权全对，sink 却 100% 异常 —— InfluxDB field 类型冲突（null 被写成 string）

- **现象**：15:03 修好 firewalld 复发、规则 `restart` 成功 `running`、
  `sink_influx2_1_0_connection_status: 1`（sink 探活成功）之后，
  `sink_influx2_1_0_records_in_total` 与 `exceptions_total` **同步增长**（1→13→74→132，
  异常数永远等于记录数）、`records_out_total: 0`，`-30m` Flux 直查空 body ⇒ 数据一条没写进库。
  这是 1.2b 里**第 7 次「某层通过 ≠ 端到端」**。
- **排查过程**：先看异常在哪个算子 —— 11:35 那次是 `op_influx2_1_0_transform_0_*`（transform 阶段、
  sink `records_in=0`，根因是 `tags` 模板），这次是 `sink_influx2_1_0_exceptions_total` 本身且
  `records_in` 在涨 ⇒ 数据已过 transform、卡在**写库阶段**，不能照搬上一条修法。
  再按「先排除旧嫌疑」四条只读诊断：
  ① `GET /rules/to_influx` 回显 `tags: {"device": "kunpeng_01"}` ⇒ 已是静态、没被改回模板版；
  ② 用规则里的 token 查 `-24h` Flux 返回 11:45 的真实 CSV（`j1=1580`）⇒ token 有效、排除 unauthorized；
  ③ 双容器 `wget` `EXIT=0` ⇒ 网络层已恢复；
  ④ 抓 `sink_influx2_1_0_last_exception` 全文，得到铁证：
  `partial write: field type conflict: input field "j1" on measurement "arm_telemetry" is type string, already exists as type float dropped=1`；
  `docker logs ekuiper` 同一文本反复出现。最后看 `sink result for rule to_influx` 里的 payload：
  **`j1~j6` 全 `null`**（`baud` / `camera_on` / `frame_id` / `det_count` 都正常）⇒ 问题只在数据内容。
- **根因**：三层叠加。
  ① 上游：`/api/state` 返回「未连接」（串口断了），`telemetry_bridge.py::build_payload()` 在
     `pulses` 不是 6 路时把 `j1~j6` 填成 JSON `null` —— 这一步「进程不崩」是对的，但**不是无害容错**；
  ② 中间：eKuiper influx2 sink **不跳过 nil**，把 `null` 序列化成 **string** 写进 line protocol
     （带引号的 value 就是 string 类型）；
  ③ 存储：InfluxDB 2.x 每个 `measurement + field` 的类型在**首次写入时锁定**，
     11:45 的历史点 `j1=1580` 已把该 field 锁成 **float**，不同类型返回 422
     `unprocessable entity` ⇒ `partial write: field type conflict`、每批 `dropped=1`、整批写不出去。
  ⇒ 不是 token、不是 `tags`、不是网络，是**数据建模层**一条脏数据把整批写库拖垮；
  也解释了为什么 11:46 能写成功（那时 bridge 拿到真实数字）。
  🔴 **明确更正 11:35 起「null 点是已知容错、不是 bug」的判断**：进程不崩是对的，
  但会拖垮写库，必须治本。
- **解决**：临时恢复 + **治本已落盘**两步。
  ✅ 临时（15:13 实证）：`POST /api/open {"port":"/dev/ttyUSB0"}` 零重启恢复串口
  ⇒ bridge 重新拿到真实六路、null 点消失 ⇒ 类型冲突消失、写入成功，
  15:12:57 起 Grafana Table 有真实点、六路曲线出来，端到端闭环成立。
  （`sleep 15` 查 status 时累计异常仍 132/132 不矛盾：那是累计值，
  串口恢复过渡几秒后新增的正常点照常写入；`dropped=1` 每批只丢冲突的那一个 point。）
  ✅ **治本（11-09 小Z 回「写」授权后已落盘；PC 自检 28 项 ALL PASS）**：
  ① `iot/telemetry_bridge.py` 新增纯函数 `is_publishable()`：六路 `j1~j6` 必须全是真数字
     （`isinstance(..., (int, float))`，并**显式排除 `bool`** —— bool 是 int 子类会被误判成数字）；
     正式 MQTT 模式下不完整就打印 `[warn] 六路脉宽不完整，跳过本次发布` 并**整条跳过 publish**；
     `--dry-run` 仍打印 payload 供诊断；`self_test()` 补「正常 / 断连 / 畸形 / bool 混值」四类断言。
  ② `iot/deploy.sh` 四处收口：每次 `up -d` 后**动态取当前 `br-*` 网桥名重绑 trusted zone**
     （治第 8 条 runtime 绑定重启丢失）、清理顺序改「先删规则再删流」、
     `tags` 从 `{{.deviceId}}` 改静态 `{"device": "kunpeng_01"}`（治第 9 条）、
     状态查询改 `GET /rules/to_influx/status`（describe 不含 status）。
  ③ `iot/README.md` 同步 1.2b 实证口径、Flux 改 `v.timeRangeStart/Stop`、
     补齐第 8~11 条速查、`:3000` 放行与 provisioning `chmod` 前置、1.2b 收口三行法。
  ⏭️ 治本代码已在 PC 侧落盘且 `bash -n` / `--self-test` / 规则 JSON / `git diff --check` 全绿；
     **仍需 scp 上板、重跑 `./deploy.sh` 与 `telemetry_bridge.py` 做一次实机复验**才算板上闭环。
     （不额外给规则 SQL 加 `WHERE j1 IS NOT NULL`：发布端已挡住脏数据，双保险非必需，
       且 eKuiper SQL 对 nil 的语义未核实，不引入未验证语法。）
  🔴 **不推荐清库 / 换 measurement 绕开** —— 会丢掉 11:45 已实证的真实数据。
- **一句话面试说辞**：「规则 running、网络通、鉴权也过了，写库却 100% 失败。
  我没去调 sink 参数，而是先分清异常在 transform 还是 sink 算子，再逐层排除 tags / token /
  网络三个旧嫌疑，最后从 `last_exception` 的 `field type conflict` 追到一条 null 数据：
  InfluxDB 2.x 的 field 类型首次写入即锁定，历史 float 撞上 string 就整批 drop。
  这让我把『发布端容错』和『存储端类型约束』连起来看 —— 进程不崩不等于数据无害，
  脏数据要在发布端就挡掉，而不是等写库报错。」

---

### 本阶段诚实边界（阶段 1.1 / 1.2，别写越线）
- ✅ 已实证（阶段 1.1，16:56 两端截图逐条闭环）：**真实串口六路脉宽 + 8080 真实帧号/检测数 → 组 JSON → MQTT 发布 → 订阅收到**。发布端 `[bridge] -> {...}` 与订阅端 `device/kunpeng_01/telemetry {...}` **十条一一对应**（`ts` 1794128066~111、`frame_id`、`det_count` 全一致）。`pulses` 恒为 `[1580,1900,2400,700,1450,1500]` = 固件初始位 INIT（臂静止在 HOME）；`frame_id` 32692→33804 / 45s ≈ 24.7 fps，与页面 26.6 fps 吻合；`det_count` 在 `0/1/4` 之间**真实变化**，不是固定假数据。🔴 图里 `"frame_id": 3269` 接折行 `2,` = `32692`，是终端宽度折行，不是数据错位。
- 🔴 桥 `/api/state` 是 **POST**，每 5 秒会真的往串口多发一条 STATE 帧（唯一副作用，已在代码注释写明并降频）；不是纯读。
- ✅ **1.2b 已实证（15:13）**：真实串口遥测 → MQTT → EMQX → eKuiper → InfluxDB → Grafana **端到端可观测性闭环成立**：Explore 出六个 `j1~j6` series，Table 有 15:12:57~15:13:27.5 的真实六路点（= 固件初始位 INIT，真实串口回读非 mock）。`-24h` Flux 返回 11 个 field 全齐的真实 CSV。八连修排障链（第 5~11 条）全部板上实证，四次错误假设已逐一证伪撤回。
- ⚠️ **曲线是臂静止在初始位的六条平线，不是抓取 / 运动过程曲线**；1.2 只增可观测性、不增抓取能力。
- 🔴 **收口进度（11-09「写」授权后）**：PC 侧工程文件已与板上实证配置**对齐** ——
  `iot/deploy.sh` 四处收口（静态 tags / 清理顺序「先删规则再删流」/ `/status` 查法 /
  firewalld 动态重绑 trusted zone）、`iot/telemetry_bridge.py` 新增 `is_publishable()` 跳过 null 点、
  `iot/README.md` 同步实证口径与第 8~11 条排障；PC 自检 28 项 ALL PASS、token 三处一致、
  `git diff --check` 通过。trusted zone 不再靠一次手工 runtime 绑定，改为**每次部署自动重绑**，
  重启 / 网络重建后由脚本自愈，不再是悬空项（也不用 `--permanent --reload`，避免冲掉 Docker 链）。
  ⏭️ **仍差最后一步实机复验**：新版 3 个文件需 scp 上板、重跑 `./deploy.sh` 与
  `telemetry_bridge.py`，确认静态 tags / 动态重绑 / 跳过 null 点在板上同样生效后再提交。
  🔴 **现在仍不能写「已交付」**：板上复验未完成；且本阶段一律只叫「可观测性闭环」。
- ❌ 不得写「云端 IoT / 数字孪生 / 自动抓取 / 闭环控制 / IoT 五件套已通 / 抓取曲线」。

---

## 三、LLM 意图解析层

> 来源：`排障记录-服务端与IoT.md`

> 记录区间：2026-11-08。格式固定：**现象 → 排查过程 → 根因 → 解决 → 一句话面试说辞**。
> 只记重要 / 关键 / 面试官关心的问题，鸡毛蒜皮不记。

### 第 1 条：`python -m unittest discover -s <目录>` 报 `ImportError: Start directory is not importable`

- **现象**：17:14 在 PowerShell 跑
  `python.exe -B -m unittest discover -s "D:\...\kunpeng\llm" -p "test_*.py"`，
  报 `ImportError: Start directory is not importable: 'D:\\...\\kunpeng\\llm'`，
  一条测试都没跑起来。当时容易误判成「刚写的代码有语法错 / 模块导入失败」。

- **排查过程**：
  1. `Glob kunpeng/llm/**` → `No files found`，确认**阶段 2 源码当时根本还没落盘**（按「点头才动手」约定，目录都不存在）；
  2. 17:18 落盘后先补 `__init__.py`，在目录内跑 `discover -s . -p "test_*.py" -v` → **26 项全绿**；
  3. 再用原样的绝对路径版 `discover -s "D:\...\kunpeng\llm"` 复跑 → exit code 0，报错不复现；
  4. 对照 CPython 3.11+ `unittest/loader.py`：`TestLoader.discover()` 要用 start_dir 推导模块名，要求该目录**可导入**；3.11 起不再支持 namespace package（无 `__init__.py` 的目录）作为发现起点。

- **根因**：**两个原因叠加**，都跟 `intent_parser.py` 的逻辑无关。
  ① 主因是**验证命令跑在代码落盘之前** —— 目录不存在，谈不上导入；
  ② 即便目录建好，Python 3.11+ 的 `unittest discover` 要求起始目录是 **regular package**（含 `__init__.py`），否则拒收。
  规范定义：PEP 420 引入 namespace package；CPython 3.11 的 unittest 移除了对它的 discover 支持（`TestLoader.discover` 里 `is_not_importable` 检查）。

- **解决**：
  1. **流程**：按「先点头 → 再落盘 → 最后给验证命令」执行；「待点头 / 尚未落盘」阶段不再把可整段复制的验证命令摆进代码块（已写进跨项目习惯）。
  2. **代码**：新增 `kunpeng/llm/__init__.py`（空文件 + docstring），让目录成为 regular package。
  3. **命令**：保留绝对路径 `-s` 也能过（包已可导入）；另给一条更稳的写法是先 `cd` 进目录跑 `discover -s .`。
  4. **实测**：`--self-test` → `SELF_TEST PASS`；`unittest discover` → `Ran 26 tests OK`；PowerShell 绝对路径版 → exit code 0。

- **一句话面试说辞**：「`unittest discover` 在 Python 3.11 之后要求起始目录是可导入包，我给模块补了 `__init__.py`；那次报错更主要的原因是验证命令跑在代码落盘之前，所以我把『先写后验』定成了流程。」

### 第 2 条：中文串口帧 XOR「按字节还是按字符」——跨语言对齐坑

- **现象**：16:35 做 ESP32 文本入口离线网关时，`--self-test` 一开始把演示用的空文本帧
  `$ASR,*3C\r\n` 判成 `CK_MISMATCH`（算出 `6C`、帧里写 `3C`），不是预期的 `EMPTY_TEXT`。
  更要紧的是：若在 Python 里按 `for ch in text: ck ^= ord(ch)` 算中文校验和，
  拿到的是 Unicode 码点（「抓」= 0x6293），远超 1 字节，与 C 固件对不上。

- **排查过程**：
  1. 回读 `Core/Src/uart_protocol.c`：L123 `ck ^= (uint8_t)payload[i]`（发包）、
     L523 `calc_ck ^= frame_buf[i]`（收包）都是对 **uint8_t 字节**做 XOR，L128-129 输出 2 位大写 HEX；
  2. 确认 Python 的 `str` 是 Unicode 码点序列，「抓」在 UTF-8 里是 3 字节 `E6 8A 93`；
  3. 用 S2 已实机的 `$PING*10` / `$OK,PONG*3E` 反查：CK("PING")=0x10、CK("OK,PONG")=0x3E，
     证明「逐字节 XOR」实现与固件一致，不是自说自话；
  4. 空文本帧那条其实是**演示用例手算 CK 写错**（`ASR,` 的 XOR 应为 6C），
     改用 `build_frame("")` 生成后，校验层才正常走到 `EMPTY_TEXT` 分支。

- **根因**：XOR 校验定义在**字节序列**上，而 Python `str` 是字符（码点）序列。
  跨语言对齐必须同时统一 ① 字符编码（UTF-8）② 字长（8 bit）。
  规范定义：纵向冗余校验（LRC）作用于 octet stream；`uint8_t` 逐字节累加后截断到 0~255，
  与 Unicode code point 不是同一数学对象。

- **解决**：
  1. `xor_checksum()` 里先 `payload.encode("utf-8")` 再逐字节 `ck ^= byte`，最后 `& 0xFF`；
  2. 把「逐字符算」与「逐字节算」两种结果都写进 `--self-test` / unittest 断言，
     用 `wrong_ck != manual` 把这条坑固化成回归用例；
  3. 用两条 S2 实机帧当锚点反查实现；
  4. 演示帧一律用 `build_frame()` 生成，不再手写 CK；
  5. 实测：`text_gateway.py --self-test` → `SELF_TEST PASS`；`unittest discover` → `Ran 44 tests OK`。

- **一句话面试说辞**：「串口帧的 XOR 是按字节算的，中文是 UTF-8 多字节；我在 Python 侧先 encode 再逐字节异或，并拿项目实机跑通的 `$PING*10`、`$OK,PONG*3E` 反查实现，避免上层语言按 Unicode 码点算导致将来一接板子就全部校验失败。」

### 本阶段诚实边界（写简历 / 面试别越线）

- ✅ 能写：「自然语言 → 结构化动作指令的**意图解析层**，mock 实现 + 26 项单元测试通过；契约定死、后端可替换」。
- ❌ 不能写：「接入了云端大模型」「自然语言控制机械臂已实现」「语音→动作闭环」「自动抓取」。
- 模块**只输出结构化指令、不下发**；`source` 恒 `"mock"`；`params` 恒空（不猜数值）；枚举里故意没有 `NAV`（车体 / 导航是阶段 6，命中即 `unknown` + warning）；`CloudBackend` 是占位、`parse()` 抛 `NotImplementedError`，`LLM_API_KEY` 只走环境变量、绝不写进代码。
- 当前真正能执行的是「人点画面 → `ArmIK` 逆解 → `$ARM,POS` 串口帧 → 总线舵机位置闭环」这条链路
  （旧 PWM 时代的「`$JOINT` 开环脉宽」口径已于 2026-11-26 作废）；自动抓取、视觉伺服闭环仍未做，
  `action_primitives.pick()` 的下降-闭合-抬起只离线自测过。