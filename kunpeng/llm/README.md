# kunpeng/llm/ —— 意图解析层（阶段 2）

> **当前状态（2026-11-09）**：mock 版 + 离线文本入口网关均已落盘、本地测试通过（**44 项 unittest 全绿**）。
> **未接云端大模型、未下发到机械臂、未接真串口**。大白话：把人话翻成机器认得的固定 JSON，
> 现在是「按关键词查表」的假翻译员 + 一个模拟 ESP32 发文本的离线网关，先把格式和契约验对。
> 规范定义：意图解析（intent parsing / slot filling），输出受限的结构化动作表示。

## 一、概念（大白话 + 规范定义，面试既要听懂也要说得像科班）

| 概念 | 大白话 | 规范定义 |
|---|---|---|
| LLM | 跑在别人服务器上的「会接话的程序」，你这头发一句话、收一句话 | 大语言模型，基于 Transformer 的自回归模型 `P(next token∣context)`；经 HTTP API（OpenAI 兼容 / DeepSeek / 通义）调用 |
| 意图解析 | 把「把红块抓起来」翻译成机器认得的固定格式 | 语义解析 / slot filling：非结构化文本 → 受限结构化动作表示（intent + slots） |
| 结构化输出 | 不许回一段作文，只准回规定好字段的 JSON | 受约束解码 / JSON Schema 约束响应，便于下游确定性解析 |
| mock | 先用按关键词查表的假解析器顶上，把格式验对，等有 Key 再换真的 | 测试替身（Test Double）：用可控实现隔离外部依赖，验证契约而非外部服务 |
| 后端抽象 | 先定好插座标准，现在插假的、以后插真的，别处一行不改 | 面向接口编程 / 策略模式；Python 用 `abc.ABC` + `@abstractmethod`（相当于 C++ 纯虚基类） |
| dataclass | 给数据装上有名字的字段，写错字段名立刻报错 | `@dataclasses.dataclass` 自动生成 `__init__` / `__repr__` / `__eq__`；`field(default_factory=...)` 避免可变默认值共享 |
| enum | 把「能输出的动作」关进固定集合，拼错立刻报错 | `enum.Enum`，成员值即 JSON 里的动作字符串 |
| 串口帧协议 | 一句话前后加 `$` `*` 和校验码，收方才知道从哪到哪、有没有传错 | 受限帧格式 `$<CMD>[,<ARG>...]*<CK><CR><LF>`；CK 为 payload 逐字节 XOR、2 位大写 HEX（`Core/Src/uart_protocol.c` v1） |
| XOR 校验 | 把每个字节异或一遍得到一个字节的「指纹」，传错一位就对不上 | 纵向冗余校验（LRC）的最简形式；定义在**字节序列**上，跨语言必须统一编码（UTF-8）与字长（8 bit） |
| 网关 / 适配层 | 门口收发室：先验包裹、再拆出内容交给里面的人，自己不干活 | Gateway / Anti-Corruption Layer：隔离外部协议与内部领域模型，此处输入帧、输出固定键 JSON 契约 |

## 二、契约（对齐已有规划文档，不另起炉灶）

已有文档里其实写好了形状，本模块直接对齐：

- `{action:"pick", target:"red_cup"}` ← `docs/项目完整流程模拟（A档终态）.md` L24
- `{"action":"move_left","dist":10}` ← `kunpeng/docs/A档执行计划.md` L55
- 「把红色零件放进盒子」→「导航→对准→抓取→放置」动作序列 ← `docs/项目实施方案.md` L72
- 「语音控制 → 意图解析 → MQTT 下发」← `docs/功能对齐与实现路径（对标幻尔套件）.md` L87

落地输出（键集合**固定**，多一个少一个都算破坏契约）：

```json
{
  "action": "pick",
  "target": "red_cup",
  "qualifier": "left",
  "params": {},
  "raw": "把左边的红色杯子抓起来",
  "source": "mock",
  "warnings": []
}
```

| 字段 | 含义 |
|---|---|
| `action` | 动作枚举：`pick / place / home / move_left / move_right / look / unknown` |
| `target` | 目标物体：`red_cup / red_block / blue_block / box`（词表在 `intent_parser.py` 常量区） |
| `qualifier` | 方位限定：`left / right / front` |
| `params` | 数值参数（对齐文档的 `dist`）；**mock 阶段恒为空**，不猜数值 |
| `raw` | 原话，原样保留，便于回溯和调试 |
| `source` | **恒为 `mock`**，防止以后看日志误以为接了真模型 |
| `warnings` | 兜底提示（不认识 / 多个动作 / 多个目标 / 缺目标 / 导航未实现） |

## 三、用法（都不联网、零第三方依赖）

> 进入 `kunpeng/llm/` 目录后执行；【PC · PowerShell】/【板 · openEuler 终端】均可，纯 Python 逻辑不碰硬件。

```bash
python3 intent_parser.py --self-test      # 纯逻辑自测，零依赖
python3 intent_parser.py                  # 跑 7 条内置演示句并打印 JSON
python3 intent_parser.py --pretty         # 演示句缩进输出
python3 text_gateway.py --self-test       # 网关自测（含 S2 实机帧反查 XOR）
python3 text_gateway.py                   # 跑内置演示帧（4 正常 + 5 坏帧）
python3 text_gateway.py --text "把红块抓起来"          # 模拟 ESP32 发来一句
python3 text_gateway.py --frame '$ASR,回初始位*XX'     # 手动喂一整帧
python3 vlm_decider.py --self-test         # P2 决策层自测（假服务，8 组用例）
python3 vlm_decider.py --instruction "把杯子抓起来" --dry-run   # 只看 prompt，不调模型
python3 -m unittest discover -s . -p "test_*.py" -v
```

`--text "..."` 可解析单句（可选，本地调试用）。当前 `unittest discover` 共 **44 项**（`intent_parser` 26 + `text_gateway` 18）；`vlm_decider.py` 的自测走 `--self-test`（内含 8 组端到端用例，用本地假 HTTP 服务代替真模型）。
> 🆕 **阶段 2 下游已接上（11-14 B 项）**：`../robot/voice_gateway.py` 通过 `sys.path` 复用本目录的
> `parse_frame` / `parse_intent` / `xor_checksum`，把「只打印不下发」升级成「本地仲裁后经 `/api/cmd` 下发」。
> 它的 **38 项** 考卷在 `kunpeng/robot/test_voice_gateway.py`，**不在本目录的 `unittest discover` 里**
> （别把 44 写成 76）；本目录 44 项口径不变。✅ **该网关阶段 2 链路 11-14 16:40 已上板实机验证**；
> 🔴 语音入口整体仍 🔶（ESP32 整机 build 已通过、已烧录实机验证，帧草案未与 ESP32 对齐）。

## 四、README 三行法（面试复习用）

### `intent_parser.py`

- **① 用了什么语法 / 技术栈**：`enum.Enum` 动作枚举、`@dataclasses.dataclass` 指令对象、`typing.Optional` 类型注解、`abc.ABC` + `@abstractmethod` 后端抽象、`argparse` 命令行、`json` 序列化、纯函数分层（`_mock_parse` / `validate` / `parse_intent` / `to_dict`）、标准库 `unittest`、`if __name__ == "__main__"` 入口；**零第三方依赖**、第三方库只在未来的 `CloudBackend` 里延迟导入。
- **② 为什么这么设计**：① 拆成独立模块、主链路一行不改 —— 改 `web_control.py` 就要 scp + 重启 Flask，重启会清掉 `_IK_MAP` 标定值、打断实机现场；② 纯函数 + 固定键集合让「取数 / 校验 / 序列化」能分开单测，出问题一眼分清是哪层；③ 抽象后端 + 默认写死 `MockBackend`，保证「不传参 = 纯本地零联网」，将来接真 API 只换一个后端；④ 与阶段 1 的 `iot/telemetry_bridge.py` 同一套风格（大白话 + 规范定义双行 docstring、常量集中、`--self-test`、分层容错），整条项目讲起来是一套东西。
- **③ 不这么写会怎样**：① 若用裸 `dict` + 字符串动作，拼错 `"pikc"` 不报错、下游按错键取值直接静默出错；② 若 `params: dict = {}` 直接写空字典当默认值，所有实例共享同一个 dict，改一条全变（经典新手坑）；③ 若把 `CloudBackend` 设成默认后端，脚本一跑就偷偷往外发请求、还可能把 Key 泄露进日志；④ 若 `source` 不恒为 `mock`，以后翻日志会误判「已经接了大模型」，写进简历就是虚假陈述；⑤ 若遇到多动作 / 导航词就瞎猜一个动作，下游会当真指令处理，比老实返回 `unknown` 危险得多。

### `text_gateway.py`

- **① 用了什么语法 / 技术栈**：`enum.Enum` 拒收原因、`@dataclasses.dataclass` 结果对象、`str.encode("utf-8")` + 逐字节 XOR、`str.split(",", 1)` 的 `maxsplit`、`argparse`、纯函数分层（`xor_checksum` / `build_frame` / `parse_frame` / `handle_frame`）、标准库 `unittest`；顶层**零第三方依赖**，不 import 串口库、不 import 网络库。
- **② 为什么这么设计**：① 把「ESP32 未来会怎么发文本」这个**入口契约**先立成可跑的代码，而不是写在文档里悬着 —— 帧格式、校验、命令字、拒收原因都能离线验证；② 复用项目统一帧协议与 `intent_parser` 的现成翻译员，一行解析逻辑都不重复，词表 / 后端只在一处改；③ 用 S2 已实机的 `$PING*10` / `$OK,PONG*3E` 当校验锚点，避免「自己算自己判」的自证循环；④ 纯函数返回「拒收原因」而不抛异常，坏帧不会把将来常驻的串口接收进程带崩；⑤ 网关输出包一层 `{"accepted", "command"}`，**不污染**意图层那 7 个固定键。
- **③ 不这么写会怎样**：① 若中文按 `ord(ch)` 逐**字符** XOR 而不是按 UTF-8 逐**字节**，算出的 CK 与 C 固件 `(uint8_t)payload[i]` 对不上，将来真接上 ESP32 会 100% 被判 `ERR,1` 校验失败（见 `docs/排障记录-LLM.md` 第 2 条）；② 若 `payload.split(",")` 不写 `maxsplit=1`，识别文本里一个逗号就把一句话切碎；③ 若模块里顺手加个 `publish()` / 串口写，mock 阶段就可能误触发真下发 —— 现在**根本没有下发函数**，边界在代码结构上就锁死；④ 若把帧格式写成「已完成」，简历就会从 📋 越线到 ✅，属虚假陈述。

### `vlm_decider.py`（阶段 6 / P2，2026-11-11 新增）

- **① 用了什么语法 / 技术栈**：`abc.ABC` + `@abstractmethod` 后端抽象（`MockBackend` / `HttpBackend` 两个实现）、`@dataclasses.dataclass`（`Candidate` / `Decision`）、`urllib.request` 调 OpenAI 兼容 `/chat/completions`、图片走 `data:image/jpeg;base64,` 内联、`temperature=0.0` 求确定性、三重容错的 `extract_json()`、显式查表的 `normalize()` 越界拒绝、`http.server.ThreadingHTTPServer` 起本地假服务做自测、`argparse`（`--dry-run` / `--backend` / `--base-url` / `--model`）；**零第三方依赖**。
- **② 为什么这么设计**：① **模型只选编号，不输出坐标** —— VLM 的 grounding 精度 ±30~50 px，20 cm 处 1 px ≈ 0.25 mm ⇒ 直接给坐标误差 ±1 cm 以上必抓偏；而本地 YOLO 的 bbox 是像素级精确的，让「语义判断」和「像素定位」各干各擅长的；② 观测只调 `/observe` 一个接口，因为它在服务端用**同一把锁**取画面与 bbox，保证同帧 —— 分两次取会出现「看第 N 帧的图、拿第 N+1 帧的 bbox」，按编号选对了物体、坐标却偏了，这种 bug 不报错只会偶尔抓偏；③ 输出复用 `intent_parser.to_dict()` 的 7 键契约，`params` 里带编号/bbox/理由，下游无歧义消费，两套枚举不会漂移；④ 模型跑在**有 GPU 的 PC**（RTX 4060 Laptop 8 GB 可跑 Qwen2.5-VL-3B），板子只出 `/observe`，因为鲲鹏 Pro 无 CUDA、纯 ARM 跑 7B VLA 单帧要几十秒，撑不起秒级闭环；⑤ `MockBackend` 不是「智能」的替身而是**链路的替身**，用来验证「候选打包 → prompt → 解析 → 校验 → 契约」这条管道本身没坏。
- **③ 不这么写会怎样**：① 若让 VLM 直接吐像素坐标，模型说得头头是道但坐标偏 1 cm，人手看日志完全发现不了；② 若 `choice` 越界不做校验直接 `cands[choice-1]`，`choice=99` 抛 `IndexError` 把进程搞崩，而 `choice=-1` 在 Python 里会**静默取到最后一个候选** —— 抓错物体还不报错，这是最阴的坑；③ 若解析失败就当「没听见」沿用上一次结果，会拿旧决策去驱动机械臂；④ 若候选为空仍调模型，模型只能瞎编编号，白烧一次推理还可能被下游当真；⑤ 若 `temperature` 不设 0，同一张图同一句话每次给不同答案，排障会变成猜谜；⑥ 若把 `source` 恒写 `"mock"` 或反过来谎报接了真模型，看日志的人会被自己骗。

## 五、🔴 诚实边界（写简历 / 面试别越线）

- ✅ 能写：「自然语言 → 结构化动作指令的**意图解析层**，mock 实现 + 单元测试通过；契约定死、后端可替换」；「搭建了 ESP32 文本入口的**离线网关**，复用统一串口帧协议与 XOR 校验，44 项单元测试通过」。
- ✅ 能写（P2，2026-11-11）：「**VLM 语义决策层**：本地检测器出候选，模型只从候选里选编号，坐标交给几何链路；越界编号/非法枚举一律拒绝执行，8 组端到端自测通过」。
- ❌ 不能写：「接入了云端大模型」「自然语言控制机械臂已实现」「语音→动作闭环」「自动抓取」「已接 ESP32-S3 语音链路 / 语音控制已实现」。
- ❌ 不能写（P2 专项）：「端到端具身智能」「VLA」。本项目是**分层**：模型出符号（选哪个），几何出坐标（在哪），执行层是开环脉宽。三者是三个模块，靠 JSON 串起来，不是一条神经网络端到端训出来的。
- `vlm_decider.py` **只产出决策，不执行动作**。它和执行层 `action_primitives.py`（P1）**还没有接成一条自动链路** —— 中间那一步「决策 → `/api/ik/solve` → pick」由人工串。
- 两个模块都**只输出结构化指令，不下发**。`text_gateway.py` 的帧格式 `$ASR,<文本>*<CK>\r\n` 是**建议草案、未与 ESP32 实机对齐、未接真串口**；模块里没有下发函数、不联网、不读 Key。
- 当前真正能执行的是「人点画面 → `ArmIK` 逆解 → `$JOINT` 串口帧」这条**开环**链路；自动抓取、下降-闭合-抬起序列、视觉伺服闭环都还没做。
- 枚举里**故意没有 `NAV`**：车体 / 导航是阶段 6，命中导航词一律 `unknown` + warning。
- `CloudBackend` 是占位，`parse()` 直接抛 `NotImplementedError`；`LLM_API_KEY` 只走环境变量，**绝不写进代码**（`kunpeng/docs/A档执行计划.md` L54）。
- `params` 恒为空：mock 不猜数值，猜了就是假数据。
- 🟢 硬件侧更新（2026-11-09 16:28）：小Z 已有**买好的成品 ESP32-S3 开发板**（CH343 / N16R8 / Type-C / 已焊）可直接用，另有 INMP441 麦模块、MAX98357A 功放模块 + 3W 喇叭（均已焊排针）。⇒ 语音前端**硬件不再是障碍**，但软件仍未上板、未接主链路，档位不变。（✅ 11-21 订正：已烧录实机，SoftAP/TCP 9000/采音通过；ESP-SR 命令词仍未实现，档位仍 🔶）

## 六、与后续阶段的接口（现在只是预留，不要当成已完成）

| 阶段 | 怎么接 | 状态 |
|---|---|---|
| 2 后续（有 Key 后） | 实现 `CloudBackend.parse()`：读 `LLM_API_KEY` → HTTP 调 OpenAI 兼容 API → 解析响应 JSON → 填 `params` / `source="cloud"` | 📋 待 Key |
| **6 P2 决策（2026-11-11）** | `vlm_decider.py`：`/observe` 拿画面+同帧 bbox → VLM 选编号 → 输出 7 键契约 | 🔶 **代码完成 + 8 组自测过；未接真模型、未接执行层** |
| 6 P2→P1 串联 | 决策的 `params.center` → `web_control` 的 `/api/ik/eye/solve` → `action_primitives.pick()` | 📋 待真模型跑通后做 |
| 6 P3 闭环 | 执行后重新 `/observe` → 问 VLM「原来的物体还在吗」→ 在则换抓取点重试 | 📋 纯规划 |
| 3 ESP32-S3 语音 | ASR 文本按 `$ASR,<文本>*<CK>\r\n` 发鲲鹏 → `text_gateway.parse_frame()` → `parse_intent()` | 🔶 成品板已到、离线网关已跑通；**未接真串口 / 未下发** |
| **2 鲲鹏侧收帧服务（11-14 B 项）** | `../robot/voice_gateway.py`：TCP `:9000` 收帧 → 复用本模块 `parse_frame` / `parse_intent` → **本地仲裁白名单** → `/api/cmd` 下发固件原生 `HOME/STOP/RESUME` | ✅ **阶段 2 网关链路 11-14 16:40 已上板实机验证**（**38 项本机 unittest 全绿** + 四层硬判据全绿，记录见 `docs/上板验收-语音网关阶段2.md`）；🔴 ESP32 已接入（发 `$ESTOP/$RESUME` 自检帧）、`$ASR` 帧草案未对齐（语音入口整体仍 🔶）；默认 dry-run、只授权三个无参数命令字；排障见 `docs/排障记录-语音网关.md` |
| 4 端到端闭环 | 结构化指令 → 鲲鹏决策 → `/api/ik/solve` 或 `$JOINT` → STM32 | 📋 纯规划 |
| 1.2 IoT | 指令可经 MQTT 下发（复用 `device/<id>/telemetry` 之外的命令主题） | 📋 纯规划 |

## 七、文件清单

| 文件 | 作用 |
|---|---|
| `intent_parser.py` | 意图解析主模块（契约 + mock 后端 + 纯函数 + `--self-test`） |
| `test_intent_parser.py` | 标准库 `unittest` 考卷 · 26 项（契约 / 正例 / 负例 / 校验 / 后端可替换） |
| `text_gateway.py` | 模拟 ESP32 串口文本入口（帧校验 + 意图解析，离线只打印不下发） |
| `test_text_gateway.py` | 标准库 `unittest` 考卷 · 18 项（校验 / 往返 / 拒收 / 契约 / 边界） |
| `vlm_decider.py` | 🆕 **P2 VLM 语义决策层**：`/observe` 拿画面+同帧候选 → 模型只选编号 → 校验越界 → 输出 7 键契约；`--self-test` 假服务 8 组用例 |
| `__init__.py` | 让 `llm/` 成为可导入包，`unittest discover` 才不报 `Start directory is not importable` |
| `README.md` | 本文件（概念双行 + 三行法 + 诚实边界） |
