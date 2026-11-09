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
python3 -m unittest discover -s . -p "test_*.py" -v
```

`--text "..."` 可解析单句（可选，本地调试用）。当前 `unittest discover` 共 **44 项**（`intent_parser` 26 + `text_gateway` 18）。

## 四、README 三行法（面试复习用）

### `intent_parser.py`

- **① 用了什么语法 / 技术栈**：`enum.Enum` 动作枚举、`@dataclasses.dataclass` 指令对象、`typing.Optional` 类型注解、`abc.ABC` + `@abstractmethod` 后端抽象、`argparse` 命令行、`json` 序列化、纯函数分层（`_mock_parse` / `validate` / `parse_intent` / `to_dict`）、标准库 `unittest`、`if __name__ == "__main__"` 入口；**零第三方依赖**、第三方库只在未来的 `CloudBackend` 里延迟导入。
- **② 为什么这么设计**：① 拆成独立模块、主链路一行不改 —— 改 `web_control.py` 就要 scp + 重启 Flask，重启会清掉 `_IK_MAP` 标定值、打断实机现场；② 纯函数 + 固定键集合让「取数 / 校验 / 序列化」能分开单测，出问题一眼分清是哪层；③ 抽象后端 + 默认写死 `MockBackend`，保证「不传参 = 纯本地零联网」，将来接真 API 只换一个后端；④ 与阶段 1 的 `iot/telemetry_bridge.py` 同一套风格（大白话 + 规范定义双行 docstring、常量集中、`--self-test`、分层容错），整条项目讲起来是一套东西。
- **③ 不这么写会怎样**：① 若用裸 `dict` + 字符串动作，拼错 `"pikc"` 不报错、下游按错键取值直接静默出错；② 若 `params: dict = {}` 直接写空字典当默认值，所有实例共享同一个 dict，改一条全变（经典新手坑）；③ 若把 `CloudBackend` 设成默认后端，脚本一跑就偷偷往外发请求、还可能把 Key 泄露进日志；④ 若 `source` 不恒为 `mock`，以后翻日志会误判「已经接了大模型」，写进简历就是虚假陈述；⑤ 若遇到多动作 / 导航词就瞎猜一个动作，下游会当真指令处理，比老实返回 `unknown` 危险得多。

### `text_gateway.py`

- **① 用了什么语法 / 技术栈**：`enum.Enum` 拒收原因、`@dataclasses.dataclass` 结果对象、`str.encode("utf-8")` + 逐字节 XOR、`str.split(",", 1)` 的 `maxsplit`、`argparse`、纯函数分层（`xor_checksum` / `build_frame` / `parse_frame` / `handle_frame`）、标准库 `unittest`；顶层**零第三方依赖**，不 import 串口库、不 import 网络库。
- **② 为什么这么设计**：① 把「ESP32 未来会怎么发文本」这个**入口契约**先立成可跑的代码，而不是写在文档里悬着 —— 帧格式、校验、命令字、拒收原因都能离线验证；② 复用项目统一帧协议与 `intent_parser` 的现成翻译员，一行解析逻辑都不重复，词表 / 后端只在一处改；③ 用 S2 已实机的 `$PING*10` / `$OK,PONG*3E` 当校验锚点，避免「自己算自己判」的自证循环；④ 纯函数返回「拒收原因」而不抛异常，坏帧不会把将来常驻的串口接收进程带崩；⑤ 网关输出包一层 `{"accepted", "command"}`，**不污染**意图层那 7 个固定键。
- **③ 不这么写会怎样**：① 若中文按 `ord(ch)` 逐**字符** XOR 而不是按 UTF-8 逐**字节**，算出的 CK 与 C 固件 `(uint8_t)payload[i]` 对不上，将来真接上 ESP32 会 100% 被判 `ERR,2` 校验失败（见 `docs/排障记录-LLM.md` 第 2 条）；② 若 `payload.split(",")` 不写 `maxsplit=1`，识别文本里一个逗号就把一句话切碎；③ 若模块里顺手加个 `publish()` / 串口写，mock 阶段就可能误触发真下发 —— 现在**根本没有下发函数**，边界在代码结构上就锁死；④ 若把帧格式写成「已完成」，简历就会从 📋 越线到 ✅，属虚假陈述。

## 五、🔴 诚实边界（写简历 / 面试别越线）

- ✅ 能写：「自然语言 → 结构化动作指令的**意图解析层**，mock 实现 + 单元测试通过；契约定死、后端可替换」；「搭建了 ESP32 文本入口的**离线网关**，复用统一串口帧协议与 XOR 校验，44 项单元测试通过」。
- ❌ 不能写：「接入了云端大模型」「自然语言控制机械臂已实现」「语音→动作闭环」「自动抓取」「已接 ESP32-S3 / 语音控制已实现」。
- 两个模块都**只输出结构化指令，不下发**。`text_gateway.py` 的帧格式 `$ASR,<文本>*<CK>\r\n` 是**建议草案、未与 ESP32 实机对齐、未接真串口**；模块里没有下发函数、不联网、不读 Key。
- 当前真正能执行的是「人点画面 → `ArmIK` 逆解 → `$JOINT` 串口帧」这条**开环**链路；自动抓取、下降-闭合-抬起序列、视觉伺服闭环都还没做。
- 枚举里**故意没有 `NAV`**：车体 / 导航是阶段 6，命中导航词一律 `unknown` + warning。
- `CloudBackend` 是占位，`parse()` 直接抛 `NotImplementedError`；`LLM_API_KEY` 只走环境变量，**绝不写进代码**（`kunpeng/docs/A档执行计划.md` L54）。
- `params` 恒为空：mock 不猜数值，猜了就是假数据。
- 🟢 硬件侧更新（2026-11-09 16:28）：小Z 已有**买好的成品 ESP32-S3 开发板**（CH343 / N16R8 / Type-C / 已焊）可直接用，另有 INMP441 麦模块、MAX98357A 功放模块 + 3W 喇叭（均已焊排针）。⇒ 语音前端**硬件不再是障碍**，但软件仍未上板、未接主链路，档位不变。

## 六、与后续阶段的接口（现在只是预留，不要当成已完成）

| 阶段 | 怎么接 | 状态 |
|---|---|---|
| 2 后续（有 Key 后） | 实现 `CloudBackend.parse()`：读 `LLM_API_KEY` → HTTP 调 OpenAI 兼容 API → 解析响应 JSON → 填 `params` / `source="cloud"` | 📋 待 Key |
| 3 ESP32-S3 语音 | ASR 文本按 `$ASR,<文本>*<CK>\r\n` 发鲲鹏 → `text_gateway.parse_frame()` → `parse_intent()` | 🔶 成品板已到、离线网关已跑通；**未接真串口 / 未下发** |
| 4 端到端闭环 | 结构化指令 → 鲲鹏决策 → `/api/ik/solve` 或 `$JOINT` → STM32 | 📋 纯规划 |
| 1.2 IoT | 指令可经 MQTT 下发（复用 `device/<id>/telemetry` 之外的命令主题） | 📋 纯规划 |

## 七、文件清单

| 文件 | 作用 |
|---|---|
| `intent_parser.py` | 意图解析主模块（契约 + mock 后端 + 纯函数 + `--self-test`） |
| `test_intent_parser.py` | 标准库 `unittest` 考卷 · 26 项（契约 / 正例 / 负例 / 校验 / 后端可替换） |
| `text_gateway.py` | 模拟 ESP32 串口文本入口（帧校验 + 意图解析，离线只打印不下发） |
| `test_text_gateway.py` | 标准库 `unittest` 考卷 · 18 项（校验 / 往返 / 拒收 / 契约 / 边界） |
| `__init__.py` | 让 `llm/` 成为可导入包，`unittest discover` 才不报 `Start directory is not importable` |
| `README.md` | 本文件（概念双行 + 三行法 + 诚实边界） |
