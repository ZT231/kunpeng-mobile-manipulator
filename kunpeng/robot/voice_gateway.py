#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
voice_gateway.py —— 鲲鹏侧语音入口网关（阶段 2 / B 项，2026-11-14）

大白话：将来 ESP32-S3 把「人说的那句话」用 Wi-Fi 发到鲲鹏的 9000 端口。
       这个程序就是鲲鹏那头的「收发室 + 门卫」：
         ① 先拆包裹（帧校验 XOR）→ ② 看是不是急停（急停最优先）
         → ③ 翻译员把人话翻成固定 JSON → ④ 门卫（本地仲裁器）决定「准不准动」
         → ⑤ 准了才通过网页服务已有的 /api/cmd 把命令发给机械臂。
       🔴 默认 **dry-run**：只打印「本该发什么」，真要臂动必须显式加 `--apply`。
规范定义：这是**协议适配层（Gateway / Anti-Corruption Layer）+ 本地仲裁器（Local
         Arbiter）**。输入是受限帧 `$ASR,<文本>*<CK>\\r\\n`，输出是固件原生命令字；
         隔离「外部语音协议」与「内部执行链路」。架构依据
         `docs/机械臂小车+语音-可行性与落地路线.md` 第六节「快慢双通道 + 本地仲裁」：
         **LLM / 意图层只"提议"，本地仲裁器才有"授权动作"的权力**；会动的走本地快通道，
         会说话的走云端慢通道。落地顺序依据 `docs/核心化路线图-ESP32与IoT.md` 阶段 2。

🔴 为什么不改 web_control.py / 不直连串口：
  5000 网页服务是**已实机验收**的主链路（眼在手全链、14 次重复性、三条闸门都在它上面），
  改它就要 scp + 重启 Flask；串口由它**独占**，本进程再开一个串口会抢帧（标定链踩过这个坑）。
  ⇒ 本文件是**独立进程**，只通过 HTTP 调它已有的 `/api/cmd`，主链路一行不碰，删掉即完整回滚。

🔴 为什么只发 HOME / STOP / RESUME 三个命令字（最重要的一条架构判断）：
  固件 `Core/Src/uart_protocol.c:428-451` 原生支持 PING/STATE/**HOME/STOP/RESUME**/JOINT，
  且都回 `OK`。这三个命令字**不带任何数值参数** ⇒ 从根上不需要「软限位裁剪 / to_pulses()
  反解校验 / 重力下垂净空折算」，也就不会踩项目里那些「静默夹紧」「算错符号」的坑。
  意图层的 PICK / PLACE / MOVE_LEFT / MOVE_RIGHT 一律**不授权**：
    - PICK/PLACE：需要先定位 + 抓取序列（未实现），且涉及下降高度与 SAG_DZ 净空闸门；
    - MOVE_*：相对位移要读 STATE + 逐轴软限位 + to_pulses() 反解，属未授权的脉宽改写；
    - LOOK：只观察、不下发（会说话的东西不该动臂）。
  ⇒ 「会动的东西」被收敛成三个无参数命令字，这是把风险从「靠人记」变成「结构上不可能」。

🔴 诚实边界（README / 简历里也写一份，别越线）：
  1) `source` 恒 "mock"（沿用 intent_parser），**没接云端大模型、没接 ASR；ESP32 已接入但只发链路自检帧**；
  2) 帧格式 `$ASR,<文本>*<CK>` 沿用 `llm/text_gateway.py` 的**建议草案**，未与 ESP32 实机对齐；
  3) ✅ **阶段 2 网关链路 2026-11-14 16:40 已上板实机验证**：38 项本机单测 + 四层硬判据全绿
     （假帧 `$ASR,回初始位*61` → `$OK,HOME*27` → `/api/state` 脉宽回初始位 → 日志 `sent:true` → 肉眼见臂动），
     记录见 `docs/上板验收-语音网关阶段2.md`。🔴 但**语音入口整体仍 🔶**：ESP32 已烧录实机（TCP 9000/采音），
     帧草案未与 ESP32 对齐，**不许写成「语音控制已实现 / 语音→动作闭环 / 已接 ESP32」**；
  4) 急停只在**网关层**冻结后续下发，并转发 `STOP` 给固件；仍是开环，无位置反馈。

🔴 11-14 代码审查加固（两处「本机 UTF-8 测不出、上板 LANG=C 才炸」的真问题）：
  a) **locale 无关输出**：板端 `LANG=C` / `PYTHONIOENCODING=ascii` 时，
     `print(中文/emoji)` 会抛 UnicodeEncodeError —— 启动横幅崩了服务就起不来、
     `--self-test` 会 rc=1、handler 日志崩了还会把**应答帧一起吞掉**。
     ⇒ 所有输出统一走 `_emit()`（UTF-8 优先、ASCII 转义兜底），
       handler 改成**先发应答、后打日志**（日志绝不阻塞回帧）。
  b) **「解除急停」不再是「急停」**：`RESUME_WORDS` 里的「解除急停」含「急停」二字，
     旧的裸子串匹配会把它判成急停（喊「解除」反而再急停一次）。
     ⇒ 抽 `_match_resume()` / `_match_estop()`，急停匹配前先排除恢复词。

用法（零第三方依赖、不联网、不读 Key）：
  python3 voice_gateway.py --self-test                    # 纯逻辑自测，不起端口不动臂
  python3 voice_gateway.py --make-frame "回初始位"        # 打印带正确 CK 的帧，供复制粘贴
  python3 voice_gateway.py --text "回初始位"              # 单机喂一帧（默认 dry-run）
  python3 voice_gateway.py --text "急停" --text2 "回初始位" # 见下方 --frames
  python3 voice_gateway.py                               # 起 TCP :9000（默认 dry-run）
  python3 voice_gateway.py --apply --host 0.0.0.0         # 真下发 + 允许 ESP32 连进来
"""

import argparse
import dataclasses
import enum
import json
import os
import socketserver
import sys
import threading
import urllib.error
import urllib.request


# ---------------------------------------------------------------------------
# 跨目录复用：把 ../llm 加进模块搜索路径，一行解析逻辑都不重复写
# ---------------------------------------------------------------------------
# 大白话：voice_gateway.py 在 robot/，翻译员和拆帧工具在隔壁 llm/。
#        Python 默认只找当前目录，得先把 llm/ 告诉它，才能 import。
# 规范定义：sys.path 是模块解析路径列表；sys.path.insert(0, ...) 让同仓库的
#          兄弟目录可被绝对导入（text_gateway.py 内部又 import intent_parser，
#          两者都在 llm/，所以只需加一次）。
_HERE = os.path.dirname(os.path.abspath(__file__))
_LLM_DIR = os.path.normpath(os.path.join(_HERE, "..", "llm"))
if _LLM_DIR not in sys.path:
    sys.path.insert(0, _LLM_DIR)

from intent_parser import parse_intent, to_dict          # noqa: E402  翻译员
from text_gateway import (                               # noqa: E402  拆帧工具
    build_frame, parse_frame, xor_checksum, RejectReason,
)


# ---------------------------------------------------------------------------
# 输出层：板端 locale 不可控，先把「打印」这件事做结实
# ---------------------------------------------------------------------------
def _configure_stdout():
    """尽量把 stdout 设成 UTF-8 容错，避免板端 LANG=C 崩在中文日志上（只跑一次）。

    大白话：print(中文) 要按「终端当前编码」去翻译文字。鲲鹏板上若 locale 是
           LANG=C（纯 ASCII），中文和 emoji 一律 UnicodeEncodeError。这里把
           stdout 的编码显式改成 UTF-8，实在编不了的字符就写成 \\uXXXX 转义，
           打印便再也不会因为编码抛异常。
    规范定义：sys.stdout 是 TextIOWrapper；reconfigure(encoding=..., errors=...)
           可就地改其编码与错误处理策略（Python 3.7+）。errors="backslashreplace"
           把无法编码的字符替换为 Python 字符串转义序列，输出恒为可编码字节。
    """
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="backslashreplace")
    except (AttributeError, ValueError, OSError):
        # 老版本 Python / 被 IDE、unittest 包装过的 stdout 没有 reconfigure，
        # 不要因为这个就起不来 —— 交给 _emit() 的退路兜底。
        pass


def _emit(text, newline=True):
    """安全地打印一行（或一段）文本，任何 locale 下都不抛编码异常。

    🔴 为什么不用裸 print：11-14 实测，ASCII stdout 下中文日志抛
       UnicodeEncodeError；若它写在应答帧之前，会把回给客户端的帧一起吞掉。
    规范定义：str.encode("ascii", "backslashreplace") 把所有非 ASCII 字符
           转成 \\uXXXX / \\UXXXXXXXX 转义，结果必为纯 ASCII，可安全 print。
    """
    data = str(text)
    try:
        print(data, end="\n" if newline else "", flush=True)
    except UnicodeEncodeError:
        safe = data.encode("ascii", "backslashreplace").decode("ascii")
        print(safe, end="\n" if newline else "", flush=True)


_configure_stdout()   # 模块加载即配置一次；失败也不影响，_emit 还有退路


# ---------------------------------------------------------------------------
# 常量集中放这一块：改配置只改这里，别散到函数里
# ---------------------------------------------------------------------------
DEFAULT_HOST = "127.0.0.1"          # 🔴 默认只本机可连；要 ESP32 进来须显式 --host 0.0.0.0
DEFAULT_PORT = 9000                  # 对齐 docs/核心化路线图-ESP32与IoT.md 阶段 1/2
DEFAULT_BASE_URL = "http://127.0.0.1:5000"
HTTP_TIMEOUT_S = 5.0
MAX_FRAME_BYTES = 256               # 超长帧直接拒，别让一个坏客户端把缓冲撑爆

# 固件原生命令字（uart_protocol.c:428-451，均回 "OK"）
CMD_HOME = "HOME"
CMD_STOP = "STOP"        # = ESTOP，冻结输出，仅 RESUME 可解除
CMD_RESUME = "RESUME"

# 急停 / 恢复词：🔴 故意放在**意图解析之前**做文本匹配。
# 急停必须最快，不能等一个可能返回 unknown 的翻译员慢慢查表。
EMERGENCY_WORDS = ("急停", "停止", "停下来", "别动", "不要动", "STOP", "ESTOP")
RESUME_WORDS = ("恢复", "解除急停", "继续", "RESUME")

# 非 ASR 帧也能走急停快速通道（ESP32 将来可直接发 $ESTOP*CK / $RESUME*CK）
SPECIAL_PAYLOADS = {
    "ESTOP": "急停",
    "STOP": "急停",
    "RESUME": "恢复",
}

# 错误码沿用固件 / 项目统一帧协议语义，别自造一套
ERR_FRAME = 1        # 帧格式 / XOR 校验错
ERR_UNKNOWN = 2      # 未识别动作 / 不在白名单
ERR_ARBITER = 3      # 本地仲裁器拒绝（超量程 / 净空 / 急停中 / 未授权动作）
ERR_DOWNLINK = 4     # 下发失败（Flask 没起 / 固件回 ERR）


# ---------------------------------------------------------------------------
# 数据结构：把「仲裁结果」关进固定集合，别散成一堆裸字符串
# ---------------------------------------------------------------------------
class Decision(enum.Enum):
    """仲裁器的三种判决。

    为什么用 enum 而不是裸字符串：写错 "alow" 不报错、日志里也看不出来；
    枚举拼错立刻 AttributeError，而且三种结果一眼看全（背书面试用）。
    """
    ALLOW = "allow"      # 授权下发
    NOOP = "noop"        # 允许，但不动臂（只观察）
    REJECT = "reject"    # 拒绝，回一个 ERR 帧


@dataclasses.dataclass
class ArbiterVerdict:
    """一次仲裁的完整结果（带名字的数据袋子）。

    🔴 用 @dataclasses.dataclass 而不是裸 dict：字段有名字和默认值，
    写错字段名立刻报错；自带 __repr__ / __eq__，日志和单测断言都好读。
    `set_estop` 用 None 表示「不改状态」、True/False 表示「进入/解除急停」——
    这样仲裁器仍是**纯函数**，改全局状态这件事交给外层 TCP 外壳做。
    """
    decision: Decision = Decision.REJECT
    payload: str = ""                 # 要下发的固件命令字；空串 = 不下发
    tag: str = ""                    # 应答帧里给客户端看的短说明
    reason: str = ""                 # 拒绝原因（人话，写进日志和 ERR 帧旁注）
    err_code: int = 0
    set_estop: object = None          # None / True / False
    command: dict = dataclasses.field(default_factory=dict)   # 意图层 7 键契约，供日志


# ---------------------------------------------------------------------------
# 纯函数层：不联网、不开端口、不读环境变量、不改全局状态
# ---------------------------------------------------------------------------
def normalize_frame(frame):
    """把命令行 / 日志里粘出来的**字面量** `\\r\\n` 还原成真实换行（纯函数）。

    🔴 真踩过的坑：在 shell 里写 `--frame '$ASR,回初始位*61\\r\\n'`，单引号里是
    4 个普通字符（反斜杠 r 反斜杠 n），不是回车换行。`parse_frame()` 里
    `rstrip("\\r\\n")` 只去真实 CR/LF，于是 `*` 后面变成 `61\\r\\n`，
    校验位长度不是 2 ⇒ 一律 `ERR,1 bad_ck_len`，好帧也被当坏帧。
    规范定义：这是**转义序列与字节值的差异**；shell 单引号不做转义展开。
    ⇒ 归一化属于「网关适配外部输入」的职责，放在本层，不改底层 text_gateway.py
    （改了会牵动它那 18 项既有单测，且底层纯函数不该替调用方猜格式）。
    """
    if not isinstance(frame, str):
        return frame
    # 顺序不能反：先换长的 \\r\\n，再换短的 \\n，否则会残留一个字面量 \\r
    return frame.replace("\\r\\n", "\r\n").replace("\\n", "\n")


def extract_payload(frame):
    """从一帧里只取出 `$` 与 `*` 之间的 payload（纯函数，不校验 CK）。

    为什么单独抽出来：非 ASR 的急停帧（`$ESTOP*CK`）经过 text_gateway.parse_frame
    会判成 NOT_ASR 并被丢掉；急停不该被丢，需要绕过去看一眼 payload 是什么。
    CK 仍由 parse_frame 校验过，这里只是「再读一次」，不是绕过校验。
    """
    if not isinstance(frame, str) or not frame.startswith("$"):
        return ""
    body = frame[1:].rstrip("\r\n")
    star = body.find("*")
    return body[:star] if star >= 0 else ""


def _match_resume(text):
    """是否命中「恢复」类词（纯函数）。

    🔴 「解除急停」是恢复词，却含「急停」二字 —— 必须单独先判，
    否则会被急停的裸子串匹配反向吃掉（11-14 审查实测，见排障记录第 7 条）。
    """
    if not isinstance(text, str):
        return False
    return any(w in text for w in RESUME_WORDS) or text.strip().upper() == "RESUME"


def _match_estop(text):
    """是否命中「急停」类词（纯函数）。

    大白话：先问「这是不是恢复」，是就直接不算急停；再问「含不含急停词」。
           这样「急停」仍然是急停，但「解除急停」不会反过来变成急停。
    规范定义：短词表子串匹配存在**子串包含歧义**（「急停」⊂「解除急停」），
           用「恢复词优先排除」消歧，避免同一文本被两条规则同时命中。
    ⚠️ 混合语句（同时含急停词与恢复词）按「恢复」处理，属 mock 词表已知边界；
       真正的急停优先级由 `arbitrate()` 的 `estop_active` 冻结闸门保证，
       阶段 4 接真模型再细化。
    """
    if not isinstance(text, str):
        return False
    if _match_resume(text):          # 🔴 先排除恢复词，见上
        return False
    stripped = text.strip()
    return (stripped == "急停"
            or any(w in text for w in EMERGENCY_WORDS)
            or stripped.upper() in ("STOP", "ESTOP"))


def special_text(frame, result):
    """判断这一帧是不是「急停/恢复」快速通道，返回对应中文文本或空串（纯函数）。

    两种入口都支持：
      ① `$ASR,急停*CK`  —— 正常 ASR 帧，命令字对、文本命中急停词；
      ② `$ESTOP*CK`     —— 将来 ESP32 直接发的急停帧（parse_frame 会判 NOT_ASR）。
    🔴 顺序：先走 ASR 文本，再走非 ASR payload；ASR 文本里**先判恢复、后判急停**
       （「解除急停」含「急停」二字，顺序反了会喊「解除」反而再急停一次）。
    """
    if result.ok:
        text = result.text.strip()
        if _match_resume(text):
            return "恢复"
        if _match_estop(text):
            return "急停"
        return ""

    if result.reason == RejectReason.NOT_ASR:
        payload = extract_payload(frame).strip().upper()
        return SPECIAL_PAYLOADS.get(payload, "")
    return ""


def arbitrate(text, estop_active=False):
    """🔴 本地仲裁器（本文件的核心）：决定「这句话准不准让臂动」（纯函数）。

    大白话：翻译员只负责「听懂」，门卫才负责「放不放行」。门卫手里有一张白名单，
           名单外的动作一律回绝，哪怕翻译员说得很像那么回事。
    规范定义：本地仲裁（local arbitration）在决策层与执行层之间做**授权**，
           按优先级依次执行：急停 > 恢复 > 急停冻结 > 意图白名单 > 量程/净空闸门。

    参数 estop_active 是「当前是否处于急停」，由外层传进来（纯函数不读全局状态）。
    """
    if not isinstance(text, str) or not text.strip():
        return ArbiterVerdict(reason="输入为空或不是文本", err_code=ERR_FRAME)

    stripped = text.strip()

    # ① 急停最优先：任何时候都允许，且立刻冻结后续一切动作
    #    🔴 用 _match_estop()（内部已先排除恢复词），不再裸子串匹配：
    #       「解除急停」不会被反向判成急停；小写 stop 也在这里被认出。
    if _match_estop(stripped):
        return ArbiterVerdict(decision=Decision.ALLOW, payload=CMD_STOP,
                              tag="ESTOP", set_estop=True,
                              reason="急停优先：冻结输出，仅「恢复」可解除")

    # ② 恢复：解除急停（未急停时也允许，对固件是幂等的）
    if _match_resume(stripped):
        return ArbiterVerdict(decision=Decision.ALLOW, payload=CMD_RESUME,
                              tag="RESUMED", set_estop=False,
                              reason="解除急停")

    # ③ 急停冻结中：除了「恢复」，谁都别想动臂（对齐固件 ESTOP 语义）
    if estop_active:
        return ArbiterVerdict(reason="急停中，仅「恢复」可解除；本帧未授权",
                              err_code=ERR_ARBITER, tag="ESTOP_ACTIVE")

    # ④ 交给翻译员，拿到固定 7 键契约
    cmd = to_dict(parse_intent(text))
    action = cmd["action"]

    # 🔴 防「将来偷偷接了真模型就绕过仲裁」：非 mock 来源一律先拦
    if cmd["source"] != "mock":
        return ArbiterVerdict(reason="非 mock 来源，阶段 2 未验证该链路 ⇒ 不授权",
                              err_code=ERR_ARBITER, command=cmd)

    # ⑤ 量程闸门：带数值参数的动作需要逐轴软限位 + to_pulses() 反解，未实现 ⇒ 不授权
    if cmd["params"]:
        return ArbiterVerdict(reason="带数值参数的动作需量程闸门（软限位 + 反解），未授权",
                              err_code=ERR_ARBITER, command=cmd)

    # ⑥ 白名单：只有三个无参数命令字能真正让臂动
    if action == "home":
        return ArbiterVerdict(decision=Decision.ALLOW, payload=CMD_HOME,
                              tag="HOME", command=cmd,
                              reason="回初始位：固件原生命令字，无参数、已实机验证")

    if action == "look":
        return ArbiterVerdict(decision=Decision.NOOP, tag="LOOK_NOOP",
                              command=cmd, reason="只观察，不下发（会说话的东西不动臂）")

    if action in ("pick", "place"):
        return ArbiterVerdict(reason="抓取/放置需先定位 + 抓取序列（未实现），且涉及下降高度与 "
                                     "SAG_DZ 净空折算闸门 ⇒ 不授权",
                              err_code=ERR_ARBITER, command=cmd)

    if action in ("move_left", "move_right"):
        return ArbiterVerdict(reason="相对位移需读 STATE + 逐轴软限位裁剪 + to_pulses() 反解，未授权",
                              err_code=ERR_ARBITER, command=cmd)

    if action == "unknown":
        return ArbiterVerdict(reason="未识别到可执行动作（或命中导航/多动作，mock 不猜）",
                              err_code=ERR_UNKNOWN, command=cmd)

    # 兜底：意图层将来扩展了新动作，仲裁器默认**拒绝**，绝不「默认放行」
    return ArbiterVerdict(reason="动作 %r 不在仲裁白名单内 ⇒ 默认拒绝" % action,
                          err_code=ERR_UNKNOWN, command=cmd)


def build_reply(payload):
    """造一条应答帧 `$<payload>*<CK>\\r\\n`（纯函数）。

    CK 复用 llm/text_gateway.xor_checksum —— 逐 **UTF-8 字节** XOR，
    与 C 固件 `(uint8_t)payload[i]` 对齐。payload 全是 ASCII，不会踩中文坑。
    """
    return "$" + payload + "*%02X\r\n" % xor_checksum(payload)


# ---------------------------------------------------------------------------
# 下发层：唯一会碰硬件的地方，独立成一个函数方便单测替换
# ---------------------------------------------------------------------------
def send_cmd(payload, base_url=DEFAULT_BASE_URL, timeout=HTTP_TIMEOUT_S):
    """把固件命令字发给已实机验收的网页服务，并检查固件回执（会碰硬件）。

    🔴 为什么必须看 resp：`/api/cmd` 的 `ok:true` 只代表「串口写进去了」，
    固件真正执行结果在 `resp` 里（OK / ERR,2 未知命令 / ERR,4 急停中）。
    只判 ok 不看 resp，就会把「帧格式写错」当成「执行成功」——
    这条与 `action_primitives.send_joint()` 是同一套口径。
    """
    body = json.dumps({"payload": payload}).encode("utf-8")
    req = urllib.request.Request(
        base_url + "/api/cmd", data=body,
        headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            got = json.loads(resp.read().decode("utf-8"))
    except (urllib.error.URLError, OSError, ValueError) as e:
        raise RuntimeError("连不上 %s/api/cmd（Flask 没起？）：%s" % (base_url, e))

    if not got.get("ok"):
        raise RuntimeError("下发失败：%s" % got.get("error", got))
    ack = (got.get("resp") or "").strip()
    if ack != "OK":
        raise RuntimeError("固件拒绝执行（resp=%r），命令字 = %s" % (ack, payload))
    return ack


# ---------------------------------------------------------------------------
# 主流程：拆帧 → 急停闸门 → 仲裁 → 下发 → 造应答（不纯，但可离线单测）
# ---------------------------------------------------------------------------
def handle_frame(frame, estop_active=False, apply=False,
                 base_url=DEFAULT_BASE_URL, send_fn=send_cmd):
    """处理一帧，返回 `(应答帧, 日志字典, 新的急停状态)`。

    🔴 `apply=False`（默认）时**绝不调用 send_fn**，只在日志里记 `sent:false` ——
    把「会不会真动臂」从「靠人记」变成「显式开关」，与 eye_check 三条入场闸门同一思路。
    `send_fn` 做成参数是为了单测能塞一个假函数进来，验证「dry-run 一次都没调」。
    """
    frame = normalize_frame(frame)      # 先把字面量 \r\n 还原，见 normalize_frame 的坑说明

    # 🔴 长度闸门放在主流程里（不只 TCP handler）：将来若有人从别处直接调
    # handle_frame()，也不会绕过它。按 **UTF-8 字节**数算，中文一句顶 3 字节。
    if isinstance(frame, str) and len(frame.encode("utf-8")) > MAX_FRAME_BYTES:
        return (build_reply("ERR,%d" % ERR_FRAME),
                {"accepted": False, "reason": "frame_too_long",
                 "detail": "帧超过 %d 字节" % MAX_FRAME_BYTES, "sent": False},
                None)

    result = parse_frame(frame)
    special = special_text(frame, result)

    # 急停快速通道：非 ASR 的 $ESTOP*CK / $RESUME*CK 也能进来
    if special and not result.ok:
        text_for_arbiter = special
    elif result.ok:
        text_for_arbiter = special or result.text
    else:
        return (build_reply("ERR,%d" % ERR_FRAME),
                {"accepted": False, "reason": result.reason.value,
                 "detail": result.detail, "sent": False},
                None)

    verdict = arbitrate(text_for_arbiter, estop_active=estop_active)
    new_estop = verdict.set_estop

    record = {
        "accepted": verdict.decision != Decision.REJECT,
        "decision": verdict.decision.value,
        "text": text_for_arbiter,
        "command": verdict.command,
        "payload": verdict.payload,
        "reason": verdict.reason,
        "apply": bool(apply),
        "sent": False,
    }

    if verdict.decision == Decision.REJECT:
        record["err_code"] = verdict.err_code
        return (build_reply("ERR,%d" % verdict.err_code), record, new_estop)

    if verdict.decision == Decision.NOOP:
        return (build_reply("OK," + verdict.tag), record, new_estop)

    # Decision.ALLOW：只有这里才可能真下发
    if apply:
        try:
            send_fn(verdict.payload, base_url=base_url)
        except RuntimeError as e:
            record["downlink_error"] = str(e)
            return (build_reply("ERR,%d" % ERR_DOWNLINK), record, new_estop)
        record["sent"] = True
    else:
        record["dry_run_note"] = "本该下发 %s，因未加 --apply 未发" % verdict.payload

    return (build_reply("OK," + verdict.tag), record, new_estop)


# ---------------------------------------------------------------------------
# TCP 外壳：只负责收字节、维持急停状态、把结果写回去
# ---------------------------------------------------------------------------
class GatewayState:
    """服务端共享状态（急停标志 + 运行参数）。

    🔴 ThreadingTCPServer 是**多线程**的，多个 ESP32 同时连进来时，
    读写 `estop` 必须上锁，否则「A 刚判完没急停、B 紧接着发了急停」会漏判。
    规范定义：互斥量（mutex / threading.Lock）保护共享可变状态，避免竞态条件。
    """
    def __init__(self, apply=False, base_url=DEFAULT_BASE_URL):
        self.apply = apply
        self.base_url = base_url
        self._lock = threading.Lock()
        self._estop = False

    def get_estop(self):
        with self._lock:
            return self._estop

    def set_estop(self, value):
        with self._lock:
            self._estop = bool(value)


class _FrameHandler(socketserver.StreamRequestHandler):
    """一个客户端连接 = 一个线程；读一行、回一帧，简单到不会藏 bug。"""

    def handle(self):
        raw = self.rfile.readline(MAX_FRAME_BYTES + 1)
        if not raw:
            return
        if len(raw) > MAX_FRAME_BYTES:
            self._write(build_reply("ERR,%d" % ERR_FRAME))
            return
        try:
            frame = raw.decode("utf-8")
        except UnicodeDecodeError:
            self._write(build_reply("ERR,%d" % ERR_FRAME))
            return

        state = self.server.gateway_state
        reply, record, new_estop = handle_frame(
            frame, estop_active=state.get_estop(),
            apply=state.apply, base_url=state.base_url)
        if new_estop is not None:
            state.set_estop(new_estop)

        # 🔴 顺序：先发应答、后打日志。日志只是给人看的，绝不能因为
        #    「打印中文在 ASCII 终端崩了」把回给客户端的帧吞掉（11-14 实测）。
        #    _emit() 本身已做编码容错，这里再把顺序定死，双保险。
        self._write(reply)
        # 板上 nohup 重定向到日志时 stdout 是块缓冲，_emit 内 flush=True 保证看得见
        _emit(json.dumps(record, ensure_ascii=False))

    def _write(self, text):
        self.wfile.write(text.encode("ascii"))
        self.wfile.flush()


class GatewayServer(socketserver.ThreadingTCPServer):
    """多线程 TCP server；allow_reuse_address 让重启不必等 TIME_WAIT。"""
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, addr, state):
        super().__init__(addr, _FrameHandler)
        self.gateway_state = state


# ---------------------------------------------------------------------------
# 自测：零依赖、不起端口、不联网、不动臂
# ---------------------------------------------------------------------------
def self_test():
    """纯逻辑自测（轻量版；详细考卷在 test_voice_gateway.py）。"""
    # 1) 🔴 实机锚点：S2 已实证的 $PING*10 / $OK,PONG*3E 反查 XOR 实现
    assert xor_checksum("PING") == 0x10
    assert xor_checksum("OK,PONG") == 0x3E

    # 2) 好帧往返：造帧 → 拆帧，中文一字不差
    text = "把左边的红色杯子抓起来"
    assert parse_frame(build_frame(text)).text == text

    # 3) 仲裁白名单：只有 home 能真动，look 不动，pick/move 一律拒
    assert arbitrate("回初始位").payload == CMD_HOME
    assert arbitrate("描述一下当前画面").decision == Decision.NOOP
    assert arbitrate("把红块抓起来").decision == Decision.REJECT
    assert arbitrate("往左移一点").decision == Decision.REJECT
    assert arbitrate("今天天气不错").err_code == ERR_UNKNOWN

    # 4) 急停优先 + 急停中冻结 + 恢复解除
    assert arbitrate("急停").payload == CMD_STOP
    assert arbitrate("急停").set_estop is True
    frozen = arbitrate("回初始位", estop_active=True)
    assert frozen.decision == Decision.REJECT and frozen.err_code == ERR_ARBITER
    assert arbitrate("恢复").set_estop is False

    # 5) 非 ASR 急停快速通道（CK 仍由 parse_frame 校验）
    ck = xor_checksum("ESTOP")
    direct = "$ESTOP*%02X\r\n" % ck
    assert special_text(direct, parse_frame(direct)) == "急停", direct

    # 🔴 字面量 \r\n 归一化：shell 单引号粘出来的帧不能被误判成坏帧
    literal = direct.replace("\r\n", "\\r\\n")
    assert normalize_frame(literal) == direct
    _, rec_lit, _ = handle_frame(literal)
    assert rec_lit["payload"] == CMD_STOP, rec_lit

    # 6) dry-run 默认不下发：塞一个会「记仇」的假发送函数
    calls = []
    def fake_send(payload, base_url=None):
        calls.append(payload)
    reply, record, _ = handle_frame(build_frame("回初始位"), apply=False, send_fn=fake_send)
    assert calls == [], "dry-run 绝不能调用下发函数"
    assert record["sent"] is False and reply.startswith("$OK,HOME")

    # 7) --apply 才真发，且只发一个无参数命令字
    calls2 = []
    handle_frame(build_frame("回初始位"), apply=True, send_fn=lambda p, base_url=None: calls2.append(p))
    assert calls2 == [CMD_HOME], calls2

    # 8) 坏帧拒收且不带任何「已下发」标记
    reply_bad, record_bad, _ = handle_frame("$ASR,抓红块*00\r\n", send_fn=fake_send)
    assert reply_bad.startswith("$ERR,1") and record_bad["sent"] is False

    # 9) 意图层 7 键契约没被本网关污染
    cmd = arbitrate("回初始位").command
    assert set(cmd) == {"action", "target", "qualifier", "params",
                        "raw", "source", "warnings"}
    assert cmd["source"] == "mock" and cmd["params"] == {}

    _emit("SELF_TEST PASS")
    _emit(json.dumps(arbitrate("回初始位").command, ensure_ascii=False, indent=2))
    return 0


# ---------------------------------------------------------------------------
# 命令行外壳
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="鲲鹏侧语音入口网关：TCP 收帧 → 本地仲裁 → /api/cmd 下发（默认 dry-run）")
    ap.add_argument("--self-test", action="store_true", help="纯逻辑自测，不起端口不动臂")
    ap.add_argument("--make-frame", default=None, metavar="TEXT",
                    help="打印带正确 CK 的 $ASR 帧，供复制粘贴（不校验、不发送）")
    ap.add_argument("--text", action="append", default=None, metavar="TEXT",
                    help="单机喂一句（可重复给多次，按顺序走完整链路，默认 dry-run）")
    ap.add_argument("--frame", action="append", default=None, metavar="FRAME",
                    help="单机喂一整帧，含 $ 与 *CK（可重复）")
    ap.add_argument("--apply", action="store_true",
                    help="🔴 真下发（不加则只打印「本该发什么」，臂绝不会动）")
    ap.add_argument("--host", default=DEFAULT_HOST,
                    help="监听地址，默认 127.0.0.1；要 ESP32 连进来须显式 0.0.0.0")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT, help="监听端口，默认 9000")
    ap.add_argument("--base-url", default=DEFAULT_BASE_URL, help="网页服务地址")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    if args.make_frame is not None:
        _emit(repr(build_frame(args.make_frame)))
        _emit(build_frame(args.make_frame), newline=False)
        return 0

    # 单机喂帧模式：不起端口，方便 PC 上先把整条链路走一遍
    if args.text or args.frame:
        frames = []
        for t in (args.frame or []):
            frames.append(t)
        for t in (args.text or []):
            frames.append(build_frame(t))
        estop = False
        for f in frames:
            reply, record, new_estop = handle_frame(
                f, estop_active=estop, apply=args.apply, base_url=args.base_url)
            if new_estop is not None:
                estop = new_estop
            _emit("[frame] " + repr(f))
            _emit(json.dumps(record, ensure_ascii=False, indent=2))
            _emit("[reply] " + repr(reply))
        return 0

    # 默认：起 TCP server
    state = GatewayState(apply=args.apply, base_url=args.base_url)
    _emit("=" * 68)
    _emit("语音入口网关 · TCP %s:%d" % (args.host, args.port))
    _emit("模式：%s" % ("🔴 --apply 真下发（臂会动！）" if args.apply
                      else "✅ dry-run（只打印，臂绝不动）"))
    _emit("下发目标：%s/api/cmd" % args.base_url)
    _emit("急停词：%s　｜　恢复词：%s" % ("/".join(EMERGENCY_WORDS), "/".join(RESUME_WORDS)))
    _emit("本机自测：另开一个终端执行")
    _emit("  echo -ne '$ASR,回初始位*XXXX\\r\\n' | nc %s %d" % (args.host, args.port))
    _emit("  （先用 python3 voice_gateway.py --make-frame '回初始位' 拿到正确 CK）")
    _emit("=" * 68)
    with GatewayServer((args.host, args.port), state) as server:
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            _emit("\n网关已停止")
    return 0


if __name__ == "__main__":
    sys.exit(main())
