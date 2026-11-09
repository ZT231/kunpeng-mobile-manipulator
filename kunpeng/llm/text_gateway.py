#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
text_gateway.py —— 阶段 2 补充：模拟 ESP32-S3 串口文本入口（离线联调，只打印不下发）

大白话：将来 ESP32-S3 把「人说的那句话」从串口发给鲲鹏。现在板子还没接进主链路，
       我们就手写一句话，套上项目统一的串口帧格式，走一遍
       「收帧 → 校验 → 抽出文本 → 翻译员翻成固定 JSON → 打印」。
       全程不下发、不联网、不碰机械臂，纯本地先把「入口契约」立住。
规范定义：这是**协议适配层 / 网关（gateway）**的离线替身（Test Double）。
         输入是受限帧格式的字符串，输出是 intent_parser 的固定键 JSON 契约；
         分层为「纯函数校验 → 纯函数解析 → 薄薄的命令行外壳」，与阶段 1
         telemetry_bridge.py 的纯函数 + 自测风格一致。

🔴 帧格式（**建议草案，未与 ESP32 实机对齐、未上板、未接真串口**）：

    $ASR,<识别出来的文本>*<CK><CR><LF>

  - ASR = Automatic Speech Recognition，语音识别结果；用命令字而不是裸文本，
    是为了将来能扩展 $WAKE / $STATUS 等，也和项目统一帧协议一致。
  - CK = payload（'$' 与 '*' 之间）**所有字节**逐字节 XOR，2 位大写十六进制。
    依据：Core/Src/uart_protocol.c L20-21 帧注释、L98-133 proto_send_frame
    发包、L522-526 proto_parse_frame 收包校验，三处完全一致。
  - 实机锚点（S2 已实证）：`$PING*10` → `$OK,PONG*3E`，
    本文件 --self-test 用这两帧反查 XOR 实现，不是自说自话。

🔴 这是 **ESP32-S3 → 鲲鹏** 的链路，不是 STM32 ↔ 鲲鹏 的 `$JOINT` 链路，
   **不进固件 uart_protocol.c**，由鲲鹏侧独立 Python 进程接收。

🔴 诚实边界（README 里也写一份，别越线）：
  1) 只打印、**不下发**：模块里根本没有下发函数， CloudBackend 也不碰；
  2) `source` 恒 "mock"，防止以后看日志误以为接了真大模型；
  3) 帧格式是草案、未实机对齐 ⇒ 不能写「语音控制已实现 / 语音→动作闭环」；
  4) 这一步只是把「ESP32 文本入口」的形状验对，**没有接入点选抓取主链路**。

用法（零第三方依赖、不联网、不读 Key）：
  python3 text_gateway.py --self-test                # 纯逻辑自测
  python3 text_gateway.py                           # 跑内置演示帧（正常 + 坏帧）
  python3 text_gateway.py --text "把红块抓起来"      # 模拟 ESP32 发来一句
  python3 text_gateway.py --frame '$ASR,回初始位*XX' # 手动喂一整帧
"""

import argparse
import dataclasses
import enum
import json
import sys

# 只从同目录导入现成的翻译员 —— 本文件不重复实现解析逻辑，
# 改词表 / 换后端只在 intent_parser.py 一处改，这里是纯粹的「入口」。
from intent_parser import parse_intent, to_dict


# ---------------------------------------------------------------------------
# 数据结构先定义：把「为什么会拒收」关进固定集合，别散落成一堆字符串
# ---------------------------------------------------------------------------
class RejectReason(enum.Enum):
    """拒收原因枚举。

    为什么不用裸字符串：写错 "ck_mismath" 不报错、日志里也看不出来；
    枚举拼错立刻 AttributeError，而且所有可能原因一眼看全（背书面试用）。
    """
    OK = "ok"
    NO_START = "no_start"
    NO_STAR = "no_star"
    BAD_CK_LEN = "bad_ck_len"
    BAD_CK_HEX = "bad_ck_hex"
    CK_MISMATCH = "ck_mismatch"
    NOT_ASR = "not_asr"
    EMPTY_TEXT = "empty_text"
    NON_STRING = "non_string"


@dataclasses.dataclass
class FrameResult:
    """一帧的解析结果（带名字的数据袋子）。

    🔴 用 @dataclasses.dataclass 而不是裸 dict：字段有名字和默认值，
    写错字段名立刻报错；自带 __repr__ / __eq__，断言和打印都好读。
    """
    ok: bool = False
    text: str = ""
    reason: RejectReason = RejectReason.NO_START
    detail: str = ""


# ---------------------------------------------------------------------------
# 纯函数层：不联网、不读环境变量、不写文件、同样输入永远同样输出
# ---------------------------------------------------------------------------
def xor_checksum(payload):
    """算 payload 的 CK：把 **UTF-8 字节**逐个 XOR，返回 0~255 的整数（纯函数）。

    🔴 本项目最容易踩的坑（写成 --self-test 的第 3 条断言）：
      C 固件里是 `ck ^= (uint8_t)payload[i]`（uart_protocol.c L123），
      对的是**字节**；中文「抓」在 UTF-8 里是 3 个字节 E6 8A 93。
      如果 Python 写成 `for ch in text: ck ^= ord(ch)`，拿到的是
      Unicode 码点 0x6293（25235），早就超出 1 字节，算出来的 CK
      和固件对不上，将来真接上 ESP32 会 100% 被判 ERR,2 校验失败。
      ⇒ 必须先 `.encode("utf-8")` 变成 bytes 再逐字节 XOR。
    规范定义：XOR 校验（longitudinal redundancy check 的最简形式）定义在
             字节序列上；跨语言对齐时须统一字符编码（此处 UTF-8）与字长（8 bit）。
    """
    ck = 0
    for byte in payload.encode("utf-8"):
        ck ^= byte
    return ck & 0xFF


def build_frame(text):
    """把一句话包成 $ASR,...*CK\\r\\n（纯函数，给演示和 --text 用）。

    大白话：模拟「ESP32 那边把识别结果按协议发出去」这个动作。
    """
    payload = "ASR," + (text if isinstance(text, str) else "")
    return "$" + payload + "*%02X\r\n" % xor_checksum(payload)


def parse_frame(frame):
    """校验并拆开一帧，返回 FrameResult（纯函数，绝不抛异常给上层）。

    为什么不抛异常：将来调用方是常驻的串口接收进程，一条坏帧
    （断线半包、干扰错码）不该把整个进程带崩 —— 与 intent_parser
    的 validate() 同一套「返回原因、容错不失控」思路。
    """
    if not isinstance(frame, str):
        return FrameResult(reason=RejectReason.NON_STRING, detail="帧不是字符串")

    if not frame.startswith("$"):
        return FrameResult(reason=RejectReason.NO_START, detail="缺起始符 $")

    # 去掉尾部回车换行；rstrip 顺手容忍多余/缺失的 \r\n
    body = frame[1:].rstrip("\r\n")
    star = body.find("*")
    if star < 0:
        return FrameResult(reason=RejectReason.NO_STAR, detail="缺分隔符 *")

    payload = body[:star]
    ck_text = body[star + 1:].upper()   # 固件发大写；接收端宽容小写，再统一校验

    if len(ck_text) != 2:
        return FrameResult(reason=RejectReason.BAD_CK_LEN,
                           detail="校验位不是 2 位（需 '*' + 2 位 HEX）")
    if not all(ch in "0123456789ABCDEF" for ch in ck_text):
        return FrameResult(reason=RejectReason.BAD_CK_HEX,
                           detail="校验位不是十六进制字符")

    calc = xor_checksum(payload)
    got = int(ck_text, 16)
    if calc != got:
        return FrameResult(reason=RejectReason.CK_MISMATCH,
                           detail="CK 不符：算出 %02X，帧里写 %s" % (calc, ck_text))

    # maxsplit=1 很关键：识别文本里可能有逗号，只切第一个逗号分出命令字，
    # 剩下的整段原样当文本。直接 split(",") 会把一句话切碎（新手常踩）。
    if "," in payload:
        cmd, text = payload.split(",", 1)
    else:
        cmd, text = payload, ""

    if cmd != "ASR":
        return FrameResult(reason=RejectReason.NOT_ASR,
                           detail="命令字是 %s，本网关只收 ASR" % cmd)
    if not text.strip():
        return FrameResult(reason=RejectReason.EMPTY_TEXT, detail="识别文本为空")

    return FrameResult(ok=True, text=text, reason=RejectReason.OK)


def handle_frame(frame):
    """网关主流程：收帧 → 校验 → 意图解析 → **只返回结果，不下发**（纯函数）。

    返回形状是网关层自己的包装，不污染 intent_parser 的 7 个固定键契约：
      成功 {"accepted": True,  "command": {7 个固定键}}
      拒收 {"accepted": False, "reason": ..., "detail": ...}
    🔴 这里没有任何 publish / send / HTTP / 串口写调用 —— 阶段 2 的边界就在这。
    """
    result = parse_frame(frame)
    if not result.ok:
        return {"accepted": False,
                "reason": result.reason.value,
                "detail": result.detail}
    return {"accepted": True, "command": to_dict(parse_intent(result.text))}


def demo_frames():
    """内置演示帧：4 条正常 + 5 条坏帧，专门演示「容错不失控」。"""
    good = (
        "把左边的红色杯子抓起来",
        "把红色零件放进盒子",
        "回初始位",
        "导航到A区再抓红块",   # 会被意图层拦成 unknown，演示「不假装能执行」
    )
    frames = [build_frame(t) for t in good]
    frames += [
        "$ASR,抓红块*00\r\n",      # 故意写错 CK
        "$ASR,回初始位\r\n",        # 缺 *
        "$ASR,抓红块*X\r\n",       # 校验位不足 2 位
        "ASR,抓红块*10\r\n",       # 缺 $
        build_frame(""),           # 文本为空
    ]
    return tuple(frames)


# ---------------------------------------------------------------------------
# 自测：零依赖、不联网，还拿 S2 实机帧当锚点
# ---------------------------------------------------------------------------
def self_test():
    """纯逻辑自测：正常 / 中文 / 坏帧 / 契约 / 不下发边界，全用 assert。"""
    # 1) 🔴 实机锚点：S2 已实证的 $PING*10 与 $OK,PONG*3E
    #    用真实跑通过的帧反查 XOR 实现，避免「自己算自己判」的自证循环。
    assert xor_checksum("PING") == 0x10, "CK(PING) 应等于 $PING*10 里的 10"
    assert xor_checksum("OK,PONG") == 0x3E, "CK(OK,PONG) 应等于 $OK,PONG*3E 里的 3E"

    # 2) 中文帧往返：造帧 → 拆帧，文本一字不差
    text = "把左边的红色杯子抓起来"
    frame = build_frame(text)
    r = parse_frame(frame)
    assert r.ok is True and r.text == text, (frame, r)

    # 3) 🔴 中文必须按字节 XOR，不是按 Unicode 码点（否则和固件对不上）
    wrong_ck = 0
    for ch in "抓":
        wrong_ck ^= ord(ch)          # 错误做法：逐字符，拿的是 Unicode 码点
    wrong_ck &= 0xFF
    manual = 0
    for b in "抓".encode("utf-8"):
        manual ^= b                  # 正确做法：逐 UTF-8 字节，对齐 C 的 uint8_t
    assert len("抓".encode("utf-8")) == 3, "前提：中文是 UTF-8 多字节"
    assert xor_checksum("抓") == manual, "中文 CK 必须逐 UTF-8 字节算"
    assert wrong_ck != manual, "逐字符与逐字节结果必须不同，否则这条坑不成立"

    # 4) 坏帧逐类拒收，且都不许抛异常
    cases = (
        ("$ASR,抓红块*00\r\n", RejectReason.CK_MISMATCH),
        ("$ASR,回初始位\r\n", RejectReason.NO_STAR),
        ("$ASR,抓红块*X\r\n", RejectReason.BAD_CK_LEN),
        ("$ASR,抓红块*ZZ\r\n", RejectReason.BAD_CK_HEX),
        ("ASR,抓红块*10\r\n", RejectReason.NO_START),
        (build_frame(""), RejectReason.EMPTY_TEXT),
        ("$PING*10\r\n", RejectReason.NOT_ASR),
        (None, RejectReason.NON_STRING),
        (123, RejectReason.NON_STRING),
    )
    for bad, want in cases:
        got = parse_frame(bad)
        assert got.ok is False and got.reason == want, (bad, got, want)

    # 5) 网关输出形状 + 意图契约不被污染
    ok_out = handle_frame(build_frame("把左边的红色杯子抓起来"))
    assert ok_out["accepted"] is True
    cmd = ok_out["command"]
    assert cmd["action"] == "pick" and cmd["target"] == "red_cup"
    assert cmd["qualifier"] == "left"
    assert cmd["source"] == "mock" and cmd["params"] == {}
    assert set(cmd) == {"action", "target", "qualifier", "params",
                        "raw", "source", "warnings"}, "意图契约 7 键必须固定"

    bad_out = handle_frame("$ASR,抓红块*00\r\n")
    assert bad_out["accepted"] is False and "command" not in bad_out

    # 6) 导航词不假装能执行（借用意图层既有兜底，证明两层接得上）
    nav = handle_frame(build_frame("导航到A区再抓红块"))["command"]
    assert nav["action"] == "unknown"
    assert any("导航" in w for w in nav["warnings"])

    # 7) 只打印不下发：网关层不产生任何「已下发」标记
    assert "sent" not in ok_out and "published" not in ok_out

    print("SELF_TEST PASS")
    print(json.dumps(ok_out, ensure_ascii=False, indent=2))
    return 0


# ---------------------------------------------------------------------------
# 命令行外壳：只负责把参数变成帧、把结果打印出来
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="模拟 ESP32-S3 文本入口 → 意图解析（离线，只打印不下发）")
    parser.add_argument("--self-test", action="store_true", help="纯逻辑自测，不联网")
    parser.add_argument("--text", default=None, help="模拟 ESP32 发来的一句识别文本")
    parser.add_argument("--frame", default=None, help="手动喂一整帧（含 $ 和 *CK）")
    args = parser.parse_args()

    if args.self_test:
        return self_test()

    if args.frame is not None:
        frames = [args.frame]
    elif args.text is not None:
        frames = [build_frame(args.text)]
    else:
        frames = list(demo_frames())

    for frame in frames:
        print("[frame] " + repr(frame))
        print(json.dumps(handle_frame(frame), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
