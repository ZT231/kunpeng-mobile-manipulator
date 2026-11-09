#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_text_gateway.py —— text_gateway 的标准库单元测试

大白话：给「ESP32 文本入口」出一张考卷，把该收下的、该拒收的、
       该原样保留的、以及「不许偷偷下发」全列出来自动判分。
规范定义：unittest（Python 标准库，xUnit 风格），TestCase + assert* 断言；
         `python -m unittest discover -s . -p "test_*.py" -v` 自动发现执行。

和 test_intent_parser.py 的分工：那边考「翻译员翻得对不对」，
这边考「门口收发室收得对不对」。两层分开测，出问题一眼分清是哪层。
"""

import unittest

from text_gateway import (
    FrameResult,
    RejectReason,
    build_frame,
    demo_frames,
    handle_frame,
    parse_frame,
    xor_checksum,
)

# intent_parser 的 7 个固定键，网关不许污染它
CONTRACT_KEYS = {"action", "target", "qualifier", "params",
                 "raw", "source", "warnings"}


class TestChecksum(unittest.TestCase):
    """校验和层：必须和 C 固件逐字节对齐。"""

    def test_s2_real_frames_are_anchors(self):
        """🔴 用 S2 已实机跑通的 $PING*10 / $OK,PONG*3E 反查实现。"""
        self.assertEqual(xor_checksum("PING"), 0x10)
        self.assertEqual(xor_checksum("OK,PONG"), 0x3E)

    def test_empty_payload_is_zero(self):
        self.assertEqual(xor_checksum(""), 0)

    def test_chinese_is_bytewise_not_codepoint(self):
        """🔴 中文是 UTF-8 多字节：按字节算才和 (uint8_t) 对齐。"""
        manual = 0
        for b in "抓".encode("utf-8"):
            manual ^= b
        wrong = 0
        for ch in "抓":
            wrong ^= ord(ch)
        self.assertEqual(len("抓".encode("utf-8")), 3)
        self.assertEqual(xor_checksum("抓"), manual)
        self.assertNotEqual(wrong & 0xFF, manual)


class TestRoundTrip(unittest.TestCase):
    """造帧 → 拆帧：文本必须一字不差。"""

    def test_ascii_and_chinese(self):
        for text in ("回初始位", "把左边的红色杯子抓起来", "home"):
            with self.subTest(text=text):
                r = parse_frame(build_frame(text))
                self.assertTrue(r.ok)
                self.assertEqual(r.text, text)
                self.assertEqual(r.reason, RejectReason.OK)

    def test_comma_in_text_is_kept(self):
        """maxsplit=1：识别文本里的逗号不许被切碎（新手常踩）。"""
        text = "抓红块，然后放下"
        r = parse_frame(build_frame(text))
        self.assertTrue(r.ok)
        self.assertEqual(r.text, text)

    def test_frame_shape(self):
        frame = build_frame("抓红块")
        self.assertTrue(frame.startswith("$ASR,"))
        self.assertTrue(frame.endswith("\r\n"))
        self.assertEqual(len(frame.split("*")[1].strip()), 2)
        self.assertEqual(frame.split("*")[1].strip(),
                         frame.split("*")[1].strip().upper())


class TestReject(unittest.TestCase):
    """坏帧逐类拒收，且绝不抛异常把常驻进程带崩。"""

    def test_each_reason(self):
        cases = (
            ("$ASR,抓红块*00\r\n", RejectReason.CK_MISMATCH),
            ("$ASR,回初始位\r\n", RejectReason.NO_STAR),
            ("$ASR,抓红块*X\r\n", RejectReason.BAD_CK_LEN),
            ("$ASR,抓红块*ZZ\r\n", RejectReason.BAD_CK_HEX),
            ("ASR,抓红块*10\r\n", RejectReason.NO_START),
            ("$PING*10\r\n", RejectReason.NOT_ASR),
            (build_frame(""), RejectReason.EMPTY_TEXT),
        )
        for frame, want in cases:
            with self.subTest(frame=frame):
                r = parse_frame(frame)
                self.assertFalse(r.ok)
                self.assertEqual(r.reason, want)
                self.assertTrue(r.detail)

    def test_non_string_never_crashes(self):
        for bad in (None, 123, [], {}):
            with self.subTest(bad=bad):
                self.assertEqual(parse_frame(bad).reason, RejectReason.NON_STRING)

    def test_result_is_dataclass(self):
        """结果对象有名字、有默认值，不是裸 dict。"""
        self.assertIsInstance(parse_frame(build_frame("回初始位")), FrameResult)
        self.assertEqual(FrameResult().reason, RejectReason.NO_START)


class TestGateway(unittest.TestCase):
    """网关输出形状 + 不污染意图契约。"""

    def test_accept_shape(self):
        out = handle_frame(build_frame("把左边的红色杯子抓起来"))
        self.assertTrue(out["accepted"])
        self.assertIn("command", out)

    def test_contract_keys_fixed(self):
        cmd = handle_frame(build_frame("抓红块"))["command"]
        self.assertEqual(set(cmd), CONTRACT_KEYS)

    def test_action_target_qualifier(self):
        cmd = handle_frame(build_frame("把左边的红色杯子抓起来"))["command"]
        self.assertEqual(cmd["action"], "pick")
        self.assertEqual(cmd["target"], "red_cup")
        self.assertEqual(cmd["qualifier"], "left")

    def test_source_mock_and_params_empty(self):
        cmd = handle_frame(build_frame("回初始位"))["command"]
        self.assertEqual(cmd["source"], "mock")
        self.assertEqual(cmd["params"], {})

    def test_raw_kept_verbatim(self):
        text = "把红色零件放进盒子"
        self.assertEqual(handle_frame(build_frame(text))["command"]["raw"], text)

    def test_nav_is_unknown_not_silently_mapped(self):
        """两层接得上：导航词在意图层被拦成 unknown + warning。"""
        cmd = handle_frame(build_frame("导航到A区再抓红块"))["command"]
        self.assertEqual(cmd["action"], "unknown")
        self.assertTrue(any("导航" in w for w in cmd["warnings"]))

    def test_bad_frame_has_no_command(self):
        out = handle_frame("$ASR,抓红块*00\r\n")
        self.assertFalse(out["accepted"])
        self.assertEqual(out["reason"], "ck_mismatch")
        self.assertNotIn("command", out)


class TestBoundary(unittest.TestCase):
    """诚实边界：只打印不下发、内置演示帧全都不许崩。"""

    def test_no_downstream_marker(self):
        out = handle_frame(build_frame("抓红块"))
        self.assertNotIn("sent", out)
        self.assertNotIn("published", out)

    def test_all_demo_frames_handled(self):
        results = [handle_frame(f) for f in demo_frames()]
        self.assertTrue(any(r["accepted"] for r in results))
        self.assertTrue(any(not r["accepted"] for r in results))


if __name__ == "__main__":
    unittest.main(verbosity=2)
