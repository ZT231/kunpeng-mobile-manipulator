#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_voice_gateway.py —— voice_gateway.py 的标准库 unittest 考卷

大白话：这份考卷专门考「门卫靠不靠谱」—— 好帧能不能过、坏帧会不会把程序带崩、
       急停是不是真最优先、dry-run 是不是一次都没偷偷下发。
       全程**不联网、不开端口、不动臂**，PC 上 `python3 -m unittest` 就能跑。
规范定义：单元测试隔离被测单元（此处用假 send_fn / patch to_dict 替换外部依赖），
         验证契约与边界；与本目录既有 test_action_primitives.py / test_pick_pipeline.py
         同一套「纯函数 + mock + 断言」风格。
"""

import os
import socket
import subprocess
import sys
import threading
import time
import unittest
from unittest.mock import patch

# 让 `python3 -m unittest discover -s .` 与本文件直接运行都能找到同目录模块
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import voice_gateway as vg          # noqa: E402
from text_gateway import xor_checksum   # noqa: E402

VG_PATH = os.path.join(_HERE, "voice_gateway.py")   # 供 ASCII stdout 子进程回归测试用


def _frame(text):
    """小工具：造一条合法 $ASR 帧。"""
    return vg.build_frame(text)


def _fake_contract(action, source="mock", params=None):
    """小工具：造一份意图层 7 键契约，供 patch to_dict 用。"""
    return {
        "action": action,
        "target": "red_block" if action in ("pick", "place") else None,
        "qualifier": None,
        "params": {} if params is None else params,
        "raw": "测试文本",
        "source": source,
        "warnings": [],
    }


class TestXorAndFrame(unittest.TestCase):
    """帧层：校验算法、往返、输入归一化、坏帧容错。"""

    def test_s2_real_anchor(self):
        """🔴 用 S2 已实机的 $PING*10 / $OK,PONG*3E 反查 XOR，避免自证循环。"""
        self.assertEqual(xor_checksum("PING"), 0x10)
        self.assertEqual(xor_checksum("OK,PONG"), 0x3E)

    def test_chinese_is_bytewise_xor(self):
        """中文按 UTF-8 逐字节 XOR，不是按 Unicode 码点（否则与 C 固件对不上）。"""
        manual = 0
        for b in "抓".encode("utf-8"):
            manual ^= b
        wrong = 0
        for ch in "抓":
            wrong ^= ord(ch)
        self.assertEqual(xor_checksum("抓"), manual & 0xFF)
        self.assertNotEqual(wrong & 0xFF, manual & 0xFF)

    def test_roundtrip_chinese(self):
        text = "把左边的红色杯子抓起来"
        r = vg.parse_frame(_frame(text))
        self.assertTrue(r.ok)
        self.assertEqual(r.text, text)

    def test_normalize_literal_crlf(self):
        """🔴 真踩过的坑：shell 单引号粘出来的是字面量 \\r\\n，不是真换行。"""
        real = "$ESTOP*5D\r\n"
        literal = "$ESTOP*5D\\r\\n"
        self.assertEqual(vg.normalize_frame(literal), real)
        # 归一化后必须能被当成合法急停帧，而不是 bad_ck_len
        _, rec, _ = vg.handle_frame(literal)
        self.assertEqual(rec["payload"], vg.CMD_STOP)

    def test_normalize_keeps_real_crlf_untouched(self):
        self.assertEqual(vg.normalize_frame("$ASR,x*00\r\n"), "$ASR,x*00\r\n")

    def test_normalize_order_matters(self):
        """先换 \\r\\n 再换 \\n，否则会残留一个字面量 \\r。"""
        self.assertEqual(vg.normalize_frame("a\\r\\nb\\n"), "a\r\nb\n")

    def test_bad_frames_all_rejected_without_crash(self):
        cases = (
            "$ASR,抓红块*00\r\n",     # CK 错
            "$ASR,回初始位\r\n",       # 缺 *
            "$ASR,抓红块*X\r\n",       # 校验位不足 2 位
            "$ASR,抓红块*ZZ\r\n",      # 非 HEX
            "ASR,抓红块*10\r\n",       # 缺 $
            _frame(""),                # 空文本
            "$PING*10\r\n",            # 命令字不是 ASR（且不是急停快速通道）
            None, 123,                 # 非字符串
        )
        for bad in cases:
            with self.subTest(frame=bad):
                reply, rec, _ = vg.handle_frame(bad)
                self.assertTrue(reply.startswith("$ERR,1"), (bad, reply))
                self.assertFalse(rec["sent"])

    def test_too_long_frame_rejected(self):
        """长度闸门在主流程里，绕过 TCP 层也拦得住。"""
        long_frame = "$ASR," + "x" * (vg.MAX_FRAME_BYTES + 10) + "*00\r\n"
        reply, rec, _ = vg.handle_frame(long_frame)
        self.assertTrue(reply.startswith("$ERR,1"))
        self.assertEqual(rec["reason"], "frame_too_long")


class TestArbiter(unittest.TestCase):
    """仲裁层：白名单、急停优先级、量程/净空闸门、默认拒绝。"""

    def test_home_is_the_only_motion_allowed(self):
        v = vg.arbitrate("回初始位")
        self.assertEqual(v.decision, vg.Decision.ALLOW)
        self.assertEqual(v.payload, vg.CMD_HOME)

    def test_look_is_noop_not_motion(self):
        v = vg.arbitrate("描述一下当前画面")
        self.assertEqual(v.decision, vg.Decision.NOOP)
        self.assertEqual(v.payload, "")

    def test_pick_and_place_rejected(self):
        for text in ("把红块抓起来", "把红色零件放进盒子"):
            with self.subTest(text=text):
                v = vg.arbitrate(text)
                self.assertEqual(v.decision, vg.Decision.REJECT)
                self.assertEqual(v.err_code, vg.ERR_ARBITER)
                self.assertIn("净空", v.reason)

    def test_move_rejected(self):
        for text in ("往左移一点", "向右转"):
            with self.subTest(text=text):
                v = vg.arbitrate(text)
                self.assertEqual(v.err_code, vg.ERR_ARBITER)
                self.assertIn("反解", v.reason)

    def test_unknown_and_navigation_rejected(self):
        self.assertEqual(vg.arbitrate("今天天气不错").err_code, vg.ERR_UNKNOWN)
        nav = vg.arbitrate("导航到A区再抓红块")
        self.assertEqual(nav.err_code, vg.ERR_UNKNOWN)

    def test_emergency_is_top_priority(self):
        """急停必须绕过意图层，且立刻置 estop 标志。"""
        v = vg.arbitrate("急停")
        self.assertEqual(v.decision, vg.Decision.ALLOW)
        self.assertEqual(v.payload, vg.CMD_STOP)
        self.assertIs(v.set_estop, True)
        # 即使在急停中，再发一次急停仍允许（幂等）
        self.assertEqual(vg.arbitrate("急停", estop_active=True).payload, vg.CMD_STOP)

    def test_estop_freezes_everything_except_resume(self):
        v = vg.arbitrate("回初始位", estop_active=True)
        self.assertEqual(v.decision, vg.Decision.REJECT)
        self.assertEqual(v.err_code, vg.ERR_ARBITER)
        self.assertIn("急停中", v.reason)
        # 恢复仍允许
        self.assertEqual(vg.arbitrate("恢复", estop_active=True).payload, vg.CMD_RESUME)

    def test_resume_clears_flag(self):
        v = vg.arbitrate("恢复")
        self.assertIs(v.set_estop, False)
        self.assertEqual(v.payload, vg.CMD_RESUME)

    def test_non_asr_estop_fast_channel(self):
        """将来 ESP32 可直接发 $ESTOP*CK / $RESUME*CK，不该被 NOT_ASR 丢掉。"""
        ck = xor_checksum("ESTOP")
        frame = "$ESTOP*%02X\r\n" % ck
        self.assertEqual(vg.special_text(frame, vg.parse_frame(frame)), "急停")
        ck2 = xor_checksum("RESUME")
        frame2 = "$RESUME*%02X\r\n" % ck2
        self.assertEqual(vg.special_text(frame2, vg.parse_frame(frame2)), "恢复")

    def test_non_mock_source_rejected(self):
        """🔴 将来偷偷接了真模型，也不能绕过本地仲裁。"""
        with patch.object(vg, "to_dict", return_value=_fake_contract("home", source="cloud")):
            v = vg.arbitrate("随便一句")
        self.assertEqual(v.err_code, vg.ERR_ARBITER)
        self.assertIn("非 mock", v.reason)

    def test_params_rejected_by_range_gate(self):
        """带数值参数 ⇒ 需要软限位 + 反解，未实现就不授权。"""
        with patch.object(vg, "to_dict",
                          return_value=_fake_contract("move_left", params={"dist": 10})):
            v = vg.arbitrate("随便一句")
        self.assertEqual(v.err_code, vg.ERR_ARBITER)
        self.assertIn("量程", v.reason)

    def test_future_action_defaults_to_reject(self):
        """意图层将来扩展新动作，仲裁器默认拒绝，绝不默认放行。"""
        with patch.object(vg, "to_dict", return_value=_fake_contract("fly")):
            v = vg.arbitrate("随便一句")
        self.assertEqual(v.decision, vg.Decision.REJECT)
        self.assertEqual(v.err_code, vg.ERR_UNKNOWN)

    def test_resume_word_containing_estop_is_not_misread(self):
        """🔴 「解除急停」含「急停」二字，但它是恢复词，不能反向判成急停。

        11-14 审查实测：旧裸子串匹配下 arbitrate("解除急停") 返回 STOP，
        喊「解除」反而又急停一次。special_text 与 arbitrate 两层都要修好。
        """
        for text in ("解除急停", "继续", "恢复"):
            with self.subTest(text=text):
                v = vg.arbitrate(text)
                self.assertEqual(v.payload, vg.CMD_RESUME, (text, v))
                self.assertIs(v.set_estop, False)
                fr = vg.build_frame(text)
                self.assertEqual(vg.special_text(fr, vg.parse_frame(fr)), "恢复")
                reply, _, _ = vg.handle_frame(fr)
                self.assertEqual(reply, "$OK,RESUMED*75\r\n")

    def test_ascii_words_recognized_by_arbiter_itself(self):
        """纯函数层也要自洽：小写 stop/resume 不能只在 special_text 里认得。"""
        self.assertEqual(vg.arbitrate("stop").payload, vg.CMD_STOP)
        self.assertEqual(vg.arbitrate("STOP").payload, vg.CMD_STOP)
        self.assertEqual(vg.arbitrate("ESTOP").payload, vg.CMD_STOP)
        self.assertEqual(vg.arbitrate("resume").payload, vg.CMD_RESUME)
        self.assertEqual(vg.arbitrate("RESUME").payload, vg.CMD_RESUME)

    def test_empty_input_rejected(self):
        for bad in ("", "   ", None, 123):
            with self.subTest(text=bad):
                self.assertEqual(vg.arbitrate(bad).err_code, vg.ERR_FRAME)


class TestHandleFrame(unittest.TestCase):
    """主流程：dry-run 边界、真下发、下发失败容错、应答帧、契约。"""

    def test_dry_run_never_calls_sender(self):
        calls = []
        reply, rec, _ = vg.handle_frame(
            _frame("回初始位"), apply=False,
            send_fn=lambda p, base_url=None: calls.append(p))
        self.assertEqual(calls, [])
        self.assertFalse(rec["sent"])
        self.assertTrue(reply.startswith("$OK,HOME"))

    def test_apply_sends_exactly_one_parameterless_cmd(self):
        calls = []
        vg.handle_frame(_frame("回初始位"), apply=True,
                        send_fn=lambda p, base_url=None: calls.append(p))
        self.assertEqual(calls, [vg.CMD_HOME])
        # 精确断言 calls==[CMD_HOME] 已保证只发 $ARM,HOME、不含任何位置参数；
        # $ARM 子命令里的逗号是分隔符，与旧 $MOVE,0,<脉宽> 的越界风险无关。

    def test_noop_never_sends_even_with_apply(self):
        calls = []
        vg.handle_frame(_frame("描述一下当前画面"), apply=True,
                        send_fn=lambda p, base_url=None: calls.append(p))
        self.assertEqual(calls, [])

    def test_downlink_failure_returns_err4_without_crash(self):
        def boom(payload, base_url=None):
            raise RuntimeError("Flask 没起（假失败）")
        reply, rec, _ = vg.handle_frame(_frame("回初始位"), apply=True, send_fn=boom)
        self.assertTrue(reply.startswith("$ERR,4"), reply)
        self.assertFalse(rec["sent"])
        self.assertIn("Flask 没起", rec["downlink_error"])

    def test_reply_frame_checksum_is_valid(self):
        for payload in ("OK,HOME", "OK,ESTOP", "OK,RESUMED", "ERR,1", "ERR,3", "ERR,4"):
            with self.subTest(payload=payload):
                reply = vg.build_reply(payload)
                self.assertEqual(reply, "$%s*%02X\r\n" % (payload, xor_checksum(payload)))

    def test_intent_contract_7_keys_not_polluted(self):
        _, rec, _ = vg.handle_frame(_frame("回初始位"))
        cmd = rec["command"]
        self.assertEqual(set(cmd), {"action", "target", "qualifier", "params",
                                    "raw", "source", "warnings"})
        self.assertEqual(cmd["source"], "mock")
        self.assertEqual(cmd["params"], {})

    def test_reject_record_has_no_sent_flag_true(self):
        _, rec, _ = vg.handle_frame(_frame("把红块抓起来"))
        self.assertFalse(rec["accepted"])
        self.assertFalse(rec["sent"])
        self.assertEqual(rec["err_code"], vg.ERR_ARBITER)

    def test_estop_state_propagates_to_caller(self):
        """仲裁器是纯函数，改状态由外层做：验证 new_estop 被正确返回。"""
        _, _, e1 = vg.handle_frame(_frame("急停"))
        self.assertIs(e1, True)
        _, _, e2 = vg.handle_frame(_frame("回初始位"), estop_active=e1)
        self.assertIsNone(e2)                 # 被冻结，不改状态
        _, _, e3 = vg.handle_frame(_frame("恢复"), estop_active=e1)
        self.assertIs(e3, False)

    def test_self_test_returns_zero(self):
        self.assertEqual(vg.self_test(), 0)


class TestGatewayState(unittest.TestCase):
    """TCP 外壳的共享状态：多线程下急停标志不能错乱。"""

    def test_toggle(self):
        s = vg.GatewayState(apply=False)
        self.assertFalse(s.get_estop())
        s.set_estop(True)
        self.assertTrue(s.get_estop())
        s.set_estop(False)
        self.assertFalse(s.get_estop())

    def test_concurrent_toggle_does_not_crash(self):
        s = vg.GatewayState()
        def worker(val):
            for _ in range(200):
                s.set_estop(val)
        threads = [threading.Thread(target=worker, args=(i % 2 == 0,)) for i in range(8)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        self.assertIn(s.get_estop(), (True, False))


class TestAsciiStdout(unittest.TestCase):
    """🔴 板端 LANG=C / PYTHONIOENCODING=ascii 回归（11-14 审查发现）。

    大白话：本机是 UTF-8，中文 print 一路绿灯；板子若设成纯 ASCII，
           启动横幅崩了服务就起不来、--self-test 会 rc=1、handler 日志
           崩了还会把应答一起吞掉。这组测试用子进程强制 ASCII stdout，
           把「上板才暴露」的问题锁死在本机回归里。
    规范定义：用 env 覆盖 PYTHONIOENCODING 模拟目标 locale，subprocess
           起真进程跑 CLI / 起真 TCP server，属于端到端回归（不再只是纯函数）。
    """

    @staticmethod
    def _ascii_env():
        env = dict(os.environ)
        env["PYTHONIOENCODING"] = "ascii"   # 关键：强制 stdout 只认 ASCII
        return env

    def test_self_test_survives_ascii_stdout(self):
        """验收规程 1.3 的板端自检命令，ASCII 下必须 rc=0。"""
        r = subprocess.run(
            [sys.executable, "-B", VG_PATH, "--self-test"],
            env=self._ascii_env(), stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, timeout=30)
        self.assertEqual(r.returncode, 0,
                         r.stderr.decode("utf-8", "replace"))
        self.assertIn(b"SELF_TEST PASS", r.stdout)

    def test_make_frame_survives_ascii_stdout(self):
        """验收规程 3.3 用它核对 CK=61，ASCII 下也不能崩。"""
        r = subprocess.run(
            [sys.executable, "-B", VG_PATH, "--make-frame", "回初始位"],
            env=self._ascii_env(), stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, timeout=30)
        self.assertEqual(r.returncode, 0,
                         r.stderr.decode("utf-8", "replace"))
        self.assertIn(b"*61", r.stdout)     # 帧本身纯 ASCII，CK 仍是 61

    def test_emit_fallback_is_ascii_safe(self):
        """没有 reconfigure 的 stdout（老 Python / 被包装）也不能抛编码异常。"""
        class AsciiOnly:
            def __init__(self):
                self.written = []
            def write(self, s):
                s.encode("ascii")           # 非 ASCII ⇒ UnicodeEncodeError
                self.written.append(s)
                return len(s)
            def flush(self):
                pass
        fake = AsciiOnly()
        with patch.object(vg.sys, "stdout", fake):
            vg._emit("中文 🔴")            # 不许抛
        joined = "".join(fake.written)
        self.assertTrue(all(ord(c) < 128 for c in joined))
        self.assertIn("\\u4e2d", joined)   # 中文被转成 \uXXXX 转义

    def test_tcp_reply_survives_ascii_stdout(self):
        """🔴 最硬一条：ASCII stdout 下 server 能起、且中文帧仍能收到应答。"""
        env = self._ascii_env()
        probe = socket.socket()            # 让系统挑空闲端口，避免撞车
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
        probe.close()

        proc = subprocess.Popen(
            [sys.executable, "-B", VG_PATH, "--port", str(port)],
            env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        try:
            ready = False
            for _ in range(100):
                try:
                    c = socket.create_connection(("127.0.0.1", port), timeout=0.2)
                    c.close()
                    ready = True
                    break
                except OSError:
                    if proc.poll() is not None:
                        self.fail("网关在 ASCII stdout 下启动即退出，rc=%s" %
                                  proc.returncode)
                    time.sleep(0.05)
            self.assertTrue(ready, "网关 5 秒内没监听端口")

            s = socket.create_connection(("127.0.0.1", port), timeout=3)
            try:
                s.sendall(vg.build_frame("回初始位").encode("utf-8"))
                s.settimeout(3)
                data = s.recv(256)
            finally:
                s.close()
            self.assertEqual(data, b"$OK,HOME*27\r\n")
        finally:
            proc.terminate()
            try:
                proc.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.communicate()


if __name__ == "__main__":
    unittest.main(verbosity=2)
