#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_intent_parser.py —— intent_parser 的标准库单元测试

大白话：给「翻译员」出一张考卷，把该翻对的、该兜底的、不该崩的
       全列出来，一条条自动判分。
规范定义：unittest（Python 标准库，xUnit 风格），TestCase + assert* 断言；
         `python -m unittest discover -p "test_*.py"` 自动发现并执行。

为什么单独一个文件、不只用 --self-test：
  --self-test 是给命令行 / 板上一眼看结果的冒烟测试；
  这里用 unittest 覆盖边界与负例，失败时能报「哪一条、期望什么、实际什么」，
  面试时也能直接说「我给意图解析层写了单元测试」。
"""

import unittest

from intent_parser import (
    Action,
    CloudBackend,
    IntentCommand,
    MOCK_BACKEND,
    MockBackend,
    ParserBackend,
    parse_intent,
    to_dict,
    validate,
)

# 契约键集合：下游按这些键取值，多一个少一个都算破坏契约
CONTRACT_KEYS = {"action", "target", "qualifier", "params", "raw", "source", "warnings"}


class TestContract(unittest.TestCase):
    """契约层：输出的形状必须稳定，否则下游没法写。"""

    def test_keys_fixed(self):
        for text in ("抓红块", "回初始位", "", "随便说点什么"):
            self.assertEqual(set(to_dict(parse_intent(text))), CONTRACT_KEYS)

    def test_action_always_in_enum(self):
        for text in ("抓红块", "往左移", "", None, 123):
            data = to_dict(parse_intent(text))
            self.assertIn(data["action"], [act.value for act in Action])

    def test_source_always_mock(self):
        """source 恒为 mock，防止以后误以为接了真模型。"""
        for text in ("抓红块", "", None):
            self.assertEqual(to_dict(parse_intent(text))["source"], "mock")

    def test_params_always_empty(self):
        """mock 不猜数值，params 必须为空。"""
        for text in ("往左移10厘米", "抓红块", ""):
            self.assertEqual(to_dict(parse_intent(text))["params"], {})

    def test_raw_kept_verbatim(self):
        text = "  把左边的红色杯子抓起来  "
        self.assertEqual(to_dict(parse_intent(text))["raw"], text)


class TestPositive(unittest.TestCase):
    """正例：文档里出现过的说法必须翻对。"""

    def test_pick_with_target_and_qualifier(self):
        data = to_dict(parse_intent("把左边的红色杯子抓起来"))
        self.assertEqual(data["action"], "pick")
        self.assertEqual(data["target"], "red_cup")
        self.assertEqual(data["qualifier"], "left")

    def test_place_from_planning_doc(self):
        """项目实施方案 L72 的原话「把红色零件放进盒子」。"""
        data = to_dict(parse_intent("把红色零件放进盒子"))
        self.assertEqual(data["action"], "place")
        self.assertEqual(data["target"], "red_block")

    def test_home(self):
        self.assertEqual(to_dict(parse_intent("回初始位"))["action"], "home")

    def test_move_left(self):
        self.assertEqual(to_dict(parse_intent("往左移一点"))["action"], "move_left")

    def test_look(self):
        self.assertEqual(to_dict(parse_intent("描述一下当前画面"))["action"], "look")

    def test_synonyms(self):
        """同义说法都要落到同一个动作（词表可扩充，行为不变）。"""
        for text in ("抓起来", "拿起", "捡起", "夹起"):
            self.assertEqual(to_dict(parse_intent(text))["action"], "pick", text)

    def test_qualifier_right_and_front(self):
        self.assertEqual(to_dict(parse_intent("抓右边的红块"))["qualifier"], "right")
        self.assertEqual(to_dict(parse_intent("抓前面的红块"))["qualifier"], "front")


class TestNegative(unittest.TestCase):
    """负例与兜底：宁可说「不认识」，也不许瞎猜。"""

    def test_unrelated_text_is_unknown(self):
        data = to_dict(parse_intent("今天天气不错"))
        self.assertEqual(data["action"], "unknown")
        self.assertTrue(data["warnings"])

    def test_empty_and_whitespace_do_not_crash(self):
        for bad in ("", "   ", "！！！"):
            self.assertEqual(to_dict(parse_intent(bad))["action"], "unknown")

    def test_non_string_input_do_not_crash(self):
        """网页 / 语音链路可能传来 None 或数字，不许把进程带崩。"""
        for bad in (None, 123, [], {}):
            self.assertEqual(to_dict(parse_intent(bad))["action"], "unknown")

    def test_nav_is_unknown_not_silently_mapped(self):
        """车体 / 导航是阶段 6，命中导航词一律 UNKNOWN。"""
        data = to_dict(parse_intent("导航到A区再抓红块"))
        self.assertEqual(data["action"], "unknown")
        self.assertTrue(any("导航" in w for w in data["warnings"]))

    def test_multi_action_is_unknown(self):
        """一句话同时含「抓」和「放」，mock 不猜用哪个。"""
        data = to_dict(parse_intent("抓红块然后放进盒子"))
        self.assertEqual(data["action"], "unknown")
        self.assertTrue(any("多个动作" in w for w in data["warnings"]))

    def test_injection_like_text_is_not_executed(self):
        """文本里混进 JSON 不许被当指令执行：只做子串查表，不 eval。"""
        text = '{"action":"home","source":"cloud"}'
        data = to_dict(parse_intent(text))
        self.assertEqual(data["source"], "mock")
        self.assertEqual(data["action"], "unknown")

    def test_pick_without_target_warns(self):
        data = to_dict(parse_intent("抓起来"))
        self.assertTrue(any("缺目标物体" in w for w in data["warnings"]))


class TestValidate(unittest.TestCase):
    """校验层单独测：返回列表、不抛异常。"""

    def test_validate_returns_list(self):
        self.assertIsInstance(validate(parse_intent("")), list)

    def test_non_mock_source_warns(self):
        cmd = IntentCommand(action=Action.HOME, raw="回初始位", source="cloud")
        self.assertTrue(any("非 mock" in w for w in validate(cmd)))

    def test_params_nonempty_warns(self):
        cmd = IntentCommand(action=Action.HOME, raw="回初始位", params={"dist": 10})
        self.assertTrue(any("params" in w for w in validate(cmd)))


class TestBackend(unittest.TestCase):
    """后端可替换性：今天插假的，明天插真的，别处不改。"""

    def test_default_backend_is_mock(self):
        self.assertIsInstance(MOCK_BACKEND, MockBackend)
        self.assertEqual(to_dict(parse_intent("抓红块"))["source"], "mock")

    def test_custom_backend_can_be_injected(self):
        class FixedBackend(ParserBackend):
            def parse(self, text):
                return IntentCommand(action=Action.HOME, raw=text, source="cloud")

        data = to_dict(parse_intent("随便说啥", backend=FixedBackend()))
        self.assertEqual(data["action"], "home")
        self.assertEqual(data["source"], "cloud")
        self.assertTrue(data["warnings"])

    def test_cloud_backend_is_placeholder(self):
        """CloudBackend 现在必须抛异常，绝不静默返回 / 偷偷联网。"""
        with self.assertRaises((RuntimeError, NotImplementedError)):
            CloudBackend().parse("抓红块")

    def test_abstract_backend_cannot_be_instantiated(self):
        with self.assertRaises(TypeError):
            ParserBackend()


if __name__ == "__main__":
    unittest.main(verbosity=2)
