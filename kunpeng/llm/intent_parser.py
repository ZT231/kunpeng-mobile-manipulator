#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
intent_parser.py —— 阶段 2：自然语言 → 结构化动作指令（先 mock，不接真大模型）

大白话：这个模块是个「翻译员」。人说的「把左边的红杯子抓起来」，
       它翻成机器认得的固定格式 {"action":"pick","target":"red_cup",...}。
       现在用的是「按关键词查表」的假翻译员（mock），不联网、不用 Key；
       等以后接云端大模型，只换一个后端，别处一行不改。
规范定义：意图解析（intent parsing / slot filling）把非结构化文本映射为
         受限的结构化动作表示；本项目输出契约为字段固定的 JSON 对象，
         与 docs/项目完整流程模拟（A档终态）.md L24 的
         {"action":"pick","target":"red_cup"} 及 kunpeng/docs/A档执行计划.md
         L55 的 {"action":"move_left","dist":10} 对齐。

🔴 诚实边界（README 里也写一份，别越线）：
  1) 本模块只「输出」结构化指令，**不下发**。当前真正能执行的是
     「人点画面 → IK → JOINT」那条开环链路，自动抓取 / 下降-闭合-抬起
     序列尚未实现 ⇒ 不能写「自然语言控制机械臂已实现」。
  2) `source` 恒为 "mock"，防止以后看日志误以为接了真模型。
  3) 枚举里没有 NAV：车体 / 导航是阶段 6 的事，命中导航词一律 UNKNOWN。
  4) `params` 恒为空：mock 不猜数值（猜了就是假数据），dist 等参数
     留到接真 LLM 时再填。

为什么是独立模块，而不是去改 web_control.py：
  和阶段 1 的 telemetry_bridge.py 同一个道理 —— 5000 网页服务是已实机
  验证过的主链路，改它就要 scp + 重启 Flask，重启会清掉 _IK_MAP 标定值、
  打断现场。这里只做纯文本处理，主链路一行不碰，删掉目录就是完整回滚。

用法（都不联网、零第三方依赖）：
  python3 intent_parser.py --self-test      # 纯逻辑自测，零依赖
  python3 intent_parser.py                  # 跑几条内置演示句并打印结果
  python3 intent_parser.py --pretty         # 演示句的 JSON 缩进输出
"""

import abc
import argparse
import dataclasses
import enum
import json
import os
import sys
from typing import Optional


# ---------------------------------------------------------------------------
# 数据结构先定义：把「能输出什么」关进固定集合，拼错立刻报错
# ---------------------------------------------------------------------------
class Action(enum.Enum):
    """动作枚举。

    为什么不用裸字符串：写错 "pikc" 字符串不会报错，枚举拼错立刻
    AttributeError；而且所有合法动作一眼看全，单测能遍历校验。
    🔴 故意不放 NAV：车体 / 导航是阶段 6，没实现就不产出该动作。
    """
    PICK = "pick"
    PLACE = "place"
    HOME = "home"
    MOVE_LEFT = "move_left"
    MOVE_RIGHT = "move_right"
    LOOK = "look"
    UNKNOWN = "unknown"


@dataclasses.dataclass
class IntentCommand:
    """一条结构化动作指令（就是个带名字的数据袋子）。

    为什么用 dataclass 而不是裸 dict：字段有名字和默认值，写错字段名
    立刻报错；自带 __repr__ / __eq__，单测断言和输出都好读。
    """
    action: Action = Action.UNKNOWN
    target: Optional[str] = None
    qualifier: Optional[str] = None
    # 🔴 field(default_factory=dict) 是新手常踩的坑：直接写 params: dict = {}
    # 会让所有实例共享同一个空字典，改一个全变。default_factory 每次新建一个。
    params: dict = dataclasses.field(default_factory=dict)
    raw: str = ""
    source: str = "mock"
    warnings: tuple = ()


# ---------------------------------------------------------------------------
# 词表常量集中放这一块：改词表只改这里，别散到函数里
# 全部对齐项目已有规划文档里出现过的说法，不另起炉灶
# ---------------------------------------------------------------------------
ACTION_KEYWORDS = (
    # 「抓」「拿」单字必须收进来：中文里最常说的就是「抓红块」「拿杯子」，
    # 只收双字词会漏判（本地单测 test_multi_action_is_unknown 抓到过这个缺陷）。
    (Action.PICK, ("抓取", "抓起来", "抓起", "抓住", "拿起", "夹起", "捡起", "抓", "拿")),
    (Action.PLACE, ("放置", "放进", "放下", "放到", "摆在", "放")),
    (Action.HOME, ("回初始位", "回到初始", "归位", "复位", "收起")),
    (Action.MOVE_LEFT, ("左移", "往左", "向左", "左转")),
    (Action.MOVE_RIGHT, ("右移", "往右", "向右", "右转")),
    (Action.LOOK, ("描述", "看看", "看一下", "识别一下", "有什么", "场景理解")),
)

# 命中导航词直接落 UNKNOWN，别假装能执行（车体还没做）
NAV_KEYWORDS = ("导航", "去A区", "走到", "开到", "行驶到")

TARGET_KEYWORDS = (
    ("red_cup", ("红色杯子", "红杯子", "红杯")),
    ("red_block", ("红色零件", "红色方块", "红色积木", "红方块", "红块")),
    ("blue_block", ("蓝色零件", "蓝色方块", "蓝色积木", "蓝方块", "蓝块")),
    ("box", ("盒子", "箱子", "收纳盒")),
)

QUALIFIER_KEYWORDS = (
    ("left", ("左边", "左侧", "左前方", "左面", "左手的")),
    ("right", ("右边", "右侧", "右前方", "右面", "右手的")),
    ("front", ("前面", "前方", "正前方")),
)

DEMO_TEXTS = (
    "把左边的红色杯子抓起来",
    "把红色零件放进盒子",
    "回初始位",
    "往左移一点",
    "描述一下当前画面",
    "导航到A区再抓红块",
    "今天天气不错",
)


# ---------------------------------------------------------------------------
# 解析器后端：先定「插座标准」，现在插假的，以后插真的
# ---------------------------------------------------------------------------
class ParserBackend(abc.ABC):
    """解析器接口，相当于 C++ 里的纯虚基类 / Java 的 interface。

    大白话：先说好「不管谁来做翻译，都得有一个 parse(文本) 方法」，
           这样 parse_intent() 只认这个标准，不关心背后是查表还是调 API。
    规范定义：面向接口编程 / 策略模式；abc.ABC + @abstractmethod
            在实例化时强制子类实现，没实现就 TypeError。
    """

    @abc.abstractmethod
    def parse(self, text):
        """把一句话翻成 IntentCommand。子类必须实现。"""
        raise NotImplementedError


class MockBackend(ParserBackend):
    """当前唯一可用的后端：关键词查表，零依赖、零联网、结果确定。"""

    def parse(self, text):
        return _mock_parse(text)


class CloudBackend(ParserBackend):
    """占位：将来接云端 LLM 时在这里写 HTTP 调用。

    🔴 现在直接抛异常，绝不偷偷联网、也绝不拿 Key 发请求。
    Key 只走环境变量（kunpeng/docs/A档执行计划.md L54），绝不写进代码；
    将来真正实现时再在这里 import urllib.request / requests，
    顶层不 import 第三方库，保证 --self-test 和 mock 分支零依赖可跑。
    """

    def parse(self, text):
        key = os.environ.get("LLM_API_KEY", "")
        if not key:
            raise RuntimeError("未设置 LLM_API_KEY（Key 走环境变量，绝不写进代码）")
        raise NotImplementedError("云端 LLM 解析尚未实现（阶段 2 先跑 mock 与本地单测）")


# 模块级单例：MockBackend 无状态，共享一个实例即可，省得每次 new
MOCK_BACKEND = MockBackend()


# ---------------------------------------------------------------------------
# 纯函数层：不联网、不读环境变量、不写文件，同样的输入永远同样的输出
# ---------------------------------------------------------------------------
def _hit_any(text, words):
    """text 里含 words 中任意一个词就返回 True（纯函数）。

    为什么用子串匹配而不上分词 / 正则：这是 mock，目标只有一个 ——
    「确定、可测」。真接 LLM 后这一整层会被替换，现在搞复杂就是过度设计。
    """
    return any(word in text for word in words)


def _mock_parse(text):
    """按关键词查表解析一句话（纯函数）。"""
    # 非字符串输入（None / 数字 / 列表）不许崩，统一当空串处理
    raw = text if isinstance(text, str) else ""
    sentence = raw.strip()
    warns = []

    # 1) 导航优先拦截：车体还没做，命中就别假装能执行
    if _hit_any(sentence, NAV_KEYWORDS):
        warns.append("命中导航/车体移动词，该功能未实现（阶段 6），mock 不产出动作")
        return IntentCommand(action=Action.UNKNOWN, raw=raw, warnings=tuple(warns))

    # 2) 动作：收集所有命中的 Action。0 个或多于 1 个都落 UNKNOWN ——
    #    mock 不瞎猜，猜错了下游会当真指令处理，比「不认识」更危险。
    actions = [act for act, words in ACTION_KEYWORDS if _hit_any(sentence, words)]
    if len(actions) == 0:
        warns.append("未识别到可执行动作")
        action = Action.UNKNOWN
    elif len(actions) > 1:
        warns.append("一句话同时命中多个动作（%s），mock 不猜，落 UNKNOWN" %
                     "/".join(act.value for act in actions))
        action = Action.UNKNOWN
    else:
        action = actions[0]

    # 3) 目标与方位：多个目标只取第一个，并如实记一条 warning
    targets = [name for name, words in TARGET_KEYWORDS if _hit_any(sentence, words)]
    target = targets[0] if targets else None
    if len(targets) > 1:
        warns.append("句中含多个目标（%s），mock 只取第一个 %s" %
                     ("/".join(targets), target))

    qualifiers = [name for name, words in QUALIFIER_KEYWORDS if _hit_any(sentence, words)]
    qualifier = qualifiers[0] if qualifiers else None

    # params 刻意留空：mock 不猜数值，猜了就是假数据
    return IntentCommand(action=action, target=target, qualifier=qualifier,
                         raw=raw, warnings=tuple(warns))


def validate(cmd):
    """校验一条指令，返回 warning 列表（不抛异常）。

    为什么不抛异常：将来调用方可能是网页 / 语音链路，一条脏输入不该把
    整个进程带崩。返回列表让上层自己决定要不要拦，容错不失控。
    """
    out = []
    if not isinstance(cmd.raw, str) or not cmd.raw.strip():
        out.append("输入为空或不是文本")
    if cmd.action in (Action.PICK, Action.PLACE) and cmd.target is None:
        out.append("PICK/PLACE 缺目标物体，无法确定抓什么")
    if cmd.source != "mock":
        out.append("非 mock 来源，阶段 2 未验证该链路")
    if cmd.params:
        out.append("mock 阶段 params 应为空（不猜数值）")
    return out


def parse_intent(text, backend=MOCK_BACKEND):
    """对外主入口：拿一个 backend 解析，再补上结构性校验。

    🔴 默认值特意写死成 MockBackend：不传参 = 纯本地、零联网。
    免得哪天把 CloudBackend 设成默认，脚本一跑就偷偷往外发请求。
    """
    cmd = backend.parse(text)
    extra = validate(cmd)
    if extra:
        cmd.warnings = tuple(cmd.warnings) + tuple(extra)
    return cmd


def to_dict(cmd):
    """序列化成固定键集合的 JSON-ready 字典（纯函数）。

    键集合固定很重要：下游（将来的 MQTT / 决策链路）可以按键取值，
    不用每次猜「这次有没有这个字段」。
    """
    return {
        "action": cmd.action.value if isinstance(cmd.action, Action) else "unknown",
        "target": cmd.target,
        "qualifier": cmd.qualifier,
        "params": dict(cmd.params),
        "raw": cmd.raw,
        "source": cmd.source,
        "warnings": list(cmd.warnings),
    }


# ---------------------------------------------------------------------------
# 命令行入口
# ---------------------------------------------------------------------------
def self_test():
    """纯逻辑自测：零依赖、不联网，喂固定句子检查契约。"""
    cases = (
        ("把左边的红色杯子抓起来", "pick", "red_cup", "left"),
        ("把红色零件放进盒子", "place", "red_block", None),
        ("回初始位", "home", None, None),
        ("往左移一点", "move_left", None, None),
        ("描述一下当前画面", "look", None, None),
        ("今天天气不错", "unknown", None, None),
        ("导航到A区再抓红块", "unknown", None, None),
    )
    for text, act, tgt, qual in cases:
        data = to_dict(parse_intent(text))
        assert data["action"] == act, (text, data["action"], act)
        assert data["target"] == tgt, (text, data["target"], tgt)
        assert data["qualifier"] == qual, (text, data["qualifier"], qual)
        assert data["source"] == "mock", "source 必须恒为 mock"
        assert data["params"] == {}, "mock 阶段 params 必须为空"
        assert set(data) == {"action", "target", "qualifier", "params",
                             "raw", "source", "warnings"}, "键集合必须固定"

    # 容错用例：空串 / 纯空格 / None / 数字 / 纯标点都不许崩
    for bad in ("", "   ", None, 123, "！！！！"):
        data = to_dict(parse_intent(bad))
        assert data["action"] in [act.value for act in Action], "动作必须在枚举内"

    # CloudBackend 占位：不许静默返回，也不许偷偷联网
    try:
        CloudBackend().parse("抓红块")
    except (RuntimeError, NotImplementedError):
        pass
    else:
        raise AssertionError("CloudBackend 必须抛异常，不能静默返回")

    print("SELF_TEST PASS")
    print(json.dumps(to_dict(parse_intent(DEMO_TEXTS[0])), ensure_ascii=False, indent=2))
    return 0


def main():
    parser = argparse.ArgumentParser(description="自然语言 → 结构化动作指令（mock）")
    parser.add_argument("--self-test", action="store_true", help="纯逻辑自测，不联网")
    parser.add_argument("--text", default=None, help="解析单句（默认跑内置演示句）")
    parser.add_argument("--pretty", action="store_true", help="JSON 缩进输出")
    args = parser.parse_args()

    if args.self_test:
        return self_test()

    texts = [args.text] if args.text is not None else list(DEMO_TEXTS)
    for text in texts:
        data = to_dict(parse_intent(text))
        print(json.dumps(data, ensure_ascii=False,
                         indent=2 if args.pretty else None))
    return 0


if __name__ == "__main__":
    sys.exit(main())
