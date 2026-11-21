#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""planner.py —— 多步任务规划器（大脑）：LLM 工具调用 + ReAct + 本地仲裁 + 多轮对话

把用户说的复杂指令（"帮我把你左前方的玩具熊猫拿过来"）拆成**多步工具调用序列**，
每步都先过本地仲裁、再执行；执行结果回喂给 LLM，让它决定下一步；需要用户拍板时
用 `ask` 暂停，等语音/文字回复后 `continue_turn` 续跑。最终用 `done` 收尾。

规范定义：这是 **ReAct（Reasoning + Acting）+ Function Calling（工具调用）** 范式，
外加一层**本地仲裁器（local arbiter）**——LLM 只"提议"工具调用，仲裁器才有"授权"
的权力（沿用 docs/可行性与落地路线.md 第六节红线：LLM 不得直连执行器）。

🔴 与上一版单步 VLM 的关系：
   - 旧 `speech_agent.py` 的慢通道是「一句指令 → pick_pipeline 一次抓/放」，单步；
   - 本文件把大脑升级成「多步规划 + 多轮对话」，`grasp`/`place` 等工具将来才接到
     pick_pipeline 上；当前这些身体动作的 executor 返回"未就绪（模拟）"，诚实不装。

🔴 诚实边界：
  1) mock 后端是**脚本化**的：不识别语义，只按固定顺序吐工具调用，用于验证
     「工具调用 → 仲裁 → 执行 → 回喂 → 续跑」这条 ReAct 管道；不代表"会规划"。
  2) search/navigate/grasp/place 目前是**模拟执行器**（身体未就绪），真实执行器
     通过 `real_executors` 字典注入，硬件到货后逐个接真。
  3) Key 只走环境变量 LLM_API_KEY，绝不进代码/Git。

用法（零第三方依赖）：
  python3 planner.py --self-test                     # 只验 ReAct 管道，不联网
  python3 planner.py --text "把左前方的玩具熊猫拿过来"  # mock 跑完整多步
  python3 planner.py --text "..." --backend openai --model Qwen/Qwen3-8B \
          --base-url https://api.siliconflow.cn/v1   # 真 LLM（需 LLM_API_KEY）
"""

import abc
import argparse
import dataclasses
import json
import os
import sys
import urllib.error
import urllib.request
import uuid


def _emit(text, newline=True):
    """安全打印：板上 LANG=C 时中文也不许崩进程。"""
    data = str(text)
    try:
        print(data, end="\n" if newline else "", flush=True)
    except UnicodeEncodeError:
        safe = data.encode("ascii", "backslashreplace").decode("ascii")
        print(safe, end="\n" if newline else "", flush=True)


# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------
MAX_STEPS = 8            # 一次任务最多 8 轮工具调用，防死循环
MAX_NAV_CM = 200.0       # 底盘单步移动上限（身体未就绪前只是仲裁闸门）
NAV_DIRECTIONS = ("forward", "backward", "left", "right",
                  "left_forward", "right_forward")

SYSTEM_PROMPT = (
    "你是机械臂小车助手。你能调用工具：home(回初始位)、estop(急停)、resume(解除急停)、"
    "observe(观察画面)、search(搜索目标)、navigate(底盘移动)、grasp(抓取)、place(放置)、"
    "ask(向用户提问)、done(结束任务)。规则：① 复杂指令先拆成多步，一步一步来；"
    "② 急停永远优先；③ 不确定目标位置/用户意图时用 ask 提问，不要瞎猜；"
    "④ 每次只调用你需要的那几个工具，看到工具结果再决定下一步；"
    "⑤ 全部做完用 done 收尾并简述做了什么。搜索/移动/抓取/放置的执行器可能返回"
    "“未就绪（模拟）”，你要据实继续或 ask。"
)


# ---------------------------------------------------------------------------
# 工具定义（OpenAI function-calling 格式）
# ---------------------------------------------------------------------------
def _fn(name, desc, props, required):
    return {"type": "function",
            "function": {"name": name, "description": desc,
                         "parameters": {"type": "object",
                                        "properties": props,
                                        "required": required}}}


TOOLS = [
    _fn("home", "让机械臂回到初始位（已实机，无参数）。", {}, []),
    _fn("estop", "急停：立刻冻结一切动作，最优先。", {}, []),
    _fn("resume", "解除急停，恢复可执行状态。", {}, []),
    _fn("observe", "观察当前画面，返回识别到的候选目标清单。", {}, []),
    _fn("search", "在指定方位搜索目标物体。",
        {"target": {"type": "string", "description": "目标物体名，如“玩具熊猫”"},
         "region": {"type": "string",
                    "description": "方位，如“左前方/正前方/右侧”，可空"}},
        ["target"]),
    _fn("navigate", "让底盘移动一小段距离。",
        {"direction": {"type": "string", "description": "方向，见系统约束"},
         "distance_cm": {"type": "number", "description": "移动距离，厘米"}},
        ["direction", "distance_cm"]),
    _fn("grasp", "抓取目标物体。",
        {"target": {"type": "string", "description": "要抓的物体名"}},
        ["target"]),
    _fn("place", "把当前夹持的物体放到指定位置。",
        {"location": {"type": "string", "description": "放置位置，如“用户手上/桌面左格”"}},
        ["location"]),
    _fn("ask", "向用户提问，等用户回答后再继续。",
        {"question": {"type": "string", "description": "要问用户的问题"}},
        ["question"]),
    _fn("done", "任务结束。",
        {"summary": {"type": "string", "description": "对已完成/未完成内容的简述"}},
        ["summary"]),
]


# ---------------------------------------------------------------------------
# 数据结构
# ---------------------------------------------------------------------------
@dataclasses.dataclass
class Verdict:
    ok: bool
    reason: str = ""


@dataclasses.dataclass
class ToolCall:
    name: str
    arguments: dict


@dataclasses.dataclass
class LlmTurn:
    """一次 LLM 回复：要么是工具调用，要么是最终文字。"""
    tool_calls: list = dataclasses.field(default_factory=list)   # [ToolCall]
    content: str = ""
    finish: str = "stop"


@dataclasses.dataclass
class PlannerResult:
    status: str = "done"      # done | ask | error
    message: str = ""
    steps: list = dataclasses.field(default_factory=list)   # 已执行步骤的记录


# ---------------------------------------------------------------------------
# 本地仲裁器：LLM 只提议，这里才授权
# ---------------------------------------------------------------------------
def arbitrate(name, args):
    """对一次工具调用做安全授权（纯函数）。"""
    args = args or {}
    if name == "estop":
        return Verdict(True, "急停，永远放行")
    if name in ("home", "resume", "observe", "ask", "done"):
        return Verdict(True)
    if name == "search":
        if not str(args.get("target", "")).strip():
            return Verdict(False, "search 缺少 target")
        return Verdict(True)
    if name == "navigate":
        d = args.get("direction", "")
        if str(d) not in NAV_DIRECTIONS:
            return Verdict(False, "navigate 方向 %r 不在白名单 %s" % (d, NAV_DIRECTIONS))
        try:
            dist = float(args.get("distance_cm", 0))
        except (TypeError, ValueError):
            return Verdict(False, "navigate 距离不是数字")
        if not (0 < dist <= MAX_NAV_CM):
            return Verdict(False, "navigate 距离 %.1f 超出 (0, %.0f]" % (dist, MAX_NAV_CM))
        return Verdict(True)
    if name == "grasp":
        if not str(args.get("target", "")).strip():
            return Verdict(False, "grasp 缺少 target")
        return Verdict(True)
    if name == "place":
        if not str(args.get("location", "")).strip():
            return Verdict(False, "place 缺少 location")
        return Verdict(True)
    return Verdict(False, "未知工具 %r" % name)


# ---------------------------------------------------------------------------
# 执行器：身体动作当前为模拟，真实执行器通过 real_executors 注入
# ---------------------------------------------------------------------------
def execute(name, args, real_executors=None):
    """执行一次已授权的工具调用，返回 (文本结果, 是否为 ask)。"""
    args = args or {}
    real = (real_executors or {}).get(name)

    if name == "ask":
        return "（等待用户回答）", True

    if real is not None:
        try:
            return real(args), False
        except RuntimeError as e:
            return "执行失败：%s" % e, False

    # ---- 模拟执行器（身体未就绪）----
    if name == "home":
        return "已回初始位（dry-run）", False
    if name == "estop":
        return "已急停，后续动作冻结（dry-run）", False
    if name == "resume":
        return "已解除急停（dry-run）", False
    if name == "observe":
        return "观察到候选：玩具熊猫 0.88、杯子 0.81、方块 0.75（模拟）", False
    if name == "search":
        t = args.get("target", "")
        return "未找到 %s，建议向左前方移动一小段后重试（模拟）" % t, False
    if name == "navigate":
        return "已向 %s 移动 %.1f cm（模拟）" % (args.get("direction"), args.get("distance_cm", 0)), False
    if name == "grasp":
        return "已抓取 %s（模拟；真实抓取待总线舵机到货）" % args.get("target", ""), False
    if name == "place":
        return "已放置到 %s（模拟）" % args.get("location", ""), False
    if name == "done":
        return args.get("summary", ""), False
    return "未实现 %s" % name, False


# ---------------------------------------------------------------------------
# LLM 后端
# ---------------------------------------------------------------------------
class LlmBackend(abc.ABC):
    name = "abstract"

    @abc.abstractmethod
    def chat(self, messages, tools):
        """输入对话 + 工具定义，返回一次 LlmTurn。"""
        raise NotImplementedError


class MockBackend(LlmBackend):
    """脚本化后端：不识别语义，只按固定顺序吐工具调用，用于验管道。"""
    name = "mock"

    def __init__(self, script=None):
        self.script = list(script or _default_script())
        self.calls = 0

    def chat(self, messages, tools):
        self.calls += 1
        if self.script:
            return self.script.pop(0)
        return LlmTurn(content="已完成（mock 无更多脚本）", finish="stop")


def _default_script():
    """固定多步脚本：search→navigate→grasp→home→ask→(续)place→done。"""
    return [
        LlmTurn(tool_calls=[ToolCall("search", {"target": "玩具熊猫", "region": "左前方"})]),
        LlmTurn(tool_calls=[ToolCall("navigate", {"direction": "left_forward", "distance_cm": 30})]),
        LlmTurn(tool_calls=[ToolCall("grasp", {"target": "玩具熊猫"})]),
        LlmTurn(tool_calls=[ToolCall("home", {})]),
        LlmTurn(tool_calls=[ToolCall("ask", {"question": "任务已完成，要把它放到哪里？"})]),
        LlmTurn(tool_calls=[ToolCall("place", {"location": "用户手上"})]),
        LlmTurn(tool_calls=[ToolCall("done", {"summary": "已搜索、抓取玩具熊猫并放到用户手上"})]),
    ]


class OpenAiBackend(LlmBackend):
    """OpenAI 兼容 chat/completions（SiliconFlow / 阿里云百炼 / 智谱等）。"""
    name = "openai"

    def __init__(self, base_url, model, api_key="", timeout=60.0):
        self.base_url = base_url.rstrip("/")
        self.model = model
        self.api_key = api_key
        self.timeout = timeout

    def chat(self, messages, tools):
        body = json.dumps({"model": self.model, "messages": messages,
                           "tools": tools}).encode("utf-8")
        headers = {"Content-Type": "application/json"}
        if self.api_key:
            headers["Authorization"] = "Bearer " + self.api_key
        req = urllib.request.Request(self.base_url + "/chat/completions",
                                     data=body, headers=headers, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                raw = resp.read().decode("utf-8", "replace")
        except urllib.error.HTTPError as e:
            detail = e.read().decode("utf-8", "replace")[:300] if e.fp else ""
            raise RuntimeError("LLM 服务返回 HTTP %s：%s" % (e.code, detail))
        except urllib.error.URLError as e:
            raise RuntimeError("连不上 LLM 服务 %s（%s）" % (self.base_url, e.reason))

        try:
            obj = json.loads(raw)
            msg = obj["choices"][0]["message"]
        except (ValueError, KeyError, IndexError):
            raise RuntimeError("LLM 响应格式不认识：%s" % raw[:300])

        turn = LlmTurn(content=msg.get("content") or "",
                       finish=obj["choices"][0].get("finish_reason", "stop"))
        for tc in msg.get("tool_calls") or []:
            fn = tc.get("function", {})
            try:
                args = json.loads(fn.get("arguments") or "{}")
            except ValueError:
                args = {}
            turn.tool_calls.append(ToolCall(fn.get("name", ""), args))
        return turn


# ---------------------------------------------------------------------------
# Planner：ReAct 主循环
# ---------------------------------------------------------------------------
class Planner:
    def __init__(self, backend, real_executors=None, apply=False, max_steps=MAX_STEPS):
        self.backend = backend
        self.real_executors = real_executors or {}
        self.apply = apply
        self.max_steps = max_steps
        self.messages = []

    # ---- 多轮对话入口 ----
    def run(self, user_text, resume=False):
        if not resume:
            self.messages = [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": user_text},
            ]
        else:
            self.messages.append({"role": "user", "content": user_text})
        return self._react()

    def continue_turn(self, user_reply):
        """上一轮停在 ask 后，用用户回复续跑。"""
        return self.run(user_reply, resume=True)

    # ---- ReAct 循环 ----
    def _react(self):
        steps = []
        for _ in range(self.max_steps):
            turn = self.backend.chat(self.messages, TOOLS)

            if not turn.tool_calls:
                return PlannerResult(status="done", message=turn.content, steps=steps)

            # 记录 assistant 的工具调用（OpenAI 约定）
            tool_calls_json = [
                {"id": "call_%s" % uuid.uuid4().hex[:8],
                 "type": "function",
                 "function": {"name": tc.name,
                              "arguments": json.dumps(tc.arguments, ensure_ascii=False)}}
                for tc in turn.tool_calls
            ]
            self.messages.append({"role": "assistant", "content": None,
                                  "tool_calls": tool_calls_json})

            for i, tc in enumerate(turn.tool_calls):
                v = arbitrate(tc.name, tc.arguments)
                if not v.ok:
                    result = "仲裁拒绝：%s" % v.reason
                    ask = False
                else:
                    result, ask = execute(tc.name, tc.arguments, self.real_executors)
                    if tc.name in ("home", "estop", "resume") and self.apply:
                        result = result.replace("（dry-run）", "（已真下发）")
                steps.append({"tool": tc.name, "args": tc.arguments,
                              "ok": v.ok, "result": result})
                self.messages.append({"role": "tool",
                                      "tool_call_id": tool_calls_json[i]["id"],
                                      "content": result})
                if ask:
                    # 暂停，等用户回复；记录到哪一步
                    q = tc.arguments.get("question", "")
                    return PlannerResult(status="ask", message=q, steps=steps)
                if tc.name == "done":
                    # done 是终止信号：直接用 summary 收尾，不再多跑一轮
                    return PlannerResult(status="done", message=result, steps=steps)
        return PlannerResult(status="error",
                             message="超过 %d 轮工具调用仍未结束" % self.max_steps,
                             steps=steps)


# ---------------------------------------------------------------------------
# 自测：不联网、用 mock 后端、真实执行器为空
# ---------------------------------------------------------------------------
def self_test():
    # 1) 仲裁：急停放行、未知工具拒绝、超量程移动拒绝
    assert arbitrate("estop", {}).ok
    assert not arbitrate("frobnicate", {}).ok
    assert not arbitrate("navigate", {"direction": "left_forward", "distance_cm": 999}).ok
    assert arbitrate("navigate", {"direction": "left_forward", "distance_cm": 30}).ok
    assert not arbitrate("grasp", {}).ok

    # 2) 完整 ReAct + 多轮：mock 脚本应 search→navigate→grasp→home→ask
    p = Planner(MockBackend())
    r = p.run("帮我把左前方的玩具熊猫拿过来")
    assert r.status == "ask", r
    names = [s["tool"] for s in r.steps]
    assert names == ["search", "navigate", "grasp", "home", "ask"], names
    assert "哪里" in r.message, r.message

    # 3) 续跑：用户回答"放在我手上" → place → done
    r2 = p.continue_turn("放在我手上")
    assert r2.status == "done", r2
    assert r2.steps[0]["tool"] == "place" and r2.steps[1]["tool"] == "done", r2.steps

    # 4) 超步数保护：给一个永远吐工具的脚本，应 status=error
    loop_script = [LlmTurn(tool_calls=[ToolCall("observe", {})])] * (MAX_STEPS + 3)
    r3 = Planner(MockBackend(loop_script)).run("看")
    assert r3.status == "error", r3

    _emit("SELF_TEST PASS（仲裁 + ReAct + 多轮续跑 + 超步保护 全验）")
    return 0


# ---------------------------------------------------------------------------
# 命令行外壳
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="多步任务规划器：LLM 工具调用 + 仲裁")
    ap.add_argument("--self-test", action="store_true", help="只验 ReAct 管道，不联网")
    ap.add_argument("--text", default=None, help="用户指令（默认跑一段固定演示）")
    ap.add_argument("--backend", choices=("mock", "openai"), default="mock")
    ap.add_argument("--base-url", default="https://api.siliconflow.cn/v1")
    ap.add_argument("--model", default="Qwen/Qwen3.8-27B")
    ap.add_argument("--apply", action="store_true", help="home/estop/resume 真下发（未接时仍 dry-run）")
    ap.add_argument("--timeout", type=float, default=60.0)
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    text = args.text or "帮我把左前方的玩具熊猫拿过来"

    if args.backend == "mock":
        be = MockBackend()
    else:
        be = OpenAiBackend(args.base_url, args.model,
                           os.environ.get("LLM_API_KEY", ""), timeout=args.timeout)

    p = Planner(be, apply=args.apply)
    _emit("=" * 66)
    _emit("用户指令：%s" % text)
    _emit("大脑后端：%s" % be.name)
    r = p.run(text)
    _emit("-" * 66)
    for s in r.steps:
        _emit("  [%s] %s -> %s" % (s["tool"], s["args"], s["result"]))
    _emit("状态：%s" % r.status)
    if r.status == "ask":
        _emit("提问：%s" % r.message)
        _emit("（此时可回一句继续；演示到此为止）")
    else:
        _emit("结论：%s" % r.message)
    return 0


if __name__ == "__main__":
    sys.exit(main())
