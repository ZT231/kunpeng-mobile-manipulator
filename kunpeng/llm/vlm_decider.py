#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vlm_decider.py —— 阶段 6 / P2：VLM 语义决策层（只选编号，不输出坐标）

大白话：这个模块是机械臂的「选择器」。给它一张照片 + 照片里已经被框出来的
       物体清单 + 人话指令，它回答「该抓第几个」以及「为什么」。
       它**绝不负责算坐标** —— 坐标由检测框 + 手眼矩阵 + IK 那条已实测的
       几何链路给出。模型只管「谁符合用户说的」，几何只管「它在哪」。

规范定义：这是分层式具身架构（SayCan / Code-as-Policies 一派）里的**语义
         策略层**。策略 π(a | o, l) 被拆成两段：
           model: (observation, language) → symbol   （本模块，VLM）
           geometry: symbol → joint command           （已有链路）
         输出复用 intent_parser.to_dict() 的固定 7 键契约，`params` 里带
         候选编号 / bbox / 理由，下游可无歧义消费。

🔴 为什么不让 VLM 直接输出像素坐标（这是本文件最核心的设计决定）：
   VLM 的视觉 grounding 精度是 ±30~50 px 量级。本项目 20 cm 处 1 px ≈ 0.25 mm
   ⇒ 误差 ±1 cm 以上，抓小物体必偏。而本地 YOLO 的 bbox 是像素级精确的。
   ⇒ **让专业的干专业的事**：模型选编号（纯符号判断，3B 小模型也可靠），
     bbox 从检测结果里取（准到像素）。这条分工是架构能不能用的分水岭。

🔴 为什么板上不跑 VLA（OpenVLA-7B / π0 这类端到端模型）：
   鲲鹏 Pro 无 CUDA，纯 ARM CPU 跑 7B 单帧要几十秒，而抓取需要秒级闭环。
   本文件因此把模型放到**有 GPU 的 PC** 上（RTX 4060 Laptop 8 GB 足够跑
   Qwen2.5-VL-3B），板子只提供 /observe（画面 + 同帧 bbox），走 HTTP。
   ⇒ 分层不只是算法选择，也是这台硬件的唯一可行解。

🔴 诚实边界（README 同步写一份，别越线）：
   1) 本模块只产出**决策**，不执行动作。执行层是 action_primitives.py（P1），
      两者由调用方串起来；当前**还没有把二者接成一条自动链路**。
   2) 模型选完编号后仍走原有的「检测框 → 手眼 → IK → 脉宽」链路，
      所以**决策不是由模型产生坐标**，不能宣传成「端到端具身智能」。
   3) `source` 字段恒标明后端与模型名（如 "vlm:qwen2.5vl:3b"），
      看日志就能分辨是 mock 还是真模型，不会自己骗自己。

用法（全部零第三方依赖，只用标准库 urllib）：
  python3 vlm_decider.py --self-test
        纯逻辑自测：起一个假 HTTP 服务，覆盖 6 种模型回复（正常 / 越界编号 /
        markdown 围栏 / 乱码 / 空候选 / HTTP 500）。不联网、不需要模型。

  python3 vlm_decider.py --instruction "把杯子抓起来" --dry-run
        只打印将要发给模型的 prompt，不调用模型。调 prompt 时用这个。

  python3 vlm_decider.py --instruction "把杯子抓起来" \\
          --detector http://192.168.77.100:8080 \\
          --backend http --model qwen2.5vl:3b
        完整跑一次：拉 /observe → 调模型 → 打印决策 JSON。
        （默认 base_url 是 http://127.0.0.1:11434/v1，即 ollama 的
          OpenAI 兼容口；换云端只改 --base-url 和 --api-key 的环境变量。）
"""

import abc
import argparse
import base64
import dataclasses
import json
import os
import sys
import urllib.error
import urllib.request


# ---------------------------------------------------------------------------
# 固定契约：能产出什么，先关进集合里，拼错立刻炸
# ---------------------------------------------------------------------------
# 与 intent_parser.Action 完全对齐（不另起炉灶，否则两套枚举必然漂移）
VALID_ACTIONS = ("pick", "place", "home", "look", "unknown")

# 放置方位。只收这三个，模型给别的一律当 null；
# 因为它们要和机械臂的实际可达工作空间对得上（左/右/正前）。
VALID_SIDES = ("left", "right", "front")


@dataclasses.dataclass
class Candidate:
    """一个候选物体（就是检测结果，加了个人类友好的编号）。

    为什么要有 idx：模型只会说「第 3 个」，不会说「bbox 是 [301,180,420,290]」。
    编号是我们给它的**唯一句柄**，选完用编号换回精确 bbox。
    """
    idx: int                    # 1 起，给模型看
    cls: str                    # COCO 类名，如 "cup"
    conf: float
    bbox: tuple                 # (x1, y1, x2, y2) 像素
    center: tuple               # (cx, cy) 像素
    direction: str = ""         # 检测器给的方位提示，如 "左"


@dataclasses.dataclass
class Decision:
    """一次决策的完整结果（含拒绝理由，方便复盘）。

    为什么不直接返回 dict：字段有名字、有默认值，写错立刻 AttributeError；
    而且 warnings 让「为什么没执行」可追溯 —— 具身系统里
    「拒绝了什么、为什么拒绝」比「执行了什么」更能说明系统是否可靠。
    """
    action: str = "unknown"
    target_idx: int = None
    target_class: str = None
    target_bbox: tuple = None
    target_center: tuple = None
    place_side: str = None
    reason: str = ""
    source: str = "mock"
    instruction: str = ""
    raw_reply: str = ""
    warnings: tuple = ()


# ---------------------------------------------------------------------------
# Prompt：模型看到的全部信息都在这里，改 prompt 只改这一块
# ---------------------------------------------------------------------------
SYSTEM_PROMPT = """你是一个桌面机械臂的视觉决策模块，负责根据用户的自然语言指令，从候选物体中选出要操作的那一个。

必须遵守的规则：
1. 只能从「候选物体」列表里选择，choice 必须是一个已存在的编号。
2. 绝对不要输出像素坐标，坐标由下游几何模块负责。
3. 如果候选里没有符合用户描述的物体，action 填 "unknown"、choice 填 null。
4. 如果用户要求的是移动/归位/观察（不涉及抓取具体物体），按其语义填 action，choice 填 null。
5. 只输出一个 JSON 对象。不要 markdown 代码块，不要任何解释性文字。"""


def build_user_prompt(instruction, cands, frame_w, frame_h):
    """把「画面尺寸 + 候选清单 + 用户指令 + 输出格式」拼成一段文本（纯函数）。

    🔴 为什么要报画面尺寸和方位：模型看得到图，但它对「哪个是左边」的判断
       在 640×480 的小图上并不可靠；显式给它每个候选的像素中心和方位，
       等于把定位这件事从「视觉感知」降级成「读表格」，准确率明显更高。
    """
    lines = []
    lines.append("画面尺寸：%d × %d 像素（原点在左上角，x 向右，y 向下）"
                 % (frame_w, frame_h))
    lines.append("")
    lines.append("候选物体（由本地目标检测器给出，编号就是你要填的 choice）：")
    for c in cands:
        lines.append("  [%d] %-14s 置信度=%.2f 中心=(%d, %d) 方位=%s"
                     % (c.idx, c.cls, c.conf, c.center[0], c.center[1],
                        c.direction or "未知"))
    lines.append("")
    lines.append("用户指令：%s" % instruction)
    lines.append("")
    lines.append("请输出如下格式的 JSON（不要代码块围栏）：")
    lines.append('{"action": "pick|place|home|look|unknown", '
                 '"choice": 编号或null, '
                 '"reason": "一句中文理由", '
                 '"place_side": "left|right|front|null"}')
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# 回复解析：模型不会永远听话，所以要有三道防线
# ---------------------------------------------------------------------------
def extract_json(text):
    """从模型回复里挖出一个 JSON 对象；挖不到返回 None（纯函数）。

    三道防线（按模型「不听话」的常见程度排序）：
      ① 原文就是合法 JSON —— prompt 里要求了，多数模型能做到；
      ② 被 ```json ... ``` 围栏包着 —— 最常见的「手滑」，小模型尤其爱干；
      ③ 前后夹了一句人话 —— "好的，我选择：{...} 因为它是红的"。
    🔴 为什么不直接 text.strip() 后 json.loads：只要模型多吐一个句号，
       整条决策就失败。而失败必须**显式记 warning 落 UNKNOWN**，
       绝不能「解析失败就当没听见、继续用上一个结果」——那会抓错东西。
    """
    if not isinstance(text, str) or not text.strip():
        return None
    t = text.strip()

    # 防线 ①：原文即 JSON
    try:
        obj = json.loads(t)
        if isinstance(obj, dict):
            return obj
    except Exception:
        pass

    # 防线 ②：去 ``` 围栏（可能是 ```json，也可能只是 ```）
    if "```" in t:
        parts = t.split("```")
        if len(parts) >= 2:
            body = parts[1]
            if body.lower().startswith("json"):
                body = body[4:]
            try:
                obj = json.loads(body.strip())
                if isinstance(obj, dict):
                    return obj
            except Exception:
                pass

    # 防线 ③：第一个 { 到最后一个 }
    i, j = t.find("{"), t.rfind("}")
    if 0 <= i < j:
        try:
            obj = json.loads(t[i:j + 1])
            if isinstance(obj, dict):
                return obj
        except Exception:
            return None
    return None


def normalize(obj, cands, reply_text, source, instruction):
    """把模型给的 JSON 校验成 Decision；任何不合规都降级成 unknown + warning（纯函数）。

    🔴 这是整个模块的**安全阀**：模型可以胡说，但我们的代码不允许执行胡说。
       尤其是 choice 越界 —— 如果直接 cands[choice-1]，IndexError 会让进程崩；
       若碰上 Python 的负下标（choice=-1 取到最后一个），更会**静默抓错物体**。
       所以这里一律用「编号→候选」显式查表，查不到就拒绝。
    """
    warns = []

    if obj is None:
        return Decision(action="unknown", source=source, instruction=instruction,
                        raw_reply=reply_text,
                        warnings=("模型回复里挖不到 JSON 对象（内容见 raw_reply）",))

    # ---- 1. action 枚举校验 ----
    act_raw = obj.get("action")
    act = str(act_raw).strip().lower() if act_raw is not None else ""
    if act not in VALID_ACTIONS:
        warns.append("action=%r 不在枚举 %s 内，落 UNKNOWN"
                     % (act_raw, "/".join(VALID_ACTIONS)))
        act = "unknown"

    # ---- 2. choice 校验（最关键的一步）----
    chosen = None
    ch = obj.get("choice")
    if ch is None or (isinstance(ch, str) and ch.strip().lower() in ("", "null", "none")):
        if act in ("pick", "place"):
            warns.append("action=%s 但 choice 为 null，没有可执行的目标" % act)
            act = "unknown"
    else:
        # 只接受整数或纯数字字符串。模型有时给 "3号"、"#3"，一律不接受 ——
        # 宁可拒绝一次，也不要猜它想说什么然后抓错东西。
        cid = None
        if isinstance(ch, bool):
            cid = None                      # True/False 是 int 子类，必须排除
        elif isinstance(ch, int):
            cid = ch
        elif isinstance(ch, str) and ch.strip().isdigit():
            cid = int(ch.strip())
        if cid is None:
            warns.append("choice=%r 不是合法编号，拒绝执行" % (ch,))
            act = "unknown"
        else:
            chosen = next((c for c in cands if c.idx == cid), None)
            if chosen is None:
                warns.append("模型给了不存在的候选编号 %d（有效范围 1~%d），拒绝执行"
                             % (cid, len(cands)))
                act = "unknown"

    # ---- 3. 方位校验 ----
    side_raw = obj.get("place_side")
    side = str(side_raw).strip().lower() if side_raw is not None else ""
    if side in ("", "null", "none"):
        side = None
    elif side not in VALID_SIDES:
        warns.append("place_side=%r 不在 %s 内，已置空" % (side_raw, "/".join(VALID_SIDES)))
        side = None

    # ---- 4. reason 只做长度截断，不做内容判断 ----
    reason = obj.get("reason")
    reason = str(reason).strip()[:200] if reason is not None else ""

    return Decision(
        action=act,
        target_idx=chosen.idx if chosen else None,
        target_class=chosen.cls if chosen else None,
        target_bbox=chosen.bbox if chosen else None,
        target_center=chosen.center if chosen else None,
        place_side=side,
        reason=reason,
        source=source,
        instruction=instruction,
        raw_reply=reply_text,
        warnings=tuple(warns),
    )


# ---------------------------------------------------------------------------
# 后端：一个抽象插座，现在插 mock 与 HTTP 两种
# ---------------------------------------------------------------------------
class Backend(abc.ABC):
    """模型后端接口。

    大白话：不管背后是 ollama、vLLM 还是云端 API，都只需要实现
           「给我 system、user、一张图，还我一段文本」。
    规范定义：策略模式 / 依赖倒置；abc.ABC + @abstractmethod 保证
             子类必须实现，没实现实例化就 TypeError。
    """

    name = "abstract"

    @abc.abstractmethod
    def complete(self, system, user, jpeg_bytes):
        """返回模型原始回复文本（字符串）。"""
        raise NotImplementedError


class MockBackend(Backend):
    """不联网的假模型：按类名关键词匹配，找不到就选置信度最高的。

    🔴 它不是「智能」的替身，而是**链路的替身**：用来验证
       「候选打包 → prompt → 解析 → 校验 → 契约」这条管道本身没坏。
       它的决策质量没有意义，README 里也不许拿它的表现说事。
    """

    name = "mock"

    # 只覆盖几个常见 COCO 类的中文说法，够自测用
    ZH_HINTS = {
        "cup": ("杯", "杯子"),
        "bottle": ("瓶", "瓶子"),
        "book": ("书", "本子"),
        "cell phone": ("手机", "电话"),
        "apple": ("苹果",),
        "banana": ("香蕉",),
        "scissors": ("剪刀",),
        "mouse": ("鼠标",),
        "keyboard": ("键盘",),
        "bowl": ("碗",),
        "fork": ("叉",),
        "knife": ("刀",),
        "spoon": ("勺",),
    }

    def __init__(self):
        self.last_prompt = None     # 自测里用来检查 prompt 是否真的拼出来了
        self.calls = 0

    def complete(self, system, user, jpeg_bytes):
        self.calls += 1
        self.last_prompt = (system, user)

        # 从 user prompt 里把候选读回来（格式由 build_user_prompt 固定）
        cands = []
        for line in user.splitlines():
            line = line.strip()
            if not line.startswith("[") or "]" not in line:
                continue
            try:
                idx = int(line[1:line.index("]")])
                cls = line[line.index("]") + 1:].split()[0]
            except (ValueError, IndexError):
                continue
            cands.append((idx, cls))

        # 指令里提到了某个类的中文说法 → 选它；否则选第一个
        instruction = user.split("用户指令：", 1)[-1].splitlines()[0] if "用户指令：" in user else ""
        picked = None
        for idx, cls in cands:
            for hint in self.ZH_HINTS.get(cls, ()):
                if hint in instruction:
                    picked = idx
                    break
            if picked is not None:
                break
        if picked is None and cands:
            picked = cands[0][0]

        if picked is None:
            return json.dumps({"action": "unknown", "choice": None,
                               "reason": "候选为空", "place_side": None},
                              ensure_ascii=False)
        return json.dumps({"action": "pick", "choice": picked,
                           "reason": "（mock 后端：按类名关键词匹配）",
                           "place_side": None}, ensure_ascii=False)


class HttpBackend(Backend):
    """OpenAI 兼容的视觉模型后端（ollama / vLLM / 阿里云百炼 / 智谱 都能用）。

    为什么统一走 OpenAI 兼容格式：这三类服务的请求体一模一样，
    换服务商只改 --base-url 和 --model，代码零改动（这就是「插座标准」的价值）。

    🔴 Key 只走环境变量（默认 VLM_API_KEY），绝不写进代码、绝不进 Git。
       本地 ollama 不需要 Key，此时留空即可。
    """

    def __init__(self, base_url, model, api_key="", timeout=60):
        self.base_url = base_url.rstrip("/")
        self.model = model
        self.api_key = api_key
        self.timeout = timeout
        self.name = "vlm:%s" % model

    def complete(self, system, user, jpeg_bytes):
        url = self.base_url + "/chat/completions"

        # 图片按 data URL 内联。为什么不先上传再传链接：
        # 本地 ollama / vLLM 没有上传接口；data URL 是 OpenAI 兼容的事实标准。
        b64 = base64.b64encode(jpeg_bytes).decode("ascii")
        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": system},
                {"role": "user", "content": [
                    {"type": "text", "text": user},
                    {"type": "image_url",
                     "image_url": {"url": "data:image/jpeg;base64," + b64}},
                ]},
            ],
            # temperature=0：决策要尽量确定。同一张图同一句话，
            # 每次给不同答案会让排障变成猜谜。
            "temperature": 0.0,
            "stream": False,
        }
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        headers = {"Content-Type": "application/json"}
        if self.api_key:
            headers["Authorization"] = "Bearer " + self.api_key

        req = urllib.request.Request(url, data=body, headers=headers, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                raw = resp.read().decode("utf-8", "replace")
        except urllib.error.HTTPError as e:
            # HTTPError 的 body 里通常有服务端的真实原因（比如模型名拼错），
            # 一定要带出来，否则只能看到 "500" 干瞪眼。
            detail = e.read().decode("utf-8", "replace")[:300] if e.fp else ""
            raise RuntimeError("模型服务返回 HTTP %s：%s" % (e.code, detail))
        except urllib.error.URLError as e:
            raise RuntimeError("连不上模型服务 %s（%s）。"
                               "本地跑请确认 ollama 已启动：ollama serve"
                               % (self.base_url, e.reason))

        try:
            obj = json.loads(raw)
            return obj["choices"][0]["message"]["content"]
        except (ValueError, KeyError, IndexError, TypeError) as e:
            raise RuntimeError("模型服务返回格式不认识（%s）：%s" % (e, raw[:300]))


# ---------------------------------------------------------------------------
# 观测：从检测服务拿「画面 + 同帧检测结果」
# ---------------------------------------------------------------------------
def fetch_observation(detector_url, timeout=8):
    """GET /observe，返回 (frame_id, candidates, frame_w, frame_h, jpeg_bytes)。

    🔴 为什么只调 /observe 一个接口，而不是 /detections + /video 两次：
       /observe 在服务端是**同一把锁**里取的画面与 bbox，天然同帧。
       分两次取会出现「模型看第 N 帧的图、我们拿第 N+1 帧的 bbox」，
       于是按编号选对了物体、坐标却偏了一点 —— 这种 bug 不报错，只会偶尔抓偏。
    """
    url = detector_url.rstrip("/") + "/observe"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            obj = json.loads(resp.read().decode("utf-8", "replace"))
    except urllib.error.URLError as e:
        raise RuntimeError("连不上检测服务 %s（%s）。"
                           "请先启动 detector_service.py" % (url, e.reason))

    frame = obj.get("frame") or {}
    w = int(frame.get("w") or 0)
    h = int(frame.get("h") or 0)

    cands = []
    for i, d in enumerate(obj.get("detections") or [], start=1):
        bbox = d.get("bbox") or [0, 0, 0, 0]
        center = d.get("center") or [0, 0]
        cands.append(Candidate(idx=i, cls=str(d.get("class") or "?"),
                               conf=float(d.get("conf") or 0.0),
                               bbox=tuple(int(v) for v in bbox[:4]),
                               center=tuple(int(v) for v in center[:2]),
                               direction=str(d.get("direction") or "")))

    img_b64 = obj.get("image_b64")
    jpeg = base64.b64decode(img_b64) if img_b64 else b""
    return obj.get("frame_id"), cands, w, h, jpeg


# ---------------------------------------------------------------------------
# 主流程：观测 → 决策
# ---------------------------------------------------------------------------
def decide_from_observation(instruction, frame_id, cands, frame_w, frame_h,
                            jpeg_bytes, backend):
    """给定一次观测和一个后端，产出 Decision（纯函数，不碰网络）。

    故意把「拿观测」和「做决策」分开：这样自测可以喂假观测，
    不必起检测服务；也方便以后把观测换成录像回放做回归测试。
    """
    instruction = (instruction or "").strip()
    if not instruction:
        return Decision(action="unknown", source=backend.name,
                        instruction="", warnings=("指令为空，不调用模型",))

    # 🔴 候选为空时**不调用模型**：没有可选项，模型只能瞎编一个编号，
    #    白白花一次推理还可能被下游当真。省下的不只是算力，是正确性。
    if not cands:
        return Decision(action="unknown", source=backend.name,
                        instruction=instruction,
                        warnings=("画面里没有检测到任何物体，不调用模型",))

    # 画面缺失时仍然可以决策（模型只看文本清单），但必须记一条 warning：
    # 没有图 = 它只能靠类名和坐标猜，准确率会掉，复盘时要能看出来。
    warns = []
    if not jpeg_bytes:
        warns.append("没拿到画面 JPEG，模型只能依据文本候选清单判断")

    user = build_user_prompt(instruction, cands, frame_w, frame_h)
    reply = backend.complete(SYSTEM_PROMPT, user, jpeg_bytes)
    obj = extract_json(reply)
    d = normalize(obj, cands, reply, backend.name, instruction)
    if warns:
        d.warnings = tuple(warns) + tuple(d.warnings)
    return d


def to_dict(d):
    """序列化成与 intent_parser.to_dict() **同构的 7 键契约**。

    键集合固定，下游按键取值不用猜。语义对应关系：
      action    → 动作（同一个枚举）
      target    → 目标的类名（intent_parser 里是 "red_cup" 这类语义名，
                  这里用检测器的 COCO 类名，如 "cup"；下游按需映射）
      qualifier → 放置方位（left/right/front），没有则为 None
      params    → 模型决策的附加信息：编号、bbox、理由
      raw       → 用户原话
      source    → "mock" 或 "vlm:<模型名>"
      warnings  → 所有降级/拒绝的理由
    """
    return {
        "action": d.action,
        "target": d.target_class,
        "qualifier": d.place_side,
        "params": {
            "choice": d.target_idx,
            "bbox": list(d.target_bbox) if d.target_bbox else None,
            "center": list(d.target_center) if d.target_center else None,
            "reason": d.reason,
        },
        "raw": d.instruction,
        "source": d.source,
        "warnings": list(d.warnings),
    }


# ---------------------------------------------------------------------------
# 自测：起假 HTTP 服务，覆盖 6 种模型回复
# ---------------------------------------------------------------------------
def self_test():
    """纯逻辑自测（不联网、不需要模型、不碰机械臂）。

    做法：起一个本地假服务，同时扮演两个角色 ——
      ① /observe          返回固定的 3 个候选 + 一小段假 JPEG
      ② /v1/chat/...      按 FAKE["reply"] 返回指定的模型回复
    于是「取观测 → 拼 prompt → 调后端 → 挖 JSON → 校验 → 出契约」
    整条链路都被真跑了一遍（只有模型本身是假的）。
    """
    import threading
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    # 一小段「像 JPEG 的」字节：后端只做 base64，不解析，所以够用
    FAKE_JPEG = b"\xff\xd8\xff\xe0FAKEJPEG\xff\xd9"

    fake_dets = [
        {"class": "cup", "conf": 0.91, "bbox": [220, 180, 300, 280],
         "center": [260, 230], "direction": "左", "priority": True},
        {"class": "bottle", "conf": 0.77, "bbox": [340, 160, 400, 300],
         "center": [370, 230], "direction": "中", "priority": False},
        {"class": "book", "conf": 0.64, "bbox": [420, 200, 560, 300],
         "center": [490, 250], "direction": "右", "priority": False},
    ]

    FAKE = {"reply": "", "http_code": 200, "calls": 0}

    class FakeHandler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _json(self, obj, code=200):
            body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path.startswith("/observe"):
                self._json({
                    "frame_id": 42,
                    "timestamp": 0.0,
                    "detections": fake_dets,
                    "target": fake_dets[0],
                    "frame": {"w": 640, "h": 480},
                    "image_b64": base64.b64encode(FAKE_JPEG).decode("ascii"),
                    "image_fmt": "jpeg",
                })
            else:
                self._json({"error": "not found"}, 404)

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            self.rfile.read(n)          # 读掉请求体（不看内容，回复由 FAKE 决定）
            FAKE["calls"] += 1
            if FAKE["http_code"] != 200:
                self._json({"error": "模型服务炸了"}, FAKE["http_code"])
                return
            self._json({"choices": [{"message": {"content": FAKE["reply"]}}]})

    srv = ThreadingHTTPServer(("127.0.0.1", 0), FakeHandler)
    port = srv.server_address[1]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = "http://127.0.0.1:%d" % port

    passed = 0
    try:
        # ---------- 用例 1：正常选择 ----------
        FAKE["reply"] = json.dumps(
            {"action": "pick", "choice": 2,
             "reason": "用户说的瓶子就是它", "place_side": None},
            ensure_ascii=False)
        fid, cands, w, h, jpg = fetch_observation(base)
        assert fid == 42 and (w, h) == (640, 480), (fid, w, h)
        assert len(cands) == 3 and cands[1].cls == "bottle", cands
        assert cands[1].idx == 2 and cands[1].bbox == (340, 160, 400, 300), cands[1]
        assert jpg == FAKE_JPEG, "JPEG 应原样解出来"

        be = HttpBackend(base + "/v1", "fake-model")
        d = decide_from_observation("把瓶子抓起来", fid, cands, w, h, jpg, be)
        assert d.action == "pick", d.action
        assert d.target_class == "bottle", d.target_class
        assert d.target_bbox == (340, 160, 400, 300), d.target_bbox
        assert d.source == "vlm:fake-model", d.source
        assert not d.warnings, d.warnings
        passed += 1

        # ---------- 用例 2：编号越界必须拒绝 ----------
        FAKE["reply"] = json.dumps(
            {"action": "pick", "choice": 99, "reason": "瞎编的编号"})
        d = decide_from_observation("把瓶子抓起来", fid, cands, w, h, jpg, be)
        assert d.action == "unknown", "越界编号必须落 UNKNOWN，不能执行"
        assert d.target_bbox is None
        assert any("99" in w for w in d.warnings), d.warnings
        passed += 1

        # ---------- 用例 3：markdown 围栏 ----------
        FAKE["reply"] = '```json\n{"action":"pick","choice":1,"reason":"红杯子"}\n```'
        d = decide_from_observation("把杯子抓起来", fid, cands, w, h, jpg, be)
        assert d.action == "pick" and d.target_class == "cup", (d.action, d.target_class)
        passed += 1

        # ---------- 用例 4：乱码回复 ----------
        FAKE["reply"] = "抱歉，我无法完成这个任务。"
        d = decide_from_observation("把杯子抓起来", fid, cands, w, h, jpg, be)
        assert d.action == "unknown", d.action
        assert any("JSON" in w for w in d.warnings), d.warnings
        passed += 1

        # ---------- 用例 5：空候选时不该调用模型 ----------
        before = FAKE["calls"]
        d = decide_from_observation("把杯子抓起来", fid, [], 640, 480, jpg, be)
        assert d.action == "unknown"
        assert FAKE["calls"] == before, "候选为空时绝不能调模型（会白烧算力还会瞎编）"
        passed += 1

        # ---------- 用例 6：模型服务 500 要抛清晰异常 ----------
        FAKE["http_code"] = 500
        FAKE["reply"] = ""
        try:
            decide_from_observation("把杯子抓起来", fid, cands, w, h, jpg, be)
        except RuntimeError as e:
            assert "500" in str(e), str(e)
        else:
            raise AssertionError("HTTP 500 必须抛 RuntimeError，不能静默返回 UNKNOWN")
        FAKE["http_code"] = 200
        passed += 1

        # ---------- 用例 7：契约 7 键必须与 intent_parser 对齐 ----------
        FAKE["reply"] = json.dumps({"action": "pick", "choice": 3,
                                    "reason": "书", "place_side": None},
                                   ensure_ascii=False)
        d = decide_from_observation("把书抓起来", fid, cands, w, h, jpg, be)
        data = to_dict(d)
        assert set(data) == {"action", "target", "qualifier", "params",
                             "raw", "source", "warnings"}, set(data)
        assert data["action"] == "pick" and data["target"] == "book", data
        assert data["params"]["bbox"] == [420, 200, 560, 300], data["params"]
        passed += 1

        # ---------- 用例 8：MockBackend 按中文关键词选对 ----------
        mb = MockBackend()
        d = decide_from_observation("把瓶子抓起来", fid, cands, w, h, jpg, mb)
        assert d.action == "pick" and d.target_class == "bottle", d.target_class
        d = decide_from_observation("把杯子抓起来", fid, cands, w, h, jpg, mb)
        assert d.target_class == "cup", d.target_class
        passed += 1

    finally:
        srv.shutdown()

    print("SELF_TEST PASS（%d 组用例全过，未联网、未用真模型、未动臂）" % passed)
    print()
    print("prompt 样例（模型实际看到的内容）：")
    print("-" * 66)
    demo_cands = [
        Candidate(1, "cup", 0.91, (220, 180, 300, 280), (260, 230), "左"),
        Candidate(2, "bottle", 0.77, (340, 160, 400, 300), (370, 230), "中"),
        Candidate(3, "book", 0.64, (420, 200, 560, 300), (490, 250), "右"),
    ]
    print(build_user_prompt("把左边的杯子抓起来放进右边", demo_cands, 640, 480))
    print("-" * 66)
    return 0


# ---------------------------------------------------------------------------
# 命令行入口
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="VLM 语义决策层：从候选里选编号，不输出坐标")
    ap.add_argument("--self-test", action="store_true",
                    help="纯逻辑自测（假服务、不联网、不需要模型）")
    ap.add_argument("--instruction", default=None, help="用户指令，如「把杯子抓起来」")
    ap.add_argument("--detector", default="http://127.0.0.1:8080",
                    help="检测服务地址（默认 http://127.0.0.1:8080）")
    ap.add_argument("--backend", default="http",
                    choices=("http", "mock"),
                    help="http=OpenAI 兼容的真实模型；mock=不联网假模型")
    ap.add_argument("--base-url", default="http://127.0.0.1:11434/v1",
                    help="OpenAI 兼容 base_url（默认 ollama 本地）")
    ap.add_argument("--model", default="qwen2.5vl:3b",
                    help="模型名（ollama 里是 qwen2.5vl:3b，云端填服务商的名字）")
    ap.add_argument("--timeout", type=float, default=60.0,
                    help="模型调用超时秒数（本地 3B 首次加载慢，留 60s）")
    ap.add_argument("--dry-run", action="store_true",
                    help="只打印 prompt，不调用模型")
    ap.add_argument("--pretty", action="store_true", help="JSON 缩进输出")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    if not args.instruction:
        print("错误：需要 --instruction（或用 --self-test）", file=sys.stderr)
        return 2

    # Key 只从环境变量读，绝不接受命令行明文（避免进 shell history / 截图）
    api_key = os.environ.get("VLM_API_KEY", "")

    if args.backend == "mock":
        backend = MockBackend()
    else:
        backend = HttpBackend(args.base_url, args.model, api_key,
                              timeout=args.timeout)

    try:
        fid, cands, w, h, jpg = fetch_observation(args.detector)
    except RuntimeError as e:
        print("观测失败：%s" % e, file=sys.stderr)
        return 3

    print("观测：帧号 %s ｜ 候选 %d 个 ｜ 画面 %d×%d ｜ 图像 %d 字节"
          % (fid, len(cands), w, h, len(jpg)))
    for c in cands:
        print("  [%d] %-12s conf=%.2f center=(%d,%d) 方位=%s"
              % (c.idx, c.cls, c.conf, c.center[0], c.center[1], c.direction))

    if args.dry_run:
        print("\n" + "=" * 66)
        print("SYSTEM:")
        print(SYSTEM_PROMPT)
        print("\nUSER:")
        print(build_user_prompt(args.instruction, cands, w, h))
        print("=" * 66)
        print("（--dry-run：未调用模型）")
        return 0

    try:
        d = decide_from_observation(args.instruction, fid, cands, w, h, jpg, backend)
    except RuntimeError as e:
        print("决策失败：%s" % e, file=sys.stderr)
        return 4

    data = to_dict(d)
    print("\n决策：")
    print(json.dumps(data, ensure_ascii=False, indent=2 if args.pretty else None))

    if d.action == "unknown":
        print("\n⚠ 未产出可执行动作，原因见 warnings。", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
