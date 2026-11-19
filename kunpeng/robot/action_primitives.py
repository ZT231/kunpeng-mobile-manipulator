#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
action_primitives.py —— 具身智能的「动作原语层」：把一次抓取拆成可校验的原子步骤

大白话：以前网页上点一下，只发一条 JOINT 就完事，臂动到哪算哪。
       但「抓一个东西」其实是好几步：先张开爪、移到物体正上方、降下去、
       合爪、再抬起来。每一步都必须**确认真的到了**再走下一步，
       否则臂跑到一半就开始合爪，抓到的是空气。
       这个文件就是把这套步骤写死、写全、每步都回读关节角核对。

规范定义：动作原语（action primitives）是机器人技能的最小可复用单元，
         在分层式具身架构里位于「决策层（大模型/规划器）」与
         「执行层（伺服控制）」之间，通常实现为带前置条件与后置校验的
         参数化技能（parameterized skills）—— 对应本文的 pick()/place()/home()。

🔴 诚实边界（README 里也写一份，别越线）：
  1) 本文件**不产生任何决策**。抓哪里、抓哪个，全部由调用方给定。
     有它 ≠ 有具身智能，它只是「行动」那一环的地基。
  2) 抓取是**俯视平面抓取**：夹爪始终竖直向下（tilt=0），只适用于躺在
     桌面上的单个物体。堆叠、侧躺、需要侧向接近的场景一概不支持。
  3) 执行是**开环脉宽**：舵机没有位置反馈，靠的是「固件斜坡把脉宽推到目标值」
     再读回脉宽确认。夹到东西后 J6 可能到不了目标值（堵转），
     所以 J6 不参与到位判据。
  4) 没有力控、没有碰撞检测。下降高度靠调用方给对 z，给低了会撞桌子。

为什么单独一个文件，而不是塞进 web_control.py：
  和 intent_parser.py 同一个道理 —— 5000 网页服务是已实机验证过的主链路，
  改它就要 scp + 重启 Flask，重启会清掉 _IK_MAP 标定值、打断现场。
  这里只通过 HTTP 调它已有的 /api/cmd 与 /api/state，主链路一行不碰，
  删掉这个文件就是完整回滚。

用法（全部零第三方依赖，只要标准库 + 同目录 arm_ik.py）：
  python3 action_primitives.py --self-test            # 纯逻辑自检，不联网不动臂
  python3 action_primitives.py --dry-run pick 20 25   # 只算脉宽并打印，不发命令
  python3 action_primitives.py state                  # 读当前 6 路脉宽
  python3 action_primitives.py home                   # 回初始位
  python3 action_primitives.py pick 20 25 --yes       # 真的抓（z 用 grab_height）
  python3 action_primitives.py place 10 30 --yes      # 真的放
"""

import argparse
import dataclasses
import json
import os
import sys
import time
import urllib.error
import urllib.request


# ---------------------------------------------------------------------------
# 常量：全部集中在这里，别散到函数里
# ---------------------------------------------------------------------------
BASE_URL = "http://127.0.0.1:5000"

# 示教配方库：与本文件同目录（scp 时一起拷到 /root/kunpeng_vision/）。
# 用 __file__ 定位而不是相对路径 —— 板上常在别的目录下跑 `python3 /root/.../action_primitives.py`，
# 相对路径会找不到文件，而报错信息往往只写「No such file」，排查很费劲。
RECIPES_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "pick_recipes.json")

JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6")

# 🔴 这 6 组是**固件软限位**（Core/Inc/main.h 的 SERVO_Jx_MIN/MAX），
#    与 kunpeng/web/web_control.py 的 PULSE_RANGES 必须逐字一致。
#    改任何一处，另外两处 + docs/舵机标定记录.md 都要同步，
#    否则会出现「Python 以为能到 2500、固件静默裁到 1850」这种假到位。
#    注意 J4 特别窄（500~1850）：它在 arm_params.json 的 map 里到 2500，
#    但那是映射自变量范围，不是机械行程上限。
SAFE_LIMITS = (
    (800, 2300),   # J1 基座回转
    (750, 2150),   # J2 大臂
    (520, 2500),   # J3 小臂
    (500, 1850),   # J4 腕俯仰（软限位收窄）
    (500, 2500),   # J5 腕回转
    (900, 2000),   # J6 夹爪：900 全张 / 1900 闭合
)

# 初始位（与固件 Arm_Init() 的 INIT 值一致）
HOME_PULSES = (1580, 1900, 2400, 700, 1450, 1500)

# 夹爪三个实测工作点（11-16 用 3.8cm 喷雾瓶逐点量出来的，别凭印象改）：
#   950  → 开口 6.8cm，全张；抓前用它下放才不会撞到物体
#   1550 → 约 3.5cm。1500 时爪指刚好**几何贴合** 3.8cm 瓶身，再合 50µs 形成过盈
#   1900 → 近乎全闭 ＝ **空夹**，抓任何 <7cm 的物体走到这里都会堵转发热
# 🔴 11-16 实机教训：J6 一路发到 1900，4 步全报「✓ 到位」却**碰都没碰到瓶子**——
#    因为臂到位了、爪子却在空气里合到底。抓取闭合值必须是 1550 这个量级。
#    换物体用 jaw_for_width() 重算，别硬套 1550。
JAW_OPEN = 950
JAW_CLOSE = 1550
JAW_CLOSED_MAX = 1900   # 完全闭合（空夹到底）：只给空动作/自检用，不用于抓取

# 到位判据：6 路脉宽与目标的差都 ≤ 这个值就算到了。
# 8 µs ≈ 软件步长 25 µs 的三分之一，比舵机死区（5 µs）略宽。
ARRIVE_TOL_US = 8

# 固件斜坡速度：每 20 ms 最多走 25 µs（main.h SERVO_STEP_US），即 1250 µs/s。
# 用它估算「发完要等多久」，超时给估算值的 2 倍 + 0.5 s 余量。
RAMP_US_PER_SEC = 1250.0

# 开合爪到位后再停一下，让舵机扭矩建立/释放。
# 有 wait_jaw 兜着，这里只需很短 —— 0.6 s 是 wait_jaw 之前留下的旧值，
# 当时靠它硬等，现在等到位已经是精确的，0.2 s 够扭矩稳住。
SETTLE_AFTER_JAW_S = 0.2


# ---------------------------------------------------------------------------
# 数据结构：把「一步动作」写成一个带名字的袋子
# ---------------------------------------------------------------------------
@dataclasses.dataclass
class Step:
    """一个原子步骤：名字 + 目标 6 路脉宽 + 人话说明。

    为什么用 dataclass 而不是裸 tuple：plan_pick() 返回一串步骤，
    dry-run 时要逐条打印给人看，字段有名字才读得懂；
    写错字段名也会立刻 AttributeError，不会静默出错。
    """
    name: str
    pulses: tuple
    note: str = ""

    def as_dict(self):
        return {"name": self.name, "pulses": list(self.pulses), "note": self.note}


@dataclasses.dataclass
class Plan:
    """整段动作计划：步骤列表 + 所有需要人注意的警告。

    🔴 warnings 不静默：目标被软限位裁过、某步根本没解出来，
    都必须让调用方看到，而不是「发下去了就假装成功」。
    """
    steps: list = dataclasses.field(default_factory=list)
    warnings: list = dataclasses.field(default_factory=list)


# ---------------------------------------------------------------------------
# HTTP 层：只和 Flask 已有的两个接口说话
# ---------------------------------------------------------------------------
def _post_json(path, obj, timeout=5.0):
    """POST 一个 JSON 到 Flask，返回解析后的 dict（失败抛 RuntimeError）。

    为什么用标准库 urllib 而不是 requests：板上环境是「不擅自装包」的，
    urllib 一定在；这一层只有 10 行，不值得为它引入依赖。
    """
    body = json.dumps(obj).encode("utf-8")
    req = urllib.request.Request(
        BASE_URL + path, data=body,
        headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except urllib.error.URLError as e:
        raise RuntimeError("连不上 %s（Flask 没起？）：%s" % (path, e))


def read_state(timeout=5.0):
    """读当前 6 路实际脉宽，返回 tuple[int x6]。

    🔴 这是全文件唯一的「真实状态来源」。任何一步动作之后都必须靠它确认，
    不能假设「发下去了就是到位了」—— 那正是开环控制的经典谎报。
    """
    got = _post_json("/api/state", {}, timeout=timeout)
    if not got.get("ok"):
        raise RuntimeError("读状态失败：%s" % got.get("error", got))
    pulses = got.get("pulses")
    if not (isinstance(pulses, list) and len(pulses) == 6):
        raise RuntimeError("状态格式异常：%r" % (pulses,))
    return tuple(int(p) for p in pulses)


def send_joint(target, timeout=5.0):
    """发一条 JOINT 并检查固件回执是否为 OK。

    🔴 为什么必须看 resp：/api/cmd 的 ok:true 只代表「串口写进去了」，
    固件真正执行结果在 resp 里（OK / ERR,1 未知命令 / ERR,3 参数不合法）。
    只判 ok 不看 resp，就会把「帧格式写错」当成「执行成功」。
    """
    payload = "JOINT," + ",".join(str(int(p)) for p in target)
    got = _post_json("/api/cmd", {"payload": payload}, timeout=timeout)
    if not got.get("ok"):
        raise RuntimeError("下发失败：%s" % got.get("error", got))
    resp = (got.get("resp") or "").strip()
    if resp != "OK":
        raise RuntimeError("固件拒绝执行（resp=%r），6 路目标 = %s"
                           % (resp, list(target)))


# ---------------------------------------------------------------------------
# 纯函数层：只算不发，可以离线单测、可以 dry-run
# ---------------------------------------------------------------------------
def clip_pulses(raw, limits=SAFE_LIMITS):
    """把 6 路脉宽裁到软限位内，返回 (裁剪后的 tuple, 被裁的关节名列表)。

    🔴 为什么不交给固件自动裁：固件的 clamp_pulse 是**静默**的，
    裁完照样回 OK。人会以为「到目标位置了」，实际差了十几度。
    这里显式裁并回报，调用方才能决定是报错还是接受。
    """
    out, clamped = [], []
    for i, p in enumerate(raw):
        lo, hi = limits[i]
        q = int(round(float(p)))
        q = max(lo, min(hi, q))
        if q != int(round(float(p))):
            clamped.append(JOINT_NAMES[i])
        out.append(q)
    return tuple(out), clamped


def estimate_travel_s(src, dst):
    """估算从 src 走到 dst 要几秒（取 6 路里走得最远的那一路）。

    为什么不用固定 3 秒：J6 只动 20 µs 却等 3 秒，抓一次要十几秒；
    而大范围移动固定 3 秒又可能不够。按最远那一路算最省也最稳。
    """
    far = max(abs(int(b) - int(a)) for a, b in zip(src, dst))
    return far / RAMP_US_PER_SEC


# ---------------------------------------------------------------------------
# 示教配方层：把人在实机上调好的 6 路脉宽存成「配方」，一键回放
#
# 为什么需要这一层：开环臂没有位置反馈，逆解里又缺重力下垂补偿（SAG_DZ），
# 于是「算出来的姿态」和「真能抓住的姿态」差 169~749 µs（11-16 实测量）。
# 而人眼一调就准。把人的那次调法记下来回放，比每次重算可靠得多。
# ---------------------------------------------------------------------------
def load_recipes(path=None):
    """读示教配方库，返回整个 dict（含 recipes / jaw / j1 段）。

    大白话：配方就是「人教过的动作录像」，一条条脉宽存下来，下次照着演一遍。
    规范定义：teach & playback（示教-再现），工业机器人最基础的示教编程方式 ——
      操作者手动把末端带到目标位姿，控制器记录各关节值，之后按记录回放。
      对**开环、无位置反馈**的臂，它比每次重算逆解可靠得多。
    """
    p = path or RECIPES_PATH
    try:
        with open(p, "r", encoding="utf-8") as f:
            data = json.load(f)
    except FileNotFoundError:
        raise RuntimeError("找不到配方库 %s（把 pick_recipes.json 和本文件放同一目录）" % p)
    except ValueError as exc:
        raise RuntimeError("配方库 %s 不是合法 JSON：%s" % (p, exc))
    if not isinstance(data.get("recipes"), dict) or not data["recipes"]:
        raise RuntimeError("配方库 %s 缺 recipes 段或为空" % p)
    return data


def build_recipe_plan(recipe, j1_offset=0):
    """把一条示教配方转成 Plan（纯函数：可 dry-run、可单测、不发命令）。

    j1_offset：J1 附加偏移（µs）。视觉算出目标方位角与示教工位不同时，
      把角度差乘 j1.us_per_deg 加在这里。
      🔴 符号没在实机上确认过 —— 先用 ±20 µs 小步试方向，别一次给几十。
    """
    raw_steps = recipe.get("steps") or []
    if not raw_steps:
        raise RuntimeError("配方没有 steps")
    steps, warnings = [], []
    for i, st in enumerate(raw_steps, 1):
        pulses = [int(v) for v in (st.get("pulses") or [])]
        if len(pulses) != 6:
            raise RuntimeError("配方第 %d 步脉宽不是 6 路：%r" % (i, pulses))
        if j1_offset:
            pulses[0] += int(j1_offset)
        pulses, clamped = clip_pulses(pulses)
        name = str(st.get("name") or ("步骤%d" % i))
        note = "示教配方第 %d/%d 步" % (i, len(raw_steps))
        if j1_offset:
            note += "｜J1 偏移 %+d µs" % int(j1_offset)
        if clamped:
            note += "；⚠ 被软限位裁：%s" % "/".join(clamped)
            warnings.append("%s：目标越界，已裁 %s" % (name, "/".join(clamped)))
        steps.append(Step(name=name, pulses=tuple(pulses), note=note))
    return Plan(steps=steps, warnings=warnings)


def j1_pulse_from_xy(x, y, ik=None, az_bias_deg=0.0):
    """工作台坐标 (x, y) → J1 脉宽（只算方位角，不做到达判定）。

    🔴 为什么把 J1 单独拿出来：J1 只由方位角决定，跟伸多远、倾角多少、
    能不能够到都没关系。所以「视觉定 J1 + 示教定 J2~J5」这条路里，J1 可以
    直接用视觉结果——哪怕 IK 因为 J2~J4 越界而解不出来。

    实测校验：示教工位 (−5.26, 25.0) → 1691（与 IK 的 J1 一致）；
             (−4.19, 23.65) → 1674（IK 给 1673，差 1µs）。

    az_bias_deg：补偿视觉方位角的固定偏小（11-16 尺子实测 −2.98°，
    与历史记录的 −2.7° 吻合）。默认 0，**不静默改**。
    """
    ik = ik or _get_ik()
    return ik.j1_pulse_for_xy(x, y, az_bias_deg=az_bias_deg)


def jaw_for_width(width_cm, grip_ref_pulses=1500, ref_cm=3.8,
                  us_per_cm=185.0, margin_us=50):
    """按物体宽度估 J6 夹持脉宽（两个实测点之间线性插值）。

    实测两点：J6=950 → 开口 6.8cm；J6=1500 → 约 3.8cm ⇒ 斜率 ≈185 µs/cm。
    物体越细 ⇒ 要合得越多 ⇒ 脉宽越大（J6 增大＝闭合）。
      3.8cm → 1550（实机 3/3 验证过）；2cm → 1883；5cm → 1329。
    ⚠️ 只在 2~6.5cm 区间大致可信，且**不能替代实物试夹**。
    ⚠️ 别拿 pick_pipeline 的 span_cm 当 width —— 它按 bbox 反投影算，实测高估约 60%。
    """
    return int(round(grip_ref_pulses + margin_us
                     + (float(ref_cm) - float(width_cm)) * us_per_cm))


def apply_tip_compensation(x, y, z):
    """把「想打到的点」换算成「IK 该瞄的点」。

    执行侧两个实测偏置（C 姿态族、方位角 96°、2026-11-12 量）：
      TIP_BIAS_XY = (0.10, −0.95) cm —— 爪尖实际落点 = FK 点 + bias
      SAG_DZ      = −3.88 cm        —— 爪尖实际高度 = 指令 z + SAG_DZ

    想让爪尖真打到 (x, y, z)，IK 就得瞄 (x − dx, y − dy, z − dz)。
    也就是：水平往回让 0.1 / 0.95 cm，垂直往上抬 3.88 cm。

    🔴 为什么必须补 z（这条以前是刻意不补的）：11-16 实测「IK 算的悬停姿态
      相对瓶子偏下 3cm」，与 |SAG_DZ|=3.88 完全吻合 —— 不补就永远矮 3.88cm。
       代价是抬 z 会让 J3/J4 一起变，下垂量随之变，所以这是**一阶经验补偿**，
       不是精确模型：越远离 C 姿态族越不准。
    ⚠️ 绝不能在标定链路里调用它（X 里吸收的正是「未补偿的那版 sag」）。
    """
    from arm_ik import TIP_BIAS_XY, SAG_DZ
    return (float(x) - TIP_BIAS_XY[0],
            float(y) - TIP_BIAS_XY[1],
            float(z) - SAG_DZ)


def best_tilt(x, y, z, ik=None, prefer=96.0, floor=41.0, step=2.0,
              compensate=True, j3_margin=60):
    """给一个目标点，找「最接近水平、且 6 路都不越软限位」的 tilt。

    为什么需要它：抓竖直圆柱时爪指越接近水平越稳，但 J4 软限位只有 1850。
    11-16 实扫（目标 x=-5.26 y=25.0）：
        tilt=41 → J4=1230 可用（但爪斜 41°，夹圆柱不稳）
        tilt=75 → J4=1851 刚好越界 1µs
        tilt=96 → J4=2257 远超
    所以「最平可用姿态」是随目标点变的，必须每个点重扫，不能写死。

    参数 j3_margin：J3 要留多少 µs 余量（默认 60）。🔴 为什么要留：
      11-16 扫 12 个点发现，不设这项时 **J3 全部顶在 2500**（用户示教只到 2320）。
      顶在软限位上虽然「不算越界」，但固件走到限位就走不动了，实际末端会偏，
      而且长时间顶住会堵转发热。留 60µs 换一个略斜一点的姿态，比顶死划算。

    返回 (tilt, pulses)；实在没有可行解则返回 None。
    🔴 扫描从大到小：先试最平的，不行才往斜了退，保证「尽可能水平」。
    """
    ik = ik or _get_ik()
    tx, ty, tz = apply_tip_compensation(x, y, z) if compensate else (x, y, z)
    j3_hi = SAFE_LIMITS[2][1] - int(j3_margin)
    t = float(prefer)
    while t >= float(floor) - 1e-9:
        try:
            ang = ik.solve(tx, ty, z=tz, tilt=t)
            pulses, clamped = clip_pulses(ik.to_pulses(ang))
        except Exception:
            t -= step          # 这个倾角解不出来（不可达 / 数值问题），退一档再试
            continue
        # 6 路都在软限位内、且 J3 还留了余量 —— 就是它
        if not clamped and pulses[2] <= j3_hi:
            return float(t), tuple(int(round(v)) for v in pulses)
        t -= step
    return None


def plan_move(x, y, z, jaw=None, ik=None, tilt=0.0, compensate=True):
    """工作台坐标 (x,y,z) → 一个「移过去」的 Step（纯函数，不发命令）。

    参数 jaw：不为 None 时用它覆盖 J6（夹爪），因为 IK 解出来的 J6 是
    「claw 角」对应的脉宽，而抓取序列要独立控制爪子开合。

    参数 tilt：夹爪前倾角(度)。🔴 必须与「拍照那一刻」的真实姿态一致：
      solve() 里 r_w = r − L3·sin(tilt)，用错 tilt 会让可达判定与落点都偏。
      tilt=0（默认）只适用于真的垂直抓取；眼在手链路应由调用方用
      `ik.tilt_from_pulses(pulses)` 从当前脉宽反算后传入（C 姿态 = 41.00°）。
      历史坑：默认 0 曾让 24cm 的目标报「超出可达范围 d=28.0」。

    参数 compensate：是否扣执行侧重力偏置（见 apply_tip_compensation）。
      🔴 默认 **True** —— 11-16 实测不扣会矮 3.88cm、近 0.95cm，
      表现就是「4 步全到位却碰都没碰到」。传 False 只用于复现旧行为/单测。
    """
    ik = ik or _get_ik()
    if compensate:
        tx, ty, tz = apply_tip_compensation(x, y, z)
    else:
        tx, ty, tz = float(x), float(y), float(z)
    angles = ik.solve(tx, ty, z=tz, tilt=tilt)
    raw = ik.to_pulses(angles)
    pulses, clamped = clip_pulses(raw)
    if jaw is not None:
        pulse_list = list(pulses)
        pulse_list[5] = int(jaw)
        pulses, clamped2 = clip_pulses(pulse_list)
        clamped = clamped + clamped2
    note = "工作台 (%.1f, %.1f, %.1f) cm" % (x, y, z)
    if compensate:
        note += "｜已补重力偏置（瞄 %.2f, %.2f, %.2f）" % (tx, ty, tz)
    if clamped:
        note += "；⚠ 被软限位裁：%s" % "/".join(clamped)
    return Step(name="move", pulses=pulses, note=note), clamped


def plan_pick(x, y, z=None, hover=4.0, jaw_open=JAW_OPEN, jaw_close=JAW_CLOSE,
              ik=None, tilt=0.0, auto_tilt=False, compensate=True):
    """规划一次抓取：移到上方 → 下降 → 合爪 → 抬起（纯函数）。

    hover = 在目标上方多少 cm 处开始下降。4 cm 是经验值：
    太矮会撞到相邻物体，太高则下降段长、耗时且更容易在途中蹭到东西。

    🔴 为什么是 4 步而不是 5 步：最初写的是「张开爪子 → 移到上方 → …」，
    dry-run 打出来发现前两步的 6 路脉宽**一模一样** —— 因为第一步的目标
    也是「上方 + 张爪」，第二步就没活干了。合并成一步：移动时就把 J6
    定在张开值，落到上方时爪已经是开的，下降段不会顶到物体。

    参数 auto_tilt：True 时不用调用方给的 tilt，而是对**抓取点**扫出
      「最接近水平且不越软限位」的那个（见 best_tilt）。这是「物体放到
      别的点也能抓」的关键 —— 写死 41° 爪指斜着夹不稳，写死 96° 又 J4 超限。
    参数 compensate：透传给 plan_move（默认 True，扣重力偏置）。
    """
    ik = ik or _get_ik()
    if z is None:
        z = ik.grab_h
    if auto_tilt:
        got = best_tilt(x, y, z, ik=ik, compensate=compensate)
        if got is None:
            raise ValueError("目标 (%.2f, %.2f, %.2f) 在 41~96° 内找不到不越软限位的姿态"
                             % (x, y, z))
        tilt = got[0]
    z_top = z + hover

    plan = Plan()
    seq = (
        ("移到上方", z_top, jaw_open, "移动时即张爪，高于目标 %.1f cm" % hover),
        ("下降到抓取高度", z, jaw_open, "这里是唯一可能撞到物体的时刻"),
        ("合爪", z, jaw_close, "停 %.1f s 等扭矩建立" % SETTLE_AFTER_JAW_S),
        ("抬起", z_top, jaw_close, "带着物体回到安全高度"),
    )
    for name, zz, jaw, note in seq:
        step, clamped = plan_move(x, y, zz, jaw=jaw, ik=ik, tilt=tilt,
                                  compensate=compensate)
        step.name = name
        step.note = note + "｜" + step.note
        plan.steps.append(step)
        if clamped:
            plan.warnings.append("%s：目标越界，已裁 %s" % (name, "/".join(clamped)))

    # 🔴 关键校验：如果任何一步被裁过，抓取位置就和预期不符，
    #    整个 pick 的可信度都要打问号 —— 必须让调用方知道。
    if plan.warnings:
        plan.warnings.append("有步骤被软限位裁剪，本次抓取的落点可能不准")
    return plan


def plan_place(x, y, z=None, hover=4.0, jaw_open=JAW_OPEN,
               jaw_close=JAW_CLOSE, ik=None, tilt=0.0,
               auto_tilt=False, compensate=True):
    """规划一次放置：移到上方 → 下降 → 张爪 → 抬起（纯函数）。

    tilt / auto_tilt / compensate 语义同 plan_pick：前倾角要么显式给、
    要么对目标点扫出「最平可用」的那个；重力偏置默认要扣。
    """
    ik = ik or _get_ik()
    if z is None:
        z = ik.grab_h
    if auto_tilt:
        got = best_tilt(x, y, z, ik=ik, compensate=compensate)
        if got is not None:
            tilt = got[0]
    z_top = z + hover

    plan = Plan()
    seq = (
        ("移到上方", z_top, jaw_close, "保持夹紧，别在半路掉东西"),
        ("下降到放置高度", z, jaw_close, "降到目标平面"),
        ("张爪", z, jaw_open, "停 %.1f s 等物体脱离" % SETTLE_AFTER_JAW_S),
        ("抬起", z_top, jaw_open, "空爪回到安全高度"),
    )
    for name, zz, jaw, note in seq:
        step, clamped = plan_move(x, y, zz, jaw=jaw, ik=ik, tilt=tilt,
                                  compensate=compensate)
        step.name = name
        step.note = note + "｜" + step.note
        plan.steps.append(step)
        if clamped:
            plan.warnings.append("%s：目标越界，已裁 %s" % (name, "/".join(clamped)))
    return plan


def plan_home():
    """回初始位（纯函数，不读硬件）。"""
    return Plan(steps=[Step(name="回初始位", pulses=tuple(HOME_PULSES),
                            note="6 路拉回 Arm_Init() 的 INIT 值")])


# ---------------------------------------------------------------------------
# 执行层：这里才开始真的动臂
# ---------------------------------------------------------------------------
def wait_arrive(target, timeout=None, tol=ARRIVE_TOL_US, check_j6=False,
                poll=0.1):
    """轮询 /api/state 直到 6 路脉宽都接近 target，返回实际读数。

    🔴 check_j6=False 是有意为之：夹爪夹住物体后舵机堵转，
    脉宽可能永远停在 1800 而到不了 1900。若把 J6 计入判据，
    每次成功抓取都会被判成「超时失败」。

    返回 (实际脉宽 tuple, 是否在超时前到位)。
    🔴 超时**不抛异常**，返回 False 让调用方自己决定（比如继续往下走、
    还是中止整段动作）。抓取途中抛异常会把臂留在半路，比继续更危险。
    """
    if timeout is None:
        src = read_state()
        timeout = estimate_travel_s(src, target) * 2.0 + 0.5

    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        last = read_state()
        idx = range(5) if not check_j6 else range(6)
        if all(abs(last[i] - int(target[i])) <= tol for i in idx):
            return last, True
        time.sleep(poll)
    return last, False


def wait_jaw(target, timeout=None, tol=60, poll=0.08):
    """等夹爪走到目标脉宽，返回 (实际脉宽, 是否接近目标)。

    🔴 这个函数**不判失败**，因为到不了是正常的：夹住物体后舵机堵转，
    J6 会停在半路（比如目标 1900 实际只到 1750）。所以它的职责只是
    「确保爪子真的动过、且给了它足够时间」，而不是精确校验到位。

    容差 60 µs 比关节的 8 µs 宽得多，同理 —— 夹爪本来就不该被当成
    精密定位轴来要求。

    🔴 为什么必须有它：J6 从 950 合到 1900 要走 950 µs，
    按固件斜坡 1250 µs/s 需要 **0.76 s**。如果只靠 wait_arrive（跳过 J6）
    加一个固定 sleep，sleep 短了就会「爪还没夹紧就抬走」，物体直接滑掉。
    这个 bug 是假 Flask 测试里打出来的：合爪一步报「到位」时 J6 才 1550。
    """
    src = read_state()
    if timeout is None:
        need = abs(int(target) - int(src[5])) / RAMP_US_PER_SEC
        timeout = need + 0.8          # 斜坡时间 + 余量
    deadline = time.time() + timeout
    last = src[5]
    while time.time() < deadline:
        last = read_state()[5]
        if abs(last - int(target)) <= tol:
            return last, True
        time.sleep(poll)
    return last, False


# ---------------------------------------------------------------------------
# 电流引导的分步合爪（接触即停）—— 是「接触检测」，不是「力控」
# ---------------------------------------------------------------------------
CUR_SAMPLES          = 3      # 每次判定读几遍取中位数（接触后单次读数会出 ±25mA 毛刺）
GRIP_STEP_COARSE     = 40     # 粗段步长 µs：离接触还远，走快点省时间
GRIP_STEP_FINE       = 15     # 细段步长 µs：接近接触区必须小步，别一跨就过
GRIP_COARSE_UNTIL    = 1480   # 粗段终点 µs（11-16 实测：1500~1510 才首次抬升）
GRIP_RISE_MA         = 45     # 「碰上了」的弱判据（只用来记候选点，不停爪）
GRIP_HOLD_MA         = 110    # 「夹住了」的强判据（到这儿才真停）
GRIP_PRELOAD         = 40     # 备选：只出现弱信号、始终没到强判据时，在候选点再压这么多 µs
GRIP_STALL_MA        = 600    # 兜底 mA：超过就回退，别硬顶（≈3.2A，离 4.3A 饱和仍有余量）
# 为什么强判据取 110 而不是 90（11-16 三点实测标定）：
#   空载基线本身会漂（同一台臂量到过 -10 / +27 / +37 mA，J6 在不同位置机械阻力不同）。
#   取 110 时，三种基线下落点分别是：base=-10→1550、base=0→1550、base=27→1570、
#   base=37→1570 —— 都在「抬得动」的验证区间内，最多偏一个细步+一个粗步。
#   取 90 的话 base=-10 会让 1510（81mA）就误判成夹住，而那处开口约 4.2cm、夹不住 3.8cm 瓶。
GRIP_DWELL_COARSE_S  = 0.22
GRIP_DWELL_FINE_S    = 0.35
# 🔴 单次 J6 跳变上限：19:2x 实机踩到的坑 —— 用 --from 1430 从 950 一步跳过去，
#    480µs 的行程让舵机启动电流冲到「夹住」判据以上，第一次读数就成了 +135，
#    于是在 1430 直接判夹住（那时开口 4.6cm，根本没碰到瓶子）。
#    ⇒ 起点离当前位置超过这个值就夹回来，宁可多走几步也不让瞬态污染判据。
GRIP_MAX_FROM_JUMP   = 80
GRIP_SETTLE_S        = 0.35   # 到达起始点后的静置时间（等启动冲击过去再取基线）
GRIP_MAX_JAW         = 1700   # 扫到这儿还没接触就判「夹空」，别再往里合


def _median(vals):
    """中位数。为什么不取平均：接触后的抖动是**单向毛刺**（爪在找平衡、
    瓶身缓慢变形），中位数把偶发高/低点直接剔掉，平均值会被它拉偏。"""
    s = sorted(vals)
    n = len(s)
    if n == 0:
        raise ValueError("空序列没有中位数")
    if n % 2:
        return s[n // 2]
    return (s[n // 2 - 1] + s[n // 2]) // 2


def read_current_ma(samples=CUR_SAMPLES, gap=0.06):
    """读 J6 电流，多读几次取中位数，返回 int(mA)。

    🔴 固件里 $CUR 有三种非数值回包，必须分别报错 —— 把任何一种当成 0，
    都会让「夹空」被误判成「已经夹住」：
      - `CUR,NA`  = 注入通道没配置成功（代码/初始化问题，不是接线问题）
      - `CUR,SAT` = ADC 封顶（真实电流 >4.3A，几乎只有「串错到母线」才会这样）
      - 空/超时   = 串口掉线
    """
    vals = []
    for _ in range(max(1, int(samples))):
        got = _post_json("/api/cmd", {"payload": "CUR"})
        if not got.get("ok"):
            raise RuntimeError("读电流失败：%s" % got.get("error", got))
        resp = (got.get("resp") or "").strip()
        if not resp.startswith("CUR,"):
            raise RuntimeError("$CUR 回包异常：%r" % resp)
        body = resp[4:]
        if body == "NA":
            raise RuntimeError("$CUR 回 NA：ADC 注入通道没配置成功"
                               "（看串口有没有 `ACS712 init FAILED`）")
        if body == "SAT":
            raise RuntimeError("$CUR 回 SAT：电流超过 ADC 满量程，"
                               "检查 ACS712 是不是串到母线上了")
        try:
            vals.append(int(body))
        except ValueError:
            raise RuntimeError("$CUR 回包无法解析：%r" % resp)
        time.sleep(gap)
    return _median(vals)


def grip_until_contact(pose5, start=None, max_jaw=GRIP_MAX_JAW,
                       rise_ma=GRIP_RISE_MA, hold_ma=GRIP_HOLD_MA,
                       preload=GRIP_PRELOAD, verbose=True):
    """从 start 往 max_jaw 分步合爪，读到「明显夹住」的电流就停。

    返回 (pulse, ma, base)；扫到 max_jaw 仍没接触则返回 None（夹空）。

    🔴 为什么不能等「堵转」才停：接触点到堵转之间电流是**连续爬升**的
    （11-16 实测 1500→1600 段约 1.2mA/µs），等到堵转时瓶身已被压变形、
    舵机也已经在发热。

    🔴 为什么用「两级判据」而不是一个阈值（11-16 实测逼出来的）：
    实测曲线（3.8cm 瓶）是 1450~1490 平在 32~37mA，1510 跳到 81，**1530 又
    回落到 59**，1550 才到 124。也就是说：
      · 「碰上了」这个信号只有 +49mA，跟 ±25mA 的毛刺同一量级 ⇒ 单靠它判停
        会停在 1510，而 1510 的开口约 4.2cm、根本夹不住 3.8cm 的瓶子；
      · 「夹得住」的信号是 +92mA 以上，量级拉开 ⇒ 拿它做停爪判据才可靠。
    所以：弱判据只用来记候选点，强判据才真停。两个都不到才判夹空。

    🔴 为什么分粗/细两段：远了可以大步走省时间；接近接触区必须小步，
    否则一步跨过接触点就直接吃过力。实测接触点在 1500~1510 ⇒ 粗段到
    1480 就转细段。（换更粗的物体时接触点会更早，粗段只是「还没到细区」，
    不影响判据本身。）

    pose5 = 前 5 路脉宽（J1~J5），全程保持不动；本函数**只改 J6**。
    返回 None 时调用方**绝不能抬臂**——那说明爪子是空的。
    """
    pose5 = tuple(int(p) for p in pose5)
    if len(pose5) != 5:
        raise ValueError("pose5 必须是 5 个脉宽（J1~J5），收到 %d 个" % len(pose5))

    real = int(read_state()[5])
    if start is None:
        start = real
    start = int(start)

    # 🔴 起点不许离当前位置太远：大跳变的启动电流会假冒「夹住」（见常量处注释）。
    if abs(start - real) > GRIP_MAX_FROM_JUMP:
        start = real + (GRIP_MAX_FROM_JUMP if start > real else -GRIP_MAX_FROM_JUMP)
        if verbose:
            print("  ⚠ 起点离当前 J6=%d 太远（单次跳变上限 %dµs）⇒ 改为从 %d 起扫"
                  % (real, GRIP_MAX_FROM_JUMP, start))
    if start != real:
        send_joint(list(pose5) + [start])
        time.sleep(GRIP_SETTLE_S)     # 等启动冲击过去，否则第一读数不可信

    base = read_current_ma()
    rise = base + int(rise_ma)
    hold = base + int(hold_ma)
    if verbose:
        print("  基线 %d mA（%d 次中位数）｜碰上 基线+%d｜夹住 基线+%d｜J6 %d → %d"
              % (base, CUR_SAMPLES, rise_ma, hold_ma, start, max_jaw))

    cand = None          # 首次「碰上了」的位置：不停爪，只记下来当备选
    last = (start, base)
    p = start
    while p <= max_jaw:
        send_joint(list(pose5) + [p])
        time.sleep(GRIP_DWELL_COARSE_S if p < GRIP_COARSE_UNTIL
                   else GRIP_DWELL_FINE_S)
        ma = read_current_ma()
        last = (p, ma)
        if verbose:
            print("    J6=%-4d → %+4d mA" % (p, ma - base))

        if ma >= GRIP_STALL_MA:
            back = max(start, p - 60)
            send_joint(list(pose5) + [back])
            raise RuntimeError("电流 %d mA 超兜底阈值 %d，已回退到 J6=%d"
                               % (ma, GRIP_STALL_MA, back))

        # 强判据：明显夹住了 ⇒ 就停在这里，不再往里合（再合只会压坏物体＋发热）
        if ma >= hold:
            if verbose:
                print("  ✅ 夹住：J6=%d，%d mA（基线+%d）" % (p, ma, ma - base))
            return p, ma, base

        # 弱判据：只是「碰上了」⇒ 记下候选点，继续等强判据
        # 为什么弱信号不停爪：实测 1510 处读到 81mA（基线+49），但 1530 又回落到
        # 59 —— 那一下只是爪指刚贴上瓶壁，力还远不够抬起来（抬得动的验证点是 1550）。
        if cand is None and ma >= rise:
            cand = p
            if verbose:
                print("    · 首次接触 J6=%d（基线+%d），继续压到「夹住」为止"
                      % (p, ma - base))

        p += GRIP_STEP_COARSE if p < GRIP_COARSE_UNTIL else GRIP_STEP_FINE

    # 扫完只有弱信号：在候选点后补一段固定预压量，绝不一路合到底
    if cand is not None:
        top = min(int(cand) + int(preload), max_jaw)
        send_joint(list(pose5) + [top])
        time.sleep(GRIP_DWELL_FINE_S)
        ma = read_current_ma()
        if verbose:
            print("  ✅ 夹住（弱信号+预压 %dµs）：J6=%d，%d mA（基线+%d）"
                  % (preload, top, ma, ma - base))
        return top, ma, base

    if verbose and last is not None:
        print("  ⚠ 扫到 J6=%d 仍未接触（最后 %d mA）⇒ 判「夹空」"
              % (last[0], last[1]))
    return None

    if verbose and last is not None:
        print("  ⚠ 扫到 J6=%d 仍未接触（最后 %d mA）⇒ 判「夹空」"
              % (last[0], last[1]))
    return None


def run_plan(plan, yes=False, dry_run=False, dynamic_grip=False):
    """依次执行计划里的每一步，每步都等到位再走下一步。

    dynamic_grip=True 时，「合爪」那一步不再发计划里的固定脉宽，而是调用
    grip_until_contact() 用电流判据当场决定夹持位置；判为夹空则**不抬臂**、
    返回 7。

    返回 0 = 全部到位；2 = 有步骤超时（已在 stderr 说明）；3 = 被限位警告拦下；
    4 = 通信失败；7 = 合爪判为夹空（爪里没有东西，停在原处）。
    """
    for i, step in enumerate(plan.steps, 1):
        print("[%d/%d] %s → %s" % (i, len(plan.steps), step.name,
                                   "/".join(str(p) for p in step.pulses)))
        if step.note:
            print("        %s" % step.note)
        if dynamic_grip and "合爪" in step.name:
            print("        （J6 由电流判据动态决定，上面的 %d 只是占位）"
                  % step.pulses[5])

    if plan.warnings:
        print("\n⚠ 警告：")
        for w in plan.warnings:
            print("  - " + w)

    if dry_run:
        print("\n[dry-run] 以上命令一条都没有发出去。")
        return 0

    if not yes:
        print("\n加 --yes 才会真的动臂（现在什么都没发）。")
        return 3

    for i, step in enumerate(plan.steps, 1):
        # 🔴 动态合爪：不发那条写死的脉宽，改用电流判据当场算夹持位置。
        #    为什么放在 send_joint 之前：一旦先发了 1550，爪子已经夹上去了，
        #    再"动态"就没意义了（而且可能已经过夹）。
        if dynamic_grip and "合爪" in step.name:
            try:
                got = grip_until_contact(step.pulses[:5])
            except RuntimeError as exc:
                print("❌ 第 %d 步「%s」合爪失败：%s"
                      % (i, step.name, exc), file=sys.stderr)
                print("   爪的状态未知，先跑一次 state 看位置，再决定手动松开还是重试。",
                      file=sys.stderr)
                return 4
            if got is None:
                print("❌ 第 %d 步「%s」判为夹空（电流全程没抬升）。"
                      % (i, step.name), file=sys.stderr)
                print("   **不要抬臂**：爪里没有东西，抬起来就是空抓。"
                      "先确认物体还在原位、有没有被推偏。", file=sys.stderr)
                return 7
            jaw, ma, base = got
            # 后续「保持夹紧」的步骤同步成实际夹持值；
            # 张爪步骤（J6=JAW_OPEN）不能动，否则会把东西半路丢掉。
            for later in plan.steps[i:]:
                if later.pulses[5] == step.pulses[5]:
                    later.pulses = tuple(later.pulses[:5]) + (jaw,)
            print("    ✓ 夹住 J6=%d（%d mA，基线+%d）" % (jaw, ma, ma - base))
            continue

        # 🔴 通信异常必须单独接住：串口中途掉线（CH341 被 USB 复位）时
        #    read_state() 会抛 RuntimeError。如果不接住，进程直接退出，
        #    而臂**正停在半路**，人看到的是「程序崩了」而不是「臂停在哪」。
        try:
            send_joint(step.pulses)
            actual, ok = wait_arrive(step.pulses)
        except RuntimeError as exc:
            print("❌ 第 %d 步「%s」通信失败：%s" % (i, step.name, exc), file=sys.stderr)
            print("   臂可能停在半路。先单独跑一次 `state` 看实际位置，"
                  "串口断了就先 POST /api/open。", file=sys.stderr)
            return 4

        # 1) 先确认前 5 路真的到位（爪步骤时这 5 路通常不变，会秒返回）
        if not ok:
            print("❌ 第 %d 步「%s」超时未到位：实际 %s / 目标 %s"
                  % (i, step.name, list(actual), list(step.pulses)),
                  file=sys.stderr)
            print("   臂停在半路。先 `state` 看一眼实际位置，再决定手动摆回还是重发。",
                  file=sys.stderr)
            return 2

        # 2) 🔴 爪步骤要单独等 J6：wait_arrive 故意跳过 J6（堵转永远到不了），
        #    但「合爪没合紧就抬起」会直接把物体甩掉，所以这里补一段
        #    按斜坡时间算的超时等待。到不了不算失败（夹住东西就该到不了）。
        if "爪" in step.name:
            # 🔴 这里必须单独接住通信异常：20:1x 实机就是在这一步崩的 ——
            #    send_joint/wait_arrive 在外层 try 里，但 wait_jaw 又会自己
            #    read_state()，那一下超时直接把整个进程崩掉，臂停在半路，
            #    人看到的是 traceback 而不是「第几步失败的」。
            try:
                jaw_got, jaw_ok = wait_jaw(step.pulses[5])
            except RuntimeError as exc:
                print("❌ 第 %d 步「%s」等爪时通信失败：%s"
                      % (i, step.name, exc), file=sys.stderr)
                print("   爪可能停在半路（物体也许已松、也许还夹着）。"
                      "先 POST /api/open 重连，再跑一次 state 看实际位置。",
                      file=sys.stderr)
                return 4
            if jaw_ok:
                print("    ✓ 爪到位 %d" % jaw_got)
            else:
                print("    · 爪停在 %d（目标 %d）——夹住物体时到不了目标值，属正常"
                      % (jaw_got, step.pulses[5]))
            time.sleep(SETTLE_AFTER_JAW_S)
            # 爪等完再读一次完整状态打印，否则打出来的是「爪还在动」时的中途值，
            # 看起来像没到位（假 Flask 测试里就出现过显示 1550 而目标 1900）。
            # 🔴 这次读取也必须接异常：它和 wait_jaw 一样是「臂停在半路时的
            #    追加读」，只是为了让日志好看，读挂了不值得把进程崩掉。
            try:
                print("    ✓ 到位 %s" % (list(read_state()),))
            except RuntimeError as exc:
                print("    ✓ 前五路到位 %s（补读完整状态失败：%s）"
                      % (list(actual), exc))
        else:
            print("    ✓ 到位 %s" % (list(actual),))

    print("\n✅ %d 步全部到位" % len(plan.steps))
    return 0


# ---------------------------------------------------------------------------
# IK 实例：延迟创建，避免 --self-test / state 也要求 arm_params.json 存在
# ---------------------------------------------------------------------------
_IK_CACHE = None


def _get_ik():
    """拿一个 ArmIK 实例（懒加载 + 缓存）。

    为什么不写在模块顶层：`state` 子命令只读脉宽，不需要 IK；
    顶层 import 会让它因为没有 arm_params.json 而直接崩掉。
    """
    global _IK_CACHE
    if _IK_CACHE is None:
        from arm_ik import ArmIK          # 同目录，用相对导入会要求它是包
        _IK_CACHE = ArmIK()
    return _IK_CACHE


# ---------------------------------------------------------------------------
# 自检：不联网、不动臂，只验纯函数
# ---------------------------------------------------------------------------
def self_test():
    """纯逻辑自检：验证计划结构、限位裁剪、步骤顺序。"""
    ik = _get_ik()

    # 1) pick 必须是 4 步，且 J6 先开、后合
    # 坐标选 (10, 18)：3D 距离约 21.5 cm < L1+L2 = 25.2 cm，在可达范围内。
    # 别用 (20,25)：水平距离就 32 cm，solve() 会直接抛「超出可达范围」。
    # ⚠️ 这里传 compensate=False：本节点验的是**计划结构**，不是几何。
    #    开补偿会把目标 z 抬 |SAG_DZ|=3.88cm，(10,18) 在 tilt=0 下 d 就变 25.4
    #    而越界 —— 那是几何事实（补偿会吃可达范围），不是 bug，seed 见第 7 项。
    plan = plan_pick(10.0, 18.0, compensate=False)
    assert len(plan.steps) == 4, len(plan.steps)
    names = [s.name for s in plan.steps]
    assert names[0] == "移到上方" and names[-1] == "抬起", names
    assert plan.steps[0].pulses[5] == JAW_OPEN, plan.steps[0].pulses
    assert plan.steps[-1].pulses[5] == JAW_CLOSE, plan.steps[-1].pulses
    # 前两步必须用同一个张开值（下降段爪不能合），后两步必须用闭合值
    assert plan.steps[1].pulses[5] == JAW_OPEN, plan.steps[1].pulses
    assert plan.steps[2].pulses[5] == JAW_CLOSE, plan.steps[2].pulses

    # 2) 每步都必须是 6 路，且都落在软限位内
    for s in plan.steps:
        assert len(s.pulses) == 6, s
        for i, p in enumerate(s.pulses):
            lo, hi = SAFE_LIMITS[i]
            assert lo <= p <= hi, (s.name, JOINT_NAMES[i], p, lo, hi)

    # 3) 高度必须变过：抬升点 ≠ 下降点
    #    用 J4 脉宽间接看高度变化（J4 是腕俯仰，高度不同它必然不同）
    zs = [s.pulses[3] for s in plan.steps]
    assert zs[0] != zs[1], "接近点和下降点应该不同（高度变了）"
    # 合爪点必须和下降点同高（不能边合边动）
    assert zs[1] == zs[2], (zs[1], zs[2])

    # 4) place 必须是 4 步，且 J6 先合后开
    p2 = plan_place(6.0, 20.0, compensate=False)
    assert len(p2.steps) == 4, len(p2.steps)
    assert p2.steps[0].pulses[5] == JAW_CLOSE, p2.steps[0].pulses
    assert p2.steps[-1].pulses[5] == JAW_OPEN, p2.steps[-1].pulses

    # 5) 限位裁剪必须生效且被报告。
    #    这里直接测 clip_pulses 而不是造一个「可达但超限位」的坐标 ——
    #    后者极难构造（越界的目标点通常先被 solve 以「不可达」拦掉），
    #    而 clip_pulses 才是真正干活的那一层，测它更直接。
    raw = (100, 9999, 2400, 2400, 1450, 3000)   # 越界的是 J1/J2/J4/J6（J3/J5 在范围内）
    clipped, names = clip_pulses(raw)
    assert clipped == (800, 2150, 2400, 1850, 1450, 2000), clipped
    assert names == ["J1", "J2", "J4", "J6"], names
    # 不越界的输入必须原样返回、且不报任何裁剪
    same, none_clipped = clip_pulses(HOME_PULSES)
    assert same == HOME_PULSES and none_clipped == [], (same, none_clipped)

    # 6) home 的 6 路必须与固件 INIT 一致
    h = plan_home().steps[0]
    assert h.pulses == HOME_PULSES, h.pulses

    # 7) 时间估算：走 1500 µs 应约 1.2 s
    est = estimate_travel_s((0, 0, 0, 0, 0, 0), (1500, 0, 0, 0, 0, 0))
    assert abs(est - 1.2) < 0.01, est

    # 8) 🔴 闭环自检：solve() → to_pulses() → fk_pulses() 必须回到原坐标。
    #    这一步能抓住「IK 和 FK 各写各的」—— 那种错在纸上永远看不出来，
    #    只有真把臂开过去才会发现差几厘米。
    #    实测：多数点 0.02~0.03 cm（脉宽 25 µs 量化极限）；
    #    J1 接近 0° 的点 0.71 cm（J1 map 伪折点，见 README 的 s=0.7549 一节）。
    #    ⇒ 阈值取 1.0 cm：放得过已知问题，又能抓住真正的错。
    try:
        from handeye_geometry import Kinematics
        kin = Kinematics(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                      "arm_params.json"), ik=ik)
        loop_worst = 0.0
        for (px, py) in ((10.0, 18.0), (6.0, 20.0), (15.0, 15.0), (5.0, 12.0)):
            ang = ik.solve(px, py, z=ik.grab_h)
            T = kin.fk_pulses(ik.to_pulses(ang))
            err = ((T[0][3] - px) ** 2 + (T[1][3] - py) ** 2
                   + (T[2][3] - ik.grab_h) ** 2) ** 0.5
            loop_worst = max(loop_worst, err)
            assert err < 1.0, "IK→FK 闭环误差 %.2f cm 超标 @ (%.0f,%.0f)" % (err, px, py)
    except ImportError:
        loop_worst = None
        print("（跳过第 8 组：没装 numpy，无法跑 IK→FK 闭环）")

    # 9) 🔴 执行侧补偿 + 姿态自适应（11-16 新增，这两项直接决定「换个位置还能不能抓」）
    #    ① 补偿必须真的改动脉宽，否则等于没生效（11-16 的坑就是压根没补）；
    #    ② best_tilt 必须返回「6 路都不越软限位」的解，且比默认 41° 更平。
    p_raw = plan_pick(-5.0, 25.0, tilt=41.0, compensate=False)
    p_cmp = plan_pick(-5.0, 25.0, tilt=41.0, compensate=True)
    assert p_raw.steps[1].pulses != p_cmp.steps[1].pulses, \
        "补偿没改变脉宽，等于没生效"
    got = best_tilt(-5.0, 25.0, 2.0)
    assert got is not None, "(-5, 25) 在 41~96° 内找不到可行姿态"
    bt, bp = got
    assert bt > 41.0, "best_tilt 应比默认 41° 更平，实际 %.1f°" % bt
    for i, v in enumerate(bp):
        lo, hi = SAFE_LIMITS[i]
        assert lo <= v <= hi, ("best_tilt 越限", JOINT_NAMES[i], v, lo, hi)
    best_tilt_deg = bt
    best_tilt_pulses = bp

    print("SELF_TEST PASS（9 组断言全过，未联网、未动臂）")
    if loop_worst is not None:
        print("  IK→FK 闭环最大误差 %.2f cm（阈值 1.0，J1 折点区是已知偏差）" % loop_worst)
    print("  best_tilt(-5, 25) = %.0f°（不越软限位的最平姿态）→ %s"
          % (best_tilt_deg, list(best_tilt_pulses)))
    print("\npick(10, 18) 的计划（compensate=False，纯结构演示）：")
    for s in plan.steps:
        print("  %-12s %s" % (s.name, s.pulses))
    return 0


# ---------------------------------------------------------------------------
# 命令行外壳
# ---------------------------------------------------------------------------
def main():
    # 🔴 global 必须在函数体第一句之前声明：Python 只要看到函数里出现
    # global X，就整个函数都把 X 当全局变量；而一旦在声明之前读取它，
    # 就会报「name is used prior to global declaration」（这里踩过）。
    global BASE_URL

    ap = argparse.ArgumentParser(
        description="动作原语层：pick / place / home（每步都回读关节角确认）")
    ap.add_argument("--self-test", action="store_true", help="纯逻辑自检，不联网不动臂")
    ap.add_argument("--dry-run", action="store_true", help="只打印计划，不发命令")
    ap.add_argument("--yes", action="store_true", help="确认真的动臂（不加只打印）")
    ap.add_argument("--hover", type=float, default=4.0, help="上方接近高度 cm，默认 4")
    ap.add_argument("--z", type=float, default=None, help="抓取高度 cm，默认 grab_height")
    ap.add_argument("--tilt", type=float, default=None,
                    help="夹爪前倾角(度)。不给就从当前 STATE 脉宽自动反算"
                         "（C 姿态=41.00°）；显式给则用它覆盖。"
                         "写死 0 会让远处目标误报「超出可达范围」")
    ap.add_argument("--auto-tilt", action="store_true",
                    help="对抓取点自动扫「最接近水平且不越软限位」的 tilt（换位置抓取推荐）")
    ap.add_argument("--no-compensate", action="store_true",
                    help="不扣重力偏置 TIP_BIAS_XY/SAG_DZ（只用于复现旧行为，正常别加）")
    ap.add_argument("--list-recipes", action="store_true",
                    help="列出 pick_recipes.json 里的所有示教配方")
    ap.add_argument("--j1-offset", type=int, default=0,
                    help="J1 附加偏移 µs（1°≈9.4µs；符号未定，先用 ±20 小步试方向）")
    ap.add_argument("--base-url", default=BASE_URL, help="Flask 地址")

    sub = ap.add_subparsers(dest="cmd")
    p_state = sub.add_parser("state", help="读当前 6 路脉宽")
    p_home = sub.add_parser("home", help="回初始位")
    p_recipe = sub.add_parser("recipe", help="回放示教配方（不走 IK，最可靠）")
    p_recipe.add_argument("name", help="配方名，见 --list-recipes")
    p_grip = sub.add_parser("grip", help="电流引导分步合爪：夹到接触就停")
    p_grip.add_argument("--from", dest="from_pulse", type=int, default=None,
                        help="起始 J6 脉宽，默认当前实际值；离当前位置超过 %dµs 会被"
                             "夹回（大跳变的启动冲击会假冒「夹住」）"
                             % GRIP_MAX_FROM_JUMP)
    p_grip.add_argument("--max", dest="max_jaw", type=int, default=GRIP_MAX_JAW,
                        help="扫到这里仍未接触就判夹空，默认 %d" % GRIP_MAX_JAW)
    p_grip.add_argument("--hold", dest="hold_ma", type=int, default=GRIP_HOLD_MA,
                        help="「夹住」判据 mA（相对基线），默认 %d" % GRIP_HOLD_MA)
    p_grip.add_argument("--rise", dest="rise_ma", type=int, default=GRIP_RISE_MA,
                        help="「碰上」判据 mA，只记候选点不停爪，默认 %d" % GRIP_RISE_MA)
    p_grip.add_argument("--lift", dest="lift_dj2", type=int, default=0,
                        help="夹住后把 J2 抬多少 µs（+60 ≈ 示教悬停增量），0=不抬，默认 0")
    for nm in ("pick", "place"):
        p = sub.add_parser(nm, help="在 (x,y) 处%s" % ("抓取" if nm == "pick" else "放置"))
        p.add_argument("x", type=float, help="工作台 x，cm")
        p.add_argument("y", type=float, help="工作台 y，cm")

    args = ap.parse_args()

    BASE_URL = args.base_url.rstrip("/")

    if args.self_test:
        return self_test()

    # --list-recipes 要放在「cmd is None 就打 help」之前，否则单独用它会被拦掉
    if args.list_recipes:
        try:
            data = load_recipes()
        except RuntimeError as exc:
            print("❌ %s" % exc, file=sys.stderr)
            return 5
        print("配方库：%s" % RECIPES_PATH)
        jaw = data.get("jaw") or {}
        if jaw:
            print("夹爪标定：全张 %s（%s cm）｜几何贴合 %s（%s cm）｜默认抓取 %s µs"
                  % (jaw.get("open_full_pulses"), jaw.get("open_full_cm"),
                     jaw.get("grip_ref_pulses"), jaw.get("grip_ref_cm"),
                     jaw.get("default_grip_pulses")))
        for nm in sorted(data["recipes"]):
            r = data["recipes"][nm]
            print("  · %-16s %s" % (nm, r.get("desc", "")))
            print("      验证：%s" % r.get("verified", "无（未在实机验证过，慎用）"))
        return 0

    if args.cmd is None:
        ap.print_help()
        return 1

    if args.cmd == "state":
        pulses = read_state()
        print("当前脉宽：" + "  ".join(
            "%s=%d" % (JOINT_NAMES[i], p) for i, p in enumerate(pulses)))
        return 0

    if args.cmd == "grip":
        # 🔴 先读一次真实姿态：J1~J5 全程保持不动，只动 J6。
        #    如果臂还没摆到抓取位（比如还在初始位），grip 强行合爪是没意义的，
        #    所以把当前脉宽打出来让人核对，而不是悄悄按「以为的位置」去动。
        try:
            pulses = read_state()
        except RuntimeError as exc:
            print("❌ %s" % exc, file=sys.stderr)
            return 4
        print("  当前脉宽 %s" % (list(pulses),))

        if args.dry_run:
            base = read_current_ma()
            print("  基线电流 %d mA（只读，不动臂）" % base)
            print("  判据：碰上 基线+%d mA（不停爪）／夹住 基线+%d mA（停爪）"
                  % (args.rise_ma, args.hold_ma))
            print("  J6 %s → %d｜粗步 %dµs / 细步 %dµs"
                  % (args.from_pulse or pulses[5], args.max_jaw,
                     GRIP_STEP_COARSE, GRIP_STEP_FINE))
            print("\n[dry-run] 一条动作命令都没有发出去。")
            return 0

        if not args.yes:
            print("\n加 --yes 才会真的合爪（现在什么都没发）。")
            return 3

        try:
            got = grip_until_contact(pulses[:5], start=args.from_pulse,
                                     max_jaw=args.max_jaw,
                                     rise_ma=args.rise_ma,
                                     hold_ma=args.hold_ma)
        except RuntimeError as exc:
            print("❌ 合爪中止：%s" % exc, file=sys.stderr)
            return 4

        if got is None:
            print("❌ 没夹到东西（电流全程没抬升）。**别抬臂**——"
                  "先确认物体还在爪间、位置有没有偏。", file=sys.stderr)
            return 6

        jaw, ma, base = got
        print("\n✅ 停在 J6=%d（基线 %d → %d mA，+%d）"
              % (jaw, base, ma, ma - base))

        def _curl(pulses6):
            """把一条 JOINT 直接渲染成可粘贴的 curl。

            🔴 为什么要在工具里生成：19:1x 我在聊天里写了
            `JOINT,...,<上一步打印的J6>` 这种带占位符的命令，用户照贴 ⇒
            中文字节进了 payload ⇒ Flask 回 `'ascii' codec can't encode`。
            数字由程序填好，就不会再有占位符这回事。
            """
            body = ",".join(str(int(v)) for v in pulses6)
            return ("  curl -s -X POST %s/api/cmd -H \"Content-Type: application/json\" "
                    "-d '{\"payload\":\"JOINT,%s\"}'; echo" % (BASE_URL, body))

        pose5 = list(pulses[:5])
        if args.lift_dj2:
            up = list(pose5)
            up[1] += int(args.lift_dj2)
            up_c, cut = clip_pulses(up + [jaw])
            try:
                send_joint(up_c)
                actual, arrived = wait_arrive(up_c)
            except RuntimeError as exc:
                print("❌ 抬起失败：%s" % exc, file=sys.stderr)
                print("   🔴 爪还夹着东西，别手动松爪、也别再发命令；"
                      "先跑一次 state 看实际位置。", file=sys.stderr)
                return 4
            print("  抬起：J2 %+dµs → %s" % (args.lift_dj2, list(up_c)))
            if cut:
                print("  ⚠ 抬起目标越界已裁 %s ⇒ 实际抬升不到 %dµs"
                      % ("/".join(cut), args.lift_dj2))
            print("  实际 %s%s" % (list(actual),
                                 "" if arrived else "（未全部到位，先看一眼再动手）"))
            print("  可粘贴放回（两步，别一次到位——半空张爪会掉一段）：")
            print("   ① 降回抓取位（仍夹紧）：")
            print(_curl(pose5 + [jaw]))
            print("   ② 张爪放下：")
            print(_curl(pose5 + [JAW_OPEN]))
        else:
            sug = list(pose5)
            sug[1] += 60
            sug_c, cut = clip_pulses(sug + [jaw])
            print("  两条可直接粘贴（数字已填好，不用改）：")
            print("   ① 抬起（J2 +60µs ≈ 示教悬停增量）：")
            print(_curl(sug_c))
            if cut:
                print("      ⚠ J2 +60 越界已裁 %s" % "/".join(cut))
            print("   ② 放回（两步，别一次到位——半空张爪会掉一段）：")
            print(_curl(pose5 + [jaw]))
            print(_curl(pose5 + [JAW_OPEN]))
        return 0

    if args.cmd == "home":
        return run_plan(plan_home(), yes=args.yes, dry_run=args.dry_run)

    if args.cmd == "recipe":
        try:
            data = load_recipes()
        except RuntimeError as exc:
            print("❌ %s" % exc, file=sys.stderr)
            return 5
        rec = data["recipes"].get(args.name)
        if not rec:
            print("❌ 没有配方 %r；可用：%s"
                  % (args.name, ", ".join(sorted(data["recipes"]))), file=sys.stderr)
            return 5
        print("配方 %s —— %s" % (args.name, rec.get("desc", "")))
        print("  验证记录：%s" % rec.get("verified", "无（未在实机验证过，慎用）"))
        if rec.get("verified_boundary"):
            print("  适用边界：%s" % rec["verified_boundary"])
        try:
            plan = build_recipe_plan(rec, j1_offset=args.j1_offset)
        except RuntimeError as exc:
            print("❌ %s" % exc, file=sys.stderr)
            return 5
        return run_plan(plan, yes=args.yes, dry_run=args.dry_run)

    # ---- tilt：不显式给就从当前真实脉宽反算（与拍照/定位那一刻的姿态一致）----
    # 🔴 为什么不能默认写死 0：solve() 里 r_w = r − L3·sin(tilt)，
    #    C 姿态(41°) 下用 0 会让 r_w 少扣约 11cm，远处目标被误判「超出可达范围」。
    tilt = args.tilt
    if tilt is None and not args.auto_tilt:
        try:
            _cur = read_state()
            tilt = _get_ik().tilt_from_pulses(_cur)
            print("  tilt 自动反算：%.2f°（当前脉宽 %s）"
                  % (tilt, ",".join(str(p) for p in _cur)))
        except Exception as e:
            tilt = 0.0
            print("  ⚠ 读不到当前姿态，tilt 退回 0（远目标可能误报不可达）：%s" % e)
    if tilt is None:
        tilt = 0.0          # --auto-tilt 时用不到它，给个占位满足签名

    try:
        if args.cmd == "pick":
            plan = plan_pick(args.x, args.y, z=args.z, hover=args.hover,
                             tilt=tilt, auto_tilt=args.auto_tilt,
                             compensate=not args.no_compensate)
        else:
            plan = plan_place(args.x, args.y, z=args.z, hover=args.hover,
                              tilt=tilt, auto_tilt=args.auto_tilt,
                              compensate=not args.no_compensate)
    except ValueError as exc:
        print("❌ %s" % exc, file=sys.stderr)
        return 5
    return run_plan(plan, yes=args.yes, dry_run=args.dry_run)


if __name__ == "__main__":
    sys.exit(main())
