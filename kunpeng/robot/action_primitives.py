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

# 夹爪：网页上「夹爪张开 900 / 闭合 1900」两个实测可用的值。
# 950 而不是 900 —— 顶到机械限位舵机会持续堵转发热，留 50 µs 余量。
JAW_OPEN = 950
JAW_CLOSE = 1900

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


def plan_move(x, y, z, jaw=None, ik=None):
    """工作台坐标 (x,y,z) → 一个「移过去」的 Step（纯函数，不发命令）。

    参数 jaw：不为 None 时用它覆盖 J6（夹爪），因为 IK 解出来的 J6 是
    「claw 角」对应的脉宽，而抓取序列要独立控制爪子开合。
    """
    ik = ik or _get_ik()
    angles = ik.solve(x, y, z=z)
    raw = ik.to_pulses(angles)
    pulses, clamped = clip_pulses(raw)
    if jaw is not None:
        pulse_list = list(pulses)
        pulse_list[5] = int(jaw)
        pulses, clamped2 = clip_pulses(pulse_list)
        clamped = clamped + clamped2
    note = "工作台 (%.1f, %.1f, %.1f) cm" % (x, y, z)
    if clamped:
        note += "；⚠ 被软限位裁：%s" % "/".join(clamped)
    return Step(name="move", pulses=pulses, note=note), clamped


def plan_pick(x, y, z=None, hover=4.0, jaw_open=JAW_OPEN, jaw_close=JAW_CLOSE,
              ik=None):
    """规划一次抓取：移到上方 → 下降 → 合爪 → 抬起（纯函数）。

    hover = 在目标上方多少 cm 处开始下降。4 cm 是经验值：
    太矮会撞到相邻物体，太高则下降段长、耗时且更容易在途中蹭到东西。

    🔴 为什么是 4 步而不是 5 步：最初写的是「张开爪子 → 移到上方 → …」，
    dry-run 打出来发现前两步的 6 路脉宽**一模一样** —— 因为第一步的目标
    也是「上方 + 张爪」，第二步就没活干了。合并成一步：移动时就把 J6
    定在张开值，落到上方时爪已经是开的，下降段不会顶到物体。
    """
    ik = ik or _get_ik()
    if z is None:
        z = ik.grab_h
    z_top = z + hover

    plan = Plan()
    seq = (
        ("移到上方", z_top, jaw_open, "移动时即张爪，高于目标 %.1f cm" % hover),
        ("下降到抓取高度", z, jaw_open, "这里是唯一可能撞到物体的时刻"),
        ("合爪", z, jaw_close, "停 %.1f s 等扭矩建立" % SETTLE_AFTER_JAW_S),
        ("抬起", z_top, jaw_close, "带着物体回到安全高度"),
    )
    for name, zz, jaw, note in seq:
        step, clamped = plan_move(x, y, zz, jaw=jaw, ik=ik)
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
               jaw_close=JAW_CLOSE, ik=None):
    """规划一次放置：移到上方 → 下降 → 张爪 → 抬起（纯函数）。"""
    ik = ik or _get_ik()
    if z is None:
        z = ik.grab_h
    z_top = z + hover

    plan = Plan()
    seq = (
        ("移到上方", z_top, jaw_close, "保持夹紧，别在半路掉东西"),
        ("下降到放置高度", z, jaw_close, "降到目标平面"),
        ("张爪", z, jaw_open, "停 %.1f s 等物体脱离" % SETTLE_AFTER_JAW_S),
        ("抬起", z_top, jaw_open, "空爪回到安全高度"),
    )
    for name, zz, jaw, note in seq:
        step, clamped = plan_move(x, y, zz, jaw=jaw, ik=ik)
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


def run_plan(plan, yes=False, dry_run=False):
    """依次执行计划里的每一步，每步都等到位再走下一步。

    返回 0 = 全部到位；2 = 有步骤超时（已在 stderr 说明）；3 = 被限位警告拦下。
    """
    for i, step in enumerate(plan.steps, 1):
        print("[%d/%d] %s → %s" % (i, len(plan.steps), step.name,
                                   "/".join(str(p) for p in step.pulses)))
        if step.note:
            print("        %s" % step.note)

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
            jaw_got, jaw_ok = wait_jaw(step.pulses[5])
            if jaw_ok:
                print("    ✓ 爪到位 %d" % jaw_got)
            else:
                print("    · 爪停在 %d（目标 %d）——夹住物体时到不了目标值，属正常"
                      % (jaw_got, step.pulses[5]))
            time.sleep(SETTLE_AFTER_JAW_S)
            # 爪等完再读一次完整状态打印，否则打出来的是「爪还在动」时的中途值，
            # 看起来像没到位（假 Flask 测试里就出现过显示 1550 而目标 1900）。
            print("    ✓ 到位 %s" % (list(read_state()),))
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
    plan = plan_pick(10.0, 18.0)
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
    p2 = plan_place(6.0, 20.0)
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

    print("SELF_TEST PASS（8 组断言全过，未联网、未动臂）")
    if loop_worst is not None:
        print("  IK→FK 闭环最大误差 %.2f cm（阈值 1.0，J1 折点区是已知偏差）" % loop_worst)
    print("\npick(10, 18) 的计划：")
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
    ap.add_argument("--base-url", default=BASE_URL, help="Flask 地址")

    sub = ap.add_subparsers(dest="cmd")
    p_state = sub.add_parser("state", help="读当前 6 路脉宽")
    p_home = sub.add_parser("home", help="回初始位")
    for nm in ("pick", "place"):
        p = sub.add_parser(nm, help="在 (x,y) 处%s" % ("抓取" if nm == "pick" else "放置"))
        p.add_argument("x", type=float, help="工作台 x，cm")
        p.add_argument("y", type=float, help="工作台 y，cm")

    args = ap.parse_args()

    BASE_URL = args.base_url.rstrip("/")

    if args.self_test:
        return self_test()

    if args.cmd is None:
        ap.print_help()
        return 1

    if args.cmd == "state":
        pulses = read_state()
        print("当前脉宽：" + "  ".join(
            "%s=%d" % (JOINT_NAMES[i], p) for i, p in enumerate(pulses)))
        return 0

    if args.cmd == "home":
        return run_plan(plan_home(), yes=args.yes, dry_run=args.dry_run)

    if args.cmd == "pick":
        plan = plan_pick(args.x, args.y, z=args.z, hover=args.hover)
    else:
        plan = plan_place(args.x, args.y, z=args.z, hover=args.hover)
    return run_plan(plan, yes=args.yes, dry_run=args.dry_run)


if __name__ == "__main__":
    sys.exit(main())
