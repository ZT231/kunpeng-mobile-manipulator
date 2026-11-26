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
  3) 到位判据靠 $ARM,READ 回读位置（总线舵机有位置闭环）。夹到东西后 J6 可能
     到不了目标值（堵转），所以 J6 不参与到位判据。
  4) 没有力控、没有碰撞检测。下降高度靠调用方给对 z，给低了会撞桌子。

为什么单独一个文件，而不是塞进 web_control.py：
  和 intent_parser.py 同一个道理 —— 5000 网页服务是已实机验证过的主链路，
  改它就要 scp + 重启 Flask，重启会清掉 _IK_MAP 标定值、打断现场。
  这里只通过 HTTP 调它已有的 /api/cmd 与 /api/state，主链路一行不碰，
  删掉这个文件就是完整回滚。

用法（全部零第三方依赖，只要标准库 + 同目录 arm_ik.py）：
  python3 action_primitives.py --self-test            # 纯逻辑自检，不联网不动臂
  python3 action_primitives.py --dry-run pick 20 25   # 只算位置并打印，不发命令
  python3 action_primitives.py state                  # 读当前 6 路位置
  python3 action_primitives.py home                   # 回初始位
  python3 action_primitives.py pick 20 25 --z 2 --yes  # 真的抓（z 显式给：深度相机或 --z）
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

# 🔴 总线舵机软限位（位置 0~1000）。J6 已实测（300~674）；J1~J5 暂用驱动安全限位
#    10~990 占位，装机标定后按机械行程收紧，并同步 servo_bus.h 与 web_control.py 的 POS_RANGES。
SAFE_LIMITS = (
    (10, 990),   # J1 基座回转（待标定）
    (10, 990),   # J2 大臂（待标定）
    (10, 990),   # J3 小臂（待标定）
    (10, 990),   # J4 腕俯仰（待标定）
    (10, 990),   # J5 腕回转（待标定）
    (300, 674),  # J6 夹爪（2026-11-26 实测：全张=300，全闭=674）
)

# 初始位 = 六轴零位（pos_zero，2026-11-26 实测：整臂竖直矗立、面朝正前、J6 闭合）
HOME_POSITIONS = (460, 514, 516, 502, 494, 674)

# J6 夹爪工作点（总线舵机，位置 0~1000）：
#   JAW_OPEN=300 全张；JAW_CLOSED_MAX=674 全闭（软限位）。
#   ⚠️ 旧 PWM 臂的「固定抓取闭合点 1550（≈3.8cm 瓶）」已作废：总线柔性夹爪按物体宽度
#   动态决定闭合点（$ARM,J,6,<目标> 走位置堵转自适应），不再预设固定 JAW_CLOSE。
JAW_OPEN = 300
JAW_CLOSE = 674          # 仅作「默认全闭」占位；真实抓取闭合点由调用方按物体宽度给
JAW_CLOSED_MAX = 674     # 完全闭合（软限位上限）

# 到位判据：6 路位置与目标的差都 ≤ 这个值就算到了（8 位置单位 ≈ 2°）。
# 🔴 旧 ARRIVE_TOL_US 的「µs 容差」已废弃：这里保留变量名仅避免大改引用，语义已是位置单位。
ARRIVE_TOL_US = 8

# 开合爪到位后再停一下，让舵机扭矩建立/释放。
# 有 wait_jaw 兜着，这里只需很短 —— 0.6 s 是 wait_jaw 之前留下的旧值，
# 当时靠它硬等，现在等到位已经是精确的，0.2 s 够扭矩稳住。
SETTLE_AFTER_JAW_S = 0.2


# ---------------------------------------------------------------------------
# 数据结构：把「一步动作」写成一个带名字的袋子
# ---------------------------------------------------------------------------
@dataclasses.dataclass
class Step:
    """一个原子步骤：名字 + 目标 6 路位置 + 人话说明。

    为什么用 dataclass 而不是裸 tuple：plan_pick() 返回一串步骤，
    dry-run 时要逐条打印给人看，字段有名字才读得懂；
    写错字段名也会立刻 AttributeError，不会静默出错。
    """
    name: str
    positions: tuple
    note: str = ""

    def as_dict(self):
        return {"name": self.name, "positions": list(self.positions), "note": self.note}


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
    """读当前 6 路实际位置，返回 tuple[int x6]。

    🔴 这是全文件唯一的「真实状态来源」。任何一步动作之后都必须靠它确认，
    不能假设「发下去了就是到位了」—— 那正是开环控制的经典谎报。
    """
    got = _post_json("/api/state", {}, timeout=timeout)
    if not got.get("ok"):
        raise RuntimeError("读状态失败：%s" % got.get("error", got))
    positions = got.get("positions")
    if not (isinstance(positions, list) and len(positions) == 6):
        raise RuntimeError("状态格式异常：%r" % (positions,))
    return tuple(int(p) for p in positions)


def send_joint(target, timeout=5.0):
    """发一条 $ARM,POS（6 路位置）并检查固件回执是否为 OK。

    🔴 为什么必须看 resp：/api/cmd 的 ok:true 只代表「串口写进去了」，
    固件真正执行结果在 resp 里（OK / ERR,2 未知命令 / ERR,3 参数不合法）。
    只判 ok 不看 resp，就会把「帧格式写错」当成「执行成功」。
    """
    payload = "ARM,POS," + ",".join(str(int(p)) for p in target)
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
def clip_positions(raw, limits=SAFE_LIMITS):
    """把 6 路总线位置裁到软限位内，返回 (裁剪后的 tuple, 被裁的关节名列表)。

    🔴 为什么不交给固件自动裁：固件侧裁剪是**静默**的，
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


def jaw_for_width(width_cm, grip_ref_positions=674, ref_cm=0.0,
                  pos_per_cm=None, margin_pos=10):
    """按物体宽度估 J6 夹持位置（两个实测点线性插值）。

    🔴 总线舵机下此标定失效、待实机重录：旧 PWM 臂标定是
      J6=950↔开口 6.8cm、J6=1500↔约 3.8cm、斜率 185 µs/cm，
      与总线位置 300（全张）~674（全闭）无对应关系。
    方向一致：J6 位置越大＝越闭合。
    重录后调用：jaw_for_width(width, grip_ref_positions=<闭合点位置>,
                              ref_cm=<闭合点开口 cm>, pos_per_cm=<位置/cm>)
    """
    if pos_per_cm is None:
        raise NotImplementedError("总线舵机 J6 开口标定未完成，需实机量两点后重录")
    return int(round(grip_ref_positions + margin_pos
                     + (float(ref_cm) - float(width_cm)) * pos_per_cm))


def best_tilt(x, y, z, ik=None, prefer=96.0, floor=41.0, step=2.0,
              j3_margin=60):
    """给一个目标点，找「最接近水平、且 6 路都不越软限位」的 tilt。

    为什么需要它：抓竖直圆柱时爪指越接近水平越稳，但 J4 有软限位。
    🔴 旧 PWM 臂实测（tilt=41→J4=1230µs、75→1851µs、96→2257µs）已作废，
    总线舵机 J4 软限位按 SAFE_LIMITS 重新扫描，不能照搬旧值。
    所以「最平可用姿态」是随目标点变的，必须每个点重扫，不能写死。

    参数 j3_margin：J3 要留多少 位置单位 余量（默认 60）。🔴 为什么要留：
      旧 PWM 臂曾因不留余量导致 J3 顶死软限位（旧值 2500/2320µs，已作废）。
      顶在软限位上虽然「不算越界」，但固件走到限位就走不动了，实际末端会偏，
      而且长时间顶住会堵转发热。留 60位置单位 换一个略斜一点的姿态，比顶死划算。

    返回 (tilt, positions)；实在没有可行解则返回 None。
    🔴 扫描从大到小：先试最平的，不行才往斜了退，保证「尽可能水平」。
    """
    ik = ik or _get_ik()
    tx, ty, tz = float(x), float(y), float(z)
    j3_hi = SAFE_LIMITS[2][1] - int(j3_margin)
    t = float(prefer)
    while t >= float(floor) - 1e-9:
        try:
            ang = ik.solve(tx, ty, z=tz, tilt=t)
            positions, clamped = clip_positions(ik.to_positions(ang))
        except Exception:
            t -= step          # 这个倾角解不出来（不可达 / 数值问题），退一档再试
            continue
        # 6 路都在软限位内、且 J3 还留了余量 —— 就是它
        if not clamped and positions[2] <= j3_hi:
            return float(t), tuple(int(round(v)) for v in positions)
        t -= step
    return None


def plan_move(x, y, z, jaw=None, ik=None, tilt=0.0):
    """工作台坐标 (x,y,z) → 一个「移过去」的 Step（纯函数，不发命令）。

    参数 jaw：不为 None 时用它覆盖 J6（夹爪），因为 IK 解出来的 J6 是
    「claw 角」对应的位置，而抓取序列要独立控制爪子开合。

    参数 tilt：夹爪前倾角(度)。🔴 必须与「拍照那一刻」的真实姿态一致：
      solve() 里 r_w = r − L3·sin(tilt)，用错 tilt 会让可达判定与落点都偏。
      tilt=0（默认）只适用于真的垂直抓取；眼在手链路应由调用方用
      `ik.tilt_from_positions(positions)` 从当前位置反算后传入。
      历史坑：默认 0 曾让 24cm 的目标报「超出可达范围 d=28.0」。
    """
    ik = ik or _get_ik()
    tx, ty, tz = float(x), float(y), float(z)
    angles = ik.solve(tx, ty, z=tz, tilt=tilt)
    raw = ik.to_positions(angles)
    positions, clamped = clip_positions(raw)
    if jaw is not None:
        position_list = list(positions)
        position_list[5] = int(jaw)
        positions, clamped2 = clip_positions(position_list)
        clamped = clamped + clamped2
    note = "工作台 (%.1f, %.1f, %.1f) cm" % (x, y, z)
    if clamped:
        note += "；⚠ 被软限位裁：%s" % "/".join(clamped)
    return Step(name="move", positions=positions, note=note), clamped


def plan_pick(x, y, z=None, hover=4.0, jaw_open=JAW_OPEN, jaw_close=JAW_CLOSE,
              ik=None, tilt=0.0, auto_tilt=False):
    """规划一次抓取：移到上方 → 下降 → 合爪 → 抬起（纯函数）。

    hover = 在目标上方多少 cm 处开始下降。4 cm 是经验值：
    太矮会撞到相邻物体，太高则下降段长、耗时且更容易在途中蹭到东西。

    🔴 为什么是 4 步而不是 5 步：最初写的是「张开爪子 → 移到上方 → …」，
    dry-run 打出来发现前两步的 6 路位置**一模一样** —— 因为第一步的目标
    也是「上方 + 张爪」，第二步就没活干了。合并成一步：移动时就把 J6
    定在张开值，落到上方时爪已经是开的，下降段不会顶到物体。

    参数 auto_tilt：True 时不用调用方给的 tilt，而是对**抓取点**扫出
      「最接近水平且不越软限位」的那个（见 best_tilt）。这是「物体放到
      别的点也能抓」的关键 —— 写死 41° 爪指斜着夹不稳，写死 96° 又 J4 超限。
    """
    ik = ik or _get_ik()
    if z is None:
        raise ValueError("需要显式 z（抓取高度 cm）；已取消 grab_height 默认值")
    if auto_tilt:
        got = best_tilt(x, y, z, ik=ik)
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
        step, clamped = plan_move(x, y, zz, jaw=jaw, ik=ik, tilt=tilt)
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
               auto_tilt=False):
    """规划一次放置：移到上方 → 下降 → 张爪 → 抬起（纯函数）。

    tilt / auto_tilt 语义同 plan_pick：前倾角要么显式给、
    要么对目标点扫出「最平可用」的那个。
    """
    ik = ik or _get_ik()
    if z is None:
        raise ValueError("需要显式 z（抓取高度 cm）；已取消 grab_height 默认值")
    if auto_tilt:
        got = best_tilt(x, y, z, ik=ik)
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
        step, clamped = plan_move(x, y, zz, jaw=jaw, ik=ik, tilt=tilt)
        step.name = name
        step.note = note + "｜" + step.note
        plan.steps.append(step)
        if clamped:
            plan.warnings.append("%s：目标越界，已裁 %s" % (name, "/".join(clamped)))
    return plan


def plan_home():
    """回初始位（纯函数，不读硬件）。"""
    return Plan(steps=[Step(name="回初始位", positions=tuple(HOME_POSITIONS),
                            note="6 路拉回 Arm_Init() 的 INIT 值")])


# ---------------------------------------------------------------------------
# 执行层：这里才开始真的动臂
# ---------------------------------------------------------------------------
def wait_arrive(target, timeout=None, tol=ARRIVE_TOL_US, check_j6=False,
                poll=0.1):
    """轮询 /api/state 直到 6 路位置都接近 target，返回实际读数。

    🔴 check_j6=False 是有意为之：夹爪夹住物体后舵机堵转，
    位置可能永远到不了全闭 674。若把 J6 计入判据，
    每次成功抓取都会被判成「超时失败」。

    返回 (实际位置 tuple, 是否在超时前到位)。
    🔴 超时**不抛异常**，返回 False 让调用方自己决定（比如继续往下走、
    还是中止整段动作）。抓取途中抛异常会把臂留在半路，比继续更危险。
    """
    if timeout is None:
        # 总线舵机运动时间由 $ARM 命令的 ms 参数决定（默认 800ms），
        # 这里只给一个宽松的兜底超时，实际到位与否由回读位置判定。
        timeout = 10.0

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
    """等夹爪走到目标位置，返回 (实际位置, 是否接近目标)。

    🔴 这个函数**不判失败**，因为到不了是正常的：夹住物体后舵机堵转，
    J6 会停在半路（比如目标全闭 674 实际停在半路）。所以它的职责只是
    「确保爪子真的动过、且给了它足够时间」，而不是精确校验到位。

    容差 60 位置单位 比关节的 8 位置单位 宽得多，同理 —— 夹爪本来就不该被当成
    精密定位轴来要求。

    🔴 为什么必须有它：J6 从全张 300 合到全闭 674 有一段行程，
    运动时间由 $ARM 命令的 ms 参数决定。如果只靠 wait_arrive（跳过 J6）
    加一个固定 sleep，sleep 短了就会「爪还没夹紧就抬走」，物体直接滑掉。
    """
    src = read_state()
    if timeout is None:
        # J6 运动时间由 $ARM 命令的 ms 参数决定，这里给兜底超时。
        timeout = 5.0
    deadline = time.time() + timeout
    last = src[5]
    while time.time() < deadline:
        last = read_state()[5]
        if abs(last - int(target)) <= tol:
            return last, True
        time.sleep(poll)
    return last, False


# ---------------------------------------------------------------------------
# 电压引导的分步合爪（接触即停）—— 是「接触检测」，不是「力控」
# ---------------------------------------------------------------------------
# 🔴 总线舵机迁移：旧 ACS712 电流判据（$CUR，mA）已随固件 $CUR 命令一起删除。
#    总线舵机用 $ARM,VIN 回读六路电压（mV），J6 抓取时电压随机械负载抬升，
#    判据从「电流 mA」改为「电压 mV」。⚠️ 以下阈值是旧 mA 占位，必须实机标定：
#     泄力后手掰 J6 全张→全闭，读 $ARM,VIN 的 J6 电压曲线，找「空载基线/接触/
#     夹住/堵转」的 mV 台阶，再回填 GRIP_*_MV。标定完成前 grip_until_contact 会
#     直接抛 NotImplementedError，避免用未标定阈值误判。
CUR_SAMPLES          = 3      # 每次判定读几遍取中位数（接触后单次读数有毛刺）
GRIP_STEP_COARSE     = 40     # 粗段步长 位置单位：离接触还远，走快点省时间
GRIP_STEP_FINE       = 15     # 细段步长 位置单位：接近接触区必须小步，别一跨就过
GRIP_COARSE_UNTIL    = 674    # 🔴 待标定：总线 J6 全闭=674，粗段终点标定后再设
GRIP_RISE_MV         = None   # 🔴 待标定：「碰上了」弱判据 mV（只记候选点，不停爪）
GRIP_HOLD_MV         = None   # 🔴 待标定：「夹住了」强判据 mV（到这儿才真停）
GRIP_PRELOAD         = 40     # 备选：只出现弱信号、始终没到强判据时，再压这么多位置单位
GRIP_STALL_MV        = None   # 🔴 待标定：兜底堵转电压 mV，超过就回退，别硬顶
GRIP_DWELL_COARSE_S  = 0.22
GRIP_DWELL_FINE_S    = 0.35
GRIP_MAX_FROM_JUMP   = 80     # 单次 J6 跳变上限，超过就夹回来，防启动瞬态污染判据
GRIP_SETTLE_S        = 0.35   # 到达起始点后的静置时间（等启动冲击过去再取基线）
GRIP_MAX_JAW         = 674    # 总线 J6 全闭软限位；扫到这儿还没接触就判「夹空」


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


def read_j6_vin_mv(samples=CUR_SAMPLES, gap=0.06):
    """读 J6 电压（总线舵机 $ARM,VIN 第 6 路），多读几次取中位数，返回 int(mV)。

    🔴 总线舵机迁移：旧 $CUR/ACS712 电流判据已删除，改用 $ARM,VIN 电压回读。
    回包格式：`$OK,VIN,<mv1>,...,<mv6>`，J6 是第 6 路（parts[7]）。
    电压堵转判据（GRIP_*_MV）未标定前，本函数只供标定读数用。
    """
    vals = []
    for _ in range(max(1, int(samples))):
        got = _post_json("/api/cmd", {"payload": "ARM,VIN"})
        if not got.get("ok"):
            raise RuntimeError("读电压失败：%s" % got.get("error", got))
        resp = (got.get("resp") or "").strip()
        if not resp.startswith("OK,VIN,"):
            raise RuntimeError("$ARM,VIN 回包异常：%r" % resp)
        parts = resp.split(",")
        try:
            vals.append(int(parts[7]))   # parts: OK,VIN,mv1..mv6 → J6=index 7
        except (IndexError, ValueError):
            raise RuntimeError("$ARM,VIN 回包无法解析：%r" % resp)
        time.sleep(gap)
    return _median(vals)


def grip_until_contact(pose5, start=None, max_jaw=GRIP_MAX_JAW,
                       rise_mv=GRIP_RISE_MV, hold_mv=GRIP_HOLD_MV,
                       preload=GRIP_PRELOAD, verbose=True):
    """从 start 往 max_jaw 分步合爪，读到「明显夹住」的电压就停。

    返回 (pulse, mv, base)；扫到 max_jaw 仍没接触则返回 None（夹空）。

    🔴 总线舵机迁移：判据由 ACS712 电流（mA）改为 $ARM,VIN 电压（mV）。
    ⚠️ 电压堵转阈值（GRIP_RISE_MV / GRIP_HOLD_MV / GRIP_STALL_MV）尚未实机标定，
    本函数会直接抛 NotImplementedError，防止用占位阈值误判夹住/夹空。
    标定法：泄力后手掰 J6 全张→全闭，逐点读 read_j6_vin_mv()，
    找「空载基线 / 接触抬升 / 夹住台阶」的 mV 值，回填常量后再启用本函数。

    pose5 = 前 5 路位置（J1~J5），全程保持不动；本函数**只改 J6**。
    返回 None 时调用方**绝不能抬臂**——那说明爪子是空的。
    """
    if rise_mv is None or hold_mv is None or GRIP_STALL_MV is None:
        raise NotImplementedError(
            "总线舵机 J6 电压堵转阈值（GRIP_RISE_MV/GRIP_HOLD_MV/GRIP_STALL_MV）未标定："
            "请先泄力手掰 J6 全张→全闭，用 read_j6_vin_mv() 逐点读 $ARM,VIN 找台阶后回填")

    pose5 = tuple(int(p) for p in pose5)
    if len(pose5) != 5:
        raise ValueError("pose5 必须是 5 个位置（J1~J5），收到 %d 个" % len(pose5))

    real = int(read_state()[5])
    if start is None:
        start = real
    start = int(start)

    # 起点不许离当前位置太远：大跳变的启动冲击会假冒「夹住」。
    if abs(start - real) > GRIP_MAX_FROM_JUMP:
        start = real + (GRIP_MAX_FROM_JUMP if start > real else -GRIP_MAX_FROM_JUMP)
        if verbose:
            print("  ⚠ 起点离当前 J6=%d 太远（单次跳变上限 %d位置单位）⇒ 改为从 %d 起扫"
                  % (real, GRIP_MAX_FROM_JUMP, start))
    if start != real:
        send_joint(list(pose5) + [start])
        time.sleep(GRIP_SETTLE_S)     # 等启动冲击过去，否则第一读数不可信

    base = read_j6_vin_mv()
    rise = base + int(rise_mv)
    hold = base + int(hold_mv)
    if verbose:
        print("  基线 %d mV（%d 次中位数）｜碰上 基线+%d｜夹住 基线+%d｜J6 %d → %d"
              % (base, CUR_SAMPLES, rise_mv, hold_mv, start, max_jaw))

    cand = None          # 首次「碰上了」的位置：不停爪，只记下来当备选
    last = (start, base)
    p = start
    while p <= max_jaw:
        send_joint(list(pose5) + [p])
        time.sleep(GRIP_DWELL_COARSE_S if p < GRIP_COARSE_UNTIL
                   else GRIP_DWELL_FINE_S)
        mv = read_j6_vin_mv()
        last = (p, mv)
        if verbose:
            print("    J6=%-4d → %+4d mV" % (p, mv - base))

        if mv >= GRIP_STALL_MV:
            back = max(start, p - 60)
            send_joint(list(pose5) + [back])
            raise RuntimeError("电压 %d mV 超兜底阈值 %d，已回退到 J6=%d"
                               % (mv, GRIP_STALL_MV, back))

        # 强判据：明显夹住了 ⇒ 就停在这里，不再往里合（再合只会压坏物体＋发热）
        if mv >= hold:
            if verbose:
                print("  ✅ 夹住：J6=%d，%d mV（基线+%d）" % (p, mv, mv - base))
            return p, mv, base

        # 弱判据：只是「碰上了」⇒ 记下候选点，继续等强判据
        if cand is None and mv >= rise:
            cand = p
            if verbose:
                print("    · 首次接触 J6=%d（基线+%d），继续压到「夹住」为止"
                      % (p, mv - base))

        p += GRIP_STEP_COARSE if p < GRIP_COARSE_UNTIL else GRIP_STEP_FINE

    # 扫完只有弱信号：在候选点后补一段固定预压量，绝不一路合到底
    if cand is not None:
        top = min(int(cand) + int(preload), max_jaw)
        send_joint(list(pose5) + [top])
        time.sleep(GRIP_DWELL_FINE_S)
        mv = read_j6_vin_mv()
        if verbose:
            print("  ✅ 夹住（弱信号+预压 %d位置单位）：J6=%d，%d mV（基线+%d）"
                  % (preload, top, mv, mv - base))
        return top, mv, base

    if verbose and last is not None:
        print("  ⚠ 扫到 J6=%d 仍未接触（最后 %d mV）⇒ 判「夹空」"
              % (last[0], last[1]))
    return None


def run_plan(plan, yes=False, dry_run=False, dynamic_grip=False):
    """依次执行计划里的每一步，每步都等到位再走下一步。

    dynamic_grip=True 时，「合爪」那一步不再发计划里的固定位置，而是调用
    grip_until_contact() 用电压判据当场决定夹持位置；判为夹空则**不抬臂**、
    返回 7。

    返回 0 = 全部到位；2 = 有步骤超时（已在 stderr 说明）；3 = 被限位警告拦下；
    4 = 通信失败；7 = 合爪判为夹空（爪里没有东西，停在原处）。
    """
    for i, step in enumerate(plan.steps, 1):
        print("[%d/%d] %s → %s" % (i, len(plan.steps), step.name,
                                   "/".join(str(p) for p in step.positions)))
        if step.note:
            print("        %s" % step.note)
        if dynamic_grip and "合爪" in step.name:
            print("        （J6 由电压判据动态决定，上面的 %d 只是占位）"
                  % step.positions[5])

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
        # 🔴 动态合爪：不发那条写死的位置，改用电压判据当场算夹持位置。
        #    为什么放在 send_joint 之前：一旦先发了全闭位置，爪子已经夹上去了，
        #    再"动态"就没意义了（而且可能已经过夹）。
        if dynamic_grip and "合爪" in step.name:
            try:
                got = grip_until_contact(step.positions[:5])
            except RuntimeError as exc:
                print("❌ 第 %d 步「%s」合爪失败：%s"
                      % (i, step.name, exc), file=sys.stderr)
                print("   爪的状态未知，先跑一次 state 看位置，再决定手动松开还是重试。",
                      file=sys.stderr)
                return 4
            if got is None:
                print("❌ 第 %d 步「%s」判为夹空（电压全程没抬升）。"
                      % (i, step.name), file=sys.stderr)
                print("   **不要抬臂**：爪里没有东西，抬起来就是空抓。"
                      "先确认物体还在原位、有没有被推偏。", file=sys.stderr)
                return 7
            jaw, mv, base = got
            # 后续「保持夹紧」的步骤同步成实际夹持值；
            # 张爪步骤（J6=JAW_OPEN）不能动，否则会把东西半路丢掉。
            for later in plan.steps[i:]:
                if later.positions[5] == step.positions[5]:
                    later.positions = tuple(later.positions[:5]) + (jaw,)
            print("    ✓ 夹住 J6=%d（%d mV，基线+%d）" % (jaw, mv, mv - base))
            continue

        # 🔴 通信异常必须单独接住：串口中途掉线（CH341 被 USB 复位）时
        #    read_state() 会抛 RuntimeError。如果不接住，进程直接退出，
        #    而臂**正停在半路**，人看到的是「程序崩了」而不是「臂停在哪」。
        try:
            send_joint(step.positions)
            actual, ok = wait_arrive(step.positions)
        except RuntimeError as exc:
            print("❌ 第 %d 步「%s」通信失败：%s" % (i, step.name, exc), file=sys.stderr)
            print("   臂可能停在半路。先单独跑一次 `state` 看实际位置，"
                  "串口断了就先 POST /api/open。", file=sys.stderr)
            return 4

        # 1) 先确认前 5 路真的到位（爪步骤时这 5 路通常不变，会秒返回）
        if not ok:
            print("❌ 第 %d 步「%s」超时未到位：实际 %s / 目标 %s"
                  % (i, step.name, list(actual), list(step.positions)),
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
                jaw_got, jaw_ok = wait_jaw(step.positions[5])
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
                      % (jaw_got, step.positions[5]))
            time.sleep(SETTLE_AFTER_JAW_S)
            # 爪等完再读一次完整状态打印，否则打出来的是「爪还在动」时的中途值，
            # 看起来像没到位（夹住物体时 J6 停在半路、到不了全闭目标值，属正常）。
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

    为什么不写在模块顶层：`state` 子命令只读位置，不需要 IK；
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
    # 坐标选 (8, 12)：hover 后腕高 d≈20 cm < L1+L2 = 23.55 cm，可达。
    # 别用太远的点：hover 抬高会让水平可达半径变小，solve() 会抛「超出可达范围」。
    plan = plan_pick(8.0, 12.0, z=2.0)
    assert len(plan.steps) == 4, len(plan.steps)
    names = [s.name for s in plan.steps]
    assert names[0] == "移到上方" and names[-1] == "抬起", names
    assert plan.steps[0].positions[5] == JAW_OPEN, plan.steps[0].positions
    assert plan.steps[-1].positions[5] == JAW_CLOSE, plan.steps[-1].positions
    # 前两步必须用同一个张开值（下降段爪不能合），后两步必须用闭合值
    assert plan.steps[1].positions[5] == JAW_OPEN, plan.steps[1].positions
    assert plan.steps[2].positions[5] == JAW_CLOSE, plan.steps[2].positions

    # 2) 每步都必须是 6 路，且都落在软限位内
    for s in plan.steps:
        assert len(s.positions) == 6, s
        for i, p in enumerate(s.positions):
            lo, hi = SAFE_LIMITS[i]
            assert lo <= p <= hi, (s.name, JOINT_NAMES[i], p, lo, hi)

    # 3) 高度必须变过：抬升点 ≠ 下降点
    #    用 J4 位置间接看高度变化（J4 是腕俯仰，高度不同它必然不同）
    zs = [s.positions[3] for s in plan.steps]
    assert zs[0] != zs[1], "接近点和下降点应该不同（高度变了）"
    # 合爪点必须和下降点同高（不能边合边动）
    assert zs[1] == zs[2], (zs[1], zs[2])

    # 4) place 必须是 4 步，且 J6 先合后开
    p2 = plan_place(5.0, 12.0, z=2.0)
    assert len(p2.steps) == 4, len(p2.steps)
    assert p2.steps[0].positions[5] == JAW_CLOSE, p2.steps[0].positions
    assert p2.steps[-1].positions[5] == JAW_OPEN, p2.steps[-1].positions

    # 5) 限位裁剪必须生效且被报告。
    #    这里直接测 clip_positions 而不是造一个「可达但超限位」的坐标 ——
    #    后者极难构造（越界的目标点通常先被 solve 以「不可达」拦掉），
    #    而 clip_positions 才是真正干活的那一层，测它更直接。
    raw = (-50, 500, 500, 1200, 500, 800)   # 越界：J1 低于下限、J4/J6 高于上限
    clipped, names = clip_positions(raw)
    assert clipped == (10, 500, 500, 990, 500, 674), clipped
    assert names == ["J1", "J4", "J6"], names
    # 不越界的输入必须原样返回、且不报任何裁剪
    same, none_clipped = clip_positions(HOME_POSITIONS)
    assert same == HOME_POSITIONS and none_clipped == [], (same, none_clipped)

    # 6) home 的 6 路必须与固件 INIT 一致
    h = plan_home().steps[0]
    assert h.positions == HOME_POSITIONS, h.positions

    # 7) 🔴 闭环自检：solve() → to_positions() → fk_positions() 必须回到原坐标。
    #    这一步能抓住「IK 和 FK 各写各的」—— 那种错在纸上永远看不出来，
    #    只有真把臂开过去才会发现差几厘米。
    #    实测：多数点 0.02~0.03 cm（位置 25 位置单位 量化极限）；
    #    J1 接近 0° 的点 0.71 cm（J1 map 伪折点，见 README 的 s=0.7549 一节）。
    #    ⇒ 阈值取 1.0 cm：放得过已知问题，又能抓住真正的错。
    try:
        from handeye_geometry import Kinematics
        kin = Kinematics(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                      "arm_params.json"), ik=ik)
        loop_worst = 0.0
        for (px, py) in ((8.0, 12.0), (5.0, 12.0), (12.0, 12.0), (6.0, 15.0)):
            ang = ik.solve(px, py, z=2.0)
            T = kin.fk_positions(ik.to_positions(ang))
            err = ((T[0][3] - px) ** 2 + (T[1][3] - py) ** 2
                   + (T[2][3] - 2.0) ** 2) ** 0.5
            loop_worst = max(loop_worst, err)
            assert err < 1.0, "IK→FK 闭环误差 %.2f cm 超标 @ (%.0f,%.0f)" % (err, px, py)
    except ImportError:
        loop_worst = None
        print("（跳过第 7 组：没装 numpy，无法跑 IK→FK 闭环）")

    # 8) 🔴 姿态自适应：best_tilt 必须返回「6 路都不越软限位」的解，且比默认 41° 更平。
    got = best_tilt(-5.0, 25.0, 2.0)
    assert got is not None, "(-5, 25) 在 41~96° 内找不到可行姿态"
    bt, bp = got
    assert bt > 41.0, "best_tilt 应比默认 41° 更平，实际 %.1f°" % bt
    for i, v in enumerate(bp):
        lo, hi = SAFE_LIMITS[i]
        assert lo <= v <= hi, ("best_tilt 越限", JOINT_NAMES[i], v, lo, hi)
    best_tilt_deg = bt
    best_tilt_positions = bp

    print("SELF_TEST PASS（8 组断言全过，未联网、未动臂）")
    if loop_worst is not None:
        print("  IK→FK 闭环最大误差 %.2f cm（阈值 1.0，J1 折点区是已知偏差）" % loop_worst)
    print("  best_tilt(-5, 25) = %.0f°（不越软限位的最平姿态）→ %s"
          % (best_tilt_deg, list(best_tilt_positions)))
    print("\npick(8, 12) 的计划（纯结构演示）：")
    for s in plan.steps:
        print("  %-12s %s" % (s.name, s.positions))
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
    ap.add_argument("--z", type=float, default=None, help="抓取高度 cm（必填；已取消 grab_height 默认值）")
    ap.add_argument("--tilt", type=float, default=None,
                    help="夹爪前倾角(度)。不给就从当前 STATE 位置自动反算；"
                         "显式给则用它覆盖。"
                         "写死 0 会让远处目标误报「超出可达范围」")
    ap.add_argument("--auto-tilt", action="store_true",
                    help="对抓取点自动扫「最接近水平且不越软限位」的 tilt（换位置抓取推荐）")
    ap.add_argument("--base-url", default=BASE_URL, help="Flask 地址")

    sub = ap.add_subparsers(dest="cmd")
    p_state = sub.add_parser("state", help="读当前 6 路位置")
    p_home = sub.add_parser("home", help="回初始位")
    p_grip = sub.add_parser("grip", help="电压引导分步合爪：夹到接触就停")
    p_grip.add_argument("--from", dest="from_pulse", type=int, default=None,
                        help="起始 J6 位置，默认当前实际值；离当前位置超过 %d位置单位 会被"
                             "夹回（大跳变的启动冲击会假冒「夹住」）"
                             % GRIP_MAX_FROM_JUMP)
    p_grip.add_argument("--max", dest="max_jaw", type=int, default=GRIP_MAX_JAW,
                        help="扫到这里仍未接触就判夹空，默认 %d" % GRIP_MAX_JAW)
    p_grip.add_argument("--hold", dest="hold_mv", type=int, default=GRIP_HOLD_MV,
                        help="「夹住」判据 mV（相对基线），标定后默认 %s" % GRIP_HOLD_MV)
    p_grip.add_argument("--rise", dest="rise_mv", type=int, default=GRIP_RISE_MV,
                        help="「碰上」判据 mV，只记候选点不停爪，标定后默认 %s" % GRIP_RISE_MV)
    p_grip.add_argument("--lift", dest="lift_dj2", type=int, default=0,
                        help="夹住后把 J2 抬多少 位置单位（+60 为经验值），0=不抬，默认 0")
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
        positions = read_state()
        print("当前位置：" + "  ".join(
            "%s=%d" % (JOINT_NAMES[i], p) for i, p in enumerate(positions)))
        return 0

    if args.cmd == "grip":
        # 🔴 先读一次真实姿态：J1~J5 全程保持不动，只动 J6。
        #    如果臂还没摆到抓取位（比如还在初始位），grip 强行合爪是没意义的，
        #    所以把当前位置打出来让人核对，而不是悄悄按「以为的位置」去动。
        try:
            positions = read_state()
        except RuntimeError as exc:
            print("❌ %s" % exc, file=sys.stderr)
            return 4
        print("  当前位置 %s" % (list(positions),))

        if args.dry_run:
            base = read_j6_vin_mv()
            print("  基线电压 %d mV（只读，不动臂）" % base)
            print("  判据：碰上 基线+%s mV（不停爪）／夹住 基线+%s mV（停爪）"
                  % (args.rise_mv, args.hold_mv))
            print("  J6 %s → %d｜粗步 %d位置单位 / 细步 %d位置单位"
                  % (args.from_pulse or positions[5], args.max_jaw,
                     GRIP_STEP_COARSE, GRIP_STEP_FINE))
            print("\n[dry-run] 一条动作命令都没有发出去。")
            return 0

        if not args.yes:
            print("\n加 --yes 才会真的合爪（现在什么都没发）。")
            return 3

        try:
            got = grip_until_contact(positions[:5], start=args.from_pulse,
                                     max_jaw=args.max_jaw,
                                     rise_mv=args.rise_mv,
                                     hold_mv=args.hold_mv)
        except RuntimeError as exc:
            print("❌ 合爪中止：%s" % exc, file=sys.stderr)
            return 4
        except NotImplementedError as exc:
            print("❌ %s" % exc, file=sys.stderr)
            return 4

        if got is None:
            print("❌ 没夹到东西（电压全程没抬升）。**别抬臂**——"
                  "先确认物体还在爪间、位置有没有偏。", file=sys.stderr)
            return 6

        jaw, mv, base = got
        print("\n✅ 停在 J6=%d（基线 %d → %d mV，+%d）"
              % (jaw, base, mv, mv - base))

        def _curl(positions6):
            """把一条 JOINT 直接渲染成可粘贴的 curl。

            🔴 为什么要在工具里生成：19:1x 我在聊天里写了
            `ARM,POS,...,<上一步打印的J6>` 这种带占位符的命令，用户照贴 ⇒
            中文字节进了 payload ⇒ Flask 回 `'ascii' codec can't encode`。
            数字由程序填好，就不会再有占位符这回事。
            """
            body = ",".join(str(int(v)) for v in positions6)
            return ("  curl -s -X POST %s/api/cmd -H \"Content-Type: application/json\" "
                    "-d '{\"payload\":\"ARM,POS,%s\"}'; echo" % (BASE_URL, body))

        pose5 = list(positions[:5])
        if args.lift_dj2:
            up = list(pose5)
            up[1] += int(args.lift_dj2)
            up_c, cut = clip_positions(up + [jaw])
            try:
                send_joint(up_c)
                actual, arrived = wait_arrive(up_c)
            except RuntimeError as exc:
                print("❌ 抬起失败：%s" % exc, file=sys.stderr)
                print("   🔴 爪还夹着东西，别手动松爪、也别再发命令；"
                      "先跑一次 state 看实际位置。", file=sys.stderr)
                return 4
            print("  抬起：J2 %+d位置单位 → %s" % (args.lift_dj2, list(up_c)))
            if cut:
                print("  ⚠ 抬起目标越界已裁 %s ⇒ 实际抬升不到 %d位置单位"
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
            sug_c, cut = clip_positions(sug + [jaw])
            print("  两条可直接粘贴（数字已填好，不用改）：")
            print("   ① 抬起（J2 +60位置单位）：")
            print(_curl(sug_c))
            if cut:
                print("      ⚠ J2 +60 越界已裁 %s" % "/".join(cut))
            print("   ② 放回（两步，别一次到位——半空张爪会掉一段）：")
            print(_curl(pose5 + [jaw]))
            print(_curl(pose5 + [JAW_OPEN]))
        return 0

    if args.cmd == "home":
        return run_plan(plan_home(), yes=args.yes, dry_run=args.dry_run)

    # ---- tilt：不显式给就从当前真实位置反算（与拍照/定位那一刻的姿态一致）----
    # 🔴 为什么不能默认写死 0：solve() 里 r_w = r − L3·sin(tilt)，
    #    用 0 会让 r_w 少扣约 11cm，远处目标被误判「超出可达范围」。
    tilt = args.tilt
    if tilt is None and not args.auto_tilt:
        try:
            _cur = read_state()
            tilt = _get_ik().tilt_from_positions(_cur)
            print("  tilt 自动反算：%.2f°（当前位置 %s）"
                  % (tilt, ",".join(str(p) for p in _cur)))
        except Exception as e:
            tilt = 0.0
            print("  ⚠ 读不到当前姿态，tilt 退回 0（远目标可能误报不可达）：%s" % e)
    if tilt is None:
        tilt = 0.0          # --auto-tilt 时用不到它，给个占位满足签名

    try:
        if args.cmd == "pick":
            plan = plan_pick(args.x, args.y, z=args.z, hover=args.hover,
                             tilt=tilt, auto_tilt=args.auto_tilt)
        else:
            plan = plan_place(args.x, args.y, z=args.z, hover=args.hover,
                              tilt=tilt, auto_tilt=args.auto_tilt)
    except ValueError as exc:
        print("❌ %s" % exc, file=sys.stderr)
        return 5
    return run_plan(plan, yes=args.yes, dry_run=args.dry_run)


if __name__ == "__main__":
    sys.exit(main())
