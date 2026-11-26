#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pick_pipeline.py —— 一句话到抓取的完整链路（A 版本，桌面固定臂）。

它是整条链路的「中间件 + 总调度」：
    检测服务 /observe  →  VLM 只选编号  →  bbox 底边两角反投影到桌面
    →  /api/ik/eye/solve 出工作台坐标 + 逆解  →  action_primitives 出 4 步动作
    →  逐步下发并回读确认（可选接着放到盒子上）

怎么用（板 · openEuler 终端）：
    # 0) 前置：Flask 带 --eye-in-hand 起好、detector_service 在跑、串口已 open
    python3 pick_pipeline.py --self-test                      # 不联网自检，先跑这个
    python3 pick_pipeline.py --instruction "把杯子抓起来" --dry-run
    python3 pick_pipeline.py --instruction "把杯子抓起来" --yes
    python3 pick_pipeline.py --instruction "把杯子抓起来" --yes \
            --place-x 6 --place-y 20                          # 抓到后放到 (6,20) cm

------------------------------------------------------------------------------
README 三行法
① 用了什么语法：argparse 子命令式参数；dataclasses 表达「一次定位结果」；
   urllib.request 发 HTTP（板上不装 requests 也能跑）；异常分层
   （_Fatal / RuntimeError / ValueError 各管一段）；可替换的函数参数
   localizer=... 做依赖注入（自测时喂假函数）；f-string % 混用按可读性选；
   contextlib 无关依赖；typing 只用于注释不当运行时；sys.exit(码) 把
   每一步失败变成**可编程的返回码**，方便脚本串联。
② 为什么这么设计：这条链上有 4 个「会静默出错」的地方 ——
   占位标定数据、臂还在动时拍帧、bbox 被画面裁掉、目标越软限位。
   它们都不会报错，只会让你「抓偏一点」，而抓偏是没法用眼睛判断是
   哪一环错的。所以本文件把每一个都做成**显式闸门**：宁可拒绝执行并
   说清原因，也不发出一条自己都不信的 JOINT。
③ 不这么写会怎样：会出现「命令发出去、臂动了、杯子没抓到」这种最难查的
   故障 —— 因为每一步都"成功"了。开环舵机 + 单目相机的组合里，
   唯一的真相来源是 /api/state 回读和几何自洽，不是"我觉得应该对了"。
------------------------------------------------------------------------------
"""

import argparse
import json
import math
import os
import sys
import time
import urllib.error
import urllib.request


# ---------------------------------------------------------------------------
# 〇、把三个目录塞进 import 路径
# ---------------------------------------------------------------------------
# 为什么需要：仓库里是 kunpeng/{robot,llm,vision}/ 三层分目录，
# 板上却是**扁平**拷到 ~/kunpeng_vision/ 一个目录里（scp 一把梭）。
# 两种布局都得能 import，所以把「本文件所在目录」「../llm」「../vision」
# 全部加进去，谁在就找谁。
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
for _p in (_THIS_DIR,
           os.path.join(_THIS_DIR, "..", "llm"),
           os.path.join(_THIS_DIR, "..", "vision")):
    _p = os.path.abspath(_p)
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

import action_primitives as ap          # noqa: E402  动作原语（同目录/扁平）
import vlm_decider as vd                # noqa: E402  语义决策（../llm/扁平）


# ---------------------------------------------------------------------------
# 一、常量：全部写死，避免"差一个默认值就抓偏"的隐性行为
# ---------------------------------------------------------------------------

# 工作台坐标系：原点在 J1 底座中心的地面投影，x 向前、y 向左、z 向上，单位 cm。
# 🔴 桌面就是 z=0 平面（arm_params.json 的 shoulder_height=8.1 是 J2 轴高）。
TABLE_PLANE_Z = 0.0

# 判定「臂已静止」的脉宽容差。8 µs = 软件步长 25 µs 的三分之一，
# 比舵机死区（5 µs）略宽 —— 用它区分「还在斜坡上走」和「已经站住」。
STILL_TOL_US = 8
STILL_TRIES = 4
STILL_GAP_S = 0.35

# 人手安全闸：目标框与 person 框的交并比超过它就拒绝执行。
# 0.3 是经验值：框只要"压住"三成，机械臂下探时就可能碰到手指。
PERSON_IOU_LIMIT = 0.30

# 目标框离画面左右边界小于这个像素数就不抓 —— 框被裁掉一侧时，
# 底边两角点的其中一个已经不在物体上了，反投影出来的宽度必错。
EDGE_MARGIN_PX = 3

# 夹爪全张时的开口宽度（cm）。超过它的物体抓不进去，只警告不阻止
# （斜着夹、夹边缘有时也能成），因为这是"经验值不是物理上限"。
JAW_MAX_OPEN_CM = 5.0

# 两个角点反投影后的 3D 距离与「按像素宽反推的宽度」差太多，说明框歪了。
# 这里只做提示，不做拒绝：透视下单目估宽本来就有一两厘米误差。
WIDTH_SANITY_CM = 6.0


# ---------------------------------------------------------------------------
# 二、异常：分三类，让 main() 能给出不同返回码
# ---------------------------------------------------------------------------
class Fatal(Exception):
    """环境没准备好（链路没启、串口没连、标定是占位）—— 一条命令都别发。

    为什么不复用 RuntimeError：环境问题和「这一帧画面不好」要有区别，
    前者必须人去看配置，后者换一帧就好。返回码分开，脚本才能自动重试。
    """

    def __init__(self, msg, code=1):
        Exception.__init__(self, msg)
        self.code = code


class Rejected(Exception):
    """这一刻不该动臂（人手在旁边、目标贴边、目标不可达）—— 合理拒绝，不是 bug。

    返回码 11：区别于 1（环境错）和 2（执行超时）。自动化里 11 表示
    「等一会儿/换个摆法再来」，不需要修代码。
    """

    def __init__(self, msg, code=11):
        Exception.__init__(self, msg)
        self.code = code


# ---------------------------------------------------------------------------
# 三、HTTP 小工具：只发 JSON，出错转成人话
# ---------------------------------------------------------------------------
def install_direct_opener():
    """把所有 urllib 请求固定成「直连」，绕开系统代理。

    🔴 这不是可选优化，是**必须**：开发机（Windows）上常常开着 Clash/V2Ray，
       环境变量里有 HTTP_PROXY=127.0.0.1:7897。urllib 会老老实实把
       "访问 127.0.0.1:5000" 也交给代理，代理不认识这个地址，回 502 Bad Gateway。
       于是你会看到「Flask 明明起着，脚本却说连不上」—— 这个现象极容易被
       误判成 Flask 没启动，从而去改一堆无关的东西。

    规范定义：`urllib.request.install_opener()` 替换模块级默认 opener，
    `ProxyHandler({})`（空字典）= 没有任何代理，即直连。

    为什么要全局替换（而不是只在本文件的发请求处用自定义 opener）：
    action_primitives / vlm_decider 内部也是 `urllib.request.urlopen`，
    它们同样受影响；只修一处等于没修。

    代价：如果以后要调**云端** VLM（需要走代理），得加 --keep-proxy 保留原环境。
    """
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    urllib.request.install_opener(opener)


def http_json(url, payload=None, timeout=8.0):
    """GET（payload=None）或 POST 一个 JSON，返回 dict。

    🔴 为什么把 HTTPError 和 URLError 分开接：前者是「服务在，但它拒绝/报错」，
       响应体里有服务端写的真实原因（比如"逆解失败：超出可达范围"），
       这条信息是排障的全部价值；后者是「根本连不上」，只需要看地址和端口。
       混在一起接，就会把最有用的那句话丢掉。
    """
    if payload is None:
        req = urllib.request.Request(url)
    else:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        req = urllib.request.Request(
            url, data=body, headers={"Content-Type": "application/json"},
            method="POST")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            raw = resp.read().decode("utf-8", "replace")
    except urllib.error.HTTPError as e:
        detail = e.read().decode("utf-8", "replace")[:300] if e.fp else ""
        raise Fatal("服务端返回 HTTP %s：%s" % (e.code, detail or url))
    except urllib.error.URLError as e:
        raise Fatal("连不上 %s（%s）" % (url, e.reason))
    try:
        return json.loads(raw)
    except ValueError:
        raise Fatal("服务端返回的不是 JSON：%r" % raw[:200])


# ---------------------------------------------------------------------------
# 四、几何小工具：都是纯函数，自测能直接喂数
# ---------------------------------------------------------------------------
def iou(box_a, box_b):
    """两个 (x1,y1,x2,y2) 框的交并比 IoU（纯函数）。

    大白话：交集面积 ÷ 并集面积，0 = 完全不相干，1 = 完全重合。
    规范定义：IoU = |A ∩ B| / |A ∪ B|，目标检测里衡量两框重叠程度的标准量。
    """
    ax1, ay1, ax2, ay2 = [float(v) for v in box_a[:4]]
    bx1, by1, bx2, by2 = [float(v) for v in box_b[:4]]
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def bottom_corners(bbox, inset_px=1.0):
    """取 bbox「底边」的左上/右上两个角点像素，返回 ((u_left, v), (u_right, v))。

    为什么要底边而不是框中心 —— 这是整个文件最关键的一处取舍：

    大白话：单目相机只看到"一条视线"，不知道物体离你多远。唯一能白捡的
            约束是「物体放在桌面上」—— 那么物体与桌面的接触点在 z=0 平面上。
            bbox 的**底边**差不多就是这条接触线，框中心却在物体半腰的空中。
    规范定义：单目成像 P = K·[R|t]·X 丢掉深度，是一条射线；给它补一个平面约束
             （z = 0）才有唯一交点。用框中心去和 z=0 求交，等于把"物体半腰"
             硬按到桌面上，会产生 tan(俯角)×高度 的系统偏差。

    为什么取"两个角点"而不是"底边中点"：圆柱/方盒的底边两端正好是底面轮廓的
    左右切点，两点反投影后取中点 = 底面中心（抖动还能互相抵消）。只取中点的话，
    得到的是**靠相机这一侧的边缘点**，会稳定偏近半个物体直径。

    inset_px：v 往上收 1 像素。YOLO 的框通常比真实轮廓低 1~2 px，
    直接压在最底行会把"框的 padding"当成物体边缘。
    """
    x1, y1, x2, y2 = [float(v) for v in bbox[:4]]
    v = y2 - float(inset_px)
    return (x1 + 0.5, v), (x2 - 0.5, v)


def clamp01(v):
    """把值夹到 [0,1]（纯函数，只为让打印好看）。"""
    return max(0.0, min(1.0, float(v)))


# ---------------------------------------------------------------------------
# 五、环境体检：三道闸，任一不过就一条命令都不发
# ---------------------------------------------------------------------------
def check_eye_ready(api_base):
    """确认眼在手链路启用、且标定不是占位数据。返回 (info, plane_z_default)。

    🔴 占位数据必须拒绝：`camera_intrinsics.json` / `handeye_X.json` 里如果还是
       gen_placeholder_calib.py 生成的假值，算出来的坐标"看起来很正常"，
       但偏十几厘米。这类错误比直接报错危险一百倍 —— 所以宁可不抓。
    """
    got = http_json(api_base + "/api/ik/eye")
    info = got.get("eye") or {}
    if not info.get("enabled"):
        raise Fatal("眼在手链路未启用：%s\n"
                    "  ⇒ 重启 Flask 时加 --eye-in-hand（见 kunpeng/web/README）"
                    % (info.get("error") or "启动参数缺 --eye-in-hand"))
    if info.get("placeholder"):
        raise Fatal("🔴 标定数据是占位值（flag=%s），坐标不代表真实精度。\n"
                    "  ⇒ 先做实标：handeye_calib.py collect / solve，"
                    "再把 handeye_X.json 换掉" % info.get("flag"))
    return info, got.get("plane_z")


def check_serial(api_base):
    """确认串口连着，并返回一次 /api/state 的脉宽。

    为什么先读一次：串口没 open 时后面每条命令都会失败，先探一下能把
    "没连串口"和"几何算错"两种故障分开 —— 它们的处理方式完全不同。
    """
    got = http_json(api_base + "/api/state", {})
    if not got.get("ok"):
        raise Fatal("读 /api/state 失败：%s\n"
                    "  ⇒ 串口没连？先 POST /api/open {\"port\":\"/dev/serial/by-id/"
                    "usb-1a86_USB_Serial-if00-port0\"}" % got.get("error"))
    pulses = got.get("pulses")
    if not (isinstance(pulses, list) and len(pulses) == 6):
        raise Fatal("状态格式异常：%r" % (pulses,))
    return tuple(int(p) for p in pulses)


def wait_still(api_base, tol=STILL_TOL_US, tries=STILL_TRIES, gap=STILL_GAP_S):
    """等 6 路脉宽连续两次读数一致，返回稳定后的脉宽。

    🔴 为什么必须等：拍照和解算是两个时刻。若拍帧时臂还在斜坡上走，
       相机看到的画面和随后算坐标用的关节角**不是同一瞬间**，
       几度的差就够让它抓空。stop-and-go 的前提本来就是"先停稳再拍"。
    """
    prev = None
    for i in range(tries):
        cur = check_serial(api_base)
        if prev is not None and max(abs(a - b) for a, b in zip(prev, cur)) <= tol:
            return cur
        prev = cur
        if i < tries - 1:
            time.sleep(gap)
    raise Rejected("等了 %.1f 秒臂还在动（最后一次 %s）——先把臂停稳再抓"
                   % (tries * gap, list(prev or [])))


# ---------------------------------------------------------------------------
# 六、定位：两个底角反投影 → 桌面坐标
# ---------------------------------------------------------------------------
def localize_pixel(api_base, u, v, pulses, plane_z=TABLE_PLANE_Z, timeout=8.0):
    """一个像素 + 当前脉宽 → 工作台坐标。走后端 /api/ik/eye/solve。

    大白话：把像素当成"从相机的眼睛射出去的一条线"，问它"打到桌面（z=0）上哪一点"。
    规范定义：d_cam = K⁻¹·[u,v,1]ᵀ；转到基座系 d_base = R_bc·d_cam；
             求 λ 使 (O_base + λ·d_base)_z = plane_z，交点即该像素对应的桌面点。
    🔴 为什么不在本文件里自己算：这套几何已经在 handeye_geometry.py 里
       实现并做过自测（7 组），服务端一份、客户端再抄一份必然漂移。
       这里只做「调接口 + 校验返回值」，单一事实来源。
    """
    payload = {"u": float(u), "v": float(v),
               "pulses": [int(p) for p in pulses],
               "plane_z": float(plane_z)}
    got = http_json(api_base + "/api/ik/eye/solve", payload, timeout=timeout)
    if not got.get("ok"):
        raise Rejected("定位失败（像素 %.1f,%.1f）：%s"
                       % (float(u), float(v), got.get("error")))
    if got.get("clamped"):
        # 服务端在这种情形下也会回 ok=True，只是把裁过的关节列出来。
        # 🔴 必须当失败处理：裁过 = 到不了目标点，抓取落点必然偏。
        raise Rejected("目标点逆解越界，被软限位裁掉：%s（目标 %.1f,%.1f,%.1f cm）"
                       % ("/".join(got["clamped"]),
                          got.get("x", 0), got.get("y", 0), got.get("z", 0)), code=13)
    return got


def locate_object(api_base, bbox, pulses, plane_z=TABLE_PLANE_Z):
    """bbox → 物体底面中心在工作台的 (x, y)，顺带估一个地面宽度。

    返回 dict：x, y, z, span_cm, corner_a, corner_b, pixel_v

    span_cm = 两个底角的三维距离，粗略等于物体"从上方看到的宽度"。
    它的用途不是测量，是**合理性检查**：杯子 7cm、瓶子 6cm，
    如果 span 算出 20cm，那多半是框歪了或是反光把轮廓拉长了。
    """
    (ua, va), (ub, vb) = bottom_corners(bbox)
    ra = localize_pixel(api_base, ua, va, pulses, plane_z)
    rb = localize_pixel(api_base, ub, vb, pulses, plane_z)

    xa, ya = float(ra["x"]), float(ra["y"])
    xb, yb = float(rb["x"]), float(rb["y"])
    span = math.hypot(xb - xa, yb - ya)

    out = {
        "x": round((xa + xb) / 2.0, 2),
        "y": round((ya + yb) / 2.0, 2),
        "z": float(plane_z),
        "span_cm": round(span, 2),
        "corner_a": (round(xa, 2), round(ya, 2)),
        "corner_b": (round(xb, 2), round(yb, 2)),
        "pixel_v": round(float(vb), 1),
        # 🔴 后端在这一步已把「当前真实前倾角」反算出来（响应里的 tilt 字段），
        # 动作计划必须用同一个值：solve() 里 r_w = r − L3·sin(tilt)，
        # 用 0 会让 C 姿态(41°) 下的 r_w 少扣约 11cm，远处目标误判「超出可达范围」。
        # 这是 11-16 实测踩到的坑（同一份目标，tilt=0 报 d=28.0、tilt=41 正常解出）。
        "tilt": float(ra.get("tilt", 0.0)),
        # 🔴 原始坐标与径向修正系数一起带出来：定位"准不准"必须能看见
        #    「修正前 → 修正后」这一步，否则只能看到一个已经乘过 k 的结果，
        #    没法判断偏差是几何算错还是 k 用错（11-16 用户量出爪尖 (−4,27)
        #    与定位 23.65 对不上时，就是缺这两个数才查不下去）。
        "x_raw": float(ra.get("x_raw", ra["x"])),
        "y_raw": float(ra.get("y_raw", ra["y"])),
        "radial_k": float(ra.get("radial_k", 1.0)),
        "radial_shift": float(ra.get("radial_shift", 0.0)),
        "depth": float(ra.get("depth", 0.0)),
    }
    if span > 40.0:
        # 两个底角跑出 40cm 以上：几乎一定是其中一个角点落到了背景上
        # （框横跨了两个物体 / 反光边缘）。这种点不能用来抓。
        raise Rejected("两个底角的三维距离 %.1f cm 离谱（框可能横跨了多个物体）"
                       % span)
    return out


# ---------------------------------------------------------------------------
# 七、安全闸：人、边界、夹爪开口
# ---------------------------------------------------------------------------
def person_guard(cands, target_bbox, limit=PERSON_IOU_LIMIT):
    """目标框与任何 person 框重叠超过 limit 就拒绝。返回警告文本或 None。

    大白话：机械臂不知道"夹到手指了"——舵机是开环的，没有力反馈、没有碰撞检测。
            所以唯一能做的安全措施是**事前不许靠近人**。
    规范定义：用 COCO 的 person 类做区域干涉检查（region interference check），
             以 IoU 为判据；这是低成本机器人里的标准"软急停"。

    🔴 阈值放在 0.3 而不是 0.5：两个框只要有三分之一的面积压在一起，
       夹爪下探 2cm 时就可能碰到手指，而我们的夹持力（25kg·cm 舵机）足以夹伤。
       这里宁可误拒，也不能漏拒。
    """
    worst = 0.0
    worst_box = None
    for c in cands:
        if c.cls != "person":
            continue
        ov = iou(c.bbox, target_bbox)
        if ov > worst:
            worst, worst_box = ov, c
    if worst >= limit:
        return ("目标框与 person 框重叠 %.0f%%（阈值 %.0f%%）——"
                "请把物体放到桌面上、人松手后退，再重新执行"
                % (worst * 100, limit * 100))
    if worst > 0.05:
        return ("⚠ 目标与人框有 %.0f%% 重叠（未超阈值），下探时请注意安全"
                % (worst * 100))
    return None


def edge_guard(bbox, frame_w, frame_h, margin=EDGE_MARGIN_PX):
    """目标框贴边就拒绝（框被裁掉一侧时底角会落在背景上）。返回原因或 None。

    这是本文件唯一"看起来多余"的检查，但它挡掉的故障很典型：
    杯子只露出半个，YOLO 仍会给 0.8 的置信度，框的一边落在画面外，
    底边那一角根本不是物体的轮廓 —— 反投影后坐标偏 3~5cm，
    而流程上"每一步都成功"。
    """
    x1, y1, x2, y2 = [float(v) for v in bbox[:4]]
    if x1 <= margin or x2 >= frame_w - 1 - margin:
        return "目标框贴到画面左右边界（x1=%.0f x2=%.0f，画面宽 %d）" % (x1, x2, frame_w)
    if y2 >= frame_h - 1 - margin:
        return "目标框底边贴到画面下边界（y2=%.0f，画面高 %d）" % (y2, frame_h)
    if y2 <= y1 + 2:
        return "目标框高度异常（y1=%.0f y2=%.0f）" % (y1, y2)
    return None


def jaw_guard(span_cm, jaw_max=JAW_MAX_OPEN_CM):
    """物体地面宽度超过夹爪开口就给警告（不拒绝）。"""
    if span_cm > jaw_max:
        return ("物体宽度约 %.1f cm > 夹爪开口 %.1f cm —— 多半夹不进去，"
                "建议换小一点的物体或侧向夹" % (span_cm, jaw_max))
    return None


# ---------------------------------------------------------------------------
# 八、决策：拿观测 → 让模型只选编号
# ---------------------------------------------------------------------------
def make_backend(kind, model, base_url, timeout):
    """构造决策后端：mock 用于演练/没有模型时，vlm 用 OpenAI 兼容接口。

    🔴 为什么保留 mock：整条链路的几何、安全闸、动作序列都能在没有模型时
       完整验证一遍。把"模型选得对不对"和"选对之后能不能抓准"分开测，
       出问题时才知道该修哪一半。
    """
    if kind == "mock":
        return vd.MockBackend()
    # 🔴 Key 只从环境变量读：云端 VLM（阿里云百炼/智谱等 OpenAI 兼容口）必须带
    #    Authorization，本地 ollama 无需 Key 时留空即可。漏传会让云端 401，
    #    而报错往往只写「HTTP 401」，容易误判成模型名拼错。
    api_key = os.environ.get("VLM_API_KEY", "")
    return vd.HttpBackend(base_url=base_url, model=model, api_key=api_key,
                          timeout=timeout)


def decide(instruction, detector_url, backend, timeout=8.0):
    """拉一次观测并做决策，返回 (decision_dict, observation_info)。"""
    try:
        frame_id, cands, w, h, jpeg = vd.fetch_observation(detector_url, timeout=timeout)
    except RuntimeError as e:
        raise Fatal(str(e))

    if not cands:
        raise Rejected("画面里一个物体都没检测到 —— 把目标放到画面中央、"
                       "确认 detector_service 在跑", code=10)

    d = vd.decide_from_observation(instruction, frame_id, cands, w, h, jpeg, backend)
    obs = {"frame_id": frame_id, "w": w, "h": h,
           "n": len(cands), "cands": cands, "jpeg": jpeg}
    return d, obs


def render_candidates(cands):
    """把候选列表排成人看的表格行（纯函数）。"""
    lines = []
    for c in cands:
        lines.append("    [%d] %-12s %.2f  中心=(%4d,%4d) 方位=%s"
                     % (c.idx, c.cls, c.conf, c.center[0], c.center[1],
                        c.direction or "-"))
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# 九、动作计划：定位结果 → 4 步抓取（+ 可选 4 步放置）
# ---------------------------------------------------------------------------
def build_plan(x, y, grab_z, hover, place_xy=None, tilt=0.0, compensate=True):
    """生成动作计划，返回 (plan, notes)。notes 是给人看的提示，不影响执行。

    🔴 tilt 必须传「定位那一刻反算出来的真实前倾角」（`locate_object()` 返回的
    `tilt` 字段）。漏传会用 0，solve() 里 r_w = r − L3·sin(tilt) 就少扣约 11cm，
    远处目标被误判「超出可达范围」——这是 11-16 实际踩到的坑。

    🔴 compensate：是否扣执行侧重力偏置（`TIP_BIAS_XY` + `SAG_DZ`）。
      默认 True —— 不扣会矮 3.88cm、近 0.95cm，表现是「4 步全到位却碰都没碰到」。
      代价是目标 z 抬高 3.88cm 后**可达范围会变小**：原先勉强够到的点可能变不可达，
      这时宁可报错也别硬抓（硬抓就是偏 3.88cm 去撞）。端到端回归传 False 是为了
      验链路结构，不代表生产行为。
    """
    notes = []
    plan = ap.plan_pick(x, y, z=grab_z, hover=hover, tilt=tilt,
                        compensate=compensate)

    if place_xy is not None:
        px, py = place_xy
        place = ap.plan_place(px, py, z=grab_z, hover=hover, tilt=tilt,
                              compensate=compensate)
        plan.steps.extend(place.steps)
        plan.warnings.extend(place.warnings)
        notes.append("放置点 (%.1f, %.1f) cm：这是**示教值**，不是视觉找出来的"
                     "（COCO 里没有 box 类，盒子只能人工摆好后量一次坐标）"
                     % (px, py))
    return plan, notes


# ---------------------------------------------------------------------------
# 十、主流程
# ---------------------------------------------------------------------------
def run(args):
    """整条链路跑一次，返回进程退出码。"""
    api_base = args.api.rstrip("/")
    detector_url = args.detector.rstrip("/")

    # ===== 阶段 -1：网络环境（必须在第一个请求之前）=====
    # 默认强制直连：本地 Flask / detector 都不该经过系统代理（会 502）。
    if not args.keep_proxy:
        install_direct_opener()

    # 🔴 把 --api 传给动作层：action_primitives 的 BASE_URL 是**模块级常量**
    #    （默认 127.0.0.1:5000），不会读本工具的 --api。不改这一行，
    #    本工具读状态/定位都正常，唯独真正动臂的那几条命令会发到 5000 端口去 ——
    #    现象是「前面每一步都成功，发命令时突然连不上」，很容易误判成 Flask 挂了。
    ap.BASE_URL = api_base

    # ===== 阶段 0：环境体检（一条命令都还没发）=====
    print("=" * 66)
    print("阶段 0 · 环境体检")
    info, plane_z_default = check_eye_ready(api_base)
    print("  眼在手链路：启用（flag=%s）" % (info.get("flag") or "?"))
    print("  服务端默认平面高度：%s cm" % plane_z_default)
    if plane_z_default not in (None, TABLE_PLANE_Z):
        print("  ⚠ 服务端默认 plane_z=%s ≠ 0，本工具显式传 %.1f（桌面），不看它的默认值"
              % (plane_z_default, TABLE_PLANE_Z))

    # ===== 阶段 1：停稳 + 拍帧 + 决策 =====
    print("-" * 66)
    print("阶段 1 · 等臂停稳 → 拍帧 → 让模型选目标")
    pulses = wait_still(api_base)
    print("  臂已静止，脉宽：%s" % ",".join(str(p) for p in pulses))

    backend = make_backend(args.backend, args.model, args.base_url, args.model_timeout)
    t0 = time.time()
    d, obs = decide(args.instruction, detector_url, backend, timeout=args.observe_timeout)
    dt = time.time() - t0

    print("  帧 %s（%dx%d）共 %d 个候选，决策耗时 %.1f s，来源 %s"
          % (obs["frame_id"], obs["w"], obs["h"], obs["n"], dt, d.source))
    print(render_candidates(obs["cands"]))
    print("  模型结论：action=%s  choice=%s  target=%s"
          % (d.action, d.target_idx, d.target_class))
    if d.reason:
        print("  理由：%s" % d.reason)
    for w in d.warnings:
        print("  ⚠ %s" % w)

    if args.save_frame and obs["jpeg"]:
        with open(args.save_frame, "wb") as f:
            f.write(obs["jpeg"])
        print("  已存观测帧 %s" % args.save_frame)

    # ===== 阶段 2：决策合法性 =====
    print("-" * 66)
    print("阶段 2 · 决策校验")
    if d.action != "pick":
        print("  结论不是 pick（是 %s），本工具只执行抓取 —— 不动臂。" % d.action)
        if d.action in ("home", "look"):
            print("  ⇒ 这两个动作请用 action_primitives.py home 或直接看视频流")
        return 10
    if not d.target_bbox:
        raise Rejected("模型说 pick 但没给编号/框（choice=%s）—— 拒绝执行"
                       % (d.target_idx,), code=10)
    bbox = tuple(int(v) for v in d.target_bbox)
    print("  目标：%s  框=%s" % (d.target_class, list(bbox)))

    # 安全闸 ①：人
    msg = person_guard(obs["cands"], bbox, limit=args.person_iou)
    if msg:
        if msg.startswith("⚠"):
            print("  " + msg)
        else:
            raise Rejected(msg)
    # 安全闸 ②：贴边
    bad = edge_guard(bbox, obs["w"], obs["h"])
    if bad:
        raise Rejected(bad + " —— 底角可能不在物体上，反投影会偏")

    # ===== 阶段 3：像素 → 桌面坐标 =====
    print("-" * 66)
    print("阶段 3 · 底边两角反投影到桌面（z=%.1f）" % TABLE_PLANE_Z)
    loc = locate_object(api_base, bbox, pulses, plane_z=TABLE_PLANE_Z)
    print("  左底角 3D：%s" % (loc["corner_a"],))
    print("  右底角 3D：%s" % (loc["corner_b"],))
    print("  ⇒ 底面中心：x=%+.2f  y=%+.2f  z=%+.2f cm（地面宽度约 %.1f cm，前倾 %.2f°）"
          % (loc["x"], loc["y"], loc["z"], loc["span_cm"], loc.get("tilt", 0.0)))
    # 🔴 把「修正前 → 修正后」摊开：定位偏了到底该怪几何还是怪 k，全看这一行。
    print("  · 修正前 x_raw=%+.2f y_raw=%+.2f（径向 %.2f cm）→ ×k=%.3f → 径向 %.2f cm（%+.2f）"
          % (loc["x_raw"], loc["y_raw"],
             math.hypot(loc["x_raw"], loc["y_raw"]),
             loc["radial_k"],
             math.hypot(loc["x"], loc["y"]),
             loc["radial_shift"]))

    warn = jaw_guard(loc["span_cm"], jaw_max=args.jaw_max_cm)
    if warn:
        print("  ⚠ " + warn)
    if loc["span_cm"] > WIDTH_SANITY_CM:
        print("  ⚠ 地面宽度 %.1f cm 偏大，框可能没贴紧物体" % loc["span_cm"])

    # ===== 阶段 4：出计划 =====
    print("-" * 66)
    print("阶段 4 · 生成动作计划（抓取高度 z=%.1f cm，悬停 +%.1f cm）"
          % (args.grab_z, args.hover))
    place_xy = None
    if args.place_x is not None and args.place_y is not None:
        place_xy = (float(args.place_x), float(args.place_y))

    if getattr(args, "recipe", ""):
        # 🔴 C 方案：J1 用视觉、J2~J5 用示教。
        #    为什么这么做：实测同一点上，IK 给的下降位是 J2=1532/J4=1103，
        #    而人工示教抓到过的是 J2=1279/J4=1850（差 253/747 µs），
        #    且 J4=1850 已顶软限位——说明 IK 模型与真实机构在那一段有系统偏差，
        #    调参消不掉。J1 反而两边一致（1673 vs 1691），所以只信 J1。
        try:
            data = ap.load_recipes()
        except RuntimeError as e:
            raise Rejected("读不到配方库：%s" % e, code=15)
        rec = data.get("recipes", {}).get(args.recipe)
        if rec is None:
            raise Rejected("配方库里没有「%s」，可用：%s"
                           % (args.recipe, ", ".join(sorted(data.get("recipes", {})))),
                           code=15)
        r_steps = rec.get("steps") or []
        if not r_steps:
            raise Rejected("配方「%s」没有 steps" % args.recipe, code=15)
        j1_recipe = int(r_steps[0]["pulses"][0])
        az_raw = math.degrees(math.atan2(loc["y"], loc["x"]))
        az_bias = float(getattr(args, "az_bias", 0.0))
        j1_vision, j1_deg, az = ap.j1_pos_from_xy(loc["x"], loc["y"],
                                                    az_bias_deg=az_bias)
        offset = j1_vision - j1_recipe
        print("  示教姿态模式（--recipe %s）：J2~J5 取示教值，J1 由视觉算"
              % args.recipe)
        print("    ⚠ 此模式下 --grab-z / --hover / --place-x / --place-y 全部不生效，"
              "抓取高度与放回点都由配方的示教值决定")
        if az_bias:
            print("    方位角 %.2f° %+0.2f°（--az-bias）= %.2f° → J1 角度 %.2f° → J1=%d"
                  % (az_raw, az_bias, az, j1_deg, j1_vision))
        else:
            print("    方位角 %.2f° → J1 角度 %.2f° → J1=%d"
                  "（未补 --az-bias；实测偏小约 2.9° ⇒ 传 2.9 会让 J1 大 %.0fµs）"
                  % (az_raw, j1_deg, j1_vision, 2.9 * 9.375))
        print("    配方示教 J1=%d，偏移 %+d µs" % (j1_recipe, offset))
        plan = ap.build_recipe_plan(rec, j1_offset=offset)
        notes = ["J2~J5 来自示教、J1 来自视觉方位角；不经 IK"]
    else:
        try:
            # 🔴 tilt 用定位阶段反算出来的真实值，不能漏（漏了就等于用 0）
            plan, notes = build_plan(loc["x"], loc["y"], args.grab_z, args.hover,
                                     place_xy,
                                     tilt=loc.get("tilt", 0.0),
                                     compensate=not getattr(args, "no_compensate", False))
        except ValueError as e:
            # 🔴 逆解抛的是 ValueError（"目标点超出机械臂可达范围"），不是 Rejected。
            #    这里必须接住并翻译成人话 —— 否则用户看到的是一段 Python traceback，
            #    而真正该告诉他的是「这个点够不到，把物体往前挪」。
            #
            #    为什么「够不到」常常发生在**悬停点**而不是抓取点：
            #    腕关节在末端上方约 L3（17cm），把末端抬高 hover 厘米，
            #    等于把腕也抬高 hover 厘米，于是水平可达半径反而变小。
            #    grab_z=2、hover=4 时，腕高 = 2+4+17-8.1 = 14.9 cm，
            #    留给水平的余量只有 sqrt(25.2² - 14.9²) ≈ 20.3 cm。
            #
            #    🔴 开了重力补偿后 z 还要再抬 3.88cm（补 SAG_DZ），余量进一步缩小：
            #       所以「关掉补偿能到、开着补偿到不了」是正常现象，不是 bug。
            raise Rejected("逆解失败（目标 %.1f, %.1f cm，悬停高度 %.1f cm）：%s\n"
                           "  ⇒ 常见原因是目标太远/太高。悬停点越高，水平可达半径越小；"
                           "把物体往 J1 方向挪几厘米，或把 --hover 调小"
                           "（开了重力补偿时 z 额外 +3.88cm，可加 --no-compensate 对比）"
                           % (loc["x"], loc["y"], args.grab_z + args.hover, e), code=12)
    for n in notes:
        print("  · " + n)

    # ===== 阶段 5：执行 =====
    print("-" * 66)
    print("阶段 5 · 执行")
    if plan.warnings and not args.allow_clamp:
        print("  🔴 计划里有软限位裁剪 —— 落点会偏，拒绝执行：")
        for w in plan.warnings:
            print("    - " + w)
        print("  确认可以接受就加 --allow-clamp")
        return 13

    code = ap.run_plan(plan, yes=args.yes, dry_run=args.dry_run,
                       dynamic_grip=args.dynamic_grip)

    print("-" * 66)
    if code == 0 and not args.dry_run:
        print("✅ 完成：%s → (%+.2f, %+.2f) cm%s"
              % (d.target_class, loc["x"], loc["y"],
                 "，已放到 (%.1f, %.1f)" % place_xy if place_xy else ""))
    elif code == 0:
        print("[dry-run] 链路全部走通，没有任何一条命令发出去。")
        print("  确认无误后把 --dry-run 换成 --yes。")
    else:
        print("❌ 执行未完成，run_plan 返回码 %d"
              "（2=超时 3=未确认 4=通信失败 7=夹空未抬臂）" % code)
    return code


# ---------------------------------------------------------------------------
# 十一、自检：不联网、不动臂、不需要模型
# ---------------------------------------------------------------------------
def self_test():
    """纯逻辑自测。覆盖 6 组：几何、安全闸、定位、计划、串起来的最小闭环。"""
    ok = 0

    # 1) IoU 基本正确性
    assert abs(iou((0, 0, 10, 10), (0, 0, 10, 10)) - 1.0) < 1e-9
    assert abs(iou((0, 0, 10, 10), (10, 10, 20, 20)) - 0.0) < 1e-9
    assert abs(iou((0, 0, 10, 10), (5, 0, 15, 10)) - (50.0 / 150.0)) < 1e-9
    assert iou((0, 0, 10, 10), (20, 20, 30, 30)) == 0.0
    ok += 1
    print("  ① IoU：重合=1、相离=0、半叠=1/3 ✓")

    # 2) 底边两角：v 要落在 y2 附近且比 y2 小（往上收 1 px）
    (ua, va), (ub, vb) = bottom_corners((100, 200, 300, 400))
    assert va == vb == 399.0, (va, vb)
    assert ua == 100.5 and ub == 299.5, (ua, ub)
    assert va == vb, "两个角点必须同一行，否则反投影不在同一深度"
    ok += 1
    print("  ② 底边两角：同一行 v=399、左右各内收 0.5 px ✓")

    # 3) 人闸：重叠 80% 必须拒绝，轻微重叠只警告
    class _C(object):
        def __init__(self, cls, bbox):
            self.cls = cls
            self.bbox = bbox

    cands = [_C("person", (90, 190, 310, 410))]
    msg = person_guard(cands, (100, 200, 300, 400))
    assert msg and not msg.startswith("⚠"), msg
    msg2 = person_guard([_C("person", (0, 0, 20, 20))], (100, 200, 300, 400))
    assert msg2 is None, msg2
    ok += 1
    print("  ③ 人闸：压住 80% 拒绝、不挨着放行 ✓")

    # 4) 贴边闸
    assert edge_guard((0, 100, 200, 300), 640, 480) is not None
    assert edge_guard((100, 100, 639, 300), 640, 480) is not None
    assert edge_guard((100, 100, 300, 479), 640, 480) is not None
    assert edge_guard((100, 100, 300, 300), 640, 480) is None
    assert edge_guard((100, 300, 300, 301), 640, 480) is not None   # 高度塌成 1px
    ok += 1
    print("  ④ 贴边闸：左/右/下贴边全拒、正常框放行 ✓")

    # 5) 定位：用假 localizer 验证"两角取中点"和"离谱宽度拒绝"
    calls = []

    def fake_http(url, payload=None, timeout=8.0):
        calls.append((payload["u"], payload["v"]))
        # 左角 → (10, 20)，右角 → (16, 20)，中点应为 (13, 20)，宽度 6
        u = payload["u"]
        return {"ok": True, "x": 10.0 if u < 200 else 16.0, "y": 20.0,
                "z": payload["plane_z"], "clamped": []}

    global http_json
    real_http = http_json
    try:
        http_json = fake_http
        loc = locate_object("http://x", (100, 200, 300, 400), (1580, 1900, 2400, 700, 1450, 1500))
        assert abs(loc["x"] - 13.0) < 1e-9, loc
        assert abs(loc["y"] - 20.0) < 1e-9, loc
        assert abs(loc["span_cm"] - 6.0) < 1e-9, loc
        assert loc["z"] == 0.0, loc
        assert len(calls) == 2, calls

        # 宽度离谱（两角相距 50cm）⇒ 必须拒绝
        def fake_far(url, payload=None, timeout=8.0):
            u = payload["u"]
            return {"ok": True, "x": 0.0 if u < 200 else 50.0, "y": 0.0,
                    "z": 0.0, "clamped": []}

        http_json = fake_far
        try:
            locate_object("http://x", (100, 200, 300, 400), (1580, 1900, 2400, 700, 1450, 1500))
            raise AssertionError("宽度 50cm 居然没被拒绝")
        except Rejected:
            pass
    finally:
        http_json = real_http
    ok += 1
    print("  ⑤ 定位：两角取中点=(13,20)、跨度=6cm；跨度 50cm 被拒 ✓")

    # 6) 计划：pick 4 步 + place 4 步，J6 先开后合再开
    plan, notes = build_plan(10.0, 18.0, 2.0, 4.0, place_xy=(6.0, 20.0),
                             compensate=False)
    assert len(plan.steps) == 8, len(plan.steps)
    names = [s.name for s in plan.steps]
    assert names[0] == "移到上方" and names[3] == "抬起", names
    assert names[4] == "移到上方" and names[7] == "抬起", names
    assert plan.steps[0].pulses[5] == ap.JAW_OPEN
    assert plan.steps[2].pulses[5] == ap.JAW_CLOSE
    assert plan.steps[6].pulses[5] == ap.JAW_OPEN
    for s in plan.steps:
        assert len(s.pulses) == 6
        for i, p in enumerate(s.pulses):
            lo, hi = ap.SAFE_LIMITS[i]
            assert lo <= p <= hi, (s.name, i, p, lo, hi)
    assert notes and "示教值" in notes[0]
    ok += 1
    print("  ⑥ 计划：抓 4 步 + 放 4 步，J6 开→合→开，全程不越软限位 ✓")

    # 7) 悬停高度真的抬高了（用 J4 脉宽间接看）
    hover_hi = ap.plan_pick(10.0, 18.0, z=2.0, hover=4.0, compensate=False)
    hover_lo = ap.plan_pick(10.0, 18.0, z=2.0, hover=1.0, compensate=False)
    assert hover_hi.steps[0].pulses != hover_lo.steps[0].pulses, "hover 没起作用"
    ok += 1
    print("  ⑦ 悬停：hover 4cm 与 1cm 的接近点脉宽不同 ✓")

    print("\n✅ 自测 %d 组全过（不联网 / 不动臂 / 不需要模型）" % ok)
    return 0


# ---------------------------------------------------------------------------
# 十二、命令行
# ---------------------------------------------------------------------------
def build_parser():
    p = argparse.ArgumentParser(
        description="一句话 → 选目标 → 反投影 → 抓取（A 版本：桌面固定臂）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="例：python3 pick_pipeline.py --instruction \"把杯子抓起来\" --dry-run")
    p.add_argument("--instruction", default="", help="自然语言指令，如「把杯子抓起来」")
    p.add_argument("--self-test", action="store_true", help="纯逻辑自检，不联网")

    g = p.add_argument_group("服务地址")
    g.add_argument("--api", default="http://127.0.0.1:5000", help="Flask 控制台")
    g.add_argument("--detector", default="http://127.0.0.1:8080", help="检测服务")

    g2 = p.add_argument_group("决策后端")
    g2.add_argument("--backend", choices=("vlm", "mock"), default="mock",
                    help="mock=不调模型（演练用），vlm=调 ollama/OpenAI 兼容接口")
    g2.add_argument("--model", default="qwen2.5vl:3b", help="模型名")
    g2.add_argument("--base-url", default="http://127.0.0.1:11434/v1",
                    help="OpenAI 兼容地址（ollama 默认 11434）")
    g2.add_argument("--model-timeout", type=float, default=120.0)
    g2.add_argument("--observe-timeout", type=float, default=8.0)

    g3 = p.add_argument_group("几何与动作")
    g3.add_argument("--grab-z", type=float, default=2.0,
                    help="抓取高度 cm（夹爪尖离桌面，深度相机给）")
    g3.add_argument("--hover", type=float, default=4.0, help="接近点比抓取点高多少 cm")
    g3.add_argument("--place-x", type=float, default=None, help="放置点 x cm（示教值）")
    g3.add_argument("--place-y", type=float, default=None, help="放置点 y cm（示教值）")
    g3.add_argument("--no-compensate", action="store_true",
                    help="不扣执行侧重力偏置 TIP_BIAS_XY/SAG_DZ。"
                         "只在「开着补偿够不到」时做对比，正常别加")
    g3.add_argument("--az-bias", type=float, default=0.0,
                    help="方位角补偿（度）。视觉方位角有固定偏小约 2.9°，"
                         "只影响 J1（横向 1.4cm 量级）。默认 0=不补，"
                         "想让 J1 对着尺子就传 2.9")
    g3.add_argument("--recipe", default="",
                    help="用示教姿态执行（C 方案）：J2~J5 取配方值、J1 由视觉方位角算。"
                         "绕开「IK 给的 J2/J4 与真实机构差几百 µs」这个已知问题。"
                         "配方名见 python3 action_primitives.py --list-recipes")

    g4 = p.add_argument_group("安全闸")
    g4.add_argument("--person-iou", type=float, default=PERSON_IOU_LIMIT,
                    help="人框与目标框 IoU 超过它就拒绝（默认 0.30）")
    g4.add_argument("--jaw-max-cm", type=float, default=JAW_MAX_OPEN_CM,
                    help="夹爪最大开口 cm，超过只警告（默认 5.0）")

    g5 = p.add_argument_group("执行")
    g5.add_argument("--yes", action="store_true", help="真的下发 JOINT")
    g5.add_argument("--dry-run", action="store_true", help="只打印计划，不发命令")
    g5.add_argument("--allow-clamp", action="store_true",
                    help="计划里有软限位裁剪也照发（默认拒绝）")
    g5.add_argument("--save-frame", default="", help="把决策用的那一帧存成 jpg")
    g5.add_argument("--dynamic-grip", action="store_true",
                    help="合爪改用电流判据（grip_until_contact）当场决定夹持脉宽，"
                         "不再用写死的 JAW_CLOSE=1550。需要 ACS712 已接好并标过零")
    g5.add_argument("--keep-proxy", action="store_true",
                    help="保留系统代理（默认强制直连；只有调云端 VLM 时才需要）")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)

    if args.self_test:
        return self_test()

    if not args.instruction.strip():
        print("缺 --instruction。例：--instruction \"把杯子抓起来\"")
        print("（只想验证逻辑：加 --self-test）")
        return 1
    if not args.yes and not args.dry_run:
        print("既没 --yes 也没 --dry-run。")
        print("  --dry-run  只打印计划，不动臂（建议先跑这个）")
        print("  --yes      真的动臂")
        return 1

    try:
        return run(args)
    except Fatal as e:
        print("\n❌ %s" % e, file=sys.stderr)
        return e.code
    except Rejected as e:
        print("\n⛔ 已拒绝执行：%s" % e, file=sys.stderr)
        return e.code
    except KeyboardInterrupt:
        print("\n已中断（Ctrl+C）。臂停在当前姿态，没发任何回初始位命令。",
              file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
