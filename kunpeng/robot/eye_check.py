#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""eye_check.py —— 眼在手（A+）点选链路自检：一颗固定目标点、几个不同姿态，看算出来的坐标重不重合。

怎么用（板 · openEuler 终端，Flask 已用 --eye-in-hand 起好、detector 已停）：
    python3 eye_check.py                # 打一次当前姿态下的定位结果
    python3 eye_check.py --save a.jpg   # 顺便存一张带标注的图
然后**换一个臂姿再跑一次**，把两次（或三次）的「工作台坐标」抄下来比一比。

------------------------------------------------------------------------------
README 三行法
① 用了什么语法：argparse 收参数；urllib.request 发 HTTP（不装 requests 也能跑）；
   cv2.findChessboardCorners + cv2.cornerSubPix 找亚像素角点、cv2.solvePnP +
   cv2.projectPoints 求「棋盘几何中心」落在哪个像素；np 拼物点；% 格式化；
   sys.exit(码) 把结果变成脚本能用的返回码。
② 为什么这么设计：眼在手链路最怕「算得出、但算错」—— 页面点得动、数字也漂亮，看不出问题。
   唯一能戳破它的判据是物理不变量：**同一个物理点，换个姿态必须还是同一个坐标**
   （棋盘固定在桌上不动，臂动了相机才动）。所以本工具只做这一件事，不去比绝对值 ——
   绝对值要卷尺基准，误差来源太多，判不出是链路错还是尺子歪。
③ 不这么写会怎样：只在一两个姿态下点几下、看到 x/y 差不多就以为通了；
   实际上手眼矩阵 X 的旋转部分错了、或抓取平面高度给错，**只在换姿态时才暴露**，
   固定姿态下的恒定偏差根本看不出来（这正是之前 FK 姿态镜像 bug 骗过 selftest 的原因）。
------------------------------------------------------------------------------
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np

try:
    import cv2
except ImportError:
    print("缺少 opencv：本工具要读相机找棋盘，请在有 cv2 的板上跑")
    sys.exit(1)

try:
    # Python 3 标准库；用它而不是 requests，是为了「板上不装包也能跑」
    from urllib.request import Request, urlopen
except ImportError:                                    # pragma: no cover
    from urllib2 import Request, urlopen                # Python 2 兜底（本项目用不到）


# ---------------------------------------------------------------------------
# 一、HTTP 小工具
# ---------------------------------------------------------------------------
def http_json(url, payload=None, timeout=8.0):
    """GET 或 POST 一个 JSON 接口，返回解析后的 dict。

    payload=None 走 GET；给了 dict 就走 POST + application/json。
    出错（连不上 / 返回不是 JSON）直接抛异常，由上层包成一句人话。
    """
    if payload is None:
        req = Request(url)
    else:
        body = json.dumps(payload).encode("utf-8")
        req = Request(url, data=body, headers={"Content-Type": "application/json"})
    with urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


# ---------------------------------------------------------------------------
# 二、棋盘格物点 + 几何中心
# ---------------------------------------------------------------------------
def chess_objp(cols, rows, square):
    """构造棋盘格内角点的物点坐标（单位：与 square 一致，这里用米）。

    物点就是「假设标定板放在原点、板面是 z=0 平面」时，每个内角点的三维坐标。
    第 i 个角点在 (列号*square, 行号*square, 0)。
    """
    objp = np.zeros((cols * rows, 3), np.float32)
    grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)    # [(0,0),(0,1)...,(8,5)]
    objp[:, :2] = grid * float(square)
    return objp


def board_center_pixel(corners, K, dist, cols, rows, square):
    """棋盘「几何中心」落在哪个像素上 —— 用它当固定目标点，比取角点均值更稳。

    大白话：先用 solvePnP 问出「板子相对相机在哪、怎么摆」，再把板子的正中心
           （物点坐标 ((cols-1)/2*S, (rows-1)/2*S, 0)）投影回画面，得到它的像素。
    规范定义：solvePnP 求 T_cam_board，再用 projectPoints 对板心做透视投影。
    为什么不用角点均值：透视投影下「像素均值」≠「三维中心的投影」，斜着看时会差几个像素，
    而我们要的是**同一个物理点**，必须严格是同一个三维点。
    """
    objp = chess_objp(cols, rows, square)
    ok, rvec, tvec = cv2.solvePnP(objp, corners, K, dist)
    if not ok:
        return None, "solvePnP 失败"
    center = np.array([[(cols - 1) * square / 2.0,
                        (rows - 1) * square / 2.0,
                        0.0]], dtype=np.float32)
    uv, _ = cv2.projectPoints(center, rvec, tvec, K, dist)
    return uv.reshape(2), ""


def detect_chessboard(cap, cols, rows, timeout_s):
    """反复读帧直到找到完整棋盘（9×6 内角点全在画面里），返回 (gray, corners, frame)。

    为什么要循环而不是读一帧就算：UVC 刚打开时前几帧常常是黑的/花的，
    只读一帧容易误判成「没有棋盘」，然后得出「链路坏了」的错误结论。
    """
    t0 = time.time()
    frame = None
    while time.time() - t0 < timeout_s:
        ret, f = cap.read()
        if not ret:
            time.sleep(0.05)
            continue
        frame = f
        gray = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCorners(gray, (cols, rows), flags)
        if found:
            # 亚像素精化：角点定位从「整像素」提到「0.1 像素」，温度/亮度抖动小一半
            try:
                crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), crit)
            except Exception:
                pass                    # 精化失败也能用，粗角点精度差一点而已
            return gray, corners, frame
        time.sleep(0.05)
    return None, None, frame


def board_guard(u, v, depth_cm, K, img_w, img_h, cols, rows, square):
    """检查棋盘离画面四条边还剩多少，快贴边就预警「还能挪近几厘米」。

    为什么需要这个：棋盘朝 J1 方向挪时，在画面里是**往下走**（相机俯角约 65°），
    中心每近 1cm 下移约 17px。一旦下边缘出画，findChessboardCorners 会因为
    凑不齐 9×6 个角点而整块返回 None，现象就是「等了 8 秒没找到完整棋盘格」——
    很容易被误判成相机坏了 / 屏幕没亮 / 光照不行，其实只是构图把角点裁掉了。

    原理（为什么用 depth 而不是棋盘坐标）：棋盘在画面里占多少像素，只由
    「相机到棋盘的距离」和焦距决定，针孔投影近似：
        像素宽 = fx × 棋盘物理宽 / 距离
    所以只要知道 depth（相机光心到棋盘中心），不用管棋盘摊在工作台的哪一处，
    都能反推它在图像里占多大一块、四条边各剩多少像素。

    「还能挪多少厘米」用经验换算：2026-11-12 实测 26.2→27.3cm 这一段，
    每挪近 1cm 中心下移约 17.3px（随俯角变化，只作量级参考）。
    """
    fx, fy = float(K[0, 0]), float(K[1, 1])
    bw = (cols - 1) * float(square) * 100.0      # 内角点跨度，cm
    bh = (rows - 1) * float(square) * 100.0
    if depth_cm <= 1.0:
        return
    w_px = fx * bw / float(depth_cm)
    h_px = fy * bh / float(depth_cm)
    left = u - w_px / 2.0
    right = img_w - (u + w_px / 2.0)
    top = v - h_px / 2.0
    bottom = img_h - (v + h_px / 2.0)
    px_per_cm = 17.3                              # 经验值，见 docstring
    side, margin = min((("下", bottom), ("上", top), ("左", left), ("右", right)),
                       key=lambda kv: kv[1])
    if margin >= 80:
        return                                    # 余量够，别废话
    print("  🔶 出画预警：棋盘在画面里约 %.0f×%.0f px，距%s边界只剩 %.0f px"
          % (w_px, h_px, side, margin))
    if side == "下":
        print("     朝 J1 方向每挪近 1cm，棋盘在画面里下移约 %.0f px ⇒ 只剩 %.1f cm 余量，"
              "再近就会裁掉角点（表现为「等 8 秒没找到棋盘格」）。"
              % (px_per_cm, margin / px_per_cm))
        print("     想看更近的点：把棋盘**垫高** 1~2cm（同时改 --plane-z），比一直朝 J1 挪有效。")
    else:
        print("     只剩约 %.1f cm 余量，再挪就会裁掉角点（表现为「等 8 秒没找到棋盘格」）。"
              % (margin / px_per_cm))


# ---------------------------------------------------------------------------
# 三、主流程
# ---------------------------------------------------------------------------
# 🔴 项目规矩：任何棋盘结论、任何舵机指令，都必须先交代「棋盘摆在哪、当前脉宽多少」。
# 这两个常量就是为了把「棋盘摆哪」从人脑记忆变成脚本自己说的话。
# 依据：C 姿态下有效纵深窗口 y∈[26.2, 28.0]（再近棋盘下边缘就出画，
# 再远则超出可达半径），实测见 docs/排障记录-手眼标定.md。
_BOARD_NEAR_CM = 26.0
_BOARD_FAR_CM = 28.0
# C 姿态（11-12 实测「定位可过」的姿态）：J1 略右转、J4 前倾 41°、J6 闭合。
_POSE_C = (1636, 1470, 2177, 1106, 1450, 1567)
_POSE_C_HINT = ("curl -s -X POST http://127.0.0.1:5000/api/cmd "
                "-H 'Content-Type: application/json' "
                "-d '{\"payload\":\"JOINT,%s\"}'"
                % ",".join(str(p) for p in _POSE_C))
# 找 arm_params.json 时优先看这里（--ik-dir，默认 ~/kunpeng_vision）
_IK_DIR_HINT = "~/kunpeng_vision"


def _load_ik():
    """拿回 arm_ik 的 Kinematics（找得到就返回，找不到返回 None）。

    为什么要 try：arm_ik.py 不在板上最小上传集里时不该让整个自检崩掉 ——
    「报出当前姿态」只是提示，不是主链路。
    """
    try:
        from arm_ik import ArmIK

        def _find_params():
            # arm_params.json 可能和本脚本同目录，也可能在 --ik-dir 指的那份
            # （板上最小上传集只放了 .py，模型文件留在 ~/kunpeng_vision）。
            cands = [os.path.join(os.path.dirname(os.path.abspath(__file__)), "arm_params.json"),
                     os.path.expanduser(os.path.join(_IK_DIR_HINT, "arm_params.json")),
                     os.path.expanduser("~/kunpeng_vision/arm_params.json")]
            for c in cands:
                if os.path.isfile(c):
                    return c
            return None

        p = _find_params()
        return ArmIK(p) if p else None
    except Exception:
        return None


def describe_pose(pulses):
    """把 6 路脉宽翻译成「爪尖在哪」+「夹爪朝向」，给棋盘定位提供前提。

    大白话：脉宽是一串没人看得懂的号码，这里用正运动学把它们换算成
            「爪尖离 J1 轴多少 cm、有多高」。
    规范定义：正运动学（Forward Kinematics, FK）—— 已知各关节角求末端位姿；
             本函数调用 `arm_ik.ArmIK.forward(to_angles(pulses))`。

    ⚠️ 只做**量级提示**，不当作验收依据。原因（2026-11-14 实测）：
      `forward()` 走的是理想几何，它给的爪尖半径 C 姿态只有 1.8~2.4 cm，
      而实测是 26 cm —— 模型对「绕 J1 的实际前伸」并没有建模对
      （L3=17 cm 被算成横向偏移，实际是沿臂前伸）。
      所以这里只报「相对初始位有没有往前/往上抬」，**绝对半径不可信**，
      判棋盘位置仍以「画面里能不能完整看到 9×6 角点」为准。
    返回 (位置描述串, 夹爪描述串)；读不到模型时返回 ("(未知)", "(未知)")。
    """
    ik = _load_ik()
    if ik is None or not pulses:
        return "(未知，arm_ik 没加载)", "(未知)"
    try:
        ang = ik.to_angles(pulses)
        fx, fy, fz = ik.forward(ang)
        txt = ("相对位姿 x=%+.1f y=%+.1f z=%.1f cm（方位角 %+.0f°，桌面上方 %.1f cm）"
               % (fx, fy, fz, math.degrees(math.atan2(fy, fx)), fz))
    except Exception as e:
        ang = None
        txt = "(FK 算不出来：%s)" % e
    # J4 = tilt（0=垂直向下）。to_angles 给的是**相对角**，tilt 要按
    # arm_ik.to_pulses 的逆运算还原：tilt = ang4 + 180 + ang3（别直接拿 ang4 当倾角）。
    try:
        tilt = ang[3] + 180.0 + ang[2]
        claw = "前倾 %.0f°（J4=%d）" % (tilt, pulses[3])
    except Exception:
        claw = "J5=%d J6=%d" % (pulses[4], pulses[5]) if len(pulses) >= 6 else "(未知)"
    return txt, claw


def _save_look(frame, save_path):
    """检测失败时把「相机现在看到什么」存下来。

    为什么必须有：失败信息只有一句「等 8 秒没找到棋盘」，看不出是
    「棋盘在画面外」还是「光照不行/屏幕没亮」—— 这就是之前连续误判的根源。
    存一张图，看一眼就知道该往哪挪。
    落地位置：给了 --save 就存到它的 .look.jpg；否则存到当前目录 look.jpg
    （与服务端 /api/ik/eye/frame 抓的图同名，方便统一翻）。
    """
    try:
        if save_path:
            p = os.path.splitext(save_path)[0] + ".look.jpg"
        else:
            p = "look.jpg"
        cv2.imwrite(p, frame)
        print("  已存「相机现在看到的画面」：%s —— 打开看一眼就知道棋盘该往哪挪" % p)
    except Exception as e:
        print("  （存失败画面失败：%s）" % e)


def do_hover(api, x, y, h, cur_pulses, tilt, bias=(0.0, 0.0), plane_z=0.0, tip_dx=0.0):
    """把夹爪尖移到 (x, y, h) 并真下发 JOINT —— 「指哪打哪」的物理验收。

    为什么这里自己调 arm_ik，而不是复用 /api/ik/eye/solve：
      · 那个端点把 z 钉死成「射线与抓取平面的交点」（平面 z=0 ⇒ z≡0），
        表达不了「同一个 x/y、但要抬高 h cm」这件事；
      · 想用 plane_z=h 绕过也不行 —— 射线打在更高的平面上，**交点会朝相机方向偏**，
        x/y 跟着变，测的就不是同一个点了。
    所以 x/y 取定位结果、z 单独给，本地逆解后只发一条 JOINT。

    bias=(dx, dy)：末端重力偏置（见 arm_ik.TIP_BIAS_XY）。IK 瞄的是
      (x−dx, y−dy) —— 因为爪尖实际会落在 FK 点 +bias 上，先扣掉才能真打到 (x,y)。
      z 不参与补偿：改 z 会让 J3/J4 大幅变动、bias 又失真。

    ⚠️ h 是**工作台系绝对高度**（不是「离棋盘面多少」）：脚本里 z 直接取 --hover，
      不叠加 --plane-z。所以真实净空 = h + SAG_DZ − plane_z，低于 0.5cm 直接拒发。

    tip_dx：**执行侧**的横向偏置补偿（cm）。和上面的 bias 不是一回事：
      · bias 补的是「重力把爪尖拽偏」，来源是舵机扛不住力矩；
      · tip_dx 补的是「J1 实际转角没走到指令值」，来源是 J1 的 map 比例/齿隙。
      2026-11-14 实测定性：同一块棋盘只改 J1 脉宽（1636→1580，56µs≈7.6°），
      画面里棋盘横移了 126px（相机确实转了），但爪尖实际落点比 IK 靶点又偏右
      约 1.26cm ⇒ 这是纯执行误差，加在定位里会把坐标一起带偏，所以只在这里扣。
      注意符号：实测爪尖偏 +x（偏右），要往左补 ⇒ 传负值。

    返回码：0 = 真的下发且固件回 OK；非 0 = 没动，调用方直接退出。
    """
    try:
        from arm_ik import ArmIK, SAG_DZ
    except Exception as e:
        print("  ⚠ 悬停跳过：import arm_ik 失败（%s）" % e)
        return 1
    ik = ArmIK()

    # ---- 安全闸门：先算真实净空，撞平面就别发 ----
    # 重力把爪尖往下拽，但拽多少**取决于力矩臂**、不是常数：
    #   · C 姿态（力矩臂 15cm）实测 3.88cm；
    #   · 臂伸到 r=29cm ⇒ 力矩臂 18.5cm ⇒ 约 4.8cm。
    # 旧版闸门只认常数 3.88，于是远端「5−3.88=1.12 放行、实际只剩 0.2cm」——
    # 这个漏洞在 30cm 那次实验里真实发生过（用户看到爪子擦着纸）。
    # 现在按力矩臂折算，并取「折算值 vs 常数」的较大者：
    # 宁可靠估得悲观一点（最多误拒），也不要放行之后蹭到台面。
    try:
        from arm_ik import BIAS_POSE_PULSES, BIAS_ARM_R
    except Exception:                    # 老版 arm_ik 没这两个常量，退回硬编码
        BIAS_POSE_PULSES, BIAS_ARM_R = (1636, 1470, 2177), 14.97
    r_tgt = math.hypot(float(x), float(y))
    arm_r = r_tgt - ik.L3 * math.sin(math.radians(float(tilt or 0.0)))
    arm_k = max(0.2, arm_r / float(BIAS_ARM_R))
    sag_est = abs(SAG_DZ) * arm_k
    sag_used = max(abs(SAG_DZ), sag_est)          # 保守：绝不低于 C 姿态常数
    # 重力把爪尖往下拽 |SAG_DZ|，所以「指令 z=3」实际只有 3−sag = 负值。
    # 之前连跑 7 次 --hover 3 压到手机屏，就是缺这道闸门。
    clearance = float(h) - sag_used - float(plane_z)
    if clearance < 0.5:
        print("")
        print("  🔴 拒绝下发：指令 z=%.2f cm，力矩臂 %.1f cm ⇒ 下垂估 %.2f cm（常数 %.2f）"
              % (float(h), arm_r, sag_used, -SAG_DZ))
        print("     减去棋盘面 %.2f cm ⇒ 净空 %.2f cm，会压到平面（手机/桌面）。"
              % (float(plane_z), clearance))
        print("     想要真实净空 3cm：--hover 写 %.1f（3 + 下垂 %.1f + 平面 %.1f）"
              % (3.0 + sag_used + float(plane_z), sag_used, float(plane_z)))
        return 2

    # ---- 姿态偏离预警：SAG_DZ 是 C 姿态的实测常数，臂伸远/收近就只是估算 ----
    # 开环舵机扛不住重力：C 姿态下 J2 少走 6.5°、J3 少走 2.2° ⇒ 爪尖低 3.88cm。
    # 下垂量 ∝ 力矩臂，力矩臂 ≈ 腕到 J1 轴的水平距离（tilt>0 时 = r − L3·sin(tilt)）。
    # 这一段不挡动作，只说明「上面那个净空是折算出来的、偏置常数也在外推」。
    d2 = d3 = 0
    try:
        d2 = int(cur_pulses[1]) - int(BIAS_POSE_PULSES[1])
        d3 = int(cur_pulses[2]) - int(BIAS_POSE_PULSES[2])
    except Exception:
        pass                             # 脉宽格式不对就不报偏离，别挡住动作
    if arm_k > 1.06 or abs(d2) > 100 or abs(d3) > 100:
        print("  🔶 已离偏置标定区：当前姿态 J2 %+d / J3 %+d us（C 姿态 %d/%d），"
              "目标力矩臂 %.1f cm（标定 %.1f）"
              % (d2, d3, BIAS_POSE_PULSES[1], BIAS_POSE_PULSES[2],
                 arm_r, float(BIAS_ARM_R)))
        print("     ⇒ 下垂按 %.2f cm 估（C 姿态实测 %.2f，按力矩臂折算）；"
              "偏置常数本身也在外推，横向可能差几毫米到 1cm"
              % (sag_used, -SAG_DZ))

    tx, ty = float(x) - bias[0], float(y) - bias[1]      # IK 瞄点 = 目标 − 偏置
    # 执行侧横向偏置：直接推 IK 的瞄点，不动 (x, y) 本身 ——
    # 这样打印出来的「目标」仍是定位结果，补偿量单独一行交代，不会被误当成定位变准了。
    if abs(float(tip_dx)) > 1e-9:
        tx += float(tip_dx)
    try:
        ang = ik.solve(tx, ty, z=float(h), tilt=float(tilt or 0.0))
        tgt = [int(round(p)) for p in ik.to_pulses(ang)]
    except Exception as e:
        print("  ⚠ 悬停逆解失败（x=%+.2f y=%+.2f z=%.2f）：%s" % (tx, ty, h, e))
        print("     把棋盘朝 J1 挪 1~2 cm 再跑；或加 --tilt 10 放宽可达半径")
        return 1

    print("")
    print("  悬停目标：x=%+.2f y=%+.2f z=%+.2f cm（夹爪尖停在该点正上方）" % (x, y, h))
    if bias != (0.0, 0.0):
        print("  末端偏置：已扣 dx=%+.2f dy=%+.2f cm ⇒ IK 实际瞄 (%+.2f, %+.2f)"
              % (bias[0], bias[1], tx - float(tip_dx), ty))
        print("             （开环舵机重力下垂，实测 C 姿态；--bias 0 可关闭）")
    else:
        print("  末端偏置：未启用（爪尖会比 IK 落点近 1cm、低 4cm）")
    if abs(float(tip_dx)) > 1e-9:
        print("  横向执行补偿：%+.2f cm（J1 转角与指令不符，实测偏右；最终 IK 瞄 x=%+.2f）"
              % (float(tip_dx), tx))
    # z 只有提示意义：爪尖真实高度 ≈ 指令 z − 下垂（不参与补偿，见 arm_ik 注释）
    print("  实际净空：约 %.2f cm（离棋盘面；指令 z=%.2f − 下垂 %.2f − 平面 %.2f）"
          % (clearance, float(h), sag_used, float(plane_z)))
    print("  当前脉宽：%s" % ",".join(str(p) for p in cur_pulses))
    print("  目标脉宽：%s" % ",".join(str(p) for p in tgt))

    payload = "JOINT,%s" % ",".join(str(p) for p in tgt)
    try:
        # /api/cmd 的字段名是 payload（写成 cmd 会被服务端挡成「空命令」）；
        # payload 内部必须逗号分隔 —— 固件用 strchr(cmd, ',') 切命令名与参数。
        r = http_json(api + "/api/cmd", {"payload": payload}, timeout=15.0)
    except Exception as e:
        print("  下发失败：%s" % e)
        return 1
    if not r.get("ok"):
        print("  下发失败：%s" % r.get("error"))
        return 1
    resp = (r.get("resp") or "").strip()
    print("  固件回：%r" % resp)
    if not resp.startswith("OK"):
        # 空串 = 串口旧 fd 静默失联（CH341 被复位后节点重建，写不报错、读一直空）
        print("  🔴 没回 OK —— 命令没被固件执行。空串先去 POST /api/open 重开串口再 PING")
        return 1

    print("")
    print("  ⇒ 等 2 秒让六路走完斜坡，再看夹爪尖相对棋盘中心偏在哪、偏几毫米：")
    print("     前后偏 ⇒ X 的平移方向或抓取平面高度；")
    print("     左右偏 ⇒ J1 的角度映射（map 折点，已知 10~15%）；")
    print("     绕着转 ⇒ X 的旋转。桌面抓取 ≤ 5 mm 就够用。")
    return 0


def main():
    ap = argparse.ArgumentParser(
        description="眼在手（A+）点选链路自检：固定棋盘 + 换姿态，看定位结果重不重合")
    ap.add_argument("--api", default="http://127.0.0.1:5000",
                    help="网页控制台地址（默认 http://127.0.0.1:5000）")
    ap.add_argument("--camera", type=int, default=0, help="相机编号（默认 0）")
    ap.add_argument("--intrinsics", default="~/kunpeng_vision/camera_intrinsics.json",
                    help="内参文件（默认 ~/kunpeng_vision/camera_intrinsics.json）")
    ap.add_argument("--cols", type=int, default=9, help="内角点列数（默认 9）")
    ap.add_argument("--rows", type=int, default=6, help="内角点行数（默认 6）")
    ap.add_argument("--square", type=float, default=None,
                    help="格子边长 m；默认读内参文件里的 square_m（本项目实测 0.0082）")
    ap.add_argument("--plane-z", type=float, default=None,
                    help="抓取平面高度 cm；不给就用服务端 /api/ik/eye 报的那个")
    ap.add_argument("--tilt", type=float, default=None,
                    help="夹爪前倾角(度)，0=垂直；不给就用服务端默认。"
                         "目标太远抓不到时试 5~20：可达上限会从 24.6 涨到 30cm 左右")
    ap.add_argument("--wait", type=float, default=8.0, help="等棋盘入画的最长秒数（默认 8）")
    ap.add_argument("--save", default="", help="存一张带标注的图到这个路径（可选）")
    ap.add_argument("--hover", type=float, default=None,
                    help="给了就真下发：把夹爪尖移到定位点正上方 N cm 处悬停。"
                         "这是「指哪打哪」的物理验收——看爪尖落在棋盘中心的哪一侧、差几毫米")
    ap.add_argument("--tip-dx", type=float, default=0.0, metavar="CM",
                    help="爪尖横向执行偏置(cm)：实测爪尖比 IK 落点偏 +x（偏右）时填负值补偿。"
                         "这是**执行侧**误差（J1 实际转角与指令不符），不是定位误差，"
                         "所以不打进定位、只在 do_hover 里扣；0 = 不补偿")
    ap.add_argument("--bias", default="auto",
                    help="末端水平偏置 dx,dy(cm)，下发前从靶点扣掉（重力下垂补偿）。"
                         "默认 auto = 用 arm_ik.TIP_BIAS_XY 的实测值 0.10,-0.95；"
                         "给 0 关闭；换结构或重标后要重新测")
    ap.add_argument("--radial-fix", default="auto",
                    help="定位的径向修正系数（补「棋盘被算近」的系统偏差）。"
                         "默认 auto = 用服务端 web_control._EYE_RADIAL_K（实测 1.091）；"
                         "给 0 关闭；也可以直接给数，如 1.091。"
                         "换相机安装角、棋盘远近或重标手眼 X 之后，这个系数必须重测")
    ap.add_argument("--allow-any-pose", action="store_true",
                    help="允许在**非 C 姿态**下跑（换姿互差实验用）。默认拒绝："
                         "当前脉宽不等于 C 姿态 1636,1470,2177,1106,1450,1567 就直接退出，"
                         "避免 --hover 跑完臂停新姿态后连跑 ⇒ 相机看不到棋盘。"
                         "换姿态实验请显式加本开关，出画风险自负")
    args = ap.parse_args()

    # --radial-fix：auto 就不传字段（服务端有自己的默认值，单一数据源）；
    # 0/off ⇒ 显式传 1.0 关掉；数字 ⇒ 用它覆盖。
    _rf = (args.radial_fix or "auto").strip().lower()
    if _rf in ("auto", ""):
        radial_fix = None
    elif _rf in ("0", "none", "off"):
        radial_fix = 1.0
    else:
        try:
            radial_fix = float(_rf)
        except ValueError:
            print("--radial-fix 要写 auto / 0 / 一个数字（如 1.091）")
            return 1
        if radial_fix <= 0:
            print("--radial-fix 必须是正数（1.0 = 不修正）")
            return 1

    # --bias 三种写法：auto（用实测值）/ 0（关闭）/ "dx,dy"（手动给）
    _b = (args.bias or "auto").strip().lower()
    if _b in ("auto", ""):
        try:
            from arm_ik import TIP_BIAS_XY
            bias = tuple(TIP_BIAS_XY)
        except Exception:
            bias = (0.0, 0.0)
    elif _b in ("0", "none", "off"):
        bias = (0.0, 0.0)
    else:
        try:
            _p = [float(t) for t in _b.replace(" ", "").split(",")]
            bias = (_p[0], _p[1]) if len(_p) == 2 else None
        except ValueError:
            bias = None
        if bias is None:
            print("--bias 要写成 dx,dy（两个数、单位 cm），例如 --bias 0.1,-0.95；或给 0 关闭")
            return 1

    # ---- ① 先问服务端：眼在手链路到底启没启、X 是不是占位 ----
    try:
        eye = http_json(args.api + "/api/ik/eye")
    except Exception as e:
        print("连不上网页控制台 %s：%s" % (args.api, e))
        print("  ⇒ 先把 web_control.py 起起来，并且带上 --eye-in-hand")
        return 1
    info = eye.get("eye") or {}
    if not info.get("enabled"):
        print("眼在手链路未启用：%s" % (info.get("error") or "启动时没加 --eye-in-hand"))
        return 1
    print("眼在手链路：已启用 | 数据标记：%s" % (info.get("flag") or "?"))
    if info.get("placeholder"):
        print("🔴 X 或 K 是占位数据 —— 这时候测出来的坐标不代表任何精度，先做实标再说")
        return 1
    # ---- ①b 抓取平面闸门：桌面棋盘一律 0，绝不吃服务端 grab_height ----
    # 🔴 服务端 /api/ik/eye 报的 plane_z 是 arm_params.json 的 grab_height（实测 2.0cm），
    #    那是「网页点选抓取时爪尖离桌的高度」，不是桌面基准。以前 --plane-z 不给就不发
    #    这个键 ⇒ 服务端按 2cm 平面求交 ⇒ 水平偏 0.87×2≈1.74cm，直接毁掉 0.30cm 精度；
    #    更坑的是成功行还打印「--plane-z 0.0」，纯属误导（11-14 定案）。
    # 现在改成 fail-safe：不给 ⇒ 强制 0.0 并**显式写进 payload**，服务端默认无权参与；
    # 手机屏（h=11mm）必须自己写 --plane-z 1.1。这样第 ⑤ 步和第 ⑥ 步永远用同一个数。
    plane_default = eye.get("plane_z")
    if args.plane_z is None:
        args.plane_z = 0.0
        print("抓取平面：0.0 cm（桌面默认；已忽略服务端报的 %s cm。"
              "手机屏 h=11mm 请显式 --plane-z 1.1）" % plane_default)
    else:
        print("抓取平面：%.1f cm（本次用 --plane-z 指定）" % args.plane_z)

    # ---- ② 当前 6 路脉宽：这是「相机此刻在哪」的唯一信息源 ----
    try:
        # 🔴 /api/state 在 web_control.py 里是 methods=["POST"]，只收 POST。
        # http_json 的规矩是 payload=None→GET、给 dict→POST，
        # 所以这里必须传 {}；写成 http_json(url) 会拿到 405 Method Not Allowed。
        st = http_json(args.api + "/api/state", {})
    except Exception as e:
        print("读 /api/state 失败：%s（串口没连？先 POST /api/open）" % e)
        return 1
    if not st.get("ok"):
        print("读 /api/state 失败：%s" % st.get("error"))
        return 1
    pulses = st["pulses"]
    print("当前脉宽：%s" % ",".join(str(p) for p in pulses))
    where_now, claw_now = describe_pose(pulses)
    print("当前爪尖：%s｜夹爪 %s" % (where_now, claw_now))

    # ---- ②b 姿态闸门：不是 C 姿态就别拍，把「人为记得回位」变成脚本强制 ----
    # 🔴 --hover 是真下发，跑完臂停在新位姿（约 13cm 高、J4 41°→46°）；不回 C 姿态
    #    就再跑一次，相机根本看不到棋盘 ⇒「等 8 秒没找到棋盘」，11-14 因此栽了 3 次。
    #    而且连跑会把上次结果当输入（闭环），连跑 7 次 y 极差 2.66cm。
    # 这一条闸门同时堵住「出画」和「连跑」两个坑：回位后脉宽一致才放行。
    # 容差 ±3µs 是给 PCA9685 量化/斜坡最后一拍留的，不影响拦截没停稳的情况。
    if len(pulses) == 6:
        _pose_dev = max(abs(int(a) - int(b)) for a, b in zip(pulses, _POSE_C))
    else:
        _pose_dev = 999
    if _pose_dev > 3 and not args.allow_any_pose:
        print("")
        print("🔴 当前不是 C 姿态，拒绝继续（相机看不到棋盘，拍了也是白拍）。")
        print("   当前脉宽：%s" % ",".join(str(p) for p in pulses))
        print("   C 姿态  ：%s" % ",".join(str(p) for p in _POSE_C))
        print("   ⇒ 先回位并静置 3 秒：")
        print("      %s" % _POSE_C_HINT)
        print("   确需换姿态做互差实验：显式加 --allow-any-pose（出画/连跑风险自负）")
        return 2
    if args.allow_any_pose:
        print("⚠️ 已放行非 C 姿态（--allow-any-pose）：出画/连跑风险自负")

    # ---- ③ 内参（K / dist / square）----
    kp = os.path.expanduser(args.intrinsics)
    with open(kp, "r", encoding="utf-8") as f:
        cal = json.load(f)
    K = np.array(cal["camera_matrix"], dtype=float)
    dist = np.array(cal.get("dist_coeffs") or [0, 0, 0, 0, 0], dtype=float)
    square = args.square if args.square else float(cal.get("square_m") or 0.0082)
    print("内参：fx=%.2f fy=%.2f 格子=%.4f m" % (K[0, 0], K[1, 1], square))

    # ---- ④ 相机找棋盘，算出板心的像素 ----
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print("打不开相机 %d（detector_service 还占着？先 pkill -f detector_service）" % args.camera)
        return 1
    # 🔴 必须把采集分辨率钉死成内参标定时的尺寸。
    # 反投影是「像素坐标 + K」一起用的：分辨率变了，fx/cx/cy 就全部对不上，
    # 而画面看起来完全正常、棋盘照样能检测到 —— 这是零提示的系统性错误。
    _sz = cal.get("image_size") or [640, 480]
    want_w, want_h = int(_sz[0]), int(_sz[1])
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, want_w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, want_h)
    got_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    got_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if (got_w, got_h) != (want_w, want_h):
        print("🔴 相机实际 %dx%d ≠ 内参标定 %dx%d：反投影会整体错，先改相机模式"
              % (got_w, got_h, want_w, want_h))
        cap.release()
        return 1
    print("采集分辨率：%dx%d（与内参一致）" % (got_w, got_h))
    try:
        gray, corners, frame = detect_chessboard(cap, args.cols, args.rows, args.wait)
        if corners is None:
            print("等了 %.1f 秒没找到完整 %d×%d 棋盘格（角点被裁掉？屏幕没亮？）"
                  % (args.wait, args.cols, args.rows))
            if claw_now:
                print("  棋盘该摆哪：棋盘要在相机的取景范围里，**整块 9×6 内角点全部可见**。")
                print("     经验窗口（C 姿态实测）：离 J1 轴水平 26~28 cm 那一段最稳；"
                      "再近棋盘下沿就出画。")
                print("  ⇒ 先送 C 姿态再看（J4 前倾 41°，此时爪尖在 %s）：" % where_now)
                print("     %s" % _POSE_C_HINT)
            if frame is not None:
                _save_look(frame, args.save)
            return 1
        uv, why = board_center_pixel(corners, K, dist, args.cols, args.rows, square)
        if uv is None:
            print("板心投影失败：%s" % why)
            return 1
        u, v = float(uv[0]), float(uv[1])
        print("棋盘中心像素：(%.1f, %.1f)" % (u, v))

        # ---- ⑤ 交给服务端算：像素 + 脉宽 → 3D 点 → 逆解 ----
        payload = {"u": u, "v": v, "pulses": pulses}
        if args.plane_z is not None:
            payload["plane_z"] = args.plane_z
        if args.tilt is not None:
            payload["tilt"] = args.tilt
        if radial_fix is not None:
            # None ⇒ 不发这个键，服务端用它自己的默认系数（单一数据源）
            payload["radial_fix"] = radial_fix
        try:
            r = http_json(args.api + "/api/ik/eye/solve", payload)
        except Exception as e:
            print("调用 /api/ik/eye/solve 失败：%s" % e)
            return 1
        if not r.get("ok"):
            # 服务端「逆解失败」时会把定位结果 x/y/z 一起带回来（见 api_ik_eye_solve）。
            # 而 A+ 验证的判据是「两个姿态算出的坐标重合」，与「这一点能不能抓到」
            # 完全无关 —— 所以这里不退出，否则一条有用的数据就白采了。
            if not all(k in r for k in ("x", "y", "z")):
                print("定位失败：%s" % r.get("error"))
                if r.get("pulses"):
                    print("  （服务端用的脉宽：%s）" % r["pulses"])
                return 1
            print("  ⚠ 逆解不可达（不影响坐标验证）：%s" % r.get("error"))
            print("     想让它可抓：把棋盘朝 J1 方向挪 2~3 cm，或把 J4 调小让相机俯得更陡")

        print("")
        print("  工作台坐标  x=%+.2f  y=%+.2f  z=%+.2f cm" % (r["x"], r["y"], r["z"]))
        # 前提交代：这份结论是在哪组脉宽下得出的 —— 换姿态复测时靠它对齐
        print("  前提：棋盘在桌面上（--plane-z %.1f），实测脉宽 %s"
              % (float(args.plane_z) if args.plane_z is not None else 0.0,
                 ",".join(str(p) for p in pulses)))
        # ---- 径向修正透明化：原始值、补了多少，都摆出来 ----
        # 不打印的话，同一块棋盘前后两次坐标差 3cm 完全没法追（这也是之前踩过的坑）。
        _k_used = r.get("radial_k")
        if _k_used is not None and abs(float(_k_used) - 1.0) > 1e-9:
            _r_fix = math.hypot(r["x"], r["y"])
            _r_raw = math.hypot(float(r.get("x_raw", r["x"])),
                                float(r.get("y_raw", r["y"])))
            print("  径向修正    ×%.3f：%.2f → %.2f cm（%+.2f）"
                  % (float(_k_used), _r_raw, _r_fix, _r_fix - _r_raw))
            print("              ↑ localize 用 FK 算「相机在哪」，可 FK 不知道舵机被重力"
                  "压下（J2 ≈6.5°）；")
            print("                相机骑在腕上（|t|≈16.7cm≈L3），被放大成坐标系统性偏近。")
            print("                系数来自 11-14 实测（定位 27.54 / 钢尺 30.04 ⇒ 1.091）；"
                  "单点值，换棋盘远近/重标 X 要重测。--radial-fix 0 可关掉对比")
            print("                ⚠️ 只修径向：横向另有约 1.4cm 稳定系统误差，别拿这个系数凑")
        if r.get("depth") is not None:
            print("  视线长度    %.2f cm（相机光心到该点）" % r["depth"])
        print("  水平距离    %.2f cm（距 J1 轴，方位角 %+.1f°）"
              % (math.hypot(r["x"], r["y"]), math.degrees(math.atan2(r["y"], r["x"]))))
        if r.get("clamped"):
            print("  ⚠ 逆解越界被裁的关节：%s" % ",".join(r["clamped"]))
        # ---- 出画预警：棋盘快贴画面边界时提前喊，别等它整块消失 ----
        # 必须有 depth 才能算像素尺寸；服务端没返回就跳过，不影响定位结果。
        if r.get("depth"):
            board_guard(u, v, float(r["depth"]), K, got_w, got_h,
                        args.cols, args.rows, square)
        print("")
        print("  ⇒ 换个臂姿（改 J1 和 J4 最有效）再跑一次，把 x/y 抄下来比：")
        print("     三组互差 ≤ 1 cm ⇒ 手眼链可信；某两个姿态差出几厘米 ⇒ X 的旋转有问题；")
        print("     全部一起偏同一方向 ⇒ 抓取平面高度（--plane-z）或桌面基准不对。")

        # ---- 可选：真下发，让爪尖真的走到这个点上方的悬停位 ----
        # 放在存图之前：万一串口没连、下发失败，图照样存下来，这次的定位数据不白采。
        # 注意相机还开着 —— 下发走的是 HTTP，不需要串口独占。
        if args.hover is not None:
            # 平面高度必须跟定位用的那个一致，否则安全闸门算错净空
            plane_used = float(args.plane_z) if args.plane_z is not None \
                else float(plane_default or 0.0)
            # 径向修正把点往外推了 ⇒ 力矩臂更大 ⇒ 下垂更多 ⇒ 同样 --hover 净空更低。
            # 提前喊一句，别等闸门报"真实净空 −0.4"才回头改参数。
            _shift = float(r.get("radial_shift") or 0.0)
            if _shift > 1.0 and float(args.hover) < 7.0:
                print("")
                print("  🔶 目标被径向推远 %.1f cm ⇒ 力矩臂变大、下垂更多："
                      "同样的 --hover %.1f 真实净空会比标定点低。"
                      % (_shift, float(args.hover)))
                print("     若被闸门拒发，它会直接算出该写多少（约 7.5）。")
            rc = do_hover(args.api, float(r["x"]), float(r["y"]),
                          args.hover, pulses, args.tilt, bias, plane_used,
                          tip_dx=args.tip_dx)
            if rc:
                return rc          # 相机由外层 finally 统一 release
            print("")
            print("  ⚠️ --hover 已真下发，臂停在新位姿。下次跑本脚本前必须回 C 姿态：")
            print("      %s" % _POSE_C_HINT)
            print("     回位后静置 ≥3 秒；脚本入场会再校验脉宽，不对会直接拒跑。")

        # ---- ⑥ 可选：存一张标注图，方便事后回看「点的是不是棋盘中心」----
        if args.save and frame is not None:
            dbg = frame.copy()
            cv2.drawChessboardCorners(dbg, (args.cols, args.rows), corners, True)
            cv2.drawMarker(dbg, (int(round(u)), int(round(v))), (0, 0, 255),
                           cv2.MARKER_CROSS, 18, 2)
            cv2.imwrite(args.save, dbg)
            print("  已存标注图 %s" % args.save)
        return 0
    finally:
        cap.release()


if __name__ == "__main__":
    sys.exit(main())
