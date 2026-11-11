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


# ---------------------------------------------------------------------------
# 三、主流程
# ---------------------------------------------------------------------------
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
    ap.add_argument("--wait", type=float, default=8.0, help="等棋盘入画的最长秒数（默认 8）")
    ap.add_argument("--save", default="", help="存一张带标注的图到这个路径（可选）")
    args = ap.parse_args()

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
    plane_default = eye.get("plane_z")

    # ---- ② 当前 6 路脉宽：这是「相机此刻在哪」的唯一信息源 ----
    try:
        st = http_json(args.api + "/api/state")
    except Exception as e:
        print("读 /api/state 失败：%s（串口没连？先 POST /api/open）" % e)
        return 1
    if not st.get("ok"):
        print("读 /api/state 失败：%s" % st.get("error"))
        return 1
    pulses = st["pulses"]
    print("当前脉宽：%s" % ",".join(str(p) for p in pulses))

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
    try:
        gray, corners, frame = detect_chessboard(cap, args.cols, args.rows, args.wait)
        if corners is None:
            print("等了 %.1f 秒没找到完整 %d×%d 棋盘格（角点被裁掉？屏幕没亮？）"
                  % (args.wait, args.cols, args.rows))
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
        try:
            r = http_json(args.api + "/api/ik/eye/solve", payload)
        except Exception as e:
            print("调用 /api/ik/eye/solve 失败：%s" % e)
            return 1
        if not r.get("ok"):
            print("定位失败：%s" % r.get("error"))
            if r.get("pulses"):
                print("  （服务端用的脉宽：%s）" % r["pulses"])
            return 1

        print("")
        print("  工作台坐标  x=%+.2f  y=%+.2f  z=%+.2f cm" % (r["x"], r["y"], r["z"]))
        print("  视线长度    %.2f cm（相机光心到该点）" % r["depth"])
        if r.get("clamped"):
            print("  ⚠ 逆解越界被裁的关节：%s" % ",".join(r["clamped"]))
        print("")
        print("  ⇒ 换个臂姿（改 J1 和 J4 最有效）再跑一次，把 x/y 抄下来比：")
        print("     三组互差 ≤ 1 cm ⇒ 手眼链可信；某两个姿态差出几厘米 ⇒ X 的旋转有问题；")
        print("     全部一起偏同一方向 ⇒ 抓取平面高度（--plane-z）或桌面基准不对。")

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
