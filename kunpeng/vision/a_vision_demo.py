#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
A 档视觉追踪 Demo —— 移动操作机器人项目（仅 A 档：0 元纯软件）
================================================================
用途：用你手上这颗 GF100 USB 摄像头，在 KunPeng Pro (openEuler aarch64) 上
      跑通「摄像头取流 → 颜色追踪 → 质心定位 → 可选 KCF 追踪」。
依赖：opencv + numpy（装法见下方「怎么跑」）

小白先懂三个概念（面试也能这么讲）：
1. 摄像头画面本质是一张张「图片/帧」，每秒 60 张就是 60fps。
   程序就是不停做：读一帧图 → 处理这帧图 → 显示这帧图 → 读下一帧。
2. 「颜色追踪」原理：把图从 RGB 转成 HSV（色相/饱和度/明度），
   按颜色范围把目标「抠」出来，再算目标的中心位置（质心）。
3. 「质心」就是目标中心点坐标 (cx, cy)。机器人知道这个点偏左/右/上/下，
   就能决定往哪边转 —— 这就是视觉伺服最底层的一步。
"""

import argparse
import sys
import cv2
import numpy as np

# ---------- 参数 ----------
# GF100 是 UVC 免驱摄像头，Linux 下一般是 /dev/video0（插多个会是 video1...）
# ssh 连板子时加 -X 才能弹窗口；没有显示器就加 --headless 只在终端打印质心。
parser = argparse.ArgumentParser(description="A档视觉追踪 Demo (GF100 + OpenCV)")
parser.add_argument("--cam", type=int, default=0, help="摄像头索引，默认 0（/dev/video0）")
parser.add_argument("--headless", action="store_true", help="无显示模式：不弹窗口，只打印质心坐标")
parser.add_argument("--width", type=int, default=640, help="画面宽，默认 640（USB2.0 带宽有限，别一上来就 1280）")
parser.add_argument("--height", type=int, default=480, help="画面高，默认 480")
args = parser.parse_args()

# ---------- 第一步：打开摄像头 ----------
cap = cv2.VideoCapture(args.cam)
if not cap.isOpened():
    print(f"[错误] 打不开摄像头索引 {args.cam}！")
    print("排查：1) 用 `ls /dev/video*` 看有几个摄像头；2) 改 --cam；3) 确认当前用户有访问权限。")
    sys.exit(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
print(f"[OK] 摄像头已打开，分辨率 {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")

# ---------- 第二步：设定要追踪的颜色（HSV 范围） ----------
# 默认追踪「红色」物体。HSV 比 RGB 更适合按颜色抠图（不受明暗影响）。
# 红色在 HSV 里跨 0 度，所以要两段范围（低红 + 高红）合并。
LOWER_RED1 = np.array([0, 120, 120])
UPPER_RED1 = np.array([10, 255, 255])
LOWER_RED2 = np.array([170, 120, 255])
UPPER_RED2 = np.array([180, 255, 255])

trail = []  # 质心历史，用来画运动轨迹

# ---------- 第三步：可选的 KCF 追踪（属于 opencv-contrib 扩展包） ----------
# KCF 是一种「相关滤波」追踪算法：第一帧你框住目标，后续帧它自动跟住。
# 没装扩展包就用不了，所以这里用 try 探测，不影响颜色追踪。
try:
    import cv2.legacy as cv2_legacy
    KCF_AVAILABLE = True
except ImportError:
    KCF_AVAILABLE = False
    if not args.headless:
        print("[提示] 没装 opencv-contrib，KCF 不可用；颜色追踪照常工作（headless 下 KCF 本就不可用）。")

kcf_tracker = None
use_kcf = False

def start_kcf(frame):
    """让用户框选目标并启动 KCF 追踪（需要图形窗口，headless 模式禁用）。"""
    global kcf_tracker, use_kcf
    bbox = cv2.selectROI("A档视觉追踪", frame, False)
    if bbox[2] > 0 and bbox[3] > 0:
        kcf_tracker = cv2_legacy.TrackerKCF_create()
        kcf_tracker.init(frame, bbox)
        use_kcf = True
        print("[KCF] 已启动，按 c 切回颜色追踪")

# ---------- 第四步：主循环 ----------
print("操作：默认颜色追踪 | 按 k 框选进入 KCF | 按 c 切回颜色 | 按 q 退出")
frame_count = 0
while True:
    ret, frame = cap.read()
    if not ret:
        print("[警告] 读不到帧，可能摄像头被占用或断连")
        break
    frame_count += 1

    if use_kcf and kcf_tracker is not None:
        # ---- KCF 模式 ----
        ok, bbox = kcf_tracker.update(frame)
        if ok:
            x, y, w, h = [int(v) for v in bbox]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, "KCF", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "KCF 跟丢", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    else:
        # ---- 颜色追踪模式 ----
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, LOWER_RED1, UPPER_RED1)
        mask2 = cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)
        mask = cv2.bitwise_or(mask1, mask2)
        # 去噪：先腐蚀后膨胀（开运算），去掉小斑点，避免误判
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 500:  # 面积太小当作噪点忽略
                (x, y, w, h) = cv2.boundingRect(c)
                cx = x + w // 2
                cy = y + h // 2
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 255, 255), -1)
                # 记录轨迹（最多保留 30 个点）
                trail.append((cx, cy))
                if len(trail) > 30:
                    trail.pop(0)
                # 画面中心十字，方便看目标偏哪
                h_f, w_f = frame.shape[:2]
                cv2.line(frame, (w_f // 2, 0), (w_f // 2, h_f), (255, 255, 255), 1)
                cv2.line(frame, (0, h_f // 2), (w_f, h_f // 2), (255, 255, 255), 1)
                # 方向提示（给机器人用的「决策依据」）
                dir_txt = ""
                if cx < w_f // 2 - 20:
                    dir_txt += "目标在左 "
                elif cx > w_f // 2 + 20:
                    dir_txt += "目标在右 "
                if cy < h_f // 2 - 20:
                    dir_txt += "上"
                elif cy > h_f // 2 + 20:
                    dir_txt += "下"
                if dir_txt:
                    cv2.putText(frame, dir_txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                # headless 模式：每 10 帧在终端打印一次质心，方便远程看
                if args.headless and frame_count % 10 == 0:
                    print(f"帧{frame_count} 质心=({cx},{cy}) 画面中心=({w_f//2},{h_f//2}) 方向={dir_txt or '居中'}")

    # 画轨迹
    for i in range(1, len(trail)):
        cv2.line(frame, trail[i - 1], trail[i], (0, 255, 255), 2)

    if not args.headless:
        cv2.imshow("A档视觉追踪", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('k') and KCF_AVAILABLE:
            start_kcf(frame)
        elif key == ord('c'):
            use_kcf = False
            kcf_tracker = None
    else:
        # headless：按帧数自动退出演示（避免卡死），或你用 Ctrl+C 中断
        if frame_count >= 300:
            print("[headless] 已跑 300 帧演示结束")
            break

cap.release()
if not args.headless:
    cv2.destroyAllWindows()
print("[完成] 已退出")
