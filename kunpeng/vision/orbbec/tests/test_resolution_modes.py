# -*- coding: utf-8 -*-
"""第四轮：分进程复测 640x480 / 15fps / 对齐到深度 / 量化台阶。"""
import sys

import cv2
import numpy as np

sys.path.insert(0, "/root/kunpeng_vision")
import pyorbbecsdk as ob
from orbbec.source import DepthCamera

mode = sys.argv[1] if len(sys.argv) > 1 else "default"


def shift_search(color, depth, rng=12):
    g = cv2.GaussianBlur(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    ce = cv2.Canny(g, 60, 160)
    dd = depth.astype(np.float32)
    med = np.median(depth[depth > 0]) if np.any(depth > 0) else 1.0
    dd[depth <= 0] = med
    dd = cv2.GaussianBlur(dd, (5, 5), 0)
    dn = cv2.normalize(dd, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    de = cv2.Canny(dn, 25, 70)

    def sc(dx, dy):
        M = np.float32([[1, 0, dx], [0, 1, dy]])
        ds = cv2.warpAffine(de, M, (de.shape[1], de.shape[0]))
        ds = cv2.dilate(ds, np.ones((5, 5), np.uint8))
        return float(np.logical_and(ce > 0, ds > 0).sum()) / max(int((ce > 0).sum()), 1)

    base = sc(0, 0)
    best = (-1, 0, 0)
    for dy in range(-rng, rng + 1, 2):
        for dx in range(-rng, rng + 1, 2):
            s = sc(dx, dy)
            if s > best[0]:
                best = (s, dx, dy)
    return base, best


print("MODE =", mode)
try:
    if mode == "480_30":
        cam = DepthCamera(color_w=640, color_h=480, depth_w=640, depth_h=480, fps=30)
    elif mode == "480_15":
        cam = DepthCamera(color_w=640, color_h=480, depth_w=640, depth_h=480, fps=15)
    else:
        cam = DepthCamera()

    print("profile: color %dx%d@%d  depth %dx%d@%d" % (
        cam.color.resolutionX, cam.color.resolutionY, cam.color.fps,
        cam.depth.resolutionX, cam.depth.resolutionY, cam.depth.fps))

    n = 0
    frames = []
    for i in range(12):
        r = cam.read()
        if r.get("ok") and r.get("color_bgr") is not None and r.get("depth_mm") is not None:
            n += 1
            if len(frames) < 3:
                frames.append((r["color_bgr"].copy(), r["depth_mm"].copy()))
    print("可用帧: %d/12" % n)

    if frames:
        c, d = frames[0]
        print("color=%s depth=%s" % (c.shape, d.shape))
        print("全图有效占比: %.3f" % (d > 0).mean())
        hh, ww = d.shape
        print("中央50%%有效: %.3f" % (d > 0)[int(hh*.25):int(hh*.75), int(ww*.25):int(ww*.75)].mean())
        base, best = shift_search(c, d)
        print("对齐平移: (0,0)=%.1f%%  最佳(dx=%d,dy=%d)=%.1f%%" % (100*base, best[1], best[2], 100*best[0]))
        b = max(1, ww // 20)
        print("每%d列有效: %s" % (b, " ".join("%.2f" % (d > 0)[:, x:x+b].mean() for x in range(0, ww, b))))

    if mode == "default" and frames:
        # 量化台阶
        vals = []
        for i in range(60):
            r = cam.read()
            if r.get("ok") and r.get("depth_mm") is not None:
                dd = r["depth_mm"]
                vals.append(int(dd[dd.shape[0]//2, dd.shape[1]//2]))
        v = np.asarray([x for x in vals if x > 0])
        uq = sorted(set(v.tolist()))
        print("中心深度 60 帧: 唯一值 %d 个 -> %s" % (len(uq), uq[:12]))
        if len(uq) > 1:
            diffs = np.diff(uq)
            print("量化台阶: 中位 %.0f mm  最小 %d mm" % (np.median(diffs), diffs.min()))
        print("极差 %d mm  标准差 %.1f mm" % (v.max()-v.min(), v.std()))

    cam.close()
    print("RESULT OK")
except Exception as e:
    print("RESULT ERROR: %r" % (e,))
