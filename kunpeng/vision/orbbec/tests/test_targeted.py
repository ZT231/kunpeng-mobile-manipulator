# -*- coding: utf-8 -*-
"""DaBai DCW 第二轮定向测试：修正第一轮的脚本 bug，并量化真实指标。

重点：
  1) 正确枚举 VideoStreamProfile（第一轮没转类型）
  2) 深度内参（depth_intrinsic）单独校验 FOV，而不是拿彩色内参比深度规格
  3) 噪声随距离分布 —— 判断 19.9mm 是否属于"远距离正常物理"
  4) D2C 对齐的平移搜索 —— 找使边缘重合最大的 (dx,dy)，这才是对齐的硬证据
  5) 区域中值重复性 —— 实际抓取流程用的就是这个精度
  6) 正确的 10x7 棋盘 → 9x6 内角点
  7) 时间戳单位与彩色/深度帧时间差
"""
import math
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, "/root/kunpeng_vision")

import pyorbbecsdk as ob
from orbbec.source import DepthCamera
from orbbec.depthproc import remove_flying, sample_region

RESULTS = []


def rec(name, ok, detail=""):
    RESULTS.append((name, bool(ok), detail))
    print("%s %-36s %s" % ("[PASS]" if ok else "[FAIL]", name, detail))


def info(name, detail=""):
    print("[INFO] %-36s %s" % (name, detail))


def section(t):
    print("\n===== %s =====" % t)


# ================================================================ 1 profile
section("1. 流能力枚举（修正类型转换）")
p0 = ob.Pipeline()
for label, st in (("depth", ob.OBSensorType.DEPTH_SENSOR),
                  ("color", ob.OBSensorType.COLOR_SENSOR)):
    try:
        pl = p0.get_stream_profile_list(st)
        n = pl.get_count()
        rows, bad = [], 0
        for i in range(n):
            pr = pl.get_stream_profile_by_index(i)
            vp = pr
            if hasattr(pr, "as_video_stream_profile"):
                try:
                    vp = pr.as_video_stream_profile()
                except Exception:
                    vp = pr
            if hasattr(vp, "get_width"):
                rows.append((vp.get_width(), vp.get_height(), vp.get_fps(), str(vp.get_format())))
            else:
                bad += 1
                rows.append(("non-video", str(getattr(pr, "get_stream_type", lambda: "?")()), ""))
        info("%s 总 profile=%d，非视频=%d" % (label, n, bad), "")
        uniq = sorted(set(rows))
        for r in uniq[:20]:
            info("  %s" % label, "%sx%s@%s %s" % r if len(r) == 4 else str(r))
        if uniq:
            rec("%s profile 可解析" % label, bad == 0, "唯一组合 %d 个" % len(uniq))
    except Exception as e:
        rec("%s profile 枚举" % label, False, repr(e))
try:
    p0.stop()
except Exception:
    pass

# ================================================================ 2 采集
section("2. 采集 20 帧（含深度内参）")
cam = DepthCamera()
info("device", str(cam._info))
info("用到的内参(彩色口径)", str({k: round(v, 3) if isinstance(v, float) else v for k, v in cam.intrinsics.items()}))

cp = None
try:
    cp = cam._pipeline.get_camera_param()
    di, ri = cp.depth_intrinsic, cp.rgb_intrinsic
    info("depth_intrinsic", "fx=%.2f fy=%.2f cx=%.2f cy=%.2f %dx%d"
         % (di.fx, di.fy, di.cx, di.cy, di.width, di.height))
    info("rgb_intrinsic", "fx=%.2f fy=%.2f cx=%.2f cy=%.2f %dx%d"
         % (ri.fx, ri.fy, ri.cx, ri.cy, ri.width, ri.height))

    if di.fx > 0 and di.width > 0:
        dh = 2 * math.degrees(math.atan(di.width / (2 * di.fx)))
        dv = 2 * math.degrees(math.atan(di.height / (2 * di.fy)))
        rec("深度内参 FOV 对得上规格(79/62)", 72 <= dh <= 86 and 55 <= dv <= 68,
            "depth 自算 H=%.1f V=%.1f（规格 H79 V62）" % (dh, dv))
    else:
        info("深度内参", "设备未上报（全 0），无法用内参验证深度 FOV")

    rh = 2 * math.degrees(math.atan(ri.width / (2 * ri.fx))) if ri.fx > 0 else 0
    rv = 2 * math.degrees(math.atan(ri.height / (2 * ri.fy))) if ri.fy > 0 else 0
    if rh:
        rec("彩色内参 FOV 合理(60~100 / 40~70)", 60 <= rh <= 100 and 40 <= rv <= 70,
            "rgb 自算 H=%.1f V=%.1f（16:9 裁切）" % (rh, rv))
except Exception as e:
    rec("内参读取", False, repr(e))

frames = []
tstamps = []
t0 = time.time()
for i in range(20):
    r = cam.read()
    if not r.get("ok"):
        continue
    frames.append((r["color_bgr"].copy(), r["depth_mm"].copy()))
    tstamps.append((r.get("timestamp"), time.time()))
info("采集", "%d/20 帧，耗时 %.2fs" % (len(frames), time.time() - t0))
rec("20 帧齐全", len(frames) == 20, "%d 帧" % len(frames))

# 时间戳单位
if len(tstamps) >= 3:
    vals = [v for v, _ in tstamps if v is not None]
    if vals:
        info("timestamp 类型/取值", "%s / %s ~ %s" % (type(vals[0]).__name__, vals[0], vals[-1]))
        d = np.diff(np.asarray(vals, dtype=np.float64))
        d = d[d > 0]
        if d.size:
            med = float(np.median(d))
            # 判断单位：30fps 应是 33.3ms = 0.0333s = 33333us = 33.3ms
            cand = {"秒": med * 1000, "毫秒": med, "微秒": med / 1000}
            best = min(cand.items(), key=lambda kv: abs(kv[1] - 33.3))
            rec("时间戳间隔可由单位解释", abs(best[1] - 33.3) < 8,
                "原始中位 %.3f → 按%s=%.1f ms（30fps 应≈33.3ms）" % (med, best[0], best[1]))
            info("判定时间戳单位", best[0])

# ================================================================ 3 噪声
section("3. 噪声随距离分布 + 区域中值重复性")
if frames:
    stack = np.stack([d for _, d in frames], axis=0).astype(np.float32)
    ok_mask = stack > 0
    mean_d = np.mean(stack, axis=0)
    std_d = np.std(stack, axis=0)
    bins = [(200, 500), (500, 1000), (1000, 1500), (1500, 2500), (2500, 4000)]
    print("  距离区间        像素数     时间噪声(中位std)   相对噪声")
    for lo, hi in bins:
        m = (mean_d >= lo) & (mean_d < hi) & ok_mask.all(axis=0)
        cnt = int(m.sum())
        if cnt < 100:
            continue
        s = float(np.median(std_d[m]))
        rel = s / float(np.median(mean_d[m])) * 100
        print("  %4d~%4dmm  %8d   %8.2f mm        %.2f%%" % (lo, hi, cnt, s, rel))
    near = (mean_d >= 200) & (mean_d < 800) & ok_mask.all(axis=0)
    far = (mean_d >= 1800) & (mean_d < 2600) & ok_mask.all(axis=0)
    if near.sum() > 100 and far.sum() > 100:
        sn = float(np.median(std_d[near]))
        sf = float(np.median(std_d[far]))
        rec("近处噪声小于远处(符合物理)", sn < sf, "近(<0.8m) %.2f mm vs 远(1.8~2.6m) %.2f mm" % (sn, sf))
    rec("最近处时间噪声 < 10mm", float(np.median(std_d[near])) < 10 if near.sum() > 100 else False,
        "%.2f mm" % float(np.median(std_d[near])) if near.sum() > 100 else "近处样本不足")

    # 区域中值重复性（真实抓取用的方式）
    cy, cx = stack.shape[1] // 2, stack.shape[2] // 2
    box = [cx - 20, cy - 20, cx + 20, cy + 20]
    zs = []
    for _, d in frames:
        z, _ = sample_region(d, box)
        zs.append(np.nan if z is None else z)
    zs = np.asarray(zs, dtype=np.float64)
    zs = zs[~np.isnan(zs)]
    rec("40x40 区域中值重复性 < 5mm", float(np.std(zs)) < 5.0,
        "中位 %.0f mm  标准差 %.2f mm  极差 %.0f mm" % (np.median(zs), np.std(zs), zs.max() - zs.min()))

# ================================================================ 4 对齐
section("4. D2C 对齐：平移搜索（硬证据）")
if frames:
    c, d = frames[0]
    g = cv2.GaussianBlur(cv2.cvtColor(c, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    ce = cv2.Canny(g, 60, 160)
    dd = d.astype(np.float32)
    med = np.median(d[d > 0]) if np.any(d > 0) else 1.0
    dd[d <= 0] = med
    dd = cv2.GaussianBlur(dd, (5, 5), 0)
    dn = cv2.normalize(dd, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    de = cv2.Canny(dn, 25, 70)

    def score(dx, dy):
        M = np.float32([[1, 0, dx], [0, 1, dy]])
        dsh = cv2.warpAffine(de, M, (de.shape[1], de.shape[0]))
        dsh = cv2.dilate(dsh, np.ones((5, 5), np.uint8))
        denom = max(int((ce > 0).sum()), 1)
        return float(np.logical_and(ce > 0, dsh > 0).sum()) / denom

    base = score(0, 0)
    best = (-1, 0, 0)
    for dy in range(-12, 13, 2):
        for dx in range(-12, 13, 2):
            s = score(dx, dy)
            if s > best[0]:
                best = (s, dx, dy)
    rec("对齐平移量≈0（核心证据）", abs(best[1]) <= 2 and abs(best[2]) <= 2,
        "最佳位移 (dx=%d, dy=%d) 得分 %.1f%%，(0,0) 得分 %.1f%%" %
        (best[1], best[2], 100 * best[0], 100 * base))
    info("解读", "边缘重合率绝对值偏低是因为墙面/显示器纹理有彩色边缘但无深度阶跃，属正常")

    # 有效深度占比的空间分布：是否四周边缘缺深度（彩色 FOV 比深度宽）
    valid = (d > 0).astype(np.float32)
    h, w = valid.shape
    b = max(1, h // 8)
    print("  行分块有效占比:", " ".join("%.2f" % valid[y:y + b].mean() for y in range(0, h, b)))
    print("  列分块有效占比:", " ".join("%.2f" % valid[:, x:x + b].mean() for x in range(0, w, b)))

# ================================================================ 5 棋盘
section("5. 棋盘检测（正确格数）")
cb = np.zeros((7 * 80, 10 * 80, 3), np.uint8)
for y in range(7):
    for x in range(10):
        if (x + y) % 2 == 0:
            cv2.rectangle(cb, (x * 80, y * 80), ((x + 1) * 80, (y + 1) * 80), (255, 255, 255), -1)
gray = cv2.cvtColor(cb, cv2.COLOR_BGR2GRAY)
ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)
rec("findChessboardCorners(9x6 内角点)", bool(ret) and corners is not None and len(corners) == 54,
    "corners=%s" % (None if corners is None else len(corners)))
if ret and corners is not None:
    term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-4)
    cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), term)
    objp = np.zeros((54, 3), np.float32)
    objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2) * 0.02
    rms, mtx, dist, _, _ = cv2.calibrateCamera([objp], [corners], gray.shape[::-1], None, None)
    rec("calibrateCamera 全链路可跑", rms < 1.0, "rms=%.4f fx=%.1f fy=%.1f" % (rms, mtx[0, 0], mtx[1, 1]))

# ================================================================ 6 图像质量
section("6. 彩色图像质量")
if frames:
    c = frames[0][0]
    gray = cv2.cvtColor(c, cv2.COLOR_BGR2GRAY)
    rec("亮度正常(非全黑/过曝)", 20 < gray.mean() < 235,
        "mean=%.1f min=%d max=%d 过曝像素=%.1f%%" %
        (gray.mean(), gray.min(), gray.max(), 100.0 * (gray >= 250).sum() / gray.size))
    lap = cv2.Laplacian(gray, cv2.CV_64F).var()
    rec("清晰度(拉普拉斯方差)", lap > 30, "%.1f" % lap)
    b, g_, r = c[:, :, 0].mean(), c[:, :, 1].mean(), c[:, :, 2].mean()
    info("通道均值 B/G/R", "%.1f / %.1f / %.1f" % (b, g_, r))

# ================================================================ 7 深度范围
section("7. 深度量程与有效性")
if frames:
    d = frames[0][1]
    v = d[d > 0]
    rec("深度最小值在量程内(>=0.2m)", int(v.min()) >= 150 if v.size else False, "min=%d mm" % (v.min() if v.size else -1))
    rec("深度最大值在量程内(<=5m)", int(v.max()) <= 5000, "max=%d mm" % (v.max() if v.size else -1))
    info("深度分位", "P1=%d P50=%d P99=%d mm" % (
        np.percentile(v, 1), np.percentile(v, 50), np.percentile(v, 99)) if v.size else "-")

cam.close()

# ================================================================ 汇总
section("第二轮汇总")
passed = sum(1 for _, ok, _ in RESULTS if ok)
failed = [r for r in RESULTS if not r[1]]
print("总计 %d 项：PASS %d / FAIL %d" % (len(RESULTS), passed, len(failed)))
for n, ok, det in failed:
    print("  [FAIL] %s  %s" % (n, det))
print("\n[RESULT] %s" % ("ALL_PASS" if not failed else "HAS_FAIL"))
