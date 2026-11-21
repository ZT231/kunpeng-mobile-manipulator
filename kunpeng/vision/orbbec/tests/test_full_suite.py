# -*- coding: utf-8 -*-
"""DaBai DCW 全功能测试（板上运行，独立于机械臂）。

覆盖：环境/库版本、流能力枚举、双流出帧、深度质量与时间噪声、D2C 对齐量化、
内参一致性、模块函数单测、生命周期重复开关、长时间稳定性、异常边界、
IR 流、demo 颜色预设、calib 棋盘检测算法。

运行：
    cd /root/kunpeng_vision
    export PYTHONPATH=/opt/pyorbbecsdk/install/lib:$PYTHONPATH
    python3 /root/_fulltest.py
"""
import json
import math
import os
import sys
import tempfile
import time

import cv2
import numpy as np

sys.path.insert(0, "/root/kunpeng_vision")

import pyorbbecsdk as ob
from orbbec.source import DepthCamera, DepthCameraError, Intrinsics
from orbbec.depthproc import remove_flying, sample_region, colorize
from orbbec.localize import pixel_to_camera

OUT = "/root/kunpeng_vision/dcw_test"
RESULTS = []


def rec(name, ok, detail=""):
    RESULTS.append((name, bool(ok), detail))
    print("%s %-34s %s" % ("[PASS]" if ok else "[FAIL]", name, detail))


def info(name, detail=""):
    print("[INFO] %-34s %s" % (name, detail))


def section(t):
    print("\n===== %s =====" % t)


def safe(fn, name, default=None):
    try:
        return fn()
    except Exception as e:
        rec(name, False, "异常: %r" % e)
        return default


# ---------------------------------------------------------------- A 环境
section("A. 环境与库版本")
info("python", sys.version.split()[0])
info("opencv", cv2.__version__)
info("numpy", np.__version__)
try:
    info("orbbecsdk", ob.get_version())
except Exception as e:
    info("orbbecsdk", "无 get_version() (%r)" % e)
info("module path", ob.__file__)
udev = "/etc/udev/rules.d/99-obsensor-libusb.rules"
rec("udev 规则已安装", os.path.isfile(udev),
    "%d bytes" % os.path.getsize(udev) if os.path.isfile(udev) else "缺失")

# ---------------------------------------------------------------- B 流能力
section("B. 流能力枚举")
p0 = ob.Pipeline()
prof_caps = {}
for label, st in (("depth", ob.OBSensorType.DEPTH_SENSOR),
                  ("color", ob.OBSensorType.COLOR_SENSOR)):
    try:
        pl = p0.get_stream_profile_list(st)
        n = pl.get_count()
        rows = []
        for i in range(min(n, 40)):
            pr = pl.get_stream_profile_by_index(i)
            rows.append((pr.get_width(), pr.get_height(), pr.get_fps(), str(pr.get_format())))
        prof_caps[label] = rows
        info("%s profile 数" % label, n)
        for r in rows[:12]:
            info("  %s" % label, "%dx%d@%d %s" % r)
    except Exception as e:
        rec("%s profile 枚举" % label, False, repr(e))

# 默认 profile
try:
    dp = p0.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR).get_default_video_stream_profile()
    cp = p0.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR).get_default_video_stream_profile()
    rec("默认深度 profile 640x360@30", (dp.get_width(), dp.get_height(), dp.get_fps()) == (640, 360, 30),
        "%dx%d@%d %s" % (dp.get_width(), dp.get_height(), dp.get_fps(), dp.get_format()))
    rec("默认彩色 profile 640x360@30", (cp.get_width(), cp.get_height(), cp.get_fps()) == (640, 360, 30),
        "%dx%d@%d %s" % (cp.get_width(), cp.get_height(), cp.get_fps(), cp.get_format()))
except Exception as e:
    rec("默认 profile", False, repr(e))
try:
    p0.stop()
except Exception:
    pass

# ---------------------------------------------------------------- C 主流程
section("C. 双流出帧（30 帧）")
cam = None
try:
    t0 = time.time()
    cam = DepthCamera()
    info("device", str(cam._info))
    info("内参", str(cam.intrinsics))
    info("registered / synced / warmup", "%s / %s / %s" % (cam.registered, cam.synced, cam.warmup_ok))
    rec("预热成功(彩色首帧可用)", cam.warmup_ok, "")
    rec("D2C 对齐已启用", cam.registered, "")
    rec("帧同步 flags 符合预期(不支持)", cam.synced is False, "Gemini E 不支持帧同步")

    n_depth = n_color = n_both = n_fail = 0
    ts_list = []
    shape_ref = None
    k_keys = {"ok", "frame_id", "timestamp", "color_bgr", "depth_mm", "K", "registered"}
    key_missing = 0
    t1 = time.time()
    frames = []
    for i in range(30):
        r = cam.read()
        if not r.get("ok"):
            n_fail += 1
            continue
        if k_keys - set(r.keys()):
            key_missing += 1
        if r.get("depth_mm") is not None:
            n_depth += 1
            shape_ref = r["depth_mm"].shape
        if r.get("color_bgr") is not None:
            n_color += 1
        if r.get("depth_mm") is not None and r.get("color_bgr") is not None:
            n_both += 1
        if r.get("timestamp") is not None:
            ts_list.append(r["timestamp"])
        if len(frames) < 8:
            frames.append((r["color_bgr"].copy(), r["depth_mm"].copy()))
    dt = time.time() - t1
    rec("30 帧全部成功", n_fail == 0 and n_both == 30,
        "ok=%d fail=%d depth=%d color=%d" % (n_both, n_fail, n_depth, n_color))
    rec("返回字段完整", key_missing == 0, "缺字段帧数=%d" % key_missing)
    rec("实测 FPS", True, "%.1f fps (30 帧 / %.2fs)" % (30.0 / max(dt, 1e-6), dt))
    if shape_ref:
        rec("深度尺寸 640x360", shape_ref == (360, 640), str(shape_ref))
    if frames:
        rec("深度 dtype uint16", frames[0][1].dtype == np.uint16, str(frames[0][1].dtype))
        rec("彩色 dtype uint8 / 3通道",
            frames[0][0].dtype == np.uint8 and frames[0][0].ndim == 3 and frames[0][0].shape[2] == 3,
            str(frames[0][0].shape))
    if len(ts_list) >= 3:
        diffs = np.diff(np.asarray(ts_list, dtype=np.float64))
        d = diffs[diffs > 0]
        rec("时间戳单调递增", bool(np.all(diffs >= 0)),
            "中位间隔 %.1f ms" % (np.median(d) * 1000 if d.size else -1))
except Exception as e:
    rec("DepthCamera 初始化", False, repr(e))
    cam = None

if cam is None:
    print("\n初始化失败，后续测试终止")
    sys.exit(2)

# ---------------------------------------------------------------- D 深度质量
section("D. 深度数据质量")
try:
    valid_ratios, medians, centers, zero_counts = [], [], [], []
    patches = []
    edge_scores = []
    for i in range(30):
        r = cam.read()
        if not r.get("ok") or r.get("depth_mm") is None:
            continue
        d = r["depth_mm"]
        c = r["color_bgr"]
        v = d[d > 0]
        valid_ratios.append(float(v.size) / float(d.size))
        zero_counts.append(int((d == 0).sum()))
        medians.append(float(np.median(v)) if v.size else 0.0)
        cy, cx = d.shape[0] // 2, d.shape[1] // 2
        centers.append(float(d[cy, cx]))
        patches.append(d[cy - 20:cy + 20, cx - 20:cx + 20].astype(np.float32))
        # 边缘对齐度：彩色 Canny 边缘里，有多少落在深度边缘 2px 内
        if i < 6 and c is not None:
            g = cv2.GaussianBlur(cv2.cvtColor(c, cv2.COLOR_BGR2GRAY), (5, 5), 0)
            ce = cv2.Canny(g, 60, 160)
            dd = d.astype(np.float32)
            med = np.median(d[d > 0]) if np.any(d > 0) else 1.0
            dd[d <= 0] = med
            dd = cv2.GaussianBlur(dd, (5, 5), 0)
            dn = cv2.normalize(dd, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            de = cv2.Canny(dn, 20, 60)
            de_d = cv2.dilate(de, np.ones((5, 5), np.uint8))
            denom = max(int((ce > 0).sum()), 1)
            edge_scores.append(float(np.logical_and(ce > 0, de_d > 0).sum()) / denom)

    rec("有效深度占比", 0.5 <= float(np.mean(valid_ratios)) <= 0.999,
        "均值 %.3f  min %.3f  max %.3f" % (np.mean(valid_ratios), np.min(valid_ratios), np.max(valid_ratios)))
    rec("深度中值合理(0.2~2.5m)", 200 <= float(np.mean(medians)) <= 2500,
        "均值 %.0f mm" % float(np.mean(medians)))
    rec("中心像素有值", all(x > 0 for x in centers),
        "均值 %.0f mm  波动 %d~%d mm" % (np.mean(centers), min(centers), max(centers)))

    stack = np.stack(patches, axis=0)
    per_pix_std = np.nanstd(stack, axis=0)
    per_pix_std[per_pix_std <= 0] = np.nan
    noise = float(np.nanmedian(per_pix_std))
    rec("静态区域时间噪声", noise < 15.0, "中位 %.2f mm（越小越好，<15 可用）" % noise)

    if edge_scores:
        rec("D2C 边缘对齐度", float(np.mean(edge_scores)) > 0.5,
            "彩色边缘落入深度边缘2px内 = %.1f%%" % (100 * float(np.mean(edge_scores))))

    # 飞行像素 / 去噪
    r = cam.read()
    d = r["depth_mm"]
    f = remove_flying(d)
    zeros_before = int((d == 0).sum())
    zeros_after = int((f == 0).sum())
    rec("remove_flying 生效", f.shape == d.shape and f.dtype == d.dtype,
        "0 值 %d → %d" % (zeros_before, zeros_after))

    # 保存对齐可视化：彩色边缘=红，深度边缘=绿，重合=黄
    c = r["color_bgr"]
    g = cv2.GaussianBlur(cv2.cvtColor(c, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    ce = cv2.Canny(g, 60, 160)
    dd = d.astype(np.float32)
    dd[d <= 0] = np.median(d[d > 0]) if np.any(d > 0) else 1.0
    dd = cv2.GaussianBlur(dd, (5, 5), 0)
    dn = cv2.normalize(dd, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    de = cv2.Canny(dn, 20, 60)
    ov = c.copy() * 0
    ov[ce > 0] = (0, 0, 255)
    ov[de > 0] = (0, 255, 0)
    ov[np.logical_and(ce > 0, de > 0)] = (0, 255, 255)
    os.makedirs(OUT, exist_ok=True)
    cv2.imwrite(os.path.join(OUT, "edge_overlay.png"), ov)
    cv2.imwrite(os.path.join(OUT, "color.png"), c)
    cv2.imwrite(os.path.join(OUT, "depth_viz.png"), colorize(remove_flying(d)))
    np.save(os.path.join(OUT, "depth_mm.npy"), d)
    info("产物", OUT)
except Exception as e:
    rec("深度质量分析", False, repr(e))

# ---------------------------------------------------------------- E 内参
section("E. 内参一致性与投影")
K = cam.intrinsics
try:
    rec("fx/fy 接近相等", abs(K["fx"] - K["fy"]) / K["fx"] < 0.02,
        "fx=%.2f fy=%.2f 差 %.2f%%" % (K["fx"], K["fy"], 100 * abs(K["fx"] - K["fy"]) / K["fx"]))
    rec("主点接近图像中心",
        abs(K["cx"] - K["w"] / 2.0) < 0.05 * K["w"] and abs(K["cy"] - K["h"] / 2.0) < 0.05 * K["h"],
        "cx=%.1f (w/2=%.1f)  cy=%.1f (h/2=%.1f)" % (K["cx"], K["w"] / 2.0, K["cy"], K["h"] / 2.0))
    hfov = 2 * math.degrees(math.atan(K["w"] / (2 * K["fx"])))
    vfov = 2 * math.degrees(math.atan(K["h"] / (2 * K["fy"])))
    rec("FOV 与规格同量级(depth 79/62)", 70 <= hfov <= 90 and 55 <= vfov <= 70,
        "由内参算得 H=%.1f V=%.1f deg" % (hfov, vfov))

    z = 1000.0
    p = pixel_to_camera(K["w"] / 2.0, K["h"] / 2.0, z, K)
    rec("主点反投影≈(0,0,z)", abs(p[0]) < 1.0 and abs(p[1]) < 1.0,
        "(%.2f, %.2f, %.2f) cm" % p)
    p2 = pixel_to_camera(K["w"] - 1, K["h"] - 1, z, K)
    rec("右边缘 x>0 / 下边缘 y>0", p2[0] > 0 and p2[1] > 0,
        "(%.2f, %.2f, %.2f) cm @1m" % p2)
    rec("深度 0 返回 None", pixel_to_camera(100, 100, 0, K) is None, "")

    # 横向视场宽度自洽：1m 处可视宽度 ≈ 2*z*tan(hfov/2)
    vis_w_mm = 1000.0 * K["w"] / K["fx"]
    info("1m 处可视宽度", "%.0f mm（标称 FOV 79° 应约 1642mm）" % vis_w_mm)

    # Intrinsics 单测
    tmp = tempfile.mktemp(suffix=".json")
    with open(tmp, "w", encoding="utf-8") as fp:
        json.dump({"camera_matrix": [[358.5, 0, 319.7], [0, 358.5, 179.0], [0, 0, 1]],
                   "w": 640, "h": 360}, fp)
    ij = Intrinsics.from_json(tmp)
    rec("Intrinsics.from_json 往返", abs(ij.fx - 358.5) < 1e-6 and ij.w == 640,
        "fx=%.2f w=%d" % (ij.fx, ij.w))
    os.remove(tmp)
    iF = Intrinsics.from_fov(640, 360, math.radians(79), math.radians(62))
    rec("Intrinsics.from_fov 标记 approx", iF.approx is True,
        "fx=%.1f fy=%.1f" % (iF.fx, iF.fy))
except Exception as e:
    rec("内参测试", False, repr(e))

# ---------------------------------------------------------------- F 模块单测
section("F. 深度处理模块单测")
try:
    r = cam.read()
    d = r["depth_mm"]
    z, ratio = sample_region(d, [d.shape[1] // 2 - 20, d.shape[0] // 2 - 20,
                                 d.shape[1] // 2 + 20, d.shape[0] // 2 + 20])
    rec("sample_region 中心区返回有效值", z is not None and z > 0,
        "z=%.0f mm ratio=%.2f" % (z or 0, ratio))
    z2, r2 = sample_region(d, [10, 10, 0, 0])
    rec("sample_region 非法框返回 None", z2 is None and r2 == 0.0, "")
    z3, r3 = sample_region(d, [-100, -100, 10000, 10000])
    rec("sample_region 越界裁剪", z3 is not None and 0 <= r3 <= 1.0,
        "z=%.0f ratio=%.2f" % (z3 or 0, r3))
    g = colorize(d)
    rec("colorize 输出 uint8 同尺寸", g is not None and g.dtype == np.uint8 and g.shape == d.shape,
        str(None if g is None else (g.shape, g.dtype)))
    rec("colorize(None) 安全", colorize(None) is None, "")
    rec("remove_flying(None) 安全", remove_flying(None) is None, "")
except Exception as e:
    rec("模块单测", False, repr(e))

# ---------------------------------------------------------------- G demo 逻辑
section("G. demo / 颜色预设逻辑")
try:
    from orbbec.cli import _color_range
    for cname in ("red", "red2", "blue", "green", "yellow"):
        try:
            lo, hi = _color_range(cname)
            rec("颜色预设 %s" % cname, lo.shape == (3,) and hi.shape == (3,), str(lo) + "~" + str(hi))
        except Exception as e:
            rec("颜色预设 %s" % cname, False, repr(e))
    # 合成红色图画走一遍 demo 检测链路
    img = np.zeros((360, 640, 3), np.uint8)
    img[100:200, 200:400] = (0, 0, 255)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lo, hi = _color_range("red")
    mask = cv2.inRange(hsv, lo, hi)
    mask = cv2.dilate(cv2.erode(mask, None, iterations=2), None, iterations=2)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    box = None
    if cnts:
        c = max(cnts, key=cv2.contourArea)
        box = cv2.boundingRect(c)
    rec("合成红色块可检出", box is not None and box[2] > 50,
        "box=%s" % (box,))
except Exception as e:
    rec("demo 逻辑", False, repr(e))

# ---------------------------------------------------------------- H calib 算法
section("H. calib 棋盘检测算法")
try:
    cb = np.zeros((600, 800, 3), np.uint8)
    for y in range(6):
        for x in range(9):
            if (x + y) % 2 == 0:
                cv2.rectangle(cb, (x * 80, y * 80), ((x + 1) * 80, (y + 1) * 80), (255, 255, 255), -1)
    gray = cv2.cvtColor(cb, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)
    rec("findChessboardCorners(9x6)", ret and corners is not None and len(corners) == 54,
        "corners=%s" % (None if corners is None else len(corners)))
    info("说明", "物理棋盘标定需真实棋盘，本次只验证算法链路")
except Exception as e:
    rec("calib 算法", False, repr(e))

# ---------------------------------------------------------------- I 生命周期
section("I. 生命周期（3 次开关）")
try:
    cam.close()
    ok_all = True
    detail = []
    for k in range(3):
        cam2 = DepthCamera()
        n = 0
        for i in range(5):
            rr = cam2.read()
            if rr.get("ok") and rr.get("color_bgr") is not None and rr.get("depth_mm") is not None:
                n += 1
        detail.append("第%d次 %d/5" % (k + 1, n))
        cam2.close()
        if n < 5:
            ok_all = False
    rec("3 次冷启动都能出完整帧", ok_all, " | ".join(detail))
    rr = cam2.read()
    rec("close 后 read 返回 ok=False", rr.get("ok") is False, str(rr.get("ok")))
except Exception as e:
    rec("生命周期", False, repr(e))

# ---------------------------------------------------------------- J 稳定性
section("J. 25 秒连续稳定流")
try:
    cam3 = DepthCamera()
    t0 = time.time()
    n_ok = n_bad = 0
    zvals = []
    fps_marks = []
    while time.time() - t0 < 25.0:
        r = cam3.read()
        if r.get("ok") and r.get("depth_mm") is not None:
            n_ok += 1
            cy, cx = r["depth_mm"].shape[0] // 2, r["depth_mm"].shape[1] // 2
            zvals.append(float(r["depth_mm"][cy, cx]))
        else:
            n_bad += 1
        if n_ok % 100 == 0 and n_ok:
            fps_marks.append(time.time() - t0)
    el = time.time() - t0
    rec("25s 无失败帧", n_bad == 0, "ok=%d bad=%d" % (n_ok, n_bad))
    rec("平均帧率", True, "%.1f fps（%d 帧 / %.1fs）" % (n_ok / el, n_ok, el))
    z = np.asarray([v for v in zvals if v > 0])
    if z.size:
        rec("中心深度长期稳定", float(np.std(z)) < 20.0,
            "中位 %.0f mm  标准差 %.1f mm  P5~P95 %.0f~%.0f mm" % (
                np.median(z), np.std(z), np.percentile(z, 5), np.percentile(z, 95)))
    cam3.close()
except Exception as e:
    rec("稳定性", False, repr(e))

# ---------------------------------------------------------------- K IR
section("K. IR 流（附加能力）")
try:
    has_ir = hasattr(ob.OBSensorType, "IR_SENSOR")
    info("OBSensorType.IR_SENSOR", str(has_ir))
    if has_ir:
        pi = ob.Pipeline()
        cfg = ob.Config()
        pl = pi.get_stream_profile_list(ob.OBSensorType.IR_SENSOR)
        pr = pl.get_default_video_stream_profile()
        cfg.enable_stream(pr)
        pi.start(cfg)
        got = 0
        for i in range(10):
            fs = pi.wait_for_frames(1000)
            if fs is None:
                continue
            f = fs.get_ir_frame()
            if f is not None:
                got += 1
        rec("IR 流可取帧", got > 0, "%d/10 帧  %dx%d" % (got, pr.get_width(), pr.get_height()) if got else "0/10")
        pi.stop()
    else:
        info("IR", "该 SDK 版本无 IR_SENSOR 枚举，跳过")
except Exception as e:
    rec("IR 流", False, repr(e))

# ---------------------------------------------------------------- 汇总
section("汇总")
passed = sum(1 for _, ok, _ in RESULTS if ok)
failed = [r for r in RESULTS if not r[1]]
print("总计 %d 项：PASS %d / FAIL %d" % (len(RESULTS), passed, len(failed)))
for name, ok, detail in failed:
    print("  [FAIL] %s  %s" % (name, detail))
print("\n[RESULT] %s" % ("ALL_PASS" if not failed else "HAS_FAIL"))
