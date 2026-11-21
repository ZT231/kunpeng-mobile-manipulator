# -*- coding: utf-8 -*-
"""DaBai DCW 第三轮：定性能用区、工作距离精度、最佳分辨率配置。

1) 深度/彩色可用分辨率全集
2) 当前 640x360 配置：中央 50% 有效占比、左/右缺深度到底缺多少
3) 工作距离(<0.8m)的区域中值重复性 vs 2m 处对比
4) 640x480(4:3) 配置：深度 FOV 是否被完整利用、对齐是否更优
5) AlignFilter 对齐到深度（depth 为主网格）作为备选方案
"""
import math
import sys

import cv2
import numpy as np

sys.path.insert(0, "/root/kunpeng_vision")

import pyorbbecsdk as ob
from orbbec.source import DepthCamera
from orbbec.depthproc import sample_region

RESULTS = []


def rec(name, ok, detail=""):
    RESULTS.append((name, bool(ok), detail))
    print("%s %-38s %s" % ("[PASS]" if ok else "[FAIL]", name, detail))


def info(name, detail=""):
    print("[INFO] %-38s %s" % (name, detail))


def section(t):
    print("\n===== %s =====" % t)


def uniq_res(pl):
    out = set()
    for i in range(pl.get_count()):
        pr = pl.get_stream_profile_by_index(i)
        if hasattr(pr, "as_video_stream_profile"):
            try:
                pr = pr.as_video_stream_profile()
            except Exception:
                pass
        if hasattr(pr, "get_width"):
            out.add((pr.get_width(), pr.get_height()))
    return sorted(out)


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


# ================================================================ 1
section("1. 可用分辨率全集")
p0 = ob.Pipeline()
dl = uniq_res(p0.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR))
cl = uniq_res(p0.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR))
info("depth 分辨率", ", ".join("%dx%d" % r for r in dl))
info("color 分辨率", ", ".join("%dx%d" % r for r in cl))
common = sorted(set(dl) & set(cl))
info("两者共有", ", ".join("%dx%d" % r for r in common))
rec("存在 4:3 的 640x480 双流", (640, 480) in common, "640x480 %s" % ("可用" if (640, 480) in common else "不可用"))
try:
    p0.stop()
except Exception:
    pass

# ================================================================ 2
section("2. 640x360 配置的可用区分析")
cam = DepthCamera()
r = cam.read()
c, d = r["color_bgr"], r["depth_mm"]
h, w = d.shape
valid = (d > 0)
rec("全图有效占比", True, "%.3f" % valid.mean())
cy0, cx0 = int(h * 0.25), int(w * 0.25)
rec("中央 50% 区域有效占比", True, "%.3f" % valid[cy0:h - cy0, cx0:w - cx0].mean())
rec("中央 20% 区域有效占比", True, "%.3f" % valid[int(h * .4):int(h * .6), int(w * .4):int(w * .6)].mean())
# 逐列有效占比（每 32 列）
colv = [valid[:, x:x + 32].mean() for x in range(0, w, 32)]
info("每32列有效占比", " ".join("%.2f" % v for v in colv))
rec("左边缘缺深度(前1/8列)", colv[0] < 0.2, "%.2f" % colv[0])
rec("右边缘基本可用(后1/8列)", colv[-1] > 0.5, "%.2f" % colv[-1])

# ================================================================ 3 工作距离精度
section("3. 工作距离(<0.8m) vs 远处(2m) 重复性")
frames = []
for i in range(25):
    rr = cam.read()
    if rr.get("ok"):
        frames.append(rr["depth_mm"].copy())
stack = np.stack(frames).astype(np.float32)
mean_d = stack.mean(axis=0)
allvalid = (stack > 0).all(axis=0)

def patch_std(lo, hi, size=40):
    mask = (mean_d >= lo) & (mean_d < hi) & allvalid
    ys, xs = np.where(mask)
    if len(ys) < size * size:
        return None, 0
    # 取靠中心的点
    order = np.argsort(np.abs(ys - mean_d.shape[0] // 2) + np.abs(xs - mean_d.shape[1] // 2))
    yy, xx = ys[order[0]], xs[order[0]]
    y1, y2 = max(0, yy - size // 2), min(mean_d.shape[0], yy + size // 2)
    x1, x2 = max(0, xx - size // 2), min(mean_d.shape[1], xx + size // 2)
    zs = []
    for f in frames:
        z, _ = sample_region(f, [x1, y1, x2, y2])
        zs.append(np.nan if z is None else z)
    zs = np.asarray(zs, float)
    zs = zs[~np.isnan(zs)]
    return float(np.std(zs)), len(zs), (x1, y1, x2, y2)

for lo, hi, label in ((250, 800, "工作距离 0.25~0.8m"),
                      (800, 1500, "中距 0.8~1.5m"),
                      (1800, 2600, "远距 1.8~2.6m")):
    std, n, box = patch_std(lo, hi)
    if std is None:
        info(label, "该距离无足够有效像素")
    else:
        info(label, "出生%.2f mm（区域中值，%d帧，box=%s）" % (std, n, box))
        rec("%s 区域重复性" % label, std < (3.0 if lo < 800 else 30.0), "%.2f mm" % std)

cam.close()

# ================================================================ 4
section("4. 640x480 (4:3) 全视野配置")
ok480 = False
try:
    cam4 = DepthCamera(color_w=640, color_h=480, depth_w=640, depth_h=480)
    info("实际 profile", "%dx%d / %dx%d" % (
        cam4.color.resolutionX, cam4.color.resolutionY,
        cam4.depth.resolutionX, cam4.depth.resolutionY))
    rr = cam4.read()
    if rr.get("ok") and rr.get("depth_mm") is not None and rr.get("color_bgr") is not None:
        c4, d4 = rr["color_bgr"], rr["depth_mm"]
        rec("640x480 双流出帧", True, "color=%s depth=%s" % (c4.shape, d4.shape))
        rec("640x480 全图有效占比", True, "%.3f" % (d4 > 0).mean())
        rec("640x480 中央50%有效", True, "%.3f" % (d4 > 0)[int(d4.shape[0]*.25):int(d4.shape[0]*.75),
                                                    int(d4.shape[1]*.25):int(d4.shape[1]*.75)].mean())
        base, best = shift_search(c4, d4)
        info("对齐平移搜索", "(0,0)=%.1f%%  最佳(dx=%d,dy=%d)=%.1f%%" % (100 * base, best[1], best[2], 100 * best[0]))
        cam4.close()
        ok480 = True
except Exception as e:
    info("640x480", "不可用: %r" % e)
    try:
        cam4.close()
    except Exception:
        pass

# ================================================================ 5
section("5. 对齐到深度（depth 为主网格）备选")
try:
    p = ob.Pipeline()
    cfg = ob.Config()
    dp = p.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR).get_default_video_stream_profile()
    cp = p.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR).get_default_video_stream_profile()
    cfg.enable_stream(dp)
    cfg.enable_stream(cp)
    p.start(cfg)
    af = ob.AlignFilter(align_to_stream=ob.OBStreamType.DEPTH_STREAM)
    n = 0
    cv = dk = None
    for i in range(30):
        fs = p.wait_for_frames(1000)
        if fs is None:
            continue
        fs = af.process(fs)
        dk = fs.get_depth_frame()
        cf = fs.get_color_frame()
        if dk is None or cf is None:
            continue
        d = np.frombuffer(dk.get_data(), dtype=np.uint16).reshape((dk.get_height(), dk.get_width()))
        raw = np.frombuffer(cf.get_data(), dtype=np.uint8)
        c = cv2.imdecode(raw, cv2.IMREAD_COLOR)
        if c is None:
            continue
        n += 1
        if n >= 6:
            break
    if n > 0:
        rec("对齐到深度可取帧", True, "%d 帧  depth=%s color=%s" % (n, d.shape, c.shape))
        rec("对齐到深度时全图有效占比", True, "%.3f" % (d > 0).mean())
        base, best = shift_search(c, d)
        info("对齐到深度 平移搜索", "(0,0)=%.1f%%  最佳(dx=%d,dy=%d)=%.1f%%" % (100 * base, best[1], best[2], 100 * best[0]))
    else:
        rec("对齐到深度可取帧", False, "0 帧")
    p.stop()
except Exception as e:
    rec("对齐到深度", False, repr(e))

# ================================================================ 汇总
section("第三轮汇总")
passed = sum(1 for _, ok, _ in RESULTS if ok)
failed = [r for r in RESULTS if not r[1]]
print("总计 %d 项：PASS %d / FAIL %d" % (len(RESULTS), passed, len(failed)))
for n, ok, det in failed:
    print("  [FAIL] %s  %s" % (n, det))
print("\n[RESULT] %s" % ("ALL_PASS" if not failed else "HAS_FAIL"))
