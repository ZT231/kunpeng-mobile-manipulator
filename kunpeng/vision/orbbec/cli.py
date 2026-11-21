# -*- coding: utf-8 -*-
"""DaBai DCW 命令行入口：探针 / 抓帧 / 预览 / 检测测距 / 内参标定。

运行（在 kunpeng 项目根目录，保证 kunpeng 包可见）：
    python -m kunpeng.vision.orbbec.cli probe
    python -m kunpeng.vision.orbbec.cli snap --out ./dcw_snap
    python -m kunpeng.vision.orbbec.cli stream --seconds 10
    python -m kunpeng.vision.orbbec.cli demo --color red
    python -m kunpeng.vision.orbbec.cli calib --cols 9 --rows 6 --square 0.02 --count 15
"""
import argparse
import os
import time

import cv2
import numpy as np

try:
    from .source import DepthCamera, DepthCameraError
    from .depthproc import remove_flying, sample_region, colorize
    from .localize import pixel_to_camera
except ImportError:  # 直接 python cli.py 运行时走这里
    from source import DepthCamera, DepthCameraError
    from depthproc import remove_flying, sample_region, colorize
    from localize import pixel_to_camera


# ---------------- 工具 ----------------

def _fmt_k(K):
    return ("fx=%.1f fy=%.1f cx=%.1f cy=%.1f %dx%d %s"
            % (K["fx"], K["fy"], K["cx"], K["cy"], K["w"], K["h"],
               "[approx-FOV]" if K.get("approx") else "[json]"))


def _apply_opts(cam, args):
    """处理 snap/stream 共用的 --laser / --range 选项。"""
    laser = getattr(args, "laser", None)
    if laser:
        cam.set_laser(laser == "on")
    rng = getattr(args, "range", None)
    if rng:
        lo, hi = (int(v) for v in rng.split(","))
        cam.set_depth_range(lo, hi)


def _coverage(depth):
    """返回 (全图有效占比, 中央20%有效占比)。"""
    if depth is None:
        return 0.0, 0.0
    h, w = depth.shape[:2]
    return (float((depth > 0).mean()),
            float((depth[int(h * .4):int(h * .6), int(w * .4):int(w * .6)] > 0).mean()))


def _save_colorized(path, depth, max_mm=3000):
    g = colorize(depth, max_mm)
    cv2.imwrite(path, g)


# ---------------- 子命令 ----------------

def cmd_probe(args):
    cam = DepthCamera(intrinsics_path=args.intrinsics)
    try:
        print("[device]", cam._info)
        vm_d = cam.depth.get_video_mode()
        vm_c = cam.color.get_video_mode()
        print("[depth mode] %dx%d@%d fmt=%d" %
              (vm_d.resolutionX, vm_d.resolutionY, vm_d.fps, vm_d.pixelFormat))
        print("[color mode] %dx%d@%d fmt=%d" %
              (vm_c.resolutionX, vm_c.resolutionY, vm_c.fps, vm_c.pixelFormat))
        print("[fov deg] depth H=%.1f V=%.1f | color H=%.1f V=%.1f" %
              (np.rad2deg(cam.depth.get_horizontal_fov()),
               np.rad2deg(cam.depth.get_vertical_fov()),
               np.rad2deg(cam.color.get_horizontal_fov()),
               np.rad2deg(cam.color.get_vertical_fov())))
        print("[registered]", cam.registered, "[synced]", cam.synced)
        print("[intrinsics]", _fmt_k(cam.intrinsics))
        print("[props]", cam.props)
    finally:
        cam.close()


def cmd_tune(args):
    """查看/调整设备侧参数，并立刻用 10 帧验证效果。"""
    cam = DepthCamera(intrinsics_path=args.intrinsics)
    try:
        print("[before]", cam.get_props())
        if args.laser:
            print("[set laser=%s] -> %s" % (args.laser, cam.set_laser(args.laser == "on")))
        if args.range:
            lo, hi = (int(v) for v in args.range.split(","))
            print("[set range=%d,%d hole_fill=%s] -> %s" % (
                lo, hi, args.hole_fill, cam.set_depth_range(lo, hi, args.hole_fill)))
        print("[after]", cam.get_props())

        covs, c20s, n = [], [], 0
        for _ in range(10):
            r = cam.read(with_color=False)
            if not r["ok"]:
                continue
            c, c20 = _coverage(r["depth_mm"])
            covs.append(c)
            c20s.append(c20)
            n += 1
        if n:
            print("[coverage] 全图 %.3f  中央20%% %.3f  (%d 帧)" % (
                sum(covs) / n, sum(c20s) / n, n))
        else:
            print("[coverage] 读不到帧（该分辨率可能不支持 D2C 对齐）")
    finally:
        cam.close()


def cmd_snap(args):
    cam = DepthCamera(intrinsics_path=args.intrinsics)
    _apply_opts(cam, args)
    os.makedirs(args.out, exist_ok=True)
    try:
        # 多读几帧等曝光/对齐稳定（取最后一帧，避免拿到属性刚改时的旧数据）
        r = {"ok": False}
        for _ in range(10):
            rr = cam.read()
            if rr["ok"]:
                r = rr
        if not r["ok"]:
            raise DepthCameraError("读不到帧")
        color = r["color_bgr"]
        depth = r["depth_mm"]
        color_path = os.path.join(args.out, "color.png")
        depth_npy = os.path.join(args.out, "depth_mm.npy")
        depth_png = os.path.join(args.out, "depth_viz.png")
        cv2.imwrite(color_path, color)
        np.save(depth_npy, depth)
        _save_colorized(depth_png, depth)
        z, ratio = sample_region(depth, [0, 0, color.shape[1], color.shape[0]])
        print("[ok] 已保存 color.png / depth_mm.npy / depth_viz.png")
        print("[sample] 全图有效深度占比=%.2f 中心区域中值=%.0f mm" % (ratio, z or 0))
    finally:
        cam.close()


def cmd_stream(args):
    cam = DepthCamera(intrinsics_path=args.intrinsics)
    _apply_opts(cam, args)
    t0 = time.time()
    n = 0
    try:
        while time.time() - t0 < args.seconds:
            # drop_stale=1：打印慢的时候先丢掉陈旧帧，避免延迟越积越大
            r = cam.read(with_color=not args.no_color, drop_stale=1)
            if not r["ok"]:
                continue
            n += 1
            depth = r["depth_mm"]
            color = r["color_bgr"]
            if color is not None:
                cyx, cxx = color.shape[0] // 2, color.shape[1] // 2
            else:
                cyx, cxx = depth.shape[0] // 2, depth.shape[1] // 2
            box = [cxx - 10, cyx - 10, cxx + 10, cyx + 10]
            z, ratio = sample_region(depth, box)
            print("frame=%d center=%s valid=%.2f cov=%.2f" %
                  (r["frame_id"],
                   ("%.0f mm" % z) if z else "none", ratio, _coverage(depth)[0]))
    finally:
        cam.close()
    print("[ok] %.1f FPS" % (n / max(time.time() - t0, 1e-3)))


def cmd_demo(args):
    cam = DepthCamera(intrinsics_path=args.intrinsics)
    hsv_lo, hsv_hi = _color_range(args.color)
    K = cam.intrinsics
    t0 = time.time()
    n = 0
    try:
        while time.time() - t0 < args.seconds:
            r = cam.read()
            if not r["ok"]:
                continue
            n += 1
            color = r["color_bgr"]
            depth = remove_flying(r["depth_mm"])
            hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, hsv_lo, hsv_hi)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
            if cnts:
                c = max(cnts, key=cv2.contourArea)
                x1, y1, w, h = cv2.boundingRect(c)
                if w * h > 400:
                    x2, y2 = x1 + w, y1 + h
                    z_mm, ratio = sample_region(depth, [x1, y1, x2, y2])
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    pt = pixel_to_camera(cx, cy, z_mm, K) if z_mm else None
                    print("frame=%d box=[%d,%d,%d,%d] valid=%.2f %s" %
                          (r["frame_id"], x1, y1, x2, y2, ratio,
                           ("xyz=(%.1f,%.1f,%.1f)cm" % pt) if pt else "depth:none"))
            # 停几秒没检测到也打一条，避免以为卡死
            if n % 15 == 0:
                print("frame=%d 未检测到目标（调 --color 或靠近物体）" % r["frame_id"])
    finally:
        cam.close()


def cmd_calib(args):
    cam = DepthCamera(intrinsics_path=None)  # 内参此刻未知，用彩色流标定
    objp = np.zeros((args.cols * args.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square
    objpoints, imgpoints = [], []
    got = 0
    try:
        while got < args.count:
            r = cam.read()
            if not r["ok"]:
                continue
            gray = cv2.cvtColor(r["color_bgr"], cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, (args.cols, args.rows), None)
            if ret:
                term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-4)
                cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), term)
                objpoints.append(objp)
                imgpoints.append(corners)
                got += 1
                print("[calib] %d/%d" % (got, args.count))
            time.sleep(0.5)
        if got == 0:
            raise DepthCameraError("一张棋盘都没找到，检查 --cols/--rows/--square 和光照")
        rms, mtx, dist, _, _ = cv2.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None)
        h, w = gray.shape
        out = {
            "camera_matrix": mtx.tolist(),
            "dist_coeffs": dist.ravel().tolist(),
            "rms": float(rms),
            "square_m": args.square,
            "w": w, "h": h,
        }
        with open(args.out, "w", encoding="utf-8") as f:
            import json
            json.dump(out, f, indent=2)
        print("[ok] 内参已写 %s rms=%.4f" % (args.out, rms))
    finally:
        cam.close()


# ---------------- 颜色预设 ----------------

def _color_range(name):
    # OpenCV HSV: H[0,180] S[0,255] V[0,255]
    presets = {
        "red": [(0, 100, 80), (10, 255, 255)],       # 红分两段，这里取低段
        "red2": [(170, 100, 80), (180, 255, 255)],   # 红的高段
        "blue": [(100, 100, 60), (130, 255, 255)],
        "green": [(40, 60, 60), (80, 255, 255)],
        "yellow": [(20, 100, 100), (35, 255, 255)],
    }
    if name not in presets:
        raise SystemExit("未知颜色 %s，可选：%s" % (name, ",".join(presets)))
    lo, hi = presets[name]
    return (np.array(lo, np.uint8), np.array(hi, np.uint8))


# ---------------- 入口 ----------------

def main():
    ap = argparse.ArgumentParser(description="DaBai DCW 深度相机工具")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("probe", help="打印设备/流/内参")
    p.add_argument("--intrinsics", default=None)
    p.set_defaults(func=cmd_probe)

    p = sub.add_parser("snap", help="存一张 RGB + 深度")
    p.add_argument("--out", default="./dcw_snap")
    p.add_argument("--intrinsics", default=None)
    p.add_argument("--laser", choices=("on", "off"), default=None)
    p.add_argument("--range", default=None, help="深度量程 mm，如 200,1200")
    p.set_defaults(func=cmd_snap)

    p = sub.add_parser("stream", help="连续取流打印中心深度")
    p.add_argument("--seconds", type=int, default=10)
    p.add_argument("--intrinsics", default=None)
    p.add_argument("--laser", choices=("on", "off"), default=None)
    p.add_argument("--range", default=None, help="深度量程 mm，如 200,1200")
    p.add_argument("--no-color", action="store_true", help="不解码彩色，只取深度")
    p.set_defaults(func=cmd_stream)

    p = sub.add_parser("tune", help="查看/调整设备参数并用 10 帧验证")
    p.add_argument("--intrinsics", default=None)
    p.add_argument("--laser", choices=("on", "off"), default=None)
    p.add_argument("--range", default=None, help="深度量程 mm，如 200,1200")
    p.add_argument("--hole-fill", action="store_true", help="开启洞填充（钳位时别开）")
    p.set_defaults(func=cmd_tune)

    p = sub.add_parser("demo", help="颜色目标检测 + 相机系 3D 测距")
    p.add_argument("--color", default="red")
    p.add_argument("--seconds", type=int, default=30)
    p.add_argument("--intrinsics", default=None)
    p.set_defaults(func=cmd_demo)

    p = sub.add_parser("calib", help="棋盘标定彩色内参")
    p.add_argument("--cols", type=int, default=9)
    p.add_argument("--rows", type=int, default=6)
    p.add_argument("--square", type=float, default=0.02)
    p.add_argument("--count", type=int, default=15)
    p.add_argument("--out", default="camera_intrinsics.json")
    p.set_defaults(func=cmd_calib)

    args = ap.parse_args()
    try:
        args.func(args)
    except DepthCameraError as e:
        raise SystemExit("[失败] %s" % e)


if __name__ == "__main__":
    main()
