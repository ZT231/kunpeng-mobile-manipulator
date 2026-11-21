# -*- coding: utf-8 -*-
"""对比：原始深度(不对齐) vs 对齐后深度 的逐列有效占比。"""
import sys

import cv2
import numpy as np

sys.path.insert(0, "/root/kunpeng_vision")
import pyorbbecsdk as ob
from orbbec.source import DepthCamera

mode = sys.argv[1]


def colprof(d, w, n=20):
    b = max(1, w // n)
    return " ".join("%.2f" % (d > 0)[:, x:x + b].mean() for x in range(0, w, b))


if mode == "raw":
    cam = DepthCamera(register=False)
    print("registered:", cam.registered)
    r = cam.read()
    d = r["depth_mm"]
    c = r["color_bgr"]
    print("raw depth=%s color=%s" % (d.shape, None if c is None else c.shape))
    print("全图有效 %.3f  中央50%% %.3f" % (
        (d > 0).mean(),
        (d > 0)[int(d.shape[0]*.25):int(d.shape[0]*.75), int(d.shape[1]*.25):int(d.shape[1]*.75)].mean()))
    print("逐列有效:", colprof(d, d.shape[1]))
    if c is not None and c.shape[:2] == d.shape[:2]:
        # 原始深度的边缘 vs 彩色边缘
        g = cv2.GaussianBlur(cv2.cvtColor(c, cv2.COLOR_BGR2GRAY), (5, 5), 0)
        ce = cv2.Canny(g, 60, 160)
        dd = d.astype(np.float32); dd[d <= 0] = np.median(d[d > 0])
        dd = cv2.GaussianBlur(dd, (5, 5), 0)
        dn = cv2.normalize(dd, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        de = cv2.dilate(cv2.Canny(dn, 25, 70), np.ones((5, 5), np.uint8))
        print("原始深度与彩色边缘重合(不额外平移): %.1f%%" % (
            100.0 * np.logical_and(ce > 0, de > 0).sum() / max(int((ce > 0).sum()), 1)))
    cam.close()

elif mode == "adepth":
    p = ob.Pipeline()
    cfg = ob.Config()
    dp = p.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR).get_default_video_stream_profile()
    cp = p.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR).get_default_video_stream_profile()
    cfg.enable_stream(dp); cfg.enable_stream(cp)
    p.start(cfg)
    af = ob.AlignFilter(align_to_stream=ob.OBStreamType.DEPTH_STREAM)
    n = 0; last = None
    for i in range(20):
        fs = p.wait_for_frames(1000)
        if fs is None:
            continue
        fs = af.process(fs)
        dk = fs.get_depth_frame(); cf = fs.get_color_frame()
        if dk is None or cf is None:
            continue
        d = np.frombuffer(dk.get_data(), dtype=np.uint16).reshape((dk.get_height(), dk.get_width()))
        raw = np.frombuffer(cf.get_data(), dtype=np.uint8)
        c = cv2.imdecode(raw, cv2.IMREAD_COLOR)
        last = (c, d)
        n += 1
    print("对齐到深度 可用帧 %d/20" % n)
    if last:
        c, d = last
        print("depth=%s color=%s 全图有效 %.3f" % (d.shape, c.shape, (d > 0).mean()))
        print("逐列有效:", colprof(d, d.shape[1]))
    p.stop()
