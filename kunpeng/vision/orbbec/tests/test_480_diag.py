# -*- coding: utf-8 -*-
"""诊断 640x480 到底哪一路不出帧。"""
import sys

import numpy as np

sys.path.insert(0, "/root/kunpeng_vision")
import pyorbbecsdk as ob

mode = sys.argv[1]


def try_open(mode):
    p = ob.Pipeline()
    cfg = ob.Config()
    dpl = p.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR)
    cpl = p.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR)
    fps = 15 if mode.endswith("15") else 30
    if mode.startswith("donly"):
        pr = dpl.get_video_stream_profile(640, 480, ob.OBFormat.Y11, fps)
        cfg.enable_stream(pr)
        label = "depth-only 640x480@%d" % fps
    elif mode.startswith("conly"):
        pr = cpl.get_video_stream_profile(640, 480, ob.OBFormat.MJPG, fps)
        cfg.enable_stream(pr)
        label = "color-only 640x480@%d" % fps
    else:
        pr = dpl.get_video_stream_profile(640, 480, ob.OBFormat.Y11, fps)
        cfg.enable_stream(pr)
        cfg2 = cpl.get_video_stream_profile(640, 480, ob.OBFormat.MJPG, fps)
        cfg.enable_stream(cfg2)
        label = "both 640x480@%d" % fps
    print("打开:", label)
    p.start(cfg)
    n = 0
    for i in range(15):
        fs = p.wait_for_frames(1000)
        if fs is None:
            continue
        d = fs.get_depth_frame()
        c = fs.get_color_frame()
        n += 1
        if n <= 2:
            print("  frame%d depth=%s color=%s" % (
                i, None if d is None else "%dx%d" % (d.get_width(), d.get_height()),
                None if c is None else "%dx%d" % (c.get_width(), c.get_height())))
    print("  可用帧: %d/15" % n)
    p.stop()
    return n


for m in sys.argv[1:]:
    try:
        try_open(m)
    except Exception as e:
        print("  ERROR %s: %r" % (m, e))
    print("")
