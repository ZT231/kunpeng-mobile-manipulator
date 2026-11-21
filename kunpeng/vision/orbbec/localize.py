# -*- coding: utf-8 -*-
"""像素 -> 相机系 3D 点。

坐标系约定（OpenCV 针孔惯例，明天手眼标定直接接这个）：
    +x 右，+y 下，+z 沿光轴向前（远离相机）
    单位 cm
    输入 depth_mm 是 OpenNI 深度（沿光轴的 Z，单位 mm）

注意：D2C 对齐后，深度图已重采样到彩色网格，故这里用彩色内参 K。
"""


def pixel_to_camera(u, v, depth_mm, K):
    """(u,v) 像素 + 该点深度(mm) + 内参 -> (x, y, z) cm（相机系）。

    返回 None 表示深度无效。
    """
    if depth_mm is None or depth_mm <= 0:
        return None
    if K is None:
        raise ValueError("缺少内参 K（fx/fy/cx/cy）")
    fx = float(K["fx"])
    fy = float(K["fy"])
    cx = float(K["cx"])
    cy = float(K["cy"])
    z_cm = float(depth_mm) / 10.0
    x_cm = (float(u) - cx) * z_cm / fx
    y_cm = (float(v) - cy) * z_cm / fy
    return x_cm, y_cm, z_cm
