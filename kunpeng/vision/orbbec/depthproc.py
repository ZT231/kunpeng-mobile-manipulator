# -*- coding: utf-8 -*-
"""深度图预处理 + 区域采样。

DaBai DCW 是双目结构光，深度图天然带这些脏东西，必须处理后再用：
  1) 飞行像素（边缘飞出来的孤立点）
  2) 孔洞（0 值，遮挡/IR 吸收）
  3) 黑色/高反物体直接测不到（有效像素占比低）
"""
import cv2
import numpy as np


def remove_flying(depth, k=5):
    """中值滤波吃掉飞行像素。depth 为 uint16(mm)；k 必须为奇数，16U 上限 5。"""
    if depth is None or depth.size == 0:
        return depth
    return cv2.medianBlur(depth, k)


def sample_region(depth, box, min_valid_ratio=0.3):
    """在 bbox 区域内采样一个稳健深度值。

    参数：
        depth  uint16 深度图(mm)，0 表示无效
        box    [x1, y1, x2, y2] 像素坐标（闭区间右开）
        min_valid_ratio  区域内有效像素占比低于此值 => 判定"测不到"(如黑物体)

    返回：
        (z_mm, valid_ratio)  z_mm=None 表示不可信
    """
    if depth is None:
        return None, 0.0
    h, w = depth.shape[:2]
    x1, y1, x2, y2 = (int(v) for v in box)
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w, x2), min(h, y2)
    if x2 <= x1 or y2 <= y1:
        return None, 0.0

    region = depth[y1:y2, x1:x2]
    valid = region[region > 0]
    ratio = float(valid.size) / float(region.size)

    if valid.size == 0 or ratio < min_valid_ratio:
        return None, ratio
    return float(np.median(valid)), ratio


def colorize(depth, max_mm=3000):
    """把 uint16 深度图转成便于肉眼看的 uint8 灰度图（0 无效像素置 0）。"""
    if depth is None:
        return None
    d = depth.astype(np.float32)
    d[d <= 0] = 0
    d = np.clip(d, 0, max_mm)
    return (d * (255.0 / max_mm)).astype(np.uint8)
