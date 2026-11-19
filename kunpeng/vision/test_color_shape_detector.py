#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阶段 1 本地自检：不摄像头、不动臂，用合成图验证「红色方块能被稳定检出」。

跑法（本地系统 python，自带 cv2；不装任何包）：
    python test_color_shape_detector.py
    python test_color_shape_detector.py --preview preview.png   # 额外存一张调试叠加图

【为什么要合成图自测】
  探查要「任意俯视姿态大范围扫」，真机姿态/光照千变万化；先在合成图上把
  「颜色分割 → 形状分类 → 方位判定」这条纯视觉链路证明是对的，再去真机调
  HSV 阈值（阈值是现场标定的事，不在这层测）。合成图颜色确定、可复现，
  是这个模块最稳的地基测试。
"""
import argparse
import math
import os
import sys
import unittest

import cv2
import numpy as np

from color_shape_detector import (
    ColorShapeDetector, draw_targets,
)

# 合成图用的 BGR 颜色（OpenCV 内部 B-G-R 顺序）
BGR_RED = (0, 0, 220)
BGR_GREEN = (0, 180, 0)
BGR_BG = (240, 240, 240)


def make_frame(layout, w=640, h=480):
    """按 layout 画一张合成图，返回 BGR ndarray。

    layout 每项：(颜色BGR, 形状, 中心(cx,cy), 尺寸s)。
    """
    img = np.full((h, w, 3), BGR_BG, np.uint8)
    for color, shape, (cx, cy), s in layout:
        if shape == "square":
            cv2.rectangle(img, (cx - s // 2, cy - s // 2),
                          (cx + s // 2, cy + s // 2), color, -1)
        elif shape == "circle":
            cv2.circle(img, (cx, cy), s // 2, color, -1)
        elif shape == "triangle":
            pts = np.array([[cx, cy - s // 2], [cx - s // 2, cy + s // 2],
                            [cx + s // 2, cy + s // 2]], np.int32)
            cv2.fillConvexPoly(img, pts, color)
        elif shape == "rectangle":
            cv2.rectangle(img, (cx - s, cy - s // 2),
                          (cx + s, cy + s // 2), color, -1)
        elif shape == "diamond":
            # 真菱形：对角线不等长（横向 s、纵向 s/2）。若画成等长对角线，
            # 那其实是「正方形转 45°」、内角仍 90°，会被正确判成 square。
            pts = np.array([[cx, cy - s // 4], [cx + s // 2, cy],
                            [cx, cy + s // 4], [cx - s // 2, cy]], np.int32)
            cv2.fillConvexPoly(img, pts, color)
        elif shape == "pentagon":
            pts = np.array([
                [cx + int(s / 2 * math.cos(math.radians(-90 + 72 * k))),
                 cy + int(s / 2 * math.sin(math.radians(-90 + 72 * k)))]
                for k in range(5)], np.int32)
            cv2.fillConvexPoly(img, pts, color)
    return img


class TestColorShapeDetector(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.det = ColorShapeDetector()

    def test_red_square_single(self):
        """画面正中一个红方块：必须检出 1 个、形状 square、中心对得上。"""
        frame = make_frame([(BGR_RED, "square", (320, 240), 80)])
        found = self.det.find(frame, color="red", shape="square")
        self.assertEqual(len(found), 1, f"应检出 1 个红方块，实得 {len(found)}")
        t = found[0]
        self.assertEqual(t.color, "red")
        self.assertEqual(t.shape, "square")
        self.assertLess(abs(t.cx - 320), 3, f"中心 x 偏太多：{t.cx}")
        self.assertLess(abs(t.cy - 240), 3, f"中心 y 偏太多：{t.cy}")

    def test_red_square_multiple_positions(self):
        """三个不同位置各放一个红方块：都要检出，证明不是只在正中间才认得。"""
        centers = [(120, 100), (320, 240), (520, 400)]
        frame = make_frame([(BGR_RED, "square", c, 70) for c in centers])
        found = self.det.find(frame, color="red", shape="square")
        self.assertEqual(len(found), 3, f"应检出 3 个红方块，实得 {len(found)}")
        got = sorted((round(t.cx), round(t.cy)) for t in found)
        exp = sorted((cx, cy) for cx, cy in centers)
        for (gx, gy), (ex, ey) in zip(got, exp):
            self.assertLess(abs(gx - ex), 4, f"中心对不上：{got} vs {exp}")
            self.assertLess(abs(gy - ey), 4, f"中心对不上：{got} vs {exp}")

    def test_color_filter_ignores_green(self):
        """红方块 + 绿方块同框：find(red) 只拿红的，颜色过滤生效。"""
        frame = make_frame([
            (BGR_RED, "square", (180, 240), 80),
            (BGR_GREEN, "square", (460, 240), 80),
        ])
        reds = self.det.find(frame, color="red", shape="square")
        greens = self.det.find(frame, color="green", shape="square")
        self.assertEqual(len(reds), 1, f"红方块应 1 个，实得 {len(reds)}")
        self.assertEqual(len(greens), 1, f"绿方块应 1 个，实得 {len(greens)}")
        self.assertLess(reds[0].cx, 320, "红方块应在画面左")
        self.assertGreater(greens[0].cx, 320, "绿方块应在画面右")

    def test_shape_filter_square_vs_circle(self):
        """红方块 + 红圆同框：形状分类要把两者分开（探查找的是方块，不是圆）。"""
        frame = make_frame([
            (BGR_RED, "square", (180, 240), 80),
            (BGR_RED, "circle", (460, 240), 80),
        ])
        squares = self.det.find(frame, color="red", shape="square")
        circles = self.det.find(frame, color="red", shape="circle")
        self.assertEqual(len(squares), 1, f"方块应 1 个，实得 {len(squares)}")
        self.assertEqual(len(circles), 1, f"圆应 1 个，实得 {len(circles)}")

    def test_all_shapes_classified(self):
        """六种形状各画一个：square/rectangle/diamond/triangle/circle/pentagon 都要认对。

        形状分类在核心里本来就全支持，这里逐个验证确实没漏（用户要求全形状）。
        """
        cases = [
            ("square", (100, 90), 60),
            ("rectangle", (300, 90), 45),
            ("triangle", (520, 90), 70),
            ("circle", (100, 330), 70),
            ("diamond", (320, 330), 70),
            ("pentagon", (540, 330), 70),
        ]
        frame = make_frame([(BGR_RED, shape, c, s) for shape, c, s in cases])
        got = {t.shape for t in self.det.find(frame, color="red")}
        for shape, _, _ in cases:
            self.assertIn(shape, got, f"形状 {shape} 没认出来；实得 {sorted(got)}")

    def test_all_colors_detected(self):
        """七种颜色各画一个方块：red/orange/yellow/green/blue/purple/pink 都要检出。

        颜色由 config 的 ranges 决定（不像形状在核心里），所以逐个验证阈值没写错。
        """
        palette = {
            "red": (0, 0, 220),
            "orange": (0, 165, 255),
            "yellow": (0, 220, 220),
            "green": (0, 180, 0),
            "blue": (255, 0, 0),
            "purple": (200, 0, 200),
            "pink": (220, 180, 255),
        }
        # 一行排开 7 个，每个方块 50px
        layout = [(bgr, "square", (50 + i * 90, 240), 50)
                  for i, bgr in enumerate(palette.values())]
        frame = make_frame(layout)
        all_found = self.det.detect(frame)
        got_colors = {t.color for t in all_found}
        for name in palette:
            self.assertIn(name, got_colors,
                          f"颜色 {name} 没检出；实得 {sorted(got_colors)}")

    def test_small_noise_filtered(self):
        """3x3 红点被 min_area 滤掉，不会被当成目标（抗噪）。"""
        frame = make_frame([(BGR_RED, "square", (320, 240), 3)])
        found = self.det.find(frame, color="red")
        self.assertEqual(len(found), 0, f"小噪点应被滤掉，实得 {len(found)}")

    def test_find_one_picks_largest(self):
        """两个红方块一大一小：find_one 取大的那个（探查锁定用）。"""
        frame = make_frame([
            (BGR_RED, "square", (160, 240), 60),
            (BGR_RED, "square", (480, 240), 100),
        ])
        t = self.det.find_one(frame, color="red", shape="square")
        self.assertIsNotNone(t)
        self.assertGreater(t.cx, 320, "应锁定右边那个更大的方块")

    def test_direction_of(self):
        """方位判定：左上 → 左 上；正中 → 居中（离散引导的输入）。"""
        frame = make_frame([])
        t_left = self.det.find_one(make_frame([(BGR_RED, "square", (120, 100), 60)]),
                                   color="red")
        t_mid = self.det.find_one(make_frame([(BGR_RED, "square", (320, 240), 60)]),
                                  color="red")
        self.assertEqual(ColorShapeDetector.direction_of(t_left, 640, 480), "左 上")
        self.assertEqual(ColorShapeDetector.direction_of(t_mid, 640, 480), "居中")

    def test_no_target_returns_empty(self):
        """空画面：find 返回空列表、find_one 返回 None，不崩。"""
        frame = make_frame([])
        self.assertEqual(self.det.find(frame, color="red"), [])
        self.assertIsNone(self.det.find_one(frame, color="red"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--preview", default=None, help="存一张调试叠加图到指定路径")
    args, unknown = ap.parse_known_args()

    # 先跑全部用例
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(TestColorShapeDetector)
    result = unittest.TextTestRunner(verbosity=2).run(suite)

    if args.preview:
        det = ColorShapeDetector()
        frame = make_frame([
            (BGR_RED, "square", (200, 180), 90),
            (BGR_GREEN, "square", (460, 300), 80),
            (BGR_RED, "circle", (480, 130), 70),
        ])
        found = det.detect(frame)
        out = frame.copy()
        draw_targets(out, found)
        cv2.imwrite(args.preview, out)
        print(f"[预览] 已存 {os.path.abspath(args.preview)}："
              f"检出 {[(t.color, t.shape, round(t.cx), round(t.cy)) for t in found]}")

    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    sys.exit(main())
