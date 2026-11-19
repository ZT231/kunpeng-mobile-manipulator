#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""颜色 + 形状检测核心（探查阶段的「眼睛」）——只认像素，不碰标定/坐标。

【这个模块在项目里干什么】
  大范围探查流程是「语音 → HOME 摆探查位 → 上下左右扫 → 画面出现红色方块就锁定
  → 回 C 姿态精定位 → 抓取」。本模块负责中间那句「画面里有没有红色方块、在哪」。
  它只回答**像素层面**的问题（颜色/形状/中心点/方位），完全不碰手眼标定 k、X、
  不做 3D 反投影——那些是锁定之后 C 姿态精定位那一步的事，和这里解耦。

【为什么要解耦：探查 ≠ 定位】（本项目 2026-11-16 订正的核心结论）
  - 探查 = 回答「目标在不在、大概偏哪边」，只要有目标 + 画面里的粗略位置即可，
    **不依赖手眼标定精度**，所以任意俯视姿态、大范围扫都能用；
  - 定位 = 回答「目标在工作台的精确 x/y」，才依赖 k=1.091（且只在 C 姿态族准）。
  把两者分开，探查就可以放开扫，不会被「必须摆 C 姿态」束缚。

【移植来源】
  核心算法移植自本仓库另一个项目 `2-1.Robotic arm vision/vision/detector.py`
  （那套 HSV+形状识别已经实机验证过）。只搬「检测」部分，**没有**搬它的
  homography / IK / 眼在外标定——那些属于那个项目的坐标体系，与本项目无关。

【概念双轨制】（每个新概念：大白话直觉 + 规范定义）
  - HSV（大白话）：把颜色拆成「什么颜色 / 有多艳 / 有多亮」三个数，比 RGB 更抗
    阴影和光照变化。规范定义：Hue-Saturation-Value 颜色空间，OpenCV 用 H∈[0,179]、
    S/V∈[0,255] 的 8 位量化表示。
  - 红色要两段（大白话）：红色的 H 在色相环上跨在 0 和 180 两头，所以要分别取
    [0,10] 和 [170,180] 两段再拼起来，不能只用一段。规范定义：色相角 θ 在
    0°/360° 处不连续，红色横跨该边界，需双区间阈值并集。
  - 形态学开/闭（大白话）：开=先腐蚀后膨胀，抠掉孤立小噪点；闭=先膨胀后腐蚀，
    填上目标内部的小洞。规范定义：morphological opening / closing，用结构元素
    对二值 mask 做腐蚀-膨胀组合，去除椒盐噪声 / 闭合细小间隙。
  - 圆度（大白话）：轮廓有多接近正圆，1.0 是完美圆，方块约 0.785。规范定义：
    circularity = 4π·Area / Perimeter²，是形状紧致度的无量纲度量。
  - 多边形逼近（大白话）：把弯弯曲曲的轮廓简化成几条直线段，数有几条边就知道
    大概什么形状。规范定义：Douglas-Peucker 算法，cv2.approxPolyDP 按弧长比例
    epsilon 抽稀轮廓顶点。

【新手风格说明】
  代码刻意写成分步、直白、注释到位的形式（像教学代码），不玩一行梭哈；
  但用到的数据类、类型标注、dataclass.asdict 等现代写法都用上——让
  「刚学完 C 语言的人」能顺着注释读懂，同时能讲清每行为什么这么写。
"""
import json
import math
import os
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

# 稳定排序用的颜色顺序（与识别顺序无关，只为了让目标列表在帧间保持稳定，
# 否则同一桌东西每帧顺序乱跳，探查「锁定第几个」就会抖）
COLOR_ORDER: Dict[str, int] = {
    "red": 0, "orange": 1, "yellow": 2, "green": 3,
    "blue": 4, "purple": 5, "pink": 6,
}

# 画调试叠加层时各颜色对应的 BGR（OpenCV 内部是 B-G-R 顺序，不是 R-G-B）
COLOR_BGR: Dict[str, Tuple[int, int, int]] = {
    "red": (0, 0, 255),
    "orange": (0, 165, 255),
    "yellow": (0, 220, 220),
    "green": (0, 200, 0),
    "blue": (255, 0, 0),
    "purple": (200, 0, 200),
    "pink": (255, 180, 220),
}


@dataclass
class Target:
    """一个被识别到的目标。坐标全部是**全图像素坐标**（配 ROI 时会加回偏移）。

    - cx/cy: 最小外接矩形的中心（像素）。探查判方位就靠它和画面中心的比较。
    - w/h/angle: minAreaRect 给的宽、高、方向角（度），描述目标朝向。
    - area: 轮廓面积（像素²），用来按面积过滤噪点。
    - confidence: 填充度 = 轮廓面积 / 外接矩形面积，越接近 1 越「实心贴合」。
    - vertices: 轮廓逼近顶点（贴合形状画框用），圆为 None。
    """
    color: str
    shape: str
    cx: float
    cy: float
    angle: float
    w: float
    h: float
    area: float
    confidence: float
    ts: float
    vertices: Optional[List[Tuple[float, float]]] = None

    def to_dict(self) -> dict:
        """转成普通 dict，方便 JSON 序列化 / 打印 / 喂给上层决策。"""
        return asdict(self)


def classify_shape(contour, area: float) -> Tuple[str, Optional[List[Tuple[float, float]]]]:
    """按轮廓特征分类形状，返回 (形状名, 贴合绘制顶点)；圆返回 (shape, None)。

    判断顺序（从粗到细）：
      1. 圆度 ≥ 0.88 → 圆（圆逼近顶点太多，单独判，不数边）；
      2. 否则 approxPolyDP 数顶点 → 三角/四边/五边/多边；
      3. 四边再用「对边长度 + 内角」细分 正方形/菱形/矩形。
    """
    peri = cv2.arcLength(contour, True)
    if peri <= 0:
        return "unknown", None

    circularity = 4.0 * math.pi * area / (peri * peri)
    if circularity >= 0.88:
        return "circle", None

    approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
    pts = approx.reshape(-1, 2).astype(np.float64)
    pts = _drop_spurious(pts)          # 去掉圆角/噪声造成的「切角」伪顶点
    n = len(pts)

    if n == 3:
        return "triangle", _to_list(pts)
    if n == 4:
        return _quad_type(pts), _to_list(pts)
    if n == 5:
        return "pentagon", _to_list(pts)
    return "polygon", _to_list(pts)


def _drop_spurious(pts: np.ndarray) -> np.ndarray:
    """删除圆角/噪声造成的「切角」伪顶点：相邻短边 < 最长边 20% 就合并该顶点。

    例：圆角矩形 approxPolyDP 常给出 5 个顶点（一个圆角被拆成两个），其中必有
    一条短边，删掉它就回到 4 顶点 → 正确判为 rectangle。不做这步，方块会
    在区间抖成 pentagon/polygon，探查「找方块」就会漏检。
    """
    pts = [tuple(p) for p in pts]
    while len(pts) > 3:
        n = len(pts)
        edges = [
            math.hypot(pts[(i + 1) % n][0] - pts[i][0],
                       pts[(i + 1) % n][1] - pts[i][1])
            for i in range(n)
        ]
        longest = max(edges)
        idx = edges.index(min(edges))
        if edges[idx] < 0.2 * longest:
            del pts[idx]
        else:
            break
    return np.array(pts, dtype=np.float64)


def _to_list(pts: np.ndarray) -> List[Tuple[float, float]]:
    """numpy 顶点数组 → Python (x, y) 元组列表，方便 JSON 序列化。"""
    return [(float(p[0]), float(p[1])) for p in pts]


def _quad_type(pts: np.ndarray) -> str:
    """四边形细分：正方形 / 菱形 / 矩形 / 普通多边形。

    先用「对边长度均值」的比值判矩形——对边均值能抵消单个顶点的噪声，比拿
    单条边比更稳；再看内角是否 90° 区分正方形（方）和菱形（斜）。
    """
    edges = [float(np.linalg.norm(pts[(i + 1) % 4] - pts[i])) for i in range(4)]
    pair_a = (edges[0] + edges[2]) / 2.0
    pair_b = (edges[1] + edges[3]) / 2.0
    ratio = max(pair_a, pair_b) / (min(pair_a, pair_b) + 1e-6)
    if ratio > 1.3:
        return "rectangle"               # 两对对边均值明显不等 → 长方形

    ang = _angle_at(pts, 0)
    return "square" if 82.0 <= ang <= 98.0 else "diamond"


def _angle_at(pts: np.ndarray, i: int) -> float:
    """求 pts[i] 处的内角（度），用向量夹角的反余弦。"""
    a, b, c = pts[(i - 1) % 4], pts[i], pts[(i + 1) % 4]
    v1, v2 = a - b, c - b
    cos = float(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6))
    cos = max(-1.0, min(1.0, cos))       # 浮点误差可能让 cos 略微超出 [-1,1]
    return math.degrees(math.acos(cos))


class ColorShapeDetector:
    """从配置加载 HSV 阈值，对一帧图做颜色+形状识别。

    用法（探查每扫一格调一次）：
        det = ColorShapeDetector()               # 读同目录 color_targets_config.json
        targets = det.find(frame, color="red", shape="square")
        if targets:
            t = det.find_one(frame, color="red", shape="square")
            print(det.direction_of(t, frame.shape[1], frame.shape[0]))
    """

    def __init__(self, config_path: Optional[str] = None):
        self.config_path = config_path or os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "color_targets_config.json")
        self.config = self._load(self.config_path)

        self.min_area = float(self.config.get("min_area", 200))
        roi = self.config.get("roi")
        # ROI 为 null/缺省 = 全画面（探查要大范围，不裁剪）
        self.roi: Optional[Tuple[int, int, int, int]] = (
            tuple(int(v) for v in roi) if roi and len(roi) == 4 else None
        )
        self.colors = self.config.get("colors", {})
        morph = self.config.get("morph", {})
        self.open_k = self._odd(morph.get("open_ksize", 5))
        self.close_k = self._odd(morph.get("close_ksize", 5))

    @staticmethod
    def _load(config_path: str) -> dict:
        with open(config_path, "r", encoding="utf-8") as f:
            return json.load(f)

    @staticmethod
    def _odd(k) -> int:
        """形态学结构元素边长必须是奇数（有中心像素），偶数就 +1 变奇数。"""
        k = int(k)
        return k if k % 2 == 1 else k + 1

    def _crop(self, frame):
        if self.roi is None:
            return frame
        x, y, w, h = self.roi
        return frame[y:y + h, x:x + w]

    @staticmethod
    def _color_mask(hsv: np.ndarray, ranges) -> np.ndarray:
        """把同一颜色的多段 HSV 范围合并成一个二值 mask（红色要两段）。"""
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for low, high in ranges:
            m = cv2.inRange(hsv, np.array(low, np.uint8), np.array(high, np.uint8))
            mask = cv2.bitwise_or(mask, m)
        return mask

    def detect(self, frame) -> List[Target]:
        """核心：对一帧 BGR 图检测所有配置颜色+形状的目标，返回 Target 列表。"""
        roi = self._crop(frame)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        open_se = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.open_k, self.open_k))
        close_se = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.close_k, self.close_k))

        ox = 0 if self.roi is None else self.roi[0]
        oy = 0 if self.roi is None else self.roi[1]
        targets: List[Target] = []

        for color, spec in self.colors.items():
            mask = self._color_mask(hsv, spec["ranges"])
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_se)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_se)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < self.min_area:
                    continue
                shape, vertices = classify_shape(cnt, area)
                if vertices is not None:
                    vertices = [(vx + ox, vy + oy) for vx, vy in vertices]
                rect = cv2.minAreaRect(cnt)
                (cx, cy), (w, h), angle = rect
                rect_area = max(w * h, 1.0)
                confidence = round(min(1.0, area / rect_area), 3)
                targets.append(Target(
                    color=color, shape=shape,
                    cx=float(cx) + ox, cy=float(cy) + oy,
                    angle=float(angle), w=float(w), h=float(h),
                    area=float(area), confidence=confidence,
                    ts=0.0, vertices=vertices,
                ))

        targets = self._dedup(targets)
        # 稳定排序：颜色固定顺序 → 纵坐标 → 横坐标（不按面积，避免帧间抖动）
        targets.sort(key=lambda t: (COLOR_ORDER.get(t.color, 99), t.cy, t.cx))
        return targets

    def find(self, frame, color: Optional[str] = None,
             shape: Optional[str] = None, min_area: Optional[float] = None) -> List[Target]:
        """按颜色/形状/最小面积过滤，探查每扫一格就调它判断「有没有目标」。

        color/shape 传 None 表示不限；min_area 传 None 用配置默认值。
        """
        out = self.detect(frame)
        if color is not None:
            out = [t for t in out if t.color == color]
        if shape is not None:
            out = [t for t in out if t.shape == shape]
        if min_area is not None:
            out = [t for t in out if t.area >= min_area]
        return out

    def find_one(self, frame, color: Optional[str] = None,
                 shape: Optional[str] = None, **kwargs) -> Optional[Target]:
        """取过滤后面积最大的那一个，供探查「锁定最近/最大目标」用；没有返回 None。"""
        found = self.find(frame, color=color, shape=shape, **kwargs)
        return max(found, key=lambda t: t.area) if found else None

    @staticmethod
    def direction_of(target: Target, frame_w: int, frame_h: int,
                     deadzone_px: int = 20) -> str:
        """给探查的离散引导用：目标相对画面中心偏哪边，返回如「左 上」/「居中」。

        大白话：把画面当十字准星，目标离中心超过 deadzone_px 才算「偏了」，
        偏多少不管（离散引导每一步只按固定角度转，不连续跟）。
        规范定义：以图像中心为原点、带死区（dead zone）的象限量化，
        deadzone 抑制目标在中心附近抖动时方位反复横跳。
        """
        parts = []
        if target.cx < frame_w / 2 - deadzone_px:
            parts.append("左")
        elif target.cx > frame_w / 2 + deadzone_px:
            parts.append("右")
        if target.cy < frame_h / 2 - deadzone_px:
            parts.append("上")
        elif target.cy > frame_h / 2 + deadzone_px:
            parts.append("下")
        return " ".join(parts) if parts else "居中"

    @staticmethod
    def _dedup(targets: List[Target], dist_px: float = 25.0) -> List[Target]:
        """合并中心距离过近的同色目标（同一物体被 HSV 拆成多块时的去重）。

        只合并「同色且中心距 < dist_px」的，保留面积大者；不同色叠放也不合并。
        """
        out: List[Target] = []
        for t in targets:
            merged = False
            for i, o in enumerate(out):
                if t.color == o.color and math.hypot(t.cx - o.cx, t.cy - o.cy) < dist_px:
                    if t.area > o.area:
                        out[i] = t
                    merged = True
                    break
            if not merged:
                out.append(t)
        return out


def draw_targets(frame, targets: List[Target]) -> np.ndarray:
    """把识别结果叠加到 frame 上（就地画并返回），供调试看图确认检测对不对。

    按轮廓本身形状画框（圆=圆、多边=多边形），中心点 + 长边方向线 + 标签。
    """
    for t in targets:
        color_bgr = COLOR_BGR.get(t.color, (255, 0, 255))
        if t.shape == "circle":
            radius = int((t.w + t.h) / 4)
            cv2.circle(frame, (int(t.cx), int(t.cy)), radius, color_bgr, 2)
            top_y = t.cy - radius
        elif t.vertices and len(t.vertices) >= 3:
            pts = np.round(t.vertices).astype(np.int32)
            cv2.polylines(frame, [pts], True, color_bgr, 2)
            top_y = min(p[1] for p in t.vertices)
        else:
            box = cv2.boxPoints(((t.cx, t.cy), (t.w, t.h), t.angle))
            box = np.round(box).astype(np.int32)
            cv2.drawContours(frame, [box], 0, color_bgr, 2)
            top_y = min(p[1] for p in box)

        cv2.circle(frame, (int(t.cx), int(t.cy)), 3, color_bgr, -1)
        label = f"{t.color}/{t.shape}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(frame, (int(t.cx) - tw // 2 - 2, int(top_y) - th - 6),
                      (int(t.cx) + tw // 2 + 2, int(top_y) - 2), (0, 0, 0), -1)
        cv2.putText(frame, label, (int(t.cx) - tw // 2, int(top_y) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return frame
