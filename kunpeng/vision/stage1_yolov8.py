#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
阶段1：YOLOv8 通用物体识别（KunPeng Pro + GF100 摄像头）
============================================================
用途：把原 a_vision_demo.py 的「颜色 + 形状」识别，升级成 **YOLOv8 学习式目标检测**。
      这样能直接认出 水杯(cup) / 饮料瓶(bottle) / 水果(apple/orange/banana) 等「大部分物体」，
      不再局限于红绿蓝方块。这是「抓大部分物体」这条链路的第一环（识别层）。

小白先懂三个概念（面试也能这么讲）：
1. **目标检测（Object Detection，规范定义）**：给定一张图，算法同时输出图中每个物体的
   「位置框 (bounding box) + 类别 (class) + 置信度 (confidence)」。
   大白话：就是「框出图里有什么、在哪、有多确定」。
2. **预训练模型（Pre-trained model）**：别人用上百万张图（COCO 数据集，80 类常见物体）训练好的权重，
   模型已经「见过」杯子、瓶子、苹果长什么样。我们不用从零训，直接拿来用（这叫「迁移学习」的思想）。
3. **推理（Inference，规范定义）**：把一张新图喂给模型、得到结果的过程。
   大白话：「用训练好的模型去认新图片」。它和「训练」相对——训练是让模型学规律，推理是用学好的规律去判断。

为什么不用之前的「颜色+形状」了？
   颜色+形状只能认「有鲜艳颜色 + 简单几何」的物体；白色水杯、透明瓶、鸡蛋、纸巾在那种逻辑下全无效。
   YOLOv8 是从数据里学「物体长什么样」，所以认物能力天差地别。本项目后续还要用自有样本微调（阶段2）
   去认纸巾/鸡蛋这类 COCO 里没有的类。

两种运行后端（自动选）：
   A. ultralytics 官方库（推荐，API 最简，且阶段2微调也用它）——依赖 torch（aarch64 有官方 wheel）。
   B. OpenCV DNN 读 ONNX（降级方案，仅推理，依赖极小，只需 opencv + numpy，不装 torch 也能跑）。
   脚本会先试 A，失败自动降级到 B，保证在「只装了 opencv」的最小环境也能出框。
"""

import argparse
import sys
import time

import cv2
import numpy as np

# ===================================================================
# COCO 80 类名称（ultralytics 默认训练集的类别顺序，索引 0~79）
# 这就是 YOLOv8n 预训练权重「天生会认」的 80 样东西。
# ===================================================================
COCO_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush",
]

# 本项目「优先高亮」的可抓取日用物体（认到这些就重点框出来；其余类淡显）
# 阶段2 微调出 tissue / egg 后，把它们加进这个集合即可。
PRIORITY_CLASSES = {
    "cup", "bottle", "bowl", "banana", "apple", "orange",
    "potted plant", "book", "wine glass",
}


# ===================================================================
# 模型加载：优先 ultralytics，降级 OpenCV DNN(ONNX)
# ===================================================================
def load_model(model_path):
    """
    加载 YOLOv8 模型，返回 (后端名, 模型对象)。
    后端名用于后续推理时走不同的分支。

    规范定义 - 模型导出格式：
      .pt  = PyTorch 权重（ultralytics 原生格式，需 torch 加载）
      .onnx = 开放神经网络交换格式（跨框架通用，OpenCV/ONNXRuntime 都能读，无需 torch）
    """
    # --- 分支 A：ultralytics（需要 pip install ultralytics，它依赖 torch）---
    try:
        from ultralytics import YOLO
        model = YOLO(model_path)
        print(f"[模型] 用 ultralytics 加载：{model_path}")
        return ("ultralytics", model)
    except Exception as e:  # 没装 torch/ultralytics 就走降级
        print(f"[信息] ultralytics 不可用（{e}），尝试 OpenCV DNN(ONNX) 降级...")

    # --- 分支 B：OpenCV DNN 读 ONNX（只需 opencv + numpy）---
    onnx_path = model_path
    if model_path.endswith(".pt"):
        # OpenCV 读不了 .pt，必须先把 .pt 导出成 .onnx（在装了 ultralytics 的机器上做一次即可）
        raise RuntimeError(
            "OpenCV DNN 只认 .onnx 文件。请用 `yolo export model=yolov8n.pt format=onnx` "
            "导出，或安装 ultralytics（pip install ultralytics）来直接读 .pt。"
        )
    net = cv2.dnn.readNetFromONNX(onnx_path)
    print(f"[模型] 用 OpenCV DNN(ONNX) 加载：{onnx_path}")
    return ("dnn", net)


# ===================================================================
# 推理：两种后端各自实现一个函数，统一返回 detections 列表
# detections 元素格式：(x1, y1, x2, y2, conf, cls_name)
# ===================================================================
def postprocess_dnn(out, h, w, conf, inp_size=640):
    """
    YOLOv8 ONNX 后处理（从网络输出到检测框列表）。抽成独立函数便于单元测试（--selftest）。

    规范定义 - YOLOv8 ONNX 输出张量：
      形状 (1, 84, 8400)：1=批次；8400=候选框数量；84=4个框坐标(cx,cy,w,h)+80个类别分数。
      我们把 (84, 8400) 转置成 (8400, 84)，逐行取「分数最高的类」当预测类别。
    坐标 (cx,cy,w,h) 是在 inp_size×inp_size 输入网格上的数值，需映射回原图 (w, h)。
    """
    out = out[0].T  # (8400, 84)
    boxes, confs, cls_ids = [], [], []
    for row in out:
        scores = row[4:]
        cls_id = int(np.argmax(scores))
        score = float(scores[cls_id])
        if score < conf:
            continue
        cx, cy, bw, bh = row[0], row[1], row[2], row[3]
        # 从网格坐标映射回原图像素坐标
        x1 = int((cx - bw / 2) / inp_size * w)
        y1 = int((cy - bh / 2) / inp_size * h)
        x2 = int((cx + bw / 2) / inp_size * w)
        y2 = int((cy + bh / 2) / inp_size * h)
        boxes.append([x1, y1, x2, y2])
        confs.append(score)
        cls_ids.append(cls_id)

    # 规范定义 - NMS（非极大值抑制 Non-Maximum Suppression）：
    # 同一物体常被多个候选框命中，NMS 按「置信度从高到低、重叠度(IoU)超阈值就删」去掉重复框。
    idxs = cv2.dnn.NMSBoxes(boxes, confs, conf, 0.45)
    dets = []
    if len(idxs) > 0:
        for i in idxs.flatten():
            dets.append((boxes[i][0], boxes[i][1], boxes[i][2], boxes[i][3],
                         confs[i], COCO_NAMES[cls_ids[i]]))
    return dets


def infer_ultralytics(model, frame, conf, imgsz=640):
    """分支 A：用 ultralytics 做推理。返回检测框列表。imgsz 越小越快。"""
    results = model(frame, conf=conf, verbose=False, imgsz=imgsz)[0]
    dets = []
    for box in results.boxes:
        x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
        conf_i = float(box.conf[0])
        cls_id = int(box.cls[0])
        name = results.names[cls_id]
        dets.append((x1, y1, x2, y2, conf_i, name))
    return dets


def infer_dnn(net, frame, conf, inp_size=640):
    """分支 B：OpenCV DNN 读 ONNX 推理，后处理交给 postprocess_dnn。"""
    h, w = frame.shape[:2]
    # blobFromImage：把图 resize 到 inp_size×inp_size、归一化(÷255)、BGR→RGB，符合模型输入
    blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (inp_size, inp_size), swapRB=True, crop=False)
    net.setInput(blob)
    out = net.forward()
    return postprocess_dnn(out, h, w, conf, inp_size)


def infer(backend, model, frame, conf, imgsz=640):
    """统一入口：根据后端分发到对应推理函数。"""
    if backend == "ultralytics":
        return infer_ultralytics(model, frame, conf, imgsz)
    return infer_dnn(model, frame, conf, imgsz)


def self_test():
    """
    不依赖权重，验证 postprocess_dnn 的坐标映射与 NMS 逻辑正确（生产代码路径实跑）。
    构造一个假的 YOLO 输出张量，模拟「一个 cup 框 + 一个重复框 + 一个低分噪声」。
    """
    print("[selftest] 构造假 YOLO 输出，验证后处理（坐标映射 + NMS）...")
    h, w = 960, 1280  # 假设原图 1280×960
    out = np.zeros((1, 84, 8400), dtype=np.float32)
    # cup(cls=41) 中心(320,240) 尺寸100x120 @640网格
    out[0, 0, 0] = 320; out[0, 1, 0] = 240; out[0, 2, 0] = 100; out[0, 3, 0] = 120
    out[0, 4 + 41, 0] = 0.90
    # 同位置重复框（NMS 应合并为 1 个）
    out[0, 0, 1] = 325; out[0, 1, 1] = 242; out[0, 2, 1] = 100; out[0, 3, 1] = 120
    out[0, 4 + 41, 1] = 0.85
    # 低分噪声（应被 conf 阈值过滤）
    out[0, 0, 2] = 100; out[0, 1, 2] = 100; out[0, 2, 2] = 50; out[0, 3, 2] = 50
    out[0, 4 + 41, 2] = 0.10

    dets = postprocess_dnn(out, h, w, 0.4)
    assert len(dets) == 1, f"应保留 1 个框，实际 {len(dets)}"
    x1, y1, x2, y2, conf, name = dets[0]
    assert name == "cup", f"类别应为 cup，实际 {name}"
    # 期望坐标：cx=320/640*1280=640, cy=240/640*960=360, w=100/640*1280=200, h=120/640*960=180
    #          → x1=640-100=540, y1=360-90=270, x2=740, y2=450
    assert (x1, y1, x2, y2) == (540, 270, 740, 450), f"坐标映射错: {(x1, y1, x2, y2)}"
    print(f"[selftest] 通过：检测到 {name} @{(x1, y1, x2, y2)} conf={conf:.2f}（坐标映射 + NMS 正确）")


# ===================================================================
# 画框 + 打印结果（复用 a_vision_demo.py 的「方向提示」思路，方便接机器人决策）
# ===================================================================
def draw_and_report(frame, dets, h_f, w_f):
    count = 0
    for (x1, y1, x2, y2, conf_i, name) in dets:
        is_priority = name in PRIORITY_CLASSES
        # 优先类用绿色高亮，其他类用蓝色淡显
        color = (0, 255, 0) if is_priority else (255, 0, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        label = f"{name} {conf_i:.2f}"
        cv2.putText(frame, label, (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        # 给机器人用的「目标中心」：取框中心 + 标记
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        cv2.circle(frame, (cx, cy), 3, (0, 255, 255), -1)
        # 方向提示（沿用阶段0/P0 的判据：偏离画面中心 ±20px 即提示方位）
        dir_txt = ""
        if cx < w_f // 2 - 20:
            dir_txt += "目标在左 "
        elif cx > w_f // 2 + 20:
            dir_txt += "目标在右 "
        if cy < h_f // 2 - 20:
            dir_txt += "上"
        elif cy > h_f // 2 + 20:
            dir_txt += "下"
        if is_priority:
            count += 1
            # headless 模式靠这行知道「抓哪个、在哪」
            print(f"  [可抓取] {name} 置信度={conf_i:.2f} 中心=({cx},{cy}) 方位={dir_txt or '居中'}")
    return count


# ===================================================================
# 主程序
# ===================================================================
def main():
    parser = argparse.ArgumentParser(description="阶段1：YOLOv8 通用物体识别 (KunPeng + GF100)")
    parser.add_argument("--source", default="0",
                        help="输入源：摄像头索引(默认0=/dev/video0) / 图片路径 / 视频路径")
    parser.add_argument("--model", default="yolov8n.pt",
                        help="模型路径：.pt(用ultralytics) 或 .onnx(用OpenCV DNN)，默认 yolov8n.pt")
    parser.add_argument("--conf", type=float, default=0.4, help="置信度阈值，默认 0.4")
    parser.add_argument("--imgsz", type=int, default=640,
                        help="推理输入分辨率(正方形边长)。默认640精度高但慢；KunPeng 纯CPU跑不动时降到320可提速约3-4倍")
    parser.add_argument("--headless", action="store_true", help="无显示：不弹窗口，终端打印检测结果")
    parser.add_argument("--width", type=int, default=640, help="摄像头分辨率宽（USB2.0 别一上来 1280）")
    parser.add_argument("--height", type=int, default=480, help="摄像头分辨率高")
    parser.add_argument("--selftest", action="store_true", help="自检后处理逻辑（不需模型权重）")
    args = parser.parse_args()

    # 自检模式：不碰摄像头、不加载模型，只验证后处理数学
    if args.selftest:
        self_test()
        return

    # 1) 加载模型
    backend, model = load_model(args.model)

    # 2) 打开输入源（摄像头 or 图片/视频文件）
    is_camera = args.source.isdigit()
    if is_camera:
        cap = cv2.VideoCapture(int(args.source))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    else:
        cap = cv2.VideoCapture(args.source)
    if not cap.isOpened():
        print(f"[错误] 打不开输入源：{args.source}")
        print("排查：摄像头用 `ls /dev/video*`；图片/视频确认路径存在。")
        sys.exit(1)
    print(f"[OK] 输入源已打开：{'摄像头'+str(args.source) if is_camera else args.source}")

    frame_count = 0
    start_time = time.time()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                if is_camera:
                    print("[警告] 读不到帧，摄像头可能被占用或断连")
                break
            frame_count += 1
            h_f, w_f = frame.shape[:2]

            # 3) 推理（imgsz 越小越快；KunPeng 纯CPU 用 320 提速明显）
            dets = infer(backend, model, frame, args.conf, args.imgsz)

            # 4) 画框 + 报告（图片/视频模式下只在有窗口时画；headless 只打印）
            if not args.headless or is_camera:
                draw_and_report(frame, dets, h_f, w_f)

            # 5) 显示 / 退出
            if not args.headless:
                cv2.imshow("阶段1 YOLOv8", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
            # headless 摄像头模式：持续运行，Ctrl+C 停止（便于机器人常驻识别）
    except KeyboardInterrupt:
        print("\n[退出] 收到 Ctrl+C，停止识别")

    # 6) 收尾：算平均 fps（衡量板子算力够不够）
    elapsed = time.time() - start_time
    if frame_count > 0 and elapsed > 0:
        fps = frame_count / elapsed
        print(f"[统计] 共 {frame_count} 帧，耗时 {elapsed:.1f}s，平均 {fps:.1f} FPS（{backend} 后端）")
        if fps < 5:
            print("[建议] FPS 偏低：用 `--imgsz 320` 降推理分辨率（计算量降到 1/4，预期 8~10 FPS）；"
                  "yolov8n 已是最轻模型；或后续导出 INT8 量化、或上 OpenCV DNN 后端；"
                  "不要指望 NPU（本项目 NPU 为已知设计缺陷不可用）。")
    cap.release()
    if not args.headless:
        cv2.destroyAllWindows()
    print("[完成] 已退出")


if __name__ == "__main__":
    main()
