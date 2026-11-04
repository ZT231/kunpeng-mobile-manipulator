#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
阶段1 常驻服务（把 YOLOv8 识别层封装成可被决策/底盘层调用的服务）
================================================================
为什么需要这一层？（小白视角）
  之前的 stage1_yolov8.py 是个「一次性 demo」：启动后只在终端打印识别结果，
  别的程序（决策层、底盘 STM32、机械臂 STM32）拿不到这些数据。
  我们要把识别做成一个「常驻服务」：它一直在后台跑摄像头 + 推理，
  把最新的识别结果存在一个地方，别的地方随时来问「现在看到什么了」就能拿到。

  类比：它像「前台接待」——摄像头是眼睛、推理是脑子，前台把「当前看到了 X 在 Y 方向」
  记在登记簿上；底盘/机械臂来问，就翻登记簿告诉他们。不用每次都重新长眼睛。

两个核心概念（面试这么讲）：
  1. 多线程（Thread，规范定义）：操作系统能在同一进程里「同时」跑多个任务。
     大白话：主线程负责开 HTTP 服务器等人来问；我们另开一个「工作线程」专门跑摄像头+推理，
     两者并行、互不卡。共享数据用一把锁（`threading.Lock`）保护，避免「工作线程正在写、
     HTTP 线程同时在读」导致数据错乱（这叫「线程安全」）。
  2. HTTP / JSON（规范定义）：HTTP 是浏览器和服务器通信的协议；JSON 是一种轻量数据格式，
     用 {键:值} 表达结构化信息，人和机器都好读。
     大白话：用 Python 标准库的 http.server 起一个迷你服务器，
     谁访问 `http://板子IP:8080/detections`，就收到一段 JSON 文本，里面是「当前识别到的物体列表」。
     这就是「打通决策/底盘层」的接口。（后续想换成 ROS2 话题也很容易——数据一样，只换传输通道。）

零依赖：只用 Python 标准库（http.server / json / threading）+ 已有的 opencv/torch/ultralytics。
板上不用再装任何新东西。

复用：识别核心直接从 stage1_yolov8 导入（COCO_NAMES / PRIORITY_CLASSES / load_model / infer），
保证和 stage1 demo 行为完全一致，不重复写推理逻辑。
"""

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

# 复用已验证的识别核心（不重复造轮子，保证和 stage1 demo 行为一致）
from stage1_yolov8 import COCO_NAMES, PRIORITY_CLASSES, load_model, infer, draw_and_report


# ===================================================================
# 方向判定（沿用 stage1 的判据：偏离画面中心 ±20px 即提示方位）
# ===================================================================
def compute_direction(cx, cy, w_f, h_f):
    """返回人类可读方位串，如 '左 上' / '右' / '居中'。"""
    parts = []
    if cx < w_f // 2 - 20:
        parts.append("左")
    elif cx > w_f // 2 + 20:
        parts.append("右")
    if cy < h_f // 2 - 20:
        parts.append("上")
    elif cy > h_f // 2 + 20:
        parts.append("下")
    return " ".join(parts) if parts else "居中"


class Detector:
    """常驻识别器：后台线程跑摄像头 + 推理，对外提供最新结构化结果。"""

    def __init__(self, model_path, source=0, imgsz=320, conf=0.4,
                 width=640, height=480):
        self.model_path = model_path
        self.source = source
        self.is_camera = str(source).isdigit()
        self.imgsz = imgsz
        self.conf = conf
        self.width = width
        self.height = height

        self.backend = None
        self.model = None
        self.cap = None

        # 最新识别结果（线程安全：用锁保护）
        self._lock = threading.Lock()
        self._latest = {"frame_id": 0, "timestamp": 0.0,
                        "detections": [], "target": None}
        # 用 Event 做「停止信号」，比普通 bool 更可靠（跨线程可见、可等待）
        self._running = threading.Event()
        self._thread = None
        self._frame_count = 0
        self._start_time = 0.0

    def start(self):
        """加载模型、开摄像头、启动后台推理线程。"""
        # 1) 加载模型（重量级，放到 start 里，避免 import 时就卡）
        self.backend, self.model = load_model(self.model_path)
        # 2) 打开输入源
        src = int(self.source) if self.is_camera else self.source
        self.cap = cv2.VideoCapture(src)
        if self.is_camera:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if not self.cap.isOpened():
            raise RuntimeError(f"打不开输入源：{self.source}")
        print(f"[OK] 输入源已打开：{'摄像头'+str(self.source) if self.is_camera else self.source}")

        # 3) 启动工作线程
        self._running.set()
        self._start_time = time.time()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print("[服务] 后台推理线程已启动")

    def _loop(self):
        """工作线程主循环：不断读帧 -> 推理 -> 更新最新结果。"""
        try:
            while self._running.is_set():
                ret, frame = self.cap.read()
                if not ret:
                    if self.is_camera:
                        print("[警告] 读不到帧，摄像头可能被占用或断连")
                    break
                self._frame_count += 1
                h_f, w_f = frame.shape[:2]

                # 推理（imgsz 越小越快；KunPeng 纯 CPU 用 320 提速明显）
                dets = infer(self.backend, self.model, frame, self.conf, self.imgsz)

                # 终端打印（复用 demo 的报告函数，便于现场调试）
                if self.is_camera:
                    draw_and_report(frame, dets, h_f, w_f)

                # 构造成结构化结果，供 HTTP / 决策层消费
                detections = []
                for (x1, y1, x2, y2, conf_i, name) in dets:
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2
                    detections.append({
                        "class": name,
                        "conf": round(conf_i, 3),
                        "bbox": [x1, y1, x2, y2],
                        "center": [cx, cy],
                        "direction": compute_direction(cx, cy, w_f, h_f),
                        "priority": name in PRIORITY_CLASSES,
                    })
                target = self._pick_target(detections)

                with self._lock:
                    self._latest = {
                        "frame_id": self._frame_count,
                        "timestamp": time.time(),
                        "detections": detections,
                        "target": target,
                    }
        except Exception as e:
            print(f"[错误] 工作线程异常：{e}")
        finally:
            if self.cap:
                self.cap.release()

    @staticmethod
    def _pick_target(detections):
        """在「优先可抓取类」里挑置信度最高的一个当作当前目标；没有则返回 None。"""
        prio = [d for d in detections if d["priority"]]
        if not prio:
            return None
        return max(prio, key=lambda d: d["conf"])

    def get_latest(self):
        """线程安全地取最新结果（返回副本，避免外部修改内部状态）。"""
        with self._lock:
            return dict(self._latest)

    def stop(self):
        """发停止信号并等待线程结束。"""
        self._running.clear()
        if self._thread:
            self._thread.join(timeout=3)
        fps = self._frame_count / (time.time() - self._start_time) if self._start_time else 0
        print(f"[统计] 共 {self._frame_count} 帧，平均 {fps:.1f} FPS（{self.backend} 后端）")
        print("[完成] 服务已停止")


# ===================================================================
# HTTP 接口（标准库实现，零依赖）
# ===================================================================
def make_handler(detector):
    """工厂函数：把 detector 实例绑进请求处理器，避免用全局变量。"""

    class Handler(BaseHTTPRequestHandler):
        # 静默默认访问日志（每帧都打会很吵）
        def log_message(self, *args):
            pass

        def _send_json(self, obj, code=200):
            body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")  # 允许跨域，方便网页/JS 调试
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path.startswith("/detections"):
                self._send_json(detector.get_latest())
            elif self.path == "/health":
                self._send_json({"status": "ok", "frame_id": detector.get_latest()["frame_id"]})
            elif self.path == "/" or self.path == "/index.html":
                self._send_html(detector.get_latest())
            else:
                self._send_json({"error": "not found"}, code=404)

        def _send_html(self, data):
            """一个简单的调试网页，浏览器打开就能看到当前识别情况。"""
            tgt = data.get("target")
            tgt_html = "无（未识别到可抓取目标）" if not tgt else \
                f"{tgt['class']} 置信度 {tgt['conf']} 方位 {tgt['direction']}"
            rows = "".join(
                f"<tr><td>{d['class']}</td><td>{d['conf']}</td>"
                f"<td>{d['direction']}</td><td>{'是' if d['priority'] else '否'}</td></tr>"
                for d in data["detections"]
            )
            html = f"""<html><head><meta charset="utf-8">
<title>阶段1 识别服务</title>
<style>body{{font-family:sans-serif;padding:20px}}
table{{border-collapse:collapse}}td,th{{border:1px solid #ccc;padding:4px 10px}}</style>
</head><body>
<h2>阶段1 识别服务</h2>
<p>当前目标：<b>{tgt_html}</b></p>
<p>最新帧号：{data['frame_id']} ｜ 检测数：{len(data['detections'])}</p>
<table><tr><th>类别</th><th>置信度</th><th>方位</th><th>优先</th></tr>
{rows}</table>
<p><a href="/detections">查看原始 JSON</a></p>
</body></html>"""
            body = html.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    return Handler


# ===================================================================
# 主程序
# ===================================================================
def main():
    parser = argparse.ArgumentParser(description="阶段1 常驻识别服务 (KunPeng + GF100)")
    parser.add_argument("--source", default="0",
                        help="输入源：摄像头索引(默认0) / 图片路径 / 视频路径")
    parser.add_argument("--model", default="yolov8n.pt",
                        help="模型路径：.pt 或 .onnx，默认 yolov8n.pt")
    parser.add_argument("--conf", type=float, default=0.4, help="置信度阈值，默认 0.4")
    parser.add_argument("--imgsz", type=int, default=320,
                        help="推理输入分辨率，默认 320（KunPeng 纯 CPU 实测约 7 FPS）")
    parser.add_argument("--width", type=int, default=640, help="摄像头分辨率宽")
    parser.add_argument("--height", type=int, default=480, help="摄像头分辨率高")
    parser.add_argument("--host", default="0.0.0.0",
                        help="HTTP 监听地址，默认 0.0.0.0（允许局域网访问）")
    parser.add_argument("--port", type=int, default=8080, help="HTTP 端口，默认 8080")
    args = parser.parse_args()

    det = Detector(args.model, args.source, args.imgsz, args.conf,
                  args.width, args.height)
    det.start()

    server = ThreadingHTTPServer((args.host, args.port), make_handler(det))
    print(f"[服务] 识别服务已启动：http://{args.host}:{args.port}/detections")
    print(f"[服务] 浏览器调试页：http://<板子IP>:{args.port}/")
    print("[服务] 按 Ctrl+C 停止")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[退出] 收到 Ctrl+C，停止服务")
    finally:
        det.stop()
        server.shutdown()


if __name__ == "__main__":
    main()
