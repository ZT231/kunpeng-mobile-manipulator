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

对外接口（全是 GET，浏览器直接能打开）：
  /detections   最新识别结果 JSON（决策层 / 网页叠加层用）
  /video        MJPEG 实时画面（网页 <img src> 用，无摄像头也能先看格式）
  /health       健康检查 + 当前帧号
  /             调试表格页
为什么 /video 只发「原始画面」不烧检测框：网页前端有可开关的叠加层，
图片里再画一层会重复，而且叠加层能点击、能单独关掉，比烧进 JPEG 灵活。

【B 方案 · 画面与推理解耦（2026-11-07 实机 6.8 FPS「卡成 PPT」后加）】
  ① 用了什么语法/技术：threading.Lock 快照 + 取模抽帧（%）+ 结果保持（hold），
     另加一个「帧号 + JPEG 一起取」的 get_snapshot() 方法供推流按帧号去重。
  ② 为什么这么设计：鲲鹏 920 四核纯 CPU 跑 YOLOv8n 约 140ms/帧，
     原来每帧都推理 → MJPEG 被锁死在 ~7 FPS；而网页前端本来就每 400ms
     （2.5Hz）才轮询一次检测框，每帧推理是纯浪费。改成「画面每帧都推、
     识别每 N 帧做一次、中间帧沿用上次框」后，画面帧率由「采集 + JPEG
     编码」决定（约 20~25 FPS），识别仍 2~5Hz，正好匹配前端轮询节奏。
  ③ 不这么写会出什么问题：只降 --imgsz / 限线程（方案 A）治标不治本，
     推理仍占着每一帧；若只改推流 sleep 不去重，同一张 JPEG 会被重复推给
     浏览器，白吃 Wi-Fi 带宽还更顿。必须「抽帧 + 按帧号去重」一起做。

  ⚠️ 诚实边界：本实现是单线程循环，做推理的那一帧仍会阻塞 ~140ms，
     所以是「平均 ~20 FPS、每 N 帧顿一下」，不是均匀 30 FPS。要彻底均匀
     得把推理拆到独立线程（生产者-消费者），属后续 B+，当前未做。

【C 方案 · 采集格式 + 画质（2026-11-07 实机：GF100 切 MJPG 10→22FPS 后加）】
  ① 用了什么语法/技术：cv2.VideoCapture(src, cv2.CAP_V4L2) 显式后端 +
     CAP_PROP_FOURCC 协商像素格式 + 启动回读 FOURCC/FPS/宽高并打印 +
     IMWRITE_JPEG_QUALITY 由 75 提到 90（全部做成命令行参数，可回退）。
  ② 为什么这么设计：B 方案把代码侧瓶颈全拿掉后，画面仍只有 10FPS；
     实机自测定位到「摄像头默认吐 YUYV 未压缩、USB2.0 带宽只够 10FPS」，
     同一颗 GF100 切到硬件 MJPEG 立刻 22FPS。糊的另一半原因是 MJPG 帧
     被 OpenCV 解码成 BGR 后又被 quality=75 压了一次（双重有损），提到 90
     减少第二次损失。
  ③ 不这么写会出什么问题：不显式指定 V4L2，pip 版 OpenCV 5.x 在 V4L2
     失败后会回退到 obsensor 后端刷无关报错；不回读实际协商值，会把
     「我们请求的 MJPG」误当成「摄像头真的给了 MJPG」（cap.set 不保证生效）；
     不把参数做成可回退，遇到不支持 MJPG 的固件就没法一键切回 YUYV。

  ⚠️ 诚实边界：这改的是「画面」帧率与清晰度，YOLO 推理仍 2~5Hz（抽帧）、
     检测框滞后；不能写「实时视觉」。720P 只保留成 --width/--height 现场可试，
     不设为默认（没实测过帧率，贸然翻倍分辨率可能把 22FPS 打回 10FPS）。
     「透传摄像头 JPEG 跳过 decode+encode」与「推理需要 BGR 帧」冲突，未做。
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


def fourcc_to_str(value):
    """把 OpenCV 的 CAP_PROP_FOURCC 数值转回可读的四个字符（如 'MJPG'）。

    大白话：驱动回的是一个整数，机器好算、人看不懂；把它拆成 4 个字节、
            每个字节当一个字母拼起来，就成了 'MJPG' / 'YUYV' 这种名字。
    规范定义：FourCC（four-character code）是把 4 个 ASCII 字符按小端序
            打包进一个 32 位整数；依次取 value 的第 0/8/16/24 位开始的
            各一个字节，即可得到原来的 4 个字符。
    """
    try:
        v = int(value)
        # -1 或 0xFFFFFFFF 这类是「还没协商好 / 读不到」的哨兵值，别硬转。
        if v < 0 or v > 0xFFFFFFFF:
            return "无效"
        chars = []
        for i in range(4):
            c = (v >> (8 * i)) & 0xFF
            # 不可打印字符说明这个值不是合法 FourCC，直接判无效。
            if c < 32 or c > 126:
                return "无效"
            chars.append(chr(c))
        return "".join(chars).strip() or "无效"
    except (ValueError, TypeError, OverflowError):
        return "无效"


class Detector:
    """常驻识别器：后台线程跑摄像头 + 推理，对外提供最新结构化结果。"""

    def __init__(self, model_path, source=0, imgsz=320, conf=0.4,
                 width=640, height=480, detect_every=4,
                 fourcc="MJPG", jpeg_quality=90):
        self.model_path = model_path
        self.source = source
        self.is_camera = str(source).isdigit()
        self.imgsz = imgsz
        self.conf = conf
        self.width = width
        self.height = height

        # 【B 方案】每 N 帧才做一次 YOLO 推理，中间帧复用上一次结果。
        # 大白话：摄像头一秒吐几十张照片，没必要每张都认一遍物体——
        #         人眼看着物体没动，框也没必要每帧重画。
        # 规范定义：检测抽帧（detection decimation）+ 结果保持（result hold），
        #         以固定步长对连续帧抽样推理，是视频检测常用的吞吐/延迟折中。
        self.detect_every = max(1, int(detect_every))

        # 【C 方案】采集像素格式 + JPEG 画质（都只影响「画面」，不影响 YOLO 推理）。
        # 大白话：fourcc 决定摄像头吐「生图」还是「压好的 JPEG」（管卡不卡）；
        #         jpeg_quality 决定我们再压一次时压多狠（管糊不糊，90 比 75 清楚）。
        # 规范定义：fourcc 是 V4L2 FourCC 四字符码，经 CAP_PROP_FOURCC 协商；
        #         jpeg_quality 对应 cv2.IMWRITE_JPEG_QUALITY（1~100，越大越清楚也越大）。
        # 夹到合法范围：写 0 / 负数 / >100 会让 imencode 直接报错或行为异常。
        # AUTO 是「不主动设置、沿用摄像头默认协商值」，留给不支持 MJPG 的固件回退。
        self.fourcc = str(fourcc).upper()
        self.jpeg_quality = max(1, min(100, int(jpeg_quality)))

        self.backend = None
        self.model = None
        self.cap = None

        # 最新识别结果（线程安全：用锁保护）
        self._lock = threading.Lock()
        self._latest = {"frame_id": 0, "timestamp": 0.0,
                        "detections": [], "target": None,
                        "frame": {"w": width, "h": height}}
        # 中间帧要沿用的「上一次推理结果」，只在真推理的帧里更新。
        self._last_detections = []
        self._last_target = None

        # 最新一帧的原始 JPEG 快照，供 /video 推流用。
        # 为什么不重新开一次摄像头去读：摄像头是独占设备，推理线程已经在读了，
        # 再开会失败；「快照」模型还能让多个浏览器同时看而互不抢帧。
        self._jpeg = None
        # 这张 JPEG 属于第几帧 —— 推流靠它判重，避免同一张图重复发给浏览器。
        self._jpeg_frame_id = 0

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
        # 【C 方案】摄像头显式指定 V4L2 后端 + 主动协商像素格式。
        # 大白话：同一颗摄像头能吐两种「照片」——YUYV 是没压过的「生图」（又大又慢），
        #         MJPG 是摄像头自己先压好的 JPEG（又小又快）。GF100 实测 YUYV 只有
        #         10FPS、MJPG 有 22FPS，差别全在「生图太大、USB 线一秒搬不了几张」。
        # 规范定义：cv2.CAP_V4L2 显式指定 Video4Linux2 后端；CAP_PROP_FOURCC 用
        #         FourCC 四字符码协商像素格式（MJPG=Motion-JPEG / YUYV=未压缩 YUV4:2:2）。
        #         显式指定 V4L2 还能避免 pip 版 OpenCV 5.x 失败后回退到 obsensor 后端、
        #         刷一堆与本摄像头无关的报错。
        src = int(self.source) if self.is_camera else self.source
        if self.is_camera:
            self.cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
            # 顺序讲究：先协商 FOURCC 再设分辨率。切像素格式会触发 V4L2 重新 S_FMT，
            # 反过来做可能把刚设好的 MJPG 又打回 YUYV。AUTO 则完全不 set，沿用默认值。
            if self.fourcc != "AUTO":
                code = cv2.VideoWriter_fourcc(*self.fourcc)
                self.cap.set(cv2.CAP_PROP_FOURCC, code)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        else:
            # 图片 / 视频文件源：没有「像素格式协商」这回事，按老办法打开即可。
            self.cap = cv2.VideoCapture(src)
        if not self.cap.isOpened():
            raise RuntimeError(f"打不开输入源：{self.source}")
        print(f"[OK] 输入源已打开：{'摄像头'+str(self.source) if self.is_camera else self.source}")

        # 2b) 回读「实际协商值」并打印 —— cap.set() 只是「请求」，摄像头未必照办。
        #     诚实：一定把驱动真正给的值打出来，别拿我们「想要的值」当结果。
        #     这也是排障最快的一行日志：FOURCC 还是 YUYV 就知道帧率上不去了。
        if self.is_camera:
            real_fourcc = fourcc_to_str(self.cap.get(cv2.CAP_PROP_FOURCC))
            real_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            real_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            real_fps = self.cap.get(cv2.CAP_PROP_FPS)
            print(f"[采集] 实际协商：{real_w}x{real_h} / {real_fourcc} / "
                  f"CAP_PROP_FPS={real_fps:.1f}（请求 {self.width}x{self.height} / {self.fourcc}）")
            if self.fourcc != "AUTO" and real_fourcc != self.fourcc:
                print(f"[警告] 摄像头没接受 {self.fourcc}，仍是 {real_fourcc}；"
                      f"帧率可能上不去。可试 --fourcc AUTO 用回默认值，或换 --fourcc YUYV。")

        # 3) 启动工作线程
        self._running.set()
        self._start_time = time.time()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print("[服务] 后台推理线程已启动")

    def _loop(self):
        """工作线程主循环：不断读帧 -> 按需推理 -> 更新最新结果。

        【B 方案：采集/推流 与 推理 解耦】
        大白话：摄像头每吐一张照片就立刻压成 JPEG 发出去（画面因此能跑到
                摄像头原生帧率）；但「认物体」这个慢活每 N 帧才做一次，
                中间 N-1 帧直接沿用上一次的框。人眼看着是连续视频，
                检测框 2~5Hz 更新一次，够决策层用。
        规范定义：检测抽帧（detection decimation）+ 结果保持（result hold），
                把吞吐瓶颈从「每帧推理耗时」降到「每帧采集 + JPEG 编码耗时」。

        ⚠️ 诚实边界：单线程循环，做推理的那一帧仍会阻塞 ~140ms，
           所以是「平均 ~20 FPS、每 N 帧顿一下」，不是均匀 30 FPS。
        """
        try:
            while self._running.is_set():
                ret, frame = self.cap.read()
                if not ret:
                    if self.is_camera:
                        print("[警告] 读不到帧，摄像头可能被占用或断连")
                    break
                self._frame_count += 1
                h_f, w_f = frame.shape[:2]

                # ---- 第 1 步：每帧都压 JPEG（先做，保证拿到的是「原始画面」）----
                # 必须在 draw_and_report 之前：那个函数可能把检测框画进 frame，
                # 而网页前端自己有可开关的叠加层，图片里再烧一层就重复了。
                # 大白话：MJPEG 就是「一串连续的 JPEG 照片」，不需要视频编码器。
                jpg_bytes = None
                try:
                    # quality 用 self.jpeg_quality（默认 90，旧版写死 75）：
                    # MJPG 帧被 OpenCV 解码成 BGR 后这里又压一次，属「双重有损」，
                    # 把它调高主要是减少第二次压缩带来的糊。
                    ok_jpg, buf = cv2.imencode(
                        ".jpg", frame,
                        [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
                    if ok_jpg:
                        jpg_bytes = buf.tobytes()
                except Exception:
                    pass          # 偶发编码失败不影响识别主流程

                # ---- 第 2 步：按需推理（默认每 4 帧一次）----
                # 取模判定：帧号 1、5、9、13... 才推理（detect_every=4 时），
                # 第 1 帧自动包含在内，不会出现「从没推理过、detections 一直是空」。
                # 为什么写 (frame_count - 1) % N == 0 而不是 frame_count % N == 1：
                # 后者在 N=1 时永远为假（n % 1 恒等于 0），会把「每帧推理」这个
                # 合法配置变成「从不推理」。减 1 后 N=1 时恒为 0，自然退化为全推理。
                # 另注意别用「上次结果为空就补推理」那种写法：画面里没物体时
                # 会退化成每帧都推理，跳帧就白做了。
                need_infer = ((self._frame_count - 1) % self.detect_every == 0)

                if need_infer:
                    dets = infer(self.backend, self.model, frame,
                                 self.conf, self.imgsz)
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
                    self._last_detections = detections
                    self._last_target = target
                    # 终端报告也只在真推理的帧打印：少刷屏，也避免
                    # SSH PTY 缓冲写满后回压（back-pressure）住推理线程。
                    if self.is_camera:
                        draw_and_report(frame, dets, h_f, w_f)
                else:
                    # 中间帧：结果保持，沿用上一次推理的框与目标。
                    detections = self._last_detections
                    target = self._last_target

                # ---- 第 3 步：同锁更新画面 + 结果 ----
                with self._lock:
                    # 画面和识别结果在同一把锁里更新，保证 /video 与 /detections
                    # 取到的是同一帧，检测框不会框到上一张图上。
                    if jpg_bytes:
                        self._jpeg = jpg_bytes
                        self._jpeg_frame_id = self._frame_count
                    self._latest = {
                        "frame_id": self._frame_count,
                        "timestamp": time.time(),
                        "detections": detections,
                        "target": target,
                        # bbox 是像素坐标，网页要靠 frame.w/h 把检测框换算成百分比。
                        # 不能用 __init__ 里请求的 width/height：cap.set() 不保证摄像头真的切过去，
                        # 必须上报这一帧的真实尺寸，否则框会整体偏移。
                        # 复用帧沿用上次推理的 bbox：本项目采集分辨率固定 640x480，
                        # 尺寸不变所以直接复用是对的；若运行时改分辨率需按比例缩放 bbox。
                        "frame": {"w": w_f, "h": h_f},
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

    def get_jpeg(self):
        """线程安全地取最新一帧 JPEG 快照；还没出帧时返回 None。"""
        with self._lock:
            return self._jpeg

    def get_snapshot(self):
        """线程安全地取「帧号 + 这一帧的 JPEG」，供 MJPEG 推流去重。

        大白话：一次把「第几张」和「这张照片」一起拿出来，推流那边就能
                认准帧号，一张照片只发一次。
        规范定义：两个字段必须在同一把锁的临界区内读出，保证快照的
                原子性；分开取会出现「第 N 张图配上第 N+1 的帧号」，
                去重判断就失效了（这是典型的竞态条件 / race condition）。
        """
        with self._lock:
            return self._jpeg_frame_id, self._jpeg

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
            elif self.path == "/video":
                self._send_mjpeg()
            elif self.path == "/" or self.path == "/index.html":
                self._send_html(detector.get_latest())
            else:
                self._send_json({"error": "not found"}, code=404)

        def _send_mjpeg(self):
            """把最新一帧 JPEG 持续推给浏览器（MJPEG 视频流）。

            大白话：摄像头每秒吐很多张照片，我们开一条「不会结束」的 HTTP 响应，
            来一张发一张，浏览器 <img> 收到就显示，看起来就是视频。
            规范定义：HTTP 长连接 + `multipart/x-mixed-replace; boundary=frame`，
            每个 part 带 `Content-Type: image/jpeg`、`Content-Length` 和二进制负载。

            为什么读「快照」而不是现场开摄像头：摄像头是独占设备，推理线程
            已经占用了；快照还能让多个浏览器同时看而互不抢帧、不堆积延迟。
            """
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                last_sent_id = -1
                while True:
                    fid, jpg = detector.get_snapshot()
                    if jpg and fid != last_sent_id:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(b"Content-Length: " +
                                         str(len(jpg)).encode("ascii") + b"\r\n")
                        self.wfile.write(b"\r\n" + jpg + b"\r\n")
                        self.wfile.flush()
                        last_sent_id = fid
                    # 20ms 轮询一次；没新帧就跳过，不再把同一张图重复发给浏览器。
                    # 大白话：以前每 66ms 无脑发一次，识别慢时同一张照片会连发两次；
                    #         现在认准帧号，一张只发一次。
                    # 规范定义：以单调递增的 frame_id 做幂等去重，避免消费端
                    #         速率高于生产端时产生重复帧（duplicate frame）。
                    time.sleep(0.02)
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass                  # 浏览器关页面 / 断流，正常收尾
            finally:
                # 没有 Content-Length 的分块响应结束时必须关连接，
                # 否则浏览器会一直等下去。
                self.close_connection = True

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
    parser.add_argument("--fourcc", type=str.upper, default="MJPG",
                        choices=["MJPG", "YUYV", "AUTO"],
                        help="摄像头像素格式（默认 MJPG）：MJPG=硬件 JPEG 压缩"
                             "（GF100 实测约 22FPS）；YUYV=未压缩（实测仅约 10FPS）；"
                             "AUTO=不主动设置、沿用摄像头默认协商值。只影响画面帧率，"
                             "不改变 YOLO 推理帧率")
    parser.add_argument("--jpeg-quality", type=int, default=90,
                        help="推流 JPEG 编码质量 1~100（默认 90，比旧版 75 更清楚）；"
                             "越高越清楚，但单帧更大、编码更慢、帧率可能略降")
    parser.add_argument("--detect-every", type=int, default=4,
                        help="每 N 帧做一次 YOLO 推理（默认 4）；画面仍每帧推流，"
                             "中间帧复用上次检测框。数值越大画面越流畅、框更新越慢")
    parser.add_argument("--host", default="0.0.0.0",
                        help="HTTP 监听地址，默认 0.0.0.0（允许局域网访问）")
    parser.add_argument("--port", type=int, default=8080, help="HTTP 端口，默认 8080")
    args = parser.parse_args()

    det = Detector(args.model, args.source, args.imgsz, args.conf,
                   args.width, args.height, args.detect_every,
                   args.fourcc, args.jpeg_quality)
    det.start()

    server = ThreadingHTTPServer((args.host, args.port), make_handler(det))
    print(f"[服务] 识别服务已启动：http://{args.host}:{args.port}/detections")
    print(f"[服务] 抽帧推理：每 {det.detect_every} 帧识别一次，画面每帧推流并按帧号去重")
    print(f"[服务] 采集格式：{det.fourcc}（实机 MJPG≈22FPS / YUYV≈10FPS）；"
          f"JPEG 质量 {det.jpeg_quality}；以 [采集] 实际协商一行为准，画质只影响画面")
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
