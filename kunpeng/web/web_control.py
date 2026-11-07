# -*- coding: utf-8 -*-
"""
手机网页遥控（S5，W 模式 Web 控制）

手机浏览器 --WiFi--> 本服务(Flask) --通道--> STM32F407
通道二选一：
  * 串口：USB-TTL/USART1（PC 串口）
  * WiFi：ESP-01S TCP server（手机/电脑连 ESP 自建热点后直连 192.168.4.1:9000）

帧格式与上位机一致：$<CMD>[,<ARG>...]*<CK><CR><LF>，CK 为 $ 与 * 之间所有字节的 XOR（2 位大写十六进制）
与 kunpeng/robot/arm_host.py 同一套协议约定。

依赖：
  pip install flask pyserial

用法：
  python web_control.py COM7 --baud 115200          # 经 PC 串口控制
  python web_control.py COM7 --camera 0             # PC 阶段：顺便开本地摄像头出 MJPEG 画面
  python web_control.py /dev/ttyUSB0 --video-url http://127.0.0.1:8080/video
                                                    # 鲲鹏阶段：画面来自 detector_service.py
  python web_control.py --wifi 192.168.4.1:9000     # 经 WiFi TCP 控制（ESP 自建热点）
  python web_control.py --host 0.0.0.0 --port 5000  # 让手机通过同一 WiFi 访问

视频是「另一条链路」：摄像头 --> MJPEG --> 浏览器 <img>，和 STM32 串口的
PING/MOVE/STATE 完全独立。所以页面怎么改都不会影响已经跑通的控制链路。

注意（2026-11-04 适配鲲鹏项目）：本项目 STM32 执行端经串口直连鲲鹏（默认 serial 模式即可，
端口传 /dev/ttyXXX，如 /dev/ttyUSB0 或板载 UART）；旧项目的 ESP-01S WiFi 模式在本项目由
ESP32-S3 替代（语音+联网），此处 wifi / tcp-server 模式仅作旧项目兼容保留，新项目请走 serial。
脉宽范围已对齐固件 main.h 的软限位（J6 上限 2000，为另行收窄的安全限位；
物理实测闭合端点 2100 仍记在 arm_params.json / docs/舵机标定记录.md 第六节，两者不是一回事）。
"""

import argparse
import json
import logging
import os
import socket
import sys
import threading
import time
import urllib.request

try:
    import serial
except ImportError:
    print("缺少依赖 pyserial，请先执行： pip install pyserial")
    sys.exit(1)

try:
    from flask import Flask, Response, jsonify, render_template_string, request
except ImportError:
    print("缺少依赖 flask，请先执行： pip install flask")
    sys.exit(1)

# OpenCV 是「可选」依赖：只有 --camera 本地预览时才需要。
# 为什么不放在最上面硬 import：鲲鹏阶段画面由 detector_service.py 提供，
# 本机可能没装 cv2，不能因为一个可选功能让整个网页起不来。
try:
    import cv2
except ImportError:
    cv2 = None

CHANNEL_COUNT = 6
JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]

# 页面滑杆脉宽范围：(MIN, MAX, 初始位)。
# MIN/MAX 来自 Core/Inc/main.h 的 SERVO_Jx_MIN/MAX（软限位，两边必须逐路一致）。
# 第三列是「初始位」= 上电 / HOME 命令的目标脉宽，也是滑杆默认值和
# 「初始位」按钮的基准；必须与 main.h 的 SERVO_Jx_INIT 逐路一致，
# 否则网页按钮会把臂送到一个和固件不一样的位置。
PULSE_RANGES = [
    (800, 2300, 1580),   # J1
    (750, 2150, 1900),   # J2
    (520, 2500, 2400),   # J3
    (500, 1850, 700),    # J4（软限位收窄；标定映射仍 520~2500，见 arm_params.json）
    (500, 2500, 1450),   # J5
    (900, 2000, 1500),   # J6（软限位收窄；物理实测闭合端点 2100 仍记在 arm_params.json）
]

JOINT_LIMITS = {
    JOINT_NAMES[i]: PULSE_RANGES[i] for i in range(CHANNEL_COUNT)
}


# ---------------------------------------------------------------------------
# A 方案：点击画面取点 -> 逆解 -> 六路脉宽 -> JOINT 下发（可选功能）
#
# ① 用了什么语法：模块级可选 import + 字典配置 + 纯函数（不碰任何已有全局状态）。
# ② 为什么这么设计：逆解依赖 arm_ik.py / arm_params.json 两个文件，鲲鹏板最小上传集
#    本来不含它们；做成「找不到就 _IK=None、接口返回明确原因」的软降级，不传文件
#    也不会影响已跑通的 PING/MOVE/STATE/视频/检测框链路，删掉文件即可回滚。
# ③ 不这么写会怎样：若在模块顶部硬 import，板上缺文件会让整个 Flask 起不来，
#    连网页都打不开 —— 一个可选功能拖垮主链路，是典型的耦合事故。
# ---------------------------------------------------------------------------
_IK = None                 # ArmIK 实例；None = 未就绪
_IK_ERROR = ""             # 未就绪原因，给页面看
_IK_MAP = {
    "span_x": 30.0,        # 画面宽度对应工作台多少 cm（手工标定，非相机内参）
    "span_y": 22.5,        # 画面高度对应工作台多少 cm（4:3 时约 span_x*3/4）
    "center_x": 0.0,       # 画面中心对应的工作台 x（cm）
    "center_y": 18.0,      # 画面中心对应的工作台 y（cm），正前方
    "z": 2.0,              # 抓取高度 cm（init_ik 成功后用 arm_params.json 的 grab_height 覆盖）
    "tilt": 0.0,           # 夹爪前倾角（度），0 = 垂直向下
    "claw": 0.0,           # J6 夹爪开合（度），0 = 闭合
    "wrist": 0.0,          # J5 夹爪朝向（度）
}


def init_ik(ik_dir):
    """尝试加载逆解模块；失败只记原因、绝不抛异常拖垮网页服务。"""
    global _IK, _IK_ERROR
    d = os.path.expanduser(ik_dir or "~/kunpeng_vision")
    try:
        if d not in sys.path:
            sys.path.insert(0, d)
        from arm_ik import ArmIK
        _IK = ArmIK()          # 默认读同目录 arm_params.json；标定文件全程只读
        _IK_ERROR = ""
        _IK_MAP["z"] = _IK.grab_h
        return True
    except Exception as e:
        _IK = None
        _IK_ERROR = "未找到逆解模块（%s）：%s" % (d, e)
        return False


def pixel_to_table(u, v, w, h):
    """像素 (u,v) -> 工作台 (x,y) cm。

    大白话：把画面按比例铺到桌面上，中心对准 (center_x, center_y)，
    往右 x 变大、往上 y 变大（正前方）。这是「平面约束单目定位」的退化解，
    z 由外部给定，不是相机测出来的。
    规范定义：已知 Z=const 的平面，用线性比例（单应性矩阵的仿射近似）把像素
    反投影到该平面；无相机内参与畸变校正，只适合固定视角的桌面示意。
    """
    if not w or not h:
        raise ValueError("图像尺寸为空，等画面 LIVE 后再取点")
    x = _IK_MAP["center_x"] + (float(u) / float(w) - 0.5) * _IK_MAP["span_x"]
    y = _IK_MAP["center_y"] + (0.5 - float(v) / float(h)) * _IK_MAP["span_y"]
    return x, y


def solve_to_pulses(x, y):
    """工作台 (x,y) -> 6 路脉宽，并裁到固件软限位 PULSE_RANGES。

    返回 (pulses, clamped)：clamped 列出被裁的关节，供页面提示「目标超出可达范围」。
    为什么 Python 侧也裁一遍：固件 cmd_joint 也会 clamp_pulse，但那是静默裁剪；
    这边显式裁并回报，人才能知道 IK 解其实越界了，而不是以为「算得很准」。
    """
    if _IK is None:
        raise RuntimeError(_IK_ERROR or "逆解模块未就绪")
    angles = _IK.solve(x, y, z=_IK_MAP["z"], wrist=_IK_MAP["wrist"],
                       claw=_IK_MAP["claw"], tilt=_IK_MAP["tilt"])
    raw = _IK.to_pulses(angles)
    pulses, clamped = [], []
    for i, p in enumerate(raw):
        lo, hi, _ = PULSE_RANGES[i]
        q = max(lo, min(hi, int(p)))
        if q != int(p):
            clamped.append(JOINT_NAMES[i])
        pulses.append(q)
    return pulses, clamped


# ---------------------------------------------------------------------------
# 协议层（与 arm_host.py 一致）
# ---------------------------------------------------------------------------
def compute_ck(payload: str) -> int:
    ck = 0
    for ch in payload:
        ck ^= ord(ch)
    return ck & 0xFF


def make_frame(payload: str) -> bytes:
    ck = compute_ck(payload)
    return ("$%s*%02X\r\n" % (payload, ck)).encode("ascii")


# ---------------------------------------------------------------------------
# 串口封装（后台读取线程，MOVE 只写不等，避免拖动时被应答阻塞卡顿）
# ---------------------------------------------------------------------------
class SerialHost:
    def __init__(self, port, baud=115200):
        self.port = port
        try:
            self.ser = serial.Serial(port, baud, timeout=0.2)
        except Exception as e:
            raise RuntimeError("无法打开串口 %s：%s" % (port, e))
        self.wlock = threading.Lock()
        self.rlock = threading.Lock()
        self.rxq = []            # 后台线程解析出的完整应答行（FIFO）
        self._reader_alive = True
        self._reader = threading.Thread(target=self._rx_loop, daemon=True)
        self._reader.start()
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        try:
            self.send("LOG,0")   # 关闭周期遥测，保持 request-response 干净
        except Exception:
            pass

    def _rx_loop(self):
        """后台不断收字节，按 \\r\\n 组装成行放入 rxq。"""
        line = bytearray()
        while self._reader_alive:
            try:
                ch = self.ser.read(1)
            except Exception:
                break
            if not ch:
                continue
            if ch in (b"\r", b"\n"):
                if line:
                    s = bytes(line).decode("ascii", errors="replace").strip()
                    # 只收 "$" 开头的协议帧。printf 重定向到 huart1 与协议共用链路，
                    # 上电 banner / RC522·KEY0 提示 / 每 3s 的 "STATE ... | PULSE ..."
                    # 及 ESP AT 回显都是裸文本；必须丢弃，否则 send() 会误取为应答导致解析失败。
                    if s.startswith("$"):
                        s = s[1:]                 # 去掉帧头 $
                        if "*" in s:
                            s = s.split("*")[0]   # 去掉 *CK 校验段
                        with self.rlock:
                            self.rxq.append(s)
                    line = bytearray()
                continue
            line += ch

    def close(self):
        self._reader_alive = False
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        finally:
            self.ser = None
            self._reader = None

    def _write(self, payload):
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("串口已断开")
        self.ser.write(make_frame(payload))

    def send(self, payload, timeout=0.6):
        """写帧并等待一条新应答（MOVE 等无应答命令由调用侧走 fire 短路）。"""
        with self.wlock:
            with self.rlock:
                self.rxq.clear()          # 丢弃历史残留（含 MOVE 应答），保证读到的就是本次应答
            self._write(payload)
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self.rlock:
                if self.rxq:
                    return self.rxq.pop(0)
            time.sleep(0.005)
        return ""

    def fire(self, payload):
        """只写不等应答：供 MOVE 拖动用，绝不阻塞。"""
        with self.wlock:
            self._write(payload)

    def cmd(self, payload):
        return self.send(payload)


# ---------------------------------------------------------------------------
# WiFi TCP 封装（ESP-01S CIPSERVER，接口与 SerialHost 一致）
# 注意：ESP 非透传模式，STM32->手机 方向应答由 ESP 自动转发到 TCP 连接
# ---------------------------------------------------------------------------
class TcpHost:
    def __init__(self, host, port, timeout=0.2):
        self.addr = "%s:%d" % (host, port)
        try:
            self.sock = socket.create_connection((host, port), timeout=3.0)
        except Exception as e:
            raise RuntimeError("无法连接 WiFi TCP %s：%s" % (self.addr, e))
        self.sock.settimeout(timeout)
        self.buf = b""
        self.wlock = threading.Lock()
        self.rlock = threading.Lock()
        self.rxq = []            # 后台线程解析出的完整应答行（FIFO）
        self._reader_alive = True
        self._reader = threading.Thread(target=self._rx_loop, daemon=True)
        self._reader.start()
        try:
            self.send("LOG,0")   # 关闭周期遥测，保持 request-response 干净
        except Exception:
            pass

    def _rx_loop(self):
        """后台不断收字节，按 \\r\\n 组装成行放入 rxq。"""
        while self._reader_alive:
            try:
                data = self.sock.recv(256)
            except socket.timeout:
                continue
            except Exception:
                break
            if not data:
                continue
            self.buf += data
            while b"\n" in self.buf:
                line, _, self.buf = self.buf.partition(b"\n")
                s = line.decode("ascii", errors="replace").strip().strip("\r")
                # 只收 "$" 开头的协议帧。printf 重定向到 huart1 与协议共用链路，
                # 上电 banner / RC522·KEY0 提示 / 每 3s 的 "STATE ... | PULSE ..."
                # 及 ESP AT 回显都是裸文本；必须丢弃，否则 send() 会误取为应答导致解析失败。
                if s.startswith("$"):
                    s = s[1:]                 # 去掉帧头 $
                    if "*" in s:
                        s = s.split("*")[0]   # 去掉 *CK 校验段
                    with self.rlock:
                        self.rxq.append(s)

    def close(self):
        self._reader_alive = False
        try:
            self.sock.close()
        except Exception:
            pass

    def _write(self, payload):
        if not self.sock:
            raise RuntimeError("连接已断开")
        self.sock.sendall(make_frame(payload))

    def send(self, payload, timeout=0.6):
        """写帧并等待一条新应答（MOVE 等无应答命令由调用侧走 fire 短路）。"""
        with self.wlock:
            with self.rlock:
                self.rxq.clear()
            self._write(payload)
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self.rlock:
                if self.rxq:
                    return self.rxq.pop(0)
            time.sleep(0.005)
        return ""

    def fire(self, payload):
        """只写不等应答：供 MOVE 拖动用，绝不阻塞。"""
        with self.wlock:
            self._write(payload)

    def cmd(self, payload):
        return self.send(payload)


class TcpServerHost:
    """电脑做 TCP server：监听端口，接受 ESP-01S 主动连入，接口与 SerialHost/TcpHost 一致。

    架构：ESP(AP 热点) --CIPSTART--> 本机 TCP server --透传--> UART 双向往来。
    透传模式下 ESP 会把 TCP 收到的字节原样发到 STM32，STM32 的应答原样回发 TCP。
    """

    def __init__(self, listen_port, timeout=1.0):
        self.addr = "0.0.0.0:%d" % listen_port
        self._port = listen_port
        self._timeout = timeout      # 存成实例属性：_accept_loop 是另一个线程，取不到构造函数的局部参数
        self.sock = None
        self.buf = b""
        self.wlock = threading.Lock()
        self.rlock = threading.Lock()
        self.rxq = []            # 后台线程解析出的完整应答行（FIFO）
        self._reader_alive = False
        self._reader = None
        # 监听 socket 与 accept 线程：ESP 起来后会自动连入
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.settimeout(1.0)
        self.srv.bind(("0.0.0.0", listen_port))
        self.srv.listen(1)
        self._acc_alive = True
        self._acc = threading.Thread(target=self._accept_loop, daemon=True)
        self._acc.start()

    def _accept_loop(self):
        """后台接受 ESP 连入；连上后启动读线程，并把响应行放进 rxq。"""
        while self._acc_alive:
            try:
                conn, _ = self.srv.accept()
            except socket.timeout:
                continue
            except Exception:
                break
            if not self._acc_alive:
                conn.close()
                break
            with self.wlock:
                if self.sock is not None:
                    try:
                        self.sock.close()
                    except Exception:
                        pass
                self.sock = conn
                self.sock.settimeout(self._timeout)
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._reader_alive = True
            self._reader = threading.Thread(target=self._rx_loop, daemon=True)
            self._reader.start()

    def _rx_loop(self):
        """后台不断收字节，按 \\r\\n 组装成行放入 rxq。"""
        while self._reader_alive:
            try:
                data = self.sock.recv(512)
            except socket.timeout:
                continue
            except Exception:
                break
            if not data:
                break
            self.buf += data
            while b"\n" in self.buf:
                line, _, self.buf = self.buf.partition(b"\n")
                s = line.decode("ascii", errors="replace").strip().strip("\r")
                # 只收 "$" 开头的协议帧。printf 重定向到 huart1 与协议共用链路，
                # 上电 banner / RC522·KEY0 提示 / 每 3s 的 "STATE ... | PULSE ..."
                # 及 ESP AT 回显都是裸文本；必须丢弃，否则 send() 会误取为应答导致解析失败。
                if s.startswith("$"):
                    s = s[1:]                 # 去掉帧头 $
                    if "*" in s:
                        s = s.split("*")[0]   # 去掉 *CK 校验段
                    with self.rlock:
                        self.rxq.append(s)

    def _write(self, payload):
        if not self.sock:
            raise RuntimeError("无设备连入")
        self.sock.sendall(make_frame(payload))

    def send(self, payload, timeout=0.6):
        with self.wlock:
            with self.rlock:
                self.rxq.clear()
            self._write(payload)
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self.rlock:
                if self.rxq:
                    return self.rxq.pop(0)
            time.sleep(0.005)
        return ""

    def fire(self, payload):
        with self.wlock:
            self._write(payload)

    def cmd(self, payload):
        return self.send(payload)

    def close(self):
        self._acc_alive = False
        self._reader_alive = False
        try:
            self.srv.close()
        except Exception:
            pass
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
        self.sock = None


_HOST = None
_HOST_STATE = {"connected": False, "port": "", "error": ""}
_MODE = "serial"          # "serial" / "wifi" / "wifi_server"
_WIFI_ADDR = ""

# 视频链路的全局配置（与串口链路完全隔离，改这些不会影响控制）
_CAM = None               # 本地摄像头推流器（仅 --camera 时创建）
_VIDEO_URL = ""           # 页面 <img> 实际用的地址："/video" 或外部 MJPEG 地址
_DETECTIONS_URL = ""      # 识别结果 JSON 地址；由本服务代理转发，避免浏览器跨域


class CameraStream:
    """本地摄像头 MJPEG 推流器（PC 阶段 --camera 用）。

    小白视角：摄像头每秒吐很多张「照片」，我们开一个后台线程不停地取最新一张、
    压成 JPEG 存起来；网页来要画面时，就把最新那张送出去。这样即使有 3 个浏览器
    同时看，也只开了一个摄像头（摄像头是独占设备，开两次会失败）。

    规范定义：线程间共享 self._jpeg 用 threading.Lock 做互斥（避免读线程读到写了一半
    的字节）；数据生产/消费解耦为「最新帧快照」模型，消费慢时自动丢旧帧，不堆积延迟。
    """

    def __init__(self, source, width=640, height=480, fps=15):
        if cv2 is None:
            raise RuntimeError("未安装 opencv-python，无法启用 --camera")
        src = int(source) if str(source).isdigit() else source
        self.cap = cv2.VideoCapture(src)
        if not self.cap.isOpened():
            raise RuntimeError("打不开摄像头 %s（检查是否被别的程序占用）" % source)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._lock = threading.Lock()
        self._jpeg = None
        self._alive = True
        self._period = 1.0 / max(1, fps)
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._alive:
            try:
                ok, frame = self.cap.read()
            except Exception:
                break
            if not ok:
                time.sleep(0.1)      # 摄像头暂时没帧，别空转烧 CPU
                continue
            # 压缩成 JPEG：MJPEG 就是「一串连续的 JPEG 图片」，不需要视频编码器
            ret, buf = cv2.imencode(".jpg", frame,
                                    [int(cv2.IMWRITE_JPEG_QUALITY), 75])
            if ret:
                with self._lock:
                    self._jpeg = buf.tobytes()
            time.sleep(self._period)

    def get_jpeg(self):
        with self._lock:
            return self._jpeg

    def stop(self):
        self._alive = False
        try:
            if self.cap:
                self.cap.release()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Flask
# ---------------------------------------------------------------------------
app = Flask(__name__)


# ---------------------------------------------------------------------------
# 日志过滤：只挡「访问日志」，保留启动提示与错误日志
# ---------------------------------------------------------------------------
# 为什么不用 setLevel(logging.ERROR)（旧写法）：
#   大白话：level 是「把整个喇叭音量拧到只报错误」——Werkzeug 启动时那句
#          "Running on http://...:5000" 也是用普通音量喊的，会被一起按没声。
#          Filter 是「给喇叭加个筛子」，只漏掉带 " - - [" 的访问日志那类句子。
#   规范定义：logging.Logger.level 是该 logger 的全局阈值，低于它的记录会被
#          isEnabledFor() 判 False 丢弃；logging.Filter.filter() 按单条
#          LogRecord 的 getMessage() 返回 bool，粒度是消息级、不改变 level，
#          故启动 banner（INFO）、开发服务器 WARNING、真错误（ERROR）都保留。
class _NoAccessLog(logging.Filter):
    """前端 1s 轮询 /api/state + 拖动 60ms 节流发 MOVE，会把终端刷满
    "POST /api/... 200"；这里只丢这一类访问日志，其余原样打印。"""

    def filter(self, record):
        # 访问日志形如：127.0.0.1 - - [06/Nov/2026 16:20:00] "GET /api/state HTTP/1.1" 200 -
        # 启动提示 / WARNING / ERROR / traceback 都不含这个特征串，会正常打印。
        return " - - [" not in record.getMessage()


# 模块导入时就挂上（用 WSGI 或测试导入也生效），不是只在 main() 里加。
logging.getLogger("werkzeug").addFilter(_NoAccessLog())

PAGE = r"""
<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>机械臂遥控</title>
<style>
  :root{--bg:#0b0f1a;--sur:#151b2e;--sur2:#1c2340;--line:#2a3352;--tx:#e9ecf7;--muted:#8a94b8;--ac:#ffa940;--ok:#35d07f;--bad:#ff6262;--info:#7fd3ff;}
  *{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent;}
  body{background:var(--bg);color:var(--tx);font-family:-apple-system,"PingFang SC",Roboto,"Microsoft YaHei",sans-serif;padding:0 0 28px;}
  button,input,select{font-family:inherit;}
  .topbar{position:sticky;top:0;z-index:20;display:flex;align-items:center;gap:9px;flex-wrap:wrap;padding:10px 16px;background:#111726;border-bottom:1px solid var(--line);}
  .brand{font-size:15px;font-weight:600;display:flex;align-items:center;gap:8px;}
  .dot{width:9px;height:9px;border-radius:50%;background:#555;display:inline-block;flex:none;}
  .dot.on{background:var(--ok);}
  .pill{font-size:11px;padding:3px 9px;border-radius:20px;background:#1d2b45;color:var(--info);white-space:nowrap;}
  .pill.warn{background:#3a2a18;color:#ffc37a;}
  .spacer{flex:1;}
  .clk{font-family:ui-monospace,Menlo,monospace;font-size:12px;color:var(--muted);}
  .wrap{max-width:1520px;margin:0 auto;padding:14px 16px;}
  .grid{display:grid;grid-template-columns:minmax(0,1.35fr) minmax(0,1fr);gap:14px;align-items:start;}
  @media(max-width:980px){.grid{grid-template-columns:1fr;}}
  .panel{background:var(--sur);border:1px solid var(--line);border-radius:12px;padding:12px;margin-bottom:14px;}
  .panel h2{font-size:12px;color:var(--muted);letter-spacing:.5px;margin-bottom:10px;display:flex;justify-content:space-between;align-items:center;gap:8px;flex-wrap:wrap;}
  .tip{font-size:11px;color:#6f7aa0;font-weight:400;letter-spacing:0;}
  .row{display:flex;gap:8px;align-items:center;flex-wrap:wrap;}
  input[type=text],select{background:#0e1424;border:1px solid var(--line);color:var(--tx);border-radius:8px;padding:9px 10px;font-size:13px;flex:1;min-width:110px;}
  button{background:#232c4d;color:var(--tx);border:1px solid #33406b;border-radius:9px;padding:9px 12px;font-size:13px;font-weight:500;cursor:pointer;min-height:38px;flex:1;}
  button:active{filter:brightness(1.25);transform:scale(.98);}
  button.sm{min-height:32px;padding:6px 9px;font-size:12px;flex:none;}
  .ac{background:var(--ac);color:#111;border-color:var(--ac);}
  .ok{background:#1d4a2e;color:#b6ffcf;border-color:#2c6b44;}
  .bad{background:#4a1d1d;color:#ffb8b8;border-color:#7a3030;}
  .g{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px;}
  /* 标定参数：每个输入框上方挂一行常驻中文标签。为什么不用 placeholder 就算了——
     页面启动时 /api/ik/config 会把数值回填进 value，placeholder 就再也不显示了，
     只剩 5 个裸数字，根本看不出哪个是 span_x、哪个是 center_y。 */
  .cf{display:flex;flex-direction:column;gap:4px;min-width:0;}
  .cf label{font-size:11px;color:var(--muted);line-height:1.35;font-weight:400;}
  .cf label b{color:var(--tx);font-weight:600;}
  .cf input{width:100%;box-sizing:border-box;min-width:0;flex:none;}
  .vwrap{position:relative;aspect-ratio:4/3;background:#070b14;border:1px solid var(--line);border-radius:10px;overflow:hidden;display:flex;align-items:center;justify-content:center;}
  .vwrap img{width:100%;height:100%;object-fit:fill;display:block;}
  .vph{position:absolute;inset:0;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:8px;color:var(--muted);font-size:13px;text-align:center;padding:20px;}
  .vph .ic{width:46px;height:34px;border:2px solid #3a456b;border-radius:6px;position:relative;}
  .vph .ic::after{content:"";position:absolute;width:12px;height:12px;border:2px solid #3a456b;border-radius:50%;left:15px;top:9px;}
  .overlay{position:absolute;inset:0;pointer-events:none;}
  .dbox{position:absolute;border:2px solid var(--ac);border-radius:3px;}
  .dbox b{position:absolute;top:-19px;left:-2px;background:var(--ac);color:#111;font-size:11px;font-weight:500;padding:1px 5px;border-radius:3px;white-space:nowrap;}
  .cross{position:absolute;background:#ffffff33;pointer-events:none;}
  .vwrap.pickable img{cursor:crosshair;}
  .pick{position:absolute;width:14px;height:14px;border:2px solid var(--ok);border-radius:50%;transform:translate(-50%,-50%);box-shadow:0 0 0 1px #0009;pointer-events:none;display:none;}
  .badge{position:absolute;top:8px;left:8px;font-size:11px;padding:3px 9px;border-radius:20px;background:#3a1d1d;color:#ffb0b0;}
  .badge.live{background:#12402a;color:#8ff0bd;}
  .vmeta{position:absolute;right:8px;bottom:8px;font-size:11px;color:#c3cbe8;background:#0009;padding:2px 7px;border-radius:4px;}
  .vtools{display:flex;gap:6px;align-items:center;flex-wrap:wrap;margin-top:9px;}
  .vtools input{flex:1;min-width:180px;font-family:ui-monospace,Menlo,monospace;font-size:11px;}
  .chips{display:flex;gap:6px;flex-wrap:wrap;margin-top:9px;min-height:24px;}
  .chip{font-size:11px;padding:3px 9px;border-radius:20px;background:#1d2b45;color:var(--info);}
  .chip.hot{background:#3a2a18;color:#ffc37a;}
  .jcard{background:var(--sur2);border:1px solid var(--line);border-radius:9px;padding:9px 11px;margin-bottom:8px;}
  .jtop{display:flex;justify-content:space-between;align-items:baseline;margin-bottom:4px;}
  .jname{font-size:13px;font-weight:600;}
  .jlim{font-size:11px;color:var(--muted);}
  .jval{font-family:ui-monospace,Menlo,monospace;font-size:16px;color:var(--ac);}
  input[type=range]{width:100%;accent-color:var(--ac);height:22px;}
  .jbtns{display:flex;gap:6px;margin-top:2px;}
  .jbtns button{min-height:30px;padding:4px 8px;font-size:12px;}
  .metrics{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:10px;margin-bottom:14px;}
  @media(max-width:640px){.metrics{grid-template-columns:repeat(2,minmax(0,1fr));}.g{grid-template-columns:repeat(2,minmax(0,1fr));}}
  .metric{background:var(--sur);border:1px solid var(--line);border-radius:10px;padding:10px 12px;}
  .metric .lb{font-size:11px;color:var(--muted);margin-bottom:4px;}
  .metric .vl{font-size:21px;font-weight:600;font-family:ui-monospace,Menlo,monospace;}
  .estop{background:#4a1717;border:1px solid #a33838;color:#ffd0d0;border-radius:10px;min-height:100%;font-size:14px;font-weight:600;letter-spacing:1px;}
  .estop:active{background:#6b1f1f;}
  .logbox summary{cursor:pointer;font-size:12px;color:var(--muted);letter-spacing:.5px;list-style:none;}
  .logbox summary::before{content:"▸ ";color:var(--ac);}
  .logbox[open] summary::before{content:"▾ ";}
  #log{font-family:ui-monospace,Menlo,monospace;font-size:11px;line-height:1.6;white-space:pre-wrap;word-break:break-all;background:#080c17;border-radius:8px;padding:10px;max-height:190px;overflow:auto;margin-top:10px;color:#a9c6e8;}
</style>
</head>
<body>

<div class="topbar">
  <span class="dot on" id="led"></span>
  <span class="brand">鲲鹏机械臂控制台</span>
  <span class="pill" id="pillChan">串口 {{port}}</span>
  <span class="pill warn" id="sub">未连接</span>
  <span class="pill" id="pillMode">指令态</span>
  <span class="pill" id="pillCam">视频 未接入</span>
  <span class="spacer"></span>
  <span class="clk" id="clock">--:--:--</span>
</div>

<div class="wrap">
<div class="grid">

  <div>
    <section class="panel">
      <h2><span>眼在手相机 · GF100</span><span class="tip">MJPEG 实时流 · 检测框由识别服务叠加</span></h2>
      <div class="vwrap" id="vwrap">
        <img id="camImg" alt="摄像头实时画面">
        <div class="vph" id="camPh">
          <div class="ic"></div>
          <div>摄像头未接入</div>
          <div style="font-size:11px;color:#6f7aa0">PC 阶段加 --camera 0 ｜ 鲲鹏阶段加 --video-url http://板IP:8080/video</div>
        </div>
        <div class="cross" style="left:50%;top:0;bottom:0;width:1px"></div>
        <div class="cross" style="top:50%;left:0;right:0;height:1px"></div>
        <div class="overlay" id="overlay"></div>
        <div class="pick" id="pickMark"></div>
        <div class="badge" id="camBadge">OFFLINE</div>
        <div class="vmeta" id="vsize">--</div>
      </div>
      <div class="vtools">
        <button class="sm" onclick="shot()">截图</button>
        <button class="sm" onclick="fullscreen()">全屏</button>
        <button class="sm" id="btnOverlay" onclick="toggleOverlay()">叠加：开</button>
        <input type="text" id="streamUrl" placeholder="视频流地址，改完回车生效">
      </div>
      <div class="chips" id="detChips"><span class="chip">检测服务未配置</span></div>
    </section>

    <section class="panel">
      <h2><span>A 方案 · 点选抓取（开环）</span><span class="tip">平面约束单目定位，不是真 3D</span></h2>
      <div class="row">
        <label style="font-size:12px;display:flex;align-items:center;gap:6px;flex:none;color:#c3cbe8">
          <input type="checkbox" id="ikPickOn" style="flex:none;width:auto"> 允许点击画面取点
        </label>
        <span class="pill warn" id="ikState">检测中…</span>
      </div>
      <div style="font-size:11px;color:#ffc37a;margin-top:8px;line-height:1.6">
        ⚠ 只做「像素 → 桌面平面 → 逆解 → 脉宽」，没有深度相机、没有力控、没有关节回读。
        确认下发会让<b>六路舵机同时运动</b>，请先确认臂周围无人、无障碍物。
      </div>
      <div class="row" style="margin-top:8px">
        <button class="sm" id="btnIkDry" onclick="ikSolve(false)" disabled>① 试算（不动臂）</button>
        <button class="sm ac" id="btnIkApply" onclick="ikSolve(true)" disabled>② 确认下发</button>
        <button class="sm" onclick="ikClear()">清除取点</button>
      </div>
      <div id="ikOut" style="font-family:ui-monospace,Menlo,monospace;font-size:11px;line-height:1.7;color:#a9c6e8;margin-top:9px;white-space:pre-wrap">(未取点)</div>
      <details style="margin-top:8px">
        <summary style="cursor:pointer;font-size:12px;color:var(--muted)">画面 → 桌面 标定参数（现场调，改完点「保存参数」生效）</summary>
        <div class="tip" style="margin-top:8px;line-height:1.6">
          大白话：这 5 个数是给「点击画面取点」用的一把尺子 —— 告诉网页整幅画面对应桌面多宽多高、中心在哪、抓取高度多少；
          点一下画面才能把像素换成厘米，再交给逆解算六路脉宽。属平面约束单目定位，<b>不是真 3D</b>。
        </div>
        <div class="g" style="margin-top:8px">
          <div class="cf">
            <label for="ik_span_x"><b>span_x</b> · 画面宽 = 桌面宽 (cm)</label>
            <input type="text" id="ik_span_x" placeholder="如 16" title="画面宽度对应工作台多少 cm；必须 > 0，太大则画面左右两侧超出臂展">
          </div>
          <div class="cf">
            <label for="ik_span_y"><b>span_y</b> · 画面高 = 桌面高 (cm)</label>
            <input type="text" id="ik_span_y" placeholder="如 12" title="画面高度对应工作台多少 cm；必须 > 0，4:3 画面约为 span_x 的 3/4">
          </div>
          <div class="cf">
            <label for="ik_center_x"><b>center_x</b> · 画面中心桌面 x (cm)</label>
            <input type="text" id="ik_center_x" placeholder="如 0" title="画面中心对应的工作台 x 坐标；相机正对底座时填 0，正值向右">
          </div>
          <div class="cf">
            <label for="ik_center_y"><b>center_y</b> · 画面中心桌面 y (cm) · 正前方</label>
            <input type="text" id="ik_center_y" placeholder="如 14" title="画面中心对应的工作台 y 坐标（臂的正前方距离）；太大会让画面上半超出 L1+L2=25.2cm 臂展">
          </div>
          <div class="cf">
            <label for="ik_z"><b>z</b> · 抓取高度 (cm)</label>
            <input type="text" id="ik_z" placeholder="如 2" title="抓取高度；逆解模块就绪后会被 arm_params.json 的 grab_height 覆盖">
          </div>
        </div>
        <div class="row" style="margin-top:8px">
          <button class="sm" onclick="ikSaveCalib()">保存参数</button>
          <span class="tip" id="ikCalibTip"></span>
        </div>
      </details>
    </section>

  </div>

  <div>
    <section class="panel">
      <h2><span>连接</span><span class="tip">串口被串口助手占用时会报「拒绝访问」</span></h2>
      {% if mode == 'wifi' %}
      <div class="row">
        <div style="flex:1;font-size:12px;color:#aab0d8">WiFi TCP：{{wifi_addr}}<br><span style="font-size:11px">手机/电脑需连接 ESP 自建热点</span></div>
        <button class="ac" id="btnConn" onclick="toggleConn()">连接</button>
      </div>
      {% elif mode == 'wifi_server' %}
      <div class="row">
        <div style="flex:1;font-size:12px;color:#aab0d8">TCP server：{{wifi_addr}}<br><span style="font-size:11px">ESP 上电后会自动连入本电脑，点「连接」接管通道</span></div>
        <button class="ac" id="btnConn" onclick="toggleConn()">连接</button>
      </div>
      {% else %}
      <div class="row">
        <input type="text" id="port" placeholder="串口号，如 COM7" value="{{port}}">
        <button class="ac" id="btnConn" onclick="toggleConn()">连接</button>
      </div>
      {% endif %}
      <div class="row" style="margin-top:8px">
        <button onclick="api('PING')">PING</button>
        <button onclick="api('STATE')">读状态</button>
        <button onclick="api('HOME')">回初始位</button>
      </div>
    </section>

    <section class="panel">
      <h2><span>关节控制 · 6 路</span><span class="tip">实际值 1s 回填，滑杆即真实脉宽</span></h2>
      {% for i in range(6) %}
      <div class="jcard" data-i="{{i}}">
        <div class="jtop">
          <span class="jname">{{names[i]}} <span class="jlim">{{mins[i]}} ~ {{maxs[i]}} μs</span></span>
          <span class="jval" id="v{{i}}">{{inits[i]}}</span>
        </div>
        <input type="range" id="s{{i}}" min="{{mins[i]}}" max="{{maxs[i]}}" value="{{inits[i]}}"
               data-init="{{inits[i]}}"
               oninput="drag({{i}}, this)"
               onchange="endDrag({{i}}, this)">
        <div class="jbtns">
          <button onclick="step({{i}},-20)">−20</button>
          <button onclick="step({{i}},20)">+20</button>
          <button onclick="goInit({{i}})">初始位</button>
        </div>
      </div>
      {% endfor %}
    </section>

    <section class="panel">
      <h2><span>夹爪 / 运动 / 示教</span></h2>
      <div class="g">
        <button onclick="apiRaw('MOVE,5,900')">夹爪张开 900</button>
        <button onclick="apiRaw('MOVE,5,1500')">夹爪半开 1500</button>
        <button onclick="apiRaw('MOVE,5,1900')">夹爪闭合 1900</button>
        <button class="ok" onclick="autoInit()">六轴回初始位</button>
        <button onclick="api('STOP')">停止</button>
        <button onclick="api('RESUME')">继续</button>
      </div>
      <div class="row" style="margin-top:8px">
        <button onclick="api('MANUAL')">进入手动</button>
        <button onclick="api('REC')">示教开始</button>
        <button onclick="api('RECEND')">示教结束</button>
        <button onclick="api('PLAY')">回放</button>
      </div>
    </section>
  </div>

</div>

<div class="metrics">
  <div class="metric"><div class="lb">目标 / 实际最大偏差</div><div class="vl" id="mDev">0 μs</div></div>
  <div class="metric"><div class="lb">识别帧率</div><div class="vl" id="mFps">-- fps</div></div>
  <div class="metric"><div class="lb">当前目标</div><div class="vl" id="mTarget" style="color:var(--ok)">—</div></div>
  <div class="metric" style="padding:0;border:none;background:none">
    <button class="estop" onclick="api('STOP')">紧急停止<br>STOP</button>
  </div>
</div>

<details class="panel logbox" open>
  <summary>运行日志</summary>
  <div id="log">(空)</div>
</details>

</div>

<script>
let connected=false;
function tag(name,val){var e=document.getElementById(name);if(e)e.textContent=val;}

function led(c){var e=document.getElementById('led');e.className='dot'+(c?' on':'');}

function setConn(on,msg){
  connected=on;
  led(on);
  tag('sub',msg);
  tag('btnConn',on?'断开':'连接');
  document.getElementById('btnConn').classList.toggle('ac',!on);
}

async function post(url,body){
  try{
    const r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body||{})});
    return await r.json();
  }catch(e){ log('⚠ 网络错误：'+e); return null; }
}

function log(s){
  const box=document.getElementById('log');
  const t='['+new Date().toLocaleTimeString()+'] '+s+"\n";
  box.textContent=t+box.textContent;
  if(box.scrollHeight>box.clientHeight*2)box.textContent=box.textContent.slice(0,4000);
}

async function doCmd(payload){
  log('>> '+payload);
  const r=await post('/api/cmd',{payload:payload});
  if(!r) return;
  if(r.ok){ log('<< '+r.resp); return r.resp; }
  else { log('✗ '+r.error); return null; }
}
function api(payload){ doCmd(payload); }
function apiRaw(payload){ doCmd(payload); }

// 拖动实时跟随：固定间隔节流，连续拖动也每 ~60ms 至少发一次最新值
// dragAt[i] 记「这路滑杆最后一次被拖的时间」，给下面 syncPulses() 判断要不要跳过回填。
const dragTimers={}, dragBusy={}, dragNext={}, dragAt={};
function sendDrag(i,v){ dragNext[i]=null; doCmd('MOVE,'+i+','+v); }
function drag(i,el){
  tag('v'+i, el.value);
  dragNext[i]=el.value;            // 记住最新目标
  dragAt[i]=Date.now();           // 正在拖：1s 状态回填先别覆盖这路
  if(dragBusy[i]) return;          // 上次发送的 60ms 窗口还没到
  dragBusy[i]=true;
  sendDrag(i, dragNext[i]);
  dragTimers[i]=setTimeout(()=>{   // 窗口结束：放行下一次，并补发遗漏的最新值
    dragBusy[i]=false;
    if(dragNext[i]!=null) sendDrag(i, dragNext[i]);
  }, 60);
}

// 松手收尾：清掉 60ms 定时器，同时把节流锁复位，再补发松手那一刻的最终值。
// ⚠️ 这里以前是 bug 高发点：原来 onchange 里只写 clearTimeout(dragTimers[i])，
//    而 dragBusy[i] 只有在定时器回调里才会被置回 false。定时器一被清掉，
//    这把锁就永远卡在 true，之后所有拖动都被 drag() 开头的 return 吞掉，
//    表现为「滑杆有时候拖不动」，且必须刷新页面才恢复（短拖最容易命中）。
//    规范说法：取消定时器时必须补偿性复位它负责的状态标志，避免状态标志泄漏。
const DRAG_HOLD_MS = 800;         // 拖动结束后这段时间内，1s 回填先绕开这路
function endDrag(i,el){
  clearTimeout(dragTimers[i]);
  dragTimers[i]=null;
  dragBusy[i]=false;
  dragNext[i]=null;
  dragAt[i]=Date.now();
  api('MOVE,'+i+','+el.value);
}

// 读实际脉宽回填界面（HOME/MOVE 后滑杆位置与实际一致）
async function syncPulses(){
  if(!connected) return;
  const r=await post('/api/state');
  if(r&&r.ok&&Array.isArray(r.pulses)&&r.pulses.length===6){
    for(let i=0;i<6;i++){
      const s=document.getElementById('s'+i);
      if(!s) continue;
      // 刚拖过这路就先别覆盖：STATE 回的是斜坡过程中的中间值，
      // 强行写回会把滑块拉回去，看起来像「拖不动 / 弹回去」。
      if(dragAt[i] && Date.now()-dragAt[i] < DRAG_HOLD_MS) continue;
      s.value=String(r.pulses[i]);
      tag('v'+i, r.pulses[i]);
    }
  }
}
// 每 1s 轮询实际位置，保证界面始终跟真机一致
setInterval(()=>{ if(connected) syncPulses(); }, 1000);

async function toggleConn(){
  if(connected){ const r=await post('/api/close'); if(r&&r.ok) setConn(false,'未连接'); return; }
  if('{{mode}}'==='wifi'||'{{mode}}'==='wifi_server'){
    const r=await post('/api/open',{});
    if(r&&r.ok){ setConn(true,('{{mode}}'==='wifi_server'?'TCP server 已就绪：':'WiFi 已连接：')+r.port); doCmd('PING'); }
    else log('✗ 连接失败：'+(r?r.error:'未知'));
    return;
  }
  const port=document.getElementById('port').value.trim();
  const r=await post('/api/open',{port:port});
  if(r&&r.ok){ setConn(true,'已连接：'+port); if(r.home)tag('sub','已连接：'+port); doCmd('PING'); }
  else log('✗ 连接失败：'+(r?r.error:'未知'));
}

function step(i,d){
  const s=document.getElementById('s'+i);
  let v=parseInt(s.value)+d;
  v=Math.max(parseInt(s.min),Math.min(parseInt(s.max),v));
  s.value=v; tag('v'+i,v);
  doCmd('MOVE,'+i+','+v);
}
function goInit(i){ api('MOVE,'+i+','+({{inits|tojson}})[i]); }

async function autoInit(){
  {% for i in range(6) %}
  await doCmd('MOVE,{{i}},{{inits[i]}}');
  {% endfor %}
}

// =========================================================================
// 以下为 UI 升级新增部分（视频链路 / 检测叠加 / 遥测）
// 设计原则：上面的串口收发函数一个字都不改，只在这里「包装」和「追加」。
// 好处：即使新功能全挂了，PING/MOVE/STATE 仍然按原来已实机验证的方式工作。
// =========================================================================

// 从滑杆 DOM 读限位和初始位，避免再往模板里塞一份数据（单一数据源）
const INITS = [], MINS = [], MAXS = [];
for (let i = 0; i < 6; i++) {
  const s = document.getElementById('s' + i);
  INITS.push(+s.dataset.init); MINS.push(+s.min); MAXS.push(+s.max);
}

const stat = {
  cmds: 0,                 // 累计下发指令数
  targets: INITS.slice(),  // 最后一次给每路下达的目标脉宽（初值=初始位）
  lastFrameId: 0,          // 上一次识别帧号（用来算 FPS）
  lastFrameT: 0
};
let CFG = { video_url: '', detections_url: '', camera_on: false, mode: 'serial', baud: 115200 };
let overlayOn = true;

// 包装 doCmd：统计指令数、记录目标值、更新模式徽章，再调原函数。
// 为什么不直接改 doCmd 内部？—— 原函数是已验证的控制链路，包一层等于零侵入。
const _doCmd0 = doCmd;
doCmd = function (payload) {
  stat.cmds++;
  const m = /^MOVE,(\d),(\d+)/.exec(payload);
  if (m) stat.targets[+m[1]] = +m[2];
  const modeText = payload === 'STOP' ? '急停' : payload === 'RESUME' ? '已恢复'
                 : payload === 'REC' ? '示教' : payload === 'PLAY' ? '回放'
                 : payload === 'MANUAL' ? '手动' : '指令态';
  tag('pillMode', modeText);
  return _doCmd0(payload);
};

function setupVideo() {
  const img = document.getElementById('camImg');
  const ph = document.getElementById('camPh');
  const badge = document.getElementById('camBadge');
  const inp = document.getElementById('streamUrl');

  // 顶部通道徽章补上波特率（串口模式才有 port 输入框）
  const pe = document.getElementById('port');
  if (pe && CFG.mode === 'serial') tag('pillChan', '串口 ' + pe.value + ' @ ' + CFG.baud);

  if (!CFG.video_url) {
    img.style.display = 'none'; ph.style.display = 'flex';
    badge.textContent = 'OFFLINE'; badge.classList.remove('live');
    tag('pillCam', '视频 未接入');
    return;
  }
  inp.value = CFG.video_url;
  tag('pillCam', CFG.camera_on ? '视频 本地预览' : '视频 已配置');

  // MJPEG 在 <img> 里第一帧到达时触发 load；断流触发 error。
  // 用占位卡而不是破图，是这类监控页的基本体验要求。
  img.onload = function () {
    img.style.display = 'block'; ph.style.display = 'none';
    badge.textContent = 'LIVE'; badge.classList.add('live');
    tag('vsize', img.naturalWidth + ' × ' + img.naturalHeight);
  };
  img.onerror = function () {
    img.style.display = 'none'; ph.style.display = 'flex';
    badge.textContent = 'OFFLINE'; badge.classList.remove('live');
    tag('vsize', '--');
  };
  img.src = CFG.video_url;
}

// 流地址可手工改：换网络、换板子 IP 时不用重启服务
document.getElementById('streamUrl').addEventListener('keydown', function (e) {
  if (e.key !== 'Enter') return;
  const v = this.value.trim();
  if (!v) return;
  CFG.video_url = v;
  document.getElementById('camImg').src = v;
  log('视频流地址已改为 ' + v);
});

function shot() {
  const img = document.getElementById('camImg');
  if (!CFG.video_url || img.style.display === 'none') { log('⚠ 没有视频流，无法截图'); return; }
  try {
    const c = document.createElement('canvas');
    c.width = img.naturalWidth || 640; c.height = img.naturalHeight || 480;
    c.getContext('2d').drawImage(img, 0, 0, c.width, c.height);
    const a = document.createElement('a');
    a.download = 'arm_' + Date.now() + '.jpg';
    a.href = c.toDataURL('image/jpeg');
    a.click();
    log('已保存截图');
  } catch (err) {
    // 跨域图片会「污染」canvas，toDataURL 抛 SecurityError。
    // 退化方案：新窗口打开原图，用户自己右键保存。
    window.open(CFG.video_url, '_blank');
    log('⚠ 跨域流无法本地截图，已在新窗口打开');
  }
}

function fullscreen() {
  const el = document.getElementById('vwrap');
  if (el.requestFullscreen) el.requestFullscreen();
  else log('⚠ 当前浏览器不支持全屏');
}

function toggleOverlay() {
  overlayOn = !overlayOn;
  document.getElementById('overlay').style.display = overlayOn ? 'block' : 'none';
  tag('btnOverlay', overlayOn ? '叠加：开' : '叠加：关');
}

// 检测框用绝对定位的 div 画，按帧宽高换算成百分比。
// 为什么不把框烧进 JPEG？—— 叠加层可开关、可点击、不损失画质，也省一次编码。
function showDetections(d) {
  // 优先用识别服务上报的真实帧尺寸；没有就退回 <img> 实际解码出来的尺寸，
  // 最后才用 640x480 兜底。三者不一致时检测框会整体错位，顺序不能反。
  const imgEl = document.getElementById('camImg');
  const w = (d.frame && d.frame.w) || imgEl.naturalWidth || 640;
  const h = (d.frame && d.frame.h) || imgEl.naturalHeight || 480;
  const dets = d.detections || [];
  let html = '';
  for (const x of dets) {
    const b = x.bbox || [0, 0, 0, 0];
    const L = (b[0] / w * 100).toFixed(2), T = (b[1] / h * 100).toFixed(2);
    const W = ((b[2] - b[0]) / w * 100).toFixed(2), H = ((b[3] - b[1]) / h * 100).toFixed(2);
    html += '<div class="dbox" style="left:' + L + '%;top:' + T + '%;width:' + W + '%;height:' + H + '%">'
          + '<b>' + x.class + ' ' + x.conf + '</b></div>';
  }
  document.getElementById('overlay').innerHTML = html;

  const t = d.target;
  let chips = t ? '<span class="chip hot">目标 ' + t.class + ' · ' + t.conf + ' · ' + t.direction + '</span>' : '';
  chips += '<span class="chip">检测 ' + dets.length + ' 个</span><span class="chip">帧号 ' + d.frame_id + '</span>';
  document.getElementById('detChips').innerHTML = chips;
  tag('mTarget', t ? t.class : '—');

  // 用 frame_id 的增量 / 真实时间算 FPS，比服务里自报更可信
  const now = performance.now();
  if (!stat.lastFrameT) { stat.lastFrameT = now; stat.lastFrameId = d.frame_id; }
  else {
    const dt = (now - stat.lastFrameT) / 1000;
    if (dt >= 0.8) {
      tag('mFps', ((d.frame_id - stat.lastFrameId) / dt).toFixed(1) + ' fps');
      stat.lastFrameT = now; stat.lastFrameId = d.frame_id;
    }
  }
}

function startDetections() {
  if (!CFG.detections_url) {
    document.getElementById('detChips').innerHTML =
      '<span class="chip">检测服务未配置（PC 阶段为裸画面）</span>';
    return;
  }
  // 走本服务的 /api/detections 代理，浏览器永远同源，不用管 CORS
  setInterval(async function () {
    try {
      const r = await fetch('/api/detections');
      const j = await r.json();
      if (j.ok) showDetections(j.data);
      else document.getElementById('detChips').innerHTML = '<span class="chip hot">识别服务未响应</span>';
    } catch (e) { /* 网络抖动忽略，下一拍再试 */ }
  }, 400);
}

// 偏差 = 最后一次 MOVE 目标 vs 1s 回填的真实脉宽，取 6 路最大值。
// 只读 DOM，不额外发 STATE —— 绝不增加串口流量。
setInterval(function () {
  let mx = 0;
  for (let i = 0; i < 6; i++) {
    const s = document.getElementById('s' + i);
    if (s) mx = Math.max(mx, Math.abs(stat.targets[i] - +s.value));
  }
  tag('mDev', mx + ' μs');
}, 300);

setInterval(function () { tag('clock', new Date().toLocaleTimeString('zh-CN', { hour12: false })); }, 1000);

// 启动时拉配置（新增的 /api/config，纯读，不碰串口）
fetch('/api/config').then(function (r) { return r.json(); }).then(function (c) {
  CFG = c; setupVideo(); startDetections();
}).catch(function () {
  setupVideo(); startDetections();
});

// =========================================================================
// A 方案：点击画面取点 -> 试算 -> 确认下发（和上面的控制链路零耦合）
//
// ① 用了什么语法：addEventListener('click') + getBoundingClientRect() 坐标换算 +
//    async/await + 模块级状态变量（ikPoint / ikLast / ikReady / ikPickOn）。
// ② 为什么这么设计：取点默认关闭（checkbox），试算与下发拆成两个按钮并二次 confirm，
//    因为六路舵机会同时运动；且 <img> 被 object-fit:fill 拉伸过，必须用
//    「显示矩形 → 原始分辨率」的比例换算，不能拿 e.offsetX 直接当像素坐标。
// ③ 不这么写会怎样：误点一下画面臂就猛甩；或画面被拉伸后取点整体偏移，
//    逆解算出的 (x,y) 全错，还误以为 IK 不准。
// =========================================================================

const IK_NAMES = {{names|tojson}};     // 和后端的 JOINT_NAMES 用同一份数据
let ikReady = false, ikPickOn = false, ikPoint = null, ikLast = null;

function tagIk(s) { document.getElementById('ikOut').textContent = s; }

function onCamClick(e) {
  if (!ikPickOn) return;                       // 默认关，必须手动勾选才生效
  const img = document.getElementById('camImg');
  if (!CFG.video_url || img.style.display === 'none' || !img.naturalWidth) {
    log('⚠ 画面未 LIVE，不能取点'); return;
  }
  const rect = img.getBoundingClientRect();
  const dx = e.clientX - rect.left, dy = e.clientY - rect.top;
  if (dx < 0 || dy < 0 || dx > rect.width || dy > rect.height) return;
  // 显示尺寸 -> 原始分辨率：object-fit:fill 把 640×480 拉伸成容器大小，
  // 所以 u = 显示横向比例 × naturalWidth，v 同理。
  const u = dx / rect.width * img.naturalWidth;
  const v = dy / rect.height * img.naturalHeight;
  ikPoint = {
    u: Math.round(u), v: Math.round(v),
    w: img.naturalWidth, h: img.naturalHeight,
    px: dx / rect.width * 100, py: dy / rect.height * 100
  };
  const mk = document.getElementById('pickMark');
  mk.style.display = 'block';
  mk.style.left = ikPoint.px + '%'; mk.style.top = ikPoint.py + '%';
  tagIk('取点  像素 (' + ikPoint.u + ', ' + ikPoint.v + ')  /  图像 '
      + ikPoint.w + '×' + ikPoint.h + '\n点「① 试算」看逆解结果（此时绝不动臂）');
  document.getElementById('btnIkDry').disabled = false;
  document.getElementById('btnIkApply').disabled = true;   // 换了点必须重新试算
  ikLast = null;
}

async function ikSolve(apply) {
  if (!ikPoint) { log('⚠ 先点画面取一个点'); return; }
  if (!ikReady) { log('⚠ 逆解模块未就绪，不能解算'); return; }
  if (apply) {
    if (!connected) { log('⚠ 串口未连接，先点「连接」'); return; }
    if (!ikLast || !ikLast.ok) { log('⚠ 请先「① 试算」成功再下发'); return; }
    if (!confirm('确认下发 JOINT 到机械臂？\n六路目标脉宽：' + ikLast.pulses.join(', ')
        + '\n\n六路舵机会同时运动，请确认臂周围无人、无障碍物。')) return;
  }
  const body = { u: ikPoint.u, v: ikPoint.v, w: ikPoint.w, h: ikPoint.h, apply: !!apply };
  log((apply ? '>> 下发 ' : '>> 试算 ') + '像素 (' + ikPoint.u + ', ' + ikPoint.v + ')');
  const r = await post('/api/ik/solve', body);
  if (!r) return;
  if (!r.ok) { log('✗ ' + r.error); tagIk('✗ ' + r.error); return; }
  ikLast = r;
  let s = '像素 (' + ikPoint.u + ',' + ikPoint.v + ')  →  桌面 (x=' + r.x
        + ', y=' + r.y + ', z=' + r.z + ') cm\n'
        + '目标脉宽(μs)：' + r.pulses.map(function (p, i) {
            return IK_NAMES[i] + '=' + p;
          }).join('  ') + '\n';
  s += (r.clamped && r.clamped.length)
     ? '⚠ 超出软限位被裁剪：' + r.clamped.join(',') + '（IK 解不完全可达）\n'
     : '✓ 六路均在软限位内\n';
  if (r.applied) {
    s += '\n已下发：JOINT,' + r.pulses.join(',') + '\n固件应答：' + (r.resp || '(空)') + '\n';
    if (r.state_pulses && r.state_pulses.length === 6)
      s += 'STATE 瞬时值：' + r.state_pulses.join(',') + '\n  → ' + r.state_note + '\n';
    s += r.note;
  } else {
    s += '\n【仅试算，未下发】确认无误后再点「② 确认下发」。';
  }
  tagIk(s);
  document.getElementById('btnIkApply').disabled = r.applied;   // 下发过就禁用，防连点
  log(r.applied ? ('<< ' + r.resp + ' | 目标 ' + r.pulses.join(','))
                : ('<< 试算完成 目标 ' + r.pulses.join(',')));
}

function ikClear() {
  ikPoint = null; ikLast = null;
  document.getElementById('pickMark').style.display = 'none';
  tagIk('(未取点)');
  document.getElementById('btnIkDry').disabled = true;
  document.getElementById('btnIkApply').disabled = true;
}

async function ikSaveCalib() {
  const keys = ['span_x', 'span_y', 'center_x', 'center_y', 'z'];
  const body = {};
  for (const k of keys) {
    const val = parseFloat(document.getElementById('ik_' + k).value);
    if (!isFinite(val)) { tag('ikCalibTip', '✗ ' + k + ' 不是数字'); return; }
    body[k] = val;
  }
  const r = await post('/api/ik/calib', body);
  if (r && r.ok) {
    tag('ikCalibTip', '✓ 已保存 ' + r.changed.length + ' 项，重新试算即生效');
    log('画面→桌面标定参数已更新：' + JSON.stringify(body));
  } else {
    tag('ikCalibTip', '✗ ' + (r ? r.error : '请求失败'));
  }
}

// 取点开关：默认关闭，防止看画面时误触发运动
document.getElementById('ikPickOn').addEventListener('change', function (e) {
  ikPickOn = e.target.checked;
  document.getElementById('vwrap').classList.toggle('pickable', ikPickOn);
  log(ikPickOn ? '已开启「点击画面取点」' : '已关闭「点击画面取点」');
});
document.getElementById('camImg').addEventListener('click', onCamClick);

// 启动拉 IK 状态（纯读）；未就绪时把原因直接写进面板，不让人瞎点
fetch('/api/ik/config').then(function (r) { return r.json(); }).then(function (c) {
  ikReady = !!c.ready;
  const badge = document.getElementById('ikState');
  badge.textContent = c.ready ? '逆解已就绪' : '逆解未就绪';
  badge.className = 'pill' + (c.ready ? '' : ' warn');
  if (!c.ready) {
    tagIk('逆解未就绪：' + c.error
        + '\n\n需要在板上 ~/kunpeng_vision/ 放 arm_ik.py + arm_params.json，'
        + '再重启 5000 服务（Ctrl+C 后重新 python3 web_control.py ...）。');
  }
  const m = c.map || {};
  ['span_x', 'span_y', 'center_x', 'center_y', 'z'].forEach(function (k) {
    const el = document.getElementById('ik_' + k);
    if (el && m[k] !== undefined && m[k] !== null) el.value = m[k];
  });
}).catch(function () {
  document.getElementById('ikState').textContent = '逆解状态读取失败';
});
</script>
</body>
</html>
"""


@app.route("/")
def index():
    return render_template_string(
        PAGE,
        port=_HOST_STATE["port"],
        mode=_MODE,
        wifi_addr=_WIFI_ADDR,
        names=JOINT_NAMES,
        mins=[r[0] for r in PULSE_RANGES],
        maxs=[r[1] for r in PULSE_RANGES],
        inits=[r[2] for r in PULSE_RANGES],
    )


@app.route("/api/open", methods=["POST"])
def api_open():
    global _HOST
    if _MODE == "wifi_server":
        # 电脑已作为 TCP server 监听（main 已创建 _HOST），复用而非重建，
        # 避免 close() 后又因 _HOST 仍非 None 造成"连接后再也不复建"。ESP 连入后即就绪。
        try:
            if _HOST is None:
                _HOST = TcpServerHost(_WIFI_ADDR)
            _HOST_STATE.update(connected=True, port="TCP server :%s (等 ESP 连入)" % _WIFI_ADDR, error="")
            return jsonify(ok=True, port=_HOST_STATE["port"])
        except Exception as e:
            _HOST_STATE.update(connected=False, error=str(e))
            return jsonify(ok=False, error=str(e))
    # 串口 / WiFi client：关闭旧连接并新建
    if _HOST is not None:
        try:
            _HOST.close()
        except Exception:
            pass
        _HOST = None
    if _MODE == "wifi":
        host, _, port_s = _WIFI_ADDR.partition(":")
        try:
            _HOST = TcpHost(host, int(port_s or 9000))
            _HOST_STATE.update(connected=True, port=_WIFI_ADDR, error="")
            return jsonify(ok=True, port=_WIFI_ADDR)
        except Exception as e:
            _HOST_STATE.update(connected=False, error=str(e))
            return jsonify(ok=False, error=str(e))
    data = request.get_json(force=True, silent=True) or {}
    port = data.get("port") or _HOST_STATE["port"]
    try:
        _HOST = SerialHost(port)
        _HOST_STATE.update(connected=True, port=port, error="")
        return jsonify(ok=True, port=port)
    except Exception as e:
        _HOST_STATE.update(connected=False, error=str(e))
        return jsonify(ok=False, error=str(e))


@app.route("/api/close", methods=["POST"])
def api_close():
    global _HOST
    if _HOST is not None:
        try:
            _HOST.close()
        except Exception:
            pass
    _HOST = None
    _HOST_STATE.update(connected=False, error="")
    return jsonify(ok=True)


@app.route("/api/cmd", methods=["POST"])
def api_cmd():
    global _HOST
    data = request.get_json(force=True, silent=True) or {}
    payload = str(data.get("payload", "")).strip()
    if not payload:
        return jsonify(ok=False, error="空命令")
    if _HOST is None:
        return jsonify(ok=False, error="未连接")
    try:
        # MOVE 下位机不回应答，走 fire 只写不等待，保证拖动实时跟手
        if payload.upper().startswith("MOVE"):
            _HOST.fire(payload)
            return jsonify(ok=True, resp="")
        resp = _HOST.cmd(payload)
        return jsonify(ok=True, resp=resp)
    except Exception as e:
        return jsonify(ok=False, error=str(e))


@app.route("/api/state", methods=["POST"])
def api_state():
    global _HOST
    if _HOST is None:
        return jsonify(ok=False, error="未连接")
    try:
        resp = _HOST.cmd("STATE")
    except Exception as e:
        return jsonify(ok=False, error=str(e))
    parts = resp.split(",")
    pulses = []
    try:
        for tok in parts[3:9]:
            pulses.append(int(tok))
    except (ValueError, IndexError):
        return jsonify(ok=False, error="STATE 响应异常: %r" % resp)
    return jsonify(ok=True, pulses=pulses)


@app.route("/api/config")
def api_config():
    """页面启动配置：视频地址、识别地址、通道信息。纯读，不碰串口。"""
    return jsonify(
        ok=True,
        video_url=_VIDEO_URL,
        detections_url=_DETECTIONS_URL,
        camera_on=_CAM is not None,
        mode=_MODE,
        baud=_HOST_STATE.get("baud", 115200),
    )


# ---------------------------------------------------------------------------
# A 方案：点选抓取的三个接口（逆解实现见文件顶部 _IK / pixel_to_table / solve_to_pulses）
#
# ① 用了什么语法：Flask 路由 + request.get_json + 字典组装 jsonify + 分层 try/except。
# ② 为什么这么设计：拆成「看状态 / 改标定 / 解算」三个独立接口，且解算默认
#    apply=False 只试算 —— 六路舵机会同时运动，必须让人先看到脉宽再确认下发。
# ③ 不这么写会怎样：点一下画面就让臂猛甩；逆解抛 ValueError（目标超出可达范围）
#    还会变成 500 错误页，页面拿不到任何有用提示。
# ---------------------------------------------------------------------------
@app.route("/api/ik/config")
def api_ik_config():
    """给页面看：逆解是否就绪、失败原因、当前「画面→桌面」映射参数。纯读，不碰串口。"""
    return jsonify(
        ok=True,
        ready=_IK is not None,
        error=_IK_ERROR,
        map=dict(_IK_MAP),
        ranges=PULSE_RANGES,
    )


@app.route("/api/ik/calib", methods=["POST"])
def api_ik_calib():
    """现场标定「画面→桌面」线性映射。

    大白话：镜头装歪了、画面里桌子看起来更大更小时，改这几个数就能对齐，不用重烧代码。
    规范定义：只更新白名单里的标量键并做数值类型校验，避免把 _IK_MAP 写成字符串，
    导致后面 pixel_to_table 里 float() 抛 TypeError（配置污染比逻辑 bug 更难查）。
    """
    data = request.get_json(force=True, silent=True) or {}
    changed = []
    for key in ("span_x", "span_y", "center_x", "center_y", "z", "tilt", "claw", "wrist"):
        if key not in data:
            continue
        value = data[key]
        # bool 是 int 的子类，必须单独挡掉，否则 True 会被悄悄写成 1.0
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return jsonify(ok=False, error="%s 必须是数字" % key)
        if key in ("span_x", "span_y") and float(value) <= 0:
            return jsonify(ok=False, error="%s 必须大于 0" % key)
        _IK_MAP[key] = float(value)
        changed.append(key)
    return jsonify(ok=True, changed=changed, map=dict(_IK_MAP))


@app.route("/api/ik/solve", methods=["POST"])
def api_ik_solve():
    """像素 (u,v,w,h) -> 桌面 (x,y) -> IK -> 6 路脉宽。

    apply=False（默认）：只试算，一个字节都不往串口发。
    apply=True：构造 JOINT,<p0>..<p5> 下发，并回读一次 STATE 的瞬时值。

    诚实边界：STATE 返回的是斜坡推进中的「当前脉宽」，不是「已到位确认」；
    到位与否靠页面已有的 1s 轮询 /api/state 持续观察，这里绝不假装确认。
    """
    data = request.get_json(force=True, silent=True) or {}
    try:
        x, y = pixel_to_table(
            float(data.get("u", 0)), float(data.get("v", 0)),
            float(data.get("w", 0)), float(data.get("h", 0)))
        pulses, clamped = solve_to_pulses(x, y)
    except (ValueError, RuntimeError) as e:
        # 目标过近 / 超出可达范围 / 逆解模块没就绪，都归成「能看懂的一句话」
        return jsonify(ok=False, error=str(e))
    except Exception as e:
        return jsonify(ok=False, error="逆解异常：%s" % e)

    out = dict(ok=True, x=round(x, 2), y=round(y, 2), z=_IK_MAP["z"],
               pulses=pulses, clamped=clamped, applied=False, resp="")

    if not data.get("apply"):
        out["note"] = "仅试算，未下发任何串口命令"
        return jsonify(out)

    if _HOST is None:
        out["ok"] = False
        out["error"] = "串口未连接：试算结果已保留，先点页面「连接」再下发"
        return jsonify(out)

    payload = "JOINT,%s" % ",".join(str(int(p)) for p in pulses)
    try:
        out["resp"] = _HOST.cmd(payload)
    except Exception as e:
        out["ok"] = False
        out["error"] = "JOINT 下发失败：%s" % e
        return jsonify(out)

    out["applied"] = True
    out["frame"] = payload
    if not out["resp"].startswith("OK"):
        out["note"] = "固件未回 OK，请检查链路（帧已发出）"
    else:
        out["note"] = "固件已回 OK；六路正走斜坡逼近目标，页面 1s 轮询会持续刷新实际值"

    # 回读一次 STATE：只展示「当下瞬时值」，绝不冒充到位确认。
    out["state_pulses"] = []
    out["state_note"] = ""
    try:
        sresp = _HOST.cmd("STATE")
        parts = sresp.split(",")
        if len(parts) >= 9:
            out["state_pulses"] = [int(t) for t in parts[3:9]]
            out["state_note"] = "斜坡中的瞬时值，非最终到位；最终值看页面 1s 轮询"
    except Exception:
        out["state_note"] = "STATE 回读失败（不影响已下发的目标）"
    return jsonify(out)


@app.route("/api/detections")
def api_detections():
    """把 detector_service 的 /detections 代理成同源接口。

    为什么要代理：浏览器直接 fetch 别的端口是跨域；更麻烦的是，手机访问时
    「127.0.0.1」指的是手机自己，会连错机器。走同源代理后由服务端去连，
    地址永远正确，也不依赖对方是否带 CORS 头。
    """
    if not _DETECTIONS_URL:
        return jsonify(ok=False, error="未配置识别服务")
    try:
        with urllib.request.urlopen(_DETECTIONS_URL, timeout=0.5) as resp:
            data = json.loads(resp.read().decode("utf-8"))
        return jsonify(ok=True, data=data)
    except Exception as e:
        return jsonify(ok=False, error=str(e))


@app.route("/video")
def video_feed():
    """本地摄像头 MJPEG 流（仅 --camera 时启用）。

    大白话：MJPEG 就是把一张张 JPEG 照片用固定分隔符串成一条不会结束的响应，
    浏览器 <img> 收到一张显示一张，看起来就是视频。
    规范定义：multipart/x-mixed-replace; boundary=frame，每个 part 带
    Content-Type: image/jpeg 与二进制负载，通过 HTTP 长连接持续推送。
    """
    if _CAM is None:
        return Response("未启用本地摄像头（启动时加 --camera 0）", status=503,
                        mimetype="text/plain; charset=utf-8")

    def gen():
        while True:
            jpg = _CAM.get_jpeg()
            if jpg:
                yield (b"--frame\r\n"
                       b"Content-Type: image/jpeg\r\n"
                       b"Content-Length: " + str(len(jpg)).encode("ascii") + b"\r\n"
                       b"\r\n" + jpg + b"\r\n")
            time.sleep(0.066)   # 约 15 fps 上限，避免空转烧 CPU

    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("serial", nargs="?", default=None, help="STM32 串口号，如 COM7")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--wifi", help="旧式 WiFi client 通道，如 192.168.4.1:9000（ESP 自建热点）")
    ap.add_argument("--tcp-server", type=int, nargs="?", const=9000,
                    metavar="PORT", help="电脑做 TCP server 等 ESP 连入（建议 9000）")
    ap.add_argument("--host", default="0.0.0.0", help="监听地址，默认 0.0.0.0")
    ap.add_argument("--port", type=int, default=5000, help="HTTP 端口，默认 5000")
    ap.add_argument("--camera", default=None, metavar="ID",
                    help="PC 阶段用本地摄像头出 MJPEG 画面，如 --camera 0")
    ap.add_argument("--video-url", default="",
                    help="鲲鹏阶段用外部 MJPEG 地址，如 http://板IP:8080/video")
    ap.add_argument("--detections-url", default="",
                    help="识别结果 JSON 地址；留空且给了 --video-url 时自动推导 /detections")
    ap.add_argument("--ik-dir", default="~/kunpeng_vision", metavar="DIR",
                    help="A 方案逆解模块目录（需含 arm_ik.py + arm_params.json）；"
                         "默认 ~/kunpeng_vision，找不到只关掉「点选抓取」、不影响主链路")
    args = ap.parse_args()

    global _MODE, _WIFI_ADDR, _CAM, _VIDEO_URL, _DETECTIONS_URL, _HOST

    if args.tcp_server is not None:
        _MODE = "wifi_server"
        _WIFI_ADDR = args.tcp_server
        try:
            _HOST = TcpServerHost(_WIFI_ADDR)
            _HOST_STATE.update(connected=True, port="TCP server :%d" % _WIFI_ADDR, error="")
        except Exception as e:
            _HOST_STATE.update(connected=False, error=str(e))
            print("  [警告] 启动 TCP server 失败：%s（可稍后在页面点「连接」重试）" % e)
        chan_desc = "TCP server :%d (等 ESP-01S 连入)" % _WIFI_ADDR
    elif args.wifi:
        _MODE = "wifi"
        host, _, port_s = args.wifi.partition(":")
        wifi_port = int(port_s or 9000)
        _WIFI_ADDR = "%s:%d" % (host, wifi_port)
        try:
            _HOST = TcpHost(host, wifi_port)
            _HOST_STATE.update(connected=True, port=_WIFI_ADDR, error="")
        except Exception as e:
            _HOST_STATE.update(connected=False, error=str(e))
            print("  [警告] 启动时连接 WiFi TCP 失败：%s（可稍后在页面点「连接」重试）" % e)
        chan_desc = "WiFi TCP %s" % _WIFI_ADDR
    else:
        _MODE = "serial"
        port = args.serial or "COM7"
        _HOST_STATE["port"] = port
        _HOST_STATE["baud"] = args.baud
        chan_desc = "串口 %s @ %d baud" % (port, args.baud)

    # ---- 视频链路（和串口控制链路完全独立，怎么改都不影响 PING/MOVE/STATE）----
    # 优先级：外部 --video-url > 本地 --camera；都不给 = 页面显示占位卡。
    # 只开一个摄像头：给了外部流就绝不再碰本地摄像头（摄像头是独占设备，
    # 开两次会失败；鲲鹏阶段更不能和 detector_service 抢同一个 /dev/video0）。
    if args.video_url:
        _VIDEO_URL = args.video_url.strip()
    elif args.camera is not None:
        try:
            _CAM = CameraStream(args.camera)
            _VIDEO_URL = "/video"
        except Exception as e:
            # 可选功能失败不能拖垮整个控制页：只告警，网页照常能连串口
            print("  [警告] 本地摄像头启动失败：%s（页面仍可正常控制）" % e)

    # 识别结果：显式给就用；没给但从 .../video 推出来的地址，把末尾换成 /detections。
    # 走本服务的 /api/detections 同源代理，浏览器不用管跨域，
    # 手机端也不会把 127.0.0.1 错指成手机自己。
    # 注意只推导 http(s) 开头的绝对地址：本地 "/video" 是相对路径，不能拿去 urlopen。
    if args.detections_url:
        _DETECTIONS_URL = args.detections_url.strip()
    elif _VIDEO_URL.startswith("http") and _VIDEO_URL.endswith("/video"):
        _DETECTIONS_URL = _VIDEO_URL[:-len("/video")] + "/detections"

    # ---- A 方案逆解（可选功能，失败只告警，绝不拖垮网页服务）----
    # ① 用了什么语法：函数调用 + 模块级全局软引用（_IK 允许是 None）。
    # ② 为什么这么设计：arm_ik.py / arm_params.json 不在板上最小上传集里，
    #    硬 import 会让缺文件时整个 5000 起不来；这里「找不到就只关掉点选抓取」。
    # ③ 不这么写会怎样：一个可选的抓取功能拖垮已跑通的 PING/MOVE/STATE/视频链路。
    ik_ok = init_ik(args.ik_dir)
    if ik_ok:
        ik_desc = "已就绪（%s）" % os.path.expanduser(args.ik_dir or "~/kunpeng_vision")
    else:
        ik_desc = "未启用：%s" % _IK_ERROR

    print("=" * 56)
    print("机械臂网页遥控已启动")
    print("  通道：%s" % chan_desc)
    print("  网页：http://<本机IP>:%d" % args.port)
    if _VIDEO_URL:
        print("  视频：%s%s" % (_VIDEO_URL, "（本地摄像头）" if _CAM else "（外部流）"))
    else:
        print("  视频：未接入（PC 加 --camera 0 ｜ 鲲鹏加 --video-url http://板IP:8080/video）")
    if _DETECTIONS_URL:
        print("  识别：%s" % _DETECTIONS_URL)
    print("  逆解：%s" % ik_desc)
    if _MODE == "wifi_server":
        print("  提示：电脑先连 ESP 自建热点 并固定 IP 192.168.4.50；"
              "手机/电脑浏览器打开 http://<本机IP>:%d 即可控制" % args.port)
    elif _MODE == "wifi":
        print("  提示：手机/电脑连接 ESP 自建热点 后，在手机浏览器打开： http://<本机IP>:%d" % args.port)
    else:
        print("  提示：手机与电脑连同一 WiFi 后，在手机浏览器打开： http://<本机IP>:%d" % args.port)
    print("=" * 56)
    # 访问日志的过滤已在模块顶部挂到 werkzeug logger 上，这里直接启动即可。
    # 不再用 setLevel(ERROR)：那样会把 "Running on http://...:5000" 一起屏蔽。
    app.run(host=args.host, port=args.port, debug=False)


if __name__ == "__main__":
    main()