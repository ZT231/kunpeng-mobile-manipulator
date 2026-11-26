# -*- coding: utf-8 -*-
"""
手机网页遥控（S5，W 模式 Web 控制）

手机浏览器 --WiFi--> 本服务(Flask) --通道--> STM32F407
通道二选一：
  * 串口：USB-TTL/USART1（PC 串口）
  * WiFi：ESP-01S TCP server（手机/电脑连 ESP 自建热点后直连 192.168.4.1:9000）

帧格式与固件一致：$<CMD>[,<ARG>...]*<CK><CR><LF>，CK 为 $ 与 * 之间所有字节的 XOR（2 位大写十六进制）
机械臂命令已迁 $ARM 协议（$ARM,POS/$ARM,READ/$ARM,J/$ARM,HOME/$ARM,STOP/$ARM,RESUME）。

依赖：
  pip install flask pyserial

用法：
  python web_control.py COM7 --baud 115200          # 经 PC 串口控制
  python web_control.py COM7 --camera 0             # PC 阶段：顺便开本地摄像头出 MJPEG 画面
  python web_control.py --video-url http://127.0.0.1:8080/video
                                                    # 鲲鹏阶段：画面来自 detector_service.py
                                                    # 不写串口号 = 自动探测（by-id 优先，
                                                    # ttyUSB0/1 互换都不影响），启动即连接
  python web_control.py /dev/ttyUSB0 --video-url http://127.0.0.1:8080/video
                                                    # 也可以显式指定端口（编号变了就得改）
  python web_control.py --wifi 192.168.4.1:9000     # 经 WiFi TCP 控制（ESP 自建热点）
  python web_control.py --host 0.0.0.0 --port 5000  # 让手机通过同一 WiFi 访问

视频是「另一条链路」：摄像头 --> MJPEG --> 浏览器 <img>，和 STM32 串口的
PING/$ARM 完全独立。所以页面怎么改都不会影响已经跑通的控制链路。

注意（2026-11-04 适配鲲鹏项目）：本项目 STM32 执行端经串口直连鲲鹏（默认 serial 模式即可，
端口传 /dev/ttyXXX，如 /dev/ttyUSB0 或板载 UART）；旧项目的 ESP-01S WiFi 模式在本项目由
ESP32-S3 替代（语音+联网），此处 wifi / tcp-server 模式仅作旧项目兼容保留，新项目请走 serial。
2026-11-12 起：串口号可整段省略 —— 自动探测 /dev/serial/by-id/usb-1a86_*（CH341 稳定别名），
所以 CH341 被 USB 复位、编号在 ttyUSB0/ttyUSB1 之间来回翻都不影响，启动时也会自动连接。
位置范围已对齐固件 main.h 的软限位（J6 上限 2000，为另行收窄的安全限位；
物理实测闭合端点 2100 仍记在 arm_params.json / docs/舵机标定记录.md 第六节，两者不是一回事）。
"""

import argparse
import json
import logging
import math
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

# 页面滑杆位置范围：(MIN, MAX, 初始位=零位)。单位：总线舵机位置 0~1000（0~240°，1 单位=0.24°）。
# 零位与 servo_bus.h 的 SB_HOME_POS_LIST 逐路一致；J6 软限位 300~674 为 2026-11-26 实测。
# J1~J5 暂用驱动安全限位 10~990 占位（servo_bus.h SB_POS_SAFE_MIN/MAX），装车实测后收紧。
POS_RANGES = [
    (10, 990, 460),   # J1 底座回转（软限位待实测）
    (10, 990, 514),   # J2 大臂（软限位待实测）
    (10, 990, 516),   # J3 小臂（软限位待实测）
    (10, 990, 502),   # J4 腕俯仰（软限位待实测）
    (10, 990, 494),   # J5 腕回转（软限位待实测）
    (300, 674, 674),  # J6 夹爪（实测：全张=300，全闭=674；零位=完全闭合）
]

JOINT_LIMITS = {
    JOINT_NAMES[i]: POS_RANGES[i] for i in range(CHANNEL_COUNT)
}


# ---------------------------------------------------------------------------
# A 方案：点击画面取点 -> 逆解 -> 六路位置 -> $ARM,POS 下发（可选功能）
#
# ① 用了什么语法：模块级可选 import + 字典配置 + 纯函数（不碰任何已有全局状态）。
# ② 为什么这么设计：逆解依赖 arm_ik.py / arm_params.json 两个文件，鲲鹏板最小上传集
#    本来不含它们；做成「找不到就 _IK=None、接口返回明确原因」的软降级，不传文件
#    也不会影响已跑通的 PING/$ARM/视频/检测框链路，删掉文件即可回滚。
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
    "z": 2.0,              # 抓取高度 cm（默认值，深度相机/前端可覆盖）
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


def solve_to_positions(x, y):
    """工作台 (x,y) -> 6 路位置，并裁到固件软限位 POS_RANGES。

    返回 (positions, clamped)：clamped 列出被裁的关节，供页面提示「目标超出可达范围」。
    为什么 Python 侧也裁一遍：固件 arm_robot 也会做软限位钳位，但那是静默裁剪；
    这边显式裁并回报，人才能知道 IK 解其实越界了，而不是以为「算得很准」。
    """
    if _IK is None:
        raise RuntimeError(_IK_ERROR or "逆解模块未就绪")
    angles = _IK.solve(x, y, z=_IK_MAP["z"], wrist=_IK_MAP["wrist"],
                       claw=_IK_MAP["claw"], tilt=_IK_MAP["tilt"])
    raw = _IK.to_positions(angles)
    positions, clamped = [], []
    for i, p in enumerate(raw):
        lo, hi, _ = POS_RANGES[i]
        q = max(lo, min(hi, int(p)))
        if q != int(p):
            clamped.append(JOINT_NAMES[i])
        positions.append(q)
    return positions, clamped


# ---------------------------------------------------------------------------
# A+ 方案：眼在手定位（手眼矩阵 X 的真正消费者，可选功能）
#
# ① 用了什么语法：模块级可选 import（try/except ImportError）+ 全局配置字典
#    + 惰性加载（第一次用到才读 JSON，避免启动时因文件缺失拖慢/拖垮服务）。
# ② 为什么这么设计：A 方案的 pixel_to_table() 是「5 个手工常量 + 假设拍照姿态固定」
#    的退化解，换任何姿态立刻失真。眼在手链路用「内参 K + 手眼矩阵 X + 当前关节角」
#    算出真正的射线-平面交点，这才对得起「相机装在臂末端」的硬件事实。
#    但它是新算路，必须能一键关掉回退到旧映射，否则一旦 X 是坏的，
#    连原本能用的点选功能也一起废掉 —— 所以做成 --eye-in-hand 显式开启。
# ③ 不这么写会怎样：默认开启 + X 是占位数据 = 页面给出看似精确、实则错的坐标，
#    比明确报错更危险；而硬 import 会让缺 numpy 的板子连 5000 都起不来。
# ---------------------------------------------------------------------------
_EYE = None                # EyeInHandLocalizer 实例；None = 未启用
_EYE_ERROR = ""            # 未启用原因，给页面看
_EYE_INFO = {              # 给页面显示的元数据（是否占位、文件路径等）
    "enabled": False,
    "placeholder": False,
    "flag": "",
    "x_path": "",
    "k_path": "",
}
_EYE_PLANE_Z = None        # 抓取平面高度 cm；None = 用 _IK_MAP["z"]（默认 2.0，深度相机可覆盖）

# 定位径向系统偏差修正系数（2026-11-13 实测标定，只用于「眼在手」这条链）。
#
# 大白话：localize 用 FK 算「相机此刻在哪」，可 FK 是一套理想角度 —— 实际舵机被
#   自身和前臂的重量压下去一点点（C 姿态实测 J2 约 6.5°）。相机骑在腕上
#   （手眼矩阵 X 的平移 |t|≈16.7cm ≈ L3），这一压会被放大成「棋盘坐标系统性算近」。
# 规范说：光轴偏 δ 时，射线与 z=常数平面的交点位移 ≈ depth·tanδ/sinα
#   （α = 视线与平面夹角），沿视线方向、随深度增大 —— 所以"沿径向按比例缩放"
#   是一阶正确的修正形式，方位角不受影响。
# 实测依据（两个基准点，取值随棋盘真值变，务必看清）：
#   · 2026-11-12：定位 25.72cm，钢尺 28.70cm ⇒ k=1.116（旧棋盘位置）；
#   · 2026-11-14：定位 27.54cm，钢尺 30.04cm（x=−1.64/y=30.0 那次）
#     ⇒ k=30.04/27.54=1.091。当天改用 1.091 后，同姿态五连拍的
#     纵向误差稳定在 0.30cm，而 1.116 会系统性多推约 0.7cm ⇒ 取 1.091 为默认。
# ⚠️ 这是**单点经验系数**，不是标定出来的真值：改相机安装角、重标手眼 X、
#   换棋盘远近、或把 --tilt 换出 41° 附近，都必须重测。请求可用 radial_fix
#   覆盖（给 0 关闭），启动可用 --radial-fix 覆盖。
# ⚠️ 只修径向，**不修横向**：2026-11-14 实测定性，横向另有约 1.4cm 的
#   稳定系统误差（方位角偏 −2.7°），属手眼 X 旋转 / J1 map 比例，别拿 k 去凑。
_EYE_RADIAL_K = 1.091


def _radial_fix_xy(x, y, k):
    """把工作台坐标沿径向按系数 k 缩放，方位角保持不变。

    只改半径、不改方向：偏差来自「光轴俯仰角错了 δ」，射线沿自身方向前后滑，
    横向（方位角）几乎不动。所以径向缩放是"照着物理原因开的药"，
    而不是随手在 x/y 上各乘一个数。
    """
    r = math.hypot(x, y)
    if r < 1e-9 or abs(k - 1.0) < 1e-9:
        return x, y
    return x * k, y * k



def init_eye_in_hand(robot_dir, x_path, k_path, plane_z=None):
    """尝试加载眼在手链路；失败只记原因、绝不抛异常拖垮网页服务。

    大白话：给程序装上"会算 3D 的两只眼睛"，装不上就明确说装不上，别硬撑。
    规范定义：惰性构造 EyeInHandLocalizer（含 CalibData + Kinematics），
             失败降级不影响原有 pixel_to_table 通路。
    """
    global _EYE, _EYE_ERROR, _EYE_PLANE_Z
    d = os.path.expanduser(robot_dir or "~/kunpeng_vision")
    xp = os.path.expanduser(x_path or os.path.join(d, "handeye_X.json"))
    kp = os.path.expanduser(k_path or os.path.join(d, "camera_intrinsics.json"))
    _EYE_INFO.update(x_path=xp, k_path=kp)
    try:
        if d not in sys.path:
            sys.path.insert(0, d)
        from handeye_geometry import load_calib, Kinematics, EyeInHandLocalizer
        cal = load_calib(xp, kp)
        # 复用已经加载好的 ArmIK 实例，保证「位置->角度」用的是同一份标定表；
        # 若逆解模块没就绪（_IK is None），Kinematics 会自己新建一个。
        kin = Kinematics(os.path.join(d, "arm_params.json"), ik=_IK)
        _EYE = EyeInHandLocalizer(cal, kin)
        _EYE_ERROR = ""
        _EYE_PLANE_Z = float(plane_z) if plane_z is not None else None
        _EYE_INFO.update(enabled=True, placeholder=cal.placeholder,
                         flag=cal.flag_text())
        return True
    except Exception as e:
        _EYE = None
        _EYE_ERROR = "眼在手链路未启用（%s）：%s" % (d, e)
        _EYE_INFO.update(enabled=False, placeholder=False, flag="")
        return False


# ---------------------------------------------------------------------------
# 协议层（$<CMD>,...*<CK><CR><LF>，机械臂走 $ARM 子命令）
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
# 串口封装（后台读取线程，$ARM,J 只写不等，避免拖动时被应答阻塞卡顿）
# ---------------------------------------------------------------------------
# 单行字节上限。正常协议帧最长约 40 字节（"OK,S,C,1636,1470,..."），512 宽松到
# 不可能误伤；设上限只为防「噪声里一直没有换行符」时 line 无界增长吃内存。
_RX_LINE_MAX = 512


# ---------------------------------------------------------------------------
# 串口自动探测：让「这次枚举成 ttyUSB0、下次成 ttyUSB1」不再影响任何命令
# ---------------------------------------------------------------------------
# ① 用了什么语法：模块级常量 + 只读探测函数（os.listdir / open 读 sysfs）。
# ② 为什么这么设计：CH341 每次被 USB 复位都会重新枚举，内核分给它的 ttyUSB 编号
#    可能从 0 变成 1（本项目实测来回翻过）。但 /dev/serial/by-id/ 下的名字是按
#    芯片里的厂商串生成的，编号怎么变它都指向同一台设备 —— 所以优先用 by-id；
#    by-id 不存在时再退一步，按 USB 身份 1a86:7523 从 /sys 里把 CH341 认出来。
# ③ 不这么写会怎样：每换一次编号，所有 curl / 脚本里写死的 /dev/ttyUSB0 全失效，
#    现象是「串口打不开」或「命令发出去没反应」，每次都得先 ls 一遍再改命令。
SERIAL_AUTO = "auto"
_BY_ID_DIR = "/dev/serial/by-id"
_SYS_TTY_DIR = "/sys/class/tty"
_CH341_VID = "1a86"
_CH341_PID = "7523"


def detect_serial_port():
    """返回当前可用的串口路径；一个都找不到时返回 ""。

    Linux：① /dev/serial/by-id/usb-1a86*（稳定别名，首选）
           ② /sys 里 idVendor:idProduct = 1a86:7523 的 ttyUSBn
           ③ 兜底：任意一个 ttyUSBn
    Windows：serial.tools.list_ports 里 VID:PID 匹配 CH341 的 COMx
    """
    if os.name == "nt":
        try:
            from serial.tools import list_ports
        except Exception:
            return ""
        others = []
        for info in list_ports.comports():
            if info.vid == 0x1A86 and info.pid == 0x7523:
                return info.device
            if info.device:
                others.append(info.device)
        return others[0] if others else ""

    # ① 稳定别名。名字形如 usb-1a86_USB_Serial-if00-port0
    try:
        for name in sorted(os.listdir(_BY_ID_DIR)):
            if name.startswith("usb-" + _CH341_VID):
                return os.path.join(_BY_ID_DIR, name)
    except OSError:
        pass

    # ② 按 VID:PID 认设备。/sys/class/tty/ttyUSBn/device 指向 usb interface，
    #    往上一级（..）才是 usb device 目录，idVendor / idProduct 在那里。
    try:
        names = sorted(n for n in os.listdir(_SYS_TTY_DIR) if n.startswith("ttyUSB"))
    except OSError:
        names = []
    fallback = ""
    for name in names:
        dev_dir = os.path.join(_SYS_TTY_DIR, name, "device", "..")
        vid = pid = ""
        try:
            with open(dev_dir + "/idVendor") as f:
                vid = f.read().strip().lower()
            with open(dev_dir + "/idProduct") as f:
                pid = f.read().strip().lower()
        except OSError:
            pass
        if vid == _CH341_VID and pid == _CH341_PID:
            return "/dev/" + name
        if not fallback:
            fallback = "/dev/" + name
    return fallback


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
        # 非空 = 这条链路已经不可用，原因写在这里。send/fire 会据此显式报错。
        # 为什么必须有它：pyserial 不感知 USB 节点被删除，ser.is_open 仍返回 True、
        # write() 也不报错，数据全进废弃的 fd。没有这个标记时，上层只能看到
        # "命令发出去了、响应是空串"，会误判成 STM32 没应答（2026-11-10 实测踩到）。
        self._dead_reason = ""
        # 打开时记下「这个设备节点是谁」（inode + 创建时间）。为什么需要：CH341 被 USB
        # 复位/重新枚举后 /dev/ttyUSB0 会被销毁重建，而旧 fd 依旧 is_open=True、write()
        # 也不报错 —— 只有比对节点身份，才能识破「名字还叫 ttyUSB0、其实已经是另一个
        # 设备」这种假活（2026-11-11 实测：命令全进黑洞，接口却回 ok:true / resp:""）。
        self._dev_id = self._read_dev_id()
        self._reader = threading.Thread(target=self._rx_loop, daemon=True)
        self._reader.start()
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        try:
            self.send("LOG,0")   # 关闭周期遥测，保持 request-response 干净
        except Exception:
            pass

    def _read_dev_id(self):
        """取设备节点的身份 (st_dev, st_ino, st_ctime)。

        不是设备文件 / 平台不支持（如 Windows 的 COM7）时返回 None，
        表示「本平台不做这项检测」，调用方按 False 处理。
        """
        try:
            st = os.stat(self.port)
            return (st.st_dev, st.st_ino, int(st.st_ctime))
        except OSError:
            return None

    def port_replaced(self):
        """True = 现在的设备节点已经不是打开时那一个（被重枚举，或被拔掉）。

        为什么要主动比对：USB 串口掉线后 read() 不抛异常，只会一直返回 b''（EOF 语义），
        所以「读线程还活着」完全不代表「链路还活着」—— 必须看节点本身还是不是原来那个。
        """
        if self._dev_id is None:
            return False          # 平台不支持检测（Windows COMx），不做判断
        now = self._read_dev_id()
        if now is None:
            return True           # 节点消失 = 设备已被拔掉
        return now != self._dev_id

    def _rx_loop(self):
        """后台不断收字节，按 \\r\\n 组装成行放入 rxq。"""
        line = bytearray()
        last_check = time.time()   # 节点身份巡检时间戳（循环里每秒体检一次）
        while self._reader_alive:
            try:
                ch = self.ser.read(1)
            except Exception as e:
                # 只有"非主动关闭"的异常才算链路故障。close() 会先把 _reader_alive
                # 置 False（此时 ser 可能已是 None），那种情况不该记成断开原因。
                if self._reader_alive:
                    self._dead_reason = "串口读取线程异常退出（%s）" % e
                break
            # 每秒体检一次「节点还是不是打开时那一个」：USB 复位/拔插后 read() 只会永远
            # 返回 b''（EOF 语义），不抛异常，光靠上面的 except 抓不到，链路会一直"假活"。
            if time.time() - last_check >= 1.0:
                last_check = time.time()
                if self.port_replaced():
                    self._dead_reason = ("串口设备节点已重建或消失（CH341 被 USB 复位/拔插？），"
                                         "底层链路已失效")
                    break
            if not ch:
                continue
            # ① 帧头重同步：一见到 "$" 就把之前攒下的残渣整个丢掉，从这一帧重新起算。
            # 为什么需要：printf 重定向到 huart1，与协议共用同一条链路，一旦混进一段
            # 没有换行符的裸文本（噪声、被截断的提示语），它就会和下一帧粘成一行；
            # 而下面要求「整行以 $ 开头」，于是那一帧被整条丢弃，上层只能看到
            # "600 ms 内未收到任何字节"。2026-11-12 实测：PING 正常、紧随其后的
            # $ARM,READ 偶发超时，原样重发即恢复 —— 就是这一帧被吃掉。
            # 安全性：协议里 "$" 只作帧头，载荷字符集是 A-Z/0-9/逗号/负号/点/冒号，
            # 不含 "$"，所以「见到 $ 就重来」不会切坏正常帧。
            if ch == b"$":
                line = bytearray(ch)
                continue
            if ch in (b"\r", b"\n"):
                if line:
                    s = bytes(line).decode("ascii", errors="replace").strip()
                    # 只收 "$" 开头的协议帧。printf 重定向到 huart1 与协议共用链路，
                    # 上电 banner / RC522·KEY0 提示 / 周期状态打印（非协议帧的裸文本）
                    # 及 ESP AT 回显都是裸文本；必须丢弃，否则 send() 会误取为应答导致解析失败。
                    if s.startswith("$"):
                        s = s[1:]                 # 去掉帧头 $
                        if "*" in s:
                            s = s.split("*")[0]   # 去掉 *CK 校验段
                        with self.rlock:
                            self.rxq.append(s)
                    line = bytearray()
                continue
            # ② 超长行一定不是协议帧：直接丢掉、等下一个 "$" 重新同步。
            # 不加这条，噪声持续没有换行符时 line 会一路涨下去（无界）。
            if len(line) >= _RX_LINE_MAX:
                line = bytearray()
                continue
            line += ch

    def close(self):
        self._reader_alive = False
        self._dead_reason = "串口已关闭"
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        finally:
            self.ser = None
            self._reader = None

    def _write(self, payload):
        # 链路已死就直接报错，别往废弃的 fd 里灌数据装成功 —— 详见 __init__ 里的注释。
        if self._dead_reason:
            raise RuntimeError("%s；请重新 POST /api/open 连串口" % self._dead_reason)
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("串口已断开")
        # 写之前再确认一次节点身份：读线程可能还没来得及体检（最多晚 1 秒），
        # 这一句能保证「绝不把命令写进已经废弃的 fd」。
        if self.port_replaced():
            self._dead_reason = ("串口设备节点已重建或消失（CH341 被 USB 复位/拔插？），"
                                 "底层链路已失效")
            raise RuntimeError("%s；请重新 POST /api/open 连串口" % self._dead_reason)
        self.ser.write(make_frame(payload))

    def send(self, payload, timeout=0.6):
        """写帧并等待一条新应答（$ARM,J 等无应答命令由调用侧走 fire 短路）。"""
        with self.wlock:
            with self.rlock:
                self.rxq.clear()          # 丢弃历史残留（含 $ARM,J 应答），保证读到的就是本次应答
            self._write(payload)
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._dead_reason:
                # 等待期间读线程死了（USB 热拔/复位）：立刻报错。
                # 若继续等满 timeout，上层只会拿到空串，误判成"STM32 没应答"。
                raise RuntimeError("%s；请重新 POST /api/open 连串口" % self._dead_reason)
            with self.rlock:
                if self.rxq:
                    return self.rxq.pop(0)
            time.sleep(0.005)
        # 超时不再返回空串：空串会被上层当成"STM32 没说话"而继续走成功分支，
        # 这正是 2026-11-11「ok:true + resp:''」假成功的来源。分不清就干脆报错。
        raise RuntimeError("应答超时：%d ms 内未收到任何字节（串口可能已掉线，或 STM32 未运行；"
                           "先 POST /api/open 重连，仍空则查 12V/5V 供电与 CH341 杜邦线）"
                           % int(timeout * 1000))

    def fire(self, payload):
        """只写不等应答：供 $ARM,J 拖动用，绝不阻塞。"""
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
                # 上电 banner / RC522·KEY0 提示 / 周期状态打印（非协议帧的裸文本）
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
        """写帧并等待一条新应答（$ARM,J 等无应答命令由调用侧走 fire 短路）。"""
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
        # 与 SerialHost.send 同一约定：超时抛异常，绝不返回空串（空串 = 假成功）。
        raise RuntimeError("应答超时：%d ms 内未收到任何字节（设备可能已断开；"
                           "WiFi/ESP 模式请检查 ESP 链路）" % int(timeout * 1000))

    def fire(self, payload):
        """只写不等应答：供 $ARM,J 拖动用，绝不阻塞。"""
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
                # 上电 banner / RC522·KEY0 提示 / 周期状态打印（非协议帧的裸文本）
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
        # 与 SerialHost.send 同一约定：超时抛异常，绝不返回空串（空串 = 假成功）。
        raise RuntimeError("应答超时：%d ms 内未收到任何字节（设备可能已断开；"
                           "WiFi/ESP 模式请检查 ESP 链路）" % int(timeout * 1000))

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


def auto_reconnect():
    """串口链路已经死掉时，按自动探测重连一次；真的重连了才返回 True。

    为什么允许「自动」：CH341 被 USB 复位后 ttyUSB 编号会变，旧 fd 只会静默返回
    b''。设备全场只有一台，重连的目标没有任何歧义，让人再手打一次端口号毫无意义。
    为什么必须限定条件：只有「根本没连」或「读线程已判定链路失效（_dead_reason）」
    才重连。普通 600ms 应答超时可能只是 STM32 正在运动，乱重连会把命令打断。
    """
    global _HOST
    if _MODE != "serial":
        return False                      # WiFi 通道没有「节点重建」这回事
    if _HOST is not None:
        if not getattr(_HOST, "_dead_reason", ""):
            return False
        try:
            _HOST.close()
        except Exception:
            pass
        _HOST = None
    port = detect_serial_port()
    if not port:
        return False
    try:
        _HOST = SerialHost(port)
    except Exception as e:
        _HOST_STATE.update(connected=False, error=str(e))
        return False
    _HOST_STATE.update(connected=True, port=port, error="")
    return True


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
    """前端 1s 轮询 /api/state + 拖动 60ms 节流发 $ARM,J，会把终端刷满
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
  /* 标定预设：整块常驻，不折叠。列表用滚动区限高，组多了也不把页面撑长。 */
  .pbox{margin-top:10px;padding-top:9px;border-top:1px dashed var(--line);}
  .pbox .ph{font-size:12px;color:var(--muted);}
  .plist{max-height:168px;overflow-y:auto;margin-top:8px;}
  .prow{display:flex;align-items:center;justify-content:space-between;gap:8px;padding:7px 9px;margin-bottom:6px;border:1px solid var(--line);border-radius:8px;cursor:pointer;}
  .prow:hover{background:#1a2240;}
  .prow b{font-size:12px;font-weight:500;}
  .prow .pn{font-size:11px;color:#6f7aa0;margin-top:2px;}
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
        ⚠ 只做「像素 → 桌面平面 → 逆解 → 位置」，没有深度相机、没有力控、没有关节回读。
        确认下发会让<b>六路舵机同时运动</b>，请先确认臂周围无人、无障碍物。
      </div>
      <div class="row" style="margin-top:8px">
        <button class="sm" id="btnIkDry" onclick="ikSolve(false)" disabled>① 试算（不动臂）</button>
        <button class="sm ac" id="btnIkApply" onclick="ikSolve(true)" disabled>② 确认下发</button>
        <button class="sm" onclick="ikClear()">清除取点</button>
      </div>
      <div id="ikOut" style="font-family:ui-monospace,Menlo,monospace;font-size:11px;line-height:1.7;color:#a9c6e8;margin-top:9px;white-space:pre-wrap">(未取点)</div>
      <div class="pbox">
        <div class="ph">画面 → 桌面 标定参数（现场调，改完点「保存参数」生效）</div>
        <div class="tip" style="margin-top:8px;line-height:1.6">
          大白话：这 5 个数是给「点击画面取点」用的一把尺子 —— 告诉网页整幅画面对应桌面多宽多高、中心在哪、抓取高度多少；
          点一下画面才能把像素换成厘米，再交给逆解算六路位置。属平面约束单目定位，<b>不是真 3D</b>。
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
            <input type="text" id="ik_z" placeholder="如 2" title="抓取高度 cm（深度相机可给）">
          </div>
        </div>
        <div class="row" style="margin-top:8px">
          <button class="sm" onclick="ikSaveCalib()">保存参数</button>
          <span class="tip" id="ikCalibTip"></span>
        </div>
        <div class="row" style="margin-top:10px">
          <input type="text" id="ikPresetName" placeholder="给这组起个名字，如 观察位A" style="flex:1;min-width:120px">
          <button class="sm" onclick="ikPresetSave()">存为预设</button>
        </div>
        <div class="ph" style="margin-top:9px">已保存的参数组（<span id="ikPresetCount">0</span>）· 点一行即切换生效，行尾可删除</div>
        <div class="plist" id="ikPresetList"></div>
      </div>
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
        <button onclick="api('ARM,READ')">读位置</button>
        <button onclick="api('ARM,HOME')">回零位</button>
      </div>
    </section>

    <section class="panel">
      <h2><span>关节控制 · 6 路</span><span class="tip">实际值 1s 回填，滑杆即真实位置</span></h2>
      {% for i in range(6) %}
      <div class="jcard" data-i="{{i}}">
        <div class="jtop">
          <span class="jname">{{names[i]}} <span class="jlim">{{mins[i]}} ~ {{maxs[i]}}</span></span>
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
        <button onclick="apiRaw('ARM,J,6,300')">夹爪张开 300</button>
        <button onclick="apiRaw('ARM,J,6,674')">夹爪闭合 674</button>
        <button class="ok" onclick="autoInit()">六轴回零位</button>
        <button onclick="api('ARM,STOP')">停止</button>
        <button onclick="api('ARM,RESUME')">继续</button>
      </div>
    </section>
  </div>

</div>

<div class="metrics">
  <div class="metric"><div class="lb">目标 / 实际最大偏差</div><div class="vl" id="mDev">0 单位</div></div>
  <div class="metric"><div class="lb">识别帧率</div><div class="vl" id="mFps">-- fps</div></div>
  <div class="metric"><div class="lb">当前目标</div><div class="vl" id="mTarget" style="color:var(--ok)">—</div></div>
  <div class="metric" style="padding:0;border:none;background:none">
    <button class="estop" onclick="api('STOP'); api('ARM,STOP')">紧急停止<br>底盘+机械臂</button>
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
function sendDrag(i,v){ dragNext[i]=null; doCmd('ARM,J,'+(i+1)+','+v); }
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
  api('ARM,J,'+(i+1)+','+el.value);
}

// 读实际位置回填界面（HOME/$ARM 后滑杆位置与实际一致）
let lastPollErr='';   // 上次轮询失败的原因：只在「原因变了」时写日志，避免 1 秒一条刷屏
async function syncPulses(){
  if(!connected) return;
  const r=await post('/api/state');
  if(r&&r.ok&&Array.isArray(r.positions)&&r.positions.length===6){
    lastPollErr='';
    for(let i=0;i<6;i++){
      const s=document.getElementById('s'+i);
      if(!s) continue;
      // 刚拖过这路就先别覆盖：$ARM,READ 回的是运动过程中的中间值，
      // 强行写回会把滑块拉回去，看起来像「拖不动 / 弹回去」。
      if(dragAt[i] && Date.now()-dragAt[i] < DRAG_HOLD_MS) continue;
      s.value=String(r.positions[i]);
      tag('v'+i, r.positions[i]);
    }
  } else if(r&&r.ok===false){
    // 链路坏了必须让人看见：以前这里静默失败，界面 LED 还绿着、滑杆一动不动，
    // 完全看不出是通信断了还是自己没点动。
    if(r.error!==lastPollErr){ lastPollErr=r.error; log('✗ 状态轮询失败：'+r.error); }
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
  doCmd('ARM,J,'+(i+1)+','+v);
}
function goInit(i){ api('ARM,J,'+(i+1)+','+({{inits|tojson}})[i]); }

async function autoInit(){
  await doCmd('ARM,HOME');
}

// =========================================================================
// 以下为 UI 升级新增部分（视频链路 / 检测叠加 / 遥测）
// 设计原则：上面的串口收发函数一个字都不改，只在这里「包装」和「追加」。
// 好处：即使新功能全挂了，PING/$ARM 仍然按原来已实机验证的方式工作。
// =========================================================================

// 从滑杆 DOM 读限位和初始位，避免再往模板里塞一份数据（单一数据源）
const INITS = [], MINS = [], MAXS = [];
for (let i = 0; i < 6; i++) {
  const s = document.getElementById('s' + i);
  INITS.push(+s.dataset.init); MINS.push(+s.min); MAXS.push(+s.max);
}

const stat = {
  cmds: 0,                 // 累计下发指令数
  targets: INITS.slice(),  // 最后一次给每路下达的目标位置（初值=初始位）
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
  const m = /^ARM,J,(\d),(\d+)/.exec(payload);
  if (m) stat.targets[+m[1] - 1] = +m[2];
  const modeText = payload === 'STOP' || payload === 'ARM,STOP' ? '急停'
                 : payload === 'RESUME' || payload === 'ARM,RESUME' ? '已恢复'
                 : '指令态';
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

// 偏差 = 最后一次 $ARM 目标 vs 1s 回填的真实位置，取 6 路最大值。
// 只读 DOM，不额外发 $ARM,READ —— 绝不增加串口流量。
setInterval(function () {
  let mx = 0;
  for (let i = 0; i < 6; i++) {
    const s = document.getElementById('s' + i);
    if (s) mx = Math.max(mx, Math.abs(stat.targets[i] - +s.value));
  }
  tag('mDev', mx + ' 单位');
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
    if (!confirm('确认下发 $ARM,POS 到机械臂？\n六路目标位置：' + ikLast.positions.join(', ')
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
        + '目标位置：' + r.positions.map(function (p, i) {
            return IK_NAMES[i] + '=' + p;
          }).join('  ') + '\n';
  s += (r.clamped && r.clamped.length)
     ? '⚠ 超出软限位被裁剪：' + r.clamped.join(',') + '（IK 解不完全可达）\n'
     : '✓ 六路均在软限位内\n';
  if (r.applied) {
    s += '\n已下发：ARM,POS,' + r.positions.join(',') + '\n固件应答：' + (r.resp || '(空)') + '\n';
    if (r.state_positions && r.state_positions.length === 6)
      s += '$ARM,READ 瞬时值：' + r.state_positions.join(',') + '\n  → ' + r.state_note + '\n';
    s += r.note;
  } else {
    s += '\n【仅试算，未下发】确认无误后再点「② 确认下发」。';
  }
  tagIk(s);
  document.getElementById('btnIkApply').disabled = r.applied;   // 下发过就禁用，防连点
  log(r.applied ? ('<< ' + r.resp + ' | 目标 ' + r.positions.join(','))
                : ('<< 试算完成 目标 ' + r.positions.join(',')));
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

// ---- 标定参数预设：人工保存 / 点一行切换 / 随时删除 ----
// 存在浏览器 localStorage：刷新页面、重启 5000 服务都还在。
// 注意 _IK_MAP 在 Flask 进程内存，重启后仍回默认 30/22.5/0/18/2，
// 这时点一下已保存的预设即可一键恢复，不用手填 5 个数。
var IK_PRESET_KEY = 'web_control_ik_presets_v1';
var IK_KEYS = ['span_x', 'span_y', 'center_x', 'center_y', 'z'];

function ikEscape(s) {
  return String(s).replace(/[&<>"']/g, function (c) {
    return { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c];
  });
}

function ikPresetLoad() {
  try {
    var arr = JSON.parse(localStorage.getItem(IK_PRESET_KEY) || '[]');
    return Object.prototype.toString.call(arr) === '[object Array]' ? arr : [];
  } catch (e) { return []; }
}

function ikPresetWrite(arr) {
  try {
    localStorage.setItem(IK_PRESET_KEY, JSON.stringify(arr));
    return true;
  } catch (e) {
    tag('ikCalibTip', '✗ 浏览器存储不可用：' + e.message);
    return false;
  }
}

function ikReadFive() {
  var body = {};
  for (var i = 0; i < IK_KEYS.length; i++) {
    var k = IK_KEYS[i];
    var v = parseFloat(document.getElementById('ik_' + k).value);
    if (!isFinite(v)) { tag('ikCalibTip', '✗ ' + k + ' 不是数字'); return null; }
    body[k] = v;
  }
  return body;
}

function ikPresetRender() {
  var arr = ikPresetLoad();
  var box = document.getElementById('ikPresetList');
  if (!box) return;
  tag('ikPresetCount', arr.length);
  if (!arr.length) {
    box.innerHTML = '<div class="tip">还没有预设：填好上面 5 个数、起个名字，点「存为预设」。</div>';
    return;
  }
  var html = '';
  for (var i = 0; i < arr.length; i++) {
    var m = arr[i].map || {};
    var nums = [];
    for (var j = 0; j < IK_KEYS.length; j++) { nums.push(m[IK_KEYS[j]]); }
    html += '<div class="prow" onclick="ikPresetApply(' + i + ')">'
          + '<div><b>' + ikEscape(arr[i].name) + '</b>'
          + '<div class="pn">' + nums.join(' / ') + ' cm</div></div>'
          + '<button class="sm" onclick="event.stopPropagation();ikPresetDelete(' + i + ')">删除</button>'
          + '</div>';
  }
  box.innerHTML = html;
}

function ikPresetSave() {
  var body = ikReadFive();
  if (!body) return;
  var name = (document.getElementById('ikPresetName').value || '').trim();
  if (!name) { tag('ikCalibTip', '✗ 先给这组起个名字'); return; }
  var arr = ikPresetLoad();
  var hit = -1;
  for (var i = 0; i < arr.length; i++) { if (arr[i].name === name) { hit = i; break; } }
  var item = { name: name, map: body, ts: new Date().toLocaleString() };
  if (hit >= 0) { arr[hit] = item; } else { arr.push(item); }
  if (!ikPresetWrite(arr)) return;
  document.getElementById('ikPresetName').value = '';
  tag('ikCalibTip', hit >= 0 ? '✓ 已更新预设「' + name + '」' : '✓ 已保存预设「' + name + '」');
  ikPresetRender();
}

async function ikPresetApply(idx) {
  var arr = ikPresetLoad();
  var item = arr[idx];
  if (!item) return;
  var body = {};
  for (var i = 0; i < IK_KEYS.length; i++) {
    var k = IK_KEYS[i];
    var v = parseFloat(item.map[k]);
    if (!isFinite(v)) { tag('ikCalibTip', '✗ 预设「' + item.name + '」数据损坏'); return; }
    body[k] = v;
    document.getElementById('ik_' + k).value = v;
  }
  var r = await post('/api/ik/calib', body);
  if (r && r.ok) {
    tag('ikCalibTip', '✓ 已切换到「' + item.name + '」并生效');
    log('标定参数已切换为预设「' + item.name + '」：' + JSON.stringify(body));
  } else {
    tag('ikCalibTip', '✗ ' + (r ? r.error : '请求失败'));
  }
}

function ikPresetDelete(idx) {
  var arr = ikPresetLoad();
  var item = arr[idx];
  if (!item) return;
  if (!confirm('删除预设「' + item.name + '」？ 只删浏览器里存的这一组，不影响当前已生效的参数。')) return;
  arr.splice(idx, 1);
  if (!ikPresetWrite(arr)) return;
  tag('ikCalibTip', '✓ 已删除预设「' + item.name + '」');
  ikPresetRender();
}

// script 位于 body 末尾，DOM 已就绪，这里直接渲染一次预设列表。
ikPresetRender();

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
        mins=[r[0] for r in POS_RANGES],
        maxs=[r[1] for r in POS_RANGES],
        inits=[r[2] for r in POS_RANGES],
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
    port = (data.get("port") or "").strip()
    # 不传 port（或传 "auto"）= 让服务自己找：优先 by-id 稳定别名，其次 1a86:7523。
    # 为什么不沿用 _HOST_STATE 里的旧端口名：编号一变（ttyUSB0<->1）那个名字就废了，
    # 沿用只会得到「无法打开串口」这种看不出真因的错误。
    if port in ("", SERIAL_AUTO):
        port = detect_serial_port() or _HOST_STATE["port"]
    if not port:
        _HOST_STATE.update(connected=False, error="没有找到任何串口")
        return jsonify(ok=False, error="没有找到任何串口：先确认 CH341 已插入（ls /dev/ttyUSB*）")
    try:
        _HOST = SerialHost(port)
    except Exception as e:
        _HOST_STATE.update(connected=False, error=str(e))
        return jsonify(ok=False, error=str(e))

    # 端口能打开 ≠ 链路可用：再 PING 一次，把「串口开了、但 STM32 一句话不说」
    # 当场暴露出来。为什么仍回 ok=True：端口确实开了，报 ok:false 会被误读成
    # 「要再连一次」，而真正该查的是供电与接线 —— 所以用 warning 字段说清楚。
    ping = ""
    warning = ""
    try:
        ping = _HOST.cmd("PING")
    except Exception as e:
        warning = str(e)
    _HOST_STATE.update(connected=True, port=port, error=warning)
    if warning:
        return jsonify(ok=True, port=port, ping=ping, warning=warning)
    return jsonify(ok=True, port=port, ping=ping)


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
        auto_reconnect()                  # 没连就先自动连一次（端口自动探测，无需端口名）
    if _HOST is None:
        return jsonify(ok=False, error="未连接：没找到串口（先确认 CH341 已插入）")
    is_move = payload.upper().startswith("ARM,J")

    def _send_once():
        # $ARM,J 拖动高频下发，走 fire 只写不等待，保证拖动实时跟手
        if is_move:
            _HOST.fire(payload)
            return ""
        return _HOST.cmd(payload)

    try:
        return jsonify(ok=True, resp=_send_once())
    except Exception as e:
        # 链路已被判失效（CH341 重新枚举、ttyUSB 编号互换）时，自动重连一次再重发。
        # 只有 _dead_reason 已置位才会真重连：普通 600ms 应答超时不会触发重发。
        if auto_reconnect():
            try:
                return jsonify(ok=True, resp=_send_once(), reconnected=True)
            except Exception as e2:
                return jsonify(ok=False, error=str(e2))
        return jsonify(ok=False, error=str(e))


@app.route("/api/state", methods=["POST"])
def api_state():
    global _HOST
    if _HOST is None:
        auto_reconnect()
    if _HOST is None:
        return jsonify(ok=False, error="未连接：没找到串口（先确认 CH341 已插入）")
    try:
        resp = _HOST.cmd("ARM,READ")
    except Exception as e:
        if not auto_reconnect():
            return jsonify(ok=False, error=str(e))
        try:
            resp = _HOST.cmd("ARM,READ")
        except Exception as e2:
            return jsonify(ok=False, error=str(e2))
    parts = resp.split(",")
    # 必须显式校验长度：$OK,POS,p1..p6 共 8 段；不足时切片 out-of-range 不抛异常，
    # 只会悄悄返回少于 6 路位置，页面拿到长度不对的数组就等于白问一次。
    if len(parts) < 8:
        return jsonify(ok=False, error="$ARM,READ 响应异常：%r（应形如 OK,POS,p1..p6）" % resp)
    try:
        positions = [int(tok) for tok in parts[2:8]]
    except ValueError:
        return jsonify(ok=False, error="$ARM,READ 响应异常：%r（位置不是整数）" % resp)
    return jsonify(ok=True, positions=positions)


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
# A 方案：点选抓取的三个接口（逆解实现见文件顶部 _IK / pixel_to_table / solve_to_positions）
#
# ① 用了什么语法：Flask 路由 + request.get_json + 字典组装 jsonify + 分层 try/except。
# ② 为什么这么设计：拆成「看状态 / 改标定 / 解算」三个独立接口，且解算默认
#    apply=False 只试算 —— 六路舵机会同时运动，必须让人先看到位置再确认下发。
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
        ranges=POS_RANGES,
        eye=dict(_EYE_INFO, error=_EYE_ERROR),
    )


@app.route("/api/ik/eye")
def api_ik_eye():
    """给页面看眼在手链路状态：是否启用、用的哪两个文件、是否占位数据。

    单独一个接口而不是塞进 /api/ik/config：眼在手是后加的独立算路，
    页面可以只关心它、也可以在它异常时给出单独的提示，不互相牵连。
    """
    return jsonify(ok=True, eye=dict(_EYE_INFO, error=_EYE_ERROR),
                   plane_z=(_EYE_PLANE_Z if _EYE_PLANE_Z is not None else _IK_MAP["z"]))


@app.route("/api/ik/eye/solve", methods=["POST"])
def api_ik_eye_solve():
    """像素 + 当前关节角 -> 手眼链 -> 工作台 3D 点 -> IK -> 6 路位置。

    与 /api/ik/solve 的区别（这是「A」相对「A 退化版」的核心升级）：
        · 旧：x = center_x + (u/w-0.5)*span_x —— 5 个手工常量，隐含「拍照姿态固定」；
        · 新：K 反投影出射线 → 用**当前 6 路实际位置**做 FK 得 T_base_end
              → T_base_cam = T_base_end·X → 与 z=Z 平面求交。
    因此臂换个姿态再点同一个像素，结果会跟着变 —— 这是对的，因为相机跟着动了。

    可靠性说明：关节角必须来自真实串口 $ARM,READ。若串口没连，宁可直接报错，
    也不拿初始位顶替 —— 那会算出"看起来对、其实差十几厘米"的坐标。
    apply=True 时才下发 $ARM,POS，且与旧接口一样先试算后确认。
    """
    if _EYE is None:
        return jsonify(ok=False, error=_EYE_ERROR or "眼在手链路未启用（启动加 --eye-in-hand）")

    data = request.get_json(force=True, silent=True) or {}
    u = float(data.get("u", 0))
    v = float(data.get("v", 0))

    # 关节角来源：优先页面传来的 positions（刚轮询到的），否则向串口要一次。
    # 为什么不默认用 _IK_MAP 的常量：$ARM,READ 是唯一能反映"相机此刻在哪"的信息源。
    positions = data.get("positions")
    if not (isinstance(positions, list) and len(positions) == 6):
        if _HOST is None:
            return jsonify(ok=False,
                           error="串口未连接，拿不到当前关节角：眼在手定位必须有真实位置"
                                 "（先点页面「连接」，或等 1s 轮询拿到 $ARM,READ）")
        try:
            sresp = _HOST.cmd("ARM,READ")
            parts = sresp.split(",")
            if len(parts) < 8:
                return jsonify(ok=False, error="$ARM,READ 响应异常，取不到 6 路位置：%r" % sresp)
            positions = [int(t) for t in parts[2:8]]
        except Exception as e:
            return jsonify(ok=False, error="读取当前关节角失败：%s" % e)
    else:
        positions = [int(p) for p in positions]

    plane_z = data.get("plane_z")
    plane_z = float(plane_z) if plane_z is not None else (_EYE_PLANE_Z if _EYE_PLANE_Z is not None else _IK_MAP["z"])

    try:
        est = _EYE.localize(u, v, positions, plane_z=plane_z)
    except ValueError as e:
        # 目标在视野外 / 视线与桌面平行 / 深度离谱，都归成一句能看懂的话
        return jsonify(ok=False, error=str(e), positions=positions)

    x_raw, y_raw, z = est["x"], est["y"], est["z"]

    # ---- 径向系统偏差修正：把「算近的那 3cm」按比例补出去 ----
    # 为什么放在这里、而不是改 handeye_geometry.localize：
    #   localize 是标定链的一环（refk_samples / AX=XB 都用它），往里塞经验系数会
    #   把 X 污染得不明不白；放在 API 出口，所有消费者（网页点选 / eye_check /
    #   pick_pipeline）一起受益，而且随时能用 radial_fix=0 关掉对比。
    # 只改半径不改方位角，理由见 _radial_fix_xy 的注释。
    try:
        _k = data.get("radial_fix")
        _k = float(_k) if _k is not None else _EYE_RADIAL_K
    except (TypeError, ValueError):
        _k = _EYE_RADIAL_K
    x, y = _radial_fix_xy(x_raw, y_raw, _k)
    radial_shift = math.hypot(x, y) - math.hypot(x_raw, y_raw)

    # 用眼在手算出的 z 参与逆解，而不是 _IK_MAP["z"]：平面已由射线求交确定，
    # 再套一个手工高度会自相矛盾。
    #
    # tilt：夹爪前倾角。0=垂直向下；>0 时腕关节不必退到爪尖正上方 L3 处，
    # 所以可达半径会明显变大（tilt=5° 约 +1.5cm，tilt=20° 约 +6cm）。
    #
    # 🔴 单一真相源（11-16 改）：默认**从当前真实位置反算**，不再用 _IK_MAP["tilt"]（恒 0）。
    # 为什么：tilt 是「当前姿态的固有属性」，不是可自由挑的参数 —— 相机骑在腕上，
    # 拍照那一刻的倾角决定相机位姿，抓取必须用同一个倾角。用常量 0 会在 C 姿态(41°) 下
    # 让 r_w 少扣 L3·sin41°≈11cm，d 虚高十几厘米，把明明够得着的目标误判成
    # 「超出可达范围」（11-16 实测：报 d=26.7，实际 d≈14.8）。
    # 优先级：请求显式给 tilt > 从 positions 反算 > （无 IK 时）不进来。
    _tilt_req = data.get("tilt")
    if _tilt_req is not None:
        try:
            tilt = float(_tilt_req)
        except (TypeError, ValueError):
            tilt = _IK.tilt_from_positions(positions)
    else:
        tilt = _IK.tilt_from_positions(positions)
    try:
        angles = _IK.solve(x, y, z=z, wrist=_IK_MAP["wrist"],
                           claw=_IK_MAP["claw"], tilt=tilt)
        raw = _IK.to_positions(angles)
    except Exception as e:
        return jsonify(ok=False, error="逆解失败：%s" % e, tilt=tilt,
                       x=round(x, 2), y=round(y, 2), z=round(z, 2),
                       x_raw=round(x_raw, 2), y_raw=round(y_raw, 2),
                       radial_k=round(_k, 4), radial_shift=round(radial_shift, 2))

    out_positions, clamped = [], []
    for i, p in enumerate(raw):
        lo, hi, _ = POS_RANGES[i]
        q = max(lo, min(hi, int(p)))
        if q != int(p):
            clamped.append(JOINT_NAMES[i])
        out_positions.append(q)

    out = dict(
        ok=True,
        mode="eye_in_hand",
        pixel=[u, v],
        x=round(x, 2), y=round(y, 2), z=round(z, 2),
        x_raw=round(x_raw, 2), y_raw=round(y_raw, 2),
        radial_k=round(_k, 4), radial_shift=round(radial_shift, 2),
        tilt=round(tilt, 2),
        depth=round(est["depth"], 2),
        positions=out_positions,
        clamped=clamped,
        source_positions=positions,
        placeholder=est["placeholder"],
        flag=est["flag"],
        applied=False,
        resp="",
    )

    if not data.get("apply"):
        out["note"] = "仅试算，未下发任何串口命令"
        if abs(_k - 1.0) > 1e-9:
            out["note"] += ("；已做径向修正 ×%.3f（%.2f→%.2f cm）"
                            % (_k, math.hypot(x_raw, y_raw), math.hypot(x, y)))
        if est["placeholder"]:
            out["note"] += "；⚠ 标定数据为占位值，坐标不代表真实精度"
        return jsonify(out)

    if _HOST is None:
        out["ok"] = False
        out["error"] = "串口未连接：试算结果已保留，先点页面「连接」再下发"
        return jsonify(out)

    payload = "ARM,POS,%s" % ",".join(str(int(p)) for p in out_positions)
    try:
        out["resp"] = _HOST.cmd(payload)
    except Exception as e:
        out["ok"] = False
        out["error"] = "$ARM,POS 下发失败：%s" % e
        return jsonify(out)

    out["applied"] = True
    out["frame"] = payload
    out["note"] = ("固件已回 OK（作业已受理），六轴由 ArmTask 异步运动"
                   if out["resp"].startswith("OK")
                   else "固件未回 OK，请检查链路（帧已发出）")
    return jsonify(out)


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
    """像素 (u,v,w,h) -> 桌面 (x,y) -> IK -> 6 路位置。

    apply=False（默认）：只试算，一个字节都不往串口发。
    apply=True：构造 ARM,POS,<p0>..<p5> 下发，并回读一次 $ARM,READ 的瞬时值。

    诚实边界：$ARM,READ 返回的是运动推进中的「当前位置」，不是「已到位确认」；
    到位与否靠页面已有的 1s 轮询 /api/state 持续观察，这里绝不假装确认。
    """
    data = request.get_json(force=True, silent=True) or {}
    try:
        x, y = pixel_to_table(
            float(data.get("u", 0)), float(data.get("v", 0)),
            float(data.get("w", 0)), float(data.get("h", 0)))
        positions, clamped = solve_to_positions(x, y)
    except (ValueError, RuntimeError) as e:
        # 目标过近 / 超出可达范围 / 逆解模块没就绪，都归成「能看懂的一句话」
        return jsonify(ok=False, error=str(e))
    except Exception as e:
        return jsonify(ok=False, error="逆解异常：%s" % e)

    out = dict(ok=True, x=round(x, 2), y=round(y, 2), z=_IK_MAP["z"],
               positions=positions, clamped=clamped, applied=False, resp="")

    if not data.get("apply"):
        out["note"] = "仅试算，未下发任何串口命令"
        return jsonify(out)

    if _HOST is None:
        out["ok"] = False
        out["error"] = "串口未连接：试算结果已保留，先点页面「连接」再下发"
        return jsonify(out)

    payload = "ARM,POS,%s" % ",".join(str(int(p)) for p in positions)
    try:
        out["resp"] = _HOST.cmd(payload)
    except Exception as e:
        out["ok"] = False
        out["error"] = "$ARM,POS 下发失败：%s" % e
        return jsonify(out)

    out["applied"] = True
    out["frame"] = payload
    if not out["resp"].startswith("OK"):
        out["note"] = "固件未回 OK，请检查链路（帧已发出）"
    else:
        out["note"] = "固件已回 OK（作业已受理）；六轴由 ArmTask 异步运动，页面 1s 轮询会持续刷新实际位置"

    # 回读一次 $ARM,READ：只展示「当下瞬时值」，绝不冒充到位确认。
    out["state_positions"] = []
    out["state_note"] = ""
    try:
        sresp = _HOST.cmd("ARM,READ")
        parts = sresp.split(",")
        if len(parts) >= 8:
            out["state_positions"] = [int(t) for t in parts[2:8]]
            out["state_note"] = "运动中的瞬时值，非最终到位；最终值看页面 1s 轮询"
    except Exception:
        out["state_note"] = "$ARM,READ 回读失败（不影响已下发的目标）"
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
    # global 必须放在函数最前面：下面的 --radial-fix help 要读 _EYE_RADIAL_K 的当前值，
    # 而「用到了才声明 global」在 Python 里是 SyntaxError（used prior to global declaration）。
    global _MODE, _WIFI_ADDR, _CAM, _VIDEO_URL, _DETECTIONS_URL, _HOST, _EYE_RADIAL_K

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
    ap.add_argument("--eye-in-hand", action="store_true",
                    help="启用眼在手定位（需 handeye_X.json + camera_intrinsics.json）；"
                         "默认关闭，保持旧的「画面→桌面」线性映射")
    ap.add_argument("--handeye-X", default="", metavar="FILE",
                    help="手眼矩阵文件，默认 <ik-dir>/handeye_X.json")
    ap.add_argument("--intrinsics", default="", metavar="FILE",
                    help="相机内参文件，默认 <ik-dir>/camera_intrinsics.json")
    ap.add_argument("--plane-z", type=float, default=None, metavar="CM",
                    help="抓取平面高度 cm（深度相机给；缺省用默认 z=2.0）")
    ap.add_argument("--radial-fix", type=float, default=None, metavar="K",
                    help="眼在手定位的径向修正系数：默认 %.3f（实测值）；给 1.0 关闭；"
                         "换相机安装/重标手眼 X 后必须重测" % _EYE_RADIAL_K)
    args = ap.parse_args()

    if args.radial_fix is not None:
        _EYE_RADIAL_K = float(args.radial_fix)

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
        # 串口号：命令行给了就用；没给（或写 auto）就自动探测，优先 by-id 稳定别名。
        port = args.serial
        if not port or port == SERIAL_AUTO:
            port = detect_serial_port() or "COM7"
        _HOST_STATE["port"] = port
        _HOST_STATE["baud"] = args.baud
        chan_desc = "串口 %s @ %d baud" % (port, args.baud)
        # 启动即连接。以前这里只记下端口名字、并不真正 open，必须手工 POST /api/open，
        # 忘了就一路「600ms 内未收到任何字节」，很容易被误判成硬件故障（实测踩过）。
        try:
            _HOST = SerialHost(port)
            ping = _HOST.cmd("PING")
            _HOST_STATE.update(connected=True, error="")
            print("  [串口] 已连接 %s，PING -> %s" % (port, ping))
        except Exception as e:
            _HOST_STATE.update(connected=False, error=str(e))
            print("  [警告] 启动时打开串口失败：%s" % e)
            print("         可稍后在页面点「连接」，或 POST /api/open（不传 port 即自动探测）")

    # ---- 视频链路（和串口控制链路完全独立，怎么改都不影响 PING/$ARM）----
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
    # ③ 不这么写会怎样：一个可选的抓取功能拖垮已跑通的 PING/$ARM/视频链路。
    ik_ok = init_ik(args.ik_dir)
    if ik_ok:
        ik_desc = "已就绪（%s）" % os.path.expanduser(args.ik_dir or "~/kunpeng_vision")
    else:
        ik_desc = "未启用：%s" % _IK_ERROR

    # ---- A+ 眼在手链路（必须显式 --eye-in-hand，默认不碰旧通路）----
    # 依赖 _IK 先加载好：Kinematics 复用它做「位置->角度」，保证标定表只有一份。
    if args.eye_in_hand:
        eye_ok = init_eye_in_hand(args.ik_dir, args.handeye_X, args.intrinsics,
                                  plane_z=args.plane_z)
        if eye_ok:
            eye_desc = "已启用%s" % ("（⚠ 占位数据）" if _EYE_INFO["placeholder"] else "（实测数据）")
        else:
            eye_desc = "未启用：%s" % _EYE_ERROR
    else:
        eye_desc = "未启用（加 --eye-in-hand 开启）"

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
    print("  眼在手：%s" % eye_desc)
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