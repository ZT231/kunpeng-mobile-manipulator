#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
gp_teleop.py —— 手柄遥控「麦轮底盘 + 六轴总线舵机臂」

链路（两条串口，互不相干）：
  手柄(Type-C / 2.4G 接收器) ──> 鲲鹏 /dev/input/jsN（标准 Linux 手柄接口，无需驱动库；
                                   不给 --js 就自动按设备名挑，插拔后编号变了也不怕）
  鲲鹏 本脚本 ─┬─ CH341 USB-TTL ──> STM32 USART1，发 $CAR,VEL / $CAR,STOP（115200 8N1）
               └─ Waveshare Bus Servo Adapter (A) ──> 舵机总线，发幻尔帧 55 55 ID LEN CMD …（115200 8N1）

分工：底盘速度环、PID、编码器、堵转保护都在 STM32 固件里跑，脚本只发目标速度；
六轴总线舵机由鲲鹏直连转接板驱动（转接板拨 **B 位 USB–SERVO**，发幻尔帧）。
🔴 这条路径与固件侧 `servo_bus.c`（USART3 + A 位 UART–SERVO）**互斥**：同一时刻只能有一方
驱动舵机总线。用本脚本时必须把转接板拨 B，且 `$ARM,*` 命令停用；改回 A 位则由固件接管，
本脚本的机械臂部分不要开。固件零改动指的是本脚本不碰固件代码，不是两条链路能同时跑。

两个模式（沿用成品手柄遥控器的做法：同一根摇杆按模式复用，而不是把所有机构塞进一套按键）：
  DRIVE 驾驶：左摇杆平移、右摇杆 X 自转、LT 油门、RT 精细档
  ARM   机械臂：左摇杆 J1/J2、右摇杆 J3/J4、十字键 J5、LB/RB 夹爪、LT 加速、RT 减速

依赖：标准库 + pyserial（板上已装）。js 设备用原始 struct 解析，不依赖 pygame / evdev。

安全：切换模式即刹停底盘；任一路异常 / Ctrl-C / 进程被杀 —— 退出前必发 $CAR,STOP 与
所有舵机 MOVE_STOP；底盘固件另有 500ms 收不到 VEL 自动刹停的兜底。
'''

import argparse
import os
import select
import signal
import struct
import time

# ============================ 1. 配置区（按实车改这里） ============================

# --- 串口 ---
CAR_PORT_DEFAULT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"  # CH341 不翻号，用 by-id
CAR_BAUD = 115200
ARM_BAUD = 115200
ARM_SEND_MS = 80          # 每次下发的舵机行程时间（ms）；越小越跟手、越大越平滑

# --- 控制节奏 ---
TICK_S = 0.04             # 25Hz 主循环；底盘 500ms 看门狗，必须远小于它

# --- 底盘速度上限（满油门时） ---
# 固件仲裁：|vx|,|vy| ≤ 1.0m/s、|wz| ≤ 5.0rad/s；机械侧轮速上限 0.45m/s
VX_MAX = 0.30             # m/s 前进
VY_MAX = 0.25             # m/s 横移
WZ_MAX = 1.20             # rad/s 自转（约 69°/s）

# --- 输入整形（手感全在这里，可按自己的习惯微调；见 README 第 7.1 节） ---
STICK_DZ = 0.07           # 摇杆死区；松手若有轻微蠕动就加到 0.08~0.10
STICK_EXPO = 1.4          # 摇杆指数：1.0=线性，越大中心越柔、前段越空
TRIG_DZ = 0.03            # 扳机死区：只挡静止噪声，油门不该有明显空行程
TRIG_EXPO = 1.0           # 扳机默认线性：油门行程 = 速度行程，最好控
ARM_DZ = 0.06             # 机械臂摇杆死区
ARM_EXPO = 1.25           # 机械臂指数：比底盘略线性，方便微调定位
FINE_SCALE = 0.5          # 按住 RT：底盘与机械臂速度都减半
BOOST_MAX = 2.0           # 机械臂模式下 LT 压到底的加速倍数（1.0 → 2.0）

# --- 手柄映射（Linux js 编号；换手柄/换模式后先 --probe 核对，必要时 --axis 覆盖） ---
# 索引, 类型：s=双极摇杆(-32767~32767)  t=单极扳机(0~255 或 -32767~32767)
AXES = {
    "lx": (0, "s"),   # 左摇杆 X
    "ly": (1, "s"),   # 左摇杆 Y
    "lt": (2, "t"),   # 左扳机
    "rx": (3, "s"),   # 右摇杆 X
    "ry": (4, "s"),   # 右摇杆 Y
    "rt": (5, "t"),   # 右扳机
    "dx": (6, "s"),   # 十字键 X（±32767，居中报 0）
    "dy": (7, "s"),   # 十字键 Y
}
BTNS = {
    "A": 0, "B": 1, "X": 2, "Y": 3,
    "LB": 4, "RB": 5, "BACK": 6, "START": 7,
    "M1": 10, "M2": 11,   # 背键：XInput 下编号因固件而异，--probe 现场确认
}
# 模式切换：背键 M1 为主、BACK 为备用（背键若不被 js 上报，BACK 照样能切）
MODE_SWITCH_BTNS = ("M1", "BACK")
# 保留键：任何情况下不参与关节绑定
RESERVED_BTNS = ("A", "B", "START")

# --- 六轴软限位与速度（单位：0~1000 ↔ 0~240°，1 单位 = 0.24°） ---
# home/dir 已于 2026-11-26 回填标定值（与 arm_params.json 一致）；
# lo/hi 除 J6（已实测 300~674）外仍是保守值，装好后按结构极限留 40 单位余量回填。
# dir=+1 表示「输入正方向」= 目标值变大；与标定 dir 一致，装反了改 dir=-1（不要拆舵机）。
JOINTS = [
    {"name": "J1", "id": 1, "lo": 100, "hi": 900, "spd": 140, "home": 460, "dir": +1},  # 底座回转（零位 460）
    {"name": "J2", "id": 2, "lo": 200, "hi": 820, "spd": 100, "home": 514, "dir": -1},  # 大臂抬降（零位 514）
    {"name": "J3", "id": 3, "lo": 200, "hi": 850, "spd": 110, "home": 516, "dir": +1},  # 肘伸缩（零位 516）
    {"name": "J4", "id": 4, "lo": 250, "hi": 800, "spd": 110, "home": 502, "dir": +1},  # 腕俯仰（零位 502）
    {"name": "J5", "id": 5, "lo": 300, "hi": 700, "spd": 140, "home": 494, "dir": -1},  # 腕滚转（零位 494）
    {"name": "J6", "id": 6, "lo": 300, "hi": 674, "spd": 140, "home": 674, "dir": -1},  # 夹爪（300=开、674=合；零位=闭合）
]

# --- 机械臂模式：关节 <- 输入 绑定 ---
# 注释里的"推上/右推"按标准 joydev 轴方向理解（右推/下推=+1、左推/上推=−1）。
# 实机用 --probe 核对轴符号，若某轴与注释相反，改该行 gain 符号（+1 ↔ −1），不要改 dir。
# ("ax", 轴名, 增益) 模拟量；("bt", 键名, 增益) 数字键按住连续走
ARM_MAP = {
    "J1": [("ax", "lx", -1.0)],                       # 左摇杆 左右：右推 = 位置减小 = 俯视顺时针（与驾驶模式右推右转一致）
    "J2": [("ax", "ly", -1.0)],                       # 左摇杆 上下：推上 = 位置减小 = 大臂后仰
    "J3": [("ax", "ry", -1.0)],                       # 右摇杆 上下：推上 = 位置增大 = 小臂后折
    "J4": [("ax", "rx", +1.0)],                       # 右摇杆 左右：右推 = 位置增大 = 夹爪前仰
    "J5": [("ax", "dx", +0.75)],                       # 十字键 左右：右 = 位置减小 = 从腕看逆时针（帽键只有 0/±1，降 25% 更好微调）
    "J6": [("bt", "LB", +1.0), ("bt", "RB", -1.0)],   # LB=位置减小=张开 / RB=位置增大=闭合
}

# --- 幻尔总线协议常量 ---
SB_CMD_MOVE = 1
SB_CMD_MOVE_STOP = 12
SB_CMD_MOVE_START = 11
SB_CMD_POS_READ = 28

JS_FMT = "<IhBB"          # time(ms) | value(int16) | type | number

# ============================ 2. 帧编码 ============================


def car_frame(payload):
    '''$<payload>*<XOR 2 位大写 hex>\\r\\n，XOR 范围 = $ 与 * 之间的全部字节'''
    ck = 0
    for c in payload.encode("ascii"):
        ck ^= c
    return ("$" + payload + "*%02X\r\n" % ck).encode("ascii")


def servo_frame(sid, cmd, params=b""):
    '''55 55 | ID | LEN(=3+参数个数) | CMD | PARAM... | CHK(=~累加和)'''
    body = bytes([sid, 3 + len(params), cmd]) + params
    chk = (~sum(body)) & 0xFF
    return b"\x55\x55" + body + bytes([chk])


def parse_pos_reply(buf):
    '''在字节流里找「读位置」回包并校验：55 55 ID 05 1C posL posH CHK（LEN=5，总长=3+LEN）。
    累加和 = ~(ID+LEN+CMD+参数)。找不到合规帧就返回 None —— 不猜、不截尾凑数。'''
    i = 0
    while True:
        i = buf.find(b"\x55\x55", i)
        if i < 0 or len(buf) - i < 4:
            return None
        ln = buf[i + 3]
        total = 3 + ln
        if ln < 5 or len(buf) - i < total:      # LEN<5 是我们自己那条 6 字节请求/回声，跳过
            i += 1
            continue
        frame = buf[i:i + total]
        if ((~sum(frame[2:-1])) & 0xFF) != frame[-1]:
            i += 1
            continue
        if frame[4] != SB_CMD_POS_READ:
            i += 1
            continue
        pos = frame[5] | (frame[6] << 8)
        return pos if 0 <= pos <= 1200 else None


def f3(x):
    '''三位小数格式化；顺手把 -0.000 归一成 0.000'''
    if abs(x) < 0.0005:
        x = 0.0
    return "%.3f" % x


def curve(x, dz=STICK_DZ, expo=STICK_EXPO):
    '''死区 + 指数曲线：|x|<dz 归零，(dz→1) 段按 expo 拉伸；expo=1 就是线性'''
    s = -1.0 if x < 0 else 1.0
    a = abs(x)
    if a < dz:
        return 0.0
    a = (a - dz) / (1.0 - dz)
    return s * (a ** expo)


def clampf(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


# ============================ 3. 底盘串口 ============================


class CarLink:
    def __init__(self, port, dry=False):
        self.dry = dry
        self.ser = None
        self._buf = bytearray()
        self.last_err = ""
        self.last_vel = (0.0, 0.0, 0.0)
        if dry:
            return
        import serial  # 延迟导入：--dry-run 不需要 pyserial
        self.ser = serial.Serial(port, CAR_BAUD, timeout=0, write_timeout=0.05)

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass

    def _tx(self, payload):
        if self.ser is not None:
            self.ser.write(car_frame(payload))

    def drain(self, collect=None):
        '''非阻塞收应答，避免 pyserial 缓冲无限增长'''
        if self.ser is None:
            return
        n = self.ser.in_waiting
        if n:
            self._buf.extend(self.ser.read(n))
        text = self._buf.decode("ascii", "ignore")
        parts = text.split("\n")
        self._buf = bytearray(parts[-1].encode("ascii", "ignore"))
        for line in parts[:-1]:
            line = line.strip()
            if not line:
                continue
            if line.startswith("$ERR"):
                self.last_err = line
            if collect is not None:
                collect.append(line)

    def vel(self, vx, vy, wz):
        self.last_vel = (vx, vy, wz)
        self._tx("CAR,VEL,%s,%s,%s" % (f3(vx), f3(vy), f3(wz)))

    def stop(self):
        self.last_vel = (0.0, 0.0, 0.0)
        self._tx("CAR,STOP")

    def query(self, payload, want=None, timeout=0.25):
        '''发一条查询并等应答；等待期间补发 VEL，避免 500ms 看门狗把车停了。
        want 给了前缀就只认这类应答（防止把上一条 $OK 当查询结果）'''
        lines = []
        self._tx(payload)
        t0 = time.monotonic()
        ka = t0
        while time.monotonic() - t0 < timeout:
            self.drain(lines)
            if lines and (want is None or lines[-1].startswith(want)):
                break
            if time.monotonic() - ka > 0.10:
                self.vel(*self.last_vel)
                ka = time.monotonic()
            time.sleep(0.005)
        return lines


# ============================ 4. 总线舵机 ============================


class ServoArm:
    def __init__(self, port, joints, dry=False):
        self.dry = dry
        self.ser = None
        self.j = {}
        for jt in joints:
            d = dict(jt)
            d["target"] = float(jt["home"])   # 目标值（软件内积分，不等同于舵机实测）
            self.j[jt["name"]] = d
        self._dirty = {}
        # live=False 的关节本次运行完全不发帧（读回失败/位置越界时用，避免把未知姿态的臂拽向 HOME）
        self.live = {n: True for n in self.j}
        if dry:
            return
        import serial
        self.ser = serial.Serial(port, ARM_BAUD, timeout=0.02, write_timeout=0.05)

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass

    def _tx(self, sid, cmd, params=b""):
        if self.ser is not None:
            self.ser.write(servo_frame(sid, cmd, params))

    # ---------- 下发 ----------
    def flush(self):
        '''只发本拍变化过的关节，25Hz 下总线不会被打满'''
        for name in list(self._dirty):
            if self.live.get(name, False):
                j = self.j[name]
                self._tx(j["id"], SB_CMD_MOVE,
                         struct.pack("<HH", int(round(j["target"])), ARM_SEND_MS))
        self._dirty.clear()

    def hold_all(self):
        '''急停/退出：位置保持。注意 CMD12 之后必须 CMD11 才恢复接受 MOVE'''
        for name, j in self.j.items():
            if self.live.get(name, False):
                self._tx(j["id"], SB_CMD_MOVE_STOP)

    def resume_all(self):
        '''解除 CMD12 的暂停（不清目标、不重发，重发交给 force_all）'''
        for name, j in self.j.items():
            if self.live.get(name, False):
                self._tx(j["id"], SB_CMD_MOVE_START)

    # ---------- 速度积分：把「输入」变成「目标」，不超限、不跳变 ----------
    def step(self, ctrl, dt, scale):
        '''ctrl 已在 curve() 里过完死区（ARM_DZ/ARM_EXPO），这里不再重复判死区。
        target 若停在软限位之外（读回值越界），先按 spd 慢速收回限位，绝不一步跳进去。'''
        for name, j in self.j.items():
            if not self.live.get(name, False):
                continue
            v = ctrl.get(name, 0.0)
            lo, hi = float(j["lo"]), float(j["hi"])
            t = j["target"]
            step = j["spd"] * dt * scale
            if t < lo:
                nt = min(lo, t + step)
            elif t > hi:
                nt = max(hi, t - step)
            else:
                nt = t + j["dir"] * v * step
                nt = min(hi, max(lo, nt))
            if abs(nt - t) > 1e-9:
                j["target"] = nt
                # 目标连续积分；只有「取整后的舵机位置」变了才发帧 —— 小推力也能累积成一格
                if int(round(nt)) != int(round(t)):
                    self._dirty[name] = True

    def home_step(self, dt, scale=1.0):
        '''一键回 HOME：每拍向 home 靠，全部到位返回 True（home 必在 [lo,hi] 内，不会越界）'''
        done = True
        for name, j in self.j.items():
            if not self.live.get(name, False):
                continue
            gap = j["home"] - j["target"]
            if abs(gap) < 1.0:
                continue
            done = False
            step = j["spd"] * dt * scale
            nt = j["target"] + (step if gap > 0 else -step)
            if abs(j["home"] - nt) < step:      # 别冲过头
                nt = float(j["home"])
            j["target"] = nt
            self._dirty[name] = True
        return done

    # ---------- 尽力读回位置（回包格式按幻尔手册，PC 验收工具已实测；失败返回 None，绝不猜） ----------
    def read_pos(self, name, timeout=0.03):
        '''读位置（CMD 28）。回包 LEN=5，位置在第 6、7 字节，最后 1 字节是校验和。
        有的转接板会把我们发出去的请求原样回声，解析器只认「CMD28 且 LEN>=5」的合规帧。'''
        if self.ser is None:
            return None
        j = self.j[name]
        try:
            self.ser.reset_input_buffer()       # 丢掉上一拍残留/回声
        except Exception:
            pass
        try:
            self._tx(j["id"], SB_CMD_POS_READ)
            buf = bytearray()
            t0 = time.monotonic()
            while time.monotonic() - t0 < timeout:
                n = self.ser.in_waiting
                if n:
                    buf.extend(self.ser.read(n))
                    pos = parse_pos_reply(buf)
                    if pos is not None:
                        return pos
                else:
                    time.sleep(0.002)
        except Exception:                        # 串口被拔/写超时：当读回失败处理，不炸主循环
            return None
        return None

    def adopt(self, name, pos):
        '''把读回值当起点，避免上电第一帧就跳'''
        self.j[name]["target"] = float(pos)

    def force_all(self):
        for name in self.j:
            if self.live.get(name, False):
                self._dirty[name] = True


# ============================ 5. 手柄原始 js 解析 ============================


JS_NAME_HINTS = ("xbox", "x-box", "xinput", "microsoft", "gamesir", "game sir", "controller")


def js_node_list():
    '''/dev/input/js* 按数字顺序排好（js2 排在 js10 前面）'''
    d = "/dev/input"
    if not os.path.isdir(d):
        return []
    names = [n for n in os.listdir(d) if n.startswith("js")]
    names.sort(key=lambda n: int("".join(c for c in n if c.isdigit()) or 0))
    return [os.path.join(d, n) for n in names]


def js_name(path):
    '''取设备名。优先读 sysfs（不需要权限），退回 joydev 的 JSIOCGNAME；失败返回空串。'''
    node = os.path.basename(path)                 # js0
    try:
        with open("/sys/class/input/%s/device/name" % node, "r") as f:
            return f.read().strip()
    except OSError:
        pass
    try:
        import fcntl
    except ImportError:
        return ""
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    except OSError:
        return ""
    try:
        buf = bytearray(128)
        fcntl.ioctl(fd, 0x80806A13, buf)          # JSIOCGNAME(128) = _IOC(READ,'j',0x13,128)
        return bytes(buf).split(b"\x00")[0].decode("utf-8", "ignore").strip()
    except Exception:
        return ""
    finally:
        os.close(fd)


def find_pad():
    '''自动挑手柄：先按设备名认（Xbox/GameSir/XInput/Controller），认不出就取编号最小的 js。
    2.4G 接收器和 Type-C 有线会各建一个节点，编号随插拔变化，所以不要写死 js0。'''
    cands = []
    for p in js_node_list():
        n = js_name(p)
        cands.append((p, n))
        if any(h in n.lower() for h in JS_NAME_HINTS):
            return p, n
    if cands:
        return cands[0]
    return None, ""


class JsPad:
    def __init__(self, path, axes, btns):
        self.path = path
        self.axes = dict(axes)
        self.btns = dict(btns)
        self.rev_ax = {v[0]: (k, v[1]) for k, v in self.axes.items()}
        self.rev_btn = {v: k for k, v in self.btns.items()}
        self.val = {k: 0.0 for k in self.axes}
        self.btn = {k: False for k in self.btns}
        self.edges = []             # 本拍新按下的键名
        self.events = []            # 本拍全部原始事件 (类型, 编号, 值)，认键用
        self.gone = False
        self._trig = {}              # 扳机零点自适应：{"lt": {"zero": 首帧值, "full": 满量程}}
        self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        self.naxes = 0
        try:
            import fcntl
            _b = bytearray(1)
            fcntl.ioctl(self.fd, 0x80016A11, _b)   # JSIOCGAXES
            self.naxes = _b[0]
        except Exception:
            pass

    def close(self):
        try:
            os.close(self.fd)
        except Exception:
            pass

    def _norm(self, name, kind, v):
        if kind == "s":
            return max(-1.0, min(1.0, v / 32767.0))
        # joydev 通常按 absinfo 把扳机归一成 int16，常见三种零点（兼容性分支，都收）：
        #   ① 松手 ≈ -32767（双极）→ 到底 +32767
        #   ② 松手 ≈ 0（单极）    → 到底 +32767
        #   ③ 少数驱动不归一化/反相：松手在高位（如 255）→ 到底 0
        st = self._trig.setdefault(name, {"zero": v})
        z = st["zero"]
        if z <= -1000:                          # ①
            return clampf((v - z) / (32767.0 - z), 0.0, 1.0)
        if z >= 200:                            # ③
            return clampf((z - v) / float(z), 0.0, 1.0)
        return clampf(v / 32767.0, 0.0, 1.0)    # ②

    def poll(self):
        '''读光当前所有事件'''
        if self.gone:
            return
        if not os.path.exists(self.path):     # 2.4G 掉线/接收器被拔：节点会消失
            self.gone = True
            return
        self.edges = []
        self.events = []
        while True:
            try:
                data = os.read(self.fd, 8 * 64)
            except BlockingIOError:
                break
            except OSError:
                self.gone = True
                break
            if not data:
                break
            for off in range(0, len(data) - 7, 8):
                _t, v, typ, num = struct.unpack_from(JS_FMT, data, off)
                init = bool(typ & 0x80)                  # 打开设备时的状态快照，不是用户动作
                typ &= 0x7F
                if typ not in (0x01, 0x02):
                    continue
                self.events.append((typ, num, v))
                if typ == 0x02:                          # 轴
                    item = self.rev_ax.get(num)
                    if item is None:
                        continue
                    name, kind = item
                    self.val[name] = self._norm(name, kind, v)
                else:                                    # 键
                    name = self.rev_btn.get(num)
                    if name is None:
                        continue
                    pressed = v > 0
                    if pressed and not self.btn[name] and not init:
                        self.edges.append(name)
                    self.btn[name] = pressed


# ============================ 6. 设备列举 / 认键 ============================


def list_devices():
    print("\n== /dev/input（手柄在这里） ==")
    if os.path.isdir("/dev/input"):
        for p in js_node_list():
            print("  %s   %s" % (p, js_name(p) or "(读不到设备名，可能需要 sudo)"))
    else:
        print("  (没有 /dev/input，手柄没插或内核没载入 joydev)")
    print("\n== /dev/serial/by-id（CH341 / 舵机转接板在这里） ==")
    d = "/dev/serial/by-id"
    if os.path.isdir(d):
        for n in sorted(os.listdir(d)):
            print("  %s/%s  ->  %s" % (d, n, os.path.realpath(os.path.join(d, n))))
    else:
        print("  (无 by-id 目录，用 /dev/ttyUSB0 … 兜底)")
    print("\n== 常见串口 ==")
    if os.path.isdir("/dev"):
        for n in sorted(os.listdir("/dev")):
            if n.startswith("ttyUSB") or n.startswith("ttyACM"):
                print("  /dev/" + n)


def probe(pad):
    '''现场认键：报的是原始编号，不管配置表认不认识都能看到'''
    print("认键模式（30 秒无操作自动结束，Ctrl-C 提前退）")
    print("逐个推摇杆、按按键；下面出现的编号就是要填进 AXES / BTNS 的值")
    print("启动时别碰扳机（首帧要采零点，按着启动会让油门零点偏掉）")
    print("当前配置：AXES =", {k: v[0] for k, v in pad.axes.items()},
          " BTNS =", pad.btns)
    t_last = time.monotonic()
    while time.monotonic() - t_last < 30:
        if not select.select([pad.fd], [], [], 0.2)[0]:
            if pad.gone:
                break
            continue
        pad.poll()
        if pad.gone:
            print("  手柄掉线了（节点消失/读失败）")
            break
        for typ, num, v in pad.events:
            t_last = time.monotonic()
            if typ == 0x02:
                name = pad.rev_ax.get(num, ("?", "s"))[0]
                print("  轴 %-2d  raw=%+7d  norm=%+.2f   %s"
                      % (num, v, pad.val.get(name, 0.0), ("← 已映射为 " + name) if name != "?" else ""))
            else:
                name = pad.rev_btn.get(num)
                print("  键 %-2d  %s   %s" % (num, "按下" if v else "松开",
                                              ("← 已映射为 " + name) if name else ""))
    print("认键结束。")


BANNER = '''
================ 键位（盖世小鸡启明星2 · XBOX 布局） =================
  手柄：2.4G 接收器 / Type-C 有线 都走 /dev/input/jsN；不给 --js 就自动挑（认名字，不写死 js0）
  两个模式：M1 背键（或 BACK）切换；切换瞬间底盘自动刹停
 【驾驶 DRIVE】
   左摇杆        前进/后退 + 左右横移（麦轮全向）
   右摇杆 X      原地自转
   LT（模拟）    油门+使能：松开=停，压得越深越快
   RT            精细档（速度减半）
 【机械臂 ARM】底盘锁定不动
   左摇杆 X / Y  J1 底座回转 / J2 大臂抬降
   右摇杆 X / Y  J4 腕俯仰 / J3 肘伸缩
   十字键 ←/→    J5 腕滚转
   LB / RB       J6 夹爪 张开 / 闭合
   LT / RT       臂速 加速（×1→2）/ 减速（×0.5）
 【通用】
   A 急停（再按解除，须先松 LT）   B 一键回 HOME
   START 打印状态
======================================================================
'''


# ============================ 7. 自检 / 主流程 ============================


def validate_config():
    '''启动即查配置：绑定冲突、越限、ID 重复、模式键与关节抢输入。
    返回问题列表（空 = 通过）。'''
    bad = []
    names = [j["name"] for j in JOINTS]
    ids = [j["id"] for j in JOINTS]

    if len(set(names)) != len(names):
        bad.append("JOINTS 里有重复的关节名")
    if len(set(ids)) != len(ids):
        bad.append("JOINTS 里有重复的舵机 ID")
    for j in JOINTS:
        if not (0 <= j["lo"] <= j["hi"] <= 1000):
            bad.append("%s 限位非法：lo=%s hi=%s" % (j["name"], j["lo"], j["hi"]))
        if not (j["lo"] <= j["home"] <= j["hi"]):
            bad.append("%s 的 home=%s 落在限位之外" % (j["name"], j["home"]))
        if j["spd"] <= 0:
            bad.append("%s 的 spd 必须 >0" % j["name"])
        if not (1 <= j["id"] <= 253):
            bad.append("%s 的舵机 ID=%s 非法（1~253；254 是广播地址）" % (j["name"], j["id"]))
        if j["dir"] not in (-1, 1):
            bad.append("%s 的 dir=%s 只能是 +1 或 -1（0 会让该轴永远不动）" % (j["name"], j["dir"]))

    used = {}
    for joint, binds in ARM_MAP.items():
        if joint not in names:
            bad.append("ARM_MAP 绑定了不存在的关节 %s" % joint)
        for kind, key, gain in binds:
            if kind not in ("ax", "bt"):
                bad.append("%s 的绑定类型 %r 只能是 'ax' 或 'bt'" % (joint, kind))
                continue
            src = AXES if kind == "ax" else BTNS
            if key not in src:
                bad.append("%s 绑定的输入 %s 不在 %s 表里" % (joint, key, "AXES" if kind == "ax" else "BTNS"))
            if gain == 0:
                bad.append("%s 绑定的 %s 增益为 0（该输入永远不动）" % (joint, key))
            if key in used:
                bad.append("输入 %s 同时绑给了 %s 和 %s（冲突）" % (key, used[key], joint))
            used[key] = joint
            if key in ("lt", "rt"):
                bad.append("%s 用了扳机 %s：扳机在本方案里是油门/精调，不绑关节" % (joint, key))
            if kind == "bt" and key in RESERVED_BTNS:
                bad.append("%s 占用了保留键 %s（A/B/START 是急停/回零/状态）" % (joint, key))

    for label, nums in (("AXES", [v[0] for v in AXES.values()]), ("BTNS", list(BTNS.values()))):
        dup = sorted({n for n in nums if nums.count(n) > 1})
        if dup:
            bad.append("%s 编号重复：%s（同一物理通道被绑了两次，--axis/--btn 覆盖后尤其要查）"
                       % (label, dup))

    for n in names:
        if n not in ARM_MAP:
            bad.append("关节 %s 没有绑定输入" % n)
    for b in MODE_SWITCH_BTNS:
        if b not in BTNS:
            bad.append("模式键 %s 不在 BTNS 表里" % b)
        if b in used:
            bad.append("模式键 %s 与关节 %s 抢同一个输入" % (b, used[b]))
        if b in RESERVED_BTNS:
            bad.append("模式键 %s 与保留键冲突" % b)
    return bad


def selftest():
    '''离线自检：配置 + 两种帧 + 回包解析 + 关节限位 + 回 HOME 收敛。不需要板子、不需要手柄。'''
    ok = True
    bad = validate_config()
    if bad:
        ok = False
        print("[配置] %d 处问题：" % len(bad))
        for b in bad:
            print("   -", b)
    else:
        print("[配置] 通过：%d 关节、%d 条绑定，无重复占用" % (len(JOINTS), sum(len(v) for v in ARM_MAP.values())))

    ref = [("CAR,STOP", "$CAR,STOP*64"), ("PING", "$PING*10")]
    for payload, want in ref:
        got = car_frame(payload).decode()
        good = got == want + "\r\n"
        ok = ok and good
        print("[底盘帧] %-10s -> %-16r %s" % (payload, got, "OK" if good else "FAIL（应为 %s）" % want))

    f = servo_frame(1, SB_CMD_MOVE, struct.pack("<HH", 500, ARM_SEND_MS))
    good = (len(f) == 10 and f[:4] == b"\x55\x55\x01\x07" and f[-1] == ((~sum(f[2:-1])) & 0xFF))
    ok = ok and good
    print("[舵机帧] J1→500/80ms = %s  %s" % (f.hex(" "), "OK" if good else "FAIL"))

    reply = b"\x55\x55\x01\x05\x1c\xf4\x01" + bytes([(~(1 + 5 + 0x1C + 0xF4 + 1)) & 0xFF])
    echo = servo_frame(1, SB_CMD_POS_READ)          # 转接板回声：6 字节请求帧
    good = (parse_pos_reply(reply) == 500 and parse_pos_reply(echo) is None
            and parse_pos_reply(reply[:-1] + b"\x00") is None)
    ok = ok and good
    print("[回包解析] 500→%s / 请求回声→%s / 坏校验→%s  %s"
          % (parse_pos_reply(reply), parse_pos_reply(echo), parse_pos_reply(reply[:-1] + b"\x00"),
             "OK" if good else "FAIL"))

    arm = ServoArm(None, JOINTS, dry=True)
    for _ in range(300):
        arm.step({"J1": 1, "J2": 1, "J3": 1, "J4": 1, "J5": 1, "J6": 1}, 0.04, 2.0)
    over = [n for n, j in arm.j.items() if not (j["lo"] <= j["target"] <= j["hi"])]
    ok = ok and not over
    print("[限位] 满输入 12s 后越限关节：%s" % (over or "无"))

    arm2 = ServoArm(None, JOINTS, dry=True)          # 回 HOME 收敛：从离 home 最远的那端出发
    for j in arm2.j.values():
        j["target"] = float(j["hi"] if (j["hi"] - j["home"]) > (j["home"] - j["lo"]) else j["lo"])
    ticks = 0
    while not arm2.home_step(0.04, 1.0) and ticks < 2000:
        ticks += 1
    ticks += 1
    done = all(abs(j["target"] - j["home"]) <= 1.0 for j in arm2.j.values())
    good = done and ticks < 2000
    ok = ok and good
    print("[HOME] 从最远端回位 %d 拍（%.1fs）%s" % (ticks, ticks * 0.04, "OK" if good else "FAIL"))

    print("自检%s" % ("通过" if ok else "失败"))
    return 0 if ok else 1



def run(args):
    # ---------- 配置自检（绑定冲突/越限）----------
    bad = validate_config()
    if bad:
        print("[配置] 发现 %d 处冲突/错误，先改配置再跑：" % len(bad))
        for b in bad:
            print("   -", b)
        return

    # ---------- 设备 ----------
    try:
        pad = JsPad(args.js, AXES, BTNS)
    except OSError as e:
        print("[!] 打不开手柄 %s：%s" % (args.js, e))
        print("    先跑 --list 看节点（Linux 上通常是 /dev/input/js0…js9）")
        return
    if pad.naxes and pad.naxes < 8:
        print("[手柄] 这台只上报 %d 根轴：十字键可能是按键而不是轴 6/7 —— 先 --probe 确认，"
              "若是按键就把 J5 改绑到 X/Y（见 README 第 8 节）" % pad.naxes)
    try:
        car = CarLink(args.car_port, dry=args.dry_run)
    except Exception as e:
        print("[!] 打不开底盘串口 %s：%s" % (args.car_port, e))
        print("    确认 CH341 插着、Flask/其它脚本没占着同一个口")
        pad.close()
        return
    arm = None
    if args.arm_port:
        try:
            arm = ServoArm(args.arm_port, JOINTS, dry=args.dry_run)
            arm.resume_all()                 # 上一次可能以 CMD12 收尾，先解除暂停（此时还不能发 MOVE）
        except Exception as e:
            print("[!] 打不开舵机转接板 %s：%s → 本次只控底盘" % (args.arm_port, e))
            arm = None

    if arm is not None and not args.dry_run:   # dry-run 不读舵机，直接进主循环验按键
        got = 0
        dead = []
        parked = []
        for jt in JOINTS:
            p = arm.read_pos(jt["name"])
            if p is None:
                dead.append(jt["name"])
                continue
            got += 1
            if p > 1000:                          # 停在越程段（1000~1200）：不能作为合法目标
                arm.live[jt["name"]] = False
                parked.append((jt["name"], p))
                continue
            arm.adopt(jt["name"], p)              # 以实测位置为起点：上电第一帧不跳
            if not (jt["lo"] <= p <= jt["hi"]):
                print("[臂] %s 实测 %d 在软限位 %d~%d 之外：本次动作会先按 spd 慢速收回限位，"
                      "确认结构安全后按第 7 节改 lo/hi" % (jt["name"], p, jt["lo"], jt["hi"]))
        for name, p in parked:
            print("[臂] %s 实测 %d 停在越程段(>1000)：本次不发帧，先用官方工具把它移回 0~1000 再重启"
                  % (name, p))
        if dead and got:
            for name in dead:
                arm.live[name] = False            # 关键：读回失败的轴一律不发帧，绝不按 HOME 硬拽
            live_n = sum(1 for v in arm.live.values() if v)
            print("[臂] 未读回：%s → 本次禁用这些轴（其余 %d 轴照常）" % (" ".join(dead), live_n))
        if got == 0:
            if args.arm_home:
                print("[臂] 读回失败，按 --arm-home 以 HOME 为起点（姿态不符请立刻 A 急停）")
            else:
                print("[臂] 读回全部失败 → 自我保护：本次只控底盘。")
                print("     确认六轴当前确实在 HOME 姿态后，加 --arm-home 重跑。")
                arm.close()
                arm = None
        elif not any(arm.live.values()):
            print("[臂] 六轴全部不可用（越程段/未读回）→ 本次只控底盘")
            arm.close()
            arm = None
        else:
            print("[臂] 读回成功 %d/6，起点已对齐实测位置" % got)
        if arm is not None:                  # 起点定了才允许下发，避免上电就甩向 HOME
            arm.force_all()
            arm.flush()

    print(BANNER)
    if args.dry_run:
        print("*** DRY-RUN：只打印帧，不写串口 ***")

    mode = "DRIVE"
    estop = False
    homing = False
    mode_guard = 0.0                  # 切到 DRIVE 后 0.3s 不接受油门，等车真正停稳/松杆
    tick_no = 0
    last_edge = {}                      # 防连击：同一个键 250ms 内只认一次
    next_tick = time.monotonic()

    def all_stop():
        car.stop()
        if arm is not None:
            arm.hold_all()

    def status():
        lines = car.query("CAR,ODOM", want="$ODOM")
        err = car.last_err                 # 取一次就清，别把上一条旧错误永远打出来
        car.last_err = ""
        print("[状态] 里程:", (lines[-1] if lines else "(无应答)"), err)
        if arm is not None:
            car.vel(*car.last_vel)           # 读舵机要花 0.2~0.4s，先喂一次底盘看门狗
            for i, (name, j) in enumerate(arm.j.items(), 1):
                if arm.live.get(name, False):
                    p = arm.read_pos(name)
                    shown = "-" if p is None else p
                else:
                    shown = "禁用"
                print("   %s id=%d 目标=%4d 限位=%d~%d 读回=%s"
                      % (name, j["id"], int(round(j["target"])), j["lo"], j["hi"], shown))
                if i % 2 == 0:
                    car.vel(*car.last_vel)   # 每两轴再喂一次，别让 6×30ms 攒到 500ms

    def switch_mode():
        nonlocal mode, mode_guard
        car.stop()                        # 切换瞬间真刹停，并复位 last_vel（下一拍照常输出）
        if mode == "DRIVE":
            mode = "ARM"
            print("[模式] 机械臂（底盘锁定）｜左摇杆 X/Y=J1/J2 · 右摇杆 X/Y=J4/J3 · 十字 ←/→=J5 · LB/RB=夹爪")
        else:
            mode = "DRIVE"
            mode_guard = time.monotonic() + 0.3
            print("[模式] 驾驶｜LT 油门 · 左摇杆平移 · 右摇杆 X 自转 · RT 精细（0.3s 后才接受油门）")

    def on_signal(_sig, _frm):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, on_signal)

    try:
        while True:
            pad.poll()
            if pad.gone:
                print("[!] 手柄设备消失（拔了/掉线）→ 停车退出")
                break

            # ---------- 按键边沿（同一个键 250ms 内只认一次，防背键宏连发）----------
            now_s = time.monotonic()
            for key in pad.edges:
                if now_s - last_edge.get(key, 0.0) < 0.25:
                    continue
                last_edge[key] = now_s
                if key == "A":
                    estop = not estop
                    if estop:
                        homing = False           # 急停优先，取消正在进行的回 HOME
                        all_stop()
                        print("[急停] 底盘已刹停、六轴保持位置（再按 A 解除）")
                    elif pad.val["lt"] > 0.05:
                        estop = True
                        print("[急停] 解除被拒：先松开 LT")
                    else:
                        if arm is not None:
                            arm.resume_all()
                            arm.force_all()      # 暂停期间目标没变，重发一遍保证舵机继续认 MOVE
                        print("[急停] 已解除")
                elif key == "B":
                    if not estop and arm is not None:
                        homing = True
                        print("[HOME] 六轴回零位（J6 闭合到 674），推任一摇杆或按 LB/RB 即中止")
                elif key == "START":
                    status()
                elif key in MODE_SWITCH_BTNS:
                    if not estop:
                        switch_mode()

            # ---------- 输入整形 ----------
            lx = curve(pad.val["lx"])
            ly = curve(pad.val["ly"])
            rx = curve(pad.val["rx"])
            ry = curve(pad.val["ry"])
            lt = curve(pad.val["lt"], TRIG_DZ, TRIG_EXPO)
            # RT 是模拟轴（不是按键）：压得越深越慢，1.0 → 0.5 连续过渡
            fine = 1.0 - (1.0 - FINE_SCALE) * curve(pad.val.get("rt", 0.0), TRIG_DZ, 1.0)

            moving = any(abs(v) > 0.05 for v in (lx, ly, rx, ry, pad.val.get("dx", 0.0), pad.val.get("dy", 0.0))) \
                or pad.btn.get("LB") or pad.btn.get("RB")
            if homing and moving:
                homing = False
                print("[HOME] 被手动输入中止")

            if estop:
                car.stop()
                car.drain()
                next_tick += TICK_S
                gap = next_tick - time.monotonic()
                if gap > 0:
                    time.sleep(gap)
                else:
                    next_tick = time.monotonic()      # 和主循环同一套调度，落后就重对齐
                continue

            # ---------- 底盘 ----------
            if mode == "DRIVE":
                thr = (lt * fine) if now_s >= mode_guard else 0.0
                vx = clampf(-ly * VX_MAX * thr, -VX_MAX, VX_MAX)
                vy = clampf(lx * VY_MAX * thr, -VY_MAX, VY_MAX)
                wz = clampf(-rx * WZ_MAX * thr, -WZ_MAX, WZ_MAX)
                car.vel(vx, vy, wz)
            else:
                vx = vy = wz = 0.0
                car.vel(0.0, 0.0, 0.0)       # 臂模式底盘锁停，同时喂看门狗
            car.drain()

            # ---------- 机械臂 ----------
            if arm is not None:
                if homing:
                    if arm.home_step(TICK_S, fine):
                        homing = False
                        off = [n for n, live in arm.live.items() if not live]
                        print("[HOME] 已到位" + ("（禁用轴不参与：%s）" % " ".join(off) if off else ""))
                elif mode == "ARM":
                    arm_scale = (1.0 + lt * (BOOST_MAX - 1.0)) * fine   # 1.0~2.0，再叠精细档
                    ctrl = {}
                    for name, binds in ARM_MAP.items():
                        v = 0.0
                        for kind, key, gain in binds:
                            if kind == "ax":
                                v += gain * curve(pad.val.get(key, 0.0), ARM_DZ, ARM_EXPO)
                            else:
                                v += gain * (1.0 if pad.btn.get(key) else 0.0)
                        ctrl[name] = clampf(v, -1.0, 1.0)
                    arm.step(ctrl, TICK_S, arm_scale)
                arm.flush()

            if args.dry_run and tick_no % 5 == 0:      # 5Hz 打印，别刷屏
                print("  [%s] VEL %s %s %s  lt=%.2f fine=%.1f"
                      % (mode, f3(vx), f3(vy), f3(wz), lt, fine))
            tick_no += 1

            next_tick += TICK_S
            gap = next_tick - time.monotonic()
            if gap > 0:
                time.sleep(gap)
            else:
                next_tick = time.monotonic()     # 落后就重对齐，不追帧

    except KeyboardInterrupt:
        print("\n[退出] Ctrl-C")
    finally:
        try:
            all_stop()
        except Exception:
            pass
        time.sleep(0.05)
        pad.close()
        car.close()
        if arm is not None:
            arm.close()
        print("[退出] 已发 $CAR,STOP 与舵机 MOVE_STOP，串口已释放")


def main():
    ap = argparse.ArgumentParser(description="手柄遥控 麦轮底盘 + 六轴总线舵机臂")
    ap.add_argument("--js", default="", help="手柄节点；不给=自动挑 /dev/input/js* 里的手柄")
    ap.add_argument("--car-port", default=CAR_PORT_DEFAULT, help="底盘 CH341 串口")
    ap.add_argument("--arm-port", default=None, help="总线舵机转接板串口；不给=只控底盘")
    ap.add_argument("--arm-home", action="store_true", help="读回失败时，允许以配置 HOME 为起点")
    ap.add_argument("--dry-run", action="store_true", help="只打印速度摘要，不写串口")
    ap.add_argument("--probe", action="store_true", help="认键模式：打印原始轴/键编号")
    ap.add_argument("--list", action="store_true", help="列出手柄节点与串口")
    ap.add_argument("--selftest", action="store_true", help="离线自检：配置/帧/回包/限位/回零，不碰硬件")
    ap.add_argument("--axis", default="", help="覆盖轴编号，如 lx:0,ly:1,lt:2,rx:3,ry:4,rt:5")
    ap.add_argument("--btn", default="", help="覆盖按键编号，如 m1:10,m2:11")
    args = ap.parse_args()

    if args.axis:
        for item in args.axis.split(","):
            item = item.strip()
            if not item:
                continue
            if ":" not in item:
                raise SystemExit("--axis 格式错：%r，应为 lx:0,ly:1,..." % item)
            k, v = item.split(":", 1)
            k = k.strip().lower()
            if k not in AXES:
                raise SystemExit("--axis 里有未知轴名：%r（可选：%s）" % (k, ",".join(AXES)))
            try:
                AXES[k] = (int(v), AXES[k][1])
            except ValueError:
                raise SystemExit("--axis 的编号不是整数：%r" % item)
    if args.btn:
        for item in args.btn.split(","):
            item = item.strip()
            if not item:
                continue
            if ":" not in item:
                raise SystemExit("--btn 格式错：%r，应为 m1:10,m2:11" % item)
            k, v = item.split(":", 1)
            k = k.strip().upper()
            if k not in BTNS:
                raise SystemExit("--btn 里有未知键名：%r（可选：%s）" % (k, ",".join(BTNS)))
            try:
                BTNS[k] = int(v)
            except ValueError:
                raise SystemExit("--btn 的编号不是整数：%r" % item)

    if args.selftest:
        raise SystemExit(selftest())

    if args.list:
        list_devices()
        return

    if not args.js:                     # 自动挑手柄：2.4G 接收器/有线插拔后 js 编号会变
        p, name = find_pad()
        if not p:
            print("[!] 没找到手柄：/dev/input/js* 是空的。")
            print("    2.4G 要插接收器（另购的那颗 dongle）、有线要插 Type-C 数据线；"
                  "用 --list 看设备，或 --js 指定节点。")
            raise SystemExit(1)
        args.js = p
        print("[手柄] %s  %s" % (p, name or "(设备无名字)"))

    if args.probe:
        try:
            pad = JsPad(args.js, AXES, BTNS)
        except OSError as e:
            print("[!] 打不开手柄 %s：%s（先跑 --list 看节点）" % (args.js, e))
            raise SystemExit(1)
        try:
            probe(pad)
        finally:
            pad.close()
        return

    run(args)


if __name__ == "__main__":
    main()
