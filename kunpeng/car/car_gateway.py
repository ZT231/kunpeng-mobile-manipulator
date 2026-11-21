#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================================
car_gateway.py —— 鲲鹏 / PC 侧「底盘统一帧」发送网关
============================================================================

【大白话】机械臂那边已经有一套说话方式：`$JOINT,...*XX`（XX 是校验和）。
底盘现在也用同一套，只是开头换成 `$CAR,...`。这个脚本就是把人话
（"前进 800 毫秒"）翻译成那一串带校验和的帧，从串口或网络发出去，
再把车回的 `$OK*04` / `$ERR,3*5A` 打印出来。

【规范定义】帧格式遵循项目内既有的 `Core/Src/uart_protocol.c`：
    $<CMD>[,<ARG>...]*<CK><CR><LF>
CK = '$' 与 '*' 之间所有字节逐字节异或（XOR），2 位大写十六进制。
这种「带帧头 + 校验 + 定界符」的 ASCII 帧也叫 NMEA-like 帧，
好处是可打印、好抓包、出错能定位到字节。

【为什么不改 web_control.py】
机械臂那套 Flask 服务已经实机验收通过，改它要重启、有回归风险。
底盘网关**独立一个进程**，谁也不碰谁；将来要挂 `/api/car` 只需
import 本文件的 build_frame()，一行就能接上。这也符合
「先做不依赖 ESP32 硬件的一段」这条既定落地顺序。

【依赖】只用 Python 标准库。串口功能需要 pyserial（可选）：
    pip install pyserial
没装也能用 --tcp，或者用 --dry-run 只打印帧，先在电脑上验校验和。

用法示例：
    # 只算帧、不发出去（电脑上先验）
    python3 car_gateway.py --dry-run --cmd FWD --ms 800
    python3 car_gateway.py --dry-run --raw '$CAR,STOP'

    # 经 USB-TTL 串口发（Linux 板子上 / Windows 上 COMx）
    python3 car_gateway.py --port /dev/ttyUSB1 --cmd FWD --ms 800
    python3 car_gateway.py --port COM7 --cmd STOP

    # 经网络透传（ESP32 / WiFi 模块做串口服务器时）
    python3 car_gateway.py --tcp 192.168.77.100:9000 --cmd BWD --ms 300

    # 带速度参数：$CAR,FWD,800,900
    python3 car_gateway.py --port COM7 --cmd FWD --ms 800 --speed 900
"""

import argparse
import sys
import time

# ---------------------------------------------------------------------------
# 第一步：帧的组装与校验
# ---------------------------------------------------------------------------

FRAME_MAX_LEN = 64          # 与 car_frame.h 里的 CAR_FRAME_MAX_LEN 保持一致
MAX_RUN_MS = 5000           # 与固件侧仲裁上限一致：上位机也别发离谱的值


def xor_ck(payload):
    """算 XOR 校验和，返回 2 位大写十六进制字符串，如 '3D'。

    大白话：把这句话里每个字符的 ASCII 码挨个「异或」一遍，
    得到的一个字节就是暗号。收的一方用同样算法再算一遍，
    对不上就说明这帧在线上被干扰了 / 波特率不对。
    """
    ck = 0
    for ch in payload:
        ck ^= ord(ch)
    return "%02X" % ck


def build_frame(payload):
    """把 '$' 与 '*' 之间的内容补成一整帧：$...*CK\\r\\n。

    注意 payload 里不要自己带 '$' 或 '*'，本函数只负责外面这层壳。
    """
    if not payload:
        raise ValueError("payload 不能为空")
    if len(payload) > FRAME_MAX_LEN:
        raise ValueError("帧太长（%d > %d），固件会回 ERR,1" % (len(payload), FRAME_MAX_LEN))
    return "$%s*%s\r\n" % (payload, xor_ck(payload))


def build_car_frame(action, ms=None, speed=None):
    """把「动作 + 时长 + 速度」拼成 $CAR,... 的 payload。

    时长 ms 是**开环定时**：车跑这么久就停，不是"走多少厘米"。
    这是硬件缺口（无编码器/里程计）导致的边界，别在简历里写成闭环。
    """
    action = action.upper()
    if action == "STOP":
        return build_frame("CAR,STOP")
    if ms is None:
        raise ValueError("%s 需要 --ms（毫秒）；只有 STOP 可以不给" % action)
    if not (0 <= int(ms) <= MAX_RUN_MS):
        raise ValueError("--ms 超出量程 0~%d（固件会回 ERR,3）" % MAX_RUN_MS)
    payload = "CAR,%s,%d" % (action, int(ms))
    if speed is not None:
        if not (0 <= int(speed) <= 999):
            raise ValueError("--speed 超出 PWM 量程 0~999")
        payload += ",%d" % int(speed)
    return build_frame(payload)


# ---------------------------------------------------------------------------
# 第二步：两种物理通道（串口 / TCP），都用同一套「发帧 + 收回包」
# ---------------------------------------------------------------------------

def _read_reply(reader, timeout_s):
    """等一小会儿，把对方回的一行读出来。

    固件回的是 `$OK*04\\r\\n` 这种以换行结尾的短帧，
    所以按「读一行」处理就够，不用做复杂的流式解析。
    """
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        line = reader()
        if line:
            return line
        time.sleep(0.02)
    return None


def send_serial(frame, port, baud, timeout_s):
    """经 USB-TTL / HC-05 透传串口发帧。"""
    try:
        import serial  # pyserial，可选依赖
    except ImportError:
        print("没有 pyserial：先 pip install pyserial，或改用 --tcp / --dry-run")
        return 2

    # with 会在出错时也把串口关掉，避免下次打开报「端口被占用」
    with serial.Serial(port, baud, timeout=0.1) as ser:
        ser.write(frame.encode("ascii"))
        ser.flush()

        def read_one():
            data = ser.readline()
            return data.decode("ascii", errors="replace").strip() if data else None

        reply = _read_reply(read_one, timeout_s)
    print("串口 %s @ %d 已发：%r" % (port, baud, frame))
    print("回包：%s" % (reply if reply else "（超时没收到，检查接线/波特率/共地）"))
    return 0


def send_tcp(frame, addr, timeout_s):
    """经网络透传发帧（ESP32 / WiFi 模块当串口服务器时用）。"""
    import socket

    host, sep, port = addr.rpartition(":")
    if not sep:
        raise ValueError("--tcp 要写成 host:port，例如 192.168.77.100:9000")
    with socket.create_connection((host, int(port)), timeout=timeout_s) as sock:
        sock.settimeout(0.1)
        sock.sendall(frame.encode("ascii"))

        def read_one():
            try:
                data = sock.recv(128)
            except socket.timeout:
                return None
            return data.decode("ascii", errors="replace").strip() if data else None

        reply = _read_reply(read_one, timeout_s)
    print("TCP %s 已发：%r" % (addr, frame))
    print("回包：%s" % (reply if reply else "（超时没收到）"))
    return 0


# ---------------------------------------------------------------------------
# 第三步：命令行入口
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="底盘统一帧网关（$CAR,...*CK），帧格式与机械臂固件一致")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="串口，如 COM7 或 /dev/ttyUSB1")
    src.add_argument("--tcp", help="TCP 地址 host:port，如 192.168.77.100:9000")
    src.add_argument("--dry-run", action="store_true", help="只打印帧，不真的发出去")

    cmd = ap.add_mutually_exclusive_group(required=True)
    cmd.add_argument("--cmd", help="动作：FWD/BWD/SPINL/SPINR/TURNL/TURNR/STOP/SPD/MODE")
    cmd.add_argument("--raw", help="直接给 payload，如 '$CAR,FWD,800' 或 'CAR,FWD,800'")

    ap.add_argument("--ms", type=int, default=800, help="运行时长 ms（默认 800 = 0.8s）")
    ap.add_argument("--speed", type=int, default=None, help="可选速度 PWM 0~999")
    ap.add_argument("--baud", type=int, default=115200, help="串口波特率，默认 115200")
    ap.add_argument("--timeout", type=float, default=1.5, help="等回包秒数，默认 1.5")
    args = ap.parse_args()

    # 组帧：两种输入都归一成一个 payload
    try:
        if args.raw:
            payload = args.raw.lstrip("$")
            frame = build_frame(payload)
        else:
            frame = build_car_frame(args.cmd, args.ms, args.speed)
    except ValueError as err:
        print("组帧失败：%s" % err)
        return 2

    # 打印成可粘贴的形式，方便贴进串口助手 / 写进验收记录
    print("帧：%r" % frame)
    print("明文：%s" % frame.rstrip("\r\n"))

    if args.dry_run:
        print("（--dry-run：没发出去，只验校验和）")
        return 0
    if args.port:
        return send_serial(frame, args.port, args.baud, args.timeout)
    return send_tcp(frame, args.tcp, args.timeout)


if __name__ == "__main__":
    sys.exit(main())
