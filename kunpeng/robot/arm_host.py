#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机械臂示教回放上位机（S3）

目标板：STM32F407 + PCA9685（下位机 UART 协议 v1）
帧格式：$<CMD>[,<ARG>...]*<CK><CR><LF>，CK 为 $ 与 * 之间所有字节的 XOR（2 位大写十六进制）

功能：
  1) 串口指令封装（校验值自动计算，无需手算）
  2) 手动指令交互菜单（逐条对齐《串口联调测试清单》）
  3) 示教记录(REC) + 回放(PLAY) + 轨迹采集
  4) 6 路脉宽曲线可视化 + 原轨迹/回放对比 + 误差统计
  5) 轨迹保存/加载 JSON

依赖：
  pip install pyserial matplotlib

用法：
  python arm_host.py COM3                  # 经串口打开交互菜单
  python arm_host.py COM3 --baud 115200    # 指定波特率
  python arm_host.py COM3 --duration 12    # 一键示教回放：记录 12 秒
  python arm_host.py --wifi 192.168.4.1:9000  # 经 WiFi TCP 连 ESP-01S
"""

import argparse
import json
import os
import socket
import sys
import time
from datetime import datetime

try:
    import serial
except ImportError:
    print("缺少依赖 pyserial，请先执行： pip install pyserial")
    sys.exit(1)

CHANNEL_COUNT = 6
JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]
CMD_MAX_PULSE = 2500


# ---------------------------------------------------------------------------
# 协议层
# ---------------------------------------------------------------------------
def compute_ck(payload: str) -> int:
    """计算 XOR 校验值：payload 中每个字符的 ASCII 逐字节异或。"""
    ck = 0
    for ch in payload:
        ck ^= ord(ch)
    return ck & 0xFF


def make_frame(payload: str) -> bytes:
    """构造完整帧：$payload*CK\\r\\n"""
    ck = compute_ck(payload)
    return ("$%s*%02X\r\n" % (payload, ck)).encode("ascii")


# ---------------------------------------------------------------------------
# 传输层抽象：串口或 TCP 均可用（duck-typed: write/read/reset_input_buffer/close）
# ---------------------------------------------------------------------------
class TcpTransport:
    """TCP 传输层：与 serial.Serial 接口对齐，供 ArmHost 无缝切换。"""

    def __init__(self, host: str, port: int, timeout: float = 0.5):
        self._addr = "%s:%d" % (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(2.0)
        self.sock.connect((host, port))
        self.sock.settimeout(timeout)
        self.is_open = True
        self._buf = bytearray()

    def write(self, data: bytes) -> int:
        return self.sock.send(data)

    def read(self, n: int = 1) -> bytes:
        try:
            data = self.sock.recv(n)
        except socket.timeout:
            return b""
        if data:
            self._buf.extend(data)
        if self._buf:
            out = bytes(self._buf[:n])
            del self._buf[:n]
            return out
        return b""

    def reset_input_buffer(self):
        self._buf.clear()
        try:
            self.sock.setblocking(False)
            while True:
                try:
                    chunk = self.sock.recv(4096)
                    if not chunk:
                        break
                except (BlockingIOError, socket.error):
                    break
            self.sock.settimeout(0.5)
        except Exception:
            pass

    def close(self):
        self.is_open = False
        try:
            self.sock.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# 串口与指令封装
# ---------------------------------------------------------------------------
class ArmHost:
    """串口/TCP 封装 + 下位机指令集 + 响应解析。"""

    def __init__(self, transport, timeout: float = 0.5):
        self.ser = transport
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        # 上电先关闭周期遥测，保证 request-response 干净、响应不混入日志帧
        self.log(0)

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    # ---- 底层收发 ----
    def _read_frame(self, timeout: float = 1.0) -> str:
        """读到一行 '$...\\r\\n' 帧，返回 $ 与 * 之间的 payload 字符串。"""
        line = bytearray()
        deadline = time.time() + timeout
        while time.time() < deadline:
            ch = self.ser.read(1)
            if not ch:
                continue
            if ch in (b"\r", b"\n"):
                if line:
                    break
                continue
            line += ch
        s = bytes(line).decode("ascii", errors="replace").strip()
        if s.startswith("$"):
            s = s[1:]
        if "*" in s:
            s = s.split("*")[0]
        return s

    def send(self, payload: str, timeout: float = 1.0) -> str:
        """发送命令并等待一行响应，返回响应 payload。"""
        self.ser.reset_input_buffer()
        self.ser.write(make_frame(payload))
        return self._read_frame(timeout)

    # ---- 基础指令 ----
    def ping(self):          return self.send("PING")
    def state(self):         return self.send("STATE")
    def home(self):          return self.send("HOME")
    def stop(self):          return self.send("STOP")
    def resume(self):        return self.send("RESUME")
    def manual(self):        return self.send("MANUAL")
    def move(self, ch, us):  return self.send("MOVE,%d,%d" % (ch, us))
    def joint(self, *us6):   return self.send("JOINT,%s" % ",".join(str(int(p)) for p in us6))
    def rec(self):           return self.send("REC")
    def recend(self):        return self.send("RECEND")
    def play(self):          return self.send("PLAY")
    def tnum(self):          return self.send("TNUM")
    def clear(self):         return self.send("CLEAR")
    def log(self, on):       return self.send("LOG,%d" % (1 if on else 0))

    # ---- 响应解析 ----
    @staticmethod
    def is_ok(resp: str) -> bool:
        return resp.startswith("OK")

    @staticmethod
    def parse_state(resp: str):
        """STATE 响应 'OK,S,<状态>,<p0>..<p5>' -> (状态字符, [6 路脉宽])"""
        parts = resp.split(",")
        if len(parts) < 9:
            raise ValueError("STATE 响应格式异常: %r" % resp)
        state_ch = parts[2].strip()
        pulses = [int(parts[i]) for i in range(3, 9)]
        return state_ch, pulses

    def state_pulses(self):
        state_ch, pulses = self.parse_state(self.state())
        return state_ch, pulses

    @staticmethod
    def parse_count(resp: str) -> int:
        """RECEND/TNUM 响应 'OK,<n>' -> n"""
        parts = resp.split(",")
        if parts and parts[0].startswith("OK") and len(parts) >= 2:
            try:
                return int(parts[1])
            except ValueError:
                return 0
        return 0

    # ---- 数据采集 ----
    def capture(self, duration_s: float, interval_s: float = 0.1):
        """轮询 STATE 采集 6 路脉宽，返回 [(t, [p0..p5]), ...]。
        用 Ctrl+C 可提前结束（已采数据保留）。"""
        traj = []
        t0 = time.time()
        warned = False
        try:
            while time.time() - t0 < duration_s:
                try:
                    _, p = self.state_pulses()
                except Exception:
                    if not warned:
                        print("  [警告] 一次 STATE 读取失败，继续…")
                        warned = True
                    time.sleep(interval_s)
                    continue
                t = time.time() - t0
                traj.append((t, p))
                print("  t=%6.2fs  " % t +
                      "  ".join("%s=%4d" % (JOINT_NAMES[i], p[i]) for i in range(CHANNEL_COUNT)))
                time.sleep(interval_s)
        except KeyboardInterrupt:
            print("\n  已手动提前结束采集。")
        return traj


# ---------------------------------------------------------------------------
# 可视化
# ---------------------------------------------------------------------------
def _plot_channels(traj, title, out_path):
    import matplotlib.pyplot as plt

    if not traj:
        print("  无数据可画。")
        return
    t = [p[0] for p in traj]
    fig, axs = plt.subplots(CHANNEL_COUNT, 1, figsize=(10, 12), sharex=True)
    for i in range(CHANNEL_COUNT):
        y = [p[1][i] for p in traj]
        axs[i].plot(t, y, label=JOINT_NAMES[i], linewidth=1.2)
        axs[i].set_ylabel("%s (us)" % JOINT_NAMES[i])
        axs[i].grid(True, alpha=0.3)
        axs[i].legend(loc="upper right", fontsize=8)
    axs[-1].set_xlabel("Time (s)")
    fig.suptitle(title, fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.98])
    if out_path:
        fig.savefig(out_path, dpi=120)
        print("  曲线已保存：%s" % out_path)
    try:
        plt.show(block=False)   # 非阻塞：弹出窗口但不卡住流程
        plt.pause(1.0)
    except Exception:
        pass
    plt.close(fig)


def plot_trajectory(traj, title="Trajectory", out_path=None):
    _plot_channels(traj, title, out_path)


def plot_compare(orig, replay, out_path=None):
    """原轨迹 vs 回放对比 + 每通道误差。"""
    import matplotlib.pyplot as plt

    if not orig or not replay:
        print("  原轨迹或回放轨迹为空，无法对比。")
        return
    n = min(len(orig), len(replay))
    t_orig = [orig[i][0] for i in range(n)]
    t_rep = [replay[i][0] for i in range(n)]

    fig, axs = plt.subplots(CHANNEL_COUNT, 1, figsize=(10, 12), sharex=True)
    err_msgs = []
    for i in range(CHANNEL_COUNT):
        yo = [orig[k][1][i] for k in range(n)]
        yr = [replay[k][1][i] for k in range(n)]
        err = [abs(yo[k] - yr[k]) for k in range(n)]
        max_e = max(err)
        avg_e = sum(err) / n
        err_msgs.append("%s 平均误差 %.1fus 最大误差 %.1fus" % (JOINT_NAMES[i], avg_e, max_e))

        axs[i].plot(t_orig, yo, label="teach", linewidth=1.2)
        axs[i].plot(t_rep, yr, label="replay", linewidth=1.2, linestyle="--", alpha=0.85)
        ax2 = axs[i].twinx()
        ax2.plot(t_rep, err, label="error", color="red", linewidth=0.8, alpha=0.5)
        ax2.set_ylabel("err (us)", fontsize=7)
        axs[i].set_ylabel("%s (us)" % JOINT_NAMES[i])
        axs[i].grid(True, alpha=0.3)
        axs[i].legend(loc="upper right", fontsize=7)

    axs[-1].set_xlabel("Time (s)")
    fig.suptitle("Teach vs Replay (n=%d points)" % n, fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.98])

    print("\n  复现误差统计（对齐前 %d 个采样点）：" % n)
    for m in err_msgs:
        print("    " + m)

    if out_path:
        fig.savefig(out_path, dpi=120)
        print("  对比图已保存：%s" % out_path)
    try:
        plt.show(block=False)   # 非阻塞
        plt.pause(1.0)
    except Exception:
        pass
    plt.close(fig)


# ---------------------------------------------------------------------------
# 轨迹 JSON 存取
# ---------------------------------------------------------------------------
def save_traj(path, traj):
    obj = {
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "channel_labels": JOINT_NAMES,
        "point_count": len(traj),
        "points": [{"t": round(t, 3), "pulses": p} for t, p in traj],
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=2)
    print("  轨迹已保存：%s (%d 点)" % (path, len(traj)))


def load_traj(path):
    with open(path, "r", encoding="utf-8") as f:
        obj = json.load(f)
    return [(pt["t"], pt["pulses"]) for pt in obj["points"]]


def _scan_cfg_path():
    """眼在手扫描姿态配置文件（与本脚本同目录；首次执行菜单 [17] 保存时自动创建）。"""
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "scan_config.json")


def _load_scan_cfg():
    try:
        with open(_scan_cfg_path(), "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}


# ---------------------------------------------------------------------------
# 工作流
# ---------------------------------------------------------------------------
def run_teach_playback(host: ArmHost, duration_s: float, out_dir: str):
    """一键示教回放：REC -> 采集 -> RECEND -> PLAY -> 采集 -> 对比图。"""
    print("\n[1/4] 开始记录（%.1f 秒），请拧动电位器…" % duration_s)
    r = host.rec()
    if not host.is_ok(r):
        print("  REC 失败：%r" % r)
        return
    print("  REC 已确认，记录中（Ctrl+C 可提前结束）：")

    orig = host.capture(duration_s)

    r = host.recend()
    n = host.parse_count(r)
    print("\n[2/4] RECEND：%r，共 %d 个示教点" % (r, n))
    if n == 0:
        print("  未采到轨迹点，检查是否在记录期间移动了电位器。")
        return

    out_prefix = os.path.join(out_dir, datetime.now().strftime("teach_%Y%m%d_%H%M%S"))
    save_traj(out_prefix + "_orig.json", orig)
    plot_trajectory(orig, "Recorded Trajectory (%d pts)" % n, out_prefix + "_orig.png")

    print("\n[3/4] 开始回放…")
    r = host.play()
    if not host.is_ok(r):
        print("  PLAY 失败：%r" % r)
        return
    replay = _capture_until_idle(host, timeout_s=duration_s * 2 + 5)

    save_traj(out_prefix + "_replay.json", replay)
    print("\n[4/4] 生成对比图…")
    plot_compare(orig, replay, out_prefix + "_compare.png")


def _capture_until_idle(host: ArmHost, timeout_s: float, interval_s: float = 0.1):
    """回放期间采样，直到状态离开 PLAYBACK(P) 或超时。返回轨迹。"""
    traj = []
    t0 = time.time()
    try:
        while time.time() - t0 < timeout_s:
            try:
                st, p = host.state_pulses()
            except Exception:
                time.sleep(interval_s)
                continue
            traj.append((time.time() - t0, p))
            print("  replay t=%6.2fs  " % (time.time() - t0) +
                  "  ".join("%s=%4d" % (JOINT_NAMES[i], p[i]) for i in range(CHANNEL_COUNT)))
            if st != "P":
                print("  回放结束（状态=%s），共采 %d 点" % (st, len(traj)))
                break
            time.sleep(interval_s)
    except KeyboardInterrupt:
        print("\n  已手动停止回放采集。")
    return traj


# ---------------------------------------------------------------------------
# 交互菜单
# ---------------------------------------------------------------------------
def interactive_menu(host: ArmHost, duration_s: float, out_dir: str):
    menu = """
================  机械臂示教回放上位机  ================
  端口已连接，命令校验值自动计算（无需手算）
------------------------------------------------------
 [1]  PING        链路测试
 [2]  STATE       查询状态 + 六路当前脉宽
 [3]  HOME        回 HOME 位
 [4]  STOP        急停冻结
 [5]  RESUME      解除急停(回手动)
 [6]  MANUAL      切手动(电位器控制)
 [7]  MOVE        单轴控制   (输入: 通道,脉宽 例 0,1500)
 [8]  JOINT       六轴同步   (输入: 6 个数,逗号分隔)
 [9]  REC         开始示教记录
 [10]  RECEND      结束记录(返回点数)
 [11]  PLAY        回放
 [12]  TNUM        查询轨迹点数
 [13]  CLEAR       清空轨迹
 [14]  LOG开关      周期遥测开关 (输入 0/1)
 [15]  一键示教回放 (记录 %.0f s + 回放 + 对比图)
 [16]  SCAN姿态    移动到扫描姿态 (读同目录 scan_config.json 的 scan_pulses)
 [17]  SCAN保存    把当前 6 路脉宽保存为扫描姿态 (眼在手用)
 [q]   退出
------------------------------------------------------"""
    print(menu)
    while True:
        try:
            k = input("选择 > ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        try:
            if k == "1":
                print("  ->", host.ping())
            elif k == "2":
                r = host.state()
                st, p = host.parse_state(r)
                print("  -> 状态=%s  %s" % (st, "  ".join("%s=%d" % (JOINT_NAMES[i], p[i]) for i in range(CHANNEL_COUNT))))
            elif k == "3":
                print("  ->", host.home())
            elif k == "4":
                print("  ->", host.stop())
            elif k == "5":
                print("  ->", host.resume())
            elif k == "6":
                print("  ->", host.manual())
            elif k == "7":
                s = input("  输入 通道,脉宽 (如 0,1500): ").strip()
                ch, us = s.split(",")
                print("  ->", host.move(int(ch), int(us)))
            elif k == "8":
                s = input("  输入 6 个脉宽,逗号分隔: ").strip()
                us6 = [int(x) for x in s.split(",")]
                if len(us6) != 6:
                    print("  需要 6 个数。")
                else:
                    print("  ->", host.joint(*us6))
            elif k == "9":
                print("  ->", host.rec())
            elif k == "10":
                r = host.recend()
                print("  ->", r, "(点数=%d)" % host.parse_count(r))
            elif k == "11":
                print("  ->", host.play())
            elif k == "12":
                r = host.tnum()
                print("  ->", r, "(点数=%d)" % host.parse_count(r))
            elif k == "13":
                print("  ->", host.clear())
            elif k == "14":
                s = input("  输入 0 关闭 / 1 开启: ").strip()
                print("  ->", host.log(int(s)))
            elif k == "15":
                run_teach_playback(host, duration_s, out_dir)
            elif k == "16":
                cfg = _load_scan_cfg()
                sp = cfg.get("scan_pulses")
                if isinstance(sp, list) and len(sp) == 6:
                    print("  扫描姿态 ->", host.joint(*[int(p) for p in sp]))
                else:
                    print("  vision/config.json 无 scan_pulses：先用 [8] JOINT 摆到扫描姿态，再按 [17] 保存")
            elif k == "17":
                _, p = host.state_pulses()
                cfg = _load_scan_cfg()
                cfg["scan_pulses"] = p
                with open(_scan_cfg_path(), "w", encoding="utf-8") as f:
                    json.dump(cfg, f, indent=2, ensure_ascii=False)
                print("  已保存扫描姿态 %s -> %s" % (p, _scan_cfg_path()))
            elif k in ("q", "Q"):
                break
            else:
                print("  未知选项，重新输入。")
        except Exception as e:
            print("  出错：", e)


def main():
    ap = argparse.ArgumentParser(description="机械臂示教回放上位机 (S3)")
    ap.add_argument("port", nargs="?", default=None, help="串口，如 COM3 或 /dev/ttyUSB0（与 --wifi 二选一）")
    ap.add_argument("--wifi", default=None, help="WiFi TCP 地址，如 192.168.4.1:9000（与 port 二选一）")
    ap.add_argument("--baud", type=int, default=115200, help="波特率，默认 115200")
    ap.add_argument("--duration", type=float, default=10.0, help="一键示教回放的记录时长(秒)，默认 10")
    ap.add_argument("--out", default=None, help="输出目录，默认脚本同目录 outputs/")
    args = ap.parse_args()

    if not args.port and not args.wifi:
        ap.error("必须指定串口 (COM3) 或 WiFi (--wifi 192.168.4.1:9000)")
    if args.port and args.wifi:
        ap.error("port 和 --wifi 二选一，不可同时指定")

    out_dir = args.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), "outputs")
    os.makedirs(out_dir, exist_ok=True)

    if args.wifi:
        wifi_host, _, port_s = args.wifi.partition(":")
        port = int(port_s) if port_s else 9000
        print("正在连接 WiFi TCP %s:%d …" % (wifi_host, port))
        try:
            transport = TcpTransport(wifi_host, port)
        except Exception as e:
            print("WiFi 连接失败：%s" % e)
            sys.exit(1)
        label = "WiFi %s:%d" % (wifi_host, port)
    else:
        try:
            transport = serial.Serial(args.port, args.baud, timeout=0.5)
        except Exception as e:
            print("无法打开串口 %s：%s" % (args.port, e))
            sys.exit(1)
        label = "%s @ %d" % (args.port, args.baud)

    host = ArmHost(transport)
    try:
        print("已连接 %s，链路测试：" % label)
        print("  ->", host.ping())
        interactive_menu(host, args.duration, out_dir)
    finally:
        host.close()
        print("\n连接已关闭。")


if __name__ == "__main__":
    main()