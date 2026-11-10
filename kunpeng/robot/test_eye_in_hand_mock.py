"""端到端 mock 验证：占位 X/K + 模拟串口 -> 像素 -> 眼在手 -> 工作台 -> IK -> 脉宽。

为什么需要这个脚本（这是「软件先行」的核心价值）：
    真机验证要相机、要串口、要标定板，全齐了才能验一次；
    但「代码链路有没有接错」这件事，完全可以在 PC 上用 Flask 的 test_client
    把所有 HTTP 接口按真实顺序打一遍 —— 不需要任何硬件。

它验的是"接线正确"：
    ✅ 接口能调通、字段能取到、数值量级合理、占位标记能透传、错误能被挡住
    ❌ 它验的不是"标定准不准" —— 标定精度只能靠真机实标

跑法（PC 或板都可以）：
    python3 test_eye_in_hand_mock.py

依赖：flask + numpy（+ pyserial 用于真实串口，本脚本用假串口替身，不碰真设备）
"""

import json
import math
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
WEB_DIR = os.path.join(os.path.dirname(HERE), "web")
ROBOT_DIR = HERE
sys.path.insert(0, WEB_DIR)
sys.path.insert(0, ROBOT_DIR)

PASSED = []
FAILED = []


def check(name, cond, detail=""):
    """记一条检查结果；失败不中断，最后统一汇总（一次跑完看全部问题）。"""
    if cond:
        PASSED.append(name)
        print("  PASS  %s  %s" % (name, detail))
    else:
        FAILED.append(name)
        print("  FAIL  %s  %s" % (name, detail))


def install_fake_serial():
    """装一个假的 serial 模块，让 web_control.py 能在没有真串口的机器上 import。

    为什么必须这么做：web_control.py 在模块顶层 `import serial`，本机没装 pyserial
    就会直接 ImportError，连 test_client 都建不起来。假模块只提供它用到的属性。
    概念（大白话）：给程序一个"仿真串口"，它会以为自己连着板子。
    规范定义：依赖替身（test double / stub），仅提供被调用接口的最小实现。
    """
    import types as _t
    if "serial" in sys.modules and getattr(sys.modules["serial"], "__fake__", False):
        return

    fake = _t.ModuleType("serial")

    class SerialException(Exception):
        pass

    class Serial:
        def __init__(self, port, baud, timeout=0.2):
            self.port = port
            self.baud = baud
            self.timeout = timeout
            self.is_open = True
            self._buf = bytearray()
            self._sent = []

        def read(self, n=1):
            if self._buf:
                ch = bytes(self._buf[:1])
                del self._buf[:1]
                return ch
            return b""

        def write(self, data):
            # 假固件：识别 PING / STATE / LOG / JOINT，把应答塞进读缓冲
            self._sent.append(data)
            text = data.decode("ascii", errors="replace").strip()
            payload = text.strip("$").split("*")[0] if text.startswith("$") else text
            if payload == "PING":
                self._push("$OK,PONG*3E\r\n")
            elif payload == "STATE":
                self._push("$OK,S,C,1580,1900,2400,700,1450,1500*00\r\n")
            elif payload.startswith("JOINT"):
                self._push("$OK,JOINT*00\r\n")
            elif payload.startswith("LOG"):
                self._push("$OK,LOG*00\r\n")
            return len(data)

        def _push(self, s):
            self._buf += s.encode("ascii")

        def reset_input_buffer(self):
            self._buf = bytearray()

        def close(self):
            self.is_open = False

    fake.Serial = Serial
    fake.SerialException = SerialException
    fake.__fake__ = True
    sys.modules["serial"] = fake


def main():
    print("眼在手链路 · 端到端 mock 验证")
    print("=" * 60)

    install_fake_serial()

    try:
        import numpy as np
    except ImportError:
        print("缺少 numpy，本机请用带 numpy 的 Python：")
        print("  D:\\Python\\Python3.14.7\\python.exe test_eye_in_hand_mock.py")
        return 1

    # ---- 0) 占位文件必须先就位 ----
    x_path = os.path.join(ROBOT_DIR, "handeye_X.json")
    k_path = os.path.join(ROBOT_DIR, "camera_intrinsics.json")
    print("\n[0] 占位标定文件")
    check("handeye_X.json 存在", os.path.isfile(x_path), x_path)
    check("camera_intrinsics.json 存在", os.path.isfile(k_path), k_path)
    if not (os.path.isfile(x_path) and os.path.isfile(k_path)):
        print("  先跑：python3 gen_placeholder_calib.py")
        return 1
    with open(x_path, "r", encoding="utf-8") as f:
        xd = json.load(f)
    with open(k_path, "r", encoding="utf-8") as f:
        kd = json.load(f)
    check("X 文件带 placeholder 标记", xd.get("placeholder") is True)
    check("K 文件带 placeholder 标记", kd.get("placeholder") is True,
          "防止占位数据被当成实测值四处引用")

    # ---- 1) 几何模块自检 ----
    print("\n[1] 几何模块自检（合成数据）")
    import handeye_geometry as hg
    ok = hg.selftest()
    check("handeye_geometry.selftest 全绿", ok)

    # ---- 2) 加载 web_control，初始化两条算路 ----
    print("\n[2] 加载 web_control 并初始化逆解 + 眼在手")
    import web_control as wc
    check("web_control 成功导入", True, "假 serial 生效")

    with wc.app.test_client() as c:
        # 逆解模块从 robot 目录加载（arm_ik.py + arm_params.json 都在这）
        ik_ok = wc.init_ik(ROBOT_DIR)
        check("逆解模块加载成功", ik_ok, wc._IK_ERROR or "已就绪")

        eye_ok = wc.init_eye_in_hand(ROBOT_DIR, x_path, k_path)
        check("眼在手链路加载成功", eye_ok, wc._EYE_ERROR or "已就绪")

        # ---- 3) 配置接口 ----
        print("\n[3] /api/ik/config 与 /api/ik/eye")
        r = c.get("/api/ik/config").get_json()
        check("config.ready=True", r.get("ready") is True)
        check("config 带 eye 字段", "eye" in r)
        check("config.eye.enabled=True", r["eye"].get("enabled") is True)
        check("config.eye.placeholder=True", r["eye"].get("placeholder") is True,
              "占位数据要在接口层就能看到")

        r = c.get("/api/ik/eye").get_json()
        check("eye.flag 含『占位』", "占位" in (r.get("eye", {}).get("flag") or ""),
              r.get("eye", {}).get("flag", ""))
        check("eye.plane_z=抓取高度", abs(float(r.get("plane_z", 0)) - wc._IK.grab_h) < 1e-9,
              "plane_z=%s" % r.get("plane_z"))

        # ---- 4) 不连串口时，眼在手接口必须明确报错而不是瞎算 ----
        print("\n[4] 串口未连接时的行为（最容易出假成功的地方）")
        r = c.post("/api/ik/eye/solve", json={"u": 320, "v": 240}).get_json()
        check("未连串口 -> ok=False", r.get("ok") is False, r.get("error", ""))
        check("错误信息点明缺关节角", "关节角" in (r.get("error") or "") or "未连接" in (r.get("error") or ""))

        # ---- 5) 连上假串口后，走完整链路 ----
        print("\n[5] 连假串口 -> 试算（不下发）")
        r = c.post("/api/open", json={"port": "FAKE"}).get_json()
        check("/api/open ok", r.get("ok") is True)

        r = c.post("/api/state").get_json()
        check("/api/state 拿到 6 路", len(r.get("pulses") or []) == 6, str(r.get("pulses")))

        body = {"u": 320, "v": 240, "apply": False}
        r = c.post("/api/ik/eye/solve", json=body).get_json()
        check("试算 ok=True", r.get("ok") is True, r.get("error", ""))
        check("mode=eye_in_hand", r.get("mode") == "eye_in_hand")
        check("返回 x/y/z 三坐标", all(k in r for k in ("x", "y", "z")))
        check("返回 depth 且为正", isinstance(r.get("depth"), (int, float)) and r["depth"] > 0,
              "depth=%.2f cm" % (r.get("depth") or 0))
        check("6 路脉宽齐全", len(r.get("pulses") or []) == 6, str(r.get("pulses")))
        check("脉宽全在软限位内",
              all(lo <= p <= hi for p, (lo, hi, _) in zip(r.get("pulses") or [], wc.PULSE_RANGES)),
              str(r.get("pulses")))
        check("placehodler 标记透传到结果", r.get("placeholder") is True)
        check("未下发（applied=False）", r.get("applied") is False)
        check("试算结果带占位警告", "占位" in (r.get("note") or ""), r.get("note", ""))

        x0, y0 = r["x"], r["y"]

        # ---- 6) 同一个像素，臂不动 -> 结果必须完全一致（确定性）----
        print("\n[6] 确定性：同一像素 + 同一脉宽 -> 同一结果")
        r2 = c.post("/api/ik/eye/solve", json=body).get_json()
        check("两次结果完全相同",
              abs(r2["x"] - x0) < 1e-9 and abs(r2["y"] - y0) < 1e-9,
              "x:%.4f y:%.4f" % (r2["x"], r2["y"]))

        # ---- 7) 臂换姿态 -> 同一像素必须得到不同结果（这正是眼在手的价值）----
        print("\n[7] 关键差异：换姿态后同一像素结果应改变")
        other = {"u": 320, "v": 240, "apply": False,
                 "pulses": [1450, 1850, 2350, 730, 1480, 1500]}
        r3 = c.post("/api/ik/eye/solve", json=other).get_json()
        # 注意判据：这里必须允许 IK 失败。因为「相机看得见」和「臂够得着」是两件事——
        # 相机在末端偏 2cm 朝下看，换个臂姿后视线落点可能刚好越过 25.2cm 臂展。
        # 本项要验的是"坐标随姿态变化"这一几何事实，不是"任意姿态都能抓"。
        check("换姿态后仍返回工作台坐标（IK 可单独失败）",
              "x" in r3 and "y" in r3,
              "ok=%s  error=%s" % (r3.get("ok"), (r3.get("error") or "")[:50]))
        moved = math.hypot(r3["x"] - x0, r3["y"] - y0)
        check("工作台坐标随姿态改变", moved > 1.0,
              "偏移 %.1f cm（旧线性映射此处恒为 0，因为它不看关节角）" % moved)
        check("IK 越界时明确报『超出可达范围』",
              r3.get("ok") is True or "可达范围" in (r3.get("error") or ""),
              r3.get("error") or "本姿态恰好可达")

        # ---- 8) 画面外的极端像素必须被护栏挡住，而不是返回离谱坐标 ----
        print("\n[8] 非法输入的护栏")
        r4 = c.post("/api/ik/eye/solve", json={"u": 99999, "v": 99999}).get_json()
        check("极端像素被拒或给出明确提示",
              r4.get("ok") is False or "超出范围" in (r4.get("note") or ""),
              "ok=%s error=%s" % (r4.get("ok"), (r4.get("error") or "")[:60]))

        r5 = c.post("/api/ik/eye/solve", json={"u": 320, "v": 240, "pulses": [1, 2]}).get_json()
        check("脉宽长度非法 -> 回退到串口读取", r5.get("ok") is True, r5.get("error", ""))

        # ---- 9) 真下发 ----
        print("\n[9] apply=True 走真实下发路径（假串口收帧）")
        r6 = c.post("/api/ik/eye/solve", json={"u": 320, "v": 240, "apply": True}).get_json()
        check("下发 ok=True", r6.get("ok") is True, r6.get("error", ""))
        check("applied=True", r6.get("applied") is True)
        check("固件应答 OK", (r6.get("resp") or "").startswith("OK"), r6.get("resp", ""))
        check("下发的帧含 6 个脉宽", (r6.get("frame") or "").count(",") == 6, r6.get("frame", ""))

        # ---- 10) 旧通路必须完好（不能被新功能改坏）----
        print("\n[10] 回归：旧的 /api/ik/solve 仍按原样工作")
        r7 = c.post("/api/ik/solve", json={"u": 320, "v": 240, "w": 640, "h": 480}).get_json()
        check("旧接口 ok=True", r7.get("ok") is True, r7.get("error", ""))
        check("旧接口无 mode 字段（保持原语义）", "mode" not in r7)
        check("旧接口仍返回 6 路脉宽", len(r7.get("pulses") or []) == 6)

        # ---- 11) 不启用眼在手时，接口必须明确拒绝 ----
        print("\n[11] 关闭眼在手后接口应拒绝")
        saved = wc._EYE
        wc._EYE = None
        r8 = c.post("/api/ik/eye/solve", json={"u": 320, "v": 240}).get_json()
        check("未启用 -> ok=False", r8.get("ok") is False, r8.get("error", ""))
        check("提示加 --eye-in-hand", "--eye-in-hand" in (r8.get("error") or ""))
        wc._EYE = saved

        # ---- 12) 页面必须仍能正常渲染 ----
        print("\n[12] 页面渲染回归")
        page = c.get("/")
        check("GET / 返回 200", page.status_code == 200)
        html = page.get_data(as_text=True)
        check("页面含滑杆 J1~J6", html.count("pulse") >= 6 or "J6" in html)
        check("页面无渲染异常标记", "Traceback" not in html)

    print("\n" + "=" * 60)
    print("通过 %d 项 / 失败 %d 项" % (len(PASSED), len(FAILED)))
    if FAILED:
        print("失败项：")
        for n in FAILED:
            print("  - %s" % n)
        return 1
    print("结论：链路在本机（占位数据 + 假串口）端到端打通。")
    print("⚠ 这不等于实机通过：标定精度、真实关节角、相机视野仍待真机验证。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
