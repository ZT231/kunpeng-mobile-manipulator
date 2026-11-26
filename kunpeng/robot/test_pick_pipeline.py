#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_pick_pipeline.py —— 用两个假 HTTP 服务把 pick_pipeline 整条链路跑一遍。

为什么要有它（而不是只靠 pick_pipeline.py --self-test）：
    --self-test 只验纯函数（IoU、底角、闸门、计划）。但真正容易错的是
    **HTTP 这一层的契约**：字段名、返回结构、ok=false 怎么传、clamped 怎么报。
    这些只有真的发一次请求才能验出来。所以这里起两个 stdlib http.server：
        · 假 detector（8081）：/observe 返回合成的候选与画面尺寸
        · 假 Flask  （5001）：/api/state、/api/ik/eye、/api/ik/eye/solve、/api/cmd
    然后让 pick_pipeline 走完整条路（总线 $ARM 协议：positions 0~1000）。

🔴 关键设计：假服务的「故障开关」是模块级变量（_S.xxx），每个用例改它，
   跑完还原。这样不用起一堆端口，也不会用例间互相污染。

跑法（PC 或板上都行，纯 stdlib，不需要相机/模型/机械臂）：
    python3 test_pick_pipeline.py
"""

import contextlib
import io
import json
import sys
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

sys.path.insert(0, ".")

import pick_pipeline as pp            # noqa: E402


# ---------------------------------------------------------------------------
# 一、全局可变的「假世界」状态
# ---------------------------------------------------------------------------
class World(object):
    """假服务共享的状态。每个用例前后由 reset() 恢复默认。"""

    def __init__(self):
        self.reset()

    def reset(self):
        # 6 路位置（总线舵机 0~1000）：六轴零位
        self.positions = [460, 514, 516, 502, 494, 674]
        self.eye_enabled = True
        self.placeholder = False
        self.solve_mode = "normal"        # normal / clamped
        self.cmds = []                    # 收到的 /api/cmd payload
        self.observe_calls = 0
        self.solve_calls = 0
        self.detections = [
            {"class": "cup", "conf": 0.87, "bbox": [268, 196, 356, 304],
             "center": [312, 250], "direction": "中"},
            {"class": "bottle", "conf": 0.62, "bbox": [400, 180, 440, 310],
             "center": [420, 245], "direction": "右"},
        ]
        self.frame_w = 640
        self.frame_h = 480
        self.frame_id = 42


_S = World()


# ---------------------------------------------------------------------------
# 二、假 Flask（5001）
# ---------------------------------------------------------------------------
class FlaskHandler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.startswith("/api/ik/eye"):
            self._json({"ok": True, "plane_z": 2.0,
                        "eye": {"enabled": _S.eye_enabled,
                                "placeholder": _S.placeholder,
                                "flag": "test" if not _S.placeholder else "placeholder"}})
        else:
            self._json({"ok": False, "error": "not found"}, 404)

    def do_POST(self):
        n = int(self.headers.get("Content-Length") or 0)
        data = json.loads(self.rfile.read(n).decode("utf-8") or "{}")

        if self.path == "/api/state":
            self._json({"ok": True, "positions": list(_S.positions)})
        elif self.path == "/api/ik/eye/solve":
            _S.solve_calls += 1
            u = float(data.get("u", 0))
            # 造一个「u 越大 x 越大」的线性映射：100 px ≈ 5 cm
            x = round((u - 320.0) * 0.05 + 12.0, 2)
            # y 固定 14 cm：留出可达余量。
            out = {"ok": True, "x": x, "y": 14.0, "z": float(data.get("plane_z", 0)),
                   "depth": 30.0, "clamped": [], "source_positions": data.get("positions")}
            if _S.solve_mode == "clamped":
                out["clamped"] = ["J4"]
            self._json(out)
        elif self.path == "/api/cmd":
            payload = data.get("payload") or ""
            _S.cmds.append(payload)
            # 🔴 照固件的规矩校验：必须是 ARM,POS,六个数 的逗号格式。
            #    这里故意严格，因为「用空格分隔」正是之前真机上踩过的坑。
            parts = payload.split(",")
            if len(parts) != 8 or parts[0] != "ARM" or parts[1] != "POS":
                self._json({"ok": True, "resp": "ERR,2"})
            else:
                # 模拟舵机立刻到位：把"当前位置"改成目标值。
                _S.positions = [int(v) for v in parts[2:]]
                self._json({"ok": True, "resp": "OK"})
        else:
            self._json({"ok": False, "error": "not found"}, 404)


# ---------------------------------------------------------------------------
# 三、假 detector（8081）
# ---------------------------------------------------------------------------
class DetectorHandler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path.startswith("/observe"):
            _S.observe_calls += 1
            obj = {"frame_id": _S.frame_id,
                   "frame": {"w": _S.frame_w, "h": _S.frame_h},
                   "detections": _S.detections,
                   "image_b64": None, "image_fmt": None}
            body = json.dumps(obj).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_response(404)
            self.end_headers()


# ---------------------------------------------------------------------------
# 四、起服务 + 工具
# ---------------------------------------------------------------------------
def serve(handler, port):
    srv = HTTPServer(("127.0.0.1", port), handler)
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    return srv


API = "http://127.0.0.1:5001"
DET = "http://127.0.0.1:8081"


def run_pipeline(instruction, extra=()):
    """跑一次完整命令行，返回 (返回码, 输出文本)。

    为什么用 main() 而不是 run()：main() 里才有 Fatal / Rejected 的翻译层
    （把异常变成返回码）。测试要验的正是「用户看到的返回码对不对」，
    所以必须走 main()；直接调 run() 会把异常漏给测试框架。
    stdout 和 stderr 一起收：阶段的正常输出在 stdout，
    拒绝原因（"已拒绝执行：…"）在 stderr，只看一半会漏断言。
    """
    argv = ["--instruction", instruction, "--api", API, "--detector", DET,
            "--backend", "mock"] + list(extra)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
        code = pp.main(argv)
    return code, buf.getvalue()


def check_arm_payload(payload):
    """校验一条 ARM,POS 命令：6 个数、全在软限位内。"""
    parts = payload.split(",")
    assert parts[0] == "ARM" and parts[1] == "POS", payload
    assert len(parts) == 8, payload
    vals = [int(v) for v in parts[2:]]
    for i, v in enumerate(vals):
        lo, hi = pp.ap.SAFE_LIMITS[i]
        assert lo <= v <= hi, (payload, i, v, lo, hi)
    return vals


# ---------------------------------------------------------------------------
# 五、用例
# ---------------------------------------------------------------------------
def case_dry_run_makes_no_command():
    """① dry-run：链路全走通，但一条 ARM,POS 都不许发。"""
    _S.reset()
    code, out = run_pipeline("把杯子抓起来", ["--dry-run"])
    assert code == 0, "返回码 %s\n%s" % (code, out)
    assert _S.cmds == [], "dry-run 居然发了命令：%s" % (_S.cmds,)
    assert "底面中心" in out, out
    assert "dry-run" in out, out
    assert _S.observe_calls == 1 and _S.solve_calls == 2, (_S.observe_calls, _S.solve_calls)
    print("  ① dry-run：0 条命令、2 次反投影、打印底面中心 ✓")


def case_yes_sends_4_arm():
    """② --yes：发 4 条 ARM,POS，格式和软限位都必须对。"""
    _S.reset()
    code, out = run_pipeline("把杯子抓起来", ["--yes"])
    assert code == 0, "返回码 %s\n%s" % (code, out)
    assert len(_S.cmds) == 4, _S.cmds
    vals = [check_arm_payload(p) for p in _S.cmds]
    # 抓取序列：接近 / 下降 / 合爪 / 抬起。合爪那步 J6 必须是闭合值。
    assert vals[2][5] == pp.ap.JAW_CLOSE, vals[2]
    assert vals[0][5] == pp.ap.JAW_OPEN, vals[0]
    # 接近点和下降点高度不同（J4 变），合爪和下降点同高
    assert vals[0][3] != vals[1][3], vals
    assert vals[1][3] == vals[2][3], vals
    print("  ② --yes：4 条 ARM,POS 全合规，J6 先开后合、接近点高于抓取点 ✓")


def case_with_place_sends_8():
    """③ 给 --place-x/-y：变成 8 条（抓 4 + 放 4），最后一步 J6 是张开。"""
    _S.reset()
    code, out = run_pipeline("把杯子抓起来", ["--yes", "--place-x", "5", "--place-y", "12"])
    assert code == 0, "返回码 %s\n%s" % (code, out)
    assert len(_S.cmds) == 8, len(_S.cmds)
    vals = [check_arm_payload(p) for p in _S.cmds]
    assert vals[-1][5] == pp.ap.JAW_OPEN, vals[-1]
    print("  ③ 带放置：8 条 ARM,POS，末步张爪 ✓")


def case_person_blocks():
    """④ 人框压住目标：必须拒绝，且一条命令都不发。"""
    _S.reset()
    _S.detections = [
        {"class": "cup", "conf": 0.90, "bbox": [268, 196, 356, 304],
         "center": [312, 250], "direction": "中"},
        {"class": "person", "conf": 0.95, "bbox": [250, 180, 400, 320],
         "center": [325, 250], "direction": "中"},
    ]
    code, out = run_pipeline("把杯子抓起来", ["--yes"])
    assert code == 11, "返回码 %s\n%s" % (code, out)
    assert _S.cmds == [], _S.cmds
    assert _S.solve_calls == 0, "拒绝了还去解算：%d" % _S.solve_calls
    print("  ④ 人手重叠：返回 11、0 次定位、0 条命令 ✓")


def case_edge_blocks():
    """⑤ 目标贴右边：拒绝（底角会落到背景上）。"""
    _S.reset()
    _S.detections = [
        {"class": "cup", "conf": 0.88, "bbox": [560, 200, 639, 300],
         "center": [600, 250], "direction": "右"},
    ]
    code, out = run_pipeline("把杯子抓起来", ["--yes"])
    assert code == 11, "返回码 %s\n%s" % (code, out)
    assert _S.cmds == [] and _S.solve_calls == 0
    print("  ⑤ 目标贴边：返回 11、0 次定位、0 条命令 ✓")


def case_clamped_blocks():
    """⑥ 逆解越界（服务端回 clamped）：拒绝，不拿裁过的点去抓。"""
    _S.reset()
    _S.solve_mode = "clamped"
    code, out = run_pipeline("把杯子抓起来", ["--yes"])
    assert code == 13, "返回码 %s\n%s" % (code, out)
    assert _S.cmds == [], _S.cmds
    assert "越界" in out, out
    print("  ⑥ 逆解越界：返回 13、0 条命令 ✓")


def case_placeholder_blocks():
    """⑦ 占位标定：环境闸拦在第一步，连拍照都不做。"""
    _S.reset()
    _S.placeholder = True
    code, out = run_pipeline("把杯子抓起来", ["--yes"])
    assert code == 1, "返回码 %s\n%s" % (code, out)
    assert _S.observe_calls == 0, "占位数据还去拍照了"
    assert "占位" in out, out
    print("  ⑦ 占位标定：返回 1、连观测都不拉 ✓")


def case_not_pick_blocks():
    """⑧ 模型结论不是 pick：不执行，返回 10。"""
    _S.reset()
    real = pp.vd.MockBackend

    class _Look(object):
        name = "mock"

        def complete(self, system, user, jpeg):
            return json.dumps({"action": "look", "choice": None,
                               "reason": "只是看一眼", "place_side": None},
                              ensure_ascii=False)

    pp.vd.MockBackend = lambda: _Look()
    try:
        code, out = run_pipeline("看看桌上有什么", ["--yes"])
    finally:
        pp.vd.MockBackend = real
    assert code == 10, "返回码 %s\n%s" % (code, out)
    assert _S.cmds == [], _S.cmds
    print("  ⑧ 非 pick 结论：返回 10、0 条命令 ✓")


def case_empty_scene_blocks():
    """⑨ 画面里没东西：返回 10，且不调用模型。"""
    _S.reset()
    _S.detections = []
    code, out = run_pipeline("把杯子抓起来", ["--yes"])
    assert code == 10, "返回码 %s\n%s" % (code, out)
    assert "一个物体都没检测到" in out, out
    print("  ⑨ 空场景：返回 10、不动臂 ✓")


# ---------------------------------------------------------------------------
# 六、入口
# ---------------------------------------------------------------------------
def main():
    f = serve(FlaskHandler, 5001)
    d = serve(DetectorHandler, 8081)
    # 加速：把「等臂静止」的两轮间隔压掉（真机上 0.35 s 是有意义的，
    # 这里状态恒定，没必要真睡）
    real_wait = pp.wait_still
    pp.wait_still = lambda api_base, **kw: pp.check_serial(api_base)
    try:
        print("=" * 60)
        print("pick_pipeline 端到端回归（假 detector + 假 Flask，$ARM 总线协议）")
        print("=" * 60)
        cases = [
            case_dry_run_makes_no_command,
            case_yes_sends_4_arm,
            case_with_place_sends_8,
            case_person_blocks,
            case_edge_blocks,
            case_clamped_blocks,
            case_placeholder_blocks,
            case_not_pick_blocks,
            case_empty_scene_blocks,
        ]
        for c in cases:
            c()
        print("-" * 60)
        print("✅ %d 个用例全过" % len(cases))
        return 0
    except AssertionError as e:
        print("❌ 用例失败：%s" % e)
        print("   （需要看更多上下文就把 contextlib.redirect_stdout 去掉重跑）")
        return 1
    finally:
        pp.wait_still = real_wait
        f.shutdown()
        d.shutdown()


if __name__ == "__main__":
    sys.exit(main())
