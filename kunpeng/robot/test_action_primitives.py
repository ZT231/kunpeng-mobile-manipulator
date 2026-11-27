#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_action_primitives.py —— 动作原语层回归测试（总线舵机 $ARM 协议版）

不需要真机、不需要 Flask，用假 HTTP 服务验证 plan_pick / plan_place / plan_home /
clip_positions / run_plan 的核心分支。

用法：
  python3 test_action_primitives.py        # 全部用例通过才返回 0
"""

import http.server
import json
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import action_primitives as ap


class FakeFlask(http.server.BaseHTTPRequestHandler):
    """冒充 web_control.py 的两个接口：/api/cmd 与 /api/state（总线 $ARM 协议）。

    每次 /api/state 让每关节向目标走 STEP 位置单位，这样 wait_arrive 的轮询逻辑
    才会真的被触发（一步到位就测不出轮询）。
    """

    state = [460, 514, 516, 502, 494, 674]   # 总线六轴零位
    target = list(state)
    fail_mode = None          # None | "err" | "stuck"
    cmd_log = []
    poll_count = 0
    STEP = 40                 # 每轮询一次每关节走 40 位置单位

    def log_message(self, *a):
        pass                  # 静音，避免输出淹没重点

    def _json(self, obj):
        body = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(n) or b"{}")

        if self.path == "/api/cmd":
            payload = body.get("payload", "")
            FakeFlask.cmd_log.append(payload)
            if FakeFlask.fail_mode == "err":
                # 固件回 ERR,2（未知命令）：ok 仍是 true，真实 /api/cmd 就是如此。
                self._json({"ok": True, "resp": "ERR,2"})
                return
            if payload.startswith("ARM,POS,"):
                FakeFlask.target = [int(x) for x in payload.split(",")[2:]]
                self._json({"ok": True, "resp": "OK"})
                return
            self._json({"ok": True, "resp": "OK"})
            return

        if self.path == "/api/state":
            FakeFlask.poll_count += 1
            for i in range(6):
                if FakeFlask.fail_mode == "stuck" and i == 3:
                    continue                      # J4 永远不动，模拟舵机卡死
                d = FakeFlask.target[i] - FakeFlask.state[i]
                if abs(d) <= FakeFlask.STEP:
                    FakeFlask.state[i] = FakeFlask.target[i]
                else:
                    FakeFlask.state[i] += FakeFlask.STEP if d > 0 else -FakeFlask.STEP
            self._json({"ok": True, "positions": list(FakeFlask.state)})
            return

        self._json({"ok": False, "error": "unknown path"})


def start_server():
    srv = http.server.ThreadingHTTPServer(("127.0.0.1", 18080), FakeFlask)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def reset():
    """每个用例开始前把假服务恢复到初始位，避免用例之间互相污染。"""
    FakeFlask.state = [460, 514, 516, 502, 494, 674]
    FakeFlask.target = list(FakeFlask.state)
    FakeFlask.fail_mode = None
    FakeFlask.cmd_log = []
    FakeFlask.poll_count = 0


def main():
    srv = start_server()
    ap.BASE_URL = "http://127.0.0.1:18080"
    ok = True

    # ---- 用例 1：plan_pick 结构 —— 4 步、J6 先开后合 ----
    plan = ap.plan_pick(8.0, 12.0, z=2.0)
    assert len(plan.steps) == 4, len(plan.steps)
    assert plan.steps[0].positions[5] == ap.JAW_OPEN
    assert plan.steps[1].positions[5] == ap.JAW_OPEN
    assert plan.steps[2].positions[5] == ap.JAW_CLOSE
    assert plan.steps[3].positions[5] == ap.JAW_CLOSE
    print("用例1 plan_pick 结构（4 步，J6 开→开→合→合）✓")

    # ---- 用例 2：plan_place 结构 —— 4 步、J6 先合后开 ----
    plan = ap.plan_place(5.0, 12.0, z=2.0)
    assert len(plan.steps) == 4, len(plan.steps)
    assert plan.steps[0].positions[5] == ap.JAW_CLOSE
    assert plan.steps[1].positions[5] == ap.JAW_CLOSE
    assert plan.steps[2].positions[5] == ap.JAW_OPEN
    assert plan.steps[3].positions[5] == ap.JAW_OPEN
    print("用例2 plan_place 结构（4 步，J6 合→合→开→开）✓")

    # ---- 用例 3：clip_positions 限位裁剪必须生效且被报告 ----
    clipped, names = ap.clip_positions((-50, 500, 500, 1200, 500, 800))
    assert clipped == (10, 500, 500, 990, 500, 674), clipped
    assert names == ["J1", "J4", "J6"], names
    same, none_clipped = ap.clip_positions(ap.HOME_POSITIONS)
    assert same == ap.HOME_POSITIONS and none_clipped == [], (same, none_clipped)
    print("用例3 限位裁剪 ✓")

    # ---- 用例 4：正常执行，4 步全部到位，rc=0，J6 以闭合收场 ----
    reset()
    rc = ap.run_plan(ap.plan_pick(8.0, 12.0, z=2.0), yes=True)
    assert rc == 0, rc
    assert len(FakeFlask.cmd_log) == 4, len(FakeFlask.cmd_log)
    assert FakeFlask.state[5] == ap.JAW_CLOSE, FakeFlask.state[5]
    print("用例4 正常执行（发 4 条命令，末 J6=%d）✓" % FakeFlask.state[5])

    # ---- 用例 5：固件回 ERR,2，必须 rc=4（不是静默成功）----
    reset()
    FakeFlask.fail_mode = "err"
    rc = ap.run_plan(ap.plan_pick(8.0, 12.0, z=2.0), yes=True)
    assert rc == 4, rc
    print("用例5 固件拒绝（rc=4）✓")

    # ---- 用例 6：wait_arrive 超时必须返回 False（不是傻等、不抛异常）----
    reset()
    FakeFlask.fail_mode = "stuck"
    FakeFlask.target = [500, 560, 560, 540, 500, 674]   # 偏离当前位置，让各关节需要走
    real_sleep = time.sleep
    ap.time.sleep = lambda s: None
    try:
        _, arrived = ap.wait_arrive(FakeFlask.target, timeout=0.5)
    finally:
        ap.time.sleep = real_sleep
    assert arrived is False, arrived
    print("用例6 卡住超时（wait_arrive 返回 False）✓")

    # ---- 用例 7：不加 --yes，一条命令都不许发出去，rc=3 ----
    reset()
    rc = ap.run_plan(ap.plan_pick(8.0, 12.0, z=2.0), yes=False)
    assert rc == 3, rc
    assert len(FakeFlask.cmd_log) == 0, FakeFlask.cmd_log
    print("用例7 未确认（rc=3，0 条命令）✓")

    # ---- 用例 8：plan_home 与 HOME_POSITIONS 一致 ----
    h = ap.plan_home().steps[0]
    assert h.positions == ap.HOME_POSITIONS, h.positions
    print("用例8 plan_home 与固件零位一致 ✓")

    # ---- 用例 9：best_tilt 返回不越软限位的可行姿态 ----
    got = ap.best_tilt(-5.0, 25.0, 2.0)
    assert got is not None, "(-5, 25) 在 0~90° 内找不到可行姿态"
    bt, bp = got
    assert 0.0 <= bt <= 90.0 and bt > 0.0, bt
    for i, v in enumerate(bp):
        lo, hi = ap.SAFE_LIMITS[i]
        assert lo <= v <= hi, (i, v, lo, hi)
    print("用例9 best_tilt(-5,25)=%.0f°（不越软限位）✓" % bt)

    srv.shutdown()
    print("\nALL PASS（%s）" % ("9 组" if ok else "FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
