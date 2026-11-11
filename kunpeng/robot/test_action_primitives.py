#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_action_primitives.py —— 动作原语层的回归测试：不需要真机、不需要 Flask

大白话：用一个「假 Flask」冒充 5000 端口，它会算斜坡、会回 ERR、会卡住不动，
       我们拿这四种脾气各试一遍，确认动作原语层的每个分支都走对。
       为什么必须这么测：真机上一旦「合爪没合紧就抬起」，物体直接甩掉，
       而那种错在纸面上看不出来 —— 只有把程序跑起来才会露。

规范定义：这是 HTTP 层的测试替身（test double / fake server），
         实现与真实服务相同的接口契约，但状态变化由测试用例控制；
         四个用例分别覆盖正常路径与三条异常路径（拒收 / 卡死 / 未确认）。

🔴 为什么不用真 Flask + 真串口：真机测试会动臂、会掉串口、一次要十几秒，
   而且「卡住」这种故障没法按需制造。假 Flask 能在 3 秒内跑完全部分支。

用法：
  python3 test_action_primitives.py        # 四个用例，全部通过才返回 0
"""

import http.server
import json
import os
import sys
import threading
import time

# 同目录导入，保证测的就是仓库里那份代码，不是装到系统里的另一份
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import action_primitives as ap


class FakeFlask(http.server.BaseHTTPRequestHandler):
    """冒充 web_control.py 的两个接口：/api/cmd 与 /api/state。

    状态机故意做得比真的简单：每次 /api/state 让每个关节向目标走 300 µs，
    这样 wait_arrive 的轮询逻辑才会真的被触发（一步到位就测不出轮询）。
    """

    state = [1580, 1900, 2400, 700, 1450, 1500]
    target = list(state)
    fail_mode = None          # None | "err" | "stuck"
    cmd_log = []
    poll_count = 0

    def log_message(self, *a):
        pass                  # 静音，否则每个请求都刷一行，输出淹没重点

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
                # 模拟固件回 ERR,1（未知命令）：注意 ok 仍是 true，
                # 真实 /api/cmd 就是这样 —— ok:true 只代表串口写进去了。
                self._json({"ok": True, "resp": "ERR,1"})
                return
            if payload.startswith("JOINT,"):
                FakeFlask.target = [int(x) for x in payload.split(",")[1:]]
            self._json({"ok": True, "resp": "OK"})
            return

        if self.path == "/api/state":
            FakeFlask.poll_count += 1
            for i in range(6):
                if FakeFlask.fail_mode == "stuck" and i == 3:
                    continue                      # J4 永远不动，模拟舵机卡死
                d = FakeFlask.target[i] - FakeFlask.state[i]
                if abs(d) <= 300:
                    FakeFlask.state[i] = FakeFlask.target[i]
                else:
                    FakeFlask.state[i] += 300 if d > 0 else -300
            self._json({"ok": True, "pulses": list(FakeFlask.state)})
            return

        self._json({"ok": False, "error": "unknown path"})


def start_server():
    srv = http.server.ThreadingHTTPServer(("127.0.0.1", 18080), FakeFlask)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def reset():
    """每个用例开始前把假服务恢复到初始位，避免用例之间互相污染。"""
    FakeFlask.state = [1580, 1900, 2400, 700, 1450, 1500]
    FakeFlask.target = list(FakeFlask.state)
    FakeFlask.fail_mode = None
    FakeFlask.cmd_log = []
    FakeFlask.poll_count = 0


def main():
    srv = start_server()
    ap.BASE_URL = "http://127.0.0.1:18080"
    ok = True

    # ---- 用例 1：正常 4 步，必须全部到位且以「夹紧」收场 ----
    reset()
    rc = ap.run_plan(ap.plan_pick(10.0, 18.0), yes=True)
    print("\n用例1 正常: rc=%d, 发 %d 条命令, 轮询 %d 次, 末状态 %s"
          % (rc, len(FakeFlask.cmd_log), FakeFlask.poll_count, FakeFlask.state))
    if rc != 0:
        ok = False
        print("  ✗ 期望 rc=0")
    if len(FakeFlask.cmd_log) != 4:
        ok = False
        print("  ✗ 期望发 4 条 JOINT，实际 %d" % len(FakeFlask.cmd_log))
    if FakeFlask.state[5] != ap.JAW_CLOSE:
        ok = False
        print("  ✗ 结束时 J6 应为闭合 %d，实际 %d" % (ap.JAW_CLOSE, FakeFlask.state[5]))

    # ---- 用例 2：固件回 ERR,1，必须抛异常并返回 4（不是静默成功）----
    reset()
    FakeFlask.fail_mode = "err"
    rc = ap.run_plan(ap.plan_pick(10.0, 18.0), yes=True)
    print("\n用例2 固件拒绝: rc=%d（期望 4）" % rc)
    if rc != 4:
        ok = False
        print("  ✗ 期望 rc=4")

    # ---- 用例 3：某一路卡住不动，必须超时返回 2（不是傻等）----
    reset()
    FakeFlask.fail_mode = "stuck"
    t0 = time.time()
    rc = ap.run_plan(ap.plan_pick(10.0, 18.0), yes=True)
    print("\n用例3 卡住: rc=%d（期望 2），耗时 %.1fs" % (rc, time.time() - t0))
    if rc != 2:
        ok = False
        print("  ✗ 期望 rc=2")

    # ---- 用例 4：不加 --yes，一条命令都不许发出去 ----
    reset()
    rc = ap.run_plan(ap.plan_pick(10.0, 18.0), yes=False)
    print("\n用例4 未确认: rc=%d, 发命令 %d 条（期望 3 和 0）"
          % (rc, len(FakeFlask.cmd_log)))
    if rc != 3 or len(FakeFlask.cmd_log) != 0:
        ok = False
        print("  ✗ 未加 --yes 竟然发了命令")

    srv.shutdown()
    print("\n" + ("ALL PASS" if ok else "FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
