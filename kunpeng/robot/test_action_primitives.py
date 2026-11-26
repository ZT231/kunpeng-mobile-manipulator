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


# J6 电流—脉宽的实测曲线（11-16，3.8cm 喷雾瓶，跨 PWM 周期采样后测的）。
# 注意 1510 的 81 与 1530 的 59 之间那个「回落」是真实数据里的毛刺 ——
# 正是它逼出了「弱判据只记候选、强判据才停爪」的两级设计，所以测试要照抄它。
JAW_CURVE = [(1450, 37), (1470, 32), (1490, 32), (1510, 81),
             (1530, 59), (1550, 124), (1570, 162), (1600, 189)]


def fake_current(j6, base=27):
    """把 J6 脉宽映射成 mA，模拟真机上的电流曲线。"""
    if j6 < JAW_CURVE[0][0]:
        return base
    for (p0, v0), (p1, v1) in zip(JAW_CURVE, JAW_CURVE[1:]):
        if p0 <= j6 <= p1:
            t = (j6 - p0) / float(p1 - p0)
            return int(round(v0 + t * (v1 - v0)))
    return int(round(189 + (j6 - 1600) * 1.35))


class FakeFlask(http.server.BaseHTTPRequestHandler):
    """冒充 web_control.py 的两个接口：/api/cmd 与 /api/state。

    状态机故意做得比真的简单：每次 /api/state 让每个关节向目标走 300 µs，
    这样 wait_arrive 的轮询逻辑才会真的被触发（一步到位就测不出轮询）。
    """

    state = [1580, 1900, 2400, 700, 1450, 1500]
    target = list(state)
    fail_mode = None          # None | "err" | "stuck"
    cur_mode = None           # None | "curve" | "low" | "na"
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
                # 模拟固件回 ERR,2（未知命令）：注意 ok 仍是 true，
                # 真实 /api/cmd 就是这样 —— ok:true 只代表串口写进去了。
                self._json({"ok": True, "resp": "ERR,2"})
                return
            if payload.startswith("JOINT,"):
                FakeFlask.target = [int(x) for x in payload.split(",")[1:]]
                self._json({"ok": True, "resp": "OK"})
                return
            if payload == "CUR":
                # 用电平模拟 J6 的电流曲线：让 grip_until_contact 的判据
                # 能在这台「假臂」上被真正走一遍，而不是只测语法。
                if FakeFlask.cur_mode == "na":
                    self._json({"ok": True, "resp": "CUR,NA"})
                    return
                if FakeFlask.cur_mode == "low":
                    self._json({"ok": True, "resp": "CUR,20"})
                    return
                self._json({"ok": True,
                            "resp": "CUR,%d" % fake_current(FakeFlask.target[5])})
                return
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
    FakeFlask.cur_mode = None
    FakeFlask.cmd_log = []
    FakeFlask.poll_count = 0


def main():
    srv = start_server()
    ap.BASE_URL = "http://127.0.0.1:18080"
    ok = True

    # ---- 用例 1：正常 4 步，必须全部到位且以「夹紧」收场 ----
    reset()
    rc = ap.run_plan(ap.plan_pick(10.0, 18.0, compensate=False), yes=True)
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

    # ---- 用例 2：固件回 ERR,2（未知命令），必须抛异常并返回 4（不是静默成功）----
    reset()
    FakeFlask.fail_mode = "err"
    rc = ap.run_plan(ap.plan_pick(10.0, 18.0, compensate=False), yes=True)
    print("\n用例2 固件拒绝: rc=%d（期望 4）" % rc)
    if rc != 4:
        ok = False
        print("  ✗ 期望 rc=4")

    # ---- 用例 3：某一路卡住不动，必须超时返回 2（不是傻等）----
    reset()
    FakeFlask.fail_mode = "stuck"
    t0 = time.time()
    rc = ap.run_plan(ap.plan_pick(10.0, 18.0, compensate=False), yes=True)
    print("\n用例3 卡住: rc=%d（期望 2），耗时 %.1fs" % (rc, time.time() - t0))
    if rc != 2:
        ok = False
        print("  ✗ 期望 rc=2")

    # ---- 用例 4：不加 --yes，一条命令都不许发出去 ----
    reset()
    rc = ap.run_plan(ap.plan_pick(10.0, 18.0, compensate=False), yes=False)
    print("\n用例4 未确认: rc=%d, 发命令 %d 条（期望 3 和 0）"
          % (rc, len(FakeFlask.cmd_log)))
    if rc != 3 or len(FakeFlask.cmd_log) != 0:
        ok = False
        print("  ✗ 未加 --yes 竟然发了命令")

    # ---- 用例 5：电流引导合爪 —— 落点必须落在「抬得动」的区间 ----
    # 期望 1550~1575：1550 是实测抬得起来的点，1575 是基线漂移时最多偏一个细步。
    reset()
    FakeFlask.state = [1691, 1279, 2320, 1850, 1450, 950]
    FakeFlask.target = list(FakeFlask.state)
    FakeFlask.cur_mode = "curve"
    real_sleep = time.sleep
    ap.time.sleep = lambda s: None            # 去掉真机上的等待，测试才跑得快
    try:
        got = ap.grip_until_contact((1691, 1279, 2320, 1850, 1450),
                                    start=950, verbose=False)
    finally:
        ap.time.sleep = real_sleep
    print("\n用例5 曲线合爪: J6=%s，%s mA（期望 1550~1575）"
          % (got[0] if got else None, got[1] if got else "-"))
    if not (got and 1550 <= got[0] <= 1575):
        ok = False
        print("  ✗ 落点不在 1550~1575")

    # ---- 用例 6：夹空 —— 电流全程不抬升，必须 rc=6 且不许一路合到底 ----
    reset()
    FakeFlask.state = [1691, 1279, 2320, 1850, 1450, 950]
    FakeFlask.target = list(FakeFlask.state)
    FakeFlask.cur_mode = "low"
    old_argv = sys.argv
    sys.argv = ["action_primitives.py", "--yes", "grip"]
    ap.time.sleep = lambda s: None
    try:
        rc = ap.main()
    finally:
        ap.time.sleep = real_sleep
        sys.argv = old_argv
    print("\n用例6 夹空: rc=%d（期望 6），最后 J6=%d（须 ≤ %d）"
          % (rc, FakeFlask.target[5], ap.GRIP_MAX_JAW))
    if rc != 6:
        ok = False
        print("  ✗ 期望 rc=6")
    if FakeFlask.target[5] > ap.GRIP_MAX_JAW:
        ok = False
        print("  ✗ 夹空时合过了 GRIP_MAX_JAW，会顶到机械限位")

    # ---- 用例 7：$CUR 回 NA —— 必须报错中止（rc=4），不能当成 0mA 继续夹 ----
    reset()
    FakeFlask.state = [1691, 1279, 2320, 1850, 1450, 950]
    FakeFlask.target = list(FakeFlask.state)
    FakeFlask.cur_mode = "na"
    sys.argv = ["action_primitives.py", "--yes", "grip"]
    ap.time.sleep = lambda s: None
    try:
        rc = ap.main()
    finally:
        ap.time.sleep = real_sleep
        sys.argv = old_argv
    print("\n用例7 CUR,NA: rc=%d（期望 4）" % rc)
    if rc != 4:
        ok = False
        print("  ✗ 期望 rc=4")

    # ---- 用例 8：grip --dry-run —— 只读电流，一条 JOINT 都不许发 ----
    reset()
    FakeFlask.state = [1691, 1279, 2320, 1850, 1450, 950]
    FakeFlask.target = list(FakeFlask.state)
    FakeFlask.cur_mode = "curve"
    sys.argv = ["action_primitives.py", "--dry-run", "grip"]
    ap.time.sleep = lambda s: None
    try:
        rc = ap.main()
    finally:
        ap.time.sleep = real_sleep
        sys.argv = old_argv
    joints = [c for c in FakeFlask.cmd_log if c.startswith("JOINT,")]
    print("\n用例8 grip --dry-run: rc=%d（期望 0），发 JOINT %d 条（期望 0）"
          % (rc, len(joints)))
    if rc != 0 or joints:
        ok = False
        print("  ✗ dry-run 竟然动了臂")

    # ---- 用例 9：--lift 60 —— 夹住后 J2 必须真的 +60，且 J6 保持夹持值不松 ----
    reset()
    FakeFlask.state = [1691, 1279, 2320, 1850, 1450, 950]
    FakeFlask.target = list(FakeFlask.state)
    FakeFlask.cur_mode = "curve"
    sys.argv = ["action_primitives.py", "--yes", "grip", "--lift", "60"]
    ap.time.sleep = lambda s: None
    try:
        rc = ap.main()
    finally:
        ap.time.sleep = real_sleep
        sys.argv = old_argv
    print("\n用例9 --lift 60: rc=%d（期望 0），末状态 J2=%d（期望 1339）J6=%d（期望 ≥1550）"
          % (rc, FakeFlask.target[1], FakeFlask.target[5]))
    if rc != 0:
        ok = False
        print("  ✗ 期望 rc=0")
    if FakeFlask.target[1] != 1339:
        ok = False
        print("  ✗ J2 没有 +60")
    if FakeFlask.target[5] < 1550:
        ok = False
        print("  ✗ 抬起时 J6 被改了（松爪会把东西掉了）")

    # ---- 用例 10：--from 离当前位置太远时必须夹回（19:2x 实机就栽在 1430 的瞬态上）----
    reset()
    FakeFlask.state = [1691, 1279, 2320, 1850, 1450, 950]
    FakeFlask.target = list(FakeFlask.state)
    FakeFlask.cur_mode = "curve"
    sys.argv = ["action_primitives.py", "--yes", "grip", "--from", "1430"]
    ap.time.sleep = lambda s: None
    try:
        rc = ap.main()
    finally:
        ap.time.sleep = real_sleep
        sys.argv = old_argv
    first_joint = next((c for c in FakeFlask.cmd_log if c.startswith("JOINT,")), "")
    first_j6 = int(first_joint.split(",")[6]) if first_joint else -1
    print("\n用例10 --from 1430（当前 950）: rc=%d，第一条 JOINT 的 J6=%d（须 ≤ %d）"
          % (rc, first_j6, 950 + ap.GRIP_MAX_FROM_JUMP))
    if first_j6 < 0 or first_j6 > 950 + ap.GRIP_MAX_FROM_JUMP:
        ok = False
        print("  ✗ 起点没有被夹回，会上演「启动冲击假冒夹住」")

    # ---- 用例 11：dynamic_grip —— 合爪改由电流决定，且"抬起"要沿用实际夹持值 ----
    reset()
    FakeFlask.cur_mode = "curve"
    ap.time.sleep = lambda s: None
    try:
        rc = ap.run_plan(ap.plan_pick(10.0, 18.0, compensate=False),
                         yes=True, dynamic_grip=True)
    finally:
        ap.time.sleep = real_sleep
    jaw_end = FakeFlask.target[5]
    print("\n用例11 dynamic_grip: rc=%d（期望 0），末 J6=%d（期望 1540~1620）"
          % (rc, jaw_end))
    if rc != 0:
        ok = False
        print("  ✗ 期望 rc=0")
    if not (1540 <= jaw_end <= 1620):
        ok = False
        print("  ✗ 夹持值没有落在电流判据给出的区间")

    # ---- 用例 12：dynamic_grip 判夹空时必须 rc=7 且**不抬臂** ----
    reset()
    FakeFlask.cur_mode = "low"
    p12 = ap.plan_pick(10.0, 18.0, compensate=False)
    j2_down = p12.steps[1].pulses[1]      # 下降那步的 J2
    j2_top = p12.steps[3].pulses[1]       # 抬起那步的 J2
    ap.time.sleep = lambda s: None
    try:
        rc = ap.run_plan(p12, yes=True, dynamic_grip=True)
    finally:
        ap.time.sleep = real_sleep
    print("\n用例12 dynamic_grip 夹空: rc=%d（期望 7），末 J2=%d（下降位 %d / 抬起位 %d）"
          % (rc, FakeFlask.target[1], j2_down, j2_top))
    if rc != 7:
        ok = False
        print("  ✗ 期望 rc=7")
    if FakeFlask.target[1] != j2_down:
        ok = False
        print("  ✗ 夹空之后还执行了抬臂（J2 到了抬起位）")

    # ---- 用例 13：J1 只由方位角决定（C 方案的地基）----
    # 期望：示教工位 (-5.26,25) → 1691（与当初 IK 给的 J1 一致）；
    #       正前方 (0,25) → 1580（等于初始位的 J1）；两者之间单调。
    try:
        j1_a, _, az_a = ap.j1_pos_from_xy(-5.26, 25.0)
        j1_b, _, az_b = ap.j1_pos_from_xy(0.0, 25.0)
        j1_c, _, az_c = ap.j1_pos_from_xy(-4.19, 23.65)
    except Exception as exc:                     # noqa: BLE001
        j1_a = j1_b = j1_c = -1
        print("  ✗ j1_pos_from_xy 抛异常：%s" % exc)
    print("\n用例13 J1 方位角: (-5.26,25)→%d（期望 1691）｜(0,25)→%d（期望 1580）"
          "｜(-4.19,23.65)→%d（期望 1673~1675）" % (j1_a, j1_b, j1_c))
    if j1_a != 1691 or j1_b != 1580 or not (1673 <= j1_c <= 1675):
        ok = False
        print("  ✗ J1 方位角映射对不上")

    # 方位角补偿：视觉 (−2.64,23.79) 给 1639；尺子真值 (−4.4,26.8) 是 1667。
    # 补 +2.98° 之后必须等于尺子那条 ⇒ 20:0x 实测的那 2.98° 偏差就是这么用的。
    j1_nb, _, _ = ap.j1_pos_from_xy(-2.64, 23.79)
    j1_wb, _, _ = ap.j1_pos_from_xy(-2.64, 23.79, az_bias_deg=2.98)
    j1_truth, _, _ = ap.j1_pos_from_xy(-4.4, 26.8)
    print("用例13b 方位角补偿: 不补=%d｜补2.98°=%d｜尺子真值=%d（三者应满足 不补<补=真值）"
          % (j1_nb, j1_wb, j1_truth))
    if not (j1_nb < j1_wb == j1_truth):
        ok = False
        print("  ✗ 方位角补偿没把 J1 拉到尺子那条")

    # ---- 用例 14：配方 + J1 偏移 —— 只动 J1，别把其它关节带跑 ----
    try:
        rec = ap.load_recipes()["recipes"]["bottle_38mm"]
        p0 = ap.build_recipe_plan(rec, j1_offset=0)
        p1 = ap.build_recipe_plan(rec, j1_offset=-18)
        same = all(a.pulses[1:] == b.pulses[1:]
                   for a, b in zip(p0.steps, p1.steps))
        shifted = all(b.pulses[0] == a.pulses[0] - 18
                      for a, b in zip(p0.steps, p1.steps))
        print("\n用例14 配方 J1 偏移: 步数 %d/%d，只改 J1=%s，J1 真偏移=%s"
              % (len(p0.steps), len(p1.steps), same, shifted))
        if not (same and shifted):
            ok = False
            print("  ✗ 偏移把别的关节也改了，或 J1 没偏移")
    except Exception as exc:                     # noqa: BLE001
        ok = False
        print("\n用例14 配方 J1 偏移: ✗ 抛异常 %s" % exc)

    srv.shutdown()
    print("\n" + ("ALL PASS" if ok else "FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
