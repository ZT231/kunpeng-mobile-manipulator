#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_esp32_link.py · ESP32-S3 链路「四层硬判据」自动判定（离线、零第三方依赖）

用途（一句话）：明天在教室烧完 ESP32，把「串口 monitor 日志」「鲲鹏 /root/gw.log」
「PC 侧 nc 探活输出」三个文本喂给本脚本，它自动逐层打勾，避免人眼在几百行日志里漏看、
或把「看着像通了」误当成实机验收通过。

🔴 边界（必须记住）：
  1. 本脚本只读文本文件，**不改任何工程文件、不发任何串口命令、不动机械臂**；
  2. 四层全绿只证明「ESP32 -> Wi-Fi -> 鲲鹏网关」链路通，**不等于**语音识别/抓取/ESP-SR 已实现；
  3. 判定通过 ≠ 抬档。仍需人工把结论填进
     `docs/明日回教室-ESP32照跑清单.md` 第八节记录表并回报，才统一订文档；`esp32/` 档位仍 🔶。

判据出处：`docs/明日回教室-ESP32照跑清单.md` 第五节 D 表（4 层）+ 第八节记录表。

--- README 三行法 ---
① 用了什么语法/技术：纯标准库 `argparse`（命令行参数）/ `pathlib`（读文件）/
   `re`（正则找 IP 与 send 字节数）/ `json`（逐行解析网关日志）/ 4 个纯函数 `check_layerN()` /
   `_emit()` 做 locale 无关输出。**零第三方包、零网络、零串口**。
② 为什么这么设计：判据写成「输入文本 -> (是否通过, 说明)」的纯函数，读文件单独隔离，
   所以 `--self-test` 用内置假日志就能自证脚本本身可靠；`_emit()` 兜底 ASCII stdout，
   是复用 `voice_gateway.py` 11-14 在鲲鹏板 `LANG=C` 下踩过的 `UnicodeEncodeError` 坑。
③ 不这么写会出什么问题：靠人眼翻串口日志找 4 条证据，最容易把 `$ERR,2*5B` 看成 `$OK,...`、
   漏看「烧的是旧固件」（打印 `$ASR,mic_ok*50`）、或没发现网关被误开成 `--apply`；
   这三类误判都会让「编译通过 / 链路看着通」被错当成实机验收通过去抬档，正是本项目反复强调的静默失效。

用法：
    python3 check_esp32_link.py --self-test
    python3 check_esp32_link.py --uart-log uart.txt --gw-log gw.log --probe-out probe.txt
    python3 check_esp32_link.py --uart-log uart.txt --gw-log gw.log --probe-text '$OK,ESTOP*75'
"""

import argparse
import json
import re
import sys
from pathlib import Path

# ---- 期望看到的应答帧（CK 由 docs/上板验收-语音网关阶段2.md 实算过）----
OK_ESTOP = "$OK,ESTOP*75"
OK_RESUME = "$OK,RESUMED*75"
ERR2 = "$ERR,2*5B"

# 旧固件的打印：新固件自检帧是 $ESTOP*5D / $RESUME*19，看到这两个词说明烧错副本
OLD_FIRMWARE_MARKS = ("$ASR,mic_ok", "$ASR,home")


def _emit(text):
    """把输出做成 locale 无关。

    大白话：终端如果只认 ASCII（板端 LANG=C / PYTHONIOENCODING=ascii），
    直接 print 中文会抛 UnicodeEncodeError，脚本当场崩、后面的判定全看不到。
    规范定义：Python 3 的 TextIOWrapper 按 sys.stdout.encoding 编码字符，
    编码不含该字符时抛 UnicodeEncodeError；这里用 backslashreplace 降级成 \\uXXXX 转义。
    """
    data = str(text)
    try:
        print(data, flush=True)
    except UnicodeEncodeError:
        print(data.encode("ascii", "backslashreplace").decode("ascii"), flush=True)


def read_log(path, label):
    """读一个日志文本。文件不存在 / 读不了都返回 (空串, 提示)，不抛异常打断另外 3 层判定。"""
    if not path:
        return "", "%s：未提供" % label
    p = Path(path)
    if not p.is_file():
        return "", "%s：文件不存在 %s" % (label, path)
    try:
        return p.read_text(encoding="utf-8", errors="replace"), ""
    except OSError as exc:
        return "", "%s：读取失败 %s" % (label, exc)


def check_layer1(uart_text):
    """第 1 层：ESP32 串口里必须真的连上鲲鹏 :9000，且两条帧都发出去了。"""
    m = re.search(r"TCP connected\s*->\s*(\d+(?:\.\d+){3}):9000", uart_text)
    sends = re.findall(r"send\s+(\d+)\s+bytes", uart_text)
    if not m:
        return False, "没找到 'TCP connected -> <IP>:9000'（Wi-Fi 没连上 / GATEWAY_HOST 填错 / 网关没起）"
    ok = len(sends) >= 2
    detail = "连到 %s:9000，发出 %d 条帧（字节数 %s）" % (
        m.group(1), len(sends), "/".join(sends) if sends else "无")
    if not ok:
        detail += "；🔴 send 记录不足 2 条"
    return ok, detail


def check_layer2(uart_text):
    """第 2 层：串口里必须有两条正确的应答帧，且不能是被拒 / 旧固件。"""
    if ERR2 in uart_text:
        return False, "出现 %s：帧没被网关接受（多半发成了 mic_ok / home，词表只认中文）" % ERR2
    old = [x for x in OLD_FIRMWARE_MARKS if x in uart_text]
    if old:
        return False, "出现旧固件打印 %s ⇒ 烧的是旧副本或没重新 build" % ", ".join(old)
    has_e = OK_ESTOP in uart_text
    has_r = OK_RESUME in uart_text
    if not (has_e and has_r):
        missing = [x for x, got in ((OK_ESTOP, has_e), (OK_RESUME, has_r)) if not got]
        return False, "缺少应答帧：%s" % ", ".join(missing)
    return True, "两条应答帧都在，且无 %s / 旧固件打印" % ERR2


def check_layer3(gw_text):
    """第 3 层：鲲鹏网关日志里必须有 STOP/RESUME 两条记录，且 apply=false（dry-run）。"""
    hits = []
    applied = []
    for line in gw_text.splitlines():
        line = line.strip()
        if not line.startswith("{"):       # 网关日志每行是一条 JSON；其它行直接跳过
            continue
        try:
            rec = json.loads(line)
        except ValueError:
            continue
        if not isinstance(rec, dict):
            continue
        if rec.get("payload") in ("STOP", "RESUME"):
            hits.append((rec.get("payload"), rec.get("apply")))
            if rec.get("apply") is True:
                applied.append(rec.get("payload"))
    if applied:
        return False, "日志出现 apply=true（%s）⇒ 这不是 dry-run，先停掉 --apply 再验" % ",".join(applied)
    got_stop = any(p == "STOP" and a is False for p, a in hits)
    got_resume = any(p == "RESUME" and a is False for p, a in hits)
    if not (got_stop and got_resume):
        return False, "缺 dry-run 记录：需 payload=STOP 与 RESUME 各一条且 apply=false，实得 %s" % (hits or "空")
    return True, "payload=STOP / RESUME 各一条，apply=false（确认没真下发）"


def check_layer4(probe_text):
    """第 4 层：PC 侧（外部主机）也能连上，证明网关不是只 bind 127.0.0.1 自连。"""
    if not probe_text.strip():
        return False, "没粘 PC 侧 nc 的输出（B4 那两枪）"
    if "$ERR" in probe_text:
        return False, "外部探活回了 $ERR（帧没被接受）"
    oks = [x for x in (OK_ESTOP, OK_RESUME) if x in probe_text]
    if not oks:
        return False, "外部探活没看到 $OK,... ⇒ 多半网关没带 --host 0.0.0.0 或防火墙挡了"
    return True, "外部主机也收到 %s（不是 127.0.0.1 自连）" % ", ".join(oks)


def evaluate(uart_text, gw_text, probe_text):
    """纯函数：把三份文本变成 4 个 (bool, 说明)。不打印、不读盘，方便 --self-test 直接断言。"""
    return [
        check_layer1(uart_text),
        check_layer2(uart_text),
        check_layer3(gw_text),
        check_layer4(probe_text),
    ]


LAYER_NAMES = (
    "1 ESP32 侧 TCP connected",
    "2 两条应答帧 / 非旧固件",
    "3 鲲鹏网关日志 dry-run",
    "4 PC 侧外部探活",
)


def run(uart_text, gw_text, probe_text, notes):
    results = evaluate(uart_text, gw_text, probe_text)
    _emit("=" * 72)
    _emit("ESP32-S3 链路四层硬判据 · 自动判定（只判据，不抬档）")
    _emit("=" * 72)
    for note in notes:
        _emit("🔴 输入提示：%s" % note)
    if notes:
        _emit("")
    for name, (ok, detail) in zip(LAYER_NAMES, results):
        _emit("%s 第 %s：%s" % ("✅ PASS" if ok else "🔴 FAIL", name, detail))
    npass = sum(1 for ok, _ in results if ok)
    _emit("")
    _emit("RESULT: PASS=%d FAIL=%d" % (npass, 4 - npass))
    _emit("🔴 判定通过 ≠ 抬档：把结果填进 docs/明日回教室-ESP32照跑清单.md 第八节记录表并回报，")
    _emit("   才统一订文档；esp32/ 已烧录实机（TCP 9000/采音），语音入口整体档位仍 🔶（识别未接）。")
    return 0 if npass == 4 else 1


# ---- 内置假日志：给 --self-test 用，证明「脚本自己」没写错 ----
GOOD_UART = """I (123) voice_main: ==== ESP32-S3 语音入口骨架启动（含 TCP 上行自检） ====
I (456) wifi_sta: Got IP 192.168.1.23
I (789) voice_main: Wi-Fi OK, is_connected=1；网关目标 192.168.1.23:9000
I (800) tcp_client: TCP connected -> 192.168.1.23:9000
I (810) tcp_client: send 11 bytes: $ESTOP*5D
I (820) tcp_client: recv 14 bytes: $OK,ESTOP*75
I (830) tcp_client: send 12 bytes: $RESUME*19
I (840) tcp_client: recv 16 bytes: $OK,RESUMED*75
I (850) voice_main: 中文帧默认关闭（SEND_CN_HOME_FRAME=0），明天联调再开
"""

GOOD_GW = (
    '{"accepted": true, "payload": "STOP", "apply": false, "sent": false, "dry_run_note": "本该下发 STOP"}\n'
    '{"accepted": true, "payload": "RESUME", "apply": false, "sent": false, "dry_run_note": "本该下发 RESUME"}\n'
)

GOOD_PROBE = "$OK,ESTOP*75\n$OK,RESUMED*75\n"


def self_test():
    """5 组用例：1 组全绿 + 4 组典型翻车，断言每层判定都符合预期。"""
    cases = [
        ("全绿（期望 4/4 全过）", GOOD_UART, GOOD_GW, GOOD_PROBE, 4),
        ("烧成旧固件：串口出现 $ASR,mic_ok*50（第 2 层该 FAIL）",
         GOOD_UART + "I (999) voice_main: [本地封帧] $ASR,mic_ok*50\n", GOOD_GW, GOOD_PROBE, 3),
        ("网关没起：gw.log 为空（第 3 层该 FAIL）",
         GOOD_UART, "", GOOD_PROBE, 3),
        ("网关误开 --apply：日志 apply=true（第 3 层该 FAIL）",
         GOOD_UART, GOOD_GW.replace('"apply": false', '"apply": true'), GOOD_PROBE, 3),
        ("收到 $ERR,2*5B（第 2 层该 FAIL）",
         GOOD_UART.replace(OK_ESTOP, ERR2), GOOD_GW, GOOD_PROBE, 3),
    ]
    failed = 0
    for name, uart, gw, probe, expect in cases:
        results = evaluate(uart, gw, probe)
        npass = sum(1 for ok, _ in results if ok)
        ok = (npass == expect)
        if not ok:
            failed += 1
        _emit("%s %s  实得 PASS=%d（期望 %d）" % ("✅" if ok else "🔴", name, npass, expect))
    _emit("")
    _emit("SELF_TEST %s  cases=%d failed=%d" % ("PASS" if failed == 0 else "FAIL", len(cases), failed))
    return 0 if failed == 0 else 1


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="ESP32-S3 链路四层硬判据自动判定（只读日志，不改工程、不动臂）")
    ap.add_argument("--uart-log", help="ESP32 串口 monitor 输出文本文件")
    ap.add_argument("--gw-log", help="鲲鹏板 /root/gw.log 文本文件")
    ap.add_argument("--probe-out", help="PC 侧 nc 探活输出文本文件")
    ap.add_argument("--probe-text", help="直接粘一行探活应答（$ 要用单引号包住，别被 shell 展开）")
    ap.add_argument("--self-test", action="store_true", help="用内置假日志自证脚本可靠，零硬件")
    args = ap.parse_args(argv)

    if args.self_test:
        return self_test()

    uart, n1 = read_log(args.uart_log, "串口日志")
    gw, n2 = read_log(args.gw_log, "网关日志")
    probe, n3 = read_log(args.probe_out, "外部探活输出")
    if args.probe_text:
        probe = (probe + "\n" + args.probe_text).strip()
        n3 = ""
    notes = [n for n in (n1, n2, n3) if n]
    return run(uart, gw, probe, notes)


if __name__ == "__main__":
    sys.exit(main())
