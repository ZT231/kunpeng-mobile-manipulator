#!/usr/bin/env python3
"""诊断 samples.json：手眼标定那 16 mm 残差到底来自哪几组数据。

🔴🔴🔴 待迁移（2026-11-26）🔴🔴🔴
本工具读旧 samples.json 的 `pulses` 字段（旧 PWM 臂 µs 标定数据）。总线舵机迁移后
samples 字段应改为 `positions`（0~1000），且需装车后用总线臂重新采集。旧样本的
诊断结论不适用于总线臂，本工具待重新采集后同步迁移。
🔴🔴🔴 待迁移（2026-11-26）🔴🔴🔴

概念（大白话）：solve 只告诉你"这批数据自己打架"，但没说是哪几张照片打架。
这个脚本把 12 组拆开逐一审，找出"谁在撒谎"。

规范定义：给定 12 组 (T_g2b_i, T_t2c_i)，AX=XB 的平移方程
  (R_A - I)·t_X = s·R_X·t_B - t_A ，其中 A = T_g2b_j^-1·T_g2b_i、B = T_t2c_j·T_t2c_i^-1。
  该方程组超定且有矛盾时，残差不会均匀分布，而是集中在少数样本上（离群点）。
  本脚本用 leave-one-out、连续子段、pair 残差分摊、标定板基座系位置四种方式定位离群样本。

用法（板上）：
    cd ~/kunpeng_vision && python3 diagnose_he.py samples.json
"""
import itertools
import json
import sys

import numpy as np

sys.path.insert(0, ".")
import handeye_calib as hc


def pose(R, t):
    """4x4 齐次矩阵：旋转 R + 平移 t。"""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, float).reshape(3)
    return T


def load(path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    S = data["samples"]
    Rg = [np.array(s["R_gripper2base"], float) for s in S]
    tg = [np.array(s["t_gripper2base"], float).reshape(3, 1) for s in S]
    Rc = [np.array(s["R_target2cam"], float) for s in S]
    tc = [np.array(s["t_target2cam"], float).reshape(3, 1) for s in S]
    return data, S, Rg, tg, Rc, tc


def fit(idx, Rg, tg, Rc, tc):
    """对指定下标子集重解 AX=XB，返回 (R_X, t_X, diag)。"""
    dg = {}
    R, t = hc.solve_handeye_axxb(
        [Rg[i] for i in idx], [tg[i] for i in idx],
        [Rc[i] for i in idx], [tc[i] for i in idx],
        verbose=False, diag=dg)
    return R, t.reshape(3), dg


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "samples.json"
    data, S, Rg, tg, Rc, tc = load(path)
    N = len(S)
    print("文件 %s：%d 组，square_m=%s" % (path, N, data.get("square_m")))

    # ---- 1) 原始观测量：|t| 与单位方向 ----
    print("")
    print("1) 每组 PnP 观测（看 |t| 是否随时间系统漂移、方向是否恒定）")
    print("   #  pulses                     |t|(m)   tx      ty      tz    单位方向")
    unit = []
    for i, s in enumerate(S):
        t = tc[i].reshape(3)
        u = t / max(float(np.linalg.norm(t)), 1e-12)
        unit.append(u)
        print("  %2d  %-26s %.4f  %+6.2f %+6.2f %+6.2f   %+5.3f %+5.3f %+5.3f"
              % (i + 1, ",".join(str(p) for p in s["pulses"]),
                 float(np.linalg.norm(tc[i])),
                 t[0] * 100.0, t[1] * 100.0, t[2] * 100.0, u[0], u[1], u[2]))
    unit = np.array(unit)
    sp = float(np.linalg.norm(unit.std(axis=0)))
    print("  单位方向散布 = %.4f（<0.01 表示棋盘只沿光轴远近变，方向没转）" % sp)

    if N < 4:
        print("")
        print("样本 < 4 组，只打印观测量；leave-one-out 与子段搜索需要更多样本。")
        return 0

    R_X, t_X, dg = fit(list(range(N)), Rg, tg, Rc, tc)
    print("")
    print("2) 全量：resid=%.2f mm  s=%.4f  axis_cond=%.3f  |t|=%.1f cm"
          % (dg["resid"] * 1000.0, dg["scale"], dg["axis_cond"],
             float(np.linalg.norm(t_X)) * 100.0))

    # ---- 3) 逐组剔除 ----
    print("")
    print("3) 逐组剔除（残差明显下降 ⇒ 被剔除那组是坏数据）")
    for k in range(N):
        idx = [i for i in range(N) if i != k]
        _, _, dk = fit(idx, Rg, tg, Rc, tc)
        tag = ""
        if dk["resid"] < dg["resid"] * 0.9:
            tag = "  <== 残差降到 %.0f%%" % (dk["resid"] / dg["resid"] * 100.0)
        print("  剔除 #%2d：resid=%6.2f mm  s=%.3f%s"
              % (k + 1, dk["resid"] * 1000.0, dk["scale"], tag))

    # ---- 4) 连续子段 ----
    print("")
    print("4) 连续子段搜索（某一段单独自洽 ⇒ 标定板在那一段之后被移动过）")
    good = []
    for a in range(N):
        for b in range(a + 3, N + 1):
            _, _, dk = fit(list(range(a, b)), Rg, tg, Rc, tc)
            if dk["resid"] < 5e-3:
                good.append((dk["resid"], a, b))
    good.sort()
    if good:
        for r, a, b in good[:8]:
            print("  #%2d..#%2d：resid=%.2f mm" % (a + 1, b, r * 1000.0))
    else:
        print("  没有长度 >= 3 的子段能把残差压到 5 mm 以内")

    # ---- 5) 每组分摊的 pair 残差 ----
    print("")
    print("5) 每组参与的 pair 平均残差（数值最大的几组就是离群组）")
    acc = np.zeros(N)
    cnt = np.zeros(N)
    for i, j in itertools.combinations(range(N), 2):
        A = hc._inv_pose(pose(Rg[j], tg[j])) @ pose(Rg[i], tg[i])
        B = pose(Rc[j], tc[j]) @ hc._inv_pose(pose(Rc[i], tc[i]))
        tB = B[:3, 3] * dg["scale"]
        r = (A[:3, :3] - np.eye(3)) @ t_X - (R_X @ tB - A[:3, 3])
        e = float(np.linalg.norm(r))
        acc[i] += e
        acc[j] += e
        cnt[i] += 1
        cnt[j] += 1
    order = np.argsort(-(acc / cnt))
    for rank, i in enumerate(order):
        print("  第 %d 差： #%2d  %.2f mm" % (rank + 1, i + 1, acc[i] / cnt[i] * 1000.0))

    # ---- 6) 标定板在基座系的位置 ----
    print("")
    print("6) 标定板在基座系的位置 P（若棋盘全程没动，12 组应当完全重合）")
    P = np.zeros((N, 3))
    X = pose(R_X, t_X)
    for i in range(N):
        Pi = pose(Rg[i], tg[i]) @ X @ pose(Rc[i], tc[i] * dg["scale"])
        P[i] = Pi[:3, 3] * 100.0
        print("  #%2d  P=[%+7.2f %+7.2f %+7.2f] cm" % (i + 1, P[i, 0], P[i, 1], P[i, 2]))
    Pm = P.mean(axis=0)
    print("  标准差：%.2f / %.2f / %.2f cm" % tuple(P.std(axis=0)))
    print("  极差：  %.2f / %.2f / %.2f cm" % tuple(P.max(axis=0) - P.min(axis=0)))
    print("")
    print("  相对均值的偏差 dP（12 个 dP 方向一致 ⇒ 棋盘真的在动；")
    print("                    方向随姿态乱指 ⇒ 是 PnP 尺度/焦距在漂，棋盘没动）")
    for i in range(N):
        d = P[i] - Pm
        print("  #%2d  dP=[%+6.2f %+6.2f %+6.2f] cm  |dP|=%5.2f"
              % (i + 1, d[0], d[1], d[2], float(np.linalg.norm(d))))
    print("")
    print("判读：极差 >1 cm 就是真有问题；三个数都 <0.3 cm 说明棋盘没动，")
    print("      残差来自角度映射（FK/脉宽↔角度），那是另一条修法。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
