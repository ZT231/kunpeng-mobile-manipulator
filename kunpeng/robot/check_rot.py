#!/usr/bin/env python3
"""不碰 X、不碰连杆尺寸，直接量"模型转了多少度 vs 相机实际转了多少度"。

概念（大白话）：
  手眼标定里，A（手臂）和 B（相机）是同一个物理转动在两个坐标系里的两种写法，
  所以**两者的转角必须一模一样**——X 只是换个坐标系看，不会改变转了多少度。
  于是只要把每一对的转角列出来对比，就能判断"模型里的关节角准不准"：
  相机实际转 29° 而模型只写 18°，那就说明脉宽↔角度映射少了 1.6 倍。

规范定义：
  A = T_g2b(j)^-1·T_g2b(i)，B = T_t2c(j)·T_t2c(i)^-1，X 满足 A·X = X·B。
  由 A = X·B·X^-1 ⇒ trace(A) = trace(B) ⇒ θ_A = θ_B（相似矩阵迹相同）。
  故 θ_B/θ_A 与 X 无关，是纯粹的"关节角标定误差"测量。

用法（板上）：
    cd ~/kunpeng_vision && python3 check_rot.py samples.json
"""
import json
import math
import sys

import numpy as np


def rot_angle_deg(R):
    """旋转矩阵 -> 转角（度）。θ = arccos((tr R − 1)/2)。"""
    c = (float(np.trace(R)) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "samples.json"
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    S = data["samples"]
    N = len(S)
    Rg = [np.array(s["R_gripper2base"], float) for s in S]
    Rc = [np.array(s["R_target2cam"], float) for s in S]
    P = [[int(x) for x in s["pulses"]] for s in S]

    print("文件 %s：%d 组" % (path, N))
    print("")
    print("逐对比较相对转角（θ_A 来自 FK，θ_B 来自相机实测，两者本应相等）")
    print("  #i→#j  类型    θ_A(FK)   θ_B(相机)   θ_B/θ_A")
    j1_r, j4_r = [], []
    rows = []
    for i in range(N):
        for j in range(i + 1, N):
            # A = T_g2b(j)^-1·T_g2b(i)  ->  R_A = Rg[j]^T·Rg[i]
            RA = Rg[j].T @ Rg[i]
            # B = T_t2c(j)·T_t2c(i)^-1  ->  R_B = Rc[j]·Rc[i]^T
            RB = Rc[j] @ Rc[i].T
            ta, tb = rot_angle_deg(RA), rot_angle_deg(RB)
            if ta < 1.0:
                continue
            same_j1 = (P[i][0] == P[j][0])
            same_j4 = (P[i][3] == P[j][3])
            if same_j1 and not same_j4:
                kind = "J4"
                j4_r.append(tb / ta)
            elif same_j4 and not same_j1:
                kind = "J1"
                j1_r.append(tb / ta)
            else:
                continue
            rows.append((i + 1, j + 1, kind, ta, tb, tb / ta))
    for r in rows:
        print("  %2d→%2d  %-4s  %7.2f°  %8.2f°   %6.3f" % r)

    print("")
    print("汇总（θ_B/θ_A：1.00=映射准；1.6=实际转角是模型的 1.6 倍）")
    for tag, arr, nominal in (("只动 J4 的对", j4_r, 18.0), ("只动 J1 的对", j1_r, 12.0)):
        if not arr:
            continue
        a = np.array(arr)
        print("  %s：%d 对，比值 中位=%.3f 均值=%.3f 极差=%.3f"
              % (tag, len(a), float(np.median(a)), float(a.mean()),
                 float(a.max() - a.min())))
        print("    ⇒ 推断实际转角 = 名义 %.1f° × %.3f = %.1f°"
              % (nominal, float(np.median(a)), nominal * float(np.median(a))))
    print("")
    print("判读：比值稳定在 1.00±0.05 ⇒ 关节角映射没问题，残差另有原因（PNP/机械）；")
    print("      比值明显偏离 1 且稳定 ⇒ 脉宽↔角度映射少了这个倍数；")
    print("      比值本身散得很开 ⇒ 相机没刚性固定在 J4 上（支架松动/回差）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
