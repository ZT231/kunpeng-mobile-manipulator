"""尺度自校正系数 s 的稳健性检验：0.7549 是"真的尺度差"还是"弱激励下的拟合噪声"。

为什么要这个工具（2026-11-11 实机踩的）：
solve 打出 s = 0.7549，字面意思是"相机侧平移比 FK 大 32.5%"。但 s 是
最小二乘顺手解出来的第 4 个未知数，它的可信度完全取决于这 12 组姿态的激励。
4 档 J1(±6°) × 3 档 J4(俯角 58/49/40°) 本来就是小运动弱激励，
s 完全有可能只是把 1~2 mm 的噪声拟合成一个数。
判断办法：不换硬件、不挪棋盘、不重采，只换样本子集重解。
  · 真的尺度差 ⇒ 任何子集都解出同一个 s（标准差小）
  · 拟合噪声   ⇒ 换个组合就差好几个百分点（标准差大）

⚠️ 先记一条走过的弯路（别再试）：
想用"|Δt| 模长比"做 X-无关的检验 —— 即比较末端原点走了多长 vs 棋盘在相机系里
走了多长。**不成立**。因为相机光心离末端原点 16.7 cm，末端一转，相机扫过的弧长
本来就比末端原点大（杠杆效应），比值是几何放大率而不是尺度误差。
合成数据自测坐实了这点：真值 k 只有 1.3247，这个比值却报出 J1 组 1.94、J4 组 2.59。
⇒ 只要涉及"位移大小"，就离不开 X，必须走完整 AX=XB。

概念（大白话）：同一把尺子量十次都能对上，才敢说尺子准；
每次换个组合就差好几个百分点，那说明量的其实是噪声。
规范定义：对 y = A·[t_X; s] 这个线性化模型，s 的可辨识性由设计矩阵
[ (R_A − I) , −R_X·t_B ] 的最小奇异值与噪声之比决定；用留一法与随机子集重解，
看 s 的经验标准差即可判定其稳健性（经验标准差 << |s − 1| 才算真实系统量）。

用法：
  python3 check_scale.py samples_fkfix.json
"""
import json
import sys

import numpy as np

sys.path.insert(0, ".")

from handeye_calib import solve_handeye_axxb


def _load(path):
    with open(path, "r", encoding="utf-8") as f:
        doc = json.load(f)
    S = doc["samples"]
    Rg = [np.array(s["R_gripper2base"], dtype=float) for s in S]
    tg = [np.array(s["t_gripper2base"], dtype=float).reshape(3, 1) for s in S]
    Rc = [np.array(s["R_target2cam"], dtype=float) for s in S]
    tc = [np.array(s["t_target2cam"], dtype=float).reshape(3, 1) for s in S]
    return Rg, tg, Rc, tc, doc


def _one(idx, Rg, tg, Rc, tc):
    """用指定下标的子集解一次，返回 (s, 残差RMS, axis_cond)。"""
    idx = list(idx)
    diag = {}
    solve_handeye_axxb([Rg[i] for i in idx], [tg[i] for i in idx],
                       [Rc[i] for i in idx], [tc[i] for i in idx],
                       verbose=False, diag=diag)
    return (float(diag.get("scale", 1.0)),
            float(diag.get("resid", 0.0)),
            float(diag.get("axis_cond", 0.0)))


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "samples_fkfix.json"
    Rg, tg, Rc, tc, doc = _load(path)
    n = len(Rg)
    print("文件 %s：%d 组样本；采集时 --square = %.5f m"
          % (path, n, float(doc.get("square_m") or 0.0)))

    s0, r0, ac0 = _one(range(n), Rg, tg, Rc, tc)
    print("")
    print("全量解：s = %.4f（相机侧平移 × %.4f）  残差 RMS = %.2f mm  axis_cond = %.3f"
          % (s0, s0, r0 * 1000.0, ac0))

    loo = [_one([i for i in range(n) if i != k], Rg, tg, Rc, tc)[0] for k in range(n)]
    print("")
    print("留一法（每次去掉 1 组，共 %d 次）：" % n)
    print("  s 最小 %.4f  最大 %.4f  标准差 %.4f" % (min(loo), max(loo), float(np.std(loo))))

    rng = np.random.default_rng(20261111)      # 固定种子 ⇒ 结果可复现
    m = max(6, n - 4)
    sub = []
    for _ in range(30):
        idx = sorted(rng.choice(n, size=m, replace=False).tolist())
        sub.append(_one(idx, Rg, tg, Rc, tc)[0])
    print("随机抽 %d 组（共 30 次）：" % m)
    print("  s 最小 %.4f  最大 %.4f  标准差 %.4f" % (min(sub), max(sub), float(np.std(sub))))

    sd = max(float(np.std(loo)), float(np.std(sub)))
    bias = abs(s0 - 1.0)
    print("")
    print("稳健性标准差 %.4f  vs  偏离量 |s−1| = %.4f" % (sd, bias))
    if sd <= 0.03 and bias > 3.0 * sd:
        print("⇒ s 对样本子集很稳定、且远大于自身散布 ⇒ 这是真实的系统性尺度差。")
        print("   下一步只能实测：横向平移 40 mm，看 PnP 报多少（横向重建只吃格子边长）。")
    elif sd <= 0.08:
        print("⇒ s 半稳：既有系统成分也有噪声，0.7549 不能当精确结论用。")
        print("   t_X 已经用 s 换算过、方程是自洽的，可以先用；s 本身别再追小数点。")
    else:
        print("⇒ s 随子集大幅跳动 ⇒ 它基本在拟合噪声，不该当物理结论。")
        print("   别再为了 0.7549 改 --square 重采：t_X 已按 s 换算，输出自洽。")
        print("   要提升就增加激励（J1 扫更大角度 / 多一个关节参与），而不是改 --square。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
