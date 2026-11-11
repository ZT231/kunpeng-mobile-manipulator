# -*- coding: utf-8 -*-
"""
把 samples.json 里的「末端相对基座」用当前版 FK 重算一遍 —— 修 FK 后不必重采
=====================================================================
概念（大白话 + 规范定义）
---------------------------------------------------------------------
采集时 `collect` 把两样东西写进了 samples.json：
    ① 机器人侧：6 路脉宽(pulses) + 由它算出的末端位姿 R_gripper2base / t_gripper2base
    ② 相机侧  ：标定板相对相机的位姿 R_target2cam / t_target2cam
`solve` 只吃 ① 里那两份**已经算好存下来的矩阵**，它不会重跑正运动学。
所以改了 `fk_gripper2base()` 之后，对**旧的 samples.json** 解出来的数一个都不会变
（这就是 11-10 那次「改完再 solve，s=0.7461 一模一样」的原因）。

本脚本做一件事：**用 pulses 按新 FK 重算 ①，相机侧 ② 原样保留**，
生成一份新的 samples 文件。因为 pulses 是原始观测、不依赖 FK 的写法，
所以这一步在数学上等价于「用新 FK 重新采一遍」，但不用碰机械臂、不用重拍。
    规范定义：给定同一组关节脉宽 q_i（原始观测），重算
    T_base_gripper(q_i) ← FK_new(q_i)，其余观测量 T_cam_target 保持不变，
    构造新的样本集 { (T_base_gripper(q_i), T_cam_target,i) } 供 AX=XB 重解。

为什么不是「直接改 samples.json 里的 R」而是重算
---------------------------------------------------------------------
因为「错在哪」比「改成什么」更难说清：镜像姿态不是把矩阵取逆能修好的，
它随 θ4 变化。只有回到关节角这个唯一可信的中间量重跑 FK，才与实物一致。

用法
---------------------------------------------------------------------
    python3 refk_samples.py samples.json --out samples_fkfix.json
    python3 handeye_calib.py solve --samples samples_fkfix.json --out handeye_X.json

只依赖 numpy + arm_ik + handeye_calib，**不 import cv2**，无相机也能跑。
"""

import argparse
import json
import math
import os
import sys

import numpy as np

from arm_ik import ArmIK
import handeye_calib as hc


def rot_angle_deg(R):
    """两个旋转矩阵之间差多少度：cosθ = (tr(R)−1)/2，再夹到 [-1,1] 防浮点越界。"""
    c = (float(np.trace(R)) - 1.0) / 2.0
    c = max(-1.0, min(1.0, c))
    return math.degrees(math.acos(c))


def tool_dir_from_angles(phys):
    """由关节角直接写出「工具轴指向」（单位矢量），与位置公式同源：
    L3 的贡献是 (cos t4, sin t4) 落在臂平面内，再整体绕 Z 转 t1。
    用途：它是**不经过 R** 的独立量，拿来校验 R 的 X 轴有没有镜像。
    """
    t1 = math.radians(phys[0] + 90.0)
    t4 = math.radians(phys[3] - 90.0)
    return np.array([math.cos(t4) * math.cos(t1),
                     math.cos(t4) * math.sin(t1),
                     math.sin(t4)])


def refk(samples, ik, verbose=True):
    """就地把每组样本的 R_gripper2base / t_gripper2base 换成新 FK 的结果。

    返回 (ok_count, skipped_count, mean_att_deg, max_att_deg, max_pos_cm, min_dot)。
    """
    ok = 0
    skipped = 0
    sum_att = 0.0
    max_att = 0.0
    max_pos = 0.0
    min_dot = 1e9
    if verbose:
        print("%3s %8s %8s %10s %10s %8s" %
              ("#", "θ1", "θ4", "姿态改动", "位置改动cm", "X轴·L3"))
    for i, s in enumerate(samples, 1):
        pulses = s.get("pulses")
        if not pulses or len(pulses) != 6:
            # 没有脉宽就无从重算——只可能是手写/裁剪过的样本文件，跳过并汇报
            skipped += 1
            if verbose:
                print("%3d  缺少 pulses，跳过" % i)
            continue
        phys = hc.pulses_to_physical([int(p) for p in pulses], ik)
        T = hc.fk_gripper2base(phys)
        R_new = T[:3, :3]
        t_new = T[:3, 3] / 100.0            # cm → m，与相机侧单位一致

        R_old = np.array(s["R_gripper2base"], float)
        t_old = np.array(s["t_gripper2base"], float).reshape(3)

        d_att = rot_angle_deg(R_old.T @ R_new)
        d_pos = float(np.linalg.norm(t_new - t_old)) * 100.0
        # 姿态自检：新 R 的 X 轴必须与工具指向重合，否则说明 FK 姿态还没修对
        dot = float(np.dot(R_new[:, 0], tool_dir_from_angles(phys)))

        max_att = max(max_att, d_att)
        sum_att += d_att
        max_pos = max(max_pos, d_pos)
        min_dot = min(min_dot, dot)
        ok += 1
        if verbose:
            print("%3d %8.1f %8.1f %9.2f° %10.3f %9.4f" %
                  (i, phys[0], phys[3], d_att, d_pos, dot))

        s["R_gripper2base"] = R_new.tolist()
        s["t_gripper2base"] = t_new.tolist()
    mean_att = (sum_att / ok) if ok else 0.0
    return ok, skipped, mean_att, max_att, max_pos, min_dot


def main():
    ap = argparse.ArgumentParser(
        description="用当前版 FK 重算 samples.json 的机器人侧位姿（改 FK 后免重采）")
    ap.add_argument("samples", help="输入的 samples.json（collect 的产物）")
    ap.add_argument("--out", default=None,
                    help="输出文件名（默认在输入名后加 _fkfix）")
    ap.add_argument("--params", default=None,
                    help="arm_params.json 路径（默认与本脚本同目录）")
    args = ap.parse_args()

    out = args.out
    if not out:
        root, ext = os.path.splitext(args.samples)
        out = root + "_fkfix" + (ext or ".json")

    with open(args.samples, "r", encoding="utf-8") as f:
        data = json.load(f)
    samples = data.get("samples") or []
    if len(samples) < 3:
        print("样本不足（%d），至少需要 3 组，无需重算" % len(samples))
        return 1

    ik = ArmIK(args.params) if args.params else ArmIK()
    square = data.get("square_m")
    print("重算前：%s（%d 组，square_m=%s）" %
          (args.samples, len(samples),
           ("%.4f" % square) if square else "未记录"))
    print("FK 几何：L1=%.0fmm L2=%.0fmm L3=%.0fmm H=%.0fmm" %
          (hc.ARM_L1 * 10, hc.ARM_L2 * 10, hc.ARM_L3 * 10, hc.ARM_H * 10))
    print("")

    ok, skipped, mean_att, max_att, max_pos, min_dot = refk(samples, ik)

    print("")
    print("姿态改动：平均 %.2f° / 最大 %.2f°（旧 FK 是镜像，最大约为 2×θ4 上限）" %
          (mean_att, max_att))
    print("位置改动：最大 = %.4f cm（新 FK 只改姿态，位置应当不动）" % max_pos)
    # 姿态自检：X 轴与工具指向的点积必须 ≈ +1；≈ −1 或 cos(2θ4) 说明还在镜像
    if min_dot > 0.999:
        print("姿态自检：X轴·L3指向 最小 = %+.4f ✅（一列都没镜像）" % min_dot)
        dot_ok = True
    else:
        print("姿态自检：X轴·L3指向 最小 = %+.4f ❌ 仍不一致，别拿这份结果去解" % min_dot)
        dot_ok = False

    if skipped:
        print("⚠️ 有 %d 组缺少 pulses，未重算——解出来的 X 会混用新旧 FK，不可信" % skipped)

    data["samples"] = samples
    data["refk"] = {
        "tool": "refk_samples.py",
        "src": os.path.basename(args.samples),
        "max_att_deg": round(float(max_att), 3),
        "max_pos_cm": round(float(max_pos), 5),
        "min_axis_dot": round(float(min_dot), 6),
    }
    with open(out, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    print("\n已写出 %s（%d 组，square_m 原样保留）" % (out, ok))
    print("下一步：python3 handeye_calib.py solve --samples %s --out handeye_X.json"
          % os.path.basename(out))
    return 0 if (dot_ok and not skipped) else 2


if __name__ == "__main__":
    sys.exit(main())
