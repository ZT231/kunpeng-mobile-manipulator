"""生成眼在手标定的**占位**文件（placeholder），用于软件先行、无硬件打通链路。

为什么要这个脚本，而不是手写两个 JSON：
    ① 手写 4x4 矩阵极容易写漏一行、写错一个数，而 R 不正交的错误会一路传到
       FK 里，表现为"坐标莫名其妙偏几十厘米"，极难定位；
    ② 占位值必须带 placeholder:true，才能在页面上明确告诉人"这不是实测精度"，
       手写时很容易忘掉这个标记，于是占位数据被当成真值四处引用；
    ③ 等真机标定完成后，把 handeye_X.json 覆盖掉即可，代码一行都不用改。

概念（大白话）：这是给机器"先装个假眼睛"，好让后面所有流程先跑通气，
              等真的眼睛装好，换掉这个文件就行。
规范定义：标定元数据占位（calibration placeholder），以名义几何参数构造
         T_end_cam ∈ SE(3) 与 K ∈ R^{3×3}，并附 placeholder 标记与来源说明，
         使下游消费者可在无实测数据时完成接口联调与回归测试。

用法（板或 PC 都可以跑，只需要标准库 + numpy 可选）：
    python3 gen_placeholder_calib.py            # 在脚本目录生成两个 JSON
    python3 gen_placeholder_calib.py --out DIR  # 指定输出目录
"""

import argparse
import json
import math
import os


def rot_y_deg(deg):
    """绕 Y 轴旋转 deg 度的 3x3 阵（相机光轴朝下 / 朝前的切换就靠它）。

    规范定义：Ry(θ) = [[cosθ, 0, sinθ], [0, 1, 0], [-sinθ, 0, cosθ]]。
    """
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]


def build_x():
    """构造名义手眼矩阵 X = T_end_cam（末端系 ← 相机系）。

    名义安装假设（**不是实测**）：
        · 相机刚性固定在末端夹爪上方，光轴竖直朝下（绕 Y 转 180°）；
        · 相对末端原点：沿 y 前移 2.0 cm、沿 z 抬 3.0 cm。

    ⚠️ 为什么偏移取 (0, +2cm, +3cm) 而不是随便填：
        占位数据虽然"假"，但必须落在**真实机械臂的可达范围内**，否则一算就报
        「目标超出机械臂可达范围 (d=25.7, L1+L2=25.2)」，链路等于没通。
        实测教训：第一版取 x 偏 1.5cm 时，初始位下画面中心射线打到 d=25.7cm，
        刚好越界 0.5cm —— 于是 13 项检查全挂。占位值也要按工作空间设计。
    """
    R = rot_y_deg(180.0)
    X = [[R[0][0], R[0][1], R[0][2], 0.000],
         [R[1][0], R[1][1], R[1][2], 0.020],
         [R[2][0], R[2][1], R[2][2], 0.030],
         [0.0, 0.0, 0.0, 1.0]]
    return X


X_DOC = {
    "X_matrix": None,
    "t_cam2gripper_cm": [0.0, 2.0, 3.0],
    "rpy_cam2gripper_deg": [0.0, 180.0, 0.0],
    "samples": 0,
    "placeholder": True,
    "note": ("占位数据：由 gen_placeholder_calib.py 按名义安装尺寸构造，"
             "未经实机标定，不可用于定位精度评估。真机标定后请用 "
             "handeye_calib.py solve 产出的 handeye_X.json 直接覆盖本文件。"),
    "units": {"translation": "m（X_matrix 的第 4 列）", "rotation": "SE(3) 旋转部分"},
    "semantics": "T_end_cam：把相机坐标系下的点变换到末端坐标系",
}

K_DOC = {
    "camera_matrix": [[600.0, 0.0, 320.0],
                      [0.0, 600.0, 240.0],
                      [0.0, 0.0, 1.0]],
    "dist_coeff": [0.0, 0.0, 0.0, 0.0, 0.0],
    "image_size": [640, 480],
    "rms": 0.0,
    "placeholder": True,
    "note": ("占位数据：fx=fy=600 px、cx=320、cy=240、畸变全 0，对应 640x480 画面，"
             "仅用于无硬件时打通链路。真机标定后请用 "
             "handeye_calib.py intrinsics 产出的 camera_intrinsics.json 覆盖。"),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="", help="输出目录，默认脚本所在目录")
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    out_dir = args.out or here
    os.makedirs(out_dir, exist_ok=True)

    x_doc = dict(X_DOC)
    x_doc["X_matrix"] = build_x()

    x_path = os.path.join(out_dir, "handeye_X.json")
    k_path = os.path.join(out_dir, "camera_intrinsics.json")
    for path, doc in ((x_path, x_doc), (k_path, K_DOC)):
        if os.path.isfile(path):
            print("⚠ 已存在，将被覆盖：%s" % path)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(doc, f, indent=2, ensure_ascii=False)
        print("已写入 %s" % path)

    print("-" * 56)
    print("⚠ 这两份都是占位数据（placeholder: true）")
    print("  用途：无硬件时打通「像素→手眼→工作台→IK」链路")
    print("  限制：不代表任何定位精度；完成线第 5 项「至少一次实标」仍未达成")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
