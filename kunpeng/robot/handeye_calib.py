# -*- coding: utf-8 -*-
"""
眼在手（eye-in-hand）手眼标定  —— 解 AX = XB 求"相机相对臂末端"的位姿 X
=====================================================================
概念（大白话 + 规范定义）
---------------------------------------------------------------------
手眼标定（eye-in-hand）：相机装在机械臂末端、随臂一起动。要让"相机看到的目标"
变成"我该把臂伸到哪"，必须知道【相机坐标系(C) 相对 末端执行器坐标系(E)】的固定
刚体位姿 X（一个 4x4 变换矩阵）。
   规范定义：hand-eye calibration，求解满足 A_i · X = X · B_i 的 X，
   其中 A_i = 末端相对基座的位姿（来自关节角→正运动学），
   B_i = 标定板相对相机的位姿（来自相机内参 + 棋盘格 solvePnP）。
   解出的 X 即相机相对末端的变换（camera-to-gripper）。

两种布置（只是数据怎么采相反）：
   - 眼在手(eye-in-hand)：相机随末端动，标定板固定在环境里不动 → 本项目用这个
   - 眼在外(eye-to-hand)：相机固定，标定板贴在末端随臂动

为什么旧项目的眼在外单应(homography)不能直接用：
   眼在外时相机和臂底座都固定，像素↔台面 mm 用一张单应矩阵一次标定永久用。
   眼在手时相机随臂动，相机和末端之间的相对变换必须用 AX=XB 重新标定，
   数学上是旋转矩阵方程（不是平面单应），需要专门拍棋盘格解算。

=====================================================================
本文件相对上一版改了什么（2026-11-09，板上实测倒逼）
---------------------------------------------------------------------
板上 openEuler 实测：opencv 5.0.0 没有 cv2.calibrateHandEye、DISPLAY 为空。
上一版两处会直接跑挂，本版两处都改掉，且**不装任何包、不降级 opencv**：

   ① 解算不再用 cv2.calibrateHandEye（它属于 opencv-contrib 扩展，主包没有）。
      改成纯 numpy：把旋转方程用 vec + Kronecker 积拉平成 9 元线性方程组，
      SVD 求零空间得到 R_X，再用最小二乘解平移 t_X。
      概念（大白话）：把"矩阵等式"改写成"一堆一次方程"，用最小二乘凑出最接近的解。
      规范定义：vec(AXB) = (B^T ⊗ A) vec(X)；由 R_A R_X = R_X R_B 得
      [I⊗R_A − R_B^T⊗I] vec(R_X) = 0，其零空间即最小奇异值对应的右奇异向量。

   ② 采集不再弹窗口（cv2.imshow / waitKey 需要 X11，服务器版没有）。
      改成"无头定时采集"：每隔固定秒数自动拍一张 / 采一组，终端打印进度，
      可选把帧存成 jpg 供事后复核。有 DISPLAY 时仍会显示窗口（PC 上调试方便）。
      概念（大白话）：没屏幕就不看画面，让程序自己按秒表采，人只管摆姿态。
      规范定义：headless 模式；cv2.VideoCapture / imwrite / findChessboardCorners /
      solvePnP 都不依赖 DISPLAY，只有 imshow / waitKey / destroyAllWindows 依赖。

   ③ collect 不再直接打开串口（2026-11-09 19:15，核准批 2 时倒逼的第二次改造）。
      旧版 collect 用 ArmHost(serial.Serial("/dev/ttyUSB0")) 独占串口读 STATE，
      但它是「只读」的：STATE 返的是固件脉宽目标值，姿态必须靠 web_control.py
      下发 JOINT 才会变；停掉 5000 就没有下发通道，不停则两个进程抢同一帧。
      改法：collect 改走 HTTP POST /api/state，串口只由 web_control.py 一个人管，
      本程序只拿结果。概念（大白话）：不抢话筒，改问主持人要答案。
      规范定义：进程间单一资源独占 + HTTP 代理读取，避免串口帧被两个读者竞争消费。

为什么这么设计（README 三行法，见 kunpeng/robot/README.md）：
   ① 用了什么：纯标准库 + numpy（SVD / lstsq / Kronecker 积）+ cv2 基础读帧。
   ② 为什么不装 contrib、不降级 opencv：鲲鹏板的 opencv 5.0.0 是已实证的
      S3b 视觉链路（detector_service.py / stage1_yolov8.py）依赖，为一次标定
      去动它会污染整条跑通的链路；把求解抽成纯 numpy 才是不动环境的解法。
   ③ 不这么写会怎样：照旧版跑，intrinsics/collect 一进 imshow 就崩，
      solve 直接 return 1，白摆十几张棋盘格；或强行 pip 装 4.x 把板子环境搞坏。

=====================================================================
复用资产（从原视觉机械臂项目抽取并适配，接口不变）：
   arm_ik.py        —— ArmIK 逆解/正解（本模块用 to_angles() 把脉宽还原成物理角）
   arm_params.json  —— 舵机脉宽↔角度标定表（保留不动）
   arm_host.py      —— 旧版 collect 直连串口读 STATE 用；本版 collect 不再 import 它
                      （板上留着无妨，其它工具仍可用）

依赖：
   numpy               —— 必需（解算、自检都用）
   opencv-python       —— 仅 intrinsics / collect 采图用，**主包即可、不需要 contrib**
   urllib.request      —— 标准库；collect 走 HTTP 读 /api/state，不再需要 pyserial
   ⚠️ selftest / solve 完全不 import cv2，不需要相机、串口、桌面。

用法（4 步；无 DISPLAY 时自动进入无头模式）：
   # 0) 自检：FK 对齐 + 合成数据还原 X + solve 输出格式（不需相机/串口/桌面）
   python handeye_calib.py selftest

   # 1) 标定相机内参：自动采 20 张，每 1 秒检测到棋盘格就存一张
   python handeye_calib.py intrinsics --camera 0 --cols 9 --rows 6 --square 0.025 \
        --count 20 --interval 1.0 --save-dir ./calib_imgs --out camera_intrinsics.json

   # 2) 采集手眼数据：自动采 12 组（提示后在网页摆不同姿态，要有明显旋转）
   #    关节角走 HTTP POST /api/state 读，不打开串口、不与 web_control.py 抢帧
   python handeye_calib.py collect --camera 0 --state-url http://127.0.0.1:5000/api/state \
        --intrinsics camera_intrinsics.json --cols 9 --rows 6 --square 0.025 \
        --count 12 --interval 5.0 --save-dir ./calib_imgs --out samples.json

   # 3) 解算 X（纯 numpy，不需要 cv2.contrib）
   python handeye_calib.py solve --samples samples.json --out handeye_X.json
=====================================================================
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np

# ---------------------------------------------------------------------------
# 机械臂几何（与旧项目 arm_params.json 一致；新臂量测后改这里）
#   单位：cm。L1 大臂 / L2 小臂 / L3 腕→夹爪 / H 肩关节离台面高度
# ---------------------------------------------------------------------------
ARM_L1 = 10.6
ARM_L2 = 14.6
ARM_L3 = 17.0
ARM_H = 8.1
# 腕部偏心（夹取点相对 J5 轴线的横向偏移 cm，与 arm_params.json 一致；新臂重测后改）
ARM_WRIST_ECC = 1.0
ARM_ECC_DIR = 0.0


# ---------------------------------------------------------------------------
# 概念·正运动学 / 齐次变换（大白话 + 规范定义）
# ---------------------------------------------------------------------------
def _rz(a):
    """绕 Z 轴旋转 a 弧度（偏航，基座转向）。

    规范定义：rotation about Z axis，标准右手旋转矩阵 Rz(θ)。"""
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)


def _ry(a):
    """绕 Y 轴旋转 a 弧度（俯仰，大臂/小臂在臂平面内抬起）。

    规范定义：rotation about Y axis，Ry(θ)。"""
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]], dtype=float)


def _tx(d):
    """沿 X 轴平移 d（连杆长度方向）。

    规范定义：translation along X，T_x(d)。"""
    return np.array([[1, 0, 0, d], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)


def _tz(d):
    """沿 Z 轴平移 d（抬到肩关节高度）。

    规范定义：translation along Z，T_z(d)。"""
    return np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, d], [0, 0, 0, 1]], dtype=float)


def fk_gripper2base(angles_deg):
    """正运动学：6 路物理关节角(度) → 末端(夹爪安装点)相对基座的 4x4 位姿。

    角度约定与旧项目 ik.py 完全一致（正前方/竖直向上=0°）：
        θ1 基座方位(绕Z)  θ2 大臂俯仰  θ3 小臂俯仰  θ4 腕俯仰
    这里用"绝对角矢量求和"直接复现 ik.forward 的模型（比矩阵连乘更直观也更不易错）：
        每节连杆 i 在其绝对俯仰角 ti 下，对径向/竖直方向的贡献为
        (Li·cos ti, Li·sin ti)；再叠加肩关节高度 H，最后整体绕基座 Z 转 θ1。
    返回值平移单位：cm（与 arm_params.json 一致）；手眼解算时会转成 m。
    ⚠️ 假设：相机刚性固定在腕部、位于 J5 轴线附近，故 J5/J6（工具轴自转/夹爪开合）
       不移动相机。若相机明显偏离 J5 轴线，需在末尾追加 J5 旋转 + 偏心平移。
    """
    t1 = math.radians(angles_deg[0] + 90.0)
    t2 = math.radians(angles_deg[1] + 90.0)   # 大臂绝对俯仰角（从水平量）
    t3 = math.radians(angles_deg[2] + 90.0)   # 小臂绝对俯仰角
    t4 = math.radians(angles_deg[3] - 90.0)   # 腕绝对俯仰角

    # 各连杆在其绝对角下的径向(x')/竖直(z')分量，求和得到腕点（臂平面坐标）
    radial = ARM_L1 * math.cos(t2) + ARM_L2 * math.cos(t3) + ARM_L3 * math.cos(t4)
    vert = ARM_H + ARM_L1 * math.sin(t2) + ARM_L2 * math.sin(t3) + ARM_L3 * math.sin(t4)
    # 整体绕基座 Z 转向，把臂平面坐标投到世界 X-Y
    x = radial * math.cos(t1)
    y = radial * math.sin(t1)
    z = vert

    # 腕部偏心补偿（与 ik.forward 同公式）：夹取点相对 J5 轴线偏了 wrist_ecc，
    # 方向随基座方位+腕旋转+ecc_dir 旋转。tilt = 腕俯仰 th4（竖直向下=0）。
    if ARM_WRIST_ECC > 1e-6:
        psi = math.radians(angles_deg[0] + 90.0 + angles_deg[4] + ARM_ECC_DIR)
        ec = ARM_WRIST_ECC * math.cos(math.radians(angles_deg[3]))
        x += ec * math.cos(psi)
        y += ec * math.sin(psi)

    # 姿态：基座偏航(Rz) + 腕俯仰(Ry)，工具指向即腕绝对角方向
    R = (_rz(t1) @ _ry(t4))[:3, :3]
    T = np.eye(4)
    T[:3, :3] = R
    T[0, 3], T[1, 3], T[2, 3] = x, y, z
    return T


def pulses_to_physical(pulses, ik):
    """6 路脉宽(us) → 6 路物理关节角(度)，喂给 fk_gripper2base 用。

    串口 STATE 读上来的是脉宽；arm_ik.ArmIK.to_angles 返回的是"舵机相对角"
    （J3=相对大臂、J4=相对小臂，见 ik.py 的 to_pulses 补偿逻辑），不是物理绝对角。
    这里做它的逆运算还原成物理角：
        θ1 = s1
        θ2 = s2
        θ3 = s3 + θ2
        θ4 = s4 + 180 + θ3        （tilt 已含在 s4 里，公式来自 ik.py）
    其中 s1..s6 = to_angles 输出。
    """
    s = ik.to_angles(pulses)
    th1 = s[0]
    th2 = s[1]
    th3 = s[2] + th2
    th4 = s[3] + 180.0 + th3
    # J5/J6 不参与末端位置，但保留原值（供将来相机偏心扩展用）
    return [th1, th2, th3, th4, s[4], s[5]]


# ===========================================================================
# 新增：纯 numpy 解 AX=XB（替代 cv2.calibrateHandEye）
# ===========================================================================
def _pose_from_rt(R, t):
    """把 3x3 旋转 R + 3 维平移 t 拼成 4x4 齐次变换矩阵。

    概念（大白话）：把"转了多少"和"挪了多少"塞进一张 4x4 表里，
    这样"先转再挪"两步能用一次矩阵乘法算完。
    规范定义：SE(3) 刚体变换 T = [[R, t], [0, 1]]，属于特殊欧氏群。
    """
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, dtype=float).reshape(3, 3)
    T[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return T


def _inv_pose(T):
    """求刚体变换的逆：把"从 A 到 B"倒过来变成"从 B 到 A"。

    规范定义：T^{-1} = [[R^T, -R^T·t], [0, 1]]（旋转阵正交，逆等于转置）。
    """
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def _rotvec_to_R(axis, angle):
    """绕单位轴 axis 转 angle 弧度 → 3x3 旋转阵（合成自检数据用）。

    规范定义：Rodrigues' rotation formula，
    R = cosθ·I + (1-cosθ)·n·n^T + sinθ·[n]_x，[n]_x 为轴向量的反对称矩阵。
    """
    n = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(n)
    if norm < 1e-9:
        n = np.array([0.0, 0.0, 1.0])
        norm = 1.0
    n = n / norm
    x, y, z = n
    K = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=float)
    c, s = math.cos(angle), math.sin(angle)
    return c * np.eye(3) + (1.0 - c) * np.outer(n, n) + s * K


def _project_so3(M):
    """把一个"接近但不一定正交"的 3x3 投影回旋转群 SO(3)。

    概念（大白话）：SVD 解出来的是一串数字，不一定严格是旋转阵；
    这一步把它掰成"最近的、行列式=+1 的正交阵"。
    规范定义：Procrustes 正交投影 —— M = UΣV^T 时，
    Frobenius 范数下最近的旋转阵为 R = U·diag(1,1,det(U·V^T))·V^T。
    """
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0.0:
        U[:, -1] *= -1.0          # 等价于中间夹一个 diag(1,1,-1)
        R = U @ Vt
    return R


def solve_handeye_axxb(R_g2b, t_g2b, R_t2c, t_t2c, verbose=True):
    """纯 numpy 解 AX=XB，返回 (R_cam2gripper, t_cam2gripper)。

    概念（大白话）：
      相机固定在臂末端、标定板固定在环境里。不管臂怎么摆，
      "末端位姿 × 相机相对末端 X × 标定板相对相机" 永远等于同一个常量。
      把两次摆姿的这个等式相除，常量就消掉了，得到 A·X = X·B，X 就是要求的。
    规范定义：
      eye-in-hand 下 T_g2b(i)·X·T_t2c(i) = T_target2base（常量），
      消去常量得 A_i·X = X·B_i，其中
        A_i = T_g2b(j)^{-1}·T_g2b(i)，  B_i = T_t2c(j)·T_t2c(i)^{-1}。
      展开成两个方程：
        R_A·R_X = R_X·R_B                      （旋转，齐次 → 用 SVD 求零空间）
        (R_A − I)·t_X = R_X·t_B − t_A          （平移，非齐次 → 最小二乘）
    规范定义（线性化）：
      vec(A·X·B) = (B^T ⊗ A)·vec(X)，vec 为按列堆叠、⊗ 为 Kronecker 积。
      由 R_A·R_X = R_X·R_B 得 [I⊗R_A − R_B^T⊗I]·vec(R_X) = 0，
      解即系数矩阵最小奇异值对应的右奇异向量。

    为什么不用 cv2.calibrateHandEye：
      它属于 opencv-contrib 扩展，板上 opencv 5.0.0 没编进去（实测 False）；
      为这一个函数降级整条已实证的视觉链路不划算，故改用纯 numpy。
    """
    N = len(R_g2b)
    if N < 3:
        raise ValueError("至少需要 3 组样本，当前只有 %d 组" % N)

    T_g2b = [_pose_from_rt(R_g2b[i], t_g2b[i]) for i in range(N)]
    T_t2c = [_pose_from_rt(R_t2c[i], t_t2c[i]) for i in range(N)]

    # 用所有两两组合（不只是相邻两组）：信息更多、个别姿态退化也不至于全盘崩。
    # 概念（大白话）：任意两次摆姿都能列一个方程，全列上比只用相邻两次更抗噪。
    K_rows = []
    motion_list = []              # 存 (R_A, t_A, t_B)，后面解平移还要用
    for i in range(N):
        for j in range(i + 1, N):
            A = _inv_pose(T_g2b[j]) @ T_g2b[i]
            B = T_t2c[j] @ _inv_pose(T_t2c[i])
            R_A, t_A = A[:3, :3], A[:3, 3]
            R_B, t_B = B[:3, :3], B[:3, 3]
            # [I⊗R_A − R_B^T⊗I]·vec(R_X) = 0
            K_rows.append(np.kron(np.eye(3), R_A) - np.kron(R_B.T, np.eye(3)))
            motion_list.append((R_A, t_A, t_B))

    K = np.vstack(K_rows)         # 形状 (9*M) x 9
    # full_matrices=False：只要右奇异向量，别让 numpy 生成 (9*M)x(9*M) 的大 U。
    # 概念（大白话）：采集 20 组时 K 有 1710 行，默认会开出 1710² 的临时矩阵，
    # 板上 16GB 内存也扛不住几次；关掉它只留 9x9 的 Vt，结果一样。
    _, svals, Vt = np.linalg.svd(K, full_matrices=False)
    v = Vt[-1]                    # 最小奇异值对应的右奇异向量 = vec(R_X) 的方向

    if verbose:
        print("  AX=XB 姿态对 %d 组；奇异值最小/次小 = %.3e / %.3e"
              % (len(K_rows), svals[-1], svals[-2]))
        # 概念（大白话）：如果臂每次都只平移不转，方程就解不出唯一的 X（退化）。
        if svals[-2] > 1e-9 and svals[-1] / svals[-2] > 1e-3:
            print("  ⚠️ 旋转激励不足（姿态间相对旋转太小或转轴太单一），X 可能不可靠")

    def _solve_t(R):
        """给定 R_X，用 (R_A − I)·t_X = R_X·t_B − t_A 最小二乘解 t_X。"""
        rows, rhs = [], []
        for R_A, t_A, t_B in motion_list:
            rows.append(R_A - np.eye(3))
            rhs.append(R @ t_B - t_A)
        A_stack = np.vstack(rows)
        b_stack = np.concatenate(rhs)
        t, _, _, _ = np.linalg.lstsq(A_stack, b_stack, rcond=None)
        return t, float(np.linalg.norm(A_stack @ t - b_stack))

    # SVD 零空间向量的符号是任意的（v 和 -v 都满足齐次旋转方程）。
    # 用"平移方程残差"挑符号：真解残差小，取反的解残差大。
    # 概念（大白话）：方程组只能定出方向定不出正负，拿另一个方程验一下哪个对。
    candidates = []
    for sign in (1.0, -1.0):
        M = (sign * v).reshape(3, 3, order="F")   # vec 是按列堆叠，故用 F 顺序还原
        R = _project_so3(M)
        t, resid = _solve_t(R)
        candidates.append((resid, R, t))
    resid, R_best, t_best = min(candidates, key=lambda c: c[0])
    if verbose:
        print("  选定候选解：平移方程残差 = %.3e" % resid)
    return R_best, t_best.reshape(3, 1)


def _is_headless(args):
    """判断要不要弹窗口。

    概念（大白话）：板上 openEuler 没桌面（DISPLAY 为空），cv2.imshow 一调就崩；
    这时改成"只打印进度 + 定时自动存图"，完全不碰窗口。
    规范定义：headless 模式；VideoCapture / imwrite / findChessboardCorners / solvePnP
    都不依赖 DISPLAY，只有 imshow / waitKey / destroyAllWindows 依赖。
    """
    if bool(getattr(args, "headless", False)):
        return True
    return not bool(os.environ.get("DISPLAY"))


# ===========================================================================
# 命令实现
# ===========================================================================
def _selftest_axxb(N, X_true, T_t2b):
    """合成数据验证 solve_handeye_axxb：造一个已知 X，看能否还原回来。

    概念（大白话）：不用相机不用串口，自己编一组"假姿态"、把答案藏进去，
    再让求解器算一遍；算得回来，说明方程方向、vec 顺序、符号都没写反。
    规范定义：ground-truth recovery 数值一致性测试 —— 由
    T_g2b(i)·X·T_t2c(i) = T_target2base 反解 T_t2c(i) = X^{-1}·T_g2b(i)^{-1}·T_t2b，
    再校验 ‖R_est − R_true‖_F 与 ‖t_est − t_true‖_2。
    """
    R_true, t_true = X_true[:3, :3], X_true[:3, 3]
    axes = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0],
            [0, 1, 1], [1, 0, 1], [1, -1, 0], [0, 1, -1]]
    R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []
    for i in range(N):
        # 每次给一个"有明显旋转激励"的末端位姿（转轴要多样，不能全是同一个轴）
        R_g = _rotvec_to_R(axes[i % len(axes)], math.radians(15.0 + 10.0 * (i % 5)))
        t_g = np.array([0.10 + 0.010 * i, -0.05 + 0.008 * i, 0.20 + 0.006 * i])
        T_g = _pose_from_rt(R_g, t_g)
        # 反解"相机看到的标定板位姿"：T_g · X · T_t2c = T_t2b
        T_c = _inv_pose(X_true) @ _inv_pose(T_g) @ T_t2b
        R_g2b.append(R_g)
        t_g2b.append(t_g)
        R_t2c.append(T_c[:3, :3])
        t_t2c.append(T_c[:3, 3])
    R_est, t_est = solve_handeye_axxb(R_g2b, t_g2b, R_t2c, t_t2c, verbose=False)
    r_err = float(np.linalg.norm(R_est - R_true))
    t_err = float(np.linalg.norm(t_est.reshape(3) - t_true))
    return r_err, t_err, R_g2b, t_g2b, R_t2c, t_t2c


def cmd_selftest(args=None):
    """离线自检：FK 对齐 + 合成数据还原 X + solve 输出格式。

    不需要相机 / 串口 / 桌面 / opencv（连 cv2 都不 import），板上跑得起来。
    """
    import tempfile
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from arm_ik import ArmIK
    ik = ArmIK()
    ok = True

    # --- 1) 原有 6 组：本文件 FK 位置 vs 旧项目 ArmIK.forward ---
    print("自检 1：fk_gripper2base 位置 vs ArmIK.forward（忽略 sub-cm 腕偏心）")
    cases = [(0, 0, 0, 0), (-45, 20, -15, 0), (90, -30, 10, 30),
             (30, 45, 45, 0), (-90, 10, 20, -20), (120, -10, -5, 10)]
    for a1, a2, a3, a4 in cases:
        ang = [a1, a2, a3, a4, 0.0, 0.0]
        T = fk_gripper2base(ang)
        fx, fy, fz = T[0, 3], T[1, 3], T[2, 3]
        efx, efy, efz = ik.forward(ang)
        err = math.hypot(fx - efx, fy - efy, fz - efz)
        ok = ok and err < 0.05
        print("  角度%-22s -> 本FK(%.2f,%.2f,%.2f) ik(%.2f,%.2f,%.2f) 误差=%.3fcm %s"
              % (str(ang[:4]), fx, fy, fz, efx, efy, efz, err, "" if err < 0.05 else " ✗"))

    # --- 2) 合成数据还原 X：10 组、标准姿态 ---
    print("自检 2：纯 numpy AX=XB 合成数据还原（10 组，无噪声）")
    X_a = _pose_from_rt(_rotvec_to_R([0.3, -0.5, 0.8], math.radians(37.0)),
                        np.array([0.045, -0.030, 0.085]))
    T_t2b = _pose_from_rt(_rotvec_to_R([1, 0, 0], math.radians(12.0)),
                          np.array([0.30, 0.10, 0.02]))
    r_err, t_err, Rg, tg, Rc, tc = _selftest_axxb(10, X_a, T_t2b)
    good = r_err < 1e-5 and t_err < 1e-5
    ok = ok and good
    print("  ‖R_est−R_true‖=%.3e  ‖t_est−t_true‖=%.3e  %s"
          % (r_err, t_err, "✅" if good else "❌"))

    # --- 3) 边界：最少 3 组样本，换一个完全不同的 X ---
    print("自检 3：纯 numpy AX=XB 合成数据还原（最少 3 组，换一个 X）")
    X_b = _pose_from_rt(_rotvec_to_R([-0.2, 0.9, 0.4], math.radians(118.0)),
                        np.array([-0.060, 0.025, -0.040]))
    r_err2, t_err2, _, _, _, _ = _selftest_axxb(3, X_b, T_t2b)
    good2 = r_err2 < 1e-5 and t_err2 < 1e-5
    ok = ok and good2
    print("  ‖R_est−R_true‖=%.3e  ‖t_est−t_true‖=%.3e  %s"
          % (r_err2, t_err2, "✅" if good2 else "❌"))

    # --- 4) 端到端：合成 samples.json → cmd_solve → 校验 handeye_X.json 结构 ---
    print("自检 4：cmd_solve 端到端（合成 samples.json → handeye_X.json）")
    with tempfile.TemporaryDirectory() as td:
        sp = os.path.join(td, "samples.json")
        op = os.path.join(td, "handeye_X.json")
        samples = []
        for i in range(len(Rg)):
            samples.append({
                "pulses": [1500, 1500, 1500, 1500, 1500, 1500],
                "R_gripper2base": np.asarray(Rg[i]).tolist(),
                "t_gripper2base": np.asarray(tg[i]).reshape(3).tolist(),
                "R_target2cam": np.asarray(Rc[i]).tolist(),
                "t_target2cam": np.asarray(tc[i]).reshape(3).tolist(),
            })
        with open(sp, "w", encoding="utf-8") as f:
            json.dump({"count": len(samples), "samples": samples,
                       "square_m": 0.025}, f, indent=2)
        rc = cmd_solve(argparse.Namespace(samples=sp, out=op))
        with open(op, "r", encoding="utf-8") as f:
            d = json.load(f)
        keys = ("X_matrix", "t_cam2gripper_cm", "rpy_cam2gripper_deg", "samples")
        keys_ok = all(k in d for k in keys)
        shape_ok = len(d.get("X_matrix", [])) == 4
        good4 = (rc == 0 and keys_ok and shape_ok)
        ok = ok and good4
        print("  rc=%d  键齐全=%s  X_matrix 4x4=%s  samples=%s  %s"
              % (rc, keys_ok, shape_ok, d.get("samples"), "✅" if good4 else "❌"))

    print("自检", "通过 ✅" if ok else "失败 ❌")
    return 0 if ok else 1


def _load_intrinsics(path):
    with open(path, "r", encoding="utf-8") as f:
        d = json.load(f)
    mtx = np.array(d["camera_matrix"], dtype=float).reshape(3, 3)
    # 内参文件里畸变系数有两种常见键名（dist_coeffs / dist_coeff），都认；
    # 不同工具链导出的名字不一样，只认一个会在换文件时莫名报 KeyError。
    dist = np.array(d.get("dist_coeffs", d.get("dist_coeff", [0, 0, 0, 0, 0])),
                    dtype=float).reshape(-1)
    return mtx, dist, d


def _reconcile_square(args, intr_doc):
    """让 collect 的 --square 自动继承内参文件里记录的 square_m，防止两处尺度打架。

    为什么必须有这层：intrinsics 和 collect 各自有 --square（默认都是 0.025）。
    如果标内参时量得 0.02、采手眼时忘了带、静静用了默认 0.025，
    相机侧的 t 会整体缩放 1.25 倍 —— R_X 不受影响（旋转与尺度无关）看着"没问题"，
    但 t_X 白白偏 25%，而且**不会有任何报错**，是最难发现的一类错。
    所以这里：没显式传 --square 就用文件里的值；两边都给了但差得多就警告。
    """
    recorded = intr_doc.get("square_m")
    if recorded is None:
        return args.square
    recorded = float(recorded)
    if abs(recorded - args.square) > 1e-9:
        # 命令行显式给了不同值：尊重命令行，但必须喊出来
        if abs(recorded - args.square) / max(recorded, 1e-9) > 0.02:
            print("⚠ 本次 --square=%.4f 与内参文件记录的 %.4f 不一致"
                  % (args.square, recorded))
            print("  两者相差 %.1f%%，会让手眼平移 t_X 同比失真（旋转 R_X 不受影响）。"
                  % (abs(args.square - recorded) / recorded * 100.0))
            print("  若非有意为之，请改回 %.4f 或重做内参标定。" % recorded)
    return args.square


def _read_state_pulses(url, timeout=1.0):
    """从 web_control.py 的 /api/state 读 6 路当前脉宽（POST，纯标准库）。

    概念（大白话）：不去抢串口，改问网页后台「现在六路是多少」。
    串口只由 web_control.py 一个人管，网页滑杆下发 JOINT、它自己回读 STATE，
    本程序只通过 HTTP 拿结果 —— 两个进程不再抢同一帧 ``$OK,S,...``。
    规范定义：HTTP POST + JSON。web_control.py 的 ``api_state()`` 返回
    ``{"ok": true, "pulses": [6 路脉宽]}``，其中 pulses 来自 STATE 帧
    ``parts[3:9]``，是斜坡推进过程中的当前值（不是「已到位确认」，
    所以摆完姿态要等到位再采）。未连串口时返回
    ``{"ok": false, "error": "未连接"}``，这里直接抛异常让本组跳过。
    """
    import urllib.request

    req = urllib.request.Request(url, data=b"", method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        raw = resp.read().decode("utf-8")
    d = json.loads(raw)
    if not d.get("ok"):
        raise RuntimeError("api/state 未就绪: %s" % d.get("error", d))
    pulses = d.get("pulses")
    if not isinstance(pulses, list) or len(pulses) != 6:
        raise RuntimeError("api/state pulses 异常: %r" % (pulses,))
    return [int(p) for p in pulses]


def _chess_objp(cols, rows, square):
    """生成棋盘格内角点的 3D 坐标（Z=0 平面）。

    规范定义：标定板坐标系下 p = (col·square, row·square, 0)，单位 m。
    """
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[:cols, :rows].T.reshape(-1, 2) * square
    return objp


def cmd_intrinsics(args):
    """标定相机内参（无头友好）：定时自动拍棋盘格，cv2.calibrateCamera 求 K/dist。

    无 DISPLAY 时不弹窗口：每 --interval 秒检测到角点就存一张，采满 --count 张结束。
    概念（大白话）：人拿着棋盘格慢慢换角度/距离，程序自己按秒表存，不用按键。
    """
    import cv2
    headless = _is_headless(args)
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print("无法打开相机 %s" % args.camera)
        return 1
    objp = _chess_objp(args.cols, args.rows, args.square)
    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)
    objpoints, imgpoints = [], []
    saved = 0
    print("内参采集：共 %d 张，每 %.1f 秒检测到棋盘格就自动存一张" % (args.count, args.interval))
    print("  请拿着棋盘格缓慢换角度/距离；无界面模式=%s（DISPLAY=%s）"
          % (headless, os.environ.get("DISPLAY", "NONE")))
    last_save = time.monotonic()
    last_hint = time.monotonic()
    start_t = time.monotonic()
    while saved < args.count:
        ret, frame = cap.read()
        if not ret:
            print("读帧失败")
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (args.cols, args.rows), None)
        now = time.monotonic()
        # 去掉"按 q 退出"后必须自己兜底：一直看不到棋盘格也要能退出，别把板子卡死。
        if (now - start_t) >= args.timeout:
            print("  采集超时（%.1f 秒），已存 %d/%d 张，退出" % (args.timeout, saved, args.count))
            break
        if not headless:
            disp = frame.copy()
            if found:
                cv2.drawChessboardCorners(disp, (args.cols, args.rows), corners, found)
            cv2.imshow("intrinsics", disp)
            cv2.waitKey(1)
        if found and (now - last_save) >= args.interval:
            objpoints.append(objp)
            imgpoints.append(corners)
            saved += 1
            last_save = now
            print("  已存 %d/%d 张" % (saved, args.count))
            if args.save_dir:
                cv2.imwrite(os.path.join(args.save_dir, "intrinsics_%02d.jpg" % saved), frame)
        elif not found and (now - last_hint) >= 3.0:
            print("  ...未检测到完整棋盘格，请把 %d×%d 个内角点全放进画面"
                  % (args.cols, args.rows))
            last_hint = now
    cap.release()
    if not headless:
        cv2.destroyAllWindows()
    if len(objpoints) < 5:
        print("有效张数太少（%d），标定不可靠" % len(objpoints))
        return 1
    rms, mtx, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    out = {"camera_matrix": mtx.tolist(), "dist_coeffs": dist.ravel().tolist(), "rms": rms,
           # 记下这次标定用的单格边长：后续 collect 会读取它做尺度一致性检查，
           # 避免两次命令用了不同 --square 却都没有任何提示。
           "square_m": float(args.square),
           "image_size": [int(gray.shape[1]), int(gray.shape[0])],
           "cols": int(args.cols), "rows": int(args.rows)}
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)
    print("内参已保存 %s （重投影 RMS=%.4f，越小越好）" % (args.out, rms))
    print("  记录 square_m=%.4f；collect 会自动沿用（除非命令行另行指定）" % float(args.square))
    return 0


def cmd_collect(args):
    """采集手眼数据（无头友好）：定时自动采一组，同时记关节角位姿 + 标定板位姿。

    每 --interval 秒采一组：POST /api/state 读 6 路脉宽 → to_angles → FK 得末端位姿；
    拍棋盘格 → solvePnP 得标定板位姿。
    概念（大白话）：摆好一个姿态等一下，程序自己记一组，不用按键。
    规范定义：关节角不走串口，改由 web_control.py 独占串口、本程序 HTTP 取（见 _read_state_pulses），
    避免两个进程抢同一帧 $OK,S,...。
    ⚠️ 姿态之间要有明显旋转变化（不能只平移），否则 AX=XB 退化解不出唯一 X。
    """
    import cv2
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from arm_ik import ArmIK

    headless = _is_headless(args)
    ik = ArmIK()
    mtx, dist, intr_doc = _load_intrinsics(args.intrinsics)
    # 尺度防呆：内参文件里若记了 square_m，就以此为准，避免 collect 悄悄用默认 0.025
    if intr_doc.get("square_m") is not None and "--square" not in sys.argv:
        print("  已从内参文件继承 --square=%.4f（未在命令行显式指定）"
              % float(intr_doc["square_m"]))
        args.square = float(intr_doc["square_m"])
    else:
        args.square = _reconcile_square(args, intr_doc)
    # 进循环前先探一次状态源：若 /api/state 还没连串口（ok=false, error=未连接），
    # 立刻明确报错退出，别让下面的采集循环每组都跳过、刷屏到 --timeout 才结束。
    try:
        _read_state_pulses(args.state_url)
    except Exception as e:
        print("状态源不可用（%s）；请先在 web_control.py 上 POST /api/open 连串口" % e)
        return 1
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print("无法打开相机 %s" % args.camera)
        return 1
    objp = _chess_objp(args.cols, args.rows, args.square)
    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)
    samples = []
    n = 0
    next_t = time.monotonic() + args.interval
    last_hint = time.monotonic()
    start_t = time.monotonic()
    print("手眼采集：共 %d 组，每 %.1f 秒自动采一组" % (args.count, args.interval))
    print("  提示后把臂摆到不同姿态（要有明显旋转变化），棋盘格始终在视野内")
    print("  无界面模式=%s（DISPLAY=%s）；状态源=%s"
          % (headless, os.environ.get("DISPLAY", "NONE"), args.state_url))
    print("  请摆第 1 个姿态，%.1f 秒后自动采集..." % args.interval)
    while n < args.count:
        ret, frame = cap.read()
        if not ret:
            print("读帧失败")
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (args.cols, args.rows), None)
        now = time.monotonic()
        # 同 intrinsics：无界面下没有"按 q"，必须靠超时兜底，避免死循环。
        if (now - start_t) >= args.timeout:
            print("  采集超时（%.1f 秒），已采 %d/%d 组，退出" % (args.timeout, n, args.count))
            break
        if not headless:
            disp = frame.copy()
            if found:
                cv2.drawChessboardCorners(disp, (args.cols, args.rows), corners, found)
            cv2.putText(disp, "saved=%d/%d" % (n, args.count), (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("collect", disp)
            cv2.waitKey(1)
        if found and now >= next_t:
            # --- 机器人端：HTTP 读 6 路脉宽 → 关节角 → 末端相对基座位姿 ---
            try:
                pulses = _read_state_pulses(args.state_url)
            except Exception as e:
                print("  状态读取失败（%s），本组跳过" % e)
                next_t = now + args.interval
                continue
            phys = pulses_to_physical(pulses, ik)
            T = fk_gripper2base(phys)
            R_g2b = T[:3, :3]
            t_g2b = T[:3, 3] / 100.0        # cm → m，和相机侧单位一致
            # --- 相机端：棋盘格 → 标定板相对相机位姿 ---
            ok_c, rvec, tvec = cv2.solvePnP(objp, corners, mtx, dist)
            if not ok_c:
                print("  solvePnP 失败，本组跳过")
                next_t = now + args.interval
                continue
            R_t2c, _ = cv2.Rodrigues(rvec)
            t_t2c = tvec.reshape(3)
            samples.append({
                "pulses": [int(p) for p in pulses],
                "R_gripper2base": R_g2b.tolist(),
                "t_gripper2base": t_g2b.tolist(),
                "R_target2cam": R_t2c.tolist(),
                "t_target2cam": t_t2c.tolist(),
            })
            n += 1
            print("  存第 %d/%d 组：关节角 %.0f,%.0f,%.0f,%.0f | 标定板距相机 %.2f m"
                  % (n, args.count, phys[0], phys[1], phys[2], phys[3],
                     float(np.linalg.norm(t_t2c))))
            if args.save_dir:
                cv2.imwrite(os.path.join(args.save_dir, "collect_%02d.jpg" % n), frame)
            next_t = now + args.interval
            if n < args.count:
                print("  请摆第 %d 个姿态，%.1f 秒后自动采集..." % (n + 1, args.interval))
        elif not found and (now - last_hint) >= 3.0:
            print("  ...未检测到完整棋盘格，本组不会采集")
            last_hint = now
    cap.release()
    if not headless:
        cv2.destroyAllWindows()
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump({"count": n, "samples": samples, "square_m": args.square}, f, indent=2)
    print("手眼样本已保存 %s（共 %d 组）" % (args.out, n))
    if n < 3:
        print("⚠️ 有效样本少于 3 组，solve 无法解算（需重采，且姿态要有旋转变化）。")
    elif n < 12:
        print("⚠️ 样本偏少（建议 ≥12），解算精度可能不足。")
    return 0


def cmd_solve(args):
    """解 AX=XB 得相机相对末端的变换 X（纯 numpy，不需要 cv2.contrib）。"""
    with open(args.samples, "r", encoding="utf-8") as f:
        data = json.load(f)
    samples = data["samples"]
    if len(samples) < 3:
        print("样本不足（%d），至少需要 3 组" % len(samples))
        return 1

    R_g2b = [np.array(s["R_gripper2base"], float) for s in samples]
    t_g2b = [np.array(s["t_gripper2base"], float).reshape(3, 1) for s in samples]
    R_t2c = [np.array(s["R_target2cam"], float) for s in samples]
    t_t2c = [np.array(s["t_target2cam"], float).reshape(3, 1) for s in samples]

    print("解算 AX=XB（纯 numpy：vec+Kronecker+SVD 求旋转，最小二乘求平移）")
    R_c2g, t_c2g = solve_handeye_axxb(R_g2b, t_g2b, R_t2c, t_t2c, verbose=True)
    X = np.eye(4)
    X[:3, :3] = R_c2g
    X[:3, 3:] = t_c2g
    # 把 X 转成可读的"相机相对末端"平移(cm) + 欧拉角(度)，方便复核装法
    t_cm = (t_c2g.reshape(3) * 100.0).tolist()
    euler = _rot_to_euler_deg(R_c2g)
    out = {"X_matrix": X.tolist(), "t_cam2gripper_cm": t_cm,
           "rpy_cam2gripper_deg": euler, "samples": len(samples)}
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)
    print("手眼标定完成，X(相机→末端) 已保存 %s" % args.out)
    print("  平移 t = [%.2f, %.2f, %.2f] cm" % tuple(t_cm))
    print("  欧拉角 rpy = [%.1f, %.1f, %.1f] 度（roll,pitch,yaw，相机系相对末端系）" % tuple(euler))
    print("用法：抓取时 target_in_cam --X--> target_in_gripper，再经正/逆解算关节角。")
    return 0


def _rot_to_euler_deg(R):
    """旋转矩阵 → 欧拉角(度, ZYX 顺序)，仅用于打印复核，不参与解算。"""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy < 1e-6:
        rx = math.atan2(-R[1, 2], R[2, 2])
        ry = math.atan2(-R[2, 0], sy)
        rz = 0.0
    else:
        rx = math.atan2(R[2, 1], R[2, 2])
        ry = math.atan2(-R[2, 0], sy)
        rz = math.atan2(R[1, 0], R[0, 0])
    return [math.degrees(rx), math.degrees(ry), math.degrees(rz)]


def _add_capture_args(p, count, interval):
    """给 intrinsics / collect 两个子命令加同一组采集参数（抽出来避免重复）。"""
    p.add_argument("--headless", action="store_true",
                   help="强制不弹窗口（默认自动：无 DISPLAY 即无界面）")
    p.add_argument("--count", type=int, default=count,
                   help="自动采集张数/组数")
    p.add_argument("--interval", type=float, default=interval,
                   help="两次采集的最小间隔秒数")
    p.add_argument("--timeout", type=float, default=300.0,
                   help="总采集超时秒数（防止一直看不到棋盘格时死循环）")
    p.add_argument("--save-dir", default="",
                   help="可选：把采集到的帧存成 jpg 供事后复核")


def main():
    ap = argparse.ArgumentParser(description="眼在手手眼标定 (eye-in-hand, AX=XB)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("selftest").set_defaults(func=cmd_selftest)

    p = sub.add_parser("intrinsics")
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--cols", type=int, default=9)
    p.add_argument("--rows", type=int, default=6)
    p.add_argument("--square", type=float, default=0.025, help="棋盘格格子边长(米)")
    p.add_argument("--out", default="camera_intrinsics.json")
    _add_capture_args(p, count=20, interval=1.0)
    p.set_defaults(func=cmd_intrinsics)

    p = sub.add_parser("collect")
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--state-url", default="http://127.0.0.1:5000/api/state",
                   help="web_control.py 的 /api/state，POST 读 6 路脉宽（不打开串口）")
    p.add_argument("--intrinsics", default="camera_intrinsics.json")
    p.add_argument("--cols", type=int, default=9)
    p.add_argument("--rows", type=int, default=6)
    p.add_argument("--square", type=float, default=0.025)
    p.add_argument("--out", default="samples.json")
    _add_capture_args(p, count=12, interval=2.0)
    p.set_defaults(func=cmd_collect)

    p = sub.add_parser("solve")
    p.add_argument("--samples", default="samples.json")
    p.add_argument("--out", default="handeye_X.json")
    p.set_defaults(func=cmd_solve)

    args = ap.parse_args()
    sys.exit(args.func(args))


if __name__ == "__main__":
    main()
