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

两种布置（OpenCV calibrateHandEye 都支持，只是数据怎么采相反）：
   - 眼在手(eye-in-hand)：相机随末端动，标定板固定在环境里不动 → 本项目用这个
   - 眼在外(eye-to-hand)：相机固定，标定板贴在末端随臂动

为什么旧项目的眼在外单应(homography)不能直接用：
   眼在外时相机和臂底座都固定，像素↔台面 mm 用一张单应矩阵一次标定永久用。
   眼在手时相机随臂动，相机和末端之间的相对变换必须用 AX=XB 重新标定，
   数学上是旋转矩阵方程（不是平面单应），需要专门拍棋盘格解算。

=====================================================================
复用资产（从原视觉机械臂项目抽取并适配，接口不变）：
   arm_ik.py        —— ArmIK 逆解/正解（正解 forward() 用来把关节角算成末端位姿）
   arm_params.json  —— 舵机脉宽↔角度标定表（新臂重测后改这里）
   arm_host.py      —— 串口读关节角（STATE 命令）

依赖：pip install opencv-python numpy
用法：
   # 0) 自检：验证本文件的正运动学与旧项目 ik.forward 完全一致（不需相机/串口）
   python handeye_calib.py selftest

   # 1) 标定相机内参（第一次需要；已有 camera_intrinsics.json 可跳过）
   python handeye_calib.py intrinsics --camera 0 --cols 9 --rows 6 --square 0.025 --out camera_intrinsics.json

   # 2) 采集手眼数据：每摆一个姿态按 'c' 存一组（自动读关节角 + 拍棋盘格），'q' 结束
   python handeye_calib.py collect --camera 0 --arm-serial /dev/ttyUSB0 \
        --intrinsics camera_intrinsics.json --cols 9 --rows 6 --square 0.025 --out samples.json

   # 3) 解算 X
   python handeye_calib.py solve --samples samples.json --out handeye_X.json
=====================================================================
"""

import argparse
import json
import math
import os
import sys

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
       不移动相机。若相机明显偏离 J5 轴线，需在末尾追加 J5 旋转 + 偏心平移（见 solve 注释）。
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
# 命令实现
# ===========================================================================
def cmd_selftest(args=None):
    """验证 fk_gripper2base 的位置与旧项目 ArmIK.forward 一致（不需要相机/串口）。"""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from arm_ik import ArmIK
    ik = ArmIK()
    print("自检：fk_gripper2base 位置 vs ArmIK.forward（忽略 sub-cm 腕偏心）")
    ok = True
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
    print("自检", "通过 ✅" if ok else "失败 ❌")
    return 0 if ok else 1


def _load_intrinsics(path):
    with open(path, "r", encoding="utf-8") as f:
        d = json.load(f)
    mtx = np.array(d["camera_matrix"], dtype=float).reshape(3, 3)
    dist = np.array(d.get("dist_coeffs", [0, 0, 0, 0, 0]), dtype=float).reshape(-1)
    return mtx, dist


def cmd_intrinsics(args):
    """标定相机内参：拍多张不同角度的棋盘格，cv2.calibrateCamera 求内参矩阵。"""
    import cv2  # 仅相机相关命令需要 cv2，放函数内避免 selftest 也依赖
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print("无法打开相机 %s" % args.camera)
        return 1
    objp = np.zeros((args.cols * args.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[:args.cols, :args.rows].T.reshape(-1, 2) * args.square
    objpoints, imgpoints, saved = [], [], 0
    print("把棋盘格对准相机，按 'c' 存一张（建议 15~25 张不同角度/距离），按 'q' 结束标定。")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("读帧失败"); break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (args.cols, args.rows), None)
        disp = frame.copy()
        if found:
            cv2.drawChessboardCorners(disp, (args.cols, args.rows), corners, found)
        cv2.imshow("intrinsics", disp)
        k = cv2.waitKey(1) & 0xFF
        if found and k == ord('c'):
            objpoints.append(objp)
            imgpoints.append(corners)
            saved += 1
            print("  已存 %d 张" % saved)
        elif k == ord('q'):
            break
    cap.release(); cv2.destroyAllWindows()
    if len(objpoints) < 5:
        print("样本太少（%d），标定不可靠" % len(objpoints)); return 1
    rms, mtx, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    out = {"camera_matrix": mtx.tolist(), "dist_coeffs": dist.ravel().tolist(), "rms": rms}
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)
    print("内参已保存 %s （重投影 RMS=%.4f，越小越好）" % (args.out, rms))
    return 0


def cmd_collect(args):
    """采集手眼数据：每摆一个姿态按 'c'，同时记录 关节角(→末端位姿) + 棋盘格(→标定板位姿)。"""
    import cv2
    import serial
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from arm_ik import ArmIK
    from arm_host import ArmHost

    ik = ArmIK()
    mtx, dist = _load_intrinsics(args.intrinsics)
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print("无法打开相机 %s" % args.camera); return 1
    try:
        host = ArmHost(serial.Serial(args.arm_serial, 115200, timeout=0.5))
    except Exception as e:
        print("无法打开串口 %s：%s" % (args.arm_serial, e)); return 1

    objp = np.zeros((args.cols * args.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[:args.cols, :args.rows].T.reshape(-1, 2) * args.square

    samples, n = [], 0
    print("让臂摆不同姿态（建议 ≥12 组，覆盖不同朝向/高度），棋盘格始终在视野内：")
    print("  按 'c' 存一组 | 按 'q' 结束")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("读帧失败"); break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (args.cols, args.rows), None)
        disp = frame.copy()
        info = ""
        if found:
            cv2.drawChessboardCorners(disp, (args.cols, args.rows), corners, found)
        cv2.putText(disp, "saved=%d  %s" % (n, info), (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("collect", disp)
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break
        if k == ord('c') and found:
            # --- 机器人端：读关节角 → 末端相对基座位姿（R_gripper2base, t_gripper2base）---
            resp = host.state()
            try:
                _, pulses = host.parse_state(resp)
            except Exception as e:
                print("  STATE 解析失败（%s），跳过本组" % e); continue
            phys = pulses_to_physical(pulses, ik)
            T = fk_gripper2base(phys)
            R_g2b = T[:3, :3]
            t_g2b = T[:3, 3] / 100.0  # cm → m，和相机侧单位一致
            # --- 相机端：棋盘格 → 标定板相对相机位姿（R_target2cam, t_target2cam）---
            ok_c, rvec, tvec = cv2.solvePnP(objp, corners, mtx, dist)
            if not ok_c:
                print("  solvePnP 失败，跳过本组"); continue
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
            print("  存第 %d 组：关节角 %.0f,%.0f,%.0f,%.0f | 标定板距相机 %.1f m"
                  % (n, phys[0], phys[1], phys[2], phys[3], float(np.linalg.norm(t_t2c))))
    cap.release(); cv2.destroyAllWindows(); host.close()
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump({"count": n, "samples": samples, "square_m": args.square}, f, indent=2)
    print("手眼样本已保存 %s（共 %d 组）" % (args.out, n))
    if n < 12:
        print("⚠️ 样本偏少（建议 ≥12），解算精度可能不足。")
    return 0


def cmd_solve(args):
    """用采集的样本解 AX=XB，得到相机相对末端的变换 X 并保存。"""
    import cv2
    if not hasattr(cv2, "calibrateHandEye"):
        # opencv 部分 5.0 构建把 calibrateHandEye 移除了/未导出，需 4.x
        print("⚠️ 当前 opencv 版本缺少 calibrateHandEye（常见于部分 opencv 5.0 构建）。")
        print("   请安装 4.x 版： pip install 'opencv-python-headless==4.10.0.84'")
        print("   （鲲鹏上为 aarch64，先确认该版本有对应 wheel；或用 conda/源码装 opencv-contrib）")
        return 1
    with open(args.samples, "r", encoding="utf-8") as f:
        data = json.load(f)
    samples = data["samples"]
    if len(samples) < 3:
        print("样本不足（%d），至少需要 3 组" % len(samples)); return 1

    R_g2b = [np.array(s["R_gripper2base"], float) for s in samples]
    t_g2b = [np.array(s["t_gripper2base"], float).reshape(3, 1) for s in samples]
    R_t2c = [np.array(s["R_target2cam"], float) for s in samples]
    t_t2c = [np.array(s["t_target2cam"], float).reshape(3, 1) for s in samples]

    # OpenCV 的 calibrateHandEye 直接解 AX=XB，返回相机→末端的 R、t
    R_c2g, t_c2g = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c,
                                        method=cv2.CALIB_HAND_EYE_TSAI)
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
    p.set_defaults(func=cmd_intrinsics)

    p = sub.add_parser("collect")
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--arm-serial", default="/dev/ttyUSB0")
    p.add_argument("--intrinsics", default="camera_intrinsics.json")
    p.add_argument("--cols", type=int, default=9)
    p.add_argument("--rows", type=int, default=6)
    p.add_argument("--square", type=float, default=0.025)
    p.add_argument("--out", default="samples.json")
    p.set_defaults(func=cmd_collect)

    p = sub.add_parser("solve")
    p.add_argument("--samples", default="samples.json")
    p.add_argument("--out", default="handeye_X.json")
    p.set_defaults(func=cmd_solve)

    args = ap.parse_args()
    sys.exit(args.func(args))


if __name__ == "__main__":
    main()
