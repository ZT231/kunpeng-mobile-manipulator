"""眼在手（eye-in-hand）几何：像素 + 相机内参 + 关节角 + 手眼矩阵 X -> 工作台 3D 点。

这个文件是 `handeye_X.json` 的**消费者**（consumer）。为什么要单独一个文件：
    handeye_calib.py 只负责"产出 X"，它从不知道 X 拿来干什么；
    web_control.py 负责"服务页面"，不该被一堆矩阵运算搅乱。
    中间这层纯数学单独成文件，才能本机单元测试、才能在没硬件时用占位 X 跑通。

════════════════════════════════════════════════════════════════════════
先讲清三个概念（大白话 + 规范定义）
════════════════════════════════════════════════════════════════════════

① 相机内参 K
   大白话：镜头把立体世界压成一张平面照片，K 就是这本"压扁说明书"——
          焦距多长（fx/fy，单位是像素）、光轴打在照片的哪个位置（cx/cy）。
   规范定义：3x3 内参矩阵 K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]，
           描述相机坐标系到归一化像平面的线性映射；配合畸变系数 dist
           构成 pinhole + 径向/切向畸变模型，由 cv2.calibrateCamera 求得。

② 齐次变换 T（4x4）与位姿链
   大白话：把"转多少 + 挪多少"打包成一张 4x4 表，两段位姿相乘就是"接力"，
          比如"相机看到物体"乘"相机装在手上"，就变成"手看到物体"。
   规范定义：SE(3) 刚体变换 T = [[R, t], [0, 1]]，R∈SO(3)、t∈R³；
           链式法则 T_A_C = T_A_B · T_B_C（右下角标从右往左读）。

③ 手眼标定矩阵 X（eye-in-hand 专用）
   大白话：相机是焊在机械臂末端的，X 就是"相机相对末端偏了多少、转了多少"，
          标定一次就不变了；知道它，才能把照片里的像素换算到底座坐标系。
   规范定义：求解 AX = XB 得固定变换 X = T_end_cam（末端系←相机系），
           使 T_base_cam(q) = T_base_end(q) · X 对任意构型 q 成立。

════════════════════════════════════════════════════════════════════════
本文件算的完整链路
════════════════════════════════════════════════════════════════════════
    像素 (u,v) + 深度 Z（工作台抓取平面高度）
        │  用 K 反投影
        ▼
    相机系射线 + 尺度 s          →  P_cam = s · K^-1 · [u, v, 1]
        │  用 6 路脉宽做正运动学
        ▼
    T_base_end(q)                →  T_base_cam = T_base_end · X
        │  射线变到基座系
        ▼
    基座系射线 O + λ·D           →  与平面 z = Z 求交
        ▼
    工作台坐标 (x, y, Z) cm      →  喂给 arm_ik.ArmIK.solve()

⚠️ 诚实边界（本文件不解决、也绝不假装解决的事）：
    · X 与 K 从哪来？本文件只读 JSON，不做标定。
    · 若 JSON 里带 "placeholder": true，说明是按名义值填的占位数据，
      算出来的 (x, y) 只用于打通代码链路，**不能当作定位精度**。
    · 单目没有真实深度：Z 必须由外部给定（抓取平面已知），这不是 3D 重建。
"""

import json
import math
import os

import numpy as np


# ---------------------------------------------------------------------------
# 基础矩阵工具
# ---------------------------------------------------------------------------
def pose_from_rt(R, t):
    """3x3 旋转 + 3 维平移 -> 4x4 齐次变换。

    大白话：把"转"和"挪"塞进同一张 4x4 表，之后就能用一次乘法连着算。
    规范定义：T = [[R, t], [0, 1]] ∈ SE(3)。
    """
    T = np.eye(4, dtype=float)
    T[:3, :3] = np.asarray(R, dtype=float).reshape(3, 3)
    T[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return T


def inv_pose(T):
    """求刚体变换的逆：把"从 A 到 B"倒成"从 B 到 A"。

    规范定义：T^-1 = [[R^T, -R^T·t], [0, 1]]；旋转阵正交，逆等于转置，
    比直接 np.linalg.inv 快且不引入数值误差。
    """
    T = np.asarray(T, dtype=float)
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4, dtype=float)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def rot_to_rpy_deg(R):
    """旋转阵 -> 欧拉角（度，ZYX 顺序），仅用于打印给人看，不参与解算。"""
    R = np.asarray(R, dtype=float).reshape(3, 3)
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy < 1e-9:
        rx = math.atan2(-R[1, 2], R[2, 2])
        ry = math.atan2(-R[2, 0], sy)
        rz = 0.0
    else:
        rx = math.atan2(R[2, 1], R[2, 2])
        ry = math.atan2(-R[2, 0], sy)
        rz = math.atan2(R[1, 0], R[0, 0])
    return [math.degrees(rx), math.degrees(ry), math.degrees(rz)]


# ---------------------------------------------------------------------------
# JSON 读取：X / 内参 K / 占位标记
# ---------------------------------------------------------------------------
class CalibData:
    """一份标定数据的容器：X 矩阵、内参、畸变、以及"是否占位"的诚实标记。

    大白话：就是个三层小盒子，把三个 JSON 里的关键数字归拢到一起，
           顺便记住"这些数字是实测的还是我瞎填的"。
    规范定义：标定数据聚合对象（calibration bundle），含 T_end_cam∈SE(3)、
           K∈R^{3x3}、dist∈R^{1x5}、flags 元数据。
    """

    def __init__(self, X, K, dist, X_placeholder, K_placeholder, sources):
        self.X = np.asarray(X, dtype=float).reshape(4, 4)
        self.K = np.asarray(K, dtype=float).reshape(3, 3)
        self.dist = np.asarray(dist, dtype=float).reshape(-1)
        self.X_placeholder = bool(X_placeholder)
        self.K_placeholder = bool(K_placeholder)
        # sources 记录每个文件从哪读的，报错时能一眼看出读错了哪个
        self.sources = dict(sources or {})

    @property
    def placeholder(self):
        """任一份数据是占位 -> 整体算占位（宁可保守，不夸大）。"""
        return self.X_placeholder or self.K_placeholder

    def flag_text(self):
        """给页面显示的一行中文说明，避免把占位数据当实测结果。"""
        if not self.placeholder:
            return "实测标定数据"
        parts = []
        if self.X_placeholder:
            parts.append("X(手眼矩阵)为占位值")
        if self.K_placeholder:
            parts.append("K(相机内参)为占位值")
        return "、".join(parts) + "；结果仅打通链路，不代表定位精度"

    def matrix_ok(self):
        """检查矩阵本身是否合理（正交性 / 末行 / 焦距正号），不检查数值准确性。

        为什么单独校验：JSON 手改错了（比如把 3x3 写成 4x3）会让后面所有
        矩阵乘法报形状错，错误信息离真正的错处很远，极难排查。
        """
        R = self.X[:3, :3]
        err_R = float(np.max(np.abs(R @ R.T - np.eye(3))))
        err_last = float(np.max(np.abs(self.X[3, :] - np.array([0, 0, 0, 1.0]))))
        det = float(np.linalg.det(R))
        fx, fy = float(self.K[0, 0]), float(self.K[1, 1])
        problems = []
        if err_R > 1e-6:
            problems.append("X 的旋转部分不正交（|R·R^T - I|max=%.2e）" % err_R)
        if err_last > 1e-9:
            problems.append("X 最后一行应为 [0,0,0,1]（偏差 %.2e）" % err_last)
        if abs(det - 1.0) > 1e-6:
            problems.append("X 的旋转行列式=%.6f，应为 +1" % det)
        if fx <= 0 or fy <= 0:
            problems.append("内参焦距必须为正（fx=%.1f, fy=%.1f）" % (fx, fy))
        return (len(problems) == 0), problems


def load_calib(X_path, K_path):
    """从两个 JSON 读出手眼 X 与相机内参 K；缺字段就抛一条能看懂的错误。

    为什么分开读两个文件：X 由 handeye_calib.py solve 产出，K 由 intrinsics 产出，
    它们本来是两个独立产物、可以分别重做；强行合成一个文件会导致重标内参时
    不得不重解 X（其实没必要）。
    """
    if not os.path.isfile(X_path):
        raise FileNotFoundError("找不到手眼矩阵文件：%s" % X_path)
    if not os.path.isfile(K_path):
        raise FileNotFoundError("找不到相机内参文件：%s" % K_path)

    with open(X_path, "r", encoding="utf-8") as f:
        dx = json.load(f)
    with open(K_path, "r", encoding="utf-8") as f:
        dk = json.load(f)

    if "X_matrix" not in dx:
        raise ValueError("%s 里没有 X_matrix 字段（应由 handeye_calib.py solve 生成）" % X_path)
    # 2026-11-10 新增：solve 现在会给结果打 valid 标记。被判无效的 X 绝不能进链路 ——
    # 它的矩阵本身是正交的，能骗过下面的矩阵合法性检查，但坐标会错十几厘米。
    # 老文件没有这个字段（get 返回 None），按"未判定"放行，保持向后兼容。
    if dx.get("valid") is False:
        why = "；".join(dx.get("reasons") or []) or "未记录原因"
        raise ValueError(
            "%s 被判为无效解（status=%s）：%s。请重新采集并 solve，别拿它算坐标。"
            % (X_path, dx.get("status", "UNKNOWN"), why))
    # 内参文件兼容两种写法：camera_matrix（handeye_calib intrinsics 的输出）
    # 与 mtx（部分 OpenCV 教程习惯），避免因为名字不同就读不进来。
    K_raw = dk.get("camera_matrix", dk.get("mtx", dk.get("K")))
    if K_raw is None:
        raise ValueError("%s 里没有 camera_matrix 字段（应由 handeye_calib.py intrinsics 生成）" % K_path)

    X = np.asarray(dx["X_matrix"], dtype=float)
    K = np.asarray(K_raw, dtype=float)
    dist = np.asarray(dk.get("dist_coeff", dk.get("dist", [0, 0, 0, 0, 0])), dtype=float).reshape(-1)

    cal = CalibData(
        X=X, K=K, dist=dist,
        X_placeholder=dx.get("placeholder", False),
        K_placeholder=dk.get("placeholder", False),
        sources={"X": os.path.abspath(X_path), "K": os.path.abspath(K_path)},
    )
    ok, problems = cal.matrix_ok()
    if not ok:
        raise ValueError("标定数据不合法：" + "；".join(problems))
    return cal


# ---------------------------------------------------------------------------
# 正运动学：脉宽 -> 末端位姿（与 handeye_calib.fk_gripper2base 同一模型）
# ---------------------------------------------------------------------------
class Kinematics:
    """把 6 路脉宽换算成末端位姿 T_base_end，供手眼链使用。

    大白话：给定六个舵机各自被推到哪，算出"手尖现在在底座坐标系的哪里、朝哪"。
    规范定义：正运动学（Forward Kinematics, FK）q ↦ T_base_end(q) ∈ SE(3)。

    为什么不用 handeye_calib 里那份：那份是标定脚本自用的、尺寸常量写死在文件顶部；
    这里读 arm_params.json，与 web_control.py / arm_ik.py 共用同一份标定表，
    避免"标定用 A 尺寸、定位用 B 尺寸"这种最难查的偏差。
    """

    def __init__(self, params_path, ik=None):
        with open(params_path, "r", encoding="utf-8") as f:
            p = json.load(f)
        self.L1 = float(p["L1"])
        self.L2 = float(p["L2"])
        self.L3 = float(p["L3"])
        self.H = float(p["shoulder_height"])
        self.wrist_ecc = float(p.get("wrist_ecc", 0.0))
        self.ecc_dir = float(p.get("ecc_dir", 0.0))
        self.grab_h = float(p.get("grab_height", 0.0))
        self.params_path = os.path.abspath(params_path)
        # ik 用于"脉宽 -> 物理角"的反向插值；复用 ArmIK.to_angles 免得再写一份标定表解析
        if ik is None:
            import sys
            here = os.path.dirname(os.path.abspath(params_path))
            if here not in sys.path:
                sys.path.insert(0, here)
            from arm_ik import ArmIK
            ik = ArmIK(params_path)
        self.ik = ik

    def pulses_to_angles(self, pulses):
        """6 路脉宽(us) -> 6 路物理绝对角(度)。

        为什么要这一步：arm_params.json 存的是"舵机角度"（J3 相对大臂、J4 相对小臂），
        而 FK 公式用的是"绝对俯仰角"。换算关系与 handeye_calib.pulses_to_physical 一致：
            θ1 = s1
            θ2 = s2
            θ3 = s3 + θ2
            θ4 = s4 + 180 + θ3
        若两处不统一，手眼 X 会与定位链错位一个关节角，表现为"越靠近边缘偏得越多"。
        """
        s = self.ik.to_angles([int(p) for p in pulses])
        th1, th2 = s[0], s[1]
        th3 = s[2] + th2
        th4 = s[3] + 180.0 + th3
        return [th1, th2, th3, th4, s[4], s[5]]

    def fk(self, angles_deg):
        """6 路物理绝对角(度) -> T_base_end（末端相对基座，平移单位 cm）。"""
        t1 = math.radians(angles_deg[0] + 90.0)
        t2 = math.radians(angles_deg[1] + 90.0)
        t3 = math.radians(angles_deg[2] + 90.0)
        t4 = math.radians(angles_deg[3] - 90.0)

        radial = (self.L1 * math.cos(t2) + self.L2 * math.cos(t3)
                  + self.L3 * math.cos(t4))
        vert = (self.H + self.L1 * math.sin(t2) + self.L2 * math.sin(t3)
                + self.L3 * math.sin(t4))
        x = radial * math.cos(t1)
        y = radial * math.sin(t1)
        z = vert

        if self.wrist_ecc > 1e-6:
            psi = math.radians(angles_deg[0] + 90.0 + angles_deg[4] + self.ecc_dir)
            ec = self.wrist_ecc * math.cos(math.radians(angles_deg[3]))
            x += ec * math.cos(psi)
            y += ec * math.sin(psi)

        # 姿态：先绕基座 Z 偏航，再绕腕 Y 俯仰（与 handeye_calib 的 _rz(t1)@_ry(t4) 一致）
        c1, s1 = math.cos(t1), math.sin(t1)
        c4, s4 = math.cos(t4), math.sin(t4)
        Rz = np.array([[c1, -s1, 0], [s1, c1, 0], [0, 0, 1]], dtype=float)
        Ry = np.array([[c4, 0, s4], [0, 1, 0], [-s4, 0, c4]], dtype=float)
        return pose_from_rt(Rz @ Ry, [x, y, z])

    def fk_pulses(self, pulses):
        """脉宽一步到位的便捷封装（内部就是 to_angles + fk）。"""
        return self.fk(self.pulses_to_angles(pulses))


# ---------------------------------------------------------------------------
# 核心：像素 -> 基座系射线 -> 与抓取平面求交
# ---------------------------------------------------------------------------
def pixel_to_ray_cam(u, v, K):
    """像素 (u,v) -> 相机系射线方向（单位向量，z 分量归一化为 1 之前的形式）。

    大白话：从相机光心出发，穿过照片上这个点，射出去的那根线。
    规范定义：d_cam = K^-1 · [u, v, 1]^T（齐次像点反投影），方向留未归一化，
             便于后面用 λ 缩放求交点；返回的 d 已按 z 分量归一，即 d = (x/z, y/z, 1)。
    """
    Kinv = np.linalg.inv(np.asarray(K, dtype=float))
    p = Kinv @ np.array([float(u), float(v), 1.0], dtype=float)
    if abs(p[2]) < 1e-12:
        raise ValueError("内参矩阵不可逆或 u/v 非法，无法反投影")
    return p / p[2]


def ray_plane_intersect(origin, direction, plane_z):
    """射线 O + λ·D 与平面 z = plane_z 的交点 λ；平行或反向时抛错。

    大白话：从相机的眼睛沿视线看出去，视线打在桌面上的那一点。
    规范定义：解 O_z + λ·D_z = z_plane，得 λ = (z_plane - O_z) / D_z。
             要求 |D_z| 足够大（视线不能与桌面近乎平行，否则误差爆炸）
             且 λ > 0（交点必须在相机前方，不能是"背后"的延长线）。
    """
    dz = float(direction[2])
    if abs(dz) < 1e-3:
        raise ValueError("视线几乎与抓取平面平行（D_z=%.4f），无法求交；请让相机朝下看" % dz)
    lam = (float(plane_z) - float(origin[2])) / dz
    if lam <= 0:
        raise ValueError("交点落在相机背后（λ=%.3f），检查 Z 与手眼矩阵符号" % lam)
    return lam


class EyeInHandLocalizer:
    """把"像素 + 当前脉宽 + 标定数据"变成"工作台坐标"，就是眼在手定位器。

    大白话：给它一张照片上的点、再告诉它机械臂此刻的姿势，
           它就能说出这个点在桌面上哪儿（单位 cm）。
    规范定义：eye-in-hand 局部化器，实现
           P_base = T_base_end(q) · X · (s·K^-1·[u,v,1]) 与 z=Z 平面求交。
    """

    def __init__(self, calib, kin, min_depth=1.0, max_depth=300.0):
        self.calib = calib
        self.kin = kin
        # 视线长度护栏：太小说明点打在相机壳上，太大说明几乎平视或手眼矩阵离谱。
        # 有护栏才能把"算错了"变成一句人话提示，而不是返回一个荒谬坐标。
        self.min_depth = float(min_depth)
        self.max_depth = float(max_depth)

    def localize(self, u, v, pulses, plane_z=None):
        """像素 -> 工作台 (x, y, z) cm。返回 dict，含可视化需要的中间量。

        参数：
            u, v     像素坐标（原图分辨率下的整数）
            pulses   6 路脉宽（来自 /api/state，即当前构型）
            plane_z  抓取平面高度 cm；None 用 arm_params.json 的 grab_height

        返回字段：
            x, y, z       工作台坐标（cm）
            depth         相机光心到交点的距离（cm）
            T_base_end    末端位姿（cm）
            T_base_cam    相机位姿（cm）
            placeholder   是否占位数据
        """
        if plane_z is None:
            plane_z = self.kin.grab_h
        if len(pulses) != 6:
            raise ValueError("需要 6 路脉宽，实际收到 %d 个" % len(pulses))

        # ① 相机系射线（无量纲方向）
        d_cam = pixel_to_ray_cam(u, v, self.calib.K)

        # ② 当前构型下末端位姿：用 FK 得 T_base_end（cm）
        T_base_end = self.kin.fk_pulses(pulses)

        # ③ 手眼链：T_base_cam = T_base_end · X。
        #    X 的平移单位是 m（handeye_calib solve 存的），末端位姿是 cm，
        #    这里必须先把 T_base_end 的平移换成 m 再乘，否则相机位置会被放大 100 倍 ——
        #    这是本文件最容易写错的一行，故单独换算而不是全链统一 m。
        T_be_m = T_base_end.copy()
        T_be_m[:3, 3] = T_be_m[:3, 3] / 100.0
        T_bc_m = T_be_m @ self.calib.X

        # ④ 方向向量不带平移，直接在"末端系 -> 基座系"旋转下变换；
        #    注意 X 作用顺序：先把相机系方向变到末端系，再随末端姿态变到基座系。
        R_be = T_be_m[:3, :3]
        R_cam_to_base = R_be @ self.calib.X[:3, :3]
        d_base = R_cam_to_base @ d_cam

        # ⑤ 与平面 z = plane_z（cm）求交；交点距离换算成 cm 再报，便于判断是否离谱
        origin_cm = T_bc_m[:3, 3] * 100.0
        lam_m = ray_plane_intersect(origin_cm / 100.0, d_base, plane_z / 100.0)
        depth_cm = lam_m * 100.0
        if not (self.min_depth <= depth_cm <= self.max_depth):
            raise ValueError("视线长度 %.1f cm 超出合理范围 [%.1f, %.1f]："
                             "检查 Z、手眼矩阵或缺省脉宽是否对得上当前臂姿"
                             % (depth_cm, self.min_depth, self.max_depth))

        p_base_m = origin_cm / 100.0 + lam_m * d_base
        x, y = p_base_m[0] * 100.0, p_base_m[1] * 100.0

        return {
            "x": float(x),
            "y": float(y),
            "z": float(plane_z),
            "depth": float(depth_cm),
            "T_base_end": T_base_end.tolist(),
            "T_base_cam": T_base_cam_to_list(T_bc_m),
            "placeholder": self.calib.placeholder,
            "flag": self.calib.flag_text(),
        }


def T_base_cam_to_list(T_m):
    """把以 m 为单位的 T_base_cam 转成列表，只用于返回给页面展示（保留原单位）。"""
    return np.asarray(T_m, dtype=float).tolist()


# ---------------------------------------------------------------------------
# 自检：没有硬件也要能验证这条链路算得对
# ---------------------------------------------------------------------------
def _probe_visible_pixel(loc, calib, kin, pulses, plane_z=None):
    """找一个"真的在相机视野内、且落在抓取平面上"的像素，供自检复用。

    大白话：随手戳一个屏幕点不一定是桌子，先算准一个肯定打在桌上的点再测。
    规范定义：沿相机光轴 z_cam 反算与 z=Z 平面的交点，再用 K 正投影得 (u,v)。
    返回 (True, (u, v, plane_z)) 或 (False, 失败原因字符串)。
    """
    if plane_z is None:
        plane_z = kin.grab_h
    T_be = kin.fk_pulses(pulses)
    T_be_m = T_be.copy()
    T_be_m[:3, 3] = T_be_m[:3, 3] / 100.0
    T_bc = T_be_m @ calib.X
    origin = T_bc[:3, 3]
    R_cb = T_bc[:3, :3]
    axis_base = R_cb @ np.array([0.0, 0.0, 1.0])
    if abs(axis_base[2]) < 1e-6:
        return False, "光轴与抓取平面平行"
    lam = (plane_z / 100.0 - origin[2]) / axis_base[2]
    if lam <= 0:
        return False, "光轴交点落在相机背后（该姿态下相机没朝桌面看）"
    p_cam = R_cb.T @ (origin + lam * axis_base - origin)
    uvw = calib.K @ (p_cam / p_cam[2])
    return True, (float(uvw[0]), float(uvw[1]), plane_z)


def selftest(verbose=True):
    """用完全已知的合成数据反向构造像素，再让本模块算回来，误差必须接近 0。

    为什么这样测：真机要相机、要串口、要标定板，全都齐了才能跑一次；
    但"公式有没有推错"完全可以用合成数据秒级验证 —— 这是本次「软件先行」的核心手法。
    做法（逆问题当正问题用）：
        1) 自己编一个 X（已知）和 K（已知）；
        2) 给一个假设的臂姿 q，摆一块已知的标定板，算出"桌面上的目标点"；
        3) 用同一套公式正着投影出像素 (u, v)；
        4) 把 (u, v) 丢给 localize()，看它能不能还原出第 2 步那个点。
    如果两处公式有符号错、单位错、乘法顺序错，第 4 步一定对不上。
    """
    here = os.path.dirname(os.path.abspath(__file__))
    params = os.path.join(here, "arm_params.json")

    # ---- 1) 编一份"已知答案"的标定数据 ----
    # X：相机装在末端上，光轴朝下（绕 Y 轴转 180°，即"倒挂俯视桌面"），
    #    再偏 x 1.5cm、抬 z 3cm —— 这是 eye-in-hand 真实装法，光轴必须指向桌面，
    #    否则相机根本看不见标定板（第一版自检用 15° 俯角就踩了这个坑，光轴朝上）。
    th = math.radians(180.0)
    Rz15 = np.array([[math.cos(th), 0, math.sin(th)],
                     [0, 1, 0],
                     [-math.sin(th), 0, math.cos(th)]], dtype=float)
    X_true = pose_from_rt(Rz15, [0.015, 0.0, 0.03])
    K_true = np.array([[600.0, 0.0, 320.0],
                       [0.0, 600.0, 240.0],
                       [0.0, 0.0, 1.0]], dtype=float)
    cal = CalibData(X=X_true, K=K_true, dist=[0, 0, 0, 0, 0],
                    X_placeholder=False, K_placeholder=False, sources={})
    ok, problems = cal.matrix_ok()
    print("自检 1：标定数据校验 -> %s %s" % ("PASS" if ok else "FAIL", problems or ""))
    if not ok:
        return False

    kin = Kinematics(params)
    loc = EyeInHandLocalizer(cal, kin)

    # ---- 2) 选两个真实可达的臂姿（用初始位脉宽，FK 出来一定在合理范围内）----
    pulses_a = [1580, 1900, 2400, 700, 1450, 1500]
    pulses_b = [1250, 1780, 2260, 760, 1520, 1500]
    ok_all = True

    for tag, pulses in (("A 初始位", pulses_a), ("B 侧摆位", pulses_b)):
        T_be = kin.fk_pulses(pulses)
        # 反推"相机在哪、朝哪"，再手工造一个桌面上的目标点，正投影成像素
        T_be_m = T_be.copy()
        T_be_m[:3, 3] = T_be_m[:3, 3] / 100.0
        T_bc = T_be_m @ X_true
        origin = T_bc[:3, 3]
        R_cb = T_bc[:3, :3]

        # 目标点由相机自己"看着造"：沿相机正前方（相机 z 轴）投到桌面高度，
        # 即真正落在画面中心的那一点 —— 这样像素必然在视野内，才能顺带验证
        # "像素 -> 反投影 -> 求交"三者在同一套约定下闭合。
        axis_cam = np.array([0.0, 0.0, 1.0], dtype=float)
        axis_base = R_cb @ axis_cam
        if abs(axis_base[2]) < 1e-6:
            print("自检 2（%s）：相机光轴与桌面平行，本姿态跳过" % tag)
            continue
        lam = (kin.grab_h / 100.0 - origin[2]) / axis_base[2]
        target = origin + lam * axis_base          # 单位 m
        target_cm = target * 100.0

        # 基座系 -> 相机系：p_cam = R_bc^T · (p_base - O)
        p_cam_m = R_cb.T @ (target - origin)
        if p_cam_m[2] <= 1e-6:
            print("自检 2（%s）：合成目标点跑到相机背后，换姿态重试" % tag)
            ok_all = False
            continue
        uvw = K_true @ (p_cam_m / p_cam_m[2])
        u, v = float(uvw[0]), float(uvw[1])

        # ---- 3) 正解：本模块应还原出 target ----
        out = loc.localize(u, v, pulses, plane_z=kin.grab_h)
        err = math.hypot(out["x"] - target_cm[0], out["y"] - target_cm[1])
        good = err < 1e-6
        ok_all = ok_all and good
        print("自检 2（%s）：像素 (%.1f, %.1f) -> (x=%.4f, y=%.4f) cm  期望 (%.4f, %.4f)  误差 %.2e cm  %s"
              % (tag, u, v, out["x"], out["y"], target_cm[0], target_cm[1], err,
                 "PASS" if good else "FAIL"))

    # ---- 4) 占位标记必须能透传到结果里（防止把占位当实测）----
    # 用自检 2 里实际算出的那个合法像素，别硬编码 (320,240)——初始位下画面中心
    # 并不在桌面上（相机朝前而不是朝下），硬编码会得到"交点在背后"。
    ok_ph, note_ph = _probe_visible_pixel(loc, cal, kin, pulses_a)
    if not ok_ph:
        print("自检 3：无法构造视野内的像素：%s  FAIL" % note_ph)
        ok_all = False
    else:
        u_ok, v_ok, plane_ok = note_ph
        cal_ph = CalibData(X=X_true, K=K_true, dist=[0, 0, 0, 0, 0],
                           X_placeholder=True, K_placeholder=False, sources={})
        loc_ph = EyeInHandLocalizer(cal_ph, kin)
        out_ph = loc_ph.localize(u_ok, v_ok, pulses_a, plane_z=plane_ok)
        good = out_ph["placeholder"] is True and "占位" in out_ph["flag"]
        ok_all = ok_all and good
        print("自检 3：占位标记透传 -> placeholder=%s  flag=%s  %s"
              % (out_ph["placeholder"], out_ph["flag"], "PASS" if good else "FAIL"))

    # ---- 5) 非法输入与几何退化必须被挡住，而不是算出一堆 NaN ----
    # 注意：这里直接测两个"护栏函数"本身，而不是硬凑一张坏的内参矩阵。
    # 第一版曾用 K 第二位写 1e-9 来造"水平视线"，但那只改变像素坐标的纵向缩放、
    # 不改变几何方向，护栏根本不触发 —— 对着错误的对象测，测了等于没测。
    cases = []
    try:
        loc.localize(320.0, 240.0, [1580, 1900], plane_z=kin.grab_h)
        cases.append(("脉宽不足 6 路", False, "未报错"))
    except ValueError as e:
        cases.append(("脉宽不足 6 路", True, str(e)))

    for name, direction, origin, plane in (
            ("视线与平面平行", [0.0, 1.0, 0.0], [0.0, 0.0, 0.1], 0.02),
            ("交点落在相机背后", [0.0, 0.0, 1.0], [0.0, 0.0, 0.1], 0.02)):
        try:
            ray_plane_intersect(origin, direction, plane)
            cases.append((name, False, "未报错"))
        except ValueError as e:
            cases.append((name, True, str(e)))

    try:
        pixel_to_ray_cam(320.0, 240.0, np.zeros((3, 3)))
        cases.append(("内参矩阵不可逆", False, "未报错"))
    except (ValueError, np.linalg.LinAlgError) as e:
        cases.append(("内参矩阵不可逆", True, str(e)[:60]))

    for name, passed, msg in cases:
        ok_all = ok_all and passed
        print("自检 4（%s）：%s  %s" % (name, "PASS" if passed else "FAIL", msg))

    # ---- 6) 往返一致性：FK -> X -> FK 再取回，等价于 inv_pose 是否写对 ----
    T = kin.fk_pulses(pulses_a)
    T_inv = inv_pose(T)
    back = T_inv @ T
    err = float(np.max(np.abs(back - np.eye(4))))
    good = err < 1e-9
    ok_all = ok_all and good
    print("自检 5：T^-1·T 应为单位阵，最大偏差 %.2e  %s" % (err, "PASS" if good else "FAIL"))

    print("-" * 56)
    print("结果：%s" % ("全部通过" if ok_all else "存在失败项，逐条看上面的 FAIL"))
    return ok_all


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "selftest":
        sys.exit(0 if selftest() else 1)

    # 直接运行 = 打印链路诊断（本机/板上都能跑，不需要相机和串口）
    print("眼在手几何模块 · 链路诊断")
    print("=" * 56)
    here = os.path.dirname(os.path.abspath(__file__))
    Xp = os.path.join(here, "handeye_X.json")
    Kp = os.path.join(here, "camera_intrinsics.json")
    print("X 文件：%s  %s" % (Xp, "存在" if os.path.isfile(Xp) else "不存在"))
    print("K 文件：%s  %s" % (Kp, "存在" if os.path.isfile(Kp) else "不存在"))
    if os.path.isfile(Xp) and os.path.isfile(Kp):
        cal = load_calib(Xp, Kp)
        kin = Kinematics(os.path.join(here, "arm_params.json"))
        loc = EyeInHandLocalizer(cal, kin)
        pulses = [1580, 1900, 2400, 700, 1450, 1500]
        out = loc.localize(320.0, 240.0, pulses, plane_z=kin.grab_h)
        print("数据状态：%s" % out["flag"])
        print("示例（画面正中心像素 320,240）：x=%.2f y=%.2f z=%.2f cm，视线长 %.1f cm"
              % (out["x"], out["y"], out["z"], out["depth"]))
        print("手眼平移 t = %s cm" % (np.array(cal.X[:3, 3]) * 100.0).round(2).tolist())
        print("末端欧拉角 rpy = %s 度" % [round(a, 1) for a in rot_to_rpy_deg(cal.X[:3, :3])])
    else:
        print("提示：先用 handeye_calib.py 产出这两个 JSON，或放占位文件打通链路。")
    print("=" * 56)
    print("提示：运行 `python3 handeye_geometry.py selftest` 可做合成数据自检。")
