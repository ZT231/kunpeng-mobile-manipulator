"""S5 平面几何逆运动学：工作台毫米坐标 -> 6 关节角度 -> 舵机脉宽。

机械臂结构（典型教学六轴，尺寸见 arm_params.json）：
  J1 基座旋转(绕Z)  J2 大臂俯仰  J3 小臂俯仰  J4 腕俯仰  J5 腕旋转  J6 夹爪

抓取姿态：末端垂直向下（夹爪朝下），J5 定夹爪朝向，J6 开合。
坐标约定：工作台面 z=0，机械臂底座投影为原点，x 向右、y 向前、z 向上。

角度约定（物理角度，与量角器实测基准一致，全部为绝对角，度）：
  θ1 基座方位角：相对正前方，右(顺)负左(逆)正；
  θ2 大臂仰角：竖直向上=0°，前伸(顺)为负，后伸(逆)为正；
  θ3 小臂仰角：竖直向上=0°，前伸为负，后伸为正（与 J2 独立标定）；
  θ4 腕俯仰：垂直向下=0°（即抓取姿态），夹爪朝前为正，朝后为负；
  θ5 腕旋转：相对正前方；θ6 夹爪开合：0=完全闭合，正=张开，90=张开最大。

tilt（前倾角）：夹爪可前倾抓取以扩大臂展。tilt=0 为垂直向下，
tilt>0 时夹爪朝前倾斜（腕部相对爪尖向底座偏移 L3*sin(tilt)、抬高 L3*cos(tilt)）。
"""
import json
import math
import os

JOINTS = ["J1", "J2", "J3", "J4", "J5", "J6"]


def _interp(points, x):
    """points: [(x0,y0),...] 升序，线性插值，越界夹紧到两端。"""
    if x <= points[0][0]:
        return points[0][1]
    if x >= points[-1][0]:
        return points[-1][1]
    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        if x0 <= x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return points[-1][1]


def _map_deg_to_pulse(j):
    """map: [[pulse, deg],...] -> 按 deg 升序的 [(deg, pulse),...]，供角度->脉宽插值。"""
    return sorted((deg, pulse) for pulse, deg in j["map"])


class ArmIK:
    def __init__(self, params_path: str = None):
        if params_path is None:
            params_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "arm_params.json")
        with open(params_path, "r", encoding="utf-8") as f:
            p = json.load(f)
        self.L1 = float(p["L1"])          # 大臂 cm
        self.L2 = float(p["L2"])          # 小臂 cm
        self.L3 = float(p["L3"])          # 腕->夹爪 cm
        self.H = float(p["shoulder_height"])   # 肩关节离台面 cm
        self.grab_h = float(p["grab_height"])  # 抓取时夹爪离台面 cm
        # 腕部偏心：夹取点相对 J5 旋转轴线的横向偏移 cm（J5 转一圈夹取点画半径为
        # wrist_ecc 的小圆）。真实结构常为舵盘臂前伸 1cm 左右，必须补偿否则垂直抓取时
        # J5 轴对准目标、夹爪却偏在一边。ecc_dir 为 J5=0 时偏心方向相对臂前向的偏角(度)，
        # 需按实际安装标定：0=沿臂前向（舵盘臂朝前），偏了就试 90/180/270。
        self.wrist_ecc = float(p.get("wrist_ecc", 0.0))
        self.ecc_dir = float(p.get("ecc_dir", 0.0))
        self.joints = p["joints"]

    # ---- 逆运动学 ----
    def solve(self, x, y, z=None, wrist=0.0, claw=0.0, tilt=0.0):
        """工作台坐标 (x,y) cm -> 6 关节角度(度)。z 缺省用 grab_height。
        wrist: J5 夹爪朝向(度)；claw: J6 夹爪开合(度)；tilt: 夹爪前倾角(度)，0=垂直向下。
        输出为物理角度（正前方/竖直向上=0°），与量角器实测基准一致。"""
        if z is None:
            z = self.grab_h

        # ---- 腕部偏心补偿：把「J5 轴位置」从「夹取点目标」中解出 ----
        # 夹取点相对 J5 轴横向偏 wrist_ecc，偏心方向随 wrist 绕轴旋转：
        #   ψ = 轴前向方位 + wrist + ecc_dir（轴前向初值用夹取点方位近似，偏心 <2cm 误差可忽略）
        # 轴点 = 夹取点 - 偏心矢量，后续 r/th1 均基于轴点计算，垂直抓取即对准夹爪。
        # tilt 时偏心水平分量按 cos(tilt) 衰减；竖直分量(≤wrist_ecc)未建模，用 --off-z 微调。
        if self.wrist_ecc > 1e-6:
            psi = math.radians(math.degrees(math.atan2(y, x)) + wrist + self.ecc_dir)
            ec = self.wrist_ecc * math.cos(math.radians(tilt))
            x -= ec * math.cos(psi)
            y -= ec * math.sin(psi)

        r = math.hypot(x, y)
        if r < 1e-6:
            raise ValueError("目标点与底座重合，无法确定方位角")

        # ---- 数学角度（标准几何，水平=0°仰角，仅供中间计算）----
        th1_m = math.degrees(math.atan2(y, x))

        # 腕关节在夹爪后上方：夹爪前倾 tilt 时，腕相对爪尖向底座偏移 L3*sin(tilt)、
        # 抬高 L3*cos(tilt)（坐标约定：竖直向上为正）。垂直抓取即 tilt=0。
        tilt_r = math.radians(tilt)
        r_w = r - self.L3 * math.sin(tilt_r)                     # 腕的水平距离
        dz = (z + self.L3 * math.cos(tilt_r)) - self.H           # 腕相对肩的高度
        if r_w < 0.5:
            raise ValueError("目标点过近，倾斜抓取无法定位手腕 (r_w=%.1f)" % r_w)
        d = math.hypot(r_w, dz)
        if d > self.L1 + self.L2 or d < abs(self.L1 - self.L2):
            raise ValueError("目标点超出机械臂可达范围 (d=%.1f, L1+L2=%.1f)" % (d, self.L1 + self.L2))

        alpha = math.atan2(dz, r_w)                          # 腕相对肩的方向角
        cos_beta = (self.L1 ** 2 + d ** 2 - self.L2 ** 2) / (2 * self.L1 * d)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.acos(cos_beta)
        th2_m = math.degrees(alpha + beta)                 # 肘上解：大臂抬肘，J2 保持在竖直附近
        wx = self.L1 * math.cos(alpha + beta)
        wz = self.H + self.L1 * math.sin(alpha + beta)
        th3_m = math.degrees(math.atan2(dz - (wz - self.H), r_w - wx))  # 小臂仰角
        th4_m = -90.0 + tilt                               # 夹爪前倾 tilt（0=垂直向下）

        # ---- 转换为物理角度（正前方/竖直向上/垂直向下 = 0°）----
        th1 = th1_m - 90.0
        th1 = (th1 + 180.0) % 360.0 - 180.0   # 归一化到 [-180,180)，避免后方目标方向翻转
        th2 = th2_m - 90.0                    # 竖直向上=0°
        th3 = th3_m - 90.0                    # 竖直向上=0°
        th4 = th4_m + 90.0                    # 垂直向下=0°

        return [th1, th2, th3, th4, wrist, claw]

    # ---- 正运动学（验证用）----
    def forward(self, angles):
        """6 关节角度(度，物理基准) -> 末端(夹爪)位置 (x, y, z) cm。
        输入为物理角度（正前方/竖直向上=0°），内部转回数学仰角计算。"""
        t1u, t2u, t3u, t4u, t5u, _ = angles
        t1 = math.radians(t1u + 90.0)
        t2 = math.radians(t2u + 90.0)
        t3 = math.radians(t3u + 90.0)
        t4 = math.radians(t4u - 90.0)
        wx = self.L1 * math.cos(t2) + self.L2 * math.cos(t3)
        wy = self.L1 * math.sin(t2) + self.L2 * math.sin(t3)
        ex = wx + self.L3 * math.cos(t4)
        ez = self.H + wy + self.L3 * math.sin(t4)
        fx = ex * math.cos(t1)
        fy = ex * math.sin(t1)
        if self.wrist_ecc > 1e-6:
            # 腕部偏心：水平分量 e*cos(tilt)，方向 = 轴方位 + wrist + ecc_dir（与 solve 同约定）
            psi = math.radians(t1u + 90.0 + t5u + self.ecc_dir)
            ec = self.wrist_ecc * math.cos(math.radians(t4u))
            fx += ec * math.cos(psi)
            fy += ec * math.sin(psi)
        return (fx, fy, ez)

    # ---- 角度 -> 脉宽 ----
    def to_pulses(self, angles):
        """6 关节角度(度) -> 6 路脉宽(us)。

        优先用各关节标定生成的 map（分段线性插值，越界夹紧到两端），无 map 时
        退回 deg/pulse 范围线性映射。舵机 0° 基准是相对前一级连杆（量角时前级在
        中位测得），而 solve 输出的是绝对角，故 J3/J4 需换算为相对角再映射：
          J3 舵机 0° = 大臂延伸方向 -> 相对角 = θ3 - θ2
          J4 舵机 0° = 小臂延伸方向 -> 相对角 = -180° - θ3（夹爪保持竖直向下）
        """
        th2, th3 = angles[1], angles[2]
        pulses = []
        for name, deg in zip(JOINTS, angles):
            j = self.joints[name]
            if name == "J3":
                deg = deg - th2      # 减 J2 绝对角，得到相对大臂的转角
            elif name == "J4":
                deg = -180.0 - th3 + angles[3]  # 补偿小臂倾角，加夹爪前倾 tilt(angles[3])，tilt=0 即竖直向下
            if "map" in j:
                pulse = _interp(_map_deg_to_pulse(j), deg)
            else:
                ratio = (deg - j["deg_min"]) / (j["deg_max"] - j["deg_min"])
                ratio = max(0.0, min(1.0, ratio))
                pulse = j["pulse_min"] + ratio * (j["pulse_max"] - j["pulse_min"])
            pulses.append(int(round(pulse)))
        return pulses

    def to_angles(self, pulses):
        """6 路脉宽(us) -> 6 关节角度(度)（to_pulses 的逆，标定/调试用）。

        有 map 时按 [pulse, deg] 表反插值，无 map 时退回线性映射。
        返回的角度与 to_pulses 输入同语义（J3=相对大臂，J4=相对小臂）。"""
        angles = []
        for name, p in zip(JOINTS, pulses):
            j = self.joints[name]
            if "map" in j:
                table = sorted((pulse, deg) for pulse, deg in j["map"])
                angles.append(_interp(table, p))
            else:
                ratio = (p - j["pulse_min"]) / (j["pulse_max"] - j["pulse_min"])
                ratio = max(0.0, min(1.0, ratio))
                angles.append(j["deg_min"] + ratio * (j["deg_max"] - j["deg_min"]))
        return angles


if __name__ == "__main__":
    import sys
    ik = ArmIK()
    # 自检：随机目标点 -> IK -> FK 应回到原位置；夹爪应按指定 tilt 前倾；脉宽应在限位内
    ok = True
    cases = [(10, 5, 0.0), (8, 8, 0.0), (5, 12, 0.0), (12, 3, 0.0), (9, 9, 0.0), (0, 15, 0.0),
             (18, 16, 30.0), (20, 15, 40.0)]   # 远目标需前倾才可达
    for x, y, tilt in cases:
        try:
            ang = ik.solve(x, y, tilt=tilt)
            pulses = ik.to_pulses(ang)
            servo = ik.to_angles(pulses)       # 舵机角度（已含 J3/J4 补偿）
            th3 = ang[2]
            claw_dir = -th3 - servo[3]         # 夹爪绝对方向（竖直向上=0°，前=+），垂直向下=180°
            fx, fy, fz = ik.forward(ang)
            err = math.hypot(math.hypot(fx - x, fy - y), fz - ik.grab_h)
            # 未夹紧校验：反解角度应与 IK 输入一致（容差 5°，脉宽取整仅 ~0.06°），
            # 不一致说明越界被夹紧到标定表端点
            expect = [ang[0], ang[1], ang[2] - ang[1], -180.0 - ang[2] + tilt, ang[4], ang[5]]
            clamped = any(abs(got - exp) > 5.0 for got, exp in zip(servo, expect))
            ok = ok and err < 0.5 and abs(claw_dir - (180.0 - tilt)) < 5.0 and not clamped
            print("目标(%.1f,%.1f) tilt=%.0f -> 角度 %s -> FK(%.1f,%.1f,%.1f) 误差=%.2fcm 夹爪=%.1f°%s | 脉宽 %s"
                  % (x, y, tilt, ["%.0f" % a for a in ang], fx, fy, fz, err, claw_dir,
                     " [夹紧!]" if clamped else "", pulses))
        except ValueError as e:
            print("目标(%.1f,%.1f) tilt=%.0f 不可达: %s" % (x, y, tilt, e))
            ok = False
    print("自检", "通过" if ok else "失败")
    sys.exit(0 if ok else 1)
