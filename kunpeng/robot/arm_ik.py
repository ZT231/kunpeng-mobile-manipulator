"""S5 平面几何逆运动学：工作台毫米坐标 -> 6 关节角度 -> 总线舵机位置。

机械臂结构（典型教学六轴，尺寸见 arm_params.json）：
  J1 基座旋转(绕Z)  J2 大臂俯仰  J3 小臂俯仰  J4 腕俯仰  J5 腕旋转  J6 夹爪

抓取姿态：末端垂直向下（夹爪朝下），J5 定夹爪朝向，J6 开合。
坐标约定：机械臂底座安装面 z=0（装车后即小车台面上表面，不是地面），
  机械臂底座投影为原点，x 向右、y 向前、z 向上。

舵机（2026-11-26 起）：六轴全部为幻尔高压总线舵机，指令值为**位置 0~1000**，
  1 位置单位 = 0.24°（0=0°、500=120°、1000=240°）。本模块输出位置，不再是旧
  PWM 脉宽 µs（旧 6×PWM 舵机方案已于 2026-11-24 废弃）。

角度约定（物理角度，与量角器实测基准一致，全部为绝对角，度）：
  θ1 基座方位角：相对正前方，右(顺)负左(逆)正；
  θ2 大臂仰角：竖直向上=0°，前伸(顺)为负，后伸(逆)为正；
  θ3 小臂仰角：竖直向上=0°，前伸为负，后伸为正（与 J2 独立标定）；
  θ4 腕俯仰：垂直向下=0°（即抓取姿态），夹爪朝前为正，朝后为负；
  θ5 腕旋转：相对正前方，正方向=从腕沿小臂轴线看向夹爪时逆时针(右手定则，轴向指爪尖)；θ6 夹爪开合：0=完全闭合，正=张开，90=张开最大。

tilt（前倾角）：夹爪可前倾抓取以扩大臂展。tilt=0 为垂直向下，
tilt>0 时夹爪朝前倾斜（腕部相对爪尖向底座偏移 L3*sin(tilt)、抬高 L3*cos(tilt)）。

位置换算（总线舵机，见 arm_params.json）：
  pos = pos_zero + deg * PPD * dir
  deg = (pos - pos_zero) * pos_per_deg * dir
  其中 PPD = 1/0.24 ≈ 4.167 位置/度；pos_zero/dir 是每关节的标定量，
  **待装机后逐关节标定**（当前为占位值，见 arm_params.json 的 note）。
"""
import json
import math
import os

JOINTS = ["J1", "J2", "J3", "J4", "J5", "J6"]

# ---------------------------------------------------------------------------
# 末端重力偏置（⚠️ 2026-11-12 在**旧开环 PWM 臂**的 C 姿态实测，方位角 96°）
#
# 物理含义：**爪尖实际落点 = forward() 算出来的点 + TIP_BIAS**。
#   e = (dx, dy, dz) = (+0.10, −0.95, −3.88) cm
# 来源：旧开环 PWM 舵机扛不住重力 —— C 姿态下 J2 少走 6.5°、J3 再少走 2.2°。
#
# ⚠️ 2026-11-26 换总线舵机后此组数**待重测**：总线舵机有位置反馈、会自持位置，
#   下垂量级与旧开环臂不同，不能直接照搬。重测前仅作净空风险提示，不做补偿。
# ---------------------------------------------------------------------------
TIP_BIAS_XY = (0.10, -0.95)   # cm，旧开环臂实测；总线臂待重测
SAG_DZ = -3.88                # cm，旧开环臂实测；总线臂待重测

BIAS_POSE_PULSES = (1636, 1470, 2177)   # 旧 PWM 臂 C 姿态 J1/J2/J3 脉宽 µs（历史记录，已不参与计算）
BIAS_ARM_R = 14.97                      # cm，旧臂 C 姿态下腕到 J1 轴的水平距离（历史记录）


class ArmIK:
    def __init__(self, params_path: str = None):
        if params_path is None:
            params_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "arm_params.json")
        with open(params_path, "r", encoding="utf-8") as f:
            p = json.load(f)
        self.L1 = float(p["L1"])          # 大臂 cm
        self.L2 = float(p["L2"])          # 小臂 cm
        self.L3 = float(p["L3"])          # 腕->夹爪 cm
        self.H = float(p["shoulder_height"])   # 肩关节离底座安装面 cm（实测 92mm）
        # 腕轴固定侧偏：J5 旋转轴线因支架安装偏离臂中心线的横向距离 cm（2026-11-26
        # 实测 0.95cm）。该侧偏不随 J5 滚转（绕自身轴滚不改变轴心），只在基座方位
        # 上随臂旋转，水平分量随腕俯仰按 cos(θ4) 衰减。ecc_dir 为侧偏方向相对臂前向
        # 的偏角(度)：0=沿臂前向、+90/−90=左右侧偏（左/右待实测，反了就取反号）。
        self.wrist_ecc = float(p.get("wrist_ecc", 0.0))
        self.ecc_dir = float(p.get("ecc_dir", 0.0))
        # 总线舵机位置换算：1 位置单位 = 0.24°
        self.pos_per_deg = float(p.get("servo_pos_per_deg", 0.24))
        self.PPD = 1.0 / self.pos_per_deg   # 位置/度 ≈ 4.167
        self.joints = p["joints"]

    # ---- 逆运动学 ----
    def solve(self, x, y, z=None, wrist=0.0, claw=0.0, tilt=0.0):
        """工作台坐标 (x,y) cm -> 6 关节角度(度)。z 为抓取高度 cm，必须显式传入
        （已取消固定 grab_height，目标高度由深度相机/调用方给出）。
        wrist: J5 夹爪朝向(度)；claw: J6 夹爪开合(度)；tilt: 夹爪前倾角(度)，0=垂直向下。
        输出为物理角度（正前方/竖直向上=0°），与量角器实测基准一致。"""
        if z is None:
            raise ValueError("solve() 需要显式 z（抓取高度 cm）；已取消 grab_height 默认值")

        # ---- 腕轴固定侧偏补偿：把「J5 轴位置」从「夹取点目标」中解出 ----
        # J5 旋转轴线因支架安装偏了 wrist_ecc（固定侧偏，2026-11-26 实测 0.95cm），
        #   该侧偏**不随 J5 滚转**（绕自身轴滚不改变轴心位置），只随基座方位 ψ 转：
        #   ψ = 目标方位 + ecc_dir（ecc_dir 为侧偏方向相对臂前向的偏角，左/右待实测 ±90）
        # 轴点 = 夹取点 - 偏心矢量，后续 r/th1 均基于轴点计算，垂直抓取即对准夹爪。
        # tilt 时偏心水平分量按 cos(tilt) 衰减；竖直分量(≤wrist_ecc)未建模，用 --off-z 微调。
        if self.wrist_ecc > 1e-6:
            psi = math.radians(math.degrees(math.atan2(y, x)) + self.ecc_dir)
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
            # 腕轴固定侧偏：水平分量 e*cos(θ4)，方向 = 基座方位 + ecc_dir（不随 J5 滚转，与 solve 同约定）
            psi = math.radians(t1u + 90.0 + self.ecc_dir)
            ec = self.wrist_ecc * math.cos(math.radians(t4u))
            fx += ec * math.cos(psi)
            fy += ec * math.sin(psi)
        return (fx, fy, ez)

    # ---- 角度 <-> 总线位置 ----
    def _deg_to_pos(self, name, deg):
        """单个关节物理角(度) -> 总线位置 0~1000（线性，不夹紧，限位交 servo_bus 安全软限位）。"""
        j = self.joints[name]
        return int(round(j["pos_zero"] + deg * self.PPD * j["dir"]))

    def _pos_to_deg(self, name, pos):
        """单个关节总线位置 -> 物理角(度)。"""
        j = self.joints[name]
        return (pos - j["pos_zero"]) * self.pos_per_deg * j["dir"]

    def to_positions(self, angles):
        """6 关节角度(度) -> 6 路总线位置 0~1000。

        按总线舵机线性律换算（0.24°/单位）。舵机 0° 基准是相对前一级连杆
        （量角时前级在标定零位），而 solve 输出的是绝对角，故 J3/J4 需换算为
        相对角再映射：
          J3 舵机 0° = 大臂延伸方向 -> 相对角 = θ3 - θ2
          J4 舵机 0° = 小臂延伸方向 -> 相对角 = -180° - θ3（夹爪保持竖直向下）
        """
        th2, th3 = angles[1], angles[2]
        out = []
        for name, deg in zip(JOINTS, angles):
            if name == "J3":
                deg = deg - th2      # 减 J2 绝对角，得到相对大臂的转角
            elif name == "J4":
                deg = -180.0 - th3 + angles[3]  # 补偿小臂倾角，加夹爪前倾 tilt(angles[3])，tilt=0 即竖直向下
            out.append(self._deg_to_pos(name, deg))
        return out

    def to_angles(self, positions):
        """6 路总线位置 -> 6 个「舵机相对角」(度)。标定 / FK / 调试用。

        ⚠️ **它不是 to_positions 的逆**：
          · 本函数返回的是**舵机相对角**：J3 相对大臂、J4 相对小臂
            （舵机 0° 基准是相对前一级连杆，量角时前级处于标定零位）；
          · `to_positions` 的输入是 **solve() 的绝对角**，内部才做 J3/J4 的相对化换算。
        ⇒ `to_positions(to_angles(p)) != p`（J4 会明显错位）。
        要真正做 to_positions 的逆，用 `positions_to_solve_angles()`。
        """
        return [self._pos_to_deg(n, p) for n, p in zip(JOINTS, positions)]

    def positions_to_solve_angles(self, positions):
        """6 路总线位置 -> solve() 输入语义的**绝对角**（这才是 to_positions 的真逆）。

        与 to_angles 的唯一区别：把 J3/J4 从「舵机相对角」还原成 solve() 要的绝对角，
        因此满足 `to_positions(positions_to_solve_angles(p)) ≈ p`。

        推导（把 to_positions 的 J3/J4 换算反过来）：
            J3:  deg_J3 = θ3_abs − θ2          ⇒ θ3_abs = deg_J3 + θ2
            J4:  deg_J4 = −180 − θ3_abs + tilt ⇒ tilt   = deg_J4 + 180 + θ3_abs

        用途：需要「位置 → 绝对角 → 再算回去」的场合（自检、离线复算、报告）。
        需要「当前夹爪前倾角」时直接用 `tilt_from_positions()`，它就是这个的第 4 项。
        """
        a = self.to_angles(positions)
        th2, th3_rel, th4_rel = a[1], a[2], a[3]
        th3_abs = th3_rel + th2
        tilt = th4_rel + 180.0 + th3_abs
        return [a[0], th2, th3_abs, tilt, a[4], a[5]]

    def j1_pos_for_xy(self, x, y, az_bias_deg=0.0):
        """工作台坐标 (x, y) → (J1 总线位置, J1 角度, 方位角)。

        🔴 为什么 J1 能单独算：工作台原点就在 J1 轴心上，J1 只跟**方位角**
        `atan2(y, x)` 有关，与臂伸多远、倾角多少、够不够得到都无关。

        方位角约定：x=横向、y=J1 正前方（+y 即 90°），J1 角度 = 方位角 − 90°。
        az_bias_deg：方位角补偿（度），默认 0，**不静默改**——要用就显式传。
        """
        if abs(x) < 1e-9 and abs(y) < 1e-9:
            raise ValueError("目标在 J1 轴心正上方，方位角无定义")
        az = math.degrees(math.atan2(y, x)) + float(az_bias_deg)
        deg = az - 90.0
        pos = self._deg_to_pos("J1", deg)
        return pos, deg, az

    def tilt_from_positions(self, positions):
        """从当前六路总线位置反算夹爪前倾角 tilt(度)。

        tilt 是「当前姿态的固有属性」：眼在手相机骑在腕上，拍照那一刻的 tilt 决定
        相机位姿；抓取时必须用同一个 tilt。推导（to_positions 的逆，注意语义）：
          to_positions 对 J4 映射的是 deg_J4 = -180 - θ3_abs + tilt，
          而 to_angles 返回的 J3/J4 是**相对角**：
              a[2] = θ3_abs - θ2  ⇒  θ3_abs = a[2] + a[1]
          ⇒ tilt = a[3] + 180 + θ3_abs = a[3] + 180 + a[2] + a[1]
        """
        # 复用 positions_to_solve_angles，公式只写一处，避免两处实现日后漂移
        return self.positions_to_solve_angles(positions)[3]


if __name__ == "__main__":
    import sys
    ik = ArmIK()
    # 自检：随机目标点 -> IK -> FK 应回到原位置；夹爪按指定 tilt 前倾；位置应在 0~1000 内
    ok = True
    cases = [(10, 5, 0.0, 2.0), (8, 8, 0.0, 2.0), (5, 12, 0.0, 5.0), (12, 3, 0.0, 3.0),
             (9, 9, 0.0, 2.0), (0, 15, 0.0, 2.0), (18, 16, 30.0, 2.0), (20, 15, 40.0, 2.0)]
    # 第 4 项 z 为抓取高度 cm（深度相机给），不同高度都要能解回
    for x, y, tilt, z in cases:
        try:
            ang = ik.solve(x, y, z=z, tilt=tilt)
            positions = ik.to_positions(ang)
            servo = ik.to_angles(positions)      # 舵机角度（已含 J3/J4 补偿）
            th3 = ang[2]
            claw_dir = -th3 - servo[3]           # 夹爪绝对方向（竖直向上=0°，前=+），垂直向下=180°
            fx, fy, fz = ik.forward(ang)
            err = math.hypot(math.hypot(fx - x, fy - y), fz - z)
            # 位置越界检查（总线位置应落在 0~1000，安全软限位另由 servo_bus 负责）
            out_of_range = any(p < 0 or p > 1000 for p in positions)
            ok = ok and err < 0.5 and abs(claw_dir - (180.0 - tilt)) < 5.0 and not out_of_range
            print("目标(%.1f,%.1f) tilt=%.0f -> 角度 %s -> FK(%.1f,%.1f,%.1f) 误差=%.2fcm 夹爪=%.1f°%s | 位置 %s"
                  % (x, y, tilt, ["%.0f" % a for a in ang], fx, fy, fz, err, claw_dir,
                     " [越界!]" if out_of_range else "", positions))
        except ValueError as e:
            print("目标(%.1f,%.1f) tilt=%.0f 不可达: %s" % (x, y, tilt, e))
            ok = False
    print("自检", "通过" if ok else "失败")
    sys.exit(0 if ok else 1)
