# kunpeng/robot/ · 眼在手手眼标定 + 定位模块

> 状态（2026-11-11 17:00）：✅ **实标达成** —— `resid 1.60 mm`、`axis_cond 0.519`、
> `|t| = 16.7 cm ≈ L3`、棋盘不动 P 极差 `0.30/0.51/0.36 cm`（修 FK 前是 4.65/10.52/5.06）。
> 尺度 `s = 0.7549` 已拆成相机侧 9.5%（PnP 正常量级）+ 机器人侧 21%（J1 map 伪折点，已定位未修）。
> ✅ **A+ 点选链路**（`eye_check.py` 验）已实机：垂直 0.8mm、点选三档 0.42cm；
> ✅ **动作原语层**（`action_primitives.py`）**11-16 实机跑通**：4 步 pick 空抓全到位；
> 🔶 **抓取流水线**（`pick_pipeline.py`）离线 9 用例全过、**上板 dry-run 全程走通**；
> 🆕 **总线舵机迁移（2026-11-26）**：旧 PWM 方案（$JOINT/$STATE + µs 脉宽 + PCA9685/ACS712）已整体
> 迁到总线舵机协议（$ARM + 位置 0~1000 + $ARM,VIN 电压）。示教配方层与 SAG_DZ 下垂补偿已删除
> （总线舵机位置闭环 + 深度相机直接测深，不再需要）。

## 文件分工

| 文件 | 作用 |
|---|---|
| `handeye_calib.py` | 标定链：4 个子命令 `selftest` / `intrinsics` / `collect` / `solve`，只负责**产出** X；`solve` 带三分类自证（`valid` / `status` / `diagnostics`） |
| `handeye_geometry.py` | 🆕 定位链：**消费** X，像素 + K + 关节角 → 工作台 3D 点（含 9 项自检） |
| `gen_placeholder_calib.py` | 🆕 生成占位 `handeye_X.json` + `camera_intrinsics.json`（带 `placeholder:true`） |
| `test_eye_in_hand_mock.py` | 🆕 端到端 mock 验证：占位数据 + 假串口，45 项检查，零硬件 |
| `refk_samples.py` | 🆕 改 FK 之后用 `positions` 重算 `samples.json` 的机器人侧位姿（🔴 待迁移：旧 samples 的 `pulses` 字段需总线臂重采后改 `positions`） |
| `check_rot.py` | 🆕 不提 X、不提连杆尺寸，直接比 `θ_A(FK)` 与 `θ_B(相机)` 判"映射错还是支架松" |
| `diagnose_he.py` | 🆕 从 `samples.json` 里逐组/逐段定位离群组与棋盘漂移 |
| `check_scale.py` | 🆕 判 `s` 是"真尺度差"还是"拟合噪声"：留一法 + 随机子集重解（直接 import 求解器，无 cv2 依赖） |
| `eye_check.py` | 🆕 验 **A+ 点选链路**（`--eye-in-hand`）：找棋盘 → 取板心像素 → 打 `/api/ik/eye/solve` → 打印工作台坐标。**判据是「换姿态后坐标必须重合」**，这是唯一能戳破「算得出但算错」的物理不变量 |
| `action_primitives.py` | 🆕 **动作原语层**：`pick` / `place` / `home`，每步发 `$ARM,POS` 后**回读位置确认到位**再走下一步；零第三方依赖，只调 Flask 的 `/api/cmd` 与 `/api/state` |
| `test_action_primitives.py` | 🆕 动作原语层的回归测试：假 Flask 模拟总线 $ARM 协议 / 拒收 / 卡死 / 未确认四种脾气，**不需要真机也不动臂** |
| `pick_pipeline.py` | 🆕 **抓取流水线（A 版本闭环）**：`/observe` → VLM 选编号 → bbox 底边两角反投影 → `/api/ik/eye/solve` → `pick`(+`place`)，五道闸门（占位标定 / 臂未停稳 / 框贴边 / 逆解越界 / 人手重叠），`--backend mock` 可在没有模型时演练 |
| `test_pick_pipeline.py` | 🆕 流水线的端到端回归：起假 detector + 假 Flask，9 个用例，**每个拒绝用例都断言零条 ARM,POS** |
| `arm_ik.py` | 逆解 + 正解 `forward()`；本模块用 `to_angles()` 把位置还原成物理角 |
| `arm_host.py` | 旧版 `collect` 直连串口读 STATE 用；**本版 collect 不再 import 它**（板上留着无妨） |
| `arm_params.json` | 位置↔角度物理标定表，**保留不动**；`handeye_geometry` 也读它，保证只有一份标定表 |
| `voice_gateway.py` | 🆕 **鲲鹏侧语音入口网关（阶段 2 / B 项）**：TCP `:9000` 收 `$ASR,<文本>*<CK>` → 帧校验 → 急停快速通道 → `parse_intent` → **本地仲裁器白名单** → `/api/cmd` 下发；默认 **dry-run**，仅 `--apply` 真下发，只授权 `HOME / STOP / RESUME` 三个无参数命令字 |
| `test_voice_gateway.py` | 🆕 网关回归考卷：XOR / 帧归一化 / 仲裁 / 急停 / dry-run / 下发失败容错 / 多线程锁，**38 项 unittest 全绿，不动臂不联网** |
| `check_esp32_link.py` | 🆕 **ESP32 链路四层硬判据自动判定**（2026-11-15 宿舍无硬件时写）：喂串口 monitor / `/root/gw.log` / PC 侧 `nc` 输出三份文本，自动逐层打勾；只读日志、零第三方依赖、`--self-test` 5 组自证，**不抬档不动臂** |
| `audio_sink.py` | 🆕 **ESP32 9001 PCM 接收端（11-21 新增）**：监听 `0.0.0.0:9001`，把 16kHz/16bit/单声道 PCM 落标准 44 字节 WAV；`--out` 默认 `/root/cap.wav`、`--max-secs 0` 不限、`--self-test` 只验 WAV 头不占端口。**尚未上板部署，9001 端到端未验收** |

前 3 个必须放**同一目录**：`arm_params.json` 按 `__file__` 同目录加载，`collect` / `selftest` / `Kinematics` 都用 `sys.path.insert` 就地 import 同目录模块。

## README 三行法

### 一、标定链（`handeye_calib.py`，两轮改造）

① **用了什么语法/技术**：纯标准库（`urllib.request` HTTP POST / `json` / `argparse` / `time.monotonic()` / `tempfile`）+
`numpy`（`np.kron` / `np.linalg.svd` / `np.linalg.lstsq` / `np.linalg.det`）+ 函数内局部 `import cv2` 的基础读帧；
`if not headless` 分支隔离 GUI。
② **为什么这么设计**：
- 板上 `opencv 5.0.0` 缺 `cv2.calibrateHandEye`（属 opencv-contrib）、`DISPLAY` 为空 ⇒ 求解抽成纯 numpy、弹窗改定时自动采集，**零装包、不动已实证的 S3b 视觉链路**。
- `collect` 原本自己 `serial.Serial("/dev/ttyUSB0")` 读 STATE，但它是**只读**的，而姿态必须靠 `web_control.py` 下发 JOINT 才会变；
  两个进程同时读串口会抢同一帧 ⇒ 改成 **HTTP POST `/api/state`**，串口只由 `web_control.py` 独占。
③ **不这么写会怎样**：旧版 `intrinsics`/`collect` 一进 `imshow` 就崩、`solve` 直接 `return 1`；删掉「按 q」却不补 `--timeout` 会死循环；
`collect` 直连串口则每组 STATE 被抢空、`samples<3`、`solve` 失败；`/api/state` 未连串口时若不预检，会刷屏到 300 秒超时才结束。

### 二、定位链（`handeye_geometry.py` + `web_control.py` 接入）

① **用了什么语法/技术**：`numpy` 矩阵运算（`inv_pose` 用 `R.T` 而非 `np.linalg.inv`、`Kinv @ [u,v,1]`、射线-平面求交）+
`@property` 派生只读属性（`CalibData.placeholder`）+ `try/except ImportError` 软降级 + Flask 新增 2 个路由。
② **为什么这么设计**：原来的 `pixel_to_table()` 是「5 个手工常量 + 隐含假设拍照姿态固定」的退化解，臂一换姿态就失真；
现在用「K 反投影出射线 → 当前 6 路**真实位置**（`$ARM,READ`）做 FK → 乘手眼 X → 与 z=Z 求交」，才对得起「相机装在臂末端」的硬件事实。
③ **不这么写会怎样**：默认开启 + X 是占位数据 = 页面给出看起来精确、实则差十几厘米的坐标，比明确报错更危险；
所以做成 `--eye-in-hand` 显式开关，关闭时完全走旧通路，新功能坏了也不影响已实证的点选链路。

### 三、结果自证 + 串口守卫（第三轮改动，2026-11-10）

① **用了什么语法/技术**：可选输出参数 `diag=None`（不新增返回值，保住自检里 5 处 `2 元组解包` 的旧调用）+
`os.path.splitext` 拼 `.rejected.json` + 多级 `if/elif` 分类树；`web_control.py` 侧加 `_dead_reason`
状态位，在 `_write()` 与 `send()` 等待循环**两处**集中拦截。
② **为什么这么设计**：
- `cmd_solve` 以前**无论解得多离谱都写 `handeye_X.json`**，而 `web_control.py --eye-in-hand` 默认就读它。
  一个「矩阵正交、行列式=1」的错误 X 能通过全部合法性检查，只会让点选坐标悄悄偏十几厘米 ——
  **静默错误比报错危险**。所以无效解改写到 `<out>.rejected.json`，从物理上断开误用路径。
- 单靠「奇异值比值」分不清「激励不足」和「无解」：实测单轴退化的 `ratio=0.73` 但残差只有毫米级，
  真无解的残差是 21~96 cm。必须加两级残差门限（2 cm / 1 cm）才能分开，且判定顺序不能交换。
- **`ratio` 也不能单独判「激励够不够」**：它比值里的分母是次小奇异值，而分子（最小奇异值）在实机数据上
  永远不是 0 —— 它的大小就等于噪声。于是 `ratio` 实际测的是「噪声小不小」，不是「几何好不好」：
  用真实 FK 加 0.2° 舵机噪声正演一份**完全可用**的 12 组数据（|t| 误差只有 3 mm），`ratio=2e-2`，
  被旧的 `1e-3` 门限判成激励不足。真正管用的是 `[R_A−I]` 的秩：所有姿态绕同一根轴转时它退化成秩 2，
  σ_min/σ_max = 0（噪声下也只有 0.02~0.19）；两根轴不平行时立刻升到 0.49~0.70，且几乎不随噪声变化。
  所以 `classify_handeye_solution()` 改用 `axis_cond < 0.25` 判激励不足。
- 残差必须是**每行 RMS**（总范数 ÷ √行数）。总范数随样本数按 √n 增长，12 组数据天然比 3 组「残差大」，
  拿它跟固定门限比就是自欺欺人：3 组 2 mm 噪声的合成数据，12 组会被判成「无解」。实机第一次踩到就是这条。
- `--square`（棋盘格边长）是我拿尺子量屏幕得来的，差 20%~30% 很正常。尺度错了**不会**报错，
  只会让残差涨到厘米级、被三分类判成 `NO_SOLUTION`，然后人去查根本不存在的硬件故障。
  所以 `solve` 把尺度也当未知量一起解：`[ (R_A−I) , −R_X t_B ]·[t_X; s] = −t_A`，
  输出 `s` 与「数据隐含的方格边长」，把尺子误差和真故障分开。
- `_rx_loop` 原本 `except Exception: break` —— USB 掉线后读线程静默退出，而 pyserial 的 `is_open`
  仍为 True、`write()` 也不报错，上层只能看到「命令发出去了、响应是空串」，会误判成 STM32 没应答。
③ **不这么写会怎样**：无效 X 被当有效加载 → 抓取偏十几厘米且零提示；断线被当成「固件不响应」→
去查杜邦线、重烧固件，真正的根因（USB 供电/hub 复位）永远找不到；残差用总范数 →
真实数据被误判无解，白采一轮；尺度不自校正 → 尺子量偏一次就白采一轮。

### 四、正运动学的姿态符号（第四轮改动，2026-11-10 实机定位）

① **用了什么语法/技术**：`R = Rz(t1) @ Ry(-t4)`（注意负号），`t4 = θ4 − 90°`；
自检用 `float(np.dot(T[:3, 0], L3方向)) > 0.999` 断言"末端 X 轴与连杆同向"。

② **为什么这么设计**：位置公式把 L3 写成 `(cos t4, +sin t4)`——θ4 变大时工具往前**上方**抬；
而 `Ry(a)·(1,0,0) = (cos a, −sin a)`，符号天生相反。要让"末端坐标系 X 轴"和"真实连杆指向"
是同一根轴，俯仰角只能取 `−t4`。`handeye_geometry.Kinematics.fk` 必须同步（它本来是照抄这份的）。

③ **不这么写会怎样**：位置说"往下俯 49°"、姿态说"往上仰 49°"，两者镜像。
手眼标定的旋转方程 `R_A·R_X = R_X·R_B` 只吃姿态，镜像后**不存在**常量 X，残差全部堆到平移上。
实机表现极具迷惑性：关节角打印全对、激励判据 0.52 合格、零空间一维，但平移残差卡在 16.7mm、
`s=0.746`、标定板在基座系的位置随 J4 规律漂 10.5cm。
⚠️ `selftest` 第 1 组只比位置（对 `ArmIK.forward` 误差 0.000cm 全绿），所以这个 bug 能一路放行；
**只验位置不验姿态**是这次最大的教训，已补第 1b 组锁住它。

### 五、改 FK 之后必须重算样本（`refk_samples.py`，第五轮改动）

① **用了什么语法/技术**：`json.load` 读回 `pulses` → `positions_to_physical()` → `fk_gripper2base()`
→ 覆盖 `R_gripper2base`/`t_gripper2base`，相机侧 `R_target2cam`/`t_target2cam` 原样不动；
自检用 `rot_angle_deg(R_old.T @ R_new)` 报"改了多大"、`dot(R_new[:,0], 工具指向)` 确认不再镜像。

② **为什么这么设计**：`collect` 是把**算好的 4×4 矩阵**写进 `samples.json` 的，`solve` 只读这两份矩阵，
**它不会重跑正运动学**。所以改了 `fk_gripper2base()` 之后对旧样本 `solve`，输出会一字不变
（实测：`s=0.7461`、`resid=1.667e-02`、奇异值 `1.636e-01/9.754e-01` 全部一模一样）。
`pulses` 是原始观测、不依赖 FK 怎么写，所以拿它重算等价于"用新 FK 重采一遍"，但不用碰机械臂。

③ **不这么写会怎样**：以为"改完代码就自动生效"，对旧样本反复 `solve`，每次都是同样的 `RC=2`，
然后误判成"FK 不是根因"，转头去查舵机下垂、重贴棋盘、甚至重买相机——方向全错。
脚本对同一份输入**幂等**（重算结果只取决于 `pulses`），所以跑一次和跑十次结果相同，可以放心重复执行。

### 六、判 `s` 是"真尺度差"还是"拟合噪声"（`check_scale.py`，第六轮新增）

① **用了什么语法/技术**：`from handeye_calib import solve_handeye_axxb` 直接复用求解器
（`verbose=False` + 传 `diag` 字典拿 `scale` / `resid` / `axis_cond`）+ `np.random.default_rng(seed)`
固定种子的不放回抽样（`rng.choice(n, size=m, replace=False)`）+ 留一法（`for k in range(n)` 去掉第 k 组）。

② **为什么这么设计**：`s` 是"相机侧平移相对机器人侧的整体倍数"，它能不能当物理结论，
取决于这 12 组姿态的激励够不够。判断办法不是看 `s` 大小合不合理，而是**换子集重解看它稳不稳**：
真尺度差对任何子集都给同一个数，拟合噪声会随子集乱跳。合成数据验证过这个判据的分辨力 ——
真 k=1.3247（`s` 应 0.7549）时，每帧加 **2 mm** 高斯噪声只让 `s` 落在 0.720~0.751（σ=0.008），
而偏离量 0.245 是 σ 的 **30 倍** ⇒ σ 小就意味着 `s` 是系统量、与噪声无关。

③ **不这么写会怎样**：看到 `s=0.7549` 就判"格子量错了"，改 `--square` 重采一轮，结果 `s` 还是 0.75 ——
白采 12 组、白花 40 分钟，然后再怀疑相机、怀疑舵机。反过来说，若 σ 真很大（子集之间差 0.1 以上），
那说明**这 12 组姿态的激励本来就不够**，该做的是加大 J1 扫幅或让第二个关节参与（`axis_cond` 会跟着变），
而不是在 `--square` 上反复试。

> ⚠️ 一条走过的弯路（别再试）：想用"两帧位移的模长比"做**不依赖 X** 的尺度检验 ——
> 即"末端原点走了多长" vs "棋盘在相机系里走了多长"。**不成立**：相机光心离末端原点 16.7 cm，
> 末端一转，相机扫过的弧长本来就比末端原点大（杠杆效应）。合成数据真值只有 1.3247，
> 这个比值却报出 J1 组 1.94 / J4 组 2.59 ⇒ 只要涉及"位移大小"就离不开 X，必须走完整 AX=XB。

### 七、动作原语层（`action_primitives.py`，2026-11-11 新增）

① **用了什么语法/技术**：纯标准库（`urllib.request` 发 HTTP、`dataclasses.dataclass` 装步骤、
`argparse` 子命令、`time.time()` 做超时）＋ 同目录 `arm_ik.ArmIK` 做逆解；
**纯函数层（`plan_*`）与副作用层（`run_plan`）严格分开**，前者可离线单测、可 dry-run。

② **为什么这么设计**：
- 抓取不是「发一条 JOINT」，而是 `移到上方 → 下降 → 合爪 → 抬起` 四段，**每段必须确认真的到了**再走下一段。
  以前的网页是单点下发，发完不管 —— 臂会不会在下降途中就开始合爪，全靠运气。
- 纯函数算计划、执行层才发命令，是为了让 `--dry-run` 和 `--self-test` 能在**没有任何硬件**时跑，
  这是唯一能提前发现「计划本身就错了」的办法。
- 不碰 `web_control.py`：只走它已有的 `/api/cmd` 与 `/api/state`，删掉这个文件就是完整回滚。

③ **不这么写会怎样**：三个真踩到的坑，全是靠测试才发现的 ——
- **合爪没合紧就抬起**：`wait_arrive` 故意跳过 J6（夹住东西会堵转、永远到不了目标），
  但最初只留了 0.6 s 固定停顿，而 J6 从 950 走到 1900 需要 `950/1250 = 0.76 s`
  ⇒ 爪子还在合的过程中，第 4 步抬起已经发了，物体直接甩掉。改法是加 `wait_jaw()` 按斜坡时间算超时。
- **`if "抓" in step.name` 漏掉张爪**：place 的步骤叫「张爪」，不含「抓」字，
  于是张爪那一步根本没等，物体还没脱离就抬走了。改成判「爪」。
- **`global` 写在使用之后**：`main()` 里 `--base-url` 的默认值先读了 `BASE_URL`，
  后面才 `global BASE_URL` ⇒ `SyntaxError: name used prior to global declaration`。
  `global` 必须在函数体第一句之前。

> 🔴 判据全在测试里，不在眼睛上：`test_action_primitives.py` 用假 Flask 造了四种脾气
> （正常斜坡 / 固件回 `ERR,1` / 某一路卡死 / 不加 `--yes`），四个用例都过才算完。
> 「看着对」在这里没有意义 —— 第 3 步打印出来的 `✓ 到位` 曾经配着一个远没到位的 J6。

### 八、抓取流水线（`pick_pipeline.py`，2026-11-11 新增，A 版本最小闭环）

一句话到抓取的**中间件 + 总调度**，把已有四块拼成一条路：

```
detector /observe ──► VLM 只选编号 ──► bbox 底边两角 ──► /api/ik/eye/solve
   (同帧图+bbox)        (vlm_decider)      (反投影到 z=0)      (像素→坐标+逆解)
                                                                    │
        action_primitives.pick / place ◄── 4(+4) 步 JOINT ◄──────────┘
              (每步回读 /api/state 确认到位)
```

① **用了什么语法/技术**：`urllib.request.install_opener(ProxyHandler({}))` 强制直连；
`dataclasses` 之外的 `Fatal` / `Rejected` **两级异常分类**（环境错 vs 这一刻不该动）；
`localizer` / `http_json` 做成**可替换的函数对象**（自测注入假实现，不装 mock 库）；
`argparse` 参数组；纯函数 `iou` / `bottom_corners` / `edge_guard` / `person_guard`；
`math.hypot` 算两底角的 3D 跨度；返回码 0/1/10/11/12/13 各带明确语义。

② **为什么这么设计**：
- **VLM 只选编号、不出坐标**：VLM 的 grounding 误差 ±30~50 px，等于 2~4 cm；
  YOLO 的 bbox 是像素级定位，几何模块负责把像素变成厘米。三者各干自己那一段，谁也不越界。
- **取 bbox 底边两角、不取框中心**：单目只给一条射线，唯一的白捡约束是「物体在桌面上」。
  底边 ≈ 接触线，两角 ≈ 底面轮廓左右切点，中点 = 底面中心；框中心在物体半腰的空中，
  和 z=0 求交会带来 `tan(俯角)×高度` 的系统偏差。只取底边中点则会稳定偏近半个直径。
- **每一步都做成显式闸门**：占位标定、臂还在动、框被画面裁掉、逆解越界（`clamped`）、
  人手在旁边 —— 这五种情况都不会自己报错，只会"抓偏一点"，而抓偏没法用眼判断是哪一环。
- **`--backend mock` 保留**：几何、闸门、动作序列能在没有模型时完整验一遍，
  把「模型选得对不对」和「选对之后抓不抓得准」分开测。

③ **不这么写会怎样**（四个真踩到的坑）：
- **系统代理把本地请求吃了**：开发机开着 Clash，`HTTP_PROXY=127.0.0.1:7897`，
  urllib 把「访问 127.0.0.1:5001」也交给代理 ⇒ `502 Bad Gateway`，
  看起来像「Flask 没起」。必须 `install_opener(ProxyHandler({}))`，而且要**全局**装 ——
  `action_primitives` / `vlm_decider` 内部也是 `urlopen`，只修一处等于没修。
- **`action_primitives.BASE_URL` 是模块常量**：不加 `ap.BASE_URL = args.api`，
  前面读状态/定位全成功，唯独发 JOINT 时连到 5000 端口去 ⇒ 现象是「最后一步突然连不上」。
- **悬停点比抓取点更容易「够不到」**：腕在末端上方约 `L3=17 cm`，抬高末端 4 cm 等于抬高腕 4 cm，
  `grab_z=2 + hover=4` 时腕高 `14.9 cm`，水平可达半径只剩 `sqrt(25.2² − 14.9²) ≈ 20.3 cm`。
  远超这个数的目标会在**接近点**就抛 `ValueError`，而不是在抓取点。
- **`STATE` 返回的是 `current_pulse`（斜坡当前值），不是目标值**（`arm_control.c` 的
  `Arm_GetCurrent()`）。这既是好事（`wait_arrive` 能真的看到进度），也是陷阱：
  **拍帧时臂还在斜坡上走，拿这组脉宽算 FK 就是错的**（相机位置和画面不在同一瞬间）。
  `wait_still()` 连续两次读数一致才放行，就是为了堵这个洞。

> 🔴 判据：`test_pick_pipeline.py` 起**两个假 HTTP 服务**（假 detector + 假 Flask），
> 9 个用例覆盖「正常抓 / 带放置 / 人手重叠 / 贴边 / 逆解越界 / 占位标定 / 非 pick / 空场景 / dry-run」，
> 每个拒绝用例都断言 **`/api/cmd` 收到 0 条命令**。
> 拒绝执行却偷偷发了几条 JOINT，是这个文件最需要防的事故。

## 概念（大白话 + 规范定义）

- **AX=XB**：相机装在臂末端、标定板固定在环境里，两次摆姿的等式相除消掉常量，得 `A·X = X·B`，解出相机相对末端的固定变换 X。
  规范定义：`T_g2b(i)·X·T_t2c(i) = T_target2base`，`A_i = T_g2b(j)^{-1}·T_g2b(i)`、`B_i = T_t2c(j)·T_t2c(i)^{-1}`。
- **相机内参 K**：镜头把立体世界压成平面照片的「压扁说明书」，写明焦距多少像素、光轴打在照片哪一点。
  规范定义：`K = [[fx,0,cx],[0,fy,cy],[0,0,1]]`，配合畸变系数 `dist` 构成针孔模型，由 `cv2.calibrateCamera` 求得。
- **射线-平面求交（本模块定位的核心一步）**：从相机眼睛沿视线看出去，打在桌面上的那一点。
  规范定义：`P = O + λ·D`，令 `O_z + λ·D_z = Z` 得 `λ = (Z − O_z)/D_z`；要求 `|D_z|` 够大（视线不能近乎平贴桌面）且 `λ > 0`（交点必须在相机前方）。
- **齐次变换链**：位姿能像接力一样连乘，`T_base_cam = T_base_end · X`，右下角从右往左读。
  规范定义：`T = [[R,t],[0,1]] ∈ SE(3)`，满足 `T_A_C = T_A_B · T_B_C`。
- **vec + Kronecker + SVD**：把矩阵方程拉平成 9 元一次方程组，取最小奇异值对应的右奇异向量作为零空间解。
  规范定义：`vec(AXB) = (B^T ⊗ A)·vec(X)`；`[I⊗R_A − R_B^T⊗I]·vec(R_X) = 0`；`(R_A − I)t_X = R_X t_B − t_A` 最小二乘解平移。
- **headless 无头**：没桌面就不弹窗，只按秒表打印进度、自动存帧。
  规范定义：`VideoCapture` / `imwrite` / `findChessboardCorners` / `solvePnP` 不依赖 DISPLAY，只有 `imshow` / `waitKey` / `destroyAllWindows` 依赖 X11。
- **旋转激励**：姿态之间必须有明显、转轴多样的相对旋转；只平移不转则系数矩阵秩亏损，X 解不唯一。
  判据是 `σ_min/σ_max of [R_A−I]`（**转轴张成度**），≥0.25 才算两根转轴不平行；单轴扫到底也只有 0.02~0.19。
- **STATE 是「目标脉宽」不是「实际位置」**：板子只记得命令舵机转到哪，没编码器反馈；姿态必须靠下发 JOINT 改变。
  规范定义：开环位置控制，状态量是 PCA9685 输出比较寄存器对应的指令脉宽，非关节角反馈。
- **单一资源独占 + HTTP 代理读取**：串口帧是字节流，两个进程同时 `read()` 会各自抢走一半字节 ⇒
  只让一个进程（`web_control.py`）打开串口，其余进程通过 HTTP 要结果。
- **占位数据（placeholder）**：给程序先装一只「假眼睛」，好让后续流程先跑通，真眼睛装好换掉文件即可。
  规范定义：以名义几何参数构造的标定元数据替身，附 `placeholder:true` 标记，使下游可在无实测数据时完成接口联调，**不得用于精度评估**。

## 用法

### 九、任务前提透明化 + 失败自证（`eye_check.py`，2026-11-14）

> ⚠️ **历史记录**：本节写的是旧 PWM 时代的设计（`_POSE_C` 固定姿态、脉宽/µs 口径）。
> 其中「C 姿态」常量与「姿态闸门」已于 2026-11-26 随总线舵机迁移删除，
> 现状请看 `eye_check.py` 文件头与第十节开头的横幅；保留本节仅作演进记录。

① **用了什么语法/技术**：模块级常量元组 `_POSE_C` + `%` 拼出的可粘贴 curl 串 `_POSE_C_HINT`；
`describe_pose()` 复用 `arm_ik.ArmIK.to_angles/forward` 做 FK；`_save_look()` 在检测失败时把
`frame` 落盘成 `look.jpg`（给了 `--save` 就存成 `<名字>.look.jpg`）；`try/except` 让「arm_ik 不在」
时只降级提示而不中断主流程。
② **为什么这么设计**：踩过太多次「跑了半天不知道当前是什么姿态、棋盘该摆哪」——开场只打印
6 个脉宽数字，人根本看不懂；一旦报「等 8 秒没找到棋盘格」，也分不清是棋盘在画面外还是屏幕没亮。
所以：**任何结论前先自报前提**（当前爪尖位置 + 夹爪前倾 + plane-z + 实际脉宽），失败时再附上
「C 姿态」的可粘贴命令和一张当时的画面，把排查从「猜」变成「看」。
③ **不这么写会怎样**：同一块棋盘换个臂姿就得到 0.9cm 互差、失败又只给一句不疼不痒的提示，
每次都从零手算「这姿态相机朝哪」；更糟的是把 FK 的**相对位姿**误当成真实半径读出（实测 C 姿态
FK 给 1.8~2.4cm，真值 26cm）—— 所以输出标签特意写成「相对位姿」，并在 docstring 里写死「绝对半径不可信」。

### 十、三条入场闸门：把「靠人记」改成「脚本强制」（`eye_check.py`，2026-11-14）

> ⚠️ **历史记录**：本节的三道闸门是 11-14 那一版（平面/姿态/收尾）。
> 🔴 其中「姿态闸门 + `--allow-any-pose`」已于 2026-11-26 删除（总线舵机闭环 + 深度相机
> 直接测 3D，不再要求固定 C 姿态）；当前只剩「平面 + 收尾」两道，`--plane-z` 仍需显式给。
> 「服务端 `grab_height=2.0`」这个边界也已取消：抓取高度一律由调用方/深度相机显式传入。

① **用了什么语法/技术**：`argparse` 的 `action="store_true"` 开关 `--allow-any-pose`；
`zip()` + `max()` 逐轴比脉宽（容差 ±3µs，给 PCA9685 量化留余量）；把 `args.plane_z` 从 `None`
改成显式 `0.0` 后再恒写进 `payload`；启动命令加 `python3 -u`（unbuffered stdout）。
② **为什么这么设计**：11-14 一天里连踩四个同类坑，全是「人忘了」而不是算法错 ——
`--hover` 真下发后臂停新姿态，不回 C 就再跑必然「等 8 秒没找到棋盘」（栽 3 次）；
连跑 7 次 y 极差 2.66cm；`--plane-z` 不给就吃服务端 `grab_height=2.0` ⇒ 水平偏 `0.87×2≈1.74cm`，
而成功行还打印「--plane-z 0.0」误导；`nohup` 重定向后中文横幅卡在 stdout 缓冲区 ⇒ 误判启动失败。
⇒ 三道闸门：**平面**（桌面默认 0，服务端默认无权参与）、**姿态**（非 C 姿态直接 `return 2`
并附可粘贴回位 curl）、**收尾**（hover 成功后打印回位提示）。日志缓冲用 `-u` 在启动侧根治，不动源码。
③ **不这么写会怎样**：这几条都**不会报错**，只会给出「看起来正常但差 1~3cm 的坐标」或
「找不到棋盘」，是最难查的一类静默失效。以前靠 README 写「记得加 `--plane-z 0`」，
实测还是会漏；现在漏了脚本直接拒绝跑，把排查从「事后自证」提前到「入场拦截」。
> 刻意**不改**的一处：服务端 `grab_height=2.0` 语义（网页点选取的是抓握高度，不是桌面基准）
> —— 这是**已知边界**，不是隐患。

### 十一、模块 E：鲲鹏侧语音入口网关（`voice_gateway.py` + `test_voice_gateway.py`，B 项 / 阶段 2，2026-11-14）

① **用了什么语法/技术**：`sys.path.insert` 跨目录复用 `../llm/` 的 `parse_intent` / `parse_frame` / `xor_checksum` / `build_frame`；
`enum.Enum` 把仲裁判决锁成 `ALLOW / NOOP / REJECT` 三种、`@dataclasses.dataclass` 装 `ArbiterVerdict`（`set_estop` 用 `None/True/False` 三态，让仲裁器保持**纯函数**）；
`socketserver.ThreadingTCPServer` + `StreamRequestHandler`（`daemon_threads=True` / `allow_reuse_address=True`）、`threading.Lock` 保护跨连接急停标志；
`urllib.request` 调 `/api/cmd`；`argparse` 的 `action="append"` 一次喂多帧；纯函数 `normalize_frame()` 还原 shell 字面量 `\r\n`；`print(..., flush=True)` 治块缓冲。

② **为什么这么设计**：
- **不改 `web_control.py`、不直连串口**：5000 是已实机验收的主链路（眼在手全链、14 次重复性、三条闸门都在它上面），串口由它独占，第二个进程再开串口会抢帧（标定链踩过）；本文件是**独立进程**，只通过 HTTP 调它已有的 `/api/cmd`，主链路一行不碰，**删掉即完整回滚**。
- **只授权 `ARM,HOME / ARM,STOP / ARM,RESUME` 三条无参命令**（`Core/Src/arm_robot.c` 的 `ARM_Exec`，均回 `OK`）：它们**不带任何数值参数** ⇒ 从根上不需要软限位裁剪 / `to_positions()` 反解 / 净空折算，把项目里「静默夹紧」「算错符号」这些坑变成**结构上不可能**。`PICK / PLACE / MOVE_LEFT / MOVE_RIGHT` 一律 `ERR,3`、`LOOK` 为 NOOP，落实「LLM / 意图层只**提议**、本地仲裁器才**授权**」（原「快慢双通道」方案文档已删除）。
- **默认 dry-run，`--apply` 才真下发**：把「会不会动臂」从「靠人记」变成显式开关，与 `eye_check.py` 三条入场闸门同一思路。
- **急停最优先**：文本匹配放在 `parse_intent()` 之前，不等一个可能返回 `unknown` 的翻译员；`$ESTOP*CK` / `$RESUME*CK` 走 `special_text()` 快速通道，否则会被 `parse_frame()` 判 `NOT_ASR` 丢掉。
- 规范定义：本模块是**协议适配层（Gateway / Anti-Corruption Layer）+ 本地仲裁器（Local Arbiter）**，隔离外部语音协议与内部执行链路；互斥量保护共享可变状态避免竞态条件。

③ **不这么写会怎样**：
- 按路线图原写法直接下发 `$JOINT,...`（旧 PWM 时代的帧）⇒ 一步踩进脉宽越界、`to_positions()` 符号、软限位静默夹紧、下垂净空折算四个雷区（阶段 2 判据已据此订正，见 `docs/排障记录-语音网关.md` 第 5 条）。
- 把归一化塞进 `text_gateway.parse_frame()` ⇒ 牵动它 18 项既有单测，且底层纯函数不该替调用方猜格式；shell 单引号里的 `\r\n` 是 4 个普通字符，好帧会被判 `bad_ck_len`（本次真踩到）。
- 多线程 TCP 下急停标志不上锁 ⇒ A 刚判完「没急停」、B 紧接着发急停会漏判；`nohup` 重定向时 `print` 不 `flush` ⇒ 日志看不见（11-14 启动事故同源）。
- 只看 `/api/cmd` 的 `ok:true` 不看 `resp` ⇒ 把「帧写进串口」当成「固件执行成功」（`action_primitives.send_joint()` 同一套口径）；Flask 没起时不兜异常 ⇒ 客户端连接直接断、收不到任何应答。
> ✅ 档位（2026-11-14 16:40 抬）：**阶段 2 网关链路已上板实机验证**（38 项本机单测 + 四层硬判据全绿，详见 F 节与 `docs/上板验收-语音网关阶段2.md`）；🔴 **语音入口整体仍 🔶**：ESP32 已烧录实机验证（TCP 9000/采音），`source` 恒 `mock`，没接云端大模型 / ASR / ESP32。
> ✅ **11-14 16:40 已上板 `--apply` 跑完该硬判据，四层证据齐 ⇒ 阶段 2 网关链路抬 ✅**（原始输出与记录表 → `docs/上板验收-语音网关阶段2.md` 第五节）。🔴 但仍不许写成「语音控制已实现 / 语音→动作闭环」—— 那只证明「网关 → Flask → 固件 → 臂」链路通，与语音识别无关。

### A. 无硬件打通链路（本轮已完成，PC 上就能跑）

【PC · PowerShell】（`Set-Location "D:\简历\STM32项目\4.ESP32-S3项目\RoboticArm_Kunpeng\kunpeng\robot"` 后执行）
```
D:\Python\Python3.14.7\python.exe -B gen_placeholder_calib.py
D:\Python\Python3.14.7\python.exe -B handeye_geometry.py selftest
D:\Python\Python3.14.7\python.exe -B handeye_geometry.py
D:\Python\Python3.14.7\python.exe -B test_eye_in_hand_mock.py
```

期望：`selftest` 9 项全绿；诊断打印 `数据状态：X(手眼矩阵)为占位值…`；mock **45 项全过**。

### B. 真机实标（✅ 2026-11-11 首次跑通，含两个数的读法）

> `collect` **不打开串口**，但依赖 `web_control.py` 的 `/api/state`；若返回「未连接」，先 `POST /api/open {"port":"/dev/ttyUSB0"}`。
> `intrinsics` / `collect` 都要独占 `/dev/video0`，跑之前**必须先停 `detector_service.py`(8080)**，采完按原参数起回。
> `--square` 只要在 `intrinsics` 时给对，`collect` 会自动从内参文件继承（本轮新增的尺度防呆）。

【板 · openEuler 终端】（`cd ~/kunpeng_vision` 后执行，值已写死）
```
python3 handeye_calib.py selftest
```
```
python3 handeye_calib.py intrinsics --camera 0 --cols 9 --rows 6 --square 0.02 --count 20 --interval 1.0 --timeout 300 --save-dir ./calib_imgs --out camera_intrinsics.json --headless
```
```
python3 handeye_calib.py collect --camera 0 --state-url http://127.0.0.1:5000/api/state --intrinsics camera_intrinsics.json --cols 9 --rows 6 --square 0.0082 --count 12 --interval 18.0 --timeout 900 --save-dir ./collect_imgs --out samples.json --headless
```
```
python3 handeye_calib.py solve --samples samples.json --out handeye_X.json
```

⚠️ **如果这份 `samples.json` 是「改 FK 之前」采的，中间必须插一步**（否则 `solve` 输出与本轮一字不变）：
```
python3 refk_samples.py samples.json --out samples_fkfix.json
```
```
python3 handeye_calib.py solve --samples samples_fkfix.json --out handeye_X.json
```
`refk_samples.py` 用样本里存的 `pulses` 按**当前版 FK** 重算机器人侧位姿，相机侧原样保留；
幂等（跑几次都一样），且会打印"姿态改了多少度""X轴·L3指向是否 = +1"，改错了当场红。

**`solve` 的返回值约定**（第三轮新增）：`0` = 解有效、已写 `handeye_X.json`；
`1` = 跑不起来（样本不足）；`2` = 跑通了但判为无效，只写 `handeye_X.rejected.json`。
看终端最后那段三分类结论即可，不用自己算奇异值。

**`solve` 的尺度自校正**（第四轮新增）：会多打一行
`尺度自校正：s = 1.2875 ⇒ 数据隐含方格边长 0.0103 m（采集时填 0.0080 m）`。
`s` 是「真实方格边长 ÷ 你填的 `--square`」，已乘进 `t`；`s≈1` 说明尺子量准了。
⚠ 解被判无效时这行的 `s` 也在拟合噪声，**不要**拿它去改 `--square`（终端会提示）。

**实标结果（2026-11-11，12 组 = J1{1528,1561,1599,1636} × J4{1034,1106,1179}）**

| 项 | 值 | 判读 |
|---|---|---|
| `status` / `RC` | `OK` / `0` | ✅ 已写 `handeye_X.json`（`valid:true`） |
| 平移残差 RMS | **1.60 mm**（改 FK 前 16.7 mm） | ✅ 门限 10 mm，剩 1/6 |
| `axis_cond` | **0.519** | ✅ 门限 0.25，与正演预测 0.52 一致 |
| 旋转零空间 | 一维 | ✅ `R_X` 唯一 |
| `t_X` 模长 | **16.7 cm** | ✅ 相机光心就在腕轴附近（见下） |
| `s` | **0.7549** | ⚠️ 相机侧只占 **9.5%**（实测 `k = 1.095`），剩余 ~21% 指向机器人侧 J1 map（见下） |
| P 极差（棋盘在基座系） | **0.30 / 0.51 / 0.36 cm**（改 FK 前 4.65 / 10.52 / 5.06） | ✅ 棋盘没动、模型对了 |
| `refk` 姿态改动 | 115.9° / 98.0° / 79.8°，位置改动 0.0000 cm | ✅ 镜像假设被精确复现 |

🔴 **`|t| = 16.7 cm` 是正常值，不是故障**：`t_X` 是「相机光心在末端系里的位置」，末端系原点在夹爪尖
（L3 = 17 cm 的末端）。16.7 ≈ 17 ⇒ 相机光心几乎落在 J4 轴线上（换算后离 J4 轴 **5.4 cm**：
沿工具轴前移 1.5 cm、横向 2.1 cm、臂平面法向 4.7 cm），与「相机死装 J4、动 J5/J6 画面不动」这条实测完全吻合，
也与之前从 PnP 距离反推的 δ≈5.6 cm 一致。
求解器本可以把相机放在任何地方，它自己落到腕轴上 —— 这是数据给出的结论，与结构事实互相印证。

🔴 **`s = 0.7549` 已实测拆分：相机侧只占 9.5%，其余 ~21% 在机器人侧（2026-11-11 16:40 定案）**

判据不是"哪个数看着合理"，而是 `diagnose_he.py` 第 6 段那个**物理不变量**：棋盘放在桌上一动没动 ⇒
把它算到基座系的位置 P 必须 12 组重合。本轮 P 极差 **0.30 / 0.51 / 0.36 cm**（改 FK 前 4.65 / 10.52 / 5.06 cm，
改善 20 倍），均值 `(+1.3, +23.7, +6.2) cm`，12 组逐对残差 1.83~3.43 mm、没有离群组。
⚠ 关键在脚本那一行 `pose(Rg[i], tg[i]) @ X @ pose(Rc[i], tc[i] * dg["scale"])`：**P 是按 `s` 缩放之后算的**
⇒ P 这么稳，说明 `s = 0.755` 是数据的真实性质、不是拟合噪声；退回 `s = 1` 残差立刻回到 16.7 mm、P 也会散开。
**P 的绝对位置也说得通**（旁证，不是证明）：同姿态（θ4=41°）FK 的腕点在 `(0, 26.5, 5.9) cm` ⇒ 格子角点离夹爪尖只有 **3.1 cm**、
几乎同高 ⇒ 手机就躺在夹爪旁边（与照片一致），也说明 FK 原点比桌面低约 5 cm（夹具夹在桌沿 / 底座下沉）。
反过来：哪天解出 `|t|` = 5 cm 或 30 cm，那才是相机装错了或数据坏了。

### 决定性实验：臂完全不动、手机横向挪 30.0 mm

`collect --count 2 --interval 15 --square 0.0082 --out scale_probe.json`，两帧都框住同一块棋盘，
臂的脉宽两帧必须是同一组（`0,0,-90,41`，臂不动）。

| 读数 | 实测值 | 含义 |
|---|---|---|
| `d12`（两帧棋盘中心距离） | **32.9 mm**（钢尺量的是 30.0） | 相机侧把 30 mm 看成 32.9 mm |
| `k = d12 / 30` | **1.095** | 相机侧只偏大 **9.5%**，不是 32.5% |
| 分向 `dx dy dz` | **−32.7 / +1.0 / −2.8 mm** | 横向主导 ✅ 方向对；深度只动 2.8 mm（噪声级） |
| 隐含格子 | **7.49 mm**（填的是 8.2） | 真实格子介于 7.5~8.2，**绝不可能 6.2** |

⇒ 若相机侧真偏大 32.5%，30 mm 必须读出 **39.7 mm**；实测 32.9 mm，差 6.8 mm ——
钢尺量 30 mm 不可能差 7 mm。**「格子实际 6.2 mm」假设被实测否决**，不要再改 `--square 0.0062` 重采。

### 剩下的 ~21% 在机器人侧，且已锁定到 J1 map

把总缩放拆开：

```
1 / s = 1.3247   ← solve 需要的总缩放（相机侧 ÷ 机器人侧）
      ÷ 1.095    ← 本轮实测的相机侧偏差
      = 1.210    ← 机器人侧位移比真实小 ~17.4%
```

三条独立证据指向同一处：

1. `check_rot.py` 是**纯角度比对**（相似矩阵迹相同 ⇒ θ 相同，与 X、与连杆长度全无关）：
   J4 中位 **0.989** ✅、**J1 中位 1.069**（相机看到的 J1 转角比 FK 大 6.9%），散度全集中在 `J1=1636` 那一行。
2. `arm_params.json` 的 J1 map 四段斜率 = **0.1147 / 0.1253 / 0.1067 / 0.1333 °/µs**，最大最小差 **25%**。
   MG996R + PCA9685 的脉宽-角度本质是单调线性的，不可能有这种锯齿 ⇒ 是 **5 点手工标定精度不足**造出来的伪折点。
   而 12 组姿态的 `J1 = 1528 / 1561 / 1599 / 1636` **正好跨在 1550 折点两侧**（斜率从 0.1253 掉到 0.1067）。
3. 用两端点（800→−93.2、2300→+86.8）线性重算，`J1` 从 1528 到 1636 应为 **12.96°**，而 5 点插值只给 **11.93°**
   ⇒ 比值 **1.086**，与 `check_rot` 实测的 1.069 只差 1.6%。

⇒ **`s = 0.7549` = 相机侧 1.095 × 机器人侧 1.210**。相机侧这一半已实测清楚（9.5%，属 PnP 在 20 cm 处的正常量级）；
机器人侧这一半来自 J1 map 伪折点，**已定位、未修** —— 改 map 会让所有已记录姿态的「脉宽↔角度」换算整体平移，
需要一次独立的卷尺实验（J1 大角度扫 + 量夹爪尖弦长）才能定案，不在标定收尾范围内。

⚠ **四条已被硬证据排除的假设，别重复试**：

| 假设 | 硬证据 | 结论 |
|---|---|---|
| 内参分辨率 ≠ 采集分辨率（`848/640 = 1.3250` 与 1/s 只差 0.02%，太巧） | `collect_imgs/*.jpg` 12 张全 `480×640`，内参 `image_size = [640, 480]` | ❌ 作废 |
| 等效焦距小 32.5%（f_true ≈ 610 px、对角 66.5°） | 内参 `rms = 1.336 px`、`placeholder = None`、`fx = 808` ⇒ 水平 43.2° / 对角 52.8° | ❌ 43.2°→66.5° 得换超广角镜头，与实测画面不符 |
| s 只是弱激励下把噪声拟合成一个数 | `check_scale.py` 合成验证：真 k=1.3247 时每帧加 **2 mm** 噪声，s 只在 0.720~0.751 动（σ=0.008） | ❌ `s − 1 = −0.245` 是 σ 的 30 倍 |
| 格子实际 6.2 mm（各向同性缩放被 s 吸收） | 30 mm 横移实测读出 32.9 mm ⇒ `k = 1.095`、隐含格子 7.49 mm | ❌ 实测否决（本轮新增） |

> 为什么这实验必须**横向**：横向重建「Δx = (u − cx) · W / w」只含格子边长 `W`、**不含焦距 `f`** ⇒ 能一刀切出 `W`。
> 为什么只敢 **30 mm**：棋盘占画面宽 64%，左右各约 160 px ≈ 25 mm，40 mm 必切边（见下面扫幅段）。

**本次 solve 出来的 X 内部是自洽的**（`t_X` 已按 `s` 换算过、`|t| = 16.7 cm ≈ L3`），可以直接用。
`s` 的不确定性影响的是「把像素换算成真实厘米」的下游精度（约 9.5%，不是 32.5%），不影响 `R_X` 与关节角解算。

- 棋盘格：**9×6 内角点**（10×7 格，四周各留 1 格白边）；无打印机时用手机/显示器全屏显示 `chessboard_9x6.png`，平放抓取平面，全程不动。
- `--square` 单位是**米**，且要量的是**黑白格子区**、不含白边：量「横向连续 10 格的总宽 W(mm)」→ `--square = W/10000`
  （屏幕/手机显示时不能量屏幕宽度，也不能只量一格—一格太短，1 mm 误差就 12%）。
- `intrinsics` 与 `collect` 的 `--square` 不一致**不再致命**：`K` 与尺度无关，`solve` 会自校正；
  而且 `collect` 未显式给时会自动继承内参文件里记录的 `square_m`。
- `collect` 每 18 秒自动采一组：用网页滑杆把臂摆到不同姿态，**等斜坡走完、STATE 稳定**再让它拍。
  🔴 **相机装在 J4 轴上 ⇒ 只有 J1（偏航）和 J4（俯仰）能改变相机朝向**；变 J2/J3 只改平移，对旋转激励零贡献。
- 🔴 **J1 扫幅受「棋盘出画」硬限制，别贪大**：镜头绕 J1 的回转半径约 18 cm，
  而棋盘离镜头只有 30~40 cm，所以臂转 1° 棋盘在画面里要动 1.5~2.4°。
  🔴 **决定预算的是「棋盘格子在画面里多大」，不是「放在几厘米」**（11-10 19:19 实拍，640×480）：
  - OpenCV 检测的是**黑白格子区**（9×6 内角点 = 10×7 格 = **8 cm × 5.6 cm**），不是手机屏幕那圈白纸。
  - 棋盘中心放 32 cm、格子区占 **408 px（64%）**；J4 三档（1034/1106/1179）只让投影变 ±3%（最坏 ~433 px）——
    因为相机随 J4 轴转动，到地面那一点的斜距几乎不变。
  - 🔴 **画面纵向只覆盖地面约 26~36 cm 一条带（纵深 ~9.7 cm）** ⇒ **挪远没用**：
    挪到 36 cm 棋盘上边缘就被切掉。唯一能调的是**手机上的棋盘显示大小**。
  - 算账：±6° 需要 ±120 px（按 ~20 px/°），最坏档余量只有 ~103 px ⇒ **把手机棋盘缩到 ~85%**
    （10 格总宽 80 → 68 mm，`--square` 随之改为约 0.0068，必须重测）。**内参不用重采**（K 与尺度无关）。
  - 居中后左右各剩 160 px 预算 ⇒ **J1 最多 ±6~7°**，±10° 必然切角
    （实测 J1=1496 时棋盘左边被切掉约 95 px）。**推荐 12 组 = J1 {1528,1561,1599,1636} × J4 {1034,1106,1179}**，
  J2/J3/J5 全程 1470/2177/1450 不动，**J6 全程锁死一个数**（实测动 J6 相机画面不动，它不参与位姿）。
  这套姿态的转轴张成度 **0.52**（门限 0.25），正演噪声下 `|t|` 误差中位 **0.69 cm**
  （±10° 是 0.56 cm，差 1 mm，不值得为它冒切角风险）。
  🔴 采前必须先把棋盘对准**扫幅中点 θ1=0（J1=1580）+ 中间那档俯仰 J4=1106**，再横向挪手机让它居中，否则预算会被偏置吃掉。
  ⚠️ **30.9 cm / 32 cm 只是「光轴落点」**（J4=1106 时画面中心那条光线打到桌面的位置，实测 32 cm，用于验证 FK 链路），
  **不是棋盘该放的距离**——棋盘放哪只看「格子区能不能完整入画」。
- `--timeout` 兜底；`--save-dir` 存帧供事后复核（detector 停后网页画面会黑，靠它 + 终端提示调整）。

### C. 在网页上用眼在手定位

【板 · openEuler 终端】（重启 5000 才生效；`--eye-in-hand` 不加则完全走旧通路）

🔴 **必须用绝对路径启动**。`~` 下曾留着一份 11-08 的老 `web_control.py`（81775B），
`cd ~ && python3 web_control.py` 会跑它 ⇒ `unrecognized arguments: --eye-in-hand`，
而 `scp` 更新的却是 `~/kunpeng_vision/` 那份，两份同时被 `pkill -f` 杀掉，极难察觉。
```
pkill -f web_control.py; sleep 1; nohup python3 -u /root/kunpeng_vision/web_control.py --host 0.0.0.0 --port 5000 --ik-dir /root/kunpeng_vision --eye-in-hand --video-url http://192.168.77.100:8080/video --detections-url http://127.0.0.1:8080/detections > /root/web.log 2>&1 &
```
判版本只认**字节数 + `grep -c`**：`web_control.py` 新版 `--eye-in-hand` 出现 5 次、
`eye_check.py` 新版 `--radial-fix` 出现 5 次。

⚠️ 想让 `eye_check.py`（命令行自检）也能用，还得多两个参数，否则自检会直接拒跑：
- 页面 URL 是 `http://板IP:5000/video` ⇒ Flask 里得有 `--video-url`；
- `eye_check` 第一步就 `GET /api/ik/eye`，**摄像头/参数错不打紧，但 `--eye-in-hand` 必须有**。

所以「跑物理验收」用的启动命令是这一条（串口自动探测，不用写 `/dev/ttyUSB0`）：
```
pkill -f web_control.py; sleep 1; nohup python3 -u /root/kunpeng_vision/web_control.py --host 0.0.0.0 --port 5000 --ik-dir /root/kunpeng_vision --eye-in-hand > /root/web.log 2>&1 &
```
🔴 **两个「不带」 / 一个「必带」**：
- **不带 `--radial-fix`**：代码里 `_EYE_RADIAL_K` 默认已是 `1.091`，命令行写了反而会覆盖默认值，以后改默认还要同步两处。想关掉对比用**客户端** `eye_check.py --radial-fix 0`（和服务端不是一回事）。
- **`-u` 必带**：`nohup … > 文件` 时 Python stdout 变**块缓冲**，启动横幅「[串口]… / 逆解：已就绪 / 眼在手：已启用」会卡在内存里不落盘，只有 Werkzeug 英文横幅（走 stderr）立刻可见 ⇒ `grep` 中文空会误以为启动失败（11-14 实测踩过）。`-u` = unbuffered，根治。
判活最可靠的是 `curl -s http://127.0.0.1:5000/api/ik/eye` 看 `"enabled":true`，不是看日志横幅；串口是否真连看 `POST /api/state` 能否回 `ok:true` + 6 路真实脉宽 —— 该接口**没有兜底值**，`_HOST` 为空会直接 `ok:false`，所以 `ok:true` 就等于「STATE 真回读」。

【PC · 浏览器】打开 `http://192.168.77.100:5000`，先点「连接」，勾选取点，点画面 → 调
`POST /api/ik/eye/solve`（页面尚未加按钮，暂用下面命令验证）：
```
curl -s -X POST http://192.168.77.100:5000/api/ik/eye/solve -H "Content-Type: application/json" -d '{"u":320,"v":240,"apply":false}'
```

期望返回里出现 `"mode": "eye_in_hand"`、`"x"/"y"/"z"`、`"depth"`、`"placeholder": true`。
**占位数据下 `placeholder` 必须为 `true`** —— 看到 `false` 才说明用上了实测标定。

### D. 一句话抓取（`pick_pipeline.py`，A 版本；先 dry-run 再 --yes）

前置三件：① Flask 带 `--eye-in-hand` 起好；② `detector_service.py` 在跑（8080）；③ 串口已 open。

【板 · openEuler 终端】（`cd ~/kunpeng_vision`）
```
python3 pick_pipeline.py --self-test
```
```
python3 pick_pipeline.py --instruction "把杯子抓起来" --dry-run
```
```
python3 pick_pipeline.py --instruction "把杯子抓起来" --yes
```
```
python3 pick_pipeline.py --instruction "把杯子抓起来" --yes --place-x 6 --place-y 20
```

- `--backend mock`（默认）：**不调模型**，按类名关键词挑目标 —— 用来先验证「几何 + 闸门 + 动作序列」这一半。
  换成真模型：`--backend vlm --model qwen2.5vl:3b --base-url http://<PC 的 IP>:11434/v1`。
- `--dry-run` 与 `--yes` 必须二选一，**两个都不给会直接退出**（防止"以为跑了其实没跑"和"以为没跑其实动了"）。
- 返回码：`0` 完成 / `1` 环境没准备好 / `10` 决策不是 pick（或画面空）/ `11` 安全拒绝 /
  `12` 逆解不可达 / `13` 计划被软限位裁剪 / `2` 执行超时 / `4` 通信失败。
- 若抓偏了，先看它打印的**底面中心 (x, y)**：拿卷尺量一下真实位置，
  偏差方向一致 ⇒ 多半是 `plane_z` 或桌面基准；偏差随姿态变 ⇒ 回去看手眼 X。

### E. 语音入口网关本机演练（`voice_gateway.py`，零硬件 / 零第三方依赖）

> 前置：想跑 `--apply` 真下发，必须先按 C 节把 Flask 带 `--eye-in-hand` 起好、串口已 open。
> 🔴 默认 **dry-run**，臂绝不会动；只有显式 `--apply` 才真发。

【PC · PowerShell 或 板 · openEuler 终端】（`cd` 到 `kunpeng/robot` 后执行，一条一条粘）
```
python3 -B voice_gateway.py --self-test
python3 -B voice_gateway.py --make-frame "回初始位"
python3 -B voice_gateway.py --text "回初始位" --text "看看桌面" --text "把红块抓起来"
python3 -B voice_gateway.py --text "急停" --text "回初始位" --text "恢复" --text "回初始位"
python3 -B -m unittest test_voice_gateway -v
```
- 期望：`SELF_TEST PASS`；`--make-frame` 给 `'$ASR,回初始位*61\r\n'`（复制粘贴用）；
  多句链路依次回 `$OK,HOME` / `$OK,LOOK_NOOP` / `$ERR,3`；
  急停链路 `$OK,ESTOP` → `$ERR,3`（冻结）→ `$OK,RESUMED` → `$OK,HOME`；
  unittest **38 项全绿**、末尾附 `SELF_TEST PASS`。
- **起 TCP server**（默认 `127.0.0.1:9000`，只本机可连；`ThreadingTCPServer` 每连接一线程）：
```
python3 -B voice_gateway.py
```
  另开终端造一条带正确 CK 的帧再喂（先用 `--make-frame` 拿 CK）：
```
python3 -B voice_gateway.py --make-frame "回初始位"
echo -ne '$ASR,回初始位*61\r\n' | nc 127.0.0.1 9000
```
- 🔴 **要 ESP32 连进来 / 要真动臂**，两个开关都要显式给：
```
python3 -B voice_gateway.py --apply --host 0.0.0.0
```
  `--apply` 时若 Flask 没起，`send_cmd()` 抛 `RuntimeError`，网关回 `$ERR,4*5D\r\n` 且不崩（已单测覆盖）。
- 非 ASR 急停快速通道：`$ESTOP*CK\r\n` / `$RESUME*CK\r\n` 同样有效（CK 仍由 `parse_frame` 校验）。
- 错误码沿用项目统一语义：`ERR,1` 帧错 / `ERR,2` 未识别 / `ERR,3` 仲裁拒绝 / `ERR,4` 下发失败。

### F. 上板验收：阶段 2 硬判据（`docs/上板验收-语音网关阶段2.md`，2026-11-14）

> ✅ **2026-11-14 16:40 已在板端跑完全部硬判据并回报，四层证据齐 ⇒ 阶段 2 网关链路抬 ✅**（记录表见规程第五节）。
> 下面 ①②③ 是这套验收方法本身的三行法；🔴 抬的只是「网关链路」，**语音入口整体仍 🔶**（ESP32 整机 build 已通过、已烧录实机验证）。
> **实机结果摘要**：八进制 `$ASR,回初始位*61` → `$OK,HOME*27` → `/api/state` 由 C 姿态变初始位 → 日志 `sent:true` → **用户肉眼确认臂动**；`PICK`→`$ERR,3`、急停四帧全对、`解除急停`→`$OK,RESUMED*75`。

① **用了什么语法/技术**：`printf` 八进制字节转义 `\345\233\236…`（POSIX 通用、绕开 SecureCRT 中文编码）+ `socket.create_connection()` 一行兜底（不依赖 `nc`）+ 精确 CK 速查表（由 `build_frame()` / `xor_checksum()` 实算）+ **四层证据链**（应答帧 / 网关日志 `sent:true` / `/api/state` 脉宽 / 肉眼）；板端命令一律单行、PC 侧 PowerShell 分两行（PS5.1 无 `&&`）。

② **为什么这么设计**：三个前提都是**已实测或已踩过**的 —— 板端 `~/kunpeng_vision/` 是**平铺目录**，而 `voice_gateway.py` 要 `import ../llm/`（本机 `python -c` 复现过 `ModuleNotFoundError`，先 import 网关才正常）⇒ 必须镜像 `llm/` + `robot/` 两层；`intent_parser.py:92` 的 HOME 词表**只有中文**（`$ASR,home*63` 实测回 `$ERR,2*5B`）⇒ 帧里必须带中文或用 UTF-8 八进制字节；`/api/cmd` 的 `ok:true` 只＝串口写进去了 ⇒ 硬判据不能只看 `$OK,HOME`，必须回读 `/api/state`，看脉宽从 C 姿态 `1636,1470,2177,1106,1450,1567` 变成初始位 `1580,1900,2400,700,1450,1500`。

③ **不这么写会怎样**：把网关直接丢进板端根目录 ⇒ 起不来；用 `echo` 手打中文且终端非 UTF-8 ⇒ 好帧一律 `ERR,1`，会误判成「网关坏了」；只看应答帧就宣布通过 ⇒ 把「帧写进串口」当成「臂真的动了」，这正是本项目反复强调的**静默失效**；验收完把 `--apply` 进程留着 ⇒ 一个会动臂的端口一直开着。

**排障见 `docs/排障记录-语音网关.md`（8 条，含第 8 条板端两个非失败误判点）；单行可粘贴命令清单 + 已填写的验收记录表见 `docs/上板验收-语音网关阶段2.md`。**

### G. 11-14 代码审查加固：locale 无关输出 + 急停/恢复词表消歧（`voice_gateway.py`）

> 🔴 只改 `voice_gateway.py` + `test_voice_gateway.py`，**没碰** `web_control.py` / `uart_protocol.c` / `intent_parser.py` / `text_gateway.py` / 任何 `.ioc`；当时档位仍 🔶 未上板，**16:40 实机验收后阶段 2 已抬 ✅**（见 F 节）。

① **用了什么语法/技术**：`sys.stdout.reconfigure(encoding="utf-8", errors="backslashreplace")` 就地改输出编码（Python 3.7+；老版本 / 被 IDE、unittest 包装过的 stdout 没有该方法，用 `try/except (AttributeError, ValueError, OSError)` 跳过）+ `str.encode("ascii", "backslashreplace")` 把非 ASCII 转成 `\uXXXX` 转义 + 所有 `print` 收口到一个 `_emit(text, newline=True)` 输出函数 + `subprocess` 强制 `PYTHONIOENCODING=ascii` 起真进程做端到端回归；词表侧抽 `_match_resume()` / `_match_estop()` 两个纯函数，用「恢复词优先排除」消除短词表子串歧义。

② **为什么这么设计**：两处都是**本机 UTF-8 测不出、上板 `LANG=C` 才炸**的问题 ——
a) 启动横幅中文 `print` 抛 `UnicodeEncodeError` ⇒ **服务直接退出**；`--self-test` 先打 `SELF_TEST PASS` 再因中文 JSON 抛异常 ⇒ rc=1，验收规程 1.3 唯一的结构自检会误判失败；handler 里日志 `print` 写在 `_write(reply)` **之前** ⇒ 异常一抛**应答帧被吞掉**，客户端 `recv` 得空字节。
b) `EMERGENCY_WORDS` 含裸词「急停」，而 `RESUME_WORDS` 里的「解除急停」是它的超集字符串 ⇒ 喊「解除」反而再急停一次（`payload='STOP'`）；且 `arbitrate()` 与 `special_text()` 各自实现匹配，小写 `stop` / `resume` 行为不一致。
⇒ 输出做成 locale 无关、应答**先于**日志发出；匹配抽成共用纯函数、显式消歧。

③ **不这么写会怎样**：只在本机跑原有 32 项单测 / TCP 四步链路全绿就交付 ⇒ 板上第一行中文横幅就崩、验收从第一步卡住；只改编码不改顺序 ⇒ 日志异常仍会吞掉应答，客户端永远收不到 `$OK,HOME` 却查不出原因；只补一个词表特判 ⇒ 两层行为继续分叉，将来接真模型时「解除急停」又变成急停。**急停优先级语义不变** —— 它由 `estop_active` 冻结闸门保证，词表顺序只决定「这一帧归类为急停还是恢复」。

**回归证据**：`PYTHONIOENCODING=ascii` 下 `--self-test` rc=0、`--make-frame` 出 `*61`、真 TCP server 能起且中文帧回 `$OK,HOME*27\r\n`；`test_voice_gateway.py` **38 项全绿**；`text_gateway.py` / `intent_parser.py` 自检 rc=0。详见 `docs/排障记录-语音网关.md` 第 6、7 条。

### H. 11-15 宿舍无硬件：四层硬判据自动判定（`check_esp32_link.py`）

> 🔴 背景：11-15 用户仍在宿舍、硬件都在教室。②③（ESP32 真发帧、ESP-SR）都被硬件卡死，
> 所以先消除**明天到教室最容易翻车的环节** —— 在几百行串口日志里靠人眼找 4 条证据。
> 本模块**只读日志、不改工程、不发串口、不动臂**，`esp32/` 与语音入口整体档位仍 🔶。

① **用了什么语法/技术**：纯标准库 `argparse`（命令行）/ `pathlib`（读文件）/ `re`（找 IP 与 send 字节数）/
`json`（逐行解析网关日志）+ 4 个纯函数 `check_layerN()` + `evaluate()` 把三份文本变成 4 个 `(bool, 说明)`；
`--self-test` 内置 5 组假日志自证脚本本身；`_emit()` 用 `backslashreplace` 做 locale 无关输出（复用 G 节踩过的 `LANG=C` 坑）。

② **为什么这么设计**：判据写成纯函数、读文件单独隔离 ⇒ 零硬件也能验证脚本自己没写错；
四个否决项全部来自已踩过的坑 —— `$ERR,2*5B`（发成 `mic_ok` / `home`，词表只认中文）、
`$ASR,mic_ok*50`（烧了旧副本 / 没重新 build）、`apply=true`（网关误开 `--apply`，会真动臂）、
第 4 层从**外部主机**探活（证明不是 `127.0.0.1` 自连、不是 `--host 0.0.0.0` 漏了）。

③ **不这么写会怎样**：人眼翻日志最容易把「帧写进去了」当成「链路通了」、把 `$ERR,2` 看成 `$OK`、
或没发现网关被开成 `--apply`；这三类误判会让「编译通过 / 看着像通」被错当成实机验收去抬档，
正是本项目反复强调的**静默失效**。所以脚本输出末尾固定打印「判定通过 ≠ 抬档」。

**用法**：`python3 check_esp32_link.py --self-test`（零硬件）；明天把串口 monitor / `/root/gw.log` / B4 `nc`
输出存成三个文本再跑 `--uart-log ... --gw-log ... --probe-out ...`，见 `docs/明日回教室-ESP32照跑清单.md` 第五节。
🔴 `--probe-text '$OK,ESTOP*75'` 里的 `$` 必须用**单引号**包住，否则会被 shell 当成变量展开。

**本机验证**：`SELF_TEST PASS cases=5 failed=0`、CLI 全绿 `RESULT: PASS=4 FAIL=0`、
旧固件用例正确 rc=1、`PYTHONIOENCODING=ascii` 下 rc=0，合计 `VERIFY PASS=8 FAIL=0`。

## 诚实边界

- 🔶 能写：「实现眼在手定位链（内参反投影 + 关节角正运动学 + 手眼变换 + 射线平面求交），并用占位数据端到端验证 45 项」。
- ✅ 实标完成后才能写：「完成一次眼在手手眼标定（相机内参 + 纯 numpy 解 AX=XB），产出 `handeye_X.json` 并接入点选链路」。
- ❌ **当前实机采过两轮，`solve` 均判无效**（2026-11-10）：第一轮 `t=42.9cm` 无解；第二轮 `t` 同样离谱 + 激励不足。
  两次都只有约 7~9 个独立朝向，且 J2/J3 的变化对相机姿态零贡献。**不能写「已完成手眼标定」。**
  第三轮复盘的补充结论：第二轮的 `resid=0.96 m` 量级与「`--square` 填 0.025、实际约 0.008」相符
  （尺度错 3.1 倍会等比放大相机侧平移，而 FK 侧平移是真的），所以那轮的「无解」**不一定**是几何不可观；
  现在 `solve` 自带尺度自校正，可以把这个变量摘掉后再判一次。
- ❌ **不能用「棋盘离相机多远」去反推 `--square`**：相机装在 J4 支架上、镜头离 J4 轴心约 5~6 cm，
  所以 `|t|` 天生比「按腕点位置算出来的距离」小一截。这个坑踩过一次，判据只能靠尺子。
- ❌ 仍不能写：「眼在手闭环 / 视觉伺服 / 边看边抓 / 实时跟踪 / 自动抓取」—— 当前只有**单帧单点**定位，
  无跟踪、无闭环伺服。抓取序列（`action_primitives` / `pick_pipeline`）**有代码、离线测试全过、未上板跑过**，
  所以最多写「实现了抓取动作序列与安全闸门，离线回归 16 项全过」，不能说「已实现自动抓取」。
- 本机验证只证明**数学与接线正确**；合成数据 `~1e-15 cm` 是机器精度，实机含噪声误差会大得多。
- `rpy_cam2gripper_deg` 在绕 Y=180° 附近有多种等价表示（`[0,180,0]` 与 `[180,0,180]` 描述同一旋转），
  这是 ZYX 欧拉角在奇异点附近的固有不唯一性，**看 `X_matrix` 才是唯一真相**。
- ✅ **`voice_gateway.py`（B 项 / 阶段 2）网关链路 11-14 16:40 已上板 `--apply` 实机验证**（38 项本机单测 + 四层硬判据全绿，记录见 `docs/上板验收-语音网关阶段2.md`）；🔴 **语音入口整体仍 🔶**：ESP32 已接入并发链路自检帧、`$ASR` 语义层未对齐、`esp32/` 已烧录实机（TCP 9000/采音）。
  `source` 恒 `mock`，没接云端大模型 / ASR / ESP32；`$ASR,<文本>*<CK>` 是 `llm/text_gateway.py` 的**建议草案、未与 ESP32 实机对齐**。
  ✅ 能写「搭建鲲鹏侧收帧网关与本地仲裁器，38 项单元测试通过、默认 dry-run 不下发；11-14 上板 `--apply` 用假帧驱动机械臂回初始位、`PICK` 被拒、急停可冻结」；
  ❌ 不能写「语音控制已实现 / 语音→动作闭环 / 已接 ESP32 语音链路 / 自然语言控制机械臂」。
  路线图阶段 2 硬判据「喂假帧 ⇒ 机械臂真的动」**已于 11-14 16:40 上板 `--apply` 实跑通过**。
- 🔴 急停只在**网关层**冻结后续下发并转发 `STOP` 给固件；仍是开环、无位置反馈，**不是**功能安全认证意义上的急停。

### 九、电压引导的分步合爪（`grip`）

**用了什么**：`read_current_ma()`（多读取中位数 + 三种非数值回包分别报错）、
`grip_until_contact()`（粗 40 µs / 细 15 µs 两段步进，两级电流判据）、
CLI 子命令 `grip`（`--from/--max/--hold/--rise`，照样受 `--dry-run` / `--yes` 管）。

**为什么这么设计**：J6 是位置控制的舵机，它**不知道自己夹没夹到东西**——原来靠
写死 `1550`（185 µs/cm 从两点外推），换个体积不同的物体就得重新试。改成「边合边看
电流」之后，判据来自被夹物体本身的阻力，不用再猜脉宽。但实测发现「刚碰上」的信号
只有 +49 mA、跟 ±25 mA 毛刺同量级，而「夹得住」是 +90 mA 以上，所以做成
**弱判据只记候选点、强判据才真停爪**；两者都没出现则判「夹空」并**拒绝抬臂**。

**不这么写会出什么问题**：① 单阈值判停会停在 1510（开口约 4.2 cm，夹不住 3.8 cm 瓶）；
② 夹空时若一路合到底，会顶到机械限位、舵机堵转发热（`GRIP_MAX_JAW=1700` 就是为此设的闸）；
③ 把 `CUR,NA`（ADC 没配起来）当成 0 mA，会把「没接上」误判成「已经夹住」。

```bash
# 前提：臂已在抓取位、物体在爪间、J6 张到 950
python3 action_primitives.py --dry-run grip     # 只读基线电流，不动臂
python3 action_primitives.py --yes grip         # 真合爪，打印停住的 J6
# 注意：--dry-run / --yes 这类全局参数必须写在子命令**前面**
```

⚠️ **边界**：这是 `action_primitives` 层的「接触即停」，**不是力控**；`pick_pipeline`
自动链路**尚未接入**，仍 🔶。实测标定只针对 3.8 cm 喷雾瓶一组；`--max 1700` 只能夹
≥2.7 cm 的物体（更小的要调大，但要留意空夹堵转）。

`--lift N`：夹住后顺手把 J2 抬 N 位置单位（`+60` 为经验值），省掉
「读数 → 手敲 ARM,POS」这一步；不加 `--lift` 时，工具会直接打印**数字已填好**的
抬起 / 放回命令。放回故意拆成「先降回抓取位（仍夹紧）→ 再张爪」两步——一次到位会在
半空松爪，物体要掉一段。🔴 之所以让程序生成命令：19:1x 在聊天里给过带 `<占位符>`
的命令，用户照贴 ⇒ 中文字节进 payload ⇒ Flask 回 `'ascii' codec can't encode`。

**接进自动链路**：`pick_pipeline.py --dynamic-grip` 会让 `run_plan` 在「合爪」那一步
改调 `grip_until_contact()`，并把后续「保持夹紧」的步骤同步成实际夹持值（张爪步骤不动）。
判为夹空时**不抬臂**、返回码 **7**。🔴 但它只解决「夹多紧」，**不解决「伸到哪」**——
`pick_pipeline` 的臂姿来自 IK（tilt≈41，总线舵机闭环下 IK 应直接可用，装车后实测验证）。

---

## 排障记录

- 语音入口网关（B 项 / 阶段 2）另见 **`docs/排障记录-语音网关.md`（8 条）**：字面量 `\r\n`、非 ASR 急停帧、`--apply` 下发失败、多线程急停锁、「下发 `$JOINT`」的设计取舍订正，以及 11-14 代码审查新增的**第 6 条（板端 `LANG=C` 服务起不来 / `--self-test` rc=1 / 日志吞应答）**、**第 7 条（「解除急停」被反向误判成急停）**、**第 8 条（板端实机验收两个「看着像故障」的非失败误判点，全程零故障）**。
- 底盘统一帧（A 项）的历史排障记录原在 `car/排障记录.md`（4 条），该目录已于 2026-11-24 删除；现行底盘见 `docs/底盘-TB6612与JGB37-520-接线、固件与调试.md`。

见 `docs/排障记录-手眼标定.md`（现 12 条，本日新增第 6 条「solve 报激励不足实为无解」、
第 7 条「USB 掉线后读线程静默死亡」、第 8 条「残差用总范数 + 棋盘格尺度 → 好数据被判无解」、
第 9 条「激励判据用错指标 → 好解被判不足、退化解被判 OK」、
第 10 条「姿态公式镜像 → 位置说往下、姿态说往上，残差卡 16.7mm」、
第 11 条「改完 FK 再 solve 输出一字未变 → 样本里存的是算好的矩阵，必须显式重算」、
第 12 条「残差 1.6 mm 怎么证明模型真对了 → 用『棋盘没动 ⇒ P 必须重合』这个物理不变量」）。
