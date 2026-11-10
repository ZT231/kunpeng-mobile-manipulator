# kunpeng/robot/ · 眼在手手眼标定 + 定位模块

> 状态（2026-11-10 13:58）：🔶 **标定链三轮改造完成 + 定位链新增并端到端打通（45 项 mock 全绿）**；
> 三轮加了**结果自证**（solve 输出三分类 + valid 标记，无效解拒写正式文件名）与**串口断线显式报错**。
> 但用的是**占位 X/K**，**实机采过两轮、solve 均判为无效**（第一轮方程无解，第二轮无解 + 激励不足）。
> 完成线第 5 项「至少一次实标」仍未达成；简历正文暂存不投递。

## 文件分工

| 文件 | 作用 |
|---|---|
| `handeye_calib.py` | 标定链：4 个子命令 `selftest` / `intrinsics` / `collect` / `solve`，只负责**产出** X；`solve` 带三分类自证（`valid` / `status` / `diagnostics`） |
| `handeye_geometry.py` | 🆕 定位链：**消费** X，像素 + K + 关节角 → 工作台 3D 点（含 9 项自检） |
| `gen_placeholder_calib.py` | 🆕 生成占位 `handeye_X.json` + `camera_intrinsics.json`（带 `placeholder:true`） |
| `test_eye_in_hand_mock.py` | 🆕 端到端 mock 验证：占位数据 + 假串口，45 项检查，零硬件 |
| `arm_ik.py` | 逆解 + 正解 `forward()`；本模块用 `to_angles()` 把脉宽还原成物理角 |
| `arm_host.py` | 旧版 `collect` 直连串口读 STATE 用；**本版 collect 不再 import 它**（板上留着无妨） |
| `arm_params.json` | 脉宽↔角度物理标定表，**保留不动**；`handeye_geometry` 也读它，保证只有一份标定表 |

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
现在用「K 反投影出射线 → 当前 6 路**真实脉宽**做 FK → 乘手眼 X → 与 z=Z 求交」，才对得起「相机装在臂末端」的硬件事实。
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
- 单靠「奇异值比值」分不清「激励不足」和「无解」：实测单轴退化的 `ratio=0.73` 但残差只有 3.4 mm，
  真无解的残差是 21~96 cm。必须加两级残差门限（2 cm / 1 mm）才能分开，且判定顺序不能交换。
- `_rx_loop` 原本 `except Exception: break` —— USB 掉线后读线程静默退出，而 pyserial 的 `is_open`
  仍为 True、`write()` 也不报错，上层只能看到「命令发出去了、响应是空串」，会误判成 STM32 没应答。
③ **不这么写会怎样**：无效 X 被当有效加载 → 抓取偏十几厘米且零提示；断线被当成「固件不响应」→
去查杜邦线、重烧固件，真正的根因（USB 供电/hub 复位）永远找不到。

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
- **旋转激励**：姿态之间必须有明显、转轴多样的相对旋转；只平移不转则系数矩阵秩亏损，X 解不唯一（代码按奇异值比值打印警告）。
- **STATE 是「目标脉宽」不是「实际位置」**：板子只记得命令舵机转到哪，没编码器反馈；姿态必须靠下发 JOINT 改变。
  规范定义：开环位置控制，状态量是 PCA9685 输出比较寄存器对应的指令脉宽，非关节角反馈。
- **单一资源独占 + HTTP 代理读取**：串口帧是字节流，两个进程同时 `read()` 会各自抢走一半字节 ⇒
  只让一个进程（`web_control.py`）打开串口，其余进程通过 HTTP 要结果。
- **占位数据（placeholder）**：给程序先装一只「假眼睛」，好让后续流程先跑通，真眼睛装好换掉文件即可。
  规范定义：以名义几何参数构造的标定元数据替身，附 `placeholder:true` 标记，使下游可在无实测数据时完成接口联调，**不得用于精度评估**。

## 用法

### A. 无硬件打通链路（本轮已完成，PC 上就能跑）

【PC · PowerShell】（`Set-Location "D:\简历\STM32项目\4.ESP32-S3项目\RoboticArm_Kunpeng\kunpeng\robot"` 后执行）
```
D:\Python\Python3.14.7\python.exe -B gen_placeholder_calib.py
D:\Python\Python3.14.7\python.exe -B handeye_geometry.py selftest
D:\Python\Python3.14.7\python.exe -B handeye_geometry.py
D:\Python\Python3.14.7\python.exe -B test_eye_in_hand_mock.py
```

期望：`selftest` 9 项全绿；诊断打印 `数据状态：X(手眼矩阵)为占位值…`；mock **45 项全过**。

### B. 真机实标（🟡 尚未执行）

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
python3 handeye_calib.py collect --camera 0 --state-url http://127.0.0.1:5000/api/state --intrinsics camera_intrinsics.json --cols 9 --rows 6 --square 0.02 --count 12 --interval 18.0 --timeout 900 --save-dir ./collect_imgs --out samples.json --headless
```
```
python3 handeye_calib.py solve --samples samples.json --out handeye_X.json
```

**`solve` 的返回值约定**（第三轮新增）：`0` = 解有效、已写 `handeye_X.json`；
`1` = 跑不起来（样本不足）；`2` = 跑通了但判为无效，只写 `handeye_X.rejected.json`。
看终端最后那段三分类结论即可，不用自己算奇异值。

- 棋盘格：**9×6 内角点**；无打印机时用手机/显示器全屏显示 `chessboard_9x6.png`，平放抓取平面，全程不动。
- `--square` 单位是**米**：量「连续 10 格总宽 `L mm`」→ `--square = L/10000`（L=200mm → 0.02）。
- `intrinsics` 与 `collect` 的 `--square` **必须一致**；`collect` 未显式给时会自动继承内参文件里记录的 `square_m`。
- `collect` 每 18 秒自动采一组：用网页滑杆把臂摆到不同姿态，**等斜坡走完、STATE 稳定**再让它拍。
  🔴 **相机装在 J4 轴上 ⇒ 只有 J1（偏航）和 J4（俯仰）能改变相机朝向**；变 J2/J3 只改平移，对旋转激励零贡献。
- `--timeout` 兜底；`--save-dir` 存帧供事后复核（detector 停后网页画面会黑，靠它 + 终端提示调整）。

### C. 在网页上用眼在手定位

【板 · openEuler 终端】（重启 5000 才生效；`--eye-in-hand` 不加则完全走旧通路）
```
pkill -f web_control.py; sleep 1; cd ~ && nohup python3 web_control.py --host 0.0.0.0 --port 5000 --ik-dir ~/kunpeng_vision --eye-in-hand --video-url http://10.160.120.1:8080/video --detections-url http://127.0.0.1:8080/detections > ~/web.log 2>&1 &
```

【PC · 浏览器】打开 `http://10.160.120.1:5000`，先点「连接」，勾选取点，点画面 → 调
`POST /api/ik/eye/solve`（页面尚未加按钮，暂用下面命令验证）：
```
curl -s -X POST http://10.160.120.1:5000/api/ik/eye/solve -H "Content-Type: application/json" -d '{"u":320,"v":240,"apply":false}'
```

期望返回里出现 `"mode": "eye_in_hand"`、`"x"/"y"/"z"`、`"depth"`、`"placeholder": true`。
**占位数据下 `placeholder` 必须为 `true`** —— 看到 `false` 才说明用上了实测标定。

## 诚实边界

- 🔶 能写：「实现眼在手定位链（内参反投影 + 关节角正运动学 + 手眼变换 + 射线平面求交），并用占位数据端到端验证 45 项」。
- ✅ 实标完成后才能写：「完成一次眼在手手眼标定（相机内参 + 纯 numpy 解 AX=XB），产出 `handeye_X.json` 并接入点选链路」。
- ❌ **当前实机采过两轮，`solve` 均判无效**（2026-11-10）：第一轮 `t=42.9cm` 无解；第二轮 `t` 同样离谱 + 激励不足。
  两次都只有约 7~9 个独立朝向，且 J2/J3 的变化对相机姿态零贡献。**不能写「已完成手眼标定」。**
- ❌ 仍不能写：「眼在手闭环 / 视觉伺服 / 边看边抓 / 自动抓取 / 实时跟踪」—— 当前只有**单帧单点**定位，无跟踪、无闭环伺服、无抓取动作序列。
- 本机验证只证明**数学与接线正确**；合成数据 `~1e-15 cm` 是机器精度，实机含噪声误差会大得多。
- `rpy_cam2gripper_deg` 在绕 Y=180° 附近有多种等价表示（`[0,180,0]` 与 `[180,0,180]` 描述同一旋转），
  这是 ZYX 欧拉角在奇异点附近的固有不唯一性，**看 `X_matrix` 才是唯一真相**。

## 排障记录

见 `docs/排障记录-手眼标定.md`（现 7 条，本日新增第 6 条「solve 报激励不足实为无解」和第 7 条「USB 掉线后读线程静默死亡」）。
