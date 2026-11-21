# DaBai DCW 深度相机链路（相机侧）

今天只做**相机侧**，和机械臂解耦。机械臂回来后，明天的手眼标定只需把本包
`localize.pixel_to_camera` 出的相机系 `(x,y,z) cm` 再乘 `T_base_cam`，本包不改。

> **代码已换成 Orbbec SDK v1 / pyorbbecsdk 1.3.4 实现**（原 openni2 版作废：
> openni2 在 aarch64 上没有可用二进制，OpenNI 私有协议只能走 pyorbbecsdk v1）。

## 板上环境（已装好，可复现）

| 组件 | 位置 |
|---|---|
| OrbbecSDK v1.10.37 | `/opt/orbbec/OrbbecSDK_v1.10.37/`（`SDK/{config,include,lib}`） |
| pyorbbecsdk v1.3.4 源码 | `/opt/pyorbbecsdk/` |
| 编译产物（**生产用这个**） | `/opt/pyorbbecsdk/install/lib/pyorbbecsdk.cpython-39-aarch64-linux-gnu.so` |
| udev 规则 | `/etc/udev/rules.d/99-obsensor-libusb.rules` |

**每次运行前必须先设 PYTHONPATH**，否则 `import pyorbbecsdk` 会失败：

```bash
export PYTHONPATH=/opt/pyorbbecsdk/install/lib:$PYTHONPATH
```

重新编译（改源码后）：

```bash
cd /opt/pyorbbecsdk && mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -Dpybind11_DIR=$(pybind11-config --cmakedir) ..
make -j3
```

## 上板自检顺序

> 板上工作目录是 `/root/kunpeng_vision`，`orbbec` 包直接放这里，所以命令用
> `python3 -m orbbec.cli`（PC 工程树里才是 `kunpeng.vision.orbbec.cli`）。

```bash
cd /root/kunpeng_vision
export PYTHONPATH=/opt/pyorbbecsdk/install/lib:$PYTHONPATH

# 1) 探针：设备 / 流参数 / FOV / 内参
python3 -m orbbec.cli probe

# 2) 抓一张看效果（存 color.png / depth_mm.npy / depth_viz.png）
python3 -m orbbec.cli snap --out ./dcw_snap

# 3) 连续取流，看中心深度是否稳定
python3 -m orbbec.cli stream --seconds 10

# 4) 放一个红色物体，检测 + 相机系 3D 测距（不需要机械臂）
python3 -m orbbec.cli demo --color red --seconds 30
```

### 实测基线（2026-11-19，室温普通室内）

```
[device] Gemini E  pid=1628(0x065c) vid=11205  sn=AY4L84200W4  USB2.0
[depth]  640x360@30  fmt=Y11   (对齐后变 Y16)
[color]  640x360@30  fmt=MJPG
[intrinsics] fx=358.5 fy=358.5 cx=319.7 cy=179.0 640x360   ← 设备真实上报
snap  : 全图有效深度 73%，中心中值 2068 mm
stream: 27.7 FPS，中心深度 2068~2112 mm 稳定
```

## 内参（和手眼分开）

**好消息**：`pipeline.start()` 之后调 `pipeline.get_camera_param()` 能拿到
**设备真实内参**（`rgb_intrinsic`：fx=fy=358.5, cx=319.7, cy=179.0, 640x360）。
注意：**管道没 start 时调它返回全 0**，别提前调。

所以棋盘标定不是必须的，只在需要**畸变系数**或怀疑内参时才跑：

```bash
python3 -m orbbec.cli calib --cols 9 --rows 6 --square 0.02 --count 15 \
    --out camera_intrinsics.json
```

有 JSON 时 `DepthCamera(intrinsics_path=...)` 优先用 JSON，否则用设备内参，
最后才 FOV 反推兜底（`approx=True`）。

## 模块分工

| 文件 | 职责 |
|---|---|
| `source.py` | pyorbbecsdk 取流：Pipeline+Config+AlignFilter，深度对齐到彩色、彩色 MJPG 解码、内参(JSON/设备/FOV) |
| `depthproc.py` | 深度预处理：去飞行像素、区域中值采样、有效占比(识别黑物体) |
| `localize.py` | 像素 + 深度 + 内参 → 相机系 (x,y,z) cm |
| `cli.py` | probe / snap / stream / demo / calib 五条命令 |

## 这台设备的坑（实测，别再踩）

1. **彩色流要先预热**：启动后前约 9 帧 `get_color_frame()` 返回 `None`，之后才正常。
   `DepthCamera.__init__` 里已做 `_warmup(60)` 空跑丢帧，`read()` 再做 3 次重试。
2. **不要调 `enable_frame_sync()`**：Gemini E 不支持，抛 `OBError`
   （`Current device does not support frame sync!`），且实测调用后彩色流更容易掉。
   `synced` 恒为 `False`。
3. **`OBFormat` 没有 `RGB888`**，叫 `RGB`；枚举比较建议先 `int()` 归一化。
   `AlignFilter` 的参数是 `OBStreamType.COLOR_STREAM`，不是 `COLOR`。
4. **深度格式**：未对齐是 `Y11`，对齐后是 `Y16`，两者都按 `uint16` 读，
   `get_depth_scale()` 实测 =1.0（单位 mm）。
5. **对齐后深度网格 = 彩色网格**（都是 640x360），`fx/fy/cx/cy` 用彩色口径。
6. SDK 日志里的 `Get usb core type failed!`、`usb_request_cancel ... Broken pipe`
   是停止时的正常冲刷，不影响出帧，可 grep 过滤。

## 设备侧参数（决定深度质量，2026-11-19 复测）

`DepthCamera` 打开时会统一管理下面几个**掉电保留的粘性属性**——上次会话改过、
忘了复位，下次会静默带着旧值跑，这是本项目最容易踩的坑。

| 参数 | 默认 | 为什么 |
|---|---|---|
| `laser` | **True** | 红外激光。关掉只剩被动双目：同一场景两轮 A/B 实测**有效像素 0.000** |
| `depth_min/max` | 0 / 10000（不钳位） | 抓取建议 `set_depth_range(200, 1200)`；每次 open 复位，防残留 |
| `hole_fill` | 钳位时强制 **False** | 洞填充会把超量程区域补成边界值（2.2m 的墙 → 1477mm 假有效障碍物） |
| `depth_auto_exp` | True | 上次锁过固定曝光会残留，换光照（宿舍/教室）就整片过曝或欠曝 |

实测（激光开、不钳位、640x360@30）：

```
覆盖: 全图 0.825~0.910 | 中央50% 0.95 | 中央20% 0.89~0.99 | 左侧1/4 0.83
近场 200~1200mm: 有效像素 38348，时间噪声 1.43mm，唯一码值 502
precision 档位: 0 最优（0=近场38348/1.43mm，2=11105/47mm，4=1129，6=0 全空）
```

**硬件 D2C 对齐不可信**：`config.set_align_mode(HW_MODE)` 能开、也能出帧，
但对齐残差的最优点落在 ±10px 搜索边界（dx=10），而软件 `AlignFilter` 是 (0,-2)。
保持软件对齐，别开 HW——否则像素→3D 会整体偏十几像素。

性能（3 核 aarch64，640x360@30）：

```
wait_for_frames 20.4ms | align.process 10.5ms | depth 转换 0.48ms | MJPG 解码 2.06ms
合计 33.5ms -> 29.9fps（30fps 流就是上限，已经跑满）
read(with_color=False) 省 2ms；read(drop_stale=1) 防延迟累积
```

调参入口：

```bash
python -m orbbec.cli probe                        # 打印 props 全量
python -m orbbec.cli tune --range 200,1200        # 改量程并用 10 帧验证覆盖率
python -m orbbec.cli tune --laser off             # 对比激光关掉的效果
python -m orbbec.cli stream --seconds 10 --no-color --range 200,1200
```

## 实测能力边界（2026-11-19 全功能测试）

完整报告见 `docs/DaBai DCW 全功能测试报告.md`。要点：

| 约束 | 实测数据 | 对策 |
|---|---|---|
| 深度有效区 | 复测激光开启后左侧1/4 有效率 0.83；早前测到 0% 那次的差异无法复现（可能当时激光状态/场景不同） | 目标仍放画面中央偏右最稳 |
| 深度是**量化**输出 | 台阶≈1.1e-5·z²：0.5m≈2.8mm，1m≈11mm，2m≈44mm | 抓取控制在 0.3~0.6m |
| 640x480 无 D2C 对齐 | 出流正常（关对齐 12/12 帧、有效率 80.6%）；开对齐 0 帧，该分辨率也不上报内参 | 链路固定 640x360@30（对齐+真内参）；若 `warmup_ok=False` 不是相机故障，换回 640x360 |
| 只能对齐到彩色 | 对齐到深度：`Depth processor claimed failed!` | 保持 `COLOR_STREAM` |
| 时间戳单位 | **毫秒**，相邻帧 33ms | 不要再乘 1000 |

精度实测：0.25~0.8m 时间噪声 1.5mm、40x40 区域中值重复性 3.4mm；
25s 743 帧 0 失败、29.7fps；IR 640x480 可用（10/10）。
内参 fx=fy=358.476 / cx=319.669 / cy=179.010，`depth_intrinsic` 与 `rgb_intrinsic` 相同（均为对齐后彩色口径）。

## 日志与目录卫生（板上不留垃圾）

SDK 默认在**当前工作目录**生成 `Log/OrbbecSDK.log.txt`（DEBUG 级，一次会话 1~2MB）。
已在 `/root/kunpeng_vision/OrbbecSDKConfig_v1.0.xml` 放了一份调过的配置：

- `LogLevel=2`（WARN）
- `FileLogLevel=5`（**OFF**，不写文件）
- `ConsoleLogLevel=3`（ERROR）

SDK 在 **CWD** 查找 `OrbbecSDKConfig_v1.0.xml`，所以：

- 从 `/root/kunpeng_vision` 启动 → **不生成 `Log/`**
- 从别的目录启动（如 `/root`）→ 仍会生成 `Log/`

PC 侧备份：`kunpeng/vision/orbbec/config/OrbbecSDKConfig_v1.0.xml`。

### 每次跑完的固定清理动作

```bash
rm -rf /root/kunpeng_vision/Log \
       /root/kunpeng_vision/__pycache__ \
       /root/kunpeng_vision/orbbec/__pycache__
```

测试脚本与抓帧产物（`tests/`、`dcw_snap/`、`dcw_test/`）都是**一次性**，验完即删；
脚本本体常驻 PC `kunpeng/vision/orbbec/tests/`，需要时再 scp 上去。



1. `handeye_calib.py`：手眼外参 `X`（相机装在 J4 时，X=相机→J4 连杆，标定锁 J5/J6）。
2. 把 `T_base_cam = T_base_link4(q1..q4) · X` 拼上 `pixel_to_camera` 的结果。
3. `pick_pipeline`：物体高度从"写死 grab_height"改成实测 z。

## 已知风险

- 黑色/高反物体深度测不到：`sample_region` 返回 `None`，demo 会打印 `depth:none`，
  不是 bug。
- USB2.0 带宽有限：默认 640x360@30 双流已跑满 27.7 FPS，别贪更高分辨率。
- 编程类环境变量不持久：新开的 shell 记得重新 `export PYTHONPATH=...`，
  或写进 `~/.bashrc`。
