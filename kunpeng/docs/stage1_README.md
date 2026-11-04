# 阶段1：YOLOv8 通用物体识别（落地文档）

> 日期：2026-11-03。阶段0 已钉死「双 GF100 是 2D 视觉、无真深度、两摄像头≠立体」的边界；
> 本阶段把 `a_vision_demo.py` 的「颜色+形状」识别升级为 **YOLOv8 学习式目标检测**，
> 让小车能认出 **水杯 / 饮料瓶 / 水果** 等大部分物体，而不只认红绿蓝方块。
> 这是「抓大部分物体」这条链路的第一环（**识别层**）。

## 交付物
- `stage1_yolov8.py` —— 推理脚本（ultralytics / OpenCV DNN 双后端，自动降级）
- 本文件

---

## 〇、最快：一键部署脚本（推荐先跑这个）

项目已附 `deploy_stage1.sh`，自动识别 openEuler(`dnf`) / Ubuntu(`apt`)、自动处理 PEP668 限制，
装好最小依赖 + 可选 A 后端 + 下载 `yolov8n.pt`。

**1) 把两个文件拷到开发板**（在 Windows / 能联网的机器上执行，替换 `<板子IP>`）：
```bash
scp stage1_yolov8.py deploy_stage1.sh orangepi@<板子IP>:/home/orangepi/kunpeng_vision/
```
> 不知道 IP？在开发板终端跑 `ip a | grep inet` 看；或路由器后台查 `orangepi` 设备。
> 若开发板用户名不是 `orangepi`，改成实际用户名（openEuler 镜像常见是 `root` 或 `openeuler`）。

**2) 在开发板终端跑部署脚本**：
```bash
cd ~/kunpeng_vision
chmod +x deploy_stage1.sh
./deploy_stage1.sh
```

脚本会装：
- **必须**：`opencv-python-headless` + `numpy`（脚本头部硬依赖，不装起不来）
- **推荐(A 后端)**：`torch`(CPU wheel) + `ultralytics`（直接读 `.pt`，阶段2 微调也要）
- **模型**：`yolov8n.pt`（约 6MB，COCO 预训练）

若不想跑脚本，手动步骤见下方「一」。

---

## 一、在 KunPeng Pro（openEuler aarch64）上安装

### 方案 A：ultralytics（推荐，阶段2 微调也用它）
```bash
# 1) torch 在 aarch64 有官方 wheel（2.x 起支持 CPU 版），二选一装：
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cpu   # 官方 CPU wheel
#   或   sudo dnf install python3-torch python3-torchvision   # 系统源（epol，版本可能旧）
# 2) 安装 YOLO 库 + opencv
pip3 install ultralytics opencv-python-headless numpy
# 3) 取模型权重（只需一次）
yolo download model=yolov8n.pt
```
> 验证：`python3 -c "import torch, ultralytics; print(torch.__version__)"` 不报错即可。

### 方案 B：OpenCV DNN（轻量降级，仅推理，**不装 torch**）
```bash
sudo dnf install -y opencv python3-numpy python3-opencv
# 把 .pt 导出成 .onnx（在任意装了 ultralytics 的机器做一次即可）
yolo export model=yolov8n.pt format=onnx
```
**说明**：B 方案依赖极小，适合只演示推理；但 **阶段2 微调必须用 A 方案**（训练需要 torch）。

---

## 二、运行（两种后端参数完全一致）
```bash
# 摄像头实时（GF100 默认 /dev/video0）
python3 stage1_yolov8.py --source 0 --model yolov8n.pt     # A 后端
python3 stage1_yolov8.py --source 0 --model yolov8n.onnx   # B 后端
python3 stage1_yolov8.py --source 0 --headless             # ssh 无窗口，终端打印结果

# ⚠️ KunPeng 纯 CPU 实测：640 输入仅 ~2.5 FPS（太卡）。板子上务必降分辨率提速：
python3 stage1_yolov8.py --source 0 --model yolov8n.pt --headless --imgsz 320
# headless 摄像头模式现为常驻运行，按 Ctrl+C 退出（不再是跑 300 帧自动停）

# 图片 / 视频文件
python3 stage1_yolov8.py --source test.jpg
python3 stage1_yolov8.py --source clip.mp4
```

---

## 三、验证「跑通」的判据（照着勾）
1. 启动打印 `[模型]` 显示用了哪个后端，且 `[OK] 输入源已打开`
2. 画面出现 **绿框=可抓取类**（cup/bottle/bowl/banana/apple/orange…），**蓝框=其他类**，框上带 `类别 置信度`
3. 终端（headless）打印：`[可抓取] cup 置信度=0.87 中心=(320,240) 方位=居中`
4. 收尾打印：`[统计] 共 N 帧 ... 平均 X.X FPS`
5. **认杯子实测**：放一个水杯/饮料瓶进画面 → 出现 `cup`/`bottle` 绿框 = 阶段1 成功

---

## 四、预期性能（鲲鹏920 4 核 ARM，纯 CPU 推理，2026-11-04 板上实测）
- `yolov8n` **640 输入**：实测 **2.5 FPS**（约 400ms/帧）—— 纯 CPU 跑 640² 太大，机器人实时跟踪会卡。
- `yolov8n` **320 输入**：计算量降为 1/4，**板上实测约 7 FPS**（每帧约 140ms），对慢速移动机械臂已够用；**板子上默认用 320**。
- 再嫌慢/要更高帧率：① 导出 INT8 量化模型（速度再提 2~3 倍，精度略降）；② 降到 `--imgsz 256`；③ 换 OpenCV DNN 后端试试。
- **不要指望 NPU**：本板 NPU 为已知设计缺陷（载板电流采样坏），不可用，纯 CPU 推理。
- 多线程：torch CPU 默认会用满 4 核；OpenCV DNN 后端可 `cv2.setNumThreads(4)`。

---

## 五、脚本里的技术点（面试能讲）
- **双后端降级**：先 `try ultralytics` 失败再 `OpenCV DNN`，保证最小环境也能出框（容错设计）
- **YOLOv8 ONNX 后处理**：输出 `(1,84,8400)` → 转置 `(8400,84)` → 每行取最大分数类 → **NMS**（非极大值抑制）去重
- **坐标映射**：onnx 输出在 640 网格空间，需 `(cx/640*w, cy/640*h)` 映射回原图像素
- **优先级高亮**：用集合 `PRIORITY_CLASSES` 区分「可抓取日用物」（绿）与「其余类」（蓝），直接回答「机器人该抓哪个」
- 复用阶段0/P0 的「方向提示」逻辑（偏离画面中心 ±20px → 左/右/上/下），输出给机器人决策

---

## 六、与现有代码的关系
- `a_vision_demo.py`：颜色+形状追踪，**不认物体类别**，只给质心。保留作 P0 基础演示。
- `stage1_yolov8.py`：**类别级识别**，认出「是什么」。二者并存，阶段2 把识别结果喂给抓取规划。
- 下一步（阶段2）：用自有样本微调认 `tissue`/`egg`；接「逆解 + 伪3D」做抓取闭环。

---

## 七、诚实边界（面试必说）
- `yolov8n` 是 **COCO 80 类**，开箱认 `cup / bottle / apple / orange / banana / potted plant`；
  **没有 `tissue`（纸巾）、`egg`（鸡蛋）** —— 这两类需阶段2 自标 30~50 张图微调。
- 识别是 **2D 框**，不给出精确距离（距离靠阶段0 的伪3D 或深度相机）。
- 透明玻璃杯 RGB 本身难认，YOLO 易漏检 —— 演示用不透明容器。

---

## 八、面试话术
> "识别层我用 YOLOv8n 做通用物体检测，在 KunPeng（鲲鹏920 4核 ARM 纯 CPU）上推理。
> 我实测 640 输入只有 2.5 FPS，所以降分辨率到 320 提到约 10 FPS——我清楚这是算力瓶颈、不是模型问题。
> 能认 COCO 里的杯瓶果；对 COCO 没有的纸巾鸡蛋，我会用自有样本微调（阶段2）。
> 检测框给机器人提供'是什么 + 在哪'，距离估计交给伪3D 或深度相机——识别、测距、抓取三层分离，我清楚每层的边界。"

---

## 九、常驻服务（对接决策 / 底盘 / 机械臂层）

阶段1 不只是个 demo。`detector_service.py` 把识别层封装成**常驻服务**：后台线程持续跑摄像头+推理，
通过 HTTP 把最新结果以 JSON 暴露出来，决策层和底盘/机械臂代码随时来问"现在看到什么"。

**启动（在板上）：**
```bash
python3 detector_service.py --model yolov8n.pt --imgsz 320
# 默认监听 0.0.0.0:8080，允许局域网访问
```

**怎么消费（三种方式）：**
1. **浏览器调试**：开 `http://<板子IP>:8080/` 看实时识别表格；`http://<板子IP>:8080/detections` 看原始 JSON
2. **命令行**：`curl http://<板子IP>:8080/detections`
3. **决策层代码（同机 Python）**：直接 `from detector_service import Detector` 实例化后 `get_latest()`，
   不必走网络（适合后续和决策/底盘逻辑同进程集成）

**返回的 JSON 结构：**
```json
{
  "frame_id": 327,
  "timestamp": 1699000000.12,
  "detections": [
    {"class":"bottle","conf":0.91,"bbox":[x1,y1,x2,y2],"center":[435,236],"direction":"右","priority":true}
  ],
  "target": {"class":"bottle","conf":0.91,"center":[435,236],"direction":"右","priority":true}
}
```
- `detections`：本帧所有识别到的物体（含优先级标记）
- `target`：**优先可抓取类里置信度最高的一个**（没有则为 `null`）——决策层直接用这个决定"抓哪个、往哪走"

**升级路径（面试能讲）：** 现在用 HTTP/JSON 做接口，零依赖、易调试；后续若接 ROS2（openEuler 有 EPOL 源 `dnf install openeuler-ros`），
只需把 `get_latest()` 的数据用 `sensor_msgs/Image` + 自定义 `Detection` 话题发布即可，**识别逻辑一行不用改**——这就是"接口与实现分离"的好处。
