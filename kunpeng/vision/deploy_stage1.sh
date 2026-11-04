#!/bin/bash
# ===================================================================
# 阶段1 YOLOv8 一键部署脚本（OrangePi KunPeng Pro / KunPeng Pro）
# -------------------------------------------------------------------
# 用途：在开发板上自动装好「阶段1 识别」所需的全部依赖 + 模型权重。
# 适用：openEuler 22.03（dnf）或 Ubuntu 22.04（apt），脚本自动识别。
# 用法：把本脚本和 stage1_yolov8.py 一起拷到板子，然后
#       chmod +x deploy_stage1.sh && ./deploy_stage1.sh
# ===================================================================
set -e

echo "==== 阶段1 YOLOv8 部署 (KunPeng Pro) ===="

# 提权前缀：已是 root 就不加 sudo
SUDO=""
if [ "$(id -u)" -ne 0 ]; then SUDO="sudo"; fi

# 识别包管理器
if command -v dnf >/dev/null 2>&1; then
  PM="dnf"
elif command -v apt >/dev/null 2>&1; then
  PM="apt"
else
  echo "[错误] 未找到 dnf / apt，请手动装 python3-pip"; exit 1
fi
echo "[系统] 包管理器 = $PM"

# pip 安装函数：自动处理 openEuler/新版 Ubuntu 的 PEP668 限制
pip_install() {
  if ! pip3 install "$@" 2>/tmp/piperr; then
    if grep -qi "externally-managed" /tmp/piperr; then
      echo "[信息] 检测到 externally-managed-environment，改用 --break-system-packages"
      pip3 install --break-system-packages "$@"
    else
      echo "[失败] pip 安装出错："; cat /tmp/piperr; exit 1
    fi
  fi
}

# 1) 基础系统包：python3-pip + numpy
if [ "$PM" = "dnf" ]; then
  $SUDO dnf install -y python3-pip python3-numpy 2>/dev/null || true
else
  $SUDO apt update -y && $SUDO apt install -y python3-pip python3-numpy
fi
# 升级 pip（openEuler 的 pip 由 rpm 安装，无法自我升级，失败可忽略，系统自带 pip 一般够用）
pip3 install --upgrade pip 2>/dev/null || echo "[信息] pip 升级跳过（系统为 rpm 版，不影响后续安装）"
# OpenCV：headless 版不依赖 libGL，SSH / 无桌面环境最稳
pip_install opencv-python-headless numpy
echo "[OK] 最小依赖（opencv + numpy）已装 —— 脚本已能启动、跑后处理"

# 2) A 后端：ultralytics + torch（CPU 版）。阶段2 微调也用它，默认装
read -r -p "是否安装 ultralytics + torch（A 后端，阶段2 微调必需）? [Y/n]: " ANS
if [ "${ANS,,}" != "n" ]; then
  # aarch64 有官方 CPU wheel（PyTorch 2.x 起支持）
  pip_install torch torchvision --index-url https://download.pytorch.org/whl/cpu
  pip_install ultralytics
  echo "[OK] A 后端（ultralytics + torch）已装"
else
  echo "[信息] 仅装了 OpenCV。后续需把 yolov8n.pt 导出成 .onnx 才能用 B 后端推理："
  echo "       在装有 ultralytics 的机器上跑：yolo export model=yolov8n.pt format=onnx"
fi

# 3) 模型权重 yolov8n.pt（约 6MB，COCO 80 类预训练）
# 多源下载：官方 yolo 下载器 → GitHub 直链(HTTP/1.1) → 国内 HF 镜像，任一成功即可。
# 验证文件 ≥1MB 才算就绪，避免残缺文件被误判（GitHub 在国内常超时/空回包）。
dl_yolov8n() {
  local ok=0 sz
  sz=$(stat -c%s yolov8n.pt 2>/dev/null || echo 0)
  if command -v yolo >/dev/null 2>&1; then
    echo "[下载] 方式1：yolo download ..."; yolo download model=yolov8n.pt 2>/dev/null && ok=1
  fi
  if [ "$ok" -ne 1 ] || [ "$sz" -lt 1000000 ]; then
    echo "[下载] 方式2：GitHub 直链(HTTP/1.1) ..."; curl -L --http1.1 --retry 5 --retry-delay 2 -o yolov8n.pt https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.pt 2>/dev/null && ok=1
  fi
  if [ "$ok" -ne 1 ] || [ "$(stat -c%s yolov8n.pt 2>/dev/null || echo 0)" -lt 1000000 ]; then
    echo "[下载] 方式3：上海交大镜像(mirror.sjtu.edu.cn) ..."; curl -L --http1.1 --retry 5 --retry-delay 2 -o yolov8n.pt https://mirror.sjtu.edu.cn/ultralytics/yolov8n.pt 2>/dev/null && ok=1
  fi
  if [ "$ok" -ne 1 ] || [ "$(stat -c%s yolov8n.pt 2>/dev/null || echo 0)" -lt 1000000 ]; then
    echo "[下载] 方式4：国内 HF 镜像(hf-mirror.com) ..."; curl -L --http1.1 --retry 5 --retry-delay 2 -o yolov8n.pt "https://hf-mirror.com/ultralytics/yolov8n/resolve/main/yolov8n.pt?download=true" 2>/dev/null && ok=1
  fi
  if [ -s yolov8n.pt ] && [ "$(stat -c%s yolov8n.pt 2>/dev/null || echo 0)" -ge 1000000 ]; then
    echo "[OK] 权重已就绪：$(ls -lh yolov8n.pt | awk '{print $5}')"
  else
    echo "[失败] 三种方式均未成功下载权重，请手动下载后 scp 到本目录"; exit 1
  fi
}
if [ ! -s yolov8n.pt ] || [ "$(stat -c%s yolov8n.pt 2>/dev/null || echo 0)" -lt 1000000 ]; then
  dl_yolov8n
else
  echo "[跳过] yolov8n.pt 已存在（$(ls -lh yolov8n.pt | awk '{print $5}')）"
fi

echo "==== 部署完成 ===="
echo "自检（不需摄像头/权重路径）：  python3 stage1_yolov8.py --selftest"
echo "运行（摄像头，SSH 无窗口）：    python3 stage1_yolov8.py --source 0 --model yolov8n.pt --headless"
echo "运行（接显示器看实时框）：      python3 stage1_yolov8.py --source 0 --model yolov8n.pt"
echo "提示：GF100 默认 /dev/video0；若打不开先 `ls /dev/video*` 确认设备号"
