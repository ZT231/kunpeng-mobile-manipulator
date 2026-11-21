# -*- coding: utf-8 -*-
"""奥比中光 DaBai DCW 深度相机工具包。

今天只做"相机侧"链路，与机械臂完全解耦：
    取流(pyorbbecsdk v1) -> 深度预处理 -> 像素到 3D(相机系) -> 检测/测距演示

手眼标定明天回来再补，届时只需要把 localize 出的相机系 (x,y,z) 再乘一次
T_base_cam 即可，本包不需要改。
"""
from .source import DepthCamera, Intrinsics, DepthCameraError
from .depthproc import remove_flying, sample_region
from .localize import pixel_to_camera

__all__ = [
    "DepthCamera",
    "Intrinsics",
    "DepthCameraError",
    "remove_flying",
    "sample_region",
    "pixel_to_camera",
]
