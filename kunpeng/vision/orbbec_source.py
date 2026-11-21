# -*- coding: utf-8 -*-
"""
⚠️ 已废弃：本文件被 `kunpeng/vision/orbbec/` 包取代（source/depthproc/localize/cli）。
   请改用：  python -m kunpeng.vision.orbbec.cli probe
   此文件保留仅作早期草稿参考，不再维护。
"""
import argparse
import time
import numpy as np

try:
    from openni import openni2
except Exception as e:  # pragma: no cover
    openni2 = None
    OPENNI_IMPORT_ERR = repr(e)


class OrbbecSource:
    """DaBai DCW 的 OpenNI 取流封装：颜色(对齐到深度) + 深度 + 内参。"""

    def __init__(self, color_w=640, color_h=480, depth_w=640, depth_h=480):
        if openni2 is None:
            raise RuntimeError("openni2 未安装或加载失败: %s" % OPENNI_IMPORT_ERR)

        openni2.initialize()
        self.dev = openni2.Device.open_any()
        self.info = self.dev.get_device_info()
        self._opened = True

        self.depth = self.dev.create_depth_stream()
        self.color = self.dev.create_color_stream()

        # 深度对齐到彩色：让 depth_map 和 color_frame 同一像素对应同一个物点。
        # 这是抓取链路的关键一步 —— 否则 bbox 中心的深度是错位的。
        try:
            self.dev.set_image_registration_mode(
                openni2.IMAGE_REGISTRATION_DEPTH_TO_COLOR)
            self.registered = True
        except Exception:
            self.registered = False

        # 流分辨率/格式：DaBai DCW 走 USB2.0，同时开 RGB+Depth 时别贪高分辨率。
        _set_mode(self.depth, depth_w, depth_h)
        _set_mode(self.color, color_w, color_h)

        self.color.start()
        self.depth.start()

        self.K = self._intrinsics_from_fov(self.depth)

    # ---------- 内参 ----------
    @staticmethod
    def _intrinsics_from_fov(stream):
        """OpenNI 的 Python 绑定拿不到 cx/cy，只有 FOV；用 FOV 反推 fx/fy，
        cx/cy 取图像中心。若后续有 C++ 侧 getIntrinsic，优先用真值覆盖。"""
        w = stream.get_video_mode().resolutionX
        h = stream.get_video_mode().resolutionY
        hfov = stream.get_horizontal_fov()
        vfov = stream.get_vertical_fov()
        fx = (w / 2.0) / np.tan(np.deg2rad(hfov / 2.0))
        fy = (h / 2.0) / np.tan(np.deg2rad(vfov / 2.0))
        return {"fx": float(fx), "fy": float(fy),
                "cx": float(w / 2.0), "cy": float(h / 2.0),
                "w": int(w), "h": int(h),
                "source": "openni_fov", "registered": None}

    # ---------- 取流 ----------
    def read(self):
        """返回 (color_bgr, depth_mm, K)。失败返回 (None, None, self.K)。

        depth_mm：和 color_bgr 逐像素对齐的 uint16 深度图（单位 mm），0 = 无效。
        """
        d_f = self.depth.read_frame()
        c_f = self.color.read_frame()

        if d_f is None or c_f is None:
            return None, None, self.K

        depth_mm = np.frombuffer(d_f.get_buffer_as_uint16(), dtype=np.uint16)
        depth_mm = depth_mm.reshape((d_f.height, d_f.width))

        # 颜色帧对齐后分辨率以深度流为准；get_buffer_as_uint8 是 RGB888。
        rgb = np.frombuffer(c_f.get_buffer_as_uint8(), dtype=np.uint8)
        if self.registered:
            rgb = rgb.reshape((d_f.height, d_f.width, 3))
        else:
            rgb = rgb.reshape((c_f.height, c_f.width, 3))
        color_bgr = rgb[:, :, ::-1].copy()

        return color_bgr, depth_mm, self.K

    def stop(self):
        if getattr(self, "_opened", False):
            self.depth.stop()
            self.color.stop()
            openni2.unload()
            self._opened = False


def _set_mode(stream, w, h):
    """OpenNI 的 set_video_mode 参数是 (pixelFormat, resolutionX, resolutionY, fps)，
    只改分辨率、保留默认格式与帧率。"""
    vm = stream.get_video_mode()
    try:
        stream.set_video_mode(vm.pixelFormat, w, h, vm.fps)
    except Exception:
        # 部分发行版用 sensor 设 mode，失败就沿用默认，打印真实值提醒。
        pass
    vm = stream.get_video_mode()
    print("[mode] %dx%d @%d" % (vm.resolutionX, vm.resolutionY, vm.fps))


def probe():
    """设备探针：等价的 Linux 版 hello_orbbec。上板第一步先跑它。"""
    openni2.initialize()
    try:
        dev = openni2.Device.open_any()
        print("[device]", dev.get_device_info())
        d = dev.create_depth_stream()
        c = dev.create_color_stream()
        vm_d = d.get_video_mode()
        vm_c = c.get_video_mode()
        print("[depth mode] %dx%d@%d fmt=%d" %
              (vm_d.resolutionX, vm_d.resolutionY, vm_d.fps, vm_d.pixelFormat))
        print("[color mode] %dx%d@%d fmt=%d" %
              (vm_c.resolutionX, vm_c.resolutionY, vm_c.fps, vm_c.pixelFormat))
        print("[hfov] %.2f°  [vfov] %.2f°" %
              (d.get_horizontal_fov(), d.get_vertical_fov()))
    finally:
        openni2.unload()


def _stream_preview(seconds=5):
    src = OrbbecSource()
    t0 = time.time()
    n = 0
    try:
        while time.time() - t0 < seconds:
            bgr, depth, K = src.read()
            if bgr is None:
                continue
            n += 1
            # 中心点深度 + 有效深度占比，验证对齐与量程。
            cy, cx = bgr.shape[0] // 2, bgr.shape[1] // 2
            valid = float(np.count_nonzero(depth)) / depth.size
            print("frame=%d center_depth=%dmm valid=%.2f" %
                  (n, int(depth[cy, cx]), valid))
    finally:
        src.stop()
    print("[ok] 平均 %.1f FPS" % (n / max(time.time() - t0, 1e-3)))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="DaBai DCW 取流探针/预览")
    ap.add_argument("--probe", action="store_true", help="打印设备与流参数")
    ap.add_argument("--stream", action="store_true", help="取流预览 5 秒")
    ap.add_argument("--seconds", type=int, default=5)
    args = ap.parse_args()

    if openni2 is None:
        raise SystemExit("openni2 不可用：%s\n请先在鲲鹏上装 Orbbec SDK v1(aarch64) "
                         "并确认 import openni2 成功。" % OPENNI_IMPORT_ERR)

    if args.probe:
        probe()
    elif args.stream:
        _stream_preview(args.seconds)
    else:
        ap.print_help()
