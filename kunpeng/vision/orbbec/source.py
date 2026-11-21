# -*- coding: utf-8 -*-
"""DaBai DCW 深度相机取流层 —— Orbbec SDK v1 / pyorbbecsdk 1.3.x 版。

DaBai DCW 在 SDK 里自报成 "Gemini E, PID 0x065c"，走 OpenNI 私有协议，
不能用 cv2.VideoCapture，也不能用 Orbbec SDK v2。本文件用 pyorbbecsdk(v1)
的 Pipeline + Config + AlignFilter 取彩色/深度双流，并保持与旧 openni2 版
完全一致的上层接口（read()/intrinsics/depth/color/registered/synced）。

板上运行前提：
    export PYTHONPATH=/opt/pyorbbecsdk/install/lib:$PYTHONPATH
"""
import json
import math
import os

import numpy as np

try:
    import pyorbbecsdk as ob
except Exception as _e:  # pragma: no cover
    ob = None
    PB_IMPORT_ERR = repr(_e)

# DaBai DCW 官方深度 FOV；彩色 FOV 未公开，先用同口径占位（approx）。
SPEC_DEPTH_FOV_DEG = (79.0, 62.0)
SPEC_COLOR_FOV_DEG = (79.0, 62.0)


class DepthCameraError(RuntimeError):
    pass


class Intrinsics:
    """针孔内参 fx/fy/cx/cy + 分辨率。JSON 优先，FOV 反推为兜底(approx)。"""

    def __init__(self, fx, fy, cx, cy, w, h, approx=False, source="unknown"):
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = float(cx)
        self.cy = float(cy)
        self.w = int(w)
        self.h = int(h)
        self.approx = bool(approx)
        self.source = source

    def as_dict(self):
        return {
            "fx": self.fx, "fy": self.fy,
            "cx": self.cx, "cy": self.cy,
            "w": self.w, "h": self.h,
            "approx": self.approx, "source": self.source,
        }

    @classmethod
    def from_json(cls, path):
        with open(path, "r", encoding="utf-8") as f:
            d = json.load(f)
        # 兼容三种写法：camera_matrix(handeye_calib 输出) / mtx / K
        m = d.get("camera_matrix", d.get("mtx", d.get("K")))
        if m is None:
            raise DepthCameraError("内参 JSON 里没有 camera_matrix/mtx/K")
        m = np.asarray(m, dtype=float).reshape(3, 3)
        w = int(d.get("w", d.get("width", 0)))
        h = int(d.get("h", d.get("height", 0)))
        return cls(m[0, 0], m[1, 1], m[0, 2], m[1, 2], w, h,
                   approx=False, source=path)

    @classmethod
    def from_fov(cls, w, h, hfov_rad, vfov_rad):
        """FOV 反推：fx=(W/2)/tan(hfov/2)，cx/cy 取图像中心（近似）。"""
        fx = (float(w) / 2.0) / np.tan(float(hfov_rad) / 2.0)
        fy = (float(h) / 2.0) / np.tan(float(vfov_rad) / 2.0)
        return cls(fx, fy, float(w) / 2.0, float(h) / 2.0, w, h,
                   approx=True, source="fov")


class _StreamInfo:
    """把 pyorbbecsdk 的 VideoStreamProfile 适配成 cli.py 期望的接口。"""

    def __init__(self, profile, intrinsics):
        self._p = profile
        self.K = intrinsics

    def get_video_mode(self):
        return self

    @property
    def resolutionX(self):
        return int(self._p.get_width())

    @property
    def resolutionY(self):
        return int(self._p.get_height())

    @property
    def fps(self):
        return int(self._p.get_fps())

    @property
    def pixelFormat(self):
        try:
            return int(self._p.get_format())
        except Exception:
            return -1

    def get_width(self):
        return self.resolutionX

    def get_height(self):
        return self.resolutionY

    def get_horizontal_fov(self):
        return 2.0 * math.atan(self.K["w"] / (2.0 * self.K["fx"]))

    def get_vertical_fov(self):
        return 2.0 * math.atan(self.K["h"] / (2.0 * self.K["fy"]))


class DepthCamera:
    """一次 open 一个 DaBai DCW，read() 返回对齐好的彩色 + 深度 + 内参。

    color_w/color_h/depth_w/depth_h 传 0 表示用设备默认 profile
    （实测默认是 640x360@30：深度 Y11/Y16，彩色 MJPG）。
    """

    def __init__(self, color_w=0, color_h=0, depth_w=0, depth_h=0,
                 intrinsics_path=None, register=True, fps=30, timeout_ms=1000,
                 laser=True, depth_min=0, depth_max=0, hole_fill=None,
                 depth_auto_exp=True):
        if ob is None:
            raise DepthCameraError("pyorbbecsdk 未安装或导入失败: %s" % PB_IMPORT_ERR)

        self._closed = True
        self._timeout_ms = int(timeout_ms)
        self._pipeline = ob.Pipeline()
        self._info = self._read_info()
        self._config = ob.Config()

        self._depth_profile = self._pick_profile(
            ob.OBSensorType.DEPTH_SENSOR, depth_w, depth_h, fps)
        self._color_profile = self._pick_profile(
            ob.OBSensorType.COLOR_SENSOR, color_w, color_h, fps)
        self._config.enable_stream(self._depth_profile)
        self._config.enable_stream(self._color_profile)
        self._pipeline.start(self._config)
        self._closed = False

        # Gemini E 不支持硬件帧同步：enable_frame_sync() 会抛 OBError，
        # 且实测调用后彩色帧恒为 None（把彩色流破坏了），所以直接不调。
        self.synced = False

        # ---- 设备侧参数（实测决定深度质量的三件事）----
        # 1) 激光必须开：关掉后只剩被动双目，本场景直接 0 有效像素。
        # 2) MIN/MAX_DEPTH 是"掉电保留"的粘性属性：上次会话的钳位会静默生效，
        #    未显式指定时这里复位成不钳位。
        # 3) 一旦钳位，洞填充会把超量程区域填成边界值（假有效），必须关掉。
        self._dev = None
        try:
            self._dev = self._pipeline.get_device()
        except Exception:
            self._dev = None

        self.laser_on = False
        if laser:
            self.laser_on = self.set_laser(True)

        # 曝光也是粘性属性：上一次会话若锁过固定曝光，这里复位成自动曝光，
        # 否则换光照条件（教室/宿舍）会整片过曝或欠曝。
        if depth_auto_exp:
            self._set_bool_prop("OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL", True)

        if depth_min or depth_max:
            self.set_depth_range(depth_min, depth_max,
                                 hole_fill=False if hole_fill is None else hole_fill)
        else:
            self._set_int_prop("OB_PROP_MIN_DEPTH_INT", 0)
            self._set_int_prop("OB_PROP_MAX_DEPTH_INT", 10000)
            if hole_fill is not None:
                self._set_bool_prop("OB_PROP_DEPTH_HOLEFILTER_BOOL", hole_fill)

        self.props = self.get_props()

        # 深度对齐到彩色网格（D2C）。
        self._align = None
        if register:
            try:
                self._align = ob.AlignFilter(
                    align_to_stream=ob.OBStreamType.COLOR_STREAM)
                self.registered = True
            except Exception:
                self._align = None
                self.registered = False
        else:
            self.registered = False

        self.intrinsics = self._resolve_intrinsics(intrinsics_path)
        # 对齐后深度已重采样到彩色网格，内参与彩色完全一致（实测 depth_intrinsic==rgb_intrinsic）。
        # 只有不注册（无对齐）时才用深度规格 FOV 占位。
        self._K_depth = (self.intrinsics if self.registered else
                         Intrinsics.from_fov(
                             self._depth_profile.get_width(), self._depth_profile.get_height(),
                             math.radians(SPEC_DEPTH_FOV_DEG[0]),
                             math.radians(SPEC_DEPTH_FOV_DEG[1])).as_dict())
        self._K_color = self.intrinsics if not self.intrinsics.get("approx") else \
            Intrinsics.from_fov(
                self._color_profile.get_width(), self._color_profile.get_height(),
                math.radians(SPEC_COLOR_FOV_DEG[0]),
                math.radians(SPEC_COLOR_FOV_DEG[1])).as_dict()

        self.depth = _StreamInfo(self._depth_profile, self._K_depth)
        self.color = _StreamInfo(self._color_profile, self._K_color)

        # 预热：把彩色流启动阶段的前若干帧丢掉，保证第一次 read() 就是完整帧。
        self.warmup_ok = self._warmup(60)
        self._frame_id = 0

    # ---------------- 内部 ----------------
    def _read_info(self):
        info = {}
        try:
            dev = self._pipeline.get_device()
            di = dev.get_device_info()
            for k, fn in (("name", "get_name"), ("pid", "get_pid"),
                          ("vid", "get_vid"), ("serial", "get_serial_number"),
                          ("uid", "get_uid"), ("connection", "get_connection_type")):
                try:
                    info[k] = getattr(di, fn)()
                except Exception:
                    info[k] = None
            try:
                info["firmware"] = dev.get_firmware_version()
            except Exception:
                pass
        except Exception as e:
            info["error"] = repr(e)
        return info

    # ---------------- 设备参数 ----------------

    def _set_bool_prop(self, name, val):
        if self._dev is None:
            return False
        try:
            self._dev.set_bool_property(getattr(ob.OBPropertyID, name), bool(val))
            return True
        except Exception:
            return False

    def _set_int_prop(self, name, val):
        if self._dev is None:
            return False
        try:
            self._dev.set_int_property(getattr(ob.OBPropertyID, name), int(val))
            return True
        except Exception:
            return False

    def _get_bool_prop(self, name):
        if self._dev is None:
            return None
        try:
            return bool(self._dev.get_bool_property(getattr(ob.OBPropertyID, name)))
        except Exception:
            return None

    def _get_int_prop(self, name):
        if self._dev is None:
            return None
        try:
            return int(self._dev.get_int_property(getattr(ob.OBPropertyID, name)))
        except Exception:
            return None

    def set_laser(self, on=True):
        """开/关红外激光发射器。

        关掉后只剩被动双目：弱纹理（白墙、黑桌面）会大面积空洞，
        实测本房间关激光 = 0 有效像素。
        """
        ok = self._set_bool_prop("OB_PROP_LASER_BOOL", on)
        got = self._get_bool_prop("OB_PROP_LASER_BOOL")
        self.laser_on = bool(got) if got is not None else bool(ok and on)
        return self.laser_on

    def set_depth_range(self, min_mm=0, max_mm=0, hole_fill=False, settle=5):
        """限制深度有效范围（mm），并默认关闭洞填充。

        为什么必须关洞填充：钳位后超量程像素会被洞填充补成**边界值**
        （例：max=1500 时 2.2m 的墙变成 1477mm），在 sample_region 眼里
        就是"有效"的假障碍物。关掉后超量程诚实地变成 0。

        settle：属性下发后深度引擎要过几帧才生效，这里直接丢掉这几帧，
        避免调用方立刻 read() 拿到还没钳位的旧数据。

        注意：这两个属性掉电保留，换会话前不会自动恢复。
        """
        self._set_int_prop("OB_PROP_MIN_DEPTH_INT", int(min_mm))
        self._set_int_prop("OB_PROP_MAX_DEPTH_INT", int(max_mm))
        self._set_bool_prop("OB_PROP_DEPTH_HOLEFILTER_BOOL", bool(hole_fill))
        for _ in range(max(0, int(settle))):
            try:
                self._pipeline.wait_for_frames(200)
            except Exception:
                break
        self.props = self.get_props()
        return (int(min_mm), int(max_mm), bool(hole_fill))

    def get_props(self):
        names = (
            ("laser", "OB_PROP_LASER_BOOL", "bool"),
            ("min_depth", "OB_PROP_MIN_DEPTH_INT", "int"),
            ("max_depth", "OB_PROP_MAX_DEPTH_INT", "int"),
            ("hole_fill", "OB_PROP_DEPTH_HOLEFILTER_BOOL", "bool"),
            ("soft_filter", "OB_PROP_DEPTH_SOFT_FILTER_BOOL", "bool"),
            ("precision", "OB_PROP_DEPTH_PRECISION_LEVEL_INT", "int"),
            ("depth_exposure", "OB_PROP_DEPTH_EXPOSURE_INT", "int"),
            ("depth_gain", "OB_PROP_DEPTH_GAIN_INT", "int"),
            ("depth_auto_exp", "OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL", "bool"),
            ("color_exposure", "OB_PROP_COLOR_EXPOSURE_INT", "int"),
            ("color_gain", "OB_PROP_COLOR_GAIN_INT", "int"),
            ("ldp_status", "OB_PROP_LDP_STATUS_BOOL", "bool"),
        )
        out = {}
        for key, name, kind in names:
            out[key] = (self._get_bool_prop(name) if kind == "bool"
                        else self._get_int_prop(name))
        return out

    def _pick_profile(self, sensor_type, w, h, fps):
        """按传感器类型挑 profile。

        深度只能用 Y11/Y16/Z16，彩色用 MJPG/YUYV/RGB/BGRA。
        早期版本把 MJPG 也拿去试深度，在 640x480 下会拿到非法组合、表现为 0 帧。
        """
        pl = self._pipeline.get_stream_profile_list(sensor_type)
        is_depth = sensor_type == ob.OBSensorType.DEPTH_SENSOR
        fmts = ((ob.OBFormat.Y11, ob.OBFormat.Y16, ob.OBFormat.Z16)
                if is_depth else
                (ob.OBFormat.MJPG, ob.OBFormat.YUYV, ob.OBFormat.RGB, ob.OBFormat.BGRA))
        if w and h:
            # 按 尺寸+合法格式+fps 精确找
            for fmt in fmts:
                try:
                    return pl.get_video_stream_profile(w, h, fmt, fps)
                except Exception:
                    continue
            # 只按尺寸找，但格式必须合法
            try:
                for i in range(pl.get_count()):
                    p = pl.get_stream_profile_by_index(i)
                    if hasattr(p, "as_video_stream_profile"):
                        try:
                            p = p.as_video_stream_profile()
                        except Exception:
                            pass
                    if not hasattr(p, "get_width"):
                        continue
                    if p.get_width() == w and p.get_height() == h:
                        try:
                            fi = int(p.get_format())
                        except Exception:
                            fi = -1
                        if any(fi == int(f) for f in fmts):
                            return p
            except Exception:
                pass
        return pl.get_default_video_stream_profile()

    def _resolve_intrinsics(self, intrinsics_path):
        if intrinsics_path and os.path.isfile(intrinsics_path):
            return Intrinsics.from_json(intrinsics_path).as_dict()
        # 设备内参：DaBai DCW/Gemini E 实测返回全 0，有值才用。
        try:
            cp = self._pipeline.get_camera_param()
            ri = cp.rgb_intrinsic
            if float(ri.fx) > 0.0 and int(ri.width) > 0:
                return Intrinsics(ri.fx, ri.fy, ri.cx, ri.cy,
                                  ri.width, ri.height,
                                  approx=False, source="device").as_dict()
        except Exception:
            pass
        # FOV 兜底：对齐后用彩色网格口径，否则用深度口径。
        if self.registered:
            return Intrinsics.from_fov(
                self._color_profile.get_width(), self._color_profile.get_height(),
                math.radians(SPEC_COLOR_FOV_DEG[0]),
                math.radians(SPEC_COLOR_FOV_DEG[1])).as_dict()
        return Intrinsics.from_fov(
            self._depth_profile.get_width(), self._depth_profile.get_height(),
            math.radians(SPEC_DEPTH_FOV_DEG[0]),
            math.radians(SPEC_DEPTH_FOV_DEG[1])).as_dict()

    @staticmethod
    def _fmt_int(frame):
        """取帧格式的整数值；pyorbbecsdk 里 get_format() 有时给枚举、有时给 int。"""
        try:
            return int(frame.get_format())
        except Exception:
            pass
        try:
            return int(frame.get_format().value)
        except Exception:
            return -1

    @classmethod
    def _to_bgr(cls, frame):
        """把彩色帧转成 HxWx3 uint8 BGR。格式一律按整数值比较，避免枚举名差异。"""
        w, h = int(frame.get_width()), int(frame.get_height())
        raw = np.frombuffer(frame.get_data(), dtype=np.uint8)
        import cv2

        fi = cls._fmt_int(frame)
        f_mjpg = int(ob.OBFormat.MJPG)
        f_rgb = int(ob.OBFormat.RGB)
        f_bgra = int(ob.OBFormat.BGRA)
        f_yuyv = int(ob.OBFormat.YUYV)

        if fi == f_mjpg:
            img = cv2.imdecode(raw, cv2.IMREAD_COLOR)
            if img is None:
                raise DepthCameraError("MJPG 解码失败 raw=%d" % raw.size)
            return img
        if fi == f_rgb:
            return raw.reshape((h, w, 3))[:, :, ::-1].copy()
        if fi == f_bgra:
            return raw.reshape((h, w, 4))[:, :, :3].copy()
        if fi == f_yuyv:
            return cv2.cvtColor(raw.reshape((h, w, 2)), cv2.COLOR_YUV2BGR_YUY2)
        # 兜底：先当 MJPG 试一次
        img = cv2.imdecode(raw, cv2.IMREAD_COLOR)
        if img is not None:
            return img
        raise DepthCameraError("无法解码彩色帧 fmt=%s" % (frame.get_format(),))

    def _grab(self):
        """取一帧，返回 (depth_frame, color_frame)；失败给 (None, None)。"""
        try:
            frames = self._pipeline.wait_for_frames(self._timeout_ms)
        except Exception:
            return None, None
        if frames is None:
            return None, None
        if self._align is not None:
            try:
                frames = self._align.process(frames)
            except Exception:
                # 对齐失败绝不能让"未对齐帧"冒充对齐帧（像素→3D 会静默错位）
                frames = None
        if frames is None:
            return None, None
        return frames.get_depth_frame(), frames.get_color_frame()

    def _warmup(self, max_frames=60):
        """彩色流启动后会先丢若干帧才出图（实测前约 9 帧 color=None），先空跑丢弃。"""
        for _ in range(max_frames):
            d, c = self._grab()
            if d is not None and c is not None:
                return True
        return False

    def read(self, with_color=True, drop_stale=0, retries=2):
        """返回 dict：
        color_bgr  HxWx3 uint8（BGR）；with_color=False 时为 None
        depth_mm   HxW uint16（已 D2C 对齐到彩色网格，0=无效）
        K          内参 dict
        ok         是否成功

        with_color=False  跳过 MJPG 解码（省 ~2ms，只要深度的场合用）
        drop_stale=N      先丢弃 N 帧陈旧数据再取最新（消费者慢时防延迟累积）
        """
        if self._closed:
            return {"ok": False}

        for _ in range(max(0, int(drop_stale))):
            try:
                if self._pipeline.wait_for_frames(1) is None:
                    break
            except Exception:
                break

        # 彩色偶发掉帧，最多重试几次直到深度+彩色同时就绪
        d_frame = c_frame = None
        for _ in range(max(1, int(retries))):
            d_frame, c_frame = self._grab()
            if d_frame is not None and (c_frame is not None or not with_color):
                break
        if d_frame is None:
            return {"ok": False}

        dh, dw = int(d_frame.get_height()), int(d_frame.get_width())
        raw = np.frombuffer(d_frame.get_data(), dtype=np.uint16)
        # 行跨距可能大于 dw*2（对齐/转换后带 padding），按 data_size 反推列数
        try:
            dsize = int(d_frame.get_data_size())
        except Exception:
            dsize = 0
        row_words = (dsize // dh // 2) if (dsize and dh) else dw
        if row_words > dw and raw.size >= dh * row_words:
            depth_mm = raw[:dh * row_words].reshape((dh, row_words))[:, :dw].copy()
        else:
            depth_mm = raw[:dh * dw].reshape((dh, dw)).copy()
        try:
            scale = float(d_frame.get_depth_scale())
            if scale and abs(scale - 1.0) > 1e-9:
                depth_mm = (depth_mm.astype(np.float32) * scale).astype(np.uint16)
        except Exception:
            pass

        color_bgr = None
        color_err = None
        if not with_color:
            color_err = "skipped"
        elif c_frame is not None:
            try:
                color_bgr = self._to_bgr(c_frame)
            except Exception as e:
                color_bgr = None
                color_err = repr(e)
        else:
            color_err = "color frame is None"

        # 对齐后深度应已是彩色网格；若仍不一致，近邻缩放到彩色网格。
        if color_bgr is not None and (dh, dw) != color_bgr.shape[:2]:
            import cv2
            depth_mm = cv2.resize(depth_mm, (color_bgr.shape[1], color_bgr.shape[0]),
                                  interpolation=cv2.INTER_NEAREST)

        ts = None
        try:
            ts = d_frame.get_timestamp()
            ts = ts.total_seconds() if hasattr(ts, "total_seconds") else ts
        except Exception:
            pass

        self._frame_id += 1
        return {
            "ok": True,
            "frame_id": self._frame_id,
            "timestamp": ts,
            "color_bgr": color_bgr,
            "depth_mm": depth_mm,
            "K": self.intrinsics,
            "registered": self.registered,
            "color_err": color_err,
        }

    def close(self):
        if not self._closed:
            try:
                self._pipeline.stop()
            except Exception:
                pass
            self._closed = True
