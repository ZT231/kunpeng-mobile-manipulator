#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tts_push.py —— 把 TTS 生成的 wav 转 16k/16bit/单声道 PCM，经 TCP 推给 ESP32 播放

ESP32 侧 `audio_rx.c` 监听 9002，收到 PCM 就送 I2S1(MAX98357A) 播放。
这里负责「wav → 16kHz/mono/int16 PCM → TCP 9002」，两端采样参数写死一致，
和上行 audio_sink/audio_tx 用的是同一套 16k/16bit/单声道约定。

🔴 诚实边界：只做格式转换 + 推流，不做降噪、不做 VAD、不回调机械臂。

用法（零第三方依赖）：
  python3 tts_push.py --self-test            # 纯本地：造正弦/立体声 wav→转 PCM，不起网络
  python3 tts_push.py --wav /root/tts/x.wav --host <ESP32在热点里的IP> --port 9002
"""

import argparse
import math
import os
import socket
import struct
import sys
import tempfile
import wave

DEFAULT_HOST = ""   # STA 模式下 ESP32 IP 是热点动态分配，须显式 --host；旧 AP 地址 192.168.77.1 已废弃
DEFAULT_PORT = 9002
TARGET_RATE = 16000
TARGET_CHANNELS = 1
TARGET_WIDTH = 2   # 16-bit


def _emit(text, newline=True):
    """安全打印：板上 locale 可能是 C/POSIX，中文日志不能把进程搞崩。"""
    try:
        print(text, end="\n" if newline else "", flush=True)
    except UnicodeEncodeError:
        safe = str(text).encode("ascii", "backslashreplace").decode("ascii")
        print(safe, end="\n" if newline else "", flush=True)


def _resample_linear(samples, src_rate, dst_rate):
    """线性插值重采样到 dst_rate（纯 Python，短 TTS 音频够快）。"""
    if src_rate == dst_rate:
        return list(samples)
    ratio = src_rate / float(dst_rate)
    n_out = int(round(len(samples) / ratio))
    out = []
    for i in range(n_out):
        pos = i * ratio
        i0 = int(pos)
        i1 = min(i0 + 1, len(samples) - 1)
        frac = pos - i0
        out.append(samples[i0] + (samples[i1] - samples[i0]) * frac)
    return out


def _clip16(samples):
    out = []
    for s in samples:
        if s > 32767:
            s = 32767
        elif s < -32768:
            s = -32768
        out.append(int(s))
    return out


def wav_to_pcm16k_mono(path):
    """读 wav → 16kHz/16bit/单声道 PCM bytes。失败抛 ValueError / OSError。"""
    with wave.open(path, "rb") as wf:
        rate = wf.getframerate()
        ch = wf.getnchannels()
        width = wf.getsampwidth()
        raw = wf.readframes(wf.getnframes())

    if width == 2:
        fmt = "<%dh" % (len(raw) // 2)
        samples = list(struct.unpack(fmt, raw))
    elif width == 1:   # 8-bit 无符号 → 左移成 16bit
        samples = [(b - 128) << 8 for b in raw]
    elif width == 4:   # 32-bit 有符号 → 取高 16bit
        fmt = "<%di" % (len(raw) // 4)
        samples = [v >> 16 for v in struct.unpack(fmt, raw)]
    elif width == 3:   # 24-bit 有符号
        samples = []
        for i in range(0, len(raw) - 2, 3):
            v = raw[i] | (raw[i + 1] << 8) | (raw[i + 2] << 16)
            if v & 0x800000:
                v -= 0x1000000
            samples.append(v >> 8)
    else:
        raise ValueError("不支持的采样位宽: %d" % width)

    # 多声道 → 单声道（取平均）
    if ch > 1:
        mono = []
        step = ch
        for i in range(0, len(samples) - step + 1, step):
            mono.append(sum(samples[i:i + step]) // step)
        samples = mono

    if rate != TARGET_RATE:
        samples = _resample_linear(samples, rate, TARGET_RATE)

    samples = _clip16(samples)
    return struct.pack("<%dh" % len(samples), *samples)


def push(host, port, pcm):
    """把 PCM bytes 经一条 TCP 连接推给 ESP32。返回已发送字节数。"""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.settimeout(5.0)
        s.connect((host, port))
    except OSError as e:
        s.close()
        raise RuntimeError("连不上 ESP32 %s:%d —— 确认热点已起、音频下行任务已烧录（%s）"
                           % (host, port, e))
    try:
        total = 0
        while total < len(pcm):
            n = s.send(pcm[total:])
            if n <= 0:
                break
            total += n
    finally:
        s.close()
    return total


def self_test():
    # ① 16k 单声道：转换后长度应精确等于 0.5s * 16000 * 2B
    rate = TARGET_RATE
    n = int(rate * 0.5)
    samples = [int(12000 * math.sin(2 * math.pi * 440 * i / rate)) for i in range(n)]
    raw = struct.pack("<%dh" % n, *samples)
    fd, path = tempfile.mkstemp(suffix=".wav")
    os.close(fd)
    with wave.open(path, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(rate)
        wf.writeframes(raw)
    pcm = wav_to_pcm16k_mono(path)
    os.remove(path)
    assert len(pcm) == n * 2, "16k 单声道 PCM 长度不对: %d" % len(pcm)

    # ② 24k 立体声 → 16k 单声道：验证混声道 + 重采样都走通
    rate2 = 24000
    n2 = int(rate2 * 0.3)
    raw2 = struct.pack("<%dh" % (n2 * 2), *([8000] * (n2 * 2)))
    fd2, path2 = tempfile.mkstemp(suffix=".wav")
    os.close(fd2)
    with wave.open(path2, "wb") as wf:
        wf.setnchannels(2)
        wf.setsampwidth(2)
        wf.setframerate(rate2)
        wf.writeframes(raw2)
    pcm2 = wav_to_pcm16k_mono(path2)
    os.remove(path2)
    assert len(pcm2) == int(TARGET_RATE * 0.3) * 2, "24k 立体声转 16k 单声道长度不对: %d" % len(pcm2)

    _emit("SELF_TEST PASS（wav→16k/16bit/单声道 PCM 转换 + 重采样正确）")
    return 0


def main():
    ap = argparse.ArgumentParser(description="TTS wav → 16k PCM → ESP32:9002 播放")
    ap.add_argument("--self-test", action="store_true", help="纯本地转换自测，不起网络")
    ap.add_argument("--wav", default=None, help="要推送的 wav 文件")
    ap.add_argument("--host", default=DEFAULT_HOST, help="ESP32 在热点里的 IP（STA 动态，串口日志看“本机 IP=”）")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT, help="ESP32 音频下行端口，默认 9002")
    args = ap.parse_args()

    if args.self_test:
        return self_test()
    if not args.wav:
        _emit("错误：需要 --wav（或用 --self-test）")
        return 2
    if not args.host:
        _emit("错误：需要 --host（ESP32 连上热点后的 IP，串口日志里看“本机 IP=”）")
        return 2

    try:
        pcm = wav_to_pcm16k_mono(args.wav)
    except (ValueError, OSError) as e:
        _emit("wav 解析失败：%s" % e)
        return 3

    _emit("已转 16k/16bit/单声道 PCM：%.2f 秒（%d 字节）"
          % (len(pcm) / 2 / TARGET_RATE, len(pcm)))
    try:
        sent = push(args.host, args.port, pcm)
    except RuntimeError as e:
        _emit("推送失败：%s" % e)
        return 4
    _emit("已推送 %d 字节 → %s:%d" % (sent, args.host, args.port))
    return 0


if __name__ == "__main__":
    sys.exit(main())
