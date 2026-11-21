#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
audio_sink.py —— 鲲鹏侧 ESP32 PCM 音频接收端（TCP 9001，2026-11-21）

大白话：ESP32-S3 把麦克风采到的 PCM（16kHz / 16bit / 单声道）经 9001 端口
       持续灌进来，这个程序负责接住、落成一个标准 WAV 文件，拿回电脑就能放，
       用来实证「ESP32 采音 → Wi-Fi → 鲲鹏落盘」这一段到底通不通。
规范定义：这是**流式二进制接收端（Streaming PCM Sink）**。PCM 没有额外帧头，
       TCP 字节流本身就是边界，采样参数由两端约定写死；9001 与文本帧 9000 分开，
       避免二进制混进 `$...*CK` 把网关的 XOR 校验搞坏。

诚实边界：只收流、只落盘，不做 VAD / 不做降噪 / 不做 ASR / 不回调臂。
用法（零第三方依赖）：
  python3 audio_sink.py                    # 监听 0.0.0.0:9001 -> /root/cap.wav，不限时长
  python3 audio_sink.py --max-secs 5       # 录 5 秒自动封口
  python3 audio_sink.py --out /root/t.wav --port 9001
  python3 audio_sink.py --self-test        # 只验证 WAV 头拼接，不起端口
"""

import argparse
import socket
import struct
import sys

DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 9001
DEFAULT_OUT = "/root/cap.wav"

# 🔴 与 ESP32 audio_tx.c / i2s_mic.h 三处必须一致：16kHz、16bit、单声道
SAMPLE_RATE = 16000
BITS = 16
CHANNELS = 1
BLOCK_ALIGN = CHANNELS * BITS // 8      # 2 字节/帧
BYTE_RATE = SAMPLE_RATE * BLOCK_ALIGN   # 32000 字节/秒
RECV_CHUNK = 4096


def _emit(text, newline=True):
    """安全打印：板上 locale 可能是 C/POSIX，中文日志不能把进程搞崩。"""
    try:
        print(text, end="\n" if newline else "", flush=True)
    except UnicodeEncodeError:
        safe = str(text).encode("ascii", "backslashreplace").decode("ascii")
        print(safe, end="\n" if newline else "", flush=True)


def make_wav_header(data_size=0):
    """拼 44 字节标准 PCM WAV 头；长度未知时先写 0，收完再回填。"""
    return (
        b"RIFF" + struct.pack("<I", 36 + data_size) +
        b"WAVE" + b"fmt " + struct.pack("<IHHIIHH", 16, 1, CHANNELS, SAMPLE_RATE,
                                          BYTE_RATE, BLOCK_ALIGN, BITS) +
        b"data" + struct.pack("<I", data_size)
    )


def patch_wav_sizes(path, data_size):
    """收完后回填 RIFF 总长与 data 长度，让文件能被任意播放器打开。"""
    with open(path, "r+b") as f:
        f.seek(4)
        f.write(struct.pack("<I", 36 + data_size))
        f.seek(40)
        f.write(struct.pack("<I", data_size))


def self_test():
    h = make_wav_header(0)
    assert len(h) == 44, "WAV 头不是 44 字节: %d" % len(h)
    assert h[:4] == b"RIFF" and h[8:12] == b"WAVE" and h[36:40] == b"data"
    # 声道数 = 1、采样率 = 16000（LE 解出来核对）
    assert struct.unpack("<H", h[22:24])[0] == CHANNELS
    assert struct.unpack("<I", h[24:28])[0] == SAMPLE_RATE
    assert struct.unpack("<H", h[34:36])[0] == BITS
    _emit("SELF_TEST PASS")
    return 0


def run(host, port, out, max_secs):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    _emit("audio_sink listening %s:%d -> %s" % (host, port, out))
    if max_secs:
        _emit("最多录 %d 秒" % max_secs)
    _emit("提示：ESP32 复位后会连进来；Ctrl+C 可提前结束并封好 WAV")

    conn, addr = srv.accept()
    _emit("connected: %s:%d" % (addr[0], addr[1]))

    limit_bytes = (BYTE_RATE * max_secs) if max_secs else 0
    data_size = 0
    last_sec = 0

    f = open(out, "wb")
    f.write(make_wav_header(0))
    try:
        while True:
            if limit_bytes and data_size >= limit_bytes:
                _emit("已到 %d 秒上限，封口" % max_secs)
                break
            chunk = conn.recv(RECV_CHUNK)
            if not chunk:
                _emit("对端断开，封口")
                break
            f.write(chunk)
            data_size += len(chunk)
            cur_sec = data_size // BYTE_RATE
            if cur_sec != last_sec:
                last_sec = cur_sec
                _emit("已录 %d 秒（%d 字节）" % (cur_sec, data_size))
    finally:
        patch_wav_sizes(out, data_size)
        f.close()
        conn.close()
        srv.close()

    _emit("saved %.2fs (%d bytes) -> %s" % (data_size / BYTE_RATE, data_size, out))
    return 0


def main():
    ap = argparse.ArgumentParser(
        description="鲲鹏侧 ESP32 PCM 音频接收端：收 9001 流 -> 落 WAV")
    ap.add_argument("--self-test", action="store_true", help="只验 WAV 头拼接，不起端口")
    ap.add_argument("--host", default=DEFAULT_HOST, help="监听地址，默认 0.0.0.0")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT, help="监听端口，默认 9001")
    ap.add_argument("--out", default=DEFAULT_OUT, help="输出 WAV，默认 /root/cap.wav")
    ap.add_argument("--max-secs", type=int, default=0, help="最多录几秒，0=不限")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    try:
        return run(args.host, args.port, args.out, args.max_secs)
    except KeyboardInterrupt:
        _emit("\ninterrupted")
        return 0


if __name__ == "__main__":
    sys.exit(main())
