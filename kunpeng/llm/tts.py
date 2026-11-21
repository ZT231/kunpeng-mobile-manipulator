#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tts.py —— 文字转语音（TTS）适配层：文本 → 音频文件，后端可插拔

把大脑（planner）要说的问句/结论变成能播的音频。当前后端用硅基流动
`FunAudioLLM/CosyVoice2-0.5B`（OpenAI 兼容 `/v1/audio/speech`，150ms 级流式，
中文含方言），Key 走环境变量 TTS_API_KEY。

规范定义：文本转语音（TTS, Text-To-Speech）把文本序列合成为语音波形；
本项目做成策略模式，后端统一实现 `synthesize(text) -> wav_path`。

🔴 诚实边界：
  1) mock 后端不合成任何声音，只打印文本并返回假路径——它验证「文本→(假)音频→
     下游」管道，不代表"会说话"。
  2) 扬声器回放链路（鲲鹏音频输出 或 ESP32 MAX98357A）尚未接；本文件只负责
     **生成音频文件**，播放由上层注入 `play_fn` 决定。
  3) Key 只走环境变量 TTS_API_KEY，绝不进代码/Git。

用法（零第三方依赖）：
  python3 tts.py --self-test
  python3 tts.py --text "任务完成，要我把它放到哪里？"          # mock 打印
  python3 tts.py --text "..." --backend siliconflow --out-dir /root/tts  # 真合成
"""

import abc
import argparse
import json
import os
import sys
import urllib.error
import urllib.request
import uuid


def _emit(text, newline=True):
    data = str(text)
    try:
        print(data, end="\n" if newline else "", flush=True)
    except UnicodeEncodeError:
        safe = data.encode("ascii", "backslashreplace").decode("ascii")
        print(safe, end="\n" if newline else "", flush=True)


# 与硅基流动 /v1/audio/speech 对齐的默认值（2026-11-21 查证）
DEFAULT_BASE_URL = "https://api.siliconflow.cn/v1"
DEFAULT_MODEL = "FunAudioLLM/CosyVoice2-0.5B"
DEFAULT_VOICE = "FunAudioLLM/CosyVoice2-0.5B:alex"   # 系统预置音色
DEFAULT_FORMAT = "wav"                                 # 支持 mp3/wav/pcm/opus


class TtsBackend(abc.ABC):
    name = "abstract"

    @abc.abstractmethod
    def synthesize(self, text):
        """text -> 音频文件路径；失败抛 RuntimeError。"""
        raise NotImplementedError


class MockBackend(TtsBackend):
    """不联网的假后端：打印文本、返回假路径。"""
    name = "mock"

    def __init__(self, out_dir="/tmp"):
        self.out_dir = out_dir
        self.calls = 0

    def synthesize(self, text):
        self.calls += 1
        _emit("[TTS] %s" % text)
        return os.path.join(self.out_dir, "tts_mock_%s.wav" % uuid.uuid4().hex[:8])


class SiliconFlowTtsBackend(TtsBackend):
    """OpenAI 兼容 `/v1/audio/speech`（硅基流动 CosyVoice2-0.5B）。

    响应是**裸音频字节**，不是 JSON；按 response_format 存成对应扩展名。
    """
    name = "siliconflow"

    def __init__(self, base_url=DEFAULT_BASE_URL, model=DEFAULT_MODEL,
                 voice=DEFAULT_VOICE, api_key="", response_format=DEFAULT_FORMAT,
                 out_dir="/root/tts", timeout=60.0):
        self.base_url = base_url.rstrip("/")
        self.model = model
        self.voice = voice
        self.api_key = api_key
        self.response_format = response_format
        self.out_dir = out_dir
        self.timeout = timeout

    def synthesize(self, text):
        os.makedirs(self.out_dir, exist_ok=True)
        payload = {"model": self.model, "input": text, "voice": self.voice,
                   "response_format": self.response_format}
        body = json.dumps(payload).encode("utf-8")
        headers = {"Content-Type": "application/json"}
        if self.api_key:
            headers["Authorization"] = "Bearer " + self.api_key
        req = urllib.request.Request(self.base_url + "/audio/speech",
                                     data=body, headers=headers, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                audio = resp.read()
        except urllib.error.HTTPError as e:
            detail = e.read().decode("utf-8", "replace")[:300] if e.fp else ""
            raise RuntimeError("TTS 服务返回 HTTP %s：%s" % (e.code, detail))
        except urllib.error.URLError as e:
            raise RuntimeError("连不上 TTS 服务 %s（%s）" % (self.base_url, e.reason))

        if not audio:
            raise RuntimeError("TTS 返回空音频")
        out = os.path.join(self.out_dir, "tts_%s.%s" % (uuid.uuid4().hex[:8],
                                                        self.response_format))
        with open(out, "wb") as f:
            f.write(audio)
        return out


def build_backend(kind, base_url, model, voice, api_key, response_format, out_dir,
                  timeout=60.0):
    if kind == "mock":
        return MockBackend(out_dir)
    return SiliconFlowTtsBackend(base_url=base_url, model=model, voice=voice,
                                 api_key=api_key, response_format=response_format,
                                 out_dir=out_dir, timeout=timeout)


def self_test():
    # mock 后端：不联网、返回假路径、打印文本
    mb = MockBackend()
    p = mb.synthesize("测试一句话")
    assert isinstance(p, str) and p.endswith(".wav") and mb.calls == 1

    # siliconflow 后端不带 Key 也能构造（Key 在请求时用）
    be = build_backend("siliconflow", DEFAULT_BASE_URL, DEFAULT_MODEL, DEFAULT_VOICE,
                       "", DEFAULT_FORMAT, "/tmp")
    assert be.name == "siliconflow" and be.voice == DEFAULT_VOICE
    assert be.base_url.endswith("/v1") and be.model == DEFAULT_MODEL

    _emit("SELF_TEST PASS")
    return 0


def main():
    ap = argparse.ArgumentParser(description="文字转语音（TTS）适配层")
    ap.add_argument("--self-test", action="store_true", help="纯逻辑自测，不联网")
    ap.add_argument("--text", default=None, help="要合成的文本")
    ap.add_argument("--backend", choices=("mock", "siliconflow"), default="mock")
    ap.add_argument("--base-url", default=DEFAULT_BASE_URL)
    ap.add_argument("--model", default=DEFAULT_MODEL)
    ap.add_argument("--voice", default=DEFAULT_VOICE)
    ap.add_argument("--format", default=DEFAULT_FORMAT)
    ap.add_argument("--out-dir", default="/root/tts")
    ap.add_argument("--timeout", type=float, default=60.0)
    args = ap.parse_args()

    if args.self_test:
        return self_test()
    if not args.text:
        _emit("错误：需要 --text（或用 --self-test）")
        return 2

    be = build_backend(args.backend, args.base_url, args.model, args.voice,
                       os.environ.get("TTS_API_KEY", ""), args.format,
                       args.out_dir, timeout=args.timeout)
    try:
        out = be.synthesize(args.text)
    except RuntimeError as e:
        _emit("合成失败：%s" % e)
        return 3
    _emit("已合成（来源 %s）：%s" % (be.name, out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
