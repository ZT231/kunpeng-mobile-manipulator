#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""asr.py —— 语音转文字（ASR）适配层：WAV → 文本，后端可插拔

大白话：语音链路里「耳朵」已经通了（ESP32 采音 → 9001 → audio_sink 落 WAV），
       这一步把 WAV 变成汉字。现在插的是假后端（mock，返回固定句子），
       等 KEY 到位后换云端 whisper 兼容接口，别处一行不改。

规范定义：自动语音识别（ASR, Automatic Speech Recognition）把音频波形映射为
         文本序列；本项目把它做成策略模式（Strategy Pattern），后端统一实现
         `transcribe(wav_path) -> str`，调用方只认这个插座，不关心背后是
         查表假后端还是云端接口。

🔴 诚实边界：
  1) mock 后端**不识别任何声音**，只返回一句固定文本 —— 它验证的是
     「WAV → 文本 → 下游」这条管道本身没坏，不验证识别正确率，
     也不许拿它的输出当成「ASR 已通」。
  2) Key 只走环境变量 `ASR_API_KEY`，绝不写进代码、绝不进 Git。
  3) 云端后端用 OpenAI 兼容 `POST /v1/audio/transcriptions`
     （OpenAI / Groq / 部分聚合器都认这个口）；阿里云 Paraformer / 讯飞 /
     火山 / 本地 sherpa-onnx 属另一套协议，需另加后端类 —— 接口已留好，
     新增子类即可，别处不改。

用法（零第三方依赖，只标准库）：
  python3 asr.py --self-test                      # 纯逻辑自测，不联网
  python3 asr.py --wav /root/cap.wav              # mock 转文字
  python3 asr.py --wav /root/cap.wav --text "回初始位"   # mock 返回指定句
  python3 asr.py --wav /root/cap.wav --backend whisper --model whisper-1
"""

import abc
import argparse
import json
import os
import sys
import urllib.error
import urllib.request
import uuid


# ---------------------------------------------------------------------------
# 输出层：板上 locale 不可控，先把打印做结实（与 voice_gateway 同一套口径）
# ---------------------------------------------------------------------------
def _emit(text, newline=True):
    """安全打印：ASCII 终端下中文也不许把进程搞崩。"""
    data = str(text)
    try:
        print(data, end="\n" if newline else "", flush=True)
    except UnicodeEncodeError:
        safe = data.encode("ascii", "backslashreplace").decode("ascii")
        print(safe, end="\n" if newline else "", flush=True)


# ---------------------------------------------------------------------------
# 后端插座
# ---------------------------------------------------------------------------
class AsrBackend(abc.ABC):
    """ASR 后端接口：统一只实现「给一个 WAV 路径，还一句文本」。

    为什么用 abc：不管背后是假表还是 HTTP，调用方都只认 transcribe()。
    拼错方法名、漏实现，实例化时就 TypeError，不会到运行时才炸。
    """
    name = "abstract"

    @abc.abstractmethod
    def transcribe(self, wav_path):
        """把 wav_path 转成文本；失败抛 RuntimeError。"""
        raise NotImplementedError


class MockBackend(AsrBackend):
    """不联网的假后端：返回固定句子。

    🔴 它不是「识别」的替身，而是「链路」的替身 —— 只用来验证
       WAV 进来、文本出去、下游能消费。识别质量没有意义。
    """
    name = "mock"

    def __init__(self, fixed_text=""):
        self.fixed_text = fixed_text
        self.calls = 0
        self.last_wav = None

    def transcribe(self, wav_path):
        self.calls += 1
        self.last_wav = wav_path
        return self.fixed_text if self.fixed_text else "把杯子抓起来"


class WhisperBackend(AsrBackend):
    """OpenAI 兼容的 ASR 后端：`POST {base}/audio/transcriptions`。

    兼容面：OpenAI（whisper-1）、Groq（whisper-large-v3 等）、以及多数
    提供 OpenAI 兼容音频转写口的聚合服务。换服务商只改 --base-url / --model。

    🔴 Key 只从环境变量 ASR_API_KEY 读（调用方传入，本类不自己 import 环境
       做隐式行为，保证可测）；本地没网时不要 new 这个后端。
    """
    name = "whisper"

    def __init__(self, base_url, model="whisper-1", api_key="",
                 timeout=60.0, language=None):
        self.base_url = base_url.rstrip("/")
        self.model = model
        self.api_key = api_key
        self.timeout = timeout
        self.language = language

    def transcribe(self, wav_path):
        try:
            with open(wav_path, "rb") as f:
                wav_bytes = f.read()
        except OSError as e:
            raise RuntimeError("读不到音频文件 %s：%s" % (wav_path, e))

        fields = {"model": self.model}
        if self.language:
            fields["language"] = self.language

        body, ctype = _multipart_body(
            fields, "file", os.path.basename(wav_path), wav_bytes, "audio/wav")
        url = self.base_url + "/audio/transcriptions"
        headers = {"Content-Type": ctype}
        if self.api_key:
            headers["Authorization"] = "Bearer " + self.api_key

        req = urllib.request.Request(url, data=body, headers=headers, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                raw = resp.read().decode("utf-8", "replace")
        except urllib.error.HTTPError as e:
            detail = e.read().decode("utf-8", "replace")[:300] if e.fp else ""
            raise RuntimeError("ASR 服务返回 HTTP %s：%s" % (e.code, detail))
        except urllib.error.URLError as e:
            raise RuntimeError("连不上 ASR 服务 %s（%s）。"
                               "确认网络可达、base_url 正确" % (self.base_url, e.reason))

        try:
            obj = json.loads(raw)
        except ValueError:
            raise RuntimeError("ASR 服务返回的不是 JSON：%s" % raw[:300])
        if "text" not in obj:
            raise RuntimeError("ASR 服务响应里没有 text 字段：%s" % raw[:300])
        return str(obj.get("text") or "").strip()


# ---------------------------------------------------------------------------
# multipart/form-data 手工编码（只用标准库，不引 requests）
# ---------------------------------------------------------------------------
def _multipart_body(fields, file_field, filename, file_bytes, content_type):
    """把「普通字段 + 一个文件」编成 multipart 请求体（纯函数）。

    返回 (body_bytes, Content-Type 字符串)。
    🔴 为什么不引 requests：板上「不擅自装包」，标准库一定在；这个编码
       就二十几行，为一个表单引入依赖不划算。
    """
    boundary = "----doubaoASR" + uuid.uuid4().hex
    parts = []
    for k, v in fields.items():
        parts.append(
            ("--" + boundary + "\r\n"
             'Content-Disposition: form-data; name="%s"\r\n\r\n%s\r\n'
             % (k, v)).encode("utf-8"))
    parts.append(
        ("--" + boundary + "\r\n"
         'Content-Disposition: form-data; name="%s"; filename="%s"\r\n'
         "Content-Type: %s\r\n\r\n" % (file_field, filename, content_type)
         ).encode("utf-8"))
    body = b"".join(parts) + file_bytes + ("\r\n--" + boundary + "--\r\n").encode("ascii")
    return body, "multipart/form-data; boundary=" + boundary


def build_backend(kind, base_url, model, api_key, timeout=60.0, language=None,
                  fixed_text=""):
    """按 kind 造一个后端（工厂函数）。

    kind=mock   → MockBackend（不联网，返回 fixed_text 或默认句）
    kind=whisper→ WhisperBackend（OpenAI 兼容）
    """
    if kind == "mock":
        return MockBackend(fixed_text)
    return WhisperBackend(base_url=base_url, model=model, api_key=api_key,
                          timeout=timeout, language=language)


# ---------------------------------------------------------------------------
# 自测：不联网、不读 Key、不需要真音频
# ---------------------------------------------------------------------------
def self_test():
    # 1) mock 后端：默认句 / 指定句都要原样返回
    assert MockBackend().transcribe("/tmp/a.wav") == "把杯子抓起来"
    assert MockBackend("回初始位").transcribe("/tmp/b.wav") == "回初始位"

    # 2) 工厂：mock 不联网即可造；whisper 不带 Key 也能造（Key 在请求时用）
    be = build_backend("mock", "", "", "")
    assert be.name == "mock"
    wb = build_backend("whisper", "http://127.0.0.1:8000/v1", "whisper-1", "")
    assert wb.name == "whisper" and wb.model == "whisper-1"

    # 3) multipart 编码：字段、文件名、文件字节、边界都要在 body 里
    body, ctype = _multipart_body({"model": "whisper-1"}, "file",
                                  "a.wav", b"RIFFxxxx", "audio/wav")
    assert body.startswith(b"--") and b"RIFFxxxx" in body
    assert b'name="model"' in body and b'name="file"' in body
    assert b'filename="a.wav"' in body and b"audio/wav" in body
    assert ctype.startswith("multipart/form-data; boundary=")

    _emit("SELF_TEST PASS")
    return 0


# ---------------------------------------------------------------------------
# 命令行外壳
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="语音转文字（ASR）适配层：WAV → 文本")
    ap.add_argument("--self-test", action="store_true", help="纯逻辑自测，不联网")
    ap.add_argument("--wav", default=None, help="WAV 文件路径")
    ap.add_argument("--backend", default="mock", choices=("mock", "whisper"),
                    help="mock=不联网假后端；whisper=OpenAI 兼容云端")
    ap.add_argument("--text", default="", help="mock 后端返回的固定句子")
    ap.add_argument("--base-url", default="https://api.openai.com/v1",
                    help="OpenAI 兼容 base_url")
    ap.add_argument("--model", default="whisper-1", help="ASR 模型名")
    ap.add_argument("--language", default=None, help="语言代码，如 zh；默认自动")
    ap.add_argument("--timeout", type=float, default=60.0, help="请求超时秒数")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    if not args.wav:
        _emit("错误：需要 --wav（或用 --self-test）")
        return 2

    api_key = os.environ.get("ASR_API_KEY", "")
    be = build_backend(args.backend, args.base_url, args.model, api_key,
                       timeout=args.timeout, language=args.language,
                       fixed_text=args.text)
    try:
        text = be.transcribe(args.wav)
    except RuntimeError as e:
        _emit("转写失败：%s" % e)
        return 3

    _emit("识别文本（来源 %s）：%s" % (be.name, text))
    return 0


if __name__ == "__main__":
    sys.exit(main())
