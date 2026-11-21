#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""voice_loop.py —— 端到端多轮语音闭环：录音 → ASR → 多步规划 → 问话/结论 → 再录

这是"最完整"的主入口。每轮：

    录音 N 秒(9001) ──► ASR 转文字 ──► planner 多步工具调用
       ▲                                      │
       │                              ┌───────┴────────┐
       │                              ▼                ▼
       │                          ask(要问用户)      done(结论)
       │                              │                │
       └──────── TTS 说出去 ◄─────────┘◄───────────────┘
                    （再录下一轮，续跑 or 新任务）

🔴 复用（不重写）：
  - audio_sink.run()      录音
  - asr.*                 语音 → 文字
  - planner.Planner       多步规划 + 仲裁 + 多轮续跑
  - tts.*                 文字 → 音频文件
  扬声器回放：本文件只调用 tts 生成音频并打印文本；**播放**通过 --play-cmd 注入
  （板上若装了 aplay/ffplay 可传入，未接扬声器就只看文本）。

🔴 安全口径：
  1) 默认 dry-run：不加 --apply，home/estop/resume 只模拟，臂绝不动。
  2) planner 的本地仲裁闸门照常生效：LLM 只提议，仲裁器授权。
  3) Key 只走环境变量：ASR_API_KEY / LLM_API_KEY / TTS_API_KEY（三者可同一值）。

用法（零第三方依赖）：
  python3 voice_loop.py --self-test
  # 真跑（key 已在环境变量）：
  python3 voice_loop.py --record 5 --apply \
      --asr-backend whisper --asr-model FunAudioLLM/SenseVoiceSmall \
      --llm-backend openai --llm-model Qwen/Qwen3.8-27B \
      --tts-backend siliconflow --tts-out-dir /root/tts
  # 单文件演练（不录音，读已有 WAV，全程 mock）：
  python3 voice_loop.py --wav /root/cap.wav
  # 多轮演练（两个已有音频当两轮输入）：
  python3 voice_loop.py --wavs 指令.m4a 回答.m4a --asr-backend whisper \
      --llm-backend openai --tts-backend siliconflow --tts-out-dir .
"""

import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_LLM_DIR = os.path.abspath(os.path.join(_HERE, "..", "llm"))
for _p in (_HERE, _LLM_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import asr as asr_mod           # noqa: E402
import audio_sink               # noqa: E402
import planner as pl            # noqa: E402
import tts as tts_mod           # noqa: E402


def _emit(text, newline=True):
    data = str(text)
    try:
        print(data, end="\n" if newline else "", flush=True)
    except UnicodeEncodeError:
        safe = data.encode("ascii", "backslashreplace").decode("ascii")
        print(safe, end="\n" if newline else "", flush=True)


# 硅基流动默认值（与 asr/tts/planner 一致）
SF_BASE = "https://api.siliconflow.cn/v1"
ASR_MODEL = "FunAudioLLM/SenseVoiceSmall"
LLM_MODEL = "Qwen/Qwen3.8-27B"


class VoiceLoop:
    """多轮闭环：记录 pending_ask 状态，决定下一句喂 planner.run 还是 continue_turn。"""

    def __init__(self, asr_be, llm_backend, tts_be, record_fn,
                 apply=False, play_fn=None):
        self.asr_be = asr_be
        self.tts_be = tts_be
        self.planner = pl.Planner(llm_backend, apply=apply)
        self.record_fn = record_fn
        self.play_fn = play_fn or (lambda wav: _emit("    （扬声器未接，音频已生成：%s）" % wav))
        self.pending_ask = False
        self.last_status = None

    def _speak(self, text):
        if not text:
            return
        wav = self.tts_be.synthesize(text)
        self.play_fn(wav)

    def step(self):
        """跑一轮：录音→ASR→规划→说。返回状态字符串；无音频时返回 no_audio。"""
        wav = self.record_fn()
        if wav is None:
            return "no_audio"
        try:
            text = self.asr_be.transcribe(wav)
        except RuntimeError as e:
            _emit("ASR 失败：%s" % e)
            return "asr_error"
        _emit("识别文本（来源 %s）：%r" % (self.asr_be.name, text))
        if not text.strip():
            _emit("没听到内容，跳过本轮")
            return "empty"

        result = (self.planner.continue_turn(text) if self.pending_ask
                  else self.planner.run(text))
        for s in result.steps:
            _emit("  [%s] %s -> %s" % (s["tool"], s["args"], s["result"]))

        if result.status == "ask":
            self.pending_ask = True
            self._speak(result.message)
        else:
            self.pending_ask = False
            self._speak(result.message)
        self.last_status = result.status
        _emit("本轮状态：%s" % result.status)
        return result.status

    def run_forever(self):
        while True:
            try:
                st = self.step()
                if st == "no_audio":
                    break
            except KeyboardInterrupt:
                _emit("\n已停止")
                break


# ---------------------------------------------------------------------------
# 自测：全 mock、不联网、不录音、不动臂
# ---------------------------------------------------------------------------
def self_test():
    texts = ["帮我把左前方的玩具熊猫拿过来", "放在我手上"]

    class _ScriptAsr:
        name = "script"

        def transcribe(self, wav):
            return texts.pop(0) if texts else ""

    loop = VoiceLoop(_ScriptAsr(), pl.MockBackend(), tts_mod.MockBackend(),
                     record_fn=lambda: "/tmp/fake.wav")
    # 第一轮：planner 拆步并停在 ask
    assert loop.step() == "ask", loop.last_status
    assert loop.pending_ask is True
    # 第二轮：用户回答 → continue_turn → place → done
    assert loop.step() == "done", loop.last_status
    assert loop.pending_ask is False

    _emit("SELF_TEST PASS（多轮闭环：ask→续跑→done 全验）")
    return 0


# ---------------------------------------------------------------------------
# 命令行外壳
# ---------------------------------------------------------------------------
def build_parser():
    p = argparse.ArgumentParser(description="端到端多轮语音闭环")
    p.add_argument("--self-test", action="store_true", help="全 mock 验闭环，不联网")

    g0 = p.add_argument_group("音频来源")
    g0.add_argument("--record", type=int, default=0, help="每轮录 N 秒（0=不录；给 --wav 时忽略）")
    g0.add_argument("--wav", default=None, help="读单个 WAV，跑一轮后退出")
    g0.add_argument("--wavs", nargs="+", default=None,
                    help="读多个 WAV 依次当多轮输入（如：指令.m4a 回答.m4a）")
    g0.add_argument("--sink-host", default="0.0.0.0")
    g0.add_argument("--sink-port", type=int, default=9001)
    g0.add_argument("--sink-out", default="/root/cap_live.wav")

    g1 = p.add_argument_group("ASR")
    g1.add_argument("--asr-backend", choices=("mock", "whisper"), default="mock")
    g1.add_argument("--asr-text", default="", help="mock 后端固定句")
    g1.add_argument("--asr-base-url", default=SF_BASE)
    g1.add_argument("--asr-model", default=ASR_MODEL)

    g2 = p.add_argument_group("LLM 规划器")
    g2.add_argument("--llm-backend", choices=("mock", "openai"), default="mock")
    g2.add_argument("--llm-base-url", default=SF_BASE)
    g2.add_argument("--llm-model", default=LLM_MODEL)

    g3 = p.add_argument_group("TTS")
    g3.add_argument("--tts-backend", choices=("mock", "siliconflow"), default="mock")
    g3.add_argument("--tts-base-url", default=SF_BASE)
    g3.add_argument("--tts-model", default=tts_mod.DEFAULT_MODEL)
    g3.add_argument("--tts-voice", default=tts_mod.DEFAULT_VOICE)
    g3.add_argument("--tts-format", default=tts_mod.DEFAULT_FORMAT)
    g3.add_argument("--tts-out-dir", default="/root/tts")
    g3.add_argument("--play-cmd", default="", help="播放命令，如 'aplay {}' 或 'ffplay -autoexit {}'")

    g4 = p.add_argument_group("执行")
    g4.add_argument("--apply", action="store_true", help="home/estop/resume 真下发")
    g4.add_argument("--timeout", type=float, default=120.0)
    return p


def main():
    args = build_parser().parse_args()

    if args.self_test:
        return self_test()

    # ASR / LLM / TTS 后端
    asr_be = asr_mod.build_backend(
        args.asr_backend, args.asr_base_url, args.asr_model,
        os.environ.get("ASR_API_KEY", ""), timeout=args.timeout,
        fixed_text=args.asr_text)

    if args.llm_backend == "mock":
        llm_be = pl.MockBackend()
    else:
        llm_be = pl.OpenAiBackend(args.llm_base_url, args.llm_model,
                                  os.environ.get("LLM_API_KEY", ""),
                                  timeout=args.timeout)

    tts_be = tts_mod.build_backend(
        args.tts_backend, args.tts_base_url, args.tts_model, args.tts_voice,
        os.environ.get("TTS_API_KEY", ""), args.tts_format, args.tts_out_dir,
        timeout=args.timeout)

    play_fn = None
    if args.play_cmd:
        def play_fn(wav, cmd=args.play_cmd):
            _emit("    （播放：%s）" % cmd.replace("{}", wav))
    if play_fn is None:
        play_fn = None  # VoiceLoop 会用默认提示

    # 录音函数
    if args.wavs:
        queue = list(args.wavs)

        def record_fn():
            return queue.pop(0) if queue else None
    elif args.wav:
        used = {"done": False}

        def record_fn():
            if used["done"]:
                return None
            used["done"] = True
            return args.wav
    elif args.record:
        def record_fn():
            audio_sink.run(args.sink_host, args.sink_port, args.sink_out, args.record)
            return args.sink_out
    else:
        _emit("错误：--wav 或 --record 至少要给一个（或用 --self-test）")
        return 1

    loop = VoiceLoop(asr_be, llm_be, tts_be, record_fn, apply=args.apply,
                     play_fn=play_fn)
    _emit("=" * 66)
    _emit("多轮语音闭环启动：ASR=%s  LLM=%s  TTS=%s" %
          (asr_be.name, llm_be.name, tts_be.name))
    _emit("模式：%s" % ("🔴 --apply 真下发" if args.apply else "✅ dry-run（臂绝不动）"))
    _emit("=" * 66)
    loop.run_forever()
    return 0


if __name__ == "__main__":
    sys.exit(main())
