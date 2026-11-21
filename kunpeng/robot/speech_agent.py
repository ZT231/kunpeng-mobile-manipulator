#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""speech_agent.py —— 端到端编排：录音 → ASR → 路由 → 执行

把「说一句话 → 理解 → 看图 → 规划 → 执行」串成一条可跑的主链路：

    ESP32 采音(9001) ──► audio_sink 落 WAV ──► asr 转文字
       │                                        │
       │                                        ▼
       │                              route() 分两条通道
       │                    ┌───────────────┴───────────────┐
       │                    ▼                               ▼
       │            快通道(本地、<300ms)              慢通道(视觉+VLM+IK)
       │            voice_gateway.arbitrate          pick_pipeline.run
       │            急停/恢复/回初始位                抓取/放置（含看图）
       │                    │                               │
       │                    └───────────┬───────────────────┘
       │                                ▼
       │                        Flask /api/cmd 下发 → 机械臂
       └──（9001 音频上行与 9000 文本帧是 ESP32 的两条上行，本文件不碰）

🔴 分工（复用，不重写，删本文件即完整回滚）：
  - audio_sink.run()              录音（9001 → WAV）
  - asr.build_backend().transcribe()   语音 → 文字
  - voice_gateway.arbitrate/send_cmd   急停/恢复/回初始位 安全快通道
  - pick_pipeline.run()                抓取/放置（视觉 + VLM + IK + 回读确认）

🔴 安全口径（沿用项目红线）：
  1) 「会动的东西走本地快通道，会说话的东西走云端慢通道」——
     急停/恢复/回初始位由本地仲裁器授权，LLM/VLM 只对「抓取」这类复杂任务
     做提议，且仍经过 pick_pipeline 的人闸/贴边闸/软限位闸，模型不直连执行器。
  2) 默认 dry-run：不加 --apply，任何命令都不会真发，臂绝不动。
  3) Key 只走环境变量：ASR_API_KEY、VLM_API_KEY；绝不写进代码、不进 Git。

🔴 诚实边界：
  1) ASR 默认 mock（不识别声音，只返回固定句）；VLM 默认 mock（按类名关键词
     选目标）。二者都只验证「管道」，不代表「听懂了/看懂了」。真模型 = 把
     --asr-backend / --vlm-backend 换成 whisper / vlm 并配好 Key 与地址。
  2) 「放到左边」这类方位目前落成 --place-x/--place-y 的**示教坐标**（COCO 里
     没有 box 类），不是模型算出来的；放置点是人在实机上量好后写死的。
  3) 录音是「连上后录 N 秒」的停-走模式，没有 VAD；说话要赶在 N 秒内。
     连续唤醒 + 端点检测是下一步，不是本文件的职责。

用法（板 · openEuler，零第三方依赖）：
  python3 speech_agent.py --self-test                 # 只验路由，不联网不动臂
  python3 speech_agent.py --wav /root/cap.wav --dry-run   # 已有 WAV 演练
  python3 speech_agent.py --record 5 --apply          # 真录音 5s → 识别 → 执行
  python3 speech_agent.py --record 5 --apply \\
          --asr-text "把杯子抓起来" --place-x 6 --place-y 20   # mock ASR 演练
"""

import argparse
import dataclasses
import json
import os
import sys

# ---------------------------------------------------------------------------
# 跨目录复用：本文件在 robot/，asr 在 ../llm/；板上扁平布局时又在同目录。
# 两种布局都要能 import（与 pick_pipeline 同一套口径）。
# ---------------------------------------------------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))
_LLM_DIR = os.path.abspath(os.path.join(_HERE, "..", "llm"))
for _p in (_HERE, _LLM_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import asr as asr_mod                       # noqa: E402
import audio_sink                           # noqa: E402
import pick_pipeline as pp                  # noqa: E402
import vlm_decider as vd                    # noqa: E402
import voice_gateway as vg                  # noqa: E402


# ---------------------------------------------------------------------------
# 输出层
# ---------------------------------------------------------------------------
def _emit(text, newline=True):
    data = str(text)
    try:
        print(data, end="\n" if newline else "", flush=True)
    except UnicodeEncodeError:
        safe = data.encode("ascii", "backslashreplace").decode("ascii")
        print(safe, end="\n" if newline else "", flush=True)


# ---------------------------------------------------------------------------
# 路由：一句文本 → 快通道 / 慢通道 / 观察 / 不执行
# ---------------------------------------------------------------------------
@dataclasses.dataclass
class RouteDecision:
    """一次路由结果（带名字的数据袋子）。"""
    channel: str = "none"        # fast | slow | look | none
    payload: str = ""            # fast：HOME/STOP/RESUME
    tag: str = ""
    reason: str = ""
    new_estop: object = None     # None=不改 / True=进入急停 / False=解除
    command: dict = dataclasses.field(default_factory=dict)


def route(text, estop_active=False):
    """把 ASR 转出的文本分到对应通道（纯函数：不联网、不碰臂）。

    🔴 复用 voice_gateway.arbitrate —— 急停/恢复/回初始位 的本地仲裁、
       白名单、急停冻结，全都已经写在那里并过了 38 项单测，这里不重写。
       只有它 REJECT 的「抓取/放置」才转交慢通道 pick_pipeline。
    """
    verdict = vg.arbitrate(text, estop_active=estop_active)
    d = RouteDecision(new_estop=verdict.set_estop,
                      command=verdict.command,
                      reason=verdict.reason)

    if verdict.decision == vg.Decision.ALLOW:
        d.channel = "fast"
        d.payload = verdict.payload
        d.tag = verdict.tag
    elif verdict.decision == vg.Decision.NOOP:
        d.channel = "look"
        d.tag = verdict.tag
    elif verdict.decision == vg.Decision.REJECT:
        act = (verdict.command or {}).get("action")
        if act in ("pick", "place"):
            d.channel = "slow"
            d.reason = verdict.reason
    return d


# ---------------------------------------------------------------------------
# 三个执行分支
# ---------------------------------------------------------------------------
def exec_fast(d, apply, api):
    """快通道：急停/恢复/回初始位，走 Flask /api/cmd（复用 voice_gateway）。"""
    _emit("  [快通道] %s → %s" % (d.reason or d.tag, d.payload))
    if not apply:
        _emit("    dry-run：本该下发 %s，因未加 --apply 未发" % d.payload)
        return 0
    try:
        vg.send_cmd(d.payload, base_url=api)
    except RuntimeError as e:
        _emit("    下发失败：%s" % e)
        return 4
    _emit("    已下发 %s，固件回 OK" % d.payload)
    return 0


def exec_look(args):
    """观察通道：只拉一帧候选清单打印，绝不动臂。"""
    try:
        fid, cands, w, h, _jpeg = vd.fetch_observation(args.detector,
                                                       timeout=args.observe_timeout)
    except RuntimeError as e:
        _emit("    观测失败：%s" % e)
        return 3
    _emit("    画面 %d×%d 帧 %s，共 %d 个候选（仅观察，不动臂）"
          % (w, h, fid, len(cands)))
    for c in cands:
        _emit("      [%d] %-12s conf=%.2f center=(%d,%d) 方位=%s"
              % (c.idx, c.cls, c.conf, c.center[0], c.center[1],
                 c.direction or "-"))
    return 0


def exec_slow(text, args):
    """慢通道：抓取/放置，整段交给 pick_pipeline（视觉 + VLM + IK + 执行）。

    🔴 用 pick_pipeline.build_parser().parse_args([]) 拿它自己的全部默认值，
       再只覆盖与本编排器有关的字段 —— 避免在这里再复制一份参数表，
       将来 pick_pipeline 加参数这里不会漂移。
    """
    pargs = pp.build_parser().parse_args([])
    pargs.instruction = text
    pargs.api = args.api
    pargs.detector = args.detector
    pargs.backend = args.vlm_backend
    pargs.model = args.vlm_model
    pargs.base_url = args.vlm_base_url
    pargs.model_timeout = args.model_timeout
    pargs.observe_timeout = args.observe_timeout
    pargs.grab_z = args.grab_z
    pargs.hover = args.hover
    pargs.place_x = args.place_x
    pargs.place_y = args.place_y
    pargs.person_iou = args.person_iou
    pargs.jaw_max_cm = args.jaw_max_cm
    pargs.yes = args.apply
    pargs.dry_run = not args.apply
    pargs.keep_proxy = args.keep_proxy
    pargs.save_frame = args.save_frame
    pargs.dynamic_grip = args.dynamic_grip
    _emit("  [慢通道] 交给 pick_pipeline（视觉+VLM+IK+执行）")
    return pp.run(pargs)


# ---------------------------------------------------------------------------
# 一轮：录音（或读 WAV）→ ASR → 路由 → 执行
# ---------------------------------------------------------------------------
def one_round(args, state):
    wav = args.wav
    if wav is None and args.record:
        wav = args.sink_out
        _emit("=" * 66)
        _emit("录音 %d 秒（9001）……对着麦说话" % args.record)
        try:
            audio_sink.run(args.sink_host, args.sink_port, wav, args.record)
        except KeyboardInterrupt:
            _emit("\n录音被中断")
            return 130
        except OSError as e:
            _emit("录音失败：%s" % e)
            return 3

    if wav is None:
        _emit("没有音频来源：给 --wav <路径> 或 --record <秒数>")
        return 2

    be = asr_mod.build_backend(
        args.asr_backend, args.asr_base_url, args.asr_model,
        os.environ.get("ASR_API_KEY", ""), timeout=args.model_timeout,
        language=args.asr_language, fixed_text=args.asr_text)
    try:
        text = be.transcribe(wav)
    except RuntimeError as e:
        _emit("ASR 失败：%s" % e)
        return 3

    _emit("-" * 66)
    _emit("识别文本（来源 %s）：%r" % (be.name, text))
    if not text.strip():
        _emit("没听到内容（空文本），跳过本轮")
        return 3

    d = route(text, estop_active=state.estop)
    if d.new_estop is not None:
        state.estop = bool(d.new_estop)
        _emit("  急停状态 → %s" % ("冻结" if state.estop else "解除"))

    if d.channel == "fast":
        return exec_fast(d, args.apply, args.api)
    if d.channel == "look":
        return exec_look(args)
    if d.channel == "slow":
        return exec_slow(text, args)

    _emit("  [不执行] %s" % d.reason)
    return 2


# ---------------------------------------------------------------------------
# 自测：只验路由，不联网、不动臂、不录音
# ---------------------------------------------------------------------------
def self_test():
    # 快通道：急停 / 恢复 / 回初始位
    d = route("急停")
    assert d.channel == "fast" and d.payload == vg.CMD_STOP and d.new_estop is True, d
    d = route("解除急停", estop_active=True)
    assert d.channel == "fast" and d.payload == vg.CMD_RESUME and d.new_estop is False, d
    d = route("回初始位")
    assert d.channel == "fast" and d.payload == vg.CMD_HOME, d

    # 急停冻结：除了恢复，别的都别想动
    d = route("回初始位", estop_active=True)
    assert d.channel == "none", d

    # 慢通道：抓取 / 放置
    assert route("把杯子抓起来").channel == "slow"
    assert route("把红色零件放进盒子").channel == "slow"

    # 观察通道
    d = route("描述一下当前画面")
    assert d.channel == "look", d

    # 不执行：闲聊 / 移动（移动未实现，mock 不授权）
    assert route("今天天气不错").channel == "none"
    assert route("往左移一点").channel == "none"

    # 非字符串输入不崩
    assert route(None).channel == "none"
    assert route("").channel == "none"

    _emit("SELF_TEST PASS（路由：fast/slow/look/none 全验）")
    return 0


class AgentState:
    """跨轮共享状态：只装一个急停标志（锁由单进程串行保证，无需线程锁）。"""

    def __init__(self):
        self.estop = False


# ---------------------------------------------------------------------------
# 命令行外壳
# ---------------------------------------------------------------------------
def build_parser():
    p = argparse.ArgumentParser(
        description="端到端语音编排：录音 → ASR → 快/慢通道路由 → 执行",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="例：python3 speech_agent.py --record 5 --apply "
               "--asr-text \"把杯子抓起来\" --place-x 6 --place-y 20")
    p.add_argument("--self-test", action="store_true", help="只验路由，不联网不动臂")

    g0 = p.add_argument_group("音频来源（二选一）")
    g0.add_argument("--record", type=int, default=0,
                    help="先录 N 秒（0=不录；用 --wav 时忽略）")
    g0.add_argument("--wav", default=None, help="直接读已有 WAV")
    g0.add_argument("--sink-host", default="0.0.0.0", help="audio_sink 监听地址")
    g0.add_argument("--sink-port", type=int, default=9001, help="audio_sink 端口")
    g0.add_argument("--sink-out", default="/root/cap_live.wav", help="录音落盘路径")

    g1 = p.add_argument_group("ASR 后端")
    g1.add_argument("--asr-backend", choices=("mock", "whisper"), default="mock",
                    help="mock=不联网假识别；whisper=OpenAI 兼容云端（需 ASR_API_KEY）")
    g1.add_argument("--asr-text", default="",
                    help="mock 后端返回的固定句（演练用）")
    g1.add_argument("--asr-base-url", default="https://api.siliconflow.cn/v1",
                    help="ASR OpenAI 兼容 base_url")
    g1.add_argument("--asr-model", default="FunAudioLLM/SenseVoiceSmall", help="ASR 模型名")
    g1.add_argument("--asr-language", default=None, help="语言代码，如 zh；默认自动")

    g2 = p.add_argument_group("VLM 后端（慢通道用）")
    g2.add_argument("--vlm-backend", choices=("mock", "vlm"), default="mock",
                    help="mock=按类名关键词选；vlm=OpenAI 兼容多模态（需 VLM_API_KEY）")
    g2.add_argument("--vlm-model", default="Qwen/Qwen3.8-27B", help="VLM 模型名")
    g2.add_argument("--vlm-base-url", default="https://api.siliconflow.cn/v1",
                    help="OpenAI 兼容地址")

    g3 = p.add_argument_group("服务地址")
    g3.add_argument("--api", default="http://127.0.0.1:5000", help="Flask 控制台")
    g3.add_argument("--detector", default="http://127.0.0.1:8080", help="检测服务")

    g4 = p.add_argument_group("几何与动作（透传 pick_pipeline）")
    g4.add_argument("--grab-z", type=float, default=2.0, help="抓取高度 cm")
    g4.add_argument("--hover", type=float, default=4.0, help="悬停高度增量 cm")
    g4.add_argument("--place-x", type=float, default=None, help="放置点 x cm（示教值）")
    g4.add_argument("--place-y", type=float, default=None, help="放置点 y cm（示教值）")
    g4.add_argument("--person-iou", type=float, default=pp.PERSON_IOU_LIMIT)
    g4.add_argument("--jaw-max-cm", type=float, default=pp.JAW_MAX_OPEN_CM)

    g5 = p.add_argument_group("执行")
    g5.add_argument("--apply", action="store_true", help="真的下发（不加=全程 dry-run）")
    g5.add_argument("--keep-proxy", action="store_true",
                    help="保留系统代理（调云端 VLM 需要时加；默认直连本地服务）")
    g5.add_argument("--save-frame", default="", help="慢通道决策帧存为 jpg")
    g5.add_argument("--dynamic-grip", action="store_true",
                    help="合爪用电流判据当场定夹持脉宽（需 ACS712 已接好）")
    g5.add_argument("--model-timeout", type=float, default=60.0, help="ASR/VLM 超时秒数")
    g5.add_argument("--observe-timeout", type=float, default=8.0, help="观测超时秒数")
    return p


def main():
    args = build_parser().parse_args()

    if args.self_test:
        return self_test()

    if args.wav is None and not args.record:
        _emit("错误：--wav 和 --record 至少要给一个（或用 --self-test）")
        return 1

    return one_round(args, AgentState())


if __name__ == "__main__":
    sys.exit(main())
