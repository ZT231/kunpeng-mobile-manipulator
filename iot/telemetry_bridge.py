#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
telemetry_bridge.py —— 阶段 1：把 5000 控制台的真实数据经 MQTT 上报

大白话：这个脚本是个「搬运工」。每隔几秒去问本机 5000 网页服务
        「机械臂现在什么状态、摄像头看到了几个东西」，把答案打包成一条
        JSON 消息贴到 MQTT 主题上，顶替掉原来 simulate_device.py 造的假温湿度。
规范定义：MQTT 3.1.1 / 5.0 是基于 TCP 的发布-订阅（pub/sub）轻量消息协议，
         主题用 / 分层，+ 为单层通配。本项目沿用 device/<id>/telemetry，
         与 iot/deploy.sh 里 eKuiper 流 DATASOURCE="device/+/telemetry" 匹配。

为什么是独立脚本，而不是去改 web_control.py：
  5000 服务是已经实机验证过的主链路。改它就要 scp + 重启 Flask，
  而重启会把 _IK_MAP 标定参数清回默认值、打断正在跑的实机现场。
  独立脚本只「读」已有的 HTTP 接口，一行主链路代码都不碰，Ctrl+C 停掉就回滚干净。

代价（诚实交代）：/api/state 是 POST，每次调用会真的往串口发一条 STATE 命令。
  所以默认 5 秒才取一次；相对页面已有的 1 秒轮询，只多约 20% 的串口帧，可忽略。
  串口没连时接口返回 ok=False，这里会容错跳过，不崩。

用法：
  python3 telemetry_bridge.py --self-test           # 纯逻辑自测，零依赖、不联网
  python3 telemetry_bridge.py --dry-run --once      # 只取数并打印，不连 MQTT（板上先验）
  python3 telemetry_bridge.py --dry-run             # 每 5 秒打印一次
  python3 telemetry_bridge.py                      # 正式：连 MQTT 上报
"""

import argparse
import json
import sys
import time
import urllib.request
import urllib.error

# ---- 常量集中放这一块，改配置只改这里 ----
BASE_URL = "http://127.0.0.1:5000"
BROKER = "127.0.0.1"
PORT = 1883
DEVICE_ID = "kunpeng_01"
TOPIC = "device/" + DEVICE_ID + "/telemetry"
INTERVAL = 5.0
TIMEOUT = 1.0


def fetch_json(path, method="GET"):
    """取一个 5000 的 HTTP 接口，返回解析后的字典；失败返回 None，不抛异常。

    大白话：就像浏览器敲个地址拿回一串 JSON，这里用 Python 干同样的事。
    规范定义：urllib.request.urlopen 发 HTTP 请求，json.loads 按 RFC 8259
             把响应体解析成 Python 字典。
    """
    url = BASE_URL + path
    req = urllib.request.Request(url, method=method)
    # POST /api/state 不需要请求体，但显式给个空 body 才是合法 POST
    data = b"" if method == "POST" else None
    try:
        with urllib.request.urlopen(req, data=data, timeout=TIMEOUT) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except (urllib.error.URLError, OSError, ValueError) as e:
        print("[warn] 取 %s 失败：%s" % (path, e), file=sys.stderr)
        return None


def build_payload(state, det, cfg):
    """把三份原始响应拼成一条遥测 payload（纯函数，方便自测）。

    为什么不直接在循环里拼：拆成纯函数后，不联网也能用假数据验证字段对不对，
    这正是 --self-test 干的事；真出问题时也能一眼分清是「取数错」还是「拼装错」。
    """
    payload = {
        "deviceId": DEVICE_ID,
        "ts": int(time.time()),
        "pulses": None,
        "frame_id": None,
        "det_count": None,
        "mode": None,
        "baud": None,
        "camera_on": None,
    }

    # 六路脉宽来自 /api/state：它返回 {"ok":true,"pulses":[p0..p5]}
    if isinstance(state, dict) and state.get("ok"):
        pulses = state.get("pulses")
        if isinstance(pulses, list) and len(pulses) == 6:
            payload["pulses"] = pulses
        else:
            print("[warn] STATE 里 pulses 不是 6 路，跳过", file=sys.stderr)

    # 帧号和检测数来自 /api/detections：外层是 {"ok":true,"data":{...}}，
    # data 里的字段名与页面 showDetections(d) 用的完全一致（d.frame_id / d.detections）。
    if isinstance(det, dict) and det.get("ok"):
        data = det.get("data") or {}
        if isinstance(data, dict):
            payload["frame_id"] = data.get("frame_id")
            dets = data.get("detections")
            payload["det_count"] = len(dets) if isinstance(dets, list) else None

    # /api/config 是纯读。mode 直接透传，别硬编码 ——
    # 实测这块板返回 "serial"，早期草稿写过 "uart"，写死就会和真实值对不上。
    if isinstance(cfg, dict) and cfg.get("ok"):
        payload["mode"] = cfg.get("mode")
        payload["baud"] = cfg.get("baud")
        payload["camera_on"] = cfg.get("camera_on")

    return payload


def collect_once():
    """取一轮数据。三个接口互相独立，任意一个挂了都不影响另外两个。"""
    cfg = fetch_json("/api/config", "GET")
    det = fetch_json("/api/detections", "GET")
    # 🔴 这个 POST 会真的往串口发一条 STATE，是全脚本唯一有副作用的地方
    state = fetch_json("/api/state", "POST")
    return build_payload(state, det, cfg)


def make_client(broker, port):
    """连 MQTT Broker。paho 延迟导入，好让 --self-test / --dry-run 零依赖可跑。"""
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        print("[fail] 缺 paho-mqtt，板上跑：pip3 install paho-mqtt", file=sys.stderr)
        sys.exit(2)
    try:
        # paho-mqtt 2.x 要求显式指定回调 API 版本；板上装的是 2.2.0
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                             client_id="bridge_" + DEVICE_ID)
    except AttributeError:
        # 兼容 1.x 老版本（没有 CallbackAPIVersion 这个枚举）
        client = mqtt.Client(client_id="bridge_" + DEVICE_ID)
    client.connect(broker, port, 60)
    client.loop_start()
    return client


def self_test():
    """纯逻辑自测：喂假数据，逐个字段检查 build_payload，不联网、不依赖 paho。"""
    state = {"ok": True, "pulses": [1580, 1900, 2400, 700, 1450, 1500]}
    det = {"ok": True, "data": {"frame_id": 1238,
                                "detections": [{"label": "book"}, {"label": "banana"}]}}
    cfg = {"ok": True, "mode": "serial", "baud": 115200, "camera_on": False}
    p = build_payload(state, det, cfg)
    assert p["deviceId"] == DEVICE_ID, "deviceId"
    assert p["pulses"] == [1580, 1900, 2400, 700, 1450, 1500], "pulses"
    assert p["frame_id"] == 1238, "frame_id"
    assert p["det_count"] == 2, "det_count"
    assert p["mode"] == "serial", "mode"
    assert p["baud"] == 115200, "baud"
    assert p["camera_on"] is False, "camera_on"
    assert isinstance(p["ts"], int), "ts"
    assert TOPIC.startswith("device/") and TOPIC.endswith("/telemetry"), "topic 格式"

    # 降级用例：串口没连（ok=False）或整个服务没起时，字段应为 None 且不能崩
    p2 = build_payload({"ok": False, "error": "未连接"}, None, None)
    assert p2["pulses"] is None and p2["mode"] is None, "降级容错"

    # 畸形用例：pulses 不是 6 路时跳过，绝不把错数据发上云
    p3 = build_payload({"ok": True, "pulses": [1, 2]}, None, None)
    assert p3["pulses"] is None, "畸形 pulses"

    print("SELF_TEST PASS")
    print(json.dumps(p, ensure_ascii=False, indent=2))
    return 0


def main():
    parser = argparse.ArgumentParser(description="5000 控制台真实遥测 → MQTT")
    parser.add_argument("--dry-run", action="store_true", help="只打印 payload，不连 MQTT")
    parser.add_argument("--once", action="store_true", help="只跑一轮就退出")
    parser.add_argument("--self-test", action="store_true", help="纯逻辑自测，不联网")
    parser.add_argument("--interval", type=float, default=INTERVAL, help="采集周期秒，默认 5")
    parser.add_argument("--broker", default=BROKER, help="MQTT Broker 地址，默认 127.0.0.1")
    parser.add_argument("--port", type=int, default=PORT, help="MQTT 端口，默认 1883")
    parser.add_argument("--topic", default=TOPIC, help="上报主题")
    args = parser.parse_args()

    if args.self_test:
        return self_test()

    client = None
    if not args.dry_run:
        client = make_client(args.broker, args.port)
        print("[bridge] 已连 %s:%d，主题 %s（Ctrl+C 退出）" %
              (args.broker, args.port, args.topic))
    else:
        print("[bridge] dry-run：只打印，不连 MQTT（Ctrl+C 退出）")

    try:
        while True:
            payload = collect_once()
            line = json.dumps(payload, ensure_ascii=False)
            if client is not None:
                client.publish(args.topic, line, qos=1)
                print("[bridge] -> " + line)
            else:
                print("[dry-run] -> " + line)
            if args.once:
                break
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\n[bridge] 已停止")
    finally:
        if client is not None:
            client.loop_stop()
            client.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
