#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
esp32_telemetry.py —— 📋 提前草案：ESP32 节点遥测「字段契约 + 板侧适配层」

大白话：这份代码不装在 ESP32 上，它跑在鲲鹏板（或 PC）里，只做三件事：
        ① 说清楚 ESP32 该往 MQTT 发哪几个数（字段契约）；
        ② 收到数之后先洗干净，脏数据不许发上云；
        ③ 判「信号太弱」和「心跳超时」，生成一条告警消息。
规范定义：MQTT 3.1.1/5.0 是基于 TCP 的发布-订阅协议，主题用 / 分层；
        InfluxDB 2.x 的 field value 只允许 float / integer / unsignedInteger /
        string / boolean 五种标量，且「measurement + field」的类型首次写入即锁定。

🔴 定位（先读这段，别把草案当成品）：
  - 按 11-14 用户重排，这项（阶段 5~6）排在「ESP32 真发帧」之后，本轮属于**提前草案**。
  - **不动 `esp32/` 固件源码**（明天要照跑烧录、当前副本已 build 通过）；
    **不动 `docker-compose.yml`**（敏感保护拦截，且板内四容器已实机）；
    **不动 `deploy.sh` 里已实机的 `to_influx` 规则**；不改 `.ioc`；不装任何包。
  - **ESP32 固件侧真正的 MQTT client 尚未写**：真节点还没烧录、没连 Wi-Fi，
    所以本模块只覆盖「契约层 + 板侧适配层」，不是端到端已接入。
  - 默认 dry-run；只有显式 `--publish` 才连 MQTT。`--self-test` 零依赖、零联网。

字段与阈值全部来自 `docs/核心化路线图-ESP32与IoT.md`，不自己发明：
  第 71 行：主题 `device/esp32_01/telemetry`，字段 `rssi / heap_free / wake_cnt / cmd_ok`
  第 59 行：告警 `rssi < -70 dBm`、`心跳超时 > 10s`
  第 72 行：告警写 `alerts` measurement

用法：
  python3 esp32_telemetry.py --self-test                 # 纯逻辑自测，零依赖
  python3 esp32_telemetry.py --dry-run --once             # 只打印，不连 MQTT
  python3 esp32_telemetry.py --dry-run --values=-82,120000,7,5 --once
  （🔴 --values 以负号开头时必须写 --values=... 等号形式：argparse 会把
     "-82,120000,7,5" 当成新选项，报 "expected one argument"）
  python3 esp32_telemetry.py --publish --broker 127.0.0.1 --port 1883
"""

import argparse
import json
import sys
import time

# ---- 常量集中放这一块：换配置只改这里 ----
DEVICE_ID = "esp32_01"
TOPIC = "device/" + DEVICE_ID + "/telemetry"
ALERT_TOPIC = "alerts/" + DEVICE_ID

# 字段契约：顺序固定，测试会逐项核对，别随手改名
CONTRACT_FIELDS = ("rssi", "heap_free", "wake_cnt", "cmd_ok")

RSSI_LOW_DBM = -70            # 路线图第 59/72 行：低于 -70dBm 告警
RSSI_MIN_DBM = -120           # 物理下限，比这更低基本是脏数据
RSSI_MAX_DBM = 0              # dBm 不会是正数
HEARTBEAT_TIMEOUT_SEC = 10.0  # 路线图第 59 行：心跳超时 > 10s

TELEMETRY_MEASUREMENT = "esp32_telemetry"  # 独立 measurement，别混进 arm_telemetry
ALERT_MEASUREMENT = "alerts"               # 路线图第 72 行

# 演示用的示例值。🔴 这不是真实 ESP32 遥测，只是给 --dry-run 有东西可打印
DEMO_VALUES = (-65.0, 120000.0, 7, 5)

ALERT_KINDS = ("rssi_low", "heartbeat_timeout")


def _configure_stdout():
    """把 stdout 尽量设成 UTF-8；板端 LANG=C / ASCII stdout 下也不许崩。

    大白话：终端只认英文时，直接 print 中文会抛 UnicodeEncodeError，
            连后面的逻辑都跑不到。11-14 在 voice_gateway.py 上真踩过这个坑。
    规范定义：Python 3.7+ 的 sys.stdout.reconfigure() 可改文本层编码；
            errors="backslashreplace" 把无法编码的字符转成 \\uXXXX 转义。
    """
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="backslashreplace")
    except (AttributeError, ValueError, OSError):
        pass


def _emit(text):
    """所有输出都走这里，保证 locale 无关、且立即刷出来（方便 tail 日志）。"""
    data = str(text)
    try:
        print(data, flush=True)
    except UnicodeEncodeError:
        safe = data.encode("ascii", "backslashreplace").decode("ascii")
        print(safe, flush=True)


def _clean_number(value, min_value, max_value, integer_only):
    """把一个值洗成合法标量；不合法返回 None，绝不抛异常打断采集循环。

    大白话：InfluxDB 一格只装一个数，装不下 None / 字符串 / 布尔。
            发出去一个 null 不是「少几个数」，而是会把整批写库拖垮。
    规范定义：bool 是 int 的子类（isinstance(True, int) 为 True），
            必须显式排除，否则 True 会被当成 1 写进浮点 field。
    """
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    # NaN / inf 不是合法 JSON 数（RFC 8259 不允许），json.dumps 会写出非法文本
    if value != value or value in (float("inf"), float("-inf")):
        return None
    if value < min_value or value > max_value:
        return None
    if integer_only and not float(value).is_integer():
        return None
    return int(value) if integer_only else value


def build_payload(rssi, heap_free, wake_cnt, cmd_ok, ts=None):
    """把四个原始读数拼成一条遥测 payload（纯函数，方便自测）。

    为什么不直接在循环里拼：拆成纯函数后，不联网也能用假数据验证字段对不对，
    真出问题时能一眼分清是「取数错」还是「拼装错」——与 telemetry_bridge.py 同一套做法。
    """
    payload = {
        "deviceId": DEVICE_ID,
        "ts": int(time.time()) if ts is None else int(ts),
        "rssi": _clean_number(rssi, RSSI_MIN_DBM, RSSI_MAX_DBM, False),
        "heap_free": _clean_number(heap_free, 0, 2 ** 31, True),
        "wake_cnt": _clean_number(wake_cnt, 0, 2 ** 31, True),
        "cmd_ok": _clean_number(cmd_ok, 0, 2 ** 31, True),
    }
    return payload


def is_publishable(payload):
    """这条 payload 能不能发上 MQTT（纯函数）。四个契约字段必须全是真数字。

    🔴 对应 `telemetry_bridge.py` 第 11 条排障：InfluxDB 2.x 的 field 类型
    首次写入即锁定；eKuiper influx2 sink 不跳过 nil，把 JSON null 写成 string
    会撞历史 float，返回 422 partial write: field type conflict、整批写不出去。
    ⇒ 脏数据必须在发布端就挡掉，进程不崩不等于数据无害。
    """
    if not isinstance(payload, dict):
        return False
    for name in CONTRACT_FIELDS:
        value = payload.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return False
    return True


def is_heartbeat_stale(last_ts, now, timeout=HEARTBEAT_TIMEOUT_SEC):
    """心跳是不是超时了（纯函数）。

    大白话：流引擎（eKuiper）靠「收到消息」才驱动窗口，设备彻底不说话时
            一条消息都没有，SQL 窗口根本不会触发 ⇒ 「离线」不能用窗口检测。
            所以超时判定放在板侧看门狗：拿上次收到心跳的时间跟现在比。
    规范定义：单调比较 `now - last_ts > timeout`；时钟回拨（now < last_ts）
            时返回 False，避免 NTP 校时把全网设备误判成离线。
    """
    if (isinstance(last_ts, bool) or isinstance(now, bool)
            or not isinstance(last_ts, (int, float))
            or not isinstance(now, (int, float))):
        return False
    if now < last_ts:
        return False
    return (now - last_ts) > timeout


def build_alert(kind, detail, ts=None):
    """生成一条告警消息（纯函数）。kind 只允许两种，写错直接报错而不是静默发出去。"""
    if kind not in ALERT_KINDS:
        raise ValueError("未知告警类型: %s" % kind)
    return {
        "deviceId": DEVICE_ID,
        "ts": int(time.time()) if ts is None else int(ts),
        "kind": kind,
        "detail": str(detail),
        "level": "critical" if kind == "heartbeat_timeout" else "warn",
    }


def parse_values(text):
    """把 "-65,120000,7,5" 解析成 (rssi, heap_free, wake_cnt, cmd_ok)。"""
    parts = text.split(",")
    if len(parts) != 4:
        raise ValueError("--values 需要 4 个逗号分隔的值，实际 %d 个" % len(parts))
    return (float(parts[0]), float(parts[1]),
            int(parts[2]), int(parts[3]))


def make_client(broker, port):
    """连 MQTT Broker。paho 延迟导入，好让 --self-test / --dry-run 零依赖可跑。"""
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        print("[fail] 缺 paho-mqtt，板上跑：pip3 install paho-mqtt", file=sys.stderr)
        sys.exit(2)
    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                             client_id="esp32_telemetry_" + DEVICE_ID)
    except AttributeError:
        client = mqtt.Client(client_id="esp32_telemetry_" + DEVICE_ID)
    client.connect(broker, port, 60)
    client.loop_start()
    return client


def self_test():
    """纯逻辑自测：喂假数据，逐个字段检查，不联网、不依赖 paho。"""
    p = build_payload(-65.0, 120000, 7, 5, ts=1700000000)
    assert list(p.keys()) == ["deviceId", "ts"] + list(CONTRACT_FIELDS), "字段顺序/契约"
    assert p["deviceId"] == DEVICE_ID and p["ts"] == 1700000000, "deviceId / ts"
    assert p["rssi"] == -65.0 and p["heap_free"] == 120000, "rssi / heap_free"
    assert p["wake_cnt"] == 7 and p["cmd_ok"] == 5, "wake_cnt / cmd_ok"
    assert is_publishable(p) is True, "正常四字段应可发布"
    assert TOPIC == "device/esp32_01/telemetry", "主题必须对齐路线图第 71 行"

    # 降级 / 畸形：脏值一律变 None，且整条不许发
    for bad in (None, "abc", True, float("nan")):
        q = build_payload(bad, 120000, 7, 5)
        assert q["rssi"] is None and is_publishable(q) is False, "脏 rssi 必须挡掉"
    assert build_payload(-200, 120000, 7, 5)["rssi"] is None, "rssi 超物理下限"
    assert build_payload(-65, -1, 7, 5)["heap_free"] is None, "剩余堆不能为负"
    assert build_payload(-65, 120000, 3.5, 5)["wake_cnt"] is None, "计数必须是整数"
    assert is_publishable({}) is False, "空 dict 不许发"

    # 心跳超时：9.9s 不报、10.1s 报、正好 10s 不报、时钟回拨不误报
    assert is_heartbeat_stale(1000.0, 1009.9) is False, "9.9s 未超时"
    assert is_heartbeat_stale(1000.0, 1010.1) is True, "10.1s 超时"
    assert is_heartbeat_stale(1000.0, 1010.0) is False, "边界：严格大于才算超时"
    assert is_heartbeat_stale(1000.0, 999.0) is False, "时钟回拨不误报"
    assert is_heartbeat_stale(None, 1010.1) is False, "脏时间戳不误报"

    # 告警消息
    a1 = build_alert("rssi_low", "rssi=-82 < -70", ts=1700000001)
    assert a1["kind"] == "rssi_low" and a1["level"] == "warn", "rssi 告警"
    a2 = build_alert("heartbeat_timeout", "last=1000 now=1011", ts=1700000002)
    assert a2["level"] == "critical", "心跳超时是 critical"
    try:
        build_alert("picked_up_egg", "x")
        raise AssertionError("未知告警类型必须报错")
    except ValueError:
        pass

    # 契约与阈值不能被悄悄改掉
    assert CONTRACT_FIELDS == ("rssi", "heap_free", "wake_cnt", "cmd_ok"), "字段契约"
    assert RSSI_LOW_DBM == -70 and HEARTBEAT_TIMEOUT_SEC == 10.0, "告警阈值"
    assert TELEMETRY_MEASUREMENT != "arm_telemetry", "measurement 必须隔离"

    print("SELF_TEST PASS")
    print(json.dumps(p, ensure_ascii=False, indent=2))
    return 0


def main():
    _configure_stdout()
    parser = argparse.ArgumentParser(description="ESP32 遥测契约 + 板侧适配层（草案）")
    parser.add_argument("--self-test", action="store_true", help="纯逻辑自测，不联网")
    parser.add_argument("--publish", action="store_true",
                        help="真的连 MQTT 发消息（默认 dry-run，只打印）")
    parser.add_argument("--dry-run", action="store_true",
                        help="显式声明只打印不发；同时给 --publish 时以 dry-run 为准（更安全）")
    parser.add_argument("--once", action="store_true", help="只跑一轮就退出")
    parser.add_argument("--values", default=None,
                        help='四个逗号分隔的示例值，如 "-65,120000,7,5"')
    parser.add_argument("--interval", type=float, default=5.0, help="采集周期秒")
    parser.add_argument("--broker", default="127.0.0.1", help="MQTT Broker 地址")
    parser.add_argument("--port", type=int, default=1883, help="MQTT 端口")
    args = parser.parse_args()

    if args.self_test:
        return self_test()

    try:
        values = parse_values(args.values) if args.values else DEMO_VALUES
    except ValueError as e:
        print("[fail] %s" % e, file=sys.stderr)
        return 2

    client = None
    # dry-run 优先：同时给了 --publish 也绝不连 MQTT，避免误发
    if args.publish and not args.dry_run:
        client = make_client(args.broker, args.port)
        _emit("[esp32] 已连 %s:%d，主题 %s" % (args.broker, args.port, TOPIC))
    else:
        _emit("[esp32] dry-run：只打印，不连 MQTT（示例值不是真实遥测）")

    try:
        while True:
            rssi, heap, wake, ok = values
            payload = build_payload(rssi, heap, wake, ok)
            line = json.dumps(payload, ensure_ascii=False)

            if not is_publishable(payload):
                _emit("[warn] 四字段不完整，跳过本次发布（避免 InfluxDB field 类型冲突）")
            elif client is not None:
                client.publish(TOPIC, line, qos=1)
                _emit("[esp32] -> " + line)
            else:
                _emit("[dry-run] -> " + line)

            # RSSI 告警：这是 eKuiper 规则 esp32_rssi_low 的板侧等价逻辑，
            # 提前草案里先在本机验证「阈值判断」对不对，规则上线后由流引擎接管。
            if is_publishable(payload) and payload["rssi"] < RSSI_LOW_DBM:
                alert = build_alert("rssi_low", "rssi=%s < %s" %
                                    (payload["rssi"], RSSI_LOW_DBM))
                alert_line = json.dumps(alert, ensure_ascii=False)
                if client is not None:
                    client.publish(ALERT_TOPIC, alert_line, qos=1)
                _emit("[alert] -> " + alert_line)

            if args.once:
                break
            time.sleep(args.interval)
    except KeyboardInterrupt:
        _emit("\n[esp32] 已停止")
    finally:
        if client is not None:
            client.loop_stop()
            client.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
