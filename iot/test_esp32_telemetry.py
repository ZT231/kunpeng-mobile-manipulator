#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_esp32_telemetry.py —— ESP32 遥测草案的本机考卷（unittest，零第三方、零联网）。

覆盖三块：
  ① esp32_telemetry.py 的纯函数（字段契约 / 脏值 / 心跳超时 / 告警构造）；
  ② esp32_rules.json 的静态结构（JSON 合法 / 字段引用 / 主题隔离 / 凭据占位符）；
  ③ 子进程回归（--self-test、ASCII stdout、--dry-run --once、脏 --values 不崩）。

🔴 只证明「本机逻辑对」，不证明「已部署 / 已上板 / 已接入 ESP32」。
"""

import json
import os
import re
import subprocess
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import esp32_telemetry as m  # noqa: E402

RULES_PATH = os.path.join(HERE, "esp32_rules.json")
MODULE_PATH = os.path.join(HERE, "esp32_telemetry.py")

# SQL 里允许出现的字段：契约四字段 + 告警消息字段 + 设备标识
ALLOWED_SQL_FIELDS = set(m.CONTRACT_FIELDS) | {
    "deviceId", "ts", "kind", "level", "detail",
}


def _walk(obj):
    """递归遍历 dict/list，产出 (key, value) 对，用来查 token / dataTemplate。"""
    if isinstance(obj, dict):
        for k, v in obj.items():
            yield k, v
            for pair in _walk(v):
                yield pair
    elif isinstance(obj, list):
        for item in obj:
            for pair in _walk(item):
                yield pair


class TestPayload(unittest.TestCase):
    def test_contract_fields_and_topic(self):
        self.assertEqual(m.CONTRACT_FIELDS,
                         ("rssi", "heap_free", "wake_cnt", "cmd_ok"))
        self.assertEqual(m.TOPIC, "device/esp32_01/telemetry")
        self.assertEqual(m.ALERT_TOPIC, "alerts/esp32_01")
        self.assertEqual(m.TELEMETRY_MEASUREMENT, "esp32_telemetry")
        self.assertEqual(m.ALERT_MEASUREMENT, "alerts")

    def test_thresholds_from_roadmap(self):
        # 阈值来自 docs/核心化路线图-ESP32与IoT.md 第 59/72 行，别悄悄改
        self.assertEqual(m.RSSI_LOW_DBM, -70)
        self.assertEqual(m.HEARTBEAT_TIMEOUT_SEC, 10.0)

    def test_build_payload_normal(self):
        p = m.build_payload(-65.0, 120000, 7, 5, ts=1700000000)
        self.assertEqual(list(p.keys()),
                         ["deviceId", "ts"] + list(m.CONTRACT_FIELDS))
        self.assertEqual(p["deviceId"], "esp32_01")
        self.assertEqual(p["ts"], 1700000000)
        self.assertEqual(p["rssi"], -65.0)
        self.assertEqual(p["heap_free"], 120000)
        self.assertEqual(p["wake_cnt"], 7)
        self.assertEqual(p["cmd_ok"], 5)
        self.assertTrue(m.is_publishable(p))

    def test_dirty_values_become_none_and_block_publish(self):
        for bad in (None, "abc", True, False, float("nan"), float("inf")):
            q = m.build_payload(bad, 120000, 7, 5)
            self.assertIsNone(q["rssi"], "脏 rssi 应被洗成 None: %r" % (bad,))
            self.assertFalse(m.is_publishable(q))

    def test_out_of_range_values(self):
        self.assertIsNone(m.build_payload(-200, 120000, 7, 5)["rssi"], "rssi 超下限")
        self.assertIsNone(m.build_payload(10, 120000, 7, 5)["rssi"], "rssi 不能为正")
        self.assertIsNone(m.build_payload(-65, -1, 7, 5)["heap_free"], "堆不能为负")
        self.assertIsNone(m.build_payload(-65, 120000, 3.5, 5)["wake_cnt"], "计数须为整数")
        self.assertIsNone(m.build_payload(-65, 120000, 7, -1)["cmd_ok"], "成功数不能为负")

    def test_publishable_rejects_empty_and_bool(self):
        self.assertFalse(m.is_publishable({}))
        self.assertFalse(m.is_publishable(None))
        p = m.build_payload(-65, 120000, 7, 5)
        p["rssi"] = True
        self.assertFalse(m.is_publishable(p), "bool 是 int 子类，必须显式排除")

    def test_parse_values(self):
        self.assertEqual(m.parse_values("-82,120000,7,5"),
                         (-82.0, 120000.0, 7, 5))
        with self.assertRaises(ValueError):
            m.parse_values("-82,120000,7")


class TestHeartbeat(unittest.TestCase):
    def test_boundaries(self):
        self.assertFalse(m.is_heartbeat_stale(1000.0, 1009.9), "9.9s 未超时")
        self.assertTrue(m.is_heartbeat_stale(1000.0, 1010.1), "10.1s 超时")
        self.assertFalse(m.is_heartbeat_stale(1000.0, 1010.0), "边界严格大于")

    def test_clock_rollback_and_dirty_ts(self):
        self.assertFalse(m.is_heartbeat_stale(1000.0, 999.0), "时钟回拨不误报")
        self.assertFalse(m.is_heartbeat_stale(None, 1010.1), "None 不误报")
        self.assertFalse(m.is_heartbeat_stale(True, 1010.1), "bool 不误报")

    def test_custom_timeout(self):
        self.assertTrue(m.is_heartbeat_stale(1000.0, 1002.1, timeout=2.0))


class TestAlert(unittest.TestCase):
    def test_build_alert(self):
        a = m.build_alert("rssi_low", "rssi=-82", ts=1700000001)
        self.assertEqual(a["kind"], "rssi_low")
        self.assertEqual(a["level"], "warn")
        self.assertEqual(a["deviceId"], "esp32_01")
        self.assertEqual(a["ts"], 1700000001)

    def test_heartbeat_alert_is_critical(self):
        a = m.build_alert("heartbeat_timeout", "last=1000", ts=1700000002)
        self.assertEqual(a["level"], "critical")

    def test_unknown_kind_raises(self):
        with self.assertRaises(ValueError):
            m.build_alert("picked_up_egg", "x")


class TestRulesJson(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open(RULES_PATH, "r", encoding="utf-8") as f:
            cls.text = f.read()
        cls.doc = json.loads(cls.text)

    def test_status_is_draft(self):
        self.assertEqual(self.doc["meta"]["status"], "draft",
                         "草案必须标 draft，不能写成已上线")

    def test_streams_and_rules_count(self):
        self.assertEqual(len(self.doc["streams"]), 2)
        self.assertEqual(len(self.doc["rules"]), 3)
        ids = [r["id"] for r in self.doc["rules"]]
        self.assertEqual(len(ids), len(set(ids)), "规则 id 不能重复")

    def test_stream_datasource_is_exact_topic(self):
        # 用精确主题，避免 device/+/telemetry 把 ESP32 消息灌进机械臂的 to_influx 规则
        sqls = " ".join(s["sql"] for s in self.doc["streams"])
        self.assertIn('DATASOURCE="device/esp32_01/telemetry"', sqls)
        self.assertIn('DATASOURCE="alerts/+"', sqls)
        self.assertIn("CREATE STREAM esp32_stream ()", sqls)
        self.assertIn("CREATE STREAM alert_stream ()", sqls)

    def test_sql_fields_all_in_contract(self):
        for rule in self.doc["rules"]:
            sql = rule["sql"]
            match = re.search(r"SELECT\s+(.*?)\s+FROM\s+", sql, re.I | re.S)
            self.assertIsNotNone(match, "SQL 缺 SELECT..FROM: %s" % sql)
            cols = [c.strip() for c in match.group(1).split(",")]
            for col in cols:
                self.assertIn(col, ALLOWED_SQL_FIELDS,
                              "规则 %s 引用了契约外字段 %s" % (rule["id"], col))

    def test_rssi_threshold_present(self):
        low = [r for r in self.doc["rules"] if r["id"] == "esp32_rssi_low"]
        self.assertEqual(len(low), 1)
        self.assertIn("rssi < -70", low[0]["sql"])

    def test_measurements_isolated(self):
        measurements = []
        for _k, v in _walk(self.doc):
            if isinstance(v, dict) and "measurement" in v:
                measurements.append(v["measurement"])
        self.assertIn("esp32_telemetry", measurements)
        self.assertIn("alerts", measurements)
        self.assertNotIn("arm_telemetry", measurements,
                         "不能混进机械臂 measurement（InfluxDB field 类型冲突）")

    def test_tokens_are_placeholders_only(self):
        tokens = [v for k, v in _walk(self.doc) if k == "token"]
        self.assertTrue(tokens, "应至少有一个 influx2 token 字段")
        for t in tokens:
            self.assertTrue(t.startswith("${") and t.endswith("}"),
                            "token 必须是 ${...} 占位符，仓库里不许出现真实值: %r" % t)

    def test_no_password_key_and_no_data_template(self):
        keys = [k for k, _v in _walk(self.doc)]
        self.assertNotIn("password", keys)
        self.assertNotIn("dataTemplate", keys,
                         "eKuiper 2.1.0 模板上下文是切片，取字段会报错（deploy.sh 第 80-83 行）")

    def test_every_rule_has_actions(self):
        for rule in self.doc["rules"]:
            self.assertIsInstance(rule["actions"], list)
            self.assertTrue(rule["actions"], "规则 %s 没有 action" % rule["id"])


class TestCli(unittest.TestCase):
    def _run(self, args, ascii_stdout=False):
        env = os.environ.copy()
        if ascii_stdout:
            env["PYTHONIOENCODING"] = "ascii"
        else:
            env["PYTHONIOENCODING"] = "utf-8"
        return subprocess.run(
            [sys.executable, "-B", MODULE_PATH] + args,
            cwd=HERE, env=env, capture_output=True,
            text=True, encoding="utf-8", errors="replace", timeout=60,
        )

    def test_self_test_plain(self):
        p = self._run(["--self-test"])
        self.assertEqual(p.returncode, 0, p.stdout + p.stderr)
        self.assertIn("SELF_TEST PASS", p.stdout)

    def test_self_test_ascii_stdout(self):
        p = self._run(["--self-test"], ascii_stdout=True)
        self.assertEqual(p.returncode, 0, p.stdout + p.stderr)
        self.assertIn("SELF_TEST PASS", p.stdout)

    def test_dry_run_once_ascii(self):
        p = self._run(["--dry-run", "--once"], ascii_stdout=True)
        self.assertEqual(p.returncode, 0, p.stdout + p.stderr)
        self.assertIn("dry-run", p.stdout)
        self.assertIn("rssi", p.stdout)
        self.assertNotIn("已连", p.stdout, "dry-run 绝不连 MQTT")

    def test_dry_run_rssi_alert_ascii(self):
        p = self._run(["--dry-run", "--once", "--values=-82,120000,7,5"],
                      ascii_stdout=True)
        self.assertEqual(p.returncode, 0, p.stdout + p.stderr)
        self.assertIn("rssi_low", p.stdout)

    def test_bad_values_not_crash(self):
        p = self._run(["--dry-run", "--once", "--values", "oops,1,2,3"])
        self.assertEqual(p.returncode, 2, p.stdout + p.stderr)


if __name__ == "__main__":
    unittest.main(verbosity=2)
