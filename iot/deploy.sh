#!/bin/bash
# 一键部署 IoT 栈: 启动容器 + 在 eKuiper 上建流与规则
# 用法: chmod +x deploy.sh && ./deploy.sh   (在 KunPeng Pro / openEuler 上执行)
set -e
cd "$(dirname "$0")"

# 兼容 docker compose (V2 插件) 与旧 docker-compose
if docker compose version >/dev/null 2>&1; then CMD="docker compose"; else CMD="docker-compose"; fi

# 检查 curl 是否可用 (openEuler 默认可能未预装, 脚本依赖它调 eKuiper REST API)
if ! command -v curl >/dev/null 2>&1; then
  echo "❌ 本机未检测到 curl，请先安装: sudo dnf install -y curl"
  exit 1
fi

echo "==> 启动容器 (emqx / ekuiper / influxdb / grafana)"
$CMD up -d

echo "==> 等待 eKuiper REST API 就绪 (最多 60s)..."
for i in $(seq 1 30); do
  if curl -s -f http://localhost:9081/ >/dev/null 2>&1; then
    echo "    eKuiper 已就绪"; break
  fi
  sleep 2
done

# 就绪守卫: 防止 eKuiper 没起来就建流导致静默失败
curl -s -f http://localhost:9081/ >/dev/null 2>&1 || {
  echo "❌ eKuiper REST API 未就绪, 请先排查: docker compose logs ekuiper"
  exit 1
}

echo "==> 先清理可能残留的流/规则 (保证可重跑)"
curl -s -X DELETE http://localhost:9081/streams/device_stream >/dev/null 2>&1 || true
curl -s -X DELETE http://localhost:9081/rules/to_influx   >/dev/null 2>&1 || true

echo "==> 创建 eKuiper 流: device_stream <- mqtt(device/+/telemetry)"
# FORMAT=\"json\" 显式声明负载为 JSON, 避免 eKuiper 默认解析方式不一致
curl -s -X POST http://localhost:9081/streams \
  -H 'Content-Type: application/json' \
  -d '{"sql":"CREATE STREAM device_stream WITH (TYPE=\"mqtt\", DATASOURCE=\"device/+/telemetry\", FORMAT=\"json\");"}'
echo

echo "==> 创建 eKuiper 规则: 选字段 -> influx2 sink -> InfluxDB"
cat > /tmp/rule.json <<'EOF'
{
  "id": "to_influx",
  "sql": "SELECT temperature, humidity, voltage, deviceId FROM device_stream",
  "actions": [
    { "log": {} },
    {
      "influx2": {
        "addr": "http://influxdb:8086",
        "token": "robotics_admin_token_2026",
        "org": "robotics",
        "bucket": "telemetry",
        "measurement": "sensor",
        "tags": "{\"device\":\"{{.deviceId}}\"}",
        "fields": ["temperature", "humidity", "voltage"]
      }
    }
  ]
}
EOF
curl -s -X POST http://localhost:9081/rules -H 'Content-Type: application/json' -d @/tmp/rule.json
echo

echo "==> 规则创建结果 (status 应为 running):"
curl -s http://localhost:9081/rules/to_influx | head -c 600
echo

echo "==> 完成! 下一步:"
echo "    1) 另开终端: pip3 install paho-mqtt && python3 simulate_device.py"
echo "    2) 浏览器打开 http://<本机IP>:3000  (Grafana, admin / robotics123)"
echo "    3) 打开即有 InfluxDB 数据源; 建面板用 README 里的 Flux 查询即可看到曲线"
