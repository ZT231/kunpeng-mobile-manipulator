#!/bin/bash
# 一键部署 IoT 栈: 启动容器 + 在 eKuiper 上建流与规则
# 用法: chmod +x deploy.sh && ./deploy.sh   (在 KunPeng Pro / openEuler 上执行)
#
# 前置（阶段 1.1 遗留，必做一次）:
#   1.1 用单条 docker run 手动起的 emqx，container_name 与 compose 定义冲突。
#   先 docker rm -f emqx，再由本脚本的 docker-compose up -d 统一托管四个容器。
#   emqx 归 compose 管之后重跑本脚本，别再删（删了会让已连上的 bridge 断一次连接）。
set -e
cd "$(dirname "$0")"

# 兼容 docker compose (V2 插件) 与旧 docker-compose
if docker compose version >/dev/null 2>&1; then CMD="docker compose"; else CMD="docker-compose"; fi

if ! command -v curl >/dev/null 2>&1; then
  echo "本机未检测到 curl，请先安装: sudo dnf install -y curl"
  exit 1
fi

echo "==> 前置提醒: 1.1 手动起的 emqx 若还在，先 docker rm -f emqx (见 README 第 1 节)"
echo "==> 启动容器 (emqx / ekuiper / influxdb / grafana)"
$CMD up -d

# 🔴 firewalld nftables 兜底 REJECT（见 docs/排障记录-IoT.md 第 8 条）：
# openEuler 的 firewalld 走 nftables 后端，docker 网桥 br-* 不在任何 zone 里，
# 容器发出的三层包落到 target: default 的兜底 REJECT，表现成 "no route to host"；
# eKuiper sink 探活一失败，规则就 stopped by error。
# 15:03 实证 runtime 绑定会在「重启 / 网络重建」后丢失 ⇒ 每次部署都动态取当前
# 网桥名重绑一次（幂等：已绑定时 firewall-cmd 给 ZONE_ALREADY_SET 警告但仍 success）。
# 不加 --permanent、不 --reload：避免冲掉 Docker 自建的 iptables 链、不重启容器。
echo "==> 把 docker 网桥纳入 firewalld trusted zone (容器间互通, 可重跑)"
BR=$(ip -4 addr show | awk '/^[0-9]+: br-/{gsub(/:/,"",$2); print $2; exit}')
if [ -n "$BR" ] && command -v firewall-cmd >/dev/null 2>&1; then
  echo "    网桥: $BR"
  firewall-cmd --zone=trusted --add-interface="$BR" >/dev/null 2>&1 \
    || echo "    [warn] trusted zone 绑定失败, 请手动检查 firewall-cmd"
else
  echo "    [warn] 未找到 docker 网桥或 firewall-cmd, 跳过 (非 openEuler 环境可忽略)"
fi

echo "==> 等待 eKuiper REST API 就绪 (最多 60s)..."
for i in $(seq 1 30); do
  if curl -s -f http://localhost:9081/ >/dev/null 2>&1; then
    echo "    eKuiper 已就绪"; break
  fi
  sleep 2
done

curl -s -f http://localhost:9081/ >/dev/null 2>&1 || {
  echo "eKuiper REST API 未就绪, 请先排查: docker-compose logs ekuiper"
  exit 1
}

echo "==> 先清理可能残留的规则/流 (保证可重跑; 规则引用流, 必须先删规则)"
curl -s -X DELETE http://localhost:9081/rules/to_influx   >/dev/null 2>&1 || true
curl -s -X DELETE http://localhost:9081/streams/device_stream >/dev/null 2>&1 || true

echo "==> 创建 eKuiper 流: device_stream <- mqtt(device/+/telemetry)"
# eKuiper 要求流名后跟列定义括号；不做字段声明(schemaless)时写空括号 ()，
# 否则报 "found WITH, expected lparen after stream name" (error 3000)。
STREAM_RESP=$(curl -s -X POST http://localhost:9081/streams \
  -H 'Content-Type: application/json' \
  -d '{"sql":"CREATE STREAM device_stream () WITH (TYPE=\"mqtt\", DATASOURCE=\"device/+/telemetry\", FORMAT=\"json\");"}') \
  || { echo "请求 eKuiper /streams 失败, 已停止"; exit 1; }
echo "$STREAM_RESP"
# REST 失败时返回的是 HTTP 200 + 带 "error" 的 JSON, curl 退出码仍是 0,
# set -e 抓不到 ⇒ 必须自己查响应体, 否则脚本会一路打印「完成!」却什么都没建成。
if echo "$STREAM_RESP" | grep -q '"error"'; then
  echo "建流失败, 已停止 (见上面的 error 码)"
  exit 1
fi
echo "    [OK] 流 device_stream 已创建"

echo "==> 创建 eKuiper 规则: 真实遥测字段 -> influx2 sink -> InfluxDB"
# 字段来源 = telemetry_bridge.py 的 build_payload():
#   deviceId / ts / pulses[6] / j1~j6 / frame_id / det_count / mode / baud / camera_on
# InfluxDB 2.x 的 field 只收标量，数组 pulses 不能直接写；bridge 因此额外拆出
# j1~j6 六个标量，规则只 SELECT 这些标量。ts 不选（避开 eKuiper 事件时间字段），
# InfluxDB 用写入时刻作时间戳。measurement 由假数据时代的 sensor 改成 arm_telemetry。
# 🔴 tags 必须是「静态字符串值」的 map 对象，不能写 {{.deviceId}}：
#   eKuiper 2.1.0 求 dataTemplate 时喂给模板的是 []map[string]interface{} 切片，
#   取 .deviceId 会报 can't evaluate field deviceId in type []map[...]（第 9 条排障），
#   每条记录都在 transform 阶段被拒。deviceId 本就是发布端常量，动态模板无收益。
cat > /tmp/rule.json <<'EOF'
{
  "id": "to_influx",
  "sql": "SELECT j1, j2, j3, j4, j5, j6, frame_id, det_count, mode, baud, camera_on, deviceId FROM device_stream",
  "actions": [
    { "log": {} },
    {
      "influx2": {
        "addr": "http://influxdb:8086",
        "token": "robotics_admin_token_2026",
        "org": "robotics",
        "bucket": "telemetry",
        "measurement": "arm_telemetry",
        "tags": {"device": "kunpeng_01"},
        "fields": ["j1", "j2", "j3", "j4", "j5", "j6", "frame_id", "det_count", "mode", "baud", "camera_on"]
      }
    }
  ]
}
EOF
RULE_RESP=$(curl -s -X POST http://localhost:9081/rules -H 'Content-Type: application/json' -d @/tmp/rule.json) \
  || { echo "请求 eKuiper /rules 失败, 已停止"; exit 1; }
echo "$RULE_RESP"
if echo "$RULE_RESP" | grep -q '"error"'; then
  echo "建规则失败, 已停止 (见上面的 error 码)"
  exit 1
fi
echo "    [OK] 规则 to_influx 已创建"

echo "==> 规则运行态 (status 应为 running; 看 records_in_total / exceptions_total):"
# GET /rules/{id} 只是 describe（回显规则定义，不含运行态）；运行态在 /status。
curl -s http://localhost:9081/rules/to_influx/status | head -c 600
echo

echo "==> 完成! 下一步:"
echo "    1) 另开终端: cd ~/iot && python3 telemetry_bridge.py --broker 127.0.0.1 --port 1883 --topic device/kunpeng_01/telemetry"
echo "    2) 浏览器打开板子 wlan0 的 IP 加 :3000  (Grafana, admin / robotics123)"
echo "    3) 数据源 InfluxDB 已自动注入; 建面板用 README 第 3 节的 Flux 查询看 j1~j6 / frame_id / det_count"
