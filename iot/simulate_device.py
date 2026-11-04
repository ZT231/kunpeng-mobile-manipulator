#!/usr/bin/env python3
# 模拟端侧设备: 每隔 2 秒向 EMQX 上报一条遥测 JSON
# 运行前: pip3 install paho-mqtt
# 在 KunPeng 上运行: python3 simulate_device.py
# 上报主题: device/esp32_01/telemetry  (匹配 eKuiper 流 device/+/telemetry)
import time, json, random
import paho.mqtt.client as mqtt

BROKER = "localhost"          # compose 已把 1883 映射到 KunPeng 本机
PORT   = 1883
TOPIC  = "device/esp32_01/telemetry"
CLIENT_ID = "sim_esp32_01"

# paho-mqtt 2.x 需要显式指定回调 API 版本
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=CLIENT_ID)
client.connect(BROKER, PORT, 60)
client.loop_start()

print(f"[sim] 开始向 {BROKER}:{PORT} 主题 {TOPIC} 上报遥测 (Ctrl+C 退出)")

try:
    while True:
        payload = {
            "deviceId": "esp32_01",
            "temperature": round(random.uniform(20.0, 30.0), 1),  # 摄氏度
            "humidity":    round(random.uniform(40.0, 60.0), 1),  # 百分比
            "voltage":     round(random.uniform(3.2, 3.4), 2),    # 3.3V 供电电压
        }
        client.publish(TOPIC, json.dumps(payload), qos=1)
        print(f"[sim] -> {payload}")
        time.sleep(2)
except KeyboardInterrupt:
    print("\n[sim] 已停止")
    client.loop_stop()
    client.disconnect()
