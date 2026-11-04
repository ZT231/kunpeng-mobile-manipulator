# -*- coding: utf-8 -*-
"""
手机网页遥控（S5，W 模式 Web 控制）

手机浏览器 --WiFi--> 本服务(Flask) --通道--> STM32F407
通道二选一：
  * 串口：USB-TTL/USART1（PC 串口）
  * WiFi：ESP-01S TCP server（手机/电脑连 ESP 自建热点后直连 192.168.4.1:9000）

帧格式与上位机一致：$<CMD>[,<ARG>...]*<CK><CR><LF>，CK 为 $ 与 * 之间所有字节的 XOR（2 位大写十六进制）
与 kunpeng/robot/arm_host.py 同一套协议约定。

依赖：
  pip install flask pyserial

用法：
  python web_control.py COM7 --baud 115200          # 经 PC 串口控制
  python web_control.py --wifi 192.168.4.1:9000     # 经 WiFi TCP 控制（ESP 自建热点）
  python web_control.py --host 0.0.0.0 --port 5000  # 让手机通过同一 WiFi 访问

注意（2026-11-04 适配鲲鹏项目）：本项目 STM32 执行端经串口直连鲲鹏（默认 serial 模式即可，
端口传 /dev/ttyXXX，如 /dev/ttyUSB0 或板载 UART）；旧项目的 ESP-01S WiFi 模式在本项目由
ESP32-S3 替代（语音+联网），此处 wifi / tcp-server 模式仅作旧项目兼容保留，新项目请走 serial。
脉宽范围已对齐新项目舵机实测标定（J6 上限 2100，取自 docs/舵机标定记录.md 的 6 路实测限位）。
"""

import argparse
import json
import os
import socket
import sys
import threading
import time

try:
    import serial
except ImportError:
    print("缺少依赖 pyserial，请先执行： pip install pyserial")
    sys.exit(1)

try:
    from flask import Flask, jsonify, render_template_string, request
except ImportError:
    print("缺少依赖 flask，请先执行： pip install flask")
    sys.exit(1)

CHANNEL_COUNT = 6
JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]

# 页面滑杆脉宽范围（依据 arm_params.json 的实测脉宽）
PULSE_RANGES = [
    (800, 2300, 1550),   # J1
    (750, 2150, 1470),   # J2
    (520, 2500, 1500),   # J3
    (520, 2500, 1500),   # J4
    (500, 2500, 1450),   # J5
    (900, 2100, 1500),   # J6
]
JOINT_LIMITS = {
    JOINT_NAMES[i]: PULSE_RANGES[i] for i in range(CHANNEL_COUNT)
}


# ---------------------------------------------------------------------------
# 协议层（与 arm_host.py 一致）
# ---------------------------------------------------------------------------
def compute_ck(payload: str) -> int:
    ck = 0
    for ch in payload:
        ck ^= ord(ch)
    return ck & 0xFF


def make_frame(payload: str) -> bytes:
    ck = compute_ck(payload)
    return ("$%s*%02X\r\n" % (payload, ck)).encode("ascii")


# ---------------------------------------------------------------------------
# 串口封装（后台读取线程，MOVE 只写不等，避免拖动时被应答阻塞卡顿）
# ---------------------------------------------------------------------------
class SerialHost:
    def __init__(self, port, baud=115200):
        self.port = port
        try:
            self.ser = serial.Serial(port, baud, timeout=0.2)
        except Exception as e:
            raise RuntimeError("无法打开串口 %s：%s" % (port, e))
        self.wlock = threading.Lock()
        self.rlock = threading.Lock()
        self.rxq = []            # 后台线程解析出的完整应答行（FIFO）
        self._reader_alive = True
        self._reader = threading.Thread(target=self._rx_loop, daemon=True)
        self._reader.start()
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        try:
            self.send("LOG,0")   # 关闭周期遥测，保持 request-response 干净
        except Exception:
            pass

    def _rx_loop(self):
        """后台不断收字节，按 \\r\\n 组装成行放入 rxq。"""
        line = bytearray()
        while self._reader_alive:
            try:
                ch = self.ser.read(1)
            except Exception:
                break
            if not ch:
                continue
            if ch in (b"\r", b"\n"):
                if line:
                    s = bytes(line).decode("ascii", errors="replace").strip()
                    # 只收 "$" 开头的协议帧。printf 重定向到 huart1 与协议共用链路，
                    # 上电 banner / RC522·KEY0 提示 / 每 3s 的 "STATE ... | PULSE ..."
                    # 及 ESP AT 回显都是裸文本；必须丢弃，否则 send() 会误取为应答导致解析失败。
                    if s.startswith("$"):
                        s = s[1:]                 # 去掉帧头 $
                        if "*" in s:
                            s = s.split("*")[0]   # 去掉 *CK 校验段
                        with self.rlock:
                            self.rxq.append(s)
                    line = bytearray()
                continue
            line += ch

    def close(self):
        self._reader_alive = False
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        finally:
            self.ser = None
            self._reader = None

    def _write(self, payload):
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("串口已断开")
        self.ser.write(make_frame(payload))

    def send(self, payload, timeout=0.6):
        """写帧并等待一条新应答（MOVE 等无应答命令由调用侧走 fire 短路）。"""
        with self.wlock:
            with self.rlock:
                self.rxq.clear()          # 丢弃历史残留（含 MOVE 应答），保证读到的就是本次应答
            self._write(payload)
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self.rlock:
                if self.rxq:
                    return self.rxq.pop(0)
            time.sleep(0.005)
        return ""

    def fire(self, payload):
        """只写不等应答：供 MOVE 拖动用，绝不阻塞。"""
        with self.wlock:
            self._write(payload)

    def cmd(self, payload):
        return self.send(payload)


# ---------------------------------------------------------------------------
# WiFi TCP 封装（ESP-01S CIPSERVER，接口与 SerialHost 一致）
# 注意：ESP 非透传模式，STM32->手机 方向应答由 ESP 自动转发到 TCP 连接
# ---------------------------------------------------------------------------
class TcpHost:
    def __init__(self, host, port, timeout=0.2):
        self.addr = "%s:%d" % (host, port)
        try:
            self.sock = socket.create_connection((host, port), timeout=3.0)
        except Exception as e:
            raise RuntimeError("无法连接 WiFi TCP %s：%s" % (self.addr, e))
        self.sock.settimeout(timeout)
        self.buf = b""
        self.wlock = threading.Lock()
        self.rlock = threading.Lock()
        self.rxq = []            # 后台线程解析出的完整应答行（FIFO）
        self._reader_alive = True
        self._reader = threading.Thread(target=self._rx_loop, daemon=True)
        self._reader.start()
        try:
            self.send("LOG,0")   # 关闭周期遥测，保持 request-response 干净
        except Exception:
            pass

    def _rx_loop(self):
        """后台不断收字节，按 \\r\\n 组装成行放入 rxq。"""
        while self._reader_alive:
            try:
                data = self.sock.recv(256)
            except socket.timeout:
                continue
            except Exception:
                break
            if not data:
                continue
            self.buf += data
            while b"\n" in self.buf:
                line, _, self.buf = self.buf.partition(b"\n")
                s = line.decode("ascii", errors="replace").strip().strip("\r")
                # 只收 "$" 开头的协议帧。printf 重定向到 huart1 与协议共用链路，
                # 上电 banner / RC522·KEY0 提示 / 每 3s 的 "STATE ... | PULSE ..."
                # 及 ESP AT 回显都是裸文本；必须丢弃，否则 send() 会误取为应答导致解析失败。
                if s.startswith("$"):
                    s = s[1:]                 # 去掉帧头 $
                    if "*" in s:
                        s = s.split("*")[0]   # 去掉 *CK 校验段
                    with self.rlock:
                        self.rxq.append(s)

    def close(self):
        self._reader_alive = False
        try:
            self.sock.close()
        except Exception:
            pass

    def _write(self, payload):
        if not self.sock:
            raise RuntimeError("连接已断开")
        self.sock.sendall(make_frame(payload))

    def send(self, payload, timeout=0.6):
        """写帧并等待一条新应答（MOVE 等无应答命令由调用侧走 fire 短路）。"""
        with self.wlock:
            with self.rlock:
                self.rxq.clear()
            self._write(payload)
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self.rlock:
                if self.rxq:
                    return self.rxq.pop(0)
            time.sleep(0.005)
        return ""

    def fire(self, payload):
        """只写不等应答：供 MOVE 拖动用，绝不阻塞。"""
        with self.wlock:
            self._write(payload)

    def cmd(self, payload):
        return self.send(payload)


class TcpServerHost:
    """电脑做 TCP server：监听端口，接受 ESP-01S 主动连入，接口与 SerialHost/TcpHost 一致。

    架构：ESP(AP 热点) --CIPSTART--> 本机 TCP server --透传--> UART 双向往来。
    透传模式下 ESP 会把 TCP 收到的字节原样发到 STM32，STM32 的应答原样回发 TCP。
    """

    def __init__(self, listen_port, timeout=1.0):
        self.addr = "0.0.0.0:%d" % listen_port
        self._port = listen_port
        self.sock = None
        self.buf = b""
        self.wlock = threading.Lock()
        self.rlock = threading.Lock()
        self.rxq = []            # 后台线程解析出的完整应答行（FIFO）
        self._reader_alive = False
        self._reader = None
        # 监听 socket 与 accept 线程：ESP 起来后会自动连入
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.settimeout(1.0)
        self.srv.bind(("0.0.0.0", listen_port))
        self.srv.listen(1)
        self._acc_alive = True
        self._acc = threading.Thread(target=self._accept_loop, daemon=True)
        self._acc.start()

    def _accept_loop(self):
        """后台接受 ESP 连入；连上后启动读线程，并把响应行放进 rxq。"""
        while self._acc_alive:
            try:
                conn, _ = self.srv.accept()
            except socket.timeout:
                continue
            except Exception:
                break
            if not self._acc_alive:
                conn.close()
                break
            with self.wlock:
                if self.sock is not None:
                    try:
                        self.sock.close()
                    except Exception:
                        pass
                self.sock = conn
                self.sock.settimeout(timeout)
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._reader_alive = True
            self._reader = threading.Thread(target=self._rx_loop, daemon=True)
            self._reader.start()

    def _rx_loop(self):
        """后台不断收字节，按 \\r\\n 组装成行放入 rxq。"""
        while self._reader_alive:
            try:
                data = self.sock.recv(512)
            except socket.timeout:
                continue
            except Exception:
                break
            if not data:
                break
            self.buf += data
            while b"\n" in self.buf:
                line, _, self.buf = self.buf.partition(b"\n")
                s = line.decode("ascii", errors="replace").strip().strip("\r")
                # 只收 "$" 开头的协议帧。printf 重定向到 huart1 与协议共用链路，
                # 上电 banner / RC522·KEY0 提示 / 每 3s 的 "STATE ... | PULSE ..."
                # 及 ESP AT 回显都是裸文本；必须丢弃，否则 send() 会误取为应答导致解析失败。
                if s.startswith("$"):
                    s = s[1:]                 # 去掉帧头 $
                    if "*" in s:
                        s = s.split("*")[0]   # 去掉 *CK 校验段
                    with self.rlock:
                        self.rxq.append(s)

    def _write(self, payload):
        if not self.sock:
            raise RuntimeError("无设备连入")
        self.sock.sendall(make_frame(payload))

    def send(self, payload, timeout=0.6):
        with self.wlock:
            with self.rlock:
                self.rxq.clear()
            self._write(payload)
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self.rlock:
                if self.rxq:
                    return self.rxq.pop(0)
            time.sleep(0.005)
        return ""

    def fire(self, payload):
        with self.wlock:
            self._write(payload)

    def cmd(self, payload):
        return self.send(payload)

    def close(self):
        self._acc_alive = False
        self._reader_alive = False
        try:
            self.srv.close()
        except Exception:
            pass
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
        self.sock = None


_HOST = None
_HOST_STATE = {"connected": False, "port": "", "error": ""}
_MODE = "serial"          # "serial" / "wifi" / "wifi_server"
_WIFI_ADDR = ""


# ---------------------------------------------------------------------------
# Flask
# ---------------------------------------------------------------------------
app = Flask(__name__)

PAGE = r"""
<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>机械臂遥控</title>
<style>
  :root{--bg:#0f1220;--card:#1a1f35;--ac:#ff9800;--ok:#22c55e;--bad:#ef4444;--tx:#e8eaf6;}
  *{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent;}
  body{background:var(--bg);color:var(--tx);font-family:-apple-system,"PingFang SC",Roboto,sans-serif;padding:14px 14px 90px;}
  h1{font-size:20px;font-weight:600;display:flex;align-items:center;gap:8px;margin-bottom:4px;}
  .sub{font-size:12px;color:#8a90b0;margin-bottom:12px;}
  .card{background:var(--card);border-radius:12px;padding:12px;margin-bottom:12px;}
  .card h2{font-size:13px;color:#aab0d8;margin-bottom:10px;letter-spacing:1px;}
  .row{display:flex;gap:8px;align-items:center;flex-wrap:wrap;}
  .dot{width:10px;height:10px;border-radius:50%;background:#555;display:inline-block;margin-right:6px;}
  .dot.on{background:var(--ok);}
  select,input[type=text]{background:#12162b;border:1px solid #2e3557;color:var(--tx);border-radius:8px;padding:9px 10px;font-size:15px;flex:1;min-width:120px;}
  button{background:#232a4d;color:var(--tx);border:none;border-radius:10px;padding:10px 14px;font-size:15px;font-weight:600;flex:1;cursor:pointer;min-height:42px;}
  button:active{filter:brightness(1.25);transform:scale(.98);}
  .ac{background:var(--ac);color:#111;}
  .ok{background:#1d4a2e;color:#b6ffcf;}
  .bad{background:#4a1d1d;color:#ffb8b8;}
  .g{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;}
  .big{font-size:15px;}
  .ch{background:var(--card);border-radius:12px;padding:10px 12px;margin-bottom:10px;}
  .ch .top{display:flex;justify-content:space-between;align-items:center;margin-bottom:6px;}
  .ch .name{font-weight:700;font-size:15px;}
  .ch .val{font-family:ui-monospace,Menlo,monospace;font-size:18px;color:var(--ac);}
  .ch .lim{font-size:11px;color:#8a90b0;}
  input[type=range]{width:100%;accent-color:var(--ac);height:30px;}
  .ch .btns{display:flex;gap:8px;margin-top:4px;}
  .ch .btns button{padding:8px;min-height:38px;font-size:13px;}
  #log{font-family:ui-monospace,Menlo,monospace;font-size:11px;line-height:1.6;white-space:pre-wrap;word-break:break-all;background:#0a0d1a;border-radius:8px;padding:10px;max-height:220px;overflow:auto;}
</style>
</head>
<body>

<h1><span class="dot on" id="led"></span>机械臂遥控</h1>
<div class="sub" id="sub">未连接</div>

<div class="card">
  <h2>连接</h2>
  {% if mode == 'wifi' %}
  <div class="row">
    <div style="flex:1;font-size:13px;color:#aab0d8">WiFi TCP：{{wifi_addr}}<br><span style="font-size:11px">手机/电脑需连接 ESP 自建热点</span></div>
    <button class="ac" id="btnConn" onclick="toggleConn()">连接</button>
  </div>
  {% elif mode == 'wifi_server' %}
  <div class="row">
    <div style="flex:1;font-size:13px;color:#aab0d8">TCP server：{{wifi_addr}}<br><span style="font-size:11px">ESP 上电后会自动连入本电脑，点「连接」接管通道</span></div>
    <button class="ac" id="btnConn" onclick="toggleConn()">连接</button>
  </div>
  {% else %}
  <div class="row">
    <input type="text" id="port" placeholder="串口号，如 COM7" value="{{port}}">
    <button class="ac" id="btnConn" onclick="toggleConn()">连接</button>
  </div>
  {% endif %}
  <div class="row" style="margin-top:8px">
    <button onclick="api('PING')">PING</button>
    <button onclick="api('STATE')">读状态</button>
    <button onclick="api('HOME')">回 HOME</button>
  </div>
</div>

{% for i in range(6) %}
<div class="ch" data-i="{{i}}">
  <div class="top">
    <span class="name">{{names[i]}}</span>
    <span class="val" id="v{{i}}">{{mids[i]}}</span>
  </div>
  <div class="lim">{{mins[i]}} ~ {{maxs[i]}} us</div>
  <input type="range" id="s{{i}}" min="{{mins[i]}}" max="{{maxs[i]}}" value="{{mids[i]}}"
         oninput="drag({{i}}, this)"
         onchange="clearTimeout(dragTimers[{{i}}]); api('MOVE,{{i}},'+this.value)">
  <div class="btns">
    <button onclick="step({{i}},-20)">-20</button>
    <button onclick="step({{i}},20)">+20</button>
    <button onclick="goMid({{i}})">中位</button>
  </div>
</div>
{% endfor %}

<div class="card">
  <h2>夹爪 / 运动</h2>
  <div class="g">
    <button onclick="apiRaw('MOVE,5,900')">夹爪张开(900)</button>
    <button onclick="apiRaw('MOVE,5,1500')">夹爪半开(1500)</button>
    <button onclick="apiRaw('MOVE,5,1900')">夹爪闭合(1900)</button>
    <button class="ok" onclick="autoHome()">六轴回中位</button>
    <button onclick="api('STOP')">停止</button>
    <button onclick="api('RESUME')">继续</button>
  </div>
  <div class="row" style="margin-top:8px">
    <button onclick="api('MANUAL')">进入手动</button>
    <button onclick="api('REC')">示教开始</button>
    <button onclick="api('RECEND')">示教结束</button>
    <button onclick="api('PLAY')">回放</button>
  </div>
</div>

<div class="card">
  <h2>运行日志</h2>
  <div id="log">(空)</div>
</div>

<script>
let connected=false;
function tag(name,val){var e=document.getElementById(name);if(e)e.textContent=val;}

function led(c){var e=document.getElementById('led');e.className='dot'+(c?' on':'');}

function setConn(on,msg){
  connected=on;
  led(on);
  tag('sub',msg);
  tag('btnConn',on?'断开':'连接');
  document.getElementById('btnConn').classList.toggle('ac',!on);
}

async function post(url,body){
  try{
    const r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body||{})});
    return await r.json();
  }catch(e){ log('⚠ 网络错误：'+e); return null; }
}

function log(s){
  const box=document.getElementById('log');
  const t='['+new Date().toLocaleTimeString()+'] '+s+"\n";
  box.textContent=t+box.textContent;
  if(box.scrollHeight>box.clientHeight*2)box.textContent=box.textContent.slice(0,4000);
}

async function doCmd(payload){
  log('>> '+payload);
  const r=await post('/api/cmd',{payload:payload});
  if(!r) return;
  if(r.ok){ log('<< '+r.resp); return r.resp; }
  else { log('✗ '+r.error); return null; }
}
function api(payload){ doCmd(payload); }
function apiRaw(payload){ doCmd(payload); }

// 拖动实时跟随：固定间隔节流，连续拖动也每 ~60ms 至少发一次最新值
const dragTimers={}, dragBusy={}, dragNext={};
function sendDrag(i,v){ dragNext[i]=null; doCmd('MOVE,'+i+','+v); }
function drag(i,el){
  tag('v'+i, el.value);
  dragNext[i]=el.value;            // 记住最新目标
  if(dragBusy[i]) return;          // 上次发送的 60ms 窗口还没到
  dragBusy[i]=true;
  sendDrag(i, dragNext[i]);
  dragTimers[i]=setTimeout(()=>{   // 窗口结束：放行下一次，并补发遗漏的最新值
    dragBusy[i]=false;
    if(dragNext[i]!=null) sendDrag(i, dragNext[i]);
  }, 60);
}

// 读实际脉宽回填界面（HOME/MOVE 后滑杆位置与实际一致）
async function syncPulses(){
  if(!connected) return;
  const r=await post('/api/state');
  if(r&&r.ok&&Array.isArray(r.pulses)&&r.pulses.length===6){
    for(let i=0;i<6;i++){
      const s=document.getElementById('s'+i);
      if(!s) continue;
      s.value=String(r.pulses[i]);
      tag('v'+i, r.pulses[i]);
    }
  }
}
// 每 1s 轮询实际位置，保证界面始终跟真机一致
setInterval(()=>{ if(connected) syncPulses(); }, 1000);

async function toggleConn(){
  if(connected){ const r=await post('/api/close'); if(r&&r.ok) setConn(false,'未连接'); return; }
  if('{{mode}}'==='wifi'||'{{mode}}'==='wifi_server'){
    const r=await post('/api/open',{});
    if(r&&r.ok){ setConn(true,('{{mode}}'==='wifi_server'?'TCP server 已就绪：':'WiFi 已连接：')+r.port); doCmd('PING'); }
    else log('✗ 连接失败：'+(r?r.error:'未知'));
    return;
  }
  const port=document.getElementById('port').value.trim();
  const r=await post('/api/open',{port:port});
  if(r&&r.ok){ setConn(true,'已连接：'+port); if(r.home)tag('sub','已连接：'+port); doCmd('PING'); }
  else log('✗ 连接失败：'+(r?r.error:'未知'));
}

function step(i,d){
  const s=document.getElementById('s'+i);
  let v=parseInt(s.value)+d;
  v=Math.max(parseInt(s.min),Math.min(parseInt(s.max),v));
  s.value=v; tag('v'+i,v);
  doCmd('MOVE,'+i+','+v);
}
function goMid(i){ api('MOVE,'+i+','+({{mids|tojson}})[i]); }

async function autoHome(){
  {% for i in range(6) %}
  await doCmd('MOVE,{{i}},{{mids[i]}}');
  {% endfor %}
}
</script>
</body>
</html>
"""


@app.route("/")
def index():
    return render_template_string(
        PAGE,
        port=_HOST_STATE["port"],
        mode=_MODE,
        wifi_addr=_WIFI_ADDR,
        names=JOINT_NAMES,
        mins=[r[0] for r in PULSE_RANGES],
        maxs=[r[1] for r in PULSE_RANGES],
        mids=[r[2] for r in PULSE_RANGES],
    )


@app.route("/api/open", methods=["POST"])
def api_open():
    global _HOST
    if _MODE == "wifi_server":
        # 电脑已作为 TCP server 监听（main 已创建 _HOST），复用而非重建，
        # 避免 close() 后又因 _HOST 仍非 None 造成"连接后再也不复建"。ESP 连入后即就绪。
        try:
            if _HOST is None:
                _HOST = TcpServerHost(_WIFI_ADDR)
            _HOST_STATE.update(connected=True, port="TCP server :%s (等 ESP 连入)" % _WIFI_ADDR, error="")
            return jsonify(ok=True, port=_HOST_STATE["port"])
        except Exception as e:
            _HOST_STATE.update(connected=False, error=str(e))
            return jsonify(ok=False, error=str(e))
    # 串口 / WiFi client：关闭旧连接并新建
    if _HOST is not None:
        try:
            _HOST.close()
        except Exception:
            pass
        _HOST = None
    if _MODE == "wifi":
        host, _, port_s = _WIFI_ADDR.partition(":")
        try:
            _HOST = TcpHost(host, int(port_s or 9000))
            _HOST_STATE.update(connected=True, port=_WIFI_ADDR, error="")
            return jsonify(ok=True, port=_WIFI_ADDR)
        except Exception as e:
            _HOST_STATE.update(connected=False, error=str(e))
            return jsonify(ok=False, error=str(e))
    data = request.get_json(force=True, silent=True) or {}
    port = data.get("port") or _HOST_STATE["port"]
    try:
        _HOST = SerialHost(port)
        _HOST_STATE.update(connected=True, port=port, error="")
        return jsonify(ok=True, port=port)
    except Exception as e:
        _HOST_STATE.update(connected=False, error=str(e))
        return jsonify(ok=False, error=str(e))


@app.route("/api/close", methods=["POST"])
def api_close():
    global _HOST
    if _HOST is not None:
        try:
            _HOST.close()
        except Exception:
            pass
    _HOST = None
    _HOST_STATE.update(connected=False, error="")
    return jsonify(ok=True)


@app.route("/api/cmd", methods=["POST"])
def api_cmd():
    global _HOST
    data = request.get_json(force=True, silent=True) or {}
    payload = str(data.get("payload", "")).strip()
    if not payload:
        return jsonify(ok=False, error="空命令")
    if _HOST is None:
        return jsonify(ok=False, error="未连接")
    try:
        # MOVE 下位机不回应答，走 fire 只写不等待，保证拖动实时跟手
        if payload.upper().startswith("MOVE"):
            _HOST.fire(payload)
            return jsonify(ok=True, resp="")
        resp = _HOST.cmd(payload)
        return jsonify(ok=True, resp=resp)
    except Exception as e:
        return jsonify(ok=False, error=str(e))


@app.route("/api/state", methods=["POST"])
def api_state():
    global _HOST
    if _HOST is None:
        return jsonify(ok=False, error="未连接")
    try:
        resp = _HOST.cmd("STATE")
    except Exception as e:
        return jsonify(ok=False, error=str(e))
    parts = resp.split(",")
    pulses = []
    try:
        for tok in parts[3:9]:
            pulses.append(int(tok))
    except (ValueError, IndexError):
        return jsonify(ok=False, error="STATE 响应异常: %r" % resp)
    return jsonify(ok=True, pulses=pulses)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("serial", nargs="?", default=None, help="STM32 串口号，如 COM7")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--wifi", help="旧式 WiFi client 通道，如 192.168.4.1:9000（ESP 自建热点）")
    ap.add_argument("--tcp-server", type=int, nargs="?", const=9000,
                    metavar="PORT", help="电脑做 TCP server 等 ESP 连入（建议 9000）")
    ap.add_argument("--host", default="0.0.0.0", help="监听地址，默认 0.0.0.0")
    ap.add_argument("--port", type=int, default=5000, help="HTTP 端口，默认 5000")
    args = ap.parse_args()

    global _MODE, _WIFI_ADDR

    if args.tcp_server is not None:
        _MODE = "wifi_server"
        _WIFI_ADDR = args.tcp_server
        try:
            _HOST = TcpServerHost(_WIFI_ADDR)
            _HOST_STATE.update(connected=True, port="TCP server :%d" % _WIFI_ADDR, error="")
        except Exception as e:
            _HOST_STATE.update(connected=False, error=str(e))
            print("  [警告] 启动 TCP server 失败：%s（可稍后在页面点「连接」重试）" % e)
        chan_desc = "TCP server :%d (等 ESP-01S 连入)" % _WIFI_ADDR
    elif args.wifi:
        _MODE = "wifi"
        host, _, port_s = args.wifi.partition(":")
        wifi_port = int(port_s or 9000)
        _WIFI_ADDR = "%s:%d" % (host, wifi_port)
        try:
            _HOST = TcpHost(host, wifi_port)
            _HOST_STATE.update(connected=True, port=_WIFI_ADDR, error="")
        except Exception as e:
            _HOST_STATE.update(connected=False, error=str(e))
            print("  [警告] 启动时连接 WiFi TCP 失败：%s（可稍后在页面点「连接」重试）" % e)
        chan_desc = "WiFi TCP %s" % _WIFI_ADDR
    else:
        _MODE = "serial"
        port = args.serial or "COM7"
        _HOST_STATE["port"] = port
        _HOST_STATE["baud"] = args.baud
        chan_desc = "串口 %s @ %d baud" % (port, args.baud)

    print("=" * 56)
    print("机械臂网页遥控已启动")
    print("  通道：%s" % chan_desc)
    print("  网页：http://<本机IP>:%d" % args.port)
    if _MODE == "wifi_server":
        print("  提示：电脑先连 ESP 自建热点 并固定 IP 192.168.4.50；"
              "手机/电脑浏览器打开 http://<本机IP>:%d 即可控制" % args.port)
    elif _MODE == "wifi":
        print("  提示：手机/电脑连接 ESP 自建热点 后，在手机浏览器打开： http://<本机IP>:%d" % args.port)
    else:
        print("  提示：手机与电脑连同一 WiFi 后，在手机浏览器打开： http://<本机IP>:%d" % args.port)
    print("=" * 56)
    app.run(host=args.host, port=args.port, debug=False)


if __name__ == "__main__":
    main()