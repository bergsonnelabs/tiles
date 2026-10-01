#!/usr/bin/env python3
"""Web-based live touch visualizer for the Sense.CAP tile (driver v1.0).

Reads the hw-sense-cap firmware serial stream and serves a live view at
http://localhost:8765: surface heatmap, finger + trail, delta bars with
threshold, event ticker; /slider is a one-axis scroll view. The surface
geometry (channels, resolution, axis switch) comes from the firmware's S
line, so every layout preset draws correctly. Stdlib + pyserial only; the
page is a single HTML5 canvas fed by Server-Sent Events.

Usage:  python3 visualizer_web.py [serial-port]
"""

import glob
import json
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial

PORT = 8765

state_lock = threading.Lock()
ser_handle = None
state = {
    "t": 0, "nf": 0, "x": 0, "y": 0, "strength": 0,
    "zone": -1, "touchbits": 0, "deltas": [0] * 6, "counts": [0] * 6,
    "alive": False,
    # Surface geometry from the S line (defaults: the 2x3 grid).
    "layout": 1, "nrx": 2, "ntx": 3, "xres": 512, "yres": 256, "sw": 1,
}
events = deque(maxlen=200)
event_seq = 0


def serial_port():
    if len(sys.argv) > 1:
        return sys.argv[1]
    ports = glob.glob("/dev/tty.usbmodem*")
    return ports[0] if ports else None


def reader():
    global event_seq
    while True:
        port = serial_port()
        if not port:
            time.sleep(1)
            continue
        try:
            s = serial.Serial(port, 115200, timeout=1)
            global ser_handle
            ser_handle = s
            with state_lock:
                state["alive"] = True
            while True:
                line = s.readline().decode(errors="replace").strip()
                if not line:
                    continue
                parts = line.split(",")
                if parts[0] == "S" and len(parts) >= 7:
                    try:
                        with state_lock:
                            state.update(
                                layout=int(parts[1]), nrx=int(parts[2]),
                                ntx=int(parts[3]), xres=int(parts[4]),
                                yres=int(parts[5]), sw=int(parts[6]))
                    except ValueError:
                        pass
                elif parts[0] == "D" and len(parts) >= 8:
                    try:
                        with state_lock:
                            n = (len(parts) - 8) // 2  # deltas then counts
                            state.update(
                                t=int(parts[1]), nf=int(parts[2]),
                                x=int(parts[3]), y=int(parts[4]),
                                strength=int(parts[5]), zone=int(parts[6]),
                                touchbits=int(parts[7], 16),
                                deltas=[int(v) for v in parts[8:8 + n]],
                                counts=[int(v) for v in parts[8 + n:8 + 2 * n]])
                    except ValueError:
                        pass
                elif parts[0] == "E" and len(parts) >= 2:
                    with state_lock:
                        event_seq += 1
                        events.append({"i": event_seq,
                                       "t": time.strftime("%H:%M:%S"),
                                       "msg": " ".join(parts[1:])})
                elif line in ("PASS", "FAIL (ATI)") or "IQS7211A" in line:
                    with state_lock:
                        event_seq += 1
                        events.append({"i": event_seq,
                                       "t": time.strftime("%H:%M:%S"),
                                       "msg": line})
        except (serial.SerialException, OSError):
            with state_lock:
                state["alive"] = False
            time.sleep(1)


PAGE = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>Sense.CAP touch</title>
<style>
  body { background:#101418; color:#c0cad4; font-family:Menlo,monospace;
         display:flex; gap:16px; padding:16px; }
  #wrap { flex:0 0 auto; }
  canvas { background:#0a0d10; border-radius:8px; }
  #ticker { flex:1; background:#0a0d10; border-radius:8px; padding:10px;
            height:520px; overflow-y:auto; font-size:12px; color:#9fd8a0; }
  #ticker div { padding:1px 0; }
  h3 { margin:4px 0 10px; color:#8090a0; font-size:13px; }
  button { background:#1d242c; color:#c0cad4; border:1px solid #39424d;
           border-radius:5px; font-family:Menlo,monospace; font-size:12px;
           padding:4px 10px; margin-right:4px; cursor:pointer; }
  button:hover { background:#2a333d; }
</style></head>
<body>
<div id="wrap">
  <h3 id="hdr">Sense.CAP &mdash; connecting&hellip;</h3>
  <div style="margin-bottom:8px">
    <button onclick="cmd('r')">Re-ATI (re-baseline)</button>
    <button onclick="cmd('x')">MCLR reset</button>
    <a href="/slider" style="color:#8090a0;margin-left:10px">scroll view</a>
  </div>
  <div style="margin-bottom:8px">
    sensitivity:
    <button onclick="cmd('s1')">1</button><button onclick="cmd('s2')">2</button><button onclick="cmd('s3')">3</button><button onclick="cmd('s4')">4</button><button onclick="cmd('s5')">5</button>
    stickiness:
    <button onclick="cmd('h1')">1</button><button onclick="cmd('h2')">2</button><button onclick="cmd('h3')">3</button><button onclick="cmd('h4')">4</button><button onclick="cmd('h5')">5</button>
  </div>
  <div style="margin-bottom:8px">
    layout:
    <button onclick="cmd('l1')">grid 2x3</button><button onclick="cmd('l2')">buttons 2x3</button><button onclick="cmd('l3')">slider 1x3</button><button onclick="cmd('l4')">slider 1x4</button><button onclick="cmd('l0')">none</button>
  </div>
  <canvas id="cv" width="820" height="560"></canvas>
</div>
<div>
  <h3>events</h3>
  <div id="ticker"></div>
</div>
<script>
// Geometry follows the firmware's S line; the canvas keeps a fixed size.
const W=820-20, H=410, BAR=120, PAD=10, LAYOUTS=['none','grid 2x3','buttons 2x3','slider 1x3','slider 1x4'];
const DELTA_FULL=200, THRESH=Math.floor(900*8/128);
let XR=512, YR=256, COLS=3, ROWS=2, NRX=2, SW=1, SX=W/512, SY=H/256;
function geom(d){
  NRX=Math.max(1,d.nrx); SW=d.sw;
  COLS=SW?d.ntx:d.nrx; ROWS=SW?d.nrx:d.ntx;
  XR=Math.max(1,d.xres); YR=Math.max(1,d.yres);
  SX=W/XR; SY=H/YR;
}
// channel for grid cell (c, r): along the Rxs first, then the next Tx (5.1.1)
function chan(c,r){ return SW ? c*NRX+r : r*NRX+c; }
const cv=document.getElementById('cv'), ctx=cv.getContext('2d');
const ticker=document.getElementById('ticker'), hdr=document.getElementById('hdr');
let trail=[], seen=0;

function heat(d){
  const f=Math.max(0,Math.min(1,d/DELTA_FULL));
  const r=0x20+f*(0xff-0x20), g=0x2a+f*(0x8a-0x2a), b=0x33+f*(0x1a-0x33);
  return `rgb(${r|0},${g|0},${b|0})`;
}

function draw(s){
  ctx.clearRect(0,0,cv.width,cv.height);
  const cw=W/COLS, ch=H/ROWS;
  for(let c=0;c<COLS;c++) for(let r=0;r<ROWS;r++){
    const z=chan(c,r), x0=PAD+c*cw, y0=PAD+r*ch;
    const touched=(s.touchbits>>z)&1;
    const dz=s.deltas[z]||0;
    ctx.fillStyle=heat(dz);
    ctx.fillRect(x0+3,y0+3,cw-6,ch-6);
    ctx.strokeStyle=touched?'#eeeeee':'#39424d';
    ctx.lineWidth=touched?3:1;
    ctx.strokeRect(x0+3,y0+3,cw-6,ch-6);
    ctx.fillStyle='#8090a0'; ctx.font='bold 13px Menlo';
    ctx.fillText(z, x0+10, y0+20);
    ctx.fillStyle='#c0cad4'; ctx.font='11px Menlo';
    ctx.fillText((dz>=0?'+':'')+dz, x0+cw-52, y0+ch-12);
  }
  // trail + finger
  trail.forEach((p,i)=>{
    const f=i/Math.max(1,trail.length), r=2+4*f;
    ctx.fillStyle=`rgba(${64+f*143|0},220,60,${0.15+0.35*f})`;
    ctx.beginPath();
    ctx.arc(PAD+p[0]*SX,PAD+p[1]*SY,r,0,7); ctx.fill();
  });
  if(s.nf>0 && s.x<65535){
    const fx=PAD+s.x*SX, fy=PAD+Math.min(s.y,YR)*SY, r=10+Math.min(20,s.strength/40);
    ctx.strokeStyle='#ffe066'; ctx.lineWidth=3;
    ctx.beginPath(); ctx.arc(fx,fy,r,0,7); ctx.stroke();
    ctx.fillStyle='#ffe066'; ctx.font='12px Menlo';
    ctx.fillText(`(${s.x},${s.y}) z${s.zone} s${s.strength}`,
                 Math.min(fx,W-140), Math.max(16,fy-18));
  }
  // delta bars
  const nch=Math.max(1,s.deltas.length), top=PAD+H+12, bw=W/nch,
        span=Math.max(DELTA_FULL,...s.deltas.map(Math.abs)),
        mid=top+(BAR-24)/2;
  for(let z=0;z<s.deltas.length;z++){
    const x0=PAD+z*bw+10, h=(s.deltas[z]/span)*(BAR-28)/2;
    ctx.fillStyle=heat(Math.abs(s.deltas[z]));
    ctx.fillRect(x0,Math.min(mid,mid-h),bw-20,Math.abs(h));
    ctx.fillStyle='#8090a0'; ctx.font='11px Menlo';
    ctx.fillText('ch'+z, x0+(bw-20)/2-12, top+BAR-8);
  }
  const ty=mid-(THRESH/span)*(BAR-28)/2;
  ctx.strokeStyle='#ff6666'; ctx.setLineDash([4,3]); ctx.lineWidth=1;
  ctx.beginPath(); ctx.moveTo(PAD,ty); ctx.lineTo(PAD+W,ty); ctx.stroke();
  ctx.setLineDash([]);
  ctx.strokeStyle='#39424d';
  ctx.beginPath(); ctx.moveTo(PAD,mid); ctx.lineTo(PAD+W,mid); ctx.stroke();
  ctx.fillStyle='#ff6666'; ctx.font='10px Menlo';
  ctx.fillText('thr '+THRESH, PAD+W-70, ty-5);
}

function cmd(c){ fetch('/cmd?c='+c); }
const es=new EventSource('/events');
es.onmessage=(m)=>{
  const d=JSON.parse(m.data);
  geom(d);
  hdr.innerHTML='Sense.CAP '+(LAYOUTS[d.layout]||'custom')+' &mdash; '+(d.alive?'live':'no serial');
  if(d.nf>0 && d.x<65535){ trail.push([d.x,d.y]); if(trail.length>40) trail.shift(); }
  else if(trail.length) trail.shift();
  draw(d);
  for(const e of d.events){
    if(e.i>seen){
      seen=e.i;
      const div=document.createElement('div');
      div.textContent=e.t+' '+e.msg;
      ticker.prepend(div);
      while(ticker.childNodes.length>120) ticker.removeChild(ticker.lastChild);
    }
  }
};
</script></body></html>
"""


SLIDER_PAGE = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>Sense.CAP scroll</title>
<style>
 body{background:#101418;color:#c0cad4;font-family:Menlo,monospace;margin:0;
      padding:18px;display:flex;gap:28px}
 #track{position:relative;width:120px;height:560px;background:#0a0d10;
        border-radius:14px;border:1px solid #39424d;overflow:hidden}
 #fill{position:absolute;left:0;right:0;bottom:0;background:#1a2028}
 #ghost{position:absolute;left:0;right:0;height:2px;background:#3b82c4;opacity:.55}
 #puck{position:absolute;left:8px;right:8px;height:52px;border-radius:10px;
       background:#ffe066}
 body.off #puck{background:#3a4048}
 body.bridged #puck{background:#c9a227}
 body.off #fill{background:#141a20}
 .panel{font-size:13px;line-height:2.0}
 .big{font-size:34px;color:#ffe066;line-height:1.3}
 body.off .big{color:#6b737d}
 .lab{color:#8090a0}
 input[type=range]{width:170px;vertical-align:middle}
 button{background:#1d242c;color:#c0cad4;border:1px solid #39424d;border-radius:5px;
        font-family:Menlo,monospace;font-size:12px;padding:5px 11px;margin-right:6px;
        cursor:pointer}
 button:hover{background:#2a333d}
 hr{border:0;border-top:1px solid #39424d;margin:14px 0}
 a{color:#8090a0}
</style></head><body class="off">
<div id="track"><div id="fill"></div><div id="ghost"></div><div id="puck"></div></div>
<div class="panel">
 <div class="big" id="pct">--</div>
 <div><span class="lab">raw</span> <span id="raw">-</span>
      &nbsp; <span class="lab">smoothed</span> <span id="sm">-</span>
      &nbsp; <span class="lab">of</span> <span id="xr">511</span></div>
 <div><span class="lab">contact</span> <span id="tch">up</span>
      &nbsp; <span class="lab">zone</span> <span id="zone">-</span></div>
 <div><span class="lab">travel</span> <span id="acc">0</span>
      &nbsp; <span class="lab">detents</span> <span id="det">0</span></div>
 <div><span class="lab">dropouts</span> <span id="drops">0</span>
      &nbsp; <span class="lab">longest gap</span> <span id="gap">0</span> ms</div>
 <hr>
 <div><span class="lab">smoothing</span>
      <input id="alpha" type="range" min="5" max="100" value="25">
      <span id="av">0.25</span></div>
 <div><span class="lab">median window</span>
      <input id="med" type="range" min="1" max="9" step="2" value="5">
      <span id="mv">5</span></div>
 <div><span class="lab">detent size</span>
      <input id="dsz" type="range" min="16" max="160" step="8" value="64">
      <span id="dv">64</span></div>
 <div><span class="lab">bridge dropouts</span>
      <input id="hold" type="range" min="0" max="400" step="25" value="200">
      <span id="hv">200</span> ms</div>
 <div><label><input type="checkbox" id="flip"> flip direction</label></div>
 <hr>
 <div><button onclick="fetch('/cmd?c=r')">Re-ATI (recalibrate)</button>
      <button onclick="resetCounters()">reset travel</button></div>
 <div><span class="lab">sensitivity</span>
      <button onclick="fetch('/cmd?c=s1')">1</button><button onclick="fetch('/cmd?c=s2')">2</button><button onclick="fetch('/cmd?c=s3')">3</button><button onclick="fetch('/cmd?c=s4')">4</button><button onclick="fetch('/cmd?c=s5')">5</button></div>
 <div><span class="lab">stickiness</span>
      <button onclick="fetch('/cmd?c=h1')">1</button><button onclick="fetch('/cmd?c=h2')">2</button><button onclick="fetch('/cmd?c=h3')">3</button><button onclick="fetch('/cmd?c=h4')">4</button><button onclick="fetch('/cmd?c=h5')">5</button></div>
 <div><span class="lab">layout</span>
      <button onclick="fetch('/cmd?c=l3')">slider 1x3</button><button onclick="fetch('/cmd?c=l4')">slider 1x4</button><button onclick="fetch('/cmd?c=l1')">grid 2x3</button></div>
 <div><a href="/">full surface view</a></div>
</div>
<script>
let XR=511;
const PUCK=52;
let buf=[], ema=null, acc=0, det=0, lastSm=null;
let rawWasDown=false, lastRawTime=0, drops=0, gapMax=0;
const $=id=>document.getElementById(id);
function resetCounters(){ acc=0; det=0; drops=0; gapMax=0;
  ['acc','det','drops','gap'].forEach(i=>$(i).textContent='0'); }
$('alpha').oninput=()=>$('av').textContent=($('alpha').value/100).toFixed(2);
$('med').oninput=()=>$('mv').textContent=$('med').value;
$('dsz').oninput=()=>$('dv').textContent=$('dsz').value;
$('hold').oninput=()=>$('hv').textContent=$('hold').value;

const es=new EventSource('/events');
es.onmessage=m=>{
 const d=JSON.parse(m.data);
 XR=Math.max(1,d.xres-1); $('xr').textContent=XR;
 const raw = d.nf>0 && d.x<65535;
 const now = Date.now();
 const HOLD = +$('hold').value;
 const H=$('track').clientHeight;

 if(raw){
   const gap = lastRawTime ? now-lastRawTime : 1e9;
   if(!rawWasDown){
     if(gap<=HOLD && ema!==null){
       // chip let go briefly mid-stroke: count it, keep the filter running
       drops++; if(gap>gapMax) gapMax=gap;
     } else {
       buf=[]; ema=null; lastSm=null;   // a genuinely new contact starts clean
     }
   }
   lastRawTime=now;
   buf.push(d.x);
   const W=+$('med').value;
   while(buf.length>W) buf.shift();
   const srt=[...buf].sort((a,b)=>a-b);
   const med=srt[Math.floor(srt.length/2)];   // median rejects single-frame spikes
   const A=$('alpha').value/100;
   ema = (ema===null) ? med : ema + A*(med-ema);
   if(lastSm!==null){
     acc += (ema-lastSm);
     det = Math.trunc(acc/(+$('dsz').value));
   }
   lastSm=ema;
 }
 rawWasDown=raw;
 // bridge brief drop-outs so one stroke reads as one gesture
 const down = raw || (lastRawTime && now-lastRawTime<HOLD && ema!==null);
 document.body.classList.toggle('off', !down);
 document.body.classList.toggle('bridged', !raw && down);

 if(ema!==null){
   let f=ema/XR; if($('flip').checked) f=1-f;
   f=Math.max(0,Math.min(1,f));
   const y=f*(H-PUCK);
   $('puck').style.top=y+'px';
   $('fill').style.height=(H-y)+'px';
   $('pct').textContent=Math.round(f*100)+'%';
   $('sm').textContent=Math.round(ema);
 }
 if(raw){
   let rf=d.x/XR; if($('flip').checked) rf=1-rf;
   rf=Math.max(0,Math.min(1,rf));
   $('ghost').style.display='block';
   $('ghost').style.top=(rf*(H-2))+'px';
   $('raw').textContent=d.x;
 } else {
   $('ghost').style.display='none';
   $('raw').textContent='-';
 }
 $('tch').textContent = raw ? 'down' : (down ? 'bridged' : 'up');
 $('zone').textContent=d.zone;
 $('acc').textContent=Math.round(acc);
 $('det').textContent=det;
 $('drops').textContent=drops;
 $('gap').textContent=Math.round(gapMax);
};
</script></body></html>
"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == "/":
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/slider":
            body = SLIDER_PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            last_seq = 0
            try:
                while True:
                    with state_lock:
                        fresh = [e for e in events if e["i"] > last_seq]
                        if fresh:
                            last_seq = fresh[-1]["i"]
                        payload = dict(state, events=fresh)
                    self.wfile.write(
                        f"data: {json.dumps(payload)}\n\n".encode())
                    self.wfile.flush()
                    time.sleep(0.04)
            except (BrokenPipeError, ConnectionResetError):
                return
        elif self.path.startswith("/cmd?c="):
            cmd = self.path.split("=", 1)[1][:4]
            ok = False
            if ser_handle is not None:
                try:
                    ser_handle.write(cmd.encode())
                    ok = True
                except (serial.SerialException, OSError):
                    pass
            body = b"ok" if ok else b"no-serial"
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_response(404)
            self.end_headers()


def main():
    threading.Thread(target=reader, daemon=True).start()
    srv = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
    print(f"serving http://localhost:{PORT}")
    srv.serve_forever()


if __name__ == "__main__":
    main()
