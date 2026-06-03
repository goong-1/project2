#!/usr/bin/env python3
"""
dashboard_node.py  (v4 — 트리플 피드 + 토글)
──────────────────────────────────────────────────────────────────
구독 토픽:
  /image_raw/compressed     BEST_EFFORT  ← 카메라 원본
  /image_yolo/compressed    BEST_EFFORT  ← YOLO 디버그
  /image_line/compressed    BEST_EFFORT  ← 차선 감지 디버그
  /traffic_sign_topic       RELIABLE
  /control_state            RELIABLE
  /vision_status            RELIABLE

실행:
  ros2 run p2_pkg dashboard_node
  브라우저: http://localhost:5000
──────────────────────────────────────────────────────────────────
"""

import threading
import time
import base64
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from flask import Flask, jsonify, render_template_string

HTML = r"""<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Robot Vision Dashboard</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link href="https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Rajdhani:wght@400;600;700&display=swap" rel="stylesheet">
<style>
:root {
  --bg:      #07090e;
  --surface: #0c1018;
  --card:    #101520;
  --border:  #1c2840;
  --bhi:     #2a3f60;
  --text:    #ccd8e8;
  --dim:     #3d5570;
  --g: #00e5a0;
  --b: #00aaff;
  --y: #ffbe00;
  --r: #ff3355;
  --mono: 'Share Tech Mono', monospace;
  --sans: 'Rajdhani', sans-serif;
}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);font-family:var(--sans);min-height:100vh;}

/* ── 헤더 ── */
header{
  display:flex;align-items:center;gap:14px;
  padding:9px 20px;
  background:var(--surface);
  border-bottom:1px solid var(--border);
}
.logo-mark{
  width:30px;height:30px;border:2px solid var(--g);border-radius:6px;
  display:grid;place-items:center;
}
.logo-mark::after{
  content:'';width:9px;height:9px;background:var(--g);border-radius:50%;
  box-shadow:0 0 7px var(--g);animation:pulse 2s ease-in-out infinite;
}
@keyframes pulse{0%,100%{transform:scale(1);opacity:1}50%{transform:scale(.6);opacity:.5}}
.logo-title{font-size:17px;font-weight:700;letter-spacing:.8px;color:#fff}
.logo-sub{font-family:var(--mono);font-size:10px;color:var(--dim)}
.badges{margin-left:auto;display:flex;gap:8px;align-items:center}
.badge{
  font-family:var(--mono);font-size:10px;
  padding:3px 9px;border-radius:20px;
  border:1px solid var(--bhi);
  display:flex;align-items:center;gap:5px;
}
.badge .dot{width:6px;height:6px;border-radius:50%;background:var(--dim)}
.badge.live{border-color:var(--g);color:var(--g)}
.badge.live .dot{background:var(--g);box-shadow:0 0 5px var(--g);animation:blink 1.2s step-end infinite}
.badge.warn{border-color:var(--y);color:var(--y)}
.badge.warn .dot{background:var(--y)}
.badge.dead{border-color:var(--r);color:var(--r)}
.badge.dead .dot{background:var(--r)}
@keyframes blink{50%{opacity:0}}

/* ── 토글 버튼 바 ── */
.toggle-bar{
  display:flex;gap:8px;
  padding:10px 20px;
  background:var(--surface);
  border-bottom:1px solid var(--border);
}
.tg-btn{
  padding:8px 16px;
  border-radius:8px;
  border:1px solid var(--bhi);
  background:transparent;
  color:var(--dim);
  font-family:var(--sans);font-size:13px;font-weight:600;
  letter-spacing:.4px;cursor:pointer;
  display:flex;align-items:center;gap:7px;
  transition:all .2s;
}
.tg-btn .tdot{width:8px;height:8px;border-radius:50%;background:var(--dim);transition:all .2s}
.tg-btn.on.raw {border-color:var(--g);color:var(--g)} .tg-btn.on.raw  .tdot{background:var(--g);box-shadow:0 0 6px var(--g)}
.tg-btn.on.yolo{border-color:var(--b);color:var(--b)} .tg-btn.on.yolo .tdot{background:var(--b);box-shadow:0 0 6px var(--b)}
.tg-btn.on.lane{border-color:var(--y);color:var(--y)} .tg-btn.on.lane .tdot{background:var(--y);box-shadow:0 0 6px var(--y)}
.tg-btn.off{opacity:.5}

/* ── 메인 ── */
.main{
  display:grid;
  grid-template-columns:1fr 260px;
  gap:12px;
  padding:14px 20px;
}
@media(max-width:1000px){.main{grid-template-columns:1fr}}

/* 피드 컨테이너 — 보이는 피드 수에 따라 자동 분할 */
.feeds{
  display:grid;
  gap:12px;
  align-content:start;
}
.feeds[data-count="1"]{grid-template-columns:1fr}
.feeds[data-count="2"]{grid-template-columns:1fr 1fr}
.feeds[data-count="3"]{grid-template-columns:1fr 1fr 1fr}
@media(max-width:1400px){
  .feeds[data-count="3"]{grid-template-columns:1fr 1fr}
}

/* ── 카드 ── */
.card{
  background:var(--card);border:1px solid var(--border);
  border-radius:10px;overflow:hidden;
}
.feed-card.hidden{display:none}
.card-hd{
  display:flex;align-items:center;justify-content:space-between;
  padding:7px 11px;border-bottom:1px solid var(--border);
}
.card-title{font-size:12px;font-weight:700;letter-spacing:.8px;text-transform:uppercase}
.card-meta{font-family:var(--mono);font-size:10px;color:var(--dim)}
.fps-tag{
  font-family:var(--mono);font-size:11px;
  padding:2px 6px;border-radius:4px;
  border:1px solid var(--bhi);
  min-width:54px;text-align:right;
}
.fps-raw {color:var(--g)}
.fps-yolo{color:var(--b)}
.fps-lane{color:var(--y)}

/* ── 피드 ── */
.feed-wrap{
  position:relative;background:#000;line-height:0;
}
.feed-wrap img{
  width:100%;height:auto;display:block;
}
.no-signal{
  width:100%;aspect-ratio:4/3;
  display:flex;flex-direction:column;align-items:center;justify-content:center;gap:8px;
  background:#050810;color:var(--dim);font-family:var(--mono);font-size:11px;
}
.no-signal svg{opacity:.2}
.scanline{
  pointer-events:none;position:absolute;inset:0;z-index:2;
  background:repeating-linear-gradient(0deg,transparent,transparent 2px,rgba(0,0,0,.06) 2px,rgba(0,0,0,.06) 4px);
}
.corner{pointer-events:none;position:absolute;width:13px;height:13px;z-index:3}
.corner.tl{top:0;left:0}  .corner.tr{top:0;right:0}
.corner.bl{bottom:0;left:0}.corner.br{bottom:0;right:0}
.raw-feed  .corner.tl,.raw-feed  .corner.bl{border-left:2px solid var(--g)}
.raw-feed  .corner.tl,.raw-feed  .corner.tr{border-top:2px solid var(--g)}
.raw-feed  .corner.br,.raw-feed  .corner.tr{border-right:2px solid var(--g)}
.raw-feed  .corner.br,.raw-feed  .corner.bl{border-bottom:2px solid var(--g)}
.yolo-feed .corner.tl,.yolo-feed .corner.bl{border-left:2px solid var(--b)}
.yolo-feed .corner.tl,.yolo-feed .corner.tr{border-top:2px solid var(--b)}
.yolo-feed .corner.br,.yolo-feed .corner.tr{border-right:2px solid var(--b)}
.yolo-feed .corner.br,.yolo-feed .corner.bl{border-bottom:2px solid var(--b)}
.lane-feed .corner.tl,.lane-feed .corner.bl{border-left:2px solid var(--y)}
.lane-feed .corner.tl,.lane-feed .corner.tr{border-top:2px solid var(--y)}
.lane-feed .corner.br,.lane-feed .corner.tr{border-right:2px solid var(--y)}
.lane-feed .corner.br,.lane-feed .corner.bl{border-bottom:2px solid var(--y)}

.feed-overlay{
  position:absolute;bottom:0;left:0;right:0;z-index:4;
  padding:4px 9px;
  background:linear-gradient(transparent,rgba(0,0,0,.8));
  display:flex;justify-content:space-between;align-items:flex-end;
}
.feed-lbl{font-family:var(--mono);font-size:10px;color:rgba(255,255,255,.35)}
.feed-age{font-family:var(--mono);font-size:10px;color:var(--g)}
.feed-age.stale{color:var(--y)}.feed-age.dead{color:var(--r)}

/* ── 사이드 ── */
.side{display:flex;flex-direction:column;gap:10px;}
.sign-wrap{padding:12px 11px 9px;text-align:center}
.sign-val{font-size:30px;font-weight:700;letter-spacing:2px;text-shadow:0 0 16px currentColor;transition:color .4s}
.sign-ts{font-family:var(--mono);font-size:10px;color:var(--dim);margin-top:3px}
.brain-row{padding:8px 11px;display:flex;flex-direction:column;gap:3px}
.bl{font-size:10px;color:var(--dim);letter-spacing:.4px}
.bv{font-family:var(--mono);font-size:13px;color:var(--b)}
.bc{font-family:var(--mono);font-size:12px;color:var(--y);opacity:.85}
.lane-detail{padding:8px 11px;display:flex;flex-direction:column;gap:5px}
.ld-row{display:flex;justify-content:space-between;align-items:center;padding:3px 0;border-bottom:1px solid var(--border)}
.ld-row:last-child{border:none}
.ld-key{font-size:11px;color:var(--dim)}
.ld-val{font-family:var(--mono);font-size:11px;color:var(--text)}
.stats{display:grid;grid-template-columns:1fr 1fr;gap:1px;background:var(--border)}
.stat{background:var(--card);padding:7px 10px}
.stat-l{font-size:10px;color:var(--dim);margin-bottom:2px}
.stat-v{font-family:var(--mono);font-size:12px}
.stat-v.ok{color:var(--g)}.stat-v.warn{color:var(--y)}.stat-v.dead{color:var(--r)}
.log-list{max-height:160px;overflow-y:auto;scrollbar-width:thin;scrollbar-color:var(--border) transparent}
.log-item{display:flex;align-items:center;gap:7px;padding:4px 11px;border-bottom:1px solid var(--border);animation:fi .3s ease}
@keyframes fi{from{opacity:0;transform:translateY(-3px)}}
.log-item:last-child{border:none}
.lt{font-family:var(--mono);font-size:10px;color:var(--dim);min-width:52px}
.lk{font-family:var(--mono);font-size:10px;font-weight:700;padding:1px 6px;border-radius:3px;min-width:66px;text-align:center}
.lk.GO         {background:#002215;color:var(--g);border:1px solid #004430}
.lk.STOP       {background:#250010;color:var(--r);border:1px solid #500030}
.lk.SPEED_LIMIT{background:#001525;color:var(--b);border:1px solid #003555}
.lk.TURN_LEFT  {background:#1a1000;color:var(--y);border:1px solid #554000}
.lk.UNKNOWN    {background:#111;color:var(--dim);border:1px solid #222}
</style>
</head>
<body>

<header>
  <div class="logo-mark"></div>
  <div>
    <div class="logo-title">Robot Vision Dashboard</div>
    <div class="logo-sub">Raspberry Pi + ROS2 · DDS Remote View</div>
  </div>
  <div class="badges">
    <div class="badge" id="bdg-raw"><div class="dot"></div>RAW</div>
    <div class="badge" id="bdg-yolo"><div class="dot"></div>YOLO</div>
    <div class="badge" id="bdg-lane"><div class="dot"></div>LANE</div>
    <div class="badge" id="bdg-brain"><div class="dot"></div>BRAIN</div>
  </div>
</header>

<!-- 토글 버튼 -->
<div class="toggle-bar">
  <button class="tg-btn raw on"  id="tg-raw"  onclick="toggleFeed('raw')">
    <span class="tdot"></span>📷 카메라 원본
  </button>
  <button class="tg-btn yolo on" id="tg-yolo" onclick="toggleFeed('yolo')">
    <span class="tdot"></span>🎯 YOLO 추적
  </button>
  <button class="tg-btn lane on" id="tg-lane" onclick="toggleFeed('lane')">
    <span class="tdot"></span>🛣️ 차선 감지
  </button>
</div>

<div class="main">

  <!-- 피드 그리드 -->
  <div class="feeds" id="feeds" data-count="3">

    <!-- 원본 -->
    <div class="card feed-card" id="card-raw">
      <div class="card-hd">
        <span class="card-title" style="color:var(--g)">📷 카메라 원본</span>
        <div style="display:flex;gap:7px;align-items:center">
          <span class="fps-tag fps-raw" id="fps-raw">-- fps</span>
        </div>
      </div>
      <div class="feed-wrap raw-feed">
        <div class="no-signal" id="raw-nosig">
          <svg width="36" height="36" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5">
            <rect x="2" y="2" width="20" height="16" rx="2"/><path d="M8 22h8M12 18v4"/>
            <line x1="2" y1="2" x2="22" y2="22" stroke-width="1.2"/>
          </svg><span>수신 대기중...</span>
        </div>
        <img id="raw-img" style="display:none" alt="raw">
        <div class="scanline"></div>
        <div class="corner tl"></div><div class="corner tr"></div>
        <div class="corner bl"></div><div class="corner br"></div>
        <div class="feed-overlay">
          <span class="feed-lbl">/image_raw/compressed</span>
          <span class="feed-age" id="age-raw">--</span>
        </div>
      </div>
    </div>

    <!-- YOLO -->
    <div class="card feed-card" id="card-yolo">
      <div class="card-hd">
        <span class="card-title" style="color:var(--b)">🎯 YOLO 추적</span>
        <div style="display:flex;gap:7px;align-items:center">
          <span class="fps-tag fps-yolo" id="fps-yolo">-- fps</span>
        </div>
      </div>
      <div class="feed-wrap yolo-feed">
        <div class="no-signal" id="yolo-nosig">
          <svg width="36" height="36" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5">
            <rect x="2" y="2" width="20" height="16" rx="2"/><path d="M8 22h8M12 18v4"/>
            <line x1="2" y1="2" x2="22" y2="22" stroke-width="1.2"/>
          </svg><span>수신 대기중...</span>
        </div>
        <img id="yolo-img" style="display:none" alt="yolo">
        <div class="scanline"></div>
        <div class="corner tl"></div><div class="corner tr"></div>
        <div class="corner bl"></div><div class="corner br"></div>
        <div class="feed-overlay">
          <span class="feed-lbl">/image_yolo/compressed</span>
          <span class="feed-age" id="age-yolo">--</span>
        </div>
      </div>
    </div>

    <!-- 차선 -->
    <div class="card feed-card" id="card-lane">
      <div class="card-hd">
        <span class="card-title" style="color:var(--y)">🛣️ 차선 감지</span>
        <div style="display:flex;gap:7px;align-items:center">
          <span class="fps-tag fps-lane" id="fps-lane">-- fps</span>
        </div>
      </div>
      <div class="feed-wrap lane-feed">
        <div class="no-signal" id="lane-nosig">
          <svg width="36" height="36" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5">
            <rect x="2" y="2" width="20" height="16" rx="2"/><path d="M8 22h8M12 18v4"/>
            <line x1="2" y1="2" x2="22" y2="22" stroke-width="1.2"/>
          </svg><span>수신 대기중...</span>
        </div>
        <img id="lane-img" style="display:none" alt="lane">
        <div class="scanline"></div>
        <div class="corner tl"></div><div class="corner tr"></div>
        <div class="corner bl"></div><div class="corner br"></div>
        <div class="feed-overlay">
          <span class="feed-lbl">/image_line/compressed</span>
          <span class="feed-age" id="age-lane">--</span>
        </div>
      </div>
    </div>

  </div>

  <!-- 사이드 -->
  <div class="side">
    <div class="card">
      <div class="card-hd"><span class="card-title">Traffic Sign</span><span class="card-meta">/traffic_sign_topic</span></div>
      <div class="sign-wrap">
        <div class="sign-val" id="sign-val">UNKNOWN</div>
        <div class="sign-ts"  id="sign-ts">수신 없음</div>
      </div>
    </div>
    <div class="card">
      <div class="card-hd"><span class="card-title">Brain FSM</span><span class="card-meta">/control_state</span></div>
      <div class="brain-row"><div class="bl">현재 상태</div><div class="bv" id="brain-state">--</div></div>
      <div class="brain-row" style="border-top:1px solid var(--border)"><div class="bl">마지막 명령 → ESP32</div><div class="bc" id="brain-cmd">--</div></div>
    </div>
    <div class="card">
      <div class="card-hd"><span class="card-title" style="color:var(--y)">차선 상세</span><span class="card-meta">/vision_status</span></div>
      <div class="lane-detail">
        <div class="ld-row"><span class="ld-key">Error</span><span class="ld-val" id="ld-error">--</span></div>
        <div class="ld-row"><span class="ld-key">Red Line</span><span class="ld-val" id="ld-red">--</span></div>
        <div class="ld-row"><span class="ld-key">Obstacle</span><span class="ld-val" id="ld-obs">--</span></div>
        <div class="ld-row"><span class="ld-key">Crosswalk</span><span class="ld-val" id="ld-cross">--</span></div>
      </div>
    </div>
    <div class="card">
      <div class="card-hd"><span class="card-title">Connection</span></div>
      <div class="stats">
        <div class="stat"><div class="stat-l">Raw FPS</div><div class="stat-v" id="s-rfps">--</div></div>
        <div class="stat"><div class="stat-l">YOLO FPS</div><div class="stat-v" id="s-yfps">--</div></div>
        <div class="stat"><div class="stat-l">Lane FPS</div><div class="stat-v" id="s-lfps">--</div></div>
        <div class="stat"><div class="stat-l">Brain 수신</div><div class="stat-v" id="s-bage">--</div></div>
      </div>
    </div>
    <div class="card">
      <div class="card-hd"><span class="card-title">Sign History</span><span class="card-meta" id="log-cnt">0 events</span></div>
      <div class="log-list" id="log-list">
        <div style="padding:12px;font-family:var(--mono);font-size:10px;color:var(--dim);text-align:center">수신된 신호 없음</div>
      </div>
    </div>
  </div>
</div>

<script>
const SIGN_COLORS={GO:'var(--g)',STOP:'var(--r)',SPEED_LIMIT:'var(--b)',TURN_LEFT:'var(--y)',UNKNOWN:'var(--dim)'};
const visible={raw:true,yolo:true,lane:true};
let prev={raw:null,yolo:null,lane:null}, lastLogLen=0;

function toggleFeed(k){
  visible[k]=!visible[k];
  document.getElementById('card-'+k).classList.toggle('hidden',!visible[k]);
  const btn=document.getElementById('tg-'+k);
  btn.className='tg-btn '+k+(visible[k]?' on':' off');
  // 보이는 개수로 그리드 분할 갱신
  const cnt=Object.values(visible).filter(Boolean).length;
  document.getElementById('feeds').setAttribute('data-count', cnt||1);
}

function ageText(a){ if(a<0)return'수신 없음'; if(a<2)return`${(a*1000).toFixed(0)} ms 전`; return`${a.toFixed(1)} s 전 ⚠`; }
function ageClass(a){ return a<0?'dead':a<2?'':a<5?'stale':'dead'; }
function badgeClass(a){ return a<0?'dead':a<2?'live':a<5?'warn':'dead'; }
function scls(v){ return v>=4?'stat-v ok':v>=1?'stat-v warn':'stat-v dead'; }

async function pollFrames(){
  try{
    const d=await(await fetch('/api/frames')).json();
    [['raw',d.raw_b64],['yolo',d.yolo_b64],['lane',d.line_b64]].forEach(([k,b64])=>{
      if(!visible[k]||!b64) return;
      const s='data:image/jpeg;base64,'+b64;
      if(s!==prev[k]){
        const img=document.getElementById(k+'-img');
        img.src=s;img.style.display='block';
        document.getElementById(k+'-nosig').style.display='none';
        prev[k]=s;
      }
    });
  }catch(e){}
  setTimeout(pollFrames,70);
}

async function pollStatus(){
  try{
    const d=await(await fetch('/api/status')).json();

    document.getElementById('fps-raw').textContent =`${d.raw_fps.toFixed(1)} fps`;
    document.getElementById('fps-yolo').textContent=`${d.yolo_fps.toFixed(1)} fps`;
    document.getElementById('fps-lane').textContent=`${d.line_fps.toFixed(1)} fps`;

    const setAge=(id,a)=>{const e=document.getElementById(id);e.textContent=ageText(a);e.className='feed-age '+ageClass(a)};
    setAge('age-raw',d.raw_age);setAge('age-yolo',d.yolo_age);setAge('age-lane',d.line_age);

    document.getElementById('bdg-raw').className  ='badge '+badgeClass(d.raw_age);
    document.getElementById('bdg-yolo').className ='badge '+badgeClass(d.yolo_age);
    document.getElementById('bdg-lane').className ='badge '+badgeClass(d.line_age);
    document.getElementById('bdg-brain').className='badge '+badgeClass(d.brain_age);

    const sv=(id,v)=>{const e=document.getElementById(id);e.textContent=v.toFixed(1);e.className=scls(v)};
    sv('s-rfps',d.raw_fps);sv('s-yfps',d.yolo_fps);sv('s-lfps',d.line_fps);
    const be=document.getElementById('s-bage');be.textContent=ageText(d.brain_age);be.className='stat-v '+ageClass(d.brain_age);

    const sval=document.getElementById('sign-val');
    sval.textContent=d.sign||'UNKNOWN';
    sval.style.color=SIGN_COLORS[d.sign]||SIGN_COLORS['UNKNOWN'];
    document.getElementById('sign-ts').textContent=d.sign_age>=0?ageText(d.sign_age):'수신 없음';

    document.getElementById('brain-state').textContent=d.brain_state||'--';
    document.getElementById('brain-cmd').textContent  =d.brain_cmd  ||'--';

    if(d.lane){
      document.getElementById('ld-error').textContent =d.lane.error.toFixed(1);
      document.getElementById('ld-red').textContent   =d.lane.red_line ?'🔴 감지':'⬜ 없음';
      document.getElementById('ld-obs').textContent   =d.lane.obstacle ?'⚠️ 있음':'✅ 없음';
      document.getElementById('ld-cross').textContent =d.lane.crosswalk?'🚶 감지':'⬜ 없음';
    }

    if(d.sign_log&&d.sign_log.length!==lastLogLen){
      lastLogLen=d.sign_log.length;
      const list=document.getElementById('log-list');
      list.innerHTML='';
      d.sign_log.slice().reverse().forEach(e=>{
        const row=document.createElement('div');row.className='log-item';
        row.innerHTML=`<span class="lt">${e.time}</span><span class="lk ${e.sign}">${e.sign}</span><span style="font-family:var(--mono);font-size:10px;color:var(--dim)">${e.elapsed}</span>`;
        list.appendChild(row);
      });
      document.getElementById('log-cnt').textContent=`${d.sign_log.length} events`;
    }
  }catch(e){}
  setTimeout(pollStatus,300);
}

pollFrames();
pollStatus();
</script>
</body>
</html>
"""

# ═══════════════════════════════════════════════════════════════
#  ROS2 노드
# ═══════════════════════════════════════════════════════════════
class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')
        self._lock = threading.Lock()

        self._raw_jpeg:  bytes | None = None
        self._yolo_jpeg: bytes | None = None
        self._line_jpeg: bytes | None = None

        self._raw_t   = -1.0
        self._yolo_t  = -1.0
        self._line_t  = -1.0
        self._sign_t  = -1.0
        self._brain_t = -1.0

        self._sign        = "UNKNOWN"
        self._brain_state = "--"
        self._brain_cmd   = "--"
        self._lane = {"error": 0.0, "red_line": False, "obstacle": False, "crosswalk": False}

        self._sign_log: deque = deque(maxlen=50)
        self._last_sign = None

        self._raw_buf:  deque = deque(maxlen=60)
        self._yolo_buf: deque = deque(maxlen=60)
        self._line_buf: deque = deque(maxlen=60)

        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(CompressedImage, 'image_raw/compressed',  self._cb_raw,    be)
        self.create_subscription(CompressedImage, 'image_yolo/compressed', self._cb_yolo,   be)
        self.create_subscription(CompressedImage, 'image_line/compressed', self._cb_line,   be)
        self.create_subscription(String, 'traffic_sign_topic',             self._cb_sign,   10)
        self.create_subscription(String, '/control_state',                 self._cb_brain,  10)
        self.create_subscription(String, '/vision_status',                 self._cb_vision, 10)

        self.get_logger().info('Dashboard Node 시작 → http://0.0.0.0:5000')

    def _cb_raw(self, msg):
        now = time.time()
        with self._lock:
            self._raw_jpeg = bytes(msg.data); self._raw_buf.append(now); self._raw_t = now

    def _cb_yolo(self, msg):
        now = time.time()
        with self._lock:
            self._yolo_jpeg = bytes(msg.data); self._yolo_buf.append(now); self._yolo_t = now

    def _cb_line(self, msg):
        now = time.time()
        with self._lock:
            self._line_jpeg = bytes(msg.data); self._line_buf.append(now); self._line_t = now

    def _cb_sign(self, msg):
        now = time.time(); sign = msg.data
        with self._lock:
            self._sign = sign; self._sign_t = now
            if sign != self._last_sign:
                self._last_sign = sign
                self._sign_log.append({"sign": sign, "time": time.strftime('%H:%M:%S'),
                                       "elapsed": "방금", "_ts": now})

    def _cb_brain(self, msg):
        now = time.time(); parts = msg.data.split('|', 1)
        with self._lock:
            self._brain_state = parts[0] if parts else "--"
            self._brain_cmd   = parts[1] if len(parts) > 1 else "--"
            self._brain_t     = now

    def _cb_vision(self, msg):
        try:
            p = msg.data.split('|')
            with self._lock:
                self._lane = {"error": float(p[0]), "red_line": bool(int(p[1])),
                              "obstacle": bool(int(p[2])), "crosswalk": bool(int(p[3]))}
        except Exception:
            pass

    def get_frames(self):
        with self._lock:
            return self._raw_jpeg, self._yolo_jpeg, self._line_jpeg

    def get_status(self):
        now = time.time()
        with self._lock:
            def age(t): return round(now - t, 3) if t > 0 else -1
            def fps(buf):
                if len(buf) < 2: return 0.0
                span = buf[-1] - buf[0]
                return (len(buf) - 1) / span if span > 0 else 0.0
            log_copy = []
            for e in self._sign_log:
                e2 = dict(e); a = now - e["_ts"]
                e2["elapsed"] = f"{a:.0f}s 전" if a < 60 else f"{a/60:.0f}m 전"
                log_copy.append(e2)
            return {
                "raw_fps": fps(self._raw_buf), "yolo_fps": fps(self._yolo_buf), "line_fps": fps(self._line_buf),
                "raw_age": age(self._raw_t), "yolo_age": age(self._yolo_t), "line_age": age(self._line_t),
                "sign": self._sign, "sign_age": age(self._sign_t),
                "brain_state": self._brain_state, "brain_cmd": self._brain_cmd, "brain_age": age(self._brain_t),
                "lane": dict(self._lane), "sign_log": log_copy,
            }


def create_app(node: DashboardNode) -> Flask:
    app = Flask(__name__)

    @app.route('/')
    def index(): return render_template_string(HTML)

    @app.route('/api/frames')
    def api_frames():
        raw, yolo, line = node.get_frames()
        return jsonify({
            "raw_b64":  base64.b64encode(raw).decode()  if raw  else None,
            "yolo_b64": base64.b64encode(yolo).decode() if yolo else None,
            "line_b64": base64.b64encode(line).decode() if line else None,
        })

    @app.route('/api/status')
    def api_status(): return jsonify(node.get_status())

    return app


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    app  = create_app(node)
    threading.Thread(
        target=lambda: app.run(host='0.0.0.0', port=5000,
                               debug=False, use_reloader=False, threaded=True),
        daemon=True
    ).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()