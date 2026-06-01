#!/usr/bin/env python3
"""
pc_dashboard_node.py  (v2 — 모드 전환 뷰어)
──────────────────────────────────────────────────────────────────
구독 토픽:
  /image_raw/compressed     BEST_EFFORT  ← 카메라 원본
  /image_yolo/compressed    BEST_EFFORT  ← YOLO 또는 차선 디버그
  /traffic_sign_topic       RELIABLE
  /control_state            RELIABLE     ← brain_node FSM

실행:
  export ROS_DOMAIN_ID=0
  python3 pc_dashboard_node.py
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

from flask import Flask, Response, jsonify, render_template_string

# ═══════════════════════════════════════════════════════════════
#  HTML
# ═══════════════════════════════════════════════════════════════
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
  --bg:       #07090e;
  --surface:  #0c1018;
  --card:     #101520;
  --border:   #1c2840;
  --border-hi:#2a3f60;
  --text:     #ccd8e8;
  --dim:      #3d5570;
  --g:  #00e5a0;  /* green  */
  --b:  #00aaff;  /* blue   */
  --y:  #ffbe00;  /* yellow */
  --r:  #ff3355;  /* red    */
  --p:  #c87cff;  /* purple */
  --mono: 'Share Tech Mono', monospace;
  --sans: 'Rajdhani', sans-serif;
}
*{box-sizing:border-box;margin:0;padding:0}
html,body{height:100%;overflow:hidden}
body{background:var(--bg);color:var(--text);font-family:var(--sans);
  display:flex;flex-direction:column;height:100vh;}

/* ── 헤더 ── */
header{
  display:flex;align-items:center;gap:16px;
  padding:10px 24px;
  background:var(--surface);
  border-bottom:1px solid var(--border);
  flex-shrink:0;
}
.logo-mark{
  width:32px;height:32px;border:2px solid var(--g);border-radius:6px;
  display:grid;place-items:center;
}
.logo-mark::after{
  content:'';width:10px;height:10px;background:var(--g);border-radius:50%;
  box-shadow:0 0 8px var(--g);animation:pulse 2s ease-in-out infinite;
}
@keyframes pulse{0%,100%{transform:scale(1);opacity:1}50%{transform:scale(.65);opacity:.5}}
.logo-title{font-size:18px;font-weight:700;letter-spacing:.8px;color:#fff}
.logo-sub{font-family:var(--mono);font-size:10px;color:var(--dim)}

.badges{margin-left:auto;display:flex;gap:10px;align-items:center}
.badge{
  font-family:var(--mono);font-size:10px;
  padding:3px 9px;border-radius:20px;
  border:1px solid var(--border-hi);
  display:flex;align-items:center;gap:5px;
  transition:border-color .3s,color .3s;
}
.badge .dot{width:6px;height:6px;border-radius:50%;background:var(--dim);transition:background .3s,box-shadow .3s}
.badge.live{border-color:var(--g);color:var(--g)}
.badge.live .dot{background:var(--g);box-shadow:0 0 5px var(--g);animation:blink 1.2s step-end infinite}
.badge.warn{border-color:var(--y);color:var(--y)}
.badge.warn .dot{background:var(--y)}
.badge.dead{border-color:var(--r);color:var(--r)}
.badge.dead .dot{background:var(--r)}
@keyframes blink{50%{opacity:0}}

/* ── 모드 버튼 ── */
.mode-bar{
  display:flex;gap:8px;
  padding:8px 24px;
  background:var(--surface);
  border-bottom:1px solid var(--border);
  flex-shrink:0;
}
.mode-btn{
  flex:1;max-width:220px;
  padding:10px 0;
  border-radius:8px;
  border:1px solid var(--border-hi);
  background:transparent;
  color:var(--dim);
  font-family:var(--sans);font-size:14px;font-weight:600;
  letter-spacing:.5px;
  cursor:pointer;
  display:flex;align-items:center;justify-content:center;gap:8px;
  transition:all .2s;
}
.mode-btn:hover{border-color:var(--border-hi);color:var(--text);background:#151d2a}
.mode-btn.active.raw   {border-color:var(--g); color:var(--g); background:#001a10;box-shadow:0 0 12px #00e5a020}
.mode-btn.active.yolo  {border-color:var(--b); color:var(--b); background:#001525;box-shadow:0 0 12px #00aaff20}
.mode-btn.active.lane  {border-color:var(--y); color:var(--y); background:#1a1200;box-shadow:0 0 12px #ffbe0020}
.mode-icon{font-size:16px}

/* ── 메인 레이아웃 ── */
.main{
  display:grid;
  grid-template-columns:1fr 280px;
  gap:12px;
  padding:12px 20px;
  flex:1;
  min-height:0;
  overflow:hidden;
}
@media(max-width:900px){.main{grid-template-columns:1fr;overflow-y:auto}}

/* ── 카드 ── */
.card{
  background:var(--card);
  border:1px solid var(--border);
  border-radius:10px;
  overflow:hidden;
}
/* 피드 카드는 남은 높이 전부 채움 */
.feed-card{
  display:flex;flex-direction:column;
  height:100%;min-height:0;
}
.card-hd{
  display:flex;align-items:center;justify-content:space-between;
  padding:8px 12px;
  border-bottom:1px solid var(--border);
  flex-shrink:0;
}
.card-title{font-size:12px;font-weight:700;letter-spacing:.8px;text-transform:uppercase}
.card-meta{font-family:var(--mono);font-size:10px;color:var(--dim)}
.fps-tag{
  font-family:var(--mono);font-size:11px;
  padding:2px 7px;border-radius:4px;
  border:1px solid var(--border-hi);color:var(--b);
  min-width:56px;text-align:right;
}

/* ── 피드 ── */
.feed-wrap{
  position:relative;background:#000;line-height:0;
  flex:1;min-height:0;
  display:flex;align-items:center;justify-content:center;
  overflow:hidden;
}
.feed-wrap img{
  max-width:100%;max-height:100%;
  width:auto;height:auto;
  object-fit:contain;display:block;
}
.no-signal{
  width:100%;height:100%;
  display:flex;flex-direction:column;align-items:center;justify-content:center;gap:8px;
  background:#050810;color:var(--dim);font-family:var(--mono);font-size:11px;
}
.no-signal svg{opacity:.25}
.scanline{
  pointer-events:none;position:absolute;inset:0;z-index:2;
  background:repeating-linear-gradient(0deg,transparent,transparent 2px,rgba(0,0,0,.06) 2px,rgba(0,0,0,.06) 4px);
}
.corner{pointer-events:none;position:absolute;width:14px;height:14px;z-index:3}
.corner.tl{top:0;left:0}   .corner.tr{top:0;right:0}
.corner.bl{bottom:0;left:0}.corner.br{bottom:0;right:0}

.feed-overlay{
  position:absolute;bottom:0;left:0;right:0;z-index:4;
  padding:5px 10px;
  background:linear-gradient(transparent,rgba(0,0,0,.8));
  display:flex;justify-content:space-between;align-items:flex-end;
}
.feed-label{font-family:var(--mono);font-size:10px;color:rgba(255,255,255,.4)}
.feed-age{font-family:var(--mono);font-size:10px;color:var(--g)}
.feed-age.stale{color:var(--y)}.feed-age.dead{color:var(--r)}

/* 모드별 테두리 색 */
.feed-raw  .corner{border-color:var(--g)}
.feed-yolo .corner{border-color:var(--b)}
.feed-lane .corner{border-color:var(--y)}
.feed-raw  .corner.tl,.feed-raw  .corner.bl{border-left:2px solid var(--g)}
.feed-raw  .corner.tl,.feed-raw  .corner.tr{border-top:2px solid var(--g)}
.feed-raw  .corner.br,.feed-raw  .corner.tr{border-right:2px solid var(--g)}
.feed-raw  .corner.br,.feed-raw  .corner.bl{border-bottom:2px solid var(--g)}
.feed-yolo .corner.tl,.feed-yolo .corner.bl{border-left:2px solid var(--b)}
.feed-yolo .corner.tl,.feed-yolo .corner.tr{border-top:2px solid var(--b)}
.feed-yolo .corner.br,.feed-yolo .corner.tr{border-right:2px solid var(--b)}
.feed-yolo .corner.br,.feed-yolo .corner.bl{border-bottom:2px solid var(--b)}
.feed-lane .corner.tl,.feed-lane .corner.bl{border-left:2px solid var(--y)}
.feed-lane .corner.tl,.feed-lane .corner.tr{border-top:2px solid var(--y)}
.feed-lane .corner.br,.feed-lane .corner.tr{border-right:2px solid var(--y)}
.feed-lane .corner.br,.feed-lane .corner.bl{border-bottom:2px solid var(--y)}

/* ── 사이드 패널 ── */
.side{display:flex;flex-direction:column;gap:12px;overflow-y:auto;min-height:0;padding-bottom:4px;}

/* sign */
.sign-wrap{padding:14px 12px 10px;text-align:center}
.sign-val{font-size:34px;font-weight:700;letter-spacing:2px;text-shadow:0 0 18px currentColor;transition:color .4s,text-shadow .4s}
.sign-ts{font-family:var(--mono);font-size:10px;color:var(--dim);margin-top:3px}

/* brain */
.brain-row{padding:9px 12px;display:flex;flex-direction:column;gap:4px}
.bl{font-size:10px;color:var(--dim);letter-spacing:.4px}
.bv{font-family:var(--mono);font-size:14px;color:var(--b)}
.bc{font-family:var(--mono);font-size:12px;color:var(--y);opacity:.85}

/* 차선 상세 (차선 모드일 때만 표시) */
.lane-detail{padding:9px 12px;display:none;flex-direction:column;gap:6px}
.lane-detail.show{display:flex}
.ld-row{display:flex;justify-content:space-between;align-items:center;padding:4px 0;border-bottom:1px solid var(--border)}
.ld-row:last-child{border:none}
.ld-key{font-size:11px;color:var(--dim)}
.ld-val{font-family:var(--mono);font-size:12px;color:var(--text)}

/* stats */
.stats{display:grid;grid-template-columns:1fr 1fr;gap:1px;background:var(--border)}
.stat{background:var(--card);padding:8px 10px}
.stat-l{font-size:10px;color:var(--dim);margin-bottom:3px}
.stat-v{font-family:var(--mono);font-size:13px}
.stat-v.ok{color:var(--g)}.stat-v.warn{color:var(--y)}.stat-v.dead{color:var(--r)}

/* 로그 */
.log-list{max-height:180px;overflow-y:auto;scrollbar-width:thin;scrollbar-color:var(--border) transparent}
.log-item{display:flex;align-items:center;gap:8px;padding:5px 12px;border-bottom:1px solid var(--border);animation:fi .3s ease}
@keyframes fi{from{opacity:0;transform:translateY(-3px)}}
.log-item:last-child{border:none}
.lt{font-family:var(--mono);font-size:10px;color:var(--dim);min-width:55px}
.lk{font-family:var(--mono);font-size:10px;font-weight:700;padding:1px 6px;border-radius:3px;min-width:68px;text-align:center}
.lk.GO         {background:#002215;color:var(--g);border:1px solid #004430}
.lk.STOP       {background:#250010;color:var(--r);border:1px solid #500030}
.lk.SPEED_LIMIT{background:#001525;color:var(--b);border:1px solid #003555}
.lk.TURN_LEFT  {background:#1a1000;color:var(--y);border:1px solid #554000}
.lk.UNKNOWN    {background:#111;color:var(--dim);border:1px solid #222}
</style>
</head>
<body>

<!-- 헤더 -->
<header>
  <div class="logo-mark"></div>
  <div>
    <div class="logo-title">Robot Vision Dashboard</div>
    <div class="logo-sub">Raspberry Pi + ROS2 · DDS Remote View</div>
  </div>
  <div class="badges">
    <div class="badge" id="bdg-raw"><div class="dot"></div>RAW</div>
    <div class="badge" id="bdg-proc"><div class="dot"></div>PROC</div>
    <div class="badge" id="bdg-brain"><div class="dot"></div>BRAIN</div>
  </div>
</header>

<!-- 모드 버튼 -->
<div class="mode-bar">
  <button class="mode-btn raw active" id="btn-raw"   onclick="setMode('raw')">
    <span class="mode-icon">📷</span> 카메라 원본
  </button>
  <button class="mode-btn yolo" id="btn-yolo" onclick="setMode('yolo')">
    <span class="mode-icon">🎯</span> YOLO 추적
  </button>
  <button class="mode-btn lane" id="btn-lane" onclick="setMode('lane')">
    <span class="mode-icon">🛣️</span> 차선 감지
  </button>
</div>

<!-- 메인 -->
<div class="main">

  <!-- 피드 카드 -->
  <div class="card feed-card">
    <div class="card-hd">
      <span class="card-title" id="feed-title">카메라 원본</span>
      <div style="display:flex;gap:8px;align-items:center">
        <span class="card-meta" id="feed-topic">/image_raw/compressed</span>
        <span class="fps-tag" id="fps-val">-- fps</span>
      </div>
    </div>
    <div class="feed-wrap feed-raw" id="feed-wrap">
      <div class="no-signal" id="feed-nosig">
        <svg width="38" height="38" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5">
          <rect x="2" y="2" width="20" height="16" rx="2"/>
          <path d="M8 22h8M12 18v4"/>
          <line x1="2" y1="2" x2="22" y2="22" stroke-width="1.2"/>
        </svg>
        <span>수신 대기중...</span>
      </div>
      <img id="feed-img" style="display:none" alt="feed">
      <div class="scanline"></div>
      <div class="corner tl"></div><div class="corner tr"></div>
      <div class="corner bl"></div><div class="corner br"></div>
      <div class="feed-overlay">
        <span class="feed-label" id="feed-lbl">/image_raw/compressed</span>
        <span class="feed-age" id="feed-age">--</span>
      </div>
    </div>
  </div>

  <!-- 사이드 -->
  <div class="side">

    <!-- Traffic Sign -->
    <div class="card">
      <div class="card-hd">
        <span class="card-title">Traffic Sign</span>
        <span class="card-meta">/traffic_sign_topic</span>
      </div>
      <div class="sign-wrap">
        <div class="sign-val" id="sign-val">UNKNOWN</div>
        <div class="sign-ts"  id="sign-ts">수신 없음</div>
      </div>
    </div>

    <!-- Brain FSM -->
    <div class="card">
      <div class="card-hd">
        <span class="card-title">Brain FSM</span>
        <span class="card-meta">/control_state</span>
      </div>
      <div class="brain-row">
        <div class="bl">현재 상태</div>
        <div class="bv" id="brain-state">--</div>
      </div>
      <div class="brain-row" style="border-top:1px solid var(--border)">
        <div class="bl">마지막 명령 → ESP32</div>
        <div class="bc" id="brain-cmd">--</div>
      </div>
      <!-- 차선 모드 상세 -->
      <div class="lane-detail" id="lane-detail">
        <div class="ld-row"><span class="ld-key">Error</span><span class="ld-val" id="ld-error">--</span></div>
        <div class="ld-row"><span class="ld-key">Red Line</span><span class="ld-val" id="ld-red">--</span></div>
        <div class="ld-row"><span class="ld-key">Obstacle Front</span><span class="ld-val" id="ld-obs">--</span></div>
        <div class="ld-row"><span class="ld-key">Crosswalk</span><span class="ld-val" id="ld-cross">--</span></div>
      </div>
    </div>

    <!-- 연결 통계 -->
    <div class="card">
      <div class="card-hd"><span class="card-title">Connection</span></div>
      <div class="stats">
        <div class="stat"><div class="stat-l">Raw FPS</div><div class="stat-v" id="s-rfps">--</div></div>
        <div class="stat"><div class="stat-l">Proc FPS</div><div class="stat-v" id="s-pfps">--</div></div>
        <div class="stat"><div class="stat-l">Raw 수신</div><div class="stat-v" id="s-rage">--</div></div>
        <div class="stat"><div class="stat-l">Proc 수신</div><div class="stat-v" id="s-page">--</div></div>
      </div>
    </div>

    <!-- 신호 히스토리 -->
    <div class="card">
      <div class="card-hd">
        <span class="card-title">Sign History</span>
        <span class="card-meta" id="log-cnt">0 events</span>
      </div>
      <div class="log-list" id="log-list">
        <div style="padding:12px;font-family:var(--mono);font-size:10px;color:var(--dim);text-align:center">
          수신된 신호 없음
        </div>
      </div>
    </div>

  </div><!-- /side -->
</div><!-- /main -->

<script>
const SIGN_COLORS={GO:'var(--g)',STOP:'var(--r)',SPEED_LIMIT:'var(--b)',TURN_LEFT:'var(--y)',UNKNOWN:'var(--dim)'};
const MODE_META={
  raw: {title:'카메라 원본', topic:'/image_raw/compressed',  cls:'feed-raw',  fps:'raw_fps',  age:'raw_age',  key:'raw_b64'},
  yolo:{title:'YOLO 추적',   topic:'/image_yolo/compressed', cls:'feed-yolo', fps:'yolo_fps', age:'yolo_age', key:'yolo_b64'},
  lane:{title:'차선 감지',   topic:'/image_line/compressed', cls:'feed-lane', fps:'line_fps', age:'line_age', key:'line_b64'},
};

let mode = 'raw';
let prevSrc = null;
let lastLogLen = 0;

function setMode(m){
  mode = m;
  // 버튼 active
  ['raw','yolo','lane'].forEach(k=>{
    const btn = document.getElementById('btn-'+k);
    btn.className = 'mode-btn '+k+(k===m?' active':'');
  });
  // 피드 카드 메타
  const meta = MODE_META[m];
  document.getElementById('feed-title').textContent = meta.title;
  document.getElementById('feed-topic').textContent = meta.topic;
  document.getElementById('feed-lbl').textContent   = meta.topic;
  // 피드 래퍼 클래스
  const wrap = document.getElementById('feed-wrap');
  wrap.className = 'feed-wrap '+meta.cls;
  // 차선 상세 패널
  document.getElementById('lane-detail').className = 'lane-detail'+(m==='lane'?' show':'');
  prevSrc = null;
}

function ageText(a){
  if(a<0)   return '수신 없음';
  if(a<2)   return `${(a*1000).toFixed(0)} ms 전`;
  return `${a.toFixed(1)} s 전 ⚠`;
}
function ageClass(a){
  if(a<0)   return 'dead';
  if(a<2)   return '';
  if(a<5)   return 'stale';
  return 'dead';
}
function badgeClass(a){
  if(a<0)   return 'dead';
  if(a<2)   return 'live';
  if(a<5)   return 'warn';
  return 'dead';
}
function statClass(v,thr1,thr2){return v>=thr1?'stat-v ok':v>=thr2?'stat-v warn':'stat-v dead'}

// ── 이미지 폴링 ─────────────────────────────────────────────
async function pollFrames(){
  try{
    const res = await fetch('/api/frames');
    const d   = await res.json();
    const src_b64 = d[MODE_META[mode].key];
    if(src_b64){
      const src='data:image/jpeg;base64,'+src_b64;
      if(src!==prevSrc){
        const img=document.getElementById('feed-img');
        img.src=src; img.style.display='block';
        document.getElementById('feed-nosig').style.display='none';
        prevSrc=src;
      }
    }
  }catch(e){}
  setTimeout(pollFrames,80);
}

// ── 상태 폴링 ────────────────────────────────────────────────
async function pollStatus(){
  try{
    const res=await fetch('/api/status');
    const d  =await res.json();
    const meta=MODE_META[mode];
    const fps =d[meta.fps], age=d[meta.age];

    // fps 배지
    document.getElementById('fps-val').textContent=`${fps.toFixed(1)} fps`;
    // 수신 나이
    const ael=document.getElementById('feed-age');
    ael.textContent=ageText(age); ael.className='feed-age '+ageClass(age);

    // 헤더 배지
    document.getElementById('bdg-raw').className  ='badge '+badgeClass(d.raw_age);
    document.getElementById('bdg-proc').className ='badge '+badgeClass(d.proc_age);
    document.getElementById('bdg-brain').className='badge '+badgeClass(d.brain_age);

    // 통계
    const sf=(id,v)=>{const el=document.getElementById(id);el.textContent=v.toFixed(1);el.className=statClass(v,4,1)};
    sf('s-rfps',d.raw_fps);
    sf('s-pfps', mode==='lane' ? d.line_fps : d.yolo_fps);
    const sa=(id,a)=>{const el=document.getElementById(id);el.textContent=ageText(a);el.className='stat-v '+ageClass(a)};
    sa('s-rage',d.raw_age);
    sa('s-page', mode==='lane' ? d.line_age : d.yolo_age);

    // Sign
    const sv=document.getElementById('sign-val');
    sv.textContent=d.sign||'UNKNOWN';
    sv.style.color=SIGN_COLORS[d.sign]||SIGN_COLORS['UNKNOWN'];
    document.getElementById('sign-ts').textContent=d.sign_age>=0?ageText(d.sign_age):'수신 없음';

    // Brain
    document.getElementById('brain-state').textContent=d.brain_state||'--';
    document.getElementById('brain-cmd').textContent  =d.brain_cmd  ||'--';

    // 차선 상세 (lane 모드)
    if(mode==='lane' && d.lane){
      document.getElementById('ld-error').textContent =d.lane.error.toFixed(1);
      document.getElementById('ld-red').textContent   =d.lane.red_line  ?'🔴 감지':'⬜ 없음';
      document.getElementById('ld-obs').textContent   =d.lane.obstacle  ?'⚠️ 있음':'✅ 없음';
      document.getElementById('ld-cross').textContent =d.lane.crosswalk ?'🚶 감지':'⬜ 없음';
    }

    // 로그
    if(d.sign_log && d.sign_log.length!==lastLogLen){
      lastLogLen=d.sign_log.length;
      const list=document.getElementById('log-list');
      list.innerHTML='';
      d.sign_log.slice().reverse().forEach(e=>{
        const row=document.createElement('div');
        row.className='log-item';
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
class PcDashboardNode(Node):
    def __init__(self):
        super().__init__('pc_dashboard_node')
        self._lock = threading.Lock()

        # 이미지
        self._raw_jpeg:  bytes | None = None
        self._yolo_jpeg: bytes | None = None   # YOLO 추적
        self._line_jpeg: bytes | None = None   # 차선 감지

        # 타임스탬프
        self._raw_t   = -1.0
        self._yolo_t  = -1.0
        self._line_t  = -1.0
        self._sign_t  = -1.0
        self._brain_t = -1.0

        # 상태
        self._sign        = "UNKNOWN"
        self._brain_state = "--"
        self._brain_cmd   = "--"

        # 차선 상태 (cam_line_node → /vision_status 대신 /control_state 파싱)
        self._lane = {"error": 0.0, "red_line": False, "obstacle": False, "crosswalk": False}

        # 로그
        self._sign_log: deque = deque(maxlen=50)
        self._last_sign = None

        # FPS
        self._raw_buf:  deque = deque(maxlen=60)
        self._yolo_buf: deque = deque(maxlen=60)
        self._line_buf: deque = deque(maxlen=60)

        # QoS
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(CompressedImage, 'image_raw/compressed',  self._cb_raw,   be)
        self.create_subscription(CompressedImage, 'image_yolo/compressed', self._cb_yolo,  be)
        self.create_subscription(CompressedImage, 'image_line/compressed', self._cb_line,  be)
        self.create_subscription(String, 'traffic_sign_topic',             self._cb_sign,  10)
        self.create_subscription(String, '/control_state',                 self._cb_brain, 10)
        self.create_subscription(String, '/vision_status',                 self._cb_vision, 10)

        self.get_logger().info('PC Dashboard Node 시작 → http://0.0.0.0:5000')

    def _cb_raw(self, msg):
        now = time.time()
        with self._lock:
            self._raw_jpeg = bytes(msg.data)
            self._raw_buf.append(now)
            self._raw_t = now

    def _cb_yolo(self, msg):
        now = time.time()
        with self._lock:
            self._yolo_jpeg = bytes(msg.data)
            self._yolo_buf.append(now)
            self._yolo_t = now

    def _cb_line(self, msg):
        now = time.time()
        with self._lock:
            self._line_jpeg = bytes(msg.data)
            self._line_buf.append(now)
            self._line_t = now

    def _cb_sign(self, msg):
        now  = time.time()
        sign = msg.data
        with self._lock:
            self._sign   = sign
            self._sign_t = now
            if sign != self._last_sign:
                self._last_sign = sign
                self._sign_log.append({
                    "sign": sign,
                    "time": time.strftime('%H:%M:%S'),
                    "elapsed": "방금",
                    "_ts": now,
                })

    def _cb_brain(self, msg):
        now    = time.time()
        parts  = msg.data.split('|', 1)
        with self._lock:
            self._brain_state = parts[0] if parts else "--"
            self._brain_cmd   = parts[1] if len(parts) > 1 else "--"
            self._brain_t     = now

    def _cb_vision(self, msg):
        # 포맷: error|red_line|obstacle_in_front|crosswalk|avoid_dir|obstacle
        try:
            p = msg.data.split('|')
            with self._lock:
                self._lane = {
                    "error":     float(p[0]),
                    "red_line":  bool(int(p[1])),
                    "obstacle":  bool(int(p[2])),
                    "crosswalk": bool(int(p[3])),
                }
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
                e2 = dict(e)
                a  = now - e["_ts"]
                e2["elapsed"] = f"{a:.0f}s 전" if a < 60 else f"{a/60:.0f}m 전"
                log_copy.append(e2)

            return {
                "raw_fps":    fps(self._raw_buf),
                "yolo_fps":   fps(self._yolo_buf),
                "line_fps":   fps(self._line_buf),
                "raw_age":    age(self._raw_t),
                "yolo_age":   age(self._yolo_t),
                "line_age":   age(self._line_t),
                "sign":       self._sign,
                "sign_age":   age(self._sign_t),
                "brain_state": self._brain_state,
                "brain_cmd":   self._brain_cmd,
                "brain_age":  age(self._brain_t),
                "lane":       dict(self._lane),
                "sign_log":   log_copy,
            }


# ═══════════════════════════════════════════════════════════════
#  Flask
# ═══════════════════════════════════════════════════════════════
def create_app(node: PcDashboardNode) -> Flask:
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


# ═══════════════════════════════════════════════════════════════
#  엔트리포인트
# ═══════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = PcDashboardNode()
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
