#!/usr/bin/env python3
"""
pc_dashboard_node.py
──────────────────────────────────────────────────────────────────
PC에서 실행하는 ROS2 DDS 구독 + Flask 대시보드

구독 토픽:
  /image_raw/compressed     (sensor_msgs/CompressedImage) BEST_EFFORT
  /image_yolo/compressed    (sensor_msgs/CompressedImage) BEST_EFFORT
  /traffic_sign_topic       (std_msgs/String)
  /control_state            (std_msgs/String)  ← brain_node 상태

실행 조건:
  export ROS_DOMAIN_ID=0   (라즈베리파이와 동일하게)
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
#  HTML 대시보드 (인라인)
# ═══════════════════════════════════════════════════════════════
HTML = r"""<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Robot Vision Dashboard</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link href="https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Rajdhani:wght@400;600;700&display=swap" rel="stylesheet">
<style>
  :root {
    --bg:        #080c10;
    --surface:   #0d1219;
    --border:    #1a2535;
    --border-hi: #243040;
    --text:      #c8d8e8;
    --dim:       #4a6070;
    --accent-g:  #00e5a0;
    --accent-b:  #00aaff;
    --accent-y:  #ffc940;
    --accent-r:  #ff4466;
    --accent-p:  #c87cff;
    --mono:      'Share Tech Mono', monospace;
    --sans:      'Rajdhani', sans-serif;
  }

  * { box-sizing: border-box; margin: 0; padding: 0; }

  body {
    background: var(--bg);
    color: var(--text);
    font-family: var(--sans);
    min-height: 100vh;
    overflow-x: hidden;
  }

  /* ── 헤더 ── */
  header {
    display: flex;
    align-items: center;
    gap: 20px;
    padding: 14px 28px;
    background: var(--surface);
    border-bottom: 1px solid var(--border);
    position: sticky; top: 0; z-index: 100;
  }
  .logo {
    display: flex;
    align-items: center;
    gap: 10px;
  }
  .logo-icon {
    width: 34px; height: 34px;
    border: 2px solid var(--accent-g);
    border-radius: 6px;
    display: grid; place-items: center;
    position: relative;
    overflow: hidden;
  }
  .logo-icon::after {
    content: '';
    position: absolute;
    width: 12px; height: 12px;
    background: var(--accent-g);
    border-radius: 50%;
    box-shadow: 0 0 10px var(--accent-g);
    animation: pulse 2s ease-in-out infinite;
  }
  @keyframes pulse {
    0%,100% { transform: scale(1); opacity: 1; }
    50%      { transform: scale(0.7); opacity: 0.5; }
  }
  .logo-text { font-size: 20px; font-weight: 700; letter-spacing: 1px; color: #fff; }
  .logo-sub  { font-family: var(--mono); font-size: 11px; color: var(--dim); }

  .header-right { margin-left: auto; display: flex; align-items: center; gap: 16px; }
  .conn-badge {
    font-family: var(--mono);
    font-size: 11px;
    padding: 4px 10px;
    border-radius: 20px;
    border: 1px solid var(--border-hi);
    display: flex; align-items: center; gap: 6px;
    transition: border-color .3s, color .3s;
  }
  .conn-badge .dot {
    width: 7px; height: 7px; border-radius: 50%;
    background: var(--dim);
    transition: background .3s, box-shadow .3s;
  }
  .conn-badge.live { border-color: var(--accent-g); color: var(--accent-g); }
  .conn-badge.live .dot {
    background: var(--accent-g);
    box-shadow: 0 0 6px var(--accent-g);
    animation: blink 1.2s step-end infinite;
  }
  .conn-badge.warn { border-color: var(--accent-y); color: var(--accent-y); }
  .conn-badge.warn .dot { background: var(--accent-y); }
  .conn-badge.dead { border-color: var(--accent-r); color: var(--accent-r); }
  .conn-badge.dead .dot { background: var(--accent-r); }
  @keyframes blink { 50% { opacity: 0; } }

  /* ── 메인 그리드 ── */
  .main {
    display: grid;
    grid-template-columns: 1fr 1fr 300px;
    grid-template-rows: auto auto;
    gap: 16px;
    padding: 18px 24px;
  }
  @media (max-width: 1100px) {
    .main { grid-template-columns: 1fr 1fr; }
    .side-col { grid-column: 1 / -1; display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }
  }
  @media (max-width: 680px) {
    .main { grid-template-columns: 1fr; }
    .side-col { grid-template-columns: 1fr; }
  }

  /* ── 카드 ── */
  .card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 12px;
    overflow: hidden;
    position: relative;
    transition: border-color .3s;
  }
  .card:hover { border-color: var(--border-hi); }
  .card-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 10px 14px;
    border-bottom: 1px solid var(--border);
  }
  .card-title {
    font-size: 13px; font-weight: 700;
    letter-spacing: .8px;
    text-transform: uppercase;
  }
  .card-meta {
    font-family: var(--mono);
    font-size: 10px;
    color: var(--dim);
  }
  .fps-badge {
    font-family: var(--mono);
    font-size: 11px;
    color: var(--accent-b);
    padding: 2px 8px;
    border: 1px solid #1a3a55;
    border-radius: 4px;
    min-width: 60px;
    text-align: right;
  }

  /* ── 카메라 피드 ── */
  .feed-wrap {
    position: relative;
    background: #000;
    line-height: 0;
  }
  .feed-wrap img {
    width: 100%;
    height: auto;
    display: block;
  }
  .feed-overlay {
    position: absolute;
    bottom: 0; left: 0; right: 0;
    padding: 6px 10px;
    background: linear-gradient(transparent, rgba(0,0,0,.75));
    display: flex;
    justify-content: space-between;
    align-items: flex-end;
  }
  .feed-label {
    font-family: var(--mono);
    font-size: 10px;
    color: rgba(255,255,255,.5);
  }
  .feed-age {
    font-family: var(--mono);
    font-size: 10px;
    color: var(--accent-g);
  }
  .feed-age.stale { color: var(--accent-y); }
  .feed-age.dead  { color: var(--accent-r); }

  .no-signal {
    width: 100%;
    aspect-ratio: 4/3;
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    gap: 10px;
    background: #050810;
    color: var(--dim);
    font-family: var(--mono);
    font-size: 12px;
  }
  .no-signal svg { opacity: .3; }

  /* ── 사이드 컬럼 ── */
  .side-col { display: flex; flex-direction: column; gap: 16px; }

  /* ── Sign 표시 ── */
  .sign-display {
    padding: 16px 14px 12px;
    text-align: center;
  }
  .sign-value {
    font-size: 36px;
    font-weight: 700;
    letter-spacing: 2px;
    transition: color .4s, text-shadow .4s;
    text-shadow: 0 0 20px currentColor;
    line-height: 1.1;
  }
  .sign-time {
    font-family: var(--mono);
    font-size: 10px;
    color: var(--dim);
    margin-top: 4px;
  }

  /* ── Brain 상태 ── */
  .state-row {
    padding: 10px 14px;
    display: flex;
    flex-direction: column;
    gap: 6px;
  }
  .state-label { font-size: 11px; color: var(--dim); letter-spacing: .5px; }
  .state-value {
    font-family: var(--mono);
    font-size: 15px;
    color: var(--accent-b);
    word-break: break-all;
  }
  .state-cmd {
    font-family: var(--mono);
    font-size: 12px;
    color: var(--accent-y);
    opacity: .85;
  }

  /* ── 신호 히스토리 로그 ── */
  .log-list {
    padding: 6px 0;
    max-height: 200px;
    overflow-y: auto;
    scrollbar-width: thin;
    scrollbar-color: var(--border) transparent;
  }
  .log-item {
    display: flex;
    align-items: center;
    gap: 10px;
    padding: 5px 14px;
    border-bottom: 1px solid var(--border);
    animation: fadein .3s ease;
  }
  @keyframes fadein { from { opacity: 0; transform: translateY(-4px); } to { opacity: 1; } }
  .log-item:last-child { border-bottom: none; }
  .log-time { font-family: var(--mono); font-size: 10px; color: var(--dim); min-width: 58px; }
  .log-tag {
    font-family: var(--mono);
    font-size: 11px;
    font-weight: bold;
    padding: 1px 7px;
    border-radius: 4px;
    min-width: 72px;
    text-align: center;
  }
  .log-tag.GO          { background: #002820; color: var(--accent-g); border: 1px solid #005040; }
  .log-tag.STOP        { background: #2a0010; color: var(--accent-r); border: 1px solid #550030; }
  .log-tag.SPEED_LIMIT { background: #102030; color: var(--accent-b); border: 1px solid #1a4060; }
  .log-tag.TURN_LEFT   { background: #201800; color: var(--accent-y); border: 1px solid #604010; }
  .log-tag.UNKNOWN     { background: #151515; color: var(--dim);      border: 1px solid #252525; }

  /* ── 연결 통계 ── */
  .stats-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 1px;
    background: var(--border);
  }
  .stat-cell {
    background: var(--surface);
    padding: 10px 12px;
  }
  .stat-label { font-size: 10px; color: var(--dim); margin-bottom: 4px; }
  .stat-val   { font-family: var(--mono); font-size: 14px; color: var(--text); }
  .stat-val.ok   { color: var(--accent-g); }
  .stat-val.warn { color: var(--accent-y); }
  .stat-val.dead { color: var(--accent-r); }

  /* ── 스캔라인 효과 (카메라 위) ── */
  .scanline {
    pointer-events: none;
    position: absolute;
    inset: 0;
    background: repeating-linear-gradient(
      0deg,
      transparent,
      transparent 2px,
      rgba(0,0,0,.06) 2px,
      rgba(0,0,0,.06) 4px
    );
    z-index: 2;
  }
  .corner {
    pointer-events: none;
    position: absolute;
    width: 14px; height: 14px;
    z-index: 3;
  }
  .corner.tl { top: 0; left: 0; border-top: 2px solid var(--accent-g); border-left: 2px solid var(--accent-g); }
  .corner.tr { top: 0; right: 0; border-top: 2px solid var(--accent-g); border-right: 2px solid var(--accent-g); }
  .corner.bl { bottom: 0; left: 0; border-bottom: 2px solid var(--accent-g); border-left: 2px solid var(--accent-g); }
  .corner.br { bottom: 0; right: 0; border-bottom: 2px solid var(--accent-g); border-right: 2px solid var(--accent-g); }
  .corner.yolo.tl { border-color: var(--accent-b); }
  .corner.yolo.tr { border-color: var(--accent-b); }
  .corner.yolo.bl { border-color: var(--accent-b); }
  .corner.yolo.br { border-color: var(--accent-b); }
</style>
</head>
<body>

<header>
  <div class="logo">
    <div class="logo-icon"></div>
    <div>
      <div class="logo-text">Robot Vision Dashboard</div>
      <div class="logo-sub">Raspberry Pi + ROS2 + DDS Remote View</div>
    </div>
  </div>
  <div class="header-right">
    <div class="conn-badge" id="badge-raw"><div class="dot"></div><span>RAW</span></div>
    <div class="conn-badge" id="badge-yolo"><div class="dot"></div><span>YOLO</span></div>
    <div class="conn-badge" id="badge-brain"><div class="dot"></div><span>BRAIN</span></div>
  </div>
</header>

<div class="main">

  <!-- Raw Camera -->
  <div class="card">
    <div class="card-header">
      <span class="card-title">Raw Camera</span>
      <div style="display:flex;gap:8px;align-items:center">
        <span class="card-meta">/image_raw/compressed</span>
        <span class="fps-badge" id="fps-raw">-- fps</span>
      </div>
    </div>
    <div class="feed-wrap" id="raw-wrap">
      <div class="no-signal" id="raw-nosignal">
        <svg width="40" height="40" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5">
          <rect x="2" y="2" width="20" height="16" rx="2"/><path d="M8 22h8M12 18v4"/>
          <line x1="2" y1="2" x2="22" y2="22" stroke-width="1.2"/>
        </svg>
        <span>수신 대기중...</span>
      </div>
      <img id="raw-img" style="display:none" alt="raw">
      <div class="scanline"></div>
      <div class="corner tl"></div><div class="corner tr"></div>
      <div class="corner bl"></div><div class="corner br"></div>
      <div class="feed-overlay">
        <span class="feed-label">/image_raw/compressed</span>
        <span class="feed-age" id="age-raw">--</span>
      </div>
    </div>
  </div>

  <!-- YOLO Debug -->
  <div class="card">
    <div class="card-header">
      <span class="card-title" style="color:var(--accent-b)">YOLO Debug</span>
      <div style="display:flex;gap:8px;align-items:center">
        <span class="card-meta">/image_yolo/compressed</span>
        <span class="fps-badge" id="fps-yolo" style="color:var(--accent-b);border-color:#1a3a55">-- fps</span>
      </div>
    </div>
    <div class="feed-wrap" id="yolo-wrap">
      <div class="no-signal" id="yolo-nosignal">
        <svg width="40" height="40" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5">
          <rect x="2" y="2" width="20" height="16" rx="2"/><path d="M8 22h8M12 18v4"/>
          <line x1="2" y1="2" x2="22" y2="22" stroke-width="1.2"/>
        </svg>
        <span>수신 대기중...</span>
      </div>
      <img id="yolo-img" style="display:none" alt="yolo">
      <div class="scanline"></div>
      <div class="corner yolo tl"></div><div class="corner yolo tr"></div>
      <div class="corner yolo bl"></div><div class="corner yolo br"></div>
      <div class="feed-overlay">
        <span class="feed-label">/image_yolo/compressed</span>
        <span class="feed-age" id="age-yolo">--</span>
      </div>
    </div>
  </div>

  <!-- 사이드 컬럼 -->
  <div class="side-col">

    <!-- Traffic Sign -->
    <div class="card">
      <div class="card-header">
        <span class="card-title">Traffic Sign</span>
        <span class="card-meta">/traffic_sign_topic</span>
      </div>
      <div class="sign-display">
        <div class="sign-value" id="sign-value">UNKNOWN</div>
        <div class="sign-time" id="sign-time">수신 없음</div>
      </div>
    </div>

    <!-- Brain FSM 상태 -->
    <div class="card">
      <div class="card-header">
        <span class="card-title">Brain FSM</span>
        <span class="card-meta">/control_state</span>
      </div>
      <div class="state-row">
        <div class="state-label">현재 상태</div>
        <div class="state-value" id="brain-state">--</div>
      </div>
      <div class="state-row" style="border-top:1px solid var(--border);padding-top:8px">
        <div class="state-label">마지막 명령 → ESP32</div>
        <div class="state-cmd" id="brain-cmd">--</div>
      </div>
    </div>

    <!-- 연결 통계 -->
    <div class="card">
      <div class="card-header">
        <span class="card-title">Connection</span>
      </div>
      <div class="stats-grid">
        <div class="stat-cell">
          <div class="stat-label">Raw FPS</div>
          <div class="stat-val" id="stat-rfps">--</div>
        </div>
        <div class="stat-cell">
          <div class="stat-label">YOLO FPS</div>
          <div class="stat-val" id="stat-yfps">--</div>
        </div>
        <div class="stat-cell">
          <div class="stat-label">Raw 수신</div>
          <div class="stat-val" id="stat-rage">--</div>
        </div>
        <div class="stat-cell">
          <div class="stat-label">YOLO 수신</div>
          <div class="stat-val" id="stat-yage">--</div>
        </div>
      </div>
    </div>

  </div><!-- /side-col -->

  <!-- 신호 히스토리 (하단 전체) -->
  <div class="card" style="grid-column: 1 / -1">
    <div class="card-header">
      <span class="card-title">Sign Detection History</span>
      <span class="card-meta" id="log-count">0 events</span>
    </div>
    <div class="log-list" id="log-list">
      <div style="padding:14px;font-family:var(--mono);font-size:11px;color:var(--dim);text-align:center">
        수신된 신호 없음
      </div>
    </div>
  </div>

</div><!-- /main -->

<script>
const SIGN_COLORS = {
  GO:          'var(--accent-g)',
  STOP:        'var(--accent-r)',
  SPEED_LIMIT: 'var(--accent-b)',
  TURN_LEFT:   'var(--accent-y)',
  UNKNOWN:     'var(--dim)',
};

let prevRawSrc  = null;
let prevYoloSrc = null;

function ageClass(age) {
  if (age < 0)   return 'dead';
  if (age < 2.0) return '';
  if (age < 5.0) return 'warn';
  return 'dead';
}

function ageText(age) {
  if (age < 0)    return '수신 없음';
  if (age < 2.0)  return `${(age*1000).toFixed(0)} ms 전`;
  return `${age.toFixed(1)} s 전 ⚠`;
}

function badgeClass(age) {
  if (age < 0)   return 'dead';
  if (age < 2.0) return 'live';
  if (age < 5.0) return 'warn';
  return 'dead';
}

// ── 이미지 폴링 ──────────────────────────────────────────────
async function pollImages() {
  try {
    const res = await fetch('/api/frames');
    const d   = await res.json();

    // Raw
    if (d.raw_b64) {
      const src = 'data:image/jpeg;base64,' + d.raw_b64;
      if (src !== prevRawSrc) {
        document.getElementById('raw-img').src = src;
        document.getElementById('raw-img').style.display = 'block';
        document.getElementById('raw-nosignal').style.display = 'none';
        prevRawSrc = src;
      }
    }

    // YOLO
    if (d.yolo_b64) {
      const src = 'data:image/jpeg;base64,' + d.yolo_b64;
      if (src !== prevYoloSrc) {
        document.getElementById('yolo-img').src = src;
        document.getElementById('yolo-img').style.display = 'block';
        document.getElementById('yolo-nosignal').style.display = 'none';
        prevYoloSrc = src;
      }
    }
  } catch(e) {}

  setTimeout(pollImages, 80);   // ~12fps 폴링
}

// ── 상태 폴링 ─────────────────────────────────────────────────
let logCount    = 0;
let lastLogLen  = 0;

async function pollStatus() {
  try {
    const res = await fetch('/api/status');
    const d   = await res.json();

    // FPS 배지
    document.getElementById('fps-raw').textContent  = `${d.raw_fps.toFixed(1)} fps`;
    document.getElementById('fps-yolo').textContent = `${d.yolo_fps.toFixed(1)} fps`;

    // 수신 나이
    const ra = d.raw_age, ya = d.yolo_age;
    const rEl = document.getElementById('age-raw');
    const yEl = document.getElementById('age-yolo');
    rEl.textContent = ageText(ra); rEl.className = 'feed-age ' + ageClass(ra);
    yEl.textContent = ageText(ya); yEl.className = 'feed-age ' + ageClass(ya);

    // 연결 배지
    ['raw','yolo','brain'].forEach(k => {
      const age = k === 'raw' ? d.raw_age : k === 'yolo' ? d.yolo_age : d.brain_age;
      const el  = document.getElementById('badge-' + k);
      el.className = 'conn-badge ' + badgeClass(age);
    });

    // 통계
    const sfps = (v, el) => {
      el.textContent = `${v.toFixed(1)}`;
      el.className = 'stat-val ' + (v > 3 ? 'ok' : v > 0 ? 'warn' : 'dead');
    };
    sfps(d.raw_fps,  document.getElementById('stat-rfps'));
    sfps(d.yolo_fps, document.getElementById('stat-yfps'));

    const sage = (age, el) => {
      el.textContent = ageText(age);
      el.className   = 'stat-val ' + ageClass(age);
    };
    sage(d.raw_age,  document.getElementById('stat-rage'));
    sage(d.yolo_age, document.getElementById('stat-yage'));

    // Traffic Sign
    const sv = document.getElementById('sign-value');
    sv.textContent = d.sign || 'UNKNOWN';
    sv.style.color = SIGN_COLORS[d.sign] || SIGN_COLORS['UNKNOWN'];

    const st = document.getElementById('sign-time');
    st.textContent = d.sign_age >= 0 ? ageText(d.sign_age) : '수신 없음';

    // Brain FSM
    document.getElementById('brain-state').textContent = d.brain_state || '--';
    document.getElementById('brain-cmd').textContent   = d.brain_cmd   || '--';

    // 히스토리 로그
    if (d.sign_log && d.sign_log.length !== lastLogLen) {
      lastLogLen = d.sign_log.length;
      const list = document.getElementById('log-list');
      list.innerHTML = '';
      d.sign_log.slice().reverse().forEach(entry => {
        const row = document.createElement('div');
        row.className = 'log-item';
        row.innerHTML = `
          <span class="log-time">${entry.time}</span>
          <span class="log-tag ${entry.sign}">${entry.sign}</span>
          <span style="font-family:var(--mono);font-size:10px;color:var(--dim)">${entry.elapsed}</span>
        `;
        list.appendChild(row);
      });
      document.getElementById('log-count').textContent = `${d.sign_log.length} events`;
    }

  } catch(e) {}

  setTimeout(pollStatus, 300);
}

pollImages();
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

        # 이미지 캐시
        self._raw_jpeg:  bytes | None = None
        self._yolo_jpeg: bytes | None = None

        # 타임스탬프
        self._raw_time   = -1.0
        self._yolo_time  = -1.0
        self._sign_time  = -1.0
        self._brain_time = -1.0

        # 상태 값
        self._sign       = "UNKNOWN"
        self._brain_state = "--"
        self._brain_cmd   = "--"

        # 신호 히스토리 (최대 50건)
        self._sign_log: deque = deque(maxlen=50)
        self._last_sign = None

        # FPS 계산
        self._raw_fps_buf  = deque(maxlen=30)
        self._yolo_fps_buf = deque(maxlen=30)

        # QoS
        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(CompressedImage, 'image_raw/compressed',  self._cb_raw,   be_qos)
        self.create_subscription(CompressedImage, 'image_yolo/compressed', self._cb_yolo,  be_qos)
        self.create_subscription(String, 'traffic_sign_topic',             self._cb_sign,  10)
        self.create_subscription(String, '/control_state',                 self._cb_brain, 10)

        self.get_logger().info('PC Dashboard Node 시작 — Flask: http://0.0.0.0:5000')

    # ── 콜백 ────────────────────────────────────────────────────
    def _cb_raw(self, msg):
        now = time.time()
        with self._lock:
            self._raw_jpeg = bytes(msg.data)
            self._raw_fps_buf.append(now)
            self._raw_time = now

    def _cb_yolo(self, msg):
        now = time.time()
        with self._lock:
            self._yolo_jpeg = bytes(msg.data)
            self._yolo_fps_buf.append(now)
            self._yolo_time = now

    def _cb_sign(self, msg):
        now = time.time()
        sign = msg.data
        with self._lock:
            self._sign      = sign
            self._sign_time = now
            if sign != self._last_sign:
                self._last_sign = sign
                self._sign_log.append({
                    "sign":    sign,
                    "time":    time.strftime('%H:%M:%S'),
                    "elapsed": "방금",
                    "_ts":     now,
                })

    def _cb_brain(self, msg):
        now = time.time()
        # 포맷: "CRUISE|G200"  or  "AVOID Step2 [WAIT]|T45"
        parts = msg.data.split('|', 1)
        with self._lock:
            self._brain_state = parts[0] if len(parts) > 0 else "--"
            self._brain_cmd   = parts[1] if len(parts) > 1 else "--"
            self._brain_time  = now

    # ── Flask용 스냅샷 ──────────────────────────────────────────
    def get_frames(self):
        with self._lock:
            return self._raw_jpeg, self._yolo_jpeg

    def get_status(self):
        now = time.time()
        with self._lock:
            def age(t):  return round(now - t, 3) if t > 0 else -1
            def fps(buf):
                if len(buf) < 2: return 0.0
                span = buf[-1] - buf[0]
                return (len(buf) - 1) / span if span > 0 else 0.0

            # elapsed 문자열 갱신
            log_copy = []
            for e in self._sign_log:
                e2 = dict(e)
                a  = now - e["_ts"]
                e2["elapsed"] = f"{a:.0f}s 전" if a < 60 else f"{a/60:.0f}m 전"
                log_copy.append(e2)

            return {
                "raw_fps":    fps(self._raw_fps_buf),
                "yolo_fps":   fps(self._yolo_fps_buf),
                "raw_age":    age(self._raw_time),
                "yolo_age":   age(self._yolo_time),
                "sign":       self._sign,
                "sign_age":   age(self._sign_time),
                "brain_state": self._brain_state,
                "brain_cmd":   self._brain_cmd,
                "brain_age":  age(self._brain_time),
                "sign_log":   log_copy,
            }


# ═══════════════════════════════════════════════════════════════
#  Flask 앱
# ═══════════════════════════════════════════════════════════════
def create_app(node: PcDashboardNode) -> Flask:
    app = Flask(__name__)

    @app.route('/')
    def index():
        return render_template_string(HTML)

    @app.route('/api/frames')
    def api_frames():
        raw, yolo = node.get_frames()
        return jsonify({
            "raw_b64":  base64.b64encode(raw).decode()  if raw  else None,
            "yolo_b64": base64.b64encode(yolo).decode() if yolo else None,
        })

    @app.route('/api/status')
    def api_status():
        return jsonify(node.get_status())

    # MJPEG 스트림 (선택적 — img src="/stream/raw" 로도 쓸 수 있음)
    def _mjpeg_gen(get_fn):
        while True:
            data = get_fn()
            if data:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + data + b'\r\n')
            time.sleep(0.04)

    @app.route('/stream/raw')
    def stream_raw():
        return Response(_mjpeg_gen(lambda: node.get_frames()[0]),
                        mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.route('/stream/yolo')
    def stream_yolo():
        return Response(_mjpeg_gen(lambda: node.get_frames()[1]),
                        mimetype='multipart/x-mixed-replace; boundary=frame')

    return app


# ═══════════════════════════════════════════════════════════════
#  엔트리포인트
# ═══════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = PcDashboardNode()
    app  = create_app(node)

    flask_thread = threading.Thread(
        target=lambda: app.run(
            host='0.0.0.0', port=5000,
            debug=False, use_reloader=False, threaded=True
        ),
        daemon=True
    )
    flask_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
