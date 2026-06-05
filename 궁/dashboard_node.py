#!/usr/bin/env python3
"""
dashboard_node.py  (v7 — 캠별 선택 녹화/캡처)
──────────────────────────────────────────────────────────────────
구독 토픽:
  /image_raw/compressed     BEST_EFFORT  ← 카메라 원본
  /image_yolo/compressed    BEST_EFFORT  ← YOLO 디버그
  /image_line/compressed    BEST_EFFORT  ← 차선 감지 디버그
  /traffic_sign_topic       RELIABLE
  /control_state            RELIABLE
  /vision_status            RELIABLE

추가 기능:
  • 각 캠 헤더의 ⦿ 토글로 녹화/캡처 "대상" 선택 (셋 다 / 하나만 자유)
  • 녹화 토글       → 선택된 캠만 .mp4 저장 (recordings/)
  • 단일 캡처       → 선택된 캠 현재 프레임 1장씩 (captures/)
  • 연속 캡처 토글  → 선택된 캠을 일정 간격마다 저장 (captures/)
  • 활동 로그       → 우측 패널에 실시간 표시

실행:
  ros2 run p2_pkg_pc dashboard_node
  브라우저: http://localhost:5000
──────────────────────────────────────────────────────────────────
"""

import threading
import time
import base64
import os
from collections import deque
from datetime import datetime

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from flask import Flask, jsonify, render_template_string, request

# 저장 폴더 (스크립트 실행 위치 기준). 한글 경로 회피를 위해 영어로.
OUTPUT_ROOT   = os.path.abspath("dashboard_output")
RECORD_DIR    = os.path.join(OUTPUT_ROOT, "recordings")
CAPTURE_DIR   = os.path.join(OUTPUT_ROOT, "captures")
BURST_INTERVAL = 0.5   # 연속 캡처 간격(초)

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
html,body{height:100%;overflow:hidden}
body{
  background:var(--bg);color:var(--text);font-family:var(--sans);
  height:100vh;display:flex;flex-direction:column;
}

/* ── 헤더 ── */
header{
  display:flex;align-items:center;gap:14px;
  padding:9px 20px;
  background:var(--surface);
  border-bottom:1px solid var(--border);
  flex-shrink:0;
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
  display:flex;gap:8px;align-items:center;
  padding:10px 20px;
  background:var(--surface);
  border-bottom:1px solid var(--border);
  flex-shrink:0;
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

/* 구분선 */
.bar-sep{width:1px;height:26px;background:var(--border);margin:0 4px}

/* ── 캡처/녹화 컨트롤 버튼 ── */
.rec-btn{
  padding:8px 16px;border-radius:8px;
  border:1px solid var(--bhi);background:transparent;color:var(--text);
  font-family:var(--sans);font-size:13px;font-weight:600;letter-spacing:.4px;
  cursor:pointer;display:flex;align-items:center;gap:7px;transition:all .2s;
}
.rec-btn:hover{border-color:var(--text)}
.rec-btn .ico{font-size:13px}
/* 녹화중 */
.rec-btn.recording{border-color:var(--r);color:var(--r);background:rgba(255,51,85,.08)}
.rec-btn.recording .ico{animation:recblink 1s step-end infinite}
@keyframes recblink{50%{opacity:.2}}
/* 연속캡처중 */
.rec-btn.bursting{border-color:var(--g);color:var(--g);background:rgba(0,229,160,.08)}
.rec-btn.bursting .ico{animation:recblink 1s step-end infinite}
/* 단일캡처 플래시 */
.rec-btn.flash{border-color:var(--b);color:var(--b);background:rgba(0,170,255,.15)}

.rec-status{
  margin-left:auto;font-family:var(--mono);font-size:11px;color:var(--dim);
  display:flex;align-items:center;gap:8px;
}
.rec-status .rdot{width:8px;height:8px;border-radius:50%;background:var(--dim)}
.rec-status.live .rdot{background:var(--r);box-shadow:0 0 6px var(--r);animation:recblink 1s step-end infinite}
.rec-status.live{color:var(--r)}

/* ── 메인 (남은 높이 전부) ── */
.main{
  display:grid;
  grid-template-columns:1fr 260px;
  gap:12px;
  padding:14px 20px;
  flex:1;
  min-height:0;
  overflow:hidden;
}

/* 피드 컨테이너 */
.feeds{
  display:grid;
  gap:12px;
  min-height:0;
  overflow:hidden;
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
.feed-card{
  display:flex;flex-direction:column;
  min-height:0;
}
.feed-card.hidden{display:none}
.card-hd{
  display:flex;align-items:center;justify-content:space-between;
  padding:7px 11px;border-bottom:1px solid var(--border);
  flex-shrink:0;
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
/* 녹화중 표시 (카드 헤더) */
.rec-pill{
  font-family:var(--mono);font-size:9px;color:var(--r);
  border:1px solid var(--r);border-radius:4px;padding:1px 5px;
  display:none;align-items:center;gap:4px;
}
.rec-pill.on{display:flex}
.rec-pill::before{content:'';width:5px;height:5px;border-radius:50%;background:var(--r);animation:recblink 1s step-end infinite}

/* 녹화/캡처 대상 선택 토글 (카드 헤더) */
.tgt-btn{
  width:22px;height:22px;border-radius:6px;
  border:1px solid var(--bhi);background:transparent;color:var(--dim);
  cursor:pointer;display:flex;align-items:center;justify-content:center;
  font-size:11px;transition:all .15s;padding:0;
}
.tgt-btn:hover{border-color:var(--text)}
.tgt-btn .tgt-ico::before{content:'○'}
.tgt-btn.on .tgt-ico::before{content:'⦿'}
.tgt-btn.on.raw {border-color:var(--g);color:var(--g)}
.tgt-btn.on.yolo{border-color:var(--b);color:var(--b)}
.tgt-btn.on.lane{border-color:var(--y);color:var(--y)}

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
.side{display:flex;flex-direction:column;gap:10px;overflow-y:auto;min-height:0;}
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

/* 활동 로그 */
.act-list{max-height:200px;overflow-y:auto;scrollbar-width:thin;scrollbar-color:var(--border) transparent}
.act-item{display:flex;align-items:center;gap:8px;padding:5px 11px;border-bottom:1px solid var(--border);animation:fi .3s ease}
@keyframes fi{from{opacity:0;transform:translateY(-3px)}}
.act-item:last-child{border:none}
.at{font-family:var(--mono);font-size:10px;color:var(--dim);min-width:52px}
.am{font-family:var(--mono);font-size:11px;flex:1}
.am.rec{color:var(--r)}.am.cap{color:var(--b)}.am.burst{color:var(--g)}.am.info{color:var(--dim)}

.log-list{max-height:140px;overflow-y:auto;scrollbar-width:thin;scrollbar-color:var(--border) transparent}
.log-item{display:flex;align-items:center;gap:7px;padding:4px 11px;border-bottom:1px solid var(--border);animation:fi .3s ease}
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

  <div class="bar-sep"></div>

  <button class="rec-btn" id="btn-record" onclick="toggleRecord()">
    <span class="ico">⏺</span><span id="btn-record-txt">녹화 시작</span>
  </button>
  <button class="rec-btn" id="btn-capture" onclick="captureSingle()">
    <span class="ico">📸</span>한 장 캡처
  </button>
  <button class="rec-btn" id="btn-burst" onclick="toggleBurst()">
    <span class="ico">🔁</span><span id="btn-burst-txt">연속 캡처</span>
  </button>

  <div class="rec-status" id="rec-status">
    <span class="rdot"></span><span id="rec-status-txt">대기 중</span>
  </div>
</div>

<div class="main">

  <div class="feeds" id="feeds" data-count="3">

    <!-- 원본 -->
    <div class="card feed-card" id="card-raw">
      <div class="card-hd">
        <span class="card-title" style="color:var(--g)">📷 카메라 원본</span>
        <div style="display:flex;gap:7px;align-items:center">
          <button class="tgt-btn raw on" id="tgt-raw" onclick="toggleTarget('raw')" title="녹화/캡처 대상에 포함"><span class="tgt-ico"></span></button>
          <span class="rec-pill" id="recpill-raw">REC</span>
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
          <button class="tgt-btn yolo on" id="tgt-yolo" onclick="toggleTarget('yolo')" title="녹화/캡처 대상에 포함"><span class="tgt-ico"></span></button>
          <span class="rec-pill" id="recpill-yolo">REC</span>
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
          <button class="tgt-btn lane on" id="tgt-lane" onclick="toggleTarget('lane')" title="녹화/캡처 대상에 포함"><span class="tgt-ico"></span></button>
          <span class="rec-pill" id="recpill-lane">REC</span>
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

  <div class="side">
    <!-- 활동 로그 (녹화/캡처) -->
    <div class="card">
      <div class="card-hd"><span class="card-title" style="color:var(--b)">활동 로그</span><span class="card-meta" id="act-cnt">0</span></div>
      <div class="act-list" id="act-list">
        <div style="padding:12px;font-family:var(--mono);font-size:10px;color:var(--dim);text-align:center">대기 중</div>
      </div>
    </div>

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
const recTarget={raw:true,yolo:true,lane:true};   // 녹화/캡처 대상 선택
let prev={raw:null,yolo:null,lane:null}, lastLogLen=0, lastActLen=0;
let isRecording=false, isBursting=false;

function selectedTargets(){
  return Object.keys(recTarget).filter(k=>recTarget[k]);
}
function toggleTarget(k){
  // 녹화/연속캡처 진행 중에는 대상 변경 잠금 (혼란 방지)
  if(isRecording||isBursting){ return; }
  recTarget[k]=!recTarget[k];
  const btn=document.getElementById('tgt-'+k);
  btn.className='tgt-btn '+k+(recTarget[k]?' on':'');
}

function toggleFeed(k){
  visible[k]=!visible[k];
  document.getElementById('card-'+k).classList.toggle('hidden',!visible[k]);
  const btn=document.getElementById('tg-'+k);
  btn.className='tg-btn '+k+(visible[k]?' on':' off');
  const cnt=Object.values(visible).filter(Boolean).length;
  document.getElementById('feeds').setAttribute('data-count', cnt||1);
}

/* ── 녹화 / 캡처 ── */
function postJSON(url){
  return fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},
                    body:JSON.stringify({targets:selectedTargets()})});
}
async function toggleRecord(){
  if(!isRecording && selectedTargets().length===0){ alert('녹화할 캠을 1개 이상 선택하세요 (카드 우측 ⦿)'); return; }
  try{
    const d=await(await postJSON('/api/record/toggle')).json();
    applyRecState(d.recording);
  }catch(e){}
}
async function toggleBurst(){
  if(!isBursting && selectedTargets().length===0){ alert('캡처할 캠을 1개 이상 선택하세요 (카드 우측 ⦿)'); return; }
  try{
    const d=await(await postJSON('/api/burst/toggle')).json();
    applyBurstState(d.bursting);
  }catch(e){}
}
async function captureSingle(){
  if(selectedTargets().length===0){ alert('캡처할 캠을 1개 이상 선택하세요 (카드 우측 ⦿)'); return; }
  const btn=document.getElementById('btn-capture');
  btn.classList.add('flash');
  setTimeout(()=>btn.classList.remove('flash'),350);
  try{ await postJSON('/api/capture/single'); }catch(e){}
}
function lockTargets(lock){
  ['raw','yolo','lane'].forEach(k=>{
    const b=document.getElementById('tgt-'+k);
    b.style.opacity = lock?'0.4':'1';
    b.style.pointerEvents = lock?'none':'auto';
  });
}
function applyRecState(state){
  // state: 캠별 녹화 여부 dict {raw,yolo,lane}
  const anyRec = state.raw||state.yolo||state.lane;
  isRecording=anyRec;
  const btn=document.getElementById('btn-record');
  btn.classList.toggle('recording',anyRec);
  document.getElementById('btn-record-txt').textContent=anyRec?'녹화 종료':'녹화 시작';
  ['raw','yolo','lane'].forEach(k=>document.getElementById('recpill-'+k).classList.toggle('on',!!state[k]));
  lockTargets(anyRec||isBursting);
  updateRecStatus(state);
}
function applyBurstState(on){
  isBursting=on;
  const btn=document.getElementById('btn-burst');
  btn.classList.toggle('bursting',on);
  document.getElementById('btn-burst-txt').textContent=on?'연속 캡처 종료':'연속 캡처';
  lockTargets(isRecording||on);
  updateRecStatus();
}
function updateRecStatus(recState){
  const st=document.getElementById('rec-status');
  const txt=document.getElementById('rec-status-txt');
  let recNames=[];
  if(recState) recNames=['raw','yolo','lane'].filter(k=>recState[k]);
  const parts=[];
  if(recNames.length) parts.push('● 녹화중 ['+recNames.join(', ')+']');
  if(isBursting) parts.push('● 연속캡처중');
  if(parts.length){ st.className='rec-status live'; txt.textContent=parts.join('  '); }
  else { st.className='rec-status'; txt.textContent='대기 중'; }
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

    /* 서버측 녹화/캡처 상태 동기화 (새로고침/다른 탭 대응) */
    if(d.recording){
      const anyRec=d.recording.raw||d.recording.yolo||d.recording.lane;
      if(anyRec!==isRecording) applyRecState(d.recording);
      else updateRecStatus(d.recording);
    }
    if(d.bursting!==isBursting) applyBurstState(d.bursting);

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

    /* 활동 로그 */
    if(d.activity&&d.activity.length!==lastActLen){
      lastActLen=d.activity.length;
      const list=document.getElementById('act-list');
      list.innerHTML='';
      d.activity.slice().reverse().forEach(e=>{
        const row=document.createElement('div');row.className='act-item';
        row.innerHTML=`<span class="at">${e.time}</span><span class="am ${e.kind}">${e.msg}</span>`;
        list.appendChild(row);
      });
      document.getElementById('act-cnt').textContent=d.activity.length;
    }

    /* 신호 로그 */
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

        self._raw_jpeg = None
        self._yolo_jpeg = None
        self._line_jpeg = None

        self._raw_t   = -1.0
        self._yolo_t  = -1.0
        self._line_t  = -1.0
        self._sign_t  = -1.0
        self._brain_t = -1.0

        self._sign        = "UNKNOWN"
        self._brain_state = "--"
        self._brain_cmd   = "--"
        self._lane = {"error": 0.0, "red_line": False, "obstacle": False, "crosswalk": False}

        self._sign_log = deque(maxlen=50)
        self._last_sign = None

        self._raw_buf  = deque(maxlen=60)
        self._yolo_buf = deque(maxlen=60)
        self._line_buf = deque(maxlen=60)

        # ── 녹화/캡처 상태 ──
        self._rec_lock   = threading.Lock()
        self._recording  = {"raw": False, "yolo": False, "lane": False}  # 캠별 녹화 여부
        self._writers    = {"raw": None, "yolo": None, "lane": None}
        self._rec_session = None       # 녹화 세션 타임스탬프 문자열
        self._bursting   = False
        self._burst_targets = []       # 연속 캡처 대상 캠
        self._burst_thread = None
        self._capture_seq = 0          # 단일/연속 캡처 통합 시퀀스
        self._activity = deque(maxlen=60)

        os.makedirs(RECORD_DIR, exist_ok=True)
        os.makedirs(CAPTURE_DIR, exist_ok=True)

        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(CompressedImage, 'image_raw/compressed',  self._cb_raw,    be)
        self.create_subscription(CompressedImage, 'image_yolo/compressed', self._cb_yolo,   be)
        self.create_subscription(CompressedImage, 'image_line/compressed', self._cb_line,   be)
        self.create_subscription(String, 'traffic_sign_topic',             self._cb_sign,   10)
        self.create_subscription(String, '/control_state',                 self._cb_brain,  10)
        self.create_subscription(String, '/vision_status',                 self._cb_vision, 10)

        self.get_logger().info('Dashboard Node 시작 → http://0.0.0.0:5000')
        self.get_logger().info(f'녹화 저장 위치: {RECORD_DIR}')
        self.get_logger().info(f'캡처 저장 위치: {CAPTURE_DIR}')

    # ── 활동 로그 추가 ──
    def _add_activity(self, kind, msg):
        with self._lock:
            self._activity.append({"kind": kind, "msg": msg,
                                   "time": time.strftime('%H:%M:%S')})
        self.get_logger().info(f'[{kind}] {msg}')

    # ── 이미지 콜백 ──
    def _cb_raw(self, msg):
        now = time.time(); data = bytes(msg.data)
        with self._lock:
            self._raw_jpeg = data; self._raw_buf.append(now); self._raw_t = now
        if self._recording['raw']:
            self._write_video('raw', data)

    def _cb_yolo(self, msg):
        now = time.time(); data = bytes(msg.data)
        with self._lock:
            self._yolo_jpeg = data; self._yolo_buf.append(now); self._yolo_t = now
        if self._recording['yolo']:
            self._write_video('yolo', data)

    def _cb_line(self, msg):
        now = time.time(); data = bytes(msg.data)
        with self._lock:
            self._line_jpeg = data; self._line_buf.append(now); self._line_t = now
        if self._recording['lane']:
            self._write_video('lane', data)

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

    # ── 녹화: VideoWriter에 프레임 기록 ──
    def _write_video(self, key, jpeg):
        arr = np.frombuffer(jpeg, np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            return
        with self._rec_lock:
            if not self._recording[key]:
                return
            w = self._writers.get(key)
            if w is None:
                h, wd = img.shape[:2]
                fps = self._measure_fps(key)
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                path = os.path.join(RECORD_DIR, f"{key}_{self._rec_session}.mp4")
                w = cv2.VideoWriter(path, fourcc, fps, (wd, h))
                self._writers[key] = w
            w.write(img)

    def _measure_fps(self, key):
        buf = {"raw": self._raw_buf, "yolo": self._yolo_buf, "lane": self._line_buf}[key]
        if len(buf) < 2:
            return 20.0
        span = buf[-1] - buf[0]
        fps = (len(buf) - 1) / span if span > 0 else 20.0
        return float(min(max(fps, 5.0), 60.0))   # 5~60 사이로 클램프

    # ── 녹화 시작/종료 (targets: 녹화할 캠 리스트) ──
    def toggle_recording(self, targets):
        any_rec = any(self._recording.values())
        if any_rec:
            # 녹화 중 → 전체 종료
            with self._rec_lock:
                self._recording = {"raw": False, "yolo": False, "lane": False}
                for k, w in self._writers.items():
                    if w is not None:
                        w.release()
                self._writers = {"raw": None, "yolo": None, "lane": None}
            self._add_activity("rec", f"⏹ 녹화 종료 → recordings/*_{self._rec_session}.mp4")
        else:
            # 녹화 시작 → 선택된 캠만
            targets = [t for t in targets if t in ("raw", "yolo", "lane")]
            if not targets:
                return dict(self._recording)
            self._rec_session = datetime.now().strftime("%Y%m%d_%H%M%S")
            with self._rec_lock:
                self._writers = {"raw": None, "yolo": None, "lane": None}
                self._recording = {k: (k in targets) for k in ("raw", "yolo", "lane")}
            self._add_activity("rec", f"⏺ 녹화 시작 [{', '.join(targets)}]")
        return dict(self._recording)

    # ── 단일 캡처 (targets: 캡처할 캠 리스트) ──
    def capture_single(self, targets):
        targets = [t for t in targets if t in ("raw", "yolo", "lane")]
        n = self._save_capture_set(targets)
        self._add_activity("cap", f"📸 캡처됨 [{', '.join(targets)}] {n}장")
        return n

    # ── 연속 캡처 시작/종료 (targets: 캡처할 캠 리스트) ──
    def toggle_burst(self, targets):
        if self._bursting:
            self._bursting = False
            self._add_activity("burst", "🔁 연속 캡처 종료")
        else:
            targets = [t for t in targets if t in ("raw", "yolo", "lane")]
            if not targets:
                return self._bursting
            self._burst_targets = targets
            self._bursting = True
            self._add_activity("burst", f"🔁 연속 캡처 시작 [{', '.join(targets)}] {BURST_INTERVAL}s 간격")
            self._burst_thread = threading.Thread(target=self._burst_loop, daemon=True)
            self._burst_thread.start()
        return self._bursting

    def _burst_loop(self):
        while self._bursting:
            self._save_capture_set(self._burst_targets)
            time.sleep(BURST_INTERVAL)

    # ── 선택된 캠의 현재 프레임을 캡처 폴더에 저장 (jpeg 그대로 → 무손실/빠름) ──
    def _save_capture_set(self, targets):
        with self._lock:
            all_frames = {"raw": self._raw_jpeg, "yolo": self._yolo_jpeg, "lane": self._line_jpeg}
            self._capture_seq += 1
            seq = self._capture_seq
        frames = {k: all_frames[k] for k in targets if k in all_frames}
        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        saved = 0
        for key, jpeg in frames.items():
            if jpeg is None:
                continue
            sub = os.path.join(CAPTURE_DIR, key)
            os.makedirs(sub, exist_ok=True)
            path = os.path.join(sub, f"{key}_{ts}_{seq:05d}.jpg")
            try:
                # 한글 경로 회피 + jpeg 원본 그대로 기록
                with open(path, "wb") as f:
                    f.write(jpeg)
                saved += 1
            except Exception as e:
                self.get_logger().warn(f"캡처 저장 실패 {path}: {e}")
        return saved

    # ── 데이터 제공 ──
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
            activity_copy = list(self._activity)
            return {
                "raw_fps": fps(self._raw_buf), "yolo_fps": fps(self._yolo_buf), "line_fps": fps(self._line_buf),
                "raw_age": age(self._raw_t), "yolo_age": age(self._yolo_t), "line_age": age(self._line_t),
                "sign": self._sign, "sign_age": age(self._sign_t),
                "brain_state": self._brain_state, "brain_cmd": self._brain_cmd, "brain_age": age(self._brain_t),
                "lane": dict(self._lane), "sign_log": log_copy,
                "recording": dict(self._recording), "bursting": self._bursting,
                "activity": activity_copy,
            }


def create_app(node):
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

    # ── 녹화/캡처 엔드포인트 ──
    def _get_targets():
        data = request.get_json(silent=True) or {}
        return data.get('targets', ['raw', 'yolo', 'lane'])

    @app.route('/api/record/toggle', methods=['POST'])
    def api_record_toggle():
        return jsonify({"recording": node.toggle_recording(_get_targets())})

    @app.route('/api/capture/single', methods=['POST'])
    def api_capture_single():
        return jsonify({"saved": node.capture_single(_get_targets())})

    @app.route('/api/burst/toggle', methods=['POST'])
    def api_burst_toggle():
        return jsonify({"bursting": node.toggle_burst(_get_targets())})

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
        # 녹화 중이었으면 안전하게 파일 닫기
        if any(node._recording.values()):
            node.toggle_recording([])
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
