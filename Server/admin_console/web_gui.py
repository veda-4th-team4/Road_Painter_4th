#!/usr/bin/env python3
"""Road-Painter 관리자 창 - HTTP 서버 + 로봇/로그 UI (진입점).

역할별로 세 파일로 나뉜다:
  rp_core.py  공통 코어 (설정 + 로그 브로드캐스트/SSE + 중앙 서버 ADMIN 링크)
  cctv.py     카메라·캘리브레이션 (CCTV 팀 작업 파일) - 대시보드 UI 포함
  web_gui.py  (이 파일) HTTP 라우팅 + 로봇 제어/로그 모니터 페이지 + main()

실행:  python3 web_gui.py [tcp_port] [http_port] [snapshot_port]
        (기본값 6000 8081 6001 - admin_console/config.sh에서 조정)
브라우저:  http://<이 Pi IP>:<http_port>            카메라 캘리브레이션
           http://<이 Pi IP>:<http_port>/robot      로봇 제어
           http://<이 Pi IP>:<http_port>/logs       로그 모니터
           http://<이 Pi IP>:<http_port>/login      로그인 (아래 게이트)

⚠️ 로그인 게이트: 위 세 화면과 모든 POST API는 중앙 서버(9000)에 LOGIN이 통과하기
전까지 열리지 않고 /login으로 돌아간다. 캘리브레이션 결과가 "그 시점에 로그인된
계정"에 저장되는 구조라(../src/router.cpp handleHMatrix), 로그인 없이 캘리를 끝내면
결과가 세션에만 남고 사라지기 때문 - 순서를 사람이 기억하는 대신 구조로 강제한다.

세션은 브라우저별이다: 로그인 성공 시 HttpOnly 세션 쿠키(rp_admin)를 발급하고,
게이트는 그 쿠키를 rp_core의 세션 목록과 대조한다. 따라서
  - 다른 PC/브라우저는 각자 로그인해야 하고 (한 명이 열어둔다고 다 열리지 않음)
  - 브라우저를 닫으면 세션 쿠키가 사라져 재로그인이 필요하며
  - 탭바의 '로그아웃'(POST /logout)으로 즉시 끊을 수 있고
  - 서버 링크가 끊기면 모든 세션이 무효화된다 (rp_core.clear_login).
판정이 서버 측 상태라 페이지 JS 조작으로는 우회되지 않는다.

stdlib만 사용 (http.server + Server-Sent Events).
"""
import html
import http.server
import json
import queue
import socketserver
import threading
import time
from http.cookies import CookieError, SimpleCookie
from urllib.parse import quote, urlparse

import rp_core
import cctv
from rp_core import (
    broadcast,
    subscribers,
    subs_lock,
    log_history,
    server_send,
    server_link_loop,
    LOG_SUBJECT_JS,
    HTTP_PORT,
)



# 관리자 창 공통 헤드: 테마 선택 스크립트 + CSS 변수(라이트/다크) + 공용 컴포넌트.
# 카메라 캘리브레이션 대시보드(PAGE)와 같은 디자인 언어를 로봇/로그 페이지에도
# 입혀 UI를 통일한다. PAGE는 자체 스타일이 방대해 그대로 두되, 변수 값을 동일하게
# 맞췄다(팔레트를 바꾸려면 PAGE의 :root와 이 상수 양쪽을 손볼 것). 테마 선택은
# localStorage 'theme' 키를 PAGE와 공유하므로, 대시보드에서 고른 테마를 로봇/로그
# 페이지가 그대로 따른다.
THEME_HEAD = """<script>
(function () {
  var t = null;
  try { t = localStorage.getItem('theme'); } catch (e) {}
  if (t !== 'light' && t !== 'dark') {
    t = (window.matchMedia &&
         window.matchMedia('(prefers-color-scheme: dark)').matches) ? 'dark' : 'light';
  }
  document.documentElement.setAttribute('data-theme', t);
})();
</script>
<style>
  :root {
    --bg:#F2F4F6; --card:#FFFFFF;
    --text:#191F28; --text2:#333D4B; --text3:#65707E;
    --line:#E5E8EB;
    --btn:#F2F4F6; --btn-hover:#E5E8EB; --seg:#EDEFF2;
    --field:#F9FAFB; --input:#FFFFFF;
    --track:rgba(0,0,0,.06);
    --shadow:0 1px 2px rgba(0,0,0,.04);
    --blue:#3182F6; --blue-dark:#1B64DA; --blue-bg:#E8F3FF;
    --red:#F04452; --red-bg:#FEF0F1;
    --green:#15C39A; --green-bg:#E7F9F4;
    --warn:#f5a623;
    --console:#17191C; --console-text:#D1D6DB;
    --mono:'SFMono-Regular',Consolas,'Liberation Mono',Menlo,monospace;
  }
  :root[data-theme="dark"] {
    --bg:#131619; --card:#1B1E22;
    --text:#E5E8EB; --text2:#B0B8C1; --text3:#8B95A1;
    --line:#2E3338;
    --btn:#2A2F36; --btn-hover:#343A42; --seg:#22262B;
    --field:#202429; --input:#16191D;
    --track:rgba(255,255,255,.09);
    --shadow:0 0 0 1px #2E3338;
    --blue:#4593FC; --blue-dark:#8CBCFF; --blue-bg:#1A2C44;
    --red:#FF6B78; --red-bg:#3A1E22;
    --green:#2AD8A8; --green-bg:#153229;
    --warn:#FBBF77;
    --console:#0E0F11; --console-text:#C9D1D9;
  }
  * { box-sizing:border-box; }
  body { font-family:-apple-system,BlinkMacSystemFont,'Pretendard','Malgun Gothic',
         system-ui,sans-serif; background:var(--bg); color:var(--text);
         margin:0; -webkit-font-smoothing:antialiased; }
  main { padding:16px 18px; max-width:880px; margin:0 auto; }
  .card { background:var(--card); border-radius:18px; padding:18px 20px;
          margin-bottom:16px; box-shadow:var(--shadow); }
  .card h2 { font-size:15px; font-weight:700; color:var(--text); margin:0 0 12px;
             letter-spacing:-.2px; }
  .btn { background:var(--btn); color:var(--text2); border:0; border-radius:12px;
         padding:11px 16px; font-size:14px; font-weight:600; font-family:inherit;
         cursor:pointer; transition:background .15s, color .15s; }
  .btn:hover { background:var(--btn-hover); }
  .btn.blue { background:var(--blue); color:#fff; }
  .btn.blue:hover { background:var(--blue-dark); }
  .btn.red { background:var(--red-bg); color:var(--red); }
  .btn.red:hover { filter:brightness(1.06); }
  .btn.active { background:var(--blue); color:#fff; }
  .rowbtns { display:flex; gap:8px; flex-wrap:wrap; }
  .chips { display:flex; gap:10px; flex-wrap:wrap; }
  .chip { background:var(--field); border:1px solid var(--line); border-radius:999px;
          padding:7px 14px; font-size:13px; color:var(--text2); display:flex;
          gap:8px; align-items:center; }
  .chip b { color:var(--text); font-weight:600; }
  .dot { width:9px; height:9px; border-radius:50%; background:var(--text3);
         display:inline-block; }
  .dot.on { background:var(--green); } .dot.off { background:var(--red); }
  .dot.warn { background:var(--warn); }
  .console { background:var(--console); color:var(--console-text); border-radius:16px;
             padding:12px; overflow:auto; font-family:var(--mono); font-size:12px;
             white-space:pre-wrap; line-height:1.55; }
  .hint { color:var(--text3); font-size:12px; line-height:1.6; }
  select, input, button { font-family:inherit; }
</style>"""

# 페이지 로컬 테마 토글 (rp-tabbar의 '테마' 버튼용). PAGE는 자체 토글이 있어
# 중복 방지를 위해 tabbar가 camera 탭에는 이 버튼을 넣지 않는다.
THEME_JS = """
function themeLabel(t){var b=document.getElementById('themeBtn');
  if(b)b.textContent=(t==='dark')?'라이트 모드':'다크 모드';}
function toggleTheme(){
  var next=document.documentElement.getAttribute('data-theme')==='dark'?'light':'dark';
  document.documentElement.setAttribute('data-theme',next);
  try{localStorage.setItem('theme',next);}catch(e){}
  themeLabel(next);}
themeLabel(document.documentElement.getAttribute('data-theme'));
"""


def tabbar(active, user_id=None):
    """Sticky top tab bar shared by all admin pages. Uses theme variables so it
    matches whichever page it's injected into (camera dashboard defines them in
    its own <head>; robot/logs get them from THEME_HEAD)."""
    def cls(name):
        return "active" if name == active else ""
    # 카메라 탭(PAGE)은 자체 테마 버튼이 있어 여기선 생략 (중복 방지).
    theme_btn = ("" if active == "camera"
                 else '<button class="themeBtn" id="themeBtn" '
                      'onclick="toggleTheme()">테마</button>')
    # 인라인 스타일로 직접 그린다 - 카메라 페이지(cctv.py PAGE)는 별도 스타일시트라
    # .chip 클래스가 없을 수 있음(CCTV팀 파일이라 여기서 건드리지 않음).
    # 로그아웃은 POST - <a href>로 두면 브라우저 프리페치에 로그아웃될 수 있다.
    user_chip = (f'<span style="margin-right:8px;padding:6px 12px;border-radius:999px;'
                 f'background:var(--field);border:1px solid var(--line);'
                 f'color:var(--text2);font-size:13px">👤 '
                 f'<b style="color:var(--text)">{html.escape(user_id)}</b></span>'
                 f'<form method="POST" action="/logout" style="display:inline;margin-right:8px">'
                 f'<button class="themeBtn" type="submit">로그아웃</button></form>'
                 if user_id else "")
    return f"""<nav class="rp-tabbar">
  <a class="{cls('camera')}" href="/">📷 카메라 캘리브레이션</a>
  <a class="{cls('robot')}" href="/robot">🤖 로봇 제어</a>
  <a class="{cls('logs')}" href="/logs">📜 로그 모니터</a>
  <span class="sp"></span>{user_chip}{theme_btn}
</nav>
<style>
.rp-tabbar{{position:sticky;top:0;z-index:99999;display:flex;gap:4px;align-items:center;
  background:var(--card);border-bottom:1px solid var(--line);padding:8px 12px;
  font-family:system-ui,sans-serif}}
.rp-tabbar a{{padding:8px 14px;color:var(--text3);text-decoration:none;font-size:13px;
  font-weight:600;border-radius:10px}}
.rp-tabbar a:hover{{background:var(--seg);color:var(--text2)}}
.rp-tabbar a.active{{color:var(--text);background:var(--seg)}}
.rp-tabbar .sp{{margin-left:auto}}
.themeBtn{{font-size:12px;font-weight:600;padding:7px 12px;border:1px solid var(--line);
  border-radius:10px;background:var(--card);color:var(--text2);cursor:pointer}}
.themeBtn:hover{{background:var(--field)}}
</style>"""


# Standalone robot-control panel (kept separate from the big camera dashboard so
# we don't disturb the existing calibration UI). Reuses the same /events SSE log.
ROBOT_PAGE = ("""<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>로봇 제어 · Admin</title>
__THEME_HEAD__
<style>
  /* 방향 패드 (D-pad). STOP 전용 버튼은 없다 - 방향 버튼을 누르는 동안
     이동하고 떼면 STOP이 나가는 조이스틱 모델이라 가운데는 비운다. */
  .pad { display:grid; grid-template-columns:repeat(3,92px); gap:8px;
         justify-content:start; }
  .pad .sp { visibility:hidden; }
  .pad button { height:66px; display:flex; align-items:center; justify-content:center;
                font-size:15px; padding:0; touch-action:none; user-select:none; }
  #log { height:280px; }
</style></head><body>
<main>
  <div class="card">
    <h2>상태</h2>
    <div class="chips">
      <span class="chip"><span id="srvDot" class="dot"></span>서버 <b id="srvState">연결 확인중…</b></span>
      <span class="chip"><span id="robDot" class="dot"></span>로봇 <b id="robState">-</b></span>
      <span class="chip">도색 <b id="paintState">-</b></span>
      <span class="chip">위치 <b id="poseState">-</b></span>
    </div>
  </div>
  <div class="card">
    <h2>비상 / 재개</h2>
    <div class="rowbtns">
      <button class="btn red" onclick="send('ESTOP')">■ 비상정지 (ESTOP)</button>
      <button class="btn" onclick="send('RESUME')">▶ 재개 (RESUME)</button>
      <button class="btn" onclick="send('CALIB_START')">◎ 캘리브레이션 시작</button>
    </div>
  </div>
  <div class="card">
    <h2>수동 조작 (조이스틱)</h2>
    <div class="pad">
      <span class="sp"></span>
      <button class="btn" data-cmd="FORWARD">▲ 전진</button>
      <span class="sp"></span>
      <button class="btn" data-cmd="TURN_LEFT">◀ 좌회전</button>
      <span class="sp"></span>
      <button class="btn" data-cmd="TURN_RIGHT">우회전 ▶</button>
      <span class="sp"></span>
      <button class="btn" data-cmd="BACKWARD">▼ 후진</button>
      <span class="sp"></span>
    </div>
    <p class="hint" style="margin:14px 0 0">※ 방향 버튼을 <b>누르는 동안</b> 이동하고, <b>떼면 정지</b>합니다(STOP). 관리자 명령은 경로 실행 중이어도 차단 없이 로봇에 전달됩니다.</p>
  </div>
  <div class="card">
    <h2>실시간 로그 (로봇 관련 위주)</h2>
    <div id="log" class="console"></div>
  </div>
</main>
<script>
__THEME_JS__
__LOG_SUBJECT_JS__
function send(c){
  fetch('/robot/cmd',{method:'POST',body:c})
    .then(r=>{if(!r.ok)add('[!] 전송 실패 (서버 미연결?) '+c);});
}
// 방향 버튼 = 조이스틱: 누르는 동안 방향 CMD, 떼면 STOP. 포인터 캡처로
// 버튼 밖에서 떼도 STOP이 확실히 나가게 한다(마우스/터치/펜 공통).
document.querySelectorAll('.pad button[data-cmd]').forEach(function(btn){
  var cmd=btn.getAttribute('data-cmd');
  btn.addEventListener('pointerdown',function(e){
    e.preventDefault();
    try{btn.setPointerCapture(e.pointerId);}catch(_){}
    btn.classList.add('active'); send(cmd);
  });
  function release(){
    if(btn.classList.contains('active')){ btn.classList.remove('active'); send('STOP'); }
  }
  btn.addEventListener('pointerup',release);
  btn.addEventListener('pointercancel',release);
});
const log=document.getElementById('log');
function add(line){
  const d=document.createElement('div');
  if(line.startsWith('[tap]'))d.style.color='var(--blue)';
  else if(line.includes('[WARN]')||line.includes('[ERROR]')||line.startsWith('[!]'))
    d.style.color='var(--warn)';
  d.textContent=line;
  log.appendChild(d);
  // DOM 상한 - 넘으면 오래된 줄부터 버린다. POS tap이 고주기로 흐르면 노드가
  // 순식간에 쌓여 페이지가 느려지므로, 로봇 페이지는 곁다리 로그라 작게 잡는다.
  while(log.childElementCount>200)log.removeChild(log.firstChild);
  log.scrollTop=log.scrollHeight;
}
// 서버 tap 로그를 파싱해 상단 상태 배너를 갱신한다.
const $=id=>document.getElementById(id);
function setDot(id,cls){$(id).className='dot '+cls;}
function jsonAfter(line,key){  // "... KEY {json}" 에서 {json} 추출
  const i=line.indexOf(key+' ');
  if(i<0)return null;
  try{return JSON.parse(line.slice(i+key.length+1));}catch(e){return null;}
}
function updateStatus(line){
  if(line.startsWith('[server] connected')){
    setDot('srvDot','on');$('srvState').textContent='연결됨';
  }
  else if(line.startsWith('[server] link down')){
    setDot('srvDot','off');$('srvState').textContent='끊김(재시도)';
    // 링크가 끊기면 서버 쪽 로그인 게이트도 잠긴다(rp_core). 새로고침하면
    // 로그인 화면으로 돌아가므로 사용자에게 미리 알려준다.
    add('[!] 서버 연결이 끊겼습니다 - 복구 후 다시 로그인이 필요합니다');
  }
  if(line.startsWith('[tap]')&&line.includes('STATUS')){
    const m=jsonAfter(line,'STATUS');
    if(m){
      const st=m.state||'-';
      $('robState').textContent=st;
      setDot('robDot', st==='ESTOPPED'||st==='ERROR'?'off':st==='MOVING'?'on':'warn');
      $('paintState').textContent=m.painting?'🖌 ON':'OFF';
    }
  }
  if(line.startsWith('[tap]')&&line.includes('POSE')){
    const m=jsonAfter(line,'POSE');
    if(m)$('poseState').textContent=`x=${m.x} y=${m.y} θ=${m.theta_deg}°`;
  }
}
const es=new EventSource('/events');
es.onmessage=e=>{
  updateStatus(e.data);  // 상태 배너는 필터와 무관하게 항상 갱신
  // 이 탭 로그는 로봇 위주로 - 카메라 전용 트래픽은 카메라 탭이나
  // 로그 모니터(/logs)에서 본다.
  if(logSubject(e.data)!=='camera')add(e.data);
};
</script>
</body></html>""".replace("__THEME_HEAD__", THEME_HEAD).replace("__THEME_JS__", THEME_JS)
             .replace("__LOG_SUBJECT_JS__", LOG_SUBJECT_JS))


# Log monitor: the same /events feed, but focused on server<->client traffic
# ([tap] relayed messages, [server] admin-link status, [srv] server-internal
# logf lines) with a peer filter. Camera/other logs are hidden unless
# "카메라/기타 로그도 보기" is checked.
LOGS_PAGE = ("""<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>로그 모니터 · Admin</title>
__THEME_HEAD__
<style>
  main { padding-bottom:0; }
  .bar { display:flex; gap:14px; align-items:center; flex-wrap:wrap;
         margin-bottom:12px; font-size:13px; color:var(--text2); }
  .bar select { background:var(--input); color:var(--text); border:1px solid var(--line);
                border-radius:10px; padding:8px 10px; font-size:13px; cursor:pointer; }
  .bar label { display:flex; align-items:center; gap:6px; cursor:pointer; }
  .bar input[type=checkbox] { accent-color:var(--blue); width:15px; height:15px; }
  #count { color:var(--text3); }
  #log { height:calc(100vh - 150px); }
  .tap{color:var(--blue)}.srv{color:var(--green)}.warn{color:var(--warn)}
  .muted{color:var(--text3)}
</style></head><body>
<main>
  <div class="card">
    <div class="bar">
      <span>대상 필터:
        <select id="f" onchange="render()">
          <option value="">전체 (서버 tap)</option>
          <option value="ROBOT">ROBOT</option>
          <option value="QT">QT</option>
          <option value="CCTV">CCTV</option>
        </select>
      </span>
      <label><input type="checkbox" id="raw" onchange="render()"> 카메라/기타 로그도 보기</label>
      <button class="btn" onclick="lines=[];render()">지우기</button>
      <span id="count"></span>
    </div>
    <div id="log" class="console"></div>
  </div>
</main>
<script>
__THEME_JS__
const log=document.getElementById('log'), f=document.getElementById('f'),
      raw=document.getElementById('raw'), count=document.getElementById('count');
// MAX_BUF: 필터를 껐다 켤 때 되살릴 수 있는 원본 줄 보관량.
// MAX_DOM: 실제로 화면에 붙여두는 노드 수 (이게 크면 스크롤/렌더가 느려진다).
// 로그 모니터는 전용 화면이라 로봇 페이지(200)보다는 넉넉히 준다.
const MAX_BUF=1000, MAX_DOM=400;
let lines=[];
function keep(l){
  if(!raw.checked && !l.startsWith('[tap]') && !l.startsWith('[server]')
     && !l.startsWith('[bridge]') && !l.startsWith('[srv]')) return false;
  const q=f.value; if(q && !l.includes(q)) return false;
  return true;
}
function cls(l){
  if(l.startsWith('[tap]'))return 'tap';
  if(l.startsWith('[server]')||l.startsWith('[bridge]'))return 'srv';
  if(l.startsWith('[srv]'))return (l.includes('[WARN]')||l.includes('[ERROR]'))?'warn':'srv';
  if(l.startsWith('[!]'))return 'warn';
  return 'muted';}
function render(){
  log.innerHTML='';
  const shown=lines.filter(keep).slice(-MAX_DOM);
  for(const l of shown){const d=document.createElement('div');
    d.className=cls(l); d.textContent=l; log.appendChild(d);}
  count.textContent=shown.length+' / '+lines.length+' 줄';
  log.scrollTop=log.scrollHeight;
}
function add(l){
  lines.push(l); if(lines.length>MAX_BUF)lines.shift();
  if(keep(l)){const d=document.createElement('div');
    d.className=cls(l); d.textContent=l; log.appendChild(d);
    while(log.childElementCount>MAX_DOM)log.removeChild(log.firstChild);
    count.textContent=(log.childElementCount)+' / '+lines.length+' 줄';
    log.scrollTop=log.scrollHeight;}
}
new EventSource('/events').onmessage=e=>add(e.data);
</script>
</body></html>""".replace("__THEME_HEAD__", THEME_HEAD).replace("__THEME_JS__", THEME_JS))


# 로그인 게이트. 서버(9000)에 LOGIN이 통과하기 전에는 관리자 창의 어떤 기능도
# 열지 않고 이 화면만 보여준다 (do_GET/do_POST가 rp_core.get_logged_in_user()로
# 판정 - 브라우저 JS 상태가 아니라 서버 측 상태라 페이지 조작으로 우회 불가).
#
# 왜 로그인을 먼저 받나: 캘리브레이션 결과는 "그 시점에 로그인된 계정"에 저장된다
# (../src/router.cpp handleHMatrix). 로그인 없이 캘리를 끝내면 세션에만 남고
# 서버 재시작 시 사라지는데, 나중에 로그인해도 소급 저장되지 않는다. 순서를
# 사람이 기억하게 두는 대신 구조로 강제한다.
LOGIN_PAGE = ("""<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>로그인 · Admin</title>
__THEME_HEAD__
<style>
  main { max-width:420px; padding-top:12vh; }
  .field { display:block; margin-bottom:10px; }
  .field input { width:100%; padding:12px 14px; border:1px solid var(--line);
                 border-radius:12px; background:var(--input); color:var(--text);
                 font-size:15px; }
  .field input:focus { outline:2px solid var(--blue); outline-offset:-1px; }
  #msg { margin:12px 0 0; font-size:13px; min-height:19px; }
  #msg.err { color:var(--red); }
  #msg.ok { color:var(--green); }
  .title { font-size:19px; font-weight:700; margin:0 0 6px; letter-spacing:-.3px; }
  .sub { color:var(--text3); font-size:13px; line-height:1.6; margin:0 0 18px; }
</style></head><body>
<main>
  <div class="card">
    <p class="title">Road-Painter 관리자 창</p>
    <p class="sub">캘리브레이션 결과는 <b>로그인한 계정</b>에 저장됩니다.
      먼저 로그인해야 설치 작업 결과가 계정에 남습니다.</p>
    <form id="f" autocomplete="on">
      <label class="field">
        <input id="loginId" name="username" placeholder="아이디" autocomplete="username" autofocus>
      </label>
      <label class="field">
        <input id="loginPw" name="password" type="password" placeholder="비밀번호"
               autocomplete="current-password">
      </label>
      <button class="btn blue" type="submit" style="width:100%" id="btn">로그인</button>
    </form>
    <p id="msg"></p>
    <p class="hint" id="srvHint">서버 연결 확인 중…</p>
  </div>
</main>
<script>
__THEME_JS__
const msg=document.getElementById('msg'), btn=document.getElementById('btn');
function say(t,cls){msg.textContent=t;msg.className=cls||'';}
document.getElementById('f').addEventListener('submit',function(e){
  e.preventDefault();
  const id=document.getElementById('loginId').value.trim();
  const pw=document.getElementById('loginPw').value;
  if(!id||!pw){say('아이디와 비밀번호를 입력하세요.','err');return;}
  btn.disabled=true; say('확인 중…');
  fetch('/server/login',{method:'POST',body:JSON.stringify({id:id,pw:pw})})
    .then(r=>r.json().then(d=>({ok:r.ok,d:d})))
    .then(function(r){
      btn.disabled=false;
      if(r.ok&&r.d.ok){
        // 서버가 LOGIN_OK를 돌려줘 게이트가 열린 상태 - 원래 가려던 곳으로.
        say('로그인 성공. 이동 중…','ok');
        location.href=new URLSearchParams(location.search).get('next')||'/';
      } else {
        say(r.d.reason||'로그인에 실패했습니다.','err');
        document.getElementById('loginPw').select();
      }
    })
    .catch(function(){ btn.disabled=false; say('요청을 보내지 못했습니다.','err'); });
});
// 서버(9000) 링크가 끊겨 있으면 로그인 자체가 불가능하므로 미리 알려준다.
new EventSource('/events').onmessage=function(e){
  const h=document.getElementById('srvHint');
  if(e.data.startsWith('[server] connected')) h.textContent='서버 연결됨 — 로그인할 수 있습니다.';
  else if(e.data.startsWith('[server] link down')) h.textContent='⚠ 서버(9000) 연결 끊김 — 서버가 실행 중인지 확인하세요.';
};
</script>
</body></html>""".replace("__THEME_HEAD__", THEME_HEAD).replace("__THEME_JS__", THEME_JS))


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    # 로그인 없이도 열어두는 경로. /login은 게이트 그 자체, /server/login은 거기서
    # 쓰는 API, /logout은 없는 세션을 지우는 것도 안전해야 하고, /events는 로그인
    # 화면이 서버 링크 상태를 보여주는 데 쓴다 (SSE는 관리자 창 로그 피드라
    # 민감정보가 아니며, 비밀번호는 마스킹된다).
    PUBLIC_PATHS = ("/login", "/server/login", "/logout", "/events")
    COOKIE_NAME = "rp_admin"

    def _session_token(self):
        raw = self.headers.get("Cookie")
        if not raw:
            return None
        try:
            jar = SimpleCookie()
            jar.load(raw)
        except CookieError:
            return None
        morsel = jar.get(self.COOKIE_NAME)
        return morsel.value if morsel else None

    def _current_user(self):
        """이 요청을 보낸 브라우저의 로그인 계정 (없으면 None)."""
        return rp_core.session_user(self._session_token())

    def _gate(self):
        """로그인 안 됐으면 /login으로 돌려보내고 True를 반환 (호출부는 즉시 return)."""
        if urlparse(self.path).path in self.PUBLIC_PATHS:
            return False
        if self._current_user() is not None:
            return False
        if self.command == "POST":
            # API 호출은 리다이렉트 대신 401 - fetch가 로그인 HTML을 받아
            # 성공으로 오해하는 일이 없게 한다.
            self._send_json({"ok": False, "reason": "로그인이 필요합니다."}, 401)
        else:
            nxt = quote(self.path, safe="")
            self.send_response(302)
            self.send_header("Location", f"/login?next={nxt}")
            self.end_headers()
        return True

    def _send_json(self, obj, status=200):
        body = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self._gate():
            return
        path = urlparse(self.path).path
        if path == "/login":
            # 이미 로그인돼 있으면 로그인 화면을 다시 보여줄 이유가 없다.
            if self._current_user() is not None:
                self.send_response(302)
                self.send_header("Location", "/")
                self.end_headers()
                return
            body = LOGIN_PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif path in ("/", "/robot", "/logs"):
            page, active = {"/": (cctv.PAGE, "camera"),
                            "/robot": (ROBOT_PAGE, "robot"),
                            "/logs": (LOGS_PAGE, "logs")}[path]
            # 탭 바만 body 시작 지점에 주입 (기존 페이지 내용은 건드리지 않음)
            bar = tabbar(active, self._current_user())
            body = page.replace("<body>", "<body>" + bar, 1).encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            # 현재 서버 링크 상태 스냅샷 1줄. 이게 없으면 서버 링크가 이미
            # 붙어있는 상태에서 페이지를 열 때 "[server] connected" 줄을 놓쳐
            # 상태 배너가 "연결 확인중…"에 멈춘다 (실제론 붙어있는데 안 붙은
            # 것처럼 보임). 웹은 클라이언트(QT/CCTV/로봇)가 하나도 없어도 항상
            # 열려야 하므로, 물어보지 않아도 바로 알 수 있게 밀어준다.
            # _server_sock은 server_link_loop가 재대입하므로 반드시 rp_core를
            # 통해 현재값을 읽는다 (from-import는 초기값 None에 고정돼 갱신 안 됨).
            with rp_core._server_lock:
                linked = rp_core._server_sock is not None
            snapshot = (f"[server] connected as ADMIN to {rp_core.SERVER_HOST}:{rp_core.SERVER_PORT}"
                        if linked else
                        "[server] link down (아직 미연결); retry in 3s")
            q = queue.Queue()
            with subs_lock:
                # 그동안 쌓인 로그 히스토리를 먼저 재생 → 스냅샷 → 이후 라이브.
                # subs_lock 안에서 재생+등록을 함께 해, broadcast가 이 사이에
                # 끼어들어 라인을 중복 전달하거나 빠뜨리는 일이 없게 한다.
                for past in log_history:
                    q.put(past)
                q.put(snapshot)
                subscribers.append(q)
            try:
                while True:
                    line = q.get()
                    self.wfile.write(f"data: {line}\n\n".encode())
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                with subs_lock:
                    if q in subscribers:
                        subscribers.remove(q)
        elif path == "/hg_reference.jpg":
            try:
                with open(cctv.HG_REFERENCE_PATH, "rb") as f:
                    body = f.read()
            except OSError:
                self.send_response(404)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
        elif path == "/hg_meta":
            try:
                with open(cctv.HG_META_PATH, "rb") as f:
                    body = f.read()
            except OSError:
                body = b"{}"
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
        elif path == "/hg_experiment/status":
            body = json.dumps(cctv.hg_experiment_status(), ensure_ascii=False).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
        elif path == "/calib/channel":
            # 현재 캘리 대상 채널 + 채널별 진행 상태 (UI 표시용)
            self._send_json(cctv.calib_channel_status())
        elif path == "/hg_experiment/export":
            with cctv.hg_experiment_lock:
                latest = cctv.hg_experiment["last_export"]
            if not latest:
                self.send_response(404)
                self.end_headers()
                return
            try:
                with open(f"{cctv.HG_EXPERIMENT_DIR}/{latest['json']}", "rb") as f:
                    body = f.read()
            except OSError:
                self.send_response(404)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Disposition", f"attachment; filename={latest['json']}")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        if self._gate():
            return
        if self.path == "/cmd":
            length = int(self.headers.get("Content-Length", 0))
            cmd = self.rfile.read(length).decode().strip()
            cctv.send_command(cmd)
            self.send_response(200)
            self.end_headers()
        elif self.path == "/robot/cmd":
            # Forward a robot CMD (ESTOP/RESUME/FORWARD/.../CALIB_START) to the
            # relay server, which relays it to the robot.
            length = int(self.headers.get("Content-Length", 0))
            cmd = self.rfile.read(length).decode().strip()
            ok = server_send("CMD", {"cmd": cmd})
            self.send_response(200 if ok else 503)
            self.end_headers()
        elif self.path == "/server/login":
            # 사용자 로그인을 ADMIN 연결로 서버에 전달하고 응답까지 기다린다.
            # 서버는 이 role에 LOGIN_OK/LOGIN_FAIL을 돌려주고, 성공 시 캘리브레이션
            # 저장 대상 계정(currentUser_)이 이 사용자로 바뀐다
            # (../src/router.cpp handleLogin). 성공하면 rp_core가 게이트를 연다.
            length = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(length) if length else b"{}"
            try:
                data = json.loads(raw)
            except (ValueError, json.JSONDecodeError):
                self._send_json({"ok": False, "reason": "잘못된 요청 형식"}, 400)
                return
            uid, pw = data.get("id", ""), data.get("pw", "")
            if not uid or not pw:
                self._send_json({"ok": False, "reason": "아이디와 비밀번호를 입력하세요."}, 400)
                return
            # 응답이 도착하기 전에 대기를 걸어둔다 (빠른 응답을 놓치지 않도록).
            rp_core.begin_login()
            # 로그 피드는 관리자 페이지를 연 누구나 보므로 비밀번호는 가린다.
            if not server_send("LOGIN", {"id": uid, "pw": pw},
                               log_payload={"id": uid, "pw": "***"}):
                self._send_json({"ok": False,
                                 "reason": "서버(9000)에 연결되어 있지 않습니다."}, 503)
                return
            ok, reason = rp_core.wait_login()
            if not ok:
                self._send_json({"ok": False, "reason": reason})
                return
            # 이 브라우저 전용 세션 발급. Max-Age를 주지 않아 세션 쿠키가 되므로
            # 브라우저를 닫으면 사라진다(= 다시 열면 재로그인).
            token = rp_core.create_session(uid)
            body = json.dumps({"ok": True}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Set-Cookie",
                             f"{self.COOKIE_NAME}={token}; Path=/; HttpOnly; SameSite=Lax")
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/logout":
            # 세션을 지우고 쿠키를 만료시킨 뒤 로그인 화면으로. 세션이 없어도
            # (이미 로그아웃/만료) 그냥 성공으로 처리한다.
            rp_core.drop_session(self._session_token())
            self.send_response(302)
            self.send_header("Location", "/login")
            self.send_header("Set-Cookie",
                             f"{self.COOKIE_NAME}=; Path=/; HttpOnly; SameSite=Lax; "
                             "Expires=Thu, 01 Jan 1970 00:00:00 GMT")
            self.end_headers()
        elif self.path == "/robot/path":
            # Forward a test PATH ({"segments":[...]}) to the robot via server.
            length = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(length) if length else b"{}"
            try:
                data = json.loads(raw)
                ok = server_send("PATH", {"segments": data.get("segments", [])})
                self.send_response(200 if ok else 503)
            except (ValueError, json.JSONDecodeError):
                self.send_response(400)
            self.end_headers()
        elif self.path == "/calib/channel":
            # 캘리브레이션 대상 채널 변경 (프로토콜 v0.4). 채널마다 렌즈 방향이
            # 달라 번들이 완전히 다르므로, 이 값이 틀리면 결과가 엉뚱한 채널 슬롯에
            # 저장된다 - 그것도 에러 없이 조용히.
            length = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(length) if length else b"{}"
            try:
                data = json.loads(raw)
            except (ValueError, json.JSONDecodeError):
                self._send_json({"ok": False, "reason": "잘못된 요청 형식"}, 400)
                return
            ok, reason = cctv.set_calib_channel(data.get("ch"))
            self._send_json({"ok": ok, "reason": reason,
                             "status": cctv.calib_channel_status()},
                            200 if ok else 400)
        elif self.path in ("/hg_experiment/start", "/hg_experiment/stop", "/hg_experiment/result"):
            try:
                length = int(self.headers.get("Content-Length", 0))
                raw = self.rfile.read(length) if length else b"{}"
                data = json.loads(raw)
                response = cctv.hg_experiment_post(self.path, data)
                body = json.dumps(response, ensure_ascii=False).encode()
                self.send_response(200)
            except (ValueError, TypeError, KeyError, json.JSONDecodeError) as e:
                body = json.dumps({"error": str(e)}, ensure_ascii=False).encode()
                self.send_response(400)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_response(404)
            self.end_headers()


class ThreadingHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main():
    # 카메라/스냅샷 수신 + CCTV 통역 브리지는 cctv 모듈, ADMIN 서버 링크는 rp_core.
    threading.Thread(target=cctv.tcp_server, daemon=True).start()
    threading.Thread(target=cctv.snapshot_server, daemon=True).start()
    threading.Thread(target=server_link_loop, daemon=True).start()
    threading.Thread(target=cctv.cctv_link_loop, daemon=True).start()
    httpd = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    broadcast(f"web GUI: http://0.0.0.0:{HTTP_PORT}")
    broadcast(f"robot control: http://0.0.0.0:{HTTP_PORT}/robot")
    httpd.serve_forever()


if __name__ == "__main__":
    main()
