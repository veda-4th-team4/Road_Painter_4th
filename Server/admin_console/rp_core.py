#!/usr/bin/env python3
"""Road-Painter 관리자 창 - 공통 코어.

설정값 + 로그 브로드캐스트/SSE 상태 + 중앙 서버(role=ADMIN) 링크를 담는다.
web_gui.py(HTTP·로봇·로그 UI)와 cctv.py(카메라·캘리브레이션)가 공유하는 기반.

의존성은 단방향이다:  rp_core  <-  cctv  <-  web_gui
이 모듈은 cctv/web_gui를 절대 import하지 않는다 (순환 방지).
"""
import json
import logging
import os
import secrets
import socket
import ssl
import sys
import threading
import time
from collections import deque
from logging.handlers import RotatingFileHandler

# 이 코드는 로그 문구에 —/⚠️/🔴 같은 non-ASCII 문자를 흔히 쓴다. 리눅스(Pi)에서는
# stdout이 기본 UTF-8이라 문제가 없었지만, 한국어 로캘 Windows에서 그대로 돌리면
# 콘솔 기본 인코딩(cp949)이 그 문자들을 못 써서 broadcast() 가 UnicodeEncodeError로
# 죽는다(예: cctv_link_loop 스레드가 접속 로그 한 줄 찍다가 죽음). stdout/stderr을
# 켜자마자 UTF-8로 못박아 로캘에 흔들리지 않게 한다.
for _stream in (sys.stdout, sys.stderr):
    if hasattr(_stream, "reconfigure"):
        _stream.reconfigure(encoding="utf-8", errors="backslashreplace")

TCP_PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 6000
HTTP_PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 8081
SNAPSHOT_PORT = int(sys.argv[3]) if len(sys.argv) > 3 else 6001
subscribers = []
subs_lock = threading.Lock()
# 최근 로그 링 버퍼: 로그 모니터(/logs)나 로봇 페이지를 "열고 있지 않아도"
# 이벤트가 쌓이게 한다. 새 구독자가 /events에 붙으면 이 히스토리를 먼저
# 재생해줘서, 창을 열자마자 그동안의 이벤트가 보인다 (예전엔 연 시점부터의
# 로그만 받아서, 창에 들어가 있어야만 쌓이는 것처럼 보였다).
# subs_lock으로 broadcast()의 append와 /events의 재생을 직렬화해 중복/누락 방지.
#
# 이 값은 "페이지를 열 때 브라우저로 한꺼번에 밀어넣는 줄 수"이기도 하다. POS tap이
# 15~30Hz로 흐르는 상황에서 크게 잡으면 창을 열 때마다 수천 줄이 DOM에 쏟아져
# 페이지가 눈에 띄게 버벅인다. 최근 맥락을 보여주기에 충분한 선에서 작게 잡는다.
LOG_HISTORY_MAX = 300
log_history = deque(maxlen=LOG_HISTORY_MAX)

# ---------------------------------------------------------------------------
# 디스크 로그(gui.log): 크기 상한을 두고 회전시킨다.
#
# 예전엔 start.sh가 stdout을 gui.log로 `>>` 리다이렉트하기만 해서 파일이 무한정
# 커졌다(실측 8.4MB / 13만 줄). Pi의 SD카드를 갉아먹으므로 여기서 직접 회전시키고,
# start.sh는 더 이상 stdout을 이 파일로 보내지 않는다(이중 기록·회전 깨짐 방지).
# 총 사용량 상한 = LOG_FILE_MAX_BYTES * (LOG_FILE_BACKUPS + 1) ≈ 8MB.
# ---------------------------------------------------------------------------
LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gui.log")
LOG_FILE_MAX_BYTES = 2 * 1024 * 1024   # 2MB마다 회전
LOG_FILE_BACKUPS = 3                    # gui.log.1 ~ gui.log.3 보관

_file_log = logging.getLogger("rp_admin")
_file_log.setLevel(logging.INFO)
_file_log.propagate = False
try:
    _handler = RotatingFileHandler(LOG_FILE, maxBytes=LOG_FILE_MAX_BYTES,
                                   backupCount=LOG_FILE_BACKUPS, encoding="utf-8")
    # 콘솔 출력과 같은 원문 그대로 (줄 자체에 이미 [tap]/[srv] 같은 접두어가 있음)
    _handler.setFormatter(logging.Formatter("%(message)s"))
    _file_log.addHandler(_handler)
except OSError as e:  # 디스크 풀/권한 등 - 파일 로그만 포기하고 계속 동작
    print(f"[!] gui.log 열기 실패({e}) - 파일 로그 없이 계속합니다", flush=True)


# ---------------------------------------------------------------------------
# 로그 한 줄이 로봇/카메라/QT 중 어디에 관한 것인지 추정하는 JS 조각.
# /events는 모든 화면(카메라·로봇·로그 모니터 탭)이 공유하는 단일 스트림이라,
# 원문 로그 줄 자체에는 발신 주체 태그가 없다([tap] 줄만 peer가 명확하고,
# [srv]로 중계되는 서버 내부 logf 줄은 그냥 텍스트다). 그래서 키워드 기반
# 휴리스틱으로 "위주로" 걸러줄 뿐, 완벽한 분류는 아니다 - 전체 로그가 필요하면
# 로그 모니터 탭(/logs)을 쓴다.
# cctv.py(카메라 탭)와 web_gui.py(로봇 탭) 양쪽에서 그대로 삽입해 쓰는 공용
# 코드라 여기 한 곳에 둔다.
# ---------------------------------------------------------------------------
LOG_SUBJECT_JS = """
const ROBOT_LOG_MARKERS = ['ROBOT','STATUS:','painting=','ESTOP','RESUME',
  'READY(','PATH_DONE','경로 이탈','BLUEPRINT','DRAW_DONE','CALIB_START'];
const CAMERA_LOG_MARKERS = ['CCTV','MARKER LOST','captured view','BOARD_CONFIG',
  'session started','params updated','SUCCESS rms','COMPUTING with',
  '[calib-K]','HG_','CALIB_K','SET_CAM_IP'];
function logSubject(l) {
  if (l.startsWith('[tap]')) {
    if (l.includes('ROBOT')) return 'robot';
    if (l.includes('QT')) return 'qt';
    if (l.includes('CCTV')) return 'camera';
    return 'other';
  }
  if (ROBOT_LOG_MARKERS.some(k => l.includes(k))) return 'robot';
  if (l.includes('QT')) return 'qt';
  if (CAMERA_LOG_MARKERS.some(k => l.includes(k))) return 'camera';
  return 'other';
}
"""


def broadcast(line):
    print(line, flush=True)          # 콘솔(직접 실행 시) - start.sh는 /dev/null로 보냄
    _file_log.info(line)             # gui.log (크기 상한 + 회전)
    with subs_lock:
        log_history.append(line)  # 창을 안 열고 있어도 이벤트가 쌓이도록 보관
        for q in subscribers:
            q.put(line)


# ---------------------------------------------------------------------------
# Main server (C++ TLS relay) link — role=ADMIN.
#
# The admin console keeps its existing direct camera link (above) AND opens a
# second connection to the production relay server (port 9000). Registered as
# role ADMIN it (1) receives a TAP copy of every QT/ROBOT/CCTV message the
# server relays — streamed into the same dashboard log feed — and (2) can push
# CMD/PATH to the robot through the server. Auto-reconnects if the server is
# down. See ../src/protocol.hpp (ADMIN role).
# ---------------------------------------------------------------------------
SERVER_HOST = os.environ.get("RP_SERVER_HOST", "127.0.0.1")
SERVER_PORT = int(os.environ.get("RP_SERVER_PORT", "9000"))

# 카메라 채널 수 (프로토콜 v0.4). PNM-C16083RVQ 는 4채널, PNO-A9081R 은 1채널.
#
# 채널마다 렌즈 방향이 달라 캘리브레이션 번들(K/D/H)이 완전히 다르므로, 관리자
# 창은 "지금 어느 채널을 캘리하는 중인가"를 들고 있어야 한다. 4채널 카메라에서
# 이걸 안 실어 보내면 네 채널이 전부 서버의 채널 1 슬롯에 덮어써져 **마지막에
# 캘리한 하나만 남는다.**
#
# 1로 두면 채널 선택 UI 가 사라지고 항상 채널 1로 보낸다 = v0.3과 동일 동작.
# (서버도 ch 가 없거나 1이면 예전 저장 슬롯을 쓴다 — config.sh 로 조정)
CAM_CHANNELS = max(1, min(8, int(os.environ.get("RP_CAM_CHANNELS", "4"))))

_server_lock = threading.Lock()
_server_sock = None   # ssl socket while connected, else None
_server_seq = 0

# ---------------------------------------------------------------------------
# Login gate: the whole admin console is locked behind a server LOGIN until
# this is set. Tracked here (not just in the browser) because the HTTP
# handlers (web_gui.py do_GET/do_POST) must decide server-side whether to
# serve real pages or the login form - relying on client-side JS state would
# let anyone who edits the page's JS bypass the gate.
# ---------------------------------------------------------------------------
_login_lock = threading.Lock()
_logged_in_user = None   # 중앙 서버가 현재 로그인으로 아는 계정 (표시/참고용)
_login_result = None     # (ok, reason) of the in-flight attempt
_login_event = threading.Event()

# 브라우저 세션: 로그인 성공 시 발급한 토큰 -> 계정 id.
# 프로세스 전역 플래그 하나로 게이트를 열면 "한 번 로그인하면 같은 네트워크의 아무
# 브라우저나 다 들어오고, 창을 닫아도 계속 열린 상태"가 된다. 브라우저마다 토큰을
# 따로 쥐게 해서 로그아웃/창 닫기가 실제로 로그아웃이 되게 한다.
# (쿠키는 Max-Age 없이 내려 세션 쿠키로 만들므로 브라우저를 닫으면 사라진다)
_sessions = {}


def get_logged_in_user():
    """중앙 서버 기준 현재 로그인 계정 (게이트 판정용이 아님 - session_user 사용)."""
    with _login_lock:
        return _logged_in_user


def create_session(user_id):
    token = secrets.token_urlsafe(32)
    with _login_lock:
        _sessions[token] = user_id
    return token


def session_user(token):
    """유효한 세션이면 계정 id, 아니면 None."""
    if not token:
        return None
    with _login_lock:
        return _sessions.get(token)


def drop_session(token):
    with _login_lock:
        _sessions.pop(token, None)


def clear_login():
    """Re-lock the gate (server link dropped): 모든 브라우저 세션을 무효화."""
    global _logged_in_user
    with _login_lock:
        _logged_in_user = None
        _sessions.clear()


def begin_login():
    """Arm the waiter BEFORE sending LOGIN, so a fast reply can't be missed."""
    global _login_result
    with _login_lock:
        _login_result = None
    _login_event.clear()


def finish_login(ok, reason=None, user_id=None):
    """Called from the server link thread when LOGIN_OK/LOGIN_FAIL arrives."""
    global _login_result, _logged_in_user
    with _login_lock:
        _login_result = (ok, reason)
        if ok:
            _logged_in_user = user_id
    _login_event.set()


def wait_login(timeout=5.0):
    """Block until the relay server answers the LOGIN. -> (ok, reason)."""
    if not _login_event.wait(timeout):
        return False, "서버 응답 없음 (시간 초과)"
    with _login_lock:
        return _login_result or (False, "알 수 없는 오류")


def _server_send_raw(obj):
    with _server_lock:
        s = _server_sock
    if s is None:
        broadcast("[server] not connected; message dropped")
        return False
    try:
        s.sendall((json.dumps(obj) + "\n").encode())
        return True
    except OSError as e:
        broadcast(f"[server] send failed: {e}")
        return False


def server_send(mtype, payload, log_payload=None):
    """Send a {type, seq, payload} message to the relay server (as ADMIN).

    log_payload: what to show in the dashboard log instead of the real payload.
    Use it when the payload holds a secret (LOGIN pw) — the log feed is visible
    to anyone with the admin page open, so the password must not land there.
    """
    global _server_seq
    _server_seq += 1
    ok = _server_send_raw({"type": mtype, "seq": _server_seq, "payload": payload})
    if ok:
        shown = payload if log_payload is None else log_payload
        broadcast(f"[server] >> {mtype} {json.dumps(shown, ensure_ascii=False)}")
    return ok


def _fmt_tap(payload):
    """Render a TAP message as one compact dashboard log line."""
    d = payload.get("dir", "?")
    peer = payload.get("peer", "?")
    m = payload.get("msg", {}) or {}
    arrow = f"{peer}->SRV" if d == "IN" else f"SRV->{peer}"
    mtype = m.get("type", "?")
    summary = json.dumps(m.get("payload", {}), ensure_ascii=False)
    if len(summary) > 200:
        summary = summary[:200] + "…"
    return f"[tap] {arrow} {mtype} {summary}"


# 주행 캘리(오도메트리) 진행도용 구조화 로그.
#
# 지점별 성공/실패는 서버가 ADMIN에게 따로 통지하지 않는다(sendTo("ADMIN") 0건).
# 그런데 필요 없다 — 서버는 ADMIN에게 IN/OUT 모든 메시지의 사본을 TAP으로
# 흘려주므로(tls_server.cpp:193/254) CCTV가 올린 CALIB_CAPTURE_OK/FAIL이 이미
# 이 링크로 들어온다. 그래서 서버(C++)를 고치지 않고 여기서 골라낸다.
#
# [tap] 줄을 JS가 직접 파싱하지 않는 이유: _fmt_tap()은 사람이 읽는 요약이라
# payload를 200자에서 자른다. 진행도 표는 잘린 값을 그대로 표시하게 된다.
_ODO_TAP_TYPES = ("CALIB_START", "CALIB_CAPTURE", "CALIB_CAPTURE_OK",
                  "CALIB_CAPTURE_FAIL", "CALIB_DONE", "CALIB_FAIL",
                  "CALIB_CANCEL", "CALIB_STOPPED", "H_MATRIX",
                  # 채널 간 정합(registration, 2026-08-15 신설) - 같은 이유로
                  # 여기 얹는다. 별도 튜플/[reg] 접두어를 새로 만들지 않는 이유는
                  # 이미 H_MATRIX가 이 목록에 있어서다 - 정합 결과도 그 안의
                  # reg_* 필드로 오므로, 이미 필터링되는 메시지 위에 타입만
                  # 몇 개 더 얹으면 된다. JS 쪽(cctv.py)은 "[odo] {json}" 줄을
                  # 파싱한 뒤 ev.type으로 다시 갈라 쓰므로 접두어가 "odo"라는
                  # 이름과 무관하게 그대로 동작한다.
                  "REGISTER_CAPTURE", "REGISTER_CAPTURE_OK",
                  "REGISTER_CAPTURE_FAIL", "REGISTER_FAIL", "REGISTER_STOPPED")


def _odo_tap_line(tap):
    """주행 캘리 관련 TAP이면 '[odo] {json}' 한 줄, 아니면 None."""
    m = tap.get("msg") or {}
    mtype = m.get("type")
    if mtype not in _ODO_TAP_TYPES:
        return None
    p = dict(m.get("payload") or {})
    # H_MATRIX의 calib 번들은 3x3 행렬 셋에 K/D까지라 로그 한 줄에 넣을 물건이
    # 아니다. 진행도 표에 필요한 건 "번들이 나갔다"는 사실과 채널뿐이다.
    if "calib" in p:
        p["calib"] = "(번들 %dB)" % len(json.dumps(p["calib"], ensure_ascii=False))
    return "[odo] " + json.dumps(
        {"dir": tap.get("dir"), "peer": tap.get("peer"), "type": mtype, "payload": p},
        ensure_ascii=False)


def server_link_loop():
    """Maintain the ADMIN connection to the relay server; reconnect forever."""
    global _server_sock
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    # Local self-signed admin link on the same host — skip hostname/CA checks.
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    while True:
        try:
            raw = socket.create_connection((SERVER_HOST, SERVER_PORT), timeout=5)
            s = ctx.wrap_socket(raw, server_hostname=SERVER_HOST)
            # connect용 5초 타임아웃을 해제해 recv가 블로킹되게 한다. 안 그러면
            # 클라이언트가 하나도 없어 TAP 트래픽이 없을 때 recv가 5초마다
            # 타임아웃 -> 링크 해제/재접속을 무한 반복해(상태 배너가 깜빡임),
            # 서버 로그에도 ADMIN 접속/해제가 계속 쌓인다.
            s.settimeout(None)
            with _server_lock:
                _server_sock = s
            s.sendall((json.dumps({"type": "HELLO", "seq": 0,
                                   "payload": {"role": "ADMIN"}}) + "\n").encode())
            broadcast(f"[server] connected as ADMIN to {SERVER_HOST}:{SERVER_PORT}")
            buf = ""
            while True:
                chunk = s.recv(4096)
                if not chunk:
                    break
                buf += chunk.decode(errors="replace")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        msg = json.loads(line)
                    except ValueError:
                        continue
                    mtype = msg.get("type")
                    if mtype == "TAP":
                        tap = msg.get("payload", {})
                        broadcast(_fmt_tap(tap))
                        odo = _odo_tap_line(tap)
                        if odo:
                            broadcast(odo)
                    elif mtype == "LOG":
                        # 서버 내부 로그(logf) 중계 - 이미 "HH:MM:SS [INFO] ..."
                        # 형태라 그대로 붙인다. [srv] 접두어로 로그 모니터가
                        # tap과 구분해 표시/필터.
                        broadcast(f"[srv] {(msg.get('payload') or {}).get('line', '')}")
                    elif mtype == "ACK":
                        broadcast(f"[server] {msg.get('payload', {}).get('msg', 'ack')}")
                    else:
                        broadcast(f"[server] << {mtype} "
                                  f"{json.dumps(msg.get('payload', {}), ensure_ascii=False)}")
                        payload = msg.get("payload") or {}
                        if mtype == "LOGIN_OK":
                            finish_login(True, user_id=payload.get("id"))
                        elif mtype == "LOGIN_FAIL":
                            # 실패해도 기존 로그인은 풀지 않는다 (이미 들어와 있는
                            # 사용자가 오타 한 번에 쫓겨나면 곤란하므로).
                            finish_login(False, payload.get("reason", "로그인 실패"))
        except OSError as e:
            broadcast(f"[server] link down ({e}); retry in 3s")
        finally:
            with _server_lock:
                _server_sock = None
            # 링크가 끊기면 게이트도 다시 잠근다. 서버는 로그인 사용자를
            # 재연결과 무관하게 기억하지만(currentUser_), 그 상태를 물어보는
            # 프로토콜이 없어서 이쪽에서 확인할 방법이 없다 - 안전하게 재로그인을
            # 요구하는 쪽을 기본값으로 삼는다.
            clear_login()
            # 응답을 기다리던 로그인 요청이 있으면 영원히 매달리지 않게 깨운다.
            finish_login(False, "서버 연결이 끊겼습니다")
        time.sleep(3)
