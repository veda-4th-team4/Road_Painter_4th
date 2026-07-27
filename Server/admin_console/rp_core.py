#!/usr/bin/env python3
"""Road-Painter 관리자 창 - 공통 코어.

설정값 + 로그 브로드캐스트/SSE 상태 + 중앙 서버(role=ADMIN) 링크를 담는다.
web_gui.py(HTTP·로봇·로그 UI)와 cctv.py(카메라·캘리브레이션)가 공유하는 기반.

의존성은 단방향이다:  rp_core  <-  cctv  <-  web_gui
이 모듈은 cctv/web_gui를 절대 import하지 않는다 (순환 방지).
"""
import json
import os
import socket
import ssl
import sys
import threading
import time
from collections import deque


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
# maxlen: POS 등 tap이 고주기로 흐르면 오래된 이벤트가 밀려나므로 넉넉히 잡음.
LOG_HISTORY_MAX = 2000
log_history = deque(maxlen=LOG_HISTORY_MAX)


def broadcast(line):
    print(line, flush=True)
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

_server_lock = threading.Lock()
_server_sock = None   # ssl socket while connected, else None
_server_seq = 0


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


def server_send(mtype, payload):
    """Send a {type, seq, payload} message to the relay server (as ADMIN)."""
    global _server_seq
    _server_seq += 1
    ok = _server_send_raw({"type": mtype, "seq": _server_seq, "payload": payload})
    if ok:
        broadcast(f"[server] >> {mtype} {json.dumps(payload, ensure_ascii=False)}")
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
                        broadcast(_fmt_tap(msg.get("payload", {})))
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
        except OSError as e:
            broadcast(f"[server] link down ({e}); retry in 3s")
        finally:
            with _server_lock:
                _server_sock = None
        time.sleep(3)
