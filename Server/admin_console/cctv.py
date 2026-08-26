#!/usr/bin/env python3
"""Road-Painter 관리자 창 - CCTV(카메라·캘리브레이션) 파트.

담당: 카메라 CAM_POSE 수신/파싱, 캘리브레이션(CALIB_K / 호모그래피 / LDC 체크),
스냅샷 수신 서버, 카메라 대시보드 UI(PAGE), 그리고 카메라를 중앙 서버에 통역해
넣어주는 CCTV 브리지(cctv_link_loop, 카메라 직결 전까지의 과도기 구조).

CCTV 팀은 주로 이 파일에서 작업하면 된다. 로그 출력(broadcast)·중앙 서버 전송
(server_send)·포트 설정은 rp_core에서 가져다 쓴다.

의존: rp_core.  web_gui가 이 모듈을 import 한다 (rp_core <- cctv <- web_gui).
"""
import csv
import json
import math
import os
import socket
import ssl
import struct
import threading
import time
import urllib.error
import urllib.request
from collections import deque

from rp_core import (
    broadcast,
    server_send,
    CAM_CHANNELS,
    TCP_PORT,
    SNAPSHOT_PORT,
    SERVER_HOST,
    SERVER_PORT,
    LOG_SUBJECT_JS,
)

WATCHDOG_S = 2.0
# 카메라가 아직 서버(9000)에 role=CCTV로 직접 붙지 못하는 동안, 이 프로세스가
# CAM_POSE를 POS로 통역해 대신 공급하는 다리(cctv_link_loop). 카메라 앱이
# CCTV_CAMERA_SPEC.md대로 직결하게 되면 반드시 꺼야 한다 — 서버는 role당 연결을
# 1개만 유지해서, 카메라와 이 다리가 동시에 role=CCTV로 붙으면 서로 계속 밀어내는
# 재접속 핑퐁이 발생한다 (양쪽 다 3초 간격 자동 재접속 루프라 무한 반복됨).
CCTV_BRIDGE_ENABLED = os.environ.get("RP_CCTV_BRIDGE", "1") not in ("0", "false", "off")
SNAPSHOT_DIR = "."
# Uploaded calibration-view JPEGs (CALIB_K_UPLOAD) go in their own subfolder
# so they don't clutter the main directory with the .ppm/.csv/logs.
CALIB_VIEW_DIR = f"{SNAPSHOT_DIR}/calib_views"
LDC_CHECK_LOG_INTERVAL_S = 1.0  # throttle: at most one CSV row per this many seconds
LDC_CHECK_LOG_PATH = f"{SNAPSHOT_DIR}/ldc_check_log.csv"
# HG_SNAPSHOT floor reference still (latest-wins) + its meta (H, anchors). Served
# to the dashboard's homography canvas overlay via /hg_reference.jpg and /hg_meta.
HG_REFERENCE_PATH = f"{SNAPSHOT_DIR}/homography_reference.jpg"
HG_META_PATH = f"{SNAPSHOT_DIR}/homography_reference.json"
HG_EXPERIMENT_DIR = f"{SNAPSHOT_DIR}/homography_experiments"

# --- WiseAI IVA area push (2026-08-18) --------------------------------------
# Env, not a config file or a source constant: the camera's admin password has
# no business sitting in this repo (see build_install.sh/calib_backup.sh,
# which read the same two names from the operator's shell rather than a
# checked-in file). Read once at import so a typo shows up immediately in
# push_iva_area()'s "not set" branch rather than failing silently mid-session.
CAMERA_USER = os.environ.get("CAMERA_USER", "")
CAMERA_PASS = os.environ.get("CAMERA_PASS", "")
# The AppID WiseAI.html's Servers block documents as the default -- override
# only if this camera's WiseAI instance was installed under a different name.
WISEAI_APP_ID = os.environ.get("WISEAI_APP_ID", "WiseAI")
# 카메라 자체 서명 인증서라 이 프로젝트 전체의 curl -k 관례와 동일하게 검증을 끈다.
_INSECURE_SSL_CTX = ssl.create_default_context()
_INSECURE_SSL_CTX.check_hostname = False
_INSECURE_SSL_CTX.verify_mode = ssl.CERT_NONE

current_conn = None
conn_lock = threading.Lock()
# 연결된 카메라의 LAN IP. handle_client()의 TCP peer 주소에서 캡처한다 --
# push_iva_area()가 PUT할 곳을 알아야 하는데, 이 하네스는 그 외엔 카메라로
# 먼저 접속을 거는 일이 없다(늘 카메라가 이쪽 7100으로 붙는다).
current_cam_ip = None

# One local timestamp per camera-side calibration session, keyed by the id the
# camera stamps into every CALIB_K_VIEW (its epoch_ms at CALIB_K_START).
#
# A view arrives as three separate connections (plain JPEG, overlay JPEG, and
# the corners/K JSON riding along with them), seconds apart. Naming each file
# by its own arrival time would scatter one view across three names, so the
# session id groups them and this map pins one stamp for the whole session.
# The stamp itself is OUR clock, not the camera's: the camera's is unsynced
# (README §7), so its numbers would sort and read wrong on this filesystem.
calib_sessions = {}
calib_sessions_lock = threading.Lock()

# Marker-observation recorder ---------------------------------------------------
# The camera streams raw detections.  This optional recorder stores every
# observed marker so a PC can choose points and attach surveyed coordinates
# later, without forcing an ID set or a fitting method in the live dashboard.
hg_experiment_lock = threading.Lock()
hg_experiment = {"active": False, "candidates": {}, "samples": {},
                 "started": None, "w": None, "h": None, "last_export": None}
hg_experiment_result = None


def _hg_stats(rows):
    n = len(rows)
    if not n:
        return {"n": 0, "mean_u": None, "mean_v": None, "std_u": None, "std_v": None}
    mu = sum(r["u"] for r in rows) / n
    mv = sum(r["v"] for r in rows) / n
    return {"n": n, "mean_u": mu, "mean_v": mv,
            "std_u": (sum((r["u"] - mu) ** 2 for r in rows) / n) ** .5,
            "std_v": (sum((r["v"] - mv) ** 2 for r in rows) / n) ** .5}


def hg_experiment_status():
    with hg_experiment_lock:
        candidates = list(hg_experiment["candidates"].values())
        samples = {str(mid): _hg_stats(hg_experiment["samples"].get(mid, []))
                   for mid in hg_experiment["candidates"]}
        return {"active": hg_experiment["active"], "started": hg_experiment["started"],
                "w": hg_experiment["w"], "h": hg_experiment["h"],
                "candidates": candidates, "samples": samples,
                "last_export": hg_experiment["last_export"],
                "result": hg_experiment_result}


def hg_experiment_observe(msg):
    """Append one raw marker center while an operator-controlled session runs."""
    if msg.get("type") != "CAM_POSE" or not msg.get("confidence"):
        return
    try:
        marker_id = int(msg["id"])
        corners = msg["corners"]
        if len(corners) < 4:
            return
        u = sum(float(p["x"]) for p in corners[:4]) / 4.0
        v = sum(float(p["y"]) for p in corners[:4]) / 4.0
    except (KeyError, TypeError, ValueError):
        return
    with hg_experiment_lock:
        if not hg_experiment["active"]:
            return
        hg_experiment["w"] = msg.get("w", hg_experiment["w"])
        hg_experiment["h"] = msg.get("h", hg_experiment["h"])
        hg_experiment["candidates"].setdefault(marker_id, {"id": marker_id})
        world = msg.get("world") if isinstance(msg.get("world"), dict) else None
        hg_experiment["samples"].setdefault(marker_id, []).append({
            "seq": msg.get("seq"), "t": msg.get("t_frame", msg.get("t")),
            "u": u, "v": v,
            "corners": [[float(p["x"]), float(p["y"])] for p in corners[:4]],
            "world": world,
        })


def hg_experiment_export():
    """Freeze the current session into portable JSON and long-form CSV."""
    with hg_experiment_lock:
        if not hg_experiment["candidates"]:
            raise ValueError("no candidate markers registered")
        os.makedirs(HG_EXPERIMENT_DIR, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        base = f"{HG_EXPERIMENT_DIR}/homography_12pt_{stamp}"
        data = {"schema": "cctv_calibration.marker-observations.v1",
                "created": stamp, "camera": {"w": hg_experiment["w"], "h": hg_experiment["h"]},
                "candidates": list(hg_experiment["candidates"].values()),
                "samples": {str(mid): rows for mid, rows in hg_experiment["samples"].items()},
                "summary": {str(mid): _hg_stats(hg_experiment["samples"].get(mid, []))
                            for mid in hg_experiment["candidates"]}}
        json_path = base + ".json"
        csv_path = base + ".csv"
        with open(json_path, "w") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["id", "seq", "t_frame_ms", "u_px", "v_px",
                             "world_x_mm", "world_y_mm", "world_theta_deg", "corners_px_json"])
            for c in data["candidates"]:
                for row in hg_experiment["samples"].get(c["id"], []):
                    world = row.get("world") or {}
                    writer.writerow([c["id"], row["seq"], row["t"], row["u"], row["v"],
                                     world.get("x", ""), world.get("y", ""), world.get("theta", ""),
                                     json.dumps(row.get("corners", []), separators=(",", ":"))])
        hg_experiment["active"] = False
        hg_experiment["last_export"] = {"json": os.path.basename(json_path),
                                         "csv": os.path.basename(csv_path)}
        return hg_experiment["last_export"]


def calib_session_stamp(session, fallback_ts):
    if session is None:
        return fallback_ts          # pre-session camera build: per-file stamp
    with calib_sessions_lock:
        return calib_sessions.setdefault(session, fallback_ts)


# LDC_CHECK streams continuously (every camera frame, ~5-30Hz) while active.
# Logging every single reading would flood the CSV, so writes are throttled
# to at most one row per LDC_CHECK_LOG_INTERVAL_S — the camera is not asked
# to slow down; the RPi just samples what it receives.
_ldc_log_last_write = 0.0


def ldc_check_log_reset():
    """Called on LDC_CHECK_ACK 'checking' (a fresh session starting)."""
    global _ldc_log_last_write
    _ldc_log_last_write = 0.0


def ldc_check_log_row(msg, undistorted):
    global _ldc_log_last_write
    now = time.time()
    if now - _ldc_log_last_write < LDC_CHECK_LOG_INTERVAL_S:
        return
    _ldc_log_last_write = now

    is_new = False
    try:
        with open(LDC_CHECK_LOG_PATH, "r"):
            pass
    except FileNotFoundError:
        is_new = True
    with open(LDC_CHECK_LOG_PATH, "a") as f:
        if is_new:
            f.write("timestamp,markers,markers_total,corners,corners_total,"
                    "straight_rms_px,straight_max_px,edge_max_px,center_max_px,"
                    "u_straight_rms_px,u_straight_max_px,u_edge_max_px,u_center_max_px\n")
        ts = time.strftime("%Y%m%d_%H%M%S")
        u = undistorted or {}
        f.write(f"{ts},{msg.get('markers')},{msg.get('markers_total')},"
                f"{msg.get('corners')},{msg.get('corners_total')},"
                f"{msg.get('straight_rms_px', '')},{msg.get('straight_max_px', '')},"
                f"{msg.get('edge_max_px', '')},{msg.get('center_max_px', '')},"
                f"{u.get('straight_rms_px', '')},{u.get('straight_max_px', '')},"
                f"{u.get('edge_max_px', '')},{u.get('center_max_px', '')}\n")


def send_command(cmd):
    with conn_lock:
        conn = current_conn
    if conn is None:
        broadcast("[!] no camera connected; command dropped")
        return
    try:
        conn.sendall((cmd + "\n").encode())
        broadcast(f"[>] sent: {cmd}")
    except OSError as e:
        broadcast(f"[!] send failed: {e}")


def push_iva_area(cam_ip, channel, points_px, enable=True, name="ArucoPose_calib"):
    """PUT a pixel polygon to WiseAI's own IVA area API on this camera.

    Not called automatically when an IVA_SYNC reply arrives (print_msg below)
    -- the operator reviews the computed hull in the browser and pushes it
    explicitly (POST /iva_push), the same "compute, then a separate confirmed
    step to commit" shape as HG_SAVE/MARKER_PLANE_SAVE. Auto-pushing on
    receipt would mean a stale or wrong hull (bad anchors, a fit run before
    the last marker move) reaches the camera's live detection area with
    nobody having looked at it.

    Digest, not Basic: WiseAI.html documents digestAuth (RFC 7616) as the
    only scheme /opensdk/{AppID}/... accepts. urllib's own
    HTTPDigestAuthHandler does the challenge/response handshake -- no
    external dependency, matching this project's stdlib-only convention.

    Returns (ok, detail) -- detail is the response body on success, an error
    message on failure. Never raises: the only caller is a request handler,
    and an uncaught exception there is a 500 with no explanation on the
    browser end.
    """
    if not CAMERA_USER or not CAMERA_PASS:
        return False, "CAMERA_USER/CAMERA_PASS not set in this server's environment"
    if not cam_ip:
        return False, "no camera connected (current_cam_ip is empty)"
    if len(points_px) < 3:
        return False, f"need at least 3 points for a polygon, got {len(points_px)}"

    url = f"https://{cam_ip}/opensdk/{WISEAI_APP_ID}/configuration/ivaarea"
    body = json.dumps({
        "channel": channel,
        "enable": enable,
        "definedArea": [{
            "index": 1,
            "name": name,
            "areaCoordinates": [
                {"x": int(round(x)), "y": int(round(y))} for x, y in points_px[:8]
            ],
            "detectionModes": ["Entering", "Exiting"],
            "appearanceDuration": 10,
            "intrusionDuration": 2,
            "loiteringDuration": 10,
            "objectTypeFilter": ["Person"],
            "ruleOverlay": True,
        }],
    }).encode("utf-8")

    pwd_mgr = urllib.request.HTTPPasswordMgrWithDefaultRealm()
    pwd_mgr.add_password(None, url, CAMERA_USER, CAMERA_PASS)
    opener = urllib.request.build_opener(
        urllib.request.HTTPDigestAuthHandler(pwd_mgr),
        urllib.request.HTTPSHandler(context=_INSECURE_SSL_CTX),
    )
    req = urllib.request.Request(url, data=body, method="PUT",
                                 headers={"Content-Type": "application/json"})
    try:
        with opener.open(req, timeout=10) as resp:
            return True, resp.read().decode("utf-8", "replace")
    except urllib.error.HTTPError as e:
        return False, f"HTTP {e.code}: {e.read().decode('utf-8', 'replace')}"
    except OSError as e:
        return False, str(e)


# ---------------------------------------------------------------------------
# CCTV data link — a SECOND connection to the relay server, role=CCTV.
#
# Until the camera app itself speaks TLS+HELLO/POS (see ../docs/CCTV_CAMERA_SPEC.md),
# this bridge translates the camera's CAM_POSE stream into the server's POS
# format and feeds it in under the CCTV role. Data path (CCTV) and admin path
# (ADMIN, above) are kept on separate connections so the roles stay clean.
# Also forwards the server's CMD CALIB_START relay down to the camera.
# ---------------------------------------------------------------------------
_cctv_lock = threading.Lock()
_cctv_sock = None
_cctv_seq = 0


def cctv_send(mtype, payload, quiet=False):
    """Send a message to the relay server on the CCTV-role connection."""
    global _cctv_seq
    with _cctv_lock:
        s = _cctv_sock
    if s is None:
        if not quiet:
            broadcast(f"[cctv-link] not connected; dropped {mtype}")
        return False
    _cctv_seq += 1
    try:
        s.sendall((json.dumps({"type": mtype, "seq": _cctv_seq,
                               "payload": payload}) + "\n").encode())
        return True
    except OSError as e:
        if not quiet:
            broadcast(f"[cctv-link] send failed: {e}")
        return False


def cctv_forward_pos(corners):
    """CAM_POSE corners [{'x','y'}x4] -> server POS {'corners':[[u,v]x4]}.
    Only called for confidence>0 frames (marker-lost heartbeats are not sent,
    per CCTV_CAMERA_SPEC.md). quiet: at 15-30Hz a down link would spam the log."""
    if not isinstance(corners, list) or len(corners) < 4:
        return
    try:
        quad = [[float(c["x"]), float(c["y"])] for c in corners[:4]]
    except (KeyError, TypeError, ValueError):
        return
    cctv_send("POS", {"corners": quad}, quiet=True)


def cctv_link_loop():
    """Maintain the CCTV-role connection to the relay server; reconnect forever.

    Skips entirely when CCTV_BRIDGE_ENABLED is false (camera connects to the
    server directly as role=CCTV instead) - see the flag's comment above."""
    if not CCTV_BRIDGE_ENABLED:
        broadcast("[cctv-link] bridge OFF (RP_CCTV_BRIDGE=0) — 카메라 직결 모드, "
                  "이 다리는 대기만 함")
        return
    global _cctv_sock
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    while True:
        try:
            raw = socket.create_connection((SERVER_HOST, SERVER_PORT), timeout=5)
            s = ctx.wrap_socket(raw, server_hostname=SERVER_HOST)
            s.settimeout(None)  # connect 타임아웃 해제 - 유휴 재접속 churn 방지
                                # (server_link_loop의 동일 주석 참고)
            with _cctv_lock:
                _cctv_sock = s
            s.sendall((json.dumps({"type": "HELLO", "seq": 0,
                                   "payload": {"role": "CCTV"}}) + "\n").encode())
            broadcast(f"[cctv-link] connected as CCTV to {SERVER_HOST}:{SERVER_PORT}"
                      " — CAM_POSE를 POS로 통역해 서버에 공급")
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
                    if mtype == "CMD":
                        # 서버가 CCTV role로 중계한 명령(QT/관리자의 CALIB_START 등)을
                        # 카메라 채널로 그대로 전달 - 버튼이 끝까지 통하게 하는 다리.
                        cmd = (msg.get("payload") or {}).get("cmd", "")
                        if cmd == "CALIB_START":
                            broadcast("[cctv-link] << CMD CALIB_START — 카메라로 전달")
                            send_command("CALIB_START")
                    # ACK 등 나머지는 조용히 무시
        except OSError as e:
            broadcast(f"[cctv-link] link down ({e}); retry in 3s")
        finally:
            with _cctv_lock:
                _cctv_sock = None
        time.sleep(3)


# ---------------------------------------------------------------------------
# Bridge: push completed calibration to the relay server as H_MATRIX.
#
# The camera reports intrinsics (CALIB_K_RESULT: fx/fy/cx/cy/dist) and the
# pixel->world homography (CALIB_RESULT / HG_SET: H, a flat
# 9-list, row-major) at separate times. We cache the latest of each and, once an
# H exists, send the combined {K,D,H_floor} bundle so the server persists it
# (user_store) and relays it to Qt. See ../src/calib.hpp for the bundle format.
# ---------------------------------------------------------------------------
# 🔴 캐시는 **채널별**이다 (프로토콜 v0.4). 채널마다 렌즈 방향이 달라 K/D/H 가
#    전부 다르므로, 캐시를 하나만 두면 CH2 를 캘리하는 순간 CH1 의 K 가 CH2 의 H 와
#    섞인 번들이 만들어진다 — 에러 없이 그럴듯한 값이 나오고 좌표만 틀린다.
_calib_caches = {}          # {ch: {"K":…, "dist":…, "H":…}}
# 기본 CH2 (2026-08-12). 지금 이 설치에서 K/dist 와 호모그래피가 완성돼 있는
# 유일한 렌즈가 CH2 이고, 부팅 후 아무것도 안 고른 상태에서 CH1 이 잡혀 있으면
# 캘리 명령이 캘리 안 된 렌즈로 나간다. 다른 현장으로 옮기면 여기를 바꾼다.
_calib_channel = 2          # 지금 캘리브레이션 중인 채널
_calib_lock = threading.Lock()


def _reshape3x3(flat):
    f = [float(x) for x in flat]
    return [f[0:3], f[3:6], f[6:9]]


def _cache_for(ch):
    """채널별 캐시 슬롯 (없으면 만든다). _calib_lock 을 잡은 상태에서 호출할 것."""
    return _calib_caches.setdefault(ch, {"K": None, "dist": None, "H": None})


def calib_channel():
    with _calib_lock:
        return _calib_channel


def set_calib_channel(ch):
    """캘리브레이션 대상 채널을 바꾸고 서버에도 알린다.

    서버는 SELECT_CHANNEL로 자기 activeChannel(POS 수신 게이트)만 바꾼다 —
    ArucoPosePNM은 4채널을 항상 동시에 스트리밍하므로 "검출 대상 전환"이라는
    개념 자체가 없다(SELECT_CHANNEL을 받아도 그냥 ack만 하고 아무 것도 안
    바꾼다, 2026-08-11 카메라 코드로 확인 — 이 문서는 원래 단일채널 cctv_app을
    염두에 둔 문구였다). 그래도 이 명령은 여전히 필요하다: 서버가 지금 어느
    채널의 POS만 받아들일지(그리고 어느 채널 캘리브레이션으로 좌표를 바꿀지)를
    이걸로 정하기 때문이다 — 안 보내면 로봇 마커가 실제로 보이는 채널과 서버의
    activeChannel이 어긋나서 POS가 조용히 버려진다(docs/PROTOCOL.md "채널 규약").
    """
    global _calib_channel
    try:
        ch = int(ch)
    except (TypeError, ValueError):
        return False, "채널 번호가 잘못되었습니다."
    if not 1 <= ch <= CAM_CHANNELS:
        return False, f"채널은 1~{CAM_CHANNELS} 범위여야 합니다."
    with _calib_lock:
        _calib_channel = ch
    server_send("CMD", {"cmd": "SELECT_CHANNEL", "ch": ch})
    broadcast(f"[calib] 캘리브레이션 대상 채널 CH{ch} (SELECT_CHANNEL 전송)")
    return True, None


def calib_channel_status():
    """UI 용 요약 — 채널별로 K/H 가 캐시돼 있는지."""
    with _calib_lock:
        cur = _calib_channel
        chans = [{"ch": ch,
                  "has_k": _calib_caches.get(ch, {}).get("K") is not None,
                  "has_h": _calib_caches.get(ch, {}).get("H") is not None}
                 for ch in range(1, CAM_CHANNELS + 1)]
    return {"channel": cur, "count": CAM_CHANNELS, "channels": chans}


# 🔴 아래 두 함수는 **기록만** 한다. 서버로 보내지 않는다 (2026-08-14).
#
# 원래 이 자리에는 push_calib_to_server() 라는 브리지가 있었다. 카메라는 바닥 H
# 를 자기 안에만 갖고 있었고, 서버가 그 값을 알게 되는 유일한 길이 "대시보드가
# HG_QUERY/CALIB_K_QUERY 로 값을 받아 캐시했다가 자기가 조립한 번들을 올린다"
# 였기 때문이다. 그 왕복이 만든 문제가 셋이었다:
#
#   1. 대시보드가 조립하는 번들은 대시보드가 아는 것만 담을 수 있었다.
#      image_size 는 드롭다운 추정값, coord_mode 는 HG_COORD_MODE 명령 이력에서
#      유추(새로고침하면 소실), K/D 는 놓치면 자리표시자. 셋 다 카메라만 아는
#      사실인데 대시보드가 추측했다. 2026-08-11 CH2 사고(raw 로 피팅된 H 가
#      undistort 로 라벨링돼 몇 시간 나가 있던 것)가 그 결과다.
#   2. 관리자 창을 안 열어 두고 캘리하면 결과가 카메라 안에만 남았다.
#   3. `HG` 는 "새 캘리"와 "단순 조회의 답"이 같은 타입으로 온다. 구분할 방법이
#      없어 값이 변했는지로 추측해야 했고, 그 추측이 틀리면 화면을 새로 여는
#      것만으로 서버의 캘리 슬롯이 덮였다 — 실기 시험에서 오도메트리 결과가
#      사라진 원인이다(Qt 로그: `1200×900mm·undistort` → 새로고침 후
#      `H만·1200×1220mm·검산 불가`).
#
# 이제 카메라가 피팅이 끝나는 그 자리에서 직접 보낸다(SendFloorBundle, 주행
# 캘리의 SendCalibBundle 과 같은 모양). 새 캘리가 났다는 것을 아는 곳은 카메라
# 뿐이므로 추측이 필요 없고, 카메라가 조립하므로 추측할 필드도 없다. 재전송이
# 필요하면 FLOOR_RESEND / ODOM_RESEND 로 카메라에게 다시 시킨다.
#
# 캐시 자체는 남긴다 — 채널 상태 UI(calib_channel_status 의 has_k/has_h)가
# "이 렌즈는 캘리가 돼 있나"를 이걸로 표시한다. 읽기 전용 거울이지 권위가 아니다.

def calib_cache_k(fx, fy, cx, cy, dist, ch=None):
    """K/dist 를 채널 캐시에 기록한다 (UI 표시용). ch: 0-based, 카메라 메시지 그대로.

    Was: always cached under _calib_channel (the UI's selector), trusting
    that whoever is looking at the K/dist result also has the right channel
    picked. If a stale reply for the PREVIOUS channel lands after the
    operator has already switched -- entirely possible, nothing here waits
    for a reply before allowing another channel switch -- it silently
    overwrote the wrong slot: no error, a plausible-looking K, wrong
    coordinates (2026-08-10 fix; see the file-level warning above
    _calib_caches). ch=None keeps the old (unsafe) fallback for any caller
    that genuinely has no channel to give.
    """
    if fx is None or fy is None:
        return None
    with _calib_lock:
        target = (ch + 1) if ch is not None else _calib_channel  # camera is 0-based, cache is 1-based
        c = _cache_for(target)
        c["K"] = [[float(fx), 0.0, float(cx or 0)],
                  [0.0, float(fy), float(cy or 0)],
                  [0.0, 0.0, 1.0]]
        c["dist"] = [float(x) for x in (dist or [])]
    return target


def calib_cache_h(h, ch=None):
    """바닥 H 를 채널 캐시에 기록한다 (UI 표시용). calib_cache_k 와 같은 규약."""
    if not isinstance(h, list) or len(h) != 9:
        return
    with _calib_lock:
        target = (ch + 1) if ch is not None else _calib_channel
        c = _cache_for(target)
        c["H"] = _reshape3x3(h)


# Above this, "net" is not a delay -- it is the camera and this server
# disagreeing about what time it is. A frame at 4-10fps plus LAN transit cannot
# plausibly take seconds; README §7 records skews of 5-13s with NTP unset.
CLOCK_SANE_MAX_MS = 2000.0
_clock_warned = False

# --- NTP-less clock-offset compensation ------------------------------------
# raw net = (server recv time) - (camera send time) is dominated by the
# CONSTANT clock skew between the two boxes (README §7 recorded 5-13s), not by
# delay. Estimate that skew as the running MINIMUM of raw net over a sliding
# window: on a LAN the smallest observed value ~= the true one-way floor, so
# net = raw - offset leaves the network delay ABOVE that floor. This makes net
# a meaningful RELATIVE latency with the camera clock unsynced -- no camera
# NTP config needed. The window follows slow relative drift. It is NOT an
# absolute one-way delay (that needs real NTP on the camera).
_CLOCK_WINDOW_MS = 120000.0     # 2 min sliding window (tracks slow drift)
_CLOCK_MIN_SAMPLES = 15         # warm-up before net is trusted
_net_samples = deque()          # (recv_ms, raw_net_ms)
_clock_lock = threading.Lock()
_clock_offset_note = False


def _update_clock_offset(recv_ms, raw_net):
    """Append a raw-net sample; return (offset_estimate_ms, sample_count)."""
    with _clock_lock:
        _net_samples.append((recv_ms, raw_net))
        cutoff = recv_ms - _CLOCK_WINDOW_MS
        while _net_samples and _net_samples[0][0] < cutoff:
            _net_samples.popleft()
        offset = min(v for _, v in _net_samples)
        return offset, len(_net_samples)


def _reset_clock_offset():
    with _clock_lock:
        _net_samples.clear()


def latency_parts(msg, now_ms):
    """Split the delay into the part we can trust and the part we cannot.

    proc = t - t_frame.  Both stamps come from the camera's OWN clock
    (aruco_detector_cv.cpp epoch_ms(), taken at frame arrival and again just
    before send), so their difference is exact no matter how wrong that clock
    is against ours. This is detection + JSON build, and at a few fps it is
    also the bulk of the total.

    net = now - t.  This one crosses clocks -- our time.time() against the
    camera's -- so it only means anything once the two are synced. NTP is still
    unconfigured, so it currently reports the offset, not a delay; report it as
    unknown rather than print a number that looks like latency and isn't.
    """
    global _clock_offset_note
    t, t_frame = msg.get("t"), msg.get("t_frame")

    proc = (t - t_frame) if (t is not None and t_frame is not None) else None
    if t is None:
        return proc, None, False

    # (now - t) crosses clocks, so it is mostly the constant skew. Subtract the
    # running-minimum estimate of that skew to recover the network delay above
    # the LAN floor (see _update_clock_offset).
    raw_net = now_ms - t
    offset, have = _update_clock_offset(now_ms, raw_net)
    net = raw_net - offset
    if net < 0.0:
        net = 0.0
    # Trust the corrected value once the min-filter has plausibly seen a
    # near-floor sample.
    ok = have >= _CLOCK_MIN_SAMPLES and net <= CLOCK_SANE_MAX_MS

    if ok and not _clock_offset_note:
        _clock_offset_note = True
        broadcast(f"[i] 시계 오프셋 보정 활성 (추정 offset ~{offset/1000:+.1f}s). "
                  f"net/total은 카메라 NTP 없이 서버측에서 추정한 "
                  f"기준선 대비 상대 네트워크 지연이다 (README §7).")
    return proc, net, ok


def latency_text(msg, now_ms):
    proc, net, ok = latency_parts(msg, now_ms)
    out = []
    if proc is not None:
        out.append(f"proc={proc:.0f}ms")
    if ok:
        out.append(f"net={net:.0f}ms")
        if proc is not None:
            out.append(f"total={proc + net:.0f}ms")
    else:
        out.append("net=?clock")
    # t_det: detectMarkers()-only cost from the camera. Appended last so the
    # existing proc=/net=/total= parsers are untouched. "det" is the marker
    # search, "rest" is proc minus it -- the split that says whether the search
    # is the bottleneck worth optimizing.
    det = msg.get("t_det")
    if det is not None and det >= 0:
        out.append(f"det={det:.0f}ms")
        if proc is not None:
            out.append(f"rest={proc - det:.0f}ms")
    return " ".join(out)


def print_msg(msg, last_seq):
    now_ms = time.time() * 1000
    mtype = msg.get("type")

    if mtype == "SHELL":
        stream = msg.get("stream")
        if stream == "start":
            broadcast(f"[shell] $ {msg.get('cmd')}")
        elif stream == "out":
            # Emitted verbatim; the camera already stripped the newline and
            # escaped the line, so this is one terminal row.
            broadcast(f"[shell] {msg.get('line')}")
        elif stream == "end":
            note = " (output truncated)" if msg.get("truncated") else ""
            broadcast(f"[shell] --- exit={msg.get('exit')} "
                      f"({msg.get('lines')} lines){note}")
        return last_seq

    # 아래 CALIB_ACK/HG/ANCHORS/HG_FIT*는 2026-08-10에 다시 맞췄다. 이전
    # 버전은 CALIB_RESULT/CALIB_HG_QUERY/CALIB_ANCHORS/CALIB_VALIDATION/
    # HG_SET(ok 필드)라는, cctv_app 시절 이름 그대로의 타입을 기다렸는데
    # ArucoPosePNM은 그런 타입을 애초에 보낸 적이 없다(HG_SET 명령의 성공도
    # "HG"로, ANCHOR_SET_ALL의 성공도 "ANCHORS"로 온다) — 그래서 이 캘리브레이션
    # 피드백 전체가 카메라를 직결한 뒤로 한 번도 화면에 뜬 적이 없었다. 카메라
    # 쪽(이미 확립된, Qt/서버도 같이 보는 프로토콜)을 바꾸는 대신 여기를 맞췄다.
    if mtype == "CALIB_ACK":
        # "[calib] camera acknowledged" 그대로 유지 -- handleHgStatus()가 이
        # 정확한 부분문자열로 "진행중" 배너를 켠다. ch는 끝에 덧붙인다.
        broadcast(f"[calib] camera acknowledged, collecting anchors... (ch={msg.get('ch')})")
        return last_seq
    if mtype == "HG":
        # 수동 HG_QUERY 응답, HG_SET/HG_COORD_MODE 성공, 그리고 CALIB_START
        # 수집이 끝났을 때(성공이든 프레임 예산 소진으로 실패든) — 전부 이
        # 하나의 타입으로 온다. ch 없이는 어느 렌즈인지 몰라 전역 변수에
        # 잘못 캐싱될 뻔한 게 바로 위 calib_cache_h()의 버그였다.
        #
        # 이 셋을 구분할 필요는 이제 없다 — 새 캘리가 끝나면 카메라가 서버로
        # 직접 올리므로(SendFloorBundle), 여기서는 화면 갱신만 하면 된다.
        ch = msg.get("ch")
        if msg.get("available"):
            H = msg.get("H")
            # undistorted 를 같이 실어 보낸다(2026-08-12). 이건 "지금 좌표계 설정"이
            # 아니라 **이 H 가 실제로 어느 픽셀 공간에서 피팅됐는지**로, 카메라의
            # FittedUndistorted(ch) 값이다. 번들의 coord_mode 는 원래 브라우저가
            # HG_COORD_MODE 를 보낸 이력(hgCoordModeActive)에서만 가져왔는데, 그건
            # 설정이지 사실이 아니다 — 새로고침만 해도 null 이 되어 coord_mode 가
            # "unknown" 으로 채워졌다. 카메라가 아는 사실을 그대로 흘려보낸다.
            broadcast(f"[calib] ch={ch} HOMOGRAPHY H={H} "
                      f"mappable={msg.get('mappable')} camera_z_mm={msg.get('camera_z_mm')} "
                      f"undistorted={msg.get('undistorted')}")
            calib_cache_h(H, ch)  # 채널 상태 UI 용 기록만 (전송은 카메라가 한다)
        else:
            broadcast(f"[calib] ch={ch} 호모그래피 아직 계산 안 됨")
        return last_seq
    if mtype == "ANCHORS":
        # ANCHOR_QUERY 응답이자 ANCHOR_SET_ALL 의 ack 둘 다 — 카메라가 실제로
        # 받아들인 표를 그대로 되돌려준다. 필드명은 "markers"다(예전 코드가
        # 기다리던 "anchors"가 아니라).
        broadcast(f"[calib] ch={msg.get('ch')} ANCHORS " +
                  json.dumps(msg.get("markers", []), separators=(",", ":")))
        return last_seq
    if mtype == "ODOM_PREFER":
        # 로봇 측위가 어느 H_marker 를 쓰는지. 좌표가 조용히 달라지는 설정이라
        # 로그에도 남기고([calib]), 진행도 UI 도 갱신하도록 [odo] 로 낸다.
        broadcast("[odo] " + json.dumps(
            {"dir": "IN", "peer": "CAM", "type": "ODOM_PREFER", "payload": msg},
            ensure_ascii=False))
        which = "측정(주행)" if msg.get("preferred") else "파생(체커보드)"
        note = ""
        if not msg.get("ok"):
            note = f" — 거부: {msg.get('reason', '')}"
        elif msg.get("k_stale") or msg.get("height_stale"):
            note = " — ⚠ 측정 당시와 " + ("K 가 " if msg.get("k_stale") else "") \
                 + ("마커 높이가 " if msg.get("height_stale") else "") + "달라졌습니다"
        broadcast(f"[calib] ch={msg.get('ch')} 로봇 측위 H_marker = {which}{note}")
        return last_seq
    if mtype == "ODOM_RESEND":
        # 주행 캘리 번들 수동 재전송의 ack (2026-08-13). 조립은 카메라가 하므로
        # 여기서는 성공 여부와 사유만 흘려보낸다 — 번들 내용은 서버가 받은 것이
        # 정본이고, 그건 TAP 의 H_MATRIX 줄로 따로 보인다.
        broadcast("[odo] " + json.dumps(
            {"dir": "IN", "peer": "CAM", "type": "ODOM_RESEND", "payload": msg},
            ensure_ascii=False))
        if msg.get("ok"):
            broadcast(f"[calib] ch={msg.get('ch')} 주행 캘리 번들 재전송됨 (H_MATRIX)")
        else:
            broadcast(f"[calib] ch={msg.get('ch')} 주행 캘리 번들 재전송 거부"
                      f" — {msg.get('reason', '')}")
        return last_seq
    if mtype == "FLOOR_RESEND":
        # 앵커 캘리 번들 수동 재전송의 ack (2026-08-14). ODOM_RESEND 와 완전히
        # 같은 모양이다 — 조립은 카메라가 하고 여기서는 성공/사유만 흘려보낸다.
        #
        # [odo] 접두사를 쓰는 건 이게 주행 캘리라서가 아니라, **카메라의 구조화된
        # 응답이 브라우저로 가는 통로**가 그것 하나뿐이기 때문이다(handleOdo 의
        # ev.type 스위치). ODOM_PREFER 도 같은 이유로 여기 실린다.
        broadcast("[odo] " + json.dumps(
            {"dir": "IN", "peer": "CAM", "type": "FLOOR_RESEND", "payload": msg},
            ensure_ascii=False))
        if msg.get("ok"):
            broadcast(f"[calib] ch={msg.get('ch')} 앵커 캘리 번들 재전송됨 (H_MATRIX)")
        else:
            broadcast(f"[calib] ch={msg.get('ch')} 앵커 캘리 번들 재전송 거부"
                      f" — {msg.get('reason', '')}")
        return last_seq
    if mtype == "ODOM_DONE":
        # 주행 캘리의 종결 보고. 서버로 가는 H_MATRIX/CALIB_FAIL 과 달리 이건
        # 카메라가 이 대시보드 링크(7100)로 직접 올리는 것이고, mm 단위 품질
        # 지표(LOO 잔차·폐합오차·체커보드 대비 scale)는 여기에만 있다 —
        # 서버는 픽셀 단위 폐합오차만 로깅한다(wire 규격 §5).
        #
        # 진행도 표가 파싱하도록 TAP 쪽과 같은 '[odo] {json}' 형식으로 낸다.
        broadcast("[odo] " + json.dumps(
            {"dir": "IN", "peer": "CAM", "type": "ODOM_DONE", "payload": msg},
            ensure_ascii=False))
        ok = msg.get("ok")
        broadcast(f"[calib] ch={msg.get('ch')} 주행캘리 {'성공' if ok else '실패'}"
                  f" n={msg.get('n')} rmse_loo={msg.get('rmse_loo_mm')}mm"
                  f" 폐합={msg.get('closure_mm')}mm"
                  + (f" 사유={msg.get('reason')}" if not ok else ""))
        return last_seq
    if mtype == "HG_FIT":
        # LOO(Leave-One-Out) 잔차 요약 — cctv_app의 "검증 마커를 피팅에서 빼서
        # 따로 확인" 방식 대신 전부 피팅에 쓰면서 정직한 표본외 오차를 얻는
        # 방식이다(homography_mapper.h 상단 설계 코멘트 참고). VALIDATION_*
        # 자리를 대신한다 — 그 명령 자체가 카메라에 없다(2026-08-10 확인).
        ch = msg.get("ch")
        if msg.get("have"):
            broadcast(f"[calib] ch={ch} FIT n={msg.get('n')} "
                      f"rmse_in={msg.get('rmse_in_mm')}mm rmse_loo={msg.get('rmse_loo_mm')}mm "
                      f"max_loo={msg.get('max_loo_mm')}mm(id={msg.get('max_loo_id')}) "
                      f"loo_valid={msg.get('loo_valid')} advisory={msg.get('advisory')}")
        else:
            broadcast(f"[calib] ch={ch} FIT 없음 — 아직 계산된 호모그래피가 없습니다")
        return last_seq
    if mtype == "HG_FIT_PT":
        # HG_FIT 하나당 최대 24줄(포인트당 하나) — POSE_SENDER_MAX_LINE(1024B)
        # 때문에 한 메시지에 다 못 실어서 카메라가 포인트마다 따로 보낸다.
        broadcast(f"[calib] ch={msg.get('ch')} FIT_PT id={msg.get('id')} "
                  f"in={msg.get('in_mm')}mm loo={msg.get('loo_mm')}mm")
        return last_seq
    if mtype == "HG_DONE":
        # CALIB_START 세션 종료 신호(성공/실패 공통, 2026-08-11 추가). 대시보드
        # handleHgStatus()는 "진행중" 배너를 "[calib] SUCCESS"/"[calib] FAILED"
        # 로만 해제하는데, 카메라가 여태 그 종료 신호를 안 보내서 배너가 영원히
        # 안 풀렸다. 카메라 ReportHgDone(HG_DONE)을 여기서 그 문자열로 변환한다.
        # 카메라의 ch 는 0-based 인데 조작자가 보는 드롭다운·라벨은 1-based 다.
        # 예전엔 그 0-based 를 그대로 "ch=1" 로 찍어서, CH2 를 캘리한 사람에게
        # "SUCCESS ch=1" 이 떴다 — 같은 화면에서 CH1 을 가리키는 말로 읽힌다.
        # 사람이 읽는 자리에는 CH{ch+1}, 기계가 거르는 자리에는 끝에 (ch=N) 를
        # 붙인다 — "[calib] camera acknowledged … (ch=N)" 과 같은 규약. (2026-08-11)
        ch = msg.get("ch")
        try:
            label = f"CH{int(ch) + 1}"
        except (TypeError, ValueError):
            label = "CH?"
        if msg.get("ok"):
            broadcast(f"[calib] SUCCESS {label} 앵커 H 계산 완료 "
                      f"(good={msg.get('good')}/{msg.get('total')}) (ch={ch})")
        else:
            broadcast(f"[calib] FAILED {label} "
                      f"{msg.get('result') or '앵커가 다 보이지 않아 H를 못 구했습니다'} "
                      f"(good={msg.get('good')}/{msg.get('total')}) (ch={ch})")
        return last_seq
    if mtype == "HG_SAVE":
        if msg.get("ok"):
            broadcast("[calib] 저장 완료 — 카메라 /mnt(PERSIST_DIR)에 H 기록됨, 재부팅해도 유지")
        else:
            broadcast(f"[calib] H 저장 실패: {msg.get('reason')}")
        return last_seq
    if mtype == "HG_COORD_MODE":
        if msg.get("ok"):
            broadcast(f"[hg-coord] SUCCESS mode={msg.get('mode')}")
        else:
            broadcast(f"[hg-coord] FAILED: {msg.get('reason')}")
        return last_seq
    # HG_CHARUCO_* (ChArUco 보드로 H 계산)는 관리자 창에서 제거했다 — 앵커 기반
    # CALIB_START와 목적이 겹치는 두 번째 H 경로였고, 어느 쪽으로 구한 H인지가
    # 화면에 드러나지 않아 앵커로 맞춘 H를 조용히 덮어쓸 수 있었다.
    # 카메라의 HG_CHARUCO_START 명령 자체는 살아 있으므로, 진단이 필요하면
    # 셸/직접 명령으로 쓸 수 있다. 여기서 받지 않으면 로그에만 남는다.
    if mtype == "CALIB_K_ACK":
        broadcast(f"[calib-K] session started target={msg.get('target')} "
                  f"board={msg.get('squares_x')}x{msg.get('squares_y')} "
                  f"square={msg.get('square_mm')}mm marker={msg.get('marker_mm')}mm "
                  f"dict={msg.get('dictionary')} — rejected frames do not advance")
        return last_seq
    if mtype == "CALIB_K_PROBE":
        # held ChArUco 코너 뷰파인더 — CALIB_PROBE_MS(약 500ms)마다 온다(카메라가
        # 실제로 새로 훑었을 때만, 2026-08-11 신설). ANCHORS와 같은 방식으로 배열을
        # JSON 그대로 실어 보낸다 — 프레임마다 오는 값이라 문장으로 만들면 로그만
        # 시끄러워진다.
        ch = msg.get("ch")
        probe = msg.get("probe", [])
        total = msg.get("corners_total")
        broadcast(f"[calib] ch={ch} PROBE total={total} " +
                  json.dumps(probe, separators=(",", ":")))
        return last_seq
    if mtype == "CALIB_K_PROGRESS":
        # ch= 필수(2026-08-11). CALIB_K_CAPTURE 한 번이 열린 세션을 "전부" 무장하므로
        # (카메라 IntrinsicsCalib::RequestCapture) 한 번 누르면 렌즈 수만큼 이 리포트가
        # 온다. 채널을 안 찍던 시절, 보드가 안 보이는 다른 렌즈의 "코너 1/24 거부"를
        # 캘리 중인 렌즈의 결과로 읽고 한참 헤맸다 — 그 렌즈는 24/24 정상이었다.
        # ch= 는 [calib-K] ch=N CURRENT VALUES 와 같은 자리·같은 표기.
        if msg.get("rejected"):
            broadcast(f"[calib-K] ch={msg.get('ch')} capture REJECTED — {msg.get('reason')} "
                      f"corners={msg.get('corners')}/{msg.get('corners_total')} "
                      f"coverage={100*msg.get('coverage',0):.1f}% "
                      f"sharpness={msg.get('sharpness',0):.1f} "
                      f"move={msg.get('move_px',-1):.1f}px views={msg.get('views')}")
        else:
            straight = ""
            if msg.get("straight_rms_px") is not None:
                # Raw lens distortion measured on this very capture frame —
                # a free by-product of the session (not calibration quality).
                straight = (f" | 왜곡 rms={msg.get('straight_rms_px'):.2f}px "
                            f"edge_max={msg.get('edge_max_px', 0):.2f}px "
                            f"center_max={msg.get('center_max_px', 0):.2f}px")
            broadcast(f"[calib-K] ch={msg.get('ch')} captured view {msg.get('views')}/{msg.get('target')} "
                      f"({msg.get('corners')}/{msg.get('corners_total')} corners, "
                      f"coverage={100*msg.get('coverage',0):.1f}%, "
                      f"sharpness={msg.get('sharpness',0):.1f}, "
                      f"move={msg.get('move_px',-1):.1f}px)"
                      + straight
                      + (" — READY TO COMPUTE" if msg.get("ready") else ""))
        return last_seq
    if mtype == "CALIB_K_PARAMS":
        broadcast(f"[calib-K] params updated: target={msg.get('target')} views, "
                  f"rms_limit={msg.get('rms_limit')}px "
                  f"(accepted views={msg.get('views')})")
        return last_seq
    if mtype == "CALIB_K_COMPUTING":
        broadcast(f"[calib-K] COMPUTING with {msg.get('views')} accepted views — please wait")
        return last_seq
    if mtype == "CALIB_K_UPLOAD":
        stage = msg.get("stage")
        if stage == "start":
            ktxt = ("K/dist 포함" if msg.get("k_available")
                    else "K/dist 없음 (아직 계산·로드 안 됨)")
            broadcast(f"[calib-K] 업로드 시작 — 보관 뷰 {msg.get('total')}장 "
                      f"(뷰당 원본+오버레이 JPEG · 코너좌표 · {ktxt})")
        elif stage == "progress":
            broadcast(f"[calib-K] 업로드 {msg.get('sent')}/{msg.get('total')} "
                      f"— view {msg.get('view')} (이미지 {msg.get('images')}장, "
                      f"{msg.get('bytes')} bytes)")
        elif stage == "error":
            broadcast(f"[calib-K] 업로드 실패 — view {msg.get('view')} "
                      f"(이미지 {msg.get('images')}/2장만 전송됨)")
        elif stage == "busy":
            broadcast("[calib-K] 업로드가 이미 진행 중입니다")
        elif stage == "done":
            broadcast(f"[calib-K] 업로드 완료 — {msg.get('sent')}/{msg.get('total')}장 저장됨")
        return last_seq
    if mtype == "CALIB_K_RESULT":
        if msg.get("ok"):
            broadcast(f"[calib-K] SUCCESS rms={msg.get('rms')}px "
                      f"views={msg.get('views')} pruned={msg.get('pruned')} "
                      f"fx={msg.get('fx')} fy={msg.get('fy')} "
                      f"cx={msg.get('cx')} cy={msg.get('cy')} dist={msg.get('dist')}")
            # 채널 상태 UI 용 기록. 서버로는 안 보낸다 — 새 K/D 를 실은 번들은
            # 카메라가 다음 캘리 완료 때(또는 FLOOR_RESEND/ODOM_RESEND 로) 직접
            # 올린다. 여기서 보내면 대시보드가 조립한 번들이 카메라가 올린 것을
            # 덮는다.
            calib_cache_k(msg.get("fx"), msg.get("fy"), msg.get("cx"),
                          msg.get("cy"), msg.get("dist"), msg.get("ch"))
        else:
            broadcast(f"[calib-K] FAILED: {msg.get('reason')}")
        return last_seq
    if mtype in ("CALIB_K_CONFIG", "CALIB_K_STATUS", "CALIB_K_UNDO"):
        if not msg.get("ok"):
            broadcast(f"[calib-K] {mtype} FAILED: {msg.get('reason')}")
        else:
            broadcast(f"[calib-K] BOARD_CONFIG views={msg.get('views')}/{msg.get('target')} "
                      f"squares={msg.get('squares_x')}x{msg.get('squares_y')} "
                      f"square={msg.get('square_mm')} marker={msg.get('marker_mm')} "
                      f"dict={msg.get('dictionary')} margin={msg.get('margin_x_mm')}/"
                      f"{msg.get('margin_y_mm')} quiet={msg.get('quiet_mm')} "
                      f"board={msg.get('board_w_mm')}x{msg.get('board_h_mm')} "
                      f"gates={msg.get('gates')}")
        return last_seq
    if mtype == "CALIB_K_GATE":
        en = msg.get("enabled")
        broadcast(f"[calib-K] 품질 게이트 {'ON' if en else 'OFF'} gates={en}")
        return last_seq
    if mtype == "ROI_SET":
        # ArucoPosePNM never actually sends this type (checked 2026-08-10) --
        # kept for whichever camera generation does. 1920x1080 was that
        # generation's fixed raw size; not ArucoPosePNM's (2592x1520), so this
        # constant is meaningless for the current camera either way.
        w, h = msg.get("w") or 0, msg.get("h") or 0
        if w and h:
            pct = w * h / (2592 * 1520) * 100
            broadcast(f"[roi] 검출 영역 ({msg.get('x')},{msg.get('y')}) {w}x{h} "
                      f"— 전체의 {pct:.0f}% (마커가 이 영역을 벗어나면 검출 안 됨)")
        else:
            broadcast("[roi] 전체 화면으로 복원")
        return last_seq
    if mtype == "ARUCO_SCAN":
        # cctv_app은 렌즈가 하나라 ch가 없었다. ArucoPosePNM은 4렌즈라 카메라가
        # ch를 붙여 보내므로(2026-08-10 포팅) 그대로 실어 보낸다 — 안 그러면
        # 어느 렌즈가 바뀐 건지 로그만 보고는 알 수 없다.
        ch = msg.get("ch")
        n = msg.get("passes")
        wins = {1: f"{msg.get('win')}", 2: "7, 17", 3: "3, 13, 23"}.get(n, "?")
        broadcast(f"[aruco] ch={ch} 이진화 스캔 {n}회 (창 {wins}) — "
                  f"det/검출률/좌표지터를 비교해 볼 것")
        return last_seq
    if mtype == "DETECT_PARAM":
        # ARUCO_SCAN과 같은 이유로 ch를 붙인다 (2026-08-10, ArucoPosePNM 포팅).
        ch = msg.get("ch")
        if msg.get("ok"):
            nm = msg.get("name") or ""
            head = f"[detect] ch={ch} {nm} 적용 — " if nm else f"[detect] ch={ch} 현재값 — "
            broadcast(head +
                      f"perim {msg.get('perim')} / ecc {msg.get('ecc')} / "
                      f"thresh {msg.get('thresh')} / poly {msg.get('poly')} "
                      f"(검출률↔오검출 트레이드오프, 재부팅 시 기본값 복귀)")
        else:
            broadcast(f"[detect] ch={ch} 거부됨: {msg.get('reason') or '알 수 없는 파라미터'}")
        return last_seq
    if mtype == "RAW_FPS_TEST":
        if msg.get("enabled"):
            broadcast("[raw-fps] 측정 모드 ON — 검출을 건너뛰고 프레임 도착만 셉니다. "
                      "이제 seq 증가 속도가 곧 SDK 전달 fps입니다. "
                      "(마커 검출 안 됨 — 측정 후 반드시 끌 것)")
        else:
            broadcast("[raw-fps] 측정 모드 OFF — 정상 검출로 복귀")
        return last_seq
    # ArucoPosePNM은 "DETECT_ENABLE"이 아니라 "DETECT"라는 type으로, 채널마다
    # 하나씩(SetDetectEnabled/ReportDetect, ch 0-based) 보낸다. refused 필드도
    # 없다 — DETECT 명령은 무조건 적용되지, 세션 중이라고 거부하지 않는다.
    # (예전 cctv_app 프로토콜을 그대로 남겨뒀던 부분 — 2026-08-10 정정)
    if mtype == "DETECT":
        ch = msg.get("ch")
        on = 1 if msg.get("enabled") else 0
        if ch is None:
            return last_seq
        broadcast(f"[detect] ch{ch} detect_enabled={on}"
                  + ("" if on else " (하트비트만 전송, 카메라 부하 감소)"))
        return last_seq
    if mtype == "CENTRAL_STATUS":
        # Same contract as CALIB_ANCHORS: the camera answers CENTRAL_QUERY and
        # every CENTRAL_ID / CENTRAL_HMATRIX with its own state, so the tab
        # never has to trust what it just typed.
        broadcast("[central] STATUS " + json.dumps({
            "link": msg.get("link"),
            "link_on": msg.get("link_on"),
            "pos_on": msg.get("pos_on"),
            "marker_id": msg.get("marker_id"),
            "server": msg.get("server"),
            "action": msg.get("action"),
            "detail": msg.get("detail"),
        }, separators=(",", ":")))
        return last_seq

    if mtype in ("MARKER_PLANE", "MARKER_PLANE_SAVE"):
        # Forwarded as JSON rather than a formatted sentence: the browser needs
        # the individual numbers (camera height, nadir, ratio) to render them,
        # and re-parsing prose on the client is how those get silently dropped.
        broadcast("[calib] MARKER_PLANE " + json.dumps(msg, ensure_ascii=False))
        return last_seq
    if mtype == "IVA_SYNC":
        # IVA_SYNC <ch>의 응답: 등록된 앵커의 raw 픽셀 convex hull. MARKER_PLANE과
        # 같은 이유로 그대로 JSON을 넘긴다 -- 브라우저가 ch/points/ok/reason을
        # 그대로 써야 하고(hull 미리보기 + /iva_push에 되먹임), 문장으로 포맷해
        # 다시 파싱시키면 그 값들이 조용히 빠지기 쉽다.
        broadcast("[calib] IVA_SYNC " + json.dumps(msg, ensure_ascii=False))
        return last_seq
    if mtype == "IVA_EVENT":
        # 사람/차량이 WiseAI IVA 영역 규칙을 넘었다는 알림 (카메라 쪽 파서 +
        # 와이어 형식은 2026-08-19 실측으로 확인 -- 카메라 앱의
        # ParseWiseAiIvaAreaEvents() 참고). IVA_SYNC와 같은 이유로 그대로
        # JSON을 넘긴다 -- 브라우저가 ch/rule/object_id/action/state를
        # 각각 열로 로그해야 하고, 문장으로 포맷해 다시 파싱시키면 그 값들이
        # 조용히 빠지기 쉽다.
        #
        # 아직 소리/릴레이 알람은 안 붙어있다 -- 그 방식(스피커 vs 릴레이)이
        # 아직 안 정해져서, 지금은 파이프라인이 눈에 보이는 것까지만 한다.
        # 도착 시각을 파이 시계로 찍어 같이 넘긴다 -- 카메라의 t_ms와 빼면
        # 링크(재시도 큐 포함)가 실제로 얼마나 밀렸는지가 나온다. 브라우저에서
        # 재면 뷰어 PC 시계가 섞여 들어가 측정이 안 된다.
        msg["rx_ms"] = int(now_ms)
        broadcast("[iva] EVENT " + json.dumps(msg, ensure_ascii=False))
        return last_seq
    if mtype == "ZONE_EVENT":
        # IVA_EVENT와 같은 사건에 대한 카메라 앱 자신의 판정이다. 둘 다 흘리는 게
        # 중복이 아닌 이유: WiseAI는 bbox '중심'으로 안/밖을 가르는데 중심은 바닥
        # 평면 위에 있지 않고, IVA 영역은 바닥 앵커로 만든 것이다. 2026-08-19 실측
        # 9건 -- Exit 4건이 전부 사람 발은 아직 영역 안일 때 발생했고, 중심과 발끝의
        # 픽셀 차는 원근에 따라 162~402px로 변해 상수 보정이 불가능했다. 그래서
        # 카메라가 발끝(bbox 아래변 중점 -- 유일하게 바닥 평면 위에 있는 점)으로
        # 다시 판정한 결과를 별도 타입으로 보낸다.
        #
        # IVA_EVENT를 지우지 않고 나란히 두는 건 교차 검증용이다 -- 둘이 갈리는
        # 지점이 곧 위 오차이고, 화면에서 바로 보이는 편이 낫다.
        msg["rx_ms"] = int(now_ms)  # PY1과 같은 이유 -- 위 주석 참고
        # 과도기 bridge 모드에서만 이 프로세스가 CCTV role을 소유한다. 운영
        # 직결 모드에서는 카메라 앱이 같은 ZONE_EVENT를 중앙 TLS로 직접 보낸다.
        if CCTV_BRIDGE_ENABLED:
            cctv_send("ZONE_EVENT", {k: v for k, v in msg.items() if k != "type"})
        broadcast("[iva] ZONE " + json.dumps(msg, ensure_ascii=False))
        return last_seq
    if mtype == "IVA_ZONE_SET":
        # 폴리곤을 직접 주입해 존을 무장한 결과. IVA_SYNC 응답과 같은 모양이고
        # 브라우저도 같은 캐시에 넣어 오버레이에 그린다 -- 존이 어디서 왔든
        # (앵커 hull이든 손으로 넣은 값이든) 화면에 보이는 건 같아야 한다.
        broadcast("[calib] IVA_ZONE_SET " + json.dumps(msg, ensure_ascii=False))
        return last_seq
    if mtype in ("ZONE_BAND", "ZONE_BANDS"):
        # ZONE_BAND = 밴드 하나의 외곽선(이미 raw 픽셀로 투영된 점 목록),
        # ZONE_BANDS = on/off 와 거리 설정. 둘 다 그리기용이라 그대로 넘긴다.
        broadcast("[calib] " + mtype + " " + json.dumps(msg, ensure_ascii=False))
        return last_seq
    if mtype == "DET_STREAM":
        broadcast("[calib] DET_STREAM " + json.dumps(msg, ensure_ascii=False))
        return last_seq
    if mtype == "WISEAI_DET":
        # 사람 검출 위치 실시간 피드 (DET_STREAM 이 켜져 있을 때만 온다).
        # 초당 수십 건이라 '[det]' 접두어를 따로 둔다 -- 브라우저의 텍스트 로그
        # 창은 이 접두어를 걸러내고 오버레이만 소비한다. 걸러내지 않으면 이
        # 스트림이 로그를 잠가서 다른 메시지가 안 보인다.
        broadcast("[det] " + json.dumps(msg, ensure_ascii=False))
        return last_seq
    if mtype == "DYNROI":
        # ReportDynRoi()가 채널마다 한 번씩(4번) 이 타입을 보낸다 — on/margin/
        # max_miss는 이제 렌즈별로 다를 수 있어(DYNROI_CH, 2026-08-11) ch를
        # 붙인다. track_ids는 DYNROI_IDS가 4채널에 한 번에 적용되는 값이라
        # (카메라 쪽에 아직 채널별 형태가 없음) 채널 구분 없이 그대로 둔다 —
        # 4번 다 같은 내용이라 중복 방송이지만 멱등이라 무해하다.
        ch = msg.get("ch")
        ids = msg.get("track_ids") or []
        broadcast("[dynroi] IDS " + json.dumps(list(ids), separators=(",", ":")))
        scope = f"id {','.join(str(i) for i in ids)}만" if ids else "모든 마커"
        if msg.get("enabled"):
            broadcast(f"[dynroi] ch={ch} 동적 ROI ON — 추적 대상 {scope}, "
                      f"max_margin={msg.get('margin')}px "
                      f"max_miss={msg.get('max_miss')} "
                      f"(마커 크기+이동량으로 margin 자동 조절)")
        else:
            broadcast(f"[dynroi] ch={ch} 동적 ROI OFF — 수동 ROI/전체 화면으로 복귀")
        return last_seq
    if mtype == "DYNROI_STATE":
        # ch: 이 상태(TRACK/SEARCH)는 렌즈마다 독립이다 — 4채널이 각자 자기
        # dynroi_[ch] 인스턴스를 가진다(sample_component.h). ch 필드가 없던 동안은
        # (2026-08-10 이전) 이 줄이 어느 렌즈 것인지 알 방법이 없어서 채널을
        # 바꿔도 화면엔 마지막으로 전이한 아무 렌즈의 상태가 떠 있었다.
        ch = msg.get("ch")
        if msg.get("tracking"):
            detail = (f" margin={msg.get('margin_used')}px"
                      f" marker={msg.get('marker_px')}px"
                      f" move={msg.get('motion_px')}px"
                      if msg.get("margin_used") is not None else "")
            broadcast(f"[dynroi] ch={ch} TRACK — 검출 영역 ({msg.get('x')},{msg.get('y')}) "
                      f"{msg.get('w')}x{msg.get('h')}{detail}")
        else:
            broadcast(f"[dynroi] ch={ch} SEARCH — 마커 놓침, 전체 재탐색 중")
        return last_seq
    if mtype == "CPU_STAT":
        # 카메라가 2초마다 자발적으로 보낸다(요청 없음, ReportCpu() 2026-08-11
        # 추가 — 그 전엔 이 타입 자체가 카메라에 없어서 아래 파싱이 死코드였다).
        # cctv_app의 app_pct/sys_pct 두 값이 아니라 ArucoPosePNM의 /status가
        # 쓰는 것과 같은 필드: cpu_pct(이 앱 vs wall clock)/cores/rss_kb/
        # heap_in_use_kb/core_pct(코어별 전체 부하). -1은 아직 측정 불가.
        cpu = msg.get("cpu_pct")
        cores = msg.get("cores")
        rss = msg.get("rss_kb")
        heap = msg.get("heap_in_use_kb")
        core_txt = ",".join(f"{v:.0f}" for v in (msg.get("core_pct") or []))
        broadcast(f"[cpu] app={cpu}% cores={cores} core=[{core_txt}] rss={rss}KB heap={heap}KB")
        return last_seq
    if mtype == "CALIB_K_QUERY":
        # session/profile 필드는 cctv_app(단일 렌즈, 이름 붙은 프로필) 시절 것이다.
        # ArucoPosePNM은 렌즈당 K/dist 하나뿐이라 그 개념 자체를 포팅하지 않았고
        # (intrinsics_calib.h 상단 설계 코멘트), 애초에 이 필드들을 보낸 적이
        # 없어서 msg.get(...)이 항상 None/기본값만 찍고 있었다. ch를 붙이는 김에
        # 같이 정리한다 (2026-08-10).
        ch = msg.get("ch")
        if msg.get("available"):
            broadcast(f"[calib-K] ch={ch} CURRENT VALUES: fx={msg.get('fx')} fy={msg.get('fy')} "
                      f"cx={msg.get('cx')} cy={msg.get('cy')} dist={msg.get('dist')}")
            # 조회 응답으로도 채널 상태 UI 를 갱신한다. 예전에는 CALIB_K_RESULT
            # (캘리가 *완료되는 순간*)에서만 캐시해서, 이 대시보드를 재시작하면
            # 카메라에는 K/dist 가 멀쩡히 저장돼 있는데도 화면은 "K 없음"이었다.
            #
            # 이 캐시가 번들에 실릴 K/D 의 출처이던 시절에는 그 공백이 실제 사고로
            # 이어졌다(서버팀 8/13 문의의 "K/D 없음" 90건). 지금은 카메라가 자기
            # K/D 를 직접 실어 보내므로 여기 있는 값은 화면 표시 이상의 의미가 없다.
            calib_cache_k(msg.get("fx"), msg.get("fy"), msg.get("cx"),
                          msg.get("cy"), msg.get("dist"), ch)
        else:
            broadcast(f"[calib-K] ch={ch} no calibration loaded on the camera right now")
        return last_seq
    if mtype == "CALIB_K_SAVE":
        if msg.get("ok"):
            broadcast(f"[calib-K] 저장 완료 — 카메라 /mnt(PERSIST_DIR)에 K/dist 기록됨(session={msg.get('session')}), 재부팅해도 유지")
        else:
            broadcast(f"[calib-K] 저장 실패: {msg.get('reason')}")
        return last_seq
    if mtype == "CALIB_K_BOARD_SAVE":
        if msg.get("ok"):
            broadcast("[calib-K] 보드 설정 저장 완료 — 카메라 /mnt(PERSIST_DIR)에 기록됨, 재부팅해도 유지")
        else:
            broadcast(f"[calib-K] 보드 설정 저장 실패: {msg.get('reason')}")
        return last_seq
    if mtype == "RAW_RES":
        lst = msg.get("list", [])
        if lst:
            resstr = ", ".join(f"{w}x{h}" for w, h in lst)
            broadcast(f"[raw-res] SDK reports {msg.get('count')} supported "
                      f"resolution(s) (err={msg.get('err')}): {resstr}")
        else:
            broadcast(f"[raw-res] no resolutions reported (err={msg.get('err')}, "
                      f"count={msg.get('count')}) — API may not behave as documented")
        return last_seq
    if mtype == "LDC_CHECK_ACK":
        state = msg.get("state")
        if state == "checking":
            ldc_check_log_reset()
            broadcast("[ldc] camera acknowledged — hold the ChArUco board in view, "
                      "move it to the frame EDGES/CORNERS. Streams continuously, logging "
                      f"one row per ~{LDC_CHECK_LOG_INTERVAL_S:.0f}s to {LDC_CHECK_LOG_PATH}. "
                      "Type LDC_CHECK_STOP to end.")
        elif state == "stopped":
            broadcast("[ldc] stopped — normal pose streaming resumed")
        else:
            broadcast(f"[ldc] start REJECTED: {msg.get('reason')}")
        return last_seq
    if mtype == "LDC_CHECK":
        mk = msg.get("markers", 0)
        mkt = msg.get("markers_total", 0)
        if not msg.get("found"):
            broadcast(f"[ldc] board partial: markers={mk}/{mkt} corners={msg.get('corners',0)} — show more of the board")
            return last_seq
        rms = msg.get("straight_rms_px", 0.0)
        emax = msg.get("edge_max_px", 0.0)
        cor = msg.get("corners", 0)
        cort = msg.get("corners_total", 0)
        u = msg.get("undistorted")
        ldc_check_log_row(msg, u)
        if u:
            # Before/after: same corners, camera-LDC-only vs + OpenCV undistort.
            broadcast(f"[ldc] BEFORE (camera LDC only): rms={rms:.2f}px "
                      f"edge_max={emax:.2f}px center_max={msg.get('center_max_px',0):.2f}px")
            broadcast(f"[ldc] AFTER  (+ OpenCV undistort): rms={u['straight_rms_px']:.2f}px "
                      f"edge_max={u['edge_max_px']:.2f}px "
                      f"center_max={u['center_max_px']:.2f}px "
                      f"(rms improved {rms - u['straight_rms_px']:+.2f}px)")
            return last_seq
        if emax < 0.5:
            verdict = "EXCELLENT (LDC sufficient)"
        elif emax < 1.5:
            verdict = "OK (adequate for most uses)"
        else:
            verdict = "POOR (server-side undistort advised)"
        broadcast(f"[ldc] markers={mk}/{mkt} corners={cor}/{cort} straight_rms={rms:.2f}px "
                  f"edge_max={emax:.2f}px center_max={msg.get('center_max_px',0):.2f}px -> {verdict}")
        return last_seq

    # CAM_POSE
    seq = msg.get("seq")
    gap = ""
    if (last_seq is not None and seq is not None
            and seq != last_seq and seq != last_seq + 1):
        gap = f"  (seq gap: {last_seq} -> {seq})"

    # ch: ArucoPosePNM은 4채널 전부를 이 한 TCP 링크로 섞어서 보낸다 (0-based).
    # 브로드캐스트 문자열에 없으면 브라우저가 어느 렌즈의 코너인지 구분할 방법이
    # 없어서, 채널을 바꿔도 화면엔 마지막으로 도착한 아무 채널의 코너가 계속
    # 떠 있었다 -- 2026-08-10 발견·수정. handleRaw()가 이 필드로 채널별
    # rawFrame(오버레이)에 나눠 담는다.
    ch = msg.get("ch")
    if msg.get("confidence", 0) > 0:
        corners = msg.get("corners", [])
        cctv_forward_pos(corners)  # 서버(9000, role=CCTV)로 POS 통역 전달
        ctxt = " ".join(f"c{i}=({c['x']:.2f},{c['y']:.2f})" for i, c in enumerate(corners))
        world = msg.get("world")
        wtxt = (f" world=({world['x']:.0f},{world['y']:.0f}mm,{world['theta']:.1f}deg)") if world else ""
        broadcast(f"seq={seq} ch={ch} id={msg.get('id')} {ctxt}{wtxt} "
                  f"{latency_text(msg, now_ms)}{gap}")
    else:
        broadcast(f"seq={seq} ch={ch} MARKER LOST (heartbeat) "
                  f"{latency_text(msg, now_ms)}{gap}")
    return seq


def handle_client(conn, addr):
    global current_conn, current_cam_ip, _clock_warned, _clock_offset_note
    broadcast(f"[+] camera connected: {addr}")
    _reset_clock_offset()
    _clock_offset_note = False
    # Re-arm the skew warning: a reconnect often means the app (or camera)
    # restarted, so the clock situation may have changed and is worth saying
    # once more.
    _clock_warned = False
    with conn_lock:
        current_conn = conn
        current_cam_ip = addr[0]
    conn.settimeout(WATCHDOG_S)
    buf = b""
    last_seq = None
    # Collapse repeated timeouts into one "started" line + one "resumed after
    # Ns" line, instead of flooding the log every WATCHDOG_S while waiting
    # (e.g. during a CALIB_K_START session, where pose streaming is
    # intentionally paused for minutes — that is expected, not an error).
    watchdog_streak = 0
    try:
        while True:
            try:
                chunk = conn.recv(4096)
            except socket.timeout:
                watchdog_streak += 1
                if watchdog_streak == 1:
                    broadcast(f"[!] WATCHDOG: no packet for {WATCHDOG_S}s -> STOP ROBOT "
                              f"(further repeats suppressed until packets resume)")
                continue
            except OSError as e:
                # e.g. ConnectionResetError when the camera app restarts (a
                # rebuild/reinstall) and drops the link abruptly instead of a
                # clean shutdown. Must NOT propagate: an uncaught exception
                # here would kill the tcp_server thread entirely, taking the
                # whole listening port down with it (port 6000 disappearing).
                broadcast(f"[-] camera connection error: {e}")
                return
            if not chunk:
                broadcast("[-] camera disconnected")
                return
            if watchdog_streak > 0:
                broadcast(f"[+] packets resumed after ~{watchdog_streak * WATCHDOG_S:.0f}s gap")
                watchdog_streak = 0
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if not line.strip():
                    continue
                try:
                    msg = json.loads(line)
                except json.JSONDecodeError as e:
                    broadcast(f"[!] bad JSON ({e}): {line[:80]!r}")
                    continue
                hg_experiment_observe(msg)
                last_seq = print_msg(msg, last_seq)
    finally:
        with conn_lock:
            current_conn = None
            current_cam_ip = None
        conn.close()


def tcp_server():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", TCP_PORT))
    srv.listen(1)
    broadcast(f"listening on 0.0.0.0:{TCP_PORT} ...")
    while True:
        conn, addr = srv.accept()
        try:
            handle_client(conn, addr)
        except Exception as e:
            # Second line of defense: whatever goes wrong with one connection
            # must not kill this loop, or the listening port silently vanishes
            # (as just happened) and every later reconnect attempt fails.
            broadcast(f"[!] tcp_server: unexpected error, still listening: {e}")


# --------------------------------------------------------------------------
# LDC_SNAPSHOT: a separate, one-shot-per-connection channel (see
# snapshot_sender.h on the camera side for why it's not mixed with the
# realtime pose channel). Wire format: uint32 json_len, json bytes,
# uint32 width, uint32 height, uint32 pixel_len, RGB24 row-major pixels.
# --------------------------------------------------------------------------

def recv_exact(conn, n):
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("connection closed mid-snapshot")
        buf += chunk
    return buf


def handle_snapshot_client(conn, addr):
    try:
        (json_len,) = struct.unpack(">I", recv_exact(conn, 4))
        msg = json.loads(recv_exact(conn, json_len))
        width, height, pixel_len = struct.unpack(">III", recv_exact(conn, 12))
        is_jpeg = msg.get("format") == "jpeg"
        if not is_jpeg and pixel_len != width * height * 3:
            broadcast(f"[!] snapshot from {addr}: size mismatch "
                      f"({width}x{height} implies {width*height*3}, got {pixel_len})")
            return
        payload = recv_exact(conn, pixel_len)
    except (ConnectionError, OSError, struct.error, json.JSONDecodeError) as e:
        broadcast(f"[!] snapshot from {addr} failed: {e}")
        return

    ts = time.strftime("%Y%m%d_%H%M%S")
    mtype = msg.get("type")

    # HG_SNAPSHOT floor reference still (also JPEG, so it must be caught before
    # the calibration-view branch below). Latest-wins: overwrite one file + its
    # meta (H + anchor world/pixel coords) that the dashboard canvas overlays.
    if is_jpeg and mtype == "HG_REF":
        try:
            with open(HG_REFERENCE_PATH, "wb") as f:
                f.write(payload)
            meta = {"w": msg.get("w"), "h": msg.get("h"),
                    "H": msg.get("H"), "anchors": msg.get("anchors", []),
                    "ts": ts}
            with open(HG_META_PATH, "w") as f:
                json.dump(meta, f)
            broadcast(f"[hg] 기준영상 수신 {width}x{height} ({len(payload)} bytes), "
                      f"anchors={len(meta['anchors'])}, "
                      f"H={'있음' if meta['H'] else '없음(캘리브 전)'}")
        except OSError as e:
            broadcast(f"[hg] 기준영상 저장 실패: {e}")
        return

    # Pre-compressed calibration view: write the JPEG bytes verbatim (no codec
    # needed). This is the CALIB_K_UPLOAD path, and it runs TWICE per view --
    # once for the plain original, once for the overlay. Both share a base name
    # (session stamp + view number), so the two images and the JSON below sit
    # together in a directory listing.
    if is_jpeg:
        view = msg.get("view")
        kind = msg.get("kind", "overlay")   # older camera builds sent overlay only
        stamp = calib_session_stamp(msg.get("session"), ts)
        os.makedirs(CALIB_VIEW_DIR, exist_ok=True)
        base = (f"{CALIB_VIEW_DIR}/calib_view_{view:02d}_{stamp}"
                if isinstance(view, int) else f"{CALIB_VIEW_DIR}/snapshot_{stamp}")
        jpg_path = base + ("_raw.jpg" if kind == "plain" else ".jpg")
        with open(jpg_path, "wb") as f:
            f.write(payload)

        # Save the corners and the session's K/dist next to the images. The
        # corners are the whole point of the upload: they are what the camera
        # actually fitted, measured on the raw NV12 frame, so they -- not the
        # JPEGs -- are the source of truth for any offline re-fit. Re-detecting
        # from a JPEG would return slightly different sub-pixel values
        # (compression), and from the overlay one it would fail outright, since
        # the drawn rings cover the marker bits. Each entry is [x, y, id]; the id
        # is what maps a pixel to a board coordinate
        # (CharucoBoard.matchImagePoints), so it must survive.
        #
        # Both uploads of a view carry the same corners/K, so this rewrites
        # identical content the second time -- harmless, and it means either
        # upload alone is enough to preserve the measurements.
        charuco = msg.get("charuco")
        saved = ""
        if charuco:
            with open(base + ".json", "w") as f:
                # "board" is what makes this file re-fittable on its own: a corner
                # id only means something against the board it was measured on,
                # and the board is changeable at runtime (CALIB_K_CONFIG).
                json.dump({"view": view, "target": msg.get("target"),
                           "session": msg.get("session"),
                           "w": msg.get("w"), "h": msg.get("h"),
                           "image_overlay": os.path.basename(base) + ".jpg",
                           "image_plain": os.path.basename(base) + "_raw.jpg",
                           "board": msg.get("board"),
                           "intrinsics": msg.get("intrinsics"),
                           "charuco": charuco}, f, indent=1)
            saved = f" + {len(charuco)} corners -> {os.path.basename(base)}.json"
        broadcast(f"[calib-K] view {view}/{msg.get('target')} {kind} JPEG saved "
                  f"{jpg_path} ({msg.get('corners')} corners, "
                  f"{len(payload)} bytes){saved}")
        return

    rgb = payload
    if mtype == "CALIB_K_VIEW":
        ppm_path = f"{SNAPSHOT_DIR}/calib_view_{msg.get('view'):02d}_{ts}.ppm"
    else:
        ppm_path = f"{SNAPSHOT_DIR}/ldc_snapshot_{ts}.ppm"
    with open(ppm_path, "wb") as f:
        f.write(f"P6\n{width} {height}\n255\n".encode())
        f.write(rgb)

    if mtype == "CALIB_K_VIEW":
        broadcast(f"[calib-K] view {msg.get('view')}/{msg.get('target')} image saved "
                  f"{ppm_path} ({msg.get('corners')} corners)")
        return
    u = msg.get("undistorted")
    if msg.get("found"):
        broadcast(f"[snapshot] saved {ppm_path} — markers={msg.get('markers')}/"
                  f"{msg.get('markers_total')} corners={msg.get('corners')}/"
                  f"{msg.get('corners_total')}")
        broadcast(f"[snapshot] BEFORE (camera LDC only): "
                  f"rms={msg.get('straight_rms_px', 0):.2f}px "
                  f"edge_max={msg.get('edge_max_px', 0):.2f}px "
                  f"center_max={msg.get('center_max_px', 0):.2f}px")
        if u:
            broadcast(f"[snapshot] AFTER  (+ OpenCV undistort): "
                      f"rms={u['straight_rms_px']:.2f}px "
                      f"edge_max={u['edge_max_px']:.2f}px "
                      f"center_max={u['center_max_px']:.2f}px "
                      f"(rms improved {msg.get('straight_rms_px', 0) - u['straight_rms_px']:+.2f}px)")
    else:
        broadcast(f"[snapshot] saved {ppm_path} — board not fully seen "
                  f"(markers={msg.get('markers')}/{msg.get('markers_total')})")

    csv_path = f"{SNAPSHOT_DIR}/ldc_snapshots.csv"
    is_new = False
    try:
        with open(csv_path, "r"):
            pass
    except FileNotFoundError:
        is_new = True
    with open(csv_path, "a") as f:
        if is_new:
            f.write("timestamp,file,markers,markers_total,corners,corners_total,"
                    "straight_rms_px,straight_max_px,edge_max_px,center_max_px,"
                    "u_straight_rms_px,u_straight_max_px,u_edge_max_px,u_center_max_px\n")
        f.write(f"{ts},{ppm_path},{msg.get('markers')},{msg.get('markers_total')},"
                f"{msg.get('corners')},{msg.get('corners_total')},"
                f"{msg.get('straight_rms_px', '')},{msg.get('straight_max_px', '')},"
                f"{msg.get('edge_max_px', '')},{msg.get('center_max_px', '')},"
                f"{u.get('straight_rms_px', '') if u else ''},"
                f"{u.get('straight_max_px', '') if u else ''},"
                f"{u.get('edge_max_px', '') if u else ''},"
                f"{u.get('center_max_px', '') if u else ''}\n")


def snapshot_server():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", SNAPSHOT_PORT))
    srv.listen(1)
    broadcast(f"listening for LDC_SNAPSHOT uploads on 0.0.0.0:{SNAPSHOT_PORT} ...")
    while True:
        conn, addr = srv.accept()
        with conn:
            handle_snapshot_client(conn, addr)


PAGE = """<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>CCTV Calibration Manager</title>
<script>
// Runs before the stylesheet paints, so the page never flashes the wrong theme.
// Saved choice wins; with none, follow the OS. Everything else keys off this
// one attribute on <html>.
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
  /* Card-based layout, themed entirely through these variables -- nothing below
     hardcodes a colour, so the dark block is the only thing that has to change.
     The consoles (#log, #shOut) stay dark in BOTH themes: a fast-scrolling
     transcript is what a terminal is for, and keeping it inverted in light mode
     separates "machine output" from "controls" at a glance.
     The theme is chosen in a <head> script (before paint, so no flash) from
     localStorage, falling back to the OS preference. */
  :root {
    --bg:#F2F4F6; --card:#FFFFFF;
    /* Toss gray scale, one step darker than stock. The old text3 (#8B95A1) sat
       at 3.0:1 on white -- the "회색이 잘 안 보인다" complaint. #65707E clears
       WCAG AA (4.5:1) on ALL three light surfaces it lands on: --card 5.03,
       --field 4.82, --btn 4.57. Do not lighten it without rechecking --btn,
       which is the tightest of the three. */
    --text:#191F28; --text2:#333D4B; --text3:#65707E;
    --line:#E5E8EB;
    --btn:#F2F4F6; --btn-hover:#E5E8EB; --seg:#EDEFF2;
    --field:#F9FAFB; --input:#FFFFFF;
    --track:rgba(0,0,0,.06);
    --shadow:0 1px 2px rgba(0,0,0,.04);
    --blue:#3182F6; --blue-dark:#1B64DA; --blue-bg:#E8F3FF;
    --red:#F04452; --red-bg:#FEF0F1;
    --green:#15C39A; --green-bg:#E7F9F4;
    --warn-bg:#FFF7ED; --warn-line:#FED7AA; --warn-text:#9A3412;
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
    /* Saturated blues vibrate on dark; these are lifted for legibility. */
    --blue:#4593FC; --blue-dark:#8CBCFF; --blue-bg:#1A2C44;
    --red:#FF6B78; --red-bg:#3A1E22;
    --green:#2AD8A8; --green-bg:#153229;
    --warn-bg:#332415; --warn-line:#6B4A22; --warn-text:#FBBF77;
    --console:#0E0F11; --console-text:#C9D1D9;
  }
  * { box-sizing:border-box; }
  html, body { height:100%; }
  body { font-family:-apple-system,BlinkMacSystemFont,'Pretendard','Malgun Gothic',
         system-ui,sans-serif; background:var(--bg); color:var(--text);
         margin:0; padding:16px 18px; display:flex; flex-direction:column;
         height:100vh; overflow:hidden; -webkit-font-smoothing:antialiased; }

  /* Header: title, tabs and the global toggles all share one row. Wraps on a
     narrow window rather than overflowing. */
  #top { display:flex; align-items:center; gap:14px; margin-bottom:14px; flex-wrap:wrap; }
  h1 { font-size:19px; font-weight:700; margin:0; letter-spacing:-.3px; white-space:nowrap; }
  #cmdbox { display:flex; gap:8px; align-items:center; margin-left:auto; flex-wrap:wrap; }
  .helpbtn { font-size:12px; font-weight:600; padding:7px 12px; border:1px solid var(--line);
             border-radius:10px; background:var(--card); color:var(--text2); cursor:pointer;
             white-space:nowrap; }
  .helpbtn:hover { background:var(--field); }
  .helpbtn.on { background:var(--blue); border-color:var(--blue); color:#fff; }
  .toggle { color:var(--text2); font-size:12px; font-weight:500; display:flex;
            align-items:center; gap:6px; white-space:nowrap; cursor:pointer; }
  .toggle input { accent-color:var(--blue); width:15px; height:15px; }

  /* Segmented tab bar — sits inline in the header next to the title. */
  #tabbar { display:inline-flex; gap:4px; background:var(--seg); padding:4px;
            border-radius:14px; flex-wrap:wrap; }
  .tab { padding:8px 14px; font-size:13px; font-weight:600; cursor:pointer;
         background:transparent; color:var(--text3); border:0; border-radius:10px;
         transition:background .15s, color .15s; }
  .tab:hover { color:var(--text2); }
  .tab.active { background:var(--card); color:var(--text); box-shadow:0 1px 3px rgba(0,0,0,.12); }

  /* Panes. Every non-default pane must be listed here, or showTab()'s
     display:flex lands on an unstyled box.
     All four tabs share this one rule, so the controls column is the same
     width everywhere. 580px is a flex-BASIS, not a fixed width: the pane still
     gives way on a narrow window instead of shoving the log off-screen. It is
     deliberately narrow -- these are short fields and buttons, and every pixel
     spent stretching them is a pixel taken from the log. */
  #content { display:flex; gap:14px; flex:1; min-height:0; }
  #groups, #homographyPane, #shellPane, #rawPane, #opsPane, #centralPane {
            display:flex; flex-flow:row wrap; align-content:flex-start;
            align-items:flex-start; gap:12px; flex:0 1 580px; min-width:0;
            overflow-y:auto; padding-right:4px; margin:0; }

  /* 터미널(오른쪽 로그) 접기 — 헤더의 [터미널] 버튼이 토글한다. 로그를 숨기면 왼쪽
     작업영역이 전체 폭을 쓴다. 좌표표·배치도처럼 넓게 봐야 하는 화면에서 필요하다.
     탭 pane 을 새로 만들면 아래 선택자에도 반드시 추가할 것 — 빠지면 그 탭만
     접기가 안 먹는다. */
  #content.log-collapsed > #main { display:none; }
  #content.log-collapsed > #groups,
  #content.log-collapsed > #homographyPane,
  #content.log-collapsed > #shellPane,
  #content.log-collapsed > #rawPane,
  #content.log-collapsed > #opsPane,
  #content.log-collapsed > #centralPane { flex:1 1 auto; }

  /* 앵커·검증점 배치도(그리드): 작업영역 평면 위에 점을 드래그로 배치한다.
     파랑=계산 앵커, 주황=검증 기준점 — 오버레이 범례와 색을 맞춘다.
     touch-action:none 은 터치에서 드래그가 스크롤로 먹히지 않게 하려는 것. */
  .hg-map { width:100%; max-width:520px; height:auto; display:block;
            border:1px solid var(--line); border-radius:10px;
            background:var(--field); touch-action:none; margin-top:4px; }

  /* 주행 배치도. .hg-map 과 모양은 같지만 클래스를 나눈다 — 스크립트 끝에서
     document.querySelectorAll('.hg-map') 에 앵커 드래그 핸들러를 일괄로 붙이는데,
     같은 클래스를 쓰면 이 SVG 에도 그 핸들러가 걸려 앵커 표를 건드리게 된다. */
  .odo-map { width:100%; max-width:520px; height:auto; display:block;
             border:1px solid var(--line); border-radius:10px;
             background:var(--field); touch-action:none; margin-top:4px; }
  /* 배치도 | 진행도 2단. 좁은 화면에서는 wrap 으로 세로로 쌓인다 — 진행도 표가
     8열이라 억지로 옆에 붙여두면 표가 찌그러진다. */
  .odo-cols { display:flex; flex-wrap:wrap; gap:12px; align-items:flex-start; }
  .odo-cols > .qbox { flex:1 1 380px; min-width:0; }
  .odo-cols table { font-size:11px; }
  .odo-st-ok   { color:#10b981; font-weight:700; }
  .odo-st-fail { color:#ef4444; font-weight:700; }
  .odo-st-wait { color:#f59e0b; font-weight:700; }
  .odo-row-now { background:rgba(245,158,11,0.10); }

  /* Cards */
  .group { background:var(--card); border-radius:18px; padding:18px 20px;
           align-self:flex-start; box-shadow:var(--shadow); }
  .group.wide { flex:1 1 100%; }
  .group.narrow { flex:1 1 240px; max-width:290px; }
  .group.mid { flex:1 1 420px; max-width:460px; }
  h2 { font-size:15px; font-weight:700; color:var(--text); margin:0 0 4px;
       letter-spacing:-.2px; }
  .sub { color:var(--text3); font-size:12px; line-height:1.6; margin:0 0 12px; }

  /* Callouts. Padding lives on the summary/body, not here, so the collapsed
     state is just the one-line summary. */
  .tip { background:var(--blue-bg); color:var(--blue-dark); border-radius:12px;
         font-size:12px; line-height:1.65; margin:0 0 12px; }
  .tip b { font-weight:700; }
  .warn { background:var(--warn-bg); border:1px solid var(--warn-line); color:var(--warn-text);
          border-radius:12px; font-size:12px; line-height:1.65; margin:0 0 12px; }
  .warn code { background:var(--track); }

  /* Collapsible blocks. Plain <details> -- works with no JS, keyboard operable,
     and Ctrl-F still finds text in a closed one. The summary must carry the
     actionable half of the message on its own, because that is all most people
     will ever read; the body holds the reasoning.
     Open/closed is remembered per block (see the fold script). */
  details.fold { }
  details.fold > summary { cursor:pointer; list-style:none; display:flex;
      align-items:center; gap:8px; padding:10px 13px; font-size:12px;
      font-weight:600; border-radius:12px; }
  details.fold > summary::-webkit-details-marker { display:none; }
  details.fold > summary::after { content:''; width:6px; height:6px; flex:none;
      margin-left:auto; border-right:2px solid currentColor;
      border-bottom:2px solid currentColor; transform:rotate(45deg)
      translate(-2px,-2px); transition:transform .15s; opacity:.65; }
  details.fold[open] > summary::after { transform:rotate(-135deg) translate(-3px,-3px); }
  details.fold > summary:hover { filter:brightness(1.04); }
  .foldbody { padding:0 13px 12px; font-size:12px; line-height:1.65; }
  /* A neutral (non-callout) fold: the board spec form. */
  .panel { border:1px solid var(--line); border-radius:12px; margin:0 0 12px; }
  .panel > summary { color:var(--text); }
  .brief { font-family:var(--mono); font-size:11px; font-weight:500;
           color:var(--text3); }

  /* Rows + buttons. Default is the quiet gray; exactly one primary (blue) per
     card, so the next step is never ambiguous. */
  .row { display:flex; align-items:center; gap:8px; margin:6px 0; flex-wrap:wrap; }
  /* 운영 탭만 컨트롤을 세로로 쌓는다(2026-08-12 요청). 다른 탭의 .row 는 그대로 —
     전역으로 바꾸면 캘리·호모그래피 탭이 통째로 길어진다. */
  #opsPane .row { flex-direction:column; align-items:flex-start; gap:6px; }
  .row button { display:inline-flex; flex-direction:column; align-items:flex-start;
                gap:1px; padding:10px 14px; font-size:13px; font-weight:600;
                font-family:inherit; cursor:pointer; border:0; border-radius:12px;
                background:var(--btn); color:var(--text2); min-width:136px;
                flex-shrink:0; transition:background .15s, color .15s; }
  .row button:hover { background:var(--btn-hover); }
  .row button .cmd { font-family:var(--mono); font-size:10px; font-weight:500;
                     opacity:.6; letter-spacing:-.2px; }
  .row button.on { background:var(--blue); color:#fff; }
  .go button { background:var(--blue); color:#fff; }
  .go button:hover { background:var(--blue-dark); }
  .stop button { background:var(--red-bg); color:var(--red); }
  .stop button:hover { filter:brightness(1.08); }
  button:disabled { opacity:.4; cursor:not-allowed !important; }
  .row .desc { color:var(--text3); font-size:11px; line-height:1.55; display:none;
               flex:1 1 100%; }
  body.show-help .row .desc { display:block; }
  .cmdflow { display:flex; flex-flow:row wrap; gap:6px; margin-top:6px; }
  .cmdflow .row { margin:0; }
  body.show-help .cmdflow { flex-direction:column; }
  /* Homography has several distinct operator workflows. These local tabs keep
     the main tab compact without hiding the relevant workflow behind a long
     sequence of unrelated fold panels. */
  .hg-subtabs { display:flex; gap:5px; flex:1 1 100%; padding:4px;
                background:var(--seg); border-radius:14px; }
  .hg-subtab { flex:1; padding:9px 10px; border:0; border-radius:10px;
               background:transparent; color:var(--text3); font-family:inherit;
               font-size:13px; font-weight:600;
               cursor:pointer; }
  .hg-subtab:hover { color:var(--text2); }
  .hg-subtab.active { background:var(--card); color:var(--text); box-shadow:0 1px 3px rgba(0,0,0,.12); }

  /* Capture status — the one number the operator watches during a session. */
  #kstatus { background:var(--blue-bg); border-radius:14px; padding:14px 16px;
             margin:12px 0; }
  #kcount { font-size:26px; font-weight:700; color:var(--blue); letter-spacing:-.6px;
            line-height:1.2; }
  /* Two lines reserved: the status text swaps between short ("세션 시작 — …")
     and long (a reject reason plus five metrics) on every capture, and without
     this the card -- and every button under it -- shifts as you work. */
  #kinfo { color:var(--text2); font-size:12px; line-height:1.55; margin-top:3px;
           min-height:37px; }
  #kbar { height:6px; background:var(--track); border-radius:99px; margin-top:11px;
          overflow:hidden; }
  #kbar i { display:block; height:100%; width:0%; background:var(--blue);
            border-radius:99px; transition:width .3s ease; }
  /* updateKStatus() assigns className outright ('', 'reject', 'done'), so the
     base look must hang off the id, not a class it would wipe. */
  #kstatus.reject { background:var(--red-bg); }
  #kstatus.reject #kcount { color:var(--red); }
  #kstatus.reject #kbar i { background:var(--red); }
  #kstatus.done { background:var(--green-bg); }
  #kstatus.done #kcount { color:var(--green); }
  #kstatus.done #kbar i { background:var(--green); }

  /* Board form: exactly 3 columns, capped and packed left. Not auto-fit -- the
     column count would then drift with the pane width, and these seven fields
     read best as a stable 3x3 block. None of them holds more than a few
     characters, so stretching them wider only makes them harder to scan. */
  .boardcfg { display:grid; grid-template-columns:repeat(3,minmax(0,132px));
              justify-content:start; gap:10px 12px; margin:10px 0 12px; padding:14px;
              background:var(--field); border-radius:14px; }
  .boardcfg label { color:var(--text3); font-size:11px; font-weight:500; }
  .boardcfg input, .boardcfg select { display:block; box-sizing:border-box; width:100%;
              margin-top:5px; padding:8px 9px; background:var(--input); color:var(--text);
              border:1px solid var(--line); border-radius:9px; font-size:13px;
              font-family:inherit; }
  #boardSummary { grid-column:1/-1; color:var(--text2); font-size:12px; line-height:1.55; }
  .kparam { color:var(--text3); font-size:11px; font-weight:500; display:inline-flex;
            align-items:center; gap:6px; }
  .kparam input { width:64px; padding:7px 8px; background:var(--input); color:var(--text);
                  border:1px solid var(--line); border-radius:9px; font-size:13px;
                  font-family:inherit; }
  .boardcfg input:focus, .boardcfg select:focus, .kparam input:focus, #shInput:focus {
                  outline:none; border-color:var(--blue);
                  box-shadow:0 0 0 3px rgba(49,130,246,.15); }

  /* Readout boxes (raw corners, H matrix, K values). Wide content scrolls inside
     its own box rather than widening the page. */
  .rawbox, .qbox { background:var(--field); border:1px solid var(--line); border-radius:14px;
                   padding:12px 14px; font-family:var(--mono); font-size:12px;
                   color:var(--text2); overflow-x:auto; }
  .rawbox { min-height:40px; }
  .qbox { margin-top:8px; }

  /* Live per-frame readouts get a FIXED height and scroll inside.
     handleRaw() rebuilds these on every CAM_POSE line -- 4-10x a
     second -- and the row count follows how many markers that frame saw, with a
     MARKER LOST frame emptying them to a single line. Sized to content they
     yo-yo between ~40px and ~400px several times a second and shove everything
     below them around. A fixed box is the only thing that holds still; the
     scrollbar only appears when several markers are up at once (e.g. the four
     homography anchors). */
  #rawCorners { height:210px; overflow-y:auto; }
  /* Not per-frame variable -- once frames flow it is always the same two tables
     -- so it only needs its final size reserved while it says "대기 중". */
  #rawLatency { min-height:186px; }
  .qtitle { color:var(--text); font-weight:700; font-size:12px; margin-bottom:6px;
            font-family:-apple-system,system-ui,sans-serif; }
  .rawbox .mid { color:var(--blue); font-weight:700; margin-top:6px; }
  .rawbox .none, .qbox .none { color:var(--text3); }
  .rawbox table, .qbox table { border-collapse:collapse; margin:2px 0 6px; }
  .rawbox td { padding:2px 14px 2px 0; color:var(--text2); }
  .rawbox .coord-raw { color:#d97706; font-weight:700; white-space:nowrap; }
  .rawbox .coord-undist { color:#0891b2; font-weight:700; white-space:nowrap; }
  .rawbox .coord-arrow { color:var(--text3); font-size:15px; padding:0 8px; }
  .qbox td { padding:2px 14px 2px 0; }
  .qbox td:first-child { color:var(--text3); }
  .qbox td:last-child { color:var(--text); font-weight:600; text-align:right; }
  .hint { color:var(--text3); font-size:12px; line-height:1.65; }
  .hint b { color:var(--text2); font-weight:600; }
  .hint code, .sub code { background:var(--btn); padding:1px 5px; border-radius:5px;
            font-family:var(--mono); font-size:11px; color:var(--text2); }

  /* Consoles */
  #main { display:flex; flex-direction:column; gap:8px; flex:1; min-height:0; margin:0; }
  #log { background:var(--console); color:var(--console-text); padding:14px; flex:1; min-height:0;
         width:auto; overflow-y:auto; white-space:pre-wrap; font-family:var(--mono);
         font-size:12px; line-height:1.6; border-radius:16px; margin:0; }
  #shInput { flex:1; max-width:380px; min-width:0; padding:10px 12px;
             font-family:var(--mono); font-size:13px; background:var(--input); color:var(--text);
             border:1px solid var(--line); border-radius:12px; }
  #shellControls { flex-wrap:nowrap; }
  #shellControls #shInput { max-width:none; }
  #shellControls button { min-width:0; padding:10px 11px; white-space:nowrap; }
  /* 셸 탭은 진짜 터미널처럼: 출력과 입력줄이 한 상자 안에 붙어 있고, 상자 아무
     데나 누르면 입력에 포커스가 간다. 콘솔은 두 테마 모두에서 어둡게 유지한다. */
  .term { background:var(--console); border-radius:14px; margin:10px 0 0;
          border:1px solid #000; overflow:hidden; display:flex; flex-direction:column; }
  #shOut { background:none; color:var(--console-text); padding:14px 14px 6px; margin:0;
           min-height:220px; max-height:46vh; overflow:auto; white-space:pre-wrap;
           word-break:break-all; font-family:var(--mono); font-size:12px;
           line-height:1.6; }
  .term-echo { color:#7dd3fc; }            /* 사용자가 친 명령 */
  .term-row { display:flex; align-items:center; gap:8px; padding:6px 14px 12px;
              font-family:var(--mono); font-size:12px; }
  .term-row .ps1 { color:#4ade80; font-weight:700; user-select:none; }
  .term-row #shInput { flex:1; min-width:0; background:none; border:none; outline:none;
                       color:var(--console-text); font-family:var(--mono); font-size:12px;
                       padding:2px 0; max-width:none; }
  .term-row #shInput::placeholder { color:#6b7280; }

  /* 빠른 명령: 버튼 더미 대신 "명령 = 설명" 표. 실제로 실행될 문자열을 그대로
     보여줘야 무엇이 돌아가는지 읽고 판단할 수 있다. */
  .cmdtable { width:100%; border-collapse:collapse; font-size:12px; margin:4px 0 0; }
  .cmdtable td { padding:5px 8px; border-bottom:1px solid var(--line); vertical-align:top; }
  .cmdtable tr:last-child td { border-bottom:none; }
  .cmdtable td.c { width:1%; white-space:nowrap; }
  .cmdtable tr.grp td { border-bottom:none; padding:12px 8px 2px; font-weight:700;
                        color:var(--muted); font-size:11px; letter-spacing:.03em; }
  .cmdlink { font-family:var(--mono); font-size:12px; background:var(--field);
             border:1px solid var(--line); border-radius:7px; padding:3px 8px;
             cursor:pointer; color:inherit; text-align:left; }
  .cmdlink:hover { border-color:var(--blue); color:var(--blue); }

  /* Below this the side-by-side split stops working: the controls pane and the
     log would each be too narrow to read. Stack them and hand scrolling back to
     the page -- the desktop layout deliberately pins the viewport
     (height:100vh; overflow:hidden) so the log scrolls internally, but that
     same rule would clip content on a phone. */
  @media (max-width: 900px) {
    body { height:auto; min-height:100vh; overflow:auto; }
    #content { flex-direction:column; }
    #groups, #homographyPane, #shellPane, #rawPane, #opsPane, #centralPane {
      flex:0 0 auto; overflow-y:visible; padding-right:0; }
    .group.wide, .group.narrow { flex:1 1 100%; max-width:none; }
    #main { flex:0 0 auto; }
    #log { min-height:240px; max-height:50vh; }
  }
  /* Phone-ish: the board form keeps its 3 columns but drops the width cap so
     they share whatever the screen has. */
  @media (max-width: 560px) {
    body { padding:10px; }
    .boardcfg { grid-template-columns:repeat(3,minmax(0,1fr)); }
    #shInput { max-width:none; width:100%; }
    .row { flex-wrap:wrap; }
    .row button { min-width:0; flex:1 1 auto; }
    .tab { padding:7px 11px; font-size:12px; }
  }
</style>
</head>
<body>

<div id="top">
  <h1>CCTV Calibration Manager</h1>
  <div id="tabbar">
    <button type="button" id="tabCalib" class="tab active" onclick="showTab('calib')">캘리브레이션</button>
    <button type="button" id="tabHmg" class="tab" onclick="showTab('homography')">호모그래피</button>
    <button type="button" id="tabRaw" class="tab" onclick="showTab('raw')">마커 검출</button>
    <button type="button" id="tabOps" class="tab" onclick="showTab('ops'); opsDraw()"
            title="fps·지연·검출률·CPU 의 시간축 추이 (표시 전용)">운영</button>
    <button type="button" id="tabCentral" class="tab" onclick="showTab('central')"
            title="카메라가 중앙 서버로 직접 보내는 채널 (이 대시보드의 서버 연결과는 별개)">서버 송신</button>
    <button type="button" id="tabShell" class="tab" onclick="showTab('shell')">셸</button>
  </div>
  <div id="cmdbox">
    <!-- 채널 선택 (프로토콜 v0.4). 채널마다 렌즈 방향이 달라 캘리브레이션이
         완전히 별개다 — 여기서 고른 채널로 결과가 저장된다.
         1채널 카메라(PNO)면 RP_CAM_CHANNELS=1 로 두어 통째로 숨긴다. -->
    <span id="chBox" class="helpbtn" style="display:none;padding:0;border:none;background:none">
      <label style="display:inline-flex;align-items:center;gap:6px">
        <b>채널</b>
        <select id="calibCh" onchange="setCalibChannel(this.value)"
                title="캘리브레이션 결과가 저장될 채널. 바꾸면 카메라에도 SELECT_CHANNEL 을 보냅니다."></select>
      </label>
      <span id="chHint" class="brief"></span>
    </span>
    <!-- 채널별 검출/동적ROI 한눈에 보기. 채널을 열로(가로로 짧게), 검출/dynROI를
         행으로 — 2026-08-11에 사용자 요청으로 행/열을 바꿈(원래는 채널이 행).
         알약을 누르면 그 채널만 토글된다: 검출은 DETECT <ch> <on>, dynROI는
         DYNROI_CH <ch> <on> <그 채널의 캐시된 margin/실패허용> — 여러 채널을
         동시에 켤 수 있고, 안 켠 채널은 계속 꺼진 채로 남는다(카메라 쪽은 이미
         채널별로 완전히 독립). dynROI 세부값(margin/실패허용) 조정은 여전히
         마커검출 탭에서. -->
    <span class="helpbtn" style="padding:2px 8px;cursor:default">
      <table id="chStatusTable" style="border-collapse:collapse;font-size:11px;line-height:1.2">
        <tr style="color:var(--text3)">
          <td></td><td style="padding:0 4px;text-align:center">CH1</td><td style="padding:0 4px;text-align:center">CH2</td>
          <td style="padding:0 4px;text-align:center">CH3</td><td style="padding:0 4px;text-align:center">CH4</td>
        </tr>
        <tr><td style="padding:0 4px;color:var(--text3)">검출</td>
          <td id="chDetCell0" style="text-align:center;padding:2px 4px"></td>
          <td id="chDetCell1" style="text-align:center;padding:2px 4px"></td>
          <td id="chDetCell2" style="text-align:center;padding:2px 4px"></td>
          <td id="chDetCell3" style="text-align:center;padding:2px 4px"></td></tr>
        <tr><td style="padding:0 4px;color:var(--text3)">dynROI</td>
          <td id="chRoiCell0" style="text-align:center;padding:2px 4px"></td>
          <td id="chRoiCell1" style="text-align:center;padding:2px 4px"></td>
          <td id="chRoiCell2" style="text-align:center;padding:2px 4px"></td>
          <td id="chRoiCell3" style="text-align:center;padding:2px 4px"></td></tr>
      </table>
    </span>
    <button type="button" id="logBtn" class="helpbtn on" onclick="toggleLogPanel()"
            title="오른쪽 로그(터미널) 패널 접기/펴기">터미널</button>
    <button type="button" id="themeBtn" class="helpbtn" onclick="toggleTheme()">테마</button>
    <button type="button" id="helpBtn" class="helpbtn" onclick="toggleHelp()">도움말</button>
    <label class="toggle"><input type="checkbox" id="hideLost"> MARKER LOST 로그 숨기기</label>
  </div>
</div>

<div id="content">
<div id="groups">

  <div class="group wide">
    <h2>카메라 캘리브레이션 (K/dist)</h2>
    <p class="sub">인쇄물의 실제 치수를 먼저 맞추고, 세션을 시작한 뒤 자세를 바꿔가며 캡처하세요.</p>

    <details class="fold tip" id="foldTilt">
      <summary><b>보드를 손에 들고 기울여서 — 상하·좌우 30~45°</b></summary>
      <div class="foldbody">
        책상에 평평히 놓고 밀거나 돌리기만 하면 몇 장을 찍든 정보가 늘지 않아
        세션 전체가 무효가 됩니다. 품질 게이트는 위치·거리만 보고 각도는 검사하지 않으므로
        이 실수를 걸러주지 못합니다. 프레임 <b>네 귀퉁이</b>로 밀어 일부가 잘린 뷰,
        화면의 <b>30~50%</b>를 채우는 가까운 뷰도 꼭 섞으세요.
      </div>
    </details>

    <details class="fold panel" id="foldBoard">
      <summary>ChArUco 보드 규격 <span id="boardBrief" class="brief"></span></summary>
      <div class="foldbody">
    <div class="boardcfg">
      <label>체스 칸 가로<input id="bsx" type="number" min="3" value="7"></label>
      <label>체스 칸 세로<input id="bsy" type="number" min="3" value="5"></label>
      <label>한 칸 크기 (mm)<input id="bsquare" type="number" min="1" step="0.1" value="70"></label>
      <label>ArUco 검은 크기 (mm)<input id="bmarker" type="number" min="1" step="0.1" value="50"></label>
      <label>Dictionary<select id="bdict">
        <option value="0">DICT_4X4_50</option>
        <option value="1">DICT_4X4_100</option>
        <option value="2">DICT_4X4_250</option>
        <option value="3">DICT_4X4_1000</option>
        <option value="4">DICT_5X5_50</option>
        <option value="5">DICT_5X5_100</option>
        <option value="6">DICT_5X5_250</option>
        <option value="7">DICT_5X5_1000</option>
        <option value="8">DICT_6X6_50</option>
        <option value="9">DICT_6X6_100</option>
        <option value="10">DICT_6X6_250</option>
        <option value="11">DICT_6X6_1000</option>
        <option value="12">DICT_7X7_50</option>
        <option value="13">DICT_7X7_100</option>
        <option value="14">DICT_7X7_250</option>
        <option value="15">DICT_7X7_1000</option>
        <option value="16">DICT_ARUCO_ORIGINAL</option>
        <option value="17">DICT_APRILTAG_16h5</option>
        <option value="18">DICT_APRILTAG_25h9</option>
        <option value="19">DICT_APRILTAG_36h10</option>
        <option value="20">DICT_APRILTAG_36h11</option>
      </select></label>
      <label>바깥 좌우 여백 (mm)<input id="bmarginx" type="number" min="0" step="0.1" value="52"></label>
      <label>바깥 상하 여백 (mm)<input id="bmarginy" type="number" min="0" step="0.1" value="35"></label>
      <div id="boardSummary"></div>
    </div>
    <div class="row"><button onclick="applyBoardConfig()">보드 설정 적용<span class="cmd">CALIB_K_CONFIG</span></button>
      <button onclick="sendCh('CALIB_K_STATUS')">현재 설정 조회<span class="cmd">CALIB_K_STATUS</span></button>
      <span class="desc">세션 시작 전에 실제 인쇄물과 일치시킵니다.</span></div>
    <div class="row go"><button onclick="send('CALIB_K_BOARD_SAVE')">보드 설정 /mnt에 저장<span class="cmd">CALIB_K_BOARD_SAVE</span></button>
      <span class="desc">지금 적용된 보드 설정을 카메라 /mnt(PERSIST_DIR)에 기록 — 재부팅해도 유지됩니다. "보드 설정 적용"은 RAM에만 반영하므로 저장은 이 버튼으로 별도 실행.</span></div>
      </div>
    </details>
    <div class="row"><button onclick="applyKParams()">세션 조건 적용<span class="cmd">CALIB_K_SET</span></button>
      <label class="kparam">목표 뷰<input id="ktarget" type="number" min="1" value="20"></label>
      <label class="kparam">RMS 한계(px)<input id="krms" type="number" min="0.1" step="0.1" value="0.8"></label>
      <span class="desc">목표 뷰 수·합격 RMS를 재빌드 없이 변경 (CALIB_K_SET).</span></div>

    <div id="kstatus">
      <div id="kcount">0 / 20</div>
      <div id="kinfo">승인된 뷰 — 세션을 시작하면 진행 상황이 표시됩니다</div>
      <div id="kbar"><i></i></div>
    </div>
    <!-- 다른 렌즈에도 세션이 열려 있어 같은 캡처에 반응했을 때만 채워진다.
         비어 있는 것이 정상 — otherLensReport() 참조. -->
    <div id="kother" class="hint"></div>

    <!-- 보드 코너 시각화 (2026-08-11) — 점 위치가 이상하게 찍힌다는 실측 보고로
         일단 꺼둠(display:none). 좌표 공간 불일치 의심 — 조사 후 다시 켤 것.
         마커검출 탭의 raw 오버레이와 같은 원리: 사진 배경 없이(app_config.h의
         의도적 선택 그대로 유지) 좌표만 찍는다. -->
    <div class="qbox" style="display:none">
      <div class="qtitle">보드 코너 — 지금 이 채널에서 잡히는 위치 (상단에서 고른 채널)</div>
      <canvas id="calibProbeCanvas" width="640" height="480" style="max-width:100%;height:auto;display:block;border:1px solid #444;background:#111;border-radius:8px;margin-top:4px"></canvas>
      <div id="calibProbeStatus" class="hint" style="margin-top:4px">세션을 시작하면 잡힌 코너 위치가 여기 표시됩니다.</div>
    </div>

    <div class="row"><label class="toggle"><input type="checkbox" id="gateChk" checked
        onchange="send('CALIB_K_GATE ' + (this.checked ? 1 : 0))"> 품질 게이트 (끄면 품질검사·분산·RMS 통과조건 없이 캡처·계산)</label></div>

    <div class="cmdflow">
    <div class="row"><button onclick="startCalibration()">세션 시작<span class="cmd">CALIB_K_START</span></button>
      <span class="desc">세션 시작(초기화). 먼저 누르세요. <b>다른 렌즈에 열려 있던 세션은 함께 닫습니다</b> — 캡처 한 번이 열린 세션을 전부 무장하기 때문입니다.</span></div>
    <div class="row"><button onclick="stopCalibration(event)">세션 종료<span class="cmd">CALIB_K_STOP</span></button>
      <span class="desc">이 채널의 세션을 닫습니다(승인된 뷰는 버려집니다). 열려만 있고 안 쓰는 세션은 캡처할 때마다 같이 반응해 결과를 헷갈리게 만듭니다. Shift를 누른 채 클릭하면 <b>4채널 전부</b> 종료.</span></div>
    <div class="row"><button id="captureBtn" onclick="captureView()">이 자세 캡처<span class="cmd">CALIB_K_CAPTURE</span></button>
      <span class="desc">품질검사를 통과한 뷰만 증가합니다. 보드를 멈춘 뒤 누르세요.</span></div>
    <div class="row"><button onclick="sendCh('CALIB_K_UNDO')">마지막 뷰 취소<span class="cmd">CALIB_K_UNDO</span></button>
      <span class="desc">방금 캡처한 자세가 잘못됐다고 판단한 경우 제거.</span></div>
    <div class="row go"><button id="computeBtn" onclick="computeCalibration()" disabled>계산하기<span class="cmd">CALIB_K_COMPUTE</span></button>
      <span class="desc">목표 뷰를 모두 통과한 뒤 직접 실행합니다. 자동 계산하지 않습니다.</span></div>
    <div class="row"><button onclick="sendCh('CALIB_K_QUERY')">현재 K값 조회<span class="cmd">CALIB_K_QUERY</span></button>
      <span class="desc">새로 캘리브 안 하고, 지금 카메라에 로드된 K/dist 값을 그대로 조회.</span></div>
    <div class="row go"><button onclick="sendCh('CALIB_K_SAVE')">/mnt에 저장<span class="cmd">CALIB_K_SAVE</span></button>
      <span class="desc">지금 로드된 K/dist를 카메라 /mnt(PERSIST_DIR)에 즉시 기록 — 재부팅해도 유지됩니다. CALIB_K_COMPUTE 성공 시 자동으로도 저장되지만, 쓰기 실패를 확인한 뒤 재시도할 때 이 버튼으로 다시 시도.</span></div>
    </div>
    <div id="kquery" class="qbox"><span class="none">현재 K값 조회를 누르면 카메라에 로드된 값이 표시됩니다</span></div>
  </div>


</div>
<div id="rawPane" style="display:none">
  <div class="group wide">
    <h2>마커 검출 — 픽셀 좌표 (raw / 보정)</h2>
    <p class="sub">인식된 마커의 네 꼭짓점을 실시간으로 확인합니다 (표시 전용).</p>
  </div>

  <div class="group wide">
    <h2>동적 ROI (마커 추적, 상단에서 고른 채널)</h2>
    <p class="sub">마커를 찾으면 그 주변만 검출해 <code>proc</code>을 줄입니다. 놓치면 넓혔다가 전체 재탐색. 렌즈마다 하는 일이 달라(호모그래피용 렌즈는 앵커를 하나도 놓치면 안 됨) 채널별로 따로 켜고 끕니다.</p>
    <div class="row">
      <label class="toggle"><input type="checkbox" id="dynRoiChk" onchange="applyDynRoi()">
        동적 ROI 사용<span class="cmd">DYNROI_CH</span></label>
      <label>최대 margin(px) <input type="number" id="dynMargin" value="240" min="0" max="960" step="10" style="width:6em" onchange="applyDynRoi()"></label>
      <label>실패 허용 <input type="number" id="dynMaxMiss" value="4" min="0" max="60" step="1" style="width:5em" onchange="applyDynRoi()"></label>
      <span id="dynRoiState" class="desc">상태: —</span>
    </div>
    <div class="row">
      <button type="button" onclick="applyDynRoiAllChannels()">이 값 4채널 전부에 적용<span class="cmd">DYNROI</span></button>
      <span class="desc">위 설정을 지금 채널뿐 아니라 4채널 모두에 한 번에 적용(확인창 있음).</span>
    </div>
    <div class="row">
      <label class="toggle"><input type="radio" name="dynRoiScope" id="dynRoiAll" value="all" checked onchange="applyDynRoiIds()">
        모든 마커 추적</label>
      <label class="toggle"><input type="radio" name="dynRoiScope" id="dynRoiPick" value="pick" onchange="applyDynRoiIds()">
        특정 ID만</label>
      <input type="text" id="dynRoiIds" placeholder="예: 49  또는  46,49" autocomplete="off"
             style="max-width:170px" oninput="onDynRoiIdsInput()" onchange="applyDynRoiIds()">
      <button type="button" onclick="applyDynRoiIds()">적용<span class="cmd">DYNROI_IDS</span></button>
      <span id="dynRoiIdsState" class="desc">추적 대상: 모든 마커</span>
    </div>
    <div class="hint" id="dynRoiIdsWarn" style="margin:2px 0 0">
      ⚠️ <b>특정 ID만</b>을 켜면 ROI가 그 마커로 좁혀지므로, <b>범위 밖 마커는 아예 검출되지 않습니다</b>
      — CAM_POSE에서도 사라집니다. 로봇만 빠르게 쫓을 때 쓰고, 바닥 앵커가 보여야 하는 작업
      (호모그래피 계산·검증) 전에는 <b>모든 마커</b>로 되돌리세요.
    </div>
    <details class="fold panel" id="foldDynRoi">
      <summary>어떻게 동작하고, margin은 어떻게 정하나</summary>
      <div class="foldbody hint">
        <b>SEARCH</b>(전체/수동 ROI)에서 마커를 찾으면 <b>TRACK</b>으로 넘어가, 검출된 마커
        전체를 감싸는 사각형 + margin 만큼만 검출한다. 마커 여러 개면 <b>합집합</b>이라
        여러 대도 함께 따라간다.<br>
        놓치면 바로 전체로 안 가고 ROI를 <b>1.5배씩 넓히며</b> 버티다가, <b>실패 허용</b>
        횟수를 넘으면 SEARCH로 돌아간다. 순간 가림·빠른 이동을 싸게 회복하기 위함.<br>
        margin은 마커 네 변의 중앙 길이와 같은 id의 프레임 간 중심 이동량으로 자동 계산한다.
        최초 검출은 마커 한 변의 2배, 추적 중에는 <code>마커크기 + 1.5×이동량 + 10px</code>이며
        입력값은 그 자동 margin의 상한이다. 로그의 <code>margin/marker/move</code>와
        <code>det</code>을 보며 최대값을 조정할 것.<br>
        <b>호모그래피 계산(CALIB_START) 중에는 자동으로 비활성</b>된다 — 등록된 앵커가
        <b>하나도 빠짐없이</b> 화면에 보여야 하기 때문.
      </div>
    </details>
  </div>

  <div class="group wide">
    <h2>검출 파라미터 — 이진화 스캔 횟수 (상단에서 고른 채널)</h2>
    <p class="sub">검출 속도와 강건성의 트레이드오프를 실기에서 비교합니다. 렌즈마다 광각·조명이 달라 따로 맞출 수 있습니다.</p>
    <div class="row">
      <button type="button" onclick="send('ARUCO_SCAN ' + chArg() + ' 3')">3회 (기본)<span class="cmd">ARUCO_SCAN 3</span></button>
      <button type="button" onclick="send('ARUCO_SCAN ' + chArg() + ' 2')">2회<span class="cmd">ARUCO_SCAN 2</span></button>
      <button type="button" onclick="send('ARUCO_SCAN ' + chArg() + ' 1 ' + (document.getElementById('scanWin').value || 13))">1회<span class="cmd">ARUCO_SCAN 1</span></button>
      <label>1회일 때 창 크기 <input type="number" id="scanWin" value="13" min="3" step="2" style="width:5em"></label>
      <button type="button" onclick="sendCh('ARUCO_SCAN')">현재값 조회<span class="cmd">ARUCO_SCAN</span></button>
    </div>
    <details class="fold panel" id="foldScan">
      <summary>무엇을 바꾸고, 무엇을 비교해야 하나</summary>
      <div class="foldbody hint">
        <code>detectMarkers</code>는 창 크기를 바꿔가며 <b>화면 전체를 여러 번 이진화</b>한 뒤
        윤곽을 찾는다. 기본값은 창 3·13·23으로 <b>3번</b> 훑는데, 이 스캔이
        <code>proc</code>의 사실상 100%다. 횟수를 줄이면 그만큼 빨라진다.<br>
        대가는 <b>강건성</b>이다. 창 3이나 23에서만 잡히던 마커는 13만 훑으면
        <b>조용히 놓친다</b> — 에러가 아니라 그냥 <code>MARKER LOST</code>로 보인다.<br>
        <b>비교할 세 가지</b>: (1) <code>det</code> 중앙값(속도) (2) 검출률(전체 프레임 중
        <code>id=</code>가 있는 비율) (3) 마커를 <b>고정</b>해두고 잰 코너 좌표의 흔들림.
        (2)(3)이 유지되면서 (1)만 줄면 성공이다. 화면 중앙과 가장자리에서 각각 볼 것.
      </div>
    </details>
    <h2 style="margin-top:18px">검출 파라미터 — 검출률 레벨</h2>
    <p class="sub">스캔 횟수·ROI가 <b>속도</b> 레버라면, 이건 <b>검출률</b> 레버다.
      오른쪽으로 갈수록 멀/작은·기울어진 마커를 더 잡되 <b>오검출·<code>proc</code>이 늘어난다</b>.
      <b>RAM 전용</b> — 재부팅 시 기본값 복귀. 한 레벨은 perim/ecc/poly 세 값을 한 번에 세팅한다.</p>
    <div class="row" id="detectLevels">
      <button type="button" data-lvl="base" onclick="applyDetectPreset('base')">기본 (지금 그대로)</button>
      <button type="button" data-lvl="cons" onclick="applyDetectPreset('cons')">보수</button>
      <button type="button" data-lvl="mid"  onclick="applyDetectPreset('mid')">중간</button>
      <button type="button" data-lvl="aggr" onclick="applyDetectPreset('aggr')">공격</button>
      <span id="detectLevelState" class="desc">현재 레벨: (미설정 — 기본값으로 동작 중)</span>
    </div>
    <details class="fold panel" id="foldDetectLevel">
      <summary>각 레벨이 뭔지 · 언제 쓰는지 · 실제 바꾸는 값</summary>
      <div class="foldbody hint">
        <p><b>［기본］ 지금 그대로</b><br>
          OpenCV 기본값 그대로. <b>오검출이 가장 적다</b>(엉뚱한 id를 마커로 착각할 확률 최소).
          마커가 크고 정면이고 잘 보이면 이걸로 충분. 여기서 놓치는 마커가 있을 때만 위로 올린다.</p>
        <p><b>［보수］ 안전하게 조금 더</b><br>
          기본에서 <b>아주 살짝</b>만 푼 단계. 오검출 위험은 거의 그대로 두면서,
          <b>약간 작거나 살짝 기운</b> 마커를 조금 더 잡는다. "기본으로 몇 개 놓치는데
          오검출은 절대 늘리기 싫다" 할 때.</p>
        <p><b>［중간］ 균형</b><br>
          검출률과 오검출의 균형점. <b>멀어서 작게 보이거나 30° 안팎으로 기운</b> 마커까지
          꽤 잡는다. 대신 오검출이 아주 가끔 섞일 수 있고 <code>proc</code>(검출 시간)도 늘어난다.
          대부분 현장의 실용 기본값으로 삼을 만한 단계.</p>
        <p><b>［공격］ 최대 검출</b><br>
          가능한 한 다 잡는다. <b>화면 구석의 작고·심하게 기울고·흐린</b> 마커까지 노린다.
          대가가 크다 — <b>오검출(엉뚱한 id)이 눈에 띄게 늘고 <code>proc</code>도 가장 무겁다</b>.
          "일단 검출부터 되는지 보자"는 테스트나, ROI로 영역을 좁혀 비용을 확보한 뒤에만 권장.</p>
        <table style="border-collapse:collapse">
          <tr><th style="text-align:left;padding:2px 14px 2px 0">레벨</th><th style="text-align:left;padding:2px 14px 2px 0">perim</th><th style="text-align:left;padding:2px 14px 2px 0">ecc</th><th style="text-align:left;padding:2px 14px 2px 0">poly</th><th style="text-align:left;padding:2px 14px 2px 0">한 줄 성격</th></tr>
          <tr><td style="padding:2px 14px 2px 0">기본</td><td style="padding:2px 14px 2px 0">0.03</td><td style="padding:2px 14px 2px 0">0.60</td><td style="padding:2px 14px 2px 0">0.03</td><td style="padding:2px 14px 2px 0">오검출 최소</td></tr>
          <tr><td style="padding:2px 14px 2px 0">보수</td><td style="padding:2px 14px 2px 0">0.025</td><td style="padding:2px 14px 2px 0">0.70</td><td style="padding:2px 14px 2px 0">0.035</td><td style="padding:2px 14px 2px 0">안전하게 조금 더</td></tr>
          <tr><td style="padding:2px 14px 2px 0">중간</td><td style="padding:2px 14px 2px 0">0.018</td><td style="padding:2px 14px 2px 0">0.80</td><td style="padding:2px 14px 2px 0">0.045</td><td style="padding:2px 14px 2px 0">균형(실용 기본)</td></tr>
          <tr><td style="padding:2px 14px 2px 0">공격</td><td style="padding:2px 14px 2px 0">0.012</td><td style="padding:2px 14px 2px 0">0.90</td><td style="padding:2px 14px 2px 0">0.06</td><td style="padding:2px 14px 2px 0">최대 검출·비용↑</td></tr>
        </table>
        <b>세 값이 하는 일</b>: <b>perim</b>↓ = 작고 먼 마커, <b>ecc</b>↑ = 블러·기울기의
        비트오류 복구, <b>poly</b>↑ = 원근으로 찌그러진 사각형. (DICT_4X4_50은 쓰는 id가
        적어 ecc를 올려도 여유가 크지만, 너무 올리면 엉뚱한 id로 오검출이 난다.)<br>
        <b>권장 절차</b>: 기본에서 한 단계씩 올리며 검출률(전체 프레임 중 <code>id=</code> 비율)·
        오검출·<code>proc</code> 중앙값을 비교한다. 화면 중앙과 가장자리에서 각각 볼 것.
        먼저 ROI로 탐색 영역을 줄여 <code>proc</code> 예산을 확보한 뒤 레벨을 올리는 게 정석.
        <code>thresh</code>(조명)는 레벨과 무관한 축이라 아래 고급에서 따로 만진다.
      </div>
    </details>

    <details class="fold panel" id="foldDetectAdvanced">
      <summary>고급 — 개별 값 직접 조정 (한 번에 하나만 바꿔 비교)</summary>
      <div class="foldbody">
        <div class="row">
          <label>perim <input type="number" id="dpPerim" value="0.03" min="0.005" max="1" step="0.005" style="width:6em"></label>
          <button type="button" onclick="send('DETECT_PARAM ' + chArg() + ' perim ' + (document.getElementById('dpPerim').value||0.03))">적용<span class="cmd">DETECT_PARAM perim</span></button>
          <span class="desc">작고 먼 마커 → 낮춤 (기본 0.03)</span>
        </div>
        <div class="row">
          <label>ecc <input type="number" id="dpEcc" value="0.6" min="0" max="1" step="0.05" style="width:6em"></label>
          <button type="button" onclick="send('DETECT_PARAM ' + chArg() + ' ecc ' + (document.getElementById('dpEcc').value||0.6))">적용<span class="cmd">DETECT_PARAM ecc</span></button>
          <span class="desc">블러·기울기 → 올림 (기본 0.6)</span>
        </div>
        <div class="row">
          <label>thresh <input type="number" id="dpThresh" value="7" min="-50" max="50" step="1" style="width:6em"></label>
          <button type="button" onclick="send('DETECT_PARAM ' + chArg() + ' thresh ' + (document.getElementById('dpThresh').value||7))">적용<span class="cmd">DETECT_PARAM thresh</span></button>
          <span class="desc">조명 불균일 → 5~10 스윕 (기본 7, 레벨과 무관)</span>
        </div>
        <div class="row">
          <label>poly <input type="number" id="dpPoly" value="0.03" min="0.005" max="0.2" step="0.005" style="width:6em"></label>
          <button type="button" onclick="send('DETECT_PARAM ' + chArg() + ' poly ' + (document.getElementById('dpPoly').value||0.03))">적용<span class="cmd">DETECT_PARAM poly</span></button>
          <span class="desc">원근 사다리꼴 → 올림 (기본 0.03)</span>
        </div>
        <div class="row">
          <button type="button" onclick="sendCh('DETECT_PARAM')">현재값 조회<span class="cmd">DETECT_PARAM</span></button>
          <span class="desc">기본값으로 되돌리려면 「기본」 레벨 또는 카메라 앱 재시작.</span>
        </div>
      </div>
    </details>
  </div>

  <div class="group wide">
    <div class="row"><button id="rawBtn" onclick="toggleRaw()">좌표 보기 시작</button>
      <span class="desc">인식된 마커의 네 꼭짓점 raw 픽셀 좌표를 실시간 표시. 검출/좌표전송엔 영향 없음(표시만).</span></div>
    <div class="row"><label class="toggle"><input type="checkbox" id="undistChk" checked onchange="toggleUndist()">
      보정 좌표 함께 표시 (현재 로드된 K/dist로 undistort)</label>
      <span id="undistState" class="desc"></span></div>
    <details class="fold panel" id="foldRawInfo">
      <summary>raw = 왜곡 보정 전, 보정 = 브라우저에서 undistort — 표시 전용, 카메라에 명령 안 보냄</summary>
      <div class="foldbody hint">
        이미 흐르고 있는 <code>CAM_POSE</code> 로그를 파싱해 표시만 한다 — 카메라에 별도
        명령을 보내지 않으므로 검출·좌표전송에 아무 영향이 없다. 카메라가 실제 전송하는
        건 언제나 raw 픽셀이다(호모그래피 경로도 raw 사용).<br>
        <b>보정 좌표</b>는 <code>CALIB_K_QUERY</code>로 캐시한 현재 K/dist를 써서
        <b>브라우저 JS가 OpenCV와 동일한 반복해</b>(<code>undistortPoints</code>)로 계산한다.
        서버측 undistort가 아니라 <b>표시용 참고값</b>이며, 유효한 K가 없으면
        (Zhang 퇴화 등) 보정값도 신뢰할 수 없다.
      </div>
    </details>
    <div id="rawCorners" class="rawbox">대기 중…</div>

    <h2 style="margin-top:18px">이미지 위 오버레이 (raw / 보정 코너)</h2>
    <p class="sub">raw 코너(주황)와 보정 코너(청록)를 프레임 좌표계에 겹쳐 그립니다. 왜곡 보정이 코너를 어디로 얼마나 미는지 눈으로 확인.</p>
    <div class="row go"><button id="rawOverlayBtn" type="button" onclick="toggleRawOverlay()">오버레이 보기 시작</button>
      <label style="display:inline-flex;align-items:center;gap:6px;margin-left:10px">
        <b>해상도</b>
        <select id="camRes" onchange="onCamResChange()" title="K가 로드되면 K의 (cx×2, cy×2)를 우선 쓰고, K가 없을 때만 여기 값을 씁니다. H_MATRIX 번들의 image_size에도 그대로 쓰입니다.">
          <option value="2592x1520" selected>2592×1520 (ArucoPosePNM 실제 해상도)</option>
          <option value="1920x1080">1920×1080 (FHD)</option>
        </select>
      </label>
      <span class="desc">배경 없이 좌표만 그립니다. 캔버스 크기는 K의 (cx×2, cy×2), K가 없으면 위 해상도로 잡습니다.</span></div>
    <details class="fold panel">
      <summary>주황 = raw(왜곡 전) · 청록 = 보정(undistort) · 점선 = 코너 이동량 · 보라 점선 = 동적 ROI</summary>
      <div class="foldbody hint">
        raw 코너는 카메라가 실제 전송하는 왜곡 픽셀이고, 보정 코너는 위 "보정 좌표 함께 표시"와
        같은 K/dist로 브라우저가 <code>undistortPoints</code> 반복해로 계산한 값이다. 두 점을 잇는
        점선이 왜곡 보정에 따른 코너 이동 방향·크기(px, id 라벨에 Δ로 표기)다. K가 없으면 raw만 그린다.
        <b>보라 점선</b>은 동적 ROI가 켜져 있을 때 <b>카메라가 다음 프레임에 실제로 훑는 범위</b>다
        — 이 사각형 밖의 마커는 아예 검출되지 않는다. 카메라는 전이 때만 상태를 보내므로
        마지막으로 보고된 margin으로 <b>근사</b>해 그린다(실제 검출 ROI와 몇 px 다를 수 있다).
        TRACK 대상이 없으면 대신 <span style="color:#dc3545">SEARCH</span>로 표시된다.<br>
        표시 전용 — 카메라에 아무 명령도 보내지 않는다.
      </div>
    </details>
    <canvas id="rawCanvas" width="640" height="480" style="max-width:100%;height:auto;display:block;border:1px solid #444;background:#111;border-radius:8px;margin-top:4px"></canvas>

    <h2 style="margin-top:18px">지연 — proc (카메라 내부 처리)</h2>
    <p class="sub">시계 동기화 없이도 믿을 수 있는 유일한 지연 값입니다.</p>
    <details class="fold panel" id="foldProcInfo">
      <summary>proc은 정확, net은 NTP 전까지 <code>?clock</code></summary>
      <div class="foldbody hint">
        <code>proc = t − t_frame</code>: 프레임 도착부터 전송 직전까지, <b>둘 다 카메라
        자신의 시계</b>로 찍은 값이라 시계가 서버와 안 맞아도 <b>차이는 정확하다</b>.
        검출 + JSON 생성 시간이며, 수 fps에서는 이게 전체 지연의 대부분이다.<br>
        <code>net = 수신 − t</code>는 <b>서로 다른 시계를 빼는 값</b>이라 NTP 동기화 전에는
        지연이 아니라 시계 차이다 (README §7 미설정). 그래서 값이 비상식적이면
        숫자 대신 <code>?clock</code>으로 표시한다.
      </div>
    </details>
    <div id="rawLatency" class="qbox"><span class="none">CAM_POSE 수신 대기 중…</span></div>
    <details class="fold panel">
      <summary>처리량 네 값을 어떻게 읽나</summary>
      <div class="foldbody hint">
        <b>수신 fps</b>는 카메라가 보고하는 값이 아니라 <b>브라우저에 줄이 도착한 간격</b>으로
        잰 값이다. 카메라 송신 주기에 네트워크 지터가 얹혀 있고, 창(최근 120줄) 전체의
        경과시간으로 나눈다 — 간격의 평균을 쓰면 한 번의 긴 정지가 값을 통째로 끌어내린다.<br>
        <b>det</b>은 <code>detectMarkers</code>만의 비용이다. <code>proc</code>의 거의 전부가
        여기라면 병목은 검출이고, ROI를 좁히거나 스캔 횟수를 줄이는 것이 유일한 수단이다.<br>
        <b>검출률</b>은 마커를 본 프레임의 비율이다. 같은 프레임에 마커가 여러 개면 줄도 여러 개
        오므로 <code>seq</code> 단위로 한 번만 센다.<br>
        <b>카메라 CPU</b>는 카메라가 2초마다 스스로 보내는 값이다(<code>/proc</code> 읽기, 요청 없음).
        <b>앱</b>은 이 <code>.cap</code> 프로세스만, <b>전체</b>는 인코더·SDK 파이프라인까지 포함한
        카메라 전부다. 둘 중 하나만으로는 "우리가 코어를 먹고 있다"와 "카메라가 원래 바쁘다"가
        구별되지 않는다. 멀티코어에서는 <b>앱이 100%를 넘을 수 있다</b>(200% = 두 코어 포화) —
        그래서 코어 수로 나누지 않고 함께 표시한다. 나누면 한 코어가 포화된 상황이 가려진다.<br>
        <b>seq 누락</b>은 번호가 건너뛴 개수다. 0이 아니면 줄이 유실됐다는 뜻이고,
        검출이 느린 것과는 다른 문제다(카메라는 프레임을 셌지만 전송이 밀린 경우).
      </div>
    </details>
  </div>
</div>
<div id="homographyPane" style="display:none">
  <div class="hg-subtabs" role="tablist" aria-label="호모그래피 작업">
    <!-- "계산" -> "floor" (2026-08-12). 이 탭이 만드는 건 바닥 호모그래피(H_floor)이고,
         옆의 Odometry 탭은 로봇 주행으로 H_marker 를 직접 만든다 — 산출물이 다르다는
         걸 이름에서 드러낸다. 내부 섹션 키는 'compute' 그대로 두었다(패널 4개의
         data-hg-section 을 전부 건드리지 않으려고). -->
    <button type="button" class="hg-subtab active" id="hgSubCompute" onclick="showHgSection('compute')">floor</button>
    <button type="button" class="hg-subtab" id="hgSubOdom" onclick="showHgSection('odom')">Odometry</button>
    <button type="button" class="hg-subtab" id="hgSubRegister" onclick="showHgSection('register')">정합</button>
    <!-- "고급" 탭은 2026-08-12 요청으로 뺐다. 패널(data-hg-section="advanced")은
         지우지 않고 그대로 뒀다 — 버튼이 없으면 showHgSection 이 어떤 섹션으로도
         선택하지 않아 계속 숨겨진다. 되살리려면 이 자리에 버튼 한 줄만 다시 넣으면 된다. -->
  </div>

  <details class="group wide fold panel" id="foldHgHealth" data-hg-section="compute" open>
    <summary>현재 호모그래피 상태</summary>
    <div class="foldbody">
      <p class="sub">H를 계산한 뒤 현장 검증을 하고, 사용할 값만 저장하세요.</p>
      <div id="hgHealth" class="qbox"><span class="none">H 상태를 확인하는 중…</span></div>
      <div class="row"><label class="kparam">H 입력 좌표계<select id="hgCoordMode"><option value="raw">raw 픽셀</option><option value="undistort" selected>K/dist 보정 픽셀</option></select></label>
        <button type="button" onclick="applyHgCoordMode()">좌표계 적용<span class="cmd">HG_COORD_MODE</span></button></div>
      <div id="hgCoordStatus" class="hint">기본값은 <b>K/dist 보정 픽셀</b>이며, 호모그래피 탭을 처음 열 때 한 번 자동 적용됩니다. 모드를 바꾸면 기존 H는 다른 좌표계의 값이므로 H를 다시 계산하세요.</div>
    </div>
  </details>

  <details class="group wide fold panel" id="foldHgAnchors" data-hg-section="compute">
    <summary>1. H 만들기 — 앵커 마커 8개 <span class="brief">권장</span></summary>
    <div class="foldbody">
      <p class="sub">등록된 계산 anchor가 <b>모두</b> 보이게 한 뒤 계산합니다. 검증 마커를 등록했다면 계산에 넣지 마세요.</p>
      <div class="tip" style="padding:11px 13px">
        <b>계산 방식</b><br>
        1) 각 프레임에서 등록된 앵커 id의 <b>마커 중심 픽셀 좌표</b>를 찾습니다.<br>
        2) 앵커가 <b>하나도 빠짐없이</b> 보인 프레임 30장을 모아, ID별 픽셀 중심을 평균냅니다. 일부라도 안 보인 프레임은 버립니다.<br>
        3) 평균 픽셀 좌표와 해당 ID에 등록된 <b>실측 바닥 좌표(mm)</b>를 대응시켜, <code>픽셀 → mm</code> 3×3 H를 RANSAC(20 px 임계값)으로 계산합니다.<br>
        4) 최대 300프레임(약 60초) 안에 30장을 못 모으면 실패합니다.<br>
        앵커는 <b>4~16개</b>를 등록할 수 있습니다. 위 프리셋으로 채우거나 표에서 직접 고치세요 — ID는 서로 달라야 합니다.
      </div>
      <div class="qbox"><div class="qtitle">배치 프리셋 — 폼보드 배치를 고르면 표가 채워집니다</div>
        <div class="row">
          <label class="kparam" style="width:auto">배치
            <select id="hgPreset" style="min-width:230px">
              <option value="">— 직접 입력 —</option>
              <option value="1x1" selected>폼보드 1장 (900×600) — 앵커 4</option>
              <option value="2x2">폼보드 2×2 (1800×1200) — 앵커 16</option>
            </select></label>
          <button type="button" onclick="applyHgPreset()">표에 채우기</button>
          <span id="hgPresetStatus" class="desc"></span>
        </div>
        <div class="hint">
          <b>채우기만 하고 전송하지 않습니다.</b> 값을 확인한 뒤 아래 <b>[입력값 모두 적용]</b>을
          눌러야 카메라에 반영됩니다.<br>
          좌표는 실측 기준입니다 — 다이소 폼보드 900×600 mm에 마커(검정 100 / 흰색 포함 120 mm)를
          네 모서리에 딱 맞게 붙이면, 호모그래피가 쓰는 <b>검정 사각형 중심</b>은 판 모서리에서
          <b>흰색/2 = 60 mm</b> 안쪽에 온다. 가운데가 120 mm로 붙는 건 인접한 두 판이 각각
          60 mm씩이라 60+60.<br>
          <b>ID는 역할이 아니라 위치가 정한다</b>: <code>id = 행 × 전체열수 + 열</code>,
          행 0 = 아래, 열 0 = 왼쪽.
        </div>
      </div>

      <div class="qbox"><div class="qtitle">배치도 — 점을 드래그해서 바닥 좌표를 정합니다</div>
        <svg class="hg-map" viewBox="0 0 480 360" role="img"
             aria-label="앵커·검증 기준점 배치도"></svg>
        <div class="hint"><span style="color:#3b82f6;font-weight:600">● 앵커(계산)</span> ·
          <span style="color:#f59e0b;font-weight:600">● 검증 기준점</span> —
          점을 끌어 옮기면 <b>아래 표</b>의 X/Y(mm)가 실시간으로 바뀝니다. 표에 숫자를 직접
          입력해도 배치도에 반영됩니다. <b>세로축은 위가 +Y</b>입니다(월드 좌표계와 동일).
          드래그는 1 mm 단위이며, 옮긴 뒤 <b>[입력값 모두 적용]</b>을 눌러야 카메라에 반영됩니다.</div>
      </div>

      <div class="qbox"><div class="qtitle">앵커 바닥 좌표 편집 (X, Y mm)</div>
        <div id="hgAnchorRows"></div>
        <div class="row"><button type="button" onclick="addHgAnchorRow()">앵커 추가</button>
          <button type="button" onclick="queryHgAnchors()">현재 값 조회<span class="cmd">ANCHOR_QUERY</span></button>
          <button type="button" onclick="applyHgAnchors()">입력값 모두 적용<span class="cmd">ANCHOR_SET_ALL</span></button></div>
        <div class="hint">카메라에 즉시 적용됩니다. 캘리브레이션 중에는 수정할 수 없으며, ID/X/Y 자체는 카메라 재시작 뒤 기본값으로 돌아갑니다. H 계산 결과는 별도의 <b>현재 H 저장</b>으로 보존하세요.</div>
        <div id="hgAnchorEditStatus" class="hint"></div>
      </div>
      <div class="qbox"><div class="qtitle" style="color:#f59e0b">● 배치 계획용 기준점 (배치도 표시 전용, 계산에 넣지 않음, ID·X·Y mm)</div>
        <div id="hgValidationRows"></div>
        <div class="row"><button type="button" onclick="addHgValidationRow()">기준점 추가</button></div>
        <div class="hint">0~16개. ID는 서로 다르고 <b>계산 앵커 ID와 겹칠 수 없습니다</b>. 배치도에서 주황 점을 끌거나 표에 직접 입력하세요 — 입력과 동시에 배치도에 반영되며, 카메라로 전송되지는 않습니다(카메라에는 이런 "검증점" 개념이 없습니다). 카메라 새로고침 뒤 기본값으로 돌아갑니다.</div>
      </div>
      <div class="qbox"><div class="qtitle" style="color:#f59e0b">● 호모그래피 정확도 (LOO 교차검증)</div>
        <div class="row"><button type="button" onclick="queryHgFitAccuracy()">정확도 조회<span class="cmd">HG_QUERY</span></button></div>
        <div class="hint">별도 검증점 없이, 계산에 쓴 앵커마다 그 점 하나만 빼고 나머지로 H를 다시 맞춰(Leave-One-Out) 뺐던 점에서 오차를 잽니다. 각 앵커가 스스로의 정확도를 검증하므로 카메라에 <b>실제로 존재하는</b> 방식입니다.</div>
        <div id="hgValidationEditStatus" class="hint"></div>
      </div>
      <div class="cmdflow">
        <div class="row go"><button id="hgCalibStartBtn" onclick="sendCh('CALIB_START')">앵커로 H 계산<span class="cmd">CALIB_START</span></button></div>
        <div class="row"><button type="button" onclick="clearHomography()">H 지우기<span class="cmd">HG_CLEAR</span></button>
          <span class="desc">이 채널의 H를 RAM과 <code>/mnt</code>에서 지웁니다. <b>등록한 앵커는 그대로 남습니다</b>(줄자로 잰 입력이라 H의 산물이 아님).
            좌표계(raw ↔ 보정)를 바꾸려면 카메라가 H가 없는 상태를 요구하므로, <b>H 지우기 → 좌표계 적용 → 앵커로 H 계산</b> 순서로 하세요.</span></div>
      </div>
      <div id="hgStatus" class="qbox"><span class="none">캘리브 시작으로 등록 앵커(최소 4점)로 호모그래피를 계산하세요</span></div>
    </div>
  </details>

<details class="group wide fold panel" id="foldHgMarkerPlane" data-hg-section="compute">
    <summary>로봇 마커 평면 (시차 보정)</summary>
    <div class="foldbody">
      <p class="sub">호모그래피는 <b>평면 하나</b>만 다룹니다. 바닥 앵커로 구한 H에 차체 위쪽 마커를
        투영하면, 카메라 바로 아래 지점(나디르)에서 <b>멀어지는 방향으로 밀린</b> 위치가 나옵니다.
        이 오차는 <b>계통 오차</b>라 평균을 내도 사라지지 않습니다.</p>
      <div class="hint">
        밀리는 양은 대략 <code>마커높이 ÷ 카메라높이</code> 비율입니다 — 카메라 1.5&nbsp;m,
        마커 250&nbsp;mm면 나디르까지 거리의 1/6입니다.<br>
        ⚠️ <b>undistort 모드에서만 계산됩니다.</b> raw 픽셀로 피팅한 H를 분해하면 렌즈 왜곡이
        회전·평행이동에 흡수돼, 보정에 쓰는 평면 법선 자체가 틀어집니다. 틀린 값이 그럴듯하게
        나오므로 카메라가 아예 거부합니다.
      </div>
      <div class="row">
        <label class="kparam">마커 높이(mm)<input id="mpHeight" type="number" min="0" step="1" value="0" style="width:90px"></label>
        <button type="button" onclick="applyMarkerHeight()">적용<span class="cmd">MARKER_HEIGHT</span></button>
      </div>
      <div class="row">
        <label class="kparam">카메라 높이 실측(mm)<input id="mpCamHeight" type="number" min="0" step="1" value="0" style="width:100px"></label>
        <button type="button" onclick="applyCameraHeight()">적용<span class="cmd">CAMERA_HEIGHT</span></button>
        <span class="desc"><b>0 = 미측정</b> (H 분해 역산값 사용). 줄자로 쟀으면 넣으세요.</span>
      </div>
      <div class="hint">
        보정량은 <code>마커높이 ÷ 카메라높이</code> 비율입니다. 마커 높이는 실측해서 넣지만
        카메라 높이는 <b>H를 분해해 역산</b>하는데, 이 분해가 핀홀을 가정하므로 남은 왜곡이
        스케일을 편향시킵니다. <b>Cz가 틀리면 보정량이 그 비율만큼 통째로 틀립니다</b> —
        예: 역산이 20% 크면 보정이 17% 부족해집니다. 실측값을 넣으면 이 항이 사라집니다.
      </div>
      <div class="row">
        <button type="button" onclick="sendCh('MARKER_PLANE_QUERY')">조회<span class="cmd">MARKER_PLANE_QUERY</span></button>
        <button type="button" onclick="sendCh('MARKER_PLANE_SAVE')">저장<span class="cmd">MARKER_PLANE_SAVE</span></button>
      </div>
      <div class="hint">적용은 RAM에만 반영됩니다 — 재부팅 뒤에도 유지하려면 <b>저장</b>을 누르세요 (두 높이 모두 함께 저장).</div>
      <div id="mpStatus" class="qbox"><span class="none">마커 높이를 입력하고 적용하세요 (H 계산 완료 후)</span></div>
      <div class="hint"><b>카메라 높이는 줄자로 검산하세요.</b> 1.5&nbsp;m에 달아둔 카메라가 4&nbsp;m로
        역산되면 K나 H가 틀린 것이고, 시차 보정값도 같이 틀립니다.</div>
    </div>
  </details>

    <details class="group wide fold panel" id="foldHgSave" data-hg-section="compute">
    <summary>3. 확정 및 저장</summary>
    <div class="foldbody">
      <p class="sub">검증 결과가 만족스러울 때만 현재 적용 H를 저장하세요. 저장 전에도 H는 즉시 좌표 변환에 사용됩니다.</p>
      <div class="row go"><button onclick="sendCh('HG_SAVE')">현재 H 저장<span class="cmd">HG_SAVE</span></button>
        <span class="desc">카메라 /mnt(PERSIST_DIR)에 기록합니다. 저장해야 재부팅 뒤에도 유지됩니다.</span></div>
    </div>
  </details>

  <details class="group wide fold panel" id="foldHgSendFloor" data-hg-section="compute">
    <summary>4. 서버 송신 — H_MATRIX</summary>
    <div class="foldbody">
      <div class="hint" style="margin-bottom:8px">
        ℹ️ <b>앵커 캘리가 성공하면 카메라가 <code>H_MATRIX</code>를 자동으로 보냅니다</b> —
        평소에는 누를 일이 없습니다. 그 번들은 <code>H_floor</code>=<b>앵커로 직접 측정한 값</b>,
        <code>H_marker</code>=<b>마커 높이만큼 파생시킨 값</b>,
        <code>method</code>=<code>static_anchors</code> 입니다(주행 캘리와 정확히 반대 방향).<br>
        이 버튼은 <b>그때 링크가 끊겼거나 서버가 저장분을 잃었을 때 쓰는 재전송</b>입니다.
        브라우저가 JSON 을 조립하지 않고 <b>카메라가 자기 상태에서 다시 조립</b>하므로,
        자동 전송과 <b>정의상 같은 번들</b>이 나갑니다.
      </div>
      <div class="row go">
        <button type="button" onclick="floorResendBundle()">바닥 H 재전송<span class="cmd">FLOOR_RESEND</span></button>
      </div>
      <div id="hgSendFloorNote" class="hint">
        앵커 캘리를 아직 완주하지 않은 채널이면 카메라가 거부하고 사유를 알려줍니다
        (바닥 H 없음 / K·dist 없음 / 프레임 크기 미수신 / 링크 끊김).</div>
    </div>
  </details>

  <details class="group wide fold panel" id="foldIvaSync" data-hg-section="compute">
    <summary>5. WiseAI IVA 영역 반영</summary>
    <div class="foldbody">
      <p class="sub">등록된 앵커들의 raw 픽셀 convex hull을 이 채널의 WiseAI IVA 영역으로 밀어넣습니다.
        앵커 hull이 곧 "호모그래피가 실제로 캘리브레이션된 영역"이라, 그 바깥의 검출은 애초에
        정확한 월드 좌표가 없습니다.</p>
      <div class="row">
        <button type="button" onclick="sendCh('IVA_SYNC')">다각형 계산<span class="cmd">IVA_SYNC</span></button>
      </div>
      <div id="ivaSyncStatus" class="qbox"><span class="none">다각형 계산을 누르세요</span></div>
      <div class="row" style="margin-top:8px">
        <button type="button" id="zoneBandsBtn" onclick="toggleZoneBands()">완충 밴드 켜기<span class="cmd">ZONE_BANDS</span></button>
        <label class="kparam">접근금지
          <input id="zoneDangerMm" type="number" min="0" max="20000" step="50" value="300" style="width:6em">mm
        </label>
        <label class="kparam">주의
          <input id="zoneWarnMm" type="number" min="0" max="20000" step="50" value="500" style="width:6em">mm
        </label>
        <button type="button" onclick="sendZoneMargin()">적용<span class="cmd">ZONE_MARGIN</span></button>
      </div>
      <div class="hint">
        존 경계 바깥으로 <b style="color:#ef4444">접근금지</b>(빨강) ·
        <b style="color:#eab308">주의</b>(노랑) 띠를 바닥 평면에 그립니다. 거리는 화면
        픽셀이 아니라 <b>실제 mm</b>라, 사람이 카메라에서 멀든 가깝든 같은 뜻입니다
        (화면에서는 가까운 쪽이 넓게 보이는 게 정상 — 그게 올바른 원근입니다).
        <b>이 채널에 호모그래피가 있어야 동작합니다</b> — 없으면 밴드는 안 그려지고
        발끝 판정도 예전 픽셀 방식으로 떨어집니다.
      </div>
      <div class="row go">
        <button type="button" id="ivaPushBtn" onclick="pushIvaArea()" disabled>WiseAI에 반영<span class="cmd">PUT /configuration/ivaarea</span></button>
        <span class="desc">카메라 관리자 계정 Digest 인증으로 이 서버가 직접 PUT합니다
          (이 서버 프로세스의 환경변수 CAMERA_USER/CAMERA_PASS 필요). 계산 결과를 검토한 뒤에만 누르세요.</span>
      </div>
      <div id="ivaPushStatus" class="qbox"><span class="none">아직 반영하지 않음</span></div>
    </div>
  </details>

  <details class="group wide fold panel" id="foldIvaEvent" data-hg-section="compute">
    <summary>6. IVA 이벤트 로그</summary>
    <div class="foldbody">
      <p class="sub">카메라가 WiseAI IVA 영역 진입/이탈을 감지하면 여기 실시간으로 쌓입니다.
        와이어 형식은 2026-08-19 실측으로 확인됨 — 카메라 쪽
        <code>ParseWiseAiIvaAreaEvents()</code>가 파싱해 <code>pose_sender_send_control_line()</code>로
        (재시도 큐 있는 채널) 보내는 것을 그대로 받는다.</p>
      <div class="hint"><b>아직 소리/릴레이 알람은 안 붙어있다</b> — 그 방식(스피커 vs 릴레이)이
        아직 안 정해져서, 지금은 파이프라인이 눈에 보이는 것까지만 한다. 여기 로그에 뜨는 걸
        확인한 뒤에 다음 단계로.</div>
      <div id="ivaEventLog" class="qbox"><span class="none">아직 이벤트 없음</span></div>
    </div>
  </details>

  <div class="group wide" data-hg-section="odom">
    <h2>Odometry — 로봇 주행 호모그래피</h2>
    <p class="sub">로봇이 알려진 크기의 사각형을 돌며 정지점마다 CCTV가 마커 픽셀을 캡처하고,
       그 대응점으로 <code>H_marker</code>를 직접 구하는 방식입니다.</p>
    <div class="hint">
      wire 규격(확정본): 파이 <code>~/Road_Painter_4th/Server/docs/CALIBRATION.md</code>
    </div>
  </div>

  <div class="group wide" data-hg-section="odom">
    <h2>1. 주행 배치도 — 사각형 크기와 출발 코너를 정합니다</h2>
    <div class="row">
      <label class="kparam">가로 m<input id="odoM" type="number" min="20" max="1000" step="1"
             value="90" oninput="renderOdoLayout()"></label> cm
      <label class="kparam">세로 n<input id="odoN" type="number" min="20" max="1000" step="1"
             value="60" oninput="renderOdoLayout()"></label> cm
      <label class="kparam">출발 코너<select id="odoCorner" onchange="renderOdoLayout()">
        <option value="bottom_left">bottom_left — 좌회전(CCW)</option>
        <option value="top_left">top_left — 우회전(CW)</option>
      </select></label>
    </div>
    <div class="odo-cols">
      <div class="qbox"><div class="qtitle">배치도 — 오른쪽 위 ◇ 핸들을 끌면 사각형 크기가 바뀝니다</div>
        <svg id="odoMap" class="odo-map" viewBox="0 0 480 360"
             aria-label="로봇 주행 정지점 배치도"></svg>
        <div class="hint">
          <b>세로축은 위가 +Y</b>입니다(월드 좌표계와 동일). 드래그는 1 cm 단위이고, 값을 위 칸에
          직접 입력해도 같습니다.<br>
          점 안의 숫자는 <code>point_index</code>(캡처 순번)입니다. <b>◎ 8번은 출발점으로 돌아온
          복귀 지점</b>이라 라벨이 0번과 같습니다 — 피팅에서 빼고 폐합오차 측정에만 씁니다.
          ↻ 표시는 로봇이 제자리 90° 회전하는 곳으로, 캡처하지 않고 지나갑니다.<br>
          정지점 9개는 <b>m·n 과 출발 코너에서 계산됩니다</b>. 로봇 경로가 11-op 고정
          (MOVE·MOVE·TURN ×3 + MOVE·MOVE)이라 점 하나만 따로 옮길 수 없기 때문입니다 —
          옮기려면 사각형 자체를 바꾸세요.
        </div>
      </div>
      <div class="qbox"><div class="qtitle">진행도 — 정지점별 좌표와 캡처 결과</div>
        <div id="odoSession" class="hint" style="margin:0 0 6px">세션 없음 — 아래는 계획 좌표입니다.</div>
        <div id="odoPoints"></div>
        <div id="odoSummary" class="hint" style="margin-top:6px"></div>
        <div class="hint">
          <b>스스로 갱신됩니다</b> — 이 대시보드는 서버에 role=ADMIN 으로 붙어 있고, 서버가
          오가는 모든 메시지의 사본(TAP)을 흘려주므로 카메라가 올린
          <code>CALIB_CAPTURE_OK/FAIL</code> 이 그대로 들어옵니다. 따로 조회할 필요가 없습니다.<br>
          <code>spread</code> 는 카메라가 정지 판정에 쓴 픽셀 표준편차입니다 — 작을수록 잘 멈춘
          것이고, 큰 값이 성공으로 통과했다면 임계값(2px)을 의심할 자리입니다.<br>
          원점 (0,0) 은 <b>로봇의 출발 위치</b>입니다. 폼보드 좌하단이 기준인 floor 탭과 달리
          <b>세션마다 원점이 바뀝니다</b>(수용된 결정, wire 규격 §6).
        </div>
      </div>
    </div>
    <div class="qbox"><div class="qtitle">세션 개시 — <code>CALIB_START</code> (관리자창 → Server)</div>
      <div class="row go">
        <button type="button" onclick="odoStartSession()">주행 캘리 시작<span class="cmd">CALIB_START</span></button>
        <button type="button" onclick="odoCancelSession()">중단<span class="cmd">CALIB_CANCEL</span></button>
      </div>
      <div id="odoSendNote" class="hint" style="margin-top:4px"></div>
      <div class="row" style="margin-top:10px">
        <b>로봇 측위에 쓸 H_marker:</b>
        <button type="button" onclick="odoSetPrefer(0)">파생 (체커보드)<span class="cmd">ODOM_PREFER 0</span></button>
        <button type="button" onclick="odoSetPrefer(1)">측정 (주행)<span class="cmd">ODOM_PREFER 1</span></button>
        <button type="button" onclick="sendCh('ODOM_PREFER_QUERY')">조회<span class="cmd">ODOM_PREFER_QUERY</span></button>
      </div>
      <div id="odoPreferNote" class="hint" style="margin-top:2px">조회를 누르면 현재 설정이 표시됩니다.</div>
      <div class="hint">
        주행 캘리를 성공해도 <b>기본은 파생값(체커보드)</b>입니다. 주행 방식은 균일 스케일
        오차를 스스로 검증하지 못하기 때문입니다 — 네 변이 같은 비율로 짧으면 궤적도 정확히
        닫히므로 <b>폐합오차도 그건 못 잡습니다.</b> 전환 전에 위 요약의
        <code>체커보드 대비 scale</code>이 1.000 근처인지 보세요.
      </div>
      <pre id="odoStartPreview" style="margin:6px 0 0;font-size:12px;overflow-x:auto"></pre>
      <div class="hint">
        ⚠️ <b>[시작]을 누르면 로봇이 실제로 주행합니다.</b> 서버가 11-op 경로(PATH)를 만들어
        로봇에 내리고, 정지점마다 카메라에 <code>CALIB_CAPTURE</code>를 물립니다. 누르기 전에
        사각형 안에 사람·장애물이 없는지 확인하세요.<br>
        [중단]은 서버가 로봇과 카메라 양쪽에 <code>CALIB_CANCEL</code>을 보내고,
        <b>양쪽의 <code>CALIB_STOPPED</code>가 다 모여야</b> 세션을 닫습니다(wire 규격 §9).
        5초 안에 안 모이면 <code>cancel_failed</code>로 남습니다 — 그때는 로봇이 실제로
        섰는지 눈으로 확인하세요. Qt는 이 세션에 관여하지 않습니다.
      </div>
    </div>
  </div>

  <details class="group wide fold panel" id="foldHgSendOdom" data-hg-section="odom">
    <summary>서버 송신 — H_MATRIX</summary>
    <div class="foldbody">
      <div class="hint" style="margin-bottom:8px">
        ℹ️ <b>주행 캘리가 성공하면 카메라가 <code>H_MATRIX</code>를 자동으로 보냅니다</b> —
        평소에는 누를 일이 없습니다. 그 번들은 <code>H_marker</code>=<b>주행으로 직접 측정한 값</b>,
        <code>H_floor</code>=<b>그 <code>H_marker</code>를 역산한 값</b>,
        <code>method</code>=<code>robot_motion</code> 입니다.<br>
        이 버튼은 <b>그때 링크가 끊겼거나 서버가 저장분을 잃었을 때 쓰는 재전송</b>입니다.
        브라우저가 JSON 을 조립하지 않고 <b>카메라가 자기 상태에서 다시 조립</b>하므로,
        자동 전송과 <b>정의상 같은 번들</b>이 나갑니다.
      </div>
      <div class="row go">
        <button type="button" onclick="odoResendBundle()">측정 H 재전송<span class="cmd">ODOM_RESEND</span></button>
      </div>
      <div id="hgSendOdomNote" class="hint">
        주행 캘리를 아직 완주하지 않은 채널이면 카메라가 거부하고 사유를 알려줍니다
        (측정 <code>H_marker</code> 없음 / K·dist 없음 / 프레임 크기 미수신 / 링크 끊김).</div>
    </div>
  </details>

  <div class="group wide" data-hg-section="register">
    <h2>정합(Registration) — 채널 간 좌표계 통일</h2>
    <p class="sub">두 채널을 각각 오도메트리로 캘리한 뒤, 겹치는 FOV 구역에서
       로봇을 <b>조이스틱으로 직접 몰면서</b> 두 채널이 동시에 잡은 위치를 짝지어
       ch_b의 좌표를 ch_a 기준 좌표계로 옮기는 닮음변환을 구합니다.</p>
    <div class="hint">
      🔴 <b>자동 경로가 없습니다</b> — 두 렌즈의 FOV가 어디서 겹치는지 아직 실측되지
      않아, 오도메트리처럼 서버가 사각형 경로를 짤 근거가 없습니다. [시작]을 누른 뒤
      <b>로봇 탭(조이스틱)으로 로봇을 겹침 구역에 직접 몰 것</b> — 그동안 서버가
      <code>REGISTER_CAPTURE</code>를 주기적으로 반복 전송하고, 로봇이 실제로 두
      채널 시야 안에 있을 때만 카메라가 성공으로 답합니다.<br>
      wire 규격: <code>~/Road_Painter_4th/Server/docs/CALIBRATION.md</code>
    </div>
    <div class="hint">
      ⚠️ 정합은 두 채널 모두 <b>오도메트리를 먼저 완주</b>하고, <b>Odometry 탭에서
      양쪽 다 [측정 (주행)]<span class="cmd">ODOM_PREFER 1</span>을 켜둔 상태</b>여야
      의미가 있습니다. 꺼져 있으면 카메라가 거부하거나(체커보드 H도 없는 채널),
      더 위험하게는 <b>체커보드 좌표 기준으로 조용히 정합이 계산</b>됩니다 —
      정합이 풀려는 문제(오도메트리 원점이 채널마다 다르다)와 무관한 값입니다.
    </div>
    <div class="row">
      <label class="kparam">기준 채널 ch_a<select id="registerChA"></select></label>
      <label class="kparam">대상 채널 ch_b<select id="registerChB"></select></label>
    </div>
    <div class="row go" style="margin-top:8px">
      <button type="button" onclick="registerStartCollect()">수집 시작<span class="cmd">REGISTER_COLLECT_START</span></button>
      <button type="button" onclick="registerStopCollect()">완료<span class="cmd">REGISTER_COLLECT_STOP</span></button>
      <button type="button" onclick="registerCancelCollect()">취소<span class="cmd">REGISTER_COLLECT_CANCEL</span></button>
    </div>
    <div id="registerSendNote" class="hint" style="margin-top:4px"></div>
    <div class="qbox" style="margin-top:10px">
      <div class="qtitle">진행도</div>
      <div id="registerSession" class="hint" style="margin:0 0 6px">세션 없음.</div>
      <div id="registerSummary" class="hint"></div>
      <div class="hint">
        <b>스스로 갱신됩니다</b> — Odometry 탭과 같은 방식으로 서버의 TAP을 통해
        <code>REGISTER_CAPTURE_OK/FAIL</code>이 그대로 들어옵니다.<br>
        <code>not_both_seen</code>은 로봇이 아직 겹침 구역 밖이라는 뜻으로, 이
        방식에서는 정상적으로 계속 나는 실패입니다 — 겹침 구역에 들어가면 자연히
        성공이 섞이기 시작합니다.
      </div>
    </div>
  </div>

  <div class="group wide" data-hg-section="advanced">
    <h2>고급 분석 — 검출 기록, PC 결과 적용, H 행렬</h2>
    <div>
      <h2>검출 마커 기록 (Pi → PC)</h2>
      <p class="sub">기록 중 카메라에 검출되는 모든 마커의 ID, raw 픽셀 중심·코너와 프레임 정보를 저장합니다. H가 적용돼 있으면 해당 월드 좌표도 함께 저장합니다.</p>
      <div id="hgExperimentRows" class="qbox"></div>
      <div class="cmdflow"><div class="row go"><button type="button" onclick="hgExperimentStart()">기록 시작</button><button type="button" onclick="hgExperimentStop()">기록 종료 · JSON/CSV 생성</button><a id="hgExperimentDownload" style="display:none" href="/hg_experiment/export">JSON/CSV 내려받기</a></div></div>
      <div class="hint">이 기록은 마커 ID나 개수를 제한하지 않습니다. 새 H를 PC에서 피팅하려면 선택한 마커의 실제 바닥 좌표(mm)는 PC 분석 단계에서 별도로 붙여야 합니다.</div>
      <div id="hgExperimentStatus" class="qbox"><span class="none">기록 시작을 누른 뒤 원하는 마커를 비추세요</span></div>
      <h2 style="margin-top:18px">PC 분석 결과 적용</h2>
      <div class="tip" style="padding:11px 13px">
        <b>PC 분석기가 아래 형식의 JSON을 만들면 그대로 붙여넣으세요.</b><br>
        <code>{"H":[h00,h01,h02,h10,h11,h12,h20,h21,h22],"source_ids":[10,11,...],"rmse_mm":12.3,"max_error_mm":28.7}</code><br>
        <b>필수:</b> <code>H</code> 9개 — raw 픽셀 좌표를 월드 mm로 바꾸는 3×3 행렬(행 우선)입니다.<br>
        <b>선택:</b> <code>source_ids</code>(분석에 쓴 마커 ID), <code>rmse_mm</code>, <code>max_error_mm</code>(PC가 계산한 오차 지표).<br>
        적용하면 H가 카메라에 즉시 반영되지만 영구 저장되지는 않습니다. 검증 뒤 <b>계산 탭 → 현재 H 저장</b>을 누르세요.
      </div>
      <textarea id="hgExperimentResult" rows="7" style="width:100%;box-sizing:border-box" placeholder='{"source_ids":[...],"H":[h00,...,h22],"rmse_mm":...,"max_error_mm":...}'></textarea>
      <div class="row go"><button type="button" onclick="hgExperimentApply()">이 결과의 H 적용<span class="cmd">HG_SET</span></button></div>
      <div id="hgExperimentResultStatus" class="qbox"><span class="none">PC 분석 결과 대기 중</span></div>
      <div class="row" style="margin-top:18px"><button onclick="sendCh('HG_QUERY')">H 행렬 조회<span class="cmd">HG_QUERY</span></button></div>
      <div id="hgMatrix" class="qbox"><span class="none">H 행렬을 조회하면 여기에 표시됩니다</span></div>
    </div>
  </div>
</div>
<div id="centralPane" style="display:none">
  <div class="group wide">
    <h2>카메라 → 중앙 서버 (TLS 9000, role=CCTV)</h2>
    <p class="sub">
      카메라가 중앙 서버에 직접 붙어 보내는 채널입니다. 이 대시보드(7000)와는 별개이며,
      서버 <code>PROTOCOL.md</code>의 <code>POS</code>/<code>H_MATRIX</code> 규격을 씁니다.
    </p>
    <div id="centralStatus" class="qbox"><span class="none">상태를 조회하면 여기에 표시됩니다</span></div>
    <div class="row">
      <button onclick="send('CENTRAL_QUERY')">상태 조회<span class="cmd">CENTRAL_QUERY</span></button>
      <button id="ctLinkBtn" onclick="toggleCentralLink()">연결 —<span class="cmd">CENTRAL_LINK</span></button>
      <button id="ctPosBtn" onclick="toggleCentralPos()">POS 전송 —<span class="cmd">CENTRAL_POS</span></button>
    </div>
    <p class="sub">
      <b>연결</b>을 끄면 TLS 세션이 끊겨 서버의 CCTV 접속 목록에서 사라집니다(재접속 시험용).
      <b>POS 전송</b>만 끄면 접속은 유지한 채 좌표만 멈춥니다 — 서버 입장에선
      "카메라는 붙어 있는데 로봇이 안 보이는" 상태가 됩니다.
    </p>

    <h2 style="margin-top:22px">POS 대상 마커</h2>
    <p class="sub">
      서버의 <code>POS</code> 스키마에는 마커 id 필드가 없어서, 보내는 좌표가 곧 로봇으로
      취급됩니다. 앵커·검증 마커가 섞이지 않도록 카메라가 <b>여기서 지정한 id 하나만</b>
      9000으로 보냅니다.
    </p>
    <div class="row">
      <input id="ctId" type="number" style="max-width:120px" placeholder="예: 15" autocomplete="off">
      <button onclick="sendCentralId()">적용<span class="cmd">CENTRAL_ID</span></button>
    </div>

    <h2 style="margin-top:22px">H_MATRIX 전송</h2>
    <p class="sub">
      서버는 <code>calib</code> 번들이 있어야 POS 픽셀을 미터로 바꿉니다. 아래 payload가
      그대로 <code>{"type":"H_MATRIX","seq":n,"payload":…}</code>에 실려 나갑니다.
      <b>H는 mm 기준</b>(pixel→world mm)으로 보내면 서버가 ÷1000로 미터 정규화합니다.
    </p>
    <details class="fold warn">
      <summary><b>⚠ 값 검증 없이 그대로 전송됩니다</b></summary>
      <div class="foldbody">
        카메라는 JSON 객체인지만 확인하고 내용은 손대지 않습니다. 서버는 이 번들을
        로그인 사용자에 <b>영속 저장</b>하고 QT에 중계하므로, 시험값을 보내면 그 값이
        저장된 캘리브레이션이 됩니다.
      </div>
    </details>
    <div class="row">
      <textarea id="ctHm" spellcheck="false"
                style="width:100%;min-height:190px;font-family:monospace;font-size:12px"></textarea>
    </div>
    <div class="row">
      <label class="kparam" style="width:auto">calib_id
        <input id="ctCalibId" type="text" spellcheck="false" autocomplete="off" style="width:150px"></label>
      <label class="kparam" style="width:auto">canvas_mm (W×H)
        <input id="ctCanvasW" type="number" min="1" step="10" style="width:78px">
        <input id="ctCanvasH" type="number" min="1" step="10" style="width:78px"></label>
    </div>
    <div class="row">
      <button onclick="sendCentralHmatrix()">전송<span class="cmd">CENTRAL_HMATRIX</span></button>
      <button onclick="fillCentralHmatrixTemplate()">QT-REQ-CCTV-001 형식으로 채우기</button>
      <button onclick="fillCentralHmatrixLegacy()">구 형식(H_floor/H_marker)</button>
    </div>
    <div id="ctHmNote" class="hint">
      카메라가 이번 세션에 보고한 K·dist·H·coord_mode를 그대로 채웁니다.
      빠진 항목이 있으면 자리표시자가 들어가므로 <b>여기 경고가 뜨면 전송하지 마세요</b>.
    </div>
    <details class="fold panel">
      <summary>두 형식의 차이 — 무엇을 언제 쓰나</summary>
      <div class="foldbody hint">
        <b>QT-REQ-CCTV-001 형식</b>(요청서 rev.2)은 최상위 평면 스키마다. 요청서 본문에는
        <code>H</code> 하나뿐이지만, 여기서는 <b><code>H_marker</code>를 함께 싣는다</b> —
        요청서가 필수 필드를 정할 뿐 추가 필드를 금지하지는 않고, 서버의 로봇 제어 경로는
        시차 보정에 그 값이 필요하기 때문이다. QT는 <code>H</code>만 읽으면 되고
        <code>H_marker</code>는 무시하면 된다.<br>
        <b>구 형식</b>은 <code>{"calib":{…}}</code> <b>중첩</b>에 <code>H_floor</code>·
        <code>H_marker</code>·<code>marker_height_mm</code>을 싣는다. 담기는 행렬은 사실상 같고
        <b>차이는 키 이름과 중첩 구조뿐</b>이다. <b>서버는 2026-07-30부터 두 형식을 동등하게
        읽는다</b> — 평면 형식의 <code>H</code>를 <code>H_floor</code>로 받고 <code>K</code>·
        <code>D</code>·<code>H_marker</code>까지 그대로 쓰므로 왜곡·시차 보정 결과가 같다.
        어느 쪽을 보내도 되고, 설치 메타데이터(<code>calib_id</code>·<code>canvas_mm</code> 등)가
        같이 저장·중계되는 쪽은 평면 형식이다.<br>
        (그 전에는 평면 형식을 보내면 서버가 레거시로 오인해 <code>H</code>만 남기고
        <code>K</code>·<code>D</code>·<code>H_marker</code>를 버렸다 — 에러 없이 보정만 꺼졌다.
        서버 로그의 <code>캘리브레이션 수신 (평면 번들 …)</code> 표기로 확인할 수 있다.)<br>
        두 형식 모두 <code>H</code>/<code>H_marker</code>는 <b>같은 픽셀 공간</b>(<code>coord_mode</code>)을
        입력으로 받는다. 짝이 안 맞는 픽셀을 넣으면 에러 없이 조용히 틀린 좌표가 나온다.
      </div>
    </details>
  </div>

  <div class="group wide">
    <h2>POS 전송 규격 — 카메라가 실제로 내보내는 줄</h2>
    <p class="sub">
      POS는 H_MATRIX처럼 사람이 편집해 보내는 값이 아니라, <b>검출될 때마다 카메라가 자동으로</b>
      보내는 스트림입니다. 여기서는 그 형식을 확인하고 현재 프레임 값으로 채워볼 수만 있습니다.
    </p>
    <div class="row">
      <textarea id="ctPos" spellcheck="false" readonly
                style="width:100%;min-height:110px;font-family:monospace;font-size:12px"></textarea>
    </div>
    <div class="row">
      <button onclick="fillCentralPosSample()">현재 프레임으로 채우기</button>
      <button onclick="copyCentralPos()">복사</button>
      <span id="ctPosNote" class="desc"></span>
    </div>
    <details class="fold panel">
      <summary>이 좌표가 서버에서 무엇이 되나 — 왜 여기엔 캘리브레이션이 없나</summary>
      <div class="foldbody hint">
        <code>corners</code>는 <b>보정 전 raw 픽셀</b>이고 순서는 카메라가 검출한 그대로다.
        마커 <b>하나</b>(중앙 대상 id)만 실리며, 놓친 프레임(하트비트)은 <b>보내지 않는다</b>.<br>
        서버는 이 픽셀을 받아 <code>undistort → H_marker</code>로 바닥 미터 좌표를 만들고,
        QT에는 <code>POSE</code>(x, y, theta_deg)로 바꿔 중계한다. 원본 픽셀은 QT로 가지 않는다 —
        캘리브레이션을 가진 쪽이 서버뿐이라 QT는 해석할 방법이 없기 때문이다.<br>
        그래서 <b>POS에는 K/H가 실리지 않는다</b>. 그 값들은 H_MATRIX로 <b>따로 한 번</b> 올려두고,
        서버가 저장해 두었다가 매 POS에 적용한다. H_MATRIX를 안 보냈거나 옛 값이면
        POS는 정상인데 위치만 조용히 틀린다 — 서버 로그의
        <code>캘리브레이션 없음 - pose 계산 불가</code>가 그 신호다.<br>
        <code>seq</code>는 카메라가 POS마다 1씩 올리는 값이며 CAM_POSE의 seq와는 별개다.<br>
        <b>채널 규약 주의</b> — <code>payload.ch</code>는 <b>1-based</b>(카메라가 내부
        index에 +1을 해서 싣는다, <code>sample_component.cc</code>). 반면 같은 pose
        링크로 흐르는 <b>CAM_POSE의 <code>ch</code>는 0-based</b>(내부 index 그대로)다.
        그래서 로그에서 <code>seq=104 ch=2 …</code>(CAM_POSE)와
        <code>POS {"ch": 3 …}</code>는 <b>같은 렌즈 CH3</b>를 가리킨다 — 어긋난 게 아니다.
      </div>
    </details>
  </div>
</div>
<div id="opsPane" style="display:none">
  <div class="group wide">
    <h2>운영 — 시간축 추이 (표시 전용)</h2>
    <p class="sub">카메라가 이미 보내고 있는 값을 시간축으로 쌓아 그립니다. 카메라·서버·다른 탭은 건드리지 않으며, 이 탭은 아무 명령도 보내지 않습니다.</p>
    <div class="row">
      <label>시간창
        <select id="opsWin" onchange="opsSetWin(this.value)">
          <option value="1">최근 1분</option>
          <option value="5" selected>최근 5분</option>
          <option value="30">최근 30분</option>
        </select>
      </label>
      <span class="desc">보관은 30분. 브라우저 탭을 닫으면 이력은 사라집니다.</span>
      <span id="opsStat" class="desc"></span>
    </div>
    <div class="row" style="gap:14px">
      <span class="desc">채널마다 <b>자기 줄</b>로 그립니다 — 줄마다 y축이 따로라 값 크기가 달라도 눌리지 않습니다.
        줄이 안 보이는 채널은 그 창에 들어온 줄이 없다는 뜻(검출 꺼짐).</span>
      <span class="desc">실선 = 프레임 fps · proc 평균 / 점선 = 수신 fps · proc 최댓값 / 회색 = det 평균</span>
    </div>
  </div>

  <div class="group wide">
    <h2>프레임 처리량</h2>
    <p class="sub"><b>프레임 fps</b>는 <code>seq</code>가 바뀐 횟수(실제 검출 프레임 수)입니다.
       <b>수신 fps</b>는 줄 도착률이라 한 프레임에 마커가 N개면 N배로 나옵니다 — 둘은 다른 값입니다.</p>
    <canvas id="opsFpsCanvas" width="900" height="200" style="max-width:100%;height:auto;display:block;border:1px solid #444;background:#111;border-radius:8px;margin-top:4px"></canvas>
  </div>

  <div class="group wide">
    <h2>지연 (카메라 내부)</h2>
    <p class="sub"><code>proc</code> 평균과 최댓값, <code>det</code> 평균입니다. 전부 카메라가 자기 시계 안에서 잰 값이라
       파이·브라우저와 시계가 안 맞아도 영향이 없습니다. 네트워크 왕복 지연은 시계 동기가 필요해 여기 넣지 않았습니다.</p>
    <canvas id="opsLatCanvas" width="900" height="200" style="max-width:100%;height:auto;display:block;border:1px solid #444;background:#111;border-radius:8px;margin-top:4px"></canvas>
  </div>

  <div class="group wide">
    <h2>검출률</h2>
    <p class="sub">마커를 하나라도 본 프레임의 비율입니다. 같은 프레임의 여러 마커는 한 번만 셉니다.
       0%가 이어지면 마커가 가렸거나 그 렌즈의 검출이 꺼져 있는 것입니다.</p>
    <canvas id="opsHitCanvas" width="900" height="200" style="max-width:100%;height:auto;display:block;border:1px solid #444;background:#111;border-radius:8px;margin-top:4px"></canvas>
  </div>

  <div class="group wide">
    <h2>카메라 CPU</h2>
    <p class="sub">앱 전체 CPU(%)와 RSS(MB). 2초마다 오는 <code>CPU_STAT</code> 기준이며 채널 구분이 없습니다.</p>
    <canvas id="opsCpuCanvas" width="900" height="200" style="max-width:100%;height:auto;display:block;border:1px solid #444;background:#111;border-radius:8px;margin-top:4px"></canvas>
    <div class="hint" style="margin-top:8px">
      데이터가 안 쌓이면 그 채널의 검출이 꺼져 있는지 먼저 보세요 — 줄이 오지 않으면 그릴 것도 없습니다.
      CPU는 카메라가 <code>CPU_STAT</code>을 보내야 나옵니다.
    </div>
  </div>

  <div class="group wide">
    <h2>마커 추적 — 꼭짓점 픽셀 좌표</h2>
    <p class="sub">특정 마커 하나의 네 꼭짓점이 시간에 따라 어디에 있었는지. 채널은 상단 드롭다운을 따릅니다.</p>
    <div class="row">
      <label>마커 id
        <input type="number" id="opsMarkerId" value="49" min="0" step="1" style="width:6em"
               onchange="opsSetMarkerId(this.value)" oninput="opsSetMarkerId(this.value)">
      </label>
      <span class="desc">기본 49 = 로봇 마커. 바꾸면 이전 마커의 좌표는 버립니다(궤적이 섞이지 않게).</span>
    </div>
    <div class="row"><span id="opsMarkHint" class="desc"></span></div>
    <div class="hint" style="margin:2px 0 8px">
      꼭짓점 순서 <code>c0~c3</code>는 카메라가 보내는 순서 그대로입니다. 마커가 제자리에 있는데 선이 흔들리면
      그 폭이 곧 <b>검출 지터</b>이고, 호모그래피 정확도의 하한이 됩니다.
    </div>
    <canvas id="opsMark0Canvas" width="900" height="108" style="max-width:100%;height:auto;display:block;border:1px solid #444;background:#111;border-radius:8px;margin-top:4px"></canvas>
    <canvas id="opsMark1Canvas" width="900" height="108" style="max-width:100%;height:auto;display:block;border:1px solid #444;background:#111;border-radius:8px;margin-top:6px"></canvas>
    <canvas id="opsMark2Canvas" width="900" height="108" style="max-width:100%;height:auto;display:block;border:1px solid #444;background:#111;border-radius:8px;margin-top:6px"></canvas>
    <canvas id="opsMark3Canvas" width="900" height="108" style="max-width:100%;height:auto;display:block;border:1px solid #444;background:#111;border-radius:8px;margin-top:6px"></canvas>
  </div>

  <div class="group wide">
    <h2>마커 추적 — world 좌표</h2>
    <p class="sub">같은 마커의 바닥 좌표(mm)와 헤딩(°). 카메라가 <code>world=(X,Ymm,Adeg)</code>를 실어 보낼 때만 그려집니다 —
       그 렌즈에 <b>마커평면(H_marker)</b>이 준비돼야 나옵니다.</p>
    <canvas id="opsWorldCanvas" width="900" height="154" style="max-width:100%;height:auto;display:block;border:1px solid #444;background:#111;border-radius:8px;margin-top:4px"></canvas>
  </div>
</div>
<div id="shellPane" style="display:none">
  <div class="group wide">
    <h2>카메라 셸 — 빠른 진단</h2>
    <p class="sub">카메라에서 직접 실행합니다. 짧은 조회 명령만 사용하고, 긴 명령은 프레임 검출을 멈출 수 있습니다.</p>
    <details class="fold warn" id="foldShellWarn">
      <summary><b>⚠ 인증 없는 원격 셸 — 진단 전용, 프레임 스레드에서 실행</b></summary>
      <div class="foldbody">
        카메라에서 <b>인증 없이</b> 명령이 실행됩니다. 실험실 진단용이며 운용 빌드에서는
        <code>ENABLE_SHELL_CMD 0</code>으로 꺼야 합니다.<br>
        명령은 <b>프레임 스레드</b>에서 돌기 때문에 오래 걸리는 명령(<code>sleep</code>,
        큰 <code>cat</code>)은 그 시간만큼 검출이 멈춥니다. 출력은 120줄에서 잘립니다.
        <code>stderr</code>도 함께 옵니다.
      </div>
    </details>
    <table class="cmdtable">
      <tr class="grp"><td colspan="2">기본 상태</td></tr>
      <tr><td class="c"><button class="cmdlink" onclick="runShell('date; uptime; id')">date; uptime; id</button></td><td>시간 · 업타임 · 실행 권한</td></tr>
      <tr><td class="c"><button class="cmdlink" onclick="runShell('ps | head -n 20')">ps | head -n 20</button></td><td>프로세스 목록 (상위 20)</td></tr>
      <tr class="grp"><td colspan="2">/mnt 저장소</td></tr>
      <tr><td class="c"><button class="cmdlink" onclick="runShell('ls -lah /mnt/opensdk/storage/')">ls -lah /mnt/opensdk/storage/</button></td><td>앱 저장소 목록 — H·K/dist 저장 파일이 있는 곳</td></tr>
      <tr><td class="c"><button class="cmdlink" onclick="runShell('df -h /mnt')">df -h /mnt</button></td><td>/mnt 남은 용량</td></tr>
      <tr><td class="c"><button class="cmdlink" onclick="runShell('mount | grep mnt')">mount | grep mnt</button></td><td>/mnt 마운트 옵션 (ro 로 잡혀 있으면 저장이 실패한다)</td></tr>
      <tr><td class="c"><button class="cmdlink" onclick="runShell('ls -ld /mnt /mnt/opensdk /mnt/opensdk/storage')">ls -ld /mnt /mnt/opensdk /mnt/opensdk/storage</button></td><td>상위 폴더 권한</td></tr>
      <tr><td class="c"><button class="cmdlink" onclick="runShell('touch /mnt/__wtest &amp;&amp; echo WRITABLE &amp;&amp; rm -f /mnt/__wtest')">touch /mnt/__wtest &amp;&amp; echo WRITABLE &amp;&amp; rm -f /mnt/__wtest</button></td><td>/mnt 쓰기 가능 여부 확인</td></tr>
    </table>
    <div class="hint" style="margin:8px 0 2px">호모그래피·K/dist 저장 파일은 <code>/mnt/opensdk/storage/&lt;appName&gt;/</code> 아래에 있습니다.</div>
    <div class="term" onclick="focusShellInput(event)">
      <pre id="shOut"></pre>
      <div class="term-row"><span class="ps1">$</span>
        <input id="shInput" type="text"
               placeholder="ls -lah /mnt   (Enter 실행 · ↑/↓ 이전 명령 · Esc 지우기)" autocomplete="off"
               spellcheck="false"></div>
    </div>
    <div class="row" style="margin-top:8px">
      <button onclick="sendShell()">실행</button>
      <button onclick="clearShellOutput()">지우기</button>
      <button onclick="copyShellOutput()">출력 복사</button>
    </div>
  </div>

  <div class="group wide">
    <h2>SDK 진단</h2>
    <p class="sub">SDK가 raw 채널에서 지원한다고 보고하는 해상도를 확인합니다.</p>
    <div class="row"><button onclick="send('GET_RAW_RES')">해상도 목록 조회<span class="cmd">GET_RAW_RES</span></button>
      <span class="desc">SDK가 raw 채널에서 실제 지원한다고 보고하는 해상도 목록 조회.
        결과는 오른쪽 로그에 <code>[raw-res]</code>로 표시된다.</span></div>
    <details class="fold panel" id="foldSdkInfo">
      <summary>"지원한다고 말하는 값" — 실측치는 원시 좌표 탭에서</summary>
      <div class="foldbody hint">
        실기는 <b>PNO-A9081R</b>(Ambarella CV2BUB)로 확인됐고, SDK 문서 §10.1은
        이 모델의 raw를 <b>1080p 최대 10fps</b>로 명시한다 — 현재 매니페스트의
        10fps와 일치한다. (§10.3의 7180R은 4fps 상한이라 다른 모델이다.)<br>
        실제 프레임 주기는 <b>원시 좌표 탭</b>에서 <code>t_frame</code> 간격으로
        따로 볼 수 있다 — 이쪽은 "지원한다고 말하는 값", 저쪽은 "실제로 나오는 값".
      </div>
    </details>

    <p class="sub">검출을 끄고 프레임 도착만 세어, SDK가 실제로 몇 fps를 주는지 확인합니다.</p>
    <div class="row"><label class="toggle"><input type="checkbox" id="rawFpsTest"
        onchange="send('RAW_FPS_TEST ' + (this.checked ? 1 : 0))">
        raw fps 측정 모드<span class="cmd">RAW_FPS_TEST</span></label>
      <span class="desc"><b>켜면 마커 검출이 멈춘다.</b> 측정이 끝나면 반드시 끌 것.</span></div>
    <details class="fold panel" id="foldRawFps">
      <summary>이 모드가 왜 필요한가 — seq만으로는 병목을 못 가린다</summary>
      <div class="foldbody hint">
        평상시 <code>seq</code>는 콜백이 불린 횟수라, 그 속도는 <b>SDK 전달</b>과
        <b>우리 처리</b> 중 <b>느린 쪽</b>에 묶인다. 그래서 "SDK가 조금밖에 안 준다"와
        "SDK는 충분히 주는데 우리가 못 따라간다"가 <b>똑같은 숫자로 보인다</b>.<br>
        이 모드는 검출을 통째로 건너뛰어 우리 비용을 0으로 만든다. 그러면 남는 건
        SDK 전달 속도뿐이므로 두 경우가 구분된다.<br>
        <b>2026-07-20 측정</b>: 이 모드 <b>9.61fps</b> vs 평상시 <b>3.5fps</b> →
        SDK는 처음부터 10fps 가까이 주고 있었고, 병목은 <code>detectMarkers</code>
        (<code>proc</code>의 100%)로 확정. 오는 프레임의 약 <b>64%를 못 쓰고</b> 있다.
      </div>
    </details>
  </div>
</div>
<div id="main">
  <pre id="log"></pre>
</div>
</div>
<script>
__LOG_SUBJECT_JS__
function send(cmd) {
  return fetch('/cmd', {method:'POST', body: cmd});
}

// ArucoPosePNM(2026-08-10~) 명령 프로토콜: 채널을 쓰는 명령은 전부 첫/유일
// 인자로 0-based <ch>를 받는다 (ArucoPosePNM::sample_component.cc의
// sscanf(cmd+N, "%d", &ch) 패턴 전부). 이 대시보드의 #calibCh 셀렉트는
// 1-based(CH1..CH4, /calib/channel 이 만드는 값)라서 그대로 붙이면 항상 한
// 칸씩 밀린 렌즈를 캘리브레이션하게 된다 — chArg()가 그 변환을 담당한다.
//
// 채널 없이 UI가 뜨는 경우(RP_CAM_CHANNELS=1)는 #calibCh 자체가 비어있는데,
// 그때 ArucoPosePNM도 항상 채널 0 하나뿐이므로 0 폴백이 맞다.
function chArg() {
  const el = document.getElementById('calibCh');
  const v = el && el.value !== '' ? Number(el.value) : 1;
  return String(v - 1);
}
// <ch>가 유일/마지막 인자인 명령용 (CALIB_K_START/COMPUTE/UNDO/SAVE,
// CALIB_START, HG_SAVE/QUERY, ANCHOR_QUERY, MARKER_PLANE_QUERY/SAVE).
function sendCh(cmd) {
  return send(cmd + ' ' + chArg());
}

// 마커검출 탭의 #camRes 셀렉트(2592×1520 기본 / 1920×1080 FHD). raw 오버레이
// 캔버스가 K 없을 때 이 크기로 잡히고, 중앙 서버 H_MATRIX 번들의 image_size도
// 여기서 가져온다 — 예전엔 두 곳 다 "카메라 raw는 1920×1080 고정"이라고 가정한
// 하드코딩이었는데, ArucoPosePNM 실제 해상도(2592×1520)와 달라서 번들에 잘못된
// image_size가 실려 나가고 있었다 (2026-08-10 발견·수정).
function camRes() {
  const el = document.getElementById('camRes');
  const v = (el && el.value) || '2592x1520';
  const [w, h] = v.split('x').map(Number);
  return {w, h};
}
function onCamResChange() {
  if (typeof rawOverlayOn !== 'undefined' && rawOverlayOn) redrawRawCanvas();
}

// ===== 테마 (라이트 / 다크) =====
// The <head> script already picked one and set data-theme; this only flips it
// and remembers the choice. The label names the theme you'd GET by clicking.
function themeLabel(t) {
  document.getElementById('themeBtn').textContent = (t === 'dark') ? '라이트 모드' : '다크 모드';
}
function toggleTheme() {
  const next = document.documentElement.getAttribute('data-theme') === 'dark' ? 'light' : 'dark';
  document.documentElement.setAttribute('data-theme', next);
  try { localStorage.setItem('theme', next); } catch (e) {}
  themeLabel(next);
}
themeLabel(document.documentElement.getAttribute('data-theme'));

const boardInputs = ['bsx','bsy','bsquare','bmarker','bdict','bmarginx','bmarginy']
  .map(id => document.getElementById(id));
const boardSummary = document.getElementById('boardSummary');
const boardBrief = document.getElementById('boardBrief');
const captureBtn = document.getElementById('captureBtn');
const computeBtn = document.getElementById('computeBtn');
let capturePending = false;
let acceptedViews = 0;
let kSessionCh = null;  // CALIB_K_START 를 보낸 채널(0-based). CAPTURE는 채널무관이라 그대로, COMPUTE만 이 채널로 고정한다.
let targetViews = 20;

function numberValue(id) {
  return Number(document.getElementById(id).value);
}

function updateBoardSummary() {
  const sx = numberValue('bsx'), sy = numberValue('bsy');
  const square = numberValue('bsquare'), marker = numberValue('bmarker');
  const mx = numberValue('bmarginx'), my = numberValue('bmarginy');
  const dict = numberValue('bdict');
  const capacities = [50,100,250,1000];
  const dictCapacity = dict <= 15 ? capacities[dict % 4]
    : [1024,30,35,2320,587][dict - 16];
  const markerCount = Math.floor(sx * sy / 2);
  const quiet = (square - marker) / 2;
  const bw = sx * square, bh = sy * square;
  const pw = bw + 2 * mx, ph = bh + 2 * my;
  const valid = sx >= 3 && sy >= 3 && square > 0 && marker > 0 &&
                marker < square && mx >= 0 && my >= 0 &&
                markerCount <= dictCapacity;
  boardSummary.textContent = valid
    ? `마커 주변 공백 ${quiet.toFixed(1)}mm · 패턴 ${bw.toFixed(1)}×${bh.toFixed(1)}mm · 용지 ${pw.toFixed(1)}×${ph.toFixed(1)}mm · 내부 코너 ${(sx-1)*(sy-1)}개`
    : '설정 오류: 치수 조건 또는 Dictionary의 마커 ID 용량을 확인하세요.';
  boardSummary.style.color = valid ? 'var(--text2)' : 'var(--red)';

  // The spec at a glance, shown on the fold's summary line so the numbers stay
  // readable while the form is collapsed -- checking them against the printout
  // is the common case; editing them is the rare one. An invalid form says so
  // here too, or collapsing it would hide the error.
  const dictName = document.querySelector('#bdict option[value="' + dict + '"]');
  boardBrief.textContent = valid
    ? `${sx}×${sy} · 칸 ${square}mm · 마커 ${marker}mm · ${dictName ? dictName.textContent : 'dict ' + dict}`
    : '설정 오류 — 확인 필요';
  boardBrief.style.color = valid ? 'var(--text3)' : 'var(--red)';
  return valid;
}

function applyBoardConfig() {
  if (!updateBoardSummary()) return;
  const values = [
    numberValue('bsx'), numberValue('bsy'), numberValue('bsquare'),
    numberValue('bmarker'), numberValue('bdict'),
    numberValue('bmarginx'), numberValue('bmarginy')
  ];
  send('CALIB_K_CONFIG ' + values.join(' '));
}

function applyKParams() {
  const t = numberValue('ktarget'), r = numberValue('krms');
  send('CALIB_K_SET ' + t + ' ' + r);
}

function captureView() {
  if (capturePending) return;
  capturePending = true;
  captureBtn.disabled = true;
  // 드롭다운 채널에만 보낸다(2026-08-11). 예전에는 채널 없이 보냈고, 카메라는
  // 그걸 "열린 세션 전부 무장"으로 처리한다 — 안 쓰는 렌즈에 세션이 남아 있으면
  // 그 렌즈도 매번 자기가 본 보드 조각을 뷰로 쌓고, 그게 그 렌즈의 K가 된다.
  // (CH1이 화면 구석 조각들로 fx=8723 짜리 K를 만든 적이 있다.)
  //
  // 구버전 카메라는 <ch> 를 무시하고 예전처럼 동작하므로 먼저 배포해도 안전하다.
  // 드롭다운이 세션 없는 채널을 가리키면 카메라가 그렇게 답한다.
  sendCh('CALIB_K_CAPTURE');
  setTimeout(() => {
    if (capturePending) {
      capturePending = false;
      captureBtn.disabled = false;
      kstatus.className = 'reject';
      kinfo.textContent = '카메라 응답 시간 초과 — 연결과 raw frame을 확인하세요';
    }
  }, 5000);
}

function startCalibration() {
  if (acceptedViews > 0 &&
      !confirm(`현재 승인된 ${acceptedViews}개 뷰를 지우고 새로 시작할까요?`)) {
    return;
  }
  const kother = document.getElementById('kother');
  if (kother) kother.textContent = '';
  // 채널 인자 없는 CALIB_K_STOP = 열린 세션 전부 종료(카메라 HandleCalibK: 인자
  // 파싱에 실패하면 모든 채널을 Stop). 남아 있던 세션은 캡처를 누를 때마다 같이
  // 무장돼 결과를 헷갈리게 만들 뿐이고, 정작 그 렌즈로 캘리를 하려면 어차피
  // 새로 시작해야 한다.
  //
  // 여러 렌즈를 일부러 동시에 모으고 싶으면(겹쳐 보는 구간을 보드 한 자세로
  // 한꺼번에 채우는 경우) 이 버튼 대신 각 채널에서 CALIB_K_START 를 직접 보내면
  // 된다 — 카메라는 그 방식을 계속 지원한다.
  send('CALIB_K_STOP');
  kSessionCh = curCh();
  sendCh('CALIB_K_START');
}

function stopCalibration(ev) {
  const all = !!(ev && ev.shiftKey);
  const ch = curCh();
  if (!confirm(all
      ? '4채널의 캘리 세션을 전부 종료합니다. 승인된 뷰는 모두 버려집니다. 계속할까요?'
      : `CH${ch + 1} 의 캘리 세션을 종료합니다. 승인된 뷰 ${acceptedViews}개는 버려집니다. 계속할까요?`)) {
    return;
  }
  const kother = document.getElementById('kother');
  if (kother) kother.textContent = '';
  if (all) {
    send('CALIB_K_STOP');
    kSessionCh = null;
  } else {
    sendCh('CALIB_K_STOP');
    if (kSessionCh === ch) kSessionCh = null;
  }
  // 카메라는 STOP 에 리포트를 보내지 않으므로(HandleCalibK 의 STOP 분기는 조용히
  // Stop 만 한다) 화면 상태는 여기서 직접 되돌린다.
  acceptedViews = 0;
  kstatus.className = '';
  kcount.textContent = `0 / ${targetViews}`;
  kinfo.textContent = all ? '전 채널 세션 종료됨' : `CH${ch + 1} 세션 종료됨`;
  captureBtn.disabled = true;
  computeBtn.disabled = true;
  updateKBar();
}

function computeCalibration() {
  if (acceptedViews < targetViews) return;
  const ch = (kSessionCh !== null) ? kSessionCh : curCh();
  if (ch !== curCh() &&
      !confirm(`캘리 세션은 CH${ch + 1}에서 시작됐는데 드롭다운은 지금 CH${curCh() + 1}입니다. 세션 채널 CH${ch + 1}로 계산합니다. 계속할까요?`)) {
    return;
  }
  computeBtn.disabled = true;
  captureBtn.disabled = true;
  kstatus.className = 'done';
  kinfo.textContent = '계산 시작 요청 — 완료 응답을 기다리는 중';
  send('CALIB_K_COMPUTE ' + ch);
}

boardInputs.forEach(input => input.addEventListener('input', updateBoardSummary));
updateBoardSummary();

// ===== 접기 블록 상태 기억 =====
// Default open/closed comes from the `open` attribute in the markup; once you
// choose, your choice sticks per block.
document.querySelectorAll('details.fold').forEach(d => {
  const key = 'fold:' + d.id;
  const saved = localStorage.getItem(key);
  if (saved !== null) d.open = (saved === '1');
  d.addEventListener('toggle', () => localStorage.setItem(key, d.open ? '1' : '0'));
});

const log = document.getElementById('log');
const kstatus = document.getElementById('kstatus');
const kcount = document.getElementById('kcount');
const kinfo = document.getElementById('kinfo');
const kbar = document.querySelector('#kbar i');

// Mirror of the accepted/target counter the log lines already carry — purely a
// second reading of the same numbers, so it can never disagree with #kcount.
function updateKBar(pct) {
  if (pct === undefined) {
    pct = targetViews > 0 ? Math.min(100, acceptedViews / targetViews * 100) : 0;
  }
  kbar.style.width = pct + '%';
}

// CALIB_K_CAPTURE 한 번은 "열려 있는 모든" 세션을 무장한다(카메라 설계 — 겹쳐
// 보는 렌즈들이 보드 한 자세를 나눠 갖게 하려는 것). 그래서 한 번 누르면 세션
// 수만큼 PROGRESS 가 오고, 그중 지금 캘리 중인 렌즈의 것은 하나뿐이다.
//
// 예전에는 채널을 안 보고 오는 대로 acceptedViews/kinfo 를 덮었다. 보드가 안
// 보이는 렌즈의 "코너 1/24 거부"가 캘리 중인 렌즈(24/24 정상)의 결과로 표시됐고,
// 승인 리포트였다면 뷰 카운터까지 남의 것으로 바뀌었을 것이다 — 표시 문제가
// 아니라 계산에 들어가는 숫자가 어긋나는 문제다.
//
// 세션 채널을 모르는 경우(kSessionCh === null — 페이지를 새로 연 직후 등)에는
// 예전처럼 전부 받는다. 그때는 어긋날 기준 자체가 없다.
function otherLensReport(ch, note) {
  if (kSessionCh === null || ch === kSessionCh) return false;
  const kother = document.getElementById('kother');
  if (kother) {
    kother.textContent = `CH${ch + 1} 도 같은 캡처에 반응했습니다` +
      (note ? ` — ${note}` : '') +
      `. 이 렌즈에도 세션이 열려 있습니다 (지금 캘리 중인 것은 CH${kSessionCh + 1}).`;
  }
  return true;  // 이 채널의 숫자는 화면 상태에 반영하지 않는다
}

function updateKStatus(line) {
  let m;
  if ((m = line.match(/session started target=(\\d+)/))) {
    acceptedViews = 0; targetViews = Number(m[1]);
    kstatus.className = ''; kcount.textContent = `0 / ${targetViews}`;
    kinfo.textContent = '세션 시작 — 보드를 기울여가며 자세를 바꿔 캡처하세요';
    captureBtn.disabled = false;
    computeBtn.disabled = true;
    updateKBar();
  } else if ((m = line.match(/ch=(\\d+) captured view (\\d+)\\/(\\d+) \\((\\d+)\\/(\\d+) corners, coverage=([\\d.]+)%, sharpness=([\\d.]+), move=([\\d.-]+)px\\)/))) {
    if (otherLensReport(Number(m[1]), null)) return;
    capturePending = false; captureBtn.disabled = false;
    acceptedViews = Number(m[2]); targetViews = Number(m[3]);
    kstatus.className = ''; kcount.textContent = m[2] + ' / ' + m[3];
    kinfo.textContent = `통과 · 코너 ${m[4]}/${m[5]} · 화면점유 ${m[6]}% · 선명도 ${m[7]} · 이동 ${m[8]}px`;
    computeBtn.disabled = acceptedViews < targetViews;
    updateKBar();
  } else if ((m = line.match(/ch=(\\d+) capture REJECTED — (.+) corners=(\\d+)\\/(\\d+) coverage=([\\d.]+)% sharpness=([\\d.]+) move=([\\d.-]+)px/))) {
    if (otherLensReport(Number(m[1]), `${m[2]} (코너 ${m[3]}/${m[4]})`)) return;
    capturePending = false; captureBtn.disabled = false;
    kstatus.className = 'reject';
    kinfo.textContent = `거부 · ${m[2]} · 코너 ${m[3]}/${m[4]} · 화면점유 ${m[5]}% · 선명도 ${m[6]}`;
  } else if ((m = line.match(/BOARD_CONFIG views=(\\d+)\\/(\\d+) squares=(\\d+)x(\\d+) square=([\\d.]+) marker=([\\d.]+) dict=(\\d+) margin=([\\d.]+)\\/([\\d.]+)/))) {
    acceptedViews = Number(m[1]); targetViews = Number(m[2]);
    document.getElementById('ktarget').value = m[2];
    document.getElementById('bsx').value = m[3];
    document.getElementById('bsy').value = m[4];
    document.getElementById('bsquare').value = m[5];
    document.getElementById('bmarker').value = m[6];
    document.getElementById('bdict').value = m[7];
    document.getElementById('bmarginx').value = m[8];
    document.getElementById('bmarginy').value = m[9];
    kcount.textContent = `${acceptedViews} / ${targetViews}`;
    computeBtn.disabled = acceptedViews < targetViews;
    updateBoardSummary();
    kinfo.textContent = '카메라에 저장된 보드 설정을 적용했습니다';
    updateKBar();
  } else if ((m = line.match(/params updated: target=(\\d+) views, rms_limit=([\\d.]+)px \\(accepted views=(\\d+)\\)/))) {
    targetViews = Number(m[1]);
    acceptedViews = Number(m[3]);
    document.getElementById('ktarget').value = m[1];
    document.getElementById('krms').value = m[2];
    kcount.textContent = `${acceptedViews} / ${targetViews}`;
    computeBtn.disabled = acceptedViews < targetViews;
    kinfo.textContent = `세션 조건 변경: 목표 ${m[1]}뷰 · 합격 RMS ${m[2]}px`;
    updateKBar();
  } else if (line.includes('COMPUTING with')) {
    kstatus.className = 'done';
    kinfo.textContent = 'OpenCV 캘리브레이션 계산 중 — 수 초 기다리세요';
    captureBtn.disabled = true;
    computeBtn.disabled = true;
  } else if ((m = line.match(/SUCCESS rms=([\\d.]+)px/))) {
    kstatus.className = 'done'; kcount.textContent = 'DONE';
    kinfo.textContent = 'calibrated! rms=' + m[1] + 'px';
    captureBtn.disabled = false;
    updateKBar(100);
  } else if (line.includes('[calib-K] FAILED')) {
    kstatus.className = 'reject';
    kinfo.textContent = line.replace('[calib-K]', '').trim();
    captureBtn.disabled = false;
    computeBtn.disabled = acceptedViews < targetViews;
  }
}

// Only auto-scroll to the bottom if the user was already there — if they've
// scrolled up to read something, new lines shouldn't yank them back down.
function isNearBottom() {
  return log.scrollHeight - log.scrollTop - log.clientHeight < 40;
}

// MARKER LOST 로그 숨기기 토글 (표시 필터만 — CSV/데이터 로깅엔 영향 없음)
const hideLostBox = document.getElementById('hideLost');
let hideLost = localStorage.getItem('hideLost') === '1';
hideLostBox.checked = hideLost;
hideLostBox.addEventListener('change', () => {
  hideLost = hideLostBox.checked;
  localStorage.setItem('hideLost', hideLost ? '1' : '0');
  if (hideLost) {
    log.textContent = log.textContent.split("\\n")
      .filter(l => !l.includes('MARKER LOST')).join("\\n");
  }
});

// Raw Corners 탭 — 이미 흐르는 CAM_POSE 로그를 파싱해 표시만 (카메라에 명령 안 보냄)
//
// 채널별 오버레이 상태 (0-based ch, chArg()/ArucoPosePNM과 동일 인덱싱).
// ArucoPosePNM은 4채널을 전부 이 한 TCP 링크로 섞어 보내는데, 예전엔
// rawFrame/rawSeq/dynRoi* 가 전역 변수 하나였다 — 채널을 바꿔도 화면엔
// "마지막으로 도착한 아무 채널"의 코너가 계속 떠 있었고, seq 판정도 채널이
// 섞이면 "새 프레임 시작"을 잘못 판단했다(각 채널이 자기 seq_[ch]를 따로
// 센다). 여기서 채널별로 나눈다 (2026-08-10).
// procs/dets/hits/arrivals/lastNet/lastSeqSeen/seqGaps도 여기 채널별 상태에
// 넣는다(2026-08-11) — 바로 위 코멘트가 설명하는 것과 정확히 같은 버그가
// handleLatencyCollect()에도 그대로 있었다: 전역 lastSeqSeen 하나에 4채널의
// seq를 다 밀어넣고 있어서, 채널이 다른 두 줄(예: ch0 seq=50 다음에 ch1
// seq=12000)이 연속으로 오면 "11949 프레임 누락"으로 잘못 세었다 — 실제
// 데이터가 아니라 채널이 섞여서 생긴 숫자다("seq 누락" 표시가 수백만까지
// 치솟은 원인, 2026-08-11 사용자 보고로 발견).
function emptyChOverlay() {
  return { rawFrame: {}, rawBuilding: {}, rawSeq: null,
           dynRoiMargin: 240, dynRoiTracking: false, dynRoiTrackSize: null,
           procs: [], dets: [], hits: [], arrivals: [],
           wdets: {},   // WISEAI_DET: object_id -> {foot_u,foot_v,foot_r,inside,ts}
           bands: {},   // ZONE_BAND: kind -> {mm, points[]} (raw 픽셀, 그리기 전용)

           lastNet: null, lastSeqSeen: null, seqGaps: 0 };
}
let chOverlay = {0: emptyChOverlay(), 1: emptyChOverlay(), 2: emptyChOverlay(), 3: emptyChOverlay()};
function curCh() { return Number(chArg()); }
function curOverlay() { return chOverlay[curCh()]; }

// 호모그래피 탭 상태, 같은 이유로 채널별 (2026-08-10). hgHfloor/mpPlane/
// hgCameraAnchors 는 예전처럼 전역 변수로 남겨뒀다 — 이미 그 값들을 읽는
// 코드가 파일 곳곳에 많아서, 여기서는 "지금 선택된 채널의 값을 그 전역에
// 넣어두는" 방식을 쓴다(dynRoiTracking을 curOverlay()로 옮긴 것과 달리).
// syncCalibFromChannel()이 채널 전환 때마다 그 동기화를 한다.
function emptyChHg() { return { Hfloor: null, mpPlane: null, anchors: null, fit: null, fitPts: {}, undistorted: null, ivaSync: null }; }
// 번들 coord_mode 의 근거. 1순위는 카메라가 보고한 "이 H 가 피팅된 공간"이고,
// 2순위가 이 브라우저 세션의 HG_COORD_MODE 이력이다. 순서가 중요하다 — 후자는
// 새로고침하면 사라지는데, 그때 coord_mode 가 "unknown" 이 되어 번들이 반려됐다.
function hgCoordModeForBundle() {
  const u = (chHg[curCh()] || {}).undistorted;
  if (u === true)  return 'undistort';
  if (u === false) return 'raw';        // R-2 위반으로 아래에서 걸린다 — 숨기지 않는다
  return hgCoordModeActive || null;
}
let chHg = {0: emptyChHg(), 1: emptyChHg(), 2: emptyChHg(), 3: emptyChHg()};

let rawOn = false;
const rawBox = document.getElementById('rawCorners');
const rawBtn = document.getElementById('rawBtn');
// 도움말 토글: 기본은 숨김(버튼만 촘촘히) → 누르면 각 버튼 설명 표시. 상태 저장.
function toggleHelp() {
  const on = document.body.classList.toggle('show-help');
  document.getElementById('helpBtn').classList.toggle('on', on);
  localStorage.setItem('showHelp', on ? '1' : '0');
}
if (localStorage.getItem('showHelp') === '1') {
  document.body.classList.add('show-help');
  document.getElementById('helpBtn').classList.add('on');
}

// 터미널(오른쪽 로그) 패널 토글: 접으면 왼쪽 작업영역이 전체 폭을 차지한다.
// 버튼 .on = 패널 보이는 상태(기본). 상태는 localStorage에 저장해 새로고침에도 유지.
// 로그 자체는 계속 흐른다 — 화면에서 감출 뿐이라 펼치면 그동안의 내용이 그대로 있다.
function toggleLogPanel() {
  const collapsed = document.getElementById('content').classList.toggle('log-collapsed');
  document.getElementById('logBtn').classList.toggle('on', !collapsed);
  localStorage.setItem('logCollapsed', collapsed ? '1' : '0');
}
if (localStorage.getItem('logCollapsed') === '1') {
  document.getElementById('content').classList.add('log-collapsed');
  document.getElementById('logBtn').classList.remove('on');
}

function toggleRaw() {
  rawOn = !rawOn;
  rawBtn.textContent = rawOn ? '좌표 보기 정지' : '좌표 보기 시작';
  rawBtn.classList.toggle('on', rawOn);
  // rawFrame is kept (the overlay may still use it); only clear the table view.
  if (!rawOn) rawBox.innerHTML = '대기 중…';
}

// 렌즈마다 K/dist가 전부 다르므로(4채널) 채널별로 캐시한다. kCalib은 "지금
// 선택된 채널의 값"을 가리키는 전역 별칭으로 예전처럼 남겨뒀다 — 이미 그 값을
// 읽는 보정좌표 렌더링 코드가 파일 곳곳에 많아서다(hgHfloor/mpPlane과 같은
// 이유, syncCalibFromChannel()이 채널 전환 때마다 동기화한다. 2026-08-10).
let chKCalib = {0: null, 1: null, 2: null, 3: null};
// 현재 카메라에 로드된 K/dist 캐시 (CALIB_K_QUERY 브로드캐스트에서 채움).
// 보정 좌표 표시는 이 값에만 의존 — 카메라에 추가 명령을 보내지 않는다.
let kCalib = null;      // {fx, fy, cx, cy, dist:[k1,k2,p1,p2,k3]}
let showUndist = true;
const undistState = document.getElementById('undistState');

function refreshUndistState() {
  if (!showUndist) { undistState.textContent = ''; return; }
  undistState.textContent = kCalib
    ? `K 로드됨 (fx=${kCalib.fx.toFixed(1)}, cx=${kCalib.cx.toFixed(1)})`
    : 'K 미로드 — CALIB_K_QUERY 요청 중…';
}
function toggleUndist() {
  showUndist = document.getElementById('undistChk').checked;
  // 켤 때 K가 없으면 한 번 조회해 캐시를 채운다(표시용, 프레임경로 영향 없음).
  if (showUndist && !kCalib) sendCh('CALIB_K_QUERY');
  refreshUndistState();
  renderRaw();
  if (rawOverlayOn) redrawRawCanvas();
}

// OpenCV undistortPoints와 동일한 반복해: 왜곡 픽셀(u,v) → 보정 픽셀.
// 정규화 → 반복적으로 왜곡 제거 → 같은 K로 재투영(P=K)이라 픽셀 좌표로 돌아온다.
function undistortPixel(u, v) {
  if (!kCalib) return null;
  const fx = kCalib.fx, fy = kCalib.fy, cx = kCalib.cx, cy = kCalib.cy;
  const d = kCalib.dist;
  const k1 = d[0] || 0, k2 = d[1] || 0, p1 = d[2] || 0, p2 = d[3] || 0, k3 = d[4] || 0;
  const x0 = (u - cx) / fx, y0 = (v - cy) / fy;
  let x = x0, y = y0;
  for (let it = 0; it < 10; it++) {
    const r2 = x * x + y * y;
    const radial = 1 / (1 + ((k3 * r2 + k2) * r2 + k1) * r2);
    const dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
    const dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
    x = (x0 - dx) * radial;
    y = (y0 - dy) * radial;
  }
  return [fx * x + cx, fy * y + cy];
}
// 네 변(c0-c1, c1-c2, c2-c3, c3-c0) 픽셀 길이의 평균 = 평균 마커 크기(px)
function markerSidePx(c) {
  let sum = 0;
  for (let i = 0; i < 4; i++) {
    const a = c[i], b = c[(i + 1) % 4];
    sum += Math.hypot(Number(a[0]) - Number(b[0]), Number(a[1]) - Number(b[1]));
  }
  return sum / 4;
}
function renderRaw() {
  const ids = Object.keys(curOverlay().rawFrame);
  // 검출이 꺼져 있으면 '마커 없음'이 아니라 꺼졌다고 말한다. 둘 다 빈 화면이라
  // 구분이 안 되면 기능이 고장난 것처럼 보인다.
  //
  // "4채널 다 켜짐" 같은 전역 값이 아니라 detectByCh[curCh()](지금 보이는 그
  // 채널만)로 판정한다 — 채널을 헤더 표에서 개별로 켜고 끌 수 있게 된 뒤로
  // (2026-08-11) 전역 판정은 4채널 중 하나라도 꺼지면 false 가 되므로, 정작
  // 지금 보고 있는 채널은 검출 중인데도 "꺼져 있습니다"로 잘못 나왔다
  // (2026-08-11 사용자 보고로 발견).
  if (!detectByCh[curCh()]) {
    rawBox.innerHTML = '<span class="none">이 채널(CH' + (curCh() + 1) + ')의 마커 검출이 꺼져 있습니다 — ' +
      '상단 표의 <b>검출</b> 행에서 CH' + (curCh() + 1) + ' 알약을 눌러 켜세요 (카메라가 하트비트만 보내는 중)</span>';
    return;
  }
  if (!ids.length) { rawBox.innerHTML = '<span class="none">이 프레임에 인식된 마커 없음</span>'; return; }
  const cmp = showUndist && kCalib;   // 보정 열을 함께 그릴지
  let html = '';
  for (const id of ids) {
    const c = curOverlay().rawFrame[id];
    html += `<div class="mid">CH${curCh() + 1} · id ${id} · 평균 변 ${markerSidePx(c).toFixed(1)}px</div><table>`;
    if (cmp) {
      html += '<tr><th>코너</th><th>raw 픽셀</th><th></th><th>보정 픽셀</th><th>이동 Δx, Δy</th><th>|Δ|</th></tr>';
      for (let i = 0; i < 4; i++) {
        const rx = Number(c[i][0]), ry = Number(c[i][1]);
        const u = undistortPixel(rx, ry);
        const dx = u[0] - rx, dy = u[1] - ry, dpx = Math.hypot(dx, dy);
        html += `<tr><td>c${i}</td><td class="coord-raw">(${rx.toFixed(1)}, ${ry.toFixed(1)})</td>` +
                `<td class="coord-arrow">→</td><td class="coord-undist">(${u[0].toFixed(1)}, ${u[1].toFixed(1)})</td>` +
                `<td>(${dx >= 0 ? '+' : ''}${dx.toFixed(1)}, ${dy >= 0 ? '+' : ''}${dy.toFixed(1)}) px</td><td>${dpx.toFixed(1)} px</td></tr>`;
      }
    } else {
      for (let i = 0; i < 4; i++) html += `<tr><td>c${i}</td><td>x=${c[i][0]}</td><td>y=${c[i][1]}</td></tr>`;
    }
    html += '</table>';
  }
  if (showUndist && !kCalib)
    html += '<div class="hint" style="margin-top:6px">보정 좌표: 카메라에 로드된 K/dist가 없어 raw만 표시합니다. 캘리브레이션 탭에서 K를 로드/계산하세요.</div>';
  rawBox.innerHTML = html;
}
// Rolling proc-time stats. Kept over the last N frames because a single value
// says little -- what matters for the robot's delay compensation is the spread
// (and the roadmap's "프레임레이트 상향하며 처리시간 실측" is exactly this
// measurement). Runs regardless of the raw-corner toggle: it costs one regex
// per line and the numbers are only trustworthy while frames actually flow.
const PROC_WINDOW = 120;
// procs/dets/hits/arrivals/lastNet/lastSeqSeen/seqGaps는 이제 chOverlay[ch]
// 안에 채널별로 있다(2026-08-11, emptyChOverlay() 참고) — 예전엔 여기 전역
// 배열이었는데, 4채널이 한 TCP 링크에 섞여 오다 보니 다른 채널의 seq가 끼어들
// 때마다 "수만~수백만 프레임 누락"으로 잘못 세고 있었다.
//
// 카메라 CPU는 채널과 무관한 앱 전체 수치라 여기는 그대로 전역이다. CAM_POSE와
// 별개로 2초마다 오므로 마지막 값을 들고 있다가 지연 표가 다시 그려질 때 같이
// 렌더한다.
//
// ArucoPosePNM이 CPU_STAT을 실제로 보내기 시작한 게 2026-08-11이라(그 전엔
// 카메라 쪽에 이 리포트 자체가 없어서 cctv_app 시절 파서가 계속 死코드였다)
// 필드도 cctv_app의 app_pct/sys_pct 두 값이 아니라 cpu_pct(이 앱 vs wall
// clock)/cores/rss_kb/heap_in_use_kb/core_pct(코어별 전체 부하)로 다르다.
let cpuApp = null, cpuCores = null, cpuCorePct = [], cpuRssKb = null, cpuHeapKb = null, cpuAt = 0;
function handleCpu(line) {
  const m = line.match(/\\[cpu\\] app=(-?[\\d.]+)% cores=(\\d+) core=\\[([^\\]]*)\\] rss=(-?\\d+)KB heap=(-?\\d+)KB/);
  if (!m) return;
  const a = Number(m[1]);
  cpuApp = a >= 0 ? a : null;      // 카메라가 -1 로 "못 읽음"을 알린다
  cpuCores = Number(m[2]);
  cpuCorePct = m[3] ? m[3].split(',').map(Number) : [];
  cpuRssKb = Number(m[4]);
  cpuHeapKb = Number(m[5]);
  cpuAt = Date.now();
  renderRawLatency();
}
function cpuText() {
  if (cpuAt === 0)
    return '<span class="none">보고 없음 (카메라 재빌드 필요)</span>';
  const stale = (Date.now() - cpuAt) > 8000 ? ' <span class="none">(오래됨)</span>' : '';
  const app = cpuApp === null ? '—' : cpuApp.toFixed(1) + '%';
  const coreTxt = cpuCorePct.length
    ? ' · 코어별 ' + cpuCorePct.map(v => (v >= 0 ? v.toFixed(0) : '—') + '%').join('/')
    : '';
  const memTxt = (cpuRssKb !== null && cpuHeapKb !== null)
    ? ' · RSS ' + (cpuRssKb / 1024).toFixed(1) + 'MB (힙 ' + (cpuHeapKb / 1024).toFixed(1) + 'MB)'
    : '';
  return '앱 ' + app + ' · ' + (cpuCores || 1) + '코어' + coreTxt + memTxt + stale;
}
const rawLatency = document.getElementById('rawLatency');

function handleLatency(line) {
  const pm = line.match(/proc=(-?\\d+)ms/);
  const chm = line.match(/ch=(\\d+)/);
  // ch 없는 줄(옛 프로토콜 흔적)은 무시 -- 어느 렌즈인지 모르고 전역에 반영하면
  // 다시 채널이 섞인다. 이 파일의 다른 채널별 핸들러들과 동일한 규칙.
  if (!pm || !chm) return;
  const ch = Number(chm[1]);
  const ov = chOverlay[ch];
  if (!ov) return;
  handleLatencyCollect(ov, line, Number(pm[1]));
  // 렌더는 scheduleRawRender() 의 rAF 에서 한 번만 한다 -- 줄마다 표를 다시 만들면
  // 마커 개수만큼 innerHTML 이 교체되어 표가 깜빡인다. 지금 보이는 채널이 아니면
  // 스케줄할 필요가 없다 -- 그 채널로 돌아올 때 chOverlay에 이미 쌓여 있다.
  if (ch === curCh()) scheduleRawRender();
}
function handleLatencyCollect(ov, line, procMs) {
  ov.procs.push(procMs);
  if (ov.procs.length > PROC_WINDOW) ov.procs.shift();

  const nm = line.match(/net=(-?\\d+)ms/);
  ov.lastNet = nm ? Number(nm[1]) : null;      // absent/?clock -> unknown

  ov.arrivals.push(Date.now());
  if (ov.arrivals.length > PROC_WINDOW) ov.arrivals.shift();

  const dm = line.match(/det=(-?\\d+)ms/);
  ov.dets.push(dm ? Number(dm[1]) : null);
  if (ov.dets.length > PROC_WINDOW) ov.dets.shift();

  // 같은 프레임에 마커가 여러 개면 seq 가 같은 줄이 여러 번 온다. 검출률은
  // "마커를 본 프레임 비율"이어야 하므로 seq 단위로 한 번만 센다. seq는 이
  // 채널(ArucoPosePNM의 seq_[ch]) 안에서만 연속이라 lastSeqSeen/seqGaps도
  // 채널별이어야 한다 — 전역이던 시절엔 다른 채널의 seq가 끼어들 때마다
  // "수십만 프레임 누락"으로 잘못 세었다(2026-08-11 발견).
  const sm = line.match(/seq=(\\d+)/);
  const seq = sm ? Number(sm[1]) : null;
  const fresh = (seq === null || seq !== ov.lastSeqSeen);
  if (fresh) {
    if (seq !== null && ov.lastSeqSeen !== null && seq > ov.lastSeqSeen + 1)
      ov.seqGaps += seq - ov.lastSeqSeen - 1;
    ov.hits.push(line.includes('MARKER LOST') ? 0 : 1);
    if (ov.hits.length > PROC_WINDOW) ov.hits.shift();
  }
  if (seq !== null) ov.lastSeqSeen = seq;
}

// 지연 표와 캔버스 HUD 가 함께 쓰는 계산. null = 아직 값이 없음. 전부 "지금
// 보이는 채널"(curOverlay()) 기준이다.
//
// fps 는 창 전체의 경과시간으로 나눈다. 프레임별 간격의 평균을 쓰면 한 번의 긴
// 정지가 평균을 통째로 끌어내려 실제 처리량보다 낮게 보인다.
function metricFps() {
  const arrivals = curOverlay().arrivals;
  if (arrivals.length < 2) return null;
  const span = (arrivals[arrivals.length - 1] - arrivals[0]) / 1000;
  return span > 0 ? (arrivals.length - 1) / span : null;
}
function metricDetCur() {
  const dv = curOverlay().dets.filter(v => v !== null);
  return dv.length ? dv[dv.length - 1] : null;
}
function metricDetAvg() {
  const dv = curOverlay().dets.filter(v => v !== null);
  return dv.length ? dv.reduce((a, b) => a + b, 0) / dv.length : null;
}
function metricHitPct() {
  const hits = curOverlay().hits;
  return hits.length ? hits.reduce((a, b) => a + b, 0) / hits.length * 100 : null;
}

function renderRawLatency() {
  const ov = curOverlay();
  const procs = ov.procs, hits = ov.hits;
  if (!procs.length) { rawLatency.innerHTML = '<span class="none">이 채널은 아직 데이터 없음 — 검출을 켜고 잠시 기다리세요</span>'; return; }
  const n = procs.length;
  const cur = procs[n - 1];
  const avg = procs.reduce((a, b) => a + b, 0) / n;
  const min = Math.min.apply(null, procs);
  const max = Math.max.apply(null, procs);
  const netTxt = ov.lastNet === null
    ? '<span class="none">시계 미동기 — 표시 불가</span>'
    : ov.lastNet + ' ms';
  const totTxt = ov.lastNet === null
    ? '<span class="none">—</span>'
    : (cur + ov.lastNet) + ' ms';

  const fpsTxt = metricFps() === null
    ? '<span class="none">측정 중…</span>' : metricFps().toFixed(1) + ' fps';
  const detTxt = metricDetCur() === null
    ? '<span class="none">—</span>'
    : metricDetCur() + ' ms (평균 ' + metricDetAvg().toFixed(1) + ')';
  const hitTxt = hits.length
    ? Math.round(metricHitPct()) + '% (' + hits.reduce((a, b) => a + b, 0) + '/' + hits.length + ' 프레임)'
    : '<span class="none">—</span>';
  rawLatency.innerHTML =
    '<div class="qtitle">proc — 카메라 내부 처리 (CH' + (curCh() + 1) + ', 최근 ' + n + '프레임)</div>' +
    '<table>' +
    '<tr><td>현재</td><td>' + cur + ' ms</td></tr>' +
    '<tr><td>평균</td><td>' + avg.toFixed(1) + ' ms</td></tr>' +
    '<tr><td>최소 / 최대</td><td>' + min + ' / ' + max + ' ms</td></tr>' +
    '</table>' +
    '<div class="qtitle" style="margin-top:8px">net — 전송 (시계 교차)</div>' +
    '<table>' +
    '<tr><td>net</td><td>' + netTxt + '</td></tr>' +
    '<tr><td>total</td><td>' + totTxt + '</td></tr>' +
    '</table>' +
    '<div class="qtitle" style="margin-top:8px">처리량</div>' +
    '<table>' +
    '<tr><td>수신 fps</td><td>' + fpsTxt + '</td></tr>' +
    '<tr><td>det (검출만)</td><td>' + detTxt + '</td></tr>' +
    '<tr><td>검출률</td><td>' + hitTxt + '</td></tr>' +
    '<tr><td>seq 누락</td><td>' + ov.seqGaps + ' 프레임 (이 채널만)</td></tr>' +
    '<tr><td>카메라 CPU</td><td>' + cpuText() + '</td></tr>' +
    '</table>';
}

// 한 프레임에 마커가 여러 개면 CAM_POSE 줄도 여러 개 온다. 줄마다 그리면 한 번의
// 시각적 갱신을 위해 DOM/캔버스를 N번 다시 만들게 되고, 화면이 깜빡인다.
//
// 두 가지로 막는다.
//  (1) 이중 버퍼 -- 수집은 rawBuilding 에 하고 프레임이 끝날 때 한 번에 교체한다.
//      예전처럼 프레임 시작에서 rawFrame 을 비우면 "마커 0개 -> 1개 -> 2개" 중간
//      상태가 그대로 그려져 깜빡임의 주된 원인이 된다. 채널별로 독립 —
//      chOverlay[ch]에 각자의 rawBuilding/rawFrame/rawSeq가 있다.
//  (2) rAF 가드 -- 브라우저가 그릴 수 있는 속도보다 프레임이 빨리 와도 여러 프레임을
//      한 번의 페인트로 합친다. 데이터는 잃지 않는다(수집은 매 줄 동기적으로 계속됨).
let rawRenderScheduled = false;
function scheduleRawRender() {
  if (rawRenderScheduled) return;
  rawRenderScheduled = true;
  requestAnimationFrame(() => {
    rawRenderScheduled = false;
    if (rawOn) renderRaw();
    if (rawOverlayOn) redrawRawCanvas();
    renderRawLatency();
  });
}

function handleRaw(line) {
  // rawFrame is shared with the homography overlay, so parse regardless of the
  // marker-detection tab's toggle; only gate the DOM renders below.
  //
  // ch 없는 줄(옛 로그, 다른 프로토콜)은 무시한다 -- 채널을 모르면 어느
  // chOverlay 버킷에 넣을지 알 수 없고, 잘못 추측하면 다른 렌즈의 코너와
  // 섞인다.
  const sm = line.match(/seq=(\\d+) ch=(\\d+)/);
  if (!sm) return;
  const ch = Number(sm[2]);
  const st = chOverlay[ch];
  if (!st) return;
  const lost = line.includes('MARKER LOST');
  const cm = line.match(/id=(\\d+)\\s+c0=\\(([\\d.]+),([\\d.]+)\\)\\s+c1=\\(([\\d.]+),([\\d.]+)\\)\\s+c2=\\(([\\d.]+),([\\d.]+)\\)\\s+c3=\\(([\\d.]+),([\\d.]+)\\)/);
  if (!cm && !lost) return;            // not a per-frame pose line
  // seq 가 바뀌면 "이전" 프레임이 완성된 것이다: 그것을 게시하고 새 수집을 시작한다.
  // 채널마다 자기 seq_[ch]를 따로 세므로(ArucoPosePNM), 이 판정도 채널별로
  // 해야 한다 -- 전역 seq 하나로 하면 채널이 섞일 때 프레임 경계를 잘못
  // 판단해서(다른 렌즈의 seq가 우연히 같거나 달라서) 코너가 섞인 프레임이
  // 만들어질 수 있었다.
  if (sm[1] !== st.rawSeq) {
    st.rawSeq = sm[1];
    st.rawFrame = st.rawBuilding;
    st.rawBuilding = {};
    if (ch === curCh()) scheduleRawRender();  // 선택된 채널일 때만 다시 그림
  }
  if (cm) st.rawBuilding[cm[1]] = [[cm[2], cm[3]], [cm[4], cm[5]], [cm[6], cm[7]], [cm[8], cm[9]]];
}

// CALIB_K_QUERY 결과를 표로 렌더 (fx/fy/cx/cy + 왜곡계수 k1,k2,p1,p2,k3)
function renderKQuery(available, fx, fy, cx, cy, dist) {
  const box = document.getElementById('kquery');
  if (!available) {
    box.innerHTML = `<span class="none">CH${curCh() + 1}: 카메라에 로드된 캘리브레이션 없음</span>`;
    return;
  }
  const dl = ['k1 (r²)', 'k2 (r⁴)', 'p1 (tangential)', 'p2 (tangential)', 'k3 (r⁶)'];
  let rows =
    `<tr><td>fx</td><td>${fx}</td></tr>` +
    `<tr><td>fy</td><td>${fy}</td></tr>` +
    `<tr><td>cx</td><td>${cx}</td></tr>` +
    `<tr><td>cy</td><td>${cy}</td></tr>`;
  for (let i = 0; i < dist.length; i++)
    rows += `<tr><td>${dl[i] || 'd' + i}</td><td>${dist[i]}</td></tr>`;
  box.innerHTML = `<div class="qtitle">CH${curCh() + 1} 현재 로드된 K / 왜곡계수</div><table>${rows}</table>`;
}

// ===== 캘리브레이션 탭 — 보드 코너 시각화 (2026-08-11) =====
// 마커검출 탭의 raw 오버레이와 같은 원리(사진 배경 없음, app_config.h의
// 의도적 선택 그대로 — 좌표만). CALIB_K_PROBE는 카메라가 CALIB_PROBE_MS마다
// held 코너를 채널별로 보내는 새 리포트라, 세션이 열려 있을 때만 값이 온다.
function emptyChCalibProbe() { return { total: 0, points: [] }; }
let chCalibProbe = {0: emptyChCalibProbe(), 1: emptyChCalibProbe(), 2: emptyChCalibProbe(), 3: emptyChCalibProbe()};
function handleCalibKProbe(line) {
  const m = line.match(/^\\[calib\\] ch=(\\d+) PROBE total=(-?\\d+) (\\[.*\\])$/);
  if (!m) return;
  const ch = Number(m[1]);
  let pts;
  try { pts = JSON.parse(m[3]); } catch (_) { return; }
  if (!Array.isArray(pts)) return;
  chCalibProbe[ch] = { total: Number(m[2]), points: pts.map(p => ({x: p[0], y: p[1], id: p[2]})) };
  if (ch === curCh()) renderCalibProbeCanvas();
}
const calibProbeCanvas = document.getElementById('calibProbeCanvas');
const calibProbeCtx = calibProbeCanvas ? calibProbeCanvas.getContext('2d') : null;
function renderCalibProbeCanvas() {
  if (!calibProbeCtx) return;
  // 카메라 raw 해상도 기준(코너가 풀프레임 픽셀 좌표라 캔버스가 프레임과
  // 같은 크기여야 위치가 맞는다) — 마커검출 탭의 #camRes 선택을 그대로 쓴다.
  const res = camRes();
  const W = res.w, H = res.h;
  if (calibProbeCanvas.width !== W || calibProbeCanvas.height !== H) {
    calibProbeCanvas.width = W; calibProbeCanvas.height = H;
  }
  calibProbeCtx.clearRect(0, 0, W, H);
  const st = chCalibProbe[curCh()] || emptyChCalibProbe();
  const statusEl = document.getElementById('calibProbeStatus');
  if (!st.points.length) {
    if (statusEl) {
      statusEl.textContent = st.total
        ? `CH${curCh() + 1} · 코너 0/${st.total} — 보드가 안 보이거나 세션이 열려 있지 않습니다`
        : '세션을 시작하면 잡힌 코너 위치가 여기 표시됩니다.';
    }
    return;
  }
  calibProbeCtx.font = '16px sans-serif';
  calibProbeCtx.fillStyle = '#22c55e';
  for (const p of st.points) {
    calibProbeCtx.beginPath();
    calibProbeCtx.arc(p.x, p.y, 6, 0, 7);
    calibProbeCtx.fill();
  }
  calibProbeCtx.fillText(`코너 ${st.points.length}/${st.total}`, 12, 24);
  if (statusEl)
    statusEl.textContent = `CH${curCh() + 1} · 코너 ${st.points.length}/${st.total} ` +
      `(최근 HOLD_MS 안에 잡힌 것 포함 — 순간 프레임 하나만 보는 것보다 안정적으로 보임)`;
}

// ===== 모드 탭 전환 (기존 기능은 그대로, 표시만 토글) =====
// Table-driven rather than boolean pairs: with three panes the old
// calib/not-calib flag no longer distinguishes them.
const TABS = {
  calib:      {pane: 'groups',         tab: 'tabCalib'},
  homography: {pane: 'homographyPane', tab: 'tabHmg'},
  raw:        {pane: 'rawPane',        tab: 'tabRaw'},
  ops:        {pane: 'opsPane',        tab: 'tabOps'},
  shell:      {pane: 'shellPane',      tab: 'tabShell'},
  central:    {pane: 'centralPane',    tab: 'tabCentral'},
};
function showTab(name) {
  for (const k in TABS) {
    document.getElementById(TABS[k].pane).style.display = (k === name) ? 'flex' : 'none';
    document.getElementById(TABS[k].tab).classList.toggle('active', k === name);
  }
  if (name === 'shell') document.getElementById('shInput').focus();
  if (name === 'central') { send('CENTRAL_QUERY'); fillCentralPosSample(); }
  if (name === 'raw') send('DYNROI');
  if (name === 'homography') {
    applyHgCoordDefault();
    sendCh('HG_QUERY');   // HG_FIT/HG_FIT_PT도 같이 돌아온다 -- 정확도(LOO) 표시까지 여기서 채워진다
    sendCh('MARKER_PLANE_QUERY');
    sendCh('ANCHOR_QUERY');
  }
}

// ===== 캘리브레이션 대상 채널 (프로토콜 v0.4) =====
// 채널마다 렌즈 방향이 달라 K/D/H 가 전부 다르다. 여기서 고른 채널로 캘리 결과가
// 저장되고(H_MATRIX.ch), 카메라에도 SELECT_CHANNEL 이 나가 그 채널을 보게 된다.
// RP_CAM_CHANNELS=1 (PNO 단일 채널) 이면 선택 UI 자체가 뜨지 않는다.
// 마지막으로 syncCalibFromChannel() 등을 돌린 채널. renderCalibChannel()의
// "바뀐 경우에만 다시 그린다" 판단을 DOM(sel.value) 대신 이걸로 한다 --
// <select onchange>는 브라우저가 핸들러를 부르기 *전에* 이미 sel.value를 새
// 값으로 바꿔 놓으므로, sel.value와 비교하면 사용자가 드롭다운을 직접
// 바꾼 경우 항상 changed=false가 되어 방금 고른 채널로 절대 안 바뀌는
// 버그였다(다른 탭에 갔다 오면 그 탭이 curCh()로 따로 재조회를 하니 그제야
// 반영된 것처럼 보였을 뿐). 2026-08-11 사용자 보고로 발견.
let lastSyncedCalibCh = null;
function renderCalibChannel(st) {
  const box = document.getElementById('chBox');
  if (!st || st.count <= 1) { box.style.display = 'none'; return; }
  box.style.display = '';
  const sel = document.getElementById('calibCh');
  if (sel.options.length !== st.count) {
    sel.innerHTML = '';
    for (const c of st.channels) {
      const o = document.createElement('option');
      o.value = c.ch;
      o.textContent = 'CH' + c.ch;
      sel.appendChild(o);
    }
  }
  // 채널이 실제로 바뀐 경우에만 오버레이를 다시 그린다 -- refreshCalibChannel()이
  // 5초마다 이 함수를 부르는데, 매번 다시 그리면 안 바뀐 채널도 깜빡인다.
  const changed = lastSyncedCalibCh !== st.channel;
  sel.value = st.channel;
  if (changed) {
    lastSyncedCalibCh = st.channel;
    // chOverlay는 이미 채널별로 쌓여 있으니(다음 패킷을 기다릴 필요 없이) 즉시
    // 새 채널 걸로 다시 그린다. kCalib(K/dist)도 이제 채널별이라 syncCalibFromChannel()
    // 안에서 같이 동기화된다(2026-08-10).
    if (rawOn) renderRaw();
    if (rawOverlayOn) redrawRawCanvas();
    renderDynRoiState();
    renderRawLatency();  // proc/fps/검출률/seq 누락도 채널별이니 즉시 다시 그림
    syncCalibFromChannel();
  }
  // 어느 채널이 아직 안 끝났는지 한눈에. H 가 없으면 카메라가 번들을 조립하지
  // 못하므로(SendFloorBundle 이 "바닥 H 없음"으로 거부) 그 채널은 Qt 에서
  // 여전히 "캘리브레이션 없음"으로 보인다.
  const left = st.channels.filter(c => !c.has_h).map(c => 'CH' + c.ch);
  document.getElementById('chHint').textContent =
    left.length ? ('미완료: ' + left.join(' ')) : '전 채널 완료';
}

function refreshCalibChannel() {
  fetch('/calib/channel').then(r => r.json()).then(renderCalibChannel).catch(() => {});
}

function setCalibChannel(ch) {
  fetch('/calib/channel', {method: 'POST', body: JSON.stringify({ch: Number(ch)})})
    .then(r => r.json())
    .then(res => { if (res.ok) renderCalibChannel(res.status); else alert(res.reason || '채널 변경 실패'); })
    .catch(() => alert('채널 변경 실패 (서버 응답 없음)'));
}
refreshCalibChannel();
// 캘리가 끝나 캐시가 채워지면 "미완료" 표시가 줄어야 한다. 폴링이 가장 단순하고,
// 5초면 사람이 다음 채널로 넘어가기 전에 갱신된다.
setInterval(refreshCalibChannel, 5000);

// ===== Homography workflow sections =====
function showHgSection(section) {
  document.querySelectorAll('[data-hg-section]').forEach(panel => {
    panel.style.display = panel.dataset.hgSection === section ? 'block' : 'none';
  });
  // 'advanced' 는 버튼이 없어져서(2026-08-12) 여기에도 없다 — 그 패널은 어떤 섹션으로도
  // 선택되지 않아 계속 숨겨진 상태로 남는다.
  const map = {compute: 'hgSubCompute', odom: 'hgSubOdom', register: 'hgSubRegister'};
  for (const name in map) {
    const b = document.getElementById(map[name]);
    if (b) b.classList.toggle('active', name === section);
  }
  // 배치도는 채널 선택을 CALIB_START 미리보기에 싣는다. 탭을 열 때 다시 그려야
  // 숨어 있는 동안 바뀐 채널이 반영된다 (이 함수는 renderOdoLayout 보다 위에
  // 정의되지만, 호출은 스크립트가 다 로드된 뒤라 hoisting 문제가 없다).
  if (section === 'odom' && typeof renderOdoLayout === 'function') {
    renderOdoLayout();
    // 지금 로봇 좌표가 어느 슬롯으로 나가는지는 화면을 열었을 때 바로 보여야
    // 한다 — 조회를 눌러야만 알 수 있으면 안 누른 사람은 계속 모른다.
    if (typeof sendCh === 'function') sendCh('ODOM_PREFER_QUERY');
  }
  if (section === 'register' && typeof registerPopulateChannelSelects === 'function')
    registerPopulateChannelSelects();
}
showHgSection('compute');

// ===== Shell 탭 =====
const shOut = document.getElementById('shOut');
const shInput = document.getElementById('shInput');
let shHistory = [], shPos = -1;

function runShell(c) {
  showTab('shell');
  shellAppend('$ ' + c + "\\n", 'term-echo');
  send('SHELL ' + c);
}
function sendShell() {
  const c = shInput.value.trim();
  if (!c) return;
  shellAppend('$ ' + c + "\\n", 'term-echo');
  send('SHELL ' + c);
  shHistory.push(c);
  shPos = shHistory.length;
  shInput.value = '';
}
function copyShellOutput() {
  const value = shOut.textContent;
  if (!value) return;
  navigator.clipboard?.writeText(value).catch(() => {});
}
shInput.addEventListener('keydown', (e) => {
  if (e.key === 'Enter') { sendShell(); }
  else if (e.key === 'ArrowUp') {
    e.preventDefault();
    if (shPos > 0) { shPos--; shInput.value = shHistory[shPos] || ''; }
  } else if (e.key === 'ArrowDown') {
    e.preventDefault();
    if (shPos < shHistory.length - 1) { shPos++; shInput.value = shHistory[shPos]; }
    else { shPos = shHistory.length; shInput.value = ''; }
  } else if (e.key === 'Escape') { shInput.value = ''; }
});

// Route [shell] lines into the terminal pane. They also stay in the main log --
// the pane is a filtered view, not a redirect, so nothing is hidden from the
// transcript.
function shellAtBottom() {
  return shOut.scrollHeight - shOut.scrollTop - shOut.clientHeight < 40;
}
function shellAppend(text, cls) {
  const stick = shellAtBottom();
  if (cls) {
    const el = document.createElement('span');
    el.className = cls;
    el.textContent = text;
    shOut.appendChild(el);
  } else {
    // 카메라 출력은 임의 문자열이다 -- 텍스트 노드로만 붙여 마크업으로 해석될
    // 여지를 없앤다.
    shOut.appendChild(document.createTextNode(text));
  }
  if (stick) shOut.scrollTop = shOut.scrollHeight;
}
function clearShellOutput() { shOut.textContent = ''; }
function focusShellInput(e) {
  // 드래그로 출력을 선택하는 중이면 가로채지 않는다.
  if (window.getSelection && String(window.getSelection()).length) return;
  const el = document.getElementById('shInput');
  if (el) el.focus();
}
function handleShell(line) {
  if (!line.startsWith('[shell]')) return;
  shellAppend(line.slice(8) + "\\n");
}

// ===== Homography Test 탭 =====
// 기본 배치 = 폼보드 1장(900×600), 마커 중심은 보드 모서리에서 60mm 안쪽.
// HG_PRESETS['1x1'] 과 같은 값이어야 한다 — 한쪽만 고치면 첫 화면과 프리셋이
// 어긋나 어느 쪽이 실제 배치인지 알 수 없게 된다.
const defaultHgAnchors = [
  {id: 0, wx: 60, wy: 60}, {id: 1, wx: 840, wy: 60},
  {id: 2, wx: 60, wy: 540}, {id: 3, wx: 840, wy: 540},
];
let hgAnchorEntries = defaultHgAnchors.map(a => ({...a}));
const hgAnchorRows = document.getElementById('hgAnchorRows');
const hgAnchorEditStatus = document.getElementById('hgAnchorEditStatus');
function renderHgAnchorRows(anchors) {
  if (anchors) hgAnchorEntries = anchors.map(a => ({id:Number(a.id), wx:Number(a.wx), wy:Number(a.wy)}));
  const list = hgAnchorEntries;
  if (!list.length) { hgAnchorRows.innerHTML = '<span class="none">등록된 계산 앵커 없음</span>'; return; }
  let html = '<table><tr><td>ID</td><td>X mm</td><td>Y mm</td></tr>';
  for (let slot = 0; slot < list.length; slot++) {
    const a = list[slot];
    html += `<tr><td><input id="hgAnchorId${slot}" type="number" min="0" step="1" value="${Number(a.id)}" style="width:58px" oninput="syncHgTablesToMap()"></td>` +
      `<td><input id="hgAnchorX${slot}" type="number" step="0.1" value="${Number(a.wx)}" style="width:92px" oninput="syncHgTablesToMap()"></td>` +
      `<td><input id="hgAnchorY${slot}" type="number" step="0.1" value="${Number(a.wy)}" style="width:92px" oninput="syncHgTablesToMap()"></td>` +
      `<td><button type="button" onclick="removeHgAnchorRow(${slot})">삭제</button></td></tr>`;
  }
  hgAnchorRows.innerHTML = html + '</table>';
  renderHgLayoutMap();
  const b = document.getElementById('hgCalibStartBtn');
  if (b) b.innerHTML = `앵커 ${list.length}개로 H 계산<span class="cmd">CALIB_START</span>`;
}
function queryHgAnchors() {
  hgAnchorEditStatus.textContent = '카메라의 현재 앵커 좌표를 조회하는 중…';
  sendCh('ANCHOR_QUERY');
}
function readHgAnchorRows() {
  return hgAnchorEntries.map((_, slot) => ({
    slot, id: Number(document.getElementById('hgAnchorId' + slot).value),
    wx: Number(document.getElementById('hgAnchorX' + slot).value),
    wy: Number(document.getElementById('hgAnchorY' + slot).value),
  }));
}
function addHgAnchorRow() {
  hgAnchorEntries = readHgAnchorRows();
  if (hgAnchorEntries.length >= 16) { hgAnchorEditStatus.textContent = '계산 앵커는 최대 16개입니다.'; return; }
  const used = new Set(hgAnchorEntries.map(a => a.id)); let id = 10; while (used.has(id)) ++id;
  hgAnchorEntries.push({id, wx: 0, wy: 0}); renderHgAnchorRows();
}
function removeHgAnchorRow(slot) {
  hgAnchorEntries = readHgAnchorRows();
  if (hgAnchorEntries.length <= 4) { hgAnchorEditStatus.textContent = '호모그래피 계산에는 앵커가 최소 4개 필요합니다.'; return; }
  hgAnchorEntries.splice(slot, 1); renderHgAnchorRows();
}
// 배치 프리셋. 좌표는 실측이며 ID는 위치가 정한다 (id = 행 × 전체열수 + 열,
// 행 0 = 아래, 열 0 = 왼쪽). 폼보드 nc×nr 이면 마커 격자는 2nc × 2nr.
//
// 마커 중심이 판 모서리에서 60 mm 안쪽인 이유: 마커를 네 모서리에 딱 맞게(흰 테두리
// 바깥이 모서리에 정렬) 붙이는데, 호모그래피가 쓰는 기준은 검정 사각형 중심이라
// 흰색/2 = 60 mm 만큼 들어온다. 가운데 열·행이 120 mm 간격인 것도 같은 이유로
// 인접 판이 각각 60 mm씩이라 60+60 이다.
const HG_PRESETS = {
  '1x1': {
    label: '폼보드 1장 (900×600)',
    anchors: [
      {id: 0, wx: 60, wy: 60}, {id: 1, wx: 840, wy: 60},
      {id: 2, wx: 60, wy: 540}, {id: 3, wx: 840, wy: 540},
    ],
    validation: [],
  },
  '2x2': {
    // 16점 전량을 앵커로 쓴다. 가운데 4점(5·6·9·10)은 영상 중앙부라 렌즈 왜곡 잔차가
    // 가장 크게 드러나는 위치이므로, 피팅에 넣어야 화면 중앙 정확도가 올라간다.
    // 그 대신 피팅 밖 검증점이 없어지므로, 일반화 오차가 필요하면 앵커를 일부만
    // 남긴 hold-out 을 따로 돌려야 한다.
    label: '폼보드 2×2 (1800×1200)',
    anchors: [
      {id: 0,  wx: 60,   wy: 60},   {id: 1,  wx: 840,  wy: 60},
      {id: 2,  wx: 960,  wy: 60},   {id: 3,  wx: 1740, wy: 60},
      {id: 4,  wx: 60,   wy: 540},  {id: 5,  wx: 840,  wy: 540},
      {id: 6,  wx: 960,  wy: 540},  {id: 7,  wx: 1740, wy: 540},
      {id: 8,  wx: 60,   wy: 660},  {id: 9,  wx: 840,  wy: 660},
      {id: 10, wx: 960,  wy: 660},  {id: 11, wx: 1740, wy: 660},
      {id: 12, wx: 60,   wy: 1140}, {id: 13, wx: 840,  wy: 1140},
      {id: 14, wx: 960,  wy: 1140}, {id: 15, wx: 1740, wy: 1140},
    ],
    validation: [],
  },
};

// 표를 채우기만 한다. 카메라로는 아무것도 보내지 않는다 — 조작자가 눈으로 확인한 뒤
// [입력값 모두 적용]을 눌러야 반영된다. 잘못된 배치를 고른 채 자동 전송되면 다음
// CALIB_START 가 조용히 엉뚱한 좌표로 피팅한다.
function applyHgPreset() {
  const key = document.getElementById('hgPreset').value;
  const st = document.getElementById('hgPresetStatus');
  const p = HG_PRESETS[key];
  if (!p) { if (st) st.textContent = '배치를 고르세요.'; return; }

  // 두 배열을 먼저 다 바꾼 뒤 그린다. 렌더 함수들은 목록이 비면 조기 반환하면서
  // 배치도 갱신도 건너뛰므로, 배치도는 마지막에 한 번 직접 다시 그린다 —
  // 그러지 않으면 검증점을 비우는 프리셋에서 옛 주황 점이 그림에 남는다.
  hgAnchorEntries = p.anchors.map(a => ({...a}));
  hgValidationEntries = p.validation.map(a => ({...a}));
  renderHgAnchorRows();
  renderHgValidationRows();
  renderHgLayoutMap();

  if (st) st.innerHTML =
    `<b>${p.label}</b> — 앵커 ${p.anchors.length} / 검증 ${p.validation.length}. ` +
    '표만 채웠습니다. <b>[입력값 모두 적용]</b>을 눌러야 카메라에 반영됩니다.';
}

function applyHgAnchors() {
  const values = readHgAnchorRows();
  for (const a of values) {
    const {slot, id, wx, wy} = a;
    if (!Number.isInteger(id) || id < 0 || !Number.isFinite(wx) || !Number.isFinite(wy)) {
      hgAnchorEditStatus.textContent = `${slot + 1}번 앵커의 ID/X/Y mm 값을 확인하세요.`;
      return;
    }
  }
  if (values.length < 4 || values.length > 16) {
    hgAnchorEditStatus.textContent = `앵커는 4~16개여야 합니다 (현재 ${values.length}개).`;
    return;
  }
  if (new Set(values.map(a => a.id)).size !== values.length) {
    hgAnchorEditStatus.textContent = '앵커 ID는 모두 달라야 합니다.';
    return;
  }
  if (!confirm(`입력한 ${values.length}개 앵커를 카메라에 적용할까요? 진행 중인 앵커 캘리브레이션은 먼저 끝내야 합니다.`)) return;
  hgAnchorEntries = values;
  hgAnchorEditStatus.textContent = `${values.length}개 앵커 좌표를 카메라에 적용하는 중…`;
  // ANCHOR_SET_ALL 로 한 번에 교체한다. 슬롯 단위(ANCHOR_SET_SLOT)는 0..7 여덟 칸
  // 고정이라 16점 배치를 넣을 수 없고, 여러 명령으로 쪼개면 중간에 하나가 실패했을 때
  // 표와 카메라가 어긋난 채로 남는다 - 그 상태로 CALIB_START 하면 조용히 틀린 H가 나온다.
  // 카메라는 이 명령 하나에 대해 완성된 앵커 표를 되돌려준다.
  //
  // ArucoPosePNM의 실제 형식은 "ANCHOR_SET_ALL <ch> <count> <id> <wx> <wy> ...".
  // <ch>/<count>가 빠지면 첫 앵커의 id가 채널로, wx가 count로 잘못 읽혀
  // 완전히 다른(그리고 조용히 틀린) 결과가 나간다 — 2026-08-10 발견.
  send('ANCHOR_SET_ALL ' + chArg() + ' ' + values.length + ' ' +
       values.map(a => `${a.id} ${a.wx} ${a.wy}`).join(' '));
}
// 카메라가 마지막으로 보고한 앵커. 중앙서버 번들의 canvas_mm 추정이 이 값을 쓴다
// (표의 입력값이 아니라 카메라가 실제로 들고 있는 값이어야 한다).
let hgCameraAnchors = null;

function handleHgAnchors(line) {
  // "[calib] ch=N ANCHORS [...]" -- ch= 앞에 붙는다 (2026-08-10, 다른
  // 채널별 핸들러들과 동일한 자리).
  const m = line.match(/^\\[calib\\] ch=(\\d+) ANCHORS (\\[.*\\])$/);
  if (!m) return;
  const ch = Number(m[1]);
  try {
    const anchors = JSON.parse(m[2]);
    if (Array.isArray(anchors) && anchors.length) {
      const parsed = anchors.map(a => ({id:Number(a.id), wx:Number(a.wx), wy:Number(a.wy)}));
      chHg[ch].anchors = parsed;
      if (ch === curCh()) {
        hgCameraAnchors = parsed;
        renderHgAnchorRows(anchors);
        hgAnchorEditStatus.textContent = '카메라의 현재 앵커 좌표를 표시했습니다.';
      }
    }
  } catch (_) { /* leave the editable values intact on a malformed reply */ }
}
renderHgAnchorRows(defaultHgAnchors);

// 1x1 프리셋과 동일하게 비워 둔다. 검증점을 쓰려면 표에서 직접 추가한다.
const defaultHgValidation = [];
let hgValidationEntries = defaultHgValidation.map(a => ({...a}));
const hgValidationRows = document.getElementById('hgValidationRows');
const hgValidationEditStatus = document.getElementById('hgValidationEditStatus');
function renderHgValidationRows() {
  if (!hgValidationEntries.length) {
    hgValidationRows.innerHTML = '<span class="none">등록된 검증 기준점 없음</span>';
    return;
  }
  let html = '<table><tr><td>ID</td><td>X mm</td><td>Y mm</td><td></td></tr>';
  hgValidationEntries.forEach((a, i) => {
    html += `<tr><td><input id="hgValId${i}" type="number" min="0" step="1" value="${Number(a.id)}" style="width:58px" oninput="syncHgTablesToMap()"></td>` +
      `<td><input id="hgValX${i}" type="number" step="0.1" value="${Number(a.wx)}" style="width:92px" oninput="syncHgTablesToMap()"></td>` +
      `<td><input id="hgValY${i}" type="number" step="0.1" value="${Number(a.wy)}" style="width:92px" oninput="syncHgTablesToMap()"></td>` +
      `<td><button type="button" onclick="removeHgValidationRow(${i})">삭제</button></td></tr>`;
  });
  hgValidationRows.innerHTML = html + '</table>';
  renderHgLayoutMap();
}
function readHgValidationRows() {
  return hgValidationEntries.map((_, i) => ({
    id: Number(document.getElementById('hgValId' + i).value),
    wx: Number(document.getElementById('hgValX' + i).value),
    wy: Number(document.getElementById('hgValY' + i).value),
  }));
}
function addHgValidationRow() {
  hgValidationEntries = readHgValidationRows();
  if (hgValidationEntries.length >= 16) {
    hgValidationEditStatus.textContent = '검증 기준점은 최대 16개입니다.';
    return;
  }
  const used = new Set(hgValidationEntries.map(a => Number(a.id)));
  let id = 20; while (used.has(id)) ++id;
  hgValidationEntries.push({id, wx: 0, wy: 0});
  renderHgValidationRows();
}
function removeHgValidationRow(index) {
  hgValidationEntries = readHgValidationRows();
  hgValidationEntries.splice(index, 1);
  renderHgValidationRows();
}
// ArucoPosePNM에는 VALIDATION_QUERY/VALIDATION_SET이 없다 -- cctv_app 시절의
// "계산에서 뺀 별도 검증점" 개념 자체가 없고, 대신 계산에 쓴 앵커마다 LOO(그
// 점만 빼고 다시 피팅)로 정확도를 낸다(homography_mapper.h 설계 코멘트).
// 그래서 조회는 HG_QUERY 하나로 충분하고(카메라가 HG_FIT/HG_FIT_PT를 같이
// 보내온다), "적용" 버튼은 보낼 대상이 없어 뺐다 (2026-08-10).
function queryHgFitAccuracy() {
  hgValidationEditStatus.style.whiteSpace = 'pre-line';
  hgValidationEditStatus.textContent = '카메라의 호모그래피 정확도(LOO)를 조회하는 중…';
  sendCh('HG_QUERY');
}
renderHgValidationRows();

// ===== 앵커·검증점 배치도 (드래그로 바닥 좌표 편집) =====
// hgAnchorEntries(파랑)·hgValidationEntries(주황)를 하나의 작업영역 평면에 그린다.
// 점을 끌면 해당 항목의 wx/wy와 표 입력값이 함께 갱신되고, 표를 직접 고치면
// oninput→syncHgTablesToMap로 배치도가 다시 그려진다.
//
// 표의 숫자만으로는 배치가 맞는지 알 수 없다 — 폼보드를 뒤집어 깔았거나 두 판의
// 좌우가 바뀌었을 때, 좌표는 그럴듯한데 실제 바닥과 어긋난다. 그림으로 보면 바로 잡힌다.
const HG_VBW = 480, HG_VBH = 360, HG_MARGIN = 40;
let hgDragBounds = null;   // 드래그 중에는 축척을 고정해 지도가 출렁이지 않게 한다
let hgDrag = null;         // {svg, kind:'a'|'v', idx}

function hgNiceStep(range) {
  const raw = Math.max(range, 1) / 5;
  const mag = Math.pow(10, Math.floor(Math.log10(raw)));
  const n = raw / mag;
  return (n < 1.5 ? 1 : n < 3 ? 2 : n < 7 ? 5 : 10) * mag;
}
function hgMapBounds() {
  const pts = hgAnchorEntries.concat(hgValidationEntries);
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  for (const p of pts) {
    minX = Math.min(minX, p.wx); maxX = Math.max(maxX, p.wx);
    minY = Math.min(minY, p.wy); maxY = Math.max(maxY, p.wy);
  }
  if (!isFinite(minX)) { minX = 0; maxX = 1000; minY = 0; maxY = 1000; }
  if (minX === maxX) { minX -= 500; maxX += 500; }
  if (minY === maxY) { minY -= 500; maxY += 500; }
  const padX = (maxX - minX) * 0.12, padY = (maxY - minY) * 0.12;
  return {minX: minX - padX, maxX: maxX + padX, minY: minY - padY, maxY: maxY + padY};
}
// 월드 mm -> SVG 좌표. y를 뒤집는다: 월드는 위가 +Y, 화면은 아래가 +y.
function hgW2S(wx, wy, b) {
  return [
    HG_MARGIN + (wx - b.minX) / (b.maxX - b.minX) * (HG_VBW - 2 * HG_MARGIN),
    (HG_VBH - HG_MARGIN) - (wy - b.minY) / (b.maxY - b.minY) * (HG_VBH - 2 * HG_MARGIN),
  ];
}
function hgS2W(sx, sy, b) {
  return [
    b.minX + (sx - HG_MARGIN) / (HG_VBW - 2 * HG_MARGIN) * (b.maxX - b.minX),
    b.minY + ((HG_VBH - HG_MARGIN) - sy) / (HG_VBH - 2 * HG_MARGIN) * (b.maxY - b.minY),
  ];
}
function renderHgLayoutMap() {
  if (!window.__hgMapReady) return;   // 배열 초기화 전 렌더 방지 (TDZ 회피)
  const maps = document.querySelectorAll('.hg-map');
  if (!maps.length) return;
  const b = hgDragBounds || hgMapBounds();
  let g = '';
  const stepX = hgNiceStep(b.maxX - b.minX), stepY = hgNiceStep(b.maxY - b.minY);
  for (let x = Math.ceil(b.minX / stepX) * stepX; x <= b.maxX; x += stepX) {
    const top = hgW2S(x, b.maxY, b), bot = hgW2S(x, b.minY, b);
    g += `<line x1="${bot[0].toFixed(1)}" y1="${bot[1].toFixed(1)}" x2="${top[0].toFixed(1)}" y2="${top[1].toFixed(1)}" stroke="#8888" stroke-width="0.5"/>`;
    g += `<text x="${bot[0].toFixed(1)}" y="${(HG_VBH - HG_MARGIN + 14).toFixed(1)}" font-size="9" fill="#999" text-anchor="middle">${Math.round(x)}</text>`;
  }
  for (let y = Math.ceil(b.minY / stepY) * stepY; y <= b.maxY; y += stepY) {
    const l = hgW2S(b.minX, y, b), r = hgW2S(b.maxX, y, b);
    g += `<line x1="${l[0].toFixed(1)}" y1="${l[1].toFixed(1)}" x2="${r[0].toFixed(1)}" y2="${r[1].toFixed(1)}" stroke="#8888" stroke-width="0.5"/>`;
    g += `<text x="${(HG_MARGIN - 6).toFixed(1)}" y="${(l[1] + 3).toFixed(1)}" font-size="9" fill="#999" text-anchor="end">${Math.round(y)}</text>`;
  }
  if (hgAnchorEntries.length >= 3) {
    const poly = hgAnchorEntries.map(a => hgW2S(a.wx, a.wy, b).map(v => v.toFixed(1)).join(',')).join(' ');
    g += `<polygon points="${poly}" fill="rgba(59,130,246,0.08)" stroke="rgba(59,130,246,0.5)" stroke-width="1"/>`;
  }
  const dot = (a, i, kind, r, fill, fs) => {
    const [px, py] = hgW2S(a.wx, a.wy, b);
    return `<g class="hg-pt" data-kind="${kind}" data-idx="${i}" style="cursor:grab">` +
      `<circle cx="${px.toFixed(1)}" cy="${py.toFixed(1)}" r="${r}" fill="${fill}" stroke="#fff" stroke-width="1.5"/>` +
      `<text x="${px.toFixed(1)}" y="${(py + 3).toFixed(1)}" font-size="${fs}" font-weight="700" fill="#fff" text-anchor="middle" pointer-events="none">${a.id}</text></g>`;
  };
  hgAnchorEntries.forEach((a, i) => { g += dot(a, i, 'a', 8, '#3b82f6', 9); });
  hgValidationEntries.forEach((a, i) => { g += dot(a, i, 'v', 7, '#f59e0b', 8.5); });
  maps.forEach(svg => { svg.innerHTML = g; });
}
function syncHgTablesToMap() {
  if (document.getElementById('hgAnchorId0')) hgAnchorEntries = readHgAnchorRows();
  if (document.getElementById('hgValId0')) hgValidationEntries = readHgValidationRows();
  renderHgLayoutMap();
}
function hgMapDown(e) {
  const pt = e.target.closest('.hg-pt');
  if (!pt) return;
  e.preventDefault();
  hgDragBounds = hgMapBounds();   // 이 드래그 동안 축척 고정
  hgDrag = {svg: e.currentTarget, kind: pt.dataset.kind, idx: Number(pt.dataset.idx)};
  window.addEventListener('pointermove', hgMapMove);
  window.addEventListener('pointerup', hgMapUp);
}
function hgMapMove(e) {
  if (!hgDrag) return;
  const svg = hgDrag.svg, ctm = svg.getScreenCTM();
  if (!ctm) return;
  const p = svg.createSVGPoint(); p.x = e.clientX; p.y = e.clientY;
  const loc = p.matrixTransform(ctm.inverse());
  const w = hgS2W(loc.x, loc.y, hgDragBounds);
  const wx = Math.round(w[0]), wy = Math.round(w[1]);
  const arr = hgDrag.kind === 'a' ? hgAnchorEntries : hgValidationEntries;
  const entry = arr[hgDrag.idx];
  if (!entry) return;
  entry.wx = wx; entry.wy = wy;
  const pre = hgDrag.kind === 'a' ? 'hgAnchor' : 'hgVal';
  const xi = document.getElementById(pre + 'X' + hgDrag.idx);
  const yi = document.getElementById(pre + 'Y' + hgDrag.idx);
  if (xi) xi.value = wx;
  if (yi) yi.value = wy;
  renderHgLayoutMap();
}
function hgMapUp() {
  hgDrag = null; hgDragBounds = null;
  window.removeEventListener('pointermove', hgMapMove);
  window.removeEventListener('pointerup', hgMapUp);
  renderHgLayoutMap();   // 드래그 종료 후 축척을 다시 맞춘다
}
window.__hgMapReady = true;
document.querySelectorAll('.hg-map').forEach(svg => svg.addEventListener('pointerdown', hgMapDown));
renderHgLayoutMap();

// ===== 주행 배치도 (Odometry) =====
// 정지점 9개를 m·n·출발코너에서 **계산해서** 그린다. 앵커 배치도(위)와 달리 점을
// 개별로 끌 수 없는데, 그게 이 방식의 제약이기 때문이다: 로봇 경로는 11-op 고정
// (MOVE·MOVE·TURN ×3 + MOVE·MOVE)이라 서버가 m·n 에서 기계적으로 만들어 내고,
// 점 하나를 임의 위치로 옮기면 로봇에게 시킬 수 있는 명령이 없어진다. 그래서
// 드래그 대상은 점이 아니라 **사각형 크기**다.
//
// 좌표표는 wire 규격 §4 의 9점 표와 같은 값을 낸다 — 그 표를 손으로 옮겨 적지
// 않고 여기서 만드는 이유는, m·n 을 바꿨을 때 표와 그림이 갈라지지 않게 하려는 것.
const ODO_VBW = 480, ODO_VBH = 360, ODO_MARGIN = 46;
let odoDragView = null;    // 드래그 중 축척 고정 (앵커 배치도의 hgDragBounds 와 같은 이유)

function odoDims() {
  const rd = (id, dflt) => {
    const el = document.getElementById(id);
    const v = el ? Number(el.value) : NaN;
    return Math.max(20, Math.min(1000, isFinite(v) && v > 0 ? v : dflt));
  };
  const m = rd('odoM', 90), n = rd('odoN', 60);
  const sel = document.getElementById('odoCorner');
  return {m_cm: m, n_cm: n, m_mm: m * 10, n_mm: n * 10,
          corner: sel ? sel.value : 'bottom_left'};
}
// wire 규격 §4. top_left 는 bottom_left 의 y 를 n 기준으로 뒤집은 것과 같다.
// idx 2·4·6 은 코너라 그 자리에서 90° 회전이 일어나고(캡처는 하지 않는다),
// idx 8 은 출발점으로 돌아온 복귀 지점이라 라벨이 idx 0 과 겹친다.
function odoStops(d) {
  const m = d.m_mm, n = d.n_mm;
  const base = [[0,0],[m/2,0],[m,0],[m,n/2],[m,n],[m/2,n],[0,n],[0,n/2],[0,0]];
  const flip = d.corner === 'top_left';
  return base.map((p, i) => ({idx: i, x: p[0], y: flip ? (n - p[1]) : p[1],
                              turn: (i === 2 || i === 4 || i === 6), closing: (i === 8)}));
}
// 축척을 x·y 공통으로 잡아 가로세로 비율을 지킨다. 앵커 배치도는 축마다 따로
// 늘리지만, 여기서는 "이 사각형이 정말 90×60 인가"가 그림의 요점이라 왜곡하면 안 된다.
function odoView(d) {
  const s = Math.min((ODO_VBW - 2 * ODO_MARGIN) / Math.max(d.m_mm, 1),
                     (ODO_VBH - 2 * ODO_MARGIN) / Math.max(d.n_mm, 1));
  return {s: s, ox: (ODO_VBW - d.m_mm * s) / 2, oy: (ODO_VBH + d.n_mm * s) / 2};
}
function odoW2S(x, y, v) { return [v.ox + x * v.s, v.oy - y * v.s]; }
function odoS2W(sx, sy, v) { return [(sx - v.ox) / v.s, (v.oy - sy) / v.s]; }

function renderOdoLayout() {
  const svg = document.getElementById('odoMap');
  if (!svg) return;
  const d = odoDims();
  const v = odoDragView || odoView(d);
  const pts = odoStops(d);
  const S = (x, y) => odoW2S(x, y, v);
  const f = (a) => a[0].toFixed(1) + ',' + a[1].toFixed(1);
  let g = '';

  const cA = S(0, 0), cB = S(d.m_mm, 0), cC = S(d.m_mm, d.n_mm), cD = S(0, d.n_mm);
  g += '<polygon points="' + [f(cA), f(cB), f(cC), f(cD)].join(' ') +
       '" fill="rgba(59,130,246,0.07)" stroke="rgba(59,130,246,0.55)" stroke-width="1"/>';

  // 진행 방향 삼각형. 순번대로 이은 선분의 중점에 하나씩 — 화살표가 없으면
  // 좌회전인지 우회전인지가 그림에서 안 보이고, 그게 start_corner 의 유일한 차이다.
  for (let i = 0; i < 8; ++i) {
    const a = S(pts[i].x, pts[i].y), b = S(pts[i + 1].x, pts[i + 1].y);
    const mx = (a[0] + b[0]) / 2, my = (a[1] + b[1]) / 2;
    const dx = b[0] - a[0], dy = b[1] - a[1];
    const L = Math.hypot(dx, dy) || 1;
    const ux = dx / L, uy = dy / L, px = -uy, py = ux;
    g += '<polygon points="' +
      [(mx + ux * 7).toFixed(1) + ',' + (my + uy * 7).toFixed(1),
       (mx - ux * 3 + px * 4.5).toFixed(1) + ',' + (my - uy * 3 + py * 4.5).toFixed(1),
       (mx - ux * 3 - px * 4.5).toFixed(1) + ',' + (my - uy * 3 - py * 4.5).toFixed(1)].join(' ') +
      '" fill="rgba(59,130,246,0.8)"/>';
  }

  // 회전 지점. 사각형 바깥쪽으로 밀어 점 라벨과 겹치지 않게 한다.
  const cx = d.m_mm / 2, cy = d.n_mm / 2;
  pts.forEach(p => {
    if (!p.turn) return;
    const q = S(p.x, p.y);
    const ox = (p.x > cx ? 1 : -1) * 20, oy = (p.y > cy ? -1 : 1) * 20;
    g += '<text x="' + (q[0] + ox).toFixed(1) + '" y="' + (q[1] + oy).toFixed(1) +
         '" font-size="13" fill="#f59e0b" text-anchor="middle">&#8635;</text>';
  });

  // 복귀 지점(idx 8)은 idx 0 과 같은 자리다. 점을 겹쳐 그리면 하나로 보여서
  // "9점을 찍는데 좌표는 8종"이라는 이 방식의 핵심이 화면에서 사라진다.
  const home = S(pts[0].x, pts[0].y);
  g += '<circle cx="' + home[0].toFixed(1) + '" cy="' + home[1].toFixed(1) +
       '" r="17" fill="none" stroke="#f59e0b" stroke-width="1.5" stroke-dasharray="3 3"/>';
  g += '<text x="' + (home[0] - 20).toFixed(1) + '" y="' + (home[1] + 24).toFixed(1) +
       '" font-size="10" font-weight="700" fill="#f59e0b" text-anchor="middle">8</text>';

  pts.forEach(p => {
    if (p.closing) return;   // 위에서 고리로 따로 그렸다
    const q = S(p.x, p.y);
    g += '<circle cx="' + q[0].toFixed(1) + '" cy="' + q[1].toFixed(1) +
         '" r="11" fill="#3b82f6" stroke="#fff" stroke-width="1.5"/>' +
         '<text x="' + q[0].toFixed(1) + '" y="' + (q[1] + 3.5).toFixed(1) +
         '" font-size="10" font-weight="700" fill="#fff" text-anchor="middle">' + p.idx + '</text>';
  });

  // 원점 + 치수 + 크기 핸들
  g += '<text x="' + (cA[0] - 6).toFixed(1) + '" y="' + (cA[1] + 20).toFixed(1) +
       '" font-size="9" fill="#999" text-anchor="middle">(0,0) 출발</text>';
  const mid1 = S(cx, 0), mid2 = S(0, cy);
  g += '<text x="' + mid1[0].toFixed(1) + '" y="' + (mid1[1] + 30).toFixed(1) +
       '" font-size="10" fill="#999" text-anchor="middle">m = ' + d.m_cm + ' cm (' + d.m_mm + ' mm)</text>';
  g += '<text x="' + (mid2[0] - 14).toFixed(1) + '" y="' + mid2[1].toFixed(1) +
       '" font-size="10" fill="#999" text-anchor="end">n = ' + d.n_cm + ' cm</text>';
  const h = S(d.m_mm, d.n_mm);
  g += '<g id="odoHandle" style="cursor:grab"><polygon points="' +
       [(h[0]).toFixed(1) + ',' + (h[1] - 9).toFixed(1),
        (h[0] + 9).toFixed(1) + ',' + h[1].toFixed(1),
        (h[0]).toFixed(1) + ',' + (h[1] + 9).toFixed(1),
        (h[0] - 9).toFixed(1) + ',' + h[1].toFixed(1)].join(' ') +
       '" fill="#10b981" stroke="#fff" stroke-width="1.5"/></g>';

  svg.innerHTML = g;

  const box = document.getElementById('odoPoints');
  if (box) {
    let t = '<table><thead><tr><th>idx</th><th>X</th><th>Y</th><th>상태</th>' +
            '<th>spread</th><th>픽셀 u,v</th><th>비고</th></tr></thead><tbody>';
    pts.forEach(p => {
      const st = odoProg.pt[p.idx] || {};
      const cls = st.state === 'ok' ? 'odo-st-ok' : st.state === 'fail' ? 'odo-st-fail'
                : st.state === 'wait' ? 'odo-st-wait' : '';
      const label = st.state === 'ok' ? '✅ 캡처' : st.state === 'fail' ? '❌ 실패'
                  : st.state === 'wait' ? '⏳ 대기' : '—';
      const note = st.reason ? st.reason
                 : p.closing ? '복귀 — 피팅 제외, 폐합 전용'
                 : p.turn ? '코너 (도착 후 90° 회전)'
                 : (p.idx === 0 ? '출발' : '');
      t += '<tr' + (st.state === 'wait' ? ' class="odo-row-now"' : '') + '>' +
           '<td>' + p.idx + '</td><td>' + p.x + '</td><td>' + p.y + '</td>' +
           '<td class="' + cls + '">' + label + '</td>' +
           '<td>' + (st.spread === undefined ? '' : st.spread.toFixed(2) + 'px') + '</td>' +
           '<td>' + (st.uv ? st.uv[0].toFixed(0) + ',' + st.uv[1].toFixed(0) : '') + '</td>' +
           '<td>' + note + '</td></tr>';
    });
    box.innerHTML = t + '</tbody></table>';
  }
  odoRenderSessionLine();

  const pv = document.getElementById('odoStartPreview');
  if (pv) {
    const chSel = document.getElementById('calibCh');
    const ch = chSel && chSel.value !== '' ? Number(chSel.value) : 1;
    // 진행 중인 세션이 있으면 그 request_id 를, 없으면 자리표시자를 보인다 —
    // 실제 값은 [시작]을 누르는 순간 만들어진다.
    pv.textContent = JSON.stringify({type: 'CMD', payload: {
      cmd: 'CALIB_START', ch: ch, request_id: odoProg.rid || 'adm-<보낼 때 생성>',
      method: 'robot_motion', m_cm: d.m_cm, n_cm: d.n_cm, start_corner: d.corner}}, null, 1);
  }
}

// --- 진행도 -----------------------------------------------------------------
//
// 출처가 둘이다. 지점별 성공/실패는 **서버 TAP 사본**으로 들어온다(서버가 ADMIN
// 에게 IN/OUT 전부를 흘려준다) — 그래서 폴링도, 서버 수정도 필요 없다. mm 단위
// 품질 지표(LOO·폐합·체커보드 대비)는 카메라가 이 대시보드로 직접 올리는
// ODOM_DONE 에만 있다. 서버는 그 시점에 새 캘리가 없어 픽셀 단위 폐합오차밖에
// 못 만든다(wire 규격 §5).
//
// rp_core 가 두 출처를 '[odo] {json}' 한 형식으로 통일해서 보낸다.
let odoProg = {ch: null, rid: null, pt: {}, done: null, state: ''};

function odoResetProgress(ch, rid) {
  odoProg = {ch: ch, rid: rid, pt: {}, done: null, state: '진행 중'};
}
function odoRenderSessionLine() {
  const el = document.getElementById('odoSession');
  if (!el) return;
  if (!odoProg.rid && !odoProg.state) {
    el.textContent = '세션 없음 — 아래는 계획 좌표입니다.';
  } else {
    const n = Object.values(odoProg.pt).filter(s => s.state === 'ok' && s.idx !== 8).length;
    el.innerHTML = '세션 <b>' + (odoProg.state || '-') + '</b>' +
      (odoProg.ch != null ? ' · CH' + odoProg.ch : '') +
      (odoProg.rid ? ' · <code>' + odoProg.rid + '</code>' : '') +
      ' · 유효 ' + n + '/8점';
  }
  const sum = document.getElementById('odoSummary');
  if (!sum) return;
  const d = odoProg.done;
  if (!d) { sum.textContent = ''; return; }
  // 폐합오차는 피팅에서 제외된 복귀 지점으로 재는 표본외 검증이다. 다만 이것도
  // 균일 스케일 오차는 못 잡는다 — 네 변이 같은 비율로 짧으면 궤적도 닫힌다.
  // 그건 cmp_scale(체커보드 대비)만 드러낸다. 그래서 둘을 같이 보여준다.
  sum.innerHTML = '<b>' + (d.ok ? '피팅 성공' : '피팅 실패') + '</b>' +
    (d.reason ? ' (' + d.reason + ')' : '') +
    ' · n=' + d.n + ' · LOO ' + d.rmse_loo_mm + 'mm' +
    (d.loo_valid ? '' : ' <span class="odo-st-wait">(LOO 무효 — 점 부족)</span>') +
    ' · 폐합 ' + d.closure_mm + 'mm' +
    (d.cmp_ok ? ' · 체커보드 대비 scale ' + Number(d.cmp_scale).toFixed(4) +
                ' / rmse ' + d.cmp_rmse_mm + 'mm'
              : ' · 체커보드 비교 불가' + (d.cmp_reason ? '(' + d.cmp_reason + ')' : ''));
}

// 세션 개시/중단. 서버로는 대시보드의 ADMIN 연결로 나간다(/odo/start 라우트).
//
// 세션이 열렸다는 사실을 TAP 되돌아오기로 알아내지 않는다 — 서버는 ADMIN 자신과
// 오간 메시지는 tap 하지 않으므로(무한 루프 방지, protocol.hpp) 우리가 보낸
// CALIB_START 는 사본이 오지 않는다. 그래서 전송 성공 응답을 받은 자리에서
// 진행도를 직접 초기화한다.
function odoSendNote(msg, bad) {
  const el = document.getElementById('odoSendNote');
  if (el) el.innerHTML = '<span class="' + (bad ? 'odo-st-fail' : 'odo-st-ok') + '">' + msg + '</span>';
}
function odoCurrentCh() {
  const sel = document.getElementById('calibCh');
  return sel && sel.value !== '' ? Number(sel.value) : 1;
}
async function odoStartSession() {
  const d = odoDims();
  const ch = odoCurrentCh();
  // 개행은 \\n 으로 쓴다 — PAGE 가 raw 문자열이 아니라서 \\n 한 겹은 파이썬이
  // 먼저 먹고 진짜 줄바꿈이 되어 JS 문자열 리터럴이 깨진다.
  if (!confirm('CH' + ch + ' 주행 캘리를 시작합니다.\\n\\n' +
               d.m_cm + ' x ' + d.n_cm + ' cm, 출발 ' + d.corner + '\\n\\n' +
               '로봇이 실제로 주행합니다. 사각형 안에 사람·장애물이 없습니까?')) return;
  const rid = 'adm-' + Date.now();
  try {
    const r = await fetch('/odo/start', {method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({ch: ch, request_id: rid, m_cm: d.m_cm, n_cm: d.n_cm,
                            start_corner: d.corner})});
    const j = await r.json();
    if (!j.ok) { odoSendNote('전송 실패 — ' + (j.reason || r.status), true); return; }
    odoResetProgress(ch, rid);
    odoProg.state = '개시 요청됨';
    odoSendNote('CALIB_START 전송됨 (' + rid + ')');
  } catch (e) {
    odoSendNote('전송 실패 — ' + e, true);
  }
  renderOdoLayout();
}
async function odoCancelSession() {
  // 취소는 지금 진행 중인 세션의 request_id 로만 보낸다. 서버가 그 값이 다르면
  // 무시하므로(router_calib.cpp), 없는 채로 보내면 로봇이 계속 굴러간다.
  if (!odoProg.rid) { odoSendNote('진행 중인 세션이 없습니다.', true); return; }
  if (!confirm('주행을 중단합니다. 로봇이 즉시 정지합니다.')) return;
  try {
    const r = await fetch('/odo/cancel', {method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({ch: odoProg.ch || odoCurrentCh(), request_id: odoProg.rid})});
    const j = await r.json();
    odoSendNote(j.ok ? 'CALIB_CANCEL 전송됨' : '전송 실패 — ' + (j.reason || r.status), !j.ok);
  } catch (e) {
    odoSendNote('전송 실패 — ' + e, true);
  }
}

// ODOM_PREFER <ch> <0|1> — 카메라 명령이라 /cmd(7100) 로 간다. 세션 개시(서버,
// 9000)와 다른 통로다. ch 는 이 앱 규약대로 0-based(chArg).
function odoSetPrefer(on) {
  if (on && !confirm('로봇 측위를 주행 캘리의 측정 H_marker 로 바꿉니다.\\n\\n' +
                     '체커보드 대비 scale 이 1.000 에서 벗어나 있으면 그만큼 좌표가 틀어집니다.\\n' +
                     '계속할까요?')) return;
  send('ODOM_PREFER ' + chArg() + ' ' + (on ? 1 : 0));
}
// ODOM_RESEND <ch> — 주행 캘리 번들 재전송 (2026-08-13).
//
// 예전에는 이 자리에 fillCentralHmatrixTemplate + sendCentralHmatrix 가 있었다.
// 그 경로는 브라우저 캐시(hgHfloor/mpPlane)로 번들을 조립하는데, **그 캐시는
// 정적 앵커(floor) 탭 흐름에서만 채워진다** — 주행 캘리 결과는 카메라가 서버로
// 직접 보내고 이 대시보드는 저장하지 않는다. 그래서 Odometry 탭에서 눌러도
// 실제로는 체커보드 번들이 나갔다: H_marker 가 측정값이 아니라 floor H 에서
// 파생된 값이었고, 역산 방향도 자동 경로와 정반대였다.
//
// 지금은 카메라에 "다시 보내라"고만 시킨다. 조립 주체가 하나뿐이라 자동 전송과
// 어긋날 수가 없다.
function odoResendBundle() {
  const ch = odoCurrentCh();
  if (!confirm('CH' + ch + ' 의 주행 캘리 번들을 서버로 다시 보냅니다.\\n\\n' +
               '카메라가 지금 들고 있는 측정 H_marker 로 번들을 다시 조립해\\n' +
               '서버에 저장되고 QT로 중계됩니다. 계속할까요?')) return;
  const el = document.getElementById('hgSendOdomNote');
  if (el) el.textContent = 'ODOM_RESEND 전송됨 — 카메라 응답 대기…';
  sendCh('ODOM_RESEND');
}

// ===== 채널 간 정합 (Registration, 2026-08-15 신설) =====
//
// odoProg/handleOdo와 같은 패턴이다 — 서버가 REGISTER_* 관련 TAP도 같은
// '[odo] {json}' 접두어로 흘려주므로(rp_core.py _ODO_TAP_TYPES에 REGISTER_*
// 추가) 별도 프로토콜 없이 handleOdo와 나란히 handleRegister를 둔다.
let regProg = {chA: null, chB: null, rid: null, okN: 0, failN: 0, state: ''};

function registerPopulateChannelSelects() {
  // #calibCh(공용 채널 셀렉트, RP_CAM_CHANNELS만큼 1-based 옵션)를 그대로
  // 복제한다 — 채널 개수 판정 로직을 여기서 새로 만들지 않기 위해서다.
  const src = document.getElementById('calibCh');
  if (!src || !src.innerHTML) return;
  ['registerChA', 'registerChB'].forEach(id => {
    const el = document.getElementById(id);
    if (el && el.innerHTML !== src.innerHTML) el.innerHTML = src.innerHTML;
  });
}

function registerSendNote(msg, bad) {
  const el = document.getElementById('registerSendNote');
  if (el) el.innerHTML = '<span class="' + (bad ? 'odo-st-fail' : 'odo-st-ok') + '">' + msg + '</span>';
}

function registerRenderProgress() {
  const el = document.getElementById('registerSession');
  if (el) {
    if (!regProg.rid && !regProg.state) {
      el.textContent = '세션 없음.';
    } else {
      const total = regProg.okN + regProg.failN;
      el.innerHTML = '세션 <b>' + (regProg.state || '-') + '</b>' +
        (regProg.chA != null ? ' · ch_a=CH' + regProg.chA : '') +
        (regProg.chB != null ? ' · ch_b=CH' + regProg.chB : '') +
        (regProg.rid ? ' · <code>' + regProg.rid + '</code>' : '') +
        ' · 유효 ' + regProg.okN + '/' + total;
    }
  }
  const sum = document.getElementById('registerSummary');
  if (sum) sum.textContent = '';  // 결과(reg_scale 등)는 H_MATRIX 전문 로그에서 확인
}

async function registerStartCollect() {
  const chA = Number(document.getElementById('registerChA').value || 0);
  const chB = Number(document.getElementById('registerChB').value || 0);
  if (!chA || !chB || chA === chB) {
    registerSendNote('ch_a/ch_b를 확인하세요 (서로 달라야 함)', true);
    return;
  }
  if (!confirm('CH' + chA + ' 기준으로 CH' + chB + ' 정합 수집을 시작합니다.\\n\\n' +
               '자동 경로가 없습니다 — 시작 후 조이스틱으로 로봇을 겹침 구역에\\n' +
               '직접 몰아야 합니다. 계속할까요?')) return;
  try {
    const r = await fetch('/register/start', {method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({ch_a: chA, ch_b: chB})});
    const j = await r.json();
    if (!j.ok) { registerSendNote('전송 실패 — ' + (j.reason || r.status), true); return; }
    regProg = {chA: chA, chB: chB, rid: null, okN: 0, failN: 0, state: '수집 중 (로봇을 겹침 구역으로 몰 것)'};
    registerSendNote('REGISTER_COLLECT_START 전송됨');
    registerRenderProgress();
  } catch (e) {
    registerSendNote('전송 실패 — ' + e, true);
  }
}
async function registerStopCollect() {
  if (!confirm('정합 수집을 마칩니다 (REGISTER_DONE) — 지금까지 모은 점으로 피팅합니다.')) return;
  try {
    const r = await fetch('/register/stop', {method: 'POST'});
    const j = await r.json();
    registerSendNote(j.ok ? 'REGISTER_COLLECT_STOP 전송됨' : '전송 실패 — ' + (j.reason || r.status), !j.ok);
  } catch (e) {
    registerSendNote('전송 실패 — ' + e, true);
  }
}
async function registerCancelCollect() {
  if (!confirm('정합 수집을 취소합니다 — 지금까지 모은 점을 버립니다.')) return;
  try {
    const r = await fetch('/register/cancel', {method: 'POST'});
    const j = await r.json();
    registerSendNote(j.ok ? 'REGISTER_COLLECT_CANCEL 전송됨' : '전송 실패 — ' + (j.reason || r.status), !j.ok);
  } catch (e) {
    registerSendNote('전송 실패 — ' + e, true);
  }
}

function handleRegister(line) {
  if (line.indexOf('[odo] ') === -1) return;
  let ev;
  try { ev = JSON.parse(line.slice(line.indexOf('[odo] ') + 6)); } catch (e) { return; }
  const p = ev.payload || {};
  if (ev.type === 'REGISTER_CAPTURE') {
    if (p.request_id && !regProg.rid) regProg.rid = p.request_id;
    regProg.state = '캡처 중 (idx ' + p.point_index + ')';
  } else if (ev.type === 'REGISTER_CAPTURE_OK') {
    ++regProg.okN;
    regProg.state = '수집 중';
  } else if (ev.type === 'REGISTER_CAPTURE_FAIL') {
    ++regProg.failN;
    // not_both_seen은 이 방식에서 정상적으로 계속 나는 실패라 상태 문구를
    // 실패로 바꾸지 않는다 — Odometry의 지점 실패와 달리 세션이 안 접힌다.
    regProg.state = (p.reason === 'not_both_seen') ? '수집 중 (아직 겹침 구역 밖)'
                                                    : '수집 중 (실패: ' + p.reason + ')';
  } else if (ev.type === 'REGISTER_FAIL') {
    regProg.state = '실패 (' + (p.reason || '') + ')';
  } else if (ev.type === 'REGISTER_STOPPED') {
    regProg.state = '취소됨';
  } else if (ev.type === 'H_MATRIX' && ('reg_scale' in (p.calib || {}))) {
    // 정합 성공 종료 — SendCalibBundle()이 실은 reg_* 필드로 판별한다.
    const c = p.calib;
    regProg.state = '완료 — scale=' + Number(c.reg_scale).toFixed(4) +
      ' rmse=' + c.reg_rmse_mm + 'mm n=' + c.reg_n;
  } else {
    return;  // 관계없는 [odo] 이벤트 (Odometry용) — 무시
  }
  registerRenderProgress();
}
function odoRenderResend(p) {
  const el = document.getElementById('hgSendOdomNote');
  if (!el) return;
  el.innerHTML = p.ok
    ? '<span class="odo-st-ok">CH' + ((Number(p.ch) || 0) + 1) +
      ' 번들을 서버로 재전송했습니다 (H_marker=측정값, H_floor=역산값).</span>'
    : '<span class="odo-st-fail">재전송 거부 — ' + (p.reason || '사유 없음') + '</span>';
}

// FLOOR_RESEND <ch> — 앵커 캘리 번들 재전송 (2026-08-14). ODOM_RESEND 와 같은
// 이유로 브라우저는 JSON 을 조립하지 않는다.
//
// 예전에는 이 자리에 fillCentralHmatrixTemplate + sendCentralHmatrix 가 있었다.
// 그 조립기가 채우던 image_size(드롭다운 추정)·coord_mode(명령 이력 유추)·
// K/D(놓치면 자리표시자)는 전부 **카메라만 아는 사실**이었다 — 브라우저가 그걸
// 추측해서 라벨을 붙이고 있었고, 2026-08-11 CH2 사고가 그 결과다.
function floorResendBundle() {
  const ch = curCh();
  if (!confirm('CH' + (ch + 1) + ' 의 앵커 캘리 번들을 서버로 다시 보냅니다.\\n\\n' +
               '카메라가 지금 들고 있는 바닥 H 로 번들을 다시 조립해\\n' +
               '서버에 저장되고 QT로 중계됩니다. 계속할까요?')) return;
  const el = document.getElementById('hgSendFloorNote');
  if (el) el.textContent = 'FLOOR_RESEND 전송됨 — 카메라 응답 대기…';
  sendCh('FLOOR_RESEND');
}
function floorRenderResend(p) {
  const el = document.getElementById('hgSendFloorNote');
  if (!el) return;
  el.innerHTML = p.ok
    ? '<span class="odo-st-ok">CH' + ((Number(p.ch) || 0) + 1) +
      ' 번들을 서버로 재전송했습니다 (H_floor=측정값, H_marker=파생값).</span>'
    : '<span class="odo-st-fail">재전송 거부 — ' + (p.reason || '사유 없음') + '</span>';
}
function odoRenderPrefer(p) {
  const el = document.getElementById('odoPreferNote');
  if (!el) return;
  const warn = [];
  if (!p.measured_ready) warn.push('측정값 없음 — 주행 캘리를 먼저 하세요');
  if (p.k_stale) warn.push('⚠ 측정 당시와 K 가 다름');
  if (p.height_stale) warn.push('⚠ 측정 당시와 마커 높이가 다름');
  if (!p.ok && p.reason) warn.push('거부: ' + p.reason);
  el.innerHTML = 'CH' + (Number(p.ch) + 1) + ' 현재: <b class="' +
    (p.preferred ? 'odo-st-ok' : '') + '">' +
    (p.preferred ? '측정 (주행)' : '파생 (체커보드)') + '</b>' +
    (warn.length ? ' <span class="odo-st-wait">' + warn.join(' · ') + '</span>' : '');
}

function handleOdo(line) {
  if (line.indexOf('[odo] ') === -1) return;
  let ev;
  try { ev = JSON.parse(line.slice(line.indexOf('[odo] ') + 6)); } catch (e) { return; }
  const p = ev.payload || {};
  const idx = p.point_index;
  if (ev.type === 'CALIB_START') {
    // ADMIN->SRV 와 SRV->CCTV 두 번 지나간다. 같은 request_id 면 두 번째를
    // 무시해야 첫 지점 대기 상태가 지워지지 않는다.
    if (odoProg.rid !== p.request_id) odoResetProgress(p.ch, p.request_id);
  } else if (ev.type === 'CALIB_CAPTURE') {
    if (idx !== undefined) odoProg.pt[idx] = {idx: idx, state: 'wait'};
    odoProg.state = '캡처 대기 (idx ' + idx + ')';
    if (p.request_id && !odoProg.rid) { odoProg.rid = p.request_id; odoProg.ch = p.ch; }
  } else if (ev.type === 'CALIB_CAPTURE_OK') {
    odoProg.pt[idx] = {idx: idx, state: 'ok', spread: Number(p.spread_px),
                       uv: Array.isArray(p.pixel_uv) ? p.pixel_uv : null};
    odoProg.state = '주행 중';
  } else if (ev.type === 'CALIB_CAPTURE_FAIL') {
    odoProg.pt[idx] = {idx: idx, state: 'fail', reason: p.reason || '실패',
                       spread: p.spread_px === undefined ? undefined : Number(p.spread_px)};
    odoProg.state = '주행 중 (실패 있음)';
  } else if (ev.type === 'CALIB_DONE') {
    odoProg.state = '피팅 중';
  } else if (ev.type === 'ODOM_PREFER') {
    odoRenderPrefer(p);
    return;   // 진행도 표와 무관한 설정 응답 — 표를 다시 그릴 이유가 없다
  } else if (ev.type === 'ODOM_RESEND') {
    odoRenderResend(p);
    return;   // 재전송 ack — 진행도 표와 무관하다(ODOM_PREFER 와 같은 이유)
  } else if (ev.type === 'FLOOR_RESEND') {
    floorRenderResend(p);
    return;   // 앵커 탭의 ack — 주행 진행도와는 아무 관계가 없다
  } else if (ev.type === 'ODOM_DONE') {
    odoProg.done = p;
    odoProg.state = p.ok ? '완료' : '실패';
  } else if (ev.type === 'H_MATRIX') {
    odoProg.state = '완료 — 번들 전송됨';
  } else if (ev.type === 'CALIB_FAIL') {
    odoProg.state = '실패 (' + (p.reason || '') + ')';
  } else if (ev.type === 'CALIB_CANCEL') {
    odoProg.state = '취소 요청됨';
  } else if (ev.type === 'CALIB_STOPPED') {
    odoProg.state = '취소됨 — 카메라 정지 확인';
  }
  renderOdoLayout();
}

// 크기 핸들 드래그. 점이 아니라 사각형을 바꾸므로 잡을 곳은 하나뿐이다.
let odoDrag = false;
function odoMapDown(e) {
  if (!e.target.closest('#odoHandle')) return;
  e.preventDefault();
  odoDragView = odoView(odoDims());   // 이 드래그 동안 축척 고정
  odoDrag = true;
  window.addEventListener('pointermove', odoMapMove);
  window.addEventListener('pointerup', odoMapUp);
}
function odoMapMove(e) {
  if (!odoDrag) return;
  const svg = document.getElementById('odoMap');
  const ctm = svg && svg.getScreenCTM();
  if (!ctm) return;
  const p = svg.createSVGPoint(); p.x = e.clientX; p.y = e.clientY;
  const loc = p.matrixTransform(ctm.inverse());
  const w = odoS2W(loc.x, loc.y, odoDragView);
  // 1 cm 단위로 스냅. 서버가 m_cm/n_cm 을 cm 정수로 받으므로(wire §1) mm 단위로
  // 끌 수 있게 두면 화면 값과 실제로 보내는 값이 갈린다.
  const clamp = (mm) => Math.max(20, Math.min(1000, Math.round(mm / 10)));
  const mi = document.getElementById('odoM'), ni = document.getElementById('odoN');
  if (mi) mi.value = clamp(w[0]);
  if (ni) ni.value = clamp(w[1]);
  renderOdoLayout();
}
function odoMapUp() {
  odoDrag = false; odoDragView = null;
  window.removeEventListener('pointermove', odoMapMove);
  window.removeEventListener('pointerup', odoMapUp);
  renderOdoLayout();   // 드래그 종료 후 축척을 다시 맞춘다
}
(function () {
  const svg = document.getElementById('odoMap');
  if (svg) svg.addEventListener('pointerdown', odoMapDown);
  renderOdoLayout();
})();

// Marker recorder: collect every raw detection first; point selection and
// surveyed world coordinates belong to the later PC analysis, not this UI.
const hgExperimentRows = document.getElementById('hgExperimentRows');
const hgExperimentStatusBox = document.getElementById('hgExperimentStatus');
async function hgExperimentRequest(path, body) {
  const r = await fetch(path, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body || {})});
  const data = await r.json();
  if (!r.ok) throw new Error(data.error || '요청 실패');
  return data;
}
async function hgExperimentStart() {
  try {
    await hgExperimentRequest('/hg_experiment/start');
    hgExperimentStatusBox.textContent = '기록 시작: 검출되는 모든 마커를 저장합니다.';
    hgExperimentRows.innerHTML = '<span class="none">검출 마커 수신 대기 중…</span>';
    document.getElementById('hgExperimentDownload').style.display = 'none';
  } catch (e) { hgExperimentStatusBox.textContent = '수집 시작 실패: ' + e.message; }
}
async function hgExperimentStop() {
  try {
    const data = await hgExperimentRequest('/hg_experiment/stop');
    hgExperimentStatusBox.textContent = `수집 완료: ${data.json}, ${data.csv} 생성됨. JSON을 PC 분석기에 넣으세요.`;
    const a = document.getElementById('hgExperimentDownload'); a.href = '/hg_experiment/export'; a.style.display = '';
  } catch (e) { hgExperimentStatusBox.textContent = '수집 종료 실패: ' + e.message; }
}
async function hgExperimentRefresh() {
  try {
    const s = await (await fetch('/hg_experiment/status')).json();
    const candidates = s.candidates || [];
    if (!candidates.length) return;
    let rows = '<div class="qtitle">기록된 마커 (ID / 프레임 / raw 픽셀 중심)</div><table><tr><td>ID</td><td>프레임</td><td>평균 U, V px</td><td>σ px</td></tr>';
    for (const c of candidates) {
      const x = s.samples[String(c.id)] || {};
      rows += `<tr><td>${c.id}</td><td>${x.n || 0}</td><td>(${Number(x.mean_u || 0).toFixed(1)}, ${Number(x.mean_v || 0).toFixed(1)})</td><td>(${Number(x.std_u || 0).toFixed(2)}, ${Number(x.std_v || 0).toFixed(2)})</td></tr>`;
    }
    hgExperimentRows.innerHTML = rows + '</table>';
    hgExperimentStatusBox.innerHTML = `<div class="qtitle">${s.active ? '기록 중' : '기록 대기/완료'} · ${s.w || '?'}×${s.h || '?'}</div>${candidates.length}개 ID가 기록되었습니다.`;
  } catch (_) { /* server may still be starting */ }
}
async function hgExperimentApply() {
  const box = document.getElementById('hgExperimentResultStatus');
  try {
    const result = JSON.parse(document.getElementById('hgExperimentResult').value);
    const data = await hgExperimentRequest('/hg_experiment/result', result);
    box.textContent = `H 적용 명령 전송 · RMSE ${data.rmse_mm ?? '?'} mm · 최대 ${data.max_error_mm ?? '?'} mm`;
  } catch (e) { box.textContent = '결과 적용 실패: ' + e.message; }
}
setInterval(hgExperimentRefresh, 1000);

// 호모그래피 3x3 H 행렬을 표로 렌더 (CALIB_START 완료 / HG_QUERY 응답)
function setHgHealth(title, detail, tone) {
  const box = document.getElementById('hgHealth');
  if (!box) return;
  const color = tone === 'ok' ? 'var(--green)' : tone === 'busy' ? 'var(--blue)' :
                tone === 'warn' ? 'var(--red)' : 'var(--text)';
  // 호출부(handleHgMatrix/handleHgStatus/handleHgPersistence)가 이미 채널
  // 검사를 마친 뒤 부르므로, 여기서 "지금 보는 채널" 배지만 붙이면 어느
  // 렌즈 얘기인지 한눈에 보인다 (가독성 개선, 2026-08-11).
  box.innerHTML = `<span class="qtitle" style="color:${color}">CH${curCh() + 1} · ${title}</span> ${detail}`;
}
function renderHgMatrix(available, arr) {
  const box = document.getElementById('hgMatrix');
  if (!available || !arr || arr.length < 9) {
    box.innerHTML = `<span class="none">CH${curCh() + 1} 호모그래피 아직 계산 안 됨 (캘리브 시작 또는 H 행렬 조회)</span>`;
    return;
  }
  let rows = '';
  for (let r = 0; r < 3; r++) {
    rows += '<tr>';
    for (let c = 0; c < 3; c++)
      rows += `<td style="text-align:right;color:var(--text);font-weight:600;padding:2px 14px 2px 0">${Number(arr[r * 3 + c]).toExponential(4)}</td>`;
    rows += '</tr>';
  }
  box.innerHTML = `<div class="qtitle">CH${curCh() + 1} 호모그래피 H (3×3, 픽셀 → mm)</div><table>${rows}</table>`;
}
// 중앙서버 번들이 싣는 H. 카메라가 보고한 값만 담는다.
let hgHfloor = null;
function handleHgMatrix(line) {
  // "[calib] ch=N HOMOGRAPHY H=[...] ..." 또는 "[calib] ch=N 호모그래피 아직
  // 계산 안 됨" -- 둘 다 ch= 를 담아 온다 (2026-08-10). ch 없는 줄(옛
  // 프로토콜 흔적)은 무시 -- 어느 렌즈인지 몰라 전역에 반영하면 다시
  // 채널이 섞인다.
  const chm = line.match(/ch=(\\d+)/);
  if (!chm) return;
  const ch = Number(chm[1]);
  const hm = line.match(/H=\\[([^\\]]*)\\]/);
  if (hm) {
    const arr = hm[1].split(',').map(s => s.trim()).filter(s => s.length);
    if (arr.length >= 9) {
      let H = arr.slice(0, 9).map(Number);
      if (H.some(v => !Number.isFinite(v))) H = null;
      chHg[ch].Hfloor = H;
      // 이 H 가 피팅된 픽셀 공간(카메라의 FittedUndistorted). 번들 coord_mode 의
      // 1순위 근거 — 설정 이력이 아니라 행렬 자체의 성질이라 새로고침에도 살아남는다.
      const um = line.match(/undistorted=(True|False)/);
      if (um) chHg[ch].undistorted = (um[1] === 'True');
      if (ch === curCh()) {
        renderHgMatrix(true, arr);
        hgHfloor = H;
        if (line.includes('HOMOGRAPHY')) {
          setHgHealth('H 사용 가능', '카메라에 적용된 H가 있습니다. 저장 여부는 마지막 저장 동작으로 확인하세요.', 'ok');
          // 완료 H 도착 시 "진행중" 배너 해제 — 카메라 HG_DONE 배포 전 대시보드측 보조 (2026-08-11).
          // 진행중일 때만 바꿔 다른 상태는 안 건드린다.
          const _sb = document.getElementById('hgStatus');
          if (_sb && _sb.innerHTML.includes('진행중'))
            _sb.innerHTML = '<span class="qtitle" style="color:var(--green)">캘리브 완료</span> H가 계산됐습니다' + hgWorldClaim();
        }
      }
    }
  } else if (line.includes('호모그래피 아직 계산 안 됨')) {
    chHg[ch].Hfloor = null;
    if (ch === curCh()) {
      renderHgMatrix(false);
      hgHfloor = null;
      setHgHealth('H 없음', '1단계에서 H를 계산하거나 고급 분석 결과를 적용하세요.', 'warn');
    }
  }
}
// "[calib] ch=N FIT n=.. rmse_in=..mm rmse_loo=..mm max_loo=..mm(id=..) loo_valid=.. advisory=.."
// 또는 "[calib] ch=N FIT 없음 — 아직 계산된 호모그래피가 없습니다" (HG_QUERY 응답, 2026-08-10).
function handleHgFit(line) {
  const mNone = line.match(/^\\[calib\\] ch=(\\d+) FIT 없음/);
  if (mNone) {
    const ch = Number(mNone[1]);
    chHg[ch] = chHg[ch] || emptyChHg();
    chHg[ch].fit = null;
    chHg[ch].fitPts = {};   // FIT_PT가 뒤따르지 않으므로 옛 앵커 잔상을 지운다
    if (ch === curCh()) renderHgFit();
    return;
  }
  const m = line.match(/^\\[calib\\] ch=(\\d+) FIT n=(\\d+) rmse_in=(\\S+)mm rmse_loo=(\\S+)mm max_loo=(\\S+)mm\\(id=([^)]+)\\) loo_valid=(True|False) advisory=(.*)$/);
  if (!m) return;
  const ch = Number(m[1]);
  const num = s => (s === 'None' ? null : Number(s));
  chHg[ch] = chHg[ch] || emptyChHg();
  chHg[ch].fit = {
    n: Number(m[2]), rmseIn: num(m[3]), rmseLoo: num(m[4]),
    maxLoo: num(m[5]), maxLooId: num(m[6]),
    looValid: m[7] === 'True', advisory: m[8] === 'True',
  };
  // 뒤이어 오는 FIT_PT들이 이번 앵커 집합으로 다시 채운다 -- 지금 비워서
  // 앵커를 뺀 뒤에도 이전 앵커의 오차 줄이 화면에 남지 않게 한다.
  chHg[ch].fitPts = {};
  if (ch === curCh()) renderHgFit();
}
// "[calib] ch=N FIT_PT id=.. in=..mm loo=..mm" -- FIT 뒤에 앵커 개수만큼 따라온다.
function handleHgFitPt(line) {
  const m = line.match(/^\\[calib\\] ch=(\\d+) FIT_PT id=(-?\\d+) in=(\\S+)mm loo=(\\S+)mm$/);
  if (!m) return;
  const ch = Number(m[1]);
  const num = s => (s === 'None' ? null : Number(s));
  chHg[ch] = chHg[ch] || emptyChHg();
  chHg[ch].fitPts[Number(m[2])] = { id: Number(m[2]), inMm: num(m[3]), looMm: num(m[4]) };
  if (ch === curCh()) renderHgFit();
}
// 정확도(LOO) 조회 버튼의 결과 표시. 캘리브레이션 탭에 있던 "검증 기준점" div를
// 그대로 재사용한다 -- 실제로 존재하는 정확도 지표가 생겼으니 자리를 물려받았다.
function renderHgFit() {
  const st = chHg[curCh()];
  const fit = st && st.fit;
  if (!fit) {
    hgValidationEditStatus.style.whiteSpace = 'pre-line';
    hgValidationEditStatus.textContent = '아직 계산된 호모그래피 정확도(LOO) 정보가 없습니다. "정확도 조회"를 눌러 확인하세요.';
    return;
  }
  // "계산 안 됨"은 null이 아니라 음수/0 신호값으로 온다(homography_mapper.cc:
  // loo_valid가 false면 rmse_loo_mm=-1, max_loo_mm=0, max_loo_id=-1로 남는다).
  // 그대로 찍으면 "0.0mm"가 실측 0오차처럼 보이므로, loo_valid가 아니면 LOO
  // 관련 값은 통째로 '-'로 가린다.
  const fmt = v => (v === null || v === undefined || v < 0 ? '-' : v.toFixed(1));
  const rmseLooTxt = fit.looValid ? fmt(fit.rmseLoo) : '-';
  const maxLooTxt = fit.looValid ? fmt(fit.maxLoo) : '-';
  const maxLooIdTxt = (fit.looValid && fit.maxLooId >= 0) ? fit.maxLooId : '-';
  let text = `n=${fit.n}  RMSE(적용점)=${fmt(fit.rmseIn)}mm  RMSE(LOO)=${rmseLooTxt}mm  ` +
             `최대오차(LOO)=${maxLooTxt}mm(id=${maxLooIdTxt})`;
  if (!fit.looValid) text += '  [표본 부족 — 앵커 5개 이상부터 LOO가 계산됩니다]';
  else if (fit.advisory) text += '  [참고용 — 앵커 8개 미만이라 LOO 오차의 신뢰도가 낮습니다]';
  const pts = Object.values(st.fitPts || {}).sort((a, b) => a.id - b.id);
  if (pts.length)
    text += '\\n' + pts.map(p => `  id=${p.id}: 적용점 오차=${fmt(p.inMm)}mm  LOO 오차=${fmt(p.looMm)}mm`).join('\\n');
  hgValidationEditStatus.style.whiteSpace = 'pre-line';
  hgValidationEditStatus.textContent = text;
}
// 캘리브레이션 채널을 바꿨을 때 호출된다. 앵커/H/마커평면/정확도/K·dist는
// rawFrame과 달리 계속 스트리밍되지 않고 조회해야만 갱신되므로, 이 채널에서
// 이미 캐시해 둔 값으로 먼저 화면을 채우고(빈 화면으로 깜빡이지 않게) 최신값도
// 다시 조회한다. K/dist도 같은 이유로 여기서 같이 동기화한다(2026-08-10 —
// 이전엔 kCalib이 채널 구분 없이 전역이라, 다른 렌즈에서 조회한 K를 엉뚱한
// 채널의 보정 좌표 표시에 그대로 쓰고 있었다).
function syncCalibFromChannel() {
  renderCalibProbeCanvas();   // 보드 코너 캐시도 채널별이니 즉시 다시 그림(요청 없이 캐시만)
  const st = chHg[curCh()] || emptyChHg();
  hgCameraAnchors = st.anchors;
  renderHgAnchorRows(st.anchors || []);
  hgHfloor = st.Hfloor;
  renderHgMatrix(!!st.Hfloor, st.Hfloor);
  mpPlane = st.mpPlane;
  redrawRawCanvas();
  renderHgFit();
  sendCh('HG_QUERY');
  sendCh('MARKER_PLANE_QUERY');
  sendCh('ANCHOR_QUERY');
  // IVA_SYNC엔 조회(QUERY) 명령이 없다 -- 카메라 상태가 아니라 "그때 계산한
  // 결과"라, 채널을 되돌아와도 다시 계산하기 전엔 캐시를 그대로 보여준다.
  // 반영(push) 결과는 채널이 다를 수 있으니 항상 초기 문구로 되돌린다.
  renderIvaSync(st.ivaSync);
  const ivaPushBox = document.getElementById('ivaPushStatus');
  if (ivaPushBox) ivaPushBox.innerHTML = '<span class="none">아직 반영하지 않음</span>';

  kCalib = chKCalib[curCh()] || null;
  renderKQuery(!!kCalib, kCalib?.fx, kCalib?.fy, kCalib?.cx, kCalib?.cy, kCalib?.dist);
  refreshUndistState();
  sendCh('CALIB_K_QUERY');
  sendCh('CALIB_K_STATUS');

  // 동적 ROI 설정도 이제 채널별이다(2026-08-11) — 캐시로 먼저 채우고
  // (bare DYNROI는 카메라 쪽 버그를 고쳐서 이제 순수 조회라 안전하다) 4채널
  // 전부 다시 조회해 최신값으로 맞춘다.
  const dcfg = chDynRoiCfg[curCh()] || emptyDynRoiCfg();
  dynRoiOn = dcfg.enabled;
  const dchk = document.getElementById('dynRoiChk');
  const dmg = document.getElementById('dynMargin');
  const dmm = document.getElementById('dynMaxMiss');
  if (dchk) dchk.checked = dcfg.enabled;
  if (dmg && document.activeElement !== dmg) dmg.value = dcfg.margin;
  if (dmm && document.activeElement !== dmm) dmm.value = dcfg.maxMiss;
  renderDynRoiState();
  send('DYNROI');
}
function handleHgStatus(line) {
  const box = document.getElementById('hgStatus');
  if (line.includes('[calib] camera acknowledged')) {
    // ch는 줄 끝에 "(ch=N)"으로 붙어 온다(2026-08-10) -- 이 채널 검사가
    // 빠져 있어서 다른 렌즈에서 CALIB_START를 눌러도 지금 보고 있는 채널의
    // "현재 호모그래피 상태" 배너가 덩달아 "진행중"으로 바뀌었다 (2026-08-11
    // 사용자 보고로 발견 — setHgHealth/renderHgMatrix는 이미 ch 검사가
    // 있었는데 이 분기만 빠져 있었다).
    const chm = line.match(/\\(ch=(\\d+)\\)/);
    if (!chm || Number(chm[1]) !== curCh()) return;
    box.innerHTML = '<span class="qtitle">캘리브 진행중…</span> 등록된 계산 anchor 마커가 <b>하나도 빠짐없이</b> 계속 보이게 유지하세요';
    setHgHealth('앵커 H 계산 중', '등록된 anchor가 전부 계속 보이게 유지하세요.', 'busy');
  } else if (line.includes('[calib] SUCCESS')) {
    // 다른 렌즈의 완료가 지금 보고 있는 채널의 배너를 덮지 않게 한다 —
    // camera acknowledged 분기와 같은 검사가 여기엔 빠져 있었다. (2026-08-11)
    const chm = line.match(/\\(ch=(\\d+)\\)/);
    if (chm && Number(chm[1]) !== curCh()) return;
    lastHgSuccessBase = '<span class="qtitle" style="color:var(--green)">캘리브 완료</span> ' +
                        line.replace(/^.*\\[calib\\]\\s*/, '').replace(/\\s*\\(ch=\\d+\\)\\s*$/, '');
    box.innerHTML = lastHgSuccessBase + hgWorldClaim();
    setHgHealth('앵커 H 적용됨 · 미저장', '2단계에서 검증한 뒤 3단계에서 저장하세요.', 'ok');
    // 마커 평면은 이 피팅으로 방금 다시 산출됐는데(카메라 RefreshMarkerPlane),
    // 대시보드는 MARKER_PLANE_QUERY 를 탭·채널 전환 때만 보낸다. 그래서 위
    // hgWorldClaim() 은 "직전에 알던" 상태로 문구를 정하고, 성공적으로 undistort
    // 공간에서 다시 피팅한 직후에도 "아직 준비되지 않아 world 좌표는 안 나갑니다"
    // 를 그대로 띄웠다 — 카메라는 이미 ready 인데. 여기서 다시 물어보고, 답이
    // 오면 handleMarkerPlane() 이 이 배너를 다시 그린다. (2026-08-11)
    sendCh('MARKER_PLANE_QUERY');
  } else if (line.includes('[calib] FAILED')) {
    const chm = line.match(/\\(ch=(\\d+)\\)/);
    if (chm && Number(chm[1]) !== curCh()) return;
    box.innerHTML = '<span class="qtitle" style="color:var(--red)">캘리브 실패</span> ' +
                    line.replace(/^.*\\[calib\\]\\s*/, '').replace(/\\s*\\(ch=\\d+\\)\\s*$/, '');
    setHgHealth('앵커 H 계산 실패', '아래 원인을 확인하고 다시 시도하세요.', 'warn');
  }
}

// "이제 world 좌표가 스트리밍됩니다" 는 H 가 생겼다는 것만으로는 참이 아니다.
// POS/CAM_POSE 에 world 가 실리려면 마커 평면(시차 보정)이 준비돼야 하고, 그건
// undistort 공간에서 피팅한 H + 그 렌즈의 K/dist 를 요구한다. raw 로 피팅한
// H 로도 이 문구가 그대로 떠서, 카메라는 "시차 보정 불가"라고 말하는데 배너는
// 스트리밍된다고 말하는 상태가 됐다 — 둘 중 하나는 거짓말이다. (2026-08-11)
// 마지막 "캘리브 완료 …" 배너의 앞부분(world 문구 제외). 마커 평면 상태가 뒤늦게
// 도착하면 이 앞부분에 새 판정을 다시 붙여 그린다.
let lastHgSuccessBase = '';

function hgWorldClaim() {
  const ch = curCh();
  if (chHg[ch] && chHg[ch].mpPlane) {
    return ' → <b>world 좌표가 스트리밍됩니다</b>';
  }
  return ' → 픽셀→바닥 변환은 됩니다. <b class="warn">다만 마커 평면(시차 보정)이 아직 준비되지 않아 world 좌표는 안 나갑니다</b>' +
         ' — 이 렌즈의 K/dist 를 로드하고, <code>HG_CLEAR</code> → <code>HG_COORD_MODE 1</code> → 재계산으로' +
         ' H 를 undistort 공간에서 다시 피팅해야 합니다 (자세한 사유는 마커 평면 카드에서 조회).';
}
function handleHgPersistence(line) {
  if (line.includes('저장 완료'))
    setHgHealth('H 저장 완료', '현재 H가 /mnt에 저장되어 재부팅 뒤에도 유지됩니다.', 'ok');
  else if (line.includes('H 저장 실패'))
    setHgHealth('H 저장 실패', '현재 H는 적용되어 있을 수 있지만 영구 저장되지 않았습니다.', 'warn');
  else if (line.includes('PC 분석 H 적용 완료'))
    setHgHealth('PC 분석 H 적용됨 · 미저장', '검증 후 3단계에서 저장하세요.', 'ok');
}
// ArucoPosePNM의 실제 형식은 "HG_COORD_MODE <ch> <0|1>" (sscanf "%d %d") —
// "undistort"/"raw" 문자열이 아니라 정수다. 문자열을 그대로 보내면 %d 파싱이
// 매 순간 실패해서 이 명령은 지금까지 조용히 항상 무시되고 있었다 (2026-08-10
// 발견, 채널 누락과는 별개의 버그).
function hgCoordModeArg(mode) {
  return (mode === 'undistort') ? '1' : '0';
}

// HG_CLEAR — 대시보드에 버튼이 없어서, 정작 카메라가 "HG_CLEAR 후 다시 계산하세요"
// 라고 시키는 상황(raw 로 피팅된 H 때문에 시차 보정이 막힌 경우)에 누를 곳이
// 없었다. 콘솔로만 가능했다. (2026-08-11)
//
// 앵커는 카메라가 의도적으로 보존한다(HomographyMapper::Clear 의 주석: 줄자로 잰
// 입력이지 행렬의 산물이 아니므로). 그 사실을 확인창에 적는 이유는, 안 적으면
// "17개를 다시 재야 하나" 때문에 눌러야 할 때 못 누르기 때문이다.
function clearHomography() {
  const ch = curCh();
  const n = (chHg[ch] && Array.isArray(chHg[ch].anchors)) ? chHg[ch].anchors.length : null;
  if (!confirm(
      `CH${ch + 1} 의 호모그래피를 지웁니다.\n\n` +
      `· RAM 과 /mnt 양쪽에서 사라집니다 (되돌릴 수 없습니다)\n` +
      `· 등록한 앵커${n !== null ? ` ${n}개` : ''}는 그대로 남습니다 — 다시 잴 필요 없습니다\n` +
      `· 이 채널의 world 좌표 출력은 다시 계산할 때까지 멈춥니다\n\n` +
      `계속할까요?`)) {
    return;
  }
  sendCh('HG_CLEAR');
  const box = document.getElementById('hgStatus');
  if (box) {
    box.innerHTML = '<span class="qtitle">H 삭제 요청</span> 이제 「H 입력 좌표계」를 고르고 ' +
                    '<b>좌표계 적용</b> → <b>앵커로 H 계산</b> 순으로 진행하세요.';
  }
  setHgHealth('H 없음', '좌표계를 정한 뒤 앵커로 다시 계산하세요.', 'warn');
}
function applyHgCoordMode() {
  hgCoordDefaultApplied = true;   // 수동 조작 뒤에는 기본값을 다시 밀어넣지 않는다
  const mode = document.getElementById('hgCoordMode').value;
  send('HG_COORD_MODE ' + chArg() + ' ' + hgCoordModeArg(mode));
}
// QT 파이프라인이 undistortPoints -> H 순서라, raw 로 피팅한 H 는 조용히 왜곡만큼
// 틀린 좌표를 만든다. 그래서 보정 픽셀을 기본으로 두고 탭 진입 시 한 번 맞춘다.
let hgCoordDefaultApplied = false;
function applyHgCoordDefault() {
  if (hgCoordDefaultApplied) return;
  hgCoordDefaultApplied = true;
  send('HG_COORD_MODE ' + chArg() + ' ' + hgCoordModeArg('undistort'));
}
// 카메라가 마지막으로 확인해 준 좌표계. 번들의 coord_mode 는 여기서만 온다.
let hgCoordModeActive = null;
function handleHgCoordMode(line) {
  if (!line.includes('[hg-coord]')) return;
  const box = document.getElementById('hgCoordStatus');
  if (line.includes('SUCCESS')) {
    const undist = line.includes('undistort');
    hgCoordModeActive = undist ? 'undistort' : 'raw';
    document.getElementById('hgCoordMode').value = hgCoordModeActive;
    box.textContent = undist ? 'K/dist 보정 좌표계 적용됨 — H를 다시 계산하세요.' : 'raw 픽셀 좌표계 적용됨 — H를 다시 계산하세요.';
  } else box.textContent = line;
}

// H를 h8=1 로 정규화. 카메라가 주는 H는 정규화돼 있지 않을 수 있고, 서버·QT는
// h8=1 을 전제하므로 내보내기 직전에만 맞춘다(내부 좌표 변환은 원본을 쓴다).
function normH(flat) {
  if (!Array.isArray(flat) || flat.length < 9) return null;
  const s = Number(flat[8]);
  if (!Number.isFinite(s) || Math.abs(s) < 1e-18) return flat.slice(0, 9);
  return flat.slice(0, 9).map(v => v / s);
}

// IVA_SYNC 결과(앵커 hull, raw 픽셀) 렌더. 값은 chHg[ch].ivaSync에 캐시되고
// (handleIvaSync가 채움, syncCalibFromChannel이 채널 전환 때 다시 그려줌),
// pushIvaArea()가 화면에 보이는 것과 정확히 같은 객체를 /iva_push로 되먹인다
// -- 서버가 계산을 다시 하지 않으므로 "화면에 보인 것 = 반영되는 것"이 항상
// 성립해야 한다.
function renderIvaSync(d) {
  const box = document.getElementById('ivaSyncStatus');
  const pushBtn = document.getElementById('ivaPushBtn');
  if (!box) return;
  if (!d) {
    box.innerHTML = '<span class="none">다각형 계산을 누르세요</span>';
    if (pushBtn) pushBtn.disabled = true;
    return;
  }
  if (!d.ok) {
    box.innerHTML = '<span class="qtitle" style="color:var(--red)">계산 실패</span> ' +
                    (d.reason || '알 수 없는 원인');
    if (pushBtn) pushBtn.disabled = true;
    return;
  }
  if (pushBtn) pushBtn.disabled = false;
  const pts = (d.points || []).map(p => `(${p.x.toFixed(0)},${p.y.toFixed(0)})`).join(' ');
  box.innerHTML = `<span class="qtitle" style="color:var(--green)">계산 완료</span> ` +
                  `ch${d.ch} ${d.points.length}점<br><span class="desc">${pts}</span>`;
}

function handleIvaSync(line) {
  const m = line.match(/^\\[calib\\] IVA_SYNC (\\{.*\\})$/);
  if (!m) return;
  let d;
  try { d = JSON.parse(m[1]); } catch (_) { return; }
  if (d.ch === undefined) return;
  chHg[d.ch] = chHg[d.ch] || emptyChHg();
  chHg[d.ch].ivaSync = d;
  if (d.ch !== curCh()) return;   // 지금 보이는 채널이 아니면 화면은 그대로 둔다
  renderIvaSync(d);
}

function pushIvaArea() {
  const ch = curCh();
  const d = chHg[ch] && chHg[ch].ivaSync;
  if (!d || !d.ok) { alert('먼저 [다각형 계산]을 눌러 hull을 받아오세요.'); return; }
  if (!confirm(`ch${ch}의 WiseAI IVA 영역을 지금 계산된 ${d.points.length}점 ` +
              '다각형으로 덮어씁니다. 계속할까요?'))
    return;
  const box = document.getElementById('ivaPushStatus');
  if (box) box.innerHTML = '<span class="none">반영 중...</span>';
  fetch('/iva_push', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({ch: ch, points: d.points, enable: true}),
  })
    .then(r => r.json())
    .then(res => {
      if (!box) return;
      box.innerHTML = res.ok
        ? `<span class="qtitle" style="color:var(--green)">반영 완료</span> ${res.detail || ''}`
        : `<span class="qtitle" style="color:var(--red)">반영 실패</span> ${res.detail || ''}`;
    })
    .catch(e => { if (box) box.innerHTML = `<span class="qtitle" style="color:var(--red)">요청 실패</span> ${e}`; });
}

// 최근 IVA_EVENT 기록 (링버퍼). 새로고침하면 사라진다 -- 카메라 쪽에 이력이
// 없으니(이벤트가 오는 순간만 안다) 여기서도 영구 보관할 근거가 없다. 채널별로
// 나누지 않는다 -- IVA_SYNC의 chHg[ch]와 달리 이 로그는 여러 채널이 섞여도
// 문제없고(ch는 그냥 테이블의 한 열), 오히려 채널 전환할 때마다 로그가 사라지면
// 더 불편하다. 알람/소리 액션은 아직 여기 안 붙어있다 -- 릴레이냐 스피커냐부터
// 미정이라, 지금은 파이프라인이 눈에 보이는 것까지만.
const ivaEventLog = [];
const IVA_EVENT_LOG_MAX = 30;

function handleIvaEvent(line) {
  // 한 테이블에 두 갈래를 모은다 -- WiseAI 자신의 판정([iva] EVENT)과 카메라 앱이
  // 발끝으로 다시 내린 판정([iva] ZONE). 나란히 놓아야 둘이 갈리는 지점이 보이고,
  // 그 갈림이 ZONE 쪽이 존재하는 이유다 (서버 쪽 ZONE_EVENT 분기 주석 참고).
  let d, src;
  let m = line.match(/^\\[iva\\] EVENT (\\{.*\\})$/);
  if (m) {
    src = 'WiseAI';
  } else {
    m = line.match(/^\\[iva\\] ZONE (\\{.*\\})$/);
    if (!m) return;
    src = '발끝';
  }
  try { d = JSON.parse(m[1]); } catch (_) { return; }

  // 시각은 카메라가 실제로 본 순간(t_ms)을 우선한다 -- 도착 시각은 링크가 밀린
  // 만큼 늦다. t_ms가 없는 이벤트(카메라가 프레임 시각을 안 실어준 경우)만
  // 도착 시각으로 떨어지고, 그 사실이 보이도록 '~'를 붙인다.
  const tf = ms => {
    const dt = new Date(ms);
    return dt.toLocaleTimeString() + '.' + String(dt.getMilliseconds()).padStart(3, '0');
  };
  const shown = (d.t_ms !== undefined) ? tf(d.t_ms)
              : (d.rx_ms !== undefined) ? '~' + tf(d.rx_ms)
              : '~' + new Date().toLocaleTimeString();
  ivaEventLog.unshift({...d, src, t: shown});
  if (ivaEventLog.length > IVA_EVENT_LOG_MAX) ivaEventLog.length = IVA_EVENT_LOG_MAX;

  const box = document.getElementById('ivaEventLog');
  if (!box) return;
  if (ivaEventLog.length === 0) {
    box.innerHTML = '<span class="none">아직 이벤트 없음</span>';
    return;
  }
  // left/top/right/bottom are present only when the camera's recent-bbox
  // cache had a hit for this object_id (see RecentWiseAiObjectBbox() on the
  // camera side) -- absent, not zero, on a miss. '-' rather than blank so a
  // scanning eye can tell "no match" apart from a column that failed to
  // render.
  const dim = 'color:var(--fg-dim,#888)';
  // 필드가 없는 것과 0인 것은 다르다 -- 없는 쪽은 '-'로 비워둬야 "매칭 실패"와
  // "화면 왼쪽 위"가 구분된다. 두 이벤트 종류가 서로 다른 필드를 채우므로
  // (WiseAI는 rule/state, 발끝은 foot_*) 빈칸은 정상이다.
  const cell = (v, fmt) => (v === undefined)
    ? `<td style="text-align:center;${dim}">-</td>` : `<td>${fmt ? fmt(v) : v}</td>`;
  const bboxCell = e => (e.left === undefined)
    ? `<td colspan="4" style="text-align:center;${dim}">-</td>`
    : `<td>${e.left.toFixed(0)}</td><td>${e.top.toFixed(0)}</td>` +
      `<td>${e.right.toFixed(0)}</td><td>${e.bottom.toFixed(0)}</td>`;
  const footCell = e => (e.foot_u === undefined)
    ? `<td style="text-align:center;${dim}">-</td>`
    : `<td>${e.foot_u.toFixed(0)},${e.foot_v.toFixed(0)}</td>`;
  const worldCell = e => (e.foot_wx === undefined)
    ? `<td style="text-align:center;${dim}">-</td>`
    : `<td>${e.foot_wx.toFixed(0)},${e.foot_wy.toFixed(0)}</td>`;
  // 카메라가 본 시각과 파이가 받은 시각의 차 = 링크 지연. 둘 다 NTP 동기된
  // 시계라 뺄셈이 성립한다. 음수가 나오면 그 전제가 깨진 것(둘 중 하나가 동기를
  // 잃음)이므로 숨기지 않고 그대로 보여준다 -- 조용히 0으로 만들면 시계가
  // 틀어진 걸 영영 모르게 된다.
  const lagCell = e => (e.t_ms === undefined || e.rx_ms === undefined)
    ? `<td style="text-align:center;${dim}">-</td>`
    : `<td${(e.rx_ms - e.t_ms) > 1000 ? ' style="color:var(--red)"' : ''}>` +
      `${e.rx_ms - e.t_ms}ms</td>`;
  const rows = ivaEventLog.map(e => {
    const tone = e.action === 'Enter' ? 'var(--red)' : 'var(--fg-dim, #888)';
    // 출처를 색으로도 구분 -- 한 테이블에 섞여 있어 열을 읽지 않고도 갈라 보게.
    const stone = e.src === '발끝' ? 'var(--accent, #4a9)' : 'var(--fg-dim, #888)';
    return `<tr><td>${e.t}</td><td style="color:${stone}">${e.src || '?'}</td>` +
           `<td>ch${e.ch}</td>${cell(e.rule)}` +
           `<td style="color:${tone}">${e.action}</td><td>${e.object_id}</td>` +
           `${cell(e.state, v => String(v))}${lagCell(e)}${footCell(e)}${worldCell(e)}${bboxCell(e)}</tr>`;
  }).join('');
  box.innerHTML = `<table style="width:100%;font-size:0.9em"><thead><tr>` +
                  `<th>시각</th><th>출처</th><th>ch</th><th>rule</th><th>action</th>` +
                  `<th>object_id</th><th>state</th><th>지연</th><th>발끝(px)</th><th>발끝(mm)</th>` +
                  `<th>left</th><th>top</th><th>right</th><th>bottom</th>` +
                  `</tr></thead><tbody>${rows}</tbody></table>`;
}

// 검출 위치가 이만큼 지나면 오버레이에서 지운다. WiseAI 가 사람을 놓치면
// WISEAI_DET 도 그냥 끊기므로(“사라졌다”는 메시지는 따로 없다), 마지막으로 본
// 시각을 기준으로 스스로 흐려져야 화면에 유령이 남지 않는다.
const WDET_TTL_MS = 1500;

function handleWiseAiDet(line) {
  if (line.indexOf('[det] ') !== 0) return;
  let d;
  try { d = JSON.parse(line.slice(6)); } catch (_) { return; }
  if (d.type !== 'WISEAI_DET') return;
  const st = chOverlay[d.ch];
  if (!st) return;
  st.wdets[d.object_id] = {
    foot_u: d.foot_u, foot_v: d.foot_v, foot_r: d.foot_r,
    inside: d.inside, zone_d: d.zone_d,
    zone_mm: d.zone_mm, level: d.level,
    left: d.left, top: d.top, right: d.right, bottom: d.bottom,
    ts: Date.now(),
  };
  if (rawOverlayOn && d.ch === curCh()) redrawRawCanvas();
}

function handleZoneBand(line) {
  const p = '[calib] ZONE_BAND ';
  if (line.indexOf(p) !== 0) return;
  let d;
  try { d = JSON.parse(line.slice(p.length)); } catch (_) { return; }
  const st = chOverlay[d.ch];
  if (!st) return;
  st.bands[d.kind] = { mm: d.mm, points: d.points || [] };
  if (rawOverlayOn && d.ch === curCh()) redrawRawCanvas();
}

function handleZoneBandsState(line) {
  const p = '[calib] ZONE_BANDS ';
  if (line.indexOf(p) !== 0) return;
  let d;
  try { d = JSON.parse(line.slice(p.length)); } catch (_) { return; }
  const btn = document.getElementById('zoneBandsBtn');
  if (btn) {
    btn.textContent = d.on ? '완충 밴드 끄기' : '완충 밴드 켜기';
    btn.classList.toggle('on', !!d.on);
  }
  const dEl = document.getElementById('zoneDangerMm');
  const wEl = document.getElementById('zoneWarnMm');
  if (dEl && d.danger_mm !== undefined) dEl.value = d.danger_mm;
  if (wEl && d.warn_mm !== undefined) wEl.value = d.warn_mm;
  // 껐으면 캐시도 비운다 -- 안 그러면 마지막 밴드가 화면에 남아 살아있는 척한다.
  if (!d.on) for (const k in chOverlay) chOverlay[k].bands = {};
  if (rawOverlayOn) redrawRawCanvas();
}

function handleIvaZoneSet(line) {
  const p = '[calib] IVA_ZONE_SET ';
  if (line.indexOf(p) !== 0) return;
  let d;
  try { d = JSON.parse(line.slice(p.length)); } catch (_) { return; }
  if (!chHg[d.ch]) return;
  // IVA_SYNC 와 같은 칸에 넣는다 -- 오버레이 입장에서 둘은 구분할 이유가 없다.
  chHg[d.ch].ivaZone = d;
  if (rawOverlayOn && d.ch === curCh()) redrawRawCanvas();
}

// 마커 검출 탭의 바닥 투영 오버레이가 쓰는 값. 카메라가 분해해 준 H_marker를
// 그대로 쓴다 — 부호 처리와 퇴화 판정이 이미 거기서 끝나 있다.
let mpPlane = null;   // {H_marker:[9], height_mm, ratio, ...}

function handleMarkerPlane(line) {
  const m = line.match(/^\\[calib\\] MARKER_PLANE (\\{.*\\})$/);
  if (!m) return;
  let d;
  try { d = JSON.parse(m[1]); } catch (_) { return; }
  // 카메라가 보내는 msg 전체를 그대로 실어 보내므로(파이썬 쪽 json.dumps(msg)),
  // ch는 이미 d.ch 로 들어 있다 — 이 핸들러만 따로 정규식으로 뽑을 필요가
  // 없다. undefined(옛 프로토콜 흔적)면 무시.
  if (d.ch === undefined) return;
  const parsed = (d.type === 'MARKER_PLANE' && d.ready &&
                  Array.isArray(d.H_marker) && d.H_marker.length >= 9)
    ? {H: d.H_marker.map(Number), height_mm: Number(d.height_mm), ratio: Number(d.ratio)}
    : null;
  if (d.type === 'MARKER_PLANE') chHg[d.ch].mpPlane = parsed;

  if (d.ch !== curCh()) return;  // 아래는 전부 "지금 보이는 채널" 화면 갱신
  if (d.type === 'MARKER_PLANE') {
    mpPlane = parsed;
    redrawRawCanvas();
    // 캘리브 완료 배너가 떠 있으면 world 좌표 판정을 이 답으로 다시 그린다.
    // 배너를 그릴 당시엔 아직 몰랐던 값이라, 이게 없으면 준비가 끝난 뒤에도
    // "world 좌표는 안 나갑니다"가 화면에 남는다. (2026-08-11)
    const hb = document.getElementById('hgStatus');
    if (hb && lastHgSuccessBase && hb.innerHTML.indexOf('캘리브 완료') >= 0)
      hb.innerHTML = lastHgSuccessBase + hgWorldClaim();
  }

  const box = document.getElementById('mpStatus');
  if (!box) return;

  if (d.type === 'MARKER_PLANE_SAVE') {
    box.innerHTML = d.ok
      ? `<span class="qtitle" style="color:var(--green)">저장됨</span> 마커 높이 ${d.height_mm} mm — 재부팅 후에도 유지됩니다`
      : '<span class="qtitle" style="color:var(--red)">저장 실패</span> 카메라 /mnt 쓰기 오류';
    return;
  }

  const inp = document.getElementById('mpHeight');
  if (inp && d.height_mm !== undefined && document.activeElement !== inp)
    inp.value = d.height_mm;

  if (!d.ready) {
    box.innerHTML = '<span class="qtitle" style="color:var(--red)">시차 보정 불가</span> ' +
                    (d.reason || '알 수 없는 원인');
    return;
  }
  // 필드 이름 주의 (2026-08-11 수정): 카메라 ReportMarkerPlane() 이 보내는 것은
  //   camera_z_mm   = 줄자로 잰 값 (0 = 아무도 안 쟀음)
  //   derived_z_mm  = H 분해가 함의하는 값
  //   nadir_mm      = [x, y] 배열
  // 여기서는 camera_z_measured_mm / camera_z_used_mm / nadir_x_mm / nadir_y_mm /
  // ratio 를 읽고 있었는데 그런 필드는 카메라에 없다 — 실측값이 역산값 자리에
  // 표시되고, 나디르와 보정비율은 NaN 이 찍히고 있었다. ratio 도 카메라가 안
  // 보내므로 표시된 두 수(h, Cz)로 여기서 나눈다.
  const camInp = document.getElementById('mpCamHeight');
  if (camInp && d.camera_z_mm !== undefined && document.activeElement !== camInp)
    camInp.value = d.camera_z_mm;

  const measured = Number(d.camera_z_mm || 0);
  const derived  = Number(d.derived_z_mm);
  const used     = measured > 0 ? measured : derived;
  const nadir    = Array.isArray(d.nadir_mm) ? d.nadir_mm : [NaN, NaN];
  const ratio    = used > 0 ? Number(d.height_mm) / used : NaN;

  // 역산 대 실측의 괴리가 K/H 품질을 판정하는 가장 유용한 한 숫자다.
  let camRow;
  if (measured > 0) {
    const gap = derived - measured;
    const pct = measured !== 0 ? (100 * gap / measured) : 0;
    const tone = Math.abs(pct) > 15 ? 'var(--red)' : Math.abs(pct) > 5 ? '#f59e0b' : 'var(--green)';
    camRow =
      `<tr><td>카메라 높이 (실측)</td><td><b>${measured.toFixed(1)} mm</b> — 보정에 사용됨</td></tr>` +
      `<tr><td>카메라 높이 (역산)</td><td>${derived.toFixed(1)} mm ` +
      `<b style="color:${tone}">(${gap >= 0 ? '+' : ''}${gap.toFixed(0)} mm, ${pct >= 0 ? '+' : ''}${pct.toFixed(1)}%)</b></td></tr>` +
      `<tr><td></td><td class="hint">괴리가 클수록 K 또는 H가 부정확하다는 뜻입니다. ` +
      `실측값을 쓰므로 시차 보정 자체는 이 오차의 영향을 받지 않습니다.</td></tr>`;
  } else {
    camRow =
      `<tr><td>카메라 높이 (역산)</td><td><b>${derived.toFixed(1)} mm</b> — 보정에 사용됨</td></tr>` +
      `<tr><td></td><td class="hint">⚠️ 실측값 미입력. <b>줄자로 재서 위에 넣으세요</b> — ` +
      `역산값이 틀리면 보정량이 같은 비율로 틀립니다.</td></tr>`;
  }

  box.innerHTML =
    `<span class="qtitle" style="color:var(--green)">H_marker 도출됨</span>` +
    `<table>` +
    `<tr><td>마커 높이</td><td><b>${Number(d.height_mm).toFixed(1)} mm</b> (입력값)</td></tr>` +
    camRow +
    `<tr><td>나디르</td><td>(${Number(nadir[0]).toFixed(1)}, ${Number(nadir[1]).toFixed(1)}) mm</td></tr>` +
    `<tr><td>보정 비율 h/Cz</td><td>${ratio.toFixed(4)} (h=${Number(d.height_mm).toFixed(0)} ÷ Cz=${used.toFixed(0)} mm)</td></tr>` +
    `</table>` +
    `<div class="hint">나디르에서 1000 mm 떨어진 지점의 마커는 약 ` +
    `<b>${(ratio * 1000).toFixed(0)} mm</b> 밀려 보입니다.</div>`;
}

// 로봇 마커 평면(시차 보정). 카메라가 H를 분해해 낸 값을 그대로 보여준다 —
// 브라우저는 계산하지 않는다.
// 두 명령 다 카메라가 성공/실패를 대시보드로 회신하지 않는다(printf 로 카메라
// stdout 에만 남는다). 그래서 보낸 뒤 곧바로 되읽어서, 화면에 뜨는 값이 "내가 친
// 값"이 아니라 "카메라가 실제로 물고 있는 값"이 되게 한다. (2026-08-11)
function applyMarkerHeight() {
  const v = parseFloat(document.getElementById('mpHeight').value);
  if (!(v >= 0)) { alert('높이는 0 이상의 mm 값이어야 합니다'); return; }
  send('MARKER_HEIGHT ' + v);      // 채널 없음이 맞다 — 로봇 마커는 4렌즈 공용
  sendCh('MARKER_PLANE_QUERY');
}
// CAMERA_HEIGHT 는 <ch> <mm> 두 인자를 받는다(sample_component.cc 의
// sscanf "%d %lf" 가 2를 요구). 채널 없이 "CAMERA_HEIGHT 1756" 을 보내면 1756 이
// 채널 번호로 읽히고 mm 이 비어 sscanf 가 1을 반환 — 카메라는 사용법만 자기
// stdout 에 찍고 조용히 버린다. 그래서 이 버튼은 여태 아무 일도 하지 않았고,
// camera_z_mm 은 계속 0(미측정)이었다. HG_COORD_MODE 가 문자열을 보내서 늘
// 무시되던 것과 같은 종류의 버그. (2026-08-11)
function applyCameraHeight() {
  const v = parseFloat(document.getElementById('mpCamHeight').value);
  if (!(v >= 0)) { alert('카메라 높이는 0 이상의 mm 값이어야 합니다 (0 = 미측정)'); return; }
  send('CAMERA_HEIGHT ' + chArg() + ' ' + v);
  sendCh('MARKER_PLANE_QUERY');
}
// POS 미리보기. 카메라가 [central_tls_sender.cpp] 에서 만드는 줄과 같은 모양을
// 그대로 쓴다 -- 여기서 새로 규격을 정하지 않는다. 값은 지금 흐르는 CAM_POSE 의
// raw 코너를 그대로 넣는다(같은 픽셀이 실제로 POS 로도 나간다).
function fillCentralPosSample() {
  const note = document.getElementById('ctPosNote');
  const box = document.getElementById('ctPos');
  const idBox = document.getElementById('ctId');
  const want = idBox ? String(Number(idBox.value)) : '';
  const c = (want && curOverlay().rawFrame[want]) ? curOverlay().rawFrame[want] : null;
  const q = c ? c.map(pt => [Number(Number(pt[0]).toFixed(2)), Number(Number(pt[1]).toFixed(2))])
              : [[0, 0], [0, 0], [0, 0], [0, 0]];
  // ch 는 카메라가 실제로 싣는 필드다(central_tls_sender_send_pos → payload.ch,
  // sample_component.cc 가 ch+1 로 넘기므로 **1-based**). 이 상자가 "카메라가
  // 실제로 내보내는 줄"을 보여준다고 적어놓고 ch 를 빼먹고 있었다 — 서버
  // 프로토콜을 이 표본으로 확인하는 사람에게는 없는 필드로 보인다.
  // 코너는 지금 고른 채널의 프레임에서 가져오므로 ch 도 그 채널이어야 맞다.
  box.value = JSON.stringify({type: 'POS', seq: 0,
                              payload: {ch: curCh() + 1, corners: q}});
  const chNote = ` (ch=${curCh() + 1} — CH${curCh() + 1}, 1-based)`;
  if (!idBox || !want) lastPosBaseNote = '대상 id를 알 수 없습니다.' + chNote;
  else if (c) lastPosBaseNote = `id ${want} 의 현재 프레임 코너를 넣었습니다.` + chNote;
  else lastPosBaseNote = `id ${want} 가 지금 화면에 없습니다 — 코너는 0 자리표시자입니다.` + chNote;
  refreshCentralPosNote();
}

// 마지막으로 채운 표본에 대한 설명. dynROI 추적 id가 바뀌면 표본은 그대로여도
// "지금 POS가 나가는가"에 대한 답이 달라지므로 문구만 따로 다시 그린다.
let lastPosBaseNote = '';

// POS는 중앙 대상 id(CENTRAL_ID) 하나에 대해서만 나간다. 그런데 dynROI를
// "특정 ID만"으로 좁히면 그 목록 밖의 마커는 아예 검출되지 않는다 — 중앙 대상
// id가 목록에 없으면 POS는 오류 없이 그냥 끊긴다. 카메라도 서버도 아무 말을
// 하지 않고, 이 상자는 여전히 그럴듯한 줄을 보여준다. 그 조합을 여기서 잡는다.
// (2026-08-11)
function refreshCentralPosNote() {
  const note = document.getElementById('ctPosNote');
  if (!note) return;
  const idBox = document.getElementById('ctId');
  const want = idBox && idBox.value !== '' ? Number(idBox.value) : null;
  let warn = '';
  if (dynRoiTrackIds.length) {
    const list = dynRoiTrackIds.join(',');
    warn = (want !== null && dynRoiTrackIds.indexOf(want) < 0)
      ? `<br><b class="bad">dynROI가 id ${list} 만 추적하고 있어 중앙 대상 id ${want} 는 검출되지 않습니다 — POS가 나가지 않습니다.</b>` +
        ` 「모든 마커」로 되돌리거나 추적 목록에 ${want} 를 넣으세요.`
      : `<br><span class="warn">dynROI 추적 대상: id ${list}</span> — 이 목록 밖의 마커는 검출되지 않습니다.`;
  }
  note.innerHTML = lastPosBaseNote + warn;
}
function copyCentralPos() {
  const box = document.getElementById('ctPos');
  if (!box.value) fillCentralPosSample();
  navigator.clipboard.writeText(box.value).then(
    () => { document.getElementById('ctPosNote').textContent = '복사했습니다.'; },
    () => { document.getElementById('ctPosNote').textContent = '복사 실패 — 직접 선택해 복사하세요.'; });
}

// 마지막으로 자동 생성한 calib_id. 조작자가 손으로 적은 값과 구별하기 위한 것이다.
// lastAutoStamp/Seq 는 같은 초에 여러 번 발행할 때 뒤에 붙일 일련번호용이다.
let lastAutoCalibId = null, lastAutoStamp = null, lastAutoSeq = 0;

// ===== 중앙 서버(9000) 탭 =====
// The camera is authoritative here too: every CENTRAL_* command is answered
// with a full CENTRAL_STATUS, so the box below only ever renders what the
// camera reports back.
function handleCentralStatus(line) {
  const m = line.match(/^\\[central\\] STATUS (\\{.*\\})$/);
  if (!m) return;
  let s;
  try { s = JSON.parse(m[1]); } catch (_) { return; }
  const LINKS = {
    online:      ['연결됨', '#2e7d32'],
    connecting:  ['TCP 연결 중…', '#ef6c00'],
    handshaking: ['TLS 핸드셰이크 중…', '#ef6c00'],
    offline:     ['끊김 (재시도 중)', '#c62828'],
    disabled:    ['꺼짐 (수동)', '#6b6b6b'],
  };
  const [label, color] = LINKS[s.link] || [s.link || '알 수 없음', '#c62828'];
  const posLabel = s.pos_on
      ? `<b style="color:#2e7d32">전송 중</b> — id ${s.marker_id}`
      : `<b style="color:#6b6b6b">중지됨</b> (대상 id ${s.marker_id})`;
  const note = s.detail ? ` — ${s.detail}` : '';
  document.getElementById('centralStatus').innerHTML =
      `<div>링크: <b style="color:${color}">${label}</b> (${s.server || '-'})</div>` +
      `<div>POS: ${posLabel}</div>` +
      (s.action ? `<div class="sub">최근 동작: ${s.action}${note}</div>` : '');
  centralLinkOn = !!s.link_on;
  centralPosOn = !!s.pos_on;
  paintCentralToggle('ctLinkBtn', '연결', centralLinkOn);
  paintCentralToggle('ctPosBtn', 'POS 전송', centralPosOn);
  const idBox = document.getElementById('ctId');
  if (document.activeElement !== idBox) idBox.value = s.marker_id;
  // 대상 id가 바뀌면 dynROI 추적 목록과의 충돌 여부도 달라진다.
  refreshCentralPosNote();
}
// Toggles mirror the camera's reported state, never a local guess: the button
// label only changes once CENTRAL_STATUS confirms the switch actually flipped.
let centralLinkOn = true, centralPosOn = true;
function paintCentralToggle(id, label, on) {
  const b = document.getElementById(id);
  if (!b) return;
  b.firstChild.nodeValue = `${label} ${on ? 'ON' : 'OFF'}`;
  b.classList.toggle('on', on);
}
function toggleCentralLink() {
  if (centralLinkOn && !confirm('중앙 서버 연결을 끊을까요? 다시 켤 때까지 POS가 전송되지 않습니다.')) return;
  send('CENTRAL_LINK ' + (centralLinkOn ? 0 : 1));
}
function toggleCentralPos() { send('CENTRAL_POS ' + (centralPosOn ? 0 : 1)); }
function sendCentralId() {
  const v = document.getElementById('ctId').value.trim();
  if (v === '' || !Number.isInteger(Number(v))) { alert('마커 ID를 정수로 입력하세요.'); return; }
  send('CENTRAL_ID ' + Number(v));
}
// 3x3 row-major flat array -> nested rows, the shape the server's calib schema
// uses. Returns null so callers can fall back to a placeholder.
function rows3(flat) {
  if (!Array.isArray(flat) || flat.length < 9) return null;
  const v = flat.map(Number);
  if (v.some(x => !Number.isFinite(x))) return null;
  return [v.slice(0, 3), v.slice(3, 6), v.slice(6, 9)];
}

// ISO 8601 with the LOCAL utc offset, e.g. 2026-07-29T14:18:03+09:00.
// toISOString() would give Z, which is valid ISO but loses the site's local
// time -- and the spec's own example is written with an offset.
function isoWithOffset(d) {
  const p = (n, w) => String(Math.abs(n)).padStart(w || 2, '0');
  const off = -d.getTimezoneOffset();                 // 분, 동쪽이 +
  const sign = off >= 0 ? '+' : '-';
  return `${d.getFullYear()}-${p(d.getMonth() + 1)}-${p(d.getDate())}` +
         `T${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}` +
         `${sign}${p(off / 60 | 0)}:${p(off % 60)}`;
}

// 앵커 실측 좌표가 도는 범위 + 마커 안쪽 여백 → 판 크기. 앵커는 검정 사각형
// 중심이고 폼보드 모서리에서 흰색/2 만큼 안쪽에 있으므로, 양쪽으로 그만큼 되돌려야
// 판 크기가 된다. 기본값 60mm 는 이 프로젝트의 100/120mm 마커 기준이며, 다른
// 배치면 조작자가 입력칸에서 고치라고 값을 채워만 둔다.
function guessCanvasMm() {
  if (!Array.isArray(hgCameraAnchors) || !hgCameraAnchors.length) return null;
  const xs = hgCameraAnchors.map(a => Number(a.wx));
  const ys = hgCameraAnchors.map(a => Number(a.wy));
  const inset = 60;
  const w = Math.max.apply(null, xs) - Math.min.apply(null, xs) + 2 * inset;
  const h = Math.max.apply(null, ys) - Math.min.apply(null, ys) + 2 * inset;
  return (Number.isFinite(w) && Number.isFinite(h) && w > 0 && h > 0) ? [w, h] : null;
}

// QT-REQ-CCTV-001 rev.2 §3. 최상위 평면 스키마이고 H 는 하나뿐이다.
// 서버는 이 객체를 가공 없이 보관·중계하므로, 여기 담기는 내용이 곧 QT 가 보는
// 내용이다 -- 자리표시자를 남기면 그게 저장된 캘리브레이션이 된다.
// outId/noteId 로 출력 위치만 갈아끼운다(2026-08-12). 호모그래피 탭의 floor·Odometry
// 서브탭에도 같은 "서버 송신" 블록을 두는데, 조립 로직을 복사하지 않기 위해서다 —
// 이미 이 파일엔 조립 함수가 둘(template/legacy) 있고, coord_mode 버그를 양쪽에
// 똑같이 두 번 고쳐야 했다. 형식이 갈라지는 걸 사람 규칙으로 막지 않는다.
// 입력칸(ctCalibId/ctCanvasW/ctCanvasH)은 서버송신 탭에 그대로 두고 여기서 읽는다 —
// 탭은 display:none 으로 숨을 뿐 DOM 에는 항상 있으므로 어느 탭에서 눌러도 읽힌다.
function fillCentralHmatrixTemplate(outId, noteId) {
  outId = outId || 'ctHm';
  noteId = noteId || 'ctHmNote';
  const missing = [], violations = [];

  const idBox = document.getElementById('ctCalibId');
  // R-8: 재발행마다 달라야 한다. 예전에는 "비어 있을 때만" 채워서, [채우기]를 두 번
  // 누르면 같은 calib_id 가 그대로 나갔다 -- 서버·QT 는 그 둘을 같은 번들로 본다.
  //
  // 그래서 자동 생성값이면 매번 새로 만들고, 조작자가 직접 적은 값은 건드리지 않는다.
  // 마지막 자동값을 기억해 두고 현재 입력과 비교하는 방식이다(빈 칸도 자동으로 본다).
  // 초까지 넣는 이유: 분 단위면 같은 분 안의 재발행이 또 충돌한다.
  if (idBox && (!idBox.value.trim() || idBox.value.trim() === lastAutoCalibId)) {
    const d = new Date(), p = n => String(n).padStart(2, '0');
    const stamp =
      `${d.getFullYear()}-${p(d.getMonth() + 1)}-${p(d.getDate())}-` +
      `${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}`;
    // 같은 초 안에 두 번 누르면 초까지 넣어도 문자열이 같아진다. 그때는 일련번호를
    // 붙여 반드시 달라지게 한다 -- R-8 은 "재발행마다 변경" 이므로, 시각이 같다는
    // 이유로 같은 id 가 나가면 서버·QT 는 두 번들을 같은 것으로 본다.
    // 시각 문자열을 정규식으로 다시 쪼개려 하면 시각 부분과 일련번호를 구분할 수
    // 없다(둘 다 '-숫자' 꼬리). 그래서 직전 시각과 카운터를 따로 들고 비교한다.
    if (stamp === lastAutoStamp) {
      lastAutoCalibId = stamp + '-' + (++lastAutoSeq);
    } else {
      lastAutoStamp = stamp;
      lastAutoSeq = 0;
      lastAutoCalibId = stamp;
    }
    idBox.value = lastAutoCalibId;
  }
  const calibId = idBox ? idBox.value.trim() : '';
  if (!calibId) missing.push('calib_id');

  const wBox = document.getElementById('ctCanvasW');
  const hBox = document.getElementById('ctCanvasH');
  if (wBox && hBox && !wBox.value && !hBox.value) {
    const g = guessCanvasMm();
    if (g) { wBox.value = g[0]; hBox.value = g[1]; }
  }
  const canvas = [Number(wBox && wBox.value), Number(hBox && hBox.value)];
  if (!(canvas[0] > 0 && canvas[1] > 0)) missing.push('canvas_mm (작업영역 크기)');

  const K = kCalib
    ? [[kCalib.fx, 0, kCalib.cx], [0, kCalib.fy, kCalib.cy], [0, 0, 1]]
    : (missing.push('K/dist (마커 검출 탭에서 CALIB_K_QUERY)'),
       [[1400, 0, 960], [0, 1400, 540], [0, 0, 1]]);
  const D = kCalib ? kCalib.dist.slice(0, 5) : [0, 0, 0, 0, 0];

  // R-1: H[2][2] = 1 로 정규화해서 보낼 것.
  const H = rows3(normH(hgHfloor));
  if (!H) missing.push('H (호모그래피 탭에서 H 계산)');

  // H_marker: 로봇 위에 달린 마커를 넣으면 차체가 실제로 선 바닥점이 나오는 H.
  // H 와 같은 픽셀 공간(coord_mode)을 입력으로 받고 출력도 같은 월드 mm 다 --
  // 다른 건 어느 평면을 기준으로 삼았는지뿐이다.
  //
  // 카메라가 H 를 분해해 만든 값을 그대로 싣는다(브라우저는 재계산하지 않는다).
  // undistort 모드에서만 도출되므로, raw 로 찍힌 번들에는 애초에 들어갈 수 없다.
  const Hm = mpPlane ? rows3(normH(mpPlane.H)) : null;
  if (!Hm)
    missing.push('H_marker (로봇 마커 평면 — "로봇 마커 평면" 패널에서 MARKER_HEIGHT 적용 필요)');
  else if (!(Number(mpPlane.height_mm) > 0))
    missing.push('marker_height_mm (마커 높이가 0 — 시차 보정이 꺼진 것과 같다)');

  // R-2 는 이 요청서의 최우선 항목이다. raw 로 피팅한 H 를 보내면 QT 가 왜곡
  // 보정을 켜는 순간 좌표가 발산한다 -- 에러가 아니라 그림이 깨져서 나타난다.
  const coordMode = hgCoordModeForBundle();
  if (!coordMode) missing.push('coord_mode (카메라가 아직 H 를 보고하지 않았고 HG_COORD_MODE 이력도 없음)');
  else if (coordMode !== 'undistort')
    violations.push(`R-2 위반: coord_mode=${coordMode} — undistort 로 재피팅이 필요합니다`);

  // R-4: 카메라 실제 해상도와 일치해야 한다.
  // 기준영상 수신 경로가 이 대시보드에는 없다(스냅샷 패널 제거). 예전엔 "카메라
  // raw는 1920×1080 고정"이라 가정하고 하드코딩했는데, ArucoPosePNM 실제
  // 해상도(2592×1520)와 달라서 틀린 값을 실어 보내고 있었다 (2026-08-10 수정).
  // 확실한 값이 아니므로 여전히 마커검출 탭의 #camRes 선택값(기본 2592×1520)을
  // 쓰되, 확인된 값이 아니라는 점은 그대로 남긴다.
  const size = [camRes().w, camRes().h];
  missing.push(`image_size (기준영상 없음 — ${size[0]}×${size[1]} 으로 가정)`);

  const bundle = {
    // ch는 payload 최상위에 실어야 한다(docs/PROTOCOL.md, docs/08.06/
    // CCTV_ACTION_ITEMS_20260806.md C-3) — 안 실으면 서버가 전부 채널 1로
    // 보고, 4채널을 캘리해도 마지막 하나만 남는다. 그때의 자동 경로
    // (push_calib_to_server)는 이미 이렇게 하고 있었는데, 이 수동 버튼은
    // 빠져 있었다(2026-08-11 발견).
    //
    // 2026-08-14: 자동 경로는 카메라로 옮겨졌고(SendFloorBundle) 이 조립기는
    // **탈출구로만** 남는다 — 서버가 요구하는 형식이 바뀌었는데 카메라를 아직
    // 못 고친 상황 같은 때. 평소 캘리에는 쓰지 말 것. 여기서 채우는
    // image_size/coord_mode/K/D 는 브라우저의 추측이고, 카메라가 보내는 것은
    // 사실이다. 이걸 보내면 카메라가 올린 정본을 덮는다.
    ch:         curCh() + 1,   // 1-based (SELECT_CHANNEL과 동일 규약)
    calib_id:   calibId || 'MISSING',
    created_at: isoWithOffset(new Date()),
    image_size: size,
    coord_mode: coordMode || 'unknown',
    unit:       'mm',                       // R-5 고정
    K: K,
    D: D,
    H: H || [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
    // R-0: 옛 서버는 H_floor 만 인식했다(H 단일 키는 파싱 실패로 번들 폐기). 현재 서버는
    // H_floor 를 먼저 보고 없을 때 H 로 넘어가므로, 같은 값을 두 이름으로 실으면 어느
    // 버전에서도 읽힌다. 값이 같으니 둘이 어긋날 여지도 없다.
    H_floor: H || [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
    H_marker: Hm || [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
    // 서버가 시차 보정에 쓰는 값. **mm 그대로** 싣는다 (2026-08-13 변경).
    //
    // 예전에는 이 필드만 m 로 바꿔 실었다("서버 스키마가 그렇다"). 그 근거가
    // 지금은 반대다: wire 규격, docs/CALIBRATION.md,
    // 서버팀 회신(REPLY_SERVER §166 "marker_height_mm처럼")이 전부 mm 이고,
    // 카메라가 주행 캘리에서 직접 조립해 올리는 번들(SendCalibBundle)도 이미
    // marker_height_mm 이다. m 로 쓰는 곳은 옛 TESTING.md 픽스처와 router.cpp
    // 주석뿐이라 잔재로 판단했다.
    //
    // 고치는 이유는 "규격에 맞추려고"가 아니라 **우리 두 경로가 서로 달랐기
    // 때문**이다. 같은 대시보드가 만드는 번들과 카메라가 만드는 번들이 같은
    // 값을 다른 이름·다른 단위로 실어 보내고 있었고, 어느 쪽이든 읽는 쪽
    // 하나는 0 을 받는다 — 시차 보정이 조용히 꺼진다.
    marker_height_mm: mpPlane ? Number(mpPlane.height_mm) : 0,
    origin_mm:  [0, 0],                     // R-6: 월드 원점 = 폼보드 좌하단
    canvas_mm:  (canvas[0] > 0 && canvas[1] > 0) ? canvas : [0, 0],
    axis:       'x_right_y_up',             // R-6 고정
  };
  const out = document.getElementById(outId);
  if (out) out.value = JSON.stringify(bundle, null, 2);

  const note = document.getElementById(noteId);
  if (!note) return;
  const parts = [];
  if (violations.length)
    parts.push('<b style="color:var(--red)">' + violations.join('<br>') + '</b>');
  if (missing.length)
    parts.push('<b style="color:var(--red)">자리표시자 포함 — 전송 전에 채우세요:</b> ' +
               missing.join(' · '));
  if (!violations.length && !missing.length)
    parts.push('<b style="color:var(--green)">QT-REQ-CCTV-001 rev.2 형식으로 채웠습니다.</b> ' +
               `coord_mode=${coordMode}, canvas ${canvas[0]}×${canvas[1]}mm`);
  // A-2 는 이 화면이 답할 수 없다. 검증점이 피팅에서 빠져 있어야 하고 그 마커가
  // 실제로 보여야 나오는 수치인데, 여기서는 어느 쪽도 확인할 방법이 없다.
  parts.push('※ 수락기준 <b>A-2(검증점 재투영 RMS ≤ 5mm)</b>는 번들에 들어가지 않습니다 — ' +
             '검증 마커가 보이는 배치에서 따로 측정해 회신하세요.');
  note.innerHTML = parts.join('<br>');
}

function fillCentralHmatrixLegacy() {
  // Built from whatever the camera has actually reported this session, not a
  // dummy identity. The server PERSISTS this bundle as the user's calibration
  // and relays it to QT, so shipping placeholder numbers is not a harmless
  // default -- it silently becomes the calibration everyone downstream trusts.
  const missing = [];

  const K = kCalib
    ? [[kCalib.fx, 0, kCalib.cx], [0, kCalib.fy, kCalib.cy], [0, 0, 1]]
    : (missing.push('K/dist (마커 검출 탭에서 CALIB_K_QUERY)'),
       [[1400, 0, 960], [0, 1400, 540], [0, 0, 1]]);
  const D = kCalib ? kCalib.dist.slice(0, 5) : [0, 0, 0, 0, 0];

  const Hf = rows3(hgHfloor);
  if (!Hf) missing.push('H_floor (호모그래피 탭에서 H 계산)');

  const Hm = mpPlane ? rows3(mpPlane.H) : null;
  if (!Hm) missing.push('H_marker (로봇 마커 평면 — 높이 적용 필요)');

  const coordMode = hgCoordModeForBundle();
  if (!coordMode) missing.push('coord_mode (카메라가 아직 H 를 보고하지 않았고 HG_COORD_MODE 이력도 없음)');

  const bundle = {
    // ch는 calib "안"이 아니라 payload 최상위(calib과 형제)에 실어야 한다
    // (docs/PROTOCOL.md, C-3) — 2026-08-11 발견, fillCentralHmatrixTemplate와
    // 같은 이유로 여기도 빠져 있었다.
    ch: curCh() + 1,   // 1-based
    calib: {
      version: 1,
      // QT checks these two before anything else: without them it has to guess
      // which pixel space H expects and which frame size K belongs to.
      // Was hardcoded [1920,1080] ("camera raw is fixed FHD") -- wrong for
      // ArucoPosePNM (2592x1520). Now takes the 마커검출 tab's #camRes pick
      // (default 2592x1520). See fillCentralHmatrixTemplate's R-4 comment.
      coord_mode: coordMode || 'unknown',
      image_size: [camRes().w, camRes().h],
      K: K,
      D: D,
      H_floor:  Hf || [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
      H_marker: Hm || [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
      // mm 그대로 — fillCentralHmatrixTemplate 의 같은 필드 주석 참고 (2026-08-13).
      marker_height_mm: mpPlane ? Number(mpPlane.height_mm) : 0
    }
  };
  document.getElementById('ctHm').value = JSON.stringify(bundle, null, 2);

  const note = document.getElementById('ctHmNote');
  if (note) {
    note.innerHTML = missing.length
      ? '<b style="color:var(--red)">자리표시자 포함 — 전송 전에 채우세요:</b> ' + missing.join(' · ')
      : '<b style="color:var(--green)">카메라의 현재 값으로 채웠습니다.</b> ' +
        `coord_mode=${coordMode}, 마커 높이 ${mpPlane.height_mm} mm`;
  }
}
function sendCentralHmatrix(outId) {
  const box = document.getElementById(outId || 'ctHm');
  if (!box) { alert('보낼 번들 상자를 찾지 못했습니다.'); return; }
  const raw = box.value.trim();
  let obj;
  // Parse before sending: the camera only checks that it starts with '{', and
  // a malformed bundle would be stored by the server as the user's calibration.
  try { obj = JSON.parse(raw); } catch (e) { alert('JSON 파싱 실패: ' + e.message); return; }
  if (!obj || typeof obj !== 'object' || Array.isArray(obj)) { alert('payload는 JSON 객체여야 합니다.'); return; }
  const compact = JSON.stringify(obj);
  // Camera-side ceiling is CENTRAL_TLS_MAX_LINE (2048) for the whole line;
  // leave room for the command word and the envelope it gets wrapped in.
  if (compact.length > 1700) { alert('payload가 너무 깁니다 (' + compact.length + 'B). 1700B 이하로 줄이세요.'); return; }
  // coord_mode is the one field whose absence is silently wrong downstream:
  // QT applies undistortPoints before H, so a bundle whose H was fitted on raw
  // pixels produces a plausible map that is off by the lens distortion.
  // Two schemas ship from this box -- the flat QT-REQ-CCTV-001 one and the
  // legacy {"calib":{...}} -- so look in both places rather than assuming.
  const mode = obj && (obj.coord_mode || (obj.calib && obj.calib.coord_mode));
  if (mode !== 'undistort' &&
      !confirm(`coord_mode 가 "${mode || '없음'}" 입니다.\\n\\n` +
               'QT는 undistort 파이프라인을 쓰므로 이대로 보내면 ' +
               '조용히 왜곡만큼 틀린 좌표가 됩니다.\\n그래도 전송할까요?'))
    return;
  if (!confirm('이 번들이 서버에 저장되고 QT로 중계됩니다. 전송할까요?')) return;
  send('CENTRAL_HMATRIX ' + compact);
}

// 상단에서 고른 채널에만 적용 (DYNROI_CH <ch> <on> <margin> <maxMiss> — 카메라는
// 이미 렌즈별로 지원하는데(호모그래피용 렌즈는 앵커를 하나도 놓치면 안 되니 켜면
// 안 되고, 로봇을 쫓는 렌즈는 켜야 하는 식) 대시보드에 그 경로가 없었다. ch가
// DYNROI_CH의 첫 인자라 sendCh()(끝에 붙임) 대신 직접 조립한다. (2026-08-11)
function applyDynRoi() {
  const on = document.getElementById('dynRoiChk').checked ? 1 : 0;
  const m  = document.getElementById('dynMargin').value || 240;
  const n  = document.getElementById('dynMaxMiss').value || 4;
  send('DYNROI_CH ' + chArg() + ' ' + on + ' ' + m + ' ' + n);
}
// 지금 입력값을 4채널 모두에 한 번에 적용하고 싶을 때(예: 호모그래피 계산 전
// 전 채널 껐다 켜기). DYNROI_CH와 별개로 전역 DYNROI 명령은 여전히 존재한다.
function applyDynRoiAllChannels() {
  const on = document.getElementById('dynRoiChk').checked ? 1 : 0;
  const m  = document.getElementById('dynMargin').value || 240;
  const n  = document.getElementById('dynMaxMiss').value || 4;
  if (!confirm('지금 값(사용 ' + (on ? 'ON' : 'OFF') + ', margin ' + m + 'px, 실패허용 ' + n + ')을 4채널 전부에 적용할까요?')) return;
  send('DYNROI ' + on + ' ' + m + ' ' + n);
}

// 추적 대상 id. 빈 목록 = 모든 마커(카메라 기본값)이라, "모든 마커" 라디오는
// 인자 없는 DYNROI_IDS 를 보낸다. 카메라가 파싱한 결과를 ACK 로 돌려주므로
// 화면 표시는 여기서 낙관적으로 갱신하지 않고 ACK([dynroi] IDS)에 맡긴다.
function readDynRoiIds() {
  const raw = document.getElementById('dynRoiIds').value || '';
  const seen = {}, out = [];
  // 쉼표든 공백이든 받는다 — 현장에서 둘 다 자연스럽게 입력된다.
  for (const tok of raw.split(/[^0-9]+/)) {
    if (!tok) continue;
    const v = Number(tok);
    if (!Number.isInteger(v) || v < 0 || seen[v]) continue;
    seen[v] = 1;
    out.push(v);
    if (out.length >= 16) break;   // 카메라의 DYNROI_MAX_TRACK_IDS 와 같은 상한
  }
  return out;
}
function onDynRoiIdsInput() {
  // 숫자를 치기 시작하면 "특정 ID만" 으로 자동 전환 — 입력해놓고 라디오를 안 눌러
  // 아무 일도 안 일어나는 게 가장 흔한 실수다. 전송은 하지 않는다(포커스 유지).
  const ids = document.getElementById('dynRoiIds').value.trim();
  if (ids) document.getElementById('dynRoiPick').checked = true;
}
function applyDynRoiIds() {
  const pick = document.getElementById('dynRoiPick').checked;
  const ids = pick ? readDynRoiIds() : [];
  const st = document.getElementById('dynRoiIdsState');
  if (pick && !ids.length) {
    // 빈 목록을 그대로 보내면 카메라는 "전체"로 알아듣는다. 조작자 의도와
    // 반대이므로 보내지 않고 왜 안 보냈는지 알린다.
    if (st) st.innerHTML = '<b style="color:var(--red)">추적할 ID를 입력하세요</b> — 비어 있으면 전체와 같아집니다';
    return;
  }
  send('DYNROI_IDS' + (ids.length ? ' ' + ids.join(' ') : ''));
}

// 동적 ROI 설정(켜짐/margin/실패허용)도 이제 렌즈별이다(DYNROI_CH, 2026-08-11
// — 전엔 DYNROI가 4채널에 한 번에 적용되는 것뿐이라 dynRoiOn이 전역이었다).
// chDynRoiCfg가 채널별 캐시, dynRoiOn은 "지금 선택된 채널" 별칭으로 예전처럼
// 남겨뒀다(kCalib/hgHfloor와 같은 이유) — syncCalibFromChannel()이 채널 전환
// 때마다 동기화한다. TRACK/SEARCH(DYNROI_STATE)는 항상 렌즈별 독립이라 그대로
// chOverlay[ch]에 담는다.
function emptyDynRoiCfg() { return { enabled: false, margin: 240, maxMiss: 4 }; }
// 카메라가 실제로 파싱해 적용한 추적 id 목록([dynroi] IDS ACK). 빈 배열 = 제한 없음.
// 화면의 입력칸이 아니라 이 값을 쓰는 이유: 조작자가 치다 만 글자가 아니라 카메라가
// 받아들인 값이라야 "지금 POS 가 나가는가"를 판정할 수 있다. DYNROI_IDS 는 아직
// 채널별이 아니라 4채널 공통이라 전역 하나로 둔다. (2026-08-11)
let dynRoiTrackIds = [];
let chDynRoiCfg = {0: emptyDynRoiCfg(), 1: emptyDynRoiCfg(), 2: emptyDynRoiCfg(), 3: emptyDynRoiCfg()};
let dynRoiOn = false;

// 채널 전환 시(overlay 재렌더와 별개로) #dynRoiState 텍스트를 "지금 선택된
// 채널"의 이미 알고 있는 상태로 즉시 맞춘다 — 다음 DYNROI_STATE 줄이 그
// 채널에서 도착할 때까지 이전 채널 문구가 남아있지 않도록.
// 카드 제목이 "상단에서 고른 채널"이라고만 하고 그게 몇 번인지는 안 적혀 있었다.
// 상태줄이 그 답까지 같이 하도록 CH 번호를 앞에 붙인다 — 헤더 알약과 카드가 서로
// 다른 채널을 말하고 있는지 한눈에 보이는 유일한 자리다. (2026-08-11)
function renderDynRoiState() {
  const el = document.getElementById('dynRoiState');
  if (!el) return;
  const ch = curCh();
  const head = `CH${ch + 1} 상태: `;
  if (!dynRoiOn) {
    // "—" 는 꺼졌다는 뜻인지 아직 모른다는 뜻인지 구별이 안 됐다. 캐시에 값이
    // 들어온 뒤라면 OFF 라고 단정해서 말한다.
    const known = !!chDynRoiCfg[ch];
    el.textContent = head + (known ? 'OFF (전체 화면 검출)' : '— (조회 중)');
    el.style.color = '';
    return;
  }
  const ov = curOverlay();
  if (ov.dynRoiTracking) {
    el.textContent = head + 'TRACK' +
      (ov.dynRoiTrackSize ? '  ' + ov.dynRoiTrackSize[0] + '×' + ov.dynRoiTrackSize[1] : '');
    el.style.color = '#28a745';
  } else {
    el.textContent = head + 'SEARCH (재탐색 — 전체 프레임을 훑습니다)';
    el.style.color = '#dc3545';
  }
  renderChStatusTable();  // 알약 tooltip 의 TRACK/SEARCH 도 같이 최신화
}

function handleDynRoi(line) {
  if (line.indexOf('[dynroi]') < 0) return;
  // 추적 대상 id — 카메라가 실제로 파싱한 결과다. 화면의 라디오/입력칸을 여기에
  // 맞춰, 조작자가 친 값이 아니라 카메라가 받아들인 값이 보이게 한다. (전역 —
  // DYNROI_IDS도 4채널 전부에 적용된다.)
  const idm = line.match(/^\\[dynroi\\] IDS (\\[.*\\])$/);
  if (idm) {
    let ids;
    try { ids = JSON.parse(idm[1]); } catch (_) { return; }
    if (!Array.isArray(ids)) return;
    dynRoiTrackIds = ids;
    if (typeof refreshCentralPosNote === 'function') refreshCentralPosNote();
    const st = document.getElementById('dynRoiIdsState');
    const pick = document.getElementById('dynRoiPick');
    const all = document.getElementById('dynRoiAll');
    const box = document.getElementById('dynRoiIds');
    if (ids.length) {
      if (pick) pick.checked = true;
      // 입력 중인 칸을 덮어쓰지 않는다 — 타이핑 도중 ACK가 오면 커서가 튄다.
      if (box && document.activeElement !== box) box.value = ids.join(',');
      if (st) st.innerHTML = `추적 대상: <b>id ${ids.join(',')}</b> — 나머지 마커는 검출되지 않습니다`;
    } else {
      if (all) all.checked = true;
      if (st) st.textContent = '추적 대상: 모든 마커';
    }
    return;
  }
  // TRACK/SEARCH: DYNROI_STATE, ch= 붙어 나온다(2026-08-10 카메라 쪽 수정).
  // ch 없는 줄은 옛 프로토콜이라 무시 — 어느 렌즈인지 모르고 전역에 반영하면
  // 다시 채널이 섞인다.
  const chm = line.match(/ch=(\\d+)/);
  if ((line.indexOf('TRACK') >= 0 || line.indexOf('SEARCH') >= 0) && !chm) return;
  if (line.indexOf('TRACK') >= 0) {
    const ch = Number(chm[1]);
    const ov = chOverlay[ch];
    if (!ov) return;
    ov.dynRoiTracking = true;
    const used = line.match(/margin=(\\d+)px/);
    if (used) ov.dynRoiMargin = Number(used[1]);
    const m = line.match(/\\((\\d+),(\\d+)\\)\\s*(\\d+)x(\\d+)/);
    ov.dynRoiTrackSize = m ? [m[3], m[4]] : null;
    // 알약은 TRACK/SEARCH 로 색이 갈리므로 지금 보고 있지 않은 채널도 갱신한다 —
    // 헤더 표는 4채널을 한꺼번에 보여주는 자리인데 현재 채널만 살아 있으면
    // 나머지 셋은 마지막으로 봤을 때의 색으로 굳는다. (2026-08-11)
    renderChStatusTable();
    if (ch === curCh()) { renderDynRoiState(); if (rawOverlayOn) redrawRawCanvas(); }
  } else if (line.indexOf('SEARCH') >= 0) {
    const ch = Number(chm[1]);
    const ov = chOverlay[ch];
    if (!ov) return;
    ov.dynRoiTracking = false;
    ov.dynRoiTrackSize = null;
    renderChStatusTable();
    if (ch === curCh()) { renderDynRoiState(); if (rawOverlayOn) redrawRawCanvas(); }
  } else if (line.indexOf('OFF') >= 0 || line.indexOf('ON') >= 0) {
    // ReportDynRoi()가 채널마다 따로 이 줄을 보낸다(DYNROI_CH 한 채널만 바꿔도,
    // 전역 DYNROI로 4채널 다 바꿔도 매번 4줄) — ch 없는 줄(옛 흔적)은 무시,
    // 있으면 그 채널 것만 갱신한다. 전역 DYNROI로 4채널을 한 번에 바꾼 경우도
    // 결국 이 채널별 처리 4번으로 자연히 다 맞게 반영된다.
    if (!chm) return;
    const ch = Number(chm[1]);
    const on = line.indexOf('ON') >= 0 && line.indexOf('OFF') < 0;
    const cfg = chDynRoiCfg[ch] || emptyDynRoiCfg();
    cfg.enabled = on;
    const m = line.match(/max_margin=(\\d+)px/);
    if (m) cfg.margin = Number(m[1]);
    const mm = line.match(/max_miss=(\\d+)/);
    if (mm) cfg.maxMiss = Number(mm[1]);
    chDynRoiCfg[ch] = cfg;
    renderChStatusTable();
    // configure()는 다시 SEARCH부터 시작한다(DynRoiTracker::configure —
    // "Always restarts from a full search"), 그 채널의 라이브 추적 표시도
    // 같이 리셋한다.
    const ov = chOverlay[ch];
    if (ov) { ov.dynRoiTracking = false; ov.dynRoiTrackSize = null; if (m) ov.dynRoiMargin = Number(m[1]); }
    if (ch === curCh()) {
      dynRoiOn = cfg.enabled;
      const chk = document.getElementById('dynRoiChk');
      const mg = document.getElementById('dynMargin');
      const mmEl = document.getElementById('dynMaxMiss');
      if (chk) chk.checked = cfg.enabled;
      if (mg && document.activeElement !== mg) mg.value = cfg.margin;
      if (mmEl && document.activeElement !== mmEl) mmEl.value = cfg.maxMiss;
      renderDynRoiState();
      if (rawOverlayOn) redrawRawCanvas();
    }
  }
}

// 마커 중심 = 네 코너 평균. raw 오버레이가 중심점·id 라벨을 찍는 데 쓴다.
function markerCenter(c) {
  let x = 0, y = 0;
  for (let i = 0; i < 4; i++) { x += Number(c[i][0]); y += Number(c[i][1]); }
  return [x / 4, y / 4];
}
let rawOverlayOn = false;
const rawCanvas = document.getElementById('rawCanvas');
const rawCtx = rawCanvas ? rawCanvas.getContext('2d') : null;

let zoneBandsOn = false;
function toggleZoneBands() {
  zoneBandsOn = !zoneBandsOn;
  send('ZONE_BANDS ' + (zoneBandsOn ? '1' : '0'));
  // 버튼 문구는 카메라 응답(handleZoneBandsState)이 확정한다 -- 여기서 먼저
  // 바꾸면 명령이 실패했을 때 화면만 켜진 척한다.
}

function sendZoneMargin() {
  const d = Number(document.getElementById('zoneDangerMm').value);
  const w = Number(document.getElementById('zoneWarnMm').value);
  if (!(d >= 0 && w > d && w <= 20000)) {
    alert('접근금지 < 주의 여야 하고, 0..20000mm 범위여야 합니다');
    return;
  }
  send('ZONE_MARGIN ' + d + ' ' + w);
}

function toggleRawOverlay() {
  rawOverlayOn = !rawOverlayOn;
  const b = document.getElementById('rawOverlayBtn');
  b.textContent = rawOverlayOn ? '오버레이 정지' : '오버레이 보기 시작';
  b.classList.toggle('on', rawOverlayOn);
  if (rawOverlayOn && showUndist && !kCalib) sendCh('CALIB_K_QUERY');
  // 검출 위치 피드는 이 오버레이가 켜져 있을 때만 의미가 있다. 카메라 쪽
  // 기본값이 꺼짐이라, 켜고 끄는 책임을 화면에 둔다 -- 아무도 안 보는데 초당
  // 수십 건이 계속 흐르는 걸 막는 가장 단순한 방법이다.
  send('DET_STREAM ' + (rawOverlayOn ? '1' : '0'));
  if (!rawOverlayOn) for (const k in chOverlay) chOverlay[k].wdets = {};
  redrawRawCanvas();
}
function drawQuad(ctx, c, col, lw) {
  ctx.strokeStyle = col; ctx.lineWidth = lw;
  ctx.beginPath();
  for (let i = 0; i < 4; i++) {
    const x = Number(c[i][0]), y = Number(c[i][1]);
    i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
  }
  ctx.closePath(); ctx.stroke();
}
// 카메라가 다음 프레임에 훑을 범위의 근사. 이번 프레임 코너 전체를 감싸는
// 사각형 + margin 이며, 카메라의 TRACK 규칙과 같은 모양이다(합집합 + 여유).
function computeDynRoiBox(W, H) {
  const ov = curOverlay();
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity, n = 0;
  for (const id in ov.rawFrame) {
    for (const p of ov.rawFrame[id]) {
      const px = Number(p[0]), py = Number(p[1]);
      if (px < x0) x0 = px;  if (py < y0) y0 = py;
      if (px > x1) x1 = px;  if (py > y1) y1 = py;
      n++;
    }
  }
  if (!n) return null;
  let x = x0 - ov.dynRoiMargin, y = y0 - ov.dynRoiMargin;
  let w = (x1 - x0) + 2 * ov.dynRoiMargin, h = (y1 - y0) + 2 * ov.dynRoiMargin;
  const kMin = 80;
  if (w < kMin) { x -= (kMin - w) / 2; w = kMin; }
  if (h < kMin) { y -= (kMin - h) / 2; h = kMin; }
  const rx = Math.max(0, x), ry = Math.max(0, y);
  return {x: rx, y: ry,
          w: Math.min(W, x + w) - rx, h: Math.min(H, y + h) - ry};
}
// 오버레이 화면 안쪽 아래에 얹는 계기판. 표는 옆에 있지만, 영상을 보는 동안
// 눈을 옮기지 않고 읽으려면 그림 안에 있어야 한다.
//
// 글자 크기는 캔버스 픽셀 기준이다 -- 캔버스는 1849×1032 로 잡히고 브라우저가
// 축소해 보여주므로, CSS 픽셀 기준으로 고르면 실제 화면에서 읽을 수 없게 작아진다.
function drawRawHud(W, H) {
  const fps = metricFps(), det = metricDetCur(), hit = metricHitPct();
  const curProcs = curOverlay().procs;
  const proc = curProcs.length ? curProcs[curProcs.length - 1] : null;

  // 값이 없어도 칸을 비우지 않는다. 항목이 나타났다 사라지면 옆 항목이 밀려서
  // 숫자를 읽는 중에 자리가 바뀐다.
  const cells = [
    ['fps',  fps  === null ? '—' : fps.toFixed(1)],
    ['proc', proc === null ? '—' : proc + 'ms'],
    ['det',  det  === null ? '—' : det + 'ms'],
    ['검출', hit  === null ? '—' : Math.round(hit) + '%'],
    ['CPU',  (cpuApp === null ? '—' : Math.round(cpuApp) + '%') + ' / ' +
             (cpuSys === null ? '—' : Math.round(cpuSys) + '%')]
  ];

  // 글자·띠 크기는 화면 폭만으로 정한다. 예전처럼 내용 길이에 맞춰 줄이면 값이
  // 갱신될 때마다 글자 크기와 띠 폭이 같이 흔들려 오히려 읽기 어려웠다.
  const fs  = Math.max(14, Math.min(34, Math.round(W / 70)));
  const lfs = Math.max(11, Math.round(fs * 0.7));
  const pad = Math.round(fs * 0.6);
  const bh  = fs + pad * 2;
  const by  = H - bh - pad;
  const bx  = pad, bw = W - pad * 2;      // 띠는 항상 같은 자리·같은 크기

  rawCtx.fillStyle = 'rgba(0,0,0,0.62)';
  rawCtx.fillRect(bx, by, bw, bh);

  // 칸 폭이 고정이라 값이 길어져도 그 칸 안에서만 늘어나고 옆 칸을 밀지 않는다.
  // 하한(14px)이 있어 이론상 아직 넘칠 수 있으니 띠 안으로 클립해 둔다.
  rawCtx.save();
  rawCtx.beginPath();
  rawCtx.rect(bx, by, bw, bh);
  rawCtx.clip();
  rawCtx.textBaseline = 'middle';
  const cy = by + bh / 2;
  const cellW = (bw - pad * 2) / cells.length;
  for (let i = 0; i < cells.length; i++) {
    const x = bx + pad + cellW * i;
    rawCtx.font = lfs + 'px sans-serif';
    rawCtx.fillStyle = '#9ca3af';
    rawCtx.fillText(cells[i][0], x, cy);
    const lw = rawCtx.measureText(cells[i][0]).width;
    // 숫자는 등폭 글꼴로. 자리수가 바뀌어도 글자가 좌우로 움직이지 않는다.
    rawCtx.font = 'bold ' + fs + 'px ui-monospace, Menlo, Consolas, monospace';
    rawCtx.fillStyle = '#e5e7eb';
    rawCtx.fillText(cells[i][1], x + lw + Math.round(fs * 0.4), cy);
  }
  rawCtx.restore();
  rawCtx.textBaseline = 'alphabetic';               // 마커 라벨 기준선 복원
}

// IVA 존(고정 다각형)과 그 안팎을 오가는 발끝 원판(사람마다 하나)을 그린다.
// 이 둘은 다른 것이다 -- 존은 바닥에 정해둔 경보 구역이고, 원판은 사람 발이
// 차지하는 면적이다. 판정은 "원판이 존에 닿았는가"이고, 그래서 둘을 겹쳐
// 보여야 왜 그런 판정이 나왔는지가 눈으로 읽힌다.
function fillBand(ctx, pts, col) {
  if (!pts || pts.length < 3) return;
  ctx.beginPath();
  ctx.moveTo(pts[0].x, pts[0].y);
  for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i].x, pts[i].y);
  ctx.closePath();
  ctx.fillStyle = col;
  ctx.fill();
}

function drawIvaZone(ctx) {
  // 밴드를 먼저, 넓은 것부터 -- 좁은 밴드가 위에 덮여야 경계가 보인다. 화면에서
  // 이 띠는 카메라에 가까운 쪽이 넓고 먼 쪽이 좁게 나오는데, 그게 "바닥에서
  // 일정한 폭"의 올바른 투영이다.
  const bands = curOverlay().bands;
  if (bands) {
    fillBand(ctx, bands.warn && bands.warn.points, 'rgba(234,179,8,0.18)');    // 노랑 = 주의
    fillBand(ctx, bands.danger && bands.danger.points, 'rgba(239,68,68,0.22)'); // 빨강 = 접근금지
  }
  const hg = chHg[curCh()];
  const z = hg && (hg.ivaZone || hg.ivaSync);
  const pts = z && z.ok !== false && z.points;
  if (pts && pts.length >= 3) {
    ctx.strokeStyle = '#22c55e';            // 초록 = IVA 존
    ctx.lineWidth = 4;
    ctx.setLineDash([]);
    ctx.beginPath();
    ctx.moveTo(pts[0].x, pts[0].y);
    for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i].x, pts[i].y);
    ctx.closePath();
    ctx.stroke();
    ctx.fillStyle = 'rgba(34,197,94,0.10)';
    ctx.fill();
    ctx.fillStyle = '#22c55e';
    ctx.fillText('IVA 존', pts[0].x + 8, pts[0].y - 10);
  }

  const st = curOverlay();
  const now = Date.now();
  for (const id in st.wdets) {
    const w = st.wdets[id];
    if (now - w.ts > WDET_TTL_MS) { delete st.wdets[id]; continue; }
    // 안에 있으면 빨강, 밖이면 파랑. 존이 없는 채널이면 inside 가 undefined 라
    // 회색 -- "밖"과 "판정할 존이 없음"은 다른 상태다.
    // level: 3 존 내부, 2 접근금지(<=danger), 1 주의(<=warn), 0 정상.
    // level 이 없으면(호모그래피 없는 채널) 예전 2단계 판정으로 떨어진다.
    const col = (w.level !== undefined)
        ? (['#38bdf8', '#eab308', '#ef4444', '#dc2626'][w.level] || '#94a3b8')
        : (w.inside === undefined) ? '#94a3b8'
        : (w.inside ? '#ef4444' : '#38bdf8');
    if (w.left !== undefined) {          // bbox (얇게)
      ctx.strokeStyle = col; ctx.lineWidth = 1; ctx.setLineDash([4, 4]);
      ctx.strokeRect(w.left, w.top, w.right - w.left, w.bottom - w.top);
      ctx.setLineDash([]);
    }
    ctx.strokeStyle = col; ctx.lineWidth = 3;   // 발끝 원판
    ctx.beginPath(); ctx.arc(w.foot_u, w.foot_v, Math.max(2, w.foot_r), 0, 7); ctx.stroke();
    ctx.fillStyle = col;                        // 발끝 점
    ctx.beginPath(); ctx.arc(w.foot_u, w.foot_v, 5, 0, 7); ctx.fill();
    // 밀리미터가 있으면 그걸 보여준다 -- 픽셀 거리는 화면 위치에 따라 의미가
    // 달라져서 사람이 읽을 값이 못 된다.
    const txt = (w.zone_mm !== undefined)
        ? (w.zone_mm === 0 ? '존 내부' : (w.zone_mm / 1000).toFixed(2) + 'm')
        : 'r' + w.foot_r.toFixed(0) +
          (w.zone_d === undefined ? '' : ' · d' + w.zone_d.toFixed(0));
    ctx.fillText(txt, w.foot_u + 10, w.foot_v + 20);
  }
}

function redrawRawCanvas() {
  if (!rawCtx) return;
  // 캔버스 해상도: K가 있으면 (cx*2, cy*2), 없으면 #camRes 선택값(기본
  // 2592×1520). raw 코너는 풀프레임 픽셀 좌표라 캔버스가 프레임과 같은
  // 크기여야 위치가 맞는다.
  //
  // 단, K가 퇴화돼 있으면(Zhang 퇴화 등으로 fx/fy/cx/cy가 터무니없이 나온
  // 경우) cx*2/cy*2가 실제 해상도와 크게 어긋나 캔버스 비율 자체가 실제
  // 프레임과 달라진다 — 2026-08-11 CH1 실측: cx*2=2960/cy*2=1076인데 실제는
  // 2592×1520(가로세로 각각 +14%/-29% 오차)이라 오버레이가 눈에 띄게
  // 가로로 길어 보였다. 해상도 대비 30% 넘게 벗어나면 K 기반 크기를 버리고
  // #camRes 선택값을 쓴다 — 좌표(undistortPoints) 자체의 정확도는 K 품질에
  // 달려 있지만, 최소한 캔버스가 실제 프레임 비율을 벗어나진 않게 한다.
  const res = camRes();
  const kOk = kCalib &&
    Math.abs(kCalib.cx * 2 - res.w) < res.w * 0.3 &&
    Math.abs(kCalib.cy * 2 - res.h) < res.h * 0.3;
  const W = kOk ? Math.round(kCalib.cx * 2) : res.w;
  const H = kOk ? Math.round(kCalib.cy * 2) : res.h;
  if (rawCanvas.width !== W || rawCanvas.height !== H) { rawCanvas.width = W; rawCanvas.height = H; }
  rawCtx.clearRect(0, 0, W, H);
  if (!rawOverlayOn) return;
  const cmp = showUndist && kCalib;   // 보정 코너를 함께 그릴지
  rawCtx.font = '18px sans-serif';

  // 동적 ROI 박스 — 마커보다 먼저 그려 코너를 가리지 않게. 이 사각형이
  // "다음 프레임에 카메라가 실제로 훑는 범위"다 (밖은 아예 검출되지 않는다).
  if (dynRoiOn) {
    const box = computeDynRoiBox(W, H);
    if (box) {
      rawCtx.strokeStyle = '#a855f7';            // 보라 = 동적 ROI
      rawCtx.lineWidth = 3;
      rawCtx.setLineDash([10, 6]);
      rawCtx.strokeRect(box.x, box.y, box.w, box.h);
      rawCtx.setLineDash([]);
      const pct = (box.w * box.h) / (W * H) * 100;
      rawCtx.fillStyle = '#a855f7';
      rawCtx.fillText('동적 ROI ' + Math.round(box.w) + '×' + Math.round(box.h) +
                      ' (' + pct.toFixed(1) + '%)',
                      box.x + 6, Math.max(20, box.y - 8));
    } else if (!curOverlay().dynRoiTracking) {
      rawCtx.fillStyle = '#dc3545';
      rawCtx.fillText('SEARCH — 전체 화면 재탐색 중', 12, 26);
    }
  }
  // 마커보다 먼저 -- 동적 ROI 박스와 같은 이유로, 코너를 가리지 않게.
  drawIvaZone(rawCtx);

  for (const id in curOverlay().rawFrame) {
    const c = curOverlay().rawFrame[id];
    drawQuad(rawCtx, c, '#f59e0b', 3);              // raw = 주황
    const [rcx, rcy] = markerCenter(c);
    rawCtx.fillStyle = '#f59e0b';
    rawCtx.beginPath(); rawCtx.arc(rcx, rcy, 5, 0, 7); rawCtx.fill();
    if (cmp) {
      const u = c.map(p => undistortPixel(Number(p[0]), Number(p[1])));
      drawQuad(rawCtx, u, '#22d3ee', 3);            // 보정 = 청록
      rawCtx.strokeStyle = 'rgba(255,255,255,0.65)'; rawCtx.lineWidth = 1;
      rawCtx.setLineDash([4, 4]);                   // raw→보정 이동 점선
      for (let i = 0; i < 4; i++) {
        rawCtx.beginPath();
        rawCtx.moveTo(Number(c[i][0]), Number(c[i][1]));
        rawCtx.lineTo(u[i][0], u[i][1]);
        rawCtx.stroke();
      }
      rawCtx.setLineDash([]);
      const [ucx, ucy] = markerCenter(u);
      rawCtx.fillStyle = '#22d3ee';
      rawCtx.beginPath(); rawCtx.arc(ucx, ucy, 5, 0, 7); rawCtx.fill();
      const dpx = Math.hypot(ucx - rcx, ucy - rcy);
      rawCtx.fillStyle = '#f59e0b';
      rawCtx.fillText('id ' + id + ' · Δ' + dpx.toFixed(1) + 'px', rcx + 9, rcy - 9);
    } else {
      rawCtx.fillStyle = '#f59e0b';
      rawCtx.fillText('id ' + id, rcx + 9, rcy - 9);
    }
  }
  drawRawHud(W, H);
}

const es = new EventSource('/events');
// ArucoPosePNM의 DETECT는 채널마다 따로다(0-based ch). 4채널 전부 물어본다.
// CALIB_K_STATUS/QUERY도 채널 인자를 생략하면 카메라가 4채널을 전부 순회해서
// 답을 보낸다(ch가 붙어서 오므로 chKCalib에 채널별로 정확히 쌓인다) — 그래서
// 여기서는 굳이 채널을 안 붙이고 한 번에 전부 채운다.
es.onopen = () => {
  send('CALIB_K_STATUS'); send('CALIB_K_QUERY');
  // ⚠ 채널을 붙이지 말 것. 카메라의 HandleDetect 는
  //     int ch = -1, on = 1;
  //     if (sscanf(p + 6, "%d %d", &ch, &on) >= 1) SetDetectEnabled(ch, on != 0);
  // 이라서, "DETECT 0" 처럼 채널만 보내면 값이 생략된 걸 조회로 보지 않고
  // on 의 초기값 1 을 그대로 써서 그 채널을 켜 버린다. 예전에는 여기서
  // 4채널을 돌며 "DETECT <c>" 를 보냈는데, 그게 곧 "새로고침할 때마다 4채널
  // 전부 켜기"였다(2026-08-12 발견 — 카메라는 부팅 시 전부 꺼진 채로 시작하고
  // 헤더 알약으로만 켜도록 해 뒀는데, 페이지를 새로 고치는 것만으로 그 의도가
  // 매번 무효가 되고 있었다).
  //
  // 인자를 아예 안 붙이면 sscanf 가 0 을 반환해 설정을 건너뛰고
  // ReportDetect() 만 돈다. ReportDetect() 는 인자와 무관하게 4채널 전부를
  // 보고하므로, 이 한 줄로 조회 목적이 그대로 달성된다.
  //
  // 카메라 쪽 근본 수정(값이 있을 때만 설정)은 재빌드가 필요해 별도 건이다.
  send('DETECT');
  // 헤더 표의 dynROI 열이 마커검출 탭을 열기 전에도 채워지도록 (2026-08-11).
  // 바 DYNROI는 순수 조회다(카메라 쪽 버그 고침, HandleDynRoi 참고) — 4채널
  // 전부 응답이 온다.
  send('DYNROI');
  // HG_QUERY는 CALIB_K_QUERY와 달리 채널 인자가 필수라(생략하면 카메라가
  // 그냥 무시) 4번 나눠 보내야 4채널 전부 채워진다. 이게 없으면 새로고침
  // 직후엔 지금 보고 있는 채널조차 "현재 호모그래피 상태"가 조회 전
  // placeholder 그대로 남아, 마치 채널별로 안 나뉘는 것처럼 보였다(2026-08-11).
  for (let c = 0; c < 4; c++) send('HG_QUERY ' + c);
};
// 마커 검출 on/off. 상태는 절대 낙관적으로 바꾸지 않는다 - 카메라가 DETECT로
// 확인해 준 값만 반영한다. 링크가 끊겨 명령이 버려졌는데 버튼만 바뀌면 실제와
// 어긋난 상태를 보여주게 된다.
//
// 검출 on/off 는 오직 헤더 표의 채널별 알약(toggleChDetectOne)으로만 한다.
//
// 예전에는 헤더에 [검출] 버튼이 하나 있어서 누르면 4채널이 한꺼번에 켜지고
// 꺼졌다(2026-08-12 제거). 문제는 그게 "빠른 마스터 스위치"로 쓰이라고 만든
// 건데 실제로는 사고의 원인이었다는 것이다 — 카메라는 부팅 시 4채널 전부
// 꺼진 채로 시작하는데(sample_component.cc 의 detect_enabled_ 기본값 false,
// 부팅 직후 전탐색으로 CPU 가 포화되던 문제 때문에 2026-08-10 에 그렇게 바꿈),
// 이 버튼을 한 번 누르면 그 의도가 통째로 무효가 된다. 실제로 마커가 보이는
// 렌즈가 하나뿐인데도 4채널이 13시간 동안 켜진 채로 돌아 CPU 82%,
// 거버너가 duty 를 4로 나눠 채널당 15% 인 상태가 발견됐다(2026-08-12).
//
// 채널을 하나씩 켜는 건 몇 번 더 클릭하는 일이지만, 4채널을 한 번에 켜는 건
// 되돌리기 전까지 계속 비용을 무는 일이다. 그래서 후자를 없앴다.
let detectByCh = {};  // ch(0-based) -> 0|1. 알약 색과 마커검출 탭 안내가 이걸 본다.
function handleDetect(line) {
  const m = line.match(/ch(\\d+) detect_enabled=([01])/);
  if (!m) return;
  const ch = Number(m[1]);
  const wasCh = detectByCh[ch];
  const nowCh = m[2] === '1';
  detectByCh[ch] = nowCh;
  renderChStatusTable();
  if (wasCh === true && nowCh === false) {
    // 이 채널이 방금 꺼졌다 — 이 채널의 마지막 프레임 코너만 버린다. 남겨두면
    // 검출이 멈춘 뒤에도 오버레이에 옛 좌표가 그대로 떠 있어 살아있는 화면처럼
    // 보인다. 채널을 헤더 표에서 개별로 켜고 끌 수 있게 된 뒤로(2026-08-11)
    // 다른 채널은 여전히 검출 중일 수 있으므로 그 채널 버퍼만 지운다 — 예전
    // "4채널 전부 지우기"는 마스터 토글(전 채널 동시 off)에만 맞았다.
    const ov = chOverlay[ch];
    if (ov) { ov.rawFrame = {}; ov.rawBuilding = {}; ov.rawSeq = null; }
    if (rawOverlayOn && ch === curCh()) redrawRawCanvas();
  }
  if (rawOn) renderRaw();
}

// 헤더의 채널별 검출/dynROI 한눈에 보기 표(2026-08-11). 행은 JS로 한 번만
// 만들고(채널당 onclick에 정확한 ch를 심어야 해서 문자열 템플릿이 편하다),
// 이후엔 색만 갱신한다. detectByCh/chDynRoiCfg는 이미 위에서 선언돼 있다.
//
// 점(●) 대신 좌우로 긴 알약 모양 스위치로 — 작은 점보다 상태가 눈에 잘 띄고
// 클릭 영역도 넓어진다. dynROI 칸도 이제 클릭하면 그 채널만 토글된다(원래는
// 상태 표시만 했는데, 요청으로 검출 칸과 동일하게 만듦) — margin/실패허용은
// 그 채널에 캐시된 마지막 값(chDynRoiCfg[ch])을 그대로 쓰고 on/off만 바꾼다.
// 세부값을 조정하려면 여전히 마커검출 탭에서.
const CH_PILL_STYLE = 'display:inline-block;width:32px;height:14px;border-radius:7px;' +
                      'cursor:pointer;vertical-align:middle;transition:background-color .15s';
function toggleChDetectOne(ch) {
  send('DETECT ' + ch + ' ' + (detectByCh[ch] ? 0 : 1));
}
function toggleChDynRoiOne(ch) {
  const cfg = chDynRoiCfg[ch] || emptyDynRoiCfg();
  send('DYNROI_CH ' + ch + ' ' + (cfg.enabled ? 0 : 1) + ' ' + cfg.margin + ' ' + cfg.maxMiss);
}
// 채널이 열(chDetCell0..3/chRoiCell0..3, HTML에 이미 고정돼 있음), 검출/dynROI가
// 행이라 셀 자체를 알약으로 쓴다 — 별도 <span> 없이 td에 직접 스타일/onclick을
// 매번 다시 씀(멱등이라 매 렌더 호출마다 새로 걸어도 무해, 대신 행을 동적으로
// 만들 필요가 없어 더 단순하다). 2026-08-11: 원래 채널이 행이었는데 요청으로
// 뒤집음.
function renderChStatusTable() {
  for (let c = 0; c < 4; c++) {
    const detCell = document.getElementById('chDetCell' + c);
    const roiCell = document.getElementById('chRoiCell' + c);
    if (detCell) {
      detCell.innerHTML = `<span onclick="toggleChDetectOne(${c})" style="${CH_PILL_STYLE};background-color:${detectByCh[c] ? '#28a745' : '#8a8a8a'}" title="클릭해서 CH${c + 1} 검출 켜기/끄기 (여러 채널 동시에 켤 수 있음)"></span>`;
    }
    if (roiCell) {
      const cfg = chDynRoiCfg[c];
      const on = cfg && cfg.enabled;
      // 켜짐만으로는 부족하다 — SEARCH 인 dynROI 는 전체 프레임을 훑으므로 비용이
      // 꺼둔 것과 같은데, 알약은 초록으로 "되고 있다"고 말한다. TRACK 일 때만
      // 진한 초록, 켜졌지만 SEARCH 면 주황. (2026-08-11)
      const tracking = on && chOverlay[c] && chOverlay[c].dynRoiTracking;
      const color = !on ? '#8a8a8a' : (tracking ? '#28a745' : '#f0ad4e');
      const state = !on ? 'OFF' : (tracking ? 'TRACK' : 'SEARCH — 전체 프레임을 훑는 중');
      roiCell.innerHTML = `<span onclick="toggleChDynRoiOne(${c})" style="${CH_PILL_STYLE};background-color:${color}" title="CH${c + 1} 동적 ROI: ${state}&#10;클릭해서 켜기/끄기 (margin·실패허용은 마커검출 탭)"></span>`;
    }
  }
}
renderChStatusTable();

// 검출률 레벨 프리셋 — perim/ecc/poly 세 값을 한 번에 세팅한다. cctv_app의
// applyPreset() 그대로, sendCh()로만 바꿨다(2026-08-10) — ArucoPosePNM은 4렌즈라
// DETECT_PARAM도 채널 인자가 필요하고, 값 정의는 카메라가 아니라 여기 있으므로
// "중간이 무슨 값인지"를 바꾸려면 이 파일만 다시 올리면 된다(재빌드 불필요).
const DETECT_PRESETS = {
  base: {perim: 0.03,  ecc: 0.60, poly: 0.03,  label: '기본'},
  cons: {perim: 0.025, ecc: 0.70, poly: 0.035, label: '보수'},
  mid:  {perim: 0.018, ecc: 0.80, poly: 0.045, label: '중간'},
  aggr: {perim: 0.012, ecc: 0.90, poly: 0.06,  label: '공격'},
};
function applyDetectPreset(name) {
  const p = DETECT_PRESETS[name];
  if (!p) return;
  // 세 개를 순서대로 전송(순서 무관 — 서로 독립 파라미터). 카메라는 각각 ACK를 보낸다.
  const ch = chArg();
  send('DETECT_PARAM ' + ch + ' perim ' + p.perim);
  send('DETECT_PARAM ' + ch + ' ecc '   + p.ecc);
  send('DETECT_PARAM ' + ch + ' poly '  + p.poly);
  // 고급 입력칸도 프리셋 값으로 맞춰 둔다(이후 미세조정 기준점).
  document.getElementById('dpPerim').value = p.perim;
  document.getElementById('dpEcc').value   = p.ecc;
  document.getElementById('dpPoly').value  = p.poly;
  // 선택 레벨 강조 + 상태 표시.
  const btns = document.querySelectorAll('#detectLevels button[data-lvl]');
  btns.forEach(b => b.classList.toggle('active', b.getAttribute('data-lvl') === name));
  const st = document.getElementById('detectLevelState');
  if (st) st.textContent = '현재 레벨: ' + p.label +
    ' (perim ' + p.perim + ' / ecc ' + p.ecc + ' / poly ' + p.poly + ') — 재부팅 시 기본값 복귀';
}

// ===== 운영 탭 — 시간축 추이 (2026-08-12 추가) =====
//
// 기존 지연 표(handleLatencyCollect + chOverlay)는 "지금 값"을 보여주는
// PROC_WINDOW(120) 롤링 창이다. 이 탭은 같은 SSE 줄을 독립 버퍼에 시간축으로
// 쌓아 추이를 그린다.
//
// 왜 chOverlay 를 재사용하지 않는가: procs/dets/arrivals 는 매 줄 무조건 push 라
// 인덱스가 맞아 그대로 쓸 수 있지만, hits 는 seq 가 바뀐 줄에서만 push 되어
// arrivals 와 길이가 다르다. 맞추려면 handleLatencyCollect 를 고쳐야 하는데
// 그건 마커검출 탭의 지연 표가 쓰는 공유 코드다. 돌아가는 탭을 건드리지 않으려고
// 줄을 한 번 더 파싱한다 -- 정규식 몇 개가 늘 뿐이고, 그 대가로 이 탭은
// 기존 어떤 코드에도 의존하지 않는다(읽기조차 하지 않는다).
const OPS_WINDOW_MS = 30 * 60 * 1000;   // 보관 30분
const OPS_MAX       = 4000;             // 채널당 표본 상한(메모리 방어)
const OPS_CH_COLOR  = ['#4da3ff', '#ffa14d', '#5ad18a', '#d15a8a'];  // CH1..CH4
let opsLines  = {0: [], 1: [], 2: [], 3: []};   // 줄 단위   {t, proc, det}
let opsFrames = {0: [], 1: [], 2: [], 3: []};   // 프레임 단위 {t, hit}
let opsSeq    = {0: null, 1: null, 2: null, 3: null};
let opsCpu    = [];                             // {t, app, rss}
let opsWinMin = 5;
// 특정 마커 하나를 시간축으로 추적한다. 기본 49 = 로봇 마커(CENTRAL_ID 와 같은 값).
// 채널별로 따로 쌓는다 — 같은 id 가 두 렌즈에 동시에 보일 수 있고, 그걸 한 배열에
// 섞으면 좌표가 두 시점 사이를 튀어다니는 그래프가 된다.
let opsMarkerId = 49;
let opsMarks = {0: [], 1: [], 2: [], 3: []};    // {t, cs:[[x,y]x4], wx, wy, wdeg}
function opsSetMarkerId(v) {
  const n = Number(v);
  if (!isFinite(n) || n < 0) return;
  if (n === opsMarkerId) return;
  opsMarkerId = n;
  // id 를 바꾸면 이전 마커의 좌표는 의미가 없다. 남겨두면 다른 마커의 궤적이
  // 새 마커 것처럼 이어져 보인다.
  for (let c = 0; c < 4; c++) opsMarks[c] = [];
  opsDraw();
}

function opsTrim(arr, now) {
  const cut = now - OPS_WINDOW_MS;
  while (arr.length && arr[0].t < cut) arr.shift();
  while (arr.length > OPS_MAX) arr.shift();
}
function handleOps(line) {
  const cm = line.match(/\\[cpu\\] app=(-?[\\d.]+)%.*rss=(-?\\d+)KB/);
  if (cm) {
    const now = Date.now();
    const a = Number(cm[1]);
    opsCpu.push({t: now, app: a >= 0 ? a : null, rss: Number(cm[2])});
    opsTrim(opsCpu, now);
    return;
  }
  const pm  = line.match(/proc=(-?\\d+)ms/);
  const chm = line.match(/ch=(\\d+)/);
  // ch 없는 줄은 무시 -- 어느 렌즈인지 모르는 값을 아무 채널에 넣으면 그래프가
  // 조용히 섞인다. 이 파일의 다른 채널별 핸들러와 같은 규칙.
  if (!pm || !chm) return;
  const ch = Number(chm[1]);
  if (!(ch in opsLines)) return;
  const now = Date.now();
  const dm = line.match(/det=(-?\\d+)ms/);
  opsLines[ch].push({t: now, proc: Number(pm[1]), det: dm ? Number(dm[1]) : null});
  opsTrim(opsLines[ch], now);
  // 같은 프레임에 마커가 여러 개면 seq 가 같은 줄이 여러 번 온다. 프레임 fps 와
  // 검출률은 seq 당 한 번만 세어야 한다(기존 지연 표와 같은 규칙).
  const sm = line.match(/seq=(\\d+)/);
  const seq = sm ? Number(sm[1]) : null;
  if (seq === null || seq !== opsSeq[ch]) {
    opsFrames[ch].push({t: now, hit: line.includes('MARKER LOST') ? 0 : 1});
    opsTrim(opsFrames[ch], now);
  }
  if (seq !== null) opsSeq[ch] = seq;

  // 추적 대상 마커의 코너/월드 좌표. 줄 형식(라이브 gui.log 실측):
  //   seq=N ch=C id=D c0=(x,y) c1=(x,y) c2=(x,y) c3=(x,y) [world=(X,Ymm,Adeg)] proc=...
  // world 는 그 렌즈에 마커평면(H_marker)이 준비됐을 때만 붙는다 — 없으면
  // null 로 두어 선이 끊기게 한다(0 으로 채우면 원점에 있는 것처럼 보인다).
  const im = line.match(/\\bid=(\\d+)/);
  if (!im || Number(im[1]) !== opsMarkerId) return;
  const cs = [];
  for (let i = 0; i < 4; i++) {
    const g = line.match(new RegExp('c' + i + '=\\\\((-?[\\\\d.]+),(-?[\\\\d.]+)\\\\)'));
    if (!g) return;                       // 코너가 하나라도 빠지면 표본을 버린다
    cs.push([Number(g[1]), Number(g[2])]);
  }
  const wm = line.match(/world=\\((-?[\\d.]+),(-?[\\d.]+)mm,(-?[\\d.]+)deg\\)/);
  opsMarks[ch].push({t: now, cs: cs,
                     wx:   wm ? Number(wm[1]) : null,
                     wy:   wm ? Number(wm[2]) : null,
                     wdeg: wm ? Number(wm[3]) : null});
  opsTrim(opsMarks[ch], now);
}
function opsSetWin(v) { opsWinMin = Number(v); opsDraw(); }

// 표본을 시간 버킷으로 한 번에 접는다. 버킷마다 전체 배열을 훑으면 30분창 ×
// 수천 표본에서 초당 수백만 번 비교가 되므로 단일 패스로 넣는다.
function opsBuckets(pts, t0, t1, bucketMs) {
  const n = Math.max(1, Math.ceil((t1 - t0) / bucketMs));
  const b = new Array(n);
  for (let i = 0; i < n; i++)
    b[i] = {t: t0 + (i + 0.5) * bucketMs, n: 0, procSum: 0, procN: 0, procMax: null,
            detSum: 0, detN: 0, hit: 0};
  for (let i = 0; i < pts.length; i++) {
    const p = pts[i];
    const k = Math.floor((p.t - t0) / bucketMs);
    if (k < 0 || k >= n) continue;
    const c = b[k];
    c.n++;
    if (typeof p.proc === 'number' && isFinite(p.proc)) {
      c.procSum += p.proc; c.procN++;
      if (c.procMax === null || p.proc > c.procMax) c.procMax = p.proc;
    }
    if (typeof p.det === 'number' && isFinite(p.det)) { c.detSum += p.det; c.detN++; }
    if (typeof p.hit === 'number') c.hit += p.hit;
  }
  return b;
}
// 값이 없는 버킷은 null 로 둔다 -- 0 으로 채우면 "멈춘 구간"이 "0 이었던 구간"으로
// 보이고, 그 둘은 완전히 다른 사건이다.
function opsSeries(buckets, pick) {
  const out = new Array(buckets.length);
  for (let i = 0; i < buckets.length; i++) out[i] = {t: buckets[i].t, v: pick(buckets[i])};
  return out;
}
function opsNiceMax(v) {
  if (!(v > 0)) return 1;
  const e = Math.pow(10, Math.floor(Math.log10(v)));
  const m = v / e;
  return (m <= 1 ? 1 : m <= 2 ? 2 : m <= 5 ? 5 : 10) * e;
}
// 행 분리 렌더러. rows = [{label, series:[{pts,color,dash,width}], yMax, unit, value}]
//
// 한 캔버스에 여러 계열을 겹쳐 그리던 것을 행으로 쪼갠 이유(2026-08-12):
// 4채널을 겹쳐 그리면 어느 선이 어느 렌즈인지 범례를 봐야 알 수 있고, 채널마다
// 값의 크기가 달라(한 렌즈는 400ms, 다른 렌즈는 100ms) 공통 y축을 쓰면 작은 쪽이
// 바닥에 눌려 변화가 안 보인다. 행마다 자기 y축을 갖게 하면 둘 다 해결된다.
//
// 각 행: [라벨] [그래프] [현재값]. pts 의 v === null 은 선을 끊는다(보간 금지 —
// 없는 값을 이어 그리면 끊긴 적 없는 것처럼 보인다).
// 한 행이 곧 하나의 그래프다. 46 으로 잡았다가 채널 하나만 켜진 상태에서 캔버스가
// 62px 짜리 띠가 되어 아무것도 안 보였다(2026-08-12). 행 하나가 제 몫의 세로 공간을
// 갖도록 충분히 키운다 — 요소를 "한 줄씩 차지하게" 한다는 게 이 뜻이다.
const OPS_ROW_H = 104;
function opsPlotRows(cv, rows, t0, t1) {
  if (!cv) return;
  const W = cv.width;
  const AXIS_H = 16;
  const H = Math.max(OPS_ROW_H, rows.length * OPS_ROW_H) + AXIS_H;
  if (cv.height !== H) cv.height = H;      // 행 수에 맞춰 캔버스를 늘린다
  const ctx = cv.getContext('2d');
  const L = 44, R = 122, T = 6, B = 6;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = '#111'; ctx.fillRect(0, 0, W, H);
  ctx.font = '11px system-ui, sans-serif';

  if (!rows.length) {
    ctx.fillStyle = '#777'; ctx.textAlign = 'center';
    ctx.fillText('데이터 없음 — 이 채널의 검출이 꺼져 있는지 확인하세요', W / 2, H / 2);
    return;
  }
  const x = (t) => L + (t - t0) / Math.max(1, t1 - t0) * (W - L - R);

  for (let r = 0; r < rows.length; r++) {
    const row = rows[r];
    const top = r * OPS_ROW_H, bot = top + OPS_ROW_H;
    const yMax = row.yMax || 1;
    const yMin = row.yMin || 0;
    const y = (v) => bot - B - ((v - yMin) / Math.max(1e-9, yMax - yMin)) * (OPS_ROW_H - T - B);

    // 눈금선 3개(위·중간·바닥) + 행 구분선
    ctx.strokeStyle = '#262626'; ctx.lineWidth = 1;
    for (let g = 0; g <= 2; g++) {
      const yy = y(yMin + (yMax - yMin) * g / 2);
      ctx.beginPath(); ctx.moveTo(L, yy); ctx.lineTo(W - R, yy); ctx.stroke();
    }
    if (r) {
      ctx.strokeStyle = '#3a3a3a';
      ctx.beginPath(); ctx.moveTo(0, top); ctx.lineTo(W, top); ctx.stroke();
    }

    ctx.fillStyle = row.labelColor || '#bbb'; ctx.textAlign = 'left';
    ctx.font = 'bold 12px system-ui, sans-serif';
    ctx.fillText(row.label, 6, top + OPS_ROW_H / 2 + 4);
    ctx.font = '11px system-ui, sans-serif';
    // y축 위/아래 눈금값
    ctx.fillStyle = '#666'; ctx.textAlign = 'right';
    ctx.fillText(String(Math.round(yMax)), L - 4, y(yMax) + 9);
    ctx.fillText(String(Math.round(yMin)), L - 4, y(yMin) - 2);

    for (let s = 0; s < row.series.length; s++) {
      const sr = row.series[s];
      if (!sr || !sr.pts) continue;
      ctx.strokeStyle = sr.color; ctx.lineWidth = sr.width || 1.6;
      if (sr.dash) ctx.setLineDash(sr.dash); else ctx.setLineDash([]);
      ctx.beginPath();
      let pen = false;
      for (let i = 0; i < sr.pts.length; i++) {
        const p = sr.pts[i];
        if (p.v === null || !isFinite(p.v)) { pen = false; continue; }
        const vv = Math.max(yMin, Math.min(p.v, yMax));
        const px = x(p.t), py = y(vv);
        if (pen) ctx.lineTo(px, py); else { ctx.moveTo(px, py); pen = true; }
      }
      ctx.stroke();
    }
    ctx.setLineDash([]);
    ctx.fillStyle = '#ddd'; ctx.textAlign = 'left';
    ctx.fillText(row.value || '', W - R + 6, top + OPS_ROW_H / 2 + 4);
  }
  ctx.fillStyle = '#666'; ctx.textAlign = 'center';
  ctx.fillText('-' + ((t1 - t0) / 60000).toFixed(0) + '분', L + 16, H - 4);
  ctx.fillText('지금', W - R - 16, H - 4);
}
// 계열의 마지막 유효값 — 행 오른쪽 현재값 표시에 쓴다.
function opsLast(pts) {
  for (let i = pts.length - 1; i >= 0; i--)
    if (pts[i].v !== null && isFinite(pts[i].v)) return pts[i].v;
  return null;
}
// 임의의 표본 배열을 버킷 평균 시리즈로. pick 이 null 을 주면 그 버킷은 빈 칸.
function opsAvgSeries(pts, t0, t1, bucketMs, pick) {
  const n = Math.max(1, Math.ceil((t1 - t0) / bucketMs));
  const sum = new Array(n).fill(0), cnt = new Array(n).fill(0);
  for (let i = 0; i < pts.length; i++) {
    const k = Math.floor((pts[i].t - t0) / bucketMs);
    if (k < 0 || k >= n) continue;
    const v = pick(pts[i]);
    if (v === null || v === undefined || !isFinite(v)) continue;
    sum[k] += v; cnt[k]++;
  }
  const out = new Array(n);
  for (let i = 0; i < n; i++)
    out[i] = {t: t0 + (i + 0.5) * bucketMs, v: cnt[i] ? sum[i] / cnt[i] : null};
  return out;
}
// 좌표처럼 0 에서 시작하면 안 되는 값의 y 범위. 0 기준으로 그리면 2146px 근처의
// 몇 px 흔들림이 바닥에 눌려 직선으로 보인다.
function opsSpan(seriesList) {
  let lo = Infinity, hi = -Infinity;
  for (const s of seriesList)
    for (const p of s) if (p.v !== null && isFinite(p.v)) { if (p.v < lo) lo = p.v; if (p.v > hi) hi = p.v; }
  if (lo === Infinity) return {yMin: 0, yMax: 1};
  if (hi - lo < 4) { const m = (lo + hi) / 2; lo = m - 2; hi = m + 2; }   // 너무 평평하면 최소 폭
  const pad = (hi - lo) * 0.15;
  return {yMin: lo - pad, yMax: hi + pad};
}
function opsFmt(v, unit, dp) {
  if (v === null || v === undefined || !isFinite(v)) return '—';
  return v.toFixed(dp === undefined ? 1 : dp) + (unit || '');
}

function opsDraw() {
  const pane = document.getElementById('opsPane');
  if (!pane) return;
  const t1 = Date.now();
  const t0 = t1 - opsWinMin * 60000;
  // 버킷 폭: 창이 길수록 넓혀 점 수를 일정하게 유지한다(1분→1s, 5분→2s, 30분→10s).
  const bucketMs = opsWinMin <= 1 ? 1000 : opsWinMin <= 5 ? 2000 : 10000;
  const perSec = 1000 / bucketMs;

  const fpsRows = [], latRows = [], hitRows = [];
  let nTotal = 0;
  for (let ch = 0; ch < 4; ch++) {
    nTotal += opsLines[ch].length;
    // 창 안에 줄이 하나도 없는 채널은 행을 만들지 않는다 — 꺼진 렌즈까지 빈 행으로
    // 자리를 차지하면 정작 도는 채널이 눌린다.
    let has = false;
    for (let i = opsLines[ch].length - 1; i >= 0; i--) { if (opsLines[ch][i].t >= t0) { has = true; break; } }
    if (!has) continue;

    const bl = opsBuckets(opsLines[ch],  t0, t1, bucketMs);
    const bf = opsBuckets(opsFrames[ch], t0, t1, bucketMs);
    const color = OPS_CH_COLOR[ch], label = 'CH' + (ch + 1);

    const frameFps = opsSeries(bf, (c) => c.n ? c.n * perSec : null);
    const lineFps  = opsSeries(bl, (c) => c.n ? c.n * perSec : null);
    let fm = 1;
    for (const s of [frameFps, lineFps]) for (const p of s) if (p.v !== null && p.v > fm) fm = p.v;
    fpsRows.push({label: label, labelColor: color, yMax: opsNiceMax(fm),
      series: [{pts: frameFps, color: color, width: 1.8},
               {pts: lineFps,  color: color, width: 1, dash: [3, 3]}],
      value: opsFmt(opsLast(frameFps), ' fps') + '  (수신 ' + opsFmt(opsLast(lineFps), '') + ')'});

    const procAvg = opsSeries(bl, (c) => c.procN ? c.procSum / c.procN : null);
    const procMax = opsSeries(bl, (c) => c.procMax);
    const detAvg  = opsSeries(bl, (c) => c.detN ? c.detSum / c.detN : null);
    let lm = 1;
    for (const p of procMax) if (p.v !== null && p.v > lm) lm = p.v;
    latRows.push({label: label, labelColor: color, yMax: opsNiceMax(lm),
      series: [{pts: procAvg, color: color, width: 1.8},
               {pts: procMax, color: color, width: 1, dash: [2, 3]},
               {pts: detAvg,  color: '#888', width: 1}],
      value: opsFmt(opsLast(procAvg), ' ms', 0) + '  (최대 ' + opsFmt(opsLast(procMax), '', 0) + ')'});

    const hitPct = opsSeries(bf, (c) => c.n ? c.hit / c.n * 100 : null);
    hitRows.push({label: label, labelColor: color, yMax: 100,
      series: [{pts: hitPct, color: color, width: 1.8}],
      value: opsFmt(opsLast(hitPct), ' %', 0)});
  }
  opsPlotRows(document.getElementById('opsFpsCanvas'), fpsRows, t0, t1);
  opsPlotRows(document.getElementById('opsLatCanvas'), latRows, t0, t1);
  opsPlotRows(document.getElementById('opsHitCanvas'), hitRows, t0, t1);

  // CPU 는 앱 전체 값이라 채널 구분이 없다 — 한 행.
  const cpuPts = opsAvgSeries(opsCpu, t0, t1, bucketMs, (s) => s.app);
  const rssPts = opsAvgSeries(opsCpu, t0, t1, bucketMs, (s) => s.rss / 1024);
  let cm = 1;
  for (const p of cpuPts) if (p.v !== null && p.v > cm) cm = p.v;
  opsPlotRows(document.getElementById('opsCpuCanvas'), [
    {label: '앱 CPU', labelColor: '#e0e0e0', yMax: opsNiceMax(cm),
     series: [{pts: cpuPts, color: '#e0e0e0', width: 1.8}],
     value: opsFmt(opsLast(cpuPts), ' %', 1)},
    {label: 'RSS', labelColor: '#8ab4f8', yMax: opsNiceMax(Math.max(1, opsLast(rssPts) || 1) * 1.3),
     series: [{pts: rssPts, color: '#8ab4f8', width: 1.6}],
     value: opsFmt(opsLast(rssPts), ' MB', 0)},
  ], t0, t1);

  opsDrawMarker(t0, t1, bucketMs);

  const stat = document.getElementById('opsStat');
  if (stat) stat.textContent = nTotal ? ('표본 ' + nTotal + '줄') : '아직 데이터 없음 — 검출이 켜져 있는지 확인하세요';
}

// 추적 마커: 꼭짓점별 차트 4개(각각 x·y 두 행) + world 좌표 1개(X·Y·헤딩 세 행).
// 채널은 상단 드롭다운(curCh())을 따른다 — 같은 id 가 다른 렌즈에도 보이면
// 아래 힌트로 알려준다.
function opsDrawMarker(t0, t1, bucketMs) {
  const ch = curCh();
  const marks = (opsMarks[ch] || []).filter((m) => m.t >= t0);
  const CX = '#4da3ff', CY = '#ffa14d';

  for (let i = 0; i < 4; i++) {
    const cv = document.getElementById('opsMark' + i + 'Canvas');
    if (!cv) continue;
    if (!marks.length) { opsPlotRows(cv, [], t0, t1); continue; }
    const xs = opsAvgSeries(marks, t0, t1, bucketMs, (m) => m.cs[i][0]);
    const ys = opsAvgSeries(marks, t0, t1, bucketMs, (m) => m.cs[i][1]);
    const sx = opsSpan([xs]), sy = opsSpan([ys]);
    opsPlotRows(cv, [
      {label: 'c' + i + ' x', labelColor: CX, yMin: sx.yMin, yMax: sx.yMax,
       series: [{pts: xs, color: CX, width: 1.8}], value: opsFmt(opsLast(xs), ' px', 1)},
      {label: 'c' + i + ' y', labelColor: CY, yMin: sy.yMin, yMax: sy.yMax,
       series: [{pts: ys, color: CY, width: 1.8}], value: opsFmt(opsLast(ys), ' px', 1)},
    ], t0, t1);
  }

  const wcv = document.getElementById('opsWorldCanvas');
  if (wcv) {
    const wx = opsAvgSeries(marks, t0, t1, bucketMs, (m) => m.wx);
    const wy = opsAvgSeries(marks, t0, t1, bucketMs, (m) => m.wy);
    const wd = opsAvgSeries(marks, t0, t1, bucketMs, (m) => m.wdeg);
    const hasWorld = opsLast(wx) !== null;
    if (!marks.length || !hasWorld) {
      opsPlotRows(wcv, [], t0, t1);
    } else {
      const sx = opsSpan([wx]), sy = opsSpan([wy]), sd = opsSpan([wd]);
      opsPlotRows(wcv, [
        {label: 'X', labelColor: CX, yMin: sx.yMin, yMax: sx.yMax,
         series: [{pts: wx, color: CX, width: 1.8}], value: opsFmt(opsLast(wx), ' mm', 0)},
        {label: 'Y', labelColor: CY, yMin: sy.yMin, yMax: sy.yMax,
         series: [{pts: wy, color: CY, width: 1.8}], value: opsFmt(opsLast(wy), ' mm', 0)},
        {label: '헤딩', labelColor: '#5ad18a', yMin: sd.yMin, yMax: sd.yMax,
         series: [{pts: wd, color: '#5ad18a', width: 1.8}], value: opsFmt(opsLast(wd), '°', 1)},
      ], t0, t1);
    }
  }

  const hint = document.getElementById('opsMarkHint');
  if (hint) {
    const elsewhere = [];
    for (let c = 0; c < 4; c++)
      if (c !== ch && (opsMarks[c] || []).some((m) => m.t >= t0)) elsewhere.push('CH' + (c + 1));
    if (marks.length) {
      const w = opsLast(opsAvgSeries(marks, t0, t1, bucketMs, (m) => m.wx));
      hint.innerHTML = 'CH' + (ch + 1) + ' 에서 id ' + opsMarkerId + ' 표본 ' + marks.length + '개' +
        (w === null ? ' · <b>world 없음</b> — 이 렌즈에 마커평면(H_marker)이 준비되지 않았습니다' : '') +
        (elsewhere.length ? ' · ' + elsewhere.join(',') + ' 에서도 보이는 중' : '');
    } else {
      hint.innerHTML = 'CH' + (ch + 1) + ' 에 id ' + opsMarkerId + ' 표본이 없습니다' +
        (elsewhere.length ? ' — <b>' + elsewhere.join(',') + '</b> 에서 보이는 중입니다 (상단 채널 드롭다운을 바꿔보세요)'
                          : ' — 검출이 켜져 있는지, id 가 맞는지 확인하세요');
    }
  }
}
// 탭이 보일 때만 그린다. rAF 상시 루프를 쓰지 않는 것은 기존 오버레이와 같은 이유.
setInterval(() => {
  const p = document.getElementById('opsPane');
  if (p && p.style.display !== 'none') opsDraw();
}, 1000);

es.onmessage = (e) => {
  handleRaw(e.data);
  handleLatency(e.data);
  handleOps(e.data);
  handleCalibKProbe(e.data);
  handleHgStatus(e.data);
  handleHgAnchors(e.data);
  handleHgFit(e.data);
  handleHgFitPt(e.data);
  handleHgMatrix(e.data);
  handleHgPersistence(e.data);
  handleHgCoordMode(e.data);
  handleShell(e.data);
  handleDetect(e.data);
  handleDynRoi(e.data);
  handleCpu(e.data);
  handleCentralStatus(e.data);
  handleMarkerPlane(e.data);
  handleIvaSync(e.data);
  handleIvaEvent(e.data);
  handleIvaZoneSet(e.data);
  handleZoneBand(e.data);
  handleZoneBandsState(e.data);
  handleWiseAiDet(e.data);
  handleOdo(e.data);
  handleRegister(e.data);
  // 이 탭 로그는 카메라 위주로 - 로봇/QT 트래픽은 로봇 탭(/robot)이나
  // 로그 모니터(/logs)에서 본다 (내부 상태 파싱은 위에서 이미 끝났으니 표시만 거른다).
  if (logSubject(e.data) === 'robot' || logSubject(e.data) === 'qt') return;
  // 검출 위치 피드는 로그에 안 찍는다 -- 초당 수십 건이라 다른 메시지를
  // 밀어내 버린다. 오버레이가 이미 소비했으므로 여기서 버려도 잃는 게 없다.
  if (e.data.indexOf('[det] ') === 0) return;
  if (hideLost && e.data.includes('MARKER LOST')) return;
  const stick = isNearBottom();
  log.textContent += e.data + "\\n";
  if (stick) log.scrollTop = log.scrollHeight;
  const gm = e.data.match(/gates=([01])/);
  if (gm) document.getElementById('gateChk').checked = gm[1] === '1';
  let qm;
  // "[calib-K] ch=N CURRENT VALUES: fx=.. ..." -- ch= 붙는다(2026-08-10, 다른
  // 채널별 핸들러들과 동일한 자리). ch 없는 줄(옛 흔적)은 무시 -- 어느 렌즈인지
  // 몰라 전역에 반영하면 다시 채널이 섞인다.
  if ((qm = e.data.match(/\\[calib-K\\] ch=(\\d+) CURRENT VALUES: fx=([-\\d.eE+]+) fy=([-\\d.eE+]+) cx=([-\\d.eE+]+) cy=([-\\d.eE+]+) dist=\\[([^\\]]*)\\]/))) {
    const ch = Number(qm[1]);
    const dist = qm[6].split(',').map(s => s.trim()).filter(s => s.length);
    chKCalib[ch] = {
      fx: parseFloat(qm[2]), fy: parseFloat(qm[3]),
      cx: parseFloat(qm[4]), cy: parseFloat(qm[5]),
      dist: dist.map(parseFloat),
    };
    if (ch === curCh()) {
      renderKQuery(true, qm[2], qm[3], qm[4], qm[5], dist);
      kCalib = chKCalib[ch];   // 보정 좌표 표시용 캐시 (마커 검출 탭)
      refreshUndistState();
      if (rawOn) renderRaw();
      if (rawOverlayOn) redrawRawCanvas();
    }
  } else if ((qm = e.data.match(/\\[calib-K\\] ch=(\\d+) no calibration loaded on the camera/))) {
    const ch = Number(qm[1]);
    chKCalib[ch] = null;
    if (ch === curCh()) {
      renderKQuery(false);
      kCalib = null;
      refreshUndistState();
      if (rawOn) renderRaw();
      if (rawOverlayOn) redrawRawCanvas();
    }
  }
  if (e.data.includes('[calib-K]')) {
    updateKStatus(e.data);
  }
};
es.onerror = () => { log.textContent += "[SSE connection lost, retrying...]\\n"; };
</script>
</body>
</html>""".replace("__LOG_SUBJECT_JS__", LOG_SUBJECT_JS)


# HTTP POST /hg_experiment/{start,stop,result} 처리 (web_gui의 Handler가 호출).
# 예전엔 Handler의 메서드였는데, CCTV 상태를 다루므로 이리로 옮겼다.
def hg_experiment_post(path, data):
    global hg_experiment_result
    if path == "/hg_experiment/start":
        with hg_experiment_lock:
            hg_experiment.update({"active": True,
                                  "candidates": {}, "samples": {},
                                  "started": time.strftime("%Y%m%d_%H%M%S"),
                                  "w": None, "h": None, "last_export": None})
        broadcast("[hg-experiment] marker recording started")
        return {"ok": True}
    if path == "/hg_experiment/stop":
        exported = hg_experiment_export()
        broadcast(f"[hg-experiment] collection exported: {exported['json']}")
        return {"ok": True, **exported}

    # PC result contract: H is mandatory. The recorder intentionally does
    # not prescribe point count, IDs, or a validation split.
    h = data.get("H")
    if not isinstance(h, list) or len(h) != 9:
        raise ValueError("result must contain H with 9 values")
    h = [float(x) for x in h]
    if not all(math.isfinite(x) for x in h):
        raise ValueError("H contains a non-finite value")
    source_ids = [int(x) for x in data.get("source_ids", [])]
    if len(source_ids) != len(set(source_ids)):
        raise ValueError("source_ids must be unique when supplied")
    hg_experiment_result = {"source_ids": source_ids, "H": h, "rmse_mm": data.get("rmse_mm"),
                            "max_error_mm": data.get("max_error_mm")}
    cmd = "HG_SET " + " ".join(f"{x:.12g}" for x in h)
    send_command(cmd)
    broadcast("[hg-experiment] PC result accepted; HG_SET sent to camera")
    return {"ok": True, **hg_experiment_result}
