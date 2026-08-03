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

current_conn = None
conn_lock = threading.Lock()

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


# 카메라가 마지막으로 보고한 동적 ROI 설정. 카메라가 권위이고 여기는 표시용 사본이다.
dynroi_state = {"enabled": False, "margin": 240, "max_miss": 4, "track_ids": []}

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
_calib_channel = 1          # 지금 캘리브레이션 중인 채널
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
    """캘리브레이션 대상 채널을 바꾸고 카메라에도 알린다.

    서버가 CMD SELECT_CHANNEL 을 CCTV 로 중계하므로, 카메라 앱이 그 채널 영상으로
    검출 대상을 바꾼다. 이걸 안 보내면 화면상 채널만 바뀌고 실제로는 계속 같은
    채널을 캘리하게 된다 — 결과가 엉뚱한 슬롯에 저장된다.
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


def calib_cache_k(fx, fy, cx, cy, dist):
    if fx is None or fy is None:
        return
    with _calib_lock:
        ch = _calib_channel
        c = _cache_for(ch)
        c["K"] = [[float(fx), 0.0, float(cx or 0)],
                  [0.0, float(fy), float(cy or 0)],
                  [0.0, 0.0, 1.0]]
        c["dist"] = [float(x) for x in (dist or [])]
    push_calib_to_server(ch)


def calib_cache_h(h):
    if not isinstance(h, list) or len(h) != 9:
        return
    with _calib_lock:
        ch = _calib_channel
        _cache_for(ch)["H"] = _reshape3x3(h)
    push_calib_to_server(ch)


def push_calib_to_server(ch=None):
    """Send the cached calibration for one channel (needs at least H_floor)."""
    with _calib_lock:
        if ch is None:
            ch = _calib_channel
        c = _calib_caches.get(ch, {})
        H, K, dist = c.get("H"), c.get("K"), c.get("dist")
    if H is None:
        return  # 서버는 최소 H_floor가 있어야 유효 (calib.hpp)
    bundle = {"version": 1, "H_floor": H}
    # 카메라는 바닥 H만 계산 → H_marker 생략(서버가 floor로 대체, 시차 보정 없음)
    if K is not None:
        bundle["K"] = K
        bundle["D"] = dist or [0, 0, 0, 0, 0]
    # ADMIN 연결로 보낸다 (CCTV 연결이 아님) - 서버 fromAdmin()이 H_MATRIX를
    # CCTV와 동일하게 처리(저장+Qt 중계)하므로 동작은 같다. CCTV 연결(cctv_link_loop)은
    # 카메라 직결 시 꺼지는(CCTV_BRIDGE_ENABLED) 대상이라, 그 연결에 캘리 결과를
    # 얹으면 브리지를 끄는 순간 이 관리자 창의 캘리 도구도 같이 죽는다 - ADMIN
    # 연결은 브리지와 무관하게 항상 떠 있으므로 여기 실어야 안전하다.
    #
    # ch 는 payload 최상위에 붙는다 (calib "안"이 아니다 — server_PROTOCOL.md).
    # 안 실으면 서버가 전부 채널 1로 보고, 4채널을 캘리해도 마지막 하나만 남는다.
    if server_send("H_MATRIX", {"ch": ch, "calib": bundle}):
        broadcast(f"[bridge] CH{ch} 캘리 결과를 서버로 전송"
                  f"(H_MATRIX ch={ch}, role=ADMIN) — 저장+Qt 중계됨")


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

    if mtype == "CALIB_ACK":
        broadcast("[calib] camera acknowledged, collecting anchors...")
        return last_seq
    if mtype == "CALIB_RESULT":
        if msg.get("ok"):
            broadcast(f"[calib] SUCCESS (frames={msg.get('frames')}) — camera now streams world coords "
                      f"H={msg.get('H')}")
            calib_cache_h(msg.get("H"))  # 서버로 H_MATRIX 전송
        else:
            broadcast(f"[calib] FAILED: {msg.get('reason')}")
        return last_seq
    if mtype == "CALIB_HG_QUERY":
        if msg.get("available"):
            broadcast(f"[calib] HOMOGRAPHY H={msg.get('H')}")
        else:
            broadcast("[calib] 호모그래피 아직 계산 안 됨")
        return last_seq
    if mtype == "CALIB_ANCHORS":
        # Keep the camera's table authoritative: this is sent for both
        # ANCHOR_QUERY and each ANCHOR_SET acknowledgement.
        broadcast("[calib] ANCHORS " + json.dumps(msg.get("anchors", []),
                                                    separators=(",", ":")))
        return last_seq
    if mtype == "CALIB_VALIDATION":
        broadcast("[calib] VALIDATION " + json.dumps(msg.get("markers", []),
                                                       separators=(",", ":")))
        return last_seq
    if mtype == "HG_SAVE":
        if msg.get("ok"):
            broadcast("[calib] 저장 완료 — 카메라 /mnt(PERSIST_DIR)에 H 기록됨, 재부팅해도 유지")
        else:
            broadcast(f"[calib] H 저장 실패: {msg.get('reason')}")
        return last_seq
    if mtype == "HG_SET":
        if msg.get("ok"):
            broadcast(f"[calib] PC 분석 H 적용 완료: {msg.get('H')}")
            calib_cache_h(msg.get("H"))  # 서버로 H_MATRIX 전송
        else:
            broadcast(f"[calib] PC 분석 H 적용 실패: {msg.get('reason')}")
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
    if mtype == "CALIB_K_PROGRESS":
        if msg.get("rejected"):
            broadcast(f"[calib-K] capture REJECTED — {msg.get('reason')} "
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
            broadcast(f"[calib-K] captured view {msg.get('views')}/{msg.get('target')} "
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
            calib_cache_k(msg.get("fx"), msg.get("fy"), msg.get("cx"),
                          msg.get("cy"), msg.get("dist"))  # 서버로 K/D 반영
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
        w, h = msg.get("w") or 0, msg.get("h") or 0
        if w and h:
            pct = w * h / (1920 * 1080) * 100
            broadcast(f"[roi] 검출 영역 ({msg.get('x')},{msg.get('y')}) {w}x{h} "
                      f"— 전체의 {pct:.0f}% (마커가 이 영역을 벗어나면 검출 안 됨)")
        else:
            broadcast("[roi] 전체 화면으로 복원")
        return last_seq
    if mtype == "ARUCO_SCAN":
        n = msg.get("passes")
        wins = {1: f"{msg.get('win')}", 2: "7, 17", 3: "3, 13, 23"}.get(n, "?")
        broadcast(f"[aruco] 이진화 스캔 {n}회 (창 {wins}) — "
                  f"det/검출률/좌표지터를 비교해 볼 것")
        return last_seq
    if mtype == "RAW_FPS_TEST":
        if msg.get("enabled"):
            broadcast("[raw-fps] 측정 모드 ON — 검출을 건너뛰고 프레임 도착만 셉니다. "
                      "이제 seq 증가 속도가 곧 SDK 전달 fps입니다. "
                      "(마커 검출 안 됨 — 측정 후 반드시 끌 것)")
        else:
            broadcast("[raw-fps] 측정 모드 OFF — 정상 검출로 복귀")
        return last_seq
    if mtype == "DETECT_ENABLE":
        on = 1 if msg.get("enabled") else 0
        refused = msg.get("refused") or ""
        if refused:
            broadcast(f"[detect] detect_enabled={on} — 끄기 거부됨 ({refused}). "
                      "수집이 끝난 뒤 다시 시도하세요")
        elif on:
            broadcast("[detect] detect_enabled=1 — 마커 검출 ON")
        else:
            broadcast("[detect] detect_enabled=0 — 마커 검출 OFF "
                      "(하트비트만 전송, 카메라 부하 감소). "
                      "캘리브레이션 전에 반드시 다시 켤 것")
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
    if mtype == "DYNROI":
        ids = msg.get("track_ids") or []
        dynroi_state.update({
            "enabled": bool(msg.get("enabled")),
            "margin": msg.get("margin", dynroi_state["margin"]),
            "max_miss": msg.get("max_miss", dynroi_state["max_miss"]),
            "track_ids": list(ids)})
        # The id list is broadcast as its own line so the browser can parse it
        # without picking it out of prose. Empty list = every marker.
        broadcast("[dynroi] IDS " + json.dumps(list(ids), separators=(",", ":")))
        scope = f"id {','.join(str(i) for i in ids)}만" if ids else "모든 마커"
        if msg.get("enabled"):
            broadcast(f"[dynroi] 동적 ROI ON — 추적 대상 {scope}, "
                      f"max_margin={msg.get('margin')}px "
                      f"max_miss={msg.get('max_miss')} "
                      f"(마커 크기+이동량으로 margin 자동 조절)")
        else:
            broadcast("[dynroi] 동적 ROI OFF — 수동 ROI/전체 화면으로 복귀")
        return last_seq
    if mtype == "DYNROI_STATE":
        if msg.get("tracking"):
            detail = (f" margin={msg.get('margin_used')}px"
                      f" marker={msg.get('marker_px')}px"
                      f" move={msg.get('motion_px')}px"
                      if msg.get("margin_used") is not None else "")
            broadcast(f"[dynroi] TRACK — 검출 영역 ({msg.get('x')},{msg.get('y')}) "
                      f"{msg.get('w')}x{msg.get('h')}{detail}")
        else:
            broadcast("[dynroi] SEARCH — 마커 놓침, 전체 재탐색 중")
        return last_seq
    if mtype == "CPU_STAT":
        # 카메라가 2초마다 자발적으로 보낸다(요청 없음). -1 은 /proc 을 못 읽은 경우.
        app = msg.get("app_pct")
        sys_ = msg.get("sys_pct")
        cores = msg.get("cores")
        broadcast(f"[cpu] app={app}% sys={sys_}% cores={cores}")
        return last_seq
    if mtype == "CALIB_K_QUERY":
        if msg.get("available"):
            broadcast(f"[calib-K] CURRENT VALUES: fx={msg.get('fx')} fy={msg.get('fy')} "
                      f"cx={msg.get('cx')} cy={msg.get('cy')} dist={msg.get('dist')} "
                      f"session={msg.get('session')} profile={msg.get('profile', '기본')} "
                      f"(calib_views/에서 이 session 값으로 원본 이미지셋 매칭)")
        else:
            broadcast("[calib-K] no calibration loaded on the camera right now")
        return last_seq
    if mtype == "CALIB_K_PROFILES":
        broadcast("[calib-K-profile] LIST " + json.dumps(msg, separators=(",", ":")))
        return last_seq
    if mtype == "CALIB_K_PROFILE":
        if msg.get("ok"):
            broadcast(f"[calib-K-profile] {msg.get('action')} 완료: {msg.get('name')} (현재={msg.get('active')})")
        else:
            broadcast(f"[calib-K-profile] {msg.get('action')} 실패: {msg.get('name')} — 영문/숫자/_/- 23자 이내인지와 /mnt 파일을 확인")
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

    if msg.get("confidence", 0) > 0:
        corners = msg.get("corners", [])
        cctv_forward_pos(corners)  # 서버(9000, role=CCTV)로 POS 통역 전달
        ctxt = " ".join(f"c{i}=({c['x']:.2f},{c['y']:.2f})" for i, c in enumerate(corners))
        world = msg.get("world")
        wtxt = (f" world=({world['x']:.0f},{world['y']:.0f}mm,{world['theta']:.1f}deg)") if world else ""
        broadcast(f"seq={seq} id={msg.get('id')} {ctxt}{wtxt} "
                  f"{latency_text(msg, now_ms)}{gap}")
    else:
        broadcast(f"seq={seq} MARKER LOST (heartbeat) "
                  f"{latency_text(msg, now_ms)}{gap}")
    return seq


def handle_client(conn, addr):
    global current_conn, _clock_warned, _clock_offset_note
    broadcast(f"[+] camera connected: {addr}")
    _reset_clock_offset()
    _clock_offset_note = False
    # Re-arm the skew warning: a reconnect often means the app (or camera)
    # restarted, so the clock situation may have changed and is worth saying
    # once more.
    _clock_warned = False
    with conn_lock:
        current_conn = conn
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
  #groups, #homographyPane, #shellPane, #rawPane, #centralPane {
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
  #content.log-collapsed > #centralPane { flex:1 1 auto; }

  /* 앵커·검증점 배치도(그리드): 작업영역 평면 위에 점을 드래그로 배치한다.
     파랑=계산 앵커, 주황=검증 기준점 — 오버레이 범례와 색을 맞춘다.
     touch-action:none 은 터치에서 드래그가 스크롤로 먹히지 않게 하려는 것. */
  .hg-map { width:100%; max-width:520px; height:auto; display:block;
            border:1px solid var(--line); border-radius:10px;
            background:var(--field); touch-action:none; margin-top:4px; }

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
    #groups, #homographyPane, #shellPane, #rawPane, #centralPane {
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
    <button type="button" id="logBtn" class="helpbtn on" onclick="toggleLogPanel()"
            title="오른쪽 로그(터미널) 패널 접기/펴기">터미널</button>
    <button type="button" id="detBtn" class="helpbtn on" onclick="toggleDetect()"
            title="카메라의 마커 검출을 켜고 끕니다">검출</button>
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
      <button onclick="send('CALIB_K_STATUS')">현재 설정 조회<span class="cmd">CALIB_K_STATUS</span></button>
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

    <div class="row"><label class="toggle"><input type="checkbox" id="gateChk" checked
        onchange="send('CALIB_K_GATE ' + (this.checked ? 1 : 0))"> 품질 게이트 (끄면 품질검사·분산·RMS 통과조건 없이 캡처·계산)</label></div>

    <div class="cmdflow">
    <div class="row"><button onclick="startCalibration()">세션 시작<span class="cmd">CALIB_K_START</span></button>
      <span class="desc">세션 시작(초기화). 먼저 누르세요.</span></div>
    <div class="row"><button id="captureBtn" onclick="captureView()">이 자세 캡처<span class="cmd">CALIB_K_CAPTURE</span></button>
      <span class="desc">품질검사를 통과한 뷰만 증가합니다. 보드를 멈춘 뒤 누르세요.</span></div>
    <div class="row"><button onclick="send('CALIB_K_UNDO')">마지막 뷰 취소<span class="cmd">CALIB_K_UNDO</span></button>
      <span class="desc">방금 캡처한 자세가 잘못됐다고 판단한 경우 제거.</span></div>
    <div class="row go"><button id="computeBtn" onclick="computeCalibration()" disabled>계산하기<span class="cmd">CALIB_K_COMPUTE</span></button>
      <span class="desc">목표 뷰를 모두 통과한 뒤 직접 실행합니다. 자동 계산하지 않습니다.</span></div>
    <div class="row"><button onclick="send('CALIB_K_UPLOAD')">보관 이미지 전송<span class="cmd">CALIB_K_UPLOAD</span></button>
      <span class="desc">이번 세션 뷰마다 원본 JPEG · 오버레이 JPEG · 코너좌표+K/dist JSON 3종을 파이 calib_views/로 전송·저장(같은 파일명으로 묶임). 캡처 순간이 아니라 지금 업로드하므로 프레임 경로가 끊기지 않음.</span></div>
    <div class="row"><button onclick="send('CALIB_K_QUERY')">현재 K값 조회<span class="cmd">CALIB_K_QUERY</span></button>
      <span class="desc">새로 캘리브 안 하고, 지금 카메라에 로드된 K/dist 값을 그대로 조회.</span></div>
    <div class="row go"><button onclick="send('CALIB_K_SAVE')">/mnt에 저장<span class="cmd">CALIB_K_SAVE</span></button>
      <span class="desc">지금 로드된 K/dist를 카메라 /mnt(PERSIST_DIR)에 즉시 기록 — 재부팅해도 유지됩니다. CALIB_K_COMPUTE 성공 시 자동으로도 저장되지만, 쓰기 실패를 확인한 뒤 재시도할 때 이 버튼으로 다시 시도.</span></div>
    </div>
    <div id="kquery" class="qbox"><span class="none">현재 K값 조회를 누르면 카메라에 로드된 값이 표시됩니다</span></div>
    <details class="fold panel" id="foldKProfiles">
      <summary>K/dist 프로필 <span id="kProfileBrief" class="brief">현재 프로필 조회 중</span></summary>
      <div class="foldbody">
        <div class="row"><label class="kparam">새 프로필 이름<input id="kProfileName" maxlength="23" placeholder="예: entrance_4k"></label>
          <button onclick="saveKProfile()">현재 K를 프로필로 저장<span class="cmd">CALIB_K_PROFILE_SAVE</span></button>
          <button onclick="refreshKProfiles()">목록 새로고침<span class="cmd">CALIB_K_PROFILE_LIST</span></button></div>
        <div class="hint">프로필은 카메라 <code>/mnt</code>에 이름별로 저장됩니다. 영문·숫자·<code>_</code>·<code>-</code>만 가능하며, 적용하면 카메라 앱의 현재 K/dist가 즉시 바뀝니다. 해상도·줌·초점이 다른 경우에는 그에 맞는 프로필을 사용한 뒤 H를 다시 계산하세요.</div>
        <div id="kProfileList" class="qbox"><span class="none">목록을 불러오는 중…</span></div>
      </div>
    </details>
  </div>

  <div class="group mid">
    <h2>잔여 왜곡 진단 (LDC)</h2>
    <p class="sub">렌즈 왜곡이 얼마나 남았는지 직선성으로 측정합니다.</p>
    <div class="cmdflow">
    <div class="row"><button onclick="send('LDC_CHECK_START')">진단 시작<span class="cmd">LDC_CHECK_START</span></button>
      <span class="desc">ChArUco 보드 비추면 STOP까지 계속 측정(전/후 비교 포함), 1초마다 CSV 기록. 이 동안 좌표전송 중단.</span></div>
    <div class="row stop"><button onclick="send('LDC_CHECK_STOP')">진단 종료<span class="cmd">LDC_CHECK_STOP</span></button>
      <span class="desc">진단 종료 → 일반 좌표 전송 복귀.</span></div>
    <div class="row snapshot"><button onclick="send('LDC_SNAPSHOT')">스냅샷 1장<span class="cmd">LDC_SNAPSHOT</span></button>
      <span class="desc">지금 순간의 지표 + 이미지 1장을 파이에 저장(.ppm + csv). 화면에는 표시하지 않으므로 파일을 직접 확인.</span></div>
    </div>
  </div>

</div>
<div id="rawPane" style="display:none">
  <div class="group wide">
    <h2>마커 검출 — 픽셀 좌표 (raw / 보정)</h2>
    <p class="sub">인식된 마커의 네 꼭짓점을 실시간으로 확인합니다 (표시 전용).</p>
  </div>

  <div class="group wide">
    <h2>동적 ROI (마커 추적)</h2>
    <p class="sub">마커를 찾으면 그 주변만 검출해 <code>proc</code>을 줄입니다. 놓치면 넓혔다가 전체 재탐색.</p>
    <div class="row">
      <label class="toggle"><input type="checkbox" id="dynRoiChk" onchange="applyDynRoi()">
        동적 ROI 사용<span class="cmd">DYNROI</span></label>
      <label>최대 margin(px) <input type="number" id="dynMargin" value="240" min="0" max="960" step="10" style="width:6em" onchange="applyDynRoi()"></label>
      <label>실패 허용 <input type="number" id="dynMaxMiss" value="4" min="0" max="60" step="1" style="width:5em" onchange="applyDynRoi()"></label>
      <span id="dynRoiState" class="desc">상태: —</span>
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
      <span class="desc">배경 없이 좌표만 그립니다. 캔버스 크기는 K의 (cx×2, cy×2), K가 없으면 1920×1080으로 잡습니다.</span></div>
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
    <button type="button" class="hg-subtab active" id="hgSubCompute" onclick="showHgSection('compute')">계산</button>
    <button type="button" class="hg-subtab" id="hgSubAdvanced" onclick="showHgSection('advanced')">고급</button>
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
      <div class="qbox"><div class="qtitle" style="color:#f59e0b">● 검증 기준점 편집 (계산에 넣지 않음, ID·X·Y mm)</div>
        <div id="hgValidationRows"></div>
        <div class="row"><button type="button" onclick="addHgValidationRow()">기준점 추가</button>
          <button type="button" onclick="queryHgValidation()">현재 값 조회<span class="cmd">VALIDATION_QUERY</span></button>
          <button type="button" onclick="applyHgValidation()">검증점 적용<span class="cmd">VALIDATION_SET</span></button></div>
        <div class="hint">0~16개. ID는 서로 다르고 <b>계산 앵커 ID와 겹칠 수 없습니다</b>. 배치도에서 주황 점을 끌거나 표에 직접 입력하세요. 카메라 재시작 뒤 기본값으로 돌아갑니다.</div>
        <div id="hgValidationEditStatus" class="hint"></div>
      </div>
      <div class="cmdflow">
        <div class="row go"><button id="hgCalibStartBtn" onclick="send('CALIB_START')">앵커로 H 계산<span class="cmd">CALIB_START</span></button></div>
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
        <button type="button" onclick="send('MARKER_PLANE_QUERY')">조회<span class="cmd">MARKER_PLANE_QUERY</span></button>
        <button type="button" onclick="send('MARKER_PLANE_SAVE')">저장<span class="cmd">MARKER_PLANE_SAVE</span></button>
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
      <div class="row go"><button onclick="send('HG_SAVE')">현재 H 저장<span class="cmd">HG_SAVE</span></button>
        <span class="desc">카메라 /mnt(PERSIST_DIR)에 기록합니다. 저장해야 재부팅 뒤에도 유지됩니다.</span></div>
    </div>
  </details>

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
      <div class="row" style="margin-top:18px"><button onclick="send('HG_QUERY')">H 행렬 조회<span class="cmd">HG_QUERY</span></button></div>
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
        <code>H_marker</code>·<code>marker_height_m</code>을 싣는다. 담기는 행렬은 사실상 같고
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
        <code>seq</code>는 카메라가 POS마다 1씩 올리는 값이며 CAM_POSE의 seq와는 별개다.
      </div>
    </details>
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

let kProfiles = [], kActiveProfile = '기본';
function profileNameValid(name) { return /^[A-Za-z0-9_-]{1,23}$/.test(name); }
function refreshKProfiles() { send('CALIB_K_PROFILE_LIST'); }
function saveKProfile() {
  const name = document.getElementById('kProfileName').value.trim();
  if (!profileNameValid(name)) { alert('프로필 이름은 영문·숫자·_·-만, 1~23자로 입력하세요.'); return; }
  send('CALIB_K_PROFILE_SAVE ' + name);
}
function loadKProfile(name) {
  if (confirm('프로필 “' + name + '”의 K/dist를 카메라에 적용할까요?\\n적용 후 호모그래피 H는 다시 계산해야 합니다.'))
    send('CALIB_K_PROFILE_LOAD ' + name);
}
function renderKProfiles() {
  const brief = document.getElementById('kProfileBrief');
  const box = document.getElementById('kProfileList');
  if (!brief || !box) return;
  brief.textContent = '현재: ' + kActiveProfile;
  if (!kProfiles.length) { box.innerHTML = '<span class="none">저장된 이름 프로필 없음 — 현재 K/dist를 이름을 정해 저장하세요.</span>'; return; }
  box.innerHTML = '<div class="qtitle">저장된 프로필</div>' + kProfiles.map(n =>
    '<div class="row"><code>' + n + '</code>' + (n === kActiveProfile ? ' <b>현재 적용 중</b>' : '') +
    '<button onclick="loadKProfile(\\'' + n + '\\')">적용</button></div>').join('');
}
function handleKProfiles(line) {
  const m = line.match(/^\\[calib-K-profile\\] LIST (.+)$/);
  if (!m) return;
  try { const data = JSON.parse(m[1]); kProfiles = data.profiles || []; kActiveProfile = data.active || '기본'; renderKProfiles(); } catch (_) {}
}

function captureView() {
  if (capturePending) return;
  capturePending = true;
  captureBtn.disabled = true;
  send('CALIB_K_CAPTURE');
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
  send('CALIB_K_START');
}

function computeCalibration() {
  if (acceptedViews < targetViews) return;
  computeBtn.disabled = true;
  captureBtn.disabled = true;
  kstatus.className = 'done';
  kinfo.textContent = '계산 시작 요청 — 완료 응답을 기다리는 중';
  send('CALIB_K_COMPUTE');
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

function updateKStatus(line) {
  let m;
  if ((m = line.match(/session started target=(\\d+)/))) {
    acceptedViews = 0; targetViews = Number(m[1]);
    kstatus.className = ''; kcount.textContent = `0 / ${targetViews}`;
    kinfo.textContent = '세션 시작 — 보드를 기울여가며 자세를 바꿔 캡처하세요';
    captureBtn.disabled = false;
    computeBtn.disabled = true;
    updateKBar();
  } else if ((m = line.match(/captured view (\\d+)\\/(\\d+) \\((\\d+)\\/(\\d+) corners, coverage=([\\d.]+)%, sharpness=([\\d.]+), move=([\\d.-]+)px\\)/))) {
    capturePending = false; captureBtn.disabled = false;
    acceptedViews = Number(m[1]); targetViews = Number(m[2]);
    kstatus.className = ''; kcount.textContent = m[1] + ' / ' + m[2];
    kinfo.textContent = `통과 · 코너 ${m[3]}/${m[4]} · 화면점유 ${m[5]}% · 선명도 ${m[6]} · 이동 ${m[7]}px`;
    computeBtn.disabled = acceptedViews < targetViews;
    updateKBar();
  } else if ((m = line.match(/capture REJECTED — (.+) corners=(\\d+)\\/(\\d+) coverage=([\\d.]+)% sharpness=([\\d.]+) move=([\\d.-]+)px/))) {
    capturePending = false; captureBtn.disabled = false;
    kstatus.className = 'reject';
    kinfo.textContent = `거부 · ${m[1]} · 코너 ${m[2]}/${m[3]} · 화면점유 ${m[4]}% · 선명도 ${m[5]}`;
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
let rawOn = false, rawSeq = null, rawFrame = {};
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
  if (showUndist && !kCalib) send('CALIB_K_QUERY');
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
  const ids = Object.keys(rawFrame);
  // 검출이 꺼져 있으면 '마커 없음'이 아니라 꺼졌다고 말한다. 둘 다 빈 화면이라
  // 구분이 안 되면 기능이 고장난 것처럼 보인다.
  if (!detectOn) {
    rawBox.innerHTML = '<span class="none">마커 검출이 꺼져 있습니다 — ' +
      '상단 [검출 OFF] 버튼을 눌러 켜세요 (카메라가 하트비트만 보내는 중)</span>';
    return;
  }
  if (!ids.length) { rawBox.innerHTML = '<span class="none">이 프레임에 인식된 마커 없음</span>'; return; }
  const cmp = showUndist && kCalib;   // 보정 열을 함께 그릴지
  let html = '';
  for (const id of ids) {
    const c = rawFrame[id];
    html += `<div class="mid">id ${id} · 평균 변 ${markerSidePx(c).toFixed(1)}px</div><table>`;
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
let procs = [], lastNet = null;
// fps 는 카메라가 알려주지 않는다 -- 브라우저에 줄이 도착한 간격으로 잰다.
// 그래서 정확히는 "수신 fps"이고, 카메라 송신 주기에 네트워크 지터가 얹힌 값이다.
let arrivals = [], dets = [], hits = [], lastSeqSeen = null, seqGaps = 0;
// 카메라 CPU. CAM_POSE 와 별개로 2초마다 오므로 마지막 값을 들고 있다가 지연 표가
// 다시 그려질 때 같이 렌더한다 -- 값이 없다고 표에서 줄이 사라지면 더 헷갈린다.
let cpuApp = null, cpuSys = null, cpuCores = null, cpuAt = 0;
function handleCpu(line) {
  const m = line.match(/\\[cpu\\] app=(-?[\\d.]+)% sys=(-?[\\d.]+)% cores=(\\d+)/);
  if (!m) return;
  const a = Number(m[1]), b = Number(m[2]);
  cpuApp = a >= 0 ? a : null;      // 카메라가 -1 로 "못 읽음"을 알린다
  cpuSys = b >= 0 ? b : null;
  cpuCores = Number(m[3]);
  cpuAt = Date.now();
  renderRawLatency();
}
function cpuText() {
  if (cpuApp === null && cpuSys === null)
    return '<span class="none">보고 없음 (카메라 재빌드 필요)</span>';
  const stale = (Date.now() - cpuAt) > 8000 ? ' <span class="none">(오래됨)</span>' : '';
  const app = cpuApp === null ? '—' : cpuApp.toFixed(1) + '%';
  const sys = cpuSys === null ? '—' : cpuSys.toFixed(1) + '%';
  return '앱 ' + app + ' · 전체 ' + sys + ' · ' + (cpuCores || 1) + '코어' + stale;
}
const rawLatency = document.getElementById('rawLatency');

function handleLatency(line) {
  const pm = line.match(/proc=(-?\\d+)ms/);
  if (!pm) return;
  // 렌더는 scheduleRawRender() 의 rAF 에서 한 번만 한다 -- 줄마다 표를 다시 만들면
  // 마커 개수만큼 innerHTML 이 교체되어 표가 깜빡인다.
  handleLatencyCollect(line, Number(pm[1]));
  scheduleRawRender();
}
function handleLatencyCollect(line, procMs) {
  procs.push(procMs);
  if (procs.length > PROC_WINDOW) procs.shift();

  const nm = line.match(/net=(-?\\d+)ms/);
  lastNet = nm ? Number(nm[1]) : null;      // absent/?clock -> unknown

  arrivals.push(Date.now());
  if (arrivals.length > PROC_WINDOW) arrivals.shift();

  const dm = line.match(/det=(-?\\d+)ms/);
  dets.push(dm ? Number(dm[1]) : null);
  if (dets.length > PROC_WINDOW) dets.shift();

  // 같은 프레임에 마커가 여러 개면 seq 가 같은 줄이 여러 번 온다. 검출률은
  // "마커를 본 프레임 비율"이어야 하므로 seq 단위로 한 번만 센다.
  const sm = line.match(/seq=(\\d+)/);
  const seq = sm ? Number(sm[1]) : null;
  const fresh = (seq === null || seq !== lastSeqSeen);
  if (fresh) {
    if (seq !== null && lastSeqSeen !== null && seq > lastSeqSeen + 1)
      seqGaps += seq - lastSeqSeen - 1;
    hits.push(line.includes('MARKER LOST') ? 0 : 1);
    if (hits.length > PROC_WINDOW) hits.shift();
  }
  if (seq !== null) lastSeqSeen = seq;
}

// 지연 표와 캔버스 HUD 가 함께 쓰는 계산. null = 아직 값이 없음.
//
// fps 는 창 전체의 경과시간으로 나눈다. 프레임별 간격의 평균을 쓰면 한 번의 긴
// 정지가 평균을 통째로 끌어내려 실제 처리량보다 낮게 보인다.
function metricFps() {
  if (arrivals.length < 2) return null;
  const span = (arrivals[arrivals.length - 1] - arrivals[0]) / 1000;
  return span > 0 ? (arrivals.length - 1) / span : null;
}
function metricDetCur() {
  const dv = dets.filter(v => v !== null);
  return dv.length ? dv[dv.length - 1] : null;
}
function metricDetAvg() {
  const dv = dets.filter(v => v !== null);
  return dv.length ? dv.reduce((a, b) => a + b, 0) / dv.length : null;
}
function metricHitPct() {
  return hits.length ? hits.reduce((a, b) => a + b, 0) / hits.length * 100 : null;
}

function renderRawLatency() {
  if (!procs.length) return;
  const n = procs.length;
  const cur = procs[n - 1];
  const avg = procs.reduce((a, b) => a + b, 0) / n;
  const min = Math.min.apply(null, procs);
  const max = Math.max.apply(null, procs);
  const netTxt = lastNet === null
    ? '<span class="none">시계 미동기 — 표시 불가</span>'
    : lastNet + ' ms';
  const totTxt = lastNet === null
    ? '<span class="none">—</span>'
    : (cur + lastNet) + ' ms';

  const fpsTxt = metricFps() === null
    ? '<span class="none">측정 중…</span>' : metricFps().toFixed(1) + ' fps';
  const detTxt = metricDetCur() === null
    ? '<span class="none">—</span>'
    : metricDetCur() + ' ms (평균 ' + metricDetAvg().toFixed(1) + ')';
  const hitTxt = hits.length
    ? Math.round(metricHitPct()) + '% (' + hits.reduce((a, b) => a + b, 0) + '/' + hits.length + ' 프레임)'
    : '<span class="none">—</span>';
  rawLatency.innerHTML =
    '<div class="qtitle">proc — 카메라 내부 처리 (최근 ' + n + '프레임)</div>' +
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
    '<tr><td>seq 누락</td><td>' + seqGaps + ' 프레임</td></tr>' +
    '<tr><td>카메라 CPU</td><td>' + cpuText() + '</td></tr>' +
    '</table>';
}

// 한 프레임에 마커가 여러 개면 CAM_POSE 줄도 여러 개 온다. 줄마다 그리면 한 번의
// 시각적 갱신을 위해 DOM/캔버스를 N번 다시 만들게 되고, 화면이 깜빡인다.
//
// 두 가지로 막는다.
//  (1) 이중 버퍼 -- 수집은 rawBuilding 에 하고 프레임이 끝날 때 한 번에 교체한다.
//      예전처럼 프레임 시작에서 rawFrame 을 비우면 "마커 0개 -> 1개 -> 2개" 중간
//      상태가 그대로 그려져 깜빡임의 주된 원인이 된다.
//  (2) rAF 가드 -- 브라우저가 그릴 수 있는 속도보다 프레임이 빨리 와도 여러 프레임을
//      한 번의 페인트로 합친다. 데이터는 잃지 않는다(수집은 매 줄 동기적으로 계속됨).
let rawBuilding = {}, rawRenderScheduled = false;
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
  const sm = line.match(/seq=(\\d+)/);
  if (!sm) return;
  const lost = line.includes('MARKER LOST');
  const cm = line.match(/id=(\\d+)\\s+c0=\\(([\\d.]+),([\\d.]+)\\)\\s+c1=\\(([\\d.]+),([\\d.]+)\\)\\s+c2=\\(([\\d.]+),([\\d.]+)\\)\\s+c3=\\(([\\d.]+),([\\d.]+)\\)/);
  if (!cm && !lost) return;            // not a per-frame pose line
  // seq 가 바뀌면 "이전" 프레임이 완성된 것이다: 그것을 게시하고 새 수집을 시작한다.
  if (sm[1] !== rawSeq) {
    rawSeq = sm[1];
    rawFrame = rawBuilding;
    rawBuilding = {};
    scheduleRawRender();
  }
  if (cm) rawBuilding[cm[1]] = [[cm[2], cm[3]], [cm[4], cm[5]], [cm[6], cm[7]], [cm[8], cm[9]]];
}

// CALIB_K_QUERY 결과를 표로 렌더 (fx/fy/cx/cy + 왜곡계수 k1,k2,p1,p2,k3)
function renderKQuery(available, fx, fy, cx, cy, dist) {
  const box = document.getElementById('kquery');
  if (!available) {
    box.innerHTML = '<span class="none">카메라에 로드된 캘리브레이션 없음</span>';
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
  box.innerHTML = `<div class="qtitle">현재 로드된 K / 왜곡계수</div><table>${rows}</table>`;
}

// ===== 모드 탭 전환 (기존 기능은 그대로, 표시만 토글) =====
// Table-driven rather than boolean pairs: with three panes the old
// calib/not-calib flag no longer distinguishes them.
const TABS = {
  calib:      {pane: 'groups',         tab: 'tabCalib'},
  homography: {pane: 'homographyPane', tab: 'tabHmg'},
  raw:        {pane: 'rawPane',        tab: 'tabRaw'},
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
    send('HG_QUERY');
    send('MARKER_PLANE_QUERY');
    send('ANCHOR_QUERY');
    send('VALIDATION_QUERY');
  }
}

// ===== 캘리브레이션 대상 채널 (프로토콜 v0.4) =====
// 채널마다 렌즈 방향이 달라 K/D/H 가 전부 다르다. 여기서 고른 채널로 캘리 결과가
// 저장되고(H_MATRIX.ch), 카메라에도 SELECT_CHANNEL 이 나가 그 채널을 보게 된다.
// RP_CAM_CHANNELS=1 (PNO 단일 채널) 이면 선택 UI 자체가 뜨지 않는다.
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
  sel.value = st.channel;
  // 어느 채널이 아직 안 끝났는지 한눈에. H 가 없으면 서버로 전송 자체가 안 되므로
  // (push_calib_to_server) 그 채널은 Qt 에서 여전히 "캘리브레이션 없음"으로 보인다.
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
  const map = {compute: 'hgSubCompute', advanced: 'hgSubAdvanced'};
  for (const name in map)
    document.getElementById(map[name]).classList.toggle('active', name === section);
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
  send('ANCHOR_QUERY');
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
  send('ANCHOR_SET_ALL ' + values.map(a => `${a.id} ${a.wx} ${a.wy}`).join(' '));
}
// 카메라가 마지막으로 보고한 앵커. 중앙서버 번들의 canvas_mm 추정이 이 값을 쓴다
// (표의 입력값이 아니라 카메라가 실제로 들고 있는 값이어야 한다).
let hgCameraAnchors = null;

function handleHgAnchors(line) {
  const m = line.match(/^\\[calib\\] ANCHORS (\\[.*\\])$/);
  if (!m) return;
  try {
    const anchors = JSON.parse(m[1]);
    if (Array.isArray(anchors) && anchors.length) {
      hgCameraAnchors = anchors.map(a => ({id:Number(a.id), wx:Number(a.wx), wy:Number(a.wy)}));
      renderHgAnchorRows(anchors);
      hgAnchorEditStatus.textContent = '카메라의 현재 앵커 좌표를 표시했습니다.';
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
function queryHgValidation() {
  hgValidationEditStatus.textContent = '카메라의 현재 검증 기준점 목록을 조회하는 중…';
  send('VALIDATION_QUERY');
}
function applyHgValidation() {
  const values = readHgValidationRows();
  const anchorIds = new Set(readHgAnchorRows().map(a => a.id));
  if (values.length > 16 || values.some(a => !Number.isInteger(a.id) || a.id < 0 || !Number.isFinite(a.wx) || !Number.isFinite(a.wy))) {
    hgValidationEditStatus.textContent = '각 기준점의 ID/X/Y mm 값을 확인하세요.';
    return;
  }
  if (new Set(values.map(a => a.id)).size !== values.length) {
    hgValidationEditStatus.textContent = '검증 기준점 ID는 모두 달라야 합니다.';
    return;
  }
  if (values.some(a => anchorIds.has(a.id))) {
    hgValidationEditStatus.textContent = '검증 기준점 ID는 계산 앵커 ID와 겹칠 수 없습니다.';
    return;
  }
  if (!confirm(`${values.length}개 검증 기준점을 카메라에 적용할까요?`)) return;
  hgValidationEntries = values;
  hgValidationEditStatus.textContent = '검증 기준점 목록을 카메라에 적용하는 중…';
  send('VALIDATION_SET ' + values.map(a => `${a.id} ${a.wx} ${a.wy}`).join(' '));
}
function handleHgValidationConfig(line) {
  const m = line.match(/^\\[calib\\] VALIDATION (\\[.*\\])$/);
  if (!m) return;
  try {
    const markers = JSON.parse(m[1]);
    if (Array.isArray(markers)) {
      hgValidationEntries = markers;
      renderHgValidationRows();
      hgValidationEditStatus.textContent = '카메라의 현재 검증 기준점 목록을 표시했습니다.';
    }
  } catch (_) { /* keep editable values on a malformed reply */ }
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
  box.innerHTML = `<span class="qtitle" style="color:${color}">${title}</span> ${detail}`;
}
function renderHgMatrix(available, arr) {
  const box = document.getElementById('hgMatrix');
  if (!available || !arr || arr.length < 9) {
    box.innerHTML = '<span class="none">호모그래피 아직 계산 안 됨 (캘리브 시작 또는 H 행렬 조회)</span>';
    return;
  }
  let rows = '';
  for (let r = 0; r < 3; r++) {
    rows += '<tr>';
    for (let c = 0; c < 3; c++)
      rows += `<td style="text-align:right;color:var(--text);font-weight:600;padding:2px 14px 2px 0">${Number(arr[r * 3 + c]).toExponential(4)}</td>`;
    rows += '</tr>';
  }
  box.innerHTML = `<div class="qtitle">호모그래피 H (3×3, 픽셀 → mm)</div><table>${rows}</table>`;
}
// 중앙서버 번들이 싣는 H. 카메라가 보고한 값만 담는다.
let hgHfloor = null;
function handleHgMatrix(line) {
  const hm = line.match(/H=\\[([^\\]]*)\\]/);
  if (hm) {
    const arr = hm[1].split(',').map(s => s.trim()).filter(s => s.length);
    if (arr.length >= 9) {
      renderHgMatrix(true, arr);
      hgHfloor = arr.slice(0, 9).map(Number);
      if (hgHfloor.some(v => !Number.isFinite(v))) hgHfloor = null;
      if (line.includes('HOMOGRAPHY'))
        setHgHealth('H 사용 가능', '카메라에 적용된 H가 있습니다. 저장 여부는 마지막 저장 동작으로 확인하세요.', 'ok');
    }
  } else if (line.includes('호모그래피 아직 계산 안 됨')) {
    renderHgMatrix(false);
    hgHfloor = null;
    setHgHealth('H 없음', '1단계에서 H를 계산하거나 고급 분석 결과를 적용하세요.', 'warn');
  }
}
function handleHgStatus(line) {
  const box = document.getElementById('hgStatus');
  if (line.includes('[calib] camera acknowledged')) {
    box.innerHTML = '<span class="qtitle">캘리브 진행중…</span> 등록된 계산 anchor 마커가 <b>하나도 빠짐없이</b> 계속 보이게 유지하세요';
    setHgHealth('앵커 H 계산 중', '등록된 anchor가 전부 계속 보이게 유지하세요.', 'busy');
  } else if (line.includes('[calib] SUCCESS')) {
    box.innerHTML = '<span class="qtitle" style="color:var(--green)">캘리브 완료</span> ' + line.replace(/^.*\\[calib\\]\\s*/, '') + ' → 이제 world 좌표가 스트리밍됩니다';
    setHgHealth('앵커 H 적용됨 · 미저장', '2단계에서 검증한 뒤 3단계에서 저장하세요.', 'ok');
  } else if (line.includes('[calib] FAILED')) {
    box.innerHTML = '<span class="qtitle" style="color:var(--red)">캘리브 실패</span> ' + line.replace(/^.*\\[calib\\]\\s*/, '');
    setHgHealth('앵커 H 계산 실패', '아래 원인을 확인하고 다시 시도하세요.', 'warn');
  }
}
function handleHgPersistence(line) {
  if (line.includes('저장 완료'))
    setHgHealth('H 저장 완료', '현재 H가 /mnt에 저장되어 재부팅 뒤에도 유지됩니다.', 'ok');
  else if (line.includes('H 저장 실패'))
    setHgHealth('H 저장 실패', '현재 H는 적용되어 있을 수 있지만 영구 저장되지 않았습니다.', 'warn');
  else if (line.includes('PC 분석 H 적용 완료'))
    setHgHealth('PC 분석 H 적용됨 · 미저장', '검증 후 3단계에서 저장하세요.', 'ok');
}
function applyHgCoordMode() {
  hgCoordDefaultApplied = true;   // 수동 조작 뒤에는 기본값을 다시 밀어넣지 않는다
  send('HG_COORD_MODE ' + document.getElementById('hgCoordMode').value);
}
// QT 파이프라인이 undistortPoints -> H 순서라, raw 로 피팅한 H 는 조용히 왜곡만큼
// 틀린 좌표를 만든다. 그래서 보정 픽셀을 기본으로 두고 탭 진입 시 한 번 맞춘다.
let hgCoordDefaultApplied = false;
function applyHgCoordDefault() {
  if (hgCoordDefaultApplied) return;
  hgCoordDefaultApplied = true;
  send('HG_COORD_MODE undistort');
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

// 마커 검출 탭의 바닥 투영 오버레이가 쓰는 값. 카메라가 분해해 준 H_marker를
// 그대로 쓴다 — 부호 처리와 퇴화 판정이 이미 거기서 끝나 있다.
let mpPlane = null;   // {H_marker:[9], height_mm, ratio, ...}

function handleMarkerPlane(line) {
  const m = line.match(/^\\[calib\\] MARKER_PLANE (\\{.*\\})$/);
  if (!m) return;
  let d;
  try { d = JSON.parse(m[1]); } catch (_) { return; }

  if (d.type === 'MARKER_PLANE') {
    mpPlane = (d.ready && Array.isArray(d.H_marker) && d.H_marker.length >= 9)
            ? {H: d.H_marker.map(Number), height_mm: Number(d.height_mm),
               ratio: Number(d.ratio)}
            : null;
    redrawRawCanvas();
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
  const camInp = document.getElementById('mpCamHeight');
  if (camInp && d.camera_z_measured_mm !== undefined && document.activeElement !== camInp)
    camInp.value = d.camera_z_measured_mm;

  const derived  = Number(d.camera_z_mm);
  const measured = Number(d.camera_z_measured_mm || 0);
  const used     = Number(d.camera_z_used_mm !== undefined ? d.camera_z_used_mm : derived);

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
    `<tr><td>나디르</td><td>(${Number(d.nadir_x_mm).toFixed(1)}, ${Number(d.nadir_y_mm).toFixed(1)}) mm</td></tr>` +
    `<tr><td>보정 비율 h/Cz</td><td>${Number(d.ratio).toFixed(4)} (Cz=${used.toFixed(0)} mm)</td></tr>` +
    `</table>` +
    `<div class="hint">나디르에서 1000 mm 떨어진 지점의 마커는 약 ` +
    `<b>${(Number(d.ratio) * 1000).toFixed(0)} mm</b> 밀려 보입니다.</div>`;
}

// 로봇 마커 평면(시차 보정). 카메라가 H를 분해해 낸 값을 그대로 보여준다 —
// 브라우저는 계산하지 않는다.
function applyMarkerHeight() {
  const v = parseFloat(document.getElementById('mpHeight').value);
  if (!(v >= 0)) { alert('높이는 0 이상의 mm 값이어야 합니다'); return; }
  send('MARKER_HEIGHT ' + v);
}
function applyCameraHeight() {
  const v = parseFloat(document.getElementById('mpCamHeight').value);
  if (!(v >= 0)) { alert('카메라 높이는 0 이상의 mm 값이어야 합니다 (0 = 미측정)'); return; }
  send('CAMERA_HEIGHT ' + v);
}
// POS 미리보기. 카메라가 [central_tls_sender.cpp] 에서 만드는 줄과 같은 모양을
// 그대로 쓴다 -- 여기서 새로 규격을 정하지 않는다. 값은 지금 흐르는 CAM_POSE 의
// raw 코너를 그대로 넣는다(같은 픽셀이 실제로 POS 로도 나간다).
function fillCentralPosSample() {
  const note = document.getElementById('ctPosNote');
  const box = document.getElementById('ctPos');
  const idBox = document.getElementById('ctId');
  const want = idBox ? String(Number(idBox.value)) : '';
  const c = (want && rawFrame[want]) ? rawFrame[want] : null;
  const q = c ? c.map(pt => [Number(Number(pt[0]).toFixed(2)), Number(Number(pt[1]).toFixed(2))])
              : [[0, 0], [0, 0], [0, 0], [0, 0]];
  box.value = JSON.stringify({type: 'POS', seq: 0, payload: {corners: q}});
  if (!idBox || !want) note.textContent = '대상 id를 알 수 없습니다.';
  else if (c) note.textContent = `id ${want} 의 현재 프레임 코너를 넣었습니다.`;
  else note.textContent = `id ${want} 가 지금 화면에 없습니다 — 코너는 0 자리표시자입니다.`;
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
function fillCentralHmatrixTemplate() {
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
    missing.push('marker_height_m (마커 높이가 0 — 시차 보정이 꺼진 것과 같다)');

  // R-2 는 이 요청서의 최우선 항목이다. raw 로 피팅한 H 를 보내면 QT 가 왜곡
  // 보정을 켜는 순간 좌표가 발산한다 -- 에러가 아니라 그림이 깨져서 나타난다.
  if (!hgCoordModeActive) missing.push('coord_mode (HG_COORD_MODE 적용 이력 없음)');
  else if (hgCoordModeActive !== 'undistort')
    violations.push(`R-2 위반: coord_mode=${hgCoordModeActive} — undistort 로 재피팅이 필요합니다`);

  // R-4: 1920x1080 이어야 한다.
  // 기준영상 수신 경로가 이 대시보드에는 없다(스냅샷 패널 제거). 카메라 raw 는
  // 1920×1080 고정이므로 그 값을 싣되, 확인된 값이 아니라는 점을 남긴다.
  const size = [1920, 1080];
  missing.push('image_size (기준영상 없음 — 1920×1080 으로 가정)');

  const bundle = {
    calib_id:   calibId || 'MISSING',
    created_at: isoWithOffset(new Date()),
    image_size: size,
    coord_mode: hgCoordModeActive || 'unknown',
    unit:       'mm',                       // R-5 고정
    K: K,
    D: D,
    H: H || [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
    // R-0: 옛 서버는 H_floor 만 인식했다(H 단일 키는 파싱 실패로 번들 폐기). 현재 서버는
    // H_floor 를 먼저 보고 없을 때 H 로 넘어가므로, 같은 값을 두 이름으로 실으면 어느
    // 버전에서도 읽힌다. 값이 같으니 둘이 어긋날 여지도 없다.
    H_floor: H || [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
    H_marker: Hm || [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
    // 서버가 시차 보정에 쓰는 값. 카메라가 보고한 mm 를 m 로 바꿔 싣는다 -- 이 필드만
    // 미터인 것은 서버 스키마가 그렇게 정해져 있기 때문이다(H 계열은 mm, R-5).
    marker_height_m: mpPlane ? Number(mpPlane.height_mm) / 1000 : 0,
    origin_mm:  [0, 0],                     // R-6: 월드 원점 = 폼보드 좌하단
    canvas_mm:  (canvas[0] > 0 && canvas[1] > 0) ? canvas : [0, 0],
    axis:       'x_right_y_up',             // R-6 고정
  };
  document.getElementById('ctHm').value = JSON.stringify(bundle, null, 2);

  const note = document.getElementById('ctHmNote');
  if (!note) return;
  const parts = [];
  if (violations.length)
    parts.push('<b style="color:var(--red)">' + violations.join('<br>') + '</b>');
  if (missing.length)
    parts.push('<b style="color:var(--red)">자리표시자 포함 — 전송 전에 채우세요:</b> ' +
               missing.join(' · '));
  if (!violations.length && !missing.length)
    parts.push('<b style="color:var(--green)">QT-REQ-CCTV-001 rev.2 형식으로 채웠습니다.</b> ' +
               `coord_mode=${hgCoordModeActive}, canvas ${canvas[0]}×${canvas[1]}mm`);
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

  if (!hgCoordModeActive) missing.push('coord_mode (HG_COORD_MODE 적용 이력 없음)');

  const bundle = {
    calib: {
      version: 1,
      // QT checks these two before anything else: without them it has to guess
      // which pixel space H expects and which frame size K belongs to.
      coord_mode: hgCoordModeActive || 'unknown',
      image_size: [1920, 1080],
      K: K,
      D: D,
      H_floor:  Hf || [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
      H_marker: Hm || [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
      marker_height_m: mpPlane ? (Number(mpPlane.height_mm) / 1000.0) : 0.0
    }
  };
  document.getElementById('ctHm').value = JSON.stringify(bundle, null, 2);

  const note = document.getElementById('ctHmNote');
  if (note) {
    note.innerHTML = missing.length
      ? '<b style="color:var(--red)">자리표시자 포함 — 전송 전에 채우세요:</b> ' + missing.join(' · ')
      : '<b style="color:var(--green)">카메라의 현재 값으로 채웠습니다.</b> ' +
        `coord_mode=${hgCoordModeActive}, 마커 높이 ${mpPlane.height_mm} mm`;
  }
}
function sendCentralHmatrix() {
  const raw = document.getElementById('ctHm').value.trim();
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

function applyDynRoi() {
  const on = document.getElementById('dynRoiChk').checked ? 1 : 0;
  const m  = document.getElementById('dynMargin').value || 240;
  const n  = document.getElementById('dynMaxMiss').value || 4;
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

// 카메라는 변화만 속도 제한해 보내므로, 마지막으로 보고된 margin으로 박스를
// 근사 표시한다. 여기 값은 표시용이며 카메라의 실제 검출 ROI에는 영향이 없다.
let dynRoiOn = false, dynRoiMargin = 240, dynRoiTracking = false;

function handleDynRoi(line) {
  if (line.indexOf('[dynroi]') < 0) return;
  if (rawOverlayOn) setTimeout(redrawRawCanvas, 0);
  const el = document.getElementById('dynRoiState');
  // 추적 대상 id — 카메라가 실제로 파싱한 결과다. 화면의 라디오/입력칸을 여기에
  // 맞춰, 조작자가 친 값이 아니라 카메라가 받아들인 값이 보이게 한다.
  const idm = line.match(/^\\[dynroi\\] IDS (\\[.*\\])$/);
  if (idm) {
    let ids;
    try { ids = JSON.parse(idm[1]); } catch (_) { return; }
    if (!Array.isArray(ids)) return;
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
  if (line.indexOf('TRACK') >= 0) {
    dynRoiTracking = true;
    const m = line.match(/\\((\\d+),(\\d+)\\)\\s*(\\d+)x(\\d+)/);
    const used = line.match(/margin=(\\d+)px/);
    if (used) dynRoiMargin = Number(used[1]);
    if (el) { el.textContent = m ? ('상태: TRACK  ' + m[3] + '×' + m[4]) : '상태: TRACK';
              el.style.color = '#28a745'; }
  } else if (line.indexOf('SEARCH') >= 0) {
    dynRoiTracking = false;
    if (el) { el.textContent = '상태: SEARCH (재탐색)'; el.style.color = '#dc3545'; }
  } else if (line.indexOf('OFF') >= 0) {
    dynRoiOn = false; dynRoiTracking = false;
    if (el) { el.textContent = '상태: —'; el.style.color = ''; }
  } else if (line.indexOf('ON') >= 0) {
    dynRoiOn = true; dynRoiTracking = false;
    const m = line.match(/max_margin=(\\d+)px/);
    if (m) dynRoiMargin = Number(m[1]);
    if (el) { el.textContent = '상태: SEARCH (시작)'; el.style.color = ''; }
  }
  if (rawOverlayOn) redrawRawCanvas();
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

function toggleRawOverlay() {
  rawOverlayOn = !rawOverlayOn;
  const b = document.getElementById('rawOverlayBtn');
  b.textContent = rawOverlayOn ? '오버레이 정지' : '오버레이 보기 시작';
  b.classList.toggle('on', rawOverlayOn);
  if (rawOverlayOn && showUndist && !kCalib) send('CALIB_K_QUERY');
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
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity, n = 0;
  for (const id in rawFrame) {
    for (const p of rawFrame[id]) {
      const px = Number(p[0]), py = Number(p[1]);
      if (px < x0) x0 = px;  if (py < y0) y0 = py;
      if (px > x1) x1 = px;  if (py > y1) y1 = py;
      n++;
    }
  }
  if (!n) return null;
  let x = x0 - dynRoiMargin, y = y0 - dynRoiMargin;
  let w = (x1 - x0) + 2 * dynRoiMargin, h = (y1 - y0) + 2 * dynRoiMargin;
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
  const proc = procs.length ? procs[procs.length - 1] : null;

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

function redrawRawCanvas() {
  if (!rawCtx) return;
  // 캔버스 해상도: K가 있으면 (cx*2, cy*2), 없으면 1920×1080. raw 코너는 풀프레임
  // 픽셀 좌표라 캔버스가 프레임과 같은 크기여야 위치가 맞는다.
  const W = kCalib ? Math.round(kCalib.cx * 2) : 1920;
  const H = kCalib ? Math.round(kCalib.cy * 2) : 1080;
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
    } else if (!dynRoiTracking) {
      rawCtx.fillStyle = '#dc3545';
      rawCtx.fillText('SEARCH — 전체 화면 재탐색 중', 12, 26);
    }
  }
  for (const id in rawFrame) {
    const c = rawFrame[id];
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
es.onopen = () => { send('CALIB_K_STATUS'); send('CALIB_K_QUERY'); send('CALIB_K_PROFILE_LIST'); send('DETECT_ENABLE'); };
// 마커 검출 on/off. 상태는 절대 낙관적으로 바꾸지 않는다 - 카메라가 DETECT_ENABLE로
// 확인해 준 값만 반영한다. 카메라가 거부할 수 있고(호모그래피 수집 중), 링크가 끊겨
// 명령이 버려졌는데 버튼만 바뀌면 실제와 어긋난 상태를 보여주게 된다.
let detectOn = true;
function toggleDetect() { send('DETECT_ENABLE ' + (detectOn ? 0 : 1)); }
function renderDetectBtn() {
  const b = document.getElementById('detBtn');
  if (!b) return;
  b.classList.toggle('on', detectOn);
  b.textContent = detectOn ? '검출 ON' : '검출 OFF';
  b.title = detectOn ? '마커 검출 중 - 눌러서 끕니다 (DETECT_ENABLE 0)'
                     : '검출 꺼짐 - 눌러서 켭니다 (DETECT_ENABLE 1)';
}
function handleDetect(line) {
  const m = line.match(/detect_enabled=([01])/);
  if (!m) return;
  const was = detectOn;
  detectOn = m[1] === '1';
  renderDetectBtn();
  if (was && !detectOn) {
    // 마지막 프레임의 코너를 버린다. 남겨두면 검출이 멈춘 뒤에도 오버레이에
    // 옛 좌표가 그대로 떠 있어 살아있는 화면처럼 보인다.
    rawFrame = {}; rawBuilding = {}; rawSeq = null;
    if (rawOverlayOn) redrawRawCanvas();
  }
  if (rawOn) renderRaw();
}
renderDetectBtn();

es.onmessage = (e) => {
  handleRaw(e.data);
  handleLatency(e.data);
  handleHgStatus(e.data);
  handleHgAnchors(e.data);
  handleHgValidationConfig(e.data);
  handleHgMatrix(e.data);
  handleHgPersistence(e.data);
  handleHgCoordMode(e.data);
  handleKProfiles(e.data);
  handleShell(e.data);
  handleDetect(e.data);
  handleDynRoi(e.data);
  handleCpu(e.data);
  handleCentralStatus(e.data);
  handleMarkerPlane(e.data);
  // 이 탭 로그는 카메라 위주로 - 로봇/QT 트래픽은 로봇 탭(/robot)이나
  // 로그 모니터(/logs)에서 본다 (내부 상태 파싱은 위에서 이미 끝났으니 표시만 거른다).
  if (logSubject(e.data) === 'robot' || logSubject(e.data) === 'qt') return;
  if (hideLost && e.data.includes('MARKER LOST')) return;
  const stick = isNearBottom();
  log.textContent += e.data + "\\n";
  if (stick) log.scrollTop = log.scrollHeight;
  const gm = e.data.match(/gates=([01])/);
  if (gm) document.getElementById('gateChk').checked = gm[1] === '1';
  let qm;
  if ((qm = e.data.match(/CURRENT VALUES: fx=([-\\d.eE+]+) fy=([-\\d.eE+]+) cx=([-\\d.eE+]+) cy=([-\\d.eE+]+) dist=\\[([^\\]]*)\\]/))) {
    const dist = qm[5].split(',').map(s => s.trim()).filter(s => s.length);
    renderKQuery(true, qm[1], qm[2], qm[3], qm[4], dist);
    // 보정 좌표 표시용 캐시 갱신 (마커 검출 탭)
    kCalib = {
      fx: parseFloat(qm[1]), fy: parseFloat(qm[2]),
      cx: parseFloat(qm[3]), cy: parseFloat(qm[4]),
      dist: dist.map(parseFloat),
    };
    const pm = e.data.match(/profile=([^\\s]+)/);
    if (pm) { kActiveProfile = pm[1]; renderKProfiles(); }
    refreshUndistState();
    if (rawOn) renderRaw();
    if (rawOverlayOn) redrawRawCanvas();
  } else if (e.data.includes('no calibration loaded on the camera')) {
    renderKQuery(false);
    kCalib = null;
    refreshUndistState();
    if (rawOn) renderRaw();
    if (rawOverlayOn) redrawRawCanvas();
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
