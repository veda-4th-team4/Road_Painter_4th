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
    TCP_PORT,
    SNAPSHOT_PORT,
    SERVER_HOST,
    SERVER_PORT,
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
# pixel->world homography (CALIB_RESULT / HG_CHARUCO_RESULT / HG_SET: H, a flat
# 9-list, row-major) at separate times. We cache the latest of each and, once an
# H exists, send the combined {K,D,H_floor} bundle so the server persists it
# (user_store) and relays it to Qt. See ../src/calib.hpp for the bundle format.
# ---------------------------------------------------------------------------
_calib_cache = {"K": None, "dist": None, "H": None}
_calib_lock = threading.Lock()


def _reshape3x3(flat):
    f = [float(x) for x in flat]
    return [f[0:3], f[3:6], f[6:9]]


def calib_cache_k(fx, fy, cx, cy, dist):
    if fx is None or fy is None:
        return
    with _calib_lock:
        _calib_cache["K"] = [[float(fx), 0.0, float(cx or 0)],
                             [0.0, float(fy), float(cy or 0)],
                             [0.0, 0.0, 1.0]]
        _calib_cache["dist"] = [float(x) for x in (dist or [])]
    push_calib_to_server()


def calib_cache_h(h):
    if not isinstance(h, list) or len(h) != 9:
        return
    with _calib_lock:
        _calib_cache["H"] = _reshape3x3(h)
    push_calib_to_server()


def push_calib_to_server():
    """Send the cached calibration to the server (needs at least H_floor)."""
    with _calib_lock:
        H, K, dist = _calib_cache["H"], _calib_cache["K"], _calib_cache["dist"]
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
    if server_send("H_MATRIX", {"calib": bundle}):
        broadcast("[bridge] 캘리 결과를 서버로 전송(H_MATRIX, role=ADMIN) — 저장+Qt 중계됨")


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
    if mtype == "HG_CHARUCO_ACK":
        broadcast("[hg-charuco] 보드 대기 중 — 18개 이상 코너가 보이면 17개 피팅 + 나머지 검증")
        return last_seq
    if mtype == "HG_CHARUCO_PROGRESS":
        broadcast(f"[hg-charuco] corners={msg.get('corners')}/{msg.get('need')} — 보드가 더 보이게 하세요")
        return last_seq
    if mtype == "HG_CHARUCO_RESULT":
        if msg.get("ok"):
            broadcast(f"[hg-charuco] SUCCESS fit={msg.get('fit')} validation={msg.get('validation')} "
                      f"rmse={msg.get('rmse_mm')}mm max={msg.get('max_error_mm')}mm H={msg.get('H')}")
            calib_cache_h(msg.get("H"))  # 서버로 H_MATRIX 전송
        else:
            broadcast(f"[hg-charuco] FAILED: {msg.get('reason')}")
        return last_seq
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
  #groups, #homographyPane, #shellPane, #rawPane {
            display:flex; flex-flow:row wrap; align-content:flex-start;
            align-items:flex-start; gap:12px; flex:0 1 580px; min-width:0;
            overflow-y:auto; padding-right:4px; margin:0; }

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
     handleRaw()/handleHgWorld() rebuild these on every CAM_POSE line -- 4-10x a
     second -- and the row count follows how many markers that frame saw, with a
     MARKER LOST frame emptying them to a single line. Sized to content they
     yo-yo between ~40px and ~400px several times a second and shove everything
     below them around. A fixed box is the only thing that holds still; the
     scrollbar only appears when several markers are up at once (e.g. the four
     homography anchors). */
  #rawCorners { height:210px; overflow-y:auto; }
  #hgWorld { height:230px; overflow-y:auto; }
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
  #shOut { background:var(--console); color:var(--console-text); padding:14px; margin:10px 0 0;
           min-height:220px; max-height:46vh; overflow:auto; white-space:pre-wrap;
           word-break:break-all; font-family:var(--mono); font-size:12px;
           line-height:1.6; border-radius:14px; }

  /* Below this the side-by-side split stops working: the controls pane and the
     log would each be too narrow to read. Stack them and hand scrolling back to
     the page -- the desktop layout deliberately pins the viewport
     (height:100vh; overflow:hidden) so the log scrolls internally, but that
     same rule would clip content on a phone. */
  @media (max-width: 900px) {
    body { height:auto; min-height:100vh; overflow:auto; }
    #content { flex-direction:column; }
    #groups, #homographyPane, #shellPane, #rawPane {
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
    <button type="button" id="tabShell" class="tab" onclick="showTab('shell')">셸</button>
  </div>
  <div id="cmdbox">
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
    <h2>검출 영역 (ROI)</h2>
    <p class="sub">검출 비용은 화소 수에 비례합니다. 영역을 좁히면 그만큼 빨라집니다.</p>
    <div class="row">
      <button onclick="send('ROI_SET 0 0 0 0')">전체 화면<span class="cmd">ROI_SET 0 0 0 0</span></button>
      <button onclick="send('ROI_SET 960 0 960 1080')">오른쪽 절반<span class="cmd">960,0 960x1080</span></button>
      <button onclick="send('ROI_SET 0 0 960 1080')">왼쪽 절반<span class="cmd">0,0 960x1080</span></button>
      <button onclick="send('ROI_SET ' + [roiX,roiY,roiW,roiH].map(i=>document.getElementById(i).value||0).join(' '))">직접 지정<span class="cmd">ROI_SET</span></button>
      <label>x <input type="number" id="roiX" value="960" style="width:5em"></label>
      <label>y <input type="number" id="roiY" value="0" style="width:5em"></label>
      <label>w <input type="number" id="roiW" value="960" style="width:5em"></label>
      <label>h <input type="number" id="roiH" value="1080" style="width:5em"></label>
    </div>
    <details class="fold panel" id="foldRoi">
      <summary>다운스케일과 다른 점, 그리고 주의할 것</summary>
      <div class="foldbody hint">
        해상도를 낮추는 방식은 화소를 줄이는 대신 <b>마커도 같이 작아져</b> 못 찾을 위험이
        생긴다. ROI는 해상도를 그대로 두고 <b>보는 범위만</b> 좁히므로 마커 크기가 유지된다.
        카메라가 고정이고 작업 구역이 화면의 일정 자리에만 잡히기 때문에 쓸 수 있는 방법이다.<br>
        <b>주의</b>: 마커가 이 사각형을 <b>완전히</b> 벗어나거나 걸치면 검출되지 않는다 —
        에러가 아니라 <code>MARKER LOST</code>로 보인다. 작업 구역 둘레로 <b>마커 한 변
        이상의 여유</b>를 두고 잡을 것.<br>
        보고되는 코너 좌표는 <b>항상 전체 화면 기준</b>이다(카메라가 ROI 원점을 더해서 보냄).
        따라서 호모그래피나 서버 계산은 ROI를 몰라도 된다.
      </div>
    </details>
  </div>

  <div class="group wide">
    <h2>검출 파라미터 — 이진화 스캔 횟수</h2>
    <p class="sub">검출 속도와 강건성의 트레이드오프를 실기에서 비교합니다.</p>
    <div class="row">
      <button onclick="send('ARUCO_SCAN 3')">3회 (기본)<span class="cmd">ARUCO_SCAN 3</span></button>
      <button onclick="send('ARUCO_SCAN 2')">2회<span class="cmd">ARUCO_SCAN 2</span></button>
      <button onclick="send('ARUCO_SCAN 1 ' + (document.getElementById('scanWin').value || 13))">1회<span class="cmd">ARUCO_SCAN 1</span></button>
      <label>1회일 때 창 크기 <input type="number" id="scanWin" value="13" min="3" step="2" style="width:5em"></label>
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
    <p class="sub">기준영상 1장을 배경으로 깔고 raw 코너(주황)와 보정 코너(청록)를 겹쳐 그립니다. 왜곡 보정이 코너를 어디로 얼마나 미는지 눈으로 확인.</p>
    <div class="row go"><button onclick="rawSnapshot()">기준영상 스냅샷<span class="cmd">HG_SNAPSHOT</span></button>
      <button id="rawOverlayBtn" type="button" onclick="toggleRawOverlay()">오버레이 보기 시작</button>
      <span class="desc">스냅샷은 호모그래피 탭과 같은 기준영상을 공유합니다. 배경이 없어도 좌표는 그려집니다.</span></div>
    <details class="fold panel">
      <summary>주황 = raw(왜곡 전) · 청록 = 보정(undistort) · 점선 = 코너 이동량</summary>
      <div class="foldbody hint">
        raw 코너는 카메라가 실제 전송하는 왜곡 픽셀이고, 보정 코너는 위 "보정 좌표 함께 표시"와
        같은 K/dist로 브라우저가 <code>undistortPoints</code> 반복해로 계산한 값이다. 두 점을 잇는
        점선이 왜곡 보정에 따른 코너 이동 방향·크기(px, id 라벨에 Δ로 표기)다. K가 없으면 raw만 그린다.
        표시 전용 — 카메라에 별도 명령을 보내지 않는다(스냅샷 제외).
      </div>
    </details>
    <div id="rawCanvasNote" class="hint" style="margin:6px 0"><span class="none">기준영상 없음 — 스냅샷을 받으면 배경이 깔립니다 (없어도 좌표는 표시)</span></div>
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
  </div>
</div>
<div id="homographyPane" style="display:none">
  <div class="hg-subtabs" role="tablist" aria-label="호모그래피 작업">
    <button type="button" class="hg-subtab active" id="hgSubCompute" onclick="showHgSection('compute')">계산</button>
    <button type="button" class="hg-subtab" id="hgSubValidate" onclick="showHgSection('validate')">검증</button>
    <button type="button" class="hg-subtab" id="hgSubAdvanced" onclick="showHgSection('advanced')">고급</button>
  </div>

  <details class="group wide fold panel" id="foldHgHealth" data-hg-section="compute" open>
    <summary>현재 호모그래피 상태</summary>
    <div class="foldbody">
      <p class="sub">H를 계산한 뒤 현장 검증을 하고, 사용할 값만 저장하세요.</p>
      <div id="hgHealth" class="qbox"><span class="none">H 상태를 확인하는 중…</span></div>
      <div class="row"><label class="kparam">H 입력 좌표계<select id="hgCoordMode"><option value="raw">raw 픽셀</option><option value="undistort">K/dist 보정 픽셀</option></select></label>
        <button type="button" onclick="applyHgCoordMode()">좌표계 적용<span class="cmd">HG_COORD_MODE</span></button></div>
      <div id="hgCoordStatus" class="hint">모드를 바꾸면 기존 H는 다른 좌표계의 값이므로 H를 다시 계산하세요.</div>
    </div>
  </details>

  <details class="group wide fold panel" id="foldHgAnchors" data-hg-section="compute">
    <summary>1. H 만들기 — 앵커 마커 8개 <span class="brief">권장</span></summary>
    <div class="foldbody">
      <p class="sub">바닥 계산 anchor id 10~17이 모두 보이게 한 뒤 계산합니다. 검증 마커 id 20~23은 계산에 넣지 마세요.</p>
      <div class="tip" style="padding:11px 13px">
        <b>계산 방식</b><br>
        1) 각 프레임에서 id 10~17의 <b>마커 중심 픽셀 좌표</b>를 찾습니다.<br>
        2) 8개가 모두 보인 프레임 30장을 모아, ID별 픽셀 중심을 평균냅니다. 일부라도 안 보인 프레임은 버립니다.<br>
        3) 평균 픽셀 좌표와 해당 ID에 등록된 <b>실측 바닥 좌표(mm)</b>를 8쌍으로 대응시켜, <code>픽셀 → mm</code> 3×3 H를 RANSAC(20 px 임계값)으로 계산합니다.<br>
        4) 8점 중 최소 6점이 일치점이어야 성공하며, 최대 300프레임(약 60초) 안에 30장을 못 모으면 실패합니다.<br>
        아래는 카메라의 기본 설정값입니다. 8개 슬롯의 마커 ID와 X/Y를 현장 배치에 맞게 바꿀 수 있으며, ID는 서로 달라야 합니다.
      </div>
      <div class="qbox"><div class="qtitle">앵커 바닥 좌표 편집 (X, Y mm)</div>
        <div id="hgAnchorRows"></div>
        <div class="row"><button type="button" onclick="addHgAnchorRow()">앵커 추가</button>
          <button type="button" onclick="queryHgAnchors()">현재 값 조회<span class="cmd">ANCHOR_QUERY</span></button>
          <button type="button" onclick="applyHgAnchors()">입력값 모두 적용<span class="cmd">ANCHOR_SET_SLOT × 8</span></button></div>
        <div class="hint">카메라에 즉시 적용됩니다. 캘리브레이션 중에는 수정할 수 없으며, ID/X/Y 자체는 카메라 재시작 뒤 기본값으로 돌아갑니다. H 계산 결과는 별도의 <b>현재 H 저장</b>으로 보존하세요.</div>
        <div id="hgAnchorEditStatus" class="hint"></div>
      </div>
      <div class="cmdflow">
        <div class="row go"><button id="hgCalibStartBtn" onclick="send('CALIB_START')">앵커 8개로 H 계산<span class="cmd">CALIB_START</span></button></div>
      </div>
      <div id="hgStatus" class="qbox"><span class="none">캘리브 시작으로 계산 anchor 8점 호모그래피를 계산하세요</span></div>
    </div>
  </details>

  <div class="group wide" data-hg-section="validate">
    <h2>현장 검증 — 두 마커 거리</h2>
    <div>
      <p class="sub">자 양 끝의 두 마커로 계산거리와 실제길이를 비교합니다.</p>
      <div class="cmdflow">
        <div class="row"><button id="hgWorldBtn" onclick="toggleHgWorld()">월드 좌표 보기 시작</button></div>
      </div>
      <div class="row">
        <label class="kparam">마커 A id<input id="hgIdA" type="number" min="0" style="width:56px"></label>
        <label class="kparam">마커 B id<input id="hgIdB" type="number" min="0" style="width:56px"></label>
        <label class="kparam">자 실제 길이(mm)<input id="hgRuler" type="number" step="1" style="width:84px"></label>
      </div>
      <div class="hint">자 양 끝에 마커 A·B를 붙이고 <b>두 마커 중심 간 실제 거리(mm)</b>를 입력하세요. 바닥에서 옮겨가며 <b>계산거리 vs 실제길이</b> 오차를 실시간 확인합니다. (두 마커는 바닥에 밀착, anchor 사각형 안에서, 캘리브 완료 후 world값 스트리밍)</div>
      <div id="hgWorld" class="qbox"><span class="none">월드 좌표 보기를 시작하고 마커를 비추세요</span></div>
    </div>
  </div>

  <div class="group wide" data-hg-section="validate">
    <h2>등록 기준점 검증 — id 20~23</h2>
    <div>
      <p class="sub">이 목록의 마커는 H 계산에 넣지 않고, 실측 등록 좌표와 결과를 비교합니다.</p>
      <div class="qbox"><div class="qtitle">검증 기준점 편집 (ID / X, Y mm)</div>
        <div id="hgValidationRows"></div>
        <div class="row"><button type="button" onclick="addHgValidationRow()">기준점 추가</button>
          <button type="button" onclick="queryHgValidation()">현재 값 조회<span class="cmd">VALIDATION_QUERY</span></button>
          <button type="button" onclick="applyHgValidation()">입력값 모두 적용<span class="cmd">VALIDATION_SET</span></button></div>
        <div class="hint">0~16개를 설정할 수 있습니다. ID는 서로 달라야 하며 계산 앵커 ID와 겹칠 수 없습니다. 이 목록은 카메라 재시작 뒤 기본값으로 돌아갑니다.</div>
        <div id="hgValidationEditStatus" class="hint"></div>
      </div>
      <div class="hint">기준영상 스냅샷을 한 번 받은 뒤 검증 마커를 비추세요. 표에는 X/Y 오차와 위치 오차, 하단에는 RMSE와 최대 오차가 표시됩니다.</div>
      <div id="hgValidation" class="qbox"><span class="none">기준영상 스냅샷 및 H 캘리브레이션 후 검증 마커를 비추세요</span></div>
    </div>
  </div>

  <div class="group wide" data-hg-section="validate">
    <h2>검증 보조 — 바닥 기준영상 오버레이</h2>
    <div>
    <p class="sub">기준영상 1장 위에 anchor 작업영역과 실시간 마커를 겹쳐 그립니다.</p>
    <div class="cmdflow">
      <div class="row go"><button onclick="hgSnapshot()">기준영상 스냅샷<span class="cmd">HG_SNAPSHOT</span></button>
        <span class="desc">카메라가 현재 프레임 1장을 JPEG로 보내 배경으로 깝니다. 계산 anchor(id 10~17)가 다 보일 때 누르세요.</span></div>
      <div class="row"><button id="hgOverlayBtn" onclick="toggleHgOverlay()">오버레이 보기 시작</button>
        <span class="desc">실시간 마커 사각형·중심·id·월드좌표와 anchor 작업영역(내부 초록/외부 빨강)을 배경 위에 그립니다.</span></div>
    </div>
    <div class="hint">배경은 정지 스냅샷(바닥은 안 움직임), 그 위 마커만 <code>CAM_POSE</code>로 실시간 갱신됩니다. H가 있으면 마커 중심의 월드 mm도 라벨로 표시. 색: 작업영역 안=초록, 밖=빨강.</div>
    <canvas id="hgCanvas" width="640" height="480" style="max-width:100%;height:auto;display:block;border:1px solid #444;background:#111;border-radius:8px;margin-top:8px"></canvas>
    <div id="hgCanvasNote" class="qbox"><span class="none">기준영상 스냅샷을 눌러 배경을 받아오세요</span></div>
    </div>
  </div>

  <details class="group wide fold panel" id="foldHgSave" data-hg-section="compute">
    <summary>3. 확정 및 저장</summary>
    <div class="foldbody">
      <p class="sub">검증 결과가 만족스러울 때만 현재 적용 H를 저장하세요. 저장 전에도 H는 즉시 좌표 변환에 사용됩니다.</p>
      <div class="row go"><button onclick="send('HG_SAVE')">현재 H 저장<span class="cmd">HG_SAVE</span></button>
        <span class="desc">카메라 /mnt(PERSIST_DIR)에 기록합니다. 저장해야 재부팅 뒤에도 유지됩니다.</span></div>
    </div>
  </details>

  <details class="group wide fold panel" id="foldHgCharuco" data-hg-section="compute">
    <summary>대체 방식 — ChArUco 보드로 H 계산 및 검증</summary>
    <div class="foldbody">
      보드를 바닥에 평평하게 고정한 뒤 실행합니다. 현재 ChArUco 보드 규격의 코너 좌표(mm)로 17점을 피팅하고, 나머지 검출 코너로 독립 검증합니다. <b>성공하면 계산된 H가 즉시 현재 H로 적용됩니다.</b>
      <div class="row go"><button onclick="send('HG_CHARUCO_START')">ChArUco 17점 H 계산<span class="cmd">HG_CHARUCO_START</span></button></div>
      <div id="hgCharucoStatus" class="qbox"><span class="none">최소 18개 코너가 필요합니다.</span></div>
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
    <div class="row" id="shellControls">
      <input id="shInput" type="text"
             placeholder="예: ls -lah /mnt   (Enter 실행 · ↑/↓ 이전 명령)" autocomplete="off">
      <button onclick="sendShell()">실행</button>
      <button onclick="document.getElementById('shOut').textContent=''">지우기</button>
      <button onclick="copyShellOutput()">출력 복사</button>
    </div>
    <h3 style="margin:16px 0 4px;font-size:13px">기본 상태</h3>
    <div class="cmdflow">
      <div class="row"><button onclick="runShell('date; uptime; id')">시간 · 업타임 · 권한</button></div>
      <div class="row"><button onclick="runShell('ps | head -n 20')">프로세스 목록</button></div>
    </div>
    <h3 style="margin:14px 0 4px;font-size:13px">/mnt 저장소</h3>
    <div class="cmdflow">
      <div class="row"><button onclick="runShell('ls -lah /mnt/opensdk/storage/')">앱 저장소 목록</button></div>
      <div class="row"><button onclick="runShell('df -h /mnt')">/mnt 용량</button></div>
      <div class="row"><button onclick="runShell('mount | grep mnt')">/mnt 마운트 옵션</button></div>
      <div class="row"><button onclick="runShell('ls -ld /mnt /mnt/opensdk /mnt/opensdk/storage')">폴더 권한</button></div>
      <div class="row"><button onclick="runShell('touch /mnt/__wtest && echo WRITABLE && rm -f /mnt/__wtest')">/mnt 쓰기 테스트</button></div>
    </div>
    <div class="hint" style="margin:10px 0 2px">호모그래피·K/dist 저장 파일은 앱의 <code>/mnt/opensdk/storage/&lt;appName&gt;/</code> 아래에 있을 수 있습니다.</div>
    <pre id="shOut"></pre>
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
const rawLatency = document.getElementById('rawLatency');

function handleLatency(line) {
  const pm = line.match(/proc=(-?\\d+)ms/);
  if (!pm) return;
  procs.push(Number(pm[1]));
  if (procs.length > PROC_WINDOW) procs.shift();

  const nm = line.match(/net=(-?\\d+)ms/);
  lastNet = nm ? Number(nm[1]) : null;      // absent/?clock -> unknown

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
    '</table>';
}

function handleRaw(line) {
  // rawFrame is shared with the homography overlay, so parse regardless of the
  // marker-detection tab's toggle; only gate the DOM renders below.
  const sm = line.match(/seq=(\\d+)/);
  if (!sm) return;
  const lost = line.includes('MARKER LOST');
  const cm = line.match(/id=(\\d+)\\s+c0=\\(([\\d.]+),([\\d.]+)\\)\\s+c1=\\(([\\d.]+),([\\d.]+)\\)\\s+c2=\\(([\\d.]+),([\\d.]+)\\)\\s+c3=\\(([\\d.]+),([\\d.]+)\\)/);
  if (!cm && !lost) return;            // not a per-frame pose line
  if (sm[1] !== rawSeq) { rawSeq = sm[1]; rawFrame = {}; }   // new frame -> reset
  if (cm) rawFrame[cm[1]] = [[cm[2], cm[3]], [cm[4], cm[5]], [cm[6], cm[7]], [cm[8], cm[9]]];
  if (rawOn) renderRaw();
  renderHgValidation();
  if (hgOverlayOn) redrawHg();
  if (rawOverlayOn) redrawRawCanvas();
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
};
function showTab(name) {
  for (const k in TABS) {
    document.getElementById(TABS[k].pane).style.display = (k === name) ? 'flex' : 'none';
    document.getElementById(TABS[k].tab).classList.toggle('active', k === name);
  }
  if (name === 'shell') document.getElementById('shInput').focus();
  if (name === 'homography') {
    send('HG_QUERY');
    send('ANCHOR_QUERY');
    send('VALIDATION_QUERY');
  }
}

// ===== Homography workflow sections =====
function showHgSection(section) {
  document.querySelectorAll('[data-hg-section]').forEach(panel => {
    panel.style.display = panel.dataset.hgSection === section ? 'block' : 'none';
  });
  const map = {compute: 'hgSubCompute', validate: 'hgSubValidate', advanced: 'hgSubAdvanced'};
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
  send('SHELL ' + c);
}
function sendShell() {
  const c = shInput.value.trim();
  if (!c) return;
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
function handleShell(line) {
  if (!line.startsWith('[shell]')) return;
  const stick = shOut.scrollHeight - shOut.scrollTop - shOut.clientHeight < 40;
  shOut.textContent += line.slice(8) + "\\n";
  if (stick) shOut.scrollTop = shOut.scrollHeight;
}

// ===== Homography Test 탭 =====
const defaultHgAnchors = [
  {id: 10, wx: 0, wy: 0}, {id: 11, wx: 1000, wy: 0},
  {id: 12, wx: 2000, wy: 0}, {id: 13, wx: 2000, wy: 750},
  {id: 14, wx: 2000, wy: 1500}, {id: 15, wx: 1000, wy: 1500},
  {id: 16, wx: 0, wy: 1500}, {id: 17, wx: 0, wy: 750},
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
    html += `<tr><td><input id="hgAnchorId${slot}" type="number" min="0" step="1" value="${Number(a.id)}" style="width:58px"></td>` +
      `<td><input id="hgAnchorX${slot}" type="number" step="0.1" value="${Number(a.wx)}" style="width:92px"></td>` +
      `<td><input id="hgAnchorY${slot}" type="number" step="0.1" value="${Number(a.wy)}" style="width:92px"></td>` +
      `<td><button type="button" onclick="removeHgAnchorRow(${slot})">삭제</button></td></tr>`;
  }
  hgAnchorRows.innerHTML = html + '</table>';
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
function applyHgAnchors() {
  const values = readHgAnchorRows();
  for (const a of values) {
    const {slot, id, wx, wy} = a;
    if (!Number.isInteger(id) || id < 0 || !Number.isFinite(wx) || !Number.isFinite(wy)) {
      hgAnchorEditStatus.textContent = `${slot + 1}번 앵커의 ID/X/Y mm 값을 확인하세요.`;
      return;
    }
  }
  if (new Set(values.map(a => a.id)).size !== values.length) {
    hgAnchorEditStatus.textContent = '앵커 ID 8개는 모두 달라야 합니다.';
    return;
  }
  if (!confirm(`입력한 ${values.length}개 앵커를 카메라에 적용할까요? 진행 중인 앵커 캘리브레이션은 먼저 끝내야 합니다.`)) return;
  hgAnchorEntries = values;
  hgAnchorEditStatus.textContent = '8개 앵커 좌표를 카메라에 적용하는 중…';
  // The camera answers each command with its complete authoritative table.
  values.forEach((a, i) => setTimeout(() => send(`ANCHOR_SET_SLOT ${a.slot} ${a.id} ${a.wx} ${a.wy}`), i * 120));
}
function handleHgAnchors(line) {
  const m = line.match(/^\\[calib\\] ANCHORS (\\[.*\\])$/);
  if (!m) return;
  try {
    const anchors = JSON.parse(m[1]);
    if (Array.isArray(anchors) && anchors.length) {
      renderHgAnchorRows(anchors);
      hgAnchorEditStatus.textContent = '카메라의 현재 앵커 좌표를 표시했습니다.';
    }
  } catch (_) { /* leave the editable values intact on a malformed reply */ }
}
renderHgAnchorRows(defaultHgAnchors);

const defaultHgValidation = [
  {id: 20, wx: 500, wy: 500}, {id: 21, wx: 1500, wy: 500},
  {id: 22, wx: 1500, wy: 1000}, {id: 23, wx: 500, wy: 1000},
];
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
    html += `<tr><td><input id="hgValId${i}" type="number" min="0" step="1" value="${Number(a.id)}" style="width:58px"></td>` +
      `<td><input id="hgValX${i}" type="number" step="0.1" value="${Number(a.wx)}" style="width:92px"></td>` +
      `<td><input id="hgValY${i}" type="number" step="0.1" value="${Number(a.wy)}" style="width:92px"></td>` +
      `<td><button type="button" onclick="removeHgValidationRow(${i})">삭제</button></td></tr>`;
  });
  hgValidationRows.innerHTML = html + '</table>';
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

let hgWorldOn = false, hgSeq = null, hgFrame = {};
function toggleHgWorld() {
  hgWorldOn = !hgWorldOn;
  const b = document.getElementById('hgWorldBtn');
  b.textContent = hgWorldOn ? '월드 좌표 보기 정지' : '월드 좌표 보기 시작';
  b.classList.toggle('on', hgWorldOn);
  if (!hgWorldOn) {
    document.getElementById('hgWorld').innerHTML = '<span class="none">정지됨</span>';
    hgFrame = {}; hgSeq = null;
  }
}
function renderHgWorld() {
  const box = document.getElementById('hgWorld');
  const ids = Object.keys(hgFrame);
  if (!ids.length) {
    box.innerHTML = '<span class="none">이 프레임에 world 좌표를 가진 마커 없음 — 호모그래피 캘리브 완료 여부 확인</span>';
    return;
  }
  // 1) 검출된 마커별 world 좌표
  let html = '<div class="qtitle">마커 월드 좌표 (mm, deg)</div><table>';
  html += '<tr><td>id</td><td>X(mm)</td><td>Y(mm)</td><td>θ(deg)</td></tr>';
  for (const id of ids) {
    const w = hgFrame[id];
    html += `<tr><td>${id}</td><td>${w[0]}</td><td>${w[1]}</td><td>${w[2]}</td></tr>`;
  }
  html += '</table>';
  // 2) 두 마커 거리 검증 (A↔B world 거리 vs 자 실제 길이)
  const idA = document.getElementById('hgIdA').value;
  const idB = document.getElementById('hgIdB').value;
  const ruler = parseFloat(document.getElementById('hgRuler').value);
  html += '<div class="qtitle" style="margin-top:8px">거리 검증 (A↔B)</div>';
  if (idA && idB && hgFrame[idA] && hgFrame[idB]) {
    const a = hgFrame[idA], b = hgFrame[idB];
    const dist = Math.hypot(Number(a[0]) - Number(b[0]), Number(a[1]) - Number(b[1]));
    let extra = '';
    if (!isNaN(ruler) && ruler > 0) {
      const err = dist - ruler, pct = err / ruler * 100;
      const sg = v => (v >= 0 ? '+' : '');
      extra = `<tr><td>실제 길이</td><td>${ruler.toFixed(1)} mm</td></tr>` +
              `<tr><td>오차</td><td>${sg(err)}${err.toFixed(1)} mm (${sg(pct)}${pct.toFixed(2)}%)</td></tr>`;
    }
    html += `<table><tr><td>계산 거리 (id ${idA}↔${idB})</td><td>${dist.toFixed(1)} mm</td></tr>${extra}</table>`;
  } else {
    html += `<span class="none">마커 A(${idA || '?'})·B(${idB || '?'}) 둘 다 이 프레임에 보여야 거리 계산</span>`;
  }
  box.innerHTML = html;
}
function handleHgWorld(line) {
  if (!hgWorldOn) return;
  const sm = line.match(/seq=(\\d+)/);
  if (!sm) return;
  const wm = line.match(/id=(\\d+).*world=\\(([-\\d.]+),([-\\d.]+)mm,([-\\d.]+)deg\\)/);
  const lost = line.includes('MARKER LOST');
  if (!wm && !lost) return;
  if (sm[1] !== hgSeq) { hgSeq = sm[1]; hgFrame = {}; }
  if (wm) hgFrame[wm[1]] = [wm[2], wm[3], wm[4]];
  renderHgWorld();
}
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
function handleHgMatrix(line) {
  const hm = line.match(/H=\\[([^\\]]*)\\]/);
  if (hm) {
    const arr = hm[1].split(',').map(s => s.trim()).filter(s => s.length);
    if (arr.length >= 9) {
      renderHgMatrix(true, arr);
      if (line.includes('HOMOGRAPHY'))
        setHgHealth('H 사용 가능', '카메라에 적용된 H가 있습니다. 저장 여부는 마지막 저장 동작으로 확인하세요.', 'ok');
    }
  } else if (line.includes('호모그래피 아직 계산 안 됨')) {
    renderHgMatrix(false);
    setHgHealth('H 없음', '1단계에서 H를 계산하거나 고급 분석 결과를 적용하세요.', 'warn');
  }
}
function handleHgStatus(line) {
  const box = document.getElementById('hgStatus');
  if (line.includes('[calib] camera acknowledged')) {
    box.innerHTML = '<span class="qtitle">캘리브 진행중…</span> 계산 anchor 마커(id 10~17)가 계속 보이게 유지하세요';
    setHgHealth('앵커 H 계산 중', 'anchor id 10~17이 계속 보이게 유지하세요.', 'busy');
  } else if (line.includes('[calib] SUCCESS')) {
    box.innerHTML = '<span class="qtitle" style="color:var(--green)">캘리브 완료</span> ' + line.replace(/^.*\\[calib\\]\\s*/, '') + ' → 이제 world 좌표가 스트리밍됩니다';
    setHgHealth('앵커 H 적용됨 · 미저장', '2단계에서 검증한 뒤 3단계에서 저장하세요.', 'ok');
  } else if (line.includes('[calib] FAILED')) {
    box.innerHTML = '<span class="qtitle" style="color:var(--red)">캘리브 실패</span> ' + line.replace(/^.*\\[calib\\]\\s*/, '');
    setHgHealth('앵커 H 계산 실패', '아래 원인을 확인하고 다시 시도하세요.', 'warn');
  }
}
function handleHgCharuco(line) {
  const box = document.getElementById('hgCharucoStatus');
  if (!box || !line.includes('[hg-charuco]')) return;
  if (line.includes('SUCCESS')) {
    box.innerHTML = '<span class="qtitle" style="color:var(--green)">ChArUco H 계산 완료</span> ' + line.replace(/^.*\\[hg-charuco\\]\\s*/, '');
    setHgHealth('ChArUco H 적용됨 · 미저장', '검증 결과를 확인한 뒤 3단계에서 저장하세요.', 'ok');
  } else if (line.includes('FAILED')) {
    box.innerHTML = '<span class="qtitle" style="color:var(--red)">ChArUco H 계산 실패</span> ' + line.replace(/^.*\\[hg-charuco\\]\\s*/, '');
    setHgHealth('ChArUco H 계산 실패', '보드가 평평한지와 검출 코너 수를 확인하세요.', 'warn');
  } else {
    box.textContent = line.replace(/^.*\\[hg-charuco\\]\\s*/, '');
    setHgHealth('ChArUco 보드 대기 중', '최소 18개 내부 코너가 보일 때까지 기다립니다.', 'busy');
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
function applyHgCoordMode() { send('HG_COORD_MODE ' + document.getElementById('hgCoordMode').value); }
function handleHgCoordMode(line) {
  if (!line.includes('[hg-coord]')) return;
  const box = document.getElementById('hgCoordStatus');
  if (line.includes('SUCCESS')) {
    const undist = line.includes('undistort');
    document.getElementById('hgCoordMode').value = undist ? 'undistort' : 'raw';
    box.textContent = undist ? 'K/dist 보정 좌표계 적용됨 — H를 다시 계산하세요.' : 'raw 픽셀 좌표계 적용됨 — H를 다시 계산하세요.';
  } else box.textContent = line;
}

// ===== 호모그래피 캔버스 오버레이 (기준영상 + 실시간 마커) =====
// 카메라 HG_SNAPSHOT이 보낸 정지 기준영상(/hg_reference.jpg)을 배경으로 깔고,
// 그 위에 anchor 작업영역과 CAM_POSE로 흐르는 실시간 마커를 겹쳐 그린다.
let hgOverlayOn = false, hgMeta = null, hgImgLoaded = false;
const hgImg = new Image();
const hgCanvas = document.getElementById('hgCanvas');
const hgCtx = hgCanvas ? hgCanvas.getContext('2d') : null;
const hgCanvasNote = document.getElementById('hgCanvasNote');

function hgSnapshot() {
  send('HG_SNAPSHOT');   // 카메라가 기준영상 1장을 스냅샷 채널로 전송 → 파이 저장
  hgCanvasNote.innerHTML = '<span class="qtitle">기준영상 요청 중…</span> 잠시 후 배경이 갱신됩니다';
  setTimeout(loadHgReference, 900);   // 파이가 저장할 시간을 준 뒤 로드
}
function loadHgReference() {
  fetch('/hg_meta?_=' + Date.now())
    .then(r => r.json())
    .then(m => { hgMeta = (m && m.w) ? m : null; })
    .catch(() => { hgMeta = null; })
    .finally(() => {
      hgImg.onload = () => {
        hgImgLoaded = true;
        hgCanvas.width = hgImg.naturalWidth;
        hgCanvas.height = hgImg.naturalHeight;
        const at = hgMeta ? new Date((hgMeta.ts || '').replace(/(\\d{8})_(\\d{6})/, '$1 $2')) : null;
        hgCanvasNote.innerHTML = '<span class="qtitle" style="color:var(--green)">기준영상 로드됨</span> ' +
          hgImg.naturalWidth + '×' + hgImg.naturalHeight +
          (hgMeta && hgMeta.H ? ' · H 있음(월드 라벨 표시)' : ' · H 없음(캘리브 전)');
        if (rawCanvasNote) rawCanvasNote.innerHTML =
          '<span class="qtitle" style="color:var(--green)">기준영상 로드됨</span> ' +
          hgImg.naturalWidth + '×' + hgImg.naturalHeight + ' · raw/보정 코너 오버레이';
        redrawHg();
        renderHgValidation();
        redrawRawCanvas();
      };
      hgImg.onerror = () => {
        hgCanvasNote.innerHTML = '<span class="none">기준영상 없음 — 스냅샷을 먼저 받으세요</span>';
        if (rawCanvasNote) rawCanvasNote.innerHTML = '<span class="none">기준영상 없음 — 스냅샷을 받으면 배경이 깔립니다 (없어도 좌표는 표시)</span>';
      };
      hgImg.src = '/hg_reference.jpg?_=' + Date.now();
    });
}
function toggleHgOverlay() {
  hgOverlayOn = !hgOverlayOn;
  const b = document.getElementById('hgOverlayBtn');
  b.textContent = hgOverlayOn ? '오버레이 정지' : '오버레이 보기 시작';
  b.classList.toggle('on', hgOverlayOn);
  if (hgOverlayOn && !hgImgLoaded) loadHgReference();
  redrawHg();
}
// H(3×3, 픽셀→mm) 적용
function applyH(H, x, y) {
  const d = H[6] * x + H[7] * y + H[8];
  return [(H[0] * x + H[1] * y + H[2]) / d, (H[3] * x + H[4] * y + H[5]) / d];
}
// Independent validation: these known markers are deliberately excluded from
// H estimation. Compare only their live pixel centers transformed by H with
// their surveyed world coordinates supplied in the snapshot metadata.
function renderHgValidation() {
  const box = document.getElementById('hgValidation');
  if (!box) return;
  if (!hgMeta || !hgMeta.H || !hgMeta.validation || !hgMeta.validation.length) {
    box.innerHTML = '<span class="none">기준영상 스냅샷 및 H 캘리브레이션 후 검증 마커를 비추세요</span>';
    return;
  }
  let rows = '', sumSq = 0, maxErr = 0, visible = 0;
  for (const v of hgMeta.validation) {
    const c = rawFrame[String(v.id)];
    if (!c) continue;
    const p = markerCenter(c);
    const w = applyH(hgMeta.H, p[0], p[1]);
    const dx = w[0] - Number(v.wx), dy = w[1] - Number(v.wy);
    const err = Math.hypot(dx, dy);
    sumSq += err * err; maxErr = Math.max(maxErr, err); ++visible;
    const signed = n => (n >= 0 ? '+' : '') + n.toFixed(1);
    rows += `<tr><td>${v.id}</td><td>${w[0].toFixed(1)}, ${w[1].toFixed(1)}</td>` +
            `<td>${Number(v.wx).toFixed(1)}, ${Number(v.wy).toFixed(1)}</td>` +
            `<td>${signed(dx)}, ${signed(dy)}</td><td>${err.toFixed(1)}</td></tr>`;
  }
  if (!visible) {
    box.innerHTML = '<span class="none">검증 마커(id 20~23)가 현재 프레임에 없습니다</span>';
    return;
  }
  const rmse = Math.sqrt(sumSq / visible);
  box.innerHTML = '<div class="qtitle">독립 검증 결과 (mm)</div><table>' +
    '<tr><td>id</td><td>계산 X, Y</td><td>실측 X, Y</td><td>ΔX, ΔY</td><td>위치 오차</td></tr>' +
    rows + '</table><div class="qtitle" style="margin-top:8px">' +
    `보이는 마커 ${visible}/${hgMeta.validation.length} · RMSE ${rmse.toFixed(1)} mm · 최대 오차 ${maxErr.toFixed(1)} mm</div>`;
}
// 점이 anchor 폴리곤 내부인지 (ray casting)
function pointInPoly(px, py, poly) {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const xi = poly[i][0], yi = poly[i][1], xj = poly[j][0], yj = poly[j][1];
    if (((yi > py) !== (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi) + xi)) inside = !inside;
  }
  return inside;
}
function markerCenter(c) {
  let x = 0, y = 0;
  for (let i = 0; i < 4; i++) { x += Number(c[i][0]); y += Number(c[i][1]); }
  return [x / 4, y / 4];
}
function redrawHg() {
  if (!hgCtx) return;
  hgCtx.clearRect(0, 0, hgCanvas.width, hgCanvas.height);
  if (hgImgLoaded) hgCtx.drawImage(hgImg, 0, 0);
  if (!hgOverlayOn) return;
  const anchorIds = {};
  let poly = null;
  // anchor 작업영역 폴리곤 + 라벨
  if (hgMeta && hgMeta.anchors && hgMeta.anchors.length) {
    const seen = hgMeta.anchors.filter(a => a.seen);
    if (seen.length >= 3) {
      poly = seen.map(a => [a.px, a.py]);
      hgCtx.beginPath();
      poly.forEach((p, i) => i ? hgCtx.lineTo(p[0], p[1]) : hgCtx.moveTo(p[0], p[1]));
      hgCtx.closePath();
      hgCtx.strokeStyle = '#3b82f6'; hgCtx.lineWidth = 3; hgCtx.stroke();
      hgCtx.fillStyle = 'rgba(59,130,246,0.12)'; hgCtx.fill();
    }
    hgCtx.font = '20px sans-serif';
    for (const a of hgMeta.anchors) {
      anchorIds[a.id] = true;
      if (!a.seen) continue;
      hgCtx.fillStyle = '#3b82f6';
      hgCtx.beginPath(); hgCtx.arc(a.px, a.py, 6, 0, 7); hgCtx.fill();
      hgCtx.fillText('a' + a.id + ' (' + a.wx + ',' + a.wy + ')', a.px + 9, a.py - 9);
    }
  }
  // 실시간 마커 (rawFrame — CAM_POSE로 갱신). anchor id는 위에서 이미 그렸으니 제외.
  hgCtx.font = '18px sans-serif';
  for (const id in rawFrame) {
    if (anchorIds[id]) continue;
    const c = rawFrame[id];
    const [cx, cy] = markerCenter(c);
    const inside = poly ? pointInPoly(cx, cy, poly) : true;
    const col = inside ? '#22c55e' : '#ef4444';
    hgCtx.strokeStyle = col; hgCtx.lineWidth = 3;
    hgCtx.beginPath();
    for (let i = 0; i < 4; i++) {
      const p = c[i];
      i ? hgCtx.lineTo(Number(p[0]), Number(p[1])) : hgCtx.moveTo(Number(p[0]), Number(p[1]));
    }
    hgCtx.closePath(); hgCtx.stroke();
    hgCtx.fillStyle = col;
    hgCtx.beginPath(); hgCtx.arc(cx, cy, 5, 0, 7); hgCtx.fill();
    let label = 'id ' + id;
    if (hgMeta && hgMeta.H) {
      const w = applyH(hgMeta.H, cx, cy);
      label += ' (' + w[0].toFixed(0) + ',' + w[1].toFixed(0) + 'mm)';
    }
    hgCtx.fillText(label, cx + 9, cy - 9);
  }
}

// ===== 마커 검출 탭 캔버스 오버레이 (기준영상 + raw/보정 코너) =====
// 호모그래피 탭과 같은 기준영상(hgImg)을 공유한다. raw 코너(주황)를 그리고,
// K가 로드돼 있고 "보정 좌표 함께 표시"가 켜져 있으면 undistort한 보정 코너(청록)와
// raw→보정 이동 점선을 겹쳐 그려 왜곡 보정량을 시각화한다.
let rawOverlayOn = false;
const rawCanvas = document.getElementById('rawCanvas');
const rawCtx = rawCanvas ? rawCanvas.getContext('2d') : null;
const rawCanvasNote = document.getElementById('rawCanvasNote');

function rawSnapshot() {
  send('HG_SNAPSHOT');   // 호모그래피 탭과 동일한 기준영상을 요청(공유)
  if (rawCanvasNote) rawCanvasNote.innerHTML = '<span class="qtitle">기준영상 요청 중…</span> 잠시 후 배경이 갱신됩니다';
  setTimeout(loadHgReference, 900);
}
function toggleRawOverlay() {
  rawOverlayOn = !rawOverlayOn;
  const b = document.getElementById('rawOverlayBtn');
  b.textContent = rawOverlayOn ? '오버레이 정지' : '오버레이 보기 시작';
  b.classList.toggle('on', rawOverlayOn);
  if (rawOverlayOn && !hgImgLoaded) loadHgReference();
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
function redrawRawCanvas() {
  if (!rawCtx) return;
  // 캔버스 해상도: 배경이 있으면 그 크기, 없으면 K(cx*2,cy*2), 그것도 없으면 1920×1080.
  // raw 코너는 풀프레임 픽셀 좌표라 배경 JPEG와 같은 좌표계에서 겹쳐진다.
  let W, H;
  if (hgImgLoaded) { W = hgImg.naturalWidth; H = hgImg.naturalHeight; }
  else if (kCalib) { W = Math.round(kCalib.cx * 2); H = Math.round(kCalib.cy * 2); }
  else { W = 1920; H = 1080; }
  if (rawCanvas.width !== W || rawCanvas.height !== H) { rawCanvas.width = W; rawCanvas.height = H; }
  rawCtx.clearRect(0, 0, W, H);
  if (hgImgLoaded) rawCtx.drawImage(hgImg, 0, 0, W, H);
  if (!rawOverlayOn) return;
  const cmp = showUndist && kCalib;   // 보정 코너를 함께 그릴지
  rawCtx.font = '18px sans-serif';
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
}

const es = new EventSource('/events');
es.onopen = () => { send('CALIB_K_STATUS'); send('CALIB_K_QUERY'); send('CALIB_K_PROFILE_LIST'); };
es.onmessage = (e) => {
  handleRaw(e.data);
  handleLatency(e.data);
  handleHgWorld(e.data);
  handleHgStatus(e.data);
  handleHgAnchors(e.data);
  handleHgValidationConfig(e.data);
  handleHgCharuco(e.data);
  handleHgMatrix(e.data);
  handleHgPersistence(e.data);
  handleHgCoordMode(e.data);
  handleKProfiles(e.data);
  handleShell(e.data);
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
</html>"""


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
