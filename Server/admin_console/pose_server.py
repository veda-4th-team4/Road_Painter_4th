#!/usr/bin/env python3
"""IF-TCP-003 pose receiver (test harness for the vision server).

Run on the RPi:  python3 pose_server.py [port]

- Accepts the camera's persistent TCP connection, splits the stream on
  newlines, parses each JSON line, prints a one-line summary with latency.
- Type a command + Enter to push it to the camera over the same link:
      CALIB_START    -> camera computes pixel->world homography from the
                        8 calculation anchors (ids 10..17; validation ids
                        20..23 are excluded, see homography_mapper.cpp)
      CALIB_K_START  -> begin an intrinsics (K, dist) calibration session
      CALIB_K_CAPTURE-> store the CURRENT frame as one calibration view (hold
                        the ChArUco board at a chosen pose, then send this).
                        Vary tilt/distance/position across captures.
      CALIB_K_COMPUTE-> run the calibration on the quality-approved views
                        (explicit only — capture never auto-computes)
      CALIB_K_SET <views> <rms>
                     -> change the target view count and the RMS pass limit
                        at runtime (0 keeps a value). e.g. "CALIB_K_SET 25 0.6"
      CALIB_K_UNDO   -> drop the most recently accepted view
      CALIB_K_STATUS -> report board config and accepted-view count
      CALIB_K_QUERY  -> read back whatever K/dist is CURRENTLY loaded on the
                        camera (no new capture/compute) — answers immediately
      GET_RAW_RES    -> ask the SDK which video resolutions it supports, to
                        check whether raw (rawVideo) capture can do 4K
                        (untested API call — first use, see code comment)
      LDC_CHECK_START-> hold a ChArUco board to the camera: it streams, live,
                        the detected marker/corner counts and the RESIDUAL lens
                        distortion (how straight the board's corner rows/cols
                        stay in the already-corrected frames). LDC_CHECK_STOP
                        ends it. Judges whether the built-in LDC is enough.
      LDC_SNAPSHOT   -> one-shot: capture the current frame's LDC metrics AND
                        the image (with marker/corner overlay drawn), and save
                        both here. Works whether or not LDC_CHECK is running.
                        Arrives on a SEPARATE connection/port (SNAPSHOT_PORT)
                        from the realtime pose stream, and as a .ppm (raw
                        image, no codec needed) since the camera build has no
                        JPEG encoder — convert with any image tool if desired,
                        e.g.: convert snapshot.ppm snapshot.jpg
- Watchdog: if no packet arrives within WATCHDOG_S the camera is reported
  lost (the production server would stop the robot here).
"""

import json
import os
import socket
import struct
import sys
import threading
import time

HOST = "0.0.0.0"
PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 6000
SNAPSHOT_PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 6001
WATCHDOG_S = 2.0  # no packet for this long -> treat camera as lost
SNAPSHOT_DIR = "."
# Uploaded calibration-view JPEGs (CALIB_K_UPLOAD) go in their own subfolder.
CALIB_VIEW_DIR = f"{SNAPSHOT_DIR}/calib_views"
LDC_CHECK_LOG_INTERVAL_S = 1.0  # throttle: at most one CSV row per this many seconds
LDC_CHECK_LOG_PATH = f"{SNAPSHOT_DIR}/ldc_check_log.csv"

current_conn = None  # the live camera connection (stdin thread sends to it)
_ldc_log_last_write = 0.0


def ldc_check_log_reset():
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


def stdin_sender():
    """Forward operator-typed lines (e.g. CALIB_START) to the camera."""
    for line in sys.stdin:
        cmd = line.strip()
        if not cmd:
            continue
        conn = current_conn
        if conn is None:
            print("[!] no camera connected; command dropped")
            continue
        try:
            conn.sendall((cmd + "\n").encode())
            print(f"[>] sent: {cmd}")
        except OSError as e:
            print(f"[!] send failed: {e}")


def print_msg(msg, last_seq):
    now_ms = time.time() * 1000
    latency = now_ms - msg.get("t", now_ms)
    mtype = msg.get("type")

    if mtype == "CALIB_ACK":
        print(f"[calib] camera acknowledged, collecting anchors...")
        return last_seq
    if mtype == "CALIB_RESULT":
        if msg.get("ok"):
            print(f"[calib] SUCCESS (frames={msg.get('frames')}) — "
                  f"camera now streams world coords")
        else:
            print(f"[calib] FAILED: {msg.get('reason')}")
        return last_seq
    if mtype == "CALIB_K_ACK":
        print(f"[calib-K] session started target={msg.get('target')} "
              f"board={msg.get('squares_x')}x{msg.get('squares_y')} "
              f"square={msg.get('square_mm')}mm marker={msg.get('marker_mm')}mm "
              f"dict={msg.get('dictionary')}")
        return last_seq
    if mtype == "CALIB_K_PROGRESS":
        if msg.get("rejected"):
            print(f"[calib-K] capture REJECTED — {msg.get('reason')} "
                  f"corners={msg.get('corners')}/{msg.get('corners_total')} "
                  f"coverage={100*msg.get('coverage',0):.1f}% "
                  f"sharpness={msg.get('sharpness',0):.1f} "
                  f"move={msg.get('move_px',-1):.1f}px views={msg.get('views')}")
        else:
            print(f"[calib-K] captured view {msg.get('views')}/{msg.get('target')} "
                  f"({msg.get('corners')}/{msg.get('corners_total')} corners, "
                  f"coverage={100*msg.get('coverage',0):.1f}%, "
                  f"sharpness={msg.get('sharpness',0):.1f}, "
                  f"move={msg.get('move_px',-1):.1f}px)"
                  + (" — READY TO COMPUTE" if msg.get("ready") else ""))
        return last_seq
    if mtype == "CALIB_K_COMPUTING":
        print(f"[calib-K] COMPUTING with {msg.get('views')} accepted views...")
        return last_seq
    if mtype == "CALIB_K_GATE":
        print(f"[calib-K] quality gates {'ON' if msg.get('enabled') else 'OFF'}")
        return last_seq
    if mtype == "CALIB_K_UPLOAD":
        stage = msg.get("stage")
        if stage == "start":
            print(f"[calib-K] upload start — {msg.get('total')} stored views")
        elif stage == "progress":
            print(f"[calib-K] upload {msg.get('sent')}/{msg.get('total')} "
                  f"— view {msg.get('view')} ({msg.get('bytes')} bytes)")
        elif stage == "error":
            print(f"[calib-K] upload error — view {msg.get('view')}")
        elif stage == "busy":
            print("[calib-K] upload already in progress")
        elif stage == "done":
            print(f"[calib-K] upload done — {msg.get('sent')}/{msg.get('total')} saved")
        return last_seq
    if mtype == "CALIB_K_RESULT":
        if msg.get("ok"):
            print(f"[calib-K] SUCCESS rms={msg.get('rms')}px "
                  f"views={msg.get('views')} pruned={msg.get('pruned')} "
                  f"fx={msg.get('fx')} fy={msg.get('fy')} "
                  f"cx={msg.get('cx')} cy={msg.get('cy')} "
                  f"dist={msg.get('dist')}")
            # A real vision server would persist these into
            # camera_intrinsics.yaml here for its own undistortPoints.
        else:
            print(f"[calib-K] FAILED: {msg.get('reason')}")
        return last_seq
    if mtype in ("CALIB_K_CONFIG", "CALIB_K_STATUS", "CALIB_K_UNDO"):
        if not msg.get("ok"):
            print(f"[calib-K] {mtype} FAILED: {msg.get('reason')}")
        else:
            print(f"[calib-K] BOARD_CONFIG views={msg.get('views')}/{msg.get('target')} "
                  f"squares={msg.get('squares_x')}x{msg.get('squares_y')} "
                  f"square={msg.get('square_mm')} marker={msg.get('marker_mm')} "
                  f"dict={msg.get('dictionary')} margin={msg.get('margin_x_mm')}/"
                  f"{msg.get('margin_y_mm')} quiet={msg.get('quiet_mm')} "
                  f"board={msg.get('board_w_mm')}x{msg.get('board_h_mm')}")
        return last_seq
    if mtype == "CALIB_K_QUERY":
        if msg.get("available"):
            print(f"[calib-K] CURRENT VALUES: fx={msg.get('fx')} fy={msg.get('fy')} "
                  f"cx={msg.get('cx')} cy={msg.get('cy')} dist={msg.get('dist')}")
        else:
            print("[calib-K] no calibration loaded on the camera right now")
        return last_seq
    if mtype == "RAW_RES":
        lst = msg.get("list", [])
        if lst:
            resstr = ", ".join(f"{w}x{h}" for w, h in lst)
            print(f"[raw-res] SDK reports {msg.get('count')} supported "
                  f"resolution(s) (err={msg.get('err')}): {resstr}")
        else:
            print(f"[raw-res] no resolutions reported (err={msg.get('err')}, "
                  f"count={msg.get('count')}) — API may not behave as documented")
        return last_seq
    if mtype == "LDC_CHECK_ACK":
        state = msg.get("state")
        if state == "checking":
            ldc_check_log_reset()
            print("[ldc] camera acknowledged — hold the ChArUco board in view, "
                  "move it to the frame EDGES/CORNERS. Streams continuously, logging "
                  f"one row per ~{LDC_CHECK_LOG_INTERVAL_S:.0f}s to {LDC_CHECK_LOG_PATH}. "
                  "Type LDC_CHECK_STOP to end.")
        elif state == "stopped":
            print("\n[ldc] stopped — normal pose streaming resumed")
        else:
            print(f"\n[ldc] start REJECTED: {msg.get('reason')}")
        return last_seq
    if mtype == "LDC_CHECK":
        mk = msg.get("markers", 0)
        mkt = msg.get("markers_total", 0)
        if not msg.get("found"):
            print(f"\r[ldc] board partial: markers={mk}/{mkt} "
                  f"corners={msg.get('corners',0)} — show more of the board   ",
                  end="", flush=True)
            return last_seq
        rms = msg.get("straight_rms_px", 0.0)
        smax = msg.get("straight_max_px", 0.0)
        emax = msg.get("edge_max_px", 0.0)
        cmax = msg.get("center_max_px", 0.0)
        cor = msg.get("corners", 0)
        cort = msg.get("corners_total", 0)
        # Judge on the WORST deviation actually seen. edge_max==0 means no
        # corner reached the frame edge this frame, so we have only sampled the
        # center — where distortion is smallest. Say so instead of trusting it.
        worst = smax
        if worst < 0.5:
            verdict = "EXCELLENT (residual distortion negligible)"
        elif worst < 1.5:
            verdict = "OK (mild residual distortion)"
        else:
            verdict = "POOR (significant residual distortion)"
        if emax <= 0.0:
            verdict += " [CENTER ONLY — move board to the frame EDGES/CORNERS]"

        u = msg.get("undistorted")
        ldc_check_log_row(msg, u)
        if u:
            # Before/after: same corners, with vs without OpenCV undistort
            # applied on top of whatever the camera's own LDC already did.
            print(f"\n[ldc] BEFORE (camera LDC only): rms={rms:.2f}px "
                  f"edge_max={emax:.2f}px center_max={cmax:.2f}px")
            print(f"[ldc] AFTER  (+ OpenCV undistort): rms={u['straight_rms_px']:.2f}px "
                  f"edge_max={u['edge_max_px']:.2f}px "
                  f"center_max={u['center_max_px']:.2f}px "
                  f"(rms improved {rms - u['straight_rms_px']:+.2f}px)")
        else:
            print(f"\r[ldc] markers={mk}/{mkt} corners={cor}/{cort} "
                  f"rms={rms:.2f}px max={smax:.2f}px "
                  f"edge_max={emax:.2f}px center_max={cmax:.2f}px "
                  f"-> {verdict}   ", end="", flush=True)
        return last_seq

    # CAM_POSE
    seq = msg.get("seq")
    gap = ""
    # seq is a FRAME counter: several markers in one frame share it, so the
    # same seq (another marker, same frame) and seq+1 (next frame) are both
    # normal. Only a jump of >1 means a frame was actually dropped.
    if (last_seq is not None and seq is not None
            and seq != last_seq and seq != last_seq + 1):
        gap = f"  (seq gap: {last_seq} -> {seq})"

    if msg.get("confidence", 0) > 0:
        corners = msg.get("corners", [])
        ctxt = " ".join(f"c{i}=({c['x']:.2f},{c['y']:.2f})"
                        for i, c in enumerate(corners))
        world = msg.get("world")
        wtxt = (f" world=({world['x']:.0f},{world['y']:.0f}mm,"
                f"{world['theta']:.1f}deg)") if world else ""
        print(f"seq={seq} id={msg.get('id')} "
              f"{ctxt}{wtxt} "
              f"latency={latency:.0f}ms{gap}")
    else:
        print(f"seq={seq} MARKER LOST (heartbeat) latency={latency:.0f}ms{gap}")
    return seq


def handle_client(conn, addr):
    global current_conn
    print(f"[+] camera connected: {addr}")
    current_conn = conn
    conn.settimeout(WATCHDOG_S)
    buf = b""
    last_seq = None
    # Collapse repeated timeouts into one "started" line + one "resumed after
    # Ns" line, instead of flooding the terminal every WATCHDOG_S while
    # waiting (e.g. during a CALIB_K_START session, where pose streaming is
    # intentionally paused for minutes — that is expected, not an error).
    watchdog_streak = 0
    try:
        while True:
            try:
                chunk = conn.recv(4096)
            except socket.timeout:
                watchdog_streak += 1
                if watchdog_streak == 1:
                    print(f"[!] WATCHDOG: no packet for {WATCHDOG_S}s -> STOP ROBOT "
                          f"(further repeats suppressed until packets resume)")
                continue
            except OSError as e:
                # e.g. ConnectionResetError when the camera app restarts and
                # drops the link abruptly. Must not propagate: an uncaught
                # exception here would kill the accept loop in main(), taking
                # the whole listening port down with it.
                print(f"[-] camera connection error: {e}")
                return
            if not chunk:
                print("[-] camera disconnected")
                return
            if watchdog_streak > 0:
                print(f"[+] packets resumed after ~{watchdog_streak * WATCHDOG_S:.0f}s gap")
                watchdog_streak = 0
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if not line.strip():
                    continue
                try:
                    msg = json.loads(line)
                except json.JSONDecodeError as e:
                    print(f"[!] bad JSON ({e}): {line[:80]!r}")
                    continue
                last_seq = print_msg(msg, last_seq)
    finally:
        current_conn = None


def recv_exact(conn, n):
    """Read exactly n bytes (recv() may return fewer than requested)."""
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("connection closed mid-snapshot")
        buf += chunk
    return buf


def handle_snapshot_client(conn, addr):
    """One LDC_SNAPSHOT upload per connection: [json][w][h][pixel_len][RGB]."""
    try:
        (json_len,) = struct.unpack(">I", recv_exact(conn, 4))
        msg = json.loads(recv_exact(conn, json_len))
        width, height, pixel_len = struct.unpack(">III", recv_exact(conn, 12))
        is_jpeg = msg.get("format") == "jpeg"
        if not is_jpeg and pixel_len != width * height * 3:
            print(f"[!] snapshot from {addr}: size mismatch "
                  f"({width}x{height} implies {width*height*3}, got {pixel_len})")
            return
        pixels = recv_exact(conn, pixel_len)
    except (ConnectionError, OSError, struct.error, json.JSONDecodeError) as e:
        print(f"[!] snapshot from {addr} failed: {e}")
        return

    ts = time.strftime("%Y%m%d_%H%M%S")
    mtype = msg.get("type")

    # Pre-compressed calibration view (CALIB_K_UPLOAD): write JPEG verbatim.
    if is_jpeg:
        view = msg.get("view")
        os.makedirs(CALIB_VIEW_DIR, exist_ok=True)
        jpg_path = (f"{CALIB_VIEW_DIR}/calib_view_{view:02d}_{ts}.jpg"
                    if isinstance(view, int) else f"{CALIB_VIEW_DIR}/snapshot_{ts}.jpg")
        with open(jpg_path, "wb") as f:
            f.write(pixels)
        print(f"[calib-K] view {view}/{msg.get('target')} JPEG saved "
              f"{jpg_path} ({msg.get('corners')} corners, {len(pixels)} bytes)")
        return

    if mtype == "CALIB_K_VIEW":
        path = f"{SNAPSHOT_DIR}/calib_view_{msg.get('view'):02d}_{ts}.ppm"
    else:
        path = f"{SNAPSHOT_DIR}/ldc_snapshot_{ts}.ppm"
    with open(path, "wb") as f:
        f.write(f"P6\n{width} {height}\n255\n".encode())
        f.write(pixels)

    if mtype == "CALIB_K_VIEW":
        print(f"[calib-K] view {msg.get('view')}/{msg.get('target')} image saved "
              f"{path} ({msg.get('corners')} corners)")
        return

    u = msg.get("undistorted")
    if msg.get("found"):
        print(f"\n[snapshot] saved {path} — markers={msg.get('markers')}/"
              f"{msg.get('markers_total')} corners={msg.get('corners')}/"
              f"{msg.get('corners_total')}")
        print(f"[snapshot] BEFORE (camera LDC only): "
              f"rms={msg.get('straight_rms_px', 0):.2f}px "
              f"edge_max={msg.get('edge_max_px', 0):.2f}px "
              f"center_max={msg.get('center_max_px', 0):.2f}px")
        if u:
            print(f"[snapshot] AFTER  (+ OpenCV undistort): "
                  f"rms={u['straight_rms_px']:.2f}px "
                  f"edge_max={u['edge_max_px']:.2f}px "
                  f"center_max={u['center_max_px']:.2f}px "
                  f"(rms improved {msg.get('straight_rms_px', 0) - u['straight_rms_px']:+.2f}px)")
    else:
        print(f"\n[snapshot] saved {path} — board not fully seen "
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
        f.write(f"{ts},{path},{msg.get('markers')},{msg.get('markers_total')},"
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
    srv.bind((HOST, SNAPSHOT_PORT))
    srv.listen(1)
    print(f"listening for LDC_SNAPSHOT uploads on {HOST}:{SNAPSHOT_PORT} ...")
    while True:
        conn, addr = srv.accept()
        with conn:
            handle_snapshot_client(conn, addr)


def main():
    threading.Thread(target=stdin_sender, daemon=True).start()
    threading.Thread(target=snapshot_server, daemon=True).start()
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(1)
    print(f"listening on {HOST}:{PORT} ... (type CALIB_START + Enter to calibrate)")
    while True:
        conn, addr = srv.accept()
        try:
            with conn:
                handle_client(conn, addr)
        except Exception as e:
            # Second line of defense: whatever goes wrong with one connection
            # must not kill this loop, or the listening port silently vanishes
            # and every later reconnect attempt fails.
            print(f"[!] main: unexpected error, still listening: {e}")


if __name__ == "__main__":
    main()
