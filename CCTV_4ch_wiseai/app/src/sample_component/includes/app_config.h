#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stddef.h>
#include <string.h>

/**
 * Copy `src` into `dst` without ever splitting a UTF-8 character.
 *
 * Lives in this header, which is otherwise only knobs, because it is needed by
 * three translation units and adding a header of its own for eight lines costs
 * more than it saves — a new source file is also the one change this project's
 * build reliably fails on the first attempt (vboxsf + CMake, see build_install).
 *
 * snprintf("%s") truncates on a BYTE boundary. Every operator-facing message
 * here is Korean, so a byte boundary has a two-in-three chance of landing
 * inside a character, and these buffers are copied verbatim into /status. Half
 * a character there is not cosmetic: the response stops being valid UTF-8,
 * JSON.parse throws, and the dashboard loses EVERY field rather than one — the
 * camera looks dead from a page that cannot say why.
 *
 * Measured 2026-08-05: two messages already overflowed their 128-byte buffer
 * (142 B and 193 B). Both happened to cut on ASCII, which is luck, not design;
 * editing one word inside either moves the cut.
 *
 * UTF-8 continuation bytes are 10xxxxxx. If the first byte we are NOT copying
 * is one, the character it belongs to began earlier and would be left half
 * written, so walk back to its lead byte and drop the whole thing.
 */
static inline void CopyUtf8(char* dst, size_t dst_size, const char* src) {
  if (dst == NULL || dst_size == 0) return;
  if (src == NULL) {
    dst[0] = '\0';
    return;
  }
  size_t n = 0;
  while (src[n] != '\0' && n + 1 < dst_size) ++n;
  // Only when the string actually got cut. A string that ended on its own is
  // complete by definition, and walking back from its terminator would eat a
  // character that fitted.
  if (src[n] != '\0') {
    while (n > 0 && ((unsigned char)src[n] & 0xC0) == 0x80) --n;
  }
  memcpy(dst, src, n);
  dst[n] = '\0';
}

/**
 * Config for ArucoPosePNM, ported knob-by-knob from cctv_app/inc/app_config.h
 * as each feature was added rather than copied wholesale up front.
 *
 * No longer "marker-detection-only": calibration, homography and the
 * central-TLS link below have all since been ported in.
 */

// ---------------------------------------------------------------------------
// Vision server (RPi dashboard)
// ---------------------------------------------------------------------------
// The 8082 dashboard instance (cctv_calibration_manager) was moved off the
// default 7000/7001 pair to avoid colliding with the 8083 production instance,
// which owns 7000. Pose goes to 7100, snapshots would go to 7101.
//   dashboard: http://192.168.0.8:8082/
// Wired (192.168.0.2), not the Pi's Wi-Fi address (192.168.0.8) — pi.env.example
// warns .8 is DHCP and can move on a Pi reboot; .2 is the stable interface for
// a link this app expects to stay up continuously.
#ifndef POSE_SERVER_IP
#define POSE_SERVER_IP    "192.168.0.2"
#endif

#ifndef POSE_SERVER_PORT
#define POSE_SERVER_PORT  7100
#endif

// Rate limit for reconnect attempts after the server drops (ms).
#ifndef POSE_RECONNECT_MS
#define POSE_RECONNECT_MS 2000
#endif

// Upper bound for one newline-delimited command line read back from the
// server. Named after the original cctv_app constant so pose_sender.cc
// compiles unchanged.
#ifndef CENTRAL_TLS_MAX_LINE
#define CENTRAL_TLS_MAX_LINE 2048
#endif

// ---------------------------------------------------------------------------
// Central server (TLS, role=CCTV) — ported from cctv_app/inc/app_config.h.
// Independent of the vision-server link above: that one keeps serving the
// RPi dashboard (CAM_POSE, calibration commands) on POSE_SERVER_PORT/7100.
// ---------------------------------------------------------------------------
#ifndef ENABLE_CENTRAL_TLS_STREAM
#define ENABLE_CENTRAL_TLS_STREAM 1
#endif

// Same RPi as POSE_SERVER_IP above, wired address (192.168.0.2) for the same
// DHCP-stability reason, tried first. CENTRAL_TLS_SERVER_IP_FALLBACK below is
// the Pi's Wi-Fi address (192.168.0.8) — wired first, wireless as a backup
// path, not the other way round; see central_tls_sender_set_fallback().
//
// CAUTION -- cert/IP mismatch, not a reachability problem: CENTRAL_TLS_CA_FILE's
// current cert (copied from cctv_app, see below) has 127.0.0.1 and 192.168.0.8
// in its Subject Alternative Name, NOT 192.168.0.2. central_tls_sender now
// pins IP verification per-connection-attempt, so .2 will complete the TCP
// connect but fail TLS verification EVERY time until the cert is reissued
// with .2 in its SAN — this is expected to make every reconnect cycle fail
// once on .2 before rotating to .8, not a bug to chase. Port 9000 was not
// accepting connections on either address when this was checked (2026-08-10)
// — no central-server process was listening at all; the link sits in
// "offline"/rotates candidates and keeps retrying until one comes up.
#ifndef CENTRAL_TLS_SERVER_IP
#define CENTRAL_TLS_SERVER_IP "192.168.0.2"
#endif
#ifndef CENTRAL_TLS_SERVER_IP_FALLBACK
#define CENTRAL_TLS_SERVER_IP_FALLBACK "192.168.0.8"
#endif
#ifndef CENTRAL_TLS_SERVER_PORT
#define CENTRAL_TLS_SERVER_PORT 9000
#endif

// Where the camera unpacks this .cap. Getting this wrong is silent —
// central_tls_sender_init() fails to load the PEM, clears have_addr, and
// never even opens a socket (confirmed empty CENTRAL_TLS_CA_FILE symptom:
// central_tls_sender_active_ip() reports "" forever, link stays "offline"
// even with a real central server up).
//
// NOT "/mnt/opensdk/apps/ArucoPosePNM" (the cctv_app value this was copied
// from) — verified wrong on 2026-08-10 via /status's own
// calib.persist.cwd, which this app already reports for the SAME reason
// (PERSIST_DIR getting this wrong is equally silent): the running process's
// cwd is ".../ArucoPosePNM/app/bin", one level deeper than cctv_app's
// layout, because this project's CMakeLists.txt puts everything under an
// app/ subdirectory (CMAKE_HOME_DIRECTORY) that cctv_app's Makefile-based
// build does not have.
#ifndef APP_DIR
#define APP_DIR "/mnt/opensdk/apps/ArucoPosePNM/app"
#endif

// PEM containing the central server certificate/CA. The TLS client fails
// closed when this file is missing or invalid. NOT app_skel.crt (that one is
// the OpenSDK app-signing skeleton, unrelated to this link).
#ifndef CENTRAL_TLS_CA_FILE
#define CENTRAL_TLS_CA_FILE APP_DIR "/res/cert/central_server.crt"
#endif

// Marker id streamed to the central server as POS. The server's POS schema
// carries no marker id — every POS it receives is taken as THE robot — so
// anchors and validation markers must be filtered on this side. Retargetable
// live with CENTRAL_ID over the RPi dashboard channel, no rebuild needed.
#ifndef ROBOT_MARKER_ID
#define ROBOT_MARKER_ID 49
#endif

// Give up on a central-server TCP connect / TLS handshake after this long and
// start over (ms). connect()/SSL_connect() are non-blocking and carry no
// deadline of their own.
#ifndef CENTRAL_TLS_HANDSHAKE_MS
#define CENTRAL_TLS_HANDSHAKE_MS 5000
#endif

// ---------------------------------------------------------------------------
// Dynamic ROI (marker tracking)
// ---------------------------------------------------------------------------
// Upper bound on how many marker ids "DYNROI_IDS" may pin the tracker to.
#ifndef DYNROI_MAX_TRACK_IDS
#define DYNROI_MAX_TRACK_IDS 16u
#endif

// ---------------------------------------------------------------------------
// On-camera debug dashboard (app/html/index.html + /status, /shell)
// ---------------------------------------------------------------------------
// Shown by /status so "which build is actually on the camera right now" has an
// answer. Bump by hand on anything worth telling apart; the build timestamp
// next to it comes from __DATE__/__TIME__ and moves on its own.
//
// That timestamp is in UTC, because the compiler runs inside the SDK docker
// image and the image is UTC while the host is KST. So a build made at 09:01
// local reports 00:01 — nine hours EARLIER, which reads exactly like a stale
// install that failed to take. Compare it against `date -u`, not `date`.
// (2026-08-06: this cost a round of stop/install/start debugging chasing an
// install that had in fact worked the first time.)
#ifndef APP_VERSION
#define APP_VERSION "0.3.0"
#endif

// The GET /status endpoint and the per-frame bookkeeping it reads. Set to 0 to
// remove BOTH — the handler and the few stores in the raw-video path — leaving
// detection byte-for-byte as it was before the dashboard existed.
//
// It is on by default because the cost of leaving it on is close to zero: the
// page polls only when someone presses [Refresh], so with nobody watching, no
// dashboard code runs at all. The only always-on part is four integer stores
// per frame next to a detect() call that takes 5..9 ms.
//
// There is deliberately no runtime toggle. A runtime flag would have to be
// checked on the hot path, which costs more than the stores it would skip.
#ifndef ENABLE_STATUS_PAGE
#define ENABLE_STATUS_PAGE 1
#endif

// ---------------------------------------------------------------------------
// Remote shell (pose link)
// ---------------------------------------------------------------------------
// "SHELL <cmd>" server command: run <cmd> through /bin/sh on the camera and
// stream its stdout+stderr back to the dashboard. The camera has no SSH
// server, so this is the practical way to answer questions like "is /mnt
// actually full, or read-only, or just the wrong path?" without a serial
// console session. Ported from cctv_app (2026-08-10) with its warning intact,
// unchanged, because the warning is still exactly true here:
//
// !! THIS IS A REMOTE SHELL WITH NO AUTHENTICATION !!
// Anyone who can reach the pose port can run arbitrary commands as this app.
// cctv_app's own comment called this lab/commissioning-only and said the
// production app must never ship it — and unlike cctv_app on a bench, this
// camera is wired into the Road_Painter_4th central server and robot-control
// network right now, over a pose link (IF-TCP-003) that has no login of its
// own. Ported anyway on 2026-08-10 at the operator's explicit, risk-
// acknowledged request, after that context was laid out. If that operational
// picture changes, set this back to 0 first and ask questions after.
//
// Also runs on the frame thread: a slow command (sleep, a huge cat) stalls
// detection for its duration, the same trade-off already accepted for
// CALIB_K_COMPUTE. Output is capped at SHELL_MAX_LINES.
#ifndef ENABLE_SHELL_CMD
#define ENABLE_SHELL_CMD 1
#endif
#define SHELL_MAX_LINES 120

// ---------------------------------------------------------------------------
// Detection duty cycle
// ---------------------------------------------------------------------------
// Share of wall-clock time the marker search is allowed to occupy on the
// scheduler thread. Frames arriving while over budget are dropped before any
// work is done on them.
//
// Why this exists: the four channels and every HTTP request run on ONE
// scheduler thread. A lens with no marker in view cannot use the dynamic ROI
// and pays a full-frame scan — measured 200..300 ms at 2592x1520 on .13
// (2026-08-04). Two such lenses at 30 fps ask for far more than a second of
// work per second, so the event queue grows without bound and /status stops
// answering (nginx 502 after ~20 s). Before this cap the app depended on an
// operator never switching on a channel that had nothing to look at.
//
// The cap trades frame rate for responsiveness, and only when oversubscribed:
// under budget nothing is dropped at all. At 60 the thread keeps ~40% free,
// which is far more than /status needs (~1 ms per refresh).
#ifndef DETECT_DUTY_PCT
#define DETECT_DUTY_PCT 60
#endif

// ---------------------------------------------------------------------------
// Reduced-scale SEARCH
// ---------------------------------------------------------------------------
// Shrink factor applied while a channel is SEARCHING (no marker locked on).
// 1 disables it. See ArucoProcessor::setSearchScale for why this is safe --
// the reduced-scale corners only place the next frame's ROI and are never
// published.
//
// Was 2 (quarters the search cost). Set to 1 (2026-08-13) per
// docs/08.13/RESULT_dynroi_size_1ch_2026-08-13.md: scale 2 drops detection to
// 0% the moment a moving marker's ROI touches the frame edge, at any margin --
// not a cost/accuracy tradeoff, a correctness one. det_ms budget has headroom
// at scale 1 across the whole measured margin range, so there is no reason to
// pay that risk for the saved search cost.
#ifndef SEARCH_DOWNSCALE
#define SEARCH_DOWNSCALE 1
#endif

// ---------------------------------------------------------------------------
// Intrinsics (K/dist) calibration — ported from cctv_app
// ---------------------------------------------------------------------------
// Where anything that must survive a restart is written. The original app used
// /mnt/opensdk/storage/cctv_app; the tree is per app name, so ours sits beside
// it. Probed for writability once at start-up and reported by /status, because
// a silent failure here means a calibration the operator believes is saved and
// is not.
#ifndef PERSIST_DIR
#define PERSIST_DIR "/mnt/opensdk/storage/ArucoPosePNM"
#endif

// The printed ChArUco board, ONE description shared by all four lenses: it is
// one physical sheet held in front of each lens in turn, so four independent
// copies could only ever disagree with reality. Persisted separately from
// K/dist (it is a property of the paper, not of a lens).
//
// Defaults match the A3-landscape 7x5 print (420x297mm sheet, 350x250mm
// board pattern, 17 markers):
//   squares:        7 x 5, 50x50mm each
//   ArUco markers:  35x35mm, DICT_4X4_50
//   margins:        35mm left/right, 23.5mm top/bottom
#define CHARUCO_SQUARES_X    7
#define CHARUCO_SQUARES_Y    5
#define CHARUCO_SQUARE_LEN   50.0f
#define CHARUCO_MARKER_LEN   35.0f
#define CHARUCO_DICTIONARY   0  // cv::aruco::DICT_4X4_50
#define CHARUCO_MARGIN_X_MM  35.0f
#define CHARUCO_MARGIN_Y_MM  23.5f
#define CHARUCO_CONFIG_FILE  PERSIST_DIR "/charuco_board.txt"

// Per-lens K/dist. %d is the channel — four separate files, so recalibrating
// one lens cannot corrupt the other three.
#define INTRINSICS_FILE_FMT  PERSIST_DIR "/camera_intrinsics_ch%d.txt"

// Session defaults (both settable at runtime with CALIB_K_SET).
#define K_CALIB_VIEWS 20
#define K_CALIB_RMS_LIMIT       0.8
#define K_CALIB_VIEW_RMS_LIMIT  1.2

// Per-capture quality gates. All of these are bypassed by CALIB_K_GATE 0.
#define K_CALIB_MIN_CORNER_RATIO 0.50  // at least half of interior corners
#define K_CALIB_MIN_COVERAGE     0.025 // convex-hull area / image area
#define K_CALIB_MIN_SHARPNESS    45.0  // variance of Laplacian in board ROI
#define K_CALIB_MIN_GAP_MS       900   // blocks accidental double-clicks
#define K_CALIB_MIN_MOVE_PX      35.0  // mean common-corner displacement

// Gates on by default: the failure they prevent (a session of near-identical
// poses that fits a confident, wrong K) is invisible in the result — RMS comes
// out LOW because the fit reproduces the views it was given.
#define CALIB_QUALITY_GATES_DEFAULT 1

// While a session is open, how often that lens re-runs board detection just to
// tell the operator "the board is visible, N corners".
//
// This exists because the calibration page draws coordinates and no photo (a
// deliberate choice — a live snapshot background costs 120..290 KB a frame).
// Without it, aiming the board would be blind: the only feedback would come
// from pressing capture and being told it failed. A full ChArUco scan is
// ~200 ms at 2592x1520, so it runs ONLY on the one lens that has a session
// open — that channel already bypasses the detection governor entirely while
// calibrating (see the `calibrating` exemption in ProcessRawVideo), so this
// IS the only throttle left on it, not just "the cheap one".
//
// Tried 100 (2026-08-10): below the scan's own ~200 ms cost, so the probe ran
// flat out, back to back, with essentially no idle between scans — on a
// single-lens session with nothing else running, that alone pinned CPU high
// enough that /status stopped answering (the same one-thread contention this
// whole governor exists to prevent, just triggered by ONE channel instead of
// several). 1000 ms left ~80% of the thread idle on that channel; 500 ms
// still leaves a real margin (~200 ms busy / ~300 ms idle per cycle) while
// halving the worst-case aiming lag. Retune with real headroom to spare, not
// down to the scan's own floor.
#ifndef CALIB_PROBE_MS
#define CALIB_PROBE_MS 500
#endif

// How long a marker (or a board) that stops appearing is still treated as
// "recently seen", in milliseconds. Three uses, one constant:
//
//   - /status + the dashboard overlay keep showing the last detection for up
//     to this long instead of flickering to "no markers" on a single missed
//     frame. Display only -- SendPosePackets and HomographyMapper::FeedFrame
//     always see the frame's own (possibly empty) result, held or not.
//
//   - a homography FIT session (HG_*) fills in a registered anchor's
//     position from its last sighting when that one anchor is momentarily
//     missing from the current frame, so a single marker's blink does not
//     throw away an otherwise-good frame (FeedFrame needs every registered
//     marker in the SAME frame or discards it whole). Anchors are
//     floor-fixed for the length of a capture session, so a couple of
//     seconds old position is still the true one.
//
//   - the K/dist calibration page's board probe (see CALIB_PROBE_MS) keeps
//     showing the last board it saw instead of blanking the aiming view
//     every time a probe lands on a bad instant.
//
// Time-based rather than frame-count: the detection governor's frame spacing
// on a channel varies from ~150 ms to 1.5 s+ depending on how many lenses are
// active, so a fixed frame count would mean a wildly different real-world
// grace period depending on load. Accuracy over latency in every one of the
// three uses above -- none of them is a real-time control loop.
#ifndef MARKER_HOLD_MS
#define MARKER_HOLD_MS 2000
#endif

// recent_wiseai_obj_ 캐시(IVA_EVENT에 bbox를 실어 보내기 위한 object_id별 최근
// bbox 기억)의 유효 기간. MARKER_HOLD_MS와 같은 성격 — bbox 스트림과 IVA 이벤트가
// 같은 콜백에 안 실려 오므로, 이벤트가 왔을 때 "최근에 본" bbox를 찾는데 그 최근이
// 얼마나 오래된 것까지 봐줄지. 값 자체는 잠정 — 실측 전.
#ifndef WISEAI_OBJECT_HOLD_MS
#define WISEAI_OBJECT_HOLD_MS 2000
#endif

// ── WiseAI 근접 경고 (ProximityGuard) ────────────────────────────────────────
// 실측 전 잠정값 — 실물 배치 후 작업영역 크기·로봇 정지거리 보고 조정할 것.
// enter < exit 로 히스테리시스를 둔다 (danger가 caution보다 안쪽).
#ifndef PROXIMITY_CAUTION_ENTER_MM
#define PROXIMITY_CAUTION_ENTER_MM 1500.0
#endif
#ifndef PROXIMITY_CAUTION_EXIT_MM
#define PROXIMITY_CAUTION_EXIT_MM 1800.0
#endif
#ifndef PROXIMITY_DANGER_ENTER_MM
#define PROXIMITY_DANGER_ENTER_MM 700.0
#endif
#ifndef PROXIMITY_DANGER_EXIT_MM
#define PROXIMITY_DANGER_EXIT_MM 900.0
#endif
// 후보 상태가 이만큼 유지돼야 실제로 전환한다 (경계에서의 프레임 단위 떨림 방지).
#ifndef PROXIMITY_MIN_DWELL_MS
#define PROXIMITY_MIN_DWELL_MS 300
#endif

// ZONE_EVENT의 발끝 판정 반경. 발은 점이 아니라 면적을 차지하므로, 발끝 한 점이
// 아니라 그 점을 중심으로 한 원이 존과 겹치면 "안에 있다"로 본다. 반경은 bbox
// 하단(가로폭)을 이 값으로 나눈 것 — 사람이 멀수록 bbox가 작아지니 반경도 같이
// 줄어야 화면 어디서든 같은 실제 크기를 뜻하게 되고, 그래서 고정 픽셀값이 아니라
// 비율이다.
//
// 4 = 폭의 1/4. 런타임에 `ZONE_RADIUS <div>`로 바꿀 수 있다 — 값이 감도를 직접
// 좌우하는데(크면 존이 그만큼 넓어지는 효과) 적정값은 현장 화각에 달려 있어서,
// 재빌드 없이 돌려보며 맞추라고 명령을 뒀다. 1 = 폭의 절반(매우 관대),
// 큰 값일수록 점 판정에 가까워진다.
#ifndef ZONE_FOOT_RADIUS_DIV
#define ZONE_FOOT_RADIUS_DIV 4
#endif

// 존 경계 바깥의 완충 밴드. 발끝의 월드 좌표에서 존까지의 거리(mm)로 판정한다 —
// ZONE_FOOT_RADIUS_DIV(bbox 폭 비례)와 달리 물리량이라 자세나 거리에 안 흔들린다.
// 실측(2026-08-20): 같은 사람의 bbox 폭이 몇 초 사이 1.3~1.7배까지 튀어서, 폭 기반
// 반경은 팔 각도만으로 임계값이 20% 움직였다.
//
// DANGER < WARN 이어야 한다 (안쪽이 더 위험).
#ifndef ZONE_MARGIN_DANGER_MM
#define ZONE_MARGIN_DANGER_MM 300
#endif
#ifndef ZONE_MARGIN_WARN_MM
#define ZONE_MARGIN_WARN_MM 500
#endif

// ch1 IVA 영역(사각형) — WiseAI REST API `GET /opensdk/WiseAI/configuration/ivaarea`
// 실측 스냅샷 (2026-08-18, "name1"). 픽셀 좌표, 호모그래피 미적용 — 사람 발끝과의
// 거리도 그래서 픽셀 단위의 "러프한" 값이지 실제 mm 거리가 아니다. 영역을 카메라
// 쪽에서 다시 그리면 이 값도 갱신해야 함 — 동적으로 GET 해오는 게 아니라 그 시점의
// 스냅샷을 박아둔 것.
#define WISEAI_IVA_CH1_POLY \
  {{744.0f, 787.0f}, {1691.0f, 774.0f}, {1795.0f, 1468.0f}, {634.0f, 1472.0f}}

// ── 주행 캘리(오도메트리) ────────────────────────────────────────────────────
// 서버팀과 합의 대기 중인 값과, 현장 실측으로 조정할 값이 섞여 있다. 한 곳에
// 모아 둔 이유는 회신이 오거나 실측이 끝났을 때 고칠 자리가 하나이기 때문이다.
//
// 정지 판정: 요청 도착 이후 촬영된 프레임을 이만큼 모으고, 그 픽셀 표준편차가
// 임계값 이하면 "멈췄다"로 본다. 5프레임/2px 은 잠정값이다 — 운영 탭의 꼭짓점
// 좌표 그래프로 실제 지터를 재서 정한다.
#ifndef ODOM_SETTLE_FRAMES
#define ODOM_SETTLE_FRAMES 5
#endif
#ifndef ODOM_SETTLE_SPREAD_PX
#define ODOM_SETTLE_SPREAD_PX 2.0f
#endif
// 한 지점을 이 시간 안에 판정 못 하면 실패로 접고 서버에 알린다. 서버는 그걸
// 받아 재시도하거나 세션을 접을 수 있다 — 응답이 없으면 서버가 자기 타임아웃까지
// 멈춰 서고, 운영자에게는 "로봇이 안 움직인다"로만 보인다.
#ifndef ODOM_CAPTURE_TIMEOUT_MS
#define ODOM_CAPTURE_TIMEOUT_MS 10000
#endif
// 세션 데드라인. CALIB_DONE 이 안 오는 경우의 탈출구다.
#ifndef ODOM_SESSION_TIMEOUT_MS
#define ODOM_SESSION_TIMEOUT_MS 600000
#endif
// 유효 대응점이 이보다 적으면 피팅하지 않는다. 산술적 하한은 4지만 그러면
// LOO 가 안 돌아 품질 지표가 사라진다(kMinLooAnchors=5, advisory 하한 8).
#ifndef ODOM_MIN_POINTS
#define ODOM_MIN_POINTS 6
#endif
// 번들의 method 필드. 서버팀 회신 §3-1.
#ifndef ODOM_METHOD_NAME
#define ODOM_METHOD_NAME "robot_motion"
#endif
// 정적 앵커(체커보드/폼보드) 방식의 method 이름. 두 방식이 같은 채널 슬롯을
// 두고 경쟁하므로, 저장된 번들만 보고 어느 쪽이 올린 것인지 갈릴 수 있어야 한다
// (created_at 과 함께 — 서버는 마지막에 받은 것을 무조건 이긴 것으로 친다).
#ifndef FLOOR_METHOD_NAME
#define FLOOR_METHOD_NAME "static_anchors"
#endif

// A CAPTURE request (button press) keeps retrying on every incoming frame,
// on each lens with a session open, for up to this long -- instead of
// committing to whatever the very next single frame happens to show. One
// unlucky frame (a hand still settling, a corner blurred mid-refinement) no
// longer costs the operator a manual retry; the app just tries past it.
// Bounded short on purpose: RequestCapture()'s whole point is that several
// lenses bank the SAME board pose, and a burst this size still describes one
// pose to within a fraction of a second on every lens, which a longer window
// would start to erode.
#ifndef CALIB_CAPTURE_BURST_MS
#define CALIB_CAPTURE_BURST_MS 800
#endif

// No shell endpoint here on purpose. The SDK ships DebugHelper, which runs
// sshd on port 55022 and logs in as the app name once app_manifest.json sets
// "UseSSH"/"SSHPassword" (Programming Guide 3.6) — a real interactive shell
// with SCP, running in its OWN process, so it cannot stall detection the way
// an in-process popen() on this scheduler thread would.
// See docs/ON_CAMERA_DEBUG_DASHBOARD_PLAN.md.

#endif // APP_CONFIG_H
