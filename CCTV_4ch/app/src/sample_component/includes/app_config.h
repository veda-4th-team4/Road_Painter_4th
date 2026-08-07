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
 * Minimal config for the marker-detection-only port (ArucoPosePNM).
 *
 * This is a trimmed replacement for cctv_app/inc/app_config.h: only the knobs
 * that pose_sender.cc actually reads are kept. Calibration / homography /
 * central-TLS settings are intentionally absent — this app streams CAM_POSE
 * and nothing else.
 */

// ---------------------------------------------------------------------------
// Vision server (RPi dashboard)
// ---------------------------------------------------------------------------
// The 8082 dashboard instance (cctv_calibration_manager) was moved off the
// default 7000/7001 pair to avoid colliding with the 8083 production instance,
// which owns 7000. Pose goes to 7100, snapshots would go to 7101.
//   dashboard: http://192.168.0.8:8082/
#ifndef POSE_SERVER_IP
#define POSE_SERVER_IP    "192.168.0.8"
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
// 2 is the conservative starting point: it quarters the search cost while
// leaving a 158 px marker at 79 px, comfortably above what a 4x4 dictionary
// needs. 4 quarters it again but puts that same marker at 39 px, near the
// floor -- worth trying on the real scene, not worth defaulting to.
#ifndef SEARCH_DOWNSCALE
#define SEARCH_DOWNSCALE 2
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
// copies could only ever disagree with reality. Defaults match the A2 7x5
// print. Persisted separately from K/dist (it is a property of the paper, not
// of a lens).
#define CHARUCO_SQUARES_X    7
#define CHARUCO_SQUARES_Y    5
#define CHARUCO_SQUARE_LEN   70.0f
#define CHARUCO_MARKER_LEN   50.0f
#define CHARUCO_DICTIONARY   0  // cv::aruco::DICT_4X4_50
#define CHARUCO_MARGIN_X_MM  52.0f
#define CHARUCO_MARGIN_Y_MM  35.0f
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
// ~200 ms at 2592x1520, so it is throttled hard and runs ONLY on the one lens
// that has a session open.
#ifndef CALIB_PROBE_MS
#define CALIB_PROBE_MS 1000
#endif

// No shell endpoint here on purpose. The SDK ships DebugHelper, which runs
// sshd on port 55022 and logs in as the app name once app_manifest.json sets
// "UseSSH"/"SSHPassword" (Programming Guide 3.6) — a real interactive shell
// with SCP, running in its OWN process, so it cannot stall detection the way
// an in-process popen() on this scheduler thread would.
// See docs/ON_CAMERA_DEBUG_DASHBOARD_PLAN.md.

#endif // APP_CONFIG_H
