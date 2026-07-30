#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/**
 * Single place for every site-specific value and feature switch.
 * Changing anything here needs only a rebuild — no code edits elsewhere.
 */

// ===========================================================================
// Feature switches (1 = on, 0 = off)
// ===========================================================================

// Stream CAM_POSE JSON to the vision server over TCP (IF-TCP-003).
#define ENABLE_POSE_STREAM 1

// Direct TLS connection to the central server (role=CCTV / HELLO + POS).
// This is independent of web_gui: 7000/7001 remain available for
// calibration commands and snapshots. Enable after the PEM file below is
// installed on the camera.
#define ENABLE_CENTRAL_TLS_STREAM 1

// BOOT DEFAULT for the raw-fps diagnostic. Toggle it live with the server
// command "RAW_FPS_TEST 0|1" (dashboard button) -- no rebuild needed. Keep this
// 0 so a reboot always returns to normal detecting mode.
//
// The mode skips ALL detection and emits one heartbeat per raw frame callback.
// That is the only way to see how fast the SDK actually hands us frames:
// normally seq counts callbacks, and the callback rate is capped by whichever
// is slower -- SDK delivery or our processing -- so a busy app and a slow SDK
// look identical. With detection off our per-frame cost is ~0, so the observed
// fps IS the SDK's delivery rate.
//
// Measured 2026-07-20 on PNO-A9081R: 9.61fps in this mode vs 3.5fps normally.
// So the SDK was never the limit (it matches the 10fps ceiling for 1080p NV12
// in SDK API Document section 10.1) -- detection cost is, and cutting it raises
// fps roughly 1:1.
//
// NOTE: no marker detection happens while the mode is on.
#define MEASURE_RAW_FPS    0

// On-camera pixel->world homography: CALIB_START command support + "world"
// field in CAM_POSE. Requires ENABLE_POSE_STREAM (commands arrive over the
// same TCP link).
#define ENABLE_HOMOGRAPHY  1

// "HG_SNAPSHOT" server command: JPEG-encode ONE current frame (no overlay) and
// upload it over the snapshot channel as a floor REFERENCE image, together
// with the current H and the anchor world/pixel positions. The dashboard's
// Homography tab draws live marker/world overlays onto this still in a browser
// canvas (see README §6.2). Reuses the bundled JPEG encoder (inc/toojpeg.h) so
// the browser can display it directly — raw RGB/.ppm is not browser-viewable.
// Requires ENABLE_HOMOGRAPHY.
#define ENABLE_HG_SNAPSHOT 1

// Pos/Rot text overlay on the live video (SDK OSD).
#define ENABLE_OSD         0

// Save an annotated JPEG (/mnt/aruco_last.jpg) whenever markers are detected.
// Debug only: adds an NV12->BGR conversion per frame. Needs imgcodecs in the
// OpenCV build (excluded by default — see opencv_cross/build_opencv.sh).
#define SAVE_DEBUG_JPG     0

// "SHELL <cmd>" server command: run <cmd> through /bin/sh on the camera and
// stream its stdout+stderr back to the dashboard. The camera has no SSH server,
// and CLAUDE.md rules out the serial console for anything but one-off checks,
// so this is the only practical way to answer questions like "is /mnt actually
// full, or read-only, or just the wrong path?" (see README known issues).
//
// !! THIS IS A REMOTE SHELL WITH NO AUTHENTICATION !!
// Anyone who can reach the pose port can run arbitrary commands as this app.
// It is here because this is the lab/commissioning build; the production app
// (Camera/) must never ship it. Keep this 0 in any build that leaves the bench.
//
// Also note the command runs on the frame thread: a slow command (sleep, a
// huge cat) stalls detection for its duration, the same trade-off already
// accepted for CALIB_K_COMPUTE. Output is capped (see SHELL_MAX_LINES).
#define ENABLE_SHELL_CMD   1
#define SHELL_MAX_LINES    120

// Persist calibration state (intrinsics K/dist, ChArUco board config, ground
// homography) so it survives a reboot. Set to 0 to keep everything in RAM only.
// Trade-off: calibrations are lost on reboot and must be redone.
//
// Was 0 because writes to the old "/mnt/*.txt" location always failed
// ("calibrated but failed to persist"). That location was almost certainly the
// bug: SDK API doc 6.1 says an app's nand-flash area is
// /mnt/opensdk/storage/<appName>/ -- NOT /mnt itself -- and warns that access
// is only guaranteed when the directory name equals the application name. The
// paths below now follow that rule (see PERSIST_DIR).
//
// Flipped back to 1 now that one_shot() (aruco_detector_cv.cpp) runs
// "mkdir -p " PERSIST_DIR once at startup, so the directory exists before any
// write is attempted. Verify with the Shell tab's /mnt checks after the next
// on-camera rebuild+upload before trusting this — if writes still fail, drop
// back to 0 and re-open the README roadmap item.
#define PERSIST_TO_MNT     1

// App-private persistent storage. MUST match <appName> in IPCameraManifest.xml
// (SDK API doc 6.1: "if the name of the application is SNTest, name the
// directory SNTest in the storage folder ... If their names differ, we cannot
// guarantee your application's access"). Keeping this app-scoped also stops it
// colliding with the sibling Camera app, which writes its own copies.
#define PERSIST_DIR "/mnt/opensdk/storage/cctv_app"

// ===========================================================================
// Network — vision server (RPi)
// ===========================================================================
//
// Deployment values (RPi IP, ports) are kept OUT of git. Copy
// app_config_local.h.example -> app_config_local.h (gitignored) and set your
// values there; it overrides the fallbacks below. Without it the tree still
// compiles, but POSE_SERVER_IP stays a placeholder that reaches no real server.
#if defined(__has_include)
#  if __has_include("app_config_local.h")
#    include "app_config_local.h"
#  endif
#endif

#ifndef POSE_SERVER_IP
#define POSE_SERVER_IP    "127.0.0.1"    // placeholder — real IP goes in app_config_local.h
#endif

// 7000/7001, NOT the 6000/6001 the sibling Camera app uses.
//
// The two camera apps are never meant to run at once, but their RPi servers
// are a different matter: pos_receiver_eo (Camera) is started at boot by
// crontab and holds 6000/6001 permanently. If this app also dialled 6000, its
// server (cctv_calibration_manager) could not bind and would have to be
// swapped in and out by hand every time. Separate ports let both instances sit
// there and simply wait for whichever camera app is installed to call.
//
// Ports must match the instance's launch args:
//   python3 web_gui.py 7000 8082 7001    (pose / GUI / snapshot)
#ifndef POSE_SERVER_PORT
#define POSE_SERVER_PORT  7000
#endif

// Second, dedicated port for on-demand LDC_SNAPSHOT uploads (metrics + raw
// image). Kept separate from POSE_SERVER_PORT: the pose channel is realtime/
// drop-tolerant (non-blocking, never waits), while a snapshot is rare,
// explicit, and must arrive whole — mixing the two would make one
// compromise the other.
//
// The image is sent as RAW RGB24 pixels, not JPEG: the on-camera OpenCV
// build intentionally excludes imgcodecs (see opencv_cross/build_opencv.sh)
// to keep the binary small, so cv::imencode is not available here. The RPi
// side (plain Python) writes it out as a .ppm, which needs no codec either.
#ifndef SNAPSHOT_SERVER_PORT
#define SNAPSHOT_SERVER_PORT 7001
#endif

// Central-server TLS endpoint. Deployment values can be overridden by the
// ignored app_config_local.h; safe fallbacks prevent accidental contact with
// a real server from an unconfigured build.
#ifndef CENTRAL_TLS_SERVER_IP
#define CENTRAL_TLS_SERVER_IP "127.0.0.1"
#endif
#ifndef CENTRAL_TLS_SERVER_PORT
#define CENTRAL_TLS_SERVER_PORT 9000
#endif
// Where the camera unpacks this .cap: appLocation (/mnt) + "opensdk/apps" +
// appName, NOT the /mnt/cctv_app the build container uses. Getting
// this wrong is silent -- central_tls_sender_init() fails to load the PEM,
// clears have_addr, and then never even opens a socket.
#define APP_DIR "/mnt/opensdk/apps/cctv_app"

// PEM containing the central server certificate/CA. The TLS client fails
// closed when this file is missing or invalid.
#ifndef CENTRAL_TLS_CA_FILE
#define CENTRAL_TLS_CA_FILE APP_DIR "/res/images/central_server.crt"
#endif

// Marker id streamed to the central server as POS. The server's POS schema
// carries no marker id -- every POS it receives is taken as THE robot -- so
// anchors and validation markers must be filtered on this side or the server
// would place the robot on a floor marker. The web_gui channel (CAM_POSE) is
// unaffected and keeps reporting every marker with its id.
// This is only the BOOT DEFAULT: the dashboard's central-server tab retargets
// it live with CENTRAL_ID (no rebuild), the same way ANCHOR_SET edits anchors.
//
// 49 is the real robot marker, confirmed by the QT client on 2026-07-28 as
// fixed. It deliberately avoids every floor id in use: the 4x4 foam-board
// layout runs 0..15, the single-board layout 0..3, and kValidationTable 9..17.
// The previous default (15) collided with the 4x4 layout's top-right anchor,
// which would have put the robot on a floor marker with no error anywhere --
// the server takes any POS it receives as the robot.
#ifndef ROBOT_MARKER_ID
#define ROBOT_MARKER_ID 49
#endif

// Reconnect attempt interval when the server link is down (ms).
#define POSE_RECONNECT_MS 2000

// Give up on a central-server TCP connect / TLS handshake after this long and
// start over. connect() and SSL_connect() are non-blocking here and carry no
// deadline of their own.
#define CENTRAL_TLS_HANDSHAKE_MS 5000

// ===========================================================================
// Detection
// ===========================================================================

// Physical marker side length in meters (match the printed marker!).
// Used by solvePnP for the OSD Pos/Rot display; the TCP pixel corners and
// the homography-based world coordinates do NOT depend on it.
#define MARKER_LENGTH_M 0.05f

// ===========================================================================
// OSD layout
// ===========================================================================

#define OSD_LINE_H   32  // vertical spacing between stacked coordinate lines
#define OSD_MARGIN_X 30  // left margin
#define OSD_MAX_LIST 5   // max markers listed at the bottom-left

// On-camera intrinsics (K, dist) calibration via chessboard + CALIB_K_START.
#define ENABLE_INTRINSICS_CALIB 1

// Default state of the calibration quality gates (per-capture coverage/
// sharpness/pose-diversity rejects + compute-time spread/RMS checks). Toggled
// live from the server with "CALIB_K_GATE 0|1" — off = accept whatever is
// captured and compute regardless (expert/quick-test mode).
#define CALIB_QUALITY_GATES_DEFAULT 1

// Residual-distortion check (LDC_CHECK_START/STOP): measures how straight the
// chessboard rows/cols stay in the ALREADY-corrected frames, to judge whether
// the camera's built-in lens-distortion correction is sufficient. Streams the
// metric live; does not calibrate. Requires ENABLE_POSE_STREAM (commands +
// results share the TCP link).
#define ENABLE_LDC_CHECK 1

// A corner is counted as "edge" (vs "center") when its distance from the image
// center exceeds this fraction of the half-diagonal. Distortion concentrates
// at the edge, so edge_max_px is the number that decides sufficiency.
#define LDC_EDGE_RADIUS_FRAC 0.55

// Default ChArUco board. Runtime values can be changed from the calibration
// UI and are persisted in CHARUCO_CONFIG_FILE. These defaults exactly match
// the project's A2 board: 7x5 squares, 70 mm square, 50 mm marker,
// DICT_4X4_50, 490x350 mm pattern, 52/35 mm outer margins on A2 landscape.
#define CHARUCO_SQUARES_X    7
#define CHARUCO_SQUARES_Y    5
#define CHARUCO_SQUARE_LEN   70.0f
#define CHARUCO_MARKER_LEN   50.0f
#define CHARUCO_DICTIONARY   0  // cv::aruco::DICT_4X4_50
#define CHARUCO_MARGIN_X_MM  52.0f
#define CHARUCO_MARGIN_Y_MM  35.0f
#define CHARUCO_CONFIG_FILE  PERSIST_DIR "/charuco_board.txt"
// A row/column of interior corners needs at least this many detected points to
// measure its bow (2 points are always collinear, so 3 is the minimum).
#define LDC_MIN_LINE_POINTS 3

// ===========================================================================
// Intrinsics calibration (ChArUco)
// ===========================================================================

// Number of ACCEPTED, diverse views required before Compute is enabled.
// Capture does not auto-compute at the final view: the UI first reports
// target/target and the operator explicitly starts the blocking calculation.
#define K_CALIB_VIEWS 20

// Per-capture quality gates. Bad frames never advance the view counter.
#define K_CALIB_MIN_CORNER_RATIO 0.50  // at least half of interior corners
#define K_CALIB_MIN_COVERAGE     0.025 // convex-hull area / image area
#define K_CALIB_MIN_SHARPNESS    45.0  // variance of Laplacian in board ROI
#define K_CALIB_MIN_GAP_MS       900   // blocks accidental double-clicks
#define K_CALIB_MIN_MOVE_PX      35.0  // mean common-corner displacement

// Accept threshold for RMS reprojection error (px).
#define K_CALIB_RMS_LIMIT       0.8
#define K_CALIB_VIEW_RMS_LIMIT  1.2

// Persisted intrinsics location (nand flash — survives restarts). See PERSIST_DIR.
#define INTRINSICS_FILE PERSIST_DIR "/camera_intrinsics.txt"

// Built-in fallback intrinsics, baked into the binary.
//
// intrinsics_init() applies these at startup when INTRINSICS_FILE is missing or
// unreadable, so a freshly flashed camera — or one whose /mnt write silently
// failed (see PERSIST_TO_MNT above) — still runs with a calibrated lens instead
// of no K at all. A persisted file always wins, and a CALIB_K_PROFILE_LOAD or
// CALIB_K_COMPUTE at runtime overrides these for the session as before.
//
// Values: ChArUco calibration of the PNO-A9081RG on the project's A2 board,
// 2026-07-28. They only mean anything at the stream resolution they were
// calibrated at (cx/cy ~924/516 => 1920x1080) — recalibrate if that changes.
// Set to 0 to restore the old "no K until calibrated" startup behaviour.
#define K_DEFAULT_INTRINSICS 1
#define K_DEFAULT_FX 1283.53
#define K_DEFAULT_FY 1283.02
#define K_DEFAULT_CX  924.43
#define K_DEFAULT_CY  515.83
// OpenCV dist order: k1 k2 p1 p2 k3
#define K_DEFAULT_D0 -0.42876    // k1 (r^2)
#define K_DEFAULT_D1  0.283696   // k2 (r^4)
#define K_DEFAULT_D2 -0.000125   // p1 (tangential)
#define K_DEFAULT_D3  0.000616   // p2 (tangential)
#define K_DEFAULT_D4 -0.142204   // k3 (r^6)

// Shown as the active profile name while the built-in values are in use, so the
// dashboard can tell "factory default" apart from "nothing loaded" ("기본").
#define K_DEFAULT_PROFILE_NAME "내장기본"

// Keep a JPEG of each accepted CALIB_K_CAPTURE view in memory (marker/corner
// overlay drawn), to be uploaded to the vision server on demand via the
// CALIB_K_UPLOAD command. Unlike the raw LDC_SNAPSHOT path, this compresses
// on-camera first (a bundled, dependency-free encoder — inc/toojpeg.h), so the
// per-view stall is just the encode (~tens of ms) and the multi-MB transfer is
// deferred off the frame thread. Requires ENABLE_INTRINSICS_CALIB.
#define ENABLE_CALIB_VIEW_UPLOAD 1

// JPEG quality (1..100) for the stored calibration views. 85 keeps corners
// crisp for later inspection while staying ~10-30x smaller than raw RGB.
#define CALIB_VIEW_JPEG_QUALITY 85

// ===========================================================================
// Homography calibration (anchor markers)
// ===========================================================================

// Calculation anchor ids + their measured positions on the ground plane (mm).
// These are DEFAULTS only: the dashboard can re-measure a site at runtime with
// the ANCHOR_SET command (homography_set_anchor) — no rebuild needed. The ids
// and count here stay fixed; only the world coordinates are runtime-editable.
// Eight anchors are distributed around the work-area perimeter. All eight must
// be visible during CALIB_START; H is fitted robustly from their averaged
// centers. TODO(현장 설치 시): replace these coordinates with surveyed values.
// All points must lie on the floor. Calculation anchors use ids 0..7.
struct AnchorConfig { int id; double wx, wy; };

// Work area 3000 x 2000 mm: corners + edge midpoints, counterclockwise from the
// bottom-left origin. TODO(현장 설치 시): replace with surveyed values.
static const AnchorConfig kAnchorTable[] = {
    {0,    0.0,    0.0},   // bottom-left
    {1, 1500.0,    0.0},   // bottom-middle
    {2, 3000.0,    0.0},   // bottom-right
    {3, 3000.0, 1000.0},   // right-middle
    {4, 3000.0, 2000.0},   // top-right
    {5, 1500.0, 2000.0},   // top-middle
    {6,    0.0, 2000.0},   // top-left
    {7,    0.0, 1000.0},   // left-middle
};

// Independent validation markers. These MUST NOT be used by the H fit. When
// visible, the dashboard compares their H-derived position against these known
// surveyed positions and reports per-marker error, RMSE and max error. ids
// 17..14 (descending) intentionally do not overlap calculation anchors (0..7).
// 8 validation points, none used in the H fit. INNER 4 test interpolation
// accuracy (should be good); OUTER 4 sit ~300 mm beyond each edge midpoint,
// outside the anchor hull, to measure how fast H degrades in extrapolation
// (large error there is EXPECTED — read inner vs outer separately).
static const AnchorConfig kValidationTable[] = {
    // inner 5 (inside hull) — interpolation: dead center + four quadrant centers
    {17, 1500.0, 1000.0},   // dead center = worst-case interpolation
    {16,  750.0,  500.0},   // lower-left quadrant
    {15, 2250.0,  500.0},   // lower-right quadrant
    {14,  750.0, 1500.0},   // upper-left quadrant
    {13, 2250.0, 1500.0},   // upper-right quadrant
    // outer 4 (~300 mm outside each edge midpoint) — extrapolation test
    {12, 1500.0, -300.0},   // below bottom edge
    {11, 1500.0, 2300.0},   // above top edge
    {10, -300.0, 1000.0},   // left of left edge
    { 9, 3300.0, 1000.0},   // right of right edge
};

// Frames with ALL 8 calculation anchors visible needed before H is computed (averaging
// suppresses per-frame subpixel jitter).
#define CALIB_GOOD_FRAMES 30

// Give up after this many total frames (at 5 fps, 300 frames ~= 60 s).
#define CALIB_MAX_FRAMES  300

// Persisted H location (nand flash — survives app restarts). See PERSIST_DIR.
#define HOMOGRAPHY_FILE PERSIST_DIR "/aruco_homography.txt"

// Robot marker mounting height above the floor plane (meters). Only the BOOT
// DEFAULT: the dashboard sets it live (MARKER_HEIGHT) and it is persisted in
// MARKER_HEIGHT_FILE, because it depends on how THIS robot was built and is
// not known until the marker is physically mounted.
//
// 0 means "H_marker == H_floor", i.e. no parallax correction — correct only if
// the marker really lies on the floor. See inc/marker_plane.h for why a marker
// mounted even 250 mm up produces metres of systematic error at the edge of a
// large work area.
#define MARKER_HEIGHT_M 0.0
#define MARKER_HEIGHT_FILE PERSIST_DIR "/marker_height.txt"

// Persisted detection tuning (ROI + adaptive-threshold pass count) — the two
// settings that decide frame rate. Written by TUNE_SAVE, reloaded at startup.
// See inc/detect_tuning.h for why these live in a file instead of here.
#define DETECT_TUNING_FILE PERSIST_DIR "/detect_tuning.txt"

// Upper bound on the id list DYNROI_IDS accepts. The point of an id filter is
// to collapse the tracked box onto a few markers, so a long list defeats it;
// this only exists to bound the command parse and the ack line. Not persisted —
// the filter is a live tuning knob and resets to "all markers" on reboot.
#define DYNROI_MAX_TRACK_IDS 16u

#endif // APP_CONFIG_H
