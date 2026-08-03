#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/wait.h>   // WEXITSTATUS for the SHELL command

#include <SDKAPI/opensdk_defines.h>
#include <SDKAPI/opensdk_device.h>
#include <SDKAPI/opensdk_profile.h>
#include <SDKAPI/opensdk_ptz.h>
#include <SDKAPI/opensdk_record.h>
#include <SDKAPI/opensdk_videoanalytics.h>
#include <SDKAPI/opensdk_videosetup.h>

// All tunables (server address, marker size, feature switches, anchors, OSD
// layout) live in app_config.h — edit there, rebuild, done.
#include "app_config.h"
#include "aruco_processor.h"
#include "detect_tuning.h"
#include "dyn_roi.h"
#include "pose_sender.h"
#include "central_tls_sender.h"
#include "homography_mapper.h"
#include "intrinsics_calibrator.h"
#include "ldc_checker.h"
#include "marker_plane.h"
#include "snapshot_sender.h"

// The bundled JPEG encoder (toojpeg) is shared by two features: stored
// calibration views (CALIB_K_UPLOAD) and the homography reference still
// (HG_SNAPSHOT). Either one alone is reason enough to pull in the encoder and
// std::string it builds JSON with.
#define NEED_JPEG_ENCODER \
    ((ENABLE_INTRINSICS_CALIB && ENABLE_CALIB_VIEW_UPLOAD) || \
     (ENABLE_HOMOGRAPHY && ENABLE_HG_SNAPSHOT))

#if NEED_JPEG_ENCODER
#include "toojpeg.h"
#include <string>
#endif

#if ENABLE_INTRINSICS_CALIB && ENABLE_CALIB_VIEW_UPLOAD
#include "calib_view_store.h"
#include <thread>
#include <atomic>
#endif

#include <sys/time.h>
#include <unistd.h>     // sysconf: clock ticks and core count for CPU_STAT
#include <vector>
#include <algorithm>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#if SAVE_DEBUG_JPG
#include <opencv2/imgcodecs.hpp>
#endif

// ArUco detector (created in one_shot, reused per frame).
static ArucoProcessor* g_aruco = NULL;

static long epoch_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long) tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

// CPU load reporting.
//
// Two numbers, because they answer different questions. "app" is this process
// (/proc/self/stat utime+stime) and says whether OUR work is the load. "sys" is
// the whole camera (/proc/stat) and says whether the box is busy regardless of
// us -- encoder and SDK pipeline included. Seeing only one of them cannot
// distinguish "we are saturating a core" from "the camera is already loaded".
//
// Both are deltas over wall time, so on a multi-core SoC "app" can exceed 100%
// (200% = two cores fully used). The core count ships alongside rather than
// normalising: dividing hides exactly the case worth seeing, one core pinned.
//
// Deliberately NOT the SHELL command: that forks /bin/sh on the frame thread,
// and it is meant to be compiled out of production builds. Reading two /proc
// files costs no fork and can stay on always.
#define CPU_REPORT_INTERVAL_MS 2000

static long g_cpu_last_ms   = 0;
static unsigned long long g_cpu_last_self  = 0;  // clock ticks
static unsigned long long g_cpu_last_busy  = 0;  // jiffies
static unsigned long long g_cpu_last_total = 0;

// utime+stime of this process, in clock ticks. 0 when unavailable.
static unsigned long long read_self_ticks(void)
{
    FILE* f = fopen("/proc/self/stat", "r");
    if (f == NULL)
        return 0;
    // Fields 14/15 are utime/stime. comm (field 2) may contain spaces inside
    // parentheses, so skip past the LAST ')' before counting fields.
    char buf[1024];
    size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    if (n == 0)
        return 0;
    buf[n] = '\0';
    const char* p = strrchr(buf, ')');
    if (p == NULL)
        return 0;
    ++p;
    // After ')' the next field is state (3), so utime is the 11th field from here.
    unsigned long long ut = 0, st = 0;
    int matched = sscanf(p, " %*c %*d %*d %*d %*d %*d %*u %*u %*u %*u %*u %llu %llu",
                         &ut, &st);
    return (matched == 2) ? (ut + st) : 0;
}

// Aggregate jiffies from /proc/stat's first line. busy excludes idle+iowait --
// iowait is not CPU work, and counting it inflates load on a flash-backed box.
static bool read_sys_jiffies(unsigned long long* busy, unsigned long long* total)
{
    FILE* f = fopen("/proc/stat", "r");
    if (f == NULL)
        return false;
    char line[512];
    if (fgets(line, sizeof(line), f) == NULL) { fclose(f); return false; }
    fclose(f);
    unsigned long long v[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    // user nice system idle iowait irq softirq steal
    int matched = sscanf(line, "cpu %llu %llu %llu %llu %llu %llu %llu %llu",
                         &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7]);
    if (matched < 4)
        return false;
    unsigned long long sum = 0;
    for (int i = 0; i < 8; ++i) sum += v[i];
    *total = sum;
    *busy  = sum - v[3] - v[4];   // minus idle and iowait
    return true;
}

// Emit CPU_STAT at most every CPU_REPORT_INTERVAL_MS. The first call only
// records a baseline: a percentage needs two samples, and reporting the
// process's whole lifetime average as if it were "now" would be wrong.
static void report_cpu_if_due(void)
{
    const long now = epoch_ms();
    if (g_cpu_last_ms != 0 && (now - g_cpu_last_ms) < CPU_REPORT_INTERVAL_MS)
        return;

    const unsigned long long self = read_self_ticks();
    unsigned long long busy = 0, total = 0;
    const bool sysOk = read_sys_jiffies(&busy, &total);

    const long  prevMs   = g_cpu_last_ms;
    const unsigned long long prevSelf  = g_cpu_last_self;
    const unsigned long long prevBusy  = g_cpu_last_busy;
    const unsigned long long prevTotal = g_cpu_last_total;

    g_cpu_last_ms    = now;
    g_cpu_last_self  = self;
    g_cpu_last_busy  = busy;
    g_cpu_last_total = total;

    if (prevMs == 0)
        return;                      // baseline only

    const long hz = sysconf(_SC_CLK_TCK);
    const double elapsedS = (now - prevMs) / 1000.0;
    double appPct = -1.0, sysPct = -1.0;

    if (hz > 0 && elapsedS > 0.0 && self >= prevSelf)
        appPct = (double) (self - prevSelf) / (double) hz / elapsedS * 100.0;
    if (sysOk && total > prevTotal && busy >= prevBusy)
        sysPct = (double) (busy - prevBusy) / (double) (total - prevTotal) * 100.0;

    const long cores = sysconf(_SC_NPROCESSORS_ONLN);
    char json[192];
    snprintf(json, sizeof(json),
             "{\"type\":\"CPU_STAT\",\"app_pct\":%.1f,\"sys_pct\":%.1f,"
             "\"cores\":%d}",
             appPct, sysPct, (int) (cores > 0 ? cores : 1));
    pose_sender_send_control_line(json);
}

static void json_escape(const char* src, char* dst, size_t dstSize)
{
    if (dstSize == 0)
        return;
    size_t out = 0;
    for (size_t i = 0; src != NULL && src[i] != '\0' && out + 1 < dstSize;
         ++i) {
        unsigned char c = (unsigned char) src[i];
        if ((c == '"' || c == '\\') && out + 2 < dstSize) {
            dst[out++] = '\\';
            dst[out++] = (char) c;
        } else if (c >= 0x20) {
            dst[out++] = (char) c;
        } else {
            dst[out++] = ' ';
        }
    }
    dst[out] = '\0';
}

// ===========================================================================
// OSD overlay (live-video Pos/Rot text) — disable via ENABLE_OSD
// ===========================================================================
#if ENABLE_OSD

/**
 * Show a text message on the live video via the SDK OSD API.
 * OPENSDK_SET_OSD { char message[45]; uint msec_duration; uint x, y; } is sent
 * with send_event(OPENSDK_SET_OSD_MESSAGE, &osd, sizeof(osd)).
 *
 * Called every raw frame so the OSD refreshes before it times out.
 */
// Send one OSD text message at a given position.
// x/y are in the raw-frame pixel space; clamp to >= 0.
static void osd_send(const char* msg, int x, int y)
{
    OPENSDK_SET_OSD osd;
    memset(&osd, 0, sizeof(osd));
    strncpy(osd.message, msg, sizeof(osd.message) - 1);
    osd.msec_duration = 2000; // outlasts one raw frame (~1 fps) so it persists
    osd.x = (x > 0) ? (unsigned int) x : 0;
    osd.y = (y > 0) ? (unsigned int) y : 0;
    OPENSDK::EVENT::send_event(OPENSDK_SET_OSD_MESSAGE, &osd, sizeof(osd));
}

static void update_osd_status(const std::vector<ArucoProcessor::Detection>& dets,
                              int width, int height)
{
    (void) width;

    // Per-marker data list, fixed at the BOTTOM-LEFT (does NOT follow markers).
    // One line each: ID, Pos(X,Y,Z in meters), Rot(Rx,Ry,Rz deg). Lines are
    // packed upward from the bottom edge.
    int n = (int) dets.size();
    if (n > OSD_MAX_LIST) n = OSD_MAX_LIST;
    for (int i = 0; i < n; ++i) {
        const ArucoProcessor::Detection& d = dets[i];

        char buf[OPENSDK_MAX_OSD_LENGTH];
        // Compact to fit the 45-char OSD limit: P = position (m), R = rotation (deg).
        snprintf(buf, sizeof(buf), "ID%d P%.2f,%.2f,%.2f R%.0f,%.0f,%.0f",
                 d.id, d.tvec[0], d.tvec[1], d.tvec[2],
                 d.euler[0], d.euler[1], d.euler[2]);

        int y = height - 40 - (n - 1 - i) * OSD_LINE_H;
        osd_send(buf, OSD_MARGIN_X, y);
        // No per-marker serial log: blocking serial writes per frame add
        // latency (see note in recv_event). The OSD itself is the live
        // feedback; the TCP stream is the machine-readable output.
    }
}

#endif // ENABLE_OSD

#if ENABLE_CENTRAL_TLS_STREAM
// Which marker id is streamed to the central server as POS. That schema has no
// id field, so whatever we send there IS the robot as far as the server is
// concerned; anchors and validation markers must never leak onto the channel.
// Boot default comes from app_config.h, CENTRAL_ID retargets it live.
static int g_central_marker_id = ROBOT_MARKER_ID;
// Separate from the link switch: this silences POS while leaving the session
// registered, which is what you want when the server should see the camera as
// present but not treat a floor marker as the robot.
static bool g_central_pos_enabled = true;
#endif

// ===========================================================================
// TCP pose streaming (IF-TCP-003) — disable via ENABLE_POSE_STREAM
// ===========================================================================
#if ENABLE_POSE_STREAM

/**
 * Stream this frame's detections to the vision server (IF-TCP-003).
 *
 * One CAM_POSE JSON line per detected marker. On a miss we still send a
 * confidence:0 heartbeat so the server can tell "camera alive, marker lost"
 * apart from "camera/link dead" (its watchdog stops the robot on the latter).
 *
 * Only FRESH detections are sent — the OSD hold window is a display nicety;
 * feeding held (stale) corners to the robot as if they were current would
 * defeat the server's latency compensation.
 */
static void send_pose_packets(const std::vector<ArucoProcessor::Detection>& fresh,
                              long t_frame_ms, int frame_w, int frame_h)
{
    static unsigned long seq = 0;
    char json[768];

    // seq increments once per frame-processing call, NOT per marker. When a
    // frame yields several markers they all share this frame's seq, so the
    // server can group same-frame detections. (seq is thus a frame counter,
    // not a packet counter — multiple packets per frame is expected.)
    ++seq;

    // "w"/"h" state which pixel space the corners live in. Required for the
    // server-side fallback path (SR-CAM-002): the sub-stream the server
    // detects on may have a different resolution, and K/H math must know
    // which coordinate space it is applying to.
    // "t_det" is the detectMarkers()-only cost (ms). proc (t - t_frame) covers
    // the whole frame path, so proc - t_det isolates everything else and tells
    // us whether the marker search is really where the time goes.
    const double detMs = (g_aruco != NULL) ? g_aruco->lastDetectMs() : -1.0;

    if (fresh.empty()) {
        snprintf(json, sizeof(json),
                 "{\"type\":\"CAM_POSE\",\"seq\":%lu,\"t\":%ld,\"t_frame\":%ld,"
                 "\"w\":%d,\"h\":%d,\"t_det\":%.1f,\"confidence\":0,\"corners\":[]}",
                 seq, epoch_ms(), t_frame_ms, frame_w, frame_h, detMs);
        pose_sender_send_line(json);
        return;
    }

    for (size_t i = 0; i < fresh.size(); ++i) {
        const ArucoProcessor::Detection& d = fresh[i];
        if (d.corners2d.size() < 4)
            continue;
        int len = snprintf(json, sizeof(json),
                 "{\"type\":\"CAM_POSE\",\"seq\":%lu,\"t\":%ld,\"t_frame\":%ld,"
                 "\"w\":%d,\"h\":%d,\"t_det\":%.1f,\"id\":%d,\"confidence\":1.0,"
                 "\"corners\":["
                 "{\"x\":%.2f,\"y\":%.2f},{\"x\":%.2f,\"y\":%.2f},"
                 "{\"x\":%.2f,\"y\":%.2f},{\"x\":%.2f,\"y\":%.2f}]",
                 seq, epoch_ms(), t_frame_ms, frame_w, frame_h, detMs, d.id,
                 d.corners2d[0].x, d.corners2d[0].y,
                 d.corners2d[1].x, d.corners2d[1].y,
                 d.corners2d[2].x, d.corners2d[2].y,
                 d.corners2d[3].x, d.corners2d[3].y);
        if (len <= 0 || len >= (int) sizeof(json))
            continue;

#if ENABLE_HOMOGRAPHY
        // If a homography has been calibrated, ALSO ship physical ground
        // coordinates (mm) + heading (deg): center via H, heading from the
        // marker's top-edge direction mapped through the same H. Raw pixel
        // corners stay in the packet so the server keeps its original
        // IF-TCP-003 view and can cross-check the camera-side mapping.
        if (homography_active()) {
            float cx = 0.f, cy = 0.f;
            for (int k = 0; k < 4; ++k) {
                cx += d.corners2d[k].x;
                cy += d.corners2d[k].y;
            }
            cx *= 0.25f;
            cy *= 0.25f;
            float tx = (d.corners2d[0].x + d.corners2d[1].x) * 0.5f;
            float ty = (d.corners2d[0].y + d.corners2d[1].y) * 0.5f;

            double wcx, wcy, wtx, wty;
            if (homography_pixel_to_world(cx, cy, &wcx, &wcy) &&
                homography_pixel_to_world(tx, ty, &wtx, &wty)) {
                double theta = atan2(wty - wcy, wtx - wcx) * 180.0 / M_PI;
                len += snprintf(json + len, sizeof(json) - len,
                                ",\"world\":{\"x\":%.1f,\"y\":%.1f,\"theta\":%.1f}",
                                wcx, wcy, theta);
                if (len >= (int) sizeof(json))
                    continue;
            }
        }
#endif // ENABLE_HOMOGRAPHY

        snprintf(json + len, sizeof(json) - len, "}");
        pose_sender_send_line(json);

#if ENABLE_CENTRAL_TLS_STREAM
        // The central server owns undistort + H_marker. Send only the raw
        // pixels in its POS schema, and never send the web_gui heartbeat.
        // POS has no id field, so every packet the server receives is treated
        // as the robot -- anchors and validation markers have to be dropped
        // here, otherwise the last marker of the frame wins and the robot
        // lands on a floor anchor.
        if (g_central_pos_enabled && d.id == g_central_marker_id) {
            float central_corners[4][2] = {
                {d.corners2d[0].x, d.corners2d[0].y},
                {d.corners2d[1].x, d.corners2d[1].y},
                {d.corners2d[2].x, d.corners2d[2].y},
                {d.corners2d[3].x, d.corners2d[3].y},
            };
            central_tls_sender_send_pos(central_corners);
        }
#endif
    }
}

// ---------------------------------------------------------------------------
// Server command dispatch.
// Adding a command = write a handler + add one row to kCommands. Matching is
// by substring, so the server may send bare words or wrap them in JSON.
// ---------------------------------------------------------------------------
typedef void (*CommandHandler)(void);

#if ENABLE_HOMOGRAPHY
static void cmd_calib_start(void)
{
#if ENABLE_INTRINSICS_CALIB
    if (intrinsics_collecting()) {
        pose_sender_send_control_line(
            "{\"type\":\"CALIB_RESULT\",\"ok\":false,"
            "\"reason\":\"finish intrinsics calibration first\"}");
        return;
    }
#endif
#if ENABLE_LDC_CHECK
    if (ldc_check_active()) {
        pose_sender_send_control_line(
            "{\"type\":\"CALIB_RESULT\",\"ok\":false,"
            "\"reason\":\"stop LDC check first\"}");
        return;
    }
#endif
    homography_start_calib();
    pose_sender_send_control_line(
        "{\"type\":\"CALIB_ACK\",\"state\":\"collecting\"}");
    debug_message("ArUco: homography calibration started\n");
}

// HG_QUERY: report the currently-loaded 3x3 homography (like CALIB_K_QUERY for
// intrinsics). Answers immediately from the command handler.
static void cmd_hg_query(void)
{
    double h[9];
    if (!homography_get(h)) {
        pose_sender_send_control_line(
            "{\"type\":\"CALIB_HG_QUERY\",\"available\":false}");
        return;
    }
    char json[384];
    snprintf(json, sizeof(json),
             "{\"type\":\"CALIB_HG_QUERY\",\"available\":true,"
             "\"H\":[%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e]}",
             h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], h[8]);
    pose_sender_send_control_line(json);
}

// HG_SAVE: persist the currently active H to PERSIST_DIR right now. No frame
// data involved, so this answers synchronously like HG_QUERY above.
static void cmd_hg_save(void)
{
    bool ok = homography_save();
    char json[192];
    snprintf(json, sizeof(json), "{\"type\":\"HG_SAVE\",\"ok\":%s,\"reason\":\"%s\"}",
             ok ? "true" : "false", ok ? "" : homography_fail_reason());
    pose_sender_send_control_line(json);
}

// MARKER_PLANE_QUERY: derive H_marker for the current marker height and report
// it together with the camera pose the floor H implies.
//
// The camera height is the check that matters. It is DERIVED, not measured, so
// an installer who knows the camera is 1.5 m up can tell at a glance whether
// the decomposition (and therefore K and H) is sane. A camera that comes back
// at 4 m means the parallax correction is wrong by the same factor.
static void cmd_marker_plane_query(void)
{
    double h[9];
    if (!homography_get(h)) {
        pose_sender_send_control_line(
            "{\"type\":\"MARKER_PLANE\",\"ready\":false,"
            "\"reason\":\"no homography\",\"action\":\"query\"}");
        return;
    }

    const double height = marker_plane_height_mm();
    const char*  reason = "";
    double cz = 0.0, nx = 0.0, ny = 0.0;
    double hm[9];
    const bool pose_ok = marker_plane_camera_pose(h, &cz, &nx, &ny, &reason);
    const bool derive_ok = pose_ok && marker_plane_derive(h, height, hm, &reason);

    if (!derive_ok) {
        char json[384];
        snprintf(json, sizeof(json),
                 "{\"type\":\"MARKER_PLANE\",\"ready\":false,\"height_mm\":%.1f,"
                 "\"reason\":\"%s\",\"action\":\"query\"}",
                 height, reason ? reason : "");
        pose_sender_send_control_line(json);
        return;
    }

    // Report BOTH heights. cz is what the decomposition implies; measured is
    // what the installer taped. When measured > 0 it is the one that drives
    // the correction, and the gap between the two is the decomposition's
    // scale error -- the single most useful number for judging K and H.
    const double measured  = marker_plane_camera_height_mm();
    const double effective = (measured > 0.0) ? measured : cz;

    char json[768];
    snprintf(json, sizeof(json),
             "{\"type\":\"MARKER_PLANE\",\"ready\":true,\"height_mm\":%.1f,"
             "\"camera_z_mm\":%.1f,\"camera_z_measured_mm\":%.1f,"
             "\"camera_z_used_mm\":%.1f,"
             "\"nadir_x_mm\":%.1f,\"nadir_y_mm\":%.1f,"
             "\"ratio\":%.4f,\"action\":\"query\","
             "\"H_marker\":[%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e]}",
             height, cz, measured, effective, nx, ny,
             (effective != 0.0) ? height / effective : 0.0,
             hm[0], hm[1], hm[2], hm[3], hm[4], hm[5], hm[6], hm[7], hm[8]);
    pose_sender_send_control_line(json);
}

// MARKER_HEIGHT <mm>: set the robot marker's mounting height. Applies to RAM
// only, like HG_SET — MARKER_PLANE_SAVE commits it.
static bool handle_marker_height(const char* cmd)
{
    const char* p = strstr(cmd, "MARKER_HEIGHT");
    if (p == NULL)
        return false;
    double mm = -1.0;
    if (sscanf(p + 13, "%lf", &mm) != 1) {   // 13 = strlen("MARKER_HEIGHT")
        pose_sender_send_control_line(
            "{\"type\":\"MARKER_PLANE\",\"ready\":false,"
            "\"reason\":\"usage: MARKER_HEIGHT <mm>\",\"action\":\"set\"}");
        return true;
    }
    if (!marker_plane_set_height_mm(mm)) {
        pose_sender_send_control_line(
            "{\"type\":\"MARKER_PLANE\",\"ready\":false,"
            "\"reason\":\"height must be 0..100000 mm\",\"action\":\"set\"}");
        return true;
    }
    debug_message("ArUco: marker height %.1f mm\n", mm);
    cmd_marker_plane_query();      // answer with the freshly derived plane
    return true;
}

// CAMERA_HEIGHT <mm>: the tape-measured mounting height. 0 clears it and
// returns to the value derived from the H decomposition.
static bool handle_camera_height(const char* cmd)
{
    const char* p = strstr(cmd, "CAMERA_HEIGHT");
    if (p == NULL)
        return false;
    double mm = -1.0;
    if (sscanf(p + 13, "%lf", &mm) != 1) {   // 13 = strlen("CAMERA_HEIGHT")
        pose_sender_send_control_line(
            "{\"type\":\"MARKER_PLANE\",\"ready\":false,"
            "\"reason\":\"usage: CAMERA_HEIGHT <mm>  (0 = use derived)\","
            "\"action\":\"set_camera\"}");
        return true;
    }
    if (!marker_plane_set_camera_height_mm(mm)) {
        pose_sender_send_control_line(
            "{\"type\":\"MARKER_PLANE\",\"ready\":false,"
            "\"reason\":\"camera height must be 0..100000 mm\","
            "\"action\":\"set_camera\"}");
        return true;
    }
    debug_message("ArUco: camera height %.1f mm (%s)\n",
                  mm, (mm > 0.0) ? "measured" : "derived");
    cmd_marker_plane_query();
    return true;
}

// MARKER_PLANE_SAVE: persist the height, like HG_SAVE for the matrix.
static void cmd_marker_plane_save(void)
{
    const bool ok = marker_plane_save_height();
    char json[192];
    snprintf(json, sizeof(json),
             "{\"type\":\"MARKER_PLANE_SAVE\",\"ok\":%s,\"height_mm\":%.1f}",
             ok ? "true" : "false", marker_plane_height_mm());
    pose_sender_send_control_line(json);
}

// HG_SET h00 h01 ... h22: install a matrix fitted by the RPi/PC experiment.
// It intentionally does not save automatically: applying a candidate and
// making it persistent are two different operator actions.
static bool handle_hg_set(const char* cmd)
{
    const char* p = strstr(cmd, "HG_SET");
    if (p == NULL)
        return false;
    double h[9];
    int n = sscanf(p + 6, "%lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   &h[0], &h[1], &h[2], &h[3], &h[4],
                   &h[5], &h[6], &h[7], &h[8]);
    bool ok = (n == 9) && homography_set(h);
    char json[384];
    if (ok) {
        snprintf(json, sizeof(json),
                 "{\"type\":\"HG_SET\",\"ok\":true,"
                 "\"H\":[%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e]}",
                 h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], h[8]);
    } else {
        snprintf(json, sizeof(json),
                 "{\"type\":\"HG_SET\",\"ok\":false,\"reason\":\"%s\"}",
                 n == 9 ? homography_fail_reason() : "need 9 matrix values");
    }
    pose_sender_send_control_line(json);
    return true;
}

static bool handle_hg_coord_mode(const char* cmd)
{
    const char* p = strstr(cmd, "HG_COORD_MODE");
    if (p == NULL) return false;
    bool undistort = strstr(p + 13, "undistort") != NULL;
    bool ok = homography_set_undistort(undistort);
    pose_sender_send_control_line(ok
        ? (undistort ? "{\"type\":\"HG_COORD_MODE\",\"ok\":true,\"mode\":\"undistort\"}"
                     : "{\"type\":\"HG_COORD_MODE\",\"ok\":true,\"mode\":\"raw\"}")
        : "{\"type\":\"HG_COORD_MODE\",\"ok\":false,\"reason\":\"K/dist unavailable or calibration active\"}");
    return true;
}

// One-shot floor-board homography.  The board's ChArUco ids imply their world
// coordinates in mm, so no per-corner surveying is required.  We fit the
// first 17 detected ids and report the remaining detected corners as an
// independent validation set.  The board must be fixed flat on the floor.
#if ENABLE_INTRINSICS_CALIB
static bool g_hg_charuco_requested = false;
static void cmd_hg_charuco_start(void)
{
    g_hg_charuco_requested = true;
    pose_sender_send_control_line("{\"type\":\"HG_CHARUCO_ACK\",\"state\":\"waiting\",\"fit_corners\":17}");
}

static void run_hg_charuco(const cv::Mat& gray)
{
    if (!g_hg_charuco_requested)
        return;
    std::vector<cv::Point2f> corners;
    std::vector<int> ids;
    if (!intrinsics_detect_charuco(gray, corners, ids)) {
        pose_sender_send_control_line("{\"type\":\"HG_CHARUCO_RESULT\",\"ok\":false,\"reason\":\"ChArUco detection error\"}");
        g_hg_charuco_requested = false;
        return;
    }
    std::vector<int> order(ids.size());
    for (size_t i = 0; i < ids.size(); ++i) order[i] = (int) i;
    std::sort(order.begin(), order.end(), [&ids](int a, int b) { return ids[a] < ids[b]; });
    if (order.size() < 18) { // 17 fit points plus at least one held-out point
        char json[160];
        snprintf(json, sizeof(json), "{\"type\":\"HG_CHARUCO_PROGRESS\",\"corners\":%d,\"need\":18}", (int) order.size());
        pose_sender_send_control_line(json);
        return; // keep waiting; the next frame may see more of the board
    }
    CharucoBoardConfig c = intrinsics_get_board_config();
    const int cols = c.squares_x - 1;
    std::vector<cv::Point2f> src, dst;
    for (int n = 0; n < 17; ++n) {
        int id = ids[order[n]];
        cv::Point2f hp;
        if (!homography_prepare_pixel(corners[order[n]].x, corners[order[n]].y, &hp)) {
            pose_sender_send_control_line("{\"type\":\"HG_CHARUCO_RESULT\",\"ok\":false,\"reason\":\"K/dist unavailable for undistort mode\"}");
            g_hg_charuco_requested = false;
            return;
        }
        src.push_back(hp);
        dst.push_back(cv::Point2f((id % cols + 1) * c.square_length_mm,
                                  (id / cols + 1) * c.square_length_mm));
    }
    cv::Mat h = cv::findHomography(src, dst, 0);
    if (h.empty()) {
        pose_sender_send_control_line("{\"type\":\"HG_CHARUCO_RESULT\",\"ok\":false,\"reason\":\"findHomography failed\"}");
        g_hg_charuco_requested = false;
        return;
    }
    double values[9];
    for (int i = 0; i < 9; ++i) values[i] = h.at<double>(i / 3, i % 3);
    if (!homography_set(values)) {
        char json[192];
        snprintf(json, sizeof(json), "{\"type\":\"HG_CHARUCO_RESULT\",\"ok\":false,\"reason\":\"%s\"}", homography_fail_reason());
        pose_sender_send_control_line(json);
        g_hg_charuco_requested = false;
        return;
    }
    double sumSq = 0.0, maxErr = 0.0;
    int validation = 0;
    for (size_t n = 17; n < order.size(); ++n) {
        int id = ids[order[n]];
        cv::Point2f p = corners[order[n]];
        double wx, wy;
        if (!homography_pixel_to_world(p.x, p.y, &wx, &wy)) continue;
        double tx = (id % cols + 1) * c.square_length_mm;
        double ty = (id / cols + 1) * c.square_length_mm;
        double err = hypot(wx - tx, wy - ty);
        sumSq += err * err; if (err > maxErr) maxErr = err; ++validation;
    }
    char json[512];
    snprintf(json, sizeof(json),
             "{\"type\":\"HG_CHARUCO_RESULT\",\"ok\":true,\"fit\":17,\"validation\":%d,\"rmse_mm\":%.3f,\"max_error_mm\":%.3f,\"H\":[%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e]}",
             validation, validation ? sqrt(sumSq / validation) : 0.0, maxErr,
             values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7], values[8]);
    pose_sender_send_control_line(json);
    g_hg_charuco_requested = false;
}
#endif // ENABLE_INTRINSICS_CALIB

// Report the current (runtime-editable) anchor world positions. Used to answer
// both ANCHOR_QUERY and the ACK for an ANCHOR_SET, so the dashboard always
// re-reads the authoritative table from the camera rather than trusting its
// own inputs.
static void send_anchors(void)
{
    AnchorConfig a[HOMOGRAPHY_MAX_ANCHORS];
    int n = homography_get_anchors(a, HOMOGRAPHY_MAX_ANCHORS);
    char json[1024];
    int len = snprintf(json, sizeof(json),
                       "{\"type\":\"CALIB_ANCHORS\",\"count\":%d,\"anchors\":[", n);
    for (int i = 0; i < n && len > 0 && len < (int) sizeof(json) - 64; ++i)
        len += snprintf(json + len, sizeof(json) - len,
                        "%s{\"id\":%d,\"wx\":%.1f,\"wy\":%.1f}",
                        i ? "," : "", a[i].id, a[i].wx, a[i].wy);
    if (len > 0 && len < (int) sizeof(json) - 2)
        snprintf(json + len, sizeof(json) - len, "]}");
    pose_sender_send_control_line(json);
}

static void cmd_anchor_query(void) { send_anchors(); }

// ANCHOR_SET <id> <wx> <wy> — move one anchor's world position (mm) at runtime.
// Carries arguments, so handled specially (not in kCommands), like CALIB_K_SET.
// Always answers with the full table (send_anchors) whether or not the id
// matched, so the UI can tell a rejected id from an applied one.
static bool handle_anchor_set(const char* cmd)
{
    const char* p = strstr(cmd, "ANCHOR_SET");
    if (p == NULL)
        return false;
    int id = -1;
    double wx = 0.0, wy = 0.0;
    if (sscanf(p + 10, "%d %lf %lf", &id, &wx, &wy) == 3) // 10 = strlen("ANCHOR_SET")
        homography_set_anchor(id, wx, wy);
    send_anchors();
    return true;
}

// ANCHOR_SET_SLOT <slot 0..7> <id> <wx> <wy> — replace one complete anchor
// entry. Slots retain the eight-point table while marker ids become editable.
static bool handle_anchor_set_slot(const char* cmd)
{
    const char* p = strstr(cmd, "ANCHOR_SET_SLOT");
    if (p == NULL)
        return false;
    int slot = -1, id = -1;
    double wx = 0.0, wy = 0.0;
    if (sscanf(p + 15, "%d %d %lf %lf", &slot, &id, &wx, &wy) == 4)
        homography_set_anchor_slot(slot, id, wx, wy);
    send_anchors();
    return true;
}

// ANCHOR_SET_ALL <id> <wx> <wy> ... — atomically replace the dynamic anchor
// list. Unlike per-slot updates, this can add or remove anchors in one command.
static bool handle_anchor_set_all(const char* cmd)
{
    const char* p = strstr(cmd, "ANCHOR_SET_ALL");
    if (p == NULL)
        return false;
    p += 14; // strlen("ANCHOR_SET_ALL")
    AnchorConfig a[HOMOGRAPHY_MAX_ANCHORS];
    int n = 0;
    bool valid = true;
    while (n < HOMOGRAPHY_MAX_ANCHORS) {
        char* end = NULL;
        long id = strtol(p, &end, 10);
        if (end == p) break;
        p = end;
        double wx = strtod(p, &end);
        if (end == p) { valid = false; break; }
        p = end;
        double wy = strtod(p, &end);
        if (end == p) { valid = false; break; }
        p = end;
        a[n++] = {(int) id, wx, wy};
    }
    if (*p != '\0') valid = false;
    if (valid) homography_set_anchors(a, n);
    send_anchors();
    return true;
}

#if ENABLE_CENTRAL_TLS_STREAM
// Answers CENTRAL_QUERY and acks CENTRAL_ID / CENTRAL_HMATRIX, so the tab
// always redraws from the camera's own state instead of its own inputs.
static void send_central_status(const char* last_action, const char* detail)
{
    char json[512];
    snprintf(json, sizeof(json),
             "{\"type\":\"CENTRAL_STATUS\",\"link\":\"%s\",\"link_on\":%s,"
             "\"pos_on\":%s,\"marker_id\":%d,"
             "\"server\":\"%s:%d\",\"action\":\"%s\",\"detail\":\"%s\"}",
             central_tls_sender_state(),
             central_tls_sender_enabled() ? "true" : "false",
             g_central_pos_enabled ? "true" : "false", g_central_marker_id,
             CENTRAL_TLS_SERVER_IP, CENTRAL_TLS_SERVER_PORT,
             last_action ? last_action : "", detail ? detail : "");
    pose_sender_send_control_line(json);
}

static void cmd_central_query(void) { send_central_status("query", ""); }

// CENTRAL_ID <n> -- retarget the POS stream without touching either switch.
static bool handle_central_id(const char* cmd)
{
    const char* p = strstr(cmd, "CENTRAL_ID");
    if (p == NULL)
        return false;
    int id = 0;
    if (sscanf(p + 10, "%d", &id) == 1) // 10 = strlen("CENTRAL_ID")
        g_central_marker_id = id;
    send_central_status("id", (id >= 0) ? "applied" : "bad argument");
    return true;
}

// CENTRAL_LINK 0|1 -- drop or restore the TLS session itself.
static bool handle_central_link(const char* cmd)
{
    const char* p = strstr(cmd, "CENTRAL_LINK");
    if (p == NULL)
        return false;
    int on = 0;
    if (sscanf(p + 12, "%d", &on) == 1) { // 12 = strlen("CENTRAL_LINK")
        central_tls_sender_set_enabled(on);
        send_central_status("link", on ? "reconnecting" : "disconnected");
    } else {
        send_central_status("link", "bad argument");
    }
    return true;
}

// CENTRAL_POS 0|1 -- stop sending POS but stay registered with the server.
static bool handle_central_pos(const char* cmd)
{
    const char* p = strstr(cmd, "CENTRAL_POS");
    if (p == NULL)
        return false;
    int on = 0;
    if (sscanf(p + 11, "%d", &on) == 1) { // 11 = strlen("CENTRAL_POS")
        g_central_pos_enabled = (on != 0);
        send_central_status("pos", on ? "streaming" : "stopped");
    } else {
        send_central_status("pos", "bad argument");
    }
    return true;
}

// CENTRAL_HMATRIX <payload json> -- forward a calibration bundle verbatim.
// The dashboard owns the numbers (this app has no solvePnP-derived H_floor /
// H_marker yet), so the payload is passed through untouched rather than being
// re-serialised here. Only the envelope and seq are added by the sender.
static bool handle_central_hmatrix(const char* cmd)
{
    const char* p = strstr(cmd, "CENTRAL_HMATRIX");
    if (p == NULL)
        return false;
    p += 15; // strlen("CENTRAL_HMATRIX")
    while (*p == ' ' || *p == '\t')
        ++p;
    if (*p != '{') {
        send_central_status("hmatrix", "payload must be a JSON object");
        return true;
    }
    int rc = central_tls_sender_send_typed("H_MATRIX", p);
    send_central_status("hmatrix", rc == 0 ? "sent" : "send failed (link down or too long)");
    return true;
}
#endif // ENABLE_CENTRAL_TLS_STREAM

static void send_validation_markers(void)
{
    AnchorConfig a[HOMOGRAPHY_MAX_VALIDATION_MARKERS];
    int n = homography_get_validation_markers(a, HOMOGRAPHY_MAX_VALIDATION_MARKERS);
    char json[1024];
    int len = snprintf(json, sizeof(json),
                       "{\"type\":\"CALIB_VALIDATION\",\"count\":%d,\"markers\":[", n);
    for (int i = 0; i < n && len > 0 && len < (int) sizeof(json) - 64; ++i)
        len += snprintf(json + len, sizeof(json) - len,
                        "%s{\"id\":%d,\"wx\":%.1f,\"wy\":%.1f}",
                        i ? "," : "", a[i].id, a[i].wx, a[i].wy);
    if (len > 0 && len < (int) sizeof(json) - 2)
        snprintf(json + len, sizeof(json) - len, "]}");
    pose_sender_send_control_line(json);
}

static void cmd_validation_query(void) { send_validation_markers(); }

// VALIDATION_SET [id wx wy]... — replace the full independent validation
// list. An empty list disables validation. The mapper rejects duplicate ids
// and ids shared with calculation anchors.
static bool handle_validation_set(const char* cmd)
{
    const char* p = strstr(cmd, "VALIDATION_SET");
    if (p == NULL)
        return false;
    p += 14; // strlen("VALIDATION_SET")
    AnchorConfig a[HOMOGRAPHY_MAX_VALIDATION_MARKERS];
    int n = 0;
    bool valid = true;
    while (n < HOMOGRAPHY_MAX_VALIDATION_MARKERS) {
        char* end = NULL;
        long id = strtol(p, &end, 10);
        if (end == p)
            break;
        p = end;
        double wx = strtod(p, &end);
        if (end == p) { valid = false; break; }
        p = end;
        double wy = strtod(p, &end);
        if (end == p) { valid = false; break; }
        p = end;
        a[n].id = (int) id;
        a[n].wx = wx;
        a[n].wy = wy;
        ++n;
    }
    // Any non-whitespace trailing text, or more than the supported maximum,
    // is malformed rather than silently truncating the operator's list.
    while (*p == ' ' || *p == '\t') ++p;
    if (*p != '\0') valid = false;
    if (valid)
        homography_set_validation_markers(a, n);
    send_validation_markers();
    return true;
}

#if ENABLE_HG_SNAPSHOT
// HG_SNAPSHOT: capture-on-demand floor REFERENCE still for the dashboard's
// homography canvas. The handler only raises a flag; encoding needs this
// frame's NV12 buffer, not in scope at dispatch time, so process_raw_video
// consumes it (take_and_send_hg_reference, defined after the JPEG encoder).
static volatile bool g_hg_snapshot_requested = false;
static void cmd_hg_snapshot(void) { g_hg_snapshot_requested = true; }
#endif
#endif

#if ENABLE_INTRINSICS_CALIB
#if ENABLE_CALIB_VIEW_UPLOAD
// Identifies the current capture session on the wire. Each view uploads THREE
// pieces (plain JPEG, overlay JPEG, corners+K JSON) as separate connections
// arriving seconds apart, so the server needs something to group them by. It
// cannot use its own arrival time (three different stamps for one view) and it
// cannot use the view number alone (that restarts at 1 every session). This
// stamp comes from the camera's own clock, which is NOT synced to the server's
// (README §7) — that is fine, it is used as an opaque id, never as a time.
static long g_calib_session = 0;

// The board as it was when this session STARTED, not as it is at upload time.
//
// intrinsics_set_board_config() refuses to change the board while a session is
// collecting, so every view in a session shares one board -- but a session can
// end (compute) and the board can then be changed BEFORE the operator uploads,
// which would stamp the old views with the new board. Snapshotting at
// CALIB_K_START closes that: the recorded board is always the one the corners
// were actually measured against. Zero until the first CALIB_K_START, which is
// harmless -- with no session there are no stored views to describe.
static CharucoBoardConfig g_calib_board = {};
#endif

static void cmd_calib_k_start(void)
{
#if ENABLE_LDC_CHECK
    if (ldc_check_active()) {
        pose_sender_send_control_line(
            "{\"type\":\"CALIB_K_RESULT\",\"ok\":false,"
            "\"reason\":\"stop LDC check first\"}");
        return;
    }
#endif
#if ENABLE_HOMOGRAPHY
    if (homography_collecting()) {
        pose_sender_send_control_line(
            "{\"type\":\"CALIB_K_RESULT\",\"ok\":false,"
            "\"reason\":\"finish homography calibration first\"}");
        return;
    }
#endif
    intrinsics_start_calib();
#if ENABLE_CALIB_VIEW_UPLOAD
    calib_view_store_reset(); // new session -> drop any views kept from before
    g_calib_session = epoch_ms();
    g_calib_board   = intrinsics_get_board_config();
    intrinsics_set_session_id(g_calib_session); // tag for a later CALIB_K_SAVE
#endif
    CharucoBoardConfig c = intrinsics_get_board_config();
    char json[384];
    snprintf(json, sizeof(json),
             "{\"type\":\"CALIB_K_ACK\",\"state\":\"collecting\","
             "\"target\":%d,\"squares_x\":%d,\"squares_y\":%d,"
             "\"square_mm\":%.3f,\"marker_mm\":%.3f,\"dictionary\":%d,"
             "\"margin_x_mm\":%.3f,\"margin_y_mm\":%.3f}",
             intrinsics_target_views(), c.squares_x, c.squares_y,
             c.square_length_mm, c.marker_length_mm, c.dictionary_id,
             c.outer_margin_x_mm, c.outer_margin_y_mm);
    pose_sender_send_control_line(json);
    debug_message("ArUco: intrinsics calibration started\n");
}

// CALIB_K_CAPTURE / CALIB_K_COMPUTE only raise flags; the actual work needs
// this frame's gray buffer (capture) or blocks for seconds (compute), so both
// are consumed in process_raw_video, not at command-dispatch time.
static volatile bool g_k_capture_requested = false;
static volatile bool g_k_compute_requested = false;

static void cmd_calib_k_capture(void) { g_k_capture_requested = true; }
static void cmd_calib_k_compute(void) { g_k_compute_requested = true; }

#if ENABLE_CALIB_VIEW_UPLOAD
// CALIB_K_UPLOAD likewise only raises a flag here (dispatch runs before the
// upload machinery is defined); the actual send is started from the frame path
// in run_calib_view_upload(). Declared up here so the command table can see it.
static volatile bool g_k_upload_requested = false;
static void cmd_calib_k_upload(void) { g_k_upload_requested = true; }
#endif

static void send_k_status(const char* type, bool ok, const char* reason)
{
    CharucoBoardConfig c = intrinsics_get_board_config();
    char safeReason[160];
    json_escape(reason ? reason : "", safeReason, sizeof(safeReason));
    char json[512];
    snprintf(json, sizeof(json),
             "{\"type\":\"%s\",\"ok\":%s,\"reason\":\"%s\","
             "\"views\":%d,\"target\":%d,\"squares_x\":%d,\"squares_y\":%d,"
             "\"square_mm\":%.3f,\"marker_mm\":%.3f,\"dictionary\":%d,"
             "\"margin_x_mm\":%.3f,\"margin_y_mm\":%.3f,"
             "\"quiet_mm\":%.3f,\"board_w_mm\":%.3f,\"board_h_mm\":%.3f,"
             "\"gates\":%d}",
             type, ok ? "true" : "false", safeReason,
             intrinsics_views(), intrinsics_target_views(),
             c.squares_x, c.squares_y,
             c.square_length_mm, c.marker_length_mm, c.dictionary_id,
             c.outer_margin_x_mm, c.outer_margin_y_mm,
             (c.square_length_mm - c.marker_length_mm) * 0.5f,
             c.squares_x * c.square_length_mm,
             c.squares_y * c.square_length_mm,
             intrinsics_quality_gates() ? 1 : 0);
    pose_sender_send_control_line(json);
}

static void cmd_calib_k_undo(void)
{
    if (intrinsics_undo_last_view()) {
#if ENABLE_CALIB_VIEW_UPLOAD
        calib_view_store_pop_last(); // keep stored JPEGs in step with the views
#endif
        send_k_status("CALIB_K_UNDO", true, "");
    } else {
        send_k_status("CALIB_K_UNDO", false, "no accepted view to remove");
    }
}

static void cmd_calib_k_status(void)
{
    send_k_status("CALIB_K_STATUS", true, "");
}

// CALIB_K_SET <views> <rms> — change the target view count and the overall
// RMS pass limit at runtime (no rebuild). Either arg may be omitted/zero to
// keep its current value, e.g. "CALIB_K_SET 25 0.6" or "CALIB_K_SET 0 1.0".
// Handled specially (not in kCommands) because it carries arguments.
static bool handle_calib_k_set(const char* cmd)
{
    const char* p = strstr(cmd, "CALIB_K_SET");
    if (p == NULL)
        return false;
    int views = 0;
    double rms = 0.0;
    sscanf(p + 11, "%d %lf", &views, &rms); // 11 = strlen("CALIB_K_SET")
    intrinsics_set_params(views, rms);      // ignores non-positive values

    char json[160];
    snprintf(json, sizeof(json),
             "{\"type\":\"CALIB_K_PARAMS\",\"target\":%d,\"rms_limit\":%.3f,"
             "\"views\":%d}",
             intrinsics_target_views(), intrinsics_rms_limit(),
             intrinsics_views());
    pose_sender_send_control_line(json);
    debug_message("ArUco: calib params -> views=%d rms_limit=%.3f\n",
                  intrinsics_target_views(), intrinsics_rms_limit());
    return true;
}

// CALIB_K_GATE <0|1> — turn the calibration quality gates on/off at runtime.
// Handled specially (carries an argument), like CALIB_K_SET.
static bool handle_calib_k_gate(const char* cmd)
{
    const char* p = strstr(cmd, "CALIB_K_GATE");
    if (p == NULL)
        return false;
    int on = 1;
    sscanf(p + 12, "%d", &on); // 12 = strlen("CALIB_K_GATE")
    intrinsics_set_quality_gates(on != 0);

    char json[96];
    snprintf(json, sizeof(json),
             "{\"type\":\"CALIB_K_GATE\",\"enabled\":%d}",
             intrinsics_quality_gates() ? 1 : 0);
    pose_sender_send_control_line(json);
    debug_message("ArUco: quality gates -> %s\n",
                  intrinsics_quality_gates() ? "ON" : "OFF");
    return true;
}

// "RAW_FPS_TEST <0|1>" -- diagnostic mode toggle. When on, process_raw_video
// skips detection entirely and only emits a heartbeat, so seq counts frames
// the SDK DELIVERS instead of frames we manage to process. Runtime rather than
// compile-time (MEASURE_RAW_FPS is just the boot default) because each rebuild
// costs a full package+upload cycle. NOTE: no marker detection happens while
// this is on -- turn it back off after measuring.
static bool g_measure_raw_fps = (MEASURE_RAW_FPS != 0);

static bool handle_raw_fps_test(const char* cmd)
{
    const char* p = strstr(cmd, "RAW_FPS_TEST");
    if (p == NULL)
        return false;
    int on = 1;
    sscanf(p + 12, "%d", &on); // 12 = strlen("RAW_FPS_TEST")
    g_measure_raw_fps = (on != 0);

    char json[96];
    snprintf(json, sizeof(json),
             "{\"type\":\"RAW_FPS_TEST\",\"enabled\":%d}",
             g_measure_raw_fps ? 1 : 0);
    pose_sender_send_control_line(json);
    debug_message("ArUco: raw-fps test mode -> %s\n",
                  g_measure_raw_fps ? "ON (detection skipped)" : "OFF");
    return true;
}

// "ARUCO_SCAN <passes> [win]" -- how many full-frame adaptive-threshold passes
// detectMarkers runs (see ArucoProcessor::setScanPasses). Runtime because the
// speed/robustness trade-off can only be judged against the real scene, and a
// rebuild costs a full package+upload cycle. Compare det + detection rate +
// corner jitter across settings; the pose stream keeps running throughout.
static bool handle_aruco_scan(const char* cmd)
{
    const char* p = strstr(cmd, "ARUCO_SCAN");
    if (p == NULL)
        return false;
    int passes = 3, win = 13;
    sscanf(p + 10, "%d %d", &passes, &win); // 10 = strlen("ARUCO_SCAN")
    if (g_aruco != NULL)
        g_aruco->setScanPasses(passes, win);

    char json[128];
    snprintf(json, sizeof(json),
             "{\"type\":\"ARUCO_SCAN\",\"passes\":%d,\"win\":%d}",
             g_aruco ? g_aruco->scanPasses() : 0,
             g_aruco ? g_aruco->scanWin() : 0);
    pose_sender_send_control_line(json);
    debug_message("ArUco: scan passes -> %d (win %d)\n",
                  g_aruco ? g_aruco->scanPasses() : 0,
                  g_aruco ? g_aruco->scanWin() : 0);
    return true;
}

// "DETECT_PARAM <name> <value>" -- tune one of the four detection-RATE knobs
// (perim/ecc/thresh/poly, see ArucoProcessor::setDetectParam). "DETECT_PARAM"
// with no args just reports the current four values. RAM-only like ARUCO_SCAN:
// a relaxed threshold that survived a reboot would raise false positives with
// nothing on screen to explain it, so these are never persisted -- only the ROI
// is (detect_tuning). Runtime because the right value depends on marker apparent
// size / tilt / lighting and can only be judged by sweeping it against the live
// scene. The reply always carries all four so the dashboard can show the full
// state after any single change.
static void send_detect_params(bool ok, const char* name, const char* reason)
{
    char safeReason[96];
    json_escape(reason ? reason : "", safeReason, sizeof(safeReason));
    char safeName[48];
    json_escape(name ? name : "", safeName, sizeof(safeName));
    char json[320];
    snprintf(json, sizeof(json),
             "{\"type\":\"DETECT_PARAM\",\"ok\":%s,\"name\":\"%s\",\"reason\":\"%s\","
             "\"perim\":%.4f,\"ecc\":%.3f,\"thresh\":%.2f,\"poly\":%.4f}",
             ok ? "true" : "false", safeName, safeReason,
             g_aruco ? g_aruco->minPerimRate()     : 0.0,
             g_aruco ? g_aruco->errCorrRate()      : 0.0,
             g_aruco ? g_aruco->adaptiveThreshC()  : 0.0,
             g_aruco ? g_aruco->polyAccuracyRate() : 0.0);
    pose_sender_send_control_line(json);
}

static bool handle_detect_param(const char* cmd)
{
    const char* p = strstr(cmd, "DETECT_PARAM");
    if (p == NULL)
        return false;
    p += 12;                        // strlen("DETECT_PARAM")

    char name[32] = {0};
    double value = 0.0;
    int n = sscanf(p, "%31s %lf", name, &value);
    if (n < 2) {
        // No name/value -> treat as a query of the current four values.
        send_detect_params(true, "", "");
        return true;
    }
    if (g_aruco == NULL) {
        send_detect_params(false, name, "detector not running");
        return true;
    }
    bool ok = g_aruco->setDetectParam(name, value);
    send_detect_params(ok, name, ok ? "" : "unknown name (use perim|ecc|thresh|poly)");
    debug_message("ArUco: detect param %s -> %.4f (%s)\n",
                  name, value, ok ? "ok" : "rejected");
    return true;
}

// "ROI_SET <x> <y> <w> <h>" -- restrict the marker search to a rectangle;
// "ROI_SET 0 0 0 0" (or any empty rect) restores the full frame. Detection
// cost scales with pixels, so this is the direct lever once the scan-pass
// count is exhausted. Reported corners stay full-frame (see setRoi).
// The operator-set ROI, kept here (not just in ArucoProcessor) because the
// dynamic tracker below overwrites the processor's ROI every frame and needs
// something to fall back to while searching.
static cv::Rect g_manual_roi;

static bool handle_roi_set(const char* cmd)
{
    const char* p = strstr(cmd, "ROI_SET");
    if (p == NULL)
        return false;
    int x = 0, y = 0, w = 0, h = 0;
    sscanf(p + 7, "%d %d %d %d", &x, &y, &w, &h); // 7 = strlen("ROI_SET")
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (w < 0) w = 0;
    if (h < 0) h = 0;
    g_manual_roi = cv::Rect(x, y, w, h);
    if (g_aruco != NULL)
        g_aruco->setRoi(g_manual_roi);

    char json[160];
    cv::Rect r = g_aruco ? g_aruco->roi() : cv::Rect();
    snprintf(json, sizeof(json),
             "{\"type\":\"ROI_SET\",\"x\":%d,\"y\":%d,\"w\":%d,\"h\":%d}",
             r.x, r.y, r.width, r.height);
    pose_sender_send_control_line(json);
    debug_message("ArUco: roi -> %d,%d %dx%d\n", r.x, r.y, r.width, r.height);
    return true;
}

// Report the live ROI/scan settings plus whether they came from a file. The
// "persisted" flag is the point of the message: a saved ROI left over from a
// previous camera position makes markers disappear with no error raised
// anywhere, so the one place it can be noticed is a status readout.
static void send_tuning_state(const char* type, bool ok, const char* reason)
{
    const cv::Rect r = g_aruco ? g_aruco->roi() : cv::Rect();
    char safeReason[160];
    json_escape(reason ? reason : "", safeReason, sizeof(safeReason));
    char json[320];
    snprintf(json, sizeof(json),
             "{\"type\":\"%s\",\"ok\":%s,\"reason\":\"%s\",\"persisted\":%s,"
             "\"roi\":{\"x\":%d,\"y\":%d,\"w\":%d,\"h\":%d},"
             "\"passes\":%d,\"win\":%d}",
             type, ok ? "true" : "false", safeReason,
             detect_tuning_loaded() ? "true" : "false",
             r.x, r.y, r.width, r.height,
             g_aruco ? g_aruco->scanPasses() : 0,
             g_aruco ? g_aruco->scanWin() : 0);
    pose_sender_send_control_line(json);
}

// TUNE_SAVE / TUNE_QUERY / TUNE_CLEAR -- persistence for the fps levers.
// Saving is deliberately a separate step from ROI_SET/ARUCO_SCAN (same split as
// CALIB_K_CONFIG vs CALIB_K_BOARD_SAVE): while hunting for a setting you want
// a power cycle to undo the experiment, not preserve it.
static void cmd_tune_save(void)
{
    bool ok = detect_tuning_save(g_aruco);
    send_tuning_state("TUNE_SAVE", ok, ok ? "" : detect_tuning_fail_reason());
}

static void cmd_tune_query(void)
{
    send_tuning_state("TUNE_QUERY", true, "");
}

// Clears the FILE only; the running detector keeps its current settings. To
// also return the live detector to defaults, follow with
// "ROI_SET 0 0 0 0" + "ARUCO_SCAN 3".
static void cmd_tune_clear(void)
{
    bool ok = detect_tuning_clear();
    send_tuning_state("TUNE_CLEAR", ok, ok ? "" : detect_tuning_fail_reason());
}


static DynRoiTracker g_dynroi;

// "DETECT_ENABLE [0|1]" -- operator switch for marker detection itself. With no
// argument it only reports the current state.
//
// Distinct from RAW_FPS_TEST, which skips detection as a side effect of a
// measurement mode: this one exists to be left off. detectMarkers() is the
// dominant per-frame cost (t_det is most of proc), so turning it off drops the
// camera to decode-only while the link, the command channel and the pose
// heartbeat all stay up.
//
// The heartbeat is deliberately kept: a silent link is indistinguishable from a
// crashed app at the dashboard, and the packet costs nothing next to a scan.
static bool g_detect_enabled = true;

static bool handle_detect_enable(const char* cmd)
{
    const char* p = strstr(cmd, "DETECT_ENABLE");
    if (p == NULL)
        return false;

    int on = g_detect_enabled ? 1 : 0;          // bare command = query
    sscanf(p + 13, "%d", &on);                  // 13 = strlen("DETECT_ENABLE")

    const char* refused = "";
#if ENABLE_HOMOGRAPHY
    // Homography collection consumes detections frame by frame; disabling the
    // detector mid-run would stall it forever with no visible cause. Refusing
    // is the only honest answer -- the operator can stop the run and retry.
    if (on == 0 && homography_collecting()) {
        on = 1;
        refused = "homography collecting";
    }
#endif

    const bool was = g_detect_enabled;
    g_detect_enabled = (on != 0);

    // Re-enabling resumes with whatever box the tracker held when it stopped,
    // which describes a scene from before the pause. Restart it from SEARCH by
    // re-applying its own configuration (configure() always resets).
    if (g_detect_enabled && !was)
        g_dynroi.configure(g_dynroi.enabled(), g_dynroi.margin(),
                           g_dynroi.maxMiss());

    char json[160];
    snprintf(json, sizeof(json),
             "{\"type\":\"DETECT_ENABLE\",\"enabled\":%d,\"refused\":\"%s\"}",
             g_detect_enabled ? 1 : 0, refused);
    pose_sender_send_control_line(json);
    debug_message("ArUco: detection -> %s%s%s\n",
                  g_detect_enabled ? "ON" : "OFF (heartbeat only)",
                  refused[0] ? " refused: " : "", refused);
    return true;
}


// Report the tracker's whole configuration. Shared by DYNROI and DYNROI_IDS so
// the dashboard never has to merge two half-answers -- every ack is complete.
static void report_dynroi(void)
{
    const std::vector<int>& ids = g_dynroi.trackIds();
    char idbuf[128];
    int  n = 0;
    idbuf[0] = '\0';
    for (size_t i = 0; i < ids.size(); ++i) {
        const int w = snprintf(idbuf + n, sizeof(idbuf) - n,
                               (i == 0) ? "%d" : ",%d", ids[i]);
        if (w <= 0 || (size_t) (n + w) >= sizeof(idbuf))
            break;                       // truncate rather than overflow
        n += w;
    }
    char json[256];
    snprintf(json, sizeof(json),
             "{\"type\":\"DYNROI\",\"enabled\":%d,\"margin\":%d,"
             "\"adaptive\":true,\"max_miss\":%d,\"track_ids\":[%s]}",
             g_dynroi.enabled() ? 1 : 0, g_dynroi.margin(), g_dynroi.maxMiss(),
             idbuf);
    pose_sender_send_control_line(json);
}

// "DYNROI_IDS [id ...]" -- restrict tracking to these marker ids.
// No ids at all means "track every marker", which is the default.
//
// MUST be dispatched BEFORE handle_dynroi: that one matches on the substring
// "DYNROI", which this name contains, so the looser handler would swallow this
// command and parse "_IDS ..." as its own numeric arguments.
static bool handle_dynroi_ids(const char* cmd)
{
    const char* p = strstr(cmd, "DYNROI_IDS");
    if (p == NULL)
        return false;
    p += 10;                             // 10 = strlen("DYNROI_IDS")

    std::vector<int> ids;
    while (*p != '\0' && ids.size() < DYNROI_MAX_TRACK_IDS) {
        char* end = NULL;
        const long v = strtol(p, &end, 10);
        if (end == p)                    // no more numbers
            break;
        // Ignore duplicates so the ack reflects what is actually being matched.
        bool dup = false;
        for (size_t i = 0; i < ids.size(); ++i)
            if (ids[i] == (int) v) { dup = true; break; }
        if (!dup && v >= 0)
            ids.push_back((int) v);
        p = end;
    }

    g_dynroi.setTrackIds(ids);
    // The box was just reset to SEARCH, so hand the ROI back to the operator's
    // manual rect until the filter locks on again. Without this the detector
    // keeps scanning the last tracked box while the tracker thinks it is
    // searching, and a filter that matches nothing would never recover.
    if (g_aruco != NULL)
        g_aruco->setRoi(g_manual_roi);

    report_dynroi();
    debug_message("ArUco: dynroi track ids = %d\n", (int) ids.size());
    return true;
}

// "DYNROI <0|1> [maxMargin] [maxMiss]" -- toggle/tune the tracker at runtime.
static bool handle_dynroi(const char* cmd)
{
    const char* p = strstr(cmd, "DYNROI");
    if (p == NULL)
        return false;
    int on = 0, margin = g_dynroi.margin(), maxMiss = g_dynroi.maxMiss();
    sscanf(p + 6, "%d %d %d", &on, &margin, &maxMiss); // 6 = strlen("DYNROI")
    if (margin  < 0)   margin  = 0;
    if (margin  > 960) margin  = 960;
    if (maxMiss < 0)   maxMiss = 0;
    if (maxMiss > 60)  maxMiss = 60;

    if (!g_dynroi.configure(on != 0, margin, maxMiss) && g_aruco != NULL)
        g_aruco->setRoi(g_manual_roi);   // hand the ROI back to the operator

    report_dynroi();
    debug_message("ArUco: dynroi %s margin=%d maxMiss=%d\n",
                  g_dynroi.enabled() ? "ON" : "OFF", g_dynroi.margin(),
                  g_dynroi.maxMiss());
    return true;
}

// "IMAGE_QUERY" -- probe whether the image-attribute API is usable on this
// model. READ-ONLY: nothing is changed.
//
// The SDK doc puts SSDR / white balance / backlight / exposure / focus / IR /
// day-night under opensdk_getPtzAttr, but that call is shaped for PTZ presets
// (it takes a preset index) and this is a fixed bullet camera with motorised
// zoom only -- no pan/tilt, so no presets. Whether it answers at all is
// therefore unverified from the documentation. This reports the raw error code
// so one call settles it, the same way GET_RAW_RES settled the resolution list.
static bool handle_image_query(const char* cmd)
{
    if (strstr(cmd, "IMAGE_QUERY") == NULL)
        return false;

    OPENSDK_PTZ_PRESET_IMAGE img;
    memset(&img, 0, sizeof(img));
    OPENSDK_ERR_CODE err =
        OPENSDK::PTZ::opensdk_getPtzAttr(0, &img, (INT32_N) sizeof(img));

    char json[192];
    snprintf(json, sizeof(json),
             "{\"type\":\"IMAGE_QUERY\",\"err\":%d,\"ok\":%s,\"struct_bytes\":%d}",
             (int) err, (err == OPENSDK_APP_OK) ? "true" : "false",
             (int) sizeof(img));
    pose_sender_send_control_line(json);
    debug_message("ArUco: getPtzAttr err=%d size=%d\n",
                  (int) err, (int) sizeof(img));
    return true;
}

// "K_LOAD fx fy cx cy d0 d1 d2 d3 d4" -- inject externally-computed intrinsics
// (a calibration done off-camera and pasted into the dashboard) without a
// capture session. Applied immediately; replies CALIB_K_RESULT ok/fail so the
// dashboard can echo the accepted values or the parse/validation error.
static bool handle_k_load(const char* cmd)
{
    const char* p = strstr(cmd, "K_LOAD");
    if (p == NULL)
        return false;
    double fx = 0, fy = 0, cx = 0, cy = 0, d[5] = {0, 0, 0, 0, 0};
    int n = sscanf(p + 6, "%lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   &fx, &fy, &cx, &cy, &d[0], &d[1], &d[2], &d[3], &d[4]);
    if (n != 9) {
        pose_sender_send_control_line(
            "{\"type\":\"CALIB_K_RESULT\",\"ok\":false,"
            "\"reason\":\"need 9 numbers: fx fy cx cy d0 d1 d2 d3 d4\"}");
        return true;
    }
    if (!intrinsics_load_values(fx, fy, cx, cy, d)) {
        pose_sender_send_control_line(
            "{\"type\":\"CALIB_K_RESULT\",\"ok\":false,"
            "\"reason\":\"invalid intrinsics (fx/fy>0, cx/cy>=0, finite dist)\"}");
        return true;
    }
    char json[512];
    snprintf(json, sizeof(json),
             "{\"type\":\"CALIB_K_RESULT\",\"ok\":true,\"rms\":-1,"
             "\"loaded\":\"external\","
             "\"fx\":%.4f,\"fy\":%.4f,\"cx\":%.4f,\"cy\":%.4f,"
             "\"dist\":[%.6f,%.6f,%.6f,%.6f,%.6f]}",
             fx, fy, cx, cy, d[0], d[1], d[2], d[3], d[4]);
    pose_sender_send_control_line(json);
    debug_message("ArUco: intrinsics loaded externally (fx=%.2f fy=%.2f)\n",
                  fx, fy);
    return true;
}

static bool handle_k_config_command(const char* cmd)
{
    if (strstr(cmd, "CALIB_K_CONFIG") == NULL)
        return false;

    CharucoBoardConfig c;
    int matched = sscanf(cmd, "CALIB_K_CONFIG %d %d %f %f %d %f %f",
                         &c.squares_x, &c.squares_y,
                         &c.square_length_mm, &c.marker_length_mm,
                         &c.dictionary_id,
                         &c.outer_margin_x_mm, &c.outer_margin_y_mm);
    if (matched != 7) {
        send_k_status("CALIB_K_CONFIG", false,
                      "expected: sx sy square_mm marker_mm dict margin_x margin_y");
        return true;
    }

    const char* reason = NULL;
    if (!intrinsics_set_board_config(c, &reason))
        send_k_status("CALIB_K_CONFIG", false, reason);
    else
        send_k_status("CALIB_K_CONFIG", true, "");
    return true;
}

// Read back whatever K/dist is CURRENTLY loaded (from a prior CALIB_K_COMPUTE
// this run, or auto-loaded from /mnt/camera_intrinsics.txt at startup) —
// no capture/compute needed, so this answers immediately from the command
// handler itself rather than being deferred to the next frame.
static void cmd_calib_k_query(void)
{
    double fx, fy, cx, cy, dist[5];
    if (!intrinsics_get(&fx, &fy, &cx, &cy, dist)) {
        pose_sender_send_control_line(
            "{\"type\":\"CALIB_K_QUERY\",\"available\":false}");
        return;
    }
    char json[288];
    snprintf(json, sizeof(json),
             "{\"type\":\"CALIB_K_QUERY\",\"available\":true,"
             "\"fx\":%.2f,\"fy\":%.2f,\"cx\":%.2f,\"cy\":%.2f,"
             "\"dist\":[%.6f,%.6f,%.6f,%.6f,%.6f],\"session\":%ld,\"profile\":\"%s\"}",
             fx, fy, cx, cy, dist[0], dist[1], dist[2], dist[3], dist[4],
             intrinsics_session_id(), intrinsics_active_profile());
    pose_sender_send_control_line(json);
}

// CALIB_K_SAVE — persist whatever K/dist is currently loaded to PERSIST_DIR
// right now. No frame data involved (unlike CAPTURE/COMPUTE), so this answers
// synchronously from the command handler, same as CALIB_K_QUERY above.
static void cmd_calib_k_save(void)
{
    bool ok = intrinsics_save();
    char safeReason[160];
    json_escape(ok ? "" : intrinsics_fail_reason(), safeReason, sizeof(safeReason));
    char json[256];
    snprintf(json, sizeof(json),
             "{\"type\":\"CALIB_K_SAVE\",\"ok\":%s,\"reason\":\"%s\",\"session\":%ld}",
             ok ? "true" : "false", safeReason, intrinsics_session_id());
    pose_sender_send_control_line(json);
}

static void send_k_profiles(void)
{
    IntrinsicsProfileInfo profiles[16];
    int count = intrinsics_list_profiles(profiles, 16);
    char json[1024]; int used = snprintf(json, sizeof(json),
        "{\"type\":\"CALIB_K_PROFILES\",\"active\":\"%s\",\"profiles\":[",
        intrinsics_active_profile());
    for (int i = 0; i < count && used > 0 && used < (int)sizeof(json) - 48; ++i)
        used += snprintf(json + used, sizeof(json) - used, "%s\"%s\"",
                         i ? "," : "", profiles[i].name);
    snprintf(json + used, sizeof(json) - used, "]}");
    pose_sender_send_control_line(json);
}

static bool handle_k_profile_command(const char* cmd)
{
    if (strstr(cmd, "CALIB_K_PROFILE_LIST") != NULL) { send_k_profiles(); return true; }
    const char* save = strstr(cmd, "CALIB_K_PROFILE_SAVE ");
    const char* load = strstr(cmd, "CALIB_K_PROFILE_LOAD ");
    const char* name = save ? save + 21 : (load ? load + 21 : NULL);
    if (!name) return false;
    char profile[32] = {0};
    if (sscanf(name, "%31s", profile) != 1) return true;
    // "persisted" = the mirror into camera_intrinsics.txt landed, i.e. this
    // profile is now also what the app boots with. False means it is applied
    // for this session only, so the dashboard must say so.
    bool persisted = false;
    bool ok = save ? intrinsics_save_profile(profile, &persisted)
                   : intrinsics_load_profile(profile, &persisted);
    char json[256];
    snprintf(json, sizeof(json), "{\"type\":\"CALIB_K_PROFILE\",\"action\":\"%s\",\"ok\":%s,\"name\":\"%s\",\"active\":\"%s\",\"persisted\":%s}",
             save ? "save" : "load", ok ? "true" : "false", profile,
             intrinsics_active_profile(), persisted ? "true" : "false");
    pose_sender_send_control_line(json);
    if (ok) send_k_profiles();
    return true;
}

// CALIB_K_BOARD_SAVE — persist the currently active ChArUco board config to
// PERSIST_DIR right now. CALIB_K_CONFIG only applies it in RAM; this is the
// separate, explicit persistence step.
static void cmd_calib_k_board_save(void)
{
    bool ok = intrinsics_save_board_config();
    send_k_status("CALIB_K_BOARD_SAVE", ok, ok ? "" : intrinsics_fail_reason());
}
#endif

#if ENABLE_LDC_CHECK
static void cmd_ldc_check_start(void)
{
#if ENABLE_INTRINSICS_CALIB
    if (intrinsics_collecting()) {
        pose_sender_send_control_line(
            "{\"type\":\"LDC_CHECK_ACK\",\"state\":\"rejected\","
            "\"reason\":\"finish intrinsics calibration first\"}");
        return;
    }
#endif
#if ENABLE_HOMOGRAPHY
    if (homography_collecting()) {
        pose_sender_send_control_line(
            "{\"type\":\"LDC_CHECK_ACK\",\"state\":\"rejected\","
            "\"reason\":\"finish homography calibration first\"}");
        return;
    }
#endif
    ldc_check_start();
    pose_sender_send_control_line(
        "{\"type\":\"LDC_CHECK_ACK\",\"state\":\"checking\"}");
    debug_message("ArUco: LDC check started\n");
}

static void cmd_ldc_check_stop(void)
{
    ldc_check_stop();
    pose_sender_send_control_line(
        "{\"type\":\"LDC_CHECK_ACK\",\"state\":\"stopped\"}");
    debug_message("ArUco: LDC check stopped\n");
}

// LDC_SNAPSHOT: capture-on-demand. The command handler only sets a flag —
// the actual frame/image work needs this frame's NV12 buffer, which isn't
// available at command-dispatch time, so process_raw_video consumes the
// flag once it has that data (see g_snapshot_requested below).
static volatile bool g_snapshot_requested = false;

static void cmd_ldc_snapshot(void)
{
    g_snapshot_requested = true;
}
#endif

// GET_RAW_RES: ask the SDK which video resolutions it actually supports.
// Real signature confirmed from opensdk_profile.h (both params documented
// [OUT]; array size is the FIXED macro OPENSDK_MAX_RESOLUTION_CNT baked into
// the signature itself, not a caller-supplied capacity):
//   OPENSDK::PROFILE::opensdk_getSupportResolution(
//       OPENSDK_RESOLUTION* vidRes[OPENSDK_MAX_RESOLUTION_CNT],
//       INT32_N* resolutionCnt);
// vidRes wants an array of POINTERS (not structs) — each slot must point to
// storage we own, hence resStorage[] + pointing vidRes[i] at it.
static void cmd_get_raw_res(void)
{
    OPENSDK_RESOLUTION resStorage[OPENSDK_MAX_RESOLUTION_CNT];
    OPENSDK_RESOLUTION* vidRes[OPENSDK_MAX_RESOLUTION_CNT];
    for (int i = 0; i < OPENSDK_MAX_RESOLUTION_CNT; ++i)
        vidRes[i] = &resStorage[i];

    INT32_N resolutionCnt = 0;
    OPENSDK_ERR_CODE err =
        OPENSDK::PROFILE::opensdk_getSupportResolution(vidRes, &resolutionCnt);

    char json[512];
    int len = snprintf(json, sizeof(json),
             "{\"type\":\"RAW_RES\",\"err\":%d,\"count\":%d,\"list\":[",
             (int) err, (int) resolutionCnt);
    int n = (int) resolutionCnt;
    if (n > OPENSDK_MAX_RESOLUTION_CNT) n = OPENSDK_MAX_RESOLUTION_CNT; // guard
    if (n < 0) n = 0;
    for (int i = 0; i < n && len > 0 && len < (int) sizeof(json) - 24; ++i) {
        len += snprintf(json + len, sizeof(json) - len, "%s[%d,%d]",
                        i ? "," : "", vidRes[i]->width, vidRes[i]->height);
    }
    if (len > 0 && len < (int) sizeof(json) - 2)
        snprintf(json + len, sizeof(json) - len, "]}");
    pose_sender_send_control_line(json);
    debug_message("ArUco: getSupportResolution err=%d count=%d\n",
                  (int) err, (int) resolutionCnt);
}

static const struct { const char* name; CommandHandler fn; } kCommands[] = {
#if ENABLE_INTRINSICS_CALIB
    // Keep more specific names before shorter ones (substring matching):
    // CAPTURE/COMPUTE/START all share the "CALIB_K_" prefix, so the full
    // names must appear before any shorter one would.
    { "CALIB_K_CAPTURE", cmd_calib_k_capture },
    { "CALIB_K_COMPUTE", cmd_calib_k_compute },
    { "CALIB_K_UNDO", cmd_calib_k_undo },
    { "CALIB_K_STATUS", cmd_calib_k_status },
    { "CALIB_K_QUERY", cmd_calib_k_query },
    { "CALIB_K_SAVE", cmd_calib_k_save },
    { "CALIB_K_BOARD_SAVE", cmd_calib_k_board_save },
    { "CALIB_K_START", cmd_calib_k_start },
#if ENABLE_CALIB_VIEW_UPLOAD
    { "CALIB_K_UPLOAD", cmd_calib_k_upload },
#endif
#endif
#if ENABLE_LDC_CHECK
    // STOP before START is not required (neither is a substring of the other),
    // but both must precede any shorter "LDC_CHECK" prefix if one is ever added.
    { "LDC_CHECK_START", cmd_ldc_check_start },
    { "LDC_CHECK_STOP", cmd_ldc_check_stop },
    { "LDC_SNAPSHOT", cmd_ldc_snapshot },
#endif
#if ENABLE_HOMOGRAPHY
    { "CALIB_START", cmd_calib_start },
    // HG_QUERY / HG_SAVE / HG_SNAPSHOT / ANCHOR_QUERY: none is a substring of
    // another, and ANCHOR_SET is intercepted before this table (it carries
    // arguments).
    { "HG_QUERY", cmd_hg_query },
#if ENABLE_INTRINSICS_CALIB
    { "HG_CHARUCO_START", cmd_hg_charuco_start },
#endif
    { "HG_SAVE", cmd_hg_save },
#if ENABLE_HG_SNAPSHOT
    { "HG_SNAPSHOT", cmd_hg_snapshot },
#endif
    { "ANCHOR_QUERY", cmd_anchor_query },
    { "VALIDATION_QUERY", cmd_validation_query },
    // MARKER_PLANE_QUERY / MARKER_PLANE_SAVE are exact matches; MARKER_HEIGHT
    // carries an argument and is intercepted before this table.
    { "MARKER_PLANE_QUERY", cmd_marker_plane_query },
    { "MARKER_PLANE_SAVE", cmd_marker_plane_save },
#endif
    // TUNE_SAVE / TUNE_QUERY / TUNE_CLEAR: distinct names, none a substring of
    // another, and all argument-free so the table can own them.
#if ENABLE_CENTRAL_TLS_STREAM
    { "CENTRAL_QUERY", cmd_central_query },
#endif
    { "TUNE_SAVE", cmd_tune_save },
    { "TUNE_QUERY", cmd_tune_query },
    { "TUNE_CLEAR", cmd_tune_clear },
    { "GET_RAW_RES", cmd_get_raw_res },
    { NULL, NULL } // sentinel
};

#if ENABLE_SHELL_CMD
// "SHELL <cmd>" -- run <cmd> via /bin/sh, stream stdout+stderr back line by
// line. Diagnostic only; see the ENABLE_SHELL_CMD warning in app_config.h.
//
// One JSON line per output line rather than one blob, because json_escape()
// flattens control characters to spaces -- a multi-line blob would arrive as
// one unreadable run-on. Per-line also lets the dashboard render it as a
// terminal transcript as it arrives.
static bool handle_shell_command(const char* cmd)
{
    const char* p = strstr(cmd, "SHELL ");
    if (p == NULL)
        return false;
    p += 6;                         // strlen("SHELL ")
    while (*p == ' ')
        ++p;
    if (*p == '\0')
        return true;                // "SHELL" with no command: ignore

    char raw[256];
    char esc[520];                  // json_escape can double the length
    char json[640];

    json_escape(p, esc, sizeof(esc));
    snprintf(json, sizeof(json),
             "{\"type\":\"SHELL\",\"stream\":\"start\",\"cmd\":\"%s\"}", esc);
    pose_sender_send_control_line(json);

    // 2>&1 so a failing command reports WHY (the error text is the whole point
    // of running it here) instead of silently producing nothing.
    char line[320];
    snprintf(line, sizeof(line), "%s 2>&1", p);
    FILE* f = popen(line, "r");
    if (f == NULL) {
        pose_sender_send_control_line(
            "{\"type\":\"SHELL\",\"stream\":\"end\",\"exit\":-1,\"lines\":0}");
        return true;
    }

    int lines = 0;
    bool truncated = false;
    while (fgets(raw, sizeof(raw), f) != NULL) {
        if (lines >= SHELL_MAX_LINES) {
            truncated = true;
            break;                  // pclose() below drops the pipe; the child
        }                           // then dies on SIGPIPE at its next write
        size_t n = strlen(raw);
        while (n > 0 && (raw[n - 1] == '\n' || raw[n - 1] == '\r'))
            raw[--n] = '\0';
        json_escape(raw, esc, sizeof(esc));
        snprintf(json, sizeof(json),
                 "{\"type\":\"SHELL\",\"stream\":\"out\",\"line\":\"%s\"}", esc);
        pose_sender_send_control_line(json);
        ++lines;
    }
    int rc = pclose(f);

    // WEXITSTATUS is only meaningful once WIFEXITED says the child exited
    // normally -- which it did NOT when we broke out early above and it died on
    // SIGPIPE. Report -1 for "killed / unknown" rather than a garbage code.
    int exit_code = -1;
    if (rc != -1 && WIFEXITED(rc))
        exit_code = WEXITSTATUS(rc);

    snprintf(json, sizeof(json),
             "{\"type\":\"SHELL\",\"stream\":\"end\",\"exit\":%d,\"lines\":%d,"
             "\"truncated\":%s}",
             exit_code, lines, truncated ? "true" : "false");
    pose_sender_send_control_line(json);
    return true;
}
#endif // ENABLE_SHELL_CMD

static void handle_server_commands(void)
{
    // Sized for CENTRAL_HMATRIX, whose argument is a whole calibration bundle
    // (K, D and two 3x3 matrices) -- an order of magnitude past every other
    // command here. pose_sender's own line buffer has to match.
    char cmd[CENTRAL_TLS_MAX_LINE];
#if ENABLE_CENTRAL_TLS_STREAM && ENABLE_HOMOGRAPHY
    // The central protocol only grants this one camera command for now.
    // web_gui remains the complete calibration command channel.
    while (central_tls_sender_poll_command(cmd, sizeof(cmd))) {
        if (strcmp(cmd, "CALIB_START") == 0)
            cmd_calib_start();
    }
#endif
    while (pose_sender_poll_command(cmd, sizeof(cmd))) {
#if ENABLE_SHELL_CMD
        // Before the table: the table matches by substring, so a command like
        // "SHELL grep CALIB_START x" would otherwise trigger cmd_calib_start.
        if (handle_shell_command(cmd))
            continue;
#endif
#if ENABLE_CENTRAL_TLS_STREAM
        // Both carry arguments, so they are intercepted like ANCHOR_SET. Also
        // note CENTRAL_HMATRIX's payload can contain the word CALIB_START in a
        // string and must never reach the substring table.
        if (handle_central_hmatrix(cmd))
            continue;
        if (handle_central_link(cmd))
            continue;
        if (handle_central_pos(cmd))
            continue;
        if (handle_central_id(cmd))
            continue;
#endif
#if ENABLE_INTRINSICS_CALIB
        if (handle_k_profile_command(cmd))
            continue;
        if (handle_k_config_command(cmd))
            continue;
        if (handle_calib_k_set(cmd))
            continue;
        if (handle_raw_fps_test(cmd))
            continue;
        if (handle_aruco_scan(cmd))
            continue;
        // "DETECT_ENABLE" and "DETECT_PARAM" share only the "DETECT_" prefix,
        // so neither matcher can swallow the other; order here is cosmetic.
        if (handle_detect_enable(cmd))
            continue;
        if (handle_detect_param(cmd))
            continue;
        if (handle_roi_set(cmd))
            continue;
        // Before handle_dynroi: "DYNROI" is a substring of "DYNROI_IDS", so the
        // looser matcher would claim this command first.
        if (handle_dynroi_ids(cmd))
            continue;
        if (handle_dynroi(cmd))
            continue;
        if (handle_k_load(cmd))
            continue;
        if (handle_image_query(cmd))
            continue;
        if (handle_calib_k_gate(cmd))
            continue;
#endif
#if ENABLE_HOMOGRAPHY
        // Carries arguments, so these cannot live in the substring table.
        // More-specific anchor commands precede ANCHOR_SET because their names
        // contain that substring.
        if (handle_anchor_set_all(cmd))
            continue;
        if (handle_anchor_set_slot(cmd))
            continue;
        if (handle_hg_coord_mode(cmd))
            continue;
        // Before the exact-match table: both take an argument, and neither
        // name is a prefix of anything else here.
        if (handle_marker_height(cmd))
            continue;
        if (handle_camera_height(cmd))
            continue;
        if (handle_anchor_set(cmd))
            continue;
        if (handle_validation_set(cmd))
            continue;
        if (handle_hg_set(cmd))
            continue;
#endif
        for (int i = 0; kCommands[i].name != NULL; ++i) {
            if (strstr(cmd, kCommands[i].name) != NULL) {
                kCommands[i].fn();
                break;
            }
        }
    }
}

#if ENABLE_INTRINSICS_CALIB || ENABLE_LDC_CHECK
// Shared helper: convert an NV12 frame to color, draw the detected marker
// overlay (plus optional ChArUco corner dots, color-coded edge/center — see
// ldc_checker.h), and upload it (as raw RGB) together with `json` over the
// dedicated snapshot channel. Used by both LDC_SNAPSHOT and CALIB_K_CAPTURE
// so a saved image can be inspected on the server later. Returns 0 on
// success. `corners`/`isEdge` are optional (pass NULL to skip the dots) and,
// if given, must be the same length — they come from LdcResult and are RAW
// (pre-undistort) positions, so they line up with this (undistorted-frame-
// unaware) raw image.
static int send_annotated_image(const uint8_t* nv12, int width, int height,
                                const char* json, int json_len,
                                const std::vector<cv::Point2f>* corners = NULL,
                                const std::vector<bool>* isEdge = NULL)
{
    cv::Mat bgr;
    if (!ArucoProcessor::nv12ToBgr(nv12, width, height, bgr))
        return -1;
    if (g_aruco != NULL)
        g_aruco->process(bgr); // draw marker outlines + axis arrows in place

    if (corners != NULL && isEdge != NULL && corners->size() == isEdge->size()) {
        for (size_t i = 0; i < corners->size(); ++i) {
            // Edge region (near frame border, where distortion is worst) in
            // red; center region in green — matches edge_max_px/center_max_px.
            cv::Scalar color = (*isEdge)[i] ? cv::Scalar(0, 0, 255)
                                            : cv::Scalar(0, 255, 0);
            cv::circle(bgr, (*corners)[i], 5, color, cv::FILLED);
            cv::circle(bgr, (*corners)[i], 5, cv::Scalar(0, 0, 0), 1); // outline
        }
    }

    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    if (!rgb.isContinuous())
        rgb = rgb.clone();

    int rgbLen = (int) (rgb.total() * rgb.elemSize());
    return snapshot_sender_send(json, json_len, rgb.cols, rgb.rows,
                                rgb.data, rgbLen);
}
#endif

#if NEED_JPEG_ENCODER
// --- Shared JPEG encoder (TooJpeg) -----------------------------------------
// Used by stored calibration views (CALIB_K_UPLOAD) and the homography
// reference still (HG_SNAPSHOT). TooJpeg emits the compressed stream one byte
// at a time through a plain function pointer (no user-data arg), so the
// destination buffer is a file-scope pointer set just around each writeJpeg()
// call. Encoding only ever runs on the frame-callback thread (one image at a
// time), so this is safe.
static std::vector<uint8_t>* g_jpegSink = NULL;
static void jpeg_sink_byte(unsigned char b)
{
    if (g_jpegSink != NULL)
        g_jpegSink->push_back(b);
}

// JPEG-encode an RGB Mat into `out`. Returns false if the encoder refused or
// produced nothing.
static bool encode_rgb_jpeg(const cv::Mat& rgb, std::vector<uint8_t>& out)
{
    out.clear();
    out.reserve(256 * 1024);
    g_jpegSink = &out;
    bool ok = TooJpeg::writeJpeg(jpeg_sink_byte, rgb.data,
                                 (unsigned short) rgb.cols,
                                 (unsigned short) rgb.rows,
                                 true, CALIB_VIEW_JPEG_QUALITY, false);
    g_jpegSink = NULL;
    return ok && !out.empty();
}
#endif // NEED_JPEG_ENCODER

#if ENABLE_INTRINSICS_CALIB
// Report a calibration outcome (IC_DONE_OK / IC_DONE_FAIL) to the server.
static void report_k_result(IntrCalibState st)
{
    char json[512];
    if (st == IC_DONE_OK) {
        double fx, fy, cx, cy, d[5];
        intrinsics_get(&fx, &fy, &cx, &cy, d);
        snprintf(json, sizeof(json),
                 "{\"type\":\"CALIB_K_RESULT\",\"ok\":true,\"rms\":%.3f,"
                 "\"views\":%d,\"pruned\":%d,"
                 "\"fx\":%.2f,\"fy\":%.2f,\"cx\":%.2f,\"cy\":%.2f,"
                 "\"dist\":[%.6f,%.6f,%.6f,%.6f,%.6f]}",
                 intrinsics_rms(), intrinsics_views(),
                 intrinsics_pruned_views(), fx, fy, cx, cy,
                 d[0], d[1], d[2], d[3], d[4]);
        pose_sender_send_control_line(json);
        debug_message("ArUco: intrinsics calibration OK (rms=%.3f)\n",
                      intrinsics_rms());
    } else if (st == IC_DONE_FAIL) {
        char safeReason[160];
        json_escape(intrinsics_fail_reason(), safeReason, sizeof(safeReason));
        snprintf(json, sizeof(json),
                 "{\"type\":\"CALIB_K_RESULT\",\"ok\":false,\"reason\":\"%s\"}",
                 safeReason);
        pose_sender_send_control_line(json);
        debug_message("ArUco: intrinsics calibration FAILED: %s\n",
                      intrinsics_fail_reason());
    }
}

#if ENABLE_CALIB_VIEW_UPLOAD
// --- Calibration-view JPEG capture + deferred upload -----------------------
// At capture we only ENCODE (fast, kept in memory); the multi-image upload is
// deferred to an explicit CALIB_K_UPLOAD and runs off the frame thread. The
// JPEG encoder itself (encode_rgb_jpeg) is shared — see NEED_JPEG_ENCODER above.

// Encode THIS frame TWICE -- once untouched, once with the overlay drawn -- and
// stash both, plus the corner list, for a later CALIB_K_UPLOAD.
//
// The plain encode must happen BEFORE any drawing, because g_aruco->process()
// and the corner rings below mutate `bgr` in place. It is not a duplicate of
// the overlay image: the overlay burns rings over the marker bits, so only the
// plain one can be re-detected offline, while only the overlay one shows a
// human what the camera actually matched.
static void encode_and_store_calib_view(const uint8_t* nv12, int width,
                                        int height, int view, int target,
                                        int corners)
{
    cv::Mat bgr;
    if (!ArucoProcessor::nv12ToBgr(nv12, width, height, bgr))
        return;

    std::vector<uint8_t> jpegPlain;
    {
        cv::Mat rgbPlain;
        cv::cvtColor(bgr, rgbPlain, cv::COLOR_BGR2RGB);
        if (!rgbPlain.isContinuous())
            rgbPlain = rgbPlain.clone();
        if (!encode_rgb_jpeg(rgbPlain, jpegPlain))
            return;
    }   // free the ~6MB RGB copy before making the second one

    if (g_aruco != NULL)
        g_aruco->process(bgr); // draw marker outlines + axis arrows in place

    // Mark the ChArUco interior corners (the chessboard corners actually used
    // for K/dist, e.g. 6x4=24 on a 7x5 board) with a ring + dot so the stored
    // view shows exactly which points fed the calibration. The same points are
    // kept as data (with their ids) and shipped by CALIB_K_UPLOAD -- the drawn
    // rings are for a human, the numbers are what the server can actually
    // re-fit.
    std::vector<cv::Point2f> ccorners;
    std::vector<int>         cids;
    std::vector<CalibViewCorner> points;
    if (intrinsics_last_view_corners(ccorners, &cids)) {
        points.reserve(ccorners.size());
        for (size_t i = 0; i < ccorners.size(); ++i) {
            cv::circle(bgr, ccorners[i], 6, cv::Scalar(0, 255, 255), 2); // yellow ring
            cv::circle(bgr, ccorners[i], 2, cv::Scalar(0, 0, 255), cv::FILLED); // red dot
            CalibViewCorner c;
            c.x  = ccorners[i].x;
            c.y  = ccorners[i].y;
            c.id = (i < cids.size()) ? cids[i] : -1;
            points.push_back(c);
        }
    }

    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    if (!rgb.isContinuous())
        rgb = rgb.clone();

    std::vector<uint8_t> jpeg;
    if (!encode_rgb_jpeg(rgb, jpeg))
        return;

    calib_view_store_add(view, target, corners, width, height,
                         std::move(points), std::move(jpeg),
                         std::move(jpegPlain));
}

// CALIB_K_UPLOAD: ship every stored view to the vision server over the
// snapshot channel, on a detached worker thread so the transfer never blocks
// the frame path. g_uploading guards against a second concurrent request.
// (g_k_upload_requested / cmd_calib_k_upload are declared earlier, above the
// command table.)
static std::atomic<bool> g_uploading(false);

// The K/dist CURRENTLY loaded, as a JSON fragment to embed in every uploaded
// view. Without it the corner lists are unusable as a record: an offline re-fit
// has nothing to compare against, and months later nobody can tell whether a
// view predates or postdates the calibration it is supposed to justify. When
// nothing is loaded that fact is stated explicitly rather than omitted, so a
// missing field always means an old camera build, never "we forgot".
static std::string intrinsics_json(bool* available)
{
    double fx, fy, cx, cy, d[5];
    if (!intrinsics_get(&fx, &fy, &cx, &cy, d)) {
        if (available != NULL)
            *available = false;
        return std::string("\"intrinsics\":{\"available\":false}");
    }
    if (available != NULL)
        *available = true;
    char buf[288];
    snprintf(buf, sizeof(buf),
             "\"intrinsics\":{\"available\":true,\"fx\":%.4f,\"fy\":%.4f,"
             "\"cx\":%.4f,\"cy\":%.4f,"
             "\"dist\":[%.8f,%.8f,%.8f,%.8f,%.8f]}",
             fx, fy, cx, cy, d[0], d[1], d[2], d[3], d[4]);
    return std::string(buf);
}

// The board the corners were measured against, as a JSON fragment.
//
// Without it a stored view cannot be re-fitted: turning a corner id back into a
// board coordinate needs the row width (interior corners are squares_x-1 wide,
// so id -> col = id % (squares_x-1), row = id / (squares_x-1)) and the scale
// (square_mm). The camera knows both; the file did not say.
//
// It has to travel WITH each view rather than be assumed by the reader, because
// the board is changeable at runtime (CALIB_K_CONFIG from the UI) and is NOT
// persisted anywhere while PERSIST_TO_MNT is 0 -- the camera itself forgets it
// on restart, and the web form is just an input box in a browser tab. Nothing
// else remembers which board a stored view was shot against. Two sessions shot
// on different boards produce JSON that looks identical, so a reader assuming
// the 7x5/70mm default would silently re-fit one of them at the wrong scale and
// get a plausible, wrong answer.
static std::string board_json(void)
{
    const CharucoBoardConfig& c = g_calib_board;  // as of CALIB_K_START
    char buf[256];
    snprintf(buf, sizeof(buf),
             "\"board\":{\"squares_x\":%d,\"squares_y\":%d,\"square_mm\":%.3f,"
             "\"marker_mm\":%.3f,\"dictionary\":%d,\"margin_x_mm\":%.3f,"
             "\"margin_y_mm\":%.3f}",
             c.squares_x, c.squares_y, c.square_length_mm, c.marker_length_mm,
             c.dictionary_id, c.outer_margin_x_mm, c.outer_margin_y_mm);
    return std::string(buf);
}

// CALIB_K_VIEW header for one uploaded image: which view/session it belongs to,
// which of the two images it is (`kind`), every ChArUco corner that fed the fit
// as [x, y, id] triplets, the board those ids refer to, and the session's K/dist.
//
// Built into a std::string rather than the fixed char[256] the rest of this
// file uses, because the corner list scales with the board: 24 points on the
// current 7x5 board is already ~500 bytes, and a denser 9x7 board would be
// ~1KB. snprintf into a short buffer would truncate it into invalid JSON.
//
// Corners and K ride on BOTH images rather than just one. They cost ~600 bytes
// against a ~200KB JPEG, and duplicating them means a view whose second
// transfer fails still lands on the server with its measurements intact.
static std::string calib_view_json(const CalibViewJpeg& v, const char* kind,
                                   const std::string& bjson,
                                   const std::string& kjson)
{
    char buf[288];
    snprintf(buf, sizeof(buf),
             "{\"type\":\"CALIB_K_VIEW\",\"format\":\"jpeg\",\"kind\":\"%s\","
             "\"session\":%ld,\"view\":%d,\"target\":%d,\"corners\":%d,"
             "\"w\":%d,\"h\":%d,\"charuco\":[",
             kind, g_calib_session, v.view, v.target, v.corners,
             v.width, v.height);
    std::string s(buf);
    for (size_t i = 0; i < v.points.size(); ++i) {
        snprintf(buf, sizeof(buf), "%s[%.3f,%.3f,%d]", i ? "," : "",
                 v.points[i].x, v.points[i].y, v.points[i].id);
        s += buf;
    }
    s += "],";
    s += bjson;
    s += ",";
    s += kjson;
    s += "}";
    return s;
}

static void do_calib_view_upload(void)
{
    int total = calib_view_store_count();
    bool haveK = false;
    const std::string kjson = intrinsics_json(&haveK);
    const std::string bjson = board_json();

    char json[256];
    snprintf(json, sizeof(json),
             "{\"type\":\"CALIB_K_UPLOAD\",\"stage\":\"start\",\"total\":%d,"
             "\"session\":%ld,\"k_available\":%s}",
             total, g_calib_session, haveK ? "true" : "false");
    calib_view_progress_push(json);

    int sent = 0;
    for (int i = 0; i < total; ++i) {
        CalibViewJpeg v;
        if (!calib_view_store_get(i, v))
            continue;

        // Plain first: if the link dies mid-view, the image an offline re-fit
        // actually needs is the one already on disk.
        const struct { const char* kind; const std::vector<uint8_t>* buf; }
        imgs[] = {
            { "plain",   &v.jpeg_plain },
            { "overlay", &v.jpeg },
        };
        int ok = 0, bytes = 0;
        for (int k = 0; k < 2; ++k) {
            if (imgs[k].buf->empty())
                continue;
            const std::string vj = calib_view_json(v, imgs[k].kind, bjson, kjson);
            if (snapshot_sender_send_jpeg(vj.c_str(), (int) vj.size(),
                                          v.width, v.height,
                                          imgs[k].buf->data(),
                                          (int) imgs[k].buf->size()) == 0) {
                ++ok;
                bytes += (int) imgs[k].buf->size();
            }
        }

        if (ok == 2) {
            ++sent;
            snprintf(json, sizeof(json),
                     "{\"type\":\"CALIB_K_UPLOAD\",\"stage\":\"progress\","
                     "\"sent\":%d,\"total\":%d,\"view\":%d,\"bytes\":%d,"
                     "\"images\":%d}",
                     sent, total, v.view, bytes, ok);
        } else {
            snprintf(json, sizeof(json),
                     "{\"type\":\"CALIB_K_UPLOAD\",\"stage\":\"error\","
                     "\"view\":%d,\"images\":%d}", v.view, ok);
        }
        calib_view_progress_push(json);
    }

    snprintf(json, sizeof(json),
             "{\"type\":\"CALIB_K_UPLOAD\",\"stage\":\"done\",\"sent\":%d,"
             "\"total\":%d}", sent, total);
    calib_view_progress_push(json);
    g_uploading = false;
}

// Called every frame from the frame thread. Forwards any progress lines the
// upload worker queued to pose_sender (which only this thread may touch), then
// starts a new upload if one was requested. A request arriving mid-upload is
// reported as busy and dropped.
static void run_calib_view_upload(void)
{
    // Drain worker-queued progress onto the (single-threaded) pose socket.
    char line[512];
    while (calib_view_progress_pop(line, sizeof(line)))
        pose_sender_send_control_line(line);

    if (!g_k_upload_requested)
        return;
    g_k_upload_requested = false;
    bool expected = false;
    if (!g_uploading.compare_exchange_strong(expected, true)) {
        pose_sender_send_control_line(
            "{\"type\":\"CALIB_K_UPLOAD\",\"stage\":\"busy\"}");
        return;
    }
    std::thread(do_calib_view_upload).detach();
}
#endif // ENABLE_CALIB_VIEW_UPLOAD

// Consume pending CALIB_K_CAPTURE / CALIB_K_COMPUTE requests using this
// frame's gray buffer, and report the outcome to the server. On an accepted
// capture the overlay image is JPEG-encoded and kept in memory (deferred
// upload via CALIB_K_UPLOAD) when ENABLE_CALIB_VIEW_UPLOAD is on.
static void run_k_calibration(const uint8_t* nv12, int width, int height,
                              const cv::Mat& gray)
{
    (void) nv12;
    (void) width;
    (void) height;
    char json[512];
    if (g_k_capture_requested) {
        g_k_capture_requested = false;
        IntrCalibViewQuality quality;
        IntrCalibState st = intrinsics_capture_view(gray, &quality);
        if (st == IC_CAPTURED) {
            int len = snprintf(json, sizeof(json),
                     "{\"type\":\"CALIB_K_PROGRESS\",\"views\":%d,\"target\":%d,"
                     "\"corners\":%d,\"corners_total\":%d,"
                     "\"coverage\":%.5f,\"sharpness\":%.1f,\"move_px\":%.1f,"
                     "\"ready\":%s",
                     intrinsics_views(), intrinsics_target_views(),
                     quality.corners_found, quality.corners_total,
                     quality.coverage_ratio, quality.sharpness,
                     quality.mean_move_px,
                     intrinsics_views() >= intrinsics_target_views()
                         ? "true" : "false");
#if ENABLE_LDC_CHECK
            // Also measure THIS view's straightness (same board, same frame)
            // so the operator sees the raw lens distortion at each capture
            // pose as a free by-product of the session.
            LdcResult r;
            if (ldc_measure_once(gray, &r) == LC_MEASURED &&
                len > 0 && len < (int) sizeof(json)) {
                len += snprintf(json + len, sizeof(json) - len,
                     ",\"straight_rms_px\":%.3f,\"edge_max_px\":%.3f,"
                     "\"center_max_px\":%.3f",
                     r.straight_rms_px, r.edge_max_px, r.center_max_px);
            }
#endif
            if (len > 0 && len < (int) sizeof(json))
                snprintf(json + len, sizeof(json) - len, "}");
            pose_sender_send_control_line(json);
#if ENABLE_CALIB_VIEW_UPLOAD
            // Encode THIS frame's overlay image now (fast); the upload is
            // deferred to CALIB_K_UPLOAD so capture stays snappy.
            encode_and_store_calib_view(nv12, width, height,
                                        intrinsics_views(),
                                        intrinsics_target_views(),
                                        quality.corners_found);
#endif
        } else if (st == IC_CAPTURE_REJECTED) {
            char safeReason[128];
            json_escape(quality.reason ? quality.reason : "quality gate failed",
                        safeReason, sizeof(safeReason));
            snprintf(json, sizeof(json),
                     "{\"type\":\"CALIB_K_PROGRESS\",\"rejected\":true,"
                     "\"corners\":%d,\"corners_total\":%d,\"views\":%d,"
                     "\"coverage\":%.5f,\"sharpness\":%.1f,\"move_px\":%.1f,"
                     "\"reason\":\"%s\"}",
                     quality.corners_found, quality.corners_total,
                     intrinsics_views(), quality.coverage_ratio,
                     quality.sharpness, quality.mean_move_px,
                     safeReason);
            pose_sender_send_control_line(json);
        } else {
            report_k_result(st);
        }
    }
    if (g_k_compute_requested) {
        g_k_compute_requested = false;
        snprintf(json, sizeof(json),
                 "{\"type\":\"CALIB_K_COMPUTING\",\"views\":%d}",
                 intrinsics_views());
        pose_sender_send_control_line(json);
        report_k_result(intrinsics_compute());
    }
}
#endif // ENABLE_INTRINSICS_CALIB

#if ENABLE_HOMOGRAPHY
// Drive the calibration state machine with this frame's detections and
// report the one-shot outcome back to the server.
static void run_calibration(const std::vector<ArucoProcessor::Detection>& dets)
{
    HomographyCalibState st = homography_feed(dets);
    char json[384];
    if (st == HG_DONE_OK) {
        double h[9] = {0};
        homography_get(h); // just succeeded -> H is valid
        snprintf(json, sizeof(json),
                 "{\"type\":\"CALIB_RESULT\",\"ok\":true,\"frames\":%d,"
                 "\"H\":[%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e]}",
                 homography_progress(),
                 h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], h[8]);
        pose_sender_send_control_line(json);
        debug_message("ArUco: homography calibration OK\n");
    } else if (st == HG_DONE_FAIL) {
        snprintf(json, sizeof(json),
                 "{\"type\":\"CALIB_RESULT\",\"ok\":false,\"reason\":\"%s\"}",
                 homography_fail_reason());
        pose_sender_send_control_line(json);
        debug_message("ArUco: homography calibration FAILED: %s\n",
                      homography_fail_reason());
    }
}
#endif // ENABLE_HOMOGRAPHY

#if ENABLE_LDC_CHECK
// Measure residual lens distortion on THIS frame's board and stream the
// straightness metric (with the before/after undistort comparison, if a
// calibration exists) live, every frame, for as long as the check is active
// — from LDC_CHECK_START until LDC_CHECK_STOP. No board -> a lightweight
// "searching" line so the operator knows the camera is alive. The RPi side
// throttles its own logging (e.g. once per second) rather than the camera
// limiting how often it reports.
static void run_ldc_check(const cv::Mat& gray)
{
    LdcResult r;
    LdcCheckState st = ldc_check_feed(gray, &r);
    char json[480];
    if (st == LC_MEASURED) {
        int len = snprintf(json, sizeof(json),
                 "{\"type\":\"LDC_CHECK\",\"found\":true,"
                 "\"markers\":%d,\"markers_total\":%d,"
                 "\"corners\":%d,\"corners_total\":%d,"
                 "\"straight_rms_px\":%.3f,\"straight_max_px\":%.3f,"
                 "\"edge_max_px\":%.3f,\"center_max_px\":%.3f",
                 r.markers_found, r.markers_total,
                 r.corners_found, r.corners_total,
                 r.straight_rms_px, r.straight_max_px,
                 r.edge_max_px, r.center_max_px);
        // If a calibration exists, append the "after OpenCV undistort"
        // comparison numbers from the SAME corners (see ldc_checker.h).
        if (r.has_undistorted && len > 0 && len < (int) sizeof(json)) {
            len += snprintf(json + len, sizeof(json) - len,
                     ",\"undistorted\":{\"straight_rms_px\":%.3f,"
                     "\"straight_max_px\":%.3f,\"edge_max_px\":%.3f,"
                     "\"center_max_px\":%.3f}",
                     r.straight_rms_px_u, r.straight_max_px_u,
                     r.edge_max_px_u, r.center_max_px_u);
        }
        if (len > 0 && len < (int) sizeof(json))
            snprintf(json + len, sizeof(json) - len, "}");
        pose_sender_send_line(json);
    } else if (st == LC_NO_BOARD) {
        // Still report marker coverage so the operator sees a partial board.
        snprintf(json, sizeof(json),
                 "{\"type\":\"LDC_CHECK\",\"found\":false,"
                 "\"markers\":%d,\"markers_total\":%d,\"corners\":%d}",
                 r.markers_found, r.markers_total, r.corners_found);
        pose_sender_send_line(json);
    }
}

// LDC_SNAPSHOT: measure this frame's metrics (regardless of LDC_CHECK mode),
// render the marker/corner overlay onto a color copy, and upload metrics +
// the RAW RGB pixels together over the dedicated snapshot connection
// (separate from the realtime pose_sender link — see snapshot_sender.h).
// No JPEG here: the on-camera OpenCV build excludes imgcodecs (kept out to
// save space, see opencv_cross/build_opencv.sh), so the RPi side encodes/
// saves the image instead.
static void take_and_send_snapshot(const uint8_t* nv12, int width, int height,
                                   const cv::Mat& gray)
{
    LdcResult r;
    LdcCheckState st = ldc_measure_once(gray, &r);

    char json[480];
    int len;
    if (st == LC_MEASURED) {
        len = snprintf(json, sizeof(json),
                 "{\"type\":\"LDC_SNAPSHOT\",\"found\":true,\"w\":%d,\"h\":%d,"
                 "\"markers\":%d,\"markers_total\":%d,"
                 "\"corners\":%d,\"corners_total\":%d,"
                 "\"straight_rms_px\":%.3f,\"straight_max_px\":%.3f,"
                 "\"edge_max_px\":%.3f,\"center_max_px\":%.3f",
                 width, height, r.markers_found, r.markers_total,
                 r.corners_found, r.corners_total,
                 r.straight_rms_px, r.straight_max_px,
                 r.edge_max_px, r.center_max_px);
        if (r.has_undistorted && len > 0 && len < (int) sizeof(json)) {
            len += snprintf(json + len, sizeof(json) - len,
                     ",\"undistorted\":{\"straight_rms_px\":%.3f,"
                     "\"straight_max_px\":%.3f,\"edge_max_px\":%.3f,"
                     "\"center_max_px\":%.3f}",
                     r.straight_rms_px_u, r.straight_max_px_u,
                     r.edge_max_px_u, r.center_max_px_u);
        }
        if (len > 0 && len < (int) sizeof(json))
            len += snprintf(json + len, sizeof(json) - len, "}");
    } else {
        len = snprintf(json, sizeof(json),
                 "{\"type\":\"LDC_SNAPSHOT\",\"found\":false,\"w\":%d,\"h\":%d,"
                 "\"markers\":%d,\"markers_total\":%d,\"corners\":%d}",
                 width, height, r.markers_found, r.markers_total, r.corners_found);
    }
    if (len <= 0 || len >= (int) sizeof(json)) {
        debug_message("ArUco: LDC_SNAPSHOT failed (JSON overflow)\n");
        return;
    }

    int rc = send_annotated_image(nv12, width, height, json, len,
                                  &r.corners, &r.is_edge);
    debug_message("ArUco: LDC_SNAPSHOT %s\n",
                  rc == 0 ? "sent" : "FAILED to send");
}
#endif // ENABLE_LDC_CHECK

#if ENABLE_HOMOGRAPHY && ENABLE_HG_SNAPSHOT
// HG_SNAPSHOT: JPEG-encode ONE current frame (no overlay — the dashboard draws
// its own overlay on top) and upload it over the snapshot channel as a floor
// reference, together with the current H and every anchor's world position and
// (if visible right now) its detected pixel center. The browser canvas needs
// the anchor PIXEL positions to draw the work-area boundary on the still, and
// H to label markers with world coordinates; both are baked in here so a single
// upload is self-contained. JPEG (not raw RGB) because a browser can display it
// directly — the raw/.ppm snapshot path cannot be shown in an <img>/canvas.
static void take_and_send_hg_reference(
        const uint8_t* nv12, int width, int height,
        const std::vector<ArucoProcessor::Detection>& dets)
{
    cv::Mat bgr;
    if (!ArucoProcessor::nv12ToBgr(nv12, width, height, bgr)) {
        pose_sender_send_control_line(
            "{\"type\":\"HG_REF\",\"ok\":false,\"reason\":\"nv12 convert failed\"}");
        return;
    }
    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    if (!rgb.isContinuous())
        rgb = rgb.clone();

    std::vector<uint8_t> jpeg;
    if (!encode_rgb_jpeg(rgb, jpeg)) {
        pose_sender_send_control_line(
            "{\"type\":\"HG_REF\",\"ok\":false,\"reason\":\"jpeg encode failed\"}");
        return;
    }

    // Header travels in the snapshot JSON. Built with std::string because the
    // anchor list is variable-length; a fixed char[] could truncate it.
    char buf[256];
    std::string s = "{\"type\":\"HG_REF\",\"format\":\"jpeg\"";
    snprintf(buf, sizeof(buf), ",\"w\":%d,\"h\":%d", width, height);
    s += buf;

    double h[9];
    if (homography_get(h)) {
        snprintf(buf, sizeof(buf),
                 ",\"H\":[%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e]",
                 h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], h[8]);
        s += buf;
    }

    AnchorConfig anchors[HOMOGRAPHY_MAX_ANCHORS];
    int na = homography_get_anchors(anchors, HOMOGRAPHY_MAX_ANCHORS);
    s += ",\"anchors\":[";
    for (int a = 0; a < na; ++a) {
        double px = 0.0, py = 0.0;
        bool seen = false;
        for (size_t i = 0; i < dets.size(); ++i) {
            if (dets[i].id == anchors[a].id && dets[i].corners2d.size() >= 4) {
                float cx = 0.f, cy = 0.f;
                for (int k = 0; k < 4; ++k) {
                    cx += dets[i].corners2d[k].x;
                    cy += dets[i].corners2d[k].y;
                }
                px = cx * 0.25f;
                py = cy * 0.25f;
                seen = true;
                break;
            }
        }
        snprintf(buf, sizeof(buf),
                 "%s{\"id\":%d,\"wx\":%.1f,\"wy\":%.1f,\"seen\":%s,"
                 "\"px\":%.2f,\"py\":%.2f}",
                 a ? "," : "", anchors[a].id, anchors[a].wx, anchors[a].wy,
                 seen ? "true" : "false", px, py);
        s += buf;
    }
    s += "]";   // close "anchors" before the next key starts

    // Validation markers have known surveyed world positions but are never
    // used to calculate H. The browser uses this table to report independent
    // position error from the same live marker stream.
    AnchorConfig validation[HOMOGRAPHY_MAX_VALIDATION_MARKERS];
    int nv = homography_get_validation_markers(validation, HOMOGRAPHY_MAX_VALIDATION_MARKERS);
    s += ",\"validation\":[";
    for (int v = 0; v < nv; ++v) {
        snprintf(buf, sizeof(buf),
                 "%s{\"id\":%d,\"wx\":%.1f,\"wy\":%.1f}",
                 v ? "," : "", validation[v].id,
                 validation[v].wx, validation[v].wy);
        s += buf;
    }
    s += "]}";

    int rc = snapshot_sender_send_jpeg(s.c_str(), (int) s.size(),
                                       rgb.cols, rgb.rows,
                                       jpeg.data(), (int) jpeg.size());
    debug_message("ArUco: HG_SNAPSHOT %s\n", rc == 0 ? "sent" : "FAILED to send");
}
#endif // ENABLE_HOMOGRAPHY && ENABLE_HG_SNAPSHOT

#endif // ENABLE_POSE_STREAM

/**
 * Run ArUco detection on the SDK's NV12 raw frame and show each marker's
 * Pos/Rot as OSD text on the live video.
 *
 * OPENSDK_ENCVIDEO_EVENT fields (opensdk_defines.h): buff, size, width,
 * height, frameType, codec, time_stamp.
 */
static void process_raw_video(OPENSDK_ENCVIDEO_EVENT* vidEvent)
{
    if (g_aruco == NULL || vidEvent == NULL)
        return;

    const uint8_t* nv12 = (const uint8_t*) vidEvent->buff;
    int width  = vidEvent->width;
    int height = vidEvent->height;

    // The NV12 Y plane is already grayscale — detect directly, no color copy.
    cv::Mat gray = ArucoProcessor::nv12GrayView(nv12, width, height);
    if (gray.empty()) {
        debug_message("ArUco: bad raw frame (%dx%d)\n", width, height);
        return;
    }

#if ENABLE_POSE_STREAM
    // Capture t_frame as close to frame arrival as possible (the SDK event
    // carries no usable epoch timestamp for our purposes).
    long t_frame_ms = epoch_ms();

    // Before any of the early returns below: CPU load is the one thing worth
    // knowing precisely when detection is off or in fps-test mode, so it must
    // not sit behind a path that skips them.
    report_cpu_if_due();
#endif

#if ENABLE_POSE_STREAM
    // Diagnostic (RAW_FPS_TEST): emit one heartbeat per callback and do nothing
    // else, so seq counts frames the SDK delivers rather than frames we manage
    // to process. Commands are polled below in normal mode, so poll here too --
    // otherwise the link goes deaf and the mode could never be switched off.
    if (g_measure_raw_fps) {
        handle_server_commands();
        std::vector<ArucoProcessor::Detection> none;
        send_pose_packets(none, t_frame_ms, width, height);
        return;
    }

    // Detection switched off by the operator (DETECT_ENABLE 0). Same shape as
    // the raw-fps path above -- skip the scan, but keep polling commands so the
    // switch can be flipped back, and keep the heartbeat so the link still
    // looks alive.
    if (!g_detect_enabled) {
        handle_server_commands();
        std::vector<ArucoProcessor::Detection> none;
        send_pose_packets(none, t_frame_ms, width, height);
        return;
    }
#endif

    // Dynamic ROI: narrow the search to where the marker just was (no-op when
    // disabled — roiForFrame() then returns the operator's manual ROI).
    // The two inputs the tracker used to read straight out of this file's
    // globals are now passed in, which is what let it move to dyn_roi.cpp.
#if ENABLE_HOMOGRAPHY
    const bool hgCollecting = homography_collecting();
#else
    const bool hgCollecting = false;
#endif
    g_aruco->setRoi(g_dynroi.roiForFrame(g_manual_roi, hgCollecting));

    std::vector<ArucoProcessor::Detection> dets;
    int count = g_aruco->detect(gray, dets);

    g_dynroi.update(dets, gray.size());

#if ENABLE_POSE_STREAM
    // Server can push commands (e.g. CALIB_START) back over the same link.
    handle_server_commands();

#if ENABLE_INTRINSICS_CALIB && ENABLE_CALIB_VIEW_UPLOAD
    // Consume a pending CALIB_K_UPLOAD (spawns a detached uploader thread).
    run_calib_view_upload();
#endif

#if ENABLE_HOMOGRAPHY && ENABLE_HG_SNAPSHOT
    // Consume a pending HG_SNAPSHOT (floor reference still for the dashboard
    // canvas). Rare, on-demand; the brief encode/upload stall is the same
    // trade-off already accepted for LDC_SNAPSHOT. dets is this frame's fresh
    // detections, used to record where the anchors currently sit in pixels.
    if (g_hg_snapshot_requested) {
        g_hg_snapshot_requested = false;
        take_and_send_hg_reference(nv12, width, height, dets);
    }
#endif

#if ENABLE_LDC_CHECK
    // Consume a pending LDC_SNAPSHOT request now that this frame's NV12/gray
    // buffers are in scope (the command handler above only set the flag).
    // Works regardless of LDC_CHECK mode — a snapshot can be taken any time.
    if (g_snapshot_requested) {
        g_snapshot_requested = false;
        take_and_send_snapshot(nv12, width, height, gray);
    }

    // Dedicated residual-distortion check: while active it OWNS the link
    // (ArUco pose streaming and calibrations are paused) and streams only the
    // straightness metric. handle_server_commands above still runs, so
    // LDC_CHECK_STOP is honoured.
    if (ldc_check_active()) {
        run_ldc_check(gray);
    } else
#endif
    {
#if ENABLE_INTRINSICS_CALIB
        // K/dist calibration: consumes pending CALIB_K_CAPTURE/COMPUTE
        // requests (manual, button-triggered). nv12 is passed so an accepted
        // capture's image can be uploaded for later inspection.
        run_k_calibration(nv12, width, height, gray);
#if ENABLE_HOMOGRAPHY
        // Floor-board H test is independent of K/dist capture.  It waits for
        // one frame with >=18 ChArUco corners, fits 17 and validates the rest.
        if (!intrinsics_collecting())
            run_hg_charuco(gray);
#endif
#endif

#if ENABLE_INTRINSICS_CALIB
        // While a K-calibration session is open it OWNS the link too: pose
        // streaming is paused so the operator sees only capture feedback, not
        // a flood of CAM_POSE. (The board's own markers would otherwise stream
        // as poses and bury the [calib-K] lines.)
        if (!intrinsics_collecting())
#endif
        {
#if ENABLE_HOMOGRAPHY
            // Homography calibration consumes FRESH detections (anchor ids in
            // app_config.h).
            run_calibration(dets);
#endif

            // TCP stream to the vision server uses FRESH results only (before
            // the OSD hold below reuses stale ones).
            send_pose_packets(dets, t_frame_ms, width, height);
        }
    }
#endif // ENABLE_POSE_STREAM

#if ENABLE_OSD
    // Temporal hold: keep showing the last good detection for a few empty
    // frames so a brief miss (angle/blur/lighting) does not flicker the OSD.
    // Display-only nicety — held (stale) results never reach the TCP stream.
    static std::vector<ArucoProcessor::Detection> lastDets;
    static int misses = 0;
    const int HOLD_FRAMES = 5;
    if (count > 0) {
        lastDets = dets;
        misses = 0;
    } else if (misses < HOLD_FRAMES) {
        ++misses;
        dets = lastDets;   // reuse last good result during the hold window
    }

    // Per-marker Pos/Rot list on the live video (SDK OSD text, bottom-left).
    update_osd_status(dets, width, height);
#endif // ENABLE_OSD

#if SAVE_DEBUG_JPG
    if (count > 0) {
        cv::Mat bgr;
        if (ArucoProcessor::nv12ToBgr(nv12, width, height, bgr)) {
            g_aruco->process(bgr, dets);            // redraw arrows on color img
            cv::imwrite("/mnt/aruco_last.jpg", bgr);
        }
    }
#else
    (void) count;
#endif
}

/**
*@ ********************************************************************
*@ Name           : recv_event                                        *
*@ Description    : Receives input event from camera SDK              *
*@ Arguments      : eventIn[IN]: Input event type                     *
*@                : pData[IN]  : Data for input event                 *
*@ Return Value   : N/A                                               *
*@ Notes          :                                                   *
*@ Change History :                                                   *
*@ ********************************************************************
**/
void recv_event(OPENSDK_INPUT_EVENT eventIn, void* pData)
{
    //Handle the input events here
    switch(eventIn) {
        case OPENSDK_RAW_VIDEO:
        {
            // No per-frame logging here: debug_message goes out over the
            // 115200-baud serial console and BLOCKS, which at 5+ fps starves
            // the detection loop (measured as visible OSD lag).
            OPENSDK_ENCVIDEO_EVENT* vidEvent;
            vidEvent = (OPENSDK_ENCVIDEO_EVENT*) pData;
            //Process raw video: ArUco detection + Pos/Rot OSD text
            process_raw_video(vidEvent);
            break;
        }
        case OPENSDK_RAW_AUDIO:
        {
            OPENSDK_ENCAUDIO_EVENT* audEvent;            
            debug_message("Event Type: OPENSDK_RAW_AUDIO\n");
            audEvent = (OPENSDK_ENCAUDIO_EVENT*) pData;
            //Process raw audio
            break;
        }
        case OPENSDK_MEDIA_VIDEO:
        {
            OPENSDK_ENCVIDEO_EVENT* vidEvent;            
            debug_message("Event Type: OPENSDK_MEDIA_VIDEO\n");
            vidEvent = (OPENSDK_ENCVIDEO_EVENT*) pData;
            //Process encoded video
            break;
        }
        case OPENSDK_MEDIA_AUDIO:
        {
            OPENSDK_ENCAUDIO_EVENT* audEvent;            
            debug_message("Event Type: OPENSDK_MEDIA_AUDIO\n");
            audEvent = (OPENSDK_ENCAUDIO_EVENT*) pData;
            //Process encoded audio
            break;
        }
        case OPENSDK_RECORDED_VIDEO:
        {
            OPENSDK_ENCVIDEO_EVENT* vidEvent;            
            debug_message("Event Type: OPENSDK_MEDIA_VIDEO\n");
            vidEvent = (OPENSDK_ENCVIDEO_EVENT*) pData;
            //Process encoded video
            break;
        }
        case OPENSDK_NEW_CLIENT:
        {
            OPENSDK_NETWORK_CLIENT* client;
            client = (OPENSDK_NETWORK_CLIENT*) pData;
            debug_message("Event Type: OPENSDK_NEW_CLIENT\n");
            debug_message("Client IP: %s, port: %d ID: %d\n", 
                    client->ip_address, client->port, client->client_id);            
            break;
        }
        case OPENSDK_NETWORK_DATA:
        {
            OPENSDK_NETWORK_PACKET* packet;            
            debug_message("Event Type: OPENSDK_NETWORK_DATA\n");            
            packet = (OPENSDK_NETWORK_PACKET*) pData;           
            break;
        } 
        case OPENSDK_CLIENT_CLOSED:
        {
            int* client_id;
            debug_message("Event Type: OPENSDK_CLIENT_CLOSED\n"); 
            
            client_id = (int*) pData;  
            debug_message("Client ID: %d\n", *client_id); 
            break;
        }
        case OPENSDK_MD_EVENT:
        {
            OPENSDK_VA_EVENTS* mdEvent;            
            mdEvent = (OPENSDK_VA_EVENTS*) pData;

            if(mdEvent->State) {
                debug_message("Event Type: OPENSDK_MD_EVENT ON\n");
            } else {
                debug_message("Event Type: OPENSDK_MD_EVENT OFF\n");
            }
            break;
        }
        case OPENSDK_FD_EVENT:
        {
            OPENSDK_VA_EVENTS* fdEvent;
            fdEvent = (OPENSDK_VA_EVENTS*) pData;

            if(fdEvent->State) {
                debug_message("Event Type: OPENSDK_FD_EVENT ON\n");
            } else {
                debug_message("Event Type: OPENSDK_FD_EVENT OFF\n");
            }
            break;
        }
        case OPENSDK_TAMP_EVENTS:
        {
            OPENSDK_VA_EVENTS* tampEvent;
            tampEvent = (OPENSDK_VA_EVENTS*) pData;

            if(tampEvent->State) {
                debug_message("Event Type: OPENSDK_TAMP_EVENT ON\n");
            } else {
                debug_message("Event Type: OPENSDK_TAMP_EVENT OFF\n");
            }
            break;
        }
        case OPENSDK_IV_PASSING_EVENT:
        {
            OPENSDK_VIDEOANALYTIC_EVENT* vaEvent;
            vaEvent = (OPENSDK_VIDEOANALYTIC_EVENT*) pData;
            debug_message("Event Type: LINE ACTION %d\n",vaEvent->Action);
			break;
        }
	case OPENSDK_ENTER_EXIT_EVENT:
        {
            OPENSDK_VIDEOANALYTIC_EVENT* vaEvent;
            vaEvent = (OPENSDK_VIDEOANALYTIC_EVENT*) pData;
            debug_message("Event Type: AREA ACTION %d\n",vaEvent->Action);
			break;
        }
	case OPENSDK_AD_EVENT:
        {
            OPENSDK_VIDEOANALYTIC_EVENT* adEvent;
            adEvent = (OPENSDK_VIDEOANALYTIC_EVENT*) pData;
            debug_message("Event Type: OPENSDK_AD_EVENT \n");
            break;
        }
        case OPENSDK_ALARM_EVENT:
        {
            OPENSDK_VA_EVENTS* alarmEvent;
            alarmEvent = (OPENSDK_VA_EVENTS*) pData;

            if(alarmEvent->Level) {
                debug_message("Event Type: OPENSDK_ALARM_EVENT ON\n");
            } else {
                debug_message("Event Type: OPENSDK_ALARM_EVENT OFF\n");
            }
            break;
        }
        case OPENSDK_SDCARD_INSERTED:
        {
            debug_message("Event Type: OPENSDK_SDCARD_INSERTED\n");
            break;
        }
        case OPENSDK_SDCARD_REMOVED:
        {
            debug_message("Event Type: OPENSDK_SDCARD_REMOVED\n");
            break;
        }
        case OPENSDK_NETWORK_CONNECTED:
        {
            debug_message("Event Type: OPENSDK_NETWORK_CONNECTED\n");
            break;
        }
        case OPENSDK_NETWORK_DISCONNECTED:
        {
            debug_message("Event Type: OPENSDK_NETWORK_DISCONNECTED\n");
            break;
        }
        case OPENSDK_CPU_USAGE_HIGH:
        {
            debug_message("Event Type: OPENSDK_CPU_USAGE_HIGH\n");
            break;
        }
        case OPENSDK_MEMORY_USAGE_HIGH:
        {
            debug_message("Event Type: OPENSDK_MEMORY_USAGE_HIGH\n");
            break;
        }
        case OPENSDK_CPU_MEMORY_USAGE_HIGH:
        {
            debug_message("Event Type: OPENSDK_CPU_MEMORY_USAGE_HIGH\n");
            break;
        }
        case OPENSDK_NETWORK_BANDWIDTH_HIGH:
        {
            debug_message("Event Type: OPENSDK_NETWORK_BANDWIDTH_HIGH\n");
            break;
        }
        case OPENSDK_DISK_USAGE_HIGH:
        {
            debug_message("Event Type: OPENSDK_DISK_USAGE_HIGH\n");
            break;
        }
        case OPENSDK_STOP_APP_CMD:
        {
            debug_message("Event Type: OPENSDK_STOP_APP_CMD\n");
            break;
        }
        case OPENSDK_NOTIFY:
        {
            debug_message("Event Type: OPENSDK_MESSAGE\n");
            break;
        }
        case OPENSDK_NETWORK_INTERFACE:
        {
            debug_message("Event Type: OPENSDK_NETWORK_INTERFACE\n");
            break;
        }
        case OPENSDK_NETWORK_PORTS:
        {
            debug_message("Event Type: OPENSDK_NETWORK_PORTS\n");
            break;
        }
        case OPENSDK_VIDEO_PROFILE:
        {
            debug_message("Event Type: OPENSDK_VIDEO_PROFILE\n");
            break;
        }
        case OPENSDK_MEDIA_CONFIG:
        {
            debug_message("Event Type: OPENSDK_MEDIA_CONFIG\n");
            break;
        }
        case OPENSDK_IMAGE_CONFIG:
        {
            debug_message("Event Type: OPENSDK_IMAGE_CONFIG\n");
            break;
        }
        case OPENSDK_STORAGE:
        {
            debug_message("Event Type: OPENSDK_STORAGE\n");
            break;
        }
        case OPENSDK_EVENT_CONFIG:
        {
            debug_message("Event Type: OPENSDK_EVENT_CONFIG\n");
            break;
        }
        default:
        {
            debug_message("Unknown event %d occurred\n", eventIn);
            break;
        }
    }

    return;
}
/**
*@ ********************************************************************
*@ Name           : recv_data                                         *
*@ Description    : Receives data from camera SDK                     *
*@ Arguments      : payload_request[IN]: Request from web page        *
*@                : payload_response[OUT]  : Response to web page     *
*@ Return Value   : N/A                                               *
*@ Notes          :                                                   *
*@ Change History :                                                   *
*@ ********************************************************************
**/
OPENSDK_ERR_CODE recv_data(void *payload_request,
                           void *payload_response)
{
    OPENSDK_PAYLOAD_REQUEST*  req_payload;
    OPENSDK_PAYLOAD_RESPONSE* res_payload;
    OPENSDK_ERR_CODE          errCode;
    
    //Initialize local variable
    errCode = OPENSDK_APP_OK;
    
    //Get the request & response pointer
    req_payload = (OPENSDK_PAYLOAD_REQUEST*)payload_request;
    res_payload = (OPENSDK_PAYLOAD_RESPONSE*)payload_response;
    
    debug_message("Request from web page: %s and len %d\n", 
                req_payload->pBuff, req_payload->pBufLen);
    
    //Process request & send response
    
    return errCode;
}

/**
*@ ********************************************************************
*@ Name           : one_shot                                          *
*@ Description    : called to initialize application                  *
*@ Arguments      : N/A                                               *
*@ Return Value   : N/A                                               *
*@ Notes          :                                                   *
*@ Change History :                                                   *
*@ ********************************************************************
**/
void one_shot(void)
{
    debug_message("one_shot\n");

    // Create the ArUco detector once. If you have a real camera calibration,
    // call g_aruco->setCameraIntrinsics(cameraMatrix, distCoeffs) here.
    if (g_aruco == NULL) {
        g_aruco = new ArucoProcessor(MARKER_LENGTH_M);
        debug_message("ArUco: detector initialized (marker=%.3fm)\n",
                      MARKER_LENGTH_M);
    }

#if ENABLE_POSE_STREAM
    pose_sender_init(POSE_SERVER_IP, POSE_SERVER_PORT);
    debug_message("ArUco: pose sender -> %s:%d\n",
                  POSE_SERVER_IP, POSE_SERVER_PORT);
#endif

#if ENABLE_CENTRAL_TLS_STREAM
    if (central_tls_sender_init(CENTRAL_TLS_SERVER_IP, CENTRAL_TLS_SERVER_PORT,
                                CENTRAL_TLS_CA_FILE) == 0) {
        debug_message("ArUco: central TLS sender -> %s:%d\n",
                      CENTRAL_TLS_SERVER_IP, CENTRAL_TLS_SERVER_PORT);
    } else {
        debug_message("ArUco: central TLS disabled (certificate/config error)\n");
    }
#endif

#if ENABLE_LDC_CHECK || ENABLE_INTRINSICS_CALIB
    // Snapshot channel is shared by LDC_SNAPSHOT and CALIB_K_CAPTURE image
    // uploads, so init it whenever either feature is on.
    snapshot_sender_init(POSE_SERVER_IP, SNAPSHOT_SERVER_PORT);
    debug_message("ArUco: snapshot sender -> %s:%d\n",
                  POSE_SERVER_IP, SNAPSHOT_SERVER_PORT);
#endif

#if PERSIST_TO_MNT
    // The app's nand-flash area is not created by the SDK automatically (API
    // doc 6.1) -- without this, every save_intrinsics()/save_h() write fails
    // because PERSIST_DIR doesn't exist yet. Idempotent, so safe every boot.
    system("mkdir -p " PERSIST_DIR);
    debug_message("ArUco: ensured %s exists\n", PERSIST_DIR);
#endif

#if ENABLE_INTRINSICS_CALIB
    // Intrinsics come back BEFORE the homography on purpose. A saved H records
    // which pixel space it was fitted in (HG_COORD_MODE), and restoring
    // "undistort" needs K to already be loaded -- homography_init() applies the
    // saved mode through the validating setter, which refuses without K.
    intrinsics_init(g_aruco); // reload persisted K/dist and apply
    debug_message("ArUco: intrinsics %s\n",
                  intrinsics_available() ? "loaded from /mnt"
                                         : "not calibrated (pinhole guess)");
#endif

#if ENABLE_HOMOGRAPHY
    homography_init(); // reload persisted H + the pixel space it was fitted in
    debug_message("ArUco: homography %s (%s pixels)\n",
                  homography_active() ? "loaded from /mnt" : "not calibrated",
                  homography_undistort_enabled() ? "undistorted" : "raw");
    // After the homography: a marker height is only meaningful alongside a
    // floor plane to measure it from.
    marker_plane_init();
    debug_message("ArUco: marker height %.1f mm\n", marker_plane_height_mm());
#endif

    // Detection tuning last: it only touches the processor's ROI/scan fields,
    // and loading it after the calibrations keeps the startup log in the order
    // an operator reads it (what we know -> how hard we look).
    if (detect_tuning_init(g_aruco)) {
        const cv::Rect r = g_aruco->roi();
        // Mirror it into g_manual_roi: every frame re-applies the ROI through
        // the dynamic tracker, which falls back to g_manual_roi while
        // searching. Without this the loaded ROI is overwritten (with the
        // empty default = full frame) on the very first callback.
        g_manual_roi = r;
        debug_message("ArUco: tuning loaded — roi %d,%d %dx%d, %d scan pass(es)\n",
                      r.x, r.y, r.width, r.height, g_aruco->scanPasses());
    }

    return;
}
