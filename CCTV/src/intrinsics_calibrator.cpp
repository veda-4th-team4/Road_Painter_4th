#include "intrinsics_calibrator.h"
#include "app_config.h"
#include "aruco_processor.h"

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <dirent.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>

// OpenCV moved aruco/charuco into objdetect in 4.7 and reworked the API.
#define ARUCO_NEW_API (CV_VERSION_MAJOR > 4 || \
                       (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7))
#if ARUCO_NEW_API
#include <opencv2/objdetect/charuco_detector.hpp>
#else
#include <opencv2/aruco/charuco.hpp>
#endif

// Lower bound for the runtime-settable target view count. A practical
// calibration wants ~10+ independent views for a stable K, but the operator
// is trusted to set fewer (e.g. for quick tests), so this is just a floor
// against non-positive values, not the recommended count.
static const int kMinViews = 1;

// Runtime-tunable session settings (defaults from app_config.h, overridable
// via intrinsics_set_params() / CALIB_K_SET so the operator can adjust them
// from the server without a rebuild). g_targetViews = accepted views that
// light up "READY TO COMPUTE"; g_rmsLimit = max overall reprojection RMS (px)
// accepted as a PASS at compute time.
static int    g_targetViews = K_CALIB_VIEWS;
static double g_rmsLimit    = K_CALIB_RMS_LIMIT;

// When false, all quality gates are bypassed: captures are accepted as long as
// enough corners are seen, and compute skips the spread/RMS checks.
static bool   g_qualityGates = CALIB_QUALITY_GATES_DEFAULT;

void intrinsics_set_quality_gates(bool on) { g_qualityGates = on; }
bool intrinsics_quality_gates(void)        { return g_qualityGates; }

static ArucoProcessor* g_proc = NULL;

static cv::Mat g_K;        // 3x3 CV_64F
static cv::Mat g_dist;     // 1x5 CV_64F
static bool    g_available = false;

// Ties saved K/dist back to the exact CALIB_K_VIEW image/corner upload set
// that produced it (see intrinsics_set_session_id() below and g_calib_session
// in aruco_detector_cv.cpp). g_pendingSessionId tracks whatever session is
// currently open; g_computedSessionId freezes it at the moment a compute
// actually succeeds, so a later CALIB_K_START (without a recompute) can never
// mislabel an already-computed K/dist with the wrong session.
static long g_pendingSessionId  = 0;
static long g_computedSessionId = 0;
static char g_activeProfile[32] = "기본";

static IntrCalibState g_state = IC_IDLE;
static std::vector<std::vector<cv::Point2f> > g_allCorners; // per-view corners
static std::vector<std::vector<int> >         g_allIds;     // per-view ids
static std::vector<cv::Point2f>               g_viewCenters;
static std::vector<double>                    g_viewCoverage;
static cv::Size g_imageSize;
static double   g_rms = 0.0;
static const char* g_failReason = "";
static int      g_prunedViews = 0;
static long long g_lastCaptureMs = 0;
static char     g_lastCaptureReason[128] = "";

static CharucoBoardConfig g_boardConfig = {
    CHARUCO_SQUARES_X,
    CHARUCO_SQUARES_Y,
    CHARUCO_SQUARE_LEN,
    CHARUCO_MARKER_LEN,
    CHARUCO_DICTIONARY,
    CHARUCO_MARGIN_X_MM,
    CHARUCO_MARGIN_Y_MM
};
static unsigned int g_boardGeneration = 1;

static long long monotonic_ms(void)
{
    return (long long) (cv::getTickCount() * 1000.0 / cv::getTickFrequency());
}

static int dictionary_capacity(int id)
{
    if (id >= 0 && id <= 15) {
        static const int capacities[4] = {50, 100, 250, 1000};
        return capacities[id % 4];
    }
    if (id == 16) return 1024; // DICT_ARUCO_ORIGINAL
    if (id == 17) return 30;   // DICT_APRILTAG_16h5
    if (id == 18) return 35;   // DICT_APRILTAG_25h9
    if (id == 19) return 2320; // DICT_APRILTAG_36h10
    if (id == 20) return 587;  // DICT_APRILTAG_36h11
    return 0;
}

#if !ARUCO_NEW_API
static cv::Ptr<cv::aruco::CharucoBoard> make_board(void)
{
    return
        cv::aruco::CharucoBoard::create(
            g_boardConfig.squares_x, g_boardConfig.squares_y,
            g_boardConfig.square_length_mm, g_boardConfig.marker_length_mm,
            cv::aruco::getPredefinedDictionary(g_boardConfig.dictionary_id));
}
#endif

static bool valid_board_config(const CharucoBoardConfig& c, const char** reason)
{
    const char* why = NULL;
    if (c.squares_x < 3 || c.squares_y < 3 ||
        c.squares_x > 100 || c.squares_y > 100)
        why = "square counts must be in range 3..100";
    else if (!std::isfinite(c.square_length_mm) ||
             c.square_length_mm <= 0.f || c.square_length_mm > 5000.f)
        why = "square length must be in range (0, 5000] mm";
    else if (!std::isfinite(c.marker_length_mm) ||
             c.marker_length_mm <= 0.f ||
             c.marker_length_mm >= c.square_length_mm)
        why = "marker length must be > 0 and < square length";
    else if (c.dictionary_id < 0 || c.dictionary_id > 20)
        why = "unsupported predefined dictionary id";
    else if ((c.squares_x * c.squares_y) / 2 >
             dictionary_capacity(c.dictionary_id))
        why = "board needs more marker ids than dictionary provides";
    else if (!std::isfinite(c.outer_margin_x_mm) ||
             !std::isfinite(c.outer_margin_y_mm) ||
             c.outer_margin_x_mm < 0.f || c.outer_margin_y_mm < 0.f)
        why = "outer margins cannot be negative";

    if (reason) *reason = why;
    return why == NULL;
}

static bool save_board_config(void)
{
#if !PERSIST_TO_MNT
    return true; // /mnt persistence disabled — board config kept in RAM only
#else
    FILE* f = fopen(CHARUCO_CONFIG_FILE, "w");
    if (!f)
        return false;
    fprintf(f, "%d %d %.6f %.6f %d %.6f %.6f\n",
            g_boardConfig.squares_x, g_boardConfig.squares_y,
            g_boardConfig.square_length_mm, g_boardConfig.marker_length_mm,
            g_boardConfig.dictionary_id,
            g_boardConfig.outer_margin_x_mm,
            g_boardConfig.outer_margin_y_mm);
    fclose(f);
    return true;
#endif
}

static bool load_board_config(void)
{
    FILE* f = fopen(CHARUCO_CONFIG_FILE, "r");
    if (!f)
        return false;
    CharucoBoardConfig c;
    int n = fscanf(f, "%d %d %f %f %d %f %f",
                   &c.squares_x, &c.squares_y,
                   &c.square_length_mm, &c.marker_length_mm,
                   &c.dictionary_id,
                   &c.outer_margin_x_mm, &c.outer_margin_y_mm);
    fclose(f);
    if (n != 7 || !valid_board_config(c, NULL))
        return false;
    g_boardConfig = c;
    ++g_boardGeneration;
    return true;
}

// File format version. Bump this if the field count/order below ever changes
// so an old file fails the version check below and falls back to "not
// available" instead of being silently misparsed.
//
// Not bumped for the trailing profile-name line added below: it is appended
// AFTER every field the v1 parser reads, so a v1 binary ignores it and this
// parser treats its absence (a file written before the line existed) as "no
// named profile". Anything that changes the first three lines still needs a bump.
static const int kIntrinsicsFileVersion = 1;

static bool save_intrinsics(void)
{
#if !PERSIST_TO_MNT
    return true; // /mnt persistence disabled — K/dist kept in RAM only
#else
    FILE* f = fopen(INTRINSICS_FILE, "w");
    if (!f)
        return false;
    // session id = g_calib_session (aruco_detector_cv.cpp) at the moment THIS
    // K/dist was computed -- the same id stamped on every CALIB_K_VIEW upload,
    // so this file can be matched back to the exact image/corner set that
    // produced it (0 if computed with view-upload disabled, or unknown).
    fprintf(f, "%d %ld\n", kIntrinsicsFileVersion, g_computedSessionId);
    fprintf(f, "%.10e %.10e %.10e %.10e\n",
            g_K.at<double>(0, 0), g_K.at<double>(1, 1),
            g_K.at<double>(0, 2), g_K.at<double>(1, 2));
    for (int i = 0; i < 5; ++i)
        fprintf(f, "%.10e ", g_dist.at<double>(0, i));
    fprintf(f, "\n");
    // Which named profile these numbers came from, so the dashboard still says
    // "profile X" after a restart instead of dropping back to "기본". Never
    // contains whitespace (valid_profile_name(), or one of the built-in labels).
    fprintf(f, "%s\n", g_activeProfile);
    fclose(f);
    return true;
#endif
}

static bool load_intrinsics(void)
{
    FILE* f = fopen(INTRINSICS_FILE, "r");
    if (!f)
        return false;
    int version;
    long sessionId;
    if (fscanf(f, "%d %ld", &version, &sessionId) != 2 ||
        version != kIntrinsicsFileVersion) {
        fclose(f);
        return false; // unknown/old format -- treat as "not available"
    }
    double fx, fy, cx, cy, d[5];
    int n = fscanf(f, "%lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   &fx, &fy, &cx, &cy, &d[0], &d[1], &d[2], &d[3], &d[4]);
    // Optional trailing profile name (absent in files written before it existed).
    char profile[32] = "";
    bool haveProfile = (n == 9 && fscanf(f, "%31s", profile) == 1);
    fclose(f);
    if (n != 9)
        return false;
    g_K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    g_dist = (cv::Mat_<double>(1, 5) << d[0], d[1], d[2], d[3], d[4]);
    g_computedSessionId = sessionId;
    if (haveProfile)
        snprintf(g_activeProfile, sizeof(g_activeProfile), "%s", profile);
    return true;
}

static void apply_to_processor(void);

// Inject externally-computed intrinsics (e.g. a calibration done off-camera and
// pasted into the dashboard) without running a capture session. Same effect as
// a successful compute: builds g_K/g_dist, marks available, and pushes them to
// the detector so solvePnP uses them immediately. Not tagged to a session id
// (0), since these did not come from an on-camera view set. Refuses obviously
// broken numbers so a typo cannot silently poison the pose stream.
bool intrinsics_load_values(double fx, double fy, double cx, double cy,
                            const double dist[5])
{
    if (!(fx > 0.0) || !(fy > 0.0) || cx < 0.0 || cy < 0.0)
        return false;
    for (int i = 0; i < 5; ++i)
        if (!std::isfinite(dist[i]))
            return false;

    g_K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    g_dist = (cv::Mat_<double>(1, 5) <<
              dist[0], dist[1], dist[2], dist[3], dist[4]);
    g_available = true;
    g_computedSessionId = 0;   // externally supplied, not from an on-camera session
    snprintf(g_activeProfile, sizeof(g_activeProfile), "기본"); // not a profile's values
    apply_to_processor();
    return true;
}

static bool valid_profile_name(const char* name)
{
    if (!name || !name[0] || strlen(name) >= 24) return false;
    for (const char* p = name; *p; ++p)
        if (!(isalnum((unsigned char)*p) || *p == '_' || *p == '-')) return false;
    return true;
}

static bool profile_path(const char* name, char* out, size_t out_size)
{
    if (!valid_profile_name(name)) return false;
    int n = snprintf(out, out_size, PERSIST_DIR "/intrinsics_%s.txt", name);
    return n > 0 && (size_t)n < out_size;
}

// Both entry points below keep one rule: the ACTIVE K/dist is always the K/dist
// the app boots with. So each one mirrors the profile into INTRINSICS_FILE, the
// only file intrinsics_init() reads. Without this a loaded profile silently
// reverted at the next restart unless the operator also pressed CALIB_K_SAVE.
//
// A failed mirror is NOT a failed load/save: the profile is already applied in
// RAM and works for this session. *persisted (optional) reports the /mnt write
// so the dashboard can warn instead of the operator finding out after a reboot.
static bool mirror_to_boot_intrinsics(bool* persisted)
{
#if !PERSIST_TO_MNT
    // save_intrinsics() reports success here without writing anything, which
    // would make the dashboard promise a reboot-safe value it does not have.
    if (persisted) *persisted = false;
    g_failReason = "PERSIST_TO_MNT is 0 — K/dist is RAM only";
    return false;
#else
    bool ok = save_intrinsics();
    if (persisted) *persisted = ok;
    if (!ok) g_failReason = "applied in RAM, but write to " PERSIST_DIR
                            " failed (see Shell tab /mnt checks)";
    return ok;
#endif
}

bool intrinsics_save_profile(const char* name, bool* persisted)
{
    if (persisted) *persisted = false;
    if (!g_available || !valid_profile_name(name)) return false;
#if !PERSIST_TO_MNT
    return false;
#else
    char path[256];
    if (!profile_path(name, path, sizeof(path))) return false;
    FILE* f = fopen(path, "w");
    if (!f) return false;
    fprintf(f, "%d %ld\n", kIntrinsicsFileVersion, g_computedSessionId);
    fprintf(f, "%.10e %.10e %.10e %.10e\n", g_K.at<double>(0, 0),
            g_K.at<double>(1, 1), g_K.at<double>(0, 2), g_K.at<double>(1, 2));
    for (int i = 0; i < 5; ++i) fprintf(f, "%.10e ", g_dist.at<double>(0, i));
    fprintf(f, "\n");
    fclose(f);
    snprintf(g_activeProfile, sizeof(g_activeProfile), "%s", name);
    mirror_to_boot_intrinsics(persisted);
    return true;
#endif
}

bool intrinsics_load_profile(const char* name, bool* persisted)
{
    if (persisted) *persisted = false;
    if (!valid_profile_name(name)) return false;
    char path[256];
    if (!profile_path(name, path, sizeof(path))) return false;
    FILE* f = fopen(path, "r");
    if (!f) return false;
    int version; long sessionId; double fx, fy, cx, cy, d[5];
    int n = fscanf(f, "%d %ld %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   &version, &sessionId, &fx, &fy, &cx, &cy,
                   &d[0], &d[1], &d[2], &d[3], &d[4]);
    fclose(f);
    if (n != 11 || version != kIntrinsicsFileVersion) return false;
    g_K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    g_dist = (cv::Mat_<double>(1, 5) << d[0], d[1], d[2], d[3], d[4]);
    g_computedSessionId = sessionId;
    g_available = true;
    snprintf(g_activeProfile, sizeof(g_activeProfile), "%s", name);
    apply_to_processor();
    mirror_to_boot_intrinsics(persisted);
    return true;
}

int intrinsics_list_profiles(IntrinsicsProfileInfo* out, int max_out)
{
    if (!out || max_out <= 0) return 0;
    DIR* dir = opendir(PERSIST_DIR);
    if (!dir) return 0;
    int count = 0;
    const char* prefix = "intrinsics_";
    const size_t plen = strlen(prefix), suffix = 4;
    struct dirent* ent;
    while ((ent = readdir(dir)) && count < max_out) {
        const char* n = ent->d_name; size_t len = strlen(n);
        if (len <= plen + suffix || strncmp(n, prefix, plen) || strcmp(n + len - suffix, ".txt")) continue;
        size_t name_len = len - plen - suffix;
        if (name_len >= sizeof(out[count].name)) continue;
        memcpy(out[count].name, n + plen, name_len); out[count].name[name_len] = 0;
        out[count].session_id = 0;
        ++count;
    }
    closedir(dir);
    return count;
}

const char* intrinsics_active_profile(void) { return g_activeProfile; }

static void apply_to_processor(void)
{
    if (g_proc != NULL)
        g_proc->setCameraIntrinsics(g_K, g_dist);
}

void intrinsics_init(ArucoProcessor* proc)
{
    g_proc = proc;
    load_board_config();
    if (load_intrinsics()) {
        g_available = true;
        apply_to_processor();
        return;
    }
#if K_DEFAULT_INTRINSICS
    // Nothing persisted (fresh flash, or a /mnt write that never landed) --
    // fall back to the built-in calibration rather than leaving the detector
    // with no K. Not tagged to a session id, same as externally loaded values.
    g_K = (cv::Mat_<double>(3, 3) << K_DEFAULT_FX, 0, K_DEFAULT_CX,
                                     0, K_DEFAULT_FY, K_DEFAULT_CY,
                                     0, 0, 1);
    g_dist = (cv::Mat_<double>(1, 5) << K_DEFAULT_D0, K_DEFAULT_D1,
                                        K_DEFAULT_D2, K_DEFAULT_D3,
                                        K_DEFAULT_D4);
    g_available = true;
    g_computedSessionId = 0;
    snprintf(g_activeProfile, sizeof(g_activeProfile), "%s",
             K_DEFAULT_PROFILE_NAME);
    apply_to_processor();
#endif
}

bool intrinsics_available(void)
{
    return g_available;
}

void intrinsics_start_calib(void)
{
    g_allCorners.clear();
    g_allIds.clear();
    g_viewCenters.clear();
    g_viewCoverage.clear();
    g_failReason = "";
    g_prunedViews = 0;
    g_lastCaptureMs = 0;
    g_lastCaptureReason[0] = '\0';
    g_imageSize = cv::Size();
    g_state = IC_COLLECTING;
}

bool intrinsics_set_board_config(const CharucoBoardConfig& config,
                                 const char** reason_out)
{
    if (g_state == IC_COLLECTING) {
        if (reason_out) *reason_out = "cannot change board during a session";
        return false;
    }
    if (!valid_board_config(config, reason_out))
        return false;

    // Applied in RAM immediately; persisting to /mnt is a separate,
    // operator-triggered step (intrinsics_save_board_config(), e.g. a
    // BOARD_SAVE command) so applying a config can never fail because of a
    // storage write.
    g_boardConfig = config;
    ++g_boardGeneration;
    if (reason_out) *reason_out = NULL;
    return true;
}

CharucoBoardConfig intrinsics_get_board_config(void)
{
    return g_boardConfig;
}

bool intrinsics_collecting(void)
{
    return g_state == IC_COLLECTING;
}

void intrinsics_set_params(int targetViews, double rmsLimit)
{
    // Ignore non-positive values (a partial command must not zero the target
    // or disable the RMS gate); any positive count the operator asks for is
    // honoured, including small ones for quick tests.
    if (targetViews >= kMinViews)
        g_targetViews = targetViews;
    if (rmsLimit > 0.0)
        g_rmsLimit = rmsLimit;
}

int intrinsics_target_views(void) { return g_targetViews; }
double intrinsics_rms_limit(void) { return g_rmsLimit; }

// Detect ChArUco corners in this frame. Returns the corner/id vectors via out
// params; returns false only on a hard OpenCV exception.
static bool detect_charuco(const cv::Mat& gray,
                           std::vector<cv::Point2f>& charucoCorners,
                           std::vector<int>& charucoIds)
{
    try {
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f> > markerCorners;
        // Detector/board/params cached — same reuse rationale as ldc_checker.
#if ARUCO_NEW_API
        static unsigned int cachedGeneration = 0;
        static cv::Ptr<cv::aruco::CharucoDetector> detector;
        if (cachedGeneration != g_boardGeneration || detector.empty()) {
            cv::aruco::DetectorParameters detectorParams;
            detectorParams.cornerRefinementMethod =
                cv::aruco::CORNER_REFINE_SUBPIX;
            cv::aruco::CharucoBoard board(
                cv::Size(g_boardConfig.squares_x, g_boardConfig.squares_y),
                g_boardConfig.square_length_mm,
                g_boardConfig.marker_length_mm,
                cv::aruco::getPredefinedDictionary(
                    g_boardConfig.dictionary_id));
            detector = cv::makePtr<cv::aruco::CharucoDetector>(
                board, cv::aruco::CharucoParameters(), detectorParams);
            cachedGeneration = g_boardGeneration;
        }
        detector->detectBoard(gray, charucoCorners, charucoIds,
                              markerCorners, markerIds);
#else
        static unsigned int cachedGeneration = 0;
        static cv::Ptr<cv::aruco::Dictionary> dict;
        static cv::Ptr<cv::aruco::CharucoBoard> board;
        static cv::Ptr<cv::aruco::DetectorParameters> params;
        if (cachedGeneration != g_boardGeneration || board.empty()) {
            dict = cv::aruco::getPredefinedDictionary(
                g_boardConfig.dictionary_id);
            board = make_board();
            params = cv::aruco::DetectorParameters::create();
            params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
            cachedGeneration = g_boardGeneration;
        }
        cv::aruco::detectMarkers(gray, dict, markerCorners, markerIds, params);
        if (!markerIds.empty())
            cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, gray,
                                                 board, charucoCorners, charucoIds);
#endif
    } catch (const cv::Exception&) {
        return false;
    }
    return true;
}

bool intrinsics_detect_charuco(const cv::Mat& gray,
                               std::vector<cv::Point2f>& corners,
                               std::vector<int>& ids)
{
    return detect_charuco(gray, corners, ids);
}

static void set_quality_reason(IntrCalibViewQuality* q, const char* reason)
{
    snprintf(g_lastCaptureReason, sizeof(g_lastCaptureReason), "%s",
             reason ? reason : "");
    if (q) q->reason = g_lastCaptureReason;
}

static double view_sharpness(const cv::Mat& gray,
                             const std::vector<cv::Point2f>& corners)
{
    cv::Rect roi = cv::boundingRect(corners);
    roi.x = std::max(0, roi.x);
    roi.y = std::max(0, roi.y);
    roi.width = std::min(gray.cols - roi.x, roi.width);
    roi.height = std::min(gray.rows - roi.y, roi.height);
    if (roi.width < 8 || roi.height < 8)
        return 0.0;

    cv::Mat lap;
    cv::Laplacian(gray(roi), lap, CV_64F);
    cv::Scalar mean, stddev;
    cv::meanStdDev(lap, mean, stddev);
    return stddev[0] * stddev[0];
}

static double view_coverage(const cv::Size& imageSize,
                            const std::vector<cv::Point2f>& corners)
{
    std::vector<cv::Point2f> hull;
    cv::convexHull(corners, hull);
    if (hull.size() < 3 || imageSize.area() <= 0)
        return 0.0;
    return std::fabs(cv::contourArea(hull)) /
           static_cast<double>(imageSize.area());
}

static double mean_common_corner_move(
    const std::vector<cv::Point2f>& corners,
    const std::vector<int>& ids)
{
    double best = -1.0;
    for (size_t v = 0; v < g_allCorners.size(); ++v) {
        std::map<int, cv::Point2f> oldById;
        for (size_t i = 0; i < g_allIds[v].size(); ++i)
            oldById[g_allIds[v][i]] = g_allCorners[v][i];

        double sum = 0.0;
        int common = 0;
        for (size_t i = 0; i < ids.size(); ++i) {
            std::map<int, cv::Point2f>::const_iterator it =
                oldById.find(ids[i]);
            if (it == oldById.end())
                continue;
            sum += cv::norm(corners[i] - it->second);
            ++common;
        }
        if (common >= 4) {
            double mean = sum / common;
            if (best < 0.0 || mean < best)
                best = mean;
        }
    }
    return best;
}

IntrCalibState intrinsics_capture_view(const cv::Mat& gray,
                                       IntrCalibViewQuality* quality_out)
{
    IntrCalibViewQuality local;
    memset(&local, 0, sizeof(local));
    local.mean_move_px = -1.0;
    local.reason = "";
    IntrCalibViewQuality* q = quality_out ? quality_out : &local;
    *q = local;

    if (g_state != IC_COLLECTING) {
        set_quality_reason(q, "start a calibration session first");
        return IC_CAPTURE_REJECTED;
    }

    if (gray.empty()) {
        set_quality_reason(q, "empty frame");
        return IC_CAPTURE_REJECTED;
    }
    if (g_imageSize.area() > 0 && gray.size() != g_imageSize) {
        set_quality_reason(q, "frame resolution changed during session");
        return IC_CAPTURE_REJECTED;
    }

    std::vector<cv::Point2f> charucoCorners;
    std::vector<int> charucoIds;
    if (!detect_charuco(gray, charucoCorners, charucoIds)) {
        set_quality_reason(q, "OpenCV board detection error");
        return IC_CAPTURE_REJECTED;
    }

    const int found = (int) charucoIds.size();
    const int total =
        (g_boardConfig.squares_x - 1) * (g_boardConfig.squares_y - 1);
    // With gates off, only require the mathematical minimum (>=4 points) so a
    // view is never rejected for being small/partial.
    const int minCorners = g_qualityGates
        ? std::max(8, (int) std::ceil(total * K_CALIB_MIN_CORNER_RATIO))
        : 4;
    q->corners_found = found;
    q->corners_total = total;
    if (found < minCorners) {
        char reason[96];
        snprintf(reason, sizeof(reason), "need >= %d/%d corners", minCorners,
                 total);
        set_quality_reason(q, reason);
        return IC_CAPTURE_REJECTED;
    }

    for (size_t i = 0; i < charucoCorners.size(); ++i) {
        const cv::Point2f& p = charucoCorners[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
            p.x < 0.f || p.y < 0.f || p.x >= gray.cols || p.y >= gray.rows) {
            set_quality_reason(q, "invalid or out-of-frame corner");
            return IC_CAPTURE_REJECTED;
        }
    }

    q->coverage_ratio = view_coverage(gray.size(), charucoCorners);
    if (g_qualityGates && q->coverage_ratio < K_CALIB_MIN_COVERAGE) {
        set_quality_reason(q, "board is too small/far in the image");
        return IC_CAPTURE_REJECTED;
    }

    q->sharpness = view_sharpness(gray, charucoCorners);
    if (g_qualityGates && q->sharpness < K_CALIB_MIN_SHARPNESS) {
        set_quality_reason(q, "image is blurred; hold the board still");
        return IC_CAPTURE_REJECTED;
    }

    const long long now = monotonic_ms();
    if (g_lastCaptureMs > 0 &&
        now - g_lastCaptureMs < K_CALIB_MIN_GAP_MS) {
        set_quality_reason(q, "capture requested too soon after previous view");
        return IC_CAPTURE_REJECTED;
    }

    q->mean_move_px = mean_common_corner_move(charucoCorners, charucoIds);
    if (g_qualityGates && q->mean_move_px >= 0.0 &&
        q->mean_move_px < K_CALIB_MIN_MOVE_PX) {
        set_quality_reason(q, "pose duplicates an already accepted view");
        return IC_CAPTURE_REJECTED;
    }

    g_allCorners.push_back(charucoCorners);
    g_allIds.push_back(charucoIds);
    g_imageSize = gray.size();
    cv::Point2f center(0.f, 0.f);
    for (size_t i = 0; i < charucoCorners.size(); ++i)
        center += charucoCorners[i];
    center *= 1.f / (float) charucoCorners.size();
    g_viewCenters.push_back(center);
    g_viewCoverage.push_back(q->coverage_ratio);
    g_lastCaptureMs = now;
    set_quality_reason(q, "accepted");

    return IC_CAPTURED;
}

bool intrinsics_undo_last_view(void)
{
    if (g_state != IC_COLLECTING || g_allCorners.empty())
        return false;
    g_allCorners.pop_back();
    g_allIds.pop_back();
    g_viewCenters.pop_back();
    g_viewCoverage.pop_back();
    g_lastCaptureMs = 0;
    return true;
}

bool intrinsics_last_view_corners(std::vector<cv::Point2f>& out,
                                  std::vector<int>* ids_out)
{
    if (g_allCorners.empty())
        return false;
    out = g_allCorners.back();
    if (ids_out != NULL)
        *ids_out = g_allIds.back();   // pushed in lockstep by capture_view()
    return true;
}

static bool session_has_pose_diversity(char* reason, size_t reasonSize)
{
    if (g_viewCenters.empty() || g_viewCoverage.empty())
        return false;

    float minX = g_viewCenters[0].x, maxX = minX;
    float minY = g_viewCenters[0].y, maxY = minY;
    double minCoverage = g_viewCoverage[0];
    double maxCoverage = minCoverage;
    for (size_t i = 1; i < g_viewCenters.size(); ++i) {
        minX = std::min(minX, g_viewCenters[i].x);
        maxX = std::max(maxX, g_viewCenters[i].x);
        minY = std::min(minY, g_viewCenters[i].y);
        maxY = std::max(maxY, g_viewCenters[i].y);
        minCoverage = std::min(minCoverage, g_viewCoverage[i]);
        maxCoverage = std::max(maxCoverage, g_viewCoverage[i]);
    }

    if (maxX - minX < g_imageSize.width * 0.25f) {
        snprintf(reason, reasonSize,
                 "view centers need more left/right spread");
        return false;
    }
    if (maxY - minY < g_imageSize.height * 0.20f) {
        snprintf(reason, reasonSize,
                 "view centers need more top/bottom spread");
        return false;
    }
    if (minCoverage <= 0.0 || maxCoverage / minCoverage < 1.35) {
        snprintf(reason, reasonSize,
                 "capture more distance/scale variation");
        return false;
    }
    return true;
}

static void build_calibration_points(
    const std::vector<size_t>& active,
    std::vector<std::vector<cv::Point3f> >& objPts,
    std::vector<std::vector<cv::Point2f> >& imgPts)
{
    objPts.clear();
    imgPts.clear();
#if ARUCO_NEW_API
    cv::aruco::CharucoBoard board(
        cv::Size(g_boardConfig.squares_x, g_boardConfig.squares_y),
        g_boardConfig.square_length_mm, g_boardConfig.marker_length_mm,
        cv::aruco::getPredefinedDictionary(g_boardConfig.dictionary_id));
    for (size_t a = 0; a < active.size(); ++a) {
        std::vector<cv::Point3f> o;
        std::vector<cv::Point2f> i;
        size_t v = active[a];
        board.matchImagePoints(g_allCorners[v], g_allIds[v], o, i);
        if (o.size() >= 4) {
            objPts.push_back(o);
            imgPts.push_back(i);
        }
    }
#else
    cv::Ptr<cv::aruco::CharucoBoard> board = make_board();
    for (size_t a = 0; a < active.size(); ++a) {
        size_t v = active[a];
        std::vector<cv::Point3f> o;
        std::vector<cv::Point2f> i;
        for (size_t p = 0; p < g_allIds[v].size(); ++p) {
            int id = g_allIds[v][p];
            if (id >= 0 && id < (int) board->chessboardCorners.size()) {
                o.push_back(board->chessboardCorners[id]);
                i.push_back(g_allCorners[v][p]);
            }
        }
        if (o.size() >= 4) {
            objPts.push_back(o);
            imgPts.push_back(i);
        }
    }
#endif
}

static double per_view_rms(const std::vector<cv::Point3f>& objectPoints,
                           const std::vector<cv::Point2f>& imagePoints,
                           const cv::Mat& rvec, const cv::Mat& tvec,
                           const cv::Mat& K, const cv::Mat& dist)
{
    std::vector<cv::Point2f> projected;
    cv::projectPoints(objectPoints, rvec, tvec, K, dist, projected);
    if (projected.empty())
        return 1e9;
    double sumSq = 0.0;
    for (size_t i = 0; i < projected.size(); ++i) {
        cv::Point2f d = projected[i] - imagePoints[i];
        sumSq += d.dot(d);
    }
    return std::sqrt(sumSq / projected.size());
}

IntrCalibState intrinsics_compute(void)
{
    static char reason[160];
    if (g_state != IC_COLLECTING) {
        g_failReason = "start a calibration session first";
        return IC_DONE_FAIL;
    }
    if ((int) g_allCorners.size() < kMinViews) {
        snprintf(reason, sizeof(reason), "need >= %d accepted views, have %d",
                 kMinViews, (int) g_allCorners.size());
        g_failReason = reason;
        g_state = IC_COLLECTING;
        return IC_DONE_FAIL;
    }

    if (g_qualityGates && !session_has_pose_diversity(reason, sizeof(reason))) {
        g_failReason = reason;
        g_state = IC_COLLECTING;
        return IC_DONE_FAIL;
    }

    std::vector<size_t> active;
    for (size_t i = 0; i < g_allCorners.size(); ++i)
        active.push_back(i);

    cv::Mat bestK, bestDist;
    double bestRms = 1e9;
    g_prunedViews = 0;

    // Refit after removing the worst reprojection outlier. This is bounded by
    // the number of captured views and runs only on explicit Compute.
    try {
        while ((int) active.size() >= kMinViews) {
            std::vector<std::vector<cv::Point3f> > objPts;
            std::vector<std::vector<cv::Point2f> > imgPts;
            build_calibration_points(active, objPts, imgPts);
            if ((int) objPts.size() < kMinViews) {
                g_failReason = "not enough usable views after point matching";
                g_state = IC_COLLECTING;
                return IC_DONE_FAIL;
            }

            cv::Mat K, dist;
            std::vector<cv::Mat> rvecs, tvecs;
            double rms = cv::calibrateCamera(
                objPts, imgPts, g_imageSize, K, dist, rvecs, tvecs);

            double worst = -1.0;
            size_t worstIndex = 0;
            for (size_t v = 0; v < objPts.size(); ++v) {
                double viewRms = per_view_rms(
                    objPts[v], imgPts[v], rvecs[v], tvecs[v], K, dist);
                if (viewRms > worst) {
                    worst = viewRms;
                    worstIndex = v;
                }
            }

            bestK = K;
            bestDist = dist;
            bestRms = rms;
            if (!g_qualityGates) // gates off: accept the fit, no RMS pruning/fail
                break;
            if (worst <= K_CALIB_VIEW_RMS_LIMIT)
                break;
            if ((int) active.size() == kMinViews) {
                snprintf(reason, sizeof(reason),
                         "worst view RMS %.2fpx exceeds %.2fpx; recapture session",
                         worst, (double) K_CALIB_VIEW_RMS_LIMIT);
                g_failReason = reason;
                g_state = IC_COLLECTING;
                return IC_DONE_FAIL;
            }
            active.erase(active.begin() + worstIndex);
            ++g_prunedViews;
        }
    } catch (const cv::Exception& e) {
        snprintf(reason, sizeof(reason), "OpenCV calibration failed: %.90s",
                 e.what());
        g_failReason = reason;
        g_state = IC_COLLECTING;
        return IC_DONE_FAIL;
    }

    // A non-finite RMS means the fit is garbage (always fail); the over-limit
    // check is a quality gate (skipped when gates are off).
    if (!std::isfinite(bestRms) || (g_qualityGates && bestRms > g_rmsLimit)) {
        snprintf(reason, sizeof(reason), "RMS %.2fpx over limit %.2fpx",
                 bestRms, g_rmsLimit);
        g_failReason = reason;
        g_state = IC_COLLECTING;
        return IC_DONE_FAIL;
    }
    if (bestK.empty() || bestDist.total() < 5) {
        g_failReason = "OpenCV returned incomplete calibration matrices";
        g_state = IC_COLLECTING;
        return IC_DONE_FAIL;
    }

    // Keep only views that survived robust per-view outlier pruning.
    std::vector<std::vector<cv::Point2f> > keptCorners;
    std::vector<std::vector<int> > keptIds;
    std::vector<cv::Point2f> keptCenters;
    std::vector<double> keptCoverage;
    for (size_t i = 0; i < active.size(); ++i) {
        keptCorners.push_back(g_allCorners[active[i]]);
        keptIds.push_back(g_allIds[active[i]]);
        keptCenters.push_back(g_viewCenters[active[i]]);
        keptCoverage.push_back(g_viewCoverage[active[i]]);
    }
    g_allCorners.swap(keptCorners);
    g_allIds.swap(keptIds);
    g_viewCenters.swap(keptCenters);
    g_viewCoverage.swap(keptCoverage);

    g_K = bestK;
    g_dist = bestDist.reshape(1, 1).colRange(0, 5).clone();
    g_rms = bestRms;
    g_available = true;
    g_computedSessionId = g_pendingSessionId; // freeze: THIS compute's session
    // These numbers are no longer any named profile's -- clear the label so the
    // dashboard (and the profile name written into INTRINSICS_FILE on save)
    // cannot keep crediting a profile that holds different values.
    snprintf(g_activeProfile, sizeof(g_activeProfile), "기본");
    apply_to_processor();
    // Persisting to /mnt is a separate, operator-triggered step
    // (intrinsics_save(), e.g. a CALIB_K_SAVE command) -- not an automatic
    // side effect of a successful compute. K/dist is usable immediately
    // either way (apply_to_processor() above already applied it in RAM).
    g_state = IC_IDLE;
    return IC_DONE_OK;
}

int intrinsics_views(void)
{
    return (int) g_allCorners.size();
}

int intrinsics_pruned_views(void)
{
    return g_prunedViews;
}

double intrinsics_rms(void)
{
    return g_rms;
}

const char* intrinsics_fail_reason(void)
{
    return g_failReason;
}

bool intrinsics_get(double* fx, double* fy, double* cx, double* cy,
                    double dist[5])
{
    if (!g_available)
        return false;
    if (fx) *fx = g_K.at<double>(0, 0);
    if (fy) *fy = g_K.at<double>(1, 1);
    if (cx) *cx = g_K.at<double>(0, 2);
    if (cy) *cy = g_K.at<double>(1, 2);
    if (dist)
        for (int i = 0; i < 5; ++i)
            dist[i] = g_dist.at<double>(0, i);
    return true;
}

void intrinsics_set_session_id(long id)
{
    g_pendingSessionId = id;
}

long intrinsics_session_id(void)
{
    return g_computedSessionId;
}

bool intrinsics_save(void)
{
    if (!g_available) {
        g_failReason = "no calibration available yet — run CALIB_K_COMPUTE first";
        return false;
    }
    if (!save_intrinsics()) {
        g_failReason = "write to " PERSIST_DIR " failed (see Shell tab /mnt checks)";
        return false;
    }
    return true;
}

bool intrinsics_save_board_config(void)
{
    if (!save_board_config()) {
        g_failReason = "write to " PERSIST_DIR " failed (see Shell tab /mnt checks)";
        return false;
    }
    return true;
}
