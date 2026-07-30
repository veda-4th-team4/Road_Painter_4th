#include "homography_mapper.h"
#include "app_config.h"
#include "intrinsics_calibrator.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#include <opencv2/calib3d.hpp>

// Anchor DEFAULTS, calibration thresholds and the persistence path all live in
// app_config.h — edit them there, not here. The default world positions are
// copied once into a mutable runtime table (g_anchors) so the dashboard can
// re-measure a site with ANCHOR_SET_ALL without a rebuild. Keep an explicit
// bounded vector: calibration uses the current entry count, rather than a
// compile-time eight-slot table.
static std::vector<AnchorConfig> g_anchors;
static bool                      g_anchorsInit = false;
static std::vector<AnchorConfig> g_validation;
static bool                      g_validationInit = false;

static void ensure_anchors(void)
{
    if (g_anchorsInit)
        return;
    const int n = (int) (sizeof(kAnchorTable) / sizeof(kAnchorTable[0]));
    for (int i = 0; i < n && i < HOMOGRAPHY_MAX_ANCHORS; ++i)
        g_anchors.push_back(kAnchorTable[i]);
    g_anchorsInit = true;
}

static void ensure_validation(void)
{
    if (g_validationInit)
        return;
    const int n = (int) (sizeof(kValidationTable) / sizeof(kValidationTable[0]));
    for (int i = 0; i < n && i < HOMOGRAPHY_MAX_VALIDATION_MARKERS; ++i)
        g_validation.push_back(kValidationTable[i]);
    g_validationInit = true;
}

static const int kCalibGoodFrames = CALIB_GOOD_FRAMES;
static const int kCalibMaxFrames  = CALIB_MAX_FRAMES;
static const char* kHomographyFile = HOMOGRAPHY_FILE;

static cv::Mat g_H;                 // 3x3 CV_64F, image px -> world mm
static bool    g_active = false;
static bool    g_undistort = false;
// Mode read from the persisted H file, applied by homography_init() once the
// intrinsics are in place. Separate from g_undistort so a file asking for
// undistort without K present cannot half-apply.
static bool    g_loadedUndistort = false;
static HomographyCalibState g_state = HG_IDLE;
static int     g_goodFrames  = 0;
static int     g_totalFrames = 0;
static double  g_sumX[HOMOGRAPHY_MAX_ANCHORS]; // accumulated anchor centers (px)
static double  g_sumY[HOMOGRAPHY_MAX_ANCHORS];
static const char* g_failReason = "";

static bool h_pixel(float x, float y, cv::Point2f* out)
{
    if (out == NULL) return false;
    if (!g_undistort) { *out = cv::Point2f(x, y); return true; }
    double fx, fy, cx, cy, d[5];
    if (!intrinsics_get(&fx, &fy, &cx, &cy, d)) return false;
    cv::Mat K = (cv::Mat_<double>(3,3) << fx,0,cx, 0,fy,cy, 0,0,1);
    cv::Mat D(1, 5, CV_64F, d);
    std::vector<cv::Point2f> in(1, cv::Point2f(x, y)), mapped;
    cv::undistortPoints(in, mapped, K, D, cv::noArray(), K);
    *out = mapped[0];
    return true;
}

static cv::Point2f marker_center(const ArucoProcessor::Detection& d)
{
    cv::Point2f c(0.f, 0.f);
    for (size_t i = 0; i < d.corners2d.size(); ++i)
        c += d.corners2d[i];
    return c * (1.0f / (float) d.corners2d.size());
}

static bool save_h(void)
{
#if !PERSIST_TO_MNT
    return true; // /mnt persistence disabled — homography kept in RAM only
#else
    FILE* f = fopen(kHomographyFile, "w");
    if (!f)
        return false;
    for (int r = 0; r < 3; ++r)
        fprintf(f, "%.10e %.10e %.10e\n",
                g_H.at<double>(r, 0), g_H.at<double>(r, 1), g_H.at<double>(r, 2));
    // Fourth line: which pixel space this H was fitted in (HG_COORD_MODE). An H
    // fitted on raw pixels and later used in undistort mode -- or the reverse --
    // fails silently: no error, just world coordinates that are wrong by the
    // lens distortion. Storing the mode WITH the matrix is what makes the pair
    // impossible to desync across a reboot. Files written before this line
    // existed simply end at line 3; load_h() reads those as raw, which is what
    // they were.
    fprintf(f, "%d\n", g_undistort ? 1 : 0);
    fclose(f);
    return true;
#endif
}

static bool load_h(void)
{
    FILE* f = fopen(kHomographyFile, "r");
    if (!f)
        return false;
    cv::Mat h(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r) {
        if (fscanf(f, "%lf %lf %lf",
                   &h.at<double>(r, 0), &h.at<double>(r, 1),
                   &h.at<double>(r, 2)) != 3) {
            fclose(f);
            return false;
        }
    }
    // Optional 4th line -- the pixel space the matrix was fitted in (see
    // save_h). Absent in files written before the mode existed; those were all
    // fitted on raw pixels, so defaulting to 0 restores them correctly.
    int undistort = 0;
    if (fscanf(f, "%d", &undistort) != 1)
        undistort = 0;
    fclose(f);
    g_H = h;
    g_loadedUndistort = (undistort != 0);
    return true;
}

void homography_init(void)
{
    g_active = load_h();
    // Apply the saved mode through the validating setter rather than assigning
    // g_undistort directly: if K is gone (calibration cleared, profile switched
    // to one without intrinsics) the setter refuses and we stay in raw mode.
    // Degrading to raw costs distortion-level accuracy; honouring the flag
    // without K would make every world lookup fail instead.
    if (g_active && g_loadedUndistort)
        homography_set_undistort(true);
}

bool homography_active(void)
{
    return g_active;
}

bool homography_set_undistort(bool on)
{
    if (g_state == HG_COLLECTING) return false;
    // Ask the intrinsics store directly. Probing through h_pixel() cannot work
    // here: it short-circuits on the CURRENT g_undistort, which is still false
    // while turning the mode on, so it returned success no matter what and this
    // guard never once refused. The mode then switched on without a K, and
    // every later lookup failed inside h_pixel instead -- CAM_POSE quietly lost
    // its "world" field with nothing reporting why.
    if (on) {
        double fx, fy, cx, cy, d[5];
        if (!intrinsics_get(&fx, &fy, &cx, &cy, d))
            return false;
    }
    g_undistort = on;
    return true;
}
bool homography_undistort_enabled(void) { return g_undistort; }
bool homography_prepare_pixel(float px, float py, cv::Point2f* out)
{ return h_pixel(px, py, out); }

void homography_start_calib(void)
{
    ensure_anchors();
    g_goodFrames  = 0;
    g_totalFrames = 0;
    memset(g_sumX, 0, sizeof(g_sumX));
    memset(g_sumY, 0, sizeof(g_sumY));
    g_failReason = "";
    g_state = HG_COLLECTING;
}

bool homography_collecting(void)
{
    return g_state == HG_COLLECTING;
}

HomographyCalibState homography_feed(
        const std::vector<ArucoProcessor::Detection>& dets)
{
    if (g_state != HG_COLLECTING)
        return g_state;

    ensure_anchors();
    const int numAnchors = (int) g_anchors.size();
    if (numAnchors < HOMOGRAPHY_MIN_ANCHORS) {
        g_state = HG_IDLE;
        g_failReason = "need at least 4 calculation anchors";
        return HG_DONE_FAIL;
    }
    ++g_totalFrames;

    // Locate every anchor in this frame (first occurrence per id).
    cv::Point2f centers[HOMOGRAPHY_MAX_ANCHORS];
    int found = 0;
    for (int a = 0; a < numAnchors; ++a) {
        for (size_t i = 0; i < dets.size(); ++i) {
            if (dets[i].id == g_anchors[a].id && dets[i].corners2d.size() >= 4) {
                if (h_pixel(marker_center(dets[i]).x, marker_center(dets[i]).y, &centers[a]))
                    ++found;
                break;
            }
        }
    }

    if (found == numAnchors) {
        for (int a = 0; a < numAnchors; ++a) {
            g_sumX[a] += centers[a].x;
            g_sumY[a] += centers[a].y;
        }
        ++g_goodFrames;
    }

    if (g_goodFrames >= kCalibGoodFrames) {
        std::vector<cv::Point2f> src, dst;
        for (int a = 0; a < numAnchors; ++a) {
            src.push_back(cv::Point2f((float) (g_sumX[a] / g_goodFrames),
                                      (float) (g_sumY[a] / g_goodFrames)));
            dst.push_back(cv::Point2f((float) g_anchors[a].wx,
                                      (float) g_anchors[a].wy));
        }
        // Four anchors are the geometric minimum; additional anchors make an
        // overdetermined fit where RANSAC can reject a bad observation.
        cv::Mat inliers;
        cv::Mat h = cv::findHomography(src, dst, cv::RANSAC, 20.0, inliers);
        g_state = HG_IDLE;
        if (h.empty()) {
            g_failReason = "findHomography failed (degenerate anchors?)";
            return HG_DONE_FAIL;
        }
        int inlierCount = 0;
        for (size_t i = 0; i < inliers.total(); ++i)
            inlierCount += inliers.at<unsigned char>((int) i) ? 1 : 0;
        if (inlierCount < HOMOGRAPHY_MIN_ANCHORS) {
            g_failReason = "too few inlier anchors (need at least 4)";
            return HG_DONE_FAIL;
        }
        g_H = h;
        g_active = true;
        // Persisting is a separate, operator-triggered step (homography_save(),
        // e.g. an HG_SAVE command) -- not an automatic side effect of a
        // successful calibration. H is usable immediately either way.
        return HG_DONE_OK;
    }

    if (g_totalFrames >= kCalibMaxFrames) {
        g_state = HG_IDLE;
        g_failReason = "not enough frames with all anchors visible";
        return HG_DONE_FAIL;
    }

    return HG_COLLECTING;
}

int homography_progress(void)
{
    return g_goodFrames;
}

const char* homography_fail_reason(void)
{
    return g_failReason;
}

bool homography_get(double h[9])
{
    if (!g_active || g_H.empty() || g_H.rows != 3 || g_H.cols != 3)
        return false;
    const double* src = (const double*) g_H.data; // row-major 3x3
    for (int i = 0; i < 9; ++i)
        h[i] = src[i];
    return true;
}

bool homography_set(const double h[9])
{
    if (h == NULL) {
        g_failReason = "missing 3x3 homography";
        return false;
    }
    if (g_state == HG_COLLECTING) {
        g_failReason = "finish on-camera calibration first";
        return false;
    }
    cv::Mat m(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) {
        if (!isfinite(h[i])) {
            g_failReason = "homography contains a non-finite value";
            return false;
        }
        m.at<double>(i / 3, i % 3) = h[i];
    }
    // A singular matrix cannot map image points to a stable ground plane.
    if (fabs(cv::determinant(m)) < 1e-12) {
        g_failReason = "homography matrix is singular";
        return false;
    }
    g_H = m;
    g_active = true;
    g_failReason = "";
    return true;
}

bool homography_save(void)
{
    if (!g_active || g_H.empty()) {
        g_failReason = "no homography available yet — run CALIB_START first";
        return false;
    }
    if (!save_h()) {
        g_failReason = "write to " PERSIST_DIR " failed (see Shell tab /mnt checks)";
        return false;
    }
    return true;
}

bool homography_pixel_to_world(float px, float py, double* wx, double* wy)
{
    if (!g_active || wx == NULL || wy == NULL)
        return false;

    cv::Point2f p;
    if (!h_pixel(px, py, &p)) return false;
    px = p.x; py = p.y;
    const double* h = (const double*) g_H.data; // row-major 3x3
    double u = h[0] * px + h[1] * py + h[2];
    double v = h[3] * px + h[4] * py + h[5];
    double w = h[6] * px + h[7] * py + h[8];
    if (w == 0.0)
        return false;
    *wx = u / w;
    *wy = v / w;
    return true;
}

bool homography_set_anchor(int id, double wx, double wy)
{
    ensure_anchors();
    if (g_state == HG_COLLECTING)
        return false; // world targets must not move mid-collection
    for (size_t i = 0; i < g_anchors.size(); ++i) {
        if (g_anchors[i].id == id) {
            g_anchors[i].wx = wx;
            g_anchors[i].wy = wy;
            return true;
        }
    }
    return false; // id not in the compiled anchor table
}

bool homography_set_anchor_slot(int slot, int id, double wx, double wy)
{
    ensure_anchors();
    if (g_state == HG_COLLECTING || slot < 0 || slot >= (int) g_anchors.size() || id < 0)
        return false;
    for (size_t i = 0; i < g_anchors.size(); ++i) {
        if (i != slot && g_anchors[i].id == id)
            return false; // one detected marker cannot serve two anchor slots
    }
    g_anchors[slot].id = id;
    g_anchors[slot].wx = wx;
    g_anchors[slot].wy = wy;
    return true;
}

bool homography_set_anchors(const AnchorConfig* entries, int count)
{
    ensure_anchors();
    if (g_state == HG_COLLECTING || entries == NULL ||
        count < HOMOGRAPHY_MIN_ANCHORS || count > HOMOGRAPHY_MAX_ANCHORS)
        return false;
    for (int i = 0; i < count; ++i) {
        if (entries[i].id < 0 || !isfinite(entries[i].wx) || !isfinite(entries[i].wy))
            return false;
        for (int j = 0; j < i; ++j)
            if (entries[i].id == entries[j].id)
                return false;
    }
    g_anchors.assign(entries, entries + count);
    return true;
}

int homography_get_anchors(AnchorConfig* out, int max)
{
    ensure_anchors();
    if (out == NULL || max <= 0)
        return 0;
    int n = ((int) g_anchors.size() < max) ? (int) g_anchors.size() : max;
    for (int i = 0; i < n; ++i)
        out[i] = g_anchors[i];
    return n;
}

int homography_get_validation_markers(AnchorConfig* out, int max)
{
    ensure_validation();
    if (out == NULL || max <= 0)
        return 0;
    int n = ((int) g_validation.size() < max) ? (int) g_validation.size() : max;
    for (int i = 0; i < n; ++i)
        out[i] = g_validation[i];
    return n;
}

bool homography_set_validation_markers(const AnchorConfig* entries, int count)
{
    ensure_anchors();
    if (g_state == HG_COLLECTING || count < 0 || count > HOMOGRAPHY_MAX_VALIDATION_MARKERS ||
        (count > 0 && entries == NULL))
        return false;
    for (int i = 0; i < count; ++i) {
        if (entries[i].id < 0)
            return false;
        for (size_t a = 0; a < g_anchors.size(); ++a)
            if (entries[i].id == g_anchors[a].id)
                return false;
        for (int j = 0; j < i; ++j)
            if (entries[i].id == entries[j].id)
                return false;
    }
    if (count == 0)
        g_validation.clear();
    else
        g_validation.assign(entries, entries + count);
    g_validationInit = true;
    return true;
}
