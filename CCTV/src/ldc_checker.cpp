#include "ldc_checker.h"
#include "app_config.h"
#include "intrinsics_calibrator.h"

#include <math.h>
#include <map>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>

// OpenCV moved aruco/charuco into the objdetect module in 4.7 and reworked the
// API (CharucoDetector vs the free interpolateCornersCharuco function).
#define ARUCO_NEW_API (CV_VERSION_MAJOR > 4 || \
                       (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7))
#if ARUCO_NEW_API
#include <opencv2/objdetect/charuco_detector.hpp>
#else
#include <opencv2/aruco/charuco.hpp>
#endif

static bool g_active = false;

static bool same_board(const CharucoBoardConfig& a,
                       const CharucoBoardConfig& b)
{
    return a.squares_x == b.squares_x &&
           a.squares_y == b.squares_y &&
           a.square_length_mm == b.square_length_mm &&
           a.marker_length_mm == b.marker_length_mm &&
           a.dictionary_id == b.dictionary_id;
}

void ldc_check_start(void) { g_active = true; }
void ldc_check_stop(void)  { g_active = false; }
bool ldc_check_active(void) { return g_active; }

// Perpendicular distance (px) from p to the line through (x0,y0) with unit
// direction (vx,vy). cv::fitLine returns a normalized direction.
static double perp_dist(const cv::Point2f& p, float vx, float vy,
                        float x0, float y0)
{
    return std::fabs((p.x - x0) * vy - (p.y - y0) * vx);
}

// Fit a line to pts and fold each point's perpendicular deviation into the
// running RMS/max, tagging each sample edge/center by its normalized radial
// position from the image center.
static void accumulate_line(const std::vector<cv::Point2f>& pts,
                            double cx, double cy, double inv_half_diag,
                            double* sumSq, int* n, double* maxAll,
                            double* maxEdge, double* maxCenter)
{
    if ((int) pts.size() < LDC_MIN_LINE_POINTS)
        return;
    cv::Vec4f ln;
    cv::fitLine(pts, ln, cv::DIST_L2, 0, 0.01, 0.01);
    const float vx = ln[0], vy = ln[1], x0 = ln[2], y0 = ln[3];
    for (size_t i = 0; i < pts.size(); ++i) {
        double d = perp_dist(pts[i], vx, vy, x0, y0);
        *sumSq += d * d;
        ++(*n);
        if (d > *maxAll)
            *maxAll = d;
        double r = std::sqrt((pts[i].x - cx) * (pts[i].x - cx) +
                             (pts[i].y - cy) * (pts[i].y - cy)) * inv_half_diag;
        if (r >= LDC_EDGE_RADIUS_FRAC) {
            if (d > *maxEdge) *maxEdge = d;
        } else {
            if (d > *maxCenter) *maxCenter = d;
        }
    }
}

// Group corners into physical rows/cols by ChArUco id and measure the
// straightness (RMS/max/edge/center) of the fitted lines. Shared by the raw
// and undistorted passes in measure_frame() — same ids, different points.
static bool compute_straightness(const std::vector<cv::Point2f>& corners,
                                 const std::vector<int>& ids,
                                 int imgW, int imgH, int gridW,
                                 int cornersTotal,
                                 double* rms, double* smax,
                                 double* emax, double* cmax)
{
    std::map<int, std::vector<cv::Point2f> > rows, cols;
    for (size_t i = 0; i < ids.size(); ++i) {
        int id = ids[i];
        if (id < 0 || id >= cornersTotal)
            continue;
        int r = id / gridW;
        int c = id % gridW;
        rows[r].push_back(corners[i]);
        cols[c].push_back(corners[i]);
    }

    const double cx = imgW * 0.5;
    const double cy = imgH * 0.5;
    const double halfDiag = 0.5 * std::sqrt((double) imgW * imgW +
                                            (double) imgH * imgH);
    const double inv_half_diag = halfDiag > 0.0 ? 1.0 / halfDiag : 0.0;

    double sumSq = 0.0, maxAll = 0.0, maxEdge = 0.0, maxCenter = 0.0;
    int n = 0;

    for (std::map<int, std::vector<cv::Point2f> >::const_iterator it =
             rows.begin(); it != rows.end(); ++it)
        accumulate_line(it->second, cx, cy, inv_half_diag,
                        &sumSq, &n, &maxAll, &maxEdge, &maxCenter);
    for (std::map<int, std::vector<cv::Point2f> >::const_iterator it =
             cols.begin(); it != cols.end(); ++it)
        accumulate_line(it->second, cx, cy, inv_half_diag,
                        &sumSq, &n, &maxAll, &maxEdge, &maxCenter);

    if (n == 0)
        return false;

    *rms = std::sqrt(sumSq / n);
    *smax = maxAll;
    *emax = maxEdge;
    *cmax = maxCenter;
    return true;
}

// The actual measurement, independent of the LDC_CHECK_START/STOP mode flag.
// Used both by ldc_check_feed() (gated on g_active, streamed every frame) and
// by the one-shot LDC_SNAPSHOT command (measures regardless of mode).
static LdcCheckState measure_frame(const cv::Mat& gray, LdcResult* out)
{
    const CharucoBoardConfig config = intrinsics_get_board_config();
    const int gridW = config.squares_x - 1;
    const int gridH = config.squares_y - 1;
    const int cornersTotal = gridW * gridH;
    const int markersTotal = (config.squares_x * config.squares_y) / 2;

    // Fill the counts up front so a partial board still reports coverage.
    if (out) {
        out->markers_found = 0;
        out->markers_total = markersTotal;
        out->corners_found = 0;
        out->corners_total = cornersTotal;
        out->straight_rms_px = 0.0;
        out->straight_max_px = 0.0;
        out->edge_max_px = 0.0;
        out->center_max_px = 0.0;
        out->has_undistorted = false;
        out->straight_rms_px_u = 0.0;
        out->straight_max_px_u = 0.0;
        out->edge_max_px_u = 0.0;
        out->center_max_px_u = 0.0;
    }

    // Everything below — detection, interpolation, and cv::fitLine — runs
    // inside the SDK's per-frame event callback, which has no exception
    // handler above it. An uncaught cv::Exception from ANY of these calls
    // previously took the whole app down (observed as all TCP traffic,
    // including heartbeats, stopping dead mid LDC_CHECK session — e.g. when
    // the board neared the frame edge and corner extraction got degenerate).
    // So the whole body is wrapped: any bad frame is reported as "no board"
    // rather than crashing the app.
    try {
        std::vector<int> markerIds, charucoIds;
        std::vector<std::vector<cv::Point2f> > markerCorners;
        std::vector<cv::Point2f> charucoCorners;

        // Board/dictionary/detector are immutable config (compile-time board
        // geometry) — built once on the first call, reused every frame. This
        // path streams per frame while LDC_CHECK is active, so per-call
        // construction was pure overhead.
#if ARUCO_NEW_API
        static bool cached = false;
        static CharucoBoardConfig cachedConfig;
        static cv::Ptr<cv::aruco::CharucoDetector> detector;
        if (!cached || !same_board(config, cachedConfig)) {
            cv::aruco::DetectorParameters detectorParams;
            detectorParams.cornerRefinementMethod =
                cv::aruco::CORNER_REFINE_SUBPIX;
            cv::aruco::CharucoBoard board(
                cv::Size(config.squares_x, config.squares_y),
                config.square_length_mm, config.marker_length_mm,
                cv::aruco::getPredefinedDictionary(config.dictionary_id));
            detector = cv::makePtr<cv::aruco::CharucoDetector>(
                board, cv::aruco::CharucoParameters(), detectorParams);
            cachedConfig = config;
            cached = true;
        }
        detector->detectBoard(gray, charucoCorners, charucoIds,
                              markerCorners, markerIds);
#else
        static bool cached = false;
        static CharucoBoardConfig cachedConfig;
        static cv::Ptr<cv::aruco::Dictionary> dict;
        static cv::Ptr<cv::aruco::CharucoBoard> board;
        static cv::Ptr<cv::aruco::DetectorParameters> params;
        if (!cached || !same_board(config, cachedConfig)) {
            dict = cv::aruco::getPredefinedDictionary(config.dictionary_id);
            board = cv::aruco::CharucoBoard::create(
                config.squares_x, config.squares_y,
                config.square_length_mm, config.marker_length_mm, dict);
            params = cv::aruco::DetectorParameters::create();
            params->cornerRefinementMethod =
                cv::aruco::CORNER_REFINE_SUBPIX;
            cachedConfig = config;
            cached = true;
        }
        cv::aruco::detectMarkers(gray, dict, markerCorners, markerIds, params);
        if (!markerIds.empty())
            cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, gray,
                                                 board, charucoCorners, charucoIds);
#endif

        const int markersFound = (int) markerIds.size();
        const int cornersFound = (int) charucoIds.size();
        if (out) {
            out->markers_found = markersFound;
            out->corners_found = cornersFound;
        }
        if (markersFound == 0 || cornersFound < LDC_MIN_LINE_POINTS)
            return LC_NO_BOARD;

        // Raw straightness (camera-LDC-only, no OpenCV undistort applied).
        double rms, smax, emax, cmax;
        if (!compute_straightness(charucoCorners, charucoIds,
                                  gray.cols, gray.rows, gridW, cornersTotal,
                                  &rms, &smax, &emax, &cmax))
            return LC_NO_BOARD; // not enough of the board formed a line

        if (out) {
            out->straight_rms_px = rms;
            out->straight_max_px = smax;
            out->edge_max_px = emax;
            out->center_max_px = cmax;

            // Raw corner positions + edge/center classification, for the
            // caller to draw an overlay (colors line up with edge_max_px vs
            // center_max_px above).
            const double cx = gray.cols * 0.5;
            const double cy = gray.rows * 0.5;
            const double halfDiag = 0.5 * std::sqrt((double) gray.cols * gray.cols +
                                                    (double) gray.rows * gray.rows);
            const double inv_half_diag = halfDiag > 0.0 ? 1.0 / halfDiag : 0.0;
            out->corners = charucoCorners;
            out->is_edge.resize(charucoCorners.size());
            for (size_t i = 0; i < charucoCorners.size(); ++i) {
                double r = std::sqrt((charucoCorners[i].x - cx) * (charucoCorners[i].x - cx) +
                                     (charucoCorners[i].y - cy) * (charucoCorners[i].y - cy)) *
                          inv_half_diag;
                out->is_edge[i] = (r >= LDC_EDGE_RADIUS_FRAC);
            }
        }

        // Before/after comparison: if a calibration exists, undistort the
        // SAME corners and re-measure — this is the direct "LDC alone vs
        // LDC + OpenCV calibration" straightness comparison.
        double fx, fy, cx0, cy0, dist[5];
        if (out && intrinsics_available() &&
            intrinsics_get(&fx, &fy, &cx0, &cy0, dist)) {
            cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx0, 0, fy, cy0, 0, 0, 1);
            cv::Mat D = (cv::Mat_<double>(1, 5) << dist[0], dist[1], dist[2],
                        dist[3], dist[4]);
            std::vector<cv::Point2f> undist;
            // P=K reprojects back into pixel space (not normalized coords),
            // so the result is directly comparable in px to the raw pass.
            cv::undistortPoints(charucoCorners, undist, K, D, cv::noArray(), K);

            double urms, usmax, uemax, ucmax;
            if (compute_straightness(undist, charucoIds, gray.cols, gray.rows,
                                     gridW, cornersTotal,
                                     &urms, &usmax, &uemax, &ucmax)) {
                out->has_undistorted = true;
                out->straight_rms_px_u = urms;
                out->straight_max_px_u = usmax;
                out->edge_max_px_u = uemax;
                out->center_max_px_u = ucmax;
            }
        }

        return LC_MEASURED;
    } catch (const cv::Exception&) {
        return LC_NO_BOARD;
    }
}

LdcCheckState ldc_check_feed(const cv::Mat& gray, LdcResult* out)
{
    if (!g_active)
        return LC_IDLE;
    return measure_frame(gray, out);
}

LdcCheckState ldc_measure_once(const cv::Mat& gray, LdcResult* out)
{
    return measure_frame(gray, out);
}
