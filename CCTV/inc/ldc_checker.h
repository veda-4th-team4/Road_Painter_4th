#ifndef LDC_CHECKER_H
#define LDC_CHECKER_H

#include <opencv2/core.hpp>
#include <vector>

/**
 * Lens-distortion-correction (LDC) / calibration sanity check from a printed
 * ChArUco board (real-time diagnostic mode).
 *
 * Purpose: the camera's OWN distortion correction is already applied to the
 * raw frames. This mode does NOT calibrate (compute K/dist) — while an
 * operator holds a ChArUco board it reports, live, per frame:
 *   - how many ArUco markers were detected (board sanity / coverage)
 *   - how many chessboard corners were interpolated
 *   - the residual distortion, as the straightness (plumb-line) error of the
 *     board's interior corner rows/columns
 *
 * Principle: a distortion-free (pinhole) projection maps straight lines to
 * straight lines. A ChArUco board's interior corner rows and columns are
 * physically straight, so their residual bow in the image is exactly the
 * residual lens distortion — no known geometry, no intrinsics required.
 * Distortion grows toward the frame edge, so edge_max_px is reported
 * separately from center_max_px and is the number that decides sufficiency.
 *
 * Why ChArUco (vs a plain chessboard): partial occlusion / tilt tolerant, and
 * it yields a detected-marker count. It uses the same persisted runtime board
 * configuration as intrinsics calibration, including the dictionary.
 *
 * Flow (server commands LDC_CHECK_START / LDC_CHECK_STOP):
 *   1. ldc_check_start()  — enters a dedicated mode (ArUco streaming paused)
 *   2. every raw gray frame -> ldc_check_feed(); on a found board the metrics
 *      are filled in and LC_MEASURED is returned for live streaming
 *   3. ldc_check_stop()   — leaves the mode; normal streaming resumes
 *
 * Before/after comparison: whenever K/dist are available (intrinsics_
 * available(), i.e. CALIB_K_COMPUTE has succeeded), the SAME corners are also
 * passed through cv::undistortPoints() and re-measured. This gives a direct
 * "raw (camera LDC only) vs raw+OpenCV-undistort" straightness comparison in
 * one shot — the core question of the LDC-vs-OpenCV-calibration evaluation.
 * Before any calibration, only the raw numbers are filled (has_undistorted
 * stays false).
 */

enum LdcCheckState {
    LC_IDLE,      // not in check mode
    LC_NO_BOARD,  // in check mode but no usable board in this frame
    LC_MEASURED   // board found — metrics valid for this frame
};

struct LdcResult {
    int    markers_found;     // ArUco markers detected this frame
    int    markers_total;     // markers the full board carries
    int    corners_found;     // interpolated ChArUco corners this frame
    int    corners_total;     // interior corners the full board has

    // Raw (camera-LDC-only) straightness — always filled on LC_MEASURED.
    double straight_rms_px;   // RMS perpendicular deviation over all row/col lines
    double straight_max_px;   // worst single-corner deviation
    double edge_max_px;       // worst deviation among corners near the frame edge
    double center_max_px;     // worst deviation among corners near the center

    // Same metrics AFTER cv::undistortPoints() with the current K/dist.
    // Only valid when has_undistorted is true (i.e. a calibration exists).
    bool   has_undistorted;
    double straight_rms_px_u;
    double straight_max_px_u;
    double edge_max_px_u;
    double center_max_px_u;

    // Raw (as-detected, NOT undistorted) corner positions and their
    // edge/center classification, for drawing an overlay on the snapshot
    // image. is_edge[i] corresponds to corners[i] and uses the same
    // LDC_EDGE_RADIUS_FRAC threshold as edge_max_px/center_max_px, so the
    // colors on the image match those two numbers directly.
    std::vector<cv::Point2f> corners;
    std::vector<bool>        is_edge;
};

// Enter/leave the dedicated check mode.
void ldc_check_start(void);
void ldc_check_stop(void);
bool ldc_check_active(void);

// Feed the current raw gray frame. On LC_MEASURED, *out is filled. On
// LC_NO_BOARD, markers_found/markers_total are still filled (0..total) so the
// caller can show "board partially visible".
LdcCheckState ldc_check_feed(const cv::Mat& gray, LdcResult* out);

// Measure the current frame regardless of ldc_check_active() — used by the
// LDC_SNAPSHOT command to capture one frame's metrics on demand, independent
// of whether the live LDC_CHECK stream is running.
LdcCheckState ldc_measure_once(const cv::Mat& gray, LdcResult* out);

#endif // LDC_CHECKER_H
