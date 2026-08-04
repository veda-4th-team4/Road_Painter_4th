#ifndef INTRINSICS_CALIBRATOR_H
#define INTRINSICS_CALIBRATOR_H

#include <opencv2/core.hpp>

class ArucoProcessor;

/**
 * Runtime ChArUco board description.
 *
 * Lengths are millimetres. Camera intrinsics are scale-independent, but the
 * square/marker ratio and marker dictionary MUST exactly match the print.
 * The outer margins do not enter OpenCV's geometry; they are persisted and
 * reported so the operator can verify the physical sheet.
 */
struct CharucoBoardConfig {
    int squares_x;
    int squares_y;
    float square_length_mm;
    float marker_length_mm;
    int dictionary_id;
    float outer_margin_x_mm;
    float outer_margin_y_mm;
};

/** Quality measurements for the most recent capture request. */
struct IntrCalibViewQuality {
    int corners_found;
    int corners_total;
    double coverage_ratio;
    double sharpness;
    double mean_move_px;
    const char* reason;
};

/**
 * On-camera intrinsics (K, dist) calibration from a printed ChArUco board,
 * with MANUAL, operator-triggered capture.
 *
 * Why on-camera: K/dist are measured in EXACTLY the pixel space the app
 * detects ArUco in (the raw NV12 frames), so there is no resolution-scaling
 * mismatch with a server-side RTSP calibration.
 *
 * Why ChArUco (vs a plain chessboard): tolerant of partial occlusion/tilt and
 * it reuses the same board as the LDC check. Uses the board defined by the
 * CHARUCO_* values in app_config.h (must match the printed board).
 *
 * Why manual capture: the operator holds the board at a chosen pose and
 * presses a button (CALIB_K_CAPTURE) to store THAT view — full control over
 * pose variety (vary tilt/distance/position), which is what makes calibration
 * well-conditioned. Auto-accept is not used.
 *
 * Flow (server commands):
 *   CALIB_K_START   -> intrinsics_start_calib()   (reset, begin a session)
 *   CALIB_K_CAPTURE -> intrinsics_capture_view()  (store current frame's view)
 *   CALIB_K_COMPUTE -> intrinsics_compute()       (run calibrateCameraCharuco)
 * Capture never starts a blocking calibration implicitly. Once K_CALIB_VIEWS
 * quality-approved views are visible in the UI, the operator explicitly runs
 * CALIB_K_COMPUTE and receives a COMPUTING acknowledgement first.
 */

enum IntrCalibState {
    IC_IDLE,             // not calibrating
    IC_COLLECTING,       // session open, waiting for captures
    IC_CAPTURED,         // this capture was accepted and stored
    IC_CAPTURE_REJECTED, // this capture had too few corners; not stored
    IC_DONE_OK,          // compute succeeded (RMS within limit)
    IC_DONE_FAIL         // compute failed (too few views / bad RMS / threw)
};

// Load persisted K/dist (if any) and apply to the processor. Call at startup.
void intrinsics_init(ArucoProcessor* proc);

// True once valid intrinsics are loaded/calibrated.
bool intrinsics_available(void);

// Begin a new calibration session (clears any collected views).
void intrinsics_start_calib(void);

// Update the physical board description (RAM only). Rejected while
// collecting. Returns false and sets reason_out when values are invalid.
// Does not touch /mnt -- call intrinsics_save_board_config() to persist it.
bool intrinsics_set_board_config(const CharucoBoardConfig& config,
                                 const char** reason_out);

// Persist the currently active board config to PERSIST_DIR right now.
// Returns false if the write fails (see intrinsics_fail_reason()).
bool intrinsics_save_board_config(void);

// Read the active board description.
CharucoBoardConfig intrinsics_get_board_config(void);

// Detect the active ChArUco board in one raw grayscale frame.  This is shared
// by K/dist calibration and the floor-homography test, so both use exactly the
// same board config, dictionary and sub-pixel detector.
bool intrinsics_detect_charuco(const cv::Mat& gray,
                               std::vector<cv::Point2f>& corners,
                               std::vector<int>& ids);

// True while a calibration session is open (between CALIB_K_START and the
// compute that ends it). Used to pause pose streaming during calibration,
// the same way LDC check does.
bool intrinsics_collecting(void);

// Runtime session parameters (CALIB_K_SET): target accepted-view count and
// the overall reprojection-RMS pass limit. Non-positive values keep the
// current setting; targetViews below the internal minimum is also ignored.
void   intrinsics_set_params(int targetViews, double rmsLimit);
int    intrinsics_target_views(void);

// Runtime on/off for the calibration quality gates (CALIB_K_GATE).
void   intrinsics_set_quality_gates(bool on);
bool   intrinsics_quality_gates(void);
double intrinsics_rms_limit(void);

// Capture the current frame as one calibration view. On IC_CAPTURED,
// quality_out receives the acceptance measurements and a human-readable
// rejection reason. A rejected frame NEVER increments the view counter.
IntrCalibState intrinsics_capture_view(const cv::Mat& gray,
                                       IntrCalibViewQuality* quality_out);

// Remove the most recently accepted view. Useful when the operator notices
// movement/glare after capture. Returns false if there is nothing to remove.
bool intrinsics_undo_last_view(void);

// Copy out the ChArUco interior-corner pixel positions of the most recently
// accepted view (full-frame coords), and optionally their board ids. Used to
// draw the corners on the stored calibration-view image, and to ship the exact
// points that fed the fit to the server (CALIB_K_VIEW).
//
// Pass ids_out whenever the corners leave this camera: an id is what says WHICH
// board corner a pixel is, so without them cv::aruco::CharucoBoard::
// matchImagePoints() cannot rebuild the object-point correspondence and the
// view is useless for any offline re-fit. Drawing does not need them.
//
// Returns false if no view has been captured yet.
bool intrinsics_last_view_corners(std::vector<cv::Point2f>& out,
                                  std::vector<int>* ids_out = NULL);

// Run the calibration now with whatever views have been captured.
IntrCalibState intrinsics_compute(void);

// Progress/result info for status reporting.
int         intrinsics_views(void);        // views stored so far
int         intrinsics_pruned_views(void); // outlier views removed at compute
double      intrinsics_rms(void);          // valid after IC_DONE_OK
const char* intrinsics_fail_reason(void);  // valid after IC_DONE_FAIL

// Copy out the current K (fx, fy, cx, cy) and 5 distortion coefficients.
// Returns false if not available.
bool intrinsics_get(double* fx, double* fy, double* cx, double* cy,
                    double dist[5]);

// Load externally-computed intrinsics directly (no capture session). Applies to
// the detector immediately so solvePnP uses them at once. Returns false on
// clearly invalid input (fx/fy <= 0, negative principal point, non-finite
// distortion). Not persisted -- pair with intrinsics_save()/a profile if the
// value should survive a reboot (subject to PERSIST_TO_MNT and /mnt writability).
// Clears the active profile name: these numbers are nobody's profile.
bool intrinsics_load_values(double fx, double fy, double cx, double cy,
                            const double dist[5]);

// Tag whatever calibration session is currently open (the same id stamped on
// every CALIB_K_VIEW image/corner upload, see g_calib_session in
// aruco_detector_cv.cpp). Call at CALIB_K_START. A later intrinsics_compute()
// success freezes this into the value intrinsics_session_id() reports and
// intrinsics_save() persists, so the saved K/dist can always be matched back
// to the exact uploaded view set that produced it.
void intrinsics_set_session_id(long id);

// The session id that produced the CURRENTLY loaded K/dist (0 if never
// computed/loaded, or loaded from a file saved before this existed).
long intrinsics_session_id(void);

// Persist the currently loaded/calibrated K/dist to PERSIST_DIR right now.
// A successful intrinsics_compute() does NOT save on its own (K/dist is applied
// in RAM and usable immediately), so this is how a calibration is made to
// survive a restart -- and how a save is retried after a write failure without
// repeating the whole capture session.
// Returns false if there is nothing calibrated yet, or if the write itself
// fails (see intrinsics_fail_reason()).
bool intrinsics_save(void);

// Named K/dist snapshots under the same PERSIST_DIR: explicit operator
// snapshots that can be swapped without re-running a calibration.
//
// Saving or loading a profile also mirrors it into camera_intrinsics.txt, the
// file intrinsics_init() reads at startup -- so the active profile IS the
// boot-time default and a load survives a reboot with no extra save step. The
// profile's NAME is mirrored too, so the dashboard still shows it after a
// restart. If that mirror write fails the load/save still returns true (the
// values are applied in RAM and work for this session); pass a non-NULL
// `persisted` to find out, and see intrinsics_fail_reason() for why.
struct IntrinsicsProfileInfo {
    char name[32];
    long session_id;
};
bool intrinsics_save_profile(const char* name, bool* persisted = NULL);
bool intrinsics_load_profile(const char* name, bool* persisted = NULL);
int  intrinsics_list_profiles(IntrinsicsProfileInfo* out, int max_out);
const char* intrinsics_active_profile(void);

#endif // INTRINSICS_CALIBRATOR_H
