#ifndef HOMOGRAPHY_MAPPER_H
#define HOMOGRAPHY_MAPPER_H

#include <vector>

#include "aruco_processor.h"
#include "app_config.h"   // AnchorConfig (anchor id + world mm)

/**
 * Pixel -> physical ground-plane mapping via homography, computed ON the
 * camera from 8 calculation ArUco markers whose real-world positions are known
 * (measured on site, in millimeters).
 *
 * Calibration flow (triggered by a server command, see aruco_detector_cv.cpp):
 *   1. homography_start_calib()
 *   2. feed every frame's FRESH detections to homography_feed(); frames where
 *      all anchors are visible accumulate their (subpixel) center points
 *   3. after enough good frames the averaged centers are robustly matched to
 *      configured world coordinates -> cv::findHomography -> H (3x3).
 *      Separate validation markers are never included in this fit.
 *   4. H stays in RAM only until homography_save() is called explicitly
 *      (e.g. an HG_SAVE server command) -- persisting is a separate,
 *      operator-triggered step, not an automatic side effect of calibrating
 *
 * Runtime: homography_pixel_to_world() maps any pixel to world mm.
 */

enum HomographyCalibState {
    HG_IDLE,        // not calibrating (H may or may not be loaded)
    HG_COLLECTING,  // gathering anchor observations
    HG_DONE_OK,     // returned exactly once when calibration succeeds
    HG_DONE_FAIL    // returned exactly once when calibration gives up
};

// Load a previously saved H from /mnt (if any). Call once at startup.
void homography_init(void);

// True once a valid H is loaded/computed; pixel_to_world is then usable.
bool homography_active(void);

// Select the pixel coordinate system used by H fitting and mapping. Enabling
// undistortion requires a valid on-camera K/dist calibration.
bool homography_set_undistort(bool on);
bool homography_undistort_enabled(void);
bool homography_prepare_pixel(float px, float py, cv::Point2f* out);

// Enter collecting state (resets any previous collection progress).
void homography_start_calib(void);
bool homography_collecting(void);

// Feed the current frame's fresh detections. Returns the state AFTER this
// frame; HG_DONE_OK / HG_DONE_FAIL are reported exactly once, then the state
// falls back to HG_IDLE.
HomographyCalibState homography_feed(
        const std::vector<ArucoProcessor::Detection>& dets);

// Progress info for status reporting.
int         homography_progress(void);     // good frames collected so far
const char* homography_fail_reason(void);  // valid after HG_DONE_FAIL

// Map an image pixel to world coordinates (mm). Returns false if H not set.
bool homography_pixel_to_world(float px, float py, double* wx, double* wy);

// Copy the current 3x3 homography (row-major, 9 doubles). False if not set.
bool homography_get(double h[9]);

// Replace the active pixel->world matrix with a matrix calculated off-camera
// (for example by the RPi/PC 12C8 experiment).  The matrix is kept in RAM;
// call homography_save() separately if persistence is enabled and desired.
// Refuses a malformed/singular matrix and changes are not allowed while the
// legacy on-camera CALIB_START collector is running.
bool homography_set(const double h[9]);

// Persist the currently active H to /mnt (PERSIST_DIR) right now. Returns
// false if there is no H yet (never calibrated / not active), or if the
// write itself fails (see homography_fail_reason()).
bool homography_save(void);

// --- Runtime-editable anchor table ----------------------------------------
// The world positions in kAnchorTable (app_config.h) are only the DEFAULTS.
// They can be changed at runtime from the dashboard (ANCHOR_SET command) so a
// site can be re-measured without a rebuild. The dashboard may replace the
// complete list (4..HOMOGRAPHY_MAX_ANCHORS entries), including marker ids.
enum { HOMOGRAPHY_MIN_ANCHORS = 4, HOMOGRAPHY_MAX_ANCHORS = 16 };

// Update one anchor's world position (mm), matched by id. Returns false if the
// id is not in the table, or if a calibration is currently collecting (the
// world targets must not change mid-collection). Takes effect on the next
// CALIB_START.
bool homography_set_anchor(int id, double wx, double wy);

// Update one numbered slot (0..7), including its marker id and world position.
// Ids must remain unique. Returns false for invalid slots/ids, duplicates, or
// while calibration is collecting.
bool homography_set_anchor_slot(int slot, int id, double wx, double wy);

// Replace the complete calculation-anchor list. Ids must be unique and all
// points must be finite; at least four non-collinear surveyed points are needed
// for a usable homography. Changes are rejected while collecting.
bool homography_set_anchors(const AnchorConfig* entries, int count);

// Copy the current anchor table into `out` (up to `max` entries). Returns the
// number written.
int homography_get_anchors(AnchorConfig* out, int max);

// Copy the compiled independent validation-marker table. These markers are
// intentionally excluded from calibration and are used only to measure error.
enum { HOMOGRAPHY_MAX_VALIDATION_MARKERS = 16 };
int homography_get_validation_markers(AnchorConfig* out, int max);

// Replace the independent validation-marker list at runtime. Zero entries
// disables validation. Ids must be unique and must not overlap anchor ids.
bool homography_set_validation_markers(const AnchorConfig* entries, int count);

#endif // HOMOGRAPHY_MAPPER_H
