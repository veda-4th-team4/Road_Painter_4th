#ifndef DETECT_TUNING_H
#define DETECT_TUNING_H

class ArucoProcessor;

/**
 * Persistence for the two detection settings that decide frame rate: the
 * search ROI and the adaptive-threshold pass count.
 *
 * Why these need a backing file. Measured 2026-07-20: with detection skipped
 * the SDK hands us 9.61 fps, with it on we see 3.5 -- so the frame budget goes
 * inside detectMarkers(), and ROI + scan passes are the two levers on it
 * (ArucoProcessor::setRoi / setScanPasses). Both were plain members with no
 * store behind them, so every reboot threw the tuning away and the camera came
 * back at the slow default with nothing on screen to say so. K/dist and H
 * already survive restarts (intrinsics_init / homography_init); this closes the
 * same gap for the settings that actually control fps.
 *
 * They belong on the camera rather than in app_config.h because the right ROI
 * depends on where the work area lands in THIS installation's frame. That is
 * per-camera field data discovered during commissioning, not a build-time
 * constant -- and baking it into a header would mean a full package+upload
 * cycle to retune one number.
 *
 * Saving is explicit (TUNE_SAVE), matching CALIB_K_SAVE / HG_SAVE /
 * CALIB_K_BOARD_SAVE: ROI_SET and ARUCO_SCAN stay RAM-only, so a bad
 * experiment is undone by a restart instead of being baked in.
 */

// Load the persisted tuning and apply it to `proc`. Call once at startup after
// the processor exists. Harmless when no file is present (defaults are kept).
// Returns true if a file was loaded and applied.
//
// Both the ROI and the scan-pass count are applied -- restarting at the 3-pass
// default puts this installation over its frame budget, so a saved setting has
// to survive the restart. See the load path for the measurements behind that.
bool detect_tuning_init(ArucoProcessor* proc);

// Write the processor's current ROI + scan settings to PERSIST_DIR.
bool detect_tuning_save(const ArucoProcessor* proc);

// Forget the persisted tuning (removes the file). The next boot starts from
// full frame and the default pass count. RAM state is left alone -- clearing
// the file is not the same as reverting the live detector.
bool detect_tuning_clear(void);

// True if the settings currently applied came from a file rather than defaults.
// Worth surfacing: a saved ROI from a previous camera position makes markers
// vanish with no error anywhere, and "the camera was moved and nobody cleared
// the ROI" is not a guess an operator makes unprompted.
bool detect_tuning_loaded(void);

// Human-readable reason for the last failed save/clear (empty when fine).
const char* detect_tuning_fail_reason(void);

#endif // DETECT_TUNING_H
