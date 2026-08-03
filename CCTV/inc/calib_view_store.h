#ifndef CALIB_VIEW_STORE_H
#define CALIB_VIEW_STORE_H

#include <stdint.h>
#include <vector>

// In-memory store of ALREADY-ENCODED JPEG images, one per accepted intrinsics
// calibration view (CALIB_K_CAPTURE). The encode happens at capture time (a
// few tens of ms, kept local), and the bulk upload to the vision server is
// deferred to an explicit CALIB_K_UPLOAD command — so the slow ~MB transfer no
// longer stalls the frame path at the moment of capture.
//
// Thread-safety: captures run on the frame-callback thread; the upload runs on
// a separate worker thread. Every function below takes an internal lock, and
// calib_view_store_get() returns a full copy, so the worker can read while the
// frame thread is idle between captures.

// One ChArUco interior corner exactly as the calibrator saw it: sub-pixel
// position in the raw NV12 frame, plus the board id saying WHICH corner it is.
//
// Both halves matter downstream. The id is what lets the server map the pixel
// back to a board coordinate (cv::aruco::CharucoBoard::matchImagePoints), so
// positions alone cannot be re-fitted. And these are the points measured on the
// raw frame -- re-detecting them from the uploaded JPEG would return slightly
// different sub-pixel values (compression artefacts) and thus analyse data the
// camera never actually fitted. Kept as plain floats so this header stays free
// of OpenCV, like the rest of the store.
struct CalibViewCorner {
    float x;
    float y;
    int   id;
};

struct CalibViewJpeg {
    int view;      // 1-based accepted-view index at capture time
    int target;    // target view count at capture time
    int corners;   // == points.size(); kept for the existing progress lines
    int width;     // source image dimensions (the JPEG also carries them)
    int height;
    std::vector<CalibViewCorner> points;  // the points that fed the fit

    // Two encodes of the SAME frame. The overlay one is for a human deciding
    // whether a view was any good; the plain one is the only one an offline
    // tool can re-detect from, because the overlay's rings sit on top of the
    // marker bits and defeat detection. Keeping both costs one extra encode at
    // capture (~tens of ms, off the realtime path) and ~200KB of RAM per view.
    std::vector<uint8_t> jpeg;        // marker/axis/corner overlay drawn
    std::vector<uint8_t> jpeg_plain;  // untouched frame, no drawing
};

void calib_view_store_reset(void);      // drop all stored views (CALIB_K_START)
void calib_view_store_pop_last(void);   // drop the most recent (CALIB_K_UNDO)
int  calib_view_store_count(void);

// Move a freshly-encoded pair of JPEGs and their corner list into the store
// (takes ownership of all three).
void calib_view_store_add(int view, int target, int corners,
                          int width, int height,
                          std::vector<CalibViewCorner>&& points,
                          std::vector<uint8_t>&& jpeg,
                          std::vector<uint8_t>&& jpeg_plain);

// Copy out item i (0-based) under lock. Returns false if i is out of range.
bool calib_view_store_get(int i, CalibViewJpeg& out);

// --- Upload-progress line queue --------------------------------------------
// pose_sender is single-threaded (no locking; a shared socket + ring buffer),
// so the upload worker thread must NOT call it directly. Instead the worker
// PUSHes progress lines here, and the frame thread POPs and forwards them to
// pose_sender. Both are internally locked.
void calib_view_progress_push(const char* line);
bool calib_view_progress_pop(char* out, int out_len);   // false if empty

#endif // CALIB_VIEW_STORE_H
