#ifndef ARUCO_PROCESSOR_H
#define ARUCO_PROCESSOR_H

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

/**
 * ArUco marker detector for Wisenet OpenSDK apps.
 *
 * Pipeline: NV12 raw frame -> gray (Y plane) -> detectMarkers -> corners2d.
 *
 * This port streams only marker id + pixel corners to the dashboard (see
 * SampleComponent::SendPosePackets); it has no per-marker 3D pose, camera
 * intrinsics, or drawing path (those were dropped along with the solvePnP
 * call they existed to feed -- nothing in this app ever consumed rvec/tvec).
 */
class ArucoProcessor
{
public:
    struct Detection
    {
        int       id;         // marker id
        std::vector<cv::Point2f> corners2d; // marker outline (4 corners)
    };

    // dictId: cv::aruco predefined dictionary id (default DICT_4X4_50).
    explicit ArucoProcessor(int dictId = -1);

    // Detect markers on a grayscale image. Fills `out`, no drawing. The NV12
    // Y plane is already grayscale, so no color conversion is needed.
    int detect(const cv::Mat& gray, std::vector<Detection>& out);

    // Wall-clock cost (ms) of the detectMarkers() call alone, from the last
    // detect(). Measurement aid: proc (t - t_frame, reported per packet) covers
    // the whole frame path, so proc - detectMs is "everything except the marker
    // search". Tells us whether the search really is the bottleneck before we
    // start optimizing it. -1 until the first detect().
    double lastDetectMs() const { return m_lastDetectMs; }

    // How many times detectMarkers binarizes the WHOLE frame before looking for
    // contours. OpenCV sweeps adaptive-threshold window sizes from Min to Max
    // in Step increments and unions the candidates, so the pass count is
    // (Max-Min)/Step + 1 and each pass costs a full-frame threshold + contour
    // scan. The default 3/23/10 means 3 passes (windows 3, 13, 23) -- robust
    // across marker sizes and lighting, and the single biggest cost in detect().
    //
    // Fewer passes trade that robustness for speed: a marker only found at
    // window 3 or 23 is silently missed when we scan 13 alone. Runtime-settable
    // (ARUCO_SCAN command) so the trade-off can be measured on the real scene
    // -- det/detection-rate/jitter before and after -- without a rebuild.
    //   passes 3 -> windows 3, 13, 23   (OpenCV default)
    //   passes 2 -> windows 7, 17
    //   passes 1 -> window `win` only   (default 13)
    void setScanPasses(int passes, int win = 13);
    int  scanPasses() const { return m_scanPasses; }
    int  scanWin()    const { return m_scanWin; }

    // Runtime knobs on the four DetectorParameters that move the detection RATE
    // (as opposed to scan passes / ROI, which trade rate for speed). Everything
    // else in detectMarkers stays at the OpenCV default. Exposed for the same
    // reason as ARUCO_SCAN: the right value depends on the real scene (marker
    // apparent size, tilt, lighting) and can only be found by sweeping it live,
    // and a rebuild is a full package+upload cycle.
    //
    //   "perim"  minMarkerPerimeterRate       (default 0.03) LOWER  -> small/far
    //   "ecc"    errorCorrectionRate          (default 0.60) HIGHER -> blur/tilt
    //   "thresh" adaptiveThreshConstant       (default 7.0)  tune   -> uneven light
    //   "poly"   polygonalApproxAccuracyRate  (default 0.03) HIGHER -> perspective
    //
    // These are RAM-only by design (never persisted): like a narrowed scan
    // sweep, a relaxed threshold that silently survives a reboot would raise the
    // false-positive rate with nothing on screen to link it to a value someone
    // changed days earlier. detect_tuning persists only the ROI. Use TUNE_QUERY
    // / a restart to get back to defaults.
    //
    // setDetectParam clamps the value to a sane range and returns false only
    // when `name` is not one of the four above. The value actually applied is
    // read back with the getters (report the clamped value, not the request).
    bool   setDetectParam(const std::string& name, double value);
    double minPerimRate()      const { return m_minPerimRate; }
    double errCorrRate()       const { return m_errCorrRate; }
    double adaptiveThreshC()   const { return m_adaptiveThreshC; }
    double polyAccuracyRate()  const { return m_polyAccuracyRate; }

    // Restrict the marker search to a sub-rectangle of the frame. The binarize
    // + contour scan inside detectMarkers costs time proportional to PIXELS, so
    // halving the area roughly halves it -- and unlike downscaling, the markers
    // keep their apparent size, so nothing gets too small to find. Valid here
    // only because the camera is fixed and the work area always lands in the
    // same part of the frame.
    //
    // Reported corners are always FULL-FRAME coordinates: detect() adds the ROI
    // origin back before returning, so the dashboard and everything downstream
    // stay in one coordinate space and need no ROI awareness.
    //
    // A marker must lie ENTIRELY inside the rect to be found, so leave a margin
    // of at least one marker width around the work area. An empty/invalid rect
    // means full frame.
    void setRoi(const cv::Rect& roi) { m_roi = roi; }
    cv::Rect roi() const { return m_roi; }

    // Detect on a 1/N shrunk copy of the search area, then multiply the corners
    // back up. 1 disables it.
    //
    // For the SEARCH case only. A lens with no marker in view cannot narrow its
    // ROI, so it rescans the whole 2592x1520 frame every frame -- 170..300 ms
    // measured, against 7..16 ms once the tracker has locked on. Cost falls
    // with PIXELS, so halving each side quarters it.
    //
    // The precision that is lost does not matter HERE, and only here: these
    // corners are used to place the next frame's ROI, which dyn_roi pads by
    // ~120 px anyway, so a few pixels of scaling error disappear into the
    // margin. The corners that reach the server come from the full-resolution
    // pass that follows -- see SampleComponent::ProcessRawVideo, which drops
    // the reduced-scale corners rather than publishing them.
    //
    // The limit is marker SIZE: at 1/4 a 158 px marker is 39 px, near the floor
    // for decoding a 4x4 dictionary, and a marker further away would be missed
    // outright. Runtime-settable for that reason -- the usable factor depends
    // on the scene and can only be found by sweeping it live.
    void setSearchScale(int n) { m_searchScale = (n < 1) ? 1 : (n > 8 ? 8 : n); }
    int  searchScale() const { return m_searchScale; }

    // True when the last detect() ran on a shrunk image, i.e. its corners are
    // approximate. Lets the caller decide what to do with them.
    bool lastWasScaled() const { return m_lastWasScaled; }

private:
    int     m_dictId;
    double  m_lastDetectMs;   // detectMarkers() only, milliseconds
    int     m_scanPasses;     // adaptive-threshold passes (1..3)
    int     m_scanWin;        // window size used when m_scanPasses == 1
    int     m_scanGen;        // bumped on change so detect() re-applies params
    // Detection-rate knobs (see setDetectParam). Initialized to the OpenCV
    // DetectorParameters defaults so an untouched detector behaves exactly as
    // before. A change bumps m_scanGen, which rebuilds the cached detector.
    double  m_minPerimRate;   // minMarkerPerimeterRate       (default 0.03)
    double  m_errCorrRate;    // errorCorrectionRate          (default 0.60)
    double  m_adaptiveThreshC;// adaptiveThreshConstant       (default 7.0)
    double  m_polyAccuracyRate;// polygonalApproxAccuracyRate (default 0.03)
    cv::Rect m_roi;           // search area; empty = full frame
    int     m_searchScale;    // 1 = detect at full resolution
    bool    m_lastWasScaled;  // did the last detect() shrink the image?
    cv::Mat m_scaled;         // reused destination, so no per-frame allocation
};

#endif // ARUCO_PROCESSOR_H
