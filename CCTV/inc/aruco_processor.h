#ifndef ARUCO_PROCESSOR_H
#define ARUCO_PROCESSOR_H

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

/**
 * ArUco marker detector + pose estimator for Wisenet OpenSDK apps.
 *
 * Pipeline:
 *   NV12 raw frame  ->  gray (Y plane)  ->  detectMarkers  ->  solvePnP  ->
 *   per-axis pose (X = red, Y = green, Z = blue).
 *
 * Each Detection also carries the projected 2D pixel coordinates of the marker
 * outline and the three axis tips, so a live-stream overlay layer can redraw
 * them on the encoded video via the SDK OSD API (see OverlayRenderer).
 *
 * Camera intrinsics: if you have a calibration for the sensor, feed it with
 * setCameraIntrinsics(). Otherwise a rough pinhole guess is derived from the
 * frame size, which is good enough for visualization but not for metric poses.
 */
class ArucoProcessor
{
public:
    struct Detection
    {
        int       id;         // marker id
        cv::Vec3d rvec;       // rotation (Rodrigues), marker -> camera
        cv::Vec3d tvec;       // translation, meters
        cv::Vec3d euler;      // rotation about X,Y,Z in DEGREES (Rx,Ry,Rz)
        double    distance;   // ||tvec||, meters

        // Projected 2D pixel coordinates (image space) for overlay drawing:
        cv::Point2f origin2d; // marker center / axis origin
        cv::Point2f xTip2d;   // +X axis tip
        cv::Point2f yTip2d;   // +Y axis tip
        cv::Point2f zTip2d;   // +Z axis tip
        std::vector<cv::Point2f> corners2d; // marker outline (4 corners)
    };

    // markerLength: physical marker side length in meters (default 5 cm).
    // dictId: cv::aruco predefined dictionary id (default DICT_4X4_50).
    explicit ArucoProcessor(float markerLength = 0.05f, int dictId = -1);

    // Override the intrinsics with a real calibration.
    void setCameraIntrinsics(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);

    // Detect + estimate pose on a grayscale image. Fills `out`, no drawing.
    // This is the lightweight path used for live-stream overlay (the NV12 Y
    // plane is already grayscale, so no color conversion is needed).
    int detect(const cv::Mat& gray, std::vector<Detection>& out);

    // Detect + draw axis arrows onto `bgr` (modified in place). Useful for
    // saving an annotated debug image. Returns marker count.
    int process(cv::Mat& bgr, std::vector<Detection>& out);
    int process(cv::Mat& bgr);

    // Convert a planar NV12 buffer (Y plane + interleaved UV) to BGR.
    static bool nv12ToBgr(const uint8_t* nv12, int width, int height, cv::Mat& bgrOut);

    // Zero-copy grayscale view over the NV12 Y plane. The returned Mat aliases
    // `nv12`, so it is only valid while that buffer stays alive.
    static cv::Mat nv12GrayView(const uint8_t* nv12, int width, int height);

    // Draw X/Y/Z arrows for one pose directly onto a cv::Mat (debug image path).
    static void drawAxisArrows(cv::Mat& img,
                               const cv::Mat& cameraMatrix,
                               const cv::Mat& distCoeffs,
                               const cv::Vec3d& rvec,
                               const cv::Vec3d& tvec,
                               float axisLength);

    float markerLength() const { return m_markerLength; }

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
    // origin back before solvePnP, so intrinsics, homography and everything
    // downstream stay in one coordinate space and need no ROI awareness.
    //
    // A marker must lie ENTIRELY inside the rect to be found, so leave a margin
    // of at least one marker width around the work area. An empty/invalid rect
    // means full frame.
    void setRoi(const cv::Rect& roi) { m_roi = roi; }
    cv::Rect roi() const { return m_roi; }

private:
    void ensureIntrinsics(const cv::Size& frameSize);

    float   m_markerLength;
    int     m_dictId;
    cv::Mat m_cameraMatrix;   // 3x3, CV_64F
    cv::Mat m_distCoeffs;     // 1x5, CV_64F
    bool    m_haveIntrinsics;
    bool    m_userIntrinsics;
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
};

#endif // ARUCO_PROCESSOR_H
