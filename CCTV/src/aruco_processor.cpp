#include "aruco_processor.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>

#include <cmath>
#include <vector>

// OpenCV changed the aruco API in 4.7: ArucoDetector / value-type Dictionary
// replaced the free functions and cv::Ptr wrappers. Support both.
#define ARUCO_NEW_API (CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7))

namespace {

// BGR colors for the three axes (OpenCV drawFrameAxes convention).
const cv::Scalar kColorX(0,   0,   255); // X -> red
const cv::Scalar kColorY(0,   255, 0);   // Y -> green
const cv::Scalar kColorZ(255, 0,   0);   // Z -> blue

int defaultDict()
{
    return cv::aruco::DICT_4X4_50;
}

} // namespace

ArucoProcessor::ArucoProcessor(float markerLength, int dictId)
    : m_markerLength(markerLength),
      m_dictId(dictId < 0 ? defaultDict() : dictId),
      m_haveIntrinsics(false),
      m_userIntrinsics(false),
      m_lastDetectMs(-1.0),
      m_scanPasses(3),   // OpenCV default sweep (windows 3, 13, 23)
      m_scanWin(13),
      m_scanGen(0),
      m_minPerimRate(0.03),    // OpenCV DetectorParameters defaults -- an
      m_errCorrRate(0.60),     // untouched detector matches prior behavior
      m_adaptiveThreshC(7.0),
      m_polyAccuracyRate(0.03),
      m_roi()            // empty = full frame
{
}

// Set one of the four detection-rate knobs by name (see the header). Clamps to
// a sane range and returns false only for an unknown name. Bumps m_scanGen so
// detect() rebuilds the cached detector with the new value on the next frame.
bool ArucoProcessor::setDetectParam(const std::string& name, double value)
{
    if (name == "perim") {
        // minMarkerPerimeterRate: perimeter as a fraction of max(w,h). Must be
        // > 0 and below maxMarkerPerimeterRate (4.0). Below ~0.005 the candidate
        // count (and cost) explodes on noise, so floor it there.
        if (value < 0.005) value = 0.005;
        if (value > 1.0)   value = 1.0;
        m_minPerimRate = value;
    } else if (name == "ecc") {
        // errorCorrectionRate: fraction of the dictionary's correctable bits to
        // actually use. Meaningful in [0,1]; 1.0 uses the full Hamming margin.
        if (value < 0.0) value = 0.0;
        if (value > 1.0) value = 1.0;
        m_errCorrRate = value;
    } else if (name == "thresh") {
        // adaptiveThreshConstant: subtracted from the local mean. OpenCV allows
        // negatives; keep a wide but bounded range as a corruption guard.
        if (value < -50.0) value = -50.0;
        if (value >  50.0) value =  50.0;
        m_adaptiveThreshC = value;
    } else if (name == "poly") {
        // polygonalApproxAccuracyRate: contour->quad tolerance. Too large lets
        // non-square blobs through; keep it in a practical band.
        if (value < 0.005) value = 0.005;
        if (value > 0.2)   value = 0.2;
        m_polyAccuracyRate = value;
    } else {
        return false;
    }
    ++m_scanGen;   // force detect() to rebuild the detector with the new value
    return true;
}

void ArucoProcessor::setScanPasses(int passes, int win)
{
    if (passes < 1) passes = 1;
    if (passes > 3) passes = 3;
    // adaptiveThreshold needs an odd window >= 3; OpenCV bumps even values up
    // internally, but keep it explicit so scanWin() reports what is really used.
    if (win < 3) win = 3;
    if (win % 2 == 0) ++win;

    m_scanPasses = passes;
    m_scanWin = win;
    ++m_scanGen;   // forces detect() to re-apply on the next frame
}

void ArucoProcessor::setCameraIntrinsics(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs)
{
    cameraMatrix.convertTo(m_cameraMatrix, CV_64F);
    if (distCoeffs.empty()) {
        m_distCoeffs = cv::Mat::zeros(1, 5, CV_64F);
    } else {
        distCoeffs.convertTo(m_distCoeffs, CV_64F);
    }
    m_haveIntrinsics = true;
    m_userIntrinsics = true;
}

// Rough pinhole model when no calibration is available: assume the principal
// point is the image center and the focal length ~ image width (FOV ~ 55 deg).
void ArucoProcessor::ensureIntrinsics(const cv::Size& frameSize)
{
    if (m_haveIntrinsics)
        return;

    double f  = static_cast<double>(frameSize.width);
    double cx = frameSize.width  * 0.5;
    double cy = frameSize.height * 0.5;

    m_cameraMatrix = (cv::Mat_<double>(3, 3) <<
                      f, 0, cx,
                      0, f, cy,
                      0, 0, 1);
    m_distCoeffs = cv::Mat::zeros(1, 5, CV_64F);
    m_haveIntrinsics = true;
}

bool ArucoProcessor::nv12ToBgr(const uint8_t* nv12, int width, int height, cv::Mat& bgrOut)
{
    if (!nv12 || width <= 0 || height <= 0)
        return false;

    cv::Mat yuv(height * 3 / 2, width, CV_8UC1, const_cast<uint8_t*>(nv12));
    cv::cvtColor(yuv, bgrOut, cv::COLOR_YUV2BGR_NV12);
    return true;
}

cv::Mat ArucoProcessor::nv12GrayView(const uint8_t* nv12, int width, int height)
{
    if (!nv12 || width <= 0 || height <= 0)
        return cv::Mat();

    // The NV12 Y plane is a full-resolution grayscale image; alias it directly.
    return cv::Mat(height, width, CV_8UC1, const_cast<uint8_t*>(nv12));
}

void ArucoProcessor::drawAxisArrows(cv::Mat& img,
                                    const cv::Mat& cameraMatrix,
                                    const cv::Mat& distCoeffs,
                                    const cv::Vec3d& rvec,
                                    const cv::Vec3d& tvec,
                                    float axisLength)
{
    std::vector<cv::Point3f> axisPts = {
        cv::Point3f(0.f,        0.f,        0.f),
        cv::Point3f(axisLength, 0.f,        0.f),
        cv::Point3f(0.f,        axisLength, 0.f),
        cv::Point3f(0.f,        0.f,        axisLength)
    };

    std::vector<cv::Point2f> img2d;
    cv::projectPoints(axisPts, rvec, tvec, cameraMatrix, distCoeffs, img2d);

    const int    thickness = 2;
    const double tipLen    = 0.20;
    cv::arrowedLine(img, img2d[0], img2d[1], kColorX, thickness, cv::LINE_AA, 0, tipLen);
    cv::arrowedLine(img, img2d[0], img2d[2], kColorY, thickness, cv::LINE_AA, 0, tipLen);
    cv::arrowedLine(img, img2d[0], img2d[3], kColorZ, thickness, cv::LINE_AA, 0, tipLen);
}

int ArucoProcessor::detect(const cv::Mat& gray, std::vector<Detection>& out)
{
    out.clear();
    if (gray.empty())
        return 0;

    ensureIntrinsics(gray.size());

    std::vector<int>                       ids;
    std::vector<std::vector<cv::Point2f> > corners;

    // Runs inside the SDK's per-frame callback, which has no exception handler
    // above it — an uncaught cv::Exception here takes the whole app down (same
    // failure class as the LDC_CHECK crash, see ldc_checker.cpp). A frame that
    // makes OpenCV throw is reported as "nothing detected" instead.
    try {

    // Search area: the ROI if one is set and fits, else the whole frame. This
    // is a VIEW into `gray` (no copy). Corners come back relative to it, so
    // roiOffset is added below to restore full-frame coordinates.
    cv::Mat searchArea = gray;
    cv::Point2f roiOffset(0.f, 0.f);
    if (m_roi.width > 0 && m_roi.height > 0 &&
        (m_roi & cv::Rect(0, 0, gray.cols, gray.rows)) == m_roi) {
        searchArea = gray(m_roi);
        roiOffset = cv::Point2f((float) m_roi.x, (float) m_roi.y);
    }

    // Adaptive-threshold sweep for the current scan setting. OpenCV runs one
    // binarize+contour pass over the search area per window size from Min to
    // Max, so the pass count is what this actually controls (setScanPasses).
    int winMin, winMax;
    switch (m_scanPasses) {
        case 1:  winMin = m_scanWin; winMax = m_scanWin; break;  // 1 pass
        case 2:  winMin = 7;         winMax = 17;        break;  // 7, 17
        default: winMin = 3;         winMax = 23;        break;  // 3, 13, 23
    }

    // ---- Detection (API differs pre/post OpenCV 4.7) ----
    // Dictionary/parameters/detector are config, not per-frame work: build once
    // and rebuild only when the dictionary or the scan setting changes.
#if ARUCO_NEW_API
    static int cachedDictId = -1;
    static int cachedScanGen = -1;
    static cv::aruco::ArucoDetector detector;
    if (cachedDictId != m_dictId || cachedScanGen != m_scanGen) {
        cv::aruco::DetectorParameters dp;
        dp.adaptiveThreshWinSizeMin  = winMin;
        dp.adaptiveThreshWinSizeMax  = winMax;
        dp.adaptiveThreshWinSizeStep = 10;
        dp.adaptiveThreshConstant      = m_adaptiveThreshC;
        dp.minMarkerPerimeterRate      = m_minPerimRate;
        dp.polygonalApproxAccuracyRate = m_polyAccuracyRate;
        dp.errorCorrectionRate         = m_errCorrRate;
        detector = cv::aruco::ArucoDetector(
            cv::aruco::getPredefinedDictionary(m_dictId), dp);
        cachedDictId = m_dictId;
        cachedScanGen = m_scanGen;
    }
    const double detectT0 = (double) cv::getTickCount();
    detector.detectMarkers(searchArea, corners, ids);
#else
    static int cachedDictId = -1;
    static int cachedScanGen = -1;
    static cv::Ptr<cv::aruco::Dictionary> dict;
    static cv::Ptr<cv::aruco::DetectorParameters> params;
    if (cachedDictId != m_dictId) {
        dict = cv::aruco::getPredefinedDictionary(m_dictId);
        params = cv::aruco::DetectorParameters::create();
        cachedDictId = m_dictId;
        cachedScanGen = -1;         // force the scan settings below to apply
    }
    if (cachedScanGen != m_scanGen) {
        params->adaptiveThreshWinSizeMin  = winMin;
        params->adaptiveThreshWinSizeMax  = winMax;
        params->adaptiveThreshWinSizeStep = 10;
        params->adaptiveThreshConstant      = m_adaptiveThreshC;
        params->minMarkerPerimeterRate      = m_minPerimRate;
        params->polygonalApproxAccuracyRate = m_polyAccuracyRate;
        params->errorCorrectionRate         = m_errCorrRate;
        cachedScanGen = m_scanGen;
    }
    const double detectT0 = (double) cv::getTickCount();
    cv::aruco::detectMarkers(searchArea, dict, corners, ids, params);
#endif
    // Detector construction above is one-time (cached), so it stays outside the
    // timed span: this measures the per-frame marker search only.
    m_lastDetectMs = ((double) cv::getTickCount() - detectT0) * 1000.0
                     / cv::getTickFrequency();

    if (ids.empty())
        return 0;

    // Back to full-frame coordinates BEFORE anything consumes them: solvePnP
    // below uses intrinsics whose principal point is in full-frame space, and
    // the caller ships these corners to the server as full-frame pixels.
    if (roiOffset.x != 0.f || roiOffset.y != 0.f) {
        for (size_t i = 0; i < corners.size(); ++i)
            for (size_t k = 0; k < corners[i].size(); ++k)
                corners[i][k] += roiOffset;
    }

    // Marker-local 3D corners, centered at the origin (order matches detector
    // output: TL, TR, BR, BL) — required layout for SOLVEPNP_IPPE_SQUARE.
    const float h = m_markerLength * 0.5f;
    std::vector<cv::Point3f> objPts = {
        cv::Point3f(-h,  h, 0.f),
        cv::Point3f( h,  h, 0.f),
        cv::Point3f( h, -h, 0.f),
        cv::Point3f(-h, -h, 0.f)
    };

    const float axisLen = m_markerLength * 0.5f;
    std::vector<cv::Point3f> axisPts = {
        cv::Point3f(0.f,     0.f,     0.f),
        cv::Point3f(axisLen, 0.f,     0.f),
        cv::Point3f(0.f,     axisLen, 0.f),
        cv::Point3f(0.f,     0.f,     axisLen)
    };

    for (size_t i = 0; i < ids.size(); ++i) {
        cv::Vec3d rvec, tvec;
        bool ok = cv::solvePnP(objPts, corners[i], m_cameraMatrix, m_distCoeffs,
                               rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
        if (!ok)
            continue;

        std::vector<cv::Point2f> axis2d;
        cv::projectPoints(axisPts, rvec, tvec, m_cameraMatrix, m_distCoeffs, axis2d);

        // Euler angles (deg) about X,Y,Z from the rotation matrix.
        cv::Matx33d R;
        cv::Rodrigues(rvec, R);
        double sy = std::sqrt(R(0,0)*R(0,0) + R(1,0)*R(1,0));
        double rx, ry, rz;
        if (sy > 1e-6) {
            rx = std::atan2(R(2,1), R(2,2));
            ry = std::atan2(-R(2,0), sy);
            rz = std::atan2(R(1,0), R(0,0));
        } else {
            rx = std::atan2(-R(1,2), R(1,1));
            ry = std::atan2(-R(2,0), sy);
            rz = 0.0;
        }
        const double RAD2DEG = 180.0 / CV_PI;

        Detection d;
        d.id       = ids[i];
        d.rvec     = rvec;
        d.tvec     = tvec;
        d.euler    = cv::Vec3d(rx * RAD2DEG, ry * RAD2DEG, rz * RAD2DEG);
        d.distance = std::sqrt(tvec[0]*tvec[0] + tvec[1]*tvec[1] + tvec[2]*tvec[2]);
        d.origin2d = axis2d[0];
        d.xTip2d   = axis2d[1];
        d.yTip2d   = axis2d[2];
        d.zTip2d   = axis2d[3];
        d.corners2d = corners[i];
        out.push_back(d);
    }

    } catch (const cv::Exception&) {
        out.clear();
        return 0;
    }

    return static_cast<int>(out.size());
}

int ArucoProcessor::process(cv::Mat& bgr)
{
    std::vector<Detection> ignored;
    return process(bgr, ignored);
}

int ArucoProcessor::process(cv::Mat& bgr, std::vector<Detection>& out)
{
    if (bgr.empty()) {
        out.clear();
        return 0;
    }

    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

    int count = detect(gray, out);
    if (count == 0)
        return 0;

    // Redraw the results onto the color image for a debug snapshot.
    std::vector<std::vector<cv::Point2f> > corners;
    std::vector<int> ids;
    corners.reserve(out.size());
    ids.reserve(out.size());
    for (size_t i = 0; i < out.size(); ++i) {
        corners.push_back(out[i].corners2d);
        ids.push_back(out[i].id);
    }
    cv::aruco::drawDetectedMarkers(bgr, corners, ids);

    const float axisLen = m_markerLength * 0.5f;
    for (size_t i = 0; i < out.size(); ++i) {
        drawAxisArrows(bgr, m_cameraMatrix, m_distCoeffs,
                       out[i].rvec, out[i].tvec, axisLen);
    }
    return count;
}
