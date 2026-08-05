#include "aruco_processor.h"

#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>  // cv::resize (reduced-scale SEARCH)

// OpenCV changed the aruco API in 4.7: ArucoDetector / value-type Dictionary
// replaced the free functions and cv::Ptr wrappers. Support both.
#define ARUCO_NEW_API (CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7))

namespace {

int defaultDict()
{
    return cv::aruco::DICT_4X4_50;
}

} // namespace

ArucoProcessor::ArucoProcessor(int dictId)
    : m_dictId(dictId < 0 ? defaultDict() : dictId),
      m_lastDetectMs(-1.0),
      m_scanPasses(3),   // OpenCV default sweep (windows 3, 13, 23)
      m_scanWin(13),
      m_scanGen(0),
      m_minPerimRate(0.03),    // OpenCV DetectorParameters defaults -- an
      m_errCorrRate(0.60),     // untouched detector matches prior behavior
      m_adaptiveThreshC(7.0),
      m_polyAccuracyRate(0.03),
      m_roi(),           // empty = full frame
      m_searchScale(1),  // full resolution until the caller asks otherwise
      m_lastWasScaled(false)
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

int ArucoProcessor::detect(const cv::Mat& gray, std::vector<Detection>& out)
{
    out.clear();
    if (gray.empty())
        return 0;

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

    // Shrink the search area when asked (SEARCH only -- see setSearchScale).
    // INTER_AREA averages the pixels it drops instead of picking one of them,
    // which keeps the black/white marker border a clean edge; INTER_NEAREST
    // aliases it and costs detections at exactly the scales worth using.
    // m_scaled is a member, so this reuses one buffer instead of allocating a
    // megapixel every frame.
    const double scaleT0 = (double) cv::getTickCount();
    m_lastWasScaled = false;
    if (m_searchScale > 1 &&
        searchArea.cols >= m_searchScale * 8 && searchArea.rows >= m_searchScale * 8) {
        cv::resize(searchArea, m_scaled,
                   cv::Size(searchArea.cols / m_searchScale, searchArea.rows / m_searchScale),
                   0, 0, cv::INTER_AREA);
        searchArea = m_scaled;
        m_lastWasScaled = true;
    }
    const double scaleMs = ((double) cv::getTickCount() - scaleT0) * 1000.0
                           / cv::getTickFrequency();

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
    // The resize is counted in: this number drives the caller's detection
    // budget, and a shrink that is not charged for would let the governor
    // over-commit the thread by exactly the amount it costs.
    m_lastDetectMs = scaleMs + ((double) cv::getTickCount() - detectT0) * 1000.0
                     / cv::getTickFrequency();

    if (ids.empty())
        return 0;

    // Back to full-frame coordinates BEFORE returning: the caller ships these
    // corners to the server as full-frame pixels.
    //
    // Order matters. The ROI was cropped first and the shrink applied to the
    // crop, so undo them the other way round: scale up out of the shrunk
    // image, THEN translate by the ROI origin. Doing it the other way would
    // multiply the origin too and put the marker somewhere else entirely.
    if (m_lastWasScaled) {
        const float s = (float) m_searchScale;
        for (size_t i = 0; i < corners.size(); ++i)
            for (size_t k = 0; k < corners[i].size(); ++k)
                corners[i][k] *= s;
    }
    if (roiOffset.x != 0.f || roiOffset.y != 0.f) {
        for (size_t i = 0; i < corners.size(); ++i)
            for (size_t k = 0; k < corners[i].size(); ++k)
                corners[i][k] += roiOffset;
    }

    for (size_t i = 0; i < ids.size(); ++i) {
        Detection d;
        d.id        = ids[i];
        d.corners2d = corners[i];
        out.push_back(d);
    }

    } catch (const cv::Exception&) {
        out.clear();
        return 0;
    }

    return static_cast<int>(out.size());
}
