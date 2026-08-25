#ifndef PAINTDIMENSIONS_H
#define PAINTDIMENSIONS_H

#include <algorithm>
#include <cmath>

namespace paintdimensions {

// 서버는 도색 구간 양끝을 실측 펜 폭의 절반씩 연장한다. Qt가 보관하고
// 전송하는 중심 경로는 그대로 두고, 치수 UI만 완성 도색 길이와 왕복 변환한다.
inline double finishedSegmentMm(double storedSegmentMm, double strokeWidthMm,
                                bool outerContour)
{
    if (!std::isfinite(storedSegmentMm) || storedSegmentMm < 0.0) return 0.0;
    if (outerContour) return storedSegmentMm;
    const double width = std::isfinite(strokeWidthMm)
                       ? std::max(0.0, strokeWidthMm) : 0.0;
    return storedSegmentMm + width;
}

inline bool storedSegmentMm(double finishedMm, double strokeWidthMm,
                            bool outerContour, double *storedOut)
{
    if (!storedOut || !std::isfinite(finishedMm) || finishedMm <= 0.0)
        return false;
    const double width = std::isfinite(strokeWidthMm)
                       ? std::max(0.0, strokeWidthMm) : 0.0;
    const double stored = outerContour ? finishedMm : finishedMm - width;
    if (!(stored > 0.05) || !std::isfinite(stored)) return false;
    *storedOut = stored;
    return true;
}

inline double finishedOuterRadiusMm(double centerRadiusMm, double strokeWidthMm,
                                    bool outerContour)
{
    if (!std::isfinite(centerRadiusMm) || centerRadiusMm < 0.0) return 0.0;
    if (outerContour) return centerRadiusMm;
    const double width = std::isfinite(strokeWidthMm)
                       ? std::max(0.0, strokeWidthMm) : 0.0;
    return centerRadiusMm + width * 0.5;
}

} // namespace paintdimensions

#endif
