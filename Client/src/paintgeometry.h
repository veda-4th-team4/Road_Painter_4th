#ifndef PAINTGEOMETRY_H
#define PAINTGEOMETRY_H

#include "motionprogram.h"

#include <QLineF>
#include <QPointF>
#include <QString>
#include <QVector>

#include <algorithm>
#include <cmath>
#include <limits>

namespace paintgeometry {

enum class Kind { PassThrough, PolygonInset, ArcInset };

struct Result {
    QVector<QPointF> points;
    Kind kind = Kind::PassThrough;
    bool ok = true;
    QString error;
    QPointF arcCenter;
    double outerRadius = 0.0;
    double centerRadius = 0.0;
    double sweepDeg = 0.0;
};

// ── 로봇 중심 ↔ 펜(노즐) 표시 위치 ────────────────────────────────────────
// 🔴 이 거리는 **표시 전용**이다. 경로 생성·직렬화에 절대 들어가지 않는다.
//    실제 기구 보정(로봇 중심-노즐 오프셋)은 서버/로봇 책임이며 Qt 는 도면 그대로
//    보낸다. 값을 바꿔도 전송되는 점·paint·MOVE/ARC/TURN·반지름·스윕·경로 길이가
//    달라지지 않아야 한다 (motionprogram_tests 의 펜 거리 불변 테스트가 이를 고정).
inline QPointF penMarkerFromRobotCenter(double centerX, double centerY,
                                        double headingDeg, double penOffsetM)
{
    const double th = headingDeg * motionprogram::kDegToRad;
    return QPointF(centerX - penOffsetM * std::cos(th),
                   centerY - penOffsetM * std::sin(th));
}

inline QPointF robotCenterFromPen(double penX, double penY,
                                  double headingDeg, double penOffsetM)
{
    const double th = headingDeg * motionprogram::kDegToRad;
    return QPointF(penX + penOffsetM * std::cos(th),
                   penY + penOffsetM * std::sin(th));
}

inline double cross(const QPointF &a, const QPointF &b)
{
    return a.x() * b.y() - a.y() * b.x();
}

inline double signedArea2(const QVector<QPointF> &points)
{
    double area = 0.0;
    for (int i = 0; i < points.size(); ++i)
        area += cross(points[i], points[(i + 1) % points.size()]);
    return area;
}

inline bool lineIntersection(const QPointF &p, const QPointF &d,
                             const QPointF &q, const QPointF &e,
                             QPointF &out)
{
    const double den = cross(d, e);
    if (std::abs(den) < 1e-10) return false;
    const double t = cross(q - p, e) / den;
    out = p + d * t;
    return std::isfinite(out.x()) && std::isfinite(out.y());
}

inline Result fail(const QString &message)
{
    Result result;
    result.ok = false;
    result.error = message;
    return result;
}

// Converts a finished paint OUTER contour to the path followed by the center of
// a rectangular marker tip. Existing centerline paths bypass this function by
// passing outerContour=false.
// unitsPerMeter: points/strokeWidth 의 단위(px/m). 미터 좌표면 1.0.
// 원호 판정이 화면(px)과 전송(m) 양쪽에서 같은 물리 임계값을 쓰게 하려고 명시한다.
inline Result centerlineFor(const QVector<QPointF> &points, bool closed,
                            bool outerContour, double strokeWidth,
                            double unitsPerMeter = 1.0)
{
    Result result;
    result.points = points;
    if (!outerContour) return result;
    if (!(strokeWidth > 0.0) || !std::isfinite(strokeWidth))
        return fail(QStringLiteral("펜촉 폭이 올바르지 않습니다."));

    QList<QPointF> run;
    run.reserve(points.size() + 1);
    for (const QPointF &point : points) run.append(point);
    if (closed && !points.isEmpty()) run.append(points.first());
    motionprogram::detail::Circle fit;
    double sweepDeg = 0.0;
    bool left = true;
    if (points.size() >= 5
        && motionprogram::detail::arcFits(run, 0, run.size() - 1,
                                          fit, sweepDeg, left, unitsPerMeter)) {
        const double centerRadius = fit.r - strokeWidth * 0.5;
        if (centerRadius <= 1e-6)
            return fail(QStringLiteral("완성 반지름은 펜촉 폭의 절반보다 커야 합니다."));
        result.points.clear();
        result.points.reserve(points.size());
        for (const QPointF &point : points) {
            const QPointF radial = point - fit.c;
            const double length = std::hypot(radial.x(), radial.y());
            if (length <= 1e-9)
                return fail(QStringLiteral("원호 외곽선의 중심과 점이 겹칩니다."));
            result.points.append(fit.c + radial * (centerRadius / length));
        }
        result.kind = Kind::ArcInset;
        result.arcCenter = fit.c;
        result.outerRadius = fit.r;
        result.centerRadius = centerRadius;
        result.sweepDeg = sweepDeg;
        return result;
    }

    if (!closed || points.size() < 3)
        return fail(QStringLiteral("외곽선 도형은 원호이거나 세 점 이상의 닫힌 경로여야 합니다."));

    const double area2 = signedArea2(points);
    if (std::abs(area2) < 1e-8)
        return fail(QStringLiteral("외곽선의 면적이 없어 중심 경로를 만들 수 없습니다."));

    // The deterministic polygon inset intentionally accepts convex contours.
    // Concave offsets require choosing how self-intersections are trimmed, which
    // would silently change the user's requested outline.
    int turnSign = 0;
    for (int i = 0; i < points.size(); ++i) {
        const QPointF a = points[(i + 1) % points.size()] - points[i];
        const QPointF b = points[(i + 2) % points.size()] - points[(i + 1) % points.size()];
        const double z = cross(a, b);
        if (std::abs(z) < 1e-9) continue;
        const int sign = z > 0.0 ? 1 : -1;
        if (turnSign != 0 && sign != turnSign)
            return fail(QStringLiteral("오목한 외곽선은 자동 중심 경로를 만들 수 없습니다."));
        turnSign = sign;
    }
    if (turnSign == 0)
        return fail(QStringLiteral("외곽선의 꼭짓점이 한 직선 위에 있습니다."));

    const double half = strokeWidth * 0.5;
    QVector<QPointF> shiftedPoint(points.size());
    QVector<QPointF> direction(points.size());
    for (int i = 0; i < points.size(); ++i) {
        const QPointF d = points[(i + 1) % points.size()] - points[i];
        const double length = std::hypot(d.x(), d.y());
        if (length <= 1e-9)
            return fail(QStringLiteral("외곽선에 서로 겹친 꼭짓점이 있습니다."));
        direction[i] = d / length;
        QPointF inward(-direction[i].y(), direction[i].x());
        if (area2 < 0.0) inward = -inward;
        shiftedPoint[i] = points[i] + inward * half;
    }

    result.points.clear();
    result.points.reserve(points.size());
    for (int i = 0; i < points.size(); ++i) {
        const int previous = (i + points.size() - 1) % points.size();
        QPointF vertex;
        if (!lineIntersection(shiftedPoint[previous], direction[previous],
                              shiftedPoint[i], direction[i], vertex))
            return fail(QStringLiteral("평행한 외곽 변 사이의 중심 경로를 계산할 수 없습니다."));
        // A very long miter means the inset has crossed or nearly collapsed.
        if (QLineF(vertex, points[i]).length() > std::max(half * 20.0, 1.0))
            return fail(QStringLiteral("꼭짓점 각도가 너무 좁아 펜촉을 안쪽에 배치할 수 없습니다."));
        result.points.append(vertex);
    }
    if (std::abs(signedArea2(result.points)) < 1e-8
        || signedArea2(result.points) * area2 <= 0.0)
        return fail(QStringLiteral("선택한 펜촉 폭이 완성 외곽선 안에 들어가지 않습니다."));
    for (int edge = 0; edge < points.size(); ++edge) {
        const QPointF outerEdge = points[(edge + 1) % points.size()] - points[edge];
        const QPointF centerEdge = result.points[(edge + 1) % points.size()] - result.points[edge];
        if (QPointF::dotProduct(outerEdge, centerEdge) <= 1e-9)
            return fail(QStringLiteral("선택한 펜촉 폭이 완성 외곽선 안에 들어가지 않습니다."));
    }
    for (const QPointF &vertex : result.points) {
        for (int edge = 0; edge < points.size(); ++edge) {
            const QPointF d = points[(edge + 1) % points.size()] - points[edge];
            const double side = cross(d, vertex - points[edge]);
            if ((area2 > 0.0 && side < -1e-7)
                || (area2 < 0.0 && side > 1e-7))
                return fail(QStringLiteral("선택한 펜촉 폭이 완성 외곽선 안에 들어가지 않습니다."));
        }
    }

    result.kind = Kind::PolygonInset;
    return result;
}

inline double minimumOuterArcRadius(double strokeWidth, double minimumCenterRadius)
{
    return minimumCenterRadius + strokeWidth * 0.5;
}

inline double constrainOuterArcScale(double outerRadius, double strokeWidth,
                                     double minimumCenterRadius, double requestedFactor)
{
    if (requestedFactor >= 1.0 || outerRadius <= 0.0) return requestedFactor;
    const double minimumOuter = minimumOuterArcRadius(strokeWidth, minimumCenterRadius);
    if (outerRadius <= minimumOuter + 1e-9) return 1.0;
    return std::max(requestedFactor, minimumOuter / outerRadius);
}

} // namespace paintgeometry

#endif
