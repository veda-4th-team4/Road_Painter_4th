#ifndef PAINTPROGRESS_H
#define PAINTPROGRESS_H

#include <QLineF>
#include <QList>
#include <QPointF>
#include <QtGlobal>

#include <cmath>
#include <limits>

namespace paintprogress {

struct PathProjection {
    bool valid = false;
    double progress01 = 0.0;
    double distanceM = std::numeric_limits<double>::max();
};

inline double pathLength(const QList<QPointF> &points, bool closed)
{
    if (points.size() < 2) return 0.0;
    double length = 0.0;
    for (int i = 1; i < points.size(); ++i)
        length += QLineF(points[i - 1], points[i]).length();
    if (closed && points.size() > 2)
        length += QLineF(points.last(), points.first()).length();
    return length;
}

// 한 도형 안에서만 펜 끝을 투영한다. 실행 순서상 아직 차례가 아닌 다른 글자에
// 가까이 지나갔다는 이유로 진행률이 건너뛰지 않게 Backend가 이 함수를 사용한다.
inline PathProjection projectOnPath(const QList<QPointF> &path, bool closed,
                                    const QPointF &pen)
{
    PathProjection out;
    const double total = pathLength(path, closed);
    if (total < 1e-9) return out;

    QList<QPointF> points = path;
    if (closed && points.size() > 2)
        points.append(points.first());

    double completed = 0.0;
    double bestDistance2 = std::numeric_limits<double>::max();
    double bestAlong = 0.0;
    for (int i = 1; i < points.size(); ++i) {
        const QPointF a = points[i - 1];
        const QPointF ab = points[i] - a;
        const double length2 = QPointF::dotProduct(ab, ab);
        if (length2 < 1e-12) continue;
        const double segmentLength = std::sqrt(length2);
        const double t = qBound(0.0, QPointF::dotProduct(pen - a, ab) / length2, 1.0);
        const QPointF delta = pen - (a + ab * t);
        const double distance2 = QPointF::dotProduct(delta, delta);
        if (distance2 < bestDistance2 - 1e-12) {
            bestDistance2 = distance2;
            bestAlong = completed + segmentLength * t;
        }
        completed += segmentLength;
    }
    if (bestDistance2 == std::numeric_limits<double>::max()) return out;
    out.valid = true;
    out.progress01 = qBound(0.0, bestAlong / total, 1.0);
    out.distanceM = std::sqrt(bestDistance2);
    return out;
}

// 도형별 도색률을 실제 도색 길이로 가중해 상단의 전체 퍼센트/ETA를 계산한다.
inline double weightedProgress(const QList<QList<QPointF>> &paths,
                               const QList<bool> &closed,
                               const QList<double> &pathProgress)
{
    double total = 0.0;
    double painted = 0.0;
    for (int i = 0; i < paths.size(); ++i) {
        const double length = pathLength(paths[i], closed.value(i, false));
        total += length;
        painted += length * qBound(0.0, pathProgress.value(i, 0.0), 1.0);
    }
    return total < 1e-9 ? 0.0 : qBound(0.0, painted / total, 1.0);
}

// 테스트 재생처럼 전체 진행률만 있는 경우 도형별 prefix 진행률로 변환한다.
inline QList<double> prefixPathProgress(const QList<QList<QPointF>> &paths,
                                        const QList<bool> &closed,
                                        double overallProgress)
{
    QList<double> out(paths.size(), 0.0);
    double total = 0.0;
    for (int i = 0; i < paths.size(); ++i)
        total += pathLength(paths[i], closed.value(i, false));
    double remain = total * qBound(0.0, overallProgress, 1.0);
    for (int i = 0; i < paths.size(); ++i) {
        const double length = pathLength(paths[i], closed.value(i, false));
        if (length < 1e-9) continue;
        out[i] = qBound(0.0, remain / length, 1.0);
        remain -= length * out[i];
    }
    return out;
}

// 펜 끝을 각 도형의 실제 도색 선에 투영해 전체 도색 길이 기준 진행률을 구한다.
// 도형 사이의 펜을 든 이동 거리는 포함하지 않는다.
inline double progressAlongPaths(const QList<QList<QPointF>> &paths,
                                 const QList<bool> &closed,
                                 const QPointF &pen)
{
    double total = 0.0;
    for (int i = 0; i < paths.size(); ++i)
        total += pathLength(paths[i], closed.value(i, false));
    if (total < 1e-9) return 0.0;

    double bestDistance2 = std::numeric_limits<double>::max();
    double bestAlong = 0.0;
    double completedBeforePath = 0.0;

    for (int pathIndex = 0; pathIndex < paths.size(); ++pathIndex) {
        QList<QPointF> points = paths[pathIndex];
        const bool isClosed = closed.value(pathIndex, false);
        if (isClosed && points.size() > 2)
            points.append(points.first());

        double completedInPath = 0.0;
        for (int i = 1; i < points.size(); ++i) {
            const QPointF a = points[i - 1];
            const QPointF ab = points[i] - a;
            const double length2 = QPointF::dotProduct(ab, ab);
            if (length2 < 1e-12) continue;

            const double segmentLength = std::sqrt(length2);
            const double t = qBound(0.0, QPointF::dotProduct(pen - a, ab) / length2, 1.0);
            const QPointF delta = pen - (a + ab * t);
            const double distance2 = QPointF::dotProduct(delta, delta);

            // 교차점처럼 거리가 같은 경우에는 먼저 실행되는 구간을 유지해
            // 진행률이 아직 지나지 않은 뒤쪽 경로로 순간 이동하지 않게 한다.
            if (distance2 < bestDistance2 - 1e-12) {
                bestDistance2 = distance2;
                bestAlong = completedBeforePath + completedInPath + segmentLength * t;
            }
            completedInPath += segmentLength;
        }
        completedBeforePath += pathLength(paths[pathIndex], isClosed);
    }

    return qBound(0.0, bestAlong / total, 1.0);
}

} // namespace paintprogress

#endif // PAINTPROGRESS_H
