#ifndef PAINTPROGRESS_H
#define PAINTPROGRESS_H

#include <QLineF>
#include <QList>
#include <QPointF>
#include <QtGlobal>

#include <cmath>
#include <limits>

namespace paintprogress {

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
