#pragma once
// ── 도형 여러 개 → 로봇이 실제로 달릴 한 줄 경로 ─────────────────────────────
//
// 왜 필요한가: 프로토콜의 BLUEPRINT 는 폴리라인 "하나"다. 그래서 화면에 그린 도형들을
// 한 줄로 이어 보내야 하는데, 그냥 작도 순서대로 이으면
//   ① 로봇이 도형 사이를 가로질러 다니느라 현장을 활보하고 (횡단보도 5줄을 1→2→3 이
//      아니라 그린 순서대로 왔다갔다),
//   ② 그 이동 구간까지 전부 도색 구간으로 나가 바닥에 없는 선이 그려진다.
// 여기서 도형 순서·진행 방향을 다시 잡고, 도형 사이 구간에 pen-up(paint=false) 표시를
// 붙인다.
//
// 좌표계: 바닥 평면 미터. (BLUEPRINT 와 동일 — 서버는 재변환하지 않는다)
#include <QLineF>
#include <QList>
#include <QPointF>
#include <QVector>
#include <cmath>

namespace routeplan {

struct Route {
    QList<QPointF> pts;      // 이어붙인 폴리라인 (BLUEPRINT.points)
    QList<bool> paint;       // paint[i] = pts[i-1]→pts[i] 구간을 칠하는가.
                             // paint[0] 은 시작점이라 구간이 없다 → 항상 false
    QList<int> shapeOf;      // pts[i] 가 속한 도형 (재배열 후 순번)
    QList<int> order;        // order[k] = 원래 도형 index (로그·미리보기용)
    QList<bool> flipped;     // 그 도형을 뒤집어 달리는가
    double paintM = 0.0;     // 도색 구간 길이 합
    double travelM = 0.0;    // 도형 사이 빈 이동 길이 합
    int shapeCount = 0;

    // 닫힌 도형 끝에 붙인 "시작점 복귀" 중복 점을 뺀 실제 꼭짓점 수.
    // 원본 점 수와 비교해 단순화로 몇 점이 줄었는지 로그에 쓴다.
    int paintPointCount() const
    {
        int n = pts.size();
        int shapeStart = 0;
        for (int i = 0; i < pts.size(); ++i) {
            const bool lastOfShape = (i + 1 == pts.size()) || shapeOf[i + 1] != shapeOf[i];
            if (!lastOfShape) continue;
            if (i > shapeStart && QLineF(pts[i], pts[shapeStart]).length() < 1e-6)
                --n;
            shapeStart = i + 1;
        }
        return n;
    }
};

// ── 점 정리 ──────────────────────────────────────────────────────────────
// 서버 buildSegments 는 1cm 미만 이동을 버리고, 로봇은 MOVE 시작마다 정지→정렬
// (READY/ALIGN/GO) 핸드셰이크를 한다. 눈에 보이지도 않는 1cm 이하 꺾임을 그대로
// 보내면 로봇이 제자리에서 멈칫거리기만 하고 오차만 쌓인다 → 미리 눌러 없앤다.
inline void dpKeep(const QList<QPointF> &pts, int a, int b, double tol, QVector<bool> &keep)
{
    if (b <= a + 1) return;
    const QPointF &p0 = pts[a];
    const QPointF &p1 = pts[b];
    const double dx = p1.x() - p0.x(), dy = p1.y() - p0.y();
    const double len = std::hypot(dx, dy);
    int worst = -1;
    double worstD = tol;
    for (int i = a + 1; i < b; ++i) {
        double d;
        if (len < 1e-12) {
            d = QLineF(p0, pts[i]).length();
        } else {
            // 점-직선 거리 (외적 / 길이)
            d = std::abs(dx * (p0.y() - pts[i].y()) - dy * (p0.x() - pts[i].x())) / len;
        }
        if (d > worstD) { worstD = d; worst = i; }
    }
    if (worst < 0) return;
    keep[worst] = true;
    dpKeep(pts, a, worst, tol, keep);
    dpKeep(pts, worst, b, tol, keep);
}

inline QList<QPointF> simplify(const QList<QPointF> &in, double tolM)
{
    // 1) 겹친 점 제거
    QList<QPointF> pts;
    for (const QPointF &p : in) {
        if (pts.isEmpty() || QLineF(pts.last(), p).length() > tolM)
            pts.append(p);
    }
    if (pts.size() < 3 || tolM <= 0.0) return pts;

    // 2) Douglas–Peucker (양 끝점은 항상 보존)
    QVector<bool> keep(pts.size(), false);
    keep.first() = keep.last() = true;
    dpKeep(pts, 0, pts.size() - 1, tolM, keep);

    QList<QPointF> out;
    for (int i = 0; i < pts.size(); ++i)
        if (keep[i]) out.append(pts[i]);
    return out;
}

// ⚠️ 여기 있던 ringLength(폴리라인 둘레) 는 지웠다 — 호출부 0.
//    경로 길이가 필요한 곳은 backend.cpp 의 polylineLength() 를 쓴다.

namespace detail {

// 도형 하나를 "이 꼭짓점에서 이 방향으로" 달릴 때의 실제 주행 순서를 만든다.
// 열린 도형: at 은 0 또는 마지막 (뒤집기만 가능)
// 닫힌 도형: 어느 꼭짓점에서 시작해도 되고 양방향 다 가능 → 마지막에 시작점으로 복귀
inline QList<QPointF> orient(const QList<QPointF> &pts, bool closed, int at, bool rev)
{
    QList<QPointF> out;
    const int n = pts.size();
    if (n == 0) return out;
    if (!closed) {
        if (!rev) for (int i = 0; i < n; ++i) out.append(pts[i]);
        else      for (int i = n - 1; i >= 0; --i) out.append(pts[i]);
        return out;
    }
    for (int k = 0; k < n; ++k) {
        const int i = rev ? ((at - k) % n + n) % n : (at + k) % n;
        out.append(pts[i]);
    }
    out.append(out.first());   // 마지막 변까지 칠하도록 시작점 복귀
    return out;
}

struct Cand { int at; bool rev; };

inline QList<Cand> entriesFor(const QList<QPointF> &pts, bool closed)
{
    QList<Cand> c;
    if (pts.size() < 2) return c;
    if (!closed) {
        c.append({0, false});
        c.append({int(pts.size()) - 1, true});
        return c;
    }
    for (int i = 0; i < pts.size(); ++i) {
        c.append({i, false});
        c.append({i, true});
    }
    return c;
}

} // namespace detail

// 재배열 없이(작도 순서 그대로) 이어붙였을 때의 도형 사이 이동거리.
// "얼마나 줄었는지"를 로그로 보여주기 위한 비교값.
inline double naiveTravel(const QList<QList<QPointF>> &paths, const QList<bool> &closed,
                          QPointF start, bool hasStart)
{
    double t = 0.0;
    QPointF cur = start;
    bool have = hasStart;
    for (int i = 0; i < paths.size(); ++i) {
        if (paths[i].size() < 2) continue;
        if (have) t += QLineF(cur, paths[i].first()).length();
        have = true;
        cur = closed.value(i, false) && paths[i].size() > 2 ? paths[i].first()
                                                            : paths[i].last();
    }
    return t;
}

// 도형들을 재배열·뒤집어 이동거리를 줄인 뒤 폴리라인 하나로 잇는다.
//   start/hasStart : 로봇의 현재 위치 (모르면 hasStart=false — 첫 도형은 작도 순서 유지)
//   tolM           : 경로 단순화 허용 오차 (기본 1cm = 서버가 버리는 최소 이동거리)
inline Route plan(const QList<QList<QPointF>> &paths, const QList<bool> &closed,
                  QPointF start, bool hasStart, double tolM = 0.01)
{
    Route r;

    // 정리 후 실제로 그릴 수 있는 도형만 남긴다
    QList<QList<QPointF>> shape;
    QList<bool> shapeClosed;
    QList<int> origIndex;
    for (int i = 0; i < paths.size(); ++i) {
        QList<QPointF> s = simplify(paths[i], tolM);
        if (s.size() < 2) continue;
        shape.append(s);
        shapeClosed.append(closed.value(i, false) && s.size() > 2);
        origIndex.append(i);
    }
    if (shape.isEmpty()) return r;

    QVector<bool> used(shape.size(), false);
    QPointF cur = start;
    bool have = hasStart;
    int nextShapeNo = 0;

    for (int done = 0; done < shape.size(); ++done) {
        int best = -1;
        detail::Cand bestC{0, false};
        double bestD = 0.0;

        if (!have) {
            // 로봇 위치를 모르면 첫 도형은 작도 순서·방향을 그대로 존중한다.
            // (사용자가 의도한 시작점을 임의로 바꾸지 않기 위함)
            best = 0;
            bestC = {0, false};
            bestD = 0.0;
        } else {
            for (int i = 0; i < shape.size(); ++i) {
                if (used[i]) continue;
                for (const detail::Cand &c : detail::entriesFor(shape[i], shapeClosed[i])) {
                    const double d = QLineF(cur, shape[i][c.at]).length();
                    if (best < 0 || d < bestD) { best = i; bestC = c; bestD = d; }
                }
            }
        }
        if (best < 0) break;
        used[best] = true;

        const QList<QPointF> run = detail::orient(shape[best], shapeClosed[best],
                                                  bestC.at, bestC.rev);
        if (run.size() < 2) continue;

        r.order.append(origIndex[best]);
        r.flipped.append(bestC.rev);

        // 앞 도형이 끝난 자리에서 그대로 이어지면 빈 이동 구간이 없다 → 두 도형을
        // 하나의 연속 구간으로 묶는다 (그래야 도형 경계 = pen-up 구간이 된다)
        const bool seam = !r.pts.isEmpty()
                          && QLineF(r.pts.last(), run.first()).length() <= tolM;
        const int shapeNo = seam ? r.shapeOf.last() : nextShapeNo++;

        if (r.pts.isEmpty()) {
            r.pts.append(run.first());
            r.paint.append(false);        // 시작점 — 서버가 여기까지 접근시킨다
            r.shapeOf.append(shapeNo);
        } else if (!seam) {
            r.travelM += QLineF(r.pts.last(), run.first()).length();
            r.pts.append(run.first());
            r.paint.append(false);        // ← pen-up: 도형 사이 이동은 칠하지 않는다
            r.shapeOf.append(shapeNo);
        }
        for (int i = 1; i < run.size(); ++i) {
            r.paintM += QLineF(run[i - 1], run[i]).length();
            r.pts.append(run[i]);
            r.paint.append(true);
            r.shapeOf.append(shapeNo);
        }
        cur = run.last();
        have = true;
    }
    r.shapeCount = nextShapeNo;   // 붙어버린 도형은 하나로 센다 (= pen-up 구간 수)
    return r;
}

} // namespace routeplan
