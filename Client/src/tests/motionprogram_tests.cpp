#include "motionprogram.h"
#include "paintgeometry.h"
#include "routeplan.h"
#include "strokefont.h"
#include "camcalib.h"

#include <QCoreApplication>
#include <QJsonArray>
#include <QJsonValue>
#include <QLineF>
#include <QList>
#include <QPointF>
#include <QSet>

#include <cmath>
#include <iostream>
#include <limits>

namespace {

int failures = 0;

void check(bool condition, const char *message)
{
    if (condition) return;
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
}

bool near(double actual, double expected, double eps = 1e-6)
{
    return std::abs(actual - expected) <= eps;
}

QList<QPointF> arcPoints(double radius, double startDeg, double sweepDeg, int segments)
{
    QList<QPointF> points;
    points.reserve(segments + 1);
    for (int i = 0; i <= segments; ++i) {
        const double a = (startDeg + sweepDeg * i / segments) * motionprogram::kDegToRad;
        points.append(QPointF(radius * std::cos(a), radius * std::sin(a)));
    }
    return points;
}

QList<motionprogram::Op> buildPainted(const QList<QPointF> &points)
{
    QList<bool> paint(points.size(), true);
    if (!paint.isEmpty()) paint[0] = false;
    return motionprogram::build(points, paint);
}

QList<motionprogram::Op> buildClosedLikeBackend(double rx, double ry, int samples)
{
    QList<QPointF> shape;
    for (int i = 0; i < samples; ++i) {
        const double a = 2.0 * motionprogram::kPi * i / samples;
        shape.append(QPointF(rx * std::cos(a), ry * std::sin(a)));
    }

    QList<QPointF> fitRun = shape;
    fitRun.append(fitRun.first());
    motionprogram::detail::Circle fit;
    double sweep = 0.0;
    bool left = true;
    const bool preserve = motionprogram::detail::arcFits(
        fitRun, 0, fitRun.size() - 1, fit, sweep, left);
    const routeplan::Route route = routeplan::plan(
        {shape}, {true}, {}, false, 0.01, {preserve});
    return motionprogram::build(route.pts, route.paint);
}

const motionprogram::Op *firstArc(const QList<motionprogram::Op> &ops)
{
    for (const motionprogram::Op &op : ops)
        if (op.kind == motionprogram::Op::Arc) return &op;
    return nullptr;
}

void testCircleOneArc()
{
    const QList<motionprogram::Op> ops = buildClosedLikeBackend(0.20, 0.20, 19);
    check(motionprogram::countOf(ops, motionprogram::Op::Arc) == 1,
          "200mm circle must produce one ARC");
    check(motionprogram::countOf(ops, motionprogram::Op::Move) == 0,
          "circle must not degrade into MOVE segments");
    check(motionprogram::countOf(ops, motionprogram::Op::Turn) == 0,
          "first ARC must not contain a duplicate approach TURN");
    const motionprogram::Op *arc = firstArc(ops);
    check(arc && near(arc->radius, 0.20, 1e-5), "circle radius must be preserved");
    check(arc && near(arc->angle, 360.0, 1e-5), "circle sweep must be 360 degrees");
    check(arc && near(arc->heading, 90.0, 1e-5),
          "CCW circle ARC heading must be its entry tangent");
    check(!ops.isEmpty() && near(ops.first().heading, 90.0, 1e-5),
          "server approach heading must use the ARC entry tangent");
}

void testPartialArcEntryHeading()
{
    const QList<motionprogram::Op> ccw = buildPainted(arcPoints(0.5, 0.0, 180.0, 18));
    const motionprogram::Op *left = firstArc(ccw);
    check(left && left->arcLeft, "CCW semicircle direction must be left");
    check(left && near(left->heading, 90.0, 1e-5),
          "CCW semicircle heading must be entry tangent, not exit tangent");
    check(motionprogram::countOf(ccw, motionprogram::Op::Turn) == 0,
          "first semicircle must not add a chord-to-tangent TURN");

    const QList<motionprogram::Op> cw = buildPainted(arcPoints(0.5, 0.0, -180.0, 18));
    const motionprogram::Op *right = firstArc(cw);
    check(right && !right->arcLeft, "CW semicircle direction must be right");
    check(right && near(right->heading, -90.0, 1e-5),
          "CW semicircle heading must be entry tangent");
}

void testServerRadiusPreflight()
{
    const QList<motionprogram::Op> tooSmall = buildClosedLikeBackend(0.0715, 0.0715, 19);
    double radius = 0.0;
    check(motionprogram::firstTooTightPaintArc(tooSmall, &radius) >= 0,
          "71.5mm painted ARC must be rejected before server upload");
    check(near(radius, 0.0715, 1e-5), "preflight must report the actual drawing radius");

    const QList<motionprogram::Op> unsafeBoundary = buildClosedLikeBackend(0.155, 0.155, 19);
    check(motionprogram::firstTooTightPaintArc(unsafeBoundary) >= 0,
          "155mm server boundary must be blocked until robot zero-radius handling is fixed");

    const QList<motionprogram::Op> motorUnsafe = buildClosedLikeBackend(0.160, 0.160, 19);
    check(motionprogram::firstTooTightPaintArc(motorUnsafe) >= 0,
          "160mm ARC must be blocked by the confirmed motor-speed limit");

    const QList<motionprogram::Op> confirmedBoundary = buildClosedLikeBackend(0.200, 0.200, 19);
    check(motionprogram::firstTooTightPaintArc(confirmedBoundary) < 0,
          "confirmed 200mm boundary must be accepted");

    motionprogram::Op roundedUp;
    roundedUp.kind = motionprogram::Op::Arc;
    roundedUp.paint = true;
    roundedUp.radius = 0.19996;
    check(motionprogram::firstTooTightPaintArc({roundedUp}) < 0,
          "199.96mm must be accepted because the wire value is 200.0mm");

    roundedUp.radius = 0.19994;
    check(motionprogram::firstTooTightPaintArc({roundedUp}) >= 0,
          "199.94mm must be rejected because the wire value is 199.9mm");
}

void testArcResizeConstraint()
{
    check(near(motionprogram::constrainPaintArcScale(0.50, 0.10), 0.40),
          "500mm ARC must stop shrinking at the 200mm boundary");
    check(near(motionprogram::constrainPaintArcScale(0.20, 0.90), 1.0),
          "ARC already at the boundary must not shrink further");
    check(near(motionprogram::constrainPaintArcScale(0.15, 0.80), 1.0),
          "legacy invalid ARC must not shrink further");
    check(near(motionprogram::constrainPaintArcScale(0.15, 1.20), 1.20),
          "legacy invalid ARC must remain freely enlargeable");
}

void testEllipseIsNotOneCircle()
{
    const QList<motionprogram::Op> ops = buildClosedLikeBackend(1.03, 1.0, 20);
    const motionprogram::Op *arc = firstArc(ops);
    const bool oneFullCircle = motionprogram::countOf(ops, motionprogram::Op::Arc) == 1
        && motionprogram::countOf(ops, motionprogram::Op::Move) == 0
        && arc && near(arc->angle, 360.0, 1e-5);
    check(!oneFullCircle, "3% ellipse must not be silently replaced by one circle");
}

void testUniformResizeRoundTripKeepsCircle()
{
    QList<QPointF> shape;
    constexpr int samples = 24;
    for (int i = 0; i < samples; ++i) {
        const double a = 2.0 * motionprogram::kPi * i / samples;
        shape.append(QPointF(0.20 * std::cos(a), 0.20 * std::sin(a)));
    }
    const QList<QPointF> original = shape;
    for (QPointF &p : shape) p *= 0.1;
    for (QPointF &p : shape) p *= 10.0;

    check(shape.size() == samples, "resize must never delete curve samples");
    for (int i = 0; i < shape.size(); ++i)
        check(QLineF(shape[i], original[i]).length() < 1e-10,
              "uniform shrink/grow must restore every curve sample");

    QList<QPointF> run = shape;
    run.append(run.first());
    QList<bool> paint(run.size(), true);
    paint[0] = false;
    const QList<motionprogram::Op> ops = motionprogram::build(run, paint);
    check(motionprogram::countOf(ops, motionprogram::Op::Arc) == 1,
          "resized circle must remain one ARC");
    check(motionprogram::countOf(ops, motionprogram::Op::Move) == 0,
          "resized circle must not become MOVE segments");
}

void testSharedServerQuarterArcFixture()
{
    QList<QPointF> points = arcPoints(0.5, -90.0, 90.0, 18);
    for (QPointF &p : points) p += QPointF(0.0, 0.5);

    check(QLineF(points.first(), QPointF(0.0, 0.0)).length() < 1e-9,
          "shared fixture must start at (0,0)");
    check(QLineF(points.last(), QPointF(0.5, 0.5)).length() < 1e-9,
          "shared fixture must end at (0.5,0.5)");

    const QList<motionprogram::Op> ops = buildPainted(points);
    const motionprogram::Op *arc = firstArc(ops);
    check(motionprogram::countOf(ops, motionprogram::Op::Arc) == 1,
          "shared fixture must produce one ARC");
    check(motionprogram::countOf(ops, motionprogram::Op::Move) == 0,
          "shared fixture must not degrade into MOVE segments");
    check(motionprogram::countOf(ops, motionprogram::Op::Turn) == 0,
          "server approach must make a first-ARC TURN unnecessary");
    check(arc && arc->arcLeft, "shared fixture direction must be left");
    check(arc && near(arc->radius, 0.5, 1e-6), "shared fixture radius must be 0.5m");
    check(arc && near(arc->angle, 90.0, 1e-6), "shared fixture sweep must be 90 degrees");
    check(arc && near(arc->heading, 0.0, 1e-6),
          "shared fixture heading must be the 0-degree entry tangent");
    check(arc && near(arc->dist, 0.5 * motionprogram::kPi / 2.0, 1e-6),
          "shared fixture arc length must be R*pi/2");
}

void testFinishedRectangleUsesTipCenterPath()
{
    const QVector<QPointF> outer{
        QPointF(0.0, 0.0), QPointF(500.0, 0.0),
        QPointF(500.0, 300.0), QPointF(0.0, 300.0)
    };
    const paintgeometry::Result tip50 = paintgeometry::centerlineFor(outer, true, true, 50.0);
    check(tip50.ok, "500x300 outer rectangle with 50mm tip must be valid");
    check(tip50.kind == paintgeometry::Kind::PolygonInset,
          "rectangle must use deterministic polygon inset");
    check(near(QLineF(tip50.points[0], tip50.points[1]).length(), 450.0),
          "50mm tip center path width must be outer width minus 50mm");
    check(near(QLineF(tip50.points[1], tip50.points[2]).length(), 250.0),
          "50mm tip center path height must be outer height minus 50mm");

    const paintgeometry::Result tip60 = paintgeometry::centerlineFor(outer, true, true, 60.0);
    check(tip60.ok, "same outer rectangle with 60mm tip must be valid");
    check(near(QLineF(tip60.points[0], tip60.points[1]).length(), 440.0),
          "changing to 60mm tip must preserve outer width and only move center path");
    check(near(QLineF(tip60.points[1], tip60.points[2]).length(), 240.0),
          "changing to 60mm tip must preserve outer height and only move center path");

    // The editable/displayed points remain the user's finished outer contour.
    // Only the derived wire path changes with the selected marker width.
    check(near(QLineF(outer[0], outer[1]).length(), 500.0),
          "displayed outer width must remain 500mm for every tip width");
    check(near(QLineF(outer[1], outer[2]).length(), 300.0),
          "displayed outer height must remain 300mm for every tip width");
}

void testFinished180mmSquareSerializes130mmCenterMoves()
{
    const QVector<QPointF> outer{
        QPointF(0.0, 0.0), QPointF(0.180, 0.0),
        QPointF(0.180, 0.180), QPointF(0.0, 0.180)
    };
    const paintgeometry::Result center =
        paintgeometry::centerlineFor(outer, true, true, 0.050);
    check(center.ok, "180mm finished square with 50mm tip must be valid");
    for (int i = 0; i < center.points.size(); ++i) {
        const int next = (i + 1) % center.points.size();
        check(near(QLineF(center.points[i], center.points[next]).length(), 0.130, 1e-6),
              "180mm finished edge must serialize as a 130mm tip-center move");
    }

    QList<QPointF> run(center.points.begin(), center.points.end());
    run.append(run.first());
    const QList<motionprogram::Op> ops = buildPainted(run);
    double paintedMoveM = 0.0;
    int paintedMoves = 0;
    for (const motionprogram::Op &op : ops) {
        if (op.kind == motionprogram::Op::Move && op.paint) {
            ++paintedMoves;
            paintedMoveM += std::abs(op.dist);
            check(near(std::abs(op.dist), 0.130, 1e-6),
                  "wire MOVE distance must be 130mm, never the 180mm outer edge");
        }
    }
    check(paintedMoves == 4, "finished square must serialize as four painted MOVE ops");
    check(near(paintedMoveM, 0.520, 1e-6),
          "finished square wire perimeter must follow the 520mm center path");
}

void testFinishedCircleRemainsOneArc()
{
    QVector<QPointF> outer;
    constexpr int samples = 24;
    for (int i = 0; i < samples; ++i) {
        const double a = 2.0 * motionprogram::kPi * i / samples;
        outer.append(QPointF(0.250 * std::cos(a), 0.250 * std::sin(a)));
    }
    const paintgeometry::Result center = paintgeometry::centerlineFor(outer, true, true, 0.050);
    check(center.ok && center.kind == paintgeometry::Kind::ArcInset,
          "finished circle must use analytic radial inset");
    check(near(center.outerRadius, 0.250, 1e-5), "finished radius must stay 250mm");
    check(near(center.centerRadius, 0.225, 1e-5), "50mm tip center radius must be 225mm");

    QList<QPointF> run(center.points.begin(), center.points.end());
    run.append(run.first());
    const QList<motionprogram::Op> ops = buildPainted(run);
    const motionprogram::Op *arc = firstArc(ops);
    check(motionprogram::countOf(ops, motionprogram::Op::Arc) == 1,
          "inset finished circle must still serialize as exactly one ARC");
    check(arc && near(arc->radius, 0.225, 1e-5),
          "serialized ARC must use the tip center radius");
    check(arc && near(arc->dist, 2.0 * motionprogram::kPi * 0.225, 1e-5),
          "serialized full-circle ARC distance must use center radius times sweep");
}

void testFinishedPartialArcPreservesSweep()
{
    const QList<QPointF> source = arcPoints(0.500, -90.0, 90.0, 18);
    const QVector<QPointF> outer(source.begin(), source.end());
    const paintgeometry::Result center = paintgeometry::centerlineFor(outer, false, true, 0.060);
    check(center.ok && center.kind == paintgeometry::Kind::ArcInset,
          "open finished arc must use analytic radial inset");
    check(near(center.centerRadius, 0.470, 1e-5),
          "60mm tip must inset a 500mm arc to 470mm");
    check(near(center.sweepDeg, 90.0, 1e-5),
          "partial arc inset must preserve its sweep angle");

    const QList<motionprogram::Op> ops = buildPainted(
        QList<QPointF>(center.points.begin(), center.points.end()));
    const motionprogram::Op *arc = firstArc(ops);
    check(motionprogram::countOf(ops, motionprogram::Op::Arc) == 1,
          "inset partial arc must serialize as one ARC");
    check(arc && near(arc->radius, 0.470, 1e-5),
          "partial ARC wire radius must be the tip center radius");
    check(arc && near(arc->angle, 90.0, 1e-5),
          "partial ARC wire sweep must remain unchanged");
    check(arc && near(arc->dist, 0.470 * motionprogram::kPi / 2.0, 1e-5),
          "partial ARC wire distance must use center radius times sweep");
}

void testOuterArcMinimumConstraint()
{
    check(near(paintgeometry::minimumOuterArcRadius(0.05, 0.20), 0.225),
          "50mm tip minimum finished outer radius must be 225mm");
    check(near(paintgeometry::minimumOuterArcRadius(0.06, 0.20), 0.230),
          "60mm tip minimum finished outer radius must be 230mm");
    check(near(paintgeometry::constrainOuterArcScale(0.50, 0.06, 0.20, 0.10), 0.46),
          "outer ARC resize must stop where center radius reaches 200mm");
}

void testInvalidFinishedContoursAreRejected()
{
    const QVector<QPointF> fieldRectangle{
        QPointF(0.0, 0.0), QPointF(374.0, 0.0),
        QPointF(374.0, 374.0), QPointF(0.0, 374.0)
    };
    check(paintgeometry::centerlineFor(fieldRectangle, true, true, 50.0).ok,
          "374mm field rectangle with 50mm tip must be valid");
    check(paintgeometry::centerlineFor(fieldRectangle, true, true, 60.0).ok,
          "374mm field rectangle with 60mm tip must be valid");
    check(!paintgeometry::centerlineFor(fieldRectangle, true, true, 500.0).ok,
          "374mm field rectangle with accidental 500mm tip must be rejected");

    const QVector<QPointF> tooSmall{
        QPointF(0.0, 0.0), QPointF(40.0, 0.0),
        QPointF(40.0, 40.0), QPointF(0.0, 40.0)
    };
    check(!paintgeometry::centerlineFor(tooSmall, true, true, 50.0).ok,
          "finished rectangle smaller than the marker width must be rejected");

    const QVector<QPointF> concave{
        QPointF(0.0, 0.0), QPointF(200.0, 0.0), QPointF(100.0, 60.0),
        QPointF(200.0, 200.0), QPointF(0.0, 200.0)
    };
    check(!paintgeometry::centerlineFor(concave, true, true, 50.0).ok,
          "concave finished contour must not be silently changed by offset trimming");
}


// ── 아래는 2026-08-12 안정화 작업으로 추가한 회귀 테스트 ────────────────────
// 목적: (a) 유효한 원/호가 표본 수와 무관하게 ARC 로 남는가,
//       (b) 직선 정리·경로 계획이 곡선을 파괴하지 않는가,
//       (c) 펜촉 폭은 중심선을 바꾸고 로봇중심-펜 거리는 전송값을 바꾸지 않는가,
//       (d) ARC 로 분류되지 않은 곡선도 최소 도색 반지름 검사를 받는가.

// Backend::buildRoute 와 같은 규칙으로 경로를 계획한다 (preservePoints 계산 포함).
routeplan::Route planLikeBackend(const QList<QList<QPointF>> &paths,
                                 const QList<bool> &closed)
{
    QList<bool> preserve;
    for (int i = 0; i < paths.size(); ++i) {
        QList<QPointF> run = paths[i];
        if (closed.value(i, false) && run.size() >= 3) run.append(run.first());
        motionprogram::detail::Circle fit;
        double sweep = 0.0;
        bool left = true;
        preserve.append(run.size() >= 5
                        && motionprogram::detail::arcFits(run, 0, run.size() - 1,
                                                          fit, sweep, left));
    }
    return routeplan::plan(paths, closed, QPointF(), false, 0.01, preserve);
}

QList<motionprogram::Op> programFor(const QList<QList<QPointF>> &paths,
                                    const QList<bool> &closed)
{
    const routeplan::Route route = planLikeBackend(paths, closed);
    return motionprogram::build(route.pts, route.paint);
}

QList<QPointF> circlePoints(double radius, int samples, QPointF center = QPointF())
{
    QList<QPointF> shape;
    for (int i = 0; i < samples; ++i) {
        const double a = 2.0 * motionprogram::kPi * i / samples;
        shape.append(center + QPointF(radius * std::cos(a), radius * std::sin(a)));
    }
    return shape;
}

bool sameOps(const QList<motionprogram::Op> &a, const QList<motionprogram::Op> &b)
{
    if (a.size() != b.size()) return false;
    for (int i = 0; i < a.size(); ++i) {
        if (a[i].kind != b[i].kind) return false;
        if (!near(a[i].dist, b[i].dist, 1e-12)) return false;
        if (!near(a[i].angle, b[i].angle, 1e-12)) return false;
        if (!near(a[i].radius, b[i].radius, 1e-12)) return false;
        if (!near(a[i].heading, b[i].heading, 1e-12)) return false;
        if (a[i].arcLeft != b[i].arcLeft) return false;
        if (a[i].paint != b[i].paint) return false;
        if (a[i].vertex != b[i].vertex) return false;
    }
    return true;
}

// (a) 표본 수 16/24/36/64 의 완전한 원은 전부 ARC 하나로 남아야 한다.
void testCircleSampleDensityKeepsOneArc()
{
    for (int samples : { 16, 24, 36, 64 }) {
        const QList<motionprogram::Op> ops =
            programFor({ circlePoints(0.25, samples) }, { true });
        const QString label = QStringLiteral("circle with %1 samples").arg(samples);
        check(motionprogram::countOf(ops, motionprogram::Op::Arc) == 1,
              qPrintable(label + " must produce exactly one ARC"));
        check(motionprogram::countOf(ops, motionprogram::Op::Move) == 0,
              qPrintable(label + " must not degrade into MOVE segments"));
        const motionprogram::Op *arc = firstArc(ops);
        check(arc && near(arc->radius, 0.25, 1e-3),
              qPrintable(label + " must keep its 250mm radius"));
        check(arc && near(arc->angle, 360.0, 1e-6),
              qPrintable(label + " must keep a 360 degree sweep"));
    }
}

// (a) 좌/우 부분호도 ARC 하나로 남고 방향·스윕이 보존돼야 한다.
void testPartialArcsBothDirections()
{
    const QList<motionprogram::Op> ccw = programFor({ arcPoints(0.30, 10.0, 120.0, 12) },
                                                    { false });
    const motionprogram::Op *l = firstArc(ccw);
    check(motionprogram::countOf(ccw, motionprogram::Op::Arc) == 1,
          "left partial arc must stay one ARC");
    check(motionprogram::countOf(ccw, motionprogram::Op::Move) == 0,
          "left partial arc must not become MOVE segments");
    check(l && l->arcLeft && near(l->angle, 120.0, 1e-6) && near(l->radius, 0.30, 1e-3),
          "left partial arc must keep direction, sweep and radius");

    const QList<motionprogram::Op> cw = programFor({ arcPoints(0.30, 10.0, -120.0, 12) },
                                                   { false });
    const motionprogram::Op *r = firstArc(cw);
    check(motionprogram::countOf(cw, motionprogram::Op::Arc) == 1,
          "right partial arc must stay one ARC");
    check(r && !r->arcLeft && near(r->angle, 120.0, 1e-6) && near(r->radius, 0.30, 1e-3),
          "right partial arc must keep direction, sweep and radius");
}

// (b) 직선 + 반원(혼합 도형)은 직선은 줄이되 반원은 ARC 하나로 남아야 한다.
void testLinePlusSemicirclePreservesArc()
{
    QList<QPointF> shape;
    for (int i = 0; i <= 10; ++i)                        // 직선 1m (중간 점 다수)
        shape.append(QPointF(-1.0 + i * 0.1, 0.0));
    const QList<QPointF> half = arcPoints(0.30, 180.0, -180.0, 18);
    for (int i = 1; i < half.size(); ++i)                // 반원 R=300mm
        shape.append(half[i] + QPointF(0.30, 0.0));

    const routeplan::Route route = planLikeBackend({ shape }, { false });
    const QList<motionprogram::Op> ops = motionprogram::build(route.pts, route.paint);
    const motionprogram::Op *arc = firstArc(ops);
    check(motionprogram::countOf(ops, motionprogram::Op::Arc) == 1,
          "line+semicircle must still serialize the curve as one ARC");
    check(arc && near(arc->radius, 0.30, 1e-3),
          "line+semicircle ARC radius must stay 300mm");
    check(arc && near(arc->angle, 180.0, 1e-3),
          "line+semicircle ARC sweep must stay 180 degrees");
    check(motionprogram::countOf(ops, motionprogram::Op::Move) == 1,
          "the straight part must collapse into a single MOVE");
}

// (b) 화면 "직선 정리"(RDP)도 곡선 샘플을 지우면 안 된다.
//     videoview 의 rdpSimplifyPreservingCurves 와 같은 규칙인 routeplan 쪽으로 검증한다.
void testSimplifyKeepsCurvesAndStraightBehaviour()
{
    const QList<QPointF> circle = circlePoints(0.25, 36);
    QList<QPointF> ring = circle;
    ring.append(ring.first());
    const QList<QPointF> kept = routeplan::simplifyPreservingCurves(ring, 0.01);
    check(kept.size() == ring.size(), "curve samples must survive simplification");

    // 직선 도형은 기존 simplify() 와 동일한 결과여야 한다 (동작 불변).
    const QList<QPointF> line{ QPointF(0, 0), QPointF(0.5, 0.0005), QPointF(1.0, 0.0) };
    const QList<QPointF> tri{ QPointF(0, 0), QPointF(1, 0), QPointF(0.5, 0.9), QPointF(0, 0) };
    const QList<QPointF> rect{ QPointF(0, 0), QPointF(1, 0), QPointF(1, 0.6),
                               QPointF(0, 0.6), QPointF(0, 0) };
    const QList<QPointF> poly{ QPointF(0, 0), QPointF(0.8, 0), QPointF(1.0, 0.5),
                               QPointF(0.5, 1.0), QPointF(0, 0.7), QPointF(0, 0) };
    for (const QList<QPointF> &shape : { line, tri, rect, poly })
        check(routeplan::simplifyPreservingCurves(shape, 0.01) == routeplan::simplify(shape, 0.01),
              "straight shapes must keep the previous simplification result");
}

// (b) 직선 도형의 전송 결과가 그대로인지 (선/삼각형/사각형/다각형 회귀)
void testStraightShapesRegression()
{
    const QList<motionprogram::Op> line =
        programFor({ { QPointF(0, 0), QPointF(1.0, 0.0) } }, { false });
    check(motionprogram::countOf(line, motionprogram::Op::Move) == 1
          && motionprogram::countOf(line, motionprogram::Op::Arc) == 0,
          "straight line must remain a single MOVE");

    const QList<motionprogram::Op> tri =
        programFor({ { QPointF(0, 0), QPointF(1, 0), QPointF(0.5, 0.9) } }, { true });
    check(motionprogram::countOf(tri, motionprogram::Op::Move) == 3
          && motionprogram::countOf(tri, motionprogram::Op::Turn) == 2
          && motionprogram::countOf(tri, motionprogram::Op::Arc) == 0,
          "triangle must remain 3 MOVE + 2 TURN");

    const QList<motionprogram::Op> rect =
        programFor({ { QPointF(0, 0), QPointF(1, 0), QPointF(1, 0.6), QPointF(0, 0.6) } },
                   { true });
    check(motionprogram::countOf(rect, motionprogram::Op::Move) == 4
          && motionprogram::countOf(rect, motionprogram::Op::Turn) == 3
          && motionprogram::countOf(rect, motionprogram::Op::Arc) == 0,
          "rectangle must remain 4 MOVE + 3 TURN");

    const QList<motionprogram::Op> poly =
        programFor({ { QPointF(0, 0), QPointF(0.8, 0), QPointF(1.0, 0.5),
                       QPointF(0.5, 1.0), QPointF(0, 0.7) } }, { true });
    check(motionprogram::countOf(poly, motionprogram::Op::Arc) == 0
          && motionprogram::countOf(poly, motionprogram::Op::Move) == 5,
          "convex polygon must remain MOVE/TURN only");
}

// (b) 가깝지만 서로 다른 두 도형을 이어붙일 때 사용자 기하가 바뀌면 안 된다.
void testNearbyShapesKeepTheirOwnGeometry()
{
    // 앞 도형의 끝점이 원의 시작점에서 8mm 떨어져 있다.
    //  · 예전①: 원의 첫 점을 버리고 앞 끝점을 대신 씀 → 그린 적 없는 R≈468mm ARC 2개
    //  · 예전②: 첫 점은 살리되 8mm 를 paint=true 로 칠함 → 안 그린 연결선을 도색
    //  · 지금  : 두 도형의 점을 모두 보존하고, 8mm 는 pen-up(이동)으로 낸다.
    const QList<QPointF> lead{ QPointF(0.0, 0.0), QPointF(0.492, 0.0) };
    const QList<QPointF> circle = circlePoints(0.50, 24);   // (0.5,0) 에서 시작
    const routeplan::Route route = planLikeBackend({ lead, circle }, { false, true });

    int connector = -1;
    for (int i = 1; i < route.pts.size(); ++i) {
        if (QLineF(route.pts[i - 1], route.pts[i]).length() > 1e-9
            && QLineF(route.pts[i], circle.first()).length() < 1e-9) { connector = i; break; }
    }
    check(connector > 0, "the joined shape must keep its own first point");
    check(connector > 0 && route.paint.value(connector) == false,
          "the 8mm connector the user never drew must not be painted");
    check(connector > 0
          && near(QLineF(route.pts[connector - 1], route.pts[connector]).length(), 0.008, 1e-9),
          "the connector must be exactly the 8mm gap between the two shapes");
    check(near(route.travelM, 0.008, 1e-9),
          "the connector must be counted as travel, not paint");

    // paintM 은 그린 구간만 = 직선 0.492m + 원 둘레(24각형 근사)
    double drawnM = 0.492;
    for (int i = 1; i < circle.size(); ++i) drawnM += QLineF(circle[i - 1], circle[i]).length();
    drawnM += QLineF(circle.last(), circle.first()).length();
    check(near(route.paintM, drawnM, 1e-9),
          "paintM must exclude the connector and match what the user drew");

    // 이어붙인 뒤에도 원은 정확히 R=500mm 하나여야 한다.
    const QList<motionprogram::Op> ops = motionprogram::build(route.pts, route.paint);
    const motionprogram::Op *arc = firstArc(ops);
    check(motionprogram::countOf(ops, motionprogram::Op::Arc) == 1,
          "a circle entered near another shape must stay one ARC");
    check(arc && near(arc->radius, 0.50, 1e-3),
          "seam handling must not invent a new radius");
    check(arc && near(arc->angle, 360.0, 1e-6), "the circle must keep a full 360 sweep");
    // 연결 구간은 도색 플래그가 바뀌므로 NOZZLE up/down 이 정확히 그 자리에 들어간다.
    check(motionprogram::countOf(ops, motionprogram::Op::Nozzle) == 4,
          "nozzle must lift for the connector and lower again for the circle");
}

// (c) 펜촉 폭(50/60mm)은 중심선을 바꾸고, 그 결과가 전송 반지름에 반영된다.
void testNibWidthChangesCenterlineOnly()
{
    const QList<QPointF> outerList = circlePoints(0.30, 36);
    const QVector<QPointF> outer(outerList.begin(), outerList.end());
    for (double nib : { 0.05, 0.06 }) {
        const paintgeometry::Result centered =
            paintgeometry::centerlineFor(outer, true, true, nib);
        check(centered.ok, "outer circle must produce a tip centerline");
        check(near(centered.centerRadius, 0.30 - nib / 2.0, 1e-9),
              "tip center radius must be outer radius minus half the nib width");
        QList<QPointF> path(centered.points.begin(), centered.points.end());
        const QList<motionprogram::Op> ops = programFor({ path }, { true });
        const motionprogram::Op *arc = firstArc(ops);
        check(arc && near(arc->radius, 0.30 - nib / 2.0, 1e-3),
              "transmitted ARC radius must follow the nib-aware centerline");
    }
    const paintgeometry::Result wide =
        paintgeometry::centerlineFor(outer, true, true, 0.06);
    const paintgeometry::Result narrow =
        paintgeometry::centerlineFor(outer, true, true, 0.05);
    check(!near(wide.centerRadius, narrow.centerRadius, 1e-9),
          "50mm and 60mm nibs must not produce the same centerline");
}

// (c) 로봇중심-펜 거리는 표시 전용이다 — 전송되는 어떤 값도 바뀌면 안 된다.
void testPenDistanceDoesNotChangeTransmittedPath()
{
    const QList<QPointF> outerList = circlePoints(0.30, 24);
    const QVector<QPointF> outer(outerList.begin(), outerList.end());
    const paintgeometry::Result centered =
        paintgeometry::centerlineFor(outer, true, true, 0.05);
    QList<QPointF> path(centered.points.begin(), centered.points.end());
    const QList<QList<QPointF>> mixed{
        path,
        { QPointF(2.0, 0.0), QPointF(3.0, 0.0), QPointF(3.0, 0.8) }
    };
    const QList<bool> closed{ true, false };

    const routeplan::Route base = planLikeBackend(mixed, closed);
    const QList<motionprogram::Op> baseOps = motionprogram::build(base.pts, base.paint);

    for (double penOffsetM : { 0.100, 0.155, 0.200 }) {
        // 펜 거리는 표시 좌표에만 들어간다 (backend 의 pushPoseToView 경로와 동일).
        const QPointF marker =
            paintgeometry::penMarkerFromRobotCenter(1.0, 2.0, 30.0, penOffsetM);
        check(std::isfinite(marker.x()) && std::isfinite(marker.y()),
              "pen marker position must be finite");

        const routeplan::Route again = planLikeBackend(mixed, closed);
        const QList<motionprogram::Op> ops = motionprogram::build(again.pts, again.paint);
        check(again.pts == base.pts, "transmitted points must not depend on pen distance");
        check(again.paint == base.paint, "paint flags must not depend on pen distance");
        check(sameOps(ops, baseOps),
              "MOVE/ARC/TURN, radius, sweep and headings must not depend on pen distance");
        check(near(motionprogram::totalTravelM(ops), motionprogram::totalTravelM(baseOps), 1e-12),
              "path length must not depend on pen distance");
    }
    // 거리를 바꾸면 화면 표시 위치는 실제로 달라져야 한다 (표시 기능 자체는 살아 있다).
    const QPointF a = paintgeometry::penMarkerFromRobotCenter(1.0, 2.0, 30.0, 0.100);
    const QPointF b = paintgeometry::penMarkerFromRobotCenter(1.0, 2.0, 30.0, 0.200);
    check(QLineF(a, b).length() > 0.09, "pen distance must still move the displayed marker");
    const QPointF back = paintgeometry::robotCenterFromPen(a.x(), a.y(), 30.0, 0.100);
    check(near(back.x(), 1.0, 1e-9) && near(back.y(), 2.0, 1e-9),
          "pen marker mapping must be reversible");
}

// (d) ARC 로 분류되지 않은 곡선도 최소 도색 반지름 검사를 받아야 한다.
void testTightCurveWithoutArcOpIsStillRejected()
{
    const QList<QPointF> tiny = circlePoints(0.015, 24);    // R=15mm, ARC 판정 실패
    const routeplan::Route route = planLikeBackend({ tiny }, { true });
    const QList<motionprogram::Op> ops = motionprogram::build(route.pts, route.paint);
    check(motionprogram::countOf(ops, motionprogram::Op::Arc) == 0,
          "fixture must exercise the non-ARC path (no ARC op expected)");
    check(motionprogram::firstTooTightPaintArc(ops) < 0,
          "the ARC-only preflight cannot see this shape (documents the gap)");
    double radiusM = 0.0;
    check(motionprogram::firstTooTightPaintCurve(route.pts, route.paint, &radiusM) >= 0,
          "curved geometry below 200mm must be rejected even without an ARC op");
    check(radiusM < 0.20, "the reported radius must be the offending curve radius");

    const QList<QPointF> wide = circlePoints(0.25, 24);
    const routeplan::Route ok = planLikeBackend({ wide }, { true });
    check(motionprogram::firstTooTightPaintCurve(ok.pts, ok.paint) < 0,
          "a 250mm circle must pass the curve radius check");

    // 다각형의 꼭짓점은 곡선이 아니다 — 직선 도형이 막히면 안 된다.
    const QList<QPointF> smallRect{ QPointF(0, 0), QPointF(0.05, 0), QPointF(0.05, 0.05),
                                    QPointF(0, 0.05) };
    const routeplan::Route rect = planLikeBackend({ smallRect }, { true });
    check(motionprogram::firstTooTightPaintCurve(rect.pts, rect.paint) < 0,
          "a small rectangle must not be treated as a tight curve");
}

// (b/c) 단위 명시: 같은 원을 미터와 TopView 픽셀로 판정했을 때 결과가 같아야 한다.
void testArcFitUnitsAreExplicit()
{
    const double pxPerM = 200.0;
    QList<QPointF> meters = circlePoints(0.25, 24);
    meters.append(meters.first());
    QList<QPointF> pixels;
    for (const QPointF &p : meters) pixels.append(p * pxPerM);

    motionprogram::detail::Circle fm, fp;
    double sm = 0.0, sp = 0.0;
    bool lm = true, lp = true;
    const bool okM = motionprogram::detail::arcFits(meters, 0, meters.size() - 1, fm, sm, lm, 1.0);
    const bool okP = motionprogram::detail::arcFits(pixels, 0, pixels.size() - 1, fp, sp, lp, pxPerM);
    check(okM && okP, "the same circle must be accepted in metres and in pixels");
    check(near(fp.r / pxPerM, fm.r, 1e-6), "pixel fit radius must scale back to the metre radius");
    check(near(sp, sm, 1e-6) && lp == lm, "sweep and direction must not depend on the unit");

    // 화면 판정이 전송보다 느슨하면 "화면은 통과, 전송은 거부"가 생긴다.
    const QList<QPointF> tinyM = circlePoints(0.005, 24);   // R=5mm < 20mm 하한
    QList<QPointF> tinyRing = tinyM;
    tinyRing.append(tinyRing.first());
    QList<QPointF> tinyPx;
    for (const QPointF &p : tinyRing) tinyPx.append(p * pxPerM);
    motionprogram::detail::Circle f2;
    double s2 = 0.0;
    bool l2 = true;
    const bool tinyOkM = motionprogram::detail::arcFits(tinyRing, 0, tinyRing.size() - 1,
                                                       f2, s2, l2, 1.0);
    const bool tinyOkPx = motionprogram::detail::arcFits(tinyPx, 0, tinyPx.size() - 1,
                                                        f2, s2, l2, pxPerM);
    check(tinyOkM == tinyOkPx,
          "a shape below the minimum fit radius must be classified the same in both units");
}

// (a) 기존 생성기(strokefont)의 'O' 획도 곡선으로 남아야 한다.
void testStrokeFontRoundGlyphKeepsCurve()
{
    double advance = 0.0;
    const strokefont::Glyph glyph = strokefont::glyphFor(QChar('O'), &advance);
    check(!glyph.isEmpty(), "stroke font must provide an O glyph");
    if (glyph.isEmpty()) return;

    const double scale = 2.0;      // em 1.0 → 2m (반지름이 200mm 하한을 넘도록)
    QList<QPointF> shape;
    for (const QPointF &p : glyph.first()) shape.append(p * scale);
    const bool closed = QLineF(shape.first(), shape.last()).length() < 1e-6;
    if (closed) shape.removeLast();

    const routeplan::Route route = planLikeBackend({ shape }, { closed });
    const QList<motionprogram::Op> ops = motionprogram::build(route.pts, route.paint);
    check(motionprogram::countOf(ops, motionprogram::Op::Arc) >= 1,
          "the round glyph must still serialize as ARC operations");
    check(motionprogram::countOf(ops, motionprogram::Op::Move)
          <= motionprogram::countOf(ops, motionprogram::Op::Arc) + 2,
          "the round glyph must not explode into a MOVE pile");
}

// (CH1~CH4) 저장·적용 전 H 검증. Backend::calibHasUsableH 가 이 함수를 그대로 쓴다.
// 요청한 응답인지와 무관하게 같은 검사를 받으므로, 요청하지 않았거나 늦게 온
// 번들도 여기서 걸러진다.
void testHomographyValidation()
{
    auto row = [](double a, double b, double c) {
        QJsonArray r; r.append(a); r.append(b); r.append(c); return r;
    };
    QJsonArray good;
    good.append(row(0.5, 0.0, 10.0));
    good.append(row(0.0, 0.5, 20.0));
    good.append(row(0.0, 0.0, 1.0));
    check(camcalib::hasUsable3x3(good), "a well-formed non-singular 3x3 H must be accepted");

    QJsonArray twoRows;
    twoRows.append(row(1, 0, 0));
    twoRows.append(row(0, 1, 0));
    check(!camcalib::hasUsable3x3(twoRows), "a 2x3 matrix must be rejected");

    QJsonArray shortRow;
    shortRow.append(row(1, 0, 0));
    shortRow.append(row(0, 1, 0));
    QJsonArray two; two.append(0.0); two.append(1.0);
    shortRow.append(two);
    check(!camcalib::hasUsable3x3(shortRow), "a row with two entries must be rejected");

    QJsonArray textCell;
    textCell.append(row(1, 0, 0));
    textCell.append(row(0, 1, 0));
    QJsonArray bad; bad.append(QStringLiteral("0")); bad.append(0.0); bad.append(1.0);
    textCell.append(bad);
    check(!camcalib::hasUsable3x3(textCell), "non-numeric cells must be rejected");

    QJsonArray nonFinite;
    nonFinite.append(row(1, 0, 0));
    nonFinite.append(row(0, std::numeric_limits<double>::infinity(), 0));
    nonFinite.append(row(0, 0, 1));
    check(!camcalib::hasUsable3x3(nonFinite), "non-finite cells must be rejected");

    QJsonArray singular;
    singular.append(row(1, 2, 3));
    singular.append(row(2, 4, 6));
    singular.append(row(0, 0, 1));
    check(!camcalib::hasUsable3x3(singular), "a singular H must be rejected");

    check(!camcalib::hasUsable3x3(QJsonArray()), "an empty H must be rejected");
}

// ── 3) 곡선의 "물리 반지름"은 잔차 검증을 통과했을 때만 보고한다 ─────────────
// 구분:
//   (a) 단순화 보존   — curveRuns 가 잡은 구간의 점은 지우지 않는다 (보수적)
//   (b) ARC 분류      — arcFits 가 통과해야 ARC op 이 된다
//   (c) 반지름 거절   — 잔차 검증을 통과한 원(ok=true)일 때만 최소 반지름으로 막는다
void testCurveRadiusOnlyFromVerifiedCircles()
{
    // 원: 보존 + ARC + 반지름 신뢰
    QList<QPointF> circle = circlePoints(0.25, 24);
    circle.append(circle.first());
    const QList<motionprogram::detail::CurveRun> circleRuns =
        motionprogram::detail::curveRuns(circle, 1.0);
    check(!circleRuns.isEmpty() && circleRuns.first().ok,
          "a real circle must be a verified circular run");
    check(!circleRuns.isEmpty() && near(circleRuns.first().radius, 0.25, 1e-6),
          "the verified circle radius must be the drawn radius");

    // 타원: 보존은 하되(ok=false) 반지름을 지어내지 않는다
    QList<QPointF> ellipse;
    for (int i = 0; i < 48; ++i) {
        const double a = 2.0 * motionprogram::kPi * i / 48;
        ellipse.append(QPointF(0.60 * std::cos(a), 0.12 * std::sin(a)));
    }
    ellipse.append(ellipse.first());
    const QList<motionprogram::detail::CurveRun> ellipseRuns =
        motionprogram::detail::curveRuns(ellipse, 1.0);
    check(!ellipseRuns.isEmpty(), "an ellipse must still be detected for preservation");
    for (const motionprogram::detail::CurveRun &r : ellipseRuns)
        check(!r.ok || near(r.radius, 0.0, 1e-9) == false,
              "an ellipse run must not be trusted unless it truly fits a circle");
    QList<bool> ellipsePaint(ellipse.size(), true);
    ellipsePaint[0] = false;
    double reported = -1.0;
    const int ellipseHit =
        motionprogram::firstTooTightPaintCurve(ellipse, ellipsePaint, &reported);
    check(ellipseHit < 0 || reported > 0.0,
          "an ellipse must never be rejected with a made-up radius");
    check(ellipseHit < 0,
          "a 600x120mm ellipse must not be rejected by the circular minimum radius");

    // 나선: 한 방향으로 계속 돌지만 원이 아니다
    QList<QPointF> spiral;
    for (int i = 0; i <= 60; ++i) {
        const double a = 2.0 * motionprogram::kPi * i / 24.0;
        const double r = 0.05 + 0.01 * i;
        spiral.append(QPointF(r * std::cos(a), r * std::sin(a)));
    }
    QList<bool> spiralPaint(spiral.size(), true);
    spiralPaint[0] = false;
    check(motionprogram::firstTooTightPaintCurve(spiral, spiralPaint) < 0,
          "a spiral must not be rejected using an unvalidated circle fit");
    check(!motionprogram::detail::curveRuns(spiral, 1.0).isEmpty(),
          "a spiral must still be preserved during simplification");

    // 직선 + 라운드 코너(R=300mm): 코너는 검증된 원호다
    QList<QPointF> rounded{ QPointF(-0.8, 0.30) };
    for (int i = 0; i <= 6; ++i) {
        const double a = motionprogram::kPi * (1.0 - 0.5 * i / 6.0);
        rounded.append(QPointF(0.30 * std::cos(a), 0.30 * std::sin(a)));
    }
    rounded.append(QPointF(0.30, -0.8));
    QList<bool> roundedPaint(rounded.size(), true);
    roundedPaint[0] = false;
    check(motionprogram::firstTooTightPaintCurve(rounded, roundedPaint) < 0,
          "a 300mm rounded corner must pass the minimum radius check");
    const QList<motionprogram::Op> roundedOps =
        motionprogram::build(rounded, roundedPaint);
    check(motionprogram::countOf(roundedOps, motionprogram::Op::Arc) == 1,
          "the rounded corner must serialize as one ARC");

    // 잡음 있는 원(±1mm): 잔차 허용(반지름의 0.5% = 1.25mm) 안이라 여전히 원이다
    QList<QPointF> noisy;
    for (int i = 0; i < 36; ++i) {
        const double a = 2.0 * motionprogram::kPi * i / 36;
        const double r = 0.25 + ((i % 2) ? 0.0008 : -0.0008);
        noisy.append(QPointF(r * std::cos(a), r * std::sin(a)));
    }
    noisy.append(noisy.first());
    const QList<motionprogram::detail::CurveRun> noisyRuns =
        motionprogram::detail::curveRuns(noisy, 1.0);
    check(!noisyRuns.isEmpty(), "a noisy circle must be preserved");
    check(!noisyRuns.isEmpty() && noisyRuns.first().ok,
          "a noisy circle inside the residual tolerance must stay a verified circle");

    // 확정된 경계값은 그대로 유지된다
    QList<QPointF> tiny = circlePoints(0.015, 24);
    tiny.append(tiny.first());
    QList<bool> tinyPaint(tiny.size(), true);
    tinyPaint[0] = false;
    double tinyR = 0.0;
    check(motionprogram::firstTooTightPaintCurve(tiny, tinyPaint, &tinyR) >= 0
          && near(tinyR, 0.015, 1e-4),
          "the confirmed R=15mm circle rejection must stay, with its real radius");
    QList<bool> okPaint(circle.size(), true);
    okPaint[0] = false;
    check(motionprogram::firstTooTightPaintCurve(circle, okPaint) < 0,
          "the confirmed R=250mm circle acceptance must stay");
}

// ── 2) 서버 채널 캘리브레이션 수용 규칙 (순수 함수 단위 테스트) ───────────────
// ⚠️ Backend/QML 통합 경로(신호 핸들러·화면 적용)는 이 바이너리에서 실행하지 않는다.
//    여기서 검증하는 것은 저장 여부를 결정하는 순수 규칙뿐이다.
void testChannelCalibAcceptanceRules()
{
    auto row = [](double a, double b, double c) {
        QJsonArray r; r.append(a); r.append(b); r.append(c); return r;
    };
    QJsonArray goodH;
    goodH.append(row(0.5, 0.0, 10.0));
    goodH.append(row(0.0, 0.5, 20.0));
    goodH.append(row(0.0, 0.0, 1.0));
    QJsonArray singular;
    singular.append(row(1, 2, 3));
    singular.append(row(2, 4, 6));
    singular.append(row(0, 0, 1));
    QJsonArray malformed;
    malformed.append(row(1, 0, 0));
    malformed.append(row(0, 1, 0));

    QJsonObject valid;   valid.insert(QStringLiteral("H"), goodH);
    QJsonObject floorOnly; floorOnly.insert(QStringLiteral("H_floor"), goodH);
    QJsonObject bad;     bad.insert(QStringLiteral("H"), malformed);
    QJsonObject sing;    sing.insert(QStringLiteral("H"), singular);

    check(camcalib::calibIsUsable(valid), "a valid H bundle must be accepted");
    check(camcalib::calibIsUsable(floorOnly), "an H_floor-only bundle must be accepted");
    check(!camcalib::calibIsUsable(bad), "a malformed H bundle must be rejected");
    check(!camcalib::calibIsUsable(sing), "a singular H bundle must be rejected");
    check(!camcalib::calibIsUsable(QJsonObject()), "an empty bundle must be rejected");

    // 수동 코너 입력(H 없음)은 기존 경로 그대로 허용한다.
    QJsonArray corners;
    for (int i = 0; i < 4; ++i) {
        QJsonObject c;
        c.insert(QStringLiteral("id"), QStringLiteral("c%1").arg(i));
        corners.append(c);
    }
    QJsonObject manual;
    manual.insert(QStringLiteral("corners"), corners);
    check(camcalib::calibIsUsable(manual),
          "the manual corner-only calibration path must keep working without H");

    QJsonObject incompleteManual;
    incompleteManual.insert(QStringLiteral("c0"),
                            QJsonObject{{QStringLiteral("id"), QStringLiteral("c0")}});
    incompleteManual.insert(QStringLiteral("c3"),
                            QJsonObject{{QStringLiteral("id"), QStringLiteral("c3")}});
    check(!camcalib::calibIsUsable(incompleteManual),
          "manual calibration must contain all four named corners");

    // CH1~CH4 혼합 맵: 유효한 채널만 남아야 하고 나머지는 독립적으로 유지된다.
    QJsonObject map;
    map.insert(QStringLiteral("1"), valid);
    map.insert(QStringLiteral("2"), bad);
    map.insert(QStringLiteral("3"), floorOnly);
    map.insert(QStringLiteral("4"), sing);
    QStringList rejected;
    const QJsonObject filtered = camcalib::filterUsableCalibMap(map, &rejected);
    check(filtered.contains(QStringLiteral("1")) && filtered.contains(QStringLiteral("3")),
          "valid channels must survive filtering");
    check(!filtered.contains(QStringLiteral("2")) && !filtered.contains(QStringLiteral("4")),
          "invalid channels must not be stored");
    check(rejected.size() == 2, "rejected channels must be reported");

    // request_id 규칙
    using camcalib::ReplyUse;
    check(camcalib::classifyHomographyReply(2, QStringLiteral("qt-2"), true, 2,
                                            QStringLiteral("qt-2")) == ReplyUse::Apply,
          "the reply we are waiting for must be applied");
    check(camcalib::classifyHomographyReply(2, QStringLiteral("qt-1"), true, 2,
                                            QStringLiteral("qt-2")) == ReplyUse::Drop,
          "a stale Qt result must not overwrite a newer calibration");
    check(camcalib::classifyHomographyReply(3, QStringLiteral("qt-2"), true, 2,
                                            QStringLiteral("qt-2")) == ReplyUse::Drop,
          "a stale Qt reply for another channel must not touch the pending channel");
    check(camcalib::classifyHomographyReply(3, QStringLiteral("adm-77"), true, 2,
                                            QStringLiteral("qt-2")) == ReplyUse::StoreOnly,
          "another channel's CCTV result must be stored without completing the pending request");
    check(camcalib::classifyHomographyReply(2, QString(), true, 2, QStringLiteral("qt-2"))
              == ReplyUse::Apply,
          "legacy empty request_id for the pending channel must still be accepted");
    check(camcalib::classifyHomographyReply(4, QString(), false, 0, QString())
              == ReplyUse::StoreOnly,
          "legacy empty request_id push without a pending request must be store-only");
    check(camcalib::classifyHomographyReply(4, QStringLiteral("adm-77"), false, 0, QString())
              == ReplyUse::StoreOnly,
          "an unsolicited channel-qualified result must be available without re-login");
    check(camcalib::classifyHomographyReply(4, QStringLiteral("qt-9"), false, 0, QString())
              == ReplyUse::Drop,
          "an unsolicited stale qt-* result must not replace stored calibration");

    // CH1~CH4 대칭: 서버 라이브 H_MATRIX(payload.ch + payload.calib)는 어느 채널이든
    // 같은 규칙을 타야 한다. 지금 보고 있는 채널이면 즉시 적용, 아니면 저장만.
    for (int ch = 1; ch <= 4; ++ch) {
        check(camcalib::appliesToCurrentView(ch, ch),
              "a live result for the displayed channel must be applied immediately");
        for (int other = 1; other <= 4; ++other)
            if (other != ch)
                check(!camcalib::appliesToCurrentView(ch, other),
                      "a live result for another channel must not touch the current TopView");
        check(!camcalib::appliesToCurrentView(ch, 0),
              "on the channel grid no live result may change the work view");
        check(camcalib::classifyHomographyReply(ch, QString(), false, 0, QString())
                  == ReplyUse::StoreOnly,
              "a live CH1..CH4 push without request_id must be stored for that channel");
        check(camcalib::resolveHomographyChannel(ch, QString(), false, 0, QString()) == ch,
              "payload.ch must select the target channel for CH1..CH4");
    }
    check(!camcalib::appliesToCurrentView(0, 0),
          "an unresolved channel must never be applied to the view");

    // 오도메트리 주행 캘리 실패 사유 문구 (서버 요청서 20260813 §2-3).
    // 모르는 reason 은 서버 msg 를 그대로 띄워야 한다 — 카메라 사유가 계속 는다.
    check(camcalib::homographyFailText(QStringLiteral("busy"), QString(),
                                      QStringLiteral("ADMIN"))
              .contains(QStringLiteral("관리자")),
          "busy+owner=ADMIN must say the admin console holds the session");
    check(camcalib::homographyFailText(QStringLiteral("busy"), QString(),
                                      QStringLiteral("QT"))
              != camcalib::homographyFailText(QStringLiteral("busy"), QString(),
                                             QStringLiteral("ADMIN")),
          "busy+owner=QT must be distinguishable from ADMIN");
    for (const char *r : { "invalid_param", "too_few_points", "fit_failed",
                           "no_intrinsics", "capture_timeout", "preempted", "not_owner" })
        check(!camcalib::homographyFailText(QLatin1String(r), QString(), QString()).isEmpty(),
              "every new odometry reason must have operator text");
    check(camcalib::homographyFailText(QStringLiteral("detect_off"),
                                      QStringLiteral("카메라 검출이 꺼져 있습니다"), QString())
              == QStringLiteral("카메라 검출이 꺼져 있습니다"),
          "an unknown reason must fall back to the server msg");

    // point_index 는 0-based 이므로 완료 정지점 = index + 1. 유효 대응점이 그보다
    // 2 이상 뒤처질 때만 경고한다 (Backend 알림과 QML 경고색이 같은 함수를 쓴다).
    check(camcalib::captureLagWarning(3, 2), "index 3 with 2 valid (4 done) must warn");
    check(!camcalib::captureLagWarning(3, 3), "index 3 with 3 valid must not warn");
    check(!camcalib::captureLagWarning(0, 0), "the first stop must not warn on its own");
    check(camcalib::captureLagWarning(1, 0), "two completed stops with none valid must warn");
    check(!camcalib::captureLagWarning(-1, -1), "missing progress values must not warn");
    check(!camcalib::captureLagWarning(-1, 0), "a missing point_index must not warn");
    check(!camcalib::captureLagWarning(5, -1), "a missing valid count must not warn");

    // 사각형 크기 검증 — 빈 칸(0)·NaN·무한은 전송되면 안 된다.
    // 서버가 받는 범위는 2cm~1000cm(양끝 포함)이다 (서버 회신 20260814).
    check(camcalib::odoSizeValidCm(2.0) && camcalib::odoSizeValidCm(90.0),
          "2cm and the 90cm default must be accepted");
    check(camcalib::odoSizeValidCm(1000.0), "the 1000cm upper limit must be inclusive");
    check(camcalib::odoSizeValidCm(999.99) && camcalib::odoSizeValidCm(2.01),
          "values just inside both limits must be accepted");
    check(!camcalib::odoSizeValidCm(1000.01) && !camcalib::odoSizeValidCm(1001.0),
          "sides above 1000cm must be rejected before CALIB_START");
    check(!camcalib::odoSizeValidCm(1.9) && !camcalib::odoSizeValidCm(0.0)
              && !camcalib::odoSizeValidCm(-5.0),
          "sides under 2cm must be rejected");
    check(!camcalib::odoSizeValidCm(std::nan("")),
          "NaN from an empty or malformed field must be rejected");
    check(!camcalib::odoSizeValidCm(std::numeric_limits<double>::infinity()),
          "an infinite side must be rejected");
    check(!camcalib::odoSizeValidCm(-std::numeric_limits<double>::infinity()),
          "a negative infinite side must be rejected");
    check(camcalib::kOdoSizeMinCm == 2.0 && camcalib::kOdoSizeMaxCm == 1000.0,
          "the documented limits must stay 2cm and 1000cm");
    // 🔴 화면에 보이는 글자 그대로가 검증·전송된다. QML validator 가 막던 시절에는
    //    "화면 1200 / 전송 90" 이 가능했다 — 그 경로를 여기서 봉인한다.
    {
        double v = -1.0;
        check(camcalib::parseOdoSizeCm(QStringLiteral("2"), &v) && v == 2.0,
              "the visible value 2 must be accepted and sent as 2");
        v = -1.0;
        check(camcalib::parseOdoSizeCm(QStringLiteral("1000"), &v) && v == 1000.0,
              "the visible value 1000 must be accepted and sent as 1000");
        v = -1.0;
        check(camcalib::parseOdoSizeCm(QStringLiteral(" 90.5 "), &v) && v == 90.5,
              "surrounding spaces must not change the sent value");
        // 실패 케이스는 값을 건드리지 않는다 — 이전 유효값으로 되돌아가면 안 된다.
        double keep = 90.0;
        check(!camcalib::parseOdoSizeCm(QStringLiteral("1200"), &keep) && keep == 90.0,
              "a visible 1200 must block the start instead of silently sending 90");
        check(!camcalib::parseOdoSizeCm(QStringLiteral("1"), &keep) && keep == 90.0,
              "a visible 1 must block the start");
        check(!camcalib::parseOdoSizeCm(QString(), &keep)
                  && !camcalib::parseOdoSizeCm(QStringLiteral("   "), &keep),
              "an empty field must block the start");
        check(!camcalib::parseOdoSizeCm(QStringLiteral("abc"), &keep)
                  && !camcalib::parseOdoSizeCm(QStringLiteral("90cm"), &keep)
                  && !camcalib::parseOdoSizeCm(QStringLiteral("9,0"), &keep),
              "non-numeric or partially numeric text must block the start");
        check(!camcalib::parseOdoSizeCm(QStringLiteral("nan"), &keep)
                  && !camcalib::parseOdoSizeCm(QStringLiteral("inf"), &keep)
                  && !camcalib::parseOdoSizeCm(QStringLiteral("-inf"), &keep),
              "NaN and infinite text must block the start");
        check(!camcalib::parseOdoSizeCm(QStringLiteral("1000.01"), &keep)
                  && !camcalib::parseOdoSizeCm(QStringLiteral("1.99"), &keep),
              "text just outside the inclusive range must block the start");
        check(camcalib::odoSizeRangeText().contains(QStringLiteral("2cm"))
                  && camcalib::odoSizeRangeText().contains(QStringLiteral("1000cm")),
              "the field-level range message must state both limits");
    }

    // 조작자 문구는 두 한계를 모두 말해야 한다 (서버 invalid_param 회신 포함).
    {
        const QString text = camcalib::homographyFailText(QStringLiteral("invalid_param"),
                                                          QString(), QString());
        check(text.contains(QStringLiteral("2cm")) && text.contains(QStringLiteral("1000cm")),
              "invalid_param text must state both the 2cm and 1000cm limits");
    }

    // 회전 방향 매핑 — 서버가 CALIB_START 원본을 그대로 중계하므로 값이 정본이다.
    check(camcalib::odoStartCorner(true) == QStringLiteral("bottom_left"),
          "CCW must map to bottom_left");
    check(camcalib::odoStartCorner(false) == QStringLiteral("top_left"),
          "CW must map to top_left");

    // 중단을 요청한 뒤 도착한 결과는 성공 완료가 아니며 저장하지 않고 폐기한다.
    // (확인 대기 중이든, watchdog 만료로 확인을 못 받았든 규칙은 같다)
    check(camcalib::afterCancelRequest(ReplyUse::Apply, true) == ReplyUse::Drop,
          "a matching result after a cancel request must be dropped, not stored");
    check(camcalib::afterCancelRequest(ReplyUse::Apply, false) == ReplyUse::Apply,
          "without a cancel request the pending result must still be applied");
    check(camcalib::afterCancelRequest(ReplyUse::StoreOnly, true) == ReplyUse::StoreOnly
              && camcalib::afterCancelRequest(ReplyUse::Drop, true) == ReplyUse::Drop,
          "cancellation must not upgrade store-only or stale replies");
    // 외부 채널 확정 결과는 Qt 취소와 무관하게 규칙이 그대로다.
    for (int ch = 1; ch <= 4; ++ch)
        check(camcalib::afterCancelRequest(
                  camcalib::classifyHomographyReply(ch, QStringLiteral("adm-77"), false, 0,
                                                    QString()), false)
                  == ReplyUse::StoreOnly,
              "external CH1..CH4 results must keep store-only routing");
    // 낡은 qt-* 결과는 취소 여부와 상관없이 계속 버려진다.
    check(camcalib::afterCancelRequest(
              camcalib::classifyHomographyReply(2, QStringLiteral("qt-1"), true, 2,
                                                QStringLiteral("qt-2")), true)
              == ReplyUse::Drop,
          "a stale qt-* reply must stay dropped during cancellation");
    // 취소된 세션의 결과는 어느 채널이든 저장조차 되지 않는다 (CH1~CH4 대칭).
    for (int ch = 1; ch <= 4; ++ch)
        check(camcalib::afterCancelRequest(
                  camcalib::classifyHomographyReply(ch, QStringLiteral("qt-7"), true, ch,
                                                    QStringLiteral("qt-7")), true)
                  == ReplyUse::Drop,
              "a cancelled session result must never be stored for CH1..CH4");
    check(camcalib::classifyHomographyReply(2, QStringLiteral("adm-77"), true, 2,
                                            QStringLiteral("qt-2")) == ReplyUse::StoreOnly,
          "a foreign result on the pending channel must not complete the Qt request");
    for (int ch = 1; ch <= 4; ++ch)
        check(camcalib::classifyHomographyReply(ch, QStringLiteral("adm-77"), false, 0,
                                                QString()) == ReplyUse::StoreOnly,
              "CH1..CH4 async results must all be stored");

    check(camcalib::classifyHomographyReply(4, QStringLiteral("qt-old"), false, 0, QString())
              == ReplyUse::Drop,
          "an unsolicited stale Qt result must be dropped");
}

// ── 3) coord_mode="undistort" 인데 K/D 가 없는 번들 (서버 회신 20260814 §2) ────
// ⚠️ 순수 규칙만 검증한다. Backend 의 채널별 배너 표시/해제는 이 바이너리에서
//    실행하지 않지만, 그 판정과 통지 키 규칙은 아래 두 함수로 결정된다.
void testUndistortLensDataWarning()
{
    auto k3x3 = []() {
        QJsonArray K, r0, r1, r2;
        r0.append(1200.0); r0.append(0.0);    r0.append(1296.0);
        r1.append(0.0);    r1.append(1200.0); r1.append(760.0);
        r2.append(0.0);    r2.append(0.0);    r2.append(1.0);
        K.append(r0); K.append(r1); K.append(r2);
        return K;
    };
    auto d5 = []() {
        QJsonArray d;
        for (double v : { -0.32, 0.11, 0.0, 0.0, -0.02 }) d.append(v);
        return d;
    };

    QJsonObject complete;
    complete.insert(QStringLiteral("coord_mode"), QStringLiteral("undistort"));
    complete.insert(QStringLiteral("K"), k3x3());
    complete.insert(QStringLiteral("D"), d5());
    check(!camcalib::lensDataMissingForUndistort(complete),
          "a complete undistort bundle must not warn");

    QJsonObject noD = complete;
    noD.remove(QStringLiteral("D"));
    check(camcalib::lensDataMissingForUndistort(noD),
          "an undistort bundle without D must warn");

    QJsonObject noK = complete;
    noK.remove(QStringLiteral("K"));
    check(camcalib::lensDataMissingForUndistort(noK),
          "an undistort bundle without K must warn");

    QJsonObject shortD = complete;
    shortD.insert(QStringLiteral("D"), QJsonArray{ 0.1, 0.2, 0.3 });
    check(camcalib::lensDataMissingForUndistort(shortD),
          "a distortion vector that is neither 4 nor 5 long is unusable");

    QJsonObject badK = complete;
    badK.insert(QStringLiteral("K"), QJsonObject{{QStringLiteral("fx"), 0.0},
                                                {QStringLiteral("fy"), 0.0},
                                                {QStringLiteral("cx"), 10.0},
                                                {QStringLiteral("cy"), 10.0}});
    check(camcalib::lensDataMissingForUndistort(badK),
          "an unusable K (fx/fy <= 1) must warn like a missing K");

    QJsonObject objK = complete;
    objK.insert(QStringLiteral("K"), QJsonObject{{QStringLiteral("fx"), 1200.0},
                                                 {QStringLiteral("fy"), 1200.0},
                                                 {QStringLiteral("cx"), 1296.0},
                                                 {QStringLiteral("cy"), 760.0}});
    check(!camcalib::lensDataMissingForUndistort(objK),
          "the {fx,fy,cx,cy} K form must count as usable");

    QJsonObject distAlias = complete;
    distAlias.remove(QStringLiteral("D"));
    distAlias.insert(QStringLiteral("dist"), QJsonArray{ -0.3, 0.1, 0.0, 0.0 });
    check(!camcalib::lensDataMissingForUndistort(distAlias),
          "the legacy 4-coefficient \"dist\" field must count as usable");

    // 표기 흔들림: {"calib":{...}} 포장과 camelCase coordMode 도 같은 판정을 받아야 한다.
    QJsonObject wrapped;
    wrapped.insert(QStringLiteral("calib"), noD);
    check(camcalib::lensDataMissingForUndistort(wrapped),
          "a wrapped {calib:{...}} bundle must be classified the same way");
    QJsonObject camel = noD;
    camel.remove(QStringLiteral("coord_mode"));
    camel.insert(QStringLiteral("coordMode"), QStringLiteral("UnDistort"));
    check(camcalib::lensDataMissingForUndistort(camel),
          "camelCase coordMode must be recognised case-insensitively");

    // raw / 미선언 번들은 이 경고 대상이 아니다 — 그쪽은 왜곡 보정을 쓰지 않는다.
    QJsonObject rawMode = noD;
    rawMode.insert(QStringLiteral("coord_mode"), QStringLiteral("raw"));
    check(!camcalib::lensDataMissingForUndistort(rawMode),
          "a raw-space bundle must not raise the distortion-data warning");
    QJsonObject silent = noD;
    silent.remove(QStringLiteral("coord_mode"));
    check(!camcalib::lensDataMissingForUndistort(silent),
          "a bundle without coord_mode must not raise the warning");
    check(!camcalib::lensDataMissingForUndistort(QJsonObject()),
          "an empty bundle must not raise the warning");

    // 경고가 있어도 번들 수용 규칙은 예전 그대로여야 한다 (H 만 보면 된다).
    QJsonArray goodH, hr0, hr1, hr2;
    hr0.append(0.5); hr0.append(0.0); hr0.append(10.0);
    hr1.append(0.0); hr1.append(0.5); hr1.append(20.0);
    hr2.append(0.0); hr2.append(0.0); hr2.append(1.0);
    goodH.append(hr0); goodH.append(hr1); goodH.append(hr2);
    QJsonObject acceptStill = noD;
    acceptStill.insert(QStringLiteral("H"), goodH);
    check(camcalib::calibIsUsable(acceptStill),
          "a bundle missing K/D must still be accepted when H is valid");

    // 채널 독립: 통지 키가 채널마다 달라야 다른 채널 경고를 지우지 않는다.
    for (int ch = 1; ch <= 4; ++ch) {
        const QString key = camcalib::lensDataNoticeKey(ch);
        check(key.contains(QStringLiteral("CH%1").arg(ch)),
              "each channel must own its distortion-warning notice key");
        for (int other = 1; other <= 4; ++other)
            if (other != ch)
                check(key != camcalib::lensDataNoticeKey(other),
                      "one channel's warning must never clear another channel's");
    }

    // CH1~CH4 지속 표시 모델: Backend 가 유지하는 "경고 채널 집합"과 같은 규칙으로
    // 한 채널만 켜지고, 그 채널에 완전한 번들이 오면 그 채널만 꺼져야 한다.
    {
        QSet<int> warned;
        auto feed = [&warned](int ch, const QJsonObject &bundle) {
            if (camcalib::lensDataMissingForUndistort(bundle)) warned.insert(ch);
            else warned.remove(ch);
        };
        for (int ch = 1; ch <= 4; ++ch) feed(ch, complete);
        check(warned.isEmpty(), "complete bundles must leave no channel flagged");

        feed(2, noD);
        check(warned.contains(2) && warned.size() == 1,
              "only the offending channel may be flagged");
        feed(4, noK);
        check(warned.contains(2) && warned.contains(4) && warned.size() == 2,
              "each channel must keep its own K/D state");
        feed(2, complete);
        check(!warned.contains(2) && warned.contains(4),
              "a later complete bundle must clear only that channel's indicator");
        feed(1, rawMode);
        feed(3, silent);
        check(!warned.contains(1) && !warned.contains(3) && warned.contains(4),
              "raw or undeclared bundles must not flag other channels");
        feed(4, complete);
        check(warned.isEmpty(), "the last channel must clear once its K/D arrives");
    }
}

void testMissingChannelResolution()
{
    using camcalib::resolveHomographyChannel;
    for (int ch = 1; ch <= 4; ++ch) {
        check(resolveHomographyChannel(ch, QString(), false, 0, QString()) == ch,
              "an explicit CH1..CH4 channel must be preserved");
        check(resolveHomographyChannel(ch, QStringLiteral("qt-1"), true, 2,
                                       QStringLiteral("qt-1")) == ch,
              "an explicit channel must never be replaced by the pending channel");
        check(resolveHomographyChannel(0, QString(), true, ch, QStringLiteral("qt-1")) == ch,
              "a missing channel must resolve to the only pending channel");
        check(resolveHomographyChannel(0, QStringLiteral("qt-1"), true, ch,
                                       QStringLiteral("qt-1")) == ch,
              "a missing channel with our request id must resolve to the pending channel");
    }
    check(resolveHomographyChannel(3, QString(), false, 0, QString()) == 3,
          "an explicit channel must be preserved");
    check(resolveHomographyChannel(0, QStringLiteral("qt-2"), true, 2,
                                   QStringLiteral("qt-2")) == 2,
          "a missing channel may use the single pending Qt request");
    check(resolveHomographyChannel(0, QString(), false, 0, QString()) == 0,
          "a missing channel without a pending request must not be guessed");
    check(resolveHomographyChannel(0, QStringLiteral("adm-77"), false, 0, QString()) == 0,
          "a foreign result without ch and pending request must remain unresolved");
    check(resolveHomographyChannel(0, QStringLiteral("adm-77"), true, 2,
                                   QStringLiteral("qt-2")) == 0,
          "a foreign request id without ch must not use the Qt pending channel");
    check(resolveHomographyChannel(5, QString(), true, 2, QStringLiteral("qt-2")) == 0,
          "an out-of-range channel must be rejected");
    check(resolveHomographyChannel(-1, QString(), true, 2, QStringLiteral("qt-2")) == 0,
          "a negative channel must be rejected");
    check(camcalib::classifyHomographyReply(
              resolveHomographyChannel(0, QString(), true, 3, QStringLiteral("qt-3")),
              QString(), true, 3, QStringLiteral("qt-3")) == camcalib::ReplyUse::Apply,
          "a ch-less legacy bundle must complete the pending CH3 request");
}

} // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    testCircleOneArc();
    testPartialArcEntryHeading();
    testServerRadiusPreflight();
    testArcResizeConstraint();
    testEllipseIsNotOneCircle();
    testUniformResizeRoundTripKeepsCircle();
    testSharedServerQuarterArcFixture();
    testFinishedRectangleUsesTipCenterPath();
    testFinished180mmSquareSerializes130mmCenterMoves();
    testFinishedCircleRemainsOneArc();
    testFinishedPartialArcPreservesSweep();
    testOuterArcMinimumConstraint();
    testInvalidFinishedContoursAreRejected();
    testCircleSampleDensityKeepsOneArc();
    testPartialArcsBothDirections();
    testLinePlusSemicirclePreservesArc();
    testSimplifyKeepsCurvesAndStraightBehaviour();
    testStraightShapesRegression();
    testNearbyShapesKeepTheirOwnGeometry();
    testNibWidthChangesCenterlineOnly();
    testPenDistanceDoesNotChangeTransmittedPath();
    testTightCurveWithoutArcOpIsStillRejected();
    testArcFitUnitsAreExplicit();
    testStrokeFontRoundGlyphKeepsCurve();
    testHomographyValidation();
    testCurveRadiusOnlyFromVerifiedCircles();
    testChannelCalibAcceptanceRules();
    testUndistortLensDataWarning();
    testMissingChannelResolution();
    if (failures == 0) {
        std::cout << "motionprogram_tests: PASS\n";
        return 0;
    }
    std::cerr << "motionprogram_tests: " << failures << " failure(s)\n";
    return 1;
}
