#include "motionprogram.h"
#include "routeplan.h"

#include <QCoreApplication>
#include <QLineF>
#include <QList>
#include <QPointF>

#include <cmath>
#include <iostream>

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

} // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    testCircleOneArc();
    testPartialArcEntryHeading();
    testServerRadiusPreflight();
    testEllipseIsNotOneCircle();
    testUniformResizeRoundTripKeepsCircle();
    testSharedServerQuarterArcFixture();
    if (failures == 0) {
        std::cout << "motionprogram_tests: PASS\n";
        return 0;
    }
    std::cerr << "motionprogram_tests: " << failures << " failure(s)\n";
    return 1;
}
