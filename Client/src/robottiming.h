#pragma once

#include <QList>

#include <algorithm>
#include <cmath>

#include "motionprogram.h"

namespace robottiming {

// Display-only lower-bound estimate based on the confirmed robot specification.
// Feedback-dependent corrections and communication waits are intentionally excluded.
constexpr double kMoveSpeedMps = 0.050;
constexpr double kMoveRampDistanceRatio = 0.20;
constexpr double kMoveRampEndSpeedRatio = 0.20;
constexpr double kTurnFastDps = 16.5;
constexpr double kTurnSlowDps = 7.4;
constexpr double kTurnSlowBandDeg = 4.0;
constexpr double kNozzleSettleSec = 2.5;
constexpr double kNozzleOffsetM = 0.155;

inline double moveSeconds(double distanceM, double speedMps = kMoveSpeedMps)
{
    const double distance = std::abs(distanceM);
    if (!(distance > 0.0) || !(speedMps > 1e-9)) return 0.0;

    // Approximate the final 20% ramp as speed changing linearly with distance.
    const double rampFactor = kMoveRampDistanceRatio
                            * std::log(1.0 / kMoveRampEndSpeedRatio)
                            / (1.0 - kMoveRampEndSpeedRatio);
    return distance / speedMps * ((1.0 - kMoveRampDistanceRatio) + rampFactor);
}

inline double turnSeconds(double angleDeg)
{
    const double angle = std::abs(angleDeg);
    if (!(angle > 0.0)) return 0.0;

    const double slowAngle = std::min(angle, kTurnSlowBandDeg);
    return (angle - slowAngle) / kTurnFastDps + slowAngle / kTurnSlowDps;
}

inline double arcSeconds(double drawingRadiusM, double sweepDeg,
                         double speedMps = kMoveSpeedMps)
{
    const double radius = std::abs(drawingRadiusM);
    const double sweep = std::abs(sweepDeg);
    if (!(sweep > 0.0) || !(speedMps > 1e-9)) return 0.0;

    // The current Robot implementation drives the wheel-center arc at 0.05 m/s.
    const double robotRadius = std::sqrt(radius * radius
                                       + kNozzleOffsetM * kNozzleOffsetM);
    return robotRadius * sweep * motionprogram::kDegToRad / speedMps;
}

inline double estimatedSeconds(const QList<motionprogram::Op> &ops)
{
    double seconds = 0.0;
    for (const motionprogram::Op &op : ops) {
        switch (op.kind) {
        case motionprogram::Op::Move:
            seconds += moveSeconds(op.dist);
            break;
        case motionprogram::Op::Turn:
            seconds += turnSeconds(op.angle);
            break;
        case motionprogram::Op::Arc:
            seconds += arcSeconds(op.radius, op.angle);
            break;
        case motionprogram::Op::Nozzle:
            seconds += kNozzleSettleSec;
            break;
        }
    }
    return seconds;
}

} // namespace robottiming
