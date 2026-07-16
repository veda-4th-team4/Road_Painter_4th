#include "robotsimulator.h"

#include <QLineF>
#include <QVariantMap>
#include <QtMath>

#include <algorithm>

namespace {
const QPointF kDockPosition(90, 640);
} // namespace

RobotSimulator::RobotSimulator(QObject *parent)
    : QObject(parent)
    , m_pos(kDockPosition)
{
    m_timer.setInterval(16);
    connect(&m_timer, &QTimer::timeout, this, &RobotSimulator::tick);
}

QString RobotSimulator::stateName() const
{
    if (!m_timer.isActive() && (m_phase == Phase::Transit || m_phase == Phase::Painting))
        return QStringLiteral("paused");
    switch (m_phase) {
    case Phase::Idle: return QStringLiteral("idle");
    case Phase::Transit: return QStringLiteral("transit");
    case Phase::Painting: return QStringLiteral("painting");
    case Phase::Done: return QStringLiteral("done");
    }
    return QStringLiteral("idle");
}

QString RobotSimulator::stateText() const
{
    const QString s = stateName();
    if (s == QStringLiteral("paused")) return QStringLiteral("일시정지");
    if (s == QStringLiteral("transit")) return QStringLiteral("시작점 이동 중 (펜 업)");
    if (s == QStringLiteral("painting")) return QStringLiteral("도장 중 (펜 다운)");
    if (s == QStringLiteral("done")) return QStringLiteral("작업 완료");
    return QStringLiteral("대기 중");
}

void RobotSimulator::setSpeed(qreal s)
{
    s = std::clamp(s, 10.0, 500.0);
    if (qFuzzyCompare(m_speed, s))
        return;
    m_speed = s;
    emit speedChanged();
}

bool RobotSimulator::setPath(const QVariantList &points, bool closed)
{
    if (m_timer.isActive())
        return false; // 주행 중 경로 교체는 무시 (Fail-Safe)

    m_path.clear();
    m_path.reserve(points.size() + 1);
    for (const QVariant &v : points) {
        const QVariantMap m = v.toMap();
        m_path.append(QPointF(m.value(QStringLiteral("x")).toDouble(),
                              m.value(QStringLiteral("y")).toDouble()));
    }
    if (closed && m_path.size() > 2)
        m_path.append(m_path.first());

    m_phase = Phase::Idle;
    m_segIndex = 0;
    m_segPos = 0;
    m_paintedLength = 0;
    m_penDown = false;
    setProgress(0);
    emit stateChanged();
    return m_path.size() >= 2;
}

qreal RobotSimulator::totalPathLength() const
{
    qreal len = 0;
    for (int i = 1; i < m_path.size(); ++i)
        len += QLineF(m_path[i - 1], m_path[i]).length();
    return len;
}

bool RobotSimulator::start()
{
    if (m_timer.isActive())
        return true;
    if (m_path.size() < 2)
        return false;

    if (m_phase == Phase::Idle || m_phase == Phase::Done) {
        m_segIndex = 0;
        m_segPos = 0;
        m_paintedLength = 0;
        m_penDown = false;
        setProgress(0);
        m_phase = Phase::Transit;
    }
    m_clock.restart();
    m_timer.start();
    emit stateChanged();
    return true;
}

void RobotSimulator::pause()
{
    if (!m_timer.isActive())
        return;
    m_timer.stop();
    emit stateChanged();
}

void RobotSimulator::reset()
{
    m_timer.stop();
    m_phase = Phase::Idle;
    m_penDown = false;
    m_segIndex = 0;
    m_segPos = 0;
    m_paintedLength = 0;
    setPose(kDockPosition, 0);
    setProgress(0);
    emit trailCleared();
    emit stateChanged();
}

void RobotSimulator::setPose(const QPointF &pos, qreal headingDeg)
{
    if (m_pos == pos && qFuzzyCompare(m_heading, headingDeg))
        return;
    m_pos = pos;
    m_heading = headingDeg;
    emit poseChanged();
}

void RobotSimulator::setPhase(Phase phase, bool penDown)
{
    if (m_phase == phase && m_penDown == penDown)
        return;
    m_phase = phase;
    m_penDown = penDown;
    emit stateChanged();
}

void RobotSimulator::setProgress(qreal p)
{
    p = std::clamp(p, 0.0, 1.0);
    if (qFuzzyCompare(m_progress + 1.0, p + 1.0))
        return;
    m_progress = p;
    emit progressChanged();
}

void RobotSimulator::tick()
{
    const qreal dt = std::min(m_clock.restart() / 1000.0, 0.1);
    qreal budget = m_speed * dt;

    if (m_phase == Phase::Transit) {
        const QPointF target = m_path.first();
        const QLineF line(m_pos, target);
        const qreal dist = line.length();
        if (dist <= budget) {
            setPose(target, m_heading);
            setPhase(Phase::Painting, true);
            budget -= dist;
        } else {
            const qreal headingDeg = qRadiansToDegrees(qAtan2(target.y() - m_pos.y(),
                                                              target.x() - m_pos.x()));
            setPose(m_pos + (target - m_pos) * (budget / dist), headingDeg);
            return;
        }
    }

    if (m_phase != Phase::Painting)
        return;

    const int segCount = static_cast<int>(m_path.size()) - 1;
    QPointF pos = m_pos;
    qreal headingDeg = m_heading;

    while (budget > 1e-9 && m_segIndex < segCount) {
        const QPointF a = m_path[m_segIndex];
        const QPointF b = m_path[m_segIndex + 1];
        const qreal segLen = QLineF(a, b).length();
        if (segLen < 1e-9) {
            ++m_segIndex;
            m_segPos = 0;
            continue;
        }
        headingDeg = qRadiansToDegrees(qAtan2(b.y() - a.y(), b.x() - a.x()));
        const qreal remaining = segLen - m_segPos;
        const qreal step = std::min(budget, remaining);
        const qreal t = (m_segPos + step) / segLen;
        const QPointF newPos = a + (b - a) * t;

        emit trailSegment(pos.x(), pos.y(), newPos.x(), newPos.y());
        pos = newPos;
        m_segPos += step;
        m_paintedLength += step;
        budget -= step;

        if (m_segPos >= segLen - 1e-9) {
            ++m_segIndex;
            m_segPos = 0;
        }
    }

    setPose(pos, headingDeg);
    const qreal total = totalPathLength();
    setProgress(total > 0 ? m_paintedLength / total : 1.0);

    if (m_segIndex >= segCount) {
        m_timer.stop();
        setPhase(Phase::Done, false);
        setProgress(1.0);
        emit finished();
    }
}
