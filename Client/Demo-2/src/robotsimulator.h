#ifndef ROBOTSIMULATOR_H
#define ROBOTSIMULATOR_H

#include <QElapsedTimer>
#include <QObject>
#include <QPointF>
#include <QTimer>
#include <QVariantList>
#include <QVector>
#include <QtQml/qqmlregistration.h>

// 로봇 이동 시뮬레이터.
// 대기 지점(dock)에서 경로 시작점까지 펜 업 상태로 이동한 뒤,
// 펜 다운으로 경로를 따라가며 trailSegment 시그널로 도장 궤적을 내보낸다.
class RobotSimulator : public QObject
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(qreal x READ x NOTIFY poseChanged)
    Q_PROPERTY(qreal y READ y NOTIFY poseChanged)
    Q_PROPERTY(qreal heading READ heading NOTIFY poseChanged)
    Q_PROPERTY(bool running READ running NOTIFY stateChanged)
    Q_PROPERTY(bool penDown READ penDown NOTIFY stateChanged)
    Q_PROPERTY(QString state READ stateName NOTIFY stateChanged)
    Q_PROPERTY(QString stateText READ stateText NOTIFY stateChanged)
    Q_PROPERTY(qreal progress READ progress NOTIFY progressChanged)
    Q_PROPERTY(qreal speed READ speed WRITE setSpeed NOTIFY speedChanged)

public:
    explicit RobotSimulator(QObject *parent = nullptr);

    qreal x() const { return m_pos.x(); }
    qreal y() const { return m_pos.y(); }
    qreal heading() const { return m_heading; }
    bool running() const { return m_timer.isActive(); }
    bool penDown() const { return m_penDown; }
    QString stateName() const;
    QString stateText() const;
    qreal progress() const { return m_progress; }
    qreal speed() const { return m_speed; }
    void setSpeed(qreal s);

    Q_INVOKABLE bool setPath(const QVariantList &points, bool closed);
    Q_INVOKABLE bool start();
    Q_INVOKABLE void pause();
    Q_INVOKABLE void reset();

signals:
    void poseChanged();
    void stateChanged();
    void progressChanged();
    void speedChanged();
    void trailSegment(qreal x1, qreal y1, qreal x2, qreal y2);
    void trailCleared();
    void finished();

private:
    enum class Phase { Idle, Transit, Painting, Done };

    void tick();
    void setPose(const QPointF &pos, qreal headingDeg);
    void setPhase(Phase phase, bool penDown);
    void setProgress(qreal p);
    qreal totalPathLength() const;

    QVector<QPointF> m_path;      // 폐곡선이면 시작점이 끝에 복제되어 있음
    Phase m_phase = Phase::Idle;
    QPointF m_pos;
    qreal m_heading = 0;
    bool m_penDown = false;
    qreal m_progress = 0;
    qreal m_speed = 120;          // px/s
    int m_segIndex = 0;
    qreal m_segPos = 0;
    qreal m_paintedLength = 0;
    QTimer m_timer;
    QElapsedTimer m_clock;
};

#endif // ROBOTSIMULATOR_H
