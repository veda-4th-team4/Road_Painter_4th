#include "backend.h"
#include "videoview.h"
#include "video_worker.h"
#include "serverclient.h"
#include "routeplan.h"

#include <QTimer>
#include <QPointF>
#include <QLineF>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QJsonArray>
#include <QtMath>
#include <cmath>
#include <utility>
#include <QStandardPaths>
#include <QRegularExpression>
#include <QPainter>
#include <QImage>
#include <QDir>
#include <QFile>
#include <QTextStream>
#include <QUuid>
#include <QSettings>

namespace {
constexpr qint64 kRobotOnlineMs = 3000;
// 작업 시작 후 이 시간까지 POSE 가 한 번도 없으면 한 번만 경고를 남긴다.
constexpr qint64 kPoseWaitWarnMs = 5000;
// ArUco 마커(ID 49) 중심 ↔ 노즐(펜) 거리. 로봇팀 제원 2026-07-30.
// **바닥 평면에서의 수평거리**다 — POSE 가 이미 바닥 투영 좌표이므로 같은 평면에서 뺀다.
// (마커는 로봇 높이 140mm 위에 있지만, 그 시차 보정은 **서버**가 하고 QT 로는 이미
//  바닥에 투영된 값이 온다. 여기서 높이를 다시 계산하면 이중 보정이 된다.)
// ⚠️ **표시 전용이다.** 전송 좌표에는 절대 반영하지 않는다 — 오프셋 보정은 로봇 RPi 몫.
constexpr double kPenOffsetM = 0.155;
// 테스트 시뮬레이션 틱. 40ms = 25fps — 4cm 후진처럼 짧은 동작도 여러 프레임에 걸친다.
constexpr int kSimTickMs = 40;

double polylineLength(const QList<QPointF> &pts, bool closed)
{
    if (pts.size() < 2) return 0.0;
    double L = 0.0;
    for (int i = 1; i < pts.size(); ++i)
        L += QLineF(pts[i - 1], pts[i]).length();
    if (closed && pts.size() > 2)
        L += QLineF(pts.last(), pts.first()).length();
    return L;
}

double pathsLength(const QList<QList<QPointF>> &paths, const QList<bool> &closed)
{
    double L = 0.0;
    for (int i = 0; i < paths.size(); ++i)
        L += polylineLength(paths[i], closed.value(i, false));
    return L;
}

int pathsPointCount(const QList<QList<QPointF>> &paths)
{
    int n = 0;
    for (const auto &p : paths) n += p.size();
    return n;
}

// Closest projection cumulative distance along path (0..L)
double distanceAlongPolyline(const QList<QPointF> &pts, bool closed, const QPointF &pose)
{
    if (pts.size() < 2) return 0.0;

    QList<QPointF> segs = pts;
    if (closed && pts.size() > 2)
        segs.append(pts.first());

    double bestDist2 = 1e300;
    double bestAlong = 0.0;
    double walked = 0.0;

    for (int i = 1; i < segs.size(); ++i) {
        const QPointF a = segs[i - 1];
        const QPointF b = segs[i];
        const QPointF ab = b - a;
        const double len2 = QPointF::dotProduct(ab, ab);
        double t = 0.0;
        if (len2 > 1e-12) {
            t = QPointF::dotProduct(pose - a, ab) / len2;
            t = qBound(0.0, t, 1.0);
        }
        const QPointF proj = a + ab * t;
        const QPointF d = pose - proj;
        const double d2 = QPointF::dotProduct(d, d);
        if (d2 < bestDist2) {
            bestDist2 = d2;
            bestAlong = walked + qSqrt(len2) * t;
        }
        walked += qSqrt(len2);
    }
    return bestAlong;
}
} // namespace

Backend::Backend(QObject *parent)
    : QObject(parent)
{
    m_client = new ServerClient(this);
    m_lastRobotBeat.invalidate();
    loadSettings();

    connect(m_client, &ServerClient::statusReceived, this, &Backend::onStatus);
    connect(m_client, &ServerClient::poseReceived, this, &Backend::onPose);
    connect(m_client, &ServerClient::connectedToServer, this, [this]() {
        m_serverConnected = true;
        m_serverLabel = m_testMode ? "테스트" : "연결됨";
        emit linkStatusChanged();
        appendLog("서버 TLS 연결");
        // 로봇은 기본 속도로 부팅한다 → 접속하자마자 설정값을 밀어 넣는다.
        // (안 하면 수동 조작이 설정과 다른 속도로 움직인다)
        pushSpeeds();
        updatePhase();
    });
    connect(m_client, &ServerClient::disconnectedFromServer, this, [this]() {
        m_serverConnected = false;
        if (!m_testMode)
            m_serverLabel = "끊김";
        emit linkStatusChanged();
        appendLog("서버 연결 종료");
        updatePhase();
    });
    // 도면 접수 확인 — 보낸 개수와 서버가 받은 개수를 그 자리에서 대조한다.
    // paint/program 이 형식 오류로 무시되면 여기서 false/0 으로 드러난다.
    connect(m_client, &ServerClient::blueprintAck, this,
            [this](int points, bool paint, int program) {
        const int sentPts = m_routePts.size();
        const int sentOps = m_program.size();
        const bool match = (points == sentPts) && (program == sentOps)
                           && (paint == !m_routePaint.isEmpty());
        appendLog(QStringLiteral("BLUEPRINT_OK — 서버 수신 점 %1 · paint %2 · 동작 %3%4")
                      .arg(points)
                      .arg(paint ? QStringLiteral("O") : QStringLiteral("X"))
                      .arg(program)
                      .arg(match ? QString()
                                 : QStringLiteral("  ⚠️ 보낸 값(점 %1 · 동작 %2)과 다름")
                                       .arg(sentPts).arg(sentOps)));
        if (!match)
            setNotice(QStringLiteral("서버가 받은 도면이 보낸 것과 다릅니다. 시스템 로그를 확인하세요."),
                      QStringLiteral("warn"), QStringLiteral("blueprint"));
        else
            clearNotice(QStringLiteral("blueprint"));
    });
    connect(m_client, &ServerClient::hMatrixReceived, this, [this](const QJsonObject &calib) {
        if (m_testMode) return;
        if (calib.isEmpty()) {
            appendLog("H_MATRIX 수신 — calib=null (서버에 보관된 번들 없음)");
            return;
        }
        // 수동 입력과 완전히 같은 경로. K/D 반영과 coord_mode 판정을 건너뛰지 않는다.
        appendLog("H_MATRIX 수신 — " + applyCalibObject(calib, QStringLiteral("H_MATRIX")));
    });
    // 도색 완료 — START_DRAW 이후 서버가 주는 유일한 "끝" 신호
    connect(m_client, &ServerClient::drawDone, this, [this]() {
        if (!m_jobActive) {
            appendLog("DRAW_DONE 수신 (진행 중인 작업 없음) — 무시");
            return;
        }
        setNotice(QStringLiteral("도색이 완료되었습니다."), QStringLiteral("info"));
        finishJob(QStringLiteral("DRAW_DONE — 도색 완료"));
    });
    connect(m_client, &ServerClient::drawFailed, this,
            [this](const QString &stage, const QString &reason, const QString &msg) {
        const QString text = msg.isEmpty()
            ? QStringLiteral("경로 처리 실패 (%1/%2)").arg(stage, reason)
            : msg;
        appendLog(QStringLiteral("DRAW_FAIL %1/%2 — %3").arg(stage, reason, text));

        // no_pose 는 실패가 아니다 — CCTV 가 로봇을 잡으면 서버가 알아서 시작한다.
        // "그리는 중" 상태를 그대로 유지해야 한다 (재전송 금지).
        if (reason == QLatin1String("no_pose")) {
            setNotice(text.isEmpty()
                          ? QStringLiteral("로봇 위치 확인 중… 잡히면 자동으로 시작합니다.")
                          : text,
                      QStringLiteral("warn"));
            return;
        }
        // busy = 이미 실행 중인데 또 눌렀다. 진행 상태는 유지.
        if (reason == QLatin1String("busy")) {
            setNotice(QStringLiteral("이미 작업이 진행 중입니다."), QStringLiteral("warn"));
            return;
        }

        setNotice(text, QStringLiteral("error"));
        // 나머지(bad_points / no_blueprint / robot_offline / not_ready)는 실행이 서지 않은
        // 것이므로 "그리는 중"을 풀고 다시 손볼 수 있게 되돌린다.
        m_jobActive = false;
        m_paintingSeen = false;
        m_jobElapsedValid = false;
        if (stage == QLatin1String("plan") || reason == QLatin1String("no_blueprint"))
            m_blueprintSent = false;   // 도면부터 다시 보내야 한다
        emit jobChanged();
        updatePhase();
    });
    connect(m_client, &ServerClient::camIpResult, this,
            [this](bool ok, const QString &camIp, const QString &reason) {
        if (ok) {
            appendLog(QStringLiteral("SET_CAM_IP_OK — 카메라 IP %1")
                          .arg(camIp.isEmpty() ? QStringLiteral("(등록 해제)") : camIp));
            setNotice(camIp.isEmpty()
                          ? QStringLiteral("카메라 IP 등록을 해제했습니다.")
                          : QStringLiteral("카메라 IP를 %1 로 저장했습니다.").arg(camIp),
                      QStringLiteral("info"));
        } else {
            appendLog(QStringLiteral("SET_CAM_IP_FAIL — %1").arg(reason));
            setNotice(QStringLiteral("카메라 IP 저장 실패: %1").arg(reason),
                      QStringLiteral("error"));
        }
    });
    connect(m_client, &ServerClient::serverAck, this, [this](const QString &msg) {
        if (!msg.isEmpty()) appendLog(QStringLiteral("ACK %1").arg(msg));
    });
    connect(m_client, &ServerClient::peersReceived, this, [this](bool robot, bool cctv) {
        // STATUS 와 같은 이유로 **바뀔 때만** 남긴다 (로그 도배 방지).
        const bool changed = (robot != m_peerRobot) || (cctv != m_peerCctv);
        m_peerRobot = robot;
        m_peerCctv = cctv;
        // PEERS 는 서버가 **직접 세는 접속 수**다 — 로봇 접속 여부의 최종 근거.
        // robot=0 이면 타임아웃을 기다리지 말고 그 자리에서 오프라인으로 내린다.
        if (robot) {
            m_hasRobotBeat = true;
            m_lastRobotBeat.restart();
        } else {
            m_hasRobotBeat = false;
            m_lastRobotBeat.invalidate();
        }
        if (cctv)
            m_cctvOnline = true;
        refreshLinkStatus();
        if (changed)
            appendLog(QString("PEERS robot=%1 cctv=%2").arg(robot).arg(cctv));
    });

    m_linkTimer = new QTimer(this);
    m_linkTimer->setInterval(500);
    connect(m_linkTimer, &QTimer::timeout, this, &Backend::refreshLinkStatus);
    m_linkTimer->start();

    // 테스트 모드 시뮬레이션 — **서버로 보낸 동작 시퀀스를 그대로 재생한다.**
    // ⚠️ 예전에는 도면 폴리라인 위를 8초에 걸쳐 훑기만 해서, 후진·제자리회전·노즐이
    //    화면에 하나도 나타나지 않았다. 미리보기가 실행본과 달라지면 로봇 없이
    //    동선을 검증할 방법이 없어진다 → 반드시 Player 로만 움직인다.
    m_testProgressTimer = new QTimer(this);
    m_testProgressTimer->setInterval(kSimTickMs);
    connect(m_testProgressTimer, &QTimer::timeout, this, [this]() {
        if (!m_jobActive || !m_testMode) {
            stopTestProgressSim();
            return;
        }
        m_sim.step(kSimTickMs / 1000.0 * m_simSpeedFactor);

        pushSimPoseToView(m_sim.center(), m_sim.headingDeg(), m_sim.nozzleDown());
        setJobProgress(m_sim.progress01());

        const QString phase = m_sim.phaseText();
        if (phase != m_simPhase) { m_simPhase = phase; emit simChanged(); }

        if (m_sim.finished()) {
            setJobProgress(1.0);
            finishJob("테스트 시뮬레이션 완료");
        }
    });

    m_jobTick = new QTimer(this);
    m_jobTick->setInterval(500);
    connect(m_jobTick, &QTimer::timeout, this, [this]() {
        if (m_jobActive || m_jobElapsedValid)
            emit jobChanged();
    });
    m_jobTick->start();
}

Backend::~Backend()
{
    stopTestProgressSim();
    if (m_worker) { m_worker->stop(); m_worker->wait(); }
}

void Backend::setKeyboardControl(bool v)
{
    if (m_keyboardControl == v) return;
    m_keyboardControl = v;
    emit keyboardControlChanged();
    if (v) appendLog("키보드 로봇 제어 ON");
}

void Backend::login(const QString &id, const QString &pw)
{
    if (id.isEmpty() || pw.isEmpty()) { emit loginFailed("아이디와 비밀번호를 입력해주세요."); return; }

    if (id == "test" && pw == "test") {
        m_testMode = true; m_userId = "test"; m_calib = QJsonObject();
        m_serverConnected = false;
        m_serverLabel = "테스트";
        loadHistory();
        emit sessionChanged();
        emit linkStatusChanged();
        startWorker();
        appendLog("테스트 모드 로그인");
        updatePhase();
        emit loginSucceeded();
        return;
    }

    m_busy = true; emit busyChanged();

    auto *guard = new QObject(this);
    auto *timer = new QTimer(guard);
    timer->setSingleShot(true);
    auto finish = [this, guard](bool ok, const QJsonObject &calib, bool hasCalib,
                                const QString &camIp, const QString &id, const QString &reason) {
        m_busy = false; emit busyChanged();
        guard->deleteLater();
        if (ok) {
            m_testMode = false; m_userId = id;
            m_serverLabel = "연결됨";
            m_calibMissing = !hasCalib;
            // 서버는 로그인 성공 시점에 보관 중인 번들을 함께 내려준다 (QT-REQ-SRV-001 S-2).
            // 수동 입력과 같은 경로를 타야 K/D 와 coord_mode 가 함께 반영된다.
            if (hasCalib)
                appendLog("LOGIN_OK calib — " + applyCalibObject(calib, QStringLiteral("LOGIN_OK")));
            else
                m_calib = calib;
            loadHistory();
            emit sessionChanged();
            emit linkStatusChanged();
            emit calibChanged();
            // 서버에 등록해둔 카메라 IP 가 있으면 그걸로 RTSP URL 을 조립한다.
            if (!camIp.trimmed().isEmpty())
                setCamIp(camIp.trimmed());
            startWorker();
            appendLog(QString("로그인 성공: %1").arg(id));
            if (m_calibMissing) {
                setNotice(QStringLiteral("이 현장은 아직 캘리브레이션이 없습니다. 관리자 창(%1)에서 "
                                         "캘리브레이션을 먼저 진행하세요.").arg(adminConsoleUrl()),
                          QStringLiteral("warn"));
                appendLog(QStringLiteral("LOGIN_OK calib=null — 관리자 창 안내: %1").arg(adminConsoleUrl()));
            }
            updatePhase();
            emit loginSucceeded();
        } else {
            emit loginFailed(reason.isEmpty() ? "로그인에 실패했습니다." : reason);
        }
    };

    connect(m_client, &ServerClient::loginResult, guard,
            [finish](bool ok, const QString &rid, const QJsonObject &calib, bool hasCalib,
                     const QString &camIp, const QString &reason) {
        finish(ok, calib, hasCalib, camIp, rid, reason);
    });
    connect(m_client, &ServerClient::socketError, guard, [finish](const QString &e) {
        finish(false, QJsonObject(), false, QString(), QString(), e);
    });
    connect(timer, &QTimer::timeout, guard, [this, finish]() {
        finish(false, QJsonObject(), false, QString(), QString(),
               QStringLiteral("서버 응답 시간 초과 (%1)").arg(serverAddress()));
    });

    if (m_client->isEncrypted()) {
        m_client->sendLogin(id, pw);
    } else {
        connect(m_client, &ServerClient::connectedToServer, guard, [this, id, pw]() {
            m_client->sendLogin(id, pw);
        });
        m_client->connectToServer();
    }
    timer->start(6000);
}

void Backend::registerUser(const QString &id, const QString &pw, const QString &camIp)
{
    if (id.isEmpty() || pw.isEmpty()) { emit registerFailed("아이디와 비밀번호를 입력해주세요."); return; }

    m_busy = true; emit busyChanged();
    auto *guard = new QObject(this);
    auto *timer = new QTimer(guard);
    timer->setSingleShot(true);
    auto finish = [this, guard](bool ok, const QString &reason) {
        m_busy = false; emit busyChanged();
        guard->deleteLater();
        if (ok) emit registerSucceeded();
        else    emit registerFailed(reason.isEmpty() ? "회원가입에 실패했습니다." : reason);
    };
    connect(m_client, &ServerClient::registerResult, guard,
            [finish](bool ok, const QString &, const QString &reason) { finish(ok, reason); });
    connect(m_client, &ServerClient::socketError, guard, [finish](const QString &e) { finish(false, e); });
    connect(timer, &QTimer::timeout, guard, [this, finish]() {
        finish(false, QStringLiteral("서버 응답 시간 초과 (%1)").arg(serverAddress()));
    });

    if (m_client->isEncrypted()) {
        m_client->sendRegister(id, pw, camIp);
    } else {
        connect(m_client, &ServerClient::connectedToServer, guard, [this, id, pw, camIp]() {
            m_client->sendRegister(id, pw, camIp);
        });
        m_client->connectToServer();
    }
    timer->start(6000);
}

QString Backend::serverAddress() const
{
    return m_client ? QStringLiteral("%1:%2").arg(m_client->host()).arg(m_client->port())
                    : QStringLiteral("—");
}

// 캘리브레이션이 없을 때 안내할 관리자 창 주소. 서버가 내려주지 않으므로 QT 고정값.
QString Backend::adminConsoleUrl() const
{
    return m_client ? QStringLiteral("http://%1:8083").arg(m_client->host())
                    : QStringLiteral("http://192.168.0.8:8083");
}

void Backend::setServerAddress(const QString &host, int port)
{
    if (!m_client || host.trimmed().isEmpty()) return;
    const bool wasConnected = m_client->isConnected();
    m_client->setServer(host.trimmed(), quint16(port > 0 ? port : 9000));
    appendLog(QStringLiteral("서버 주소 변경: %1").arg(serverAddress()));
    emit linkStatusChanged();
    if (wasConnected) reconnectServer();
}

// ArUco 검출 결과 → 양쪽 뷰 오버레이 + 요약 문자열 ("3개 · ID 0, 4, 7")
void Backend::onAruco(const QList<int> &ids, const QList<QPolygonF> &corners)
{
    if (m_originalView) m_originalView->setArucoMarkers(ids, corners);
    if (m_topView) m_topView->setArucoMarkers(ids, corners);

    QString summary;
    if (!ids.isEmpty()) {
        QList<int> sorted = ids;
        std::sort(sorted.begin(), sorted.end());
        sorted.erase(std::unique(sorted.begin(), sorted.end()), sorted.end());
        QStringList sl;
        for (int id : std::as_const(sorted)) sl << QString::number(id);
        summary = QStringLiteral("마커 %1개 · ID %2").arg(ids.size()).arg(sl.join(", "));
    }
    if (summary != m_arucoSummary) {
        m_arucoSummary = summary;
        emit arucoChanged();
    }
}

void Backend::setArucoOverlay(bool on)
{
    if (m_arucoOverlay == on) return;
    m_arucoOverlay = on;
    if (m_originalView) m_originalView->setArucoVisible(on);
    if (m_topView) m_topView->setArucoVisible(on);
    emit arucoChanged();
    appendLog(on ? "ArUco 마커 표시 켬" : "ArUco 마커 표시 끔");
}

void Backend::setNotice(const QString &text, const QString &level, const QString &key)
{
    if (m_notice == text && m_noticeLevel == level) return;
    m_notice = text;
    m_noticeLevel = level;
    m_noticeKey = key;
    emit noticeChanged();
}

// 특정 상황용 통지만 골라서 내린다 (예: 비상정지가 풀리면 비상정지 안내를 치운다)
void Backend::clearNotice(const QString &key)
{
    if (m_notice.isEmpty() || m_noticeKey != key) return;
    m_notice.clear();
    m_noticeKey.clear();
    emit noticeChanged();
}

void Backend::dismissNotice()
{
    if (m_notice.isEmpty()) return;
    m_notice.clear();
    m_noticeKey.clear();
    emit noticeChanged();
}

void Backend::logout()
{
    stopTestProgressSim();
    clearMission();
    if (m_worker) { m_worker->stop(); m_worker->wait(); m_worker->deleteLater(); m_worker = nullptr; }
    m_workerStarted = false;
    m_topView = nullptr;
    m_originalView = nullptr;

    if (m_drawing) { m_drawing = false; emit drawingChanged(); }
    if (m_keyboardControl) { m_keyboardControl = false; emit keyboardControlChanged(); }
    m_estopActive = false;

    if (m_client && m_client->isConnected()) m_client->disconnectFromServer();

    appendLog(QString("로그아웃: %1").arg(m_userId.isEmpty() ? "사용자" : m_userId));
    m_userId.clear();
    m_camIp.clear();
    m_testMode = false;
    m_calib = QJsonObject();
    m_calibMissing = false;
    m_notice.clear();
    emit noticeChanged();
    emit calibChanged();
    m_serverConnected = false;
    m_serverLabel = "미연결";
    m_cctvOnline = false;
    m_cctvFps = 0;
    m_robotOnline = false;
    m_poseValid = false;
    m_hasRobotBeat = false;
    m_lastRobotBeat.invalidate();
    m_hasPoseBeat = false;
    m_lastPoseBeat.invalidate();
    m_poseEverSeen = false;
    m_poseWaitWarned = false;
    m_peerRobot = m_peerCctv = -1;
    emit sessionChanged();
    emit linkStatusChanged();
    emit poseChanged();

    m_robotStatus = "대기";
    m_robotState = "IDLE";
    m_painting = false;
    emit robotStatusChanged();
    updatePhase();
    emit loggedOut();
}

void Backend::reconnectServer()
{
    if (m_testMode) {
        appendLog("테스트 모드 — 서버 재연결 없음");
        return;
    }
    if (m_client->isConnected())
        m_client->disconnectFromServer();
    m_serverLabel = "재연결…";
    emit linkStatusChanged();
    m_client->connectToServer();
    appendLog("서버 재연결 시도");
}

void Backend::wireWorker(video_worker *w)
{
    if (!w) return;
    // 실제 영상이 한 장이라도 들어오면 오프라인 대체 캔버스를 끈다.
    // (RTSP 주소를 바꿔 워커를 새로 만들 때도 반드시 다시 걸려야 한다)
    connect(w, &video_worker::frameReceived, this, [this](const QImage &img) {
        // 캘리브 해상도와 대조하는 용도. 재연결로 프로파일이 바뀌면 값도 따라와야 하므로
        // 첫 프레임만이 아니라 매번 갱신한다 (int 두 개라 비용은 무시할 수준).
        m_frameW = img.width();
        m_frameH = img.height();
        if (m_gotRealFrame) return;
        m_gotRealFrame = true;
        if (m_frameWatch) m_frameWatch->stop();
        emit linkStatusChanged();
        // 영상이 들어왔으면 "영상 없음/연결 실패" 배너는 거짓말이 된다. 같이 내린다.
        clearNotice(QStringLiteral("camera"));
        clearNotice(QStringLiteral("rtsp"));
        appendLog("카메라 영상 수신 시작 — 실시간 화면으로 전환");
    });
    connect(w, &video_worker::statsUpdated, this, [this](const StreamStats &s) {
        m_cctvOnline = s.isConnected;
        m_cctvFps = s.fps;
        m_cctvLatencyMs = s.latencyMs;
        emit linkStatusChanged();
    });
    // 스트림을 못 열면 그대로 포기한다(재시도 없음 — 틀린 비밀번호로 계속 두드리면
    // 카메라가 계정을 잠근다). 대신 이유를 로그에 남겨서 조작자가 주소를 고칠 수 있게 한다.
    connect(w, &video_worker::openFailed, this, [this](const QString &url) {
        appendLog(QStringLiteral("RTSP 연결 실패 — %1").arg(url));
        appendLog(QStringLiteral("설정에서 카메라 주소를 확인하세요 "
                                 "(IP·계정·프로파일 경로). 자동 재시도는 하지 않습니다."));
        setNotice(QStringLiteral("카메라 영상을 열지 못했습니다. 설정에서 RTSP 주소를 확인하세요."),
                  QStringLiteral("warn"), QStringLiteral("rtsp"));
    });
    // 카메라 쪽에서 끊긴 뒤 스스로 다시 붙었을 때. 조작자 입장에서는 화면이 잠깐
    // 멈췄다 돌아온 것이라, 이유를 안 남기면 "앱이 렉 걸렸다" 로만 보인다.
    connect(w, &video_worker::streamReconnected, this, [this]() {
        appendLog(QStringLiteral("영상이 끊겨 재연결했습니다 "
                                 "(카메라 설정 저장·세션 경합 등으로 스트림이 재시작된 경우)"));
    });
    connect(w, &video_worker::arucoDetected, this, &Backend::onAruco);
    if (m_topView)
        connect(w, &video_worker::frameReceived, m_topView, &VideoView::onFrame);
    if (m_originalView)
        connect(w, &video_worker::frameReceived, m_originalView, &VideoView::onFrame);
    // 백프레셔 해제는 **맨 마지막에** 연결한다 — 큐드 연결은 연결 순서대로 배달되므로,
    // 뷰들이 프레임을 다 처리한 뒤에야 "소비했다"고 알리게 된다.
    connect(w, &video_worker::frameReceived, this, [w]() { w->frameConsumed(); });
}

void Backend::startWorker()
{
    if (m_workerStarted) return;
    m_worker = new video_worker(m_rtspUrl, this);
    wireWorker(m_worker);
    m_worker->start();
    m_workerStarted = true;

    // 카메라가 아직(또는 끝내) 안 붙어도 작도는 되어야 한다.
    // 잠깐 기다려보고 영상이 없으면 빈 바닥 캔버스를 대신 띄운다.
    if (!m_frameWatch) {
        m_frameWatch = new QTimer(this);
        m_frameWatch->setInterval(2500);
        connect(m_frameWatch, &QTimer::timeout, this, &Backend::pushPlaceholderFrame);
    }
    m_frameWatch->start();
    QTimer::singleShot(2500, this, &Backend::pushPlaceholderFrame);
}

// 카메라 없이 쓰는 빈 바닥. 데모 바닥이 초록이라 같은 톤으로 깔아
// 흰 도장선이 실제로 어떻게 보일지 미리 가늠할 수 있게 한다.
void Backend::pushPlaceholderFrame()
{
    if (m_gotRealFrame) return;
    if (!m_topView && !m_originalView) return;

    QImage img(1280, 720, QImage::Format_RGB888);
    img.fill(QColor(42, 78, 54));
    {
        QPainter p(&img);
        p.setPen(QPen(QColor(255, 255, 255, 24), 1));
        for (int x = 0; x < img.width(); x += 64) p.drawLine(x, 0, x, img.height());
        for (int y = 0; y < img.height(); y += 64) p.drawLine(0, y, img.width(), y);
    }

    // TopView 는 이 프레임을 호모그래피로 뒤집어 펴므로 글자를 넣으면 거꾸로 보인다.
    // 안내 문구는 원본(CCTV) 화면에만 얹는다.
    if (m_topView) m_topView->onFrame(img);
    if (m_originalView) {
        QImage note = img;
        QPainter p(&note);
        p.setPen(QColor(255, 255, 255, 120));
        p.setFont(QFont("Pretendard", 15, QFont::DemiBold));
        p.drawText(note.rect(), Qt::AlignCenter,
                   QStringLiteral("카메라 영상 없음 · 오프라인 작도 모드"));
        p.end();
        m_originalView->onFrame(note);
    }

    static bool warned = false;
    if (!warned) {
        warned = true;
        emit linkStatusChanged();
        setNotice(QStringLiteral("카메라 영상이 없어 오프라인 작도 모드로 시작했습니다. "
                                 "도면 작성·저장은 그대로 되고, 영상이 들어오면 자동 전환됩니다."),
                  QStringLiteral("warn"), QStringLiteral("camera"));
        appendLog("카메라 미수신 — 오프라인 작도 캔버스로 대체");
    }
}

void Backend::configureView(VideoView *v, bool topView)
{
    // 뷰가 새로 생겨도 렌즈 보정 상태가 따라가야 한다 (설정에서 켜둔 채 재시작한 경우)
    if (topView && v) { v->setLensModel(m_cam); v->setLensCorrection(m_lensOn); }

    v->setStrokeWidthMm(m_strokeWidthMm);
    v->setArucoVisible(m_arucoOverlay);
    if (topView) {
        v->setRobotVisible(m_robotVisible);   // 뷰가 새로 생겨도 토글 상태가 따라간다
        v->setTopView(true);
        v->setInteractive(true);
        if (m_testMode) {
            v->configureTopViewTest();
        } else if (!m_calib.isEmpty()) {
            // 🔴 지연 적용 경로. LOGIN_OK 이 화면보다 먼저 오면 applyCalibObject 가
            //    "보관됨 — 화면 준비 후 적용"만 남기고 실제 적용은 여기서 일어난다.
            //    예전에는 여기가 **아무 로그도 남기지 않아서**, 로그만 보면 캘리브가
            //    적용됐는지 · 단위를 뭐로 판정했는지 알 수 없었다.
            const bool ok = v->configureTopViewCalib(m_calib);
            m_calibMissing = false;
            refreshCalibStatus();
            appendLog(ok ? QStringLiteral("캘리브 적용(지연, %1): ").arg(m_calibSource)
                               + m_calibStatus
                         : QStringLiteral("캘리브 실패(지연, %1) — 테스트 보정으로 폴백")
                               .arg(m_calibSource));
            logCalibUnit(v);
        } else {
            v->configureTopViewTest();
        }
        refreshCalibStatus();
        pushMissionToView();
        pushOverlayToOriginal();
    } else {
        v->setTopView(false);
        v->setInteractive(false);
        pushMissionToView();
    }
}

void Backend::registerView(VideoView *view, bool topView)
{
    if (!view) return;
    if (topView) {
        m_topView = view;
        connect(view, &VideoView::pathChanged, this, [this]() {
            pushOverlayToOriginal();
            if (m_drawing && m_topView && m_topView->isClosed()) {
                m_drawing = false;
                emit drawingChanged();
            }
            const int n = m_topView ? m_topView->totalPointCount() : 0;
            if (!m_jobActive && n >= 2) {
                QList<bool> cl;
                const auto paths = m_topView->pathsToMeters(&cl);
                m_pathLengthM = pathsLength(paths, cl);
                m_waypointCount = pathsPointCount(paths);
                m_waypointIndex = 0;
                m_workName = "편집 중";
            } else if (!m_jobActive && n < 2 && m_missionPaths.isEmpty()) {
                m_pathLengthM = 0;
                m_waypointCount = 0;
                m_workName = "—";
            }
            emit jobChanged();
            updatePhase();
        });
        connect(view, &VideoView::pathClosed, this, [this]() {
            appendLog("첫 점과 만나 경로를 닫았습니다.");
        });
        connect(view, &VideoView::pointsMerged, this, [this](int n) {
            appendLog(QStringLiteral("가까운 점 %1개를 하나로 합쳤습니다.").arg(n));
        });
        connect(view, &VideoView::freeDrawEnded, this, [this]() {
            if (!m_drawing) return;
            m_drawing = false;
            emit drawingChanged();
            appendLog("작도 종료 (더블클릭)");
            updatePhase();
        });
    } else {
        m_originalView = view;
    }
    configureView(view, topView);
    if (m_worker)
        connect(m_worker, &video_worker::frameReceived, view, &VideoView::onFrame);
}

void Backend::startDrawSession(bool preset)
{
    if (m_jobActive) return;   // 실행 중 편집 금지 (먼저 중단)
    if (m_jobProgress > 0)
        clearMission();
    if (preset) {
        // 프리셋은 addPreset / Drop 으로 바로 놓음 — 별도 세션 불필요
        m_drawing = false;
        emit drawingChanged();
        updatePhase();
        return;
    }
    m_drawing = true;
    emit drawingChanged();
    if (m_topView) m_topView->startDraw();
    appendLog("자유 작도 — 클릭으로 점 추가, 더블클릭으로 종료");
    updatePhase();
}

void Backend::addPreset(const QString &type)
{
    if (m_jobActive) return;   // 실행 중 편집 금지
    if (m_phase == QLatin1String("done"))
        clearMission();
    if (!m_topView) return;
    m_topView->setPresetShape(type);
    // 프리셋은 편집 가능한 닫힌 경로 → 점 찍기 모드 아님
    if (m_drawing) {
        m_drawing = false;
        emit drawingChanged();
    }
    appendLog(QString("프리셋 %1").arg(type));
    updatePhase();
}

void Backend::undo()
{
    if (m_topView) m_topView->undo();
    updatePhase();
}

// 편집 중인 도형뿐 아니라 이미 완성해 쌓아둔 도형까지 센다.
// (활성 도형만 세면 프리셋을 하나 더 얹는 순간 "전체 지우기"·"전송"이 꺼져버린다)
int Backend::pathPointCount() const
{
    return m_topView ? m_topView->totalPointCount() : 0;
}

// ── 주행 경로 계획 ────────────────────────────────────────────────────
// 도형들을 로봇 현재 위치에서 가장 가깝게 이어지도록 재배열·뒤집어 한 줄로 만든다.
// 로봇 위치를 모르면(POSE 미수신) 첫 도형은 작도 순서·방향을 그대로 존중한다.
routeplan::Route Backend::buildRoute(const QList<QList<QPointF>> &paths,
                                     const QList<bool> &closed) const
{
    return routeplan::plan(paths, closed, QPointF(m_poseX, m_poseY), m_poseValid);
}

// 계획 결과를 사람이 읽을 수 있게 로그로 남긴다 — "왜 이 순서로 도는가"를
// 조작자가 확인할 수 있어야 로봇이 엉뚱하게 도는지 판단할 수 있다.
void Backend::logRoute(const routeplan::Route &route,
                       const QList<QList<QPointF>> &paths,
                       const QList<bool> &closed) const
{
    if (route.shapeCount <= 0) return;
    const int dropped = pathsPointCount(paths) - route.paintPointCount();
    if (dropped > 0)
        const_cast<Backend *>(this)->appendLog(
            QString("경로 단순화 — 1cm 이하 꺾임 %1점 정리 (로봇이 헛멈추지 않게)").arg(dropped));
    if (route.shapeCount > 1) {
        const double before = routeplan::naiveTravel(paths, closed,
                                                     QPointF(m_poseX, m_poseY), m_poseValid);
        const_cast<Backend *>(this)->appendLog(
            QString("도형 %1개 순서 재배열 — 도형 사이 빈 이동 %2 m → %3 m (도색 %4 m)")
                .arg(route.shapeCount)
                .arg(before, 0, 'f', 2)
                .arg(route.travelM, 0, 'f', 2)
                .arg(route.paintM, 0, 'f', 2));
        const_cast<Backend *>(this)->appendLog(
            QStringLiteral("도형 사이 이동 구간은 pen-up(paint=false) 으로 보냅니다 — 칠하지 않고 지나갑니다"));
    }
}

// 계획된 폴리라인을 다시 도형 단위로 쪼갠다 (미션 표시·이력·진행률용).
QList<QList<QPointF>> Backend::routeShapes(const routeplan::Route &route,
                                           QList<bool> *closedOut)
{
    QList<QList<QPointF>> out;
    if (closedOut) closedOut->clear();
    QList<QPointF> cur;
    int shape = -1;
    auto flush = [&]() {
        if (cur.size() < 2) { cur.clear(); return; }
        // orient() 가 닫힌 도형 끝에 시작점을 복귀시켜 두었다 → 중복 점을 걷어내고
        // closed 플래그로 되돌린다 (미션 렌더러가 스스로 닫아 그린다)
        bool isClosed = cur.size() > 3
                        && QLineF(cur.first(), cur.last()).length() < 1e-6;
        if (isClosed) cur.removeLast();
        out.append(cur);
        if (closedOut) closedOut->append(isClosed);
        cur.clear();
    };
    for (int i = 0; i < route.pts.size(); ++i) {
        if (route.shapeOf.value(i, shape) != shape) { flush(); shape = route.shapeOf.value(i, shape); }
        cur.append(route.pts[i]);
    }
    flush();
    return out;
}

// 1단계: BLUEPRINT 전송 = 서버에 도면을 "저장"만 한다.
// (2026-07-27 서버 변경) 이 시점에 로봇은 움직이지 않는다. 실행은 startPainting() 부터.
void Backend::commitDrawing()
{
    if (!m_topView) return;
    QList<bool> closed;
    const QList<QList<QPointF>> paths = m_topView->pathsToMeters(&closed);
    const int n = pathsPointCount(paths);
    if (n < 2) { appendLog("작도된 경로가 없습니다."); return; }
    if (m_jobActive) { appendLog("이미 작업이 진행 중입니다."); return; }

    // 프로토콜의 BLUEPRINT 는 폴리라인 1개다 → 도형들을 한 줄로 이어야 한다.
    // 작도 순서 그대로 이으면 로봇이 도형 사이를 가로지르며 현장을 활보하고, 그
    // 이동 구간까지 도색된다. 여기서 순서·진행 방향을 다시 잡고 이동 구간에는
    // pen-up(paint=false) 을 붙인다. (routeplan.h)
    const routeplan::Route route = buildRoute(paths, closed);
    const QList<QPointF> &blueprint = route.pts;
    if (blueprint.size() < 2) { appendLog("작도된 경로가 없습니다."); return; }

    // 동작 시퀀스는 **Qt 가 만들어 그대로 보낸다** (2026-07-28 설계 변경).
    // 경로는 불변이고 실시간성이 필요 없다 — 서버는 저장·중계하고 CCTV 보정만 한다.
    // 그래서 화면의 미리보기가 곧 실행본이 된다.
    // ⚠️ 펜 오프셋은 넘기지 않는다. program 은 **도면 그대로**의 논리 동작이고,
    //    꼭짓점 보정은 로봇이 자기 하드웨어 상수로 알아서 한다 (프로토콜 2026-07-28).
    const QList<motionprogram::Op> program =
        motionprogram::build(blueprint, route.paint, m_speeds);

    if (m_testMode) {
        appendLog(QString("[테스트] BLUEPRINT 전송 생략 (%1점 / 동작 %2개) — 서버 저장 시뮬레이션")
                      .arg(n).arg(program.size()));
    } else {
        m_client->sendBlueprint(blueprint, route.paint, program);
        appendLog(QString("BLUEPRINT %1점 + 동작 시퀀스 %2개 전송 — 서버에 저장됨 (로봇은 아직 대기)")
                      .arg(blueprint.size()).arg(program.size()));
    }
    logRoute(route, paths, closed);
    if (!program.isEmpty()) {
        appendLog(QString("동작 시퀀스 — 직진 %1 · 곡선 %2 · 회전 %3 · 노즐 %4 · 총 %5 m")
                      .arg(motionprogram::countOf(program, motionprogram::Op::Move))
                      .arg(motionprogram::countOf(program, motionprogram::Op::Arc))
                      .arg(motionprogram::countOf(program, motionprogram::Op::Turn))
                      .arg(motionprogram::countOf(program, motionprogram::Op::Nozzle))
                      .arg(motionprogram::totalTravelM(program), 0, 'f', 2));
    }

    // 로봇이 실제로 달릴 순서로 다시 묶은 도형 — 미션 표시·진행률·이력이 전부
    // 전송한 경로와 같은 순서를 보게 한다.
    QList<bool> orderedClosed;
    const QList<QList<QPointF>> ordered = routeShapes(route, &orderedClosed);

    // 이력에 남긴다 — 나중에 "불러와 수정" 으로 그대로 꺼내 쓴다
    m_currentJobId = recordJob(ordered, orderedClosed,
                               m_workName == "편집 중" || m_workName == "—" ? QString() : m_workName,
                               QStringLiteral("대기"));

    storeMission(ordered, orderedClosed);
    m_routePts = route.pts;
    m_routePaint = route.paint;
    m_travelLengthM = route.travelM;
    m_program = program;
    m_blueprintSent = true;

    m_topView->clearPath();
    if (m_originalView) m_originalView->setOverlayPaths({}, {});
    m_drawing = false; emit drawingChanged();
    emit jobChanged();
    updatePhase();
}

// 2단계: "그림그리기 시작" — 이 한 번으로 접근 → 도색 → 완료까지 서버가 자동 진행한다.
// 접근 완료는 통지되지 않으므로 중간에 누를 버튼을 만들지 말 것. 끝은 DRAW_DONE 뿐이다.
void Backend::startPainting()
{
    if (m_jobActive) return;                      // 연타 방지 (서버도 busy 로 막지만 UI 가 먼저)
    if (!canEditMission()) { appendLog("전송된 경로가 없습니다."); return; }
    if (!m_blueprintSent) {
        appendLog("도면을 먼저 전송하세요.");
        setNotice(QStringLiteral("도면이 서버에 없습니다. '경로 전송'을 먼저 누르세요."),
                  QStringLiteral("warn"));
        return;
    }

    if (m_testMode) {
        appendLog("[테스트] START_DRAW 생략 — 접근+도색 시뮬레이션");
    } else {
        m_client->sendCmd("START_DRAW");
        appendLog("CMD START_DRAW — 접근→도색까지 서버가 자동 진행");
    }
    beginPainting();
    if (m_testMode)
        startTestProgressSim();
}

// 진행 중인 작업 중단. 경로 실행 중에는 서버가 수동 STOP 을 무시하므로 ESTOP 을 쓴다.
void Backend::cancelJob()
{
    if (!m_jobActive) return;
    const double doneSoFar = m_jobProgress;   // 이력에는 중단 시점 진행률을 남긴다
    stopTestProgressSim();

    sendRobotCmd("ESTOP", "작업 중단");
    m_jobActive = false;
    m_paintingSeen = false;
    m_jobElapsedValid = false;
    m_jobProgress = 0.0;
    m_phase = "idle";
    if (m_topView) m_topView->setMissionProgress(0.0);
    if (m_originalView) m_originalView->setMissionProgress(0.0);
    updateJobRecord(m_currentJobId, QStringLiteral("중단"), doneSoFar);
    emit jobChanged();
    setNotice(QStringLiteral("작업을 중단했습니다. 로봇은 비상정지 상태입니다 — "
                             "다시 움직이려면 ESTOP 해제를 누르세요."),
              QStringLiteral("warn"), QStringLiteral("estop"));
    appendLog("작업 중단 (ESTOP)");
    updatePhase();
}

// 전송했던 경로를 다시 편집 가능한 상태로 되돌린다 (중단/완료 후 "경로 수정").
void Backend::editMission()
{
    if (!canEditMission()) return;
    if (m_jobActive) {
        appendLog("작업을 먼저 중단한 뒤 수정하세요.");
        return;
    }
    const QList<QList<QPointF>> paths = m_missionPaths;
    const QList<bool> closed = m_missionClosed;

    stopTestProgressSim();
    m_jobProgress = 0.0;
    m_jobElapsedValid = false;
    // 편집하면 서버에 올려둔 도면은 낡은 값이 된다 → 다시 전송해야 한다
    m_blueprintSent = false;
    m_phase = "ready";
    if (m_topView) {
        m_topView->clearMission();
        m_topView->setEditPathsMeters(paths, closed);
    }
    if (m_originalView) m_originalView->clearMission();
    m_missionPaths.clear();
    m_missionClosed.clear();
    m_routePts.clear();
    m_routePaint.clear();
    m_program.clear();
    m_travelLengthM = 0.0;
    m_workName = "편집 중";
    m_drawing = false;
    emit drawingChanged();
    emit jobChanged();
    appendLog(QStringLiteral("경로 수정 — 도형 %1개(%2점)를 편집 상태로 되돌림")
                  .arg(paths.size()).arg(pathsPointCount(paths)));
    updatePhase();
}

// Esc / Enter — 점 찍기 종료 (더블·우클릭과 동일)
void Backend::finishDrawing()
{
    if (!m_topView || !m_drawing) return;
    m_topView->finishDraw();
}

void Backend::cancelDrawing()
{
    if (m_topView) m_topView->clearPath();
    if (m_originalView) m_originalView->setOverlayPaths({}, {});
    m_drawing = false; emit drawingChanged();
    appendLog("경로 지우기");
    updatePhase();
}

void Backend::clearMission()
{
    stopTestProgressSim();
    m_missionPaths.clear();
    m_missionClosed.clear();
    m_routePts.clear();
    m_routePaint.clear();
    m_program.clear();
    m_travelLengthM = 0.0;
    m_jobActive = false;
    m_blueprintSent = false;
    m_jobProgress = 0.0;
    m_pathLengthM = 0.0;
    m_waypointIndex = 0;
    m_waypointCount = 0;
    m_workName = "—";
    m_jobElapsedValid = false;
    if (m_topView)
        m_topView->clearMission();
    if (m_originalView)
        m_originalView->clearMission();
    emit jobChanged();
    updatePhase();
}

// 전송한 도면을 미션(표시·진행률 계산의 기준)으로 잡아둔다.
void Backend::storeMission(const QList<QList<QPointF>> &paths, const QList<bool> &closed)
{
    m_missionPaths = paths;
    m_missionClosed = closed;
    m_jobActive = false;
    m_jobProgress = 0.0;
    m_pathLengthM = pathsLength(paths, closed);
    m_waypointCount = pathsPointCount(paths);
    m_waypointIndex = 0;
    m_workName = QString("path_%1").arg(QDateTime::currentDateTime().toString("HHmmss"));
    m_jobElapsedValid = false;
    pushMissionToView();
    emit jobChanged();
}

void Backend::beginPainting()
{
    m_jobActive = true;
    m_paintingSeen = false;
    m_jobProgress = 0.0;
    m_waypointIndex = 0;
    m_jobElapsed.restart();
    m_jobElapsedValid = true;
    m_poseWaitWarned = false;   // 작업마다 한 번씩은 경고할 기회를 준다
    emit jobChanged();
    appendLog("작업 시작 — 접근 후 도색까지 자동 진행 (완료 시 DRAW_DONE)");
    updatePhase();
}

void Backend::finishJob(const QString &reason)
{
    stopTestProgressSim();
    m_jobActive = false;
    m_paintingSeen = false;
    m_blueprintSent = false;   // 완료된 도면은 다시 그리려면 재전송이 필요하다
    setJobProgress(1.0);
    m_waypointIndex = qMax(0, m_waypointCount - 1);
    m_phase = "done";
    m_phaseHint = "완료되었습니다. 새 작업을 시작하세요.";
    updateJobRecord(m_currentJobId, QStringLiteral("완료"), 1.0);
    emit jobChanged();
    appendLog(reason.isEmpty() ? m_phaseHint : reason);
}

// ── 프로토콜 PATH 미리보기 ────────────────────────────────────────────
// 서버가 BLUEPRINT 로부터 만들어낼 MOVE/TURN 시퀀스를 클라이언트에서 미리 계산한다.
// 부호 규약은 SERVER_PROTOCOL 과 동일: TURN.angle_deg 양수 = 좌회전.
// 서버로 보낸(또는 보낼) 동작 시퀀스를 그대로 QML 에 넘긴다.
// ⚠️ 여기서 시퀀스를 "다시 계산"하지 말 것 — 미리보기와 실행본이 어긋나는 순간
// 조작자가 로봇 동선을 검증할 방법이 없어진다.
QList<motionprogram::Op> Backend::currentProgram() const
{
    if (!m_program.isEmpty()) return m_program;
    if (m_routePts.size() >= 2)
        return motionprogram::build(m_routePts, m_routePaint, m_speeds);
    if (!m_topView) return {};
    QList<bool> closed;
    const routeplan::Route r = buildRoute(m_topView->pathsToMeters(&closed), closed);
    return motionprogram::build(r.pts, r.paint, m_speeds);
}

QVariantList Backend::motionPlan() const
{
    QVariantList out;
    for (const motionprogram::Op &o : currentProgram()) {
        QVariantMap m;
        m["op"] = o.opName();
        m["dist"] = o.dist;            // 음수 = 후진
        m["angle"] = o.angle;
        m["heading"] = o.heading;
        m["paint"] = o.paint;
        m["down"] = o.down;
        m["vertex"] = o.vertex;
        m["radius"] = o.radius;        // ARC — 도면상 곡선 반지름(m)
        m["dir"] = o.arcDirection();   // ARC — left / right
        m["speed"] = o.speed;          // 로컬 전용(전송 안 함). MOVE/ARC=m/s, TURN=deg/s
        out << m;
    }
    return out;
}

int Backend::turnCount() const
{
    return motionprogram::countOf(currentProgram(), motionprogram::Op::Turn);
}

// ── 작업 이력 ─────────────────────────────────────────────────────────
QString Backend::historyPath() const
{
    QString dir = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    if (dir.isEmpty()) dir = QDir::homePath() + "/.roadpainter";
    QDir().mkpath(dir);
    const QString who = m_userId.isEmpty() ? QStringLiteral("local") : m_userId;
    return dir + "/jobs-" + who + ".json";
}

void Backend::loadHistory()
{
    m_history = QJsonArray();
    QFile f(historyPath());
    if (f.open(QIODevice::ReadOnly)) {
        const QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
        if (doc.isArray()) m_history = doc.array();
        f.close();
    }
    emit historyChanged();
}

void Backend::saveHistory()
{
    // 너무 불어나지 않게 최근 60건만 남긴다
    while (m_history.size() > 60) m_history.removeAt(0);
    QFile f(historyPath());
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        appendLog("작업 이력을 저장하지 못했습니다: " + f.errorString());
        return;
    }
    f.write(QJsonDocument(m_history).toJson(QJsonDocument::Compact));
    f.close();
    emit historyChanged();
}

QString Backend::recordJob(const QList<QList<QPointF>> &paths, const QList<bool> &closed,
                           const QString &name, const QString &status)
{
    if (paths.isEmpty()) return QString();
    QJsonArray jpaths, jclosed;
    for (int i = 0; i < paths.size(); ++i) {
        QJsonArray one;
        for (const QPointF &p : paths[i]) {
            QJsonArray xy;
            xy.append(p.x());
            xy.append(p.y());
            one.append(xy);
        }
        jpaths.append(one);
        jclosed.append(closed.value(i, false));
    }

    QJsonObject o;
    const QString id = QUuid::createUuid().toString(QUuid::WithoutBraces).left(8);
    o["id"] = id;
    o["name"] = name.isEmpty()
                    ? QDateTime::currentDateTime().toString("MM/dd HH:mm") + " 작업"
                    : name;
    o["created"] = QDateTime::currentDateTime().toString(Qt::ISODate);
    o["status"] = status;
    o["progress"] = 0.0;
    o["lengthM"] = pathsLength(paths, closed);
    o["shapes"] = paths.size();
    o["points"] = pathsPointCount(paths);
    o["strokeMm"] = m_strokeWidthMm;
    o["paths"] = jpaths;
    o["closed"] = jclosed;
    m_history.append(o);
    saveHistory();
    return id;
}

void Backend::updateJobRecord(const QString &id, const QString &status, double progress)
{
    if (id.isEmpty()) return;
    for (int i = 0; i < m_history.size(); ++i) {
        QJsonObject o = m_history.at(i).toObject();
        if (o.value("id").toString() != id) continue;
        o["status"] = status;
        o["progress"] = progress;
        o["finished"] = QDateTime::currentDateTime().toString(Qt::ISODate);
        m_history.replace(i, o);
        saveHistory();
        return;
    }
}

QVariantList Backend::jobHistory() const
{
    QVariantList out;
    for (int i = m_history.size() - 1; i >= 0; --i) {   // 최신 먼저
        const QJsonObject o = m_history.at(i).toObject();
        QVariantMap m;
        m["id"] = o.value("id").toString();
        m["name"] = o.value("name").toString();
        m["status"] = o.value("status").toString();
        m["progress"] = o.value("progress").toDouble();
        m["lengthM"] = o.value("lengthM").toDouble();
        m["shapes"] = o.value("shapes").toInt();
        m["points"] = o.value("points").toInt();
        const QDateTime dt = QDateTime::fromString(o.value("created").toString(), Qt::ISODate);
        m["created"] = dt.isValid() ? dt.toString("yyyy-MM-dd HH:mm") : QString();

        QVariantList ps;
        const QJsonArray jp = o.value("paths").toArray();
        for (const QJsonValue &pv : jp) {
            QVariantList one;
            for (const QJsonValue &qv : pv.toArray()) {
                const QJsonArray xy = qv.toArray();
                one << QPointF(xy.at(0).toDouble(), xy.at(1).toDouble());
            }
            ps << QVariant(one);
        }
        m["paths"] = ps;

        QVariantList cl;
        for (const QJsonValue &c : o.value("closed").toArray()) cl << c.toBool();
        m["closed"] = cl;
        out << m;
    }
    return out;
}

void Backend::saveCurrentDrawing(const QString &name)
{
    if (!m_topView) return;
    QList<bool> closed;
    const QList<QList<QPointF>> paths = m_topView->pathsToMeters(&closed);
    if (pathsPointCount(paths) < 2) {
        appendLog("저장할 경로가 없습니다.");
        return;
    }
    recordJob(paths, closed, name, QStringLiteral("보관"));
    appendLog(QStringLiteral("도면 저장: %1").arg(name.isEmpty() ? "이름 없음" : name));
}

bool Backend::loadJob(const QString &id)
{
    if (m_jobActive) {
        appendLog("작업 진행 중에는 다른 도면을 불러올 수 없습니다.");
        setNotice(QStringLiteral("작업이 진행 중입니다. 먼저 중단한 뒤 불러오세요."),
                  QStringLiteral("warn"));
        return false;
    }
    for (int i = 0; i < m_history.size(); ++i) {
        const QJsonObject o = m_history.at(i).toObject();
        if (o.value("id").toString() != id) continue;

        QList<QList<QPointF>> paths;
        QList<bool> closed;
        for (const QJsonValue &pv : o.value("paths").toArray()) {
            QList<QPointF> one;
            for (const QJsonValue &qv : pv.toArray()) {
                const QJsonArray xy = qv.toArray();
                one << QPointF(xy.at(0).toDouble(), xy.at(1).toDouble());
            }
            if (one.size() >= 2) paths << one;
        }
        const QJsonArray jc = o.value("closed").toArray();
        for (int k = 0; k < paths.size(); ++k) closed << jc.at(k).toBool();
        if (paths.isEmpty()) return false;

        clearMission();
        if (m_topView) m_topView->setEditPathsMeters(paths, closed);
        m_drawing = false;
        m_workName = o.value("name").toString();
        emit drawingChanged();
        emit jobChanged();
        updatePhase();
        appendLog(QStringLiteral("이력에서 불러옴: %1 (도형 %2개)")
                      .arg(m_workName).arg(paths.size()));
        return true;
    }
    return false;
}

void Backend::renameJob(const QString &id, const QString &name)
{
    const QString clean = name.trimmed();
    if (clean.isEmpty()) return;
    for (int i = 0; i < m_history.size(); ++i) {
        QJsonObject o = m_history.at(i).toObject();
        if (o.value("id").toString() != id) continue;
        o["name"] = clean;
        m_history.replace(i, o);
        saveHistory();
        return;
    }
}

void Backend::deleteJob(const QString &id)
{
    for (int i = 0; i < m_history.size(); ++i) {
        if (m_history.at(i).toObject().value("id").toString() != id) continue;
        m_history.removeAt(i);
        saveHistory();
        return;
    }
}

// H 가 mm 를 내는지 m 를 내는지 판정한 결과를 로그에 남긴다.
// 번들의 `unit` 은 믿을 수 없다(서버가 ÷1000 후 갱신하거나 안 하거나) — 그래서 QT 가
// canvas_mm·image_size 로 검산한다. **어느 쪽으로 판정했는지가 로그에 없으면
// 좌표가 1000배 어긋났을 때 원인을 찾을 수가 없다.**
void Backend::logCalibUnit(VideoView *v)
{
    if (!v) return;
    const QString note = v->calibUnitNote();
    if (note.isEmpty()) return;
    appendLog(note.startsWith(QChar(0x26A0))     // ⚠️ 로 시작하면 불일치 경고다
                  ? note
                  : QStringLiteral("H 단위 검산 — ") + note);
}

// TopView 보정 요약 + 축척(1px = ? mm)을 UI로 밀어준다.
void Backend::refreshCalibStatus()
{
    if (!m_topView) return;
    m_calibStatus = m_topView->calibSummary();
    // 어느 번들이 걸려 있는지 상태줄에서 바로 보이게 한다. 화면과 좌표가 어긋날 때
    // 제일 먼저 확인해야 하는 게 이 두 개다.
    if (!m_calibId.isEmpty())
        m_calibStatus += QStringLiteral(" · %1").arg(m_calibId);
    if (!m_coordMode.isEmpty())
        m_calibStatus += QStringLiteral(" · %1").arg(m_coordMode);
    if (!m_calibSource.isEmpty())
        m_calibStatus += QStringLiteral(" · %1").arg(m_calibSource);
    m_mmPerPx = m_topView->mmPerPx();
    emit calibChanged();
}

QString Backend::scaleText() const
{
    if (m_mmPerPx <= 1e-9) return QStringLiteral("축척 —");
    return QStringLiteral("1 px = %1 mm · %2 px/mm")
        .arg(m_mmPerPx, 0, 'f', m_mmPerPx >= 1.0 ? 2 : 3)
        .arg(1.0 / m_mmPerPx, 0, 'f', 2);
}

void Backend::refreshJobMetrics()
{
    if (m_waypointCount <= 0) {
        m_waypointIndex = 0;
        return;
    }
    m_waypointIndex = qBound(0, int(m_jobProgress * (m_waypointCount - 1) + 0.5), m_waypointCount - 1);
}

QString Backend::elapsedText() const
{
    if (!m_jobElapsedValid) return "00:00:00";
    const qint64 sec = m_jobElapsed.elapsed() / 1000;
    const int h = int(sec / 3600);
    const int m = int((sec % 3600) / 60);
    const int s = int(sec % 60);
    return QString("%1:%2:%3")
        .arg(h, 2, 10, QChar('0'))
        .arg(m, 2, 10, QChar('0'))
        .arg(s, 2, 10, QChar('0'));
}

QString Backend::etaText() const
{
    if (!m_jobActive || m_jobProgress < 0.02 || !m_jobElapsedValid)
        return "—";
    const double elapsed = m_jobElapsed.elapsed() / 1000.0;
    const double totalEst = elapsed / m_jobProgress;
    const int remain = qMax(0, int(totalEst - elapsed));
    const int m = remain / 60;
    const int s = remain % 60;
    return QString("%1분 %2초").arg(m).arg(s);
}

void Backend::pushMissionToView()
{
    if (m_missionPaths.isEmpty()) {
        if (m_topView) m_topView->clearMission();
        if (m_originalView) m_originalView->clearMission();
        return;
    }
    if (m_topView) {
        m_topView->setMissionPathsMeters(m_missionPaths, m_missionClosed);
        m_topView->setMissionProgress(m_jobProgress);
        if (m_poseValid)
            pushPoseToView(true);
    }
    if (m_originalView && m_topView) {
        QList<QList<QPointF>> cctvPaths;
        QList<bool> cctvClosed;
        for (int i = 0; i < m_missionPaths.size(); ++i) {
            QList<QPointF> pts = m_missionPaths[i];
            // 닫는 변도 원본에서는 휜다 → 첫 점을 붙여 함께 변환하고 closed 는
            // false 로 넘긴다. (true 면 받는 쪽이 곧은 직선을 한 번 더 긋는다)
            if (i < m_missionClosed.size() && m_missionClosed[i] && pts.size() > 2)
                pts.append(pts.first());
            cctvPaths.append(m_topView->metersToOriginal(pts));
            cctvClosed.append(false);
        }
        m_originalView->setMissionPathsPixels(cctvPaths, cctvClosed);
        m_originalView->setMissionProgress(m_jobProgress);
    }
}

// 원본(CCTV) 뷰의 경로 오버레이 + 도포 폭 밴드를 다시 만들어 밀어 넣는다.
// 경로가 바뀔 때뿐 아니라 **캘리브레이션이 바뀔 때도** 불러야 한다 — 좌표 변환이
// 통째로 달라지므로 안 하면 원본 뷰만 예전 자리에 남는다.
void Backend::pushOverlayToOriginal()
{
    if (!m_originalView || !m_topView) return;
    QList<bool> ocl;
    const auto opaths = m_topView->overlayPathsForOriginal(&ocl);
    m_originalView->setOverlayPaths(opaths, ocl);
    m_originalView->setOverlayBands(m_topView->overlayBandsForOriginal());
}

void Backend::setJobProgress(double p)
{
    p = qBound(0.0, p, 1.0);
    // monotonic during job
    if (m_jobActive && p < m_jobProgress)
        p = m_jobProgress;
    if (qFuzzyCompare(p, m_jobProgress)) {
        if (m_topView) m_topView->setMissionProgress(m_jobProgress);
        if (m_originalView) m_originalView->setMissionProgress(m_jobProgress);
        return;
    }
    m_jobProgress = p;
    if (m_topView)
        m_topView->setMissionProgress(m_jobProgress);
    if (m_originalView)
        m_originalView->setMissionProgress(m_jobProgress);
    refreshJobMetrics();
    emit jobChanged();
    updatePhase();
}

// 미션 전체를 주행 순서대로 이어붙인 폴리라인 (닫힌 도형은 시작점 복귀 포함)
QList<QPointF> Backend::missionTravelLine() const
{
    QList<QPointF> travel;
    for (int i = 0; i < m_missionPaths.size(); ++i) {
        travel += m_missionPaths[i];
        if (m_missionClosed.value(i, false) && m_missionPaths[i].size() > 2)
            travel.append(m_missionPaths[i].first());
    }
    return travel;
}

double Backend::progressAlongPath(const QPointF &pose) const
{
    const QList<QPointF> travel = missionTravelLine();
    const double L = polylineLength(travel, false);
    if (L < 1e-6) return 0.0;
    return qBound(0.0, distanceAlongPolyline(travel, false, pose) / L, 1.0);
}

// 서버가 주는 POSE(x, y)는 **ArUco 마커(ID 49)의 중심**이다. 로봇팀 제원(2026-07-30):
//
//      ArUco 마커 중심  ──── 155mm ────  노즐(= 펜 = 페인트가 나오는 곳)
//
// 즉 **마커 = 로봇 기준점**이고, 펜은 거기서 155mm 떨어져 있다. 화면은 이렇게 나눈다:
//   · setRobotPose  ← POSE 그대로            = 마커 = 로봇 기준점 (섀시를 여기 그린다)
//   · setPenMarker  ← POSE − 155mm·heading   = 노즐 (실제로 칠하는 점)
//
// 방향이 **뒤**인 근거: 로봇은 "1m 직진"을 받으면 오프셋만큼 **더 가서** 1m 를 긋는다
// (사용자 확인). 펜이 뒤에 달려 있어야 그 보정이 성립한다 — 중심이 d 만큼 더 나아가야
// 뒤따라오는 펜이 시작점에 닿는다. 앞에 달렸다면 반대로 덜 가야 한다.
// 방향이 틀린 것으로 밝혀지면 아래 부호 하나만 뒤집으면 된다.
//
// ⚠️ 진행률·시작점 도착 판정은 **보정 안 한 POSE(마커)** 를 그대로 쓴다. 서버가 이탈
//    감시를 하는 기준도 같은 POSE 라, 여기서만 다른 점을 쓰면 두 판정이 어긋난다.
// ⚠️ 155mm 를 반영해서 로봇에 보내지 않는다. QT 는 도면 그대로만 낸다. 오프셋 보정은
//    로봇 RPi 전담이다(`pen_offset_m` 은 2026-07-28 프로토콜에서 폐지).
void Backend::pushPoseToView(bool valid)
{
    if (!m_topView) return;
    const double th = m_poseTheta * 0.017453292519943295;   // deg → rad
    m_topView->setRobotPose(m_poseX, m_poseY, m_poseTheta, valid);
    m_topView->setPenMarker(m_poseX - kPenOffsetM * std::cos(th),
                            m_poseY - kPenOffsetM * std::sin(th),
                            m_painting, valid);
}

// 테스트 재생 화면 표시 — 위와 **정확히 거울 관계**다.
//
// 실주행: 마커(POSE)를 받아서   → 노즐 = 마커 − 155mm
// 재생  : 노즐(도면 위 점)에서 → 마커 = 노즐 + 155mm
//
// 재생기는 시퀀스를 그대로 굴리므로 그 좌표가 곧 노즐이다(motionsim.h 참고).
// 🔴 예전에는 재생기 좌표를 setRobotPose 에도 그대로 넣어서, **미리보기에서는
//    섀시가 도면 선 위에** 있고 실주행에서는 155mm 옆에 있었다. 같은 도면인데
//    미리보기와 실주행의 로봇 자세가 달라 보이면 미리보기를 믿을 수 없게 된다.
void Backend::pushSimPoseToView(const QPointF &nozzleM, double headingDeg, bool down)
{
    if (!m_topView) return;
    const double th = headingDeg * 0.017453292519943295;    // deg → rad
    m_topView->setRobotPose(nozzleM.x() + kPenOffsetM * std::cos(th),
                            nozzleM.y() + kPenOffsetM * std::sin(th),
                            headingDeg, true);
    m_topView->setPenMarker(nozzleM.x(), nozzleM.y(), down, true);
}

void Backend::onPose(double x, double y, double thetaDeg)
{
    m_poseX = x; m_poseY = y; m_poseTheta = thetaDeg;
    m_poseValid = true;

    // 🔴 POSE 를 로봇 접속 신호로 쓰면 **안 된다.**
    //    POSE 는 서버가 *CCTV 가 본 마커*를 변환해 만드는 값이다. 로봇이 서버에
    //    접속돼 있든 아니든, 마커 49 가 화면에 보이기만 하면 계속 발행된다.
    //    예전에는 여기서 m_lastRobotBeat 를 리셋해서, 전원만 켜 둔(=접속 안 된)
    //    로봇도 "연결됨" 으로 떴고, **마커를 손으로 치우면 오프라인**이 됐다.
    //    로그에는 `PEERS robot=0` 이 찍혀 있는데 화면은 연결됨 — 정면으로 모순.
    //    접속 여부는 PEERS(서버가 직접 세는 접속 수)와 STATUS(로봇이 직접 보냄)만
    //    판단한다. 여기서는 **위치 수신 시각**만 기록한다.
    m_hasPoseBeat = true;
    m_lastPoseBeat.restart();

    // 첫 수신만 남긴다. POSE 는 초당 15~30개라 매번 찍으면 로그가 통째로 덮인다.
    // "한 번도 안 옴" 과 "오다가 끊김" 을 화면에서 구분하려면 이 한 줄이 있어야 한다.
    if (!m_poseEverSeen) {
        m_poseEverSeen = true;
        appendLog(QStringLiteral("로봇 위치 수신 시작 — POSE (%1, %2) m, %3°")
                      .arg(x, 0, 'f', 3).arg(y, 0, 'f', 3).arg(thetaDeg, 0, 'f', 1));
    }
    emit poseChanged();

    pushPoseToView(true);

    // 진행률은 "실제로 칠하기 시작한 뒤"부터만 의미가 있다.
    // START_DRAW 직후에는 로봇이 시작점으로 접근하는 중이라, 그때 경로상 위치를
    // 진행률로 환산하면 엉뚱한 값(예: 끝점 근처에서 출발)이 나온다.
    if (m_jobActive && !m_testMode && m_paintingSeen)
        setJobProgress(progressAlongPath(QPointF(x, y)));

    // ⚠️ 완료 판정은 하지 않는다. 끝은 서버의 DRAW_DONE 하나뿐이다.
    refreshLinkStatus();
}

void Backend::onStatus(const QString &state, bool painting)
{
    // STATUS 는 로봇이 초당 여러 번 보낸다 — 값이 그대로면 로그에 남기지 않는다.
    // 예전에는 무조건 찍어서 `STATUS IDLE painting=false` 가 로그를 통째로 덮었고,
    // 정작 봐야 할 줄이 스크롤 밖으로 밀려났다. 상태 자체는 상단 표시로 항상 보인다.
    const bool changed = (state != m_robotState) || (painting != m_painting);

    m_robotState = state;
    m_painting = painting;
    // 접근이 끝나고 도색으로 넘어가는 순간은 통지되지 않는다 → painting 으로 유추
    if (painting && m_jobActive && !m_paintingSeen) {
        m_paintingSeen = true;
        appendLog("도색 시작 감지 (STATUS.painting) — 접근 완료");
    }
    if (state == "IDLE")
        m_robotStatus = "대기";
    else if (state == "MOVING")
        m_robotStatus = painting ? "도장 중" : "이동 중";
    else if (state == "ESTOPPED")
        m_robotStatus = "비상정지";
    else if (state == "ERROR")
        m_robotStatus = "오류";
    else
        m_robotStatus = state;

    m_hasRobotBeat = true;
    m_lastRobotBeat.restart();
    if (state == "ESTOPPED") {
        m_estopActive = true;
    } else if (state == "IDLE" || state == "MOVING") {
        if (m_estopActive) clearNotice(QStringLiteral("estop"));
        m_estopActive = false;
    }

    emit robotStatusChanged();
    if (changed)
        appendLog(QString("STATUS %1 painting=%2").arg(state, painting ? "true" : "false"));

    // ⚠️ 여기서 완료 판정을 하지 말 것. 접근 도중에도 IDLE 이 잠깐씩 뜨기 때문에
    //    (READY 정렬 대기 등) 오판으로 작업이 끝나버린다. 끝은 DRAW_DONE 뿐이다.
    refreshLinkStatus();
    updatePhase();
}

void Backend::refreshLinkStatus()
{
    const bool wasOnline = m_robotOnline;
    m_robotOnline = m_hasRobotBeat && m_lastRobotBeat.isValid()
                    && m_lastRobotBeat.elapsed() < kRobotOnlineMs;
    if (m_testMode && !m_serverConnected)
        m_serverLabel = "테스트";
    else if (m_serverConnected)
        m_serverLabel = "연결됨";
    else if (m_serverLabel != "재연결…")
        m_serverLabel = m_userId.isEmpty() ? "대기" : "끊김";

    // POSE 가 끊기면 로봇 표시도 내린다.
    // 🔴 서버는 POSE 를 주기적으로 만들지 않는다(SRV-REP-QT-001 §2): CCTV 가 마커를
    //    놓친 프레임은 POSE 가 **아예 발행되지 않는다**. 예전에는 m_poseValid 를 로그아웃
    //    때만 껐기 때문에, 마커를 놓치면 로봇이 마지막 위치에 계속 붙어 있어서
    //    "그 자리에 멈춴 것"과 "위치를 못 받는 것"이 화면에서 구분되지 않았다.
    //
    // ⚠️ 판정 근거는 **m_robotOnline 이 아니라 POSE 자체의 수신 시각**이다.
    //    둘은 다른 사실이다 — 로봇이 접속 안 돼 있어도 마커만 보이면 POSE 는 오고,
    //    그때 위치 표시는 유효하다(로봇을 손으로 밀어도 화면에서 따라간다).
    //    예전처럼 m_robotOnline 에 물려두면 접속이 끊긴 순간 멀쩡히 들어오는 위치까지
    //    같이 지워버린다.
    const bool poseFresh = m_hasPoseBeat && m_lastPoseBeat.isValid()
                           && m_lastPoseBeat.elapsed() < kRobotOnlineMs;
    if (!poseFresh && m_poseValid && !m_testMode) {
        m_poseValid = false;
        pushPoseToView(false);   // 섀시·노즐 표시를 함께 내린다
        emit poseChanged();
        updatePhase();
        appendLog(QStringLiteral("로봇 위치 수신 끊김 (%1초) — 마커 검출 실패")
                      .arg(kRobotOnlineMs / 1000));
    }

    // 작업을 시작했는데 POSE 가 **한 번도** 안 오는 경우. 화면에는 "위치 수신 대기" 만
    // 뜨고 왜 안 오는지는 안 나와서, 로봇 아이콘이 안 보인다는 신고로만 들어온다.
    // 흔한 오해를 여기서 끊는다: CCTV 패널의 ArUco 표시는 **QT 로컬 검출**이라,
    // 거기 ID 49 가 보인다고 POSE 가 오는 게 아니다. POSE 는 다른 경로다.
    if (m_jobActive && !m_testMode && !m_poseEverSeen && !m_poseWaitWarned
        && m_jobElapsedValid && m_jobElapsed.elapsed() > kPoseWaitWarnMs) {
        m_poseWaitWarned = true;
        appendLog(QStringLiteral("⚠️ POSE 미수신 %1초 — 서버가 로봇 위치를 못 만들고 있습니다. "
                                 "경로: CCTV 마커 검출 → 서버 변환 → QT. "
                                 "왼쪽 영상의 ArUco 표시는 QT 자체 검출이라 이 경로와 무관합니다.")
                      .arg(kPoseWaitWarnMs / 1000));
    }

    // 라벨의 경과시간 표시가 계속 갱신돼야 해서 상태 변화가 없어도 매번 알린다.
    Q_UNUSED(wasOnline);
    emit linkStatusChanged();
}

void Backend::updatePhase()
{
    QString phase = "idle";
    QString hint = "작도를 시작하거나 프리셋을 놓으세요.";

    if (m_estopActive || m_robotState == "ESTOPPED") {
        phase = "paused_estop";
        hint = "비상정지 — 상단에서 비상 해제를 누르세요.";
    } else if (m_jobActive) {
        // 접근 → 도색은 서버가 이어서 하고 중간 통지가 없다.
        // 그래서 STATUS.painting 으로만 지금 칠하는 중인지 유추해 보여준다.
        phase = "running";
        if (!m_poseValid && !m_testMode)
            hint = "작업 중 — 로봇 위치 수신 대기";
        else if (m_painting || m_testMode)
            hint = QString("도색 진행 중 %1%").arg(jobPercent());
        else
            hint = "시작점으로 이동 중 — 도착하면 자동으로 도색이 시작됩니다.";
    } else if (m_blueprintSent && canEditMission()) {
        // "ready"(작도만 끝난 상태)와 구분한다 — 단계 표시가 거짓말을 하지 않도록
        phase = "sent";
        hint = "도면이 서버에 저장됨 — [그림그리기 시작]을 누르면 접근부터 도색까지 자동 진행됩니다.";
    } else if (m_phase == "done" && m_jobProgress >= 0.999 && canEditMission()) {
        phase = "done";
        hint = "완료되었습니다. 새 작업을 시작하세요.";
    } else if (m_drawing) {
        phase = "drawing";
        hint = pathPointCount() < 2
            ? "클릭으로 점을 찍으세요. (더블클릭=종료)"
            : "클릭=점 추가 · 더블클릭=작도 종료 · 그다음 작업 시작";
    } else if (pathPointCount() >= 2) {
        phase = "ready";
        hint = "경로 준비됨 — 오른쪽에서 [경로 전송]을 누르세요.";
    } else if (pathPointCount() == 1) {
        phase = "drawing";
        hint = "점이 1개입니다. 하나 더 찍으세요.";
    } else {
        phase = "idle";
        hint = "경로 추가 또는 프리셋으로 경로를 만드세요.";
    }

    // Keep done until cleared
    if (phase == "idle" && m_jobProgress >= 0.999 && !m_missionPaths.isEmpty() && !m_jobActive) {
        phase = "done";
        hint = m_phaseHint.isEmpty() ? "작업 완료. 새 작업을 시작하세요." : m_phaseHint;
    }

    if (phase != m_phase || hint != m_phaseHint) {
        m_phase = phase;
        m_phaseHint = hint;
        emit jobChanged();
    }
}

void Backend::startTestProgressSim()
{
    if (!m_testProgressTimer) return;

    // 서버로 보낸(=화면 시퀀스 미리보기와 같은) 프로그램을 그대로 재생한다.
    const QList<motionprogram::Op> prog = currentProgram();
    const QList<QPointF> pts = m_routePts;
    if (prog.isEmpty() || pts.size() < 2) {
        appendLog("[테스트] 재생할 동작 시퀀스가 없습니다.");
        return;
    }
    // 시퀀스가 **도면 그대로**(펜 기준)라 재생기도 노즐을 그대로 따라간다.
    // 오프셋은 로봇이 자기 안에서 흡수하므로 재생기는 관여하지 않는다.
    m_sim.load(prog, motionprogram::approachCenter(pts),
               motionprogram::approachHeadingDeg(pts));
    m_simPhase = m_sim.phaseText();
    m_simRunning = true;
    emit simChanged();

    pushSimPoseToView(m_sim.center(), m_sim.headingDeg(), false);
    appendLog(QString("[테스트] 동작 %1개 재생 — 예상 %2초 (%3배속)")
                  .arg(prog.size())
                  .arg(m_sim.remainingSec(), 0, 'f', 0)
                  .arg(m_simSpeedFactor, 0, 'g', 2));
    m_testProgressTimer->start();
}

void Backend::stopTestProgressSim()
{
    if (m_testProgressTimer)
        m_testProgressTimer->stop();
    if (m_topView)
        m_topView->setPenMarker(0, 0, false, false);
    if (m_simRunning) {
        m_simRunning = false;
        m_simPhase.clear();
        emit simChanged();
    }
}

// ⚠️ 여기 있던 setLensDistortion(k1,k2) 는 지웠다 — 캡처 프레임을 remap 으로 펴던
//    옛 2계수 경로다. 왜곡 보정 출처는 camcalib(K + 5계수) 하나로 통일했다.
//    (video_worker.h 상단 주석 참고)

void Backend::setSimSpeedFactor(double v)
{
    const double f = qBound(0.5, v, 20.0);
    if (qFuzzyCompare(m_simSpeedFactor, f)) return;
    m_simSpeedFactor = f;
    saveSettings();
    emit simChanged();
}

void Backend::sendRobotCmd(const QString &cmd, const QString &statusText)
{
    // 프로토콜 v0.3: 경로 실행 중 QT 수동조작은 서버가 무시한다.
    // 버튼만 안 먹는 것처럼 보이지 않도록 여기서 막고 이유를 알려준다.
    static const QStringList manualCmds{ "FORWARD", "BACKWARD", "TURN_LEFT", "TURN_RIGHT", "STOP" };
    if (manualCmds.contains(cmd) && !manualEnabled()) {
        setNotice(QStringLiteral("경로 실행 중에는 수동 조작이 차단됩니다. "
                                 "먼저 작업을 중단하세요."), QStringLiteral("warn"));
        return;
    }

    m_robotStatus = statusText;
    if (cmd == "ESTOP") {
        m_robotState = "ESTOPPED";
        m_estopActive = true;
    } else if (cmd == "RESUME") {
        m_robotState = "IDLE";
        m_estopActive = false;
        clearNotice(QStringLiteral("estop"));   // 풀렸으니 안내 배너도 같이 내린다
    } else if (cmd == "STOP") {
        m_robotState = "IDLE";
    } else if (cmd == "FORWARD" || cmd == "BACKWARD" || cmd == "TURN_LEFT" || cmd == "TURN_RIGHT") {
        m_robotState = "MOVING";
    }
    emit robotStatusChanged();
    if (m_client && !m_testMode) {
        m_client->sendCmd(cmd);
        appendLog(QString("CMD %1").arg(cmd));
    } else {
        appendLog(QString("[테스트] CMD %1").arg(cmd));
    }
    updatePhase();
}

// 노즐 올림/내림. 서버는 START_DRAW 를 제외한 CMD 를 그대로 ROBOT 에 중계하므로
// 프로토콜 변경 없이 이 경로로 나간다 (로봇 펌웨어가 NOZZLE_UP/DOWN 을 알아야 함).
// 도색 경로 실행 중에는 자동 제어와 충돌하니 수동 조작과 동일하게 잠근다.
void Backend::setNozzle(bool down)
{
    if (!manualEnabled()) {
        setNotice(QStringLiteral("경로 실행 중에는 노즐을 수동으로 조작할 수 없습니다. "
                                 "먼저 작업을 중단하세요."), QStringLiteral("warn"));
        return;
    }
    if (m_nozzleDown == down) return;
    m_nozzleDown = down;
    emit robotStatusChanged();
    const QString cmd = down ? QStringLiteral("NOZZLE_DOWN") : QStringLiteral("NOZZLE_UP");
    if (m_client && !m_testMode) {
        m_client->sendCmd(cmd);
        appendLog(QStringLiteral("CMD %1").arg(cmd));
    } else {
        appendLog(QStringLiteral("[테스트] CMD %1").arg(cmd));
    }
}

void Backend::toggleEstop()
{
    if (!m_estopActive)
        sendRobotCmd("ESTOP", "비상정지");
    else
        sendRobotCmd("RESUME", "재개");
}

void Backend::clearRobotLog()
{
    m_robotLog.clear(); emit robotLogChanged();
}

void Backend::appendLog(const QString &line)
{
    const QString ts = QDateTime::currentDateTime().toString("HH:mm:ss");
    if (!m_robotLog.isEmpty()) m_robotLog += '\n';
    m_robotLog += QString("[%1] %2").arg(ts, line);
    // cap log size
    const int maxLen = 8000;
    if (m_robotLog.size() > maxLen)
        m_robotLog = m_robotLog.right(maxLen);
    emit robotLogChanged();
}

void Backend::setRtsp(const QString &url)
{
    if (url.isEmpty() || url == m_rtspUrl) return;
    m_rtspUrl = url;
    emit rtspChanged();
    if (!m_worker) return;

    // ⚠️ 여기서 wait() 로 스레드를 기다리면 안 된다.
    // 캡처 스레드는 죽은 RTSP 주소에서 cap.read() 로 수십 초씩 막혀 있을 수 있고,
    // 그동안 GUI 스레드가 통째로 멈춰 "주소를 바꿨더니 앱이 굳는" 증상이 된다.
    // 종료만 요청해두고 스레드가 스스로 끝나면 정리되게 넘긴다.
    video_worker *old = m_worker;
    m_worker = nullptr;
    old->disconnect();                                   // 이 워커의 모든 신호 연결 해제
    connect(old, &QThread::finished, old, &QObject::deleteLater);
    old->stop();

    m_gotRealFrame = false;                              // 새 스트림 기준으로 다시 판단
    m_worker = new video_worker(m_rtspUrl, this);
    m_worker->setVideoFilters(m_brightness, m_contrast, m_sharpen, m_saturation);
    wireWorker(m_worker);
    m_worker->start();
    if (m_frameWatch) m_frameWatch->start();             // 새 주소도 안 붙으면 다시 대체 캔버스
    emit linkStatusChanged();
    saveSettings();                                      // 다음 실행에도 이 주소를 쓴다
    appendLog("RTSP 변경: " + url);
}

// 서버가 준 카메라 IP(LOGIN_OK.cam_ip)로 RTSP URL 을 조립한다. 조립 규칙은 QT 담당.
// 로컬 RTSP 만 바꾼다 (LOGIN_OK.cam_ip 로 받은 값을 반영할 때 사용).
void Backend::setCamIp(const QString &ip)
{
    const QString clean = ip.trimmed();
    if (clean.isEmpty()) return;
    m_camIp = clean;
    emit sessionChanged();
    QString url = m_rtspTemplate;
    url.replace(QStringLiteral("{ip}"), clean);
    appendLog(QStringLiteral("카메라 IP %1 → RTSP 조립").arg(clean));
    setRtsp(url);
}

// 설정 화면에서 사용자가 바꿀 때 — 로컬 반영 + 서버에 영속 저장(SET_CAM_IP).
// 서버는 형식 검증을 하지 않으므로 여기서 최소한의 형태만 확인한다.
void Backend::applyCamIp(const QString &ip)
{
    const QString clean = ip.trimmed();
    if (!clean.isEmpty()) {
        static const QRegularExpression re(
            QStringLiteral("^((25[0-5]|2[0-4]\\d|1?\\d?\\d)\\.){3}(25[0-5]|2[0-4]\\d|1?\\d?\\d)$"));
        if (!re.match(clean).hasMatch()) {
            setNotice(QStringLiteral("카메라 IP 형식이 올바르지 않습니다: %1").arg(clean),
                      QStringLiteral("error"));
            return;
        }
        setCamIp(clean);
    } else {
        m_camIp.clear();
        emit sessionChanged();
    }

    if (m_testMode || !m_serverConnected) {
        appendLog(QStringLiteral("[로컬] 카메라 IP %1 (서버 미연결 — SET_CAM_IP 생략)")
                      .arg(clean.isEmpty() ? QStringLiteral("해제") : clean));
        return;
    }
    m_client->sendSetCamIp(clean);
    appendLog(QStringLiteral("SET_CAM_IP %1 전송")
                  .arg(clean.isEmpty() ? QStringLiteral("(해제)") : clean));
}

void Backend::setVideoFilters(int b, int c, int s, int sat)
{
    m_brightness = b; m_contrast = c; m_sharpen = s; m_saturation = sat;
    if (m_worker) m_worker->setVideoFilters(b, c, s, sat);
}

// 설정창의 수동 붙여넣기. 파싱만 하고 실제 적용은 서버 경로와 똑같은 applyCalibObject 에 맡긴다.
// (테스트용 임시 경로다 — 서버 전달이 안정화되면 이 입력란만 지우면 된다)
QString Backend::applyCalibJson(const QString &jsonText)
{
    QJsonParseError err;
    const QJsonDocument doc = QJsonDocument::fromJson(jsonText.toUtf8(), &err);
    if (err.error != QJsonParseError::NoError || !doc.isObject())
        return QStringLiteral("JSON 파싱 실패: %1").arg(err.errorString());

    return applyCalibObject(doc.object(), QStringLiteral("수동 입력"));
}

// 표기 흔들림을 한곳에서 흡수한다. 서버/CCTV 가 어떤 형태로 주든 아래 로직은
// 항상 같은 모양의 객체만 보게 된다.
QJsonObject Backend::normalizeCalibObject(const QJsonObject &raw)
{
    QJsonObject obj = raw;

    // CCTV 번들은 {"calib": {...}} 로 한 겹 감싸 온다. 벗겨서 아래 로직이 그대로 보게 한다.
    if (obj.contains(QStringLiteral("calib")) && obj.value(QStringLiteral("calib")).isObject()) {
        QJsonObject inner = obj.value(QStringLiteral("calib")).toObject();
        for (auto it = obj.constBegin(); it != obj.constEnd(); ++it)
            if (it.key() != QLatin1String("calib") && !inner.contains(it.key()))
                inner.insert(it.key(), it.value());   // QML 이 붙인 corners/canvas_mm 보존
        obj = inner;
    }
    // 번들은 바닥 호모그래피를 H_floor 로 부른다. 기존 코드는 H 를 본다.
    // (H_marker 는 로봇 마커 평면용이라 작도에는 쓰지 않는다 — 작도는 바닥 평면이다)
    if (!obj.contains(QStringLiteral("H")) && obj.contains(QStringLiteral("H_floor")))
        obj.insert(QStringLiteral("H"), obj.value(QStringLiteral("H_floor")));

    // 코너가 루트에 따로 온 경우 corners 배열로 정규화
    if (!obj.contains(QStringLiteral("corners")) && obj.contains(QStringLiteral("c0"))) {
        QJsonArray arr;
        for (const char *k : { "c0", "c1", "c2", "c3" }) {
            if (!obj.contains(QLatin1String(k))) continue;
            QJsonObject c = obj.value(QLatin1String(k)).toObject();
            if (c.isEmpty()) {
                // { "c0": { "px":[], "mm":[] } } 형태가 아니면 스킵
                continue;
            }
            c.insert(QStringLiteral("id"), QLatin1String(k));
            arr.append(c);
        }
        if (!arr.isEmpty())
            obj.insert(QStringLiteral("corners"), arr);
    }
    if (!obj.contains(QStringLiteral("unit")))
        obj.insert(QStringLiteral("unit"), QStringLiteral("mm"));

    // coord_mode 는 H 의 **입력 픽셀 공간**을 뜻한다 (QT-REQ-CCTV-001 R-2).
    // camelCase 로 올 수도 있어 snake_case 로 통일해 둔다.
    if (!obj.contains(QStringLiteral("coord_mode")) && obj.contains(QStringLiteral("coordMode")))
        obj.insert(QStringLiteral("coord_mode"), obj.value(QStringLiteral("coordMode")));

    return obj;
}

// 번들이 어디서 들어오든(수동 입력 · LOGIN_OK · H_MATRIX) 여기 한 곳만 지난다.
QString Backend::applyCalibObject(const QJsonObject &raw, const QString &source)
{
    const QJsonObject obj = normalizeCalibObject(raw);
    if (obj.isEmpty())
        return QStringLiteral("빈 캘리브레이션입니다.");

    m_calibSource = source;
    m_calibId     = obj.value(QStringLiteral("calib_id")).toString();
    m_coordMode   = obj.value(QStringLiteral("coord_mode")).toString().toLower();

    // ⚠️ 예전에는 여기서 최상위 k1/k2 를 읽어 캡처 프레임을 통째로 폈다.
    //    지금은 무시한다 — 왜곡 보정은 아래 K + dist(plumb-bob 5계수) 하나만 쓴다.
    //    (경로가 둘이면 어느 쪽이 켜졌는지에 따라 같은 번들에서 좌표가 달라진다)
    if (obj.contains(QStringLiteral("k1")) && !obj.contains(QStringLiteral("K")))
        appendLog(QStringLiteral("⚠️ 번들에 k1/k2 만 있고 K·dist 가 없습니다 — "
                                 "렌즈 보정을 적용할 수 없습니다"));

    // CCTV 가 주는 정식 내부 파라미터 (K + plumb-bob 5계수).
    if (obj.contains(QStringLiteral("K"))) {
        QString cerr;
        const camcalib::Model m = camcalib::parse(obj, &cerr);
        if (m.valid) {
            m_cam = m;
            if (m_topView) m_topView->setLensModel(m_cam);
            emit lensChanged();
            appendLog(QStringLiteral("렌즈 파라미터 적용: ") + camcalib::describe(m));
            // K 와 H 는 이 해상도에서 뽑은 값이다. 프레임 크기가 다르면 좌표가 통째로
            // 어긋나는데, 크롭인지 스케일인지 구분할 수 없어 비례 보정은 오히려 위험하다.
            // (CCTV 권고) 그래서 보정하지 않고 경고만 남긴다.
            if (m.imgW > 0 && m_frameW > 0 && (m.imgW != m_frameW || m.imgH != m_frameH)) {
                appendLog(QStringLiteral("⚠️ 캘리브 해상도 %1×%2 ≠ 영상 %3×%4 — 좌표가 맞지 않습니다")
                              .arg(m.imgW).arg(m.imgH).arg(m_frameW).arg(m_frameH));
                setNotice(QStringLiteral("캘리브레이션 해상도와 카메라 영상 해상도가 다릅니다. "
                                         "카메라 프로파일을 %1×%2 로 되돌리세요.")
                              .arg(m.imgW).arg(m.imgH),
                          QStringLiteral("warn"), QStringLiteral("calibsize"));
            } else {
                clearNotice(QStringLiteral("calibsize"));
            }
        } else {
            appendLog(QStringLiteral("렌즈 파라미터 무시 — ") + cerr);
        }
    }

    // coord_mode 가 H 의 입력 픽셀 공간을 알려준다. 사람이 토글을 잘못 두면 좌표가
    // 통째로 어긋나므로, 번들이 명시했으면 그 말을 따른다.
    //   undistort → H 는 왜곡보정 픽셀을 받는다  ⇒ 배경도 보정해야 격자와 맞는다 (ON)
    //   raw       → H 는 원본 픽셀을 받는다      ⇒ 보정하면 오히려 어긋난다     (OFF)
    if (!m_coordMode.isEmpty()) {
        const bool wantLens = (m_coordMode == QLatin1String("undistort"));
        if (!wantLens && m_coordMode != QLatin1String("raw")
                      && m_coordMode != QLatin1String("distorted")) {
            appendLog(QStringLiteral("⚠️ 알 수 없는 coord_mode \"%1\" — 렌즈보정은 그대로 둡니다")
                          .arg(m_coordMode));
        } else if (m_lensOn != wantLens) {
            setLensCorrection(wantLens);
            appendLog(QStringLiteral("coord_mode=%1 → 렌즈보정 자동 %2")
                          .arg(m_coordMode, wantLens ? QStringLiteral("ON") : QStringLiteral("OFF")));
        }
    } else if (obj.contains(QStringLiteral("H"))) {
        appendLog(QStringLiteral("⚠️ coord_mode 없음 — H 의 픽셀 공간을 알 수 없어 "
                                 "렌즈보정을 수동 토글에 맡깁니다 (현재 %1)")
                      .arg(m_lensOn ? QStringLiteral("ON") : QStringLiteral("OFF")));
    }

    // TopView 가 아직 없어도 번들은 보관한다. configureView() 가 만들어질 때 이 값으로 구성된다.
    m_calib = obj;
    if (!m_topView) {
        m_calibMissing = false;
        emit calibChanged();
        appendLog(QStringLiteral("캘리브 수신(%1) — TopView 생성 시 적용됩니다").arg(source));
        return QStringLiteral("보관됨 — 화면 준비 후 적용");
    }

    const bool ok = m_topView->configureTopViewCalib(obj);
    m_calibMissing = false;
    refreshCalibStatus();
    pushMissionToView();
    pushOverlayToOriginal();   // 좌표 변환이 바뀌었으니 원본 뷰 오버레이도 다시 만든다
    appendLog(ok ? QStringLiteral("캘리브 적용(%1): ").arg(source) + m_calibStatus
                 : QStringLiteral("캘리브 실패(%1) — 테스트 보정으로 폴백").arg(source));
    logCalibUnit(m_topView);
    return ok ? (QStringLiteral("적용됨 · ") + m_calibStatus)
              : QStringLiteral("적용 실패 (테스트 보정 사용)");
}

// 곡선을 ARC op 으로 보낼지.
// ⚠️ 여기 있던 setArcEnabled / m_arcEnabled 는 지웠다.
//    곡선은 **항상 ARC op** 으로 보낸다 — 켜고 끌 대상이 아니다.
//      · 동작 수가 4~10배 줄어든다 (O: 49 → 4, STOP: 157 → 25).
//      · 로봇에 원호 주행 코드가 **이미 있고 실측 검증됐다** (arc_test.cpp):
//          R_robot = √(R_paint² + 0.155²)                  ← 노즐 오프셋 보정
//          r_left/right = R_robot ∓ 0.364/2                ← 트랙 폭
//          목표 스텝 = |r_wheel · θ| · 15433.09 pulses/m
//          바퀴 속도 = 771.65 · (r_wheel / R_robot)         ← 동시 도착
//    남은 작업은 로봇 main.cpp 의 op 분기에 이 로직을 연결하는 것뿐이다.

// TopView 배경에 렌즈 보정을 적용할지 토글한다.
// 끄면 예전 동작(호모그래피만)이라, 같은 화면에서 켜고 끄며 어느 쪽이 잘 펴지는지
// 눈으로 바로 비교할 수 있다. 원본 영상과 좌표 계산에는 영향이 없다 — TopView 배경뿐이다.
void Backend::setLensCorrection(bool on)
{
    if (m_lensOn == on) return;
    m_lensOn = on;
    if (m_topView) m_topView->setLensCorrection(on);
    if (on && !lensReady())
        appendLog(QStringLiteral("렌즈 보정 ON — 다만 K/왜곡계수가 아직 없어 화면은 그대로입니다"));
    else
        appendLog(on ? QStringLiteral("렌즈 보정 ON — TopView 배경을 remap 으로 펍니다")
                     : QStringLiteral("렌즈 보정 OFF — 호모그래피만 사용 (예전 동작)"));
    emit lensChanged();
    saveSettings();
}

// 로봇 아이콘을 켜고 끈다. 265mm 기체를 실축으로 그리면 900×600 도면의 1/4 을 덮어서,
// 로봇이 도면 한가운데 서 있으면 그리던 선이 통째로 가려진다.
// ⚠️ 표시만 끈다 — POSE 수신·진행률·시작점 판정은 그대로 돈다.
void Backend::setRobotVisible(bool on)
{
    if (m_robotVisible == on) return;
    m_robotVisible = on;
    if (m_topView) m_topView->setRobotVisible(on);
    appendLog(on ? QStringLiteral("[표시] 로봇 아이콘 ON")
                 : QStringLiteral("[표시] 로봇 아이콘 OFF — 위치 수신은 계속됩니다"));
    emit robotVisibleChanged();
    saveSettings();
}

void Backend::addWorldPointMm(double xMm, double yMm)
{
    if (!m_topView || m_jobActive) return;
    if (m_phase == QLatin1String("done"))
        clearMission();
    // 새 경로일 때만 startDraw(클리어). 기존 점이 있으면 이어서 추가.
    if (!m_drawing && m_topView->currentPoints().isEmpty()) {
        m_drawing = true;
        emit drawingChanged();
        m_topView->startDraw();
    } else if (!m_drawing) {
        m_drawing = true;
        emit drawingChanged();
    }
    m_topView->appendWorldPointMm(xMm, yMm);
    appendLog(QStringLiteral("월드점 추가: (%1, %2) mm")
                  .arg(xMm, 0, 'f', 1).arg(yMm, 0, 'f', 1));
    updatePhase();
}

void Backend::addRectWorldMm(double widthMm, double heightMm)
{
    if (!m_topView || m_jobActive) return;
    if (widthMm < 1.0 || heightMm < 1.0) {
        appendLog(QStringLiteral("사각 mm: 가로/세로를 1 이상 입력하세요."));
        return;
    }
    if (m_phase == QLatin1String("done"))
        clearMission();
    m_topView->addRectWorldMm(widthMm, heightMm);
    m_drawing = false;
    emit drawingChanged();
    appendLog(QStringLiteral("사각 %1 × %2 mm").arg(widthMm, 0, 'f', 0).arg(heightMm, 0, 'f', 0));
    updatePhase();
}

// 글자 → 획(중심선) 경로. outline=true 면 외곽선.
void Backend::addTextWorldMm(const QString &text, double heightMm, bool outline)
{
    if (!m_topView || m_jobActive) return;
    const QString t = text.trimmed();
    if (t.isEmpty()) { appendLog(QStringLiteral("넣을 글자를 입력하세요.")); return; }
    if (m_phase == QLatin1String("done"))
        clearMission();

    const int before = m_topView->shapeCount();
    m_topView->addTextWorld(t, heightMm, outline);
    const int made = m_topView->shapeCount() - before;
    m_drawing = false;
    emit drawingChanged();
    appendLog(QStringLiteral("글자 '%1' · 높이 %2 mm — %3 %4획")
                  .arg(t).arg(heightMm, 0, 'f', 0)
                  .arg(outline ? QStringLiteral("외곽선") : QStringLiteral("중심선(획)"))
                  .arg(std::max(0, made)));

    // 글자 높이가 붓 폭보다 너무 작으면 뭉개진다 — 미리 알려준다
    if (heightMm < m_strokeWidthMm * 2.5) {
        setNotice(QStringLiteral("글자 높이 %1 mm 는 붓 폭 %2 mm 에 비해 작아 뭉개질 수 있습니다. "
                                 "%3 mm 이상을 권장합니다.")
                      .arg(heightMm, 0, 'f', 0)
                      .arg(m_strokeWidthMm, 0, 'f', 0)
                      .arg(m_strokeWidthMm * 2.5, 0, 'f', 0),
                  QStringLiteral("warn"));
    }
    updatePhase();
}

// 거의 일직선인 점 제거 → 로봇이 "직진 + 회전"만 하게
void Backend::simplifyPaths(double toleranceMm)
{
    if (!m_topView || m_jobActive) return;
    const int removed = m_topView->simplifyAll(toleranceMm);
    appendLog(removed > 0
              ? QStringLiteral("경로 단순화 — 점 %1개 제거 (허용오차 %2 mm)")
                    .arg(removed).arg(toleranceMm, 0, 'f', 0)
              : QStringLiteral("경로 단순화 — 지울 점이 없습니다"));
}

void Backend::rotateShape(double deg)
{
    if (!m_topView || m_jobActive) return;
    m_topView->rotateActive(deg);
}

void Backend::flipShape(bool horizontal)
{
    if (!m_topView || m_jobActive) return;
    m_topView->flipActive(horizontal);
    appendLog(horizontal ? QStringLiteral("도형 좌우 반전") : QStringLiteral("도형 상하 반전"));
}

void Backend::scaleShape(double factor)
{
    if (!m_topView || m_jobActive) return;
    m_topView->scaleActive(factor);
}

void Backend::setStrokeWidthMm(double mm)
{
    const double v = qBound(1.0, mm, 500.0);
    if (qFuzzyCompare(m_strokeWidthMm, v)) return;
    m_strokeWidthMm = v;
    if (m_topView) m_topView->setStrokeWidthMm(v);
    if (m_originalView) m_originalView->setStrokeWidthMm(v);
    emit strokeWidthChanged();
    appendLog(QStringLiteral("도장 폭 %1 mm 적용").arg(v, 0, 'f', 0));
}

// 다시 켰을 때 그대로여야 하는 값만 파일에 남긴다.
// (나머지 설정은 세션 한정이라 저장하지 않는다)
void Backend::loadSettings()
{
    QSettings s;
    m_speeds.travelMps = qBound(0.01, s.value("robot/travelMps", 0.20).toDouble(), 2.0);
    m_speeds.paintMps  = qBound(0.01, s.value("robot/paintMps",  0.10).toDouble(), 2.0);
    m_speeds.turnDps   = qBound(1.0,  s.value("robot/turnDps",   30.0).toDouble(), 360.0);
    m_simSpeedFactor   = qBound(0.5,  s.value("ui/simSpeed",      4.0).toDouble(), 20.0);
    m_lensOn = s.value("camera/lensCorrection", false).toBool();
    m_robotVisible = s.value("ui/robotVisible", true).toBool();

    // 마지막으로 실제로 붙었던 카메라 주소. 이게 없으면 앱이 매번 하드코딩 기본값으로
    // 되돌아가는데, 그 IP 에 카메라가 없으면 "왜 영상이 안 나오지" 로만 보인다.
    // 서버가 LOGIN_OK.cam_ip 를 주면 그 값이 이걸 덮는다.
    const QString savedUrl = s.value("camera/rtspUrl").toString().trimmed();
    if (!savedUrl.isEmpty()) m_rtspUrl = savedUrl;
    const QString savedIp = s.value("camera/ip").toString().trimmed();
    if (!savedIp.isEmpty()) m_camIp = savedIp;
}

void Backend::saveSettings() const
{
    QSettings s;
    s.setValue("robot/travelMps", m_speeds.travelMps);
    s.setValue("robot/paintMps", m_speeds.paintMps);
    s.setValue("robot/turnDps", m_speeds.turnDps);
    s.setValue("ui/simSpeed", m_simSpeedFactor);
    s.setValue("camera/lensCorrection", m_lensOn);
    s.setValue("ui/robotVisible", m_robotVisible);
    s.setValue("camera/rtspUrl", m_rtspUrl);
    s.setValue("camera/ip", m_camIp);
}

// ── 로봇 속도 ────────────────────────────────────────────────────────
// 도색 속도와 이동 속도는 다르다: 칠할 때는 도료가 고르게 깔려야 해서 느리고,
// 도형 사이를 그냥 지나갈 때는 빠르게 가도 된다. 그래서 MOVE 마다 속도를 실어 보낸다.
void Backend::setTravelSpeedMps(double v)
{
    const double x = qBound(0.01, v, 2.0);
    if (qFuzzyCompare(m_speeds.travelMps, x)) return;
    m_speeds.travelMps = x;
    saveSettings();
    emit speedChanged();
    emit jobChanged();          // 시퀀스·예상 시간이 바뀐다
    appendLog(QStringLiteral("이동 속도 %1 m/s").arg(x, 0, 'f', 2));
    pushSpeeds();
}

void Backend::setPaintSpeedMps(double v)
{
    const double x = qBound(0.01, v, 2.0);
    if (qFuzzyCompare(m_speeds.paintMps, x)) return;
    m_speeds.paintMps = x;
    saveSettings();
    emit speedChanged();
    emit jobChanged();
    appendLog(QStringLiteral("도색 속도 %1 m/s").arg(x, 0, 'f', 2));
    pushSpeeds();
}

void Backend::setTurnSpeedDps(double v)
{
    const double x = qBound(1.0, v, 360.0);
    if (qFuzzyCompare(m_speeds.turnDps, x)) return;
    m_speeds.turnDps = x;
    saveSettings();
    emit speedChanged();
    emit jobChanged();
    appendLog(QStringLiteral("회전 속도 %1 °/s").arg(x, 0, 'f', 0));
    pushSpeeds();
}

// 🔴 SET_SPEED 는 더 이상 보내지 않는다 (2026-07-28 프로토콜 확정).
//
//   "속도는 프로토콜에 없다 — 주행/도색/회전 속도는 전부 로봇 펌웨어 고정값을
//    쓴다. op별 speed_mps/speed_dps 도, 이를 바꾸는 CMD 도 만들지 않기로 확정했다."
//
//   실제로 서버 router.cpp 는 SET_SPEED 를 그냥 ROBOT 에 중계하고, 로봇
//   NetworkManager.cpp 에는 이 cmd 분기가 아예 없어서 조용히 버려진다.
//   보내봐야 아무 일도 안 일어나고 로그만 어지럽혀서 전송을 끊었다.
//
//   설정값(m_speeds)은 남긴다 — 화면 미리보기 재생 속도와 예상 소요시간
//   계산에 쓰이는 **로컬 값**이다.
void Backend::pushSpeeds()
{
    appendLog(QStringLiteral("속도 설정(로컬) — 이동 %1 · 도색 %2 m/s · 회전 %3 °/s "
                             "· 실제 주행 속도는 로봇 펌웨어 고정값")
                  .arg(m_speeds.travelMps, 0, 'f', 2)
                  .arg(m_speeds.paintMps, 0, 'f', 2)
                  .arg(m_speeds.turnDps, 0, 'f', 0));
}

// 설정한 속도로 계산한 이번 도면의 예상 소요 시간 (작업 전 미리보기용).
QString Backend::planTimeText() const
{
    const QList<motionprogram::Op> prog = currentProgram();
    if (prog.isEmpty()) return QStringLiteral("—");
    const int sec = int(motionprogram::totalSeconds(prog) + 0.5);
    return QStringLiteral("%1분 %2초").arg(sec / 60).arg(sec % 60);
}

// ⚠️ 여기 있던 setPenOffsetMm / setPenOffsetFromServer 는 지웠다.
//    program 이 펜 프레임 그대로 나가게 되면서(2026-07-28 프로토콜, pen_offset_m 폐지)
//    이 값은 전송에도, 시퀀스 생성에도, 시뮬레이션에도 쓰이지 않는 순수 잔재였다.
//    펜 오프셋 보정은 로봇 전담이다.
