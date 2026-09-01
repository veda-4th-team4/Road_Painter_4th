#include "backend.h"
#include "videoview.h"
#include "video_worker.h"
#include "preview_worker.h"
#include "channel_tile.h"
#include "serverclient.h"
#include "routeplan.h"
#include "paintgeometry.h"
#include "paintprogress.h"
#include "paintdimensions.h"
#include "camcreds.h"
#include "robottiming.h"

#include <QTimer>
#include <QPointF>
#include <QLineF>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QJsonArray>
#include <QtMath>
#include <cmath>
#include <limits>
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
constexpr auto kDefaultFourChannelCameraIp = "192.168.0.13";
constexpr int kFourChannelCount = 4;
// 작업 시작 후 이 시간까지 POSE 가 한 번도 없으면 한 번만 경고를 남긴다.
constexpr qint64 kPoseWaitWarnMs = 5000;
// 테스트 시뮬레이션 틱. 40ms = 25fps — 4cm 후진처럼 짧은 동작도 여러 프레임에 걸친다.
constexpr int kSimTickMs = 40;
// 캘리브레이션 대기 한도. 정적 앵커는 기존 5분 유지, 오도메트리 주행(robot_motion)은
// 주행 2~4분 + 카메라 H 계산까지 들어와야 해서 10분으로 늘렸다 (요청서 §4 안 A).
constexpr int kStaticHomographyTimeoutMs = 5 * 60 * 1000;
constexpr int kOdoHomographyTimeoutMs   = 10 * 60 * 1000;
// 오도메트리 사각형의 정지점 수(서버 total=9 고정). 서버가 아직 total 을 안 보낸
// 첫 화면에서도 "9개 중" 표기를 유지하기 위한 기본값일 뿐, 서버 값이 오면 덮인다.
constexpr int kOdoStopPoints = 9;
// 중단 요청 뒤 종결 응답(CALIB_CANCELLED/FAIL)을 기다리는 로컬 감시 시간.
// 🔴 이 시간이 지나도 **취소 성공으로 처리하지 않는다** — 로봇이 실제로 섰다는
//    확인은 서버만 줄 수 있다. "정지 확인 실패"만 알리고 재시도를 허용한다.
constexpr int kCancelConfirmWatchdogMs = 15 * 1000;
// 통지 키를 분리한다 — 캡처 지연 회복이 중단 경고를 지우면 안 된다.
const QString kNoticeCaptureLag = QStringLiteral("calib-lag");
const QString kNoticeCancel     = QStringLiteral("calib-cancel");

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

// 표시 기하 기본값(펜 폭·펜 오프셋)의 세대 번호. loadSettings 가 이 값보다 낮은
// 설정을 만나면 옛 기본값만 골라 현재 실측값으로 이관하고, saveSettings 가 같은
// 값을 다시 기록한다.
// 🔴 두 곳이 같은 상수를 봐야 한다 — 숫자를 직접 쓰면 다음에 세대를 올렸을 때
//    save 가 옛 번호를 남겨 마이그레이션이 매 실행마다 다시 돈다.
constexpr int kGeometryDefaultsVersion = 2;

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
        if (m_homographyPending)
            failHomography(QStringLiteral("서버 연결이 끊겨 호모그래피 진행 상태를 확인할 수 없습니다."),
                            QStringLiteral("server_disconnected"));
        if (m_abortPending) {
            m_abortPending = false;
            setNotice(QStringLiteral("서버 연결이 끊겨 작업 취소 여부를 확인하지 못했습니다."),
                      QStringLiteral("error"), QStringLiteral("abort"));
            emit jobChanged();
        }
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
        // START_DRAW는 서버가 program까지 정상 접수했다는 ACK 뒤에만 허용한다.
        // 전송 직후 true로 두면 arc_too_tight/bad_program 응답과 경합해 빈 도면을
        // 시작할 수 있다.
        m_blueprintSent = match;
        emit jobChanged();
        updatePhase();
    });
    connect(m_client, &ServerClient::hMatrixReceived, this,
            [this](int rawCh, const QJsonObject &calib, const QString &requestId) {
        if (m_testMode) return;
        const int ch = camcalib::resolveHomographyChannel(
            rawCh, requestId, m_homographyPending, m_homographyCh,
            m_homographyRequestId, m_channelCount);
        if (ch < 1 || ch > m_channelCount) {
            appendLog(QStringLiteral("H_MATRIX 수신 — 채널을 확정할 수 없어 무시합니다 "
                                     "(ch=%1, request_id=%2, pending=%3)")
                          .arg(rawCh)
                          .arg(requestId.isEmpty() ? QStringLiteral("없음") : requestId)
                          .arg(m_homographyPending ? QStringLiteral("CH%1").arg(m_homographyCh)
                                                   : QStringLiteral("없음")));
            return;
        }
        // 내 요청의 실패로 인정하려면 결과가 명시적으로 내 것임을 확인한다.
        // 채널과 request_id가 모두 없는 외부 푸시는 pending 채널에 귀속될 수 있어도
        // 사용자가 시작한 계산을 실패시키면 안 된다.
        const bool attributable = (rawCh >= 1)
            || (!requestId.isEmpty() && requestId == m_homographyRequestId);
        if (calib.isEmpty()) {
            appendLog(QStringLiteral("H_MATRIX 수신 (CH%1) — calib=null (서버에 보관된 번들 없음)").arg(ch));
            if (attributable && matchesHomographyReply(ch, requestId))
                failHomography(QStringLiteral("서버가 빈 호모그래피 결과를 보냈습니다."),
                               QStringLiteral("invalid_result"));
            return;
        }
        // 🔴 구조 검증은 **대기 중이던 응답인지와 무관하게** 먼저 한다.
        //    예전에는 이 검사가 matchesHomographyReply 블록 안에 있어서, 요청하지
        //    않은/늦게 온/깨진 번들이 검증 없이 m_calibs 에 저장되고 화면 채널이면
        //    그대로 적용됐다 — 좌표계가 조용히 갈아엎힌다.
        if (!calibHasUsableH(calib)) {
            appendLog(QStringLiteral("H_MATRIX 수신 (CH%1) — 유효한 3x3 H 가 없어 버립니다").arg(ch));
            if (attributable && matchesHomographyReply(ch, requestId))
                failHomography(QStringLiteral("서버 결과에 유효한 3x3 H 행렬이 없습니다."),
                               QStringLiteral("invalid_result"));
            return;
        }
        // 🔴 중단을 요청한 세션의 결과는 내 요청의 답이라도 통째로 버린다
        //    (확인 대기 중 · 확인 실패 둘 다) — 저장·적용·완료·화면전환 모두 없음.
        const camcalib::ReplyUse use = camcalib::afterCancelRequest(
            camcalib::classifyHomographyReply(ch, requestId, m_homographyPending,
                                              m_homographyCh, m_homographyRequestId),
            m_homographyCancelRequested);
        if (use == camcalib::ReplyUse::Drop) {
            appendLog(m_homographyCancelRequested
                ? QStringLiteral("H_MATRIX 수신 (CH%1) — 중단 요청된 세션의 결과라 폐기: %2")
                      .arg(ch).arg(requestId.isEmpty() ? QStringLiteral("없음") : requestId)
                : QStringLiteral("H_MATRIX 수신 (CH%1) — 늦은 Qt 요청 결과 무시: %2")
                      .arg(ch).arg(requestId));
            return;
        }
        if (use == camcalib::ReplyUse::StoreOnly) {
            // 외부에서 시작한 결과라도 지금 작업 화면과 같은 채널이면 즉시 TopView에
            // 적용한다. 다른 채널 결과는 화면 좌표계를 바꾸지 않고 채널 맵에만 저장한다.
            bool appliedNow = false;
            QString applied;
            if (camcalib::appliesToCurrentView(ch, m_workingCh)) {
                bool ok = false;
                applied = applyCalibObject(
                    calib, QStringLiteral("H_MATRIX 외부 CH%1").arg(ch), &ok);
                if (!ok) {
                    appendLog(QStringLiteral("H_MATRIX 외부 결과 (CH%1) — TopView 적용 실패: %2")
                                  .arg(ch).arg(applied));
                    setNotice(QStringLiteral("CH%1의 새 호모그래피를 화면에 적용하지 못했습니다.")
                                  .arg(ch),
                              QStringLiteral("error"), QStringLiteral("homography"));
                    return;
                }
                appliedNow = true;
                clearNotice(QStringLiteral("chcalib"));
            }
            m_calibs[QString::number(ch)] = calib;
            updateLensDataWarning(ch, calib);
            emit channelChanged();
            appendLog(appliedNow
                ? QStringLiteral("H_MATRIX 외부 결과 (CH%1) — 현재 TopView 즉시 적용: %2")
                      .arg(ch).arg(applied)
                : QStringLiteral("H_MATRIX 외부 결과 (CH%1) — 채널별 보관 (현재 화면 CH%2)")
                      .arg(ch).arg(m_workingCh));
            return;
        }

        // Apply = 지금 기다리던 결과. 구조가 유효해도 TopView 구성이 실패하면
        // 완료로 표시하거나 채널의 정상 번들을 덮어쓰지 않는다.
        if (use == camcalib::ReplyUse::Apply && matchesHomographyReply(ch, requestId)) {
            bool ok = false;
            const QString applied = applyCalibObject(
                calib, QStringLiteral("H_MATRIX CH%1").arg(ch), &ok);
            if (!ok) {
                failHomography(QStringLiteral("서버 호모그래피를 TopView에 적용하지 못했습니다."),
                               QStringLiteral("apply_failed"));
                return;
            }
            m_calibs[QString::number(ch)] = calib;
            emit channelChanged();
            appendLog(QStringLiteral("CH%1 호모그래피 완료 — %2").arg(ch).arg(applied));
            resetHomography();
            m_highlightedCh = ch;
            emit channelChanged();
            setNotice(QStringLiteral("CH%1 호모그래피를 적용했습니다. 작업 화면으로 전환합니다.").arg(ch),
                      QStringLiteral("info"), QStringLiteral("homography"));
            // 성공 안내 **뒤에** 검사한다 — 왜곡 보정 데이터가 없으면 그 경고가
            // 마지막에 남아야 조작자가 놓치지 않는다.
            updateLensDataWarning(ch, calib);
            startChannelWork();
            return;
        }
    });
    connect(m_client, &ServerClient::calibStarted, this,
            [this](int ch, const QString &requestId, const QString &message) {
        if (!matchesHomographyReply(ch, requestId)) return;
        if (m_homographyOdometry && m_homographyPhase == QLatin1String("requesting"))
            m_homographyPhase = QStringLiteral("driving");
        m_homographyStatus = message.isEmpty()
            ? (m_homographyOdometry
                   ? QStringLiteral("로봇이 사각형 주행을 시작했습니다.")
                   : QStringLiteral("로봇 이동과 영상 계산을 시작했습니다."))
            : message;
        emit homographyChanged();
        appendLog(QStringLiteral("CALIB_STARTED CH%1 request=%2").arg(ch).arg(requestId));
    });
    connect(m_client, &ServerClient::calibProgress, this,
            [this](int ch, const QString &requestId, double progress,
                   const QString &stage, const QString &message,
                   const QString &phase, int pointIndex, int total, int valid) {
        if (!matchesHomographyReply(ch, requestId)) return;
        // 오도메트리 주행은 카메라가 진행률을 안 주고 서버가 정지점 진행으로
        // 합성해 보낸다 (요청서 §2-2). 9개 중 몇 번째인지 = 진행률.
        m_homographyPointIndex = pointIndex;
        if (total > 0) m_homographyPointTotal = total;
        m_homographyValidPoints = valid;
        if (!phase.isEmpty()) m_homographyPhase = phase;
        else if (m_homographyOdometry && m_homographyPhase.isEmpty())
            m_homographyPhase = QStringLiteral("driving");
        if (progress < 0.0 && pointIndex >= 0 && total > 0)
            progress = double(pointIndex + 1) / double(total);
        m_homographyProgress = progress < 0.0 ? -1.0 : qBound(0.0, progress, 1.0);
        // 🔴 중단을 요청한 세션에서는 진행 값만 갱신하고 **상태 문구와 통지는
        //    건드리지 않는다.** 늦게 온 진행 보고가 "정지 확인 대기/실패"를 덮으면
        //    취소를 누른 조작자에게 정상 진행 중인 화면이 보인다. 취소 상태는
        //    일치하는 종결 응답이나 resetHomography() 만 바꿀 수 있다.
        if (m_homographyCancelRequested) {
            emit homographyChanged();
            return;
        }
        if (!message.isEmpty()) m_homographyStatus = message;
        else if (phase == QLatin1String("solving"))
            m_homographyStatus = QStringLiteral("주행 완료 — 카메라가 호모그래피를 계산 중입니다.");
        else if (pointIndex >= 0 && total > 0)
            m_homographyStatus = QStringLiteral("정지점 %1/%2 캡처 완료 (유효 %3개)")
                                     .arg(pointIndex + 1).arg(total).arg(qMax(valid, 0));
        else if (!stage.isEmpty()) m_homographyStatus = stage;
        // 유효 대응점이 완료 정지점보다 2 이상 뒤처지면 카메라가 로봇을 놓치고 있다.
        // 6개 미만으로 끝나면 세션이 실패하므로 주행 중에 알려서 중단할 수 있게 한다.
        // 판정은 homographyCaptureLag() 와 동일한 순수 함수를 쓴다 (QML 경고색과 일치).
        // 🔴 이 경고는 **자기 키만** 세우고 내린다. 예전처럼 "homography" 키를
        //    통째로 지우면 늦게 온 진행 보고 하나가 "정지 확인 실패" 경고까지
        //    지워서, 취소를 요청한 세션이 정상 진행 중인 것처럼 보인다.
        if (homographyCaptureLag())
            setNotice(QStringLiteral("카메라가 로봇을 자주 놓치고 있습니다 (정지점 %1개 완료 · "
                                     "유효 %2개). 조명과 사각형 크기를 확인하고, 필요하면 "
                                     "중단하세요.")
                          .arg(pointIndex + 1).arg(valid),
                      QStringLiteral("warn"), kNoticeCaptureLag);
        else
            clearNotice(kNoticeCaptureLag);
        emit homographyChanged();
    });
    connect(m_client, &ServerClient::calibFailed, this,
            [this](int ch, const QString &requestId, const QString &reason,
                   const QString &message, const QString &owner) {
        if (!matchesHomographyReply(ch, requestId)) return;
        failHomography(camcalib::homographyFailText(reason, message, owner), reason);
    });
    connect(m_client, &ServerClient::calibCancelled, this,
            [this](int ch, const QString &requestId, const QString &message) {
        // 🔴 방어(요청서 §6 ③): 대기 중이 아닐 때, 또는 내 request_id 와 다른
        //    종결 응답은 무시한다. 관리자 창이 세션 없이 누른 CALIB_CANCEL 이
        //    QT 로 새어 나오는 서버 결함이 지금도 재현된다.
        if (!m_homographyPending) return;
        if (requestId.isEmpty() || requestId != m_homographyRequestId) {
            appendLog(QStringLiteral("CALIB_CANCELLED 무시 — 내 요청(%1)과 다른 request_id(%2)")
                          .arg(m_homographyRequestId,
                               requestId.isEmpty() ? QStringLiteral("없음") : requestId));
            return;
        }
        if (!matchesHomographyReply(ch, requestId)) return;
        const QString text = message.isEmpty()
            ? QStringLiteral("CH%1 호모그래피가 중단되었습니다.").arg(ch) : message;
        resetHomography();
        setNotice(text, QStringLiteral("warn"), QStringLiteral("homography"));
        appendLog(text);
    });
    // LOGIN_OK의 CH1~CH4 캘리브레이션 번들 맵.
    connect(m_client, &ServerClient::calibChannelsReceived, this,
            [this](const QJsonObject &calibs, int activeCh) {
        // 저장 전에 채널별로 검증한다. 깨진 항목 하나가 그 채널을 "캘리 됨"으로
        // 만들면 좌표를 믿을 수 없는 채널에서 [작업하기]가 열린다.
        QStringList rejected;
        // 구형 서버가 calibs를 보내지 않으면 세션 중 H_MATRIX로 저장한 채널 결과를
        // 빈 객체로 덮어쓰지 않는다. logout()은 m_calibs를 비우므로 계정 간 값은 남지 않는다.
        if (calibs.isEmpty() && !m_calibs.isEmpty()) {
            appendLog(QStringLiteral("LOGIN_OK calibs 없음 — 세션 채널 캘리브 %1건 유지")
                          .arg(m_calibs.size()));
            return;
        }
        m_calibs = camcalib::filterUsableCalibMap(calibs, &rejected);
        // 채널마다 따로 본다 — 한 채널의 K/D 누락이 다른 채널 경고를 바꾸면 안 된다.
        for (int ch = 1; ch <= m_channelCount; ++ch)
            updateLensDataWarning(ch, calibOfChannel(ch));
        emit channelChanged();
        if (!rejected.isEmpty())
            appendLog(QStringLiteral("LOGIN_OK calibs — 사용할 수 없는 번들 무시: CH%1")
                          .arg(rejected.join(QStringLiteral(", CH"))));
        if (m_calibs.isEmpty()) return;
        QStringList ready;
        for (int ch = 1; ch <= m_channelCount; ++ch)
            if (channelCalibrated(ch)) ready << QStringLiteral("CH%1").arg(ch);
        appendLog(QStringLiteral("LOGIN_OK calibs — 캘리 완료 채널: %1 (서버 활성 CH%2)")
                      .arg(ready.isEmpty() ? QStringLiteral("없음") : ready.join(", "))
                      .arg(activeCh));
    });
    // LOGIN_OK 의 중계 스트림 주소 (선택 필드). 있으면 QSettings 값을 덮는다.
    //
    // 카메라가 4채널로 바뀌면서 RTSP 를 카메라에서 직접이 아니라 **중계 서버를 거쳐**
    // 받는다. 그 주소를 사람이 설정에 손으로 넣는 대신 서버가 로그인 때 알려주는 것이
    // 운영상 맞다 — 중계 주소가 바뀌어도 클라이언트를 안 건드린다.
    //
    // 서버가 stream 필드를 안 보내면 signal 자체가 오지 않아 기존 설정을 유지한다.
    // enabled=false를 명시하면 저장된 중계를 지우고 4채널 카메라 직결로 전환한다.
    // enabled=false면 중계만 해제하고 .13 카메라 직결을 유지한다.
    connect(m_client, &ServerClient::streamInfoReceived, this,
            [this](bool enabled, const QString &base, int channels) {
        if (!enabled) {
            setRelayBase(QString());
            appendLog(QStringLiteral("LOGIN_OK stream.enabled=false — 중계 해제, 카메라 직결 사용"));
            return;
        }
        if (base.trimmed().isEmpty()) return;
        if (channels > 0 && channels != kFourChannelCount)
            appendLog(QStringLiteral("LOGIN_OK stream.channels=%1 무시 — PNM은 4채널로 고정")
                          .arg(channels));
        if (base.trimmed() == m_relayBase) return;     // 같은 값이면 조용히 넘어간다
        setRelayBase(base);
        appendLog(QStringLiteral("LOGIN_OK stream — 중계 주소를 서버 값으로 설정: %1 (%2채널)")
                      .arg(m_relayBase).arg(m_channelCount));
    });
    // 채널 전환 결과. startChannelWork() 가 이미 갖고 있던 번들로 화면을 만들었으므로
    // 보통은 확인용이지만, 서버에만 있는 최신 번들이 여기서 오는 경우가 있다.
    connect(m_client, &ServerClient::channelResult, this,
            [this](bool ok, int ch, const QJsonObject &calib, bool hasCalib,
                   const QString &reason) {
        m_channelAckPending = false;
        if (!ok) {
            appendLog(QStringLiteral("CHANNEL_FAIL — %1").arg(reason));
            setNotice(QStringLiteral("채널 전환을 서버가 거절했습니다 (%1).").arg(reason),
                      QStringLiteral("error"));
            return;
        }
        clearNotice(QStringLiteral("channel-sync"));
        // 저장 전에 검증한다 (H_MATRIX 경로와 같은 규칙).
        const bool usable = hasCalib && camcalib::calibIsUsable(calib);
        if (hasCalib && !usable)
            appendLog(QStringLiteral("CHANNEL_OK CH%1 — 번들에 쓸 수 있는 H 가 없어 저장하지 않습니다")
                          .arg(ch));
        if (usable) m_calibs[QString::number(ch)] = calib;
        if (usable) updateLensDataWarning(ch, calib);
        emit channelChanged();
        if (ch != m_workingCh) return;   // 이미 다른 채널로 넘어갔으면 늦은 응답이다
        if (!usable) {
            appendLog(QStringLiteral("CHANNEL_OK CH%1 — 서버에 쓸 수 있는 채널 캘리브레이션이 없습니다").arg(ch));
            m_calibMissing = true;
            emit calibChanged();
            setNotice(QStringLiteral("CH%1 은 서버에 쓸 수 있는 캘리브레이션이 없습니다. 좌표를 "
                                     "믿을 수 없으니 설정 > 캘리브에서 이 채널의 호모그래피를 "
                                     "계산하세요.").arg(ch),
                      QStringLiteral("warn"), QStringLiteral("chcalib"));
            return;
        }
        appendLog(QStringLiteral("CHANNEL_OK CH%1 — ").arg(ch)
                  + applyCalibObject(calib, QStringLiteral("CH%1").arg(ch)));
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
    // 작업 취소 확인. 요청 주체와 무관하게 서버 ACK 뒤에만 로컬 상태를 접는다.
    connect(m_client, &ServerClient::drawAborted, this, [this](bool wasActive) {
        completeAbort(wasActive, m_abortPending);
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
        m_abortPending = false;
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

    // 중단 확인 감시. 시간이 지나도 "취소됨"으로 바꾸지 않는다 — 로봇이 실제로
    // 섰다는 것은 서버 종결 응답만 보증한다 (요청서 §4).
    m_cancelWatchdog = new QTimer(this);
    m_cancelWatchdog->setSingleShot(true);
    connect(m_cancelWatchdog, &QTimer::timeout, this, [this]() {
        if (!m_homographyPending || !m_homographyCancelPending) return;
        m_homographyCancelPending = false;          // 재시도 허용 (requested 는 유지)
        m_homographyStatus = QStringLiteral("중단 확인을 받지 못했습니다. 로봇이 아직 움직이는 "
                                            "중일 수 있습니다.");
        emit homographyChanged();
        setNotice(QStringLiteral("중단 요청에 대한 서버 확인이 없습니다. 로봇이 정지했는지 직접 "
                                 "확인하고, 필요하면 비상정지를 사용하세요."),
                  QStringLiteral("error"), kNoticeCancel);
        appendLog(QStringLiteral("CALIB_CANCEL 확인 없음 — 취소로 처리하지 않고 대기 유지"));
    });

    m_homographyTimer = new QTimer(this);
    m_homographyTimer->setSingleShot(true);
    m_homographyTimer->setInterval(kStaticHomographyTimeoutMs);
    connect(m_homographyTimer, &QTimer::timeout, this, [this]() {
        if (!m_homographyPending) return;
        failHomography(QStringLiteral("%1분 동안 완료 응답이 없어 대기를 종료했습니다. "
                                      "로봇과 서버 상태를 확인하세요.")
                           .arg(m_homographyTimer->interval() / 60000),
                       QStringLiteral("timeout"));
    });

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
    // 종료할 때는 실제로 기다린다. 여기서 안 기다리면 QThread 가 살아 있는 채로
    // 파괴돼 "QThread: Destroyed while thread is still running" 으로 죽는다.
    // (stopPreviews() 는 UI 응답성 때문에 기다리지 않으므로 여기서 별도로 처리)
    //
    // ⚠️ **먼저 전부 stop 을 걸고, 그다음에 기다린다.** 하나씩 stop→wait 을 반복하면
    //    워커가 4개라 최악의 경우 대기가 4배로 직렬화된다(스트림이 죽어 있으면
    //    각각 소켓 타임아웃까지 간다). 다 같이 멈추게 해두면 한 번의 대기로 끝난다.
    for (preview_worker *w : std::as_const(m_previews))
        if (w) { w->disconnect(); w->stop(); }
    for (preview_worker *w : std::as_const(m_previews)) {
        if (!w) continue;
        // 소멸자(~preview_worker)도 같은 이유로 기다린다 — 여기 값은 그것과 맞춘다.
        if (!w->wait(7000))
            qWarning() << "[shutdown] CH" << w->channel() << "미리보기 스레드가 안 끝났습니다";
    }
    qDeleteAll(m_previews);
    m_previews.clear();
}

// ⚠️ 여기 있던 setKeyboardControl / keyboardControl / m_keyboardControl 은 지웠다.
//    **아무 데서도 안 쓰였다** — QML 이 이 프로퍼티를 읽지도 쓰지도 않았고, C++ 에서
//    setter 를 부르는 곳도 없었다. 즉 m_keyboardControl 은 항상 false 였고, 그 값을
//    조건으로 쓰는 코드도 없어서 켜져도 아무 일이 안 일어났다(로그 한 줄 빼고).
//    키보드 로봇 제어를 실제로 붙일 때는 "값을 읽어 동작을 가르는 곳"부터 만들 것 —
//    프로퍼티만 되살리면 똑같이 죽은 코드가 된다.

void Backend::login(const QString &id, const QString &pw)
{
    // 더블클릭이 QML의 enabled 갱신보다 먼저 들어오면 같은 소켓에 로그인 절차가
    // 두 개 붙는다. 한 번 시작한 인증이 끝날 때까지 추가 요청은 받지 않는다.
    if (m_busy) return;
    if (id.isEmpty() || pw.isEmpty()) { emit loginFailed("아이디와 비밀번호를 입력해주세요."); return; }

    if (id == "test" && pw == "test") {
        m_testMode = true; m_userId = "test"; m_calib = QJsonObject();
        m_serverConnected = false;
        m_serverLabel = "테스트";
        loadHistory();
        emit sessionChanged();
        emit linkStatusChanged();
        enterInitialView();
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
                                 const QString &camIp, const QString &responseId,
                                 const QString &reason) {
        // loginResult/socketError/timeout이 같은 이벤트 루프에 연달아 도착해도 화면
        // 전환과 완료 처리는 정확히 한 번만 수행한다.
        if (guard->property("finished").toBool()) return;
        guard->setProperty("finished", true);
        m_busy = false; emit busyChanged();
        guard->deleteLater();
        if (ok) {
            m_testMode = false; m_userId = responseId;
            m_serverLabel = "연결됨";
            // 4채널 서버는 단일 calib 대신 calibs 맵만 보낼 수 있다. 둘 중 하나라도
            // 있으면 사이트 전체가 미보정이라는 경고를 띄우지 않는다.
            m_calibMissing = !hasCalib && m_calibs.isEmpty();
            // 서버는 로그인 성공 시점에 보관 중인 번들을 함께 내려준다 (QT-REQ-SRV-001 S-2).
            // 수동 입력과 같은 경로를 타야 K/D 와 coord_mode 가 함께 반영된다.
            if (hasCalib) {
                appendLog("LOGIN_OK calib — " + applyCalibObject(calib, QStringLiteral("LOGIN_OK")));
                // 채널이 없는 legacy 단일 번들이라 ch=0 키로 알린다 (채널별 경고와 별개).
                updateLensDataWarning(0, calib);
            } else {
                m_calib = calib;
            }
            loadHistory();
            emit sessionChanged();
            emit linkStatusChanged();
            emit calibChanged();
            // 운영 장치는 .13 PNM 한 대다. 서버 계정에 과거 카메라 주소가 남아
            // 있어도 CH1~CH4의 로컬 스트림 선택을 바꾸지 않는다.
            const QString serverCamIp = camIp.trimmed();
            if (!serverCamIp.isEmpty()
                && serverCamIp != QLatin1String(kDefaultFourChannelCameraIp))
                appendLog(QStringLiteral("LOGIN_OK cam_ip %1 무시 — 운영 카메라 %2 사용")
                              .arg(serverCamIp, QLatin1String(kDefaultFourChannelCameraIp)));
            setCamIp(QLatin1String(kDefaultFourChannelCameraIp));
            // 항상 CH1~CH4 그리드부터 시작한다.
            enterInitialView();
            appendLog(QString("로그인 성공: %1").arg(responseId));
            if (m_calibMissing) {
                setNotice(QStringLiteral("저장된 캘리브레이션이 없습니다. 설정 > 캘리브에서 "
                                         "채널을 선택해 호모그래피 계산을 시작하세요."),
                          QStringLiteral("warn"));
                appendLog(QStringLiteral("LOGIN_OK calib/calibs 없음 — 캘리브 설정 안내"));
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
    if (m_busy) return;
    if (id.isEmpty() || pw.isEmpty()) { emit registerFailed("아이디와 비밀번호를 입력해주세요."); return; }
    Q_UNUSED(camIp);
    const QString registeredCamIp = QLatin1String(kDefaultFourChannelCameraIp);

    m_busy = true; emit busyChanged();
    auto *guard = new QObject(this);
    auto *timer = new QTimer(guard);
    timer->setSingleShot(true);
    auto finish = [this, guard](bool ok, const QString &reason) {
        if (guard->property("finished").toBool()) return;
        guard->setProperty("finished", true);
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
        m_client->sendRegister(id, pw, registeredCamIp);
    } else {
        connect(m_client, &ServerClient::connectedToServer, guard,
                [this, id, pw, registeredCamIp]() {
            m_client->sendRegister(id, pw, registeredCamIp);
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
    // 🔴 워커에도 알린다 — 예전에는 뷰의 표시 플래그만 껐고 **검출은 계속 돌았다.**
    //    끄나 켜나 CPU 를 똑같이 태웠다는 뜻이다. 이제 끄면 검출 자체를 안 한다.
    if (m_worker) m_worker->setArucoEnabled(on);
    emit arucoChanged();
    appendLog(on ? QStringLiteral("ArUco 검출 켬")
                 : QStringLiteral("ArUco 검출 끔 — 검출 자체를 멈춥니다 (로봇 위치는 서버 POSE로 계속 수신)"));
}

// 번들은 예전과 똑같이 수용한다. 다만 coord_mode="undistort" 라고 선언해 놓고
// K 또는 D 가 없으면 화면은 왜곡 보정을 할 수 없다 — 조작자에게 알린다.
void Backend::updateLensDataWarning(int ch, const QJsonObject &calib)
{
    // clearNotice 는 **같은 키**일 때만 배너를 내리므로 CH1~CH4 경고가 서로를 못 지운다.
    const QString key = camcalib::lensDataNoticeKey(ch);
    if (camcalib::lensDataMissingForUndistort(calib)) {
        if (!m_lensDataMissingCh.contains(ch)) {
            m_lensDataMissingCh.insert(ch);
            appendLog(ch >= 1
                ? QStringLiteral("⚠️ CH%1 번들 coord_mode=undistort 인데 K/D 가 없습니다 "
                                 "— 렌즈 왜곡 보정 없이 사용합니다").arg(ch)
                : QStringLiteral("⚠️ 번들 coord_mode=undistort 인데 K/D 가 없습니다 "
                                 "— 렌즈 왜곡 보정 없이 사용합니다"));
            emit channelChanged();
        }
        setNotice(ch >= 1
                      ? QStringLiteral("CH%1 캘리브레이션은 undistort 기준이라고 하는데 "
                                       "렌즈 왜곡 보정 데이터(K/D)가 없습니다. 좌표 오차가 "
                                       "커질 수 있으니 이 채널의 카메라 내부 보정을 다시 "
                                       "받으세요.").arg(ch)
                      : QStringLiteral("받은 캘리브레이션은 undistort 기준이라고 하는데 "
                                       "렌즈 왜곡 보정 데이터(K/D)가 없습니다. 좌표 오차가 "
                                       "커질 수 있으니 카메라 내부 보정을 다시 받으세요."),
                  QStringLiteral("warn"), key);
        return;
    }
    if (m_lensDataMissingCh.remove(ch)) {
        appendLog(ch >= 1
            ? QStringLiteral("CH%1 번들에 K/D 가 갖춰졌습니다 — 왜곡 보정 데이터 경고 해제").arg(ch)
            : QStringLiteral("번들에 K/D 가 갖춰졌습니다 — 왜곡 보정 데이터 경고 해제"));
        emit channelChanged();
    }
    clearNotice(key);      // 같은 채널의 경고만 내린다
}

void Backend::setNotice(const QString &text, const QString &level, const QString &key)
{
    if (m_notice == text && m_noticeLevel == level && m_noticeKey == key) return;
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
    resetHomography();
    stopTestProgressSim();
    clearMission();
    // 미리보기 4개를 먼저 정리한다. 안 하면 로그인 화면 뒤에서 서브스트림 4개가
    // 계속 돌면서 카메라 세션과 CPU 를 붙잡고 있는다.
    stopPreviews();
    m_tiles.clear();
    m_highlightedCh = 0;
    m_workingCh = 0;
    m_calibs = QJsonObject();
    m_lensDataMissingCh.clear();
    emit channelChanged();
    if (m_worker) { m_worker->stop(); m_worker->wait(); m_worker->deleteLater(); m_worker = nullptr; }
    m_workerStarted = false;
    m_topView = nullptr;
    m_originalView = nullptr;

    if (m_drawing) { m_drawing = false; emit drawingChanged(); }
    m_estopActive = false;

    if (m_client && m_client->isConnected()) m_client->disconnectFromServer();

    appendLog(QString("로그아웃: %1").arg(m_userId.isEmpty() ? "사용자" : m_userId));
    m_userId.clear();
    // 카메라 IP는 장치 설정이다. 로그아웃 직후 test/test로 들어가도 마지막으로
    // 사용한 4채널 카메라를 그대로 열 수 있어야 한다.
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
    m_serverLabel = "재연결…";
    emit linkStatusChanged();
    m_client->reconnectToServer();
    appendLog("서버 재연결 시도");
}

void Backend::wireWorker(video_worker *w)
{
    if (!w) return;
    // 이 워커가 속한 세대. 채널을 바꾸면 m_streamGen 이 올라가므로, 아래 람다들은
    // 전부 "내 세대가 아직 현재인가"를 먼저 본다 (이전 채널 프레임 차단).
    const quint64 gen = m_streamGen;
    // 실제 영상이 한 장이라도 들어오면 오프라인 대체 캔버스를 끈다.
    // (RTSP 주소를 바꿔 워커를 새로 만들 때도 반드시 다시 걸려야 한다)
    connect(w, &video_worker::frameReceived, this, [this, gen](const QImage &img) {
        if (gen != m_streamGen) return;              // 옛 채널 워커의 늦은 프레임
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
        appendLog(QStringLiteral("카메라 영상 수신 시작 — %1×%2, 실시간 화면으로 전환")
                      .arg(m_frameW).arg(m_frameH));
        if (!m_channelUrlTemplate.isEmpty()
            && (m_frameW != 2592 || m_frameH != 1520)) {
            appendLog(QStringLiteral("⚠️ PNM profile2 디코더 출력이 운용 기준 2592×1520과 다릅니다."));
            setNotice(QStringLiteral("현재 영상은 %1×%2입니다. 카메라 profile2를 2592×1520으로 확인하세요.")
                          .arg(m_frameW).arg(m_frameH),
                      QStringLiteral("warn"), QStringLiteral("frame-size"));
        } else {
            clearNotice(QStringLiteral("frame-size"));
        }
    });
    connect(w, &video_worker::statsUpdated, this, [this, gen](const StreamStats &s) {
        if (gen != m_streamGen) return;
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
    connect(w, &video_worker::arucoDetected, this,
            [this, gen](const QList<int> &ids, const QList<QPolygonF> &corners) {
        if (gen != m_streamGen) return;              // 옛 채널 마커를 새 화면에 얹지 않는다
        onAruco(ids, corners);
    });
    // 워커가 새로 만들어질 때마다(채널 전환·주소 변경) 현재 토글 상태를 물려준다.
    // 안 하면 껐던 사람이 채널을 바꾸는 순간 검출이 다시 켜진다.
    w->setArucoEnabled(m_arucoOverlay);
    // 🔴 뷰로 가는 프레임도 세대 검사를 통과해야 한다. 워커→뷰 직결(큐드)이면
    //    이미 큐에 들어간 이전 채널 프레임이 새 채널 화면에 그대로 그려진다.
    if (m_topView) {
        VideoView *view = m_topView;
        connect(w, &video_worker::frameReceived, view, [this, gen, view](const QImage &img) {
            if (gen != m_streamGen) return;
            view->onFrame(img);
        });
    }
    if (m_originalView) {
        VideoView *view = m_originalView;
        connect(w, &video_worker::frameReceived, view, [this, gen, view](const QImage &img) {
            if (gen != m_streamGen) return;
            view->onFrame(img);
        });
    }
    // 백프레셔 해제는 **맨 마지막에** 연결한다 — 큐드 연결은 연결 순서대로 배달되므로,
    // 뷰들이 프레임을 다 처리한 뒤에야 "소비했다"고 알리게 된다.
    connect(w, &video_worker::frameReceived, this, [w]() { w->frameConsumed(); });
}

void Backend::startWorker()
{
    if (m_workerStarted) return;
    ++m_streamGen;                                   // 새 스트림 세대 시작
    m_worker = new video_worker(camcreds::apply(m_rtspUrl), this);
    m_worker->setVideoFilters(m_brightness, m_contrast, m_sharpen, m_saturation);
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
            m_calibMissing = !ok;   // 폴백(테스트 보정)을 "보정됨"으로 표시하지 않는다
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

                double tightestM = std::numeric_limits<double>::infinity();
                for (int i = 0; i < paths.size(); ++i) {
                    QList<QPointF> run = paths[i];
                    if (cl.value(i, false) && run.size() >= 3) run.append(run.first());
                    QList<bool> paint(run.size(), true);
                    if (!paint.isEmpty()) paint[0] = false;
                    double radiusM = 0.0;
                    if (motionprogram::firstTooTightPaintArc(
                            motionprogram::build(run, paint, m_speeds), &radiusM) >= 0)
                        tightestM = std::min(tightestM, radiusM);
                    // ARC 로 분류되지 못한 곡선(촘촘한 다각형 근사)도 같은 하한을 받는다
                    double curveRadiusM = 0.0;
                    if (motionprogram::firstTooTightPaintCurve(run, paint, &curveRadiusM) >= 0)
                        tightestM = std::min(tightestM, curveRadiusM);
                }
                if (std::isfinite(tightestM)) {
                    setNotice(QStringLiteral(
                        "현재 곡선 R %1 mm · 최소 도색 반지름은 200 mm입니다. "
                        "도형을 키워야 전송할 수 있습니다.")
                        .arg(tightestM * 1000.0, 0, 'f', 1),
                        QStringLiteral("warn"), QStringLiteral("arc-radius"));
                } else {
                    clearNotice(QStringLiteral("arc-radius"));
                }
            } else if (!m_jobActive && n < 2 && m_missionPaths.isEmpty()) {
                m_pathLengthM = 0;
                m_waypointCount = 0;
                m_workName = "—";
                clearNotice(QStringLiteral("arc-radius"));
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
                                     const QList<bool> &closed,
                                     bool preserveInputOrder) const
{
    QList<bool> preservePoints;
    preservePoints.reserve(paths.size());
    for (int i = 0; i < paths.size(); ++i) {
        QList<QPointF> run = paths[i];
        if (closed.value(i, false) && run.size() >= 3)
            run.append(run.first());

        motionprogram::detail::Circle fit;
        double sweep = 0.0;
        bool left = true;
        const bool oneArc = run.size() >= 5
                         && motionprogram::detail::arcFits(
                                run, 0, run.size() - 1, fit, sweep, left);
        preservePoints.append(oneArc);
    }
    return routeplan::plan(paths, closed, QPointF(m_poseX, m_poseY), m_poseValid,
                           0.01, preservePoints, preserveInputOrder);
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
    bool rearranged = false;
    for (int i = 0; i < route.order.size(); ++i) {
        if (route.order[i] != i || route.flipped.value(i, false)) {
            rearranged = true;
            break;
        }
    }
    if (route.shapeCount > 1 && rearranged) {
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
    } else if (route.shapeCount > 1) {
        const_cast<Backend *>(this)->appendLog(
            QString("도형 %1개 입력 순서·방향 유지 — 글자 획 순서 보호 (도색 %2 m)")
                .arg(route.shapeCount).arg(route.paintM, 0, 'f', 2));
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
    QString geometryError;
    const QList<QList<QPointF>> paths = m_topView->pathsToMeters(&closed, &geometryError);
    QList<bool> editClosed;
    QList<bool> editOuter;
    const QList<QList<QPointF>> editPaths =
        m_topView->editablePathsToMeters(&editClosed, &editOuter);
    if (!geometryError.isEmpty()) {
        appendLog(QStringLiteral("도면 전송 불가: %1").arg(geometryError));
        setNotice(QStringLiteral("도면을 확인하세요: %1").arg(geometryError),
                  QStringLiteral("error"), QStringLiteral("path-geometry"));
        return;
    }
    const int n = pathsPointCount(paths);
    if (n < 2) { appendLog("작도된 경로가 없습니다."); return; }
    if (m_jobActive) { appendLog("이미 작업이 진행 중입니다."); return; }

    // 프로토콜의 BLUEPRINT 는 폴리라인 1개다 → 도형들을 한 줄로 이어야 한다.
    // 일반 도형은 빈 이동을 줄이도록 순서·방향을 계획하고, 글자 도장은 필순을
    // 그대로 잠근다. 어느 경우든 도형 사이 이동에는 pen-up(paint=false)을 붙인다.
    const bool preserveInputOrder = m_topView->preservePathOrder();
    const routeplan::Route route = buildRoute(paths, closed, preserveInputOrder);
    const QList<QPointF> &blueprint = route.pts;
    if (blueprint.size() < 2) { appendLog("작도된 경로가 없습니다."); return; }

    // Qt는 노즐이 바닥에 그려야 하는 논리 동작을 만든다. 서버 v2는 이 program을
    // 로봇 op으로 변환하며 부호, 펜 오프셋, 노즐 타이밍과 ARC 실행 반지름을 보정한다.
    // ⚠️ 펜 오프셋은 넘기지 않는다. program은 도면 그대로의 논리 동작이다.
    const QList<motionprogram::Op> program =
        motionprogram::build(blueprint, route.paint, m_speeds);

    double tightRadiusM = 0.0;
    int tightOp = motionprogram::firstTooTightPaintArc(program, &tightRadiusM);
    if (tightOp < 0) {
        // ARC op 이 하나도 안 나온 곡선(예: 24각형 R=15mm)도 물리적으로는 곡선이다.
        // 여기서 막지 않으면 화면은 "도색 불가"인데 전송은 통과한다.
        tightOp = motionprogram::firstTooTightPaintCurve(blueprint, route.paint,
                                                        &tightRadiusM);
    }
    if (tightOp >= 0) {
        const QString msg = QStringLiteral(
            "곡선 반지름 %1 mm는 현재 서버·로봇의 최소 도색 반지름 %2 mm보다 작습니다. "
            "도형을 키우거나 곡선을 수정하세요.")
            .arg(tightRadiusM * 1000.0, 0, 'f', 1)
            .arg(motionprogram::kServerConfirmedMinPaintRadiusM * 1000.0, 0, 'f', 0);
        appendLog(QStringLiteral("BLUEPRINT 전송 차단 — 곡선 #%1, %2")
                      .arg(tightOp + 1).arg(msg));
        setNotice(msg, QStringLiteral("error"), QStringLiteral("blueprint"));
        m_blueprintSent = false;
        emit jobChanged();
        updatePhase();
        return;
    }

    // 이전 도면 ACK를 새 도면에 재사용하지 않는다. 실제 서버 접수 확인은
    // blueprintAck 핸들러가 다시 true로 만든다.
    m_blueprintSent = m_testMode;

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
                               m_workName == "편집 중" || m_workName == "-" ? QString() : m_workName,
                               QStringLiteral("대기"),
                               editPaths, editClosed, editOuter);

    storeMission(ordered, orderedClosed);
    m_missionEditPaths = editPaths;
    m_missionEditClosed = editClosed;
    m_missionEditOuter = editOuter;
    m_routePts = route.pts;
    m_routePaint = route.paint;
    m_travelLengthM = route.travelM;
    m_program = program;
    // 실서버에서는 BLUEPRINT_OK가 올 때까지 false다. 테스트 모드만 위에서 true.

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

// 작업 취소는 ESTOP과 다르다. ABORT_DRAW는 서버와 로봇의 실행 경로를 폐기하고,
// DRAW_ABORTED ACK가 와야 성공이다. ACK 전에는 로컬 상태를 유지한다.
void Backend::cancelJob()
{
    if (!m_jobActive || m_abortPending) return;

    m_abortPending = true;
    emit jobChanged();
    if (m_testMode) {
        appendLog(QStringLiteral("[테스트] CMD ABORT_DRAW — 즉시 ACK 시뮬레이션"));
        completeAbort(true, true);
    } else if (m_client) {
        m_client->sendAbortDraw();
        appendLog(QStringLiteral("CMD ABORT_DRAW — 서버·로봇 실행 경로 폐기 요청"));
        setNotice(QStringLiteral("작업 취소 응답을 기다리는 중입니다."),
                  QStringLiteral("warn"), QStringLiteral("abort"));
        QTimer::singleShot(3000, this, [this]() {
            if (!m_abortPending) return;
            m_abortPending = false;
            appendLog(QStringLiteral("ABORT_DRAW 응답 시간 초과 — 작업 상태 유지"));
            setNotice(QStringLiteral("작업 취소 응답을 받지 못했습니다. 서버 연결을 확인한 뒤 다시 시도하세요."),
                      QStringLiteral("error"), QStringLiteral("abort"));
            emit jobChanged();
            updatePhase();
        });
    }
}

void Backend::completeAbort(bool wasActive, bool requestedHere)
{
    m_abortPending = false;
    appendLog(QStringLiteral("DRAW_ABORTED — 서버·로봇 실행 경로 폐기 (진행 중이던 작업 %1)")
                  .arg(wasActive ? QStringLiteral("있음") : QStringLiteral("없음")));
    clearNotice(QStringLiteral("abort"));
    if (!m_jobActive) {
        emit jobChanged();
        return;
    }

    stopTestProgressSim();
    m_jobActive = false;
    m_paintingSeen = false;
    m_jobElapsedValid = false;
    m_jobProgress = 0.0;
    m_missionPathProgress = QList<double>(m_missionPaths.size(), 0.0);
    m_paintPathIndex = 0;
    m_paintPathLiftCount = 0;
    if (m_topView) {
        m_topView->setMissionProgress(0.0);
        m_topView->setMissionPathProgress(m_missionPathProgress);
    }
    if (m_originalView) {
        m_originalView->setMissionProgress(0.0);
        m_originalView->setMissionPathProgress(m_missionPathProgress);
    }
    updateJobRecord(m_currentJobId, QStringLiteral("중단"), 0.0);
    setNotice(requestedHere
                  ? QStringLiteral("작업이 취소됐습니다. 같은 도면을 다시 시작하거나 경로를 수정할 수 있습니다.")
                  : QStringLiteral("다른 곳에서 작업이 취소됐습니다."),
              requestedHere ? QStringLiteral("info") : QStringLiteral("warn"),
              QStringLiteral("abort-result"));
    emit jobChanged();
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
    const QList<QList<QPointF>> editPaths = m_missionEditPaths;
    const QList<bool> editClosed = m_missionEditClosed;
    const QList<bool> editOuter = m_missionEditOuter;

    stopTestProgressSim();
    m_jobProgress = 0.0;
    m_jobElapsedValid = false;
    // 편집하면 서버에 올려둔 도면은 낡은 값이 된다 → 다시 전송해야 한다
    m_blueprintSent = false;
    m_phase = "ready";
    if (m_topView) {
        m_topView->clearMission();
        m_topView->setEditPathsMeters(editPaths.isEmpty() ? paths : editPaths,
                                     editPaths.isEmpty() ? closed : editClosed,
                                     editPaths.isEmpty() ? QList<bool>() : editOuter);
    }
    if (m_originalView) m_originalView->clearMission();
    m_missionPaths.clear();
    m_missionClosed.clear();
    m_missionPathProgress.clear();
    m_paintPathIndex = 0;
    m_paintPathLiftCount = 0;
    m_missionEditPaths.clear();
    m_missionEditClosed.clear();
    m_missionEditOuter.clear();
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
    m_missionPathProgress.clear();
    m_paintPathIndex = 0;
    m_paintPathLiftCount = 0;
    m_missionEditPaths.clear();
    m_missionEditClosed.clear();
    m_missionEditOuter.clear();
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
    m_missionPathProgress = QList<double>(paths.size(), 0.0);
    m_paintPathIndex = 0;
    m_paintPathLiftCount = 0;
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
    m_abortPending = false;
    m_paintingSeen = false;
    m_jobProgress = 0.0;
    m_missionPathProgress = QList<double>(m_missionPaths.size(), 0.0);
    m_paintPathIndex = 0;
    m_paintPathLiftCount = 0;
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
    m_abortPending = false;
    m_paintingSeen = false;
    m_blueprintSent = false;   // 완료된 도면은 다시 그리려면 재전송이 필요하다
    // STATUS.painting=false가 마지막 POSE보다 먼저 도착하면 표시 진행률이 잠시
    // 마지막 투영값에 머물 수 있다. 완료의 유일한 권위인 DRAW_DONE이 이 함수로
    // 들어오면 반드시 100%로 확정하므로 "완료인데 96%" 상태는 남지 않는다.
    m_missionPathProgress = QList<double>(m_missionPaths.size(), 1.0);
    m_paintPathIndex = m_missionPaths.size();
    m_paintPathLiftCount = 0;
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
    const routeplan::Route r = buildRoute(m_topView->pathsToMeters(&closed), closed,
                                          m_topView->preservePathOrder());
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
                           const QString &name, const QString &status,
                           const QList<QList<QPointF>> &editPaths,
                           const QList<bool> &editClosed,
                           const QList<bool> &editOuter)
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
    if (!editPaths.isEmpty()) {
        QJsonArray jeditPaths, jeditClosed, jeditOuter;
        for (int i = 0; i < editPaths.size(); ++i) {
            QJsonArray one;
            for (const QPointF &point : editPaths[i]) {
                QJsonArray xy;
                xy.append(point.x());
                xy.append(point.y());
                one.append(xy);
            }
            jeditPaths.append(one);
            jeditClosed.append(editClosed.value(i, false));
            jeditOuter.append(editOuter.value(i, false));
        }
        o["editPaths"] = jeditPaths;
        o["editClosed"] = jeditClosed;
        o["editOuter"] = jeditOuter;
    }
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
    QString geometryError;
    const QList<QList<QPointF>> paths = m_topView->pathsToMeters(&closed, &geometryError);
    if (!geometryError.isEmpty()) {
        appendLog(QStringLiteral("도면 저장 불가: %1").arg(geometryError));
        setNotice(QStringLiteral("도면을 확인하세요: %1").arg(geometryError),
                  QStringLiteral("error"), QStringLiteral("path-geometry"));
        return;
    }
    if (pathsPointCount(paths) < 2) {
        appendLog("저장할 경로가 없습니다.");
        return;
    }
    QList<bool> editClosed;
    QList<bool> editOuter;
    const QList<QList<QPointF>> editPaths =
        m_topView->editablePathsToMeters(&editClosed, &editOuter);
    recordJob(paths, closed, name, QStringLiteral("보관"),
              editPaths, editClosed, editOuter);
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

        QList<QList<QPointF>> editPaths;
        QList<bool> editClosed;
        QList<bool> editOuter;
        for (const QJsonValue &pathValue : o.value("editPaths").toArray()) {
            QList<QPointF> one;
            for (const QJsonValue &pointValue : pathValue.toArray()) {
                const QJsonArray xy = pointValue.toArray();
                if (xy.size() >= 2)
                    one << QPointF(xy.at(0).toDouble(), xy.at(1).toDouble());
            }
            if (one.size() >= 2) editPaths << one;
        }
        const QJsonArray jEditClosed = o.value("editClosed").toArray();
        const QJsonArray jEditOuter = o.value("editOuter").toArray();
        for (int k = 0; k < editPaths.size(); ++k) {
            editClosed << (k < jEditClosed.size() && jEditClosed.at(k).toBool());
            editOuter << (k < jEditOuter.size() && jEditOuter.at(k).toBool());
        }

        clearMission();
        const double savedStroke = o.value("strokeMm").toDouble(m_strokeWidthMm);
        if (std::isfinite(savedStroke)) setStrokeWidthMm(savedStroke);
        if (m_topView)
            m_topView->setEditPathsMeters(editPaths.isEmpty() ? paths : editPaths,
                                         editPaths.isEmpty() ? closed : editClosed,
                                         editPaths.isEmpty() ? QList<bool>() : editOuter);
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
        m_topView->setMissionPathProgress(m_missionPathProgress);
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
        m_originalView->setMissionPathProgress(m_missionPathProgress);
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
    if (m_testMode && m_jobActive)
        m_missionPathProgress = paintprogress::prefixPathProgress(
            m_missionPaths, m_missionClosed, p);
    if (qFuzzyCompare(p, m_jobProgress)) {
        if (m_topView) {
            m_topView->setMissionProgress(m_jobProgress);
            m_topView->setMissionPathProgress(m_missionPathProgress);
        }
        if (m_originalView) {
            m_originalView->setMissionProgress(m_jobProgress);
            m_originalView->setMissionPathProgress(m_missionPathProgress);
        }
        return;
    }
    m_jobProgress = p;
    if (m_topView) {
        m_topView->setMissionProgress(m_jobProgress);
        m_topView->setMissionPathProgress(m_missionPathProgress);
    }
    if (m_originalView) {
        m_originalView->setMissionProgress(m_jobProgress);
        m_originalView->setMissionPathProgress(m_missionPathProgress);
    }
    refreshJobMetrics();
    emit jobChanged();
    updatePhase();
}

void Backend::updatePaintContact(const QPointF &pen)
{
    if (m_missionPaths.isEmpty()) return;
    if (m_missionPathProgress.size() != m_missionPaths.size())
        m_missionPathProgress = QList<double>(m_missionPaths.size(), 0.0);
    if (m_paintPathIndex < 0) m_paintPathIndex = 0;
    if (m_paintPathIndex >= m_missionPaths.size()) return;

    // 펜 반폭 + 측위 여유. 실행 차례의 획에 실제로 가까울 때만 주황색을 늘린다.
    const double strokeMm = m_topView ? m_topView->strokeWidthMm() : 60.0;
    const double contactToleranceM = qMax(0.04, strokeMm / 2000.0 + 0.03);

    // 🔴 현재 차례부터 **앞으로만** 훑는다. 두 가지를 동시에 지킨다:
    //   · 뒤로 안 간다 — 이미 지난 획 옆을 지나가도 그 획으로 되돌아가지 않는다.
    //   · 멈추지 않는다 — 예전에는 현재 획이 완료 판정을 못 받으면 인덱스가 영원히
    //     제자리라, 그 뒤의 모든 획이 끝까지 주황색이 되지 않았다. 완료 판정은
    //     onStatus 의 nozzle-up 한 경로뿐이라 한 번만 놓쳐도 화면이 통째로 굳는다.
    // 로봇은 보낸 순서대로만 그리므로, 펜이 뒤쪽 획 위에 **도색 중으로** 올라와
    // 있다면 그 앞의 획들은 실제로 이미 끝난 것이다 — 그때만 건너뛴다.
    int hitIndex = -1;
    paintprogress::PathProjection hit;
    for (int i = m_paintPathIndex; i < m_missionPaths.size(); ++i) {
        const paintprogress::PathProjection pr = paintprogress::projectOnPath(
            m_missionPaths[i], m_missionClosed.value(i, false), pen);
        if (pr.valid && pr.distanceM <= contactToleranceM) { hitIndex = i; hit = pr; break; }
    }
    if (hitIndex < 0) return;

    for (int i = m_paintPathIndex; i < hitIndex; ++i)
        m_missionPathProgress[i] = 1.0;
    if (hitIndex != m_paintPathIndex) {
        m_paintPathIndex = hitIndex;
        m_paintPathLiftCount = 0;
    }

    m_missionPathProgress[hitIndex] = qMax(m_missionPathProgress[hitIndex], hit.progress01);
    if (m_missionPathProgress[hitIndex] >= 0.985)
        m_missionPathProgress[hitIndex] = 1.0;
    setJobProgress(paintprogress::weightedProgress(
        m_missionPaths, m_missionClosed, m_missionPathProgress));
}


// 서버가 주는 POSE(x, y)는 **ArUco 마커(ID 49)의 중심**이다. 로봇팀 제원(2026-07-30):
//
//      ArUco 마커 중심  ── 설정 거리(기본 172mm) ──  노즐(= 펜 = 페인트가 나오는 곳)
//
// 즉 **마커 = 로봇 기준점**이고, 펜은 설정된 거리만큼 떨어져 있다. 화면은 이렇게 나눈다:
//   · setRobotPose  ← POSE 그대로            = 마커 = 로봇 기준점 (섀시를 여기 그린다)
//   · setPenMarker  ← POSE − 설정거리·heading = 노즐 (실제로 칠하는 점)
//
// 방향이 **뒤**인 근거: 로봇은 "1m 직진"을 받으면 오프셋만큼 **더 가서** 1m 를 긋는다
// (사용자 확인). 펜이 뒤에 달려 있어야 그 보정이 성립한다 — 중심이 d 만큼 더 나아가야
// 뒤따라오는 펜이 시작점에 닿는다. 앞에 달렸다면 반대로 덜 가야 한다.
// 방향이 틀린 것으로 밝혀지면 아래 부호 하나만 뒤집으면 된다.
//
// ⚠️ 서버의 시작점 도착·이탈 판정은 POSE(마커) 기준이다. 화면의 주황색 도색 진행선만
//    실제 도포 지점과 맞추기 위해 아래 노즐 위치를 사용한다.
// ⚠️ 표시 거리를 반영해서 로봇에 보내지 않는다. QT 는 도면 그대로만 낸다. 오프셋 보정은
//    서버 v2/로봇 전담이다(`pen_offset_m`은 Qt 프로토콜에서 폐지).
void Backend::pushPoseToView(bool valid)
{
    if (!m_topView) return;
    const double displayOffsetM = m_penDisplayOffsetMm / 1000.0;
    const QPointF pen = paintgeometry::penMarkerFromRobotCenter(
        m_poseX, m_poseY, m_poseTheta, displayOffsetM);
    m_topView->setRobotPose(m_poseX, m_poseY, m_poseTheta, valid);
    m_topView->setPenMarker(pen.x(), pen.y(), m_painting, valid);
}

// 테스트 재생 화면 표시 — 위와 **정확히 거울 관계**다.
//
// 실주행: 마커(POSE)를 받아서   → 노즐 = 마커 − 표시 거리
// 재생  : 노즐(도면 위 점)에서 → 마커 = 노즐 + 표시 거리
//
// 재생기는 시퀀스를 그대로 굴리므로 그 좌표가 곧 노즐이다(motionsim.h 참고).
// 🔴 예전에는 재생기 좌표를 setRobotPose 에도 그대로 넣어서, **미리보기에서는
//    섀시가 도면 선 위에** 있고 실주행에서는 설정 거리만큼 옆에 있었다. 같은 도면인데
//    미리보기와 실주행의 로봇 자세가 달라 보이면 미리보기를 믿을 수 없게 된다.
void Backend::pushSimPoseToView(const QPointF &nozzleM, double headingDeg, bool down)
{
    if (!m_topView) return;
    const double displayOffsetM = m_penDisplayOffsetMm / 1000.0;
    const QPointF center = paintgeometry::robotCenterFromPen(
        nozzleM.x(), nozzleM.y(), headingDeg, displayOffsetM);
    m_topView->setRobotPose(center.x(), center.y(), headingDeg, true);
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

    // 흰색으로 채워진 펜 마커(m_painting=true)가 실제 경로 위를 지날 때만
    // 주황색 도색 구간을 늘린다. 한 번 도색을 시작했다는 m_paintingSeen 으로
    // 계속 갱신하면 펜을 든 도형 간 이동도 칠한 것처럼 보인다.
    if (m_jobActive && !m_testMode && m_painting) {
        const double displayOffsetM = m_penDisplayOffsetMm / 1000.0;
        const QPointF pen = paintgeometry::penMarkerFromRobotCenter(
            x, y, thetaDeg, displayOffsetM);
        updatePaintContact(pen);
    }

    // ⚠️ 완료 판정은 하지 않는다. 끝은 서버의 DRAW_DONE 하나뿐이다.
    refreshLinkStatus();
}

void Backend::onStatus(const QString &state, bool painting)
{
    // STATUS 는 로봇이 초당 여러 번 보낸다 — 값이 그대로면 로그에 남기지 않는다.
    // 예전에는 무조건 찍어서 `STATUS IDLE painting=false` 가 로그를 통째로 덮었고,
    // 정작 봐야 할 줄이 스크롤 밖으로 밀려났다. 상태 자체는 상단 표시로 항상 보인다.
    const bool wasPainting = m_painting;
    const bool changed = (state != m_robotState) || (painting != m_painting);

    m_robotState = state;
    m_painting = painting;
    // 마지막 true POSE가 끝점 직전에 찍혀도 정상적인 nozzle-up 전환을 놓치지 않는다.
    // 단일 직선은 한 번, 짧은 꺾은선 글자는 선분 수만큼의 lift를 완료 근거로 쓴다.
    if (wasPainting && !painting && m_jobActive && !m_testMode
        && m_paintPathIndex >= 0 && m_paintPathIndex < m_missionPathProgress.size()) {
        ++m_paintPathLiftCount;
        const int pointCount = m_missionPaths.value(m_paintPathIndex).size();
        const int segmentCount = qMax(1, pointCount - 1
            + (m_missionClosed.value(m_paintPathIndex, false) && pointCount > 2 ? 1 : 0));
        // 한 구간 획은 짧아서 POSE 표본이 하나도 없어도 down->up 자체가 실제 도색
        // 완료 증거다. H/A/N/W 같은 짧은 꺾은선은 각 TURN마다 펜을 들기 때문에
        // 선분 수만큼 lift를 보았을 때 완료한다. 샘플이 많은 ARC는 투영률로 끝을 안다.
        const bool completed = pointCount == 2
            || m_missionPathProgress[m_paintPathIndex] >= 0.90
            || (segmentCount <= 6 && m_paintPathLiftCount >= segmentCount);
        if (completed) {
            m_missionPathProgress[m_paintPathIndex] = 1.0;
            ++m_paintPathIndex;
            m_paintPathLiftCount = 0;
            setJobProgress(paintprogress::weightedProgress(
                m_missionPaths, m_missionClosed, m_missionPathProgress));
        }
    }
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

// ── 다채널 카메라 (PNM-C16083RVQ, 프로토콜 v0.4) ──────────────────────────
//
// 화면은 두 단계다:
//   [그리드]  2x2 미리보기 4채널 (서브스트림, 마커검출 없음)
//      │ 타일 클릭 → 하이라이트 + [작업하기] 활성화
//      ▼ [작업하기]
//   [작업]    고른 채널 1개만 메인스트림 + 마커검출 + 기존 Qt 기능 전부
//
// 핵심은 **작업 화면이 기존 코드 경로를 그대로 탄다**는 것이다. 채널을 고르는
// 행위 = setRtsp() 를 새 URL 로 부르는 것 + 그 채널의 캘리브레이션을 적용하는 것.
// 캘리브레이션·ArUco·작도·도면 변환 코드는 한 줄도 안 건드린다.

// 주소는 두 갈래다:
//   · 중계 있음 → {relayBase}/chN · /chNs   (Server/relay/README.md 와 짝. 여기만
//                                            바꾸면 안 되고 mediamtx.yml 도 같이)
    //   · 중계 없음 → 카메라 직결 템플릿 ({ip}/{ch0} 치환, 센서 번호는 0부터)
// **중계가 있으면 중계가 이긴다.** 서버가 나중에 LOGIN_OK.stream 으로 중계 주소를
// 주면 직결 템플릿은 저절로 안 쓰이게 된다 — 그때 코드를 고칠 필요가 없다.
QString Backend::channelUrl(int ch, bool sub) const
{
    if (!m_relayBase.isEmpty())
        return QStringLiteral("%1/ch%2%3").arg(m_relayBase).arg(ch)
                                          .arg(sub ? QStringLiteral("s") : QString());

    // 🔴 자격증명이 없으면 주소를 만들지 않는다. 빈 아이디로 반복 접속하면
    //    한화 카메라가 계정을 잠근다 (camcreds.h 참고).
    if (m_channelUrlTemplate.contains(QStringLiteral("{user}")) && !camcreds::available())
        return QString();

    if (m_channelUrlTemplate.contains(QStringLiteral("{ip}")) && m_camIp.isEmpty())
        return QString();

    // 직결. ⚠️ 이 카메라에는 저해상도 서브 프로파일이 없어서 sub 여부와 무관하게
    //    같은 주소가 나간다 (현재 4채널 profile2는 H.264 2592x1520 20fps).
    //    서브가 생기면 여기서 sub 일 때 다른 템플릿을 쓰도록 갈라주면 된다.
    QString url = m_channelUrlTemplate;
    url.replace(QStringLiteral("{ip}"),  m_camIp);
    url.replace(QStringLiteral("{ch0}"), QString::number(ch - 1));
    url.replace(QStringLiteral("{ch}"),  QString::number(ch));
    return camcreds::apply(url);   // {user}/{pass} 를 실행 시점에 채운다
}

QString Backend::mainUrl(int ch) const { return channelUrl(ch, false); }
QString Backend::subUrl(int ch)  const { return channelUrl(ch, true);  }

// 지금 영상을 어디서 받는지 한 줄로. ⚠️ 직결 URL 에는 계정이 들어 있으므로
// 비밀번호를 가린 뒤에 낸다 — 이 문자열은 화면에도 로그에도 나간다.
static QString maskRtspPassword(const QString &url)
{
    // rtsp://user:pass@host/... 에서 pass 만 ***** 로 바꾼다
    static const QRegularExpression re(QStringLiteral("^(\\w+://[^:/@]+:)([^@/]*)(@)"));
    QString out = url;
    const auto m = re.match(out);
    if (m.hasMatch())
        out.replace(m.capturedStart(2), m.capturedLength(2), QStringLiteral("*****"));
    return out;
}

QString Backend::streamSourceText() const
{
    if (!m_relayBase.isEmpty())
        return QStringLiteral("중계 %1 · 메인 /ch1 … 서브 /ch1s").arg(m_relayBase);
    return QStringLiteral("카메라 직결 · %1 (서브 없음 — 미리보기도 풀해상도)")
               .arg(maskRtspPassword(channelUrl(1, false)));
}

QJsonObject Backend::calibOfChannel(int ch) const
{
    return m_calibs.value(QString::number(ch)).toObject();
}

bool Backend::channelCalibrated(int ch) const
{
    // 저장 시점에 이미 걸러지지만, 옛 저장값/수동 경로까지 같은 기준으로 본다.
    return camcalib::calibIsUsable(calibOfChannel(ch));
}

QVariantList Backend::calibratedChannels() const
{
    QVariantList out;
    for (int ch = 1; ch <= m_channelCount; ++ch)
        if (channelCalibrated(ch)) out << ch;
    return out;
}

// 채널별 "왜곡 보정 데이터 없음" 표시용. 배너는 하나뿐이라 다른 통지에 덮이지만,
// 이 목록은 그 채널에 완전한 번들이 들어올 때까지 화면에 남는다.
QVariantList Backend::lensDataMissingChannels() const
{
    QVariantList out;
    for (int ch = 1; ch <= m_channelCount; ++ch)
        if (m_lensDataMissingCh.contains(ch)) out << ch;
    return out;
}

void Backend::setRelayBase(const QString &base)
{
    QString clean = base.trimmed();
    // 뒤 슬래시를 남기면 URL 이 "…:8554//ch1" 이 되어 중계가 경로를 못 찾는다.
    while (clean.endsWith('/')) clean.chop(1);

    // 이 값은 MediaMTX 같은 중계 서버의 베이스 URL이다. 카메라의 완성된
    // /media.smp URL을 넣으면 뒤에 /ch1s가 붙어 반드시 404가 난다. 현장에서
    // 가장 쉽게 혼동되는 입력이므로 저장하지 않고 카메라 직결 모드로 복귀시킨다.
    if (clean.contains(QStringLiteral("/media.smp"), Qt::CaseInsensitive)) {
        appendLog(QStringLiteral("4채널 중계 주소에 카메라 RTSP URL을 넣을 수 없습니다. "
                                 "카메라 IP만 사용해 직결합니다."));
        setNotice(QStringLiteral("중계 주소에는 rtsp://서버IP:8554만 입력하세요. "
                                 "카메라 RTSP 전체 주소는 사용할 수 없습니다."),
                  QStringLiteral("warn"), QStringLiteral("relay-url"));
        clean.clear();
    }
    if (clean == m_relayBase) return;

    // LOGIN_OK에서는 stream 신호가 loginResult보다 먼저 온다. 로그인 처리 중에
    // setRelayBase가 화면을 열면 저장돼 있던 옛 cam_ip로 한 차례 접속한 뒤 다시
    // 열게 된다. 이때는 설정만 반영하고, loginResult가 cam_ip까지 적용한 다음
    // enterInitialView()에서 정확한 주소를 한 번만 연다.
    if (m_busy) {
        m_relayBase = clean;
        saveSettings();
        emit channelChanged();
        return;
    }

    m_relayBase = clean;
    saveSettings();

    if (m_relayBase.isEmpty()) {
        // 중계만 해제됐고 직결 템플릿은 남아 있다 → 4채널을 **카메라 직결**로 계속한다.
        // 화면에 떠 있는 스트림이 중계 URL 이면 그대로 두면 안 되므로 다시 잡는다.
        appendLog(QStringLiteral("중계 주소 해제 — 카메라 직결로 4채널을 계속합니다: %1")
                      .arg(channelUrl(1, false)));
        emit channelChanged();
        // ⚠️ 주소가 바뀌었으므로 미리보기를 **완전히 끊고** 새 주소로 다시 연다.
        //    startPreviews() 는 이미 워커가 있으면 재개만 하므로, 여기서 안 끊으면
        //    옛 주소를 계속 보게 된다.
        stopPreviews();
        if (m_workingCh > 0) setRtsp(mainUrl(m_workingCh));
        else                 showChannelGrid();
        return;
    }

    appendLog(QStringLiteral("중계 주소 설정: %1 (채널 %2개)")
                  .arg(m_relayBase).arg(m_channelCount));
    emit channelChanged();
    stopPreviews();   // 위와 같은 이유 — 새 주소로 다시 열어야 한다
    showChannelGrid();
}

void Backend::registerTile(ChannelTile *tile, int ch)
{
    if (!tile || ch <= 0) return;
    m_tiles.insert(ch, tile);
    tile->setChannel(ch);
    tile->setSelected(ch == m_highlightedCh);
    // QML 이 그리드를 다시 만들면(화면 전환) 타일 객체가 새로 생긴다. 사라진
    // 타일에 프레임을 밀면 죽으므로, 파괴될 때 등록을 지운다.
    connect(tile, &QObject::destroyed, this, [this, ch](QObject *o) {
        if (m_tiles.value(ch) == o) m_tiles.remove(ch);
    });
    connect(tile, &ChannelTile::clicked, this, &Backend::highlightChannel,
            Qt::UniqueConnection);
}

void Backend::highlightChannel(int ch)
{
    if (m_homographyPending) return;
    if (ch <= 0 || ch > m_channelCount) return;
    if (m_highlightedCh == ch) return;
    m_highlightedCh = ch;
    for (auto it = m_tiles.constBegin(); it != m_tiles.constEnd(); ++it)
        if (it.value()) it.value()->setSelected(it.key() == ch);
    appendLog(QStringLiteral("CH%1 선택 — 캘리브레이션 %2")
                  .arg(ch)
                  .arg(channelCalibrated(ch) ? QStringLiteral("있음")
                                             : QStringLiteral("없음 (좌표 작업 전 설정 필요)")));
    if (!channelCalibrated(ch))
        setNotice(QStringLiteral("CH%1 은 아직 캘리브레이션이 없습니다. 설정 > 캘리브에서 "
                                 "이 채널의 호모그래피를 먼저 계산하세요.").arg(ch),
                  QStringLiteral("warn"), QStringLiteral("chcalib"));
    else
        clearNotice(QStringLiteral("chcalib"));
    emit channelChanged();
}

void Backend::startChannelWork()
{
    if (m_homographyPending) return;
    if (!canStartChannelWork()) return;
    const int ch = m_highlightedCh;

    // 미리보기는 **끄지 않고 일시정지**한다. 작업 화면에서 안 보이는 건 맞지만,
    // 끄면 그리드로 돌아올 때 4채널을 다시 열어야 하고 그게 4.6초다(직렬화 때문).
    // 일시정지는 grab 만 돌려 세션만 살려두므로 복귀가 즉시다.
    // 대가가 있는지 실측했다: 메인+미리보기4 동시에도 메인이 15.0fps · p50 67.0ms 로
    // 단독과 같았다 (2026-08-04 유선). 즉 작업 화면 성능을 깎지 않는다.
    pausePreviews(true);
    m_workingCh = ch;

    // 서버에 "이 채널을 본다"고 알린다. 서버가 CCTV 에 중계해 그 채널의 마커를
    // 잡게 하고, POS 를 이 채널 캘리브레이션으로 변환한다. 이걸 빠뜨리면 영상만
    // 바뀌고 로봇 위치는 옛 채널 기준이라 조용히 어긋난다.
    const quint64 requestSerial = ++m_channelRequestSerial;
    m_channelAckPending = m_client && !m_testMode;
    if (m_channelAckPending) {
        m_client->sendSelectChannel(ch);
        QTimer::singleShot(2000, this, [this, ch, requestSerial]() {
            if (!m_channelAckPending || requestSerial != m_channelRequestSerial
                || ch != m_workingCh) return;
            m_channelAckPending = false;
            appendLog(QStringLiteral("SELECT_CHANNEL CH%1 응답 없음 — 영상만 전환됨").arg(ch));
            setNotice(QStringLiteral("CH%1 영상은 열렸지만 서버의 채널 전환 응답이 없습니다. "
                                     "좌표 피드백 채널은 확인되지 않았으므로 자동 주행을 "
                                     "시작하지 마세요.").arg(ch),
                      QStringLiteral("error"), QStringLiteral("channel-sync"));
        });
    }

    // 그 채널의 캘리브레이션을 적용한다. 서버 CHANNEL_OK 로도 같은 번들이 오지만,
    // 왕복을 기다리면 화면이 잠깐 옛 좌표계로 떠 있게 된다 — 갖고 있으면 먼저 쓴다.
    const QJsonObject calib = calibOfChannel(ch);
    if (!calib.isEmpty()) {
        bool applied = false;
        appendLog(QStringLiteral("CH%1 캘리브레이션 적용 — ").arg(ch)
                  + applyCalibObject(calib, QStringLiteral("CH%1").arg(ch), &applied));
        if (applied) {
            clearNotice(QStringLiteral("chcalib"));
            // 이 채널로 들어왔으니 K/D 누락 경고도 이 채널 기준으로 다시 평가한다.
            updateLensDataWarning(ch, calib);
        } else {
            // 적용 실패(테스트 보정 폴백)인데 경고를 지우면, 좌표를 믿을 수 없는
            // 상태가 화면에서 정상처럼 보인다.
            m_calibMissing = true;
            m_calibs.remove(QString::number(ch));
            updateLensDataWarning(ch, QJsonObject());   // 버린 번들의 경고는 남기지 않는다
            emit channelChanged();
            emit calibChanged();
            appendLog(QStringLiteral("⚠️ CH%1 캘리브레이션 적용 실패 — 좌표를 믿을 수 없습니다").arg(ch));
            setNotice(QStringLiteral("CH%1 캘리브레이션을 적용하지 못했습니다. 좌표가 맞지 않으므로 "
                                     "설정 > 캘리브에서 이 채널의 호모그래피를 다시 계산하세요.").arg(ch),
                      QStringLiteral("warn"), QStringLiteral("chcalib"));
        }
    } else {
        // 🔴 이 채널에는 번들이 없다. **이전 채널의 호모그래피를 그대로 물려받으면
        //    안 된다** — CH2 를 보다가 CH3 로 오면 CH3 영상이 CH2 좌표계로 펴져서
        //    "CH3 은 캘리 결과가 안 바뀐다"처럼 보이고, 도면은 엉뚱한 곳에 칠해진다.
        //    채널 전용 번들이 없으면 중립(테스트) 매핑으로 되돌린다.
        if (m_topView) m_topView->configureTopViewTest();
        m_calib = QJsonObject();
        m_calibId.clear();
        m_coordMode.clear();
        m_calibSource = QStringLiteral("없음");
        refreshCalibStatus();
        // 막지는 않는다(위 canStartChannelWork 주석 참고). 대신 좌표를 믿으면 안
        // 된다는 것을 확실히 남긴다 — 이 상태로 그린 도면은 엉뚱한 곳에 칠해진다.
        m_calibMissing = true;
        emit calibChanged();
        appendLog(QStringLiteral("⚠️ CH%1 캘리브레이션 없음 — 좌표를 믿을 수 없습니다 "
                                 "(영상 확인·수동 조작만 하세요)").arg(ch));
        setNotice(QStringLiteral("CH%1 은 캘리브레이션이 없습니다. 도면 좌표가 맞지 않으므로 "
                                 "설정 > 캘리브에서 이 채널의 호모그래피를 먼저 계산하세요.").arg(ch),
                  QStringLiteral("warn"), QStringLiteral("chcalib"));
    }

    // ← 여기가 이 설계의 핵심. setRtsp() 가 워커 교체와 뷰 재연결을 전부 한다.
    //    채널 전환은 그 함수를 다른 URL 로 부르는 것 이상이 아니다.
    setRtsp(mainUrl(ch));
    if (!m_workerStarted) startWorker();

    appendLog(QStringLiteral("CH%1 작업 시작 — 메인스트림 + 마커검출").arg(ch));
    emit channelChanged();
    updatePhase();
}

void Backend::showChannelGrid()
{
    if (m_jobActive) {
        setNotice(QStringLiteral("작업이 진행 중이라 채널을 바꿀 수 없습니다. 먼저 작업을 "
                                 "완료하거나 취소하세요."),
                  QStringLiteral("warn"));
        return;
    }
    ++m_channelRequestSerial;
    m_channelAckPending = false;
    m_workingCh = 0;
    // 작업용 메인스트림을 끊는다. 안 끊으면 그리드(4채널) 위에 메인 디코드가
    // 계속 얹혀 돈다 — 서브 프로파일이 없어진 뒤로는 그리드 자체가 1080p 4장이라
    // 이걸 안 끊으면 5장을 동시에 디코딩하게 된다.
    if (m_worker) {
        video_worker *old = m_worker;
        m_worker = nullptr;
        m_workerStarted = false;
        ++m_streamGen;                    // 그리드로 나온 뒤 도착하는 프레임은 전부 버린다
        old->disconnect();
        connect(old, &QThread::finished, old, &QObject::deleteLater);
        old->stop();
    }
    if (m_topView) m_topView->clearFrame();
    if (m_originalView) m_originalView->clearFrame();
    if (m_frameWatch) m_frameWatch->stop();
    startPreviews();
    appendLog(QStringLiteral("채널 목록으로 — 미리보기 %1채널").arg(m_channelCount));
    emit channelChanged();
    updatePhase();
}

bool Backend::matchesHomographyReply(int ch, const QString &requestId) const
{
    if (!m_homographyPending || ch != m_homographyCh) return false;
    return requestId.isEmpty() || requestId == m_homographyRequestId;
}

void Backend::resetHomography()
{
    if (m_homographyTimer) m_homographyTimer->stop();
    if (m_cancelWatchdog) m_cancelWatchdog->stop();
    const bool changed = m_homographyPending || m_homographyCancelPending
        || m_homographyCancelRequested
        || m_homographyCh != 0 || m_homographyProgress >= 0.0
        || !m_homographyStatus.isEmpty() || !m_homographyRequestId.isEmpty();
    m_homographyPending = false;
    m_homographyCancelPending = false;
    m_homographyCh = 0;
    m_homographyProgress = -1.0;
    m_homographyStatus.clear();
    m_homographyRequestId.clear();
    m_homographyOdometry = false;
    m_homographyCancelRequested = false;
    m_homographyPhase.clear();
    m_homographyPointIndex = -1;
    m_homographyPointTotal = -1;
    m_homographyValidPoints = -1;
    clearNotice(kNoticeCaptureLag);
    clearNotice(kNoticeCancel);
    if (changed) emit homographyChanged();
    if (changed) emit jobChanged();
}

void Backend::failHomography(const QString &message, const QString &reason)
{
    const int ch = m_homographyCh;
    resetHomography();
    setNotice(message, QStringLiteral("error"), QStringLiteral("homography"));
    appendLog(QStringLiteral("CALIB_FAIL CH%1 reason=%2 — %3").arg(ch).arg(reason, message));
}

// 사각형 한 변의 절반이 서버 min_move_m(1cm) 이상이어야 하므로 각 변 2cm 이상,
// 상한은 서버가 정한 1000cm(10m) 이하다 (양끝 포함).
// 서버가 invalid_param 으로 거절하기 전에 여기서 먼저 막는다 (요청서 §1).
bool Backend::startOdometryHomography(int ch, double mCm, double nCm, bool ccw)
{
    // QML TextField 의 Number("") 는 0, Number("abc") 는 NaN 이다. 그대로 실으면
    // 서버가 invalid_param 으로 거절하거나 로봇이 엉뚱한 거리를 돈다.
    if (!camcalib::odoSizeValidCm(mCm) || !camcalib::odoSizeValidCm(nCm)) {
        setNotice(camcalib::odoSizeRangeText(),
                  QStringLiteral("warn"), QStringLiteral("homography"));
        return false;
    }
    m_odoMCm = mCm;
    m_odoNCm = nCm;
    m_odoCcw = ccw;
    emit homographyChanged();     // 화면 값과 전송 값을 같은 원본으로 되돌린다
    return startHomography(ch, true);
}

// 🔴 화면에 **보이는 글자 그대로**로 시작한다. 입력칸 텍스트를 그대로 받아
//    검증하므로, 조작자가 보는 값과 CALIB_START 로 나가는 값이 절대 갈라지지
//    않는다. 검증 실패면 이전 유효값으로 조용히 되돌아가지 않고 시작을 막는다.
bool Backend::startOdometryHomographyFromText(int ch, const QString &widthText,
                                              const QString &heightText, bool ccw)
{
    double mCm = 0.0, nCm = 0.0;
    if (!camcalib::parseOdoSizeCm(widthText, &mCm)
        || !camcalib::parseOdoSizeCm(heightText, &nCm)) {
        setNotice(camcalib::odoSizeRangeText(),
                  QStringLiteral("warn"), QStringLiteral("homography"));
        appendLog(QStringLiteral("주행 캘리 시작 거부 — 입력값 \"%1\" × \"%2\" 가 "
                                 "%3~%4cm 범위의 숫자가 아닙니다")
                      .arg(widthText.trimmed(), heightText.trimmed())
                      .arg(camcalib::kOdoSizeMinCm, 0, 'g', 3)
                      .arg(camcalib::kOdoSizeMaxCm, 0, 'g', 4));
        return false;
    }
    return startOdometryHomography(ch, mCm, nCm, ccw);
}

bool Backend::startHomography(int ch, bool odometry)
{
    if (m_homographyPending) {
        setNotice(QStringLiteral("이미 CH%1 호모그래피를 계산 중입니다.").arg(m_homographyCh),
                  QStringLiteral("warn"), QStringLiteral("homography"));
        return false;
    }
    if (m_testMode || !m_serverConnected || !m_client || !m_client->isConnected()) {
        setNotice(QStringLiteral("호모그래피 계산은 서버에 로그인한 상태에서만 시작할 수 있습니다."),
                  QStringLiteral("warn"), QStringLiteral("homography"));
        return false;
    }
    if (ch < 1 || ch > m_channelCount) {
        setNotice(QStringLiteral("유효한 CCTV 채널을 선택하세요."),
                  QStringLiteral("warn"), QStringLiteral("homography"));
        return false;
    }
    if (m_jobActive || m_drawing) {
        setNotice(QStringLiteral("도면 편집 또는 로봇 작업을 끝낸 뒤 호모그래피를 시작하세요."),
                  QStringLiteral("warn"), QStringLiteral("homography"));
        return false;
    }
    if (!m_robotOnline || !m_cctvOnline) {
        setNotice(QStringLiteral("로봇과 CCTV가 모두 연결된 상태에서 시작하세요."),
                  QStringLiteral("warn"), QStringLiteral("homography"));
        return false;
    }

    if (m_workingCh != 0) showChannelGrid();
    m_highlightedCh = ch;
    for (auto it = m_tiles.constBegin(); it != m_tiles.constEnd(); ++it)
        if (it.value()) it.value()->setSelected(it.key() == ch);

    // 🔴 방식 상태를 **emit 보다 먼저** 세운다. 순서가 뒤집히면 첫 대기 화면이
    //    정적 앵커 화면으로 한 번 그려졌다가 서버 이벤트가 와야 주행 화면으로
    //    바뀐다 — 로봇이 움직이는 작업인데 안전 문구가 늦게 뜨는 셈이다.
    m_homographyPending = true;
    m_homographyCancelPending = false;
    m_homographyCancelRequested = false;
    m_homographyOdometry = odometry;
    m_homographyPhase = odometry ? QStringLiteral("requesting") : QString();
    m_homographyPointIndex = -1;
    m_homographyPointTotal = odometry ? kOdoStopPoints : -1;
    m_homographyValidPoints = -1;
    m_homographyCh = ch;
    m_homographyProgress = -1.0;
    m_homographyStatus = QStringLiteral("서버가 시작 요청을 확인하고 있습니다.");
    m_homographyRequestId = QStringLiteral("qt-%1-%2")
        .arg(QDateTime::currentMSecsSinceEpoch()).arg(++m_homographyRequestSerial);
    clearNotice(QStringLiteral("homography"));
    emit channelChanged();
    emit homographyChanged();
    emit jobChanged();
    // 주행 캘리는 로봇이 실제로 사각형을 도는 작업이라 2~4분 + 카메라 계산이
    // 더 붙는다. 5분은 빠듯하므로 이 방식일 때만 10분으로 늘린다 (요청서 §4 안 A).
    // 취소 버튼은 대기 내내 살아 있다 (cancelHomography).
    m_homographyTimer->setInterval(odometry ? kOdoHomographyTimeoutMs
                                            : kStaticHomographyTimeoutMs);
    m_homographyTimer->start();
    if (odometry) {
        const QString corner = camcalib::odoStartCorner(m_odoCcw);
        m_client->sendCalibStart(ch, m_homographyRequestId,
                                 QStringLiteral("robot_motion"),
                                 m_odoMCm, m_odoNCm, corner);
        appendLog(QStringLiteral("CALIB_START CH%1 request=%2 method=robot_motion "
                                 "%3×%4cm %5")
                      .arg(ch).arg(m_homographyRequestId)
                      .arg(m_odoMCm).arg(m_odoNCm)
                      .arg(m_odoCcw ? QStringLiteral("반시계(bottom_left)")
                                    : QStringLiteral("시계(top_left)")));
    } else {
        m_client->sendCalibStart(ch, m_homographyRequestId);
        appendLog(QStringLiteral("CALIB_START CH%1 request=%2").arg(ch).arg(m_homographyRequestId));
    }
    return true;
}

void Backend::cancelHomography()
{
    if (!m_homographyPending || m_homographyCancelPending || !m_client) return;
    m_homographyCancelPending = true;
    m_homographyCancelRequested = true;
    clearNotice(kNoticeCancel);            // 재요청하면 이전 "확인 없음" 경고를 내린다
    m_homographyStatus = QStringLiteral("서버에 중단을 요청했습니다. 로봇이 실제로 섰다는 "
                                        "확인을 기다립니다.");
    emit homographyChanged();
    m_client->sendCalibCancel(m_homographyCh, m_homographyRequestId);
    appendLog(QStringLiteral("CALIB_CANCEL CH%1 request=%2")
                  .arg(m_homographyCh).arg(m_homographyRequestId));
    // 🔴 종결 응답이 안 오면 "정지 중"이 캘리 한도(최대 10분)까지 그대로 남는다.
    //    로컬 감시로 **확인 실패만** 알린다 — 취소 성공으로 바꾸지 않는다.
    //    세션은 계속 열어 두고(로봇이 아직 돌고 있을 수 있다) 재시도를 허용한다.
    if (m_cancelWatchdog) m_cancelWatchdog->start(kCancelConfirmWatchdogMs);
}

// 수동 새로고침. 자동 재접속이 있는데도 이게 필요한 이유는 backend.h 주석 참고.
void Backend::refreshStreams()
{
    if (m_workingCh > 0) {
        // 작업 화면 — 메인스트림만 다시 연다. 미리보기는 일시정지 상태 그대로 둔다.
        appendLog(QStringLiteral("새로고침 — CH%1 메인스트림을 다시 엽니다").arg(m_workingCh));
        setRtsp(mainUrl(m_workingCh));
        if (!m_workerStarted) startWorker();
        return;
    }
    // 그리드 — 4채널을 전부 끊고 다시 연다 (약 4.6초)
    appendLog(QStringLiteral("새로고침 — 미리보기 %1채널을 다시 엽니다 (몇 초 걸립니다)")
                  .arg(m_channelCount));
    stopPreviews();
    startPreviews();
}

void Backend::pausePreviews(bool on)
{
    for (preview_worker *w : std::as_const(m_previews))
        if (w) w->setPaused(on);
    if (!m_previews.isEmpty())
        appendLog(on ? QStringLiteral("미리보기 일시정지 (세션은 유지 — 복귀가 즉시입니다)")
                     : QStringLiteral("미리보기 재개"));
}

void Backend::startPreviews()
{
    if (m_relayBase.isEmpty() && m_camIp.isEmpty()) {
        setNotice(QStringLiteral("4채널 카메라 IP가 설정되지 않아 영상을 열지 않았습니다."),
                  QStringLiteral("warn"), QStringLiteral("camera-ip"));
        return;
    }
    // 이미 돌고 있으면 **다시 열지 않는다.** 작업 화면에서 돌아온 경우가 이쪽인데,
    // 여기서 stop→start 를 하면 채널당 1.1초 x 4채널 직렬 = 4.6초를 그대로 낸다.
    // ⚠️ 단, 채널 수가 달라졌으면 재개하면 안 된다 — m_channelCount 는 서버의
    //    LOGIN_OK.stream 으로 **런타임에 바뀔 수 있다**(streamInfoReceived 참고).
    //    그대로 재개하면 채널이 늘어도 옛 개수만 보이고, 줄면 없는 채널을 계속 문다.
    if (!m_previews.isEmpty()) {
        if (m_previews.size() == m_channelCount) {
            pausePreviews(false);
            return;
        }
        appendLog(QStringLiteral("채널 수가 %1 → %2 로 바뀌어 미리보기를 다시 엽니다")
                      .arg(m_previews.size()).arg(m_channelCount));
        stopPreviews();
    }
    for (int ch = 1; ch <= m_channelCount; ++ch) {
        auto *w = new preview_worker(ch, subUrl(ch), this);
        w->setVideoFilters(m_brightness, m_contrast, m_sharpen, m_saturation);
        connect(w, &preview_worker::frameReceived, this,
                [this](int c, const QImage &img) {
            if (ChannelTile *t = m_tiles.value(c)) t->onFrame(img);
        });
        connect(w, &preview_worker::liveChanged, this, [this](int c, bool live) {
            if (ChannelTile *t = m_tiles.value(c)) t->setLive(live);
        });
        connect(w, &preview_worker::openFailed, this,
                [this](int c, const QString &url) {
            if (ChannelTile *t = m_tiles.value(c)) t->setFailed(true);
            appendLog(QStringLiteral("CH%1 미리보기 연결 실패 — %2 "
                                     "(자동 재시도는 하지 않습니다: 계정 잠김 방지)")
                          .arg(c).arg(url));
        });
        // 백프레셔 해제는 **맨 마지막에** 연결한다 — 큐드 연결은 연결 순서대로
        // 배달되므로, 타일이 프레임을 다 그린 뒤에야 "소비했다"고 알리게 된다.
        connect(w, &preview_worker::frameReceived, this, [w]() { w->frameConsumed(); });
        m_previews.append(w);
        w->start();
    }
}

void Backend::stopPreviews()
{
    for (preview_worker *w : std::as_const(m_previews)) {
        if (!w) continue;
        // ⚠️ 여기서 wait() 하지 않는다. 캡처 스레드는 죽은 RTSP 주소에서 수 초씩
        //    막혀 있을 수 있고, 그동안 GUI 스레드가 통째로 멈춘다 — 채널을 눌렀는데
        //    앱이 굳는 증상이 된다. 종료만 요청하고 스스로 끝나면 정리되게 넘긴다.
        w->disconnect();
        connect(w, &QThread::finished, w, &QObject::deleteLater);
        w->stop();
    }
    m_previews.clear();
    for (auto it = m_tiles.constBegin(); it != m_tiles.constEnd(); ++it)
        if (it.value()) it.value()->setLive(false);
}

// 로그인 직후 .13 PNM의 CH1~CH4 그리드를 연다.
void Backend::enterInitialView()
{
    // 테스트 모드도 실제 CCTV 영상과 채널 선택을 검증할 수 있어야 한다. 서버 연결과
    // 채널 선택 명령은 계속 생략하지만 RTSP 미리보기 경로는 일반 모드와 공유한다.
    m_workingCh = 0;
    m_highlightedCh = 0;
    startPreviews();
    emit channelChanged();
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
    // 오프셋은 서버/로봇 실행 단계의 책임이므로 논리 경로 재생기는 관여하지 않는다.
    m_sim.load(prog, motionprogram::approachCenter(pts),
               motionprogram::approachHeadingDeg(prog, pts));
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

void Backend::engageEstop()
{
    if (m_estopActive) {
        appendLog(QStringLiteral("비상정지 단축키 — 이미 정지 상태입니다 (해제는 버튼으로만)"));
        return;
    }
    sendRobotCmd("ESTOP", QStringLiteral("비상정지"));
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
    if (!m_worker) return;

    // ⚠️ 여기서 wait() 로 스레드를 기다리면 안 된다.
    // 캡처 스레드는 죽은 RTSP 주소에서 cap.read() 로 수십 초씩 막혀 있을 수 있고,
    // 그동안 GUI 스레드가 통째로 멈춰 "주소를 바꿨더니 앱이 굳는" 증상이 된다.
    // 종료만 요청해두고 스레드가 스스로 끝나면 정리되게 넘긴다.
    video_worker *old = m_worker;
    m_worker = nullptr;
    // 🔴 세대를 **먼저** 올린다. disconnect() 는 이미 큐에 실린 프레임까지 되돌리지
    //    못하므로, 세대가 바뀐 뒤 도착하는 옛 워커 프레임은 람다에서 버려진다.
    ++m_streamGen;
    old->disconnect();                                   // 이 워커의 모든 신호 연결 해제
    connect(old, &QThread::finished, old, &QObject::deleteLater);
    old->stop();

    // 이전 채널의 마지막 프레임을 화면에서 즉시 지운다. 안 지우면 새 스트림 첫
    // 프레임이 올 때까지 CH1 영상이 CH3 화면인 것처럼 1~2초 남는다.
    if (m_topView) m_topView->clearFrame();
    if (m_originalView) m_originalView->clearFrame();

    m_gotRealFrame = false;                              // 새 스트림 기준으로 다시 판단
    m_worker = new video_worker(camcreds::apply(m_rtspUrl), this);
    m_worker->setVideoFilters(m_brightness, m_contrast, m_sharpen, m_saturation);
    wireWorker(m_worker);
    m_worker->start();
    if (m_frameWatch) m_frameWatch->start();             // 새 주소도 안 붙으면 다시 대체 캔버스
    emit linkStatusChanged();
    appendLog("RTSP 변경: " + url);
}

// 서버가 준 카메라 IP(LOGIN_OK.cam_ip)로 RTSP URL 을 조립한다. 조립 규칙은 QT 담당.
// 로컬 RTSP 만 바꾼다 (LOGIN_OK.cam_ip 로 받은 값을 반영할 때 사용).
void Backend::setCamIp(const QString &ip)
{
    const QString clean = ip.trimmed();
    if (!clean.isEmpty() && clean != QLatin1String(kDefaultFourChannelCameraIp))
        appendLog(QStringLiteral("카메라 IP %1 요청 무시 — 운영 카메라는 %2")
                      .arg(clean, QLatin1String(kDefaultFourChannelCameraIp)));
    const bool changed = (m_camIp != QLatin1String(kDefaultFourChannelCameraIp));
    m_camIp = QLatin1String(kDefaultFourChannelCameraIp);
    emit sessionChanged();

    clearNotice(QStringLiteral("camera-ip"));
    saveSettings();
    if (changed && m_relayBase.isEmpty()) {
        stopPreviews();
        if (m_workingCh > 0) setRtsp(mainUrl(m_workingCh));
        else if (!m_userId.isEmpty()) startPreviews();
    }
    emit channelChanged();
}

// 설정 화면에서 사용자가 바꿀 때 — 로컬 반영 + 서버에 영속 저장(SET_CAM_IP).
// 서버는 형식 검증을 하지 않으므로 여기서 최소한의 형태만 확인한다.
void Backend::applyCamIp(const QString &ip)
{
    const QString clean = ip.trimmed();
    if (!clean.isEmpty() && clean != QLatin1String(kDefaultFourChannelCameraIp)) {
        setNotice(QStringLiteral("운영 카메라는 %1만 사용합니다.")
                      .arg(QLatin1String(kDefaultFourChannelCameraIp)),
                  QStringLiteral("warn"), QStringLiteral("camera-ip"));
        return;
    }
    setCamIp(QLatin1String(kDefaultFourChannelCameraIp));

    if (m_testMode || !m_serverConnected) {
        appendLog(QStringLiteral("[로컬] 카메라 IP %1 (서버 미연결 — SET_CAM_IP 생략)")
                      .arg(QLatin1String(kDefaultFourChannelCameraIp)));
        return;
    }
    m_client->sendSetCamIp(QLatin1String(kDefaultFourChannelCameraIp));
    appendLog(QStringLiteral("SET_CAM_IP %1 전송")
                  .arg(QLatin1String(kDefaultFourChannelCameraIp)));
}

void Backend::setVideoFilters(int b, int c, int s, int sat)
{
    b = qBound(-100, b, 100);
    c = qBound(-100, c, 100);
    s = qBound(0, s, 100);
    sat = qBound(-100, sat, 100);
    if (m_brightness == b && m_contrast == c
        && m_sharpen == s && m_saturation == sat) return;

    m_brightness = b;
    m_contrast = c;
    m_sharpen = s;
    m_saturation = sat;
    if (m_worker) m_worker->setVideoFilters(b, c, s, sat);
    for (preview_worker *preview : std::as_const(m_previews))
        if (preview) preview->setVideoFilters(b, c, s, sat);

    QSettings settings;
    settings.setValue(QStringLiteral("video/brightness"), b);
    settings.setValue(QStringLiteral("video/contrast"), c);
    settings.setValue(QStringLiteral("video/sharpen"), s);
    settings.setValue(QStringLiteral("video/saturation"), sat);
    emit videoFiltersChanged();
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

// 3x3 · 유한 실수 · 특이하지 않은 H 인가. 저장/적용 전 공통 관문이다.
bool Backend::calibHasUsableH(const QJsonObject &raw)
{
    const QJsonObject obj = normalizeCalibObject(raw);
    if (obj.isEmpty()) return false;
    const QJsonArray h = obj.value(QStringLiteral("H")).toArray().isEmpty()
        ? obj.value(QStringLiteral("H_floor")).toArray()
        : obj.value(QStringLiteral("H")).toArray();
    // 행렬 규칙은 camcalib 에 둔다 — 단위 테스트가 같은 함수를 검증한다.
    return camcalib::hasUsable3x3(h);
}

// 번들이 어디서 들어오든(수동 입력 · LOGIN_OK · H_MATRIX) 여기 한 곳만 지난다.
QString Backend::applyCalibObject(const QJsonObject &raw, const QString &source,
                                  bool *okOut)
{
    if (okOut) *okOut = false;
    const QJsonObject obj = normalizeCalibObject(raw);
    if (obj.isEmpty())
        return QStringLiteral("빈 캘리브레이션입니다.");

    // H(H_floor) 가 들어 있으면 저장·적용 전에 반드시 쓸 수 있는 행렬이어야 한다.
    // (코너만 있는 수동 번들은 H 없이 오므로 그 경우는 검사 대상이 아니다)
    if ((obj.contains(QStringLiteral("H")) || obj.contains(QStringLiteral("H_floor")))
        && !calibHasUsableH(obj)) {
        appendLog(QStringLiteral("캘리브 거부(%1) — 유효한 3x3 H 가 아닙니다").arg(source));
        return QStringLiteral("유효한 3x3 H 행렬이 아닙니다.");
    }

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
                // ⚠️ "프로파일을 되돌리세요" 라고만 쓰지 않는다. 해상도를 **일부러**
                //    바꾸는 경우가 있어서(16:9 모니터에 맞춰 2592x1520 → 1920x1080 으로
                //    바꾼 2026-08-04 처럼) 그때는 되돌리는 게 아니라 캘리브레이션을
                //    새 해상도로 다시 잡는 것이 맞다. 둘 다 제시해야 조작자가 고른다.
                setNotice(QStringLiteral("캘리브레이션은 %1×%2 기준인데 영상은 %3×%4 입니다. "
                                         "카메라 프로파일을 %1×%2 로 되돌리거나, "
                                         "캘리브레이션을 %3×%4 로 다시 잡아야 합니다.")
                              .arg(m.imgW).arg(m.imgH).arg(m_frameW).arg(m_frameH),
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
        if (okOut) *okOut = true;   // 보관 성공 — 화면이 생기면 그대로 적용된다
        appendLog(QStringLiteral("캘리브 수신(%1) — TopView 생성 시 적용됩니다").arg(source));
        return QStringLiteral("보관됨 — 화면 준비 후 적용");
    }

    // 🔴 채널/보정이 바뀌면 화면 픽셀 ↔ 월드 mm 대응이 통째로 바뀐다. 작도 중인
    //    도형은 TopView 픽셀로 들고 있으므로, 새 대응이 들어오기 전에 월드 좌표(m)로
    //    떠 두었다가 그대로 되돌린다. 안 그러면 같은 픽셀이 다른 mm 로 재해석돼
    //    "CH2 에서 그린 도형이 CH3 좌표로 전송"되는 조용한 사고가 난다.
    QList<bool> draftClosed, draftOuter;
    const QList<QList<QPointF>> draftM =
        m_topView->editablePathsToMeters(&draftClosed, &draftOuter);

    const bool ok = m_topView->configureTopViewCalib(obj);
    // TopView 구성이 실패하면 테스트 보정으로 폴백한 상태다 — 그 화면을 "보정됨"
    // 으로 표시하면 경고가 정확히 필요한 순간에 사라진다.
    m_calibMissing = !ok;
    if (!draftM.isEmpty()) {
        m_topView->setEditPathsMeters(draftM, draftClosed, draftOuter);
        appendLog(QStringLiteral("작도 중인 도형 %1개를 월드 좌표 기준으로 유지했습니다 "
                                 "(새 보정으로 재투영)").arg(draftM.size()));
    }
    refreshCalibStatus();
    pushMissionToView();
    pushOverlayToOriginal();   // 좌표 변환이 바뀌었으니 원본 뷰 오버레이도 다시 만든다
    appendLog(ok ? QStringLiteral("캘리브 적용(%1): ").arg(source) + m_calibStatus
                 : QStringLiteral("캘리브 실패(%1) — 테스트 보정으로 폴백").arg(source));
    logCalibUnit(m_topView);
    if (okOut) *okOut = ok;
    return ok ? (QStringLiteral("적용됨 · ") + m_calibStatus)
              : QStringLiteral("적용 실패 (테스트 보정 사용)");
}

// 곡선을 ARC op 으로 보낼지.
// ⚠️ 여기 있던 setArcEnabled / m_arcEnabled 는 지웠다.
//    곡선은 **항상 ARC op** 으로 보낸다 — 켜고 끌 대상이 아니다.
//      · 동작 수가 4~10배 줄어든다 (O: 49 → 4, STOP: 157 → 25).
// 서버 v2에서 ARC 실행 반지름과 바퀴 제어는 서버/로봇 계약의 책임이다.
// Qt는 도면 반지름, 스윕, 방향과 정확한 진입 접선만 제공한다.

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

    // 🔴 heightMm 은 **완성 도색 높이**다. 획 글자는 글리프가 펜 폭만큼 작아지므로
    //    완성 높이가 펜 폭 이하이면 글리프가 0 이하가 된다 — 만들기 전에 막고
    //    사유를 보여준다 (변 길이 입력창이 하는 것과 같은 처리).
    //    외곽선(outline)은 닫힌 도형이라 이 보정 대상이 아니다.
    if (!outline) {
        double storedMm = 0.0;
        if (!paintdimensions::storedSegmentMm(heightMm, m_strokeWidthMm,
                                              /*outerContour=*/false, &storedMm)) {
            setNotice(QStringLiteral("완성 도색 높이 %1 mm 는 붓 폭 %2 mm 보다 커야 합니다.")
                          .arg(heightMm, 0, 'f', 0).arg(m_strokeWidthMm, 0, 'f', 0),
                      QStringLiteral("warn"));
            return;
        }
    }
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
    // 작업 영역에 들어가는지 손으로 계산하지 않게 실제 도색 크기를 남긴다.
    const QSizeF paintedMm = m_topView->lastTextPaintedMm();
    if (!outline && paintedMm.isValid() && paintedMm.width() > 0.0)
        appendLog(QStringLiteral("  실제 도색 크기 %1 x %2 mm (가로 x 세로)")
                      .arg(paintedMm.width(), 0, 'f', 0)
                      .arg(paintedMm.height(), 0, 'f', 0));

    // 글자 높이가 붓 폭보다 너무 작으면 뭉개진다 — 미리 알려준다.
    // ⚠️ 기준이 **완성 높이**로 바뀌었다. 예전 규칙은 '중심 높이 >= 붓폭 x 2.5' 였고,
    //    완성 = 중심 + 붓폭 이므로 완성 기준으로는 x 3.5 다. 문구의 권장값도 같이 쓴다.
    const double kMinRecommendedMm = m_strokeWidthMm * 3.5;
    // 🔴 실제 도색 크기를 **화면 통지로도** 띄운다.
    //    글자는 획이 여러 개라 치수 라벨이 활성 획 하나에만 붙는다 — 사각형처럼
    //    변마다 숫자가 보이지 않는다. 조작자가 "내가 친 높이가 진짜 저 크기인가"를
    //    확인할 곳이 로그밖에 없으면 놓친다. 경고가 있으면 아래 경고가 이긴다.
    if (!outline && paintedMm.isValid() && paintedMm.width() > 0.0
        && heightMm >= kMinRecommendedMm) {
        setNotice(QStringLiteral("실제 도색 크기 %1 x %2 mm (가로 x 세로) — 붓 폭 %3 mm 포함")
                      .arg(paintedMm.width(), 0, 'f', 0)
                      .arg(paintedMm.height(), 0, 'f', 0)
                      .arg(m_strokeWidthMm, 0, 'f', 0),
                  QStringLiteral("info"));
    }
    if (!outline && heightMm < kMinRecommendedMm) {
        setNotice(QStringLiteral("완성 도색 높이 %1 mm 는 붓 폭 %2 mm 에 비해 작아 뭉개질 수 있습니다. "
                                 "%3 mm 이상을 권장합니다.")
                      .arg(heightMm, 0, 'f', 0)
                      .arg(m_strokeWidthMm, 0, 'f', 0)
                      .arg(kMinRecommendedMm, 0, 'f', 0),
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
    if (!std::isfinite(mm)) return;
    const double v = qBound(1.0, mm, 500.0);
    if (qFuzzyCompare(m_strokeWidthMm, v)) return;

    // 서버에 보낸 중심 경로는 당시 펜촉 폭으로 계산됐다. 작업 중 값을 바꾸면
    // 화면과 실제 실행이 달라지므로 막고, 실행 전이면 외곽 편집 상태로 되돌려
    // 새 폭으로 다시 계산·전송하게 한다.
    if (m_jobActive) {
        appendLog(QStringLiteral("작업 중에는 펜촉 폭을 변경할 수 없습니다."));
        setNotice(QStringLiteral("작업을 중단하거나 완료한 뒤 펜촉 폭을 변경하세요."),
                  QStringLiteral("warn"), QStringLiteral("stroke-width"));
        return;
    }
    if (canEditMission()) {
        editMission();
        setNotice(QStringLiteral("펜촉 폭이 바뀌어 경로를 다시 계산해야 합니다. 도면을 확인하고 경로를 재전송하세요."),
                  QStringLiteral("warn"), QStringLiteral("stroke-width"));
    }

    m_strokeWidthMm = v;
    if (m_topView) m_topView->setStrokeWidthMm(v);
    if (m_originalView) m_originalView->setStrokeWidthMm(v);
    QSettings().setValue(QStringLiteral("paint/strokeWidthMm"), v);
    emit strokeWidthChanged();
    appendLog(QStringLiteral("도장 폭 %1 mm 적용").arg(v, 0, 'f', 0));
}

void Backend::setPenDisplayOffsetMm(double mm)
{
    if (!std::isfinite(mm)) return;
    const double value = qBound(0.0, mm, 500.0);
    if (qFuzzyCompare(m_penDisplayOffsetMm + 1.0, value + 1.0)) return;

    m_penDisplayOffsetMm = value;
    QSettings().setValue(QStringLiteral("ui/penDisplayOffsetMm"), value);
    emit penDisplayOffsetChanged();

    // 실주행 표시는 즉시, 시뮬레이션 표시는 다음 40ms 틱에서 새 값을 사용한다.
    if (!m_simRunning) pushPoseToView(m_poseValid);
    appendLog(QStringLiteral("중심-펜 표시 거리 %1 mm 적용").arg(value, 0, 'f', 0));
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
    double savedPenDisplayOffset = s.value("ui/penDisplayOffsetMm", 172.0).toDouble();
    double savedStrokeWidth = s.value("paint/strokeWidthMm", 60.0).toDouble();

    // v1의 기본값(50mm/155mm)이 QSettings에 저장된 PC에서는 단순히 C++ 기본값만
    // 바꿔도 계속 옛 값이 살아난다. 사용자가 직접 넣은 다른 값은 보존하고,
    // 정확히 옛 기본값인 경우에만 프로젝트 실측값 60mm/172mm로 한 번 이관한다.
    const int savedGeometryVersion = s.value("paint/geometryDefaultsVersion", 1).toInt();
    if (savedGeometryVersion < kGeometryDefaultsVersion) {
        if (!s.contains("ui/penDisplayOffsetMm")
            || std::abs(savedPenDisplayOffset - 155.0) < 0.01)
            savedPenDisplayOffset = 172.0;
        if (!s.contains("paint/strokeWidthMm")
            || std::abs(savedStrokeWidth - 50.0) < 0.01)
            savedStrokeWidth = 60.0;
        s.setValue("ui/penDisplayOffsetMm", savedPenDisplayOffset);
        s.setValue("paint/strokeWidthMm", savedStrokeWidth);
        s.setValue("paint/geometryDefaultsVersion", kGeometryDefaultsVersion);
    }
    m_penDisplayOffsetMm = std::isfinite(savedPenDisplayOffset)
        ? qBound(0.0, savedPenDisplayOffset, 500.0) : 172.0;
    m_brightness = qBound(-100, s.value("video/brightness", 45).toInt(), 100);
    m_contrast = qBound(-100, s.value("video/contrast", 8).toInt(), 100);
    m_sharpen = qBound(0, s.value("video/sharpen", 0).toInt(), 100);
    m_saturation = qBound(-100, s.value("video/saturation", 0).toInt(), 100);
    m_strokeWidthMm = std::isfinite(savedStrokeWidth)
        ? qBound(1.0, savedStrokeWidth, 500.0) : 60.0;

    // 운영 카메라는 .13 PNM 한 대와 CH1~CH4로 고정한다. 과거 카메라 설정이나
    // 임의 RTSP 주소는 읽지 않고 현재 설정 키를 정상값으로 즉시 이관한다.
    m_camIp = QLatin1String(kDefaultFourChannelCameraIp);
    m_channelCount = kFourChannelCount;
    m_channelUrlTemplate = QStringLiteral(
        "rtsp://{user}:{pass}@{ip}:554/{ch0}/profile2/media.smp");
    s.remove("camera/rtspUrl");
    s.setValue("camera/ip", m_camIp);
    s.setValue("camera/channelIp", m_camIp);
    s.setValue("camera/channelCount", m_channelCount);
    s.setValue("camera/channelUrlTemplate", m_channelUrlTemplate);

    // 중계 주소는 선택 사항이다. 비어 있으면 .13 카메라에 직접 연결한다.
    m_relayBase = s.value("camera/relayBase").toString().trimmed();
    while (m_relayBase.endsWith('/')) m_relayBase.chop(1);
    // 과거 UI에서 카메라의 완성된 RTSP URL을 중계 베이스로 저장한 경우
    // /chNs가 뒤에 붙어 404가 반복된다. 해당 오입력만 제거하고 직결로 복구한다.
    if (m_relayBase.contains(QStringLiteral("/media.smp"), Qt::CaseInsensitive)) {
        m_relayBase.clear();
        s.setValue("camera/relayBase", QString());
    }
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
    s.setValue("ui/penDisplayOffsetMm", m_penDisplayOffsetMm);
    s.setValue("paint/strokeWidthMm", m_strokeWidthMm);
    s.setValue("paint/geometryDefaultsVersion", kGeometryDefaultsVersion);
    s.setValue("video/brightness", m_brightness);
    s.setValue("video/contrast", m_contrast);
    s.setValue("video/sharpen", m_sharpen);
    s.setValue("video/saturation", m_saturation);
    s.setValue("camera/ip", m_camIp);
    if (!m_channelUrlTemplate.isEmpty())
        s.setValue("camera/channelIp", m_camIp);
    s.setValue("camera/relayBase", m_relayBase);
    s.setValue("camera/channelUrlTemplate", m_channelUrlTemplate);
    s.setValue("camera/channelCount", m_channelCount);
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
//   설정값(m_speeds)은 남긴다 — 화면 미리보기 재생에만 쓰이는 **로컬 값**이다.
void Backend::pushSpeeds()
{
    appendLog(QStringLiteral("속도 설정(로컬) — 이동 %1 · 도색 %2 m/s · 회전 %3 °/s "
                             "· 실제 주행 속도는 로봇 펌웨어 고정값")
                  .arg(m_speeds.travelMps, 0, 'f', 2)
                  .arg(m_speeds.paintMps, 0, 'f', 2)
                  .arg(m_speeds.turnDps, 0, 'f', 0));
}

// 확정 로봇 사양으로 계산한 이번 도면의 최소 예상 소요 시간.
// 카메라 피드백 보정과 통신 대기는 도면만으로 알 수 없어 제외한다.
QString Backend::planTimeText() const
{
    const QList<motionprogram::Op> prog = currentProgram();
    if (prog.isEmpty()) return QStringLiteral("—");
    const int sec = int(robottiming::estimatedSeconds(prog) + 0.5);
    return QStringLiteral("%1분 %2초 이상").arg(sec / 60).arg(sec % 60);
}

// ⚠️ 여기 있던 setPenOffsetMm / setPenOffsetFromServer 는 지웠다.
//    program 이 펜 프레임 그대로 나가게 되면서(2026-07-28 프로토콜, pen_offset_m 폐지)
//    이 값은 전송에도, 시퀀스 생성에도, 시뮬레이션에도 쓰이지 않는 순수 잔재였다.
//    펜 오프셋 보정은 서버 v2/로봇 전담이다.
