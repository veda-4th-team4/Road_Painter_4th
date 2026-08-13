#include "serverclient.h"

#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonValue>
#include <QHostAddress>
#include <QSslCertificate>
#include <QSslConfiguration>
#include <QDebug>
#include <QStringList>
#include <cmath>

ServerClient::ServerClient(QObject *parent)
    : QObject(parent)
{
    m_socket = new QSslSocket(this);

    const QList<QSslCertificate> trustedCertificates = QSslCertificate::fromPath(
        QStringLiteral(":/certs/server.crt"), QSsl::Pem);
    QSslConfiguration tls = m_socket->sslConfiguration();
    tls.setCaCertificates(trustedCertificates);
    tls.setPeerVerifyMode(QSslSocket::VerifyPeer);
    m_socket->setSslConfiguration(tls);
    m_tlsCertificateReady = trustedCertificates.size() == 1
                         && !trustedCertificates.constFirst().isNull();

    connect(m_socket, &QSslSocket::encrypted, this, &ServerClient::onEncrypted);
    connect(m_socket, &QSslSocket::readyRead, this, &ServerClient::onReadyRead);
    connect(m_socket, &QSslSocket::disconnected, this, &ServerClient::onDisconnected);
    connect(m_socket, &QSslSocket::sslErrors, this, &ServerClient::onErrors);
    connect(m_socket, &QAbstractSocket::errorOccurred, this, [this](QAbstractSocket::SocketError) {
        emit socketError(m_socket->errorString());
    });
}

ServerClient::~ServerClient()
{
    if (m_socket && m_socket->state() != QAbstractSocket::UnconnectedState) {
        m_socket->abort();
    }
}

void ServerClient::connectToServer()
{
    // connectToHostEncrypted()를 ConnectingState에서 다시 호출하면 Qt가 경고를
    // 남기고 TLS 핸드셰이크/로그인 콜백이 중복될 수 있다. 연결 완료뿐 아니라
    // 조회·접속·종료 중인 상태도 새 연결을 시작하지 않는다.
    if (!m_socket || m_socket->state() != QAbstractSocket::UnconnectedState) return;
    if (!m_tlsCertificateReady) {
        emit socketError(QStringLiteral("내장 서버 인증서를 읽을 수 없습니다. 프로그램을 다시 배포해 주세요."));
        return;
    }
    if (!QSslSocket::supportsSsl()) {
        emit socketError(QStringLiteral("TLS 백엔드를 사용할 수 없습니다. 프로그램 배포 파일을 확인해 주세요."));
        return;
    }
    m_seq = 0;
    m_helloSent = false;
    m_buffer.clear();
    m_socket->connectToHostEncrypted(m_host, m_port);
}

void ServerClient::reconnectToServer()
{
    if (!m_socket) return;
    if (m_socket->state() != QAbstractSocket::UnconnectedState)
        m_socket->abort();
    connectToServer();
}

void ServerClient::setServer(const QString &host, quint16 port)
{
    if (host.isEmpty()) return;
    m_host = host;
    m_port = port ? port : 9000;
}

void ServerClient::disconnectFromServer()
{
    if (m_socket) m_socket->disconnectFromHost();
}

bool ServerClient::isConnected() const
{
    return m_socket && m_socket->state() == QAbstractSocket::ConnectedState;
}

bool ServerClient::isEncrypted() const
{
    return m_socket && m_socket->isEncrypted();
}

void ServerClient::onErrors(const QList<QSslError> &errors)
{
    QStringList messages;
    messages.reserve(errors.size());
    for (const QSslError &error : errors)
        messages.append(error.errorString());
    emit socketError(QStringLiteral("서버 인증서 검증 실패: %1")
                         .arg(messages.join(QStringLiteral(", "))));
}

// 접속 직후(암호화 완료) HELLO {role:"QT"} 를 1회 전송한다.
void ServerClient::onEncrypted()
{
    if (!m_helloSent) {
        QJsonObject payload; payload["role"] = "QT";
        sendJson("HELLO", payload);
        m_helloSent = true;
    }
    emit connectedToServer();
}

void ServerClient::onDisconnected()
{
    m_helloSent = false;
    emit disconnectedFromServer();
}

void ServerClient::sendJson(const QString &type, const QJsonObject &payload)
{
    if (!m_socket || m_socket->state() != QAbstractSocket::ConnectedState) return;
    QJsonObject root;
    root["type"] = type;
    root["seq"] = m_seq++;
    root["payload"] = payload;
    QByteArray line = QJsonDocument(root).toJson(QJsonDocument::Compact);
    line.append('\n'); // JSON Lines 프레이밍
    m_socket->write(line);
    m_socket->flush();
}

void ServerClient::sendLogin(const QString &id, const QString &pw)
{
    QJsonObject p; p["id"] = id; p["pw"] = pw;
    sendJson("LOGIN", p);
}

// cam_ip 는 선택. 서버는 검증 없이 저장만 하고 LOGIN_OK 로 그대로 돌려준다.
void ServerClient::sendRegister(const QString &id, const QString &pw, const QString &camIp)
{
    QJsonObject p; p["id"] = id; p["pw"] = pw;
    if (!camIp.trimmed().isEmpty())
        p["cam_ip"] = camIp.trimmed();
    sendJson("REGISTER", p);
}

// 바닥 평면 미터 좌표 폴리라인 → BLUEPRINT. (서버는 재변환하지 않음)
void ServerClient::sendBlueprint(const QList<QPointF> &meterPoints,
                                 const QList<bool> &paint,
                                 const QList<motionprogram::Op> &program)
{
    QJsonArray pts;
    for (const QPointF &p : meterPoints) {
        QJsonArray pair;
        pair.append(p.x());
        pair.append(p.y());
        pts.append(pair);
    }
    QJsonObject p; p["points"] = pts;
    // paint 는 points 와 길이가 같을 때만 싣는다. 서버는 길이가 다르면 경고 후
    // 전 구간 도색으로 fallback하므로, 모호한 배열을 보내지 않고 생략한다.
    // 정상 Qt 경로 생성에서는 항상 points와 같은 길이로 만든다.
    if (paint.size() == meterPoints.size() && !paint.isEmpty()) {
        QJsonArray flags;
        for (bool b : paint) flags.append(b);
        p["paint"] = flags;
    }
    // 도면 기준 논리 동작 — 서버 v2가 로봇 op으로 변환한다.
    //
    // ⚠️ 프로토콜에 **없는 필드는 절대 싣지 않는다** (v0.3 §program op 규약):
    //    · pen_offset_m — 폐지. 펜 오프셋 보정은 서버 v2와 로봇 실행부 전담.
    //    · pivot        — 폐지. program 은 도면 그대로의 논리 동작만.
    //    · speed_mps / speed_dps — "속도는 프로토콜에 없다"(로봇 펌웨어 고정값).
    //      Op::speed 는 화면 미리보기·예상시간 계산용 로컬 값으로만 남는다.
    if (!program.isEmpty()) {
        QJsonArray ops;
        for (const motionprogram::Op &o : program) {
            QJsonObject j;
            j["op"] = o.opName();
            j["v"] = o.vertex;                 // 필수 — 없으면 서버가 program 전체를 거부
            // heading_deg: MOVE=진행 방향, TURN=회전 후 방향, ARC=진입 접선(CCW+).
            j["heading_deg"] = std::round(o.heading * 10.0) / 10.0;
            switch (o.kind) {
            case motionprogram::Op::Move:
                j["dist_m"] = std::round(o.dist * 10000.0) / 10000.0;  // 음수 = 후진
                j["paint"] = o.paint;                                   // 표시용
                break;
            case motionprogram::Op::Turn:
                j["angle_deg"] = std::round(o.angle * 10.0) / 10.0;     // 양수 = 좌회전
                break;
            case motionprogram::Op::Arc:
                j["dist_m"]    = std::round(o.dist * 10000.0) / 10000.0;   // 호 길이 S=R·θ
                j["radius_m"]  = std::round(o.radius * 10000.0) / 10000.0; // 도면상 반지름
                j["angle_deg"] = std::round(o.angle * 10.0) / 10.0;        // 양수
                j["direction"] = o.arcDirection();                         // left / right
                j["paint"]     = o.paint;                                  // 표시용
                break;
            case motionprogram::Op::Nozzle:
                j["down"] = o.down;            // 노즐 제어의 유일한 수단
                break;
            }
            ops.append(j);
        }
        p["program"] = ops;
    }
    sendJson("BLUEPRINT", p);
}

void ServerClient::sendCmd(const QString &cmd)
{
    QJsonObject p; p["cmd"] = cmd;
    sendJson("CMD", p);
}

// ESTOP 만 보내던 예전 "작업 중단"은 사실 일시정지였다 — 서버의 경로 상태도
// 로봇의 세그먼트 커서도 남아서, RESUME 한 번이면 도색이 멈춘 지점부터 재개됐다.
// 이 명령을 받으면 서버와 로봇이 받아둔 경로를 버린다 (프로토콜 v0.4).
void ServerClient::sendAbortDraw()
{
    sendCmd(QStringLiteral("ABORT_DRAW"));
}

// CMD 에 ch 를 같이 싣는다 (프로토콜 v0.4). sendCmd 를 재사용하지 않는 이유는
// payload 에 필드가 하나 더 붙기 때문이고, 그 하나 때문에 sendCmd 시그니처에
// 채널 인자를 달면 채널과 무관한 호출 20곳이 다 지저분해진다.
void ServerClient::sendSelectChannel(int ch)
{
    QJsonObject p;
    p["cmd"] = QStringLiteral("SELECT_CHANNEL");
    p["ch"] = ch;
    sendJson("CMD", p);
}

void ServerClient::sendCalibStart(int ch, const QString &requestId,
                                 const QString &method, double mCm, double nCm,
                                 const QString &startCorner)
{
    QJsonObject p;
    p["cmd"] = QStringLiteral("CALIB_START");
    p["ch"] = ch;
    p["request_id"] = requestId;
    // 오도메트리 주행 방식만 4필드를 싣는다. method 가 없거나 다른 값이면
    // 서버는 기존 정적 앵커 경로로 간다 (요청서 §1).
    if (!method.isEmpty()) {
        p["method"] = method;
        if (method == QLatin1String("robot_motion")) {
            p["m_cm"] = mCm;
            p["n_cm"] = nCm;
            p["start_corner"] = startCorner;
        }
    }
    sendJson("CMD", p);
}

void ServerClient::sendCalibCancel(int ch, const QString &requestId)
{
    QJsonObject p;
    p["cmd"] = QStringLiteral("CALIB_CANCEL");
    p["ch"] = ch;
    p["request_id"] = requestId;
    sendJson("CMD", p);
}


// 로그인 상태에서만 동작한다. 서버는 IP 형식을 검사하지 않으므로 검증은 Qt 몫.
void ServerClient::sendSetCamIp(const QString &camIp)
{
    QJsonObject p; p["cam_ip"] = camIp.trimmed();
    sendJson("SET_CAM_IP", p);
}

// 수신 루프: '\n' 단위로 잘라 JSON 파싱, type 별 분기. 모르는 type 은 조용히 무시.
void ServerClient::onReadyRead()
{
    m_buffer.append(m_socket->readAll());
    qsizetype nl;
    while ((nl = m_buffer.indexOf('\n')) != -1) {
        QByteArray line = m_buffer.left(nl);
        m_buffer.remove(0, nl + 1);
        if (line.trimmed().isEmpty()) continue;

        QJsonParseError err{};
        QJsonDocument doc = QJsonDocument::fromJson(line, &err);
        if (err.error != QJsonParseError::NoError || !doc.isObject()) {
            qWarning() << "[ServerClient] JSON 파싱 실패:" << err.errorString();
            continue;
        }
        dispatch(doc.object());
    }
}

void ServerClient::dispatch(const QJsonObject &msg)
{
    const QString type = msg.value("type").toString();
    const QJsonObject payload = msg.value("payload").toObject();

    if (type == "ACK") {
        emit serverAck(payload.value("msg").toString());
    } else if (type == "LOGIN_OK") {
        // calib 가 null 이면 캘리브레이션 미완료 → 관리자 창 안내 대상
        const QJsonValue calibVal = payload.value("calib");
        const bool hasCalib = calibVal.isObject() && !calibVal.toObject().isEmpty();
        // 채널별 맵을 loginResult **보다 먼저** 흘린다 — Backend 가 로그인 처리
        // 안에서 채널 화면을 띄우는데, 그때 이 맵이 이미 있어야 "어느 채널이
        // 캘리 됐는지"를 그리드에 바로 표시할 수 있다.
        // 서버가 아직 저장한 번들이 없으면 빈 오브젝트가 전달된다.
        emit calibChannelsReceived(payload.value("calibs").toObject(),
                                   payload.value("active_ch").toInt(1));
        // 필드가 없으면 "서버가 모름"이므로 기존 설정을 유지한다. 명시적
        // enabled=false만 "중계 OFF, 직결 전환"으로 Backend에 전달한다.
        const QJsonValue streamValue = payload.value("stream");
        if (streamValue.isObject()) {
            const QJsonObject stream = streamValue.toObject();
            emit streamInfoReceived(stream.value("enabled").toBool(true),
                                    stream.value("base").toString(),
                                    stream.value("channels").toInt(0));
        }
        emit loginResult(true, payload.value("id").toString(),
                         calibVal.toObject(), hasCalib,
                         payload.value("cam_ip").toString(), QString());
    } else if (type == "LOGIN_FAIL") {
        emit loginResult(false, QString(), QJsonObject(), false, QString(),
                         payload.value("reason").toString());
    } else if (type == "REGISTER_OK") {
        emit registerResult(true, payload.value("id").toString(), QString());
    } else if (type == "REGISTER_FAIL") {
        emit registerResult(false, QString(), payload.value("reason").toString());
    } else if (type == "POSE") {
        emit poseReceived(payload.value("x").toDouble(),
                          payload.value("y").toDouble(),
                          payload.value("theta_deg").toDouble());
    } else if (type == "STATUS") {
        emit statusReceived(payload.value("state").toString(),
                            payload.value("painting").toBool());
    } else if (type == "PEERS") {
        emit peersReceived(payload.value("robot").toBool(),
                           payload.value("cctv").toBool());
    } else if (type == "H_MATRIX") {
        QJsonObject calib = payload.value("calib").toObject();
        // 평면 스키마는 payload 자체가 번들이다. K/D/H_marker 등의 보정
        // 메타데이터까지 보존하고 전송 envelope 필드만 제거한다.
        if (calib.isEmpty() && (payload.contains("H") || payload.contains("H_floor"))) {
            calib = payload;
            calib.remove("ch");
            calib.remove("request_id");
        }
        // 최상위 envelope가 우선이고, 없으면 calib 내부 메타데이터를 사용한다.
        int ch = payload.value("ch").toInt(0);
        if (ch == 0) ch = payload.value("calib").toObject().value("ch").toInt(0);
        QString requestId = payload.value("request_id").toString();
        if (requestId.isEmpty())
            requestId = payload.value("calib").toObject().value("request_id").toString();
        // ch=0은 Backend가 단일 pending 요청으로 안전하게 복원할 수 있으므로 전달한다.
        // 범위 밖 값만 프로토콜 오류로 거부한다.
        if (ch < 0 || ch > 4) {
            qWarning() << "[ServerClient] H_MATRIX 채널 범위 오류:" << ch;
            return;
        }
        if (ch == 0)
            qInfo() << "[ServerClient] H_MATRIX ch 누락 — Backend에서 pending 요청과 대조";
        emit hMatrixReceived(ch, calib, requestId);
    } else if (type == "CALIB_STARTED") {
        emit calibStarted(payload.value("ch").toInt(),
                          payload.value("request_id").toString(),
                          payload.value("msg").toString());
    } else if (type == "CALIB_PROGRESS") {
        double progress = payload.value("progress").toDouble(-1.0);
        if (progress > 1.0) progress /= 100.0;
        emit calibProgress(payload.value("ch").toInt(),
                           payload.value("request_id").toString(), progress,
                           payload.value("stage").toString(),
                           payload.value("msg").toString(),
                           payload.value("phase").toString(),
                           payload.value("point_index").toInt(-1),
                           payload.value("total").toInt(-1),
                           payload.value("valid").toInt(-1));
    } else if (type == "CALIB_FAIL") {
        emit calibFailed(payload.value("ch").toInt(),
                         payload.value("request_id").toString(),
                         payload.value("reason").toString(),
                         payload.value("msg").toString(),
                         payload.value("owner").toString());
    } else if (type == "CALIB_CANCELLED") {
        emit calibCancelled(payload.value("ch").toInt(),
                            payload.value("request_id").toString(),
                            payload.value("msg").toString());
    } else if (type == "CHANNEL_OK") {
        const QJsonObject calib = payload.value("calib").toObject();
        emit channelResult(true, payload.value("ch").toInt(), calib,
                           !calib.isEmpty(), QString());
    } else if (type == "CHANNEL_FAIL") {
        emit channelResult(false, 0, QJsonObject(), false,
                           payload.value("reason").toString());
    } else if (type == "BLUEPRINT_OK") {
        emit blueprintAck(payload.value("points").toInt(),
                          payload.value("paint").toBool(),
                          payload.value("program").toInt());
    } else if (type == "SET_CAM_IP_OK") {
        emit camIpResult(true, payload.value("cam_ip").toString(), QString());
    } else if (type == "SET_CAM_IP_FAIL") {
        emit camIpResult(false, QString(), payload.value("reason").toString());
    } else if (type == "DRAW_DONE") {
        emit drawDone();
    } else if (type == "DRAW_ABORTED") {
        emit drawAborted(payload.value("was_active").toBool());
    } else if (type == "DRAW_FAIL") {
        emit drawFailed(payload.value("stage").toString(),
                        payload.value("reason").toString(),
                        payload.value("msg").toString());
    }
    // 그 외 모르는 type 은 조용히 무시 (에러로 끊지 않음).
    // POS(마커 원본 픽셀)는 2026-07-27부터 QT 로 오지 않는다 — 분기 없음이 정상.
    // 로봇 위치는 POSE 하나로 충분하고, 마커 표시는 로컬 ArUco 검출이 담당한다.
}
