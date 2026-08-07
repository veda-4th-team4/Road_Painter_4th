#include "serverclient.h"

#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonValue>
#include <QHostAddress>
#include <QDebug>
#include <cmath>

ServerClient::ServerClient(QObject *parent)
    : QObject(parent)
{
    m_socket = new QSslSocket(this);
    // 1차 연동: 자가서명 인증서 검증 생략 (권장 방식은 server.crt 를 신뢰 CA 로 추가)
    m_socket->setPeerVerifyMode(QSslSocket::VerifyNone);

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
    if (isConnected()) return;
    m_seq = 0;
    m_helloSent = false;
    m_buffer.clear();
    // 자가서명 인증서라도 우선 접속을 진행하도록 에러를 무시한다.
    m_socket->ignoreSslErrors();
    m_socket->connectToHostEncrypted(m_host, m_port);
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
    Q_UNUSED(errors);
    // VerifyNone 이므로 계속 진행. (검증 강화 시 여기서 화이트리스트 처리)
    m_socket->ignoreSslErrors();
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
    int nl;
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
        // v0.3 서버는 calibs 를 안 보내므로 빈 오브젝트가 나가고, 그러면
        // channelMode 가 꺼진 단일 채널 경로라 아무도 안 본다.
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
        // v0.3: calib 번들. 레거시 {"H":[[..]x3]} 도 당분간 허용.
        QJsonObject calib = payload.value("calib").toObject();
        if (calib.isEmpty() && payload.contains("H")) {
            calib = QJsonObject{ { "H", payload.value("H") } };
        }
        // 평면 스키마는 payload 자체가 번들이라 여기서 calib 가 비어 있다.
        // 그 경우 payload 를 그대로 번들로 본다 (Backend 가 정규화한다).
        if (calib.isEmpty() && payload.contains("H_floor"))
            calib = payload;
        // ch 없으면 1 (단일 채널 카메라·v0.3 서버 하위호환)
        emit hMatrixReceived(payload.value("ch").toInt(1), calib);
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
