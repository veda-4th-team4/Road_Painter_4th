#ifndef SERVERCLIENT_H
#define SERVERCLIENT_H

#include <QObject>
#include <QSslSocket>
#include <QByteArray>
#include <QString>
#include <QJsonObject>
#include <QList>
#include <QPointF>
#include <QVariantList>

#include "motionprogram.h"

// 💡 중앙 서버(TLS 9000, role=QT) 접속 클라이언트.
//   기준 문서: Server/docs/QT_CLIENT_SPEC.md + Server/src/protocol.hpp (2026-07-27판).
//     - 접속 직후 HELLO {role:"QT"} 1회 전송 → ACK
//     - REGISTER {id,pw,cam_ip?} / LOGIN {id,pw}
//       · LOGIN_OK {id, calib|null, cam_ip|null}  ※ calib=null 이면 캘리브레이션 필요
//     - BLUEPRINT (도면 = 바닥 평면 미터 좌표) — 서버는 "저장만" 한다
//     - CMD {START_DRAW | ESTOP | RESUME | 조이스틱}
//     - SET_CAM_IP {cam_ip} → SET_CAM_IP_OK/FAIL
//     - POSE / STATUS / PEERS / H_MATRIX / DRAW_DONE / DRAW_FAIL 수신
//   메시지 프레이밍: JSON Lines (JSON 1개 + '\n' = 1메시지)
//   서버 자가서명 인증서는 실행파일에 내장해 체인과 접속 주소를 모두 검증한다.
//
//   ⚠️ 2026-07-27 변경 (이전 v0.3 문서와 다름 — 꼭 확인):
//     · BLUEPRINT 를 보내도 로봇은 움직이지 않는다. 실행은 START_DRAW 부터.
//     · START_DRAW 하나로 접근 → 도색 → 완료까지 서버가 전부 자동 진행한다.
//       "접근 완료"는 QT 에 통지되지 않으므로 2단계 버튼을 만들지 말 것.
//     · 끝은 DRAW_DONE 하나로만 온다.
//     · POS(CCTV 원본 픽셀)는 더 이상 QT 로 오지 않는다 → 수신 분기 없음.
//   QT 는 선택 채널의 로봇 주행 호모그래피를 CALIB_START 로 요청하고,
//   서버/CCTV가 계산한 H_MATRIX 결과만 적용한다. 이동·계산 로직은 QT에 없다.
class ServerClient : public QObject
{
    Q_OBJECT
public:
    explicit ServerClient(QObject *parent = nullptr);
    ~ServerClient() override;

    // 서버에 접속(암호화)한다. 이미 연결돼 있으면 무시.
    void connectToServer();
    void disconnectFromServer();
    void setServer(const QString &host, quint16 port);
    QString host() const { return m_host; }
    quint16 port() const { return m_port; }

    bool isConnected() const;
    bool isEncrypted() const;

    // Qt → 서버 메시지
    void sendLogin(const QString &id, const QString &pw);
    void sendRegister(const QString &id, const QString &pw, const QString &camIp = QString());
    // 바닥 평면 미터 좌표 폴리라인 전송 (top-view 픽셀 ÷ S 를 마친 값이어야 함)
    //   paint: 점마다 "직전 점 → 이 점" 구간을 칠하는지. 도형 사이 이동 구간을
    //          false 로 표시해 로봇이 펜을 든 채 지나가게 한다.
    //          비워서 보내면 서버는 예전처럼 전 구간 도색으로 취급한다(하위 호환).
    //   program: Qt가 만든 도면 기준 논리 동작. 서버 v2가 로봇 op으로 변환한다.
    //          비워 보내면 서버가 points에서 논리 동작을 직접 만든다(하위 호환).
    //   ⚠️ pen_offset_m 인자는 없앴다 — 2026-07-28 프로토콜에서 폐지된 필드다.
    void sendBlueprint(const QList<QPointF> &meterPoints,
                       const QList<bool> &paint = QList<bool>(),
                       const QList<motionprogram::Op> &program = {});
    void sendCmd(const QString &cmd);
    // 작업 완전 중지 (프로토콜 v0.4). ESTOP 과 달리 서버·로봇이 받아둔 경로를
    // 버리므로, RESUME 해도 도색이 재개되지 않는다. 도면은 서버에 남는다.
    void sendAbortDraw();
    // 작업 채널 전환 (프로토콜 v0.4, 다채널 카메라). 서버가 활성 채널을 바꾸고
    // CCTV 에 중계해 그 채널의 마커를 잡게 한다 → 응답 CHANNEL_OK{ch,calib}.
    // ⚠️ 영상만 바꾸고 이걸 안 보내면 로봇 위치가 옛 채널 기준이라 조용히 어긋난다.
    void sendSelectChannel(int ch);
    // 선택 채널의 로봇 주행 호모그래피 시작/취소. requestId는 늦게 도착한
    // 이전 결과를 현재 요청으로 오인하지 않기 위한 상관관계 키다.
    void sendCalibStart(int ch, const QString &requestId);
    void sendCalibCancel(int ch, const QString &requestId);
    // ⚠️ sendSpeeds 는 없앴다 — 프로토콜에 속도 CMD 가 없고(2026-07-28 확정),
    //    로봇에도 SET_SPEED 분기가 없어 보내봐야 버려진다.
    // 로그인 후 카메라 IP 변경 (빈 문자열이면 등록 해제)
    void sendSetCamIp(const QString &camIp);

signals:
    void connectedToServer();               // TLS 핸드셰이크 완료 + HELLO 전송 후
    void disconnectedFromServer();
    void socketError(const QString &message);
    void serverAck(const QString &msg);     // HELLO 등록 확인

    // 인증 결과. hasCalib=false 면 서버에 저장된 캘리브레이션이 없음(=관리자 창 안내 대상)
    void loginResult(bool ok, const QString &id, const QJsonObject &calib,
                     bool hasCalib, const QString &camIp, const QString &reason);
    void registerResult(bool ok, const QString &id, const QString &reason);

    // 실시간 수신
    void poseReceived(double x, double y, double thetaDeg);
    void statusReceived(const QString &state, bool painting);
    void peersReceived(bool robot, bool cctv);
    // 캘리브레이션 갱신 → top-view 재생성. ch 는 이 번들이 어느 채널의 것인지
    // (단일 채널 카메라·v0.3 서버는 ch 를 안 실으므로 1로 온다).
    void hMatrixReceived(int ch, const QJsonObject &calib, const QString &requestId);
    void calibStarted(int ch, const QString &requestId, const QString &message);
    void calibProgress(int ch, const QString &requestId, double progress,
                       const QString &stage, const QString &message);
    void calibFailed(int ch, const QString &requestId, const QString &reason,
                     const QString &message);
    void calibCancelled(int ch, const QString &requestId, const QString &message);
    // LOGIN_OK 의 채널별 번들 맵 + 서버가 기억하는 활성 채널 (프로토콜 v0.4).
    // loginResult 와 따로 두는 이유: 단일 채널 경로는 calib 하나만 보면 되고,
    // 그 시그니처를 건드리면 4채널과 무관한 코드까지 다 바뀐다.
    void calibChannelsReceived(const QJsonObject &calibs, int activeCh);
    // LOGIN_OK 의 선택 stream 오브젝트. 필드 자체가 없으면 signal도 내지 않아 기존
    // QSettings를 유지한다. 명시적 enabled=false는 저장된 중계를 해제하라는 계약이다.
    void streamInfoReceived(bool enabled, const QString &base, int channels);
    // 채널 전환 결과. ok=false 면 서버가 거절한 것(범위 밖 채널 등).
    // hasCalib=false 면 그 채널은 아직 캘리브레이션이 없다.
    void channelResult(bool ok, int ch, const QJsonObject &calib, bool hasCalib,
                       const QString &reason);
    // 도면 접수 확인 — 보낸 것과 서버가 받은 것을 그 자리에서 대조한다.
    // paint/program 이 형식 오류로 무시됐으면 false / 0 으로 온다.
    void blueprintAck(int points, bool paint, int program);
    // 카메라 IP 변경 결과
    void camIpResult(bool ok, const QString &camIp, const QString &reason);
    // 도색 완료 — START_DRAW 이후 정상적인 "끝" 신호
    void drawDone();
    // 작업 취소 확인 (ABORT_DRAW 응답). wasActive=false 면 진행 중인 작업이
    // 없었다는 뜻 — 어느 쪽이든 UI 는 "그리는 중" 표시를 끄면 된다.
    void drawAborted(bool wasActive);
    // 경로 생성/전송 실패 또는 대기 통지
    void drawFailed(const QString &stage, const QString &reason, const QString &msg);

private slots:
    void onEncrypted();
    void onReadyRead();
    void onDisconnected();
    void onErrors(const QList<QSslError> &errors);

private:
    void sendJson(const QString &type, const QJsonObject &payload);
    void dispatch(const QJsonObject &msg);

    QSslSocket *m_socket = nullptr;
    QByteArray  m_buffer;   // 개행 단위 프레이밍용 수신 버퍼
    int         m_seq = 0;  // 연결마다 0부터 증가 (로그 추적용)
    QString     m_host = QStringLiteral("192.168.0.8");
    quint16     m_port = 9000;
    bool        m_helloSent = false;
    bool        m_tlsCertificateReady = false;
};

#endif // SERVERCLIENT_H
