#ifndef BACKEND_H
#define BACKEND_H

#include <QObject>
#include <QString>
#include <QJsonObject>
#include <QJsonArray>
#include <QDateTime>
#include <QList>
#include <QPointF>
#include <QElapsedTimer>
#include <QVariantList>
#include <QPolygonF>
#include <QHash>

#include "routeplan.h"
#include "motionprogram.h"
#include "motionsim.h"
#include "camcalib.h"

class video_worker;
class preview_worker;
class ChannelTile;
class ServerClient;
class VideoView;
class QTimer;

class Backend : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString userId READ userId NOTIFY sessionChanged)
    Q_PROPERTY(bool testMode READ testMode NOTIFY sessionChanged)
    Q_PROPERTY(QString camIp READ camIp NOTIFY sessionChanged)
    Q_PROPERTY(bool drawing READ drawing NOTIFY drawingChanged)
    Q_PROPERTY(QString robotStatus READ robotStatus NOTIFY robotStatusChanged)
    Q_PROPERTY(QString robotState READ robotState NOTIFY robotStatusChanged)
    Q_PROPERTY(bool painting READ painting NOTIFY robotStatusChanged)
    Q_PROPERTY(bool nozzleDown READ nozzleDown NOTIFY robotStatusChanged)
    Q_PROPERTY(QString robotLog READ robotLog NOTIFY robotLogChanged)
    Q_PROPERTY(bool busy READ busy NOTIFY busyChanged)

    Q_PROPERTY(bool serverConnected READ serverConnected NOTIFY linkStatusChanged)
    Q_PROPERTY(QString serverLabel READ serverLabel NOTIFY linkStatusChanged)
    Q_PROPERTY(QString serverAddress READ serverAddress NOTIFY linkStatusChanged)
    Q_PROPERTY(QString adminConsoleUrl READ adminConsoleUrl NOTIFY linkStatusChanged)
    Q_PROPERTY(bool robotOnline READ robotOnline NOTIFY linkStatusChanged)
    Q_PROPERTY(bool cctvOnline READ cctvOnline NOTIFY linkStatusChanged)
    Q_PROPERTY(double cctvFps READ cctvFps NOTIFY linkStatusChanged)
    Q_PROPERTY(double cctvLatencyMs READ cctvLatencyMs NOTIFY linkStatusChanged)

    Q_PROPERTY(double poseX READ poseX NOTIFY poseChanged)
    Q_PROPERTY(double poseY READ poseY NOTIFY poseChanged)
    Q_PROPERTY(bool poseValid READ poseValid NOTIFY poseChanged)
    // ⚠️ poseTheta 프로퍼티는 지웠다 — QML 이 poseX/poseY 만 쓴다. 로봇 방위는
    //    화면에 숫자로 안 나오고 VideoView 가 setRobotPose 로 받아 직접 그린다.
    //    (m_poseTheta 자체는 그 경로에서 계속 쓰인다 — 멤버는 남아 있다)

    Q_PROPERTY(bool jobActive READ jobActive NOTIFY jobChanged)
    Q_PROPERTY(bool abortPending READ abortPending NOTIFY jobChanged)
    // ⚠️ blueprintSent 프로퍼티도 지웠다 — QML 은 이걸 직접 안 읽고 canStart 를 본다.
    //    (m_blueprintSent 는 canStart 안에서 계속 쓰인다)
    Q_PROPERTY(bool manualEnabled READ manualEnabled NOTIFY jobChanged)
    Q_PROPERTY(bool canEditMission READ canEditMission NOTIFY jobChanged)
    Q_PROPERTY(double jobProgress READ jobProgress NOTIFY jobChanged)
    Q_PROPERTY(int jobPercent READ jobPercent NOTIFY jobChanged)
    Q_PROPERTY(QString phase READ phase NOTIFY jobChanged)
    Q_PROPERTY(QString phaseHint READ phaseHint NOTIFY jobChanged)
    Q_PROPERTY(bool canCommit READ canCommit NOTIFY jobChanged)
    Q_PROPERTY(bool canStart READ canStart NOTIFY jobChanged)
    Q_PROPERTY(bool hasPath READ hasPath NOTIFY jobChanged)
    Q_PROPERTY(double pathLengthM READ pathLengthM NOTIFY jobChanged)
    // 도형 사이를 칠하지 않고 지나가는 빈 이동 거리 (0 이면 한붓그리기)
    Q_PROPERTY(double travelLengthM READ travelLengthM NOTIFY jobChanged)
    Q_PROPERTY(double remainingM READ remainingM NOTIFY jobChanged)
    Q_PROPERTY(QString elapsedText READ elapsedText NOTIFY jobChanged)
    Q_PROPERTY(QString etaText READ etaText NOTIFY jobChanged)
    Q_PROPERTY(int waypointIndex READ waypointIndex NOTIFY jobChanged)
    Q_PROPERTY(int waypointCount READ waypointCount NOTIFY jobChanged)
    Q_PROPERTY(QString workName READ workName NOTIFY jobChanged)
    Q_PROPERTY(QString calibStatus READ calibStatus NOTIFY calibChanged)
    Q_PROPERTY(bool calibMissing READ calibMissing NOTIFY calibChanged)
    // ⚠️ calibId / coordMode / calibSource 프로퍼티는 지웠다 — 셋 다 QML 이 안 읽는다.
    //    화면에는 이미 calibStatus 하나로 합쳐서 나가고(backend.cpp 의 calibStatus 조립),
    //    개별 노출은 중복이었다. 멤버 m_calibId/m_coordMode/m_calibSource 는 그 조립과
    //    렌즈보정 판정에 계속 쓰이므로 남아 있다.
    Q_PROPERTY(QString scaleText READ scaleText NOTIFY calibChanged)
    // ── 4채널 카메라 (PNM-C16083RVQ) ─────────────────────────────────────
    Q_PROPERTY(QString relayBase READ relayBase NOTIFY channelChanged)
    // 화면 표시용 — "중계 …" 또는 "카메라 직결 …". 비밀번호는 가려져 있다.
    Q_PROPERTY(QString streamSourceText READ streamSourceText NOTIFY channelChanged)
    Q_PROPERTY(int channelCount READ channelCount NOTIFY channelChanged)
    // 클릭해서 테두리가 켜진 채널 (0 = 아무것도 안 고름). 아직 작업 시작 전이다.
    Q_PROPERTY(int highlightedChannel READ highlightedChannel NOTIFY channelChanged)
    // [작업하기]로 들어간 채널 (0 = 4채널 그리드 화면). 이 값이 화면을 가른다.
    Q_PROPERTY(int workingChannel READ workingChannel NOTIFY channelChanged)
    Q_PROPERTY(bool canStartChannelWork READ canStartChannelWork NOTIFY channelChanged)
    // 캘리브레이션이 끝난 채널 번호 목록. QML 바인딩용이다 — channelCalibrated() 는
    // Q_INVOKABLE 이라 캘리가 새로 들어와도 바인딩이 다시 계산되지 않는다.
    Q_PROPERTY(QVariantList calibratedChannels READ calibratedChannels NOTIFY channelChanged)
    Q_PROPERTY(bool homographyPending READ homographyPending NOTIFY homographyChanged)
    Q_PROPERTY(int homographyChannel READ homographyChannel NOTIFY homographyChanged)
    Q_PROPERTY(double homographyProgress READ homographyProgress NOTIFY homographyChanged)
    Q_PROPERTY(QString homographyStatus READ homographyStatus NOTIFY homographyChanged)
    Q_PROPERTY(bool homographyCancelPending READ homographyCancelPending NOTIFY homographyChanged)
    // 오도메트리 주행 캘리브레이션 — 대기 화면이 "로봇이 움직이는 중"임을 구분해
    // 보여주고, 정지점 진행(9개 중 n번째)을 표시하기 위한 값들.
    Q_PROPERTY(bool homographyOdometry READ homographyOdometry NOTIFY homographyChanged)
    Q_PROPERTY(bool homographyCaptureLag READ homographyCaptureLag NOTIFY homographyChanged)
    Q_PROPERTY(bool homographyCancelUnconfirmed READ homographyCancelUnconfirmed NOTIFY homographyChanged)
    Q_PROPERTY(QString homographyPhase READ homographyPhase NOTIFY homographyChanged)
    Q_PROPERTY(int homographyPointIndex READ homographyPointIndex NOTIFY homographyChanged)
    Q_PROPERTY(int homographyPointTotal READ homographyPointTotal NOTIFY homographyChanged)
    Q_PROPERTY(int homographyValidPoints READ homographyValidPoints NOTIFY homographyChanged)
    Q_PROPERTY(double odoWidthCm READ odoWidthCm WRITE setOdoWidthCm NOTIFY homographyChanged)
    Q_PROPERTY(double odoHeightCm READ odoHeightCm WRITE setOdoHeightCm NOTIFY homographyChanged)
    Q_PROPERTY(bool odoCcw READ odoCcw WRITE setOdoCcw NOTIFY homographyChanged)

    // 서버 통지(DRAW_FAIL 등) 배너
    Q_PROPERTY(QString notice READ notice NOTIFY noticeChanged)
    Q_PROPERTY(QString noticeLevel READ noticeLevel NOTIFY noticeChanged)

    // 카메라 영상이 없을 때의 오프라인 작도 모드
    Q_PROPERTY(bool offlineCanvas READ offlineCanvas NOTIFY linkStatusChanged)

    // ArUco 마커 상시 검출
    Q_PROPERTY(QString arucoSummary READ arucoSummary NOTIFY arucoChanged)
    Q_PROPERTY(bool arucoOverlay READ arucoOverlay WRITE setArucoOverlay NOTIFY arucoChanged)
    // 로봇 도장 폭 (mm) — 고정 사양이지만 기체가 바뀌면 조정
    Q_PROPERTY(double strokeWidthMm READ strokeWidthMm WRITE setStrokeWidthMm NOTIFY strokeWidthChanged)
    // TopView의 마커 중심↔펜 표시 거리. BLUEPRINT 및 주행 보정에는 사용하지 않는다.
    Q_PROPERTY(double penDisplayOffsetMm READ penDisplayOffsetMm WRITE setPenDisplayOffsetMm
               NOTIFY penDisplayOffsetChanged)
    Q_PROPERTY(int videoBrightness READ videoBrightness NOTIFY videoFiltersChanged)
    Q_PROPERTY(int videoContrast READ videoContrast NOTIFY videoFiltersChanged)
    Q_PROPERTY(int videoSharpen READ videoSharpen NOTIFY videoFiltersChanged)
    Q_PROPERTY(int videoSaturation READ videoSaturation NOTIFY videoFiltersChanged)
    // 로봇 속도 — 도색 중에는 도료가 고르게 깔려야 해서 이동보다 느리다
    Q_PROPERTY(double travelSpeedMps READ travelSpeedMps WRITE setTravelSpeedMps NOTIFY speedChanged)
    Q_PROPERTY(double paintSpeedMps READ paintSpeedMps WRITE setPaintSpeedMps NOTIFY speedChanged)
    Q_PROPERTY(double turnSpeedDps READ turnSpeedDps WRITE setTurnSpeedDps NOTIFY speedChanged)
    // 위 속도로 계산한 이번 도면의 예상 소요 시간
    Q_PROPERTY(QString planTimeText READ planTimeText NOTIFY jobChanged)
    // 테스트 모드 시뮬레이션 — 동작 시퀀스를 실제 속도로 재생한다
    Q_PROPERTY(bool lensCorrection READ lensCorrection WRITE setLensCorrection NOTIFY lensChanged)
    Q_PROPERTY(bool lensReady READ lensReady NOTIFY lensChanged)
    // TopView 위 로봇 아이콘 표시 — 작도할 때 도면을 가려서 끌 수 있어야 한다
    Q_PROPERTY(bool robotVisible READ robotVisible WRITE setRobotVisible NOTIFY robotVisibleChanged)
    Q_PROPERTY(QString lensSummary READ lensSummary NOTIFY lensChanged)
    Q_PROPERTY(double simSpeedFactor READ simSpeedFactor WRITE setSimSpeedFactor NOTIFY simChanged)
    Q_PROPERTY(bool simRunning READ simRunning NOTIFY simChanged)
    Q_PROPERTY(QString simPhase READ simPhase NOTIFY simChanged)

    // 프로토콜 PATH 미리보기 (MOVE/TURN 시퀀스) + 작업 이력
    Q_PROPERTY(QVariantList motionPlan READ motionPlan NOTIFY jobChanged)
    Q_PROPERTY(int turnCount READ turnCount NOTIFY jobChanged)
    Q_PROPERTY(QVariantList jobHistory READ jobHistory NOTIFY historyChanged)
    Q_PROPERTY(int historyCount READ historyCount NOTIFY historyChanged)

public:
    explicit Backend(QObject *parent = nullptr);
    ~Backend() override;

    QString userId() const { return m_userId; }
    bool testMode() const { return m_testMode; }
    QString camIp() const { return m_camIp; }
    bool drawing() const { return m_drawing; }
    QString robotStatus() const { return m_robotStatus; }
    QString robotState() const { return m_robotState; }
    bool painting() const { return m_painting; }
    // 로봇이 노즐 상태를 STATUS 로 알려주지 않으므로 마지막으로 보낸 명령을 기억한다
    bool nozzleDown() const { return m_nozzleDown; }
    QString robotLog() const { return m_robotLog; }
    // ⚠️ keyboardControl 프로퍼티/게터/세터는 지웠다 — 아무도 안 읽고 안 썼다.
    //    (근거는 backend.cpp 의 같은 자리 주석)
    bool busy() const { return m_busy; }

    bool serverConnected() const { return m_serverConnected; }
    QString serverLabel() const { return m_serverLabel; }
    QString serverAddress() const;
    QString adminConsoleUrl() const;
    bool robotOnline() const { return m_robotOnline; }
    bool cctvOnline() const { return m_cctvOnline; }
    double cctvFps() const { return m_cctvFps; }
    double cctvLatencyMs() const { return m_cctvLatencyMs; }

    double poseX() const { return m_poseX; }
    double poseY() const { return m_poseY; }
    bool poseValid() const { return m_poseValid; }

    bool jobActive() const { return m_jobActive; }
    bool abortPending() const { return m_abortPending; }
    // 경로 실행(접근+도색) 중에는 서버가 QT 수동조작을 무시한다 → UI도 잠근다
    bool manualEnabled() const { return !m_jobActive && !m_homographyPending; }
    bool canEditMission() const {
        int n = 0;
        for (const auto &p : m_missionPaths) n += p.size();
        return n >= 2;
    }
    double jobProgress() const { return m_jobProgress; }
    int jobPercent() const { return int(m_jobProgress * 100.0 + 0.5); }
    QString phase() const { return m_phase; }
    QString phaseHint() const { return m_phaseHint; }
    bool canCommit() const { return !m_jobActive && pathPointCount() >= 2; }
    // START_DRAW 를 눌러도 되는 상태인가 (연타 방지 — 서버도 busy 로 막지만 UI 가 먼저)
    bool canStart() const { return !m_jobActive && m_blueprintSent && canEditMission(); }
    bool hasPath() const { return pathPointCount() >= 1; }
    double pathLengthM() const { return m_pathLengthM; }
    double travelLengthM() const { return m_travelLengthM; }
    double remainingM() const { return m_pathLengthM * (1.0 - m_jobProgress); }
    QString elapsedText() const;
    QString etaText() const;
    int waypointIndex() const { return m_waypointIndex; }
    int waypointCount() const { return m_waypointCount; }
    QString workName() const { return m_workName; }
    QString calibStatus() const { return m_calibStatus; }
    bool calibMissing() const { return m_calibMissing; }
    QString scaleText() const;
    // ── 다채널 ────────────────────────────────────────────────────────────
    QString relayBase() const { return m_relayBase; }
    // 지금 어디서 영상을 받고 있는지 한 줄로. 중계인지 카메라 직결인지 화면에
    // 보여주기 위한 것 — relayBase 만 표시하면 직결일 때 빈칸이라 "설정이 안 됐나"
    // 로 보인다. ⚠️ URL 에 계정이 들어 있으므로 **비밀번호는 가려서** 낸다.
    QString streamSourceText() const;
    int channelCount() const { return m_channelCount; }
    int highlightedChannel() const { return m_highlightedCh; }
    int workingChannel() const { return m_workingCh; }
    // 채널을 고르기만 하면 들어갈 수 있다.
    // 캘리브레이션 전에도 영상 확인과 수동 조작은 필요하므로 진입은 허용하고,
    // 좌표가 정확하지 않을 수 있다는 경고를 표시한다.
    bool canStartChannelWork() const {
        return m_highlightedCh > 0;
    }
    // 그 채널에 캘리브레이션 번들이 있는가 (LOGIN_OK.calibs / CHANNEL_OK 기준)
    Q_INVOKABLE bool channelCalibrated(int ch) const;
    QVariantList calibratedChannels() const;
    // 중계 주소 변경. 빈 문자열이면 .13 카메라 직결 템플릿으로 돌아간다.
    Q_INVOKABLE void setRelayBase(const QString &base);
    // 타일 클릭 — 하이라이트만 바뀐다 (스트림은 이미 4개 다 흐르는 중)
    Q_INVOKABLE void highlightChannel(int ch);
    // [작업하기] — 미리보기를 끄고 그 채널 메인스트림 1개 + 마커검출로 전환
    Q_INVOKABLE void startChannelWork();
    // [◀ 채널 목록] — 작업 화면에서 4채널 그리드로 복귀
    Q_INVOKABLE void showChannelGrid();
    // [새로고침] — 지금 보고 있는 스트림을 **완전히 끊고 다시 연다.**
    // 평소에는 필요 없다(끊기면 워커가 알아서 다시 붙는다). 이건 그래도 화면이
    // 안 돌아올 때 쓰는 수동 탈출구다 — 한 채널만 죽어 있거나, 카메라 설정을
    // 바꾼 직후처럼 스트림 구성 자체가 달라졌을 때.
    // ⚠️ 다시 여는 데 채널당 약 1.1초가 들고 4채널은 직렬이라 4.6초쯤 걸린다.
    //    그래서 화면 전환에는 쓰지 않는다 (그쪽은 일시정지/재개로 처리한다).
    Q_INVOKABLE void refreshStreams();
    // ChannelGrid.qml 의 타일을 Backend 에 등록한다 (프레임을 밀어 넣기 위해)
    Q_INVOKABLE void registerTile(ChannelTile *tile, int ch);
    bool homographyPending() const { return m_homographyPending; }
    int homographyChannel() const { return m_homographyCh; }
    double homographyProgress() const { return m_homographyProgress; }
    QString homographyStatus() const { return m_homographyStatus; }
    bool homographyCancelPending() const { return m_homographyCancelPending; }
    bool homographyOdometry() const { return m_homographyOdometry; }
    // Backend 알림과 QML 경고색이 **같은 규칙**을 쓰도록 한 곳에서만 판단한다.
    bool homographyCaptureLag() const {
        return camcalib::captureLagWarning(m_homographyPointIndex, m_homographyValidPoints);
    }
    // 요청은 했는데 서버 확인이 아직/끝내 없는 상태 (QML 경고 표시용)
    bool homographyCancelUnconfirmed() const {
        return m_homographyCancelRequested && !m_homographyCancelPending;
    }
    QString homographyPhase() const { return m_homographyPhase; }
    int homographyPointIndex() const { return m_homographyPointIndex; }
    int homographyPointTotal() const { return m_homographyPointTotal; }
    int homographyValidPoints() const { return m_homographyValidPoints; }
    double odoWidthCm() const { return m_odoMCm; }
    double odoHeightCm() const { return m_odoNCm; }
    bool odoCcw() const { return m_odoCcw; }
    void setOdoWidthCm(double v) { if (v == m_odoMCm) return; m_odoMCm = v; emit homographyChanged(); }
    void setOdoHeightCm(double v) { if (v == m_odoNCm) return; m_odoNCm = v; emit homographyChanged(); }
    void setOdoCcw(bool v) { if (v == m_odoCcw) return; m_odoCcw = v; emit homographyChanged(); }
    Q_INVOKABLE bool startHomography(int ch, bool odometry = false);
    // 오도메트리 주행 캘리브레이션 (요청서 20260813 §1). ccw=true → bottom_left.
    Q_INVOKABLE bool startOdometryHomography(int ch, double mCm, double nCm, bool ccw);
    Q_INVOKABLE void cancelHomography();

    QString notice() const { return m_notice; }
    QString noticeLevel() const { return m_noticeLevel; }
    QString arucoSummary() const { return m_arucoSummary; }
    bool arucoOverlay() const { return m_arucoOverlay; }
    void setArucoOverlay(bool on);
    double strokeWidthMm() const { return m_strokeWidthMm; }
    double penDisplayOffsetMm() const { return m_penDisplayOffsetMm; }
    void setPenDisplayOffsetMm(double mm);
    int videoBrightness() const { return m_brightness; }
    int videoContrast() const { return m_contrast; }
    int videoSharpen() const { return m_sharpen; }
    int videoSaturation() const { return m_saturation; }
    double travelSpeedMps() const { return m_speeds.travelMps; }
    double paintSpeedMps() const { return m_speeds.paintMps; }
    double turnSpeedDps() const { return m_speeds.turnDps; }
    void setTravelSpeedMps(double v);
    void setPaintSpeedMps(double v);
    void setTurnSpeedDps(double v);
    QString planTimeText() const;
    // TopView 배경에 렌즈 보정을 적용할지. 끄면 예전 동작(호모그래피만) 그대로라
    // 켜고 끄며 어느 쪽이 잘 펴지는지 눈으로 비교할 수 있다.
    bool lensCorrection() const { return m_lensOn; }
    Q_INVOKABLE void setLensCorrection(bool on);
    // 로봇 아이콘 표시. 265mm 기체를 900×600 도면에 실축으로 그리면 캔버스의 1/4 라
    // 로봇이 도면 위에 서 있으면 그리던 선이 통째로 가려진다. 그래서 끌 수 있게 뒀다.
    bool robotVisible() const { return m_robotVisible; }
    Q_INVOKABLE void setRobotVisible(bool on);
    // 렌즈 파라미터가 실제로 들어와 있는지 (없으면 켜도 아무 일이 없다)
    bool lensReady() const { return m_cam.valid && m_cam.hasDistortion(); }
    QString lensSummary() const { return camcalib::describe(m_cam); }
    double simSpeedFactor() const { return m_simSpeedFactor; }
    void setSimSpeedFactor(double v);
    bool simRunning() const { return m_simRunning; }
    QString simPhase() const { return m_simPhase; }
    // 로봇에 현재 속도를 알린다 (수동 조작도 이 속도를 따르게)
    Q_INVOKABLE void pushSpeeds();
    void setStrokeWidthMm(double mm);
    bool offlineCanvas() const { return !m_gotRealFrame; }

    Q_INVOKABLE void login(const QString &id, const QString &pw);
    Q_INVOKABLE void registerUser(const QString &id, const QString &pw,
                                  const QString &camIp = QString());
    Q_INVOKABLE void logout();
    Q_INVOKABLE void reconnectServer();
    Q_INVOKABLE void setServerAddress(const QString &host, int port);

    Q_INVOKABLE void registerView(VideoView *view, bool topView);

    Q_INVOKABLE void startDrawSession(bool preset);
    Q_INVOKABLE void addPreset(const QString &type);
    Q_INVOKABLE void undo();
    Q_INVOKABLE int  pathPointCount() const;
    Q_INVOKABLE void commitDrawing();     // BLUEPRINT 전송 → 로봇 접근 단계
    Q_INVOKABLE void startPainting();     // CMD START_DRAW → 도색 시작
    // ABORT_DRAW를 보내고 DRAW_ABORTED를 받은 뒤에만 로컬 작업 상태를 종료한다.
    // 즉시 일시정지하는 ESTOP은 별도 버튼(toggleEstop)으로 유지한다.
    Q_INVOKABLE void cancelJob();
    Q_INVOKABLE void editMission();       // 전송한 경로를 다시 편집 상태로
    Q_INVOKABLE void finishDrawing();
    Q_INVOKABLE void cancelDrawing();
    Q_INVOKABLE void clearMission();

    Q_INVOKABLE void sendRobotCmd(const QString &cmd, const QString &statusText);
    // 노즐 올림/내림 — 서버는 START_DRAW 외의 CMD 를 그대로 ROBOT 에 중계한다
    Q_INVOKABLE void setNozzle(bool down);
    Q_INVOKABLE void toggleEstop();
    // 단축키(Space)용 — **거는 방향으로만** 동작한다. 토글이면 이미 정지된 상태에서
    // 한 번 더 눌렀을 때 비상정지가 풀린다(안전 역행). 이미 걸려 있으면 아무것도
    // 보내지 않고 안내만 남긴다.
    Q_INVOKABLE void engageEstop();
    Q_INVOKABLE void clearRobotLog();
    Q_INVOKABLE void dismissNotice();

    Q_INVOKABLE void setRtsp(const QString &url);
    Q_INVOKABLE void setCamIp(const QString &ip);   // IP → RTSP URL 조립 (로컬만)
    Q_INVOKABLE void applyCamIp(const QString &ip); // 로컬 + 서버 저장(SET_CAM_IP)
    Q_INVOKABLE void setVideoFilters(int brightness, int contrast, int sharpen, int saturation);

    // 캘리브(H/코너 JSON) · 월드 mm 작도
    Q_INVOKABLE QString applyCalibJson(const QString &jsonText);
    Q_INVOKABLE void addWorldPointMm(double xMm, double yMm);
    Q_INVOKABLE void addRectWorldMm(double widthMm, double heightMm);
    Q_INVOKABLE void addTextWorldMm(const QString &text, double heightMm, bool outline = false);
    Q_INVOKABLE void simplifyPaths(double toleranceMm);
    // 도형 변환 (선택된 점이 있으면 그것만)
    Q_INVOKABLE void rotateShape(double deg);
    Q_INVOKABLE void flipShape(bool horizontal);
    Q_INVOKABLE void scaleShape(double factor);

    // 프로토콜 PATH 미리보기 — 서버가 만들 MOVE/TURN 시퀀스를 그대로 보여준다
    QVariantList motionPlan() const;
    int turnCount() const;

    // 작업 이력 (그린 도면 저장 · 다시 그리기 · 수정 · 이름변경 · 삭제)
    QVariantList jobHistory() const;
    int historyCount() const { return m_history.size(); }
    Q_INVOKABLE void saveCurrentDrawing(const QString &name);
    Q_INVOKABLE bool loadJob(const QString &id);     // 편집 상태로 불러오기
    Q_INVOKABLE void renameJob(const QString &id, const QString &name);
    Q_INVOKABLE void deleteJob(const QString &id);

signals:
    void sessionChanged();
    void drawingChanged();
    void robotStatusChanged();
    void robotLogChanged();
    void busyChanged();
    void linkStatusChanged();
    void poseChanged();
    void jobChanged();
    void loginSucceeded();
    void loginFailed(const QString &reason);
    void loggedOut();
    void registerSucceeded();
    void registerFailed(const QString &reason);
    void calibChanged();
    void noticeChanged();
    void arucoChanged();
    void strokeWidthChanged();
    void penDisplayOffsetChanged();
    void videoFiltersChanged();
    void speedChanged();
    void simChanged();
    void lensChanged();
    void robotVisibleChanged();
    void channelChanged();
    void historyChanged();
    void homographyChanged();

private:
    void startWorker();
    void wireWorker(video_worker *w);
    // 채널 n 의 URL. 메인 = 작업용, 서브 = 2x2 미리보기용.
    // 중계가 있으면 relay/README.md 의 경로 규약(/chN, /chNs), 없으면 카메라 직결.
    // ⚠️ 직결일 때는 서브 프로파일이 없어서 서브도 메인과 같은 주소다 (아래 주석 참고).
    QString channelUrl(int ch, bool sub) const;
    QString mainUrl(int ch) const;
    QString subUrl(int ch) const;
    void startPreviews();   // 미리보기 4개 기동 (없을 때만 — 이미 있으면 재개한다)
    void stopPreviews();    // 미리보기 **완전 종료** (로그아웃 · 주소 변경 · 종료)
    // 작업 화면을 드나들 때는 stop 이 아니라 이걸 쓴다. 세션을 살려두므로 복귀가
    // 즉시다 — 다시 열면 채널당 1.1초에 4채널이 직렬화돼 4.6초가 든다.
    // (근거와 실측은 preview_worker.h 의 setPaused 주석)
    void pausePreviews(bool on);
    // 로그인 직후 어디로 갈지 — 4채널이면 그리드, 아니면 예전처럼 바로 작업 화면
    void enterInitialView();
    // 채널별 캘리브레이션 맵(LOGIN_OK.calibs)에서 한 채널 번들을 꺼낸다
    QJsonObject calibOfChannel(int ch) const;
    // 카메라가 없어도 작도는 가능해야 한다 — 빈 바닥 프레임을 대신 밀어준다
    void pushPlaceholderFrame();
    void appendLog(const QString &line);
    void configureView(VideoView *v, bool topView);
    void refreshLinkStatus();
    void updatePhase();
    void setJobProgress(double p);
    void loadSettings();
    void saveSettings() const;
    void storeMission(const QList<QList<QPointF>> &paths, const QList<bool> &closed);
    // 도형들을 로봇 주행 순서로 재배열해 한 줄 경로로 만든다 (routeplan.h)
    routeplan::Route buildRoute(const QList<QList<QPointF>> &paths,
                                const QList<bool> &closed) const;
    void logRoute(const routeplan::Route &route,
                  const QList<QList<QPointF>> &paths, const QList<bool> &closed) const;
    // 계획된 폴리라인을 다시 도형 단위로 쪼갠다 (미션 표시·이력용)
    static QList<QList<QPointF>> routeShapes(const routeplan::Route &route,
                                             QList<bool> *closedOut);
    // 전송했으면 전송본, 아니면 지금 도면으로 즉석 생성한 동작 시퀀스
    QList<motionprogram::Op> currentProgram() const;
    void beginPainting();
    void finishJob(const QString &reason);
    void completeAbort(bool wasActive, bool requestedHere);
    void onPose(double x, double y, double thetaDeg);
    // 수신 POSE(=ArUco 마커)를 탑뷰 두 겹(섀시·노즐)으로 나눠 넘긴다. 정의부 주석 참고.
    void pushPoseToView(bool valid);
    // 테스트 재생용 — 위와 거울 관계(재생기 좌표가 노즐이라 섀시를 앞으로 민다).
    void pushSimPoseToView(const QPointF &nozzleM, double headingDeg, bool down);
    void onStatus(const QString &state, bool painting);
    double progressAlongPath(const QPointF &pose) const;
    // 미션 전체를 주행 순서대로 이어붙인 폴리라인 (닫힌 도형은 시작점 복귀 포함)
    QList<QPointF> missionTravelLine() const;
    void pushMissionToView();
    void pushOverlayToOriginal();
    void onAruco(const QList<int> &ids, const QList<QPolygonF> &corners);
    void startTestProgressSim();
    void stopTestProgressSim();
    void refreshJobMetrics();
    void refreshCalibStatus();
    // H 단위 검산 결과를 로그에 남긴다 (즉시 적용·지연 적용 두 경로 공용)
    void logCalibUnit(VideoView *v);
    // 캘리브레이션 번들은 세 곳에서 들어온다 — 설정창 수동 붙여넣기, LOGIN_OK, H_MATRIX.
    // 셋 다 **같은 경로**를 타야 서버 연동을 켜는 순간 바로 동작한다.
    //   normalizeCalibObject : 표기 흔들림(H_floor, {"calib":{}}, c0..c3) 흡수
    //   applyCalibObject     : K/D 반영 + coord_mode 로 렌즈보정 자동 판정 + TopView 재구성
    static QJsonObject normalizeCalibObject(const QJsonObject &raw);
    //   calibHasUsableH      : 3x3 · 유한 실수 · 특이하지 않은 H(H_floor) 인지 검사.
    //     저장·적용 **전에** 반드시 통과해야 한다. 요청하지 않았거나 늦게 온 번들도
    //     같은 검사를 받는다 — 그래야 화면 좌표계가 조용히 갈아엎히지 않는다.
    static bool calibHasUsableH(const QJsonObject &raw);
    // okOut: 실제로 TopView 에 적용됐는지. 실패(폴백)면 false — 호출부가 이 값을
    //        보고 "캘리브레이션 없음" 경고를 유지/해제해야 한다.
    QString applyCalibObject(const QJsonObject &raw, const QString &source,
                             bool *okOut = nullptr);
    void setNotice(const QString &text, const QString &level,
                   const QString &key = QString());
    void clearNotice(const QString &key);
    bool matchesHomographyReply(int ch, const QString &requestId) const;
    void failHomography(const QString &message, const QString &reason);
    void resetHomography();

    // 작업 이력 파일 (사용자별 AppData/jobs.json)
    QString historyPath() const;
    void loadHistory();
    void saveHistory();
    QString recordJob(const QList<QList<QPointF>> &paths, const QList<bool> &closed,
                      const QString &name, const QString &status,
                      const QList<QList<QPointF>> &editPaths = {},
                      const QList<bool> &editClosed = {},
                      const QList<bool> &editOuter = {});
    void updateJobRecord(const QString &id, const QString &status, double progress);

    video_worker *m_worker = nullptr;
    ServerClient *m_client = nullptr;
    VideoView *m_topView = nullptr;
    VideoView *m_originalView = nullptr;
    QTimer *m_linkTimer = nullptr;
    QTimer *m_testProgressTimer = nullptr;
    QTimer *m_jobTick = nullptr;
    QTimer *m_frameWatch = nullptr;
    QTimer *m_homographyTimer = nullptr;
    QTimer *m_cancelWatchdog = nullptr;   // 중단 확인 감시 (성공 처리 금지)
    bool m_gotRealFrame = false;
    // 스트림 세대(generation). 채널/주소를 바꿀 때마다 1 증가한다. 워커에서 오는
    // 프레임·마커·통계는 **자기가 만들어진 세대와 현재 세대가 같을 때만** 반영한다.
    // 큐드 시그널은 disconnect 시점과 무관하게 이미 보내진 것이 배달될 수 있어,
    // 이 검사 없이는 CH1 프레임이 CH3 화면에 그려진다.
    quint64 m_streamGen = 0;

    QString m_userId;
    QString m_camIp;
    QJsonObject m_calib;
    bool m_testMode = false;
    bool m_workerStarted = false;
    bool m_drawing = false;
    bool m_busy = false;
    bool m_estopActive = false;
    QString m_robotStatus = "대기";
    QString m_robotState = "IDLE";
    bool m_painting = false;
    bool m_nozzleDown = false;
    QString m_robotLog;
    // 현재 작업 채널의 실제 RTSP URL. 채널 선택 시 CH1~CH4 중 하나로 바뀐다.
    QString m_rtspUrl = "rtsp://admin:5hanwha!@192.168.0.13:554/0/profile2/media.smp";
    int m_brightness = 45, m_contrast = 8, m_sharpen = 0, m_saturation = 0;

    // ── 다채널 (PNM-C16083RVQ) ────────────────────────────────────────────
    // 비어 있으면 직결 템플릿+cam_ip로 CH1~CH4를 계속한다.
    //    QSettings: camera/relayBase, camera/channelCount
    QString m_relayBase;
    // 🔴 중계 없이 **카메라에 직결**할 때 쓰는 채널 URL 템플릿.
    //    치환: {ip} = LOGIN_OK.cam_ip, {ch0} = 0부터 센 채널번호,
    //    {ch} = 1부터 센 채널번호.
    //    한화 멀티센서 경로는 **센서 번호가 0부터**다 — 웹 UI 의 CH1 이 `0` 이다.
    //    (2026-08-04 원시 DESCRIBE 로 0~3 전부 200 OK 확인)
    //    중계(m_relayBase)가 설정돼 있으면 **중계가 이긴다**. 즉 나중에 서버가
    //    LOGIN_OK.stream 으로 중계 주소를 내려주면 이 템플릿은 자동으로 안 쓰인다.
    // ⚠️ 이 카메라에는 **저해상도 서브 프로파일이 없다.** 그래서 subUrl 은 직결
    //    모드에서 메인과 같은 주소를 준다 — 미리보기 4장이 전부 같은 해상도를
    //    디코딩한다. 서브가 생기면 여기만 고치면 된다.
    //    현장 운용(2026-08-11): profile2는 4채널 모두 2592x1520 20fps.
    QString m_channelUrlTemplate =
        QStringLiteral("rtsp://admin:5hanwha!@{ip}:554/{ch0}/profile2/media.smp");
    int m_channelCount = 4;
    int m_highlightedCh = 0;   // 클릭한 채널 (0 = 없음)
    int m_workingCh = 0;       // 작업 중인 채널 (0 = 그리드 화면)
    quint64 m_channelRequestSerial = 0;
    bool m_channelAckPending = false;
    // LOGIN_OK.calibs / CHANNEL_OK 로 받은 채널별 번들. 키는 채널 번호 문자열.
    // "어느 채널이 캘리 됐는지"를 그리드에 표시하고, 채널 전환 시 그 번들을 적용한다.
    QJsonObject m_calibs;
    QList<preview_worker *> m_previews;
    QHash<int, ChannelTile *> m_tiles;
    bool m_homographyPending = false;
    bool m_homographyCancelPending = false;
    int m_homographyCh = 0;
    double m_homographyProgress = -1.0;
    QString m_homographyStatus;
    QString m_homographyRequestId;
    // 오도메트리 주행 캘리브레이션 상태 (요청서 §2-2 진행률 · §5-1 입력값)
    bool    m_homographyOdometry = false;
    // 🔴 "중단을 요청한 적이 있다"와 "지금 확인을 기다린다"를 분리한다.
    //    watchdog 이 만료되면 pending 은 내려가지만 requested 는 세션이 끝날 때까지
    //    남는다 — 늦게 온 H_MATRIX 가 취소를 성공 완료로 뒤집지 못하게 하는 근거다.
    bool    m_homographyCancelRequested = false;
    QString m_homographyPhase;            // "driving" | "solving"
    int     m_homographyPointIndex = -1;  // 0~8, 방금 캡처를 마친 정지점
    int     m_homographyPointTotal = -1;  // 9 고정
    int     m_homographyValidPoints = -1; // 유효 대응점 수 (6 미만이면 실패)
    double  m_odoMCm = 90.0;              // 사각형 가로(cm) 기본 90
    double  m_odoNCm = 60.0;              // 사각형 세로(cm) 기본 60
    bool    m_odoCcw = true;              // true=반시계(bottom_left)
    quint64 m_homographyRequestSerial = 0;

    bool m_serverConnected = false;
    QString m_serverLabel = "대기";
    bool m_robotOnline = false;
    bool m_robotVisible = true;
    bool m_cctvOnline = false;
    double m_cctvFps = 0.0;
    double m_cctvLatencyMs = 0.0;
    // 로봇 **접속** 신호 — PEERS(robot>0) 와 STATUS 만 갱신한다. POSE 는 아니다.
    QElapsedTimer m_lastRobotBeat;
    bool m_hasRobotBeat = false;
    // 로봇 **위치 수신** 신호 — POSE 전용. 접속과 별개다(마커만 보여도 POSE 는 온다).
    QElapsedTimer m_lastPoseBeat;
    bool m_hasPoseBeat = false;
    bool m_poseEverSeen = false;    // 이번 세션에서 POSE 를 한 번이라도 받았는가
    bool m_poseWaitWarned = false;  // "POSE 미수신" 경고를 이미 남겼는가 (작업당 1회)

    double m_poseX = 0, m_poseY = 0, m_poseTheta = 0;
    bool m_poseValid = false;
    // PEERS 마지막 값 — 바뀔 때만 로그에 남기기 위한 비교용 (-1 = 아직 못 받음)
    int m_peerRobot = -1, m_peerCctv = -1;

    bool m_jobActive = false;      // START_DRAW ~ DRAW_DONE 사이 (접근+도색 전체)
    bool m_abortPending = false;   // ABORT_DRAW 전송 후 DRAW_ABORTED 대기
    bool m_blueprintSent = false;  // 도면이 서버에 저장돼 있는가
    bool m_paintingSeen = false;   // 이번 작업에서 STATUS.painting=true 를 본 적 있는가
    double m_jobProgress = 0.0;
    QString m_phase = "idle";
    QString m_phaseHint = "경로를 작성한 뒤 작업을 시작하세요.";
    QList<QList<QPointF>> m_missionPaths;   // 미터 좌표, 도형별
    QList<bool> m_missionClosed;            // 도형별 닫힘 여부
    QList<QList<QPointF>> m_missionEditPaths; // 편집용 완성 외곽(미터)
    QList<bool> m_missionEditClosed;
    QList<bool> m_missionEditOuter;
    // 마지막으로 서버에 보낸 BLUEPRINT 그대로 — 동작 시퀀스 미리보기가 실제로
    // 전송한 경로와 어긋나지 않게 계획 결과를 붙잡아 둔다.
    QList<QPointF> m_routePts;
    QList<bool> m_routePaint;
    // 서버로 보낸 동작 시퀀스 원본. 미리보기가 이것을 그대로 그리므로 화면과
    // 실제 주행이 어긋날 수 없다.
    QList<motionprogram::Op> m_program;
    double m_travelLengthM = 0.0;
    double m_pathLengthM = 0.0;
    int m_waypointIndex = 0;
    int m_waypointCount = 0;
    QString m_workName = "—";
    QElapsedTimer m_jobElapsed;
    bool m_jobElapsedValid = false;
    QString m_calibStatus = "기본 테스트 보정";
    bool m_calibMissing = false;
    QString m_calibId;        // 번들의 calib_id
    QString m_coordMode;      // 번들의 coord_mode ("undistort" / "raw" / 빈 값)
    QString m_calibSource;    // 어디서 들어왔는지 ("수동 입력" / "LOGIN_OK" / "H_MATRIX")
    double m_mmPerPx = 0.0;
    QString m_notice;
    QString m_noticeLevel = "info";
    QString m_noticeKey;          // 상황이 풀리면 그 통지만 골라 내리기 위한 태그
    QString m_arucoSummary;
    bool m_arucoOverlay = true;
    double m_strokeWidthMm = 50.0;   // 로봇이 그리는 선 두께 (5 cm)
    double m_penDisplayOffsetMm = 155.0; // TopView 표시 전용, 서버 전송에는 미사용
    motionprogram::Speeds m_speeds;
    // CCTV 가 준 정식 내부 파라미터 (K + plumb-bob 5계수). 왜곡 보정의 **유일한** 출처다.
    // ⚠️ 여기 있던 m_lensK1/m_lensK2(옛 2계수 경로)는 지웠다 — 같은 일을 하는 길이
    //    둘이라 어느 쪽이 켜졌는지에 따라 좌표가 달라졌다.
    camcalib::Model m_cam;
    bool m_lensOn = false;
    // 마지막으로 받은 프레임 크기. 캘리브 해상도와 맞는지 확인하는 데만 쓴다.
    int m_frameW = 0, m_frameH = 0;

    // 테스트 모드 시뮬레이션 — 서버로 보낸 동작 시퀀스를 그대로 재생한다.
    // 로봇 없이 후진·제자리회전·노즐까지 화면에서 확인하는 용도.
    motionprogram::Player m_sim;
    double m_simSpeedFactor = 4.0;   // 재생 배속 (실제 속도로 보면 몇 분씩 걸린다)
    bool m_simRunning = false;
    QString m_simPhase;

    QJsonArray m_history;            // 오래된 것 → 최신 순
    QString m_currentJobId;          // 지금 진행 중인 작업의 이력 id
};

#endif // BACKEND_H
