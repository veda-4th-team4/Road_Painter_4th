// 역할별 메시지 분배 (Router의 진입점). 각 role이 보낸 type을 보고 실제 처리를
// 담당 파일로 넘긴다 - 여기 남은 것은 "누가 무엇을 보냈나"의 분기와, 그 자리에서
// 끝나는 단순 중계뿐이다.
//   router_align.cpp - READY 정렬 판정 (ALIGN/GO)
//   router_calib.cpp - LOGIN / H_MATRIX / SELECT_CHANNEL
//   router_path.cpp  - START_DRAW / ABORT_DRAW / 경로 생성·복귀
// 상태(멤버 변수)는 전부 router.hpp에 있고 네 파일이 공유한다. onMessage의
// 뮤텍스 하나로 직렬화되므로 어느 파일에서 만지든 락을 더 잡을 필요는 없다.
#include "router.hpp"
#include "log.hpp"
#include <algorithm>  // std::min (이상치 게이트 허용치 상한)
#include <chrono>
#include <cmath>

// 단조 증가 시각 (쿨다운/타임아웃 계산용). 벽시계가 아니라 steady_clock이라
// 시스템 시간이 바뀌어도(NTP 보정 등) 간격 계산이 뒤틀리지 않는다.
long Router::nowMs() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch())
        .count();
}

void Router::onMessage(const std::string& role, const json& msg) {
    // 세션 스레드마다 진입하므로 라우터 상태 접근을 직렬화 (router.hpp mtx_ 참고)
    std::lock_guard<std::mutex> lk(mtx_);
    if (role == "QT") fromQt(msg);
    else if (role == "ROBOT") fromRobot(msg);
    else if (role == "CCTV") fromCctv(msg);
    else if (role == "ADMIN") fromAdmin(msg);
    // 이번 메시지로 조건이 충족됐을 수 있으니(POS로 새 pose가 들어왔거나, 그냥
    // 시간이 지났거나) 유예해 둔 READY를 여기서 한 곳에서 회수한다.
    resolvePendingReady();
    logPosStats();  // POS가 끊긴 것도 보이도록 POS 핸들러 밖에서 (함수 주석 참고)
}

// POS 수신/채택/폐기 요약. POS 핸들러가 아니라 onMessage() 말단에서 부르는
// 이유는 resolvePendingReady와 같다 - POS가 완전히 끊기면 POS 핸들러는 다시 안
// 도는데, 그 "0장"이야말로 제일 보고 싶은 값이다. 로봇 STATUS(2Hz)가 heartbeat로
// 이 요약을 밀어준다.
void Router::logPosStats() {
    if (posStatMs_ == 0) return;  // POS를 한 번도 못 받았으면 조용히 (CCTV 미접속 등)
    long dt = nowMs() - posStatMs_;
    if (dt < kPosStatPeriodMs) return;
    logf("[INFO] POS %.0f초 요약 - 수신 %d, 채택 %d (%.1fHz), 파싱실패 %d, "
         "속도게이트 %d, 코너뒤집힘 %d",
         dt / 1000.0, posRecv_, posAccept_, posAccept_ * 1000.0 / dt,
         posParseFail_, posGateRate_, posGateFlip_);
    posRecv_ = posAccept_ = posParseFail_ = posGateRate_ = posGateFlip_ = 0;
    posStatMs_ = nowMs();
}

// TlsServer가 세션 등록/정리 시 통지 (재접속 교체는 false 없이 넘어감 - tls_server.cpp 참고)
void Router::onPeerChange(const std::string& role, bool connected) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (role == "QT") {
        if (connected) sendPeers();  // 막 접속한 QT에 현재 스냅샷 1회 전송
        return;
    }
    if (role != "ROBOT" && role != "CCTV") return;  // QT 관심사만 (ADMIN 등 무시)
    sendPeers();
    logf("[INFO] PEERS 통지 - %s %s", role.c_str(), connected ? "접속" : "해제");
}

// 현재 ROBOT/CCTV 접속 여부를 조회해 QT로 전송. QT 미접속이면 다른 중계와
// 동일하게 sendTo가 WARN 로그만 남기고 조용히 실패한다 (특별 취급 안 함).
void Router::sendPeers() {
    bool robotOn = false, cctvOn = false;
    for (auto& r : srv_.connectedRoles()) {
        if (r == "ROBOT") robotOn = true;
        else if (r == "CCTV") cctvOn = true;
    }
    srv_.sendTo("QT", makeMsg("PEERS", {{"robot", robotOn}, {"cctv", cctvOn}}));
}

// 관리자 창(admin_console)에서 온 명령. 점검/설치용이라 경로 실행 중이어도
// 차단 없이 그대로 로봇에 전달한다 (QT 수동조작의 A안 차단과 다름).
void Router::fromAdmin(const json& msg) {
    std::string type = msg.value("type", "");
    json payload = msg.value("payload", json::object());
    if (type == "CMD") {
        std::string cmd = payload.value("cmd", "");
        // SELECT_CHANNEL은 카메라 전용이다. 관리자 창이 채널을 골라 캘리브레이션할 때
        // 쓰므로, 로봇에 보내면 로봇이 모르는 명령을 받는 셈이라 CCTV로만 보낸다.
        if (cmd == "SELECT_CHANNEL") {
            logf("[INFO] (ADMIN) CMD SELECT_CHANNEL -> CCTV");
            selectChannel(payload, msg);
            return;
        }
        // ABORT_DRAW는 서버 경로 상태까지 정리해야 하므로 QT와 같은 처리를 탄다
        // (그냥 중계만 하면 로봇은 경로를 버리는데 서버는 계속 실행 중으로 안다).
        if (cmd == "ABORT_DRAW") {
            bool wasActive = abortDraw();
            logf("[INFO] (ADMIN) CMD ABORT_DRAW -> ROBOT (진행 중이던 작업 %s)",
                 wasActive ? "폐기" : "없음");
            srv_.sendTo("ROBOT", msg);
            srv_.sendTo("QT", makeMsg("DRAW_ABORTED", {{"was_active", wasActive}}));
            return;
        }
        bool toCctv = (cmd == "CALIB_START");
        // 요약 INFO를 sendTo보다 먼저 (미접속 [WARN]이 뒤따르도록 - fromQt 참고)
        logf("[INFO] (ADMIN) CMD %s -> ROBOT%s", cmd.c_str(),
             toCctv ? " + CCTV" : "");
        srv_.sendTo("ROBOT", msg);
        if (toCctv) srv_.sendTo("CCTV", msg);
    } else if (type == "PATH") {
        logf("[INFO] (ADMIN) PATH -> ROBOT (%zu 세그먼트)",
             payload.value("segments", json::array()).size());
        srv_.sendTo("ROBOT", msg);  // 관리자 테스트 경로
    } else if (type == "H_MATRIX") {
        // 관리자 창(카메라 캘리 도구)이 계산 결과를 서버로 올림 - CCTV와 동일 처리
        handleHMatrix(msg);
    } else if (type == "LOGIN") {
        // 캘리 결과는 currentUser_에게 저장되므로, QT가 없는 설치 현장에서
        // 관리자 창이 직접 로그인해 저장 대상 계정을 정할 수 있게 한다.
        handleLogin(payload, "ADMIN");
    } else {
        logf("[WARN] ADMIN으로부터 알 수 없는 type: %s", type.c_str());
    }
}

void Router::fromQt(const json& msg) {
    std::string type = msg.value("type", "");
    json payload = msg.value("payload", json::object());

    if (type == "REGISTER") {
        std::string id = payload.value("id", ""), err;
        bool ok = users_.registerUser(id, payload.value("pw", ""),
                                      payload.value("cam_ip", ""), err);
        srv_.sendTo("QT", makeMsg(ok ? "REGISTER_OK" : "REGISTER_FAIL",
                                  ok ? json{{"id", id}} : json{{"reason", err}}));
        logf("[INFO] REGISTER %s: %s", id.c_str(), ok ? "성공" : err.c_str());
    } else if (type == "LOGIN") {
        handleLogin(payload, "QT");
    } else if (type == "SET_CAM_IP") {
        // Qt 설정란에서 카메라 IP 교체. REGISTER 때와 동일하게 형식 검증은 하지
        // 않고 저장만 한다 (Qt가 이 값으로 RTSP URL을 조립).
        std::string ip = payload.value("cam_ip", "");
        if (currentUser_.empty()) {
            srv_.sendTo("QT", makeMsg("SET_CAM_IP_FAIL", {{"reason", "로그인 필요"}}));
            logf("[WARN] SET_CAM_IP 수신 - 로그인 사용자 없음");
        } else if (!users_.setCamIp(currentUser_, ip)) {
            srv_.sendTo("QT", makeMsg("SET_CAM_IP_FAIL", {{"reason", "저장 실패"}}));
            logf("[WARN] SET_CAM_IP 저장 실패 - 사용자 '%s'", currentUser_.c_str());
        } else {
            srv_.sendTo("QT", makeMsg("SET_CAM_IP_OK",
                                      {{"cam_ip", users_.getCamIp(currentUser_)}}));
            logf("[INFO] SET_CAM_IP - 사용자 '%s' 카메라 IP 변경: '%s'",
                 currentUser_.c_str(), ip.c_str());
        }
    } else if (type == "CMD") {
        std::string cmd = payload.value("cmd", "");
        // START_DRAW: Qt "그림그리기 시작" 버튼. 로봇에 중계하지 않고 서버가
        // 1단계(접근) 경로를 만들어 PATH로 전송한다. 이후 접근 완료(PATH_DONE) ->
        // 2단계(도색) 전송 -> 도색 완료(PATH_DONE) -> DRAW_DONE까지 전부 자동이라
        // Qt가 중간에 더 눌러야 하는 버튼은 없다.
        if (cmd == "START_DRAW") {
            startApproach();
            return;
        }
        // ABORT_DRAW: "작업 중단" 버튼. ESTOP과 달리 받아둔 경로를 버린다.
        // 서버 상태를 먼저 정리하고 로봇에도 중계해 로봇의 PATH까지 폐기시킨다.
        // (ESTOP만 보내던 예전에는 RESUME 한 번에 도색이 그대로 재개됐다)
        if (cmd == "ABORT_DRAW") {
            bool wasActive = abortDraw();
            logf("[INFO] CMD ABORT_DRAW -> ROBOT (진행 중이던 작업 %s)",
                 wasActive ? "폐기" : "없음");
            srv_.sendTo("ROBOT", msg);
            srv_.sendTo("QT", makeMsg("DRAW_ABORTED", {{"was_active", wasActive}}));
            return;
        }
        // SELECT_CHANNEL: 4채널 카메라에서 작업 채널 전환. 로봇과는 무관하므로
        // ROBOT이 아니라 CCTV로 간다.
        if (cmd == "SELECT_CHANNEL") {
            selectChannel(payload, msg);
            return;
        }
        bool manualCmd = (cmd == "FORWARD" || cmd == "BACKWARD" ||
                          cmd == "TURN_LEFT" || cmd == "TURN_RIGHT" || cmd == "STOP");
        // [A안] 경로 실행(도색) 중에는 수동 조작을 차단한다 - 자동이 우선.
        // 도색 도중 조이스틱으로 로봇을 흔들어 그림을 망치는 것을 막기 위함.
        // 단 ESTOP/RESUME/CALIB_START 같은 비수동 명령은 아래로 흘려보내 항상 통과시킨다
        // (비상정지가 막히면 위험).
        if (manualCmd && planActive_) {
            logf("[WARN] CMD %s 무시 - 경로 실행 중이라 수동 조작 차단", cmd.c_str());
            return;
        }
        bool toCctv = (cmd == "CALIB_START");
        // 요약 INFO를 sendTo보다 먼저 찍는다 - sendTo가 미접속 시 [WARN]을 그
        // 자리에서 남기므로, 요약이 뒤에 오면 "경고 먼저, 정체는 나중"이라
        // 로그를 읽기 헷갈린다 (어느 명령의 경고인지 위로 스크롤해야 앎).
        logf("[INFO] CMD %s -> ROBOT%s%s", cmd.c_str(), toCctv ? " + CCTV" : "",
             manualCmd ? " [수동모드]" : "");
        srv_.sendTo("ROBOT", msg);
        if (toCctv) srv_.sendTo("CCTV", msg);
        // 여기 오는 수동 CMD는 경로가 없는(planActive_==false) 상태뿐이다.
        // 수동 조작에 진입하면 자동 경로추종/재계획을 멈춰 충돌을 막는다.
        // (자동 복귀는 새 BLUEPRINT 수신 시)
        if (manualCmd) manualMode_ = true;
    } else if (type == "BLUEPRINT") {
        // points = Qt가 top-view 픽셀 -> 바닥 미터 변환을 마친 좌표.
        // (top-view 위에 그린 점은 정의상 바닥 평면 위라 높이 보정 불필요)
        planPts_.clear();
        planPaint_.clear();
        planProgram_ = json::array();
        planCursor_ = 0;
        planActive_ = false;
        awaitingArrival_ = false;
        drawRequested_ = false;  // 이전 도면에 걸려 있던 시작 요청은 무효
        manualMode_ = false;  // 새 도면 = 자동 모드 복귀
        for (auto& p : payload.value("points", json::array())) {
            // 점 하나라도 형식이 어긋나면 도면 전체를 버린다 - 파싱이 중간에
            // 끊겨 반쪽짜리 planPts_로 경로를 만드는 것을 방지
            if (!p.is_array() || p.size() < 2 || !p[0].is_number() ||
                !p[1].is_number()) {
                planPts_.clear();
                logf("[WARN] BLUEPRINT points 형식 오류 - 도면 무시");
                sendDrawFail("plan", "bad_points", "도면 좌표 형식 오류");
                return;
            }
            planPts_.push_back({p[0].get<double>(), p[1].get<double>()});
        }
        if (planPts_.size() < 2) {
            logf("[WARN] 도면에 points가 부족함 (%zu개) - 경로 생성 불가",
                 planPts_.size());
            sendDrawFail("plan", "bad_points", "도면 점이 2개 미만");
            return;
        }
        // ----- 선택 필드 (없으면 종전과 100% 동일하게 동작) -----
        // paint[i] = points[i-1] -> points[i] 구간 도색 여부. paint[0]은 대응하는
        // 구간이 없어 무시된다. 길이가 어긋나면 도면을 버리지는 않고 이 필드만
        // 무시한다 (전 구간 도색으로 되돌아감) - 도형은 살리는 쪽이 안전.
        json paintJson = payload.value("paint", json::array());
        if (paintJson.is_array() && !paintJson.empty()) {
            if (paintJson.size() != planPts_.size()) {
                logf("[WARN] BLUEPRINT paint 길이 불일치 (%zu, points %zu) - "
                     "무시하고 전 구간 도색", paintJson.size(), planPts_.size());
            } else {
                for (auto& b : paintJson)
                    planPaint_.push_back(b.is_boolean() && !b.get<bool>() ? 0 : 1);
            }
        }
        // program: Qt가 입력한 MOVE/TURN/NOZZLE 값. 있으면 서버는 도색 경로를
        // 생성하지 않고 그대로 중계한다 (dist_m/angle_deg는 Qt 값 그대로 -
        // points에서 재계산하지 않는다. 사용자가 미세조정했을 수 있어서).
        json programJson = payload.value("program", json::array());
        if (programJson.is_array() && !programJson.empty()) {
            // 검증을 빡빡하게 하는 이유 두 가지:
            //  * 모르는 op을 그대로 중계하면 로봇이 그 세그먼트에서 영구 정지한다
            //    (로봇 실행부에 MOVE/TURN 분기밖에 없고 else가 없음). 서버가
            //    입구에서 막는 게 현장에서 원인 찾는 것보다 훨씬 싸다.
            //  * v(출발 꼭짓점 index)가 없으면 buildRecovery가 재개 지점을 못 찾아
            //    이탈 복귀가 조용히 죽는다. 있는 척하고 넘기느니 폴백이 낫다.
            const char* reason = nullptr;
            for (auto& op : programJson) {
                if (!op.is_object() || !op.contains("op")) {
                    reason = "op 필드 없음"; break;
                }
                std::string o = op.value("op", "");
                if (o != "MOVE" && o != "TURN" && o != "NOZZLE" && o != "ARC") {
                    reason = "지원하지 않는 op (MOVE/TURN/NOZZLE/ARC만 가능)"; break;
                }
                if (!op.contains("v") || !op["v"].is_number_integer()) {
                    reason = "v(출발 꼭짓점 index) 없음"; break;
                }
            }
            if (!reason) planProgram_ = programJson;
            else logf("[WARN] BLUEPRINT program 거부(%s) - 서버가 직접 생성", reason);
        }
        // 도면은 저장만 한다. 경로 생성/전송은 Qt가 "그림그리기 시작"(START_DRAW)을
        // 누른 뒤에 시작한다 - 도면을 올려놓고 로봇이 제멋대로 출발하지 않게.
        logf("[INFO] 도면 수신 (%zu점, paint %s, program %zu동작) - START_DRAW 대기",
             planPts_.size(), planPaint_.empty() ? "없음" : "있음",
             planProgram_.size());
        // Qt가 보낸 것과 서버가 받은 것을 즉시 대조할 수 있게 요약을 회신한다.
        srv_.sendTo("QT", makeMsg("BLUEPRINT_OK",
            {{"points", planPts_.size()},
             {"paint", !planPaint_.empty()},
             {"program", planProgram_.size()}}));
    } else {
        logf("[WARN] QT로부터 알 수 없는 type: %s", type.c_str());
    }
}

void Router::fromRobot(const json& msg) {
    std::string type = msg.value("type", "");
    json payload = msg.value("payload", json::object());

    if (type == "STATUS") {
        srv_.sendTo("QT", msg);  // Qt로 상태 중계 (지속 모니터링)
        logf("[INFO] STATUS: state=%s painting=%s",
             payload.value("state", "?").c_str(),
             payload.value("painting", false) ? "true" : "false");
    } else if (type == "READY") {
        // 로봇이 TURN을 마치고 다음 동작(직진 or 대기) 직전에 정렬 확인을 요청.
        // ALIGN을 보낸 뒤 온 READY(= 미세회전 완료 통지)라면 그 회전이 pose에
        // 반영될 때까지 판정을 미룬다 (router.hpp kAlignFreshFrames 참고).
        int seg = payload.value("seg", -1);
        if (seg != alignSegIdx_) {  // 새 세그먼트 정렬 시작
            alignSegIdx_ = seg;
            alignTries_ = 0;
        }
        if (alignTries_ > 0) {  // 미세회전 완료 통지 - 그 회전이 pose에 실릴 때까지 유예
            clearPendingReady();  // 이번 유예분만 모으도록 누적을 비우고 시작
            pendingReadySeg_ = seg;
            pendingPosSeq_ = posSeq_;
            pendingReadyMs_ = nowMs();
            logf("[INFO] READY(seg=%d) - 직전 ALIGN 반영 대기 (새 POS %d장 필요)",
                 seg, kAlignFreshFrames);
            return;
        }
        judgeReady(seg);
    } else if (type == "PATH_DONE") {
        // 받은 PATH를 끝까지 수행했다는 로봇의 통지. 어느 단계였는지는 서버 상태로
        // 판단하고, payload.phase는 어긋났을 때 경고를 남기는 용도로만 쓴다
        // (로봇이 phase를 안 실어도 동작하게 - 서버가 단계의 주인).
        std::string phase = payload.value("phase", "");
        if (awaitingArrival_) {
            if (!phase.empty() && phase != "approach")
                logf("[WARN] PATH_DONE phase=%s - 서버는 접근 대기 중이라 접근 완료로 처리",
                     phase.c_str());
            // 접근 완료는 Qt에 알리지 않는다 (Qt 입장에선 "그리는 중"이 계속됨).
            logf("[INFO] 로봇 접근 완료 - 2단계 도색 경로로 이어감");
            sendDrawPath();
        } else if (planActive_) {
            if (!phase.empty() && phase != "draw")
                logf("[WARN] PATH_DONE phase=%s - 서버는 도색 중이라 도색 완료로 처리",
                     phase.c_str());
            planActive_ = false;
            activeSegs_ = json::array();
            resetAlign();
            planCursor_ = 0;
            srv_.sendTo("QT", makeMsg("DRAW_DONE", json::object()));
            logf("[INFO] 도색 완료 - QT에 DRAW_DONE 통지");
        } else {
            logf("[WARN] PATH_DONE 수신 - 진행 중인 경로 없음 (무시)");
        }
    } else {
        logf("[WARN] ROBOT으로부터 알 수 없는 type: %s", type.c_str());
    }
}


void Router::fromCctv(const json& msg) {
    std::string type = msg.value("type", "");
    json payload = msg.value("payload", json::object());

    if (type == "POS") {
        // corners = CCTV 원본 픽셀 좌표. 좌표 변환(undistort -> H_marker)은
        // 여기(서버)서만 수행 - CCTV는 캘리브레이션 데이터를 가질 필요 없음.
        // 로봇에는 중계하지 않는다 - v0.3 설계에서 로봇은 좌표를 모르며,
        // 위치 보정은 서버가 각도로 변환해 ALIGN/DRIFT로만 내려준다.
        // 원본 POS는 QT에 중계하지 않는다: Qt가 쓰는 건 아래 POSE(미터 좌표)뿐이고,
        // 원본 픽셀은 Qt에서 해석할 방법이 없다 (캘리브레이션은 서버만 갖고 있음).
        const int ch = channelOf(payload);  // ch 없으면 1 (단일 채널 하위호환)
        if (ch != activeChannel_) {
            // 활성 채널이 아닌 곳에서 온 마커. 받아들이면 pose가 두 채널 사이에서
            // 튄다 - 채널마다 H가 달라 좌표계 자체가 다르기 때문.
            // POS는 15~30Hz라 매번 찍으면 로그가 뒤덮이므로 채널이 바뀔 때만 남긴다.
            // (조용히 버리면 "왜 로봇이 안 보이지"를 몇 시간씩 찾게 된다)
            if (ch != lastIgnoredPosCh_) {
                lastIgnoredPosCh_ = ch;
                logf("[WARN] POS 채널 %d 무시 - 활성 채널은 %d 다 "
                     "(CCTV가 다른 채널을 보고 있거나 ch를 잘못 싣는 중)",
                     ch, activeChannel_);
            }
            return;
        }
        lastIgnoredPosCh_ = 0;
        ++posRecv_;
        if (posStatMs_ == 0) posStatMs_ = nowMs();  // 첫 POS부터 요약 구간 시작

        Pose p;
        if (poseFromPos(payload, activeCalib(), p)) {
            // ----- 이상치 게이트: 물리적으로 불가능한 pose 변화는 검출 오류다 -----
            // 두 가지를 본다 (router.hpp의 kPoseGate*/kFlip* 주석 참고):
            //   속도: 각도 변화가 "노이즈 몫 + 물리 회전 몫"을 넘는가 (상한 있음)
            //   플립: 각도는 크게 변했는데 중심(코너 평균)은 제자리인가
            //         = 코너 순서가 한 칸 돌아간 것. 마커가 실제로 돌았다면
            //           중심도 최소한 흔들리므로 이 조합은 나올 수 없다.
            long dtMs = nowMs() - lastPoseMs_;
            double dTheta =
                std::fabs(normDeg((p.theta - pose_.theta) * 180.0 / M_PI));
            double dCenter = std::hypot(p.x - pose_.x, p.y - pose_.y);
            double allowed = std::min(
                kPoseGateBaseDeg + kPoseGateRateDps * (dtMs / 1000.0),
                kPoseGateMaxDeg);
            const char* reject = nullptr;
            if (poseValid_) {
                if (dTheta > kFlipMinDeg && dCenter < kFlipStillM) reject = "flip";
                else if (dTheta > allowed) reject = "rate";
            }
            // 플립도 연속 거부 한도를 같이 쓴다 - 사람이 로봇을 제자리에서 돌려
            // 놓으면 "중심 그대로 + 각도 급변"이 진짜로 성립하므로, 영원히
            // 거부만 하면 복구가 안 된다.
            if (reject && poseRejects_ < kPoseRejectMax) {
                ++poseRejects_;
                if (reject[0] == 'f') {
                    ++posGateFlip_;
                    logf("[WARN] POS 코너 순서 뒤집힘 의심 - theta %.1f도 급변인데 "
                         "중심은 %.3fm만 이동 (%ldms) 폐기 %d/%d",
                         dTheta, dCenter, dtMs, poseRejects_, kPoseRejectMax);
                } else {
                    ++posGateRate_;
                    logf("[WARN] POS 이상치 - theta %.1f도 급변 (%ldms 내 허용 %.1f도) "
                         "폐기 %d/%d", dTheta, dtMs, allowed, poseRejects_,
                         kPoseRejectMax);
                }
                return;  // pose_ 갱신도, QT 중계도, posSeq_ 증가도 하지 않는다
            }
            if (poseRejects_ >= kPoseRejectMax)
                logf("[WARN] POS 이상치 연속 %d회 - 추적을 놓친 것으로 보고 재동기",
                     poseRejects_);
            poseRejects_ = 0;
            ++posAccept_;
            pose_ = p;
            poseValid_ = true;
            lastPoseMs_ = nowMs();
            ++posSeq_;  // ALIGN 반영 여부 판정용 (router.hpp kAlignFreshFrames)
            // 유예 중인 READY가 있으면 판정용 theta 표본으로 모아둔다
            if (pendingReadySeg_ >= 0) {
                pendingSin_ += std::sin(p.theta);
                pendingCos_ += std::cos(p.theta);
                ++pendingPoseN_;
            }
            // 계산된 pose(바닥 미터 좌표)를 QT에 전송 - top-view 위 로봇 표시용
            srv_.sendTo("QT", makeMsg("POSE",
                {{"x", std::round(p.x * 1000) / 1000},
                 {"y", std::round(p.y * 1000) / 1000},
                 {"theta_deg", std::round(p.theta * 180.0 / M_PI * 10) / 10}}));
        } else {
            // 캘리브레이션이 없는 경우만 여기서 소리를 낸다. corners 형식 오류는
            // 매 프레임 같은 로그가 쏟아지므로 카운터로만 세고, 주기 요약
            // (logPosStats)의 "파싱실패" 항목으로 드러낸다.
            ++posParseFail_;
            if (!activeCalib().valid)
                logf("[WARN] POS 수신했으나 채널 %d 캘리브레이션 없음 - pose 계산 불가 "
                     "(그 채널을 캘리브레이션할 것)", ch);
            return;  // pose를 못 구하면 여기서 끝 (QT로 나가는 건 POSE뿐)
        }

        // 수동 조작 중엔 자동 경로/재계획을 하지 않는다 (POSE 모니터링은 위에서 이미 전송).
        if (manualMode_) return;
        if (!planActive_) {
            // START_DRAW를 눌렀는데 그때 pose가 없어 미뤄둔 경우에만 접근을 시작한다.
            // (도면만 올라와 있는 상태에서는 로봇을 움직이지 않는다)
            if (drawRequested_) startApproach();
            return;
        }

        // ----- 주행 중 각도 피드백(DRIFT): 실행 중인 세그먼트의 목표 방향 대비 -----
        // 얼마나 틀어졌는지 지속 전송. 부호: 가려는 방향이 0도 기준,
        // 시계방향(오른쪽)으로 틀어져 있으면 양수, 반시계(왼쪽)면 음수.
        // (값 자체가 "좌회전으로 보정해야 할 양"과 같음 - ALIGN과 동일 규약)
        if (alignSegIdx_ >= 0 && alignSegIdx_ < (int)activeSegs_.size() &&
            nowMs() - lastDriftMs_ >= kDriftPeriodMs) {
            const json& seg = activeSegs_[alignSegIdx_];
            double target = seg.value("heading_deg", 1e9);
            if (target < 1e8) {
                double drift = normDeg(target - pose_.theta * 180.0 / M_PI);
                srv_.sendTo("ROBOT", makeMsg("DRIFT",
                    {{"angle_deg", std::round(drift * 10) / 10}}));
                lastDriftMs_ = nowMs();
            }
        }

        // 1단계(접근) 중엔 이탈 재계획을 하지 않는다 - 로봇이 도면 폴리라인에서
        // 떨어져 있는 게 정상이라 오탐이 나기 때문. (READY/ALIGN + DRIFT로 충분)
        if (awaitingArrival_) return;

        // ----- 이탈 감시: "지금 달리는 구간"에서 임계값 이상 벗어나면 재계획 -----
        // 도면 전체와 비교하면 옆줄 위에 올라타도 "경로 위"로 판정돼 이탈을 못 잡고,
        // 재시작 지점도 옆줄로 튄다 (path_planner.hpp 진행 커서 주석 참고).
        advanceCursor({pose_.x, pose_.y}, planPts_, planCursor_, kVertexReachM);
        // kPenOffsetM을 넘겨 "마커 중심이 꼭짓점을 d만큼 지나쳐 있는" 정상 상태를
        // 오차 0으로 본다 (도면은 펜 자취인데 pose는 마커 중심이라 상시 d가 뜬다).
        double dev = distToActiveSegment({pose_.x, pose_.y}, planPts_,
                                         planCursor_, kPenOffsetM);
        if (dev > kDevThresholdM && nowMs() - lastPlanMs_ > kReplanCooldownMs) {
            // 성공하든 실패하든 쿨다운을 건다. 안 걸면 실패가 계속되는 동안
            // POS가 올 때마다(10~30Hz) 재시도하며 로그를 뒤덮는다.
            lastPlanMs_ = nowMs();
            // 원래 향하던 꼭짓점으로 복귀시킨다 (전체 최근접이 아니라)
            size_t k = std::min(planCursor_ + 1, planPts_.size() - 1);
            json segs = buildRecovery(k);
            if (segs.empty()) {
                logf("[WARN] 경로 이탈 %.2fm (구간 %zu) - 복귀 경로 생성 실패",
                     dev, planCursor_);
            } else if (srv_.sendTo("ROBOT", makePathMsg(segs, "draw"))) {
                activeSegs_ = segs;
                resetAlign();  // 새 경로 = 정렬 상태 리셋
                logf("[WARN] 경로 이탈 %.2fm (구간 %zu) - 복귀 PATH 전송 "
                     "(%zu번 꼭짓점으로, %zu 동작)",
                     dev, planCursor_, k, segs.size());
            } else {
                logf("[WARN] 경로 이탈 %.2fm - 복귀 PATH 전송 실패 (로봇 미접속)",
                     dev);
            }
        }
    } else if (type == "H_MATRIX") {
        handleHMatrix(msg);
    } else {
        logf("[WARN] CCTV로부터 알 수 없는 type: %s", type.c_str());
    }
}
