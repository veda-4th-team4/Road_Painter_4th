#include "router.hpp"
#include "log.hpp"
#include <chrono>

static long nowMs() {
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
        blueprint_ = payload;
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
        lastStatus_ = payload;
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
            alignSegIdx_ = -1, alignTries_ = 0; clearPendingReady();
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

void Router::clearPendingReady() {
    pendingReadySeg_ = -1;
    pendingSin_ = pendingCos_ = 0;
    pendingPoseN_ = 0;
}

// READY 정렬 판정. CCTV pose의 실제 각도 vs 해당 세그먼트의 목표 heading 비교:
//   오차 > 임계값 -> ALIGN(그만큼 미세 회전 후 로봇이 다시 READY)
//   오차 <= 임계값 or 반복 초과 -> GO(직진 시작 / 접근 마지막 TURN이면 대기)
// 여기 들어왔다는 건 "지금 pose로 판정해도 된다"가 이미 정해졌다는 뜻이다
// (READY 수신 즉시 or 유예 후 새 POS 확보). 어느 경로로 오든 반드시 로봇에
// 응답을 하나 보낸다 - 응답을 빠뜨리면 로봇이 영원히 정지한 채 대기한다.
void Router::judgeReady(int seg) {
    // 유예 중 모아둔 theta가 있으면 그 평균으로 판정한다. 기다리는 동안 공짜로
    // 쌓인 표본이라 지연 없이 노이즈만 1/sqrt(N)로 준다. 각도 평균은 반드시
    // 원 위에서(sin/cos) - 도를 그냥 더하면 ±180도 경계에서 깨진다.
    double thetaDeg = pose_.theta * 180.0 / M_PI;
    char avgDesc[32];
    if (pendingPoseN_ > 0) {
        thetaDeg = std::atan2(pendingSin_, pendingCos_) * 180.0 / M_PI;
        snprintf(avgDesc, sizeof avgDesc, "POS %d장 평균", pendingPoseN_);
    } else {
        snprintf(avgDesc, sizeof avgDesc, "단일 POS");
    }
    clearPendingReady();
    // heading_deg가 있는 세그먼트(MOVE 전부 + 접근 경로 마지막 TURN)만 판정 가능
    double target = 1e9;
    if (planActive_ && !manualMode_ && seg >= 0 && seg < (int)activeSegs_.size())
        target = activeSegs_[seg].value("heading_deg", 1e9);

    if (target > 1e8 || !poseValid_) {
        // 판정 불가(계획 없음/수동모드/seg 이상/pose 없음)면 로봇을 세워두지 않는다
        srv_.sendTo("ROBOT", makeMsg("GO", json::object()));
        logf("[WARN] READY(seg=%d) - 정렬 판정 불가, 그냥 GO", seg);
        return;
    }
    double err = normDeg(target - thetaDeg);
    if (std::fabs(err) > kAlignThresholdDeg && alignTries_ < kAlignMaxTries) {
        ++alignTries_;
        srv_.sendTo("ROBOT",
                    makeMsg("ALIGN", {{"angle_deg", std::round(err * 10) / 10}}));
        logf("[INFO] READY(seg=%d) 각도오차 %.1f도 (%s) - ALIGN 전송 (%d/%d회)",
             seg, err, avgDesc, alignTries_, kAlignMaxTries);
    } else {
        srv_.sendTo("ROBOT", makeMsg("GO", json::object()));
        logf("[INFO] READY(seg=%d) 오차 %.1f도 (%s) - GO%s", seg, err, avgDesc,
             alignTries_ >= kAlignMaxTries ? " (ALIGN 반복 초과)" : "");
    }
}

// 유예해 둔 READY가 있으면 판정 조건을 확인한다. READY 수신 이후 새 POS가
// kAlignFreshFrames만큼 쌓였거나, 그 전에 kAlignWaitMaxMs가 지나면(POS가 끊긴
// 상황) 있는 pose로 판정한다.
// POS 수신 때뿐 아니라 onMessage 진입 때마다 호출한다 - POS가 완전히 끊기면 POS
// 핸들러는 다시 안 돌아 타임아웃이 영영 안 걸리기 때문. 그 경우 로봇 STATUS가
// heartbeat가 되어 유예된 READY를 반드시 회수한다. 단 STATUS는 500ms 주기(2Hz,
// 로봇 main.cpp)라 타임아웃 판정이 최대 그만큼 늦다 - POS가 죽은 상황의 최악
// 응답 지연은 kAlignWaitMaxMs + 500ms다.
void Router::resolvePendingReady() {
    if (pendingReadySeg_ < 0) return;
    long got = posSeq_ - pendingPosSeq_;
    bool fresh = got >= kAlignFreshFrames;
    bool timeout = nowMs() - pendingReadyMs_ > kAlignWaitMaxMs;
    if (!fresh && !timeout) return;
    // 🔴 새 POS가 "한 장도" 없으면 판정하지 않고 GO를 보낸다 (2026-08-04).
    // 여기서 judgeReady를 부르면 pose_가 직전 ALIGN을 계산할 때와 글자 그대로
    // 같으므로 err도 같고, 결국 **똑같은 ALIGN이 한 번 더 나가는 것이 보장**된다.
    // 2026-08-04 로그에서 실제로 -34.6도가 값까지 동일하게 세 번 나갔고, 로봇은
    // 그만큼 세 번 돌아 약 104도를 회전했다.
    // GO를 택한 이유: 남은 각도오차는 주행 중 DRIFT가 이어서 잡아주지만, 중복
    // ALIGN이 만든 누적 오버슛은 되돌릴 방법이 없다. 판정 근거가 없을 때 로봇을
    // 세워두지 않는다는 기존 원칙(judgeReady의 "판정 불가 -> GO")과도 같다.
    // alignTries_는 소모하지 않는다 - 실제로 정렬을 시도한 것이 아니기 때문이다.
    if (got == 0) {
        int seg = pendingReadySeg_;
        long waited = nowMs() - pendingReadyMs_;
        clearPendingReady();
        srv_.sendTo("ROBOT", makeMsg("GO", json::object()));
        logf("[WARN] READY(seg=%d) 유예 %ldms 동안 새 POS 0장 - 직전과 같은 pose라 "
             "판정 불가, GO (같은 ALIGN 반복 방지)", seg, waited);
        return;
    }
    if (!fresh)
        logf("[WARN] READY(seg=%d) 유예 %ldms 초과 (새 POS %ld/%d장) - 현재 pose로 판정",
             pendingReadySeg_, nowMs() - pendingReadyMs_, got, kAlignFreshFrames);
    judgeReady(pendingReadySeg_);
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
        lastPos_ = payload;
        ++posRecv_;
        if (posStatMs_ == 0) posStatMs_ = nowMs();  // 첫 POS부터 요약 구간 시작

        Pose p;
        if (poseFromPos(payload, calib_, p)) {
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
            if (!calib_.valid)
                logf("[WARN] POS 수신했으나 캘리브레이션 없음 - pose 계산 불가");
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
                alignSegIdx_ = -1, alignTries_ = 0; clearPendingReady();  // 새 경로 = 정렬 상태 리셋
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

// 로그인 처리 (QT/ADMIN 공용). 성공하면 서버가 기억하는 로그인 사용자(currentUser_)를
// 갱신하고 그 계정에 저장돼 있던 캘리브레이션을 현재 세션에 복원한다.
// 캘리브레이션(H_MATRIX)은 "그 시점의 currentUser_"에게 영속 저장되므로, QT가 아직
// 붙지 않은 설치 현장에서도 관리자 창이 먼저 로그인해두면 캘리 결과가 계정에 남는다.
void Router::handleLogin(const json& payload, const std::string& replyRole) {
    std::string id = payload.value("id", "");
    if (!users_.login(id, payload.value("pw", ""))) {
        srv_.sendTo(replyRole,
                    makeMsg("LOGIN_FAIL", {{"reason", "id 또는 비밀번호 불일치"}}));
        logf("[WARN] LOGIN %s 실패 (%s 요청)", id.c_str(), replyRole.c_str());
        return;
    }
    currentUser_ = id;
    json stored = users_.getCalib(id);
    Calib c;
    if (!stored.is_null() && calibFromJson(stored, c))
        calib_ = c;  // 저장된 캘리브레이션을 현재 세션에 복원
    srv_.sendTo(replyRole, makeMsg("LOGIN_OK",
        {{"id", id}, {"calib", stored}, {"cam_ip", users_.getCamIp(id)}}));
    logf("[INFO] LOGIN %s 성공 (%s 요청, 캘리브레이션 %s)", id.c_str(),
         replyRole.c_str(), stored.is_null() ? "없음 - 캘리브레이션 필요" : "전달");
}

// 캘리브레이션 번들 수신 (CCTV 직접 or 관리자 창 ADMIN 경유 공용). 세 형태를 받는다:
//   중첩:   payload.calib = {K, D, H_floor, H_marker, marker_height_m, version}
//   평면:   payload 자체가 번들 = {calib_id, K, D, H, H_marker, canvas_mm, ...}
//           (QT-REQ-CCTV-001 rev.2 — 바닥 H를 H_floor가 아니라 H로 부른다)
//   레거시: payload.H = [[...]x3] 뿐 (왜곡 보정 없이 바닥/마커 공용으로 사용)
// CCTV는 mm 기준(pixel->world mm) 호모그래피를 보낸다. 서버 입구에서 미터로 정규화한 뒤
// 저장/중계하므로, 이후 pose/POSE/BLUEPRINT/PATH와 QT top-view는 전부 미터로 통일된다.
void Router::handleHMatrix(const json& msg) {
    json payload = msg.value("payload", json::object());
    // 평면 번들과 레거시는 둘 다 최상위 "H"를 갖는다. 예전엔 "calib이 없고 H가 있으면
    // 레거시"로 단정해 payload["H"] 행렬 하나만 떼어냈고, 그래서 평면 번들이 오면
    // K/D/H_marker가 통째로 버려져 왜곡 보정과 시차 보정이 조용히 꺼졌다 - 좌표는
    // 그럴듯하게 나오고 렌즈 왜곡만큼 틀린다. 캘리 내용 필드가 H와 같이 왔으면
    // 평면 번들로 본다.
    const bool nested = payload.contains("calib");
    const bool legacyH = !nested && payload.contains("H") &&
                         !payload.contains("K") && !payload.contains("D") &&
                         !payload.contains("H_floor") && !payload.contains("H_marker");
    json bundle = nested ? payload["calib"] : (legacyH ? payload["H"] : payload);
    normalizeBundleMmToM(bundle);  // mm -> m (÷1000). 이후 번들은 미터 기준.
    aliasFloorKey(bundle);  // 평면 스키마의 "H"에 "H_floor" 별칭 (QT는 H_floor만 봄)
    Calib c;
    if (!calibFromJson(bundle, c)) {
        logf("[WARN] H_MATRIX 파싱 실패 - calib/H 형식 확인 필요: %s",
             payload.dump().c_str());
        return;
    }
    calib_ = c;
    // 정규화된(미터) 번들로 다시 싸서 QT에 중계 + 영속 저장 - QT는 미터 H_floor로 top-view.
    json outMsg = msg;
    if (nested) outMsg["payload"]["calib"] = bundle;
    else if (legacyH) outMsg["payload"]["H"] = bundle;
    else outMsg["payload"] = bundle;  // 평면 번들은 payload 자체가 번들
    srv_.sendTo("QT", outMsg);
    // 어느 스키마로 읽혔는지 남긴다 - 평면 번들을 보냈는데 "레거시"로 찍히면
    // K/D가 빠졌다는 뜻이라, 로그만 보고 바로 알 수 있어야 한다.
    const char* schema = nested ? "중첩 calib" : (legacyH ? "레거시 H" : "평면 번들");
    // 로그인 상태와 무관하게 전역 슬롯에 먼저 남긴다 (QT-REQ-SRV-001 R-1).
    // 캘리브레이션은 현장 속성이라, 아무도 로그인하지 않은 채 올려도 서버 재시작 후
    // 살아남아야 한다 - 예전엔 이 경우 메모리에만 남아 그대로 유실됐다.
    users_.setGlobalCalib(bundle);
    const char* detail_marker =
        c.hasMarker ? "H_marker 포함" : "H_marker 없음 - 시차 보정 생략됨";
    const char* detail_kd = c.hasKD ? ", K/D 포함" : ", K/D 없음 - 왜곡 보정 생략됨";
    if (!currentUser_.empty() && users_.setCalib(currentUser_, bundle))
        logf("[INFO] 캘리브레이션 수신 (%s, mm->m 정규화, %s%s) - 사용자 '%s' + 전역 슬롯에 영속 저장",
             schema, detail_marker, detail_kd, currentUser_.c_str());
    else
        logf("[INFO] 캘리브레이션 수신 (%s, mm->m 정규화, %s%s) - 로그인 사용자 없음, "
             "전역 슬롯에 영속 저장 (다음 로그인 때 전달됨)",
             schema, detail_marker, detail_kd);
}

// 1단계: Qt "그림그리기 시작" -> 로봇 현재 위치에서 도면 시작점(planPts_[0])까지
// 접근 경로를 만들어 전송. 시작점 도착 후 첫 도색 방향으로 미리 회전까지 시켜두고,
// 로봇이 PATH_DONE으로 도착을 알리면 서버가 곧바로 2단계(도색)로 이어간다.
void Router::startApproach() {
    if (planPts_.size() < 2) {
        logf("[WARN] START_DRAW 수신 - 도면 없음 (무시)");
        sendDrawFail("draw", "no_blueprint", "도면 없음 - 먼저 도면을 전송할 것");
        drawRequested_ = false;
        return;
    }
    if (planActive_) {  // 이미 접근/도색 진행 중 - 중복 시작 방지
        logf("[WARN] START_DRAW 수신 - 이미 경로 실행 중 (무시)");
        sendDrawFail("draw", "busy", "이미 경로 실행 중 - 완료 또는 새 도면 전송 후 시작할 것");
        return;
    }
    if (!poseValid_) {
        // 실패가 아니라 대기다. CCTV로 pose가 잡히는 순간 POS 핸들러가 재호출한다.
        drawRequested_ = true;
        logf("[INFO] START_DRAW 수신 - 로봇 위치 미확인, CCTV POS 수신 후 전송 예정");
        sendDrawFail("draw", "no_pose", "로봇 위치 미확인 - CCTV POS 수신 후 자동 전송 예정");
        return;
    }
    planCursor_ = 0;
    // 시작점까지 (paint=false). 접근 목표는 planPts_[0] "그대로"다 - 로봇
    // 중심을 도면 시작점에 세우기만 하면 되고, 펜 오프셋 보정은 로봇이 스스로
    // 한다 (서버/Qt 둘 다 관여하지 않음).
    // 접근 구간에는 NOZZLE을 끼우지 않는다. 전부 paint=false라 withNozzleOps를
    // 통과시켜도 아무것도 안 붙고(노즐은 올라간 채로 시작·유지), 도색/복귀 경로가
    // 항상 NOZZLE up으로 끝나므로 여기서 다시 올릴 필요도 없다.
    json segs = buildSegments(pose_, {planPts_[0]});

    // 도착 직후 방향: 접근 MOVE의 heading, 이동이 없었다면 현재 각도
    double arrival = pose_.theta * 180.0 / M_PI;
    for (auto& s : segs)
        if (s.value("op", "") == "MOVE") arrival = s.value("heading_deg", arrival);
    // 첫 도색 구간(시작점 -> 두번째 점) 방향으로 미리 회전
    double first = std::atan2(planPts_[1][1] - planPts_[0][1],
                              planPts_[1][0] - planPts_[0][0]) * 180.0 / M_PI;
    double turn = normDeg(first - arrival);
    if (std::fabs(turn) > 2.0)
        segs.push_back({{"op", "TURN"},
                        {"angle_deg", std::round(turn * 10) / 10},
                        {"heading_deg", std::round(first * 10) / 10}});
    // (마지막 TURN에도 heading_deg를 실어 로봇이 READY로 정렬 확인 가능)

    if (segs.empty()) {
        // 로봇이 이미 시작점에 첫 도색 방향으로 서 있는 경계 케이스. 접근할 게
        // 없으므로 도착 통지를 기다리지 않고 곧바로 2단계로 넘어간다.
        logf("[INFO] 접근 불필요 (이미 시작점) - 곧바로 도색 경로 전송");
        drawRequested_ = false;
        awaitingArrival_ = true;  // sendDrawPath의 단계 가드 통과용
        sendDrawPath();
        return;
    }
    if (srv_.sendTo("ROBOT", makePathMsg(segs, "approach"))) {
        planActive_ = true;
        awaitingArrival_ = true;
        drawRequested_ = false;
        activeSegs_ = segs;
        alignSegIdx_ = -1, alignTries_ = 0; clearPendingReady();  // 새 경로 = 정렬 상태 리셋
        lastPlanMs_ = nowMs();
        logf("[INFO] 1단계 접근 경로 전송 (%zu 세그먼트) - 로봇 도착(PATH_DONE) 대기",
             segs.size());
    } else {
        logf("[WARN] 1단계 접근 경로 전송 실패 - 로봇 미접속");
        sendDrawFail("draw", "robot_offline", "로봇 미접속 - 경로 전송 실패");
        drawRequested_ = false;
    }
}

// 2단계: 로봇 접근 완료(PATH_DONE) -> 시작점에 서 있는 로봇에게 도색 경로 전송.
// 로봇은 이 PATH(phase=draw)를 받으면 IMU 현재 방향을 0도로 세팅하고 주행 시작.
void Router::sendDrawPath() {
    // 아래 실패 경로들은 전부 "접근은 끝났는데 도색을 못 시작한" 상황이다.
    // 상태를 접어두지 않으면 planActive_/awaitingArrival_가 켜진 채로 남아
    // Qt가 START_DRAW로 다시 시도할 수도, 완료 통지를 받을 수도 없게 된다.
    if (!awaitingArrival_) {  // 내부 오류 (정상 흐름에서는 도달하지 않음)
        logf("[WARN] 도색 경로 전송 요청 - 접근 완료 대기 상태가 아님 (무시)");
        sendDrawFail("draw", "not_ready", "로봇이 시작점 접근 완료 대기 상태가 아님");
        return;
    }
    if (!poseValid_ || planPts_.size() < 2) {
        logf("[WARN] 도색 경로 생성 불가 - pose/도면 없음");
        sendDrawFail("draw", !poseValid_ ? "no_pose" : "no_blueprint",
                     !poseValid_ ? "로봇 위치 미확인" : "도면 없음");
        awaitingArrival_ = false;
        planActive_ = false;
        return;
    }
    planCursor_ = 0;  // 도색 시작 = 첫 구간부터
    json segs;
    if (planProgram_.is_array() && !planProgram_.empty()) {
        // Qt가 입력한 동작 시퀀스를 손대지 않고 그대로 넘긴다. dist_m/angle_deg는
        // Qt 값 그대로 - 서버는 재계산하지 않는다. 펜 오프셋 보정은 로봇이
        // TURN 실행 시 스스로 한다.
        segs = planProgram_;
        logf("[INFO] Qt 동작 시퀀스 사용 (%zu 동작)", segs.size());
    } else {
        // 하위호환: program이 없으면 종전대로 서버가 도면에서 직접 생성.
        // 시작점에 서 있으므로 남은 경로 = 두번째 점부터.
        // withNozzleOps로 감싸는 이유: 노즐 제어의 단일 결정권이 NOZZLE op이라
        // (path_planner.hpp 참고), MOVE.paint만 실어 보내면 로봇이 노즐을
        // 영영 안 내린다. buildSegments 결과는 반드시 이걸 통과시킬 것.
        std::vector<Pt> rest(planPts_.begin() + 1, planPts_.end());
        std::vector<char> restPaint;
        if (planPaint_.size() == planPts_.size())
            restPaint.assign(planPaint_.begin() + 1, planPaint_.end());
        segs = withNozzleOps(buildSegments(pose_, rest, /*firstPaint=*/true,
                             restPaint.empty() ? nullptr : &restPaint));
    }
    if (segs.empty()) {  // 도색할 구간 없음 (드문 경계 케이스) - 그린 것으로 친다
        logf("[INFO] 도색할 구간 없음 - 곧바로 완료 처리");
        awaitingArrival_ = false;
        planActive_ = false;
        activeSegs_ = json::array();
        srv_.sendTo("QT", makeMsg("DRAW_DONE", json::object()));
        return;
    }
    if (srv_.sendTo("ROBOT", makePathMsg(segs, "draw"))) {
        awaitingArrival_ = false;
        planActive_ = true;
        activeSegs_ = segs;
        alignSegIdx_ = -1, alignTries_ = 0; clearPendingReady();
        lastPlanMs_ = nowMs();
        logf("[INFO] 2단계 도색 경로 전송 (%zu 세그먼트) - 도색 시작", segs.size());
    } else {
        logf("[WARN] 2단계 도색 경로 전송 실패 - 로봇 미접속");
        sendDrawFail("draw", "robot_offline", "로봇 미접속 - 경로 전송 실패");
        awaitingArrival_ = false;
        planActive_ = false;
    }
}

// 이탈 복귀 경로: 로봇을 planPts_[k]로 되돌린 뒤 원래 하던 일을 이어가게 한다.
//
// ⚠️ Qt program이 있으면 절대 다시 만들지 않는다. 재생성하면 Qt 화면의
// 미리보기와 실제 실행이 달라져 조작자가 동선을 검증할 수 없게 된다.
// 대신 "복귀 구간만 새로 만들고 원본을 잘라 이어 붙인다". 펜 오프셋 보정은
// 로봇이 자기 TURN 실행 중에 스스로 하므로(program에는 안 드러남) 여기서는
// 신경 쓰지 않는다 - k번 꼭짓점 이후를 담당하는 첫 op을 찾아 그 앞에 복귀
// 주행만 이어붙이면 끝이다.
//
//   [NOZZLE 올림]            복귀 주행 중에는 칠하지 않는다
//   [현재 pose -> planPts_[k]]  로봇 "중심"을 꼭짓점에 (buildSegments)
//   [TURN -> 재개 방위]
//   [원본 program의 재개 지점부터 끝까지]
json Router::buildRecovery(size_t k) {
    if (planPts_.size() < 2 || k >= planPts_.size()) return json::array();

    // ----- 하위호환: program이 없으면 남은 도면으로 직접 생성 (종전 동작) -----
    if (!planProgram_.is_array() || planProgram_.empty()) {
        std::vector<Pt> rest(planPts_.begin() + k, planPts_.end());
        std::vector<char> restPaint;
        if (planPaint_.size() == planPts_.size()) {
            restPaint.assign(planPaint_.begin() + k, planPaint_.end());
            restPaint[0] = 0;  // 복귀 구간(현재 위치 -> pts[k])은 칠하지 않는다
        }
        // 여기도 NOZZLE을 끼워 넣는다 (sendDrawPath 폴백과 같은 이유).
        return withNozzleOps(buildSegments(pose_, rest, /*firstPaint=*/false,
                             restPaint.empty() ? nullptr : &restPaint));
    }

    // ----- 재개 지점 찾기: v >= k인 첫 op -----
    size_t start = planProgram_.size();
    for (size_t i = 0; i < planProgram_.size(); ++i) {
        if (planProgram_[i].value("v", -1) >= (int)k) { start = i; break; }
    }
    if (start >= planProgram_.size()) {
        // k가 program의 모든 op보다 뒤 - 남은 program이 없다는 뜻(주로 마지막
        // 꼭짓점 부근에서 이탈했을 때). 이어붙일 동작이 없으니 꼭짓점까지만
        // 데려간다 - 그게 이 복귀에서 할 수 있는 전부다.
        json segs = json::array();
        segs.push_back({{"op", "NOZZLE"}, {"down", false}});
        std::vector<Pt> to{planPts_[k]};
        std::vector<char> noPaint{0};
        json back = buildSegments(pose_, to, /*firstPaint=*/false, &noPaint);
        for (auto& s : back) segs.push_back(s);
        logf("[INFO] 복귀 경로: %zu번 꼭짓점 이후 남은 program 없음 - "
             "꼭짓점까지만 이동", k);
        return segs;
    }

    // 재개 동작이 바라볼 방위. NOZZLE처럼 방위가 없는 op면 뒤에서 찾는다.
    double heading = 1e9;
    for (size_t i = start; i < planProgram_.size() && heading > 1e8; ++i)
        heading = planProgram_[i].value("heading_deg", 1e9);
    if (heading > 1e8) heading = pose_.theta * 180.0 / M_PI;

    json segs = json::array();
    segs.push_back({{"op", "NOZZLE"}, {"down", false}});

    // 로봇 중심을 꼭짓점으로 (펜 오프셋 보정은 로봇이 TURN 실행 시 스스로 함)
    std::vector<Pt> to{planPts_[k]};
    std::vector<char> noPaint{0};
    json back = buildSegments(pose_, to, /*firstPaint=*/false, &noPaint);
    double arrival = pose_.theta * 180.0 / M_PI;
    for (auto& s : back) {
        if (s.value("op", "") == "MOVE") arrival = s.value("heading_deg", arrival);
        segs.push_back(s);
    }

    double turn = normDeg(heading - arrival);
    if (std::fabs(turn) > 2.0)
        segs.push_back({{"op", "TURN"},
                        {"angle_deg", std::round(turn * 10) / 10},
                        {"heading_deg", std::round(heading * 10) / 10}});
    // 재개 지점이 곧바로 도색 MOVE면 노즐을 내려준다 (위에서 올려놨으므로).
    // program이 NOZZLE로 시작하는 정상 경우엔 중복되지 않는다.
    if (planProgram_[start].value("op", "") == "MOVE" &&
        planProgram_[start].value("paint", false))
        segs.push_back({{"op", "NOZZLE"}, {"down", true}});

    for (size_t i = start; i < planProgram_.size(); ++i)
        segs.push_back(planProgram_[i]);
    return segs;
}

// 경로 생성/전송 실패(또는 대기) 시 Qt에 통지. reason 코드로 상황 구분.
void Router::sendDrawFail(const char* stage, const char* reason,
                          const std::string& msg) {
    srv_.sendTo("QT", makeMsg("DRAW_FAIL",
        {{"stage", stage}, {"reason", reason}, {"msg", msg}}));
}
