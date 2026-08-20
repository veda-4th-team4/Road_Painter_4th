#include "router.hpp"
#include "log.hpp"

void Router::onMessage(const std::string& role, const json& msg) {
    // 세션 스레드마다 진입하므로 라우터 상태 접근을 직렬화 (router.hpp mtx_ 참고)
    std::lock_guard<std::mutex> lk(mtx_);
    if (role == "QT") fromQt(msg);
    else if (role == "ROBOT") fromRobot(msg);
    else if (role == "CCTV") fromCctv(msg);
    else if (role == "ADMIN") fromAdmin(msg);
    // 이번 메시지로 조건이 충족됐을 수 있으니(POS로 새 pose가 들어왔거나, 그냥
    // 시간이 지났거나) 아래 셋을 한 곳에서 회수한다. POS가 완전히 끊겨도
    // 로봇 STATUS(500ms 주기)가 heartbeat가 되어 이 셋을 계속 돌려준다.
    sweep();
}

// 시간이 지나 조건이 충족됐을 수 있는 것들을 한곳에서 회수한다.
// onMessage 꼬리와 tick() 둘 다에서 불린다 - 메시지가 왔을 때는 즉시 반응하고,
// 아무것도 안 오면 tick이 뒤늦게라도 챙긴다.
// ⚠️ 호출부가 이미 mtx_를 쥐고 있어야 한다.
void Router::sweep() {
    checkPosLoss();
    resolveBoundary();
    logPosStats();
    // 캘리 세션도 같은 방식으로 회수한다. 종결 응답이 안 오는 세션을 서버가
    // 먼저 접지 않으면 calibActive_가 켜진 채 남아 이후 요청이 전부 busy가 된다.
    checkCalibTimeout();
    // 오도메트리 캡처 ack 개별 타임아웃 - 세션 전체 타임아웃(위)보다 훨씬
    // 짧게 걸려서 CCTV가 침묵하는 순간 3m 주행을 계속 돌리지 않는다.
    checkOdoCaptureTimeout();
    // 채널 간 정합 수집 - 주기적으로 REGISTER_CAPTURE를 다시 쏘고, 세션
    // 데드라인/종료 ack 타임아웃을 본다 (router_registration.cpp).
    checkRegistrationTick();
}

// 주기 호출 (main의 tick 스레드).
//
// 🔴 예전에는 위 회수 작업이 onMessage 꼬리에서만 돌았다. "로봇 STATUS가
//   500ms마다 오니 그게 heartbeat"라는 전제였는데, 그 전제가 깨지는 경우가
//   정확히 이 타이머가 필요한 경우다: 로봇이 TCP는 붙잡은 채 응답만 멈추면
//   (펌웨어 헹, 시리얼 두절) 아무 메시지도 안 오고, 그래서 타임아웃 판정 자체가
//   돌지 않아 캘리 세션이 영원히 busy로 남았다. onPeerChange도 소켓이 살아
//   있으니 안 불린다. 감시자가 감시 대상의 생존에 의존하면 안 된다.
void Router::tick() {
    std::lock_guard<std::mutex> lk(mtx_);
    sweep();
}

// TlsServer가 세션 등록/정리 시 통지 (재접속 교체는 false 없이 넘어감 - tls_server.cpp 참고)
void Router::onPeerChange(const std::string& role, bool connected) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (role == "QT") {
        if (connected) {
            sendPeers();  // 막 접속한 QT에 현재 스냅샷 1회 전송
            return;
        }
        // 🔴 QT가 소유한 캘리 세션은 주인이 사라졌다. 예전에는 여기서 그냥
        //   return이라 세션이 그대로 남았는데, 정적 앵커 방식에서는 로봇이
        //   움직이지 않아 "타임아웃까지 busy"로 끝났다. 오도메트리 방식은
        //   다르다 - **주인 없는 로봇이 계속 사각형을 그린다.** 조작자는 Qt
        //   화면이 죽은 것을 보고 자리를 뜨거나 로봇 쪽으로 걸어간다.
        if (calibActive_ && calibOwner_ == CalibOwner::QT) {
            if (calibIsOdo_)
                abortOdoCalib("qt_offline",
                              "Qt 연결이 끊겨 주행을 중단했습니다.");
            else
                failCalib("qt_offline",
                          "Qt 연결이 끊겨 캘리브레이션을 중단했습니다.");
        }
        return;
    }
    if (role != "ROBOT" && role != "CCTV") return;  // QT 관심사만 (ADMIN 등 무시)
    sendPeers();
    logf("[INFO] PEERS 통지 - %s %s", role.c_str(), connected ? "접속" : "해제");
    // 로봇이 빠지면 STATUS 로그 억제 상태를 비운다 - 재접속 후 첫 STATUS가
    // 이전과 같은 값이어도 한 줄 남아야 "다시 붙었고 이 상태다"가 보인다.
    if (role == "ROBOT" && !connected) clearStatusLog();
    // 로봇이 빠지면 진행 중이던 경로는 의미가 없다. 상태를 접지 않으면
    // planActive_가 켜진 채로 남아 재접속 후 START_DRAW가 "busy"로 거절된다.
    if (role == "ROBOT" && !connected && (planActive_ || awaitingArrival_)) {
        logf("[WARN] 경로 실행 중 로봇 연결 끊김 - 경로 폐기");
        sendDrawFail("draw", "robot_offline", "경로 실행 중 로봇 연결 끊김");
        clearPath();
    }
    // 캘리 세션도 마찬가지다. 여기서 접지 않으면 결과를 만들어줄 상대가 사라진
    // 채로 타임아웃(기본 3분)까지 Qt가 대기 화면에 갇힌다 - 원인을 이미 아는데
    // 3분을 기다리게 할 이유가 없다.
    if (!connected && calibActive_) {
        const char* reason = role == "ROBOT" ? "robot_offline" : "cctv_offline";
        const std::string m = std::string(role == "ROBOT" ? "로봇" : "카메라") +
                              " 연결이 끊겨 캘리브레이션을 계속할 수 없습니다.";
        // 🔴 카메라만 빠지고 로봇은 살아 있는 오도메트리 주행에서는 failCalib이
        //   위험하다. 그건 서버 상태만 정리할 뿐 로봇을 세우지 않으므로, 아무도
        //   보고 있지 않은 로봇이 사각형을 마저 그린다. 로봇이 아직 붙어 있으면
        //   정지 핸드셰이크로 세운다.
        if (calibIsOdo_ && role == "CCTV" && !odoAwaitingResult_) {
            abortOdoCalib(reason, m);
            // 떠난 카메라의 CALIB_STOPPED는 영영 오지 않는다. 그걸 기다리다
            // cancel_failed로 닫으면 "로봇 상태를 직접 확인하세요"라는 엉뚱한
            // 경고가 뜬다 - 정작 확인이 필요한 로봇은 ack를 보내올 것이다.
            cancelAckCctv_ = true;
        } else {
            failCalib(reason, m);
        }
    }
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
        // 캘리는 관리자 창도 개시할 수 있다(2026-07-23 결정, 아직 유효). 다만
        // 세션 상태는 QT 경로와 공유한다 - 로봇이 한 대뿐이라 두 세션이 동시에
        // 돌면 서로의 주행을 자기 관측으로 착각한다.
        if (cmd == "CALIB_START") {
            startCalib(payload, msg, "ADMIN");
            return;
        }
        // 관리자 창은 자기 세션만 취소할 수 있다. 단 {"force":true}면 QT 세션도
        // 강제로 회수한다 - 로봇이 굴러가는 중인데 Qt 단말 앞에 사람이 없을 수
        // 있고, 그때 관리자가 로봇을 세울 방법이 없으면 안 된다 (§3-3).
        if (cmd == "CALIB_CANCEL") {
            cancelCalib(payload, msg, "ADMIN");
            return;
        }
        // 채널 간 정합(registration) 수집 - ADMIN 전용, router_registration.cpp.
        // 로봇에는 아무것도 안 보낸다 - 이 세션은 조이스틱 수동 조작을 전제로
        // 하므로(자동 경로 없음), 로봇 쪽에 새로 알려야 할 상태가 없다.
        if (cmd == "REGISTER_COLLECT_START") {
            startRegistrationCollect(payload);
            return;
        }
        if (cmd == "REGISTER_COLLECT_STOP") {
            stopRegistrationCollect(false, "operator_stop");
            return;
        }
        if (cmd == "REGISTER_COLLECT_CANCEL") {
            stopRegistrationCollect(true, "operator_cancel");
            return;
        }
        // 요약 INFO를 sendTo보다 먼저 (미접속 [WARN]이 뒤따르도록 - fromQt 참고)
        logf("[INFO] (ADMIN) CMD %s -> ROBOT", cmd.c_str());
        srv_.sendTo("ROBOT", msg);
    } else if (type == "PATH") {
        // 관리자 테스트 경로는 손대지 않고 그대로 넘긴다 (v2 ops 형식으로 직접
        // 작성해야 한다 - 서버는 내용을 검사하지도, op_index를 부여하지도 않는다).
        // ⚠️ 서버는 이 경로의 activeMeta_를 모르므로 READY에 GO만 돌려준다.
        logf("[INFO] (ADMIN) PATH -> ROBOT (%zu op)",
             payload.value("ops", json::array()).size());
        srv_.sendTo("ROBOT", msg);
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
        //
        // 캘리브레이션(H_MATRIX)과 같은 규약이다: 카메라는 현장에 한 대뿐이라
        // 주소는 현장 속성이므로 **로그인 여부와 무관하게 전역 슬롯에 먼저** 남기고,
        // 로그인 중이면 그 계정에도 같이 쓴다. 예전에는 로그인이 없으면 아예 거절해서,
        // 설치 기사가 계정을 만들기 전에는 카메라를 등록할 수 없었다.
        std::string ip = payload.value("cam_ip", "");
        users_.setGlobalCamIp(ip);
        if (currentUser_.empty()) {
            srv_.sendTo("QT", makeMsg("SET_CAM_IP_OK",
                                      {{"cam_ip", users_.getGlobalCamIp()}}));
            logf("[INFO] SET_CAM_IP - 로그인 사용자 없음, 전역 슬롯에 저장: '%s'",
                 ip.c_str());
        } else if (!users_.setCamIp(currentUser_, ip)) {
            srv_.sendTo("QT", makeMsg("SET_CAM_IP_FAIL", {{"reason", "저장 실패"}}));
            logf("[WARN] SET_CAM_IP 계정 저장 실패 - 사용자 '%s' (전역에는 저장됨)",
                 currentUser_.c_str());
        } else {
            srv_.sendTo("QT", makeMsg("SET_CAM_IP_OK",
                                      {{"cam_ip", users_.getCamIp(currentUser_)}}));
            logf("[INFO] SET_CAM_IP - 사용자 '%s' + 전역 슬롯에 카메라 IP 저장: '%s'",
                 currentUser_.c_str(), ip.c_str());
        }
    } else if (type == "CMD") {
        std::string cmd = payload.value("cmd", "");
        // START_DRAW: Qt "그림그리기 시작" 버튼. 로봇에 중계하지 않고 서버가
        // 1단계(접근) 경로를 만들어 PATH로 전송한다. 이후 접근 완료(PATH_DONE) ->
        // 2단계(도색) 전송 -> 도색 완료(PATH_DONE) -> DRAW_DONE까지 전부 자동이라
        // Qt가 중간에 더 눌러야 하는 버튼은 없다.
        if (cmd == "START_DRAW") {
            // 🔴 수동 모드 래치를 여기서 푼다 (2026-08-10).
            //   예전에는 새 BLUEPRINT가 와야만 풀렸다. 그래서 "도면 전송 →
            //   대기 중 조이스틱으로 로봇 위치 잡기 → START_DRAW"라는 지극히
            //   자연스러운 순서를 밟으면, 그 작업 **전체**가 ALIGN/MORE/DRIFT
            //   없이 개루프로 돌았다. 조용히 도색 품질만 나빠져서 로그로도
            //   드러나지 않는다.
            //   START_DRAW 시점에는 래치를 유지할 이유가 없다: 접근 경로를
            //   지금 pose에서 새로 만들기 때문에 "서버가 든 경로가 낡았다"는
            //   전제 자체가 사라진다.
            if (manualMode_) {
                manualMode_ = false;
                logf("[INFO] START_DRAW - 수동 모드 해제, 자동 판정 복귀");
            }
            startApproach();
            return;
        }
        // ABORT_DRAW: Qt "작업 중단" 버튼. ESTOP과 달리 받아둔 경로를 버린다
        // (v0.4). ESTOP은 일시정지라 RESUME하면 멈춘 지점부터 이어서 그렸고,
        // 서버도 planActive_가 살아 있어 다음 START_DRAW를 busy로 거절했다.
        // ⚠️ 로봇에도 그대로 중계해야 한다. 서버 경로만 비우고 로봇 버퍼를 안
        //    비우면 로봇이 남은 op을 계속 실행한다 - 화면은 멈췄는데 로봇은
        //    도색을 계속하는 상태가 된다.
        if (cmd == "ABORT_DRAW") {
            const bool wasActive = abortDraw();
            logf("[INFO] CMD ABORT_DRAW -> ROBOT (%s)",
                 wasActive ? "실행 중이던 경로 폐기" : "실행 중인 경로 없음 - 정지만");
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
        // CALIB_START/CALIB_CANCEL: 로봇 주행 호모그래피 세션 (2026-08-10 계약).
        // 예전에는 CALIB_START가 아래 일반 CMD 경로로 흘러 검증도 회신도 없이
        // 그대로 중계됐다 - 도색 중에도 통과했고, 실패하면 Qt는 아무 소식도
        // 못 받았다. 이제 router_calib.cpp가 세션으로 관리한다.
        if (cmd == "CALIB_START") {
            startCalib(payload, msg, "QT");
            return;
        }
        if (cmd == "CALIB_CANCEL") {
            cancelCalib(payload, msg, "QT");
            return;
        }
        // 로봇을 움직이는 수동 조작 (조이스틱).
        bool manualCmd = (cmd == "FORWARD" || cmd == "BACKWARD" ||
                          cmd == "TURN_LEFT" || cmd == "TURN_RIGHT" || cmd == "STOP");
        // 노즐 수동 조작. 차단은 같이 받지만 manualMode_는 켜지 않는다 -
        // 로봇이 움직이지 않으므로 자동 판정과 충돌할 일이 없고, 켜면 대기 중
        // 노즐 점검 한 번에 다음 작업 전체가 개루프로 돈다 (아래 래치 주석 참고).
        bool nozzleCmd = (cmd == "NOZZLE_DOWN" || cmd == "NOZZLE_UP" ||
                          cmd == "PAINT_ON" || cmd == "PAINT_OFF");
        // [A안] 경로 실행(도색) 중에는 수동 조작을 차단한다 - 자동이 우선.
        // 도색 도중 조이스틱으로 로봇을 흔들어 그림을 망치는 것을 막기 위함.
        // 🔴 v2에서는 차단이 더 중요해졌다: 로봇이 READY로 서버 응답을 기다리는
        //   중에 사람이 로봇을 옮기면, 서버가 들고 있는 경로와 실제 위치가
        //   어긋난 채 GO가 나간다. 수동으로 개입하려면 먼저 경로를 끝내거나
        //   새 도면을 보내야 한다.
        // 노즐도 같이 막는다(2026-08-10, QT 검토 §4-2): 도색 op의 노즐 상태는
        // buildDrawOps가 paint+펜오프셋으로 단독 결정하는데, 그 사이에 수동
        // 명령이 서보를 뒤집으면 다음 nozzle op이 올 때까지 반대 상태로 그린다.
        // ⚠️ 관리자 창(fromAdmin)은 이 판정을 거치지 않는다 - 점검용이라
        //    의도적으로 열어둔 경로이므로 수동 노즐 탈출구는 그쪽에 남는다.
        // 단 ESTOP/RESUME/CALIB_START 같은 비수동 명령은 아래로 흘려보내 항상 통과시킨다
        // (비상정지가 막히면 위험).
        if ((manualCmd || nozzleCmd) && planActive_) {
            logf("[WARN] CMD %s 무시 - 경로 실행 중이라 %s 차단", cmd.c_str(),
                 nozzleCmd ? "수동 노즐" : "수동 조작");
            return;
        }
        // 요약 INFO를 sendTo보다 먼저 찍는다 - sendTo가 미접속 시 [WARN]을 그
        // 자리에서 남기므로, 요약이 뒤에 오면 "경고 먼저, 정체는 나중"이라
        // 로그를 읽기 헷갈린다 (어느 명령의 경고인지 위로 스크롤해야 앎).
        logf("[INFO] CMD %s -> ROBOT%s", cmd.c_str(),
             manualCmd ? " [수동모드]" : "");
        srv_.sendTo("ROBOT", msg);
        // 여기 오는 수동 CMD는 경로가 없는(planActive_==false) 상태뿐이다.
        // 수동 조작에 진입하면 자동 판정을 멈춰 충돌을 막는다 - 사람이 로봇을
        // 미는 동안 서버가 옛 경로 기준으로 ALIGN/MORE/DRIFT를 쏘면 서로 싸운다.
        // (해제는 새 BLUEPRINT 또는 START_DRAW - 아래 startApproach 호출부 주석)
        if (manualCmd) manualMode_ = true;
    } else if (type == "BLUEPRINT") {
        // points = Qt가 top-view 픽셀 -> 바닥 미터 변환을 마친 좌표.
        // (top-view 위에 그린 점은 정의상 바닥 평면 위라 높이 보정 불필요)
        blueprint_ = payload;
        planPts_.clear();
        planPaint_.clear();
        planProgram_ = json::array();
        clearPath();
        drawRequested_ = false;  // 이전 도면에 걸려 있던 시작 요청은 무효
        manualMode_ = false;  // 새 도면 = 자동 모드 복귀 (START_DRAW에서도 푼다)
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
        // ----- 선택 필드 (없으면 서버가 points로 직접 만든다) -----
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
        // program: Qt가 입력한 MOVE/TURN/NOZZLE/ARC. 서버가 이것을 로봇 op으로
        // 변환한다 (v1처럼 그대로 중계하지 않는다). dist_m/angle_deg 값 자체는
        // points에서 재계산하지 않는다 - 사용자가 미세조정했을 수 있어서.
        json programJson = payload.value("program", json::array());
        if (programJson.is_array() && !programJson.empty()) {
            // 검증을 빡빡하게 하는 이유: 모르는 op을 그대로 넘기면 로봇이 그
            // op에서 READY를 보낸 채 영구 정지하고, v가 없으면 MORE가 목표
            // 좌표를 못 찾는다. 서버 입구에서 막는 게 현장에서 원인 찾는 것보다
            // 훨씬 싸다. arc 반지름 하한(§5.5)도 여기서 걸러 NaN 전파를 막는다.
            std::string reason, rmsg;
            if (validateProgram(programJson, reason, rmsg)) {
                planProgram_ = programJson;
            } else {
                logf("[WARN] BLUEPRINT program 거부(%s: %s)", reason.c_str(),
                     rmsg.c_str());
                sendDrawFail("plan", reason.c_str(), rmsg);
                // arc 반지름 위반은 도면 자체가 그릴 수 없는 것이라 폴백도
                // 의미가 없다. program 형식 오류는 points로 폴백할 수 있지만,
                // Qt가 의도한 동선과 달라지므로 조용히 대체하지 않는다.
                planPts_.clear();
                return;
            }
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
        // 로그는 값이 바뀐 순간만 남긴다 (2026-08-18). STATUS는 2Hz 하트비트라
        // 그대로 찍으면 초당 두 줄씩 다른 로그를 밀어내는데, 정작 알고 싶은
        // "언제 바뀌었나"는 같은 줄 수백 개 사이에 묻힌다. 중계와 lastStatus_
        // 갱신은 그대로이므로 Qt 화면과 서버 판단에는 아무 영향이 없다.
        std::string state = payload.value("state", "?");
        bool painting = payload.value("painting", false);
        if (!lastStatusSeen_ || state != lastStatusState_ ||
            painting != lastStatusPainting_) {
            logf("[INFO] STATUS: state=%s painting=%s",
                 state.c_str(), painting ? "true" : "false");
            lastStatusState_ = state;
            lastStatusPainting_ = painting;
            lastStatusSeen_ = true;
        }
    } else if (type == "READY") {
        if (!payload.contains("op_index")) {
            // v1 로봇은 READY{"seg":n}을 MOVE 앞에서만 보냈다. 그 로봇을 v2
            // 서버에 붙이면 index 의미가 달라 경로가 엉키므로 맞춰주지 않는다.
            logf("[WARN] READY에 op_index 없음 (v1 로봇?) - 그냥 GO");
            srv_.sendTo("ROBOT", makeMsg("GO", json::object()));
            return;
        }
        onReady(payload.value("op_index", -1));
    } else if (type == "PATH_DONE") {
        // 받은 PATH를 끝까지 수행했다는 로봇의 통지. 어느 단계였는지는 서버 상태로
        // 판단하고, payload.phase는 어긋났을 때 경고를 남기는 용도로만 쓴다
        // (로봇이 phase를 안 실어도 동작하게 - 서버가 단계의 주인).
        std::string phase = payload.value("phase", "");
        if (activePhase_ == "calib") {
            // 로봇 코드(main.cpp R-4)는 마지막 op 완료 후 READY 없이 곧장
            // PATH_DONE을 보낸다 - 9번째(복귀) 캡처는 그래서 여기서 트리거한다.
            onCalibPathDone();
        } else if (awaitingArrival_) {
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
            clearPath();
            srv_.sendTo("QT", makeMsg("DRAW_DONE", json::object()));
            logf("[INFO] 도색 완료 - QT에 DRAW_DONE 통지");
        } else {
            logf("[WARN] PATH_DONE 수신 - 진행 중인 경로 없음 (무시)");
        }
    } else if (type == "CALIB_STOPPED") {
        onCalibStopped("ROBOT");
    } else if (type == "CALIB_FAIL") {
        // 캘리 주행이 실패했다는 로봇의 통지 (motion_failed 등). 이게 없으면
        // Qt는 타임아웃까지 기다린다.
        relayCalibFail(payload);
    } else {
        logf("[WARN] ROBOT으로부터 알 수 없는 type: %s", type.c_str());
    }
}

// ===== READY/GO 핸드셰이크 (§3, §6) =====
//
// 🔴 여기 들어온 READY는 반드시 응답 하나를 받고 나가야 한다 (GO 또는 ALIGN/
//   MORE, 또는 대기 창을 열어 resolveBoundary가 나중에 답하거나). 응답을
//   빠뜨리면 로봇은 영원히 그 자리에 선다.
void Router::onReady(int k) {
    runningOp_ = -1;  // READY = 직전 op 실행이 끝났다는 뜻 (DRIFT 중단)

    // 오도메트리 캘리 경로는 별도 핸드셰이크를 탄다 (CALIB_CAPTURE ack까지
    // GO를 미룸) - 아래 도색용 판정 로직(ALIGN/MORE 대기 창)과는 무관하다.
    // planActive_는 sendPath()가 이미 세워뒀으므로 이 분기는 그 체크보다
    // 먼저 와야 한다 (docs/ROBOT_ODOMETRY_HOMOGRAPHY_WIRE_20260812.md §3).
    if (activePhase_ == "calib") {
        onCalibReady(k);
        return;
    }

    if (!planActive_ || activeMeta_.empty()) {
        sendGo(k, "진행 중인 경로 없음");
        return;
    }
    if (k < 0 || k > (int)activeMeta_.size()) {
        logf("[WARN] READY(op %d) - 경로 범위(0~%zu) 밖", k, activeMeta_.size());
        sendGo(k, "op_index 범위 밖");
        return;
    }
    if (manualMode_) {
        sendGo(k, "수동 모드");
        return;
    }
    // 새 boundary면 재시도 카운터를 0으로 (§6.1). 같은 k로 다시 온 READY는
    // ALIGN/MORE 수행 완료 통지이므로 카운터를 유지한 채 창만 다시 연다.
    if (k != boundaryIdx_) {
        boundaryIdx_ = k;
        alignTries_ = moreTries_ = 0;
        moreSettled_ = false;
    }

    double dummy;
    bool judge = (!moreSettled_ && needsMore(k)) || needsAlign(k, dummy);
    if (!judge) {
        sendGo(k, "판정 대상 아님");
        return;
    }
    // 대기 창을 연다. 기준 시각은 ALIGN/MORE 송신 시각이 아니라 READY 수신
    // 시각이다 - 로봇이 회전/전진하는 동안 흘러간 시간을 대기에 포함시키면
    // 정작 "동작이 끝난 뒤의 장면"을 못 보고 판정하게 된다 (§6.2).
    pendingIdx_ = k;
    pendingMs_ = nowMs();
    pendingSin_ = pendingCos_ = pendingX_ = pendingY_ = 0;
    pendingPoseN_ = 0;
}

void Router::sendGo(int k, const char* why) {
    clearBoundary();
    srv_.sendTo("ROBOT", makeMsg("GO", {{"op_index", k}}));
    runningOp_ = k;
    lastDriftMs_ = 0;  // 새 op 시작 - 첫 DRIFT를 기다리게 하지 않는다
    clearDriftAvg();   // 이전 op의 heading 표본이 새 구간에 섞이면 안 된다
    if (why) logf("[INFO] GO(op %d) - %s", k, why);
    else logf("[INFO] GO(op %d)", k);
}

void Router::clearBoundary() {
    pendingIdx_ = -1;
    pendingSin_ = pendingCos_ = pendingX_ = pendingY_ = 0;
    pendingPoseN_ = 0;
}

// 직전 op(k-1)이 도착 꼭짓점을 아는 move/arc면 거리 오차를 MORE로 교정한다 (§6.1).
//
// 🔴 role은 보지 않는다. 판정 기준은 hasTarget 하나다 - "이 op이 끝났을 때 마커
//   중심이 어디 있어야 하는지 서버가 아는가". 도면 동작(role=path)뿐 아니라
//   오프셋 보정 다리(role=offset, ±a)도 목표를 알기 때문에 대상이다.
//
//   예전에는 isPath를 요구해서 오프셋 다리가 조건에서 통째로 빠졌다. 그래서 매
//   꼭짓점마다 ±a(150mm)를 슬립계수 하나만 믿고 개루프로 찍고, 틀려도 아무도
//   고치지 않았다 - 펜 오프셋 설정값과 실측값이 5mm만 어긋나도 그 오차가 보정
//   없이 꼭짓점마다 그대로 쌓였다(2026-08-11 삼각형 시험에서 관측).
// 🔴 arc는 대상이 아니다 (2026-08-13 실기 시험에서 "원을 망침"으로 관측).
//   MORE는 exitHeadingDeg 방향으로 **직선 이동**을 시키는 보정이다. 직진에는
//   맞지만 호에서는 틀린다 - 원 위의 점에서 접선으로 나가면 원을 벗어나므로,
//   호 끝마다 접선 직선이 덧붙는다. 그것도 more_max_tries(4회)까지 반복한다.
//   설계 의도와도 어긋났다: needsAlign 주석대로 호는 개루프 구간이고 진입
//   pose가 원의 위치를 100% 결정한다 - 진입만 맞추고 끝까지 두는 게 규약이다.
//   호의 거리 오차를 정말 잡으려면 직선이 아니라 추가 호(Δθ)여야 하고, 그건
//   로봇 쪽 op이 하나 더 필요한 별도 작업이다.
bool Router::needsMore(int k) const {
    if (k <= 0 || k > (int)activeMeta_.size()) return false;
    const OpMeta& m = activeMeta_[(size_t)k - 1];
    return m.op == "move" && m.hasTarget && hasHeading(m.exitHeadingDeg);
}

// ALIGN 판정이 가능한 boundary인지. 목표 방위(CCW 절대각)를 targetCcw에 채운다.
//   ① 직전 op이 turn (role 무관)       -> 그 turn이 만들어야 할 방위
//   ② k==0 이고 phase=="draw"          -> 첫 주행 op의 방위 (도색 시작 직전)
//   ③ 다음 op이 role=path인 arc        -> 진입 접선 방향
// ③이 별도 항인 이유: arc는 개루프 구간이라(DRIFT 없음) 진입 pose가 원의 위치를
// 100% 결정한다. 진입 각도가 5도 틀어지면 원 전체가 통째로 어긋난다 (§4.4).
//
// 🔴 ①에서 role=path 요구를 걷어냈다 (2026-08-14 실기 시험).
//   호를 빠져나온 뒤 각도를 맞추는 자리가 통째로 없었다. 호는 개루프라 스윕만큼
//   오차가 그대로 남는데, 그 뒤에 오는 것은 closePaint가 넣는 위상 복원
//   turn(role=offset) 하나뿐이라 ①에 걸리지 않았다. 그래서 호 다음 직선이
//   기울어진 채로 시작했다("호 뒤에 이어 그은 선이 삐뚤어짐").
//
//   이 자리는 노즐이 이미 올라간 뒤라(closePaint가 먼저 올린다) 제자리 회전을
//   시켜도 도료를 문지르지 않는다. 캘리 사각형 경로의 turn은 목표 방위를
//   kNoHeading으로 실어 보내므로 hasHeading 검사에서 그대로 걸러진다.
bool Router::needsAlign(int k, double& targetCcw) const {
    if (k > 0 && k <= (int)activeMeta_.size()) {
        const OpMeta& m = activeMeta_[(size_t)k - 1];
        if (m.op == "turn" && hasHeading(m.exitHeadingDeg)) {
            targetCcw = m.exitHeadingDeg;
            return true;
        }
    }
    if (k == 0 && activePhase_ == "draw") {
        // 경로 맨 앞은 보통 오프셋 move(+a)다. 그 전진이 첫 도색 방향과
        // 어긋나면 펜이 시작점 정중앙에 안 떨어지므로 여기서 미리 맞춘다.
        // 앞쪽에 turn이 먼저 오면 정렬해봐야 그 turn이 다시 돌리므로 건너뛴다.
        // 🔴 role은 보지 않는다. 호 진입 위상 보정 turn(φ)은 role=offset인데,
        //   그걸 건너뛰고 뒤 op의 방위로 정렬하면 그 turn이 φ를 한 번 더 더해
        //   정확히 φ만큼 과회전한 채로 호에 들어간다.
        for (const OpMeta& m : activeMeta_) {
            if (m.op == "nozzle") continue;
            if (m.op == "turn") return false;
            if (hasHeading(m.headingDeg)) {
                targetCcw = m.headingDeg;
                return true;
            }
        }
        return false;
    }
    if (k >= 0 && k < (int)activeMeta_.size()) {
        const OpMeta& m = activeMeta_[(size_t)k];
        if (m.isPath && m.op == "arc" && hasHeading(m.headingDeg)) {
            targetCcw = m.headingDeg;
            return true;
        }
    }
    return false;
}

// 열어둔 대기 창이 다 찼으면 판정한다. 한 boundary에서 MORE와 ALIGN이 둘 다
// 걸릴 수 있으므로(도색 move 직후 곧바로 arc) MORE를 먼저 소진하고 ALIGN을
// 돌린 뒤 GO한다 (§6.1).
void Router::resolveBoundary() {
    if (pendingIdx_ < 0) return;
    // HOLD 중에는 GO/ALIGN/MORE를 일절 보내지 않는다 (§7). 해제될 때
    // 대기 창을 처음부터 다시 세므로 여기서는 그냥 물러난다.
    if (holdActive_) return;
    if (nowMs() - pendingMs_ < params().feedback_wait_ms) return;

    const int k = pendingIdx_;
    // 🔴 창 안에 채택된 POS가 0장이면 판정하지 않고 곧바로 GO (§6.2).
    //   여기서 판정하면 pose_가 직전 판정 때와 글자 그대로 같으므로 오차도 같고,
    //   결국 "똑같은 보정이 한 번 더 나가는 것"이 보장된다 (v1에서 -34.6도
    //   ALIGN이 값까지 동일하게 3번 나가 로봇이 104도를 돈 사례가 있다).
    //   재시도 카운터는 소모하지 않는다 - 실제로 시도한 게 아니라서다.
    if (pendingPoseN_ == 0) {
        logf("[WARN] READY(op %d) %ldms 대기했지만 새 POS 0장 - 직전과 같은 pose라 "
             "판정 불가", k, params().feedback_wait_ms);
        sendGo(k, "POS 0장");
        return;
    }
    const double thetaDeg =
        std::atan2(pendingSin_, pendingCos_) * 180.0 / M_PI;
    const double cx = pendingX_ / pendingPoseN_;
    const double cy = pendingY_ / pendingPoseN_;
    const int n = pendingPoseN_;
    clearBoundary();

    const Params& P = params();

    // ----- ① MORE: 주행 거리 보정 (§6.5) -----
    if (!moreSettled_ && needsMore(k)) {
        const OpMeta& m = activeMeta_[(size_t)k - 1];
        const double head = m.exitHeadingDeg;
        const double ux = std::cos(head * M_PI / 180.0);
        const double uy = std::sin(head * M_PI / 180.0);
        // 🔴 마커 중심의 목표는 꼭짓점보다 진행방향으로 centerAheadM 앞이다.
        //   이 항을 빠뜨리면 도색 구간마다 15cm 전진 오차를 잡고 있다고 착각해
        //   매번 MORE{-0.150}를 쏜다 (§5.2).
        //
        //   값은 op마다 다르다 (ops_builder.hpp OpMeta::centerAheadM 참고).
        //   예전에는 bool이라 "참이면 pen_offset_m"으로 여기서 상수를 꺼내 썼는데,
        //   펜 두께 보정(2026-08-13)이 들어오며 진입(a-w/2)/도색(a+w/2)/이탈(0)로
        //   갈려서 경로를 만든 쪽이 값을 실어 보내는 구조로 바꿨다. 여기서 다시
        //   상수를 쓰면 MORE가 그 두께 보정을 매 boundary마다 되돌린다.
        const double off = m.centerAheadM;
        const double tx = m.penTarget[0] + off * ux;
        const double ty = m.penTarget[1] + off * uy;
        const double dist = (tx - cx) * ux + (ty - cy) * uy;
        if (std::fabs(dist) > P.more_max_m) {
            logf("[WARN] READY(op %d) 거리오차 %.3fm - 상한 %.2fm 초과라 판정 폐기 "
                 "(POS %d장 평균, pose가 틀렸을 가능성)", k, dist, P.more_max_m, n);
            moreSettled_ = true;
        } else if (std::fabs(dist) <= P.more_deadband_m) {
            logf("[INFO] READY(op %d) 거리오차 %.3fm (POS %d장) - MORE 불필요",
                 k, dist, n);
            moreSettled_ = true;
        } else if (moreTries_ >= P.more_max_tries) {
            logf("[WARN] READY(op %d) 거리오차 %.3fm - MORE %d회 소진, 포기",
                 k, dist, P.more_max_tries);
            moreSettled_ = true;
        } else {
            ++moreTries_;
            srv_.sendTo("ROBOT", makeMsg("MORE",
                {{"op_index", k}, {"dist_m", round3(dist)}}));
            logf("[INFO] READY(op %d) 거리오차 %.3fm (POS %d장 평균) - MORE 전송 "
                 "(%d/%d회)", k, dist, n, moreTries_, P.more_max_tries);
            return;  // 로봇이 이동 후 같은 op_index로 READY를 다시 보낸다
        }
    }

    // ----- ② ALIGN: 출발 각도 정렬 (§6.4) -----
    double target;
    if (needsAlign(k, target)) {
        const double err = normDeg(target - thetaDeg);  // CCW 기준 오차
        if (std::fabs(err) <= P.align_threshold_deg) {
            logf("[INFO] READY(op %d) 각도오차 %.1f도 (POS %d장) - 정렬 완료",
                 k, err, n);
        } else if (alignTries_ >= P.align_max_tries) {
            logf("[WARN] READY(op %d) 각도오차 %.1f도 - ALIGN %d회 소진, 포기",
                 k, err, P.align_max_tries);
        } else {
            ++alignTries_;
            // 🔴 여기가 유일한 부호 반전 지점이다. 로봇 대면 양수 = 오른쪽 (§1.3).
            srv_.sendTo("ROBOT", makeMsg("ALIGN",
                {{"op_index", k}, {"angle_deg", round1(toRobotDeg(err))}}));
            logf("[INFO] READY(op %d) 각도오차 %.1f도 (POS %d장 평균) - ALIGN 전송 "
                 "(%d/%d회)", k, err, n, alignTries_, P.align_max_tries);
            return;  // 로봇이 회전 후 같은 op_index로 READY를 다시 보낸다
        }
    }
    sendGo(k, nullptr);
}

// POS 두절 감시 (§7). 마지막으로 "채택된" POS로부터 pos_lost_ms가 지나면
// 로봇을 즉시 세운다 - 서버가 눈을 감은 채로 로봇이 계속 달리는 상황을 막는다.
// 해제는 POS 핸들러 쪽에서(연속 pos_recover_frames장 채택).
void Router::checkPosLoss() {
    if (!planActive_ || holdActive_ || !poseValid_) return;
    if (nowMs() - lastPoseMs_ <= params().pos_lost_ms) return;
    holdActive_ = true;
    posRecoverN_ = 0;
    srv_.sendTo("ROBOT",
                makeMsg("HOLD", {{"hold", true}, {"reason", "pos_lost"}}));
    logf("[WARN] POS %ldms 두절 - HOLD 전송 (로봇 즉시 정지)",
         nowMs() - lastPoseMs_);
}

// POS 수신/채택/폐기 요약. POS 핸들러가 아니라 onMessage() 말단에서 부르는
// 이유: POS가 완전히 끊기면 POS 핸들러는 다시 안 도는데, 그 "0장"이야말로
// 제일 보고 싶은 값이다. 로봇 STATUS(2Hz)가 heartbeat로 이 요약을 밀어준다.
void Router::logPosStats() {
    if (posStatMs_ == 0) return;  // POS를 한 번도 못 받았으면 조용히 (CCTV 미접속 등)
    long dt = nowMs() - posStatMs_;
    if (dt < params().pos_stat_period_ms) return;
    logf("[INFO] POS %.0f초 요약 - 수신 %d, 채택 %d (%.1fHz), 파싱실패 %d, "
         "이상치폐기 %d",
         dt / 1000.0, posRecv_, posAccept_, posAccept_ * 1000.0 / dt,
         posParseFail_, posGateRate_);
    posRecv_ = posAccept_ = posParseFail_ = posGateRate_ = 0;
    posStatMs_ = nowMs();
}

void Router::fromCctv(const json& msg) {
    std::string type = msg.value("type", "");
    json payload = msg.value("payload", json::object());

    if (type == "POS") {
        // corners = CCTV 원본 픽셀 좌표. 좌표 변환(undistort -> H_marker)은
        // 여기(서버)서만 수행 - CCTV는 캘리브레이션 데이터를 가질 필요 없음.
        // 로봇에는 중계하지 않는다 - 로봇은 좌표를 모르며, 보정은 서버가
        // ALIGN/MORE/DRIFT로만 내려준다.
        // 원본 POS는 QT에 중계하지 않는다: Qt가 쓰는 건 아래 POSE(미터 좌표)뿐이고,
        // 원본 픽셀은 Qt에서 해석할 방법이 없다 (캘리브레이션은 서버만 갖고 있음).
        //
        // ----- 채널 필터 (v0.4) -----
        // 활성 채널이 아닌 곳에서 온 마커는 버린다. 받아들이면 pose가 두 채널
        // 사이에서 튄다 - 채널마다 H가 달라 좌표계 자체가 다르기 때문이다.
        // ⚠️ 게이트/카운터보다 먼저 걸러야 한다. 다른 채널의 좌표는 이상치가
        //    아니라 "다른 단위계의 정상값"이라, 게이트를 태우면 연속 폐기
        //    한도만 소모하고 결국 재동기로 받아들여진다.
        const int ch = channelOf(payload);  // ch 없으면 1 (단일 채널 하위호환)
        if (ch != activeChannel_) {
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
        lastPos_ = payload;
        ++posRecv_;
        if (posStatMs_ == 0) posStatMs_ = nowMs();  // 첫 POS부터 요약 구간 시작

        const Params& P = params();
        Pose p;
        if (!poseFromPos(payload, activeCalib(), p)) {
            // 캘리브레이션이 없는 경우만 여기서 소리를 낸다. corners 형식 오류는
            // 매 프레임 같은 로그가 쏟아지므로 카운터로만 세고, 주기 요약
            // (logPosStats)의 "파싱실패" 항목으로 드러낸다.
            ++posParseFail_;
            if (!activeCalib().valid)
                logf("[WARN] POS 수신했으나 채널 %d 캘리브레이션 없음 - pose 계산 불가",
                     activeChannel_);
            return;  // pose를 못 구하면 여기서 끝 (QT로 나가는 건 POSE뿐)
        }
        // ----- 이상치 게이트: 물리적으로 불가능한 각도 변화는 검출 오류다 -----
        // 로봇의 물리적 최대 회전속도는 알려져 있다(약 17.8도/s). 그보다 훨씬
        // 빠른 각도 변화는 주행이 아니라 검출 오류다 - 코너 순서가 한 칸 돌아가면
        // theta가 정확히 90/180도 튀고, 부분 가림·반사로 코너 하나만 어긋나도
        // 수십 도가 어긋난다. 한 프레임이면 엉뚱한 ALIGN/DRIFT가 나가기 충분하다.
        // 허용치 = 상수항(측정 노이즈 몫) + 속도항(실제 회전 몫). 나눗셈이 없어
        // 프레임 간격이 0에 가까워도 터지지 않고, 간격이 길면 자연히 헐거워진다.
        // ⚠️ 허용치에 상한을 두지 말 것. POS가 1~2Hz인 현장에서는 간격이 5~8초까지
        //   벌어지고 그동안 로봇은 물리 상한으로도 100도 넘게 정당하게 돈다
        //   (실측에서 49도 회전이 5회 연속 폐기되어 재동기까지 갔다).
        long dtMs = nowMs() - lastPoseMs_;
        double dTheta = std::fabs(normDeg((p.theta - pose_.theta) * 180.0 / M_PI));
        double allowed = P.pose_gate_base_deg + P.pose_gate_rate_dps * (dtMs / 1000.0);
        if (poseValid_ && dTheta > allowed && poseRejects_ < P.pose_reject_max) {
            ++poseRejects_;
            ++posGateRate_;
            posRecoverN_ = 0;  // HOLD 해제는 "연속" 채택이 조건이다
            logf("[WARN] POS 이상치 - theta %.1f도 급변 (%ldms 내 허용 %.1f도) "
                 "폐기 %d/%d", dTheta, dtMs, allowed, poseRejects_, P.pose_reject_max);
            return;  // pose_ 갱신도, QT 중계도, posSeq_ 증가도 하지 않는다
        }
        if (poseRejects_ >= P.pose_reject_max)
            logf("[WARN] POS 이상치 연속 %d회 - 추적을 놓친 것으로 보고 재동기",
                 poseRejects_);
        poseRejects_ = 0;
        ++posAccept_;
        pose_ = p;
        poseValid_ = true;
        lastPoseMs_ = nowMs();
        ++posSeq_;

        // 계산된 pose(바닥 미터 좌표)를 QT에 전송 - top-view 위 로봇 표시용
        srv_.sendTo("QT", makeMsg("POSE",
            {{"x", round3(p.x)},
             {"y", round3(p.y)},
             {"theta_deg", round1(p.theta * 180.0 / M_PI)}}));

        // ----- HOLD 해제 (§7) -----
        if (holdActive_) {
            if (++posRecoverN_ < P.pos_recover_frames) return;
            holdActive_ = false;
            posRecoverN_ = 0;
            srv_.sendTo("ROBOT", makeMsg("HOLD", {{"hold", false}}));
            logf("[INFO] POS 복구 (연속 %d장 채택) - HOLD 해제, 로봇 재개",
                 P.pos_recover_frames);
            // 유예 중이던 boundary의 대기 창은 처음부터 다시 센다 - HOLD 동안
            // 모은(또는 못 모은) 표본으로 판정하면 정지 전 장면이 섞인다.
            if (pendingIdx_ >= 0) {
                pendingMs_ = nowMs();
                pendingSin_ = pendingCos_ = pendingX_ = pendingY_ = 0;
                pendingPoseN_ = 0;
            }
            return;
        }

        // 수동 조작 중엔 자동 판정을 하지 않는다 (POSE 모니터링은 위에서 이미 전송).
        if (manualMode_) return;
        if (!planActive_) {
            // START_DRAW를 눌렀는데 그때 pose가 없어 미뤄둔 경우에만 접근을 시작한다.
            // (도면만 올라와 있는 상태에서는 로봇을 움직이지 않는다)
            if (drawRequested_) startApproach();
            return;
        }

        // 판정 대기 창이 열려 있으면 표본으로 모아둔다 (§6.2). 기다리는 동안
        // 공짜로 쌓이는 표본이라 지연 없이 노이즈만 1/sqrt(N)로 준다.
        if (pendingIdx_ >= 0) {
            pendingSin_ += std::sin(p.theta);
            pendingCos_ += std::cos(p.theta);
            pendingX_ += p.x;
            pendingY_ += p.y;
            ++pendingPoseN_;
            return;  // 판정 중에는 DRIFT를 보내지 않는다 (로봇은 정지 상태)
        }

        // ----- 주행 중 각도 피드백 (DRIFT, §6.3) -----
        // 🔴 role="path"인 move를 실행 중일 때만 보낸다. arc는 개루프 구간이고
        //   (바퀴 속도비가 고정이라 중간에 틀면 원이 깨진다), 오프셋 move는
        //   15.5cm짜리라 보정할 값이 없다.
        if (runningOp_ >= 0 && runningOp_ < (int)activeMeta_.size()) {
            const OpMeta& m = activeMeta_[(size_t)runningOp_];
            if (!(m.isPath && m.op == "move")) {
                clearDriftAvg();  // 직진 도색 구간이 아니면 표본을 섞지 않는다
            } else if (hasHeading(m.headingDeg)) {
                // 최근 kDriftAvgN개의 평균 heading으로 판정한다. 표본이 아직 덜
                // 찼으면 있는 것만으로 평균한다 - 구간 첫머리에 보정을 통째로
                // 쉬는 것보다는 낫다(그때가 제일 많이 틀어져 있다).
                driftSin_[driftPos_] = std::sin(p.theta);
                driftCos_[driftPos_] = std::cos(p.theta);
                driftPos_ = (driftPos_ + 1) % kDriftAvgN;
                if (driftN_ < kDriftAvgN) ++driftN_;
                double ss = 0, cc = 0;
                for (int i = 0; i < driftN_; ++i) {
                    ss += driftSin_[i];
                    cc += driftCos_[i];
                }
                const double thetaAvg = std::atan2(ss, cc) * 180.0 / M_PI;
                if (nowMs() - lastDriftMs_ >= P.drift_period_ms) {
                    const double err = normDeg(m.headingDeg - thetaAvg);
                    if (std::fabs(err) >= P.drift_deadband_deg) {
                        srv_.sendTo("ROBOT", makeMsg("DRIFT",
                            {{"op_index", runningOp_},
                             {"angle_deg", round1(toRobotDeg(err))}}));
                        lastDriftMs_ = nowMs();
                        // v1은 DRIFT를 로그에 남기지 않아 "보냈는지"를 tap_dump로만
                        // 확인할 수 있었다. 전송률이 drift_period_ms로 제한돼 있어
                        // (기본 2.5Hz) 로그가 뒤덮이지 않으므로 남긴다.
                        // 평균값과 방금 들어온 값을 같이 남긴다 - 둘이 크게
                        // 벌어져 있으면 그 구간 POS가 튀고 있다는 뜻이다.
                        logf("[INFO] DRIFT(op %d) %.1f도 (목표 %.1f, 평균 %.1f "
                             "/ 최근 %.1f, 표본 %d)",
                             runningOp_, toRobotDeg(err), m.headingDeg, thetaAvg,
                             p.theta * 180.0 / M_PI, driftN_);
                    }
                }
            }
        }
    } else if (type == "H_MATRIX") {
        handleHMatrix(msg);
    } else if (type == "CALIB_PROGRESS") {
        relayCalibProgress(msg);
    } else if (type == "CALIB_FAIL") {
        // insufficient_samples / solve_failed 등 - 서버가 만들 수 없는 사유다.
        relayCalibFail(payload);
    } else if (type == "CALIB_STOPPED") {
        onCalibStopped("CCTV");
    } else if (type == "CALIB_CAPTURE_OK") {
        // 오도메트리 캘리 캡처 ack (2026-08-12 신설, router_odocalib.cpp).
        // 정적 앵커 세션과는 다른 메시지 타입이라 여기서 갈릴 필요 없이
        // onCalibCaptureAck 내부에서 calibIsOdo_를 확인한다.
        onCalibCaptureAck(payload, true, "");
    } else if (type == "CALIB_CAPTURE_FAIL") {
        onCalibCaptureAck(payload, false, payload.value("reason", "unknown"));
    } else if (type == "REGISTER_CAPTURE_OK") {
        // 채널 간 정합 캡처 ack (2026-08-15 신설, router_registration.cpp).
        onRegisterCaptureAck(payload, true, "");
    } else if (type == "REGISTER_CAPTURE_FAIL") {
        onRegisterCaptureAck(payload, false, payload.value("reason", "unknown"));
    } else if (type == "REGISTER_FAIL") {
        onRegisterFail(payload);
    } else if (type == "REGISTER_STOPPED") {
        onRegisterStopped(payload);
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
    // 저장된 채널별 번들을 현재 세션에 통째로 복원한다. 예전에는 번들이 하나뿐이라
    // 한 줄이면 됐지만, 이제 채널마다 따로 들고 있어야 한다.
    json storedMap = users_.getCalibs(id);
    // 예전 서버는 수신 즉시 ÷1000 해서 저장했으므로 파일에 남은 번들은 미터다.
    // 규격(2026-08-11)은 Qt로 나가는 번들이 mm이어야 하니 읽는 김에 되돌린다.
    // 저장 파일은 손대지 않는다 - 순수 변환이라 다음 로그인에도 결과가 같다.
    int remmed = 0;
    for (auto it = storedMap.begin(); it != storedMap.end(); ++it)
        if (bundleToMm(*it)) ++remmed;
    if (remmed)
        logf("[INFO] 저장된 캘리브레이션 %d채널을 mm 규격으로 환산해 전달 "
             "(예전 서버가 미터로 저장해 둔 번들)", remmed);
    calibs_.clear();
    for (auto it = storedMap.begin(); it != storedMap.end(); ++it) {
        if (it->is_null()) continue;
        const int ch = std::atoi(it.key().c_str());
        if (!validChannel(ch)) {
            logf("[WARN] 저장된 캘리브레이션에 이상한 채널 키 '%s' - 무시",
                 it.key().c_str());
            continue;
        }
        Calib c;
        if (calibFromJson(*it, c)) calibs_[ch] = c;
        else logf("[WARN] 채널 %d 캘리브레이션 파싱 실패 - 무시", ch);
    }
    const Calib& act = activeCalib();
    // calib(활성 채널 번들)은 v0.3과 의미가 같아 옛 클라이언트도 그대로 동작한다.
    // calibs(채널별 맵)는 4채널 UI가 "어느 채널이 준비됐는지" 표시하는 데 쓴다.
    // 카메라 IP는 "계정 값 → 없으면 전역 값" (user_store.hpp getCamIp 주석 참고).
    // 카메라가 현장에 한 대뿐이라, 계정에 아무것도 없는 새 사용자도 전역 슬롯에
    // 등록된 주소를 그대로 받는다 - 사용자마다 다시 넣을 필요가 없다.
    const json camIp = users_.getCamIp(id);
    json out{{"id", id},
             {"calib", act.valid ? act.raw : json()},
             {"calibs", storedMap},
             {"active_ch", activeChannel_},
             {"cam_ip", camIp}};
    // 중계 RTSP 주소. 설정 파일이 있을 때만 싣는다 - 필드를 null로라도 보내면
    // QT가 "서버가 값을 줬는데 비어있다"와 "안 줬다"를 구분하지 못한다.
    // ⚠️ cam_ip는 그대로 둔다. 저건 카메라 IP(PNO 직결용)라 의미가 다르고,
    //    합치면 PNO로 되돌아갈 수 없게 된다 (stream_cfg.hpp 주석 참고).
    const StreamCfg stream = loadStreamCfg(kStreamCfgFile);
    // 켰으면 주소를, 껐으면 {"enabled":false}를 싣는다. 설정 파일 자체가 없으면
    // 아예 안 싣는다 - QT는 "필드 없음 = 서버가 모름"으로 보고 자기 설정을
    // 유지하고, "enabled=false"는 저장된 중계 주소를 지우라는 지시로 읽는다
    // (stream_cfg.hpp toJson 주석 참고).
    if (stream.shouldSend()) out["stream"] = stream.toJson();
    srv_.sendTo(replyRole, makeMsg("LOGIN_OK", out));
    const std::string camIpStr = camIp.is_string() ? camIp.get<std::string>()
                                                   : std::string("없음 - QT 설정값 사용");
    // 중계는 세 가지 상태를 구분해서 남긴다 - QT의 동작이 셋 다 다르기 때문에,
    // 화면이 이상할 때 로그만 보고 어느 경우인지 바로 알 수 있어야 한다.
    const std::string streamStr =
        stream.valid()   ? stream.base
        : stream.present ? std::string("끔 - QT에 해제 지시(직결)")
                         : std::string("설정 없음 - QT 설정값 유지");
    logf("[INFO] LOGIN %s 성공 (%s 요청, 캘리브레이션 %zu채널 보유, 활성 채널 %d %s, "
         "카메라 IP %s, 중계 %s)",
         id.c_str(), replyRole.c_str(), calibs_.size(), activeChannel_,
         act.valid ? "전달" : "없음 - 캘리브레이션 필요", camIpStr.c_str(),
         streamStr.c_str());
}

// 수신한 캘리브레이션 번들이 이번 운용 규격(QT_CCTV_SERVER_CALIBRATION_FORMAT
// 2026-08-11 §1/§3)에 맞는지 대조해, 어긋난 항목만 로그로 남긴다.
//
// 어긋나도 거절하지 않는다. 규격 위반이어도 좌표는 그럴듯하게 나오기 때문에 조용히
// 틀리는 게 문제지, 캘리를 통째로 막는 건 현장에서 더 나쁘다. "이 값을 믿지 마라"는
// 판단은 영상 크기를 실제로 아는 Qt가 한다(규격 §5-6). 서버는 눈에 보이게만 남긴다.
static void warnCalibSpec(int ch, const json& bundle) {
    if (!bundle.is_object()) return;  // 레거시 단일 H 배열은 메타데이터가 없다
    static constexpr int kSpecW = 2592, kSpecH = 1520;  // §1 이번 운용 조건
    const std::string unit = bundle.value("unit", std::string());
    if (unit != "mm")
        logf("[WARN] 캘리브레이션(채널 %d) unit=%s - 규격은 \"mm\"이다. 이 번들은 "
             "mm로 환산되지 않은 채 저장·중계된다",
             ch, unit.empty() ? "(없음)" : unit.c_str());
    const std::string mode = bundle.value("coord_mode", std::string());
    if (mode != "undistort")
        logf("[WARN] 캘리브레이션(채널 %d) coord_mode=%s - 규격은 \"undistort\"다 "
             "(H에 넣는 픽셀이 K/D로 왜곡 보정된 좌표라는 뜻)",
             ch, mode.empty() ? "(없음)" : mode.c_str());
    const json sz = bundle.value("image_size", json());
    if (!(sz.is_array() && sz.size() == 2 && sz[0] == kSpecW && sz[1] == kSpecH))
        logf("[WARN] 캘리브레이션(채널 %d) image_size=%s - 이번 운용 규격은 [%d,%d]다. "
             "Qt가 디코딩하는 영상 크기와 다르면 좌표가 그만큼 틀어진다",
             ch, sz.is_null() ? "(없음)" : sz.dump().c_str(), kSpecW, kSpecH);
    // coord_mode=undistort인데 K/D가 없으면 서버는 Calib::hasKD 게이트 때문에
    // 왜곡 보정을 조용히 건너뛰고 H를 raw 픽셀에 그대로 적용한다 - 에러 없이
    // 좌표만 틀어진다. 이 프로젝트에서 실제로 겪은 사고 부류다
    // (admin_console/cctv.py 주석: "K·D·H_marker를 버렸다 - 에러 없이 보정만
    // 꺼졌다"). 거부하지 않는다(레거시 번들 호환) - 눈에 보이게만 남긴다.
    if (mode == "undistort" && !(bundle.contains("K") && bundle.contains("D")))
        logf("[WARN] 캘리브레이션(채널 %d) coord_mode=undistort인데 K/D가 없음 - "
             "서버가 왜곡 보정을 건너뛰고 H를 raw 픽셀에 그대로 적용한다 "
             "(에러 없이 좌표만 렌즈 왜곡만큼 틀어짐)", ch);
}

// 캘리브레이션 번들 수신 (CCTV 직접 or 관리자 창 ADMIN 경유 공용). 세 형태를 받는다:
//   중첩:   payload.calib = {K, D, H_floor, H_marker, marker_height_m, version}
//   평면:   payload 자체가 번들 = {calib_id, K, D, H, H_marker, canvas_mm, ...}
//           (QT-REQ-CCTV-001 rev.2 — 바닥 H를 H_floor가 아니라 H로 부른다)
//   레거시: payload.H = [[...]x3] 뿐 (왜곡 보정 없이 바닥/마커 공용으로 사용)
//
// 단위 (QT_CCTV_SERVER_CALIBRATION_FORMAT 2026-08-11): CCTV가 보낸 mm 번들을 **그대로**
// 저장·중계한다. 예전에는 여기서 번들 자체를 ÷1000 해서 저장·중계했는데, 그러면 unit
// 필드와 실제 H 값이 어긋날 여지가 남아 좌표가 조용히 1000배 틀어질 수 있었다.
// 미터 변환은 calibFromJson이 서버 내부 계산용 사본(Calib::Hf/Hm)에만 적용한다 -
// pose/POSE/BLUEPRINT/PATH와 로봇 프로토콜은 예전 그대로 미터다.
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
    // 어느 채널의 번들인가. payload 최상위의 "ch"를 본다 (없으면 1 - 단일 채널
    // 카메라와 v0.3 클라이언트 하위호환). 평면 스키마는 payload 자체가 번들이라
    // "ch"가 번들 안에 남는데, calib_id처럼 손대지 않는 메타데이터로 보존한다 -
    // 저장된 번들만 봐도 어느 채널 것인지 알 수 있어 오히려 낫다.
    const int ch = channelOf(payload);
    // mm은 mm 그대로 둔다. unit이 없으면 옛 CCTV로 보고 "mm"을 박아 둔다 - 이후로는
    // 저장·중계·파싱이 전부 이 필드 하나만 보고 단위를 판단한다.
    const bool assumedUnit = stampCalibUnit(bundle);
    aliasFloorKey(bundle);  // 평면 스키마의 "H"에 "H_floor" 별칭 (QT는 H_floor만 봄)
    if (assumedUnit)
        logf("[WARN] 캘리브레이션(채널 %d) unit 필드 없음 - mm으로 가정했다 "
             "(규격상 필수. CCTV가 unit:\"mm\"을 실어 보내야 한다)", ch);
    warnCalibSpec(ch, bundle);
    Calib c;
    if (!calibFromJson(bundle, c)) {
        logf("[WARN] H_MATRIX(채널 %d) 파싱 실패 - calib/H 형식 확인 필요: %s",
             ch, payload.dump().c_str());
        return;
    }
    // 🔴 격하 감시 (2026-08-13). 이미 들고 있는 것보다 **빈약한** 번들이 덮어쓰는
    //   상황을 크게 남긴다. 값이 그럴듯하고 에러도 안 나서, 실기 시험에서
    //   오도메트리 결과(H_marker+K/D 포함)가 관리자 창의 floor 번들에 덮이는 걸
    //   아무도 눈치채지 못했다 - 서버는 H_marker가 없으면 바닥 H로 대신 쓰므로
    //   (calib.hpp) 두 평면이 조용히 하나로 붕괴하고 로봇 pose가 시차만큼 틀어진다.
    //
    //   막지는 않는다. "최신 번들이 이긴다"가 규약이고, 바닥 재캘리처럼 H_marker가
    //   원래 없는 정당한 갱신도 이 모양이기 때문이다. 판단은 사람이 하되,
    //   그러려면 보여야 한다.
    {
        auto prev = calibs_.find(ch);
        if (prev != calibs_.end() && prev->second.valid) {
            const Calib& p = prev->second;
            std::string lost;
            if (p.hasMarker && !c.hasMarker) lost += "H_marker ";
            if (p.hasKD && !c.hasKD) lost += "K/D ";
            const bool prevId = p.raw.contains("calib_id");
            if (prevId && !bundle.contains("calib_id")) lost += "calib_id ";
            if (!lost.empty())
                logf("[WARN] 캘리브레이션(채널 %d) 격하 - 기존 번들에 있던 [%s]이(가) "
                     "새 번들에 없다. 덮어쓴다(규약)지만 의도한 것인지 확인할 것 "
                     "(기존 calib_id=%s)",
                     ch, lost.c_str(),
                     prevId ? p.raw["calib_id"].dump().c_str() : "(없음)");
        }
    }
    calibs_[ch] = c;
    // mm 번들 그대로 다시 싸서 QT에 중계 + 영속 저장 - QT는 mm H_floor로 top-view.
    json outMsg = msg;
    if (nested) outMsg["payload"]["calib"] = bundle;
    else if (legacyH) outMsg["payload"]["H"] = bundle;
    else outMsg["payload"] = bundle;  // 평면 번들은 payload 자체가 번들
    // 중계본에도 채널을 명시한다. 중첩/레거시 스키마는 ch가 payload 밖이라 원본에
    // 없으면 사라지는데, Qt는 "지금 보고 있는 채널의 번들인가"를 판단해야 한다 -
    // 다른 채널 캘리로 top-view를 갈아엎으면 화면과 좌표가 통째로 어긋난다.
    outMsg["payload"]["ch"] = ch;
    // ----- 캘리 세션의 종결 응답 (2026-08-10 계약 §3.4) -----
    // 🔴 request_id는 서버가 직접 찍는다. CCTV가 되돌려주기를 기대하면 안 된다:
    //   평면 번들 스키마에서는 바로 위 `outMsg["payload"] = bundle`이 payload를
    //   통째로 갈아치우므로 CCTV가 실어 보낸 필드가 여기서 사라진다.
    // 채널이 어긋나면 세션을 닫지 않는다 - 계약 §6 "다른 요청의 늦은 결과가 현재
    // 대기를 풀면 안 된다"와 같은 이유다. 번들 자체는 정상이므로 저장은 한다.
    //
    // ⚠️ 오도메트리 세션도 이 판정을 그대로 탄다. QT 개시면 CALIB_DONE 이후에도
    //    calibActive_가 켜져 있으므로(odoAwaitingResult_) 여기서 정상적으로
    //    종결 처리된다. ADMIN 개시면 이미 꺼져 있어 저장·중계만 하고 지나간다 -
    //    기다리는 Qt가 없으니 그게 맞다.
    const bool closesSession = calibActive_ && ch == calibCh_;
    if (closesSession && !calibReqId_.empty())
        outMsg["payload"]["request_id"] = calibReqId_;
    else if (calibActive_)
        logf("[WARN] H_MATRIX 채널 %d - 캘리 세션은 채널 %d 진행 중이라 "
             "종결로 보지 않음 (저장만)", ch, calibCh_);
    srv_.sendTo("QT", outMsg);
    if (closesSession) {
        logf("[INFO] 캘리 세션 성공 종료 [채널 %d] request_id=%s", ch,
             calibReqId_.empty() ? "(없음)" : calibReqId_.c_str());
        clearCalib();  // 종결 응답(H_MATRIX)을 이미 보낸 뒤라 안전하다
    }
    // ----- 채널 간 정합(registration) 세션의 종결 (2026-08-15 신설) -----
    // REGISTER_COLLECT_STOP -> REGISTER_DONE 을 보낸 뒤 카메라가 성공하면
    // (실패는 REGISTER_FAIL, onRegisterFail()이 따로 접는다) 이 번들로 알려온다
    // - reg_* 필드는 위에서 이미 bundle 그대로 실려 나갔다(손대지 않는다).
    // 여기서는 세션 로컬 상태(regActive_)를 접는 것만 한다. calib* 세션과는
    // 완전히 별개 상태라 위 closesSession 판정과 겹치지 않는다 - 같은
    // H_MATRIX 하나가 두 세션을 동시에 닫을 수도 있다(오도메트리 결과 대기 중에
    // 그 채널을 ch_b로 정합까지 걸어놨다면).
    if (regActive_ && regStopping_ && ch == regChB_) {
        const double regScale = bundle.value("reg_scale", 0.0);
        logf("[INFO] 정합 세션 성공 종료 [ch_a=%d ch_b=%d] request_id=%s "
             "reg_scale=%.4f reg_rmse_mm=%.1f reg_n=%d%s",
             regChA_, regChB_, regReqId_.c_str(), regScale,
             bundle.value("reg_rmse_mm", -1.0), bundle.value("reg_n", 0),
             (regScale > 0.0 && std::fabs(regScale - 1.0) > 0.05)
                 ? " - ⚠️ 스케일이 1.0에서 크게 벗어남, 두 채널 중 한쪽 오도메트리 캘리를 의심할 것"
                 : "");
        clearRegistration();
    }
    // 어느 스키마로 읽혔는지 남긴다 - 평면 번들을 보냈는데 "레거시"로 찍히면
    // K/D가 빠졌다는 뜻이라, 로그만 보고 바로 알 수 있어야 한다.
    const char* schema = nested ? "중첩 calib" : (legacyH ? "레거시 H" : "평면 번들");
    // 로그인 상태와 무관하게 전역 슬롯에 먼저 남긴다 (QT-REQ-SRV-001 R-1).
    // 캘리브레이션은 현장 속성이라, 아무도 로그인하지 않은 채 올려도 서버 재시작 후
    // 살아남아야 한다 - 예전엔 이 경우 메모리에만 남아 그대로 유실됐다.
    users_.setGlobalCalib(ch, bundle);
    const char* detail_marker =
        c.hasMarker ? "H_marker 포함" : "H_marker 없음 - 시차 보정 생략됨";
    const char* detail_kd = c.hasKD ? ", K/D 포함" : ", K/D 없음 - 왜곡 보정 생략됨";
    // ch를 안 실어 보내면 전부 채널 1에 덮어써진다 - 4채널을 캘리한 줄 알았는데
    // 마지막 하나만 남는 상황이라, 어느 채널로 저장됐는지 로그에 반드시 남긴다.
    if (!currentUser_.empty() && users_.setCalib(currentUser_, ch, bundle))
        logf("[INFO] 캘리브레이션 수신 [채널 %d] (%s, mm 보존, %s%s) - "
             "사용자 '%s' + 전역 슬롯에 영속 저장",
             ch, schema, detail_marker, detail_kd, currentUser_.c_str());
    else
        logf("[INFO] 캘리브레이션 수신 [채널 %d] (%s, mm 보존, %s%s) - "
             "로그인 사용자 없음, 전역 슬롯에 영속 저장 (다음 로그인 때 전달됨)",
             ch, schema, detail_marker, detail_kd);
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
    // 접근 목표는 planPts_[0] "그대로"다 - 노즐이 올라가 있으므로 마커 중심을
    // 도면 시작점에 세우면 된다 (§5.2 불변식). 펜 오프셋 보정은 도색 경로의
    // 맨 앞 op이 담당한다.
    // 첫 도색 동작의 방향으로 미리 회전까지 시켜둔다.
    // 🔴 Qt program이 있으면 그쪽 heading_deg를 쓴다. 꼭짓점 두 개를 잇는
    //   직선 방향으로 계산하면 첫 동작이 ARC일 때 크게 어긋난다 - 호의 진입
    //   접선은 시작점과 끝점을 잇는 현(弦)의 방향이 아니기 때문이다
    //   (반원이면 정확히 90도 차이가 난다. 실측으로 확인됨).
    double first = firstEntryHeading(planProgram_);
    if (!hasHeading(first))  // program이 없거나 heading이 없으면 직선으로 폴백
        first = std::atan2(planPts_[1][1] - planPts_[0][1],
                           planPts_[1][0] - planPts_[0][0]) * 180.0 / M_PI;
    PlannedPath path = buildApproachOps(pose_, planPts_[0], first);

    if (path.ops.empty()) {
        // 로봇이 이미 시작점에 첫 도색 방향으로 서 있는 경계 케이스. 접근할 게
        // 없으므로 도착 통지를 기다리지 않고 곧바로 2단계로 넘어간다.
        logf("[INFO] 접근 불필요 (이미 시작점) - 곧바로 도색 경로 전송");
        drawRequested_ = false;
        awaitingArrival_ = true;  // sendDrawPath의 단계 가드 통과용
        sendDrawPath();
        return;
    }
    if (sendPath(std::move(path), "approach")) {
        awaitingArrival_ = true;
        drawRequested_ = false;
        logf("[INFO] 1단계 접근 경로 전송 (%zu op) - 로봇 도착(PATH_DONE) 대기",
             activeOps_.size());
    } else {
        logf("[WARN] 1단계 접근 경로 전송 실패 - 로봇 미접속");
        sendDrawFail("draw", "robot_offline", "로봇 미접속 - 경로 전송 실패");
        drawRequested_ = false;
    }
}

// 2단계: 로봇 접근 완료(PATH_DONE) -> 시작점에 서 있는 로봇에게 도색 경로 전송.
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
        clearPath();
        return;
    }
    // Qt program이 없으면 도면에서 같은 형식의 시퀀스를 만들어 쓴다. 그러면
    // 오프셋 보정·노즐 타이밍 규칙이 buildDrawOps 한 곳에만 존재하게 된다.
    json program = planProgram_;
    if (!program.is_array() || program.empty()) {
        program = programFromPoints(planPts_, planPaint_);
        logf("[INFO] Qt program 없음 - 도면에서 %zu동작 생성", program.size());
    } else {
        logf("[INFO] Qt 동작 시퀀스 사용 (%zu 동작)", program.size());
    }
    PlannedPath path = buildDrawOps(program, planPts_);
    if (!path.ok) {
        logf("[WARN] 도색 경로 생성 실패 (%s: %s)", path.failReason.c_str(),
             path.failMsg.c_str());
        sendDrawFail("draw", path.failReason.c_str(), path.failMsg);
        clearPath();
        return;
    }
    if (path.ops.empty()) {  // 도색할 구간 없음 (드문 경계 케이스) - 그린 것으로 친다
        logf("[INFO] 도색할 구간 없음 - 곧바로 완료 처리");
        clearPath();
        srv_.sendTo("QT", makeMsg("DRAW_DONE", json::object()));
        return;
    }
    if (sendPath(std::move(path), "draw")) {
        awaitingArrival_ = false;
        logf("[INFO] 2단계 도색 경로 전송 (%zu op) - 도색 시작", activeOps_.size());
    } else {
        logf("[WARN] 2단계 도색 경로 전송 실패 - 로봇 미접속");
        sendDrawFail("draw", "robot_offline", "로봇 미접속 - 경로 전송 실패");
        clearPath();
    }
}

// 경로를 로봇에 보내고 실행 상태를 세운다. 새 PATH = 모든 판정 상태 리셋
// (op_index는 PATH 하나 안에서만 유효하므로, 이전 경로의 유예/카운터가
// 남아 있으면 새 경로의 0번 op을 엉뚱하게 판정한다).
bool Router::sendPath(PlannedPath&& path, const std::string& phase) {
    if (!srv_.sendTo("ROBOT", makePathMsg(path.ops, phase))) return false;
    activeOps_ = std::move(path.ops);
    activeMeta_ = std::move(path.meta);
    activePhase_ = phase;
    planActive_ = true;
    runningOp_ = -1;
    boundaryIdx_ = -1;
    alignTries_ = moreTries_ = 0;
    moreSettled_ = false;
    lastDriftMs_ = 0;
    clearBoundary();
    // HOLD는 로봇이 새 PATH를 받아도 유지되지 않는다 - 새 경로는 정지 상태에서
    // READY부터 시작하므로, 세워둘 이유가 사라진다.
    if (holdActive_) {
        holdActive_ = false;
        posRecoverN_ = 0;
        srv_.sendTo("ROBOT", makeMsg("HOLD", {{"hold", false}}));
    }
    return true;
}

void Router::clearPath() {
    planActive_ = false;
    awaitingArrival_ = false;
    activeOps_ = json::array();
    activeMeta_.clear();
    activePhase_.clear();
    runningOp_ = -1;
    boundaryIdx_ = -1;
    alignTries_ = moreTries_ = 0;
    moreSettled_ = false;
    holdActive_ = false;
    posRecoverN_ = 0;
    clearBoundary();
}

// 경로 생성/전송 실패(또는 대기) 시 Qt에 통지. reason 코드로 상황 구분.
void Router::sendDrawFail(const char* stage, const char* reason,
                          const std::string& msg) {
    srv_.sendTo("QT", makeMsg("DRAW_FAIL",
        {{"stage", stage}, {"reason", reason}, {"msg", msg}}));
}
