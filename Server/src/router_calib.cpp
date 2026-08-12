// 로봇 주행 호모그래피 세션 (2026-08-10 신설).
//
// 규격: QT_HOMOGRAPHY_SERVER_CONTRACT_2026-08-10.md (Qt팀 제안),
//       회신 docs/QT_HOMOGRAPHY_REPLY_20260810_SERVER.md.
//       메시지 목록은 protocol.hpp의 [로봇 주행 호모그래피 세션] 절에 있다.
//
// 역할 경계: Qt는 "CH n 시작/중단"만 보낸다. 로봇 경로·샘플링 위치·계산 알고리즘은
// 서버도 Qt도 관여하지 않는다 (ROBOT/CCTV 담당). 이 파일이 하는 일은 딱 셋이다.
//   1. 요청을 받아도 되는 상태인지 검증한다 (계약 §4)
//   2. 활성 채널 전환과 세션 개시를 **한 덩어리로** 처리한다 (계약 §7-1)
//   3. 시작한 세션은 반드시 종결 응답 하나로 닫는다
//
// 🔴 3번이 이 파일의 존재 이유다. Qt는 CALIB_START를 보낸 뒤 전체 화면 대기로
//   들어가 채널 전환·새 작업·수동 주행·로그아웃을 전부 막아둔다. 종결 응답
//   (H_MATRIX | CALIB_FAIL | CALIB_CANCELLED)을 빠뜨리면 조작자는 5분 동안
//   아무것도 못 한다. 그래서 아래 모든 이탈 경로 - 사전 검증 실패, 피어 이탈,
//   타임아웃, 취소 ACK 불발 - 가 전부 전송으로 끝난다.
//
// ⚠️ 개시자가 둘이다. 2026-07-23에 "캘리는 관리자 창 담당"으로 정했고 그 경로가
//    아직 살아 있는데, 이번 계약으로 QT도 개시자가 됐다. 둘은 같은 calibActive_를
//    공유하므로 한쪽이 도는 동안 다른 쪽은 busy로 거절된다 - 로봇이 한 대뿐이라
//    동시에 두 세션이 돌면 서로의 주행을 자기 관측으로 착각한다.
#include "router.hpp"
#include "log.hpp"

// 세션 상태를 전부 되돌린다.
// 🔴 단독으로 부르지 말 것. 이걸 부르는 순간 Qt는 "서버가 잊었다"는 사실을 알
//   방법이 없다 - 반드시 종결 응답을 보낸 직후에만 부른다.
void Router::clearCalib() {
    calibActive_ = false;
    calibFromQt_ = false;
    calibReqId_.clear();
    calibCh_ = 0;
    calibStartMs_ = 0;
    calibCancelling_ = false;
    calibCancelMs_ = 0;
    cancelAckRobot_ = cancelAckCctv_ = false;
    // 오도메트리 세션 상태도 여기서 같이 비운다 - startCalib()가 다시 이
    // 필드들을 세팅하기 전에 이전 세션의 잔재(예: odoPointIdx_)가 남아있으면
    // 다음 세션의 첫 캡처 ack가 엉뚱한 boundary에 GO를 보낼 수 있다.
    calibIsOdo_ = false;
    odoMmm_ = odoNmm_ = 0;
    odoCcw_ = true;
    odoPointIdx_ = -1;
    odoPendingGoOp_ = -2;
    odoCaptureMs_ = 0;
    odoValidCount_ = 0;
    odoHaveFirstPix_ = false;
    odoFirstPixU_ = odoFirstPixV_ = 0;
}

// 진행 중인 세션을 실패로 닫는다. ADMIN이 시작한 세션은 QT가 기다리는 것이
// 없으므로 보내지 않는다 - 안 그러면 Qt가 누르지도 않은 작업의 실패창을 본다.
void Router::failCalib(const char* reason, const std::string& m) {
    if (!calibActive_) return;
    const int ch = calibCh_;
    const std::string reqId = calibReqId_;
    const bool toQt = calibFromQt_;
    // 오도메트리 세션은 sendPath()로 로봇 경로 상태(planActive_/activePhase_)를
    // 세워뒀으므로 clearPath()로 같이 비워야 한다. 정적 앵커 세션은 애초에
    // sendPath()를 부르지 않아 이 상태가 항상 비어 있으므로(도색 중엔 계약
    // §4-4가 캘리 시작 자체를 막는다) 여기서 clearPath()를 불러도 안전하지만,
    // 굳이 손대지 않는다 - calibIsOdo_로만 좁혀서 기존 경로 무변경을 보장한다.
    const bool wasOdo = calibIsOdo_;
    clearCalib();
    if (wasOdo) clearPath();
    if (toQt)
        srv_.sendTo("QT", makeMsg("CALIB_FAIL", {{"ch", ch},
                                                 {"request_id", reqId},
                                                 {"reason", reason},
                                                 {"msg", m}}));
    logf("[WARN] 캘리 세션 실패 [채널 %d] reason=%s - %s (%s)", ch, reason,
         m.c_str(), toQt ? "QT에 CALIB_FAIL 통지" : "ADMIN 개시라 통지 없음");
}

// CALIB_START (계약 §3.1). 검증 -> 채널 전환 -> 중계 -> CALIB_STARTED.
//
// 계약 §7-1이 물었던 "SELECT_CHANNEL과 CALIB_START 사이 경쟁 조건"은 여기서
// 구조적으로 사라진다: onMessage() 전체가 mtx_ 하나로 직렬화되므로(router.cpp),
// 아래 applyChannel()과 세션 개시 사이에 다른 메시지가 끼어들 수 없다.
void Router::startCalib(const json& payload, const json& msg, const char* origin) {
    const bool fromQt = (std::string(origin) == "QT");
    const std::string reqId = payload.value("request_id", "");
    // ch 검증에 channelOf()를 쓰지 않는다. 그 함수는 잘못된 값을 조용히 1로
    // 바꾸는데, 여기서 그러면 조작자가 CH3를 눌렀는데 로봇이 CH1을 캘리한다.
    const bool hasCh = payload.contains("ch") && payload["ch"].is_number_integer();
    const bool chOk = hasCh && validChannel(payload["ch"].get<int>());
    // ⚠️ ch는 QT에만 필수다. 관리자 창은 예전부터 {"cmd":"CALIB_START"} 하나만
    //    보내왔고(admin_console/web_gui.py "/robot/cmd"), 단일 채널 PNO 현장도
    //    ch를 실은 적이 없다. 여기서 막으면 살아 있는 버튼이 오늘부터 안 눌린다 -
    //    ADMIN이 ch를 생략하면 "지금 보고 있는 채널"로 읽는다.
    const int ch = chOk           ? payload["ch"].get<int>()
                   : (!hasCh && !fromQt) ? activeChannel_
                                     : 0;  // 0 = 형식 오류 (아래에서 거절)

    // 실패 회신용 도우미. 아직 세션이 없으므로 failCalib()를 쓸 수 없다
    // (failCalib은 calibActive_를 전제로 한다).
    auto reject = [&](const char* reason, const std::string& m) {
        if (fromQt)
            srv_.sendTo("QT", makeMsg("CALIB_FAIL", {{"ch", ch},
                                                     {"request_id", reqId},
                                                     {"reason", reason},
                                                     {"msg", m}}));
        logf("[WARN] (%s) CALIB_START 거절 [채널 %d] reason=%s - %s", origin, ch,
             reason, m.c_str());
    };

    // ----- §4-5 멱등: 같은 request_id 재수신은 새 작업을 만들지 않는다 -----
    // 재전송·재접속으로 같은 요청이 두 번 오는 것은 정상이다. 여기서 걸러내지
    // 않으면 로봇이 같은 캘리 주행을 두 번 하고, 두 번째 H_MATRIX가 첫 번째
    // 세션의 종결 응답인 척 Qt의 대기를 잘못 푼다.
    if (calibActive_ && fromQt && !reqId.empty() && reqId == calibReqId_) {
        srv_.sendTo("QT", makeMsg("CALIB_STARTED",
            {{"ch", calibCh_}, {"request_id", calibReqId_},
             {"msg", "이미 진행 중인 요청입니다 (상태 재전송)"}}));
        logf("[INFO] (%s) CALIB_START 재수신 - 같은 request_id라 상태만 재전송 "
             "[채널 %d]", origin, calibCh_);
        return;
    }
    if (calibActive_) {
        reject("busy", "채널 " + std::to_string(calibCh_) +
                           " 캘리브레이션이 이미 진행 중입니다.");
        return;
    }
    // ----- §4-4: 도색 중에는 시작하지 않는다 -----
    // 예전에는 CALIB_START가 일반 CMD로 취급돼 도색 중에도 그대로 통과했다.
    // fromQt()의 수동조작 차단은 조이스틱 5종만 걸러서 여기까지 오지 못했다.
    if (planActive_ || awaitingArrival_ || drawRequested_) {
        reject("busy", "도색 작업이 진행 중입니다. 먼저 작업을 끝내거나 중단하세요.");
        return;
    }
    if (!validChannel(ch)) {
        reject("invalid_channel",
               "채널 번호가 없거나 범위(" + std::to_string(kMinChannel) + ".." +
                   std::to_string(kMaxChannel) + ") 밖입니다.");
        return;
    }
    // ----- §4-1: 로그인 상태 -----
    // ⚠️ 서버는 로그인 사용자를 세션별이 아니라 전역으로 1명만 기억한다
    //    (currentUser_). 그래서 이건 "이 QT 세션이 로그인했나"가 아니라 "결과를
    //    저장할 계정이 정해져 있나"에 가깝다. 계약 §4-1의 의도(익명 요청 거부)는
    //    QT 개시에 한해 충족되고, ADMIN 개시는 일부러 통과시킨다 - 계정을 만들기
    //    전의 설치 기사가 캘리를 못 하면 안 되기 때문이다(QT-REQ-SRV-001 R-1).
    if (fromQt && currentUser_.empty()) {
        reject("internal_error", "로그인 상태가 아닙니다. 먼저 로그인하세요.");
        return;
    }
    // ----- §4-3: ROBOT/CCTV 접속 -----
    // 여기서 막지 않으면 CALIB_START가 허공으로 나가고, Qt는 아무도 응답하지
    // 않는 세션을 타임아웃까지 기다린다.
    bool robotOn = false, cctvOn = false;
    for (auto& r : srv_.connectedRoles()) {
        if (r == "ROBOT") robotOn = true;
        else if (r == "CCTV") cctvOn = true;
    }
    if (!robotOn) {
        reject("robot_offline", "로봇이 연결되어 있지 않아 시작할 수 없습니다.");
        return;
    }
    if (!cctvOn) {
        reject("cctv_offline", "카메라가 연결되어 있지 않아 시작할 수 없습니다.");
        return;
    }

    // ----- method 분기: 오도메트리 주행이면 여기서 갈라진다 -----
    // 위 검증(멱등/busy/도색중/채널/로그인/피어접속)은 두 방식 공통이라 같이
    // 통과시켰다. 다른 건 "무엇을 로봇/CCTV에 보내는가"뿐이다 - 정적 앵커는
    // 원본 CALIB_START를 그대로 중계하고(아래), 오도메트리는 서버가 만든
    // 사각형 op을 PATH{phase:"calib"}로 보낸다
    // (docs/ROBOT_ODOMETRY_HOMOGRAPHY_WIRE_20260812.md §1, §3).
    if (payload.value("method", "") == "robot_motion") {
        // v1은 Qt 트리거를 지원하지 않는다 (계획서 §1) - 이 방식은 세션이
        // CALIB_DONE에서 끝나고 H_MATRIX는 나중에 별도 경로로 오므로, Qt의
        // 대기 화면을 여닫는 CALIB_STARTED/H_MATRIX 짝이 성립하지 않는다.
        if (fromQt) {
            reject("unsupported_from_qt",
                   "로봇 주행 캘리브레이션은 관리자 창에서만 시작할 수 있습니다.");
            return;
        }
        startOdoCalib(payload, reqId, ch);
        return;
    }

    // ----- 수락 (정적 앵커) -----
    calibActive_ = true;
    calibFromQt_ = fromQt;
    calibReqId_ = reqId;
    calibCh_ = ch;
    calibStartMs_ = nowMs();

    // 채널 전환과 세션 개시가 같은 락 안에서 끝난다 (계약 §7-1).
    // CCTV에는 SELECT_CHANNEL을 따로 만들어 보낸다 - CALIB_START를 그대로
    // 재활용하면 카메라가 "채널을 바꿔라"와 "캘리를 시작해라"를 구분할 수 없다.
    //
    // ⚠️ activeChannel_가 이미 ch라도 **생략하지 않는다.** 그 변수는 서버의
    //    믿음일 뿐이고, 카메라가 그 사이 재시작했으면 실제로는 다른 채널을 보고
    //    있다. 생략하면 서버는 영영 그 어긋남을 바로잡지 못한 채 엉뚱한 채널의
    //    관측으로 캘리를 끝낸다. 한 줄 더 보내는 값은 0이다.
    applyChannel(ch);
    srv_.sendTo("CCTV", makeMsg("CMD", {{"cmd", "SELECT_CHANNEL"}, {"ch", ch}}));
    // Qt에도 채널을 알린다. 계약에는 없지만 Qt는 계산 중 선택 채널만 크게
    // 띄우므로(계약 §5), 서버가 바꾼 활성 채널을 Qt가 모르면 화면과 좌표계가
    // 어긋난다. CHANNEL_OK는 이미 쓰던 신호라 Qt에 새 처리가 필요 없다.
    if (fromQt) {
        const Calib& c = activeCalib();
        srv_.sendTo("QT", makeMsg("CHANNEL_OK",
            {{"ch", ch}, {"calib", c.valid ? c.raw : json()}}));
    }

    // ROBOT/CCTV에는 원본을 그대로 넘긴다 - ch/request_id/method가 보존된다.
    srv_.sendTo("ROBOT", msg);
    srv_.sendTo("CCTV", msg);
    if (fromQt)
        srv_.sendTo("QT", makeMsg("CALIB_STARTED",
            {{"ch", ch}, {"request_id", reqId},
             {"msg", "CH" + std::to_string(ch) +
                         " 로봇 주행 호모그래피를 시작했습니다."}}));
    logf("[INFO] (%s) CALIB_START 수락 [채널 %d] request_id=%s -> ROBOT+CCTV "
         "(타임아웃 %lds)", origin, ch,
         reqId.empty() ? "(없음)" : reqId.c_str(),
         params().calib_timeout_ms / 1000);
}

// CALIB_CANCEL (계약 §3.6).
//
// 🔴 중계했다는 사실만으로 CALIB_CANCELLED를 보내면 안 된다. 그것은 "명령을
//   전달했다"는 뜻이지 "로봇이 섰다"는 뜻이 아니다 - Qt는 이 응답을 보고 대기를
//   풀고 조작자를 다시 화면 앞에 앉히는데, 그 순간 로봇이 아직 굴러가고 있으면
//   사람이 다치는 쪽에 서 있게 된다. ROBOT/CCTV 양쪽의 CALIB_STOPPED를 받고
//   나서야 확인해준다 (ABORT_DRAW의 DRAW_ABORTED와 의도적으로 다른 규약).
void Router::cancelCalib(const json& payload, const json& msg) {
    const std::string reqId = payload.value("request_id", "");
    const int ch = channelOf(payload);

    if (!calibActive_) {
        // Qt만 대기 중이고 서버는 이미 세션을 접은 상태 - 여기서 침묵하면 Qt는
        // 5분 타임아웃까지 갇힌다. 취소의 목적(대기 해제)은 이미 달성됐으므로
        // 확인해주는 쪽이 맞다.
        srv_.sendTo("QT", makeMsg("CALIB_CANCELLED",
            {{"ch", ch}, {"request_id", reqId},
             {"msg", "진행 중인 캘리브레이션이 없습니다 (이미 종료됨)."}}));
        logf("[INFO] CALIB_CANCEL 수신 - 진행 중인 세션 없음, 대기 해제만 회신");
        return;
    }
    // 늦게 도착한 이전 요청의 취소가 지금 세션을 죽이면 안 된다.
    if (!reqId.empty() && !calibReqId_.empty() && reqId != calibReqId_) {
        srv_.sendTo("QT", makeMsg("CALIB_FAIL",
            {{"ch", calibCh_}, {"request_id", reqId}, {"reason", "cancel_failed"},
             {"msg", "취소 대상 요청이 현재 진행 중인 요청과 다릅니다."}}));
        logf("[WARN] CALIB_CANCEL request_id 불일치 (요청 %s, 진행 중 %s) - 무시",
             reqId.c_str(), calibReqId_.c_str());
        return;
    }
    if (calibCancelling_) {
        logf("[INFO] CALIB_CANCEL 재수신 - 이미 정지 확인 대기 중 (ROBOT %s, CCTV %s)",
             cancelAckRobot_ ? "완료" : "대기", cancelAckCctv_ ? "완료" : "대기");
        return;
    }
    // 오도메트리 세션은 로봇이 실제로 굴러가는 중이라 이 함수의 나머지(원본
    // CALIB_CANCEL을 ROBOT에 그대로 중계)로는 로봇을 못 세운다 - 로봇 펌웨어에
    // CALIB_* 핸들러가 없다. abortOdoCalib()이 대신 ABORT_DRAW로 세운다.
    if (calibIsOdo_) {
        abortOdoCalib("cancelled", "조작자가 취소했습니다.");
        return;
    }
    calibCancelling_ = true;
    calibCancelMs_ = nowMs();
    cancelAckRobot_ = cancelAckCctv_ = false;
    srv_.sendTo("ROBOT", msg);
    srv_.sendTo("CCTV", msg);
    logf("[INFO] CALIB_CANCEL -> ROBOT+CCTV [채널 %d] - 양쪽 CALIB_STOPPED 대기 "
         "(%ldms 한도)", calibCh_, params().calib_cancel_ack_ms);
}

// ROBOT/CCTV의 안전 정지 ACK. 둘 다 모여야 Qt의 대기를 푼다.
void Router::onCalibStopped(const std::string& role) {
    if (!calibCancelling_) {
        logf("[WARN] %s CALIB_STOPPED 수신 - 취소를 요청한 적이 없음 (무시)",
             role.c_str());
        return;
    }
    if (role == "ROBOT") cancelAckRobot_ = true;
    else if (role == "CCTV") cancelAckCctv_ = true;
    if (!cancelAckRobot_ || !cancelAckCctv_) {
        logf("[INFO] %s 정지 확인 - 아직 %s 대기 중", role.c_str(),
             cancelAckRobot_ ? "CCTV" : "ROBOT");
        return;
    }
    const int ch = calibCh_;
    const std::string reqId = calibReqId_;
    const bool toQt = calibFromQt_;
    const bool wasOdo = calibIsOdo_;  // clearCalib()이 지우기 전에 떼어둔다
    clearCalib();
    if (wasOdo) clearPath();  // sendPath()로 세운 로봇 경로 상태를 같이 비운다
    if (toQt)
        srv_.sendTo("QT", makeMsg("CALIB_CANCELLED",
            {{"ch", ch}, {"request_id", reqId},
             {"msg", "호모그래피 작업을 안전하게 중단했습니다."}}));
    logf("[INFO] 캘리 취소 완료 [채널 %d] - ROBOT/CCTV 정지 확인됨%s", ch,
         toQt ? ", QT에 CALIB_CANCELLED 통지" : " (ADMIN 개시)");
}

// CCTV가 올린 진행률. 서버는 진행률을 계산할 방법이 없다(샘플 개수도 알고리즘
// 단계도 카메라만 안다). ch/request_id만 채워 그대로 넘긴다.
void Router::relayCalibProgress(const json& msg) {
    if (!calibActive_ || !calibFromQt_) return;  // 기다리는 Qt가 없으면 버린다
    json out = msg;
    out["payload"]["ch"] = calibCh_;
    out["payload"]["request_id"] = calibReqId_;
    srv_.sendTo("QT", out);
}

// CCTV/ROBOT이 알려준 실패. 계약의 motion_failed/insufficient_samples/
// solve_failed는 서버가 만들 수 없는 값이라 이 경로로만 나온다.
void Router::relayCalibFail(const json& payload) {
    const std::string reason = payload.value("reason", "internal_error");
    const std::string m = payload.value("msg", "캘리브레이션에 실패했습니다.");
    if (!calibActive_) {
        logf("[WARN] CALIB_FAIL(%s) 수신 - 진행 중인 세션 없음 (무시)",
             reason.c_str());
        return;
    }
    failCalib(reason.c_str(), m);
}

// 종결 응답 없이 늘어지는 세션을 서버가 먼저 접는다.
//
// 타이머 스레드를 따로 두지 않는다 - onMessage() 꼬리에서 불리고, 로봇 STATUS가
// 500ms마다 오므로 그게 heartbeat 역할을 한다 (checkPosLoss/resolveBoundary와
// 같은 방식). ⚠️ 로봇이 통째로 죽어 STATUS가 끊기면 이 함수도 안 불리는데,
// 그 경우는 onPeerChange가 robot_offline으로 먼저 접는다.
void Router::checkCalibTimeout() {
    if (!calibActive_) return;
    const long now = nowMs();
    if (calibCancelling_) {
        if (now - calibCancelMs_ < params().calib_cancel_ack_ms) return;
        // 여기서 "아마 섰겠지" 하고 CALIB_CANCELLED를 보내면 안 된다.
        // 확인하지 못했다는 사실을 그대로 알린다 - Qt는 대기를 풀되 조작자에게
        // 오류를 띄우므로, 사람이 로봇을 눈으로 확인하러 간다.
        failCalib("cancel_failed",
                  std::string("정지 확인 응답을 받지 못했습니다 (ROBOT ") +
                      (cancelAckRobot_ ? "확인" : "무응답") + ", CCTV " +
                      (cancelAckCctv_ ? "확인" : "무응답") +
                      "). 로봇 상태를 직접 확인하세요.");
        return;
    }
    // 오도메트리 세션은 별개 파라미터(calib_odo_timeout_ms)를 쓴다 - Qt가
    // 관여하지 않아 calib_timeout_ms의 "Qt 5분보다 짧아야" 제약이 없다
    // (params.hpp 참고). failCalib()을 바로 부르면 안 된다 - 그건 서버 상태만
    // 정리하고 QT에만 통지할 뿐 로봇을 세우지 않는다. abortOdoCalib()이
    // ABORT_DRAW로 로봇을 세운 뒤 (CCTV ack까지 받고 나서) 세션을 닫는다.
    if (calibIsOdo_) {
        if (now - calibStartMs_ < params().calib_odo_timeout_ms) return;
        abortOdoCalib("timeout",
            "제한 시간 " + std::to_string(params().calib_odo_timeout_ms / 1000) +
                "초 안에 주행이 끝나지 않았습니다.");
        return;
    }
    if (now - calibStartMs_ < params().calib_timeout_ms) return;
    failCalib("timeout",
              "제한 시간 " + std::to_string(params().calib_timeout_ms / 1000) +
                  "초 안에 결과가 오지 않았습니다. 로봇/카메라 상태를 확인하세요.");
}
