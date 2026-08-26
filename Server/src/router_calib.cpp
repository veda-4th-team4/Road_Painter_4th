// 로봇 주행 호모그래피 세션 (2026-08-10 신설).
//
// 규격: QT_HOMOGRAPHY_SERVER_CONTRACT_2026-08-10.md (Qt팀 제안),
//       회신 docs/CALIBRATION.md.
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
//
// ⚠️ 소유권 (2026-08-13 추가). 위 "busy로 거절된다"만으로는 부족하다는 게
//    드러났다 - 시작은 막혔지만 **취소는 아무나 할 수 있었다.** ADMIN이
//    request_id 없이 CALIB_CANCEL을 보내면 QT 세션이 그대로 죽고, 진행 중인
//    세션이 없을 때는 누가 눌렀든 QT에 CALIB_CANCELLED가 갔다. calibOwner_로
//    소유자를 명시하고, 취소는 소유자만 - 단 관리자 창의 강제 회수
//    (CALIB_CANCEL{force:true})는 예외로 둔다. 로봇이 실제로 바닥을 굴러다니는
//    중인데 Qt 단말 앞에 사람이 없을 수 있고, 그때 관리자가 로봇을 못 세우면
//    안 되기 때문이다. 상세는
//    docs/CALIBRATION.md '공통 계약'.
#include "router.hpp"
#include "log.hpp"

// 세션 상태를 전부 되돌린다.
// 🔴 단독으로 부르지 말 것. 이걸 부르는 순간 Qt는 "서버가 잊었다"는 사실을 알
//   방법이 없다 - 반드시 종결 응답을 보낸 직후에만 부른다.
void Router::clearCalib() {
    calibActive_ = false;
    calibOwner_ = CalibOwner::NONE;
    calibReqId_.clear();
    calibCh_ = 0;
    calibStartMs_ = 0;
    calibCancelling_ = false;
    calibCancelMs_ = 0;
    cancelAckRobot_ = cancelAckCctv_ = false;
    calibAbortReason_.clear();
    calibAbortMsg_.clear();
    // 오도메트리 세션 상태도 여기서 같이 비운다 - startCalib()가 다시 이
    // 필드들을 세팅하기 전에 이전 세션의 잔재(예: odoPointIdx_)가 남아있으면
    // 다음 세션의 첫 캡처 ack가 엉뚱한 boundary에 GO를 보낼 수 있다.
    calibIsOdo_ = false;
    odoAwaitingResult_ = false;
    odoResultWaitMs_ = 0;
    odoMmm_ = odoNmm_ = 0;
    odoCcw_ = true;
    odoPointIdx_ = -1;
    odoPendingGoOp_ = -2;
    odoCaptureMs_ = 0;
    odoValidCount_ = 0;
    for (int i = 0; i < 9; ++i) {
        odoPixU_[i] = odoPixV_[i] = 0.0;
        odoPixOk_[i] = false;
    }
}

// 세션을 시작하지 못했을 때의 거절 회신 (아직 calibActive_가 아니라
// failCalib()을 쓸 수 없는 자리 - startCalib/startOdoCalib 공용).
//
// ⚠️ ADMIN에는 보내지 않고 로그만 남긴다. 관리자 창은 서버가 주고받는 모든
//    메시지의 사본을 TAP으로 받고 서버 로그도 LOG로 중계받으므로
//    (tls_server.cpp), 여기서 따로 보내지 않아도 거절 사실이 화면에 뜬다.
void Router::rejectCalib(bool toQt, int ch, const std::string& reqId,
                         const char* origin, const char* reason,
                         const std::string& m, const json& extra) {
    if (toQt) {
        json p = {{"ch", ch}, {"request_id", reqId}, {"reason", reason},
                  {"msg", m}};
        for (auto& [k, v] : extra.items()) p[k] = v;
        srv_.sendTo("QT", makeMsg("CALIB_FAIL", p));
    }
    // "요청"으로 뭉뚱그린다 - CALIB_START 거절과 CALIB_CANCEL의 not_owner가
    // 같이 쓰는 자리라 메시지를 START로 못박으면 로그가 거짓말을 한다.
    logf("[WARN] (%s) 캘리 요청 거절 [채널 %d] reason=%s - %s (%s)", origin, ch,
         reason, m.c_str(), toQt ? "QT에 CALIB_FAIL 통지" : "ADMIN - 로그만");
}

// 진행 중인 세션을 실패로 닫는다. ADMIN이 시작한 세션은 QT가 기다리는 것이
// 없으므로 보내지 않는다 - 안 그러면 Qt가 누르지도 않은 작업의 실패창을 본다.
void Router::failCalib(const char* reason, const std::string& m) {
    if (!calibActive_) return;
    const int ch = calibCh_;
    const std::string reqId = calibReqId_;
    const bool toQt = calibToQt();
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

    const CalibOwner owner = fromQt ? CalibOwner::QT : CalibOwner::ADMIN;

    // 실패 회신용 도우미. 아직 세션이 없으므로 failCalib()를 쓸 수 없다
    // (failCalib은 calibActive_를 전제로 한다).
    auto reject = [&](const char* reason, const std::string& m,
                      const json& extra = json::object()) {
        rejectCalib(fromQt, ch, reqId, origin, reason, m, extra);
    };

    // ----- §4-5 멱등: 같은 request_id 재수신은 새 작업을 만들지 않는다 -----
    // 재전송·재접속으로 같은 요청이 두 번 오는 것은 정상이다. 여기서 걸러내지
    // 않으면 로봇이 같은 캘리 주행을 두 번 하고, 두 번째 H_MATRIX가 첫 번째
    // 세션의 종결 응답인 척 Qt의 대기를 잘못 푼다.
    //
    // ⚠️ 소유자까지 봐야 한다. request_id는 개시자가 스스로 만드는 값이라
    //    ADMIN과 QT가 같은 문자열을 쓸 가능성이 0이 아닌데, 그때 소유자를 안
    //    보면 남의 세션에 "재수신"으로 응답하게 된다.
    if (calibActive_ && calibOwner_ == owner && !reqId.empty() &&
        reqId == calibReqId_) {
        if (fromQt)
            srv_.sendTo("QT", makeMsg("CALIB_STARTED",
                {{"ch", calibCh_}, {"request_id", calibReqId_},
                 {"msg", "이미 진행 중인 요청입니다 (상태 재전송)"}}));
        logf("[INFO] (%s) CALIB_START 재수신 - 같은 request_id라 상태만 재전송 "
             "[채널 %d]", origin, calibCh_);
        return;
    }
    if (calibActive_) {
        // owner를 실어 보낸다 - Qt가 "관리자 창이 쓰는 중"과 "내가 이미 시작함"을
        // 문구로 구분할 수 있어야 조작자가 다음 행동을 정한다 (§3-2).
        reject("busy",
               std::string(calibOwner_ == CalibOwner::ADMIN ? "관리자 창이 "
                                                            : "") +
                   "채널 " + std::to_string(calibCh_) +
                   " 캘리브레이션을 진행 중입니다.",
               {{"owner", ownerName(calibOwner_)}});
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

    // ----- 방식 분기: 오도메트리 주행이면 여기서 갈라진다 -----
    // 위 검증(멱등/busy/도색중/채널/로그인/피어접속)은 두 방식 공통이라 같이
    // 통과시켰다. 다른 건 "무엇을 로봇/CCTV에 보내는가"뿐이다 - 정적 앵커는
    // 원본 CALIB_START를 그대로 중계하고(아래), 오도메트리는 서버가 만든
    // 사각형 op을 PATH{phase:"calib"}로 보낸다
    // (docs/CALIBRATION.md '공통 계약').
    //
    // 🔴 method로 가르지 않는다. 두 규격이 같은 값을 쓰기 때문이다:
    //     - 2026-08-10 Qt 계약: Qt가 **정적 앵커** 요청에 method:"robot_motion"을
    //       실어 보낸다 (protocol.hpp:259, docs/CALIBRATION.md
    //       §225 표. "로봇 주행 호모그래피"라는 기능 이름에서 온 값이다)
    //     - 2026-08-12 오도메트리 wire 스펙: 같은 값을 **오도메트리** 방식의
    //       판별자로 다시 썼다
    //   그래서 2026-08-12부터 Qt의 정상적인 정적 앵커 요청이 전부 오도메트리로
    //   해석돼 unsupported_from_qt로 거절돼 왔다 - Qt의 캘리 버튼이 그날부터
    //   죽어 있었다는 뜻이다 (tools/calib_session_test.cpp T1a가 이걸 잡는다).
    //
    //   판별자를 오도메트리 **전용 필드의 존재**로 바꾼다. 이 셋은 정적 앵커
    //   요청에 실릴 이유가 없고, 이미 배포된 Qt/CCTV/관리자 창을 하나도 안 고쳐도
    //   된다. method는 그대로 받되 무시한다.
    const bool hasOdoField = payload.contains("m_cm") ||
                             payload.contains("n_cm") ||
                             payload.contains("start_corner");
    if (hasOdoField) {
        // 사각형 치수 검증. startOdoCalib()이 아니라 여기서 하는 이유는
        // reject() 하나로 QT 회신과 ADMIN 로그가 같이 처리되기 때문이다 -
        // 예전에는 저쪽에서 로그만 남겨서, Qt를 열면 조작자가 왜 아무 일도
        // 일어나지 않는지 알 방법이 없었다.
        const double mCm = payload.value("m_cm", 0.0);
        const double nCm = payload.value("n_cm", 0.0);
        const std::string corner = payload.value("start_corner", "");
        if (corner != "bottom_left" && corner != "top_left") {
            reject("invalid_param",
                   "start_corner가 \"bottom_left\"/\"top_left\"가 아닙니다: \"" +
                       corner + "\"");
            return;
        }
        // 반쪽 구간(각 변을 반으로 쪼갠 것, wire 스펙 §2-1)이 min_move_m 미만이면
        // 그 op이 경로 생성 필터에 걸려 로봇이 실행할 수 없는 미세 동작이 된다.
        const double halfMin = params().min_move_m;
        if (!(mCm > 0.0) || !(nCm > 0.0) || mCm / 200.0 < halfMin ||
            nCm / 200.0 < halfMin) {
            char b[192];
            snprintf(b, sizeof b,
                     "가로/세로가 너무 작습니다 (m=%.1fcm n=%.1fcm, 각 변의 "
                     "절반이 %.0fcm 이상이어야 합니다)",
                     mCm, nCm, halfMin * 100.0);
            reject("invalid_param", b);
            return;
        }
        // 상한. 하한과 달리 기하가 아니라 안전 장치다 - 이 값이 그대로 로봇의
        // 실제 주행 거리가 되므로, 90을 900으로 잘못 친 요청이 통과하면 로봇이
        // 9m 사각형을 그린다. Qt는 하한(2cm)만 검증하고 상한이 없다고 회신했고
        // (2026-08-13 §2), 관리자 창의 브라우저 검증은 개발자도구로 우회된다.
        // 그래서 여기가 두 개시 경로 공통의 유일한 방어선이다.
        const double maxCm = params().calib_odo_max_side_m * 100.0;
        if (mCm > maxCm || nCm > maxCm) {
            char b[192];
            snprintf(b, sizeof b,
                     "가로/세로가 너무 큽니다 (m=%.1fcm n=%.1fcm, 한 변은 "
                     "%.0fcm 이하여야 합니다)",
                     mCm, nCm, maxCm);
            reject("invalid_param", b);
            return;
        }
        startOdoCalib(payload, msg, reqId, ch, fromQt);
        return;
    }

    // ----- 수락 (정적 앵커) -----
    calibActive_ = true;
    calibOwner_ = owner;
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
void Router::cancelCalib(const json& payload, const json& msg,
                         const char* origin) {
    const std::string reqId = payload.value("request_id", "");
    const int ch = channelOf(payload);
    const bool fromQt = (std::string(origin) == "QT");
    // 관리자 창의 강제 회수 (§3-3). Qt가 보내도 의미가 없다 - 아래 소유권
    // 검사에서 QT는 ADMIN 세션을 못 건드리게 막힌다.
    const bool force = payload.value("force", false);

    if (!calibActive_) {
        // 요청한 쪽이 대기 중이고 서버는 이미 세션을 접은 상태 - 여기서
        // 침묵하면 Qt는 자체 타임아웃까지 갇힌다(정적 앵커 5분, 주행 10분).
        // 취소의 목적(대기 해제)은 이미 달성됐으므로 확인해주는 쪽이 맞다.
        //
        // 🔴 요청한 쪽에만 보낸다. 예전에는 origin을 안 받아서, 관리자 창이
        //   취소를 누르기만 해도 Qt에 CALIB_CANCELLED가 날아갔다 - Qt가 아무
        //   작업도 안 하고 있을 때 취소 완료 알림이 뜬다.
        if (fromQt)
            srv_.sendTo("QT", makeMsg("CALIB_CANCELLED",
                {{"ch", ch}, {"request_id", reqId},
                 {"msg", "진행 중인 캘리브레이션이 없습니다 (이미 종료됨)."}}));
        logf("[INFO] (%s) CALIB_CANCEL 수신 - 진행 중인 세션 없음%s", origin,
             fromQt ? ", 대기 해제만 회신" : " (로그만)");
        return;
    }
    // ----- 소유권 (§3-3) -----
    // 시작을 busy로 막는 것만으로는 부족하다 - 취소가 열려 있으면 남의 세션을
    // 죽일 수 있다. 예외는 관리자 창의 강제 회수 하나다: 로봇이 실제로 굴러가는
    // 중인데 Qt 단말 앞에 사람이 없을 수 있고, 그때 관리자가 로봇을 세울 방법이
    // 없으면 안 된다. (ESTOP은 이 검사와 무관하게 항상 통한다 - fromQt/fromAdmin이
    // 캘리 경로를 타지 않고 곧장 ROBOT으로 중계한다.)
    const CalibOwner asker = fromQt ? CalibOwner::QT : CalibOwner::ADMIN;
    bool preempting = false;
    if (calibOwner_ != asker) {
        if (!fromQt && force) {
            preempting = true;
            logf("[WARN] (ADMIN) CALIB_CANCEL{force} - %s 소유 세션을 강제 회수 "
                 "[채널 %d]", ownerName(calibOwner_), calibCh_);
        } else {
            rejectCalib(fromQt, calibCh_, reqId, origin, "not_owner",
                        std::string("이 캘리브레이션은 ") +
                            (calibOwner_ == CalibOwner::ADMIN ? "관리자 창"
                                                              : "Qt") +
                            "에서 시작한 작업이라 중단할 수 없습니다.");
            return;
        }
    }
    // 늦게 도착한 이전 요청의 취소가 지금 세션을 죽이면 안 된다.
    // (강제 회수는 이 검사를 건너뛴다 - 관리자 창은 Qt의 request_id를 모른다.)
    if (!preempting && !reqId.empty() && !calibReqId_.empty() &&
        reqId != calibReqId_) {
        if (fromQt)
            srv_.sendTo("QT", makeMsg("CALIB_FAIL",
                {{"ch", calibCh_}, {"request_id", reqId},
                 {"reason", "cancel_failed"},
                 {"msg", "취소 대상 요청이 현재 진행 중인 요청과 다릅니다."}}));
        logf("[WARN] (%s) CALIB_CANCEL request_id 불일치 (요청 %s, 진행 중 %s) "
             "- 무시", origin, reqId.c_str(), calibReqId_.c_str());
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
        if (preempting)
            abortOdoCalib("preempted",
                          "관리자 창이 캘리브레이션을 회수했습니다.");
        else
            abortOdoCalib("cancelled", "조작자가 취소했습니다.");
        return;
    }
    // 정적 앵커 세션. 강제 회수면 소유자(Qt)에게는 취소가 아니라 실패로 알려야
    // 한다 - 누르지도 않은 취소가 성공한 것처럼 보이면 안 된다.
    if (preempting) {
        calibAbortReason_ = "preempted";
        calibAbortMsg_ = "관리자 창이 캘리브레이션을 회수했습니다.";
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
//
// 🔴 종결 응답이 두 갈래다. 정지 핸드셰이크는 "조작자가 취소했다"와 "실패해서
//   로봇을 세웠다" 양쪽이 공유하는데, Qt에 나가는 메시지는 달라야 한다.
//   calibAbortReason_가 비어 있으면 순수 취소(CALIB_CANCELLED), 값이 있으면
//   실패(CALIB_FAIL{그 reason})다. 예전에는 무조건 CALIB_CANCELLED라
//   capture_timeout으로 죽은 세션이 "안전하게 중단했습니다"로 떴다 - 조작자는
//   자기가 누르지도 않은 취소가 성공한 줄 알고 결과를 기다린다.
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
    const bool toQt = calibToQt();
    const bool wasOdo = calibIsOdo_;  // clearCalib()이 지우기 전에 떼어둔다
    const std::string reason = calibAbortReason_;  // 비어 있으면 순수 취소
    const std::string abortMsg = calibAbortMsg_;
    clearCalib();
    if (wasOdo) clearPath();  // sendPath()로 세운 로봇 경로 상태를 같이 비운다
    if (toQt) {
        if (reason.empty())
            srv_.sendTo("QT", makeMsg("CALIB_CANCELLED",
                {{"ch", ch}, {"request_id", reqId},
                 {"msg", "호모그래피 작업을 안전하게 중단했습니다."}}));
        else
            srv_.sendTo("QT", makeMsg("CALIB_FAIL",
                {{"ch", ch}, {"request_id", reqId}, {"reason", reason},
                 {"msg", abortMsg}}));
    }
    logf("[INFO] 캘리 정지 완료 [채널 %d] reason=%s - ROBOT/CCTV 정지 확인됨%s",
         ch, reason.empty() ? "(취소)" : reason.c_str(),
         !toQt ? " (ADMIN 개시)"
               : reason.empty() ? ", QT에 CALIB_CANCELLED 통지"
                                : ", QT에 CALIB_FAIL 통지");
}

// CCTV가 올린 진행률. 서버는 진행률을 계산할 방법이 없다(샘플 개수도 알고리즘
// 단계도 카메라만 안다). ch/request_id만 채워 그대로 넘긴다.
void Router::relayCalibProgress(const json& msg) {
    if (!calibActive_ || !calibToQt()) return;  // 기다리는 Qt가 없으면 버린다
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
    if (calibIsOdo_) {
        // 주행이 끝나고 카메라의 H 계산을 기다리는 구간 (CALIB_DONE 이후).
        // 여기서는 abortOdoCalib()을 쓰면 안 된다 - 로봇은 이미 서 있고 경로도
        // 비웠으므로 세울 대상이 없다. CALIB_CANCEL을 보내봐야 로봇/카메라의
        // CALIB_STOPPED를 기다리다 cancel_failed로 한 번 더 늘어질 뿐이다.
        if (odoAwaitingResult_) {
            if (now - odoResultWaitMs_ < params().calib_odo_result_wait_ms)
                return;
            failCalib("timeout",
                "주행은 끝났지만 카메라가 " +
                    std::to_string(params().calib_odo_result_wait_ms / 1000) +
                    "초 안에 결과를 보내지 않았습니다. 카메라 상태를 확인하세요.");
            return;
        }
        // 주행 구간. failCalib()을 바로 부르면 안 된다 - 그건 서버 상태만
        // 정리하고 QT에만 통지할 뿐 로봇을 세우지 않는다. abortOdoCalib()이
        // 정지 핸드셰이크로 로봇을 세운 뒤 세션을 닫는다.
        const long budget = odoDriveBudgetMs();
        if (now - calibStartMs_ < budget) return;
        abortOdoCalib("timeout", "제한 시간 " + std::to_string(budget / 1000) +
                                     "초 안에 주행이 끝나지 않았습니다.");
        return;
    }
    if (now - calibStartMs_ < params().calib_timeout_ms) return;
    failCalib("timeout",
              "제한 시간 " + std::to_string(params().calib_timeout_ms / 1000) +
                  "초 안에 결과가 오지 않았습니다. 로봇/카메라 상태를 확인하세요.");
}
