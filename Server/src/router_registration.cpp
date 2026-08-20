// 채널 간 정합(cross-channel registration) 수집 세션 (2026-08-15 신설).
//
// 규격: docs/REGISTER_WIRE_20260815.md (이 패치와 함께 추가).
// 카메라 쪽 구현: ArucoPosePNM_4ch, homography_mapper.h/.cc
//   (StartRegistration/AddCorrespondencePoint/FinishRegistration/ApplyRegistration),
//   sample_component.cc (CentralRegisterCapture/Done/Cancel).
//
// router_odocalib.cpp(오도메트리 주행 캘리)과 가장 큰 차이는 **로봇 경로가
// 없다**는 것이다 - FOV 겹침 구역이 아직 실측되지 않아 자동 사각형 경로를
// (오도메트리처럼) 짤 근거가 없다. 그래서 조작자가 조이스틱(QT CMD
// FORWARD/BACKWARD/TURN_LEFT/TURN_RIGHT, manualMode_)으로 로봇을 겹침 구역에
// 직접 몬다. 이 세션은 그동안 REGISTER_CAPTURE를 일정 주기로 반복 발사한다 -
// 로봇이 실제로 두 채널 시야 안에 있을 때만 카메라가 OK로 답하고, 아닐 때는
// 그냥 FAIL(not_both_seen)이라 조용히 다음 주기로 넘어간다. 오도메트리의
// READY/GO 핸드셰이크와 달리 한 번 실패해도 세션을 접지 않는다.
//
// 계산(닮음변환 피팅)도 저장(hmm_ch%d.txt, registration_ch%d.txt)도 전부
// 카메라가 한다 - 서버는 REGISTER_CAPTURE를 트리거하고 세션 하나(어느 채널
// 쌍을, 언제부터, 몇 번 캡처했는지)를 관리할 뿐이다. 결과는 기존 H_MATRIX
// 번들에 실려 오고(카메라 쪽 SendCalibBundle() 재사용), handleHMatrix()가
// 이미 그 번들을 그대로 저장·중계하므로 reg_* 필드도 손대지 않고 그대로
// 흘러간다 - 이 파일이 하는 일은 그 도착을 감지해서 regActive_ 를 접는
// 것뿐이다(router.cpp handleHMatrix()의 훅 참고).
//
// ADMIN(관리자 창) 전용이다. QT 소유권/대기 budget 개념이 없다 - 관리자
// 창은 서버 로그를 TAP으로 이미 받고 있으므로, 거절/진행 상황은 회신
// 메시지 없이 로그(logf)로만 남긴다(router_calib.cpp rejectCalib()의
// toQt=false 분기와 같은 판단).
#include "router.hpp"
#include "log.hpp"

void Router::startRegistrationCollect(const json& payload) {
    if (regActive_) {
        logf("[WARN] REGISTER_COLLECT_START 거절 - 이미 채널 %d/%d 정합 수집 중 "
             "(request_id=%s) - 먼저 REGISTER_COLLECT_STOP/CANCEL 할 것",
             regChA_, regChB_, regReqId_.c_str());
        return;
    }
    const int chA = payload.value("ch_a", 0);
    const int chB = payload.value("ch_b", 0);
    if (!validChannel(chA) || !validChannel(chB) || chA == chB) {
        logf("[WARN] REGISTER_COLLECT_START 거절 - ch_a=%d ch_b=%d 이상 "
             "(범위 %d..%d, 서로 달라야 함)", chA, chB, kMinChannel, kMaxChannel);
        return;
    }
    // 두 채널 중 하나라도 오도메트리/앵커 세션이 도는 중이면 거절한다 - 그
    // 채널의 hmm_이 지금 바뀌는 중인데 그 위에서 정합을 모으면 의미가 없다
    // (카메라 쪽 StartOdom()이 정적 앵커 수집과 겹치면 거절하는 것과 같은
    // 이유 - 여기서도 미리 걸러야 "정합을 끝냈는데 실은 낡은 hmm_ 위였다"를
    // 막을 수 있다).
    if (calibActive_ && (calibCh_ == chA || calibCh_ == chB)) {
        logf("[WARN] REGISTER_COLLECT_START 거절 - 채널 %d 캘리 세션이 진행 중"
             " (오도메트리/앵커를 다시 재는 중일 수 있음)", calibCh_);
        return;
    }

    regActive_ = true;
    regChA_ = chA;
    regChB_ = chB;
    // 상관관계 ID는 서버가 발급한다 - 오도메트리는 Qt가 request_id를 주지만,
    // 이 세션은 ADMIN이 여는 채널 쌍일 뿐 그런 발급자가 없다. 시각 기반이면
    // 로그와 대조하기 쉽다(카메라 쪽 SendCalibBundle()의 calib_id 관례와 같다).
    regReqId_ = "reg-" + std::to_string(nowMs());
    regStartMs_ = nowMs();
    regLastCaptureMs_ = 0;  // 다음 sweep()에서 바로 첫 캡처가 나가도록
    regPointIdx_ = 0;
    regOkCount_ = 0;
    regFailCount_ = 0;
    regStopping_ = false;

    logf("[INFO] REGISTER_COLLECT_START [ch_a=%d ch_b=%d] request_id=%s - "
         "%ldms 주기로 REGISTER_CAPTURE 시작. 로봇을 조이스틱으로 겹침 구역에 "
         "몰 것 - 자동 경로 없음(FOV 겹침 미실측)",
         regChA_, regChB_, regReqId_.c_str(), params().reg_capture_interval_ms);
}

// cancel=false: 정상 종료(REGISTER_DONE, 결과는 H_MATRIX 또는 REGISTER_FAIL로 옴).
// cancel=true : 중단(REGISTER_CANCEL, 결과는 REGISTER_STOPPED로 옴).
void Router::stopRegistrationCollect(bool cancel, const char* why) {
    if (!regActive_) {
        logf("[INFO] REGISTER_COLLECT_%s - 열린 정합 세션이 없음 (무시)",
             cancel ? "CANCEL" : "STOP");
        return;
    }
    if (regStopping_) {
        logf("[INFO] REGISTER_COLLECT_%s - 이미 종료 처리 중 (무시)",
             cancel ? "CANCEL" : "STOP");
        return;
    }
    regStopping_ = true;
    regStopMs_ = nowMs();
    srv_.sendTo("CCTV", makeMsg(cancel ? "REGISTER_CANCEL" : "REGISTER_DONE",
        {{"ch_a", regChA_}, {"ch_b", regChB_}, {"request_id", regReqId_}}));
    logf("[INFO] REGISTER_%s 전송 [ch_a=%d ch_b=%d] request_id=%s "
         "유효캡처=%d/%d (%s) - CCTV의 %s 대기",
         cancel ? "CANCEL" : "DONE", regChA_, regChB_, regReqId_.c_str(),
         regOkCount_, regOkCount_ + regFailCount_, why ? why : "",
         cancel ? "REGISTER_STOPPED" : "H_MATRIX(성공) 또는 REGISTER_FAIL(실패)");
}

// sweep()에서 매 tick(기본 200ms) 호출.
void Router::checkRegistrationTick() {
    if (!regActive_) return;

    // 종료 처리 중(REGISTER_DONE/CANCEL 보내고 응답 대기)이면 새 캡처를 쏘지
    // 않는다. 응답이 안 와도 여기서 무한정 기다리지 않는다 - 링크가 끊긴
    // 채로 굳는 것을 막기 위해 ack 타임아웃이 지나면 로컬 상태만 정리한다
    // (카메라가 CENTRAL_TLS 링크로 붙어 있지 않으면 애초에 응답할 수 없다).
    if (regStopping_) {
        if (nowMs() - regStopMs_ > params().reg_capture_ack_timeout_ms * 3) {
            logf("[WARN] REGISTER_COLLECT 종료 응답을 못 받음 [ch_a=%d ch_b=%d] "
                 "request_id=%s - 로컬 상태만 정리 (CCTV 링크가 끊겼을 수 있음)",
                 regChA_, regChB_, regReqId_.c_str());
            clearRegistration();
        }
        return;
    }

    // 세션 전체 데드라인 - 켜둔 채 잊어버렸을 때의 워치독
    // (checkCalibTimeout()과 같은 자리, 오도메트리 쪽은 로봇이 실제로 도는
    // 중이라 별도 정지 확인이 있지만 여기는 로봇 경로가 없어 그냥 접는다).
    if (nowMs() - regStartMs_ > params().reg_session_timeout_ms) {
        logf("[WARN] REGISTER_COLLECT 세션 시간 초과 [ch_a=%d ch_b=%d] (%ldms, "
             "유효캡처 %d/%d) - 자동 취소",
             regChA_, regChB_, params().reg_session_timeout_ms,
             regOkCount_, regOkCount_ + regFailCount_);
        stopRegistrationCollect(true, "session_timeout");
        return;
    }

    if (nowMs() - regLastCaptureMs_ < params().reg_capture_interval_ms) return;
    sendRegisterCapture();
}

void Router::sendRegisterCapture() {
    regLastCaptureMs_ = nowMs();
    const int idx = regPointIdx_++;
    srv_.sendTo("CCTV", makeMsg("REGISTER_CAPTURE",
        {{"ch_a", regChA_}, {"ch_b", regChB_}, {"point_index", idx},
         {"request_id", regReqId_}}));
}

// CCTV의 REGISTER_CAPTURE_OK/FAIL 수신 (router.cpp의 fromCctv()가 type을
// 보고 ok/reason만 채워 위임한다 - onCalibCaptureAck()와 같은 패턴).
void Router::onRegisterCaptureAck(const json& payload, bool ok, const std::string& reason) {
    if (!regActive_) {
        logf("[WARN] REGISTER_CAPTURE_%s 수신 - 진행 중인 정합 세션 없음 (무시)",
             ok ? "OK" : "FAIL");
        return;
    }
    const int chA = payload.value("ch_a", 0);
    const int chB = payload.value("ch_b", 0);
    const std::string reqId = payload.value("request_id", "");
    if (chA != regChA_ || chB != regChB_ || reqId != regReqId_) {
        logf("[WARN] REGISTER_CAPTURE_%s 매칭 실패 (ch_a %d/%d, ch_b %d/%d, "
             "reqId \"%s\"/\"%s\") - 무시", ok ? "OK" : "FAIL",
             chA, regChA_, chB, regChB_, reqId.c_str(), regReqId_.c_str());
        return;
    }

    if (ok) {
        ++regOkCount_;
    } else {
        ++regFailCount_;
        // not_both_seen은 로봇이 아직 겹침 구역 밖이라는, 이 방식에서는 예상된
        // 실패다 - 주기마다 계속 나므로 매번 남기면 로그가 뒤덮인다. 그 외
        // 사유(예: session_refused - ODOM_PREFER 꺼짐/hmm_ 없음)는 계속 실패할
        // 사유라 남긴다.
        if (reason != "not_both_seen") {
            logf("[WARN] REGISTER_CAPTURE_FAIL [ch_a=%d ch_b=%d] point_index=%d "
                 "reason=%s", chA, chB, payload.value("point_index", -1),
                 reason.c_str());
        }
    }
    // 5번마다 한 번 진행 요약 - not_both_seen이 대부분이라도 조작자가 "지금
    // 몇 개나 모였나"를 로그만 보고 알 수 있어야 한다.
    const int total = regOkCount_ + regFailCount_;
    if (total % 5 == 0) {
        logf("[INFO] REGISTER_COLLECT 진행 [ch_a=%d ch_b=%d] 유효캡처 %d/%d",
             regChA_, regChB_, regOkCount_, total);
    }
}

// CCTV의 REGISTER_FAIL(세션 수준 실패, 예: fit_failed) 수신.
void Router::onRegisterFail(const json& payload) {
    if (!regActive_) {
        logf("[WARN] REGISTER_FAIL 수신 - 진행 중인 정합 세션 없음 (무시)");
        return;
    }
    const int chB = payload.value("ch_b", 0);
    const std::string reqId = payload.value("request_id", "");
    if (chB != regChB_ || reqId != regReqId_) {
        logf("[WARN] REGISTER_FAIL 매칭 실패 (ch_b %d/%d, reqId \"%s\"/\"%s\") - 무시",
             chB, regChB_, reqId.c_str(), regReqId_.c_str());
        return;
    }
    logf("[WARN] REGISTER_FAIL [ch_a=%d ch_b=%d] reason=%s (유효캡처 %d/%d) - "
         "정합 수집 세션 종료", regChA_, regChB_,
         payload.value("reason", "unknown").c_str(),
         regOkCount_, regOkCount_ + regFailCount_);
    clearRegistration();
}

// CCTV의 REGISTER_STOPPED(REGISTER_CANCEL에 대한 ack) 수신.
void Router::onRegisterStopped(const json& payload) {
    if (!regActive_) {
        logf("[INFO] REGISTER_STOPPED 수신 - 진행 중인 정합 세션 없음 "
             "(늦은 응답이거나 이미 정리됨)");
        return;
    }
    const std::string reqId = payload.value("request_id", "");
    // request_id만 확인한다 - 카메라의 CentralRegisterCancel()은 채널을 못
    // 읽은 전체 취소 요청에도 응답하는데, 그 ack의 ch_a/ch_b는 "실제로 접은
    // 채널"이라 우리가 보낸 값과 다를 수 있다. request_id는 이 세션 것 그대로
    // 돌아온다.
    if (reqId != regReqId_) {
        logf("[WARN] REGISTER_STOPPED 매칭 실패 (reqId \"%s\"/\"%s\") - 무시",
             reqId.c_str(), regReqId_.c_str());
        return;
    }
    logf("[INFO] REGISTER_STOPPED 수신 [ch_a=%d ch_b=%d] aborted=%d - "
         "정합 수집 세션 종료", regChA_, regChB_, payload.value("aborted", 0));
    clearRegistration();
}

void Router::clearRegistration() {
    regActive_ = false;
    regStopping_ = false;
    regChA_ = regChB_ = 0;
    regReqId_.clear();
    regPointIdx_ = 0;
    regOkCount_ = regFailCount_ = 0;
}
