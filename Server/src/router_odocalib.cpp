// 로봇 오도메트리 주행 호모그래피 세션 (2026-08-12 신설).
//
// 규격: docs/ROBOT_ODOMETRY_HOMOGRAPHY_WIRE_20260812.md (정본),
//       docs/ROBOT_ODOMETRY_HOMOGRAPHY_PLAN_20260811.md (설계 근거).
//
// router_calib.cpp(정적 앵커 캘리, 2026-08-10)과 calibActive_/calibReqId_/
// calibCh_/calibStartMs_/calibFromQt_를 공유한다 - busy 판정이 method 무관하게
// 걸리도록 하기 위해서다. calibIsOdo_ 하나로 "지금 도는 세션이 어느 방식인가"를
// 구분한다. startCalib()이 payload.method를 보고 startOdoCalib()으로 분기한다
// (router_calib.cpp).
//
// 로봇 경로는 sendPath(..., "calib")로 보낸다 - 도색 경로와 같은 상태
// (planActive_/activeOps_/activeMeta_/activePhase_)를 그대로 재사용한다.
// READY/PATH_DONE 수신 시 onReady()/PATH_DONE 핸들러가 activePhase_=="calib"을
// 보고 이 파일의 onCalibReady()/onCalibPathDone()으로 위임한다 (router.cpp).
#include "router.hpp"
#include "log.hpp"
#include <cmath>

// CALIB_START{method:"robot_motion"} 수락. router_calib.cpp의 startCalib()이
// 공통 검증(멱등/busy/도색중/채널/로그인/피어접속)을 이미 통과시킨 뒤 호출한다.
//
// ⚠️ ADMIN 개시 전용이다(호출부에서 fromQt를 이미 걸렀다) - 그래서 검증 실패를
// 알릴 QT 회신 채널이 없다. 기존 ADMIN-개시 reject()와 같은 패턴으로 로그만
// 남긴다 ("ADMIN 개시라 통지 없음").
void Router::startOdoCalib(const json& payload, const std::string& reqId, int ch) {
    const double mCm = payload.value("m_cm", 0.0);
    const double nCm = payload.value("n_cm", 0.0);
    const std::string corner = payload.value("start_corner", "");
    if (corner != "bottom_left" && corner != "top_left") {
        logf("[WARN] (ADMIN) CALIB_START(robot_motion) 거절 [채널 %d] - "
             "start_corner가 \"bottom_left\"/\"top_left\"가 아님: \"%s\" "
             "(ADMIN 개시라 통지 없음)", ch, corner.c_str());
        return;
    }
    const double mM = mCm / 100.0, nM = nCm / 100.0;
    const double halfMin = params().min_move_m;
    // 반쪽 구간(각 변을 반으로 쪼갠 것, §2-1)이 min_move_m 미만이면 그 op이
    // 경로 생성 필터에 걸려 로봇이 실행할 수 없는 미세 동작이 된다.
    if (!(mM > 0.0) || !(nM > 0.0) || mM / 2.0 < halfMin || nM / 2.0 < halfMin) {
        logf("[WARN] (ADMIN) CALIB_START(robot_motion) 거절 [채널 %d] - "
             "m_cm=%.1f n_cm=%.1f 너무 작음 (반쪽 구간이 min_move_m=%.3fm 미만, "
             "ADMIN 개시라 통지 없음)", ch, mCm, nCm, halfMin);
        return;
    }
    const bool ccw = (corner == "bottom_left");

    // 채널 전환은 정적 앵커 경로와 동일하게 여기서 한다 - CCTV가 CALIB_CAPTURE를
    // 받기 전에 올바른 렌즈를 보고 있어야 한다.
    applyChannel(ch);
    srv_.sendTo("CCTV", makeMsg("CMD", {{"cmd", "SELECT_CHANNEL"}, {"ch", ch}}));

    PlannedPath path = buildCalibRectOps(mM, nM, ccw);
    if (!sendPath(std::move(path), "calib")) {
        logf("[WARN] (ADMIN) CALIB_START(robot_motion) 거절 [채널 %d] - "
             "로봇에 PATH 전송 실패 (연결 끊김?, ADMIN 개시라 통지 없음)", ch);
        return;
    }

    calibActive_ = true;
    calibFromQt_ = false;  // v1은 Qt 트리거를 지원하지 않는다 (계획서 §1)
    calibReqId_ = reqId;
    calibCh_ = ch;
    calibStartMs_ = nowMs();
    calibIsOdo_ = true;
    odoMmm_ = mCm * 10.0;
    odoNmm_ = nCm * 10.0;
    odoCcw_ = ccw;
    odoPointIdx_ = -1;
    odoPendingGoOp_ = -2;
    odoValidCount_ = 0;
    odoHaveFirstPix_ = false;

    logf("[INFO] (ADMIN) CALIB_START(robot_motion) 수락 [채널 %d] m=%.0fmm n=%.0fmm "
         "start_corner=%s (%s) - PATH(calib) 11-op 전송, 첫 READY부터 캡처 시작 "
         "(타임아웃 %ldms)",
         ch, odoMmm_, odoNmm_, corner.c_str(), ccw ? "CCW" : "CW",
         params().calib_odo_timeout_ms);
}

// READY(k) 수신, activePhase_=="calib"일 때 onReady()가 위임한다 (router.cpp).
//
// boundary(READY의 op_index) -> point_index 매핑은 buildCalibRectOps()의
// 고정 11-op 구조(op 2/5/8이 TURN)에서 나온다 - m/n 값이 달라져도 구조 자체는
// 안 바뀐다. -1은 "직전 op이 TURN이라 같은 물리 위치" (wire 스펙 §4).
void Router::onCalibReady(int k) {
    const int pointIdx = odoReadyToPoint(k);
    if (pointIdx == -2) {
        // 방어적 처리 - 로봇이 범위 밖 op_index를 보내면 그냥 진행시킨다.
        // 막힌 채로 로봇을 세워두는 것보다 낫다 (대응점이 줄어드는 것뿐).
        logf("[WARN] READY(op %d) - 캘리 경로 범위(0~10) 밖 (그냥 GO)", k);
        sendGo(k, "캘리 - op_index 범위 밖");
        return;
    }
    if (pointIdx < 0) {
        sendGo(k, "캘리 - 회전 직후라 같은 위치");
        return;
    }
    odoPendingGoOp_ = k;  // ack가 오면 이 op_index에 GO
    sendCalibCapture(pointIdx);
}

// PATH_DONE(calib) 수신, activePhase_=="calib"일 때 위임한다 (router.cpp).
// 로봇 코드(main.cpp R-4)는 마지막 op 완료 후 READY 없이 곧장 PATH_DONE을
// 보낸다 - 9번째(복귀) 캡처는 그래서 여기서 트리거한다 (wire 스펙 §4).
void Router::onCalibPathDone() {
    odoPendingGoOp_ = -1;  // ack 후 GO 대신 CALIB_DONE
    sendCalibCapture(8);
}

// point_index 0..8의 물리좌표(world_xy_mm)를 계산해 CALIB_CAPTURE를 보내고
// ack 대기 상태를 연다. 좌표 계산 자체는 odoPointWorldMm()(ops_builder.hpp)에
// 있다 - Router 없이도 단위 테스트가 되도록 순수 함수로 뺐다(odo_calib_test.cpp).
void Router::sendCalibCapture(int pointIdx) {
    const auto xy = odoPointWorldMm(pointIdx, odoMmm_, odoNmm_, odoCcw_,
                                    params().marker_offset_m);
    odoPointIdx_ = pointIdx;
    odoCaptureMs_ = nowMs();
    srv_.sendTo("CCTV", makeMsg("CALIB_CAPTURE",
        {{"ch", calibCh_}, {"request_id", calibReqId_}, {"point_index", pointIdx},
         {"world_xy_mm", json::array({round1(xy[0]), round1(xy[1])})}}));
    logf("[INFO] CALIB_CAPTURE 전송 [채널 %d] point_index=%d world=(%.1f,%.1f)mm",
         calibCh_, pointIdx, xy[0], xy[1]);
}

// CCTV의 CALIB_CAPTURE_OK/CALIB_CAPTURE_FAIL 수신 (router.cpp의 fromCctv()가
// type을 보고 ok/reason만 채워 위임한다).
void Router::onCalibCaptureAck(const json& payload, bool ok, const std::string& reason) {
    if (!calibActive_ || !calibIsOdo_) {
        logf("[WARN] CALIB_CAPTURE_%s 수신 - 진행 중인 오도메트리 세션 없음 (무시)",
             ok ? "OK" : "FAIL");
        return;
    }
    const int ch = channelOf(payload);
    const std::string reqId = payload.value("request_id", "");
    const int pointIdx = payload.value("point_index", -1);
    // 매칭 키 3종 전부 일치해야 한다 (wire 스펙 §2-1) - 세션 재시작 시
    // point_index가 다시 0부터라 request_id만으로는 이전 세션의 늦은 ack가
    // 새 세션의 같은 인덱스 대기를 잘못 풀 수 있다.
    if (ch != calibCh_ || reqId != calibReqId_ || pointIdx != odoPointIdx_) {
        logf("[WARN] CALIB_CAPTURE_%s 매칭 실패 (ch %d/%d, reqId \"%s\"/\"%s\", "
             "point %d/%d) - 무시", ok ? "OK" : "FAIL", ch, calibCh_,
             reqId.c_str(), calibReqId_.c_str(), pointIdx, odoPointIdx_);
        return;
    }
    odoPointIdx_ = -1;  // 대기 해제 (checkOdoCaptureTimeout이 더 이상 안 본다)

    if (!ok) {
        logf("[WARN] CALIB_CAPTURE_FAIL [채널 %d] point_index=%d reason=%s",
             calibCh_, pointIdx, reason.c_str());
        // 세션 수준 실패는 계속 진행해봐야 의미가 없다 - 즉시 중단.
        // 지점 수준 실패(marker_not_found/not_settled/unmappable)는 그 점만
        // 버리고 계속 진행한다 (wire 스펙 §7, CCTV 2차 §3-3 확정).
        if (reason == "no_intrinsics" || reason == "session_conflict") {
            abortOdoCalib(reason.c_str(),
                "카메라가 세션 수준 실패를 보고했습니다: " + reason);
            return;
        }
        // 지점 수준 실패 - odoValidCount_ 증가 없이 다음 op으로 진행.
    } else {
        double u = 0.0, v = 0.0;
        if (payload.contains("pixel_uv") && payload["pixel_uv"].is_array() &&
            payload["pixel_uv"].size() == 2 && payload["pixel_uv"][0].is_number() &&
            payload["pixel_uv"][1].is_number()) {
            u = payload["pixel_uv"][0].get<double>();
            v = payload["pixel_uv"][1].get<double>();
        }
        if (pointIdx >= 0 && pointIdx <= 7) ++odoValidCount_;  // idx 8은 진단 전용
        if (pointIdx == 0) {
            odoHaveFirstPix_ = true;
            odoFirstPixU_ = u;
            odoFirstPixV_ = v;
        } else if (pointIdx == 8 && odoHaveFirstPix_) {
            // 폐합오차 - raw 픽셀 단위로만 로깅한다. mm 환산은 불가능하다(이
            // 시점엔 새 캘리가 아직 없다) - mm 값은 카메라 기록이 정본이다
            // (wire 스펙 §5).
            const double du = u - odoFirstPixU_, dv = v - odoFirstPixV_;
            logf("[INFO] 오도메트리 폐합오차 [채널 %d] %.1fpx "
                 "(idx0=(%.1f,%.1f) idx8=(%.1f,%.1f))",
                 calibCh_, std::sqrt(du * du + dv * dv),
                 odoFirstPixU_, odoFirstPixV_, u, v);
        }
        logf("[INFO] CALIB_CAPTURE_OK [채널 %d] point_index=%d pixel=(%.1f,%.1f) "
             "spread=%.2fpx", calibCh_, pointIdx, u, v,
             payload.value("spread_px", 0.0));
    }

    if (odoPendingGoOp_ == -1) {
        // PATH_DONE 트리거였다 - 9번째(복귀) 캡처까지 끝났으니 세션을 닫는다.
        // 유효 대응점 개수(6개 하한 등) 판정은 카메라 몫이다 (wire 스펙 §7) -
        // 서버는 CALIB_DONE만 보내고 그 결과(H_MATRIX 또는 CALIB_FAIL)를 여기서
        // 기다리지 않는다. 서버 관점의 세션은 여기서 끝난다.
        srv_.sendTo("CCTV", makeMsg("CALIB_DONE",
            {{"ch", calibCh_}, {"request_id", calibReqId_},
             {"m_mm", odoMmm_}, {"n_mm", odoNmm_}}));
        logf("[INFO] 오도메트리 캘리 세션 종료 [채널 %d] 유효점 %d/8 - CALIB_DONE 전송",
             calibCh_, odoValidCount_);
        clearCalib();
        clearPath();
    } else {
        sendGo(odoPendingGoOp_, "캘리 캡처 완료");
    }
}

// sweep()에서 매 tick(기본 200ms) 호출. 캡처 ack가 calib_capture_timeout_ms
// 안에 안 오면 세션을 안전하게 접는다 - 카메라가 침묵하는 상황에서 3m 주행을
// 계속 돌리는 건 의미가 없다 (건너뛰지 않는다, wire 스펙 §8).
void Router::checkOdoCaptureTimeout() {
    if (!calibActive_ || !calibIsOdo_ || odoPointIdx_ < 0) return;
    if (calibCancelling_) return;  // 이미 안전 정지 절차가 도는 중
    if (nowMs() - odoCaptureMs_ < params().calib_capture_timeout_ms) return;
    abortOdoCalib("capture_timeout",
        "CALIB_CAPTURE(point_index=" + std::to_string(odoPointIdx_) +
            ") ack를 " + std::to_string(params().calib_capture_timeout_ms / 1000) +
            "초 안에 받지 못했습니다.");
}

// 안전 정지. 정적 앵커 방식의 cancelCalib()/CALIB_CANCEL 중계와 다른 경로를
// 탄다 - 로봇이 실제로 주행 중인데 로봇 펌웨어에는 CALIB_* 핸들러가 없어
// CALIB_CANCEL을 보내도 로봇이 무시하고 계속 굴러간다
// (Paint_Robot/RaspberryPi/ 검색 0건, 계획서 §7).
//
// 로봇 쪽은 fire-and-forget이다: ABORT_DRAW는 로봇(main.cpp:77)에서 즉시
// 동기 처리되고(그 자리에서 속도 0을 내림) 서버로 돌아오는 ack가 없다.
// abortDraw()는 쓰지 않는다 - 그건 "도색" 상태(planActive_ 등)를 정리하는
// 함수라 지금(activePhase_=="calib") 상태와 안 맞는다. 로봇 정지는 CMD 중계
// 한 줄로 끝난다.
//
// CCTV 쪽만 CALIB_CANCEL/CALIB_STOPPED 핸드셰이크를 그대로 쓴다 - 기존
// calibCancelling_/cancelAckRobot_/cancelAckCctv_ 상태 기계를 재사용하되
// cancelAckRobot_을 미리 true로 채워 "로봇 확인 대기"를 건너뛴다. 이러면
// onCalibStopped("CCTV")와 checkCalibTimeout()의 calibCancelling_ 분기가
// 코드 변경 없이 그대로 동작한다 (완료 시 clearPath()도 wasOdo_ 가드로
// 이미 처리돼 있다 - router_calib.cpp).
void Router::abortOdoCalib(const char* reason, const std::string& msg) {
    if (!calibActive_ || !calibIsOdo_) return;
    if (calibCancelling_) return;  // 이미 정지 절차 진행 중 - 중복 트리거 방지
    srv_.sendTo("ROBOT", makeMsg("CMD", {{"cmd", "ABORT_DRAW"}}));
    calibCancelling_ = true;
    calibCancelMs_ = nowMs();
    cancelAckRobot_ = true;   // 로봇 정지는 위 한 줄로 이미 끝났다 (동기 처리, ack 없음)
    cancelAckCctv_ = false;
    srv_.sendTo("CCTV", makeMsg("CMD",
        {{"cmd", "CALIB_CANCEL"}, {"ch", calibCh_}, {"request_id", calibReqId_}}));
    logf("[WARN] 오도메트리 캘리 중단 [채널 %d] reason=%s - %s "
         "(ROBOT 즉시 정지, CCTV CALIB_STOPPED 대기 %ldms)",
         calibCh_, reason, msg.c_str(), params().calib_cancel_ack_ms);
}
