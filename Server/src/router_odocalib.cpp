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

// 이 세션의 주행 데드라인(CALIB_START -> CALIB_DONE).
//
// ADMIN 개시는 params 값을 그대로 쓴다 - 기다리는 Qt가 없으므로 깎을 이유가
// 없고, 목적도 "로봇/카메라가 죽었을 때 접는 워치독"이다. QT 개시는 다르다:
// Qt가 5분 뒤 스스로 대기를 푸는데 서버가 그때까지 세션을 쥐고 있으면, Qt는
// 이미 포기했는데 서버만 busy로 남아 다음 요청이 전부 거절된다. 게다가 주행이
// 끝난 뒤 카메라 계산 시간(calib_odo_result_wait_ms)도 같은 예산 안에서 나가야
// 하므로, 주행 몫은 그만큼 더 줄어든다.
long Router::odoDriveBudgetMs() const {
    const long p = params().calib_odo_timeout_ms;
    if (calibOwner_ != CalibOwner::QT) return p;
    const long cap = kQtOdoWaitCapMs - params().calib_odo_result_wait_ms;
    return p < cap ? p : cap;
}

// 주행 진행률을 QT에 합성해 보낸다 (wire 스펙에 없는 서버 자체 신호).
//
// 카메라는 이 방식에서 CALIB_PROGRESS를 올리지 않는다 - 정적 앵커 방식과 다른
// 점이다. 그런데 주행이 2~4분이라 그동안 Qt 대기 화면이 통째로 비어 있으면
// 조작자는 멈춘 것으로 본다. 서버는 정지점 진행을 정확히 알고 있으므로
// (point_index 0..8) 그걸로 만들어 보낸다.
//
// valid가 pointIdx를 따라오지 못하면 카메라가 마커를 놓치고 있다는 뜻이다 -
// 유효점이 6개 미만이면 세션이 실패하므로(wire 스펙 §7), Qt가 주행이 다 끝나기
// 전에 조작자에게 경고할 수 있도록 개수를 같이 싣는다.
void Router::sendOdoProgress(const char* phase, int pointIdx) {
    if (!calibToQt()) return;
    srv_.sendTo("QT", makeMsg("CALIB_PROGRESS",
        {{"ch", calibCh_}, {"request_id", calibReqId_}, {"phase", phase},
         {"point_index", pointIdx}, {"total", 9}, {"valid", odoValidCount_}}));
}

// CALIB_START{method:"robot_motion"} 수락. router_calib.cpp의 startCalib()이
// 공통 검증(멱등/busy/도색중/채널/로그인/피어접속)과 치수 검증(m_cm/n_cm/
// start_corner)을 이미 통과시킨 뒤 호출한다 - 그래서 여기서는 다시 보지 않는다.
//
// ⚠️ 2026-08-13부터 QT도 개시자가 될 수 있다. 예전에는 ADMIN 전용이라 실패를
//    알릴 곳이 없어 로그로만 끝냈는데, 이제 남은 실패 경로(PATH 전송 실패)는
//    rejectCalib()으로 QT에도 통지한다.
void Router::startOdoCalib(const json& payload, const json& msg,
                           const std::string& reqId, int ch, bool fromQt) {
    const char* origin = fromQt ? "QT" : "ADMIN";
    const double mCm = payload.value("m_cm", 0.0);
    const double nCm = payload.value("n_cm", 0.0);
    const std::string corner = payload.value("start_corner", "");
    const double mM = mCm / 100.0, nM = nCm / 100.0;
    const bool ccw = (corner == "bottom_left");

    // 채널 전환은 정적 앵커 경로와 동일하게 여기서 한다 - CCTV가 CALIB_CAPTURE를
    // 받기 전에 올바른 렌즈를 보고 있어야 한다.
    applyChannel(ch);
    srv_.sendTo("CCTV", makeMsg("CMD", {{"cmd", "SELECT_CHANNEL"}, {"ch", ch}}));

    // 🔴 CALIB_START 원본을 ROBOT/CCTV 양쪽에 중계한다 (정적 앵커 경로와 동일).
    // 예전에는 "로봇 펌웨어에 CALIB_* 핸들러가 없으니 보낼 필요 없다"고 판단해
    // 생략했는데, 로봇팀이 R-1을 구현하면서(main.cpp "CALIB_START" 분기) 전제가
    // 바뀌었다. 두 가지가 여기에 달려 있다:
    //   ROBOT: R-1이 auto_nozzle=0 + SendControlNozzle(0)으로 노즐을 강제로
    //     올린다. 이게 없으면 직전 도색이 노즐을 내린 채 비정상 종료된 경우
    //     (연결 끊김/HOLD 중 포기) auto_nozzle=1이 그대로 남는데 - PATH 적용은
    //     manual_nozzle만 0으로 되돌리고 auto_nozzle은 안 건드린다 - 캘리 op에
    //     nozzle op이 하나도 없어서 로봇이 사각형을 그리며 바닥을 칠한다.
    //   CCTV: CALIB_START가 카메라 세션의 시작점이다 (CCTV팀 2차 회신 §5-B).
    //     이걸 안 보내면 카메라가 세션 없이 CALIB_CAPTURE를 받는다.
    srv_.sendTo("ROBOT", msg);
    srv_.sendTo("CCTV", msg);

    PlannedPath path = buildCalibRectOps(mM, nM, ccw);
    if (!sendPath(std::move(path), "calib")) {
        rejectCalib(fromQt, ch, reqId, origin, "robot_offline",
                    "로봇에 경로를 전송하지 못했습니다 (연결이 끊겼을 수 있습니다).");
        return;
    }

    calibActive_ = true;
    calibOwner_ = fromQt ? CalibOwner::QT : CalibOwner::ADMIN;
    calibReqId_ = reqId;
    calibCh_ = ch;
    calibStartMs_ = nowMs();
    calibIsOdo_ = true;
    odoAwaitingResult_ = false;
    odoMmm_ = mCm * 10.0;
    odoNmm_ = nCm * 10.0;
    odoCcw_ = ccw;
    odoPointIdx_ = -1;
    odoPendingGoOp_ = -2;
    odoValidCount_ = 0;
    for (int i = 0; i < 9; ++i) odoPixOk_[i] = false;

    // Qt에도 채널을 알린다 - 정적 앵커 경로와 같은 이유다(계약 §5). 서버가
    // 바꾼 활성 채널을 Qt가 모르면 화면과 좌표계가 어긋난다.
    if (fromQt) {
        const Calib& c = activeCalib();
        srv_.sendTo("QT", makeMsg("CHANNEL_OK",
            {{"ch", ch}, {"calib", c.valid ? c.raw : json()}}));
        srv_.sendTo("QT", makeMsg("CALIB_STARTED",
            {{"ch", ch}, {"request_id", reqId},
             {"msg", "CH" + std::to_string(ch) + " 주행 캘리브레이션을 " +
                         "시작했습니다. 로봇이 " + std::to_string((int)mCm) +
                         "x" + std::to_string((int)nCm) +
                         "cm 사각형을 그립니다 - 로봇 주변에서 비켜주세요."}}));
        sendOdoProgress("driving", -1);  // 대기 화면에 0/9를 즉시 띄운다
    }

    logf("[INFO] (%s) CALIB_START(robot_motion) 수락 [채널 %d] m=%.0fmm n=%.0fmm "
         "start_corner=%s (%s) -> ROBOT+CCTV 중계 + PATH(calib) 11-op 전송, "
         "첫 READY부터 캡처 시작 (주행 데드라인 %ldms%s)",
         origin, ch, odoMmm_, odoNmm_, corner.c_str(), ccw ? "CCW" : "CW",
         odoDriveBudgetMs(),
         odoDriveBudgetMs() < params().calib_odo_timeout_ms
             ? ", QT 개시라 Qt 대기 예산에 맞춰 깎임" : "");
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
        //
        // detect_off/session_refused/store_failed 는 카메라가 실제로 보내는데
        // 스펙 §7 표에는 없던 사유다(2026-08-12 추가). 셋 다 "다음 점에서도
        // 똑같이 실패할 게 확정"인 상태라 지점 실패로 볼 이유가 없다 - 그냥
        // 두면 로봇이 9개 정지점을 다 돌고 나서야 too_few_points 로 끝난다.
        //
        // 목록에 없는 사유를 통째로 세션 중단으로 돌리지는 않는다. 카메라가
        // 나중에 지점 수준 사유를 늘렸을 때 멀쩡한 세션이 죽는다.
        if (reason == "no_intrinsics" || reason == "session_conflict" ||
            reason == "detect_off" || reason == "session_refused" ||
            reason == "store_failed") {
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
        if (pointIdx >= 0 && pointIdx <= 8) {
            odoPixU_[pointIdx] = u;
            odoPixV_[pointIdx] = v;
            odoPixOk_[pointIdx] = true;
        }
        if (pointIdx == 8) logOdoClosure();
        logf("[INFO] CALIB_CAPTURE_OK [채널 %d] point_index=%d pixel=(%.1f,%.1f) "
             "spread=%.2fpx", calibCh_, pointIdx, u, v,
             payload.value("spread_px", 0.0));
    }

    if (odoPendingGoOp_ == -1) {
        // PATH_DONE 트리거였다 - 9번째(복귀) 캡처까지 끝났다. 유효 대응점
        // 개수(6개 하한 등) 판정은 카메라 몫이다 (wire 스펙 §7).
        srv_.sendTo("CCTV", makeMsg("CALIB_DONE",
            {{"ch", calibCh_}, {"request_id", calibReqId_},
             {"m_mm", odoMmm_}, {"n_mm", odoNmm_}}));
        // 로봇 몫은 여기서 끝난다 - 경로 상태는 어느 쪽이든 비운다.
        clearPath();
        if (calibToQt()) {
            // 🔴 QT 개시면 세션을 닫지 않는다. Qt의 대기 화면은 종결 응답
            //   (H_MATRIX|CALIB_FAIL|CALIB_CANCELLED)으로만 열리고 닫히는데,
            //   그 둘은 **지금부터** 카메라가 만든다. 여기서 clearCalib()을
            //   부르면 calibActive_가 꺼져서 두 가지가 동시에 깨진다:
            //     - 뒤늦게 오는 H_MATRIX가 "종결"로 안 잡혀 request_id 없이
            //       중계된다 (handleHMatrix의 closesSession 판정)
            //     - 카메라의 CALIB_FAIL{too_few_points|fit_failed}이
            //       "진행 중인 세션 없음"으로 버려진다 (relayCalibFail)
            //   둘 다 Qt를 타임아웃까지 대기 화면에 가둔다.
            odoAwaitingResult_ = true;
            odoResultWaitMs_ = nowMs();
            sendOdoProgress("solving", 8);
            logf("[INFO] 오도메트리 주행 종료 [채널 %d] 유효점 %d/8 - CALIB_DONE "
                 "전송, 카메라 결과 대기 (%ldms 한도)",
                 calibCh_, odoValidCount_, params().calib_odo_result_wait_ms);
        } else {
            // ADMIN 개시는 기다리는 쪽이 없으므로 예전대로 여기서 끝낸다.
            // 뒤에 오는 H_MATRIX는 평소 캘리 갱신과 동일한 경로로 저장된다.
            logf("[INFO] 오도메트리 캘리 세션 종료 [채널 %d] 유효점 %d/8 - "
                 "CALIB_DONE 전송 (ADMIN 개시라 결과를 기다리지 않음)",
                 calibCh_, odoValidCount_);
            clearCalib();
        }
    } else {
        sendOdoProgress("driving", pointIdx);
        sendGo(odoPendingGoOp_, "캘리 캡처 완료");
    }
}

// 폐합오차 로그. idx 0(출발 자리, 아직 한 발도 안 움직인 상태)과 idx 8(사각형을
// 한 바퀴 돌고 돌아온 자리)은 **같은 물리 지점**이므로(odoPointWorldMm의 case 0과
// default가 같은 좌표), 로봇이 정확히 복귀했다면 두 픽셀이 같아야 한다. 차이가
// 곧 한 바퀴 누적된 주행 오차다 - H가 없어도 성립하는 유일한 자기검증이다.
//
// 🔴 진단 전용이다. 이 값으로 세션을 실패시키거나 Qt에 알리지 않는다. 두 가지
//   이유가 있다: (1) 유효점 판정과 H 산출은 카메라 몫이고(wire 스펙 §7) 서버가
//   뒤집으면 권한이 겹친다, (2) 아래 한계 때문에 "작다 = 정확하다"가 성립하지
//   않아 임계값 판정의 근거로 삼기에 약하다.
//
// ⚠️ 이 값의 한계 (로그를 읽는 사람이 알아야 한다)
//   - **위치만 본다.** 제자리에 정확히 돌아왔어도 로봇이 엉뚱한 방향을 보고
//     있을 수 있다. 마커 중심 하나만 비교하므로 자세 오차는 안 잡힌다.
//   - **오차가 상쇄될 수 있다.** 세 코너가 같은 쪽으로 오버슛하면 사각형이 작게
//     닫히면서 오히려 출발점 근처로 돌아온다. 작다고 정확한 게 아니다.
//     반대는 성립한다 - 크면 확실히 틀렸다.
//   - **mm 환산이 거칠다.** 아래 스케일은 네 변의 실측 픽셀 길이를 명령 길이로
//     나눠 평균한 값이라, 카메라가 기울어져 있으면 화면 위치마다 실제 px/mm가
//     다르다. 자릿수를 보는 용도지 정밀 측정이 아니다. 정본은 카메라 기록이다
//     (wire 스펙 §5).
void Router::logOdoClosure() {
    if (!odoPixOk_[0] || !odoPixOk_[8]) {
        logf("[INFO] 오도메트리 폐합오차 [채널 %d] 측정 불가 "
             "(idx0 %s, idx8 %s - 캡처 실패)", calibCh_,
             odoPixOk_[0] ? "OK" : "실패", odoPixOk_[8] ? "OK" : "실패");
        return;
    }
    const double du = odoPixU_[8] - odoPixU_[0], dv = odoPixV_[8] - odoPixV_[0];
    const double closurePx = std::sqrt(du * du + dv * dv);

    // px/mm 스케일: 네 변의 (실측 픽셀 길이 / 명령 mm)를 평균한다. 코너 캡처가
    // 실패한 변은 건너뛴다. 사각형이 조금 일그러져 있어도 스케일 자체는 그
    // 일그러짐만큼만(수 %) 틀리므로 자릿수 판단에는 충분하다.
    struct Side { int a, b; double mm; };
    const Side sides[4] = {{0, 2, odoMmm_}, {2, 4, odoNmm_},
                           {4, 6, odoMmm_}, {6, 0, odoNmm_}};
    double scaleSum = 0.0;
    int scaleN = 0;
    for (const Side& s : sides) {
        if (!odoPixOk_[s.a] || !odoPixOk_[s.b] || s.mm <= 0.0) continue;
        const double sdu = odoPixU_[s.b] - odoPixU_[s.a];
        const double sdv = odoPixV_[s.b] - odoPixV_[s.a];
        scaleSum += std::sqrt(sdu * sdu + sdv * sdv) / s.mm;
        ++scaleN;
    }
    if (scaleN == 0) {
        // 코너 캡처가 부족해 스케일을 못 뽑는다 - 픽셀 값만 남긴다.
        logf("[INFO] 오도메트리 폐합오차 [채널 %d] %.1fpx (mm 환산 불가 - "
             "코너 캡처 부족) idx0=(%.1f,%.1f) idx8=(%.1f,%.1f)",
             calibCh_, closurePx, odoPixU_[0], odoPixV_[0], odoPixU_[8],
             odoPixV_[8]);
        return;
    }
    const double pxPerMm = scaleSum / scaleN;
    const double closureMm = closurePx / pxPerMm;
    const double perimeterMm = 2.0 * (odoMmm_ + odoNmm_);
    const double pct = perimeterMm > 0.0 ? closureMm / perimeterMm * 100.0 : 0.0;

    logf("[INFO] 오도메트리 폐합오차 [채널 %d] %.1fpx = 약 %.0fmm "
         "(둘레 %.0fmm의 %.2f%%) | 스케일 %.3fpx/mm (%d변 평균) | "
         "idx0=(%.1f,%.1f) idx8=(%.1f,%.1f)",
         calibCh_, closurePx, closureMm, perimeterMm, pct, pxPerMm, scaleN,
         odoPixU_[0], odoPixV_[0], odoPixU_[8], odoPixV_[8]);
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

// 안전 정지. 정적 앵커 방식의 cancelCalib()과 **같은** CALIB_CANCEL/CALIB_STOPPED
// 핸드셰이크를 쓴다 - 로봇팀이 R-2를 구현해서(main.cpp "CALIB_CANCEL" 분기)
// 로봇이 속도 0 + 노즐 UP + 경로 폐기를 마친 뒤 100ms 정착 후 CALIB_STOPPED를
// 실제로 회신한다.
//
// 🔴 ABORT_DRAW를 쓰지 않는 이유: 그것도 로봇을 세우기는 하지만 ack가 없어서
//   "명령을 보냈다"까지만 알 수 있다. router_calib.cpp의 규약이 지키려는 것은
//   "로봇이 실제로 섰다는 확인"이다 - 확인 없이 대기를 풀면 조작자가 아직
//   굴러가는 로봇 쪽으로 걸어간다. 받을 수 있는 ack를 일부러 버릴 이유가 없다.
//
// 이 함수가 하는 일은 상태를 세우고 메시지를 보내는 것뿐이고, 그 뒤는 기존
// 상태 기계가 그대로 처리한다:
//   양쪽 ACK 도착 -> onCalibStopped()가 clearCalib()+clearPath() (calibIsOdo_ 가드)
//   ACK 불발      -> checkCalibTimeout()의 calibCancelling_ 분기가 cancel_failed
//
// reason은 onCalibStopped()가 종결 응답을 고르는 데 쓴다: "cancelled"면 순수
// 취소라 CALIB_CANCELLED, 나머지(capture_timeout/timeout/preempted/...)는
// 실패라 CALIB_FAIL{reason}이 QT에 간다.
void Router::abortOdoCalib(const char* reason, const std::string& msg) {
    if (!calibActive_ || !calibIsOdo_) return;
    if (calibCancelling_) return;  // 이미 정지 절차 진행 중 - 중복 트리거 방지
    // 실패 사유를 들고 있다가 onCalibStopped()에서 종결 응답을 가른다.
    // "cancelled"만 비워둔다 - 조작자가 직접 누른 취소다.
    if (std::string(reason) != "cancelled") {
        calibAbortReason_ = reason;
        calibAbortMsg_ = msg;
    }
    // 주행이 이미 끝난 뒤(CALIB_DONE 이후)라면 세울 로봇이 없다. 그대로 정지
    // 핸드셰이크를 돌리면 로봇/카메라의 CALIB_STOPPED를 5초 기다리다
    // cancel_failed로 한 번 더 늘어지고, Qt에는 "정지 확인 실패, 로봇 상태를
    // 직접 확인하세요"라는 엉뚱한 경고가 뜬다 - 로봇은 멀쩡히 서 있는데.
    if (odoAwaitingResult_) {
        logf("[WARN] 오도메트리 캘리 중단 [채널 %d] reason=%s - %s "
             "(주행은 이미 끝나 정지 핸드셰이크 생략)", calibCh_, reason,
             msg.c_str());
        if (calibAbortReason_.empty()) {
            // 결과 대기 중의 순수 취소 - 로봇을 세울 것도 없으니 바로 닫는다.
            const int ch = calibCh_;
            const std::string reqId = calibReqId_;
            const bool toQt = calibToQt();
            clearCalib();
            if (toQt)
                srv_.sendTo("QT", makeMsg("CALIB_CANCELLED",
                    {{"ch", ch}, {"request_id", reqId},
                     {"msg", "호모그래피 작업을 중단했습니다."}}));
        } else {
            failCalib(reason, msg);
        }
        return;
    }
    const json cancelMsg = makeMsg("CMD",
        {{"cmd", "CALIB_CANCEL"}, {"ch", calibCh_}, {"request_id", calibReqId_}});
    calibCancelling_ = true;
    calibCancelMs_ = nowMs();
    cancelAckRobot_ = cancelAckCctv_ = false;
    srv_.sendTo("ROBOT", cancelMsg);
    srv_.sendTo("CCTV", cancelMsg);
    logf("[WARN] 오도메트리 캘리 중단 [채널 %d] reason=%s - %s "
         "(ROBOT+CCTV CALIB_STOPPED 대기 %ldms)",
         calibCh_, reason, msg.c_str(), params().calib_cancel_ack_ms);
}
