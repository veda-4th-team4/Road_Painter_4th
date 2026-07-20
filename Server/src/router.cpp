#include "router.hpp"
#include "log.hpp"
#include <chrono>

static long nowMs() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch())
        .count();
}

void Router::onMessage(const std::string& role, const json& msg) {
    if (role == "QT") fromQt(msg);
    else if (role == "ROBOT") fromRobot(msg);
    else if (role == "CCTV") fromCctv(msg);
}

void Router::fromQt(const json& msg) {
    std::string type = msg.value("type", "");
    json payload = msg.value("payload", json::object());

    if (type == "REGISTER") {
        std::string id = payload.value("id", ""), err;
        bool ok = users_.registerUser(id, payload.value("pw", ""), err);
        srv_.sendTo("QT", makeMsg(ok ? "REGISTER_OK" : "REGISTER_FAIL",
                                  ok ? json{{"id", id}} : json{{"reason", err}}));
        logf("[INFO] REGISTER %s: %s", id.c_str(), ok ? "성공" : err.c_str());
    } else if (type == "LOGIN") {
        std::string id = payload.value("id", "");
        if (users_.login(id, payload.value("pw", ""))) {
            currentUser_ = id;
            json stored = users_.getCalib(id);
            Calib c;
            if (!stored.is_null() && calibFromJson(stored, c))
                calib_ = c;  // 저장된 캘리브레이션을 현재 세션에 복원
            srv_.sendTo("QT", makeMsg("LOGIN_OK", {{"id", id}, {"calib", stored}}));
            logf("[INFO] LOGIN %s 성공 (캘리브레이션 %s)", id.c_str(),
                 stored.is_null() ? "없음 - 캘리브레이션 필요" : "전달");
        } else {
            srv_.sendTo("QT", makeMsg("LOGIN_FAIL",
                                      {{"reason", "id 또는 비밀번호 불일치"}}));
            logf("[WARN] LOGIN %s 실패", id.c_str());
        }
    } else if (type == "CMD") {
        std::string cmd = payload.value("cmd", "");
        // START_DRAW: Qt "그림그리기 시작" 버튼. 로봇에 중계하지 않고 서버가
        // 2단계(도색) 경로를 만들어 PATH로 전송한다 (로봇은 PATH 수신 = 시작 신호).
        if (cmd == "START_DRAW") {
            sendDrawPath();
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
        srv_.sendTo("ROBOT", msg);
        bool toCctv = (cmd == "CALIB_START");
        if (toCctv) srv_.sendTo("CCTV", msg);
        // 여기 오는 수동 CMD는 경로가 없는(planActive_==false) 상태뿐이다.
        // 수동 조작에 진입하면 자동 경로추종/재계획을 멈춰 충돌을 막는다.
        // (자동 복귀는 새 BLUEPRINT 수신 시)
        if (manualCmd) manualMode_ = true;
        logf("[INFO] CMD %s -> ROBOT%s%s", cmd.c_str(), toCctv ? " + CCTV" : "",
             manualCmd ? " [수동모드]" : "");
    } else if (type == "BLUEPRINT") {
        // points = Qt가 top-view 픽셀 -> 바닥 미터 변환을 마친 좌표.
        // (top-view 위에 그린 점은 정의상 바닥 평면 위라 높이 보정 불필요)
        blueprint_ = payload;
        planPts_.clear();
        planActive_ = false;
        awaitingStart_ = false;
        manualMode_ = false;  // 새 도면 = 자동 모드 복귀
        for (auto& p : payload.value("points", json::array()))
            planPts_.push_back({p[0].get<double>(), p[1].get<double>()});
        if (planPts_.size() < 2) {
            logf("[WARN] 도면에 points가 부족함 (%zu개) - 경로 생성 불가",
                 planPts_.size());
            return;
        }
        logf("[INFO] 도면 수신 (%zu점) - 경로 생성 시도", planPts_.size());
        tryPlanAndSend();
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
        // CCTV pose의 실제 각도 vs 해당 세그먼트의 목표 heading 비교:
        //   오차 > 임계값 -> ALIGN(그만큼 미세 회전 후 다시 READY)
        //   오차 <= 임계값 or 반복 초과 -> GO(직진 시작 / 접근 마지막 TURN이면 대기)
        int seg = payload.value("seg", -1);
        if (seg != alignSegIdx_) {  // 새 세그먼트 정렬 시작
            alignSegIdx_ = seg;
            alignTries_ = 0;
        }
        // heading_deg가 있는 세그먼트(MOVE 전부 + 접근 경로 마지막 TURN)만 판정 가능
        double target = 1e9;
        if (planActive_ && !manualMode_ && seg >= 0 &&
            seg < (int)activeSegs_.size())
            target = activeSegs_[seg].value("heading_deg", 1e9);

        if (target > 1e8 || !poseValid_) {
            // 판정 불가(계획 없음/수동모드/seg 이상/pose 없음)면 로봇을 세워두지 않는다
            srv_.sendTo("ROBOT", makeMsg("GO", json::object()));
            logf("[WARN] READY(seg=%d) - 정렬 판정 불가, 그냥 GO", seg);
            return;
        }
        double err = normDeg(target - pose_.theta * 180.0 / M_PI);
        if (std::fabs(err) > kAlignThresholdDeg && alignTries_ < kAlignMaxTries) {
            ++alignTries_;
            srv_.sendTo("ROBOT",
                        makeMsg("ALIGN", {{"angle_deg", std::round(err * 10) / 10}}));
            logf("[INFO] READY(seg=%d) 각도오차 %.1f도 - ALIGN 전송 (%d/%d회)",
                 seg, err, alignTries_, kAlignMaxTries);
        } else {
            srv_.sendTo("ROBOT", makeMsg("GO", json::object()));
            logf("[INFO] READY(seg=%d) 오차 %.1f도 - GO%s", seg, err,
                 alignTries_ >= kAlignMaxTries ? " (ALIGN 반복 초과)" : "");
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
        lastPos_ = payload;
        srv_.sendTo("ROBOT", msg);  // 위치 보정용 (원본 그대로)
        srv_.sendTo("QT", msg);     // 모니터링용 (원본 그대로)

        Pose p;
        if (poseFromPos(payload, calib_, p)) {
            pose_ = p;
            poseValid_ = true;
            // 계산된 pose(바닥 미터 좌표)를 QT에 전송 - top-view 위 로봇 표시용
            srv_.sendTo("QT", makeMsg("POSE",
                {{"x", std::round(p.x * 1000) / 1000},
                 {"y", std::round(p.y * 1000) / 1000},
                 {"theta_deg", std::round(p.theta * 180.0 / M_PI * 10) / 10}}));
        } else {
            if (!calib_.valid)
                logf("[WARN] POS 수신했으나 캘리브레이션 없음 - pose 계산 불가 (중계만 함)");
            return;  // pose를 못 구하면 중계만 하고 끝
        }

        // 수동 조작 중엔 자동 경로/재계획을 하지 않는다 (POSE 모니터링은 위에서 이미 전송).
        if (manualMode_) return;
        if (!planActive_) {
            tryPlanAndSend();  // 도면이 먼저 와 있었으면 이제 1단계(접근) 경로 전송
            return;
        }

        // ----- 주행 중 각도 피드백(DRIFT): 실행 중인 세그먼트의 목표 방향 대비 -----
        // 얼마나 틀어졌는지 지속 전송. 부호: 가려는 방향이 0도 기준,
        // 시계방향(오른쪽)으로 틀어져 있으면 양수, 반시계(왼쪽)면 음수.
        // (값 자체가 "좌회전으로 보정해야 할 양"과 같음 - ALIGN과 동일 규약)
        if (alignSegIdx_ >= 0 && alignSegIdx_ < (int)activeSegs_.size() &&
            nowMs() - lastDriftMs_ >= kDriftPeriodMs) {
            double target = activeSegs_[alignSegIdx_].value("heading_deg", 1e9);
            if (target < 1e8) {
                double drift = normDeg(target - pose_.theta * 180.0 / M_PI);
                srv_.sendTo("ROBOT", makeMsg("DRIFT",
                    {{"angle_deg", std::round(drift * 10) / 10}}));
                lastDriftMs_ = nowMs();
            }
        }

        // 1단계(접근) 중엔 이탈 재계획을 하지 않는다 - 로봇이 도면 폴리라인에서
        // 떨어져 있는 게 정상이라 오탐이 나기 때문. (READY/ALIGN + DRIFT로 충분)
        if (awaitingStart_) return;

        // ----- 이탈 감시: 계획 경로에서 임계값 이상 벗어나면 재계획 -----
        double dev = distToPolyline({pose_.x, pose_.y}, planPts_);
        if (dev > kDevThresholdM && nowMs() - lastPlanMs_ > kReplanCooldownMs) {
            size_t k = nearestVertex({pose_.x, pose_.y}, planPts_);
            std::vector<Pt> rest(planPts_.begin() + k, planPts_.end());
            json segs = buildSegments(pose_, rest);
            if (!segs.empty() && srv_.sendTo("ROBOT", makePathMsg(segs, "draw"))) {
                activeSegs_ = segs;
                alignSegIdx_ = -1, alignTries_ = 0;  // 새 경로 = 정렬 상태 리셋
                lastPlanMs_ = nowMs();
                logf("[WARN] 경로 이탈 %.2fm - 재계획 PATH 전송 (%zu번 꼭짓점부터, %zu 세그먼트)",
                     dev, k, segs.size());
            }
        }
    } else if (type == "H_MATRIX") {
        // 신규: payload.calib = {K, D, H_floor, H_marker, marker_height_m, version}
        // 레거시: payload.H = [[...]x3] (왜곡 보정 없이 바닥/마커 공용으로 사용)
        json bundle =
            payload.contains("calib") ? payload["calib"] : payload.value("H", json());
        Calib c;
        if (!calibFromJson(bundle, c)) {
            logf("[WARN] H_MATRIX 파싱 실패 - calib/H 형식 확인 필요: %s",
                 payload.dump().c_str());
            return;
        }
        calib_ = c;
        srv_.sendTo("QT", msg);  // Qt도 즉시 새 캘리브레이션으로 top-view 갱신
        if (!currentUser_.empty() && users_.setCalib(currentUser_, bundle))
            logf("[INFO] 캘리브레이션 수신 (%s%s) - 사용자 '%s'에 영속 저장",
                 c.hasMarker ? "H_marker 포함" : "H_marker 없음 - 시차 보정 생략됨",
                 c.hasKD ? ", K/D 포함" : ", K/D 없음 - 왜곡 보정 생략됨",
                 currentUser_.c_str());
        else
            logf("[WARN] 캘리브레이션 수신 - 로그인 사용자 없음, 세션에만 유지");
    } else {
        logf("[WARN] CCTV로부터 알 수 없는 type: %s", type.c_str());
    }
}

// 1단계: 로봇 현재 위치 -> 도면 시작점(planPts_[0])까지 접근 경로.
// 시작점 도착 후 첫 도색 방향으로 미리 회전까지 시켜두고 로봇은 대기,
// 이후 Qt의 START_DRAW를 기다린다.
void Router::tryPlanAndSend() {
    if (planPts_.size() < 2) return;  // 도면 없음
    if (!poseValid_) {
        logf("[INFO] 로봇 위치 미확인 - CCTV POS 수신 후 PATH 전송 예정");
        return;
    }
    json segs = buildSegments(pose_, {planPts_[0]});  // 시작점까지 (paint=false)

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

    if (!segs.empty() && srv_.sendTo("ROBOT", makePathMsg(segs, "approach"))) {
        planActive_ = true;
        awaitingStart_ = true;
        activeSegs_ = segs;
        alignSegIdx_ = -1, alignTries_ = 0;  // 새 경로 = 정렬 상태 리셋
        lastPlanMs_ = nowMs();
        logf("[INFO] 1단계 접근 경로 전송 (%zu 세그먼트) - START_DRAW 대기",
             segs.size());
    }
}

// 2단계: Qt "그림그리기 시작" -> 시작점에 서 있는 로봇에게 도색 경로 전송.
// 로봇은 이 PATH(phase=draw)를 받으면 IMU 현재 방향을 0도로 세팅하고 주행 시작.
void Router::sendDrawPath() {
    if (!awaitingStart_) {
        logf("[WARN] START_DRAW 수신 - 접근 완료 대기 상태가 아님 (무시)");
        return;
    }
    if (!poseValid_ || planPts_.size() < 2) {
        logf("[WARN] START_DRAW 수신 - pose/도면 없음 (무시)");
        return;
    }
    // 시작점에 서 있으므로 남은 경로 = 두번째 점부터. 모든 MOVE가 도색 구간.
    std::vector<Pt> rest(planPts_.begin() + 1, planPts_.end());
    json segs = buildSegments(pose_, rest, /*firstPaint=*/true);
    if (!segs.empty() && srv_.sendTo("ROBOT", makePathMsg(segs, "draw"))) {
        awaitingStart_ = false;
        planActive_ = true;
        activeSegs_ = segs;
        alignSegIdx_ = -1, alignTries_ = 0;
        lastPlanMs_ = nowMs();
        logf("[INFO] 2단계 도색 경로 전송 (%zu 세그먼트) - 도색 시작", segs.size());
    }
}
