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
        srv_.sendTo("ROBOT", msg);
        bool toCctv = (cmd == "CALIB_START");
        if (toCctv) srv_.sendTo("CCTV", msg);
        // 수동 조작(조이스틱: 누르는 동안 이동, STOP=뗌)에 진입하면 자동 경로추종을
        // 중단한다. 안 그러면 사용자가 로봇을 수동으로 움직이는 순간 서버가 '경로 이탈'로
        // 판단해 재계획 PATH를 쏴서 수동 조작과 충돌한다. (자동 복귀는 새 BLUEPRINT 때)
        bool manualCmd = (cmd == "FORWARD" || cmd == "BACKWARD" ||
                          cmd == "TURN_LEFT" || cmd == "TURN_RIGHT" || cmd == "STOP");
        if (manualCmd) {
            manualMode_ = true;
            planActive_ = false;  // 진행 중이던 자동 경로 폐기 (수동이 우선)
        }
        logf("[INFO] CMD %s -> ROBOT%s%s", cmd.c_str(), toCctv ? " + CCTV" : "",
             manualCmd ? " [수동모드]" : "");
    } else if (type == "BLUEPRINT") {
        // points = Qt가 top-view 픽셀 -> 바닥 미터 변환을 마친 좌표.
        // (top-view 위에 그린 점은 정의상 바닥 평면 위라 높이 보정 불필요)
        blueprint_ = payload;
        planPts_.clear();
        planActive_ = false;
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
            tryPlanAndSend();  // 도면이 먼저 와 있었으면 이제 경로 전송
            return;
        }
        // ----- 이탈 감시: 계획 경로에서 임계값 이상 벗어나면 재계획 -----
        double dev = distToPolyline({pose_.x, pose_.y}, planPts_);
        if (dev > kDevThresholdM && nowMs() - lastPlanMs_ > kReplanCooldownMs) {
            size_t k = nearestVertex({pose_.x, pose_.y}, planPts_);
            std::vector<Pt> rest(planPts_.begin() + k, planPts_.end());
            json segs = buildSegments(pose_, rest);
            if (!segs.empty() && srv_.sendTo("ROBOT", makePathMsg(segs))) {
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

void Router::tryPlanAndSend() {
    if (planPts_.size() < 2) return;  // 도면 없음
    if (!poseValid_) {
        logf("[INFO] 로봇 위치 미확인 - CCTV POS 수신 후 PATH 전송 예정");
        return;
    }
    json segs = buildSegments(pose_, planPts_);
    if (!segs.empty() && srv_.sendTo("ROBOT", makePathMsg(segs))) {
        planActive_ = true;
        lastPlanMs_ = nowMs();
        logf("[INFO] 경로 생성 완료 - PATH 전송 (%zu 세그먼트)", segs.size());
    }
}
