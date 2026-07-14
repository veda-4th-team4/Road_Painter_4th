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
            json H = users_.getH(id);
            if (!H.is_null()) hMatrix_ = H;  // 저장된 H를 현재 세션에 복원
            srv_.sendTo("QT", makeMsg("LOGIN_OK", {{"id", id}, {"H", H}}));
            logf("[INFO] LOGIN %s 성공 (H행렬 %s)", id.c_str(),
                 H.is_null() ? "없음 - 캘리브레이션 필요" : "전달");
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
        logf("[INFO] CMD %s -> ROBOT%s", cmd.c_str(), toCctv ? " + CCTV" : "");
    } else if (type == "BLUEPRINT") {
        blueprint_ = payload;
        planPts_.clear();
        planActive_ = false;
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
        lastPos_ = payload;
        srv_.sendTo("ROBOT", msg);  // 위치 보정용
        srv_.sendTo("QT", msg);     // 모니터링용

        Pose p;
        if (poseFromPos(payload, hMatrix_, p)) {
            pose_ = p;
            poseValid_ = true;
        } else {
            return;  // pose를 못 구하면 중계만 하고 끝
        }

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
        hMatrix_ = payload.value("H", json());
        srv_.sendTo("QT", msg);  // Qt도 즉시 새 H로 top-view 갱신
        if (!currentUser_.empty() && users_.setH(currentUser_, hMatrix_))
            logf("[INFO] H 행렬 수신 - 사용자 '%s'에 영속 저장", currentUser_.c_str());
        else
            logf("[WARN] H 행렬 수신 - 로그인 사용자 없음, 세션에만 유지");
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
