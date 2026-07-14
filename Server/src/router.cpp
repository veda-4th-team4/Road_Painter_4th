#include "router.hpp"
#include "log.hpp"

void Router::onMessage(const std::string& role, const json& msg) {
    if (role == "QT") fromQt(msg);
    else if (role == "ROBOT") fromRobot(msg);
    else if (role == "CCTV") fromCctv(msg);
}

void Router::fromQt(const json& msg) {
    std::string type = msg.value("type", "");
    json payload = msg.value("payload", json::object());

    if (type == "CMD") {
        std::string cmd = payload.value("cmd", "");
        srv_.sendTo("ROBOT", msg);
        bool toCctv = (cmd == "CALIB_START");
        if (toCctv) srv_.sendTo("CCTV", msg);
        logf("[INFO] CMD %s -> ROBOT%s", cmd.c_str(), toCctv ? " + CCTV" : "");
    } else if (type == "BLUEPRINT") {
        blueprint_ = payload;
        logf("[INFO] 도면 수신 (Qt) - 저장 완료");
        // TODO: 도면 -> 경로 생성 후 srv_.sendTo("ROBOT", makeMsg("PATH", ...))
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
        logf("[INFO] STATUS: state=%s x=%s y=%s bat=%s",
             payload.value("state", "?").c_str(),
             payload.contains("x") ? payload["x"].dump().c_str() : "?",
             payload.contains("y") ? payload["y"].dump().c_str() : "?",
             payload.contains("battery") ? payload["battery"].dump().c_str() : "?");
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
    } else if (type == "H_MATRIX") {
        hMatrix_ = payload.value("H", json());
        logf("[INFO] H 행렬 수신 - 저장 완료");
    } else {
        logf("[WARN] CCTV로부터 알 수 없는 type: %s", type.c_str());
    }
}
