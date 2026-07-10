// 서버 ↔ 로봇 메시지 정의 (ICD 부록 C).
// 개발 단계는 JSON 한 줄(\n 종단)로 주고받고, 나중에 바이너리로 바꿔도 필드 의미는 동일.
#pragma once
#include "geometry.hpp"
#include <string>
#include <sstream>

// ── 하향: 서버 → 로봇 ──
// POSE : 10~15Hz 상시 (로봇 경로추종 입력)
inline std::string makePose(const Pose& p, long seq) {
    std::ostringstream o;
    o << "{\"type\":\"POSE\",\"seq\":" << seq
      << ",\"x\":" << p.x << ",\"y\":" << p.y << ",\"th\":" << p.theta
      << ",\"t\":" << p.t << ",\"conf\":" << p.conf << "}";
    return o.str();
}

// PATH : 작업 시작 시 1회 (웨이포인트 배열)
inline std::string makePath(const std::vector<Waypoint>& wp, long seq) {
    std::ostringstream o;
    o << "{\"type\":\"PATH\",\"seq\":" << seq << ",\"wp\":[";
    for (size_t i = 0; i < wp.size(); ++i) {
        o << "[" << wp[i].x << "," << wp[i].y << "," << (wp[i].paint ? 1 : 0) << "]";
        if (i + 1 < wp.size()) o << ",";
    }
    o << "]}";
    return o.str();
}

// ── 상향: 로봇 → 서버 ──
// STATUS : 10Hz (스텝카운트·상태·배터리·장애물)
struct Status {
    long   seq = 0;
    unsigned left_steps = 0, right_steps = 0;
    int    state = 0;         // 0 IDLE / 1 RUN / 2 ESTOP
    bool   stall = false;
    int    batt_mV = 0;
    int    obstacle_mm = -1;
};
