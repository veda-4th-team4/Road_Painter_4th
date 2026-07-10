// 공용 자료구조. 좌표계: 지면평면, 단위 m / rad (CCTV-서버 H가 정의한 좌표계).
#pragma once
#include <cmath>
#include <vector>

struct Point2 { double x = 0, y = 0; };

struct Pose {              // 로봇/마커 현재 상태 (POSE 메시지 본체)
    double x = 0, y = 0;   // 펜 끝 지면 좌표 [m]
    double theta = 0;      // 헤딩 [rad]
    double t = 0;          // 카메라 프레임 타임스탬프 [epoch s]
    double conf = 0;       // 검출 신뢰도 (0 = 실패)
};

struct Waypoint {
    double x = 0, y = 0;
    bool   paint = false;  // 이 지점 이동 중 노즐 on/off
};

struct Segment {           // 경로생성 입력: 도색할 라인 한 구간
    Point2 p0, p1;
    bool   paint = true;
};

inline double dist(double ax, double ay, double bx, double by) {
    return std::hypot(ax - bx, ay - by);
}
inline double normAngle(double a) {
    while (a >  M_PI) a -= 2 * M_PI;
    while (a <= -M_PI) a += 2 * M_PI;
    return a;
}
