#pragma once
// 경로 계산 (러프 버전) - 디폴트 가정으로 일단 동작하게 만든 것. 추후 구체화 예정.
//  * POS 마커 4점 (원본 픽셀) -> undistort -> H_marker -> 로봇 pose 추정
//  * 도면 폴리라인 (바닥 미터, Qt가 변환 완료) + 로봇 pose -> TURN/MOVE 시퀀스 생성
//  * 계획 경로와의 이탈 거리 계산 (재계획 판단용)
#include "calib.hpp"
#include "protocol.hpp"
#include <array>
#include <cmath>
#include <vector>

struct Pose {
    double x = 0, y = 0;
    double theta = 0;  // 라디안, 바닥 좌표계 +x축 기준 반시계
};
using Pt = std::array<double, 2>;

// (-180, 180] 범위로 정규화
inline double normDeg(double a) {
    while (a > 180) a -= 360;
    while (a <= -180) a += 360;
    return a;
}

// POS payload -> 바닥 좌표계 pose.
//   {"corners":[[u,v]x4]}  순서 = [전좌, 전우, 후우, 후좌] (CCTV "원본 픽셀" 좌표)
//     -> 코너별 undistort -> H_marker -> 미터 좌표 4점 -> 중심/방향 계산.
//        방향은 반드시 "변환 후" 좌표로 계산 (호모그래피는 각도 비보존).
//   {"x":..,"y":..,"theta_deg":..} 가 오면 이미 바닥 좌표 값으로 보고 그대로 사용 (테스트용)
inline bool poseFromPos(const json& p, const Calib& calib, Pose& out) {
    if (p.contains("x") && p.contains("y")) {
        out.x = p["x"].get<double>();
        out.y = p["y"].get<double>();
        out.theta = p.value("theta_deg", 0.0) * M_PI / 180.0;
        return true;
    }
    if (!p.contains("corners") || !p["corners"].is_array() ||
        p["corners"].size() != 4 || !calib.valid)
        return false;
    Pt c[4];
    for (int i = 0; i < 4; ++i)
        if (!pixelToMarkerPlane(calib, p["corners"][i][0].get<double>(),
                                p["corners"][i][1].get<double>(), c[i]))
            return false;
    out.x = (c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4;
    out.y = (c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4;
    double fx = (c[0][0] + c[1][0]) / 2, fy = (c[0][1] + c[1][1]) / 2;  // 앞변 중점
    double bx = (c[2][0] + c[3][0]) / 2, by = (c[2][1] + c[3][1]) / 2;  // 뒷변 중점
    out.theta = std::atan2(fy - by, fx - bx);
    return true;
}

// 시작 pose에서 폴리라인 pts를 그리는 TURN/MOVE 시퀀스 생성.
// pts[0]까지는 이동만(paint=false), 이후 구간은 도색(paint=true).
inline json buildSegments(const Pose& start, const std::vector<Pt>& pts) {
    json segs = json::array();
    double x = start.x, y = start.y, th = start.theta;
    for (size_t i = 0; i < pts.size(); ++i) {
        double dx = pts[i][0] - x, dy = pts[i][1] - y;
        double dist = std::hypot(dx, dy);
        if (dist < 0.01) continue;  // 1cm 미만 이동은 생략
        double desired = std::atan2(dy, dx);
        double turn = normDeg((desired - th) * 180.0 / M_PI);
        if (std::fabs(turn) > 2.0)  // 2도 미만 회전은 생략
            segs.push_back(
                {{"op", "TURN"}, {"angle_deg", std::round(turn * 10) / 10}});
        segs.push_back({{"op", "MOVE"},
                        {"dist_m", std::round(dist * 1000) / 1000},
                        {"paint", i > 0}});
        x = pts[i][0], y = pts[i][1], th = desired;
    }
    return segs;
}

// 점 p와 선분 a-b 사이 최단 거리
inline double pointSegDist(const Pt& p, const Pt& a, const Pt& b) {
    double vx = b[0] - a[0], vy = b[1] - a[1];
    double wx = p[0] - a[0], wy = p[1] - a[1];
    double vv = vx * vx + vy * vy;
    double t = vv < 1e-12 ? 0 : std::max(0.0, std::min(1.0, (wx * vx + wy * vy) / vv));
    return std::hypot(p[0] - (a[0] + t * vx), p[1] - (a[1] + t * vy));
}

// 폴리라인 전체와의 최소 거리 (이탈 판정)
inline double distToPolyline(const Pt& p, const std::vector<Pt>& pts) {
    if (pts.size() == 1) return std::hypot(p[0] - pts[0][0], p[1] - pts[0][1]);
    double best = 1e18;
    for (size_t i = 0; i + 1 < pts.size(); ++i)
        best = std::min(best, pointSegDist(p, pts[i], pts[i + 1]));
    return best;
}

// 가장 가까운 꼭짓점 index (재계획 시 재시작 지점 - 러프 기준)
inline size_t nearestVertex(const Pt& p, const std::vector<Pt>& pts) {
    size_t k = 0;
    double best = 1e18;
    for (size_t i = 0; i < pts.size(); ++i) {
        double d = std::hypot(p[0] - pts[i][0], p[1] - pts[i][1]);
        if (d < best) best = d, k = i;
    }
    return k;
}
