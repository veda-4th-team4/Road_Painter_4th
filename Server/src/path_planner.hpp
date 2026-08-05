#pragma once
// 좌표/자세 계산.
//  * POS 마커 4점 (원본 픽셀) -> undistort -> H_marker -> 로봇 pose 추정
//
// ⚠️ 경로 생성은 여기 없다 - ops_builder.hpp로 옮겼다 (프로토콜 v2).
//   v1에 있던 buildSegments / withNozzleOps / advanceCursor / pointSegDistExt /
//   distToActiveSegment는 전부 삭제됐다. 앞의 둘은 ops_builder가 대체했고,
//   뒤의 셋은 "0.3m 이탈 시 복귀 PATH 재전송"을 위한 것이었는데 그 기능
//   자체가 폐지됐다 (v2는 ALIGN/MORE/DRIFT 세 피드백으로만 보정한다).
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
        if (!p["x"].is_number() || !p["y"].is_number()) return false;
        out.x = p["x"].get<double>();
        out.y = p["y"].get<double>();
        out.theta = (p.contains("theta_deg") && p["theta_deg"].is_number()
                         ? p["theta_deg"].get<double>() : 0.0) * M_PI / 180.0;
        return true;
    }
    if (!p.contains("corners") || !p["corners"].is_array() ||
        p["corners"].size() != 4 || !calib.valid)
        return false;
    Pt c[4];
    for (int i = 0; i < 4; ++i) {
        const json& q = p["corners"][i];
        if (!q.is_array() || q.size() < 2 || !q[0].is_number() ||
            !q[1].is_number())
            return false;
        if (!pixelToMarkerPlane(calib, q[0].get<double>(),
                                q[1].get<double>(), c[i]))
            return false;
    }
    out.x = (c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4;
    out.y = (c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4;
    double fx = (c[0][0] + c[1][0]) / 2, fy = (c[0][1] + c[1][1]) / 2;  // 앞변 중점
    double bx = (c[2][0] + c[3][0]) / 2, by = (c[2][1] + c[3][1]) / 2;  // 뒷변 중점
    out.theta = std::atan2(fy - by, fx - bx);
    return true;
}
