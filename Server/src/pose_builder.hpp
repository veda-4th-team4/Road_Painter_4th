// ① 좌표계산: 마커 코너 픽셀 → 펜 끝 POSE(x, y, θ).
//
// 전체 흐름(런타임):
//   [카메라앱이 준 코너 px] → undistortPoints(K,dist)  ← ★ TODO: OpenCV 필요
//                          → H_marker 변환(아래 applyH)
//                          → 코너 기하로 헤딩 θ
//                          → 펜 끝 오프셋 적용
// 지금은 undistort를 뺀 순수 계산부만 구현(테스트 가능). OpenCV 연동 시 앞단만 채우면 됨.
#pragma once
#include "geometry.hpp"
#include <array>

struct Homography {        // 3x3, 픽셀→지면. H_marker 또는 H_floor.
    double m[9];
    Point2 apply(double u, double v) const {
        double X = m[0]*u + m[1]*v + m[2];
        double Y = m[3]*u + m[4]*v + m[5];
        double W = m[6]*u + m[7]*v + m[8];
        return { X / W, Y / W };
    }
};

// corners: (이미 왜곡보정된) 마커 4꼭짓점 픽셀. H: H_marker. 반환: 펜 끝 POSE.
inline Pose buildPose(const std::array<Point2,4>& corners,
                      const Homography& Hm, double t, double conf) {
    // 4점을 모두 지면으로
    std::array<Point2,4> g;
    for (int i = 0; i < 4; ++i) g[i] = Hm.apply(corners[i].x, corners[i].y);
    // 중심
    Point2 c{ (g[0].x+g[1].x+g[2].x+g[3].x)/4, (g[0].y+g[1].y+g[2].y+g[3].y)/4 };
    // ★ 헤딩은 '지면평면'에서 한 변 벡터로 계산(원근 왜곡 회피)
    double theta = std::atan2(g[1].y - g[0].y, g[1].x - g[0].x);
    Pose p; p.x = c.x; p.y = c.y; p.theta = theta; p.t = t; p.conf = conf;
    // TODO: 펜 끝 오프셋(마커 중심→펜) 적용. 마커-펜 동축이면 오프셋 0.
    return p;
}
