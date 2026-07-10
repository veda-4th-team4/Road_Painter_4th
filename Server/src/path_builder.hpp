// ② 경로생성: 도색 라인 세그먼트 → 촘촘한 웨이포인트 (PATH의 본체).
// 실제로는 앞단에 [Qt 작도 픽셀 → undistort → H_floor 변환]이 붙는다(pose_builder와 유사).
// 여기서는 이미 지면좌표가 된 세그먼트를 받아 획을 웨이포인트로 분해한다.
#pragma once
#include "geometry.hpp"

class PathBuilder {
public:
    explicit PathBuilder(double step_m = 0.05) : step_(step_m) {}

    std::vector<Waypoint> build(const std::vector<Segment>& segs) const {
        std::vector<Waypoint> path;
        Point2 cur{0, 0}; bool have = false;
        for (const auto& s : segs) {
            if (have && dist(cur.x, cur.y, s.p0.x, s.p0.y) > step_)
                densify(cur, s.p0, false, path);   // 획 사이 이동(펜 업)
            densify(s.p0, s.p1, s.paint, path);     // 획 본체
            cur = s.p1; have = true;
        }
        return path;
    }
private:
    double step_;
    void densify(Point2 a, Point2 b, bool paint, std::vector<Waypoint>& out) const {
        double d = dist(a.x, a.y, b.x, b.y);
        int n = std::max(1, (int)std::ceil(d / step_));
        for (int i = 1; i <= n; ++i) {
            double u = (double)i / n;
            out.push_back({a.x + (b.x - a.x) * u, a.y + (b.y - a.y) * u, paint});
        }
    }
};
