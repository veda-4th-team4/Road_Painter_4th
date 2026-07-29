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

// 시작 pose에서 폴리라인 pts를 따라가는 TURN/MOVE 시퀀스 생성.
//
// paintFlags(선택): pts와 같은 길이. paintFlags[i] = "pts[i]로 들어가는 구간을
//   칠하는가". Qt가 BLUEPRINT.paint로 구간별 도색 여부를 지정하면 그걸 그대로
//   따른다 - 도형 여러 개를 한 폴리라인으로 이어 보낼 때 "도형 사이 이동"까지
//   칠해버리던 문제를 막는다(한붓그리기 -> 여러 획).
// paintFlags가 없으면 레거시 규칙:
//   firstPaint=false: pts[0]까지는 이동만(paint=false), 이후 구간은 도색
//   firstPaint=true : 첫 구간부터 도색 (이미 시작점에 서 있는 경우)
inline json buildSegments(const Pose& start, const std::vector<Pt>& pts,
                          bool firstPaint = false,
                          const std::vector<char>* paintFlags = nullptr) {
    json segs = json::array();
    double x = start.x, y = start.y, th = start.theta;
    for (size_t i = 0; i < pts.size(); ++i) {
        double dx = pts[i][0] - x, dy = pts[i][1] - y;
        double dist = std::hypot(dx, dy);
        if (dist < 0.01) continue;  // 1cm 미만 이동은 생략
        double desired = std::atan2(dy, dx);
        double headingDeg = std::round(desired * 180.0 / M_PI * 10) / 10;
        double turn = normDeg((desired - th) * 180.0 / M_PI);
        if (std::fabs(turn) > 2.0)  // 2도 미만 회전은 생략
            // TURN에도 heading_deg(회전 후 바라볼 절대 각도)를 실어, 로봇이 회전
            // 직후 READY를 보내도 서버가 정렬 판정을 할 수 있게 한다.
            segs.push_back({{"op", "TURN"},
                            {"angle_deg", std::round(turn * 10) / 10},
                            {"heading_deg", headingDeg}});
        bool paint = (paintFlags && i < paintFlags->size())
                         ? (*paintFlags)[i] != 0
                         : (firstPaint || i > 0);
        // heading_deg: 로봇이 바라봐야 할 절대 각도(월드 기준). 전진만 하는
        // 이 함수에서는 진행 방향과 같다. 로봇은 무시해도 되고, 서버가 출발 전
        // 정렬(READY/ALIGN/GO)과 주행 중 DRIFT 판정에 사용한다.
        segs.push_back({{"op", "MOVE"},
                        {"dist_m", std::round(dist * 1000) / 1000},
                        {"paint", paint},
                        {"heading_deg", headingDeg}});
        x = pts[i][0], y = pts[i][1], th = desired;
    }
    return segs;
}

// MOVE.paint가 바뀌는 자리마다 NOZZLE op을 끼워 넣는다.
//
// 🔴 노즐 제어의 단일 결정권은 NOZZLE op이다 (2026-07-28 확정). MOVE.paint는
// "이 구간이 도색 구간인가"를 나타내는 표시일 뿐이고, 로봇은 그것만 보고
// 노즐을 움직이지 않는다. 규약이 하나여야 로봇 실행부의 분기가 하나로 끝난다.
//
// 왜 필요한가: Qt program은 NOZZLE을 직접 넣어 보내지만, program이 없어
// 서버가 buildSegments로 직접 만든 경로에는 NOZZLE이 하나도 없었다. 로봇이
// NOZZLE만 보고 움직이면 그 경로에서는 노즐이 영영 안 내려간다. 그 간극을
// 이 함수가 메운다 - buildSegments 결과를 로봇에 보내기 전에 반드시 통과시킬 것.
//
// startDown: 이 시퀀스가 시작될 때 노즐이 이미 내려가 있는가. 서버가 만드는
//   경로는 전부 노즐이 올라간 상태에서 시작하므로 기본 false다.
inline json withNozzleOps(const json& segs, bool startDown = false) {
    json out = json::array();
    bool down = startDown;
    for (const auto& s : segs) {
        if (s.value("op", "") == "MOVE") {
            bool paint = s.value("paint", false);
            if (paint != down) {
                out.push_back({{"op", "NOZZLE"}, {"down", paint}});
                down = paint;
            }
        }
        out.push_back(s);
    }
    // 경로가 끝나면 노즐은 반드시 올려둔다 - 다음 경로(접근/복귀)가 노즐이
    // 올라간 상태를 전제로 시작하기 때문. Qt program도 같은 규약으로 끝난다.
    if (down) out.push_back({{"op", "NOZZLE"}, {"down", false}});
    return out;
}

// 점 p와 선분 a-b 사이 최단 거리
inline double pointSegDist(const Pt& p, const Pt& a, const Pt& b) {
    double vx = b[0] - a[0], vy = b[1] - a[1];
    double wx = p[0] - a[0], wy = p[1] - a[1];
    double vv = vx * vx + vy * vy;
    double t = vv < 1e-12 ? 0 : std::max(0.0, std::min(1.0, (wx * vx + wy * vy) / vv));
    return std::hypot(p[0] - (a[0] + t * vx), p[1] - (a[1] + t * vy));
}

// ===== 진행 커서 (이탈 판정 / 재계획 재시작 지점) =====
//
// ⚠️ 예전에는 "폴리라인 전체에서 최소 거리"로 이탈을 재고, "전체에서 가장 가까운
// 꼭짓점"부터 재계획했다(distToPolyline / nearestVertex - 2026-07-28 삭제).
// 도면이 횡단보도처럼 나란한 줄 여러 개면 이게 두 가지로 깨진다:
//   1) 3번 줄을 칠하다 밀린 로봇에게는 4번 줄 꼭짓점이 제일 가까워, 재계획 한 번에
//      3번 줄을 버리고 4번 줄로 건너뛴다 (= "도면 순서대로 안 움직인다"의 정체).
//   2) 옆줄 위로 흘러가면 "경로 위"로 판정돼 이탈을 아예 못 잡는다.
// 그래서 "지금 몇 번째 구간을 달리는 중"인지를 커서로 들고, 판정도 재계획도
// 그 구간 기준으로만 한다.

// 커서 전진: 다음 꼭짓점(cur+1)에 reachM 이내로 닿았거나 그 구간을 지나쳤으면
// 한 칸. 절대 뒤로 가지 않는다 - 꼭짓점 동작에서 일부러 후진하는 구간이 있어도
// 커서가 되돌아가면 같은 구간을 무한 반복하기 때문.
//
// ⚠️ 단조 증가의 대가: 로봇이 크게 밀려 뒤로 물러난 경우 커서는 앞선 채로 남아
// 이미 지난 구간으로는 복귀하지 않는다. 되돌리려면 새 BLUEPRINT가 필요하다.
inline void advanceCursor(const Pt& p, const std::vector<Pt>& pts, size_t& cur,
                          double reachM) {
    while (cur + 1 < pts.size()) {
        const Pt& a = pts[cur];
        const Pt& b = pts[cur + 1];
        if (std::hypot(p[0] - b[0], p[1] - b[1]) <= reachM) {
            ++cur;
            continue;
        }
        double vx = b[0] - a[0], vy = b[1] - a[1];
        double vv = vx * vx + vy * vy;
        if (vv < 1e-12) {  // 길이 0 구간은 그냥 넘긴다
            ++cur;
            continue;
        }
        double t = ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / vv;
        // 지나침 판정은 "그 구간 위에 있을 때"만 신뢰한다. 크게 벗어난 로봇의
        // 투영값은 여러 구간에서 동시에 1을 넘어, 한 번에 꼭짓점 여러 개를
        // 건너뛰어 버린다 (커서는 되돌릴 수 없으므로 그대로 굳는다).
        double perp = std::fabs(vx * (p[1] - a[1]) - vy * (p[0] - a[0])) /
                      std::sqrt(vv);
        if (t > 1.0 && perp <= reachM) {
            ++cur;
            continue;
        }
        break;
    }
}

// 선분 a-b의 "끝"을 진행 방향으로 extM만큼 늘린 뒤 거리를 잰다.
//
// 왜 늘리냐: 도면은 펜이 지나갈 자취인데 서버가 보는 pose는 마커 중심이고,
// 펜은 중심 뒤 d에 달려 있다. 그래서 펜이 꼭짓점 B에 정확히 닿은 순간 마커
// 중심은 이미 B를 d만큼 지나 있다 - 정상인데 "d만큼 이탈"로 찍힌다.
// 끝을 d만큼 늘려두면 그 상태가 오차 0이 되고, 거기서 더 간 만큼만 이탈로 센다.
// (= "B에서 d보다 더 가면 이탈")
inline double pointSegDistExt(const Pt& p, const Pt& a, const Pt& b,
                              double extM) {
    if (extM <= 0) return pointSegDist(p, a, b);
    double vx = b[0] - a[0], vy = b[1] - a[1];
    double len = std::hypot(vx, vy);
    if (len < 1e-9) return pointSegDist(p, a, b);
    Pt b2{b[0] + vx / len * extM, b[1] + vy / len * extM};
    return pointSegDist(p, a, b2);
}

// 지금 달리고 있는 구간과의 거리 = 진짜 이탈량.
// 커서 전진이 한 박자 늦을 수 있어 다음 구간까지만 같이 본다(꼭짓점 부근에서
// 정상 주행이 이탈로 오탐되는 것 방지).
// penOffsetM: 위 pointSegDistExt 참고. 0이면 종전과 동일(중심 기준 그대로).
inline double distToActiveSegment(const Pt& p, const std::vector<Pt>& pts,
                                  size_t cur, double penOffsetM = 0.0) {
    if (pts.empty()) return 0.0;
    if (pts.size() == 1) return std::hypot(p[0] - pts[0][0], p[1] - pts[0][1]);
    if (cur + 1 >= pts.size()) cur = pts.size() - 2;
    double d = pointSegDistExt(p, pts[cur], pts[cur + 1], penOffsetM);
    if (cur + 2 < pts.size())
        d = std::min(d, pointSegDistExt(p, pts[cur + 1], pts[cur + 2], penOffsetM));
    return d;
}
