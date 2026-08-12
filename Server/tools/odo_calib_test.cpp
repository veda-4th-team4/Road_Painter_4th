// 로봇 오도메트리 주행 캘리 회귀 테스트 (2026-08-12 신설).
//   make odo_calib_test && ./tools/odo_calib_test
//
// 규격: docs/ROBOT_ODOMETRY_HOMOGRAPHY_WIRE_20260812.md (정본),
//       docs/ROBOT_ODOMETRY_HOMOGRAPHY_PLAN_20260811.md.
//
// Router(TLS 연결·뮤텍스)를 띄우지 않고 ops_builder.hpp의 순수 함수만 테스트한다
// (buildCalibRectOps/odoReadyToPoint/odoPointWorldMm) - 이 세션의 매칭 키·안전
// 정지 등 Router 상태를 쥔 로직(router_odocalib.cpp)은 실제 로봇/CCTV 없이는
// 통합 테스트로만 검증 가능하다. 여기서는 "로봇에 나가는 경로가 맞는가"와
// "CCTV에 보낼 좌표가 맞는가"라는, 조용히 틀려도 눈에 안 띄는 두 부분을 못박는다.
#include "ops_builder.hpp"
#include <cstdio>
#include <cmath>

static int fails = 0;
#define CHECK(cond, ...)                                                       \
    do {                                                                       \
        if (cond) { printf("  ok   "); } else { printf("  FAIL "); ++fails; }   \
        printf(__VA_ARGS__); printf("\n");                                      \
    } while (0)

static bool near(double a, double b, double eps = 1e-6) { return std::fabs(a - b) < eps; }

// ----- T1: 11-op 시퀀스 -----
static void testOpSequence() {
    printf("-- T1: buildCalibRectOps 11-op 시퀀스 --\n");
    for (bool ccw : {true, false}) {
        PlannedPath p = buildCalibRectOps(0.90, 0.60, ccw);
        CHECK(p.meta.size() == 11, "op 개수 11 (ccw=%d, 실제 %zu)", ccw, p.meta.size());
        CHECK(p.ops.size() == 11, "ops json 개수 11 (ccw=%d, 실제 %zu)", ccw, p.ops.size());

        // op 2/5/8이 turn, 나머지는 move (0-based)
        static const bool isTurn[11] = {false, false, true, false, false, true,
                                        false, false, true, false, false};
        bool structOk = true;
        for (int i = 0; i < 11; ++i) {
            const std::string expect = isTurn[i] ? "turn" : "move";
            if (p.meta[i].op != expect) structOk = false;
        }
        CHECK(structOk, "op 구조 [move move turn]x3 + move move (ccw=%d)", ccw);

        // 반쪽 구간 거리 = m/2 또는 n/2 (m,n 교대: leg0=m leg1=n leg2=m leg3=n)
        CHECK(near((double)p.ops[0]["dist_m"], 0.45) && near((double)p.ops[1]["dist_m"], 0.45),
              "1구간(m) 반쪽 = 0.45m (ccw=%d)", ccw);
        CHECK(near((double)p.ops[3]["dist_m"], 0.30) && near((double)p.ops[4]["dist_m"], 0.30),
              "2구간(n) 반쪽 = 0.30m (ccw=%d)", ccw);
        CHECK(near((double)p.ops[6]["dist_m"], 0.45) && near((double)p.ops[7]["dist_m"], 0.45),
              "3구간(m) 반쪽 = 0.45m (ccw=%d)", ccw);
        CHECK(near((double)p.ops[9]["dist_m"], 0.30) && near((double)p.ops[10]["dist_m"], 0.30),
              "4구간(n) 반쪽 = 0.30m (ccw=%d)", ccw);

        // 회전 부호: 로봇 대면은 "양수=오른쪽"(toRobotDeg가 CCW를 반전) -
        // ccw=true(CCW, 왼쪽)면 전선에는 음수, ccw=false(CW, 오른쪽)면 양수로 나간다.
        const double wireDeg = (double)p.ops[2]["angle_deg"];
        CHECK(ccw ? wireDeg < 0 : wireDeg > 0,
              "TURN 부호 ccw=%d -> 전선값 %.1f (음수=왼쪽/CCW)", ccw, wireDeg);
        CHECK(near(std::fabs(wireDeg), 90.0), "TURN 크기 90도 (실제 %.1f)", wireDeg);

        // 피드백 차단: 전부 hasTarget=false, headingDeg=kNoHeading, isPath=false
        bool feedbackBlocked = true;
        for (auto& m : p.meta) {
            if (m.hasTarget || m.isPath || hasHeading(m.headingDeg) ||
                hasHeading(m.exitHeadingDeg))
                feedbackBlocked = false;
        }
        CHECK(feedbackBlocked,
              "전부 hasTarget=false/isPath=false/headingDeg=kNoHeading (ccw=%d)", ccw);
    }
}

// ----- T2: boundary -> point_index 매핑 -----
static void testReadyToPoint() {
    printf("-- T2: odoReadyToPoint boundary 매핑 --\n");
    static const int expect[11] = {0, 1, 2, -1, 3, 4, -1, 5, 6, -1, 7};
    bool ok = true;
    for (int k = 0; k < 11; ++k)
        if (odoReadyToPoint(k) != expect[k]) ok = false;
    CHECK(ok, "READY(0..10) -> point_index 표 일치 (TURN 직후 3곳만 -1)");
    CHECK(odoReadyToPoint(-1) == -2, "범위 밖(-1) -> -2 (호출측 방어 처리 신호)");
    CHECK(odoReadyToPoint(11) == -2, "범위 밖(11) -> -2");
}

// ----- T3: 9점 좌표표 -----
static void testWorldCoords() {
    printf("-- T3: odoPointWorldMm 9점 좌표표 (m_cm=90,n_cm=60 -> m=900mm,n=600mm) --\n");
    const double m = 900.0, n = 600.0;

    // bottom_left(ccw=true): wire 스펙 §4 표
    static const double expectBL[9][2] = {
        {0, 0}, {450, 0}, {900, 0}, {900, 300}, {900, 600},
        {450, 600}, {0, 600}, {0, 300}, {0, 0}};
    bool blOk = true;
    for (int i = 0; i < 9; ++i) {
        auto xy = odoPointWorldMm(i, m, n, /*ccw=*/true);
        if (!near(xy[0], expectBL[i][0], 1e-3) || !near(xy[1], expectBL[i][1], 1e-3))
            blOk = false;
    }
    CHECK(blOk, "bottom_left 9점 전부 일치");

    // top_left(ccw=false): wire 스펙 §4 표
    static const double expectTL[9][2] = {
        {0, 600}, {450, 600}, {900, 600}, {900, 300}, {900, 0},
        {450, 0}, {0, 0}, {0, 300}, {0, 600}};
    bool tlOk = true;
    for (int i = 0; i < 9; ++i) {
        auto xy = odoPointWorldMm(i, m, n, /*ccw=*/false);
        if (!near(xy[0], expectTL[i][0], 1e-3) || !near(xy[1], expectTL[i][1], 1e-3))
            tlOk = false;
    }
    CHECK(tlOk, "top_left 9점 전부 일치");

    // idx 8(복귀)은 idx 0과 라벨이 같아야 한다 (findHomography 입력 제외 근거)
    auto p0bl = odoPointWorldMm(0, m, n, true), p8bl = odoPointWorldMm(8, m, n, true);
    CHECK(near(p0bl[0], p8bl[0]) && near(p0bl[1], p8bl[1]),
          "bottom_left: idx8 == idx0 (복귀점 라벨 일치)");
    auto p0tl = odoPointWorldMm(0, m, n, false), p8tl = odoPointWorldMm(8, m, n, false);
    CHECK(near(p0tl[0], p8tl[0]) && near(p0tl[1], p8tl[1]),
          "top_left: idx8 == idx0 (복귀점 라벨 일치)");

    // idx3/idx7(세로변 중점)은 두 start_corner에서 동일해야 한다 (계획서 §3 표 각주)
    auto p3bl = odoPointWorldMm(3, m, n, true), p3tl = odoPointWorldMm(3, m, n, false);
    CHECK(near(p3bl[0], p3tl[0]) && near(p3bl[1], p3tl[1]),
          "idx3 (세로변 중점)은 start_corner 무관하게 동일 (900,300)");
    auto p7bl = odoPointWorldMm(7, m, n, true), p7tl = odoPointWorldMm(7, m, n, false);
    CHECK(near(p7bl[0], p7tl[0]) && near(p7bl[1], p7tl[1]),
          "idx7 (세로변 중점)은 start_corner 무관하게 동일 (0,300)");
}

// ----- T4: 마커 오프셋 보정 -----
static void testMarkerOffset() {
    printf("-- T4: 마커 오프셋 보정 (marker_offset_m != 0) --\n");
    const double m = 900.0, n = 600.0;
    const double offM = 0.1;  // 100mm

    // offset=0이면 보정 없음 (기본값 경로가 그대로임을 확인)
    auto noOff = odoPointWorldMm(1, m, n, true, 0.0);
    CHECK(near(noOff[0], 450.0) && near(noOff[1], 0.0),
          "offset=0.0 -> 표 그대로 (idx1 bottom_left)");

    // idx0,1,2 (leg0, heading 0도=+x)에서 offset은 x에만 더해진다
    auto withOff = odoPointWorldMm(1, m, n, true, offM);
    CHECK(near(withOff[0], 450.0 + 100.0) && near(withOff[1], 0.0),
          "offset=0.1m, heading 0도(+x) -> x만 +100mm (idx1 bottom_left, 실제 (%.1f,%.1f))",
          withOff[0], withOff[1]);

    // idx3,4 (leg1, bottom_left heading 90도=+y)에서 offset은 y에만 더해진다
    auto withOffLeg1 = odoPointWorldMm(3, m, n, true, offM);
    CHECK(near(withOffLeg1[0], 900.0) && near(withOffLeg1[1], 300.0 + 100.0),
          "offset=0.1m, heading 90도(+y) -> y만 +100mm (idx3 bottom_left, 실제 (%.1f,%.1f))",
          withOffLeg1[0], withOffLeg1[1]);

    // top_left는 leg1 heading이 -90도(-y)라 부호가 반대
    auto withOffTL = odoPointWorldMm(3, m, n, false, offM);
    CHECK(near(withOffTL[0], 900.0) && near(withOffTL[1], 300.0 - 100.0),
          "offset=0.1m, top_left heading -90도(-y) -> y만 -100mm (idx3 top_left, 실제 (%.1f,%.1f))",
          withOffTL[0], withOffTL[1]);
}

int main() {
    testOpSequence();
    testReadyToPoint();
    testWorldCoords();
    testMarkerOffset();
    printf("\n%s (%d fail)\n", fails == 0 ? "전부 통과" : "실패 있음", fails);
    return fails == 0 ? 0 : 1;
}
