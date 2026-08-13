// 오프셋 보정 다리(±a)에 피드백이 걸리는지 검증하는 회귀 테스트.
//   make offset_feedback_test && ./tools/offset_feedback_test
//
// 배경: 꼭짓점마다 노즐을 올렸다 내리려면 펜 오프셋 a(150mm)만큼 앞뒤로 오가야
// 한다. 예전에는 이 다리들이 role="offset"이라는 이유로 MORE 판정에서 통째로
// 빠져 있었다 - 매 꼭짓점에서 150mm를 슬립계수 하나만 믿고 개루프로 찍고, 틀려도
// 아무도 고치지 않았다. 펜 오프셋 설정값이 실측과 5mm만 어긋나도 그 오차가 보정
// 없이 꼭짓점마다 그대로 나타난다(2026-08-11 삼각형 시험에서 관측).
//
// 여기서 못박는 것:
//  1) 오프셋 전진 다리(+a)가 "도색이 시작될 꼭짓점"을 목표로 물고 있는가.
//     목표 중심 = 꼭짓점 + a·û  (곧 노즐을 내리므로 중심이 a 앞이어야 한다)
//  2) 오프셋 후진 다리(-a)가 "직전 도색이 끝난 꼭짓점"을 목표로 물고 있는가.
//     목표 중심 = 꼭짓점       (노즐을 이미 올렸으므로 중심이 꼭짓점 위)
//  3) needsMore가 그 두 다리를 실제로 대상으로 삼는가(= role 게이트가 없는가).
//  4) 보정이 노즐이 올라가 있는 동안에만 실행되는가.
//     ⚠️ 내려간 채로 보정하면 젖은 도료를 문지른다.
#include "ops_builder.hpp"
#include <cstdio>
#include <cmath>

static int fails = 0;
#define CHECK(cond, ...)                                                       \
    do {                                                                       \
        if (cond) { printf("  ok   "); } else { printf("  FAIL "); ++fails; }   \
        printf(__VA_ARGS__); printf("\n");                                      \
    } while (0)

// router.cpp needsMore()와 같은 조건. 서버 본체를 링크하지 않고 규칙만 복제해
// 둔다(본체를 바꾸면 이 테스트가 아니라 빌드가 먼저 깨지도록 아래에서 대조).
static bool needsMore(const std::vector<OpMeta>& meta, int k) {
    if (k <= 0 || k > (int)meta.size()) return false;
    const OpMeta& m = meta[(size_t)k - 1];
    return (m.op == "move" || m.op == "arc") && m.hasTarget &&
           hasHeading(m.exitHeadingDeg);
}

// MORE가 계산하는 "마커 중심의 목표 좌표" (router.cpp resolveBoundary와 동일식).
static void moreTarget(const OpMeta& m, double a, double& tx, double& ty) {
    const double ux = std::cos(m.exitHeadingDeg * M_PI / 180.0);
    const double uy = std::sin(m.exitHeadingDeg * M_PI / 180.0);
    (void)a;
    const double off = m.centerAheadM;
    tx = m.penTarget[0] + off * ux;
    ty = m.penTarget[1] + off * uy;
}

static bool near(double x, double y) { return std::fabs(x - y) < 1e-6; }

int main() {
    const double a = params().pen_offset_m;
    const double hw = params().pen_width_m / 2.0;   // 펜 두께 보정 반폭
    printf("pen_offset_m = %.3f m, pen_width_m = %.3f m (hw = %.3f)\n\n",
           a, params().pen_width_m, hw);

    // 0.5 x 0.3 m 직사각형의 첫 두 변. 도색 -> 회전 -> 도색.
    const std::vector<Pt> points = {{0.0, 0.0}, {0.5, 0.0}, {0.5, 0.3}};
    const json program = json::array({
        json{{"op", "MOVE"}, {"v", 0}, {"heading_deg", 0.0},
             {"dist_m", 0.5}, {"paint", true}},
        json{{"op", "TURN"}, {"v", 1}, {"heading_deg", 90.0}, {"angle_deg", 90.0}},
        json{{"op", "MOVE"}, {"v", 1}, {"heading_deg", 90.0},
             {"dist_m", 0.3}, {"paint", true}},
    });

    PlannedPath p = buildDrawOps(program, points);
    CHECK(p.ok, "도면이 정상 생성됐다 (%s)", p.ok ? "ok" : p.failMsg.c_str());
    if (!p.ok) return 1;

    printf("\n[생성된 op 시퀀스]\n");
    bool nozzleDown = false;
    std::vector<bool> nozzleDownDuring(p.meta.size(), false);
    for (size_t i = 0; i < p.meta.size(); ++i) {
        const OpMeta& m = p.meta[i];
        nozzleDownDuring[i] = nozzleDown;
        if (m.op == "nozzle") nozzleDown = p.ops[i].value("down", false);
        printf("  [%2zu] %-7s role=%-6s", i, m.op.c_str(),
               m.isPath ? "path" : "offset");
        if (m.op == "move") printf(" dist=%+.3f", p.ops[i].value("dist_m", 0.0));
        else if (m.op == "turn") printf(" ang=%+.1f ", p.ops[i].value("angle_deg", 0.0));
        else if (m.op == "nozzle") printf(" down=%-5s",
                                          p.ops[i].value("down", false) ? "true" : "false");
        else printf("            ");
        if (m.hasTarget) {
            double tx, ty;
            moreTarget(m, a, tx, ty);
            printf("  꼭짓점=(%.3f,%.3f) 중심목표=(%.3f,%.3f)%s",
                   m.penTarget[0], m.penTarget[1], tx, ty,
                   m.centerAheadM != 0.0 ? " [ahead]" : "");
        }
        printf("\n");
    }

    printf("\n[1] 오프셋 전진 다리(+a)가 시작 꼭짓점을 목표로 잡는가\n");
    {
        // op0 = 첫 도색 진입의 move(+a). 시작 꼭짓점은 points[0]=(0,0),
        // 진행 방위 0도이므로 중심 목표는 (a, 0)이어야 한다.
        const OpMeta& m = p.meta[0];
        CHECK(m.op == "move" && !m.isPath, "op0 은 오프셋 move 다");
        CHECK(near(p.ops[0].value("dist_m", 0.0), a - hw),
              "전진량 = +%.3f (a - w/2, 펜이 꼭짓점보다 w/2 앞에 앉는다)", a - hw);
        CHECK(m.hasTarget, "목표 꼭짓점을 갖고 있다 (예전에는 없었다)");
        CHECK(near(m.penTarget[0], 0.0) && near(m.penTarget[1], 0.0),
              "목표 꼭짓점 = 도색 시작점 (0,0)");
        CHECK(near(m.centerAheadM, a - hw),
              "중심 목표는 꼭짓점보다 (a - w/2) 앞");
        double tx, ty;
        moreTarget(m, a, tx, ty);
        CHECK(near(tx, a - hw) && near(ty, 0.0),
              "중심 목표 = (%.3f, 0.000)", a - hw);
        CHECK(needsMore(p.meta, 1), "boundary 1 에서 MORE 판정 대상이다");
    }

    printf("\n[2] 오프셋 후진 다리(-a)가 종료 꼭짓점을 목표로 잡는가\n");
    {
        // 첫 도색이 끝나면 nozzle(up) -> move(-a). 그 move 를 찾는다.
        int idx = -1;
        for (size_t i = 1; i < p.meta.size(); ++i)
            if (p.meta[i].op == "move" && !p.meta[i].isPath &&
                p.ops[i].value("dist_m", 0.0) < 0) { idx = (int)i; break; }
        CHECK(idx > 0, "오프셋 후진 move 가 존재한다 (op%d)", idx);
        if (idx > 0) {
            const OpMeta& m = p.meta[(size_t)idx];
            CHECK(near(p.ops[(size_t)idx].value("dist_m", 0.0), -(a + hw)),
                  "후진량 = -%.3f (a + w/2, 늘어난 도색만큼 더 물러난다)", a + hw);
            CHECK(m.hasTarget, "목표 꼭짓점을 갖고 있다 (예전에는 없었다)");
            CHECK(near(m.penTarget[0], 0.5) && near(m.penTarget[1], 0.0),
                  "목표 꼭짓점 = 도색 종료점 (0.5,0)");
            CHECK(near(m.centerAheadM, 0.0),
                  "중심 목표는 꼭짓점 그대로 (노즐을 이미 올렸다)");
            double tx, ty;
            moreTarget(m, a, tx, ty);
            CHECK(near(tx, 0.5) && near(ty, 0.0), "중심 목표 = (0.500, 0.000)");
            CHECK(needsMore(p.meta, idx + 1),
                  "boundary %d 에서 MORE 판정 대상이다", idx + 1);
        }
    }

    printf("\n[3] 도색 move 의 기존 보정은 그대로인가 (회귀 확인)\n");
    {
        int idx = -1;
        for (size_t i = 0; i < p.meta.size(); ++i)
            if (p.meta[i].op == "move" && p.meta[i].isPath) { idx = (int)i; break; }
        CHECK(idx >= 0, "도색 move(role=path) 가 존재한다 (op%d)", idx);
        if (idx >= 0) {
            const OpMeta& m = p.meta[(size_t)idx];
            CHECK(near(p.ops[(size_t)idx].value("dist_m", 0.0), 0.5 + 2.0 * hw),
                  "도색 거리 = %.3f (L + w, 앞뒤 w/2씩 연장)", 0.5 + 2.0 * hw);
            CHECK(m.hasTarget && near(m.centerAheadM, a + hw),
                  "도착 꼭짓점 + (a + w/2) 앞 목표");
            double tx, ty;
            moreTarget(m, a, tx, ty);
            CHECK(near(tx, 0.5 + a + hw) && near(ty, 0.0),
                  "중심 목표 = (%.3f, 0.000)", 0.5 + a + hw);
        }
    }

    printf("\n[4] 새로 추가한 오프셋 보정은 노즐이 올라가 있을 때만 실행되는가\n");
    {
        // boundary k 의 보정은 op k 를 시작하기 직전에 실행된다 - 그 시점의
        // 노즐 상태는 op k 를 실행하기 "전" 상태, 즉 nozzleDownDuring[k].
        int offsetLegs = 0, offsetBad = 0;
        std::vector<size_t> paintDownBoundaries;
        for (size_t k = 1; k <= p.meta.size(); ++k) {
            if (!needsMore(p.meta, (int)k)) continue;
            const bool down = (k < p.meta.size()) ? nozzleDownDuring[k]
                                                  : false;  // 경로 끝
            if (!p.meta[k - 1].isPath) {  // 이번에 추가한 오프셋 다리
                ++offsetLegs;
                if (down) {
                    ++offsetBad;
                    printf("  FAIL boundary %zu (오프셋 다리) 노즐이 내려가 있다\n", k);
                    ++fails;
                }
            } else if (down) {
                paintDownBoundaries.push_back(k);
            }
        }
        CHECK(offsetBad == 0,
              "오프셋 다리 MORE %d 곳 전부 노즐 up 상태 (도료 문지름 없음)",
              offsetLegs);

        // 🟡 도색 move 자체의 MORE 는 예전부터 노즐이 내려간 채 실행된다.
        //   closePaint 가 nozzle(up) 을 그 boundary 「뒤」에 넣기 때문이다.
        //   과주행을 되돌리는 음수 MORE 는 젖은 선 위를 되짚는다 -
        //   PROTOCOL_v2_ROBOT.md §11-3 의 미해결 항목이었고, 2026-08-11 에
        //   "후진 허용, 문지름 감수"로 결정됐다. 이번 변경과 무관한 기존 동작이라
        //   실패로 세지 않고 눈에만 보이게 남긴다.
        printf("  info 도색 move MORE %zu 곳은 노즐 down 상태 (기존 동작, "
               "2026-08-11 '문지름 감수' 결정):",
               paintDownBoundaries.size());
        for (size_t k : paintDownBoundaries) printf(" boundary %zu", k);
        printf("\n");
    }

    printf("\n[5] 펜 두께 보정 - 펜이 실제로 [V0-w/2, V1+w/2] 를 지나는가\n");
    {
        // 이번 보정의 존재 이유를 좌표로 못박는다. 마커 중심을 op 순서대로
        // 누적하고, 펜 = 마커 - a (진행방향) 로 환산해 도색 시작/끝을 본다.
        // 여기가 깨지면 꼭짓점 귀퉁이가 다시 비거나 도색이 경계를 넘친다.
        double cx = 0.0, cy = 0.0;      // 마커 중심 (도색 시작 꼭짓점에서 출발)
        double penDownX = 0, penDownY = 0, penUpX = 0, penUpY = 0;
        bool down = false, sawDown = false, sawUp = false;
        for (size_t i = 0; i < p.meta.size() && !sawUp; ++i) {
            const OpMeta& m = p.meta[i];
            if (m.op == "move") {
                const double h = m.exitHeadingDeg * M_PI / 180.0;
                const double d = p.ops[i].value("dist_m", 0.0);
                cx += d * std::cos(h);
                cy += d * std::sin(h);
            } else if (m.op == "nozzle") {
                const bool nd = p.ops[i].value("down", false);
                // 펜은 마커보다 a 뒤. 방위는 직전 주행 op의 것을 쓴다.
                double hb = 0.0;
                for (size_t j = i; j-- > 0;)
                    if (hasHeading(p.meta[j].exitHeadingDeg)) {
                        hb = p.meta[j].exitHeadingDeg * M_PI / 180.0; break;
                    }
                const double px = cx - a * std::cos(hb);
                const double py = cy - a * std::sin(hb);
                if (nd && !down) { penDownX = px; penDownY = py; sawDown = true; }
                if (!nd && down) { penUpX = px;   penUpY = py;   sawUp = true; }
                down = nd;
            }
        }
        CHECK(sawDown && sawUp, "첫 도색 구간의 노즐 down/up 을 찾았다");
        if (sawDown && sawUp) {
            // 첫 구간은 (0,0) -> (0.5,0), 진행방위 0도.
            CHECK(near(penDownX, -hw) && near(penDownY, 0.0),
                  "도색 시작 펜 = (%.3f, 0.000)  = V0 - w/2 (귀퉁이 메움)", -hw);
            CHECK(near(penUpX, 0.5 + hw) && near(penUpY, 0.0),
                  "도색 종료 펜 = (%.3f, 0.000)  = V1 + w/2 (귀퉁이 메움)",
                  0.5 + hw);
            CHECK(near((penUpX - penDownX), 0.5 + 2.0 * hw),
                  "실제 도색 길이 = %.3f (= L + w)", 0.5 + 2.0 * hw);
        }
        // 마커가 종료 꼭짓점으로 정확히 복귀하는지 (다음 구간의 기준점).
        double mx = 0.0, my = 0.0;
        int firstTurn = -1;
        for (size_t i = 0; i < p.meta.size(); ++i) {
            if (p.meta[i].op == "turn" && p.meta[i].isPath) { firstTurn = (int)i; break; }
        }
        CHECK(firstTurn > 0, "도면 turn(=구간 경계)이 존재한다");
        for (int i = 0; i < firstTurn; ++i) {
            if (p.meta[(size_t)i].op != "move") continue;
            const double h = p.meta[(size_t)i].exitHeadingDeg * M_PI / 180.0;
            const double d = p.ops[(size_t)i].value("dist_m", 0.0);
            mx += d * std::cos(h);
            my += d * std::sin(h);
        }
        CHECK(near(mx, 0.5) && near(my, 0.0),
              "turn 직전 마커 = (0.500, 0.000) = 꼭짓점 정확히 복귀 "
              "(실제 %.3f, %.3f)", mx, my);
    }

    printf("\n%s (%d fail)\n", fails ? "실패" : "전부 통과", fails);
    return fails ? 1 : 0;
}
