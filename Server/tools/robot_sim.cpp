// 로봇 + CCTV 대역 시뮬레이터 (서버 알고리즘 드라이런용) - TCP/TLS
// 🔴 프로토콜 v2 (server-driven) - 규격: docs/PROTOCOL_v2_ROBOT.md
//
// 목적: 실제 로봇 없이 서버의 주행 알고리즘 전체를 닫힌 루프로 돌려본다.
//   서버가 보낸 PATH를 실행 -> 그 결과 위치를 POS로 되돌려줌 -> 서버가 그걸 보고
//   ALIGN/MORE/DRIFT를 다시 보냄. 즉 "서버가 보낸 대로 움직인 결과"가 다시
//   서버 입력이 되므로, 한 번 찍고 끝나는 게 아니라 실시간 피드백까지 검증된다.
//
// 왜 한 프로세스가 ROBOT과 CCTV 두 role로 붙나:
//   서버는 "로봇이 어디 있는지"를 오직 CCTV의 POS로만 안다. 시뮬레이터가 자기
//   위치를 POS로 흘려넣지 않으면 서버는 로봇이 움직이는 걸 볼 수 없다.
//   POS는 {"x","y","theta_deg"} 형식(서버 poseFromPos의 테스트 경로)으로 보내
//   캘리브레이션 없이도 돌아가게 한다 - 좌표 변환은 이 도구의 검증 대상이 아니다.
//
// 🔴 이 시뮬이 지키는 v2 규약 (실제 로봇도 똑같이 해야 하는 것):
//   - 모든 op 앞에서 READY{op_index}를 보내고 GO를 받을 때까지 움직이지 않는다
//   - ALIGN/MORE를 받으면 수행 후 "같은 op_index로" READY를 다시 보낸다
//   - 자기가 기다리는 index와 다른 GO/ALIGN/MORE/DRIFT는 조용히 버린다
//   - 로봇 대면 각도는 "양수 = 오른쪽(CW)"이다
//   - arc.radius_m은 이미 펜 보정이 끝난 값이라 자체 보정을 더하지 않는다.
//     정지 조건도 바퀴중심 기준 radius_m x θ_rad다 (펜 기준 호 길이가 아니다)
//   - role 필드는 읽지 않는다 (관측용 메타데이터)
//
// 빌드: make robot_sim   (Server/ 디렉토리에서)
// 사용: ./robot_sim <서버IP> [server.crt경로] [옵션...]   (기본 crt: certs/server.crt)
//   --pen <m>          펜이 마커 중심 뒤로 떨어진 거리 (기본 0.155 = 로봇 실측)
//   --start <x,y,deg>  시작 pose (기본 -0.5,-0.5,0 - 도면 밖이라 접근 단계를 탄다)
//   --drift-dps <d>    주행 중 초당 d도씩 휘는 조향 오차 주입 (기본 0)
//   --slip <r>         주행 거리에 r 비율 오차 주입 (예 0.05 = 5% 덜 감. MORE 검증용)
//   --ignore-drift     서버 DRIFT 피드백을 무시 (기본은 반영)
//   --nozzle-ms <n>    노즐 액추에이터 동작 대기 (기본 1000 = 실측)
//   --pos-drop <s,d>   s초 뒤부터 d초간 POS 송신 중단 (HOLD 검증용)
//   --port <n>         서버 포트 (기본 9000)
//
// 종료(Ctrl+C 또는 도색 완료) 시 펜 자취 요약을 출력한다: 획마다 시작/끝 펜 좌표와
// 도색 길이. 이걸 도면 꼭짓점과 비교하면 "펜이 꼭짓점을 지나는가"를 로봇 없이 본다.
#include "tls_client.hpp"
#include <cmath>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static std::atomic<bool> g_run{true};
static void onSig(int) { g_run = false; }

// ----- 옵션 -----
static double optPen = 0.155;
static double optDriftDps = 0.0;
static double optSlip = 0.0;
static bool optIgnoreDrift = false;
static long optNozzleMs = 1000;
static double optStartX = -0.5, optStartY = -0.5, optStartDeg = 0.0;
static double optPosDropAt = -1, optPosDropDur = 0;

// 기본 속도. 실제 로봇 하드코딩값에 맞춘 러프 값.
static constexpr double kDefaultMps = 0.15;
static constexpr double kDefaultDps = 45.0;
static constexpr double kTickS = 0.05;  // 시뮬 한 틱 (20Hz)

static double normDeg(double a) {
    while (a > 180) a -= 360;
    while (a <= -180) a += 360;
    return a;
}
static long nowMs() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

// 한 획(노즐 내림 -> 올림) 동안의 펜 자취 요약
struct Stroke {
    double x0, y0, x1, y1;
    double len = 0;  // 펜이 실제로 지나간 길이 (호면 호 길이)
};

struct Sim {
    std::mutex mtx;
    double x = 0, y = 0, th = 0;  // th: rad, 바닥 +x축 기준 반시계 = "바라보는 방향"
    bool nozzleDown = false;

    json ops = json::array();
    std::string phase;
    size_t idx = 0;
    bool active = false;

    bool readySent = false;
    bool waitingGo = false;
    // 서버가 준 보정 잔량. 둘 다 "로봇 대면" 단위 (각도는 양수=오른쪽).
    double pendingAlignDeg = 0;
    double pendingMoreM = 0;
    double driftFeedbackDeg = 0;  // 서버가 마지막으로 알려준 각도 오차 (양수=오른쪽)
    bool held = false;            // HOLD{true} 수신 상태

    bool started = false;    // 현재 op의 실행이 시작됐는가
    double moveRemain = 0;   // 부호 있음 (음수 = 후진)
    double turnRemain = 0;   // 로봇 대면 부호 (양수 = 오른쪽)
    double arcRemainRad = 0, arcR = 0;
    int arcSign = 1;         // +1 = 왼쪽(CCW), -1 = 오른쪽(CW)
    long nozzleUntilMs = 0;  // 노즐 액추에이터 대기 종료 시각

    std::vector<Stroke> strokes;
    Stroke cur;
    double lastPenX = 0, lastPenY = 0;

    // 펜 위치 = 마커 중심에서 "바라보는 방향 반대"로 pen만큼
    void penPos(double& px, double& py) const {
        px = x - optPen * std::cos(th);
        py = y - optPen * std::sin(th);
    }
};

static Sim g;

static void startStroke() {
    g.penPos(g.cur.x0, g.cur.y0);
    g.cur.len = 0;
    g.lastPenX = g.cur.x0;
    g.lastPenY = g.cur.y0;
}
static void endStroke() {
    g.penPos(g.cur.x1, g.cur.y1);
    g.strokes.push_back(g.cur);
}
// 펜이 실제로 움직인 거리를 누적한다 (중심 이동거리가 아니라). 호 주행에서는
// 펜 반지름이 바퀴중심 반지름보다 커서 둘이 다르다 - 그 차이가 곧 검증 대상이다.
static void accumPen() {
    if (!g.nozzleDown) return;
    double px, py;
    g.penPos(px, py);
    g.cur.len += std::hypot(px - g.lastPenX, py - g.lastPenY);
    g.lastPenX = px;
    g.lastPenY = py;
}

// 새 PATH 수신: 기존 경로는 즉시 폐기한다 (protocol.hpp PATH 규약).
static void loadPath(const json& payload) {
    std::lock_guard<std::mutex> lk(g.mtx);
    g.ops = payload.value("ops", json::array());
    g.phase = payload.value("phase", "");
    g.idx = 0;
    g.active = !g.ops.empty();
    g.readySent = false;
    g.waitingGo = false;
    g.pendingAlignDeg = 0;
    g.pendingMoreM = 0;
    g.started = false;
    g.moveRemain = g.turnRemain = g.arcRemainRad = 0;
    // 이전 경로를 기준으로 계산된 DRIFT는 새 경로에 적용하면 안 된다.
    g.driftFeedbackDeg = 0;
    tlogf("ROBOT", "PATH 수신 phase=%s, %zu op - 실행 시작", g.phase.c_str(),
          g.ops.size());
}

// 제자리 회전 한 틱. deg는 로봇 대면 부호(양수 = 오른쪽 = th 감소).
// 남은 각도를 돌려준다.
static double spinTick(double remainDeg) {
    double step = std::min(std::fabs(remainDeg), kDefaultDps * kTickS);
    double sgn = remainDeg >= 0 ? 1.0 : -1.0;
    g.th -= sgn * step * M_PI / 180.0;  // 🔴 양수 = 오른쪽 = 시계방향 = th 감소
    accumPen();
    return remainDeg - sgn * step;
}

// 직진 한 틱. remain은 부호 있음(음수 = 후진). 남은 거리를 돌려준다.
static double driveTick(double remainM, bool feedback) {
    double step = std::min(std::fabs(remainM), kDefaultMps * kTickS);
    double sgn = remainM >= 0 ? 1.0 : -1.0;
    // slip: 실제로는 명령보다 덜(또는 더) 간다. 스텝 카운터는 다 셌다고 믿으므로
    // remain은 명령값 기준으로 줄고, 실제 위치만 (1-slip)배로 움직인다.
    g.x += sgn * step * (1.0 - optSlip) * std::cos(g.th);
    g.y += sgn * step * (1.0 - optSlip) * std::sin(g.th);
    if (feedback) {
        g.th += optDriftDps * kTickS * M_PI / 180.0;  // 조향 오차 주입
        if (!optIgnoreDrift && g.driftFeedbackDeg != 0) {
            // DRIFT는 "오른쪽으로 이만큼 보정하라"는 양. 한 틱에 다 돌리지 않고
            // 비례 제어로 조금씩 먹인다 (실제 로봇 제어루프와 같은 성격).
            double corr = g.driftFeedbackDeg * 0.2;
            g.th -= corr * M_PI / 180.0;
            g.driftFeedbackDeg -= corr;
        }
    }
    accumPen();
    return remainM - sgn * step;
}

// 시뮬 한 틱. PATH가 끝나면 donePhase에 phase를 담아 true를 반환한다
// (PATH_DONE 전송은 락 밖에서 하려고 분리 - send가 블로킹될 수 있음).
static bool tick(TlsLink& robot, std::string& donePhase) {
    std::lock_guard<std::mutex> lk(g.mtx);
    if (!g.active) return false;
    // HOLD 중에는 실행 중인 op 도중이라도 그 자리에서 멈춘다. op을 포기하지는
    // 않으므로 남은 거리/각도(moveRemain 등)는 그대로 들고 있는다.
    if (g.held) return false;

    if (g.idx >= g.ops.size()) {
        // 🔴 마지막 op을 마친 뒤에는 READY{N}을 보내지 않는다. PATH_DONE이
        //   그 자리를 대신한다 (§3.2).
        if (g.nozzleDown) { endStroke(); g.nozzleDown = false; }
        g.active = false;
        donePhase = g.phase;
        return true;
    }

    const json& op = g.ops[g.idx];
    const std::string o = op.value("op", "");

    // ----- 모든 op 앞에서 READY -> GO 핸드셰이크 (§3) -----
    if (!g.readySent) {
        g.readySent = true;
        g.waitingGo = true;
        robot.send("READY", {{"op_index", (int)g.idx}});
        tlogf("ROBOT", "[op %zu] READY 송신 (%s)", g.idx, o.c_str());
    }
    if (g.waitingGo) {
        // ALIGN/MORE를 수행 중이면 그것부터 끝낸다. 끝나면 같은 op_index로
        // READY를 다시 보낸다 - 서버는 그 READY에 또 응답을 하나 준다.
        if (g.pendingAlignDeg != 0) {
            g.pendingAlignDeg = spinTick(g.pendingAlignDeg);
            if (std::fabs(g.pendingAlignDeg) < 1e-6) {
                g.pendingAlignDeg = 0;
                robot.send("READY", {{"op_index", (int)g.idx}});
                tlogf("ROBOT", "[op %zu] ALIGN 완료 - READY 재송신", g.idx);
            }
        } else if (g.pendingMoreM != 0) {
            g.pendingMoreM = driveTick(g.pendingMoreM, /*feedback=*/false);
            if (std::fabs(g.pendingMoreM) < 1e-9) {
                g.pendingMoreM = 0;
                robot.send("READY", {{"op_index", (int)g.idx}});
                tlogf("ROBOT", "[op %zu] MORE 완료 - READY 재송신", g.idx);
            }
        }
        return false;  // GO를 기다리는 동안은 제자리
    }

    // ----- GO를 받았다 - op 실행 -----
    auto finishOp = [&] {
        g.started = false;
        g.readySent = false;
        g.driftFeedbackDeg = 0;  // 이 op용 피드백이므로 다음 op으로 넘기지 않는다
        ++g.idx;
    };

    if (o == "nozzle") {
        if (!g.started) {
            g.started = true;
            bool down = op.value("down", false);
            if (down && !g.nozzleDown) startStroke();
            if (!down && g.nozzleDown) endStroke();
            g.nozzleDown = down;
            // 액추에이터가 완전히 착지/상승할 때까지 정지 대기 (실측 약 1초)
            g.nozzleUntilMs = nowMs() + optNozzleMs;
            tlogf("ROBOT", "[op %zu] nozzle %s (%ldms 대기)", g.idx,
                  down ? "down" : "up", optNozzleMs);
        }
        if (nowMs() >= g.nozzleUntilMs) finishOp();
        return false;
    }

    if (o == "turn") {
        if (!g.started) {
            g.started = true;
            g.turnRemain = op.value("angle_deg", 0.0);
            tlogf("ROBOT", "[op %zu] turn %.1f도 (%s) 시작", g.idx, g.turnRemain,
                  g.turnRemain >= 0 ? "오른쪽" : "왼쪽");
        }
        g.turnRemain = spinTick(g.turnRemain);
        if (std::fabs(g.turnRemain) < 1e-6) { g.turnRemain = 0; finishOp(); }
        return false;
    }

    if (o == "move") {
        if (!g.started) {
            g.started = true;
            g.moveRemain = op.value("dist_m", 0.0);
            tlogf("ROBOT", "[op %zu] move %.3fm 시작", g.idx, g.moveRemain);
        }
        g.moveRemain = driveTick(g.moveRemain, /*feedback=*/true);
        if (std::fabs(g.moveRemain) < 1e-9) { g.moveRemain = 0; finishOp(); }
        return false;
    }

    if (o == "arc") {
        if (!g.started) {
            g.started = true;
            g.arcR = op.value("radius_m", 0.0);
            g.arcRemainRad = std::fabs(op.value("angle_deg", 0.0)) * M_PI / 180.0;
            g.arcSign = op.value("direction", "left") == "left" ? +1 : -1;
            // 🔴 자체 보정을 하지 않는다. radius_m은 서버가 이미 펜 오프셋을
            //   반영한 "바퀴 중심" 반지름이다. 여기서 sqrt(R²±d²)를 또 하면
            //   이중 보정이 된다 (§4.4 마이그레이션 #1).
            tlogf("ROBOT", "[op %zu] arc R=%.3fm %.1f도 %s 시작 "
                  "(도면 반지름 %.3fm)", g.idx, g.arcR,
                  g.arcRemainRad * 180 / M_PI,
                  g.arcSign > 0 ? "왼쪽" : "오른쪽",
                  op.value("radius_draw_m", 0.0));
        }
        // 좌우 바퀴 속도비를 고정한 채 도는 한 번의 연속 곡선 주행.
        // 정지 조건은 "바퀴중심 이동거리 = radius_m x θ_rad" (§4.4 #2).
        double dphi = g.arcR > 1e-6 ? (kDefaultMps * kTickS) / g.arcR
                                    : kDefaultDps * kTickS * M_PI / 180.0;
        dphi = std::min(dphi, g.arcRemainRad);
        // ICR은 좌우 바퀴 축선 위, 중심에서 안쪽으로 R. 그 점을 축으로 회전한다.
        double cx = g.x + g.arcSign * g.arcR * std::cos(g.th + M_PI / 2);
        double cy = g.y + g.arcSign * g.arcR * std::sin(g.th + M_PI / 2);
        double s = g.arcSign * dphi;
        double dx = g.x - cx, dy = g.y - cy;
        g.x = cx + dx * std::cos(s) - dy * std::sin(s);
        g.y = cy + dx * std::sin(s) + dy * std::cos(s);
        g.th += s;
        accumPen();
        g.arcRemainRad -= dphi;
        if (g.arcRemainRad < 1e-9) { g.arcRemainRad = 0; finishOp(); }
        return false;
    }

    tlogf("ROBOT", "[op %zu] ⚠️ 모르는 op '%s' - 건너뜀 "
          "(실제 로봇은 여기서 영구 정지할 수 있음)", g.idx, o.c_str());
    finishOp();
    return false;
}

static void printSummary() {
    std::lock_guard<std::mutex> lk(g.mtx);
    fprintf(stderr, "\n===== 펜 자취 요약 (펜 오프셋 %.3fm) =====\n", optPen);
    if (g.strokes.empty()) {
        fprintf(stderr, "  (도색 구간 없음)\n");
        return;
    }
    double total = 0;
    for (size_t i = 0; i < g.strokes.size(); ++i) {
        const Stroke& s = g.strokes[i];
        double straight = std::hypot(s.x1 - s.x0, s.y1 - s.y0);
        fprintf(stderr,
                "  획 %zu: (%.3f, %.3f) -> (%.3f, %.3f)  도색길이 %.3fm "
                "(직선거리 %.3fm)\n",
                i + 1, s.x0, s.y0, s.x1, s.y1, s.len, straight);
        total += s.len;
    }
    fprintf(stderr, "  총 도색 길이: %.3fm, 획 수: %zu\n", total, g.strokes.size());
    fprintf(stderr, "  ※ 위 좌표가 도면 꼭짓점과 일치해야 한다 "
                    "(마커 중심이 아니라 펜 기준)\n");
    fprintf(stderr, "  ※ 호를 그렸다면 도색길이/직선거리로 펜 반지름을 역산해 "
                    "도면 반지름과 대조할 것\n\n");
}

int main(int argc, char** argv) {
    signal(SIGINT, onSig);
    signal(SIGTERM, onSig);  // 스크립트에서 kill로 접을 때도 요약이 나오도록
    std::string ip = argc > 1 ? argv[1] : "127.0.0.1";
    std::string crt = "certs/server.crt";
    int port = 9000;
    int ai = 2;
    if (argc > 2 && argv[2][0] != '-') crt = argv[ai++];
    for (; ai < argc; ++ai) {
        std::string a = argv[ai];
        if (a == "--pen" && ai + 1 < argc) optPen = atof(argv[++ai]);
        else if (a == "--drift-dps" && ai + 1 < argc) optDriftDps = atof(argv[++ai]);
        else if (a == "--slip" && ai + 1 < argc) optSlip = atof(argv[++ai]);
        else if (a == "--ignore-drift") optIgnoreDrift = true;
        else if (a == "--nozzle-ms" && ai + 1 < argc) optNozzleMs = atol(argv[++ai]);
        else if (a == "--pos-drop" && ai + 1 < argc)
            sscanf(argv[++ai], "%lf,%lf", &optPosDropAt, &optPosDropDur);
        else if (a == "--start" && ai + 1 < argc)
            sscanf(argv[++ai], "%lf,%lf,%lf", &optStartX, &optStartY, &optStartDeg);
        else if (a == "--port" && ai + 1 < argc) port = atoi(argv[++ai]);
        else { fprintf(stderr, "알 수 없는 옵션: %s\n", a.c_str()); return 1; }
    }
    g.x = optStartX, g.y = optStartY, g.th = optStartDeg * M_PI / 180.0;

    TlsLink robot("ROBOT"), cctv("CCTV");
    if (!robot.connect(ip, port, crt, "ROBOT")) return 1;
    if (!cctv.connect(ip, port, crt, "CCTV")) return 1;
    tlogf("SIM", "시작 pose (%.3f, %.3f, %.1f도), 펜 오프셋 %.3fm%s%s",
          g.x, g.y, optStartDeg, optPen,
          optDriftDps != 0 ? " [조향오차 주입]" : "",
          optSlip != 0 ? " [거리오차 주입]" : "");

    // 서버 -> 로봇 수신 스레드
    std::thread rx([&] {
        json msg;
        while (g_run && robot.recv(msg)) {
            std::string type = msg.value("type", "?");
            json payload = msg.value("payload", json::object());
            if (type == "PATH") {
                loadPath(payload);
                continue;
            }
            if (type == "HOLD") {
                std::lock_guard<std::mutex> lk(g.mtx);
                g.held = payload.value("hold", false);
                tlogf("ROBOT", "HOLD %s 수신 (%s)", g.held ? "true" : "false",
                      payload.value("reason", "-").c_str());
                continue;
            }
            if (type == "GO" || type == "ALIGN" || type == "MORE" ||
                type == "DRIFT") {
                std::lock_guard<std::mutex> lk(g.mtx);
                // 🔴 자기가 기다리는 index와 다른 응답은 조용히 버린다 (§3.1).
                //   지연 도착한 이전 경로의 응답이 새 경로를 움직이는 것을 막는다.
                int k = payload.value("op_index", -1);
                if (!g.active || k != (int)g.idx) {
                    tlogf("ROBOT", "%s(op %d) 무시 - 현재 op %zu", type.c_str(), k,
                          g.idx);
                    continue;
                }
                if (type == "GO") {
                    g.waitingGo = false;
                    tlogf("ROBOT", "GO(op %d) 수신 - 실행", k);
                } else if (type == "ALIGN") {
                    g.pendingAlignDeg = payload.value("angle_deg", 0.0);
                    tlogf("ROBOT", "ALIGN(op %d) %.1f도 (%s) 수신", k,
                          g.pendingAlignDeg,
                          g.pendingAlignDeg >= 0 ? "오른쪽" : "왼쪽");
                } else if (type == "MORE") {
                    g.pendingMoreM = payload.value("dist_m", 0.0);
                    tlogf("ROBOT", "MORE(op %d) %.3fm (%s) 수신", k, g.pendingMoreM,
                          g.pendingMoreM >= 0 ? "전진" : "후진");
                } else {
                    g.driftFeedbackDeg = payload.value("angle_deg", 0.0);
                    tlogf("ROBOT", "DRIFT(op %d) %.1f도 (%s) 수신", k,
                          g.driftFeedbackDeg,
                          g.driftFeedbackDeg >= 0 ? "오른쪽" : "왼쪽");
                }
                continue;
            }
            if (type == "CMD")
                tlogf("ROBOT", "CMD %s 수신", payload.value("cmd", "?").c_str());
            else if (type != "ACK")
                tlogf("ROBOT", "수신 [%s] %s", type.c_str(), payload.dump().c_str());
        }
        g_run = false;
    });

    // 메인 루프: 시뮬 틱 + POS/STATUS 주기 송신
    const long t0 = nowMs();
    bool dropLogged = false;
    int posDiv = 0, statusDiv = 0;
    while (g_run) {
        std::string donePhase;
        if (tick(robot, donePhase)) {
            robot.send("PATH_DONE", {{"phase", donePhase}});
            tlogf("ROBOT", "PATH_DONE(%s) 송신", donePhase.c_str());
            // 도색이 끝난 시점의 펜 자취를 바로 찍는다 - 이게 이 도구의 결과물이고,
            // Ctrl+C를 기다리게 하면 스크립트로 돌릴 때 못 받는다.
            if (donePhase == "draw") printSummary();
        }
        // POS 10Hz - 서버가 로봇 위치를 아는 유일한 경로
        if (++posDiv >= 2) {
            posDiv = 0;
            double elapsed = (nowMs() - t0) / 1000.0;
            bool drop = optPosDropAt >= 0 && elapsed >= optPosDropAt &&
                        elapsed < optPosDropAt + optPosDropDur;
            if (drop && !dropLogged) {
                tlogf("CCTV", "POS 송신 중단 %.1f초 (HOLD 검증)", optPosDropDur);
                dropLogged = true;
            }
            if (!drop) {
                std::lock_guard<std::mutex> lk(g.mtx);
                cctv.send("POS", {{"x", g.x}, {"y", g.y},
                                  {"theta_deg", normDeg(g.th * 180.0 / M_PI)}});
            }
        }
        // STATUS 2Hz - 하트비트 겸 Qt 모니터링용
        if (++statusDiv >= 10) {
            statusDiv = 0;
            std::lock_guard<std::mutex> lk(g.mtx);
            robot.send("STATUS", {{"state", g.active ? "MOVING" : "IDLE"},
                                  {"painting", g.nozzleDown}});
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds((int)(kTickS * 1000)));
    }

    robot.shutdownRead();
    cctv.shutdownRead();
    if (rx.joinable()) rx.join();
    printSummary();
    return 0;
}
