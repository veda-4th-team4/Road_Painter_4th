// 로봇 + CCTV 대역 시뮬레이터 (서버 알고리즘 드라이런용) - TCP/TLS
//
// 목적: 실제 로봇 없이 서버의 주행 알고리즘 전체를 닫힌 루프로 돌려본다.
//   서버가 보낸 PATH를 실행 -> 그 결과 위치를 POS로 되돌려줌 -> 서버가 그걸 보고
//   ALIGN/DRIFT/복귀 PATH를 다시 보냄. 즉 "서버가 보낸 대로 움직인 결과"가 다시
//   서버 입력이 되므로, path_test처럼 한 번 찍고 끝나는 게 아니라 실시간 피드백과
//   이탈 재계획까지 검증된다.
//
// 왜 한 프로세스가 ROBOT과 CCTV 두 role로 붙나:
//   서버는 "로봇이 어디 있는지"를 오직 CCTV의 POS로만 안다. 시뮬레이터가 자기
//   위치를 POS로 흘려넣지 않으면 서버는 로봇이 움직이는 걸 볼 수 없다.
//   POS는 {"x","y","theta_deg"} 형식(서버 poseFromPos의 테스트 경로)으로 보내
//   캘리브레이션 없이도 돌아가게 한다 - 좌표 변환은 이 도구의 검증 대상이 아니다.
//
// 🔴 실제 로봇과 다른 점 (일부러 다르게 둔 것):
//   이 시뮬은 "프로토콜대로 동작하는 이상적인 로봇"이다. 2026-07-28 기준 실제
//   rpi-robot 코드는 PATH_DONE을 안 보내고, bypass_server_go=true라 GO를 안
//   기다리며, NOZZLE op에서 영구 정지한다. 그 상태를 재현해보려면 --bypass-go를
//   쓰면 된다. 이 도구의 목적은 "서버가 맞게 짜였나"를 로봇 수정과 무관하게
//   먼저 확정하는 것이다.
//
// 빌드: make robot_sim   (Server/ 디렉토리에서)
// 사용: ./robot_sim <서버IP> [server.crt경로] [옵션...]   (기본 crt: certs/server.crt)
//   --pen <m>          펜이 마커 중심 뒤로 떨어진 거리 (기본 0.155 = 로봇 실측)
//   --start <x,y,deg>  시작 pose (기본 -0.5,-0.5,0 - 도면 밖이라 접근 단계를 탄다)
//   --drift-dps <d>    주행 중 초당 d도씩 휘는 조향 오차 주입 (이탈 재현용, 기본 0)
//   --ignore-drift     서버 DRIFT 피드백을 무시 (기본은 반영)
//   --bypass-go        READY만 보내고 GO를 안 기다림 (현재 실제 로봇 재현)
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
static bool optIgnoreDrift = false;
static bool optBypassGo = false;
static double optStartX = -0.5, optStartY = -0.5, optStartDeg = 0.0;

// 기본 속도 (op에 speed가 없을 때). 실제 로봇 하드코딩값에 맞춘 러프 값.
static constexpr double kDefaultMps = 0.15;
// ⚠️ 45도/s는 실제 로봇의 물리적 상한(17.8도/s - router.hpp kPoseGateRateDps 주석의
// 계산 참고)보다 빠르다. 그래서 드라이런을 돌리면 TURN 구간마다 서버의 POS 이상치
// 게이트(허용 3도 + 40도/s)가 걸려 "[WARN] POS 이상치 ... 폐기 n/5"가 줄줄이 찍히고
// 5프레임마다 재동기된다 - 서버 버그가 아니라 이 상수가 비물리적이라는 뜻이다.
// 게이트까지 같이 검증하려면 17.8 이하로 내려서 돌릴 것 (그만큼 드라이런이 느려진다).
static constexpr double kDefaultDps = 45.0;
static constexpr double kTickS = 0.05;  // 시뮬 한 틱 (20Hz)

static double normDeg(double a) {
    while (a > 180) a -= 360;
    while (a <= -180) a += 360;
    return a;
}

// 한 획(노즐 내림 -> 올림) 동안의 펜 자취 요약
struct Stroke {
    double x0, y0, x1, y1;
    double len = 0;
};

struct Sim {
    std::mutex mtx;
    double x = 0, y = 0, th = 0;  // th: rad, 바닥 +x축 기준 반시계 = "바라보는 방향"
    bool nozzleDown = false;

    json segs = json::array();
    std::string phase;
    size_t idx = 0;
    bool active = false;

    bool readySent = false;
    bool waitingGo = false;
    double pendingAlignDeg = 0;  // ALIGN으로 받은 제자리 회전 잔량
    double driftFeedbackDeg = 0; // 서버가 마지막으로 알려준 각도 오차

    bool moveStarted = false;
    double moveRemain = 0;  // 부호 있음 (음수 = 후진)
    double turnRemain = 0;

    std::vector<Stroke> strokes;
    Stroke cur;

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
}
static void endStroke() {
    g.penPos(g.cur.x1, g.cur.y1);
    g.strokes.push_back(g.cur);
}

// 새 PATH 수신: 기존 경로는 즉시 폐기한다 (protocol.hpp PATH 규약).
static void loadPath(const json& payload) {
    std::lock_guard<std::mutex> lk(g.mtx);
    g.segs = payload.value("segments", json::array());
    g.phase = payload.value("phase", "");
    g.idx = 0;
    g.active = !g.segs.empty();
    g.readySent = false;
    g.waitingGo = false;
    g.pendingAlignDeg = 0;
    g.moveStarted = false;
    g.moveRemain = 0;
    g.turnRemain = 0;
    // 이전 경로를 기준으로 계산된 DRIFT는 새 경로에 적용하면 안 된다.
    // (안 지우면: 접근 마지막 TURN 동안 쌓인 "45도 되돌아가라"가 도색 첫 동작에
    //  그대로 먹혀 로봇이 엉뚱한 방향으로 휘어 나간다 - 실제로 겪었다)
    g.driftFeedbackDeg = 0;
    tlogf("ROBOT", "PATH 수신 phase=%s, %zu 동작 - 실행 시작",
          g.phase.c_str(), g.segs.size());
}

// 시뮬 한 틱. PATH가 끝나면 donePhase에 phase를 담아 true를 반환한다
// (PATH_DONE 전송은 락 밖에서 하려고 분리 - sendTo가 블로킹될 수 있음).
static bool tick(TlsLink& robot, std::string& donePhase) {
    std::lock_guard<std::mutex> lk(g.mtx);
    if (!g.active) return false;

    if (g.idx >= g.segs.size()) {
        if (g.nozzleDown) { endStroke(); g.nozzleDown = false; }
        g.active = false;
        donePhase = g.phase;
        return true;
    }

    const json& op = g.segs[g.idx];
    std::string o = op.value("op", "");

    if (o == "NOZZLE") {
        bool down = op.value("down", false);
        if (down && !g.nozzleDown) startStroke();
        if (!down && g.nozzleDown) endStroke();
        g.nozzleDown = down;
        tlogf("ROBOT", "[seg %zu] NOZZLE %s", g.idx, down ? "down" : "up");
        ++g.idx;
        return false;
    }

    if (o == "TURN") {
        if (g.turnRemain == 0 && !g.moveStarted) {
            g.turnRemain = op.value("angle_deg", 0.0);
            g.moveStarted = true;  // "이 TURN을 시작했다" 표시로 재사용
            tlogf("ROBOT", "[seg %zu] TURN %.1f도 시작", g.idx, g.turnRemain);
        }
        double dps = op.value("speed_dps", kDefaultDps);
        double step = std::min(std::fabs(g.turnRemain), dps * kTickS);
        double sgn = g.turnRemain >= 0 ? 1.0 : -1.0;
        g.th += sgn * step * M_PI / 180.0;
        g.turnRemain -= sgn * step;
        if (std::fabs(g.turnRemain) < 1e-6) {
            g.turnRemain = 0;
            g.moveStarted = false;
            ++g.idx;
        }
        return false;
    }

    if (o != "MOVE") {  // 모르는 op - 실제 로봇은 여기서 영구 정지한다
        tlogf("ROBOT", "[seg %zu] ⚠️ 모르는 op '%s' - 건너뜀 "
              "(실제 로봇은 여기서 영구 정지함)", g.idx, o.c_str());
        ++g.idx;
        return false;
    }

    // ----- MOVE: 출발 전 정렬 핸드셰이크 -----
    if (!g.readySent) {
        g.readySent = true;
        g.waitingGo = !optBypassGo;
        robot.send("READY", {{"seg", (int)g.idx}});
        if (optBypassGo)
            tlogf("ROBOT", "[seg %zu] READY 송신 (--bypass-go: GO 안 기다림)", g.idx);
    }
    if (g.waitingGo) {
        if (g.pendingAlignDeg != 0) {  // ALIGN 미세회전 수행 중
            double step = std::min(std::fabs(g.pendingAlignDeg), kDefaultDps * kTickS);
            double sgn = g.pendingAlignDeg >= 0 ? 1.0 : -1.0;
            g.th += sgn * step * M_PI / 180.0;
            g.pendingAlignDeg -= sgn * step;
            if (std::fabs(g.pendingAlignDeg) < 1e-6) {
                g.pendingAlignDeg = 0;
                robot.send("READY", {{"seg", (int)g.idx}});  // 정렬 후 재확인
            }
        }
        return false;  // GO를 기다리는 동안은 제자리
    }

    if (!g.moveStarted) {
        g.moveRemain = op.value("dist_m", 0.0);
        g.moveStarted = true;
        tlogf("ROBOT", "[seg %zu] MOVE %.3fm (paint=%s) 시작", g.idx, g.moveRemain,
              op.value("paint", false) ? "true" : "false");
    }
    double mps = op.value("speed_mps", kDefaultMps);
    double step = std::min(std::fabs(g.moveRemain), mps * kTickS);
    double sgn = g.moveRemain >= 0 ? 1.0 : -1.0;
    // 전진/후진 모두 "바라보는 방향" 기준. 후진해도 th는 안 바뀐다.
    g.x += sgn * step * std::cos(g.th);
    g.y += sgn * step * std::sin(g.th);
    if (g.nozzleDown) g.cur.len += step;
    // 조향 오차 주입 (이탈 재현용) + 서버 DRIFT 피드백 반영
    g.th += optDriftDps * kTickS * M_PI / 180.0;
    if (!optIgnoreDrift && g.driftFeedbackDeg != 0) {
        // DRIFT는 "좌회전으로 이만큼 보정하라"는 양. 한 틱에 다 돌리지 않고
        // 비례 제어로 조금씩 먹인다 (실제 로봇 제어루프와 같은 성격).
        double corr = g.driftFeedbackDeg * 0.2;
        g.th += corr * M_PI / 180.0;
        g.driftFeedbackDeg -= corr;
    }
    g.moveRemain -= sgn * step;
    if (std::fabs(g.moveRemain) < 1e-9) {
        g.moveStarted = false;
        g.readySent = false;
        g.driftFeedbackDeg = 0;  // 이 MOVE용 피드백이므로 다음 동작으로 넘기지 않는다
        ++g.idx;
    }
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
                    "(마커 중심이 아니라 펜 기준)\n\n");
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
        else if (a == "--ignore-drift") optIgnoreDrift = true;
        else if (a == "--bypass-go") optBypassGo = true;
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
          optBypassGo ? " [GO 무시]" : "");

    // 서버 -> 로봇 수신 스레드
    std::thread rx([&] {
        json msg;
        while (g_run && robot.recv(msg)) {
            std::string type = msg.value("type", "?");
            json payload = msg.value("payload", json::object());
            if (type == "PATH") {
                loadPath(payload);
            } else if (type == "GO") {
                std::lock_guard<std::mutex> lk(g.mtx);
                g.waitingGo = false;
                tlogf("ROBOT", "GO 수신 - 출발");
            } else if (type == "ALIGN") {
                std::lock_guard<std::mutex> lk(g.mtx);
                g.pendingAlignDeg = payload.value("angle_deg", 0.0);
                tlogf("ROBOT", "ALIGN %.1f도 수신 - 미세회전", g.pendingAlignDeg);
            } else if (type == "DRIFT") {
                std::lock_guard<std::mutex> lk(g.mtx);
                g.driftFeedbackDeg = payload.value("angle_deg", 0.0);
            } else if (type == "CMD") {
                tlogf("ROBOT", "CMD %s 수신",
                      payload.value("cmd", "?").c_str());
            } else if (type != "ACK") {
                tlogf("ROBOT", "수신 [%s] %s", type.c_str(), payload.dump().c_str());
            }
        }
        g_run = false;
    });

    // 메인 루프: 시뮬 틱 + POS/STATUS 주기 송신
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
            std::lock_guard<std::mutex> lk(g.mtx);
            cctv.send("POS", {{"x", g.x}, {"y", g.y},
                              {"theta_deg", normDeg(g.th * 180.0 / M_PI)}});
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
