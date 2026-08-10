// Qt 대역 도색 테스트기 (사각형 도면 + 동작 시퀀스 전송) - TCP/TLS
//
// 목적: Qt 클라이언트가 아직 program을 못 보내는 동안, Qt가 보낼 것과 똑같은
//   BLUEPRINT(points + paint + program)를 만들어 서버에 넣고 START_DRAW까지
//   눌러본다. robot_sim과 짝으로 돌리면 로봇도 Qt도 없이 접근 -> 도색 ->
//   실시간 피드백 -> 완료 전 과정을 볼 수 있다.
//
// qt_sim과 뭐가 다른가: qt_sim의 blueprint는 하드코딩 3점(points만)이라
//   도색 구간 지정도 program 중계도 검증할 수 없다.
//
// program은 도면 그대로의 논리 동작만 담는다(2026-07-28 최종 확정) - 펜 오프셋
// 보정(꼭짓점 후진/회전/재전진), pen_offset_m, 속도(speed_mps/speed_dps),
// pivot 플래그는 전부 여기 없다. 그건 로봇이 TURN을 실행할 때 스스로 하는
// 하드웨어 영역이고, Qt/서버는 "어디서 어디까지, 칠할지 말지, 몇 m·몇 도"만
// 다룬다 (router.hpp Router::buildRecovery 주석 참고 - 서버도 이 값들을 모른
// 채로 program을 그대로 중계한다).
//
// 빌드: make draw_test   (Server/ 디렉토리에서)
// 사용: ./draw_test <서버IP> [server.crt경로] [옵션...]   (기본 crt: certs/server.crt)
//   --side <m>      사각형 한 변 (기본 0.20)
//   --origin <x,y>  사각형 시작 꼭짓점 (기본 0,0)
//   --id / --pw     로그인 계정 (기본 test / 1234)
//   --no-program    program 없이 points+paint만 전송 (서버 직접 생성 폴백 검증)
//   --arc <R>       사각형 대신 반지름 R의 반원(ARC) 하나를 도색 (기본 끄기)
//                   -> 서버가 R_robot = sqrt(R^2 - 0.155^2)로 바꿔 보내는지,
//                      그 결과 펜이 반지름 R의 호를 그리는지 검증
//                      (docs/PROTOCOL_v2_ROBOT.md §5.4 / §11 남은항목 1)
//   --port <n>      서버 포트 (기본 9000)
#include "tls_client.hpp"
#include <cmath>
#include <chrono>
#include <csignal>
#include <thread>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static std::atomic<bool> g_run{true};
static void onSig(int) { g_run = false; }

int main(int argc, char** argv) {
    signal(SIGINT, onSig);
    std::string ip = argc > 1 ? argv[1] : "127.0.0.1";
    std::string crt = "certs/server.crt";
    int port = 9000;
    double side = 0.20, ox = 0, oy = 0;
    std::string id = "test", pw = "1234";
    bool noProgram = false;
    double arcR = 0;  // > 0 이면 사각형 대신 반원 도색
    // --nudge: BLUEPRINT_OK 직후 START_DRAW 전에 조이스틱 명령을 한 번 끼운다.
    // 현장에서 가장 흔한 순서("도면 올리고 → 로봇 위치 잡고 → 시작")를 재현한다.
    // 🔴 이 순서가 자동 판정(ALIGN/MORE/DRIFT)을 죽이던 버그의 회귀 확인용이다:
    //   서버 로그에 GO 사유 "수동 모드"가 한 번이라도 찍히면 재발한 것이다.
    bool nudge = false;

    int ai = 2;
    if (argc > 2 && argv[2][0] != '-') crt = argv[ai++];
    for (; ai < argc; ++ai) {
        std::string a = argv[ai];
        if (a == "--side" && ai + 1 < argc) side = atof(argv[++ai]);
        else if (a == "--origin" && ai + 1 < argc)
            sscanf(argv[++ai], "%lf,%lf", &ox, &oy);
        else if (a == "--id" && ai + 1 < argc) id = argv[++ai];
        else if (a == "--pw" && ai + 1 < argc) pw = argv[++ai];
        else if (a == "--no-program") noProgram = true;
        else if (a == "--arc" && ai + 1 < argc) arcR = atof(argv[++ai]);
        else if (a == "--port" && ai + 1 < argc) port = atoi(argv[++ai]);
        else if (a == "--nudge") nudge = true;
        else { fprintf(stderr, "알 수 없는 옵션: %s\n", a.c_str()); return 1; }
    }

    // ----- 사각형 도면: 반시계로 한 바퀴 돌아 시작점으로 닫는다 (점 5개) -----
    // points는 "펜이 지나갈 자취"다. 로봇 중심 경로가 아니다.
    std::vector<std::array<double, 2>> v = {
        {ox, oy}, {ox + side, oy}, {ox + side, oy + side},
        {ox, oy + side}, {ox, oy}};
    // 구간 i의 진행 방위 (v[i] -> v[i+1])
    double head[4] = {0.0, 90.0, 180.0, -90.0};

    json points = json::array();
    for (auto& p : v) points.push_back({p[0], p[1]});
    // paint[i] = v[i-1] -> v[i] 구간을 칠하는가. paint[0]은 대응 구간이 없어 무시.
    json paint = json::array({false, true, true, true, true});

    // ----- 동작 시퀀스: 한붓그리기(노즐 한 번 내렸다 끝에 올림) -----
    // v 필드 = 이 op가 "출발하는" 도면 꼭짓점 index. 서버 buildRecovery가
    // "꼭짓점 k로 복귀 후 재개할 op"을 v >= k인 첫 op으로 찾는다.
    json program = json::array();
    if (!noProgram) {
        program.push_back({{"op", "NOZZLE"}, {"v", 0}, {"down", true}});
        for (int i = 0; i < 4; ++i) {
            program.push_back({{"op", "MOVE"}, {"v", i}, {"dist_m", side},
                               {"paint", true}, {"heading_deg", head[i]}});
            if (i < 3)
                program.push_back({{"op", "TURN"}, {"v", i + 1},
                                   {"angle_deg", 90.0},
                                   {"heading_deg", head[i + 1]}});
        }
        program.push_back({{"op", "NOZZLE"}, {"v", 4}, {"down", false}});
    }

    // ----- 반원 도면 (--arc): 원점에서 출발해 오른쪽으로 180도 돈다 -----
    // 펜은 (ox,oy)에서 (ox, oy-2R)까지 반지름 R의 반원을 그려야 한다.
    // 서버가 radius_m을 sqrt(R^2 - d^2)로 바꿔 보내는지, 그 결과 펜 자취의
    // 반지름이 R로 나오는지가 검증 대상이다 (§5.4).
    if (arcR > 0) {
        v = {{ox, oy}, {ox, oy - 2 * arcR}};
        points = json::array();
        for (auto& p : v) points.push_back({p[0], p[1]});
        paint = json::array({false, true});
        program = json::array();
        if (!noProgram) {
            program.push_back({{"op", "ARC"},
                               {"v", 0},
                               {"radius_m", arcR},
                               {"angle_deg", 180.0},
                               {"direction", "right"},
                               {"paint", true},
                               // heading_deg = 호에 "진입할 때"의 접선.
                               // (0,0)에서 동쪽(0도)으로 들어가 우회전 180도를
                               // 쓸면 (0,-2R)에 서쪽(180도)을 보고 도착한다 -
                               // 출구 접선은 서버가 스윕을 더해 만든다.
                               {"heading_deg", 0.0}});
        }
    }

    TlsLink qt("QT");
    if (!qt.connect(ip, port, crt, "QT")) return 1;

    if (arcR > 0) {
        tlogf("QT", "반원 도면: 반지름 %.3fm, 시작 (%.3f, %.3f) -> (%.3f, %.3f)",
              arcR, v[0][0], v[0][1], v[1][0], v[1][1]);
        tlogf("QT", "기대: 서버가 로봇에 radius_m=%.4f 전송, 펜 자취 반지름 %.3f "
              "(호 길이 %.3fm)",
              std::sqrt(arcR * arcR - 0.155 * 0.155), arcR, M_PI * arcR);
    } else {
        tlogf("QT", "사각형 한 변 %.3fm, 시작 (%.3f, %.3f)%s", side, ox, oy,
              noProgram ? " [program 없음 - 서버 생성 폴백]" : "");
        tlogf("QT", "기대 펜 자취(도면 꼭짓점): (%.3f,%.3f) (%.3f,%.3f) (%.3f,%.3f) "
              "(%.3f,%.3f) -> 시작점 복귀",
              v[0][0], v[0][1], v[1][0], v[1][1], v[2][0], v[2][1], v[3][0], v[3][1]);
    }

    // 수신 스레드: 서버 응답을 그대로 찍고, 진행 단계를 자동으로 이어간다
    std::thread rx([&] {
        json msg;
        while (g_run && qt.recv(msg)) {
            std::string type = msg.value("type", "?");
            json payload = msg.value("payload", json::object());
            if (type == "POSE") continue;  // 10Hz라 로그가 뒤덮인다 - 생략
            tlogf("QT", "수신 [%s] %s", type.c_str(), payload.dump().c_str());

            if (type == "LOGIN_OK") {
                json bp{{"points", points}, {"paint", paint}};
                if (!program.empty()) bp["program"] = program;
                qt.send("BLUEPRINT", bp);
                tlogf("QT", "BLUEPRINT 송신 (점 %zu, 동작 %zu)",
                      points.size(), program.size());
            } else if (type == "BLUEPRINT_OK") {
                // 서버가 받은 개수가 보낸 것과 같은지 여기서 바로 대조된다
                if (nudge) {
                    qt.send("CMD", {{"cmd", "FORWARD"}});
                    qt.send("CMD", {{"cmd", "STOP"}});
                    tlogf("QT", "[--nudge] 시작 전 조이스틱 조작 1회 - 서버가 "
                                "수동 모드로 들어간다");
                }
                qt.send("CMD", {{"cmd", "START_DRAW"}});
                tlogf("QT", "CMD START_DRAW 송신 - 접근 단계 시작 대기");
            } else if (type == "DRAW_DONE") {
                tlogf("QT", "✅ 도색 완료 - robot_sim의 펜 자취 요약과 "
                            "위 기대 꼭짓점을 비교할 것");
                g_run = false;
            } else if (type == "DRAW_FAIL") {
                tlogf("QT", "❌ 실패 stage=%s reason=%s",
                      payload.value("stage", "?").c_str(),
                      payload.value("reason", "?").c_str());
            }
        }
        g_run = false;
    });

    qt.send("LOGIN", {{"id", id}, {"pw", pw}});
    tlogf("QT", "LOGIN 송신 (%s)", id.c_str());

    while (g_run) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    qt.shutdownRead();
    if (rx.joinable()) rx.join();
    return 0;
}
