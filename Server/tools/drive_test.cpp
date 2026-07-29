// 로봇 주행 단독 테스트기 (접근·CCTV·도면 없이 제자리 사각형) - TCP/TLS
//
// 목적: 로봇 하나만 놓고 "받은 동작 시퀀스를 제대로 실행하는가"를 본다.
//   서버의 경로생성·pose 판정을 전혀 타지 않으므로 CCTV도 BLUEPRINT도 필요 없다.
//   로봇이 지금 서 있는 자리에서 곧바로 사각형을 그린다(접근 단계 없음).
//
// 어떻게 접근을 건너뛰나: ADMIN role로 붙어 PATH를 보낸다. 서버는 ADMIN의 PATH를
//   "점검·설치용"으로 보고 도면/pose 검사 없이 로봇에 그대로 중계한다
//   (src/router.cpp fromAdmin). START_DRAW 흐름(접근 -> 도색)을 아예 안 타므로
//   pose가 없어도 no_pose로 막히지 않는다.
//
// path_test와 뭐가 다른가: path_test는 CCTV 스냅샷(H+corners)을 넣어 "서버가"
//   경로를 만들게 하는 도구라 캘리브레이션 자료가 있어야 한다. 이 도구는 동작
//   시퀀스를 직접 만들어 넣으므로 로봇과 서버만 있으면 된다.
//
// 기본 도형: 한 변 0.30m 정사각형. 변마다 노즐을 내렸다 올린다(획 4개).
//   NOZZLE down -> MOVE(paint) -> NOZZLE up -> TURN  x4
//   변마다 노즐을 끊는 이유: NOZZLE op이 노즐 제어의 단일 결정권이라
//   (server_PROTOCOL.md "수신: PATH"), 내리고 올리는 동작이 실제로 먹는지가
//   이 테스트의 핵심이다. 한붓그리기로 보고 싶으면 --continuous.
//
// ⚠️ 방향은 "로봇 자기 기준"이다. 로봇은 phase="draw" PATH를 받는 순간 IMU 현재
//   방향을 0도로 세팅하므로, 첫 변은 항상 "지금 바라보는 쪽"으로 나간다. CCTV가
//   없어 서버가 정렬 판정을 못 하니(READY에 그냥 GO), 실제 각도는 로봇 IMU 정확도에
//   전적으로 달려 있다 - 그걸 재는 게 이 테스트의 목적이기도 하다.
//
// 빌드: make drive_test   (Server/ 디렉토리에서)
// 사용: ./drive_test <서버IP> [server.crt경로] [옵션...]   (기본 crt: certs/server.crt)
//   --side <m>      한 변 길이 (기본 0.30)
//   --sides <n>     변 개수 (기본 4 = 사각형. 2면 왕복선, 1이면 직선 한 줄)
//   --turn <deg>    꼭짓점 회전각 (기본 90, 양수 = 좌회전)
//   --continuous    한붓그리기 - 노즐을 처음 한 번만 내리고 끝에 올린다
//   --no-paint      전부 paint=false (노즐 안 내리고 동선만 확인)
//   --print-only    보내지 않고 만들어질 시퀀스만 출력
//   --port <n>      서버 포트 (기본 9000)
#include "tls_client.hpp"
#include <csignal>
#include <cstdlib>
#include <string>
#include <thread>

static std::atomic<bool> g_run{true};
static void onSig(int) { g_run = false; }

// (-180, 180] 범위로 정규화 (서버 path_planner.hpp normDeg와 동일 규약)
static double normDeg(double a) {
    while (a > 180) a -= 360;
    while (a <= -180) a += 360;
    return a;
}

int main(int argc, char** argv) {
    signal(SIGINT, onSig);
    std::string ip = argc > 1 ? argv[1] : "127.0.0.1";
    std::string crt = "certs/server.crt";
    int port = 9000;
    double side = 0.30, turnDeg = 90.0;
    int sides = 4;
    bool continuous = false, noPaint = false, printOnly = false;

    int ai = 2;
    if (argc > 2 && argv[2][0] != '-') crt = argv[ai++];
    for (; ai < argc; ++ai) {
        std::string a = argv[ai];
        if (a == "--side" && ai + 1 < argc) side = atof(argv[++ai]);
        else if (a == "--sides" && ai + 1 < argc) sides = atoi(argv[++ai]);
        else if (a == "--turn" && ai + 1 < argc) turnDeg = atof(argv[++ai]);
        else if (a == "--continuous") continuous = true;
        else if (a == "--no-paint") noPaint = true;
        else if (a == "--print-only") printOnly = true;
        else if (a == "--port" && ai + 1 < argc) port = atoi(argv[++ai]);
        else { fprintf(stderr, "알 수 없는 옵션: %s\n", a.c_str()); return 1; }
    }
    if (sides < 1 || side <= 0) {
        fprintf(stderr, "--sides는 1 이상, --side는 0보다 커야 한다\n");
        return 1;
    }
    bool paint = !noPaint;

    // ----- 동작 시퀀스 생성 -----
    // heading_deg는 "이 동작 후 로봇이 바라보는 절대 방위"다. 로봇이 PATH 수신
    // 시점을 0도로 잡으므로 여기서도 0에서 시작해 회전각을 누적한다.
    json segs = json::array();
    double heading = 0.0;
    if (paint && continuous) segs.push_back({{"op", "NOZZLE"}, {"down", true}});
    for (int i = 0; i < sides; ++i) {
        if (paint && !continuous)
            segs.push_back({{"op", "NOZZLE"}, {"down", true}});
        segs.push_back({{"op", "MOVE"},
                        {"dist_m", side},
                        {"paint", paint},
                        {"heading_deg", std::round(heading * 10) / 10}});
        if (paint && !continuous)
            segs.push_back({{"op", "NOZZLE"}, {"down", false}});
        // 마지막 변에서도 돈다 - 닫힌 도형이면 출발 자세로 되돌아와야
        // 같은 명령을 반복 실행해도 제자리에 머문다.
        heading = normDeg(heading + turnDeg);
        segs.push_back({{"op", "TURN"},
                        {"angle_deg", turnDeg},
                        {"heading_deg", std::round(heading * 10) / 10}});
    }
    if (paint && continuous) segs.push_back({{"op", "NOZZLE"}, {"down", false}});

    tlogf("DRIVE", "한 변 %.3fm x %d변, 회전 %.1f도, %s%s", side, sides, turnDeg,
          !paint ? "도색 없음(동선만)" : (continuous ? "한붓그리기" : "변마다 노즐 업다운"),
          printOnly ? " [전송 안 함]" : "");
    tlogf("DRIVE", "총 %zu 동작 (이동거리 %.2fm, 총 회전 %.0f도)", segs.size(),
          side * sides, turnDeg * sides);

    if (printOnly) {
        printf("%s\n", json{{"phase", "draw"}, {"segments", segs}}.dump(2).c_str());
        return 0;
    }

    // ----- ADMIN으로 붙어 PATH 중계 -----
    // ADMIN은 서버가 중계하는 모든 메시지 사본(TAP)도 받으므로, 로봇이 보내는
    // STATUS/READY/PATH_DONE과 서버가 내려주는 GO/ALIGN을 여기서 그대로 볼 수 있다.
    TlsLink admin("DRIVE");
    if (!admin.connect(ip, port, crt, "ADMIN")) return 1;

    std::thread rx([&] {
        json msg;
        while (g_run && admin.recv(msg)) {
            std::string type = msg.value("type", "");
            if (type != "TAP") continue;
            json p = msg.value("payload", json::object());
            json inner = p.value("msg", json::object());
            std::string t = inner.value("type", "");
            std::string peer = p.value("peer", "");
            // POS/POSE/DRIFT는 CCTV가 붙어 있을 때 초당 수십 개씩 흘러 로그를
            // 통째로 덮는다. 이 도구의 관심사는 "로봇이 op을 순서대로 실행하는가"라
            // 위치 스트림은 버린다 (robot_sim을 짝으로 띄운 경우에만 나온다).
            if (t == "POS" || t == "POSE" || t == "DRIFT") continue;
            // STATUS는 2초마다 오는 하트비트라 로그를 덮어버린다 - 상태가
            // 바뀌는 순간만 남긴다.
            if (t == "STATUS") {
                static std::string last;
                std::string cur = inner["payload"].dump();
                if (cur == last) continue;
                last = cur;
            }
            // PATH는 통째로 찍으면 한 줄이 수백 자라, 동작 개수만 요약한다.
            std::string body = t == "PATH"
                ? std::to_string(inner["payload"].value("segments", json::array()).size()) + "개 동작"
                : inner.value("payload", json::object()).dump();
            tlogf("TAP", "%s %-5s %s %s", p.value("dir", "") == "IN" ? "->" : "<-",
                  peer.c_str(), t.c_str(), body.c_str());
            if (t == "PATH_DONE") {
                tlogf("DRIVE", "로봇이 경로를 끝냈다 (PATH_DONE) - 종료");
                g_run = false;
                admin.shutdownRead();
            }
        }
        g_run = false;
    });

    // 로봇이 붙어 있는지 서버가 알려줄 방법이 없으므로(PEERS는 QT 전용) 잠깐 기다린 뒤
    // 그냥 보낸다. 로봇이 없으면 서버 콘솔에 "ROBOT 미접속" WARN이 뜬다.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (!admin.send("PATH", {{"phase", "draw"}, {"segments", segs}})) {
        tlogf("DRIVE", "PATH 전송 실패");
        g_run = false;
    } else {
        tlogf("DRIVE", "PATH 전송 완료 - 로봇 실행 대기 (Ctrl+C로 중단)");
    }

    while (g_run) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    admin.shutdownRead();
    if (rx.joinable()) rx.join();
    return 0;
}
