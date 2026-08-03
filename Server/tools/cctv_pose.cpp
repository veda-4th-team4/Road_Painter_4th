// CCTV 대역 pose 주입기 - 로봇의 시작 위치를 파일에서 읽어 POS로 쏜다.
//
// 왜 필요한가: 서버는 로봇이 어디 있는지를 오직 CCTV의 POS로만 안다. CCTV가 ROI
//   추적으로 옮겨가는 중이라 실시간 POS가 아직 안 나오는 동안, "시작 위치 한 번"만
//   넣어주면 접근 -> 도색 흐름 전체가 개루프로 돈다:
//     - START_DRAW는 pose가 한 번이라도 잡혀야 진행된다 (router.cpp handleStartDraw)
//     - 한 번 잡힌 pose는 만료되지 않는다 (poseValid_는 켜지기만 하고 꺼지는 곳이 없음)
//     - DRIFT/이탈 재계획은 POS 핸들러 안에만 있어서, POS가 더 안 오면 자연히 안 돈다
//     - READY 정렬은 낡은 pose로 판정되지만 kAlignMaxTries(4)회 후 무조건 GO라 안 멈춘다
//
// 🔴 전제: Qt가 program을 실어 BLUEPRINT를 보내야 한다. program이 없으면 서버가
//   도색 경로를 "낡은 출발 위치" 기준으로 만들어(router.cpp sendDrawPath 폴백)
//   로봇이 이미 시작점에 가 있는 것과 어긋난다.
//
// ⚠️ 순서: Qt가 먼저 로그인해야 한다. 서버는 캘리브레이션을 로그인 시점에 계정에서
//   불러오므로, 그 전에 POS를 쏘면 "캘리브레이션 없음 - pose 계산 불가"로 버려진다.
//   그래서 이 도구는 파일을 매번 다시 읽고 Enter로 재전송할 수 있게 돼 있다.
//
// 빌드: make cctv_pose   (Server/ 디렉토리에서)
// 사용: ./tools/cctv_pose <서버IP> [server.crt경로] [옵션...]   (기본 crt: certs/server.crt)
//   --file <경로>    pose 파일 (기본 tools/cctv_pose.json)
//   --repeat <초>    N초마다 자동 재전송 (기본 0 = Enter 칠 때만)
//   --port <n>       서버 포트 (기본 9000)
#include "tls_client.hpp"
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <thread>

static std::atomic<bool> g_run{true};
static void onSig(int) { g_run = false; }

// pose 파일을 매번 새로 읽는다 - 도구를 띄운 채로 파일만 고쳐 다시 쏠 수 있게.
// 파일 내용이 곧 POS 페이로드다(서버가 모르는 키는 무시하므로 _note 같은 건 그대로 둬도 됨).
static bool loadPose(const std::string& path, json& out) {
    std::ifstream f(path);
    if (!f) {
        tlogf("CCTV", "파일 열기 실패: %s", path.c_str());
        return false;
    }
    std::string txt((std::istreambuf_iterator<char>(f)),
                    std::istreambuf_iterator<char>());
    out = json::parse(txt, nullptr, false);
    if (out.is_discarded() || !out.is_object()) {
        tlogf("CCTV", "JSON 파싱 실패: %s", path.c_str());
        return false;
    }
    // 서버 poseFromPos가 받는 두 형식 중 하나여야 한다 (path_planner.hpp 참고)
    bool hasXy = out.contains("x") && out.contains("y");
    bool hasCorners = out.contains("corners") && out["corners"].is_array() &&
                      out["corners"].size() == 4;
    if (!hasXy && !hasCorners) {
        tlogf("CCTV", "corners(4개) 또는 x/y 중 하나는 있어야 함: %s", path.c_str());
        return false;
    }
    return true;
}

// 보낸 값을 사람이 확인할 수 있게 한 줄로 요약
static void logSent(const json& p) {
    if (p.contains("x") && p.contains("y")) {
        tlogf("CCTV", "POS 전송 (직접 좌표) x=%.3f y=%.3f theta=%.1f",
              p["x"].get<double>(), p["y"].get<double>(),
              p.value("theta_deg", 0.0));
    } else {
        const json& c = p["corners"];
        tlogf("CCTV", "POS 전송 (원본 픽셀 4점) [%.1f,%.1f] [%.1f,%.1f] [%.1f,%.1f] [%.1f,%.1f]",
              c[0][0].get<double>(), c[0][1].get<double>(),
              c[1][0].get<double>(), c[1][1].get<double>(),
              c[2][0].get<double>(), c[2][1].get<double>(),
              c[3][0].get<double>(), c[3][1].get<double>());
    }
}

int main(int argc, char** argv) {
    signal(SIGINT, onSig);
    std::string ip = argc > 1 ? argv[1] : "127.0.0.1";
    std::string crt = "certs/server.crt";
    std::string file = "tools/cctv_pose.json";
    int port = 9000, repeatSec = 0;

    int ai = 2;
    if (argc > 2 && argv[2][0] != '-') crt = argv[ai++];
    for (; ai < argc; ++ai) {
        std::string a = argv[ai];
        if (a == "--file" && ai + 1 < argc) file = argv[++ai];
        else if (a == "--repeat" && ai + 1 < argc) repeatSec = atoi(argv[++ai]);
        else if (a == "--port" && ai + 1 < argc) port = atoi(argv[++ai]);
        else { fprintf(stderr, "알 수 없는 옵션: %s\n", a.c_str()); return 1; }
    }

    json pose;
    if (!loadPose(file, pose)) return 1;  // 접속 전에 파일부터 검증

    TlsLink cctv("CCTV");
    if (!cctv.connect(ip, port, crt, "CCTV")) return 1;

    // 서버가 CCTV로 내려보내는 것(CALIB_START 등)을 그냥 찍어준다.
    // 연결이 끊기면 루프를 세운다.
    std::thread rx([&] {
        json msg;
        while (g_run && cctv.recv(msg))
            tlogf("CCTV", "수신 %s", msg.value("type", "?").c_str());
        g_run = false;
    });

    cctv.send("POS", pose);
    logSent(pose);
    tlogf("CCTV", "⚠️ Qt 로그인 전에 쏘면 서버가 캘리브레이션이 없어 버린다 - "
                  "서버 로그에 'pose 계산 불가'가 뜨면 로그인 후 다시 쏠 것");

    if (repeatSec > 0) {
        tlogf("CCTV", "%d초마다 자동 재전송 (Ctrl+C 종료) - 파일 수정은 즉시 반영됨",
              repeatSec);
        while (g_run) {
            for (int i = 0; i < repeatSec && g_run; ++i)
                std::this_thread::sleep_for(std::chrono::seconds(1));
            if (!g_run) break;
            if (loadPose(file, pose)) {  // 실패하면 직전 값 유지하고 계속
                cctv.send("POS", pose);
                logSent(pose);
            }
        }
    } else {
        tlogf("CCTV", "Enter = %s 다시 읽어서 재전송 / Ctrl+C = 종료", file.c_str());
        std::string line;
        while (g_run && std::getline(std::cin, line)) {
            if (!loadPose(file, pose)) continue;  // 고칠 때까지 다시 Enter
            cctv.send("POS", pose);
            logSent(pose);
        }
    }

    g_run = false;
    cctv.shutdownRead();
    rx.join();
    tlogf("CCTV", "연결 종료");
    return 0;
}
