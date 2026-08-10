// Road-Painter 중앙 서버 (서버 RPi에서 실행) - TCP/TLS
//
// 파일 구성:
//   main.cpp       - 시작/조립 + 테스트용 콘솔
//   tls_server.*   - TLS 네트워크 레이어 (접속, role 등록, 송수신)
//   router.*       - 메시지 라우팅 로직 (QT/ROBOT/CCTV)
//   protocol.hpp   - 메시지 생성 헬퍼
//
// 실행 전: ./gen_cert.sh <서버IP> 로 certs/server.crt, server.key 생성
// 콘솔 명령: path / estop / resume / calib / who / quit
#include "log.hpp"
#include "params.hpp"
#include "protocol.hpp"
#include "router.hpp"
#include "tls_server.hpp"
#include <atomic>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <thread>

// 전역 상태: 안전한 종료 신호
static std::atomic<bool> gShutdown(false);
static std::unique_ptr<TlsServer> gpServer;

// 종료 신호 핸들러 (Ctrl+C, kill -TERM)
static void signalHandler(int sig) {
    logf("[INFO] 종료 신호 수신 (sig=%d)", sig);
    gShutdown = true;
    // accept() 루프를 깨우기 위해 리스닝 소켓을 종료
    if (gpServer) gpServer->shutdown();
}

int main(int argc, char** argv) {
    try {
        // 종료 신호 등록
        std::signal(SIGINT, signalHandler);   // Ctrl+C
        std::signal(SIGTERM, signalHandler);  // kill -TERM

        // 포트는 인자로 덮어쓸 수 있다(기본 9000). 운영 중인 서버를 건드리지 않고
        // robot_sim/draw_test 드라이런을 옆 포트에 따로 띄우기 위한 것.
        int port = argc > 1 ? std::atoi(argv[1]) : 9000;
        if (port <= 0 || port > 65535) {
            logf("[WARN] 잘못된 포트 '%s' - 기본 9000 사용", argv[1]);
            port = 9000;
        }
        logf("[INFO] Road-Painter TLS 서버 시작 (포트 %d)", port);
        // 튜닝 파라미터(임계값/주기/기하)를 파일에서 읽는다. 코드에 박아두지
        // 않는 이유는 현장에서 값 하나 바꾸려고 재컴파일하지 않기 위해서다.
        // 경로는 두번째 인자로 덮어쓸 수 있다 (드라이런용 대체 설정 등).
        // 스레드가 뜨기 전에 한 번만 로드하고, 이후로는 읽기 전용이다.
        loadParams(argc > 2 ? argv[2] : "config/params.json");
        gpServer = std::make_unique<TlsServer>(port, "certs/server.crt", "certs/server.key");
        Router router(*gpServer);
        gpServer->setHandler([&](const std::string& role, const json& msg) {
            router.onMessage(role, msg);
        });
        gpServer->setPeerHandler([&](const std::string& role, bool connected) {
            router.onPeerChange(role, connected);
        });
        // 서버 내부 로그(logf)를 관리자 창(ADMIN)에도 흘려보낸다 - 웹 로그
        // 모니터가 tap뿐 아니라 서버 처리 로그(도면 수신/send 실패 등)까지
        // 보게 한다. 스레드 시작 전에 등록해 데이터 레이스를 피한다.
        // (싱크는 relayLogToAdmin만 호출하며 logf를 다시 부르지 않음 - 재귀 방지)
        logSink() = [](const std::string& line) {
            if (gpServer) gpServer->relayLogToAdmin(line);
        };

        // 네트워크 스레드 시작 (srv.run()은 블로킹)
        std::thread netThread([&] { gpServer->run(); });

        // 시간 기반 판정 스레드. 로봇 STATUS(500ms)를 heartbeat로 삼던 것을
        // 대체한다 - 상대가 조용해져도 서버의 감시는 계속 돌아야 한다
        // (Router::tick 주석 참고). 200ms면 판정 지연이 체감되지 않고
        // mtx_ 경합도 무시할 수준이다.
        std::thread tickThread([&] {
            while (!gShutdown) {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                router.tick();
            }
        });

        // ----- 임시 테스트용 콘솔 (별도 스레드) -----
        // std::getline(std::cin, ...)은 시그널로 깨울 수 없는 블로킹 호출이라
        // 메인 스레드에서 직접 돌리면 Ctrl+C/kill을 받고도 Enter를 한 번 더
        // 쳐야만 실제로 종료되는 문제가 생긴다 (실측 확인됨). 그래서 별도
        // 스레드로 분리해 detach하고, 메인 스레드는 콘솔을 기다리지 않는다.
        // 종료 시 이 스레드는 프로세스 종료와 함께 그냥 버려진다.
        std::thread consoleThread([&] {
            std::string cmd;
            while (!gShutdown && std::getline(std::cin, cmd)) {
                if (cmd == "path") {
                    // 테스트 경로 (프로토콜 v2 ops): 2m 직진 도색 -> 우회전 90도
                    // -> 1m 직진 도색. 오프셋 보정 op까지 손으로 적어둔 것이라
                    // 서버가 생성하는 경로와 형태가 같다.
                    // ⚠️ 이 경로는 Router를 거치지 않아 서버가 activeMeta_를
                    //   모른다 - READY에는 GO만 나가고 ALIGN/MORE/DRIFT는 없다.
                    json ops = json::parse(R"([
                        {"op":"move","role":"offset","dist_m":0.155,"op_index":0},
                        {"op":"nozzle","role":"offset","down":true,"op_index":1},
                        {"op":"move","role":"path","dist_m":2.0,"op_index":2},
                        {"op":"nozzle","role":"offset","down":false,"op_index":3},
                        {"op":"move","role":"offset","dist_m":-0.155,"op_index":4},
                        {"op":"turn","role":"path","angle_deg":90.0,"op_index":5},
                        {"op":"move","role":"offset","dist_m":0.155,"op_index":6},
                        {"op":"nozzle","role":"offset","down":true,"op_index":7},
                        {"op":"move","role":"path","dist_m":1.0,"op_index":8},
                        {"op":"nozzle","role":"offset","down":false,"op_index":9},
                        {"op":"move","role":"offset","dist_m":-0.155,"op_index":10}
                    ])");
                    bool ok = gpServer->sendTo("ROBOT", makePathMsg(ops, "draw"));
                    logf("[INFO] PATH 전송 %s", ok ? "성공" : "실패");
                } else if (cmd == "estop") {
                    gpServer->sendTo("ROBOT", makeMsg("CMD", {{"cmd", "ESTOP"}}));
                    logf("[INFO] ESTOP 전송");
                } else if (cmd == "resume") {
                    gpServer->sendTo("ROBOT", makeMsg("CMD", {{"cmd", "RESUME"}}));
                    logf("[INFO] RESUME 전송");
                } else if (cmd == "calib" || cmd.rfind("calib ", 0) == 0) {
                    // 🔴 Router를 거쳐 보낸다. 예전처럼 sendTo로 직접 쏘면 서버가
                    // 모르는 캘리 세션이 시작돼, 그 결과 H_MATRIX가 아무도
                    // 기다리지 않는 종결 응답으로 Qt에 떨어진다. ADMIN 개시로
                    // 취급되므로 검증·busy·타임아웃이 전부 똑같이 걸린다.
                    // 사용법: "calib" (활성 채널) 또는 "calib 3" (채널 지정)
                    int ch = 0;
                    if (cmd.size() > 6) ch = std::atoi(cmd.c_str() + 6);
                    json p{{"cmd", "CALIB_START"}, {"method", "robot_motion"}};
                    if (validChannel(ch)) p["ch"] = ch;
                    router.onMessage("ADMIN", makeMsg("CMD", p));
                } else if (cmd == "who") {
                    std::string s;
                    for (auto& r : gpServer->connectedRoles()) s += r + " ";
                    logf("[INFO] 접속 중: %s", s.empty() ? "없음" : s.c_str());
                } else if (cmd == "quit") {
                    logf("[INFO] 종료 명령 수신");
                    gShutdown = true;
                    break;
                } else if (!cmd.empty()) {
                    logf("[INFO] 명령어: path / estop / resume / calib / who / quit");
                }
            }
            logf("[INFO] 콘솔 입력 종료");
        });
        consoleThread.detach();

        // 신호(Ctrl+C/kill)만으로 즉시 반응 - 콘솔 입력을 기다리지 않는다
        while (!gShutdown) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        logf("[INFO] 네트워크 스레드 정리 중...");
        // 콘솔 'quit' 경로는 시그널 핸들러를 거치지 않아 여기서 직접 리스닝
        // 소켓을 닫아야 run()의 accept 루프가 끝난다 (안 하면 join이 영원히
        // 안 돌아옴). 시그널 경로에서 이미 닫았으면 no-op (내부에서 처리).
        gpServer->shutdown();
        netThread.join();  // 스레드 정상 종료 대기
        tickThread.join();  // router를 참조하므로 router보다 먼저 끝나야 한다

        logf("[INFO] 서버 정상 종료");
        // return 대신 즉시 종료. 이유: detach된 콘솔 스레드가 getline(std::cin)에서
        // stdin 잠금을 쥔 채 대기 중이라, 정상 return 시 프로세스 종료 절차의 스트림
        // 정리가 그 잠금을 기다리며 데드락 → Enter를 쳐야만 종료되는 버그가 생긴다
        // (실측 확인). _Exit는 그 정리를 건너뛰고 바로 끝낸다. 이 시점엔 스레드 join,
        // 소켓 정리, 로그 flush, users.json 저장이 모두 끝나 있어 잃을 데이터 없음.
        std::_Exit(0);
    } catch (const std::exception& e) {
        logf("[ERROR] %s", e.what());
        return 1;
    }
}
