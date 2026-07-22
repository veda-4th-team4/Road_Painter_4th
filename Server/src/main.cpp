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

int main() {
    try {
        // 종료 신호 등록
        std::signal(SIGINT, signalHandler);   // Ctrl+C
        std::signal(SIGTERM, signalHandler);  // kill -TERM

        logf("[INFO] Road-Painter TLS 서버 시작");
        gpServer = std::make_unique<TlsServer>(9000, "certs/server.crt", "certs/server.key");
        Router router(*gpServer);
        gpServer->setHandler([&](const std::string& role, const json& msg) {
            router.onMessage(role, msg);
        });
        gpServer->setPeerHandler([&](const std::string& role, bool connected) {
            router.onPeerChange(role, connected);
        });

        // 네트워크 스레드 시작 (srv.run()은 블로킹)
        std::thread netThread([&] { gpServer->run(); });

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
                    // 테스트 경로: 2m 직진(도색) -> 우회전 90도 -> 1m 직진(도색)
                    json segs = json::parse(R"([
                        {"op":"MOVE","dist_m":2.0,"paint":true},
                        {"op":"TURN","angle_deg":-90},
                        {"op":"MOVE","dist_m":1.0,"paint":true}
                    ])");
                    bool ok = gpServer->sendTo("ROBOT", makePathMsg(segs, "draw"));
                    logf("[INFO] PATH 전송 %s", ok ? "성공" : "실패");
                } else if (cmd == "estop") {
                    gpServer->sendTo("ROBOT", makeMsg("CMD", {{"cmd", "ESTOP"}}));
                    logf("[INFO] ESTOP 전송");
                } else if (cmd == "resume") {
                    gpServer->sendTo("ROBOT", makeMsg("CMD", {{"cmd", "RESUME"}}));
                    logf("[INFO] RESUME 전송");
                } else if (cmd == "calib") {
                    json m = makeMsg("CMD", {{"cmd", "CALIB_START"}});
                    gpServer->sendTo("CCTV", m);
                    gpServer->sendTo("ROBOT", m);
                    logf("[INFO] CALIB_START 전송");
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
