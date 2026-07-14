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
#include <cstdlib>
#include <iostream>
#include <thread>

int main() {
    try {
        TlsServer srv(9000, "certs/server.crt", "certs/server.key");
        Router router(srv);
        srv.setHandler([&](const std::string& role, const json& msg) {
            router.onMessage(role, msg);
        });

        std::thread netThread([&] { srv.run(); });
        netThread.detach();

        // ----- 임시 테스트용 콘솔 -----
        std::string cmd;
        while (std::getline(std::cin, cmd)) {
            if (cmd == "path") {
                // 테스트 경로: 2m 직진(도색) -> 우회전 90도 -> 1m 직진(도색)
                json segs = json::parse(R"([
                    {"op":"MOVE","dist_m":2.0,"paint":true},
                    {"op":"TURN","angle_deg":-90},
                    {"op":"MOVE","dist_m":1.0,"paint":true}
                ])");
                bool ok = srv.sendTo("ROBOT", makePathMsg(segs));
                logf("[INFO] PATH 전송 %s", ok ? "성공" : "실패");
            } else if (cmd == "estop") {
                srv.sendTo("ROBOT", makeMsg("CMD", {{"cmd", "ESTOP"}}));
            } else if (cmd == "resume") {
                srv.sendTo("ROBOT", makeMsg("CMD", {{"cmd", "RESUME"}}));
            } else if (cmd == "calib") {
                json m = makeMsg("CMD", {{"cmd", "CALIB_START"}});
                srv.sendTo("CCTV", m);
                srv.sendTo("ROBOT", m);
            } else if (cmd == "who") {
                std::string s;
                for (auto& r : srv.connectedRoles()) s += r + " ";
                logf("[INFO] 접속 중: %s", s.empty() ? "없음" : s.c_str());
            } else if (cmd == "quit") {
                logf("[INFO] 서버 종료");
                std::exit(0);
            } else if (!cmd.empty()) {
                logf("[INFO] 명령어: path / estop / resume / calib / who / quit");
            }
        }
        // stdin이 닫혀도 (백그라운드 실행) 서버는 계속 동작
        for (;;) std::this_thread::sleep_for(std::chrono::hours(1));
    } catch (const std::exception& e) {
        logf("[ERROR] %s", e.what());
        return 1;
    }
}
