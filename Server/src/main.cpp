// 서버 RPi 진입점 (뼈대).
// 지금 하는 일: TLS 서버를 열고 → 로봇(클라이언트) 접속을 받고
//   → 예시 PATH 1회 전송 → 예시 POSE를 주기적으로 전송하며 → STATUS 수신 출력.
// 이후 채울 것: 실제 측위피드 수신(pose_builder), Qt 작도 수신(path_builder), 캘리브레이션 로드.
//
// 빌드: 최상위에서  cmake -B build && cmake --build build   → ./build/server_rpi
// 실행 전: certs/server.crt, certs/server.key 준비(openssl로 생성).

#include "net/tls_server.hpp"
#include "path_builder.hpp"
#include "messages.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

int main() {
    const int PORT = 8443;
    TlsServer srv;
    if (!srv.start("certs/server.crt", "certs/server.key", PORT)) {
        std::cerr << "서버 시작 실패 (인증서 확인)\n";
        return 1;
    }

    // 예시 PATH(주차칸 ㄷ자) — 실제로는 Qt 작도 + H_floor 변환 결과로 대체
    std::vector<Segment> segs = {
        {{0,0},{0,2.0},true}, {{0,2.0},{2.5,2.0},true}, {{2.5,2.0},{2.5,0},true},
    };
    auto path = PathBuilder(0.05).build(segs);

    while (true) {                       // 로봇 재접속 대응
        SSL* cli = srv.acceptClient();
        if (!cli) continue;

        // 1) 접속 직후 PATH 1회 전송
        TlsServer::writeLine(cli, makePath(path, 1));
        std::cout << "[TX] PATH " << path.size() << " waypoints\n";

        // 2) POSE 주기 전송 루프 (지금은 데모 값; 실제론 pose_builder 출력)
        long seq = 2;
        auto t0 = std::chrono::steady_clock::now();
        while (true) {
            double t = std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - t0).count();
            Pose demo; demo.x = 0.3; demo.y = t * 0.1; demo.theta = M_PI/2;
            demo.t = t; demo.conf = 1.0;
            if (!TlsServer::writeLine(cli, makePose(demo, seq++))) break;  // 끊김

            // (선택) 로봇 STATUS가 오면 읽어서 출력 — 여기선 단순화
            std::this_thread::sleep_for(std::chrono::milliseconds(80));    // ~12Hz
        }
        std::cout << "[TLS] 연결 종료, 재대기\n";
        TlsServer::closeClient(cli);
    }
    return 0;
}
