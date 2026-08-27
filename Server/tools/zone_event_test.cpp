// CCTV ZONE_EVENT가 기존 TLS 프로토콜로 ROBOT에 정확히 중계되는지 검증한다.
// 서버를 먼저 띄운 뒤 실행:
//   ./tools/zone_event_test 127.0.0.1 19000 /tmp/zone-event-test/certs/server.crt
#include "tls_client.hpp"
#include <cstdio>
#include <cstdlib>

static bool expectAck(TlsLink& link, const char* role) {
    json msg;
    if (!link.recv(msg)) return false;
    return msg.value("type", "") == "ACK" &&
           msg.value("payload", json::object()).value("msg", "") ==
               std::string("registered as ") + role;
}

int main(int argc, char** argv) {
    SSL_library_init();
    SSL_load_error_strings();

    const std::string ip = argc > 1 ? argv[1] : "127.0.0.1";
    const int port = argc > 2 ? std::atoi(argv[2]) : 19000;
    const std::string ca = argc > 3 ? argv[3] : "certs/server.crt";

    TlsLink robot("ROBOT"), cctv("CCTV");
    if (!robot.connect(ip, port, ca, "ROBOT") || !expectAck(robot, "ROBOT")) {
        std::fprintf(stderr, "FAIL: ROBOT TLS 등록 실패\n");
        return 1;
    }
    if (!cctv.connect(ip, port, ca, "CCTV") || !expectAck(cctv, "CCTV")) {
        std::fprintf(stderr, "FAIL: CCTV TLS 등록 실패\n");
        return 1;
    }

    // 아래 세 메시지가 하나라도 중계되면 ROBOT이 Enter보다 먼저 그것을 받으므로 실패한다.
    cctv.send("ZONE_EVENT", "not-an-object");
    cctv.send("ZONE_EVENT", {{"action", 1}, {"ch", 2}});
    cctv.send("ZONE_EVENT", {{"action", "Exit"}, {"ch", 3}});
    cctv.send("ZONE_EVENT", {{"action", "Enter"}, {"ch", 4},
                              {"object_id", "zone-test"}});

    json got;
    if (!robot.recv(got)) {
        std::fprintf(stderr, "FAIL: ROBOT ZONE_EVENT 수신 실패\n");
        return 1;
    }
    const json payload = got.value("payload", json::object());
    const bool ok = got.value("type", "") == "ZONE_EVENT" && payload.is_object() &&
                    payload.value("action", "") == "Enter" &&
                    payload.value("ch", 0) == 4 &&
                    payload.value("object_id", "") == "zone-test";
    if (!ok) {
        std::fprintf(stderr, "FAIL: 예상하지 않은 메시지: %s\n", got.dump().c_str());
        return 1;
    }

    std::printf("PASS: CCTV Enter만 기존 TLS 세션으로 ROBOT에 중계됨\n");
    return 0;
}
