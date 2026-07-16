// Qt 쪽 임시 클라이언트 (테스트용 시뮬레이터) - TCP/TLS
// 실제 Qt(QSslSocket) 네트워킹 코드 나오기 전까지 서버와 주고받기 테스트용.
//
// 동작:
//   - 서버에 TLS 접속 (server.crt 로 서버 검증) 후 HELLO(role=QT)
//   - 콘솔 명령으로 REGISTER/LOGIN/CMD/BLUEPRINT 전송
//   - 백그라운드 스레드가 서버가 보내는 모든 메시지
//     (ACK/REGISTER_OK/LOGIN_OK/H_MATRIX/POS/STATUS 등)를 그대로 로그 출력
//
// 빌드: make qt_sim   (Server/ 디렉토리에서)
// 사용: ./qt_sim <서버IP> [server.crt경로]   (기본 crt: ../certs/server.crt)
//
// 콘솔 명령:
//   register <id> <pw>
//   login <id> <pw>
//   cmd estop|resume|calib
//   blueprint              (하드코딩 테스트 도면 전송: [0,0]->[2,0]->[2,1])
//   quit
#include <nlohmann/json.hpp>
#include <openssl/err.h>
#include <openssl/ssl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

using json = nlohmann::json;

static void logf(const char* fmt, ...) {
    static std::mutex m;
    std::lock_guard<std::mutex> lk(m);
    char ts[16];
    time_t t = time(nullptr);
    struct tm tmv;
    localtime_r(&t, &tmv);
    strftime(ts, sizeof(ts), "%H:%M:%S", &tmv);
    fprintf(stderr, "%s [QT] ", ts);
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fprintf(stderr, "\n");
    fflush(stderr);
}

static std::atomic<long> g_seq{0};

static json makeMsg(const std::string& type, const json& payload) {
    return json{{"type", type}, {"seq", ++g_seq}, {"payload", payload}};
}

static bool sslSendLine(SSL* ssl, std::mutex& wmtx, const json& msg) {
    std::string data = msg.dump() + "\n";
    std::lock_guard<std::mutex> lk(wmtx);
    return SSL_write(ssl, data.data(), (int)data.size()) > 0;
}

static bool sslReadLine(SSL* ssl, std::string& buf, std::string& line) {
    for (;;) {
        auto pos = buf.find('\n');
        if (pos != std::string::npos) {
            line = buf.substr(0, pos);
            buf.erase(0, pos + 1);
            return true;
        }
        char tmp[4096];
        int n = SSL_read(ssl, tmp, sizeof(tmp));
        if (n <= 0) return false;
        buf.append(tmp, (size_t)n);
    }
}

// 서버가 보내는 메시지를 계속 읽어서 로그로 출력하는 스레드
static void recvLoop(SSL* ssl, std::atomic<bool>& alive) {
    std::string buf, line;
    while (alive && sslReadLine(ssl, buf, line)) {
        if (line.empty()) continue;
        json msg = json::parse(line, nullptr, false);
        if (msg.is_discarded()) {
            logf("JSON 파싱 실패: %s", line.c_str());
            continue;
        }
        std::string type = msg.value("type", "?");
        json payload = msg.value("payload", json::object());
        logf("수신 [%s] %s", type.c_str(), payload.dump().c_str());
    }
    alive = false;
    logf("서버 연결 종료됨 (읽기 실패) - 프로그램을 재시작하세요");
}

int main(int argc, char** argv) {
    std::string serverIp = argc > 1 ? argv[1] : "127.0.0.1";
    std::string caFile = argc > 2 ? argv[2] : "../certs/server.crt";
    const int port = 9000;

    SSL_CTX* ctx = SSL_CTX_new(TLS_client_method());
    SSL_CTX_set_min_proto_version(ctx, TLS1_2_VERSION);
    // 서버 자체서명 인증서를 신뢰 CA로 지정 (인증서 피닝)
    if (SSL_CTX_load_verify_locations(ctx, caFile.c_str(), nullptr) != 1) {
        logf("server.crt 로드 실패: %s (서버에서 복사해왔는지 확인)", caFile.c_str());
        return 1;
    }
    SSL_CTX_set_verify(ctx, SSL_VERIFY_PEER, nullptr);

    int fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, serverIp.c_str(), &addr.sin_addr);

    if (connect(fd, (sockaddr*)&addr, sizeof(addr)) != 0) {
        logf("서버 접속 실패 (%s:%d) - IP/포트 확인", serverIp.c_str(), port);
        return 1;
    }
    SSL* ssl = SSL_new(ctx);
    SSL_set_fd(ssl, fd);
    if (SSL_connect(ssl) != 1) {
        logf("TLS 핸드셰이크 실패 (인증서 확인)");
        ERR_print_errors_fp(stderr);
        return 1;
    }
    logf("서버 TLS 접속 성공 %s:%d", serverIp.c_str(), port);

    std::mutex wmtx;
    sslSendLine(ssl, wmtx, makeMsg("HELLO", {{"role", "QT"}}));

    std::atomic<bool> alive{true};
    std::thread rx(recvLoop, ssl, std::ref(alive));

    logf("명령어: register <id> <pw> / login <id> <pw> / cmd estop|resume|calib / blueprint / quit");
    std::string lineIn;
    while (alive && std::getline(std::cin, lineIn)) {
        std::istringstream iss(lineIn);
        std::string cmd;
        iss >> cmd;

        if (cmd == "register" || cmd == "login") {
            std::string id, pw;
            iss >> id >> pw;
            if (id.empty() || pw.empty()) {
                logf("사용법: %s <id> <pw>", cmd.c_str());
                continue;
            }
            std::string type = (cmd == "register") ? "REGISTER" : "LOGIN";
            sslSendLine(ssl, wmtx, makeMsg(type, {{"id", id}, {"pw", pw}}));
        } else if (cmd == "cmd") {
            std::string sub;
            iss >> sub;
            std::string c = sub == "estop"   ? "ESTOP"
                            : sub == "resume" ? "RESUME"
                            : sub == "calib"  ? "CALIB_START"
                                              : "";
            if (c.empty()) {
                logf("사용법: cmd estop|resume|calib");
                continue;
            }
            sslSendLine(ssl, wmtx, makeMsg("CMD", {{"cmd", c}}));
        } else if (cmd == "blueprint") {
            json pts = json::array({{0.0, 0.0}, {2.0, 0.0}, {2.0, 1.0}});
            sslSendLine(ssl, wmtx, makeMsg("BLUEPRINT", {{"points", pts}}));
            logf("테스트 도면 전송: [0,0]->[2,0]->[2,1]");
        } else if (cmd == "quit") {
            break;
        } else if (!cmd.empty()) {
            logf("명령어: register / login / cmd / blueprint / quit");
        }
    }

    alive = false;
    shutdown(fd, SHUT_RDWR);  // recvLoop의 SSL_read 깨우기
    rx.join();
    SSL_shutdown(ssl);
    SSL_free(ssl);
    close(fd);
    SSL_CTX_free(ctx);
    return 0;
}
