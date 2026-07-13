// 로봇 RPi 쪽 임시 클라이언트 (테스트용 시뮬레이터) - TCP/TLS
// 실제 로봇 코드 나오기 전까지 서버와 주고받기 테스트용.
//
// 동작:
//   - 서버에 TLS 접속 (server.crt 로 서버 검증) 후 HELLO(role=ROBOT)
//   - 1초마다 STATUS 송신 (가짜 위치/배터리)
//   - PATH 수신 -> MOVING / CMD: ESTOP->ESTOP, RESUME->MOVING, CALIB_START->CALIB
//   - POS 수신 -> 위치 보정값 로그 (실제론 센서퓨전에 사용)
//   - 끊기면 3초 후 자동 재접속
//
// 빌드: make
// 사용: ./robot_sim <서버IP> [server.crt경로]   (기본 crt: ./server.crt)
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
#include <mutex>
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
    fprintf(stderr, "%s [ROBOT] ", ts);
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fprintf(stderr, "\n");
    fflush(stderr);
}

// ---------- 로봇 상태 (가짜) ----------
struct RobotState {
    std::mutex mtx;
    std::string state = "IDLE";
    double x = 0, y = 0;
    int battery = 100;
    json path;
};

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

// 1초마다 STATUS 송신 + MOVING이면 가짜 이동
static void statusLoop(SSL* ssl, std::mutex& wmtx, RobotState& rs,
                       std::atomic<bool>& alive) {
    while (alive) {
        json st;
        {
            std::lock_guard<std::mutex> lk(rs.mtx);
            if (rs.state == "MOVING") {
                rs.x += 0.1;
                rs.y += 0.05;
                if (rs.battery > 0) rs.battery--;
            }
            st = {{"state", rs.state}, {"x", rs.x}, {"y", rs.y},
                  {"battery", rs.battery}};
        }
        if (!sslSendLine(ssl, wmtx, makeMsg("STATUS", st))) {
            alive = false;
            return;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

static void handleMsg(const json& msg, RobotState& rs) {
    std::string type = msg.value("type", "");
    json payload = msg.value("payload", json::object());
    std::lock_guard<std::mutex> lk(rs.mtx);

    if (type == "PATH") {
        rs.path = payload.value("points", json::array());
        rs.state = "MOVING";
        logf("경로 수신 (%zu 포인트) -> MOVING", rs.path.size());
    } else if (type == "CMD") {
        std::string cmd = payload.value("cmd", "");
        if (cmd == "ESTOP") rs.state = "ESTOP";
        else if (cmd == "RESUME") rs.state = rs.path.empty() ? "IDLE" : "MOVING";
        else if (cmd == "CALIB_START") rs.state = "CALIB";
        logf("CMD 수신: %s -> state=%s", cmd.c_str(), rs.state.c_str());
    } else if (type == "POS") {
        logf("POS 보정 수신: x=%s y=%s theta=%s",
             payload.contains("x") ? payload["x"].dump().c_str() : "?",
             payload.contains("y") ? payload["y"].dump().c_str() : "?",
             payload.contains("theta") ? payload["theta"].dump().c_str() : "?");
    } else if (type == "ACK") {
        logf("서버 등록 완료: %s", payload.value("msg", "").c_str());
    }
}

int main(int argc, char** argv) {
    std::string serverIp = argc > 1 ? argv[1] : "127.0.0.1";
    std::string caFile = argc > 2 ? argv[2] : "server.crt";
    const int port = 9000;

    SSL_CTX* ctx = SSL_CTX_new(TLS_client_method());
    SSL_CTX_set_min_proto_version(ctx, TLS1_2_VERSION);
    // 서버 자체서명 인증서를 신뢰 CA로 지정 (인증서 피닝)
    if (SSL_CTX_load_verify_locations(ctx, caFile.c_str(), nullptr) != 1) {
        logf("server.crt 로드 실패: %s (서버에서 복사해왔는지 확인)", caFile.c_str());
        return 1;
    }
    SSL_CTX_set_verify(ctx, SSL_VERIFY_PEER, nullptr);

    RobotState rs;
    for (;;) {  // 자동 재접속 루프
        int fd = socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        inet_pton(AF_INET, serverIp.c_str(), &addr.sin_addr);

        SSL* ssl = nullptr;
        std::atomic<bool> alive{false};
        if (connect(fd, (sockaddr*)&addr, sizeof(addr)) == 0) {
            ssl = SSL_new(ctx);
            SSL_set_fd(ssl, fd);
            if (SSL_connect(ssl) == 1) {
                std::mutex wmtx;
                logf("서버 TLS 접속 %s:%d", serverIp.c_str(), port);
                sslSendLine(ssl, wmtx, makeMsg("HELLO", {{"role", "ROBOT"}}));

                alive = true;
                std::thread st(statusLoop, ssl, std::ref(wmtx), std::ref(rs),
                               std::ref(alive));
                std::string buf, line;
                while (alive && sslReadLine(ssl, buf, line)) {
                    if (line.empty()) continue;
                    json msg = json::parse(line, nullptr, false);
                    if (!msg.is_discarded()) handleMsg(msg, rs);
                }
                alive = false;
                shutdown(fd, SHUT_RDWR);  // statusLoop의 SSL_write 깨우기
                st.join();
            } else {
                logf("TLS 핸드셰이크 실패 (인증서 확인)");
                ERR_print_errors_fp(stderr);
            }
        }
        if (ssl) { SSL_shutdown(ssl); SSL_free(ssl); }
        close(fd);
        logf("연결 끊김 - 3초 후 재시도");
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }
}
