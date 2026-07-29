#pragma once
// 테스트 도구용 TLS 클라이언트 뼈대 (robot_sim / draw_test 공용).
//
// qt_sim.cpp / path_test.cpp가 각자 복사해 쓰던 접속·송수신 코드를 한 곳에 모은 것.
// 서버와 동일하게 JSON 한 줄 + '\n' (JSON Lines) 프레이밍을 쓴다.
//
// 인증서 피닝: 서버 자체서명 certs/server.crt를 신뢰 CA로 지정해 검증한다
// (SSL_VERIFY_PEER). crt 경로가 틀리면 접속 자체가 실패하도록 두는 게 맞다 -
// 검증을 끄면 테스트에서만 통과하고 현장에서 다른 문제로 둔갑한다.
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

using json = nlohmann::json;

// 시각 + 태그가 붙은 stderr 로그 (여러 스레드에서 불러도 줄이 안 섞이게 직렬화)
inline void tlogf(const char* tag, const char* fmt, ...) {
    static std::mutex m;
    std::lock_guard<std::mutex> lk(m);
    char ts[16];
    time_t t = time(nullptr);
    struct tm tmv;
    localtime_r(&t, &tmv);
    strftime(ts, sizeof(ts), "%H:%M:%S", &tmv);
    fprintf(stderr, "%s [%s] ", ts, tag);
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fprintf(stderr, "\n");
    fflush(stderr);
}

// 서버에 붙은 role 하나. 한 프로세스가 여러 role로 붙을 수 있어서
// (robot_sim은 ROBOT + CCTV 두 개) 접속 상태를 객체로 들고 다닌다.
class TlsLink {
public:
    TlsLink(const char* tag) : tag_(tag) {}
    ~TlsLink() { close(); }

    TlsLink(const TlsLink&) = delete;
    TlsLink& operator=(const TlsLink&) = delete;

    // 접속 + HELLO{role} 까지. 실패하면 이유를 로그로 남기고 false.
    bool connect(const std::string& ip, int port, const std::string& caFile,
                 const std::string& role) {
        ctx_ = SSL_CTX_new(TLS_client_method());
        SSL_CTX_set_min_proto_version(ctx_, TLS1_2_VERSION);
        if (SSL_CTX_load_verify_locations(ctx_, caFile.c_str(), nullptr) != 1) {
            tlogf(tag_, "server.crt 로드 실패: %s", caFile.c_str());
            return false;
        }
        SSL_CTX_set_verify(ctx_, SSL_VERIFY_PEER, nullptr);

        fd_ = socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons((uint16_t)port);
        inet_pton(AF_INET, ip.c_str(), &addr.sin_addr);
        if (::connect(fd_, (sockaddr*)&addr, sizeof(addr)) != 0) {
            tlogf(tag_, "서버 접속 실패 (%s:%d)", ip.c_str(), port);
            return false;
        }
        ssl_ = SSL_new(ctx_);
        SSL_set_fd(ssl_, fd_);
        if (SSL_connect(ssl_) != 1) {
            tlogf(tag_, "TLS 핸드셰이크 실패 (인증서 확인)");
            ERR_print_errors_fp(stderr);
            return false;
        }
        tlogf(tag_, "접속 성공 %s:%d (role=%s)", ip.c_str(), port, role.c_str());
        return send("HELLO", {{"role", role}});
    }

    bool send(const std::string& type, const json& payload) {
        json msg{{"type", type}, {"seq", ++seq_}, {"payload", payload}};
        std::string data = msg.dump() + "\n";
        std::lock_guard<std::mutex> lk(wmtx_);
        if (!ssl_) return false;
        return SSL_write(ssl_, data.data(), (int)data.size()) > 0;
    }

    // 한 줄 수신 (블로킹). 연결이 끊기면 false.
    bool recv(json& out) {
        for (;;) {
            auto pos = buf_.find('\n');
            if (pos != std::string::npos) {
                std::string line = buf_.substr(0, pos);
                buf_.erase(0, pos + 1);
                if (line.empty()) continue;
                out = json::parse(line, nullptr, false);
                if (out.is_discarded()) {
                    tlogf(tag_, "JSON 파싱 실패: %s", line.c_str());
                    continue;
                }
                return true;
            }
            char tmp[4096];
            int n = SSL_read(ssl_, tmp, sizeof(tmp));
            if (n <= 0) return false;
            buf_.append(tmp, (size_t)n);
        }
    }

    // 수신 스레드의 SSL_read를 깨워 정상 종료시킨다
    void shutdownRead() {
        if (fd_ >= 0) ::shutdown(fd_, SHUT_RDWR);
    }

    void close() {
        if (ssl_) { SSL_shutdown(ssl_); SSL_free(ssl_); ssl_ = nullptr; }
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
        if (ctx_) { SSL_CTX_free(ctx_); ctx_ = nullptr; }
    }

private:
    const char* tag_;
    SSL_CTX* ctx_ = nullptr;
    SSL* ssl_ = nullptr;
    int fd_ = -1;
    std::string buf_;
    std::mutex wmtx_;
    std::atomic<long> seq_{0};
};
