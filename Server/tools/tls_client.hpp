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
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <csignal>
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
        // 끊긴 소켓에 SSL_write(특히 close()의 SSL_shutdown)를 하면 SIGPIPE로
        // 프로세스가 통째로 죽는다 - 상대가 먼저 나가는 것은 테스트에서 흔한
        // 정상 상황인데, 그때마다 도구가 죽으면 결과를 못 본다. 오류는 반환값
        // 으로 받는다.
        ::signal(SIGPIPE, SIG_IGN);
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
        // 🔴 핸드셰이크가 끝난 뒤에야 논블로킹으로 바꾼다. 대부분의 도구가
        //   "수신 스레드 + 메인 스레드 송신" 구조라 SSL_read와 SSL_write가 같은
        //   SSL 객체에 동시에 들어가는데, OpenSSL의 SSL 객체는 그렇게 쓰라고
        //   만들어진 물건이 아니다. 아래 io_ 하나로 둘을 직렬화하되, 블로킹
        //   SSL_read가 락을 쥔 채 잠들면 송신이 영영 막히므로 poll()로 읽을
        //   것이 생겼을 때만 락을 잡는다.
        //   (이 레이스는 실제로 관측됐다: 접속 직후 보낸 LOGIN이 서버에 아예
        //    도착하지 않고 세션이 끊기는 현상이 몇 번에 한 번씩 재현됐다.)
        int fl = fcntl(fd_, F_GETFL, 0);
        fcntl(fd_, F_SETFL, fl | O_NONBLOCK);
        tlogf(tag_, "접속 성공 %s:%d (role=%s)", ip.c_str(), port, role.c_str());
        return send("HELLO", {{"role", role}});
    }

    bool send(const std::string& type, const json& payload) {
        json msg{{"type", type}, {"seq", ++seq_}, {"payload", payload}};
        std::string data = msg.dump() + "\n";
        size_t off = 0;
        while (off < data.size()) {
            int n;
            {
                std::lock_guard<std::mutex> lk(io_);
                if (!ssl_) return false;
                n = SSL_write(ssl_, data.data() + off, (int)(data.size() - off));
                if (n <= 0) {
                    int e = SSL_get_error(ssl_, n);
                    if (e != SSL_ERROR_WANT_READ && e != SSL_ERROR_WANT_WRITE)
                        return false;
                }
            }
            if (n > 0) off += (size_t)n;
            else waitIo(POLLOUT, 200);  // 커널 송신 버퍼가 빌 때까지
        }
        return true;
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
            // SSL_read가 이미 복호화해 들고 있는 바이트는 poll()에 안 잡힌다
            // (커널 소켓은 비어 있는데 SSL 내부 버퍼에는 남아 있는 상태).
            // 그래서 SSL_pending을 먼저 보고, 없을 때만 fd를 기다린다.
            {
                std::lock_guard<std::mutex> lk(io_);
                if (!ssl_) return false;
                char tmp[4096];
                int n = SSL_read(ssl_, tmp, sizeof(tmp));
                if (n > 0) {
                    buf_.append(tmp, (size_t)n);
                    continue;
                }
                int e = SSL_get_error(ssl_, n);
                if (e != SSL_ERROR_WANT_READ && e != SSL_ERROR_WANT_WRITE)
                    return false;  // 정상 종료 또는 오류
            }
            if (!waitIo(POLLIN, 200) && closed_) return false;
        }
    }

    // 수신 스레드의 SSL_read를 깨워 정상 종료시킨다
    void shutdownRead() {
        closed_ = true;
        if (fd_ >= 0) ::shutdown(fd_, SHUT_RDWR);
    }

    void close() {
        if (ssl_) { SSL_shutdown(ssl_); SSL_free(ssl_); ssl_ = nullptr; }
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
        if (ctx_) { SSL_CTX_free(ctx_); ctx_ = nullptr; }
    }

private:
    // fd가 준비될 때까지 기다린다. 🔴 반드시 io_ 락을 놓은 상태로 부를 것 -
    // 여기서 잠든 채 락을 쥐고 있으면 반대 방향이 통째로 막힌다.
    bool waitIo(short ev, int timeoutMs) {
        if (fd_ < 0) return false;
        pollfd p{fd_, ev, 0};
        return ::poll(&p, 1, timeoutMs) > 0;
    }

    const char* tag_;
    SSL_CTX* ctx_ = nullptr;
    SSL* ssl_ = nullptr;
    int fd_ = -1;
    std::string buf_;
    // SSL 객체 하나에 대한 읽기/쓰기를 직렬화한다 (송신 전용이 아니다).
    std::mutex io_;
    std::atomic<bool> closed_{false};
    std::atomic<long> seq_{0};
};
