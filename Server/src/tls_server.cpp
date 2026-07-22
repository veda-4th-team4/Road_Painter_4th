#include "tls_server.hpp"
#include "log.hpp"
#include <arpa/inet.h>
#include <netinet/in.h>
#include <openssl/err.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <stdexcept>
#include <thread>

// ADMIN = 관리자 창(admin_console). 로그 tap 수신 + 로봇 제어용 (protocol.hpp 참고)
static const char* ROLES[] = {"QT", "ROBOT", "CCTV", "ADMIN"};

static bool validRole(const std::string& r) {
    for (auto* x : ROLES)
        if (r == x) return true;
    return false;
}

TlsServer::TlsServer(int port, const std::string& certFile,
                     const std::string& keyFile)
    : port_(port) {
    ctx_ = SSL_CTX_new(TLS_server_method());
    if (!ctx_) throw std::runtime_error("SSL_CTX_new 실패");
    SSL_CTX_set_min_proto_version(ctx_, TLS1_2_VERSION);
    if (SSL_CTX_use_certificate_file(ctx_, certFile.c_str(), SSL_FILETYPE_PEM) != 1 ||
        SSL_CTX_use_PrivateKey_file(ctx_, keyFile.c_str(), SSL_FILETYPE_PEM) != 1) {
        ERR_print_errors_fp(stderr);
        throw std::runtime_error("인증서/키 로드 실패. gen_cert.sh 먼저 실행했는지 확인");
    }

    listenFd_ = socket(AF_INET, SOCK_STREAM, 0);
    int on = 1;
    setsockopt(listenFd_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons((uint16_t)port_);
    // ::bind로 명시 - listenFd_가 atomic<int>라 ADL이 std::bind를 끌어들여
    // 모호해지는 것을 방지 (POSIX bind() 호출임을 명확히 함)
    if (::bind(listenFd_, (sockaddr*)&addr, sizeof(addr)) < 0 ||
        listen(listenFd_, 8) < 0)
        throw std::runtime_error("bind/listen 실패 (포트 사용중?)");
}

TlsServer::~TlsServer() {
    if (listenFd_ >= 0) close(listenFd_);
    if (ctx_) SSL_CTX_free(ctx_);
}

void TlsServer::run() {
    // 끊긴 소켓에 쓰기(SSL_write/SSL_shutdown) 시 SIGPIPE로 프로세스가 죽지 않도록.
    // (같은 role 재접속으로 기존 세션을 끊는 순간 서버 전체가 종료되는 문제 방지)
    signal(SIGPIPE, SIG_IGN);
    logf("[INFO] Road-Painter TLS 서버 시작 0.0.0.0:%d", port_);
    for (;;) {
        // 순수 블로킹 accept() 대신 poll()로 200ms마다 깨어나 listenFd_(shutdown
        // 여부)를 직접 확인한다. 다른 스레드의 close()가 블로킹 중인 accept()를
        // 즉시 깨워준다는 보장이 리눅스에서도 없어(실측: 1~9초+ 가변, 때로는
        // 전혀 안 깨어남) poll 타임아웃 방식이 유일하게 신뢰할 수 있는 종료 경로.
        int lfd = listenFd_;
        if (lfd < 0) break;  // shutdown()이 의도적으로 닫음

        pollfd pfd{lfd, POLLIN, 0};
        int pr = poll(&pfd, 1, 200);
        if (pr <= 0) continue;  // 타임아웃 또는 일시 오류 - listenFd_ 다시 확인하러 루프 처음으로

        sockaddr_in peer{};
        socklen_t len = sizeof(peer);
        int fd = accept(lfd, (sockaddr*)&peer, &len);
        if (fd < 0) continue;  // poll이 준비됐다고 했지만 accept 실패 - 일시 오류, 재시도

        auto c = std::make_shared<Client>();
        c->fd = fd;
        char ip[INET_ADDRSTRLEN] = {0};
        inet_ntop(AF_INET, &peer.sin_addr, ip, sizeof(ip));
        c->peer = std::string(ip) + ":" + std::to_string(ntohs(peer.sin_port));

        std::thread(&TlsServer::sessionThread, this, c).detach();
    }
    logf("[INFO] accept 루프 종료");
}

// 개행 단위로 한 줄 읽기 (buf: 연결별 수신 버퍼).
// SSL 호출은 sslMtx로 보호해 sendTo(다른 스레드)의 SSL_write와 동시 실행을
// 막는다. 락을 쥔 채 블로킹 대기하지 않도록, poll()로 "읽을 데이터가 있을
// 때만" SSL_read를 호출한다 (SSL 내부에 남은 복호화 데이터는 SSL_pending으로
// 확인). 유휴 대기는 poll이 담당하므로 timeoutMs로 무수신 타임아웃을 앱
// 레벨에서 구현한다 (0 = 무한 대기).
bool TlsServer::readLine(Client& c, std::string& buf, std::string& line,
                         long timeoutMs) {
    for (;;) {
        auto pos = buf.find('\n');
        if (pos != std::string::npos) {
            line = buf.substr(0, pos);
            buf.erase(0, pos + 1);
            return true;
        }
        bool pending;
        {
            std::lock_guard<std::mutex> lk(c.sslMtx);
            if (c.dead || !c.ssl) return false;
            pending = SSL_pending(c.ssl) > 0;
        }
        if (!pending) {
            long waited = 0;
            for (;;) {
                if (c.dead) return false;
                pollfd pfd{c.fd, POLLIN, 0};
                int pr = poll(&pfd, 1, 200);
                if (pr > 0) break;  // 데이터 도착 (또는 HUP - SSL_read가 판정)
                if (pr < 0 && errno != EINTR) return false;
                waited += 200;
                if (timeoutMs > 0 && waited >= timeoutMs)
                    return false;  // 무수신 타임아웃 (하트비트 끊김)
            }
        }
        char tmp[4096];
        int n;
        {
            std::lock_guard<std::mutex> lk(c.sslMtx);
            if (c.dead || !c.ssl) return false;
            n = SSL_read(c.ssl, tmp, sizeof(tmp));
        }
        if (n <= 0) return false;  // 끊김/에러
        buf.append(tmp, (size_t)n);
    }
}

void TlsServer::sessionThread(ClientPtr c) {
    std::string role;
    c->ssl = SSL_new(ctx_);
    SSL_set_fd(c->ssl, c->fd);

    // 커널 수신 타임아웃(백스톱): 핸드셰이크나 불완전한 TLS 레코드가 sslMtx를
    // 쥔 채 무한 블로킹하는 것을 방지. 유휴 대기는 readLine의 poll이 담당하므로
    // 이 값이 조용한 클라이언트를 끊지는 않는다.
    timeval tv{10, 0};
    setsockopt(c->fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    std::string buf, line;
    do {
        if (SSL_accept(c->ssl) != 1) {
            logf("[WARN] TLS 핸드셰이크 실패 %s", c->peer.c_str());
            break;
        }
        // 첫 메시지는 반드시 HELLO {"payload":{"role":"ROBOT"}} - 10초 내
        if (!readLine(*c, buf, line, 10000)) break;
        json hello = json::parse(line, nullptr, false);
        if (hello.is_discarded() || hello.value("type", "") != "HELLO") {
            logf("[WARN] 첫 메시지가 HELLO 아님 %s", c->peer.c_str());
            break;
        }
        // payload가 없거나 오브젝트가 아니어도 throw하지 않도록 방어적으로 파싱
        json hp = hello.value("payload", json::object());
        role = hp.is_object() ? hp.value("role", "") : "";
        for (auto& ch : role) ch = (char)toupper(ch);
        if (!validRole(role)) {
            logf("[WARN] 알 수 없는 role '%s' %s", role.c_str(), c->peer.c_str());
            role.clear();
            break;
        }

        // 같은 role 기존 연결이 있으면 끊고 교체 (재접속 대응)
        {
            std::lock_guard<std::mutex> lk(mtx_);
            auto it = clients_.find(role);
            if (it != clients_.end())
                ::shutdown(it->second->fd, SHUT_RDWR);  // 이전 세션 스레드가 정리
            clients_[role] = c;
        }
        // 등록 후: ROBOT은 STATUS 주기전송(하트비트) 규칙이 있어 10초 무수신이면
        // 끊김으로 간주. QT/CCTV는 한동안 조용할 수 있으므로 무한 대기.
        long idleTimeoutMs = (role == "ROBOT") ? 10000 : 0;
        logf("[INFO] [접속] %s %s", role.c_str(), c->peer.c_str());
        sendTo(role, makeMsg("ACK", {{"msg", "registered as " + role}}));

        // 수신 루프
        while (readLine(*c, buf, line, idleTimeoutMs)) {
            if (line.empty()) continue;
            json msg = json::parse(line, nullptr, false);
            if (msg.is_discarded()) {
                logf("[WARN] JSON 파싱 실패 (%s)", role.c_str());
                continue;
            }
            tapToAdmin("IN", role, msg);  // 관리자 창에 사본 전달 (엿듣기)
            if (handler_) {
                try {
                    handler_(role, msg);
                } catch (const std::exception& e) {
                    // 필드 타입이 어긋난 메시지(json type_error 등) 하나가
                    // 세션 스레드를 터뜨려 서버 전체가 죽는 것을 방지.
                    // 해당 메시지만 버리고 세션은 유지한다.
                    logf("[WARN] 메시지 처리 중 예외 (%s): %s", role.c_str(),
                         e.what());
                }
            }
        }
    } while (false);

    // 정리: 먼저 레지스트리에서 제거해 새 sendTo가 이 연결을 못 잡게 하고,
    // sslMtx 안에서 dead 표시 후 해제한다 - 이미 ClientPtr를 쥔 다른 스레드가
    // 해제된 SSL에 SSL_write하는 use-after-free 방지 (sendTo는 dead 확인).
    if (!role.empty()) {
        std::lock_guard<std::mutex> lk(mtx_);
        auto it = clients_.find(role);
        if (it != clients_.end() && it->second == c) clients_.erase(it);
    }
    {
        std::lock_guard<std::mutex> lk(c->sslMtx);
        c->dead = true;
        SSL_shutdown(c->ssl);
        SSL_free(c->ssl);
        c->ssl = nullptr;
    }
    close(c->fd);
    logf("[INFO] [해제] %s %s", role.empty() ? "?" : role.c_str(), c->peer.c_str());
}

bool TlsServer::sendTo(const std::string& role, const json& msg) {
    ClientPtr c;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        auto it = clients_.find(role);
        if (it != clients_.end()) c = it->second;
    }
    if (!c) {
        logf("[WARN] send 실패: %s 미접속 (type=%s)", role.c_str(),
             msg.value("type", "?").c_str());
        return false;
    }
    bool ok = false;
    {
        std::string data = msg.dump() + "\n";
        std::lock_guard<std::mutex> lk(c->sslMtx);
        if (!c->dead && c->ssl)
            ok = SSL_write(c->ssl, data.data(), (int)data.size()) > 0;
    }
    tapToAdmin("OUT", role, msg);  // 관리자 창에 사본 전달 (엿듣기)
    return ok;
}

// 관리자 창(ADMIN role)에게 오가는 메시지의 사본을 TAP으로 흘려준다.
//   dir="IN"  : peer가 서버로 보낸 것 / dir="OUT": 서버가 peer에게 보낸 것
// ADMIN 자신과의 트래픽은 tap하지 않는다(무한 루프 방지). 미접속이면 조용히 스킵.
// sendTo가 아닌 admin 소켓에 직접 써서 재귀/로그 스팸을 피한다.
void TlsServer::tapToAdmin(const char* dir, const std::string& peer,
                           const json& msg) {
    if (peer == "ADMIN") return;
    ClientPtr admin;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        auto it = clients_.find("ADMIN");
        if (it != clients_.end()) admin = it->second;
    }
    if (!admin) return;
    json tap = makeMsg("TAP", {{"dir", dir}, {"peer", peer}, {"msg", msg}});
    std::string data = tap.dump() + "\n";
    std::lock_guard<std::mutex> lk(admin->sslMtx);
    if (!admin->dead && admin->ssl)
        SSL_write(admin->ssl, data.data(), (int)data.size());
}

std::vector<std::string> TlsServer::connectedRoles() {
    std::lock_guard<std::mutex> lk(mtx_);
    std::vector<std::string> v;
    for (auto& kv : clients_) v.push_back(kv.first);
    return v;
}

// 리스닝 소켓 종료: accept() 루프를 빠져나오게 함
// (accept()는 블로킹이므로 signal handler에서 직접 종료 불가 — 이 메서드 호출 필요)
void TlsServer::shutdown() {
    // exchange로 원자적으로 빼앗아, 시그널 핸들러와 메인 스레드(quit 경로)가
    // 동시에 호출해도 close가 한 번만 실행되게 한다
    int fd = listenFd_.exchange(-1);
    if (fd >= 0) {
        close(fd);
        logf("[INFO] 리스닝 소켓 종료");
    }
}
