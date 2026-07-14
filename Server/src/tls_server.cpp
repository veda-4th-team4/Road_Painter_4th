#include "tls_server.hpp"
#include "log.hpp"
#include <arpa/inet.h>
#include <netinet/in.h>
#include <openssl/err.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>
#include <thread>

static const char* ROLES[] = {"QT", "ROBOT", "CCTV"};

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
    if (bind(listenFd_, (sockaddr*)&addr, sizeof(addr)) < 0 ||
        listen(listenFd_, 8) < 0)
        throw std::runtime_error("bind/listen 실패 (포트 사용중?)");
}

TlsServer::~TlsServer() {
    if (listenFd_ >= 0) close(listenFd_);
    if (ctx_) SSL_CTX_free(ctx_);
}

void TlsServer::run() {
    logf("[INFO] Road-Painter TLS 서버 시작 0.0.0.0:%d", port_);
    for (;;) {
        sockaddr_in peer{};
        socklen_t len = sizeof(peer);
        int fd = accept(listenFd_, (sockaddr*)&peer, &len);
        if (fd < 0) continue;

        auto c = std::make_shared<Client>();
        c->fd = fd;
        char ip[INET_ADDRSTRLEN] = {0};
        inet_ntop(AF_INET, &peer.sin_addr, ip, sizeof(ip));
        c->peer = std::string(ip) + ":" + std::to_string(ntohs(peer.sin_port));

        std::thread(&TlsServer::sessionThread, this, c).detach();
    }
}

// 개행 단위로 한 줄 읽기 (buf: 연결별 수신 버퍼)
bool TlsServer::readLine(Client& c, std::string& buf, std::string& line) {
    for (;;) {
        auto pos = buf.find('\n');
        if (pos != std::string::npos) {
            line = buf.substr(0, pos);
            buf.erase(0, pos + 1);
            return true;
        }
        char tmp[4096];
        int n = SSL_read(c.ssl, tmp, sizeof(tmp));
        if (n <= 0) return false;  // 끊김/에러
        buf.append(tmp, (size_t)n);
    }
}

void TlsServer::sessionThread(ClientPtr c) {
    std::string role;
    c->ssl = SSL_new(ctx_);
    SSL_set_fd(c->ssl, c->fd);

    // 등록(HELLO) 전까지 10초 타임아웃
    timeval tv{10, 0};
    setsockopt(c->fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    std::string buf, line;
    do {
        if (SSL_accept(c->ssl) != 1) {
            logf("[WARN] TLS 핸드셰이크 실패 %s", c->peer.c_str());
            break;
        }
        // 첫 메시지는 반드시 HELLO {"payload":{"role":"ROBOT"}}
        if (!readLine(*c, buf, line)) break;
        json hello = json::parse(line, nullptr, false);
        if (hello.is_discarded() || hello.value("type", "") != "HELLO") {
            logf("[WARN] 첫 메시지가 HELLO 아님 %s", c->peer.c_str());
            break;
        }
        role = hello["payload"].value("role", "");
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
                shutdown(it->second->fd, SHUT_RDWR);  // 이전 세션 스레드가 정리
            clients_[role] = c;
        }
        // 등록 후: ROBOT은 STATUS 주기전송(하트비트) 규칙이 있어 10초 무수신이면
        // 끊김으로 간주. QT/CCTV는 한동안 조용할 수 있으므로 무한 대기.
        timeval tvAfter = (role == "ROBOT") ? timeval{10, 0} : timeval{0, 0};
        setsockopt(c->fd, SOL_SOCKET, SO_RCVTIMEO, &tvAfter, sizeof(tvAfter));
        logf("[INFO] [접속] %s %s", role.c_str(), c->peer.c_str());
        sendTo(role, makeMsg("ACK", {{"msg", "registered as " + role}}));

        // 수신 루프
        while (readLine(*c, buf, line)) {
            if (line.empty()) continue;
            json msg = json::parse(line, nullptr, false);
            if (msg.is_discarded()) {
                logf("[WARN] JSON 파싱 실패 (%s)", role.c_str());
                continue;
            }
            if (handler_) handler_(role, msg);
        }
    } while (false);

    // 정리
    if (!role.empty()) {
        std::lock_guard<std::mutex> lk(mtx_);
        auto it = clients_.find(role);
        if (it != clients_.end() && it->second == c) clients_.erase(it);
    }
    SSL_shutdown(c->ssl);
    SSL_free(c->ssl);
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
    std::string data = msg.dump() + "\n";
    std::lock_guard<std::mutex> lk(c->writeMtx);
    return SSL_write(c->ssl, data.data(), (int)data.size()) > 0;
}

std::vector<std::string> TlsServer::connectedRoles() {
    std::lock_guard<std::mutex> lk(mtx_);
    std::vector<std::string> v;
    for (auto& kv : clients_) v.push_back(kv.first);
    return v;
}
