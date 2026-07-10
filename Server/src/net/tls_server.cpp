#include "tls_server.hpp"
#include <openssl/err.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

TlsServer::~TlsServer() {
    if (listen_fd_ >= 0) ::close(listen_fd_);
    if (ctx_) SSL_CTX_free(ctx_);
}

bool TlsServer::start(const std::string& cert, const std::string& key, int port) {
    // 1) OpenSSL 컨텍스트 + 인증서/키 로드
    ctx_ = SSL_CTX_new(TLS_server_method());
    if (!ctx_) { ERR_print_errors_fp(stderr); return false; }
    if (SSL_CTX_use_certificate_file(ctx_, cert.c_str(), SSL_FILETYPE_PEM) <= 0 ||
        SSL_CTX_use_PrivateKey_file(ctx_, key.c_str(), SSL_FILETYPE_PEM) <= 0) {
        ERR_print_errors_fp(stderr); return false;
    }
    if (!SSL_CTX_check_private_key(ctx_)) {
        std::cerr << "인증서와 키가 짝이 아님\n"; return false;
    }

    // 2) TCP 소켓 열고 포트 바인딩
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) { perror("socket"); return false; }
    int opt = 1;
    setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;   // 모든 인터페이스(=Tailscale 포함)에서 수신
    addr.sin_port = htons(port);
    if (::bind(listen_fd_, (sockaddr*)&addr, sizeof(addr)) < 0) { perror("bind"); return false; }
    if (::listen(listen_fd_, 1) < 0) { perror("listen"); return false; }
    std::cout << "[TLS] 포트 " << port << " 대기 중...\n";
    return true;
}

SSL* TlsServer::acceptClient() {
    sockaddr_in cli{}; socklen_t len = sizeof(cli);
    int fd = ::accept(listen_fd_, (sockaddr*)&cli, &len);
    if (fd < 0) { perror("accept"); return nullptr; }

    SSL* ssl = SSL_new(ctx_);
    SSL_set_fd(ssl, fd);
    if (SSL_accept(ssl) <= 0) {         // TLS 핸드셰이크
        ERR_print_errors_fp(stderr);
        SSL_free(ssl); ::close(fd);
        return nullptr;
    }
    std::cout << "[TLS] 클라이언트 접속, 암호화 세션 수립\n";
    return ssl;
}

std::string TlsServer::readLine(SSL* ssl) {
    std::string line; char c;
    while (true) {
        int n = SSL_read(ssl, &c, 1);
        if (n <= 0) return line.empty() ? std::string() : line;  // 끊김
        if (c == '\n') break;
        line.push_back(c);
    }
    return line;
}

bool TlsServer::writeLine(SSL* ssl, const std::string& msg) {
    std::string out = msg + "\n";
    return SSL_write(ssl, out.data(), (int)out.size()) > 0;
}

void TlsServer::closeClient(SSL* ssl) {
    if (!ssl) return;
    int fd = SSL_get_fd(ssl);
    SSL_shutdown(ssl);
    SSL_free(ssl);
    if (fd >= 0) ::close(fd);
}
