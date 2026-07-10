// OpenSSL TLS 서버 래퍼. (앞서 CLI로 한 openssl s_server 를 코드로 옮긴 것)
// 서버 RPi가 서버 역할, 로봇/Qt가 클라이언트로 접속.
#pragma once
#include <openssl/ssl.h>
#include <string>

class TlsServer {
public:
    ~TlsServer();
    // 인증서/키 로드 + 8443 등 포트 열기
    bool start(const std::string& cert_path, const std::string& key_path, int port);
    // 클라이언트 1명 접속 대기(블로킹). 성공 시 SSL* 반환, 실패 시 nullptr.
    SSL* acceptClient();

    // 한 줄(\n 종단) 수신. 연결 끊기면 빈 문자열.
    static std::string readLine(SSL* ssl);
    // 문자열 송신(끝에 \n 자동 부가).
    static bool writeLine(SSL* ssl, const std::string& msg);
    static void closeClient(SSL* ssl);

private:
    SSL_CTX* ctx_ = nullptr;
    int listen_fd_ = -1;
};
