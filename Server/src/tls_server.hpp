#pragma once
// TLS 네트워크 레이어: 리스닝, 클라이언트 세션(스레드), role 레지스트리.
// 메시지 "내용" 처리는 하지 않고 Router(핸들러)에 넘긴다.
#include "protocol.hpp"
#include <openssl/ssl.h>
#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

class TlsServer {
public:
    // (보낸 클라이언트 role, 파싱된 JSON 메시지)
    using Handler = std::function<void(const std::string&, const json&)>;
    // (role, true=접속/false=해제). 재접속으로 기존 세션이 교체될 때는
    // false가 안 나간다 (실제로 아무도 없어졌을 때만 호출 - sessionThread 참고).
    using PeerHandler = std::function<void(const std::string&, bool)>;

    TlsServer(int port, const std::string& certFile, const std::string& keyFile);
    ~TlsServer();

    void setHandler(Handler h) { handler_ = std::move(h); }
    void setPeerHandler(PeerHandler h) { peerHandler_ = std::move(h); }
    void run();  // accept 루프 (블로킹) - 별도 스레드에서 호출
    void shutdown();  // 리스닝 소켓 종료 (accept 루프에서 빠져나옴)

    // 특정 role 클라이언트에 msg 전송. 미접속이면 false.
    bool sendTo(const std::string& role, const json& msg);
    std::vector<std::string> connectedRoles();

private:
    struct Client {
        SSL* ssl = nullptr;
        int fd = -1;
        std::string peer;
        // 이 연결의 모든 SSL_* 호출 보호. OpenSSL의 SSL 객체는 read/write가
        // 서로 다른 스레드여도 동시 사용이 안전하지 않다 (세션 스레드의
        // SSL_read vs sendTo의 SSL_write). 세션 정리(SSL_free)도 이 락 안에서
        // dead 표시 후 수행해 해제된 SSL에 쓰는 use-after-free를 막는다.
        std::mutex sslMtx;
        std::atomic<bool> dead{false};  // true면 SSL 사용 금지 (해제됨/해제 중)
    };
    using ClientPtr = std::shared_ptr<Client>;

    void sessionThread(ClientPtr c);  // 클라이언트 1개 전담
    // timeoutMs > 0: 그 시간 동안 새 데이터가 없으면 false (무수신 타임아웃)
    static bool readLine(Client& c, std::string& buf, std::string& line,
                         long timeoutMs);
    // 오가는 메시지 사본을 ADMIN(관리자 창)에게 TAP으로 전달
    void tapToAdmin(const char* dir, const std::string& peer, const json& msg);

    int port_;
    // shutdown()(신호 핸들러/메인 스레드)이 닫고, run()(네트워크 스레드)이
    // accept 루프에서 읽는다 - 스레드 간 공유라 atomic 필요.
    std::atomic<int> listenFd_{-1};
    SSL_CTX* ctx_ = nullptr;
    Handler handler_;
    PeerHandler peerHandler_;
    std::mutex mtx_;                            // clients_ 보호
    std::map<std::string, ClientPtr> clients_;  // role -> 연결
};
