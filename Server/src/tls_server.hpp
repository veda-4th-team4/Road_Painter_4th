#pragma once
// TLS 네트워크 레이어: 리스닝, 클라이언트 세션(스레드), role 레지스트리.
// 메시지 "내용" 처리는 하지 않고 Router(핸들러)에 넘긴다.
#include "protocol.hpp"
#include <openssl/ssl.h>
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

    TlsServer(int port, const std::string& certFile, const std::string& keyFile);
    ~TlsServer();

    void setHandler(Handler h) { handler_ = std::move(h); }
    void run();  // accept 루프 (블로킹) - 별도 스레드에서 호출

    // 특정 role 클라이언트에 msg 전송. 미접속이면 false.
    bool sendTo(const std::string& role, const json& msg);
    std::vector<std::string> connectedRoles();

private:
    struct Client {
        SSL* ssl = nullptr;
        int fd = -1;
        std::string peer;
        std::mutex writeMtx;  // 같은 연결에 동시 쓰기 방지
    };
    using ClientPtr = std::shared_ptr<Client>;

    void sessionThread(ClientPtr c);  // 클라이언트 1개 전담
    static bool readLine(Client& c, std::string& buf, std::string& line);

    int port_;
    int listenFd_ = -1;
    SSL_CTX* ctx_ = nullptr;
    Handler handler_;
    std::mutex mtx_;                            // clients_ 보호
    std::map<std::string, ClientPtr> clients_;  // role -> 연결
};
