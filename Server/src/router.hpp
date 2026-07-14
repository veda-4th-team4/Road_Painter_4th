#pragma once
// 라우팅 레이어: "누가(role) 무엇을(type) 보냈나"에 따라 중계/저장.
//   QT    -> CMD       -> ROBOT (CALIB_START이면 CCTV에도)
//   QT    -> BLUEPRINT -> 저장 (추후 경로 생성 -> PATH 송신)
//   ROBOT -> STATUS    -> QT 중계
//   CCTV  -> POS       -> ROBOT(보정) + QT(모니터링) 중계
//   CCTV  -> H_MATRIX  -> 저장
#include "protocol.hpp"
#include "tls_server.hpp"

class Router {
public:
    explicit Router(TlsServer& srv) : srv_(srv) {}
    void onMessage(const std::string& role, const json& msg);

private:
    void fromQt(const json& msg);
    void fromRobot(const json& msg);
    void fromCctv(const json& msg);

    TlsServer& srv_;
    json hMatrix_;     // CCTV가 보낸 호모그래피 행렬
    json blueprint_;   // Qt가 보낸 도면
    json lastStatus_;  // 로봇 최신 상태
    json lastPos_;     // CCTV 최신 위치
};
