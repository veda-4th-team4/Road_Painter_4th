#pragma once
// 라우팅 레이어: "누가(role) 무엇을(type) 보냈나"에 따라 중계/저장/판단.
//   QT    -> REGISTER/LOGIN -> 사용자 등록/검증, 저장된 캘리브레이션 회신
//   QT    -> CMD       -> ROBOT (CALIB_START이면 CCTV에도)
//   QT    -> BLUEPRINT -> 저장 + 로봇 위치 알면 경로 생성해 PATH 전송
//               (points는 Qt가 top-view 픽셀 -> 바닥 미터 변환을 마친 좌표)
//   ROBOT -> STATUS    -> QT 중계
//   CCTV  -> POS       -> 원본 픽셀 4코너 수신. 서버가 undistort -> H_marker로
//               pose 계산 (좌표 변환은 전부 서버 담당) + ROBOT/QT 중계 +
//               계산된 POSE를 QT로 전송 + 이탈 시 재계획
//   CCTV  -> H_MATRIX  -> 캘리브레이션 번들(K,D,H_floor,H_marker) 수신,
//               로그인 사용자에 영속 저장 + QT 중계
#include "calib.hpp"
#include "path_planner.hpp"
#include "protocol.hpp"
#include "tls_server.hpp"
#include "user_store.hpp"

class Router {
public:
    explicit Router(TlsServer& srv) : srv_(srv), users_("config/users.json") {}
    void onMessage(const std::string& role, const json& msg);

private:
    void fromQt(const json& msg);
    void fromRobot(const json& msg);
    void fromCctv(const json& msg);
    // 도면+pose가 모두 준비됐으면 경로 생성해서 로봇에 전송
    void tryPlanAndSend();

    // 러프 디폴트 (추후 현장 튜닝)
    static constexpr double kDevThresholdM = 0.3;    // 이탈 판정 거리
    static constexpr long kReplanCooldownMs = 3000;  // 재계획 최소 간격

    TlsServer& srv_;
    UserStore users_;          // id/pw/H행렬 영속 저장소
    std::string currentUser_;  // 로그인된 사용자 (단일 사용자 가정)
    Calib calib_;      // 캘리브레이션 (현재 세션. raw는 calib_.raw)
    json blueprint_;   // Qt가 보낸 도면 원본
    json lastStatus_;  // 로봇 최신 상태
    json lastPos_;     // CCTV 최신 마커 검출 원본 (픽셀)

    std::vector<Pt> planPts_;  // 도면 폴리라인 (바닥 미터)
    bool planActive_ = false;  // PATH를 로봇에 보낸 상태인지
    bool manualMode_ = false;  // Qt 수동 조작(조이스틱) 중 - 자동 경로추종/재계획 중단.
                               // 새 BLUEPRINT 수신 시 해제(자동 모드 복귀)
    Pose pose_;                // 로봇 최신 pose (top-view)
    bool poseValid_ = false;
    long lastPlanMs_ = 0;      // 마지막 PATH 전송 시각 (쿨다운용)
};
