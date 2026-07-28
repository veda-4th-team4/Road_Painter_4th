#pragma once
// 라우팅 레이어: "누가(role) 무엇을(type) 보냈나"에 따라 중계/저장/판단.
//   QT    -> REGISTER/LOGIN -> 사용자 등록/검증, 저장된 캘리브레이션 + cam_ip 회신
//               (cam_ip는 REGISTER 때 같이 등록, LOGIN_OK에 그대로 회신 - 검증 없음)
//   ADMIN -> LOGIN     -> QT와 동일 처리 (응답만 ADMIN으로). 캘리브레이션은
//               "현재 로그인된 사용자"에게 저장되므로, QT 없이 관리자 창만으로
//               캘리를 계정에 남기려면 여기서 먼저 로그인해둔다.
//   QT    -> SET_CAM_IP -> 로그인 사용자의 카메라 IP 교체 (Qt 설정란)
//   QT    -> CMD       -> ROBOT (CALIB_START이면 CCTV에도)
//   QT    -> BLUEPRINT -> 저장만 한다 (경로는 START_DRAW 때 생성)
//               (points는 Qt가 top-view 픽셀 -> 바닥 미터 변환을 마친 좌표)
//               2026-07-28: paint[] / pen_offset_m / program[] 선택 필드 추가.
//               program이 오면 도색 동작 시퀀스를 서버가 만들지 않고 그대로
//               로봇에 넘긴다 (펜 오프셋 보정이 들어있어 Qt만 만들 수 있다).
//   QT    -> CMD START_DRAW -> 1단계(접근) PATH 생성·전송. 이후는 전부 자동:
//               로봇 PATH_DONE(접근) -> 서버가 2단계(도색) PATH 자동 전송 ->
//               로봇 PATH_DONE(도색) -> QT에 DRAW_DONE 통지.
//               실패/대기 시 QT에 DRAW_FAIL{stage,reason,msg} 통지
//   ROBOT -> STATUS    -> QT 중계
//   ROBOT -> READY     -> MOVE 출발 직전 정렬 확인 요청. 서버가 CCTV pose의
//               실제 각도와 목표 heading을 비교해 ALIGN(미세회전) 또는 GO 응답
//   ROBOT -> PATH_DONE -> 받은 PATH를 끝까지 수행했다는 통지. 접근 완료면 서버가
//               곧바로 도색 PATH를 이어 보내고(QT에는 알리지 않음), 도색 완료면
//               QT에 DRAW_DONE을 통지하고 경로 상태를 정리한다.
//   CCTV  -> POS       -> 원본 픽셀 4코너 수신. 서버가 undistort -> H_marker로
//               pose 계산 (좌표 변환은 전부 서버 담당) + 계산된 POSE를 QT로 전송
//               + 이탈 시 재계획. POS 원본은 QT에 중계하지 않는다 (POSE만 사용)
//               (로봇은 좌표를 받지 않음 - 각도 피드백 ALIGN/DRIFT로만 보정)
//   CCTV  -> H_MATRIX  -> 캘리브레이션 번들(K,D,H_floor,H_marker) 수신,
//               로그인 사용자에 영속 저장 + QT 중계
//   ROBOT/CCTV 접속·해제 -> QT에 PEERS{"robot":bool,"cctv":bool} 통지
//               (QT 접속 시에도 현재 스냅샷 1회 전송)
#include "calib.hpp"
#include "path_planner.hpp"
#include "protocol.hpp"
#include "tls_server.hpp"
#include "user_store.hpp"
#include <mutex>

class Router {
public:
    explicit Router(TlsServer& srv) : srv_(srv), users_("config/users.json") {}
    void onMessage(const std::string& role, const json& msg);
    // ROBOT/CCTV 접속·해제 시(TlsServer가 통지) QT에 PEERS로 알림.
    // QT 자신이 막 접속했을 때도 호출되어 현재 접속 현황 스냅샷을 보낸다.
    void onPeerChange(const std::string& role, bool connected);

private:
    // 현재 ROBOT/CCTV 접속 여부를 모아 PEERS 메시지로 QT에 전송
    void sendPeers();
    void fromQt(const json& msg);
    void fromRobot(const json& msg);
    void fromCctv(const json& msg);
    void fromAdmin(const json& msg);  // 관리자 창 -> 로봇 제어(CMD/PATH) + 캘리(H_MATRIX)
    // 로그인 처리 (QT/ADMIN 공용): currentUser_ 갱신 + 저장된 캘리 복원 + 결과 회신.
    // replyRole = LOGIN_OK/LOGIN_FAIL을 돌려줄 role ("QT" 또는 "ADMIN").
    void handleLogin(const json& payload, const std::string& replyRole);
    // 캘리브레이션 번들 수신 처리 (CCTV/ADMIN 공용): 저장 + Qt 중계
    void handleHMatrix(const json& msg);
    // Qt의 START_DRAW 수신 시 1단계(시작점 접근) 경로를 로봇에 전송.
    // pose를 아직 모르면 drawRequested_만 세워두고 첫 POS 수신 때 재시도한다.
    void startApproach();
    // 로봇의 접근 완료(PATH_DONE) 수신 시 2단계(도색) 경로를 로봇에 전송
    void sendDrawPath();
    // 이탈 복귀 경로. planPts_[k]로 돌아가는 구간만 새로 만들고, Qt program이
    // 있으면 원본을 잘라 이어 붙인다 (재생성하면 꼭짓점 동작이 사라진다).
    json buildRecovery(size_t k);
    // 경로 생성/전송 실패(또는 대기) 시 Qt에 DRAW_FAIL로 통지
    void sendDrawFail(const char* stage, const char* reason, const std::string& msg);

    // 러프 디폴트 (추후 현장 튜닝)
    static constexpr double kDevThresholdM = 0.3;    // 이탈 판정 거리
    static constexpr long kReplanCooldownMs = 3000;  // 재계획 최소 간격
    static constexpr double kAlignThresholdDeg = 2.0;  // 출발 전 정렬 허용 오차
    static constexpr int kAlignMaxTries = 4;  // ALIGN 최대 반복 (초과 시 그냥 GO)
    static constexpr long kDriftPeriodMs = 200;  // 주행 중 각도 피드백(DRIFT) 최소 간격
    // 진행 커서를 다음 꼭짓점으로 넘기는 도달 반경. 이탈 임계값(0.3m)보다
    // 확실히 작게 둘 것 - 커서가 너무 일찍 넘어가면 꼭짓점 직전의 정상 주행이
    // "다음 구간에서 벗어남"으로 오탐된다.
    static constexpr double kVertexReachM = 0.15;

    // 각 클라이언트 세션 스레드가 onMessage()를 호출하므로 아래 상태 전부가
    // 스레드 간 공유다 (예: QT의 BLUEPRINT가 planPts_를 비우는 동안 CCTV의
    // POS가 순회하면 UB). onMessage() 전체를 이 뮤텍스 하나로 직렬화한다.
    std::mutex mtx_;
    TlsServer& srv_;
    UserStore users_;          // id/pw/H행렬 영속 저장소
    std::string currentUser_;  // 로그인된 사용자 (단일 사용자 가정)
    Calib calib_;      // 캘리브레이션 (현재 세션. raw는 calib_.raw)
    json blueprint_;   // Qt가 보낸 도면 원본
    json lastStatus_;  // 로봇 최신 상태
    json lastPos_;     // CCTV 최신 마커 검출 원본 (픽셀)

    std::vector<Pt> planPts_;  // 도면 폴리라인 (바닥 미터) = 펜이 지나갈 자취
    // 구간별 도색 여부. planPaint_[i] = "planPts_[i-1] -> planPts_[i] 구간을
    // 칠하는가". planPts_와 길이가 같을 때만 유효(다르면 비운다). 도형 여러 개를
    // 한 폴리라인으로 이어 보낼 때 도형 사이 이동을 칠하지 않기 위한 것.
    std::vector<char> planPaint_;
    // Qt가 만들어 보낸 동작 시퀀스 원본(MOVE/TURN/NOZZLE). 비어 있지 않으면
    // 서버는 도색 경로를 생성하지 않고 이것을 그대로 로봇에 넘긴다.
    // 펜 오프셋 보정(꼭짓점 후진/회전/재전진)이 이미 들어 있어 서버가 다시
    // 만들면 펜 위치가 어긋나고 Qt 미리보기와도 달라진다.
    json planProgram_;
    // 펜(노즐)이 로봇 회전중심 뒤로 떨어진 거리(m). 원천은 Qt(기구값)이고,
    // 서버는 이탈 복귀 경로의 펜 정렬(MOVE +d)에만 쓴다.
    double penOffsetM_ = 0.0;
    // 진행 커서: 지금 달리는 구간 = planPts_[planCursor_] -> [planCursor_+1].
    // 단조 증가 (path_planner.hpp advanceCursor 참고)
    size_t planCursor_ = 0;
    bool planActive_ = false;  // PATH를 로봇에 보낸 상태인지
    bool drawRequested_ = false;  // START_DRAW를 받았지만 pose가 없어 접근 전송을 못 한 상태.
                                  // 첫 POS로 pose가 잡히면 자동으로 접근 경로를 보낸다.
    bool awaitingArrival_ = false;  // 1단계(접근) PATH 전송 후 로봇 PATH_DONE 대기 중.
                                    // 도착 통지가 오면 서버가 곧바로 2단계(도색)를 전송한다.
    json activeSegs_;          // 마지막으로 로봇에 보낸 segments (READY 정렬 판정용)
    int alignSegIdx_ = -1;     // 현재 정렬 중/실행 중인 세그먼트 index
    int alignTries_ = 0;       // 그 세그먼트에서 ALIGN을 보낸 횟수
    long lastDriftMs_ = 0;     // 마지막 DRIFT 전송 시각 (전송률 제한용)
    bool manualMode_ = false;  // Qt 수동 조작(조이스틱) 중 - 자동 경로추종/재계획 중단.
                               // 새 BLUEPRINT 수신 시 해제(자동 모드 복귀)
    Pose pose_;                // 로봇 최신 pose (top-view)
    bool poseValid_ = false;
    long lastPlanMs_ = 0;      // 마지막 PATH 전송 시각 (쿨다운용)
};
