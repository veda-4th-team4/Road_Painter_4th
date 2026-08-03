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
//               2026-07-28: paint[] / program[] 선택 필드 추가. program이 오면
//               서버는 도색 동작 시퀀스를 생성하지 않고 Qt가 입력한 MOVE/TURN/
//               NOZZLE 값을 그대로 로봇에 넘긴다. 펜 오프셋 보정은 로봇이 TURN
//               실행 시 자체적으로 한다 (서버/Qt 둘 다 관여하지 않음).
//   QT    -> CMD START_DRAW -> 1단계(접근) PATH 생성·전송. 이후는 전부 자동:
//               로봇 PATH_DONE(접근) -> 서버가 2단계(도색) PATH 자동 전송 ->
//               로봇 PATH_DONE(도색) -> QT에 DRAW_DONE 통지.
//               실패/대기 시 QT에 DRAW_FAIL{stage,reason,msg} 통지
//   QT    -> CMD ABORT_DRAW -> 경로 실행 상태를 전부 버리고 ROBOT에 중계
//               (로봇도 받아둔 PATH를 버린다). 도면은 남긴다 - 다시 START_DRAW를
//               누르면 같은 도면으로 처음부터. -> QT에 DRAW_ABORTED{was_active}
//   QT    -> CMD SELECT_CHANNEL{ch} -> 활성 채널 전환 + CCTV 중계
//               -> QT에 CHANNEL_OK{ch,calib} (그 채널의 번들. null이면 미캘리)
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
//               payload.ch = 이 마커를 본 채널(없으면 1). 활성 채널이 아니면 무시.
//   CCTV  -> H_MATRIX  -> 캘리브레이션 번들(K,D,H_floor,H_marker) 수신,
//               로그인 사용자에 영속 저장 + QT 중계.
//               payload.ch = 이 번들의 채널(없으면 1). 채널별로 따로 저장된다.
//   ROBOT/CCTV 접속·해제 -> QT에 PEERS{"robot":bool,"cctv":bool} 통지
//               (QT 접속 시에도 현재 스냅샷 1회 전송)
#include "calib.hpp"
#include "path_planner.hpp"
#include "protocol.hpp"
#include "tls_server.hpp"
#include "user_store.hpp"
#include <map>
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
    // Qt의 ABORT_DRAW 수신 시 경로 실행 상태를 전부 버린다 (도면은 남긴다).
    // 로봇 중계는 호출부에서 한다 - 이 함수는 서버 상태만 정리한다.
    // 반환값 = 실제로 진행 중이던 작업을 껐는지 (DRAW_ABORTED.was_active).
    bool abortDraw();
    // Qt의 SELECT_CHANNEL 수신 시 활성 채널 전환 + CCTV 중계 + CHANNEL_OK 회신
    void selectChannel(const json& payload, const json& msg);
    // 현재 활성 채널의 캘리브레이션 (없으면 valid=false인 빈 Calib)
    const Calib& activeCalib() const;
    // 로봇의 접근 완료(PATH_DONE) 수신 시 2단계(도색) 경로를 로봇에 전송
    void sendDrawPath();
    // 이탈 복귀 경로. planPts_[k]로 돌아가는 구간만 새로 만들고, Qt program이
    // 있으면 재개 지점(v >= k인 첫 op)부터 원본을 잘라 이어 붙인다.
    json buildRecovery(size_t k);
    // 경로 생성/전송 실패(또는 대기) 시 Qt에 DRAW_FAIL로 통지
    void sendDrawFail(const char* stage, const char* reason, const std::string& msg);

    // 러프 디폴트 (추후 현장 튜닝)
    static constexpr double kDevThresholdM = 0.3;    // 이탈 판정 거리
    static constexpr long kReplanCooldownMs = 3000;  // 재계획 최소 간격
    static constexpr double kAlignThresholdDeg = 2.0;  // 출발 전 정렬 허용 오차
    static constexpr int kAlignMaxTries = 4;  // ALIGN 최대 반복 (초과 시 그냥 GO)
    static constexpr long kDriftPeriodMs = 200;  // 주행 중 각도 피드백(DRIFT) 최소 간격
    // 펜(노즐)이 마커 중심 뒤로 떨어진 거리 d. 로봇 실측값 = 155mm
    // (rpi-robot PathFollower.h NOZZLE_OFFSET_M = 0.155f). BLUEPRINT 필드이
    // 아니라 서버 상수 - 펜 보정 자체는 로봇이 스스로 하고, 서버는 이 값을
    // 이탈 판정(distToActiveSegment)에서만 쓴다.
    static constexpr double kPenOffsetM = 0.155;
    // 진행 커서를 다음 꼭짓점으로 넘기는 도달 반경.
    // 🔴 펜 오프셋(0.155)보다 크고 이탈 임계값(0.3)보다 작아야 한다.
    //   - d보다 작으면: 펜이 꼭짓점에 닿았을 때 중심은 이미 d를 지나쳐 있어
    //     "도달"로 안 잡히고, 커서가 지나침 판정에만 의존하게 된다.
    //   - 이탈 임계값보다 크면: 커서가 너무 일찍 넘어가 꼭짓점 직전의 정상
    //     주행이 "다음 구간에서 벗어남"으로 오탐된다.
    static constexpr double kVertexReachM = 0.20;

    // 각 클라이언트 세션 스레드가 onMessage()를 호출하므로 아래 상태 전부가
    // 스레드 간 공유다 (예: QT의 BLUEPRINT가 planPts_를 비우는 동안 CCTV의
    // POS가 순회하면 UB). onMessage() 전체를 이 뮤텍스 하나로 직렬화한다.
    std::mutex mtx_;
    TlsServer& srv_;
    UserStore users_;          // id/pw/H행렬 영속 저장소
    std::string currentUser_;  // 로그인된 사용자 (단일 사용자 가정)
    // 채널별 캘리브레이션 (현재 세션. raw는 각 Calib의 .raw).
    // 채널마다 렌즈 방향이 달라 K/D/H가 전부 다르다 - 하나로 공유하면 채널을
    // 바꾸는 순간 좌표가 통째로 틀린다. 그것도 에러 없이 조용히.
    std::map<int, Calib> calibs_;
    // 지금 로봇을 보고 있는 채널. Qt의 SELECT_CHANNEL로 바뀐다.
    // 🔴 활성 채널과 다른 ch의 POS는 무시한다 - 다른 채널이 우연히 로봇 마커를
    //    잡았을 때 pose가 두 채널 사이에서 튀는 것을 막기 위함.
    int activeChannel_ = kMinChannel;
    json blueprint_;   // Qt가 보낸 도면 원본
    json lastStatus_;  // 로봇 최신 상태
    json lastPos_;     // CCTV 최신 마커 검출 원본 (픽셀)

    std::vector<Pt> planPts_;  // 도면 폴리라인 (바닥 미터) = 펜이 지나갈 자취
    // 구간별 도색 여부. planPaint_[i] = "planPts_[i-1] -> planPts_[i] 구간을
    // 칠하는가". planPts_와 길이가 같을 때만 유효(다르면 비운다). 도형 여러 개를
    // 한 폴리라인으로 이어 보낼 때 도형 사이 이동을 칠하지 않기 위한 것.
    std::vector<char> planPaint_;
    // Qt가 입력한 동작 시퀀스 원본(MOVE/TURN/NOZZLE, dist_m/angle_deg는 Qt
    // 입력값 그대로). 비어 있지 않으면 서버는 도색 경로를 생성하지 않고
    // 이것을 그대로 로봇에 넘긴다. 펜 오프셋 보정은 여기 들어있지 않다 -
    // 로봇이 TURN을 실행할 때 스스로 한다.
    json planProgram_;
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
    // 마지막으로 "활성 채널이 아니라서 버린" POS의 채널 (0 = 없음).
    // POS는 15~30Hz라 버릴 때마다 로그를 찍으면 뒤덮인다 - 채널이 바뀔 때만
    // 한 줄 남기려고 들고 있는 값이다.
    int lastIgnoredPosCh_ = 0;
    long lastPlanMs_ = 0;      // 마지막 PATH 전송 시각 (쿨다운용)
};
