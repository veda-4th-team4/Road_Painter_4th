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
    // READY 정렬 판정 본체(ALIGN 또는 GO 회신). READY 수신 즉시 호출되거나,
    // ALIGN 반영을 기다리느라 유예됐다면 새 POS가 충분히 쌓인 뒤 호출된다.
    void judgeReady(int seg);
    // 유예해 둔 READY가 있으면 조건(새 POS 확보 or 타임아웃)을 확인해 판정한다.
    void resolvePendingReady();
    // 유예 상태(대기 중인 seg + 모아둔 theta)를 비운다.
    void clearPendingReady();
    // POS 수신/채택/폐기 카운터를 주기적으로 한 줄 요약해 남긴다.
    void logPosStats();
    // 캘리브레이션 번들 수신 처리 (CCTV/ADMIN 공용): 저장 + Qt 중계
    void handleHMatrix(const json& msg);
    // Qt의 START_DRAW 수신 시 1단계(시작점 접근) 경로를 로봇에 전송.
    // pose를 아직 모르면 drawRequested_만 세워두고 첫 POS 수신 때 재시도한다.
    void startApproach();
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
    // 출발 전 정렬 허용 오차.
    // 🔴 2.0 -> 4.0 (2026-08-03). 정렬 루프의 "1회 오차"가 임계값과 같은 크기라
    //   수렴할 수 없었다: ① 로봇 메인 루프가 80ms라 회전 중 1틱=1.42°를 눈 감고
    //   더 돈다(400 SPS / 22.58스텝도) - 오버슛이 항상 목표를 지나치는 한쪽 방향
    //   계통 편향이다. ② 서버 theta는 마커 코너 픽셀에서 매 프레임 생으로 뽑아
    //   노이즈가 sigma ~= 코너오차(mm)/마커한변(mm) rad (마커 13cm 기준 1px당 약
    //   0.7도). 둘이 겹치면 ALIGN 직후 잔차가 2도를 넘는 일이 흔해 kAlignMaxTries를
    //   태우고 정렬이 안 된 채 GO가 나갔다. 제어 정밀도는 1회 오차의 3배는 잡아야
    //   한다. 로봇이 감속 접근(또는 예측 정지)을 넣어 오버슛을 줄이면 다시 내릴 것.
    //   ⚠️ 정지 상태 theta 노이즈 실측 전까지는 러프 디폴트다.
    static constexpr double kAlignThresholdDeg = 4.0;
    static constexpr int kAlignMaxTries = 4;  // ALIGN 최대 반복 (초과 시 그냥 GO)
    // 주행 중 각도 피드백(DRIFT) 최소 간격.
    // 🔴 200 -> 400ms (2026-08-04). 현장 POS 실측이 1~2Hz라(kPosStatPeriodMs 요약
    //   참고) 200ms(5Hz) 상한은 애초에 병목이 아니었다 - POS가 도착하는 족족
    //   내보내던 것과 다를 바 없었다. 주기를 늘려도 응답성 손해는 없고, 대신
    //   로봇 쪽 로그/UART 부담과 DRIFT 스팸을 줄인다.
    static constexpr long kDriftPeriodMs = 400;
    // ALIGN 이후 온 READY를 재판정하기 전에 무조건 기다리는 고정 시간.
    // 기준점은 ALIGN 송신 시각이 아니라 READY 수신 시각이다 - ALIGN을 보낸
    // 직후부터 세면 로봇이 회전하는 동안 흘러간 시간까지 포함돼 버려서,
    // 정작 "회전이 끝난 뒤의 장면"을 못 보고 판정하게 된다.
    // 🔴 왜 필요한가: 로봇은 스텝 카운터가 목표에 닿는 즉시 READY를 다시 보내는데,
    //   미세회전은 0.1~0.3초짜리라 그 회전이 CCTV -> 서버 pose에 반영되기 전에
    //   READY가 도착한다. 그대로 판정하면 "회전 전 각도"로 같은 ALIGN을 또
    //   계산해 과회전 -> 반대 방향 ALIGN -> 진동이 나고, kAlignMaxTries를 태워
    //   정렬이 안 된 채 GO가 나간다.
    // 🔴 2026-08-04: "새 POS N장이 쌓일 때까지"(프레임 카운트 기반)에서
    //   "고정 시간만큼"으로 단순화했다. 현장 POS 실측이 1~2Hz로 들쭉날쭉해서,
    //   프레임 카운트 기준은 실제로는 거의 항상 타임아웃(당시 1000ms)에 먼저
    //   걸렸다 - "3장 쌓이면 즉시 판정"이라는 지연 최소화 의도가 현장에서는
    //   실현되지 않고 오히려 판정 시점만 들쭉날쭉하게 만들었다. 고정 2초 대기가
    //   더 단순하고, 실측 1~2Hz 기준으로도 대부분 2~4장의 POS가 쌓여 평균 효과는
    //   그대로 얻는다. 그래도 한 장도 못 받으면(POS 완전 두절) 같은 ALIGN을
    //   반복하지 않고 GO로 빠진다 (resolvePendingReady 참고).
    static constexpr long kAlignWaitMs = 2000;

    // ----- POS 이상치 게이트 (운동학 기반) -----
    // 로봇이 물리적으로 낼 수 있는 최대 회전속도는 알려져 있다: TURN 400 SPS,
    // 15433스텝/m, 축간거리 ~0.167m -> 2*(400/15433)/0.167 = 0.31 rad/s = 17.8도/s.
    // 그보다 훨씬 빠른 각도 변화는 주행이 아니라 검출 오류다 - 코너 순서가 한 칸
    // 돌아가면 theta가 정확히 90/180도 튀고, 부분 가림·반사로 코너 하나만 어긋나도
    // 수십 도가 어긋난다. 한 프레임이면 엉뚱한 ALIGN/DRIFT가 나가기 충분한데
    // 지금까지 아무 검사가 없었다.
    // 허용치 = 상수항(측정 노이즈 몫) + 속도항(실제 회전 몫). 나눗셈이 없어 프레임
    // 간격이 0에 가까워도 터지지 않고, 간격이 길면 자연히 헐거워진다.
    static constexpr double kPoseGateBaseDeg = 3.0;    // 노이즈 몫 (sigma ~0.7도의 4배)
    static constexpr double kPoseGateRateDps = 40.0;   // 물리 상한 17.8도/s에 2배 여유
    // ⚠️ 2026-08-04에 두 가지를 덧붙였다가 되돌렸다. 남겨두는 이유는 같은 아이디어가
    //   다시 나오기 때문이다.
    //   ① 허용치 상한(45도): 간격이 벌어질수록 게이트가 헐거워지는 게 싫어서 넣었는데,
    //      POS가 1~2Hz인 현장에서는 간격이 5~8초까지 벌어지고 그동안 로봇은 물리
    //      상한으로도 100도 넘게 정당하게 돈다. 실측 로그에서 49도 회전이 5회 연속
    //      폐기되어 재동기까지 갔다 - 상한이 정상 데이터를 잘랐다.
    //   ② "각도는 급변인데 중심은 제자리 = 코너 순서 뒤집힘" 판정: 차동구동 로봇의
    //      제자리 회전은 회전 중심이 곧 바퀴 중심이라 마커 중심이 **정확히 제자리**다.
    //      즉 이 조합은 뒤집힘의 지문이 아니라 정상 회전의 지문이다. 실측 로그에서
    //      4번 발동했고 4번 다 ALIGN 미세회전 중이었다(오탐 100%).
    //   교훈: 폐기는 연쇄한다. 버려진 프레임은 lastPoseMs_를 갱신하지 않아 다음
    //   프레임의 dt와 dTheta가 더 커지고, kPoseRejectMax를 채울 때까지 계속 버린다.
    //   POS가 초당 한 장인 상황에서 한 장은 비싸다 - 뒤집힘을 한 번 놓치면 다음
    //   ALIGN이 잡아주지만, 정상 프레임을 버리면 정렬 자체를 못 한다.
    // 연속 거부 한도. 넘으면 그냥 받아들여 재동기한다 - 로봇을 들어 옮겼거나
    // 추적을 놓친 뒤 새 위치로 잡힌 경우 영원히 거부만 하면 복구가 안 된다.
    static constexpr int kPoseRejectMax = 5;

    // POS 수신/폐기 요약 로그 주기. POS가 완전히 끊긴 것도 보이도록 POS 핸들러가
    // 아니라 onMessage() 말단에서 찍는다 (로봇 STATUS가 heartbeat 역할).
    static constexpr long kPosStatPeriodMs = 10000;
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
    Calib calib_;      // 캘리브레이션 (현재 세션. raw는 calib_.raw)
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
    long posSeq_ = 0;          // POS로 pose가 갱신된 누적 횟수 (신선도 판정용)
    long lastPoseMs_ = 0;      // 마지막으로 채택된 pose의 시각 (이상치 게이트용)
    int poseRejects_ = 0;      // 게이트가 연속으로 버린 프레임 수 (kPoseRejectMax)
    // ----- POS 수신 통계 (kPosStatPeriodMs마다 요약 후 0으로) -----
    // 예전엔 poseFromPos 실패가 캘리브레이션 없을 때만 로그를 남기고 나머지는
    // 조용히 return이라, POS가 초당 몇 장 들어오는지 서버 로그로 알 방법이
    // 아예 없었다 - 실제로 "새 POS 0/3장" 같은 간접 증거로 역추적해야 했다.
    int posRecv_ = 0, posAccept_ = 0, posParseFail_ = 0;
    int posGateRate_ = 0;
    long posStatMs_ = 0;       // 마지막 요약 시각 (0 = 아직 POS를 한 번도 못 받음)
    int pendingReadySeg_ = -1;   // ALIGN 반영을 기다리며 유예해 둔 READY의 seg (-1 = 없음)
    long pendingPosSeq_ = 0;     // 그 READY를 받은 시점의 posSeq_ (여기서부터 새 프레임을 센다)
    long pendingReadyMs_ = 0;    // 그 READY를 받은 시각 (kAlignWaitMs 대기 기준)
    // 유예 중 모은 theta를 원 위에서 평균내기 위한 누적(각도를 그냥 더하면 ±180도
    // 경계에서 깨진다 - 이 시스템 heading은 바로 그 근처에 산다). 어차피 기다리는
    // 시간이라 추가 지연 0으로 노이즈가 1/sqrt(N)로 준다.
    double pendingSin_ = 0, pendingCos_ = 0;
    int pendingPoseN_ = 0;
    long lastDriftMs_ = 0;     // 마지막 DRIFT 전송 시각 (전송률 제한용)
    bool manualMode_ = false;  // Qt 수동 조작(조이스틱) 중 - 자동 경로추종/재계획 중단.
                               // 새 BLUEPRINT 수신 시 해제(자동 모드 복귀)
    Pose pose_;                // 로봇 최신 pose (top-view)
    bool poseValid_ = false;
    long lastPlanMs_ = 0;      // 마지막 PATH 전송 시각 (쿨다운용)
};
