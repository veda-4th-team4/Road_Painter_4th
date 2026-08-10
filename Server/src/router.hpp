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
//               program이 오면 서버가 그것을 로봇 op으로 "변환"한다 - v1처럼
//               그대로 중계하지 않는다 (부호 반전 + 펜 오프셋 보정 op 삽입 +
//               arc 반지름 치환. ops_builder.hpp buildDrawOps 참고).
//   QT    -> CMD START_DRAW -> 1단계(접근) PATH 생성·전송. 이후는 전부 자동:
//               로봇 PATH_DONE(접근) -> 서버가 2단계(도색) PATH 자동 전송 ->
//               로봇 PATH_DONE(도색) -> QT에 DRAW_DONE 통지.
//               실패/대기 시 QT에 DRAW_FAIL{stage,reason,msg} 통지
//   ROBOT -> STATUS    -> QT 중계
//   ROBOT -> READY     -> "op n을 실행해도 되나?" 서버가 GO / ALIGN / MORE 중
//               정확히 하나로 답한다 (§3, §6). 답을 빠뜨리면 로봇이 영원히 멈춘다.
//   ROBOT -> PATH_DONE -> 받은 PATH를 끝까지 수행했다는 통지. 접근 완료면 서버가
//               곧바로 도색 PATH를 이어 보내고(QT에는 알리지 않음), 도색 완료면
//               QT에 DRAW_DONE을 통지하고 경로 상태를 정리한다.
//   CCTV  -> POS       -> 원본 픽셀 4코너 수신. 서버가 undistort -> H_marker로
//               pose 계산 (좌표 변환은 전부 서버 담당) + 계산된 POSE를 QT로 전송
//               + 주행 중이면 DRIFT. POS 원본은 QT에 중계하지 않는다 (POSE만 사용)
//   CCTV  -> H_MATRIX  -> 캘리브레이션 번들(K,D,H_floor,H_marker) 수신,
//               로그인 사용자에 영속 저장 + QT 중계
//   ROBOT/CCTV 접속·해제 -> QT에 PEERS{"robot":bool,"cctv":bool} 통지
//               (QT 접속 시에도 현재 스냅샷 1회 전송)
//
// 🔴 로봇 대면 규격은 docs/PROTOCOL_v2_ROBOT.md가 정본이다. 아래 주석의 §n은
//   그 문서의 절 번호다. 튜닝 상수는 이 파일에 없다 - 전부 params.hpp/
//   config/params.json에 있다 (params().xxx로 읽는다).
#include "calib.hpp"
#include "ops_builder.hpp"
#include "params.hpp"
#include "path_planner.hpp"
#include "protocol.hpp"
#include "stream_cfg.hpp"
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
    // 주기 호출(main의 tick 스레드, 기본 200ms). 시간이 지나야 판정되는 것들
    // (POS 두절 HOLD, 판정 대기 창, 캘리 타임아웃)을 메시지 수신과 무관하게
    // 돌린다. 🔴 이게 없으면 상대가 조용해진 순간 감시도 같이 멈춘다.
    void tick();

private:
    // 시간 경과로 충족됐을 수 있는 판정들을 한곳에서 회수. mtx_를 쥔 채 부를 것.
    void sweep();
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

    // ----- 채널 (v0.4, router_channel.cpp) -----
    // 지금 보고 있는 채널의 캘리브레이션. 그 채널이 미캘리면 valid=false인 빈 값.
    // 🔴 pose 계산은 반드시 이걸 거칠 것 - 채널마다 H가 달라 좌표계가 다르다.
    const Calib& activeCalib() const;
    // SELECT_CHANNEL: 작업 채널 전환. CCTV로 중계 + QT에 CHANNEL_OK/FAIL 회신.
    void selectChannel(const json& payload, const json& msg);
    // 활성 채널 전환에 딸린 서버 상태 정리만 한다 (전송 없음). selectChannel과
    // startCalib이 공유한다 - 채널을 바꾸면서 pose를 안 버리면 새 채널의 첫 POS가
    // 오기 전까지 서버가 옛 좌표계의 위치를 믿는다.
    void applyChannel(int ch);
    // 진행 중인 도색 작업을 취소한다 (CMD ABORT_DRAW).
    // ESTOP과 달리 "받아둔 경로를 버린다" - 반환값은 실제로 취소할 게 있었는지.
    bool abortDraw();

    // ----- 로봇 주행 호모그래피 세션 (2026-08-10, router_calib.cpp) -----
    // 🔴 불변식: calibActive_가 켜졌다면 반드시 종결 응답 하나로만 꺼진다
    //   (H_MATRIX / CALIB_FAIL / CALIB_CANCELLED). 조용히 끄면 Qt가 대기 화면에
    //   갇힌다 - clearCalib()를 단독으로 부르지 말고 항상 전송과 짝지을 것.
    // origin = "QT" | "ADMIN". 개시자가 둘이라 회신 대상과 검증 강도가 다르다.
    void startCalib(const json& payload, const json& msg, const char* origin);
    void cancelCalib(const json& payload, const json& msg);
    // ROBOT/CCTV의 CALIB_STOPPED 수신. 둘 다 모이면 CALIB_CANCELLED를 보낸다.
    void onCalibStopped(const std::string& role);
    // CCTV가 올린 CALIB_PROGRESS/CALIB_FAIL을 ch/request_id를 채워 QT로 넘긴다.
    void relayCalibProgress(const json& msg);
    void relayCalibFail(const json& payload);
    // 세션이 도는 중이면 CALIB_FAIL을 QT에 보내고 상태를 접는다 (개시자가 ADMIN
    // 이면 QT는 기다리는 게 없으므로 로그만 남긴다).
    void failCalib(const char* reason, const std::string& m);
    // 종결 응답 없이 늘어지는 세션을 서버가 먼저 접는다 (Qt 5분 한도보다 짧게).
    void checkCalibTimeout();
    void clearCalib();

    bool calibActive_ = false;     // 세션이 도는 중인가 (busy 판정의 단일 근거)
    bool calibFromQt_ = false;     // QT가 개시했나 (아니면 ADMIN - QT에 회신 안 함)
    std::string calibReqId_;       // Qt가 준 상관관계 ID (ADMIN 개시면 빈 문자열)
    int calibCh_ = 0;              // 이 세션이 캘리 중인 채널
    long calibStartMs_ = 0;        // 수락 시각 (calib_timeout_ms 기준점)
    bool calibCancelling_ = false;   // CALIB_CANCEL을 중계하고 ACK를 기다리는 중
    long calibCancelMs_ = 0;         // 중계 시각 (calib_cancel_ack_ms 기준점)
    bool cancelAckRobot_ = false;    // ROBOT의 CALIB_STOPPED를 받았나
    bool cancelAckCctv_ = false;     // CCTV의 CALIB_STOPPED를 받았나

    // ----- 로봇 핸드셰이크 (§3, §6) -----
    // READY 수신. 판정이 필요 없는 boundary면 즉시 GO, 필요하면 대기 창을 연다.
    void onReady(int opIndex);
    // 열어둔 대기 창이 다 찼으면 판정한다 (MORE 먼저, 그다음 ALIGN, 아니면 GO).
    void resolveBoundary();
    void sendGo(int opIndex, const char* why);
    void clearBoundary();
    // 이 boundary에서 MORE 판정이 가능한가 (직전 op이 role=path인 move/arc).
    bool needsMore(int k) const;
    // 이 boundary에서 ALIGN 판정이 가능한가. 가능하면 목표 방위(CCW)를 채운다.
    bool needsAlign(int k, double& targetCcw) const;
    // POS 두절 감시 -> HOLD (§7)
    void checkPosLoss();
    // POS 수신/채택/폐기 카운터를 주기적으로 한 줄 요약해 남긴다.
    void logPosStats();

    // Qt의 START_DRAW 수신 시 1단계(시작점 접근) 경로를 로봇에 전송.
    // pose를 아직 모르면 drawRequested_만 세워두고 첫 POS 수신 때 재시도한다.
    void startApproach();
    // 로봇의 접근 완료(PATH_DONE) 수신 시 2단계(도색) 경로를 로봇에 전송
    void sendDrawPath();
    // 생성된 경로를 로봇에 전송하고 실행 상태를 세운다. 실패하면 false.
    bool sendPath(PlannedPath&& path, const std::string& phase);
    // 경로 실행 상태를 전부 비운다 (완료/실패/새 도면 수신 시)
    void clearPath();
    // 경로 생성/전송 실패(또는 대기) 시 Qt에 DRAW_FAIL로 통지
    void sendDrawFail(const char* stage, const char* reason, const std::string& msg);

    // 각 클라이언트 세션 스레드가 onMessage()를 호출하므로 아래 상태 전부가
    // 스레드 간 공유다 (예: QT의 BLUEPRINT가 planPts_를 비우는 동안 CCTV의
    // POS가 순회하면 UB). onMessage() 전체를 이 뮤텍스 하나로 직렬화한다.
    std::mutex mtx_;
    TlsServer& srv_;
    UserStore users_;          // id/pw/H행렬 영속 저장소
    std::string currentUser_;  // 로그인된 사용자 (단일 사용자 가정)
    // 중계 스트림 설정 파일 경로. 로그인마다 다시 읽으므로 값은 들고 있지 않는다
    // (stream_cfg.hpp loadStreamCfg 주석 - 서버 재시작 없이 주소를 바꾸기 위함).
    static constexpr const char* kStreamCfgFile = "config/stream.json";
    // 채널별 캘리브레이션 (v0.4). 채널마다 H가 다르므로 하나로 합칠 수 없다.
    // 단일 채널 현장은 키가 1 하나뿐인 맵으로 그대로 동작한다 (channelOf가
    // ch 없는 payload를 1로 읽어주므로 구버전 CCTV/QT도 수정 없이 붙는다).
    std::map<int, Calib> calibs_;
    int activeChannel_ = kMinChannel;  // 지금 작업 중인 채널 (SELECT_CHANNEL로 전환)
    // 직전에 "활성 채널이 아니라서" 버린 POS의 채널. 같은 채널이 계속 들어올 때
    // 로그를 한 번만 남기기 위한 것 - POS는 15~30Hz라 매번 찍으면 로그가 덮인다.
    int lastIgnoredPosCh_ = 0;
    json blueprint_;   // Qt가 보낸 도면 원본
    json lastStatus_;  // 로봇 최신 상태
    json lastPos_;     // CCTV 최신 마커 검출 원본 (픽셀)

    std::vector<Pt> planPts_;  // 도면 폴리라인 (바닥 미터) = 펜이 지나갈 자취
    // 구간별 도색 여부. planPaint_[i] = "planPts_[i-1] -> planPts_[i] 구간을
    // 칠하는가". planPts_와 길이가 같을 때만 유효(다르면 비운다).
    std::vector<char> planPaint_;
    // Qt가 입력한 동작 시퀀스 원본 (대문자 MOVE/TURN/NOZZLE/ARC, CCW 양수).
    // 비어 있으면 서버가 planPts_로 같은 형식의 시퀀스를 만들어 쓴다
    // (programFromPoints - 변환 경로를 하나로 유지하기 위함).
    json planProgram_;

    bool planActive_ = false;  // PATH를 로봇에 보낸 상태인지
    bool drawRequested_ = false;  // START_DRAW를 받았지만 pose가 없어 접근 전송을 못 한 상태.
                                  // 첫 POS로 pose가 잡히면 자동으로 접근 경로를 보낸다.
    bool awaitingArrival_ = false;  // 1단계(접근) PATH 전송 후 로봇 PATH_DONE 대기 중.
                                    // 도착 통지가 오면 서버가 곧바로 2단계(도색)를 전송한다.

    // ----- 실행 중인 경로 (§3) -----
    json activeOps_;                 // 마지막으로 로봇에 보낸 ops (로그/디버깅용)
    std::vector<OpMeta> activeMeta_;  // 같은 길이. 서버만 아는 판정 정보
    std::string activePhase_;        // "approach" | "draw"
    // 지금 로봇이 실행 중인 op = 마지막으로 GO를 보낸 index. READY가 오면 -1.
    // DRIFT를 "role=path인 move 실행 중"에만 보내기 위한 것 (§6.3).
    int runningOp_ = -1;

    // ----- boundary 판정 상태 (§6.1, §6.2) -----
    int boundaryIdx_ = -1;      // 지금 판정 중인 boundary의 op_index (-1 = 없음)
    int alignTries_ = 0;        // 이 boundary에서 ALIGN을 보낸 횟수
    int moreTries_ = 0;         // 이 boundary에서 MORE를 보낸 횟수
    bool moreSettled_ = false;  // 이 boundary의 MORE 판정을 끝냈는가(=이제 ALIGN 차례)
    int pendingIdx_ = -1;       // 대기 창이 열린 READY의 op_index (-1 = 창 없음)
    long pendingMs_ = 0;        // 그 READY를 받은 시각 (창의 기준 시각)
    // 창 동안 모은 pose 표본. 각도는 반드시 원 위에서 평균낸다(sin/cos 누적) -
    // 도를 그냥 더하면 ±180도 경계에서 깨지고, 이 시스템 heading은 바로 그
    // 근처에 산다. 어차피 기다리는 시간이라 추가 지연 0으로 노이즈가 1/sqrt(N).
    double pendingSin_ = 0, pendingCos_ = 0, pendingX_ = 0, pendingY_ = 0;
    int pendingPoseN_ = 0;
    long lastDriftMs_ = 0;     // 마지막 DRIFT 전송 시각 (전송률 제한용)

    // ----- POS 두절 (§7) -----
    bool holdActive_ = false;  // HOLD{true}를 보내 로봇을 세워둔 상태
    int posRecoverN_ = 0;      // HOLD 중 연속으로 채택된 POS 장수

    bool manualMode_ = false;  // Qt 수동 조작(조이스틱) 중 - 자동 판정 중단.
                               // 새 BLUEPRINT 수신 시 해제(자동 모드 복귀)
    Pose pose_;                // 로봇 최신 pose (top-view)
    bool poseValid_ = false;
    long posSeq_ = 0;          // POS로 pose가 갱신된 누적 횟수
    long lastPoseMs_ = 0;      // 마지막으로 채택된 pose의 시각 (이상치 게이트/두절 판정)
    int poseRejects_ = 0;      // 게이트가 연속으로 버린 프레임 수

    // ----- POS 수신 통계 (params().pos_stat_period_ms마다 요약 후 0으로) -----
    // 예전엔 poseFromPos 실패가 캘리브레이션 없을 때만 로그를 남기고 나머지는
    // 조용히 return이라, POS가 초당 몇 장 들어오는지 서버 로그로 알 방법이
    // 아예 없었다 - 실제로 "새 POS 0/3장" 같은 간접 증거로 역추적해야 했다.
    int posRecv_ = 0, posAccept_ = 0, posParseFail_ = 0;
    int posGateRate_ = 0;
    long posStatMs_ = 0;       // 마지막 요약 시각 (0 = 아직 POS를 한 번도 못 받음)
};
