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
    // origin이 필요하다: 소유자가 아닌 쪽의 취소를 걸러야 하고, 회신도 요청한
    // 쪽에만 가야 한다 (예전엔 인자가 없어 ADMIN이 눌러도 QT에 회신이 갔다).
    void cancelCalib(const json& payload, const json& msg, const char* origin);
    // ROBOT/CCTV의 CALIB_STOPPED 수신. 둘 다 모이면 종결 응답을 보낸다
    // (순수 취소면 CALIB_CANCELLED, 중단 사유가 있으면 CALIB_FAIL - §3-3).
    void onCalibStopped(const std::string& role);
    // 세션을 시작하지 못했을 때의 거절 회신. 아직 calibActive_가 아니라
    // failCalib()을 쓸 수 없는 자리에서 쓴다 (startCalib/startOdoCalib 공용).
    // extra는 payload에 덧붙일 필드 (busy일 때의 owner 등). toQt=false면
    // 로그만 남긴다 - 관리자 창은 TAP으로 서버 로그를 이미 받고 있다.
    void rejectCalib(bool toQt, int ch, const std::string& reqId,
                     const char* origin, const char* reason, const std::string& m,
                     const json& extra = json::object());
    // CCTV가 올린 CALIB_PROGRESS/CALIB_FAIL을 ch/request_id를 채워 QT로 넘긴다.
    void relayCalibProgress(const json& msg);
    void relayCalibFail(const json& payload);
    // 세션이 도는 중이면 CALIB_FAIL을 QT에 보내고 상태를 접는다 (개시자가 ADMIN
    // 이면 QT는 기다리는 게 없으므로 로그만 남긴다). calibIsOdo_ 세션이면
    // 대신 abortOdoCalib()으로 넘긴다 - 로봇이 실제로 굴러가고 있기 때문.
    void failCalib(const char* reason, const std::string& m);
    // 종결 응답 없이 늘어지는 세션을 서버가 먼저 접는다 (Qt 5분 한도보다 짧게).
    void checkCalibTimeout();
    void clearCalib();

    // ----- 세션 소유권 (2026-08-13) -----
    // 🔴 잠금 자체는 mtx_가 이미 한다 - onMessage() 전체가 직렬화되므로 두
    //   CALIB_START가 동시에 통과하는 경합은 구조적으로 불가능하다. 여기서
    //   추가하는 것은 "누가 쥐고 있는가"다. 예전에는 calibFromQt_(bool) 하나로
    //   회신 대상만 구분했는데, 그것만으로는 취소 권한을 판정할 수 없다 -
    //   ADMIN이 request_id 없이 CALIB_CANCEL을 보내면 QT 세션이 그냥 죽었다.
    enum class CalibOwner { NONE, QT, ADMIN };
    CalibOwner calibOwner_ = CalibOwner::NONE;
    // 종결 응답을 QT로 보내야 하는가. calibFromQt_를 대체한다.
    bool calibToQt() const { return calibOwner_ == CalibOwner::QT; }
    static const char* ownerName(CalibOwner o) {
        return o == CalibOwner::QT ? "QT" : o == CalibOwner::ADMIN ? "ADMIN" : "NONE";
    }

    bool calibActive_ = false;     // 세션이 도는 중인가 (busy 판정의 단일 근거)
    std::string calibReqId_;       // Qt가 준 상관관계 ID (ADMIN 개시면 빈 문자열)
    int calibCh_ = 0;              // 이 세션이 캘리 중인 채널
    long calibStartMs_ = 0;        // 수락 시각 (calib_timeout_ms 기준점)
    bool calibCancelling_ = false;   // CALIB_CANCEL을 중계하고 ACK를 기다리는 중
    long calibCancelMs_ = 0;         // 중계 시각 (calib_cancel_ack_ms 기준점)
    bool cancelAckRobot_ = false;    // ROBOT의 CALIB_STOPPED를 받았나
    bool cancelAckCctv_ = false;     // CCTV의 CALIB_STOPPED를 받았나
    // 정지 핸드셰이크가 "실패로 인한 중단"인가. 비어 있으면 순수 취소다.
    // 🔴 이게 없으면 capture_timeout/no_intrinsics 같은 실패가 Qt 화면에
    //   CALIB_CANCELLED("안전하게 중단했습니다")로 떠서, 조작자는 자기가
    //   누르지도 않은 취소가 성공한 줄 안다. 실제로는 캘리가 실패한 것이다.
    std::string calibAbortReason_;
    std::string calibAbortMsg_;

    // ----- 로봇 오도메트리 주행 캘리 (2026-08-12, router_odocalib.cpp) -----
    // calibActive_/calibReqId_/calibCh_/calibStartMs_/calibOwner_ 등 위 필드를
    // 그대로 공유해서 쓴다 (busy 판정이 method 무관하게 걸리도록).
    // calibIsOdo_만으로 "지금 도는 세션이 어느 방식인가"를 구분한다.
    //
    // ⚠️ 2026-08-13부터 QT도 이 방식을 개시할 수 있다. 예전 주석의 "calibFromQt_는
    //    이 방식에서 항상 false"는 더 이상 사실이 아니다 - 그 전제 위에 서 있던
    //    것들(세션이 CALIB_DONE에서 끝난다, 타임아웃에 Qt 제약이 없다)이 전부
    //    바뀌었다. docs/ROBOT_ODOMETRY_HOMOGRAPHY_REQUEST_QT_20260813.md 참고.
    //
    // QT/ADMIN이 CALIB_START{method:"robot_motion", m_cm, n_cm, start_corner}를
    // 보내면 여기로 분기한다 (startCalib() 내부에서 method를 보고 갈라짐).
    // m_cm/n_cm/start_corner 검증은 startCalib()이 이미 마쳤다 - 거기서 해야
    // reject() 하나로 QT 회신과 ADMIN 로그가 같이 처리된다.
    void startOdoCalib(const json& payload, const json& msg,
                       const std::string& reqId, int ch, bool fromQt);
    // 이 세션의 주행 데드라인(CALIB_START -> CALIB_DONE). QT 개시면 Qt 대기
    // 예산에서 결과 대기 몫을 뺀 값으로 깎는다 (params.hpp 주석 참고).
    long odoDriveBudgetMs() const;
    // 주행 진행률을 QT에 합성해 보낸다. 카메라는 이 방식에서 CALIB_PROGRESS를
    // 올리지 않으므로(정적 앵커 방식과 다름) 서버가 정지점 진행으로 만든다.
    // 주행이 2~4분이라 이게 없으면 Qt 대기 화면이 그동안 통째로 비어 있다.
    void sendOdoProgress(const char* phase, int pointIdx);
    // READY(k) 수신, activePhase_=="calib"일 때 onReady()가 위임한다.
    // 캡처 대상 boundary면 CALIB_CAPTURE를 보내고 ack까지 GO를 미룬다.
    void onCalibReady(int k);
    // PATH_DONE(calib) 수신. 9번째(복귀) 캡처 요청 후 CALIB_DONE으로 마감한다.
    void onCalibPathDone();
    // CCTV의 CALIB_CAPTURE_OK/CALIB_CAPTURE_FAIL 수신.
    void onCalibCaptureAck(const json& payload, bool ok, const std::string& reason);
    // 캡처 1건에 대해 CALIB_CAPTURE를 보내고 대기 상태를 연다.
    void sendCalibCapture(int pointIdx);
    // 캡처 ack가 calib_capture_timeout_ms 안에 안 오면 호출 (sweep()에서).
    void checkOdoCaptureTimeout();
    // 폐합오차(idx0과 idx8은 같은 물리 지점이므로 두 픽셀은 같아야 한다)를
    // 로그 한 줄로 남긴다. 🔴 진단 전용 - 판정도 통지도 하지 않는다.
    void logOdoClosure();
    // 안전 정지: 로봇은 fire-and-forget ABORT_DRAW, CCTV는 CALIB_CANCEL 후
    // CALIB_STOPPED 대기. 정적 앵커 방식의 cancelCalib()/failCalib()과 다른
    // 경로다 - 로봇이 실제로 주행 중이라 CALIB_CANCEL이 로봇에 안 먹는다
    // (로봇 펌웨어에 CALIB_* 핸들러가 없음). 상세는 위 계획서 §7.
    void abortOdoCalib(const char* reason, const std::string& msg);

    bool calibIsOdo_ = false;       // 지금 세션이 robot_motion 방식인가
    // CALIB_DONE을 보내고 카메라의 H_MATRIX/CALIB_FAIL을 기다리는 중인가.
    // 🔴 QT 개시 세션에서만 켜진다. 예전에는 CALIB_DONE 자리에서 그냥
    //   clearCalib()을 불러 세션이 끝났는데, 그러면 뒤늦게 오는 H_MATRIX가
    //   request_id 없이 중계되고(종결로 안 잡힘) 카메라의
    //   CALIB_FAIL{too_few_points}은 "진행 중인 세션 없음"으로 버려진다 -
    //   둘 다 Qt를 타임아웃까지 대기 화면에 가둔다. ADMIN 개시는 기다리는
    //   쪽이 없으므로 예전대로 CALIB_DONE에서 끝낸다.
    bool odoAwaitingResult_ = false;
    long odoResultWaitMs_ = 0;      // 결과 대기 시작 시각 (calib_odo_result_wait_ms 기준점)
    double odoMmm_ = 0, odoNmm_ = 0;  // 사각형 치수 (mm)
    bool odoCcw_ = true;             // start_corner=="bottom_left"
    int odoPointIdx_ = -1;           // 지금 CCTV 응답을 기다리는 point_index (-1=없음)
    // ack를 받으면 이 op_index에 GO를 보낸다. -2 = 아직 캡처 요청 전.
    // -1 = 이번 캡처가 PATH_DONE 트리거였다는 뜻 - ack 후 GO 대신 CALIB_DONE.
    int odoPendingGoOp_ = -2;
    long odoCaptureMs_ = 0;          // CALIB_CAPTURE 전송 시각 (캡처 타임아웃 기준)
    int odoValidCount_ = 0;          // 유효 캡처 개수 (idx 0~7 중 OK, findHomography 입력)
    // 정지점 9곳의 raw 픽셀. 폐합오차(idx0 vs idx8)와 그 mm 환산에 쓴다 -
    // 환산 스케일을 네 변의 실측 픽셀 길이에서 뽑기 때문에 코너 픽셀이 필요하다.
    // 진단 로그 전용이다. 서버는 이 값으로 아무 판정도 하지 않는다 (H 산출과
    // 유효점 판정은 전부 카메라 몫, wire 스펙 §7).
    double odoPixU_[9] = {0}, odoPixV_[9] = {0};
    bool odoPixOk_[9] = {false};

    // ----- 채널 간 정합 (registration, 2026-08-15 신설, router_registration.cpp) -----
    //
    // 오도메트리(위)로 구한 H_marker는 채널마다 world 원점이 다르다(그 세션
    // 로봇 출발점) - 겹치는 FOV 구역에서 두 채널이 "같은 순간, 같은 로봇 위치"를
    // 본 correspondence 점들로, ch_b의 좌표를 ch_a 기준 좌표계로 옮기는
    // 닮음변환을 계산하는 건 **카메라 앱**이 한다(ArucoPosePNM HomographyMapper::
    // FinishRegistration). 서버가 하는 일은 그 계산에 필요한 REGISTER_CAPTURE를
    // 주기적으로 보내는 것과, 세션 하나를 관리하는 것뿐 - 계산도 저장도 안 한다.
    //
    // 🔴 로봇 경로가 없다. FOV 겹침 구역이 아직 실측되지 않아 자동 사각형 경로를
    //   (오도메트리처럼) 짤 수 없다 - 그래서 조작자가 조이스틱(수동 조작, §QT CMD
    //   FORWARD/TURN_*)으로 로봇을 겹침 구역에 직접 몬다. 그동안 이 세션은
    //   REGISTER_CAPTURE를 일정 주기로 반복 발사한다 - 로봇이 실제로 두 채널
    //   시야 안에 있을 때만 카메라가 OK로 답하고, 아닐 때는 그냥 FAIL(not_both_seen)
    //   이라 조용히 넘어간다(오도메트리의 READY/GO 핸드셰이크와 달리 실패해도
    //   세션을 접지 않는다 - 다음 주기에 다시 시도).
    //
    // 오도메트리 세션(calibActive_ 등)과 완전히 분리된 상태를 쓴다 - 이유는
    // calibCh_가 채널 하나뿐인데, 정합은 항상 두 채널(ch_a/ch_b)이 동시에
    // 필요하기 때문이다. 대신 startRegistrationCollect()가 calibActive_ 를
    // 검사해서, 관련 채널의 오도메트리/앵커 세션이 도는 중이면 거절한다 -
    // 그 채널의 hmm_이 지금 바뀌는 중인데 그 위에서 정합을 모으면 의미가 없다.
    //
    // ADMIN(관리자 창) 전용이다 - QT 소유권/대기 budget 개념이 없다. 관리자
    // 창은 서버 로그를 TAP으로 이미 받고 있으므로, 거절/진행 상황은 회신
    // 메시지 없이 로그(logf)로만 남긴다 (rejectCalib()의 toQt=false 분기와
    // 같은 판단).
    //
    // CMD REGISTER_COLLECT_START{ch_a, ch_b} 로 연다 (fromAdmin()).
    void startRegistrationCollect(const json& payload);
    // CMD REGISTER_COLLECT_STOP(정상 종료, REGISTER_DONE) /
    //     REGISTER_COLLECT_CANCEL(중단, REGISTER_CANCEL) 로 닫는다.
    void stopRegistrationCollect(bool cancel, const char* why);
    // sweep()에서 매 tick 호출. 주기가 찼으면 REGISTER_CAPTURE를 새로 보내고,
    // 세션 전체 타임아웃(reg_session_timeout_ms)이 지났으면 안전하게 접는다 -
    // 켜둔 채 잊어버렸을 때의 워치독(오도메트리의 checkCalibTimeout()과 같은 자리).
    void checkRegistrationTick();
    void sendRegisterCapture();
    // CCTV의 REGISTER_CAPTURE_OK/FAIL 수신. 오도메트리와 달리 이 ack는 다음
    // 캡처 발사를 막지 않는다 - 그냥 통계(regOkCount_/regFailCount_)에 반영하고
    // 로그만 남긴다. 로봇 경로가 없어 "이 캡처가 끝나야 다음으로 넘어간다"는
    // 동기화 자체가 필요 없기 때문이다.
    void onRegisterCaptureAck(const json& payload, bool ok, const std::string& reason);
    // CCTV의 REGISTER_FAIL(세션 수준 실패, 예: fit_failed) 수신 - 세션을 접는다.
    void onRegisterFail(const json& payload);
    // CCTV의 REGISTER_STOPPED(REGISTER_CANCEL/DONE에 대한 ack) 수신.
    void onRegisterStopped(const json& payload);
    void clearRegistration();

    bool regActive_ = false;        // 수집 세션이 도는 중인가
    int regChA_ = 0, regChB_ = 0;   // 참여 채널 (1-based, wire 그대로 - 카메라와 규약 통일)
    std::string regReqId_;          // 이 세션의 상관관계 ID (서버가 발급)
    long regStartMs_ = 0;           // 세션 시작 시각 (reg_session_timeout_ms 기준점)
    long regLastCaptureMs_ = 0;     // 마지막 REGISTER_CAPTURE 전송 시각 (주기 기준점)
    int regPointIdx_ = 0;           // 다음 캡처에 실을 point_index (0부터 순증 - 상한 없음,
                                     // AddCorrespondencePoint가 kMaxAnchors=24로 알아서 막는다)
    int regOkCount_ = 0, regFailCount_ = 0;  // 진행 통계 (로그/대시보드 표시용)
    bool regStopping_ = false;      // REGISTER_DONE/CANCEL 전송 후 REGISTER_STOPPED 대기 중
                                     // (그 동안은 checkRegistrationTick이 새 캡처를 안 쏜다)
    long regStopMs_ = 0;            // 그 전송 시각 (ack 타임아웃 기준점 - 못 받아도 그냥 접는다)

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

    // ----- 주행 중 각도의 이동평균 (2026-08-14) -----
    // 꼭짓점 판정(ALIGN/MORE)은 1초간 모은 POS의 평균을 쓰는데, 주행 중
    // DRIFT는 방금 들어온 값 하나로 바로 계산해 쏘고 있었다. 이상치 게이트를
    // 통과할 만큼 작게 튀는 잡음(몇 도)은 그대로 실려 로봇을 틀어버린다.
    // 직진 중 목표 heading은 상수이므로 평균이 정당하다 - 지연 대신 잡음을
    // 1/sqrt(N)로 줄인다. 도 단위로 더하면 ±180도 경계에서 깨지므로 sin/cos.
    static constexpr int kDriftAvgN = 3;
    double driftSin_[kDriftAvgN] = {0}, driftCos_[kDriftAvgN] = {0};
    int driftN_ = 0, driftPos_ = 0;   // 채워진 개수, 다음에 덮어쓸 자리
    void clearDriftAvg() { driftN_ = 0; driftPos_ = 0; }

    // ----- STATUS 로그 억제 (2026-08-18) -----
    // 로봇 STATUS는 2Hz 하트비트라 그대로 찍으면 초당 두 줄씩 로그를 덮는다.
    // 값이 바뀐 순간만 남긴다 - Qt 중계(sendTo)와 lastStatus_ 갱신은 그대로다.
    // 로봇이 살아있다는 것은 [접속]/[해제]와 POS 요약이 이미 보여주므로,
    // 같은 값의 반복은 정보가 아니라 소음이다.
    // 로봇이 빠지면 비운다 - 재접속 후 첫 STATUS는 값이 같아도 한 줄 남아야
    // "다시 붙어서 이 상태였다"가 로그에 보인다.
    std::string lastStatusState_;
    bool lastStatusPainting_ = false;
    bool lastStatusSeen_ = false;   // 한 번이라도 찍었나 (첫 STATUS는 무조건 남김)
    void clearStatusLog() {
        lastStatusState_.clear();
        lastStatusPainting_ = false;
        lastStatusSeen_ = false;
    }

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
