#pragma once
// ===== Road-Painter 메시지 프로토콜 (v0.3) =====
// 공통 형식: {"type": "...", "seq": n, "payload": {...}} + 개행(\n)  [TLS 위 JSON Lines]
//
// 좌표계 규약 (v0.3 핵심):
//   - CCTV는 "원본 픽셀 좌표"만 보낸다. 좌표 변환은 전부 서버가 한다.
//     (undistort -> H_marker. 캘리브레이션 데이터는 서버 한 곳에만 존재)
//   - QT는 "바닥 미터 좌표"로 변환을 마친 도면을 보낸다.
//     (top-view 위에 그린 점 = 바닥 평면 위의 점이라 스케일 나눗셈이 전부)
//   - 서버 내부 월드 좌표계: 바닥 평면, 단위 미터.
//
// [클라이언트 -> 서버, 접속 직후 1회]
//   HELLO  payload: {"role":"QT"|"ROBOT"|"CCTV"|"ADMIN"}
//     -> 서버 응답 ACK payload: {"msg":"registered as ROBOT"}
//
// [ADMIN role - 관리자 창(admin_console/web_gui.py)이 사용]
//   목적: (1) 서버가 중계하는 모든 메시지를 엿보고(로그 모니터)
//         (2) 로봇에 명령을 내린다(점검/설치용).
//   서버 -> ADMIN:
//     TAP  payload: {"dir":"IN"|"OUT", "peer":"QT"|"ROBOT"|"CCTV", "msg":{원본메시지}}
//       - dir=IN : peer가 서버로 보낸 메시지 사본 (peer -> 서버)
//       - dir=OUT: 서버가 peer에게 보낸 메시지 사본 (서버 -> peer)
//       - ADMIN 자신과 오간 메시지는 tap하지 않는다(무한 루프 방지).
//   ADMIN -> 서버:
//     CMD  payload: {"cmd":...}   -> ROBOT 전달 (CALIB_START는 CCTV에도)
//     PATH payload: {"segments":[...]}  -> ROBOT 전달 (테스트 경로)
//       - 관리자는 점검/설치용이라, 경로 실행 중이어도 차단 없이 항상 전달한다.
//         (QT 수동조작이 도색 중 차단되는 것과 다름)
//
// [서버 -> 로봇]
//   PATH   payload: {"phase":"approach"|"draw", "segments":[
//              {"op":"TURN","angle_deg":-90},            // 제자리 회전 (+: 좌회전, -: 우회전)
//              {"op":"MOVE","dist_m":2.0,"paint":true,   // 직진 (paint: 도색 여부, 생략=false)
//               "heading_deg":35.0} ]}                   // 이 직진의 목표 절대각도 (정렬용, 로봇은 무시 가능)
//     - 로봇은 좌표를 모르므로 경로는 동작 명령 시퀀스로 전달.
//     - PATH가 오면 기존 경로 즉시 폐기하고 새 경로로 교체 (TCP가 순서 보장).
//     - phase="approach": 도면 시작점까지 이동(전부 paint=false) + 첫 도색 방향으로
//       회전까지. 끝나면 로봇은 그 자리에서 대기 (Qt의 그리기 시작을 기다림).
//       마지막 TURN에는 heading_deg가 실려 READY 정렬 확인 가능.
//     - phase="draw": Qt START_DRAW 후 전송되는 도색 경로(전부 paint=true).
//       로봇은 이 PATH를 받는 순간 IMU 현재 방향을 0도로 세팅하고 주행 시작.
//   ALIGN  payload: {"angle_deg": -2.5}   // READY 응답: 출발 전 미세 회전 보정
//   GO     payload: {}                    // READY 응답: 정렬 OK, 다음 동작 진행
//   DRIFT  payload: {"angle_deg": 2.0}    // 주행(직진) 중 지속 각도 피드백 (~5Hz)
//     - 가려는 방향이 0도 기준. 시계방향(오른쪽)으로 틀어져 있으면 양수,
//       반시계(왼쪽)면 음수. 값 = 좌회전으로 보정해야 할 양 (ALIGN과 동일 규약).
//   CMD    payload: {"cmd": ...}  (응답 불필요, fire-and-forget)
//     - 이벤트: "ESTOP" | "RESUME" | "CALIB_START"
//     - 수동 조작(조이스틱, 누르는 동안 이동 / STOP=뗌, 이동량 없음):
//         "FORWARD" | "BACKWARD" | "TURN_LEFT" | "TURN_RIGHT" | "STOP"
//       수동 CMD가 오면 서버는 자동 경로추종/재계획을 멈춘다(충돌 방지).
//       자동 모드 복귀는 새 BLUEPRINT 수신 시.
//
// [로봇 -> 서버]
//   STATUS payload: {"state":"IDLE"|"MOVING"|"ESTOPPED"|"ERROR", "painting":true}
//     - painting: 노즐 동작 여부 (지금 도색 중인지)
//     - 2초 이내 간격으로 주기 전송 필수 = 하트비트 겸용.
//     - 서버는 로봇에게서 10초간 무수신이면 연결 끊김으로 간주하고 세션 종료.
//   READY  payload: {"seg": 3}
//     - 출발 전 정렬 확인: TURN을 마치고 MOVE를 시작하기 직전에 정지 상태로 전송.
//       seg = segments 배열에서 곧 실행할 MOVE의 인덱스 (0부터).
//     - 서버가 CCTV 마커로 잰 실제 각도와 그 MOVE의 heading_deg를 비교해서
//       오차 > 2도면 ALIGN{angle_deg}(미세 회전 후 다시 READY),
//       오차 <= 2도(또는 3회 반복 초과)면 GO{} 응답. GO를 받으면 직진 시작.
//
// [QT -> 서버]
//   REGISTER payload: {"id":"user1","pw":"..."}
//     -> 응답 REGISTER_OK {"id":...} | REGISTER_FAIL {"reason":...}
//   LOGIN    payload: {"id":"user1","pw":"..."}
//     -> 응답 LOGIN_OK {"id":..., "calib":{...}|null} | LOGIN_FAIL {"reason":...}
//        (calib는 저장된 캘리브레이션 번들, null이면 캘리브레이션 필요)
//   CMD      payload: {"cmd":...}  -> ROBOT 중계 (CALIB_START는 CCTV에도)
//     이벤트 ESTOP/RESUME/CALIB_START + 수동 조작 FORWARD/BACKWARD/TURN_LEFT/
//     TURN_RIGHT/STOP (조이스틱: 누르는 동안 이동, 이동량 없음)
//     + START_DRAW ("그림그리기 시작" 버튼): 접근 완료 대기 중인 로봇에게
//       서버가 도색 경로(PATH phase="draw")를 생성·전송. 로봇 중계는 안 함.
//   BLUEPRINT payload: {"points":[[x,y],...]}  바닥 평면 미터 좌표 폴리라인
//     - Qt가 top-view 픽셀 -> 미터 변환(÷ S px/m)을 마친 값. 서버는 재변환하지 않음.
//     - 서버는 우선 1단계(시작점 접근) PATH만 전송. 도색은 START_DRAW 이후.
//
// [서버 -> QT]
//   STATUS / POS : 그대로 중계 (모니터링용)
//   POSE   payload: {"x":1.234,"y":0.567,"theta_deg":90.0}
//     - 서버가 POS를 변환해 계산한 로봇 pose (바닥 미터 좌표) - top-view 표시용
//   H_MATRIX : 캘리브레이션 갱신 직후 그대로 중계 (top-view 재생성용)
//   PEERS  payload: {"robot":true,"cctv":false}
//     - ROBOT/CCTV 접속·해제될 때마다 전송. QT 접속 직후에도 현재 스냅샷 1회 전송.
//     - QT가 "로봇/CCTV가 지금 붙어있는지"를 STATUS/POS 유무로 유추하지 않고
//       바로 알 수 있게 하는 접속 상태 신호 (2026-07-22 추가)
//
// [CCTV -> 서버]
//   H_MATRIX payload: {"calib":{"version":1, "K":[[...]x3], "D":[k1,k2,p1,p2,k3],
//                      "H_floor":[[...]x3], "H_marker":[[...]x3], "marker_height_m":0.25}}
//     - 캘리브레이션 1회 수행 후 전송. 로그인 사용자에 영속 저장 + QT 중계.
//     - H_floor  : 왜곡 보정된 픽셀 -> 바닥 평면 미터 (Qt top-view 용)
//     - H_marker : 왜곡 보정된 픽셀 -> 마커 장착 높이 평면 미터 (로봇 측위용,
//                  마커가 바닥에서 떠 있어 생기는 시차를 흡수)
//     - 레거시 {"H":[[...]x3]}도 허용 (왜곡/시차 보정 없이 동작)
//   POS      payload: {"corners":[[u,v]x4]}  로봇 마커 4점 = "원본 CCTV 픽셀" 좌표
//     순서 = [전좌, 전우, 후우, 후좌]
//     - CCTV는 절대 좌표 변환하지 말 것 (undistort도 하지 말 것).
//       서버가 undistort(P=K 동등) -> H_marker -> pose 계산까지 담당.
//     - 테스트용으로 {"x","y","theta_deg"}(바닥 미터 좌표)도 허용
//     -> QT 중계 + POSE를 QT 전송 + 계획 경로에서 0.3m 초과 이탈 시
//        재계획 PATH 전송 (최소 3초 간격)
//     - 로봇에는 중계하지 않음: 로봇은 좌표를 모르며(PATH 참고), 위치 보정은
//       서버가 각도로 변환해 ALIGN/DRIFT로만 내려준다
#include <nlohmann/json.hpp>
#include <atomic>
#include <string>

using json = nlohmann::json;

// type/payload 받아서 seq 자동 증가한 메시지 생성
inline json makeMsg(const std::string& type, const json& payload) {
    static std::atomic<long> seq{0};
    return json{{"type", type}, {"seq", ++seq}, {"payload", payload}};
}

// PATH 메시지 생성. phase: "approach"(시작점 접근) | "draw"(도색 경로)
inline json makePathMsg(const json& segments, const std::string& phase) {
    return makeMsg("PATH", {{"phase", phase}, {"segments", segments}});
}
