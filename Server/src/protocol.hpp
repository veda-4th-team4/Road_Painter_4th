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
//   HELLO  payload: {"role":"QT"|"ROBOT"|"CCTV"}
//     -> 서버 응답 ACK payload: {"msg":"registered as ROBOT"}
//
// [서버 -> 로봇]
//   PATH   payload: {"segments":[
//              {"op":"MOVE","dist_m":2.0,"paint":true},  // 직진 (paint: 도색 여부, 생략=false)
//              {"op":"TURN","angle_deg":-90} ]}          // 제자리 회전 (+: 좌회전, -: 우회전)
//     - 로봇은 좌표를 모르므로 경로는 동작 명령 시퀀스로 전달.
//     - PATH가 오면 기존 경로 즉시 폐기하고 새 경로로 교체 (TCP가 순서 보장).
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
//   BLUEPRINT payload: {"points":[[x,y],...]}  바닥 평면 미터 좌표 폴리라인
//     - Qt가 top-view 픽셀 -> 미터 변환(÷ S px/m)을 마친 값. 서버는 재변환하지 않음.
//     - 서버가 로봇 위치와 조합해 PATH 생성·전송.
//
// [서버 -> QT]
//   STATUS / POS : 그대로 중계 (모니터링용)
//   POSE   payload: {"x":1.234,"y":0.567,"theta_deg":90.0}
//     - 서버가 POS를 변환해 계산한 로봇 pose (바닥 미터 좌표) - top-view 표시용
//   H_MATRIX : 캘리브레이션 갱신 직후 그대로 중계 (top-view 재생성용)
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
//     -> ROBOT/QT 중계 + POSE를 QT 전송 + 계획 경로에서 0.3m 초과 이탈 시
//        재계획 PATH 전송 (최소 3초 간격)
#include <nlohmann/json.hpp>
#include <atomic>
#include <string>

using json = nlohmann::json;

// type/payload 받아서 seq 자동 증가한 메시지 생성
inline json makeMsg(const std::string& type, const json& payload) {
    static std::atomic<long> seq{0};
    return json{{"type", type}, {"seq", ++seq}, {"payload", payload}};
}

// PATH 메시지 생성
inline json makePathMsg(const json& segments) {
    return makeMsg("PATH", {{"segments", segments}});
}
