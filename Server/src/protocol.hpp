#pragma once
// ===== Road-Painter 메시지 프로토콜 (v0.2) =====
// 공통 형식: {"type": "...", "seq": n, "payload": {...}} + 개행(\n)  [TLS 위 JSON Lines]
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
//   CMD    payload: {"cmd":"ESTOP"|"RESUME"|"CALIB_START"}  (응답 불필요, fire-and-forget)
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
//     -> 응답 LOGIN_OK {"id":..., "H":[[...]]|null} | LOGIN_FAIL {"reason":...}
//        (H는 저장된 호모그래피, null이면 캘리브레이션 필요)
//   CMD      payload: {"cmd":...}  -> ROBOT 중계 (CALIB_START는 CCTV에도)
//   BLUEPRINT payload: {"points":[[x,y],...]}  top-view 미터 좌표 폴리라인
//     (Qt팀과 확정 전 디폴트 포맷. 서버가 로봇 위치와 조합해 PATH 생성·전송)
//
// [CCTV -> 서버]
//   H_MATRIX payload: {"H":[[...]x3]} -> 로그인 사용자에 영속 저장 + QT 중계
//   POS      payload: {"corners":[[u,v]x4]}  로봇 마커 4점 (CCTV 픽셀)
//     순서 = [전좌, 전우, 후우, 후좌] (CCTV팀과 확정 전 디폴트 가정)
//     테스트용으로 {"x","y","theta_deg"}(top-view)도 허용
//     -> ROBOT/QT 중계 + 서버가 pose 계산, 계획 경로에서 0.3m 초과 이탈 시
//        재계획 PATH 전송 (최소 3초 간격)
//
// [CCTV -> 서버]
//   H_MATRIX payload: {"H":[[...]]} -> 로그인 사용자에 영속 저장 + QT 즉시 중계
//   POS      payload: 로봇 마커 4점 검출 결과 -> ROBOT + QT 중계
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
