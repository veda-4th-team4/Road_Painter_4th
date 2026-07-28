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
//     LOGIN payload: {"id":"user1","pw":"..."}   (2026-07-27 추가)
//       -> 응답 LOGIN_OK/LOGIN_FAIL을 ADMIN에게 회신 (QT와 동일 처리)
//       - 캘리브레이션(H_MATRIX)은 "그 시점에 로그인된 사용자"에게 저장되므로,
//         QT가 아직 없는 설치 현장에서 관리자 창이 먼저 로그인해두면 캘리 결과가
//         계정에 영속 저장된다. 로그인 없이 캘리하면 세션에만 남고 재시작 시 소실.
//       - 서버는 로그인 사용자를 1명만 기억한다(currentUser_). ADMIN이 로그인한 뒤
//         QT가 다른 계정으로 로그인하면 저장 대상이 QT 쪽으로 바뀐다.
//
// [서버 -> 로봇]
//   PATH   payload: {"phase":"approach"|"draw", "segments":[
//              {"op":"TURN","angle_deg":-90},            // 제자리 회전 (+: 좌회전, -: 우회전)
//              {"op":"MOVE","dist_m":2.0,"paint":true,   // 직진 (paint: 도색 여부, 생략=false)
//               "heading_deg":35.0} ]}                   // 이 직진의 목표 절대각도 (정렬용, 로봇은 무시 가능)
//     - 로봇은 좌표를 모르므로 경로는 동작 명령 시퀀스로 전달.
//     - PATH가 오면 기존 경로 즉시 폐기하고 새 경로로 교체 (TCP가 순서 보장).
//     - phase="approach": 도면 시작점까지 이동(전부 paint=false) + 첫 도색 방향으로
//       회전까지. 끝나면 로봇은 PATH_DONE을 보내고 그 자리에서 대기한다.
//       마지막 TURN에는 heading_deg가 실려 READY 정렬 확인 가능.
//     - phase="draw": 접근 완료(PATH_DONE) 직후 서버가 자동으로 보내는 도색
//       경로(전부 paint=true). Qt가 버튼을 한 번 더 누르는 절차는 없다.
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
//       오차 <= 2도(또는 4회 반복 초과)면 GO{} 응답. GO를 받으면 직진 시작.
//       (임계값/횟수 실제값은 router.hpp의 kAlignThresholdDeg/kAlignMaxTries)
//   PATH_DONE payload: {"phase":"approach"|"draw"}
//     - 받은 PATH의 마지막 세그먼트까지 수행을 마쳤을 때 1회 전송 (2026-07-27 추가).
//       phase는 방금 끝낸 PATH의 phase를 그대로 되돌려준다.
//     - phase="approach" -> 서버가 곧바로 도색 PATH를 이어 보낸다 (Qt 개입 없음).
//       phase="draw"     -> 서버가 경로 상태를 정리하고 QT에 DRAW_DONE을 통지한다.
//     - 서버는 자기 상태로 단계를 판단하므로 phase가 없거나 어긋나도 동작한다
//       (어긋나면 WARN 로그만 남김). 이탈 재계획으로 PATH가 교체된 경우에는
//       "마지막으로 받은 PATH"를 끝냈을 때 보내면 된다.
//
// [QT -> 서버]
//   REGISTER payload: {"id":"user1","pw":"...","cam_ip":"192.168.0.31"}
//     -> 응답 REGISTER_OK {"id":...} | REGISTER_FAIL {"reason":...}
//        (cam_ip는 선택 - 카메라 IP. 서버는 검증 없이 저장만 함)
//   LOGIN    payload: {"id":"user1","pw":"..."}
//     -> 응답 LOGIN_OK {"id":..., "calib":{...}|null, "cam_ip":"..."|null}
//        | LOGIN_FAIL {"reason":...}
//        (calib는 저장된 캘리브레이션 번들, null이면 캘리브레이션 필요.
//         cam_ip는 REGISTER 때 등록한 카메라 IP, 없으면 null)
//   SET_CAM_IP payload: {"cam_ip":"192.168.0.31"}   (2026-07-27 추가)
//     -> 응답 SET_CAM_IP_OK {"cam_ip":"..."|null} | SET_CAM_IP_FAIL {"reason":...}
//        (로그인 상태에서만 가능. REGISTER와 마찬가지로 형식 검증 없이 저장만 하고,
//         빈 문자열을 보내면 등록을 지운다(null). Qt 설정란에서 카메라 교체용)
//   CMD      payload: {"cmd":...}  -> ROBOT 중계
//     이벤트 ESTOP/RESUME + 수동 조작 FORWARD/BACKWARD/TURN_LEFT/
//     TURN_RIGHT/STOP (조이스틱: 누르는 동안 이동, 이동량 없음)
//     + START_DRAW ("그림그리기 시작" 버튼): 서버가 1단계(접근) 경로부터
//       생성·전송하고, 이후 접근 완료 -> 도색 -> 완료까지 자동 진행한다.
//       로봇 중계는 안 함.
//     ※ CALIB_START는 QT가 안 보냄(2026-07-23) - 캘리 시작은 관리자 창(ADMIN)
//       담당. 서버는 하위호환으로 QT의 CALIB_START도 여전히 CCTV까지 중계함.
//   BLUEPRINT payload: {"points":[[x,y],...],          // 필수
//                       "paint":[bool,...],            // 선택 (2026-07-28)
//                       "pen_offset_m":0.04,           // 선택 (2026-07-28)
//                       "program":[{op...},...]}       // 선택 (2026-07-28)
//     - points: 바닥 평면 미터 좌표 폴리라인 = "펜이 지나갈 자취".
//       Qt가 top-view 픽셀 -> 미터 변환(÷ S px/m)을 마친 값. 서버는 재변환하지 않음.
//     - 서버는 저장만 한다. 로봇은 START_DRAW 전까지 움직이지 않는다.
//     - 응답 BLUEPRINT_OK로 서버가 받은 개수를 회신한다(아래 [서버 -> QT]).
//     - paint[i] = "points[i-1] -> points[i] 구간을 칠하는가". points와 길이가
//       같아야 하며 paint[0]은 대응 구간이 없어 무시된다. 도형 여러 개를 한
//       폴리라인으로 이어 보낼 때 도형 사이 이동을 칠하지 않기 위한 것
//       (한붓그리기 -> 여러 획). 길이가 어긋나면 도면은 살리고 이 필드만 무시
//       (= 전 구간 도색).
//     - pen_offset_m: 펜(노즐)이 로봇 회전중심 "뒤"로 떨어진 거리 d.
//       기구값이고 원천은 Qt. 서버는 이탈 복귀 경로의 펜 정렬에만 쓴다.
//     - program: Qt가 만든 도색 동작 시퀀스. 있으면 서버는 도색 경로를 생성하지
//       않고 이것을 그대로 로봇에 넘긴다 (2026-07-28 구조 변경).
//       ⚠️ 서버가 다시 만들면 안 된다 - 꼭짓점 동작(아래)이 사라져 펜 자취가
//       어긋나고 Qt 화면의 미리보기와도 달라진다.
//       없으면 종전대로 서버가 points로 직접 생성한다(하위호환).
//
//   [program op 규약 - Qt가 만들고 서버는 그대로 중계]
//     MOVE   {"dist_m":±m, "paint":bool, "heading_deg":deg,
//             "speed_mps":m/s, "v":꼭짓점idx, "pivot":bool}
//       - dist_m 음수 = 후진. 바라보는 방향은 바뀌지 않는다.
//     TURN   {"angle_deg":deg, "heading_deg":deg, "speed_dps":deg/s,
//             "v":꼭짓점idx, "pivot":bool}     (양수 = 좌회전, 종전과 동일)
//     NOZZLE {"down":bool, "v":꼭짓점idx, "pivot":bool}
//     공통 v     : 이 op가 담당하는 도면 꼭짓점 index (이탈 복귀 재개 지점 계산용)
//     공통 pivot : "꼭짓점 방향전환 동작"인가 (아래)
//     heading_deg는 "로봇이 바라보는 절대 방위"다. 전진 중에는 진행 방향과
//     같지만 후진 op에서는 180도 다르다 - pose의 theta(마커 앞변 기준)와
//     직접 비교되므로 바라보는 방향이어야 맞다.
//
//   [꼭짓점 동작 (pivot) - 펜 오프셋 보정]
//     펜이 회전중심에서 d만큼 뒤에 있어, 제자리 회전하면 펜이 반지름 d짜리 호를
//     그려 바닥에 자국이 남는다(5cm·90도 = 7.9cm). 그래서 꼭짓점마다:
//       노즐 올림 -> 후진 d -> 제자리 회전 -> 전진 d -> 노즐 내림
//     후진량과 재전진량이 "정확히 d"여야 회전축이 꼭짓점이 되고 펜이 제자리로
//     돌아온다. 이 op들에 pivot:true가 붙는다.
//     🔴 서버는 pivot 구간에서 ALIGN/DRIFT를 보내지 않는다 - 각도를 보정하면
//        후진 방향이 틀어져 "후진량 = 재전진량"이 깨지기 때문. READY가 오면
//        판정 없이 곧바로 GO를 준다 (router.cpp fromRobot 참고).
//     시작 lead-in: 서버는 로봇 "중심"을 points[0]에 세우기만 한다. 그 순간 펜은
//        d만큼 뒤에 있으므로 program 맨 앞에 MOVE +d(paint=false)가 들어간다.
//        ⚠️ 서버가 접근 목표를 points[0] + d로 밀면 이중 적용된다.
//
// [서버 -> QT]
//   STATUS : 그대로 중계 (모니터링용)
//   POSE   payload: {"x":1.234,"y":0.567,"theta_deg":90.0}
//     - 서버가 POS를 변환해 계산한 로봇 pose (바닥 미터 좌표) - top-view 표시용
//     - CCTV의 POS 원본(픽셀)은 QT에 중계하지 않는다 (2026-07-27 변경).
//       Qt에는 캘리브레이션이 없어 픽셀을 해석할 방법이 없기 때문.
//   H_MATRIX : 캘리브레이션 갱신 직후 그대로 중계 (top-view 재생성용)
//   PEERS  payload: {"robot":true,"cctv":false}
//     - ROBOT/CCTV 접속·해제될 때마다 전송. QT 접속 직후에도 현재 스냅샷 1회 전송.
//     - QT가 "로봇/CCTV가 지금 붙어있는지"를 STATUS/POS 유무로 유추하지 않고
//       바로 알 수 있게 하는 접속 상태 신호 (2026-07-22 추가)
//   BLUEPRINT_OK payload: {"points":n,"paint":bool,"program":n,"pen_offset_m":d}
//     - BLUEPRINT를 받아 저장한 직후 회신 (2026-07-28 추가). Qt가 "보낸 것과
//       서버가 받은 것이 같은지"를 그 자리에서 대조할 수 있게 한다.
//       (예전엔 응답이 없어, 실패를 START_DRAW 때 DRAW_FAIL로 뒤늦게 알았다)
//     - paint/program이 형식 오류로 무시됐으면 여기에 false/0으로 나타난다.
//   DRAW_DONE payload: {}   (2026-07-27 추가)
//     - 도색 경로를 끝까지 마쳤을 때 1회 통지 (로봇 PATH_DONE(draw)이 트리거).
//       접근 완료는 Qt에 알리지 않는다 - Qt 입장에선 START_DRAW부터 여기까지가
//       한 덩어리의 "그리는 중"이다.
//   DRAW_FAIL payload: {"stage":"plan"|"draw", "reason":"<코드>", "msg":"<설명>"}
//     - 경로 생성/전송이 실패했거나 아직 불가능한 상태(로봇 위치 미확인 - 대기
//       성격)일 때 통지 (2026-07-23 추가)
//     - stage=plan: BLUEPRINT 처리 중 (reason: bad_points)
//     - stage=draw: START_DRAW 이후 처리 중
//       (reason: no_blueprint/busy/no_pose/robot_offline/not_ready)
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
//     -> POSE를 QT 전송 (POS 원본은 중계하지 않음) + 계획 경로에서 0.3m 초과
//        이탈 시 재계획 PATH 전송 (최소 3초 간격)
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
