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
// ============================================================================
// [서버 <-> 로봇] 🔴 프로토콜 v2 (server-driven) - 규격 전문은
//   docs/PROTOCOL_v2_ROBOT.md. 아래는 요약이고, 어긋나면 그 문서가 정본이다.
//
//   v1과 달라진 핵심 4가지:
//     1. 각도 부호     : 로봇에 나가는 모든 각도가 "양수 = 오른쪽으로 틀어라"
//                        (서버 내부는 여전히 CCW 양수. 송신 직전에만 뒤집는다)
//     2. 펜 오프셋 보정: 서버가 ±0.155m move op을 경로에 끼워 넣는다.
//                        로봇은 자체 보정을 하지 않는다 (하면 이중 보정)
//     3. 핸드셰이크    : 모든 op마다 READY -> GO. MOVE 앞에서만이 아니다
//     4. 피드백        : ALIGN / MORE(신설) / DRIFT
// ============================================================================
//
// [서버 -> 로봇]
//   PATH   payload: {"phase":"approach"|"draw", "ops":[
//              {"op":"turn","role":"path","angle_deg":-90.0,"op_index":0},
//                                                    // 제자리 회전 (+: 우회전)
//              {"op":"move","role":"offset","dist_m":0.155,"op_index":1},
//                                                    // 직진 (음수 = 후진)
//              {"op":"nozzle","role":"offset","down":true,"op_index":2},
//              {"op":"arc","role":"path","radius_m":0.476,"angle_deg":180.0,
//               "direction":"right","radius_draw_m":0.5,"op_index":3} ]}
//     - 로봇은 좌표를 모르므로 경로는 동작 명령 시퀀스로 전달.
//     - PATH가 오면 기존 경로 즉시 폐기하고 새 경로로 교체 (TCP가 순서 보장).
//     - op_index는 경로 전체에 0부터 빈틈없이. 새 PATH를 받으면 다시 0부터.
//     - 🔴 role은 관측용 메타데이터다. 로봇은 읽지 않으며 값에 따라 동작을
//       바꾸지 않는다. 로봇 실행부에 role 분기가 생기면 그 자체가 버그다.
//     - move에는 paint 필드가 없다. 노즐 상태를 바꾸는 것은 nozzle op뿐이다.
//     - arc.radius_m은 "로봇 마커 중심"이 그려야 할 반지름(서버가 이미 펜
//       오프셋을 반영한 실행값)이다. 로봇은 여기에 자기 보정을 더하지 않는다.
//       정지 조건도 펜 기준 호 길이가 아니라 radius_m x θ_rad다.
//       radius_draw_m은 참고용(도면상 펜 자취 반지름) - 로봇은 무시한다.
//     - phase="approach": 도면 시작점까지 이동 + 첫 도색 방향 회전. 오프셋
//       보정 op만 없고 ALIGN/MORE/DRIFT는 전부 적용된다.
//     - phase="draw": 접근 완료(PATH_DONE) 직후 서버가 자동으로 이어 보낸다.
//   GO     payload: {"op_index":n}         // READY 응답: op n을 실행하라
//   ALIGN  payload: {"op_index":n, "angle_deg":±d}
//     - 제자리 미세 회전 후 "같은 op_index로 READY를 다시" 보낸다.
//   MORE   payload: {"op_index":n, "dist_m":±m}
//     - 현재 방향으로 전/후진(음수=후진) 후 "같은 op_index로 READY를 다시".
//   DRIFT  payload: {"op_index":n, "angle_deg":±d}
//     - 직진 주행 "중" 각도 보정. READY로 응답하지 않는다 (fire-and-forget).
//     - 로봇 거동은 연속 조향(멈추지 않고 좌우 바퀴 속도차로 흡수)으로 정했다.
//   HOLD   payload: {"hold":true|false, "reason":"pos_lost"}
//     - true: 실행 중인 op 도중이라도 즉시 정지 (op을 포기하지는 않는다)
//       false: 멈춘 지점에서 같은 op을 남은 거리/각도부터 이어서 수행
//     - HOLD 중에는 서버가 GO/ALIGN/MORE/DRIFT를 일절 보내지 않는다.
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
//   READY  payload: {"op_index": 3}
//     - 🔴 op_index = "이제부터 실행하려는" op의 index다 (완료한 op이 아니다).
//     - 모든 op 앞에서 정지 상태로 전송한다 (v1처럼 MOVE 앞에서만이 아니다).
//     - 서버 응답은 GO / ALIGN / MORE 중 정확히 하나다. ALIGN/MORE를 받으면
//       그 동작을 수행하고 "같은 op_index로 READY를 다시" 보낸다. GO를 받아야만
//       다음 op으로 넘어간다.
//     - 서버 응답은 최악 (feedback_wait_ms + STATUS 주기)만큼 늦다. 로봇은
//       READY에 자체 타임아웃을 걸어 임의로 출발하면 안 된다.
//     - 자기가 기다리는 index와 다른 GO/ALIGN/MORE/DRIFT는 조용히 버릴 것
//       (지연 도착한 이전 경로의 응답이 새 경로를 움직이는 것을 막는다).
//   PATH_DONE payload: {"phase":"approach"|"draw"}
//     - 받은 PATH의 마지막 op까지 수행을 마쳤을 때 1회 전송. 마지막 op을 마친
//       뒤에는 READY{N}을 보내지 않는다 (PATH_DONE이 그 자리를 대신한다).
//       phase는 방금 끝낸 PATH의 phase를 그대로 되돌려준다.
//     - phase="approach" -> 서버가 곧바로 도색 PATH를 이어 보낸다 (Qt 개입 없음).
//       phase="draw"     -> 서버가 경로 상태를 정리하고 QT에 DRAW_DONE을 통지한다.
//     - 서버는 자기 상태로 단계를 판단하므로 phase가 없거나 어긋나도 동작한다
//       (어긋나면 WARN 로그만 남김).
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
//     ※ 2026-07-23 "CALIB_START는 QT가 안 보냄, 캘리는 관리자 창(ADMIN) 담당"은
//       2026-08-10 Qt팀 계약서로 **뒤집혔다** - 이제 QT도 개시자다. 개시자가
//       둘이 되었으므로 두 경로 모두 같은 세션 상태(calibActive_)를 공유한다:
//       한쪽이 도는 동안 다른 쪽 요청은 busy로 거절된다. 아래 [로봇 주행
//       호모그래피 세션] 절이 정본이다.
//   BLUEPRINT payload: {"points":[[x,y],...],          // 필수
//                       "paint":[bool,...],            // 선택 (2026-07-28)
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
//     - program: Qt가 만든 도색 동작 시퀀스(도면 그대로의 논리 동작 - MOVE/TURN/
//       NOZZLE). 있으면 서버는 도색 경로를 생성하지 않고 이것을 그대로 로봇에
//       넘긴다 (2026-07-28 구조 변경). 없으면 종전대로 서버가 points로 직접
//       생성한다(하위호환).
//
//     ⚠️ pen_offset_m 필드는 폐지됐다 (2026-07-28). 값 자체는 서버가
//     params().pen_offset_m으로 들고 있다 (config/params.json에서 조정).
//     🔴 v2에서 보정 주체가 바뀌었다: 로봇이 아니라 **서버**가 program을 로봇
//     op으로 변환하면서 ±0.155m move op을 끼워 넣는다 (ops_builder.hpp
//     buildDrawOps). Qt는 종전대로 도면 그대로의 논리 동작만 보내면 되고,
//     로봇은 자체 보정을 하지 않는다 - 하면 이중 보정이 된다.
//
//   [program op 규약 - Qt 입력값 그대로, 서버는 손대지 않고 중계]
//     MOVE   {"dist_m":±m, "paint":bool, "heading_deg":deg, "v":꼭짓점idx}
//       - dist_m 음수 = 후진. 바라보는 방향은 바뀌지 않는다.
//       - paint: "이 구간이 도색 구간인가"를 나타내는 표시일 뿐이다. 이 값으로
//         노즐을 움직이지 말 것 (아래 NOZZLE 단일 결정권 참고).
//     TURN   {"angle_deg":deg, "heading_deg":deg, "v":꼭짓점idx}
//       (양수 = 좌회전, 종전과 동일)
//     NOZZLE {"down":bool, "v":꼭짓점idx}
//     ARC    {"dist_m":m, "radius_m":m, "angle_deg":deg, "direction":"left"|"right",
//             "paint":bool, "heading_deg":deg, "v":꼭짓점idx}   (2026-07-29 신설)
//       - 곡선(원호) 주행. 알파벳 'D'/'O', 도로 곡선 표지 도색용.
//       - radius_m = 도면 상의 곡선 반지름(= 펜 자취 반지름, 양수).
//         dist_m(호 길이)은 서버가 쓰지 않는다 - v2 로봇 arc의 정지 조건은
//         "바퀴중심 기준" radius_m x θ_rad라 펜 기준 호 길이와 다르기 때문.
//       - 🔴 서버가 R_robot = sqrt(radius_m² - 0.155²)로 바꿔서 로봇에 보낸다
//         (docs/PROTOCOL_v2_ROBOT.md §5.4). Qt는 도면 그대로의 값만 주면 된다.
//       - paint=true인 ARC의 radius_m이 params().min_paint_radius_m보다 작으면
//         서버가 도면을 거부한다 (DRAW_FAIL reason="arc_too_tight"). 노즐이
//         그릴 수 있는 최소 원의 반지름이 곧 펜 오프셋 0.155m이기 때문.
//       - 🔴 도색 ARC는 차체가 접선을 향하면 안 된다. 서버가 진입 앞뒤에
//         turn(±φ) 오프셋 op을 끼워 넣는다 (φ = atan(0.155 / R_robot),
//         docs/PROTOCOL_v2_ROBOT.md §5.7). Qt는 도면 그대로의 값만 주면 된다.
//     공통 v     : 필수. 이 op가 "출발하는" 도면 꼭짓점 index.
//       v2에서는 MORE(주행 거리 보정)의 목표 좌표를 여기서 얻는다: 방금 끝낸
//       주행 op의 "도착" 꼭짓점 = 그다음 op의 v. 추측항법으로 누적하지 않고
//       Qt가 준 좌표만 쓰므로 오차가 쌓이지 않는다. 없으면 서버가 도면을
//       거부한다 (DRAW_FAIL reason="bad_program").
//     heading_deg는 "로봇이 바라보는 절대 방위"다. 전진 중에는 진행 방향과
//     같지만 후진 op에서는 180도 다르다 - pose의 theta(마커 앞변 기준)와
//     직접 비교되므로 바라보는 방향이어야 맞다.
//
//   🔴 [Qt NOZZLE op은 서버가 버린다] (v2 변경)
//     노즐 타이밍은 펜 오프셋 보정 op과 한 몸이라(노즐 down = 중심이 꼭짓점보다
//     a 앞) 서버가 전부 다시 만든다. Qt는 종전 형식대로 NOZZLE을 넣어 보내도
//     되고 빼도 된다 - 서버가 무시한다. 로봇에 나가는 nozzle op은 여전히
//     down/up이 번갈아 나오며(그 규약은 유지), move에는 paint 필드가 없다.
//
//     여기 없는 것: pivot 필드, 펜 보정 서브스텝, 속도(speed_mps/speed_dps).
//     Qt도 서버도 모르거나 관여하지 않는 값이라 program에 안 들어간다.
//
//   [회전 중 DRIFT 억제 - v2에서는 서버가 안다]
//     v1은 로봇이 MOVE 앞에서만 READY를 보내 서버가 "지금 TURN 중"을 알 수 없었고,
//     그래서 회전 중에도 직전 MOVE 목표각 기준 DRIFT를 계속 쏘았다(실측 최대
//     -40도). v2는 모든 op마다 READY/GO가 오가므로 서버가 실행 중인 op을 정확히
//     안다 - DRIFT는 role="path"인 move를 실행 중일 때만 나간다. turn/arc/
//     오프셋 move 중에는 서버가 아예 보내지 않으므로 로봇 쪽 필터가 필요 없다.
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
//   BLUEPRINT_OK payload: {"points":n,"paint":bool,"program":n}
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
// ============================================================================
// [로봇 주행 호모그래피 세션] (2026-08-10 신설, router_calib.cpp)
//   규격 원본: QT_HOMOGRAPHY_SERVER_CONTRACT_2026-08-10.md (Qt팀 제안),
//   서버 회신: docs/QT_HOMOGRAPHY_REPLY_20260810_SERVER.md
//
//   Qt는 "CH n 캘리 시작"만 보내고, 로봇 이동·샘플링·계산에는 관여하지 않는다.
//   서버는 요청을 검증해 수락 여부를 즉시 회신하고, 끝날 때까지 반드시
//   **종결 응답 하나**(H_MATRIX | CALIB_FAIL | CALIB_CANCELLED)를 돌려준다.
//   🔴 종결 응답을 빠뜨리면 Qt는 5분 타임아웃까지 대기 화면에 갇힌다.
//
//   [QT -> 서버]
//     CMD payload: {"cmd":"CALIB_START","ch":2,"request_id":"qt-...","method":"robot_motion"}
//       - ch/request_id 필수. 서버가 이 요청 하나를 처리하면서 활성 채널까지
//         바꾸므로, Qt는 앞에 SELECT_CHANNEL을 따로 보낼 필요가 없다
//         (onMessage 전체가 mtx_로 직렬화되어 채널 전환과 세션 개시가 원자적).
//       - 같은 request_id 재수신은 새 작업을 만들지 않고 CALIB_STARTED를 재전송한다(멱등).
//     CMD payload: {"cmd":"CALIB_CANCEL","ch":2,"request_id":"qt-..."}
//       - 서버는 ROBOT/CCTV 양쪽에 중계하고 **둘 다** CALIB_STOPPED로 답해야
//         CALIB_CANCELLED를 보낸다. 못 받으면 CALIB_FAIL{cancel_failed}.
//
//   [서버 -> QT]
//     CALIB_STARTED   payload: {"ch","request_id","msg"}      수락 즉시
//     CALIB_PROGRESS  payload: {"ch","request_id","progress","stage","msg"}
//       - 서버가 만들지 않는다. CCTV가 보내면 ch/request_id를 채워 중계할 뿐이다.
//         진행률을 못 주면 아예 안 보내면 된다 (Qt는 무한 진행 표시로 폴백).
//     H_MATRIX        payload: {"ch","request_id", ...번들}    성공 = 대기 해제
//       - 세션 중이면 서버가 request_id를 직접 찍어 넣는다 (CCTV 회신에 의존 안 함).
//     CALIB_FAIL      payload: {"ch","request_id","reason","msg"}
//       - reason: invalid_channel | busy | robot_offline | cctv_offline |
//                 motion_failed | insufficient_samples | solve_failed |
//                 cancel_failed | timeout | internal_error
//         (timeout은 서버가 추가한 코드 - 계약서 권장 목록에 없다. 회신 §4 참고)
//       - 서버가 자력으로 만들 수 있는 것은 사전 검증 실패와 timeout/오프라인뿐이다.
//         motion_failed/insufficient_samples/solve_failed는 ROBOT/CCTV가 CALIB_FAIL을
//         보내줘야 나온다 - 안 보내면 params().calib_timeout_ms 뒤 timeout이 된다.
//     CALIB_CANCELLED payload: {"ch","request_id","msg"}
//
//   [ROBOT/CCTV -> 서버]  🔴 아직 어느 쪽도 구현하지 않았다 (회신 §3)
//     CALIB_STOPPED payload: {}
//       - CALIB_CANCEL을 받아 안전 정지 + 작업 폐기를 마쳤다는 ACK.
//         이것이 없으면 취소는 항상 cancel_failed로 끝난다 - 서버가 로봇이
//         실제로 섰는지 확인할 방법이 없기 때문이다(추정으로 OK를 주지 않는다).
//     CALIB_FAIL payload: {"reason","msg"}  -> 그대로 QT에 종결 전달
//     CALIB_PROGRESS payload: {"progress","stage","msg"}  -> QT에 중계
// ============================================================================
//
// [CCTV -> 서버]
//   H_MATRIX payload: {"calib":{"version":1, "K":[[...]x3], "D":[k1,k2,p1,p2,k3],
//                      "H_floor":[[...]x3], "H_marker":[[...]x3], "marker_height_m":0.25}}
//     - 캘리브레이션 1회 수행 후 전송. 로그인 사용자에 영속 저장 + QT 중계.
//     - H_floor  : 왜곡 보정된 픽셀 -> 바닥 평면 미터 (Qt top-view 용)
//     - H_marker : 왜곡 보정된 픽셀 -> 마커 장착 높이 평면 미터 (로봇 측위용,
//                  마커가 바닥에서 떠 있어 생기는 시차를 흡수)
//     - 평면 스키마(QT-REQ-CCTV-001 rev.2)도 허용: payload 자체가 번들이고
//       바닥 H의 이름이 "H"다 -> {"calib_id","created_at","image_size",
//       "coord_mode","unit","K","D","H","H_marker","origin_mm","canvas_mm","axis"}.
//       H를 H_floor로 읽고 K/D/H_marker까지 그대로 쓴다(보정 동작은 중첩과 동일).
//       설치 메타데이터는 손대지 않고 저장·중계하되, "unit"만 정규화 후 "m"으로 고친다.
//     - 레거시 {"H":[[...]x3]}만 온 경우도 허용 (왜곡/시차 보정 없이 동작)
//   POS      payload: {"corners":[[u,v]x4]}  로봇 마커 4점 = "원본 CCTV 픽셀" 좌표
//     순서 = [전좌, 전우, 후우, 후좌]
//     - CCTV는 절대 좌표 변환하지 말 것 (undistort도 하지 말 것).
//       서버가 undistort(P=K 동등) -> H_marker -> pose 계산까지 담당.
//     - 테스트용으로 {"x","y","theta_deg"}(바닥 미터 좌표)도 허용
//     -> POSE를 QT 전송 (POS 원본은 중계하지 않음)
//     - 로봇에는 중계하지 않음: 로봇은 좌표를 모르며(PATH 참고), 위치/각도 보정은
//       서버가 ALIGN/MORE/DRIFT로만 내려준다
//     - ⚠️ "0.3m 이상 이탈 시 복귀 PATH 재전송"은 v2에서 폐지됐다. 보정 수단은
//       ALIGN/MORE/DRIFT 세 가지뿐이다.
//     - 마지막 채택 POS로부터 params().pos_lost_ms가 지나면 서버가 로봇에
//       HOLD{hold:true}를 보내 즉시 세운다 (복구되면 HOLD{hold:false}).
#include <nlohmann/json.hpp>
#include <atomic>
#include <chrono>
#include <string>

using json = nlohmann::json;

// 프로토콜 타이밍의 기준 시계. 판정 대기 창·POS 두절·캘리 타임아웃이 전부 이걸
// 쓴다. 🔴 반드시 단조 시계여야 한다 - 벽시계를 쓰면 NTP 보정 한 번에 대기 창이
// 음수가 되거나 타임아웃이 즉시 터진다.
inline long nowMs() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch())
        .count();
}

// type/payload 받아서 seq 자동 증가한 메시지 생성
inline json makeMsg(const std::string& type, const json& payload) {
    static std::atomic<long> seq{0};
    return json{{"type", type}, {"seq", ++seq}, {"payload", payload}};
}

// PATH 메시지 생성. phase: "approach"(시작점 접근) | "draw"(도색 경로)
// ⚠️ v2에서 배열 필드명이 segments -> ops로 바뀌었다.
inline json makePathMsg(const json& ops, const std::string& phase) {
    return makeMsg("PATH", {{"phase", phase}, {"ops", ops}});
}

// ===== 채널 (v0.4) =====================================================
// 채널 번호는 1부터 시작한다 (카메라 웹UI의 CH1~CH4와 같은 번호).
constexpr int kMinChannel = 1;
// PNM-C16083RVQ는 4채널이지만 상한을 딱 4로 박지 않는다 - 채널이 더 많은 모델로
// 바꿀 때 서버까지 고쳐야 하기 때문. 이 값은 오타/쓰레기값을 걸러내는 형식 검증용일
// 뿐이라 넉넉해도 손해가 없다 (없는 채널은 캘리브레이션이 없어 어차피 못 쓴다).
constexpr int kMaxChannel = 8;

inline bool validChannel(int ch) { return ch >= kMinChannel && ch <= kMaxChannel; }

// payload에서 채널 번호를 읽는다 (POS/H_MATRIX용, 관대한 해석).
// 🔴 없거나 형식이 어긋나면 1이다 - 단일 채널 카메라(PNO)와 v0.3 클라이언트가
// ch를 한 번도 안 실어도 그대로 동작해야 한다.
// ⚠️ SELECT_CHANNEL에는 쓰지 말 것. 거기서는 "잘못된 채널"을 CHANNEL_FAIL로
//    돌려줘야 하는데, 이 함수는 조용히 1로 바꿔버린다.
inline int channelOf(const json& payload) {
    if (!payload.is_object()) return kMinChannel;
    auto it = payload.find("ch");
    if (it == payload.end() || !it->is_number_integer()) return kMinChannel;
    int ch = it->get<int>();
    return validChannel(ch) ? ch : kMinChannel;
}

// 채널별 맵의 키. JSON 오브젝트 키는 문자열이어야 하므로 정수를 문자열로 쓴다.
inline std::string chKey(int ch) { return std::to_string(ch); }
