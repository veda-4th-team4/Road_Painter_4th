#pragma once
// ===== Road-Painter 메시지 프로토콜 (v0.4) =====
// 공통 형식: {"type": "...", "seq": n, "payload": {...}} + 개행(\n)  [TLS 위 JSON Lines]
//
// 좌표계 규약 (v0.3 핵심):
//   - CCTV는 "원본 픽셀 좌표"만 보낸다. 좌표 변환은 전부 서버가 한다.
//     (undistort -> H_marker. 캘리브레이션 데이터는 서버 한 곳에만 존재)
//   - QT는 "바닥 미터 좌표"로 변환을 마친 도면을 보낸다.
//     (top-view 위에 그린 점 = 바닥 평면 위의 점이라 스케일 나눗셈이 전부)
//   - 서버 내부 월드 좌표계: 바닥 평면, 단위 미터.
//
// 채널 규약 (v0.4 핵심 - 다채널 카메라 PNM-C16083RVQ):
//   - "ch" = 채널 번호, 1부터 시작 (카메라 웹UI의 CH1~CH4와 같은 번호).
//   - 채널마다 렌즈 방향이 달라 캘리브레이션 번들(K/D/H)이 완전히 다르다.
//     그래서 서버는 캘리브레이션을 채널별 맵으로 들고 있다.
//   - 붙는 곳: H_MATRIX.payload.ch / POS.payload.ch (CCTV -> 서버),
//     CMD{"cmd":"SELECT_CHANNEL","ch":n} (QT -> 서버 -> CCTV),
//     CHANNEL_OK.payload.ch / LOGIN_OK.payload.calibs (서버 -> QT).
//   - 🔴 ch는 전부 선택 필드다. 없으면 서버가 1로 본다 - 단일 채널 카메라(PNO)와
//     기존 클라이언트는 한 줄도 안 고쳐도 v0.3과 100% 동일하게 동작한다.
//   - 서버는 "활성 채널" 1개를 기억한다(기본 1, SELECT_CHANNEL로 변경).
//     활성 채널과 다른 ch의 POS는 무시한다 - 다른 채널이 우연히 로봇 마커를
//     잡았을 때 pose가 두 채널 사이에서 튀는 것을 막기 위함.
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
//              {"op":"MOVE","dist_m":2.0,"paint":true,   // 직진 (paint: 도색 구간 표시)
//               "heading_deg":35.0},                     // 이 직진의 목표 절대각도 (정렬용, 로봇은 무시 가능)
//              {"op":"NOZZLE","down":true},               // 노즐 내림/올림 (노즐 제어는 이 op만)
//              {"op":"ARC","dist_m":0.628,"radius_m":0.20, // 곡선(원호) 주행 (2026-07-29 추가)
//               "angle_deg":180,"direction":"right"} ]}    // radius_m: 도면 곡선 반지름, direction: 회전 방향
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
//     - 이벤트: "ESTOP" | "RESUME" | "CALIB_START" | "ABORT_DRAW"
//     - 수동 조작(조이스틱, 누르는 동안 이동 / STOP=뗌, 이동량 없음):
//         "FORWARD" | "BACKWARD" | "TURN_LEFT" | "TURN_RIGHT" | "STOP"
//       수동 CMD가 오면 서버는 자동 경로추종/재계획을 멈춘다(충돌 방지).
//       자동 모드 복귀는 새 BLUEPRINT 수신 시.
//
//   🔴 [ABORT_DRAW = 작업 완전 중지] (2026-08-03 신설, 로봇팀 신규 구현 대상)
//     ESTOP과 다른 점은 딱 하나 - "받아둔 PATH를 버린다".
//                        ESTOP            ABORT_DRAW
//       모터/노즐 정지     O                O
//       비상정지 래치      O (RESUME 필요)  O (RESUME 필요)
//       받아둔 경로        유지 -> RESUME   폐기 -> RESUME 해도
//                          하면 멈춘 데서   아무 데도 안 감
//                          이어서 달림
//     즉 ESTOP은 일시정지, ABORT_DRAW는 취소다. 로봇이 할 일:
//       1) 정지 + 노즐 올림 + 비상정지 래치 (ESTOP과 동일)
//       2) 경로 버퍼를 비운다 - 세그먼트 커서, PATH_DONE 전송 대기,
//          노즐 오프셋 서브시퀀스 상태까지 전부. RESUME 후에 자동 주행이
//          되살아나면 안 된다.
//       3) PATH_DONE을 보내지 않는다 (끝낸 게 아니라 버린 것).
//     다음 작업은 서버가 새 PATH를 보내는 것으로 시작한다.
//     ⚠️ ESTOP + ABORT_DRAW를 따로 보내지 않는다 - 두 개로 나누면 순서가
//     뒤바뀌거나 하나가 누락됐을 때 "섰는데 경로가 살아있는" 상태가 생긴다.
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
//     -> 응답 LOGIN_OK {"id":..., "calib":{...}|null, "calibs":{"1":{...},"2":null},
//                       "active_ch":1, "cam_ip":"..."|null}
//        | LOGIN_FAIL {"reason":...}
//        (calib  = 활성 채널의 번들. null이면 그 채널은 캘리브레이션 필요.
//                  v0.3과 의미가 같아 옛 클라이언트도 그대로 동작한다.
//         calibs = 채널별 번들 맵 (2026-08-03 신설). 키는 채널 번호 문자열.
//                  4채널 UI가 "어느 채널이 캘리 됐는지" 표시하는 데 쓴다.
//         active_ch = 서버가 기억하는 활성 채널 (기본 1)
//         cam_ip = REGISTER 때 등록한 카메라 IP, 없으면 null)
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
//     + ABORT_DRAW ("작업 중단" 버튼, 2026-08-03 신설): 진행 중인 작업을 취소.
//       서버가 경로 실행 상태(planActive_/awaitingArrival_/drawRequested_/
//       activeSegs_/planCursor_)를 전부 버리고 같은 명령을 ROBOT에 중계해
//       로봇도 받아둔 PATH를 버리게 한다(위 [ABORT_DRAW] 절).
//       -> 응답 DRAW_ABORTED {"was_active":bool}
//       - 도면(BLUEPRINT)은 지우지 않는다. 다시 START_DRAW를 누르면 같은
//         도면으로 처음부터 시작한다.
//       - 로봇은 비상정지 래치가 걸린 채 남으므로 RESUME이 필요하다. 다만
//         경로는 이미 버려졌으므로 RESUME으로 도색이 재개되지는 않는다.
//     + SELECT_CHANNEL {"cmd":"SELECT_CHANNEL","ch":n} (2026-08-03 신설):
//       작업 채널 전환. 서버가 활성 채널을 바꾸고 CCTV에 중계한다(로봇에는
//       안 보냄 - 로봇과 무관). -> 응답 CHANNEL_OK {"ch":n,"calib":{...}|null}
//       | CHANNEL_FAIL {"reason":"bad_channel"}
//       - calib이 null이면 그 채널은 아직 캘리브레이션이 안 된 것이다.
//       - ⚠️ 경로 실행 중에는 바꾸지 말 것. 서버는 막지 않지만 지금 로봇을
//         보고 있는 채널을 바꾸면 pose 공급이 끊긴다 (QT가 UI로 잠근다).
//     ※ CALIB_START는 QT가 안 보냄(2026-07-23) - 캘리 시작은 관리자 창(ADMIN)
//       담당. 서버는 하위호환으로 QT의 CALIB_START도 여전히 CCTV까지 중계함.
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
//     ⚠️ pen_offset_m 필드는 폐지됐다 (2026-07-28). 펜 오프셋 보정은 로봇이
//     TURN 실행 시 자기 하드웨어 상수(실측 155mm)로 전적으로 담당한다 - 서버/Qt
//     둘 다 관여하지 않는다. 서버는 이 값을 router.hpp의 kPenOffsetM 상수로만
//     갖고 있고, 오직 이탈 판정 여유값 계산에만 쓴다 (path_planner.hpp
//     distToActiveSegment 참고 - 도면은 펜 자취인데 pose는 마커 중심이라 정상
//     주행에도 구간 끝에서 상시 d가 벌어지는 것을 상쇄). 로봇의 실제 오프셋이
//     바뀌면 이 상수도 같이 고쳐야 한다 - 자동 동기화되지 않는다.
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
//       - dist_m = 호 길이(S = R·θ_rad), radius_m = 도면 상의 곡선 반지름(양수).
//       - 로봇은 155mm 후방 노즐 오프셋을 자기 피타고라스 공식
//         (R_robot = sqrt(radius_m² + 0.155²))으로 자율 보정한다 - TURN과
//         동일하게 Qt/서버는 도면 그대로의 값만 주고 관여하지 않는다.
//     공통 v     : 필수. 이 op가 "출발하는" 도면 꼭짓점 index.
//       서버 buildRecovery가 "꼭짓점 k로 복귀 후 재개할 op"을 v >= k 인 첫 op으로
//       찾으므로, 도착 꼭짓점이 아니라 출발 꼭짓점이어야 복귀 지점에서 떠나는
//       MOVE가 잡힌다. 없으면 서버가 program 전체를 거부한다(폴백으로 직접 생성).
//     heading_deg는 "로봇이 바라보는 절대 방위"다. 전진 중에는 진행 방향과
//     같지만 후진 op에서는 180도 다르다 - pose의 theta(마커 앞변 기준)와
//     직접 비교되므로 바라보는 방향이어야 맞다.
//
//   🔴 [노즐 제어의 단일 결정권 = NOZZLE op] (2026-07-28 확정)
//     로봇의 노즐 상태를 바꾸는 것은 오직 NOZZLE op이다. MOVE.paint는 참고용
//     표시이고, 그것만 보고 노즐을 내리거나 올리면 안 된다.
//     - 규약이 둘이면(NOZZLE op / MOVE.paint) 로봇 실행부가 "둘 중 뭘 믿나"를
//       분기해야 하고, 팀마다 다르게 구현될 여지가 생긴다. 하나로 못박는다.
//     - 경로의 시작은 항상 "노즐 올라간 상태"이고 끝도 NOZZLE up이다. 접근
//       (phase=approach) 경로에는 NOZZLE이 아예 없는데, 전부 paint=false라
//       내릴 일이 없고 직전 경로가 NOZZLE up으로 끝났기 때문이다.
//     - 서버가 program 없이 직접 만드는 경로(하위호환)에도 서버가 NOZZLE을
//       끼워 넣는다 (path_planner.hpp withNozzleOps). 예전에는 이 경로에
//       NOZZLE이 없어, NOZZLE만 보는 로봇은 노즐을 영영 못 내렸다.
//     - 🔴 NOZZLE은 반드시 down/up이 번갈아 나온다 (2026-07-29 확정). 같은
//       상태가 연속으로 두 번 오지 않는다 - down 다음은 무조건 up, up 다음은
//       무조건 down. 예: ㄷ자로 세로 획 두 개를 그릴 때, 첫 획 끝나고 NOZZLE up
//       한 번 -> (도색 없는 이동+회전) -> 다음 획 시작 직전 NOZZLE down 한 번.
//       이동 구간에서 "이미 올라가 있다"고 NOZZLE up을 또 보내지 않는다.
//       Qt program도, 서버가 만드는 폴백(withNozzleOps)도 이 규칙을 지킨다 -
//       withNozzleOps는 애초에 상태가 실제로 바뀔 때만 op을 끼워 넣으므로
//       자동으로 성립한다.
//
//     여기 없는 것: pivot 필드, 펜 보정 서브스텝(후진/전진), 속도(speed_mps/
//     speed_dps). 전부 Qt도 서버도 모르거나 관여하지 않는 값이라 program에
//     안 들어간다 - 로봇이 TURN을 실행할 때 자기 하드웨어 영역에서 알아서
//     처리한다(2026-07-28 최종 확정). 서버 buildRecovery도 이제 "v >= k인
//     첫 op"만 찾는 단순 검색이다 - pivot을 걸러낼 이유가 없어졌다.
//
//   [회전 중 DRIFT 억제 - 로봇 담당 (2026-07-28 확정)]
//     서버는 로봇이 지금 TURN을 실행 중인지 모른다. alignSegIdx_는 로봇이 READY를
//     보낼 때만 갱신되는데 로봇은 MOVE 앞에서만 READY를 보내므로, TURN을 도는
//     동안에도 서버는 직전 MOVE 목표각을 기준으로 DRIFT를 계속 쏜다(실측 최대
//     -40도). 그대로 두면 로봇이 자기 회전과 서버 보정이 싸운다.
//     🔴 서버가 "지금 TURN 중"임을 알려주는 신호를 새로 만들지 않기로 했다 -
//     이탈 감시(POS 기반 진행 커서)는 alignSegIdx_와 무관하게 로봇 실제 위치만
//     보고 판단하므로 서버가 회전 여부를 몰라도 다른 기능에 영향이 없다.
//     로봇이 자기 상태(IsTurning() 등)로 TURN 실행 중엔 들어오는 DRIFT를 무시.
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
//   CHANNEL_OK   payload: {"ch":2,"calib":{...}|null}     (2026-08-03 추가)
//   CHANNEL_FAIL payload: {"reason":"bad_channel"}
//     - CMD SELECT_CHANNEL에 대한 응답. calib은 그 채널의 번들이며 null이면
//       그 채널은 아직 캘리브레이션이 안 된 것이다.
//     - QT는 이 calib으로 top-view를 다시 만든다 - H_MATRIX를 받았을 때와
//       같은 경로를 타야 한다(K/D 반영, coord_mode 판정까지).
//   DRAW_ABORTED payload: {"was_active":true}              (2026-08-03 추가)
//     - CMD ABORT_DRAW를 받아 서버가 경로 상태를 버린 직후 1회 전송.
//     - was_active=false면 진행 중인 작업이 없었다는 뜻(버튼 연타 등).
//     - ⚠️ QT는 이 응답을 기다리지 말고 UI를 먼저 정리해도 된다. 취소는
//       사용자가 급할 때 누르는 버튼이라 서버 왕복을 기다리면 안 된다.
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
// [서버 -> CCTV]
//   CMD payload: {"cmd":"CALIB_START"}          캘리브레이션 시작
//   CMD payload: {"cmd":"SELECT_CHANNEL","ch":2}  작업 채널 전환 (2026-08-03 신설)
//     - 다채널 카메라에서 "지금 로봇을 봐야 할 채널"이 바뀌었음을 알린다. 카메라
//       앱은 그 채널 영상으로 검출 대상을 바꾸고 이후 POS에 그 ch를 싣는다.
//     - 단일 채널 카메라(PNO)는 무시해도 된다. 서버는 응답을 기대하지 않는다.
//     - ⚠️ 현재 카메라 앱(CCTV/src/central_tls_sender.cpp)의 수신 파서는
//       "cmd":"CALIB_START" 문자열 하나만 인식한다 - 파서를 고쳐야 받을 수 있다.
//
// [CCTV -> 서버]
//   H_MATRIX payload: {"ch":2, "calib":{"version":1, "K":[[...]x3], "D":[k1,k2,p1,p2,k3],
//                      "H_floor":[[...]x3], "H_marker":[[...]x3], "marker_height_m":0.25}}
//     - 캘리브레이션 1회 수행 후 전송. 로그인 사용자에 영속 저장 + QT 중계.
//     - ch(2026-08-03 추가): 이 번들이 어느 채널의 것인지. payload 최상위에
//       붙는다 - calib "안"이 아니다. 생략하면 채널 1.
//       ⚠️ 안 실으면 4채널이 전부 채널 1에 덮어써져 같은 번들 하나를 공유한다.
//       평면 스키마는 payload 자체가 번들이라 ch가 번들 안에 남는데, 서버는
//       그것을 calib_id처럼 손대지 않는 메타데이터로 보존한다.
//     - H_floor  : 왜곡 보정된 픽셀 -> 바닥 평면 미터 (Qt top-view 용)
//     - H_marker : 왜곡 보정된 픽셀 -> 마커 장착 높이 평면 미터 (로봇 측위용,
//                  마커가 바닥에서 떠 있어 생기는 시차를 흡수)
//     - 평면 스키마(QT-REQ-CCTV-001 rev.2)도 허용: payload 자체가 번들이고
//       바닥 H의 이름이 "H"다 -> {"calib_id","created_at","image_size",
//       "coord_mode","unit","K","D","H","H_marker","origin_mm","canvas_mm","axis"}.
//       H를 H_floor로 읽고 K/D/H_marker까지 그대로 쓴다(보정 동작은 중첩과 동일).
//       설치 메타데이터는 손대지 않고 저장·중계하되, "unit"만 정규화 후 "m"으로 고친다.
//     - 레거시 {"H":[[...]x3]}만 온 경우도 허용 (왜곡/시차 보정 없이 동작)
//   POS      payload: {"ch":2, "corners":[[u,v]x4]}  로봇 마커 4점 = "원본 CCTV 픽셀"
//     순서 = [전좌, 전우, 후우, 후좌]
//     - ch(2026-08-03 추가, 선택 - 생략하면 1): 이 마커를 본 채널. 서버는 이
//       채널의 캘리브레이션(K/D/H_marker)으로 좌표를 변환한다.
//       ⚠️ 틀린 채널을 실으면 좌표가 통째로 어긋난다 - 에러 없이 그럴듯한 값이
//       나오므로 특히 주의. 활성 채널과 다른 ch가 오면 그 POS는 무시된다.
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
