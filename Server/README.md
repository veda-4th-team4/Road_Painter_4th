# Road-Painter 중앙 서버

Qt(관제 UI) · 로봇(도색 로봇) · CCTV · 관리자 창 네 클라이언트를 중계하고, 도면으로부터 로봇 경로를 생성하는 중앙 서버입니다. 서버 RPi에서 실행합니다.

## 하는 일

- **TLS 릴레이**: 클라이언트가 role(QT / ROBOT / CCTV / ADMIN)로 등록하면, 메시지를 규칙에 따라 상대에게 중계
- **구역 진입 경보**: CCTV의 `ZONE_EVENT Enter`를 기존 TLS 세션으로 ROBOT에 중계. 별도 UDP 포트는 사용하지 않음
- **로그인 / 캘리브레이션 저장**: 사용자(id/비번)별로 캘리브레이션 번들(K, D, H행렬)을 저장했다가 재로그인 시 Qt에 돌려줌. **캘리브레이션은 채널별로 따로 저장된다** (v0.4 — 채널마다 렌즈 방향이 달라 K/D/H가 전부 다르다)
- **채널 전환 (v0.4)**: 4채널 카메라(PNM-C16083RVQ)에서 `SELECT_CHANNEL`로 작업 채널을 바꾸면 그 채널의 캘리브레이션이 적용되고, **활성 채널이 아닌 `POS`는 무시한다**(채널마다 좌표계가 달라 pose가 튀는 것을 막는다). ⚠️ `ch`는 전부 선택 필드라 **단일 채널 현장(PNO)은 한 줄도 안 고쳐도 그대로 동작**한다
- **작업 완전 중지 (`ABORT_DRAW`, v0.4)**: `ESTOP`이 일시정지라면 이건 취소다 — 서버가 들고 있는 경로 상태를 비우고 로봇에 중계해 **받아둔 경로까지 버리게** 한다. 로봇 쪽 구현은 `bce553f`에서 완료됐다. ⚠️ 서버의 `DRAW_ABORTED`는 **로봇에 중계한 직후의 접수 ACK**이지 물리적 정지 완료 ACK가 아니다
- **중계 스트림 주소 배포** (2026-08-04): `config/stream.json`이 있으면 `LOGIN_OK.stream`으로 중계(MediaMTX) RTSP 베이스 주소를 Qt에 내려준다 — 사용자가 설정 화면에 손으로 입력하지 않아도 된다. **파일이 없으면 필드를 안 보내고 Qt는 종전대로 자기 설정값을 쓴다**(= 중계를 안 쓰는 PNO 직결 현장은 그대로). ⚠️ `cam_ip`(카메라 IP, PNO 직결용)와는 **별개 필드**다 — 합치면 PNO로 되돌아갈 수 없다
> 🔴 **프로토콜 v2 (server-driven)로 갈아엎었다.** 서버↔로봇 구간의 정본은
> **[docs/PROTOCOL_v2_ROBOT.md](docs/PROTOCOL_v2_ROBOT.md)** 이다. Qt↔서버, CCTV↔서버 구간은 v1 그대로 바뀌지 않았다.
> 아래 목록의 서버↔로봇 항목은 전부 v2 기준이다.

- **경로 생성 (2단계)**: 1단계 접근(approach)은 **서버**가 CCTV로 파악한 로봇 위치에서 도면 시작점까지 만든다. 2단계 도색(draw)은 Qt가 보낸 동작 시퀀스(`BLUEPRINT.program`)를 **서버가 로봇 op으로 변환**한다 (v1처럼 그대로 중계하지 않는다). 변환에서 하는 일 네 가지: ① 각도 부호 반전(로봇 대면은 **양수 = 오른쪽**) ② **펜 오프셋(155mm) 보정 op 삽입** ③ **arc 반지름 치환**(`R_robot = sqrt(R_paint² − 0.155²)`) ④ **arc 진입 위상 보정**(`turn(±φ)`, `φ = atan(0.155/R_robot)`) — ④가 없으면 반지름은 맞는데 원이 통째로 어긋난 자리에 그려진다(실측 0.31m). `program`이 없으면 서버가 도면에서 같은 형식의 시퀀스를 만들어 똑같은 변환을 태운다
- **펜 오프셋 보정 주체가 로봇 → 서버로 바뀌었다**: 도색 진입 전에 `move(+0.155)`, 이탈 후에 `move(-0.155)`를 서버가 경로에 끼워 넣는다. 불변식은 "**노즐 down ⟺ 마커 중심이 꼭짓점보다 진행방향으로 정확히 a 앞**". 🔴 로봇은 자체 보정을 하면 안 된다 — 하면 이중 보정이다
- **모든 op마다 READY → GO 핸드셰이크**: v1은 MOVE 앞에서만이었다. `READY{op_index}`는 "이제부터 실행하려는" op을 가리키고, 서버는 **모든 READY에 GO / ALIGN / MORE 중 정확히 하나**로 답한다 (답을 빠뜨리면 로봇이 영원히 멈춘다)
- **피드백 3종 (ALIGN / MORE / DRIFT)**: `turn` 직후 boundary에서 각도를 **ALIGN**(최대 6회), `move`/`arc` 직후 boundary에서 거리를 **MORE**(최대 4회, 신설), `role=path`인 `move` 주행 중에만 **DRIFT**(2.5Hz). 한 boundary에 둘 다 걸리면 MORE를 먼저 소진하고 ALIGN을 돌린다. **`arc` 주행 중에는 DRIFT를 보내지 않는다**(바퀴 속도비가 고정이라 중간에 틀면 원이 깨진다) — 대신 진입 직전 ALIGN이 그만큼 중요하다
- **READY는 즉답하지 않는다**: 그 READY 수신 시점부터 **2초를 고정으로 기다린 뒤** 판정한다. 미세회전(0.1~0.3초)이 CCTV→pose 반영 지연보다 짧아 회전 전 각도로 같은 ALIGN을 재발사하던 진동을 막기 위함 — 로봇 입장에선 응답이 **최악 2.5초**까지 늦어질 뿐이라 **READY에 자체 타임아웃을 걸어 임의 출발하면 안 된다**. 대기 중 모인 `POS`는 버리지 않고 **theta 평균**(sin/cos 누적)과 위치 평균에 써서 노이즈를 1/√N로 줄인다 — 추가 지연 0. **2초 동안 새 `POS`가 한 장도 없으면 같은 보정을 반복하지 않고 `GO`로 빠진다**(pose가 직전과 동일해 재판정해도 같은 값이 나오는 것이 보장되므로)
- **도색 언더슛 (2026-08-24, `paint_undershoot_m`)**: 펜을 내리고 긋는 **직선**은 계산값을 그대로 보내지 않고 기본 2cm를 **일부러 덜** 명령한다. 남은 몫은 `MORE`가 CCTV 실측 기준으로 채운다 — 개루프 한 방의 오버슛보다 실측 보정이 정확하다는 전제다. 🔴 **목표 좌표는 줄이지 않는다**(명령만 줄여야 그 차이가 MORE에 오차로 보인다). 보정 시점엔 노즐이 아직 내려가 있어 남은 구간도 이어서 칠해진다. `arc`·오프셋 다리·비도색 이동에는 적용되지 않으며, `0`으로 두면 종전 동작 그대로다. ⚠️ `more_deadband_m`보다 커야 의미가 있다 — 이하면 MORE가 "보정 불필요"로 판정해 덜 그은 채 끝난다(서버가 기동 시 WARN)
- **POS 두절 시 HOLD** (v2 신설): 마지막 채택 `POS`로부터 2초가 지나면 `HOLD{true}`로 **실행 중인 op 도중이라도 즉시 정지**시킨다. 연속 2장이 다시 채택되면 `HOLD{false}` — 로봇은 멈춘 지점에서 같은 op을 남은 거리/각도부터 이어서 한다. HOLD 중에는 서버가 GO/ALIGN/MORE/DRIFT를 일절 보내지 않는다
- **🔴 튜닝 상수는 코드에 없다**: 임계값·주기·기하 상수 전부 [`config/params.json`](config/params.json)에 있고, 서버 시작 때 읽어 적용값을 로그로 남긴다. **값을 고치고 서버만 재시작하면 된다 — 재컴파일이 필요 없다.** 항목 설명은 [`src/params.hpp`](src/params.hpp), 근거는 프로토콜 문서 §10 상수표
- **POS 수신 요약 로그** (2026-08-04 신규): 10초마다 `POS 10초 요약 - 수신 N, 채택 N (N.NHz), 파싱실패 N, 이상치폐기 N`. 기존엔 pose 계산 실패가 대부분 조용히 `return`이라 **`POS`가 초당 몇 장 오는지 알 방법이 없었다**. ⚠️ **현장 실측 1.2~1.5Hz**(설계 전제 15~30Hz)이고 **도색 구간에선 0Hz까지 떨어진다** — CCTV 검출율 개선이 최우선 선행 과제
- **정렬 허용 오차 4°** (`align_threshold_deg`): 2°였을 때는 정렬 루프의 1회 오차가 임계값과 같은 크기라 수렴이 불가능했다. 로봇 메인 루프 80ms × 400 SPS = **틱당 1.42° 오버슛**(한쪽 방향 계통 편향) + 평활화 없는 `theta` 노이즈(마커 13cm 기준 코너 1px당 약 0.7°). 로봇이 감속 접근/예측 정지로 오버슛을 줄이면 다시 내릴 것
- **POS 이상치 게이트**: 각도 변화가 `3° + 40°/s × 경과시간`을 넘는 프레임은 폐기(연속 5회 초과 시 재동기). 로봇의 물리적 최대 회전속도가 17.8°/s로 알려져 있어, 그보다 빠른 변화는 코너 순서 회전(θ가 90°/180° 점프)이나 부분 가림 같은 **검출 오류**다. ⚠️ 허용치에 상한을 두지 말 것 — `POS`가 1~2Hz인 현장에서는 간격이 5~8초까지 벌어지고 그동안 로봇은 물리 상한으로도 100° 넘게 정당하게 돈다
- **노즐 제어**: 로봇에 나가는 `nozzle` op이 단일 결정권이고, `move`에는 `paint` 필드가 아예 없다. **Qt가 보낸 `NOZZLE` op은 서버가 전부 버리고 다시 만든다** — 노즐 타이밍이 펜 오프셋 보정 op과 한 몸이라 둘을 따로 정할 수 없기 때문. 규칙상 down/up 교대가 자동으로 보장된다
- **이탈 감시·재계획은 폐지됐다** (v2): "0.3 m 이상 벗어나면 복귀 PATH 재전송"은 없어졌다. 보정 수단은 ALIGN / MORE / DRIFT 세 가지뿐이다
- **하트비트**: 로봇이 10초간 무응답이면 연결 끊김으로 처리
- **관리자 창 지원(ADMIN)**: 서버가 중계하는 모든 메시지 사본을 TAP으로 관리자 창에 전달(로그 모니터), 관리자 창에서 온 로봇 명령/캘리 결과를 처리

## 문서

| 문서 | 내용 |
|---|---|
| **[docs/PROTOCOL_v2_ROBOT.md](docs/PROTOCOL_v2_ROBOT.md)** | 🔴 **서버↔로봇 v2 규격 (정본)**. 로봇팀이 봐야 할 문서 |
| **[server_PROTOCOL.md](server_PROTOCOL.md)** | 통신 규격 전체 (Qt/CCTV 팀용). ⚠️ 서버↔로봇 절은 v1 시절 내용이라 위 문서가 우선한다 |
| **[docs/SERVER_ARCHITECTURE.md](docs/SERVER_ARCHITECTURE.md)** | 서버 구조·불변식·함정. 코드를 처음 볼 때 여기부터 |
| [docs/TESTING.md](docs/TESTING.md) | 서버/Qt 테스트 가이드 |
| [admin_console/PLAN.md](admin_console/PLAN.md) | 관리자 창 설계/진행 상황 |
| [relay/README.md](relay/README.md) | RTSP 4채널 패스스루 중계 (카메라 → 서버 → Qt 영상 경로) |

### 진행 중인 팀 간 문서 (2026-08-10)

| 문서 | 내용 |
|---|---|
| [docs/ROBOT_ACTION_ITEMS_20260810.md](docs/ROBOT_ACTION_ITEMS_20260810.md) | 로봇 캘리 세션 R-1~R-4 (R-1~R-2·R-4 반영됨, R-3 호출부 미착수) |
| [docs/QT_HOMOGRAPHY_REPLY_20260810_SERVER.md](docs/QT_HOMOGRAPHY_REPLY_20260810_SERVER.md) | 호모그래피 세션 계약 §7 회신 |
| [docs/QT_NOZZLE_PATH_REPLY_20260810_SERVER.md](docs/QT_NOZZLE_PATH_REPLY_20260810_SERVER.md) | 경로·노즐 계약 §6 회신 |
| [docs/QT_TLS_VERIFY_20260810.md](docs/QT_TLS_VERIFY_20260810.md) | Qt TLS 검증 요청 (PR #42로 반영 완료, 기록용) |

### 종결된 문서 — 노션 아카이브

날짜가 박힌 1회성 서신·계획서 9건은 [서버 문서 아카이브](https://app.notion.com/p/3b914dc6aecf812183e3c64d07c1a8ac)로 옮겼습니다 (2026-08-10). 원본은 git 이력에 그대로 있습니다.

`REFACTOR_SUMMARY` · `DRIVE_TEST_PLAN` · `ROBOT_ACTION_ITEMS` 20260803/05/06/07 · `CCTV_ACTION_ITEMS_20260806` · `QT_ACTION_ITEMS_20260807` (+ 회신)

## 파일 구성

```
Server/
├── Makefile            빌드 스크립트
├── server_PROTOCOL.md  통신 프로토콜 문서 (로봇/QT/CCTV 팀용, 단일 창구)
├── start.sh            통합 실행 (관리자 창 자동 시작 + 서버 실행)
├── gen_cert.sh         TLS 자체서명 인증서 생성 (최초 1회)
├── certs/              server.crt(공개) / server.key(비밀, git 제외)
├── config/             params.json (튜닝 상수 - 편집 대상) + users.json (자동 생성, git 제외)
│   └── stream.json.example  중계 RTSP 베이스 주소 템플릿 (stream.json으로 복사, git 제외)
├── docs/               부속 문서 (v2 프로토콜 규격, 테스트 가이드, 주행 테스트 계획)
├── src/
│   ├── main.cpp            시작점 + 파라미터 로드 + 테스트용 콘솔 + graceful shutdown
│   ├── tls_server.hpp/cpp  TLS 네트워크 층 (접속, role 등록, 세션 스레드, ADMIN tap)
│   ├── router.hpp/cpp      메시지 라우팅 (중계 규칙 + 경로 전송 + READY/피드백 판정)
│   ├── router_channel.cpp  채널 전환(SELECT_CHANNEL) + 작업 취소(ABORT_DRAW)
│   ├── ops_builder.hpp     Qt program → 로봇 op 변환 (부호반전·펜보정 삽입·arc 반지름)
│   ├── params.hpp          🔴 튜닝 상수 정의 + config/params.json 로딩 (하드코딩 금지 지점)
│   ├── path_planner.hpp    마커 4점 → pose 추정 (좌표 계산만 남음)
│   ├── calib.hpp           캘리브레이션 번들 파싱 + undistort/호모그래피 수학 (채널별)
│   ├── stream_cfg.hpp      중계 RTSP 주소 설정 로딩 (LOGIN_OK.stream)
│   ├── user_store.hpp/cpp  사용자 저장소 (비번 해시 + 채널별 캘리브레이션 영속화)
│   ├── protocol.hpp        메시지 스펙 주석 + 생성 헬퍼 + 채널 헬퍼(channelOf 등)
│   └── log.hpp             타임스탬프 로그
├── tools/
│   ├── tls_client.hpp      테스트 도구 공용 TLS 클라이언트 뼈대
│   ├── robot_sim.cpp       ★ ROBOT+CCTV 대역 시뮬 (PATH 실행 → 결과를 POS로 되돌림, 펜 자취 계산)
│   ├── draw_test.cpp       ★ QT 대역 (사각형 도면 + 동작 시퀀스 생성 → START_DRAW)
│   ├── drive_test.cpp      ★ 로봇 주행 단독 (ADMIN으로 PATH 직접 중계 - 접근·CCTV·도면 없이 제자리 사각형)
│   ├── qt_sim.cpp          Qt 대역 테스트 클라이언트 (Qt 네트워킹 나오기 전 검증용)
│   ├── path_test.cpp       최초 1회 경로생성 테스트기 (CCTV 스냅샷 주입 → 접근 PATH 검증)
│   ├── seed_user.py        테스트 계정 생성 (기본 test/1234 + 예시 캘리브레이션)
│   └── *_snapshot.json     path_test용 CCTV 스냅샷 (호모그래피 + 마커 4코너)
├── relay/                  RTSP 4채널 패스스루 중계 (카메라 영상 → Qt, 재인코딩 없음)
│   ├── mediamtx.yml           중계 설정 (카메라 주소·계정 없음 — 커밋 안전)
│   ├── cameras.env.example    카메라 접속 정보 템플릿 (cameras.env로 복사, git 제외)
│   ├── probe_onvif.py         ★ ONVIF로 채널별 RTSP 주소 조회 (경로 짐작 금지 — 계정 잠김)
│   ├── install.sh             MediaMTX 바이너리 설치 (bin/은 git 제외)
│   ├── start.sh               중계 기동 (-d 로 백그라운드)
│   └── README.md              구성·문제 해결
└── admin_console/          관리자 창 (Python 웹 GUI - 카메라 캘리 도구 + 서버 로그/로봇 제어)
    ├── web_gui.py              진입점: HTTP 라우팅 + 로봇 제어( /robot )·로그 모니터( /logs ) + main()
    ├── cctv.py                 ★ CCTV 파트 (카메라 CAM_POSE·캘리브레이션·스냅샷 + 대시보드 UI) — CCTV팀 작업 파일
    ├── rp_core.py              공통 코어 (설정 + 로그 브로드캐스트/SSE + 중앙 서버 ADMIN 링크)
    ├── pose_server.py          터미널판 카메라 하니스 (web_gui의 원형)
    ├── start.sh                백그라운드 실행 스크립트 (포트는 config.sh에서)
    ├── config.sh.example       포트 설정 템플릿 (config.sh로 복사해 사용, config.sh는 git 제외)
    └── PLAN.md                 설계/진행 상황
```

관리자 창 모듈 의존 방향: `rp_core` ← `cctv` ← `web_gui` (단방향, 순환 없음). CCTV팀은 카메라·캘리브레이션 로직과 대시보드 UI가 모두 든 **`cctv.py`** 에서 작업하면 되고, 로그 출력(`broadcast`)·중앙 서버 전송(`server_send`)은 `rp_core`에서 가져다 씁니다.

구조: 접속한 클라이언트마다 전담 스레드가 생겨 자기 소켓을 읽고, 받은 메시지는 Router가 규칙에 따라 다른 클라이언트 소켓으로 배달합니다 (thread-per-connection).

## 빌드 & 실행

```bash
# 필요 패키지 (최초 1회)
sudo apt install g++ make libssl-dev nlohmann-json3-dev

# 인증서 생성 (최초 1회, 서버 IP 넣기)
./gen_cert.sh 192.168.0.8
# -> certs/server.crt 를 로봇/Qt/CCTV 클라이언트에 복사 (신뢰 CA로 사용)

# 빌드 & 실행 (관리자 창 웹 GUI도 자동으로 같이 뜸)
make
./start.sh
```

포트는 **9000** (TCP/TLS). 실행하면 포그라운드에서 돌며 로그를 출력합니다. Ctrl+C 또는 `kill -TERM`으로 정상 종료됩니다.

`./start.sh`는 서버를 띄우기 전에 관리자 창(웹 GUI, 아래 참고)이 안 떠 있으면 백그라운드로
자동 실행합니다 — Qt가 언제든 `http://<서버IP>:8083` 주소를 열 수 있게 하기 위함입니다
(포트는 `admin_console/config.sh`에서 설정 — 기본 8081은 이 RPi에서 부팅 자동실행되는
옛 복사본 `~/pos_receiver_eo`가 쓰고 있어 8083으로 비켜둠). 이미 그 포트에서 서비스
중이면 그대로 재사용하고, 서버만 Ctrl+C로 꺼도 웹 창은 계속 살아 있습니다. 웹 GUI 없이
서버만 띄우려면 기존처럼 `./server`를 직접 실행하면 됩니다.

### 관리자 창 (admin_console)

카메라 캘리브레이션 도구 + 서버 로그 모니터 + 로봇 제어 패널을 겸하는 웹 GUI입니다. 서버(9000)에 ADMIN(감시·제어)과 CCTV(카메라 좌표 통역) role로 접속합니다. 보통은 위의 `./start.sh`가 자동으로 띄워주므로 직접 실행할 일은 포트를 바꿀 때 정도입니다.

```bash
# 수동 실행 (서버 실행 후 별도 터미널에서)
./admin_console/start.sh    # 포트는 admin_console/config.sh에서 (현 서버: 6100 8083 6101)
# 또는: cd admin_console && python3 web_gui.py [카메라TCP포트] [HTTP포트] [스냅샷포트]
# 브라우저: http://<서버IP>:8083       카메라 캘리브레이션 대시보드
#          http://<서버IP>:8083/robot  로봇 제어 + 상태 배너
#          http://<서버IP>:8083/logs   서버 트래픽 로그 모니터 (role별 필터)
#          (로그인 전에는 위 셋 모두 /login 으로 리다이렉트됨)
# 서버가 다른 호스트면: RP_SERVER_HOST=x.x.x.x python3 web_gui.py
# 카메라 앱이 CAM_POSE를 보내는 대상(통역 다리)도 이 인스턴스의 TCP 포트(6100)로 맞출 것
```

⚠️ 같은 카메라를 바라보는 web_gui 인스턴스는 **한 개만** 띄울 것 (카메라는 설정된 포트 하나로만 접속). 다른 인스턴스가 이미 카메라 포트를 쓰고 있으면 포트가 겹치지 않게 조정하세요.

📋 **로그 상한** (POS tap이 15~30Hz로 흘러 무한정 쌓이는 걸 막기 위한 값들 — `rp_core.py`/`web_gui.py`에서 조정):

| 대상 | 상한 | 비고 |
|---|---|---|
| `admin_console/gui.log` | **2MB × 4개 (총 8MB)** | 초과 시 자동 회전 (`gui.log.1`~`.3`). 예전엔 무제한이라 8.4MB까지 커졌음 |
| 메모리 히스토리 (`LOG_HISTORY_MAX`) | **300줄** | 페이지를 열 때 브라우저로 한꺼번에 밀어넣는 양 — 크면 창이 버벅임 |
| 로봇 제어 페이지 화면 | **200줄** | 곁다리 로그라 작게 |
| 로그 모니터 화면 | **400줄** (원본 버퍼 1000줄) | 전용 화면이라 넉넉히 |

`gui_err.log`에는 파이썬 크래시 트레이스백만 쌓입니다(평소엔 비어 있음).

🔑 **관리자 창은 로그인 게이트 뒤에 있습니다.** 어느 주소로 들어가든 로그인 전에는 `/login` 화면만 뜨고, 로그인해야 카메라 캘리브레이션·로봇 제어·로그 모니터가 열립니다. 세션은 **브라우저별**이라 다른 PC는 각자 로그인해야 하고, **브라우저를 닫으면 재로그인**이 필요합니다(세션 쿠키). 상단 탭바의 **로그아웃** 버튼으로 즉시 끊을 수 있고, 서버(9000) 연결이 끊기면 모든 세션이 무효화됩니다.

이유는 **캘리 결과가 "그 시점에 로그인된 사용자"의 계정에 저장**되기 때문입니다. 로그인 없이 캘리를 끝내면 세션에만 남아 서버 재시작 시 사라지고(`[WARN] 캘리브레이션 수신 - 로그인 사용자 없음, 세션에만 유지`), **나중에 로그인해도 소급 저장되지 않습니다.** 그래서 순서를 사람이 기억하는 대신 게이트로 강제했습니다. (서버는 로그인 사용자를 한 명만 기억하므로, 이후 Qt가 다른 계정으로 로그인하면 저장 대상이 그쪽으로 바뀝니다.)

테스트 계정은 `python3 tools/seed_user.py`로 만들 수 있습니다 (기본 `test`/`1234`, [docs/TESTING.md](docs/TESTING.md) §2-1).

### 로봇·Qt 없이 주행 알고리즘 돌려보기

`robot_sim`(로봇+CCTV 대역)과 `draw_test`(Qt 대역)를 같이 띄우면 접근 → 도색 →
실시간 피드백 → 이탈 복귀 → 완료가 전부 돕니다. **운영 중인 9000 서버를 건드리지
않도록 옆 포트에 테스트 인스턴스를 띄우세요.**

```bash
make sim
```

터미널 3개에서:

```bash
./server 9100
```

```bash
./tools/robot_sim 127.0.0.1 --port 9100
```

```bash
./tools/draw_test 127.0.0.1 --port 9100 --side 1.0
```

`robot_sim`이 끝에 **펜 자취 요약**(획별 시작/끝 좌표와 도색 길이)을 찍습니다. 이 좌표가
`draw_test`가 출력한 도면 꼭짓점과 맞으면 "펜이 꼭짓점을 지난다"가 검증된 것입니다.
이탈 복귀까지 보려면 `robot_sim`에 조향 오차를 주입하세요:
`--drift-dps 15 --ignore-drift`. 자세한 항목은
[DRIVE_TEST_PLAN (노션 아카이브)](https://app.notion.com/p/3b914dc6aecf812183e3c64d07c1a8ac) 단계 C.

### 로봇 주행만 단독으로 (CCTV·도면 없이)

실제 로봇에 **접근 단계 없이** 제자리 사각형만 그리게 시킵니다. `drive_test`가
ADMIN role로 붙어 `PATH`를 로봇에 직접 중계시키므로 CCTV도 `BLUEPRINT`도 필요
없습니다 (서버의 경로생성·pose 판정을 아예 안 탑니다).

```bash
make drive_test
```

```bash
./server
```

```bash
./tools/drive_test 127.0.0.1 --side 0.3
```

한 변 0.3 m 정사각형을 **변마다 노즐을 내렸다 올리며**(획 4개) 그립니다. 첫 시운전은
`--no-paint`로 동선만 확인하세요. 방향은 로봇 자기 기준(PATH 수신 시점이 0도)이고,
CCTV가 없어 서버가 정렬 판정을 못 하므로 각도 정확도는 로봇 IMU에 달려 있습니다.
옵션·합격 기준은 [DRIVE_TEST_PLAN (노션 아카이브)](https://app.notion.com/p/3b914dc6aecf812183e3c64d07c1a8ac) 단계 A-3.

### 실시간 CCTV 없이 접근 → 도색 돌려보기 (cctv_pose)

실시간 POS가 아직 안 나올 때, **로봇 시작 위치만 한 번** 넣어주면 접근 → 도색 흐름이
개루프로 끝까지 돕니다. 서버는 pose가 한 번이라도 잡히면 `START_DRAW`를 진행하고,
그 pose는 만료되지 않습니다(`poseValid_`는 켜지기만 함). `DRIFT`·이탈 재계획은
`POS` 핸들러 안에만 있어 POS가 더 안 오면 자연히 멈춥니다.

```bash
make cctv_pose
```

```bash
./tools/cctv_pose 127.0.0.1
```

좌표는 [tools/cctv_pose.json](tools/cctv_pose.json)에서 읽습니다 — 값을 고치고 도구 콘솔에서
**Enter**를 치면 파일을 다시 읽어 재전송하므로, 재시작 없이 여러 번 시험할 수 있습니다
(`--repeat <초>`로 자동 반복도 가능). `corners`(CCTV 원본 픽셀 4점) 대신 `x`/`y`/`theta_deg`를
넣으면 캘리브레이션 없이도 좌표를 직접 지정할 수 있습니다.

⚠️ **Qt 로그인 후에 쏘세요.** 서버는 캘리브레이션을 로그인 시점에 계정에서 불러오므로,
그 전에 보낸 `POS`는 `pose 계산 불가`로 버려집니다.

🔴 **이 방식은 Qt가 `program`을 실어 보낸다는 전제입니다.** `program`이 없으면 서버가
도색 경로를 **낡은 출발 위치** 기준으로 만들어(`sendDrawPath` 폴백), 이미 시작점에 가 있는
로봇과 어긋납니다.

**카메라가 서버(9000)에 role=CCTV로 직접 붙게 되면** ([server_PROTOCOL.md](server_PROTOCOL.md)의 CCTV 연동 규격대로 전환 후),
web_gui의 CAM_POSE→POS 통역 다리를 꺼야 합니다 (안 끄면 카메라와 이 다리가 같은 role로
동시에 붙으려 해 서버가 재접속을 반복시킵니다):

```bash
RP_CCTV_BRIDGE=0 python3 web_gui.py   # 또는 admin_console/config.sh에 RP_CCTV_BRIDGE=0
```

캘리 도구(카메라 캘리브레이션 대시보드)·로그 모니터·로봇 제어는 이 다리와 무관하게
계속 동작합니다(ADMIN 연결로 별도 유지).

### 테스트용 콘솔 명령 (서버 실행 중 입력)

| 명령 | 동작 |
|---|---|
| `path` | 하드코딩 테스트 경로를 로봇에 전송 |
| `estop` / `resume` | 로봇 긴급정지 / 재개 |
| `calib` | 캘리브레이션 시작 (CCTV+로봇에 전달) |
| `who` | 현재 접속 중인 role 목록 |
| `quit` | 서버 종료 |

## 테스트 방법

### Qt 대역 시뮬레이터 (qt_sim)

실제 Qt 앱에 네트워킹이 붙기 전까지, QT 역할로 접속해 서버를 검증하는 도구입니다.

```bash
make qt_sim
./tools/qt_sim 127.0.0.1 certs/server.crt   # 같은 기기에서 서버 띄운 경우
```

접속 후 콘솔 명령: `register <id> <pw>` / `login <id> <pw>` / `cmd estop|resume|calib` / `blueprint`(테스트 도면 전송) / `quit`. 서버가 중계해주는 STATUS/POS/H_MATRIX 등은 자동으로 로그에 찍힙니다.

### 수동 테스트 (openssl)

클라이언트 흉내는 openssl만으로도 가능합니다:

```bash
openssl s_client -connect 127.0.0.1:9000 -CAfile certs/server.crt -quiet
# 접속 후 JSON 한 줄씩 입력 (첫 줄은 반드시 HELLO)
{"type":"HELLO","seq":1,"payload":{"role":"ROBOT"}}
```

⚠️ ROBOT role은 STATUS를 주기 전송(2초 이내 간격)해야 합니다 — 10초간 조용하면 서버가 연결을 끊습니다. QT/CCTV는 해당 없음.

## 자주 걸리는 것

- **"인증서/키 로드 실패"**: `./gen_cert.sh <서버IP>` 를 먼저 실행했는지 확인 (server.key는 git에 없으므로 기기마다 생성 필요)
- **"bind/listen 실패"**: 9000 포트를 이미 다른 서버 프로세스가 잡고 있음 (`pkill server` 후 재시도)
- **클라이언트 TLS 핸드셰이크 실패**: 클라이언트가 갖고 있는 server.crt가 현재 서버 것과 다름 (인증서 재생성했다면 다시 배포)
- `config/users.json` (사용자 계정 데이터)은 서버가 자동 생성하며 git에 올라가지 않습니다

## 아직 러프한 부분 (팀 협의 후 확정)

- POS 마커 `corners` 순서 — CCTV팀과 확정 필요
- 이탈 임계값 0.3 m / 재계획 간격 3초 / 커서 도달 반경 0.20 m — 현장 튜닝 예정
  (커서 반경은 펜 오프셋 155mm보다 크고 이탈 임계값보다는 작아야 한다 — router.hpp `kVertexReachM` 주석 참고)
- **펜 오프셋 `d`는 서버 상수로 확정** — 실측 **155 mm** (로봇 `NOZZLE_OFFSET_M`).
  `BLUEPRINT.pen_offset_m` 필드는 폐지됐고, 보정 자체를 로봇이 전담하기로 확정됐다
  (2026-07-28). 서버는 `router.hpp`의 `kPenOffsetM` 상수를 이탈 판정 여유값에만
  쓴다 — **로봇의 실제 오프셋이 바뀌면 이 상수도 손으로 같이 고쳐야 한다** (자동 동기화 없음)
- **회전 중 `DRIFT` 억제 = 로봇 담당으로 확정** — 로봇이 `TURN` 실행 중엔 스스로
  무시한다. 서버가 "지금 TURN 중"임을 알려주는 신호는 만들지 않는다 (이탈 감시는
  pose 기반이라 영향 없음). [DRIVE_TEST_PLAN (노션 아카이브)](https://app.notion.com/p/3b914dc6aecf812183e3c64d07c1a8ac) C-3
- **마커 중심 = 로봇 회전중심 가정** — 서버 pose는 마커 4코너 평균이라, ArUco 판이 회전중심 위에 있지 않으면 펜 오프셋과 별개인 또 하나의 오프셋이 생긴다. 기구팀 확인 필요
