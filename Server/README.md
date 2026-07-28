# Road-Painter 중앙 서버

Qt(관제 UI) · 로봇(도색 로봇) · CCTV · 관리자 창 네 클라이언트를 중계하고, 도면으로부터 로봇 경로를 생성하는 중앙 서버입니다. 서버 RPi에서 실행합니다.

## 하는 일

- **TLS 릴레이**: 클라이언트가 role(QT / ROBOT / CCTV / ADMIN)로 등록하면, 메시지를 규칙에 따라 상대에게 중계
- **로그인 / 캘리브레이션 저장**: 사용자(id/비번)별로 캘리브레이션 번들(K, D, H행렬)을 저장했다가 재로그인 시 Qt에 돌려줌
- **경로 생성 (2단계)**: 1단계 접근(approach)은 **서버**가 CCTV로 파악한 로봇 위치에서 도면 시작점까지 만든다. 2단계 도색(draw)은 **Qt가 만들어 보낸 동작 시퀀스(`BLUEPRINT.program`)를 서버가 그대로 중계**한다 (2026-07-28 변경 — 꼭짓점 펜 오프셋 보정 때문. `program`이 없으면 종전대로 서버가 직접 생성)
- **출발 전 정렬 / 주행 피드백**: MOVE 시작 전 READY/ALIGN/GO 핸드셰이크로 각도 미세조정, 직진 중 DRIFT로 각도 이탈 피드백. 단 **꼭짓점 동작(`pivot`) 구간에서는 둘 다 차단** (각도를 건드리면 펜이 꼭짓점으로 못 돌아옴)
- **이탈 감시·재계획**: **지금 달리는 구간**에서 0.3 m 이상 벗어나면 원래 향하던 꼭짓점으로 복귀시킨다 (도면 전체 기준이면 나란한 줄 사이에서 옆줄로 튄다)
- **하트비트**: 로봇이 10초간 무응답이면 연결 끊김으로 처리
- **관리자 창 지원(ADMIN)**: 서버가 중계하는 모든 메시지 사본을 TAP으로 관리자 창에 전달(로그 모니터), 관리자 창에서 온 로봇 명령/캘리 결과를 처리

## 문서

| 문서 | 내용 |
|---|---|
| **[server_PROTOCOL.md](server_PROTOCOL.md)** | 통신 규격 전체 (각 팀이 봐야 할 문서). CCTV/Qt/로봇 연동 스펙도 여기로 통합됨 |
| [docs/TESTING.md](docs/TESTING.md) | 서버/Qt 테스트 가이드 |
| [docs/DRIVE_TEST_PLAN.md](docs/DRIVE_TEST_PLAN.md) | 로봇 주행 통합 테스트 계획 (단계 A~D, 합격 기준) |
| [docs/REFACTOR_SUMMARY.md](docs/REFACTOR_SUMMARY.md) | graceful shutdown 개선 기록 |
| [admin_console/PLAN.md](admin_console/PLAN.md) | 관리자 창 설계/진행 상황 |

## 파일 구성

```
Server/
├── Makefile            빌드 스크립트
├── server_PROTOCOL.md  통신 프로토콜 문서 (로봇/QT/CCTV 팀용, 단일 창구)
├── start.sh            통합 실행 (관리자 창 자동 시작 + 서버 실행)
├── gen_cert.sh         TLS 자체서명 인증서 생성 (최초 1회)
├── certs/              server.crt(공개) / server.key(비밀, git 제외)
├── config/             users.json (서버가 자동 생성, git 제외)
├── docs/               부속 문서 (테스트 가이드, 주행 테스트 계획, 리팩터 기록)
├── src/
│   ├── main.cpp            시작점 + 테스트용 콘솔 + graceful shutdown
│   ├── tls_server.hpp/cpp  TLS 네트워크 층 (접속, role 등록, 세션 스레드, ADMIN tap)
│   ├── router.hpp/cpp      메시지 라우팅 (중계 규칙 + 경로생성/재계획/정렬 판단)
│   ├── path_planner.hpp    경로 계산 (마커→pose, 도면→MOVE/TURN, 이탈 거리)
│   ├── calib.hpp           캘리브레이션 번들 파싱 + undistort/호모그래피 수학
│   ├── user_store.hpp/cpp  사용자 저장소 (비번 해시 + 캘리브레이션 영속화)
│   ├── protocol.hpp        메시지 스펙 주석 + 생성 헬퍼
│   └── log.hpp             타임스탬프 로그
├── tools/
│   ├── qt_sim.cpp          Qt 대역 테스트 클라이언트 (Qt 네트워킹 나오기 전 검증용)
│   ├── path_test.cpp       최초 1회 경로생성 테스트기 (CCTV 스냅샷 주입 → 접근 PATH 검증)
│   ├── seed_user.py        테스트 계정 생성 (기본 test/1234 + 예시 캘리브레이션)
│   └── *_snapshot.json     path_test용 CCTV 스냅샷 (호모그래피 + 마커 4코너)
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
./qt_sim 127.0.0.1 certs/server.crt   # 같은 기기에서 서버 띄운 경우
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
- 이탈 임계값 0.3 m / 재계획 간격 3초 / 커서 도달 반경 0.15 m — 현장 튜닝 예정
  (커서 반경은 이탈 임계값보다 확실히 작아야 한다 — 너무 크면 꼭짓점 직전의 정상 주행이 오탐된다)
- **펜 오프셋 `d`의 출처** — 지금은 Qt 설정값(기본 40 mm, 임시)을 `BLUEPRINT.pen_offset_m`으로 받는다. 기구값이므로 최종적으로 서버가 내려주는 형태가 맞는지 Qt팀과 협의 필요
- **마커 중심 = 로봇 회전중심 가정** — 서버 pose는 마커 4코너 평균이라, ArUco 판이 회전중심 위에 있지 않으면 펜 오프셋과 별개인 또 하나의 오프셋이 생긴다. 기구팀 확인 필요
