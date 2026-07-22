# Road-Painter 중앙 서버

Qt(관제 UI) · 로봇(도색 로봇) · CCTV · 관리자 창 네 클라이언트를 중계하고, 도면으로부터 로봇 경로를 생성하는 중앙 서버입니다. 서버 RPi에서 실행합니다.

## 하는 일

- **TLS 릴레이**: 클라이언트가 role(QT / ROBOT / CCTV / ADMIN)로 등록하면, 메시지를 규칙에 따라 상대에게 중계
- **로그인 / 캘리브레이션 저장**: 사용자(id/비번)별로 캘리브레이션 번들(K, D, H행렬)을 저장했다가 재로그인 시 Qt에 돌려줌
- **경로 생성 (2단계)**: Qt 도면 + CCTV 마커로 파악한 로봇 위치를 조합해 동작 시퀀스(직진/회전)를 생성. 1단계 접근(approach) → Qt "그림그리기 시작" → 2단계 도색(draw)
- **출발 전 정렬 / 주행 피드백**: MOVE 시작 전 READY/ALIGN/GO 핸드셰이크로 각도 미세조정, 직진 중 DRIFT로 각도 이탈 피드백
- **이탈 감시·재계획**: 로봇이 계획 경로에서 0.3 m 이상 벗어나면 현재 위치 기준으로 경로를 다시 짜서 재전송
- **하트비트**: 로봇이 10초간 무응답이면 연결 끊김으로 처리
- **관리자 창 지원(ADMIN)**: 서버가 중계하는 모든 메시지 사본을 TAP으로 관리자 창에 전달(로그 모니터), 관리자 창에서 온 로봇 명령/캘리 결과를 처리

## 문서

| 문서 | 내용 |
|---|---|
| **[PROTOCOL.md](PROTOCOL.md)** | 통신 규격 전체 (각 팀이 봐야 할 문서) |
| [docs/TESTING.md](docs/TESTING.md) | 서버/Qt 테스트 가이드 |
| [docs/CCTV_CAMERA_SPEC.md](docs/CCTV_CAMERA_SPEC.md) | 카메라 앱을 서버(9000)에 직접 붙이기 위한 CCTV팀 전달용 스펙 |
| [docs/QT_CLIENT_SPEC.md](docs/QT_CLIENT_SPEC.md) | Qt 관제 클라이언트를 서버(9000)에 붙이기 위한 Qt팀 전달용 스펙 |
| [docs/ROBOT_INTEGRATION_TODO.md](docs/ROBOT_INTEGRATION_TODO.md) | 로봇 RPi 코드의 v0.3 프로토콜 반영 갭 리스트 (로봇팀 전달용) |
| [docs/REFACTOR_SUMMARY.md](docs/REFACTOR_SUMMARY.md) | graceful shutdown 개선 기록 |
| [admin_console/PLAN.md](admin_console/PLAN.md) | 관리자 창 설계/진행 상황 |

## 파일 구성

```
Server/
├── Makefile            빌드 스크립트
├── PROTOCOL.md         통신 프로토콜 문서 (로봇/QT/CCTV 팀용)
├── gen_cert.sh         TLS 자체서명 인증서 생성 (최초 1회)
├── certs/              server.crt(공개) / server.key(비밀, git 제외)
├── config/             users.json (서버가 자동 생성, git 제외)
├── docs/               부속 문서 (테스트 가이드, CCTV 스펙, 리팩터 기록)
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
│   └── qt_sim.cpp          Qt 대역 테스트 클라이언트 (Qt 네트워킹 나오기 전 검증용)
└── admin_console/          관리자 창 (Python 웹 GUI - 카메라 캘리 도구 + 서버 로그/로봇 제어)
    ├── web_gui.py              대시보드( / ), 로봇 제어( /robot ), 로그 모니터( /logs )
    ├── pose_server.py          터미널판 카메라 하니스 (web_gui의 원형)
    ├── start.sh                백그라운드 실행 스크립트 (포트는 config.sh에서)
    ├── config.sh.example       포트 설정 템플릿 (config.sh로 복사해 사용, config.sh는 git 제외)
    └── PLAN.md                 설계/진행 상황
```

구조: 접속한 클라이언트마다 전담 스레드가 생겨 자기 소켓을 읽고, 받은 메시지는 Router가 규칙에 따라 다른 클라이언트 소켓으로 배달합니다 (thread-per-connection).

## 빌드 & 실행

```bash
# 필요 패키지 (최초 1회)
sudo apt install g++ make libssl-dev nlohmann-json3-dev

# 인증서 생성 (최초 1회, 서버 IP 넣기)
./gen_cert.sh 192.168.0.8
# -> certs/server.crt 를 로봇/Qt/CCTV 클라이언트에 복사 (신뢰 CA로 사용)

# 빌드 & 실행
make
./server
```

포트는 **9000** (TCP/TLS). 실행하면 포그라운드에서 돌며 로그를 출력합니다. Ctrl+C 또는 `kill -TERM`으로 정상 종료됩니다.

### 관리자 창 (admin_console)

카메라 캘리브레이션 도구 + 서버 로그 모니터 + 로봇 제어 패널을 겸하는 웹 GUI입니다. 서버(9000)에 ADMIN(감시·제어)과 CCTV(카메라 좌표 통역) role로 접속합니다.

```bash
# 서버 실행 후 별도 터미널에서
cd admin_console
python3 web_gui.py [카메라TCP포트] [HTTP포트] [스냅샷포트]   # 기본 6000 8081 6001
# 브라우저: http://<서버IP>:8081       카메라 캘리브레이션 대시보드
#          http://<서버IP>:8081/robot  로봇 제어 + 상태 배너
#          http://<서버IP>:8081/logs   서버 트래픽 로그 모니터 (role별 필터)
# 서버가 다른 호스트면: RP_SERVER_HOST=x.x.x.x python3 web_gui.py
```

⚠️ 같은 카메라를 바라보는 web_gui 인스턴스는 **한 개만** 띄울 것 (카메라는 설정된 포트 하나로만 접속). 다른 인스턴스가 이미 카메라 포트를 쓰고 있으면 포트가 겹치지 않게 조정하세요.

**카메라가 서버(9000)에 role=CCTV로 직접 붙게 되면** (`docs/CCTV_CAMERA_SPEC.md` 완료 후),
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

- BLUEPRINT `points` 포맷 — Qt팀과 확정 필요
- POS 마커 `corners` 순서 — CCTV팀과 확정 필요
- 이탈 임계값 0.3 m / 재계획 간격 3초 — 현장 튜닝 예정
