# Road-Painter 중앙 서버

Qt(관제 UI) · 로봇(도색 로봇) · CCTV 세 클라이언트를 중계하고, 도면으로부터 로봇 경로를 생성하는 중앙 서버입니다. 서버 RPi에서 실행합니다.

## 하는 일

- **TLS 릴레이**: 클라이언트가 role(QT / ROBOT / CCTV)로 등록하면, 메시지를 규칙에 따라 상대에게 중계
- **로그인 / H행렬 저장**: 사용자(id/비번)별로 캘리브레이션 결과(호모그래피 H행렬)를 저장했다가 재로그인 시 Qt에 돌려줌
- **경로 생성**: Qt가 보낸 도면(top-view 좌표) + CCTV 마커 검출로 파악한 로봇 위치를 조합해, 로봇용 동작 명령 시퀀스(직진/회전)를 만들어 전송
- **이탈 감시·재계획**: 로봇이 계획 경로에서 0.3 m 이상 벗어나면 현재 위치 기준으로 경로를 다시 짜서 재전송
- **하트비트**: 로봇이 10초간 무응답이면 연결 끊김으로 처리

메시지 형식·필드 등 통신 규격 전체는 **[PROTOCOL.md](PROTOCOL.md)** 참고 (각 팀이 봐야 할 문서).

## 파일 구성

```
Server/
├── Makefile            빌드 스크립트
├── PROTOCOL.md         통신 프로토콜 문서 (로봇/QT/CCTV 팀용)
├── gen_cert.sh         TLS 자체서명 인증서 생성 (최초 1회)
├── certs/              server.crt(공개) / server.key(비밀, git 제외)
├── src/
│   ├── main.cpp            시작점 + 테스트용 콘솔
│   ├── tls_server.hpp/cpp  TLS 네트워크 층 (접속, role 등록, 세션 스레드)
│   ├── router.hpp/cpp      메시지 라우팅 (중계 규칙 + 경로생성/재계획 판단)
│   ├── path_planner.hpp    경로 계산 (마커→pose, 도면→MOVE/TURN, 이탈 거리)
│   ├── user_store.hpp/cpp  사용자 저장소 (비번 해시 + H행렬 영속화)
│   ├── protocol.hpp        메시지 스펙 주석 + 생성 헬퍼
│   └── log.hpp             타임스탬프 로그
└── tools/
    └── qt_sim.cpp          Qt 대역 테스트 클라이언트 (Qt 네트워킹 나오기 전 검증용)
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

포트는 **9000** (TCP/TLS). 실행하면 포그라운드에서 돌며 로그를 출력합니다.

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
