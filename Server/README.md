# Road-Painter 중앙 서버

Qt(관제 UI) · 로봇(도색 로봇) · CCTV · 관리자 창 — 네 클라이언트를 중계하고,
도면으로부터 로봇 경로를 생성하는 중앙 서버. 서버 RPi 에서 실행한다.

```
            ROBOT ─┐ 
               Qt ─┐                        
                   ├── TLS 9000 ── [서버]
             CCTV ─┘
             ADMIN─┘
```

---

## 📖 문서

| 문서 | 이런 게 궁금할 때 |
|---|---|
| **[docs/PROTOCOL.md](docs/PROTOCOL.md)** | 메시지를 어떤 형식으로 주고받지? |
| **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)** | 코드 어디를 고쳐야 하지? |
| **[docs/PATH_GEOMETRY.md](docs/PATH_GEOMETRY.md)** | 로봇이 왜 저기로 가지? (펜 오프셋·호 기하) |
| **[docs/CALIBRATION.md](docs/CALIBRATION.md)** | 캘리브레이션 어떻게 하지? |
| **[docs/TUNING.md](docs/TUNING.md)** | 이 값 어디서 바꾸지? |
| **[tools/README.md](tools/README.md)** | 테스트 어떻게 돌리지? |
| **[relay/README.md](relay/README.md)** | 카메라 영상 중계는? |
| **[admin_console/README.md](admin_console/README.md)** | 관리자 창은? |

처음 보는 사람은 **ARCHITECTURE → PROTOCOL** 순서로 읽으면 된다.

---

## 하는 일

- **TLS 릴레이** — 클라이언트가 role(`QT`/`ROBOT`/`CCTV`/`ADMIN`)로 등록하면 규칙에 따라 상대에게 중계한다.
- **경로 생성** — Qt 가 보낸 도면을 로봇 op 배열로 바꾼다.
  각도 부호 반전, 펜 오프셋·펜 두께 보정 op 삽입, 호 반지름 치환과 진입 위상 보정,
  도색 언더슛까지 전부 서버 몫이다 → [PATH_GEOMETRY](docs/PATH_GEOMETRY.md)
- **주행 판정** — 로봇은 op 하나마다 `READY` 를 보내고 서버가 `GO`/`ALIGN`/`MORE` 중
  하나로 답해야 움직인다. 직진 중에는 `DRIFT`, 위치를 놓치면 `HOLD` → [PROTOCOL](docs/PROTOCOL.md)
- **로그인·캘리브레이션 저장** — 계정별로 번들(`K`,`D`,`H`)을 저장했다가 재로그인 때 Qt 에 돌려준다.
  **채널별로 따로 저장한다** — 채널마다 렌즈 방향이 달라 값이 전부 다르다.
- **채널 전환** — `SELECT_CHANNEL` 로 작업 채널을 바꾸면 그 채널 캘리가 적용되고,
  **활성 채널이 아닌 `POS` 는 무시한다.** `ch` 는 전부 선택 필드라 단일 채널 현장은 그대로 동작한다.
- **작업 완전 중지 (`ABORT_DRAW`)** — `ESTOP` 이 일시정지라면 이건 취소다.
  서버가 든 경로 상태를 비우고 로봇에도 중계해 받아둔 경로까지 버리게 한다.
- **구역 진입 경보** — CCTV 의 `ZONE_EVENT Enter` 를 기존 TLS 세션으로 ROBOT 에 중계한다.
  별도 UDP 포트는 쓰지 않는다.
- **중계 스트림 주소 배포** — `config/stream.json` 이 있으면 `LOGIN_OK.stream` 으로
  RTSP 베이스 주소를 Qt 에 내려준다. 파일이 없으면 필드를 안 보내고 Qt 는 자기 설정값을 쓴다.
- **관리자 창 지원** — 중계하는 모든 메시지 사본을 `TAP` 으로 보내고, 관리자 창의 명령을 처리한다.

> 🔴 **튜닝 상수는 코드에 없다.** 전부 [`config/params.json`](config/params.json) 에 있고
> 값을 고치고 **서버만 재시작하면 된다** — 재컴파일 불필요 → [TUNING](docs/TUNING.md)

---

## 빌드 & 실행

```bash
sudo apt install g++ make libssl-dev nlohmann-json3-dev
```

```bash
./gen_cert.sh 192.168.0.8
```

인증서는 최초 1회만 만든다. **서버 IP 를 반드시 넣어야** 클라이언트 검증이 통과한다.
생성된 `certs/server.crt` 를 로봇·Qt·CCTV 에 복사해 신뢰 CA 로 등록한다.

```bash
make && ./start.sh
```

포트는 **9000**(TCP/TLS). 포그라운드로 돌며 로그를 출력하고 `Ctrl+C` 로 정상 종료된다.

`start.sh` 는 서버를 띄우기 전에 [관리자 창](admin_console/README.md)이 안 떠 있으면
백그라운드로 같이 띄운다. 서버만 끄더라도 웹 창은 계속 살아 있다.
웹 GUI 없이 서버만 띄우려면 `./server` 를 직접 실행한다.

> ⚠️ **테스트 서버는 다른 포트로 띄울 것** (`./server 9100`).
> 같은 role 로 새로 접속하면 기존 연결이 끊기므로, 9000 에 띄우면 현장 카메라가 쫓겨난다.

---

## 파일 구성

```
Server/
├── README.md           ← 지금 이 문서
├── Makefile            빌드 스크립트
├── start.sh            통합 실행 (관리자 창 자동 시작 + 서버)
├── gen_cert.sh         TLS 자체서명 인증서 생성 (최초 1회)
├── certs/              server.crt(공개) / server.key(비밀, git 제외)
├── config/
│   ├── params.json         🔴 튜닝 상수 — 편집 대상
│   ├── users.json          계정 + 채널별 캘리 번들 (자동 생성, git 제외)
│   ├── calib_latest.json   전역 캘리 슬롯
│   ├── camera.json         전역 카메라 IP
│   └── stream.json.example 중계 RTSP 주소 템플릿
│
├── docs/               문서 5종 (위 표 참고)
│
├── src/
│   ├── main.cpp                시작점 · 파라미터 로드 · 콘솔 · graceful shutdown
│   ├── tls_server.hpp/cpp      TLS 층 (접속, role 등록, 세션 스레드, ADMIN tap)
│   ├── router.hpp/cpp          ★ 라우팅 + 모든 상태 + 주행 판정
│   ├── router_channel.cpp      채널 전환 · 작업 취소
│   ├── router_calib.cpp        캘리 세션 (정적 앵커)
│   ├── router_odocalib.cpp     캘리 세션 (오도메트리 주행)
│   ├── router_registration.cpp 채널 간 정합 수집
│   ├── ops_builder.hpp         ★ Qt program → 로봇 op (부호·펜보정·호·언더슛)
│   ├── params.hpp              🔴 튜닝 상수 정의 + JSON 로딩 (하드코딩 금지 지점)
│   ├── path_planner.hpp        마커 4점 → pose 추정
│   ├── calib.hpp               캘리 번들 파싱 + undistort/호모그래피
│   ├── protocol.hpp            메시지 계약 주석 + 생성 헬퍼 + 채널 헬퍼
│   ├── stream_cfg.hpp          LOGIN_OK.stream 설정
│   ├── user_store.hpp/cpp      계정 저장소 (비번 해시 + 채널별 캘리 영속화)
│   └── log.hpp                 타임스탬프 로그
│
├── tools/              테스트 도구 · 시뮬레이터  → tools/README.md
├── relay/              4채널 RTSP 중계 (MediaMTX) → relay/README.md
└── admin_console/      관리자 웹 GUI (Python)     → admin_console/README.md
```

접속한 클라이언트마다 전담 스레드가 자기 소켓을 읽고, 받은 메시지를 Router 가 규칙에 따라
다른 클라이언트 소켓으로 배달한다 (thread-per-connection).
자세한 구조는 [ARCHITECTURE](docs/ARCHITECTURE.md) 참고.

---

## 자주 걸리는 것

| 증상 | 원인 |
|---|---|
| 접속하자마자 끊긴다 | `HELLO` 를 10초 안에 안 보냈다 |
| 인증서 검증 실패 | `gen_cert.sh` 에 서버 IP 를 안 넣었다 |
| 붙었는데 반응이 없다 | 같은 role 로 다른 클라이언트가 이미 붙어 있어 밀려났다 |
| `POS` 를 보내는데 pose 가 안 잡힌다 | 활성 채널이 아닌 채널의 `POS` 는 전부 버려진다 |
| 로봇이 op 하나에서 영원히 멈춰 있다 | 그 `READY` 에 서버가 응답을 안 보냈다 |
| 정렬이 계속 반복된다 | `feedback_wait_ms` 가 너무 짧아 회전 전 각도로 판정하고 있다 |

더 많은 증상과 재현 방법은 [tools/README](tools/README.md) 참고.

---

## 알아둘 것

- **`POS` 실측이 설계 전제보다 훨씬 느리다.** 설계는 15~30 Hz 전제인데 현장은 **1~2 Hz**,
  도색 구간에서는 **0 Hz** 까지 떨어진다. 피드백 튜닝 문제 대부분이 여기서 나온다.
- **서버는 클라이언트를 인증하지 않는다(mTLS 없음).** 누구나 `HELLO{role:"QT"}` 로
  진짜 Qt 를 밀어낼 수 있다.
- 10초간 무응답인 로봇은 연결 끊김으로 처리한다. QT/CCTV 는 무한 대기다.
