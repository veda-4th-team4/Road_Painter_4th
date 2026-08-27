<p align="center">
  <img src="https://img.shields.io/badge/C++-17-00599C?style=flat&logo=c%2B%2B" />
  <img src="https://img.shields.io/badge/OpenSSL-TLS_1.2+-721412?style=flat&logo=openssl" />
  <img src="https://img.shields.io/badge/nlohmann-json-2C8EBB?style=flat" />
  <img src="https://img.shields.io/badge/MediaMTX-1.19.3-1976D2?style=flat" />
  <img src="https://img.shields.io/badge/Platform-Raspberry_Pi_4B-C51A4A?style=flat&logo=raspberrypi" />
  <img src="https://img.shields.io/badge/build-no_warnings-success?style=flat" />
</p>

# Road Painter — 중앙 관제 서버

> **"판단은 서버가, 실행은 로봇이"**
> Qt · 로봇 · CCTV 를 TLS 로 중계하고, 도면을 로봇 동작 시퀀스로 변환하는 단일 판단 지점
> **[↑ 프로젝트 루트로](../README.md)**

---

## 1. 개요 (Overview)

로봇은 자기가 어디 있는지 모릅니다. 좌표도 도면도 갖고 있지 않습니다.

**중앙 서버**는 카메라가 보낸 픽셀 좌표를 바닥 좌표로 환산하고, Qt 가 그린 도면을 현재 로봇
자세와 결합해 **실행 가능한 동작 시퀀스**로 바꾼 뒤, 로봇이 동작 하나를 실행할 때마다
**실제 관측 위치를 근거로 진행 여부와 보정량을 결정**합니다. 좌표 해석과 경로 결정 권한을
한곳에 집약한 구조(방식 B)이며, 로봇은 받은 동작만 실행합니다.

### 핵심 역할

1. **TLS 라우팅** — `QT`/`ROBOT`/`CCTV`/`ADMIN` 역할 기반 세션. 역할당 1연결, 수신 처리 전체를
   단일 뮤텍스로 직렬화해 세션 경합을 구조적으로 차단
2. **경로 생성 & 기구 보정** — 각도 부호 반전, 펜 오프셋·펜 두께 보정 op 삽입, 호 반지름 치환과
   진입 위상 보정, 도색 언더슛까지 전부 서버 몫 → [PATH_GEOMETRY](docs/PATH_GEOMETRY.md)
3. **폐루프 주행 판정** — 로봇의 `READY` 하나에 `GO`/`ALIGN`/`MORE` 중 **정확히 하나**로 응답
   (불변식). 직진 중에는 `DRIFT`, 측위 두절 2초면 `HOLD` → [PROTOCOL](docs/PROTOCOL.md)
4. **캘리브레이션 세션 중재** — 정적 앵커 · 오도메트리 주행 · 채널 간 정합 3종. 시작한 세션은
   **반드시 종결 응답 하나로만** 닫아 클라이언트가 대기 화면에 갇히지 않게 함
5. **4채널 RTSP 중계** — MediaMTX passthrough. 카메라 계정은 서버에만 보관하고 클라이언트에는
   중계 주소만 발급 → [relay](relay/README.md)

---

## 2. 아키텍처 (Architecture)

```
   ┌──────────────┐                                    ┌──────────────┐
   │  Qt 관제 PC   │◄────── TLS 9000 · JSON Lines ─────►│              │
   └──────┬───────┘         BLUEPRINT / START_DRAW      │              │
          │                 GO·ALIGN·MORE·HOLD          │   중앙 서버   │
          │ RTSP 8554                                   │  (RPi 4B)    │
          │                                             │              │
   ┌──────▼───────┐                                     │  ┌────────┐  │
   │   MediaMTX   │◄──────── RTSP 554 ─────────┐        │  │ Router │  │
   │  (passthru)  │                            │        │  └────────┘  │
   └──────────────┘                            │        │              │
                                        ┌──────┴─────┐  │              │
                                        │  CCTV 4ch  │◄─┤              │
                                        │  (엣지 앱)  │  │              │
                                        └────────────┘  │              │
   ┌──────────────┐                                     │              │
   │  도장 로봇    │◄────── TLS 9000 · JSON Lines ──────►│              │
   │  (RPi+STM32) │         PATH / READY / STATUS       └──────────────┘
   └──────────────┘
```

**제어(TLS 9000)와 영상(RTSP 8554)은 완전히 분리**되어 있습니다. 영상 대역폭이 명령 지연에
영향을 주지 않고, 카메라 자격증명은 서버 밖으로 나가지 않습니다.

---

## 3. 정량 지표 (Key Results)

| 평가 항목 | 개선 전 | **최종 결과** | 의미 |
| :--- | :---: | :---: | :--- |
| **원호(Arc) 궤적 이탈** | 310 mm | **1 mm 이내** | 진입 위상 보정으로 폐루프 진원 완성 |
| **사각형 꼭짓점 오차** | 6~8 mm | **1~2 mm** | 펜 두께 보정 + MORE 피드백 |
| **회전 정렬(ALIGN) 반복** | 최대 6회 진동 | **1~2회 수렴** | `cur_yaw` 전달 + 판정 대기창 |
| **4채널 중계 프레임 간격** | — | **p50 50.3 ms** | 20 fps 입력 한계 도달, 손실 0 (유선) |
| **4채널 동시 대역폭** | — | **약 10 Mbps** | 2592×1520 · 20 fps · 재인코딩 없음 |
| **튜닝 파라미터 외부화** | 코드 하드코딩 | **28개 무재컴파일** | 값 수정 후 서버만 재시작 |

---

## 4. 빠른 시작 (Quick Start)

### 1) 의존성 설치
```bash
sudo apt install g++ make libssl-dev nlohmann-json3-dev
```

### 2) TLS 인증서 생성 (최초 1회)
```bash
./gen_cert.sh 192.168.0.8
```
> 🔴 **서버 IP 를 반드시 인자로 넣을 것.** 안 넣으면 클라이언트 인증서 검증이 실패합니다.
> 생성된 `certs/server.crt` 를 로봇 · Qt · CCTV 에 복사해 신뢰 CA 로 등록합니다.

### 3) 빌드 & 실행
```bash
make -j4 && ./start.sh
```
포트는 **9000** (TCP/TLS). 포그라운드로 돌며 로그를 출력하고 `Ctrl+C` 로 정상 종료됩니다.
`start.sh` 는 [관리자 창](admin_console/README.md)이 안 떠 있으면 백그라운드로 같이 띄웁니다.
웹 GUI 없이 서버만 띄우려면 `./server` 를 직접 실행합니다.

### 4) 영상 중계 (선택)
```bash
cd relay && ./start.sh -d
```

> ⚠️ **테스트 서버는 다른 포트로** (`./server 9100`). 같은 role 로 새로 접속하면 기존 연결이
> 끊기므로, 9000 에 띄우면 현장 카메라가 쫓겨납니다.

---

## 5. 문서 (Documentation)

| 문서 | 이런 게 궁금할 때 |
| :--- | :--- |
| **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)** | 코드 어디를 고쳐야 하지? |
| **[docs/PROTOCOL.md](docs/PROTOCOL.md)** | 메시지를 어떤 형식으로 주고받지? |
| **[docs/PATH_GEOMETRY.md](docs/PATH_GEOMETRY.md)** | 로봇이 왜 저기로 가지? (펜 오프셋·호 기하) |
| **[docs/CALIBRATION.md](docs/CALIBRATION.md)** | 캘리브레이션 어떻게 하지? |
| **[docs/TUNING.md](docs/TUNING.md)** | 이 값 어디서 바꾸지? |
| **[tools/README.md](tools/README.md)** | 테스트 어떻게 돌리지? |
| **[relay/README.md](relay/README.md)** | 카메라 영상 중계는? |
| **[admin_console/README.md](admin_console/README.md)** | 관리자 창은? |

처음 보는 사람은 **ARCHITECTURE → PROTOCOL** 순서로 읽으면 됩니다.

> 🔴 **튜닝 상수는 코드에 없습니다.** 전부 [`config/params.json`](config/params.json) 에 있고
> 값을 고치고 **서버만 재시작하면 됩니다** — 재컴파일 불필요 → [TUNING](docs/TUNING.md)

---

## 6. 저장소 구조

```
Server/                         C++17 · 18파일 · 5,738행
├── Makefile                    빌드 스크립트 (-Wall, 경고 0)
├── start.sh                    통합 실행 (관리자 창 자동 시작 + 서버)
├── gen_cert.sh                 TLS 자체서명 인증서 생성 (최초 1회)
│
├── certs/                      server.crt(공개) / server.key(비밀, git 제외)
├── config/
│   ├── params.json             🔴 튜닝 상수 28개 — 편집 대상
│   ├── users.json              계정 + 채널별 캘리 번들 (자동 생성, git 제외)
│   ├── calib_latest.json       전역 캘리 슬롯
│   ├── camera.json             전역 카메라 IP
│   └── stream.json.example     중계 RTSP 주소 템플릿
│
├── src/
│   ├── main.cpp                시작점 · 파라미터 로드 · graceful shutdown
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
│   ├── protocol.hpp            메시지 계약 주석 + 생성 헬퍼
│   ├── stream_cfg.hpp          LOGIN_OK.stream 설정
│   ├── user_store.hpp/cpp      계정 저장소 (PBKDF2-SHA256 + 채널별 캘리 영속화)
│   └── log.hpp                 타임스탬프 로그
│
├── docs/                       문서 5종 (위 표 참고)
├── tools/                      회귀 테스트 · 시뮬레이터 14종 → tools/README.md
├── relay/                      4채널 RTSP 중계 (MediaMTX)   → relay/README.md
└── admin_console/              관리자 웹 GUI (Python)        → admin_console/README.md
```

접속한 클라이언트마다 전담 스레드가 자기 소켓을 읽고, 받은 메시지를 `Router` 가 규칙에 따라
다른 클라이언트 소켓으로 배달합니다 (thread-per-connection).
자세한 구조는 [ARCHITECTURE](docs/ARCHITECTURE.md) 참고.

### 단독 검증

로봇 · 카메라 · Qt 없이도 서버만 돌려볼 수 있습니다.

```bash
make robot_sim && ./robot_sim      # 로봇 + CCTV 대역 시뮬레이터
make draw_test && ./draw_test      # 도면 등록부터 도색 완료까지
```

회귀 테스트 6종 · 시뮬레이터 5종 · 진단 도구 3종 → [tools/README.md](tools/README.md)

---

## 7. 자주 걸리는 것 (Troubleshooting)

| 증상 | 원인 |
| :--- | :--- |
| 접속하자마자 끊긴다 | `HELLO` 를 10초 안에 안 보냈다 |
| 인증서 검증 실패 | `gen_cert.sh` 에 서버 IP 를 안 넣었다 |
| 붙었는데 반응이 없다 | 같은 role 로 다른 클라이언트가 이미 붙어 있어 밀려났다 |
| `POS` 를 보내는데 pose 가 안 잡힌다 | 활성 채널이 아닌 채널의 `POS` 는 전부 버려진다 |
| 로봇이 op 하나에서 영원히 멈춰 있다 | 그 `READY` 에 서버가 응답을 안 보냈다 |
| 정렬이 계속 반복된다 | `feedback_wait_ms` 가 너무 짧아 회전 전 각도로 판정하고 있다 |

더 많은 증상과 재현 방법은 [tools/README](tools/README.md) 참고.

---

## 8. 알아둘 것 (Known Limitations)

- **`POS` 실측이 설계 전제보다 훨씬 느립니다.** 설계는 15~30 Hz 전제인데 현장은 **1~2 Hz**,
  도색 구간에서는 **0 Hz** 까지 떨어집니다. 피드백 튜닝 문제 대부분이 여기서 나옵니다.
- **서버는 클라이언트를 인증하지 않습니다 (mTLS 없음).** 누구나 `HELLO{role:"QT"}` 로
  진짜 Qt 를 밀어낼 수 있습니다.
- **영상 중계 구간은 평문 RTSP 입니다.** 현장 폐쇄망을 전제로 읽기 접근을 열어두었고,
  카메라 자격증명만 서버에 격리합니다. 개방망 적용 시 RTSPS 와 읽기 계정이 필요합니다.
- 10초간 무응답인 로봇은 연결 끊김으로 처리합니다. QT/CCTV 는 무한 대기입니다.
