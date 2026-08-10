# 서버 구조 정리

**작성일**: 2026-08-10 · **기준**: `main` (프로토콜 v2 + v0.4 채널 + 캘리 세션)

소스 전체(4,024줄)를 읽고 정리한 것이다. 각 파일의 주석이 "왜 이렇게 했는가"를
잘 남겨두고 있으므로, 이 문서는 그것을 반복하지 않고 **파일을 넘나드는 구조와
불변식**, 그리고 **혼자 읽어서는 안 보이는 함정**에 집중한다.

---

## 1. 서버가 하는 일 / 하지 않는 일

한 줄로: **좌표계의 주인이자 로봇 주행의 두뇌.** 중계기가 아니다.

| 하는 일 | 하지 않는 일 |
|---|---|
| 픽셀 → 미터 좌표 변환 (왜곡 보정 + 호모그래피) | 영상 처리 (OpenCV·ffmpeg 링크 안 함) |
| Qt 논리 동작 → 로봇 op 시퀀스 변환 | RTSP 중계 (MediaMTX 별도 프로세스) |
| 주행 중 실시간 피드백 (ALIGN/MORE/DRIFT) | 마커 검출 (CCTV 앱) |
| 캘리브레이션 영속 저장 | 캘리브레이션 **계산** (CCTV 앱) |
| 사용자 인증 (PBKDF2-SHA256) | 경로 **작도** (Qt) |

`Server/relay/`에 MediaMTX 설정이 같이 있지만 서버 프로세스와 무관하다.

---

## 2. 파일 지도

```
main.cpp          부팅 · 스레드 3개 · 콘솔 명령
 └ tls_server.*   TLS · role 레지스트리 · JSON Lines · ADMIN 미러링
    └ router.*    ★ 전부. 라우팅 + 모든 상태 + 주행 판정
       ├ router_channel.cpp   채널 전환
       └ router_calib.cpp     호모그래피 세션 (2026-08-10)

계산·데이터 (헤더 온리):
  protocol.hpp     메시지 계약 문서 + makeMsg/nowMs/채널 헬퍼
  calib.hpp        번들 파싱 · undistort · 호모그래피 적용
  path_planner.hpp POS 4코너 → pose (여기 "경로 생성"은 없다)
  ops_builder.hpp  ★ Qt program → 로봇 op (부호·펜오프셋·호 위상)
  params.hpp       튜닝 상수 전부 (X-매크로로 필드/로딩/검증 자동 생성)
  user_store.*     users.json / calib_latest.json / camera.json
  stream_cfg.hpp   LOGIN_OK.stream 3-상태
  log.hpp          logf + ADMIN 싱크
```

`protocol.hpp` 상단 300줄은 **주석으로 된 프로토콜 정본**이다. 메시지를 추가할
때 여기부터 고치는 것이 관행이다.

---

## 3. 스레드와 락

```
[accept 루프]  poll(200ms) → 접속마다 세션 스레드 생성 (detach)
[세션 스레드]  N개. readLine → Router::onMessage
[tick 스레드]  200ms 주기 → Router::tick
[콘솔 스레드]  getline (detach)
```

**락은 세 종류, 겹치지 않는다.**

| 락 | 보호 대상 | 규칙 |
|---|---|---|
| `Router::mtx_` | 라우터 상태 **전부** | `onMessage`/`tick` 진입 시 통째로 |
| `TlsServer::mtx_` | role→클라이언트 레지스트리 | 짧게만 |
| `Client::sslMtx` | SSL 객체 하나 (읽기+쓰기) | 블로킹 중 쥐지 않음 |

🔴 **`Router::mtx_`가 모든 것을 직렬화한다는 사실이 설계의 토대다.** 계약 §7-1이
물었던 "채널 전환과 세션 개시의 원자성"이 공짜로 얻어진 것도 이것 때문이다.
동시성 버그를 걱정할 필요가 없는 대신, 라우터 안에서 블로킹 I/O를 하면 서버
전체가 멈춘다.

**데드락 회피 두 곳** (주석에 이유가 있다):
- `peerHandler_`는 `TlsServer::mtx_`를 **놓은 뒤** 호출한다. 핸들러가 `sendTo`로
  같은 락을 다시 잡기 때문.
- `readLine`은 `poll()`로 데이터 도착을 확인한 **뒤에만** `sslMtx`를 잡는다.
  블로킹 `SSL_read`가 락을 쥔 채 자면 그 연결로 나가는 송신이 전부 막힌다.

> 참고: `tools/tls_client.hpp`에는 이 규칙이 빠져 있어 `SSL_read`/`SSL_write`가
> 같은 객체에 동시 진입했다. 접속 직후 메시지가 통째로 사라지는 간헐 증상으로
> 나타났고 2026-08-10에 서버와 같은 방식으로 고쳤다.

---

## 4. 연결 모델 — role당 하나

`clients_`는 `map<role, ClientPtr>`다. **같은 role로 새로 접속하면 기존 연결이
끊긴다.**

```cpp
auto it = clients_.find(role);
if (it != clients_.end()) ::shutdown(it->second->fd, SHUT_RDWR);
clients_[role] = c;
```

🔴 **함정**: 테스트 서버를 9000번에 띄우면 현장의 실제 카메라(`.12`)가 CCTV
자리를 빼앗는다. 실제로 겪었다 — 테스트용 CCTV가 조용히 쫓겨나 절반이 실패했다.
**테스트는 다른 포트를 쓸 것** (`./server 9100`).

정리 시 `it->second == c` 비교가 핵심이다. 재접속으로 교체된 옛 세션은 이 조건에
안 걸려서, PEERS가 `false→true`로 깜빡이지 않는다.

**ROBOT만 10초 무수신 타임아웃**이 걸린다 (STATUS 2Hz가 하트비트). QT/CCTV는
무한 대기 — 오래 조용할 수 있는 역할이기 때문.

---

## 5. 좌표계 — 이 시스템의 척추

**규약 다섯 개.** 하나라도 어기면 에러 없이 조용히 틀어진다.

### ① CCTV는 원본 픽셀만 보낸다
변환은 서버 독점. CCTV는 캘리브레이션 데이터를 가질 필요가 없다.
`POS{corners:[[u,v]×4]}` → undistort → `H_marker` → 미터 4점 → 중심·방향.

> 방향은 **변환 후** 좌표로 계산한다. 호모그래피는 각도를 보존하지 않으므로
> 픽셀에서 각도를 재면 틀린다. (`path_planner.hpp` `poseFromPos`)

### ② 단위는 서버 입구에서 미터로 통일
CCTV는 mm 기준 H를 보낸다. `normalizeBundleMmToM()`이 0·1행에 ÷1000을 곱한다.
이후 pose·POSE·BLUEPRINT·PATH·Qt top-view가 전부 미터다.

### ③ 서버 내부는 CCW 양수, 로봇 대면만 뒤집는다
반전 지점은 **`toRobotDeg()` 하나뿐**이다. 호출처는 `turnOp` / `ALIGN` / `DRIFT`
세 곳. 다른 데서 또 뒤집으면 이중 반전이 된다.

### ④ 펜 오프셋(a=0.155m)은 서버가 소유한다
서버가 경로에 `move(±a)` op을 끼워 넣는다. **로봇은 자체 보정을 하지 않는다** —
하면 이중 보정. Qt도 계산하지 않는다.

불변식:
```
노즐 down ⟺ 마커 중심이 꼭짓점보다 진행방향으로 정확히 a 앞
노즐 up   ⟺ 마커 중심이 꼭짓점 위
```
보정 op은 이 두 상태를 오갈 때만 삽입된다. 그래서 도색 move가 연달아 나와도
사이에 불필요한 노즐 up/down이 안 낀다.

### ⑤ 도색 호는 차체가 접선을 향하면 안 된다
ICR(회전 중심)은 좌우 바퀴 축선 위에 있고 그 축선은 마커 중심 C를 지난다.
도면 원 중심 O에 ICR을 놓으려면 C가 O에서 `R_robot`, 펜에서 `a` 떨어져야 한다.

```
R_robot = sqrt(R_paint² − a²)
φ       = atan(a / R_robot)      ← 접선에서 호 안쪽으로 트는 각
```

빼먹으면 반지름은 맞는데 **원이 통째로 다른 자리에 그려진다.** 실측에서
R=0.5m 반원의 펜 도착점이 0.31m 어긋났다(φ=18.06°).

검산: `R_paint = a`면 `R_robot = 0`, `φ = 90°` — 차체가 접선과 직각, 즉 마커
중심이 원 중심에 가만히 있고 펜만 반지름 a로 도는 자명한 상황과 일치한다.

---

## 6. 도색 흐름 — 2단계 자동 진행

```
Qt: BLUEPRINT ──────────▶ 서버 저장만 (로봇 안 움직임) ──▶ BLUEPRINT_OK
Qt: CMD START_DRAW ─────▶ 1단계 접근 경로 생성·전송
                             │
로봇: PATH_DONE(approach) ───┘  Qt에 안 알림 ("그리는 중"이 계속됨)
                             ▼
                          2단계 도색 경로 자동 전송
                             │
로봇: PATH_DONE(draw) ───────┘──▶ Qt에 DRAW_DONE
```

Qt가 중간에 누를 버튼은 없다. **단계 판단의 주인은 서버**이고, 로봇이 보내는
`phase`는 어긋났을 때 WARN을 남기는 용도로만 쓴다.

`pose`가 없으면 `START_DRAW`는 실패가 아니라 **대기**다 — `drawRequested_`를
세워두고 첫 POS가 들어오면 자동으로 이어간다.

---

## 7. 주행 피드백 — 모든 op마다 READY/GO

🔴 **들어온 READY는 반드시 응답 하나를 받고 나가야 한다.** 빠뜨리면 로봇은
영원히 그 자리에 선다.

```
로봇 READY(op k)
   │
   ├─ 판정 대상 아님 ─────────────────────▶ GO
   └─ 판정 대상 ──▶ 대기 창 2초 (POS 표본 수집)
                      │
                      ├─ 표본 0장 ────────▶ GO  (같은 pose로 재판정 금지)
                      ├─ MORE (거리) ─────▶ 로봇 이동 후 같은 k로 READY 재수신
                      ├─ ALIGN (각도) ────▶ 로봇 회전 후 같은 k로 READY 재수신
                      └─ 통과 ────────────▶ GO
```

**MORE를 먼저 소진하고 ALIGN을 돌린다** (한 boundary에 둘 다 걸릴 수 있음).

**대기 창의 기준 시각은 ALIGN/MORE 송신이 아니라 READY 수신 시각**이다. 로봇이
회전하는 동안 흘러간 시간을 대기에 포함시키면 "동작이 끝난 뒤의 장면"을 못 본다.

**각도 평균은 원 위에서** 낸다(sin/cos 누적). 도(度)를 그냥 더하면 ±180° 경계에서
깨지는데, 이 시스템의 heading이 바로 그 근처에 산다.

**표본 0장이면 판정하지 않는다.** pose가 직전과 글자 그대로 같으므로 오차도 같고,
"똑같은 보정이 한 번 더 나가는 것"이 보장된다. v1에서 −34.6° ALIGN이 값까지
동일하게 3번 나가 로봇이 104°를 돈 사례가 있다.

| 피드백 | 언제 | 왜 그때만 |
|---|---|---|
| `MORE` | 직전 op이 role=path인 move/arc | 도착 꼭짓점을 알아야 목표가 생김 |
| `ALIGN` | ① 직전이 path turn ② 도색 시작 직전 ③ **다음이 arc** | ③은 arc가 개루프라 진입각이 원 위치를 100% 결정 |
| `DRIFT` | role=path인 move **실행 중**만 | arc는 개루프(중간에 틀면 원이 깨짐), 오프셋 move는 15.5cm라 보정할 게 없음 |
| `HOLD` | POS 2초 두절 | 서버가 눈 감은 채 로봇이 달리는 것 방지 |

**MORE의 목표에 펜 오프셋이 들어간다**: 노즐이 내려가 있었다면 마커 중심의
목표는 꼭짓점보다 a 앞이다. 이 항을 빼면 도색 구간마다 15.5cm 오차를 잡고 있다고
착각해 매번 `MORE{-0.155}`를 쏜다.

### 이상치 게이트
```
허용 = pose_gate_base_deg(3°) + pose_gate_rate_dps(40°/s) × Δt
```
⚠️ **상한을 두지 말 것.** POS가 1~2Hz인 현장에서는 간격이 5~8초까지 벌어지고
그동안 로봇은 100° 넘게 정당하게 돈다 (실측에서 49° 회전이 5회 연속 폐기되어
재동기까지 갔다). 나눗셈이 없어 Δt→0에서도 안 터진다.

---

## 8. 데이터 저장

```
config/users.json         {id: {salt, hash, calib:{"1":{...},"2":{...}}, cam_ip}}
config/calib_latest.json  전역 캘리 {"1":{...},...}   ← 계정과 분리
config/camera.json        전역 카메라 IP             ← 계정과 분리
config/params.json        튜닝 상수 (선택)
config/stream.json        중계 주소 (선택, gitignore)
```

**읽기 규칙: 계정 값 → 없으면 전역 값.**

전역 슬롯이 있는 이유: 캘리브레이션과 카메라 IP는 **현장의 속성**이지 사용자
속성이 아니다. 예전에는 로그인 사용자에게만 매달려서, 아무도 로그인하지 않은 채
캘리를 올리면 메모리에만 남고 재시작 시 사라졌다.

**구형 포맷 흡수**: 저장된 값이 채널 맵이 아니면 "채널 1의 번들"로 승격시킨다
(`asCalibChannelMap`). 판별은 "키가 전부 숫자 문자열인가"로 한다 — 번들은 키가
`H_floor`/`K`라서 절대 안 걸리고, 레거시는 배열이라 안 걸린다.

---

## 9. 🔴 함정 모음 — 읽어서는 안 보이는 것들

### ① `activeChannel_`은 접속을 넘어 살아남는다
현장 상태지 세션 상태가 아니다. 클라이언트가 나가도 그대로다.
**증상**: CCTV가 다른 채널 POS를 보내면 전부 버려지고 pose가 안 잡힌다. Qt는
아무 에러도 못 받고, 로봇이 화면에서 안 보일 뿐이다. 단서는 서버 WARN 한 줄.

### ② `currentUser_`는 전역 1명
세션별이 아니다. ADMIN이 로그인한 뒤 QT가 다른 계정으로 로그인하면 저장 대상이
바뀐다. 캘리 세션의 "로그인 검사"도 엄밀히는 "저장할 계정이 정해져 있나"에 가깝다.

### ③ `channelOf()`는 잘못된 값을 조용히 1로 바꾼다
POS·H_MATRIX의 하위호환용이다. **`SELECT_CHANNEL`·`CALIB_START`에는 쓰면 안 된다** —
조작자가 CH3를 눌렀는데 로봇이 CH1을 캘리한다. 그쪽은 명시적으로 검증한다.

### ④ `manualMode_` 래치 (2026-08-10 수정됨)
조이스틱 명령에 켜지고 자동 판정(ALIGN/MORE/DRIFT)을 전부 멈춘다.
**해제는 새 BLUEPRINT 또는 START_DRAW 두 곳.** 예전에는 BLUEPRINT뿐이라
"도면 → 위치 조정 → 시작"이라는 자연스러운 순서에서 작업 전체가 개루프로 돌았다.
회귀 확인: `tools/draw_test --nudge`.

### ⑤ 감시자는 감시 대상의 생존에 의존하면 안 된다 (2026-08-10 수정됨)
타임아웃 판정이 `onMessage` 꼬리에서만 돌던 시절, 로봇이 TCP는 붙잡은 채 응답만
멈추면 — 정확히 타임아웃이 필요한 그 경우 — 판정 자체가 안 돌았다. 소켓이 살아
있으니 `onPeerChange`도 안 불린다. 지금은 `Router::tick()`(200ms)이 `sweep()`을
돌린다.

### ⑥ 캘리 세션은 반드시 종결 응답 하나로 닫힌다
`calibActive_`가 켜졌다면 `H_MATRIX` / `CALIB_FAIL` / `CALIB_CANCELLED` 중 하나로만
꺼진다. `clearCalib()`를 단독 호출하면 Qt가 대기 화면에 갇힌다(5분).

### ⑦ `H_MATRIX` 스키마가 세 가지다
중첩 `calib` / 평면(payload 자체가 번들) / 레거시 `H`. 평면 스키마에서는
`outMsg["payload"] = bundle`이 payload를 **통째로 갈아치우므로**, `ch`와
`request_id`를 그 뒤에 다시 심어야 한다.

### ⑧ 취소 ACK ≠ 접수 ACK
`DRAW_ABORTED`는 **접수** ACK다 (로봇에 중계했다는 뜻).
`CALIB_CANCELLED`는 ROBOT·CCTV **양쪽의 실제 정지 확인 후**에만 나간다.
의도적으로 다르다 — 캘리 중 Qt는 전체 화면 대기라, 거기서 "중단됨"을 먼저 띄우면
아직 굴러가는 로봇 쪽으로 사람이 걸어간다.

### ⑨ ARC의 `heading_deg`는 **진입** 접선
2026-08-07 Qt팀과 확정. MOVE/TURN은 진입=출구라 구분이 없지만 호는 스윕만큼
다르다. 한때 서버를 출구 기준으로 바꿨다가(91ec2e2) 되돌린 이력이 있다.

### ⑩ `params.json`은 X-매크로 한 줄로 늘어난다
`RP_PARAM_LIST`에 `X(타입, 이름, 기본값, 설명)` 한 줄 = 구조체 필드 + JSON 로딩 +
시작 로그 덤프 + 오타 검출이 전부 따라온다.

---

## 10. 알려진 미해결

| 항목 | 상태 |
|---|---|
| CCTV가 `CALIB_START` payload를 문자열 검색으로만 처리 | `ch`/`request_id` 소멸 — 카메라 앱 수정 필요 |
| 로봇에 `CALIB_START` 핸들러 없음 | 캘리 주행 자체가 불가 |
| `CALIB_STOPPED` 미구현 (양쪽) | 취소가 항상 `cancel_failed`로 끝남 |
| Qt가 `VerifyNone` + `ignoreSslErrors()` | MITM 노출 |
| 카메라 자격증명이 저장소에 평문 | 교체 권장 |

---

## 11. 회귀 테스트

```bash
cd Server && make && ./server 9100 &          # ⚠️ 9000은 실제 카메라가 붙는다
make calib_session_test && tools/calib_session_test 127.0.0.1 9100   # 21케이스
make calib_channel_test && tools/calib_channel_test                  # 저장 포맷
make sim && tools/robot_sim 127.0.0.1 --port 9100 &                  # 주행 왕복
tools/draw_test 127.0.0.1 --port 9100 --nudge                        # ④ 회귀
```

`calib_session_test`는 끝날 때 활성 채널을 1로 되돌린다 — 안 그러면 이어 붙는
`robot_sim`(항상 CH1 POS)의 관측이 전부 버려져 **피드백 회귀와 구분되지 않는
증상**이 나온다 (실제로 한 번 오진했다).
