# Qt 관제 클라이언트 → 중앙 서버 접속 스펙 (Qt팀 전달용)

작성: 서버 파트 / 2026-07-21
대상: `Client/` (Qt 관제 UI, Windows 노트북) 담당자

## 0. 한 줄 요약

Qt 클라이언트가 **중앙 서버(TLS 9000)에 `role=QT`로 접속**해서 ① 로그인/캘리브레이션
수신, ② 사용자가 그린 도면(BLUEPRINT) 전송, ③ 로봇 제어 명령(CMD) 전송, ④ 로봇 위치·
상태(POSE/STATUS) 실시간 수신을 하면 됩니다. 서버는 이미 QT 접속을 받도록 구현돼 있어
**서버 코드 수정은 0줄**입니다. 현재 Client에는 네트워킹 코드가 없어 이 문서가 처음부터의
연동 가이드입니다.

```
Qt ──① 로그인/도면/제어 (TLS) ──▶ 서버 192.168.0.8:9000
   ◀── ② POSE/STATUS/H_MATRIX 수신 ──
카메라 ── RTSP 영상 ────────────▶ Qt (측위와 별개 채널, 기존대로)
```

## 1. 연결 정보

| 항목 | 값 |
|---|---|
| 주소 | `192.168.0.8:9000` (서버 RPi, 공유기 LAN) |
| 전송 | TCP 위 **TLS 1.2 이상** (평문 TCP 아님) |
| 서버 인증서 | 자가서명. `Server/certs/server.crt` 파일을 드립니다 |
| 인증서 검증 | 권장: `server.crt`를 신뢰 CA로 추가 + `VERIFY_PEER`. SAN에 `IP:192.168.0.8` 포함. 1차 연동은 검증 생략으로 시작해도 됨 |
| 클라이언트 인증서 | 불필요 |
| 메시지 프레이밍 | **JSON Lines** — JSON 1개 + 개행(`\n`)이 1메시지. 모든 메시지는 `{"type":..., "seq":..., "payload":{...}}` |
| seq | 연결마다 0부터 증가하는 정수. 로그 추적용 (서버가 검사하진 않음) |

참고 구현: 로봇 RPi가 같은 방식(OpenSSL, TLS1.2+, server.crt 검증, JSON Lines)으로 이미
붙어 있습니다 — `Paint_Robot/RaspberryPi/src/NetworkManager.cpp`. Qt는 `QSslSocket`으로
동일하게 구현하면 됩니다(`readyRead`에서 `\n` 단위로 잘라 JSON 파싱).

## 2. 접속 직후 — HELLO (필수, 1회)

```json
{"type":"HELLO","seq":0,"payload":{"role":"QT"}}
```

- 접속 후 **10초 안에** 보내야 합니다 (안 보내면 서버가 끊음).
- 서버 응답: `{"type":"ACK","seq":n,"payload":{"msg":"registered as QT"}}`
- 같은 role로 재접속하면 서버가 기존 연결을 끊고 새 연결로 교체합니다.

## 3. Qt → 서버 메시지

### 3.1 REGISTER / LOGIN (사용자 인증)

```json
{"type":"REGISTER","seq":1,"payload":{"id":"user1","pw":"..."}}
{"type":"LOGIN","seq":2,"payload":{"id":"user1","pw":"..."}}
```

서버 응답:

| 응답 | payload |
|---|---|
| `REGISTER_OK` | `{"id":"user1"}` |
| `REGISTER_FAIL` | `{"reason":"이미 존재하는 id"}` 등 |
| `LOGIN_OK` | `{"id":"user1","calib":{...}}` — 저장된 캘리브레이션 번들. **`null`이면 캘리브레이션 필요** |
| `LOGIN_FAIL` | `{"reason":"id 또는 비밀번호 불일치"}` |

- 비밀번호는 서버가 PBKDF2-SHA256 해시로 저장합니다(평문 저장 안 함).
- **로그인 성공 시 `calib` 번들을 받아 top-view 생성에 사용**합니다(아래 §5).

### 3.2 BLUEPRINT (도면 전송)

```json
{"type":"BLUEPRINT","seq":4,"payload":{"points":[[0.0,0.0],[2.0,0.0],[2.0,1.0]]}}
```

- `points`: **바닥 평면 미터 좌표** 폴리라인 (그릴 선).
- ⚠️ **Qt가 변환을 마친 값이어야 합니다**: top-view 위 드로잉 픽셀 → `÷ S`(축척 px/m) →
  미터. **서버는 재변환하지 않습니다.** (top-view 위에 그린 점 = 바닥 평면 위의 점이라
  스케일 나눗셈이 전부)
- `points`는 최소 2점. 숫자가 아닌 값이 섞이면 서버가 도면 전체를 무시합니다.
- 서버 동작: 로봇 위치(CCTV POS)를 알고 있으면 즉시 1단계(접근) 경로를 생성해 로봇에
  PATH 전송, 모르면 저장해뒀다가 첫 POS 수신 시 전송.

### 3.3 CMD (로봇 제어 / 캘리 / 그리기 시작)

```json
{"type":"CMD","seq":5,"payload":{"cmd":"START_DRAW"}}
```

| cmd | 동작 |
|---|---|
| `START_DRAW` | **"그림그리기 시작" 버튼.** 로봇이 접근(1단계)을 마치고 대기 중일 때 누르면 서버가 도색 경로(PATH phase=draw)를 로봇에 전송. 로봇에 중계되지 않음(PATH 수신이 곧 시작 신호). 접근 미완료면 서버가 무시 |
| `ESTOP` / `RESUME` | 비상 정지 / 재개. 로봇에 중계 |
| `CALIB_START` | 캘리브레이션 시작. 로봇+CCTV에 중계 |
| 수동 조작 | `FORWARD` / `BACKWARD` / `TURN_LEFT` / `TURN_RIGHT` / `STOP` — 조이스틱. 버튼 누름=방향, 뗌=`STOP`. 이동량 없음(로봇 고정 속도) |

- ⚠️ **경로 실행(도색) 중에는 수동 조작 CMD가 서버에서 차단**됩니다(자동 우선, 그림 보호).
  `ESTOP`/`RESUME`/`CALIB_START`는 항상 통과. **현재 거절 응답 메시지는 없고** 서버 로그로만
  확인되므로, QT에선 "버튼이 안 먹는 것처럼" 보일 수 있습니다(정상 동작).
- 경로가 없는 상태에서 수동 조작이 오면 서버는 자동 모드를 끄고 수동 모드로 전환합니다.
  **자동 복귀는 새 `BLUEPRINT` 수신 시.**

## 4. 서버 → Qt 메시지 (수신 처리)

`readyRead` 수신 루프에서 아래를 처리하세요. **모르는 type은 조용히 무시**(에러로 끊지 말 것).

| type | payload | 처리 |
|---|---|---|
| `ACK` | `{"msg":"registered as QT"}` | 등록 확인 (무시 가능) |
| `REGISTER_OK/FAIL`, `LOGIN_OK/FAIL` | §3.1 참고 | 로그인 UI 상태 갱신 |
| `POSE` | `{"x":1.234,"y":0.567,"theta_deg":90.0}` | **top-view 위 로봇 표시용은 이걸 사용.** x,y=바닥 미터, theta_deg=+x축 기준 반시계 |
| `STATUS` | `{"state":"IDLE"\|"MOVING"\|"ESTOPPED"\|"ERROR","painting":true}` | 대시보드 상태 표시 |
| `POS` | `{"corners":[[u,v]x4]}` | CCTV 원본 픽셀 중계(모니터링용). 로봇 표시는 POSE를 쓰는 게 정확 |
| `H_MATRIX` | `{"calib":{...}}` | 캘리브레이션 갱신 직후 중계. top-view 재생성 |

## 5. top-view 생성 (calib 번들 사용)

`LOGIN_OK` 또는 `H_MATRIX`로 받는 `calib` 번들:

```json
{"version":1, "K":[[fx,0,cx],[0,fy,cy],[0,0,1]], "D":[k1,k2,p1,p2,k3],
 "H_floor":[[..3x3..]], "H_marker":[[..3x3..]], "marker_height_m":0.25}
```

- Qt는 **`H_floor`(+ `K`,`D`)로 top-view를 생성**합니다: 프레임 왜곡 보정(`undistort`) →
  `warpPerspective(S·H_floor)` (S = 렌더링 축척 px/m).
- `H_floor` = 왜곡 보정된 픽셀 → **바닥 평면** 미터. (로봇 측위용 `H_marker`와 구분 —
  도면·표시는 H_floor, 로봇 pose는 서버가 H_marker로 계산)
- `calib`가 `null`이면 아직 캘리브레이션 전 → "캘리브레이션 필요" 안내.
- 레거시 `{"H":[[..3x3..]]}` 단일 행렬도 허용(왜곡·시차 보정 없이 동작, 데모 전용).

## 6. 연동 테스트 절차

1. 서버 실행 확인 (서버 RPi): `cd ~/Road_Painter_4th/Server && ./server`
2. 손 테스트 — Qt 개발 PC에서:
   ```bash
   openssl s_client -connect 192.168.0.8:9000 -quiet
   # 붙은 뒤 아래를 한 줄씩(개행 포함) 입력:
   {"type":"HELLO","seq":0,"payload":{"role":"QT"}}
   # → ACK "registered as QT"
   {"type":"REGISTER","seq":1,"payload":{"id":"u1","pw":"p1"}}
   {"type":"LOGIN","seq":2,"payload":{"id":"u1","pw":"p1"}}
   # → LOGIN_OK {"id":"u1","calib":null}  (아직 캘리 없으면 null)
   {"type":"BLUEPRINT","seq":3,"payload":{"points":[[0,0],[1,0]]}}
   ```
3. 서버 로그에서 `[접속] QT <ip>`, `REGISTER/LOGIN` 결과, `도면 수신` 확인.
4. 관리자 창(`http://192.168.0.8:8081/logs`)에서 `[tap] QT->SRV ...`가 흐르면 OK.
5. CCTV가 붙어 POS가 흐르면 QT로 `POSE`가 오는지 확인(top-view 표시 검증).

## 7. 체크리스트 (Qt 클라이언트 구현 요약)

- [ ] `QSslSocket` TLS 클라이언트 (TLS1.2+, `server.crt`)
- [ ] 접속 직후 `HELLO {role:"QT"}` 1회 전송
- [ ] 수신 루프: `\n` 단위로 잘라 JSON 파싱, `type`별 분기, 모르는 type 무시
- [ ] REGISTER/LOGIN UI + `LOGIN_OK.calib`로 top-view 생성 (§5)
- [ ] 도면 드로잉 → **미터 좌표 변환 후** `BLUEPRINT` 전송 (÷S)
- [ ] 제어 UI → `CMD`(START_DRAW / ESTOP / RESUME / CALIB_START / 조이스틱)
- [ ] `POSE` 수신 → top-view 위 로봇 표시, `STATUS` → 대시보드
- [ ] `H_MATRIX` 수신 → top-view 재생성
- [ ] 끊기면 재접속 루프

문의: 서버 파트. 전체 프로토콜 원문은 `Server/PROTOCOL.md`(QT 절)와
`Server/src/protocol.hpp` 주석 참고.
