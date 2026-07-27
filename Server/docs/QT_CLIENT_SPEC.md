# Qt 관제 클라이언트 → 중앙 서버 접속 스펙 (Qt팀 전달용)

작성: 서버 파트 / 2026-07-21 · 최종 수정: 2026-07-27 (그리기 흐름 자동화, SET_CAM_IP,
POS 중계 중단, DRAW_DONE 추가)
대상: `Client/` (Qt 관제 UI, Windows 노트북) 담당자

## 0. 한 줄 요약

Qt 클라이언트가 **중앙 서버(TLS 9000)에 `role=QT`로 접속**해서 ① 로그인/캘리브레이션
수신, ② 사용자가 그린 도면(BLUEPRINT) 전송, ③ 로봇 제어 명령(CMD) 전송, ④ 로봇 위치·
상태(POSE/STATUS) 실시간 수신을 하면 됩니다. 서버는 이미 아래 내용을 전부 구현해뒀으니
Qt는 이 문서대로 붙이기만 하면 됩니다. 현재 Client에는 네트워킹 코드가 없어 이 문서가
처음부터의 연동 가이드입니다.

⚠️ **2026-07-27 그리기 흐름이 바뀌었습니다** (이미 이 문서 이전 버전을 반영해 작업 중이면
꼭 확인하세요):
- `BLUEPRINT`는 이제 **저장만** 됩니다. 로봇은 그 즉시 움직이지 않습니다.
- "그림그리기 시작" 버튼(`START_DRAW`)을 누르면 **접근→미세조정→도색→완료까지 서버가
  전부 자동으로 진행**합니다. 중간에 Qt가 더 눌러야 하는 버튼은 없습니다.
- 접근 완료는 Qt에 통지되지 않습니다. 완료는 `DRAW_DONE` 하나로만 옵니다(§4).
- `POS`(CCTV 원본 픽셀)는 더 이상 QT로 오지 않습니다. `POSE`만 쓰세요(§4).
- 카메라 IP를 로그인 후에도 바꿀 수 있는 `SET_CAM_IP`가 신설됐습니다(§3.4).

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
{"type":"REGISTER","seq":1,"payload":{"id":"user1","pw":"...","cam_ip":"192.168.0.31"}}
{"type":"LOGIN","seq":2,"payload":{"id":"user1","pw":"..."}}
```

- `cam_ip`(신규, 2026-07-23): 회원가입 화면에서 같이 입력받는 **CCTV 카메라 IP**. 서버는
  형식 검증 없이 문자열 그대로 저장·회신만 합니다. **RTSP URL 조립(포트·경로 등)은 Qt
  담당**입니다 — 서버는 IP만 알고 있습니다. 생략해도 등록은 됩니다(그 경우 로그인 시
  `cam_ip: null`).

서버 응답:

| 응답 | payload |
|---|---|
| `REGISTER_OK` | `{"id":"user1"}` |
| `REGISTER_FAIL` | `{"reason":"이미 존재하는 id"}` 등 |
| `LOGIN_OK` | `{"id":"user1","calib":{...}\|null,"cam_ip":"192.168.0.31"\|null}` — `calib`은 저장된 캘리브레이션 번들(**`null`이면 캘리브레이션 필요**), `cam_ip`는 등록해둔 카메라 IP(없으면 `null`) |
| `LOGIN_FAIL` | `{"reason":"id 또는 비밀번호 불일치"}` |

- 비밀번호는 서버가 PBKDF2-SHA256 해시로 저장합니다(평문 저장 안 함).
- **로그인 성공 시 `calib` 번들을 받아 top-view 생성에 사용**합니다(아래 §5).
- **로그인 성공 시 `cam_ip`를 받아 CCTV 영상(RTSP)을 띄웁니다.**
- ⚠️ **`calib`가 `null`이면** (아직 캘리브레이션 전) 캘리브레이션이 필요하다는 안내와 함께
  **관리자 창으로 이동하는 하이퍼링크**를 보여주세요 — 주소
  `http://192.168.0.8:8083` (서버 RPi IP + 관리자 창 HTTP 포트, `admin_console/config.sh`
  에서 설정). 이 URL은 서버가 내려주지 않으므로 **Qt 쪽에 고정값으로 둡니다**. 관리자
  창에서 카메라 설치 기사가 캘리브레이션을 완료하면 서버가 `H_MATRIX`를 QT로 즉시
  중계합니다(§4).

### 3.2 BLUEPRINT (도면 전송)

```json
{"type":"BLUEPRINT","seq":4,"payload":{"points":[[0.0,0.0],[2.0,0.0],[2.0,1.0]]}}
```

- `points`: **바닥 평면 미터 좌표** 폴리라인 (그릴 선).
- ⚠️ **Qt가 변환을 마친 값이어야 합니다**: top-view 위 드로잉 픽셀 → `÷ S`(축척 px/m) →
  미터. **서버는 재변환하지 않습니다.** (top-view 위에 그린 점 = 바닥 평면 위의 점이라
  스케일 나눗셈이 전부)
- `points`는 최소 2점. 숫자가 아닌 값이 섞이면 서버가 도면 전체를 무시합니다.
- ⚠️ **서버는 저장만 합니다** (2026-07-27 변경) — 로봇은 이 시점엔 움직이지 않습니다.
  실행은 §3.3의 `START_DRAW`부터 시작됩니다.
- 점 형식이 잘못됐거나 2개 미만이면 `DRAW_FAIL{stage:"plan", reason:"bad_points"}`이 옵니다(§4).

### 3.3 CMD (로봇 제어 / 캘리 / 그리기 시작)

```json
{"type":"CMD","seq":5,"payload":{"cmd":"START_DRAW"}}
```

| cmd | 동작 |
|---|---|
| `START_DRAW` | **"그림그리기 시작" 버튼.** 도면을 올린 뒤 누르면 서버가 **접근→도색→완료까지 전부 자동으로 진행**합니다(2026-07-27 변경 — 이전엔 접근 완료를 기다렸다가 눌러야 했음). 로봇에 중계되지 않음(서버가 대신 PATH를 만들어 보냄). 실패/대기 시 `DRAW_FAIL`(§4) |
| `ESTOP` / `RESUME` | 비상 정지 / 재개. 로봇에 중계 |
| 수동 조작 | `FORWARD` / `BACKWARD` / `TURN_LEFT` / `TURN_RIGHT` / `STOP` — 조이스틱. 버튼 누름=방향, 뗌=`STOP`. 이동량 없음(로봇 고정 속도) |

- ⚠️ **`CALIB_START`(캘리브레이션 시작)는 QT가 보내지 않습니다** (2026-07-23 변경). 카메라
  설치/캘리브레이션은 **관리자 창(admin_console, `http://192.168.0.8:8083`)에서 시작**합니다.
  QT는 캘리 **결과**만 받습니다: `LOGIN_OK.calib`(로그인 시) / `H_MATRIX`(갱신 시) → top-view용.
  (§3.1의 `calib==null` 안내가 관리자 창으로 유도하는 것과 같은 맥락)
- ⚠️ **경로 실행(접근+도색) 중에는 수동 조작 CMD가 서버에서 차단**됩니다(자동 우선, 그림 보호).
  `ESTOP`/`RESUME`는 항상 통과. **현재 거절 응답 메시지는 없고** 서버 로그로만
  확인되므로, QT에선 "버튼이 안 먹는 것처럼" 보일 수 있습니다(정상 동작).
- 경로가 없는 상태에서 수동 조작이 오면 서버는 자동 모드를 끄고 수동 모드로 전환합니다.
  **자동 복귀는 새 `BLUEPRINT` 수신 시.**

### 3.4 SET_CAM_IP (카메라 IP 변경, 신규 2026-07-27)

```json
{"type":"SET_CAM_IP","seq":6,"payload":{"cam_ip":"192.168.0.44"}}
```

- 설정 화면에서 카메라 IP를 바꿀 때 씁니다. **로그인 상태에서만** 동작(현재 로그인된
  사용자 값을 바꿈).
- `REGISTER`의 `cam_ip`와 동일하게 **서버는 형식 검증을 하지 않습니다** — IP 형식 확인은
  Qt 몫입니다. 빈 문자열(`""`)을 보내면 등록을 지웁니다.
- 응답: `SET_CAM_IP_OK {"cam_ip":"192.168.0.44"|null}` | `SET_CAM_IP_FAIL {"reason":"..."}`
  (`"로그인 필요"` 또는 `"저장 실패"`)
- 저장 즉시 파일에 반영되어 다음 로그인부터 `LOGIN_OK.cam_ip`로 새 값이 옵니다.

## 4. 서버 → Qt 메시지 (수신 처리)

`readyRead` 수신 루프에서 아래를 처리하세요. **모르는 type은 조용히 무시**(에러로 끊지 말 것).

| type | payload | 처리 |
|---|---|---|
| `ACK` | `{"msg":"registered as QT"}` | 등록 확인 (무시 가능) |
| `REGISTER_OK/FAIL`, `LOGIN_OK/FAIL` | §3.1 참고 | 로그인 UI 상태 갱신 |
| `SET_CAM_IP_OK/FAIL` | §3.4 참고 | 카메라 IP 설정 화면 결과 표시 |
| `POSE` | `{"x":1.234,"y":0.567,"theta_deg":90.0}` | **top-view 위 로봇 표시용.** x,y=바닥 미터, theta_deg=+x축 기준 반시계 |
| `STATUS` | `{"state":"IDLE"\|"MOVING"\|"ESTOPPED"\|"ERROR","painting":true}` | 대시보드 상태 표시 |
| `H_MATRIX` | `{"calib":{...}}` | 캘리브레이션 갱신 직후 중계. top-view 재생성 |
| `PEERS` | `{"robot":true,"cctv":false}` | **로봇/CCTV 접속 상태.** 상단에 🟢/🔴 표시등 2개 다는 용도로 쓰세요 |
| `DRAW_DONE` | `{}` | **도색 완료** (신규 2026-07-27). "그리는 중" 표시를 여기서 끄면 됩니다. 아래 참고 |
| `DRAW_FAIL` | `{"stage":"plan"\|"draw","reason":"...","msg":"..."}` | **경로 생성/전송 실패 또는 대기 통지**. 아래 표 참고 |

⚠️ **`POS`(CCTV 원본 픽셀)는 더 이상 오지 않습니다** (2026-07-27 변경). 이전 문서를 보고
`POS` 분기를 만들어뒀다면 지워도 됩니다 — 로봇 위치 표시는 `POSE` 하나로 충분합니다.

- `PEERS`는 ROBOT/CCTV가 붙거나 끊길 때마다 옵니다. **Qt 자신이 접속한 직후에도
  현재 상태 스냅샷을 1회 보내주니**, 별도로 물어볼 필요 없이 그냥 수신 루프에서
  받아 표시등만 갱신하면 됩니다.
- ⚠️ STATUS/POSE가 한동안 안 온다고 "로봇/CCTV가 없나 보다"라고 유추하지 마세요
  (예: 로봇이 붙었지만 첫 STATUS 전송 전인 순간도 있습니다). 접속 여부는 반드시
  `PEERS`로만 판단하세요.

**"그리는 중" 상태 표시 (2026-07-27 변경)**: `START_DRAW`를 보낸 시점부터 `DRAW_DONE`이
올 때까지를 한 덩어리로 취급하세요. **로봇이 접근을 마치고 도색으로 넘어가는 순간은
별도로 통지되지 않습니다** (서버가 알아서 이어 보냄) — 세부 진행 상황을 보여주고
싶으면 `POSE`(위치가 목표에 가까워지는지)나 `STATUS`(`painting` 필드)로 유추하세요.

`DRAW_FAIL`의 `reason` 코드:

| stage | reason | 의미 | Qt에서 |
|---|---|---|---|
| `plan` | `bad_points` | BLUEPRINT 좌표 형식 오류/2점 미만 | 도면을 다시 그려서 재전송 안내 |
| `draw` | `no_blueprint` | START_DRAW 시점에 도면이 없음 (신규) | 도면부터 그리라고 안내 |
| `draw` | `busy` | 이미 접근/도색 실행 중인데 START_DRAW를 다시 누름 (신규) | 버튼 비활성화 권장 — 진행 중엔 다시 못 누르게 |
| `draw` | `no_pose` | 로봇 위치 아직 미확인 — **실패 아님**, CCTV POS 오면 서버가 자동으로 시작 | "로봇 위치 확인 중..." 대기 표시 (재전송 불필요, "그리는 중" 유지) |
| `draw` | `robot_offline` | 로봇 미접속이라 경로 전송 실패 | 로봇 연결 확인 안내 |
| `draw` | `not_ready` | 서버 내부 상태 오류 (정상 흐름에선 안 나옴) | 발생하면 서버팀에 로그와 함께 문의 |

- `msg`는 그대로 화면에 띄워도 되는 한글 설명입니다. `reason`으로 로직 분기, `msg`로 표시.
- ⚠️ **`START_DRAW` 버튼은 "그리는 중"엔 비활성화하세요** (연타 방지) — 서버가 `busy`로
  막아주긴 하지만, Qt에서 미리 막아두는 게 사용자 경험이 낫습니다.

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
   {"type":"REGISTER","seq":1,"payload":{"id":"u1","pw":"p1","cam_ip":"192.168.0.31"}}
   {"type":"LOGIN","seq":2,"payload":{"id":"u1","pw":"p1"}}
   # → LOGIN_OK {"id":"u1","calib":null,"cam_ip":"192.168.0.31"}  (아직 캘리 없으면 calib:null)
   {"type":"BLUEPRINT","seq":3,"payload":{"points":[[0,0],[1,0]]}}
   # 저장만 됨 - 여기선 아직 아무 응답도 안 옴 (로봇이 안 움직임)
   {"type":"CMD","seq":4,"payload":{"cmd":"START_DRAW"}}
   # 로봇/CCTV 미접속 상태면 → DRAW_FAIL {"stage":"draw","reason":"no_pose"|"robot_offline",...}
   ```
3. 서버 로그에서 `[접속] QT <ip>`, `REGISTER/LOGIN` 결과, `도면 수신`, `START_DRAW` 결과 확인.
4. 관리자 창(`http://192.168.0.8:8083/logs`)에서 `[tap] QT->SRV ...`가 흐르면 OK.
5. 로봇·CCTV까지 다 붙어 있으면: `POSE`가 오는지(top-view 표시 검증) → 로봇이 접근 후
   도색까지 이어가는지 → 마지막에 `DRAW_DONE`이 오는지 확인.

## 7. 체크리스트 (Qt 클라이언트 구현 요약)

- [ ] `QSslSocket` TLS 클라이언트 (TLS1.2+, `server.crt`)
- [ ] 접속 직후 `HELLO {role:"QT"}` 1회 전송
- [ ] 수신 루프: `\n` 단위로 잘라 JSON 파싱, `type`별 분기, 모르는 type 무시
- [ ] REGISTER 화면에 카메라 IP 입력란 추가 → `cam_ip`로 전송 (§3.1)
- [ ] REGISTER/LOGIN UI + `LOGIN_OK.calib`로 top-view 생성, `LOGIN_OK.cam_ip`로 CCTV 영상 표시 (§5)
- [ ] `calib`가 `null`이면 관리자 창 하이퍼링크(`http://192.168.0.8:8083`, Qt 고정값) 안내 (§3.1)
- [ ] 설정 화면에 카메라 IP 변경란 추가 → `SET_CAM_IP` 전송, `OK/FAIL` 처리 (§3.4)
- [ ] 도면 드로잉 → **미터 좌표 변환 후** `BLUEPRINT` 전송 (÷S) — 이 시점엔 로봇이 안 움직임
- [ ] 제어 UI → `CMD`(START_DRAW / ESTOP / RESUME / 조이스틱) — **CALIB_START는 제외**(관리자 창 담당)
- [ ] "그림그리기 시작" 버튼: 누르면 `START_DRAW` 1회 전송, **`DRAW_DONE`/`DRAW_FAIL`(실패 계열) 받을 때까지 비활성화**
- [ ] `POSE` 수신 → top-view 위 로봇 표시, `STATUS` → 대시보드 (**`POS`는 안 옴 — 수신 분기 불필요**)
- [ ] `H_MATRIX` 수신 → top-view 재생성
- [ ] `PEERS` 수신 → 로봇/CCTV 접속 표시등 갱신 (§4)
- [ ] `DRAW_DONE` 수신 → "그리는 중" 표시 종료 (§4)
- [ ] `DRAW_FAIL` 수신 → `reason`별 안내 문구 표시 (§4)
- [ ] 끊기면 재접속 루프

문의: 서버 파트. 전체 프로토콜 원문은 `Server/PROTOCOL.md`(QT 절)와
`Server/src/protocol.hpp` 주석 참고.
