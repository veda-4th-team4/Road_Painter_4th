# Road-Painter 서버 통신 프로토콜 (v0.2)

> 최종 수정: 2026-07-14 · 구현 기준: `feature/server` 브랜치 (`Server/src/protocol.hpp`와 동일 내용)

## 공통

- **전송**: TCP + TLS, 포트 **9000**
  - 서버만 인증서 제시 (클라이언트는 `certs/server.crt`를 신뢰 CA로 등록해 서버를 검증. 클라이언트 인증서 불필요)
- **프레이밍**: JSON 한 줄 + 개행(`\n`) — JSON Lines
- **공통 형식**: `{"type": "...", "seq": n, "payload": {...}}`
- **접속 절차**: TLS 핸드셰이크 → 첫 메시지로 `HELLO` 전송 (10초 내 미전송 시 서버가 연결 종료) → `ACK` 수신 후 통신 시작

```json
{"type":"HELLO","seq":1,"payload":{"role":"ROBOT"}}
```
- `role`: `"QT"` | `"ROBOT"` | `"CCTV"`
- 서버 응답: `{"type":"ACK","seq":n,"payload":{"msg":"registered as ROBOT"}}`
- 같은 role이 재접속하면 기존 세션은 자동으로 끊고 새 연결로 교체

---

## 로봇 (ROBOT)

### 수신: PATH (서버 → 로봇)

로봇은 좌표를 모르므로 경로는 **동작 명령 시퀀스**로 전달한다.

```json
{"type":"PATH","seq":5,"payload":{
  "segments": [
    {"op":"MOVE","dist_m":2.0,"paint":true},
    {"op":"TURN","angle_deg":-90},
    {"op":"MOVE","dist_m":1.0,"paint":true}
  ]
}}
```

| 필드 | 설명 |
|---|---|
| `op: "MOVE"` | 직진. `dist_m` = 거리(m), `paint` = 도색 여부 (생략 시 `false` = 단순 이동) |
| `op: "TURN"` | 제자리 회전. `angle_deg` **양수 = 좌회전, 음수 = 우회전** |

- **PATH가 오면 진행 중이던 기존 경로를 즉시 폐기하고 새 경로로 교체** (서버의 이탈 감지 → 재계획 대응. 한 TCP 연결이라 순서는 보장됨)

### 수신: CMD (서버 → 로봇)

```json
{"type":"CMD","seq":7,"payload":{"cmd":"ESTOP"}}
```
- `cmd`: `"ESTOP"` | `"RESUME"` | `"CALIB_START"`
- ACK 응답 불필요 (fire-and-forget)

### 송신: STATUS (로봇 → 서버) — 주기 전송 필수

```json
{"type":"STATUS","seq":12,"payload":{
  "state": "MOVING",
  "painting": true
}}
```

| 필드 | 설명 |
|---|---|
| `state` | `"IDLE"` \| `"MOVING"` \| `"ESTOPPED"` \| `"ERROR"` |
| `painting` | 노즐 동작 여부 (지금 도색 중인지) |

- **2초 이내 간격으로 계속 전송 (IDLE 상태에서도)** — 하트비트 겸용
- 서버는 로봇에게서 **10초간 무수신이면 연결 끊김으로 간주하고 세션 종료** (로봇은 재접속 + HELLO 재등록)
- 서버는 STATUS를 QT로 중계함

### ⚠️ v0.1 → v0.2 변경 요약 (로봇팀 반영 필요)

1. PATH가 좌표 배열(`points`) → **동작 시퀀스(`segments`)** 로 변경. 새 PATH가 오면 기존 경로 즉시 폐기
2. STATUS 필드가 `state` + `painting`(노즐 동작 여부)으로 변경 — `x`, `y`, `battery` 제거
3. STATUS **주기 전송이 의무**가 됨 — 안 보내면 10초 후 서버가 연결을 끊음

---

## QT

### 송신: REGISTER / LOGIN (QT → 서버)

```json
{"type":"REGISTER","seq":1,"payload":{"id":"user1","pw":"..."}}
{"type":"LOGIN","seq":2,"payload":{"id":"user1","pw":"..."}}
```

서버 응답:

| 응답 | payload |
|---|---|
| `REGISTER_OK` | `{"id":"user1"}` |
| `REGISTER_FAIL` | `{"reason":"이미 존재하는 id"}` 등 |
| `LOGIN_OK` | `{"id":"user1","H":[[...]]}` — 저장된 호모그래피. **`null`이면 캘리브레이션 필요** |
| `LOGIN_FAIL` | `{"reason":"id 또는 비밀번호 불일치"}` |

### 송신: CMD (QT → 서버)

`{"cmd":"ESTOP"|"RESUME"|"CALIB_START"}` — 서버가 ROBOT에 중계, `CALIB_START`는 CCTV에도 중계

### 송신: BLUEPRINT (QT → 서버)

```json
{"type":"BLUEPRINT","seq":4,"payload":{"points":[[0.0,0.0],[2.0,0.0],[2.0,1.0]]}}
```
- `points`: top-view 미터 좌표의 폴리라인 (그릴 선). **Qt팀과 확정 전 디폴트 포맷**
- 서버 동작: 로봇 위치(CCTV POS)를 알고 있으면 즉시 경로를 생성해 로봇에 PATH 전송,
  모르면 저장해뒀다가 첫 POS 수신 시 전송

### 수신 (서버 → QT)

- `STATUS`: 로봇 상태 중계 (지속 모니터링용)
- `POS`: CCTV 마커 검출 결과 중계
- `H_MATRIX`: 캘리브레이션 직후 새 H행렬 즉시 중계 (top-view 갱신용)

---

## CCTV

### 수신: CMD (서버 → CCTV)

`{"cmd":"CALIB_START"}` — 캘리브레이션 시작

### 송신: H_MATRIX (CCTV → 서버)

```json
{"type":"H_MATRIX","seq":3,"payload":{"H":[[...],[...],[...]]}}
```
- 서버가 로그인된 사용자에 영속 저장하고 QT로 즉시 중계

### 송신: POS (CCTV → 서버)

```json
{"type":"POS","seq":10,"payload":{"corners":[[u1,v1],[u2,v2],[u3,v3],[u4,v4]]}}
```
- `corners`: 로봇 마커 4점 (CCTV 픽셀 좌표), 순서 = **[전좌, 전우, 후우, 후좌]**
  — ⚠️ CCTV팀과 확정 전 디폴트 가정. 테스트용으로 `{"x","y","theta_deg"}`(top-view)도 허용
- 서버 동작:
  1. ROBOT(보정용)과 QT(모니터링용)로 중계
  2. H행렬로 top-view 변환해 로봇 pose(중심·방향) 계산
  3. 계획 경로에서 **0.3 m 초과 이탈** 시 가장 가까운 꼭짓점부터 재계획한 PATH를
     로봇에 재전송 (최소 3초 간격) — 임계값은 러프 디폴트, 현장 튜닝 예정
