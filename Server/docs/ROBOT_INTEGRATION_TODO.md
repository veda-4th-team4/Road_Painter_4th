# 로봇 RPi ↔ 중앙 서버 통합 반영 사항 (로봇팀 전달용)

작성: 서버 파트 / 2026-07-21 · 최종 수정: 2026-07-27 (PATH_DONE 신규 - 필수)
대상: `Paint_Robot/RaspberryPi/` (NetworkManager / PathFollower / main) 담당자

> 🔴 **2026-07-27 중요**: 서버가 접근→도색 전환을 **로봇의 `PATH_DONE` 보고로 판단**하도록
> 바뀌었습니다. 이 메시지를 안 보내면 **로봇이 접근 지점에 도착해도 도색이 시작되지
> 않습니다** (서버가 계속 기다림). §4-1 필독.

## 0. 한 줄 요약

로봇의 TLS 접속·STATUS 송신은 이미 잘 붙어 있습니다. 다만 서버가 **v0.3 프로토콜**로
바뀌면서 로봇이 아직 처리하지 않는 신규 메시지(`READY`/`ALIGN`/`GO`/`DRIFT`, PATH의
`phase`/`heading_deg`)가 생겼고, 반대로 **더는 오지 않는 메시지(`POS`)를 파싱하는 죽은
코드**가 남아 있습니다. 이 문서는 "새 스펙"이 아니라 **현재 로봇 코드에서 뭘 고쳐야
하는지 파일·라인으로 짚은 갭 리스트**입니다.

> ✅ **설계 분기 확정 (2026-07-23): 방식 B(현재 서버 구현, v0.3)로 진행합니다.**
> SRS v1.6에 남아있는 "로봇이 POSE(x,y,θ)를 받아 횡방향/헤딩 오차를 직접 계산해
> 추종"하는 구조(방식 A)는 **채택하지 않습니다.** 확정된 방식 B는 **로봇에 좌표를
> 주지 않고 동작 시퀀스(TURN/MOVE) + 각도 피드백(ALIGN/DRIFT)만** 보내는 현재 구현
> 그대로입니다. 아래 3~4번은 이 방식(B) 기준이며 그대로 반영하면 됩니다. SRS
> 문서(v1.6/v1.7 로봇 파트)는 이 결정에 맞춰 추후 정합화 예정 — 로봇팀은 SRS의
> POSE 기반 서술을 무시하고 아래 갭 리스트만 따르면 됩니다.

## 1. 연결 정보 (이미 구현됨 — 참고용)

| 항목 | 값 |
|---|---|
| 주소 | `192.168.0.8:9000` (기본값은 main.cpp의 `192.168.0.8`) |
| 전송 | TCP 위 **TLS 1.2 이상**, `server.crt` 검증 (`SSL_VERIFY_PEER`) |
| 프레이밍 | **JSON Lines** — `{"type":.., "seq":.., "payload":{..}}` + 개행(`\n`) |
| 등록 | 접속 직후 `HELLO {role:"ROBOT"}` 1회 → 서버 `ACK` |
| 하트비트 | **STATUS를 2초 이내 간격으로 계속 송신 필수.** 서버는 10초 무수신 시 세션 종료 |

현재 `NetworkManager.cpp`가 위를 이미 잘 구현하고 있습니다. 아래 2~5번만 반영하면 됩니다.

## 2. 🔴 죽은 코드 제거 — POS 파싱 (서버가 더는 안 보냄)

- **위치**: `NetworkManager.cpp` `parse_incoming_data()`의 `else if (type == "POS")` 분기
- **문제**: v0.3에서 **서버는 로봇에 `POS`를 중계하지 않습니다** (좌표는 서버만 다룸).
  그런데 로봇 코드는 `POS`의 `payload.value("x"/"y"/"theta")`를 파싱해 `latest_pose`에
  넣고 `has_new_pose=true`로 세팅합니다. 서버가 원본 픽셀 `corners`를 보내던 시절엔
  x/y/theta가 없어 **(0,0,0) 가짜 pose**가 만들어졌습니다. 지금은 POS 자체가 안 오므로
  이 분기는 **실행되지 않는 죽은 코드**입니다.
- **할 일**: `POS` 분기 제거. pose 기반 제어를 유지할 거라면 서버와 좌표 전달 방식을
  재협의(위 설계 분기) — 현재 서버는 로봇에 좌표를 주지 않습니다.

## 3. 🔴 테스트 에코 코드 제거

- **위치**: `NetworkManager.cpp` — PATH 수신 시(`[Test Echo]`), POS 수신 시(`[Test Echo]`)
  더미 `SendStatus()` 호출 2곳.
- **문제**: PATH/POS를 받을 때마다 즉시 가짜 STATUS를 되쏩니다. 특히 고주기 메시지가
  오면 그 속도로 가짜 `IDLE` STATUS가 서버·QT로 퍼져 **실제 로봇 상태를 덮습니다.**
- **할 일**: `[Test Echo]` 주석이 달린 즉시 SendStatus 2곳 삭제. STATUS는 아래 main 루프의
  주기 송신(500ms)만 남깁니다.

## 4. 🟠 신규 수신 메시지 처리 (v0.3 핵심)

서버 → 로봇으로 아래가 새로 옵니다. `parse_incoming_data()`에 분기 추가 필요:

| type | payload | 로봇이 할 일 |
|---|---|---|
| `PATH` | `{"phase":"approach"\|"draw", "segments":[...]}` | `phase` 처리 추가. `approach`=시작점 접근 후 **그 자리 대기**(자동 도색 금지), `draw`=**수신 즉시 IMU 현재 방향 0°로 세팅** 후 도색 주행. 세그먼트의 `heading_deg`(MOVE·접근 마지막 TURN에 실림)는 정렬 목표각 |
| `ALIGN` | `{"angle_deg": -2.5}` | 출발 전 미세 회전 보정. `angle_deg`만큼 제자리 회전(양수=좌회전) 후 다시 `READY` 전송 |
| `GO` | `{}` | 정렬 OK. 직진(MOVE) 시작 |
| `DRIFT` | `{"angle_deg": 2.0}` | 직진 중 각도 피드백(~5Hz). 양수=시계방향으로 틀어짐=좌회전 보정량. IMU와 융합해 방향 유지. 정지 중 오는 건 무시 가능 |
| `CMD` | `{"cmd": ...}` | 이미 처리 중(ESTOP/RESUME). **수동 조작** `FORWARD`/`BACKWARD`/`TURN_LEFT`/`TURN_RIGHT`/`STOP` 분기 추가 필요(현재 main은 ESTOP/RESUME만 STM32로 중계) |

### 4-1. 🔴 신규 송신 메시지 — PATH_DONE (필수, 2026-07-27)

로봇 → 서버로 **새로 추가해야 하는** 메시지입니다. 받은 `PATH`의 **마지막 세그먼트까지
수행을 마쳤을 때 1회** 보내세요.

```json
{"type":"PATH_DONE","seq":n,"payload":{"phase":"approach"}}
```

- `phase`: 방금 끝낸 `PATH`의 `phase`를 그대로 되돌려주면 됩니다(`"approach"` 또는 `"draw"`,
  받은 PATH 메시지에 이미 들어있는 값을 그대로 씀).
- **왜 필요한가**: 서버는 더 이상 "접근 완료 → 곧바로 Qt가 누른 버튼으로 도색 시작"이 아니라
  "접근 완료 → 로봇이 `PATH_DONE(approach)`을 보고 → 서버가 곧바로 도색 PATH 전송"으로
  바뀌었습니다. 로봇이 이 보고를 안 하면 서버는 로봇이 아직 접근 중인 줄 알고 계속
  기다립니다 (Qt에는 진행 상황이 없어 사용자도 "먹통"으로 느낌).
- 도색(`phase="draw"`)의 마지막 세그먼트를 끝냈을 때도 동일하게 보내세요 — 서버가 이걸
  받아야 Qt에 "도색 완료"(`DRAW_DONE`)를 알려줍니다.
- ⚠️ **이탈 재계획으로 PATH가 중간에 교체된 경우**: 기존 경로는 폐기되고 새 `PATH`가
  오므로, **그때그때 실제로 받은(마지막) `PATH`를 끝냈을 때** 보내면 됩니다. 폐기된
  경로에 대해서는 보내지 않습니다.
- fire-and-forget입니다 (서버 응답 없음). PathFollower에서 `current_waypoint_idx`가
  마지막 세그먼트를 넘어가는 시점(§5) 바로 그 자리에 넣으면 됩니다.

### 출발 전 정렬 핸드셰이크 (로봇 → 서버 `READY`)

```
로봇: TURN 완료, 정지 상태
로봇 → 서버: {"type":"READY","seq":n,"payload":{"seg":3}}   ← 곧 실행할 MOVE의 index(0부터)
서버: CCTV로 잰 실제 각도 vs 그 MOVE의 heading_deg 비교
  ├─ 오차 > 2°  → ALIGN{angle_deg} (미세 회전 후 다시 READY, 최대 4회)
  └─ 오차 ≤ 2°  → GO{} (직진 시작)
```

- **MOVE를 시작하기 직전마다** 정지 상태로 `READY {"seg":k}` 전송 → `ALIGN` 또는 `GO` 대기.
- 서버가 판정 불가한 상황(pose 없음 등)이면 세워두지 않으려고 그냥 `GO`를 보냅니다.

## 5. 🟠 PathFollower — 세그먼트 실행 완성

- **위치**: `PathFollower.cpp` `Update()`
- **문제**: `current_waypoint_idx`를 증가시키는 코드가 없어 **첫 세그먼트를 무한 실행**합니다.
  `dist_m`/`angle_deg` 목표 도달 판정도 없습니다. 또 pose가 새로 안 오면 매 루프 정지
  명령을 보내는 구조라 v0.3(좌표 미수신)에선 사실상 주행이 안 됩니다.
- **할 일 (v0.3 기준)**: 좌표 대신 **자체 오도메트리(스텝 카운트/IMU)** 로 각 세그먼트의
  목표(거리/각도) 도달을 판정하고 `current_waypoint_idx`를 진행. 방향 유지에는 서버의
  `DRIFT`(직진 중)와 `ALIGN`(출발 전)을 반영. 노즐 ON/OFF는 세그먼트의 `paint` 필드 사용.
  **마지막 세그먼트 완료 판정 시점에 `PATH_DONE`(§4-1)을 반드시 전송할 것.**

## 6. 참고 — STATUS 필드 (현재 구현)

서버가 기대하는 STATUS payload (`NetworkManager::SendStatus`가 이미 생성):

```json
{"type":"STATUS","seq":n,"payload":{"state":"IDLE"|"MOVING"|"ESTOPPED"|"ERROR","painting":true}}
```

- `ERROR` 상태는 프로토콜엔 있으나 로봇 flags 매핑에 없음 — 필요 시 추가.
- SRS 부록 C.3의 상세 필드(left_steps/right_steps/stall_flag/batt_mV 등)는 현재 서버
  구현엔 아직 없습니다. 확장하려면 `Server/PROTOCOL.md`와 함께 갱신해 주세요.

## 7. 체크리스트 (로봇 코드 수정 요약)

- [ ] 설계 분기 확정: POSE 기반 추종 vs 동작 시퀀스 실행 (서버파트와 협의)
- [ ] `POS` 파싱 분기 제거 (죽은 코드 — §2)
- [ ] `[Test Echo]` 즉시 SendStatus 2곳 제거 (§3)
- [ ] `PATH` 분기에 `phase`(approach/draw) + IMU 0° 세팅 처리 (§4)
- [ ] `ALIGN`/`GO`/`DRIFT` 수신 분기 추가 (§4)
- [ ] `READY {seg:k}` 송신 (MOVE 시작 전마다) (§4)
- [ ] 🔴 **`PATH_DONE {phase}` 송신** (경로 마지막 세그먼트 완료 시, §4-1) — **필수, 안 하면 접근→도색 전환이 멈춤**
- [ ] `CMD` 수동 조작(FORWARD/BACKWARD/TURN_LEFT/TURN_RIGHT/STOP) 분기 (§4)
- [ ] PathFollower 세그먼트 도달 판정 + waypoint 진행 + 완료 시 `PATH_DONE` 전송 (§5)
- [ ] STATUS 2초 이내 주기 송신 유지 (하트비트)

문의: 서버 파트. 전체 프로토콜 원문은 `Server/PROTOCOL.md`(로봇 절)와
`Server/src/protocol.hpp` 주석 참고.
