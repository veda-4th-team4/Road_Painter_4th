# 로봇팀 전달 사항 (2026-08-05) — 프로토콜 v2 적용

**근거**: `PROTOCOL_v2_ROBOT.md`(정본) ↔ 로봇 커밋 `3a1ea5b` 코드 대조 + v2 서버 구현 완료
**대상 브랜치**: `feature/server-driven-v2`
**서버 상태**: v2 구현 완료(`1886009`). 드라이런에서 접근→도색→피드백→완료 전 과정 검증됨.

> **결론 한 줄**: `3a1ea5b`는 프로토콜 문서보다 **먼저** 올라온 커밋이라
> "MORE 수신 + ARC 자체보정 제거"까지만 반영돼 있다. 지금 v2 서버에 붙이면
> **PATH를 받고도 op을 하나도 실행하지 못한다.** 아래 R-1 ~ R-3이 그 원인이다.

---

## 0. 요약표

| # | 항목 | 우선도 | 위치 | 한 줄 |
|---|---|---|---|---|
| R-1 | `segments` → `ops` | **P0** | `NetworkManager.cpp:347` | 경로가 빈 배열이 된다 |
| R-2 | op 이름 소문자화 | **P0** | `main.cpp:159,166` | 어느 분기에도 안 걸린다 |
| R-3 | `READY{"seg"}` → `{"op_index"}` | **P0** | `NetworkManager.cpp:208` | 서버가 판정을 포기하고 맨 `GO`만 준다 |
| R-4 | READY 의미 = "이제부터 실행할 op" | **P0** | `main.cpp:267,279,289` | 같은 index를 두 번 보내 서버와 어긋난다 |
| R-5 | `nozzle`도 READY→GO를 기다릴 것 | **P1** | `main.cpp:159-165` | 유일하게 핸드셰이크를 건너뛴다 |
| R-6 | `DRIFT`를 실제로 반영할 것 | **P1** | `main.cpp:137-146` | 지금은 로그만 찍고 버린다 |
| R-7 | `HOLD` 구현 | **P1** | (없음) | POS 두절 시 로봇이 안 선다 |
| R-8 | `op_index` 대조 후 불일치는 버릴 것 | **P2** | `NetworkManager.cpp:378-400` | 지연 응답이 새 경로를 움직인다 |
| R-9 | 노즐 대기 500ms → 1000ms | **P2** | `main.cpp:163` | 실측에 맞출 것 |
| R-10 | ARC 로그 문구 정정 | **P3** | `PathFollower.cpp:204` | 동작은 맞고 문구만 오해를 부른다 |
| **R-11** | **`ABORT_DRAW` 구현** | **🔴 P0** | `main.cpp:70-104` (없음) | **[작업 중단] 버튼이 로봇을 못 멈춘다** |

**R-1 ~ R-4만 고치면 일단 움직인다.** R-5 ~ R-7까지 해야 문서대로 동작한다.

> 🔴 **R-11은 2026-08-06 추가분이며, 위 R-1~R-10과 성격이 다르다.**
> 나머지가 "v2로 옮기면서 맞춰야 할 것"이라면 R-11은 **지금 현장에서 이미
> 깨져 있는 안전 결함**이다. QT는 v0.4 기준으로 이미 `ABORT_DRAW`를 보내고
> 있는데 로봇이 그 명령을 모른다.

---

## ✅ 이미 맞는 것 — 건드리지 말 것

`3a1ea5b`에서 잘 처리된 부분이다. 되돌리지 않도록 명시해 둔다.

| 항목 | 근거 |
|---|---|
| **ARC 자체 보정 제거** | `PathFollower.cpp:184` `float r_robot = radius_m;` — 서버가 이미 보정한 값을 그대로 쓴다. 🔴 여기에 `sqrt(R²±d²)`를 되살리면 **이중 보정**이다 (문서 §4.4 #1) |
| **ARC 정지 조건이 바퀴중심 기준** | `r_left/r_right * angle_rad`를 스텝으로 환산 — 펜 기준 호 길이를 쓰지 않는다 (문서 §4.4 #2). v2에는 `dist_m` 필드 자체가 없다 |
| **각도 부호 규약** | `main.cpp:182,273` `robot_turn_deg = -angle_deg` + "positive = turn right (CW)" 주석. 문서 §1.2와 일치 |
| **`role` 필드를 안 읽음** | 관측용 메타데이터다. 여기에 분기가 생기면 그 자체가 버그다 (문서 §4) |
| **`MORE` 수신 처리** | `main.cpp:194-202`. `waiting_for_go` 창에서만 소비하는 것도 맞다 |

---

## R-1. `PATH.ops` (P0)

배열 필드명이 `segments` → **`ops`** 로 바뀌었다.

```cpp
// NetworkManager.cpp:347
auto segments = payload.value("segments", json::array());   // ✗ 항상 빈 배열
auto ops = payload.value("ops", json::array());             // ✓
```

**증상**: `Path update (phase=draw): 0 segments received` 를 찍고 아무것도 안 한다.

각 원소의 필드도 바뀌었다:

| op | 필드 |
|---|---|
| `move` | `dist_m` (음수 = 후진). 🔴 **`paint` 필드는 없다** |
| `turn` | `angle_deg` (양수 = 오른쪽) |
| `nozzle` | `down` (bool) |
| `arc` | `radius_m`, `angle_deg`(크기, 항상 양수), `direction`, `radius_draw_m`(참고용, 무시) |
| 공통 | `op_index` (0부터 연속), `role` (읽지 말 것) |

`heading_deg`는 **더 이상 로봇에게 가지 않는다** — 서버가 판정에 쓰고 로봇에는 안 싣는다.
파싱 코드에서 지워도 되고, 없으면 0이 들어가니 안 써도 무해하다.

---

## R-2. op 이름 소문자 (P0)

```cpp
// main.cpp:159,166
if (current_seg.op == "NOZZLE")                    // ✗
else if (op == "MOVE" || op == "TURN" || op == "ARC")   // ✗

if (current_seg.op == "nozzle")                    // ✓
else if (op == "move" || op == "turn" || op == "arc")   // ✓
```

**증상**: R-1을 고쳐도 어느 분기에도 안 걸려 로봇이 정지한 채로 있는다.

> ⚠️ 대소문자 양쪽을 다 받는 관대한 파싱은 **하지 말 것.** Qt→서버 구간은 아직
> 대문자(`MOVE`/`TURN`)를 쓰므로, 로봇이 둘 다 받으면 "서버가 변환을 안 해도
> 동작하는 것처럼 보이는" 상태가 생겨 변환 버그가 조용히 숨는다.

---

## R-3. `READY{"op_index": n}` (P0)

```cpp
// NetworkManager.cpp:208
json ready_payload = {{"seg", seg_index}};        // ✗
json ready_payload = {{"op_index", seg_index}};   // ✓
```

**증상**: 서버 로그에 이게 찍히고 피드백이 전부 죽는다.

```
[WARN] READY에 op_index 없음 (v1 로봇?) - 그냥 GO
```

서버는 `op_index`가 없으면 ALIGN/MORE 판정을 포기하고 맨 `GO`만 보낸다
(로봇을 영원히 세워두지 않기 위한 방어 동작이지, 호환 동작이 아니다).

---

## R-4. 🔴 READY의 의미가 반대다 (P0)

> **`READY.op_index` = "이제부터 실행하려는" op의 index다. 완료한 op이 아니다.** (문서 §3.1)

현재 코드는 op을 **끝낸 뒤에도** 같은 index로 READY를 보낸다:

```cpp
// main.cpp:265-269  (TURN:279, ARC:289도 동일)
if (path_follower.UpdateMove(...)) {
    net_manager.SendReady(seg_idx);        // ✗ 방금 "끝낸" k를 보냄
    path_follower.AdvanceSegment();
}
```

앞쪽 167줄에서 이미 같은 index로 한 번 보냈으므로 **서버는 `READY{k}`를 두 번 받는다.**
서버는 두 번째를 "k를 아직 실행 안 했구나"로 읽고 boundary k를 다시 판정해 `GO{k}`를
또 준다. 그 사이 로봇은 이미 k+1로 넘어가 있어 상태가 어긋난다.

**고치는 법**: 완료 시점의 `SendReady`를 지우고, `AdvanceSegment()` 후 다음 루프에서
`ready_seg_sent != seg_idx` 조건이 자연히 `READY{k+1}`을 보내게 둔다.

```cpp
if (path_follower.UpdateMove(...)) {
    std::cout << "[MAIN MOVE] op " << seg_idx << " complete." << std::endl;
    path_follower.AdvanceSegment();        // ✓ READY는 다음 루프가 알아서 보낸다
}
```

단 **`ALIGN`/`MORE` 수행을 마친 뒤의 재전송은 그대로 유지**해야 한다
(`main.cpp:219,241`). 그건 "같은 op_index로 READY를 다시"가 맞다.

정리하면 READY를 보내는 경우는 딱 두 가지다:

1. op k를 **실행하기 직전** → `READY{k}`
2. `ALIGN`/`MORE`를 **수행 완료한 직후** → `READY{k}` (같은 k)

그리고 마지막 op을 마친 뒤에는 **`READY{N}`을 보내지 않는다** — `PATH_DONE`이 그 자리를
대신한다 (문서 §3.2). 지금 `path_done_sent` 로직은 맞으니 유지할 것.

---

## R-5. `nozzle`도 READY→GO를 기다릴 것 (P1)

```cpp
// main.cpp:159-165 — 유일하게 핸드셰이크를 건너뛰는 분기
if (current_seg.op == "NOZZLE") {
    robot_comm.SendControlNozzle(auto_nozzle);
    std::this_thread::sleep_for(500ms);
    net_manager.SendReady(seg_idx);      // 실행 후에 보냄 + GO를 안 기다림
    path_follower.AdvanceSegment();
}
```

v2는 **예외 없이 모든 op이 READY→GO**다 (문서 §3). `move`/`turn`/`arc`와 같은
구조로 합칠 것 — `READY{k}` 전송 → `GO{k}` 대기 → 노즐 구동 → 1초 대기 → `AdvanceSegment()`.

서버는 `nozzle` boundary에 **판정 없이 즉시 `GO`**를 주므로 추가 지연은 사실상 없다.
구조를 통일하는 것이 목적이다 (분기가 하나면 R-4 같은 어긋남이 다시 안 생긴다).

---

## R-6. `DRIFT`를 실제로 반영할 것 (P1)

```cpp
// main.cpp:137-146 — 받아서 로그만 찍고 버린다
if (net_manager.GetDriftCorrection(drift_angle)) {
    if (path_follower.IsTurning()) {
        std::cout << "[DRIFT IGNORED] ..." << std::endl;
    } else {
        std::cout << "[DRIFT] ... " << drift_angle << std::endl;   // ← 여기서 끝
    }
}
```

`drift_angle`이 어디에도 쓰이지 않는다. **주행 중 각도 보정이 0인 상태**다.

- 거동은 **연속 조향**으로 확정됐다 (문서 §6.3): 멈추지 않고 좌우 바퀴 속도차로 흡수한다.
  실주행 후 "잠깐 멈추고 → 틀고 → 재출발"로 바꿔도 되고, **서버 동작은 어느 쪽이든 같다**
  (DRIFT는 fire-and-forget이라 서버가 로봇 거동을 알 필요가 없다).
- 부호: **양수 = 오른쪽으로 그만큼 틀어라.** `turn`/`ALIGN`과 같은 규약이다.
  ⚠️ v1 주석("값 = 좌회전으로 보정해야 할 양")은 **부호가 정확히 반대**다.
- `IsTurning()` 무시 로직은 **지워도 된다.** v2 서버는 `role="path"`인 `move`를
  실행 중일 때만 DRIFT를 보낸다 — `turn`/`arc`/오프셋 `move` 중에는 아예 안 나간다.
- `DRIFT`에는 **READY로 응답하지 않는다** (지금도 안 하고 있음 ✅).

---

## R-7. `HOLD` 구현 (P1, 신설)

코드 어디에도 핸들러가 없다. **POS가 끊겨도 로봇이 계속 달린다.**

```json
{"type":"HOLD", "payload":{"hold":true, "reason":"pos_lost"}}
{"type":"HOLD", "payload":{"hold":false}}
```

- `hold=true`: **실행 중인 op 도중이라도 그 자리에서 즉시 정지.**
  op을 포기하지는 않는다 — 남은 거리/각도(`moveRemain`, `turnRemain`, 호 잔여각)를
  그대로 들고 있는다.
- `hold=false`: 멈춘 지점에서 **같은 op을 남은 만큼 이어서** 수행.
- HOLD 중에는 서버가 `GO`/`ALIGN`/`MORE`/`DRIFT`를 **일절 보내지 않는다.**
- 발동 조건: 마지막으로 채택된 `POS`로부터 `pos_lost_ms`(기본 2000ms) 경과.
  해제 조건: `POS`가 연속 `pos_recover_frames`(기본 2)장 채택.

> 이게 필요한 이유: 2026-08-04 실측에서 **도색 구간 `POS`가 0Hz까지 떨어졌다.**
> 서버가 눈을 감은 채로 로봇이 계속 그리는 것보다 서 있는 편이 낫다.

---

## R-8. `op_index` 대조 후 불일치는 버릴 것 (P2)

```cpp
// NetworkManager.cpp:378-400 — GO/ALIGN/MORE/DRIFT 전부 op_index를 안 본다
} else if (type == "ALIGN") {
    float angle = payload.value("angle_deg", 0.0f);   // op_index 미확인
```

> 로봇은 자기가 기다리는 index와 다른 `GO`/`ALIGN`/`MORE`/`DRIFT`는 **조용히 버린다.**
> (문서 §3.1)

지연 도착한 **이전 경로의 응답이 새 경로를 움직이는 것**을 막기 위한 규약이다.
`op_index`는 `PATH` 하나 안에서만 유효하고, 새 `PATH`를 받으면 다시 0부터다.

같은 맥락으로 — **새 `PATH` 수신 시 래치를 전부 비울 것**:
`has_align_cmd`, `has_more_cmd`, `has_drift_cmd`, `go_signal_received`.
(2026-08-03 전달사항의 stale ALIGN 래치 항목과 같은 문제다)

---

## R-9. 노즐 대기 1000ms (P2)

`main.cpp:163`의 `500ms` → **`1000ms`**. 로봇팀 기구학 다이어그램 2·4단계 실측값이다
(문서 §4.3). 액추에이터가 덜 내려간 상태로 출발하면 획 시작부가 비거나 번진다.

꼭짓점 하나당 노즐 op이 2개이므로 실행 시간에 **약 2초**가 붙는다. 실측으로 더 짧게
잡을 수 있으면 알려줄 것 — 총 소요 견적에 직접 들어간다.

---

## R-10. ARC 로그 문구 정정 (P3)

```cpp
// PathFollower.cpp:204 — 동작은 맞는데 문구가 거짓말을 한다
std::cout << "[PathFollower ARC] StartArc: R_paint=" << radius_m
          << "m -> R_robot=" << r_robot << "m | ...";
```

`radius_m`은 서버가 이미 보정을 마친 **`R_robot`** 이다. 지금 로그는 "받은 값이
`R_paint`이고 로봇이 `R_robot`으로 바꿨다"고 읽혀서, 나중에 누가 보고 "보정 코드가
빠졌네" 하고 되살릴 위험이 있다. 아래처럼 바꿀 것.

```cpp
std::cout << "[PathFollower ARC] StartArc: R_robot=" << radius_m
          << "m (서버가 펜 오프셋을 이미 반영한 실행값 - 그대로 사용) | ...";
```

(도면상 반지름 `radius_draw_m`은 `StartArc`까지 안 내려온다. 굳이 같이 찍고 싶으면
`Segment_t`에 필드를 하나 늘려 넘길 것 — 참고용이라 없어도 무방하다.)

---

## R-11. 🔴 `ABORT_DRAW` 구현 (P0, 2026-08-06 추가)

### 지금 무슨 일이 벌어지고 있나

**QT의 [작업 중단] 버튼을 누르면 화면은 "작업 취소 / ESTOPPED"로 바뀌는데
로봇은 계속 도색한다.**

체인이 이렇게 끊겨 있다:

| 단계 | 코드 | 상태 |
|---|---|---|
| QT | `backend.cpp` `cancelJob()` | `ABORT_DRAW` **하나만** 보낸다. `ESTOP`은 **안 보낸다** |
| 서버 | `router.cpp` `fromQt` CMD | ✅ 2026-08-06 구현 완료 (경로 폐기 + 로봇 중계) |
| **로봇** | `main.cpp:70-104` | ❌ **`ABORT_DRAW` 분기가 없다** |

로봇의 CMD 처리는 `if (cmd == "ESTOP") ... else if (cmd == "RESUME") ...` 형태의
문자열 체인이고, **어느 분기에도 안 걸렸을 때의 기본 동작이 없다.** 그래서
`ABORT_DRAW`는 조용히 버려진다.

> ⚠️ QT가 `ESTOP`을 따로 보내지 않는 것은 **의도된 설계**다. 두 개로 나누면
> 순서가 뒤바뀌거나 하나가 누락됐을 때 "섰는데 경로가 살아있는" 어중간한
> 상태가 생긴다. `ABORT_DRAW` 하나가 정지·래치·폐기를 전부 책임진다.
> (`server_PROTOCOL.md` v0.4 절 참고)

### `ESTOP`과 무엇이 다른가

| | `ESTOP` | `ABORT_DRAW` |
|---|---|---|
| 모터·노즐 즉시 정지 | O | O |
| 비상정지 래치 (`RESUME` 필요) | O | O |
| **받아둔 경로** | **유지** — `RESUME`하면 멈춘 op부터 이어서 달림 | **폐기** — `RESUME`해도 아무 데도 안 감 |

즉 **`ESTOP`은 일시정지, `ABORT_DRAW`는 취소**다.

### 구현 요청

`main.cpp`의 CMD 체인에 분기 하나를 추가하면 된다. `ESTOP`이 하는 일을 전부
하고, 거기에 **경로 버퍼 비우기**를 더한다:

```cpp
} else if (cmd == "ABORT_DRAW") {
    // 1) ESTOP과 동일: 모터 정지 + 노즐 올림 + 비상정지 래치
    robot_comm.SendEmergencyStop(0x01);
    manual_override = true;
    manual_speed = {0, 0};
    manual_nozzle = 0;
    robot_comm.SendControlNozzle(0);

    // 2) 여기부터가 ESTOP과 다른 부분 — 받아둔 경로를 버린다.
    //    RESUME 후에 자동 주행이 되살아나면 안 된다.
    path_follower.ClearPath();     // ⚠️ 아래 참고 - 현재 없는 함수다
    has_pending_path = false;      // 아직 적용 안 한 PATH도 버린다
    waiting_for_go = false;
    ready_seg_sent = 0xFFFFFFFF;
    path_done_sent = true;         // 3) PATH_DONE을 보내지 않기 위해
    net_manager.ClearLatches();    // 남아있는 ALIGN/MORE/GO/DRIFT 래치 제거
}
```

**세 가지만 지켜주면 된다:**

1. **경로 버퍼를 비운다** — op 커서, `PATH_DONE` 전송 대기, 노즐 오프셋
   서브시퀀스 상태까지 전부 초기화
2. **`PATH_DONE`을 보내지 않는다** — 끝낸 게 아니라 버린 것이다.
   보내면 서버가 "접근 완료"로 오해해 곧바로 도색 경로를 내려보낸다
3. **`ClearLatches()`를 반드시 부른다** — 이미 R-8에서 만들어 둔 함수다
   (`NetworkManager.cpp:264`). 취소 직전에 도착해 있던 `GO`가 래치에 남아
   있으면, 취소 직후 그게 소비되면서 로봇이 한 op을 더 실행한다

### ⚠️ `PathFollower::ClearPath()`가 지금 없다

`PathFollower.h`에 `SetPath()`와 `IsPathFinished()`는 있지만 **경로를 비우는
함수가 없다.** 추가가 필요하다:

```cpp
// PathFollower.h
void ClearPath();

// PathFollower.cpp
void PathFollower::ClearPath() {
    path.clear();
    current_waypoint_idx = 0;
    drift_offset_deg = 0.0f;
    is_turning = false;
    is_moving_straight = false;
    is_arc = false;              // v2에서 추가된 상태 - 같이 비울 것
}
```

> 참고: `feature/pnm` 브랜치에 같은 이름의 함수가 이미 있었다
> (`PathFollower.h:32`). v2로 넘어오면서 빠진 것이라, 그 구현을 그대로 가져다
> `is_arc`만 추가하면 된다.

### 다음 작업은 어떻게 시작되나

서버가 **새 `PATH`를 보내는 것**으로 시작한다. 로봇은 평소처럼 새 경로로
교체하면 된다 — `ABORT_DRAW` 이후 로봇이 먼저 뭔가를 보낼 필요는 없다.

### 검증 방법

로봇 없이도 확인할 수 있다 (부록 B의 `robot_sim` 사용):

1. `START_DRAW`로 도색을 시작시킨다
2. 도색 중간에 QT에서 [작업 중단]을 누른다
3. **기대**: 로봇 로그에 `ABORT_DRAW` 수신이 찍히고 모터가 선다.
   이후 `RESUME`을 해도 **아무 데도 가지 않는다**
4. **기대**: 서버 로그에 `[INFO] CMD ABORT_DRAW -> ROBOT (실행 중이던 경로 폐기)`
5. **기대**: 다시 [그림그리기 시작]을 누르면 `DRAW_FAIL{busy}` 없이 처음부터 진행된다

---

## 부록 A. 서버가 보내는 실제 경로 예시

도면 `points = [[0,0],[2,0],[2,1.5]]`, 전 구간 도색일 때 서버가 실제로 만드는
`PATH{phase:"draw"}.ops`다 (v2 서버 드라이런 실측 출력).

```json
[{"op":"move",  "role":"offset","dist_m":0.155, "op_index":0},
 {"op":"nozzle","role":"offset","down":true,    "op_index":1},
 {"op":"move",  "role":"path",  "dist_m":2.0,   "op_index":2},
 {"op":"nozzle","role":"offset","down":false,   "op_index":3},
 {"op":"move",  "role":"offset","dist_m":-0.155,"op_index":4},
 {"op":"turn",  "role":"path",  "angle_deg":-90.0,"op_index":5},
 {"op":"move",  "role":"offset","dist_m":0.155, "op_index":6},
 {"op":"nozzle","role":"offset","down":true,    "op_index":7},
 {"op":"move",  "role":"path",  "dist_m":1.5,   "op_index":8},
 {"op":"nozzle","role":"offset","down":false,   "op_index":9},
 {"op":"move",  "role":"offset","dist_m":-0.155,"op_index":10}]
```

- `±0.155` 오프셋 보정은 **서버가 넣은 것**이다. 로봇은 시킨 대로 가기만 하면 된다.
- `turn`의 `-90.0`은 "왼쪽 90도"다 (양수 = 오른쪽).
- `role`은 보면 안 된다. 위 경로에서 `offset`을 특별 취급하는 순간 깨진다.

서버가 이 경로에서 주는 응답:

| READY | 직전 op | 서버 응답 |
|---|---|---|
| `{0}` | (경로 시작) | 2초 대기 → **ALIGN** ×≤6 → `GO{0}` |
| `{1}`,`{2}` | offset move / nozzle | **즉시 `GO`** |
| — | *op 2 주행 중* | **DRIFT** (≤2.5Hz) |
| `{3}` | path move | 2초 대기 → **MORE** ×≤4 → `GO{3}` |
| `{4}`,`{5}` | nozzle / offset move | **즉시 `GO`** |
| `{6}` | path turn | 2초 대기 → **ALIGN** ×≤6 → `GO{6}` |
| … | | |
| — | op 10 실행 후 | `READY{11}` 대신 **`PATH_DONE{"draw"}`** |

---

## 부록 B. 로봇 없이 검증하는 법

서버 쪽에 v2 시뮬레이터가 있다. **프로토콜대로 동작하는 이상적인 로봇**이라,
여기 로그와 실기 로그를 나란히 놓으면 어디서 어긋나는지 바로 보인다.

```bash
cd Server && make server sim
./server 9000                                          # 1번 창
./tools/robot_sim 127.0.0.1 --start -0.5,-0.5,0        # 2번 창 (ROBOT+CCTV 대역)
./tools/draw_test 127.0.0.1                            # 3번 창 (QT 대역)
```

오차를 주입해 피드백을 태워볼 수도 있다:

```bash
./tools/robot_sim 127.0.0.1 --drift-dps 4 --slip 0.08 --pos-drop 30,4
```

`--drift-dps` 조향오차(°/s) · `--slip` 거리오차(비율) · `--pos-drop <s,d>` POS 두절(HOLD 검증).
`tools/robot_sim.cpp`가 **R-4 ~ R-8을 전부 지킨 참조 구현**이니 그대로 베껴도 된다.

---

## 부록 C. 🔴 호(arc) 도색은 아직 쓰지 말 것

**로봇 잘못이 아니다. 서버/문서 쪽 미해결 항목이다** (`PROTOCOL_v2_ROBOT.md` §11-2).

v2 서버 드라이런에서 반지름 0.5m 반원을 실제로 그려본 결과:

| 항목 | 결과 |
|---|---|
| 서버가 보낸 `radius_m` | `0.4754` ✅ 공식대로 |
| 펜이 그린 호의 반지름 | `0.4996` ✅ **도면 0.5m와 일치 — 반지름 공식은 옳다** |
| 펜 도착점 | `(0.310, −0.950)` — 도면 `(0, −1.0)` 대비 **0.31m 어긋남** ❌ |

반지름은 맞는데 **원의 위치가 틀린다.** 직선용으로 만든 `move(+0.155)` 진입 규칙을
호에 쓰면 ICR이 도면 원의 중심에 안 놓여서, 원 전체가 `atan(d/R_robot)`(0.5m에서 18.06°)
만큼 회전한 자리에 그려진다. 해법(진입 전 `turn(∓φ)` 추가)은 **서버만 고치면 되고
전선 형식·로봇 동작은 그대로**다.

**로봇팀은 R-1 ~ R-10만 처리하면 된다.** 호 관련해서 지금 확실한 것:

- 받은 `radius_m`을 **그대로** 쓸 것 (자체 보정 금지)
- 정지 조건은 `radius_m × θ_rad` (바퀴중심 기준)
- 서버는 `arc` 주행 중 `DRIFT`를 보내지 않는다 — 개루프 구간이다
- `radius_m < W/2 ≈ 0.0835m`면 안쪽 바퀴가 **역회전**한다.
  🔴 **모터 드라이버가 좌우 반대 방향 동시 구동을 처리하는지 확인해 알려줄 것** —
  이게 실사용 최소 반지름의 하한을 먼저 결정할 수 있다.

---

## 확인 요청 사항 (로봇팀 → 서버팀)

| # | 질문 | 왜 필요한가 |
|---|---|---|
| 1 | 노즐 액추에이터 실제 동작 시간 | R-9. 총 소요 시간 견적에 직접 들어간다 |
| 2 | 모터 드라이버 좌우 역방향 동시 구동 가능 여부 | 호 최소 반지름의 하한을 결정한다 |
| 3 | 미세회전 오버슛 실측 (현재 1틱 ≈ 1.42° 추정) | 줄일 수 있으면 `align_threshold_deg`를 4°에서 내린다 |
| 4 | `PathFollower.h`의 `NOZZLE_OFFSET_M` 현재 값 | 서버 `pen_offset_m`(0.155)과 **반드시 같아야 한다.** 자동 동기화되지 않는다 |
| 5 | **R-11 반영 예정 시점** | 그때까지 [작업 중단] 버튼이 로봇을 못 세운다. 현장 시연 전에는 반드시 들어가야 한다 |

서버 쪽 임계값·주기는 전부 `Server/config/params.json`에 있고 **서버 재시작만으로 바뀐다**
(재컴파일 불필요). 현장에서 조정이 필요하면 값만 알려주면 된다.
