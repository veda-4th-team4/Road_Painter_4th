# 로봇팀 전달 사항 (2026-08-06) — ABORT_DRAW 보완 + ARC 상태 가드

**근거**: `PROTOCOL_v2_ROBOT.md`(정본) ↔ 로봇 커밋 `bce553f` 코드 대조
**대상 브랜치**: `feature/server-driven-v2` @ `bce553f`
**서버 상태**: v2 + v0.4(4채널) `main` `734a529`. ARC 진입 위상 보정은 서버에서 별도 작업 중(아래 §4).

> **결론 한 줄**: `bce553f`로 `ABORT_DRAW`가 들어온 것을 확인했고 **정리 항목이
> 모두 맞습니다.** 다만 같은 커밋에서 `ESTOP`도 경로를 지우게 되면서,
> **ESTOP → RESUME 하면 Qt에 "도색 완료"가 뜨는** 경로가 생겼습니다.
> 그리고 `arc`는 상태 가드가 어긋나 있어 아직 완주하지 못합니다.

---

## 0. 요약표

| # | 항목 | 우선도 | 위치 | 한 줄 |
|---|---|---|---|---|
| R-9 | ESTOP이 경로를 지움 | **P0** | `main.cpp:77~82` | RESUME하면 "도색 완료"가 오보고된다 |
| R-10 | 취소 후 `PATH_DONE` 발신 | **P0** | `main.cpp:97` | 한 글자. ADMIN 취소 시 같은 오보고 |
| R-11 | `arc` 상태 가드 어긋남 | **P0** | `main.cpp:327` 외 4곳 | 호가 영원히 안 끝난다 |
| R-12 | `STOP`이 ABORT에 합쳐짐 | P1 | `main.cpp:84` | 조이스틱 떼면 경로가 날아간다 |

---

## ✅ 이미 맞는 것 — 건드리지 말 것

| 항목 | 근거 |
|---|---|
| **`ABORT_DRAW` 정리 항목 6개** | `main.cpp:92~98`. `SetPath({})` / `pending_path` / `waiting_for_go` / `ready_seg_sent` / `ClearLatches()` 전부 정확합니다. 서버가 기대하는 그대로입니다 |
| **`CalculateMoveSteps` ×1.010 보정** | 예전에 드린 "TURN엔 K_slip이 있는데 MOVE엔 보정이 없다"는 지적이 해결됐습니다 |
| **`CalculateTurnSteps` 기하식 정리** | 하드코딩 상수(22.58044)에서 `turn_circ` 기반 + `1.030`으로 바뀐 것 확인했습니다 |
| **부호 규약 4경로 전부** | `turn` op / `ALIGN` / `DRIFT` / `MORE` 모두 서버 CCW ↔ 로봇 "양수=오른쪽" 변환이 정확합니다. **이중 반전 없습니다** (서버에서 전 경로 추적 검증) |
| **`radius_draw_m` 무시** | 규격대로입니다. 로봇은 `radius_m`만 보면 됩니다 |

---

## R-9. 🔴 ESTOP 분기의 경로 삭제를 되돌려 주세요

**위치**: `src/main.cpp:77~82` (ESTOP 분기 안)

### 왜 문제인가

`ESTOP`은 프로토콜상 **일시정지**입니다. RESUME하면 멈춘 지점부터 이어서 달려야
하는데, 경로를 지우면 이어갈 대상이 없어집니다.

더 중요한 것은 **서버가 ESTOP에서 `planActive_`를 지우지 않는다**는 점입니다
(일시정지이므로 의도된 동작입니다). 그래서 이렇게 됩니다:

```
ESTOP  → 로봇: 경로 삭제, manual_override = true
                (이 시점에는 main.cpp:340 조건이 manual_override에 막혀 PATH_DONE 억제됨)
RESUME → 로봇: manual_override = false            (main.cpp:101)
       → 다음 루프에서 main.cpp:340 조건 성립
         (IsPathFinished() = true, path_done_sent = false)
       → PATH_DONE 전송  🔴
서버   → planActive_ 가 살아 있음
       → clearPath() + Qt에 DRAW_DONE       ("도색 완료"로 화면에 표시됨)
```

접근 단계(`awaitingArrival_`)에서 ESTOP을 눌렀다면 대신 **도색 PATH가 즉시
전송**됩니다. 로봇은 비상정지한 자리에 서 있는데 도색 경로를 받게 됩니다.

### 수정

ESTOP 분기에서 아래 6줄을 **제거**해 주세요.

```cpp
path_follower.SetPath({});      // ← 제거
has_pending_path = false;       // ← 제거
pending_path.clear();           // ← 제거
waiting_for_go = false;         // ← 제거
ready_seg_sent = 0xFFFFFFFF;    // ← 제거
path_done_sent = false;         // ← 제거
```

`SendEmergencyStop()` / `manual_override = true` / 노즐 off / `ClearLatches()` 는
그대로 두시면 됩니다. 밀린 `GO` 래치를 지우는 것은 안전합니다.

---

## R-10. 🔴 `path_done_sent = true` 로 바꿔 주세요

**위치**: `src/main.cpp:97` (ABORT_DRAW 분기 안)

### 왜 문제인가

`SetPath({})` 직후 `IsPathFinished()` 가 true가 되고 `manual_override = false`
이므로, **다음 루프(`main.cpp:340`)에서 곧바로 `PATH_DONE` 이 나갑니다.**

Qt에서 누른 경우는 우연히 안전합니다 — 서버가 자기 상태를 **먼저** 비우고
로봇에 중계하기 때문에(`Server/src/router.cpp:141`) 늦게 온 `PATH_DONE` 이
`[WARN] 진행 중인 경로 없음` 으로 무시됩니다.

**하지만 관리자 창(ADMIN)에서 보내면 터집니다.** 서버의 ADMIN 경로는
`ABORT_DRAW` 를 특별취급 없이 그대로 중계만 하므로
(`Server/src/router.cpp:74`) `planActive_` 가 살아 있고, 로봇의 `PATH_DONE` 이
**"도색 완료"로 해석되어 Qt에 `DRAW_DONE`이 나갑니다.** R-9와 같은 증상입니다.

### 수정

```cpp
path_done_sent = true;   // 취소는 완료가 아니다 - PATH_DONE 을 보내지 않는다
```

한 글자입니다.

---

## R-11. 🔴 `arc` op이 영원히 끝나지 않습니다

### 증상

`op:"arc"` 를 받으면 로봇이 원을 그리기 시작한 뒤 **정지 조건에 도달하지
않습니다.** 같은 원을 계속 돌고 `PATH_DONE` 도 나가지 않습니다.

> ⚠️ 미초기화 문제(아래 ②-1)가 겹쳐 있어, 부팅 시점 메모리 값에 따라
> **"무한 루프"와 "호를 통째로 건너뜀"** 사이에서 비결정적으로 갈립니다.
> 재현이 들쭉날쭉했다면 그 때문입니다.

### ① 원인 — 가드가 다른 상태변수를 봅니다

`src/main.cpp:326`

```cpp
} else if (current_seg.op == "arc") {
    if (!path_follower.IsMovingStraight()) {   // ← arc 중에도 항상 false
        path_follower.StartArc(...);           // ← 20ms마다 재호출됨
    }
    if (path_follower.UpdateArc(l_steps, r_steps, target_speed)) {
```

`StartArc()` 가 세우는 것은 `is_arc` 인데 가드는 `is_moving_straight` 를 봅니다.
arc 실행 중 `is_moving_straight` 는 계속 `false` 이므로 **루프마다
`StartArc()` 가 다시 호출**되고, 그때마다 `arc_start_l_steps` 가 현재 스텝값으로
리셋됩니다. 그래서 `src/PathFollower.cpp:213` 의

```cpp
int32_t dl = std::abs(cur_l_steps - arc_start_l_steps);   // → 항상 ≈ 0
```

가 `arc_target_l_steps` 에 영원히 도달하지 못합니다.

### ② 근본 원인 — `IsArc()` 접근자가 없습니다

`include/PathFollower.h` 의 상태 조회자는 둘뿐입니다.

| 행 | 선언 |
|---|---|
| 64 | `bool IsTurning() const { return is_turning; }` |
| 96 | `bool IsMovingStraight() const { return is_moving_straight; }` |
| 111 | `bool is_arc;` ← **조회자 없음** |

`main.cpp` 가 올바르게 가드할 수단 자체가 없는 상태입니다.

**②-1. 생성자 초기화 리스트에 없습니다** (`PathFollower.cpp:11~20`).
`is_turning(false)`, `is_moving_straight(false)` 는 있는데 `is_arc` 만 빠져
있어 **미초기화 bool** 입니다. 값이 참이 아니면 `UpdateArc()` 가
`if (!is_arc) return true;` 로 **첫 호출에 즉시 "완료"를 반환**해 호를 통째로
건너뜁니다.

**②-2. `SetPath()`(24행)와 `AdvanceSegment()`(41행)가 리셋하지 않습니다.**
둘 다 `is_turning = false; is_moving_straight = false;` 만 합니다(29행, 46행).
새 PATH를 받거나 다음 op으로 넘어가도 `is_arc` 가 `true` 로 남습니다.

### 수정 (5줄)

```cpp
// ① PathFollower.h — 96행 옆에 추가
bool IsArc() const { return is_arc; }

// ② PathFollower.cpp:11 생성자 — 초기화 리스트에 추가
is_arc(false),

// ③ PathFollower.cpp:29 (SetPath 안) — is_moving_straight = false 옆에 추가
is_arc = false;

// ④ PathFollower.cpp:46 (AdvanceSegment 안) — 같은 위치에 추가
is_arc = false;

// ⑤ main.cpp:327 — 가드 교체
if (!path_follower.IsArc()) {
    path_follower.StartArc(...);
}
```

### 참고

- `radius_m` 은 서버가 이미 펜 오프셋을 반영한 **마커 중심 기준 실행값**입니다
  (`R_robot = sqrt(R_paint² − 0.155²)`). 로봇에서 추가 보정하지 마세요
  (`PROTOCOL_v2_ROBOT.md` §5.4 마이그레이션 체크리스트 #1).
- 🟡 `PathFollower.cpp:202` 로그가 이 값을 `R_paint=` 로 찍고 있습니다. 실제로는
  이미 보정된 `R_robot` 이라 현장에서 오독하기 쉽습니다. 라벨만 고쳐 주시면
  좋겠습니다.

---

## R-12. 🟠 `STOP` 을 `ABORT_DRAW` 에서 분리해 주세요

**위치**: `src/main.cpp:84`

```cpp
} else if (cmd == "ABORT_DRAW" || cmd == "CANCEL_DRAW" || cmd == "STOP") {
```

`STOP` 은 프로토콜상 **조이스틱 버튼을 뗀 것**입니다(이동량 없음 —
`Server/src/protocol.hpp:86`). 지금은 버튼에서 손을 떼면 받아둔 경로가 통째로
삭제됩니다.

Qt 경로는 서버가 막아줍니다 — 경로 실행 중에는 수동 CMD를 차단합니다
(`Server/src/router.cpp:164`). **다만 관리자 창(ADMIN)은 차단 대상이 아니라
그대로 통과합니다.**

또 기존 `STOP` 은 `manual_override = true` (정지한 채 수동모드 유지)였는데
지금은 `false` 가 되어 수동모드에서 빠져나옵니다.

### 수정

```cpp
} else if (cmd == "STOP") {
    manual_override = true;
    manual_speed = {0, 0};
    manual_nozzle = 0;
    robot_comm.SendControlNozzle(0);
}
```

`CANCEL_DRAW` 는 프로토콜에 없는 명령이라 서버가 보내지 않습니다. 두셔도
무해하지만 지워도 됩니다.

---

## 세 명령의 차이 (요약)

| | `ESTOP` | `ABORT_DRAW` | `STOP` |
|---|---|---|---|
| 성격 | **일시정지** | **취소** | 조이스틱 뗌 |
| 로봇 경로 | **유지** | **버린다** | 유지 |
| 복귀 | `RESUME` → 이어서 실행 | 새 `START_DRAW` 필요 | 다시 누르면 됨 |
| 서버 `planActive_` | 유지 | `false` | 유지 |
| `PATH_DONE` | **보내면 안 됨** | **보내면 안 됨** | **보내면 안 됨** |

`PATH_DONE` 은 **"받은 PATH의 마지막 op까지 정상 수행"** 일 때만 나갑니다
(`PROTOCOL_v2_ROBOT.md` §2.2). 정지·취소는 완료가 아닙니다.

---

## 4. 서버 쪽에서 진행 중 — 로봇 작업 아님 (참고)

혼선 방지를 위해 공유합니다. **아래는 전부 서버 코드만 고치면 되는 항목이고,
전선 형식과 로봇 동작은 바뀌지 않습니다.**

| 항목 | 내용 |
|---|---|
| **ARC 진입 위상 보정** | `PROTOCOL_v2_ROBOT.md` §11-2. 호 진입 시 차체를 접선에서 `φ = atan(d/R_robot)` 만큼 틀어야 원이 도면 자리에 놓입니다. 드라이런에서 **도착점 0.31m 이탈**이 실측됐습니다. 서버가 `turn(±φ)` op을 오프셋 보정으로 끼워 넣는 방식으로 처리합니다 |
| `heading_deg` 정의 | ARC에서 진입/출구 접선 중 무엇인지 규격이 모호했습니다. **"이 op을 마쳤을 때"(출구)로 통일**하고 서버가 진입 접선을 역산합니다 |
| 최소 도색 반지름 | 현재 이론 하한 `0.155m` 를 그대로 쓰고 있어 `R_robot → 0` 이 됩니다. 실사용 하한으로 올립니다 (아래 확인 요청과 연동) |

> ⚠️ **R-11을 고치셔도 위 첫 항목이 서버에 들어가기 전까지는 호가 도면과 다른
> 자리에 그려집니다.** 호 도색 통합 시험은 서버 작업 완료 후에 잡는 것이
> 효율적입니다. 직선 경로는 영향 없습니다.

---

## 5. 확인 요청 (미해결 항목 리마인드)

`PROTOCOL_v2_ROBOT.md` §11 "남은 항목" 4번에 올려둔 것인데 아직 답을 못 받았습니다.
서버의 최소 반지름 하한을 정하려면 이 답이 필요합니다.

### 안쪽 바퀴 역회전을 모터 드라이버가 감당하나요?

`R_paint < 0.176m` 이면 `R_robot < W/2` 가 되어 안쪽 바퀴 반지름이 음수 —
**좌우 바퀴가 반대 방향으로 동시 구동**됩니다 (§4.4).

| `R_paint` | `R_robot` | 안쪽바퀴 반지름 (W/2 = 0.083) | 결과 |
|---|---|---|---|
| 0.155 | **0** | −0.083 | `PathFollower.cpp:199` 에서 **0으로 나눗셈** → `inf` → `int16_t` 캐스팅 UB |
| 0.16 | 0.039 | −0.044 | 안쪽바퀴 역회전 |
| 0.176 | 0.083 | 0 | 안쪽바퀴 정지 (제자리 선회) |
| 0.25 | 0.196 | +0.113 | 정상 |

- **역회전 불가** → 서버가 하한을 `sqrt(d² + (W/2)²) ≈ 0.176m` 로 올려 도면
  단계에서 거부하겠습니다 (`DRAW_FAIL{reason:"arc_too_tight"}`).
- **역회전 가능** → 감당 가능한 **최소 `R_robot`** 값을 알려 주시면 그 값으로
  하한을 맞추겠습니다.

### 바퀴 축간거리(W)가 서버와 1mm 다릅니다

| 위치 | 값 |
|---|---|
| 서버 `params.hpp` `wheel_base_m` | `0.167` |
| 로봇 `PathFollower.cpp:17` `wheelbase_m` | `0.166` |

어느 쪽이 실측값인지 알려 주시면 서버를 맞추겠습니다.

---

## 우선순위

| 순서 | 항목 | 이유 |
|---|---|---|
| 1 | **R-9** | ESTOP → RESUME 이 "완료"로 오보고됩니다. 안전 + 오동작 |
| 2 | **R-10** | 한 글자. ADMIN 취소 시 같은 오보고 |
| 3 | **R-11** | 곡선 도형을 쓰는 순간 재현. 5줄 |
| 4 | R-12 | ADMIN 경유만 영향. 프로토콜 정합성 |
| 5 | §5 확인 요청 | 답변만 주시면 서버가 처리 |

R-9(6줄 제거) + R-10(1글자) + R-11(5줄) 은 합쳐서 **한 커밋 분량**입니다.
