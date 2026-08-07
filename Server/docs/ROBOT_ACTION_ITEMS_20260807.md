# 로봇팀 전달 사항 (2026-08-07) — ARC 완주 불가 + ESTOP 오보고

**대조 기준**: 로봇 `feature/server-driven-v2` @ `d3b6ec1` (2026-08-07) 실제 코드
**정본 규격**: `PROTOCOL_v2_ROBOT.md`

> **결론 한 줄**: `d3b6ec1`로 **A-1(`arc` 상태 가드)이 해결된 것을 확인했습니다.**
> 저희가 요청드리려던 것보다 더 꼼꼼합니다 — 감사합니다. 남은 것은 `ESTOP`이 경로를
> 지워서 **ESTOP → RESUME 하면 Qt에 "도색 완료"가 뜨는** 문제(A-2, A-3)입니다.

---

## ⚠️ 번호 표기 안내

이전 요청서 두 건에서 `R-9`~`R-11` 번호가 서로 다른 항목에 중복 사용됐습니다
(`R-11`이 문서에 따라 `ABORT_DRAW`이기도 하고 `arc` 상태 가드이기도 합니다).
혼선을 없애려고 **이 문서는 `A-` 번호를 새로 씁니다.** 이전 `R-` 번호는 인용하지
않겠습니다. 앞으로는 이 문서를 기준으로 봐 주세요.

---

## 0. 요약표

| # | 항목 | 우선도 | 위치 | 한 줄 |
|---|---|---|---|---|
| A-1 | `arc` 상태 가드 어긋남 | ✅ **해결** | — | `d3b6ec1`에서 처리됨 |
| A-2 | `ESTOP`이 경로를 지움 | **P0** | `main.cpp:70~83` | RESUME하면 "도색 완료"가 오보고된다 |
| A-3 | 취소 후 `path_done_sent` | **P0** | `main.cpp:97` | 한 글자. 같은 오보고 |
| A-4 | `STOP`이 `ABORT_DRAW`에 합쳐짐 | P1 | `main.cpp:84` | 조이스틱 떼면 경로가 날아간다 |
| A-5 | `R_robot = 0`에서 0으로 나눗셈 | P1 | `PathFollower.cpp:200` | 최소 반지름 원에서 속도가 미정의 |

---

## ✅ 이미 맞는 것 — 건드리지 말 것

| 항목 | 근거 |
|---|---|
| **`ABORT_DRAW` 정리 항목 6개** | `main.cpp:92~98`. `SetPath({})` / `has_pending_path` / `pending_path` / `waiting_for_go` / `ready_seg_sent` / `ClearLatches()` 전부 서버가 기대하는 그대로입니다 |
| **노즐 1000ms 대기** | `main.cpp:299`. 실측 반영 확인했습니다 |
| **READY 의미 = "이제부터 실행할 op"** | `AdvanceSegment()` 후 다음 루프에서 보내는 구조, 정확합니다 |
| **부호 규약** | `turn` op / `ALIGN` / `DRIFT` / `MORE` 네 경로 모두 서버 CCW ↔ 로봇 "양수=오른쪽" 변환이 맞습니다. 이중 반전 없습니다 |
| **`radius_draw_m` 무시** | 규격대로입니다. 로봇은 `radius_m`만 보면 됩니다 |
| **축간거리 0.166** | `calibration_test.cpp` 실측값 확인했습니다 (§3 참고) |
| **`arc` 상태 가드 (`d3b6ec1`)** | 아래 A-1 참고. 요청드리려던 것보다 처리 범위가 넓습니다 |

---

## A-1. ✅ `arc` op이 영원히 끝나지 않던 문제 — **해결 확인**

`d3b6ec1 fix(rpi-robot): add IsArc() status check and reset logic for arc segment execution`

원인은 `main.cpp`의 `arc` 분기가 `IsMovingStraight()`를 보는데 `StartArc()`는 `is_arc`를
세우는 것이었습니다. 가드가 항상 참이라 20ms마다 `StartArc()`가 재호출되고
`arc_start_l_steps`가 매번 현재값으로 리셋돼 누적 거리가 쌓이지 않았습니다.

### 적용된 수정 — 확인했습니다

| 위치 | 내용 |
|---|---|
| `PathFollower.h` | `IsArc()` 접근자 추가 |
| `main.cpp:327` | `!IsMovingStraight()` → `!IsArc()` |
| `PathFollower.cpp` `SetPath()` | `is_arc = false` 리셋 추가 |
| `main.cpp:139` | 새 PATH 수락 가드에 `&& !IsArc()` 추가 |

**뒤의 두 개는 저희가 요청드리지 않은 것인데, 넣으신 게 맞습니다.**

- `SetPath()`의 `is_arc = false` — 호 도중에 `ABORT_DRAW`가 오면 `is_arc`가 `true`로
  남아, 다음 경로의 첫 `arc`에서 `StartArc()`가 아예 호출되지 않고 **이전 호의 속도·목표
  스텝을 그대로 이어 돕니다.** 리셋이 없으면 이 경로가 살아 있었습니다.
- 새 PATH 수락 가드 — 호 주행 중에 새 경로가 들어오면 중간에 갈아타 버립니다.
  `move`/`turn`만 막고 `arc`는 안 막고 있었습니다.

### 남은 확인 하나

`UpdateArc()`가 완주 시 `is_arc = false`로 내리는 것은 확인했습니다. 다만 **호 완주 판정이
좌우 중 하나라도 목표 스텝에 도달하면 종료**되는 구조입니다:

```cpp
if (static_cast<uint32_t>(dl) >= arc_target_l_steps ||
    static_cast<uint32_t>(dr) >= arc_target_r_steps) {
```

안쪽/바깥쪽 바퀴의 목표 스텝이 크게 다르므로(200mm 호 기준 약 1:4.8), 스텝 카운트에
오차가 있으면 **먼저 도달한 쪽 기준으로 일찍 멈춥니다.** 실주행에서 호가 목표 각도보다
덜 돌면 이 부분을 `&&`로 바꾸거나 바깥쪽 바퀴 기준으로 잡는 것을 검토해 주세요.
지금 지적이 아니라 시험 때 참고하실 사항입니다.

---

## A-2. 🔴 `ESTOP` 분기의 경로 삭제를 되돌려 주세요 (P0)

### 왜 문제인가

`ESTOP`은 **일시정지**입니다. `RESUME`으로 이어서 계속 갈 수 있어야 합니다.
`ABORT_DRAW`(취소)와 달라야 하는데, 지금은 둘이 똑같이 경로를 지웁니다.

`main.cpp:70-83`

```cpp
if (cmd == "ESTOP") {
    robot_comm.SendEmergencyStop(0x01);
    manual_override = true;
    ...
    path_follower.SetPath({});        // ← 경로 삭제
    has_pending_path = false;         // ←
    pending_path.clear();             // ←
    waiting_for_go = false;           // ←
    ready_seg_sent = 0xFFFFFFFF;      // ←
    path_done_sent = false;           // ←
    net_manager.ClearLatches();       // ←
}
```

### 어떤 일이 벌어지나

1. 도색 중 `ESTOP` → 경로가 비워짐
2. `RESUME` → `manual_override = false`
3. 다음 루프에서 `IsPathFinished()`가 **참**이 됩니다 (경로가 비었으니까)
4. `path_done_sent`도 `false`로 리셋돼 있어 `main.cpp:339` 분기에 걸립니다
5. **`PATH_DONE` 전송** → 서버가 도색 완료로 판단 → Qt에 `DRAW_DONE`

실제로는 아무것도 안 그렸는데 "도색 완료"가 뜹니다.

### 수정

`ESTOP` 분기에서 아래 6줄을 **제거**해 주세요. STM32 정지와 노즐 올림만 남깁니다.

```cpp
  if (cmd == "ESTOP") {
      robot_comm.SendEmergencyStop(0x01);
      manual_override = true;
      manual_speed = {0, 0};
      manual_nozzle = 0;
      auto_nozzle = 0;
      robot_comm.SendControlNozzle(0);
-     path_follower.SetPath({});
-     has_pending_path = false;
-     pending_path.clear();
-     waiting_for_go = false;
-     ready_seg_sent = 0xFFFFFFFF;
-     path_done_sent = false;
-     net_manager.ClearLatches();
  }
```

경로를 지우는 것은 `ABORT_DRAW` 분기의 역할입니다 — 그쪽은 지금 그대로 두시면 됩니다.

---

## A-3. 🔴 `path_done_sent = true` 로 바꿔 주세요 (P0)

`main.cpp:97`, `ABORT_DRAW` 분기 안입니다.

```cpp
-  path_done_sent = false;
+  path_done_sent = true;
```

### 왜

`ABORT_DRAW`는 경로를 비웁니다. 그러면 `IsPathFinished()`가 참이 되는데,
`path_done_sent`가 `false`면 A-2와 똑같은 경로로 `PATH_DONE`이 나갑니다.

`true`로 두면 "이 경로에 대한 `PATH_DONE`은 이미 처리됐다"는 뜻이 되어
`main.cpp:339`의 `&& !path_done_sent` 조건에서 걸러집니다.

취소했는데 완료로 보고되는 일이 없어집니다.

---

## A-4. 🟠 `STOP` 을 `ABORT_DRAW` 에서 분리해 주세요 (P1)

`main.cpp:84`

```cpp
} else if (cmd == "ABORT_DRAW" || cmd == "CANCEL_DRAW" || cmd == "STOP") {
```

`STOP`은 ADMIN 조이스틱에서 **버튼을 뗐을 때** 나가는 명령입니다. "속도 0"이라는
뜻이지 "작업 취소"가 아닙니다. 지금은 조이스틱을 한 번 건드렸다 떼면 실행 중이던
도색 경로가 통째로 날아갑니다.

### 수정

```cpp
-  } else if (cmd == "ABORT_DRAW" || cmd == "CANCEL_DRAW" || cmd == "STOP") {
+  } else if (cmd == "ABORT_DRAW" || cmd == "CANCEL_DRAW") {
```

그리고 `STOP`은 수동 속도만 0으로 만드는 분기로 따로 빼 주세요
(`FORWARD`/`BACKWARD` 같은 수동 명령들 옆이 자연스럽습니다):

```cpp
} else if (cmd == "STOP") {
    manual_speed = {0, 0};
    robot_comm.SendSetSpeed(0, 0);
}
```

### 세 명령의 차이

| 명령 | 뜻 | 경로 | 이어서 진행 |
|---|---|---|---|
| `ESTOP` | 비상정지 | **유지** | `RESUME`으로 가능 |
| `STOP` | 속도 0 (조이스틱 뗌) | **유지** | 수동 조작 계속 |
| `ABORT_DRAW` | 작업 취소 | **삭제** | 불가 (새 도면 필요) |

---

## A-5. 🟠 `R_robot = 0` 에서 0으로 나눗셈 (P1)

`PathFollower.cpp:200-201` (`d3b6ec1` 기준)

```cpp
float base_sps = 771.65f;
arc_sps_l = static_cast<int16_t>(base_sps * (r_left / r_robot));
arc_sps_r = static_cast<int16_t>(base_sps * (r_right / r_robot));
```

서버가 지금 최소 도색 반지름을 이론 하한 `0.155m`로 두고 있어, `R_paint = 0.155`인
원이 들어오면 `R_robot = sqrt(0.155² − 0.155²) = 0`을 그대로 보냅니다.

`float`에서 `x/0.0f`는 `±inf`로 정의되지만, **그 `inf`를 `int16_t`로 캐스팅하는 것은
C++ 표준상 정의되지 않은 동작(UB)** 입니다. 어떤 속도가 나갈지 예측할 수 없습니다.

### 수정 방향

`r_robot`이 임계값 이하이면 제자리 회전 전용 분기로 빼 주세요. 제자리 회전은
좌우 바퀴가 같은 크기로 반대 방향이라 비율 계산이 필요 없습니다:

```cpp
const float kMinArcRadius = 1e-3f;
if (std::fabs(r_robot) < kMinArcRadius) {
    arc_sps_l = is_left ? -base_sps : +base_sps;
    arc_sps_r = is_left ? +base_sps : -base_sps;
} else {
    arc_sps_l = static_cast<int16_t>(base_sps * (r_left / r_robot));
    arc_sps_r = static_cast<int16_t>(base_sps * (r_right / r_robot));
}
```

**서버 쪽에서도 막겠습니다.** §3의 답을 주시면 최소 반지름 하한을 올려서
이 값이 애초에 로봇까지 오지 않게 하겠습니다. 다만 방어 코드는 양쪽에 있는 편이
안전합니다.

---

## 1. 답변 감사합니다 — 축간거리 확정

`PROTOCOL_v2_ROBOT.md` §4.4에 200mm 예시를 넣어 주시면서 `W = 0.166`을 쓰신 것으로
답을 대신 받았습니다. 서버를 맞췄습니다:

| 위치 | 이전 | 지금 |
|---|---|---|
| `Server/config/params.json` `wheel_base_m` | `0.167` | **`0.166`** |
| `Server/src/params.hpp` 기본값 | `0.167` | **`0.166`** |

이 값은 서버에서 안쪽 바퀴 역회전 경고 판정에만 쓰이고, 로봇에 보내는 `radius_m`
계산에는 들어가지 않습니다. 로봇 쪽 `wheelbase_m`은 그대로 두시면 됩니다.

---

## 2. 200mm 예시 검산 결과 — 일치합니다

`PROTOCOL_v2_ROBOT.md` §4.4에 넣어주신 계산을 서버 쪽에서 독립적으로 확인했습니다.

| 항목 | 로봇팀 계산 | 서버 확인 |
|---|---|---|
| `R_robot = √(0.200² − 0.155²)` | `0.1264 m` | ✅ 일치 |
| `R_in = 0.1264 − 0.083` | `0.0434 m` | ✅ 일치 |
| `R_out = 0.1264 + 0.083` | `0.2094 m` | ✅ 일치 |
| `SPS_L : SPS_R` | `1 : 4.82` | ✅ 일치 |

---

## 3. 확인 요청 (아직 답을 못 받았습니다)

### 안쪽 바퀴 역회전을 모터 드라이버가 감당하나요?

`R_paint < 0.176m` 이면 `R_robot < W/2` 가 되어 안쪽 바퀴 반지름이 음수 —
**좌우 바퀴가 반대 방향으로 동시 구동**됩니다.

| `R_paint` | `R_robot` | 안쪽바퀴 반지름 (W/2 = 0.083) | 결과 |
|---|---|---|---|
| 0.155 | **0** | −0.083 | A-5의 0으로 나눗셈 (UB) |
| 0.16 | 0.039 | −0.044 | 안쪽바퀴 역회전 |
| **0.176** | **0.083** | **0** | 안쪽바퀴 정지 (제자리 선회) |
| 0.25 | 0.196 | +0.113 | 정상 |

- **역회전 불가** → 서버가 하한을 `√(d² + (W/2)²) ≈ 0.176m` 로 올려 도면 단계에서
  거부하겠습니다 (`DRAW_FAIL{reason:"arc_too_tight"}`)
- **역회전 가능** → 감당 가능한 **최소 `R_robot`** 값을 알려 주시면 그 값에 맞추겠습니다

이 답이 있어야 Qt팀에 "최소 도색 반지름 얼마"를 확정해서 줄 수 있습니다.

---

## 4. 서버 쪽에서 진행 중 — 로봇 작업 아님 (참고)

혼선 방지를 위해 공유합니다. **아래는 서버 코드만 바뀌고, 전선 형식과 로봇 동작은
그대로입니다.**

| 항목 | 내용 |
|---|---|
| **ARC 진입 위상 보정** | 호 진입 시 차체를 접선에서 `φ = atan(d/R_robot)` 만큼 틀어야 원이 도면 자리에 놓입니다. 드라이런에서 **도착점 0.31m 이탈**이 실측됐습니다. 서버가 `turn(±φ)` 오프셋 op을 끼워 넣는 방식으로 처리합니다 |
| `heading_deg` 정의 | Qt→서버 구간 규격이 모호했던 항목입니다. **"호에 진입할 때의 접선"으로 확정**하고 서버가 종료 접선을 역산합니다. 로봇에 나가는 op에는 이 필드가 없어 영향 없습니다 |
| 최소 도색 반지름 | §3 답변에 따라 하한을 올립니다 |

> ⚠️ **A-1을 고치셔도 위 첫 항목이 서버에 들어가기 전까지는 호가 도면과 다른 자리에
> 그려집니다.** 호 도색 통합 시험은 서버 작업 완료 후에 잡는 것이 효율적입니다.
> **직선 경로는 영향 없습니다** — A-1~A-4 수정 후 바로 시험 가능합니다.

---

## 우선순위

| 순서 | 항목 | 이유 |
|---|---|---|
| ~~0~~ | ~~A-1~~ | ✅ `d3b6ec1`에서 해결 |
| 1 | **A-2** | ESTOP → RESUME 이 "완료"로 오보고됩니다. 안전 + 오동작. 6줄 제거 |
| 2 | **A-3** | 한 글자 |
| 3 | §3 답변 | 답만 주시면 서버가 최소 반지름 하한을 확정합니다. Qt팀도 이 값을 기다리는 중입니다 |
| 4 | A-4 | ADMIN 조이스틱 경유만 영향 |
| 5 | A-5 | 서버가 하한을 올리면 당장은 도달하지 않지만, 방어 코드로 필요 |

**A-2 + A-3 은 합쳐서 한 커밋 분량입니다** (6줄 제거 + 1글자 수정).

---

## 다음 시험 계획

A-2·A-3이 들어가면 **직선 경로 통합 시험**은 바로 가능합니다.

**호 도색 시험은 조금 더 기다려 주세요.** 서버의 호 진입 위상 보정(§4)이 방금
들어갔고, Qt 쪽 `heading_deg` 규약 수정이 아직 push되지 않아 부분 원호가 틀어집니다.
양쪽이 맞물리는 대로 다시 알려드리겠습니다. 온전한 원(360°)은 영향이 없어서
**A-1 검증용으로는 지금 돌려보셔도 됩니다.**
