# 로봇팀 요청 — 오도메트리 캘리 관련 확인 2건

- 작성일: 2026-08-12
- 수신: 로봇팀 (Paint_Robot)
- 발신: 서버팀
- 대상 커밋: `6f145b7`(calib_path_test 추가), `01dd855`(IMU 폐루프 회전)
- 전체 설계: [`ROBOT_ODOMETRY_HOMOGRAPHY.md`](ROBOT_ODOMETRY_HOMOGRAPHY.md)

---

## 0. 요약

`feature/server-driven-v2`에 올려주신 두 커밋을 서버 구현과 대조했습니다.
**11-op 구조 / 반쪽 구간 / boundary 스킵 / 2.5초 정착 / 폐합오차 출력이 전부
서버와 일치합니다.**

다만 두 가지가 걸려서 확인 부탁드립니다.

| # | 항목 | 성격 |
|---|---|---|
| 1 | `calib_path_test`의 회전 부호 반전 누락 | 코드 |
| 2 | IMU 폐루프가 서버 주행 경로에서는 꺼져 있음 | 확인 |

### 🔴 핵심: 도구와 실제 서버 주행이 두 군데에서 다르게 동작합니다

| | `calib_path_test` | `main.cpp` (서버 주행) |
|---|---|---|
| 회전 부호 반전 | **없음** (`seg.angle_deg` 그대로) | 있음 (`-angle_deg`) |
| IMU 폐루프 | **켜짐** (`has_imu` 전달) | **꺼짐** (기본값 `false`) |

즉 **도구로 잘 돌아도 서버가 몰아서 도는 실제 세션은 다르게 동작합니다.**
통합 시험에서 도구 결과를 그대로 근거로 삼을 수 없다는 게 실질적인 문제입니다.
두 경로를 맞춰주시면 도구가 실제 주행의 대역이 됩니다.

---

## 1. `calib_path_test`에 회전 부호 반전이 빠진 것 같습니다

### 확인한 것

`PathFollower`는 **내부 규약(양수 = 좌회전)** 을 쓰고, `main.cpp`가 전선값을
뒤집어 넘겨서 프로토콜(`양수 = 우회전 CW`)과 이어줍니다. 두 곳 모두 그렇게
돼 있습니다:

```cpp
// main.cpp:344 (도색 TURN)
// Protocol rule: positive angle = turn right (CW) -> StartTurn(-angle_deg)
float robot_turn_deg = -current_seg.angle_deg;
path_follower.StartTurn(robot_turn_deg, l_steps, r_steps);

// main.cpp:243 (ALIGN)
float robot_turn_deg = -align_deg;
```

**그런데 `calib_path_test.cpp:133`에는 이 반전이 없습니다:**

```cpp
float turn_angle_deg = (start_corner == "top_left") ? 90.0f : -90.0f;   // :39
...
path_follower.StartTurn(seg.angle_deg, start_l, start_r, start_yaw);    // :133  ← 그대로 전달
```

### 그래서 어떻게 되나

`bottom_left`에서 `-90`을 `PathFollower`에 그대로 넘기면, 내부 규약상 음수는
**우회전(CW)** 입니다. 그런데 코드 주석은 반대로 적혀 있습니다:

```cpp
// bottom_left: CCW left turn (-90.0 deg)     ← 주석은 CCW
// top_left   : CW right turn (+90.0 deg)
```

기하학적으로는 **`bottom_left`가 CCW(좌회전)가 맞습니다** — 좌하단에서 +x로
출발했으니 사각형을 그리려면 첫 모서리에서 왼쪽으로 돌아야 합니다.

즉 지금 상태로 실행하면 **주석·설계 의도와 반대 방향(CW)으로 돌 가능성이
높습니다.**

### 왜 이게 위험한가 — 조용히 틀립니다

회전 **각도는 90°로 맞고 방향만 반대**면, 로봇은 사각형을 거울상으로 그립니다:

```
서버가 CCTV에 보내는 라벨:  (0,0) (450,0) (900,0) (900,300) (900,600) ...
로봇이 실제로 간 위치:       (0,0) (450,0) (900,0) (900,-300) (900,-600) ...
```

y축 반전 = **반사 변환**이고, 반사도 아핀 변환입니다. `findHomography`가 그대로
흡수해버려서:

- 재투영 잔차 → **0**
- 폐합오차 → **0** (사각형은 여전히 닫힘)

**두 검증 장치가 모두 정상이라고 보고합니다.** 결과물은 좌우가 뒤집힌
`H_marker`이고, 그걸로 도색하면 도면이 거울상으로 그려집니다. 설계 문서 §8-1이
경고한 사각지대와 정확히 같은 부류입니다.

### 확인 방법 — 첫 모서리만 보면 됩니다

```bash
./calib_path_test 90 60 bottom_left
```

| 관찰 | 판정 |
|---|---|
| 첫 TURN에서 **왼쪽(CCW)** 으로 돈다 | 정상 |
| 첫 TURN에서 **오른쪽(CW)** 으로 돈다 | 반전 누락 — `StartTurn(-seg.angle_deg, ...)`로 수정 필요 |

> 서버가 보내는 전선값은 `bottom_left → -90`, `top_left → +90`으로 이 도구의
> `turn_angle_deg`와 **동일합니다.** 그래서 `main.cpp`와 같은 방식으로 반전만
> 넣어주시면 서버 주행과 이 도구의 결과가 일치하게 됩니다.

---

## 2. IMU 폐루프가 서버 주행 경로에서는 동작하지 않습니다

### 확인한 것

`UpdateTurn`의 새 인자가 **기본값을 갖고 있습니다**:

```cpp
bool UpdateTurn(int32_t cur_left_steps, int32_t cur_right_steps,
                Msg_SetSpeed_t& out_speed,
                float cur_imu_yaw = 0.0f, bool has_imu = false);   // ← 기본값
```

`calib_path_test`는 IMU 값을 넘기지만, **`main.cpp`는 커밋 `01dd855`에서
수정되지 않았습니다**:

```cpp
// main.cpp:270, 349 — 인자 3개만 전달 → has_imu = false
path_follower.UpdateTurn(l_steps, r_steps, target_speed);
```

`has_imu = false`면 IMU 분기를 타지 않고 **기존 스텝 오도메트리 폴백**으로
종료합니다.

### 확인 부탁드리는 것

**의도하신 것인지** 알고 싶습니다. 두 경우 다 말이 됩니다:

- **의도적** — 도색 주행은 서버 피드백(ALIGN/DRIFT)이 각도를 잡아주니 IMU가
  불필요하고, 캘리처럼 피드백이 없는 개루프에서만 IMU를 쓴다
- **미적용** — `main.cpp` 반영이 아직 안 된 것

**서버 주행 캘리(`PATH{phase:"calib"}`)에는 IMU가 필요합니다.** 이 주행은
서버가 ALIGN/MORE/DRIFT를 **하나도 보내지 않는 완전 개루프**라(카메라 피드백을
쓰면 옛 캘리로 새 캘리를 보정하는 순환 참조가 됩니다), 회전 정확도가 전적으로
로봇에 달려 있습니다.

즉 `calib_path_test`로는 IMU가 켜진 채 잘 돌아도, **서버가 몰아서 도는 실제
세션에서는 IMU 없이 도는** 상태가 됩니다.

### 함께 봐주실 것 — 종료 조건 부호 (`calib_path_test` 한정)

IMU 분기의 종료 판정이 이렇습니다:

```cpp
float target_yaw = turn_start_imu_yaw + turn_target_angle_deg;
float yaw_error  = std::fabs(cur_imu_yaw - target_yaw);
if (yaw_error <= 0.8f || progress_steps >= turn_target_steps * 1.35f) { ... }
```

`turn_target_angle_deg`는 **`PathFollower` 내부 규약(양수=좌회전)** 값입니다.
자이로 Z가 CCW 양수라면 부호가 맞아떨어지지만, **센서 장착 방향에 따라 반대일
수 있어** 코드만으로는 확정할 수 없었습니다.

부호가 반대면 `yaw_error`가 영영 0.8° 안에 못 들어오고 **1.35배 스텝 가드로만
종료**됩니다. 그 경우 계산상:

```
CalculateTurnSteps(90) = 2073 steps        (90°)
× 1.35                 = 2798 steps        (약 121.5°)
```

**매 회전이 90° 대신 약 121.5°** 가 되고, 3번 돌면 폐합오차가 **약 813mm**로
벌어집니다(사각형이 아니라 찌그러진 삼각형).

> ⚠️ 이 위험은 **`calib_path_test`에만** 해당합니다. `main.cpp`는 `has_imu=false`
> 라 IMU 분기 자체를 안 타고 기존 스텝 종료 로직(`progress_steps >=
> turn_target_steps`)으로 돌기 때문에 1.35배 가드에 걸릴 일이 없습니다.

### 로그로 바로 판별됩니다

`calib_path_test` 1회 실행 시 나오는 **TURN 3줄 + 마지막 SUMMARY 1줄**만
보내주시면 확인됩니다:

```
정상 → Turn IMU Closed-loop finished! Target Yaw: -90.00 deg
       | Final IMU Yaw: -89.65 deg (Error: 0.35 deg) | Steps: 2081/2073

이상 → ... (Error: 179.xx deg) | Steps: 2798/2073      ← 딱 1.35배에서 끊김
```

```
[CALIB SUMMARY] Final Accumulated IMU Yaw Error: +XX deg
   → 정상이면 몇 도 이내, 부호가 반대면 누적 90도 이상
```

---

## 3. 정리 — 부탁드리는 것

**`calib_path_test 90 60 bottom_left`를 1회 실행하고 두 가지만 알려주세요.**

1. **첫 모서리에서 로봇이 어느 쪽으로 도나요?** (왼쪽이 정상)
   → 오른쪽이면 `StartTurn`에 부호 반전이 빠진 것입니다 (§1)

2. **TURN 로그 3줄 + 마지막 SUMMARY 줄**
   → IMU 폐루프가 `Error 0.8도 이내`로 끝나는지, `Steps: 2798/2073`처럼 1.35배
   가드에서 끊기는지 (§2)

추가로 **`main.cpp`의 `UpdateTurn` 호출에 IMU를 넘기지 않은 것이 의도인지**만
확인 부탁드립니다 (§2). 서버 주행 캘리에는 IMU가 켜져 있어야 합니다.

> **로봇 주행 코드 자체는 이 기능을 위해 바꾸실 게 없습니다.** 서버가
> `CALIB_START`(R-1 노즐 UP) + `PATH{phase:"calib"}`를 보내고, 나머지는 기존
> `READY`/`GO`/`PATH_DONE` 핸드셰이크를 그대로 탑니다. 중단은 `CALIB_CANCEL`
> → `CALIB_STOPPED`(R-2)로 처리되고, 서버가 그 회신을 반드시 기다립니다.
