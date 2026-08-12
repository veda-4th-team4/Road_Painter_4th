# 로봇 오도메트리 호모그래피 — 실행 계획 (서버팀)

- 작성일: 2026-08-12
- 상태: **Phase A~D 완료 (2026-08-12). 서버 구현·오프라인 테스트 통과.**
- 🔴 **기능 정본은 `docs/ROBOT_ODOMETRY_HOMOGRAPHY.md`** — 이 문서는 구현
  지시서로, 코드 앵커와 완료 기준을 담는다. 구현 현황 요약은 정본 §10에 있다.
- 실행자에게: 이 문서는 **이 대화 맥락 없이 단독 실행**할 수 있게 쓰였다.
  §0의 배경 문서 4개를 먼저 읽고 시작할 것. 판단이 갈리는 지점은 §7에 "이미 내린
  결정"으로 박아뒀다 — 다시 논의하지 말고 그대로 따를 것.

## 0. 배경 문서 (읽는 순서)

| # | 문서 | 역할 |
|---|---|---|
| 1 | `docs/ROBOT_ODOMETRY_HOMOGRAPHY.md` | **설계 정본** (Phase B의 산출물 — 아래 참고) |
| 2 | `docs/ROBOT_ODOMETRY_HOMOGRAPHY_REPLY_CCTV_20260812.md` | CCTV 1차 회신 |
| 3 | `docs/ROBOT_ODOMETRY_HOMOGRAPHY_REPLY_SERVER_20260812.md` | 서버 1차 회신 |
| 4 | `docs/ROBOT_ODOMETRY_HOMOGRAPHY_REPLY_CCTV_02_20260812.md` | CCTV 2차 회신 (확정 요청 5건) |
| 참고 | `docs/PROTOCOL_v2_ROBOT.md` | 로봇 대면 규격 **정본** |
| 참고 | `QT_CCTV_SERVER_CALIBRATION_FORMAT_20260811.md` | 캘리 번들 mm 규약 |

**한 줄 요약**: 로봇이 알려진 크기의 직사각형을 개루프로 돌고, 9개 정지점마다
CCTV가 마커를 캡처해 (물리좌표, 픽셀) 쌍 8개를 모아 `H_marker`를 직접 구한다.
로봇 펌웨어 변경은 없다 — 기존 `PATH{phase:"calib"}` 핸드셰이크를 그대로 탄다.

---

## 1. 전체 순서

```
Phase A  wire 스펙 문서          ← 최우선. CCTV가 이걸 보고 동시에 코딩한다
Phase B  설계 본문 갱신          ← 1·2차 회신 반영
Phase C  서버 구현               ← A 확정 후 착수
Phase D  회귀 테스트 + 빌드
```

**A를 먼저 하는 이유**: CCTV팀이 이미 카메라 구현(2차 회신 §5의 A~E·H)에
들어갔다. 양쪽이 각자의 이해로 짜면 통합에서 어긋난다. 지금 가장 값어치 있는
산출물은 두 팀이 같이 보고 코딩할 **메시지 형식 확정본 한 장**이다.

---

## 2. Phase A — wire 스펙 문서

**산출물**: `docs/ROBOT_ODOMETRY_HOMOGRAPHY_WIRE_20260812.md`

논의 문서가 아니라 **규격서**다. 근거·검토 과정은 배경 문서에 있으니 되풀이하지
말고, 구현자가 필드 하나하나를 확인할 수 있는 형태로만 쓴다.

담을 내용:

1. **메시지 5종의 정확한 형식** — 필드명 / 타입 / 필수·선택 / 단위

```
Server → CCTV   CALIB_CAPTURE       {ch, request_id, point_index, world_xy_mm:[x,y]}
CCTV   → Server CALIB_CAPTURE_OK    {ch, request_id, point_index, pixel_uv:[u,v], spread_px}
CCTV   → Server CALIB_CAPTURE_FAIL  {ch, request_id, point_index, reason}
Server → CCTV   CALIB_DONE          {ch, request_id, m_mm, n_mm}
CCTV   → Server H_MATRIX            {ch, calib:{...}}        (기존 계약)
```

2. **매칭 키 = `ch` + `request_id` + `point_index` 세 개 전부 일치.**
   하나라도 어긋나면 무시하고 WARN. (`request_id`만으로도, `point_index`만으로도
   부족한 이유는 CCTV 2차 §3-2 참고 — 세션 재시작 시 `point_index`가 0으로
   리셋되므로 재시도 직후 충돌이 흔하다.)

3. **`world_xy_mm`의 정의** — 🔴 이 한 줄이 가장 중요하다.
   > `world_xy_mm` = 그 순간 **마커 중심**의 월드 좌표 (로봇 기준점이 아니다).
   오프셋 보정은 서버 내부 계산이고 카메라는 무관하다.

4. **9점 좌표표** — `start_corner` 2종 × `point_index` 0~8. 계획서 §3의 표를
   그대로 옮기되 8점판(중점 포함)으로. `point_index 8`은 복귀점이며
   **피팅 입력에서 제외**(라벨이 0번과 겹침), 폐합오차 진단 전용임을 명시.

5. **실패 사유 2층 구분** — 실행자가 표로 만들 것.
   - **지점 수준** (건너뛰고 계속): `marker_not_found`, `not_settled`, `unmappable`
   - **세션 수준** (즉시 중단): `no_intrinsics`, `session_conflict`, `fit_failed`,
     `too_few_points`
   - 유효 대응점 **6개 미만이면 `CALIB_FAIL`** (카메라가 판정)

6. **타임아웃 3종** — 누가 재고 무엇을 하는가
   | 주체 | 값 | 대상 | 만료 시 |
   |---|---|---|---|
   | 서버 | `calib_capture_timeout_ms` (15000) | 캡처 ack 1건 | 세션 안전 중단 |
   | 서버 | `calib_odo_timeout_ms` (300000, 신설·별개 파라미터) | 세션 전체 | 세션 안전 중단 |
   | 카메라 | 10분 | 마지막 `CALIB_CAPTURE`부터 | 카메라 세션 폐기, `/status`에 기록 |
   | 카메라 | 10초 | 정지 판정 1건 | `CALIB_CAPTURE_FAIL{not_settled}` |

7. **채널 번호는 1-based** (`src/protocol.hpp:358` 규약과 동일)

8. **번들 필수 필드** — `method`(`"checkerboard"`|`"robot_motion"`), `K`, `D`,
   `H_marker`, `H_floor`(역산), `marker_height_mm`, `image_size`,
   `coord_mode:"undistort"`, `unit:"mm"`, `calib_id`, `created_at`,
   `origin_mm`/`canvas_mm`. **`K`/`D`가 비면 카메라가 전송 거부.**

9. **안전 정지 경로** (§7-D 결정 반영) — `robot_motion` 세션의 중단은 로봇에
   `ABORT_DRAW`, CCTV에 `CALIB_CANCEL`. 카메라 쪽 규약은 변화 없음.

---

## 3. Phase B — 설계 본문 갱신 ✅ 완료

**당초 대상**: `ROBOT_ODOMETRY_HOMOGRAPHY_PLAN_20260811.md`(설계 초안)

**실제 결과**: 아래 B1~B8을 초안에 반영한 뒤, 정정 각주가 누적되어 신규 독자에게
읽히지 않는 문제가 남았다. 그래서 내용을 **`docs/ROBOT_ODOMETRY_HOMOGRAPHY.md`
(정본)로 재작성해 통합하고 초안은 삭제했다**(2026-08-12). 초안 원문이 필요하면
git 히스토리 커밋 `43c4f83`을 볼 것.

1차·2차 회신에서 확정된 것을 본문에 반영한다. 개별 편집 항목:

| # | 절 | 편집 |
|---|---|---|
| B1 | §2-1, §2-3 | "유일한 자체 검증 수단" / "유일한 직접 측정값" 표현 삭제. 서버 1차 회신 §2-2의 표(잔차·폐합이 각각 무엇을 놓치는가)를 옮겨 붙이고, 사각지대가 **스케일 계열 2종**임을 명시 |
| B2 | §3 | `world_xy_mm` 정의를 "마커 중심"으로 교체. 마커 오프셋 문단은 "서버가 헤딩으로 보정, `marker_offset_m` 기본 0.0" 으로 재작성 |
| B3 | §3-1 | 메시지 형식에 `spread_px` / `m_mm` / `n_mm` / `CALIB_CAPTURE_FAIL` 추가. 매칭 키 3종 명시 |
| B4 | §4 | 대응점 보관·피팅·번들 조립·전송 주체가 **카메라**임을 명시 (계획서는 관리자창까지의 경로가 비어 있었다) |
| B5 | §4-2 | 1 m 직진 실측(엔코더 스케일 보정계수)을 **선행 항목**으로 추가. 그 값을 확보한 뒤에도 남는 이점 3가지(직접 측정 / 작업 영역 전체 / 물리 타깃 불필요)를 함께 |
| B6 | §5 | 서버 작업 목록을 §4의 C1~C8로 교체 |
| B7 | §6 | 답변 완료된 CCTV 검토 항목 삭제. 남는 것: 1 m 실측, 360° 오프셋 확인 |
| B8 | 신설 | 안전 정지 절 — 취소·타임아웃이 `ABORT_DRAW`를 타는 이유 (§7-D) |

---

## 4. Phase C — 서버 구현

### C0. 신규 파일 배치

| 파일 | 내용 |
|---|---|
| `src/router_odocalib.cpp` (신규) | 주행 캘리 세션 로직 전부 |
| `src/ops_builder.hpp` | `buildCalibRectOps()` 추가 |
| `src/router.hpp` | 상태 멤버 + 메서드 선언 |
| `src/router.cpp` | `onReady`/`PATH_DONE`/`fromCctv` 분기 3곳 |
| `src/params.hpp`, `config/params.json` | 파라미터 3종 |
| `Makefile` | 새 소스 + 새 테스트 타깃 |

`router_calib.cpp`(기존 정적 앵커 세션, 304행)는 **건드리지 않는다.** 분기가
필요한 두 함수(`cancelCalib`, `checkCalibTimeout`)만 최소 수정한다 — C7 참고.

### C1. `buildCalibRectOps()` — `src/ops_builder.hpp`

```cpp
// 닫힌 직사각형 11-op. 피드백은 전부 꺼진 채로 나간다 (hasTarget=false,
// heading=kNoHeading) - phase 가드(C3)와 이중 방어다.
inline PlannedPath buildCalibRectOps(double m_m, double n_m, bool ccw);
```

- ops: `MOVE(m/2), MOVE(m/2), TURN(±90), MOVE(n/2), MOVE(n/2), TURN(±90),
  MOVE(m/2), MOVE(m/2), TURN(±90), MOVE(n/2), MOVE(n/2)` — 총 11개
- `ccw=true`(= `start_corner=="bottom_left"`)면 `TURN(+90)`, `false`면 `TURN(-90)`
- 모든 op은 `isPath=false`, `hasTarget=false`, `headingDeg=kNoHeading`,
  `centerAheadByA=false`
- `nozzle` op 없음 (도색하지 않는다)
- 기존 `moveOp(distM, isPath, headCcw, hasTgt, tgt, centerAhead)` /
  `turnOp(angCcw, isPath, headAfterCcw)` 헬퍼를 그대로 쓴다

**검증**: `min_move_m`(0.01) 대비 반쪽 구간이 충분히 큰지 확인.
기본값 90×60cm면 450mm/300mm라 여유 있다. 사용자가 아주 작은 값을 넣으면
`CALIB_START`를 거절할 것 — 하한은 `min_move_m * 2 * 1000` mm.

### C2. 9점 좌표 계산

`start_corner`별 표 (단위 mm, `m`/`n`은 `m_cm*10`/`n_cm*10`):

| idx | `bottom_left` | `top_left` | 헤딩 |
|---:|---|---|---:|
| 0 | `(0, 0)` | `(0, n)` | 0° |
| 1 | `(m/2, 0)` | `(m/2, n)` | 0° |
| 2 | `(m, 0)` | `(m, n)` | 0° |
| 3 | `(m, n/2)` | `(m, n/2)` | 90° / 270° |
| 4 | `(m, n)` | `(m, 0)` | 90° / 270° |
| 5 | `(m/2, n)` | `(m/2, 0)` | 180° |
| 6 | `(0, n)` | `(0, 0)` | 180° |
| 7 | `(0, n/2)` | `(0, n/2)` | 270° / 90° |
| 8 | `(0, 0)` | `(0, n)` | 270° / 90° |

**마커 오프셋 보정**: 위 표는 로봇 회전 중심의 좌표다. `marker_offset_m != 0`이면
각 점에 헤딩 방향으로 오프셋을 더한다:

```
world = center + offset_mm * (cos(heading), sin(heading))
```

기본값 0.0이므로 표 그대로 나간다. 값이 0이 아닌 것으로 밝혀지면
`params.json`만 고치면 되고 코드는 그대로다.

### C3. `onReady` 분기 — `src/router.cpp:394`

`Router::onReady(int k)`의 `runningOp_ = -1;` 직후, `planActive_` 가드
**다음**·`manualMode_` 검사 **이전**에 삽입:

```cpp
if (activePhase_ == "calib") { onCalibReady(k); return; }
```

`onCalibReady(k)`:

```
odoPointOfBoundary_[k] < 0        → sendGo(k, "캘리 - 회전 직후라 같은 위치")
그 외                              → CALIB_CAPTURE 전송,
                                      odoPointIdx_ = 그 point_index,
                                      odoPendingReadyOp_ = k,
                                      odoCaptureMs_ = nowMs()
                                      (GO는 ack를 받고 나서 보낸다)
```

boundary→point_index 표 (직전 op이 TURN이면 -1):

```
READY  0  1  2  3  4  5  6  7  8  9 10   PATH_DONE
idx    0  1  2 -1  3  4 -1  5  6 -1  7      8
```

> 🔴 `onReady`의 불변식: **들어온 READY는 반드시 응답 하나를 받고 나가야 한다.**
> 캘리 분기에서는 그 응답이 ack 수신 후로 미뤄지는 것이지 생략되는 게 아니다.
> 캡처 타임아웃(C6)이 그 미뤄진 응답의 안전망이다.

### C4. `PATH_DONE(calib)` 분기 — `src/router.cpp`

`PATH_DONE` 핸들러에서 `activePhase_ == "calib"`이면 도색 완료 처리로 가지 말고:

```
point_index 8 의 CALIB_CAPTURE 전송
odoPendingReadyOp_ = -1     // ack 후 GO 대신 CALIB_DONE
```

### C5. CCTV 응답 처리 — `fromCctv`

`CALIB_CAPTURE_OK` / `CALIB_CAPTURE_FAIL` 수신:

```
1. 매칭 검사: ch, request_id, point_index 3개 전부 일치 아니면 WARN 후 무시
2. OK  : pixel_uv/spread_px 로깅, odoValid_++
   FAIL: reason 이 세션 수준이면 abortOdoCalib() 후 return
         지점 수준이면 로그만 남기고 계속 (odoValid_ 증가 없음)
3. odoPointIdx_ = -1
4. odoPendingReadyOp_ >= 0 → sendGo(odoPendingReadyOp_, "캘리 캡처 완료")
   그 외 (PATH_DONE 트리거였음) → CALIB_DONE{ch,request_id,m_mm,n_mm} 전송 후
                                   clearCalib() + clearPath()
5. idx 0 과 idx 8 의 pixel_uv 차이를 폐합오차로 로깅 (픽셀 단위. mm 환산은
   불가능하다 - 그 시점엔 새 캘리가 아직 없다. mm 값은 카메라 기록이 정본)
```

**세션 종료 시 Qt 통지 없음**: 주행 캘리는 ADMIN 개시 전용이라 `calibFromQt_`가
false다. `failCalib()`이 이미 쓰는 패턴(`if (toQt) send`)과 같다. 호모그래피는
나중에 관리자창 → 서버 `H_MATRIX` 경로로 따로 들어와 Qt에 중계된다.

### C6. 파라미터 — `src/params.hpp` + `config/params.json`

| 키 | 값 | 비고 |
|---|---:|---|
| `calib_capture_timeout_ms` | 15000 | 신설. 카메라 자체 한도 10초에 여유 |
| `calib_odo_timeout_ms` | 300000 | **신설, 별개 파라미터** — 아래 참고 |
| `marker_offset_m` | 0.0 | 신설. 마커 중심 − 회전 중심 |

> ⚠️ **`calib_timeout_ms`(기존 값, 180000)는 건드리지 않는다.** 처음엔 이 값
> 자체를 300000으로 올릴 계획이었으나, `params.hpp`의 `sanitize()`가 이미
> `calib_timeout_ms >= 300000`을 290000으로 깎는 클램프를 갖고 있다 — 근거는
> "Qt가 종결 응답 없이 5분 지나면 스스로 포기하므로 서버가 그보다 짧아야 한다"
> 인데, **이 제약은 정적 앵커 방식에는 지금도 유효하다**(그쪽은 Qt가 계속
> 트리거한다). 두 방식이 값을 공유하면 정적 앵커의 안전장치가 깨지므로,
> 오도메트리 전용 `calib_odo_timeout_ms`를 새로 만들어 `checkCalibTimeout()`이
> `calibIsOdo_`로 갈라 쓴다. 이 값에는 300000 상한 클램프가 없다 — 오도메트리는
> v1에서 Qt가 절대 트리거하지 않으므로 그 제약이 애초에 안 걸린다.

`sweep()`에 캡처 타임아웃 검사 추가 — `odoPointIdx_ >= 0` 이고
`now - odoCaptureMs_ > calib_capture_timeout_ms`면 `abortOdoCalib("capture_timeout")`.
**건너뛰지 말고 중단한다** — 카메라가 침묵하는 상황에서 3 m를 계속 도는 것은
의미가 없다.

### C7. 🔴 안전 정지 — 이번 작업에서 가장 중요한 항목

**문제**: 지금 코드는 이 방식에서 **로봇을 못 세운다.**

1. `cancelCalib()`(`router_calib.cpp:222`)은 `CALIB_CANCEL`을 ROBOT에 중계하고
   `CALIB_STOPPED`를 기다린다. 그런데 **로봇 펌웨어에 `CALIB_*` 핸들러가 없다**
   (`Paint_Robot/RaspberryPi/`에 0건). 로봇은 무시하고 ack를 안 보내
   `calib_cancel_ack_ms`(5초) 뒤 `cancel_failed`로 떨어지는데, **그동안 로봇은
   계속 굴러간다.**
2. `failCalib()`(`router_calib.cpp:42`)은 QT에만 통지하고 `clearCalib()`으로
   조용히 세션을 지운다. ROBOT·CCTV 통지가 없다. 300초에 터지면 로봇은 주행 중,
   카메라는 렌즈를 잡은 채 서버만 손을 뗀다.

기존 정적 앵커 방식은 로봇이 관여하지 않아 문제가 안 됐다. 로봇을 주행시키는
이 방식이 그 경로를 처음 밟는다. 코드에 이미 적힌 우려가 그대로 현실이 된다:

> 그 순간 로봇이 아직 굴러가고 있으면 사람이 다치는 쪽에 서 있게 된다
> — `router_calib.cpp:190`

> ⚠️ **확인 결과 (구현 중 코드 대조)**: `abortDraw()`(`router_channel.cpp:78`)는
> **로봇에 아무것도 보내지 않는다** — 서버 로컬 상태(`clearPath()`,
> `drawRequested_=false`)만 정리하고 `wasActive`를 반환할 뿐이다. 실제
> `ABORT_DRAW` 전송은 호출부(`router.cpp:197`)가 Qt가 보낸 원본 CMD 메시지를
> `srv_.sendTo("ROBOT", msg)`로 그대로 릴레이하는 것이다. 그리고 로봇
> (`main.cpp:77`)은 `ABORT_DRAW` 수신 즉시 STM32에 속도 0을 내리는 **동기
> 처리**라 서버로 돌아오는 ack가 아예 없다 — `DRAW_ABORTED`는 로봇의 회신이
> 아니라 서버가 명령을 릴레이한 직후 QT에 보내는 통지다(`router.cpp:199`).
> 아래 절차는 이 사실에 맞게 조정했다: 로봇 쪽은 fire-and-forget, CCTV
> 쪽만 ack를 기다린다.

**구현**: `abortOdoCalib(const char* reason, const std::string& msg)` 신설

```
1. 로봇 정지 (fire-and-forget, ack 대기 없음):
   srv_.sendTo("ROBOT", makeMsg("CMD", {{"cmd","ABORT_DRAW"}}))
   abortDraw() 는 호출하지 않는다 — 그건 planActive_/awaitingArrival_ 등
   "도색" 상태를 정리하는 함수라 캘리 세션(activePhase_=="calib") 상태와
   맞지 않는다. 로봇 정지 자체는 위 한 줄로 끝난다.
2. CCTV 에는 CALIB_CANCEL 을 그대로 보낸다 (카메라 규약 변화 없음)
3. CCTV 의 CALIB_STOPPED 만 기다린다. calib_cancel_ack_ms 안에 안 오면
   로그를 남기고 진행한다 — 여기서 갇히면 세션이 영영 안 닫힌다. (로봇의
   동기 정지는 이미 1번에서 끝났으므로 기다릴 대상이 아니다.)
4. clearCalib() + clearPath()
```

**분기 2곳** (기존 함수 최소 수정):
- `cancelCalib()` 진입부: `if (calibIsOdo_) { abortOdoCalib("cancelled", ...); return; }`
- `checkCalibTimeout()`: 만료 시 `calibIsOdo_`면 `failCalib()` 대신 `abortOdoCalib()`

**정적 앵커 방식(`method != "robot_motion"`)의 동작은 한 줄도 바꾸지 않는다.**

### C8. 번들 검증 WARN — `src/router.cpp`

기존 `warnCalibSpec()`에 한 줄 추가: `coord_mode == "undistort"`인데 `K` 또는
`D`가 없으면 WARN. **거부하지 않는다** (레거시 번들 호환).

이유: 서버의 왜곡 보정은 `Calib::hasKD`로만 켜진다. `K`/`D`가 빠진 번들이 오면
서버는 보정을 **조용히 건너뛰고** H를 raw 픽셀에 그대로 적용한다 — H는 보정된
픽셀을 기대하는데. 에러 없이 좌표만 틀어진다. 이 프로젝트에서 이미 겪은 사고다
(`admin_console/cctv.py` 주석: "에러 없이 보정만 꺼졌다").

---

## 5. Phase D — 테스트

**신규**: `tools/odo_calib_test.cpp`

> ⚠️ **이름 정정 (2026-08-12, 구현 중 발견)**: 처음엔 `tools/calib_session_test.cpp`로
> 계획했으나, 그 이름은 이미 `Makefile`(§`calib_session_test:`)과 `.gitignore`에
> **다른 테스트용으로 예약돼 있었다** — 2026-08-10 계약(정적 앵커, QT 트리거)의
> `tools/tls_client.hpp` 기반 **통합** 테스트(서버를 띄워두고 QT/ROBOT/CCTV
> 세 role로 실제 접속해 "종결 응답 하나로 닫힌다"를 검증) 자리다. 아직
> 작성되지 않은 파일이라 이름만 봐서는 비어 보였지만, 내가 계획한
> **오프라인 단위 테스트**(서버 미기동, `ops_builder.hpp`/좌표 계산 함수를
> 직접 호출)와는 성격이 다르다. 이름을 뺏으면 나중에 그 통합 테스트를 쓸 때
> 충돌한다 — `odo_calib_test`로 새로 짓는다.

| # | 검증 |
|---|---|
| T1 | 11-op 시퀀스 — `bottom_left`는 `TURN(+90)`, `top_left`는 `TURN(-90)`. 개수·거리 |
| T2 | 9점 좌표표 — 두 `start_corner` 모두, `m_cm=90,n_cm=60` 기준 실제 값 대조 |
| T3 | boundary→point_index 매핑 — 직전 op이 TURN인 3곳이 -1인지 |
| T4 | 매칭 키 — `request_id` 불일치 / `point_index` 불일치 / `ch` 불일치 각각 거부 |
| T5 | 부분 실패 — 지점 수준 사유는 계속 진행, 세션 수준 사유는 중단 |
| T6 | `marker_offset_m = 0.1` 일 때 헤딩별 보정이 표와 다르게 나오는지 |
| T7 | 모든 op의 `hasTarget=false`, `headingDeg=kNoHeading` (피드백 차단 확인) |

`Makefile`에 빌드 타깃 + `clean` 등록. 기존 `calib_unit_test` /
`offset_feedback_test` 패턴을 그대로 따를 것.

**빌드·회귀 실행**:

```bash
cd /home/team4/Road_Painter_4th/Server && make clean && make server odo_calib_test calib_unit_test calib_channel_test offset_feedback_test && ./tools/odo_calib_test && ./tools/calib_unit_test && ./tools/calib_channel_test && ./tools/offset_feedback_test
```

---

## 6. 완료 기준

- [ ] `make server` 경고 없이 통과
- [ ] 회귀 테스트 4종 전부 0 fail
- [ ] 정적 앵커 캘리(`calib_channel_test`)가 **회귀 없음** — 이 작업의 최대 위험
- [ ] `CALIB_START{method:"robot_motion"}` → 11-op `PATH{phase:"calib"}` 로그 확인
- [ ] 캘리 주행 중 ALIGN/MORE/DRIFT가 **한 번도 안 나가는지** 로그 확인
- [ ] 캡처 타임아웃·세션 타임아웃·취소 세 경로 모두에서 로봇에 정지가 나가는지
- [ ] `params.hpp` 기본값과 `params.json` 값이 일치 (과거에 3건 어긋난 적 있음)

**통합 시험은 CCTV의 카메라 구현(2차 회신 §5 A~E·H) 완료 후.** 서버는 그와
무관하게 지금 짜고 오프라인 회귀까지 갈 수 있다.

---

## 7. 이미 내린 결정 (다시 논의하지 말 것)

| # | 결정 | 근거 |
|---|---|---|
| A | Qt 트리거는 v1에서 **보류**. 관리자창 전용 | 기존 `CALIB_START` 종결 응답 계약(`H_MATRIX`\|`CALIB_FAIL`\|`CALIB_CANCELLED` 중 하나)을 깨지 않기 위해. 캡처 완료와 H 산출 사이에 사람 입력이 낀다 |
| B | 서버 세션은 `CALIB_DONE`에서 **끝난다**. H는 나중에 별도 경로로 | 위와 같음 |
| C | `world_xy_mm`는 **마커 중심** | 프로토콜이 물리 측정 결과를 기다리지 않게. 오프셋 0이면 표 그대로 |
| D | 중단 시 로봇에는 `ABORT_DRAW`, CCTV에는 `CALIB_CANCEL` | 로봇에 `CALIB_*` 핸들러가 없다. 안전 문제 |
| E | 매칭 키 3종 전부 | 세션 재시작 시 `point_index` 리셋 충돌 |
| F | 부분 실패는 **계속 진행**, 유효 6점 하한 | 한 점 때문에 3 m 주행을 버리는 건 비싸다. 4~5점이면 LOO가 무의미 |
| G | 캡처 **타임아웃**은 건너뛰지 않고 **중단** | 카메라 침묵 = 링크 문제. 계속 도는 게 무의미 |
| H | 사각형 첫 주행은 **900×600 유지** | 키우면 원근항 조건수는 좋아지나 오도메트리 드리프트가 같이 는다. 폐합오차를 보고 판단 |
| I | 잔차·폐합오차는 **스케일 오차를 못 잡는다** | 균일 스케일은 `diag(s,s,1)·H`로 완전히 흡수된다. 검증은 체커보드 비교(닮음변환 `s`)로만 가능 |

## 8. 하지 말 것

- 🔴 **`admin_console/cctv.py`, `admin_console/restart.sh` 수정 금지.**
  CCTV팀이 작업 중인 미커밋 파일이다. `git add`에도 넣지 말 것
- 🔴 **커밋·푸시 금지.** 명령만 제시하고 사용자가 직접 실행한다
- 캘리 번들을 미터로 바꾸지 말 것 — 저장·중계본은 **mm 보존**이 규약이다.
  ÷1000은 내부 계산용 사본(`Calib::Hf`/`Hm`)에서만 일어난다
- 정적 앵커 캘리 경로의 동작 변경 금지 (C7의 분기 2곳 제외)
- `Road_Painter_4th-server-driven-v2` 저장소는 더 이상 사용하지 않는다.
  이쪽으로 무엇도 옮기지 말 것
- SRS의 오도메트리 금지 조항을 근거로 작업을 되돌리지 말 것 — 그 조항을
  알고도 새 시도로 진행하기로 한 결정이다 (계획서 머리말 참고)

## 9. 미결 — 구현을 막지는 않는 것

| 항목 | 담당 | 성격 |
|---|---|---|
| 1 m 직진 실측 (엔코더 스케일 보정계수) | 로봇팀 | 검증 입력. `marker_offset_m`처럼 나중에 값만 반영 |
| 360° 회전으로 마커 오프셋 확인 | 로봇팀 | 위와 같이 측정. 원이 그려지면 그 반지름이 오프셋 |
| `method` 필드 / `K`·`D` 게이트 / `K` stale 표시 | CCTV | 2차 회신에서 전부 수용됨 |
