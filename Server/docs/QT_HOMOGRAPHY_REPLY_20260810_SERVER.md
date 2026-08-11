# 서버팀 회신 — 로봇 주행 호모그래피 연동 계약 (§7)

**회신일**: 2026-08-10
**대상 문서**: `QT_HOMOGRAPHY_SERVER_CONTRACT_2026-08-10.md` (Qt팀, `ch4_qt_client_only`)
**서버 기준**: `main`, 신규 `Server/src/router_calib.cpp`
**상태**: 서버 구현 완료 · 회귀 테스트 21/21 통과 · **CCTV/로봇 작업이 남아 통합 시험 불가**

---

## 0. 먼저 — 계약 자체는 그대로 받습니다

역할 경계(Qt는 시작/중단만), `ch`+`request_id` 상관관계, 성공·실패·중단의
명시적 종결 응답 — 세 가지 모두 이견 없습니다. 서버에 그대로 구현했습니다.

**다만 이 계약은 지금 상태로는 통합 시험에 들어갈 수 없습니다.** 서버 문제가
아니라 §3에 적은 두 가지 때문인데, §7이 서버팀에만 질문을 주셔서 그 사실이
문서에 안 드러나 있습니다. **§3을 §7보다 먼저 읽어주시기 바랍니다.**

---

## 1. §7-1 — `ch` 전달과 원자적 채널 전환

**위치**: `Server/src/router_calib.cpp` `Router::startCalib()`.
`fromQt()`/`fromAdmin()`의 `CMD` 분기에서 `CALIB_START`를 가로채 호출합니다.

**원자성은 구조적으로 보장됩니다.** `Router::onMessage()` 전체가 뮤텍스
하나(`mtx_`)로 직렬화되어 있어서, 채널 전환과 세션 개시 사이에 다른 메시지가
끼어들 수 없습니다. 우려하신 "`SELECT_CHANNEL`과 `CALIB_START` 사이의 경쟁
조건"은 **Qt가 `SELECT_CHANNEL`을 아예 안 보내는 것**으로 사라집니다 —
`CALIB_START` 하나가 채널 전환까지 겸합니다.

수락 시 서버가 하는 일의 순서:

1. 검증 (§2)
2. `activeChannel_` 전환 + pose 폐기 (`applyChannel()` — 채널마다 좌표계가 달라
   옛 pose를 남기면 새 채널 첫 POS 전까지 엉뚱한 위치를 믿습니다)
3. CCTV에 `CMD SELECT_CHANNEL{ch}` 전송
4. Qt에 `CHANNEL_OK{ch, calib}` 전송 — **계약에 없는 추가분입니다** (§5 참고)
5. ROBOT·CCTV에 원본 `CALIB_START` 그대로 중계 (`ch`/`request_id`/`method` 보존)
6. Qt에 `CALIB_STARTED`

3번은 채널이 안 바뀌었어도 **매번** 보냅니다. `activeChannel_`은 서버의 믿음일
뿐이고, 카메라가 그 사이 재시작했으면 실제로는 다른 채널을 보고 있습니다.
생략하면 서버가 그 어긋남을 영영 바로잡지 못한 채 엉뚱한 채널의 관측으로
캘리를 끝냅니다.

**세션 중 `SELECT_CHANNEL`은 거절합니다** — `CHANNEL_FAIL{reason:"calib_busy"}`.
로봇이 이미 옛 채널 기준으로 돌아다니는 중에 좌표계를 갈아치울 수는 없습니다.
Qt는 §5에 "계산 중 채널 전환 차단"을 이미 구현하셨으니 정상 흐름에서는 안 보이고,
서버는 방어선으로만 둡니다.

---

## 2. 검증 (§4) — 구현 내용과 한 가지 완화

| §4 항목 | 구현 | 실패 시 |
|---|---|---|
| 1. 로그인 상태 | QT 개시만 검사 (아래 ⚠️) | `internal_error` |
| 2. `ch` 범위 | `validChannel()` (1..8) | `invalid_channel` |
| 3. Robot·CCTV 접속 | `connectedRoles()` 조회 | `robot_offline` / `cctv_offline` |
| 4. 다른 작업 실행 중 | `planActive_ ‖ awaitingArrival_ ‖ drawRequested_`, `calibActive_` | `busy` |
| 5. 중복 `request_id` | 같으면 `CALIB_STARTED` 재전송(멱등) | — |

⚠️ **§4-1은 QT 개시에만 겁니다.** 서버는 로그인 사용자를 세션별이 아니라
**전역으로 1명만** 기억합니다(`currentUser_`). 그래서 이 검사는 "이 Qt 세션이
로그인했나"가 아니라 "결과를 저장할 계정이 정해져 있나"에 가깝습니다. 그리고
관리자 창 개시는 일부러 통과시킵니다 — 계정을 만들기 전의 설치 기사가 캘리를
못 하면 안 된다는 기존 결정(QT-REQ-SRV-001 R-1)과 충돌하기 때문입니다.
Qt 흐름은 어차피 전부 로그인 뒤에 있으므로 실무상 차이는 없습니다.

**🔴 개시자가 둘이 됐습니다.** 2026-07-23에 "캘리 시작은 관리자 창 담당,
Qt는 안 보냄"으로 정했고 그 경로가 아직 살아 있습니다(관리자 창의 "◎ 캘리브레이션
시작" 버튼). 이번 계약이 그 결정을 뒤집으므로, 두 경로가 **같은 세션 상태를
공유**하게 했습니다 — 한쪽이 도는 동안 다른 쪽은 `busy`입니다. 로봇이 한 대뿐이라
동시에 두 세션이 돌면 서로의 주행을 자기 관측으로 착각합니다.

> 관리자 창은 예전부터 `{"cmd":"CALIB_START"}` 하나만 보내왔습니다(`ch` 없음).
> 그 경로를 살리려고 **ADMIN이 `ch`를 생략하면 활성 채널로** 읽습니다.
> QT는 계약대로 `ch` 필수입니다(생략 시 `invalid_channel`).

---

## 3. 🔴 통합 시험을 막고 있는 것 — CCTV와 로봇

계약서 §7은 서버팀에만 물으셨는데, **서버를 다 고쳐도 지금은 동작하지 않습니다.**

### (a) CCTV가 `CALIB_START`의 payload를 통째로 버립니다

`CCTV/src/central_tls_sender.cpp:90`

```c
bool ok = strstr(read_buf,"\"type\":\"CMD\"") && strstr(read_buf,"\"cmd\":\"CALIB_START\"");
...
snprintf(out, out_len, "CALIB_START"); return 1;
```

JSON을 파싱하지 않고 **문자열 검색**만 한 뒤 `"CALIB_START"` 리터럴 하나만
넘깁니다. 서버가 `ch`와 `request_id`를 아무리 정성껏 실어 보내도 카메라 앱
경계에서 소멸합니다. **§7-1의 답이 성립하려면 이쪽 수정이 선행되어야 합니다.**

### (b) 로봇에 `CALIB_START` 핸들러가 없습니다

`Paint_Robot/RaspberryPi/src/main.cpp:66-115`는 `ESTOP`/`RESUME`/`ABORT_DRAW`/
조이스틱/노즐만 처리하고 `CALIB_START`는 `else`로 빠져 조용히 무시됩니다.
계약의 `method`가 `"robot_motion"`인데 **로봇이 그 주행 명령을 모릅니다.**

### (c) 그래서 실패 사유의 절반은 나올 수가 없습니다

권장 `reason` 중 서버가 **자력으로** 만들 수 있는 것:
`invalid_channel` · `busy` · `robot_offline` · `cctv_offline` · `cancel_failed` ·
`internal_error` · `timeout`.

`motion_failed` · `insufficient_samples` · `solve_failed`는 로봇/카메라만 아는
사실이라, 그쪽이 서버로 알려주지 않으면 나오지 않습니다. 서버는 그 경우
`timeout`으로 떨어뜨립니다(§4).

---

## 4. §7-2 — `H_MATRIX.calib` 스키마와 저장 시점

**서버는 스키마를 정하지 않고 CCTV가 보낸 형태를 그대로 중계합니다.**
`Router::handleHMatrix()`(`router.cpp`)가 세 가지를 받습니다:

| 형태 | payload 모양 | 비고 |
|---|---|---|
| 중첩 | `{"ch":n, "calib":{K,D,H_floor,H_marker,...}}` | 계약서 §3.4 예시가 이것 |
| 평면 | payload **자체가 번들** `{calib_id,K,D,H,H_marker,canvas_mm,...}` | QT-REQ-CCTV-001 rev.2 |
| 레거시 | `{"H":[[...]x3]}` 뿐 | 왜곡 보정 없음 |

**값은 mm→m 정규화 후 나갑니다** (`normalizeBundleMmToM`). 평면 스키마의 `H`에는
`H_floor` 별칭을 붙입니다. 서버 밖으로 나가는 좌표는 전부 미터입니다.

> ⚠️ **2026-08-11 개정**: 위 문단은 이 회신을 쓴 시점(08-10) 기준입니다. 이후
> `QT_CCTV_SERVER_CALIBRATION_FORMAT_20260811.md` 규격에 맞춰 **캘리 번들은 mm
> 그대로 저장·중계**하도록 바꿨습니다. ÷1000은 서버 내부 계산용 사본에서만 하므로
> `POSE`/`PATH.dist_m` 등 좌표 메시지는 종전대로 미터입니다. `H_floor` 별칭과
> `request_id` 처리는 아래 설명 그대로 유효합니다.

**`request_id`는 서버가 직접 찍습니다.** CCTV가 되돌려주기를 기대하지 않습니다 —
평면 스키마에서는 `outMsg["payload"] = bundle`이 payload를 통째로 갈아치우므로
CCTV가 실어 보낸 필드가 그 자리에서 사라집니다.

**저장 시점** (`handleHMatrix` 내부 순서):

```
1. 파싱 + mm→m 정규화
2. 메모리 반영 (calibs_[ch])
3. QT로 중계  ←── Qt가 결과를 받는 시점
4. 전역 슬롯 저장 (calib_latest.json)
5. 로그인 계정 저장 (users.json)   ← 로그인 상태일 때만
```

**즉 중계가 영속 저장보다 먼저입니다.** `LOGIN_OK.calibs[ch]`는 4·5번의 결과를
읽으므로, "Qt가 `H_MATRIX`를 받은 직후 재로그인"하면 아주 짧은 창에서 아직 안
읽힐 수 있습니다. 순서를 뒤집지 않은 것은 저장 실패로 Qt의 종결 응답이 막히면
안 되기 때문입니다. 문제가 되면 알려주세요.

**채널이 어긋난 결과는 세션을 닫지 않습니다.** 진행 중인 세션이 CH2인데 CH1의
`H_MATRIX`가 오면 저장·중계는 하되 `request_id`를 붙이지 않습니다 — 계약 §6의
"다른 `request_id`의 늦은 결과가 현재 대기를 풀면 안 된다"와 같은 이유입니다.

---

## 5. §7-3 — 취소 ACK와 최대 응답 시간

**기존 프로토콜에 없었습니다. 서버가 규격을 제안하고 구현해뒀습니다.**

```
Qt   ──CMD{CALIB_CANCEL, ch, request_id}──▶ Server
Server ──그대로 중계──▶ ROBOT  +  CCTV
ROBOT  ──CALIB_STOPPED{}──▶ Server     ┐ 둘 다 모여야
CCTV   ──CALIB_STOPPED{}──▶ Server     ┘
Server ──CALIB_CANCELLED{ch, request_id}──▶ Qt
```

- **한도**: `params().calib_cancel_ack_ms` = **5000ms** (`config/params.json`으로 조정)
- **한쪽이라도 응답이 없으면** `CALIB_FAIL{reason:"cancel_failed"}` — `msg`에
  어느 쪽이 무응답인지 적어 보냅니다.

🔴 **중계했다는 사실만으로는 절대 `CALIB_CANCELLED`를 보내지 않습니다.** 그것은
"명령을 전달했다"는 뜻이지 "로봇이 섰다"는 뜻이 아닙니다. Qt는 이 응답을 보고
대기를 풀고 조작자를 다시 화면 앞에 앉히는데, 그 순간 로봇이 아직 굴러가고 있으면
사람이 다치는 쪽에 서 있게 됩니다. (`ABORT_DRAW`의 `DRAW_ABORTED`가 중계 직후
ACK인 것과 **의도적으로 다른 규약**입니다.)

⚠️ **현재 ROBOT·CCTV 어느 쪽도 `CALIB_STOPPED`를 보내지 않습니다.** 그래서 오늘
취소를 누르면 5초 뒤 `cancel_failed`로 끝납니다. 이것이 정직한 상태입니다 —
확인하지 못한 것을 확인했다고 하지 않습니다. 두 팀이 구현하면 그대로 동작합니다.

**ESTOP은 이 흐름과 무관하게 항상 처리됩니다** (계약 §3.6 요구사항 그대로).

---

## 6. 서버가 추가한 것 — `timeout`

권장 `reason` 목록에 없는 코드를 하나 추가했습니다.

- **`timeout`** — `params().calib_timeout_ms` = **180000ms(3분)** 안에 종결 응답이
  없으면 서버가 먼저 세션을 접고 Qt에 알립니다.

`internal_error`로 뭉뚱그리지 않은 이유는, 조작자가 "서버가 고장났다"와 "로봇/
카메라가 응답을 안 한다"를 구분해야 다음 행동이 달라지기 때문입니다.
Qt는 `msg`를 표시하시므로 화면상 문제는 없을 것으로 봅니다. **이 코드를 받기
어려우시면 알려주세요 — `internal_error`로 바꾸겠습니다.**

🔴 **이 값은 Qt의 5분 한도보다 반드시 짧아야 합니다.** 길면 Qt는 이미 포기했는데
서버만 `busy`로 남아, 이후 모든 요청이 거절됩니다(조작자 눈에는 "캘리가 영영 안
되는" 상태). `params.hpp`가 300000 이상을 넣으면 경고 후 290000으로 내립니다.

---

## 7. 진행률(§3.3) — 서버는 만들지 않습니다

샘플 개수도 알고리즘 단계도 카메라만 압니다. CCTV가 `CALIB_PROGRESS`를 올리면
서버가 `ch`/`request_id`만 채워 그대로 넘깁니다. 안 올리면 아무것도 안 보내고,
Qt는 설계하신 대로 무한 진행 표시로 폴백합니다. **CCTV 쪽 필수 작업이 아닙니다.**

---

## 8. 메시지 정리 (구현 완료분)

**QT → 서버**
| 메시지 | payload |
|---|---|
| `CMD` | `{"cmd":"CALIB_START","ch":n,"request_id":"...","method":"robot_motion"}` |
| `CMD` | `{"cmd":"CALIB_CANCEL","ch":n,"request_id":"..."}` |

**서버 → QT**
| 메시지 | payload | 시점 |
|---|---|---|
| `CALIB_STARTED` | `{ch, request_id, msg}` | 수락 즉시 |
| `CHANNEL_OK` | `{ch, calib}` | 수락 시 함께 (계약 외 추가) |
| `CALIB_PROGRESS` | `{ch, request_id, progress, stage, msg}` | CCTV가 올릴 때만 |
| `H_MATRIX` | `{ch, request_id, ...번들}` | 성공 = 종결 |
| `CALIB_FAIL` | `{ch, request_id, reason, msg}` | 실패 = 종결 |
| `CALIB_CANCELLED` | `{ch, request_id, msg}` | 취소 확인 = 종결 |
| `CHANNEL_FAIL` | `{reason:"calib_busy"}` | 세션 중 채널 전환 시도 |

**ROBOT/CCTV → 서버** (🔴 아직 미구현)
| 메시지 | payload |
|---|---|
| `CALIB_STOPPED` | `{}` |
| `CALIB_FAIL` | `{reason, msg}` |
| `CALIB_PROGRESS` | `{progress, stage, msg}` (CCTV만) |

---

## 9. 확인한 것 (§6 시험표)

`Server/tools/calib_session_test.cpp` — QT·ROBOT·CCTV 세 role로 동시에 붙어
계약 §6 표를 돌립니다. **21/21 통과, 3회 반복 재현.**

```bash
cd Server && make calib_session_test && ./server 9100 &
tools/calib_session_test 127.0.0.1 9100
```

| 시험 | 결과 |
|---|---|
| 정상 왕복 + `ch`/`request_id` 보존 (ROBOT·CCTV 양쪽) | ✅ |
| 같은 `request_id` 재요청 → 상태 재전송(멱등) | ✅ |
| 다른 `request_id` 재요청 → `busy` | ✅ |
| 세션 중 `SELECT_CHANNEL` → `CHANNEL_FAIL{calib_busy}` | ✅ |
| `CALIB_PROGRESS`에 서버가 `ch`/`request_id` 주입 | ✅ |
| **다른 채널의 늦은 결과가 대기를 풀지 않음** | ✅ |
| 해당 채널 결과 → `request_id` 붙어 종결, 세션 실제로 닫힘 | ✅ |
| **중계만으로 `CALIB_CANCELLED`를 보내지 않음** | ✅ |
| 정지 ACK 없음 → 5초 뒤 `cancel_failed` | ✅ |
| ROBOT만 ACK → 아직 대기, CCTV까지 오면 `CALIB_CANCELLED` | ✅ |
| 범위 밖 채널 / QT의 `ch` 생략 → `invalid_channel` | ✅ |
| CCTV `CALIB_FAIL` → 사유 그대로 + `request_id` 종결 | ✅ |
| 세션 중 로봇 이탈 → 즉시 `robot_offline` (타임아웃 안 기다림) | ✅ |
| 로봇 없을 때 시작 → 즉시 `robot_offline` (허공 전송 안 함) | ✅ |

미검증: 도색 중 시작 → `busy` (도면·pose 준비가 필요해 수동 확인 예정).

### 구현 중 같이 고친 것 두 가지

**(a) 서버 감시 타이머가 상대의 생존에 의존하고 있었습니다.**
타임아웃 판정이 `onMessage` 꼬리에서만 돌아서, "로봇이 TCP는 붙잡은 채 응답만
멈추는" 경우 — 정확히 타임아웃이 필요한 그 경우 — 판정 자체가 돌지 않았습니다.
소켓이 살아 있으니 접속 해제 통지도 안 옵니다. `Router::tick()`(200ms 주기
스레드)을 추가했습니다. POS 두절 HOLD와 판정 대기 창도 같이 튼튼해집니다.

**(b) 콘솔 `calib` 명령이 Router를 우회했습니다.**
`sendTo`로 직접 쏘고 있어서 서버가 모르는 세션이 시작되고, 그 결과 `H_MATRIX`가
아무도 기다리지 않는 종결 응답으로 Qt에 떨어졌습니다. 이제 ADMIN 개시로
Router를 거칩니다 (`calib` 또는 `calib 3`).

---

## 10. Qt팀에 필요한 회신

1. **`timeout` reason 코드**를 받으실 수 있는지 (§6). 안 되면 `internal_error`로 바꿉니다.
2. **`CHANNEL_OK`를 `CALIB_STARTED`와 함께 보내는 것**이 §5의 "계산 중 선택 채널만
   크게 표시"와 충돌하지 않는지 (§1-4). 충돌하면 빼겠습니다.
3. **`H_MATRIX` 스키마 세 형태 중 Qt가 실제로 받는 것**이 무엇인지 (§4).
   서버는 CCTV가 보낸 대로 넘기므로, Qt가 특정 형태만 처리한다면 CCTV팀에
   그 형태로 고정해달라고 같이 요청해야 합니다.

**나머지는 서버 쪽에서 물을 것이 없습니다.** CCTV(§3-a)와 로봇(§3-b)만 되면
통합 시험에 들어갈 수 있습니다.
