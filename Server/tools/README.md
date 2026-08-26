# 테스트 도구

> **테스트 어떻게 돌리지?** 에 답하는 문서.
> 관련: [../README.md](../README.md) · [docs/PROTOCOL](../docs/PROTOCOL.md) · [docs/ARCHITECTURE](../docs/ARCHITECTURE.md)

로봇·카메라·Qt 없이 서버를 검증할 수 있게 만든 도구 모음이다.
전부 `Server/` 에서 `make <이름>` 으로 빌드하고 `./tools/<이름>` 으로 실행한다.

> 🔴 **테스트 서버는 반드시 9000 이 아닌 포트로 띄울 것.**
> 같은 role 로 새로 접속하면 기존 연결이 끊기므로, 9000 에 띄우면 현장 카메라가 쫓겨난다.
>
> ```bash
> ./server 9100
> ```

---

## 1. 오프라인 단위 테스트 — 서버를 띄울 필요도 없다

TLS 도 라우터도 없이 계산만 검사한다. 제일 빠르고 제일 자주 돌릴 것.

| 도구 | 검사 대상 |
|---|---|
| `calib_unit_test` | 캘리 번들 파싱, 좌표계, mm↔m 변환, 구형식 흡수 |
| `odo_calib_test` | 오도메트리 11-op 시퀀스, 9점 좌표표, 마커 오프셋 보정 |
| `offset_feedback_test` | 펜 오프셋·펜 두께·도색 언더슛 보정 |
| `calib_channel_test` | 채널별 번들 저장, 예전 전역 슬롯 호환 |

```bash
make calib_unit_test odo_calib_test offset_feedback_test calib_channel_test
for t in calib_unit_test odo_calib_test offset_feedback_test calib_channel_test; do
    ./tools/$t | tail -1
done
```

전부 `전부 통과 (0 fail)` 로 끝나야 한다.
`offset_feedback_test` 는 생성된 op 시퀀스를 눈으로 볼 수 있게 표로 찍어준다 —
경로 생성을 고쳤을 때 **무엇이 달라졌는지 보는 용도**로도 쓴다.

## 2. 세션 흐름 테스트 — 서버가 떠 있어야 한다

TLS 로 붙어서 실제 메시지를 주고받는다. 인자는 `<서버IP> <포트> <인증서>` 순.

| 도구 | 검사 대상 |
|---|---|
| `zone_event_test` | `ZONE_EVENT` 가 CCTV → 서버 → ROBOT 으로 중계되는지, 잘못된 payload 가 걸러지는지 |
| `calib_session_test` | 정적 앵커 캘리 세션 계약 (종결 응답 보장, 취소, 타임아웃) |
| `odo_session_test` | 오도메트리 캘리 세션 (QT/ROBOT/CCTV/ADMIN 네 role 로 동시 접속) |

```bash
./server 9100 &
make zone_event_test && ./tools/zone_event_test 127.0.0.1 9100 certs/server.crt
```

> ⚠️ `calib_session_test` 는 끝날 때 활성 채널을 1 로 되돌린다.
> 안 그러면 뒤이어 붙는 `robot_sim`(항상 CH1 POS)의 관측이 전부 버려져
> **피드백 회귀와 구분되지 않는 증상**이 나온다 (실제로 한 번 오진했다).

## 3. 시뮬레이터 — 없는 장비 흉내내기

| 도구 | 흉내내는 것 | 쓰는 상황 |
|---|---|---|
| `robot_sim` | 로봇 | 로봇 없이 주행 왕복을 돌려본다 |
| `qt_sim` | Qt 관제 | Qt 없이 로그인·도면 전송을 해본다 |
| `draw_test` | Qt 도면 | 사각형·원호 도면을 만들어 `START_DRAW` 까지 태운다 |
| `cctv_pose` | 카메라 `POS` | 실시간 카메라 없이 pose 를 먹인다 |
| `drive_test` | 수동 조작 | 조이스틱 명령을 쏴본다 |
| `path_test` | — | 서버 RPi ↔ 로봇 RPi 경로 생성 종단 확인 |
| `tap_dump` | ADMIN | 서버 트래픽을 그대로 받아 찍는다 |

```bash
make sim && ./tools/robot_sim 127.0.0.1 --port 9100 &
./tools/draw_test 127.0.0.1 --port 9100                 # 사각형
./tools/draw_test 127.0.0.1 --port 9100 --arc 0.5       # 반지름 0.5m 원호
./tools/draw_test 127.0.0.1 --port 9100 --nudge         # 수동조작 래치 회귀
```

`--nudge` 는 "도면 → 위치 조정 → 시작" 순서에서 작업 전체가 개루프로 돌던
버그의 회귀 테스트다. 조이스틱을 한 번 쓰면 자동 판정이 멈추는데,
그 해제가 `BLUEPRINT` 에서만 일어나던 시절이 있었다.

## 4. 계정 만들기

```bash
python3 tools/seed_user.py <id> <pw>
```

`config/users.json` 에 계정을 추가한다. Qt 로그인 테스트에 필요하다.

## 5. 손으로 붙어보기

```bash
openssl s_client -connect 127.0.0.1:9100 -CAfile certs/server.crt -quiet
```

접속 후 JSON 을 한 줄씩 입력한다. **첫 줄은 반드시 `HELLO`** 이고 10초 안에 보내야 한다.

```json
{"type":"HELLO","seq":1,"payload":{"role":"QT"}}
{"type":"LOGIN","seq":2,"payload":{"id":"user1","pw":"..."}}
```

## 6. 자주 걸리는 것

| 증상 | 원인 |
|---|---|
| 접속하자마자 끊긴다 | `HELLO` 를 10초 안에 안 보냈다 |
| 인증서 검증 실패 | `certs/server.crt` 를 생성할 때 **서버 IP** 를 넣지 않았다 |
| 붙었는데 아무 반응이 없다 | 같은 role 로 다른 클라이언트가 이미 붙어 있어 밀려났다 |
| `POS` 를 보내는데 pose 가 안 잡힌다 | 활성 채널이 아닌 채널의 `POS` 는 전부 버려진다 |
| 캘리 테스트 뒤부터 피드백이 안 온다 | 활성 채널이 안 돌아왔다 (§2 경고 참고) |
