# 서버팀 회신 — Qt팀 2026-08-07 회신에 대한 답

**회신일**: 2026-08-07
**서버 기준**: `feature/server` (main `54530fd` = Qt PR #39 병합분 포함)
**대상 문서**: Qt팀 회신 `QT_ACTION_ITEMS_REPLY_20260807.md`

---

## 1. 요청하신 한 가지 — 반영 완료

> Q-2 제안 A 채택 확인:
> stream.json enabled=false일 때 LOGIN_OK.payload.stream={"enabled":false}를
> 명시적으로 전송한다.

**✅ 그대로 반영했습니다.** 지적하신 근거("생략은 구버전 서버·설정 미확정과
구분되지 않으므로 Qt가 저장된 중계 주소를 임의로 지우면 안 된다")가 정확합니다.
서버가 상태를 **세 가지로 구분해서** 응답하도록 고쳤습니다.

| `config/stream.json` | `LOGIN_OK.stream` | 의도 |
|---|---|---|
| 파일 없음 / **파싱 실패** | **필드 없음** | "서버가 모른다" → Qt 설정 유지 |
| `enabled: false` | `{"enabled":false}` | "중계 끔" → Qt가 `relayBase` 해제 |
| `enabled: true` + `base` | `{"enabled":true,"base":…,"channels":…}` | 그 주소 사용 |

구현: `StreamCfg::shouldSend()` / `StreamCfg::toJson()` (`Server/src/stream_cfg.hpp`),
`Router::handleLogin()` (`Server/src/router.cpp`).

### 설계 결정 두 가지 (회신에 없던 부분이라 서버가 정했습니다)

**(a) `enabled:false`일 때 `base`를 같이 보내지 않습니다.**
"끄라"는 지시에 주소가 붙으면 Qt가 어느 쪽을 따라야 할지 애매해집니다.
그래서 payload는 정확히 `{"enabled":false}` 하나뿐입니다.
회신에 적어주신 예시와 동일합니다.

**(b) 파일 파싱에 실패하면 "끔"이 아니라 "모름"으로 처리합니다.**
깨진 JSON 때문에 `enabled:false`를 잘못 내려보내면 **Qt의 멀쩡한 중계 주소를
지워버립니다.** 파싱 실패는 `stream` 필드 자체를 생략합니다.
→ 이 경우 Qt는 "서버가 모름"으로 보고 QSettings를 유지하므로 안전한 쪽으로 실패합니다.

**하위호환**: 켤 때도 `"enabled":true`를 명시적으로 싣지만, 회신에 적어주신 대로
Qt가 키 없는 경우를 `true`로 읽으므로 옛 서버와도 문제없습니다. 반대로 `enabled`
키가 없는 **옛 `stream.json` 파일**도 서버가 `true`로 읽습니다.

### 확인한 것

6가지 상태를 단위로 확인했습니다 (파일없음 / `enabled:false` / `enabled:true`+base /
깨진 파일 / `enabled` 키 없음 / `enabled:true`인데 `base` 빈 값) — **전부 통과**.

서버 로그로 어느 상태인지 바로 구분됩니다:

```
중계 rtsp://192.168.0.2:8554        ← enabled=true
중계 끔 - QT에 해제 지시(직결)        ← enabled=false
중계 설정 없음 - QT 설정값 유지        ← 파일 없음/깨짐
```

---

## 2. Qt 반영 내용 확인 결과

PR #39(`54530fd`)를 병합해서 코드로 대조했습니다. **회신 내용과 전부 일치합니다.**

| 회신 항목 | 서버팀 확인 |
|---|---|
| `{ip}` 치환 | ✅ `m_channelUrlTemplate`이 `{ip}` 템플릿, `channelUrl()`이 `m_camIp`로 치환 |
| IP 미설정 안전 처리 | ✅ `channelMode()`가 `{ip}` 템플릿일 때 `cam_ip`까지 요구 — 깨진 URL을 아예 안 만듦 |
| 재로그인 `/FullHD/` 차단 | ✅ `setCamIp()`가 채널 템플릿 활성 시 단일채널 경로를 안 탐 |
| QSettings 마이그레이션 | ✅ 정확히 옛 기본값만 `{ip}` 템플릿으로 이관 |
| `stream.enabled` 3-상태 | ✅ `streamInfoReceived(bool enabled, …)` → `enabled==false`면 `setRelayBase("")` |

**특히 `channelMode()`에 `cam_ip` 조건을 넣으신 것이 좋았습니다.** 서버 요청서에는
"빈 문자열을 치환하지 말라"까지만 적었는데, 4채널 모드 자체를 안 켜는 쪽이
더 확실합니다 — 미리보기 워커가 시작조차 안 되니 계정 잠김 경로가 원천 차단됩니다.

---

## 3. `ABORT_DRAW` — 서버 계약 확인

회신에 적어주신 서버 동작이 코드와 일치합니다. 확인한 내용:

- `Router::abortDraw()`는 `drawRequested_ = false` 후 `clearPath()`를 호출합니다.
  실제 상태 정리(경로·커서·boundary 래치)는 **`clearPath()`가 전부** 합니다 —
  v2에 새 상태가 추가돼도 한 곳만 고치면 되도록 모아둔 구조입니다.
- 로봇 중계와 `DRAW_ABORTED{was_active}` 회신 모두 있습니다.
- `manualMode_`는 일부러 안 건드립니다 — `planActive_`가 꺼졌으므로 수동 CMD가
  이미 통과하고, 취소 직후 조이스틱으로 로봇을 빼내는 것이 자연스러운 흐름입니다.

⚠️ 회신에 적어주신 주의사항이 정확합니다 — **`DRAW_ABORTED`는 서버가 로봇에
중계한 직후의 ACK이지 물리적 정지 완료 ACK가 아닙니다.** 통합 시험에서
로봇 로그(속도 0, 노즐 up)로 직접 확인해야 합니다.

---

## 4. Q-3 — 서버도 같은 판단입니다

"서버가 `LOGIN_OK`로 비번을 보내는 것만으로는 해결되지 않는다"에 동의합니다.
Qt가 QSettings에 저장하면 로컬 디스크에 평문으로 남는 것이 맞습니다.

**정책이 정해지기 전까지 서버는 payload에 자격 증명 필드를 추가하지 않겠습니다.**
`config/camera.json`은 이미 오브젝트라 정책이 정해지면 키만 늘리면 됩니다.

한 가지 덧붙이면 — **저장소가 공개라 기존 비밀번호는 이미 비밀이 아닙니다.**
정책 논의와 별개로 **운영 카메라 자격 증명 교체**는 먼저 하는 게 좋겠습니다.
(교체하면 소스의 하드코딩 값이 자동으로 무력화되므로 급한 불은 꺼집니다.)

---

## 5. Q-4 — 현행 유지에 동의

직결 모드에서 `sub`가 메인과 같은 URL을 쓰는 것은 이 카메라에 저해상도 프로파일이
없다는 전제와 일치합니다. 2x2가 풀해상도를 디코딩하는 CPU 부담도 인지하고 있습니다.

카메라에 서브 프로파일을 만들게 되면 **경로를 짐작하지 말고**
`Server/relay/probe_onvif.py`로 카메라가 직접 알려주는 값을 받아 쓰시기 바랍니다
(틀린 경로 반복 접속 → 계정 잠김).

---

## 6. 서버 쪽에서 Qt팀에 필요한 회신

**없습니다.** Q-2가 닫혔으므로 계약 변경 없이 통합 시험에 들어갈 수 있습니다.

Q-3만 **자격 증명 보관 정책**을 정해서 알려주시면 서버가 맞추겠습니다
(회신에 적어주신 1·2·3안 중 어느 것이든).

---

## 7. 통합 시험 전 서버 쪽 준비 상태

| 항목 | 현재 값 | 비고 |
|---|---|---|
| `config/camera.json` | `{"cam_ip": "192.168.0.13"}` | PNM 4채널. gitignore 대상이라 **PC마다 따로 만들어야 함** |
| `config/stream.json` | `{"enabled": false, "base": "rtsp://192.168.0.2:8554", "channels": 4}` | 직결 모드. `base`는 복귀용으로 보존 |
| MediaMTX (중계) | **정지 상태** | 복귀하려면 `relay/start.sh -d` |
| 서버 프로세스 | 재기동 필요 | 새 바이너리로 띄워야 위 계약이 적용됨 |

⚠️ **`camera.json`·`stream.json`은 `.gitignore` 대상입니다.** pull만으로는 안 생기니
통합 시험하는 PC/파이에서 직접 만들어야 합니다. `stream.json`은
`config/stream.json.example`을 복사하면 됩니다.

통합 시험 항목은 [`QT_ACTION_ITEMS_20260807.md` §4](QT_ACTION_ITEMS_20260807.md)에
카메라 직결 / 중계 왕복 / 작업 취소 3그룹으로 정리해뒀습니다.
