# Qt팀 전달 사항 (2026-08-07) — 카메라 주소를 서버가 내려주는 구조로

**근거**: 서버 `feature/server` 구현 ↔ `Client/src/` 코드 대조
**서버 상태**: 아래 "서버가 이미 한 것" 3가지 구현 완료 (빌드 통과, 단위 확인 완료)
**대상 코드**: `Client/src/backend.h`, `Client/src/backend.cpp`

> **결론 한 줄**: 서버가 `LOGIN_OK.cam_ip`로 **현장 카메라 주소를 전역으로** 내려주게
> 바꿨다. 그런데 Qt의 4채널 직결 URL은 **IP가 소스에 하드코딩**돼 있어서
> (`backend.h:508`), 서버가 뭘 내려주든 **4채널 모드에서는 안 쓰인다.**
> 카메라를 옮기거나 IP가 바뀌면 지금도 **클라이언트를 재빌드해야 한다.**
>
> ⚠️ **Q-1 하나만 해도 목표는 달성된다.** Q-2는 운영 편의, Q-3은 보안 정리다.

---

## 0. 요약표

| # | 항목 | 우선도 | 위치 | 한 줄 |
|---|---|---|---|---|
| **Q-1** | 4채널 직결 템플릿에 `{ip}` 치환 | **P0** | `backend.h:508`, `backend.cpp:1918` | 서버가 준 `cam_ip`가 4채널에서 무시된다 |
| Q-2 | 중계 OFF 신호로 `relayBase` 자동 해제 | P1 | `backend.cpp:205` | PC마다 손으로 [비우기]를 눌러야 직결로 넘어간다 |
| Q-3 | RTSP 계정·비번 하드코딩 정리 | P2 | `backend.h:479,484,508` | 비번이 소스와 화면 로그에 그대로 있다 |
| Q-4 | (확인만) 서브스트림 `/chNs` 정책 | — | `backend.cpp:1924` | 이미 맞게 돼 있는지 확인만 부탁 |

---

## 1. 서버가 이미 한 것 (Qt는 받기만 하면 된다)

### 1-1. `LOGIN_OK.cam_ip` — 계정 값 → 없으면 **전역 값**

카메라는 현장에 **한 대**뿐인데(`user_store.hpp` 맨 위 "한 현장 = 카메라 1대 가정")
지금까지 `cam_ip`는 **계정마다** `users.json`에 따로 박혀 있었다. 그래서 새 계정으로
로그인하면 카메라 주소가 비어 있고, 카메라를 옮기면 전 계정을 순회하며 고쳐야 했다.

이제 캘리브레이션(`calib_latest.json`)과 **똑같은 규약**으로 전역 슬롯을 둔다:

```
config/camera.json   ← 전역 카메라 IP (계정과 무관, 현장 설비 정보)
    ↓ 계정에 cam_ip 가 없으면 이 값이 내려간다
LOGIN_OK.payload.cam_ip
```

| 상황 | `LOGIN_OK.cam_ip` |
|---|---|
| 계정에 `cam_ip` 있음 | **계정 값** (전역이 덮지 않는다 — 기존 현장 보호) |
| 계정에 없음 / 새 계정 | **전역 값** (`config/camera.json`) |
| 둘 다 없음 | `null` |

**Qt가 할 일 없음.** 필드 이름·타입 그대로다. 값이 더 자주 채워져서 올 뿐이다.

### 1-2. `SET_CAM_IP` — 로그인 없이도 저장된다

전역 슬롯에 **항상** 쓰고, 로그인 중이면 그 계정에도 같이 쓴다
(`H_MATRIX`가 전역+계정에 쓰는 것과 같은 규약). 예전에는 로그인이 없으면
`SET_CAM_IP_FAIL {"reason":"로그인 필요"}`로 거절했는데, 이제 성공한다.

⚠️ **의미가 하나 바뀐다**: 한 사용자가 카메라 IP를 바꾸면 **전역이 바뀌므로
다른 사용자에게도 반영된다.** 카메라가 한 대라 의도한 동작이지만, Qt UI에
"이건 현장 전체 설정"이라는 안내가 필요하면 넣어주면 좋겠다.

### 1-3. `LOGIN_OK.stream` — `enabled` 플래그로 껐다

`config/stream.json`에 `"enabled": false`를 넣으면 `base`가 적혀 있어도
`stream` 필드를 **안 싣는다.** 지금 현장은 이 상태다 → **직결 모드.**

파일을 지우지 않고 플래그로 끈 이유는 되돌릴 때 주소를 잃지 않기 위해서다.
중계로 복귀하려면 **두 가지가 모두** 필요하다:

```bash
# 1) 서버가 주소를 알려주게: config/stream.json 의 enabled 를 true 로
# 2) 중계 프로세스를 띄우기
cd Server/relay && ./start.sh -d
```

---

## 2. ✅ 이미 맞는 것 — 건드리지 말 것

| 항목 | 근거 |
|---|---|
| **중계가 있으면 중계가 이긴다** | `backend.cpp:1920`. 이 우선순위 자체는 맞다. Q-2는 "중계를 끄는 신호"를 더하자는 것이지 순서를 뒤집자는 게 아니다 |
| **`m_directRtspUrl` 분리 보관** | `backend.h:489`. 중계 주소가 단일채널 주소를 덮어쓰지 않게 막는 장치다. 직결 복귀에 꼭 필요하다 |
| **`channelMode()`가 "주소를 만들 방법"으로 판정** | `backend.h:213`. 중계든 직결 템플릿이든 하나만 있으면 4채널이 켜진다. 우리 구조(중계 OFF + 직결)에 **딱 맞는 조건이다.** 유지 |
| **`applyCamIp()`의 IP 형식 검증** | `backend.cpp:2445`. 서버는 검증을 안 하므로 Qt가 하는 게 맞다 |
| **`streamInfoReceived`에서 빈 base 무시** | `backend.cpp:207`. "서버가 안 준다"와 "지우라"를 구분하는 현재 규약. Q-2는 이걸 깨지 말고 **별도 신호**로 풀자는 제안이다 |

### ⚠️ 주석 하나가 낡았다 (코드는 맞고 주석이 틀렸다)

`backend.h:493`은 아직 이렇게 적혀 있다:

```
// 🔴 비어 있으면 4채널 기능 전체가 꺼지고 위 m_rtspUrl 직결 경로만 돈다.
QString m_relayBase;
```

**지금은 사실이 아니다.** `channelMode()`(`backend.h:213`)가
`!m_relayBase.isEmpty() || !m_channelUrlTemplate.isEmpty()`로 바뀌었고,
`m_channelUrlTemplate`에는 하드코딩 기본값이 있어서(`backend.h:508`)
**`relayBase`를 비워도 4채널은 그대로 켜진 채 "직결 4채널"로 넘어간다.**

동작 자체는 우리가 원하는 그대로다 — 고칠 것은 **주석뿐**이다. 다만 이 주석을
믿고 "중계 주소를 비우면 단일채널로 돌아간다"고 판단하면 틀리므로, Q-1 작업할 때
같이 정리해주면 좋겠다. (단일채널로 완전히 돌아가려면 `m_channelUrlTemplate`도
비워야 한다.)

---

## Q-1. 🔴 P0 — 4채널 직결 템플릿에 `{ip}` 치환

### 지금 무슨 일이 일어나는가

```cpp
// backend.h:508 — IP 가 박혀 있다
QString m_channelUrlTemplate =
    QStringLiteral("rtsp://admin:5hanwha!@192.168.0.13:554/{ch0}/H.264/media.smp");
```

```cpp
// backend.cpp:1918 channelUrl() — 직결 분기는 {ch0}/{ch} 만 치환한다
QString url = m_channelUrlTemplate;
url.replace(QStringLiteral("{ch0}"), QString::number(ch - 1));
url.replace(QStringLiteral("{ch}"),  QString::number(ch));
return url;                      // ← cam_ip 가 끼어들 자리가 없다
```

한편 서버가 준 `cam_ip`를 쓰는 곳은 **단일채널 경로뿐**이다:

```cpp
// backend.cpp:2426 setCamIp() — m_rtspTemplate({ip}) 에만 치환한다
QString url = m_rtspTemplate;    // "rtsp://…@{ip}:554/FullHD/media.smp"
url.replace(QStringLiteral("{ip}"), clean);
setRtsp(url);
```

즉 **4채널 모드에서 `cam_ip`는 어디에도 안 쓰인다.** 서버가 전역 주소를 아무리
잘 내려줘도 화면은 `192.168.0.13`으로만 붙는다.

### 부탁드리는 것

`m_channelUrlTemplate`에 `{ip}`를 넣고, `channelUrl()`의 직결 분기에서
**서버가 준 `cam_ip`(`m_camIp`)로 치환**해달라.

```cpp
// backend.h — IP 를 빼고 {ip} 로
QString m_channelUrlTemplate =
    QStringLiteral("rtsp://admin:5hanwha!@{ip}:554/{ch0}/H.264/media.smp");

// backend.cpp channelUrl() 직결 분기
QString url = m_channelUrlTemplate;
url.replace(QStringLiteral("{ip}"),  m_camIp);   // ← LOGIN_OK.cam_ip
url.replace(QStringLiteral("{ch0}"), QString::number(ch - 1));
url.replace(QStringLiteral("{ch}"),  QString::number(ch));
```

**주의할 점 2개:**

1. **`m_camIp`가 비었을 때 무엇을 할지 정해야 한다.** 빈 문자열을 그대로 치환하면
   `rtsp://admin:pw@:554/0/…` 같은 깨진 URL로 접속을 시도한다. 한화 카메라는
   **틀린 URL로 반복 접속하면 계정을 잠그므로**(`RTSP/1.0 490 Account Blocked`,
   `relay/README.md`) **아예 열지 말고** "카메라 IP 미설정" 안내를 띄우는 쪽을 권한다.
2. **경로(`/{ch0}/H.264/media.smp`)는 그대로 두는 게 맞다.** 이건 2026-08-04에
   원시 DESCRIBE로 0~3 전부 200 OK를 확인한 값이다(`backend.h:500` 주석).
   서버가 내려주는 건 **IP뿐**이고 프로파일 경로는 Qt가 계속 쥔다.

### 🔴 같이 고쳐야 하는 것 — 재로그인 때 없는 프로파일을 두드린다

Q-1과 **같은 뿌리**라 따로 떼지 말고 함께 봐달라. 로그인 성공 시:

```cpp
// backend.cpp:455 — cam_ip 가 있으면 무조건 setCamIp()
if (!camIp.trimmed().isEmpty())
    setCamIp(camIp.trimmed());
```

`setCamIp()`는 **단일채널 템플릿**으로 URL을 만들어 `setRtsp()`를 부른다:

```
rtsp://admin:5hanwha!@192.168.0.13:554/FullHD/media.smp
                                       ^^^^^^^ PNM 에는 없는 프로파일
```

**첫 로그인은 안전하다** — 4채널 모드에서는 `enterInitialView()`가 `startPreviews()`
쪽으로 가서(`backend.cpp:2248`) 메인 워커가 없고, `setRtsp()`가
`if (!m_worker) return;`(`backend.cpp:2401`)으로 빠져 URL만 저장된다.
문제는 **채널 작업 중 로그아웃 → 재로그인**이다. 이때는 워커가 살아 있어서
위 URL을 **실제로 연다.**

한화 카메라는 없는 경로를 반복해서 두드리면 **계정을 잠근다**
(`RTSP/1.0 490 Account Blocked`). 잠기면 카메라 웹 UI에서 직접 풀어야 한다.

> ⚠️ 이건 원래 있던 문제지만(계정 `cam_ip`가 `.13`인 사용자는 지금도 해당),
> 서버가 전역 `cam_ip`를 내려주기 시작하면 **`cam_ip`가 비어 있던 계정까지**
> 이 경로를 타게 되어 노출이 넓어진다.

제안: `setCamIp()`에서 **4채널 모드면 단일채널 URL로 `setRtsp()`를 하지 말고**
`m_camIp`만 갱신하고 채널 URL을 다시 조립하게 갈라달라. (`channelMode()`가
이미 있으니 분기 한 줄이면 된다.)

### 확인 방법

1. `config/camera.json`의 `cam_ip`를 바꾸고 Qt를 **재로그인**만 하면 4채널이
   새 주소로 붙어야 한다. **재빌드가 필요 없어야 통과다.**
2. 채널 작업 화면에서 **로그아웃 → 재로그인**했을 때 로그에
   `/FullHD/`가 들어간 RTSP 접속 시도가 **안 나와야** 통과다.

---

## Q-2. P1 — 중계 OFF 신호로 `relayBase` 자동 해제

### 문제

서버가 중계를 껐는데(`enabled:false`) Qt는 그걸 알 방법이 없다. 현재 규약은
"서버가 `stream`을 안 보내면 **아무것도 하지 않는다**"이고(`backend.cpp:207`),
`relayBase`는 QSettings에 남아 있다. 그리고 `channelUrl()`은 **중계가 이긴다.**

결과: **중계 프로세스는 죽었는데 Qt는 계속 `rtsp://…:8554/ch1`로 접속을 시도한다.**
직결로 넘기려면 **PC마다 사람이 설정 화면에서 [비우기]를 눌러야 한다.**

빈 문자열로도 안 지워진다 — `if (base.trimmed().isEmpty()) return;`이 먼저 걸린다.

### 제안 (서버는 Qt가 정하는 대로 맞추겠다)

셋 중 하나면 된다. 서버 쪽 구현은 어느 쪽이든 한 줄이다.

| 안 | 내용 | 장단 |
|---|---|---|
| **A** | `stream:{"enabled":false}`를 **명시적으로 실어 보낸다** → Qt가 받으면 `relayBase` 해제 | 의도가 명확. "안 보냄"(모름)과 "끔"(직결)이 구분된다. **권장** |
| B | `stream:{"base":""}` 를 보내고 Qt가 빈 base를 "해제"로 해석 | 현재 규약(`빈 base = 무시`)을 뒤집어야 해서 하위호환이 깨진다 |
| C | Qt가 중계 접속 N회 실패하면 자동으로 직결 폴백 | 서버 변경 0. 다만 실패까지 시간이 걸리고, 카메라 계정 잠김 위험과 얽힌다 |

A로 가면 서버는 `stream` 필드를 `{"enabled":false}`로 실어 보내고, Qt는:

```cpp
// enabled 가 명시적으로 false 면 저장된 중계 주소를 지우고 직결로
if (obj.contains("enabled") && !obj["enabled"].toBool()) { setRelayBase(""); return; }
```

**어느 안으로 갈지만 알려주면 서버 쪽은 맞춰서 바로 반영하겠다.**

---

## Q-3. P2 — RTSP 계정·비번 하드코딩

```cpp
backend.h:479   "rtsp://admin:5hanwha!@{ip}:554/FullHD/media.smp"
backend.h:484   "rtsp://admin:5hanwha!@192.168.0.12:554/FullHD/media.smp"
backend.h:508   "rtsp://admin:5hanwha!@192.168.0.13:554/{ch0}/H.264/media.smp"
```

카메라 비밀번호가 **소스에 3곳** 박혀 있고, 리포지토리는 공개 GitHub에 있다.
`maskRtspPassword()`(`backend.cpp:1938`)로 화면·로그에서 가리고는 있지만
소스 자체는 그대로다.

지금 당장 급한 건 아니지만(폐쇄망 전제), **비번을 바꾸면 재빌드가 필요하다**는
운영 문제가 Q-1과 같은 성격이다. 원하면 서버가 `LOGIN_OK`로 계정까지 내려주도록
확장할 수 있다 — **필요 여부만 알려달라.** 서버는 이미 `config/camera.json`을
오브젝트로 만들어 뒀으니 키만 늘리면 된다:

```json
{ "cam_ip": "192.168.0.13", "cam_user": "admin", "cam_pass": "…" }
```

⚠️ 다만 이러면 **비밀번호가 TLS 세션으로 흐른다.** 서버↔Qt는 TLS라 전송 자체는
안전하지만, Qt가 그 값을 QSettings(평문)에 저장하면 의미가 없어진다. 넘길지 여부는
Qt팀 판단에 맡긴다.

---

## Q-4. (확인만) 서브스트림 `/chNs`

`backend.cpp:1924` 주석대로 **이 카메라에는 저해상도 서브 프로파일이 없어서**
직결 모드에서는 `sub` 여부와 무관하게 같은 URL이 나간다 — 즉 2x2 미리보기 4장이
전부 풀해상도를 디코딩한다. **코드는 이미 그렇게 돼 있으니 바꿀 것은 없다.**

다만 성능이 문제가 되면 알려달라. 카메라 웹 UI에서 채널당 저해상도 프로파일을
추가하면 되고, 그때는 `channelUrl()`의 직결 분기에서 `sub`일 때 다른 템플릿을
쓰도록 갈라주면 된다.

> 참고: 중계 모드에서는 `ch1s`~`ch4s`가 `ch1`~`ch4`로 **redirect**되게 해뒀다
> (`relay/mediamtx.yml`). 예전에는 이 경로들이 `source: publisher`로 남아 있어서
> MediaMTX가 **기동 자체를 거부**했고(`'sourceOnDemand' is useless when source is
> 'publisher'`), 그 바람에 `ch1`~`ch4`까지 통째로 안 떴다. 2026-08-07 수정됨.

---

## 3. 테스트 순서

각 단계마다 **직결/중계 왕복이 되는지**까지 봐야 한다.

| 단계 | 확인 |
|---|---|
| 1 | `config/camera.json`에 IP를 넣고 로그인 → 서버 로그에 `카메라 IP 192.168.0.13` |
| 2 | Q-1 적용 후, 설정에서 중계 주소 [비우기] → 4채널이 **직결**로 붙는다 |
| 3 | `config/camera.json`의 IP만 바꾸고 **재로그인** → 재빌드 없이 새 주소로 붙는다 |
| 4 | 계정에 `cam_ip`가 박힌 기존 계정(`test` 등)으로 로그인 → **계정 값이 이긴다** |
| 5 | 중계 복귀: `stream.json` `enabled:true` + `relay/start.sh -d` → 중계로 돌아간다 |
| 6 | 회귀: 단일채널 PNO(`192.168.0.12`)로 로그인 → 기존 동작 그대로 |

---

## 4. 되돌리기

서버 쪽은 전부 **파일 한 줄**로 되돌아간다 (코드 재배포 불필요):

```bash
# 전역 카메라 IP 무시하고 예전처럼 계정 값만 쓰기
rm Server/config/camera.json        # 계정에 있는 cam_ip 는 그대로 살아있다

# 중계로 복귀
# config/stream.json 의 "enabled" 를 true 로 바꾸고
cd Server/relay && ./start.sh -d
```

Qt 쪽은 설정 화면의 **중계 주소 칸**이 그대로 스위치다 — 채우면 중계, 비우면 직결.
