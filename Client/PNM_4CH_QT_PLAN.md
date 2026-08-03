# PNM 4채널 Qt 작업 계획서

> 이 문서는 **Windows PC 에서 Qt 클라이언트를 작업할 사람(또는 Claude Code)** 을 위한
> 것이다. 서버(라즈베리파이) 쪽 준비는 끝나 있고, 남은 것은 Qt 다.
>
> 브랜치: `feature/pnm` · 작성 2026-08-03

---

## 0. 30초 요약

카메라가 1채널(PNO-A9081R) → 4채널(PNM-C16083RVQ)로 바뀐다. 서버에 RTSP 중계를
세워놨으므로 Qt 는 **서버 한 곳만 보면 4채널을 다 받을 수 있다**.

만들 것:

```
[2x2 미리보기 4채널]  --채널 클릭-->  [그 채널 1개를 크게 + 마커검출 + 그리기]
   저해상도, 가볍게                      기존 파이프라인 그대로 재사용
```

핵심 설계 원칙 하나: **선택된 채널은 기존 코드 경로를 그대로 탄다.** 채널을 고르는
행위 = 기존 `Backend::setRtsp()` 를 새 URL 로 부르는 것. 캘리브레이션·ArUco·그리기·
도면 변환은 **한 줄도 안 건드린다**.

---

## 1. 전제 — 서버는 이미 준비돼 있다

### 1-1. 중계 주소

라즈베리파이에서 MediaMTX 가 카메라 4채널을 **재인코딩 없이** 그대로 넘겨준다.

| 용도 | URL | 스펙 |
|---|---|---|
| 메인 (선택 채널용) | `rtsp://192.168.0.2:8554/ch1` … `/ch4` | 2592x1520 H.264 30fps |
| 서브 (2x2 미리보기용) | `rtsp://192.168.0.2:8554/ch1s` … `/ch4s` | 640x480 H.264 10fps |

- `ch1`↔카메라 웹UI `CH1`, `ch2`↔`CH2`, … 순서 그대로다.
- 인증 없다. TCP 로 붙어야 한다 (`rtsp_transport=tcp`).
- **on-demand** — 아무도 안 보면 카메라에서 당겨오지 않는다. 첫 프레임까지 1~3초 걸린다.

### 1-2. 실측 (2026-08-03)

| | 4채널 합계 | 비고 |
|---|---|---|
| 메인스트림 | 9.9 Mbps | 카메라 설정 상한은 채널당 5120kbps → 최악 20.5Mbps |
| 서브스트림 | 3.14 Mbps | 메인의 약 1/3 |

둘 다 손실·경고 0. 대역폭은 문제가 아니다.

### 1-3. ⚠️ 서버 쪽 함정 (Qt 문제로 오인하기 쉬움)

**영상이 깨지면 Qt 를 의심하기 전에 서버 경로부터 확인할 것.**

파이는 유선(eth0, 192.168.0.2)과 무선(wlan0, 192.168.0.8)이 같은 서브넷에 동시에
올라와 있고, 그냥 두면 커널이 **무선을 고른다**. 무선으로 나가면 4채널이 깨진다
(카메라가 송신 버퍼를 포기해서 RTP 유실 → 화면 깨짐).

```bash
# 파이에서 확인 — dev eth0 여야 정상
ip route get 192.168.0.13

# 무선으로 나가고 있으면
sudo ip route add 192.168.0.13/32 dev eth0 src 192.168.0.2 metric 50
```

**이 경로는 재부팅하면 사라진다.** 파이를 껐다 켠 뒤 영상이 깨지면 십중팔구 이것이다.

자세한 내용: [`Server/relay/README.md`](../Server/relay/README.md)

---

## 2. 지금 Qt 구조 (건드리기 전에 이해할 것)

### 2-1. 영상이 흐르는 길

```
video_worker (QThread, 1개)          Backend                    VideoView (2개)
  cv::VideoCapture(rtspUrl)   ──────▶ m_worker          ┌──▶ m_originalView  (RTSP 화면)
  ArUco 검출 (같은 스레드)      frameReceived(QImage) ───┤
                               arucoDetected            └──▶ m_topView       (도면/TopView)
```

| 위치 | 내용 |
|---|---|
| `src/video_worker.h/cpp` | RTSP 1개를 열고 프레임 + ArUco 결과를 emit |
| `src/backend.cpp:536,573-578` | 워커 시그널 → 뷰 연결 |
| `src/backend.cpp:668` `registerView()` | QML 의 VideoView 를 Backend 에 등록 |
| `src/backend.h:396` `m_rtspTemplate` | `rtsp://admin:...@{ip}:554/FullHD/media.smp` — **단일 채널 하드코딩** |
| `src/backend.h:401` `m_rtspUrl` | 현재 URL. QSettings `camera/rtspUrl` 이 우선 |
| `src/backend.cpp:1721` `setRtsp(url)` | **URL 바꾸고 워커를 새로 만든다 ← 채널 전환의 핵심** |
| `src/VideoPane.qml` | `VideoView` 하나 + 라벨. `topRole` 로 RTSP/TopView 구분 |
| `src/MainPage.qml:544,840` | VideoPane 2개 배치 (Camera RTSP / Top View) |

### 2-2. 중요한 사실

- **워커는 1개뿐**이다. `Backend::m_worker`.
- **ArUco 검출이 워커 안에 있다.** 프레임을 받으면서 같은 스레드에서 마커를 찾는다.
  주석(`video_worker.cpp:41-45`)에 따르면 1080p 에서 검출 한 번에 20~30ms,
  사전 3개를 다 뒤지면 100ms 에 육박한다. 새 카메라는 **1080p 의 2.4배 픽셀**이다.
  → **4채널에 이걸 다 돌리면 안 된다. 절대로.**
- `VideoView` 는 무겁다 (편집·캘리브·도면 변환 포함, 3000줄+). 미리보기 타일에
  재사용하지 말 것 — 새 경량 아이템을 만든다.

---

## 3. 목표 UX

```
┌─────────────────────────────────────────┐
│  [기본 화면] 2x2 미리보기               │
│  ┌────────┬────────┐                    │
│  │  CH1   │  CH2   │   각 칸: 저해상도  │
│  ├────────┼────────┤   마커검출 안 함   │
│  │  CH3   │  CH4   │   클릭 가능        │
│  └────────┴────────┘                    │
└─────────────────────────────────────────┘
                 │ 채널 클릭
                 ▼
┌─────────────────────────────────────────┐
│  [작업 화면] 선택 채널 1개              │
│  ┌──────────────────┬────────────────┐  │
│  │  CH2 메인스트림   │   Top View     │  │
│  │  마커 검출 ON     │   (도면)       │  │
│  │  그리기 가능      │                │  │
│  └──────────────────┴────────────────┘  │
│  [◀ 채널 목록으로]  [CH1][CH2][CH3][CH4]│
└─────────────────────────────────────────┘
```

- 기본은 4채널 미리보기. 어디서 무슨 일이 일어나는지 한눈에 본다.
- 채널을 고르면 **기존 작업 화면 그대로** — 마커 검출, 도면 그리기, 로봇 제어 전부.
- 작업 중에도 상단/하단의 채널 버튼으로 바로 전환.
- 작업 화면에 있는 동안 나머지 3채널 미리보기는 **끈다** (대역·CPU 절약).

---

## 4. 설계

### 4-1. 절대 원칙 — 기존 동작을 깨지 않는다

PNM 은 **아직 시도 단계**다. PNO 단일채널 직결로 언제든 돌아갈 수 있어야 한다.

- 중계 주소 설정이 **비어 있으면 지금과 100% 동일하게** 동작할 것.
- 기존 `m_rtspUrl` / `setRtsp()` / `applyCamIp()` 경로를 지우지 말 것.
- 캘리브레이션·ArUco·도면 변환 코드는 손대지 말 것.

### 4-2. 채널 URL 조립

QSettings 에 중계 주소만 넣고 나머지는 규칙으로 만든다.

```cpp
// QSettings 키
//   camera/relayBase   예: "rtsp://192.168.0.2:8554"   (비면 4채널 기능 전체 off)
//   camera/channelCount 기본 4

QString mainUrl(int ch) { return m_relayBase + "/ch" + QString::number(ch); }
QString subUrl (int ch) { return m_relayBase + "/ch" + QString::number(ch) + "s"; }
```

`relayBase` 가 비어 있으면 `m_channelMode == false` → 기존 단일 채널 그대로.

> 서버가 `LOGIN_OK` 로 중계 주소를 내려주게 하는 것도 방법이지만(§7), 지금은
> **QSettings 로 충분**하다. 서버 프로토콜을 안 건드려야 Qt 작업과 서버 작업이
> 서로를 안 막는다.

### 4-3. 미리보기 계층 (새로 만드는 것)

| 새 파일 | 역할 |
|---|---|
| `src/preview_worker.h/cpp` | 서브스트림 1개 디코드 전용. **ArUco 없음**, 필터 없음 |
| `src/channel_tile.h/cpp` | `QQuickPaintedItem`. QImage 를 칸에 맞춰 그리고 클릭 시그널 |
| `src/ChannelGrid.qml` | `ChannelTile` 4개를 2x2 로 배치 + 라벨/상태 |

`preview_worker` 는 `video_worker` 를 통째로 복사하지 말고 **필요한 부분만** 가져온다:

가져올 것 — 재연결 백오프, `openFailed` 1회 포기 가드, 프레임 큐 백프레셔(`m_queued`)
버릴 것 — ArUco 검출, 밝기/대비/샤픈 필터, 통계 EMA

### 4-4. 채널 선택 = 기존 경로 재사용

```cpp
// Backend 에 추가
Q_INVOKABLE void selectChannel(int ch);   // 1..4

void Backend::selectChannel(int ch)
{
    if (!m_channelMode || ch == m_selectedChannel) return;
    m_selectedChannel = ch;
    stopPreviews();                 // 미리보기 4개 정지 (대역·CPU 회수)
    setRtsp(mainUrl(ch));           // ← 기존 함수. 워커 교체 + 뷰 재연결까지 알아서 함
    emit channelChanged();
}

void Backend::showChannelGrid()
{
    m_selectedChannel = 0;
    stopMainWorker();
    startPreviews();
}
```

**이게 이 설계의 핵심이다.** `setRtsp()` 는 이미 워커를 죽이고 새로 만들고 뷰에
다시 연결하는 일을 전부 한다 (`backend.cpp:1721-1746`). 채널 전환은 그 함수를
다른 URL 로 부르는 것 이상이 아니다.

### 4-5. Q_PROPERTY 추가 (QML 이 볼 것)

```cpp
Q_PROPERTY(bool channelMode      READ channelMode      NOTIFY channelChanged)  // 4채널 기능 켜짐?
Q_PROPERTY(int  channelCount     READ channelCount     NOTIFY channelChanged)
Q_PROPERTY(int  selectedChannel  READ selectedChannel  NOTIFY channelChanged)  // 0 = 그리드 화면
```

`MainPage.qml` 은 `Backend.selectedChannel === 0` 이면 `ChannelGrid`, 아니면 기존
작업 화면을 보여주면 된다. **기존 레이아웃을 갈아엎지 말고 조건부로 감쌀 것.**

---

## 5. 작업 순서

각 단계가 끝날 때마다 **PNO 모드(relayBase 비움)로도 돌아가는지** 확인하고 커밋한다.

### Phase 1 — 채널 설정 배관 (기능 변화 없음)
1. `Backend` 에 `m_relayBase`, `m_channelCount`, `m_selectedChannel` + QSettings 읽기/쓰기
2. `mainUrl()` / `subUrl()` 헬퍼
3. Q_PROPERTY 3개 + `selectChannel()` / `showChannelGrid()` 뼈대
4. 설정 화면에 "중계 주소" 입력칸 추가
- **확인**: relayBase 를 넣어도 화면은 아직 그대로. 비우면 당연히 그대로.

### Phase 2 — 선택 채널만 먼저 (미리보기 없이)
5. `selectChannel(n)` → `setRtsp(mainUrl(n))` 연결
6. 임시로 채널 버튼 4개만 UI 에 놓고 전환 테스트
- **확인**: CH1~4 전환이 되고, 마커 검출·그리기가 **전과 똑같이** 동작한다.
- 여기까지만 해도 실사용 가능하다. 급하면 여기서 멈춰도 된다.

### Phase 3 — 2x2 미리보기
7. `preview_worker` 구현
8. `channel_tile` 구현
9. `ChannelGrid.qml` + `MainPage.qml` 조건부 전환
10. 그리드↔작업 화면 왕복 시 워커 정리 확인 (**누수·좀비 스레드 주의**)
- **확인**: 그리드에서 4칸 다 나오고, 클릭하면 그 채널이 크게 뜬다.

### Phase 4 — 성능 (필요하면)
11. 하드웨어 디코딩 (§6)
12. 프로파일링 후 조정

---

## 6. 성능 — 하드웨어 디코딩

작업 PC: **i5-1135G7 / Iris Xe / 32GB / Win11**. 이 CPU 에는 퀵싱크(Quick Sync)가
있어서 H.264 디코딩을 전담 처리할 수 있다.

지금 코드는 `cv::VideoCapture(url, cv::CAP_FFMPEG)` — **소프트웨어 디코딩**이다.

- 서브스트림 4개(640x480 10fps)는 소프트웨어로도 여유롭다 → **Phase 3 까지는 그대로 가도 된다.**
- 메인스트림 1개(2592x1520 30fps) + ArUco 도 아마 버틴다.
- 문제가 되면 그때 하드웨어 가속을 켠다:

```cpp
cv::VideoCapture cap;
cap.open(url, cv::CAP_FFMPEG, {
    cv::CAP_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_ANY,
});
```

번들된 OpenCV 는 4.12.0 (`Client/libopencv_*4120.dll`) 이라 이 API 를 지원한다.
켠 뒤에는 **작업관리자에서 GPU "Video Decode" 그래프가 올라가는지** 반드시 확인할 것
— 조용히 소프트웨어로 폴백되는 경우가 흔하다.

### ⚠️ `stimeout` 문제

`video_worker.cpp:114` 에서:

```cpp
qputenv("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp|stimeout;5000000");
```

`stimeout` 은 **FFmpeg 5.0 에서 `timeout` 으로 이름이 바뀌었다.** 번들 FFmpeg 이
최신이면 이 옵션은 조용히 무시되고 타임아웃이 기본값(30초)으로 돌아간다. 그러면
스트림이 죽었을 때 `grab()` 이 30초를 통째로 잡아먹는다.

**확인 방법**: 카메라를 뽑고 재연결까지 몇 초 걸리는지 잰다. 30초면 무시된 것이다.
그 경우 `timeout;5000000` 을 같이 넣지 말고 **둘 중 맞는 하나만** 쓴다 —
`video_worker.cpp:108-113` 주석에 왜 둘 다 넣으면 안 되는지 적혀 있다.

또한 `qputenv` 는 **프로세스 전역**이다. 워커가 여러 개가 되므로 각 워커의 `run()`
에서 부르지 말고 `main()` 에서 한 번만 설정하는 쪽으로 옮기는 것이 안전하다.

---

## 7. 나중에 (지금은 하지 말 것)

- **서버가 중계 주소를 내려주기**: `LOGIN_OK` 에 선택적 `stream` 필드
  (`{base, channels}`) 를 더한다. 있으면 쓰고 없으면 QSettings 로 폴백.
  `cam_ip` 의 의미는 **바꾸지 말 것** — 바꾸면 PNO 경로가 깨진다.
  서버 쪽 파일: `Server/src/router.cpp:87`, `Server/server_PROTOCOL.md:257`
- **채널별 캘리브레이션**: 채널마다 렌즈·각도가 다르므로 캘리브 번들도 채널별로
  가져야 한다. 지금은 채널 1개만 제대로 쓴다는 전제로 진행한다.
- **4채널 동시 마커 검출**: 하지 말 것. 로봇은 한 채널에만 보인다.

---

## 8. 하면 안 되는 것 (기존 코드 주석에서 옮김)

| 금지 | 이유 |
|---|---|
| 프레임을 `remap` 으로 펴기 | 2026-07-29 확정 — 원본 왜곡을 유지하고 **선을 왜곡시켜** 그린다. 프레임을 펴면 이중 보정으로 오버레이가 어긋난다 (`video_worker.h:37-46`) |
| `video_worker` 에 FilterMode/setHomographyPoints 되살리기 | 호출부가 없어서 지운 죽은 코드다 (`video_worker.h:20-24`) |
| 스트림 못 열었을 때 계속 재시도 | Hanwha 는 계정을 잠근다 (`RTSP/1.0 490 Account Blocked`). **한 번이라도 프레임을 받은 뒤**에만 재연결 (`video_worker.cpp:142-150`). 워커가 4개가 되면 위험도 4배다 |
| 열기 성공 후 `retry` 를 0 으로 리셋 | 접속은 되는데 데이터가 안 오는 상태에서 무한 재접속 루프가 된다. **프레임을 실제로 받았을 때만** 리셋 (`video_worker.cpp:154-158`) |
| 프레임 큐 백프레셔 제거 | 2592x1520 RGB 한 장이 ~11MB 다. GUI 가 못 따라가면 메모리와 지연이 같이 폭발한다 (`video_worker.h:44-48`) |

---

## 9. 테스트

### 9-1. 손으로 먼저 확인 (코드 짜기 전에)

PC 에서 VLC 나 ffplay 로 열어본다. 여기서 안 나오면 Qt 문제가 아니다.

```
rtsp://192.168.0.2:8554/ch1     메인
rtsp://192.168.0.2:8554/ch1s    서브
```

VLC: `도구 → 환경설정 → 입력/코덱 → 라이브 캡처 스트리밍 방식: TCP`

### 9-2. 단계별 확인

| 단계 | 확인 |
|---|---|
| Phase 1 | relayBase 넣어도/비워도 기존과 동일 |
| Phase 2 | CH1~4 전환 OK. 마커 검출·그리기·도면 **전과 동일** |
| Phase 3 | 그리드 4칸 표시. 클릭 전환. 왕복해도 스레드/메모리 안 늘어남 |
| 회귀 | **relayBase 비우고** PNO(192.168.0.12) 로 붙여서 기존 동작 확인 |

### 9-3. 성능 확인

- 작업관리자: CPU, GPU Video Decode
- 그리드(서브 4개) vs 작업화면(메인 1개 + ArUco) 각각 측정
- 메모리가 계속 우상향하면 프레임 큐 백프레셔가 깨진 것이다

---

## 10. 되돌리기

작업은 `feature/pnm` 브랜치의 별도 워크트리에서 한다. 원본 `feature/server` 는
PNO 로 계속 돌아간다.

```bash
# PNM 이 엎어지면 — 워크트리만 지우면 끝
git worktree remove ../Road_Painter_PNM

# Qt 만 되돌리려면
git checkout feature/server -- Client/
```

앱 안에서 급히 되돌릴 때는 **설정에서 중계 주소를 비우면** 즉시 기존 단일 채널
동작으로 돌아간다. 재빌드 필요 없다 — 이게 §4-1 원칙을 지키는 이유다.
