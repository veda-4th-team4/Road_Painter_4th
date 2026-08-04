# Qt 클라이언트 빌드 매뉴얼 (초안)

> 작성 2026-08-04 · 브랜치 `feature/pnm`
>
> ⚠️ **이 문서는 초안이다.** 특히 TLS 인증서 검증 부분(§4)은 "생략된 상태 그대로"를
> 기록해둔 것이지, 검증을 완료했다는 뜻이 아니다. 프로덕션 배포 전 반드시 다시
> 볼 것 — 아래 §4에서 왜 위험한지, 뭘 해야 하는지 적어뒀다.

---

## 0. 30초 요약

`Client/` 루트에 이미 빌드된 `RoadPainter.exe` + DLL 전부가 커밋돼 있다.
**그냥 쓰기만 할 거면 `git pull` 하고 더블클릭하면 끝** — 아무것도 설치할 필요 없다.

코드를 고치고 다시 빌드하려면 얘기가 다르다: 이 브랜치는 원래 팀 공유
OpenCV 빌드(`C:/opencv/mingw`, 4.12) 대신 **이 PC 로컬에 설치한 MSYS2 OpenCV
4.13**으로 링크돼 있다. 그대로는 다른 PC에서 빌드가 안 된다 — §2 참고.

---

## 1. 그냥 실행만 할 사람 (팀원 대부분)

```bash
git pull
```

그리고 `Client/RoadPainter.exe` 더블클릭. 끝.

- Qt·MSYS2·OpenCV 아무것도 설치할 필요 없다. DLL이 전부 `Client/` 루트에
  같이 커밋돼 있어서 폴더 자체로 독립 실행된다.
- 확인 방법: 이 exe를 **MSYS2가 전혀 없는 PATH 상태**에서 직접 실행해서
  4초 이상 안 죽고 로그인 화면이 뜨는 것까지 검증했다.

---

## 2. 코드를 고치고 다시 빌드하려는 사람

### 2-1. 지금 상태 — 왜 그냥 열면 안 되는가

`Client/src/CMakeLists.txt`:

```cmake
set(OpenCV_DIR "C:/msys64/mingw64/lib/cmake/opencv4")
```

이 경로는 **이 브랜치를 작업한 PC 하나에만** 있다. 팀 공유 OpenCV 빌드가
있는 PC에서 그대로 열면 `find_package(OpenCV REQUIRED)`가 그 시점에
`C:/msys64/...`를 찾다가 실패한다. 반대로 아무 OpenCV도 없는 PC에서 열어도
당연히 실패한다.

### 2-2. 옵션 A — 팀 공유 빌드(OpenCV 4.12)를 쓴다

원래 하던 방식. `C:/opencv/mingw`에 팀 공유 OpenCV 4.12 MinGW 빌드를 받아두고,
`CMakeLists.txt`를 원래대로 되돌린다:

```cmake
set(OpenCV_DIR "C:/opencv/mingw")
```

### 2-3. 옵션 B — MSYS2로 새로 받는다 (이 브랜치 지금 상태와 동일하게)

팀 공유 빌드가 없거나 못 구했을 때. **주의: OpenCV 4.12가 아니라 4.13이 깔린다**
(팀 공유 빌드와 정확히 같은 버전이 아니다 — 지금까지는 문제없이 링크·실행됐지만
100% 같은 코드는 아니라는 뜻).

```powershell
winget install --id MSYS2.MSYS2
```

설치 후 (MSYS2 터미널에서, 첫 실행은 자기 자신을 업데이트하며 창이 닫힐 수 있음 —
그러면 다시 열어서 이어서):

```bash
pacman -Syu
pacman -S mingw-w64-x86_64-opencv
```

`CMakeLists.txt`는 지금 상태(`C:/msys64/mingw64/lib/cmake/opencv4`) 그대로 두면 된다.

### 2-4. Qt Creator에서 열기

필요한 것 (Qt 온라인 설치 프로그램으로 한 번에 깔림):

| 구성요소 | 이 브랜치 검증 당시 버전 |
|---|---|
| Qt | 6.11.0, MinGW 64-bit 킷 |
| MinGW | 13.1.0 (Qt 번들) |
| CMake / Ninja | Qt Tools에 포함된 것 사용 |

1. Qt Creator → 파일 → 파일 또는 프로젝트 열기 → `Client/src/CMakeLists.txt`
2. 킷: **Desktop Qt 6.11.0 MinGW 64-bit**
3. 빌드

### 2-5. 빌드 후 배포본 갱신 (실행 파일을 다시 커밋할 때)

CMake 빌드 결과물은 `Client/src/build/<킷>/dist/`에 생긴다 (이 폴더 자체는
`.gitignore`에 걸려있어 커밋 안 됨). 이걸 실제 배포본으로 반영하려면:

```powershell
# Qt 자체 DLL·QML 모듈을 dist/ 안에 채워 넣기 (OpenCV DLL은 이미 dist/에 있음)
C:\Qt\6.11.0\mingw_64\bin\windeployqt.exe --qmldir Client\src Client\src\build\<킷>\dist\RoadPainter.exe

# 그 dist/ 내용을 Client/ 루트로 복사 (기존 파일 덮어씀)
```

그다음 `Client/` 루트에서 `git status`로 바뀐 파일 확인 후 커밋한다.
(개별 DLL 300개 넘게 나올 수 있다 — `git add Client/`로 한 번에.)

---

## 3. 4채널(PNM) 모드 켜기

기본은 기존 1채널 모드다. 로그인 후 **설정 → "4채널 중계 주소"** 에
`rtsp://<중계서버>:8554` 형태로 넣어야 `ChannelGrid` 화면(2×2 채널 선택)이 뜬다.
비워두면 100% 기존 동작(`Backend::channelMode() == false`).

---

## 4. ⚠️ TLS 인증서 검증 — 지금 생략돼 있다 (초안 미해결)

`Client/src/serverclient.cpp:15`:

```cpp
m_socket->setPeerVerifyMode(QSslSocket::VerifyNone);
```

**Qt 클라이언트는 서버가 내미는 TLS 인증서를 전혀 검증하지 않는다.** 이건 이번
세션에서 만든 게 아니라 원래 코드에 있던 것이고("1차 연동이므로... 생략한다"라는
주석이 이미 있었음), 이번에 로컬 테스트하면서 실제로 그 덕에 제가 만든 테스트용
자가서명 인증서로도 그냥 붙는 걸 확인했다.

**의미**: 같은 네트워크에서 `192.168.0.8:9000`인 척하는 아무 서버나 갖다 놔도
Qt는 의심 없이 붙는다. 사내망 초기 개발 단계에서는 큰 문제가 안 되지만,
**외부에 노출되거나 실제 현장에 배포하기 전에는 반드시 고쳐야 한다.**

권장 방향(코드 주석에도 이미 적혀있음): `server.crt`를 신뢰 CA로 등록하고
`VerifyNone` → 정식 검증으로 바꾸는 것. 이번 세션에서는 손대지 않았다 — 이
초안 문서에 남기는 이유가 이거다.
