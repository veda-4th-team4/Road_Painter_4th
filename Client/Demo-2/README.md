# Road-Painter — QML 관제 클라이언트 (Mock)

CCTV 기반 자율 구획 도장 로봇 시스템 **Road-Painter**의 관제 클라이언트를
C++17 + Qt6 + Qt Quick(QML)으로 재구현한 프로젝트입니다.
실제 하드웨어(한화비전 CCTV RTSP, 서버 RPi, 로봇)가 없어도 **단독 실행·시연**이
가능하도록 외부 의존 요소는 전부 목업으로 대체했습니다.

## 기능

| 기능 | 구현 방식 |
|---|---|
| 영상 표시 | RTSP 대신 QML Canvas로 렌더링한 **목업 CCTV 탑뷰 씬**(지하주차장 바닥, ArUco 기준 마커, OSD 타임스탬프/REC) |
| 경로 작도 → JSON 내보내기 | 영상 위 클릭으로 정점 추가, 우클릭으로 완료 → `임시 저장(path_temp.json)` 또는 파일 다이얼로그로 내보내기 |
| 경로 편집 | 정점 드래그 이동, 정점 우클릭 삭제, 바운딩 박스 드래그로 전체 이동, 우하단 핸들로 크기 조절 |
| 경로 프리셋 | 수평 직선 / 직사각형 / 삼각형 / 원형(육각형) 루프 / **주차 구획(4칸)** — 생성 후 편집 가능 |
| 비디오 필터 | QML **ShaderEffect**(qsb 셰이더): 밝기·대비·채도·선명도(언샤프 마스크)·그레이스케일 |
| 좌표 측정 | 십자선 + 실시간 이미지 좌표 표시, 클릭으로 마커 고정 |
| 로봇 이동 시뮬레이션 | 대기 지점 → 시작점(펜 업) → 경로 추종(펜 다운)하며 바닥에 도장 궤적을 남김. 속도/도료 색상/일시정지/리셋 |
| 4채널 관제 컨셉 | CH1 라이브 미니뷰 + CH2~4 NO SIGNAL 목업 (기획서 B안 반영) |
| JSON 불러오기 | BLUEPRINT 스키마 + 레거시(`path[]`) 스키마 모두 지원 |

## 빌드

### 요구 사항

- Qt 6.5 이상 (검증: **Qt 6.11.0 MinGW 64-bit**)
- 필요 Qt 모듈: `Quick`, `QuickControls2`, `ShaderTools` (+ 런타임에 `QuickDialogs2`)
- CMake 3.21+, Ninja(권장)

### Windows (MinGW, 이 저장소에서 검증한 커맨드)

```bat
cmake -S . -B build-release -G Ninja -DCMAKE_BUILD_TYPE=Release ^
  -DCMAKE_PREFIX_PATH=C:/Qt/6.11.0/mingw_64 ^
  -DCMAKE_C_COMPILER=C:/Qt/Tools/mingw1310_64/bin/gcc.exe ^
  -DCMAKE_CXX_COMPILER=C:/Qt/Tools/mingw1310_64/bin/g++.exe ^
  -DCMAKE_MAKE_PROGRAM=C:/Qt/Tools/Ninja/ninja.exe
cmake --build build-release
```

실행 (Qt DLL 경로 필요):

```bat
set PATH=C:\Qt\6.11.0\mingw_64\bin;%PATH%
build-release\appRoadPainter.exe
```

또는 Qt Creator에서 `CMakeLists.txt`를 열고 Qt 6 키트로 빌드/실행하면 됩니다.

### 헤드리스 셀프테스트

백엔드 로직(경로 모델, JSON 직렬화/역직렬화, 프리셋, 로봇 시뮬레이션)을
GUI 없이 검증합니다:

```bat
build-release\appRoadPainter.exe --selftest
```

24개 항목이 `[PASS]`로 출력되고 종료 코드 0이면 정상입니다.

## 사용법 (시연 흐름)

1. 좌측 툴바 **작도** → 영상 위를 클릭해 정점 추가 → **우클릭**으로 완료
2. 완료 다이얼로그에서 **임시 저장**(실행 폴더의 `path_temp.json`) 또는 **파일로 내보내기**
3. 또는 **프리셋** → 도형 선택 → 드래그 이동 / 우하단 핸들로 크기 조절
4. 우측 패널 **경로 데이터**에서 폐곡선 여부·좌표계(픽셀/미터) 선택 후 내보내기
5. **로봇 이동 시뮬레이션** → 시작: 로봇이 경로를 따라 주행하며 바닥에 선을 그림
6. **비디오 필터** 슬라이더로 영상 품질 조정 (기획서의 영상 품질 향상 요구 대응)

단축키: `Esc` 선택 모드, `Ctrl+Z` 마지막 점 삭제, `Ctrl+E` JSON 내보내기

## 프로젝트 구조

```
RoadPainter/
├── CMakeLists.txt            # qt_add_executable / qt_add_qml_module / qt_add_shaders
├── src/                      # C++ 백엔드 (QML_ELEMENT로 QML에 노출)
│   ├── main.cpp              #   진입점 + --selftest 하네스
│   ├── pathmodel.{h,cpp}     #   경로 모델(QAbstractListModel) + JSON 직렬화
│   ├── presetlibrary.{h,cpp} #   프리셋 도형 생성기
│   ├── robotsimulator.{h,cpp}#   로봇 주행 시뮬레이터(QTimer 기반)
│   └── filtersettings.{h,cpp}#   비디오 필터 파라미터 상태
├── qml/                      # UI
│   ├── Main.qml              #   메인 윈도우(헤더/툴바/뷰포트/우측 패널/다이얼로그)
│   ├── Theme.qml             #   다크 디자인 토큰 싱글턴
│   ├── VideoViewport.qml     #   목업 영상 + 필터 + 오버레이 합성
│   ├── CctvScene.qml         #   목업 CCTV 탑뷰 씬 + OSD
│   ├── PathOverlay.qml       #   작도/편집/측정 인터랙션
│   ├── RobotMarker.qml       #   로봇 목업(ArUco 플레이트)
│   └── ...                   #   공용 컴포넌트(AppButton, LabeledSlider 등)
├── shaders/videofilter.frag  # 비디오 필터 셰이더 (qsb로 컴파일됨)
├── resources/                # 샘플 JSON 3종
└── docs/SCHEMA.md            # JSON 스키마 문서
```

## 아키텍처

- **C++ 백엔드**: `QObject`/`QAbstractListModel` 파생 클래스를 `QML_ELEMENT`로 선언하고
  `qt_add_qml_module`의 `SOURCES`로 등록. 좌표 데이터, 파일 I/O, JSON 직렬화,
  시뮬레이션 타이밍은 전부 C++에서 처리합니다.
- **QML UI**: 스트림 이미지 좌표계(1280×720)를 가진 `stage` 아이템을 뷰포트에 맞춰
  스케일하므로, 마우스 좌표가 곧 이미지 픽셀 좌표가 됩니다.
  목업 영상(씬+도장 궤적+로봇)은 `ShaderEffectSource` → `ShaderEffect`로 필터링되고,
  작도 오버레이는 필터 위에 그려집니다.
- **디자인 시스템**: 색상·간격·타이포 토큰을 `Theme.qml` 싱글턴으로 정의해 재사용.

## 주요 결정 사항 (기획서/기존 구현 대비)

1. **RTSP → 목업 씬**: 샘플 영상 파일 없이도 실행되도록 QML Canvas로 CCTV 탑뷰를 렌더링.
   OSD(타임스탬프, REC, 코덱 정보)로 관제 영상의 룩을 재현.
2. **호모그래피 → 모의 캘리브레이션**: OpenCV 의존성을 제거하고 미터 내보내기는
   8.0×4.5 m 작업 구역 선형 매핑으로 대체. 스키마(`meta.workAreaSize`)에 명시.
3. **JSON 스키마 유지 + 확장**: 기존 `BLUEPRINT` 구조를 그대로 쓰되 `meta`를 추가,
   레거시 `path[]` 형식은 불러오기로 호환. 상세: [docs/SCHEMA.md](docs/SCHEMA.md)
4. **프리셋에 주차 구획 추가**: 기획서 MVP(직선·사각형·주차구획)를 반영해
   기존 4종에 주차 구획 한붓그리기 프리셋을 추가.
5. **테스트 모드/설정 다이얼로그 생략**: 기존 데모의 RTSP 주소 변경·채널 설정 등
   하드웨어 종속 기능은 목업 범위에서 제외.
