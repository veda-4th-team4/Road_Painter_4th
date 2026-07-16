# Road-Painter 경로 JSON 스키마

## BLUEPRINT 스키마 (v1.1, 기본 내보내기 형식)

기존 위젯 데모의 `BLUEPRINT` 포맷(`type` / `seq` / `payload.points`)을 그대로 유지하고,
소비자가 무시해도 되는 `meta` 블록을 추가했다.

```json
{
    "type": "BLUEPRINT",
    "seq": 1,
    "payload": {
        "points": [ [399.6, 298.7], [799.2, 248.9], [999.8, 499.3] ]
    },
    "meta": {
        "app": "Road-Painter QML Client",
        "schemaVersion": "1.1",
        "createdAt": "2026-07-16T14:24:33",
        "coordinateSpace": "pixel",
        "imageSize": [1280, 720],
        "closed": false,
        "pointCount": 3
    }
}
```

| 필드 | 타입 | 설명 |
|---|---|---|
| `type` | string | 항상 `"BLUEPRINT"` (로봇 제어 프로토콜의 메시지 타입) |
| `seq` | int | 내보내기 시퀀스 번호. 세션 내에서 1부터 증가 (기존 데모는 4 고정) |
| `payload.points` | number[][] | `[[x, y], ...]` 순서 있는 정점 목록. 로봇은 이 순서대로 주행 |
| `meta.coordinateSpace` | string | `"pixel"` 또는 `"meter"` |
| `meta.imageSize` | int[2] | 픽셀 좌표의 기준 스트림 해상도 `[width, height]` |
| `meta.workAreaSize` | number[2] | `meter`일 때만 존재. 작업 구역 실측 크기 `[m, m]` |
| `meta.closed` | bool | 폐곡선 여부. `true`면 마지막 정점 후 시작 정점으로 복귀 (points에 시작점을 중복 저장하지 않음) |
| `meta.pointCount` | int | `payload.points` 길이 (검증용) |

### 좌표계

- **pixel** (기본): 목업 CCTV 스트림의 이미지 픽셀 좌표. 원점은 좌상단, x→오른쪽, y→아래.
  소수점 1자리로 반올림.
- **meter**: 작업 구역(기본 8.0 × 4.5 m)에 대한 **모의 캘리브레이션** 좌표.
  `x_m = x_px × workAreaWidth / imageWidth` 형태의 선형 매핑이며, 소수점 4자리로 반올림.
  실제 시스템에서는 이 단계가 OpenCV 호모그래피 변환(`cv::findHomography` +
  `cv::perspectiveTransform`)으로 대체된다. (본 목업은 OpenCV 의존성이 없어 선형 매핑으로 대신함)

### 폐곡선 표현에 대한 결정

기존 데모는 폐곡선 개념이 없어 사용자가 시작점을 한 번 더 클릭하는 방식이었다.
본 구현은 `meta.closed` 플래그로 표현하고 정점을 중복 저장하지 않는다.
레거시 소비자는 `meta`를 무시하므로 열린 경로로 해석된다 (호환 유지).

## 레거시 스키마 (불러오기 전용 호환)

기존 데모 초기 버전의 형식도 불러오기를 지원한다:

```json
{
    "path": [
        { "sequence": 1, "x": 350, "y": 300 },
        { "sequence": 2, "x": 900, "y": 300 }
    ],
    "total_points": 2
}
```

`path[].x`, `path[].y`를 배열 순서대로 읽는다 (픽셀 좌표로 간주).

## 파일 위치

- **임시 저장**: 실행 디렉터리의 `path_temp.json` (기존 데모와 동일한 동작)
- **파일로 내보내기**: 파일 다이얼로그로 위치 지정
- 샘플: `resources/sample_path_pixel.json`, `resources/sample_path_meter.json`,
  `resources/sample_path_legacy.json`
