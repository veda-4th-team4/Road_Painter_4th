#pragma once
// 캘리브레이션 데이터 (CCTV가 H_MATRIX로 올리는 번들) 파싱/적용.
//
// 좌표계 규약:
//   - CCTV는 마커 코너를 "원본 픽셀 좌표"로만 보낸다 (변환 금지).
//   - 서버가 undistort(왜곡 보정) -> H_marker 적용으로 바닥 좌표를 계산한다.
//   - CCTV가 보내는 H_floor/H_marker는 "왜곡 보정된 픽셀 -> 월드 평면 mm" 사영변환이다.
//     (H_marker = 마커 장착 높이 평면용. 마커가 바닥에서 떠 있어 생기는
//      시차(parallax)를 캘리브레이션 단계에서 흡수한 것)
//
// 단위 규약 (QT_CCTV_SERVER_CALIBRATION_FORMAT 2026-08-11):
//   번들은 CCTV가 보낸 **mm 그대로** 저장·중계한다. ÷1000은 서버 내부 좌표 계산용
//   사본(Calib::Hf/Hm)에서만 일어나고, 그래서 pose/POSE/BLUEPRINT/PATH와 로봇
//   프로토콜은 예전처럼 전부 미터다. 바뀐 것은 "저장·중계본의 단위"뿐이다.
//
//   예전에는 수신 즉시 번들 자체를 ÷1000 해서 저장·중계했다. 그러면 unit 필드가
//   말하는 단위와 H에 실제로 든 단위가 어긋날 여지가 남는다 - 변환을 안 거치는
//   경로가 하나라도 생기면 좌표가 조용히 1000배 틀어지는데 로그는 정상으로 찍힌다.
//   단위 판단을 번들의 unit 필드 한 곳으로 모아 그 여지를 없앤 것이다.
//
// 번들 포맷 (2026-08-11 정규형): 아래를 전부 갖춘 오브젝트. 서버는 숫자 배열을
//   값과 순서까지 그대로 보존해 저장·중계한다.
//   {"calib_id","created_at","image_size":[2592,1520],"coord_mode":"undistort",
//    "unit":"mm", "K":[[fx,0,cx],[0,fy,cy],[0,0,1]], "D":[k1,k2,p1,p2,k3],
//    "H_floor":[[...]x3], "H_marker":[[...]x3], "marker_height_mm":160,
//    "origin_mm":[0,0], "canvas_mm":[900,600], "axis":"x_right_y_up"}
// 평면 포맷 (QT-REQ-CCTV-001 rev.2): 같은 내용이되 바닥 H의 이름이 "H"다.
//   서버는 H를 H_floor로 읽고 나머지는 손대지 않은 채 저장·중계한다.
// 레거시 포맷 (v0.2 이하): [[...]x3] 단일 H -> H_floor=H_marker=H, 왜곡 보정 생략
#include "protocol.hpp"
#include <array>
#include <cmath>

struct Calib {
    bool valid = false;      // H가 하나라도 있으면 true
    bool hasKD = false;      // K/D가 있어야 왜곡 보정 수행
    double fx = 1, fy = 1, cx = 0, cy = 0;   // K (내부 파라미터)
    std::array<double, 5> D{};               // 왜곡 계수 (k1,k2,p1,p2,k3)
    double Hf[3][3] = {};    // 왜곡 보정 픽셀 -> 바닥 평면 (m). 서버 내부 계산용 사본
    double Hm[3][3] = {};    // 왜곡 보정 픽셀 -> 마커 높이 평면 (m). 서버 내부 계산용 사본
    bool hasFloor = false, hasMarker = false;
    json raw;                // 영속 저장/QT 중계용 원본 번들 (mm. 손대지 않는다)
};

inline bool parseMat3(const json& j, double m[3][3]) {
    if (!j.is_array() || j.size() != 3) return false;
    for (int r = 0; r < 3; ++r) {
        if (!j[r].is_array() || j[r].size() != 3) return false;
        for (int c = 0; c < 3; ++c) {
            if (!j[r][c].is_number()) return false;
            m[r][c] = j[r][c].get<double>();
        }
    }
    return true;
}

// 호모그래피 행렬의 0,1행에 스칼라 s를 곱한다 (world = H·[u,v,1]의 0,1행 / 2행이므로
// 0,1행을 s배하면 변환 결과가 s배). 형식이 어긋난 원소는 건너뛴다(파싱 검증은 이후 단계).
inline void scaleMat3Rows01(json& m, double s) {
    if (!m.is_array() || m.size() != 3) return;
    for (int r = 0; r < 2; ++r) {
        if (!m[r].is_array() || m[r].size() != 3) return;
        for (int c = 0; c < 3; ++c)
            if (m[r][c].is_number()) m[r][c] = m[r][c].get<double>() * s;
    }
}

// 같은 연산의 C 배열 판 (파싱이 끝난 Calib::Hf/Hm 용).
inline void scaleMat3Rows01(double m[3][3], double s) {
    for (int r = 0; r < 2; ++r)
        for (int c = 0; c < 3; ++c) m[r][c] *= s;
}

// 번들이 스스로 밝히는 단위 -> 미터 환산 계수. 단위 판단은 전부 이 함수 하나를 거친다.
//   "mm"  : 0.001  정규 형식. CCTV 송신분이자 서버가 저장·중계하는 형태
//   "m"   : 1.0    구 서버가 ÷1000 해서 저장해 둔 번들
//   없음  : 1.0    구 서버 저장분 (수신 경로는 stampCalibUnit()이 "mm"을 미리 박는다)
// 레거시 단일 H 배열은 태그할 자리가 없어 수신 때 미터로 바꿔 보관한다 -> 1.0.
inline double bundleMeterScale(const json& bundle) {
    if (!bundle.is_object()) return 1.0;
    if (!bundle.contains("unit") || !bundle["unit"].is_string()) return 1.0;
    return bundle["unit"].get<std::string>() == "mm" ? 0.001 : 1.0;
}

// 갓 수신한 번들을 저장·중계할 정규형으로 맞춘다. **mm은 mm 그대로 둔다.**
// 반환값 = unit이 없어서 서버가 "mm"으로 가정해 박았는가 (호출부 경고 로그용).
inline bool stampCalibUnit(json& bundle) {
    if (bundle.is_array()) {  // 레거시 단일 H: 단위를 적어둘 자리가 없다.
        scaleMat3Rows01(bundle, 0.001);  // 예전처럼 미터로 바꿔 보관한다.
        return false;
    }
    if (!bundle.is_object()) return false;
    if (bundle.contains("unit") && bundle["unit"].is_string()) return false;
    bundle["unit"] = "mm";  // 규격상 필수 필드다. 없으면 옛 CCTV로 보고 mm으로 가정.
    return true;
}

// 구 서버가 미터로 저장해 둔 번들을 규격 단위(mm)로 되돌린다.
//
// 저장 파일은 건드리지 않는다. 읽을 때마다 같은 결과가 나오는 순수 변환이라, 굳이
// 현장 users.json을 다시 쓰는 위험을 질 이유가 없다. 저장분이 미터라는 것은 추론이
// 아니다 - setCalib/setGlobalCalib을 부르는 곳이 handleHMatrix 하나뿐이었고 거기서
// 무조건 ÷1000 한 뒤 저장했다.
inline bool bundleToMm(json& bundle) {
    if (!bundle.is_object()) return false;                // 레거시 배열은 대상 아님
    if (bundleMeterScale(bundle) == 0.001) return false;  // 이미 mm
    const double s = 1000.0;
    if (bundle.contains("H_floor")) scaleMat3Rows01(bundle["H_floor"], s);
    if (bundle.contains("H_marker")) scaleMat3Rows01(bundle["H_marker"], s);
    // 평면 스키마(QT-REQ-CCTV-001)는 바닥 H를 "H"로 부른다. 같이 환산하지 않으면
    // 한 번들 안에서 두 평면의 단위가 어긋난다.
    if (bundle.contains("H")) scaleMat3Rows01(bundle["H"], s);
    bundle["unit"] = "mm";
    return true;
}

// 평면 스키마의 바닥 H("H")에 "H_floor" 별칭을 달아준다.
//
// 서버는 두 스키마를 동등하게 읽지만(calibFromJson), 저장·중계하는 JSON은 들어온
// 그대로여서 출력 키 이름이 입력 형식에 따라 달라졌다. QT는 `calib.H_floor`만 보므로
// (QT-REQ-SRV-001 rev.3 C-1), 평면 번들로 올리면 QT가 좌표계를 못 잡았다 - 에러 없이
// 조용히. 관리자 창에서 어느 버튼을 눌렀는지가 QT 동작을 갈랐다.
//
// 원본 "H"는 지우지 않고 남긴다: QT-REQ-CCTV-001 형식을 그대로 보존해야 번들을
// 발행한 CCTV 쪽과 대조할 수 있고, 이미 "H"를 읽는 소비자가 있어도 깨지지 않는다.
inline void aliasFloorKey(json& bundle) {
    if (!bundle.is_object()) return;  // 레거시 단일 H 배열은 대상 아님
    if (bundle.contains("H") && !bundle.contains("H_floor")) bundle["H_floor"] = bundle["H"];
}

// ===== 채널별 저장소 (v0.4) =============================================
// 캘리브레이션은 채널마다 완전히 다르다(렌즈 방향이 다름). 그래서 저장 형식이
// 번들 하나에서 채널별 맵으로 바뀌었다:  {"1": {…번들…}, "2": {…번들…}}
//
// 이미 배포된 파일(users.json / calib_latest.json)에는 예전 형식(번들 하나)이
// 들어 있으므로, 읽을 때 그것을 "채널 1의 번들"로 승격시켜 준다 - 마이그레이션
// 스크립트를 돌리지 않아도 기존 현장이 그대로 뜬다.

// 채널별 맵인가? 키가 전부 숫자 문자열이면 맵으로 본다.
// 번들은 키가 H_floor/K/calib_id... 라서 절대 이 조건에 걸리지 않고, 레거시
// 번들은 아예 배열이라 걸리지 않는다.
inline bool isCalibChannelMap(const json& j) {
    if (!j.is_object() || j.empty()) return false;
    for (auto it = j.begin(); it != j.end(); ++it) {
        const std::string& k = it.key();
        if (k.empty()) return false;
        for (char c : k)
            if (c < '0' || c > '9') return false;
    }
    return true;
}

// 저장된 값(맵 | 예전 번들 하나 | null) -> 항상 채널별 맵.
inline json asCalibChannelMap(const json& stored) {
    if (stored.is_null()) return json::object();
    if (isCalibChannelMap(stored)) return stored;
    // 예전 형식: 번들 하나뿐이었다 -> 채널 1의 것으로 읽는다
    return json{{chKey(kMinChannel), stored}};
}

// 채널별 맵에서 한 채널의 번들을 꺼낸다 (없으면 null).
inline json calibOfChannel(const json& stored, int ch) {
    const json m = asCalibChannelMap(stored);
    auto it = m.find(chKey(ch));
    return (it == m.end()) ? json() : *it;
}

// H_MATRIX payload의 번들(json) -> Calib. 신규 오브젝트/레거시 3x3 배열 모두 허용.
inline bool calibFromJson(const json& bundle, Calib& out) {
    out = Calib{};
    if (bundle.is_array()) {  // 레거시: H 하나 -> 바닥/마커 공용, 왜곡 보정 없음
        if (!parseMat3(bundle, out.Hf)) return false;
        std::copy(&out.Hf[0][0], &out.Hf[0][0] + 9, &out.Hm[0][0]);
        out.hasFloor = out.hasMarker = out.valid = true;
        out.raw = bundle;
        return true;
    }
    if (!bundle.is_object()) return false;
    out.hasFloor = bundle.contains("H_floor") && parseMat3(bundle["H_floor"], out.Hf);
    // 평면 스키마는 바닥 H의 이름이 "H"다 (H_floor 없음). H_floor를 먼저 보고,
    // 없을 때만 H로 넘어간다 - 둘 다 있으면 명시적인 H_floor가 이긴다.
    if (!out.hasFloor)
        out.hasFloor = bundle.contains("H") && parseMat3(bundle["H"], out.Hf);
    out.hasMarker = bundle.contains("H_marker") && parseMat3(bundle["H_marker"], out.Hm);
    if (!out.hasFloor && !out.hasMarker) return false;
    if (!out.hasMarker) {  // 마커용 H가 없으면 바닥 H로 대체 (시차 보정 없이 동작)
        std::copy(&out.Hf[0][0], &out.Hf[0][0] + 9, &out.Hm[0][0]);
    } else if (!out.hasFloor) {
        std::copy(&out.Hm[0][0], &out.Hm[0][0] + 9, &out.Hf[0][0]);
    }
    // ÷1000은 여기 한 곳에서만 일어난다. out.raw(저장·중계본)는 mm 그대로 두고,
    // 서버 내부 좌표 계산에 쓰는 Hf/Hm만 미터로 만든다.
    const double s = bundleMeterScale(bundle);
    if (s != 1.0) {
        scaleMat3Rows01(out.Hf, s);
        scaleMat3Rows01(out.Hm, s);
    }
    double K[3][3];
    if (bundle.contains("K") && parseMat3(bundle["K"], K) && bundle.contains("D") &&
        bundle["D"].is_array() && bundle["D"].size() >= 4) {
        out.fx = K[0][0], out.fy = K[1][1], out.cx = K[0][2], out.cy = K[1][2];
        for (size_t i = 0; i < 5 && i < bundle["D"].size(); ++i)
            out.D[i] = bundle["D"][i].get<double>();
        out.hasKD = (out.fx != 0 && out.fy != 0);
    }
    out.valid = true;
    out.raw = bundle;
    return true;
}

// 렌즈 왜곡 보정: 원본 픽셀 -> 왜곡 보정된 픽셀.
// cv::undistortPoints(pts, K, D, noArray(), K) 와 동일 (P=K: 결과를 픽셀 단위로 유지).
// H_floor/H_marker가 "왜곡 보정된 픽셀" 기준으로 캘리브레이션되므로 H 적용 전 필수.
inline void undistortPixel(const Calib& c, double u, double v, double& uo, double& vo) {
    if (!c.hasKD) {
        uo = u, vo = v;
        return;
    }
    double k1 = c.D[0], k2 = c.D[1], p1 = c.D[2], p2 = c.D[3], k3 = c.D[4];
    double xd = (u - c.cx) / c.fx, yd = (v - c.cy) / c.fy;  // 왜곡된 정규화 좌표
    double x = xd, y = yd;
    for (int i = 0; i < 5; ++i) {  // 왜곡 모델 역산 (고정점 반복)
        double r2 = x * x + y * y;
        double radial = 1 + r2 * (k1 + r2 * (k2 + r2 * k3));
        double dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
        double dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
        x = (xd - dx) / radial;
        y = (yd - dy) / radial;
    }
    uo = c.fx * x + c.cx;
    vo = c.fy * y + c.cy;
}

// H(3x3) 사영변환: 왜곡 보정된 픽셀 -> 평면 미터 좌표
inline bool applyH(const double H[3][3], double u, double v,
                   std::array<double, 2>& out) {
    double r[3];
    for (int i = 0; i < 3; ++i) r[i] = H[i][0] * u + H[i][1] * v + H[i][2];
    if (std::fabs(r[2]) < 1e-12) return false;
    out = {r[0] / r[2], r[1] / r[2]};
    return true;
}

// 원본 픽셀 -> 마커 높이 평면 미터 (undistort -> H_marker)
inline bool pixelToMarkerPlane(const Calib& c, double u, double v,
                               std::array<double, 2>& out) {
    if (!c.valid) return false;
    double uu, vv;
    undistortPixel(c, u, v, uu, vv);
    return applyH(c.Hm, uu, vv, out);
}
