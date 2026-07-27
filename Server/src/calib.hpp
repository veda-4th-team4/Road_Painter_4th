#pragma once
// 캘리브레이션 데이터 (CCTV가 H_MATRIX로 올리는 번들) 파싱/적용.
//
// 좌표계 규약:
//   - CCTV는 마커 코너를 "원본 픽셀 좌표"로만 보낸다 (변환 금지).
//   - 서버가 undistort(왜곡 보정) -> H_marker 적용으로 바닥 좌표를 계산한다.
//   - CCTV가 보내는 H_floor/H_marker는 "왜곡 보정된 픽셀 -> 월드 평면 mm" 사영변환이다.
//     서버는 수신 즉시 mm -> m 로 정규화(normalizeBundleMmToM, ÷1000)하므로, 아래
//     Calib 구조체(Hf/Hm)와 이후 모든 서버 좌표(pose/POSE/BLUEPRINT/PATH)는 미터 기준이다.
//     (H_marker = 마커 장착 높이 평면용. 마커가 바닥에서 떠 있어 생기는
//      시차(parallax)를 캘리브레이션 단계에서 흡수한 것)
//
// 번들 포맷 (신규):
//   {"version":1, "K":[[fx,0,cx],[0,fy,cy],[0,0,1]], "D":[k1,k2,p1,p2,k3],
//    "H_floor":[[...]x3], "H_marker":[[...]x3], "marker_height_m":0.25}
// 레거시 포맷 (v0.2 이하): [[...]x3] 단일 H -> H_floor=H_marker=H, 왜곡 보정 생략
#include "protocol.hpp"
#include <array>
#include <cmath>

struct Calib {
    bool valid = false;      // H가 하나라도 있으면 true
    bool hasKD = false;      // K/D가 있어야 왜곡 보정 수행
    double fx = 1, fy = 1, cx = 0, cy = 0;   // K (내부 파라미터)
    std::array<double, 5> D{};               // 왜곡 계수 (k1,k2,p1,p2,k3)
    double Hf[3][3] = {};    // 왜곡 보정 픽셀 -> 바닥 평면 (m)
    double Hm[3][3] = {};    // 왜곡 보정 픽셀 -> 마커 높이 평면 (m)
    bool hasFloor = false, hasMarker = false;
    json raw;                // 영속 저장/QT 중계용 원본 번들
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

// CCTV는 항상 mm 기준(pixel -> world mm) 호모그래피를 보낸다는 전제로, 서버 입구에서
// 미터로 정규화(÷1000)한다. 이후 pose 계산/BLUEPRINT/POSE/PATH/QT top-view가 전부
// 미터로 통일된다 (로봇에 나가는 dist_m도 그대로 미터). H_floor/H_marker(신규) 및
// 레거시 단일 H 배열을 처리한다. K/D는 픽셀 단위라 건드리지 않는다.
inline void normalizeBundleMmToM(json& bundle) {
    const double s = 0.001;  // mm -> m
    if (bundle.is_array()) {  // 레거시: 단일 H
        scaleMat3Rows01(bundle, s);
        return;
    }
    if (!bundle.is_object()) return;
    if (bundle.contains("H_floor")) scaleMat3Rows01(bundle["H_floor"], s);
    if (bundle.contains("H_marker")) scaleMat3Rows01(bundle["H_marker"], s);
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
    out.hasMarker = bundle.contains("H_marker") && parseMat3(bundle["H_marker"], out.Hm);
    if (!out.hasFloor && !out.hasMarker) return false;
    if (!out.hasMarker) {  // 마커용 H가 없으면 바닥 H로 대체 (시차 보정 없이 동작)
        std::copy(&out.Hf[0][0], &out.Hf[0][0] + 9, &out.Hm[0][0]);
    } else if (!out.hasFloor) {
        std::copy(&out.Hm[0][0], &out.Hm[0][0] + 9, &out.Hf[0][0]);
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
