// 캘리브레이션 번들 단위 규약 회귀 테스트
// (QT_CCTV_SERVER_CALIBRATION_FORMAT 2026-08-11 §1 "단위 통일 결정", §4).
//   make calib_unit_test && ./tools/calib_unit_test
//
// 규약은 한 줄이다: **번들은 mm로 저장·중계하고, ÷1000은 서버 내부 좌표 사본에서만.**
//
// 이 파일이 존재하는 이유는 이 부류의 버그가 조용하기 때문이다. 단위가 1000배
// 어긋나도 예외도 파싱 에러도 안 나고, 로그는 정상으로 찍히며, 좌표는 "그럴듯한
// 숫자"로 나온다. 현장에서 로봇이 엉뚱한 데로 가야 비로소 드러난다.
//
// 특히 못박아 두는 것:
//  1) mm 번들을 받으면 raw(저장·중계본)는 mm 그대로, Hf/Hm만 미터가 된다.
//  2) 예전 서버가 미터로 저장해 둔 번들도 결국 같은 미터 좌표를 낸다.
//     (bundleToMm 으로 mm 복원 -> calibFromJson 이 다시 ÷1000. 왕복 항등)
//     ⚠️ 여기가 깨지면 이미 배포된 현장 users.json 의 캘리가 1000배 틀어진다.
//  3) bundleToMm 은 멱등이다 (mm 번들을 또 부르면 ×1000 되지 않는다).
#include "calib.hpp"
#include <cstdio>
#include <cmath>

static int fails = 0;
#define CHECK(cond, ...)                                                       \
    do {                                                                       \
        if (cond) { printf("  ok   "); } else { printf("  FAIL "); ++fails; }   \
        printf(__VA_ARGS__); printf("\n");                                      \
    } while (0)

// 규격 문서 §3 에 실린 실제 CH2 번들의 H_floor (mm 기준).
static const double kHmm[3][3] = {
    {1.818503819, 0.5278524691, -2129.003704},
    {0.2240010208, -1.972624096, 1934.719311},
    {-0.00003211646644, 0.0006737258966, 1}};

static json mat3(const double m[3][3]) {
    json j = json::array();
    for (int r = 0; r < 3; ++r) j.push_back({m[r][0], m[r][1], m[r][2]});
    return j;
}

// 정규형(2026-08-11) 번들. unit 을 넣을지 말지는 인자로 고른다.
static json specBundle(bool withUnit) {
    json b{{"calib_id", "2026-08-11-170933"},
           {"image_size", {2592, 1520}},
           {"coord_mode", "undistort"},
           {"K", {{1728.6862, 0, 1243.5087}, {0, 1730.2905, 658.448}, {0, 0, 1}}},
           {"D", {-0.75924926271, 3.3407040464, 0.031775252611, -0.0013203081515,
                  -7.9783995166}},
           {"H_floor", mat3(kHmm)},
           {"H_marker", mat3(kHmm)},
           {"marker_height_mm", 160},
           {"origin_mm", {0, 0}},
           {"canvas_mm", {900, 600}},
           {"axis", "x_right_y_up"}};
    if (withUnit) b["unit"] = "mm";
    return b;
}

// 예전 서버가 남긴 저장분: 수신 즉시 ÷1000 해서 저장했고 unit 은 없거나 "m" 이었다.
static json legacyStored(bool taggedM) {
    json b = specBundle(false);
    scaleMat3Rows01(b["H_floor"], 0.001);
    scaleMat3Rows01(b["H_marker"], 0.001);
    if (taggedM) b["unit"] = "m";
    return b;
}

// 테스트 픽셀 한 점을 mm 행렬로 직접 변환해 "정답"을 만든다 (미터).
// 손으로 계산한 상수를 박아두지 않는다 - 그러면 오타가 곧 통과가 된다.
static void expectMeters(double u, double v, double& x, double& y) {
    double r[3];
    for (int i = 0; i < 3; ++i) r[i] = kHmm[i][0] * u + kHmm[i][1] * v + kHmm[i][2];
    x = (r[0] / r[2]) * 0.001;  // mm -> m
    y = (r[1] / r[2]) * 0.001;
}

static bool near(double a, double b) { return std::fabs(a - b) < 1e-9; }

// 번들을 파싱해 나온 Hf 가 기대 미터 좌표를 내는가.
static bool metersMatch(const json& bundle, const char* what) {
    const double u = 1243.5087, v = 658.448;
    double ex, ey;
    expectMeters(u, v, ex, ey);
    Calib c;
    if (!calibFromJson(bundle, c)) {
        printf("  FAIL %s: calibFromJson 실패\n", what);
        ++fails;
        return false;
    }
    std::array<double, 2> got{};
    if (!applyH(c.Hf, u, v, got)) {
        printf("  FAIL %s: applyH 실패\n", what);
        ++fails;
        return false;
    }
    const bool ok = near(got[0], ex) && near(got[1], ey);
    CHECK(ok, "%s -> (%.6f, %.6f) m (기대 %.6f, %.6f)", what, got[0], got[1], ex, ey);
    return ok;
}

int main() {
    printf("[1] 정규형 mm 번들: raw 는 mm 그대로, Hf 만 미터\n");
    {
        json b = specBundle(true);
        CHECK(!stampCalibUnit(b), "unit 이 이미 있으면 서버가 가정하지 않는다");
        CHECK(b["unit"] == "mm", "unit 은 mm 그대로");
        CHECK(near(b["H_floor"][0][2].get<double>(), -2129.003704),
              "저장·중계본의 H_floor 는 mm 그대로 (÷1000 되지 않는다)");
        metersMatch(b, "mm 번들");
        Calib c;
        calibFromJson(b, c);
        CHECK(near(c.raw["H_floor"][0][2].get<double>(), -2129.003704),
              "Calib::raw 도 mm (Qt 로 이 값이 나간다)");
        CHECK(near(c.Hf[0][2], -2.129003704), "Calib::Hf 만 미터");
    }

    printf("[2] unit 없는 CCTV 번들: mm 으로 가정하고 태그를 박는다\n");
    {
        json b = specBundle(false);
        CHECK(stampCalibUnit(b), "unit 이 없으면 서버가 박았다고 알려준다");
        CHECK(b["unit"] == "mm", "mm 으로 가정");
        CHECK(near(b["H_floor"][0][2].get<double>(), -2129.003704), "값은 안 건드린다");
        metersMatch(b, "unit 없는 mm 번들");
    }

    printf("[3] 예전 서버 저장분(미터) -> 같은 미터 좌표가 나오는가\n");
    for (int tagged = 0; tagged <= 1; ++tagged) {
        const char* what = tagged ? "unit=\"m\" 저장분" : "unit 없는 저장분";
        json b = legacyStored(tagged != 0);
        // 마이그레이션 전에도 좌표는 맞아야 한다 (bundleMeterScale=1.0 로 읽힌다)
        metersMatch(b, tagged ? "저장분(unit=m), 환산 전" : "저장분(unit 없음), 환산 전");
        CHECK(bundleToMm(b), "%s -> mm 으로 환산됐다", what);
        CHECK(b["unit"] == "mm", "환산 후 unit=mm");
        CHECK(near(b["H_floor"][0][2].get<double>(), -2129.003704),
              "환산 결과가 원본 mm 과 일치 (Qt 로 나갈 값)");
        metersMatch(b, tagged ? "저장분(unit=m), 환산 후" : "저장분(unit 없음), 환산 후");
    }

    printf("[4] bundleToMm 은 멱등 (두 번 불러도 ×1000 되지 않는다)\n");
    {
        json b = legacyStored(false);
        bundleToMm(b);
        CHECK(!bundleToMm(b), "이미 mm 이면 아무것도 안 한다");
        CHECK(near(b["H_floor"][0][2].get<double>(), -2129.003704), "값도 그대로");
        metersMatch(b, "두 번 환산한 번들");
    }

    printf("[5] 평면 스키마(바닥 H 의 이름이 \"H\")도 같은 규약을 탄다\n");
    {
        json b = specBundle(true);
        b["H"] = b["H_floor"];
        b.erase("H_floor");
        stampCalibUnit(b);
        aliasFloorKey(b);
        CHECK(b.contains("H_floor"), "H -> H_floor 별칭이 붙는다 (Qt 는 H_floor 만 본다)");
        CHECK(near(b["H"][0][2].get<double>(), -2129.003704), "원본 H 도 mm 보존");
        metersMatch(b, "평면 스키마");
        json m = legacyStored(false);
        m["H"] = m["H_floor"];
        m.erase("H_floor");
        bundleToMm(m);
        CHECK(near(m["H"][0][2].get<double>(), -2129.003704),
              "평면 스키마 저장분도 H 가 같이 환산된다 (한 번들 안 단위가 갈리면 안 됨)");
    }

    printf("[6] 레거시 단일 H 배열 (v0.2): 예전대로 수신 때 미터로 바꿔 보관\n");
    {
        json arr = mat3(kHmm);
        CHECK(!stampCalibUnit(arr), "배열은 태그할 자리가 없다");
        CHECK(near(arr[0][2].get<double>(), -2.129003704), "수신 때 미터로 변환됨");
        CHECK(near(bundleMeterScale(arr), 1.0), "이후 파싱은 그대로 (이중 변환 금지)");
        metersMatch(arr, "레거시 배열");
        CHECK(!bundleToMm(arr), "레거시 배열은 mm 환산 대상이 아니다");
    }

    printf("\n%s (%d fail)\n", fails ? "실패" : "전부 통과", fails);
    return fails ? 1 : 0;
}
