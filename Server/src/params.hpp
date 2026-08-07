#pragma once
// ===== 튜닝 파라미터 한 곳 모음 (프로토콜 v2) =====
//
// 🔴 임계값·주기·기하 상수를 코드에 박지 않는다. 전부 여기 한 곳에 모아두고,
//   서버 시작 때 config/params.json으로 덮어쓸 수 있게 한다. 현장에서 값 하나
//   바꾸려고 재컴파일·재배포하지 않기 위한 것 - 파일 고치고 서버만 재시작하면 된다.
//
// 사용법:
//   - 코드에서: params().align_threshold_deg  (읽기 전용으로 쓸 것)
//   - 현장에서: config/params.json 편집 후 서버 재시작
//   - 파일이 없으면 아래 기본값 그대로 동작한다 (파일은 선택 사항)
//
// 값의 근거는 docs/PROTOCOL_v2_ROBOT.md §10 상수표에 정리되어 있다.
// 새 상수를 추가할 때는 아래 RP_PARAM_LIST에 한 줄 넣기만 하면
// 구조체 필드 / JSON 로딩 / 시작 로그 덤프 / 오타 검출이 전부 따라온다.
#include "log.hpp"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

using json = nlohmann::json;

// X(타입, 이름, 기본값, 설명)
#define RP_PARAM_LIST(X)                                                       \
    /* ---- 기하 (로봇 하드웨어 실측값) ---- */                                \
    X(double, pen_offset_m, 0.155,                                             \
      "펜(노즐)이 마커 중심 뒤로 떨어진 거리 a. 로봇 PathFollower.h와 같아야 함")\
    X(double, wheel_base_m, 0.166,                                             \
      "좌우 바퀴 축간거리 W(로봇팀 실측). arc 안쪽바퀴 역회전 경고 판정에만 사용")\
    X(double, min_paint_radius_m, 0.155,                                       \
      "도색 가능한 최소 펜 반지름. 이론 하한 = a. 미만이면 도면 거부")         \
    /* ---- ALIGN (turn 직후 각도 정렬) ---- */                                \
    X(double, align_threshold_deg, 4.0,                                        \
      "이 각도 이내면 정렬 완료로 보고 GO")                                    \
    X(int, align_max_tries, 6,                                                 \
      "한 boundary에서 ALIGN 최대 반복. 소진하면 포기하고 GO")                 \
    /* ---- MORE (move/arc 직후 거리 보정) ---- */                             \
    X(double, more_deadband_m, 0.03,                                           \
      "이 거리 이내면 보정하지 않고 GO")                                       \
    X(double, more_max_m, 0.5,                                                 \
      "이보다 큰 보정량은 물리적으로 말이 안 되므로 판정 폐기 + GO")           \
    X(int, more_max_tries, 4,                                                  \
      "한 boundary에서 MORE 최대 반복. 소진하면 포기하고 GO")                  \
    /* ---- 피드백 판정 대기 창 ---- */                                        \
    X(long, feedback_wait_ms, 2000,                                            \
      "READY 수신 후 판정까지 고정 대기. 이 동안 POS를 모아 평균낸다")         \
    /* ---- DRIFT (직진 주행 중 각도 피드백) ---- */                           \
    X(long, drift_period_ms, 400, "DRIFT 최소 전송 간격")                      \
    X(double, drift_deadband_deg, 1.0, "이 각도 미만이면 DRIFT를 보내지 않음") \
    /* ---- POS 두절 / 이상치 게이트 ---- */                                   \
    X(long, pos_lost_ms, 2000, "마지막 채택 POS로부터 이만큼 지나면 HOLD")     \
    X(int, pos_recover_frames, 2, "HOLD 해제에 필요한 연속 채택 POS 장수")     \
    X(double, pose_gate_base_deg, 3.0, "이상치 게이트 상수항 (측정 노이즈 몫)")\
    X(double, pose_gate_rate_dps, 40.0, "이상치 게이트 속도항 (실제 회전 몫)") \
    X(int, pose_reject_max, 5, "연속 거부 한도. 넘으면 받아들여 재동기")       \
    X(long, pos_stat_period_ms, 10000, "POS 수신 요약 로그 주기")              \
    /* ---- 경로 생성 ---- */                                                  \
    X(double, min_turn_deg, 2.0, "이 각도 미만의 회전은 op으로 만들지 않음")   \
    X(double, min_move_m, 0.01, "이 거리 미만의 이동은 op으로 만들지 않음")

struct Params {
#define RP_DECL(T, N, D, C) T N = D;
    RP_PARAM_LIST(RP_DECL)
#undef RP_DECL
};

// 전역 파라미터. loadParams()는 스레드 시작 전(main 초반)에 한 번만 부르고,
// 그 뒤로는 읽기 전용이라 별도 동기화가 필요 없다.
inline Params& params() {
    static Params p;
    return p;
}

namespace param_detail {

// JSON 값 -> 필드. 키가 없으면 기본값 유지, 타입이 어긋나면 경고 후 기본값 유지.
// (타입 오류로 조용히 0이 되면 현장에서 원인을 찾기 어렵다)
inline void readOne(const json& j, const char* key, double& dst) {
    auto it = j.find(key);
    if (it == j.end()) return;
    if (!it->is_number()) {
        logf("[WARN] params '%s' 값이 숫자가 아님 - 기본값 유지", key);
        return;
    }
    dst = it->get<double>();
}
inline void readOne(const json& j, const char* key, int& dst) {
    auto it = j.find(key);
    if (it == j.end()) return;
    if (!it->is_number()) {
        logf("[WARN] params '%s' 값이 숫자가 아님 - 기본값 유지", key);
        return;
    }
    dst = (int)std::llround(it->get<double>());
}
inline void readOne(const json& j, const char* key, long& dst) {
    auto it = j.find(key);
    if (it == j.end()) return;
    if (!it->is_number()) {
        logf("[WARN] params '%s' 값이 숫자가 아님 - 기본값 유지", key);
        return;
    }
    dst = (long)std::llround(it->get<double>());
}

inline std::string show(double v) {
    char b[32];
    snprintf(b, sizeof b, "%g", v);
    return b;
}
inline std::string show(int v) { return std::to_string(v); }
inline std::string show(long v) { return std::to_string(v); }

// 물리적으로/논리적으로 말이 안 되는 값은 여기서 잡는다. 잘못된 params.json
// 하나로 로봇이 이상하게 움직이는 것보다, 시작 시점에 경고 한 줄이 낫다.
inline void sanitize(Params& p) {
    auto floorAt = [](const char* name, auto& v, auto lo) {
        if (v < lo) {
            logf("[WARN] params '%s'=%s 은(는) 너무 작음 - %s로 올림", name,
                 show(v).c_str(), show((decltype(v))lo).c_str());
            v = (decltype(v))lo;
        }
    };
    floorAt("pen_offset_m", p.pen_offset_m, 0.0);
    floorAt("wheel_base_m", p.wheel_base_m, 0.001);
    floorAt("align_max_tries", p.align_max_tries, 0);
    floorAt("more_max_tries", p.more_max_tries, 0);
    floorAt("align_threshold_deg", p.align_threshold_deg, 0.0);
    floorAt("more_deadband_m", p.more_deadband_m, 0.0);
    floorAt("more_max_m", p.more_max_m, 0.0);
    floorAt("feedback_wait_ms", p.feedback_wait_ms, 0L);
    floorAt("drift_period_ms", p.drift_period_ms, 0L);
    floorAt("pos_lost_ms", p.pos_lost_ms, 0L);
    floorAt("pos_recover_frames", p.pos_recover_frames, 1);
    floorAt("pose_reject_max", p.pose_reject_max, 1);
    floorAt("pos_stat_period_ms", p.pos_stat_period_ms, 1000L);
    // 펜이 중심 뒤 a에 달려 있는 이상 노즐이 그릴 수 있는 최소 반지름은 a다
    // (docs/PROTOCOL_v2_ROBOT.md §5.5). 그보다 작은 하한은 의미가 없다.
    if (p.min_paint_radius_m < p.pen_offset_m) {
        logf("[WARN] params 'min_paint_radius_m'=%g < 'pen_offset_m'=%g - "
             "물리 하한인 %g로 올림 (§5.5)",
             p.min_paint_radius_m, p.pen_offset_m, p.pen_offset_m);
        p.min_paint_radius_m = p.pen_offset_m;
    }
}

}  // namespace param_detail

// config/params.json을 읽어 기본값을 덮어쓴다. 파일이 없으면 기본값 그대로
// (경고가 아니라 정보 로그 - 파일은 선택 사항이다).
// 적용된 값은 항상 전부 로그로 남긴다: "지금 서버가 실제로 쓰는 값"을 로그만
// 보고 확인할 수 있어야 현장 튜닝이 성립한다.
inline void loadParams(const std::string& path) {
    Params& p = params();
    std::ifstream f(path);
    if (!f) {
        logf("[INFO] 파라미터 파일 없음 (%s) - 기본값 사용", path.c_str());
    } else {
        json j;
        try {
            f >> j;
        } catch (const std::exception& e) {
            logf("[WARN] 파라미터 파일 파싱 실패 (%s: %s) - 기본값 사용",
                 path.c_str(), e.what());
            j = json::object();
        }
        if (!j.is_object()) {
            logf("[WARN] 파라미터 파일이 객체가 아님 (%s) - 기본값 사용",
                 path.c_str());
            j = json::object();
        }
        // 모르는 키는 십중팔구 오타다. 조용히 무시하면 "값을 바꿨는데 안 먹는다"가
        // 되므로 반드시 소리를 낸다.
        std::vector<std::string> known;
#define RP_KNOWN(T, N, D, C) known.push_back(#N);
        RP_PARAM_LIST(RP_KNOWN)
#undef RP_KNOWN
        for (auto it = j.begin(); it != j.end(); ++it) {
            if (it.key().rfind("_", 0) == 0) continue;  // "_comment" 등은 주석 취급
            if (std::find(known.begin(), known.end(), it.key()) == known.end())
                logf("[WARN] params '%s' - 모르는 항목 (오타?) 무시",
                     it.key().c_str());
        }
#define RP_READ(T, N, D, C) param_detail::readOne(j, #N, p.N);
        RP_PARAM_LIST(RP_READ)
#undef RP_READ
        logf("[INFO] 파라미터 파일 적용: %s", path.c_str());
    }
    param_detail::sanitize(p);
    logf("[INFO] ----- 적용된 파라미터 -----");
#define RP_DUMP(T, N, D, C)                                                    \
    logf("[INFO]   %-22s = %-8s %s", #N, param_detail::show(p.N).c_str(),      \
         (param_detail::show(p.N) == param_detail::show((T)(D)) ? "" : "(변경됨)"));
    RP_PARAM_LIST(RP_DUMP)
#undef RP_DUMP
    logf("[INFO] ---------------------------");
}
