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
    X(double, pen_offset_m, 0.150,                                             \
      "펜(노즐)이 마커 중심 뒤로 떨어진 거리 a. 로봇 PathFollower.h와 같아야 함")\
    X(double, wheel_base_m, 0.166,                                             \
      "좌우 바퀴 축간거리 W(로봇팀 실측). arc 안쪽바퀴 역회전 경고 판정에만 사용")\
    X(double, min_paint_radius_m, 0.200,                                       \
      "도색 가능한 최소 펜 반지름. 미만이면 도면 거부(arc_too_tight). "        \
      "이론 하한은 a(0.155)지만 실제 하한은 모터가 정한다 - 아래 주석 참조")   \
    /* ---- ALIGN (turn 직후 각도 정렬) ---- */                                \
    X(double, align_threshold_deg, 2.0,                                        \
      "이 각도 이내면 정렬 완료로 보고 GO")                                    \
    X(int, align_max_tries, 6,                                                 \
      "한 boundary에서 ALIGN 최대 반복. 소진하면 포기하고 GO")                 \
    /* ---- MORE (move/arc 직후 거리 보정) ---- */                             \
    X(double, more_deadband_m, 0.005,                                          \
      "이 거리 이내면 보정하지 않고 GO")                                       \
    X(double, more_max_m, 0.5,                                                 \
      "이보다 큰 보정량은 물리적으로 말이 안 되므로 판정 폐기 + GO")           \
    X(int, more_max_tries, 4,                                                  \
      "한 boundary에서 MORE 최대 반복. 소진하면 포기하고 GO")                  \
    /* ---- 피드백 판정 대기 창 ---- */                                        \
    X(long, feedback_wait_ms, 1000,                                            \
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
    X(double, min_move_m, 0.01, "이 거리 미만의 이동은 op으로 만들지 않음")    \
    /* ---- 로봇 주행 호모그래피 세션 (2026-08-10) ---- */                     \
    X(long, calib_timeout_ms, 180000,                                          \
      "CALIB_START 후 이만큼 지나도 종결 응답이 없으면 CALIB_FAIL{timeout}. "  \
      "🔴 Qt 자체 타임아웃(5분)보다 반드시 짧아야 한다 - 아래 주석 참조")      \
    X(long, calib_cancel_ack_ms, 5000,                                         \
      "CALIB_CANCEL 후 ROBOT/CCTV의 CALIB_STOPPED를 기다리는 한도. "           \
      "넘으면 CALIB_FAIL{cancel_failed} (정지를 추정으로 확인하지 않는다)")

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
    floorAt("calib_cancel_ack_ms", p.calib_cancel_ack_ms, 1000L);
    // 0을 넣으면 CALIB_START가 수락되자마자 타임아웃으로 죽는다. 하한을 두는
    // 편이 "왜 캘리가 즉시 실패하지"를 현장에서 찾는 것보다 싸다.
    floorAt("calib_timeout_ms", p.calib_timeout_ms, 5000L);
    // 🔴 Qt는 종결 응답이 5분(300s) 없으면 스스로 대기를 푼다. 서버 타임아웃이
    // 그보다 길면 Qt는 이미 포기했는데 서버만 busy로 남아, 다음 요청이 전부
    // busy로 거절된다 (사람 눈에는 "캘리가 영영 안 되는" 상태로 보인다).
    if (p.calib_timeout_ms >= 300000L) {
        logf("[WARN] params 'calib_timeout_ms'=%ld 은(는) Qt 대기 한도(300000)"
             " 이상 - 290000으로 내림", p.calib_timeout_ms);
        p.calib_timeout_ms = 290000L;
    }
    // min_paint_radius_m 기본값 0.200은 기하가 아니라 모터가 정한 값이다.
    // 호에서는 바깥 바퀴가 base_sps × (R_robot + W/2) / R_robot 으로 돌고,
    // 이 값이 1/R_robot 로 발산한다. 로봇팀 회신(2026-08-07): NEMA 17 기준
    // 2000 SPS를 넘으면 토크가 떨어져 탈조 위험. R_paint=0.200 -> 1278 SPS(권장),
    // 0.180 -> 약 1450 SPS(마진 하한). 그래서 0.200을 기본으로 둔다.
    //
    // 🔴 이 값은 로봇의 호 주행 속도(base_sps=771.65, 0.05 m/s)에 묶여 있다.
    //   로봇이 호를 더 빠르게 돌게 되면 같은 반지름에서 SPS가 비례해 올라가므로
    //   이 하한도 같이 올려야 한다. 속도를 바꿀 때 반드시 재계산할 것.
    //
    // 아래 검사는 그와 별개인 기하학적 하한이다. 펜이 중심 뒤 a에 달려 있는 이상
    // 노즐이 그릴 수 있는 반지름은 a 미만이 될 수 없다 (PROTOCOL_v2_ROBOT.md §5.5).
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
