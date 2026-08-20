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

// Qt가 캘리 종결 응답(H_MATRIX|CALIB_FAIL|CALIB_CANCELLED)을 기다리는 한도.
// Qt 클라이언트 쪽 상수라 서버가 바꿀 수 없다 - params.json으로 덮어쓸 수 있게
// 두면 "현장에서 늘렸는데 Qt는 그대로"라는 어긋남만 만든다. 그래서 코드 상수다.
// 🔴 Qt팀이 이 값을 바꾸면 여기도 같이 바꿔야 한다
// (docs/ROBOT_ODOMETRY_HOMOGRAPHY_REQUEST_QT_20260813.md §4).
inline constexpr long kQtCalibWaitMs = 300000L;
// 서버가 쓸 수 있는 실효 예산. Qt가 스스로 포기하기 전에 서버가 먼저 종결
// 응답을 보내야 하므로 한 뼘 짧게 잡는다.
inline constexpr long kQtCalibWaitCapMs = 290000L;

// 🔴 robot_motion 은 Qt 대기 한도가 다르다 - 10분이다
// (2026-08-13 Qt팀 회신 §6, 요청서 §4의 선택지 A 채택).
//
// 위의 5분을 그대로 쓰면 서버가 주행을 3분 50초(290s - 결과대기 60s)에 끊는다.
// Qt는 6분을 더 기다릴 준비가 돼 있는데 서버가 먼저 죽이는 꼴이고, 하필 첫
// 실기 주행이 제일 느린 조건이다 - 정지점 9개 × 카메라 정지판정 최대 10초에
// IMU 폐루프 회전 재시도가 얹힌다. 방식별로 상수를 나눈다.
inline constexpr long kQtOdoWaitMs = 600000L;
inline constexpr long kQtOdoWaitCapMs = 590000L;

// X(타입, 이름, 기본값, 설명)
#define RP_PARAM_LIST(X)                                                       \
    /* ---- 기하 (로봇 하드웨어 실측값) ---- */                                \
    X(double, pen_offset_m, 0.150,                                             \
      "펜(노즐)이 마커 중심 뒤로 떨어진 거리 a. 로봇 PathFollower.h와 같아야 함")\
    X(double, pen_width_m, 0.05,                                               \
      "펜이 실제로 칠하는 폭 w (마커 사양 5cm). 도색 구간을 앞뒤로 w/2씩 늘려 "\
      "꼭짓점 귀퉁이가 비지 않게 한다 - docs/PEN_WIDTH_COMPENSATION_20260813.md. "\
      "0으로 두면 보정이 통째로 꺼진다(예전 동작). 실측값을 넣을 것 - 마커 "   \
      "사양보다 압력·속도에 따라 달라진다")                                    \
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
      "넘으면 CALIB_FAIL{cancel_failed} (정지를 추정으로 확인하지 않는다)")     \
    /* ---- 로봇 오도메트리 주행 캘리 (2026-08-12) ---- */                     \
    X(long, calib_odo_timeout_ms, 300000,                                      \
      "오도메트리 주행 세션(method=robot_motion)의 **주행** 데드라인 "         \
      "(CALIB_START -> CALIB_DONE). calib_timeout_ms와 별개 값이다. 목적은 "   \
      "주행 시간을 조이는 게 아니라 로봇/카메라가 죽었을 때 세션을 접는 "      \
      "워치독. ⚠️ ADMIN 개시일 때의 값이다 - QT 개시 세션은 Qt 대기 한도에 "   \
      "맞춰 kQtOdoWaitCapMs - calib_odo_result_wait_ms 로 깎인다")             \
    X(long, calib_odo_result_wait_ms, 60000,                                   \
      "CALIB_DONE 전송 후 카메라의 H_MATRIX/CALIB_FAIL을 기다리는 한도. "      \
      "주행이 끝난 뒤 카메라가 findHomography + LOO를 도는 구간이다 - 이 "     \
      "동안 로봇은 이미 서 있으므로 만료 시 abortOdoCalib이 아니라 "          \
      "failCalib(timeout)으로 접는다 (세울 로봇이 없다)")                      \
    X(long, calib_capture_timeout_ms, 15000,                                   \
      "CALIB_CAPTURE 전송 후 CCTV의 ack(OK/FAIL) 한도. 카메라 자체 정지판정 "  \
      "한도(10초)보다 여유 있게 잡는다. 넘으면 세션 중단(abortOdoCalib)")      \
    X(double, calib_odo_max_side_m, 10.0,                                      \
      "오도메트리 캘리 사각형 한 변의 상한. 이 값을 넘는 m_cm/n_cm은 "         \
      "invalid_param으로 거절한다. 하한(min_move_m)과 달리 기하가 아니라 "     \
      "**현장 안전** 값이다 - 90을 900으로 잘못 치면 로봇이 9m를 그대로 "      \
      "달린다. Qt는 하한만 검증하고 상한이 없으므로(2026-08-13 회신 §2) "      \
      "여기가 유일한 방어선이다. 바닥이 넓으면 올려도 된다")                    \
    X(double, marker_offset_m, 0.0,                                            \
      "마커 중심이 로봇 회전 중심에서 진행방향으로 떨어진 거리. 오도메트리 "    \
      "캘리의 world_xy_mm 계산에만 쓴다. 0이면 보정 없음 (로봇팀 실측 전 기본)")\
    /* ---- 채널 간 정합 (registration, 2026-08-15) ---- */                     \
    X(long, reg_capture_interval_ms, 800,                                       \
      "REGISTER_CAPTURE 자동 반복 주기. 로봇 경로가 없어(FOV 겹침 미실측) "      \
      "조이스틱으로 겹침 구역을 지나가는 동안 여러 지점을 모으려면 계속 "        \
      "쏴야 한다 - 오도메트리의 READY/GO 같은 동기화 지점이 없다")               \
    X(long, reg_capture_ack_timeout_ms, 3000,                                   \
      "REGISTER_DONE/CANCEL 전송 후 CCTV의 응답(H_MATRIX|REGISTER_FAIL|"        \
      "REGISTER_STOPPED)을 기다리는 한도의 1/3 단위 (실제 한도는 이 값의 "       \
      "3배 - checkRegistrationTick() 참고). 링크가 끊긴 채로 굳는 것을 막는다")  \
    X(long, reg_session_timeout_ms, 600000,                                     \
      "정합 수집 세션 전체 데드라인. 켜둔 채 잊어버렸을 때의 워치독 - "          \
      "calib_odo_timeout_ms와 같은 역할")

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
    floorAt("pen_width_m", p.pen_width_m, 0.0);
    // 🔴 w/2 가 a 를 넘으면 도색 진입 오프셋(a - w/2)이 음수가 된다 - 펜을 내리기
    // 전에 뒤로 물러나는 꼴이라 기하가 성립하지 않는다. 물리적으로도 "펜 반폭이
    // 마커~펜 거리보다 크다"는 뜻이라 현장 오입력으로 보고 잘라낸다.
    if (p.pen_width_m / 2.0 > p.pen_offset_m) {
        logf("[WARN] params 'pen_width_m'=%g 의 절반이 pen_offset_m(%g)을 넘음 "
             "- 도색 진입 오프셋이 음수가 되므로 %g 로 내림",
             p.pen_width_m, p.pen_offset_m, p.pen_offset_m * 2.0);
        p.pen_width_m = p.pen_offset_m * 2.0;
    }
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
    if (p.calib_timeout_ms >= kQtCalibWaitMs) {
        logf("[WARN] params 'calib_timeout_ms'=%ld 은(는) Qt 대기 한도(%ld)"
             " 이상 - %ld으로 내림", p.calib_timeout_ms, kQtCalibWaitMs,
             kQtCalibWaitCapMs);
        p.calib_timeout_ms = kQtCalibWaitCapMs;
    }
    // ⚠️ calib_odo_timeout_ms에는 여기서 상한 클램프를 걸지 않는다. 예전에는
    // "오도메트리는 ADMIN 전용이라 Qt 제약이 없다"가 이유였는데, 2026-08-13에
    // Qt 개시를 열면서 그 전제가 깨졌다. 그렇다고 값 자체를 깎으면 ADMIN 개시
    // 세션까지 같이 짧아진다 - 그쪽은 Qt가 기다리지 않으므로 깎을 이유가 없다.
    // 그래서 클램프를 **세션 단위**로 옮겼다: Router::odoDriveBudgetMs()가
    // QT 개시일 때만 kQtOdoWaitCapMs - calib_odo_result_wait_ms 로 깎는다.
    floorAt("calib_odo_timeout_ms", p.calib_odo_timeout_ms, 5000L);
    floorAt("calib_odo_result_wait_ms", p.calib_odo_result_wait_ms, 1000L);
    // 결과 대기가 Qt 예산을 통째로 먹으면 주행 데드라인이 0 이하가 된다 -
    // QT 개시 세션이 시작하자마자 타임아웃으로 죽는다. 예산의 절반으로 자른다.
    // 🔴 이 구간은 오도메트리에만 있으므로 기준은 kQtOdoWaitCapMs(10분)다.
    if (p.calib_odo_result_wait_ms > kQtOdoWaitCapMs / 2) {
        logf("[WARN] params 'calib_odo_result_wait_ms'=%ld 은(는) Qt 예산(%ld)의 "
             "절반을 넘음 - %ld으로 내림 (QT 개시 세션의 주행 시간이 남지 않는다)",
             p.calib_odo_result_wait_ms, kQtOdoWaitCapMs, kQtOdoWaitCapMs / 2);
        p.calib_odo_result_wait_ms = kQtOdoWaitCapMs / 2;
    }
    // 0을 넣으면 checkRegistrationTick()이 매 tick(200ms)마다 REGISTER_CAPTURE를
    // 쏜다 - 카메라가 감당 못 할 정도는 아니지만 의도한 값은 아닐 가능성이 높다.
    floorAt("reg_capture_interval_ms", p.reg_capture_interval_ms, 100L);
    floorAt("reg_capture_ack_timeout_ms", p.reg_capture_ack_timeout_ms, 500L);
    floorAt("reg_session_timeout_ms", p.reg_session_timeout_ms, 10000L);
    floorAt("calib_capture_timeout_ms", p.calib_capture_timeout_ms, 1000L);
    // 상한이 하한(min_move_m*2)보다 작으면 어떤 값도 통과하지 못한다 - 캘리가
    // 통째로 막히고, 조작자는 무슨 값을 넣어도 invalid_param만 본다.
    floorAt("calib_odo_max_side_m", p.calib_odo_max_side_m, p.min_move_m * 2.0);
    floorAt("marker_offset_m", p.marker_offset_m, 0.0);
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
