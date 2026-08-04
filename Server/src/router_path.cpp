// 경로 생성/전송과 그 실행 상태 관리. router.cpp가 START_DRAW / ABORT_DRAW /
// PATH_DONE / 이탈 감지를 받으면 이리로 넘어온다.
//
// 2단계 구성이다:
//   1단계 접근(approach) - 로봇 현재 위치에서 도면 시작점까지. 서버가 만든다.
//   2단계 도색(draw)     - Qt가 보낸 동작 시퀀스(planProgram_)를 그대로 중계.
//                          program이 없으면 종전대로 서버가 points로 직접 생성.
// 주행 중 도면에서 벗어나면 buildRecovery로 복귀 경로를 만들어 갈아끼운다.
// 펜 오프셋 보정은 어느 경로에서도 하지 않는다 - 로봇이 전담한다.
#include "router.hpp"
#include "log.hpp"
#include <cmath>

// ABORT_DRAW: 진행 중인 작업을 취소한다.
//
// 버리는 것은 "경로 실행 상태"뿐이다. 도면(planPts_/planPaint_/
// planProgram_)은 그대로 남긴다 - 취소한 뒤 다시 START_DRAW를 누르면 같은 도면으로
// 처음부터 시작할 수 있어야 하기 때문. 도면까지 지우면 조작자가 Qt에서 경로를
// 다시 전송해야 하는데, 취소는 보통 "출발 방향이 이상하니 다시" 정도의 상황이다.
bool Router::abortDraw() {
    const bool wasActive = planActive_ || awaitingArrival_ || drawRequested_;
    planActive_ = false;
    awaitingArrival_ = false;
    drawRequested_ = false;  // pose를 기다리며 미뤄둔 시작 요청도 같이 취소한다
    activeSegs_ = json::array();
    planCursor_ = 0;
    // 유예해 둔 READY까지 반드시 같이 버린다 - 남겨두면 취소 직후에 그 유예가
    // 만료되면서 방금 경로를 버린 로봇에게 GO가 날아간다.
    resetAlign();
    // manualMode_는 건드리지 않는다. planActive_가 꺼졌으므로 수동 CMD는 이미
    // 통과하고, 취소 직후 조이스틱으로 로봇을 빼내는 것이 자연스러운 흐름이다.
    return wasActive;
}

// 1단계: Qt "그림그리기 시작" -> 로봇 현재 위치에서 도면 시작점(planPts_[0])까지
// 접근 경로를 만들어 전송. 시작점 도착 후 첫 도색 방향으로 미리 회전까지 시켜두고,
// 로봇이 PATH_DONE으로 도착을 알리면 서버가 곧바로 2단계(도색)로 이어간다.
void Router::startApproach() {
    if (planPts_.size() < 2) {
        logf("[WARN] START_DRAW 수신 - 도면 없음 (무시)");
        sendDrawFail("draw", "no_blueprint", "도면 없음 - 먼저 도면을 전송할 것");
        drawRequested_ = false;
        return;
    }
    if (planActive_) {  // 이미 접근/도색 진행 중 - 중복 시작 방지
        logf("[WARN] START_DRAW 수신 - 이미 경로 실행 중 (무시)");
        sendDrawFail("draw", "busy", "이미 경로 실행 중 - 완료 또는 새 도면 전송 후 시작할 것");
        return;
    }
    if (!poseValid_) {
        // 실패가 아니라 대기다. CCTV로 pose가 잡히는 순간 POS 핸들러가 재호출한다.
        drawRequested_ = true;
        logf("[INFO] START_DRAW 수신 - 로봇 위치 미확인, CCTV POS 수신 후 전송 예정");
        sendDrawFail("draw", "no_pose", "로봇 위치 미확인 - CCTV POS 수신 후 자동 전송 예정");
        return;
    }
    planCursor_ = 0;
    // 시작점까지 (paint=false). 접근 목표는 planPts_[0] "그대로"다 - 로봇
    // 중심을 도면 시작점에 세우기만 하면 되고, 펜 오프셋 보정은 로봇이 스스로
    // 한다 (서버/Qt 둘 다 관여하지 않음).
    // 접근 구간에는 NOZZLE을 끼우지 않는다. 전부 paint=false라 withNozzleOps를
    // 통과시켜도 아무것도 안 붙고(노즐은 올라간 채로 시작·유지), 도색/복귀 경로가
    // 항상 NOZZLE up으로 끝나므로 여기서 다시 올릴 필요도 없다.
    json segs = buildSegments(pose_, {planPts_[0]});

    // 첫 도색 구간(시작점 -> 두번째 점) 방향으로 미리 회전.
    // (마지막 TURN에도 heading_deg가 실려 로봇이 READY로 정렬 확인 가능)
    double first = std::atan2(planPts_[1][1] - planPts_[0][1],
                              planPts_[1][0] - planPts_[0][0]) * 180.0 / M_PI;
    appendTurnTo(segs, arrivalHeading(segs, pose_.theta * 180.0 / M_PI), first);

    if (segs.empty()) {
        // 로봇이 이미 시작점에 첫 도색 방향으로 서 있는 경계 케이스. 접근할 게
        // 없으므로 도착 통지를 기다리지 않고 곧바로 2단계로 넘어간다.
        logf("[INFO] 접근 불필요 (이미 시작점) - 곧바로 도색 경로 전송");
        drawRequested_ = false;
        awaitingArrival_ = true;  // sendDrawPath의 단계 가드 통과용
        sendDrawPath();
        return;
    }
    if (srv_.sendTo("ROBOT", makePathMsg(segs, "approach"))) {
        planActive_ = true;
        awaitingArrival_ = true;
        drawRequested_ = false;
        activeSegs_ = segs;
        resetAlign();  // 새 경로 = 정렬 상태 리셋
        lastPlanMs_ = nowMs();
        logf("[INFO] 1단계 접근 경로 전송 (%zu 세그먼트) - 로봇 도착(PATH_DONE) 대기",
             segs.size());
    } else {
        logf("[WARN] 1단계 접근 경로 전송 실패 - 로봇 미접속");
        sendDrawFail("draw", "robot_offline", "로봇 미접속 - 경로 전송 실패");
        drawRequested_ = false;
    }
}

// 2단계: 로봇 접근 완료(PATH_DONE) -> 시작점에 서 있는 로봇에게 도색 경로 전송.
// 로봇은 이 PATH(phase=draw)를 받으면 IMU 현재 방향을 0도로 세팅하고 주행 시작.
void Router::sendDrawPath() {
    // 아래 실패 경로들은 전부 "접근은 끝났는데 도색을 못 시작한" 상황이다.
    // 상태를 접어두지 않으면 planActive_/awaitingArrival_가 켜진 채로 남아
    // Qt가 START_DRAW로 다시 시도할 수도, 완료 통지를 받을 수도 없게 된다.
    if (!awaitingArrival_) {  // 내부 오류 (정상 흐름에서는 도달하지 않음)
        logf("[WARN] 도색 경로 전송 요청 - 접근 완료 대기 상태가 아님 (무시)");
        sendDrawFail("draw", "not_ready", "로봇이 시작점 접근 완료 대기 상태가 아님");
        return;
    }
    if (!poseValid_ || planPts_.size() < 2) {
        logf("[WARN] 도색 경로 생성 불가 - pose/도면 없음");
        sendDrawFail("draw", !poseValid_ ? "no_pose" : "no_blueprint",
                     !poseValid_ ? "로봇 위치 미확인" : "도면 없음");
        awaitingArrival_ = false;
        planActive_ = false;
        return;
    }
    planCursor_ = 0;  // 도색 시작 = 첫 구간부터
    json segs;
    if (planProgram_.is_array() && !planProgram_.empty()) {
        // Qt가 입력한 동작 시퀀스를 손대지 않고 그대로 넘긴다. dist_m/angle_deg는
        // Qt 값 그대로 - 서버는 재계산하지 않는다. 펜 오프셋 보정은 로봇이
        // TURN 실행 시 스스로 한다.
        segs = planProgram_;
        logf("[INFO] Qt 동작 시퀀스 사용 (%zu 동작)", segs.size());
    } else {
        // 하위호환: program이 없으면 종전대로 서버가 도면에서 직접 생성.
        // 시작점에 서 있으므로 남은 경로 = 두번째 점부터.
        // withNozzleOps로 감싸는 이유: 노즐 제어의 단일 결정권이 NOZZLE op이라
        // (path_planner.hpp 참고), MOVE.paint만 실어 보내면 로봇이 노즐을
        // 영영 안 내린다. buildSegments 결과는 반드시 이걸 통과시킬 것.
        std::vector<Pt> rest(planPts_.begin() + 1, planPts_.end());
        std::vector<char> restPaint;
        if (planPaint_.size() == planPts_.size())
            restPaint.assign(planPaint_.begin() + 1, planPaint_.end());
        segs = withNozzleOps(buildSegments(pose_, rest, /*firstPaint=*/true,
                             restPaint.empty() ? nullptr : &restPaint));
    }
    if (segs.empty()) {  // 도색할 구간 없음 (드문 경계 케이스) - 그린 것으로 친다
        logf("[INFO] 도색할 구간 없음 - 곧바로 완료 처리");
        awaitingArrival_ = false;
        planActive_ = false;
        activeSegs_ = json::array();
        srv_.sendTo("QT", makeMsg("DRAW_DONE", json::object()));
        return;
    }
    if (srv_.sendTo("ROBOT", makePathMsg(segs, "draw"))) {
        awaitingArrival_ = false;
        planActive_ = true;
        activeSegs_ = segs;
        resetAlign();
        lastPlanMs_ = nowMs();
        logf("[INFO] 2단계 도색 경로 전송 (%zu 세그먼트) - 도색 시작", segs.size());
    } else {
        logf("[WARN] 2단계 도색 경로 전송 실패 - 로봇 미접속");
        sendDrawFail("draw", "robot_offline", "로봇 미접속 - 경로 전송 실패");
        awaitingArrival_ = false;
        planActive_ = false;
    }
}

// 이탈 복귀 경로: 로봇을 planPts_[k]로 되돌린 뒤 원래 하던 일을 이어가게 한다.
//
// ⚠️ Qt program이 있으면 절대 다시 만들지 않는다. 재생성하면 Qt 화면의
// 미리보기와 실제 실행이 달라져 조작자가 동선을 검증할 수 없게 된다.
// 대신 "복귀 구간만 새로 만들고 원본을 잘라 이어 붙인다". 펜 오프셋 보정은
// 로봇이 자기 TURN 실행 중에 스스로 하므로(program에는 안 드러남) 여기서는
// 신경 쓰지 않는다 - k번 꼭짓점 이후를 담당하는 첫 op을 찾아 그 앞에 복귀
// 주행만 이어붙이면 끝이다.
//
//   [NOZZLE 올림]            복귀 주행 중에는 칠하지 않는다
//   [현재 pose -> planPts_[k]]  로봇 "중심"을 꼭짓점에 (buildSegments)
//   [TURN -> 재개 방위]
//   [원본 program의 재개 지점부터 끝까지]
json Router::buildRecovery(size_t k) {
    if (planPts_.size() < 2 || k >= planPts_.size()) return json::array();

    // ----- 하위호환: program이 없으면 남은 도면으로 직접 생성 (종전 동작) -----
    if (!planProgram_.is_array() || planProgram_.empty()) {
        std::vector<Pt> rest(planPts_.begin() + k, planPts_.end());
        std::vector<char> restPaint;
        if (planPaint_.size() == planPts_.size()) {
            restPaint.assign(planPaint_.begin() + k, planPaint_.end());
            restPaint[0] = 0;  // 복귀 구간(현재 위치 -> pts[k])은 칠하지 않는다
        }
        // 여기도 NOZZLE을 끼워 넣는다 (sendDrawPath 폴백과 같은 이유).
        return withNozzleOps(buildSegments(pose_, rest, /*firstPaint=*/false,
                             restPaint.empty() ? nullptr : &restPaint));
    }

    // ----- 재개 지점 찾기: v >= k인 첫 op -----
    size_t start = planProgram_.size();
    for (size_t i = 0; i < planProgram_.size(); ++i) {
        if (planProgram_[i].value("v", -1) >= (int)k) { start = i; break; }
    }
    if (start >= planProgram_.size()) {
        // k가 program의 모든 op보다 뒤 - 남은 program이 없다는 뜻(주로 마지막
        // 꼭짓점 부근에서 이탈했을 때). 이어붙일 동작이 없으니 꼭짓점까지만
        // 데려간다 - 그게 이 복귀에서 할 수 있는 전부다.
        json segs = json::array();
        segs.push_back({{"op", "NOZZLE"}, {"down", false}});
        std::vector<Pt> to{planPts_[k]};
        std::vector<char> noPaint{0};
        json back = buildSegments(pose_, to, /*firstPaint=*/false, &noPaint);
        for (auto& s : back) segs.push_back(s);
        logf("[INFO] 복귀 경로: %zu번 꼭짓점 이후 남은 program 없음 - "
             "꼭짓점까지만 이동", k);
        return segs;
    }

    // 재개 동작이 바라볼 방위. NOZZLE처럼 방위가 없는 op면 뒤에서 찾는다.
    double heading = 1e9;
    for (size_t i = start; i < planProgram_.size() && heading > 1e8; ++i)
        heading = planProgram_[i].value("heading_deg", 1e9);
    if (heading > 1e8) heading = pose_.theta * 180.0 / M_PI;

    json segs = json::array();
    segs.push_back({{"op", "NOZZLE"}, {"down", false}});

    // 로봇 중심을 꼭짓점으로 (펜 오프셋 보정은 로봇이 TURN 실행 시 스스로 함)
    std::vector<Pt> to{planPts_[k]};
    std::vector<char> noPaint{0};
    json back = buildSegments(pose_, to, /*firstPaint=*/false, &noPaint);
    const double arrival = arrivalHeading(back, pose_.theta * 180.0 / M_PI);
    for (auto& s : back) segs.push_back(s);
    appendTurnTo(segs, arrival, heading);
    // 재개 지점이 곧바로 도색 MOVE면 노즐을 내려준다 (위에서 올려놨으므로).
    // program이 NOZZLE로 시작하는 정상 경우엔 중복되지 않는다.
    if (planProgram_[start].value("op", "") == "MOVE" &&
        planProgram_[start].value("paint", false))
        segs.push_back({{"op", "NOZZLE"}, {"down", true}});

    for (size_t i = start; i < planProgram_.size(); ++i)
        segs.push_back(planProgram_[i]);
    return segs;
}

// 경로 생성/전송 실패(또는 대기) 시 Qt에 통지. reason 코드로 상황 구분.
void Router::sendDrawFail(const char* stage, const char* reason,
                          const std::string& msg) {
    srv_.sendTo("QT", makeMsg("DRAW_FAIL",
        {{"stage", stage}, {"reason", reason}, {"msg", msg}}));
}

// segs를 다 수행한 뒤 로봇이 바라보는 방위. MOVE만 방위를 바꾸므로 마지막 MOVE의
// heading_deg가 답이고, MOVE가 없으면 제자리라 fallbackDeg(보통 현재 pose 각도).
double Router::arrivalHeading(const json& segs, double fallbackDeg) {
    double arrival = fallbackDeg;
    for (const auto& s : segs)
        if (s.value("op", "") == "MOVE") arrival = s.value("heading_deg", arrival);
    return arrival;
}

// arrivalDeg -> headingDeg 회전을 TURN op으로 붙인다 (kMinTurnDeg 이하면 생략).
// 접근 경로 끝의 "첫 도색 방향으로 미리 회전"과 복귀 경로 끝의 "재개 방위로
// 회전"이 같은 계산이라 한 곳에 모았다. heading_deg를 반드시 같이 실어야
// 로봇이 이 TURN 뒤에 READY를 보냈을 때 서버가 정렬 판정을 할 수 있다.
void Router::appendTurnTo(json& segs, double arrivalDeg, double headingDeg) {
    const double turn = normDeg(headingDeg - arrivalDeg);
    if (std::fabs(turn) <= kMinTurnDeg) return;
    segs.push_back({{"op", "TURN"},
                    {"angle_deg", std::round(turn * 10) / 10},
                    {"heading_deg", std::round(headingDeg * 10) / 10}});
}
