// 출발 전 정렬(READY -> ALIGN/GO) 판정. router.cpp의 fromRobot이 READY를 받으면
// 이리로 넘어온다.
//
// 핵심은 "언제 판정하느냐"다. 로봇은 미세회전(ALIGN)을 마치는 즉시 READY를 다시
// 보내는데, 그 회전은 0.1~0.3초짜리라 CCTV -> 서버 pose에 아직 반영돼 있지 않다.
// 그 상태로 판정하면 회전 전 각도를 보고 같은 ALIGN을 또 쏴 진동한다. 그래서
// ALIGN 뒤에 온 READY는 새 POS가 쌓일 때까지 유예했다가 판정한다
// (router.hpp kAlignFreshFrames / kAlignWaitMaxMs 참고).
#include "router.hpp"
#include "log.hpp"
#include <cmath>
#include <cstdio>  // snprintf (판정 근거를 로그에 남길 때)

// 새 경로를 보냈거나 경로가 끝났을 때의 정렬 상태 초기화 (router.hpp 참고).
void Router::resetAlign() {
    alignSegIdx_ = -1;
    alignTries_ = 0;
    clearPendingReady();
}

void Router::clearPendingReady() {
    pendingReadySeg_ = -1;
    pendingSin_ = pendingCos_ = 0;
    pendingPoseN_ = 0;
}

// READY 정렬 판정. CCTV pose의 실제 각도 vs 해당 세그먼트의 목표 heading 비교:
//   오차 > 임계값 -> ALIGN(그만큼 미세 회전 후 로봇이 다시 READY)
//   오차 <= 임계값 or 반복 초과 -> GO(직진 시작 / 접근 마지막 TURN이면 대기)
// 여기 들어왔다는 건 "지금 pose로 판정해도 된다"가 이미 정해졌다는 뜻이다
// (READY 수신 즉시 or 유예 후 새 POS 확보). 어느 경로로 오든 반드시 로봇에
// 응답을 하나 보낸다 - 응답을 빠뜨리면 로봇이 영원히 정지한 채 대기한다.
void Router::judgeReady(int seg) {
    // 유예 중 모아둔 theta가 있으면 그 평균으로 판정한다. 기다리는 동안 공짜로
    // 쌓인 표본이라 지연 없이 노이즈만 1/sqrt(N)로 준다. 각도 평균은 반드시
    // 원 위에서(sin/cos) - 도를 그냥 더하면 ±180도 경계에서 깨진다.
    double thetaDeg = pose_.theta * 180.0 / M_PI;
    char avgDesc[32];
    if (pendingPoseN_ > 0) {
        thetaDeg = std::atan2(pendingSin_, pendingCos_) * 180.0 / M_PI;
        snprintf(avgDesc, sizeof avgDesc, "POS %d장 평균", pendingPoseN_);
    } else {
        snprintf(avgDesc, sizeof avgDesc, "단일 POS");
    }
    clearPendingReady();
    // heading_deg가 있는 세그먼트(MOVE 전부 + 접근 경로 마지막 TURN)만 판정 가능
    double target = 1e9;
    if (planActive_ && !manualMode_ && seg >= 0 && seg < (int)activeSegs_.size())
        target = activeSegs_[seg].value("heading_deg", 1e9);

    if (target > 1e8 || !poseValid_) {
        // 판정 불가(계획 없음/수동모드/seg 이상/pose 없음)면 로봇을 세워두지 않는다
        srv_.sendTo("ROBOT", makeMsg("GO", json::object()));
        logf("[WARN] READY(seg=%d) - 정렬 판정 불가, 그냥 GO", seg);
        return;
    }
    double err = normDeg(target - thetaDeg);
    if (std::fabs(err) > kAlignThresholdDeg && alignTries_ < kAlignMaxTries) {
        ++alignTries_;
        srv_.sendTo("ROBOT",
                    makeMsg("ALIGN", {{"angle_deg", std::round(err * 10) / 10}}));
        logf("[INFO] READY(seg=%d) 각도오차 %.1f도 (%s) - ALIGN 전송 (%d/%d회)",
             seg, err, avgDesc, alignTries_, kAlignMaxTries);
    } else {
        srv_.sendTo("ROBOT", makeMsg("GO", json::object()));
        logf("[INFO] READY(seg=%d) 오차 %.1f도 (%s) - GO%s", seg, err, avgDesc,
             alignTries_ >= kAlignMaxTries ? " (ALIGN 반복 초과)" : "");
    }
}

// 유예해 둔 READY가 있으면 판정 조건을 확인한다. READY 수신 이후 새 POS가
// kAlignFreshFrames만큼 쌓였거나, 그 전에 kAlignWaitMaxMs가 지나면(POS가 끊긴
// 상황) 있는 pose로 판정한다.
// POS 수신 때뿐 아니라 onMessage 진입 때마다 호출한다 - POS가 완전히 끊기면 POS
// 핸들러는 다시 안 돌아 타임아웃이 영영 안 걸리기 때문. 그 경우 로봇 STATUS가
// heartbeat가 되어 유예된 READY를 반드시 회수한다. 단 STATUS는 500ms 주기(2Hz,
// 로봇 main.cpp)라 타임아웃 판정이 최대 그만큼 늦다 - POS가 죽은 상황의 최악
// 응답 지연은 kAlignWaitMaxMs + 500ms다.
void Router::resolvePendingReady() {
    if (pendingReadySeg_ < 0) return;
    long got = posSeq_ - pendingPosSeq_;
    bool fresh = got >= kAlignFreshFrames;
    bool timeout = nowMs() - pendingReadyMs_ > kAlignWaitMaxMs;
    if (!fresh && !timeout) return;
    // 🔴 새 POS가 "한 장도" 없으면 판정하지 않고 GO를 보낸다 (2026-08-04).
    // 여기서 judgeReady를 부르면 pose_가 직전 ALIGN을 계산할 때와 글자 그대로
    // 같으므로 err도 같고, 결국 **똑같은 ALIGN이 한 번 더 나가는 것이 보장**된다.
    // 2026-08-04 로그에서 실제로 -34.6도가 값까지 동일하게 세 번 나갔고, 로봇은
    // 그만큼 세 번 돌아 약 104도를 회전했다.
    // GO를 택한 이유: 남은 각도오차는 주행 중 DRIFT가 이어서 잡아주지만, 중복
    // ALIGN이 만든 누적 오버슛은 되돌릴 방법이 없다. 판정 근거가 없을 때 로봇을
    // 세워두지 않는다는 기존 원칙(judgeReady의 "판정 불가 -> GO")과도 같다.
    // alignTries_는 소모하지 않는다 - 실제로 정렬을 시도한 것이 아니기 때문이다.
    if (got == 0) {
        int seg = pendingReadySeg_;
        long waited = nowMs() - pendingReadyMs_;
        clearPendingReady();
        srv_.sendTo("ROBOT", makeMsg("GO", json::object()));
        logf("[WARN] READY(seg=%d) 유예 %ldms 동안 새 POS 0장 - 직전과 같은 pose라 "
             "판정 불가, GO (같은 ALIGN 반복 방지)", seg, waited);
        return;
    }
    if (!fresh)
        logf("[WARN] READY(seg=%d) 유예 %ldms 초과 (새 POS %ld/%d장) - 현재 pose로 판정",
             pendingReadySeg_, nowMs() - pendingReadyMs_, got, kAlignFreshFrames);
    judgeReady(pendingReadySeg_);
}
