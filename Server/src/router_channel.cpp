// 채널 전환과 작업 취소 (프로토콜 v0.4).
//
// 캘리브레이션은 채널마다 완전히 다르다(렌즈 방향이 달라 K/D/H가 전부 다름).
// 그래서 서버는 채널별 맵(calibs_)으로 들고, "활성 채널" 하나를 기억한다.
// 저장은 계정(users.json)과 전역 슬롯(calib_latest.json) 두 군데에 하는데,
// 아무도 로그인하지 않은 채 캘리를 올려도 재시작 후 살아남아야 하기 때문이다
// (user_store.hpp setGlobalCalib 주석 참고).
//
// ⚠️ 이 파일은 영상(RTSP)을 다루지 않는다. 4채널 스트림 중계는 MediaMTX가
//    별도 프로세스로 하고(Server/relay/), 서버는 "지금 어느 채널을 보는가"라는
//    제어 상태만 관리한다. 서버는 OpenCV도 ffmpeg도 링크하지 않는다.
#include "router.hpp"
#include "log.hpp"

// 활성 채널의 캘리브레이션. 그 채널이 아직 캘리브레이션되지 않았으면 valid=false인
// 빈 값을 돌려준다 - 호출부는 calib.valid만 보면 되고 null 검사를 따로 안 해도 된다.
const Calib& Router::activeCalib() const {
    static const Calib kNone;
    auto it = calibs_.find(activeChannel_);
    return (it == calibs_.end()) ? kNone : it->second;
}

// SELECT_CHANNEL: 작업 채널 전환. 로봇과는 무관하므로 CCTV로만 중계한다.
void Router::selectChannel(const json& payload, const json& msg) {
    // ⚠️ 여기서는 channelOf()를 쓰지 않는다. 그 함수는 잘못된 값을 조용히 1로
    //    바꿔버리는데, 채널 전환은 "그런 채널 없다"를 QT에 알려줘야 한다.
    if (!payload.contains("ch") || !payload["ch"].is_number_integer() ||
        !validChannel(payload["ch"].get<int>())) {
        srv_.sendTo("QT", makeMsg("CHANNEL_FAIL", {{"reason", "bad_channel"}}));
        logf("[WARN] SELECT_CHANNEL - ch가 없거나 범위(%d..%d) 밖: %s",
             kMinChannel, kMaxChannel, payload.dump().c_str());
        return;
    }
    const int ch = payload["ch"].get<int>();
    activeChannel_ = ch;
    // 예전 채널 기준으로 잡아둔 pose는 새 채널에서 의미가 없다 (좌표계가 다르다).
    // 그대로 두면 새 채널의 첫 POS가 오기 전까지 서버가 엉뚱한 위치를 믿는다.
    poseValid_ = false;
    lastIgnoredPosCh_ = 0;
    // 카메라도 그 채널을 봐야 POS가 이 채널 기준으로 온다.
    srv_.sendTo("CCTV", msg);
    const Calib& c = activeCalib();
    srv_.sendTo("QT", makeMsg("CHANNEL_OK",
        {{"ch", ch}, {"calib", c.valid ? c.raw : json()}}));
    logf("[INFO] 활성 채널 %d 로 전환 -> CCTV 중계 (캘리브레이션 %s)", ch,
         c.valid ? "있음" : "없음 - 그 채널을 먼저 캘리브레이션할 것");
}

// 진행 중인 도색 작업을 취소한다 (CMD ABORT_DRAW).
//
// ESTOP과 다른 점은 딱 하나, **받아둔 경로를 버린다**는 것이다. ESTOP은
// 일시정지라 RESUME하면 멈춘 세그먼트부터 이어서 달리고, 서버도 planActive_가
// 살아 있어 다음 START_DRAW를 DRAW_FAIL{busy}로 거절했다.
//
// 실제 상태 정리는 clearPath()가 전부 한다 - 경로/커서/boundary 판정 래치까지
// 한곳에 모여 있어서, 여기서 따로 비우면 v2에 새 상태가 추가될 때마다 두 곳을
// 같이 고쳐야 한다. 🔴 특히 boundary 래치를 안 지우면 취소 직후 열려 있던
// 판정 창이 만료되면서 방금 경로를 버린 로봇에게 GO가 날아간다.
//
// manualMode_는 건드리지 않는다. planActive_가 꺼졌으므로 수동 CMD는 이미
// 통과하고, 취소 직후 조이스틱으로 로봇을 빼내는 것이 자연스러운 흐름이다.
bool Router::abortDraw() {
    const bool wasActive = planActive_ || awaitingArrival_ || drawRequested_;
    // pose를 기다리며 미뤄둔 시작 요청도 같이 취소한다. 안 그러면 취소한 뒤에
    // 첫 POS가 들어오는 순간 접근 경로가 저절로 나간다 (clearPath()는 이걸
    // 지우지 않는다 - 거기선 "경로 실행 상태"만 다루기 때문).
    drawRequested_ = false;
    clearPath();
    return wasActive;
}
