// 계정/캘리브레이션/채널 관련 처리. router.cpp의 각 from*()가 LOGIN·H_MATRIX·
// SELECT_CHANNEL을 받으면 이리로 넘어온다.
//
// 캘리브레이션은 채널마다 완전히 다르다(렌즈 방향이 달라 K/D/H가 전부 다름).
// 그래서 서버는 채널별 맵(calibs_)으로 들고, "활성 채널" 하나를 기억한다.
// 저장은 계정(users.json)과 전역 슬롯(calib_latest.json) 두 군데에 하는데,
// 아무도 로그인하지 않은 채 캘리를 올려도 재시작 후 살아남아야 하기 때문이다
// (user_store.hpp setGlobalCalib 주석 참고).
#include "router.hpp"
#include "log.hpp"
#include <cstdlib>  // std::atoi (채널별 캘리 맵의 키 파싱)

// 로그인 처리 (QT/ADMIN 공용). 성공하면 서버가 기억하는 로그인 사용자(currentUser_)를
// 갱신하고 그 계정에 저장돼 있던 캘리브레이션을 현재 세션에 복원한다.
// 캘리브레이션(H_MATRIX)은 "그 시점의 currentUser_"에게 영속 저장되므로, QT가 아직
// 붙지 않은 설치 현장에서도 관리자 창이 먼저 로그인해두면 캘리 결과가 계정에 남는다.
void Router::handleLogin(const json& payload, const std::string& replyRole) {
    std::string id = payload.value("id", "");
    if (!users_.login(id, payload.value("pw", ""))) {
        srv_.sendTo(replyRole,
                    makeMsg("LOGIN_FAIL", {{"reason", "id 또는 비밀번호 불일치"}}));
        logf("[WARN] LOGIN %s 실패 (%s 요청)", id.c_str(), replyRole.c_str());
        return;
    }
    currentUser_ = id;
    // 저장된 채널별 번들을 현재 세션에 통째로 복원한다. 예전에는 번들이 하나뿐이라
    // 한 줄이면 됐지만, 이제 채널마다 따로 들고 있어야 한다.
    const json storedMap = users_.getCalibs(id);
    calibs_.clear();
    for (auto it = storedMap.begin(); it != storedMap.end(); ++it) {
        if (it->is_null()) continue;
        const int ch = std::atoi(it.key().c_str());
        if (!validChannel(ch)) {
            logf("[WARN] 저장된 캘리브레이션에 이상한 채널 키 '%s' - 무시",
                 it.key().c_str());
            continue;
        }
        Calib c;
        if (calibFromJson(*it, c)) calibs_[ch] = c;
        else logf("[WARN] 채널 %d 캘리브레이션 파싱 실패 - 무시", ch);
    }
    const Calib& act = activeCalib();
    // calib(활성 채널 번들)은 v0.3과 의미가 같아 옛 클라이언트도 그대로 동작한다.
    // calibs(채널별 맵)는 4채널 UI가 "어느 채널이 준비됐는지" 표시하는 데 쓴다.
    json out{{"id", id},
             {"calib", act.valid ? act.raw : json()},
             {"calibs", storedMap},
             {"active_ch", activeChannel_},
             {"cam_ip", users_.getCamIp(id)}};
    // 중계 RTSP 주소. 설정 파일이 있을 때만 싣는다 - 필드를 null로라도 보내면
    // QT가 "서버가 값을 줬는데 비어있다"와 "안 줬다"를 구분하지 못한다.
    // ⚠️ cam_ip는 그대로 둔다. 저건 카메라 IP(PNO 직결용)라 의미가 다르고,
    //    합치면 PNO로 되돌아갈 수 없게 된다 (stream_cfg.hpp 주석 참고).
    const StreamCfg stream = loadStreamCfg(kStreamCfgFile);
    if (stream.valid()) out["stream"] = stream.toJson();
    srv_.sendTo(replyRole, makeMsg("LOGIN_OK", out));
    logf("[INFO] LOGIN %s 성공 (%s 요청, 캘리브레이션 %zu채널 보유, 활성 채널 %d %s, "
         "중계 주소 %s)",
         id.c_str(), replyRole.c_str(), calibs_.size(), activeChannel_,
         act.valid ? "전달" : "없음 - 캘리브레이션 필요",
         stream.valid() ? stream.base.c_str() : "없음 - QT 설정값 사용");
}

// 활성 채널의 캘리브레이션. 그 채널이 아직 캘리브레이션되지 않았으면 valid=false인
// 빈 값을 돌려준다 - 호출부는 calib.valid만 보면 되고 null 검사를 따로 안 해도 된다.
const Calib& Router::activeCalib() const {
    static const Calib kNone;
    auto it = calibs_.find(activeChannel_);
    return (it == calibs_.end()) ? kNone : it->second;
}

// SELECT_CHANNEL: 작업 채널 전환. 로봇과는 무관하므로 CCTV로만 중계한다.
void Router::selectChannel(const json& payload, const json& msg) {
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

// 캘리브레이션 번들 수신 (CCTV 직접 or 관리자 창 ADMIN 경유 공용). 세 형태를 받는다:
//   중첩:   payload.calib = {K, D, H_floor, H_marker, marker_height_m, version}
//   평면:   payload 자체가 번들 = {calib_id, K, D, H, H_marker, canvas_mm, ...}
//           (QT-REQ-CCTV-001 rev.2 — 바닥 H를 H_floor가 아니라 H로 부른다)
//   레거시: payload.H = [[...]x3] 뿐 (왜곡 보정 없이 바닥/마커 공용으로 사용)
// CCTV는 mm 기준(pixel->world mm) 호모그래피를 보낸다. 서버 입구에서 미터로 정규화한 뒤
// 저장/중계하므로, 이후 pose/POSE/BLUEPRINT/PATH와 QT top-view는 전부 미터로 통일된다.
void Router::handleHMatrix(const json& msg) {
    json payload = msg.value("payload", json::object());
    // 평면 번들과 레거시는 둘 다 최상위 "H"를 갖는다. 예전엔 "calib이 없고 H가 있으면
    // 레거시"로 단정해 payload["H"] 행렬 하나만 떼어냈고, 그래서 평면 번들이 오면
    // K/D/H_marker가 통째로 버려져 왜곡 보정과 시차 보정이 조용히 꺼졌다 - 좌표는
    // 그럴듯하게 나오고 렌즈 왜곡만큼 틀린다. 캘리 내용 필드가 H와 같이 왔으면
    // 평면 번들로 본다.
    const bool nested = payload.contains("calib");
    const bool legacyH = !nested && payload.contains("H") &&
                         !payload.contains("K") && !payload.contains("D") &&
                         !payload.contains("H_floor") && !payload.contains("H_marker");
    json bundle = nested ? payload["calib"] : (legacyH ? payload["H"] : payload);
    // 어느 채널의 번들인가. payload 최상위의 "ch"를 본다 (없으면 1 - 단일 채널
    // 카메라와 v0.3 클라이언트 하위호환). 평면 스키마는 payload 자체가 번들이라
    // "ch"가 번들 안에 남는데, calib_id처럼 손대지 않는 메타데이터로 보존한다 -
    // 저장된 번들만 봐도 어느 채널 것인지 알 수 있어 오히려 낫다.
    const int ch = channelOf(payload);
    normalizeBundleMmToM(bundle);  // mm -> m (÷1000). 이후 번들은 미터 기준.
    aliasFloorKey(bundle);  // 평면 스키마의 "H"에 "H_floor" 별칭 (QT는 H_floor만 봄)
    Calib c;
    if (!calibFromJson(bundle, c)) {
        logf("[WARN] H_MATRIX(채널 %d) 파싱 실패 - calib/H 형식 확인 필요: %s",
             ch, payload.dump().c_str());
        return;
    }
    calibs_[ch] = c;
    // 정규화된(미터) 번들로 다시 싸서 QT에 중계 + 영속 저장 - QT는 미터 H_floor로 top-view.
    json outMsg = msg;
    if (nested) outMsg["payload"]["calib"] = bundle;
    else if (legacyH) outMsg["payload"]["H"] = bundle;
    else outMsg["payload"] = bundle;  // 평면 번들은 payload 자체가 번들
    // 중계본에도 채널을 명시한다. 중첩/레거시 스키마는 ch가 payload 밖이라 원본에
    // 없으면 사라지는데, Qt는 "지금 보고 있는 채널의 번들인가"를 판단해야 한다 -
    // 다른 채널 캘리로 top-view를 갈아엎으면 화면과 좌표가 통째로 어긋난다.
    outMsg["payload"]["ch"] = ch;
    srv_.sendTo("QT", outMsg);
    // 어느 스키마로 읽혔는지 남긴다 - 평면 번들을 보냈는데 "레거시"로 찍히면
    // K/D가 빠졌다는 뜻이라, 로그만 보고 바로 알 수 있어야 한다.
    const char* schema = nested ? "중첩 calib" : (legacyH ? "레거시 H" : "평면 번들");
    // 로그인 상태와 무관하게 전역 슬롯에 먼저 남긴다 (QT-REQ-SRV-001 R-1).
    // 캘리브레이션은 현장 속성이라, 아무도 로그인하지 않은 채 올려도 서버 재시작 후
    // 살아남아야 한다 - 예전엔 이 경우 메모리에만 남아 그대로 유실됐다.
    users_.setGlobalCalib(ch, bundle);
    const char* detail_marker =
        c.hasMarker ? "H_marker 포함" : "H_marker 없음 - 시차 보정 생략됨";
    const char* detail_kd = c.hasKD ? ", K/D 포함" : ", K/D 없음 - 왜곡 보정 생략됨";
    // ch를 안 실어 보내면 전부 채널 1에 덮어써진다 - 4채널을 캘리한 줄 알았는데
    // 마지막 하나만 남는 상황이라, 어느 채널로 저장됐는지 로그에 반드시 남긴다.
    if (!currentUser_.empty() && users_.setCalib(currentUser_, ch, bundle))
        logf("[INFO] 캘리브레이션 수신 [채널 %d] (%s, mm->m 정규화, %s%s) - "
             "사용자 '%s' + 전역 슬롯에 영속 저장",
             ch, schema, detail_marker, detail_kd, currentUser_.c_str());
    else
        logf("[INFO] 캘리브레이션 수신 [채널 %d] (%s, mm->m 정규화, %s%s) - "
             "로그인 사용자 없음, 전역 슬롯에 영속 저장 (다음 로그인 때 전달됨)",
             ch, schema, detail_marker, detail_kd);
}

