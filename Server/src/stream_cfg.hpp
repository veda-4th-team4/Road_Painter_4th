#pragma once
// 중계(MediaMTX) RTSP 스트림 주소 설정 - LOGIN_OK.stream 으로 QT에 내려준다.
// (QT-REQ-SRV 중계 스트림 주소 요청, 2026-08-04)
//
// 왜 서버가 파일에서 읽는가:
//   서버는 자기 주소를 모른다. 리스닝이 INADDR_ANY라 "클라이언트가 닿을 수 있는
//   IP"를 알아낼 방법이 없고, 중계(MediaMTX)는 별도 프로세스라 서버와 같은 호스트에
//   있으리란 보장도 없다. 그래서 설치할 때 사람이 적어주는 값으로 둔다.
//
// 없으면 없는 대로 동작한다:
//   파일이 없거나 base가 비어 있으면 LOGIN_OK에 stream 필드를 아예 싣지 않는다.
//   QT는 그 경우 종전대로 자기 설정값을 쓰므로, 중계를 안 쓰는 현장(PNO 직결)은
//   이 파일을 만들지 않으면 그만이다 - 4채널 전환은 아직 시도 단계이므로 언제든
//   PNO로 되돌릴 수 있어야 한다.
//
// ⚠️ cam_ip와 헷갈리지 말 것. cam_ip는 "카메라 IP"이고 QT가 그것으로 PNO 단일채널
//    직결 URL을 조립한다. 여기 base는 "중계 서버 주소"다. 둘을 한 필드로 합치면
//    PNO로 되돌아갈 수 없게 된다 - 그래서 별개 필드로 둔다.
#include "log.hpp"
#include "protocol.hpp"
#include <fstream>
#include <string>

struct StreamCfg {
    // 중계 서버 RTSP 베이스 주소. 끝에 '/' 없이 (QT가 "/ch1"을 붙여 쓴다).
    std::string base;
    // 채널 수. 0이면 LOGIN_OK에 싣지 않고 QT가 자기 기본값(4)을 쓴다.
    int channels = 0;
    // 중계를 쓸지. false면 base가 적혀 있어도 LOGIN_OK에 싣지 않는다.
    //
    // 왜 "파일을 지운다"가 아니라 플래그인가: 중계를 껐다 켜는 건 되돌릴 수 있어야
    // 하는데, 파일을 지우면 다시 켤 때 주소를 어디서 가져올지 아무도 모른다.
    // base는 남겨두고 이 플래그만 뒤집으면 왕복이 한 줄로 끝난다.
    // (relay/README.md의 "되돌리려면 프로세스만 죽이면 된다" 원칙과 같은 결.)
    // ⚠️ 이 플래그는 **서버가 주소를 알려줄지**만 정한다. 중계 프로세스(MediaMTX)를
    //    띄우고 내리는 것은 별개다 - relay/start.sh 를 손으로 실행해야 한다.
    bool enabled = true;

    bool valid() const { return enabled && !base.empty(); }

    // LOGIN_OK.stream 에 실을 형태. channels는 0이면 생략한다 - "모른다"와
    // "0채널이다"를 구분해야 QT가 기본값으로 넘어갈 수 있다.
    json toJson() const {
        json j{{"base", base}};
        if (channels > 0) j["channels"] = channels;
        return j;
    }
};

// config/stream.json 을 읽는다. 없으면 valid()==false인 빈 값 (정상 - 위 주석 참고).
//
// 로그인마다 다시 읽는다. 서버를 재시작하면 로봇/CCTV 연결이 통째로 끊기는데,
// 중계 주소 한 줄 바꾸자고 작업 중인 로봇을 떨어뜨릴 수는 없다 - 파일을 고치고
// QT가 재로그인하면 새 값이 내려간다. 로그인은 드물어 파일 I/O 비용은 무시할 수준.
inline StreamCfg loadStreamCfg(const std::string& file) {
    StreamCfg c;
    std::ifstream f(file);
    if (!f) return c;  // 중계를 안 쓰는 현장 - 파일이 없는 게 정상이다
    json j = json::parse(f, nullptr, false);
    if (!j.is_object()) {
        logf("[WARN] 중계 스트림 설정 파싱 실패, 무시: %s", file.c_str());
        return c;
    }
    if (j.contains("base") && j["base"].is_string())
        c.base = j["base"].get<std::string>();
    // 끝의 '/'는 떼어낸다. QT가 "{base}/ch1"로 조립하므로 남아 있으면 "//ch1"이
    // 되어 경로가 안 잡힌다 - 손으로 적는 값이라 실제로 자주 들어간다.
    while (!c.base.empty() && c.base.back() == '/') c.base.pop_back();
    if (j.contains("channels") && j["channels"].is_number_integer())
        c.channels = j["channels"].get<int>();
    if (c.channels < 0) c.channels = 0;  // 음수는 "미지정"과 같이 취급
    // 키가 없으면 true - 이 플래그가 생기기 전에 만들어진 stream.json 은 "쓰려고
    // 만든 파일"이므로 그대로 켜진 채로 읽혀야 한다.
    if (j.contains("enabled") && j["enabled"].is_boolean())
        c.enabled = j["enabled"].get<bool>();
    if (!c.enabled && !c.base.empty())
        logf("[INFO] 중계 주소가 있으나 enabled=false - QT에 알리지 않음 (직결 모드): %s",
             c.base.c_str());
    return c;
}
