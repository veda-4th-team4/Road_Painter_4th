#pragma once
// 사용자 저장소: id / 비밀번호(PBKDF2-SHA256 해시) / 캘리브레이션 번들을 JSON 파일로 영속화.
// 로그인 = 저장된 캘리브레이션 재사용 목적 (동시 다중 사용자 없음, 한 현장 = 카메라 1대 가정)
// 번들 = {K, D, H_floor, H_marker, marker_height_m, version} (calib.hpp 참고)
//
// 2026-08-03 (프로토콜 v0.4): 카메라가 4채널(PNM-C16083RVQ)이 되면서 번들이
// 채널마다 달라졌다 - 렌즈 방향이 다르면 K/D/H가 전부 다르다. 그래서 저장 형식이
// 번들 하나에서 채널별 맵으로 바뀌었다:  "calib": {"1": {…}, "2": {…}}
// 이미 배포된 파일의 예전 형식(번들 하나)은 읽을 때 채널 1의 번들로 승격시킨다
// (calib.hpp asCalibChannelMap) - 마이그레이션 스크립트 없이 기존 현장이 그대로 뜬다.
#include "protocol.hpp"
#include <mutex>
#include <string>

class UserStore {
public:
    explicit UserStore(const std::string& file);

    // 신규 등록. camIp는 선택(빈 문자열 허용 - QT가 안 보내도 등록 가능).
    // 실패 시 false + err에 사유("이미 존재하는 id" 등)
    bool registerUser(const std::string& id, const std::string& pw,
                      const std::string& camIp, std::string& err);
    // id/비번 검증
    bool login(const std::string& id, const std::string& pw);
    // ── 캘리브레이션 (2026-08-03부터 채널별) ────────────────────────────
    // 저장 형식이 번들 하나에서 채널별 맵({"1":{…},"2":{…}})으로 바뀌었다.
    // 예전 형식이 파일에 있으면 채널 1의 번들로 읽어준다 (calib.hpp
    // asCalibChannelMap) - 마이그레이션 없이 기존 현장이 그대로 뜬다.

    // 한 채널의 번들 (없으면 null 반환. 구버전 "H" 키도 읽어줌).
    // 계정에 그 채널이 없으면 전역 슬롯(setGlobalCalib) 값으로 대체한다 - 아래 참고.
    json getCalib(const std::string& id, int ch);
    // 계정 + 전역을 합친 채널별 맵 전체 (LOGIN_OK.calibs용).
    // 채널마다 "계정 값 → 없으면 전역 값" 규칙이 각각 적용된다.
    json getCalibs(const std::string& id);
    // 한 채널의 번들 저장 + 파일 반영 (다른 채널은 건드리지 않는다)
    bool setCalib(const std::string& id, int ch, const json& calib);

    // 계정과 무관한 전역 캘리브레이션 슬롯 (QT-REQ-SRV-001 R-1).
    // 캘리브레이션은 "현장(카메라+바닥)의 속성"이지 사용자 속성이 아니다. 예전에는
    // 번들이 로그인 사용자에게만 매달려서, 아무도 로그인하지 않은 채 캘리를 올리면
    // 메모리에만 남고 서버 재시작 시 사라졌다 - 설치 기사가 "QT 로그인 먼저"를 매번
    // 기억해야 했다. 이제 로그인 상태와 무관하게 별도 파일에 남기고, 계정에 자기
    // 캘리가 없으면 이 값을 내려준다.
    bool setGlobalCalib(int ch, const json& calib);
    json getGlobalCalib(int ch);
    // 등록 시 저장한 카메라 IP (없으면 null 반환)
    json getCamIp(const std::string& id);
    // 카메라 IP 변경 + 파일 반영 (Qt 설정란에서 교체. 빈 문자열이면 null로 지움)
    bool setCamIp(const std::string& id, const std::string& camIp);

private:
    void load();
    void save();  // mtx_ 잡은 상태에서 호출
    void loadGlobalCalib();
    void saveGlobalCalib();  // mtx_ 잡은 상태에서 호출
    // 계정에 저장된 캘리브레이션을 채널별 맵으로 (구버전 형식 흡수).
    // mtx_ 잡은 상태에서 호출
    json accountCalibMap(const std::string& id);

    std::string file_;
    std::string calibFile_;  // 전역 캘리 파일 (users.json 옆의 calib_latest.json)
    std::mutex mtx_;
    // { "<id>": {"salt":hex, "hash":hex, "calib":{"1":{...},"2":{...}}|null} }
    json users_;
    // 계정과 분리된 최신 번들의 채널별 맵 (없으면 빈 오브젝트).
    // 파일에 예전 형식(번들 하나)이 있으면 load 시점에 채널 1의 맵으로 승격된다.
    json globalCalib_;
};
