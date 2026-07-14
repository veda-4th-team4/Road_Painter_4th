#pragma once
// 사용자 저장소: id / 비밀번호(PBKDF2-SHA256 해시) / H행렬을 JSON 파일로 영속화.
// 로그인 = 저장된 H행렬 재사용 목적 (동시 다중 사용자 없음, 한 현장 = CCTV 1대 가정)
#include "protocol.hpp"
#include <mutex>
#include <string>

class UserStore {
public:
    explicit UserStore(const std::string& file);

    // 신규 등록. 실패 시 false + err에 사유("이미 존재하는 id" 등)
    bool registerUser(const std::string& id, const std::string& pw, std::string& err);
    // id/비번 검증
    bool login(const std::string& id, const std::string& pw);
    // 저장된 H행렬 (없으면 null 반환)
    json getH(const std::string& id);
    // H행렬 저장 + 파일 반영
    bool setH(const std::string& id, const json& H);

private:
    void load();
    void save();  // mtx_ 잡은 상태에서 호출

    std::string file_;
    std::mutex mtx_;
    json users_;  // { "<id>": {"salt":hex, "hash":hex, "H":[[...]]|null} }
};
