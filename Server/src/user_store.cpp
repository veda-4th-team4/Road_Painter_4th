#include "user_store.hpp"
#include "log.hpp"
#include <openssl/evp.h>
#include <openssl/rand.h>
#include <fstream>
#include <sys/stat.h>

static const int kPbkdf2Iters = 10000;
static const size_t kHashLen = 32;  // SHA-256

static std::string toHex(const unsigned char* buf, size_t n) {
    static const char* d = "0123456789abcdef";
    std::string s;
    s.reserve(n * 2);
    for (size_t i = 0; i < n; ++i) {
        s += d[buf[i] >> 4];
        s += d[buf[i] & 0xf];
    }
    return s;
}

// PBKDF2-SHA256(pw, salt) -> hex 해시 (비번은 절대 평문 저장하지 않음)
static std::string hashPw(const std::string& pw, const std::string& saltHex) {
    unsigned char out[kHashLen];
    PKCS5_PBKDF2_HMAC(pw.c_str(), (int)pw.size(),
                      (const unsigned char*)saltHex.data(), (int)saltHex.size(),
                      kPbkdf2Iters, EVP_sha256(), sizeof(out), out);
    return toHex(out, sizeof(out));
}

UserStore::UserStore(const std::string& file) : file_(file) {
    // 저장 디렉토리 보장 (예: config/users.json -> config/)
    auto pos = file_.find_last_of('/');
    if (pos != std::string::npos) mkdir(file_.substr(0, pos).c_str(), 0755);
    // 전역 캘리는 users.json과 같은 폴더에 둔다. 계정 파일과 분리하는 이유는
    // users.json이 비번 해시 때문에 .gitignore 대상이고 백업·교체 주기가 다르기 때문.
    calibFile_ = (pos == std::string::npos ? std::string()
                                           : file_.substr(0, pos + 1)) + "calib_latest.json";
    load();
    loadGlobalCalib();
}

void UserStore::load() {
    users_ = json::object();
    std::ifstream f(file_);
    if (!f) return;  // 첫 실행이면 파일 없음 - 정상
    json j = json::parse(f, nullptr, false);
    if (j.is_object()) users_ = j;
    else logf("[WARN] 사용자 파일 파싱 실패, 빈 상태로 시작: %s", file_.c_str());
}

void UserStore::save() {
    std::ofstream f(file_);
    if (!f) {
        logf("[ERROR] 사용자 파일 저장 실패: %s", file_.c_str());
        return;
    }
    f << users_.dump(2) << "\n";
}

bool UserStore::registerUser(const std::string& id, const std::string& pw,
                             const std::string& camIp, std::string& err) {
    if (id.empty() || pw.empty()) {
        err = "id/비밀번호는 비울 수 없음";
        return false;
    }
    std::lock_guard<std::mutex> lk(mtx_);
    if (users_.contains(id)) {
        err = "이미 존재하는 id";
        return false;
    }
    unsigned char salt[16];
    RAND_bytes(salt, sizeof(salt));
    std::string saltHex = toHex(salt, sizeof(salt));
    users_[id] = {{"salt", saltHex}, {"hash", hashPw(pw, saltHex)}, {"calib", nullptr},
                  {"cam_ip", camIp.empty() ? json() : json(camIp)}};
    save();
    return true;
}

bool UserStore::login(const std::string& id, const std::string& pw) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!users_.contains(id)) return false;
    auto& u = users_[id];
    return hashPw(pw, u.value("salt", "")) == u.value("hash", "");
}

void UserStore::loadGlobalCalib() {
    globalCalib_ = nullptr;
    std::ifstream f(calibFile_);
    if (!f) return;  // 아직 캘리를 한 번도 안 올렸으면 파일 없음 - 정상
    json j = json::parse(f, nullptr, false);
    if (j.is_object() || j.is_array()) globalCalib_ = j;
    else logf("[WARN] 전역 캘리브레이션 파싱 실패, 무시: %s", calibFile_.c_str());
}

void UserStore::saveGlobalCalib() {
    std::ofstream f(calibFile_);
    if (!f) {
        logf("[ERROR] 전역 캘리브레이션 저장 실패: %s", calibFile_.c_str());
        return;
    }
    f << globalCalib_.dump(2) << "\n";
}

bool UserStore::setGlobalCalib(const json& calib) {
    std::lock_guard<std::mutex> lk(mtx_);
    globalCalib_ = calib;
    saveGlobalCalib();
    return true;
}

json UserStore::getGlobalCalib() {
    std::lock_guard<std::mutex> lk(mtx_);
    return globalCalib_;
}

json UserStore::getCalib(const std::string& id) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!users_.contains(id)) return nullptr;
    json c = users_[id].value("calib", json());
    if (c.is_null()) c = users_[id].value("H", json());  // 구버전 파일 호환
    // 계정에 자기 캘리가 없으면 전역 슬롯으로 대체 (R-1). 계정 값이 있으면 그게
    // 이긴다 - 사용자가 특정 번들을 자기 계정에 고정해둔 경우를 덮지 않기 위해.
    if (c.is_null()) c = globalCalib_;
    return c;
}

bool UserStore::setCalib(const std::string& id, const json& calib) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!users_.contains(id)) return false;
    users_[id]["calib"] = calib;
    users_[id].erase("H");  // 구버전 키 정리
    save();
    return true;
}

json UserStore::getCamIp(const std::string& id) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!users_.contains(id)) return nullptr;
    return users_[id].value("cam_ip", json());
}

bool UserStore::setCamIp(const std::string& id, const std::string& camIp) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!users_.contains(id)) return false;
    // registerUser와 동일 규약: 빈 문자열은 "등록 안 함"을 뜻하는 null로 저장
    users_[id]["cam_ip"] = camIp.empty() ? json() : json(camIp);
    save();
    return true;
}
