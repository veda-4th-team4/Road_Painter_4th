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
    load();
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
                             std::string& err) {
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
    users_[id] = {{"salt", saltHex}, {"hash", hashPw(pw, saltHex)}, {"H", nullptr}};
    save();
    return true;
}

bool UserStore::login(const std::string& id, const std::string& pw) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!users_.contains(id)) return false;
    auto& u = users_[id];
    return hashPw(pw, u.value("salt", "")) == u.value("hash", "");
}

json UserStore::getH(const std::string& id) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!users_.contains(id)) return nullptr;
    return users_[id].value("H", json());
}

bool UserStore::setH(const std::string& id, const json& H) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!users_.contains(id)) return false;
    users_[id]["H"] = H;
    save();
    return true;
}
