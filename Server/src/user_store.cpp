#include "user_store.hpp"
#include "calib.hpp"  // asCalibChannelMap / calibOfChannel (채널별 저장 형식)
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
    const std::string dir = (pos == std::string::npos ? std::string()
                                                      : file_.substr(0, pos + 1));
    calibFile_ = dir + "calib_latest.json";
    // 전역 카메라 IP도 같은 이유로 users.json 바깥에 둔다 - 현장 설비 정보라
    // 계정 파일(비번 해시 포함, .gitignore 대상)과 수명이 다르다.
    camFile_ = dir + "camera.json";
    load();
    loadGlobalCalib();
    loadGlobalCam();
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
    globalCalib_ = json::object();
    std::ifstream f(calibFile_);
    if (!f) return;  // 아직 캘리를 한 번도 안 올렸으면 파일 없음 - 정상
    json j = json::parse(f, nullptr, false);
    if (!j.is_object() && !j.is_array()) {
        logf("[WARN] 전역 캘리브레이션 파싱 실패, 무시: %s", calibFile_.c_str());
        return;
    }
    // 파일에는 예전 형식(번들 하나)이 들어 있을 수 있다 - 채널 1의 맵으로 승격.
    // 여기서 한 번 정규화해두면 이후 코드는 항상 맵만 보면 된다.
    const bool wasFlat = !isCalibChannelMap(j);
    globalCalib_ = asCalibChannelMap(j);
    if (wasFlat)
        logf("[INFO] 전역 캘리브레이션이 구 형식(번들 하나) - 채널 %d의 번들로 읽음",
             kMinChannel);
}

void UserStore::saveGlobalCalib() {
    std::ofstream f(calibFile_);
    if (!f) {
        logf("[ERROR] 전역 캘리브레이션 저장 실패: %s", calibFile_.c_str());
        return;
    }
    f << globalCalib_.dump(2) << "\n";
}

bool UserStore::setGlobalCalib(int ch, const json& calib) {
    std::lock_guard<std::mutex> lk(mtx_);
    // 채널 하나만 갈아끼운다. 통째로 대입하면 다른 채널의 번들이 날아가서,
    // 채널 2를 캘리하는 순간 채널 1이 사라진다.
    globalCalib_ = asCalibChannelMap(globalCalib_);
    globalCalib_[chKey(ch)] = calib;
    saveGlobalCalib();
    return true;
}

json UserStore::getGlobalCalib(int ch) {
    std::lock_guard<std::mutex> lk(mtx_);
    return calibOfChannel(globalCalib_, ch);
}

// 계정에 저장된 캘리브레이션 맵 (구버전 "H" 키 + 번들 하나 형식까지 흡수).
// mtx_를 잡은 상태에서만 호출할 것.
json UserStore::accountCalibMap(const std::string& id) {
    if (!users_.contains(id)) return json::object();
    json c = users_[id].value("calib", json());
    if (c.is_null()) c = users_[id].value("H", json());  // 구버전 파일 호환
    return asCalibChannelMap(c);
}

json UserStore::getCalib(const std::string& id, int ch) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!users_.contains(id)) return nullptr;
    json c = calibOfChannel(accountCalibMap(id), ch);
    // 계정에 그 채널의 캘리가 없으면 전역 슬롯으로 대체 (R-1). 계정 값이 있으면
    // 그게 이긴다 - 사용자가 특정 번들을 자기 계정에 고정해둔 경우를 덮지 않기 위해.
    // 🔴 판정은 채널 단위다. 계정에 채널 1만 있으면 채널 2는 전역에서 온다.
    if (c.is_null()) c = calibOfChannel(globalCalib_, ch);
    return c;
}

json UserStore::getCalibs(const std::string& id) {
    std::lock_guard<std::mutex> lk(mtx_);
    // 전역을 깔고 계정 값으로 덮는다 - getCalib의 "계정 → 없으면 전역"과 같은
    // 우선순위를 맵 전체에 한 번에 적용한 것.
    json out = asCalibChannelMap(globalCalib_);
    if (users_.contains(id)) {
        const json acc = accountCalibMap(id);
        for (auto it = acc.begin(); it != acc.end(); ++it)
            if (!it->is_null()) out[it.key()] = *it;
    }
    return out;
}

bool UserStore::setCalib(const std::string& id, int ch, const json& calib) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!users_.contains(id)) return false;
    json m = accountCalibMap(id);   // 구 형식이면 여기서 채널 1의 맵으로 승격된다
    m[chKey(ch)] = calib;           // 그 채널만 교체 - 나머지 채널은 그대로
    users_[id]["calib"] = m;
    users_[id].erase("H");  // 구버전 키 정리
    save();
    return true;
}

void UserStore::loadGlobalCam() {
    globalCamIp_ = json();
    std::ifstream f(camFile_);
    if (!f) return;  // 아직 카메라를 등록 안 했으면 파일 없음 - 정상
    json j = json::parse(f, nullptr, false);
    if (!j.is_object()) {
        logf("[WARN] 전역 카메라 설정 파싱 실패, 무시: %s", camFile_.c_str());
        return;
    }
    // 오브젝트로 감싸 둔다 - 나중에 계정/포트/프로파일 경로가 더 붙어도 파일
    // 형식을 안 바꾸고 키만 늘리면 되게.
    const json ip = j.value("cam_ip", json());
    if (ip.is_string() && !ip.get<std::string>().empty()) {
        globalCamIp_ = ip;
        logf("[INFO] 전역 카메라 IP 로드: %s", ip.get<std::string>().c_str());
    }
}

void UserStore::saveGlobalCam() {
    std::ofstream f(camFile_);
    if (!f) {
        logf("[ERROR] 전역 카메라 설정 저장 실패: %s", camFile_.c_str());
        return;
    }
    f << json{{"cam_ip", globalCamIp_}}.dump(2) << "\n";
}

bool UserStore::setGlobalCamIp(const std::string& camIp) {
    std::lock_guard<std::mutex> lk(mtx_);
    globalCamIp_ = camIp.empty() ? json() : json(camIp);
    saveGlobalCam();
    return true;
}

json UserStore::getGlobalCamIp() {
    std::lock_guard<std::mutex> lk(mtx_);
    return globalCamIp_;
}

json UserStore::getCamIp(const std::string& id) {
    std::lock_guard<std::mutex> lk(mtx_);
    // 계정 값 → 없으면 전역 값. getCalib과 같은 우선순위다 - 계정에 IP를 고정해둔
    // 현장을 전역이 덮지 않는다.
    if (users_.contains(id)) {
        const json c = users_[id].value("cam_ip", json());
        if (!c.is_null()) return c;
    }
    return globalCamIp_;
}

bool UserStore::setCamIp(const std::string& id, const std::string& camIp) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!users_.contains(id)) return false;
    // registerUser와 동일 규약: 빈 문자열은 "등록 안 함"을 뜻하는 null로 저장
    users_[id]["cam_ip"] = camIp.empty() ? json() : json(camIp);
    save();
    return true;
}
