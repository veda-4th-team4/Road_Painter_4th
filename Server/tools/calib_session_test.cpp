// 로봇 주행 호모그래피 세션 회귀 테스트 (2026-08-10 계약 §6 시험표).
//
// 사용법:
//   ./server                                  # 다른 창에서 서버를 띄워두고
//   make calib_session_test && tools/calib_session_test [ip] [port] [id] [pw]
//
// QT / ROBOT / CCTV 세 role로 동시에 붙어 계약서 §6 표를 그대로 돌린다.
// 🔴 이 테스트가 지키려는 단 하나의 성질: **시작한 세션은 반드시 종결 응답
//   하나로 닫힌다.** Qt는 CALIB_START 뒤 전체 화면 대기로 들어가므로, 서버가
//   응답을 빠뜨리면 조작자가 5분간 아무것도 못 한다. "응답이 안 왔다"는
//   컴파일 에러도 크래시도 아니라서, 이렇게 못 박아두지 않으면 현장에서야 안다.
//
// ⚠️ 서버 config/params.json의 calib_cancel_ack_ms를 건드렸다면 아래 T7의
//    대기 시간(6초)도 같이 늘려야 한다.
#include "tls_client.hpp"
#include <chrono>
#include <deque>
#include <thread>

static const char* kCa = "certs/server.crt";
static int gPass = 0, gFail = 0;

// ---------------------------------------------------------------------------
// 수신 스레드가 채우는 큐. 테스트 본문은 "이 타입이 N초 안에 오는가"만 묻는다.
// ---------------------------------------------------------------------------
class Inbox {
public:
    void push(const json& m) {
        std::lock_guard<std::mutex> lk(m_);
        q_.push_back(m);
    }
    // type과 일치하는 첫 메시지를 꺼낸다. 없으면 timeoutMs 뒤 false.
    // 일치하지 않는 메시지는 큐에 남긴다 - 순서를 가정하지 않기 위함.
    bool wait(const std::string& type, int timeoutMs, json& out) {
        using namespace std::chrono;
        auto until = steady_clock::now() + milliseconds(timeoutMs);
        for (;;) {
            {
                std::lock_guard<std::mutex> lk(m_);
                for (auto it = q_.begin(); it != q_.end(); ++it) {
                    if (it->value("type", "") == type) {
                        out = *it;
                        q_.erase(it);
                        return true;
                    }
                }
            }
            if (steady_clock::now() >= until) return false;
            std::this_thread::sleep_for(milliseconds(20));
        }
    }
    // 이 타입이 timeoutMs 동안 **오지 않아야** 한다 (늦은 결과가 대기를 풀지
    // 않는지 확인할 때 쓴다).
    bool absent(const std::string& type, int timeoutMs) {
        json j;
        return !wait(type, timeoutMs, j);
    }
    void clear() {
        std::lock_guard<std::mutex> lk(m_);
        q_.clear();
    }

private:
    std::mutex m_;
    std::deque<json> q_;
};

static void check(bool ok, const char* name, const std::string& detail = "") {
    if (ok) {
        ++gPass;
        tlogf("PASS", "%s", name);
    } else {
        ++gFail;
        tlogf("FAIL", "%s %s", name, detail.c_str());
    }
}

int main(int argc, char** argv) {
    SSL_library_init();
    SSL_load_error_strings();
    std::string ip = argc > 1 ? argv[1] : "127.0.0.1";
    int port = argc > 2 ? atoi(argv[2]) : 9000;
    // ⚠️ 로그인이 안 되면 이 계정을 서버의 config/users.json에 만든다. QT 개시
    //    요청은 로그인을 요구하는데(계약 §4-1), 테스트가 현장 계정의 비밀번호를
    //    알 수는 없기 때문이다. 이름을 따로 둔 것은 실제 조작자 계정에 테스트
    //    캘리 결과가 저장되지 않게 하기 위함이다.
    std::string id = argc > 3 ? argv[3] : "_calibtest";
    std::string pw = argc > 4 ? argv[4] : "_calibtest";

    TlsLink qt("QT"), robot("ROBOT"), cctv("CCTV");
    if (!qt.connect(ip, port, kCa, "QT") || !robot.connect(ip, port, kCa, "ROBOT") ||
        !cctv.connect(ip, port, kCa, "CCTV")) {
        tlogf("TEST", "접속 실패 - 서버가 떠 있는지, certs/server.crt 경로가 맞는지 확인");
        return 1;
    }
    Inbox qtBox, robotBox, cctvBox;
    // 링크가 닫히면 recv()가 false를 돌려주므로 별도 종료 플래그가 필요 없다.
    // (공용 플래그를 쓰면 T11에서 ROBOT 하나를 끊으려다 QT/CCTV 수신까지 멈춘다)
    auto pump = [](TlsLink& l, Inbox& box) {
        json m;
        while (l.recv(m)) box.push(m);
    };
    std::thread tq(pump, std::ref(qt), std::ref(qtBox));
    std::thread tr(pump, std::ref(robot), std::ref(robotBox));
    std::thread tc(pump, std::ref(cctv), std::ref(cctvBox));

    json j;
    // 로그인 - 서버는 QT 개시 요청에 로그인 사용자를 요구한다.
    qt.send("LOGIN", {{"id", id}, {"pw", pw}});
    bool loggedIn = qtBox.wait("LOGIN_OK", 3000, j);
    if (!loggedIn) {
        tlogf("TEST", "LOGIN 실패 - '%s' 계정을 새로 만들고 다시 시도", id.c_str());
        qtBox.clear();
        qt.send("REGISTER", {{"id", id}, {"pw", pw}});
        qtBox.wait("REGISTER_OK", 3000, j);
        qt.send("LOGIN", {{"id", id}, {"pw", pw}});
        loggedIn = qtBox.wait("LOGIN_OK", 3000, j);
    }
    if (!loggedIn) {
        tlogf("TEST", "LOGIN 실패 - 서버 로그를 확인할 것");
        qt.shutdownRead(); robot.shutdownRead(); cctv.shutdownRead();
        tq.join(); tr.join(); tc.join();
        return 1;
    }

    // 캘리 결과 번들. 평면 스키마(payload 자체가 번들)로 보낸다 - 서버가 payload를
    // 통째로 갈아치우는 경로라 request_id가 지워지기 가장 쉬운 형태다.
    auto bundle = [](int ch) {
        return json{{"ch", ch},
                    {"H", {{1000, 0, 0}, {0, 1000, 0}, {0, 0, 1}}},
                    {"canvas_mm", {6000, 4000}}};
    };

    // ---- T1: 정상 왕복 (CH2) - CALIB_STARTED -> H_MATRIX{ch,request_id} ----
    qtBox.clear(); cctvBox.clear(); robotBox.clear();
    qt.send("CMD", {{"cmd", "CALIB_START"}, {"ch", 2},
                    {"request_id", "rid-1"}, {"method", "robot_motion"}});
    check(qtBox.wait("CALIB_STARTED", 2000, j) &&
              j["payload"].value("request_id", "") == "rid-1" &&
              j["payload"].value("ch", 0) == 2,
          "T1a CALIB_STARTED{ch=2, rid-1} 수신");
    check(cctvBox.wait("CMD", 2000, j) &&
              j["payload"].value("cmd", "") == "SELECT_CHANNEL" &&
              j["payload"].value("ch", 0) == 2,
          "T1b CCTV가 SELECT_CHANNEL{ch=2}를 먼저 받음");
    // CALIB_START 원본이 ch/request_id를 보존한 채 양쪽에 갔는가 (계약 §2-1)
    bool cctvGotStart = false, robotGotStart = false;
    while (cctvBox.wait("CMD", 500, j))
        if (j["payload"].value("cmd", "") == "CALIB_START")
            cctvGotStart = j["payload"].value("ch", 0) == 2 &&
                           j["payload"].value("request_id", "") == "rid-1";
    while (robotBox.wait("CMD", 500, j))
        if (j["payload"].value("cmd", "") == "CALIB_START")
            robotGotStart = j["payload"].value("ch", 0) == 2 &&
                            j["payload"].value("request_id", "") == "rid-1";
    check(cctvGotStart, "T1c CCTV의 CALIB_START에 ch/request_id 보존");
    check(robotGotStart, "T1d ROBOT의 CALIB_START에 ch/request_id 보존");

    // ---- T2: 진행 중 재요청 ----
    qt.send("CMD", {{"cmd", "CALIB_START"}, {"ch", 2}, {"request_id", "rid-1"}});
    check(qtBox.wait("CALIB_STARTED", 2000, j) &&
              j["payload"].value("request_id", "") == "rid-1",
          "T2a 같은 request_id 재요청 -> CALIB_STARTED 재전송(멱등)");
    qt.send("CMD", {{"cmd", "CALIB_START"}, {"ch", 3}, {"request_id", "rid-2"}});
    check(qtBox.wait("CALIB_FAIL", 2000, j) &&
              j["payload"].value("reason", "") == "busy",
          "T2b 다른 request_id 재요청 -> busy");

    // ---- T3: 캘리 중 채널 전환 차단 ----
    qt.send("CMD", {{"cmd", "SELECT_CHANNEL"}, {"ch", 4}});
    check(qtBox.wait("CHANNEL_FAIL", 2000, j) &&
              j["payload"].value("reason", "") == "calib_busy",
          "T3 캘리 중 SELECT_CHANNEL -> CHANNEL_FAIL{calib_busy}");

    // ---- T4: CCTV 진행률 중계 (ch/request_id를 서버가 채움) ----
    cctv.send("CALIB_PROGRESS", {{"progress", 0.45}, {"stage", "collecting_samples"}});
    check(qtBox.wait("CALIB_PROGRESS", 2000, j) &&
              j["payload"].value("request_id", "") == "rid-1" &&
              j["payload"].value("ch", 0) == 2,
          "T4 CALIB_PROGRESS에 서버가 ch/request_id를 채워 중계");

    // ---- T5: 다른 채널의 늦은 결과는 대기를 풀지 않는다 (계약 §6) ----
    cctv.send("H_MATRIX", bundle(1));
    check(qtBox.wait("H_MATRIX", 2000, j) &&
              !j["payload"].contains("request_id"),
          "T5a 다른 채널 H_MATRIX는 중계되지만 request_id가 안 붙음");
    // 세션이 아직 살아 있어야 한다 - 살아 있으면 새 요청이 여전히 busy다.
    qt.send("CMD", {{"cmd", "CALIB_START"}, {"ch", 3}, {"request_id", "rid-3"}});
    check(qtBox.wait("CALIB_FAIL", 2000, j) &&
              j["payload"].value("reason", "") == "busy",
          "T5b 늦은 결과 뒤에도 세션 유지(여전히 busy)");

    // ---- T6: 해당 채널 결과 -> 종결 ----
    cctv.send("H_MATRIX", bundle(2));
    check(qtBox.wait("H_MATRIX", 2000, j) &&
              j["payload"].value("request_id", "") == "rid-1" &&
              j["payload"].value("ch", 0) == 2,
          "T6a 해당 채널 H_MATRIX에 request_id가 붙어 종결");
    qt.send("CMD", {{"cmd", "CALIB_START"}, {"ch", 1}, {"request_id", "rid-4"}});
    check(qtBox.wait("CALIB_STARTED", 2000, j) &&
              j["payload"].value("request_id", "") == "rid-4",
          "T6b 종결 후 새 요청 수락(세션이 실제로 닫힘)");

    // ---- T7: 취소 - ACK가 없으면 cancel_failed ----
    qt.send("CMD", {{"cmd", "CALIB_CANCEL"}, {"ch", 1}, {"request_id", "rid-4"}});
    check(qtBox.absent("CALIB_CANCELLED", 1500),
          "T7a 중계만으로 CALIB_CANCELLED를 보내지 않음");
    check(qtBox.wait("CALIB_FAIL", 6000, j) &&
              j["payload"].value("reason", "") == "cancel_failed",
          "T7b 정지 ACK 없음 -> CALIB_FAIL{cancel_failed}");

    // ---- T8: 취소 - 양쪽 ACK가 오면 CALIB_CANCELLED ----
    qtBox.clear();
    qt.send("CMD", {{"cmd", "CALIB_START"}, {"ch", 1}, {"request_id", "rid-5"}});
    qtBox.wait("CALIB_STARTED", 2000, j);
    qt.send("CMD", {{"cmd", "CALIB_CANCEL"}, {"ch", 1}, {"request_id", "rid-5"}});
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    robot.send("CALIB_STOPPED", json::object());
    check(qtBox.absent("CALIB_CANCELLED", 800),
          "T8a ROBOT만 정지 확인 -> 아직 CALIB_CANCELLED 없음");
    cctv.send("CALIB_STOPPED", json::object());
    check(qtBox.wait("CALIB_CANCELLED", 2000, j) &&
              j["payload"].value("request_id", "") == "rid-5",
          "T8b 양쪽 정지 확인 -> CALIB_CANCELLED");

    // ---- T9: 잘못된 채널 ----
    qt.send("CMD", {{"cmd", "CALIB_START"}, {"ch", 99}, {"request_id", "rid-6"}});
    check(qtBox.wait("CALIB_FAIL", 2000, j) &&
              j["payload"].value("reason", "") == "invalid_channel",
          "T9a 범위 밖 채널 -> invalid_channel");
    qt.send("CMD", {{"cmd", "CALIB_START"}, {"request_id", "rid-7"}});
    check(qtBox.wait("CALIB_FAIL", 2000, j) &&
              j["payload"].value("reason", "") == "invalid_channel",
          "T9b QT는 ch 생략 불가 -> invalid_channel");

    // ---- T10: CCTV가 실패를 알리면 그대로 종결 ----
    qtBox.clear();
    qt.send("CMD", {{"cmd", "CALIB_START"}, {"ch", 2}, {"request_id", "rid-8"}});
    qtBox.wait("CALIB_STARTED", 2000, j);
    cctv.send("CALIB_FAIL", {{"reason", "insufficient_samples"},
                             {"msg", "관측점이 부족합니다"}});
    check(qtBox.wait("CALIB_FAIL", 2000, j) &&
              j["payload"].value("reason", "") == "insufficient_samples" &&
              j["payload"].value("request_id", "") == "rid-8",
          "T10 CCTV CALIB_FAIL -> 사유 그대로 + request_id 붙여 종결");

    // ---- T11: 로봇이 빠지면 즉시 종결 (타임아웃까지 안 기다린다) ----
    qtBox.clear();
    qt.send("CMD", {{"cmd", "CALIB_START"}, {"ch", 2}, {"request_id", "rid-9"}});
    qtBox.wait("CALIB_STARTED", 2000, j);
    robot.shutdownRead();     // ROBOT만 끊는다 (QT/CCTV 수신은 계속 살아 있어야 함)
    tr.join();
    robot.close();
    check(qtBox.wait("CALIB_FAIL", 3000, j) &&
              j["payload"].value("reason", "") == "robot_offline",
          "T11a 세션 중 로봇 이탈 -> 즉시 robot_offline");
    qt.send("CMD", {{"cmd", "CALIB_START"}, {"ch", 2}, {"request_id", "rid-10"}});
    check(qtBox.wait("CALIB_FAIL", 2000, j) &&
              j["payload"].value("reason", "") == "robot_offline",
          "T11b 로봇 없을 때 시작 요청 -> 즉시 robot_offline (허공 전송 안 함)");

    // ---- 뒷정리: 활성 채널을 되돌린다 ----
    // 🔴 activeChannel_은 접속과 무관하게 서버에 남는 현장 상태다. 이 테스트는
    //   CH2로 끝나는데, 같은 서버에 이어 붙는 robot_sim은 POS를 항상 CH1로 보내
    //   서버가 전부 버린다("POS 채널 1 무시"). 그러면 pose가 안 잡혀 START_DRAW가
    //   no_pose로 죽는데, 주행 테스트 쪽에서 보면 증상이 "판정이 하나도 안 돈다"라
    //   **피드백 회귀와 구분되지 않는다** (실제로 한 번 오진했다).
    //   공유 상태를 건드린 테스트는 스스로 원상복구한다.
    // ⚠️ 큐를 먼저 비운다. 앞선 CALIB_START들이 CHANNEL_OK{ch:2}를 남겨뒀는데,
    //    wait()는 타입만 보고 가장 오래된 것을 꺼내므로 그 옛것에 걸린다.
    qtBox.clear();
    qt.send("CMD", {{"cmd", "SELECT_CHANNEL"}, {"ch", 1}});
    if (qtBox.wait("CHANNEL_OK", 2000, j) && j["payload"].value("ch", 0) == 1)
        tlogf("TEST", "활성 채널을 1로 복구");
    else
        tlogf("TEST", "⚠️ 활성 채널 복구 실패 - 이 서버로 주행 테스트를 이어서 "
                      "돌리면 POS가 전부 버려진다");

    qt.shutdownRead();
    cctv.shutdownRead();
    tq.join();
    tc.join();
    tlogf("TEST", "===== 통과 %d / 실패 %d =====", gPass, gFail);
    return gFail == 0 ? 0 : 1;
}
