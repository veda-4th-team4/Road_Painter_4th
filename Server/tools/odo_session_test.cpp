// 오도메트리 주행 캘리 세션 회귀 테스트 (2026-08-13, Qt 개시 지원).
//
// 사용법:
//   ./server 9100                              # 다른 창에서 서버를 띄워두고
//   make odo_session_test && tools/odo_session_test 127.0.0.1 9100
//
// calib_session_test.cpp(정적 앵커 방식)의 짝이다. 저쪽이 "Qt가 시작한 세션은
// 반드시 종결 응답 하나로 닫힌다"를 지킨다면, 이 파일은 **주행이 끝난 뒤에도
// 그 불변식이 유지되는가**를 지킨다.
//
// 🔴 이 테스트가 존재하는 이유. 오도메트리 세션은 서버 관점에서 CALIB_DONE에서
//   끝나는데, Qt의 대기 화면을 닫는 H_MATRIX는 그 **뒤에** 카메라가 만든다.
//   2026-08-12 구현은 CALIB_DONE 자리에서 clearCalib()을 불렀고, 그러면:
//     - 뒤늦게 온 H_MATRIX에 request_id가 안 실려 Qt가 종결로 못 알아본다
//     - 카메라의 CALIB_FAIL{too_few_points}이 "세션 없음"으로 버려진다
//   둘 다 조작자를 대기 화면에 가두는데, 크래시도 컴파일 에러도 아니라서
//   이렇게 못 박아두지 않으면 현장에서야 안다. T3/T4가 그 두 경로다.
//
// ⚠️ 서버 config/params.json의 calib_cancel_ack_ms를 건드렸다면 T6의 대기
//    시간(6초)도 같이 늘려야 한다.
#include "tls_client.hpp"
#include <chrono>
#include <deque>
#include <thread>

static const char* kCa = "certs/server.crt";
static int gPass = 0, gFail = 0;

class Inbox {
public:
    void push(const json& m) {
        std::lock_guard<std::mutex> lk(m_);
        q_.push_back(m);
    }
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
    // CMD 중 특정 cmd를 기다린다 (CALIB_START/SELECT_CHANNEL/CALIB_CANCEL 구분용).
    bool waitCmd(const std::string& cmd, int timeoutMs, json& out) {
        using namespace std::chrono;
        auto until = steady_clock::now() + milliseconds(timeoutMs);
        for (;;) {
            {
                std::lock_guard<std::mutex> lk(m_);
                for (auto it = q_.begin(); it != q_.end(); ++it) {
                    if (it->value("type", "") == "CMD" &&
                        (*it)["payload"].value("cmd", "") == cmd) {
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
    // CALIB_PROGRESS는 주행 중 9번 쌓이므로 type만으로 기다리면 늘 옛것이 먼저
    // 잡힌다. phase로 골라내야 "주행이 끝났다"를 실제로 확인할 수 있다.
    bool waitProgress(const std::string& phase, int timeoutMs, json& out) {
        using namespace std::chrono;
        auto until = steady_clock::now() + milliseconds(timeoutMs);
        for (;;) {
            {
                std::lock_guard<std::mutex> lk(m_);
                for (auto it = q_.begin(); it != q_.end(); ++it) {
                    if (it->value("type", "") == "CALIB_PROGRESS" &&
                        (*it)["payload"].value("phase", "") == phase) {
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

// 캘리 번들 한 벌 (H_MATRIX 회신용). 값의 정확성은 이 테스트의 관심사가 아니다 -
// 서버가 파싱에 성공해 세션을 닫는지만 본다.
static json calibBundle(int ch) {
    return {{"ch", ch},
            {"calib",
             {{"calib_id", "odo-test"},
              {"unit", "mm"},
              {"coord_mode", "undistort"},
              {"method", "robot_motion"},
              {"image_size", {2592, 1520}},
              {"K", {{1200.0, 0.0, 1296.0}, {0.0, 1200.0, 760.0}, {0.0, 0.0, 1.0}}},
              {"D", {0.0, 0.0, 0.0, 0.0, 0.0}},
              {"H_floor", {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
              {"H_marker", {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
              {"marker_height_mm", 160},
              {"origin_mm", {0, 0}},
              {"canvas_mm", {900, 600}},
              {"axis", "x_right_y_up"}}}};
}

// 로봇 + 카메라 대역. PATH를 받은 뒤 11개 op을 READY/GO로 끝까지 돌고
// PATH_DONE을 보낸다. 캡처 요청이 뜨면 CCTV 몫으로 OK를 회신한다.
//
// 반환: 서버가 CALIB_DONE을 CCTV에 보냈는가.
static bool driveWholeRect(TlsLink& robot, TlsLink& cctv, Inbox& robotBox,
                           Inbox& cctvBox, const std::string& reqId, int ch,
                           int failAtPoint = -1, const char* failReason = "") {
    json j;
    if (!robotBox.wait("PATH", 3000, j)) {
        tlogf("TEST", "PATH를 받지 못했다");
        return false;
    }
    const size_t nOps = j["payload"]["ops"].size();
    if (nOps != 11) tlogf("TEST", "⚠️ op 개수가 11이 아님: %zu", nOps);

    // op 0..10을 READY -> (필요하면 캡처 ack) -> GO 로 소화한다.
    for (int k = 0; k < (int)nOps; ++k) {
        robot.send("READY", {{"op_index", k}});
        // 캡처 대상 boundary면 서버가 CCTV에 CALIB_CAPTURE를 먼저 보낸다.
        // 아니면 곧장 GO가 온다 - 둘 중 뭐가 오는지는 서버가 정한다.
        if (cctvBox.waitCmd("__never__", 0, j)) {}  // no-op (형식 맞춤)
        json cap;
        if (cctvBox.wait("CALIB_CAPTURE", 700, cap)) {
            const int pi = cap["payload"].value("point_index", -1);
            if (pi == failAtPoint)
                cctv.send("CALIB_CAPTURE_FAIL",
                          {{"ch", ch}, {"request_id", reqId},
                           {"point_index", pi}, {"reason", failReason}});
            else
                cctv.send("CALIB_CAPTURE_OK",
                          {{"ch", ch}, {"request_id", reqId},
                           {"point_index", pi}, {"pixel_uv", {100.0 + pi, 200.0 + pi}},
                           {"spread_px", 0.4}});
        }
        if (!robotBox.wait("GO", 3000, j)) {
            tlogf("TEST", "op %d에서 GO를 받지 못했다", k);
            return false;
        }
    }
    // 마지막 op 완료 -> PATH_DONE (로봇은 READY 없이 곧장 보낸다, wire §4)
    robot.send("PATH_DONE", {{"phase", "calib"}});
    // 서버가 9번째(복귀) 캡처를 요청한다.
    json cap;
    if (!cctvBox.wait("CALIB_CAPTURE", 3000, cap)) {
        tlogf("TEST", "PATH_DONE 뒤 복귀 캡처 요청이 없다");
        return false;
    }
    const int pi = cap["payload"].value("point_index", -1);
    cctv.send("CALIB_CAPTURE_OK",
              {{"ch", ch}, {"request_id", reqId}, {"point_index", pi},
               {"pixel_uv", {100.0, 200.0}}, {"spread_px", 0.4}});
    return cctvBox.wait("CALIB_DONE", 3000, cap);
}

int main(int argc, char** argv) {
    SSL_library_init();
    SSL_load_error_strings();
    std::string ip = argc > 1 ? argv[1] : "127.0.0.1";
    int port = argc > 2 ? atoi(argv[2]) : 9000;
    std::string id = argc > 3 ? argv[3] : "_calibtest";
    std::string pw = argc > 4 ? argv[4] : "_calibtest";

    TlsLink qt("QT"), robot("ROBOT"), cctv("CCTV"), admin("ADMIN");
    if (!qt.connect(ip, port, kCa, "QT") || !robot.connect(ip, port, kCa, "ROBOT") ||
        !cctv.connect(ip, port, kCa, "CCTV") ||
        !admin.connect(ip, port, kCa, "ADMIN")) {
        tlogf("TEST", "접속 실패 - 서버가 떠 있는지, certs/server.crt 경로 확인");
        return 1;
    }
    Inbox qtBox, robotBox, cctvBox, adminBox;
    auto pump = [](TlsLink& l, Inbox& box) {
        json m;
        while (l.recv(m)) box.push(m);
    };
    std::thread tq(pump, std::ref(qt), std::ref(qtBox));
    std::thread tr(pump, std::ref(robot), std::ref(robotBox));
    std::thread tc(pump, std::ref(cctv), std::ref(cctvBox));
    std::thread ta(pump, std::ref(admin), std::ref(adminBox));

    json j;
    qt.send("LOGIN", {{"id", id}, {"pw", pw}});
    if (!qtBox.wait("LOGIN_OK", 3000, j)) {
        qtBox.clear();
        qt.send("REGISTER", {{"id", id}, {"pw", pw}});
        qtBox.wait("REGISTER_OK", 3000, j);
        qt.send("LOGIN", {{"id", id}, {"pw", pw}});
        if (!qtBox.wait("LOGIN_OK", 3000, j)) {
            tlogf("TEST", "LOGIN 실패 - 중단");
            return 1;
        }
    }

    const int ch = 2;
    auto odoStart = [&](TlsLink& link, const char* rid, double m_cm, double n_cm,
                        const char* corner) {
        link.send("CMD", {{"cmd", "CALIB_START"}, {"ch", ch}, {"request_id", rid},
                          {"method", "robot_motion"}, {"m_cm", m_cm},
                          {"n_cm", n_cm}, {"start_corner", corner}});
    };

    // ---- T0: 판별자 - 오도메트리 필드가 없으면 정적 앵커로 가야 한다 ----
    // 🔴 2026-08-12~13의 회귀. method:"robot_motion"은 Qt의 정적 앵커 요청에도
    //   실리는 값이라(2026-08-10 계약) 판별자가 될 수 없다.
    qtBox.clear(); cctvBox.clear(); robotBox.clear();
    qt.send("CMD", {{"cmd", "CALIB_START"}, {"ch", ch}, {"request_id", "t0"},
                    {"method", "robot_motion"}});
    check(qtBox.wait("CALIB_STARTED", 2000, j), "T0a method만으로는 오도메트리가 아니다(정적 앵커)");
    check(robotBox.absent("PATH", 800), "T0b 정적 앵커에는 PATH를 보내지 않는다");
    qt.send("CMD", {{"cmd", "CALIB_CANCEL"}, {"ch", ch}, {"request_id", "t0"}});
    robot.send("CALIB_STOPPED", {{"ch", ch}});
    cctv.send("CALIB_STOPPED", {{"ch", ch}});
    qtBox.wait("CALIB_CANCELLED", 3000, j);

    // ---- T1: Qt 개시 수락 + 개시 응답 3종 ----
    qtBox.clear(); cctvBox.clear(); robotBox.clear();
    odoStart(qt, "odo-1", 90, 60, "bottom_left");
    check(qtBox.wait("CALIB_STARTED", 2000, j) &&
              j["payload"].value("request_id", "") == "odo-1",
          "T1a Qt 개시 수락 -> CALIB_STARTED");
    check(qtBox.waitProgress("driving", 2000, j) &&
              j["payload"].value("total", 0) == 9,
          "T1b 개시 직후 CALIB_PROGRESS{driving}");
    check(cctvBox.waitCmd("SELECT_CHANNEL", 2000, j), "T1c CCTV에 SELECT_CHANNEL");
    check(robotBox.waitCmd("CALIB_START", 2000, j), "T1d ROBOT에 CALIB_START 중계");

    // ---- T2: 주행 완주 -> CALIB_DONE, 그러나 세션은 아직 안 닫힌다 ----
    check(driveWholeRect(robot, cctv, robotBox, cctvBox, "odo-1", ch),
          "T2a 11-op 완주 -> CCTV에 CALIB_DONE");
    check(qtBox.waitProgress("solving", 2000, j),
          "T2b 주행 종료 -> CALIB_PROGRESS{solving}");
    // 🔴 핵심: 여기서 세션이 닫혀 있으면 안 된다.
    qt.send("CMD", {{"cmd", "CALIB_START"}, {"ch", 3}, {"request_id", "odo-x"}});
    check(qtBox.wait("CALIB_FAIL", 2000, j) &&
              j["payload"].value("reason", "") == "busy" &&
              j["payload"].value("owner", "") == "QT",
          "T2c CALIB_DONE 뒤에도 세션 유지(busy) + owner=QT");

    // ---- T3: 뒤늦게 온 H_MATRIX가 종결 응답이 된다 ----
    qtBox.clear();
    cctv.send("H_MATRIX", calibBundle(ch));
    check(qtBox.wait("H_MATRIX", 3000, j) &&
              j["payload"].value("request_id", "") == "odo-1",
          "T3a 결과 대기 중 H_MATRIX에 request_id가 붙어 종결");
    odoStart(qt, "odo-2", 90, 60, "bottom_left");
    check(qtBox.wait("CALIB_STARTED", 2000, j), "T3b 종결 후 새 요청 수락(세션이 실제로 닫힘)");

    // ---- T4: 카메라가 CALIB_DONE 뒤 CALIB_FAIL을 올리는 경로 ----
    check(driveWholeRect(robot, cctv, robotBox, cctvBox, "odo-2", ch),
          "T4a 두 번째 주행 완주");
    qtBox.clear();
    cctv.send("CALIB_FAIL", {{"ch", ch}, {"reason", "too_few_points"},
                             {"msg", "유효 대응점이 부족합니다"}});
    check(qtBox.wait("CALIB_FAIL", 3000, j) &&
              j["payload"].value("reason", "") == "too_few_points" &&
              j["payload"].value("request_id", "") == "odo-2",
          "T4b 결과 대기 중 CALIB_FAIL이 버려지지 않고 종결 응답이 된다");

    // ---- T5: 치수 검증 ----
    qtBox.clear();
    odoStart(qt, "odo-3", 90, 60, "sideways");
    check(qtBox.wait("CALIB_FAIL", 2000, j) &&
              j["payload"].value("reason", "") == "invalid_param",
          "T5a 잘못된 start_corner -> invalid_param");
    odoStart(qt, "odo-4", 1, 60, "bottom_left");
    check(qtBox.wait("CALIB_FAIL", 2000, j) &&
              j["payload"].value("reason", "") == "invalid_param",
          "T5b 너무 작은 m_cm -> invalid_param");

    // ---- T6: 소유권 - 관리자 창이 Qt 세션을 그냥은 못 건드린다 ----
    qtBox.clear(); adminBox.clear(); robotBox.clear(); cctvBox.clear();
    odoStart(qt, "odo-5", 90, 60, "bottom_left");
    qtBox.wait("CALIB_STARTED", 2000, j);
    robotBox.wait("PATH", 3000, j);  // 주행 중 상태로 둔다
    admin.send("CMD", {{"cmd", "CALIB_CANCEL"}, {"ch", ch}});
    check(qtBox.absent("CALIB_CANCELLED", 1200),
          "T6a 관리자의 일반 취소로 QT 세션이 죽지 않는다");
    // 관리자 창이 시작한 적 없는 세션이므로 QT에 취소 통지가 가서도 안 된다.

    // ---- T7: 강제 회수는 CALIB_CANCELLED가 아니라 CALIB_FAIL{preempted} ----
    qtBox.clear();
    admin.send("CMD", {{"cmd", "CALIB_CANCEL"}, {"ch", ch}, {"force", true}});
    check(robotBox.waitCmd("CALIB_CANCEL", 2000, j),
          "T7a 강제 회수 -> ROBOT에 정지 명령");
    robot.send("CALIB_STOPPED", {{"ch", ch}});
    cctv.send("CALIB_STOPPED", {{"ch", ch}});
    check(qtBox.wait("CALIB_FAIL", 3000, j) &&
              j["payload"].value("reason", "") == "preempted",
          "T7b 강제 회수는 CALIB_FAIL{preempted} (취소로 위장하지 않는다)");

    // ---- T8: 진행 중인 세션이 없을 때 관리자의 취소가 QT로 새지 않는다 ----
    qtBox.clear();
    admin.send("CMD", {{"cmd", "CALIB_CANCEL"}, {"ch", ch}});
    check(qtBox.absent("CALIB_CANCELLED", 1200),
          "T8 세션 없을 때 관리자 취소 -> QT에 통지 안 감");

    // ---- T9: Qt 소유 세션에서 Qt가 끊기면 로봇을 세운다 ----
    qtBox.clear(); robotBox.clear(); cctvBox.clear();
    odoStart(qt, "odo-6", 90, 60, "bottom_left");
    qtBox.wait("CALIB_STARTED", 2000, j);
    robotBox.wait("PATH", 3000, j);
    qt.close();  // 조작자 단말이 죽은 상황
    check(robotBox.waitCmd("CALIB_CANCEL", 3000, j),
          "T9 QT 이탈 -> 주인 없는 로봇에 정지 명령");
    robot.send("CALIB_STOPPED", {{"ch", ch}});
    cctv.send("CALIB_STOPPED", {{"ch", ch}});

    tlogf("TEST", "===== 통과 %d / 실패 %d =====", gPass, gFail);
    robot.close();
    cctv.close();
    admin.close();
    if (tq.joinable()) tq.join();
    if (tr.joinable()) tr.join();
    if (tc.joinable()) tc.join();
    if (ta.joinable()) ta.join();
    return gFail == 0 ? 0 : 1;
}
