// 최초 1회 경로생성 테스트기 (서버 RPi <-> 로봇 RPi 검증용) - TCP/TLS
//
// 목적:
//   CCTV가 딱 한 번 만들어 폴더에 넣어둔 자료(호모그래피 + 로봇 시작 네 꼭짓점)를
//   읽어서, 서버에 CCTV+QT 역할로 흘려넣는다. 그러면 서버의 "기존 경로생성 로직"이
//   그대로 돌아 로봇을 (0,0)으로 보내는 approach PATH(TURN/MOVE 시퀀스)를 딱 한 번
//   생성해 로봇에 전송한다. 실시간 피드백(DRIFT/이탈 재계획)은 자연히 빠진다
//   (POS를 한 번만 주고, approach 단계는 서버가 재계획을 하지 않음).
//
//   ※ 이 도구는 좌표/각도 계산을 전혀 하지 않는다. undistort->H_marker->pose 계산과
//     TURN/MOVE 시퀀스 생성은 전부 서버(src/path_planner.hpp, src/router.cpp)가 한다.
//     도구는 "입력 주입 + 응답 로깅"만 담당한다.
//
// 흐름:
//   1) QT 역할로 접속 + HELLO           (POSE/DRAW_FAIL 등 서버 응답을 여기서 관찰)
//   2) CCTV 역할로 접속 + HELLO
//   3) CCTV -> H_MATRIX(calib)          (서버가 호모그래피 번들 적재)
//   4) CCTV -> POS(corners)             (서버가 pose 계산 -> QT에 POSE 전송)
//   5) QT   -> BLUEPRINT(kTargetPoints)  (서버가 1단계 approach PATH 생성 -> ROBOT)
//   6) 로봇이 접근점 도착 후, Enter -> QT가 START_DRAW -> 서버가 2단계 도색 PATH 전송
//   -> 결과는 로봇 RPi 콘솔(PATH 수신)과 서버 콘솔([INFO] 접근/도색 경로 전송)에서 확인.
//
// 빌드: make path_test   (Server/ 디렉토리에서)
// 사용: ./path_test <서버IP> [server.crt경로] [스냅샷.json]
//        기본 crt      = ../certs/server.crt
//        기본 스냅샷   = tools/sample_snapshot.json  (실제 CCTV 자료 생기면 그 파일 지정)
//
// 스냅샷 파일 포맷 (CCTV가 채워줄 자료 - 나중에 실제 포맷이 정해지면
//   loadSnapshot() 한 함수만 고치면 된다):
//   {
//     "H": [[..]x3],             // pixel -> world "mm" 호모그래피 (CCTV가 준 그대로, undistort 미적용)
//     "corners": [[u1,v1],[u2,v2],[u3,v3],[u4,v4]]  // 로봇 마커 4코너 원본 픽셀
//   }
//   - 단위: H는 mm 기준(pixel->world mm). mm->m 환산은 서버가 입구에서 수행하므로 도구는
//     원본 mm를 그대로 올린다 (여기서 환산하면 이중 스케일). world_unit 필드는 이제 불필요.
//   - corners 순서는 서버가 [전좌,전우,후우,후좌]로 해석한다(앞변=[0,1], 뒷변=[2,3]).
//     CCTV의 detectMarkers 순서(TL,TR,BR,BL)를 그대로 넣으면 마커 윗변을 로봇 '앞'으로
//     본다. 계산된 heading이 뒤집혀 나오면 마커 장착 방향에 맞춰 코너 순서만 바꾸면 된다.
//   - K/D(왜곡계수)는 넣지 않는다: CCTV가 undistort를 적용하지 않았으므로 서버도 생략.
//   - "calib":{...} 번들을 직접 넣어도 된다(이 경우도 H는 mm 기준 - 서버가 환산).
#include <nlohmann/json.hpp>
#include <openssl/err.h>
#include <openssl/ssl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

using json = nlohmann::json;

// ── 목표 도면 (mm 기준 - CCTV와 동일 단위로 통일) ─────────────────────────────
// 서버 2단계 흐름:
//   points[0] = 접근 목표 (approach). 로봇이 여기까지 이동 후 대기.
//   points[1..] = 그 다음 이동/도색 구간. START_DRAW를 받으면 여기로 진행.
// 여기 값은 mm로 적는다 (CCTV와 같은 단위). 서버/로봇으로 나갈 땐 도구가 미터로
// 환산(÷1000)해 보낸다 - 로봇 전송 단위는 m 유지.
static const json kTargetPointsMm = json::array({{60.0, 60.0}, {300.0, 60.0}});
static constexpr double kMmToM = 0.001;

// mm 좌표 목록 -> 미터 좌표 목록 (BLUEPRINT 전송용)
static json targetPointsInMeters() {
    json pts = json::array();
    for (const auto& p : kTargetPointsMm)
        pts.push_back({p[0].get<double>() * kMmToM, p[1].get<double>() * kMmToM});
    return pts;
}

static void logf(const char* tag, const char* fmt, ...) {
    static std::mutex m;
    std::lock_guard<std::mutex> lk(m);
    char ts[16];
    time_t t = time(nullptr);
    struct tm tmv;
    localtime_r(&t, &tmv);
    strftime(ts, sizeof(ts), "%H:%M:%S", &tmv);
    fprintf(stderr, "%s [%s] ", ts, tag);
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fprintf(stderr, "\n");
    fflush(stderr);
}

static std::atomic<long> g_seq{0};

static json makeMsg(const std::string& type, const json& payload) {
    return json{{"type", type}, {"seq", ++g_seq}, {"payload", payload}};
}

// ── 스냅샷 로더 (CCTV 자료 -> 서버 입력. CCTV 포맷 의존은 여기 한 곳에만) ────────
// 실제 CCTV 출력 포맷이 바뀌면 이 함수만 고치면 나머지는 그대로 동작한다.
// 출력: calib = 서버 H_MATRIX용 번들(mm 원본 그대로), corners = POS용 4코너(원본 픽셀).
// ※ mm -> m 환산은 이제 서버가 담당한다(입구에서 normalizeBundleMmToM). 도구는 CCTV가
//   준 mm 호모그래피를 손대지 않고 그대로 올린다 - 여기서 또 환산하면 이중 스케일이 된다.
static bool loadSnapshot(const std::string& path, json& calib, json& corners) {
    std::ifstream f(path);
    if (!f) {
        logf("TEST", "스냅샷 파일 열기 실패: %s", path.c_str());
        return false;
    }
    json snap = json::parse(f, nullptr, false);
    if (snap.is_discarded() || !snap.is_object()) {
        logf("TEST", "스냅샷 JSON 파싱 실패: %s", path.c_str());
        return false;
    }
    if (!snap.contains("corners") || !snap["corners"].is_array() ||
        snap["corners"].size() != 4) {
        logf("TEST", "스냅샷에 'corners'(네 꼭짓점)가 없거나 4개가 아님");
        return false;
    }
    corners = snap["corners"];

    if (snap.contains("calib")) {  // 이미 번들 형태면 그대로 사용
        calib = snap["calib"];
        return true;
    }
    if (!snap.contains("H") || !snap["H"].is_array() || snap["H"].size() != 3) {
        logf("TEST", "스냅샷에 'H'(3x3 호모그래피) 또는 'calib' 번들이 필요함");
        return false;
    }
    // K/D 없음 = undistort 생략(CCTV가 미적용). H_floor/H_marker 동일 행렬(mm 원본) 사용.
    json Hm = snap["H"];
    calib = {{"version", 1}, {"H_floor", Hm}, {"H_marker", Hm},
             {"marker_height_m", snap.value("marker_height_m", 0.0)}};
    logf("TEST", "호모그래피(mm 원본) 그대로 전송 - mm->m 환산은 서버가 수행");
    return true;
}

// ── TLS 연결 하나를 표현 (송신 직렬화 + 수신 로깅 스레드) ─────────────────────
struct Conn {
    std::string tag;
    int fd = -1;
    SSL* ssl = nullptr;
    std::mutex wmtx;
    std::atomic<bool> alive{true};
    std::thread rx;

    bool send(const json& msg) {
        std::string data = msg.dump() + "\n";
        std::lock_guard<std::mutex> lk(wmtx);
        return SSL_write(ssl, data.data(), (int)data.size()) > 0;
    }
    void close_() {
        alive = false;
        if (fd >= 0) shutdown(fd, SHUT_RDWR);  // 수신 스레드의 SSL_read 깨우기
        if (rx.joinable()) rx.join();
        if (ssl) { SSL_shutdown(ssl); SSL_free(ssl); }
        if (fd >= 0) ::close(fd);
    }
};

static void recvLoop(Conn* c) {
    std::string buf, line;
    for (;;) {
        auto pos = buf.find('\n');
        if (pos == std::string::npos) {
            char tmp[4096];
            int n = SSL_read(c->ssl, tmp, sizeof(tmp));
            if (n <= 0) break;
            buf.append(tmp, (size_t)n);
            continue;
        }
        line = buf.substr(0, pos);
        buf.erase(0, pos + 1);
        if (line.empty()) continue;
        json msg = json::parse(line, nullptr, false);
        if (msg.is_discarded()) continue;
        logf(c->tag.c_str(), "수신 [%s] %s", msg.value("type", "?").c_str(),
             msg.value("payload", json::object()).dump().c_str());
    }
    if (c->alive) logf(c->tag.c_str(), "서버 연결 종료됨");
    c->alive = false;
}

// 서버에 TLS 접속하고 HELLO(role) 전송 후 수신 스레드 시작
static bool openConn(Conn& c, SSL_CTX* ctx, const std::string& ip, int port,
                     const std::string& role) {
    c.tag = role;
    c.fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &addr.sin_addr);
    if (connect(c.fd, (sockaddr*)&addr, sizeof(addr)) != 0) {
        logf(role.c_str(), "서버 접속 실패 (%s:%d)", ip.c_str(), port);
        return false;
    }
    c.ssl = SSL_new(ctx);
    SSL_set_fd(c.ssl, c.fd);
    if (SSL_connect(c.ssl) != 1) {
        logf(role.c_str(), "TLS 핸드셰이크 실패 (server.crt 확인)");
        ERR_print_errors_fp(stderr);
        return false;
    }
    logf(role.c_str(), "TLS 접속 성공 %s:%d", ip.c_str(), port);
    c.rx = std::thread(recvLoop, &c);
    c.send(makeMsg("HELLO", {{"role", role}}));
    return true;
}

static void pause_ms(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main(int argc, char** argv) {
    std::string ip = argc > 1 ? argv[1] : "127.0.0.1";
    std::string caFile = argc > 2 ? argv[2] : "../certs/server.crt";
    std::string snapPath = argc > 3 ? argv[3] : "tools/sample_snapshot.json";
    const int port = 9000;

    json calib, corners;
    if (!loadSnapshot(snapPath, calib, corners)) return 1;
    logf("TEST", "스냅샷 로드 완료: %s (corners=%s)", snapPath.c_str(),
         corners.dump().c_str());

    SSL_CTX* ctx = SSL_CTX_new(TLS_client_method());
    SSL_CTX_set_min_proto_version(ctx, TLS1_2_VERSION);
    if (SSL_CTX_load_verify_locations(ctx, caFile.c_str(), nullptr) != 1) {
        logf("TEST", "server.crt 로드 실패: %s (서버에서 복사해왔는지 확인)",
             caFile.c_str());
        return 1;
    }
    SSL_CTX_set_verify(ctx, SSL_VERIFY_PEER, nullptr);

    // 1) QT 역할 - 서버가 보내는 POSE/DRAW_FAIL 등을 여기서 관찰한다.
    Conn qt;
    if (!openConn(qt, ctx, ip, port, "QT")) return 1;
    pause_ms(300);  // ACK/PEERS 수신 여유

    // 2) CCTV 역할 - 호모그래피와 마커 위치를 주입한다.
    Conn cctv;
    if (!openConn(cctv, ctx, ip, port, "CCTV")) { qt.close_(); return 1; }
    pause_ms(300);

    // 3) 호모그래피 번들 주입 (POS보다 반드시 먼저 - calib이 없으면 pose 계산 불가)
    cctv.send(makeMsg("H_MATRIX", {{"calib", calib}}));
    logf("TEST", "H_MATRIX(calib) 전송");
    pause_ms(300);

    // 4) 로봇 시작 위치(네 꼭짓점) 주입 -> 서버가 pose 계산 후 QT에 POSE를 보내온다.
    cctv.send(makeMsg("POS", {{"corners", corners}}));
    logf("TEST", "POS(corners) 전송 - QT 쪽 'POSE' 수신 로그를 확인하세요");
    pause_ms(500);

    // 5) 목표 도면 주입 -> 서버가 1단계 approach PATH(points[0]까지)를 로봇에 전송.
    //    mm로 적힌 목표를 미터로 환산해 보낸다 (로봇 전송 단위 m 유지).
    json targetM = targetPointsInMeters();
    qt.send(makeMsg("BLUEPRINT", {{"points", targetM}}));
    logf("TEST", "BLUEPRINT 전송 - 목표 %s mm (= %s m) 접근 PATH 생성",
         kTargetPointsMm.dump().c_str(), targetM.dump().c_str());

    // 결과 관찰: PATH는 로봇에게만 가므로 여기선 안 보인다. 로봇 RPi 콘솔의
    // 'PATH received'와 서버 콘솔의 '[INFO] 1단계 접근 경로 전송'을 확인할 것.
    // 로봇 미접속이면 서버가 QT에 DRAW_FAIL(robot_offline)을 보내와 여기 찍힌다.
    pause_ms(1000);

    // 6) 2단계: 로봇이 접근점(points[0])에 도착해 대기하면, START_DRAW로 다음 구간
    //    (points[1..])을 진행시킨다. 실시간 피드백이 없어 도착 시점을 서버가 모르므로,
    //    사람이 눈으로 도착을 확인하고 Enter를 눌러 트리거한다 (Qt "그림그리기 시작" 대역).
    //    ※ 로봇이 아직 이동 중일 때 START_DRAW를 보내면 서버가 도색 PATH를 즉시 보내
    //      로봇이 접근을 버리고 그 자리에서 새 경로를 시작하니, 반드시 도착 후에 누를 것.
    logf("TEST", "로봇이 접근점(60,60)에 '도착해 멈춘' 뒤 Enter -> (300,60)으로 진행"
                 " (건너뛰고 종료하려면 Ctrl+C)");
    std::string dummy;
    std::getline(std::cin, dummy);

    // 접근 완료로 간주하고 "도착 pose"를 POS로 주입해 서버의 pose_를 접근점으로 갱신한다.
    // 실시간 CCTV가 없으면 서버는 로봇이 아직 스냅샷 위치에 있는 줄 알아 2단계 경로를
    // 엉뚱하게 계산한다. 도착 위치 = points[0], 도착 방향 = points[0]->points[1] 방향
    // (approach 마지막 TURN이 이 방향으로 정렬시켜 둠). {x,y,theta_deg}는 테스트용 POS 포맷.
    double ax = targetM[0][0].get<double>(), ay = targetM[0][1].get<double>();
    double adeg = std::atan2(targetM[1][1].get<double>() - ay,
                             targetM[1][0].get<double>() - ax) * 180.0 / M_PI;
    cctv.send(makeMsg("POS", {{"x", ax}, {"y", ay}, {"theta_deg", adeg}}));
    logf("TEST", "접근 완료 가정 -> POS(x=%.3f, y=%.3f, theta=%.1f°) 주입 (서버 pose 갱신)",
         ax, ay, adeg);
    pause_ms(300);

    qt.send(makeMsg("CMD", {{"cmd", "START_DRAW"}}));
    logf("TEST", "START_DRAW 전송 - 서버가 도색 PATH(-> 300,60)를 로봇에 전송");
    pause_ms(3000);

    logf("TEST", "종료");
    cctv.close_();
    qt.close_();
    SSL_CTX_free(ctx);
    return 0;
}
