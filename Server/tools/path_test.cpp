// 최초 1회 경로생성 테스트기 (서버 RPi <-> 로봇 RPi 검증용) - TCP/TLS
//
// 목적:
//   CCTV가 딱 한 번 만들어 폴더에 넣어둔 자료(호모그래피 + 로봇 시작 네 꼭짓점)를
//   읽어서, 서버에 CCTV+QT 역할로 흘려넣는다. 그러면 서버의 "기존 경로생성 로직"이
//   그대로 돌아 로봇이 "지금 서 있는 그 자리"에서 한 변 10cm짜리 정사각형을 그리는
//   PATH(TURN/MOVE 시퀀스)를 로봇에 전송한다. 접근(approach) 단계는 의도적으로
//   건너뛴다: BLUEPRINT의 시작점(points[0])을 로봇의 현재 pose와 동일하게 잡아,
//   서버가 "이미 시작점에 서 있음"으로 판단해 곧바로 도색 PATH만 보내게 한다.
//   실시간 피드백(DRIFT/이탈 재계획)은 자연히 빠진다(POS를 한 번만 주입하므로).
//
//   ※ 이 도구는 좌표/각도 계산을 전혀 하지 않는다. undistort->H_marker->pose 계산과
//     TURN/MOVE 시퀀스 생성은 전부 서버(src/path_planner.hpp, src/router.cpp)가 한다.
//     다만 "지금 서 있는 자리"를 시작점으로 잡으려면 도구도 서버와 같은 방식으로
//     스냅샷(H+corners)에서 pose를 미리 한 번 계산해야 한다(poseFromSnapshot 참고,
//     서버의 poseFromPos와 동일한 공식) - BLUEPRINT 좌표를 만들기 위해서일 뿐, 서버가
//     실제로 쓰는 값은 여전히 서버가 CCTV POS로 자체 계산한 pose_다.
//
// 흐름 (2026-07-27 변경: 접근 생략 + 제자리 정사각형):
//   1) QT 역할로 접속 + HELLO           (POSE/DRAW_DONE/DRAW_FAIL 등 서버 응답 관찰)
//   2) CCTV 역할로 접속 + HELLO
//   3) CCTV -> H_MATRIX(calib)          (서버가 호모그래피 번들 적재)
//   4) CCTV -> POS(corners)             (서버가 pose 계산 -> QT에 POSE 전송)
//      (도구도 같은 H+corners로 로컬 pose를 계산해 정사각형 좌표를 만든다)
//   5) QT   -> BLUEPRINT(제자리 사각형)  (서버는 저장만 함 - 아직 로봇 안 움직임)
//   6) QT   -> CMD{START_DRAW}          (points[0]==현재 pose이므로 서버가 접근을
//      건너뛰고 곧바로 도색 PATH(10cm 이동 + 90도 회전 x4)를 ROBOT에 전송)
//   -> 결과는 로봇 RPi 콘솔(PATH 수신)과 서버 콘솔([INFO] 도색 경로 전송,
//      도색 완료)에서 확인. QT 대역(이 도구)에는 최종적으로 DRAW_DONE이 온다.
//
// ⚠️ 알려진 한계: 실제 CCTV처럼 지속적으로 POS를 보내는 게 아니라 딱 1회만 주입하므로,
//   실시간 이탈 재계획은 검증되지 않는다. 접근을 생략하고 곧장 도색 PATH가 나가는지,
//   그리고 사각형 좌표/각도 계산(TURN/MOVE 시퀀스)이 맞는지 확인하는 용도다.
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

// ── 목표 도면: 로봇이 "지금 서 있는 자리"에서 접근 없이 곧바로 그리는 정사각형 ──
// points[0] = 로봇 현재 pose(그대로 approach 목표로 넣어 이동거리 0으로 만듦
//   -> 서버가 접근 단계를 건너뛰고 곧장 도색 PATH를 전송).
// points[1..4] = 현재 heading 기준 10cm 전진 + 90도 좌회전을 3번 반복해 사각형을
//   닫는다 (TURN 90 + MOVE 0.1m 세그먼트 x4, 서버 buildSegments가 계산).
// 서버 내부 단위는 미터이므로 좌표는 처음부터 미터로 만든다.
static constexpr double kMmToM = 0.001;
static constexpr double kSquareSideM = 0.10;   // 10cm
static constexpr double kTurnDeg = 90.0;       // 코너마다 좌회전 90도

struct PoseM { double x = 0, y = 0, theta = 0; };  // 미터, 라디안

// H(3x3, mm 기준 raw) 픽셀 -> 바닥 mm 좌표. 서버 calib.hpp의 applyH와 동일한
// 동차좌표 계산이나, mm->m 스케일을 서버처럼 행렬에 미리 곱해두지 않고 결과를
// 마지막에 ÷1000 한다(수학적으로 동일 - scaleMat3Rows01은 0,1행 전체를 스케일해도
// 결과는 그대로 s배가 되므로).
static std::array<double, 2> applyHmm(const json& H, double u, double v) {
    double r0 = H[0][0].get<double>() * u + H[0][1].get<double>() * v + H[0][2].get<double>();
    double r1 = H[1][0].get<double>() * u + H[1][1].get<double>() * v + H[1][2].get<double>();
    double r2 = H[2][0].get<double>() * u + H[2][1].get<double>() * v + H[2][2].get<double>();
    return {r0 / r2, r1 / r2};
}

// calib 번들 + corners(원본 픽셀) -> 로봇 현재 pose(미터). 서버 path_planner.hpp의
// poseFromPos와 동일한 공식(중심 = 4코너 평균, heading = 앞변 중점 -> 뒷변 중점).
// undistort는 생략(이 도구가 다루는 스냅샷은 K/D 없음 전제 - loadSnapshot 참고).
static PoseM poseFromSnapshot(const json& calib, const json& corners) {
    json H = calib.contains("H_marker") ? calib["H_marker"] : calib;
    std::array<double, 2> c[4];
    for (int i = 0; i < 4; ++i) {
        auto mm = applyHmm(H, corners[i][0].get<double>(), corners[i][1].get<double>());
        c[i] = {mm[0] * kMmToM, mm[1] * kMmToM};
    }
    PoseM p;
    p.x = (c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4;
    p.y = (c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4;
    double fx = (c[0][0] + c[1][0]) / 2, fy = (c[0][1] + c[1][1]) / 2;
    double bx = (c[2][0] + c[3][0]) / 2, by = (c[2][1] + c[3][1]) / 2;
    p.theta = std::atan2(fy - by, fx - bx);
    return p;
}

// 현재 pose에서 시작해 10cm 전진 -> 90도 좌회전을 4번 반복하는 제자리 정사각형
// BLUEPRINT(미터). pts[0]==pose이므로 approach 이동거리가 0이 되어 서버가 접근을
// 생략하고(있는 그대로) 곧장 도색 PATH를 전송한다.
static json squareBlueprint(const PoseM& pose) {
    json pts = json::array();
    double x = pose.x, y = pose.y, th = pose.theta;
    pts.push_back({x, y});
    for (int i = 0; i < 4; ++i) {
        x += kSquareSideM * std::cos(th);
        y += kSquareSideM * std::sin(th);
        pts.push_back({x, y});
        th += kTurnDeg * M_PI / 180.0;
    }
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

    // 서버가 CCTV POS로 계산할 pose를 도구도 미리 계산해, "지금 서 있는 자리"를
    // 시작점 삼는 정사각형 BLUEPRINT를 만든다 (squareBlueprint 주석 참고).
    PoseM startPose = poseFromSnapshot(calib, corners);
    json blueprintM = squareBlueprint(startPose);
    logf("TEST", "시작 pose 추정: x=%.3f y=%.3f theta=%.1f도 - 이 자리에서 10cm"
                 " 정사각형 BLUEPRINT %s",
         startPose.x, startPose.y, startPose.theta * 180.0 / M_PI,
         blueprintM.dump().c_str());

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

    // 5) 목표 도면 주입 -> 서버는 저장만 한다 (points[0]==현재 pose).
    qt.send(makeMsg("BLUEPRINT", {{"points", blueprintM}}));
    logf("TEST", "BLUEPRINT 전송 - 제자리 정사각형 %s m, 저장만 됨 (아직 PATH 없음)",
         blueprintM.dump().c_str());
    pause_ms(300);

    // 6) "그림그리기 시작" (Qt 버튼 대역) -> points[0]이 현재 pose와 같아 서버가
    //    접근(approach) 단계를 건너뛰고 곧바로 도색 PATH(10cm 이동 + 90도 회전 x4)를
    //    ROBOT에 전송한다.
    qt.send(makeMsg("CMD", {{"cmd", "START_DRAW"}}));
    logf("TEST", "CMD START_DRAW 전송 - 서버가 접근을 생략하고 곧장 도색 PATH를 전송");

    // 결과 관찰: PATH는 로봇에게만 가므로 여기선 안 보인다. 로봇 RPi 콘솔의
    // 'PATH received'와 서버 콘솔의 '[INFO] 접근 불필요 ... 곧바로 도색 경로 전송' /
    // '[INFO] 2단계 도색 경로 전송'을 확인할 것. 로봇 미접속/도면 없음/위치 미확인이면
    // 서버가 QT에 DRAW_FAIL을 보내와 여기 찍힌다.
    pause_ms(1000);

    // 7) 도색 완료(DRAW_DONE) 또는 실패(DRAW_FAIL)를 기다린다. 실제 로봇이 도색을
    //    끝내고 PATH_DONE{draw}를 보내야 도착하므로 넉넉히 기다린다.
    logf("TEST", "DRAW_DONE/DRAW_FAIL 대기 중 (최대 10초)...");
    pause_ms(10000);

    logf("TEST", "종료");
    cctv.close_();
    qt.close_();
    SSL_CTX_free(ctx);
    return 0;
}
