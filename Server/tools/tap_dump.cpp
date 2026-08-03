// TAP 원문 뷰어 (ADMIN role) - CMD 창에 서버가 중계하는 메시지를 잘리지 않고 통째로 찍는다.
//
// 왜 필요한가: 관리자 창(admin_console)의 대시보드 로그는 메시지 하나당 200자로
//   잘라서 보여준다(rp_core.py _fmt_tap). BLUEPRINT처럼 program이 긴 메시지는
//   화면에서도, gui.log 파일에서도 잘린 채로만 남는다 - 한번 잘리면 그 메시지는
//   영영 복구 불가능하다. 이 도구는 admin_console과 완전히 별개의 프로세스로
//   붙어서 같은 TAP을 받되, 자르지 않고 그대로 stdout에 찍는다.
//
// ⚠️ admin_console 코드는 건드리지 않는다 - 다른 세션이 작업 중이라 손대면 안 됨.
//   이 도구는 새 파일이고, ADMIN role로 별도 접속하는 것뿐이라 기존 admin 세션과
//   충돌하지 않는다(TlsServer는 role당 세션 여러 개를 다 받아준다 - CCTV/ROBOT/QT만
//   재접속 시 교체되고 ADMIN은 그런 제약이 없다).
//
// 빌드: make tap_dump   (Server/ 디렉토리에서)
// 사용: ./tap_dump <서버IP> [server.crt경로] [옵션...]   (기본 crt: certs/server.crt)
//   --type <TYPE>    이 type만 보기 (예: --type BLUEPRINT). 여러 번 줄 수 있음
//   --peer <ROLE>    이 peer(QT/ROBOT/CCTV)만 보기. 여러 번 줄 수 있음
//   --all            STATUS/POS/POSE/DRIFT 같은 고빈도 메시지도 다 보기 (기본은 숨김)
//   --pretty         JSON을 여러 줄로 예쁘게 출력 (기본은 한 줄)
//   --port <n>       서버 포트 (기본 9000)
#include "tls_client.hpp"
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <set>
#include <string>

static std::atomic<bool> g_run{true};
static void onSig(int) { g_run = false; }

int main(int argc, char** argv) {
    signal(SIGINT, onSig);
    std::string ip = argc > 1 ? argv[1] : "127.0.0.1";
    std::string crt = "certs/server.crt";
    int port = 9000;
    bool showAll = false, pretty = false;
    std::set<std::string> typeFilter, peerFilter;

    int ai = 2;
    if (argc > 2 && argv[2][0] != '-') crt = argv[ai++];
    for (; ai < argc; ++ai) {
        std::string a = argv[ai];
        if (a == "--type" && ai + 1 < argc) typeFilter.insert(argv[++ai]);
        else if (a == "--peer" && ai + 1 < argc) peerFilter.insert(argv[++ai]);
        else if (a == "--all") showAll = true;
        else if (a == "--pretty") pretty = true;
        else if (a == "--port" && ai + 1 < argc) port = atoi(argv[++ai]);
        else { fprintf(stderr, "알 수 없는 옵션: %s\n", a.c_str()); return 1; }
    }

    // 기본으로 숨기는 고빈도 메시지 - --all 이거나 --type으로 명시하면 보인다.
    static const std::set<std::string> kNoisy = {"STATUS", "POS", "POSE", "DRIFT"};

    TlsLink admin("TAP");
    if (!admin.connect(ip, port, crt, "ADMIN")) return 1;
    tlogf("TAP", "대기 중 (Ctrl+C 종료)%s%s", showAll ? "" : " - STATUS/POS/POSE/DRIFT 숨김(--all로 해제)",
          typeFilter.empty() ? "" : " - type 필터 적용됨");

    json msg;
    while (g_run && admin.recv(msg)) {
        if (msg.value("type", "") != "TAP") continue;
        json p = msg.value("payload", json::object());
        json inner = p.value("msg", json::object());
        std::string t = inner.value("type", "");
        std::string peer = p.value("peer", "");

        if (!typeFilter.empty() && !typeFilter.count(t)) continue;
        if (!peerFilter.empty() && !peerFilter.count(peer)) continue;
        if (typeFilter.empty() && !showAll && kNoisy.count(t)) continue;

        char ts[16];
        time_t now = time(nullptr);
        struct tm tmv;
        localtime_r(&now, &tmv);
        strftime(ts, sizeof(ts), "%H:%M:%S", &tmv);

        std::string arrow = p.value("dir", "") == "IN" ? (peer + "->SRV") : ("SRV->" + peer);
        json payload = inner.value("payload", json::object());
        printf("%s [%s] %-10s %s\n", ts, arrow.c_str(), t.c_str(),
               payload.dump(pretty ? 2 : -1).c_str());
        fflush(stdout);
    }
    tlogf("TAP", "연결 종료");
    return 0;
}
