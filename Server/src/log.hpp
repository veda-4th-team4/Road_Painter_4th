#pragma once
// 타임스탬프 붙은 간단 로그 (스레드 안전)
#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <functional>
#include <mutex>
#include <string>

// 선택적 로그 싱크: 등록되면 stderr 출력과 함께 각 로그 줄(타임스탬프 포함)을
// 이 콜백에도 넘긴다. 서버가 관리자 창(ADMIN)에 로그를 중계하는 데 사용
// (웹 로그 모니터가 서버 내부 로그도 보게). main.cpp에서 등록.
// ⚠️ 싱크 구현은 절대 logf()를 다시 부르면 안 된다 (무한 재귀).
inline std::function<void(const std::string&)>& logSink() {
    static std::function<void(const std::string&)> sink;
    return sink;
}

inline void logf(const char* fmt, ...) {
    char ts[16];
    time_t t = time(nullptr);
    struct tm tmv;
    localtime_r(&t, &tmv);
    strftime(ts, sizeof(ts), "%H:%M:%S", &tmv);
    char body[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(body, sizeof(body), fmt, ap);
    va_end(ap);
    std::string line = std::string(ts) + " " + body;
    {
        // stderr 출력만 락으로 보호 (여러 스레드의 줄 섞임 방지).
        static std::mutex m;
        std::lock_guard<std::mutex> lk(m);
        fprintf(stderr, "%s\n", line.c_str());
        fflush(stderr);
    }
    // 싱크는 락 밖에서 호출 - 싱크 내부(ADMIN 소켓 write)에서 문제가 나도
    // 위 stderr 락과 얽혀 데드락 나지 않도록.
    auto& sink = logSink();
    if (sink) sink(line);
}
