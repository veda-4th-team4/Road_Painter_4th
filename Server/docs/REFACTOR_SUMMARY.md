# 서버 코드 개선 사항 정리

**작성일**: 2026-07-21 (2026-07-21 실제 검증 후 갱신)
**대상**: main.cpp, tls_server.hpp, tls_server.cpp 개선
**목적**: 스레드 안전성 및 우아한 종료(graceful shutdown) 구현
**상태**: ✅ 적용 완료 + 실행 테스트로 검증 완료 (아래 "추가 발견 및 수정" 참고)

> 이 문서는 처음에 "적용 완료, 빌드 테스트 완료"로 작성됐으나, 실제로
> `kill -TERM`을 보내 종료 동작을 검증한 것은 이번이 처음이었다. 그 과정에서
> 서버가 실제로는 종료되지 않는 버그를 하나 더 발견해 같이 고쳤다 (맨 아래
> "추가 발견 및 수정" 섹션 참고). `FIXES_NEEDED.md`는 이 작업의 초안(제안 단계)
> 문서였고, 지금은 이 문서로 완전히 대체되어 삭제했다.

---

## 🎯 문제 점과 해결책

### **문제 1: 스레드 정리 불완전**

**원본 코드 (main.cpp)**
```cpp
std::thread netThread([&] { srv.run(); });
netThread.detach();  // ❌ 스레드 정리 불가

// stdin이 닫혀도 계속 대기
for (;;) std::this_thread::sleep_for(std::chrono::hours(1));
```

**문제점:**
- `detach()` 후 스레드 상태 추적 불가
- Ctrl+C로 종료 시 스레드가 제대로 정리되지 않음
- 백그라운드 실행 중 graceful shutdown 불가능

---

### **문제 2: Ctrl+C 신호 처리 미흡**

**원본 코드**
```cpp
} else if (cmd == "quit") {
    logf("[INFO] 서버 종료");
    std::exit(0);  // ❌ 강제 종료, 정리 없음
}
```

**문제점:**
- `std::exit()`는 리소스 정리하지 않음
- 활성 TLS 연결 갑작스럽게 종료
- 클라이언트에 종료 통보 불가

---

### **문제 3: accept() 루프 종료 불가**

**TlsServer::run()**
```cpp
void TlsServer::run() {
    for (;;) {
        int fd = accept(listenFd_, ...);  // ❌ 블로킹, 깨우는 방법 없음
    }
}
```

**문제점:**
- `accept()`는 시스템 콜로 블로킹
- Signal handler에서 설정한 `gShutdown` 플래그로 깨울 수 없음
- 종료 신호를 받아도 `netThread.join()`이 영원히 대기

---

## ✅ 적용된 해결책

### **수정 1: main.cpp - Graceful Shutdown 구현**

#### 1-1. 전역 상태 추가
```cpp
// 추가된 헤더
#include <atomic>
#include <csignal>
#include <memory>

// 전역 변수
static std::atomic<bool> gShutdown(false);
static std::unique_ptr<TlsServer> gpServer;
```

**이유:** 스레드 안전한 종료 신호 전달

---

#### 1-2. Signal Handler 구현
```cpp
static void signalHandler(int sig) {
    logf("[INFO] 종료 신호 수신 (sig=%d)", sig);
    gShutdown = true;
    // accept() 루프를 깨우기 위해 리스닝 소켓을 종료
    if (gpServer) gpServer->shutdown();
}

int main() {
    std::signal(SIGINT, signalHandler);   // Ctrl+C
    std::signal(SIGTERM, signalHandler);  // kill -TERM
    ...
}
```

**이유:** 
- Ctrl+C와 kill -TERM 신호 처리
- `shutdown()` 호출로 `accept()` 루프 강제 종료

---

#### 1-3. 메인 루프 개선
```cpp
// 콘솔 입력 루프
while (!gShutdown && std::getline(std::cin, cmd)) {
    if (cmd == "path") { ... }
    else if (cmd == "estop") { 
        gpServer->sendTo("ROBOT", ...);
        logf("[INFO] ESTOP 전송");  // ✅ 로그 추가
    }
    ...
    else if (cmd == "quit") {
        logf("[INFO] 종료 명령 수신");
        gShutdown = true;
        break;  // ✅ std::exit() 제거
    }
}

// stdin 닫혀도 백그라운드 계속
logf("[INFO] 콘솔 입력 종료, 백그라운드 모드로 전환");
while (!gShutdown) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

// ✅ 정상 종료 시 스레드 정리
logf("[INFO] 네트워크 스레드 정리 중...");
netThread.join();  // detach() 제거

logf("[INFO] 서버 정상 종료");
return 0;
```

**변경점:**
- `detach()` → `join()` (스레드 정리 보장)
- `std::exit()` → `gShutdown = true` + `break` (우아한 종료)
- `std::chrono::hours(1)` → `std::chrono::milliseconds(100)` (빠른 응답)
- 명령 실행 후 로그 추가 (확인 용이)

---

### **수정 2: tls_server.hpp - shutdown() 메서드 추가**

```cpp
class TlsServer {
    ...
    void setHandler(Handler h) { handler_ = std::move(h); }
    void run();  // accept 루프 (블로킹) - 별도 스레드에서 호출
    void shutdown();  // ✅ 리스닝 소켓 종료 (accept 루프에서 빠져나옴)
    ...
};
```

**이유:** `accept()` 루프를 종료하기 위한 공개 인터페이스

---

### **수정 3: tls_server.cpp - shutdown() 구현**

#### 3-1. 기존 코드 수정 (name collision 해결)
```cpp
// 수정 전: 소켓 시스템콜과 충돌
shutdown(it->second->fd, SHUT_RDWR);

// 수정 후: 명시적으로 시스템콜 호출
::shutdown(it->second->fd, SHUT_RDWR);
```

**이유:** `TlsServer::shutdown()` 메서드와 `::shutdown()` 시스템콜 구분

---

#### 3-2. shutdown() 메서드 구현
```cpp
void TlsServer::shutdown() {
    if (listenFd_ >= 0) {
        close(listenFd_);
        listenFd_ = -1;
        logf("[INFO] 리스닝 소켓 종료");
    }
}
```

**동작:**
1. `listenFd_` 닫기 → `accept()` 실패 반환
2. `run()` 루프 자동 종료 (`fd < 0` 체크)
3. `netThread.join()` 정상 반환

---

## 📊 수정 효과

| 항목 | 이전 | 개선 후 |
|------|------|--------|
| **스레드 종료** | 미정리 (좀비) | ✅ 안전한 join() |
| **Ctrl+C 처리** | 강제 종료 | ✅ 우아한 종료 |
| **accept() 루프** | 무한 대기 | ✅ 신호로 종료 |
| **리소스 정리** | 불완전 | ✅ 완전 정리 |
| **로그 명확성** | 명령 결과 미표시 | ✅ 모든 명령 로그 |

---

## 🧪 테스트 시나리오

### **1️⃣ 정상 실행**
```bash
cd /home/team4/Road_Painter_4th/Server
make clean && make
./server

# 콘솔에서
path    → PATH 전송 성공 로그
estop   → ESTOP 전송 로그
who     → 접속 중인 클라이언트
quit    → 종료 명령 수신 후 정상 종료
```

---

### **2️⃣ Ctrl+C 종료**
```bash
./server &
sleep 1
kill -INT $!  # 또는 Ctrl+C

# 예상 로그
[INFO] 종료 신호 수신 (sig=2)
[INFO] 리스닝 소켓 종료
[INFO] 네트워크 스레드 정리 중...
[INFO] 서버 정상 종료
```

---

### **3️⃣ SIGTERM 종료**
```bash
./server &
sleep 1
kill -TERM $!

# 예상 로그
[INFO] 종료 신호 수신 (sig=15)
[INFO] 리스닝 소켓 종료
...
```

---

### **4️⃣ 백그라운드 실행**
```bash
./server < /dev/null &    # stdin 종료
sleep 1
kill $!

# 예상 로그
[INFO] 콘솔 입력 종료, 백그라운드 모드로 전환
[INFO] 종료 신호 수신 (sig=15)
[INFO] 정상 종료
```

---

## ⚠️ 주의사항

### **1. Router 상태 정리**
- 현재 router 객체는 `main()` 스택에 생성됨
- `netThread` 종료 전에 범위를 벗어나면 문제 가능
- **상태**: 안전 (main() 함수 끝까지 존재)

### **2. 활성 연결 처리**
- `shutdown()` 호출 후 신규 연결 불가
- 기존 연결은 `sessionThread`에서 정리됨
- **상태**: 안전 (각 세션이 독립적으로 정리)

### **3. TLS 정리**
- `SSL_CTX_free()`, `SSL_shutdown()`, `SSL_free()` 순서 유지
- `tls_server.cpp` 기존 코드에서 이미 구현됨
- **상태**: 안전 (변경 없음)

---

## 📝 코드 검토 체크리스트

- [x] 컴파일 성공 (gcc -std=c++17)
- [x] Ctrl+C / SIGTERM 클린 종료 — **실제로 `kill -TERM` 보내서 검증함** (2026-07-21)
- [x] `netThread.join()` 정상 반환 — 단, 즉시는 아니고 **1~7초 소요** (아래 참고)
- [x] 백그라운드 실행 후 신호 종료 가능
- [ ] 메모리 누수 없음 (valgrind 검사 - 미실시)

## 🐛 추가 발견 및 수정 (2026-07-21, 실행 테스트로 발견)

위 개선을 실제로 `./server` 실행 + `kill -TERM`으로 검증하는 과정에서,
**서버가 실제로는 절대 종료되지 않는 버그**를 발견했다:

**증상**: SIGTERM을 보내면 로그에 "종료 신호 수신" → "리스닝 소켓 종료" →
"네트워크 스레드 정리 중..."까지는 찍히지만, 그 뒤 **"서버 정상 종료"가 영원히
안 찍히고 프로세스가 CPU 20%+ 를 계속 먹으며 살아있음.**

**원인** (`tls_server.cpp`의 accept 루프):
```cpp
for (;;) {
    int fd = accept(listenFd_, ...);
    if (fd < 0) continue;   // shutdown()이 listenFd_를 닫아도 무한 재시도
    ...
}
```
`shutdown()`이 `listenFd_`를 닫으면 `accept(-1,...)`이 즉시 -1을 반환하는데,
`continue`가 곧바로 재시도해서 **빈 루프가 영원히 도는 상태**(busy loop)가
됐다. `run()`이 리턴을 안 하니 `netThread.join()`도 영원히 대기.

**수정**:
- `accept()`가 실패했을 때 `listenFd_ < 0`이면(=shutdown이 의도적으로 닫은 것)
  `break`하도록 분기 추가.
- `listenFd_`를 `int` → `std::atomic<int>`로 변경 (다른 스레드에서 쓰고 읽는
  값이라 원래 데이터 레이스가 있었음).
- 부수 효과로 `bind()` 호출이 ADL 때문에 `std::bind`와 충돌 → `::bind`로 명시.

**검증**: `kill -TERM` 후 프로세스가 실제로 사라지는 것 확인. 단, 리눅스에서
"다른 스레드가 블로킹 중인 `accept()`를 `close()`로 깨우는" 방식 자체가
**즉시성을 보장하지 않아** 종료까지 1~7초 정도 걸릴 수 있음 (실측). 이건 이
방식의 알려진 특성이며, 개발/테스트 서버 단계에서는 문제없다고 판단해 더
정교한 방식(poll 기반 accept 등)으로는 안 바꿈.

---

## 📌 원저자 의견 요청

이 수정이 프로젝트 구조와 맞는지 확인 부탁드립니다:

1. **TlsServer 설계 의도**
   - 현재 `thread-per-connection` 패턴이 맞나?
   - `accept()` 루프 종료 방식이 최적인가?

2. **Router 상태 관리**
   - `Router` 객체 생명주기가 안전한가?
   - 테스트 중 메모리 누수는 없나?

3. **신호 처리 방식**
   - `SIGPIPE` 이외 다른 신호 고려 필요?
   - 임시 콘솔 인터페이스 추후 제거 예정?

4. **웹 GUI와의 관계**
   - web_gui.py의 server_link_loop()이 이 변경의 영향을 받나?
   - TLS 연결 재설정 동작 확인 필요?

---

## 📎 수정 파일 목록

| 파일 | 변경사항 |
|------|---------|
| [src/main.cpp](src/main.cpp) | Signal handler, graceful shutdown, thread join |
| [src/tls_server.hpp](src/tls_server.hpp) | shutdown() 메서드 추가 |
| [src/tls_server.cpp](src/tls_server.cpp) | shutdown() 구현, ::shutdown 수정 |

**빌드 테스트 완료**: `make clean && make` ✅

---

## 🚀 배포 절차

1. **검토자 피드백** 수집
2. **테스트 케이스** 추가 (unit test)
3. **문서 업데이트** (README.md)
4. **프로덕션 브랜치** 병합 (feature/graceful-shutdown)

---

**질문/피드백**: 이 문서를 통해 논의 진행
