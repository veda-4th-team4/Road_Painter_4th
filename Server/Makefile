# Road-Painter 서버 빌드
# 필요 패키지: sudo apt install g++ make libssl-dev
# (nlohmann/json 은 include/ 에 포함되어 있어서 설치 불필요)
CXX      ?= g++
CXXFLAGS += -std=c++17 -O2 -Wall -Iinclude -Isrc -pthread
LDLIBS   += -lssl -lcrypto -pthread

SRCS = src/main.cpp src/tls_server.cpp src/router.cpp src/user_store.cpp

server: $(SRCS) src/*.hpp
	$(CXX) $(CXXFLAGS) $(SRCS) -o $@ $(LDLIBS)

# 테스트용 시뮬레이터 (실제 로봇/Qt 코드 나오기 전까지 서버 검증용)
qt_sim: tools/qt_sim.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDLIBS)

# 최초 1회 경로생성 테스트기 (CCTV 스냅샷 주입 -> (0,0) approach PATH 생성 검증)
path_test: tools/path_test.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDLIBS)

# 서버 알고리즘 드라이런 한 쌍 (docs/DRIVE_TEST_PLAN.md 단계 C).
#   robot_sim : ROBOT+CCTV 대역. PATH를 실행하고 그 결과 위치를 POS로 되돌린다
#   draw_test : QT 대역. 사각형 도면 + 동작 시퀀스를 만들어 START_DRAW까지
# 둘을 같이 띄우면 로봇도 Qt도 없이 접근->도색->피드백->완료가 전부 돈다.
robot_sim: tools/robot_sim.cpp tools/tls_client.hpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDLIBS)

draw_test: tools/draw_test.cpp tools/tls_client.hpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDLIBS)

sim: robot_sim draw_test

# 로봇 주행 단독 테스트기 (docs/DRIVE_TEST_PLAN.md 단계 A-3).
# ADMIN role로 PATH를 로봇에 직접 중계 - 접근·CCTV·도면 없이 제자리 사각형만 그린다.
drive_test: tools/drive_test.cpp tools/tls_client.hpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDLIBS)

clean:
	rm -f server qt_sim path_test robot_sim draw_test drive_test
.PHONY: clean sim
