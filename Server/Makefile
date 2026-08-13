# Road-Painter 서버 빌드
# 필요 패키지: sudo apt install g++ make libssl-dev
# (nlohmann/json 은 include/ 에 포함되어 있어서 설치 불필요)
CXX      ?= g++
CXXFLAGS += -std=c++17 -O2 -Wall -Iinclude -Isrc -pthread
LDLIBS   += -lssl -lcrypto -pthread

SRCS = src/main.cpp src/tls_server.cpp src/router.cpp src/router_channel.cpp \
       src/router_calib.cpp src/router_odocalib.cpp src/user_store.cpp

server: $(SRCS) src/*.hpp
	$(CXX) $(CXXFLAGS) $(SRCS) -o $@ $(LDLIBS)

# 디버깅/테스트/시뮬레이터 도구 - 소스도 바이너리도 전부 tools/ 밑에 모아둔다
# (server 실행파일만 운영용이라 예외로 루트에 남긴다)

# 테스트용 시뮬레이터 (실제 로봇/Qt 코드 나오기 전까지 서버 검증용)
qt_sim: tools/qt_sim.cpp
	$(CXX) $(CXXFLAGS) $< -o tools/$@ $(LDLIBS)

# 최초 1회 경로생성 테스트기 (CCTV 스냅샷 주입 -> (0,0) approach PATH 생성 검증)
path_test: tools/path_test.cpp
	$(CXX) $(CXXFLAGS) $< -o tools/$@ $(LDLIBS)

# 서버 알고리즘 드라이런 한 쌍 (주행 테스트 계획(노션 아카이브) 단계 C).
#   robot_sim : ROBOT+CCTV 대역. PATH를 실행하고 그 결과 위치를 POS로 되돌린다
#   draw_test : QT 대역. 사각형 도면 + 동작 시퀀스를 만들어 START_DRAW까지
# 둘을 같이 띄우면 로봇도 Qt도 없이 접근->도색->피드백->완료가 전부 돈다.
robot_sim: tools/robot_sim.cpp tools/tls_client.hpp
	$(CXX) $(CXXFLAGS) $< -o tools/$@ $(LDLIBS)

draw_test: tools/draw_test.cpp tools/tls_client.hpp
	$(CXX) $(CXXFLAGS) $< -o tools/$@ $(LDLIBS)

sim: robot_sim draw_test

# 로봇 주행 단독 테스트기 (주행 테스트 계획(노션 아카이브) 단계 A-3).
# ADMIN role로 PATH를 로봇에 직접 중계 - 접근·CCTV·도면 없이 제자리 사각형만 그린다.
drive_test: tools/drive_test.cpp tools/tls_client.hpp
	$(CXX) $(CXXFLAGS) $< -o tools/$@ $(LDLIBS)


# ADMIN으로 붙어 TAP 원문을 안 잘리게 CMD 창에 찍는다 (관리자 창 대시보드는 200자 제한).
tap_dump: tools/tap_dump.cpp tools/tls_client.hpp
	$(CXX) $(CXXFLAGS) $< -o tools/$@ $(LDLIBS)

# CCTV 대역 pose 주입기 - 로봇 시작 위치를 파일에서 읽어 POS로 한 번 쏜다.
# 실시간 CCTV가 아직 없을 때 접근->도색 흐름을 개루프로 돌려보는 용도.
cctv_pose: tools/cctv_pose.cpp tools/tls_client.hpp
	$(CXX) $(CXXFLAGS) $< -o tools/$@ $(LDLIBS)

# 채널별 캘리브레이션 저장 회귀 테스트 (프로토콜 v0.4).
# 현장에 이미 배포된 users.json/calib_latest.json 의 예전 형식이 채널 1로 계속
# 읽히는지, 채널 하나를 저장할 때 다른 채널이 날아가지 않는지 확인한다.
# 전부 "틀려도 에러가 안 나고 캘리만 조용히 사라지는" 종류라 테스트로 못박아 둔다.
calib_channel_test: tools/calib_channel_test.cpp src/user_store.cpp src/*.hpp
	$(CXX) $(CXXFLAGS) tools/calib_channel_test.cpp src/user_store.cpp -o tools/$@ $(LDLIBS)

# 오프셋 보정 다리(±a)에 MORE 피드백이 걸리는지 못박는 회귀 테스트.
# 이 다리가 보정에서 빠지면 에러 없이 꼭짓점만 조금씩 어긋난다 - 눈으로 못 잡는다.
offset_feedback_test: tools/offset_feedback_test.cpp src/*.hpp
	$(CXX) $(CXXFLAGS) $< -o tools/$@ $(LDLIBS)

# 캘리브레이션 번들 단위 규약 회귀 테스트 (2026-08-11 규격 §1/§4).
# "번들은 mm로 저장·중계, ÷1000은 서버 내부 사본에서만"을 못박는다. 단위가 1000배
# 어긋나도 에러가 안 나고 좌표만 조용히 틀리는 부류라, 눈으로 검수할 수가 없다.
calib_unit_test: tools/calib_unit_test.cpp src/*.hpp
	$(CXX) $(CXXFLAGS) $< -o tools/$@ $(LDLIBS)

# 로봇 주행 호모그래피 세션 회귀 테스트 (2026-08-10 계약 §6 시험표).
# QT/ROBOT/CCTV 세 role로 동시에 붙어 "시작한 세션은 반드시 종결 응답 하나로
# 닫힌다"를 못 박는다 - 응답 누락은 크래시가 아니라 조작자가 5분 갇히는 것으로만
# 드러나서, 테스트 없이는 현장에서야 알게 된다. 서버를 먼저 띄워두고 실행할 것.
calib_session_test: tools/calib_session_test.cpp tools/tls_client.hpp
	$(CXX) $(CXXFLAGS) $< -o tools/$@ $(LDLIBS)

# 오도메트리 주행 캘리 세션 회귀 테스트 (2026-08-13, Qt 개시 지원).
# calib_session_test(정적 앵커)의 짝. QT/ROBOT/CCTV/ADMIN 네 role로 붙어
# 주행 완주 -> 결과 대기 -> 종결까지 돌리고, 소유권/강제회수/판별자도 본다.
odo_session_test: tools/odo_session_test.cpp tools/tls_client.hpp
	$(CXX) $(CXXFLAGS) $< -o tools/$@ $(LDLIBS)

# 로봇 오도메트리 주행 캘리 회귀 테스트 (2026-08-12 신설).
# ops_builder.hpp의 순수 함수(buildCalibRectOps/odoReadyToPoint/odoPointWorldMm)만
# 검증한다 - 로봇 경로 구조와 CCTV로 보낼 좌표가 조용히 틀려도 눈에 안 띄는 부류라
# 못박아 둔다. Router/TLS는 필요 없다 (offset_feedback_test/calib_unit_test와 같은
# 오프라인 헤더 테스트 패턴).
odo_calib_test: tools/odo_calib_test.cpp src/*.hpp
	$(CXX) $(CXXFLAGS) $< -o tools/$@ $(LDLIBS)

clean:
	rm -f server tools/qt_sim tools/path_test tools/robot_sim tools/draw_test tools/drive_test tools/tap_dump tools/cctv_pose tools/calib_channel_test tools/calib_unit_test tools/offset_feedback_test tools/calib_session_test tools/odo_session_test tools/odo_calib_test
.PHONY: clean sim
