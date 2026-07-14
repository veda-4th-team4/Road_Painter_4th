# Road-Painter 서버 빌드
# 필요 패키지: sudo apt install g++ make libssl-dev
# (nlohmann/json 은 include/ 에 포함되어 있어서 설치 불필요)
CXX      ?= g++
CXXFLAGS += -std=c++17 -O2 -Wall -Iinclude -Isrc -pthread
LDLIBS   += -lssl -lcrypto -pthread

SRCS = src/main.cpp src/tls_server.cpp src/router.cpp src/user_store.cpp

server: $(SRCS) src/*.hpp
	$(CXX) $(CXXFLAGS) $(SRCS) -o $@ $(LDLIBS)

clean:
	rm -f server
.PHONY: clean
