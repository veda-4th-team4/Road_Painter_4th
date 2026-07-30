#Auto generated, please dont edit. Editing might result in build failure or may render project unable to build.

SRC = src
SOURCES = $(shell find $(SRC) -name "*.cpp")
OPENSDK_ROOT = /opt/opensdk/

###################  DO NOT EDIT - BEGIN #########################
#version_begin
VERSION = 5.00
#version_end

#include_begin
INCL = $(OPENSDK_ROOT)/opensdk-$(VERSION)/common/inc/
#include_end

#pp5_begin
CXXFLAGS5 = -O2 -g -Wall -fmessage-length=0
#pp5_end

#pp6_begin
CXXFLAGS6 = -O2 -g -Wall -fmessage-length=0
#pp6_end

#pp7_begin
CXXFLAGS7 = -O2 -g -Wall -fmessage-length=0
#pp7_end

#pp8_begin
CXXFLAGS8 = -O2 -g -Wall -fmessage-length=0
#pp8_end

#pp9_begin
CXXFLAGS9 = -O2 -g -Wall -fmessage-length=0
#pp9_end

#pp10_begin
CXXFLAGS10 = -O2 -g -Wall -fmessage-length=0
#pp10_end

#lib5_begin
LIBS5 = -Llibs -lapphelper_wn5
#lib5_end

#lib6_begin
LIBS6 = -Llibs -lapphelper_s3l -lrt
#lib6_end

#lib7_begin
LIBS7 = -Llibs -lapphelper_cv2 -lrt
#lib7_end

#lib8_begin
LIBS8 = -Llibs -lapphelper_wn7 -lrt
#lib8_end

#lib9_begin
LIBS9 = -Llibs -lapphelper_cv2x -lrt
#lib9_end

#lib10_begin
LIBS10 = -Llibs -lapphelper_mt8137 -lrt
#lib10_end
###################  DO NOT EDIT - END  #########################

APP_HEADERS = $(shell find inc -name "*.h")
INC_APP = $(addprefix -I, $(sort $(dir $(APP_HEADERS))))
INC = $(addprefix -I, $(INCL))

# ---- OpenCV (statically cross-compiled for the target) ------------------
# Built by opencv_cross/build_opencv.sh into OPENCV_ROOT. No pkg-config needed:
# the module libs are listed explicitly in dependency order (most dependent
# first), then the bundled 3rdparty archives (zlib, etc.), then system libs.
OPENCV_ROOT ?= /mnt/third_party/opencv-aarch64

OPENCV_CFLAGS ?= -I$(OPENCV_ROOT)/include/opencv4

# OpenCV module archives, ordered so each lib appears before its dependencies.
OPENCV_MODULE_LIBS = -lopencv_aruco -lopencv_calib3d -lopencv_features2d \
                     -lopencv_flann -lopencv_imgproc -lopencv_core
# Bundled 3rdparty static libs (zlib, etc.) — leaf deps, so linked last.
OPENCV_3RDPARTY = $(wildcard $(OPENCV_ROOT)/lib/opencv4/3rdparty/*.a)

OPENCV_LIBS ?= -L$(OPENCV_ROOT)/lib $(OPENCV_MODULE_LIBS) $(OPENCV_3RDPARTY) \
               -ldl -lpthread -lm -lrt
# -------------------------------------------------------------------------

# ---- OpenSSL (CV2x/aarch64; built by openssl_cross/build_openssl.sh) -----
# Static target archives and generated target headers.  Keeping this separate
# from the SDK's runtime-only libssl/libcrypto makes the TLS client build
# reproducible in the OpenSDK container.
OPENSSL_ROOT ?= /mnt/third_party/openssl-aarch64
OPENSSL_CFLAGS ?= -I$(OPENSSL_ROOT)/include
OPENSSL_LIBS ?= -L$(OPENSSL_ROOT)/lib -lssl -lcrypto
# -------------------------------------------------------------------------

PATH5 = $(OPENSDK_ROOT)/opensdk-$(VERSION)/armv7-cortex_A7-linux-gnueabihf/bin
CXX5 = $(PATH5)/armv7-cortex_A7-linux-gnueabihf-g++
TOOLS_PATH5 = $(PATH5)/../armv7-cortex_A7-linux-gnueabihf/sysroot
INC5 = -I$(TOOLS_PATH5)/usr/include/
OBJS5 = $(patsubst $(SRC)%.cpp, $(SRC)%_wn5.o, $(SOURCES))
APP_HELPER_PATH5 = $(TOOLS_PATH5)/usr/lib
TARGET5 = bin/cctv_app_wn5

PATH6 = $(OPENSDK_ROOT)/opensdk-$(VERSION)/linaro-armv7ahf-2017.08-gcc7.1/bin
CXX6 = $(PATH6)/arm-linux-gnueabihf-g++
TOOLS_PATH6 = $(PATH6)/../arm-linux-gnueabihf/libc
INC6 = -I$(TOOLS_PATH6)/usr/include/
OBJS6 = $(patsubst $(SRC)%.cpp, $(SRC)%_s3l.o, $(SOURCES))
APP_HELPER_PATH6 = $(TOOLS_PATH6)/usr/lib
TARGET6 = bin/cctv_app_s3l

PATH7 = $(OPENSDK_ROOT)/opensdk-$(VERSION)/linaro-aarch64-2018.08-gcc8.2/bin
CXX7 = $(PATH7)/aarch64-linux-gnu-g++
TOOLS_PATH7 = $(PATH7)/../aarch64-linux-gnu/libc
INC7 = -I$(TOOLS_PATH7)/usr/include/
OBJS7 = $(patsubst $(SRC)%.cpp, $(SRC)%_cv2.o, $(SOURCES))
APP_HELPER_PATH7 = $(TOOLS_PATH7)/lib_cv2
LD_FLAGS7 = -Wl,-E,-rpath-link=$(APP_HELPER_PATH7)
TARGET7 = bin/cctv_app_cv2

PATH8 = $(OPENSDK_ROOT)/opensdk-$(VERSION)/armv7-wn7-linux-gnueabihf/bin
CXX8 = $(PATH8)/armv7-wn7-linux-gnueabihf-g++
TOOLS_PATH8 = $(PATH8)/../armv7-wn7-linux-gnueabihf/sysroot
INC8 = -I$(TOOLS_PATH8)/usr/include/
OBJS8 = $(patsubst $(SRC)%.cpp, $(SRC)%_wn7.o, $(SOURCES))
APP_HELPER_PATH8 = $(TOOLS_PATH8)/usr/lib
TARGET8 = bin/cctv_app_wn7

PATH9 = $(OPENSDK_ROOT)/opensdk-$(VERSION)/linaro-aarch64-2020.09-gcc10.2-linux5.4/bin
CXX9 = $(PATH9)/aarch64-linux-gnu-g++
TOOLS_PATH9 = $(PATH9)/../aarch64-linux-gnu/libc
INC9 = -I$(TOOLS_PATH9)/usr/include/
OBJS9 = $(patsubst $(SRC)%.cpp, $(SRC)%_cv2x.o, $(SOURCES))
APP_HELPER_PATH9 = $(TOOLS_PATH9)/lib_cv2x
LD_FLAGS9 = -Wl,-E,-rpath-link=$(APP_HELPER_PATH9)
TARGET9 = bin/cctv_app_cv2x

PATH10 = $(OPENSDK_ROOT)opensdk-$(VERSION)/aarch64-poky-linux/sysroots/x86_64-oesdk-linux/usr/bin/aarch64-poky-linux
CXX10 = $(PATH10)/aarch64-poky-linux-clang++
TOOLS_PATH10 =  $(OPENSDK_ROOT)opensdk-$(VERSION)/aarch64-poky-linux/sysroots/aarch64-poky-linux/lib
INC10 = -I$(OPENSDK_ROOT)opensdk-$(VERSION)/aarch64-poky-linux/sysroots/aarch64-poky-linux/usr/include
OBJS10 = $(patsubst $(SRC)%.cpp, $(SRC)%_mt8137.o, $(SOURCES))
APP_HELPER_PATH10 = $(TOOLS_PATH10)/lib_mt8137
LD_FLAGS10 = -Wl,-E,-rpath-link=$(APP_HELPER_PATH10)
TARGET10 = bin/cctv_app_mt8137

all: $(TARGET9) 

$(TARGET5): clean_bin $(OBJS5)
	$(CXX5) $(OPENSDK_ROOT)/opensdk-$(VERSION)/templates/libs/main_wn5.o $(OBJS5) -o $@ -L$(APP_HELPER_PATH5) $(LIBS5)
#strip5_begin
	$(PATH5)/armv7-cortex_A7-linux-gnueabihf-strip --strip-all $@
#strip5_end

src/%_wn5.o: src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	$(CXX5) $(CXXFLAGS5) -c -o '$@' '$<' $(INC5) $(INC) $(INC_APP)
	@echo 'Finished building: $<'
	@echo ' '

$(TARGET6): clean_bin $(OBJS6)
	$(CXX6) $(OPENSDK_ROOT)/opensdk-$(VERSION)/templates/libs/main_s3l.o $(OBJS6) -o $@ -L$(APP_HELPER_PATH6) $(LIBS6)
#strip6_begin
	$(PATH6)/arm-linux-gnueabihf-strip --strip-all $@
#strip6_end

src/%_s3l.o: src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	$(CXX6) $(CXXFLAGS6) -c -o '$@' '$<' $(INC6) $(INC) $(INC_APP)
	@echo 'Finished building: $<'
	@echo ' '

$(TARGET7): clean_bin $(OBJS7)
	$(CXX7) $(OPENSDK_ROOT)/opensdk-$(VERSION)/templates/libs/main_cv2.o $(OBJS7) -o $@ -L$(APP_HELPER_PATH7) $(LIBS7) $(LD_FLAGS7)
#strip7_begin
	$(PATH7)/aarch64-linux-gnu-strip --strip-all $@
#strip7_end

src/%_cv2.o: src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	$(CXX7) $(CXXFLAGS7) -c -o '$@' '$<' $(INC7) $(INC) $(INC_APP)
	@echo 'Finished building: $<'
	@echo ' '

$(TARGET8): clean_bin $(OBJS8)
	$(CXX8) $(OPENSDK_ROOT)/opensdk-$(VERSION)/templates/libs/main_wn7.o $(OBJS8) -o $@ -L$(APP_HELPER_PATH8) $(LIBS8)
#strip8_begin
	$(PATH8)/armv7-wn7-linux-gnueabihf-strip --strip-all $@
#strip8_end

src/%_wn7.o: src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	$(CXX8) $(CXXFLAGS8) -c -o '$@' '$<' $(INC8) $(INC) $(INC_APP)
	@echo 'Finished building: $<'
	@echo ' '

$(TARGET9): clean_bin $(OBJS9)
	$(CXX9) $(OPENSDK_ROOT)/opensdk-$(VERSION)/templates/libs/main_cv2x.o $(OBJS9) -o $@ -L$(APP_HELPER_PATH9) $(LIBS9) $(OPENSSL_LIBS) $(OPENCV_LIBS) $(LD_FLAGS9)
#strip9_begin
	$(PATH9)/aarch64-linux-gnu-strip --strip-all $@
#strip9_end

src/%_cv2x.o: src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	$(CXX9) $(CXXFLAGS9) -Wno-unused-but-set-variable -c -o '$@' '$<' $(INC9) $(INC) $(INC_APP) $(OPENCV_CFLAGS) $(OPENSSL_CFLAGS)
	@echo 'Finished building: $<'
	@echo ' '

$(TARGET10): clean_bin $(OBJS10)
	$(CXX10) $(OPENSDK_ROOT)opensdk-$(VERSION)/templates/libs/main_mt8137.o $(OBJS10) -o $@ --sysroot=/opt/opensdk/opensdk-5.00/aarch64-poky-linux/sysroots/aarch64-poky-linux -L$(APP_HELPER_PATH10) $(LIBS10) $(LD_FLAGS10)
#strip10_begin
	$(PATH10)/aarch64-poky-linux-strip --strip-all $@
#strip10_end

src/%_mt8137.o: src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	$(CXX10) -O2 -Wall -c -fPIC -fmessage-length=0 -o '$@' '$<' $(INC10) $(INC) $(INC_APP)
	@echo 'Finished building: $<'
	@echo ' '

clean_bin:
	-@rm -f bin/* 2>/dev/null
	@mkdir -p bin

clean:
	rm -f bin/* $(OBJS9)  *.cap
