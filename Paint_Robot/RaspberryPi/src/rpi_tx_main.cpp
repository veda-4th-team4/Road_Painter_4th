#include <iostream>
#include <cstring>
#include <cstdio>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringSerial.h>

#ifndef TG_ALT0
#define TG_ALT0 4  // 라즈베리파이 레지스터 기준 ALT0 모드는 물리값 4 (0b100) 입니다.
#endif

#pragma pack(push, 1)
typedef struct {
    int16_t left_sps;
    int16_t right_sps;
} Msg_SetSpeed_t;
#pragma pack(pop)

uint8_t calculate_crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x07;
            else crc <<= 1;
        }
    }
    return crc;
}

int main() {
    int fd;
    const char* device = "/dev/ttyAMA0";

    // 1. wiringPi 로우 레벨 환경 초기화
    if (wiringPiSetup() == -1) {
        std::cerr << "wiringPi 초기화 실패" << std::endl;
        return 1;
    }

    // GPIO 14 (wPi 15 / Physical 8) -> UART TX
    // GPIO 15 (wPi 16 / Physical 10) -> UART RX
    pinModeAlt(14, TG_ALT0); 
    pinModeAlt(15, TG_ALT0);

    // 2. 115200 Baudrate로 시리얼 포트 개방
    if ((fd = serialOpen(device, 115200)) < 0) {
        std::cerr << "포트 오픈 실패: " << device << std::endl;
        return 1;
    }

    std::cout << "라즈베리파이 패킷 송신 엔진 가동 (핀 모드 ALT0 변환 완료)" << std::endl;

    Msg_SetSpeed_t speed_cmd;
    speed_cmd.left_sps = 400;
    speed_cmd.right_sps = -400;

    uint8_t tx_packet[64];
    tx_packet[0] = 0xAA;                  
    tx_packet[1] = sizeof(Msg_SetSpeed_t); 
    tx_packet[2] = 0x01;                  
    std::memcpy(&tx_packet[3], &speed_cmd, sizeof(Msg_SetSpeed_t));

    uint8_t crc_target_len = tx_packet[1] + 2;
    uint8_t crc_val = calculate_crc8(&tx_packet[1], crc_target_len);
    tx_packet[3 + sizeof(Msg_SetSpeed_t)] = crc_val; 
    tx_packet[4 + sizeof(Msg_SetSpeed_t)] = 0x55;    

    int total_frame_size = sizeof(Msg_SetSpeed_t) + 5; 

    while (true) {
        for (int i = 0; i < total_frame_size; i++) {
            serialPutchar(fd, tx_packet[i]);
        }
        std::cout << "[RPi -> STM32] 패킷 송신 완료" << std::endl;
        sleep(1);
    }

    serialClose(fd);
    return 0;
}