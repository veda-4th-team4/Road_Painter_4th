#include "SerialManager.h"
#include <wiringPi.h>
#include <wiringSerial.h>
#include <iostream>
#include <cstring>

#ifndef TG_ALT0
#define TG_ALT0 4
#endif

SerialManager::SerialManager(const std::string& dev, uint32_t baud) 
    : fd(-1), device_path(dev), baudrate(baud) {}

SerialManager::~SerialManager() {
    Close();
}

bool SerialManager::Init() {
    // 1. wiringPi 로우 레벨 환경 초기화
    if (wiringPiSetup() == -1) {
        std::cerr << "[SerialManager] 오류: wiringPi 초기화 실패" << std::endl;
        return false;
    }

    // 2. 외부 확장 핀 모드를 ALT0(시리얼 전용)로 강제 변환
    pinModeAlt(14, TG_ALT0); 
    pinModeAlt(15, TG_ALT0);

    // 3. 포트 개방
    if ((fd = serialOpen(device_path.c_str(), baudrate)) < 0) {
        std::cerr << "[SerialManager] 오류: 포트 개방 실패 (" << device_path << ")" << std::endl;
        return false;
    }

    std::cout << "[SerialManager] 인프라 바인딩 성공: " << device_path << " (" << baudrate << " bps)" << std::endl;
    return true;
}

void SerialManager::Close() {
    if (fd >= 0) {
        serialClose(fd);
        fd = -1;
    }
}

uint8_t SerialManager::calculate_crc8(const uint8_t *data, uint8_t len) {
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

// 명세서 규격 공통 패킷 조립 및 유선 송신 엔진
bool SerialManager::send_packet(uint8_t cmd, const uint8_t *payload, uint8_t payload_len) {
    if (fd < 0) return false;

    uint8_t tx_buf[64];
    tx_buf[0] = 0xAA;        // STX
    tx_buf[1] = payload_len; // LEN
    tx_buf[2] = cmd;         // CMD

    if (payload != nullptr && payload_len > 0) {
        std::memcpy(&tx_buf[3], payload, payload_len);
    }

    // CRC8 연산 및 패킹 (LEN 위치부터 PAYLOAD 끝단까지 = payload_len + 2바이트 크기)
    uint8_t crc_val = calculate_crc8(&tx_buf[1], payload_len + 2);
    tx_buf[3 + payload_len] = crc_val;
    tx_buf[4 + payload_len] = 0x55; // ETX

    int total_frame_size = payload_len + 5;

    // 바이트 직렬 스트림 송신
    for (int i = 0; i < total_frame_size; i++) {
        serialPutchar(fd, tx_buf[i]);
    }
    return true;
}

bool SerialManager::SendSetSpeed(int16_t left, int16_t right) {
    Msg_SetSpeed_t payload;
    payload.left_sps = left;
    payload.right_sps = right;
    return send_packet(0x01, reinterpret_cast<const uint8_t*>(&payload), sizeof(Msg_SetSpeed_t));
}

bool SerialManager::SendControlNozzle(uint8_t on) {
    Msg_ControlNozzle_t payload;
    payload.nozzle_on = on;
    return send_packet(0x02, reinterpret_cast<const uint8_t*>(&payload), sizeof(Msg_ControlNozzle_t));
}

bool SerialManager::SendEmergencyStop(uint8_t reason) {
    Msg_EStop_t payload;
    payload.fault_reason = reason;
    return send_packet(0x03, reinterpret_cast<const uint8_t*>(&payload), sizeof(Msg_EStop_t));
}