#include "SerialManager.h"
#include <wiringPi.h>
#include <wiringSerial.h>
#include <iostream>
#include <cstring>

#ifndef TG_ALT0
#define TG_ALT0 4
#endif

SerialManager::SerialManager(const std::string& dev, uint32_t baud) 
    : fd(-1),
      device_path(dev),
      baudrate(baud),
      rx_alive(false),
      status_received_once(false),
      rx_state(STATE_STX),
      rx_len(0),
      rx_cmd(0),
      rx_payload_idx(0),
      rx_calculated_crc(0),
      rx_received_crc(0)
{
    std::memset(&latest_status, 0, sizeof(Msg_Status_t));
}

SerialManager::~SerialManager() {
    Close();
}

bool SerialManager::Init() {
    // 1. Initialize wiringPi low-level GPIO
    if (wiringPiSetup() == -1) {
        std::cerr << "[SerialManager] Error: wiringPi initialization failed" << std::endl;
        return false;
    }

    // 2. Set GPIO 14/15 pins to ALT0 mode for hardware serial UART
    pinModeAlt(14, TG_ALT0); 
    pinModeAlt(15, TG_ALT0);

    // 3. Open serial port
    if ((fd = serialOpen(device_path.c_str(), baudrate)) < 0) {
        std::cerr << "[SerialManager] Error: Failed to open serial port (" << device_path << ")" << std::endl;
        return false;
    }

    // 4. Spin up background receive thread
    rx_alive = true;
    rx_thread = std::thread(&SerialManager::rx_loop, this);

    std::cout << "[SerialManager] Successfully bound to " << device_path << " (" << baudrate << " bps) and initialized RX pipeline." << std::endl;
    return true;
}

void SerialManager::Close() {
    rx_alive = false;
    if (rx_thread.joinable()) {
        rx_thread.join();
    }
    if (fd >= 0) {
        serialClose(fd);
        fd = -1;
    }
}

uint8_t SerialManager::calculate_crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc = crc8_update(crc, data[i]);
    }
    return crc;
}

uint8_t SerialManager::crc8_update(uint8_t crc, uint8_t data) {
    crc ^= data;
    for (uint8_t bit = 0; bit < 8; bit++) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0x07;
        } else {
            crc <<= 1;
        }
    }
    return crc;
}

bool SerialManager::send_packet(uint8_t cmd, const uint8_t *payload, uint8_t payload_len) {
    if (fd < 0) return false;

    uint8_t tx_buf[64];
    tx_buf[0] = 0xAA;        // STX
    tx_buf[1] = payload_len; // LEN
    tx_buf[2] = cmd;         // CMD

    if (payload != nullptr && payload_len > 0) {
        std::memcpy(&tx_buf[3], payload, payload_len);
    }

    // Calculate CRC8 over LEN, CMD, and PAYLOAD (total length = payload_len + 2)
    uint8_t crc_val = calculate_crc8(&tx_buf[1], payload_len + 2);
    tx_buf[3 + payload_len] = crc_val;
    tx_buf[4 + payload_len] = 0x55; // ETX

    int total_frame_size = payload_len + 5;

    // Transmit raw byte stream
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

bool SerialManager::GetLatestStatus(Msg_Status_t& out_status) {
    std::lock_guard<std::mutex> lock(status_mutex);
    if (!status_received_once) {
        return false;
    }
    out_status = latest_status;
    return true;
}

void SerialManager::rx_loop() {
    Msg_Status_t temp_status;
    while (rx_alive) {
        if (fd < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        int avail = serialDataAvail(fd);
        if (avail <= 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        for (int i = 0; i < avail; i++) {
            int ch = serialGetchar(fd);
            if (ch < 0) break;
            
            if (parse_byte(static_cast<uint8_t>(ch), temp_status)) {
                std::lock_guard<std::mutex> lock(status_mutex);
                latest_status = temp_status;
                status_received_once = true;
            }
        }
    }
}

bool SerialManager::parse_byte(uint8_t byte, Msg_Status_t& out_status) {
    switch (rx_state) {
    case STATE_STX:
        if (byte == 0xAA) {
            rx_state = STATE_LEN;
        }
        break;

    case STATE_LEN:
        if (byte > 16) { // Max payload threshold
            rx_state = STATE_STX;
            break;
        }
        rx_len = byte;
        rx_payload_idx = 0;
        rx_calculated_crc = crc8_update(0, byte);
        rx_state = STATE_CMD;
        break;

    case STATE_CMD:
        rx_cmd = byte;
        rx_calculated_crc = crc8_update(rx_calculated_crc, byte);
        if (rx_len == 0) {
            rx_state = STATE_CRC;
        } else {
            rx_state = STATE_PAYLOAD;
        }
        break;

    case STATE_PAYLOAD:
        rx_payload[rx_payload_idx++] = byte;
        rx_calculated_crc = crc8_update(rx_calculated_crc, byte);
        if (rx_payload_idx >= rx_len) {
            rx_state = STATE_CRC;
        }
        break;

    case STATE_CRC:
        rx_received_crc = byte;
        rx_state = STATE_ETX;
        break;

    case STATE_ETX:
        rx_state = STATE_STX;
        if (byte == 0x55 && rx_received_crc == rx_calculated_crc) {
            if (rx_cmd == 0x81 && rx_len == sizeof(Msg_Status_t)) {
                std::memcpy(&out_status, rx_payload, sizeof(Msg_Status_t));
                return true;
            }
        }
        break;

    default:
        rx_state = STATE_STX;
        break;
    }
    return false;
}