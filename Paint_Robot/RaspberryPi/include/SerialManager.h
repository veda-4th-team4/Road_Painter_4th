#ifndef __SERIAL_MANAGER_H__
#define __SERIAL_MANAGER_H__

#include <cstdint>
#include <string>

// 명세서 규격 1바이트 정렬 구조체 정의들
#pragma pack(push, 1)
typedef struct {
    int16_t left_sps;
    int16_t right_sps;
} Msg_SetSpeed_t; // CMD 0x01

typedef struct {
    uint8_t nozzle_on;
} Msg_ControlNozzle_t; // CMD 0x02

typedef struct {
    uint8_t fault_reason;
} Msg_EStop_t; // CMD 0x03
#pragma pack(pop)

class SerialManager {
private:
    int fd;
    std::string device_path;
    uint32_t baudrate;

    // 내부 유틸리티 함수
    uint8_t calculate_crc8(const uint8_t *data, uint8_t len);
    bool send_packet(uint8_t cmd, const uint8_t *payload, uint8_t payload_len);

public:
    SerialManager(const std::string& dev = "/dev/ttyAMA0", uint32_t baud = 115200);
    ~SerialManager();

    bool Init();
    void Close();

    // 상위 계층에 노출할 하드웨어 추상화 API 멤버 함수
    bool SendSetSpeed(int16_t left, int16_t right);
    bool SendControlNozzle(uint8_t on);
    bool SendEmergencyStop(uint8_t reason);
};

#endif // __SERIAL_MANAGER_H__