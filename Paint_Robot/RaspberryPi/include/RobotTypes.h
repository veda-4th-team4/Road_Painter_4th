#pragma once
#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <stdint.h>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <ctime>

inline std::string GetTimestampStr() {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    auto timer = std::chrono::system_clock::to_time_t(now);
    std::tm bt{};
#if defined(_WIN32)
    localtime_s(&bt, &timer);
#else
    localtime_r(&timer, &bt);
#endif
    std::ostringstream oss;
    oss << "[" << std::put_time(&bt, "%H:%M:%S") << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
    return oss.str();
}

/**
 * @brief Common state indicators for packet parser state machine.
 */
typedef enum {
    STATE_STX,
    STATE_LEN,
    STATE_CMD,
    STATE_PAYLOAD,
    STATE_CRC,
    STATE_ETX
} ParserState_t;

/** @brief RPi에서 설정하여 부팅 시 STM32로 동적 전송할 서보모터 PWM 펄스 폭 [us] */
#define RPI_SERVO_OFF_US 1600U  /* 노즐 OFF (UP) 위치 */
#define RPI_SERVO_ON_US  1200U  /* 노즐 ON (DOWN) 위치 (200us 줄인 각도) */

#define UART_CMD_SET_SERVO_CONFIG 0x07

#pragma pack(push, 1)

/**
 * @brief CMD 0x01: Set Speed payload structure (4 bytes).
 */
typedef struct {
    int16_t left_sps;
    int16_t right_sps;
} Msg_SetSpeed_t;

/**
 * @brief CMD 0x02: Control Nozzle payload structure (1 byte).
 */
typedef struct {
    uint8_t nozzle_on;
} Msg_ControlNozzle_t;

/**
 * @brief CMD 0x03: Emergency Stop payload structure (1 byte).
 */
typedef struct {
    uint8_t fault_reason;
} Msg_EStop_t;

/**
 * @brief CMD 0x07: Dynamic Servo Config payload structure (4 bytes).
 */
typedef struct {
    uint16_t off_us;
    uint16_t on_us;
} Msg_SetServoConfig_t;

/**
 * @brief CMD 0x81: Status telemetry payload structure (9 bytes).
 */
typedef struct {
    uint32_t left_steps;
    uint32_t right_steps;
    uint8_t flags;
} Msg_Status_t;

#pragma pack(pop)

/**
 * @brief Pose coordinates computed by vision server.
 */
typedef struct {
    float x;
    float y;
    float theta;
    uint32_t timestamp_ms;
    uint8_t confidence;
} Pose_t;

/**
 * @brief Waypoint structure containing position and nozzle command.
 */
typedef struct {
    float x;
    float y;
    uint8_t nozzle_on;
    float speed;
} Waypoint_t;

/**
 * @brief Segment (Operation) structure containing path operation sequence (Protocol v2).
 */
typedef struct {
    uint32_t op_index{0};  // Global unique operation index (0-based)
    std::string op;        // "move", "turn", "nozzle", or "arc" (lowercase)
    std::string role;      // "path" or "offset" (metadata)
    float dist_m{0.0f};    // for move (positive = forward, negative = reverse)
    float angle_deg{0.0f}; // for turn (+ = CW right) or arc (positive size)
    float radius_m{0.0f};  // for arc (robot center radius R_c)
    std::string direction; // for arc ("left" or "right")
    bool down{false};      // for nozzle (true = lower/paint ON, false = raise/paint OFF)
} Segment_t;

/**
 * @brief Server feedback command structures carrying op_index (Protocol v2).
 */
typedef struct {
    uint32_t op_index;
    float angle_deg;
} AlignCmd_t;

typedef struct {
    uint32_t op_index;
    float dist_m;
} MoreCmd_t;

typedef struct {
    uint32_t op_index;
    float angle_deg;
} DriftCmd_t;

typedef struct {
    bool hold;
    std::string reason;
} HoldCmd_t;

#endif /* ROBOT_TYPES_H */
