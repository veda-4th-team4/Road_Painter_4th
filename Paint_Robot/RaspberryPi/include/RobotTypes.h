#ifndef __ROBOT_TYPES_H__
#define __ROBOT_TYPES_H__

#include <stdint.h>

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

#endif /* __ROBOT_TYPES_H__ */
