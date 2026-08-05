#ifndef __ROBOT_TYPES_H__
#define __ROBOT_TYPES_H__

#include <stdint.h>
#include <string>

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

#endif /* __ROBOT_TYPES_H__ */
