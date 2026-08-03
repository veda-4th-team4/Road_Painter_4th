#ifndef __PATH_FOLLOWER_H__
#define __PATH_FOLLOWER_H__

#include <vector>
#include "RobotTypes.h"

/**
 * @brief Computes guidance error calculations and translates them into motor speed pulses.
 */
class PathFollower {
public:
    PathFollower();
    ~PathFollower();

    /**
     * @brief Loads the target path segments.
     */
    void SetPath(const std::vector<Segment_t>& new_path);

    /**
     * @brief Discards the loaded path entirely (server CMD "ABORT_DRAW").
     *
     * Unlike SetPath() this does not swap in a new path -- it leaves the
     * follower with nothing to drive. ESTOP alone is a PAUSE: it stops the
     * motors but the segment cursor survives, so RESUME picks the paint job up
     * exactly where it stopped. ABORT_DRAW must not do that, hence this.
     *
     * NOTE: an empty path makes IsPathFinished() true, so the caller must also
     * suppress the PATH_DONE it would normally send on completion -- the path
     * was thrown away, not finished (protocol v0.4).
     */
    void ClearPath();

    /**
     * @brief Gets current active segment index (0-based).
     */
    size_t GetCurrentSegmentIndex() const { return current_waypoint_idx; }

    /**
     * @brief Gets current active segment info.
     */
    bool GetCurrentSegment(Segment_t& out_seg) const;

    /**
     * @brief Advances to the next path segment.
     */
    void AdvanceSegment();

    /**
     * @brief Checks if all path segments are completed.
     */
    bool IsPathFinished() const;

    /**
     * @brief Sets DRIFT angular correction feedback from server.
     */
    void SetDriftOffset(float offset_deg);

    /**
     * @brief Calculates required motor step count for turn angle (supports 0.1 deg resolution).
     */
    uint32_t CalculateTurnSteps(float angle_deg) const;

    /**
     * @brief Starts turn segment tracking.
     */
    void StartTurn(float angle_deg, int32_t start_left_steps, int32_t start_right_steps);

    /**
     * @brief Updates turn execution based on latest step counts.
     * @return true if target turn angle is reached.
     */
    bool UpdateTurn(int32_t cur_left_steps, int32_t cur_right_steps, Msg_SetSpeed_t& out_speed);

    /**
     * @brief Checks if a turn is currently in progress.
     */
    bool IsTurning() const { return is_turning; }

    /**
     * @brief Calculates required motor step count for linear distance (in meters).
     */
    uint32_t CalculateMoveSteps(float dist_m) const;

    /**
     * @brief Starts straight move segment tracking.
     */
    void StartMove(float dist_m, int32_t start_left_steps, int32_t start_right_steps);

    /**
     * @brief Updates straight move execution based on latest step counts and IMU yaw heading.
     * @return true if target distance is reached.
     */
    bool UpdateMove(int32_t cur_left_steps, int32_t cur_right_steps, Msg_SetSpeed_t& out_speed, uint8_t& out_nozzle_on, float imu_yaw_deg = 0.0f);

    /**
     * @brief Physical offset distance from robot wheel center to rear spray nozzle (-155mm).
     */
    static constexpr float NOZZLE_OFFSET_M = -0.155f;

    /**
     * @brief Sub-sequence states for nozzle offset compensation.
     */
    enum class OffsetSeqState {
        IDLE,
        APPROACH_START,   // Position rear nozzle at line start
        MAIN_DRAW,        // Draw main line segment
        CORNER_REVERSE,   // Reverse -155mm to bring wheel center to vertex
        IN_PLACE_TURN,    // In-place turn at vertex
        CORNER_ADVANCE    // Advance +155mm to bring rear nozzle to next line start
    };

    /**
     * @brief Calculates motor steps for the 155mm nozzle offset (2,392 pulses).
     */
    uint32_t GetNozzleOffsetSteps() const { return CalculateMoveSteps(std::abs(NOZZLE_OFFSET_M)); }

    /**
     * @brief Starts an offset distance move (+155mm forward or -155mm backward).
     */
    void StartOffsetMove(float dist_m, int32_t start_left_steps, int32_t start_right_steps);

    /**
     * @brief Updates offset move step execution.
     * @return true when offset distance is reached.
     */
    bool UpdateOffsetMove(int32_t cur_left_steps, int32_t cur_right_steps, Msg_SetSpeed_t& out_speed, uint8_t& out_nozzle_on);

    /**
     * @brief Checks if a straight move is currently in progress.
     */
    bool IsMovingStraight() const { return is_moving_straight; }

    /**
     * @brief Computes guidance error calculations and outputs left/right motor target speed (sps).
     * @param current_pose Current absolute coordinate of the robot.
     * @param out_speed Output struct to store speed commands.
     * @param out_nozzle_on Output command indicating if the paint spray nozzle should trigger.
     */
    void Update(const Pose_t& current_pose, Msg_SetSpeed_t& out_speed, uint8_t& out_nozzle_on);

private:
    std::vector<Segment_t> path;
    size_t current_waypoint_idx;
    float drift_offset_deg;

    // Turn tracking state
    bool is_turning;
    float turn_target_angle_deg;
    uint32_t turn_target_steps;
    int32_t turn_start_left_steps;
    int32_t turn_start_right_steps;

    // Straight move tracking state
    bool is_moving_straight;
    float offset_move_dist;
    uint32_t move_target_steps;
    int32_t move_start_left_steps;
    int32_t move_start_right_steps;

    // Robot physical constants
    float wheel_diameter_m;
    float wheelbase_m;
    float gear_ratio;
    uint16_t steps_per_rev;

    /**
     * @brief Translates target linear velocity (v) and angular velocity (w) into steps-per-second.
     */
    Msg_SetSpeed_t velocity_to_sps(float v, float w);
};

#endif /* __PATH_FOLLOWER_H__ */
