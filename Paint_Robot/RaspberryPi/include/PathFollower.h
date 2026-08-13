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
    void StartTurn(float angle_deg, int32_t start_left_steps, int32_t start_right_steps, float start_imu_yaw = 0.0f);

    /**
     * @brief Updates turn execution based on latest step counts and optional IMU closed-loop yaw feedback.
     * @return true if target turn angle is reached.
     */
    bool UpdateTurn(int32_t cur_left_steps, int32_t cur_right_steps, Msg_SetSpeed_t& out_speed, float cur_imu_yaw = 0.0f, bool has_imu = false);

    /**
     * @brief Performs active post-turn micro-correction (trimming residual overshoot/undershoot).
     * @return true when micro-correction is finished and yaw error is <= 0.15 deg.
     */
    bool TrimTurn(float target_yaw, float cur_imu_yaw, Msg_SetSpeed_t &out_speed);

    /**
     * @brief Checks if a turn is currently in progress.
     */
    bool IsTurning() const { return is_turning; }

    /**
     * @brief Calculates required motor step count for linear distance (in meters).
     */
    uint32_t CalculateMoveSteps(float dist_m) const;

    /**
     * @brief Starts straight move segment tracking with optional custom speed (default 0.05 m/s, e.g. 0.015 m/s for MORE micro-moves).
     */
    void StartMove(float dist_m, int32_t start_left_steps, int32_t start_right_steps, float target_v_m_s = 0.05f);

    /**
     * @brief Updates straight move execution based on latest step counts and IMU yaw heading.
     * @return true if target distance is reached.
     */
    bool UpdateMove(int32_t cur_left_steps, int32_t cur_right_steps, Msg_SetSpeed_t& out_speed, float imu_yaw_deg = 0.0f);

    /**
     * @brief Starts arc (curved) motion.
     */
    void StartArc(float radius_m, float angle_deg, const std::string& direction, int32_t start_l_steps, int32_t start_r_steps);

    /**
     * @brief Updates arc motion execution.
     * @return true if target arc distance is reached.
     */
    bool UpdateArc(int32_t cur_l_steps, int32_t cur_r_steps, Msg_SetSpeed_t& out_speed);

    /**
     * @brief Checks if a straight move is currently in progress.
     */
    bool IsMovingStraight() const { return is_moving_straight; }

    /**
     * @brief Checks if an arc motion is currently in progress.
     */
    bool IsArc() const { return is_arc; }

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
    float turn_start_imu_yaw;

    // Arc tracking state
    bool is_arc;
    bool arc_is_left;
    uint32_t arc_target_l_steps;
    uint32_t arc_target_r_steps;
    int32_t arc_start_l_steps;
    int32_t arc_start_r_steps;
    int16_t arc_sps_l;
    int16_t arc_sps_r;

    // Straight move tracking state
    bool is_moving_straight;
    float move_dist_m;
    float move_speed_m_s;
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
