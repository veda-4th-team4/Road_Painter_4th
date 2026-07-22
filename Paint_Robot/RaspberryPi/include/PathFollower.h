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
     * @brief Computes cross-track/heading errors and outputs left/right motor target speed (sps).
     * @param current_pose Current absolute coordinate of the robot.
     * @param out_speed Output struct to store speed commands.
     * @param out_nozzle_on Output command indicating if the paint spray nozzle should trigger.
     */
    void Update(const Pose_t& current_pose, Msg_SetSpeed_t& out_speed, uint8_t& out_nozzle_on);

private:
    std::vector<Segment_t> path;
    size_t current_waypoint_idx;

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
