#include "PathFollower.h"
#include <iostream>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

PathFollower::PathFollower()
    : current_waypoint_idx(0),
      wheel_diameter_m(0.065f), // 65mm wheels (example)
      wheelbase_m(0.160f),      // 160mm wheel track (example)
      gear_ratio(1.0f),         // Direct drive (example)
      steps_per_rev(400)        // 0.9 deg steps (example)
{
}

PathFollower::~PathFollower() {}

void PathFollower::SetPath(const std::vector<Waypoint_t>& new_path) {
    path = new_path;
    current_waypoint_idx = 0;
    std::cout << "[PathFollower] Path loaded with " << path.size() << " waypoints." << std::endl;
}

void PathFollower::Update(const Pose_t& current_pose, Msg_SetSpeed_t& out_speed, uint8_t& out_nozzle_on) {
    if (path.empty() || current_waypoint_idx >= path.size()) {
        // No path active, send stop command
        out_speed.left_sps = 0;
        out_speed.right_sps = 0;
        out_nozzle_on = 0;
        return;
    }

    // TODO: Calculate tracking errors
    // float cross_track_err = ...
    // float heading_err = ...
    
    // TODO: Generate v and w control inputs via PID / Pure Pursuit
    float target_v = 0.05f; // 5cm/s speed (example)
    float target_w = 0.0f;

    // Kinematic translation to steps-per-second
    out_speed = velocity_to_sps(target_v, target_w);
    out_nozzle_on = path[current_waypoint_idx].nozzle_on;
}

Msg_SetSpeed_t PathFollower::velocity_to_sps(float v, float w) {
    Msg_SetSpeed_t speed_cmd;
    
    // Differential drive kinematics
    float v_left = v - (w * wheelbase_m) / 2.0f;
    float v_right = v + (w * wheelbase_m) / 2.0f;

    // Wheel circumference
    float wheel_circ = M_PI * wheel_diameter_m;

    // Convert linear speed to steps-per-second (sps)
    float left_rps = v_left / wheel_circ;
    float right_rps = v_right / wheel_circ;

    speed_cmd.left_sps = static_cast<int16_t>(left_rps * steps_per_rev * gear_ratio);
    speed_cmd.right_sps = static_cast<int16_t>(right_rps * steps_per_rev * gear_ratio);

    return speed_cmd;
}
