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

void PathFollower::SetPath(const std::vector<Segment_t>& new_path) {
    path = new_path;
    current_waypoint_idx = 0;
    std::cout << "[PathFollower] Path loaded with " << path.size() << " segments." << std::endl;
}

void PathFollower::Update(const Pose_t& current_pose, Msg_SetSpeed_t& out_speed, uint8_t& out_nozzle_on) {
    if (path.empty() || current_waypoint_idx >= path.size()) {
        // No path active, send stop command
        out_speed.left_sps = 0;
        out_speed.right_sps = 0;
        out_nozzle_on = 0;
        return;
    }

    // Process segment-based operations (MOVE and TURN)
    const auto& current_seg = path[current_waypoint_idx];
    float target_v = 0.0f;
    float target_w = 0.0f;

    if (current_seg.op == "MOVE") {
        target_v = 0.05f; // 5 cm/s straight speed
        target_w = 0.0f;
        out_nozzle_on = current_seg.paint ? 1 : 0;
    } else if (current_seg.op == "TURN") {
        target_v = 0.0f;
        // Positive angle_deg means left turn (positive angular velocity), negative is right turn
        target_w = (current_seg.angle_deg > 0.0f) ? 0.2f : -0.2f; 
        out_nozzle_on = 0;
    }

    // Kinematic translation to steps-per-second
    out_speed = velocity_to_sps(target_v, target_w);
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
