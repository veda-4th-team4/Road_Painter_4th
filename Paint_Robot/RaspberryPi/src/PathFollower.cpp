#include "PathFollower.h"
#include <iostream>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

PathFollower::PathFollower()
    : current_waypoint_idx(0),
      drift_offset_deg(0.0f),
      is_turning(false),
      turn_target_angle_deg(0.0f),
      turn_target_steps(0),
      turn_start_left_steps(0),
      turn_start_right_steps(0),
      is_moving_straight(false),
      move_target_steps(0),
      move_start_left_steps(0),
      move_start_right_steps(0),
      wheel_diameter_m(0.066f), // 66mm wheels
      wheelbase_m(0.166f),      // 166mm wheel track
      gear_ratio(1.0f),         // Direct drive
      steps_per_rev(3200)       // 200 full steps * 16 microsteps = 3200
{
}

PathFollower::~PathFollower() {}

void PathFollower::SetPath(const std::vector<Segment_t>& new_path) {
    path = new_path;
    current_waypoint_idx = 0;
    drift_offset_deg = 0.0f;
    is_turning = false;
    is_moving_straight = false;
    std::cout << "[PathFollower] Path loaded with " << path.size() << " segments." << std::endl;
}

bool PathFollower::GetCurrentSegment(Segment_t& out_seg) const {
    if (path.empty() || current_waypoint_idx >= path.size()) return false;
    out_seg = path[current_waypoint_idx];
    return true;
}

void PathFollower::AdvanceSegment() {
    if (current_waypoint_idx < path.size()) {
        current_waypoint_idx++;
        drift_offset_deg = 0.0f; // Reset drift offset on segment change
        is_turning = false;
        is_moving_straight = false;
        std::cout << "[PathFollower] Advanced to segment index: " << current_waypoint_idx << std::endl;
    }
}

bool PathFollower::IsPathFinished() const {
    return path.empty() || current_waypoint_idx >= path.size();
}

void PathFollower::SetDriftOffset(float offset_deg) {
    drift_offset_deg = offset_deg;
}

uint32_t PathFollower::CalculateTurnSteps(float angle_deg) const {
    // 90 deg = 2032.24 pulses (with K_slip = 1.01)
    // 0.1 deg resolution: 2.258044 pulses per 0.1 deg (22.58044 pulses/deg)
    float abs_angle = std::fabs(angle_deg);
    float steps = abs_angle * 22.58044f;
    return static_cast<uint32_t>(std::round(steps));
}

void PathFollower::StartTurn(float angle_deg, int32_t start_left_steps, int32_t start_right_steps) {
    is_turning = true;
    turn_target_angle_deg = angle_deg;
    turn_target_steps = CalculateTurnSteps(angle_deg);
    turn_start_left_steps = start_left_steps;
    turn_start_right_steps = start_right_steps;
    std::cout << "[PathFollower] StartTurn: target angle=" << angle_deg 
              << " deg | target_steps=" << turn_target_steps << std::endl;
}

bool PathFollower::UpdateTurn(int32_t cur_left_steps, int32_t cur_right_steps, Msg_SetSpeed_t& out_speed) {
    if (!is_turning) return true;

    int32_t diff_left = cur_left_steps - turn_start_left_steps;
    int32_t diff_right = cur_right_steps - turn_start_right_steps;
    uint32_t delta_left = static_cast<uint32_t>(std::abs(diff_left));
    uint32_t delta_right = static_cast<uint32_t>(std::abs(diff_right));
    uint32_t progress_steps = (delta_left + delta_right) / 2;

    if (progress_steps >= turn_target_steps) {
        // Turn target reached
        out_speed.left_sps = 0;
        out_speed.right_sps = 0;
        is_turning = false;
        std::cout << "[PathFollower] Turn completed! Reached " << progress_steps 
                  << "/" << turn_target_steps << " steps." << std::endl;
        return true;
    }

    // Turn in progress (positive angle = left turn)
    int16_t turn_sps = 400; // 400 sps turn speed
    if (turn_target_angle_deg > 0.0f) {
        // Left turn: Left wheel backward (-sps), Right wheel forward (+sps)
        out_speed.left_sps = -turn_sps;
        out_speed.right_sps = turn_sps;
    } else {
        // Right turn: Left wheel forward (+sps), Right wheel backward (-sps)
        out_speed.left_sps = turn_sps;
        out_speed.right_sps = -turn_sps;
    }

    return false;
}

uint32_t PathFollower::CalculateMoveSteps(float dist_m) const {
    // 3200 steps/rev, 66mm wheel diameter -> C = pi * 0.066 = 0.20734515 m
    // 3200 / 0.20734515 = 15433.09 steps/meter (~15.433 steps/mm)
    float abs_dist = std::fabs(dist_m);
    float steps = abs_dist * 15433.09f;
    return static_cast<uint32_t>(std::round(steps));
}

void PathFollower::StartMove(float dist_m, int32_t start_left_steps, int32_t start_right_steps) {
    is_moving_straight = true;
    move_target_steps = CalculateMoveSteps(dist_m);
    move_start_left_steps = start_left_steps;
    move_start_right_steps = start_right_steps;
    std::cout << "[PathFollower] StartMove: target dist=" << dist_m 
              << " m | target_steps=" << move_target_steps << std::endl;
}

bool PathFollower::UpdateMove(int32_t cur_left_steps, int32_t cur_right_steps, Msg_SetSpeed_t& out_speed, uint8_t& out_nozzle_on, float imu_yaw_deg) {
    if (!is_moving_straight) return true;

    int32_t diff_left = cur_left_steps - move_start_left_steps;
    int32_t diff_right = cur_right_steps - move_start_right_steps;
    uint32_t delta_left = static_cast<uint32_t>(std::abs(diff_left));
    uint32_t delta_right = static_cast<uint32_t>(std::abs(diff_right));
    uint32_t progress_steps = (delta_left + delta_right) / 2;

    Segment_t current_seg;
    GetCurrentSegment(current_seg);

    if (progress_steps >= move_target_steps) {
        // Move target reached
        out_speed.left_sps = 0;
        out_speed.right_sps = 0;
        out_nozzle_on = 0;
        is_moving_straight = false;
        std::cout << "[PathFollower] Move completed! Reached " << progress_steps 
                  << "/" << move_target_steps << " steps." << std::endl;
        return true;
    }

    // Straight move in progress with server DRIFT + IMU Yaw fusion correction
    float target_v = 0.05f; // 5 cm/s
    float total_heading_error = drift_offset_deg + imu_yaw_deg;
    float target_w = -total_heading_error * 0.05f; 
    out_speed = velocity_to_sps(target_v, target_w);
    out_nozzle_on = current_seg.paint ? 1 : 0;

    static int move_log_count = 0;
    if (++move_log_count % 10 == 0) {
        std::cout << "[PathFollower MOVE] Progress: " << progress_steps << "/" << move_target_steps
                  << " steps | IMU Yaw: " << imu_yaw_deg << " deg | Server Drift: " << drift_offset_deg
                  << " deg -> Target SPS (L: " << out_speed.left_sps << ", R: " << out_speed.right_sps << ")" << std::endl;
    }

    return false;
}

void PathFollower::StartOffsetMove(float dist_m, int32_t start_left_steps, int32_t start_right_steps) {
    is_moving_straight = true;
    offset_move_dist = dist_m;
    move_target_steps = CalculateMoveSteps(std::abs(dist_m));
    move_start_left_steps = start_left_steps;
    move_start_right_steps = start_right_steps;
    std::cout << "[PathFollower Offset] StartOffsetMove: dist=" << dist_m 
              << " m | target_steps=" << move_target_steps << std::endl;
}

bool PathFollower::UpdateOffsetMove(int32_t cur_left_steps, int32_t cur_right_steps, Msg_SetSpeed_t& out_speed, uint8_t& out_nozzle_on) {
    if (!is_moving_straight) return true;

    int32_t diff_left = cur_left_steps - move_start_left_steps;
    int32_t diff_right = cur_right_steps - move_start_right_steps;
    uint32_t delta_left = static_cast<uint32_t>(std::abs(diff_left));
    uint32_t delta_right = static_cast<uint32_t>(std::abs(diff_right));
    uint32_t progress_steps = (delta_left + delta_right) / 2;

    if (progress_steps >= move_target_steps) {
        out_speed.left_sps = 0;
        out_speed.right_sps = 0;
        out_nozzle_on = 0;
        is_moving_straight = false;
        std::cout << "[PathFollower Offset] Offset move completed! Reached " << progress_steps 
                  << "/" << move_target_steps << " steps." << std::endl;
        return true;
    }

    // Determine direction (+0.05m/s forward or -0.05m/s backward)
    float target_v = (offset_move_dist >= 0.0f) ? 0.05f : -0.05f;
    out_speed = velocity_to_sps(target_v, 0.0f);
    out_nozzle_on = 0; // Spray OFF during offset positioning

    return false;
}

void PathFollower::StartArc(float radius_m, float angle_deg, const std::string& direction, int32_t start_l_steps, int32_t start_r_steps) {
    is_arc = true;
    bool is_left = (direction == "left");

    // Pythagorean corrected robot wheel center radius: R_robot = sqrt(R_paint^2 + 0.155^2)
    float r_robot = std::sqrt(radius_m * radius_m + NOZZLE_OFFSET_M * NOZZLE_OFFSET_M);
    float r_left = is_left ? (r_robot - wheelbase_m / 2.0f) : (r_robot + wheelbase_m / 2.0f);
    float r_right = is_left ? (r_robot + wheelbase_m / 2.0f) : (r_robot - wheelbase_m / 2.0f);

    float angle_rad = angle_deg * (M_PI / 180.0f);
    float pulses_per_m = (steps_per_rev * gear_ratio) / (M_PI * wheel_diameter_m);

    arc_target_l_steps = static_cast<uint32_t>(std::abs(r_left * angle_rad) * pulses_per_m);
    arc_target_r_steps = static_cast<uint32_t>(std::abs(r_right * angle_rad) * pulses_per_m);
    arc_start_l_steps = start_l_steps;
    arc_start_r_steps = start_r_steps;

    float base_sps = 771.65f; // 0.05 m/s base speed
    arc_sps_l = static_cast<int16_t>(base_sps * (r_left / r_robot));
    arc_sps_r = static_cast<int16_t>(base_sps * (r_right / r_robot));

    std::cout << "[PathFollower ARC] StartArc: R_paint=" << radius_m << "m -> R_robot=" << r_robot
              << "m | angle=" << angle_deg << " deg (" << direction << ") | SPS_L=" << arc_sps_l << ", SPS_R=" << arc_sps_r << std::endl;
}

bool PathFollower::UpdateArc(int32_t cur_l_steps, int32_t cur_r_steps, Msg_SetSpeed_t& out_speed) {
    if (!is_arc) return true;

    int32_t dl = std::abs(cur_l_steps - arc_start_l_steps);
    int32_t dr = std::abs(cur_r_steps - arc_start_r_steps);

    if (static_cast<uint32_t>(dl) >= arc_target_l_steps || static_cast<uint32_t>(dr) >= arc_target_r_steps) {
        out_speed.left_sps = 0;
        out_speed.right_sps = 0;
        is_arc = false;
        std::cout << "[PathFollower ARC] Arc motion completed!" << std::endl;
        return true;
    }

    out_speed.left_sps = arc_sps_l;
    out_speed.right_sps = arc_sps_r;
    return false;
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
        // Apply DRIFT angle correction: positive angle = clockwise drift -> turn left
        target_w = -drift_offset_deg * 0.05f; 
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
